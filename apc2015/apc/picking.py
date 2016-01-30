from klampt import *
from klampt import vectorops,so3,se3,ik,loader
from klampt.robotsim import Geometry3D
import apc
import baxter
import planner
import time
import math
import random
import perception_fake as perception

#Turn this on to skip path planning (goes straight to milestones)
SKIP_PATH_PLANNING = 0


class PickingController:
    """Maintains the robot's knowledge base and internal state.  Most of
    your code will go here.  Members include:
    - knowledge: a KnowledgeBase object
    - planner: a planner.LimbPlanner object
    - state: either 'ready', or 'holding'
    - configuration: the robot's current configuration
    - active_limb: the limb currently active, either holding or viewing a state
    - current_bin: the name of the bin where the camera is viewing or the gripper is located
    - held_object: the held object, if one is held, or None otherwise

    External modules can call viewBinAction(), graspAction(), ungraspAction(),
    and placeInOrderBinAction()
    """
    def __init__(self,world,robotController):
        self.world = world
        self.robot = world.robot(0)
        self.controller = robotController
        self.knowledge = apc.KnowledgeBase()
        self.planner = planner.LimbPlanner(self.world,self.knowledge)
        self.state = 'ready'
        self.active_limb = None
        self.active_grasp = None
        self.current_bin = None
        self.held_object = None
        #these may be helpful
        self.left_camera_link = self.robot.getLink(baxter.left_camera_link_name)
        self.right_camera_link = self.robot.getLink(baxter.right_camera_link_name)
        self.left_gripper_link = self.robot.getLink(baxter.left_gripper_link_name)
        self.right_gripper_link = self.robot.getLink(baxter.right_gripper_link_name)
        self.left_arm_links = [self.robot.getLink(i) for i in baxter.left_arm_link_names]
        self.right_arm_links = [self.robot.getLink(i) for i in baxter.right_arm_link_names]
        self.left_arm_indices = [i.index for i in self.left_arm_links]
        self.right_arm_indices = [i.index for i in self.right_arm_links]

    def waitForMove(self,timeout = None, pollRate = 0.5):
        """Waits for the move to complete, or timeout seconds is elapsed,
        before terminating."""
        iters = 0
        t = 0
        while self.controller.isMoving():
            if iters % 10 == 0:
                print "Waiting for move to complete..."
            time.sleep(pollRate)
            t += pollRate
            if timeout != None and t > timeout:
                return False
            iters += 1
        return True

    def viewBinAction(self,b):
        self.waitForMove()
            
        if self.state != 'ready':
            print "Already holding an object, can't move to bin"
            return False
        else:
            if b in apc.bin_names:
                if self.move_camera_to_bin(b):
                    self.waitForMove()
                    self.current_bin = b
                    perception.run_perception_on_bin(self.knowledge,b)
                    print "Sensed bin",b,"with camera",self.active_limb
                    return True
                else:
                    print "Move to bin",b,"failed"
                    return False
            else:
                print "Invalid bin",b
                return False
        return True
        
    def graspAction(self):
        self.waitForMove()
            
        if self.current_bin == None:
            print "Not located at a bin"
            return False
        elif self.state != 'ready':
            print "Already holding an object, can't grasp another"
            return False
        elif len(self.knowledge.bin_contents[self.current_bin])==0:
            print "The current bin is empty"
            return False
        else:
            if self.move_to_grasp_object(self.knowledge.bin_contents[self.current_bin][0]):
                self.waitForMove()
                #now close the gripper
                self.controller.commandGripper(self.active_limb,self.active_grasp.gripper_close_command)
                self.waitForMove()
                
                self.held_object = self.knowledge.bin_contents[self.current_bin].pop(0)
                self.state = 'holding'
                print "Holding object",self.held_object.info.name,"in hand",self.active_limb
                return True
            else:
                print "Grasp failed"
                return False

    def ungraspAction(self):
        self.waitForMove()
            
        if self.state != 'holding':
            print "Not holding an object"
            return False
        else:
            if self.move_to_ungrasp_object(self.held_object):
                self.waitForMove()
                #now open the gripper
                self.controller.commandGripper(self.active_limb,self.active_grasp.gripper_open_command)
                self.waitForMove()
                
                print "Object",self.held_object.info.name,"placed back in bin"
                self.knowledge.bin_contents[self.current_bin].append(self.held_object)
                self.state = 'ready'
                self.held_object = None
                return True
            else:
                print "Ungrasp failed"
                return False
        
    def placeInOrderBinAction(self):
        self.waitForMove()

        if self.state != 'holding':
            print "Not holding an object"
        else:
            if self.move_to_order_bin(self.held_object):
                self.waitForMove()                
                #now open the gripper
                self.controller.commandGripper(self.active_limb,self.active_grasp.gripper_open_command)
                self.waitForMove()
                print "Successfully placed",self.held_object.info.name,"into order bin"
                self.knowledge.order_bin_contents.append(self.held_object)
                self.active_limb = self.active_grasp = None
                self.held_object.xform = None
                self.held_object.bin_name = 'order_bin'
                self.state = 'ready'
                self.held_object = None
                return True
            else:
                print "Move to order bin failed"
                return False

    def fulfillOrderAction(self,objectList):
        """Given a list of objects to be put in the order bin, run
        until completed."""
        remainingObjects = objectList
        for b in apc.bin_names:
            if self.knowledge.bin_contents[b]==None:
                if not self.viewBinAction(b):
                    print "Could not view bin",b
                    continue

            donextbin = False
            while any(o.info.name in remainingObjects for o in self.knowledge.bin_contents[b]) and not donextbin:
                #pick up and put down objects until you are holding one that is in the remainingObjects list
                if not self.graspAction():
                    print "Error grasping object"
                    donextbin = True
                    break
                while not donextbin and (self.held_object == None or self.held_object.info.name not in remainingObjects):
                    #cycle through objects by putting down and picking up the next object
                    if not self.ungraspAction():
                        print "Error putting down object"
                        return False
                    if not self.graspAction():
                        print "Error grasping object"
                        donextbin = True
                        break
                obj = self.held_object
                if self.placeInOrderBinAction():
                    remainingObjects.remove(obj.info.name)
                else:
                    print "Error putting object into order bin"
                    return False
            if len(remainingObjects)==0:
                return True
        print "These items are remaining from the order:",remainingObjects
        return False

    def randomize_limb_position(self,limb,center=None, range=None):
        """Helper: randomizes the limb configuration in self.robot.
        limb can be 'left' or 'right'.  If range is provided, then
        this samples in a range around the config center.  If center is not
        provided, it uses the current commanded config"""
        qmin,qmax = self.robot.getJointLimits()
        q = baxter.rest_config[:]
        if range == None:
            if limb == 'left':
                for j in self.left_arm_indices:
                    q[j] = random.uniform(qmin[j],qmax[j])
            else:
                for j in self.right_arm_indices:
                    q[j] = random.uniform(qmin[j],qmax[j])
            self.robot.setConfig(q)
        else:
            if center==None:
                center = self.controller.getCommandedConfig()
            if limb == 'left':
                for j in self.left_arm_indices:
                    q[j] = random.uniform(max(qmin[j],center[j]-range),min(qmax[j],center[j]+range))
            else:
                for j in self.right_arm_indices:
                    q[j] = random.uniform(max(qmin[j],center[j]-range),min(qmax[j],center[j]+range))
            self.robot.setConfig(q)
        return

    def get_ik_solutions(self,goals,limbs,initialConfig=None,maxResults=10,maxIters=100,tol=1e-3,validity_checker=None):
        """Given a list of goals and their associated limbs, returns a list
        of (q,index) pairs, where q is an IK solution for goals[index].
        The results are sorted by distance from initialConfig.

        Arguments:
            - goals: a list of IKObjectives
            - limbs: a list of 'left'/'right', one for each goal, corresponding
              to the limb that the goal is defined on
            - initialConfig: optionally, a configuration for the IK seed
            - maxResults: the maximum number of IK results to return
            - maxIters: the maximum number of random samples to draw
            - tol: the ik solving tolerance
            - validity_checker: optionally, a special collision checker f(limb)
              that returns True if the robot's current limb configuration is valid.
              If None, the standard planner collision checker is used.

        Returns: a list [(solution1,index1),...,(solutionn,indexn)] of up to
        maxResults collision-free solutions.
        """
        if initialConfig == None:
            initialConfig = self.controller.getCommandedConfig()
        if validity_checker == None:
            validity_checker = self.planner.check_collision_free
        numTrials = [0]*len(goals)
        ikSolutions = []
        for i in range(maxIters):
            index = random.randint(0,len(goals)-1)
            goal = goals[index]
            limb = limbs[index]
            numTrials[index] += 1
            if numTrials[index] == 1:
                self.robot.setConfig(initialConfig)
            else:
                self.randomize_limb_position(limb,center=initialConfig,range=0.05*(numTrials[index]-1))
            if ik.solve(goal,tol=tol):
                if validity_checker(limb):
                    ikSolutions.append((self.robot.getConfig(),index))
                    if len(ikSolutions) >= maxResults: break
                else:
                    print "IK solution for goal",index,"was in collision, trying again"
        if len(ikSolutions)==0:
            print "No collision free IK solution"
            return []
        sortedSolutions = sorted([(vectorops.distanceSquared(solution[0],initialConfig),solution) for solution in ikSolutions])
        return [s[1] for s in sortedSolutions]


    def move_camera_to_bin(self,bin_name):
        """Starts a motion so the camera has a viewpoint that
        observes bin_name.  Will also change self.active_limb to the
        appropriate limb.

        If successful, sends the motion to the low-level controller and
        returns True.
        
        Otherwise, does not modify the low-level controller and returns False.
        """
        world_offset = self.knowledge.bin_vantage_point(bin_name)
        
        #place +z in the +x axis, y in the +z axis, and x in the -y axis
        left_goal = ik.objective(self.left_camera_link,R=[0,0,-1,1,0,0,0,1,0],t=world_offset)
        right_goal = ik.objective(self.right_camera_link,R=[0,0,-1,1,0,0,0,1,0],t=world_offset)
        qcmd = self.controller.getCommandedConfig()
        limbs = ['left','right']
        sortedSolutions = self.get_ik_solutions([left_goal,right_goal],limbs,qcmd)
        if len(sortedSolutions)==0: return False;
        #prototyping hack: move straight to target
        if SKIP_PATH_PLANNING:
            self.controller.setMilestone(sortedSolutions[0][0])
            self.active_limb = limbs[sortedSolutions[0][1]]
            return True
        for solution in sortedSolutions:
            path = self.planner.plan(qcmd,solution[0])
            if path != None:
                self.sendPath(path)
                self.active_limb = limbs[solution[1]]
                return True
        print "Failed to plan path"
        return False

    def get_path_to_bin(self, bin_name):
        world_offset = self.knowledge.bin_vantage_point(bin_name)

        #place +z in the +x axis, y in the +z axis, and x in the -y axis
        left_goal = ik.objective(self.left_camera_link,R=[0,0,-1,1,0,0,0,1,0],t=world_offset)
        right_goal = ik.objective(self.right_camera_link,R=[0,0,-1,1,0,0,0,1,0],t=world_offset)
        qcmd = self.controller.getCommandedConfig()
        limbs = ['left','right']
        sortedSolutions = self.get_ik_solutions([left_goal,right_goal],limbs,qcmd)
        if len(sortedSolutions)==0: return False;
        for solution in sortedSolutions:
            path = self.planner.plan(qcmd,solution[0])
            if path != None:
                return path, limbs[solution[1]]
        print "Failed to plan path"
        return None, None

    def move_to_grasp_object(self,object):
        """Sets the robot's configuration so the gripper grasps object at
        one of its potential grasp locations.  Might change self.active_limb
        to the appropriate limb.  Must change self.active_grasp to the
        selected grasp.

        If successful, sends the motion to the low-level controller and
        returns True.
        
        Otherwise, does not modify the low-level controller and returns False.
        """
        self.waitForMove()
        self.controller.commandGripper(self.active_limb,[1])
        grasps = self.knowledge.grasp_xforms(object)
        qmin,qmax = self.robot.getJointLimits()
        
        #get the end of the commandGripper motion
        qcmd = self.controller.getCommandedConfig()
        self.robot.setConfig(qcmd)
        set_model_gripper_command(self.robot,self.active_limb,[1])
        qcmd = self.robot.getConfig()
        
        #solve an ik solution to one of the grasps
        grasp_goals = []
        pregrasp_goals = []
        pregrasp_shift = [0,0,0.05]
        for (grasp,gxform) in grasps:
            if self.active_limb == 'left':
                Tg = se3.mul(gxform,se3.inv(baxter.left_gripper_center_xform))
                goal = ik.objective(self.left_gripper_link,R=Tg[0],t=Tg[1])
                Tpg = se3.mul(gxform,se3.inv((baxter.left_gripper_center_xform[0],vectorops.add(left_gripper_center_xform[1],pregrasp_shift))))
                pregoal = ik.objective(self.left_gripper_link,R=Tpg[0],t=Tpg[1])
            else:
                Tg = se3.mul(gxform,se3.inv(baxter.right_gripper_center_xform))
                goal = ik.objective(self.right_gripper_link,R=Tg[0],t=Tg[1])
                Tpg = se3.mul(gxform,se3.inv((baxter.right_gripper_center_xform[0],vectorops.add(baxter.right_gripper_center_xform[1],pregrasp_shift))))
                pregoal = ik.objective(self.right_gripper_link,R=Tpg[0],t=Tpg[1])
            grasp_goals.append(goal)
            pregrasp_goals.append(pregoal)
        sortedSolutions = self.get_ik_solutions(pregrasp_goals,[self.active_limb]*len(grasp_goals),qcmd)
        if len(sortedSolutions)==0: return False

        #prototyping hack: move straight to target
        if SKIP_PATH_PLANNING:
            for pregrasp in sortedSolutions:
                graspIndex = pregrasp[1]
                gsolns = self.get_ik_solutions([grasp_goals[graspIndex]],[self.active_limb],pregrasp[0],maxResults=1)
                if len(gsolns)==0:
                    print "Couldn't find grasp config for pregrasp? Trying another"
                else:
                    self.waitForMove()
                    self.sendPath([pregrasp[0],gsolns[0][0]])
                    self.active_grasp = grasps[graspIndex][0]
                    return True
            print "Planning failed"
            return False
        for solution in sortedSolutions:
            path = self.planner.plan(qcmd,solution[0])
            graspIndex = solution[1]
            if path != None:
                #now solve for grasp
                gsolns = self.get_ik_solutions([grasp_goals[graspIndex]],[self.active_limb],path[-1],maxResults=1)
                if len(gsolns)==0:
                    print "Couldn't find grasp config??"
                else:
                    self.waitForMove()
                    self.sendPath(path+[gsolns[0][0]])
                    self.active_grasp = grasps[graspIndex][0]
                    return True
        print "Planning failed"
        return False

    def move_to_ungrasp_object(self,object):
        """Sets the robot's configuration so the gripper ungrasps the object.

        If successful, sends the motion to the low-level controller and
        returns True.
        
        Otherwise, does not modify the low-level controller and returns False.
        """
        assert len(object.info.grasps) > 0,"Object doesn't define any grasps"
        return True

    def move_gripper_upright(self,limb,moveVector,collisionchecker = None):
        vertical = [0,0,0.1]
        if self.active_limb == 'left':
            gripperlink = self.left_gripper_link
        else:
            gripperlink = self.right_gripper_link
        qcur = self.robot.getConfig()
        vertical_in_gripper_frame = so3.apply(so3.inv(gripperlink.getTransform()[0]),vertical)
        centerpos = se3.mul(gripperlink.getTransform(),baxter.left_gripper_center_xform)[1]
        move_target = vectorops.add(centerpos,moveVector)
        movegoal = ik.objective(gripperlink,local=[baxter.left_gripper_center_xform[1],vectorops.add(baxter.left_gripper_center_xform[1],vertical_in_gripper_frame)],world=[move_target,vectorops.add(move_target,vertical)])
            
        sortedSolutions = self.get_ik_solutions([movegoal],[self.active_limb],qcur,validity_checker=collisionchecker,maxResults=1)
        if len(sortedSolutions) == 0:
            print "No upright-movement config found"
            return False
        self.robot.setConfig(sortedSolutions[0][0])
        return True



    def move_to_order_bin(self,object):
        """Sets the robot's configuration so the gripper is over the order bin

        If successful, sends the motion to the low-level controller and
        returns True.
        
        Otherwise, does not modify the low-level controller and returns False.
        """
        
        qcmd = self.controller.getCommandedConfig()        
        left_target = se3.apply(apc.order_bin_xform,[0.0,0.2,apc.order_bin_bounds[1][2]+0.1])
        right_target = se3.apply(apc.order_bin_xform,[0.0,-0.2,apc.order_bin_bounds[1][2]+0.1])
        #retraction goal -- maintain vertical world axis
        liftVector = [0,0,0.04]
        retractVector = [-0.2,0,0]
        self.planner.rebuild_dynamic_objects()
        self.robot.setConfig(qcmd)
        if self.move_gripper_upright(self.active_limb,liftVector):
            self.controller.setMilestone(self.robot.getConfig())
            print "Moved to lift goal"
            self.waitForMove()
        collisionchecker = lambda x:self.planner.check_collision_free_with_object(x,object.info.geometry,self.active_grasp)
        if self.move_gripper_upright(self.active_limb,retractVector,collisionchecker):
            self.controller.setMilestone(self.robot.getConfig())
            print "Moved to retract goal"
            self.waitForMove()
        retractConfig = self.robot.getConfig()

        if self.active_limb == 'left':
            placegoal = ik.objective(self.left_gripper_link,local=left_gripper_center_xform[1],world=left_target)
        else:
            placegoal = ik.objective(self.right_gripper_link,local=baxter.right_gripper_center_xform[1],world=right_target)
        sortedSolutions = self.get_ik_solutions([placegoal],[self.active_limb],retractConfig,tol=0.1)
        if len(sortedSolutions) == 0:
            print "Failed to find placement config"
        for solution in sortedSolutions:
            path = self.planner.plan_transfer(retractConfig,solution[0],self.active_limb,object,self.active_grasp)
            if path != None:
                self.waitForMove()
                self.sendPath(path)
                return True
        print "Planning failed"        
        return False

    def sendPath(self,path):
        self.controller.setMilestone(path[0])
        for q in path[1:]:
            self.controller.appendMilestone(q)
