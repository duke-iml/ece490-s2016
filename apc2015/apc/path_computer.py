#!/usr/bin/python

from klampt import *
from klampt.glprogram import *
from klampt import vectorops,so3,se3,gldraw,ik,loader
from klampt.robotsim import Geometry3D
from planner import *
import baxter
#import baxter_reflex as baxter
import motion
import apc
import os
import math
import random
import pickle
import json
from threading import Thread,Lock
from Queue import Queue

#configuration variables
#Question 1,2,3: set NO_SIMULATION_COLLISIONS = 1
#Question 4: set NO_SIMULATION_COLLISIONS = 0
NO_SIMULATION_COLLISIONS = 1
#Turn this on to help fast prototyping of later stages
FAKE_SIMULATION = 1
#Turn this on to log commands to file
LOG_COMMANDS = 0
command_log_file = 'commands.txt'

SKIP_PATH_PLANNING = 0

#The path of the klampt_models directory
model_dir = "../klampt_models/"

#the transformation of the order bin
order_bin_xform = (so3.identity(),[0.5,0,0])
#the local bounding box of the order bin
order_bin_bounds = ([-0.2,-0.4,0],[0.2,0.4,0.7])



#a list of actual items -- this is only used for the fake perception module, and your
#code should not use these items directly
ground_truth_items = []
ground_truth_shelf_xform = se3.identity()
def init_ground_truth():
    global ground_truth_items
    apc.load_fake_items()
    apc.load_all_items(gripper=baxter.gripper.gripper_name)
    #ground_truth_items = [apc.ItemInBin(apc.item_info['tall_item'],'bin_C'),
    ground_truth_items = [apc.ItemInBin(apc.item_info['cheezit_bit_original'],'bin_C'),
                          apc.ItemInBin(apc.item_info['small_item'],'bin_D'),
                          apc.ItemInBin(apc.item_info['med_item'],'bin_J')]
    #only load geometries for items that were spawned
    names = set()
    for item in ground_truth_items:
        names.add(item.info.name)
    print "Loading geometries for",','.join(names)
    for name in names:
        item = apc.item_info[name]
        item.geometry = apc.load_item_geometry(item)

    #set_in_bin_xform uses bounding boxes to set vertical offsets, and 
    #bounding boxes are set up only *after* geometries are loaded
    ground_truth_items[0].set_in_bin_xform(ground_truth_shelf_xform,0.3,0.3,0.0)
    ground_truth_items[1].set_in_bin_xform(ground_truth_shelf_xform,0.5,0.1,math.pi/4)
    ground_truth_items[2].set_in_bin_xform(ground_truth_shelf_xform,0.4,0.25,math.pi/2)
    
    return

def run_perception_on_shelf(knowledge):
    """This is a fake perception module that simply reveals the shelf
    xform."""
    knowledge.shelf_xform = ground_truth_shelf_xform

def run_perception_on_bin(knowledge,bin_name):
    """This is a fake perception module that simply reveals all the items
    the given bin."""
    global ground_truth_items
    if knowledge.bin_contents[bin_name]==None:
        #not sensed yet
        knowledge.bin_contents[bin_name] = []
        for item in ground_truth_items:
            if item.bin_name == bin_name:
                #place it in the bin
                knowledge.bin_contents[bin_name].append(item)
    return




class PickingController:
    """Maintains the robot's knowledge base and internal state.  Most of
    your code will go here.  Members include:
    - knowledge: a KnowledgeBase object
    - planner: an LimbPlanner object, which *you will implement and use*
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
        self.planner = LimbPlanner(self.world,self.knowledge)
        self.state = 'ready'
        self.active_limb = None
        self.planningLimb = 'right'
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
        id_to_index = dict([(self.robot.getLink(i).getID(),i) for i in range(self.robot.numLinks())])
        self.left_arm_indices = [id_to_index[i.getID()] for i in self.left_arm_links]
        self.right_arm_indices = [id_to_index[i.getID()] for i in self.right_arm_links]

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
                    run_perception_on_bin(self.knowledge,b)
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
                #else:
                    #print "IK solution for goal",index,"was in collision, trying again"
        if len(ikSolutions)==0:
            #print "No collision free IK solution"
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

        #try:
        #    pathFile = open('cached_motions/' + self.planningLimb + '_home_2' + bin_name)
        #    path = pickle.load(pathFile)
        #    print "found plan in cache"
        #except IOError:

        #place +z in the +x axis, y in the +z axis, and x in the -y axis
        if self.planningLimb == 'left':
            goal = ik.objective(self.left_camera_link,R=[0,0,-1,1,0,0,0,1,0],t=world_offset)
        else:
            goal = ik.objective(self.right_camera_link,R=[0,0,-1,1,0,0,0,1,0],t=world_offset)

        qcmd = self.controller.getCommandedConfig()
        #limbs = ['left','right']
        limbs = [self.planningLimb]
        sortedSolutions = self.get_ik_solutions([goal],limbs,qcmd)
        #sortedSolutions = self.get_ik_solutions([left_goal,right_goal],limbs,qcmd)
        #restrict to only right hand movements
        #limbs = ['right']
        #sortedSolutions = self.get_ik_solutions([right_goal],limbs,qcmd)
        if len(sortedSolutions)==0:
            print 'no ik solutions'
            return False;
        #prototyping hack: move straight to target
        if SKIP_PATH_PLANNING:
            self.controller.setMilestone(sortedSolutions[0][0])
            self.active_limb = limbs[sortedSolutions[0][1]]
            return True

        i = 0
        for solution in sortedSolutions:
            path = self.planner.plan(qcmd,solution[0])
            i = i + 1
            if path != None:
                print 'number', i
                self.controller.setMilestone(sortedSolutions[0][0])
                self.active_limb = self.planningLimb
                self.printPath(path,'cached_motions/'+self.planningLimb+'_home_2'+bin_name)
                return True
        print "Failed to plan path"
        return False

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
        baxter.set_model_gripper_command(self.robot,self.active_limb,[1])
        qcmd = self.robot.getConfig()
        
        #solve an ik solution to one of the grasps
        grasp_goals = []
        pregrasp_goals = []
        pregrasp_shift = [0,0,0.05]
        for (grasp,gxform) in grasps:
            if self.active_limb == 'left':
                Tg = se3.mul(gxform,se3.inv(baxter.left_gripper_center_xform))
                Tpg = se3.mul(gxform,se3.inv((baxter.left_gripper_center_xform[0],vectorops.add(baxter.left_gripper_center_xform[1],pregrasp_shift))))
                Tg = (so3.from_moment(so3.moment(Tg[0])),Tg[1])
                Tpg = (so3.from_moment(so3.moment(Tpg[0])),Tpg[1])
                goal = ik.objective(self.left_gripper_link,R=Tg[0],t=Tg[1])
                pregoal = ik.objective(self.left_gripper_link,R=Tpg[0],t=Tpg[1])
            else:
                Tg = se3.mul(gxform,se3.inv(baxter.right_gripper_center_xform))
                Tpg = se3.mul(gxform,se3.inv((baxter.right_gripper_center_xform[0],vectorops.add(baxter.right_gripper_center_xform[1],pregrasp_shift))))
                #hack: weird matrices coming from ODE aren't rotations within tolerance, gives abort
                Tg = (so3.from_moment(so3.moment(Tg[0])),Tg[1])
                Tpg = (so3.from_moment(so3.moment(Tpg[0])),Tpg[1])
                goal = ik.objective(self.right_gripper_link,R=Tg[0],t=Tg[1])
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
        left_target = se3.apply(order_bin_xform,[0.0,0.2,order_bin_bounds[1][2]+0.1])
        right_target = se3.apply(order_bin_xform,[0.0,-0.2,order_bin_bounds[1][2]+0.1])
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
            placegoal = ik.objective(self.left_gripper_link,local=baxter.left_gripper_center_xform[1],world=left_target)
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

    def goToOrderBin(self,limb,fromWhere):
        qcmd = self.controller.getCommandedConfig()        
        left_target = se3.apply(order_bin_xform,[0.0,0.2,order_bin_bounds[1][2]+0.1])
        right_target = se3.apply(order_bin_xform,[0.0,-0.2,order_bin_bounds[1][2]+0.1])

        if limb == 'left':
            placegoal = ik.objective(self.left_gripper_link,local=baxter.left_gripper_center_xform[1],world=left_target)
        else:
            placegoal = ik.objective(self.right_gripper_link,local=baxter.right_gripper_center_xform[1],world=right_target)

        sortedSolutions = self.get_ik_solutions([placegoal],[limb],qcmd,tol=0.1)
        if len(sortedSolutions) == 0:
            print "Failed to find placement config"
        for solution in sortedSolutions:
            if limb == 'left':
                path = self.planner.plan(qcmd, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.3717782274123387, -0.674937510436548, -0.33948575074327003, 2.1672104893835455, -0.08734593872357976, 0.0, 0.12728394408809324, 9.587078444893871e-09, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0600000000000005, 0.0, 2.31, 0.0, 0.0, 0.28, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            else:
                path = self.planner.plan(qcmd, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.1399999999999997, 0.0, 2.32, 0.0, 0.0, 0.32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.13866956622769833, -0.6983335926844152, 0.43471512229342457, 2.241413168181005, 0.09879187313355131, 0.0, 0.30929795433686946, 0.13997178733833623, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

                #path = self.planner.plan(qcmd,solution[0])
            if path != None:
                if(fromWhere == 'nofile'):
                    self.waitForMove()
                    self.sendPath(path)
                else:
                    self.waitForMove()
                    self.sendPath(path)
                    self.printPath(path,'cached_motions/'+limb+'_from_bin'+fromWhere+'_2home')
                return True
        print "Planning failed"        
        return False

    def printPath(self,path,filename):
        file = open(filename,'w')
        #pickle.dump(path, file)
        json.dump(path,file)

    def makePlanCache(self,limb):
        if limb == 'left':
            self.planningLimb = 'left'
            self.goToOrderBin('left','default')
            vec = ['a','b','c','d','e','f','g','h','i','j','k','l']
            for letter in vec:
                self.viewBinAction('bin_'+letter.upper())
                self.goToOrderBin('left',letter.upper())
        else:
            self.planningLimb = 'right'
            self.goToOrderBin('right','default')
            vec = ['a','b','c','d','e','f','g','h','i','j','k','l']
            for letter in vec:
                self.viewBinAction('bin_'+letter.upper())
                self.goToOrderBin('right',letter.upper())

    def spawn_item_test(self):
        self.world.loadElement('../klampt_models/kiva_pod/meshes/pod_lowres.stl')
        reorient = ([1,0,0,0,0,1,0,-1,0],[0,0,0.01])
        #kiva
        Trel = (so3.rotation((0,0,1),-math.pi/2),[.3,0,0])
        
        T = reorient
        self.world.terrain(2).geometry().transform(*se3.mul(Trel,T))
        self.planner.updateWorld(self.world)
        


def draw_xformed(xform,localDrawFunc):
    """Draws something given a se3 transformation and a drawing function
    that draws the object in its local frame.

    E.g., draw_xformed(xform,lambda:gldraw.box([ax,ay,az],[bx,by,bz])) draws
    a box oriented and translated by xform."""
    mat = zip(*se3.homogeneous(xform))
    mat = sum([list(coli) for coli in mat],[])

    glPushMatrix()
    glMultMatrixf(mat)
    localDrawFunc()
    glPopMatrix()

def draw_oriented_box(xform,bmin,bmax):
    """Helper: draws an oriented box"""
    draw_xformed(xform,lambda:gldraw.box(bmin,bmax))

def draw_wire_box(bmin,bmax):
    """Helper: draws a wireframe box"""
    glBegin(GL_LINE_LOOP)
    glVertex3f(bmin[0],bmin[1],bmin[2])
    glVertex3f(bmin[0],bmin[1],bmax[2])
    glVertex3f(bmin[0],bmax[1],bmax[2])
    glVertex3f(bmin[0],bmax[1],bmin[2])
    glEnd()
    glBegin(GL_LINE_LOOP)
    glVertex3f(bmax[0],bmin[1],bmin[2])
    glVertex3f(bmax[0],bmin[1],bmax[2])
    glVertex3f(bmax[0],bmax[1],bmax[2])
    glVertex3f(bmax[0],bmax[1],bmin[2])
    glEnd()
    glBegin(GL_LINES)
    glVertex3f(bmin[0],bmin[1],bmin[2])
    glVertex3f(bmax[0],bmin[1],bmin[2])
    glVertex3f(bmin[0],bmin[1],bmax[2])
    glVertex3f(bmax[0],bmin[1],bmax[2])
    glVertex3f(bmin[0],bmax[1],bmax[2])
    glVertex3f(bmax[0],bmax[1],bmax[2])
    glVertex3f(bmin[0],bmax[1],bmin[2])
    glVertex3f(bmax[0],bmax[1],bmin[2])
    glEnd()

def draw_oriented_wire_box(xform,bmin,bmax):
    """Helper: draws an oriented wireframe box"""
    draw_xformed(xform,lambda:draw_wire_box(bmin,bmax))


def run_controller(controller,command_queue):
    run_perception_on_shelf(controller.knowledge)
    while True:
        c = command_queue.get()
        if c != None:
            print "Running command",c
            if c >= 'a' and c <= 'l':
                controller.viewBinAction('bin_'+c.upper())
            elif c == 'x':
                controller.graspAction()
            elif c == 'u':
                controller.ungraspAction()
            elif c == 'p':
                controller.placeInOrderBinAction()
            elif c == 'o':
                controller.fulfillOrderAction(['med_item','small_item'])
            elif c == 'r':
                #controller.goToOrderBin('left')
                controller.goToOrderBin('right','nofile')
            elif c =='t':
                controller.makePlanCache('left')
                controller.makePlanCache('right')
            elif c == 'w':
                 controller.spawn_item_test()
            elif c=='q':
                break
        else:
            print "Waiting for command..."
            time.sleep(0.1)
    print "Done"
    

class MyGLViewer(GLRealtimeProgram):
    """This class is used to simulate / interact with with the world model
    in hw4.
    
    Pressing 'a-l' runs the view_bin method which should set the robot to a
    configuration that places a hand camera such that it points inside the
    bin.

    Pressing 's' should pause / unpause the simulation.

    Pressing 'x' should "grasp" an object in the currently pointed-to-bin
    with either one of the hands at the designated grasp point.

    Pressing 'u' should "ungrasp" an object currently grasped inside a bin.

    Pressing 'p' should "put down" an object in the order bin
    """
    def __init__(self,simworld,planworld):
        GLRealtimeProgram.__init__(self,"My GL program")
        self.simworld = simworld
        self.planworld = planworld
        self.sim = Simulator(simworld)
        self.simulate = True
        #self.sim.simulate(0)
        
        #you can set these to true to draw the bins, grasps, and/or gripper/camera frames
        self.draw_bins = False
        self.draw_grasps = True
        self.draw_gripper_and_camera = True

        #initialize controllers
        if FAKE_SIMULATION:
            self.low_level_controller = motion.FakeLowLevelController(simworld.robot(0))
        else:
            self.low_level_controller = motion.SimLowLevelController(simworld.robot(0),self.sim.getController(0))
        if LOG_COMMANDS:
            self.low_level_controller = motion.LoggingController(self.low_level_controller,command_log_file)
        self.command_queue = Queue()
        self.picking_controller = PickingController(planworld,self.low_level_controller)
        self.picking_thread = Thread(target=run_controller,args=(self.picking_controller,self.command_queue))
        self.picking_thread.start()

    def idle(self):
        if self.simulate:
            self.sim.simulate(self.dt)
            #for Q2
            if self.simworld.numRigidObjects() >= len(ground_truth_items):
                ofs = self.simworld.numRigidObjects()-len(ground_truth_items)
                for i,item in enumerate(ground_truth_items):
                    T = self.sim.getBody(self.simworld.rigidObject(ofs+i)).getTransform()
                    item.xform = T
            glutPostRedisplay()

    def display(self):
        #you may run auxiliary openGL calls, if you wish to visually debug

        #draw the world
        self.sim.updateWorld()
        self.simworld.drawGL()

        #if you're doing question 1, this will draw the shelf and floor
        if self.simworld.numTerrains()==0:
            for i in range(self.planworld.numTerrains()):
                self.planworld.terrain(i).drawGL()

        #draw commanded configurations
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0,1,0,0.5])
        for i in xrange(self.simworld.numRobots()):
            r = self.simworld.robot(i)
            #q = self.sim.getController(i).getCommandedConfig()
            q = self.low_level_controller.getCommandedConfig()
            r.setConfig(q)
            r.drawGL(False)
        glDisable(GL_BLEND)
        
        global ground_truth_items
        
        #show bin boxes
        if self.draw_bins:
            glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,1,0,1])
            for b in apc.bin_bounds.values():
                draw_oriented_box(self.picking_controller.knowledge.shelf_xform,b[0],b[1])
            for b in apc.bin_names:
                c = self.picking_controller.knowledge.bin_front_center(b)
                if c:
                    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,1,0.5,1])                    
                    r = 0.01
                    gldraw.box([c[0]-r,c[1]-r,c[2]-r],[c[0]+r,c[1]+r,c[2]+r])
                c = self.picking_controller.knowledge.bin_vantage_point(b)
                if c:
                    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0.5,1,0.5,1])
                    r = 0.01
                    gldraw.box([c[0]-r,c[1]-r,c[2]-r],[c[0]+r,c[1]+r,c[2]+r])

        
        #show object state
        for i in ground_truth_items:
            if i.xform == None:
                continue
            if i.bin_name == 'order_bin':
                continue
            #if perceived, draw in solid color
            if self.picking_controller.knowledge.bin_contents[i.bin_name]!=None and i in self.picking_controller.knowledge.bin_contents[i.bin_name]:
                glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,0.5,0,1])
                draw_oriented_box(i.xform,i.info.bmin,i.info.bmax)
            else:
                #otherwise, draw in wireframe
                glDisable(GL_LIGHTING)
                glColor3f(1,0.5,0)
                draw_oriented_wire_box(i.xform,i.info.bmin,i.info.bmax)
                glEnable(GL_LIGHTING)
            if self.draw_grasps:
                #draw grasps, if available
                g = self.picking_controller.knowledge.grasp_xforms(i)
                if g:
                    for grasp,xform in g:
                        gldraw.xform_widget(xform,0.05,0.005,fancy=False)

        obj,limb,grasp = self.picking_controller.held_object,self.picking_controller.active_limb,self.picking_controller.active_grasp
        if obj != None:
            if limb == 'left':
                gripper_xform = self.simworld.robot(0).getLink(baxter.left_gripper_link_name).getTransform()
            else:
                gripper_xform = self.simworld.robot(0).getLink(baxter.right_gripper_link_name).getTransform()
            objxform = se3.mul(gripper_xform,se3.mul(baxter.left_gripper_center_xform,se3.inv(grasp.grasp_xform)))
            glDisable(GL_LIGHTING)
            glColor3f(1,0.5,0)
            draw_oriented_wire_box(objxform,obj.info.bmin,obj.info.bmax)
            glEnable(GL_LIGHTING)

        #show gripper and camera frames
        if self.draw_gripper_and_camera:
            left_camera_link = self.simworld.robot(0).getLink(baxter.left_camera_link_name)
            right_camera_link = self.simworld.robot(0).getLink(baxter.right_camera_link_name)
            left_gripper_link = self.simworld.robot(0).getLink(baxter.left_gripper_link_name)
            right_gripper_link = self.simworld.robot(0).getLink(baxter.right_gripper_link_name)
            gldraw.xform_widget(left_camera_link.getTransform(),0.1,0.01,fancy=False)
            gldraw.xform_widget(right_camera_link.getTransform(),0.1,0.01,fancy=False)
            gldraw.xform_widget(se3.mul(left_gripper_link.getTransform(),baxter.left_gripper_center_xform),0.05,0.005,fancy=False)
            gldraw.xform_widget(se3.mul(right_gripper_link.getTransform(),baxter.right_gripper_center_xform),0.05,0.005,fancy=False)

        #draw order box 
        glDisable(GL_LIGHTING)
        glColor3f(1,0,0)
        draw_oriented_wire_box(order_bin_xform,order_bin_bounds[0],order_bin_bounds[1])
        glEnable(GL_LIGHTING)
        return

    def keyboardfunc(self,c,x,y):
        c = c.lower()
        if c=='s':
            self.simulate = not self.simulate
            print "Simulating:",self.simulate
        else:
            self.command_queue.put(c)
            if c=='q':
                self.picking_thread.join()
                exit(0)
        glutPostRedisplay()

def load_apc_world():
    """Produces a world with only the Baxter, shelf, and ground plane in it."""
    world = WorldModel()
    #uncomment these lines and comment out the next 2 if you want to use the
    #full Baxter model
    #print "Loading full Baxter model (be patient, this will take a minute)..."
    #world.loadElement(os.path.join(model_dir,"baxter.rob"))
    print "Loading simplified Baxter model..."
    world.loadElement(os.path.join(model_dir,baxter.klampt_model_name))
    
    #print "Loading Kiva pod model..."
    #world.loadElement(os.path.join(model_dir,"kiva_pod/meshes/pod_lowres.stl"))
    
    print 'Loading north hall shelf...'
    world.loadElement(os.path.join(model_dir,"north_shelf/mesh/shelf.stl"))
    
    print "Loading plane model..."
    world.loadElement(os.path.join(model_dir,"plane.env"))
    
    #shift the Baxter up a bit (95cm)
    Rbase,tbase = world.robot(0).getLink(0).getParentTransform()
    world.robot(0).getLink(0).setParentTransform(Rbase,(0,0,0.95))
    world.robot(0).setConfig(world.robot(0).getConfig())
    
    #translate pod to be in front of the robot, and rotate the pod by 90 degrees 
    reorient = ([1,0,0,0,0,1,0,-1,0],[0,0,0])
    
    #kiva
    #Trel = (so3.rotation((0,0,1),-math.pi/2),[1.3,0,0])
    
    #north shelf
    Trel = (so3.rotation((0,0,1),-3*math.pi/2),[1.,0,0])
    Trel = se3.mul(Trel,(so3.rotation((1,0,0),-math.pi/2),[0,0,0]))
    
    T = reorient
    #world.terrain(0).geometry().transform(*Trel)
    world.terrain(0).geometry().transform(*se3.mul(Trel,T))
    

    #initialize the shelf xform for the visualizer and object
    #xform initialization
    global ground_truth_shelf_xform
    ground_truth_shelf_xform = se3.mul(Trel,T)

#####################################test#################################################
    # world.loadElement('../klampt_models/kiva_pod/meshes/pod_lowres.stl')
    # reorient = ([1,0,0,0,0,1,0,-1,0],[0,0,0.01])
    # #kiva
    # Trel = (so3.rotation((0,0,1),-math.pi/2),[.3,0,0])
    
    # T = reorient
    # world.terrain(2).geometry().transform(*se3.mul(Trel,T))

##########################################################################################
    return world

def load_baxter_only_world():
    """Produces a world with only the Baxter in it."""
    world = WorldModel()
    print "Loading simplified Baxter model..."
    world.loadElement(os.path.join(model_dir,baxter.klampt_model_name))
    #shift the Baxter up a bit (95cm)
    Rbase,tbase = world.robot(0).getLink(0).getParentTransform()
    world.robot(0).getLink(0).setParentTransform(Rbase,(0,0,0.95))
    world.robot(0).setConfig(world.robot(0).getConfig())
    return world

def main():
    """The main loop that loads the planning / simulation models and
    starts the OpenGL visualizer."""
    world = load_apc_world()

    init_ground_truth()

    if NO_SIMULATION_COLLISIONS:
        simworld = load_baxter_only_world()
    else:
        simworld = load_apc_world()
        global ground_truth_items
        apc.spawn_items_in_world(ground_truth_items,simworld)

    simworld.robot(0).setConfig(baxter.rest_config)

    
    #run the visualizer
    visualizer = MyGLViewer(simworld,world)
    visualizer.run()

	


if __name__ == "__main__":
    main()

###############################################################
# WRITTEN ANSWERS / COMMENTS:
# Q1.
#
#
# Q2.
#
#
# Q3.
#
#
# Q4.
#
#
# Q5 (bonus question).
#
#
