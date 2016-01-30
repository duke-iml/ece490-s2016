#!/usr/bin/python

from klampt import *
from klampt.glprogram import *
from klampt import vectorops,so3,se3,gldraw,ik,loader
from klampt.robotsim import Geometry3D
from baxter import *
from hw4_planner_mtd13 import *
import apc
import os
import math
import random
from threading import Thread,Lock
from Queue import Queue


#configuration variables
#Question 1,2,3: set NO_SIMULATION_COLLISIONS = 1
#Question 4: set NO_SIMULATION_COLLISIONS = 0
NO_SIMULATION_COLLISIONS = 0
#Set FAKE_SIMULATION to 1 to help fast prototyping of later stages.
#You won't have to wait for the arm to move.
FAKE_SIMULATION = 0


#The path of the klampt_models directory
model_dir = "../klampt_models/"

#resting configuration
baxter_rest_config = [0.0]*60

#the transformation of the order bin
order_bin_xform = (so3.identity(),[0.5,0,0])
#the local bounding box of the order bin
order_bin_bounds = ([-0.2,-0.4,0],[0.2,0.4,0.7])

debug_xform = None
debug_xform2 = None
debug_xform3 = None
debug_xform4 = None

class KnowledgeBase:
    """A structure containing the robot's dynamic knowledge about the world.
    Members:
    - bin_contents: a map from bin names to lists of known items in
      the bin.  Items are given by apc.ItemInBin objects.
    - order_bin_contents: the list of objects already in the order bin.
      also given by apc.ItemInBin objects
    - shelf_xform: the transformation (rotation, translation) of the bottom
      center of the shelf in world coordinates.  The x coordinate increases
      from left to right, the y coordinate increases from bottom to top,
      and the z coordinate increases from back to front.
      this will be loaded dynamically either from perception or hard coded.

    (in this homework assignment we will use the fake perception module
    to populate the bin contents, and assume the shelf xform is
    estimated perfectly.)
    """
    def __init__(self):
        self.bin_contents = dict((n,None) for n in apc.bin_names)
        self.order_bin_contents = []
        self.shelf_xform = se3.identity()

    def bin_front_center(self,bin_name):
        bmin,bmax = apc.bin_bounds[bin_name]
        local_center = [(bmin[0]+bmax[0])*0.5,(bmin[1]+bmax[1])*0.5,bmax[2]]
        world_center = se3.apply(self.shelf_xform,local_center)
        return world_center


    def bin_vantage_point(self,bin_name):
        world_center = self.bin_front_center(bin_name)
        #20cm offset
        world_offset = so3.apply(self.shelf_xform[0],[0,0,0.2])
        return vectorops.add(world_center,world_offset)


    def grasp_xforms(self,object):
        if object.xform == None: return None
        res = []
        for g in object.info.grasps:
            grasp_xform_world = se3.mul(object.xform,g.grasp_xform)
            res.append((g,grasp_xform_world))
        return res


#a list of actual items -- this is only used for the fake perception module, and your
#code should not use these items directly
ground_truth_items = []
ground_truth_shelf_xform = se3.identity()
def init_ground_truth():
    global ground_truth_items
    ground_truth_items = [apc.ItemInBin(apc.tall_item,'bin_B'),
                          apc.ItemInBin(apc.small_item,'bin_D'),
                          apc.ItemInBin(apc.med_item,'bin_H')]
    ground_truth_items[0].set_in_bin_xform(ground_truth_shelf_xform,0.25,0.2,0.0)
    ground_truth_items[1].set_in_bin_xform(ground_truth_shelf_xform,0.5,0.1,math.pi/4)
    ground_truth_items[2].set_in_bin_xform(ground_truth_shelf_xform,0.6,0.4,math.pi/2)

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


class LowLevelController:
    """A low-level interface to the Baxter robot (with parallel jaw
    grippers).  Does appropriate locking for multi-threaded use.
    You should use this in your picking controller."""
    def __init__(self,robotModel,robotController):
        self.robotModel = robotModel
        self.controller = robotController
        self.lock = Lock()
    def getSensedConfig(self):
        self.lock.acquire()
        res = self.controller.getSensedConfig()
        self.lock.release()
        return res
    def getSensedVelocity(self):
        self.lock.acquire()
        res = self.controller.getSensedVelocity()
        self.lock.release()
        return res
    def getCommandedConfig(self):
        self.lock.acquire()
        res = self.controller.getCommandedConfig()
        self.lock.release()
        return res
    def getCommandedVelocity(self):
        self.lock.acquire()
        res = self.controller.getCommandedVelocity()
        self.lock.release()
        return res
    def setPIDCommand(self,configuration,velocity):
        """Sets the controller to a PID command mode"""
        self.lock.acquire()
        self.controller.setPIDCommand(configuration,velocity)
        self.lock.release()
    def setMilestone(self,destination,endvelocity=None):
        """Immediately sets the motion queue to move to the given
        milestone.  If endvelocity is given, then the end of the
        queue will be moving at that velocity.  Otherwise, the end
        velocity will be zero."""
        self.lock.acquire()
        if endvelocity == None: self.controller.setMilestone(destination)
        else: self.controller.setMilestone(destination,endvelocity)
        self.lock.release()
    def addMilestone(self,destination,endvelocity=None):
        """Appends a milestone to the motion queue.  If endvelocity
        is given, then the end of the queue will be moving at that velocity.
        Otherwise, the end velocity will be zero."""
        self.lock.acquire()
        if endvelocity == None: self.controller.addMilestone(destination)
        else: self.controller.addMilestone(destination,endvelocity)
        self.lock.release()
    def isMoving(self):
        return self.controller.remainingTime()>0
    def remainingTime(self):
        return self.controller.remainingTime()
    def commandGripper(self,limb,command):
        """Sends the command to the indicated gripper.
        For the parallel-jaw gripper, [0] is closed, [1] is open
        Warning: don't do this while moving"""
        self.lock.acquire()
        q = self.controller.getCommandedConfig()
        self.robotModel.setConfig(q)
        value = command[0]
        if limb=='left':
            print "Opening left gripper to",value
            self.robotModel.getDriver(15).setValue(value*0.045)
            self.robotModel.getDriver(16).setValue(-value*0.045)
        else:
            print "Opening right gripper to",value
            self.robotModel.getDriver(17).setValue(value*0.045)
            self.robotModel.getDriver(18).setValue(-value*0.045)
        self.controller.setMilestone(self.robotModel.getConfig())
        self.lock.release()


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
        self.knowledge = KnowledgeBase()
        self.planner = LimbPlanner(self.world,self.knowledge)
        self.state = 'ready'
        self.active_limb = None
        self.active_grasp = None
        self.current_bin = None
        self.held_object = None
        #these may be helpful
        self.left_camera_link = self.robot.getLink(left_camera_link_name)
        self.right_camera_link = self.robot.getLink(right_camera_link_name)
        self.left_gripper_link = self.robot.getLink(left_gripper_link_name)
        self.right_gripper_link = self.robot.getLink(right_gripper_link_name)
        self.left_arm_links = [self.robot.getLink(i) for i in left_arm_link_names]
        self.right_arm_links = [self.robot.getLink(i) for i in right_arm_link_names]
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
                else:
                    print "Move to bin",b,"failed"
                    return False
            else:
                print "Invalid bin",b
                return False
        return True

    def graspAction(self):
        self.waitForMove()
        self.controller.commandGripper(self.active_limb,[1])
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

    def randomize_limb_position(self,limb,range=None):
        """Helper: randomizes the limb configuration in self.robot.
        limb can be 'left' or 'right'.  If range is provided, then
        this samples in a range around the current commanded config"""
        qmin,qmax = self.robot.getJointLimits()
        if range == None:
            q = baxter_rest_config[:]
            if limb == 'left':
                for j in self.left_arm_indices:
                    q[j] = random.uniform(qmin[j],qmax[j])
            else:
                for j in self.right_arm_indices:
                    q[j] = random.uniform(qmin[j],qmax[j])
            self.robot.setConfig(q)
        else:
            q = self.controller.getCommandedConfig()
            if limb == 'left':
                for j in self.left_arm_indices:
                    q[j] = max(qmin[j],min(qmax[j],random.uniform(q[j]-range,q[j]+range)))
            else:
                for j in self.right_arm_indices:
                    q[j] = max(qmin[j],min(qmax[j],random.uniform(q[j]-range,q[j]+range)))
            self.robot.setConfig(q)
        return

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
        for i in range(1000):
            if random.random() < 0.5:
                if i == 0:
                    self.robot.setConfig(qcmd)
                else:
                    self.randomize_limb_position('left')
                if ik.solve(left_goal):
                    # check that the solution is collision-free
                    if self.planner.check_limb_collision_free('left', self.planner.get_limb_config(self.robot.getConfig(), 'left')):
                        # get a path from the starting position to this solution
                        path = self.planner.plan(qcmd, self.robot.getConfig())
                        if path:
                            # send the path
                            for q in path:
                                if not FAKE_SIMULATION:
                                    time.sleep(0.01)
                                self.controller.addMilestone(q)
                            self.active_limb = 'left'
                            return True
            else:
                if i == 0:
                    self.robot.setConfig(qcmd)
                else:
                    self.randomize_limb_position('right')
                if ik.solve(right_goal):
                    # check that the solution is collision-free
                    if self.planner.check_limb_collision_free('right', self.planner.get_limb_config(self.robot.getConfig(), 'right')):
                        # get a path from the starting position to this solution
                        path = self.planner.plan(qcmd, self.robot.getConfig())
                        if path:
                            # send the path
                            for q in path:
                                if not FAKE_SIMULATION:
                                    time.sleep(0.01)
                                self.controller.addMilestone(q)
                            self.active_limb = 'right'
                            return True

            time.sleep(0.01)

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

        grasps = self.knowledge.grasp_xforms(object)
        qcmd = self.controller.getCommandedConfig()

        # sort grasps by distance from the CoM
        a = se3.apply(object.xform, object.info.bmin)
        b = se3.apply(object.xform, object.info.bmax)
        com = (numpy.array(a) + numpy.array(b)) / 2
        sorted_grasps = sorted(grasps, key=lambda g: sum((g[1][1] - com)**2) * g[1][1][0])

        attempted_grasp_centers = []
        for (grasp,gxform) in sorted_grasps:
            # skip duplicate grasp centers
            if gxform[1] in attempted_grasp_centers:
                continue
            attempted_grasp_centers.append(gxform[1])

            def position_in_front_of_object(start):
                for m in range(3):
                    for n in range(3):
                        # start off aligned with the grasp target but 10 cm away
                        t = vectorops.add(gxform[1], [-0.1,0,0.01*m+0.05])

                        R = [0,0,-1, 0,1,0, 1,0,0]
                        R = so3.mul(R, so3.rotation([0,0,1], pi/2))
                        # try different approach angles from above
                        # this helps clear the shelf lip for short objects
                        R = so3.mul(so3.rotation([0,1,0], pi/10*n), R)

                        # produce an IK goal in the gripper frame
                        if self.active_limb == 'left':
                            (Rg, tg) = se3.mul((R, t), se3.inv(left_gripper_center_xform))
                            goal = ik.objective(self.left_gripper_link,R=Rg,t=tg)
                        else:
                            (Rg, tg) = se3.mul((R, t), se3.inv(right_gripper_center_xform))
                            goal = ik.objective(self.right_gripper_link,R=Rg,t=tg)

                        global debug_xform
                        debug_xform = (Rg, tg)

                        # only try 5 iterations because the grasp configuration
                        # can be altered to help find a solution
                        for i in range(5):
                            # do not randomize the limb since the limb is already so close
                            # to the desired location
                            self.robot.setConfig(start)
                            if ik.solve(goal):
                                # check that the solution is collision-free
                                if self.planner.check_limb_collision_free(self.active_limb, self.planner.get_limb_config(self.robot.getConfig(), self.active_limb)):
                                    # get a path from the starting position to this solution
                                    path = self.planner.plan(start, self.robot.getConfig())
                                    if path:
                                        return path
                            time.sleep(0.01)

                return None

            def position_grasp_center_on_object(start):
                a = se3.apply(object.xform, object.info.bmin)
                b = se3.apply(object.xform, object.info.bmax)
                # compute the object height
                obj_height = abs(a[2] - b[2])
                if obj_height > 0.05:
                    # approach large objects level first
                    ns = range(3)
                else:
                    # approach short object with a greater angle first
                    ns = list(reversed(range(3)))

                t = gxform[1]
                for m in range(6):
                    for n in ns:
                        for l in [ 0, 1, -1, 2, -2, 3, -3 ]:
                            # try shifting the grasp offset vertically
                            # this helps grasp short objects when the gripper might scrape the shelf
                            tg = vectorops.add(t, [0,0,0.005*m])

                            R = [0,0,-1, 0,1,0, 1,0,0]
                            R = so3.mul(R, so3.rotation([0,0,1], pi/2))
                            # try different approach angles from above
                            # this helps clear the shelf lip for short objects
                            R = so3.mul(so3.rotation([0,1,0], pi/10*n), R)
                            R = so3.mul(so3.rotation([0,0,1], pi/20*l), R)

                            # produce an IK goal in the gripper frame
                            if self.active_limb == 'left':
                                (Rg, tg) = se3.mul((R, tg), se3.inv(left_gripper_center_xform))
                                goal = ik.objective(self.left_gripper_link,R=Rg,t=tg)
                            else:
                                (Rg, tg) = se3.mul((R, tg), se3.inv(right_gripper_center_xform))
                                goal = ik.objective(self.right_gripper_link,R=Rg,t=tg)

                            global debug_xform2
                            debug_xform2 = (Rg, tg)

                            # only try 5 iterations because the grasp configuration
                            # can be altered to help find a solution
                            for i in range(5):
                                # do not randomize the limb since the limb is already so close
                                # to the desired location
                                self.robot.setConfig(start)
                                if ik.solve(goal):
                                    # check that the solution is collision-free
                                    if self.planner.check_limb_collision_free(self.active_limb, self.planner.get_limb_config(self.robot.getConfig(), self.active_limb)):
                                        # get a path from the starting position to this solution
                                        path = self.planner.plan(start, self.robot.getConfig())
                                        if path:
                                            return path

                                time.sleep(0.01)

                return None

            path1 = position_in_front_of_object(qcmd)
            if path1:
                path2 = position_grasp_center_on_object(path1[-1])
                if path2:
                    # send the path
                    for (i, q) in enumerate(self.planner.resample_path(path1 + path2, pi/500)):
                        if not FAKE_SIMULATION:
                            time.sleep(0.1)
                            if i in [ 0, len(path1) ]:
                                # let the robot settle a bit
                                time.sleep(1)
                        self.controller.addMilestone(q)

                    self.active_grasp = grasp
                    return True

        return False

    def move_to_ungrasp_object(self,object):
        """Sets the robot's configuration so the gripper ungrasps the object.

        If successful, sends the motion to the low-level controller and
        returns True.

        Otherwise, does not modify the low-level controller and returns False.
        """
        assert len(object.info.grasps) > 0,"Object doesn't define any grasps"
        return True

    def move_to_order_bin(self,object):
        """Sets the robot's configuration so the gripper is over the order bin

        If successful, sends the motion to the low-level controller and
        returns True.

        Otherwise, does not modify the low-level controller and returns False.
        """

        for i in range(100):

            def lift_up_object(start):
                self.robot.setConfig(start)
                if self.active_limb == 'left':
                    (R, t) = self.left_gripper_link.getTransform()
                else:
                    (R, t) = self.right_gripper_link.getTransform()

                # move the object to the center of the bin while lifting
                t[1] = self.knowledge.bin_front_center(self.current_bin)[1]

                for n in [ 3, 4, 2, 5, 1, 6 ]:
                    # move up the gripper up from its current location
                    tg = vectorops.add(t, [0,0,0.01*n])

                    # produce an IK goal in the gripper frame
                    if self.active_limb == 'left':
                        goal = ik.objective(self.left_gripper_link,R=R,t=tg)
                    else:
                        goal = ik.objective(self.right_gripper_link,R=R,t=tg)

                    global debug_xform
                    debug_xform = (R, tg)

                    # only try 5 iterations because the grasp configuration
                    # can be altered to help find a solution
                    for i in range(5):
                        # do not randomize the limb since the limb is already so close
                        # to the desired location
                        self.robot.setConfig(start)
                        if ik.solve(goal):
                            # check that the solution is collision-free
                            # ignore the grasped object
                            if self.planner.check_limb_collision_free(self.active_limb, self.planner.get_limb_config(self.robot.getConfig(), self.active_limb), ignore=[ object ]):
                                # get a path from the starting position to this solution
                                # ignore the grasped object
                                path = self.planner.plan(start, self.robot.getConfig(), ignore=[ object ])
                                if path:
                                    return path
                        time.sleep(0.01)

                return None

            def withdraw_object(start):
                a = se3.apply(object.xform, object.info.bmin)
                b = se3.apply(object.xform, object.info.bmax)
                # compute the object depth
                obj_depth = abs(a[0] - b[0])

                self.robot.setConfig(start)
                if self.active_limb == 'left':
                    (R, t) = self.left_gripper_link.getTransform()
                    ns = [ 0, 1, 2, 3, 4, 5, -1, -2, -3, -4, -5 ]
                else:
                    (R, t) = self.right_gripper_link.getTransform()
                    ns = [ 0, -1, -2, -3, -4, -5, 1, 2, 3, 4, 5 ]

                # compute the withdraw position with some padding
                t[0] = self.knowledge.bin_front_center(self.current_bin)[0] - obj_depth - 0.05

                for m in range(10):
                    for n in ns:
                        # allow the Z rotation of the gripper to change
                        Rg = so3.mul(so3.rotation([0,0,1], pi/10*n), R)

                        # produce an IK goal in the gripper frame
                        if self.active_limb == 'left':
                            (_, tg) = se3.mul((Rg, t), se3.inv(left_gripper_center_xform))
                            goal = ik.objective(self.left_gripper_link,R=Rg,t=tg)
                        else:
                            (_, tg) = se3.mul((Rg, t), se3.inv(right_gripper_center_xform))
                            goal = ik.objective(self.right_gripper_link,R=Rg,t=tg)

                        global debug_xform2
                        debug_xform2 = (Rg, tg)

                        # only try 5 iterations because the grasp configuration
                        # can be altered to help find a solution
                        for i in range(5):
                            # do not randomize the limb since the limb is already so close
                            # to the desired location
                            self.robot.setConfig(start)
                            if ik.solve(goal):
                                # check that the solution is collision-free
                                # ignore the grasped object
                                if self.planner.check_limb_collision_free(self.active_limb, self.planner.get_limb_config(self.robot.getConfig(), self.active_limb), ignore=[ object ]):
                                    # get a path from the starting position to this solution
                                    # ignore the grasped object
                                    path = self.planner.plan(start, self.robot.getConfig(), ignore=[ object ])
                                    if path:
                                        return path
                            time.sleep(0.01)
                    t[0] += 0.01

                return None

            def hold_over_order_bin(start):
                t = se3.apply(order_bin_xform,[0,0,order_bin_bounds[1][2]])
                t[0] -= 0.1
                R = [0,0,-1, 0,1,0, 1,0,0]
                R = so3.mul(R, so3.rotation([0,0,1], pi/2))

                # move over the order bin center at the current gripper height
                # also point the gripper opposite of the limb to simply later
                # path planning
                self.robot.setConfig(start)
                if self.active_limb == 'left':
                    t[1] = self.left_gripper_link.getTransform()[1][1] + 0.2
                    t[2] = self.left_gripper_link.getTransform()[1][2] - 0.2
                    R = so3.mul(so3.rotation([0,0,1], -pi/2), R)

                    (Rg, tg) = se3.mul((R, t), se3.inv(left_gripper_center_xform))
                    goal = ik.objective(self.left_gripper_link,R=Rg,t=tg)
                else:
                    t[1] = self.right_gripper_link.getTransform()[1][1] - 0.2
                    t[2] = self.right_gripper_link.getTransform()[1][2] - 0.2
                    R = so3.mul(so3.rotation([0,0,1], pi/2), R)

                    (Rg, tg) = se3.mul((R, t), se3.inv(right_gripper_center_xform))
                    goal = ik.objective(self.right_gripper_link,R=Rg,t=tg)

                global debug_xform3
                debug_xform3 = (Rg, tg)

                for i in range(1000):
                    # set IK solver initial configuration
                    if i==0:
                        self.robot.setConfig(start)
                    else:
                        self.randomize_limb_position(self.active_limb)
                    if ik.solve(goal, tol=0.1):
                        # check that the solution is collision-free
                        # ignore the grasped object
                        if self.planner.check_limb_collision_free(self.active_limb, self.planner.get_limb_config(self.robot.getConfig(), self.active_limb), ignore=[ object ]):
                            # get a path from the starting position to this solution
                            # ignore the grasped object
                            path = self.planner.plan(start, self.robot.getConfig(), ignore=[ object ])
                            if path:
                                return path
                    time.sleep(0.01)

                return None

            def place_in_order_bin(start):
                t = se3.apply(order_bin_xform,[0,0,order_bin_bounds[1][2]])
                R = [0,0,-1, 0,1,0, 1,0,0]
                R = so3.mul(R, so3.rotation([0,0,1], pi/2))

                # move the gripper to the order bin top pointed to the ground
                self.robot.setConfig(start)
                if self.active_limb == 'left':
                    t[1] += 0.10
                    R = so3.mul(so3.rotation([0,0,1], -pi/2), R)
                    R = so3.mul(so3.rotation([1,0,0], pi/2), R)

                    (Rg, tg) = se3.mul((R, t), se3.inv(left_gripper_center_xform))
                    goal = ik.objective(self.left_gripper_link,R=Rg,t=tg)
                else:
                    t[1] -= 0.10
                    R = so3.mul(so3.rotation([0,0,1], pi/2), R)
                    R = so3.mul(so3.rotation([1,0,0], -pi/2), R)

                    (Rg, tg) = se3.mul((R, t), se3.inv(right_gripper_center_xform))
                    goal = ik.objective(self.right_gripper_link,R=Rg,t=tg)

                global debug_xform4
                debug_xform4 = (Rg, tg)

                for i in range(1000):
                    # set IK solver initial configuration
                    if i==0:
                        self.robot.setConfig(start)
                    else:
                        self.randomize_limb_position(self.active_limb)
                    if ik.solve(goal, tol=0.1):
                        # check that the solution is collision-free
                        # ignore the grasped object
                        if self.planner.check_limb_collision_free(self.active_limb, self.planner.get_limb_config(self.robot.getConfig(), self.active_limb), ignore=[ object ]):
                            # get a path from the starting position to this solution
                            # ignore the grasped object
                            path = self.planner.plan(start, self.robot.getConfig(), ignore=[ object ])
                            if path:
                                return path
                    time.sleep(0.01)

            qcmd = self.controller.getCommandedConfig()
            path1 = lift_up_object(qcmd)
            if path1:
                path2 = withdraw_object(path1[-1])
                if path2:
                    path3 = hold_over_order_bin(path2[-1])
                    if path3:
                        path4 = place_in_order_bin(path3[-1])
                        if path4:
                            # send the path
                            for q in self.planner.resample_path(path1 + path2, pi/500) + self.planner.resample_path(path3 + path4, pi/50):
                                if not FAKE_SIMULATION:
                                    time.sleep(0.1)
                                self.controller.addMilestone(q)

                            return True

        return False


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
            elif c=='q':
                break
        else:
            print "Waiting for command..."
            time.sleep(0.1)
    print "Done"

class FakeLowLevelController:
    """A faked low-level interface to the Baxter robot (with parallel jaw
    grippers).  Does appropriate locking for multi-threaded use.
    Replace LowLevelController with this for prototyping, because you
    don't have to wait for motions to complete."""
    def __init__(self,robotModel,robotController):
        self.robotModel = robotModel
        self.config = robotModel.getConfig()
        self.lastCommandTime = time.time()
        self.lock = Lock()
    def getSensedConfig(self):
        self.lock.acquire()
        res = self.config
        self.lock.release()
        return res
    def getSensedVelocity(self):
        return [0.0]*len(self.config)
    def getCommandedConfig(self):
        self.lock.acquire()
        res = self.config
        self.lock.release()
        return res
    def getCommandedVelocity(self):
        return [0.0]*len(self.config)
    def setPIDCommand(self,configuration,velocity):
        """Sets the controller to a PID command mode"""
        self.lock.acquire()
        self.config = configuration[:]
        self.lastCommandTime = time.time()
        self.lock.release()
    def setMilestone(self,destination,endvelocity=None):
        """Immediately sets the motion queue to move to the given
        milestone.  If endvelocity is given, then the end of the
        queue will be moving at that velocity.  Otherwise, the end
        velocity will be zero."""
        self.lock.acquire()
        self.config = destination[:]
        self.lastCommandTime = time.time()
        self.lock.release()
    def addMilestone(self,destination,endvelocity=None):
        """Appends a milestone to the motion queue.  If endvelocity
        is given, then the end of the queue will be moving at that velocity.
        Otherwise, the end velocity will be zero."""
        self.lock.acquire()
        self.config = destination[:]
        self.lastCommandTime = time.time()
        self.lock.release()
    def isMoving(self):
        return self.remainingTime() > 0
    def remainingTime(self):
        return (self.lastCommandTime + 0.1) - time.time()
    def commandGripper(self,limb,command):
        """Sends the command to the indicated gripper.
        For the parallel-jaw gripper, [0] is closed, [1] is open
        Warning: don't do this while moving"""
        self.lock.acquire()
        self.robotModel.setConfig(self.config)
        set_model_gripper_command(self.robotModel,limb,command)
        self.config = self.robotModel.getConfig()
        self.lastCommandTime = time.time()
        self.lock.release()


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
        #self.dt = 1e-3
        self.simulate = True
        #self.sim.simulate(0)

        #you can set these to true to draw the bins, grasps, and/or gripper/camera frames
        self.draw_bins = False
        self.draw_grasps = False
        self.draw_gripper_and_camera = False

        #initialize controllers
        self.low_level_controller = LowLevelController(simworld.robot(0),self.sim.getController(0))
        if FAKE_SIMULATION:
            self.low_level_controller = FakeLowLevelController(simworld.robot(0),self.sim.getController(0))
        else:
            self.low_level_controller = LowLevelController(simworld.robot(0),self.sim.getController(0))
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

        gldraw.xform_widget(se3.identity(), 0.5, 0.05)
        if debug_xform:
            gldraw.xform_widget(debug_xform, 0.1, 0.01)
        if debug_xform2:
            gldraw.xform_widget(debug_xform2, 0.1, 0.01)
        if debug_xform3:
            gldraw.xform_widget(debug_xform3, 0.1, 0.01)
        if debug_xform4:
            gldraw.xform_widget(debug_xform4, 0.1, 0.01)

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
                        gldraw.xform_widget(xform,0.05,0.005)

        #show gripper and camera frames
        if self.draw_gripper_and_camera:
            left_camera_link = self.simworld.robot(0).getLink(left_camera_link_name)
            right_camera_link = self.simworld.robot(0).getLink(right_camera_link_name)
            left_gripper_link = self.simworld.robot(0).getLink(left_gripper_link_name)
            right_gripper_link = self.simworld.robot(0).getLink(right_gripper_link_name)
            gldraw.xform_widget(left_camera_link.getTransform(),0.1,0.01)
            gldraw.xform_widget(right_camera_link.getTransform(),0.1,0.01)
            gldraw.xform_widget(se3.mul(left_gripper_link.getTransform(),left_gripper_center_xform),0.05,0.005)
            gldraw.xform_widget(se3.mul(right_gripper_link.getTransform(),right_gripper_center_xform),0.05,0.005)

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
    world.loadElement(os.path.join(model_dir,"baxter_with_parallel_gripper_col.rob"))
    print "Loading Kiva pod model..."
    world.loadElement(os.path.join(model_dir,"kiva_pod/meshes/pod_lowres.stl"))
    print "Loading plane model..."
    world.loadElement(os.path.join(model_dir,"plane.env"))

    #shift the Baxter up a bit (95cm)
    Rbase,tbase = world.robot(0).getLink(0).getParentTransform()
    world.robot(0).getLink(0).setParentTransform(Rbase,(0,0,0.95))
    world.robot(0).setConfig(world.robot(0).getConfig())

    #translate pod to be in front of the robot, and rotate the pod by 90 degrees
    reorient = ([1,0,0,0,0,1,0,-1,0],[0,0,0.01])
    Trel = (so3.rotation((0,0,1),-math.pi/2),[1.2,0,0])
    T = reorient
    world.terrain(0).geometry().transform(*se3.mul(Trel,T))

    #initialize the shelf xform for the visualizer and object
    #xform initialization
    global ground_truth_shelf_xform
    ground_truth_shelf_xform = se3.mul(Trel,T)
    return world

def load_baxter_only_world():
    """Produces a world with only the Baxter in it."""
    world = WorldModel()
    print "Loading simplified Baxter model..."
    world.loadElement(os.path.join(model_dir,"baxter_with_parallel_gripper_col.rob"))
    #shift the Baxter up a bit (95cm)
    Rbase,tbase = world.robot(0).getLink(0).getParentTransform()
    world.robot(0).getLink(0).setParentTransform(Rbase,(0,0,0.95))
    world.robot(0).setConfig(world.robot(0).getConfig())
    return world

def spawn_objects_from_ground_truth(world):
    """For all ground_truth_items, spawns RigidObjects in the world
    according to their sizes / mass properties"""
    global ground_truth_items
    print "Initializing world objects"
    for item in ground_truth_items:
        obj = world.makeRigidObject(item.info.name)
        bmin,bmax = item.info.bmin,item.info.bmax
        center = vectorops.div(vectorops.add(bmin,bmax),2.0)
        m = obj.getMass()
        m.setMass(item.info.mass)
        m.setCom([0,0,0])
        m.setInertia(vectorops.mul([bmax[0]-bmin[0],bmax[1]-bmin[1],bmax[2]-bmin[2]],item.info.mass/12.0))
        obj.setMass(m)
        c = obj.getContactParameters()
        c.kFriction = 0.6
        c.kRestitution = 0.1;
        c.kStiffness = 100000
        c.kDamping = 100000
        obj.setContactParameters(c)
        cube = obj.geometry()
        if not cube.loadFile(os.path.join(model_dir,"cube.tri")):
            print "Error loading cube file",os.path.join(model_dir,"cube.tri")
            exit(1)
        scale = [bmax[0]-bmin[0],0,0,0,bmax[1]-bmin[1],0,0,0,bmax[2]-bmin[2]]
        translate = vectorops.sub(bmin,center)
        cube.transform(scale,translate)
        mesh = cube.getTriangleMesh()
        obj.setTransform(item.xform[0],item.xform[1])
    return

def main():
    """The main loop that loads the planning / simulation models and
    starts the OpenGL visualizer."""
    world = load_apc_world()

    init_ground_truth()

    if NO_SIMULATION_COLLISIONS:
        simworld = load_baxter_only_world()
    else:
        simworld = load_apc_world()
        spawn_objects_from_ground_truth(simworld)

    #load the resting configuration from klampt_models/baxter_rest.config
    global baxter_rest_config
    f = open(model_dir+'baxter_with_parallel_gripper_rest.config','r')
    baxter_rest_config = loader.readVector(f.readline())
    f.close()
    simworld.robot(0).setConfig(baxter_rest_config)


    #run the visualizer
    visualizer = MyGLViewer(simworld,world)
    visualizer.run()

if __name__ == "__main__":
    main()

###############################################################
# WRITTEN ANSWERS / COMMENTS:
# Q1.

# 1) unnecessary collisions with and self and shelf

# The path planner frequently flails the limb of interest in space.  The path
# frequently collides with the robot and the shelf.  This is unnecessary because
# occasionally the planner chooses a path without collision, proving the existence
# of sufficient obstacle-free C-space for a collision free path.  The solution to
# this problem is to introduce obstacles into the planner's view of C-space.  This
# is equivalent to providing a planner with a subset of all C-space such that no
# configurations lie in collision with an obstacle.

# 2) many candidate grasps for cuboid objects become unavailable when the object
#    rests on a shelf

# The cuboid object models used for this homework have a great many grasp positions
# in which the parallel grippers contact parallel faces of the object.  In practice,
# a subset of these candidate grasps are accessible in an obstacle-free way.
# Specifically, grasps involving the bottom face (and thus the top face) become
# inaccessible when the object lays on the bin floor.  Moreover, grasps which do not
# permit sufficient space to maneuver the gripper jaw beside the object without
# colliding with the shelf become inaccessible.  Finally, grasps with impossible or
# colliding arm poses become inaccessible because the gripper cannot be safely
# positionied to grasp the object.  The most efficient solution here is to take
# a subset of possible grasps using the objects know configuration from the
# perception module.  Top and bottom face grasps will be excluded.  Grasps in close
# proximity to the shelf will be excluded.  To address the arm pose constraint,
# which is much more complex to precompute, it will suffice to repeatedly plan
# grasps at the remaining grasp sites until a collision-free and kinematically
# possible grasp is found.

# 3) trajectory to move object to order bin passses directly through the shelf

# Presently, when moving an object to the order bin, the path planner ignores the
# shelf and thus drives the robot arm directly through it.  A potential solution
# is to decompose the path into two stages.  The first stage plans a withdrawal
# path that frees the object from the shelf.  The second stage plans a path from
# this withdrawn position into the order bin.  For the second stage, the collosion
# model for the shelf can be approxiated as a cuboid rather than a complex model.

# 4) path planner output is influenced by small changes in initial conditions

# The path planner may choose elaborate paths to reposition the gripper from one
# position to another very close position.  This can be seen by repeatedly
# commanding the robot to sense the same bin.  Sometimes the gripper will not move.
# Other times, the gripper will undertake a compelx motion, only to arrive at its
# starting configuration.  The best method for working around this is increasing
# the goal tolerance for inverse kinematic solutions, especially when the viewing
# angle for a bin need not be as precise as a grasp.

#
# Q2.
#

# Collision-free IK solutions are obtained by rejecting all solutions that cause
# a self-collision.  In the case of a rejection, a new IK solution is sought after
# randomizing the limbs.  Immediately after an IK solution is produced, checking
# for self-collisions is performed.  This checks all active arm link geometry,
# including the gripper, against other roboot links that are marked self-collidable.

# This method works very well.  Experimentation shows that no self-collisions are
# permitted.  It seems that very few IK solutions actually have self-collisions in
# the first place.  Brief testing revealed 1-2 self-collisions in a test of ~20 IK
# solutions.  In another round of testing, 5-6 self-collisions were observed
# before a self-collision-free IK solution was found.  Thus, it seems the rate of
# self-collisions can very greatly, which is to be expected given the number of
# potential IK solutions and their initial conditions.

# The factor with the greatest influence on the number of colliding IK solutions
# is the physical contraints around the goal.  Camera vantage positions directly
# in front of the robot cannot permit many arm configurations so more of the IK
# solutions bring the robot into self-collision.  For example, viewing bin A will
# produce a colliding solution about 1 in 10 runs whereas viewing bin B will
# produce such a solution about 9 in 10 runs.  This also varies by the arm the
# IK solver randomly chooses to select when solving for motions.

#
# Q3.
#
#
# Q4.
#

# The configuration space for grasping is typically very narrow since the robot
# manipulator must approach obstacles, such as the obejct to be grasped, very
# closely.  Sampling-based planners do not handle narrow configuration spaces
# well since they are less likely to sample points along narrow regions due to
# that region's small volume.  This, sampling-based planners are unlikely to
# produce good grasping paths because they poorly explore areas of configuration
# space that will yield good grasps.

# Grasp reliability varies by object and is subject to some simulation quirks.
# The tall and medium objects are grasped most reliably provided the
# simulation issues described below do not come into play.  The tall object is
# grasped at ~90% reliability unless the robot lurches.  The medium object is
# grasped at ~75% reliability because the robot arm sometimes fails to track
# the commanded configuration and scrapes the shelf.  The small object is
# grasped at ~50% reliability since sometimes the planner cannot find a grasp
# path without shifting the grasp center up vertically so much so that the
# object is missed.

# First, short objects are most difficult to grasph since the planner must
# must position the parallel gripper jaws around the object wihtout contacting
# the ground.  This requires shifting the grasp centers vertically to avoid
# collision.  In some cases, depending upon the planners sampling, the grasp
# center is shifted too high such that the grasp fails.  Unfortunately, this
# happens frequently enough that reliability is only moderate.  This issue
# can be improved by increasing the planner runtime such that it is more likely
# to find successful plans without shifting the grasp center up.

# Second, the simulated robot does not always perfectly track the commanded
# position.  This becomes problematic for grasps that bring the arm or
# gripper close to the shelf.  A small deviation from the commanded
# configuration results in a collision with the shelf and the robot becomes
# stuck.  This can be addressed by inflating the obstacles or tuning the
# robot controller gains.  This issue is particularly apparent when liting
# objects, which weight down the arm, causing it to sag, and collide with
# the shelf.

# Thrid, some aspect of the simulation sometimes (seeminly randomly) causes
# the robot to lurch, almost as if the simulation is missing update cycles.
# This completely throws off grasps, especially when the robot overshoots
# into the object when trying to rejoin the commanded configuration.  This
# issue is minimized by require that the robot make very small motions,
# adding a delay between sending motion milestones, and pausing between
# components of a trajectory to ensure the robot arm settles before a
# delicate operation (e.g., a grasp) is attempted.

#
# Q5 (bonus question).
#

# The changes to improve object removal consist of a set of waypoints for
# removing objects from bins.  First, the object is lifted up and moved to
# the horizontal center of the bin.  Second, the object is withdrawn from the
# bin a distance based on its depth that will ensure it exits then bin. Then,
# two waypoints are used to bring the object safely to the order bin, hopefully,
# minimizing collisions.  This waypoint method was easier that modifying the
# planner to incorporate the held object into its planning algorithm.  The
# tradeoff is that sometimes the object scrapes the shelf when the planner
# produces a non-intuitive path.
