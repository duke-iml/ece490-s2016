#!/usr/bin/python

# NOTE: Key sequence to control the baxter as desired:
#       bin number (A~L) --> scoop (S) --> move spatula to center (N) --> move gripper to center (M)
#       --> grasp object (X) --> place object in order bin (P)
#
#       unscoop (Y) to place object in spatula back in shelf
#       ungrasp (U) to place object in gripper back in spatula


# TODO: 1) Spatula doesn't push the object. It seems like the spatula gets stuck (?)
#          before reaching the object. Sometimes this doesn't happen the first time the
#          spatula is actuated.
#
#       2) Need to turn off collision detection during tilt_wrist because the planner
#          thinks the configuration is infeasible (collision checks). Is this collision
#          check with the object or the shelf? Does the collision checker check shelf model?
#
#       3) toggle between simulation modes during run (global config variable)
#
#       4) spawn shelf objects a little behind (gets in the spatula during tilt_wrist)
#
#       5) Fixing end-effector orientation throughout trajectory?
#
#       6) fix 2 points of spatula edge to shelf during wrist tilt

from klampt import robotsim
from klampt.glprogram import *
from klampt import vectorops, se3, so3, loader, gldraw, ik
from klampt.robotsim import Geometry3D
from klampt import visualization
from baxter import *
from motionPlanner import *
import apc
import os
import math
import random
import copy
from threading import Thread,Lock
from Queue import Queue
from operator import itemgetter

# configuration variables
config = 1
NO_SIMULATION_COLLISIONS = config
FAKE_SIMULATION = config
SKIP_PATH_PLANNING = config
RECORD_TRAJECTORY = 0

# The path of the klampt_models directory
model_dir = "klampt_models/"

# global variable for baxter's restinag configuration
global baxter_rest_config

# The transformation of the order bin
order_bin_xform = (so3.identity(),[.65,-0.85,0])
order_bin_bounds = ([-0.2,-0.4,0],[0.2,0.4,0.7])

# Order list. Can be parsed from JSON input
global orderList
orderList = ['tall_item', 'med_item',  'tall_item',
             'med_item',  'tall_item', 'med_item',
             'tall_item', 'med_item',  'tall_item',
             'med_item',  'tall_item', 'med_item']

# Declare the knowledge base
global knowledge

def init_ground_truth():
    global ground_truth_items
    ground_truth_items = [
                          # apc.ItemInBin(apc.tall_item,'bin_A'),
                          apc.ItemInBin(apc.med_item,'bin_B'),
                          apc.ItemInBin(apc.tall_item,'bin_C'),
                          apc.ItemInBin(apc.med_item,'bin_D'),
                          apc.ItemInBin(apc.tall_item,'bin_E'),
                          apc.ItemInBin(apc.med_item,'bin_F'),
                          apc.ItemInBin(apc.tall_item,'bin_G'),
                          apc.ItemInBin(apc.med_item,'bin_H'),
                          apc.ItemInBin(apc.tall_item,'bin_I'),
                          apc.ItemInBin(apc.med_item,'bin_J'),
                          apc.ItemInBin(apc.tall_item,'bin_K'),
                          apc.ItemInBin(apc.med_item,'bin_L')]
    for i in range(len(ground_truth_items)):
        ux = random.uniform(0.3,0.6)
        uy = random.uniform(0.1,0.3)
        if ground_truth_items[i].info.name == 'med_item':
            # theta = random.uniform(math.pi/4, 3*math.pi/4)
            theta = math.pi/2
        else:
            theta = random.uniform(-math.pi/6, math.pi/6)
        ground_truth_items[i].set_in_bin_xform(ground_truth_shelf_xform, ux, uy, theta)
    for item in ground_truth_items:
        item.info.geometry = load_item_geometry(item)

def load_item_geometry(item,geometry_ptr = None):
    """Loads the geometry of the given item and returns it.  If geometry_ptr
    is provided, then it is assumed to be a Geometry3D object and the object
    geometry is loaded into it."""
    if geometry_ptr == None:
        geometry_ptr = Geometry3D()
    if item.info.geometryFile == None:
        return None
    elif item.info.geometryFile == 'box':
        fn = model_dir + "cube.tri"
        if not geometry_ptr.loadFile(fn):
            print "Error loading cube file",fn
            exit(1)
        bmin,bmax = item.info.bmin,item.info.bmax
        center = vectorops.mul(vectorops.add(bmin,bmax),0.5)
        scale = [bmax[0]-bmin[0],0,0,0,bmax[1]-bmin[1],0,0,0,bmax[2]-bmin[2]]
        translate = vectorops.sub(bmin,center)

        geometry_ptr.transform(scale,translate)
        geometry_ptr.setCurrentTransform(item.xform[0],item.xform[1])
        return geometry_ptr
    else:
        if not geometry_ptr.loadFile(item.info.geometryFile):
            print "Error loading geometry file",item.info.geometryFile
            exit(1)
        return geometry_ptr

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

        # item = apc.ItemInBin(apc.shelf, 'bin_SHELF')
        # item.set_in_bin_xform(knowledge.shelf_xform, 0,0,0)
        # item.info.geometry = load_item_geometry(item)
        # self.bin_contents['bin_SHELF'] = []
        # self.bin_contents['bin_SHELF'].append(item)
        self.shelf = []
        self.center_point = None

    def bin_front_center(self,bin_name):
        bmin,bmax = apc.bin_bounds[bin_name]
        # local_center = [(bmin[0]+bmax[0])*0.5, (bmin[1]+bmax[1])*0.5, bmax[2]]
        local_center = [(bmin[0]+bmax[0])*0.5, bmin[1], bmax[2]]

        if bin_name == 'bin_A' or bin_name == 'bin_D' or bin_name == 'bin_G' or bin_name == 'bin_J':
            local_center = vectorops.add(local_center, [0.01,0,0])
        elif bin_name == 'bin_C' or bin_name == 'bin_F' or bin_name == 'bin_I' or bin_name == 'bin_L':
            local_center = vectorops.add(local_center, [-0.02,0,0])

        world_center = se3.apply(knowledge.shelf_xform, local_center)
        return world_center

    def bin_vantage_point(self,bin_name):
        world_center = self.bin_front_center(bin_name)
        # Vantage point has 20cm offset from bin center
        # world_offset = so3.apply(knowledge.shelf_xform[0],[0,-0.1,0.35])
        world_offset = so3.apply(knowledge.shelf_xform[0],[0,0.03,0.55])

        return vectorops.add(world_center,world_offset)

    def bin_center_point(self):
        # bin_name = 'bin_L'
        # world_center = self.bin_front_center(bin_name)
        # # Vantage point has 20cm offset from bin center
        # # world_offset = so3.apply(knowledge.shelf_xform[0],[0,-0.1,0.35])
        # world_offset = so3.apply(knowledge.shelf_xform[0],[0,0,0.5])

        # return vectorops.add(world_center,world_offset)
        return self.center_point

    def bin_vertical_point(self):
        world_center = self.bin_center_point()
        # Vantage point has 20cm offset from bin center
        # world_offset = so3.apply(knowledge.shelf_xform[0],[0,-0.1,0.35])
        world_offset = so3.apply(knowledge.shelf_xform[0],[0,0.5,0])

        return vectorops.add(world_center,world_offset)

    def grasp_xforms(self,object):
        if object.xform == None: return None
        res = []
        for g in object.info.grasps:
            # NOTE: g.grasp_xform: the transformation of the gripper fingers
            #                      relative to the object's local frame
            #        object.xform: the transformation of the object center
            #                      relative to the world frame
            #   grasp_xform_world: the transformation of the gripper fingers
            #                      relative to the world frame
            grasp_xform_world = se3.mul(object.xform,g.grasp_xform)
            res.append((g,grasp_xform_world))
        return res

def run_perception_on_shelf(knowledge):
    """This is a fake perception module that simply reveals the shelf
    xform."""

    # knowledge.shelf_xform = input from perception team
    knowledge.shelf_xform = ground_truth_shelf_xform

def run_perception_on_bin(knowledge,bin_name):
    """This is a fake perception module that simply reveals all the items
    the given bin."""
    # if the dictionary "bin_contents" doesn't contain any values for the key "bin_name"
    if knowledge.bin_contents[bin_name]==None:
        # not sensed yet
        knowledge.bin_contents[bin_name] = []
        for item in ground_truth_items:
            if item.bin_name == bin_name:
                # add the item to the list of sensed items for the bin
                knowledge.bin_contents[bin_name].append(item)
    return

# TODO
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
    def appendMilestone(self,destination,endvelocity=None):
        """Appends a milestone to the motion queue.  If endvelocity
        is given, then the end of the queue will be moving at that velocity.
        Otherwise, the end velocity will be zero."""
        self.lock.acquire()
        if endvelocity == None: self.controller.addMilestoneLinear(destination)
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
        set_model_gripper_command(self.robotModel,limb,command)
        self.controller.setMilestone(self.robotModel.getConfig())
        self.lock.release()
# TODO
class FakeLowLevelController:
    """A faked low-level interface to the Baxter robot (with parallel jaw
    grippers).  Does appropriate locking for multi-threaded use.
    Useful for prototyping."""
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
    def appendMilestone(self,destination,endvelocity=None):
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
        self.planner = LimbPlanner(self.world, knowledge)

        # either 'ready' or 'holding'
        self.stateLeft = 'ready'
        self.stateRight = 'ready'
        self.active_limb = None
        self.active_grasp = None
        self.current_bin = None
        self.held_object = None

        #these may be helpful
        self.left_camera_link = self.robot.link(left_camera_link_name)
        self.right_camera_link = self.robot.link(right_camera_link_name)
        self.left_gripper_link = self.robot.link(left_gripper_link_name)
        self.right_gripper_link = self.robot.link(right_gripper_link_name)
        self.left_arm_links = [self.robot.link(i) for i in left_arm_link_names]
        self.right_arm_links = [self.robot.link(i) for i in right_arm_link_names]

        # define mapping from "link ID" to "link index"
        id_to_index = dict([(self.robot.link(i).getID(),i) for i in range(self.robot.numLinks())])

        # define a list of link indices on both arms
        # self.left_arm_indices = [id_to_index[i.getID()] for i in self.left_arm_links]
        # self.right_arm_indices = [id_to_index[i.getID()] for i in self.right_arm_links]
        self.left_arm_indices = left_arm_geometry_indices
        self.right_arm_indices = right_arm_geometry_indices

        # frames to draw
        self.frames = []

    def waitForMove(self,timeout = None, pollRate = 0.01):
        """Waits for the move to complete, or timeout seconds is elapsed,
        before terminating."""
        if timeout == None:
            timeout = 300
        iters = 0
        t = 0
        # print "Waiting for move to complete",
        while self.controller.isMoving(): # remaining time > 0
            # if iters % 10 == 0:
            #     print ".",
            time.sleep(pollRate)
            t += pollRate
            if timeout != None and t > timeout:
                print "Timed Out!"
                return False
            iters += 1
        # print "--> done\n"
        return True

    def viewBinAction(self,b):
        self.waitForMove()

        if self.stateLeft != 'ready':
            print "Already holding an object, can't move to bin"
            return False
        else:
            # If a valid bin name
            if b in apc.bin_names:
                # print "Valid bin (", b, ")"

                if self.move_camera_to_bin(b):
                    self.waitForMove()
                    self.current_bin = b
                    run_perception_on_bin(knowledge, b)
                    print "Sensed bin", b, "with camera", self.active_limb
                    return True
                else:
                    print "but move to bin",b,"failed"
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
        elif self.stateRight != 'ready':
            print "Already holding an object, can't grasp another"
            return False
        elif self.stateLeft != 'holding':
            print "No object in spatula"
            return False
        else:
            # if self.move_gripper_to_center():
            #     self.waitForMove()

            # now close the gripper
            self.controller.commandGripper(self.active_limb, [1])
            self.waitForMove()

            if self.move_to_grasp_object(self.held_object):
                self.waitForMove()

                # now close the gripper
                self.controller.commandGripper(self.active_limb, [0])
                self.waitForMove()

                # self.held_object = knowledge.bin_contents[self.current_bin].pop()
                self.stateRight = 'holding'

                print "Holding object",self.held_object.info.name,"in hand",self.active_limb
                return True
            else:
                print "Grasp failed"
                return False

    def scoopAction(self):
        self.waitForMove()

        if self.current_bin == None:
            print "Not located at a bin"
            return False
        elif self.stateLeft != 'ready':
            print "Already holding an object, can't grasp another"
            return False
        elif len(knowledge.bin_contents[self.current_bin])==0:
            print "The current bin is empty"
            return False
        else:
            if self.tilt_wrist('down'):
                self.waitForMove()
                # push spatula base
                # self.controller.commandGripper('left', [1])
                # self.waitForMove()

                # tilt wrist up
                self.tilt_wrist('up')
                self.waitForMove()

                # pull spatula base
                # self.controller.commandGripper('left', [0])
                # self.waitForMove()

                self.move_camera_to_bin(self.current_bin)
                self.waitForMove()


                self.held_object = knowledge.bin_contents[self.current_bin].pop()

                self.held_object.randRt = [ so3.rotation([0,1,0], random.uniform(0,math.pi/2)),
                                            [random.uniform(-0.07,0.07) , 0.005+(self.held_object.info.bmax[1]-self.held_object.info.bmin[1])/2 , random.uniform(-0.05, -0.25)]]

                self.stateLeft = 'holding'
                print "Holding object",self.held_object.info.name,"in spatula"
                return True
            else:
                print "Grasp failed"
                return False

    def unscoopAction(self):
        self.waitForMove()

        if self.current_bin == None:
            print "Not located at a bin"
            return False
        if self.stateLeft != 'holding':
            print "Not holding an object"
            return False
        else:
            if self.tilt_wrist('up'):
                self.waitForMove()

                # push spatula base
                # self.controller.commandGripper('left', [1])
                # self.waitForMove()

                # tilt wrist up
                self.tilt_wrist('down')
                self.waitForMove()

                # pull spatula base
                # self.controller.commandGripper('left', [0])
                # self.waitForMove()

                self.move_camera_to_bin(self.current_bin)
                self.waitForMove()


                print "Object",self.held_object.info.name,"placed back in bin"
                knowledge.bin_contents[self.current_bin].append(self.held_object)
                self.stateLeft = 'ready'
                self.held_object = None
                return True
            else:
                print "Grasp failed"
                return False

    def tilt_wrist(self,direction):
        """Tilt the robot's wrist before and after actuating the spatula
        so that the objects are picked up.
        """
        self.waitForMove()
        self.world.terrain(0).geometry().setCollisionMargin(0)
        bin_name = self.current_bin
        world_center = knowledge.bin_front_center(bin_name)

        # Setup ik objectives for both arms
        # place +z in the +x axis, -y in the +z axis, and x in the -y axis
        # left_goal = ik.objective(self.left_camera_link,R=R_camera,t=t_camera)
        left_goal = []

        # tilted angle view for spatula
        if direction == 'down':
            R_camera = so3.mul(so3.rotation([0,0,1], -math.pi/2),so3.rotation([1,0,0], -math.pi/2 - math.pi/360*10))
            # [(+Right/-Left), (+Up/-Down), (+In/-Out)
            world_offset = so3.apply( knowledge.shelf_xform[0],[0,0.0275,0.3325])
            t_camera = vectorops.add(world_center,world_offset)
            left_goal.append(ik.objective(self.left_camera_link,R=R_camera,t=t_camera))

            world_offset = so3.apply( knowledge.shelf_xform[0],[0,0.028,0.3325])
            t_camera = vectorops.add(world_center,world_offset)
            left_goal.append(ik.objective(self.left_camera_link,R=R_camera,t=t_camera))

            world_offset = so3.apply( knowledge.shelf_xform[0],[0,0.0285,0.3325])
            t_camera = vectorops.add(world_center,world_offset)
            left_goal.append(ik.objective(self.left_camera_link,R=R_camera,t=t_camera))

        elif direction == 'up':
            R_camera = so3.mul(so3.rotation([0,0,1], -math.pi/2),so3.rotation([1,0,0], -math.pi/2 + math.pi/360*10))
            # world_offset = so3.apply( knowledge.shelf_xform[0],[0,-0.02,0.3335])
            world_offset = so3.apply( knowledge.shelf_xform[0],[0,-0.01,0.4])
            t_camera = vectorops.add(world_center,world_offset)
            left_goal.append(ik.objective(self.left_camera_link,R=R_camera,t=t_camera))

        # debuggin code
        # self.frames.append([R_camera,t_camera])



        qcmd = self.controller.getCommandedConfig()
        # qcmd = self.controller.getSensedConfig()
        limbs = ['left']

        print "Solving for TILT_WRIST (", direction,")"

        for i in range(500):
            sortedSolutions = self.get_ik_solutions([left_goal], limbs, qcmd, maxResults=100, maxIters=100)

            if len(sortedSolutions)==0:
                continue

            # prototyping hack: move straight to target
            if SKIP_PATH_PLANNING:
                self.controller.setMilestone(sortedSolutions[0][0])
                self.active_limb = limbs[sortedSolutions[0][1]]
                return True

            # else, if we want to path plan
            numSol = 0
            for solution in sortedSolutions:
                numSol += 1
                print numSol, "solutions planned out of", len(sortedSolutions)
                path = self.planner.plan(qcmd,solution[0],'left')
                if path == 1 or path == 2 or path == False:
                    break
                elif path != None:
                    self.sendPath(path)
                    self.active_limb = limbs[solution[1]]
                    return True
        print "Failed to plan path"
        return False

    def spatula(self):
        self.waitForMove()

        self.controller.commandGripper('left', [1])
        self.waitForMove()

        self.controller.commandGripper('left', [0])
        self.waitForMove()

    def move_spatula_to_center(self):
        """Tilt the robot's wrist before and after actuating the spatula
        so that the objects are picked up.
        """
        self.waitForMove()

        R_wrist = so3.mul( so3.rotation([0,0,1], -math.pi), so3.rotation([1,0,0], -math.pi/2) )
        t_wrist = knowledge.bin_center_point()

        # Setup ik objectives for both arms
        # place +z in the +x axis, -y in the +z axis, and x in the -y axis
        left_goal = ik.objective(self.left_camera_link,R=R_wrist,t=t_wrist)

        qcmd = self.controller.getCommandedConfig()
        # qcmd = self.controller.getSensedConfig()
        limbs = ['left']

        print "\nSolving for MOVE_SPATULA_TO_CENTER (Left Hand)"

        for i in range(100):
            sortedSolutions = self.get_ik_solutions([left_goal], limbs, qcmd, maxResults=100, maxIters=100)

            if len(sortedSolutions)==0:
                continue

            # prototyping hack: move straight to target
            if SKIP_PATH_PLANNING:
                self.controller.setMilestone(sortedSolutions[0][0])
                self.active_limb = limbs[sortedSolutions[0][1]]
                return True

            # else, if we want to path plan
            else:
                numSol = 0
                for solution in sortedSolutions:
                    numSol += 1
                    print numSol, "solutions planned out of", len(sortedSolutions)
                    path = self.planner.plan(qcmd,solution[0],'left')
                    if path == 1 or path == 2 or path == False:
                        break
                    elif path != None:
                        self.sendPath(path)
                        self.active_limb = limbs[solution[1]]
                        return True
        print "Failed to plan path"
        return False

    def move_gripper_to_center(self):
        self.waitForMove()

        if self.stateLeft == 'holding':
            R_wrist = so3.mul(so3.rotation([1,0,0], math.pi), so3.rotation([0,0,1], math.pi))
            t_wrist = knowledge.bin_vertical_point()

            # Setup ik objectives for both arms
            # place +z in the +x axis, -y in the +z axis, and x in the -y axis
            right_goal = ik.objective(self.right_gripper_link,R=R_wrist,t=t_wrist)

            qcmd = self.controller.getCommandedConfig()
            # qcmd = self.controller.getSensedConfig()
            limbs = ['right']

            print "\nSolving for MOVE_GRIPPER_TO_CENTER (Right Hand)"
            for i in range(100):
                sortedSolutions = self.get_ik_solutions([right_goal], limbs, qcmd, maxResults=100, maxIters=100)

                if len(sortedSolutions)==0:
                    continue

                # prototyping hack: move straight to target
                if SKIP_PATH_PLANNING:
                    self.controller.setMilestone(sortedSolutions[0][0])
                    self.active_limb = limbs[sortedSolutions[0][1]]
                    return True

                # else, if we want to path plan
                else:
                    numSol = 0
                    for solution in sortedSolutions:
                        numSol += 1
                        print numSol, "solutions planned out of", len(sortedSolutions)
                        path = self.planner.plan(qcmd,solution[0],'right')
                        if path == 1 or path == 2 or path == False:
                            break
                        elif path != None:
                            self.sendPath(path)
                            self.active_limb = limbs[solution[1]]
                            return True
            print "Failed to plan path"
            return False
        else:
            print "no item in spatula"
            return False

    def placeInOrderBinAction(self):
        self.waitForMove()

        if self.stateRight != 'holding':
            print "Not holding an object"
        else:
            if self.move_to_order_bin(self.held_object):
                self.waitForMove()

                # now open the gripper
                self.controller.commandGripper(self.active_limb,[1])
                self.waitForMove()

                knowledge.order_bin_contents.append(self.held_object)
                self.active_limb = 'left'
                self.active_grasp = None
                self.held_object.xform = None
                self.held_object.bin_name = 'order_bin'
                self.drop_in_order_bin(self.held_object)
                self.stateRight = 'ready'
                self.stateLeft = 'ready'
                print "Successfully placed",self.held_object.info.name,"into order bin"

                self.held_object = None
                return True
            else:
                print "Move to order bin failed"
                return False

    def drop_in_order_bin(self,object):
        R, t = order_bin_xform
        tRandom = [random.uniform(-0.05,0.1),random.uniform(-0.4,.4),0]
        objHeight = object.info.bmax[2] - object.info.bmin[2]
        object.xform = (R, [t[0]+tRandom[0], t[1]+tRandom[1], t[2]+objHeight/2])
        if object.info.name in orderList:
            print "\nOrderList:", orderList
            print "Picked Item:", object.info.name, "\n"
            orderList.remove(object.info.name)
        else:
            print "\nOrderList:", orderList
            print "Wrongly Picked Item:", object.info.name, "\n"
        return True

    # TODO: change implementation to fit spatula/gripper combination
    def fulfillOrderAction(self,objectList):
        """Given a list of objects to be put in the order bin, run
        until completed."""
        # go through all bins
        for b in apc.bin_names:
            # if the bin is empty
            if knowledge.bin_contents[b]==None:
                # try to view the bin
                if not self.viewBinAction(b):
                    print "Could not view bin",b
                    continue

            doNextBin = False
            # if any of the objects in the bin is in the "remaining objets" list, and we want to keep searching in current bin
            while any(o.info.name in objectList for o in knowledge.bin_contents[b]) and not doNextBin:
                #pick up and put down objects until you are holding one that is in the objectList list

                # grasp failed
                if not self.graspAction():
                    print "Error grasping object"
                    doNextBin = True
                    break
                # if robot is not holding on to something, or the object it is holding is not in order
                while (self.held_object == None or self.held_object.info.name not in objectList) and not doNextBin:
                    # cycle through objects by putting down and picking up the next object
                    if not self.ungraspAction():
                        print "Error putting down object"
                        return False
                    if not self.graspAction():
                        print "Error grasping object"
                        doNextBin = True
                        break

                # temporarily store held object because placeInOrderBinAction sets held object to None
                obj = self.held_object
                if self.placeInOrderBinAction():
                    # don't want to remove obj from objList twice. It is already taken care of in drop_order_in_bin
                    # objectList.remove(obj.info.name)
                    print ""
                else:
                    print "Error putting object into order bin"
                    return False

            if len (objectList)==0:
                return True
        print "These items are remaining from the order:", objectList
        return False

    def randomize_limb_position(self,limb,center=None, range=None):
        """Helper: randomizes the limb configuration in self.robot.
        limb can be 'left' or 'right'.  If range is provided, then
        this samples in a range around the config center.  If center is not
        provided, it uses the current commanded config"""
        qmin,qmax = self.robot.getJointLimits()

        # Initialze non-active limb with current position, instead of
        # resting_config to keep its current position
        # if self.stateLeft == 'holding':
        #     q = self.controller.getCommandedConfig()
        # else:
        #     q = baxter_rest_config[:]
        q = self.controller.getCommandedConfig()

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

    def get_ik_solutions(self,goals,limbs,initialConfig=None,maxResults=10,maxIters=1000,tol=1e-3,validity_checker=None,printer=False):
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
        numSolutions = [0]*len(goals)
        numColFreeSolutions = [0]*len(goals)
        for i in range(maxIters):
            index = random.randint(0,len(goals)-1) # choose which ik goals to solve
            goal = goals[index]
            limb = limbs[index]
            numTrials[index] += 1
            # if first time trying the ik goal, initialize with current config
            if numTrials[index] == 1:
                self.robot.setConfig(initialConfig)
            # else, initialize with a random q, incrementally perturbing more from inital config
            else:
                self.randomize_limb_position(limb,center=initialConfig,range=0.005*(numTrials[index]-1))
                # self.randomize_limb_position(limb,center=initialConfig,range=None)

            if ik.solve(goal,tol=tol):
                numSolutions[index] += 1
                if validity_checker(limb):
                    numColFreeSolutions[index] += 1
                    ikSolutions.append((self.robot.getConfig(),index))
                    if len(ikSolutions) >= maxResults: break
        #     print i
        # return []
        if printer:
            print "< IK Summary >",
            for i in range(len(goals)):
                if numTrials != 0:
                    print "Goal: #", i, "; Attempted:", numTrials[i], "/", maxIters, "; Result:", numColFreeSolutions[i], "/", numSolutions[i], "col. free. solutions"
            print " "

        # sort solutions by distance to initial config
        sortedSolutions = []
        for solution in ikSolutions:
            # this line was buggy: the sortedSolutions only had one entry after the sort !!
            # sortedSolutions = sorted([(vectorops.distanceSquared(solution[0],initialConfig),solution) for solution in ikSolutions])
            dist = vectorops.distanceSquared(solution[0],initialConfig)
            config = solution[0]
            ind = solution[1]
            sortedSolutions.append( ((dist), (config, ind)) )


        sortedSolutions = sorted(sortedSolutions, key=itemgetter(0))

        # s[0] contains the distance-sqaured values
        # s[1] contains the ikSolution, which has [0]: config and [1]: index
        return [s[1] for s in sortedSolutions]

    def move_camera_to_bin(self,bin_name):
        """Starts a motion so the camera has a viewpoint that
        observes bin_name.  Will also change self.active_limb to the
        appropriate limb.

        If successful, sends the motion to the low-level controller and
        returns True.

        Otherwise, does not modify the low-level controller and returns False.
        """
        R_shelf = knowledge.shelf_xform[0]
        R_camera = so3.mul(R_shelf, so3.rotation([1,0,0], math.pi))
        t_camera = knowledge.bin_vantage_point(bin_name)

        # Setup ik objectives for both arms
        # place +z in the +x axis, -y in the +z axis, and x in the -y axis
        left_goal = ik.objective(self.left_camera_link,R=R_camera,t=t_camera)

        qcmd = self.controller.getCommandedConfig()
        # qcmd = self.controller.getSensedConfig()
        limbs = ['left']

        # temporarily increase collision margin of shelf
        self.world.terrain(0).geometry().setCollisionMargin(0.05)

        # # ClosedLoopRobotCSpace IK Constraint
        # # ik_constraint = ik.objective(self.robot.link(54), R=so3.identity(), t=[0,0,0])
        # print "****************"
        # ik_constraint = IKObjective()
        # ik_constraint.setLinks(55)
        # print ik_constraint.numRotDims()
        # ik_constraint.setAxialRotConstraint([0,-1,0], [0,0,1])
        # # ik_constraint.setAxialRotConstraint([1,0,0], [0,0,1])
        # # ik_constraint.setAxialRotConstraint([0,0,1], [0,0,1])
        # # ik_constraint.setAxialRotConstraint([0,-1,0], [0,1,0])
        # # ik_constraint.setAxialRotConstraint([1,0,0], [0,1,0])
        # # ik_constraint.setAxialRotConstraint([0,0,1], [0,1,0])
        # # ik_constraint.setAxialRotConstraint([0,-1,0], [1,0,0])
        # # ik_constraint.setAxialRotConstraint([1,0,0], [1,0,0])
        # # ik_constraint.setAxialRotConstraint([0,0,1], [1,0,0])
        # print ik_constraint.numRotDims()
        # print ik_constraint.getRotationAxis()
        # print ik_constraint.link()
        # print ik_constraint.destLink()
        # print "****************"

        print "\nSolving for MOVE_CAMERA_TO_BIN (", bin_name, ")"
        for i in range(100):
            sortedSolutions = self.get_ik_solutions([left_goal], limbs, qcmd, maxResults=100, maxIters=100)

            if len(sortedSolutions)==0:
                continue

            # prototyping hack: move straight to target
            if SKIP_PATH_PLANNING:
                self.controller.setMilestone(sortedSolutions[0][0])
                self.active_limb = limbs[sortedSolutions[0][1]]
                return True

            # else, if we want to path plan
            numSol = 0
            for solution in sortedSolutions:
                numSol += 1
                print numSol, "solutions planned out of", len(sortedSolutions)
                path = self.planner.plan(qcmd,solution[0],'left')
                # path = self.planner.plan(qcmd,solution[0], 'left', iks = ik_constraint)
                if path == 1 or path == 2 or path == False:
                    break
                elif path != None:
                    self.sendPath(path)
                    self.active_limb = limbs[solution[1]]
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

        """Tilt the robot's wrist before and after actuating the spatula
        so that the objects are picked up.
        """
        self.waitForMove()

        R_obj = object.randRt[0]
        t_obj = object.randRt[1]

        # object xform relative to world frame
        objxform = se3.mul( self.left_gripper_link.getTransform() ,
                            # object xform relative to gripper link =
                            # gripper center xform relative to gripper link (X) obj xform relative to gripper center
                            se3.mul(left_gripper_center_xform,
                                    # object xform relative to gripper center xform
                                    [so3.mul(R_obj, so3.rotation([1,0,0],math.pi/2)), t_obj] ))

        Rt = se3.mul(objxform ,se3.inv(right_gripper_center_xform))

        # Setup ik objectives for both arms
        right_goal = ik.objective(self.right_gripper_link,R=Rt[0],t=Rt[1])

        qcmd = self.controller.getCommandedConfig()
        # qcmd = self.controller.getSensedConfig()
        limbs = ['right']

        print "\nSolving for GRASP_OBJECT"
        for i in range(100):
            sortedSolutions = self.get_ik_solutions([right_goal], limbs, qcmd, maxResults=100, maxIters=100)

            if len(sortedSolutions)==0:
                continue

            # prototyping hack: move straight to target
            if SKIP_PATH_PLANNING:
                self.controller.setMilestone(sortedSolutions[0][0])
                self.active_limb = limbs[sortedSolutions[0][1]]
                return True

            # else, if we want to path plan
            else:
                numSol = 0
                for solution in sortedSolutions:
                    numSol += 1
                    print numSol, "solutions planned out of", len(sortedSolutions)
                    path = self.planner.plan(qcmd,solution[0],'right')
                    if path == 1 or path == 2 or path == False:
                        break
                    # elif path != None:
                    else:
                        self.sendPath(path)
                        self.active_limb = limbs[solution[1]]
                        return True
        print "Failed to plan path"
        return False

    def move_to_order_bin(self,object):
        """Sets the robot's configuration so the gripper is over the order bin

        If successful, sends the motion to the low-level controller and
        returns True.

        Otherwise, does not modify the low-level controller and returns False.
        """

        qcmd = self.controller.getCommandedConfig()
        # qcmd = self.controller.getSensedConfig()
        target = se3.apply(order_bin_xform,[0,0, order_bin_bounds[1][2]+0.1])

        R_obj = object.randRt[0]
        t_obj = object.randRt[1]

        # object xform relative to world frame
        objxform = se3.mul( self.left_gripper_link.getTransform() ,
                            # object xform relative to gripper link =
                            # gripper center xform relative to gripper link (X) obj xform relative to gripper center
                            se3.mul(left_gripper_center_xform,
                                    # object xform relative to gripper center xform
                                    [so3.mul(R_obj, so3.rotation([1,0,0],math.pi/2)), t_obj] ))

        Rt = se3.mul(objxform ,se3.inv(right_gripper_center_xform))

        placegoal = ik.objective(self.right_gripper_link,R=Rt[0],t=target)

        sortedSolutions = []

        print "solving MOVE_TO_ORDER_BIN ..."
        for i in range(100):
            sortedSolutions = self.get_ik_solutions([placegoal],['right'],qcmd,tol=1e-2, maxIters=100, maxResults = 100)

            if len(sortedSolutions)==0:
                continue

            # prototyping hack: move straight to target
            if SKIP_PATH_PLANNING:
                self.controller.setMilestone(sortedSolutions[0][0])
                self.active_limb = [sortedSolutions[0][1]]
                return True

            # else, if we want to path plan
            else:
                numSol = 0
                for solution in sortedSolutions:
                    numSol+=1
                    path = self.planner.plan(qcmd,solution[0],'right')
                    if path == 1 or path == 2 or path == False:
                        if path==1:
                            print "Invalid StartConfig = ", qcmd
                        break
                    elif path != None:
                        self.waitForMove()
                        self.sendPath(path)
                        return True
        print "Planning failed"
        return False

    def sendPath(self,path):
        q = path[0]
        qmin,qmax = self.robot.getJointLimits()

        q[55] = 0 # don't move spatula

        # removing AppendRamp claming error
        for i in [23,30,31,43,50,51,54,56,57]: q[i] = 0

        q = self.clampJointLimits(q,qmin,qmax)

        self.controller.setMilestone(path[0])

        for q in path[1:]:
            q[55] = 0 # don't move spatula
            for i in [23,30,31,43,50,51,54,56,57]:
                # print i, qmin[i], q[i], qmax[i]
                q[i] = 0
            q = self.clampJointLimits(q,qmin,qmax)
            self.controller.appendMilestone(q)

            if RECORD_TRAJECTORY:
                with open("configOutput.txt", "a") as myfile:
                    myfile.write(str(q))
                    myfile.write("\n")

    def clampJointLimits(self,q,qmin,qmax):
        for i in range(len(q)):
            if (q[i] < qmin[i]) :
                print "Joint #",i,"(",q[i],") out of limits (min:",qmin[i],")"
                print "Changed joint value to its minimum"
                q[i] = qmin[i]

            if (q[i] > qmax[i]) :
                print "Joint #",i,"(",q[i],") out of limits (max:",qmax[i],")"
                print "Changed joint value to its maximum"
                q[i] = qmax[i]
        return q

    def readFromTrajectoryFile(self, fileName):
        trajectoryToSend = []
        with open(fileName) as f:
            trajectory = f.readlines()
        for i in range(len(trajectory)):
            milestone = trajectory[i]
            milestone = milestone.translate(None, '[]')
            milestone = milestone.strip()
            milestone = milestone.split(', ')
            trajectoryToSend.append(map(float, milestone))

            if i==0:
                self.controller.setMilestone(trajectoryToSend[0])
            else:
                self.controller.appendMilestone(trajectoryToSend[i])

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

# this function is called on a thread
def run_controller(controller,command_queue):
    # simply reveals the shelf xform
    run_perception_on_shelf(knowledge)
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
                # controller.fulfillOrderAction(orderList)

                binList = ['A','B','C','D','E','F','G','H','I','J','K','L']
                for i in range(len(binList)):
                    controller.viewBinAction('bin_'+binList[i])
                    controller.scoopAction()
                    controller.move_spatula_to_center()
                    controller.move_gripper_to_center()
                    controller.graspAction()
                    controller.placeInOrderBinAction()


            elif c == 's':
                controller.scoopAction()
            elif c == 'y':
                controller.unscoopAction()
            elif c == 't':
                controller.spatula()
            elif c == 'n':
                controller.move_spatula_to_center()
            elif c == 'm':
                controller.move_gripper_to_center()
            elif c == 'r':
                var = raw_input("Please enter trajectory file name : ")
                print "Running recorded trajectory... (", var, ")"
                controller.readFromTrajectoryFile(var)
                # controller.readFromTrajectoryFile('testConfigOutput.txt')
            elif c == '`':
                global config
                config = (not config)
            elif c=='q':
                break
        else:
            print "Waiting for command..."
            time.sleep(0.1)
    print "Done"

def restart_program():
    """Restarts the current program.
    Note: this function does not return. Any cleanup action (like
    saving data) must be done before calling this function."""
    python = sys.executable
    os.execl(python, python, * sys.argv)

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
        # self.camera.dist = 3.0
        # # #x field of view in degrees
        # self.fov = 60
        # self.width *= 2
        # self.height *= 2

        self.simworld = simworld
        self.planworld = planworld
        self.sim = Simulator(simworld)

        self.simulate = True
        # self.sim.simulate(0)

        #you can set these to true to draw the bins, grasps, and/or gripper/camera frames
        self.draw_bins = False
        self.draw_grasps = True
        self.draw_gripper_and_camera = True
        self.drawVE = False
        self.drawPath = True

        # initialize controllers, and starts a thread running "run_controller" with the
        # specified picking controller and command queue
        if FAKE_SIMULATION:
            self.low_level_controller = FakeLowLevelController(simworld.robot(0),self.sim.controller(0))
        else:
            self.low_level_controller =     LowLevelController(simworld.robot(0),self.sim.controller(0))
        self.command_queue = Queue()
        # visualization.add("world",planworld)
        # visualization.dialog()
        self.picking_controller = PickingController(planworld,self.low_level_controller)
        self.picking_thread = Thread(target=run_controller,args=(self.picking_controller,self.command_queue))
        self.picking_thread.start()

    # where is this function called? somewhere in GLRealtimeProgram in glprogram
    def idle(self):
        if self.simulate:
            self.sim.simulate(self.dt)
            #for Q2
            if self.simworld.numRigidObjects() >= len(ground_truth_items):
                ofs = self.simworld.numRigidObjects()-len(ground_truth_items)
                for i,item in enumerate(ground_truth_items):
                    T = self.sim.body(self.simworld.rigidObject(ofs+i)).getTransform()
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
        # only 1 robot in this case, but still use for-loop for generality
        for i in xrange(self.simworld.numRobots()):
            r = self.simworld.robot(i)
            # q = self.sim.controller(i).getCommandedConfig()
            q = self.low_level_controller.getCommandedConfig()
            r.setConfig(q)
            r.drawGL(False)
        glDisable(GL_BLEND)

        #show bin boxes
        if self.draw_bins:
            glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,1,0,1])
            for b in apc.bin_bounds.values():
                draw_oriented_box(knowledge.shelf_xform,b[0],b[1])
            for b in apc.bin_names:
                c = knowledge.bin_front_center(b)
                if c:
                    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,1,0.5,1])
                    r = 0.01
                    gldraw.box([c[0]-r,c[1]-r,c[2]-r],[c[0]+r,c[1]+r,c[2]+r])
                c = knowledge.bin_vantage_point(b)
                if c:
                    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0.5,1,0.5,1])
                    r = 0.01
                    gldraw.box([c[0]-r,c[1]-r,c[2]-r],[c[0]+r,c[1]+r,c[2]+r])
            c = knowledge.bin_center_point()
            if c:
                glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,0.5,0.5,1])
                r = 0.01
                gldraw.box([c[0]-r,c[1]-r,c[2]-r],[c[0]+r,c[1]+r,c[2]+r])
            c = knowledge.bin_vertical_point()
            if c:
                glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0.5,0.5,0.5,1])
                r = 0.01
                gldraw.box([c[0]-r,c[1]-r,c[2]-r],[c[0]+r,c[1]+r,c[2]+r])

        #show object state
        for i in ground_truth_items:
            if i.xform == None:
                continue

            # if i.bin_name == 'order_bin':
            #     continue

            if i.bin_name == 'order_bin':
            # draw in wireframe
                glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,1,1,1])
                draw_oriented_box(i.xform,i.info.bmin,i.info.bmax)
                continue

            #if perceived, draw in solid color
            if knowledge.bin_contents[i.bin_name]!=None and i in knowledge.bin_contents[i.bin_name]:
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
                g = knowledge.grasp_xforms(i)
                if g:
                    for grasp,xform in g:
                        gldraw.xform_widget(xform,0.02,0.002)

        # Draws the object held on gripper
        obj,limb,grasp = self.picking_controller.held_object,self.picking_controller.active_limb,self.picking_controller.active_grasp

        if obj != None:
            # spatula scooping
            if self.picking_controller.stateLeft == 'holding':
                R_obj = obj.randRt[0]
                t_obj = obj.randRt[1]
                gripper_xform = self.simworld.robot(0).link(left_gripper_link_name).getTransform()
                objxform = se3.mul( gripper_xform,  se3.mul(  left_gripper_center_xform,  [R_obj, t_obj] )  )
                # objxform = se3.mul( gripper_xform,  se3.mul(  left_gripper_center_xform,  se3.inv(grasp.grasp_xform) )  )

            # gripper
            elif self.picking_controller.stateRight == 'holding':
                gripper_xform = self.simworld.robot(0).link(right_gripper_link_name).getTransform()
                # objxform = se3.mul(gripper_xform,se3.mul(left_gripper_center_xform,se3.inv(grasp.grasp_xform)))
                # objxform = se3.mul(gripper_xform,se3.mul(left_gripper_center_xform,se3.inv(se3.identity())))
                objxform = se3.mul(gripper_xform,left_gripper_center_xform)

            glDisable(GL_LIGHTING)
            glColor3f(1,1,1)
            draw_oriented_wire_box(objxform,obj.info.bmin,obj.info.bmax)
            glEnable(GL_LIGHTING)



        #show gripper and camera frames
        if self.draw_gripper_and_camera:
            left_camera_link = self.simworld.robot(0).link(left_camera_link_name)
            right_camera_link = self.simworld.robot(0).link(right_camera_link_name)
            left_gripper_link = self.simworld.robot(0).link(left_gripper_link_name)
            right_gripper_link = self.simworld.robot(0).link(right_gripper_link_name)
            gldraw.xform_widget(left_camera_link.getTransform(),0.1,0.01,fancy=False)
            gldraw.xform_widget(right_camera_link.getTransform(),0.1,0.01,fancy=False)
            gldraw.xform_widget(se3.mul(left_gripper_link.getTransform(),left_gripper_center_xform),0.05,0.005,fancy=False)
            gldraw.xform_widget(se3.mul(right_gripper_link.getTransform(),right_gripper_center_xform),0.05,0.005,fancy=False)

        # Show world frame and shelf frame
        gldraw.xform_widget(knowledge.shelf_xform, 0.1, 0.015, lighting=False, fancy=True)
        gldraw.xform_widget(se3.identity(), 0.2, 0.037, lighting=False, fancy=True)


        #draw order box
        glDisable(GL_LIGHTING)
        glColor3f(1,0,0)
        draw_oriented_wire_box(order_bin_xform,order_bin_bounds[0],order_bin_bounds[1])
        glEnable(GL_LIGHTING)


        # draw milestones
        glDisable(GL_LIGHTING)

        if self.drawVE :
            V,E =self.picking_controller.planner.roadmap
            positions = []

            gldraw.xform_widget(self.simworld.robot(23).link(left_camera_link_name).getTransform(), 0.1, 0.015, lighting=False, fancy=True)

            for v in V:
                qcmd = self.planworld.robot(0).getConfig()
                for k in range(len(self.picking_controller.planner.limb_indices)):
                    qcmd[self.picking_controller.planner.limb_indices[k]] = v[k]

                self.planworld.robot(0).setConfig(qcmd)

                linkNum = 23
                R = self.planworld.robot(0).link(linkNum).getTransform()[0]
                t = self.planworld.robot(0).link(linkNum).getTransform()[1]
                # loc = self.planworld.robot(0).link(54).getTransform()[1]
                positions.append(t)

                # remove this line later (slows down the visualizer)
                # gldraw.xform_widget([R,t], 0.015, 0.002, lighting=False, fancy=True)

            glColor3f(0.1,0.1,0.1)
            glLineWidth(0.1)
            glBegin(GL_LINES)
            for (i,j) in E:
                glVertex3f(*positions[i])
                glVertex3f(*positions[j])
            glEnd()

        if self.drawPath:
            #if the path is found, draw configurations along the path
            path = self.picking_controller.planner.pathToDraw

            qcmd = self.planworld.robot(0).getConfig()
            if path and len(path)>0:
                # path smoother
                smoothMaxIter = 5
                for smoothIter in range(smoothMaxIter):
                    # path = path
                    smoothePath = [0]*(len(path)*2-1)
                    for i in range(len(path)-1):
                        smoothePath[i*2] = path[i]
                        smoothePath[i*2 +1] = vectorops.div(vectorops.add(path[i],path[i+1]), 2)
                    smoothePath[-1] = path[-1]
                    path = smoothePath

                glColor3f(0,1,0)
                glLineWidth(5.0)
                glBegin(GL_LINE_STRIP)
                # print "Drawing path (limb", self.picking_controller.planner.activeLimb,"(",len(self.picking_controller.planner.limb_indices),"/",len(path[0]),"))"
                for q in path:
                    for k in range(len(self.picking_controller.planner.limb_indices)):
                        if k>len(q): print "Index Out of Range: (k,len(q),q)", k,len(q),q
                        qcmd[self.picking_controller.planner.limb_indices[k]] = q[k]
                    self.planworld.robot(0).setConfig(qcmd)
                    glVertex3f(*self.planworld.robot(0).link(23).getTransform()[1])
                glEnd()

                glColor3f(0,1,0)
                glLineWidth(5.0)
                glBegin(GL_LINE_STRIP)
                for q in path:
                    for k in range(len(self.picking_controller.planner.limb_indices)):
                        qcmd[self.picking_controller.planner.limb_indices[k]] = q[k]
                    self.planworld.robot(0).setConfig(qcmd)
                    glVertex3f(*self.planworld.robot(0).link(43).getTransform()[1])
                glEnd()

                glLineWidth(1.0)

        glEnable(GL_LIGHTING)


        for frame in self.picking_controller.frames:
            R = frame[0]
            t = frame[1]
            gldraw.xform_widget([R,t], 0.03, 0.0045, lighting=True, fancy=True)

        return

    def keyboardfunc(self,c,x,y):
        c = c.lower()
        if c=='z':
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
    world = robotsim.WorldModel()
    #uncomment these lines and comment out the next 2 if you want to use the
    #full Baxter model
    # print "Loading full Baxter model (be patient, this will take a minute)..."
    # world.loadElement(os.path.join(model_dir,"baxter.rob"))
    print "Loading simplified Baxter model..."
    # world.loadElement(os.path.join(model_dir,"baxter_with_parallel_gripper_col.rob"))
    world.loadElement(os.path.join(model_dir,"baxter_with_new_spatula_col.rob"))
    print "Loading Kiva pod model..."
    # world.loadElement(os.path.join(model_dir,"kiva_pod/meshes/pod_lowres.stl"))
    # world.loadElement(os.path.join(model_dir,"Amazon_Picking_Shelf.rob"))
    world.loadElement(os.path.join(model_dir,"Amazon_Picking_Shelf.STL"))

    print "Loading plane model..."
    world.loadElement(os.path.join(model_dir,"plane.env"))

    #shift the Baxter up a bit (95cm)
    Rbase,tbase = world.robot(0).link(0).getParentTransform()
    world.robot(0).link(0).setParentTransform(Rbase,(0,0,0.95))
    world.robot(0).setConfig(world.robot(0).getConfig())

    #translate pod to be in front of the robot, and rotate the pod by 90 degrees
    # shelf_xform_from_perception =
    t_obj_shelf = [0.5,0,0]
    t_shelf = [-1.5,0,0.1]
    reorient = ([1,0,0,0,0,1,0,-1,0],vectorops.add(t_shelf,t_obj_shelf))
    reorient_with_scale = ([0.001,0,0,0,0,0.001,0,-0.001,0],t_shelf)
    Trel = (so3.rotation((0,0,1),-math.pi/4),[2,0.75,-0.1])

    xform = se3.mul(Trel,reorient)
    xform_with_scale = se3.mul(Trel,reorient_with_scale)
    world.terrain(0).geometry().transform(xform_with_scale[0], xform_with_scale[1])

    #initialize the shelf xform for the visualizer and object
    global ground_truth_shelf_xform
    ground_truth_shelf_xform = xform
    return world

def load_baxter_only_world():
    """Produces a world with only the Baxter in it."""
    world = robotsim.WorldModel()
    print "Loading simplified Baxter model..."
    world.loadElement(os.path.join(model_dir,"baxter_with_new_spatula_col.rob"))

    #shift the Baxter up a bit (95cm)
    Rbase,tbase = world.robot(0).link(0).getParentTransform()
    world.robot(0).link(0).setParentTransform(Rbase,(0,0,0.95))
    world.robot(0).setConfig(world.robot(0).getConfig())
    return world

def spawn_objects_from_ground_truth(world):
    """For all ground_truth_items, spawns RigidObjects in the world
    according to their sizes / mass properties"""

    print "Initializing world objects"

    for i in range(len(ground_truth_items)):
        item = ground_truth_items[i]
        # obj = world.makeRigidObject(item.info.name)
        obj = world.makeRigidObject(item.bin_name)
        bmin,bmax = item.info.bmin,item.info.bmax
        center = vectorops.div(vectorops.add(bmin,bmax),2.0)

        m = obj.getMass()
        m.setMass(item.info.mass)
        m.setCom([0,0,0])
        m.setInertia(vectorops.mul([bmax[0]-bmin[0],bmax[1]-bmin[1],bmax[2]-bmin[2]],item.info.mass/12.0))
        obj.setMass(m)

        c = obj.getContactParameters()

        c.kFriction = 0.1
        c.kRestitution = 0.001;
        c.kStiffness = 100
        c.kDamping = 10
        obj.setContactParameters(c)

        simgeometry = obj.geometry()
        load_item_geometry(item,simgeometry)

        # Spawn objects a little bit higher than bin floor
        t = item.xform[1]
        t = [t[0], t[1], t[2]+0.01]
        obj.setTransform(item.xform[0],t)

    return

def myCameraSettings(visualizer):
    visualizer.camera.tgt = [-1, -.5, -0.75]
    visualizer.camera.rot = [0,0.5,0.9]
    visualizer.camera.dist = 4
    visualizer.fov = 60
    visualizer.width *= 2
    visualizer.height *= 2
    return


global simWorld

if __name__ == "__main__":
    """The main loop that loads the planning / simulation models and
    starts the OpenGL visualizer."""
    world = load_apc_world()
    init_ground_truth()

    # Instantiate global knowledge base
    knowledge = KnowledgeBase()

    # TODO
    if NO_SIMULATION_COLLISIONS:
        # simworld = load_baxter_only_world()
        simWorld = load_apc_world()
    else:
        simWorld = load_apc_world()
        spawn_objects_from_ground_truth(simWorld)

        # NOTE: is this necessary?
        # spawn_shelf(world)
        spawn_objects_from_ground_truth(world)

    # load the resting configuration from klampt_models/baxter_rest.config
    global baxter_rest_config
    f = open(model_dir+'baxter_new_spatula_rest.config','r')
    baxter_rest_config = loader.readVector(f.readline())
    f.close()
    simWorld.robot(0).setConfig(baxter_rest_config)

    # Add initial joint values to additional joints
    n = world.robot(0).numLinks()
    if len(baxter_rest_config) < n:
        baxter_rest_config += [0.0]*(n-len(baxter_rest_config))
        print "# links in rest_config < # links in robot"
    world.robot(0).setConfig(baxter_rest_config)
    # set spatula center point
    knowledge.center_point = simWorld.robot(0).link(left_camera_link_name).getTransform()[1]

    #run the visualizer
    visualizer = MyGLViewer(simWorld,world)
    myCameraSettings(visualizer)
    visualizer.run()

