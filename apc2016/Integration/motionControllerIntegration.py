#!/usr/bin/python

# NOTE: Key sequence to control the baxter as desired:
#       bin number (A~L) --> scoop (S) --> move spatula to center (N) --> move gripper to center (M)
#       --> grasp object (X) --> place object in order bin (P) --> bin number (A~L) --> unscoop (Y) --> return to starting position (N)


# TODO:
        # JSON Parser
        # Pyserial to 1) turn on/off vacuum; 2) control spatula; 3) read vacuum pressure
        # Communicate with perception group


#
import sys, struct, time

sys.path.insert(0, "../../common")
sys.path.insert(0, "..")

#sys.path.insert(0, "/home/hpb3/Klampt/Python")

from klampt import robotsim
from klampt.glprogram import *
from klampt import vectorops, se3, so3, loader, gldraw, ik
from klampt.robotsim import Geometry3D
from klampt import visualization,trajectory
import os, math, random, copy

from threading import Thread,Lock
from Queue import Queue
from operator import itemgetter
import cPickle as pickle
import subprocess
import numpy as np

from Trajectories.camera_to_bin import *
from Trajectories.view_to_grasp import *
from Trajectories.grasp_to_stow_90 import *
from Trajectories.grasp_to_stow_straight import *
#-----------------------------------------------------------
#Imports require internal folders

from Motion import motion
from Motion import config
from Motion import motion_debouncer


from Group2Helper import apc
from Group2Helper.baxter import *
from Group2Helper.motionPlanner import *
from Group2Helper import binOrder


# import baxter_interface

# Perception
from perception import perception

from Group2Helper import Vacuum_Comms
#================================================================
# End of Imports


NO_SIMULATION_COLLISIONS = 0
FAKE_SIMULATION = 0
PHYSICAL_SIMULATION = 1


#============================================================
# configuration variables



ALL_ARDUINOS = 0
MOTOR = 0 or ALL_ARDUINOS
VACUUM = 1 or ALL_ARDUINOS

WAIT_TIME = 2

SPEED = 3

REAL_SCALE = False
REAL_CAMERA = True
REAL_JSON = True
REAL_PRESSURE = True

#TASK = 'pick'
TASK = 'stow'

SHELF_STATIONARY = False

PRESSURE_THRESHOLD = 840

SKIP_GRASP_FROM_TOTE= False
SKIP_STOWING_INPUT = False


INIT_DEGREE_OFFSET = 0

TOTE_BOUNDS = [[],[]]
TOTE_BOUNDS[0] = [0.51508961858971, 0.20121429760198456, 0.398518110006264]
TOTE_BOUNDS[1] =  [0.7662464010255284, -0.26451044561443077, 0.20620107315207947]

TOTE_BOUNDS_Z_MAX= max(TOTE_BOUNDS[0][2], TOTE_BOUNDS[1][2])
TOTE_BOUNDS_Z_MIN = min(TOTE_BOUNDS[0][2], TOTE_BOUNDS[1][2])

#KLAMPT_MODEL="baxter.rob"
KLAMPT_MODEL="baxter_with_two_vacuums.rob"


PERCEPTION_FAIL_THRESHOLD = 3
#========================================================================================

CALIBRATE = True
SHOW_BIN_CONTENT = True # setting this to True will show bin content as perceived by camera
SHOW_TOTE_CONTENT = True # setting this to True will show tote content as perceived by camera
SHOW_BIN_BOUNDS = True # setting this to True will draw bin bounds

LOAD_TRAJECTORY_DEFAULT = False
LOAD_PHYSICAL_TRAJECTORY = True
FORCE_WAIT = False


DRAW_STOW_DEBUG = True

if TASK == 'stow':
    END_EFFECTOR = '_STRAIGHT'
elif TASK == 'pick':
    END_EFFECTOR = '_90'

PRESSURE_FILE_PREFIX = "../Sensors/pressureReading_"
JSON_STOW_OUTPUT_FILE = "../JSON_FILES/JSON_stow_file_output.json"
JSON_PICK_OUTPUT_FILE = "../JSON_FILES/JSON_pick_file_output.json"
JSON_STOW_INPUT_FILE = "../JSON_FILES/apc_stow_task.json"
JSON_PICK_INPUT_FILE = "../JSON_FILES/apc_pick_task.json"



PICK_TIME = 9000
STOW_TIME = 9000


visualizer = None

if REAL_SCALE:
    from Sensors import scale
    from Group2Helper import stowHandler
    if TASK == 'stow':
        stowHandler = stowHandler.stowHandler(JSON_STOW_INPUT_FILE)

if REAL_JSON:
    # JSON parser
    binOrderParser = binOrder.binOrder()

    # Order list. Can be parsed from JSON input
    global binList
    global singleItemList

    global orderList
    (binList, orderList, singleItemList) = binOrderParser.workBinOrder("../JSON_FILES/apc_pick_task.json")


perceiver = perception.Perceiver()

# # NOTE: Arduino stuff
# # import Pressure_Comms
if PHYSICAL_SIMULATION:
    if MOTOR:
        import Motor_Comms_2
        spatulaController = Motor_Comms_2.MoveSpatula()
        spatulaCommand = [0,0,0,0]

    if VACUUM:
        vacuumController = Vacuum_Comms.CommVacuum()

# The path of the klampt_models directory
model_dir = "../klampt_models/"
TRAJECTORIES_PATH = "../Trajectories/"
PATHING_PATH = "../Trajectories"


# The transformation of the order bin
order_bin_xform = (so3.identity(),[0.65,-0.55,0])
order_bin_bounds = ([-0.2,-0.4,0],[0.2,0.4,0.7])

SCAN_STATE = 'scan'
GRASP_STATE = 'grasp'
GRASP_PREPPED_STATE = 'graspPrepped'
REST_STATE = 'ready'
STOW_STATE = 'stow'


class KnowledgeBase:
    def __init__(self):
        self.bin_contents = dict((n,None) for n in apc.bin_names)
        self.order_bin_contents = []
        self.center_point = None
    def bin_front_center(self,bin_name):

        #center of the front face of the bin

        bmin,bmax = apc.bin_bounds[bin_name]
        # local_center = [(bmin[0]+bmax[0])*0.5, (bmin[1]+bmax[1])*0.5, bmax[2]]
        local_center = [(bmin[0]+bmax[0])*0.5, bmin[1], bmax[2]]

        # horizontal adjustment
        if bin_name == 'bin_A' or bin_name == 'bin_D' or bin_name == 'bin_G' or bin_name == 'bin_J':
            local_center = vectorops.add(local_center, [-0.0,0,0])
        elif bin_name == 'bin_B' or bin_name == 'bin_E' or bin_name == 'bin_H' or bin_name == 'bin_K':
            # local_center = vectorops.add(local_center, [-0.00,0,0])
            local_center = vectorops.add(local_center, [0.00,0,0])
        elif bin_name == 'bin_C' or bin_name == 'bin_F' or bin_name == 'bin_I' or bin_name == 'bin_L':
            local_center = vectorops.add(local_center, [-0.01,0,0])

        if bin_name == 'bin_J':
            local_center = vectorops.add(local_center, [-0.01,0,0])
        if bin_name == 'bin_K':
            local_center = vectorops.add(local_center, [-0.025,0,0])
        # in/out adjustment
        #if bin_name == 'bin_J' or bin_name == 'bin_K' or bin_name == 'bin_L':
        #    local_center = vectorops.add(local_center, [0,0,-0.003])


        world_center = se3.apply(knowledge.shelf_xform, local_center)
        return world_center
    def bin_vantage_point(self,bin_name):

        # a little in front of the front_center of the bin

        world_center = self.bin_front_center(bin_name)
        world_offset = so3.apply(knowledge.shelf_xform[0],[0,0.04,0.55])

        if bin_name == 'bin_A' or bin_name == 'bin_D' or bin_name == 'bin_G' or bin_name == 'bin_J':
            world_offset = vectorops.add(world_offset, [-0.04,0,0])
        elif bin_name == 'bin_B' or bin_name == 'bin_E' or bin_name == 'bin_H' or bin_name == 'bin_K':
            world_offset = vectorops.add(world_offset, [-0.04,0,0])
        elif bin_name == 'bin_C' or bin_name == 'bin_F' or bin_name == 'bin_I' or bin_name == 'bin_L':
            world_offset = vectorops.add(world_offset, [-0.04,0,0])

        return vectorops.add(world_center,world_offset)
   
    def getGlobalBounds(self, bin_name):
        #center of the front face of the bin
        bmin,bmax = apc.bin_bounds[bin_name]

        globPoint1 = se3.apply(knowledge.shelf_xform, bmin)
        globPoint2 = se3.apply(knowledge.shelf_xform, bmax)

        if globPoint1[0] > globPoint2[0]:
            xmax = globPoint1[0]
            xmin = globPoint2[0]
        else:
            xmin = globPoint1[0]
            xmax = globPoint2[0]

        if globPoint1[1] > globPoint2[1]:
            ymax = globPoint1[1]
            ymin = globPoint2[1]
        else:
            ymin = globPoint1[1]
            ymax = globPoint2[1]

        if globPoint1[2] > globPoint2[2]:
            zmax = globPoint1[2]
            zmin = globPoint2[2]
        else:
            zmin = globPoint1[2]
            zmax = globPoint2[2]

        minPoint = [xmin, ymin, zmin]
        maxPoint = [xmax, ymax, zmax]

        #world_center = se3.apply(knowledge.shelf_xform, local_center)
        # print '[minPoint, maxPoint] = ', [minPoint, maxPoint]
        return [minPoint, maxPoint]

    def getBinFrontCenterTop(self, bin_name):
        #center of the front face of the bin near the top of the bin
        minMax = apc.bin_bounds[bin_name]
        #get the min of  x, 
        #get the midpoint of y
        #get close to the top of z
        pointX = minMax[0][0]*.5 + minMax[1][0]*.5
        pointY = minMax[0][1]*.2+minMax[1][1]*.8
        pointZ = minMax[1][2]

        return se3.apply(knowledge.shelf_xform, [pointX, pointY, pointZ])

    def getBinMidCenterTop(self, bin_name):
        #center of the front face of the bin
        minMax = apc.bin_bounds[bin_name]
        #get the midpoint of x, 
        #get the midpoint of y
        #get close to the top of z
        pointX = minMax[0][0]*.5 + minMax[1][0]*.5
        pointY = minMax[0][1]*.2 + minMax[1][1]*.8
        pointZ = minMax[0][2]*.5+minMax[1][2]*.5

        return se3.apply(knowledge.shelf_xform, [pointX, pointY, pointZ])

    def getBinFrontCenter(self, bin_name):
        #center of the front face of the bin
        minMax = apc.bin_bounds[bin_name]
        #get the min of  x, 
        #get the midpoint of y
        #get the midpoint of z
        pointX = minMax[0][0]*.5 + minMax[1][0]*.5
        pointY = minMax[0][1]*.5 + minMax[1][1]*.5
        pointZ = minMax[1][2]

        return se3.apply(knowledge.shelf_xform, [pointX, pointY, pointZ])

    def getBinTrueCenter(self, bin_name):
        #get the very center of the bin
        minMax = apc.bin_bounds[bin_name]
        #get the midpoint of  x, 
        #get the midpoint of y
        #get the midpoint of z
        pointX = minMax[0][0]*.5 + minMax[1][0]*.5
        pointY = minMax[0][1]*.5 + minMax[1][1]*.5
        pointZ = minMax[0][2]*.5+minMax[1][2]*.5

        return se3.apply(knowledge.shelf_xform, [pointX, pointY, pointZ])

    def getBinWorldPosition(self, bin_name, localPoint):
        #minMax = self.getGlobalBounds(bin_name)

        bound = apc.bin_bounds[bin_name]

        if len(localPoint) ==3 and  0<= localPoint[0] <= 1 and 0<=localPoint[1] <=1 and 0<=localPoint[2] <=1:
            myPointX = bound[0][0] * localPoint[0] + (1-localPoint[0])*bound[1][0]
            myPointY = bound[0][1] * localPoint[1] + (1-localPoint[1])*bound[1][1]
            myPointZ = bound[0][2] * localPoint[2] + (1-localPoint[2])*bound[1][2]
            return se3.apply(knowledge.shelf_xform, [myPointX, myPointY, myPointZ])

        else:
            print 'Error incorrect entry format'
            return False

    def sampleBin(self, bin_name, sample=None, xPoints=3, yPoints=3, zPoints=3, startRatio = 0.1, endRatio = 0.9):
        minMax = apc.bin_bounds[bin_name]

        if sample != None:
            try:
                assert(len(sample)==3)
            except:
                print 'error in sample size - should be a list of 3 integers'
                return False
            xPoints = sample[0]
            yPoints = sample[1]
            zPoints = sample[2]

        #defaultRange = 0.1 - 0.9
        incX = (endRatio-startRatio)/xPoints
        incY = (endRatio-startRatio)/yPoints
        incZ = (endRatio-startRatio)/zPoints

        startX = startRatio

        endX = endRatio
        endY = endRatio
        endZ = endRatio

        returnPoints = []

        while startX < endX:
            startY = startRatio
            while startY < endY:
                startZ = startRatio
                while startZ < endZ:
                    xLoc = startX * minMax[0][0] + (1-startX)*minMax[1][0]
                    yLoc = startY*minMax[0][1] + (1-startY)*minMax[1][1]
                    zLoc = startZ*minMax[0][2] + (1-startZ)*minMax[1][2]
                    returnPoints.append(se3.apply(knowledge.shelf_xform, [xLoc,yLoc,zLoc]))
                    startZ = startZ + incZ
                startY = startY +incY
            startX = startX + incX

        return returnPoints

    def applyShelfXform(self, point):
        return se3.apply(knowledge.shelf_xform, point)


    def getShelfNormal(self):
        # assume z = 0
        # it's already so close to zero it doesn't affect much
        rot_matrix = knowledge.shelf_xform[0]
        # normal to the back plane of the shelf appears to point in the +z direction
        return rot_matrix[6:9]



class LowLevelController:
    def __init__(self,robotModel,robotController, simulator):
        self.robotModel = robotModel
        self.controller = robotController
        self.sim = simulator
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
    def setLinear(self,destination,dt=0.1):
        """Immediately sets the motion queue to move to the given
        milestone.  If endvelocity is given, then the end of the
        queue will be moving at that velocity.  Otherwise, the end
        velocity will be zero."""
        self.lock.acquire()
        self.controller.setLinear(destination,dt)
        self.lock.release()
    def appendLinear(self,destination,dt=0.1):
        """Appends a milestone to the motion queue.  If endvelocity
        is given, then the end of the queue will be moving at that velocity.
        Otherwise, the end velocity will be zero."""
        self.lock.acquire()
        self.controller.appendLinear(destination,dt)
        self.lock.release()
    def isMoving(self):
        return self.controller.remainingTime()>0
    def remainingTime(self):
        return self.controller.remainingTime()
    def commandGripper(self,limb,command,spatulaPart = None):
        """Sends the command to the indicated gripper.
        For the parallel-jaw gripper, [0] is closed, [1] is open
        Warning: don't do this while moving"""
        self.lock.acquire()
        q = self.controller.getCommandedConfig()
        self.robotModel.setConfig(q)
        set_model_gripper_command(self.robotModel,limb,command,spatulaPart)
        self.controller.setMilestone(self.robotModel.getConfig())
        self.lock.release()
    def appendMilestoneRight(self, destination, dt=2, endvelocity=None):
        self.lock.acquire()
        myConfig = self.robotModel.getConfig()
        if len(destination) < len(myConfig):
            for i in range(len(right_arm_geometry_indices)):
                myConfig[right_arm_geometry_indices[i]] = destination[i]  
        if endvelocity == None: self.controller.addMilestoneLinear(myConfig)
        else: self.controller.addMilestone(myConfig,endvelocity)
        self.lock.release()
    def appendMilestoneLeft(self, destination, dt=2, endvelocity=None):
        self.lock.acquire()
        myConfig = self.robotModel.getConfig()
        if len(destination) < len(myConfig):
            for i in range(len(left_arm_geometry_indices)):
                myConfig[left_arm_geometry_indices[i]] = destination[i]  
        if endvelocity == None: self.controller.addMilestoneLinear(myConfig)
        else: self.controller.addMilestone(myConfig,endvelocity)
        self.lock.release()


class FakeLowLevelController:
    def __init__(self,robotModel,robotController):
        self.robotModel = robotModel
        self.controller = robotController
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
    def commandGripper(self,limb,command,spatulaPart = None):
        """Sends the command to the indicated gripper.
        For the parallel-jaw gripper, [0] is closed, [1] is open
        Warning: don't do this while moving"""
        self.lock.acquire()
        self.robotModel.setConfig(self.config)
        set_model_gripper_command(self.robotModel,limb,command,spatulaPart)
        self.config = self.robotModel.getConfig()
        self.lastCommandTime = time.time()
        self.lock.release()
    def appendMilestoneRight(self, destination, dt=2):
        myConfig = self.robotModel.getConfig()

        self.lock.acquire()

        if len(destination) < len(myConfig):
            for i in range(len(right_arm_geometry_indices)):
                myConfig[right_arm_geometry_indices[i]] = destination[i]  
        self.config = myConfig
        self.lastCommandTime = time.time()
        self.lock.release()
    def appendMilestoneLeft(self, destination, dt=2):
        self.lock.acquire()
        myConfig = self.robotModel.getConfig()
        if len(destination) < len(myConfig):
            for i in range(len(left_arm_geometry_indices)):
                myConfig[left_arm_geometry_indices[i]] = destination[i]  
        self.config = myConfig
        self.lastCommandTime = time.time()
        self.lock.release()

class PhysicalLowLevelController(LowLevelController):
    """A low-level controller for the real robot."""
    def __init__(self,robotModel,klampt_model=KLAMPT_MODEL):
        print "Setting up motion_physical library with model",klampt_model
        #motion.setup(klampt_model = "../klampt_models/"+klampt_model)
        #motion.setup(mode="physical",libpath="", klampt_model = klampt_model)
        #self.left_arm_indices = [robotModel.getLink(i).index for i in baxter.left_arm_link_names]
        #self.right_arm_indices = [robotModel.getLink(i).index for i in baxter.right_arm_link_names]
        self.left_arm_indices = left_arm_geometry_indices
        self.right_arm_indices = right_arm_geometry_indices 
        #if not motion.robot.startup():
        #    raise RuntimeError("Robot could not be started")

        motion.robot.left_limb.enableSelfCollisionAvoidance(True)
        motion.robot.right_limb.enableSelfCollisionAvoidance(True)
        #logger.warn('!!! Baxter self-collision avoidance disabled !!!')

        calibfile = "hardware_layer/baxter_motor_calibration_gravity_only.json"
        if not motion.robot.loadCalibration(calibfile):
            print "Warning, could not load gravity compensation calibration file",calibfile
        else:
            print "Using gravity compensation from",calibfile
    def getSensedConfig(self):
        return motion.robot.getKlamptSensedPosition()
    def getSensedVelocity(self):
        return motion.robot.getKlamptSensedVelocity()
    def getCommandedConfig(self):
        return motion.robot.getKlamptCommandedPosition()
    def getCommandedVelocity(self):
        return motion.robot.getKlamptCommandedVelocity()
    def setCartesianVelocityCommand(self,limb,velocity,angularVelocity=[0.0,0.0,0.0]):
        if limb == 'left':
            motion.robot.left_ee.driveCommand(angularVelocity,velocity)
        else:
            motion.robot.right_ee.driveCommand(angularVelocity,velocity)
    def setPIDCommand(self,configuration,velocity):
        if not motion.robot.left_limb.positionCommand([configuration[v] for v in self.left_arm_indices]): raise RuntimeError()
        if not motion.robot.right_limb.positionCommand([configuration[v] for v in self.right_arm_indices]): raise RuntimeError()
        return True
    def setConfig(self,destination,duration=0.1):
        if not motion.robot.left_mq.setLinear(0.1, [destination[v] for v in self.left_arm_indices]): raise RuntimeError()
        if not motion.robot.right_mq.setLinear(0.1, [destination[v] for v in self.right_arm_indices]): raise RuntimeError()
        return True
    def appendConfig(self,destination,duration=0.1):
        if not motion.robot.left_mq.appendLinear(0.1, [destination[v] for v in self.left_arm_indices]): raise RuntimeError()
        if not motion.robot.right_mq.appendLinear(0.1, [destination[v] for v in self.right_arm_indices]): raise RuntimeError()
        return True
    def setMilestone(self,destination,endvelocity=None):
        #if not motion.robot.left_mq.setRamp([destination[v] for v in self.left_arm_indices]): raise RuntimeError()
        #if not motion.robot.right_mq.setRamp([destination[v] for v in self.right_arm_indices]): raise RuntimeError()

        #new debounced code
        #motion_debouncer.send_debounced_motion_command(motion,'left',[destination[v] for v in self.left_arm_indices],append=False)
        #motion_debouncer.send_debounced_motion_command(motion,'right',[destination[v] for v in self.right_arm_indices],append=False)

        self.setConfig(destination,endvelocity)
        return True
    def appendMilestone(self,destination,endvelocity=None):
        #if not motion.robot.left_mq.appendRamp([destination[v] for v in self.left_arm_indices]): raise RuntimeError()
        #if not motion.robot.right_mq.appendRamp([destination[v] for v in self.right_arm_indices]): raise RuntimeError()

        #new debounced code
        #motion_debouncer.send_debounced_motion_command(motion,'left',[destination[v] for v in self.left_arm_indices],append=True)
        #motion_debouncer.send_debounced_motion_command(motion,'right',[destination[v] for v in self.right_arm_indices],append=True)

        self.appendConfig(destination,endvelocity)
        return True
    def setLinear(self,destination,dt=0.1):
        self.setConfig(destination,dt)
    def appendLinear(self,destination,dt=0.1):
        self.appendConfig(destination,dt)
    def isMoving(self):
        return motion.robot.moving()
    def remainingTime(self):
        return max(motion.robot.left_mq.moveTime(),motion.robot.right_mq.moveTime())
    def commandGripper(self,limb,command,spatulaPart = None):
        global spatulaController
        global spatulaCommand
        global vacuumController

        # spatula
        if limb == 'left':
            if command[0] == 1:
                spatulaCommand = spatulaController.advance(spatulaPart)
            elif command[0] == 0:
                spatulaController.reset_spatula()
            elif command[0] == 0.5:
                "running prepare"
                spatulaController.prepare()

        # vacuum
        elif limb == 'right':
            # turn vacuum on
            vacuumController.change_vacuum_state(command[0])
        return

    def appendMilestoneRight(self, destination, dt=2):
        print "appending milestone right...len(destination)=", len(destination)
        if len(destination) == 7:
            #while len(destination) < len(self.right_arm_indices):
            #    destination = destination + [0]
            if not motion.robot.right_mq.appendLinear(dt, destination): raise RuntimeError()
        else:
            print 'Desitnation is: ', [destination[v] for v in self.right_arm_indices[:7]]
            if not motion.robot.right_mq.appendLinear(dt, [destination[v] for v in self.right_arm_indices[:7]]): raise RuntimeError()
        return True
    def appendMilestoneLeft(self, destination, dt=2):
        #dt = 10
        if len(destination) == 7:
            #while len(destination) < len(self.left_arm_indices):
            #    destination = destination + [0]
            # ask about DOF's

            #if not motion.robot.left_mq.appendLinear(dt, destination): raise RuntimeError()
            if not motion.robot.left_mq.appendLinear(dt, destination): raise RuntimeError()
        else:
            #if not motion.robot.left_mq.appendLinear(dt, [destination[v] for v in self.left_arm_indices]): raise RuntimeError()
            if not motion.robot.left_mq.appendLinear(dt, [destination[v] for v in self.left_arm_indices[:7]]): raise RuntimeError()
        return True
        


class PickingController:
    def __init__(self,simworld,world,robotController):
        self.simworld = simworld
        self.world = world
        self.robot = world.robot(0)

        print "initialized picking controller number of robots = ", self.world.numRobots()

        self.controller = robotController
        self.planner = LimbPlanner(self.world, knowledge)

        # either 'ready' or 'holding'
        self.stateLeft = 'ready'
        self.stateRight = 'ready'
        self.active_grasp = None
        self.current_bin = None
        self.held_object = None

        #self.perceptionTransform = ([1,0,0,0,1,0,0,0,1], [0,0.0,0.0])
        self.perceptionTransform = ( so3.rotation([0,0,1], -1*INIT_DEGREE_OFFSET*math.pi/180), [0,0,0])
        self.perceptionTransform = ( so3.rotation([0,0,1], 1*INIT_DEGREE_OFFSET*math.pi/180), [0,0,0])
        self.shelf_xform = ([1,0,0,0,1,0,0,0,1],[0,0,0])

        self.cameraTransform = ([-0.028246296697197804, 0.9930125976810241, -0.11457804139398291, -0.09707450692261138, -0.1168069991014857, -0.9883990414132534, -0.9948762168373654, -0.016796005706505204, 0.09969557344075664], [-0.16500000000000004, -0.010000000000000014, 0.024999999999999994])
        self.vacuumTransform = []

        if TASK == 'stow':
            self.vacuumTransform = ([1, 0, 0, 0, 1, 0, 0, 0, 1], [-0.01, -0.01, 0.55])
        else:
            self.vacuumTransform = ([1, 0, 0, 0, 1, 0, 0, 0, 1], [0.04, 0.035, 0.51])                 #90 degrees


        #transform to end effector

        self.left_bin = None
        self.right_bin = None


        self.stowItems = None

        #these may be helpful
        self.left_camera_link = self.robot.link(left_camera_link_name)
        self.right_camera_link = self.robot.link(right_camera_link_name)
        self.left_gripper_link = self.robot.link(left_gripper_link_name)
        self.right_gripper_link = self.robot.link(right_gripper_link_name)

        self.q_most_recent_right = None 
        self.q_most_recent_left = None

        self.points = None
        self.bin_content_cloud = None
        self.bin_render_downsample_rate = 5
        self.bin_render_ptsize = 1
        self.pick_pick_pos = None

        self.tote_cloud = None
        self.tote_render_downsample_rate = 5
        self.tote_render_ptsize = 1
        self.stow_pick_pos = None
        self.all_stow_pick_poss = None
        self.perceptionFailCount = 0
        # self.left_arm_links = [self.robot.link(i) for i in left_arm_link_names]
        # self.right_arm_links = [self.robot.link(i) for i in right_arm_link_names]
        self.vacuum_link = self.robot.link("vacuum:vacuum")
        self.debugPoints = []
        self.debugFailedPoints = []



        self.debugFailedLeftPoints = []
        self.debugFailedRightPoints = []
        self.leftStowPoints = []
        self.rightStowPoints = []

        # mapping from "link ID" to "link index"
        id_to_index = dict([(self.robot.link(i).getID(),i) for i in range(self.robot.numLinks())])

        # list of link indices on both arms
        # self.left_arm_indices = [id_to_index[i.getID()] for i in self.left_arm_links]
        # self.right_arm_indices = [id_to_index[i.getID()] for i in self.right_arm_links]
        self.left_arm_indices = left_arm_geometry_indices
        self.right_arm_indices = right_arm_geometry_indices

        self.pickBin = None
        self.pickedObject = None
        #for use in picking

        self.toteContents = []
        self.binContents = []
        # frames to draw
        self.frames = []

    def getWorld(self):
        return self.world

    def getPerceivedPoints(self):
        return self.points

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

    def calibratePerception(self, bin_letter, limb='left'):
        #assumes you are already at the location where you want to do the transformation
        # try:
            # sends the camera transform so that perceiver can send back the shelf transform in world frame
        self.perceptionTransform = perceiver.get_shelf_transformation(bin_letter, *self.getCameraToWorldXform(limb), limb=limb)
        print self.perceptionTransform
        self.perceptionTransformInv = se3.inv(self.perceptionTransform)
        knowledge.shelf_xform = self.perceptionTransform
        self.world.terrain(0).geometry().transform(self.perceptionTransform[0], self.perceptionTransform[1])
        # except:
        #     print "perception calibration not working"
        print "Camera calibration DONE\n"

    def saveCanonicalBinPointCloud(self, bin_letter, limb='left'):
        print 'Saving canonical point cloud for bin', bin_letter
        perceiver.save_canonical_bin_point_cloud(bin_letter, *self.getCameraToWorldXform(limb), limb=limb)

    def saveCanonicalTotePointCloud(self, limb='left'):
        print 'Saving canonical point cloud for tote'
        perceiver.save_canonical_tote_cloud(*self.getCameraToWorldXform(limb), limb=limb)

    def renderBinContent(self, bin_letter, render_type, limb='left'):
        print 'Render bin content for bin %s. render_type: %s - %s'%(bin_letter, render_type, ['none', 'full', 'content', 'cc'][int(render_type)])
        if render_type in ['full', '1']:
            self.bin_content_cloud = perceiver.get_current_point_cloud(*self.getCameraToWorldXform(limb), limb=limb, colorful=True)
            self.bin_render_downsample_rate = 50
            self.bin_render_ptsize = 5
        elif render_type in ['content', '2']:
            local_bound = apc.bin_bounds['bin_'+bin_letter]
            min_local_bound, max_local_bound = local_bound
            min_global_bound = se3.apply(knowledge.shelf_xform, min_local_bound)
            max_global_bound = se3.apply(knowledge.shelf_xform, max_local_bound)

            if min_global_bound[0] < max_global_bound[0]:
                min_global_x = min_global_bound[0]
                max_global_x = max_global_bound[0]
            else:
                min_global_x = max_global_bound[0]
                max_global_x = min_global_bound[0]
            if min_global_bound[1] < max_global_bound[1]:
                min_global_y = min_global_bound[1]
                max_global_y = max_global_bound[1]
            else:
                min_global_y = max_global_bound[1]
                max_global_y = min_global_bound[1]
            
            if min_global_bound[2] < max_global_bound[2]:
                min_global_z = min_global_bound[2]
                max_global_z = max_global_bound[2]
            else:
                min_global_z = max_global_bound[2]
                max_global_z = min_global_bound[2]
            self.bin_content_cloud = perceiver.get_current_bin_content_cloud(bin_letter, *self.getCameraToWorldXform(limb), limb=limb, colorful=True, crop=True, 
                bin_bound=[[min_global_x, min_global_y, min_global_z], [max_global_x, max_global_y, max_global_z]])
            self.bin_render_downsample_rate = 1
            self.bin_render_ptsize = 5
        elif render_type in ['cc', '3']:
            self.bin_content_cloud = perceiver.get_current_bin_content_cc_cloud(bin_letter, *self.getCameraToWorldXform(limb), limb=limb)
            self.bin_render_downsample_rate = 1
            self.bin_render_ptsize = 5

    def renderToteContent(self, including_tote, limb='left'):
        '''
        including_tote is True if the tote is also rendered and False if the tote needs to be subtracted
        '''
        print 'Render tote content. Also rendering tote?', including_tote
        if including_tote:
            self.tote_cloud = perceiver.get_current_point_cloud(*self.getCameraToWorldXform(limb), limb=limb)
            self.tote_render_downsample_rate = 50
            self.tote_render_ptsize = 5
        else:
            self.tote_cloud = perceiver.get_current_tote_content_cloud(*self.getCameraToWorldXform(limb), limb=limb)
            self.tote_render_downsample_rate = 1
            self.tote_render_ptsize = 5

    def getPickPositionForPick(self, bin_letter, target_item=None, possible_items=None, limb='left'):
        local_bound = apc.bin_bounds['bin_'+bin_letter]
        min_local_bound, max_local_bound = local_bound
        min_global_bound = se3.apply(knowledge.shelf_xform, min_local_bound)
        max_global_bound = se3.apply(knowledge.shelf_xform, max_local_bound)
        
        if min_global_bound[0] < max_global_bound[0]:
            min_global_x = min_global_bound[0]
            max_global_x = max_global_bound[0]
        else:
            min_global_x = max_global_bound[0]
            max_global_x = min_global_bound[0]
        if min_global_bound[1] < max_global_bound[1]:
            min_global_y = min_global_bound[1]
            max_global_y = max_global_bound[1]
        else:
            min_global_y = max_global_bound[1]
            max_global_y = min_global_bound[1]
        
        if min_global_bound[2] < max_global_bound[2]:
            min_global_z = min_global_bound[2]
            max_global_z = max_global_bound[2]
        else:
            min_global_z = max_global_bound[2]
            max_global_z = min_global_bound[2]

        if target_item is None:
            target_item = raw_input('Enter target item: ')
        if possible_items is None:
            possible_items = raw_input('Enter possible items, separated by comma: ')
            if possible_items=='':
                possible_items = [target_item]
            elif ',' not in possible_items: # single item input
                possible_items = [possible_items]
            else:
                possible_items = map(lambda x:x.strip(), possible_items.split(','))

        print 'target item is %s'%target_item
        print 'possible items are %s'%str(possible_items)

        res = perceiver.get_picking_position_for_bin(target_item, possible_items, bin_letter, 
            *self.getCameraToWorldXform(limb), limb=limb, colorful=True, crop=True, 
            bin_bound=[[min_global_x, min_global_y, min_global_z], [max_global_x, max_global_y, max_global_z]], 
            return_bin_content_cloud=True)
        
        if res is None:
            'Object not found'
            return False

        pos, cloud = res
        print 'Picking position is:', pos
        self.pick_pick_pos = pos
        self.bin_content_cloud = cloud
        self.bin_render_downsample_rate = 1
        self.bin_render_ptsize = 5
        return True
        

    def getPickPositionForStow(self, limb='left'):
        res = perceiver.get_candidate_picking_positions_for_stowing(*self.getCameraToWorldXform(limb), limb=limb, fit=True, return_tote_content_cloud=True)
        if res is None:
            print 'Perception Failed for Getting Picking Position for Stowing'
            return False
        poss, cloud = res
        print 'All picking positions are:', poss
        if len(poss)==0:
            print 'No valid picking position. Empty poss list'
            return
        self.stow_pick_pos = poss[0]
        self.all_stow_pick_poss = poss[0:5]
        self.tote_cloud = cloud
        self.tote_render_downsample_rate = 1
        self.tote_render_ptsize = 5
        return True
    #====================================================
    '''
    DEBUG methods
    '''

    def testBinIKHelper(self, limb='right', bin='bin_A'):
        if bin in apc.bin_names:
            if limb == 'left':
                self.moveArmAway('right')
            if limb == 'right':
                self.moveArmAway('left')
            self.waitForMove()

            self.viewBinAction(b=bin, limb=limb)
            self.waitForMove()
            self.prepGraspFromBinAction(b=bin, limb=limb)
            self.waitForMove()
            points = knowledge.sampleBin(bin_name=bin)
            
            numSuccess = 0
            numFailure = 0
            numTrials = len(points)

            for point in points:

                self.waitForMove()
                #self.moveToObjectInBinFromSide()
                if self.moveToObjectInBinFromTop(position=point, limb=limb):
                    #print point, 'Successful for ', bin
                    numSuccess+=1
                else:
                    self.debugFailedPoints.append(point)
                    # print point, 'Unsuccessful for ', bin
                    numFailure +=1
            if limb == 'left':
                self.stateLeft = 'grasp'
            if limb == 'right':
                self.stateRight = 'grasp'
                #prepGraspFromBinAction makes the states graspPrepped

            self.waitForMove()
            self.moveToRestConfig()
            self.waitForMove()

            print "Summary (", bin,"):",numSuccess,"/",numTrials,"successful"

    #preprepGraspFromBinAction(self, limb, b=None)
    #viewBinAction(self,b, limb)  
    #moveToObjectInBinFromTop(self, position=None, limb=None, step=None, naiive=True)

    def testBinIK(self, limb='right', bin='all'):
        if bin == 'all':
            for bin in apc.bin_names:
                self.testBinIKHelper(limb=limb, bin=bin)
        else:
            self.testBinIKHelper(limb=limb, bin=bin)

    def testStowIK(self, limb='right', sample=None, sliceX=12, sliceY=6, sliceZ = 2):
        if sliceZ < 2:
            print "minimum sliceZ is 2 (Start, Goal)"
            return False 

        #use / to find these relative to the vacuum gripper
        point1 = [0.4798129387979302, 0.27387636103959584, 0.4489993529429332]
        point2 = [0.729562827187139, -0.22440848211492773, 0.23683088095005633]


        point1 = [0.7345506986310872, -0.15495465859843727, 0.23459600770185196]
        point2 = [0.28799199514357793, 0.1297286843466862, 0.4367729217053905]


        point1 = TOTE_BOUNDS[0]
        point2 = TOTE_BOUNDS[1]

        minX = min(point1[0], point2[0])
        minY = min(point1[1], point2[1])
        minZ = min(point1[2], point2[2])
        maxX = max(point1[0], point2[0])
        maxY = max(point1[1], point2[1])
        maxZ = max(point1[2], point2[2])

        totePoints = []

        curX = minX
        curY = minY

        while curX < maxX:
            curY = minY
            while curY < maxY:
                totePoints.append([curX, curY])
                curY = curY + (maxY-minY)*1.0/sliceY

                # print 'x, y is : ', curX, ' ',curY
            curX = curX + (maxX-minX)*1.0/sliceX

        self.viewToteAction(limb=limb)
        self.waitForMove()
        self.prepGraspFromToteAction(limb=limb)
        self.waitForMove()

        numSuccess = 0
        numFailure = 0
        numTrials = len(totePoints)
        for point in totePoints:
            self.waitForMove()
            #self.debugPoints.append()
            #self.moveToObjectInBinFromSide()
            if self.graspFromToteAction(position=point, limb=limb, points=sliceZ-1, endZ=minZ, startZ =maxZ + .1 ):
                numSuccess += 1
            else:
                numFailure += 1
        print "Summary ( Tote ):",numSuccess,"/",numTrials*sliceZ,"successful"
        
    #=======================================================

    # Process for stowing:
    '''TODO

    improve path for placeInBinAction 
    '''
    def runStowingTask(self):
        #


        print stowHandler.getToteContents()

        startTime = time.clock()
        #print startTime
        endTime = time.clock()

        if REAL_SCALE:
            toteContents = stowHandler.getToteContents()
            #get toteContents 
            print toteContents

        print endTime - startTime 

        #print toteContents == []

        while (endTime - startTime < STOW_TIME and not toteContents == []):

            #bestLimb = evalBestLimb()
            self.runPickFromTote(limb='right')
            if REAL_SCALE:
                toteContents = stowHandler.getToteContents()
            endTime = time.clock()
            print (endTime - startTime)
        
        
        if REAL_SCALE:
            stowHandler.jsonOutput(JSON_STOW_OUTPUT_FILE)
        #print out bin contents

    def runPickFromTote(self, limb, bin=None):

        #bin = getKnowledgeFromParser   

        if limb == 'left':
            self.moveArmAway('right')
        elif limb == 'right':
            self.moveArmAway('left')

        self.waitForMove()


        if self.all_stow_pick_poss is None or self.all_stow_pick_poss == []:

            if REAL_CAMERA:

                time.sleep(2)

                if self.viewToteAction(limb=limb):

                    self.waitForMove()

                    target = self.all_stow_pick_poss.pop(0)

                else:
                    print 'View action failed'
                    target = self.vacuumSearch()

        else:
            target = self.all_stow_pick_poss.pop(0)


        if REAL_CAMERA:
            value = self.chooseLimb(target=target)
        else:
            value = self.chooseLimb()

        print 'value = ', value
        if value != False:
            limb = value[0]
            path = value[1]
        else:
            return False


        self.moveToRestConfig()
        self.waitForMove()

        # need to clean path configs
        if limb == 'left':
            indices = self.right_arm_indices
        else:
            indices = self.left_arm_indices

        cleanPath = []

        for milestone in path:
            for index in indices:
                milestone[index] = self.controller.getSensedConfig()[index]
            cleanPath.append(milestone)
        


        if self.prepGraspFromToteAction(limb=limb):

            self.waitForMove()
            if SKIP_GRASP_FROM_TOTE:
                pass
            else:
                if self.graspFromToteAction(limb=limb, path=cleanPath):
                    self.waitForMove()
                else: 
                    print 'Grasp From Tote Action Failed'
                    self.moveToRestConfig()
                    return False

            self.waitForMove()

            if self.evaluateObjectAction(limb=limb):
                #self.pickedObject should be populated
                #self.pickBin should be populated
                if bin == None:
                    bin = self.pickBin

                self.waitForMove()
                if self.placeInBinAction(limb=limb, bin=bin):
                    #place successful
                    #update result
                    print 'Place successful'
                    return True
                else:
                    turnOffVacuum(limb)
                    print 'Place In Bin Action Failed'
            else:
                print 'Evaluating the object Failed'
        else:
            print 'Prep Grasp From Tote Action Failed'


        return False

    def runPickFromToteNew(self, limb, bin=None):
        if limb == 'left':
            self.moveArmAway('right')
        elif limb == 'right':
            self.moveArmAway('left')

        
        if self.all_stow_pick_poss is None or self.all_stow_pick_poss == []:

            if self.viewToteAction(limb=limb) and self.perceptionFailCount < PERCEPTION_FAIL_THRESHOLD:
                #perception hasn't failed yet
                pass
            else:
                self.perceptionFailCount = self.perceptionFailCount + 1
                if self.toteGraspBackup(limb=limb):
                    #successful - go again
                    return limb
                else:
                    #try the other limb
                    if limb == 'left':
                        return 'right'
                    else:
                        return 'left'
        
        stow_pick_pos = self.all_stow_pick_poss.pop(0)

        [chosenLimb, chosenPath] = self.chooseLimb(target = stow_pick_pos)

        self.prepGraspFromToteActionNew(limb=chosenLimb)
        self.waitForMove()

        if self.graspFromToteActionNew(limb=chosenLimb, path = chosenPath):
            # we got something
            bin = self.evaluateObjectAction()

        else:
            # we failed
            # turn off the vacuum and return the most recently used limb
            # ideally, this method will be called again
            return chosenLimb

    def viewToteAction(self,limb, doPerception=True):

        #start state = ready
        #end state = viewTote

        if LOAD_PHYSICAL_TRAJECTORY:
            if limb is not None:
                if self.moveArm(limb, statusConditional='ready', path_name='Q_VIEW_TOTE_'+limb.upper(), finalState='viewTote'):
                        self.waitForMove()
                        #time.sleep(5)
                        if REAL_CAMERA:
                            if doPerception:
                                return self.getPickPositionForStow(limb)
                        return True
                else:
                    if limb == 'left':
                        print 'left arm state is', self.stateLeft
                    else:
                        print 'right arm state is ',self.stateRight
                    print "Error in moveArm (from viewToteAction)"
                    return False
            else:
                print 'Error in viewTote action can\'t move with no limb'
                return False

            #take a picture and get a position to move back

            #global x,y = self.stow_pick_pos

    def prepGraspFromToteAction(self, limb):

        # start state = ready
        # end state = prepTote

        #assumes we're using the constants file

        #used to be Q_stow_arm_straight

        if limb is not None:
            if self.moveArm(limb, statusConditional='ready', path_name='PREP_TOTE_STOW_'+limb.upper(), finalState='prepTote'):
                return True
            else:
                print "Error in moveArm (from preGraspFromToteAction"
        else:

            print 'Error in viewTote action can\'t move with no limb'

    def graspFromToteAction(self, limb, position=None, points=None, startZ = TOTE_BOUNDS_Z_MAX+.15, endZ =TOTE_BOUNDS_Z_MIN, path=None):

        #start state = prepTote
        # end state = graspTote

        #if perception has picked something

        #x = forward
        #y = left

        #startZ
        #endZ

        if path is not None:
            print 'running moveIntoTote then returning from graspFromToteAction'
            return self.moveIntoTote(limb, path)


        if REAL_CAMERA:

            goalXY = self.stow_pick_pos
        else:
            if position is None:

                myInts = []
                while( 1):
                    myInput = raw_input("Where would you like to pick? (separated by commmas) ")
                    if myInput == 'skip':
                        self.stow_pick_pos = [0.5, 0, 0]
                        break

                    myInput = myInput.split(',')
                    for el in myInput:
                        try:
                            myInts.append(float(el))
                            assert(len(myInput)==3 or len(myInput)==2)
                            goodInput = True     
                        except:
                            goodInput = False

                    if goodInput:
                        print myInts
                        self.stow_pick_pos = myInts
                        response = raw_input("Continue? y/n ")
                        if response == 'y' and len(myInts)>=2:
                            break
                        else:
                            myInts = []
                            self.stow_pick_pos = None
            else:
                self.stow_pick_pos = position


        if not self.isInTote(self.stow_pick_pos):
            print 'Error, point is not within tote bounds'
            return False
        # goalXY = [0.5,-0.5]
        if points == None:
            points = 15.0

        incZ = (endZ-startZ)/points

        goalZ = startZ

        #vacuum X_form
        #match with several points going down

        local1 = self.vacuumTransform[1]
        local2 = vectorops.add(self.vacuumTransform[1], [0, 0, 0.5])
        #along the axis of the wrist and 0.1m fruther
        goals = []
        limbs = []
        sortedSolutions=[]

        #q_start = [conf for conf in motion.robot.getKlamptSensedPosition()]
        q_start = [conf for conf in self.simworld.robot(0).getConfig()]



        link = self.robot.link(limb+'_wrist')
        # print 'Current limb is', limb
        # print self.simworld.robot(0).link('right_wrist').getName(), self.robot.link('right_wrist')
        # print self.simworld.robot(0).link('left_wrist').getName(), self.robot.link('left_wrist')
        # print link.getName(), link
        # print 'link_xform', link.getTransform()
        path = [q_start]


        # print 'beginning debugging'
        
        while goalZ >= endZ:
            global1 = [self.stow_pick_pos[0], self.stow_pick_pos[1], goalZ]
            global2 = [self.stow_pick_pos[0],self.stow_pick_pos[1], goalZ-0.5]

            goal  = ik.objective(link,local=[local1,local2],world=[global1,global2])
            #goal1 = ik.objective(link,local=local1,world=global1)
            #goal2 = ik.objective(link,local=local2,world=global2)



            self.simworld.robot(0).setConfig([conf for conf in q_start])

            #if ik.solve([goal1, goal2], tol=1e-3):
            milestone = []
            milestone = self.simpleIK(goal = goal, limb = limb)
            

            # print "DEBUG"
            # print [milestone[v] for v in self.right_arm_indices[:7]]
                
            # print [milestone[v] for v in self.right_arm_indices[:7]]

            if milestone:
                #print milestone
                path.append(milestone)
                self.debugPoints.append(global1)
                print 'Goal Z = ', goalZ, ' solved at ', i

            else:
                self.debugFailedPoints.append(global1)
                break
                #currently skips lower points if we fail for 1


            goalZ+=incZ    
       
        # print len(path)


        # print 'Milestones in path: ========================================'
        # for milestone in path:
        #     print [milestone[v] for v in self.right_arm_indices[:7]]

        if len(path)>1:
            #print 'sending'

            #turn vacuum on
            return self.moveIntoTote(limb, path)


        print "Failed to plan path"
        self.robot.setConfig(q_start)
        return False

    def moveIntoTote(self, limb, path):

        try:
            turnOnVacuum(limb)
        except:
            print 'Error in vacuum Controller (controller is off?)'

        reversePath = []

        if REAL_PRESSURE:
            index = 0

            pressureDrop = False
            print 'length of path is ', len(path)


            self.waitForMove()
            q_current = [v for v in self.simworld.robot(0).getConfig()]

            while (not pressureDrop) and index < len(path):
                # print "milestone #",index
                if index == 0:
                    self.waitForMove()
                    print "1"
                    #self.controller.appendMilestone(q_current, path[index])
                    print "2"
                    self.sendPath([q_current, path[index]])
                else:
                    self.waitForMove()
                    self.sendPath([path[index-1], path[index]], INCREMENTAL=True)
                self.waitForMove()
                #getPressureReading
                #reevaluate noPressureDrop                    

                time.sleep(0.5)
                pressureDrop = readPressure(limb)
                print pressureDrop


                if pressureDrop:
                    #we did grab something 
                    #lift back out the reverse path
                    self.waitForMove()
                    # self.sendPath(path[::-1], limb=limb)
                    # self.sendPath(reversePath[::-1], limb=limb)
                    # self.waitForMove()
                    break
                    
                else:
                    reversePath.append(path[index])

                index= index+1
           

            if index == 1:
                self.sendPath([reversePath])
            else:
                self.sendPath(reversePath[::-1], limb=limb, internalSpeed =8)
            self.waitForMove()
            if readPressure(limb):
                print 'we actually got something'
                return True
            else:
                print 'Didn\'t grab anything'
                print 'Or dropped something on the way back up '
                turnOffVacuum(limb)
                return False

        else:                
            self.sendPath(path, limb=limb)
            # print 'sent path to go down'
            time.sleep(0.1)
            # if FAKE_SIMULATION:
            #     raw_input()
            self.sendPath(path[::-1], limb=limb)
            # print 'sent path to go up'


        #print 'sent'
        return True

    def evaluateObjectAction(self, limb):

        #start state = graspTote
        #end state = graspTote

        #TODO - 

        path_name = 'Q_EVAL_SCALE_'+limb.upper()
        print "self.moveArm(",path_name,")...",
        self.moveArm(limb = limb, path_name = path_name)

        self.waitForMove()

        time.sleep(2)

        if REAL_SCALE:
            (self.stowItems, self.pickBin) = stowHandler.pickWhichObj(limb, True)
            if self.pickBin==None:
                print "No target object in the weight range"
            # if len(items)>1:
            #     print 'More than One Item'
            #     self.pickBin = stowHandler.getBin(items)
            # elif len(items)==1:
            #     print 'Item picked is:', items
            #     self.pickBin = stowHandler.getBin(items)

            #     print 'got no Item'
            #     return False
            print self.stowItems
            print self.pickBin
            return True
        else:
            print "SCALE is off, using bin_B"
            self.pickBin = 'bin_B'
            return True

    def placeInBinAction(self, limb, bin='H'):

        #start state = graspTote
        #end state = placeInBin or retry1, retry2, retry3


        # limb, statusConditional=None, path_name=None, path=None, finalState=None, reverse=False
        if limb == 'left':
            if len(bin)==1:
                self.left_bin = 'bin_'+bin
            else:
                self.left_bin = bin
        elif limb == 'right':
            if len(bin)==1:
                self.right_bin = 'bin_'+bin
            else:
                self.right_bin = bin

        if len(bin)==5:
            #like bin_A
            bin = bin[4]

        path_name = 'PREP_TOTE_STOW_'+limb.upper()
        print "self.moveArm(",path_name,")...",
        if self.moveArm(limb = limb, path_name = path_name, reverse=True):
            print "success"      
        else:
            print "fail"
            return False

        self.waitForMove()

        path_name = 'GRASP_TO_STOW_'+bin.upper() + '_'+limb.upper() + END_EFFECTOR
        print "self.moveArm(",path_name,")...",
        if self.moveArm(limb = limb, path_name =path_name ,reverse=True, finalState ='placeInBin'):
            print "success"

            print "self.moveObjectIntoBin(", bin,")..."
            if self.moveObjectIntoBin(limb = limb, bin = bin):
                print "\tsuccess"

                print "moveArm(",path_name,")...",
                if self.moveArm(limb=limb, path_name = path_name, finalState = 'ready'):
                    print "success"
                    self.waitForMove()
                    return True
            else:
                print 'Error in moving into the bin'

            #then actually move into bin a bi
        else:
            print 'Error in moving from grasping tote object to stowing tote object'
            return False

    def moveObjectIntoBin(self, limb=None, bin=None):
        self.waitForMove()
        #self.moveToObjectInBinFromTop(limb=limb)
        #return False
        # self.graspFromBinAction(limb=limb, bin='bin_B')
        
        step1 = []
        step2 = []
        step3 = []

        path_name = 'GRASP_TO_STOW_'+bin.upper() + '_'+limb.upper() + END_EFFECTOR
        
        if bin == None:
            if limb =='left':
                bin = self.left_bin
            elif limb =='right':
                bin = self.right_bin
                
        step = 1

        if REAL_PRESSURE:

                pressureDrop = readPressure(limb)

                if pressureDrop:
                    
                    stowHandler.updateBin(bin='bin_'+bin ,item=self.stowItems)
                    pass
                else:
                    # we dropped the object
                    print 'object was dropped'
                    return False


        #log current effort

        if step==1:
            #move to the top center of the bin
            #target1 = knowledge.getBinFrontCenter('bin_'+bin)
            #x is more to the left, y is down, and z is to the back

            target1 = knowledge.getBinWorldPosition(bin_name = 'bin_'+bin, localPoint = [.5, .3, 0])

            ikGoal = self.buildIKGoalSuctionDown(limb = limb, target=target1)
            #ik to top center of bin, normal to the shelf
            #use ik seed and knowledge of shelf
            # constraintst: suction cup down, vacuum/wrist forward direction in direction of shelf
            #print target1



            print '\ttrying step1 ik'
            step1 = self.simpleIK(limb=limb, goal=ikGoal)



            if not step1:
                #failed
                #check pressure sensor for limb
                #-check in loop

                # if still on, return to tote
                # if off, return to tote - turn vacuum off
                
                print "\tIK solve failed, reverting to previous position"
                self.moveArm(limb=limb, path_name = path_name, finalState = 'ready' )
                self.waitForMove()
                # try: 
                #     turnOffVacuum(limb)
                # except:
                #     print '\tError in vacuum Comms'
                return False
            self.debugPoints.append(target1)
            #ik returns a configuration

            self.sendPath(path=[step1], limb=limb)

            #check effort - if we're colliding, return

            time.sleep(0.5)

            step =2

        if step==2: 
            #target2 = knowledge.getBinTrueCenter('bin_'+bin)
            target2 = knowledge.getBinWorldPosition(bin_name = 'bin_'+bin, localPoint = [.5, .3, .25])
            ikGoal = self.buildIKGoalSuctionDown(limb=limb, target = target2)

            print '\ttrying step2 ik'
            step2 = self.simpleIK(limb=limb, goal=ikGoal)
            if not step2:
                #failed
                #check pressure sensor for limb
                # if still on, return to tote
                print "\tIK solve failed, reverting to previous position"
                self.moveArm(limb=limb, path_name = path_name, finalState = 'ready' )
                time.sleep(0.5)

                # I recommend we keep this off if failed because we don't know if we made it
                # try:
                #     turnOffVacuum(limb)
                # except:
                #     print '\tError in vacuum Comms'
                # return False
            self.debugPoints.append(target2)
            self.sendPath(path = [step2], limb = limb)

            step = 3

        if step==3: 
            #target2 = knowledge.getBinTrueCenter('bin_'+bin)
            target3 = knowledge.getBinWorldPosition(bin_name = 'bin_'+bin, localPoint = [.5, .3, .5])
            ikGoal = self.buildIKGoalSuctionDown(limb=limb, target = target3)

            print '\ttrying step3 ik'
            step3 = self.simpleIK(limb=limb, goal=ikGoal)
            if not step3:
                #failed
                #check pressure sensor for limb
                # if still on, return to tote
                print "\tIK solve failed, reverting to previous position"
                self.sendPath(path = [step2, step1], limb = limb)
                self.waitForMove()
                self.moveArm(limb=limb, path_name = path_name, finalState = 'ready' )
                time.sleep(0.5)
                try:
                    turnOffVacuum(limb)
                except:
                    print '\tError in vacuum Comms'
                return False
            self.debugPoints.append(target3)
            self.sendPath(path = [step3], limb = limb)



            self.waitForMove()
            time.sleep(1)
            try:
                turnOffVacuum(limb)
            except:
                print '\tError in vacuum Comms'

            print "waiting for object to be released"
            time.sleep(5)

            step = 4
            # move up out of the bin

        if step==4: 
            #target2 = knowledge.getBinTrueCenter('bin_'+bin)
            target4 = knowledge.getBinWorldPosition(bin_name = 'bin_'+bin, localPoint = [.5, .15, .5])
            ikGoal = self.buildIKGoalSuctionDown(limb=limb, target = target4)

            print '\ttrying step4 ik'
            step4 = self.simpleIK(limb=limb, goal=ikGoal)
            if not step4:
                #failed
                #check pressure sensor for limb
                # if still on, return to tote
                print "\tIK solve failed, reverting to previous position"
                self.sendPath(path = [step3, step2, step1], limb = limb)
                self.waitForMove()
                self.moveArm(limb=limb, path_name = path_name, finalState = 'ready' )
                time.sleep(0.5)
                try:
                    turnOffVacuum(limb)
                except:
                    print '\tError in vacuum Comms'
                return False
            self.debugPoints.append(target4)
            self.sendPath(path = [step4], limb = limb)

            step = 5


        if step==5: 
            #target2 = knowledge.getBinTrueCenter('bin_'+bin)
            target5 = knowledge.getBinWorldPosition(bin_name = 'bin_'+bin, localPoint = [.5, .15, 0])
            ikGoal = self.buildIKGoalSuctionDown(limb=limb, target = target5)

            print '\ttrying step5 ik'
            step5 = self.simpleIK(limb=limb, goal=ikGoal)
            if not step5:
                #failed
                #check pressure sensor for limb
                # if still on, return to tote
                print "\tIK solve failed, reverting to previous position"
                self.sendPath(path = [step4, step3, step2, step1], limb = limb)
                self.waitForMove()
                self.moveArm(limb=limb, path_name = path_name, finalState = 'ready' )
                time.sleep(0.5)
                try:
                    turnOffVacuum(limb)
                except:
                    print '\tError in vacuum Comms'
                return False
            self.debugPoints.append(target5)
            self.sendPath(path = [step5], limb = limb)

        #turn vacuum off

        turnOffVacuum(limb=limb)
        self.sendPath(path = [step1], limb=limb)
        return True
    
    def chooseLimb(self, target=None, endZ =TOTE_BOUNDS_Z_MIN , startZ = TOTE_BOUNDS_Z_MAX+.15, points = 10):
        

        if REAL_SCALE:
            stowHandler.updateWeight()

        while target is None:
            myInts = []
            myInput = raw_input("Where would you like to pick? (separated by commmas) ")
            if myInput == 'skip':
                target = [0.5, 0, 0]
                break
            myInput = myInput.split(',')
            for el in myInput:
                try:
                    myInts.append(float(el))
                    assert(len(myInput)==3 or len(myInput)==2)
                    goodInput = True     
                except:
                    print 'Error bad input, try again'
                    goodInput = False

            if goodInput:
                print myInts
                self.stow_pick_pos = myInts
                response = raw_input("Continue? y/n ")
                if response == 'y' and len(myInts)>=2:
                    target = myInts
                else:
                    myInts = []
                    self.stow_pick_pos = None

        #target is now set up


        #print 'got to starting ik'

        thresholdZ = TOTE_BOUNDS_Z_MAX
        incZ = (endZ-startZ)/points
        goalZ = startZ

        #vacuum X_form
        #match with several points going down
        local1 = self.vacuumTransform[1]
        local2 = vectorops.add(self.vacuumTransform[1], [0, 0, 0.5])
        #along the axis of the wrist and 0.1m fruther
        #q_start = [conf for conf in motion.robot.getKlamptSensedPosition()]



        q_start = [conf for conf in self.controller.getSensedConfig()]

        #print 'q_start =', q_start

        test_config = [v for v in q_start]
        for i in range(len(self.left_arm_indices)):
            test_config[self.left_arm_indices[i]] = eval('PREP_TOTE_STOW_LEFT')[-1][i]
        for i in range(len(self.right_arm_indices)):
            test_config[self.right_arm_indices[i]] = eval('PREP_TOTE_STOW_RIGHT')[-1][i]    


        #print 'test_config = ', test_config
        leftLink = self.robot.link('left_wrist')
        rightLink = self.robot.link('right_wrist')
        # print 'Current limb is', limb
        # print self.simworld.robot(0).link('right_wrist').getName(), self.robot.link('right_wrist')
        # print self.simworld.robot(0).link('left_wrist').getName(), self.robot.link('left_wrist')
        # print link.getName(), link
        # print 'link_xform', link.getTransform()
        rPath = []
        lPath = []

        # print 'beginning debugging'


        self.leftStowPoints = []
        self.rightStowPoints = []
        self.debugFailedLeftPoints = []
        self.debugFailedRightPoints = []

        #print 'about to do left arm testing'

        while goalZ >= endZ:
            global1 = [target[0], target[1], goalZ]
            global2 = [target[0],target[1], goalZ-0.5]

            goal  = ik.objective(leftLink,local=[local1,local2],world=[global1,global2])
            self.simworld.robot(0).setConfig([conf for conf in test_config])

            milestone = []
            milestone = self.simpleIK(goal = goal, limb = 'left')

            if milestone:
                #print milestone
                lPath.append(milestone)
                self.leftStowPoints.append(global1)
                print 'Goal Z = ', goalZ, ' solved at ', i

            else:
                self.debugFailedLeftPoints.append(global1)
                break
                #currently skips lower points if we fail for 1

            goalZ+=incZ

        #print 'about to do right arm testing'

        goalZ = startZ

        while goalZ >= endZ:
            global1 = [target[0], target[1], goalZ]
            global2 = [target[0],target[1], goalZ-0.5]

            goal  = ik.objective(rightLink,local=[local1,local2],world=[global1,global2])
            self.simworld.robot(0).setConfig([conf for conf in test_config])

            milestone = []
            milestone = self.simpleIK(goal = goal, limb = 'right')

            if milestone:
                #print milestone
                rPath.append(milestone)
                self.rightStowPoints.append(global1)
                print 'Goal Z = ', goalZ, ' solved at ', i

            else:
                self.debugFailedRightPoints.append(global1)
                break
                #currently skips lower points if we fail for 1


            goalZ+=incZ      

        #print 'lPath = ', lPath
        #print 'rPath = ', rPath

        #print 'about to choose data'

        #print 'z of right', self.rightStowPoints[-1][2]
        #print 'z of left', self.leftStowPoints[-1][2]
        #print 'len of right', len(self.rightStowPoints)
        #print 'len of left',len(self.leftStowPoints)
        #print TOTE_BOUNDS_Z_MAX

        #print self.rightStowPoints[-1][2] < TOTE_BOUNDS_Z_MAX
        #print self.leftStowPoints[-1][2] < TOTE_BOUNDS_Z_MAX
        #print (len(self.leftStowPoints) == len(self.rightStowPoints))



        if self.leftStowPoints == []:
            if self.rightStowPoints == [] or self.rightStowPoints[-1][2] > TOTE_BOUNDS_Z_MAX:
                return False
            else:
                return ['right', rPath]
        elif self.rightStowPoints == []:
            if self.leftStowPoints[-1][2] > TOTE_BOUNDS_Z_MAX:
                return False
            else:
                return ['left', lPath]
        elif (len(self.leftStowPoints) )> len(self.rightStowPoints) and self.leftStowPoints[-1][2] < TOTE_BOUNDS_Z_MAX:
            #return we want to move the left arm along the left path
            return ['left', lPath]
        elif (len(self.rightStowPoints) > len(self.leftStowPoints)) and self.rightStowPoints[-1][2] < TOTE_BOUNDS_Z_MAX:
            return ['right', rPath]
        elif (len(self.leftStowPoints) == len(self.rightStowPoints)) and self.rightStowPoints[-1][2] < TOTE_BOUNDS_Z_MAX:
            #random 
            randomChoice = random.random()
            if randomChoice >=0.5:
                return ['left', lPath]
            else:
                return ['right', rPath]
        else:
            return 'did not choose anything'

            return False


    def vacuumSearch(self, timeout=300):
        raise NotImplemented




    # Stowing Debug

    def isInTote(self, point):
        tote1 = TOTE_BOUNDS[0]
        tote2 = TOTE_BOUNDS[1]

        minX = min(tote1[0], tote2[0])
        maxX = max(tote1[0], tote2[0])
        minY = min(tote1[1], tote2[1])
        maxY = max(tote1[1], tote2[1])

        xVal = minX <= point[0] <=maxX
        yVal = minY <= point[1] <= maxY

        return xVal and yVal

    def isInBin(self, bin, point):

        #TODO - may need work

        minMax = knowledge.getGlobalBounds(bin)

        xVal = minMax[0][0] <= point[0] <=minMax[1][0]
        yVal = minMax[0][1] <= point[1] <= minMax[1][1]
        zVal = minMax[0][2]<= point[2] <= minMax[1][2]
        return xVal and yVal and zVal   

    #================================================
    # Process for picking
    '''TODO

    Recalibrate viewBinAction
    Calibrate IK_SEEDs for graspFromBinAction
    Create IK methods to go:
        Into bin
        Up/Down in Bin
        Sideways in bin
        Normal to given direction in bin (fixed x location)

    Methods to determine which paths should be taken
    Connect to vacuum
    Verify that something is attached to vaccum
    Verify that correct object is dropped in Tote
    Update Output files

    '''

    def runPickingTask(self):
        #
        startTime = time.clock()
        #print startTime
        endTime = time.clock()

        binQueue = []
        armQueue = []


        while (endTime - startTime > PICK_TIME and pickQueue is not []):

            runPickFromBin(bin = binQueue.pop(0), limb = binQueue.pop(0))
            endTime = time.clock()

    def runPickFromBin(self, bin, limb):

        self.moveToRestConfig()
        self.waitForMove()
        if self.viewBinAction(b=bin, limb = limb):
            self.waitForMove()
            if self.prepGraspFromBinAction(limb=limb):
                self.waitForMove()
                if self.graspFromBinAction(limb = limb):
                    self.waitForMove()
                    if self.placeInToteAction(limb=limb):
                        self.waitForMove()
                        return True
                    else:
                        print 'Error in placeInToteAction'
                        return False
                else:
                    #binQueue.append(eval('self.'+limb+'_bin'))
                    #armQueue.append(limb)
                    print 'Error in graspFromBinAction'
                    return False
            else:
                #binQueue.append(eval('self.'+limb+'_bin'))
                #armQueue.append(limb)
                print 'Error in prepGraspFromBinAction'
                return False
        else:
            #binQueue.append(eval('self.'+limb+'_bin'))
            #armQueue.append(limb)
            print 'Error in View Bin Action'
            return False

    def viewBinAction(self,b, limb, doPerception=True):
        self.waitForMove()

        if LOAD_PHYSICAL_TRAJECTORY:
            if limb is not None:
                if b in apc.bin_names:
                    path_name = 'CAMERA_TO_' + b.upper() + '_'+limb.upper()
                    #EX: 'CAMERA_TO_BIN_A_LEFT'
                    if self.moveArm(limb, statusConditional='ready', path_name=path_name, finalState='scan'):

                            self.waitForMove()

                            scan_name = 'Q_SCAN_'+b.upper() + '_'+limb.upper()

                            self.moveToOffset(limb, q_name = scan_name)                    

                            time.sleep(5)
                            
                            if REAL_CAMERA:
                                if doPerception:
                                    self.getPickPositionForPick(bin_letter=b[4], limb=limb)
                            
                            if limb == 'left':
                                self.left_bin = b
                            elif limb == 'right':
                                self.right_bin = b
                            return True
                else:
                    print "Error in moveArm (from viewBinAction)"
                    return False
            else:
                print 'Error in viewTote action can\'t move with no limb'
                return False

    def prepGraspFromBinAction(self, limb, b=None):

        if limb == 'left':
            b = self.left_bin
        elif limb == 'right':
            b = self.right_bin

        if LOAD_PHYSICAL_TRAJECTORY:
            if eval('self.'+limb+'_bin') in apc.bin_names:
                path_name = 'VIEW_TO_GRASP_' + b[4].upper() + '_' + limb.upper()
                #EX: 'GRASP_TO_STOW_B_RIGHT'
                path = eval(path_name)


                if self.moveArm(limb, statusConditional = 'scan', path_name=path_name, finalState = 'graspPrepped'):
                    self.waitForMove()

                    self.moveToOffset(limb, milestone=path[-1])
                    return True
                else:
                    print 'Error in moveArm (from prepGraspFromBinAction)'
                    return False
            else:
                print 'Error bin not valid'
                return False

    def graspFromBinAction(self, limb, bin=None):
        #Method to be called after we've scanned and know where we want to go to grasp
        
        #Ideally move to the bin we're aiming for unless perception tells us that we can't grasp it
        self.waitForMove()

        position = None
        if bin is None:
            bin = eval('self.'+limb+'_bin')
            print bin

        if bin in apc.bin_names:

            #we've moved to configuration
            #arm's state should be graspPrep or something like that

            #move to shelfOffset
            #self.moveToOffset(limb, milestone=eval(), finalState = 'grasp')
            # covered in self.move_from_scan_to_grasp

            graspDirection = ['up', [0,0,1]]
            #graspDirection[0] = direction
            #graspDirection[1] = normal in vector format

            try:
                graspDirection = perception.getGraspDirection
            except:
                print 'perception grasp direction not set up yet'

            print graspDirection[0]


            if graspDirection[0] == 'up':
                try:
                    position = perception.getPickPositionForPick()
                except:
                    print 'Error perception code for point not set up'

                if self.moveToObjectInBinFromTop(position=position, limb=limb, step=-1):

                    if limb == 'left':
                        self.stateLeft = 'grasp'
                    elif limb == 'right':
                        self.stateRight = 'grasp' 

                    return True

                else:
                    print 'Grasp Unsuccessful'
                    #TODO
                    return True
                    #return False
            elif graspDirection[0] =='side':
                try:
                    position = perception.getPickPositionForPick()
                except:
                    print 'Error perception code for point not set up'

                if self.moveToObjectInBinFromSide(position=position, limb=limb, step=-1):



                    if limb == 'left':
                        self.stateLeft = 'grasp'
                    elif limb == 'right':
                        self.stateRight = 'grasp' 

                    return True
                else: 
                    print 'Grasp Unsuccessful'
                    return False
            else:
                print 'Error, unprepared grasp direction'
                return False
        else:
            print 'Invalid bin', eval('self.'+limb+'_bin')

    def placeInToteAction(self, limb, b = None):
        #method to be called once we've grasped an object and wish to place it in the tote
        print "LOAD_PHYSICAL_TRAJECTORY", LOAD_PHYSICAL_TRAJECTORY


        self.waitForMove()
        if b == None:
            b = eval('self.'+limb+'_bin')

        if LOAD_PHYSICAL_TRAJECTORY:
            if eval('self.'+limb+'_bin') in apc.bin_names:
                path_name = 'GRASP_TO_STOW_' + b[4].upper() + '_' + limb.upper() + END_EFFECTOR
                #EX: 'GRASP_TO_STOW_B_RIGHT'
                stow_name = 'Q_STOW_' + limb.upper() + END_EFFECTOR
                if self.moveArm(limb, statusConditional = 'grasp', path_name=path_name, finalState = 'stow'):
                    self.waitForMove()
                    print "moveArm to", path_name

                    if self.moveArm(limb, statusConditional = 'stow', path_name = stow_name, finalState = 'stow'):
                        self.waitForMove()
                        print "moveArm to", stow_name

                        try:
                            time.sleep(2)
                            #wait for two seconds
                            #maybe forcewait until it's in stow state
                            turnOffVacuum(limb)
                        except:
                            print 'Error in vacuum Controller'
                            return True
                    else:
                        print 'Error in moving to stow location'
                else:
                    print 'Error in moveArm (from prepGraspFromBinAction)'
                    return False
                        #turn off vacuum
                        #check scale
                        #update output

                        # go to rest
            else:
                print 'Illegal Bin'
                return False

    #==============================================

    def graspAction(self):
        self.waitForMove()


        # for quick debugging perception
        self.current_bin = "center"
        self.stateRight = "ready"
        self.stateLeft = "holding"
        xPos = 0.33*0.5
        yPos = 0.435*0.5
        #run_perception_on_bin(knowledge, 'bin_A')
        self.held_object = knowledge.bin_contents['bin_A'].pop()        
        self.held_object.randRt = [ so3.rotation([0,1,0], random.uniform(0,math.pi/2)),
                                   vectorops.add([-0.165,0,-0.33], [xPos,0,yPos])]
        global binIndex
        binIndex = 0



        if self.current_bin != "center":
            print "Spatula is not located at the center bin"
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

            if self.move_to_grasp_object(self.held_object, step=1):
                self.waitForMove()

                self.move_to_grasp_object(self.held_object, step=2)
                self.waitForMove()

                # now close the gripper
                self.controller.commandGripper('right', [1])
                self.waitForMove()

                # New Version
                if PHYSICAL_SIMULATION:
                    val = 0
                    step = 3
                    while val==0:
                        with open("pressureReading.pkl", "rb") as f:
                            valList = pickle.load(f)
                            val = valList[0]
                            print "Vacuum sensor state:", val, "Pressure:", valList[1]
                        self.move_to_grasp_object(self.held_object, step=step)
                        self.waitForMove()
                        step += 1
                    print "GRASPED OBJECT"

                else:
                    # Old Version
                    for i in range(25):
                        self.move_to_grasp_object(self.held_object, step=i+2)
                        self.waitForMove()

                    # now close the gripper
                    self.controller.commandGripper('right', [0])
                    self.waitForMove()

                # self.held_object = knowledge.bin_contents[self.current_bin].pop()
                self.stateRight = 'holding'

                print "Holding object",self.held_object.info.name,"in right hand"
                return True
            else:
                print "Grasp failed"
                return False   

    def tilt_wrist(self,direction, step=0, ignoreColShelfSpatula = True):
        self.waitForMove()
        self.world.terrain(0).geometry().setCollisionMargin(0)
        #self.robot.setConfig(self.controller.getCommandedConfig())

        bin_name = self.current_bin
        world_center = knowledge.bin_front_center(bin_name)

        # Setup ik objectives for both arms
        # place +z in the +x axis, -y in the +z axis, and x in the -y axis
        left_goal = []
        incremental= False

        # tilted angle view for spatula
        if direction == 'down':
            #R_camera = so3.mul(knowledge.shelf_xform[0], so3.rotation([1,0,0], math.pi - math.pi/360*20))
            R_camera = so3.mul(knowledge.shelf_xform[0], so3.rotation([1,0,0], math.pi - math.pi/360*30))

            # [(+Right/-Left), (+Up/-Down), (+In/-Out)
            #world_offset = so3.apply( knowledge.shelf_xform[0],[-0.0275,0.095,0.4375])
            world_offset = so3.apply( knowledge.shelf_xform[0],[-0.0275,0.155,0.4225])

            t_camera = vectorops.add(world_center,world_offset)
            dist = vectorops.distance(self.left_camera_link.getTransform()[1], t_camera)
            if step == 0:
                left_goal.append(ik.objective(self.left_camera_link,R=R_camera,t=t_camera))
                maxSmoothIters=2
                incremental = True
            elif step == 1:
                # print "tilting down (tilt-wrist part 1)"
                left_goal.append(ik.objective(self.left_camera_link,R=R_camera,t=self.left_camera_link.getTransform()[1]))
                maxSmoothIters=0

            elif step == 2:
                # print "moving up/side (tilt-wrist part 2)"
                #left_goal.append(ik.objective(self.left_camera_link,R=R_camera,t=vectorops.add(world_center, so3.apply(knowledge.shelf_xform[0],[-0.0275,0.115,0.4675]))))
                left_goal.append(ik.objective(self.left_camera_link,R=R_camera,t=vectorops.add(world_center, so3.apply(knowledge.shelf_xform[0],[-0.0275,0.115+0.15,0.4675-0.02]))))
                # print "goal pos =", vectorops.add(world_center, so3.apply(knowledge.shelf_xform[0],[-0.0275,0.115+0.05,0.4675-0.02]))
                maxSmoothIters=3
                incremental = True

            elif step == 3:
                # print "tilting down (tilt-wrist part 3)"
                left_goal.append(ik.objective(self.left_camera_link,R=R_camera,t=t_camera))
                maxSmoothIters=5
                incremental = True

            elif step == 4:
                #R_camera = so3.mul(knowledge.shelf_xform[0], so3.rotation([1,0,0], math.pi - math.pi/360*20))
                R_camera = so3.mul(knowledge.shelf_xform[0], so3.rotation([1,0,0], math.pi - math.pi/360*10))

                # [(+Right/-Left), (+Up/-Down), (+In/-Out)
                #world_offset = so3.apply( knowledge.shelf_xform[0],[-0.0275,0.095,0.4375])
                world_offset = so3.apply( knowledge.shelf_xform[0],[-0.0275,0.055,0.4375])

                t_camera = vectorops.add(world_center,world_offset)
                dist = vectorops.distance(self.left_camera_link.getTransform()[1], t_camera)                                

                # print "tilting down (tilt-wrist part 3)"
                left_goal.append(ik.objective(self.left_camera_link,R=R_camera,t=t_camera))
                maxSmoothIters=1
                incremental = True


        elif direction == 'up':
            # method 1
            R_camera = so3.mul(knowledge.shelf_xform[0], so3.rotation([1,0,0], math.pi + math.pi/360*2*step))
            #world_offset = so3.apply( knowledge.shelf_xform[0],[-0.0275,0.015+0.05,0.4575])
            world_offset = so3.apply( knowledge.shelf_xform[0],[-0.0275,0.015+0.05,0.5175])

            t_camera = vectorops.add(world_center,world_offset)
            left_goal.append(ik.objective(self.left_camera_link,R=R_camera,t=t_camera))

            dist = (vectorops.distance(self.left_camera_link.getTransform()[1], t_camera))/10.0
            left_goal.append(ik.objective(self.left_camera_link, R=R_camera, t=t_camera))

            maxSmoothIters = 1
            ignoreColShelfSpatula = True

        limbs = ['left']
        qcmd = self.controller.getCommandedConfig()

        print "Solving for TILT_WRIST (", direction, step, ")"
        for i in range(500):
            sortedSolutions = self.get_ik_solutions([left_goal], limbs, qcmd, maxResults=100, maxIters=100,rangeVal=dist/1000)

            if len(sortedSolutions)==0:
                print "No Sorted Solutions"
                continue

            # prototyping hack: move straight to target
            if FAKE_SIMULATION:
                q = sortedSolutions[0][0]
                n = self.robot.numLinks()
                if len(q)<n:
                    q += [0.0]*(n-len(q))
                self.controller.setMilestone(q)
                return True

            # else, if we want to path plan
            numSol = 0
            for solution in sortedSolutions:
                numSol += 1
                # print numSol, "solutions planned out of", len(sortedSolutions)

                path = [qcmd, solution[0]]

                if path == 1 or path == 2 or path == False:
                    continue
                elif path != None:
                    self.sendPath(path, maxSmoothIters = maxSmoothIters, INCREMENTAL = incremental)
                    return True


        print "Failed to plan path"
        return False

    def move_spatula_to_center(self, ik_constrain=True):
        """Tilt the robot's wrist before and after actuating the spatula
        so that the objects are picked up.
        """
        self.waitForMove()

        print "Loading "+self.current_bin+"_spatula_to_center Trajectory..."
        path = loaded_trajectory[self.current_bin+"_spatula_to_center"]
        # print "loaded pathlength =", len(path)
        if FAKE_SIMULATION:
            q = path[-1]
            n = self.robot.numLinks()
            if len(q)<n:
                q += [0.0]*(n-len(q))
            self.controller.setMilestone(q)
        else:
            sortedSolutions = [loadFromFile("../IK_Solutions/"+self.current_bin), 0]
            qcmd = sortedSolutions[0][0]
            self.robot.setConfig(qcmd)
            # print "starting config", qcmd
            self.sendPathClosedLoop(path)
        self.current_bin = "center"
        return True

        # self.robot.setConfig(self.controller.getCommandedConfig())
        # self.world.terrain(0).geometry().setCollisionMargin(0.05)

        # R_wrist = so3.mul( so3.rotation([0,0,1], -math.pi), so3.rotation([1,0,0], -math.pi/2) )
        # t_wrist = knowledge.bin_center_point()

        # # Setup ik objectives for both arms
        # # place +z in the +x axis, -y in the +z axis, and x in the -y axis
        # left_goal = ik.objective(self.left_camera_link,R=R_wrist,t=t_wrist)

        # qcmd = self.controller.getCommandedConfig()
        # # qcmd = self.controller.getSensedConfig()
        # limbs = ['left']

        # ik_constraint = None
        # if ik_constrain:
        #     ik_constraint = IKObjective()
        #     ik_constraint.setLinks(55)
        #     ik_constraint.setAxialRotConstraint([-1,0,0], [0,0,1])

        # print "\nSolving for MOVE_SPATULA_TO_CENTER (Left Hand)"

        # for i in range(100):
        #     # if LOAD_IK_SOLUTIONS:
        #     #     sortedSolutions = [loadFromFile("../IK_Solutions/move_spatula_to_center"),0]
        #     # else:
        #     #     sortedSolutions = self.get_ik_solutions([left_goal], limbs, qcmd, maxResults=100, maxIters=100)
        #     # # sortedSolutions = [loadFromFile("../IK_Solutions/move_spatula_to_center"),0]

        #     # if len(sortedSolutions)==0:
        #     #     continue

        #     sortedSolutions = [[baxter_rest_config],0]

        #     # prototyping hack: move straight to target


        #     # else, if we want to path plan
        #     else:
        #         numSol = 0
        #         for solution in sortedSolutions:
        #             numSol += 1
        #             print numSol, "solutions planned out of", len(sortedSolutions)
        #             if ik_constraint==None:
        #                 path = self.planner.plan(qcmd,solution[0],'left')
        #             else:
        #                 path = self.planner.plan(qcmd,solution[0], 'left', iks = ik_constraint)
        #             if path == 1 or path == 2 or path == False:
        #                 break
        #             elif path != None:
        #                 if ik_constraint==None:
        #                     self.sendPath(path)
        #                 else:
        #                     self.sendPathClosedLoop(path)
        #                 self.current_bin = None
        #                 return True
        # print "Failed to plan path"
        # return False

    def move_gripper_to_center(self, FROM_ORDER_BIN=1):
        self.waitForMove()

        if FROM_ORDER_BIN:
            print "Loading gripper_to_center Trajectory..."
            path = loaded_trajectory["gripper_to_center"]
            if FAKE_SIMULATION:
                q = path[-1]
                n = self.robot.numLinks()
                if len(q)<n:
                    q += [0.0]*(n-len(q))
                self.controller.setMilestone(q)
            else:
                # make a copy of start position
                q0 = copy.deepcopy(path[0])

                # make a copy of goal position
                q1 = copy.deepcopy(path[-1])

                # change right shoulder joint so that the movement takes 2 steps
                q1[35] = -0.3

                # first step (move up)
                self.sendPath([q0, q1], maxSmoothIters = 1)
                self.waitForMove()

                # secpmd step (move left, above spatula)
                self.sendPath([q1, path[-1]], maxSmoothIters = 1)
            return True
        else:
        # if LOAD_TRAJECTORY and not FAKE_SIMULATION:
        #     print "Loading Trajectory..."
        #     path = loaded_trajectory['gripper_to_center']
        #     # print path
        #     self.sendPathClosedLoop(path)
        #     return True

        # self.robot.setConfig(self.controller.getCommandedConfig())

        # if self.stateLeft == 'holding':
        #     R_wrist = so3.mul(so3.rotation([1,0,0], math.pi), so3.rotation([0,0,1], -math.pi))
        #     t_wrist = knowledge.bin_vertical_point()

        #     # Setup ik objectives for both arms
        #     # place +z in the +x axis, -y in the +z axis, and x in the -y axis
        #     right_goal = ik.objective(self.right_gripper_link,R=R_wrist,t=t_wrist)

        #     qcmd = self.controller.getCommandedConfig()
        #     # qcmd = self.controller.getSensedConfig()
        #     limbs = ['right']

        #     print "\nSolving for MOVE_GRIPPER_TO_CENTER (Right Hand)"
        #     for i in range(100):

        #         # if LOAD_IK_SOLUTIONS:
            sortedSolutions = [loadFromFile("../IK_Solutions/gripper_to_center"),0]
        #         # else:
        #         #     sortedSolutions = self.get_ik_solutions([right_goal], limbs, qcmd, maxResults=100, maxIters=100)
        #         sortedSolutions = self.get_ik_solutions([right_goal], limbs, qcmd, maxResults=100, maxIters=100)

            if len(sortedSolutions)==0:
                print "Failed to plan path"
                return False
        #         # prototyping hack: move straight to target
            if FAKE_SIMULATION:
                q = sortedSolutions[0][0]
                n = self.robot.numLinks()
                if len(q)<n:
                    q += [0.0]*(n-len(q))
                self.controller.setMilestone(q)
                return True

            qcmd = self.controller.getCommandedConfig()
            numSol = 0
            for solution in sortedSolutions:
                numSol += 1
                print numSol, "solutions planned out of", len(sortedSolutions)
                path = self.planner.plan(qcmd,solution[0],'right')
                if path == 1 or path == 2 or path == False:
                    break
                elif path != None:
                    self.sendPath(path)
                    return True
            print "Failed to plan path"
            return False
        # else:
        #     print "no item in spatula"
        #     return False

    def placeInOrderBinAction(self):
        self.waitForMove()
        self.robot.setConfig(self.controller.getCommandedConfig())

        if self.stateRight != 'holding':
            print "Not holding an object"
        else:
            if self.move_gripper_to_center(FROM_ORDER_BIN=False):
                self.waitForMove()

                self.move_to_order_bin(self.held_object,step=1)
                self.waitForMove()

                self.move_to_order_bin(self.held_object,step=2)
                self.waitForMove()

                # now turn off vacuum
                time.sleep(5)
                self.controller.commandGripper('right',[0])
                self.waitForMove()
                time.sleep(3)

                knowledge.order_bin_contents.append(self.held_object)
                print "Successfully placed",self.held_object.info.name,"into order bin"
                self.drop_in_order_bin(self.held_object)
                self.active_grasp = None
                self.held_object = None
                # self.held_object.xform = None
                # self.held_object.bin_name = 'order_bin'
                self.stateRight = 'ready'
                self.stateLeft = 'ready'
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

    def fulfillOrderAction(self,objectList):
        """Given a list of objects to be put in the order bin, run
        until completed."""
        # go through all bins
        DEFAULT_LIMB = 'left'

        for b in apc.bin_names:
            # # if the bin is empty
            # if knowledge.bin_contents[b]==None:
                # try to view the bin
            if not self.viewBinAction(b,limb=DEFAULT_LIMB):
                print "Could not view bin",b
                continue
            # PICK STEPS
            self.saveCanonicalTotePointCloud(limb=DEFAULT_LIMB)
            self.preparePickBinAction(b,limb=DEFAULT_LIMB)
            # GET OBJECT POSITION FROM PERCEPTION
            # self.moveToObjectInBinFromTop(position,limb=DEFAULT_LIMB, step)

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

    def randomize_limb_position(self,limb,center=None, rangeVal=None):
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

        if rangeVal == None:
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
                    q[j] = random.uniform(max(qmin[j],center[j]-rangeVal),min(qmax[j],center[j]+rangeVal))
            else:
                for j in self.right_arm_indices:
                    q[j] = random.uniform(max(qmin[j],center[j]-rangeVal),min(qmax[j],center[j]+rangeVal))
            self.robot.setConfig(q)

        #R = self.left_camera_link.getTransform()[0]
        #t = self.left_camera_link.getTransform()[1]
        #self.frames.append((R,t))

        # for i in range(len(q)):
        #     inv = (so3.inv(self.robot.link(i).getTransform()[0]), 5)
        #     transpose = (self.robot.link(i).getTransform()[0], 5)
        #     for j in range(len(inv)):
        #         if inv[j] != transpose[j]:
        #             "print ****************"
        return

    def get_ik_solutions(self,goals,limbs,initialConfig=None,maxResults=10,maxIters=1000,tol=1e-3,validity_checker=None,printer=False,rangeVal=0.005):
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

        printer =True

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
            #print goal
            limb = limbs[index]
            numTrials[index] += 1
            # if first time trying the ik goal, initialize with current config
            if numTrials[index] == 1:
                self.robot.setConfig(initialConfig)
            # else, initialize with a random q, incrementally perturbing more from inital config
            else:
                self.randomize_limb_position(limb,center=initialConfig,rangeVal=rangeVal*(numTrials[index]-1))
                # self.randomize_limb_position(limb,center=initialConfig,range=None)
            #print self.robot.getConfig()
            if ik.solve(goal,tol=tol):
                numSolutions[index] += 1
                #print "validity checker(limb) = ",
                # print validity_checker(limb)

                print "Solved ", numSolutions[index]

                if validity_checker(limb):
                    numColFreeSolutions[index] += 1
                    ikSolutions.append((self.robot.getConfig(),index))
                    if len(ikSolutions) >= maxResults: break
            # a = raw_input("enter")
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
            # sortedSolutions = sorted([(vectorops.distanceSquared(solution[0],initialConfig),solution) for solution in ikSolution
            # Add initial joint values to additional joints
            n = len(solution[0])
            if len(initialConfig) < n:
                initialConfig += [0.0]*(n-len(initialConfig))
                print "# links in qcmd < # links in ik solution"

            dist = vectorops.distanceSquared(solution[0],initialConfig)
            config = solution[0]
            ind = solution[1]
            sortedSolutions.append( ((dist), (config, ind)) )


        sortedSolutions = sorted(sortedSolutions, key=itemgetter(0))

        # s[0] contains the distance-sqaured values
        # s[1] contains the ikSolution, which has [0]: config and [1]: index
        return [s[1] for s in sortedSolutions]


    #===================================================================================
    # New Movement Functions

    def move_camera_to_bin(self,bin_name, colMargin = 0.05, ik_constrain=True,ignoreColShelfSpatula=True, LOAD_TRAJECTORY=LOAD_TRAJECTORY_DEFAULT, limb=None):
        if LOAD_TRAJECTORY:
            print "Loading "+bin_name+" Trajectory..."
            path = loaded_trajectory[bin_name]
            if FAKE_SIMULATION:
                q = path[-1]
                n = self.robot.numLinks()
                if len(q)<n:
                    q += [0.0]*(n-len(q))
                self.controller.setMilestone(q)
            else:
                self.sendPathClosedLoop(path, clearRightArm=True)
                #if self.stateLeft == 'ready' and self.held_object == None:
                #    self.sendPath(path, clearRightArm=True)
                #else:
                #    self.sendPathClosedLoop(path, clearRightArm=True)
            return True

        if LOAD_PHYSICAL_TRAJECTORY:
            print "loading "+bin_name+" Physically Planned Trajectory"
            if(limb is not None):
                #path = LOADED_PHYSICAL_TRAJECTORY['CAMERA_TO_' + bin_name+'_' + limb.toUpper())]
                camera_path = 'CAMERA_TO_'+ bin_name.upper()+'_'+limb.upper()
                scan_path = 'Q_SCAN_' + bin_name.upper()+'_'+limb.upper()
            
                if limb=='left':
                    self.left_bin = bin_name
                if limb == 'right':
                    self.right_bin = bin_name


            if PHYSICAL_SIMULATION:

                #splitting things up into two motions isn't working
                self.moveArm(limb=limb, path_name=camera_path)
                if SHELF_STATIONARY:
                    self.moveArm(limb=limb, path_name=scan_path, finalState = 'scan')
                else:
                    self.moveToOffset(limb=limb, q_name=scan_path, finalState='scan')
                    #self.moveToOffset2(limb=limb, bin_name=bin_name, finalState = 'scan')


        # If we are backing off from bin to view camera
        else:
            print "Loading "+bin_name+" IK Solution..."

            ik_constraint = None
            sortedSolutions = [loadFromFile("../IK_Solutions/"+bin_name), 0]

            if len(sortedSolutions)==0:
                print "Failed to plan path"
                return False

            # prototyping hack: move straight to target
            if FAKE_SIMULATION:
                q = sortedSolutions[0][0]
                n = self.robot.numLinks()
                if len(q)<n:
                    q += [0.0]*(n-len(q))
                self.controller.setMilestone(q)
                return True

            qcmd = self.controller.getCommandedConfig()
            numSol = 0
            for solution in sortedSolutions:
                numSol += 1
                path = [qcmd, solution[0]]
                if path == 1 or path == 2 or path == False:
                    break
                elif path != None:
                    if ik_constraint==None:
                        self.sendPath(path, clearRightArm=True)
                    else:
                        self.sendPathClosedLoop(path)
                    return True
        print "Failed to plan path"
        return False

    def move_from_scan_to_grasp(self, limb, reverse=False):
        if limb is None:
            return

        bin = eval('self.'+limb+'_bin')[4]

        if LOAD_PHYSICAL_TRAJECTORY:
            print "loading "+bin_name+" Physically Planned Trajectory"
            
            grasp_path = 'VIEW_TO_GRASP_'+ bin.upper()+'_'+limb.upper()

            if PHYSICAL_SIMULATION:

                if not reverse:
                    self.moveArm(statusConditional = 'scan', limb=limb, path = eval(grasp_path)[0:-1], finalState='graspMove')
                    # the above is the first to the second from last position
                    #self.moveArm(statusConditional='scan', limb=limb, path_name=grasp_path, finalState='graspMove')
                    # the above is the first to the last position
                    self.moveToOffset(limb=limb, milestone=eval(grasp_path)[-1], finalState='graspPrep')

                else:
                    self.moveArm(limb = limb, path_name = grasp_path, reverse = True)

    def move_from_grasp_to_stow(self, limb, reverse=False):
        if limb is None:
            return

        bin = eval('self.'+limb+'_bin')[4]

        if LOAD_PHYSICAL_TRAJECTORY:
            print "loading "+bin_name+" Physically Planned Trajectory"
            
            stow_path = 'GRASP_TO_STOW_'+ bin.upper()+'_'+limb.upper()

            if PHYSICAL_SIMULATION:

                if not reverse:
                    self.moveArm(statusConditional = 'grasp', limb=limb, path_name = stow_path, finalState='stow')
                else:
                    self.moveArm(limb=limb, path_name=stow_path, reverse=True)

    def move_from_rest_to_grasp(self, limb, reverse=False):
        print 'Error move_from_rest_to_grasp is not implemented'
        return False


    #============================================================================================================================================
    #The above methods are somewhat clunky - consider deprecating

    # Streamlined movement functions

    def moveToOffset(self, limb=None, statusConditional=None, q_name=None, milestone=None, finalState=None):

        # print 'Last position\'s transform is: ',self.simworld.robot(0).link(limb+'_wrist').getTransform()   

        #self.frames.append(self.simworld.robot(0).link(limb+'_wrist').getTransform())

        assert (q_name is not None) or (milestone is not None), 'Either q_name or milestone must not be None'

        self.waitForMove()
        q_start = [conf for conf in self.simworld.robot(0).getConfig()]
        #timing issues?


        q_canonical = [q for q in q_start]

        if q_name is not None:
            if milestone is None:
                milestone = eval(q_name)

        if limb == 'left' and milestone is not None:

            for i in range(7):
                q_canonical[self.left_arm_indices[i]] = milestone[i]


        elif limb == 'right' and milestone is not None:

            for i in range(7):
                q_canonical[self.right_arm_indices[i]] = milestone[i]

        else:
            return False


        # print q_start
        # print q_canonical

        self.simworld.robot(0).setConfig([conf for conf in q_canonical])

        curEndEffectorTransform = self.simworld.robot(0).link(limb+'_wrist').getTransform()
        # curEndEffectorTransform = self.simworld.robot(0).link(41).getTransform()

        # if transformType == 'vacuum':
        #     curEndEffectorTransform = se3.mul(self.simworld.robot(0).link(limb+'_wrist').getTransform(), self.vacuumTransform)
        # elif transformType == 'camera'
        #     curEndEffectorTransform = se3.mul(self.simworld.robot(0).link(limb+'_wrist').getTransform(), self.cameraTransform)
        # else:
        #     return False

        #print 'Current Tranaform is ', curEndEffectorTransform
 
        self.frames.append(curEndEffectorTransform)

        newEndEffectorTransform = se3.mul( self.perceptionTransform, curEndEffectorTransform)
        #print 'New Transform is ', newEndEffectorTransform

        self.frames.append(newEndEffectorTransform)

        #q_start is initial position - found by hand

        # goal = ik.objective(self.simworld.robot(0).link(limb+'_wrist'), R=newEndEffectorTransform[0], t=newEndEffectorTransform[1])
        p1_world = se3.apply(newEndEffectorTransform, [1,0,0])
        p2_world = se3.apply(newEndEffectorTransform, [0,1,0])
        p3_world = se3.apply(newEndEffectorTransform, [0,0,1])
        #goal = ik.objective(self.simworld.robot(0).link(limb+'_wrist'), local=[[1,0,0],[0,1,0],[0,0,1]], world=[p1_world, p2_world, p3_world])
        goal = ik.objective(self.robot.link(limb+'_wrist'), local=[[1,0,0], [0,1,0], [0,0,1]], world = [p1_world, p2_world, p3_world])


        path = [q_canonical]
        #path = [q_start]

        q_next = self.simpleIK(q_seed = q_canonical, limb = limb, goal=goal)

        #print 'q_next is : ', q_next, 'this is the value returned from simpleIK'

        if q_next:
            path.append(q_next)
        else:
            print 'ik failed in moveToOffset'
            return False
       


        self.frames.append(self.simworld.robot(0).link(limb+'_wrist').getTransform())

        self.sendPath(path=path, limb=limb)

        return True
 
    def moveToObjectInBinFromTop(self, position=None, limb=None, step=None, naiive=True):
        #Assumes we have moved so that we are in a configuration where we are ready to pick up things from a bin
        #Assumes we have scanned the bin already and determined the x,y,z of where we want to move
        #Assumes we have determined we want to pick up the object from above

        #We need to calculate the shelf normal
        #We want to aim into the shelf with the suction cup down and enter with the wrist pointing in the direction of the normal of the shelf

        #ik_constraint = IKObjective()
        #ik_constraint.setLinks(self.simworld.robot(0).link(limb+'_wrist'))
        #ik_constraint.setAxialRotConstraint([0,0,1],knowledge.getShelfNormal())
        

        # want the forward axis of the wrist to be constrained to normal to the shelf
        #forward axis of the wrist is +z
        

        #p1_world = se3.apply(newEndEffectorTransform, [1,0,0])
        #p2_world = se3.apply(newEndEffectorTransform, [0,1,0])
        #p3_world = se3.apply(newEndEffectorTransform, [0,0,1])

        #DEFAULT_GOAL = [1.1151453560415345, -0.046864004769163026, 1.1370113707939946]
        DEFAULT_GOAL = [1.2091107903500762, -0.022612772106925652, 1.5419692648895018]
        DEFAULT_NORMAL = [0, 1, 1] #45 degree angle

        q_seed = [i for i in self.simworld.robot(0).getConfig()]

        step1 = []
        step2 = []
        step3 = []
        step4 = []
        step5 = []

        #target = []
        if position != None:
            self.pick_pick_pos = position
        # print 'physical simulation is off'
        # PHYSICAL_SIMULATION = 0

        if naiive:
            #fix the suction cup's direction aim for the top of the bin, ik to a position slightly above the object
            #ik down
            #
            try:
                target = self.pick_pick_pos
                assert self.pick_pick_pos is not None, 'Perception failed, falling back to DEFAULT'
            except:
                myInput = None

                if SKIP_STOWING_INPUT:  
                    self.pick_pick_pos = knowledge.getBinWorldPosition(bin_name=eval('self.'+limb+'_bin'), localPoint=[.5,.5,.5])
                else:
                    while(1):
                        print 'Enter a position of where you would like to pick - defaults to world'
                        print 'If you would like to pick a location local to a bin, enter: l, x,y,z '
                        print 'Where x, y, and z are floating point numbers < 1 that refer to a ratio related to the bin'
                        print 'I.E. l, 0.5,0.5,0.5 picks the midde of the given bin'

                        myInput = raw_input("So, where would you like to pick? (comma delimited) ")
                        myInput = myInput.split(',')
                        
                        goodInput = False

                        if myInput[0] =='l' and len(myInput) ==4:
                            myInts = []
                            for i in range(1, 4):
                                try:
                                    myInts.append(float(myInput[i]))
                                    goodInput = True
                                except:
                                    goodInput = False
                            if goodInput:
                                self.pick_pick_pos = knowledge.getBinWorldPosition(bin_name=eval('self.'+limb+'_bin'), localPoint=myInts)
                        else:  

                            myInts = []
                            for el in myInput:
                                try:
                                    myInts.append(float(el))
                                    goodInput = True
                                except:
                                    goodInput = False
                            #if not knowledge.getBinWorldPosition(point=target):
                            #we failed
                            #    target = None
                            if goodInput:
                                self.pick_pick_pos = myInts
                        
                        if goodInput:
                            response = raw_input("Continue? y/n ")
                            if response == 'y':
                                break
                            else:
                                myInput = None
                                self.pick_pick_pos = None

            target = self.pick_pick_pos
            self.pick_pick_pos = None

            inv_perturb_R, inv_perturb_t = se3.inv(self.perceptionTransform)

            checkPoint = se3.apply([inv_perturb_R, inv_perturb_t], target)

            step = 1;

            bin = None
            if limb =='left':
                bin = self.left_bin
            elif limb =='right':
                bin = self.right_bin
                
            if not self.isInBin(bin=bin, point=checkPoint):
                print 'Error, point not in ', bin
                return False 

            if step==1:
                #move to the top center of the bin
                target1 = knowledge.getBinFrontCenterTop(bin)
                ikGoal = self.buildIKGoalSuctionDown(limb = limb, target=target1)
                #ik to top center of bin, normal to the shelf
                #use ik seed and knowledge of shelf
                # constraintst: suction cup down, vacuum/wrist forward direction in direction of shelf
                #print "Target position =", target
                #print 'trying step1 ik'
                step1 = self.simpleIK(limb=limb, goal=ikGoal)
                if step1 is None:
                    print 'ik failed in step1 of moveToObjectInBinFromTop'
                    return False
                self.debugPoints.append(target1)
                self.sendPath(path=[step1], limb=limb)
                #self.sendPath(path=step1, limb = limb)
                self.waitForMove()

                step =3

            # if step==2:
            #     #move towards the object
            #     # check to make sure point is within the
            #     target = knowledge.getBinMidCenterTop(bin)
            #     self.debugPoints.append(target)
            #     ikGoal = self.buildIKGoalSuctionDown(limb = limb, target=target)
            #     step2 = self.simpleIK(limb=limb, goal=ikGoal)
            #     #ik to top center of bin, normal to the shelf
            #     #use ik seed and knowledge of shelf
            #     # constraintst: suction cup down, vacuum/wrist forward direction in direction of shelf
            #     print 'trying step2 ik'
            #     self.sendPath(path=step2, limb=limb)
            #     time.sleep(2)
            #     step = 3

            if step==3:
                #keep the x, y - throw out the z

                #check to make sure target is actually in bin
                target3 = knowledge.getBinMidCenterTop(bin)
                target3[0] = target[0]
                target3[1] = target[1]
                ikGoal = self.buildIKGoalSuctionDown(limb = limb, target=target3)
                #ik to top center of bin, normal to the shelf
                #use ik seed and knowledge of shelf
                # constraintst: suction cup down, vacuum/wrist forward direction in direction of shelf
                #print 'trying step3 ik'
                step3 = self.simpleIK(limb=limb, goal=ikGoal)
                if step3 is None:
                    print 'ik failed in step3 of moveToObjectInBinFromTop'
                    return False

                self.debugPoints.append(target3)
                self.sendPath(path=[step3], limb=limb)
                self.waitForMove()
                step=4

            if step==4 :           
                #turn on vacuum and move down
                try:
                    turnOnVacuum(limb)
                except:
                    print 'Error in vacuum Controller'
                #keep the x, y - throw out the z
                target4 = target
                #check to make sure target is actually in bin
                ikGoal = self.buildIKGoalSuctionDown(limb = limb, target=target4)
                #ik to top center of bin, normal to the shelf
                #use ik seed and knowledge of shelf
                # constraintst: suction cup down, vacuum/wrist forward direction in direction of shelf
                #print 'trying step4 ik'
                step4 = self.simpleIK(limb=limb, goal=ikGoal)
                if step4 is None:
                    print 'ik failed in step4 of moveToObjectInBinFromTop'
                    self.sendPath(path=[step1], limb=limb)
                    return False
                self.debugPoints.append(target4)
                self.sendPath(path=[step4], limb = limb)
                self.waitForMove()
                #print 'Got to step 4'

                step = 5

            if step==5:
                #pull back out

                self.waitForMove()
                self.sendPath(path=[step3], limb=limb)
                self.waitForMove()
                self.sendPath(path=[step1], limb=limb)

                #check that object is still held
                return True

        else:
           
            if step == 1:
                pass

            elif step==2:
                #step 2: enter along vacuum/wrist axis until far enough in
                pass
            elif step==3:
                #step 3: move along y direction to get above object
                pass

            elif step==4:
                pass
                #turn vacuum on
                #attempt to move down to half the object's height from the ground

            elif step==5:
                pass
                #check to make sure we have sucked something and that it is not the shelf
                #throw in various other checks here

            elif step==6: 
                pass
                #take the reverse path back out of the bins - something is in front of the front of the shelf

            #get back out from 

        # print 'physical simulation is on'
        # PHYSICAL_SIMULATION = 1

    def moveToObjectInBinFromSide(self, position, limb, step, naiive = True):
        #Assumes we have moved so that we are in a configuration where we are ready to pick up things from a bin
        #Assumes we have scanned the bin already and determined the x,y,z of where we want to move
        #Assumes we have determined we want to pick up the object from above

        #We need to calculate the shelf normal
        #We want to aim into the shelf with the suction cup down and enter with the wrist pointing in the direction of the normal of the shelf


        DEFAULT_GOAL = [1.1151453560415345, -0.046864004769163026, 1.1370113707939946]
        DEFAULT_NORMAL = [0, 1, 1] #45 degree angle

        target = []
        normal = []

        step1 =[]
        step2 = []
        step3 = []
        step4 = []
        step5 = []

        if naiive:
            #fix the suction cup's direction aim for the top of the bin, ik to a position slightly above the object
            #ik down
            #

            target = []
            normal = []
            try:
                target = self.pick_pick_pos
                assert self.pick_pick_pos is not None, 'Perception failed, falling back to DEFAULT'
            except:
                print 'perception not set up yet'
                target = DEFAULT_GOAL


            try:
                target = perception.getNormal()
            except:
                print 'perception normal not set up yet'
                normal = DEFAULT_NORMAL


            #step 1 move to the top middle of the front face of the given bin
            #step 2 move in as far as the target indicates (rotation about x)
            #   -i.e. given point is x,y,z keep z and y the same as before but move in a given x distance
            #step 3 get the normal and compute a point that is 5cm away - move - rotate the suction cup while doing this
            #   note: 1 = 1m so 5cm = 0.05
            #step 4 move to the actual point utilizing the normal to position the suction cup


            step = 1

            bin = 'bin_H'
            if limb =='left':
                #bin = self.left_bin
                pass
            elif limb =='right':
                #bin = self.right_bin
                pass

            if step==1:
                #move to the top center of the bin
                target = knowledge.getBinFrontCenterTop(bin)
                self.debugPoints.append(target)
                ikGoal = self.buildIKGoalSuctionDown(limb = limb, target=target)
                #ik to top center of bin, normal to the shelf
                #use ik seed and knowledge of shelf
                # constraintst: suction cup down, vacuum/wrist forward direction in direction of shelf
                print target
                print 'trying step1 ik'
                step1 = self.simpleIK(limb=limb, goal=ikGoal)
                self.sendPath(path=step1, limb=limb)
                time.sleep(2)

                step =2

            if step==2:
                target2 = knowledge.getBinFrontCenterTop(bin)
                self.debugPoints.append(target)

                #dummy = knowledge.applyShelfXform([target[0], 0, 0])
                dummy = se3.mul(self.percetionTransform, [target[0], 0, 0])

                target2[0] = dummy[0]

                #target should be the middle of the bin, near the top but only so far in as would be indicated by x

                ikGoal = self.buildIKGoalSuctionDown(limb = limb, target=target2)
                #ik to top center of bin, normal to the shelf
                #use ik seed and knowledge of shelf
                # constraintst: suction cup down, vacuum/wrist forward direction in direction of shelf
                print target
                print 'trying step2 ik'
                step1 = self.simpleIK(limb=limb, goal=ikGoal)
                self.sendPath(path=[step1], limb=limb)
                time.sleep(2)

                step =3


            if step==3:
                #keep the x, y - throw out the z

                #check to make sure target is actually in bin

                target3 = target

                self.debugPoints.append(vectorops.add(target, vectorops.mul(vectorops.unit(normal), 0.05)))
                ikGoal = self.buildIKGoalSuctionNormal(limb = limb, target=target3, normal = vectorops.unit(normal), normalDisplacement = 0.05)
                print 'trying step3 ik'
                step3 = self.simpleIK(limb=limb, goal=ikGoal)
                self.sendPath(path=[step3], limb=limb)
                time.sleep(2)
                step=4

            if step==4 :           
                #turn on vacuum and move down
                #keep the x, y - throw out the z
                #check to make sure target is actually in bin
                try:
                    turnOnVacuum(limb)
                except:
                    print 'Error in vacuum Controller'

                target4= target
                ikGoal = self.buildIKGoalSuctionNormal(limb = limb, target=target4, normal = klampt.vectorops.unit(normal), normalDisplacement = 0)
                #ik to top center of bin, normal to the shelf
                #use ik seed and knowledge of shelf
                # constraintst: suction cup down, vacuum/wrist forward direction in direction of shelf

                print 'trying step4 ik'
                step4 = self.simpleIK(limb=limb, goal=ikGoal)

                self.sendPath(path=[step4], limb = limb)

                print 'Got to step 4'

                step = 5

            if step ==5:
                self.sendPath(path = [step3], limb=limb)
                self.sendPath(path = [step2], limb = limb)
                self.sendPath(path=[step1], limb=limb)
                #check that object is still held

                return True

        else:
            if step==1:
                #ik to top center of bin, normal to the shelf
                #use ik seed and knowledge of shelf
                # constraintst: suction cup down, vacuum/wrist forward direction in direction of shelf
                pass
            elif step==2:
                #step 2: enter along vacuum/wrist axis until far enough in
                pass
            elif step==3:
                #step 3: rotate suction cup so that it moves towards the normal

                #if we collide with something, move out, down and then back in (push the object away)
                pass
            elif step==4:
                #move until you intersect the normal
                pass
            elif step==5:
                #step5: roate suction to be normal to object
                pass
            elif step==6:
                #turn vacuum on
                #attempt to move down to half the object's height from the ground
                pass
            elif step==7:
                #check to make sure we have sucked something and that it is not the shelf
                #throw in various other checks here
                pass
            elif step==8: 
                pass
                #take the reverse path back out of the bin


        #potential issues - something is in front of the front of the shelf

    def moveToRestConfig(self, limb='both'):
        print "Moving to rest config..."

        baxter_startup_config = self.robot.getConfig()

        if LOAD_PHYSICAL_TRAJECTORY:

            if limb =='both':

                # print 'moving to left'
                # self.moveToLeftRest()
                # print 'moving to right'
                # self.moveToRightRest()
                pathL = self.getLeftRestPath()
                pathR = self.getRightRestPath()

                self.moveBothArms(pathL =pathL,pathR = pathR, finalState = 'ready', INCREMENTAL=True)

                self.left_bin=None
                self.right_bin = None

            elif limb == 'left':
                self.moveToLeftRest()

            elif limb == 'right':
                self.moveToRightRest()

            self.waitForMove()

        else:
            path = [baxter_startup_config, baxter_rest_config]
            #self.sendPath(path)

            self.controller.setMilestone(baxter_rest_config)
            self.waitForMove()
        #print "Done"

    def moveToLeftRest(self):
        if(self.stateLeft == 'scan'):
            lPath = eval('CAMERA_TO_'+ self.left_bin.upper()+'_LEFT')[::-1]
            #rPath = eval('CAMERA_TO_'+self.right_bin+'_RIGHT')[::-1]
            self.moveLeftArm(path =lPath)

      
        elif(self.stateLeft == 'grasp' ):
            #not set up yet
            # go to store then rest
            lPath = eval('GRASP_TO_STOW_' + self.left_bin.upper()[4]+'_LEFT' + END_EFFECTOR)
            self.moveLeftArm(path = lPath)

            return False

        elif self.stateLeft == 'stow':
            #go right ahead through the code
            pass

        else:
            #not set up yet
            #find the nearest milestone and follow the path back
            pass

        self.moveLeftArm(path_name = 'Q_DEFAULT_LEFT', finalState = 'ready')
        self.left_bin = None

    def getLeftRestPath(self):
        path =  [[self.simworld.robot(0).getConfig()[v] for v in self.left_arm_indices]]

        if(self.stateLeft == 'scan'):
            lPath = eval('CAMERA_TO_'+ self.left_bin.upper()+'_LEFT')[::-1]
            #rPath = eval('CAMERA_TO_'+self.right_bin+'_RIGHT')[::-1]

            if lPath !=[]:
                for milestone in lPath:
                    path.append(milestone)
      
        elif(self.stateLeft == 'grasp' or self.stateLeft == 'graspPrepped' ):
            #not set up yet
            # go to store then rest
            lPath = eval('GRASP_TO_STOW_' + self.left_bin.upper()[4]+'_LEFT'+END_EFFECTOR)

            if lPath !=[]:
                for milestone in lPath:
                    path.append(milestone)

            path.append(eval('Q_OUT_OF_TOTE_LEFT'))

        elif self.stateLeft == 'stow':
            #go right ahead through the code

            path.append(eval('Q_OUT_OF_TOTE_LEFT'))
    

        elif self.stateLeft == 'viewTote':
            lPath = eval('Q_VIEW_TOTE_LEFT')
            try:
                len(lPath[0])
            except:
                lPath = [lPath]
            lPath = lPath[::-1]
            if lPath != []:
                for milestone in lPath:
                    path.append(milestone)


        else:
            #not set up yet
            #find the nearest milestone and follow the path back
            pass

        path.append(eval('Q_DEFAULT_LEFT'))
        self.left_bin = None

        return path

    def moveToRightRest(self):
        if(self.stateRight == 'scan'):
            rPath = eval('CAMERA_TO_'+ self.right_bin.upper()+'_RIGHT')[::-1]
            self.moveRightArm(path=rPath)

      
        elif(self.stateRight == 'grasp' or self.stateRight == 'toteStowInBin'):
            #not set up yet
            # go to store then rest
            rPath = eval('GRASP_TO_STOW_' + self.right_bin.upper()+'_RIGHT'+END_EFFECTOR)
            self.moveRightArm(path=rPath)
            #sendPath(path = rPath, )
            #self.moveRightArm()

            return False

        elif self.stateRight == 'stow':
            #go right ahead through the code
            pass

        else:
            #not set up yet
            #find the nearest milestone and follow the path back
            pass

        self.moveRightArm(path_name = 'Q_DEFAULT_RIGHT', finalState = 'ready')
        self.right_bin = None

    def getRightRestPath(self):
        path = [[self.simworld.robot(0).getConfig()[v] for v in self.right_arm_indices]]

        if(self.stateRight == 'scan'):
            rPath = eval('CAMERA_TO_'+ self.right_bin.upper()+'_RIGHT')[::-1]
            #rPath = eval('CAMERA_TO_'+self.right_bin+'_RIGHT')[::-1]
            if rPath !=[]:
                for milestone in rPath:
                    path.append(milestone)
      
        elif(self.stateRight == 'grasp' or self.stateRight == 'graspPrepped'):
            #not set up yet
            # go to store then rest
            rPath = eval('GRASP_TO_STOW_' + self.right_bin.upper()[4]+'_RIGHT'+END_EFFECTOR)
            if rPath != []:
                for milestone in rPath:
                    path.append(milestone)

            path.append(eval('Q_OUT_OF_TOTE_RIGHT'))

        elif self.stateRight == 'stow':
            #go right ahead through the code
            path.append(eval('Q_OUT_OF_TOTE_RIGHT'))

        elif self.stateRight == 'viewTote':
            rPath = eval('Q_VIEW_TOTE_RIGHT')
            try:
                len(rPath[0])
            except:
                rPath = [rPath]
            rPath = rPath[::-1]
            if rPath != []:
                for milestone in rPath:
                    path.append(milestone)

        else:
            #not set up yet
            #find the nearest milestone and follow the path back
            pass

        path.append(eval('Q_DEFAULT_RIGHT'))
        self.right_bin = None

        return path

    def moveArm(self, limb, statusConditional=None, path_name=None, path=None, finalState=None, reverse=False):
        if limb == 'left':
            if self.moveLeftArm(statusConditional, path_name, path, finalState, reverse):
                return True
        if limb == 'right':
            if self.moveRightArm(statusConditional, path_name, path, finalState, reverse):
                return True
        if limb =='both':
            path_nameL = path_name
            path_nameR = path_name
            reverseL = reverse
            reverseR = reverse
            pathL = path
            pathR = path
            if self.moveBothArms(statusConditional, path_nameL, pathL, path_nameR, pathR, finalState, reverseL, reverseR):
                pass
        #wasn't able to move
        print 'Error with move'+limb+'arm'
        return False

    def moveArmAway(self, limb):
        if limb == 'left':
            self.moveLeftArm(path_name = 'Q_AWAY_LEFT', finalState = 'away')
        else:
            self.moveRightArm(path_name = 'Q_AWAY_RIGHT', finalState = 'away')

    def moveBothArms(self, statusConditional=None, path_nameL=None, pathL=None, path_nameR=None, pathR = None, finalState=None, reverseL=False, reverseR=False, INCREMENTAL=True):

        dummyConfig = [0.0]*self.robot.numLinks()

        if pathL is None:
                try:
                    pathL = eval(path_nameL)
                except:
                    print 'Error no '+path_nameL+'in recorded constants'
                    return False
        if pathR is None:
                try:
                    pathR = eval(path_nameR)
                except:
                    print 'Error no '+path_nameR+'in recorded constants'
                    return False



        try:
            len(pathL[0])
        except:
            #path is empty
            if pathL == []:
                print "empty path"
                if finalState is not None:
                    self.stateLeft = finalState
                    self.stateRight = finalState
                return True
            #path is a single milestone
            pathL=[pathL]

        try:
            len(pathR[0])
        except:
            #path is empty
            if pathR == []:
                print "empty path"
                if finalState is not None:
                    self.stateLeft = finalState
                    self.stateRight = finalState
                return            
            #path is a single milestone                
            pathR=[pathR]

        #print 'pathR before appends = ',pathR

        while len(pathL) < len(pathR):
            pathL.append(pathL[-1])
            
    
        while len(pathR) < len(pathL):
            pathR.append(pathR[-1])

        #print 'pathR after appends =', pathR

        realPath = []

        realConfig = [v for v in dummyConfig]
        for j in range(len(pathL)):
            for i in range(len(self.left_arm_indices)):
                realConfig[self.left_arm_indices[i]] = pathL[j][i]
            for i in range(len(self.right_arm_indices)):
                realConfig[self.right_arm_indices[i]] = pathR[j][i]                
            realPath.append([v for v in realConfig])

        #print "realPath =", realPath
        self.sendPath(path = realPath, INCREMENTAL=INCREMENTAL)

        if finalState is not None:
            self.stateLeft = finalState
            self.stateRight = finalState


    def moveLeftArm(self, statusConditional=None, path_name=None, path=None, finalState=None, reverse=False):
        if(self.stateLeft == statusConditional or statusConditional == None or self.stateLeft in statusConditional):
            if path is None:
                try:
                    path = eval(path_name)
                except:
                    print 'Error no '+path_name+'in recorded constants'
                    return False


            #for milestone in path:
            try:
                len(path[0])
            except:
                #path is a single milestone
                if path ==[]:
                    if finalState is not None:
                        self.stateLeft = finalState
                    return True

                print 'Hit path exception for single milestones or empty paths'
                path=[path]
                path.append(path[0])
                qAdd = None
                if self.q_most_recent_left is None:
                    # qAdd = motion.robot.getKlamptSensedPosition()
                    qAdd = self.controller.getSensedConfig()
                    qAdd = [qAdd[v] for v in self.left_arm_indices[:7]]
                else: 
                    qAdd = self.q_most_recent_left
                path[0]=qAdd
                #print path

            #print 'sending path ', path

            if reverse:
                path = path[::-1]

            #if len(path)>=1:
            #    self.sendPath(path, limb='left', readConstants=True)

            for milestone in path:
                self.controller.appendMilestoneLeft(milestone, 1)
                #move to the milestone in 1 second

                time.sleep(0.1)
                self.waitForMove() #still doesn't do anything, but it's the thought that counts
                if FORCE_WAIT:
                    self.forceWait(milestone, self.left_arm_indices, 0.01)
                else:
                    self.controller.appendMilestoneLeft(milestone, WAIT_TIME)
                #wait at the milestone for 2 seconds
                #later should replace with Hyunsoo's code setting milestones if dt is too large

            self.q_most_recent_left = path[-1]

            if finalState is not None:
                self.stateLeft = finalState
            return True
        else:
            print "Error, arm is not in state "+ statusConditional
            return False

    def moveRightArm(self, statusConditional=None, path_name=None, path=None, finalState=None, reverse=False):
        if(self.stateRight == statusConditional or statusConditional == None):# or self.stateRight in statusConditional):
            if path is None:
                try:
                    path = eval(path_name)
                except:
                    print 'Error no '+path_name+'in recorded constants'
                    return False


            #for milestone in path:
            try:
                len(path[0])
            except:
                #path is a single milestone
                if path == []:
                    if finalState is not None:
                        self.stateRight = finalState
                    return True

                print 'Hit path exception for single milestones or empty paths'
                path=[path]
                path.append(path[0])

                #change to most recently commanded position
                #qAdd = None
                #if self.q_most_recent_right is None:
                    # qAdd = motion.robot.getKlamptSensedPosition()
                #    qAdd = self.controller.getSensedConfig()
                #    qAdd = [qAdd[v] for v in self.right_arm_indices[:7]]
                    #print self.right_arm_indices
                #else: 
                #    qAdd = self.q_most_recent_right
                #path[0]=qAdd
                #print path 

            #print 'sending path ', path

            if reverse:
                path = path[::-1]

            # if len(path)>=1:
            #     self.sendPath(path, limb='right', readConstants=True)
            # ^Hyunsoo's code - delinked the arms from moving simultaneously


            # print "DEBUGGING"
            # print "path = ", path
            # self.sendPath(path, limb='right')
            # return False

            for milestone in path:

                #print milestone
                self.controller.appendMilestoneRight(milestone, 1)
                
                #move to the milestone in 1 second

                #self.waitForMove() #still doesn't do anything, but it's the thought that counts
                #time.sleep(1)
                if FORCE_WAIT:
                    self.forceWait(milestone, self.right_arm_indices, 0.01)
                else:
                    self.controller.appendMilestoneRight(milestone, WAIT_TIME)
                #wait at the milestone for 2 seconds


            self.q_most_recent_right = path[-1]


            if finalState is not None:
                self.stateRight = finalState
        
            return True
        else:
            print "Error, arm is not in state "+ statusConditional
            return False

    #=============================================================================================================================================

    def buildIKGoalSuctionDown(self,limb, target, offset=0.5, debug=False):

        targetZOffset = vectorops.add(target, [0,0,-offset])
        #targetAxisConstraint = so3.apply(knowledge.shelf_xform[0], vectorops.add(target, [offset, 0,0]))

        #appliedTransform = knowledge.shelf_xform
        appliedTransform = self.perceptionTransform

        targetAxisConstraint = vectorops.add(target, so3.apply(appliedTransform[0], [offset, 0,0]))

        if debug:
            print 'target = ', target
            print 'targetZOffset = ', targetZOffset
            print 'targetAxisConstraint = ', targetAxisConstraint
            print 'Distance between the two =', vectorops.distance(target, targetAxisConstraint)
            #might be better to apply perception transform

        vacuumPoint = self.vacuumTransform[1]
        vacuumOffset = vectorops.add(self.vacuumTransform[1], [offset, 0, 0])
        vacuumAxisOffset = vectorops.add(self.vacuumTransform[1], [0,0,offset])

        return ik.objective(self.robot.link(limb+'_wrist'), local = [vacuumPoint, vacuumOffset, vacuumAxisOffset], world=[target, targetZOffset, targetAxisConstraint])

    def buildIKGoalSuctionNormal(self, limb, target, normal, offset = 0.5, normalDisplacement = 0, debug=False):

        #appliedTransform = knowledge.shelf_xform
        target = vectorops.add(target, vectorops.mul(normal, normalDisplacement))


        appliedTransform = self.perceptionTransform

        targetAxisConstraint = vectorops.add(target, so3.apply(appliedTransform[0], [offset, 0,0]))
        targetZOffset = vectorops.add(target, vectorops.mul(normal, -offset))

        if debug:
            print 'target = ', target
            print 'targetZOffset = ', targetZOffset
            print 'targetAxisConstraint = ', targetAxisConstraint
            print 'Distance between the two =', vectorops.distance(target, targetAxisConstraint)
            #might be better to apply perception transform

        vacuumPoint = self.vacuumTransform[1]
        vacuumOffset = vectorops.add(self.vacuumTransform[1], [offset, 0, 0])
        vacuumAxisOffset = vectorops.add(self.vacuumTransform[1], [0,0,offset])

        return ik.objective(self.robot.link(limb+'_wrist'), local = [vacuumPoint, vacuumOffset, vacuumAxisOffset], world=[target, targetZOffset, targetAxisConstraint])

    def simpleIK(self, numIter=1000, q_seed=None, tol=1e-4, limb=None, goal=None, oneSolution=True):

        ikSolutions = []
        sortedSolutions = []

        if q_seed is None:
            q_seed = [q for q in self.simworld.robot(0).getConfig()]

        for i in range(numIter):

            self.robot.setConfig([conf for conf in q_seed])
                        
            self.randomize_limb_position(limb=limb, rangeVal=0.01*i)


            if ik.solve(goal, tol=tol):
                ikSolutions.append([conf for conf in self.robot.getConfig()])
                #print 'Found Transform is ', self.robot.link(limb+'_wrist').getTransform()
                #print 'Solved at ', i
                #self.sendPath(path=path, limb=limb)
                
        # TODO: sort path by distance from current config, and return the first one

        if ikSolutions is None or ikSolutions ==[]:
            print "all failed for ik"
            return None


        for solution in ikSolutions:
        # this line was buggy: the sortedSolutions only had one entry after the sort !!
        # sortedSolutions = sorted([(vectorops.distanceSquared(solution[0],initialConfig),solution) for solution in ikSolution
        # Add initial joint values to additional joints
            #print solution
            # n = len(solution[0])
            # if len(initialConfig) < n:
            #     initialConfig += [0.0]*(n-len(initialConfig))
            #     print "# links in qcmd < # links in ik solution"

            dist = vectorops.distanceSquared(solution,q_seed)
            config = [v for v in solution]
            ind = solution[1]
            sortedSolutions.append( ((dist), (config, ind)) )


        #print 'this is the length of ikSolutions', len(ikSolutions)

        # print "length of sorted solutions before sorting", len(sortedSolutions)
        sortedSolutions = sorted(sortedSolutions, key=itemgetter(0))
        # print "length of sorted solutions after sorting", len(sortedSolutions)

        #print 'this is the length of sorted solutions (should be 2)', len(sortedSolutions)
        #print 'this is the length of the second index of sortedSolutions (should also be 2)', len(sortedSolutions[1])
        #print 'this is the length of the 0\'th entry to sortedSolutions[1]', len(sortedSolutions[1][0])
        # s[0] contains the distance-sqaured values
        # s[1] contains the ikSolution, which has [0]: config and [1]: index


        if oneSolution:
            #print 'solution 1 = ', sortedSolutions[0][1][0]
            # get the first entry, the second part of that (config, ind) and then the first of those (config)
            #print sortedSolutions
            #print sortedSolutions[0]
            return sortedSolutions[0][1][0]
        else:
            return sortedSolutions

    #######################################################################################
    #Old Movement Functions
                
    def move_to_grasp_object(self,object,step):
        '''
        This method ignores the object orientation (for vacuum only)
        '''
        if step == 1:
            offset = 0.25
        elif step == 2:
            offset = 0.25
        elif step >= 3:
            offset = 0.25 - 0.15 - 0.01*step

        self.waitForMove()
        self.robot.setConfig(self.controller.getCommandedConfig())

        global xPos
        global yPos 
        xScale = 0.33
        yScale = 0.435
        xPos = 0.5 * xScale
        yPos = 0.5 * yScale


        # if single item:
        if step == 1:
            print "\n=========================="        
            if PHYSICAL_SIMULATION:
                print "Running perception on order #", binIndex
                time.sleep(5)
                if singleItemList[binIndex]:
                    print "Single object..."
                    # xyPos = perceiver.perceive_single_object() OLD CODE
                    if xyPos == None:
                        print "Failed to detect object"
                        return False
                    xPos = xyPos[0]
                    yPos = xyPos[1]
                else:
                    print "Multiple objects..."
                    # xyPos = perceiver.perceive(orderList[binIndex]) OLD CODE
                    if xyPos == None:
                        print "Failed to detect object"
                        return False
                    xPos = xyPos[0]
                    yPos = xyPos[1]
                # change coordinate 
                print "pre-adjust object position =", (xPos, yPos)                
                temp = yPos
                yPos = 1 - xPos
                xPos = temp                    
                print "adjusted object position =", (xPos, yPos)
                xPos = xPos * xScale
                yPos = yPos * yScale
            else:
                print "Running FAKE perception on bin", binIndex,"=", self.current_bin
                xPos = 0.33
                yPos = 0.435
            self.held_object.randRt = [ so3.rotation([0,1,0], math.pi/4),
                                       vectorops.add([-0.165,0,-0.33], [xPos,0,yPos])]
            time.sleep(5)
        t_obj = object.randRt[1]
        #print "t_obj =",t_obj
        # t_obj = [0,0,0]
        R_obj = so3.identity()

        # object xform relative to world frame
        objxform = se3.mul( self.left_gripper_link.getTransform() ,
                            # object xform relative to gripper link =
                            # gripper center xform relative to gripper link (X) obj xform relative to gripper center
                            se3.mul(left_gripper_center_xform,
                                    # object xform relative to gripper center xform
                                    [so3.mul(R_obj, so3.rotation([1,0,0],math.pi/2)), t_obj] ))

        goal_loc = vectorops.add(objxform[1], [0,0,offset])

        # self.frames.append((R_obj, t_obj))
        # self.frames.append((objxform[0], objxform[1]))

        # Setup ik objectives for both arms
        right_goal = []
        right_goal.append(ik.objective(self.vacuum_link,local=[[0,0,0],[0,0,0.01]], world=[goal_loc, vectorops.sub(goal_loc, [0,0,0.01])]))

        qcmd = self.controller.getCommandedConfig()
        # qcmd = self.controller.getSensedConfig()
        limbs = ['right']

        if step==1:
            print "Solving for GRASP_OBJECT"
        for i in range(500):
            sortedSolutions = self.get_ik_solutions(right_goal, limbs, qcmd, maxResults=100, maxIters=100, rangeVal = 0.001)

            if len(sortedSolutions)==0:
                continue

            # prototyping hack: move straight to target
            if FAKE_SIMULATION:
                q = sortedSolutions[0][0]
                n = self.robot.numLinks()
                if len(q)<n:
                    q += [0.0]*(n-len(q))
                self.controller.setMilestone(q)
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
                        return True
        print "Failed to plan path"
        return False

    def move_to_grasp_object_backup(self,object):
        '''
        This method ignores the object orientation (for vacuum only)
        '''
        self.waitForMove()
        self.robot.setConfig(self.controller.getCommandedConfig())

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
        right_goal = []
        right_goal.append(ik.objective(self.right_gripper_link,local=[[0,0,0],[0,0,0.01]], world=[Rt[1],vectorops.add(Rt[1],[0,0,-0.01])]))

        qcmd = self.controller.getCommandedConfig()
        # qcmd = self.controller.getSensedConfig()
        limbs = ['right']

        print "\nSolving for GRASP_OBJECT"
        for i in range(100):
            sortedSolutions = self.get_ik_solutions(right_goal, limbs, qcmd, maxResults=100, maxIters=100)

            if len(sortedSolutions)==0:
                continue

            # prototyping hack: move straight to target
            if FAKE_SIMULATION:
                q = sortedSolutions[0][0]
                n = self.robot.numLinks()
                if len(q)<n:
                    q += [0.0]*(n-len(q))
                self.controller.setMilestone(q)
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
                        return True
        print "Failed to plan path"
        return False

    def move_to_order_bin(self,object,step):
        if step == 1:
            q = self.robot.getConfig()
            q[35] = -0.3
            self.controller.appendMilestone(q)
            return True

        elif step == 2:
            self.controller.appendMilestone(baxter_rest_config)
            return True

        print "Planning failed"
        return False

    def sendPath(self,path,maxSmoothIters = 0, INCREMENTAL=False, clearRightArm = False, limb = None, readConstants=False, internalSpeed=SPEED):


        # interpolate path linearly between two endpoints
        if path == None:
            print "sending empty path"
            return False

        # if INCREMENTAL:
        #     assert(len(path)==2)
        #     i = 0
        #     endIndex = 1
        #     while i <= endIndex-1:
        #         q = path[i]
        #         qNext = path[i+1]
        #         dt = vectorops.distance(q,qNext)

        #         if dt>0.01:
        #             qInterp = vectorops.div(vectorops.add(q, qNext), 2)
        #             path.insert(i+1, qInterp)
        #             endIndex +=1
        #         else:
        #             i += 1

        # for p in path:
        #     for c in p:
        #         print '%0.2f,'%c,
        #     print ''

        n = self.robot.numLinks()
        for i in range(len(path)):

            # if we specify a limb, and the path only has seven numbers (DOF for each arm, we shouldn't append 0's)
            if readConstants and limb is not None:
                # if we're reading a path from the milestones
                pass
            else:
                
                #print path
                if len(path[i])<n:
                    path[i] += [0.0]*(n-len(path[i]))
                #pass


        #print 'path in sendPath is ', path

        #print "Got to self.plan"

        for smoothIter in range(maxSmoothIters):
            # path = path
            smoothePath = [0]*(len(path)*2-1)
            for i in range(len(path)-1):
                smoothePath[i*2] = path[i]
                smoothePath[i*2 +1] = vectorops.div(vectorops.add(path[i],path[i+1]), 2)
            smoothePath[-1] = path[-1]
            path = smoothePath


        '''
        q = path[0]
        qmin,qmax = self.robot.getJointLimits()

        # removing AppendRamp clamping error
        for i in [23,30,31,43,50,51,54]: q[i] = 0
        for i in self.right

        q = self.clampJointLimits(q,qmin,qmax)
        '''

        #q keeps getting overwritten
        #self.controller.setMilestone(path[0])
        #self.controller.setMilestone(self.controller.getConfig())


        if not readConstants:
            #print "\ttotal", len(path), "milestones in path"
            for j in xrange(0, len(path)):
                #print "moving to milestone #", j
            # for q in path[1:]:
                #for i in [23,30,31,43,50,51,54]:
                #    # print i, qmin[i], q[i], qmax[i]
                #    path[j][i] = 0

                #if clearRightArm:
                #    q[35] = -0.3

                #q = self.clampJointLimits(q,qmin,qmax)

                #print "PHYSICAL_SIMULATION=",PHYSICAL_SIMULATION
                if not PHYSICAL_SIMULATION:
                    self.controller.controller.setVelocity([1]*61,0.1)
                    self.controller.appendMilestone(path[j])
                    
                    #self.controller.setMilestone(path[j])
                    

                    #print "got to this point"
                    # if INCREMENTAL:
                    #     self.waitForMove()
                    #     diff = vectorops.distance(self.controller.getCommandedConfig(), self.controller.getSensedConfig())
                    #     print round(diff,3)
                    #     if diff > 0.03:
                    #         self.robot.setConfig(self.controller.getSensedConfig())
                    #         return
                    #     self.controller.controller.addMilestone(q)
                    # #     # ft = self.controller.sim.getJointForces(self.world.robot(0).link(55))
                    # #     # for i in range(len(ft)):
                    # #     #     ft[i] = round(ft[i], 2)
                    # #     # print vectorops.norm(ft[0:3])
                    # #     self.waitForMove()
                    # #     print round(vectorops.distance(self.controller.getCommandedConfig(), self.controller.getSensedConfig()),2)
                    # else:
                    #     self.controller.controller.setVelocity([1]*61,0.1)
                    #     self.controller.appendMilestone(q)

            # original
        # print "reached"

        if PHYSICAL_SIMULATION:
            i = 0
            endIndex = len(path)            
            # if endIndex==1:
            #     q=path[0]
            #     path[0]=self.robot.getConfig()
            #     path.append(q)

            counter = 0

            #path.append(path[-1])
            #endIndex = len(path)
        
            #print 'myPath = ', path

            while i <endIndex-1:
                # print i, endIndex
                q = path[i]
                qNext = path[i+1]
                dt = vectorops.distance(q,qNext)

                # smooth trajectory by interpolating between two consecutive configurations
                # if the distance between the two is big

                # Note: might want to make dt bigger for arm movements (only 7 configurations vs 52)
                if dt>0.1:
                    qInterp = vectorops.div(vectorops.add(q, qNext), 2)
                    path.insert(i+1, qInterp)
                    endIndex +=1
                    continue
                else:
                    i += 1
                    self.waitForMove()
                    # if INCREMENTAL:
                    #     time.sleep(.1)
                    if counter%internalSpeed == 0 or INCREMENTAL:
                        if limb == 'left':
                            self.controller.appendMilestoneLeft(q)
                        elif limb == 'right':
                            self.controller.appendMilestoneRight(q)
                        else:
                            #print 'milestone #', i, q
                            self.controller.appendMilestone(q)
                    counter +=1
            if limb == 'left':
                self.controller.appendMilestoneLeft(path[-1])
            elif limb == 'right':
                self.controller.appendMilestoneRight(path[-1])
            else:
                #print 'last milestone', path[-1]
                self.controller.appendMilestone(path[-1])
            # print 'Done with moving'

    def sendPathClosedLoop(self,path, clearRightArm = False):
        # print "pathlength starting =", len(path)
        # removing AppendRamp clamping error
        q = path[0]
        for i in [23,30,31,43,50,51,54]: q[i] = 0

        qmin,qmax = self.robot.getJointLimits()
        q = self.clampJointLimits(q,qmin,qmax)

        self.controller.setLinear(q,0.1)
        spatulaConfig = q[55:58]

        qOriginal = q
        qPrev = q
        for q in path[1:]:
            for i in right_arm_geometry_indices:
                q[i] = qOriginal[i]

            for i in [23,30,31,43,50,51,54]:
                q[i] = 0
            if clearRightArm:
                q[35] = -0.3
            q = self.clampJointLimits(q,qmin,qmax)

            #restore spatulaConfig
            q[55:58] = spatulaConfig[0:3]

        # clean up trajectory by skipping configurations where the spatula isn't parallel to ground
        i = 0
        endIndex = len(path)
        self.robot.setConfig(path[0])
        xform0 = self.robot.link(55).getTransform()
        p0 = xform0[1]
        p1 = se3.apply(xform0,[1,0,0])
        u = vectorops.sub(p1,p0)    # x-axis of spatula base (-z axis in world frame)
        # print u[2]

        counter = 0
        while i < endIndex-1:
            # print i, endIndex
            q = path[i]
            qNext = path[i+1]
            dt = vectorops.distance(q,qNext)

            self.robot.setConfig(q)
            xform0 = self.robot.link(55).getTransform()
            p0 = xform0[1]
            p1 = se3.apply(xform0,[1,0,0])
            v = vectorops.sub(p1,p0) # x-axis of spatula base (-z axis in world frame)
            # print v[2]

            # if the two angles deviate too much, then skip it
            if abs(v[2]-u[2]) > 0.00001:
                i+=1
                continue

            # smooth trajectory by interpolating between two consecutive configurations
            # if the distance between the two is big
            if dt>0.01:
                qInterp = vectorops.div(vectorops.add(q, qNext), 2)
                path.insert(i+1, qInterp)
                endIndex +=1
                continue
            else:
                i += 1
                self.waitForMove()
                if counter%SPEED == 0:
                    self.controller.appendLinear(q,dt*3)
                counter += 1
        # print "pathlength ending =",endIndex
        self.waitForMove()

    def clampJointLimits(self,q,qmin,qmax):
        for i in range(len(q)):
            if (q[i] < qmin[i]) :
                # print "Joint #",i,"(",q[i],") out of limits (min:",qmin[i],")"
                # print "Changed joint value to its minimum"
                q[i] = qmin[i]

            if (q[i] > qmax[i]) :
                # print "Joint #",i,"(",q[i],") out of limits (max:",qmax[i],")"
                # print "Changed joint value to its maximum"
                q[i] = qmax[i]
        return q

       
    #============================================================
    # Calibration Functions

    def calibrateShelf(self):
        #blocking
        while(True):
            try:
                input_var = raw_input("Shelf: Enter joint and angle to change to separated by a comma: ").split(',');
                #translational transformation
                calibrateR = [1,0,0,0,1,0,0,0,1]
                calibrateT = [0,0,0]

                if(input_var[0] == "x" ):
                    calibrateT[0] = calibrateT[0] + float(input_var[1])
                elif(input_var[0] == "y" ):
                    calibrateT[1] = calibrateT[1] + float(input_var[1])
                elif(input_var[0] == "z" ):
                    calibrateT[2] = calibrateT[2] + float(input_var[1])
                #rotational transformations
                elif(input_var[0] == "xr" ):
                    calibrateR = so3.mul(calibrateR, so3.rotation([1, 0, 0], float(input_var[1])))
                elif(input_var[0] == "yr" ):
                    calibrateR = so3.mul(calibrateR, so3.rotation([0, 1, 0], float(input_var[1])))
                elif(input_var[0] == "zr" ):
                    calibrateR = so3.mul(calibrateR, so3.rotation([0, 0, 1], float(input_var[1])))

                elif(input_var[0] == "q"):
                    break

                time.sleep(0.1);

                calibrate = (calibrateR, calibrateT)
                print 'calibrate = ', calibrate
                self.simworld.terrain(0).geometry().transform( calibrate[0], calibrate[1] )
                self.shelf_xform = se3.mul(calibrate, self.shelf_xform)

                #rotation matrices are applied left to right

                print self.shelf_xform
            except: 
                print "input error\n"
                print sys.exc_info()
                
    def calibrateCamera(self, limb='left'):
        #blocking
        while(True):

            try:
                input_var = raw_input("Camera: Enter joint and angle to change to separated by a comma: ").split(',');
                #translational transformation
                calibrateR = self.cameraTransform[0]
                calibrateT = self.cameraTransform[1]

                if(input_var[0] == "x" ):
                    calibrateT[0] = calibrateT[0] + float(input_var[1])
                elif(input_var[0] == "y" ):
                    calibrateT[1] = calibrateT[1] + float(input_var[1])
                elif(input_var[0] == "z" ):
                    calibrateT[2] = calibrateT[2] + float(input_var[1])
                #rotational transformations
                elif(input_var[0] == "xr" ):
                    calibrateR = so3.mul(calibrateR, so3.rotation([1, 0, 0], float(input_var[1])))
                elif(input_var[0] == "yr" ):
                    calibrateR = so3.mul(calibrateR, so3.rotation([0, 1, 0], float(input_var[1])))
                elif(input_var[0] == "zr" ):
                    calibrateR = so3.mul(calibrateR, so3.rotation([0, 0, 1], float(input_var[1])))

                elif(input_var[0] == "q"):
                    break

            except: 
                print "input error\n"
                #print error.strerror

            time.sleep(0.1);
            calibrate = (calibrateR, calibrateT)
            self.cameraTransform = ( calibrate[0], calibrate[1] )
            

            print self.simworld.robot(0).link(limb + '_wrist').getTransform()
            totalCameraXform = self.getCameraToWorldXform(limb)

            print self.cameraTransform

            current_cloud = perceiver.get_current_point_cloud(*self.getCameraToWorldXform(limb), limb=limb, tolist=False)
            self.points = current_cloud.tolist()

    def calibrateVacuum(self, limb='left'):
         while(True):

            try:
                input_var = raw_input("Vacuum: Enter joint and angle to change to separated by a comma: ").split(',');
                #translational transformation
                calibrateR = self.vacuumTransform[0]
                calibrateT = self.vacuumTransform[1]

                if(input_var[0] == "x" ):
                    calibrateT[0] = calibrateT[0] + float(input_var[1])
                elif(input_var[0] == "y" ):
                    calibrateT[1] = calibrateT[1] + float(input_var[1])
                elif(input_var[0] == "z" ):
                    calibrateT[2] = calibrateT[2] + float(input_var[1])
                #rotational transformations
                elif(input_var[0] == "xr" ):
                    calibrateR = so3.mul(calibrateR, so3.rotation([1, 0, 0], float(input_var[1])))
                elif(input_var[0] == "yr" ):
                    calibrateR = so3.mul(calibrateR, so3.rotation([0, 1, 0], float(input_var[1])))
                elif(input_var[0] == "zr" ):
                    calibrateR = so3.mul(calibrateR, so3.rotation([0, 0, 1], float(input_var[1])))

                elif(input_var[0] == "q"):
                    break

                time.sleep(0.1);
                calibrate = (calibrateR, calibrateT)
                self.vacuumTransform = ( calibrate[0], calibrate[1] )

                print self.vacuumTransform

                #vacuumTotalTransform = se3.mul(self.simworld.robot(0).link(limb + '_wrist').getTransform(), self.vacuumTransform)
                #self.frames = []
                #self.frames.append(vacuumTotalTransform)
                
            except: 
                print "input error\n"

    #===========================================================

    def getCameraToWorldXform(self, limb='left'):
        '''
        get current transformation (R, t) of camera in world frame. 
        '''
        return se3.mul(self.simworld.robot(0).link(limb + '_wrist').getTransform(), self.cameraTransform)

        #return se3.mul(self.cameraTransform, self.simworld.robot(0).link(limb)+'_wrist').getTransform())
        #IMPORTANT
        #figure out whether this is right

    def forceWait(self, milestone1, indices, eps):

        milestone2 = [self.controller.getSensedConfig()[v] for v in indices]
        while (np.linalg.norm(np.array(milestone2)-milestone1)) >= eps:
            milestone2 = [self.controller.getSensedConfig()[v] for v in indices]
            print (np.linalg.norm(np.array(milestone2)-milestone1))

    def getBounds(self, bin='bin_A'):

        if len(bin)==1:
            bin = 'bin_'+bin

        local_xform = apc.bin_bounds[bin]
        #print local_xform
        min_point = se3.apply(knowledge.shelf_xform, local_xform[0])
        max_point = se3.apply(knowledge.shelf_xform, local_xform[1])
        
        #print 'Min point = ',min_point
        #print 'Max point = ',max_point

        return (min_point, max_point)

    def printEndEffectorPosition(self, limb):
        
        EndEffectorTransform = se3.mul(self.simworld.robot(0).link(limb+'_wrist').getTransform(), self.vacuumTransform)
        print EndEffectorTransform[1]

    #=========================================================


            


class MyGLViewer(GLRealtimeProgram):
    def __init__(self,simworld,planworld):
        GLRealtimeProgram.__init__(self,"My GL program")

        self.simworld = simworld
        self.planworld = planworld
        self.sim = Simulator(simworld)
        
        if not FAKE_SIMULATION and not PHYSICAL_SIMULATION:
            self.simulate = True
        else:
            self.simulate = False

        # draw settings
        self.draw_bins = False
        self.draw_gripper_and_camera = True
        self.drawVE = False
        self.drawPath = False

        # initialize controllers
        if FAKE_SIMULATION:
            self.low_level_controller = FakeLowLevelController(simworld.robot(0),self.sim.controller(0))
        elif PHYSICAL_SIMULATION:
            self.low_level_controller = PhysicalLowLevelController(simworld.robot(0))
        elif not FAKE_SIMULATION and not PHYSICAL_SIMULATION:
            self.low_level_controller = LowLevelController(simworld.robot(0),self.sim.controller(0),self.sim)
        self.command_queue = Queue()

        # visualize world model
        # visualization.add("world",planworld)
        # visualization.dialog()

        # starts a thread running "run_controller" with the specified picking controller and command queue
        self.picking_controller = PickingController(simworld, planworld,self.low_level_controller)
        self.picking_thread = Thread(target=run_controller,args=(self.picking_controller,self.command_queue))
        self.picking_thread.start()

    def idle(self):
        if self.simulate:
            self.dt = 0.01
            self.sim.simulate(self.dt)
            glutPostRedisplay()

    def glShowPointCloud(self, pc, downsample_rate=5, pt_size=None):
        # print glGetFloatv(GL_CURRENT_COLOR)
        pc = np.array(pc)
        # print 'Rendering %d points'%pc.shape[0]
        d = pc.shape[1]
        assert d==3 or d==4, 'Unrecognized point cloud shape: '+str(pc.shape)
        pc = pc[::downsample_rate, :]
        pc = pc.tolist()
        glDisable(GL_LIGHTING)
        glColor3f(0, 1, 0)
        if pt_size is not None:
            glPointSize(pt_size)
        glBegin(GL_POINTS)
        for p in pc:
            if d==4:
                r, g, b = pcl_float_to_rgb(p[3])
                r /= 255.0
                g /= 255.0
                b /= 255.0
                glColor3f(r, g, b)
            glVertex3f(p[0], p[1], p[2])
        glEnd()
        glEnable(GL_LIGHTING)
        glColor3f(1,0,0)

    def printStuff(self):
        print "shelf xform:", knowledge.shelf_xform
        print self.simworld.terrain(0).geometry()
        #print "terrain xform:", self.simworld.terrain(0).geometry().getTransform(), "\n\n"


    def display(self):
        #self.printStuff()

        #you may run auxiliary openGL calls, if you wish to visually debug
        # ft = self.sim.getJointForces(self.simworld.robot(0).link(55))
        # for i in range(len(ft)):
        #     ft[i] = round(ft[i], 2)
        # print vectorops.norm(ft[0:3])
        # diff = vectorops.distance(self.low_level_controller.getCommandedConfig(), self.low_level_controller.getSensedConfig())
        # print round(diff,2)
        # if diff > 0.03:
        #     self.simworld.robot(0).setConfig(self.low_level_controller.getSensedConfig())
        # print self.simworld.robot(0).getConfig()[-1], self.planworld.robot(0).getConfig()[-1]

        #draw the world
        if not PHYSICAL_SIMULATION:
            self.sim.updateWorld()
            #self.simworld.drawGL()
        else:
            robot = motion.robot
            q = robot.getKlamptSensedPosition()
            self.simworld.robot(0).setConfig(q)
            #self.simworld.drawGL()

        if CALIBRATE:
            self.planworld = self.picking_controller.getWorld()
            points = self.picking_controller.getPerceivedPoints()
            if points is not None:
                self.glShowPointCloud(points, pt_size=1)

            glDisable(GL_LIGHTING)
            glColor3f(1, 0, 0)
            glPointSize(5)
            glBegin(GL_POINTS)
            glVertex3f(* se3.mul(self.simworld.robot(0).link('right_wrist').getTransform(), self.picking_controller.vacuumTransform)[1])
            #glVertex3f(*se3.mul(self.simworld.robot(0).link('right_wrist').getTransform(), self.pickick_controller.vacuumTransform))
            glEnd()
            glEnable(GL_LIGHTING)

            gldraw.xform_widget(se3.mul(self.simworld.robot(0).link('right_wrist').getTransform(), self.picking_controller.vacuumTransform), 0.05, 0.01)

            glDisable(GL_LIGHTING)
            glColor3f(0, 0, 1)
            glPointSize(10)
            glBegin(GL_POINTS)
            for point in self.picking_controller.debugPoints:
                glVertex3f(*point)
            

            if self.picking_controller.pick_pick_pos is not None:
                glVertex3f(*self.picking_controller.pick_pick_pos)

            glEnd()
            glEnable(GL_LIGHTING)


            glDisable(GL_LIGHTING)
            glColor3f(1, 0, 0)
            glPointSize(15)
            glBegin(GL_POINTS)
            for point in self.picking_controller.debugFailedPoints:
                glVertex3f(*point)

            glEnd()
            glEnable(GL_LIGHTING)


            # if self.picking_controller.getPerceivedPoints() is not None:
            #     glColor3f(1.0,0.0,0.0)
            #     glBegin(GL_POINTS)
            #     for point in self.picking_controller.points[::5]:
            #         glVertex3f(point[0], point[1], point[2])


            #     glEnd()
            self.simworld.drawGL()


        if DRAW_STOW_DEBUG:

            glDisable(GL_LIGHTING)
            glColor3f(0, 1, 0)
            glPointSize(15)
            glBegin(GL_POINTS)
            for point in self.picking_controller.rightStowPoints:
                glVertex3f(*point)

            glEnd()
            glEnable(GL_LIGHTING)

            glDisable(GL_LIGHTING)
            glColor3f(0, 0, 1)
            glPointSize(15)
            glBegin(GL_POINTS)
            for point in self.picking_controller.leftStowPoints:
                glVertex3f(*point)

            glEnd()
            glEnable(GL_LIGHTING)

            glDisable(GL_LIGHTING)
            glColor3f(.5, 0, .5)
            glPointSize(15)
            glBegin(GL_POINTS)
            for point in self.picking_controller.debugFailedLeftPoints:
                glVertex3f(*point)

            glEnd()
            glEnable(GL_LIGHTING)

            glDisable(GL_LIGHTING)
            glColor3f(.5, .5, 0)
            glPointSize(15)
            glBegin(GL_POINTS)
            for point in self.picking_controller.debugFailedRightPoints:
                glVertex3f(*point)

            glEnd()
            glEnable(GL_LIGHTING)


        if SHOW_BIN_CONTENT:
            points = self.picking_controller.bin_content_cloud
            if points is not None:
                self.glShowPointCloud(points, self.picking_controller.bin_render_downsample_rate, self.picking_controller.bin_render_ptsize)
            pick_pick_pos = self.picking_controller.pick_pick_pos
            if pick_pick_pos is not None:
                gldraw.xform_widget(([1,0,0,0,1,0,0,0,1], [pick_pick_pos[0], pick_pick_pos[1], pick_pick_pos[2]]), 0.5, 0.01, fancy=False)
                pass

        if SHOW_TOTE_CONTENT:
            points = self.picking_controller.tote_cloud
            if points is not None:
                self.glShowPointCloud(points, self.picking_controller.tote_render_downsample_rate, self.picking_controller.tote_render_ptsize)
            stow_pick_pos = self.picking_controller.stow_pick_pos
            all_stow_pick_poss = self.picking_controller.all_stow_pick_poss
            if stow_pick_pos is not None:
                gldraw.xform_widget(([1,0,0,0,1,0,0,0,1], [stow_pick_pos[0], stow_pick_pos[1], -0.1]), 1, 0.01, fancy=False)
                pass
            if all_stow_pick_poss is not None:
                for stow_pick_pos in all_stow_pick_poss:
                    gldraw.xform_widget(([1,0,0,0,1,0,0,0,1], [stow_pick_pos[0], stow_pick_pos[1], -0.1]), 0.5, 0.1, fancy=False)
                
                
        if SHOW_BIN_BOUNDS:
            for letter in ['A','B','C','D','E','F','G','H','I','J','K','L']:
            # for letter in ['F']:
                draw_wire_box(*map(lambda p:se3.apply(knowledge.shelf_xform, p), apc.bin_bounds['bin_'+letter]))
        # draw the shelf and floor
        # if self.simworld.numTerrains()==0:
        #     for i in range(self.planworld.numTerrains()):
        #         self.planworld.terrain(i).drawGL()


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
        # for i in ground_truth_items:
        #     if i.xform == None:
        #         continue

        #     if i.bin_name == 'order_bin':
        #     # draw in wireframe
        #         glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,1,1,1])
        #         draw_oriented_box(i.xform,i.info.bmin,i.info.bmax)
        #         continue

        #     #if perceived, draw in solid color
        #     if knowledge.bin_contents[i.bin_name]!=None and i in knowledge.bin_contents[i.bin_name]:
        #         glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,0.5,0,1])
        #         draw_oriented_box(i.xform,i.info.bmin,i.info.bmax)
        #     else:
        #         #otherwise, draw in wireframe
        #         glDisable(GL_LIGHTING)
        #         glColor3f(1,0.5,0)
        #         draw_oriented_wire_box(i.xform,i.info.bmin,i.info.bmax)
        #         glEnable(GL_LIGHTING)

        # # Draws the object held on gripper
        # obj,grasp = self.picking_controller.held_object,self.picking_controller.active_grasp

        # if obj != None:
        #     # spatula scooping
        #     if self.picking_controller.stateLeft == 'holding':
        #         R_obj = obj.randRt[0]
        #         t_obj = obj.randRt[1]
        #         gripper_xform = self.simworld.robot(0).link(left_gripper_link_name).getTransform()
        #         objxform = se3.mul( gripper_xform,  se3.mul(  left_gripper_center_xform,  [R_obj, t_obj] )  )
        #         # objxform = se3.mul( gripper_xform,  se3.mul(  left_gripper_center_xform,  se3.inv(grasp.grasp_xform) )  )

        #     # gripper
        #     elif self.picking_controller.stateRight == 'holding':
        #         gripper_xform = self.simworld.robot(0).link(right_gripper_link_name).getTransform()
        #         # objxform = se3.mul(gripper_xform,se3.mul(left_gripper_center_xform,se3.inv(grasp.grasp_xform)))
        #         # objxform = se3.mul(gripper_xform,se3.mul(left_gripper_center_xform,se3.inv(se3.identity())))
        #         objxform = se3.mul(gripper_xform,left_gripper_center_xform)

        #     glDisable(GL_LIGHTING)
        #     glColor3f(1,1,1)
        #     draw_oriented_wire_box(objxform,obj.info.bmin,obj.info.bmax)
        #     glEnable(GL_LIGHTING)

        #show gripper and camera frames
        if self.draw_gripper_and_camera:
            left_camera_link = self.simworld.robot(0).link(left_camera_link_name)
            right_camera_link = self.simworld.robot(0).link(right_camera_link_name)
            left_gripper_link = self.simworld.robot(0).link(left_gripper_link_name)
            right_gripper_link = self.simworld.robot(0).link(right_gripper_link_name)
            vacuum_link = self.simworld.robot(0).link("vacuum:vacuum")
            gldraw.xform_widget(left_camera_link.getTransform(),0.1,0.01,fancy=False)
            gldraw.xform_widget(right_camera_link.getTransform(),0.1,0.01,fancy=False)
            #gldraw.xform_widget(left_gripper_link.getTransform(),0.1,0.01,fancy=False)
            gldraw.xform_widget(right_gripper_link.getTransform(),0.1,0.01,fancy=False)
            gldraw.xform_widget(vacuum_link.getTransform(),0.1,0.01,fancy=False)
            # gldraw.xform_widget(se3.mul(left_gripper_link.getTransform(),left_gripper_center_xform),0.05,0.005,fancy=False)
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
                    if len(q)<len(qcmd):
                        for k in range(len(self.picking_controller.planner.limb_indices)):
                            if k>len(q): print "Index Out of Range: (k,len(q),q)", k,len(q),q
                            qcmd[self.picking_controller.planner.limb_indices[k]] = q[k]
                        self.planworld.robot(0).setConfig(qcmd)
                    else:
                        self.planworld.robot(0).setConfig(q)
                    glVertex3f(*self.planworld.robot(0).link(23).getTransform()[1])
                glEnd()

                glColor3f(0,1,0)
                glLineWidth(5.0)
                glBegin(GL_LINE_STRIP)
                for q in path:
                    if len(q)<len(qcmd):
                        for k in range(len(self.picking_controller.planner.limb_indices)):
                            qcmd[self.picking_controller.planner.limb_indices[k]] = q[k]
                        self.planworld.robot(0).setConfig(qcmd)
                    else:
                        self.planworld.robot(0).setConfig(q)
                    glVertex3f(*self.planworld.robot(0).link(43).getTransform()[1])
                glEnd()

                glLineWidth(1.0)

        glEnable(GL_LIGHTING)

        for frame in self.picking_controller.frames:
            R = frame[0]
            t = frame[1]
            gldraw.xform_widget([R,t], 0.025, 0.005, lighting=True, fancy=True)

        return

    def keyboardfunc(self,c,x,y):
        #c = c.lower()
        if c=='z':
            self.simulate = not self.simulate
            print "Simulating:",self.simulate
        else:
            self.command_queue.put(c)
            if c=='q':
                self.picking_thread.join()
                exit(0)
        glutPostRedisplay()

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

# USED LETTERS: 
# z, f, g, 


def run_controller(controller,command_queue):
    DEFAULT_LIMB = 'right'
    while True:
        c = command_queue.get()
        if c != None:
            c = c.lower()
            print "\n================================"
            print "Running command", c
            if c=='h':
                print 'H: Help (Show this text)'
                print 'E: View Bin or Tote'
                print 'Y: Prepare Pick for Bin or Tote'
                print 'X: Grasp Action from Tote or Bin'
                print 'P: Place in Tote or Bin'
                print 'I: Save Canonical Point Cloud for Bin/Tote'
                print 'A: Render Bin Content'
                print 'T: Render Tote Content'
                print 'N: Get Pick Position for Bin'
                print 'L: Get Pick Position for Stow'
                print 'V: Change Default Limb'
                print 'R: Move to Rest Configuration'
                print 'M: Move Gripper to Center'
                print '`: Run ICP to Find Transformation of Perturbed Shelf'
                print 'S: Calibrate Canonical (Unperturbed) Shelf Transformation'
                print 'W: Calibrate Camera Transformation Relative to Wrist'
                print 'O: Calibrate Vacuum Transform Relative to Wrist'
                print 'F: Transform Shelf from Canonical to Perturbed Pose'
                print 'G: Transform Shelf from Perturbed to Canonical Pose'
                print 'C: See(C) the Tote'
                print 'B: Get the Bounds of a Bin'
                print '/: Get the global position of the end effector' 
                print 'Q: Quit'
                print 'D: Debug method - test IK for bins'
                print 'M: Debug method - test IK for tote'
                print '1: Run Stow Task and stow object in selected bin (current default = H)'
                print '2: Run Pick Task for selected bin'
                print '+: Fully Autonomous Pick Run'
                print '=: Fully Autonomous Stow Run'
            elif c=='e':
                print 'View Bin - Press Bin Letter on GUI. Press X to cancel'
                bin_letter = command_queue.get()
                while bin_letter is None:
                    print bin_letter
                    bin_letter = command_queue.get()

                if bin_letter.lower()=='t':
                    controller.viewToteAction(limb=DEFAULT_LIMB)
                    continue
                if bin_letter.lower()=='x':
                    print 'Canceled'
                    continue
                if not ( ('a'<=bin_letter.lower()<='l') ):
                    print 'Unrecognized Letter. Canceled'
                    continue
                # bin_letter = raw_input('View Bin - Enter Bin Letter in Terminal: ')
                controller.viewBinAction('bin_'+bin_letter.upper(), limb=DEFAULT_LIMB)
            elif c=='y':
                print 'Prepare Pick for Bin or Tote - Press Bin Letter on GUI or T for Tote. Press X to cancel'
                bin_letter = command_queue.get()
                while bin_letter is None:
                    bin_letter = command_queue.get()
                if bin_letter.lower()=='t':
                    print 'preparing grasp from tote'
                    controller.prepGraspFromToteAction(limb=DEFAULT_LIMB)
                    continue
                if bin_letter.lower()=='x':
                    print 'Canceled'
                    continue
                if not ( ('a'<=bin_letter.lower()<='l') ):
                    print 'Unrecognized Letter. Canceled'
                    continue                
                controller.prepGraspFromBinAction( limb=DEFAULT_LIMB,b=bin_letter.upper())
            elif c == 'x':
                #controller.graspAction()
                print 'Grasp Object for Bin or Tote - Press Bin Letter on GUI or T for Tote. Press X to cancel'
                bin_letter = command_queue.get()
                while bin_letter is None:
                    bin_letter = command_queue.get()
                if bin_letter.lower()=='t':
                    print 'Grasping from tote'
                    controller.graspFromToteAction(limb=DEFAULT_LIMB)
                    continue
                if bin_letter.lower()=='x':
                    print 'Canceled'
                    continue

                if not ( ('a'<=bin_letter.lower()<='l') ):
                    print 'Unrecognized Letter. Canceled'
                    continue
                controller.graspFromBinAction(limb= DEFAULT_LIMB, bin='bin_'+ bin_letter.upper())          
            elif c == 'p':
                print 'Place object in Tote (from Bin) or Bin (from Tote) - Press Bin Letter on GUI or T for Tote. Press X to cancel'
                bin_letter = command_queue.get()
                while bin_letter is None:
                    bin_letter = command_queue.get()
                if bin_letter.lower()=='t':
                    print 'Putting object in tote (from bin)'
                    controller.placeInToteAction(limb=DEFAULT_LIMB)
                    continue
                if bin_letter.lower()=='x':
                    print 'Canceled'
                    continue
                if not ( ('a'<=bin_letter.lower()<='l') ):
                    print 'Unrecognized Letter. Canceled'
                    continue
                controller.placeInBinAction(limb=DEFAULT_LIMB, bin=bin_letter.upper())

            elif c=='i':
                print 'Save Canonical Point Cloud for Bin/Tote - Press Bin Letter on GUI. Press T to save tote. Press X to cancel. '
                bin_letter = command_queue.get()
                while bin_letter is None:
                    bin_letter = command_queue.get()
                if bin_letter.lower()=='x':
                    print 'Canceled'
                    continue
                # bin_letter = raw_input('Save Canonical Point Cloud for Bin - Enter Bin Letter In Terminal: ')
                if bin_letter.upper()!='T':
                    controller.saveCanonicalBinPointCloud(bin_letter.upper(), limb=DEFAULT_LIMB)
                else:
                    controller.saveCanonicalTotePointCloud(limb=DEFAULT_LIMB)
            elif c=='j':
                print 'Reach into bin. Press X to cancel. '
                bin_letter = command_queue.get()
                while bin_letter is None:
                    bin_letter = command_queue.get()
                if bin_letter.lower()=='x':
                    print 'Canceled'
                    continue
                controller.moveObjectIntoBin(limb=DEFAULT_LIMB, bin = bin_letter.upper())

            elif c=='d':
                print 'Run Debug method - currently testing IK for Bins'
                binNumber = raw_input('Enter bin name (ex. all, bin_A, ... ): ')
                controller.testBinIK(limb=DEFAULT_LIMB, bin=binNumber)    

            elif c=='m':
                print 'Run Debug method - currently testing IK for Tote'
                controller.testStowIK(limb=DEFAULT_LIMB)
                # stderr.write('Don')   

            elif c=='a':
                print 'Render Bin/Tote Content - Press Bin Letter or T for tote on GUI. Press X to cancel'
                bin_letter = command_queue.get()
                while bin_letter is None:
                    bin_letter = command_queue.get()
                if bin_letter.lower()=='x':
                    print 'Canceled'
                    continue
                if not ( ('a'<=bin_letter.lower()<='l') or bin_letter.lower()=='t' ):
                    print 'Unrecognized Letter. Canceled'
                    continue
                print 'Render Bin/Tote Content - Press 1 to render objects including bin/tote, 2 to render only objects (bin/tote subtracted)'
                flag = command_queue.get()
                while flag is None:
                    flag = command_queue.get()
                if flag in ['x', 'X']:
                    print 'Canceled'
                    continue
                if flag not in ['1', '2', '3']:
                    print 'Unrecognized Input. Canceled'
                    continue
                if bin_letter!='t':
                    a = time.time()
                    controller.renderBinContent(bin_letter.upper(), flag, limb=DEFAULT_LIMB)
                    print 'compute time', time.time() - a
                else:
                    controller.renderToteContent(flag=='1', limb=DEFAULT_LIMB)
            elif c=='n':
                print 'Get Pick Position for Bin - Press Bin Letter on GUI and Then Enter Object Information in Console. Press X to cancel'
                bin_letter = command_queue.get()
                while bin_letter is None:
                    bin_letter = command_queue.get()
                if bin_letter.lower()=='x':
                    print 'Canceled'
                    continue
                if not ( ('a'<=bin_letter.lower()<='l') or bin_letter.lower()=='t' ):
                    print 'Unrecognized Letter. Canceled'
                    continue
                controller.getPickPositionForPick(bin_letter.upper(), limb=DEFAULT_LIMB)
            elif c=='l':
                controller.getPickPositionForStow(limb=DEFAULT_LIMB)
            elif c=='v':
                if DEFAULT_LIMB=='left':
                    DEFAULT_LIMB='right'
                else:
                    DEFAULT_LIMB='left'
                print 'Limb is now ' + DEFAULT_LIMB
            elif c =='d':
                readPressure(limb)

            elif c == 'r':
                controller.moveToRestConfig()
            elif c == '`':
                print 'Press Bin Letter of the Bin to Calibrate'
                bin_letter = command_queue.get()
                while bin_letter is None:
                    bin_letter = command_queue.get()
                controller.calibratePerception(bin_letter.upper(), limb=DEFAULT_LIMB)
            elif c=='s':
                controller.calibrateShelf()
            elif c=='w':
                controller.calibrateCamera(limb=DEFAULT_LIMB)
            elif c=='f':
                visualizer.simworld.terrain(0).geometry().transform(*controller.perceptionTransform)
            elif c=='c':

                print 'View Tote or Bin without running perception'
                bin_letter = command_queue.get()
                while bin_letter is None:
                    bin_letter = command_queue.get()
                if not ( ('a'<=bin_letter.lower()<='l') or bin_letter.lower()=='t' ):
                    print 'Unrecognized Letter. Canceled'
                    continue
                if bin_letter.lower() == 't':
                    print 'viewing tote'
                    controller.viewToteAction(limb=DEFAULT_LIMB, doPerception=False)
                else:
                    print 'viewing bin ', bin_letter.lower()
                    controller.viewBinAction('bin_'+bin_letter.upper(), limb=DEFAULT_LIMB, doPerception=False)

            elif c=='g':
                visualizer.simworld.terrain(0).geometry().transform(*controller.perceptionTransformInv)
            elif c == '1':
                print 'Do complete Stow Task - Press Bin Letter for bin to be stowed to. Press X to cancel'
                bin_letter = command_queue.get()
                while bin_letter is None:
                    bin_letter = command_queue.get()
                if not ( ('a'<=bin_letter.lower()<='l') ):
                    print 'Unrecognized Letter. Canceled'
                    continue
                controller.runPickFromTote(limb=DEFAULT_LIMB, bin='bin_'+bin_letter.upper())

            elif c == '2':
                controller.moveToRestConfig()
                controller.waitForMove()
                #controller.fulfillOrderAction(orderList)

                print 'Do complete pick task - Press Bin Letter. Press X to cancel'
                bin_letter = command_queue.get()
                while bin_letter is None:
                    bin_letter = command_queue.get()
                if bin_letter.lower()=='x':
                    print 'Canceled'
                    continue

                controller.runPickFromBin(limb = DEFAULT_LIMB, bin='bin_'+bin_letter.upper())

            elif c =='+':
                controller.runPickingTask()

            elif c =='=':
                controller.runStowingTask()


                # print "================================"
                # print "Bin Order:", binList
                # print "Object Order:", orderList
                # print "Single Item?:", singleItemList
                # print "================================\n"


                # #binList = ['A','B','C','D','E','F','G','H','I','J','K','L']
                # global binIndex 
                # binIndex = 0
                # # for i in range(len(binList)):
                #     controller.viewBinAction(binList[i])
                #     controller.scoopAction()
                #     controller.move_spatula_to_center()
                #     controller.move_gripper_to_center()
                #     controller.graspAction()
                #     controller.placeInOrderBinAction()
                #     controller.viewBinAction(binList[i])
                #     controller.unscoopAction()
                #     #controller.moveToRestConfig()
                #     binIndex += 1
            elif c=='q':
                turnOffVacuum('left')
                turnOffVacuum('right')
                break
            elif c=='b':
                print 'Get Bin Bounds - Press Bin Letter on GUI. Press X to cancel'
                bin_letter = command_queue.get()
                while bin_letter is None:
                    bin_letter = command_queue.get()
                if bin_letter.lower()=='x':
                    print 'Canceled'
                    continue
                print controller.getBounds('bin_'+bin_letter.upper())
            elif c=='o':
                controller.calibrateVacuum(DEFAULT_LIMB)
            elif c=='/':
                controller.printEndEffectorPosition(DEFAULT_LIMB)
        else:
            print "Waiting for command..."
            time.sleep(0.1)

    print "Done"

def load_apc_world():
    """Produces a world with only the Baxter, shelf, and ground plane in it."""
    world = robotsim.WorldModel()

    print "Loading simplified Baxter model..."
    # world.loadElement(os.path.join(model_dir,"baxter_with_new_spatula_col.rob"))
    # world.loadElement(os.path.join(model_dir,"baxter_with_new_spatula_col2.rob"))
    #world.loadElement(os.path.join(model_dir,"baxter_with_new_spatula_col3.rob"))
    world.loadElement(os.path.join(model_dir,KLAMPT_MODEL))
    print "Loading shelf model..."
    world.loadElement(os.path.join(model_dir,"Amazon_Picking_Shelf.STL"))

    print "Loading plane model..."
    world.loadElement(os.path.join(model_dir,"plane.env"))

    #shift the Baxter up a bit (95cm)
    Rbase,tbase = world.robot(0).link(0).getParentTransform()
    world.robot(0).link(0).setParentTransform(Rbase,(0,0,0.95))
    world.robot(0).setConfig(world.robot(0).getConfig())

    #translate pod to be in front of the robot, and rotate the pod by 90 degrees
    t_obj_shelf = [0.45,0,0]
    t_shelf = [0,0,0]


    print ground_truth_shelf_xform
    # world.terrain(0).geometry().transform(xform_with_scale[0], xform_with_scale[1])
    #world.terrain(0)
    newTransform = se3.mul( (vectorops.div(ground_truth_shelf_xform[0], 1000), vectorops.sub(ground_truth_shelf_xform[1],[-2,-3,0])), ([1,0,0,0,0,1,0,-1,0], [0,0,0]))
    reorient = se3.mul( (ground_truth_shelf_xform[0], vectorops.sub(ground_truth_shelf_xform[1], [-2,-2.56,0])), ([1,0,0,0,0,1,0,-1,0], [0,0,0])) 
    #note: shift occurs in reorient because STL file is aligned with left side while shelf is aligned with center


    calibration = ([1, 0, 0, 0, 1, 0, 0, 0, 1], [-1.970, -2.546, -0.03])
    # perceptionTransform = ([ 0.99631874,  0.01519797, -0.08436826, -0.01459558,  0.9998634, 0.00775227, 0.08447454,   -0.00649233,    0.99640444], [-0.06180821,  0.0082858,  -0.00253027])


    # ([ 0.99631874, -0.01459558,  0.08447454, 0.01519797,  0.9998634,  -0.00649233, -0.08436826,  0.00775227,  0.99640444], [-0.06180821,  0.0082858,  -0.00253027])


    world.terrain(0).geometry().transform( newTransform[0], newTransform[1])
    world.terrain(0).geometry().transform(calibration[0], calibration[1])

    testingTransform = (so3.rotation([0,0,1], INIT_DEGREE_OFFSET*math.pi/180), [0,0,0] )
    world.terrain(0).geometry().transform(*testingTransform)
    knowledgeTransform = (so3.rotation([0,1,0], INIT_DEGREE_OFFSET*math.pi/180), [0,0,0] )

    #knowledge.shelf_xform = se3.mul(reorient, calibration)
    knowledge.shelf_xform = se3.mul(calibration, reorient)
    knowledge.shelf_xform = se3.mul(testingTransform, knowledge.shelf_xform)


    # world.terrain(0).geometry().transform(*perceptionTransform)

    #initialize the shelf xform for the visualizer and object
    #ground_truth_shelf_xform = se3.mul(Trel,reorient)
    return world

def readPressure(limb):
    with open(PRESSURE_FILE_PREFIX + limb.upper()+'.pkl', "rb") as f:
        valList = pickle.load(f)
        pressureAverage = float(valList[0])
        print "Vacuum Pressure: ",pressureAverage, "Threshold:", PRESSURE_THRESHOLD 
        if pressureAverage <= PRESSURE_THRESHOLD:
            "pressure dropped below threshold"
            return True
        return False

def myCameraSettings(visualizer):
    visualizer.camera.tgt = [-1, -.5, -0.75]
    visualizer.camera.rot = [0,0.5,0.9]
    visualizer.camera.dist = 4
    visualizer.fov = 60
    visualizer.width *= 1
    visualizer.height *= 1
    return

def saveToFile(variable, fileName):
    # Saving the objects:
    with open(fileName, 'wb') as f:
        pickle.dump(variable, f)

def loadFromFile(fileName):
    # Getting back the objects:
    with open(fileName, 'rb') as f:
        return pickle.load(f)

def f_addr_to_i(f):
    return struct.unpack('I', struct.pack('f', f))[0]
    
def i_addr_to_f(i):
    return struct.unpack('f', struct.pack('I', i))[0]

def turnOffVacuum(limb):

    if limb =='left':
        command = 0
    if limb =='right':
        command = 2

    try:
        vacuumController.change_vacuum_state(command)
    except:
        print 'Error in vacuum Controller'

def turnOnVacuum(limb):
    if limb=='left':
        command = 1
    if limb == 'right':
        command = 3

    try:
        vacuumController.change_vacuum_state(command)
    except:
        print 'Error in vacuum Controller'


def pcl_float_to_rgb(f):
    i = f_addr_to_i(f)
    r = i >> 16 & 0x0000ff
    g = i >> 8 & 0x0000ff
    b = i >> 0 & 0x0000ff
    return r,g,b

if __name__ == "__main__":
    """The main loop that loads the planning / simulation models and
    starts the OpenGL visualizer."""
    # Declare the knowledge base
    global knowledge
    global ground_truth_shelf_xform

    ground_truth_shelf_xform=([0.009999333346666592, -0.9999000033332889, 0.00999983333416667, 0.9999500004166664, 0.009999833334166692, 7.049050118954173e-18, -9.99966667111252e-05, 0.009999333346666538, 0.9999500004166657], [1.4299999999999995, -0.05500000000000002, 3.469446951953614e-18])
    #ground_truth_shelf_xform=([1,0,0,0,1,0,0,0,1], [0,0,0])

    knowledge = KnowledgeBase()


    # shelf_xform_from_perception goes here
    # simply reveals the shelf xform

    knowledge.shelf_xform = ([1,0,0,0,1,0,0,0,1],[0,0,0])

    world = load_apc_world()


    simworld = load_apc_world()
    # load shelf objects (wire frames)
    # init_ground_truth()


    # load the resting configuration from klampt_models/baxter_rest.config
    global baxter_rest_config, bin_viewing_configs
    f = open(model_dir+'baxter_new_spatula_rest3.config','r')
    baxter_rest_config = loader.readVector(f.readline())
    f.close()

    bin_viewing_configs = {}

    # to be changed
    # these should be constants stored somewhere

    f = open(model_dir+'bin_viewing_configs/A.config','r')
    bin_viewing_configs['A'] = loader.readVector(f.readline())
    f.close()

    # Add initial joint values to additional joints
    n = world.robot(0).numLinks()
    if len(baxter_rest_config) < n:
        baxter_rest_config += [0.0]*(n-len(baxter_rest_config))
        print "# links in rest_config < # links in robot"
    simworld.robot(0).setConfig(baxter_rest_config)
    # world.robot(0).setConfig(baxter_rest_config)
    # set spatula center point (comes from baxter rest config)
    knowledge.center_point = simworld.robot(0).link(left_camera_link_name).getTransform()[1]


    # load the resting configuration from klampt_models/baxter_rest.config
    global baxter_startup_config
    f = open(model_dir+'baxter_rest.config','r')
    baxter_startup_config = loader.readVector(f.readline())
    f.close()

    # Add initial joint values to additional joints
    n = world.robot(0).numLinks()
    if len(baxter_startup_config) < n:
        baxter_startup_config += [0.0]*(n-len(baxter_startup_config))
        print "# links in rest_config < # links in robot"

    if PHYSICAL_SIMULATION:
        motion.setup(mode=config.mode,klampt_model=config.klampt_model,libpath="./")
        res = motion.robot.startup()
        if not res:
            print "Error starting up Motion Module"
            exit(1)
        time.sleep(0.1)
        q = motion.robot.getKlamptSensedPosition()
        simworld.robot(0).setConfig(q)
        world.robot(0).setConfig(q)
        # res = world.readFile(config.klampt_model)
        # if not res:
        #     raise RuntimeError("Unable to load Klamp't model ")
    else:
        simworld.robot(0).setConfig(baxter_startup_config)
        world.robot(0).setConfig(baxter_startup_config)

    print "Loading precomputed trajectories... (this may take a while)"

    bin_list = ['A','B','C','D','E','F','G','H','I','J','K','L']

    global loaded_trajectory
    loaded_trajectory = {}
    for bin_name in ['bin_'+c for c in bin_list]:
        loaded_trajectory[bin_name] = loadFromFile(TRAJECTORIES_PATH+bin_name)
        loaded_trajectory[bin_name+"_spatula_to_center"] = loadFromFile(TRAJECTORIES_PATH+bin_name+"_spatula_to_center")
    loaded_trajectory['gripper_to_center'] = loadFromFile(TRAJECTORIES_PATH+"gripper_to_center")
    loaded_trajectory['move_to_order_bin'] = loadFromFile(TRAJECTORIES_PATH+"move_to_order_bin")


    '''
    global LOADED_PHYSICAL_TRAJECTORY
    for bin_name in ['BIN_'+c for c in bin_list]:
        try:
            LOADED_PHYSICAL_TRAJECTORY["CAMERA_TO_"+bin_name+'_RIGHT'] = eval('CAMERA_TO_' + bin_name+'_RIGHT')
            LOADED_PHYSICAL_TRAJECTORY["CAMERA_TO_"+bin_name+'_LEFT'] = eval('CAMERA_TO_' + bin_name+'_LEFT')
        except:
            print "failed to load"

    for bin_letter in bin_list:
        try:
            LOADED_PHYSICAL_TRAJECTORY['VIEW_TO_GRASP_'+bin_letter+'_RIGHT'] = eval('VIEW_TO_GRASP_'+bin_letter+'_RIGHT')
            LOADED_PHYSICAL_TRAJECTORY['VIEW_TO_GRASP_'+bin_letter+'_LEFT'] = eval('VIEW_TO_GRASP_'+bin_letter+'_LEFT']
            LOADED_PHYSICAL_TRAJECTORY['GRASP_TO_STOW_'+bin_letter+'_RIGHT'] = eval('GRASP_TO_STOW_'+bin_letter+'_RIGHT']
            LOADED_PHYSICAL_TRAJECTORY['GRASP_TO_STOW_'+bin_letter+'_LEFT'] = eval('GRASP_TO_STOW_'+bin_letter+'_LEFT']
        except:
            print "failed to load"
    '''

    #run the visualizer



    visualizer = MyGLViewer(simworld,world)
    myCameraSettings(visualizer)
    visualizer.run()
