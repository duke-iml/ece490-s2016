#!/usr/bin/python

# NOTE: Key sequence to control the baxter as desired:
#       bin number (A~L) --> scoop (S) --> move spatula to center (N) --> move gripper to center (M)
#       --> grasp object (X) --> place object in order bin (P) --> bin number (A~L) --> unscoop (Y) --> return to starting position (N)


# TODO:
        # JSON Parser
        # Pyserial to 1) turn on/off vacuum; 2) control spatula; 3) read vacuum pressure
        # Communicate with perception group


#
import sys, struct, time, json

sys.path.insert(0, "../../common")
sys.path.insert(0, "..")


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
from Trajectories.grasp_to_rest import *
#-----------------------------------------------------------
#Imports require internal folders

from Motion import motion
from Motion import config
from Motion import motion_debouncer


from Group2Helper import apc
from Group2Helper.baxter import *
from Group2Helper.motionPlanner import *
from Group2Helper import Vacuum_Comms
#================================================================
# End of Imports


NO_SIMULATION_COLLISIONS = 0
FAKE_SIMULATION = 0
PHYSICAL_SIMULATION = 1




#============================================================
# configuration variables



PATH_DICTIONARY = {}
RUN_EXP_IMMEDIATELY = False

try:
    file = open('data.json', 'rw') 
    PATH_DICTIONARY = json.load(file)
    file.close()
except:
    print 'Path Dictionary failed to load'



MAX_RATING = 4

ALL_ARDUINOS = 0
MOTOR = 0 or ALL_ARDUINOS
VACUUM = 0 or ALL_ARDUINOS

WAIT_TIME = 2

SPEED = 3

REAL_SCALE = False
REAL_CAMERA = False
REAL_JSON = False
REAL_PRESSURE = False

TASK = 'pick'
#TASK = 'stow'

SHELF_STATIONARY = False

PRESSURE_THRESHOLD = 850

SKIP_GRASP_FROM_TOTE= False
SKIP_STOWING_INPUT = False


INIT_DEGREE_OFFSET = 0

TOTE_BOUNDS = [[],[]]

TOTE_BOUNDS[0] = [0.7560075384967612, -0.21018285279301704, 0.34623535198332]
TOTE_BOUNDS[1] = [0.49803990865039477, 0.28197204467225134, 0.5341581622402093]


TERRAIN_CALIBRATION = None

SHELF_CALIBRATION = ([1, 0, 0, 0, 1, 0, 0, 0, 1], [-1.970, -2.546, -0.03])

PERCEPTION_OFFSET_DISTANCE = .02

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

MIDDLE_BINS = ['bin_B', 'bin_E', 'bin_H', 'bin_K']

PICK_TIME = 900
STOW_TIME = 900


visualizer = None

if REAL_SCALE:
    from Sensors import scale
    from Group2Helper import stowHandler
    if TASK == 'stow':
        stowHandler = stowHandler.stowHandler(JSON_STOW_INPUT_FILE)

if REAL_JSON:
    # JSON parser
    from Group2Helper import pickHandler
    pickHandler = pickHandler.pickHandler(JSON_PICK_INPUT_FILE)

    # Order list. Can be parsed from JSON input
    global binList
    global singleItemList
    global rightTrueList
    global orderList
    global binMap
    global easinessList
    binMap=pickHandler.binMap
    (binList, orderList, rightTrueList, easinessList) = pickHandler.workBinOrder()
    print 'binList is ', binList

if REAL_CAMERA:
    # Perception
    from perception import perception
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


left_arm_geometry_indices = [15,16,17,18,19,21,22]
right_arm_geometry_indices = [35,36,37,38,39,41,42]

# The transformation of the order bin
order_bin_xform = (so3.identity(),[0.65,-0.55,0])
order_bin_bounds = ([-0.2,-0.4,0],[0.2,0.4,0.7])

SCAN_STATE = 'scan'
GRASP_STATE = 'grasp'
GRASP_PREPPED_STATE = 'graspPrepped'
REST_STATE = 'ready'
STOW_STATE = 'stow'

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
        self.robotModel.setConfig(myConfig)
        self.lock.release()
    def appendMilestoneLeft(self, destination, dt=2, endvelocity=None):
        self.lock.acquire()
        myConfig = self.robotModel.getConfig()
        if len(destination) < len(myConfig):
            for i in range(len(left_arm_geometry_indices)):
                myConfig[left_arm_geometry_indices[i]] = destination[i]  
        if endvelocity == None: self.controller.addMilestoneLinear(myConfig)
        else: self.controller.addMilestone(myConfig,endvelocity)
        self.robotModel.setConfig(myConfig)
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
        
        self.robotModel = robotModel
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
        #print "appending milestone right...len(destination)=", len(destination)
        if len(destination) == 7:
            #while len(destination) < len(self.right_arm_indices):
            #    destination = destination + [0]
            if not motion.robot.right_mq.appendLinear(dt, destination): raise RuntimeError()
        else:
            #print 'Desitnation is: ', [destination[v] for v in self.right_arm_indices[:7]]
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

        self.recorder = None

        print "initialized picking controller number of robots = ", self.world.numRobots()

        self.controller = robotController
        #self.planner = LimbPlanner(self.world, knowledge)

        self.left_arm_indices = [15,16,17,18,19,21,22]
        self.right_arm_indices = [35,36,37,38,39,41,42]
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

        self.debugPoints = []
        self.debugFailedPoints = []
        self.debugFailedLeftPoints = []
        self.debugFailedRightPoints = []
        self.bin_content_cloud = None
        self.tote_cloud = None
        self.targetOffsetPoints = []
        self.rightStowPoints = []
        self.leftStowPoints = []
        self.pick_pick_pos = None
        self.stow_pick_pos = None
        self.all_stow_pick_poss = None
        self.frames = []

        # calibration constants
        #==========================================================================================================================

        self.left_bin = None
        self.right_bin = None


    def runHIExperiment(self):


        while(1):

            goodInput = ['1','2','3','4','q']

            choice = raw_input("How would you like to move? ")

            if choice in goodInput:

                if choice == '1':
                    self.supplyBoxes(1)
                if choice == '2':
                    self.supplyBoxes(2)
                if choice == '3':
                    self.supplyBoxes(3)
                if choice == '4':
                    self.supplyBoxes(4)
                if choice == 'q':
                    break
            else:
                print 'Error, command not recognized. Type q to exit'


    def supplyBoxes(self, choice):

        '''
        1 - Suply the right box with the right arm
        2 - Supply the middle box with the right arm
        3 - Supply the middle box with the left arm
        4 - Supply the left box with the left arm   
        '''

        limb = None

        if choice == 1 or choice == 2:
            limb = 'right'
        elif choice == 3 or choice == 4:
            limb = 'left'
            #messed up saving names
            if choice == 4:
                choice = 2
        else:
            raise Exception('Invalid entry')

        path1_name = limb.upper()+'_SUPPLY'
        path2_name = limb.upper()+'_DROP_'+str(choice)

        self.moveArm(limb = limb, path_name = path1_name)
        self.waitForMove()
        self.moveArm(limb = limb, path_name = path2_name)
        self.waitForMove()
        
        #move back
        self.moveArm(limb = limb, path_name = path2_name, reverse = True)
        self.waitForMove()
        self.moveArm(limb = limb, path_name = path1_name, reverse = True)
        self.waitForMove()



    def getWorld(self):
        return self.world

    def updatePathDictionary(self):

        global PATH_DICTIONARY


        try:
            file = open('data.json', 'rw') 
            PATH_DICTIONARY = json.load(file)
            file.close()
        except:
            print 'Path Dictionary failed to load'


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

 #====================================================
    '''
    DEBUG methods
    '''
   #=======================================================

    # Process for stowing:
    '''TODO

    improve path for placeInBinAction 
    '''


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



        if position is None:
            position = self.stow_pick_pos

        # if still none - get input

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



                self.numStowFails = self.numStowFails + 1
                turnOffVacuum(limb)
                time.sleep(3)
                #let it fall off
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
        #end state = evaluateObject

        #TODO - 

        path_name = 'Q_EVAL_SCALE_'+limb.upper()
        print "self.moveArm(",path_name,")...",
        self.moveArm(limb = limb, path_name = path_name, finalState = 'evaluateObject')

        self.waitForMove()

        time.sleep(2)

        if REAL_SCALE:
            (self.stowItems, self.pickBin, self.tilt) = stowHandler.pickWhichObj(limb, True)
            if self.pickBin==None:
                print "No target object in the weight range"

                self.moveArm(limb = limb, path_name = 'Q_REPLACE_'+limb.upper())
                self.waitForMove()
                
                turnOffVacuum(limb)

                time.sleep(2)
                self.moveArm(limb = limb, path_name = path_name)
                self.waitForMove()
                return False
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
        if self.tilt:
            self.tilt = False
            return self.placeHardcoded(limb)
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
        
        path = [eval(path_name)[-1]]

        if self.moveArm(limb = limb, path = path, reverse=True):
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
                print "moveArm(",path_name,")...",
                if self.moveArm(limb=limb, path_name = path_name, finalState = 'toteStow'):
                    print "success"
                    self.waitForMove()
                print 'Error in moving into the bin'
                return False

            #then actually move into bin a bi
        else:
            print 'Error in moving from grasping tote object to stowing tote object'
            return False

    # This can be cleaned up quite a bit
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
            target2 = knowledge.getBinWorldPosition(bin_name = 'bin_'+bin, localPoint = [.5, .3, .35])
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


                return False
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
            target3 = knowledge.getBinWorldPosition(bin_name = 'bin_'+bin, localPoint = [.5, .3, .7])
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
                return True
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
                return True
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


    # Stowing Debug

    def placeHarcoded(self, limb):


        path_name = 'GRASP_TO_STOW_B_' + limb.upper() + '_HARDCODED'
        self.moveArm(limb = limb, path_name = path_name, reverse=True, finalState = 'placeInBin')

        success = False

        self.waitForMove()
        time.sleep(.5)

        if REAL_PRESSURE:
            if readPressure(limb):
                success = True
                stowHandler.updateBin(bin='bin_B',item=self.stowItems)

        time.sleep(2)
        turnOffVacuum(limb)

        self.moveArm(limb = limb, path_name = path_name, finalState = 'toteStow')

        return success

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

        point = se3.apply(se3.inv(self.perceptionTransform), point)

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

    def evaluateShelf(self):


        bin,limb = self.getMostEmptyBin()

        print 'evaluating bin ', bin
        self.viewBinAction(b = bin, limb = limb, doPerception = False)

        self.waitForMove()
        time.sleep(3)

        self.calibratePerception(bin[4].upper(), limb)
        
        self.moveToRestConfig()
        self.waitForMove()

    def runPickFromBin(self, bin, limb):

        rating =easinessList.pop(0)
        if rating < MAX_RATING-1:
            rating = rating+1


        self.moveToRestConfig()
        self.waitForMove()
        if len(bin) == 1:
            bin = 'bin_'+bin.upper()

        if self.viewBinAction(b=bin, limb = limb, shortcut = self.shortcutViewBin):
            self.shortcutViewBin = False
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
                else:
                    print 'Error in graspFromBinAction'
            else:
                print 'Error in prepGraspFromBinAction'
        else:



            shortCutList = eval(limb.upper()+'_BIN_VIEW_SHORTCUTS')[bin]
            
            print shortCutList
            
            if REAL_JSON:
                print self.binQueue[0]

                if self.binQueue[0] in shortCutList:
                # if we can shortcut
                    rating =easinessList.pop(0)
                    rating = rating+1
                # move to the desired bin 
                # return 
                # call this function again with viewBinActio                

                    self.updateQueue(bin, limb, rating)
                    return False
                else:
                    self.shortcutViewBin = False


            print 'Error in View Bin Action'

        if self.currentPickObject is not None:

            print 'readding things to queue'

            if REAL_JSON:

                self.updateQueue(bin, limb, rating)

        else:
            rating = easinessList.pop()


        turnOffVacuum(limb)
        self.moveToRestConfig()
        self.waitForMove()

        return False

    def viewBinAction(self,b, limb, doPerception=True, shortcut = False):
        self.waitForMove()

        if LOAD_PHYSICAL_TRAJECTORY:

            print limb
            if limb is not None:
                if b in apc.bin_names:
                    path_name = 'CAMERA_TO_' + b.upper() + '_'+limb.upper()
                    #EX: 'CAMERA_TO_BIN_A_LEFT'
                    if shortcut:

                            # if we want to shortcut, just move to the bin position (we were previously at the other bin)
                            scan_name = 'Q_SCAN_'+b.upper() + '_'+limb.upper()+'_NEW'
                            try:
                                eval(scan_name)
                            except:
                                print 'Failed to evaluate new scan name -reverting to old scan name'                           
                                scan_name = 'Q_SCAN_'+b.upper() + '_'+limb.upper()

                            self.moveToOffset(limb, q_name = scan_name)                    
                            self.waitForMove()
                            time.sleep(5)
                            if limb == 'left':
                                self.left_bin = b
                            elif limb == 'right':
                                self.right_bin = b
                            if REAL_CAMERA:
                                if doPerception:
                                    if REAL_JSON:
                                        if self.full_run:
                                            self.currentPickObject = orderList.pop(0)
                                        else:
                                            index =  binList.index(b)
                                            self.currentPickObject = orderList[index]
                                    
                                    if self.getPickPositionForPick(bin_letter=b[4],target_item=self.currentPickObject,possible_items=binMap[b], limb=limb):
                                        return True
                                    else:
                                        print 'perception failed'
                                        return False
                            else:
                                return True
                    else:
                        #typical motion
                        if self.moveArm(limb, statusConditional='ready', path_name=path_name, finalState='scan'):
                            self.waitForMove()
                            scan_name = 'Q_SCAN_'+b.upper() + '_'+limb.upper()+'_NEW'
                            try:
                                eval(scan_name)
                            except:
                                print 'Failed to evaluate new scan name -reverting to old scan name'                           
                                scan_name = 'Q_SCAN_'+b.upper() + '_'+limb.upper()

                            self.moveToOffset(limb, q_name = scan_name)                    
                            self.waitForMove()
                            time.sleep(5)
                            if limb == 'left':
                                self.left_bin = b
                            elif limb == 'right':
                                self.right_bin = b
                            if REAL_CAMERA:
                                if doPerception:
                                    if REAL_JSON:
                                        if self.full_run:
                                            self.currentPickObject = orderList.pop(0)
                                        else:
                                            index =  binList.index(b)
                                            self.currentPickObject = orderList[index]                                  
                                    if self.getPickPositionForPick(bin_letter=b[4],target_item=self.currentPickObject,possible_items=binMap[b], limb=limb):
                                        return True
                                    else:
                                        print 'perception failed'
                                        return False
                            else:
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
            graspDirection = ['up', [0,1,0]]
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
                    if REAL_PRESSURE:
                        time.sleep(.5)
                        #let stuff catch up
                        if readPressure(limb):
                            pickHandler.updateBin(bin, self.currentPickObject)
                            return True
                        else:
                            print 'did not grasp item'
                            return False
                    return True

                else:
                    print 'Movement Unsuccessful'

                    if REAL_PRESSURE:
                        time.sleep(.5)
                        # let stuff catch up
                        if readPressure(limb):
                            pickHandler.updateBin(bin, self.currentPickObject)
                            print 'still got something'
                            return True
                        else:
                            try:
                                turnOffVacuum(limb)
                            except:
                                print 'Error in vacuum comms'
                            return False
                    #TODO
                    return False
                    #return False
            elif graspDirection[0] =='side':
                try:
                    position = perception.getPickPositionForPick()
                except:
                    print 'Error perception code for point not set up'

                if self.moveToObjectInBinFromSide(position=position, limb=limb, normal=vectorops.unit(graspDirection[1]), step=-1):

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

                        #check pressure
                        #if true
                        if REAL_PRESSURE:
                            if readPressure(limb):
                                pickHandler.updateTote(self.currentPickObject)
                            else:
                                self.currentPickObject = None
                                print 'dropped object'
                                return False

                        try:
                            time.sleep(2)
                            #wait for two seconds
                            #maybe forcewait until it's in stow state

                            turnOffVacuum(limb)

                            time.sleep(2)
                            # wait for object to drop
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
 
    #TODO Cleanup this method
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

            #check object is in bin

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
                ikGoal = self.buildIKGoalSuctionDown(limb = limb, target=target1, targetFurtherBack=True)
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

            if step==3:
                #keep the x, y - throw out the z

                #check to make sure target is actually in bin
                target3 = knowledge.getBinMidCenterTop(bin)
                target3[0] = target[0]
                target3[1] = target[1]
                ikGoal = self.buildIKGoalSuctionDown(limb = limb, target=target3, targetFurtherBack=True)
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
                ikGoal = self.buildIKGoalSuctionDown(limb = limb, target=target4, targetFurtherBack=True)
                #ik to top center of bin, normal to the shelf
                #use ik seed and knowledge of shelf
                # constraintst: suction cup down, vacuum/wrist forward direction in direction of shelf
                #print 'trying step4 ik'
                step4 = self.simpleIK(limb=limb, goal=ikGoal)
                if step4 is None:
                    print 'ik failed in step4 of moveToObjectInBinFromTop'
                    self.sendPath(path=[step1], limb=limb)
                    self.waitForMove()
                    return False
                self.debugPoints.append(target4)
                self.sendPath(path=[step4], limb = limb)
                
                time.sleep(1)
                self.waitForMove()
                #print 'Got to step 4'


                time.sleep(1)
                if REAL_PRESSURE:
                    if readPressure(limb):
                        # I got something
                        print 'chose step 5'
                        step = 5
                    else:
                        print 'chose step 6'
                        step = 6



            if step==5:
                #pull back out

                self.sendPath(path=[step3] , limb=limb)
                self.waitForMove()
                self.sendPath(path=[step1],limb=limb)
                self.waitForMove()

                #check that object is still held
                return True

            if step==6:

                #keep the x, y - throw out the z
                target6 = vectorops.sub(target, [0,0,.005])
                    #go down 1/2cm
                #check to make sure target is actually in bin
                ikGoal = self.buildIKGoalSuctionDown(limb = limb, target=target6, targetFurtherBack=True)
                #ik to top center of bin, normal to the shelf
                #use ik seed and knowledge of shelf
                # constraintst: suction cup down, vacuum/wrist forward direction in direction of shelf
                #print 'trying step6 ik'
                step6 = self.simpleIK(limb=limb, goal=ikGoal)
                if step6 is None:
                    print 'ik failed in step6 of moveToObjectInBinFromTop'
                    
                    self.sendPath(path =[step3],  limb=limb)
                    self.waitForMove()
                    self.sendPath(path=[step1], limb=limb)
                    self.waitForMove()
                    return False
                self.debugPoints.append(target6)
                self.waitForMove()
                self.sendPath(path=[step6], limb = limb)
                self.waitForMove()

                step = 7

            if step == 7:

                self.sendPath(path = [step4], limb=limb)
                self.waitForMove()
                self.sendPath(path =[step3],  limb=limb)
                self.waitForMove()
                self.sendPath(path = [step1], limb=limb)
                self.waitForMove()


                return True

            #get back out from 

        # print 'physical simulation is on'
        # PHYSICAL_SIMULATION = 1

    #TODO clean up this method
    def moveToObjectInBinFromSide(self, position, limb, normal=None, step=0, naiive = True):
        #Assumes we have moved so that we are in a configuration where we are ready to pick up things from a bin
        #Assumes we have scanned the bin already and determined the x,y,z of where we want to move
        #Assumes we have determined we want to pick up the object from above

        #We need to calculate the shelf normal
        #We want to aim into the shelf with the suction cup down and enter with the wrist pointing in the direction of the normal of the shelf


        DEFAULT_GOAL = [1.1151453560415345, -0.046864004769163026, 1.1370113707939946]
        DEFAULT_NORMAL = [0, 1, 1] #45 degree angle

        step1 =[]
        step2 = []
        step3 = []
        step4 = []
        step5 = []

        if position != None:
            self.pick_pick_pos = position
            target = position

        if naiive:
            #fix the suction cup's direction aim for the top of the bin, ik to a position slightly above the object
            #ik down
            #

            bin = None
            if limb =='left':
                bin = self.left_bin
            elif limb =='right':
                bin = self.right_bin
                
            # if not self.isInBin(bin=bin, point=checkPoint):
            #     print 'Error, point not in ', bin
            #     return False 

            try:
                target = self.pick_pick_pos
                assert self.pick_pick_pos is not None, 'Perception failed, falling back to DEFAULT'
            except:
                print 'perception not set up yet'
                target = DEFAULT_GOAL



            if normal is None:
                try:
                    normal = perception.getNormal()
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


            if step==1:
                #move to the top center of the bin
                target1 = knowledge.getBinFrontCenterTop(bin)
                self.debugPoints.append(target1)
                ikGoal = self.buildIKGoalSuctionDown(limb = limb, target=target1)
                #ik to top center of bin, normal to the shelf
                #use ik seed and knowledge of shelf
                # constraintst: suction cup down, vacuum/wrist forward direction in direction of shelf
                print target1
                print 'trying step1 ik'
                step1 = self.simpleIK(limb=limb, goal=ikGoal)
                if step1 is None:
                    print 'ik failed in step1 of moveToObjectInBinFromSide'
                    self.sendPath(path=[step1], limb=limb)
                    return False
                self.sendPath(path=[step1], limb=limb)
                time.sleep(2)
                self.waitForMove()
                
                step =2

            if step==2:
                target2 = knowledge.getBinFrontCenterTop(bin)
                self.debugPoints.append(target)

                #dummy = knowledge.applyShelfXform([target[0], 0, 0])
                dummy = se3.apply(self.perceptionTransform, [target[0], 0, 0])
                target2[0] = dummy[0]

                #target should be the middle of the bin, near the top but only so far in as would be indicated by x

                ikGoal = self.buildIKGoalSuctionDown(limb = limb, target=target2)
                #ik to top center of bin, normal to the shelf
                #use ik seed and knowledge of shelf
                # constraintst: suction cup down, vacuum/wrist forward direction in direction of shelf
                print target
                print 'trying step2 ik'
                step2 = self.simpleIK(limb=limb, goal=ikGoal)
                if step2 is None:
                    print 'ik failed in step2 of moveToObjectInBinFromSide'
                    self.sendPath(path=[step1], limb=limb)
                    return False
                self.sendPath(path=[step2], limb=limb)
                self.waitForMove()
                
                step =3


            if step==3:
                #keep the x, y - throw out the z

                #check to make sure target is actually in bin

                target3 = target

                self.debugPoints.append(vectorops.add(target, vectorops.mul(vectorops.unit(normal), 0.05)))
                ikGoal = self.buildIKGoalSuctionNormal(limb = limb, target=target3, normal = vectorops.unit(normal), normalDisplacement = 0.05, targetFurtherBack=True)
                print 'trying step3 ik'
                step3 = self.simpleIK(limb=limb, goal=ikGoal)
                if step3 is None:
                    print 'ik failed in step3 of moveToObjectInBinFromSide'
                    self.sendPath(path=[step1], limb=limb)
                    return False
                self.sendPath(path=[step3], limb=limb)
                turnOnVacuum(limb)
                self.waitForMove()

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
                ikGoal = self.buildIKGoalSuctionNormal(limb = limb, target=target4, normal = vectorops.unit(normal), normalDisplacement = 0, targetFurtherBack=True)
                #ik to top center of bin, normal to the shelf
                #use ik seed and knowledge of shelf
                # constraintst: suction cup down, vacuum/wrist forward direction in direction of shelf

                print 'trying step4 ik'
                step4 = self.simpleIK(limb=limb, goal=ikGoal)
                if step4 is None:
                    print 'ik failed in step1 of moveToObjectInBinFromSide'
                    self.sendPath(path=[step2], limb=limb)
                    self.waitForMove()
                    self.sendPath(path = [step1], limb=limb)
                    self.waitForMove()
                    return False
                self.sendPath(path=[step4], limb = limb)
                self.waitForMove()
                

                print 'Got to step 4'

                step = 5

            if step ==5:
                self.sendPath(path = [step3], limb=limb)
                self.waitForMove()
                self.sendPath(path = [step2], limb = limb)
                self.waitForMove()
                self.sendPath(path=[step1], limb=limb)
                self.waitForMove()
                #check that object is still held

                if REAL_PRESSURE:
                    time.sleep(.5)
                    if readPressure(limb):
                        return True
                    else:
                        time.sleep(1)
                        turnOffVacuum(limb)
                        return False
                else:
                    return True


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
                pathL = self.getLeftRestPath()
                self.moveArm(path = pathL, limb = 'left', finalState = 'ready')

            elif limb == 'right':
                pathR = self.getRightRestPath()
                self.moveArm(path = pathR, limb = 'left', finalState = 'ready')
            self.waitForMove()

        else:
            path = [baxter_startup_config, baxter_rest_config]
            #self.sendPath(path)

            self.controller.setMilestone(baxter_rest_config)
            self.waitForMove()
        #print "Done"



    def getLeftRestPath(self):
        path =  [[self.simworld.robot(0).getConfig()[v] for v in self.left_arm_indices]]

        print 'left state is ', self.stateLeft


        if(self.stateLeft == 'scan'):
            lPath = eval('CAMERA_TO_'+ self.left_bin.upper()+'_LEFT')[::-1]
            #rPath = eval('CAMERA_TO_'+self.right_bin+'_RIGHT')[::-1]

            if lPath !=[]:
                for milestone in lPath:
                    path.append(milestone)
      
        elif(self.stateLeft == 'grasp' or self.stateLeft == 'graspPrepped' ):
            #not set up yet
            # go to store then rest
            lPath = eval('GRASP_TO_REST_' + self.left_bin.upper()[4]+'_LEFT')

            if lPath !=[]:
                for milestone in lPath:
                    path.append(milestone)

        elif self.stateLeft == 'stow':
            #go right ahead through the code

            path.append(eval('Q_OUT_OF_TOTE_LEFT'))

        elif self.stateLeft == 'evaluateObject' or self.stateLeft == 'toteStow' or self.stateLeft == 'prepTote':
            lPath = (eval('PREP_TOTE_STOW_LEFT'))[::-1]
            if lPath !=[]:
                for milestone in lPath:
                    path.append(milestone)  

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

    def getRightRestPath(self):
        path = [[self.simworld.robot(0).getConfig()[v] for v in self.right_arm_indices]]

        print 'right state is ', self.stateRight
        

        if(self.stateRight == 'scan'):
            rPath = eval('CAMERA_TO_'+ self.right_bin.upper()+'_RIGHT')[::-1]
            #rPath = eval('CAMERA_TO_'+self.right_bin+'_RIGHT')[::-1]
            if rPath !=[]:
                for milestone in rPath:
                    path.append(milestone)
      
        elif(self.stateRight == 'grasp' or self.stateRight == 'graspPrepped'):
            #not set up yet
            # go to store then rest
            rPath = eval('GRASP_TO_REST_' + self.right_bin.upper()[4]+'_RIGHT')
            if rPath != []:
                for milestone in rPath:
                    path.append(milestone)

        elif self.stateRight == 'stow':
            #go right ahead through the code
            path.append(eval('Q_OUT_OF_TOTE_RIGHT'))


        elif self.stateRight == 'evaluateObject' or self.stateRight == 'prepTote' or self.stateRight =='toteStow':
            rPath = (eval('PREP_TOTE_STOW_RIGHT'))[::-1]
            if rPath !=[]:
                for milestone in rPath:
                    path.append(milestone)               

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
                return True
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

        while len(pathL) < len(pathR):
            pathL.append(pathL[-1])          
    
        while len(pathR) < len(pathL):
            pathR.append(pathR[-1])
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

                if path_name in PATH_DICTIONARY:
                    path = PATH_DICTIONARY[path_name][1][1]
                else:
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


            if finalState is not None:
                self.stateLeft = finalState
            return True
        else:
            print "Error, arm is not in state ", statusConditional
            return False

    def moveRightArm(self, statusConditional=None, path_name=None, path=None, finalState=None, reverse=False):
        if(self.stateRight == statusConditional or statusConditional == None):# or self.stateRight in statusConditional):
            if path is None:

                if path_name in PATH_DICTIONARY:
                    path = PATH_DICTIONARY[path_name][2][1]
                else:
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

            #print 'sending path ', path

            if reverse:
                path = path[::-1]

            # if len(path)>=1:
            #     self.sendPath(path, limb='right', readConstants=True)
            # ^Hyunsoo's code - delinked the arms from moving simultaneously

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

            if finalState is not None:
                self.stateRight = finalState
        
            return True
        else:
            print "Error, arm is not in state ", statusConditional
            return False

    #=============================================================================================================================================

    def buildIKGoalSuctionDown(self,limb, target, offset=0.5, debug=False, targetFurtherBack = False):

        #appliedTransform = knowledge.shelf_xform
        appliedTransform = self.perceptionTransform

        if targetFurtherBack:
            target =  vectorops.add(target, so3.apply(appliedTransform[0],[PERCEPTION_OFFSET_DISTANCE,0,0]))
            self.targetOffsetPoints.append(target)

        targetZOffset = vectorops.add(target, [0,0,-offset])
        #targetAxisConstraint = so3.apply(knowledge.shelf_xform[0], vectorops.add(target, [offset, 0,0]))

        #targetAxisConstraint = vectorops.add(target, so3.apply(appliedTransform[0], [offset, 0,0]))
        targetAxisConstraint = se3.apply(appliedTransform,vectorops.add(target, [offset,0,0]))



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

    def buildIKGoalSuctionNormal(self, limb, target, normal, offset = 0.5, normalDisplacement = 0, debug=False, targetFurtherBack = False):

        #appliedTransform = knowledge.shelf_xform
        appliedTransform = self.perceptionTransform


        if targetFurtherBack:
            target =  vectorops.add(target, so3.apply(appliedTransform[0],[PERCEPTION_OFFSET_DISTANCE,0,0]))
            self.targetOffsetPoints.append(target)

        target = vectorops.add(target, vectorops.mul(normal, normalDisplacement))



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

    def setRecorder(self, recordIn):
        self.recorder = recordIn

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

    def calibrateShelf(self, index=0):
        #blocking
        global SHELF_CALIBRATION

        while(True):
            try:
                print 'Terrain', index, 
                input_var = raw_input(": Enter joint and angle to change to separated by a comma: ").split(',');
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
                TERRAIN_CALIBRATION[index]
                calibrate = (calibrateR, calibrateT)
                TERRAIN_CALIBRATION[index] = se3.mul(calibrate, TERRAIN_CALIBRATION[index])
                print 'calibration Overall = ', TERRAIN_CALIBRATION[index]
                self.simworld.terrain(index).geometry().transform( calibrate[0], calibrate[1] )
                #self.shelf_xform = se3.mul(calibrate, self.shelf_xform)

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
                calibrateR = eval('self.cameraTransform_'+limb.upper())[0]
                calibrateT = eval('self.cameraTransform_'+limb.upper())[1]

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
            if limb == 'left':
                self.cameraTransform_LEFT = ( calibrate[0], calibrate[1] )
            else:
                self.cameraTransform_RIGHT = ( calibrate[0], calibrate[1] )

            #print self.simworld.robot(0).link(limb + '_wrist').getTransform()
            totalCameraXform = self.getCameraToWorldXform(limb)

            print limb.upper(), ' camera transform is ', eval('self.cameraTransform_'+limb.upper())

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
        return se3.mul(self.simworld.robot(0).link(limb + '_wrist').getTransform(), eval('self.cameraTransform_'+limb.upper()))

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

        if self.picking_controller.recorder is not None:
            self.dt = 0.01
            self.picking_controller.recorder.fake_controller.sim.simulate(self.dt)
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

        if self.picking_controller.recorder is not None:

            recorder = self.picking_controller.recorder

            glEnable(GL_BLEND)

            glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
            glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,0,0,0.5])
            # only 1 robot in this case, but still use for-loop for generality
            for i in xrange(self.simworld.numRobots()):
                r = recorder.fake_controller.robotModel
                # q = self.sim.controller(i).getCommandedConfig()
                q = recorder.fake_controller.getCommandedConfig()
                r.setConfig(q)
                r.drawGL(False)
            
            glDisable(GL_BLEND)

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



            glDisable(GL_LIGHTING)
            glColor3f(.1, .8, .1)
            glPointSize(10)
            glBegin(GL_POINTS)

            for point in self.picking_controller.targetOffsetPoints:
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
                
                
        # draw the shelf and floor
        # if self.simworld.numTerrains()==0:
        #     for i in range(self.planworld.numTerrains()):
        #         self.planworld.terrain(i).drawGL()

        for i in xrange(self.simworld.numTerrains()):
            self.simworld.terrain(i).drawGL()

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
            # print 'Key code is', int(c)
            if c  == chr(27):
                #c == esc
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

    if RUN_EXP_IMMEDIATELY:
        controller.runHIExperiment()

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
                print 'V: Change Default Limb'
                print 'R: Move to Rest Configuration'
                print 'M: Move Gripper to Center'
                print '`: Run ICP to Find Transformation of Perturbed Shelf'
                print 'S: Calibrate Canonical (Unperturbed) Shelf Transformation'
                print 'W: Calibrate Camera Transformation Relative to Wrist'
                print 'U: Calibrate Vacuum Transform Relative to Wrist'
                print 'O: Begin recording functions'
                print 'G: Run Baxter Human Interaction Experiment'
                print 'C: See(C) the Tote'
                print 'B: Get the Bounds of a Bin'
                print '/: Get the global position of the end effector' 
                print 'Q: Quit'
                print 'D: Debug method - test IK for bins'
                print 'M: Debug method - test IK for tote'
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

           
            elif c=='d':
                print 'Run Debug method - currently testing IK for Bins'
                binNumber = raw_input('Enter bin name (ex. all, bin_A, ... ): ')
                controller.testBinIK(limb=DEFAULT_LIMB, bin=binNumber)    

            elif c=='m':
                print 'Run Debug method - currently testing IK for Tote'
                controller.testStowIK(limb=DEFAULT_LIMB)
                # stderr.write('Don')   

            elif c=='v':
                if DEFAULT_LIMB=='left':
                    DEFAULT_LIMB='right'
                else:
                    DEFAULT_LIMB='left'
                print 'Limb is now ' + DEFAULT_LIMB

            elif c == 'r':
                controller.moveToRestConfig()
            elif c=='s':
                controller.calibrateShelf()
            elif c=='w':
                controller.calibrateCamera(limb=DEFAULT_LIMB)
            elif c=='u':
                controller.calibrateVacuum(DEFAULT_LIMB)
            elif c =='o':
                myRecorder  = armRecorder.Recorder(controller, command_queue)
                controller.setRecorder(myRecorder)
                myRecorder.run()
                controller.setRecorder(None)
                controller.updatePathDictionary()
                
            elif c=='/':
                controller.printEndEffectorPosition(DEFAULT_LIMB)
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
                controller.runHIExperiment()

            elif c=='q':
                turnOffVacuum('left')
                turnOffVacuum('right')
                #break

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
    #print "Loading shelf model..."
    #world.loadElement(os.path.join(model_dir,"Amazon_Picking_Shelf.STL"))

    print "Loading plane model..."
    world.loadElement(os.path.join(model_dir,"plane.env"))
    leftWall = world.loadElement(os.path.join(model_dir, 'plane1.env'))

    rightWall = world.loadElement(os.path.join(model_dir, 'plane2.env'))

    table = world.loadElement(os.path.join(model_dir, 'plane3.env'))

    print world.numTerrains(), ' is the number of terrains loaded'


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


    factor = .1
    wallfactor = .25
    wall1Transform = ([wallfactor,0,0,0,0,wallfactor,0,wallfactor,0],[1.8,.7,.8])
    wall2Transform = ([wallfactor,0,0,0,0,wallfactor,0,wallfactor,0],[1.8,-.7,.8])
    tableTransform = ([factor, 0, 0, 0, factor, 0, 0, 0, factor],[1, 0, .75])

    global TERRAIN_CALIBRATION

    TERRAIN_CALIBRATION = [0]*world.numTerrains()

    TERRAIN_CALIBRATION[0] = ([1,0,0,0,1,0,0,0,1],[0,0,0])
    TERRAIN_CALIBRATION[1] = wall1Transform
    TERRAIN_CALIBRATION[2] = wall2Transform
    TERRAIN_CALIBRATION[3] = tableTransform
    


    # perceptionTransform = ([ 0.99631874,  0.01519797, -0.08436826, -0.01459558,  0.9998634, 0.00775227, 0.08447454,   -0.00649233,    0.99640444], [-0.06180821,  0.0082858,  -0.00253027])


    # ([ 0.99631874, -0.01459558,  0.08447454, 0.01519797,  0.9998634,  -0.00649233, -0.08436826,  0.00775227,  0.99640444], [-0.06180821,  0.0082858,  -0.00253027])
    world.terrain(1).geometry().transform(*wall1Transform)
    world.terrain(2).geometry().transform(*wall2Transform)
    world.terrain(3).geometry().transform(*tableTransform)

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
    import armRecorder
    """The main loop that loads the planning / simulation models and
    starts the OpenGL visualizer."""
    # Declare the knowledge base
    global knowledge
    global ground_truth_shelf_xform

    ground_truth_shelf_xform=([0.009999333346666592, -0.9999000033332889, 0.00999983333416667, 0.9999500004166664, 0.009999833334166692, 7.049050118954173e-18, -9.99966667111252e-05, 0.009999333346666538, 0.9999500004166657], [1.4299999999999995, -0.05500000000000002, 3.469446951953614e-18])
    #ground_truth_shelf_xform=([1,0,0,0,1,0,0,0,1], [0,0,0])
    # shelf_xform_from_perception goes here
    # simply reveals the shelf xform
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
