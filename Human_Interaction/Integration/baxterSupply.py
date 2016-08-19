#moveArmsOnly
import sys, struct, time, json


START_TIME = time.time()

sys.path.insert(0, "../../common")
sys.path.insert(0, "..")

SPEED = 16
SEND_PATH = True

REAL_CAMERA = True
CAMERA_TRANSFORM = {}
CAMERA_TRANSFORM[0] = ([0.028595067482461656, 0.9918980356264576, -0.12377644782449986, 0.9995161363070233, -0.029889119405285713, -0.008610098899508051, -0.012239909213560033, -0.12347035053628959, -0.9922727735662678], [1.1600000000000001, 0.12, 1.96])

REGION_DIR = {}
REGION_DIR[4] = ([.5, .2, .8],[1.7, .6, 2])
REGION_DIR[2] = ([.5, -.2,.8],[1.7, .2, 2])
REGION_DIR[3] = ([.5, -.2,.8],[1.7, .2, 2])
REGION_DIR[1] = ([.5, -.6, .8],[1.7, -.2, 2])

FILE_DIR = {}
FILE_DIR[1] = 'RIGHT_BOX_RIGHT_ARM'
FILE_DIR[2] = 'MID_BOX_RIGHT_ARM'
FILE_DIR[3] = 'MID_BOX_LEFT_ARM'
FILE_DIR[4] = 'LEFT_BOX_LEFT_ARM'

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

from Motion import motion
from Motion import config
from Motion import motion_debouncer

from motionController import LowLevelController
from motionController import FakeLowLevelController
from motionController import PhysicalLowLevelController


def supplyBoxes(choice):

    '''
    1 - Supply the right box with the right arm
    2 - Supply the middle box with the right arm
    3 - Supply the middle box with the left arm
    4 - Supply the left box with the left arm   
    '''

    limb = None
    num = None

    if choice == 1 or choice == 2:
        limb = 'right'
        num = choice
    elif choice == 3 or choice == 4:
        limb = 'left'
        # if choice == 3:
        #   num = 2
        # else:
        #   num = 1
        # # choosing 4 means go to box 3
        # choosing 3 means go to box 2
    else:
        raise Exception('Invalid entry')

    # path1_name = limb.upper()+'_SUPPLY'
    # path2_name = limb.upper()+'_DROP_'+str(choice)

    path1_name = limb.upper()+'_SUPPLY_'+str(choice)
    path2_name = limb.upper()+'_DEPOSIT_'+str(choice)

    moveArm(limb = limb, path_name = path1_name, reverse = False)
    print 'moving to halfway'
    waitForMove()

    #checkForPerson(choice)

    moveArm(limb = limb, path_name = path2_name)
    print 'depositing'
    waitForMove()

    #move back
    moveArm(limb = limb, path_name = path2_name, reverse = True)
    print 'backing up'
    waitForMove()
    moveArm(limb = limb, path_name = path1_name, reverse = True)
    print 'back to rest'
    waitForMove()


def checkForPerson(choice):


    filename = FILE_DIR[choice]
    region = REGION_DIR[choice]

    R,t = getCameraToWorldXform()
    #region - a pair of x,y,z points signifying a box box
    #currently just a straightforward box

    while (1):

        # should be the subtracted data 
        if REAL_CAMERA:
            picture_data = perceiver.get_current_content_cloud(R,t, filename, tolist=True)
            #print picture_data

            number = countPointsInRegion(picture_data, region)

            if number >= 20:
                print 'A person is there - waiting'
            else:
                print 'Continuing'
                break
        else:
            return

        time.sleep(.1)

    # we are at the location we want
    # take a picture
    # subtract out the arm
    # are there people there?
    # if yes, continue this loop
    # if no, 

def countPointsInRegion(picture_data, region):
    # region, an x,y,z box 
    counter = 0

    return 0

    for point in picture_data:

        if counter%100 == 0:
            print point

        in_x = region[0][0] <= point[0] <= region[1][0]
        in_y = region[0][1] <= point[1] <= region[1][1]
        in_z = region[0][2] <= point[2] <= region[1][2]
        if in_x and in_y and in_z:
            counter = counter + 1

    return counter


def saveCanonicalModelCloud():
    name = raw_input('Enter a name to save this point cloud as')
    R,t = getCameraToWorldXform()
    perceiver.save_canonical_cloud(filename, R,t , limb=limb)




def waitForMove(timeout = None, pollRate = 0.01):
    """Waits for the move to complete, or timeout seconds is elapsed,
    before terminating."""
    global CONTROLLER

    if timeout == None:
        timeout = 300
    iters = 0
    t = 0
    # print "Waiting for move to complete",
    while CONTROLLER.isMoving(): # remaining time > 0
        # if iters % 10 == 0:
        #     print ".",
        time.sleep(pollRate)
        t += pollRate
        if timeout != None and t > timeout:
            print "Timed Out!"
            return False
        iters += 1
    # print "--> done\n"


def moveArm( limb, statusConditional=None, path_name=None, path=None, finalState=None, reverse=False):
    if limb == 'left':
        if moveLeftArm(statusConditional, path_name, path, finalState, reverse):
            return True
    if limb == 'right':
        if moveRightArm(statusConditional, path_name, path, finalState, reverse):
            return True
    if limb =='both':
        path_nameL = path_name
        path_nameR = path_name
        reverseL = reverse
        reverseR = reverse
        pathL = path
        pathR = path
        if moveBothArms(statusConditional, path_nameL, pathL, path_nameR, pathR, finalState, reverseL, reverseR):
            return True
    #wasn't able to move
    print 'Error with move'+limb+'arm'
    return False

def moveLeftArm( statusConditional=None, path_name=None, path=None, finalState=None, reverse=False):

    global STATE_LEFT
    global PATH_DICTIONARY
    global CONTROLLER
    global SEND_PATH

    if(STATE_LEFT == statusConditional or statusConditional == None or STATE_LEFT in statusConditional):
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
                    STATE_LEFT = finalState
                return True

            print 'Hit path exception for single milestones or empty paths'
            path=[path]
            path.append(path[0])
            qAdd = None

            path[0]=qAdd
            #print path
        if reverse:
            path = path[::-1]
        if SEND_PATH:
            if len(path)>=1:
                sendPath(path, limb='left', readConstants=True)
        else:    
            for milestone in path:
                CONTROLLER.appendMilestoneLeft(milestone, 1)
                #move to the milestone in 1 second

                time.sleep(0.1)
                waitForMove() #still doesn't do anything, but it's the thought that counts
                if FORCE_WAIT:
                    forceWait(milestone, left_arm_indices, 0.01)
                else:
                    CONTROLLER.appendMilestoneLeft(milestone, WAIT_TIME)
                #wait at the milestone for 2 seconds
                #later should replace with Hyunsoo's code setting milestones if dt is too large


        if finalState is not None:
            STATE_LEFT = finalState
        return True
    else:
        print "Error, arm is not in state ", statusConditional
        return False


def moveRightArm( statusConditional=None, path_name=None, path=None, finalState=None, reverse=False):

    global STATE_RIGHT
    global PATH_DICTIONARY
    global CONTROLLER
    global SEND_PATH

    print 'called moverightarm'

    if(STATE_RIGHT == statusConditional or statusConditional == None):# or STATE_RIGHT in statusConditional):
     
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
                    STATE_RIGHT = finalState
                return True
            print 'Hit path exception for single milestones or empty paths'
            path=[path]
            path.append(path[0])
        if reverse:
            path = path[::-1]

        if SEND_PATH:
            if len(path)>=1:
                sendPath(path, limb='right')
        else:
            for milestone in path:

                #print milestone
                CONTROLLER.appendMilestoneRight(milestone, 1)
                #move to the milestone in 1 second
                #waitForMove() #still doesn't do anything, but it's the thought that counts
                #time.sleep(1)
                if FORCE_WAIT:
                    forceWait(milestone, right_arm_indices, 0.01)
                else:
                    CONTROLLER.appendMilestoneRight(milestone, WAIT_TIME)
                #wait at the milestone for 2 seconds

        if finalState is not None:
            STATE_RIGHT = finalState

        return True
    else:
        print "Error, arm is not in state ", statusConditional
        return False


def sendPath(path,maxSmoothIters =0, INCREMENTAL=False, limb = None, readConstants=False, internalSpeed=SPEED):


    # interpolate path linearly between two endpoints
    if path == None:
        print "sending empty path"
        return False

    # n = self.robot.numLinks()
    # for i in range(len(path)):

    #     # if we specify a limb, and the path only has seven numbers (DOF for each arm, we shouldn't append 0's)
    #     if readConstants and limb is not None:
    #         # if we're reading a path from the milestones
    #         pass
    #     else:
            
    #         #print path
    #         if len(path[i])<n:
    #             path[i] += [0.0]*(n-len(path[i]))
    #         #pass
    for smoothIter in range(maxSmoothIters):
        # path = path
        smoothePath = [0]*(len(path)*2-1)
        for i in range(len(path)-1):
            smoothePath[i*2] = path[i]
            smoothePath[i*2 +1] = vectorops.div(vectorops.add(path[i],path[i+1]), 2)
        smoothePath[-1] = path[-1]
        path = smoothePath



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
            waitForMove()
            # if INCREMENTAL:
            #     time.sleep(.1)
            if counter%internalSpeed == 0 or INCREMENTAL:
                if limb == 'left':
                    CONTROLLER.appendMilestoneLeft(q)
                    #pass
                elif limb == 'right':
                    CONTROLLER.appendMilestoneRight(q)
                    #pass
                else:
                    #print 'milestone #', i, q
                    CONTROLLER.appendMilestone(q)
                    #pass
            counter +=1
    if limb == 'left':
        CONTROLLER.appendMilestoneLeft(path[-1])
        #pass
    elif limb == 'right':
        CONTROLLER.appendMilestoneRight(path[-1])
        #pass
    else:
        #pass
        #print 'last milestone', path[-1]
        CONTROLLER.appendMilestone(path[-1])
    # print 'Done with moving'

def forceWait( milestone1, indices, eps):

	global CONTROLLER

	milestone2 = [CONTROLLER.getSensedConfig()[v] for v in indices]
	while (np.linalg.norm(np.array(milestone2)-milestone1)) >= eps:
	    milestone2 = [CONTROLLER.getSensedConfig()[v] for v in indices]
	    print (np.linalg.norm(np.array(milestone2)-milestone1))

def load_exp_world():
    """Produces a world with only the Baxter, shelf, and ground plane in it."""
    world = robotsim.WorldModel()

    print "Loading simplified Baxter model..."
    world.loadElement(os.path.join(model_dir,KLAMPT_MODEL))

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
    
    world.terrain(1).geometry().transform(*wall1Transform)
    world.terrain(2).geometry().transform(*wall2Transform)
    world.terrain(3).geometry().transform(*tableTransform)

    return world



model_dir = "../klampt_models/"
KLAMPT_MODEL = "baxter_col.rob"
PATH_DICTIONARY = {}
RUN_EXP_IMMEDIATELY = True
FORCE_WAIT = False
STATE_LEFT = 'ready'
STATE_RIGHT = 'ready'

WAIT_TIME = 2

WORLD = robotsim.WorldModel()
print "Loading simplified Baxter model..."
WORLD.loadElement(os.path.join(model_dir,KLAMPT_MODEL))

try:
    file = open('data.json', 'rw') 
    PATH_DICTIONARY = json.load(file)
    file.close()
except:
    raise Exception('Path Dictionary failed to load')
    
#start motion queue
motion.setup(mode=config.mode,klampt_model=config.klampt_model,libpath="./")
res = motion.robot.startup()
if not res:
    print "Error starting up Motion Module"
    exit(1)
time.sleep(0.1)

ROBOT = WORLD.robot(0)
LOW_LEVEL_CONTROLLER = PhysicalLowLevelController(ROBOT)
CONTROLLER = LOW_LEVEL_CONTROLLER
CHOICE = int(sys.argv[1])
#print 'Choice is', CHOICE
supplyBoxes(CHOICE)

END_TIME = time.time()

print 'Total Time = ', END_TIME-START_TIME