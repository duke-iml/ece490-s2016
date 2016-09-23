#moveArmsOnly
import sys, struct, time, json


START_TIME = time.time()

sys.path.insert(0, "../../common")
sys.path.insert(0, "..")

import CustomGLViewer
from klampt import robotsim
from klampt.glprogram import *
from klampt import vectorops, se3, so3, loader, gldraw, ik
from klampt.robotsim import Geometry3D
from klampt import visualization,trajectory
import os, math, random, copy
import string

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

from datetime import datetime

SPEED = 18
visualizer = CustomGLViewer.CustomGLViewer()
REAL_CAMERA = True
CAMERA_TRANSFORM = {}
CAMERA_TRANSFORM[0] =  ([-0.005161218572267102, -0.9997067781777349, -0.023658391499492008, -0.017969530697323825, 0.023747606408628236, -0.9995564752210745, 0.9998252136195455, -0.00473379925164146, -0.018086828226030516], [0.8350000000000001, -0.44, 0.968])
CAMERA_TRANSFORM[1] =  ([0.04869661256859728, -0.9987945892311386, 0.006165099102761133, 0.027778736693568153, -0.004815722998978523, -0.9996024962952568, 0.9984272540911809, 0.048848514149358035, 0.027510742508093464], [0.8540000000000001, -0.03600000000000003, 0.9649999999999999])
CAMERA_TRANSFORM[2] =  ([-0.01736342375172043, -0.9998242986481636, 0.007062814476406195, 0.005270028659627553, -0.007155298526801169, -0.9999605134708677, 0.9998353556028844, -0.017325516895674346, 0.005393343188858899], [0.8680000000000001, 0.389, 0.9659999999999997])


WORKING_CAMERAS = [0,1,2]

ZMIN = .95
ZMAX = 1.2

REGION_DIR = {}
REGION_DIR[4] = ([.5, .2, ZMIN],[1.41, .5, ZMAX])
REGION_DIR[2] = ([.5, -.2, ZMIN],[1.47, .2, ZMAX])
REGION_DIR[3] = ([.5, -.2, ZMIN],[1.47, .2, ZMAX])
REGION_DIR[1] = ([.5, -.5, ZMIN],[1.47, -.2, ZMAX])

FILE_DIR = {}
FILE_DIR[1] = 'RIGHT_BOX_RIGHT_ARM'
FILE_DIR[2] = 'MID_BOX_RIGHT_ARM'
FILE_DIR[3] = 'MID_BOX_LEFT_ARM'
FILE_DIR[4] = 'LEFT_BOX_LEFT_ARM'

OUTPUT_FILE_NAME = 'results.txt'
OUTPUT_RESULTS = True

if REAL_CAMERA:
    # Perception
    from perception import perception
    perceiver = perception.Perceiver()

def loop():
    counter = 0
    trialNumber = 1
    while (1):


        if counter %100== 0:
            pass
            #if REAL_CAMERA:
            #    takePicture()
        else:

            if loopHelper(1, trialNumber):
                trialNumber = trialNumber + 1
            elif loopHelper(2, trialNumber):
                trialNumber = trialNumber + 1
            elif loopHelper(3, trialNumber):
                trialNumber = trialNumber + 1
            elif loopHelper(4, trialNumber):
                trialNumber = trialNumber + 1
            elif loopHelper(5, trialNumber):
                trialNumber = trialNumber + 1
            elif loopHelper(6, trialNumber):
                trialNumber = trialNumber + 1
            elif loopHelper(7, trialNumber):
                trialNumber = trialNumber + 1
            elif loopHelper(8, trialNumber):
                trialNumber = trialNumber + 1

		time.sleep(.1)
        counter = counter+1

def loopHelper(number, trialNumber):

    global OUTPUT_FILE_NAME

    file_prefix = "./" + str(trialNumber) + "-"
    if os.path.exists(file_prefix+str(number)+".com"):
        with open(OUTPUT_FILE_NAME, 'a') as f:
            s = 'Started movement type '+str(number)+' at '+ datetime.isoformat(datetime.now()) + '\n'
            f.write(s)
        with open('start.txt', 'w') as f:
            s = 'Started movement type '+str(number)+' at '+ datetime.isoformat(datetime.now()) + '\n'
            f.write(s)
        if os.path.exists('./end.txt'):
            os.remove('./end.txt')


        supplyBoxes(number)
        with open(OUTPUT_FILE_NAME, 'a') as f:
            s = 'Finished movement type '+str(number)+' at ' + datetime.isoformat(datetime.now())+ '\n'
            f.write(s)
        with open('end.txt', 'w') as f:
            s = 'Finished movement type '+str(number)+' at '+ datetime.isoformat(datetime.now()) + '\n'
            f.write(s)

        if os.path.exists('./start.txt'):
            os.remove('./start.txt')

        os.remove(file_prefix + str(number)+".com")
        return True
    else:
        return False


def supplyBoxes(choice):

    '''
    1 - Supply the right box with the right arm
    2 - Supply the middle box with the right arm
    3 - Supply the middle box with the left arm
    4 - Supply the left box with the left arm   

    5-8 = 1-4 without camera

    '''



    limb = None
    num = None

    checkPeople = True

    if 5 <= choice <= 8:
        checkPeople = False
        choice = choice - 4

    if choice == 1 or choice == 2:
        limb = 'right'
        num = choice
    elif choice == 3 or choice == 4:
        limb = 'left'
        # if choice == 3:
        # 	num = 2
        # else:
        # 	num = 1
        # # choosing 4 means go to box 3
        # choosing 3 means go to box 2
    else:
        raise Exception('Invalid entry')
    # path1_name = limb.upper()+'_SUPPLY'
    # path2_name = limb.upper()+'_DROP_'+str(choice)

    path1_name = limb.upper()+'_SUPPLY_'+str(choice)
    path2_name = limb.upper()+'_DEPOSIT_'+str(choice)

    moveArm(limb = limb, path_name = path1_name, reverse = False)
    waitForMove()

    if checkPeople:
        checkForPerson(choice)

    moveArm(limb = limb, path_name = path2_name, internalSpeed=100)
    waitForMove()

    #move back
    moveArm(limb = limb, path_name = path2_name, reverse = True, internalSpeed=100)
    waitForMove()
    moveArm(limb = limb, path_name = path1_name, reverse = True)
    waitForMove()


def checkForPerson(choice):

    global OUTPUT_FILE_NAME, OUTPUT_RESULTS

    filename = FILE_DIR[choice]
    region = REGION_DIR[choice]

    index = None
    if choice == 1:
        index = 0
    elif choice == 2 or choice == 3:
        index = 1
    elif choice == 4:
        index = 2
    else:
        raise Exception('Invalid choice in checkForPerson')


    R,t = getCameraToWorldXform(index=index)
    #region - a pair of x,y,z points signifying a box box
    #currently just a straightforward box

    #seenCount = 0

    while (1):

        # should be the subtracted data 
        if REAL_CAMERA:
            #picture_data = perceiver.get_current_content_cloud(R,t, filename,  tolist=True, index=index)
            # ^does point cloud subtraction

            picture_data = perceiver.get_current_point_cloud(R,t, limb=None, tolist=True, index=index)
            # grabs the entire point cloud
            visualizer.updatePoints(picture_data)


            number = countPointsInRegion(picture_data, region)

            if number >= 20:

                #seenCount = seenCount + 1
                print 'A person is there - waiting'

                if OUTPUT_RESULTS:
                    with open(OUTPUT_FILE_NAME, 'a') as f:
                        s = 'Noticed person in region '+str(index + 1)+' at '+ datetime.isoformat(datetime.now()) + '\n'
                        f.write(s)

            else:
                print 'Continuing'

                if OUTPUT_RESULTS:
                    with open(OUTPUT_FILE_NAME, 'a') as f:
                        s = 'Continued at '+ datetime.isoformat(datetime.now()) + '\n'
                        f.write(s)

                if os.path.exists('seen'+str(index + 1)+'.txt'):
                    os.remove('./seen'+str(index + 1)+'.txt')
                #while seenCount > 0:
                    #os.remove('./seen'+str(seenCount)+'.txt')
                    #os.remove('./seen'+str(index)+'.txt')
                    #seenCount = seenCount - 1
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

    for point in picture_data:

        # if counter%100 == 0:
        #     print point

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


def moveArm( limb, statusConditional=None, path_name=None, path=None, finalState=None, reverse=False, internalSpeed=SPEED):
    if limb == 'left':
        if moveLeftArm(statusConditional, path_name, path, finalState, reverse, internalSpeed):
            return True
    if limb == 'right':
        if moveRightArm(statusConditional, path_name, path, finalState, reverse, internalSpeed):
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



def moveLeftArm( statusConditional=None, path_name=None, path=None, finalState=None, reverse=False, internalSpeed=SPEED):

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
		        sendPath(path, limb='left', readConstants=True, internalSpeed=internalSpeed)
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


def moveRightArm( statusConditional=None, path_name=None, path=None, finalState=None, reverse=False, internalSpeed=SPEED):

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
				sendPath(path, limb='right', internalSpeed=internalSpeed)
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

def forceWait( milestone1, indices, eps):

	global CONTROLLER

	milestone2 = [CONTROLLER.getSensedConfig()[v] for v in indices]
	while (np.linalg.norm(np.array(milestone2)-milestone1)) >= eps:
	    milestone2 = [CONTROLLER.getSensedConfig()[v] for v in indices]
	    print (np.linalg.norm(np.array(milestone2)-milestone1))


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

    speed = 1
    while i+1 <endIndex:
        # print i, endIndex
        q = path[i]
        qNext = path[i+1]
        dt = vectorops.distance(q,qNext)
        dt = dt / speed

        i += 1
        waitForMove()
        # if INCREMENTAL:
        #     time.sleep(.1)
        if limb == 'left':
            CONTROLLER.appendMilestoneLeft(q,dt)
            #pass
        elif limb == 'right':
            CONTROLLER.appendMilestoneRight(q,dt)
            #pass
        else:
            #print 'milestone #', i, q
            #CONTROLLER.appendMilestone(q)
            CONTROLLER.appendLinear(q,dt)
            #pass


    """
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
                    CONTROLLER.appendMilestoneLeft(q,1)
                    #pass
                elif limb == 'right':
                    CONTROLLER.appendMilestoneRight(q,1)
                    #pass
                else:
                    #print 'milestone #', i, q
                    #CONTROLLER.appendMilestone(q)
                    CONTROLLER.appendLinear(q,1)
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
    """
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

def takePicture():
    global visualizer
    global WORKING_CAMERAS

    print 'taking picture'

    current_cloud = []

    for index in WORKING_CAMERAS:
        cloud = perceiver.get_current_point_cloud(*getCameraToWorldXform(index=index), tolist=True, index=index)
        for i in cloud:
            current_cloud.append(i)
    print 'number of points is: ', len(current_cloud)
    picture_data = current_cloud
    #print 'picture data is: ', picture_data


    visualizer.updatePoints(current_cloud)

    return picture_data

def getCameraToWorldXform(linkName=None, index=0):
    '''
    get current transformation (R, t) of camera in world frame. 
    '''

    global CAMERA_TRANSFORM, SIMWORLD

    if len(CAMERA_TRANSFORM) < index:
        print 'Error, camera index does not exist'
        return ([0,0,0,0,0,0,0,0,0],[0,0,0])

    if linkName != None:
        return se3.mul(SIMWORLD.robot(0).link(linkName).getTransform(), CAMERA_TRANSFORM[index])
    else: 
        return CAMERA_TRANSFORM[index]


def calibrateExperiment():
    global FILE_DIR
    
    path_names = ['RIGHT_SUPPLY_1','RIGHT_SUPPLY_2','LEFT_SUPPLY_3','LEFT_SUPPLY_4']
    path_names2 = ['RIGHT_DEPOSIT_1', 'RIGHT_DEPOSIT_2','LEFT_DEPOSIT_3','LEFT_DEPOSIT_4']
    indices = [0,1,1,2]

    for i in range(len(path_names)):
        limb = path_names[i].split('_')[0].lower()
        name = path_names[i]
        name2 = path_names2[i]
        filename = FILE_DIR[i+1]
        index = indices[i]

        print 'moving through path 1'
        moveArm(limb = limb, path_name = name, reverse = False)
        waitForMove()
    


        #moveArm(limb = limb, path_name = name2, reverse=False)
        #waitForMove()

        print 'sleeping'
        time.sleep(2)

        R,t = getCameraToWorldXform(index=index)
        perceiver.save_canonical_cloud(R,t, filename, index=index)
        #save picture
        #moveArm(limb = limb, path_name = name2, reverse=True)
        #waitForMove()



        print 'moving back'
        moveArm(limb = limb, path_name = name, reverse = True)
        waitForMove()


class Helper():
    def __init__(self):
        pass
    def run(self):
        control_thread = Thread(target=loop)
        print 'starting thread'
        control_thread.start()
        #control_thread.run()



model_dir = "../klampt_models/"
KLAMPT_MODEL = "baxter_col.rob"
PATH_DICTIONARY = {}
RUN_EXP_IMMEDIATELY = True
FORCE_WAIT = False
STATE_LEFT = 'ready'
STATE_RIGHT = 'ready'

WAIT_TIME = 2

WORLD = load_exp_world()
SIMWORLD = load_exp_world()
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
SEND_PATH = True

#CHOICE = int(sys.argv[1])
#print 'Choice is', CHOICE

#calibrateExperiment()

with open(OUTPUT_FILE_NAME, 'w') as f:
    f.write('')
    #clears file

for i in range(8):
    for file_n in os.listdir("."):
        if file_n.endswith(".com"):
            os.remove(file_n)

myHelp = Helper()
visualizer = CustomGLViewer.CustomGLViewer(SIMWORLD, WORLD, LOW_LEVEL_CONTROLLER, helper=myHelp)

#viewing_thread = Thread(target=CustomGLViewer.run(), args=visualizer)
#viewing_thread.start()
#viewing_thread.run()
print 'about to run visualizer'
visualizer.run()


