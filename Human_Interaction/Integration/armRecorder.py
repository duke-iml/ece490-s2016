#HigherTech record

from motionController import LowLevelController
from motionController import PickingController
import CustomGLViewer

import sys, struct, time, os
from klampt import robotsim
import json
from Motion import motion
from Motion import config
import string

RIGHT = 'right'
LEFT = 'left'
BOTH = 'both'
#rospy.init_node('oeu')
#LEFT_LIMB = baxter_interface.Limb('left')
LEFT_JOINTS = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
LEFT_ARM_INDICES = [15,16,17,18,19,21,22]
#RIGHT_LIMB = baxter_interface.Limb('right')
RIGHT_JOINTS = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
RIGHT_ARM_INDICES = [35,36,37,38,39,41,42]

PATH_DIR = "../Trajectories/"
FILE_NAME = "Human_Test"

model_dir = "../klampt_models/"
KLAMPT_MODEL = "baxter_col.rob"

SPEED = 13

class Recorder:
    def __init__(self, controller = None, command_Queue = None):


        try:
            self.file = open('data.json', 'rw') 
            self.pathDictionary = json.load(self.file)
            self.file.close()
        except:
            self.pathDictionary = {}

        self.leftPath = []
        self.rightPath = []
        self.leftSaved = False
        self.rightSaved = False
        self.currentLimb = 'both'

        self.rate = -1
        self.degreeChange = 1

        self.command_Queue = None
        simworld = None
        world = None

        self.controller = None
        self.command_Queue = command_Queue     


        if controller != None:
            #assuming I'm being given a picking controller 
            self.controller = controller
            simworld = self.controller.simworld
            world = self.controller.world
            sim = robotsim.Simulator(simworld)
        else:
            simworld = self.makeWorld()
            planworld = self.makeWorld()
            sim = robotsim.Simulator(simworld)
            low_level_controller = LowLevelController(simworld.robot(0),sim.controller(0),sim)
            self.controller = PickingController(simworld, planworld, low_level_controller)

            visualizer = CustomGLViewer.CustomGLViewer(simworld,world, low_level_controller, sim)
            visualizer.run()
            self.command_Queue = visualizer.command_Queue

        
        self.fake_controller = LowLevelController(simworld.robot(0),sim.controller(0),sim)
        


     
    def run(self):

        print 'Beginning recorder functionalities.'
        

        while(1):
            # wait for input
            #method  = raw_input("What do you want to do? (h for help)").lower()
            method = self.command_Queue.get().lower()
            print '\nCommand is ', method, '\n=================================='

            if method is not None:            
                if method == 'h':
                    self.printHelp()
                elif method == 's':
                    self.savePath()
                elif method == 't' or method == 'run':
                    self.testPath()
                elif method == 'a':
                    self.appendPath()
                elif method == 'u':
                    self.changePath()
                elif method == 'v':
                    self.changeLimb()
                elif method == 'c':
                    self.commandRobot()
                elif method == 'l':
                    self.loadPath()
                elif method == 'f':
                    self.changeHz()
                elif method == 'r':
                    self.reversePath()
                elif method == 'x':
                    self.clearPaths()
                elif method == 'p':
                    self.printPaths()
                elif method == 'm':
                    self.reset()
                elif method == 'q':
                    print 'Ending recorder functionalities' 

                    break
                else:
                    print 'Error did not understand command: ', method
        return 

    def makeWorld(self):

        """Produces a world with only the Baxter, table, partitions, and ground plane in it."""
        world = robotsim.WorldModel()

        print "Loading simplified Baxter model..."
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

        #initialize the shelf xform for the visualizer and object
        #ground_truth_shelf_xform = se3.mul(Trel,reorient)
        return world

    def savePath(self):

        name = self.getName()



        entry = None

        if self.currentLimb == 'left':
            entry = [self.currentLimb, ['left', self.leftPath], ['right', []] ]
        elif self.currentLimb == 'right':
            entry = [self.currentLimb, ['left', []], ['right', self.rightPath ] ]
        else:
            entry = [self.currentLimb, ['left', self.leftPath], ['right', self.rightPath] ]
        #json.dumps({name: entry}, self.file)

        self.pathDictionary[name] =  entry
        with open('data.json', 'w') as outfile:
            json.dumps(self.pathDictionary, outfile)


        self.leftSaved = True
        self.rightSaved = True

    def testPath(self, reverse = False):

        global LEFT_ARM_INDICES
        global RIGHT_ARM_INDICES

        currentSetup = self.controller.controller.getSensedConfig()
        startConfig = [v for v in currentSetup]
       
        if self.leftPath != []:
            for i in range(len(LEFT_ARM_INDICES)):
                startConfig[LEFT_ARM_INDICES[i]] = self.leftPath[0][i]

        if self.rightPath != []:
            for i in range(len(RIGHT_ARM_INDICES)):
                startConfig[RIGHT_ARM_INDICES[i]] = self.rightPath[0][i]

        #self.fake_controller.sim.getWorld().robot(0).setConfig(startConfig)
        #self.fake_controller.setVelocity([1]*61,0.1)
                   
        if self.currentLimb == 'left':
            for milestone in self.leftPath:
                self.fake_controller.appendMilestoneLeft(milestone)
        elif self.currentLimb == 'right':
            for milestone in self.rightPath:
                self.fake_controller.appendMilestoneRight(milestone)

        elif self.currentLimb == 'both':
            for milestone in self.rightPath:
                self.fake_controller.appendMilestoneRight(milestone)

            for milestone in self.leftPath:
                self.fake_controller.appendMilestoneLeft(milestone)
        return 

    def loadPath(self):
        #TODO

        print 'Options are:'
        for key in self.pathDictionary:
            print key

        load_name = self.getName()

        if load_name in self.pathDictionary:
            self.leftPath = self.pathDictionary[load_name][1][1]
            self.rightPath = self.pathDictionary[load_name][2][1]
            print 'Loaded'
        else:
            print 'Sorry, name not recognized.'
        return 

    def changePath(self, method=None):

        if self.rate > 0:
            self.recordPath()
        else:   

            print ''
            print 'Replace (p), Insert (i), Remove (x), or Append (a)?'
            method = self.command_Queue.get().lower()

            if method == 'p':
                self.replacePath()
            elif method == 'i':
                self.insertPath()
            elif method == 'x':
                self.removeMilestone()
            elif method == 'a':
                self.appendPath()
            else:
                print 'Unknown command. Returning to main loop'
        return


    def changeLimb(self):

        print ("What type of limb would you like to record?")
        print ("Options: Left (l), Right (r), Both (b), Cancel (x) ")
        limbResponse = self.getName().lower()

        leftResponses = ['l', 'left']
        rightResponses = ['r', 'right']
        bothResponses = ['b', 'both']
        cancelResponses = ['x', 'cancel']

        if limbResponse in cancelResponses:
            print 'Cancelling'
            return
        elif limbResponse in leftResponses:
            print 'Changing to left limb'
        elif limbResponse in rightResponses:
            print 'Changing to right limb'
            if not self.rightSaved:
                #confirm = raw_input('Delete current right path? (y/n) ')
                print ('Delete current right path (y/n)')
                confirm = self.getName().lower()
                if confirm == 'y':
                    self.rightPath = []
            else:
                self.currentLimb = RIGHT

            self.leftPath = []
            self.currentLimb = RIGHT

    
    def commandRobot(self):

        #make it so various numbers allow you to change how the robot behaves

        if self.currentLimb == BOTH:
            print 'Error, won\'t adjust both arms at once'
            return
        jointCommands = ['1','!','2','@','3','#','4','$','5','%','6','^','7','&']
        print 'Current incremental change is ', self.degreeChange, ' degrees'

        while(1):
            movement = self.command_Queue.get().lower()

            if method is not None: 
                if movement in jointCommands:
                    self.moveRobot(movement)
                elif method == 'h':
                    self.printRobotHelp()  
                elif method == 'c':
                    self.changeDegrees()
                elif method == 'q':
                    print 'Ending command of robot'
                    break
                else:
                    print 'Error did not understand command: ', method


    def moveRobot(self, movement=None):

        #TODO - move the simulation robot and keep it at that location

        return

    def changeDegrees(self):

        entry = raw_input('Enter a number in degrees')
        self.degreeChange = float(entry)

    def changeHz(self):

        rate = raw_input("Enter a rate ")
        self.rate = float(rate)

        return
    

    def recordPath(self):

        #TODO - test and make sure it works fine

        fn = PATH_DIR + FILE_NAME

        f = open(fn, 'w')
        try:
            print "Beginning capture loop, press Ctrl+C to quit"
            t0 = time.time()
            while True:
                t = time.time() - t0
                if t < 1e-3: t = 0
                if switch == 'sensed':
                    q = motion.robot.left_limb.sensedPosition()+motion.robot.right_limb.sensedPosition()
                else:
                    q = motion.robot.left_limb.commandedPosition()+motion.robot.right_limb.commandedPosition()
                if q != lastq:
                    if lastq is not None:
                        f.write(str(lastt) + '\t' + str(len(lastq)) + '\t'+ ' '.join(str(v) for v in lastq)+'\n')
                    f.write(str(t) + '\t' + str(len(q)) + '\t'+ ' '.join(str(v) for v in q)+'\n')
                    lastt = t
                    lastq = q
                if rate <= 0:
                    raw_input("Press enter to capture > ")
                else:
                    time.sleep(1.0/self.rate)
        except KeyboardInterrupt:
            f.close()
            print "Saved %g seconds of motion. Exiting."%(time.time()-t0,)
            pass

    def insertPath(self):

        global LEFT_ARM_INDICES
        global RIGHT_ARM_INDICES

        print 'Inserting'

        config = self.controller.controller.getSensedConfig()
        leftArm = [config[v] for v in LEFT_ARM_INDICES]
        rightArm = [config[v] for v in RIGHT_ARM_INDICES]

        index = self.getNumber()
        if index == 'q':
            return 


        if self.currentLimb == 'left':
            if len(self.leftPath) > index:
                self.leftPath.insert(index, [a for a in leftArm])
            else:
                print 'Error, index outside of list bounds'
        elif self.currentLimb == 'right':
            if len(self.rightPath) > index:
                self.rightPath.insert(index, [a for a in rightArm])
            else:
                print 'Error, index outside of list bounds'
        elif self.currentLimb == 'both':

            if len(self.leftPath) > index:
                self.leftPath.insert(index, [a for a in leftArm])
            else:
                print 'Error, index outside of left list bounds'

            if len(self.rightPath) > index:
                self.rightPath.insert(index, [a for a in rightArm])
            else:
                print 'Error, index outside of right list bounds'
        return

    def appendPath(self):

        global LEFT_ARM_INDICES
        global RIGHT_ARM_INDICES

        print 'Appending'

        config = self.controller.controller.getSensedConfig()
        leftArm = [config[v] for v in LEFT_ARM_INDICES]
        rightArm = [config[v] for v in RIGHT_ARM_INDICES]

        if self.currentLimb == 'left':
            self.leftPath.append([a for a in leftArm])
        elif self.currentLimb == 'right':
            self.rightPath.append([a for a in rightArm])            
        elif self.currentLimb == 'both':
            self.leftPath.append([a for a in leftArm])
            self.rightPath.append([a for a in rightArm]) 
        return

    def removeMilestone(self):
        #TODO remove mileston

        print 'Removing'

        index = self.getNumber()
        if index == 'q':
            return 


        if self.currentLimb == 'left':
            if len(self.leftPath) > index:
                self.leftPath.pop(index)
            else:
                print 'Error, index outside of list bounds'
        elif self.currentLimb == 'right':
            if len(self.rightPath) > index:
                self.rightPath.pop(index)
            else:
                print 'Error, index outside of list bounds'
        elif self.currentLimb == 'both':
            if len(self.leftPath) > index:
                self.leftPath.pop(index)
            else:
                print 'Error, index outside of left list bounds'
            if len(self.rightPath) > index:
                self.rightPath.pop(index)
            else:
                print 'Error, index outside of right list bounds'
        return

    def replacePath(self):

        global LEFT_ARM_INDICES
        global RIGHT_ARM_INDICES

        print 'Replacing'
        #could take out the leftarm/rightarm step
        config = self.controller.controller.getSensedConfig()
        leftArm = [config[v] for v in LEFT_ARM_INDICES]
        rightArm = [config[v] for v in RIGHT_ARM_INDICES]

        index = self.getNumber()
        if index == 'q':
            return


        if self.currentLimb == 'left':
            if len(self.leftPath) > index:
                self.leftPath[index] = [a for a in leftArm]
            else:
                print 'Error, index outside of list bounds'
        elif self.currentLimb == 'right':
            if len(self.rightPath) > index:
                self.rightPath[index] = [a for a in rightArm]
            else:
                print 'Error, index outside of list bounds'
        elif self.curentLimb == 'both':
            if len(self.leftPath) > index:
                self.leftPath[index] = [a for a in leftArm]
            else:
                print 'Error, index outside of left list bounds'
            if len(self.rightPath) > index:
                self.rightPath[index] = [a for a in rightArm]
            else:
                print 'Error, index outside of right list bounds'

    def reset(self):

        self.currentLimb = 'both'

        load_name = 'Q_DEFAULT_LOAD'

        milestone_ldefault = self.pathDictionary[load_name][1][1]
        milestone_rdefault = self.pathDictionary[load_name][2][1]

        self.leftPath = []
        self.rightPath = []

        print milestone_ldefault
        print milestone_rdefault

        self.controller.controller.appendMilestoneLeft(milestone_ldefault[0], 1)
        self.controller.controller.appendMilestoneRight(milestone_rdefault[0], 1)
        time.sleep(.1)
        print 'Done resetting'

    
    def getNumber(self):

        number = ''
        numVals = ['1','2','3','4','5','6','7','8','9','0']
        while(1):
            digit = self.command_Queue.get().lower()

            if digit == chr(13):
                try:
                    intNum = int(number)
                    return intNum
                except:
                    print 'Error, number cannot be evaluated'
                    return 'q'
            elif digit is not None: 
            
                if digit in numVals:
                    number += digit
                    os.system('clear')
                    print number
                elif digit == '\x08' or digit == '\x7f':
                    # backspace or delete
                    number = number[:-1]
                    os.system('clear')
                    print number
                else:
                    print 'Error, value not recognized'
                    print 'Press Enter to submit your entry or quit'

    def getName(self):

        name = ''
        
        while(1):
            letter = self.command_Queue.get()

            if letter == chr(13):
                return name
            elif letter is not None: 
            
                if letter in string.printable:
                    name += letter
                    os.system('clear')
                    print name
                elif letter == '\x08' or letter == '\x7f':
                    # backspace or delete
                    name = name[:-1]
                    os.system('clear')
                    print name
                else:
                    print 'Error, value not recognized'
                    print 'Press Enter to submit your entry or quit'


    def sendPath(path,maxSmoothIters =0, INCREMENTAL=False, limb = None, readConstants=False, internalSpeed=SPEED):


        # interpolate path linearly between two endpoints
        if path == None:
            print "sending empty path"
            return False

        for smoothIter in range(maxSmoothIters):
            # path = path
            smoothePath = [0]*(len(path)*2-1)
            for i in range(len(path)-1):
                smoothePath[i*2] = path[i]
                smoothePath[i*2 +1] = vectorops.div(vectorops.add(path[i],path[i+1]), 2)
            smoothePath[-1] = path[-1]
            path = smoothePath



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
                if counter%internalSpeed == 0 or INCREMENTAL:
                    if limb == 'left':
                        #CONTROLLER.appendMilestoneLeft(q)
                        pass
                    elif limb == 'right':
                        #CONTROLLER.appendMilestoneRight(q)
                        pass
                    else:
                        #print 'milestone #', i, q
                        #CONTROLLER.appendMilestone(q)
                        pass
                counter +=1
        if limb == 'left':
            #CONTROLLER.appendMilestoneLeft(path[-1])
            pass
        elif limb == 'right':
            #CONTROLLER.appendMilestoneRight(path[-1])
            pass
        else:
            pass
            #print 'last milestone', path[-1]
            #CONTROLLER.appendMilestone(path[-1])
        # print 'Done with moving'

    def clearPaths(self):

        self.leftPath = []
        self.rightPath = []

        return

    def printPaths(self):
        print 'Left path is: ', self.leftPath
        print 'Right path is: ', self.rightPath

    def reversePath(self):

        return
        #TODO Fix - currently crashes the code

        leftLength = len(self.leftPath)
        rightLength = len(self.rightPath)

        if self.currentLimb == 'left':
            for i in range(leftLength):
                leftPathRev[leftLength-i] = self.leftPath[i]
            self.leftPath = [a for a in leftPathRev]
        elif self.currentLimb == 'right':
            for i in range(rightLength):
                rightPathRev[rightLength-i] = self.rightPath[i]
            self.rightPath = [a for a in rightPathRev]
        elif self.curentLimb == 'both':
            for i in range(rightLength):
                rightPathRev[rightLength-i] = self.rightPath[i]
            self.rightPath = [a for a in rightPathRev]
            for i in range(leftLength):
                leftPathRev[leftLength-i] = self.leftPath[i]
            self.leftPath = [a for a in leftPathRev]

    def printHelp(self):

        print 'H: Help (show this text)'
        print 'Q: Quit (return to standard controller)'
        print 'S: Save current path'
        print 'T: Test/Simulate current path'
        print 'A: Append current milestone to end of path'
        print 'U: Update path manually (i.e. insert/replace milestones)'
        print 'L: Load path from file'
        print 'V: Switch limb to record'
        print 'F: Change recording frequency'
        print 'R: Begin coninuous recording loop'
        print 'X: Clear/Reset paths'
        print 'P: Print current paths'
        print 'M: Reset to default config'
        return 

    def printRobotHelp(self):
        print 'H: Help (show this text)'
        print '1/!: Move joint 1'        
        print '2/@: Move joint 2'
        print '3/#: Move joint 3'
        print '4/$: Move joint 4'
        print '5/%: Move joint 5'
        print '6/^: Move joint 6'
        print '7/&: Move joint 7'
        print 'C: Change the amount by which a joint rotates (degress)'
        print 'Q: Quit (return to main loop)'
        
if __name__ == "__main__":

    myRecorder = Recorder()
    myRecorder.run()
