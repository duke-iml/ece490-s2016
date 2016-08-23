#HigherTech record

from motionController import LowLevelController
from motionController import PhysicalLowLevelController
from motionController import PickingController
import CustomGLViewer

from klampt import vectorops
import sys, struct, time, os, math
from klampt import robotsim
import json
from Motion import motion
from Motion import config
from klampt import so3
import string
from threading import Thread, Lock

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

#KLAMPT_MODEL = "baxter_col.rob"
KLAMPT_MODEL = "baxter.rob"


SPEED = 13

REAL_CAMERA = True
CAMERA_TRANSFORM = {}
CAMERA_TRANSFORM[0] =  ([-0.03513538175810132, -0.9991291152501236, -0.022505910517439685, -0.027963767034245017, 0.023493872890698906, -0.9993328102638538, 0.9989912575603299, -0.03448258975341347, -0.028764880008976604], [0.79, -0.43, 0.97])
CAMERA_TRANSFORM[1] =  ([0.008844227366159113, -0.9999500092784283, 0.0046646099910533645, 0.03776197731422964, -0.004327480126834385, -0.9992773919113119, 0.9992476233204751, 0.009013981352587987, 0.03772181634757153], [0.804, -0.021000000000000036, 0.9699999999999999])
CAMERA_TRANSFORM[2] =   ([-0.02236270887835915, -0.9997242784962567, 0.007160742752108415, 0.000270070414744631, -0.0071685745350301925, -0.9999742689695111, 0.9997498868785105, -0.02236019955805023, 0.00043030469592360635], [0.81, 0.399, 0.9699999999999999])

BOX_COORDS = {}
BOX_COORDS[2] = ([.5, .2, .90],[1.44, .6, 1.2])
BOX_COORDS[1] = ([.5, -.2,.90],[1.44, .2, 1.2])
BOX_COORDS[0] = ([.5, -.6, .90],[1.44, -.2, 1.2])


OVERHEAD = ([0.02200958349401458, 0.9957794720964701, -0.08910006277046488, 0.9720448364185201, -0.0004760777365310359, 0.23479482392524428, 0.23376144726305412, -0.09177699222174122, -0.9679527723356234], [0.9600000000000002, 0.06000000000000002, 1.9599999999999997])


myHelper = None

if REAL_CAMERA:
    # Perception
    from perception import perception
    perceiver = perception.Perceiver()

class Recorder:
    def __init__(self, controller = None, command_Queue = None, physical=False):

        global myHelper

        try:
            self.file = open('data.json', 'rw') 
            self.pathDictionary = json.load(self.file)
            self.file.close()
        except:
            self.pathDictionary = dict()

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
        self.visualizer = None
        self.controller = None
    
        self.simType = 'virtual'
        if physical:
            self.simType = 'physical'

        if controller != None:
            #assuming I'm being given a picking controller 
            self.controller = controller
            simworld = self.controller.simworld
            self.simworld = simworld
            world = self.controller.world
            sim = robotsim.Simulator(simworld)
            #assuming visualizer was already created
        else:
            self.setupController(physical)

        self.fake_controller = LowLevelController(simworld.robot(0),sim.controller(0),sim)
        
    def setupController(self, physical=False):
        
        simworld = self.makeWorld()
        planworld = self.makeWorld()
        visualizer = None    
        sim = None

        if not physical:
            #virtual simulation
            sim = robotsim.Simulator(simworld)
            low_level_controller = LowLevelController(simworld.robot(0),sim.controller(0),sim)
            self.controller = PickingController(simworld, planworld, low_level_controller)
            myHelper = Helper(self)
            visualizer = CustomGLViewer.CustomGLViewer(simworld,world, low_level_controller, sim, helper = myHelper)
            self.command_Queue = visualizer.command_queue
        else:
            #actually connected to the robot
            motion.setup(mode=config.mode,klampt_model=config.klampt_model,libpath="./")
            res = motion.robot.startup()
            if not res:
                print "Error starting up Motion Module"
                exit(1)
            time.sleep(0.1)
            q = motion.robot.getKlamptSensedPosition()
            simworld.robot(0).setConfig(q)
            planworld.robot(0).setConfig(q)

            low_level_controller = PhysicalLowLevelController(simworld.robot(0))
            self.controller = PickingController(simworld, planworld, low_level_controller)

            myHelper = Helper(self)
            visualizer = CustomGLViewer.CustomGLViewer(simworld,planworld, low_level_controller, helper=myHelper)
            self.command_Queue = visualizer.command_queue
            sim = robotsim.Simulator(simworld)
    
        self.simworld = simworld
        self.fake_controller = LowLevelController(simworld.robot(0),sim.controller(0),sim)
        self.visualizer = visualizer
        visualizer.run()

     
    def run(self):

        print 'Beginning recorder functionalities.'
        

        while(1):
            # wait for input
            #method  = raw_input("What do you want to do? (h for help)").lower()
            method = self.command_Queue.get().lower()
            print '\nCommand is ', method, '\n=================================='

            if method is not None:            
                if method == 'h':
                    #self.printHelp()
                    pass
                elif method == 's':
                    self.savePath()
                elif method == 't' or method == 'run':
                    self.testPath()
                elif method == 'a':
                    self.appendMilestone()
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
                elif method == 'w':
                    print 'Choose camera index'
                    index = self.getNumber()
                    self.calibrateCamera(index=index)

                elif method == 'b':
                    print 'Choose box index'
                    index = self.getNumber()
                    self.calibrateBox(index=index)

                elif method == 'g':
                    self.runPhysicalPath()
                elif method == 'q':
                    self.checkSave()
                    print 'Ending recorder functionalities' 

                    break
                else:
                    print 'Error did not understand command: ', method

                print '\n===================================='
                self.printHelp()
        return 

    def checkSave(self):

        #TODO - finish
        return

        if self.currentLimb == 'both':
            if self.leftSaved == False:
                answer = self.getName()
            self.rightSaved = False
        elif self.currentLimb == 'right':
            self.rightSaved = False
        elif self.currentLimb == 'left':
            self.leftSaved = False

    def updateSaved(self):
        if self.currentLimb == 'both':
            self.leftSaved = False
            self.rightSaved = False
        elif self.currentLimb == 'right':
            self.rightSaved = False
        elif self.currentLimb == 'left':
            self.leftSaved = False

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
            json.dump(self.pathDictionary, outfile, sort_keys = True, indent=4)

        # with open('data.json', 'w') as file:
        #     json.dump([key for key in self.pathDictionary : self.pathDictionary[key] for key in self.pathDictionary], file, indent=4)

        self.leftSaved = True
        self.rightSaved = True

        print 'Saved'

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

        print 'Test Concluded'

        return 

    def runPhysicalPath(self):

        if self.simType == 'physical':

            for milestone in self.leftPath:
                self.controller.controller.appendMilestoneLeft(milestone)
            for milestone in self.rightPath:
                self.controller.controller.appendMilestoneRight(milestone)



    def loadPath(self):

        self.checkSave()

        print 'Options are:'
        for key in self.pathDictionary:
            print key

        load_name = self.getName()

        if load_name in self.pathDictionary:
            self.leftPath = self.pathDictionary[load_name][1][1]
            self.rightPath = self.pathDictionary[load_name][2][1]
            print 'Loaded'
        
            self.currentLimb = self.pathDictionary[load_name][0]
            self.saveLeft = True
            self.saveRight = True

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
                self.replaceMilestone()
            elif method == 'i':
                self.insertMilestone()
            elif method == 'x':
                self.removeMilestone()
            elif method == 'a':
                self.appendMilestone()
            else:
                print 'Unknown command. Returning to main loop'
        return


    def changeLimb(self):

        #TODO check saved

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
            self.currentLimb = LEFT
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
        elif limbResponse in bothResponses:
            print 'Chagning to both limbs'
            self.currentLimb = BOTH
  
    def commandRobot(self):

        #make it so various numbers allow you to change how the robot behaves

        if self.currentLimb == BOTH:
            print 'Error, won\'t adjust both arms at once'
            return
        jointCommands = ['1','!','2','@','3','#','4','$','5','%','6','^','7','&']
        print 'Current incremental change is ', self.degreeChange, ' degrees'

        while(1):
            movement = self.command_Queue.get().lower()

            if movement is not None: 
                if movement in jointCommands:
                    self.moveRobot(movement)
                elif movement == 'h':
                    self.printRobotHelp()  
                elif movement == 'c':
                    self.changeDegrees()
                elif movement == 'q':
                    print 'Ending command of robot'
                    break
                else:
                    print 'Error did not understand command: ', method

    def moveRobot(self, movement=None):

        #TODO - move the simulation robot and keep it at that location

        print 'Moving robot'
        global LEFT_ARM_INDICES
        global RIGHT_ARM_INDICES

        joint_ones = ['1', '!']
        joint_twos = ['2', '@']
        joint_threes = ['3', '#']
        joint_fours = ['4', '$']
        joint_fives = ['5', '%']
        joint_sixes = ['6', '^']
        joint_sevens = ['7', '&']

        plus = ['1','2','3','4','5','6','7']
        minus = ['!','@','#','$','%','^','&']

        index = 0
        factor = 0
        adjustment = [0,0,0,0,0,0,0]

        if movement in joint_ones:
            index = 0
        elif movement in joint_twos:
            index = 1
        elif movement in joint_threes:
            index = 2
        elif movement in joint_fours:
            index = 3
        elif movement in joint_fives:
            index = 4
        elif movement in joint_sixes:
            index = 5
        elif movement in joint_sevens:
            index = 6

        if movement in plus:
            factor = 1
        elif movement in minus:
            factor = -1

        adjustment[index] = factor*self.degreeChange*math.pi/180.0
        currentSetup = self.controller.controller.getSensedConfig()
        
        if self.currentLimb == LEFT:
            adjustedMilestone = [currentSetup[v] for v in LEFT_ARM_INDICES]
            #print 'before adjustment', adjustedMilestone
            adjustedMilestone = vectorops.add(adjustedMilestone, adjustment)
            #print 'after adjustment', adjustedMilestone
            self.controller.controller.appendMilestoneLeft(adjustedMilestone, .1)
        elif self.currentLimb == RIGHT:
            adjustedMilestone = [currentSetup[v] for v in LEFT_ARM_INDICES]
            adjustedMilestone = vectorops.add(adjustedMilestone, adjustment)
            self.controller.controller.appendMilestoneRight(adjustedMilestone, .1)
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

        if self.simType != 'physical':
            print 'Error, physical simulation is not set up'
            return

        if self.currentLimb != 'both':
            print 'Warning, record path was initially meant to record both arms at the same time - is your system set up for only recording',
            print self.currentLimb + ' arm? (y/n)'
            response = raw_input()

            if response == 'y':
                pass
            else:
                print 'Not y, returning'
                return

        f = open(fn, 'w')
        try:
            print "Beginning capture loop, press Ctrl+C to quit"
            t0 = time.time()
            while True:
                t = time.time() - t0
                if t < 1e-3: t = 0
                if switch == 'sensed':
                    if self.curentLimb == 'both':
                        q = motion.robot.left_limb.sensedPosition()+motion.robot.right_limb.sensedPosition()
                    elif self.currentLimb == 'right':
                        q = motion.robot.right_limb.sensedPosition()
                    elif self.currentLimb == 'left':
                        q = motion.robot.left_limb.sensedPosition()
                    self.updateSaved()
                else:
                    if self.curentLimb == 'both':
                        q = motion.robot.left_limb.commandedPosition()+motion.robot.right_limb.commandedPosition()
                    elif self.currentLimb == 'right':
                        q = motion.robot.right_limb.commandedPosition()
                    elif self.currentLimb == 'left':
                        q = motion.robot.left_limb.commandedPosition()
                    self.updateSaved()
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

    def insertMilestone(self):

        global LEFT_ARM_INDICES
        global RIGHT_ARM_INDICES

        print 'Inserting'
        print 'Enter an index'

        config = self.controller.controller.getSensedConfig()
        leftArm = [config[v] for v in LEFT_ARM_INDICES]
        rightArm = [config[v] for v in RIGHT_ARM_INDICES]

        index = self.getNumber()
        if index == 'q':
            return 

        if self.currentLimb == 'left':
            if len(self.leftPath) > index:
                self.leftPath.insert(index, [a for a in leftArm])
                self.updateSaved()
            else:
                print 'Error, index outside of list bounds'
        elif self.currentLimb == 'right':
            if len(self.rightPath) > index:
                self.rightPath.insert(index, [a for a in rightArm])
                self.updateSaved()
            else:
                print 'Error, index outside of list bounds'
        elif self.currentLimb == 'both':
            if len(self.leftPath) > index:
                self.leftPath.insert(index, [a for a in leftArm])
                self.updateSaved()
            else:
                print 'Error, index outside of left list bounds'

            if len(self.rightPath) > index:
                self.rightPath.insert(index, [a for a in rightArm])
                self.updateSaved()
            else:
                print 'Error, index outside of right list bounds'
        return

    def appendMilestone(self):

        #updates saving

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
        self.updateSaved()
        return

    def removeMilestone(self):
        #updates saving

        print 'Removing'
        print 'Enter an index'

        index = self.getNumber()
        if index == 'q':
            return 


        if self.currentLimb == 'left':
            if len(self.leftPath) > index:
                self.leftPath.pop(index)
                self.updateSaved()
            else:
                print 'Error, index outside of list bounds'
        elif self.currentLimb == 'right':
            if len(self.rightPath) > index:
                self.rightPath.pop(index)
                self.updateSaved()
            else:
                print 'Error, index outside of list bounds'
        elif self.currentLimb == 'both':
            if len(self.leftPath) > index:
                self.leftPath.pop(index)
                self.updateSaved()
            else:
                print 'Error, index outside of left list bounds'
            if len(self.rightPath) > index:
                self.rightPath.pop(index)
                self.updateSaved()
            else:
                print 'Error, index outside of right list bounds'
        return

    def replaceMilestone(self):
        #updates saving

        global LEFT_ARM_INDICES
        global RIGHT_ARM_INDICES

        print 'Replacing'
        print 'Enter an index'
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
                self.updateSaved()
            else:
                print 'Error, index outside of list bounds'
        elif self.currentLimb == 'right':
            if len(self.rightPath) > index:
                self.rightPath[index] = [a for a in rightArm]
                self.updateSaved()
            else:
                print 'Error, index outside of list bounds'
        elif self.curentLimb == 'both':
            if len(self.leftPath) > index:
                self.leftPath[index] = [a for a in leftArm]
                self.updateSaved()
            else:
                print 'Error, index outside of left list bounds'
            if len(self.rightPath) > index:
                self.rightPath[index] = [a for a in rightArm]
                self.updateSaved()
            else:
                print 'Error, index outside of right list bounds'

    def reset(self):

        self.currentLimb = 'both'
        self.clearPaths()

        load_name = 'Q_DEFAULT_LOAD'

        milestone_ldefault = self.pathDictionary[load_name][1][1]
        milestone_rdefault = self.pathDictionary[load_name][2][1]

        print 'left default milestone ' + milestone_ldefault
        print 'right default milestone ' + milestone_rdefault

        self.controller.controller.appendMilestoneLeft(milestone_ldefault[0], 1)
        self.controller.controller.appendMilestoneRight(milestone_rdefault[0], 1)
        time.sleep(.1)

        self.leftSaved = True
        self.rightSaved = True
        print 'Done resetting'
  

    def resetMotionQueue(self):

        if self.simType == 'physical':
            motion.shutdown()
            setupController(physical=True)
        else:
            print 'Error, sim type is not physical'
            return

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

    def getName(self, options=None):

        name = ''
        
        while(1):
            letter = self.command_Queue.get()

            if letter == chr(13):
                return name
            elif letter is not None: 
            
                if letter in string.printable:
                    name += letter
                elif letter == '\x08' or letter == '\x7f':
                    # backspace or delete
                    name = name[:-1]
                else:
                    print 'Error, value not recognized'
                    print 'Press Enter to submit your entry or quit'

                os.system('clear')
                
                if options is not None:
                    print 'Options are:'
                    for one_option in options:
                        if name in one_option:
                            print one_option
                print '\n' + name

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

        self.checkSave()

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

    def calibrateBox(self, index=0):
        global BOX_COORDS
        while(True):
            try:
                print 'Options: x1, x2, y1, y2, z1, z2'
                input_var = raw_input("Wire Box: Enter coordinate and distance to change separated by a comma: ").split(',');
                #translational transformation
                local_coords = BOX_COORDS[index]

                if(input_var[0] == "x1" ):
                    local_coords[0][0] = local_coords[0][0] + float(input_var[1])
                elif(input_var[0] == "y1" ):
                    local_coords[0][1] = local_coords[0][1] + float(input_var[1])
                elif(input_var[0] == "z1" ):
                    local_coords[0][2] = local_coords[0][2] + float(input_var[1])
                    #rotational transformations
                elif(input_var[0] == "x2" ):
                    local_coords[1][0] = local_coords[1][0] + float(input_var[1])
                elif(input_var[0] == "y2" ):
                    local_coords[1][1] = local_coords[1][1] + float(input_var[1])
                elif(input_var[0] == "z2" ):
                     local_coords[1][2] = local_coords[1][2] + float(input_var[1])

                elif(input_var[0] == "q"):
                    break

                BOX_COORDS[index] = local_coords

            except: 
                print "input error\n"
                #print error.strerror

            time.sleep(0.1);

            if self.visualizer != None:
                self.takePicture(index)
                self.visualizer.updateBox(index, BOX_COORDS[index])

            print 'local box ', index, ' coords are ', BOX_COORDS[index]

    def calibrateCamera(self, index=0):
        global CAMERA_TRANSFORM
        global REAL_CAMERA
        if REAL_CAMERA:


            if len(CAMERA_TRANSFORM) < index:
                return False

            while(True):
                try:
                    input_var = raw_input("Camera: Enter joint and angle to change to separated by a comma: ").split(',');
                    #translational transformation
                    calibrateR = CAMERA_TRANSFORM[index][0]
                    calibrateT = CAMERA_TRANSFORM[index][1]

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



                CAMERA_TRANSFORM[index] = (calibrateR, calibrateT)
                self.takePicture(index)
                
                print 'local camera ', index, ' transform is ', CAMERA_TRANSFORM[index]

    def takePicture(self, index=0):
        global perceiver
        current_cloud = perceiver.get_current_point_cloud(*self.getCameraToWorldXform(index=index), tolist=True, index=index)
        if self.visualizer != None:
            self.visualizer.updatePoints(current_cloud)
        else:
            raise NotImplemented
            #self.controller.



    def getCameraToWorldXform(self, linkName=None, index=0):
        '''
        get current transformation (R, t) of camera in world frame. 
        '''

        global CAMERA_TRANSFORM

        if len(CAMERA_TRANSFORM) < index:
            print 'Error, camera index does not exist'
            return ([0,0,0,0,0,0,0,0,0],[0,0,0])

        if linkName != None:
            return se3.mul(self.simworld.robot(0).link(linkName).getTransform(), CAMERA_TRANSFORM[index])
        else: 
            return CAMERA_TRANSFORM[index]


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
        print 'C: Command robot to move'
        print 'W: Calibrate Camera'
        print 'B: Adjust wire boxes'
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

def go(dummy, recorder):
    recorder.run()

class Helper():
    def __init__(self, recorder):
        self.myRecorder = recorder
    def run(self):
        recorder = self.myRecorder
        control_thread = Thread(target=go, args=(1, recorder))
        print 'starting thread'
        control_thread.start()
        #control_thread.run()

        
if __name__ == "__main__":

    myRecorder = Recorder(physical=True)



