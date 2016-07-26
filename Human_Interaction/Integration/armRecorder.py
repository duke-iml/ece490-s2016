#HigherTech record

from motionController import LowLevelController
import sys, struct, time
from klampt import robotsim
import json

RIGHT = 'right'
LEFT = 'left'
BOTH = 'both'
#rospy.init_node('oeu')
#LEFT_LIMB = baxter_interface.Limb('left')
LEFT_JOINTS = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']

#RIGHT_LIMB = baxter_interface.Limb('right')
RIGHT_JOINTS = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
PATH_DIR = "../Trajectories/"
FILE_NAME = "Human_Test"


class Recorder:
    def __init__(self, controller, command_Queue):


        self.leftPath = []
        self.rightPath = []
        self.leftSaved = False
        self.rightSaved = False
        self.currentLimb = 'both'

        self.rate = -1

        self.command_Queue = command_Queue
        self.controller = controller

        simworld = self.controller.simworld
        world = self.controller.world
        sim = robotsim.Simulator(simworld)


        self.fake_controller = LowLevelController(simworld.robot(0),sim.controller(0),sim)
        

        try:
            self.file = open('data.json', 'rw') 
            self.pathDictionary = json.load(self.file)
            self.file.close()
        except:
            self.pathDictionary = {}
     
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
                elif method == 'u':
                    self.changePath()
                elif method == 'v':
                    self.changeLimb()
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
                elif method == 'q':
                    print 'Ending recorder functionalities'

                    break
                else:
                    print 'Error did not understand command: ', method
        return 

    def savePath(self):

        name = raw_input('Name the path ')


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

        currentSetup = self.controller.controller.getSensedConfig()

        startConfig = [v for v in currentSetup]
        
        if self.leftPath != []:
            for i in range(len(self.controller.left_arm_indices)):
                startConfig[self.controller.left_arm_indices[i]] = self.leftPath[0][i]

        if self.rightPath != []:
            for i in range(len(self.controller.right_arm_indices)):
                startConfig[self.controller.right_arm_indices[i]] = self.rightPath[0][i]

        #self.fake_controller.sim.getWorld().robot(0).setConfig(startConfig)
        #self.fake_controller.setVelocity([1]*61,0.1)
                   
        if self.currentLimb == 'left':
            for milestone in self.leftPath:
                self.fake_controller.appendMilestoneLeft(milestone)
        elif self.currentLimb == 'right':
            for milestone in self.rightPath:
                self.fake_controller.appendMilestoneRight(milestone)

        elif self.currentLimb == 'both':
            # rNum = len(self.rightPath)
            # lNum = len(self.leftPath)
            # minNum = min(lNum, rNum)
            # for i in range(minNum):
            #     lMilestone = self.leftPath[i]
            #     rMilestone = self.rightPath[i]
            #     self.fake_controller.appendMilestoneBoth(left=lMilestone, right=rMilestone)
            for milestone in self.rightPath:
                self.fake_controller.appendMilestoneRight(milestone)

            for milestone in self.leftPath:
                self.fake_controller.appendMilestoneLeft(milestone)
        
        #self.fake_controller.sim.getWorld().robot(0).setConfig(currentSetup)
        #self.fake_controller.sim.updateWorld()

        return 

    def loadPath(self):
        #TODO

        print 'Options are:'
        for key in self.pathDictionary:
            print key

        load_name = raw_input('Enter a name to load ')

        if load_name in self.pathDictionary:
            self.leftPath = self.pathDictionary[load_name][1][1]
            self.rightPath = self.pathDictionary[load_name][2][1]
        else:
            print 'Sorry, name not recognized.'
        return 

    def changePath(self):

        if self.rate > 0:
            self.recordPath()
        else:
            self.insertPath()
        return


    def changeLimb(self):

        print ("What type of limb would you like to record?")
        limbResponse = raw_input("Options: Left (l), Right (r), Both (b), Cancel (x) ").lower()

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
                confirm = raw_input('Delete current right path? (y/n) ')
                if confirm == 'y':
                    self.rightPath = []
            else:
                self.currentLimb = RIGHT

            self.leftPath = []
            self.currentLimb = RIGHT

    def changeHz(self):

        rate = raw_input("Enter a rate ")
        self.rate = rate

        return
    

    def recordPath(self):
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

    def insertPath(self, index=None):

        config = self.controller.controller.getSensedConfig()
        leftArm = [config[v] for v in self.controller.left_arm_indices]
        rightArm = [config[v] for v in self.controller.right_arm_indices]

        if self.currentLimb == 'left':
            if index == None:
                self.leftPath.append([a for a in leftArm])
            else:
                self.leftPath.insert(index, [a for a in leftArm])
        elif self.currentLimb == 'right':
            if index == None:
                self.rightPath.append([a for a in rightArm])
            else:
                self.rightPath.insert(index, [a for a in rightArm])
        elif self.currentLimb == 'both':
            if index == None:
                self.leftPath.append([a for a in leftArm])
            else:
                self.leftPath.insert(index, [a for a in leftArm])
            if index == None:
                self.rightPath.append([a for a in rightArm])
            else:
                self.rightPath.insert(index, [a for a in rightArm])
        return

    def replacePath(self, index):

        #could take out the leftarm/rightarm step
        config = self.controller.getSensedConfig()
        leftArm = [config[v] for v in self.controller.left_arm_indices]
        rightArm = [config[v] for v in self.controller.right_arm_indices]

        if self.currentLimb == 'left':
            self.leftPath[index] = [a for a in leftArm]
        elif self.currentLimb == 'right':
            self.rightPath[index] = [a for a in rightArm]
        elif self.curentLimb == 'both':
            self.leftPath[index] = [a for a in leftArm]
            self.rightPath[index] = [a for a in rightArm]



    def clearPaths(self):

        self.leftPath = []
        self.rightPath = []

        return

    def printPaths(self):
        print 'Left path is: ', self.leftPath
        print 'Right path is: ', self.rightPath

    def reversePath(self):

        if self.currentLimb == 'left':
            self.leftPath[index] = [a for a in leftArm]
        elif self.currentLimb == 'right':
            self.rightPath[index] = [a for a in rightArm]
        elif self.curentLimb == 'both':
            self.leftPath[index] = [a for a in leftArm]
            self.rightPath[index] = [a for a in rightArm]


    def printHelp(self):

        print 'H: Help (show this text)'
        print 'Q: Quit (return to standard controller)'
        print 'S: Save current path'
        print 'T: Test/Simulate current path'
        print 'U: Update path manually'
        print 'L: Load path from file'
        print 'V: Switch limb to record'
        print 'F: Change recording frequency'
        print 'R: Begin coninuous recording loop'
        print 'X: Clear/Reset paths'
        print 'P: Print current paths'
        return 