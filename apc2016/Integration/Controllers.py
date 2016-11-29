from threading import Thread, Lock


class LowLevelController:
    '''A low level controller for a non-physical robot
        Uses Klampt's physics simulation to actually physically mimic what
        the robot may do in practice. Use this controller if you can't connect
        to the robot but want to run a simulation that is physically realistic

    '''
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
    '''A low-level controller for a fake robot. You can use this controller
        to make the robot jump between various configurations as no simulation
        is occurring in this controller. Run this controller if you want to
        test quickly (Ex ik, control loop)
    '''
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
    """A low-level controller for the real robot.
        This is the controller used to get the robot to actually move    
    """

    def __init__(self,robotModel,klampt_model):
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
