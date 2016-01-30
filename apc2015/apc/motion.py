import baxter_scoop as baxter
from threading import Lock
from klampt import loader
import time

import logging
logger = logging.getLogger(__name__)

class LowLevelController:
    """An abstract interface to the Baxter robot, either simulated or virtual"""
    def __init__(self,robotModel):
        self.robotModel = robotModel
    def getSensedConfig(self):
        raise NotImplementedError()
    def getSensedVelocity(self):
        raise NotImplementedError()
    def getCommandedConfig(self):
        raise NotImplementedError()
    def getCommandedVelocity(self):
        raise NotImplementedError()
    def setPIDCommand(self,configuration,velocity):
        """Sets the controller to a PID command mode"""
        raise NotImplementedError()
    def setMilestone(self,destination,endvelocity=None):
        """Immediately sets the motion queue to move to the given
        milestone.  If endvelocity is given, then the end of the
        queue will be moving at that velocity.  Otherwise, the end
        velocity will be zero."""
        raise NotImplementedError()
    def appendMilestone(self,destination,endvelocity=None):
        """Appends a milestone to the motion queue.  If endvelocity
        is given, then the end of the queue will be moving at that velocity.
        Otherwise, the end velocity will be zero."""
        raise NotImplementedError()
    def isMoving(self):
        raise NotImplementedError()
    def remainingTime(self):
        raise NotImplementedError()
    def commandGripper(self,limb,command):
        """Sends the command to the indicated gripper.
        For the parallel-jaw gripper, [0] is closed, [1] is open
        Warning: don't do this while moving"""
        raise NotImplementedError()


class SimLowLevelController(LowLevelController):
    """A low-level interface to the simulated Baxter robot (with parallel jaw
    grippers).  Does appropriate locking for multi-threaded use."""
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
        #if endvelocity == None: self.controller.setMilestone(destination)
        if endvelocity == None: self.controller.addMilestoneLinear(destination)
        else: self.controller.setMilestone(destination,endvelocity)
        self.lock.release()
    def appendMilestone(self,destination,endvelocity=None):
        """Appends a milestone to the motion queue.  If endvelocity
        is given, then the end of the queue will be moving at that velocity.
        Otherwise, the end velocity will be zero."""
        self.lock.acquire()
        #if endvelocity == None: self.controller.addMilestone(destination)
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
        baxter.set_model_gripper_command(self.robotModel,limb,command)
        self.controller.setMilestone(self.robotModel.getConfig())
        self.lock.release()

class FakeLowLevelController(LowLevelController):
    """A faked low-level interface to the Baxter robot (with parallel jaw
    grippers).  Does appropriate locking for multi-threaded use.
    Useful for prototyping."""
    def __init__(self,robotModel,unused=None):
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
        baxter.set_model_gripper_command(self.robotModel,limb,command)
        self.config = self.robotModel.getConfig()
        self.lastCommandTime = time.time()
        self.lock.release()

from hardware_layer.Motion import motion
from hardware_layer import motion_debouncer

class PhysicalLowLevelController(LowLevelController):
    """A low-level controller for the real robot."""
    def __init__(self,robotModel,klampt_model="baxter_with_scoop_col.rob"):
        print "Setting up motion_physical library with model",klampt_model
        motion.setup(klampt_model = "klampt_models/"+klampt_model)
        
        self.left_arm_indices = [robotModel.getLink(i).index for i in baxter.left_arm_link_names]
        self.right_arm_indices = [robotModel.getLink(i).index for i in baxter.right_arm_link_names]
        if not motion.robot.startup():
            raise RuntimeError("Robot could not be started")
            
        motion.robot.left_limb.enableSelfCollisionAvoidance(False)
        motion.robot.right_limb.enableSelfCollisionAvoidance(False)
        logger.warn('!!! Baxter self-collision avoidance disabled !!!')

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
        motion_debouncer.send_debounced_motion_command(motion,'left',[destination[v] for v in self.left_arm_indices],append=False)
        motion_debouncer.send_debounced_motion_command(motion,'right',[destination[v] for v in self.right_arm_indices],append=False)
        return True        
    def appendMilestone(self,destination,endvelocity=None):
        #if not motion.robot.left_mq.appendRamp([destination[v] for v in self.left_arm_indices]): raise RuntimeError()
        #if not motion.robot.right_mq.appendRamp([destination[v] for v in self.right_arm_indices]): raise RuntimeError()
        #new debounced code
        motion_debouncer.send_debounced_motion_command(motion,'left',[destination[v] for v in self.left_arm_indices],append=True)
        motion_debouncer.send_debounced_motion_command(motion,'right',[destination[v] for v in self.right_arm_indices],append=True)
        return True        
    def isMoving(self):
        return motion.robot.moving()
    def remainingTime(self):
        return max(motion.robot.left_mq.moveTime(),motion.robot.right_mq.moveTime())
    def commandGripper(self,limb,command):
        if limb=="left":
            if command==[0]:
                motion.robot.left_gripper.close()
            elif command==[1]:
                motion.robot.left_gripper.open()
            else:
                motion.robot.left_gripper.command(command,[1]*len(command),[1]*len(command))
        else:
            if command==[0]:
                motion.robot.right_gripper.close()
            elif command==[1]:
                motion.robot.right_gripper.open()
            else:
                motion.robot.right_gripper.command(command,[1]*len(command),[1]*len(command))

class LoggingController(LowLevelController):
    """Piggybacks on top of another controller"""
    def __init__(self,base,filename):
        print "LOGGING LOW LEVEL COMMANDS TO",filename
        self.file = open(filename,'w')
        self.starttime = time.time()
        self.base = base
        self.file.write('0 startConfig\t'+loader.writeVector(self.base.getCommandedConfig())+'\n')
    def getSensedConfig(self):
        return self.base.getSensedConfig()
    def getSensedVelocity(self):
        return self.base.getSensedVelocity()
    def getCommandedConfig(self):
        return self.base.getCommandedConfig()
    def getCommandedVelocity(self):
        return self.base.getCommandedVelocity()
    def setPIDCommand(self,configuration,velocity):
        t = time.time()-self.starttime
        self.file.write(str(t)+' PID\t'+loader.writeVector(configuration)+'\t'+loader.writeVector(velocity)+'\n')
        return self.base.setPIDCommand(configuration,velocity)
    def setMilestone(self,destination,endvelocity=None):
        t = time.time()-self.starttime
        self.file.write(str(t)+' setMilestone\t'+loader.writeVector(destination))
        if endvelocity != None:
            self.file.write('\t'+loader.writeVector(endvelocity))
        self.file.write('\n')
        return self.base.setMilestone(destination,endvelocity)
    def appendMilestone(self,destination,endvelocity=None):
        t = time.time()-self.starttime
        self.file.write(str(t)+' appendMilestone\t'+loader.writeVector(destination))
        if endvelocity != None:
            self.file.write('\t'+loader.writeVector(endvelocity))
        self.file.write('\n')
        return self.base.appendMilestone(destination,endvelocity)
    def isMoving(self):
        return self.base.isMoving()
    def remainingTime(self):
        return self.base.remainingTime()
    def commandGripper(self,limb,command):
        t = time.time()-self.starttime
        self.file.write(str(t)+' commandGripper\t'+limb+'\t'+loader.writeVector(command)+'\n')
        return self.base.commandGripper(limb,command)
