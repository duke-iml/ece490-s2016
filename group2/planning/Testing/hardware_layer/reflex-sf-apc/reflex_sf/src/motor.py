#!/usr/bin/env python
#
# RightHand Robotics code for interfacing with components of the ReFlex SF hand
#

from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import TorqueEnable
from dynamixel_controllers.srv import SetSpeed
import rospy
import math
from std_msgs.msg import Float64

LOAD_FILTER_CONSTANT = 0.4

class Motor(object):
    def __init__(self, name):
        '''
        Assumes that "name" is the name of the controller with a preceding
        slash, e.g. /reflex_sf_f1
        '''
        self.name = name[1:]
        self.zero_point = rospy.get_param(self.name + '/zero_point')
        self.angle_range = rospy.get_param(self.name + '/angle_range')
        self.flipped = rospy.get_param(self.name + '/flipped')
        self.raw_direction = -1.0 if self.flipped else 1.0
        self.current_raw_position = 0.0
        self.current_position = 0.0
        self.current_raw_goal_position = None
        self.current_goal_position = None
        self.current_raw_commanded_position = 0.0
        self.current_commanded_position = 0.0
        self.current_velocity = 0
        self.controller_enabled = True
        self.is_moving = False
        self.nominalSpeed = rospy.get_param(self.name + '/joint_speed')
        self.acceleration = 3.0
        self.command_speed = self.nominalSpeed
        self.filtered_load = 0
        self.closing = False
        self.overload_count = 0
        self.OVERLOAD_DURATION = 5
        self.OVERLOAD_THRESHOLD = 0.4
        self.BLOCKED_VELOCITY = 0.1
        self.deadTemperatureThreshold = 45
        self.nominalTemperature = 32
        self.nominalLoad = rospy.get_param(self.name + '/nominal_load')
        self.loadThreshold = self.nominalLoad
        self.pub = rospy.Publisher(name + '/command', Float64, queue_size=10)
        self.set_speed_service = rospy.ServiceProxy(name+'/set_speed', SetSpeed)
        self.torque_enable_service = rospy.ServiceProxy(name + '/torque_enable',
                                                        TorqueEnable)
        self.torque_enabled = True
        self.sub = rospy.Subscriber(name + '/state', JointState,
                                    self.receiveStateCb)
        #a function (param,data) that is called whenever the JointState message is received
        self.subscriber_hook = None

    def enableController(self,active):
        self.controller_enabled = active

    def setMotorZeroPoint(self):
        self.zero_point = self.current_raw_position
        rospy.set_param(self.name + '/zero_point', self.current_raw_position)

    def getRawCurrentPosition(self):
        return self.current_raw_position

    def getCurrentPosition(self):
        return self.current_position

    def getRawCurrentGoalPosition(self):
        return self.current_raw_goal_position

    def getCurrentGoalPosition(self):
        return self.current_goal_position

    def setRawMotorPosition(self, goal_pos):
        '''
        Sets the given raw position to the motor
        '''
        self.current_raw_goal_position = goal_pos
        if self.flipped:
            self.current_goal_position = self.zero_point - self.current_raw_goal_position
        else:
            self.current_goal_position = self.current_raw_goal_position - self.zero_point
        if self.controller_enabled:
            print "Warning: setRawMotorPosition called while controller is running"
        self.pub.publish(goal_pos)

    def setMotorPosition(self, goal_pos):
        '''
        Bounds the given motor command and sets it to the motor
        '''
        print "Command sent to",self.name,":",goal_pos,"->",self.checkMotorCommand(goal_pos)
        if goal_pos > self.current_position:
            closing = True
            self.is_moving = True
        else:
            closing = False
            self.is_moving = True
            #reset state estimator
            self.filtered_load = 0
        self.current_goal_position = min(max(goal_pos,0),self.angle_range)
        self.current_raw_goal_position = self.checkMotorCommand(goal_pos)
        if not self.controller_enabled:
            self.pub.publish(self.checkMotorCommand(goal_pos))

    def checkMotorCommand(self, angle_command):
        '''
        Returns the given command if it's within the allowable range, returns
        the bounded command if it's out of range
        '''
        angle_command = self.correctMotorOffset(angle_command)
        if self.flipped:
            bounded_command = max(min(angle_command, self.zero_point),
                                  self.zero_point - self.angle_range)
        else:
            bounded_command = min(max(angle_command, self.zero_point),
                                  self.zero_point + self.angle_range)
        return bounded_command

    def correctMotorOffset(self, angle_command):
        '''
        Adjusts for the zero point offset
        '''
        if self.flipped:
            return self.zero_point - angle_command
        else:
            return self.zero_point + angle_command

    def setForceThreshold(self, forceThreshold, relative=True):
        if relative:
            self.loadThreshold = forceThreshold*self.nominalLoad
        else:
            self.loadThreshold = forceThreshold

    def setSpeed(self,speed, relative=True):
        if relative:
            self.command_speed = speed*self.nominalSpeed
        else:
            self.command_speed = speed
 
        try:
            self.set_speed_service(self.command_speed*0.5)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def enableTorque(self):
        self.torque_enabled = True
        self.torque_enable_service(True)

    def disableTorque(self):
        self.torque_enabled = False
        self.torque_enable_service(False)

    def checkForOverload(self, load, velocity):
        if abs(load) > self.OVERLOAD_THRESHOLD\
           and abs(velocity) < self.BLOCKED_VELOCITY:
            self.overload_count += 1
            if self.overload_count >= self.OVERLOAD_DURATION:
                print("Motor %s overloaded to load=%f, loosening" %(self.name,load))
                self.loosen()
                return True
        else:
            self.overload_count = 0
        return False

    def tighten(self, tighten_angle=0.05):
        '''
        Takes the given angle offset in radians and tightens the motor
        '''
        if self.flipped:
            tighten_angle *= -1
        self.setRawMotorPosition(self.current_raw_position + tighten_angle)

    def loosen(self, loosen_angle=0.05):
        '''
        Takes the given angle offset in radians and loosens the motor
        '''
        if self.flipped:
            loosen_angle *= -1
        self.setRawMotorPosition(self.current_raw_position - loosen_angle)

    def compute_command(self,data):
        #safety check 1: check for temperatures
        if data.motor_temps[0] > self.deadTemperatureThreshold:
            print "Motor %s hit temperature threshold, disabling torques until temperature reaches %d\n"%(self.name,self.nominalTemperature)
            self.disableTorque()
            return
        if self.torque_enabled == False and data.motor_temps[0] <= self.nominalTemperature:
            print "Motor %s cooled down enough, now enabling torques\n"%(self.name,)
            self.enableTorque()

        #safety check 2: do force checking
        if self.checkForOverload(data.load, data.velocity):
            return

        #do normal command loop
        #TEMP: estimated control rate at 5Hz?
        command = None
        dt = 0.2
        loadDeadband = 0.05
        if 'f1' in self.name:
            print "Motor %s cur %f load %f goal %s"%(self.name,self.current_position,self.filtered_load,str(self.current_goal_position))
            print "Threshold %f velocity %f"%(self.loadThreshold,data.velocity)
        #check if blocked or normal operation
        if self.filtered_load > self.loadThreshold and abs(data.velocity) < 0.5:
            #blocked operation: work to reduce load to loadThreshold
            loadGain = 0.1
            adjustment = (self.filtered_load - self.loadThreshold)*loadGain
            if 'f1' in self.name:
                print("Motor %s over force limit %f > %f, loosening by %f" %(self.name,self.filtered_load,self.loadThreshold,adjustment))
            command = self.current_position - adjustment
        elif self.filtered_load > self.loadThreshold-loadDeadband and abs(data.velocity) < 0.5:
            #don't do anything
            pass
        elif self.current_goal_position != None and self.is_moving:
            #normal operation: constant velocity curve
            #TODO: trapezoidal velocity profile
            err = self.current_goal_position - self.current_position
            if abs(err) <= self.command_speed*dt:
                command = self.current_goal_position
                self.is_moving = False
            else:
                command = self.current_position + dt*math.copysign(self.command_speed,err)
        if command != None:
            self.pub.publish(self.checkMotorCommand(command))            


    def receiveStateCb(self, data):
        self.current_raw_position = data.current_pos
        if self.flipped:
            self.current_position = self.zero_point - self.current_raw_position
        else:
            self.current_position = self.current_raw_position - self.zero_point

        self.current_velocity = data.velocity*self.raw_direction

        #update command data structures
        self.current_raw_command_position = data.goal_pos
        if self.flipped:
            self.current_command_position = self.zero_point - self.current_raw_command_position
        else:
            self.current_command_position = self.current_raw_command_position - self.zero_point

        #update state estimator
        self.filtered_load = self.filtered_load + LOAD_FILTER_CONSTANT*(-data.load*self.raw_direction - self.filtered_load)

        if self.controller_enabled:
            #calculate and send the command
            self.compute_command(data)
        else:
            self.is_moving = data.is_moving

        #call hand subscriber hook
        if self.subscriber_hook:
            self.subscriber_hook(self.name,data)
