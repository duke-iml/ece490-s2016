
import re
import time
from serial import Serial
from serial import SerialException


class MoveSpatula:
    message_time = .01

    # COMMUNICATION COMMANDS

    # initialize the serial comm
    def __init__(self, port="COM4"):
        # on linux use port="/dev/ttyUSB2"
        # attempt to connect
        while True:
            try:
                self.com = Serial(port, 9600)
                break
            except SerialException:
                print 'Error: No device is on port' + port
                print 'Type an alternative ttyUSB number or a full port name'
                port = input('Port = ')
                if port.isdigit():
                    port = '/dev/ttyUSB' + port
        self.send_commands([0, 0, 0, 0])
        print('connected to arduino\n')
        time.sleep(0.5)

    def close(self):
        self.com.close()

    # send commands to the arduino for motor and solenoids
    def send_commands(self, motor_values):
        msg = '<'
        for i in range(len(motor_values)):
            msg += str(motor_values[i])
            if i < len(motor_values) - 1:
                msg += ','
        msg += '>'
        print msg
        self.com.write(msg)
        time.sleep(self.message_time)

    # read the motor current
    def read_current(self):
        self.com.reset_input_buffer()
        msg_length = 2
        m = 0
        while m != 1:
            msg = self.com.readline()
            m = re.search('<([\d,]+)>', msg)
            feedback = m.group(1)
            feedback = str.split(feedback, ",")
            if all(i >= 0 for i in feedback) and len(feedback) == msg_length:
                measured_current = int(feedback[0])
                print str(measured_current)
                return measured_current
            print 'Error: Could not read current - trying again.'

    # read the touch sensor
    def read_touch(self):
        self.com.reset_input_buffer()
        msg_length = 2
        m = 0
        while m != 1:
            msg = self.com.readline()
            m = re.search('<([\d,]+)>', msg)
            feedback = m.group(1)
            feedback = str.split(feedback, ",")
            if all(i >= 0 for i in feedback) and len(feedback) == msg_length:
                measured_touch = int(feedback[1])
                print str(measured_touch)
                return measured_touch
            print 'Error: Could not read touch - trying again.'

    # SPATULA MOVEMENT COMMANDS

    # advance the fence or either of the spatulas
    # configuration: 1 = fence; 2 = narrow spatula; 3 = wide spatula
    def advance(self, config):
        k = 0
        if config == 1:
            # advance just the fence
            command = [255, 0, 0, 1]
            while (k < 3) and (self.read_touch() < 700):
                self.to_stall(command)
                k += 1
            return command
        elif config == 2:
            # advance the narrow spatula
            command = [255, 0, 0, 0]
            while (k < 3) and (self.read_touch() < 700):
                self.to_stall(command)
                k += 1
            return command
        elif config == 3:
            # advance the wide spatula
            command = [255, 0, 1, 0]
            while (k < 3) and (self.read_touch() < 700):
                self.to_stall(command)
                k += 1
            return command
        else:
            print 'Error: did not input a possible configuration'

    #[0,0,0,0]
    # reset the spatula to the resting state
    def reset_spatula(self, previous_command):
        # set the spatula to go in reverse with last solenoid configuration
        command = [255, 1] + previous_command[2:4]
        # execute all solenoid configurations in case it got jammed up
        self.to_stall(command)
        #self.set_solenoids(0, 1, command)
        #command = [230, 1, 0, 1]
        #self.to_stall(command)
        #self.set_solenoids(1, 0, command)
        #command = [230, 1, 1, 0]
        #self.to_stall(command)
        #self.set_solenoids(1, 1, command)
        #command = [230, 1, 1, 1]
        #self.to_stall(command)
        self.set_solenoids(0, 0, command)
        #command = [255, 1, 0, 0]
        #self.to_stall(command)

    # run the spatula gently to stall in whichever direction the previous command specifies
    def to_stall(self, previous_command):
        self.com.reset_input_buffer()
        self.com.reset_output_buffer()
        # run the spatula to stall at three different levels
        stall_current = 550  # stall current for different motor powers
        stall_power = 255    # powers corresponding to above currents
        rest_current = 10
        curr = 0
        previous_command[0] = stall_power
        while curr < stall_current:
            curr = self.read_current()
            self.send_commands(previous_command)
            print previous_command
        time.sleep(.5)
        # after running it to stall, let the motor current go to zero
        previous_command[0] = 0
        while curr > rest_current:
            curr = self.read_current()
            self.send_commands(previous_command)
            time.sleep(.02)
        return previous_command

    # sets solenoids after letting the motor current go to zero
    def set_solenoids(self, solenoid_1, solenoid_2, previous_command):
        self.com.reset_input_buffer()
        self.com.reset_output_buffer()
        # initialize some variables
        curr = 100
        rest_current = 10
        previous_command[0] = 0
        # let the motor current go to zero
        while curr > rest_current:
            curr = self.read_current()
            print(str(curr) + '\n')
            self.send_commands(previous_command)
            time.sleep(.02)
        time.sleep(1)
        # set the solenoids
        t_end = time.time() + .2
        previous_command = previous_command[0:2] + [solenoid_1, solenoid_2]
        while time.time() < t_end:
            self.send_commands(previous_command)
            time.sleep(.02)
        return previous_command

if __name__ == "__main__":
    spat = MoveSpatula()
    spat.advance(2)
