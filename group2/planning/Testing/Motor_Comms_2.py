
import re
import time
from serial import Serial
from serial import SerialException
import serial.tools.list_ports


class MoveSpatula:
    message_time = .01

    # COMMUNICATION COMMANDS

    # initialize the serial comm
    def __init__(self):
        # on linux use port="/dev/ttyUSB2"
        # attempt to connect
        foundPort = False
        ports = list(serial.tools.list_ports.comports())
        portArray = []
        for p in ports:
            if "Arduino Mega" in p[1]:
                #print "Port =", p[0]
                port = p[0]
                #print port
                portArray.append(port)
                foundPort = True
        if foundPort == False:
            print "No Port Found (Arduino Mega)"

        for i in range(len(portArray)):
            port = portArray[i]
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
            print('connected to an arduino... checking ID (motor)...')
            time.sleep(0.5)

            # Reset Arduino
            self.com.setDTR(False)
            time.sleep(0.022)
            self.com.setDTR(True)

            arduinoID = self.com.readline()
            while not (("c" in arduinoID) or ("b" in arduinoID) or ("a" in arduinoID)):
                arduinoID = self.com.readline()
                time.sleep(0.01)

            # print "ID:", arduinoID
            if "b" in arduinoID:
                print "ID Checked (motor controller)"
                break
            else:
                print "Wrong ID:", arduinoID

        self.send_commands([0, 0, 0, 0])
        time.sleep(0.5)

    def close(self):
        self.send_commands([0, 0, 0, 0])
        self.com.close()

    # send commands to the arduino for motor and solenoids
    def send_commands(self, motor_values):
        self.com.reset_output_buffer()
        msg = '<'
        for i in range(len(motor_values)):
            msg += str(motor_values[i])
            if i < len(motor_values) - 1:
                msg += ','
        msg += '>'
        #print msg
        self.com.write(msg)
        self.com.flush()

    # read the motor current
    def read_current(self):
        self.com.reset_input_buffer()
        msg_length = 3
        m = 0
        while m != 1:
            msg = self.com.readline()
            m = re.search('<([\d,]+)>', msg)
            if m==None:
                continue
            feedback = m.group(1)
            feedback = str.split(feedback, ",")
            if all(i >= 0 for i in feedback) and len(feedback) == msg_length:
                measured_current = int(feedback[0])
                #print str(measured_current)
                return measured_current
            print 'Error: Could not read current - trying again.'

    # read the touch sensor
    def read_switches(self, which_switch):
        self.com.reset_input_buffer()
        switches = [0, 0]
        msg_length = 3
        m = 0
        while m != 1:
            msg = self.com.readline()
            m = re.search('<([\d,]+)>', msg)
            if m==None:
                continue
            feedback = m.group(1)
            feedback = str.split(feedback, ",")
            if all(i >= 0 for i in feedback) and len(feedback) == msg_length:
                switches[0] = int(feedback[1])
                switches[1] = int(feedback[2])
                print str(switches)
                # switches are defined as [far switch, near switch]
                return switches[which_switch]
            print 'Error: Could not read touch - trying again.'

    # SPATULA MOVEMENT COMMANDS

    # advance the fence or either of the spatulas
    # configuration: 1 = fence; 2 = narrow spatula; 3 = wide spatula
    def advance(self, config):
        if config == 1:
            command = [255, 1, 0, 1]  # advance just the wide spatula
        elif config == 2:
            command = [255, 1, 0, 0]  # advance the narrow spatula
        elif config == 3:
            command = [255, 1, 1, 0]  # advance the fence
        else:
            print 'Error: did not input a possible configuration'
        self.set_solenoids(command[1], command[2], command)
        self.to_stall(command, 0)
        time.sleep(.02)

    def prepare(self):
        command = [255, 1, 0, 0]
        stall_current = 600
        advance_time = 1
        t_end = time.time() + advance_time
        curr = 0
        while (time.time() < t_end) & (curr < stall_current):
            curr = self.read_current()
            #print "current =", curr
            self.send_commands(command)
        command = [0, 0, 0, 0]
        self.send_commands(command)        

    # [0,0,0,0]
    # reset the spatula to the resting state
    def reset_spatula(self):
        # set the spatula to go in reverse with last solenoid configuration
        command = [255, 0, 0, 0]
        self.set_solenoids(0, 0, command)
        self.to_stall(command, 1)

    # run the spatula to stall in whichever direction the previous command specifies
    def to_stall(self, previous_command, which_switch):
        stall_current = 600  # stall current
        stall_power = 255    # motor power
        rest_current = 10
        switch_val = 0
        curr = 0
        previous_command[0] = stall_power
        while (curr < stall_current) & (switch_val == 0):
            curr = self.read_current()
            switch_val = self.read_switches(which_switch)
            #print(str(switch_val))
            self.send_commands(previous_command)
        # after running it to stall, let the motor current go to zero
        previous_command[0] = 0
        while curr > rest_current:
            curr = self.read_current()
            self.send_commands(previous_command)
            # time.sleep(.02)

    # sets solenoids after letting the motor current go to zero
    def set_solenoids(self, solenoid_1, solenoid_2, previous_command):
        # initialize some variables
        curr = 100
        rest_current = 10
        previous_command[0] = 0
        # let the motor current go to zero
        while curr > rest_current:
            curr = self.read_current()
            #print(str(curr) + '\n')
            self.send_commands(previous_command)
            time.sleep(.02)
        time.sleep(.5)
        # set the solenoids
        previous_command = previous_command[0:2] + [solenoid_1, solenoid_2]
        self.send_commands(previous_command)

if __name__ == "__main__":
    spat = MoveSpatula()
    #spat.reset_spatula()
    # spat.advance(2)
    #spat.prepare()
    #print spat.read_switches(0)
