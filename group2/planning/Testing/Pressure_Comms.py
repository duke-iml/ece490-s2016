
import re
import time
from serial import Serial
from serial import SerialException
import cPickle as pickle
import random

class CommPressure:
    message_time = .02

    # initialize the serial comm
    # def __init__(self, port="COM5"):
    def __init__(self, port="/dev/ttyACM0"):
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
        print('connected to arduino\n')
        time.sleep(0.5)

    def close(self):
        self.com.close()

    # read the pressure
    def read_pressure(self):
        self.com.reset_input_buffer()
        self.com.reset_output_buffer()
        time.sleep(self.message_time)
        msg_length = 1
        msg = self.com.readline()
        m = re.search('<([\d,]+)>', msg)
        if m:
            feedback = m.group(1)
            feedback = str.split(feedback, ",")
            if all(i >= 0 for i in feedback) and len(feedback) == msg_length:
                pressure_threshold = int(feedback[0])
                return pressure_threshold
        print 'Error: Could not read pressure - trying again.'

if __name__ == "__main__":
    compress = CommPressure()
    while True:
        attached = compress.read_pressure()

        # if random.uniform(0,1) > 0.9:
        #     print "attached!"
        #     attached = 0

        # attached = random.uniform(0,10)
        print 'attached: ', attached
        with open("pressureReading.pkl", "wb") as f:
            pickle.dump(attached, f)
        time.sleep(1)
