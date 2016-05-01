
import time
from serial import Serial
from serial import SerialException


class CommVacuum:
    message_time = .01

    # initialize the serial comm
    def __init__(self, port="COM7"):
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

    # send commands to the arduino for the vacuum, should be 0 or 1 (on or off)
    def change_vacuum_state(self, comm):
        self.com.write(str(comm))
        time.sleep(self.message_time)
