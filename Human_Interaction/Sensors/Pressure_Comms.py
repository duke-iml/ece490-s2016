
import re
import time
from serial import Serial
from serial import SerialException
import cPickle as pickle
import random
import serial.tools.list_ports

class CommPressure:
    message_time = 0.02

    # initialize the serial comm
    # def __init__(self, port="COM5"):
    # def __init__(self, port="/dev/ttyACM1"):
    #     # on linux use port="/dev/ttyUSB2"
    #     # attempt to connect
    #     while True:
    #         try:
    #             self.com = Serial(port, 9600)
    #             print('connected to arduino\n')
    #             break
    #         except SerialException:
    #             print 'Error: No device is on port' + port
    #             print 'Type an alternative ttyUSB number or a full port name'
    #             port = input('Port = ')
    #             if port.isdigit():
    #                 port = '/dev/ttyUSB' + port
    #     time.sleep(0.5)

    # initialize the serial comm
    def __init__(self, port="COM7"):
        # on linux use port="/dev/ttyUSB2"
        # attempt to connect
        foundPort = False
        ports = list(serial.tools.list_ports.comports())
        portArray = []
        for p in ports:
            print p
            if "Arduino Micro" in p[1] or "Arduino Micro" in p[2] or "VID:PID=2a03:0042" in p[1] or "VID:PID=2a03:0042" in p[2]:

                # print "Port =", p[0]
                port = p[0]
                # print port
                portArray.append(port)
                foundPort = True
        if foundPort == False:
            print "No Port Found (Arduino Micro)"

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

            self.com.setDTR(False)
            time.sleep(1)
            # toss any data already received, see
            # http://pyserial.sourceforge.net/pyserial_api.html#serial.Serial.flushInput
            self.com.flushInput()
            self.com.setDTR(True)

            print('connected to an arduino... checking ID...')
            time.sleep(0.01)

            arduinoID = self.com.readline()
            while not ("c" in arduinoID) or ("l" in arduinoID) or ("r" in arduinoID):
                print arduinoID
                arduinoID = self.com.readline()
                time.sleep(0.01)

            # print "ID:", arduinoID
            if "c" in arduinoID:
                print "ID Checked (pressure sensor", arduinoID, ")"
            else:
                print "Wrong ID:", arduinoID





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
        #print m.group(1)
        if m:
            feedback = m.group(1)
            feedback = str.split(feedback, ",")
            #print feedback
            if all(i >= 0 for i in feedback) and len(feedback) == msg_length:
                #pressure_threshold = int(feedback[0])
                pressure_val = feedback[0]
                #return [pressure_threshold, pressure_val]
                return [pressure_val]
        print 'Error: Could not read pressure - trying again.'

if __name__ == "__main__":
    ports = list(serial.tools.list_ports.comports())
    foundPort = False
    for p in ports:
        print p
        if "Arduino Micro" in p[1]:
            print "Port =", p[0]
            compress = CommPressure(port = p[0])
            foundPort = True
    if foundPort == False:
        print "No Port Found (Arduino Micro)"

    while True:
        attached = compress.read_pressure()

        # if random.uniform(0,1) > 0.9:
        #     print "attached!"
        #     attached = 0

        # attached = random.uniform(0,10)
        #print 'attached: ', attached[0], "pressure:", attached[1]
        print "pressure:", attached[0]
        with open("pressureReading.pkl", "wb") as f:
            pickle.dump(attached, f)
        time.sleep(1)
