
import time
from serial import Serial
from serial import SerialException
import serial.tools.list_ports


class CommVacuum:
    message_time = .01

    # initialize the serial comm
    def __init__(self, port="COM7"):
        # on linux use port="/dev/ttyUSB2"
        # attempt to connect
        foundPort = False
        ports = list(serial.tools.list_ports.comports())
        portArray = []
        for p in ports:
            if "Arduino Mega" in p[1]:
                # print "Port =", p[0]
                port = p[0]
                # print port
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
            print('connected to an arduino... checking ID...')
            time.sleep(0.5)

            arduinoID = self.com.readline()
            while not ("c" in arduinoID) or ("b" in arduinoID) or ("a" in arduinoID):
                arduinoID = self.com.readline()
                time.sleep(0.01)

            # print "ID:", arduinoID
            if "c" in arduinoID:
                print "ID Checked (vacuum controller)"
            else:
                print "Wrong ID:", arduinoID


    # send commands to the arduino for the vacuum, should be 0 or 1 (on or off)
    def change_vacuum_state(self, comm):
        self.com.write(str(comm))
        time.sleep(self.message_time)

if __name__ == "__main__":
    comm = CommVacuum()
    while True:
        # comm.change_vacuum_state(0)
        comm.change_vacuum_state(1)
