
import time
from serial import Serial
from serial import SerialException
import serial.tools.list_ports


class CommVacuum:
    message_time = .01

    # initialize the serial comm
    def __init__(self):
        # on linux use port="/dev/ttyUSB2"
        # attempt to connect
        foundPort = False
        ports = list(serial.tools.list_ports.comports())
        portArray = []

        print ports
        for p in ports:
            if "Arduino Mega" in p[1]:
                # print "Port =", p[0]
                port = p[0]
                # print port
                portArray.append(port)
                foundPort = True
        if foundPort == False:

            portArray.append('/dev/ttyACM1')
            print "No Port Found (Arduino Mega)"
            print "Attempting to input dev/ttyACM1"



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
            print('connected to an arduino... checking ID (vacuum)...')
            time.sleep(0.5)

            # Reset Arduino
            self.com.setDTR(False) # Drop DTR
            time.sleep(0.022)    # Read somewhere that 22ms is what the UI does.
            self.com.setDTR(True)  # UP the DTR back

            arduinoID = self.com.readline()
            while not (("c" in arduinoID) or ("b" in arduinoID) or ("a" in arduinoID)):
                arduinoID = self.com.readline()
                time.sleep(0.01)

            # print "ID:", arduinoID
            if "c" in arduinoID:
                print "ID Checked (vacuum controller)"
                break
            else:
                print "Wrong ID:", arduinoID


    # send commands to the arduino for the vacuum, should be 0 or 1 (on or off)
    def change_vacuum_state(self, comm):
        self.com.write(str(comm))
        time.sleep(self.message_time)

if __name__ == "__main__":
    comm = CommVacuum()
    while True:
        comm.change_vacuum_state(0)
        #comm.change_vacuum_state(1)
