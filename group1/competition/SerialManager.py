import threading
import serial
import time

class SerialManager(object):
    def __init__(self):
        self.ser = None
        self.running = False
        self.thread = None

    def connect(self, location):
        if self.ser or self.running or self.thread:
            self.endSerial()

        try:
            self.ser = serial.Serial(location, 9600, timeout=0.5)
        except:
            print("Could not open serial:", location)
            return False

        self.running = True
        self.thread = threading.Thread(target=self.workerThread)
        self.thread.start()
        return True

    def writeLine(self, msg):
        if self.ser:
            self.ser.write((msg + "\r\n").encode())

    def endSerial(self):
        self.running = False
        if self.thread:
            self.thread.join()
            self.thread = None

    def workerThread(self):
        while self.running:
            msg = self.ser.readline().decode()
            if msg == 'some message':
                pass #doSomething()
		
            time.sleep(0.1)
        self.ser.close()
        self.ser = None 
