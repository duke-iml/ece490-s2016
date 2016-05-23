from SerialManager import SerialManager

class HandHandler(object):
    def __init__(self):
        self.sm = SerialManager()
        self.sm.connect('/dev/ttyACM1')

    def openHand(self):
        self.sm.writeLine('o')

    def closeHand(self):
        self.sm.writeLine('c')
