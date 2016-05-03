import SerialManager

class HandController:
    def __init__(self):
        self.serial_manager = SerialManager.SerialManager()
        self.serial_manager.connect('\\COM3')

    def openHand(self):
        self.serial_manager.writeLine('open')

    def closeHand(self):
        self.serial_manager.writeLine('close')

    def cleanUpHand(self):
        self.serial_manager.endSerial()

