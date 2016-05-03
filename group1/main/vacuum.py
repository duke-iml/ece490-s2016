##Runs vacuum control commands
import os 
class vacuum:

    def on(self):
        os.system("powerusb 2:0:3 on");
    def off(self):
        os.system("powerusb 2:0:3 off");
        
