from Motion import motion
import time
import serial

def main():
    S_MN = 0    ## Slider min
    S_MX = 250  ## Slider max
    
    ## Fort number to automatically connect to. If set to -1, user will be prompted for a port.
    portOverride = 0;

    ## Open the serial port that the Ardino is connected to
    if (portOverride == -1):
        port = raw_input("Enter Arduino serial port number (/dev/ttyACM#): ")
        ser = serial.Serial("/dev/ttyACM" + port, 9600)
    else:
        ser = serial.Serial("/dev/ttyACM" + str(portOverride), 9600)

    ## Startup
    print "Testing APC Motion..."
    robot = motion.setup(libpath="./",klampt_model="/home/motion/Klampt/data/robots/baxter_col.rob")
    print "Starting up..."
    res = robot.startup()
    print "Start up result:",res
    if res:
        print "Is robot started?",robot.isStarted()
        print
        
        ## Infinite read/send loop
        while True:
            message = ser.readline()
            message = message[:-2]  ## Remove new line characters
            sliderRaw = str.split(message,",")
            ## Parse check before converting to int
            if len(sliderRaw) == 7:
                try:
                    sliderVals = [float(n) for n in sliderRaw]
                    print sliderVals
                    jointAngles = [0,0,0,0,0,mapVal(sliderVals[0],S_MN,S_MX,-1.5708,2.094),0]
                    print jointAngles
                    robot.right_mq.setRamp(jointAngles)
                except ValueError:
                    pass

def mapVal(valIn, inMin, inMax, outMin, outMax):
    # Figure ranges
    inRange = inMax - inMin
    outRange = outMax - outMin

    # Convert the left range into a 0-1 range (float)
    scaledVal = float(valIn - inMin) / float(inRange)

    # Convert the 0-1 range into a value in the right range.
    return outMin + (scaledVal * outRange)

if __name__ == "__main__":
    main()
