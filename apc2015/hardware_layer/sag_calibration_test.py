from OpenGL.GL import *
from klampt import *
from klampt import WidgetSet,RobotPoser
from klampt.glrobotprogram import GLWidgetProgram
import math
import time
import os
from Motion import motion
import csv
import sys
from motion_debouncer import *

#################### SYSTEM CONFIGURATION VARIABLES #######################

#change this to None to use the default mode linked to by libebolabot.so
#change this to "kinematic" to test in kinematic mode
#change this to "physical" if you are ready to test on the real robot
#mode = 'kinematic' 
mode = 'physical'

#assumes ece590-s2015 is in the home directory
#klampt_model = os.path.join(os.path.expanduser("~"),"ece590-s2015/klampt_models/baxter_col.rob")
#klampt_model = os.path.join(os.path.expanduser("~"),"ece590-s2015/klampt_models/baxter_with_parallel_gripper_col.rob")
#klampt_model = os.path.join(os.path.expanduser("~"),"ece590-s2015/klampt_models/baxter_with_reflex_gripper_col.rob")
klampt_model = os.path.join(os.path.expanduser("~"),"ece590-s2015/klampt_models/baxter_with_scoop_col.rob")

#########################################################################


class MyGLViewer(GLWidgetProgram):
    def __init__(self,world):
        GLWidgetProgram.__init__(self,world,"Manual poser")

        self.world.robot(0).setConfig(motion.robot.getKlamptCommandedPosition())
        #motion.robot.left_limb.enableSelfCollisionAvoidance(False)
        #motion.robot.right_limb.enableSelfCollisionAvoidance(False)

        self.robotPoser = RobotPoser(world.robot(0))
        self.widgetMaster.add(self.robotPoser)
        robot = world.robot(0)
        left_arm_link_names = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
        right_arm_link_names = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']
        self.left_arm_link_indices = [robot.getLink(l).index for l in left_arm_link_names]
        self.right_arm_link_indices = [robot.getLink(l).index for l in right_arm_link_names]
        self.outputFileLeft = open("sag_calibration_test_left.csv","w")
        self.outputFileRight = open("sag_calibration_test_right.csv","w")
        #write headers
        self.outputFileLeft.write("time,")
        self.outputFileLeft.write(",".join("qsns"+str(i) for i in range(1,8)))
        self.outputFileLeft.write(",")
        self.outputFileLeft.write(",".join("qcmd"+str(i) for i in range(1,8)))
        self.outputFileLeft.write("\n")
        self.outputFileLeft.write("time,")
        self.outputFileRight.write(",".join("qsns"+str(i) for i in range(1,8)))
        self.outputFileRight.write(",")
        self.outputFileRight.write(",".join("qcmd"+str(i) for i in range(1,8)))
        self.outputFileRight.write("\n")
        self.t0 = time.time()
        self.lastLeftMove = -100
        self.lastRightMove = -100 

    def display(self):
        #Put your display handler here
        #the current example draws the sensed robot in grey and the
        #commanded configurations in transparent green

        #this line with draw the world
        robot = motion.robot
        q = robot.getKlamptSensedPosition()
        self.world.robot(0).setConfig(q)
        self.world.robot(0).drawGL()
        GLWidgetProgram.display(self)

        #draw commanded configuration
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0,1,0,0.5])
        q = robot.getKlamptCommandedPosition()
        self.world.robot(0).setConfig(q)
        self.world.robot(0).drawGL(False)
        glDisable(GL_BLEND)

    def idle(self):
        GLWidgetProgram.idle(self)
        t = time.time()-self.t0
        if motion.robot.left_mq.moving():
            self.lastLeftMove = t
        if motion.robot.right_mq.moving():
            self.lastRightMove = t
        if t < self.lastLeftMove + 3:
            #print "Saving left",t
            qsensed = motion.robot.left_limb.sensedPosition()
            qcmd = motion.robot.left_limb.commandedPosition()
            self.outputFileLeft.write(str(t)+",")
            self.outputFileLeft.write(','.join(str(v) for v in qsensed+qcmd))
            self.outputFileLeft.write('\n')
        if t < self.lastRightMove + 3:
            #print "Saving right",t
            qsensed = motion.robot.right_limb.sensedPosition()
            qcmd = motion.robot.right_limb.commandedPosition()
            self.outputFileRight.write(str(t)+",")
            self.outputFileRight.write(','.join(str(v) for v in qsensed+qcmd))
            self.outputFileRight.write('\n')

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        if c == 'h':
            print '[space]: send the current posed milestone'
            print 'q: clean quit'
        elif c == ' ':
            q = self.robotPoser.get()
            robot = motion.robot
            send_debounced_motion_command(motion,'left',robot.left_limb.configFromKlampt(q))
            send_debounced_motion_command(motion,'right',robot.right_limb.configFromKlampt(q))
            #robot.left_mq.setRamp(robot.left_limb.configFromKlampt(q))
            #robot.right_mq.setRamp(robot.right_limb.configFromKlampt(q))
            qlg = robot.left_gripper.configFromKlampt(q)
            qrg = robot.right_gripper.configFromKlampt(q)
            if qlg:
                robot.left_gripper.command(qlg,[1]*len(qlg),[1]*len(qlg))
            if qrg:
                robot.right_gripper.command(qrg,[1]*len(qrg),[1]*len(qrg))
            #robot.left_mq.setRamp([q[i] for i in self.left_arm_link_indices])
            #robot.right_mq.setRamp([q[i] for i in self.right_arm_link_indices])
        elif c == 'q':
            motion.robot.shutdown()
            exit(0)
        else:
            GLWidgetProgram.keyboardfunc(self,c,x,y)
            self.refresh()


if __name__ == "__main__":
    print "sag_calibration_test.py [file]: tests a sag calibration file"
    print "Default value is baxter_motor_calibration.json"
    print "Press [space] to send milestones.  Press q to exit."
    print

    print "Loading APC Motion model",klampt_model
    motion.setup(mode=mode,klampt_model=klampt_model,libpath="./",)
    calibfile = "baxter_motor_calibration.json"
    if len(sys.argv) > 1:
        calibfile = sys.argv[1]
    print "Loading calibration file",calibfile
    res = motion.robot.loadCalibration(calibfile)
    if not res:
        print "Error loading Baxter calibration"
        exit(1)
    res = motion.robot.startup()
    if not res:
        print "Error starting up APC Motion"
        exit(1)
    time.sleep(0.1)
    world = WorldModel()
    res = world.readFile(klampt_model)
    if not res:
        raise RuntimeError("Unable to load APC Motion model "+fn)
    
    viewer = MyGLViewer(world)
    viewer.run()
