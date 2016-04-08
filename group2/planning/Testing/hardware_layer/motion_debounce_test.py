from OpenGL.GL import *
from klampt import *
from klampt import WidgetSet,RobotPoser
from klampt.glrobotprogram import GLWidgetProgram
from motion_debouncer import *
import math
import time
import os
from Motion import motion
import csv
import numpy as np

#################### SYSTEM CONFIGURATION VARIABLES #######################

#change this to None to use the default mode linked to by libebolabot.so
#change this to "kinematic" to test in kinematic mode
#change this to "physical" if you are ready to test on the real robot
#mode = 'kinematic' 
mode = 'physical'

#assumes ece590-s2015 is in the home directory
klampt_model = os.path.join(os.path.expanduser("~"),"ece590-s2015/klampt_models/baxter_col.rob")
#klampt_model = os.path.join(os.path.expanduser("~"),"ece590-s2015/klampt_models/baxter_with_parallel_gripper_col.rob")
#klampt_model = os.path.join(os.path.expanduser("~"),"ece590-s2015/klampt_models/baxter_with_reflex_gripper_col.rob")

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

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        if c == 'h':
            print '[space]: send the current posed milestone'
            print 'q: clean quit'
        elif c == ' ':
            q = self.robotPoser.get()
            robot = motion.robot
            qdes = robot.left_limb.configFromKlampt(q)
            send_debounced_motion_command("left",qdes)

            qdes = robot.right_limb.configFromKlampt(q)
            send_debounced_motion_command("right",qdes)

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
    print "limb_debouncer.py: manually sends configurations to the Motion"
    print "module, testing the debouncing method."
    print "Press [space] to send milestones.  Press q to exit."
    print

    print "Loading APC Motion model",klampt_model
    motion.setup(mode=mode,klampt_model=klampt_model,libpath="./",)
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
