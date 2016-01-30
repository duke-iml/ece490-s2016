from klampt import *
from klampt import WidgetSet,RobotPoser
from klampt.glprogram import *
import math
import time
import os
from Motion import motion

#################### SYSTEM CONFIGURATION VARIABLES #######################

#change this to None to use the default mode linked to by libebolabot.so
#change this to "kinematic" to test in kinematic mode
#change this to "physical" if you are ready to test on the real robot
mode = 'kinematic' # 'physical'
#mode = 'physical'

#assumes ece590-s2015 is in the home directory
#klampt_model = os.path.join(os.path.expanduser("~"),"ece590-s2015/klampt_models/baxter_col.rob")
#klampt_model = os.path.join(os.path.expanduser("~"),"ece590-s2015/klampt_models/baxter_with_parallel_gripper_col.rob")
klampt_model = os.path.join(os.path.expanduser("~"),"ece590-s2015/klampt_models/baxter_with_reflex_gripper_col.rob")

#########################################################################

class GLWidgetProgram(GLRealtimeProgram):
    """A base class for using widgets.  Right-clicks are passed onto widgets.
    
    Subclasses should call self.widgetMaster.add() upon initializiation to
    add widgets to the program.
    """
    def __init__(self,world,name="My GL Widget Program"):
        GLRealtimeProgram.__init__(self,name)
        self.world = world
        self.world.robot(0).setConfig(motion.robot.getKlamptCommandedPosition())
        self.widgetMaster = WidgetSet()
        self.widgetButton = 2  #right-clicks
        self.draggingWidget = False

    def display(self):
        #Put your display handler here
        #the next few lines draw everything but the robot
        for i in xrange(self.world.numTerrains()):
            self.world.terrain(i).drawGL()
        for i in xrange(self.world.numRigidObjects()):
            self.world.rigidObject(i).drawGL()
        for i in xrange(1,self.world.numRobots()):
            self.world.robot(i).drawGL()
        #this line will draw the robot
        self.widgetMaster.drawGL(self.viewport())

    def idle(self):
        self.widgetMaster.idle()
        if self.widgetMaster.wantsRedraw():
            self.refresh()

    def mousefunc(self,button,state,x,y):
        #print "mouse",button,state,x,y
        if button==self.widgetButton:
            if state==0:
                if self.widgetMaster.beginDrag(x,self.height-y,self.viewport()):
                    self.draggingWidget = True
            else:
                if self.draggingWidget:
                    self.widgetMaster.endDrag()
                    self.draggingWidget = False
            if self.widgetMaster.wantsRedraw():
                self.refresh()
            self.lastx,self.lasty = x,y
            return
        GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y):
        if self.draggingWidget:
            self.widgetMaster.drag(x-self.lastx,self.lasty-y,self.viewport())
            if self.widgetMaster.wantsRedraw():
                self.refresh()
            self.lastx,self.lasty = x,y
        else:
            res = self.widgetMaster.hover(x,self.height-y,self.viewport())
            if self.widgetMaster.wantsRedraw():
                self.refresh()
            GLRealtimeProgram.motionfunc(self,x,y)

    def specialfunc(self,c,x,y):
        #Put your keyboard special character handler here
        print c,"pressed"
        self.widgetMaster.keypress(c)
        self.refresh()

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        print c,"pressed"
        self.widgetMaster.keypress(c)
        self.refresh()

class MyGLViewer(GLWidgetProgram):
    def __init__(self,world):
        GLWidgetProgram.__init__(self,world,"Manual poser")
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

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        if c == 'h':
            print '[space]: send the current posed milestone'
            print 'q: clean quit'
        elif c == ' ':
            q = self.robotPoser.get()
            robot = motion.robot
            robot.left_mq.setRamp(robot.left_limb.configFromKlampt(q))
            robot.right_mq.setRamp(robot.right_limb.configFromKlampt(q))
            qlg = robot.left_gripper.configFromKlampt(q)
            qrg = robot.right_gripper.configFromKlampt(q)
            robot.left_gripper.command(qlg,[1]*len(qlg),[1]*len(qlg))
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
    print "widget_control.py: manually sends configurations to the Motion module"
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
