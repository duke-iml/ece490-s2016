from klampt import *
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

#assumes ece590-s2015 is in the home directory
#klampt_model = os.path.join(os.path.expanduser("~"),"ece590-s2015/klampt_models/baxter_col.rob")
klampt_model = os.path.join(os.path.expanduser("~"),"ece590-s2015/klampt_models/baxter_with_parallel_gripper_col.rob")
#klampt_model = os.path.join(os.path.expanduser("~"),"ece590-s2015/klampt_models/baxter_with_reflex_gripper_col.rob")

#########################################################################

class MyEEDriveProgram(GLRealtimeProgram):
    """A base class for using widgets.  Right-clicks are passed onto widgets.
    
    Subclasses should call self.widgetMaster.add() upon initializiation to
    add widgets to the program.
    """
    def __init__(self,world,name="My GL Widget Program"):
        GLRealtimeProgram.__init__(self,name)
        self.world = world
        self.world.robot(0).setConfig(motion.robot.getKlamptCommandedPosition())
        self.driveVel = [0,0,0]
        self.driveAngVel = [0,0,0]
        self.driveArm = 'l'
        self.increment = 0.05
        self.incrementang = 0.1
        self.trace_int = {'l':[],
                          'r':[]}
        self.trace_sensed = {'l':[],
                             'r':[]}
        print "Moving to neutral configuration"
        q = motion.robot.left_limb.commandedPosition()
        q[1] = -1.0;
        q[3] = 2.0;
        q[5] = 1.0;
        motion.robot.left_mq.appendLinearRamp(q)
        q = motion.robot.right_limb.commandedPosition()
        q[1] = -1.0;
        q[3] = 2.0;
        q[5] = 1.0;
        motion.robot.right_mq.setRamp(q)

    def display(self):
        #Put your display handler here
        robot = motion.robot
        q = robot.getKlamptSensedPosition()
        self.world.robot(0).setConfig(q)
        self.world.drawGL()

        #draw commanded configuration
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0,1,0,0.5])
        q = robot.getKlamptCommandedPosition()
        self.world.robot(0).setConfig(q)
        self.world.robot(0).drawGL(False)
        glDisable(GL_BLEND)

        #draw trace
        glDisable(GL_LIGHTING)
        glColor3f(0,1,0)
        for arm in self.trace_int:
            glBegin(GL_LINE_STRIP)
            for T in self.trace_int[arm]:
                glVertex3f(*T[1])
            glEnd()
        glColor3f(1,0.5,0)
        for arm in self.trace_sensed:
            glBegin(GL_LINE_STRIP)
            for T in self.trace_sensed[arm]:
                glVertex3f(*T[1])
            glEnd()
        glEnable(GL_LIGHTING)
        return

    def specialfunc(self,c,x,y):
        #Put your keyboard special character handler here
        print c,"pressed"
        self.refresh()

    def updateDrive(self):
        if motion.robot.left_mq.moving() or motion.robot.right_mq.moving():
            return
        print "Drive angular vel.",self.driveAngVel,", translational vel.",self.driveVel
        if self.driveArm=='l':
            motion.robot.left_ee.driveCommand(self.driveAngVel,self.driveVel)
        elif self.driveArm=='r':
            motion.robot.right_ee.driveCommand(self.driveAngVel,self.driveVel)
        else:
            raise ValueError("Invalid value for self.driveArm")

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        print c,"pressed"
        if c.lower()=='l':
            motion.robot.right_ee.driveCommand([0]*3,[0]*3)
            print "Switching to left arm"
            self.driveArm = 'l'
        elif c.lower()=='r':
            motion.robot.left_ee.driveCommand([0]*3,[0]*3)
            print "Switching to right arm"
            self.driveArm = 'r'
        elif c==' ':
            self.driveVel = [0,0,0]
            self.driveAngVel = [0,0,0]
            self.updateDrive()
            self.left_limb.velocityCommand([0]*motion.numLimbDofs)
            self.right_limb.velocityCommand([0]*motion.numLimbDofs)
        elif c=='X':
            self.driveVel[0] += self.increment
            self.updateDrive()
        elif c=='x':
            self.driveVel[0] -= self.increment
            self.updateDrive()
        elif c=='Y':
            self.driveVel[1] += self.increment
            self.updateDrive()
        elif c=='y':
            self.driveVel[1] -= self.increment
            self.updateDrive()
        elif c=='Z':
            self.driveVel[2] += self.increment
            self.updateDrive()
        elif c=='z':
            self.driveVel[2] -= self.increment
            self.updateDrive()
        elif c=='!':
            self.driveAngVel[0] -= self.incrementang
            self.updateDrive()
        elif c=='1':
            self.driveAngVel[0] -= self.incrementang
            self.updateDrive()
        elif c=='@':
            self.driveAngVel[1] += self.incrementang
            self.updateDrive()
        elif c=='2':
            self.driveAngVel[1] -= self.incrementang
            self.updateDrive()
        elif c=='#':
            self.driveAngVel[2] += self.incrementang
            self.updateDrive()
        elif c=='3':
            self.driveAngVel[2] -= self.incrementang
            self.updateDrive()
        self.refresh()

    def extendTrace(self,trace,T):
        prev = trace[-1] if len(trace)==1 else trace[-2]
        if se3.distance(prev,T) > 1e-2 or len(trace)==1:
            trace.append(T)
        else:
            trace[-1] = T

    def idle(self):
        if motion.robot.left_mq.moving() or motion.robot.right_mq.moving():
            return
        dt = time.time()-self.lasttime
        #integrate trace
        trace = self.trace_int[self.driveArm]
        if len(trace)==0:
            self.trace_int['l'] = [motion.robot.left_ee.commandedTransform()]
            self.trace_int['r'] = [motion.robot.right_ee.commandedTransform()]
            self.trace_sensed['l'] = [motion.robot.left_ee.commandedTransform()]
            self.trace_sensed['r'] = [motion.robot.right_ee.commandedTransform()]
            trace = self.trace_int[self.driveArm]
        Tlast = trace[-1]
        tint = vectorops.madd(Tlast[1],self.driveVel,dt)
        Rint = so3.mul(so3.from_moment(vectorops.mul(self.driveAngVel,dt)),Tlast[0])
        self.extendTrace(self.trace_int[self.driveArm],(Rint,tint))
        #update sensed trace
        if self.driveArm=='l':
            T = motion.robot.left_ee.commandedTransform()
        else:
            T = motion.robot.right_ee.commandedTransform()
        self.extendTrace(self.trace_sensed[self.driveArm],T)
        self.refresh()

if __name__ == "__main__":
    print "drive_control.py: allows driving end effectors with the Motion module"
    print "Press l/r to switch to left/right arm"
    print "Press [space] to stop movement"
    print "Press X/x to increase/decrease x velocity"
    print "Press Y/y to increase/decrease y velocity"
    print "Press Z/z to increase/decrease z velocity"
    print "Press !/1 to increase/decrease x ang velocity"
    print "Press @/2 to increase/decrease y ang velocity"
    print "Press #/3 to increase/decrease z ang velocity"
    print "Press q to exit."
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
    
    viewer = MyEEDriveProgram(world)
    viewer.run()
