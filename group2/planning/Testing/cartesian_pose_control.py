from klampt import *
from klampt import vectorops,so3,se3
from klampt.glprogram import *
from klampt import gldraw
import math
import time
import sys
import os
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Motion import motion
from Motion import config

class MyEEPoseProgram(GLRealtimeProgram):
    """A class for driving around end effectors using the keyboard.  Used
    primarily for demonstrating the end effector moveto function and for
    testing the Motion library.
    """
    def __init__(self,world,name="My GL Widget Program"):
        GLRealtimeProgram.__init__(self,name)
        self.world = world
        self.world.robot(0).setConfig(motion.robot.getKlamptCommandedPosition())
        self.drivePos = [0,0,0]
        self.driveRot = so3.identity()
        self.driveArm = 'l'
        self.useRotation = True
        self.localMotion = True
        self.increment = 0.05
        self.incrementang = 0.1
        self.trace_sensed = {'l':[],
                             'r':[]}

        self.baseCommand = [0,0,0]
        self.increment = 0.01
        self.incrementang = 0.02

        LEFT_CONFIG = [-1.145883647241211, -0.6657476611816406, 0.794602047216797, 1.0722525695068361, -0.5284563808227539, -1.5719468105895997, 2.829811055218506]
        RIGHT_CONFIG = None#[0.8624806970031739, -0.7213544646789551, -0.700262228869629, 0.4651796733947754, 0.585213669909668, 1.9167089922729494, -0.20555342534179688]


        print "Moving to neutral configuration"
        q = motion.robot.left_limb.commandedPosition()
        if(not LEFT_CONFIG is None ):
            print len(LEFT_CONFIG)
            if len(LEFT_CONFIG) < 8:
                for i in range(len(LEFT_CONFIG)):
                    q[i] = LEFT_CONFIG[i]
        else:
            q[1] = -1.0;
            q[3] = 2.0;
            q[5] = 1.0;
        motion.robot.left_mq.appendLinearRamp(q)
        q = motion.robot.right_limb.commandedPosition()
        if(not RIGHT_CONFIG is None) :
            print len(RIGHT_CONFIG)
            if len(RIGHT_CONFIG) < 8:
                for i in range(len(RIGHT_CONFIG)):
                    q[i] = RIGHT_CONFIG[i]
        else:
            q[1] = -1.0;
            q[3] = 2.0;
            q[5] = 1.0;
        motion.robot.right_mq.setRamp(q)
        motion.robot.left_ee.setOffset([0,0,0.1])
        motion.robot.right_ee.setOffset([0,0,0.1])
        self.state = 'move to start'

    def closeEvent(self,event):
        """Called when the "X" button is pressed"""
        motion.robot.shutdown()
        event.accept()

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

        #draw commanded pose
        gldraw.xform_widget((self.driveRot,self.drivePos),0.1,0.02,fancy=True)

        #draw trace
        glDisable(GL_LIGHTING)
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

    def specialfunc(self,c,x,y):
        #Put your keyboard special character handler here
        print c,"pressed"
        if c == GLUT_KEY_LEFT:
            self.baseCommand[1] += self.increment
        elif c == GLUT_KEY_RIGHT:
            self.baseCommand[1] -= self.increment
        elif c == GLUT_KEY_UP:
            self.baseCommand[0] += self.increment
        elif c == GLUT_KEY_DOWN:
            self.baseCommand[0] -= self.increment
        self.updateBaseCommand()
        self.refresh()

    def updateBaseCommand(self):
        motion.robot.base.moveVelocity(*self.baseCommand)
        self.refresh()

    def updateCommand(self):
        jointErr = 0.5 if self.localMotion else 0.0
        positionBand = 0.2
        rotationBand = 2
        if self.useRotation:
            print "Moving to rotation (quat):",so3.quaternion(self.driveRot),", position:",self.drivePos
            if self.driveArm=='l':
                if not motion.robot.left_ee.moveTo(self.driveRot,self.drivePos,maxPositionError=positionBand,maxRotationError=rotationBand,maxJointDeviation=jointErr):
                    self.driveRot,self.drivePos = motion.robot.left_ee.commandedTransform()
            elif self.driveArm=='r':
                if not motion.robot.right_ee.moveTo(self.driveRot,self.drivePos,maxPositionError=positionBand,maxRotationError=rotationBand,maxJointDeviation=jointErr):
                    self.driveRot,self.drivePos = motion.robot.right_ee.commandedTransform()
            else:
                raise ValueError("Invalid value for self.driveArm")
        else:
            print "Moving to position:",self.drivePos
            if self.driveArm=='l':
                R = motion.robot.left_ee.commandedTransform()[0]
                if not motion.robot.left_ee.moveTo(R,self.drivePos,maxPositionError=positionBand,maxJointDeviation=jointErr):
                    self.driveRot,self.drivePos = motion.robot.left_ee.commandedTransform()
            elif self.driveArm=='r':
                R = motion.robot.right_ee.commandedTransform()[0]
                if not motion.robot.right_ee.moveTo(R,self.drivePos,maxPositionError=positionBand,maxJointDeviation=jointErr):
                    self.driveRot,self.drivePos = motion.robot.right_ee.commandedTransform()
            else:
                raise ValueError("Invalid value for self.driveArm")

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        print c,"pressed"
        if c.lower()=='q':
            motion.robot.shutdown()
            self.close()
        elif c.lower()=='h':
            self.print_help()
        elif c.lower()=='o':
            self.useRotation = not self.useRotation
            print "Using rotation?",self.useRotation
        elif c.lower()=='o':
            self.localMotion = not self.localMotion
            print "Local motion?",self.localMotion
        elif c.lower()=='l':
            print "Switching to left arm"
            self.driveArm = 'l'
            self.driveRot,self.drivePos = motion.robot.left_ee.commandedTransform()
        elif c.lower()=='r':
            print "Switching to right arm"
            self.driveArm = 'r'
            self.driveRot,self.drivePos = motion.robot.right_ee.commandedTransform()
        elif c==' ':
            motion.robot.stopMotion()
        elif c=='X':
            self.drivePos[0] += self.increment
            self.updateCommand()
        elif c=='x':
            self.drivePos[0] -= self.increment
            self.updateCommand()
        elif c=='Y':
            self.drivePos[1] += self.increment
            self.updateCommand()
        elif c=='y':
            self.drivePos[1] -= self.increment
            self.updateCommand()
        elif c=='Z':
            self.drivePos[2] += self.increment
            self.updateCommand()
        elif c=='z':
            self.drivePos[2] -= self.increment
            self.updateCommand()
        elif c=='!':
            R = so3.rotation([1,0,0],self.increment)
            self.driveRot = so3.mul(R,self.driveRot)
            self.updateCommand()
        elif c=='1':
            R = so3.rotation([1,0,0],-self.increment)
            self.driveRot = so3.mul(R,self.driveRot)
            self.updateCommand()
        elif c=='@':
            R = so3.rotation([0,1,0],self.increment)
            self.driveRot = so3.mul(R,self.driveRot)
            self.updateCommand()
        elif c=='2':
            R = so3.rotation([0,1,0],-self.increment)
            self.driveRot = so3.mul(R,self.driveRot)
            self.updateCommand()
        elif c=='#':
            R = so3.rotation([0,0,1],self.increment)
            self.driveRot = so3.mul(R,self.driveRot)
            self.updateCommand()
        elif c=='3':
            R = so3.rotation([0,0,1],-self.increment)
            self.driveRot = so3.mul(R,self.driveRot)
            self.updateCommand()
        elif c==',' or c=='<':
            self.baseCommand[2] += self.incrementang
            self.updateBaseCommand()
        elif c=='.' or c=='>':
            self.baseCommand[2] -= self.incrementang
            self.updateBaseCommand()
        elif c=='p':
            #if( self.driveArm == 'r' ):
            #    print motion.robot.right_limb.sensedPosition()
            #elif (self.driveArm == 'l'):
            #    print motion.robot.left_limb.sensedPosition()
            print motion.robot.getKlamptSensedPosition()

        self.refresh()

    def extendTrace(self,trace,T):
        prev = trace[-1] if len(trace)==1 else trace[-2]
        if se3.distance(prev,T) > 1e-2 or len(trace)==1:
            trace.append(T)
        else:
            trace[-1] = T

    def idle(self):
        if self.state == 'move to start':
            if motion.robot.left_mq.moving() or motion.robot.right_mq.moving():
                return
            self.state = 'posing'
            self.driveRot,self.drivePos = motion.robot.left_ee.commandedTransform()
        dt = time.time()-self.lasttime
        #integrate trace
        trace = self.trace_sensed[self.driveArm]
        if len(trace)==0:
            self.trace_sensed['l'] = [motion.robot.left_ee.commandedTransform()]
            self.trace_sensed['r'] = [motion.robot.right_ee.commandedTransform()]
        #update sensed trace
        if self.driveArm=='l':
            T = motion.robot.left_ee.commandedTransform()
        else:
            T = motion.robot.right_ee.commandedTransform()
        self.extendTrace(self.trace_sensed[self.driveArm],T)
        self.refresh()
        
    def print_help(self):
        print "Press h to print this message"
        print "Press l/r to switch to left/right arm"
        print "Press o to toggle orientation control"
        print "Press [space] to stop movement"
        print "Press X/x to increase/decrease x position"
        print "Press Y/y to increase/decrease y position"
        print "Press Z/z to increase/decrease z position"
        print "Press !/1 to increase/decrease x orientation"
        print "Press @/2 to increase/decrease y orientation"
        print "Press #/3 to increase/decrease z orientation"
        print "Press arrow keys to control base velocity"
        print "Press </> to increase/decrease base rotation speed"
        print "Press p to print the current arm's configuration"
        print "Press q to exit."
        print


if __name__ == "__main__":
    config.parse_args()
    print "drive_control.py: allows driving end effectors with the Motion module"
    print
    print "Loading Motion Module model",config.klampt_model
    motion.setup(mode=config.mode,klampt_model=config.klampt_model,libpath="./",)
    res = motion.robot.startup()
    if not res:
        print "Error starting up Motion Module"
        exit(1)
    time.sleep(0.01)
    print "Started Motion Module"
    world = WorldModel()
    res = world.readFile(config.klampt_model)
    if not res:
        raise RuntimeError("Unable to load Klamp't model "+config.klampt_model)
  
    viewer = MyEEPoseProgram(world)
    viewer.print_help()
    viewer.run()
    motion.robot.shutdown()
