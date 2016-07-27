from motionController import LowLevelController
from motionController import PickingController

import sys, struct, time
from klampt import robotsim
from klampt.glprogram import *
import json
from Queue import Queue



class CustomGLViewer(GLRealtimeProgram):
    def __init__(self,simworld,planworld, controller, sim=None):
        GLRealtimeProgram.__init__(self,"My GL program")

        self.simworld = simworld
        self.planworld = planworld
        self.sim = sim

        self.simulate = True

        
        if self.sim == None:
            self.simulate = False
        self.controller = controller
        self.command_queue = Queue()


    def idle(self):
        if self.simulate:
            self.dt = 0.01
            self.sim.simulate(self.dt)
            glutPostRedisplay()

    def printStuff(self):
        print "shelf xform:", knowledge.shelf_xform
        print self.simworld.terrain(0).geometry()
        #print "terrain xform:", self.simworld.terrain(0).geometry().getTransform(), "\n\n"


    def display(self):

        
        glEnable(GL_BLEND)

        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,0,0,0.5])
        #only 1 robot in this case, but still use for-loop for generality
        for i in xrange(self.simworld.numRobots()):
            r = self.controller.robotModel
            q = self.controller.getCommandedConfig()
            r.setConfig(q)
            r.drawGL(False)
        
        glDisable(GL_BLEND)

        for i in xrange(self.simworld.numTerrains()):
            self.simworld.terrain(i).drawGL()

    def keyboardfunc(self,c,x,y):
        #c = c.lower()
        if c=='z':
            self.simulate = not self.simulate
            print "Simulating:",self.simulate
        else:
            self.command_queue.put(c)

            print int(c)

            if c  == chr(27):
                #c == esc
                #self.picking_thread.join()
                exit(0)

        glutPostRedisplay()