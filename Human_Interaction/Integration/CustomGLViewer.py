from motionController import LowLevelController
from motionController import PickingController

import sys, struct, time
from klampt import robotsim
from klampt.glprogram import *
import json
from Queue import Queue
import numpy as np


class CustomGLViewer(GLRealtimeProgram):
    def __init__(self,simworld=None,planworld=None, controller=None, sim=None, helper=None):

        if simworld == None and planworld == None:
            #Dummy GLViewer
            print 'simworld and planworld are None'
            return

        GLRealtimeProgram.__init__(self,"My GL program")

        self.simworld = simworld
        self.planworld = planworld
        self.sim = sim

        self.simulate = True

        
        if self.sim == None:
            self.simulate = False
        self.controller = controller
        self.command_queue = Queue()

        if helper != None:
            print 'helper is not none'
            helper.run()

        self.points = None

    def idle(self):
        if self.simulate:
            self.dt = 0.01
            self.sim.simulate(self.dt)
            glutPostRedisplay()

    def printStuff(self):
        print "shelf xform:", knowledge.shelf_xform
        print self.simworld.terrain(0).geometry()
        #print "terrain xform:", self.simworld.terrain(0).geometry().getTransform(), "\n\n"


    def updatePoints(self, points):
        self.points = points

    def glShowPointCloud(self, pc, downsample_rate=5, pt_size=None):
        # print glGetFloatv(GL_CURRENT_COLOR)
        pc = np.array(pc)
        # print 'Rendering %d points'%pc.shape[0]
        try:
            d = pc.shape[1]
            assert d==3 or d==4, 'Unrecognized point cloud shape: '+str(pc.shape)
            pc = pc[::downsample_rate, :]
            pc = pc.tolist()
            glDisable(GL_LIGHTING)
            glColor3f(0, 1, 0)
            if pt_size is not None:
                glPointSize(pt_size)
            glBegin(GL_POINTS)
            for p in pc:
                if d==4:
                    r, g, b = pcl_float_to_rgb(p[3])
                    r /= 255.0
                    g /= 255.0
                    b /= 255.0
                    glColor3f(r, g, b)
                glVertex3f(p[0], p[1], p[2])
            glEnd()
            glEnable(GL_LIGHTING)
            glColor3f(1,0,0)
        except:
            #print 'Insufficient data'
            pass



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

        if self.points != None:
            self.glShowPointCloud(self.points)    

        self.draw_wire_box([.5, .2, .8],[1.7, .6, 2])
        self.draw_wire_box([.5, -.2,.8],[1.7, .2, 2])
        self.draw_wire_box([.5, -.6, .8],[1.7, -.2, 2])


    def draw_wire_box(self, bmin,bmax):
        """Helper: draws a wireframe box"""
        glBegin(GL_LINE_LOOP)
        glVertex3f(bmin[0],bmin[1],bmin[2])
        glVertex3f(bmin[0],bmin[1],bmax[2])
        glVertex3f(bmin[0],bmax[1],bmax[2])
        glVertex3f(bmin[0],bmax[1],bmin[2])
        glEnd()
        glBegin(GL_LINE_LOOP)
        glVertex3f(bmax[0],bmin[1],bmin[2])
        glVertex3f(bmax[0],bmin[1],bmax[2])
        glVertex3f(bmax[0],bmax[1],bmax[2])
        glVertex3f(bmax[0],bmax[1],bmin[2])
        glEnd()
        glBegin(GL_LINES)
        glVertex3f(bmin[0],bmin[1],bmin[2])
        glVertex3f(bmax[0],bmin[1],bmin[2])
        glVertex3f(bmin[0],bmin[1],bmax[2])
        glVertex3f(bmax[0],bmin[1],bmax[2])
        glVertex3f(bmin[0],bmax[1],bmax[2])
        glVertex3f(bmax[0],bmax[1],bmax[2])
        glVertex3f(bmin[0],bmax[1],bmin[2])
        glVertex3f(bmax[0],bmax[1],bmin[2])
        glEnd()

    def keyboardfunc(self,c,x,y):
        #c = c.lower()
        if c=='z':
            self.simulate = not self.simulate
            print "Simulating:",self.simulate
        else:
            self.command_queue.put(c)
            #print int(c)
            if c  == chr(27):
                #c == esc
                #self.picking_thread.join()
                exit(0)

        glutPostRedisplay()


