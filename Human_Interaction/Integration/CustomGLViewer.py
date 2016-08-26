from motionController import LowLevelController
from motionController import PickingController

import sys, struct, time
from klampt import *
from klampt import robotsim
from klampt.glprogram import *
from klampt import WidgetSet, PointPoser, RobotPoser
import json
from Queue import Queue
import numpy as np

BOX_COORDS = {}
BOX_COORDS[2] = ([.5, .2, .95],[1.44, .6, 1.2])
BOX_COORDS[1] = ([.5, -.2,.95],[1.44, .2, 1.2])
BOX_COORDS[0] = ([.5, -.6, .95],[1.44, -.2, 1.2])


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

        self.widgetMaster = WidgetSet()
        self.widgetButton = 2  #right-clicks
        self.draggingWidget = False

        self.poseWidget = PointPoser()
        self.widgetMaster.add(self.poseWidget)
        self.robotWidget = RobotPoser(self.simworld.robot(0))
        self.widgetMaster.add(self.robotWidget)

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
        self.widgetMaster.idle()
        if self.widgetMaster.wantsRedraw():
            print 'widgetMaster wants redraw'
            self.refresh()

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


    def updateBox(self, index, vertices):
        BOX_COORDS[index] = vertices


    def display(self):
        
        global BOX_COORDS

        glEnable(GL_BLEND)

        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,0,0,0.5])
        #only 1 robot in this case, but still use for-loop for generality
        for i in xrange(self.simworld.numRobots()):
            r = self.controller.robotModel
            q = self.controller.getCommandedConfig()
            if self.widgetMaster.wantsRedraw():
                #restart sim - start it with the new robot config
                # if simulating
                if self.sim != None:
                    q = self.robotWidget.get()
                    print 'self.sim is not None'
                    self.controller.robotModel.setConfig(q)
                    self.simworld.robot(0).setConfig(q)
                    self.controller.setLinear(q, .01)

            #q = self.controller.getSensedConfig()
            r.setConfig(q)
            r.drawGL(False)
        
        glDisable(GL_BLEND)

        for i in xrange(self.simworld.numTerrains()):
            self.simworld.terrain(i).drawGL()

        if self.points != None:
            self.glShowPointCloud(self.points)    

        for i in BOX_COORDS:
            self.draw_wire_box(*BOX_COORDS[i])

        self.widgetMaster.drawGL(self.viewport())

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
        
        print c,"pressed"
        
        if c == ' ':
            config = self.robotWidget.get()
            print "Config:",config
            self.controller.robotModel.setConfig(config)
            self.simworld.robot(0).setConfig(config)
        else:
            self.widgetMaster.keypress(c)
            self.refresh()
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


    def mousefunc(self,button,state,x,y):
        print "mouse",button,state,x,y
        if button==self.widgetButton:
            if state==0:
                print 'state was zero'
                if self.widgetMaster.beginDrag(x,self.height-y,self.viewport()):
                    print 'beginning drag'
                    self.draggingWidget = True
            else:
                print 'state of button not zero'
                if self.draggingWidget:
                    print 'dragging widget true - ending drag'
                    self.widgetMaster.endDrag()
                    self.draggingWidget = False
            if self.widgetMaster.wantsRedraw():
                self.refresh()
            self.lastx,self.lasty = x,y
            return
        GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y,dx,dy):
        if self.draggingWidget:
            self.widgetMaster.drag(dx,-dy,self.viewport())
            if self.widgetMaster.wantsRedraw():
                self.refresh()
        else:
            res = self.widgetMaster.hover(x,self.height-y,self.viewport())
            if self.widgetMaster.wantsRedraw():
                self.refresh()
            GLRealtimeProgram.motionfunc(self,x,y,dx,dy)

    def specialfunc(self,c,x,y):
        #Put your keyboard special character handler here
        print c,"pressed"
        self.widgetMaster.keypress(c)
        self.refresh()

