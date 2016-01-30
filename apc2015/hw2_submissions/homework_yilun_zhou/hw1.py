#!/usr/bin/python

from klampt import *
from klampt.glprogram import *
import os
import math

#The path of the klampt_models directory
model_dir = "../klampt_models/"


class MyGLViewer(GLRealtimeProgram):
    """This is a generic class to start up a simulation for a given world.
    You may put in whatever control loop you wish into the control_loop function
    to drive your robot(s).

    Pressing 's' toggles simulation, pressing 'm' toggles saving screenshots
    to disk.
    """
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"My GL program")
        self.world = world
        #Put your initialization code here
        #the current example creates a collision class, simulator, 
        #simulation flag, and screenshot flags
        self.collider = robotcollide.WorldCollider(world)
        self.sim = Simulator(world)
        self.simulate = False

        self.saveScreenshots = False
        self.nextScreenshotTime = 0
        self.screenshotCount = 0

    def display(self):
        #Put your display handler here
        #the current example draws the simulated world in grey and the
        #commanded configurations in transparent green
        self.sim.updateWorld()
        self.world.drawGL()

        #draw commanded configurations
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0,1,0,0.5])
        for i in xrange(self.world.numRobots()):
            r = self.world.robot(i)
            q = self.sim.getController(i).getCommandedConfig()
            r.setConfig(q)
            r.drawGL(False)
        glDisable(GL_BLEND)

    def control_loop(self):
        #Put your control handler here
        pass

    def idle(self):
        #Put your idle loop handler here
        #the current example simulates with the current time step self.dt
        if self.simulate and self.saveScreenshots:
            #The following line saves movies on simulation time
            if self.sim.getTime() >= self.nextScreenshotTime:
            #The following line saves movies on wall clock time
            #if self.ttotal >= self.nextScreenshotTime:
                self.save_screen("image%04d.ppm"%(self.screenshotCount,))
            self.screenshotCount += 1
            self.nextScreenshotTime += 1.0/30.0;

        if self.simulate:
            self.control_loop()
            self.sim.simulate(self.dt)
            glutPostRedisplay()

    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click
        print "mouse",button,state,x,y
        if button==2:
            if state==0:
                print [o.getName() for o in self.click_world(x,y)]
            return
        GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def specialfunc(self,c,x,y):
        #Put your keyboard special character handler here
        print c,"pressed"

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        print c,"pressed"
        if c == 's':
            self.simulate = not self.simulate
            print "Simulating:",self.simulate
        elif c == 'm':
            self.saveScreenshots = not self.saveScreenshots
            print "Movie mode:",self.saveScreenshots
        glutPostRedisplay()

    def click_world(self,x,y):
        """Helper: returns a list of world objects sorted in order of
        increasing distance."""
        #get the viewport ray
        (s,d) = self.click_ray(x,y)
        
        #run the collision tests
        collided = []
        for g in self.collider.geomList:
            (hit,pt) = g[1].rayCast(s,d)
            if hit:
                dist = vectorops.dot(vectorops.sub(pt,s),d)
                collided.append((dist,g[0]))
        return [g[1] for g in sorted(collided)]


def main():
    """The main loop that loads the models and starts the OpenGL visualizer"""
    
    world = WorldModel()
    #uncomment these lines and comment out the next 2 if you want to use the
    #full Baxter model
    #print "Loading full Baxter model (be patient, this will take a minute)..."
    #world.loadElement(os.path.join(model_dir,"baxter.rob"))
    print "Loading simplified Baxter model..."
    world.loadElement(os.path.join(model_dir,"baxter_col.rob"))
    print "Loading Kiva pod model..."
    world.loadElement(os.path.join(model_dir,"kiva_pod/model.obj"))
    print "Loading plane model..."
    world.loadElement(os.path.join(model_dir,"plane.env"))
    
    #shift the Baxter up a bit (95cm)
    Rbase,tbase = world.robot(0).getLink(0).getParentTransform()
    world.robot(0).getLink(0).setParentTransform(Rbase,(0,0,0.95))
    
    #translate pod to be in front of the robot, and rotate the pod by 90 degrees 
    Trel = (so3.rotation((0,0,1),math.pi/2),[1.1,0,0])
    T = world.rigidObject(0).getTransform()
    world.rigidObject(0).setTransform(*se3.mul(Trel,T))
    
    #run the visualizer
    visualizer = MyGLViewer(world)
    visualizer.run()

if __name__ == "__main__":
    main()
