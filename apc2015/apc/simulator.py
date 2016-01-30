import perception_fake as perception
import picking
import motion
import apc
import baxter
from threading import Thread,Lock
from Queue import Queue
from klampt import *
from klampt import vectorops,se3,so3,gldraw
from klampt.glprogram import *
import math
import os

#Turns off collision checking in simulation
NO_SIMULATION_COLLISIONS = 0
#Turn this on to help fast prototyping of later stages
FAKE_SIMULATION = 0
#Turn this on to log commands to file
LOG_COMMANDS = 1
command_log_file = 'commands.txt'

#The path of the klampt_models directory
model_dir = "../klampt_models/"


def init_ground_truth(world):
    loaded_item_infos = {}
    items = []
    for i in xrange(world.numRigidObjects()):
        n = world.rigidObject(i).getName()
        name,binname = n.split(',')
        if name not in loaded_item_infos:
            loaded_item_infos[name] = apc.load_item(n)
            load_item_infos[name].geometry = world.rigidObject(i).geometry()
        item = ItemInBin(loaded_item_infos[n],binname)
        item.xform = world.rigidObject(i).getTransform()
        items.append(item)
    ground_truth_shelf_xform = ([6.123233995736766e-17, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, -6.123233995736766e-17, 0.0], [1.4, 0.0, 0.01])
    perception.set_ground_truth(ground_truth_shelf_xform,items)

def update_ground_truth(world):
    for i in xrange(world.numRigidObjects()):
        T = world.rigidObject(i).getTransform()
        perception._ground_truth_items[i].xform = T
    return


def draw_xformed(xform,localDrawFunc):
    """Draws something given a se3 transformation and a drawing function
    that draws the object in its local frame.

    E.g., draw_xformed(xform,lambda:gldraw.box([ax,ay,az],[bx,by,bz])) draws
    a box oriented and translated by xform."""
    mat = zip(*se3.homogeneous(xform))
    mat = sum([list(coli) for coli in mat],[])

    glPushMatrix()
    glMultMatrixf(mat)
    localDrawFunc()
    glPopMatrix()

def draw_oriented_box(xform,bmin,bmax):
    """Helper: draws an oriented box"""
    draw_xformed(xform,lambda:gldraw.box(bmin,bmax))

def draw_wire_box(bmin,bmax):
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

def draw_oriented_wire_box(xform,bmin,bmax):
    """Helper: draws an oriented wireframe box"""
    draw_xformed(xform,lambda:draw_wire_box(bmin,bmax))


def run_controller(controller,command_queue):
    perception.run_perception_on_shelf(controller.knowledge)
    while True:
        c = command_queue.get()
        if c != None:
            print "Running command",c
            if c >= 'a' and c <= 'l':
                controller.viewBinAction('bin_'+c.upper())
            elif c == 'x':
                controller.graspAction()
            elif c == 'u':
                controller.ungraspAction()
            elif c == 'p':
                controller.placeInOrderBinAction()
            elif c == 'o':
                controller.fulfillOrderAction(['med_item','small_item'])
            elif c=='q':
                break
        else:
            print "Waiting for command..."
            time.sleep(0.1)
    print "Done"
    

class MyGLViewer(GLRealtimeProgram):
    """This class is used to simulate / interact with with the world model
    in hw4.
    
    Pressing 'a-l' runs the view_bin method which should set the robot to a
    configuration that places a hand camera such that it points inside the
    bin.

    Pressing 's' should pause / unpause the simulation.

    Pressing 'x' should "grasp" an object in the currently pointed-to-bin
    with either one of the hands at the designated grasp point.

    Pressing 'u' should "ungrasp" an object currently grasped inside a bin.

    Pressing 'p' should "put down" an object in the order bin
    """
    def __init__(self,simworld,planworld):
        GLRealtimeProgram.__init__(self,"My GL program")
        self.simworld = simworld
        self.planworld = planworld
        self.sim = Simulator(simworld)
        self.simulate = True
        #self.sim.simulate(0)
        
        #you can set these to true to draw the bins, grasps, and/or gripper/camera frames
        self.draw_bins = False
        self.draw_grasps = True
        self.draw_gripper_and_camera = True

        #initialize controllers
        if FAKE_SIMULATION:
            self.low_level_controller = motion.FakeLowLevelController(simworld.robot(0))
        else:
            self.low_level_controller = motion.SimLowLevelController(simworld.robot(0),self.sim.getController(0))
        if LOG_COMMANDS:
            self.low_level_controller = motion.LoggingController(self.low_level_controller,command_log_file)
        self.command_queue = Queue()
        self.picking_controller = picking.PickingController(planworld,self.low_level_controller)
        self.picking_thread = Thread(target=run_controller,args=(self.picking_controller,self.command_queue))
        self.picking_thread.start()

    def idle(self):
        if self.simulate:
            self.sim.simulate(self.dt)
            self.sim.updateWorld()
            update_ground_truth(self.simworld)
            glutPostRedisplay()

    def display(self):
        #you may run auxiliary openGL calls, if you wish to visually debug

        #draw the world
        self.sim.updateWorld()
        self.simworld.drawGL()

        #if you're doing question 1, this will draw the shelf and floor
        if self.simworld.numTerrains()==0:
            for i in range(self.planworld.numTerrains()):
                self.planworld.terrain(i).drawGL()

        #draw commanded configurations
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0,1,0,0.5])
        for i in xrange(self.simworld.numRobots()):
            r = self.simworld.robot(i)
            #q = self.sim.getController(i).getCommandedConfig()
            q = self.low_level_controller.getCommandedConfig()
            r.setConfig(q)
            r.drawGL(False)
        glDisable(GL_BLEND)
               
        #show bin boxes
        if self.draw_bins:
            glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,1,0,1])
            for b in apc.bin_bounds.values():
                draw_oriented_box(self.picking_controller.knowledge.shelf_xform,b[0],b[1])
            for b in apc.bin_names:
                c = self.picking_controller.knowledge.bin_front_center(b)
                if c:
                    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,1,0.5,1])                    
                    r = 0.01
                    gldraw.box([c[0]-r,c[1]-r,c[2]-r],[c[0]+r,c[1]+r,c[2]+r])
                c = self.picking_controller.knowledge.bin_vantage_point(b)
                if c:
                    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0.5,1,0.5,1])
                    r = 0.01
                    gldraw.box([c[0]-r,c[1]-r,c[2]-r],[c[0]+r,c[1]+r,c[2]+r])

        
        #show object state
        for bin_name,contents in self.picking_controller.knowledge.bin_contents.iteritems():
            if contents == None: continue
            for i in contents:
                #draw in wireframe
                glDisable(GL_LIGHTING)
                glColor3f(1,0.5,0)
                draw_oriented_wire_box(i.xform,i.info.bmin,i.info.bmax)
                glEnable(GL_LIGHTING)
                if self.draw_grasps:
                    #draw grasps, if available
                    g = self.picking_controller.knowledge.grasp_xforms(i)
                    if g:
                        for grasp,xform in g:
                            gldraw.xform_widget(xform,0.05,0.005,fancy=False)

        obj,limb,grasp = self.picking_controller.held_object,self.picking_controller.active_limb,self.picking_controller.active_grasp
        if obj != None:
            if limb == 'left':
                gripper_xform = self.simworld.robot(0).getLink(left_gripper_link_name).getTransform()
            else:
                gripper_xform = self.simworld.robot(0).getLink(right_gripper_link_name).getTransform()
            objxform = se3.mul(gripper_xform,se3.mul(baxter.left_gripper_center_xform,se3.inv(grasp.grasp_xform)))
            glDisable(GL_LIGHTING)
            glColor3f(1,0.5,0)
            draw_oriented_wire_box(objxform,obj.info.bmin,obj.info.bmax)
            glEnable(GL_LIGHTING)

        #show gripper and camera frames
        if self.draw_gripper_and_camera:
            left_camera_link = self.simworld.robot(0).getLink(baxter.left_camera_link_name)
            right_camera_link = self.simworld.robot(0).getLink(baxter.right_camera_link_name)
            left_gripper_link = self.simworld.robot(0).getLink(baxter.left_gripper_link_name)
            right_gripper_link = self.simworld.robot(0).getLink(baxter.right_gripper_link_name)
            gldraw.xform_widget(left_camera_link.getTransform(),0.1,0.01,fancy=False)
            gldraw.xform_widget(right_camera_link.getTransform(),0.1,0.01,fancy=False)
            gldraw.xform_widget(se3.mul(left_gripper_link.getTransform(),baxter.left_gripper_center_xform),0.05,0.005,fancy=False)
            gldraw.xform_widget(se3.mul(right_gripper_link.getTransform(),baxter.right_gripper_center_xform),0.05,0.005,fancy=False)

        #draw order box 
        glDisable(GL_LIGHTING)
        glColor3f(1,0,0)
        draw_oriented_wire_box(apc.order_bin_xform,apc.order_bin_bounds[0],apc.order_bin_bounds[1])
        glEnable(GL_LIGHTING)
        return

    def keyboardfunc(self,c,x,y):
        c = c.lower()
        if c=='s':
            self.simulate = not self.simulate
            print "Simulating:",self.simulate
        else:
            self.command_queue.put(c)
            if c=='q':
                self.picking_thread.join()
                exit(0)
        glutPostRedisplay()


def main():
    """The main loop that loads the planning / simulation models and
    starts the OpenGL visualizer."""
    world = WorldModel()
    world.readFile(model_dir+"apc.xml")

    simworld = WorldModel()
    simworld.readFile(model_dir+"apc_with_test_objects.xml")
    init_ground_truth(simworld)

    if NO_SIMULATION_COLLISIONS:
        #discard all that
        simworld = World()
        simworld.readFile(model_dir+"apc_baxter_only.xml")

    #load the resting configuration from klampt_models/baxter_X_rest.config
    baxter.load_rest_config(model_dir)
    simworld.robot(0).setConfig(baxter.rest_config)

    
    #run the visualizer
    visualizer = MyGLViewer(simworld,world)
    visualizer.run()

if __name__ == "__main__":
    main()
