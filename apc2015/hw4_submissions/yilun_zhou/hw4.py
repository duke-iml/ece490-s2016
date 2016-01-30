#!/usr/bin/python

from klampt import *
from klampt.glprogram import *
from klampt import vectorops,so3,se3,gldraw,ik,loader
from klampt.robotsim import Geometry3D
from baxter import *
from hw4_planner import *
import apc
import os
import math
import random
from threading import Thread,Lock
from Queue import Queue
from custom_util import print_instance_var

#The path of the klampt_models directory
model_dir = "../klampt_models/"

#resting configuration
baxter_rest_config = [0.0]*60

#the transformation of the order bin
order_bin_xform = (so3.identity(),[0.5,0,0])
#the local bounding box of the order bin
order_bin_bounds = ([-0.2,-0.4,0],[0.2,0.4,0.7])

global_planner = None
global_robot = None

class KnowledgeBase:
    """A structure containing the robot's dynamic knowledge about the world.
    Members:
    - bin_contents: a map from bin names to lists of known items in
      the bin.  Items are given by apc.ItemInBin objects.
    - order_bin_contents: the list of objects already in the order bin.
      also given by apc.ItemInBin objects
    - shelf_xform: the transformation (rotation, translation) of the bottom
      center of the shelf in world coordinates.  The x coordinate increases
      from left to right, the y coordinate increases from bottom to top,
      and the z coordinate increases from back to front.
      this will be loaded dynamically either from perception or hard coded.

    (in this homework assignment we will use the fake perception module
    to populate the bin contents, and assume the shelf xform is
    estimated perfectly.)
    """
    def __init__(self):
        self.bin_contents = dict((n,None) for n in apc.bin_names)
        self.order_bin_contents = []
        self.shelf_xform = se3.identity()
    
    def bin_front_center(self,bin_name):
        bmin,bmax = apc.bin_bounds[bin_name]
        local_center = [(bmin[0]+bmax[0])*0.5,(bmin[1]+bmax[1])*0.5,bmax[2]]
        world_center = se3.apply(self.shelf_xform,local_center)
        return world_center


    def bin_vantage_point(self,bin_name):
        world_center = self.bin_front_center(bin_name)
        #20cm offset
        world_offset = so3.apply(self.shelf_xform[0],[0,0,0.2])
        return vectorops.add(world_center,world_offset)


    def grasp_xforms(self,object):
        if object.xform == None: return None
        res = []
        for g in object.info.grasps:
            grasp_xform_world = se3.mul(object.xform,g.grasp_xform)
            res.append((g,grasp_xform_world))
        return res
        

#a list of actual items -- this is only used for the fake perception module, and your
#code should not use these items directly
ground_truth_items = []
ground_truth_shelf_xform = se3.identity()
def init_ground_truth():
    global ground_truth_items
    ground_truth_items = [apc.ItemInBin(apc.tall_item,'bin_B'),
                          apc.ItemInBin(apc.small_item,'bin_D'),
                          apc.ItemInBin(apc.med_item,'bin_H')]
    ground_truth_items[0].set_in_bin_xform(ground_truth_shelf_xform,0.25,0.2,0.0)
    ground_truth_items[1].set_in_bin_xform(ground_truth_shelf_xform,0.5,0.1,math.pi/4)
    ground_truth_items[2].set_in_bin_xform(ground_truth_shelf_xform,0.6,0.4,math.pi/2)

def run_perception_on_shelf(knowledge):
    """This is a fake perception module that simply reveals the shelf
    xform."""
    knowledge.shelf_xform = ground_truth_shelf_xform

def run_perception_on_bin(knowledge,bin_name):
    """This is a fake perception module that simply reveals all the items
    the given bin."""
    global ground_truth_items
    if knowledge.bin_contents[bin_name]==None:
        #not sensed yet
        knowledge.bin_contents[bin_name] = []
        for item in ground_truth_items:
            if item.bin_name == bin_name:
                #place it in the bin
                knowledge.bin_contents[bin_name].append(item)
    return


class LowLevelController:
    """A low-level interface to the Baxter robot (with parallel jaw
    grippers).  Does appropriate locking for multi-threaded use.
    You should use this in your picking controller."""
    def __init__(self,robotModel,robotController):
        self.robotModel = robotModel
        self.controller = robotController
        self.lock = Lock()
    def getSensedConfig(self):
        self.lock.acquire()
        res = self.controller.getSensedConfig()
        self.lock.release()
        return res
    def getSensedVelocity(self):
        self.lock.acquire()
        res = self.controller.getSensedVelocity()
        self.lock.release()
        return res
    def getCommandedConfig(self):
        self.lock.acquire()
        res = self.controller.getCommandedConfig()
        self.lock.release()
        return res
    def getCommandedVelocity(self):
        self.lock.acquire()
        res = self.controller.getCommandedVelocity()
        self.lock.release()
        return res
    def setPIDCommand(self,configuration,velocity):
        """Sets the controller to a PID command mode"""
        self.lock.acquire()
        self.controller.setPIDCommand(configuration,velocity)
        self.lock.release()
    def setMilestone(self,destination,endvelocity=None):
        """Immediately sets the motion queue to move to the given
        milestone.  If endvelocity is given, then the end of the
        queue will be moving at that velocity.  Otherwise, the end
        velocity will be zero."""
        self.lock.acquire()
        if endvelocity == None: self.controller.setMilestone(destination)
        else: self.controller.setMilestone(destination,endvelocity)
        self.lock.release()
    def appendMilestone(self,destination,endvelocity=None):
        """Appends a milestone to the motion queue.  If endvelocity
        is given, then the end of the queue will be moving at that velocity.
        Otherwise, the end velocity will be zero."""
        self.lock.acquire()
        if endvelocity == None: self.controller.appendMilestone(destination)
        else: self.controller.appendMilestone(destination,endvelocity)
        self.lock.release()
    def isMoving(self):
        return self.controller.remainingTime()>0
    def remainingTime(self):
        return self.controller.remainingTime()
    def commandGripper(self,limb,command):
        """Sends the command to the indicated gripper.
        For the parallel-jaw gripper, [0] is closed, [1] is open
        Warning: don't do this while moving"""
        self.lock.acquire()
        q = self.controller.getCommandedConfig()
        self.robotModel.setConfig(q)
        value = command[0]
        if limb=='left':
            print "Opening left gripper to",value
            self.robotModel.getDriver(15).setValue(value*0.03)
            self.robotModel.getDriver(16).setValue(-value*0.03)
        else:
            print "Opening right gripper to",value
            self.robotModel.getDriver(17).setValue(value*0.03)
            self.robotModel.getDriver(18).setValue(-value*0.03)
        self.controller.setMilestone(self.robotModel.getConfig())
        self.lock.release()


class PickingController:
    """Maintains the robot's knowledge base and internal state.  Most of
    your code will go here.  Members include:
    - knowledge: a KnowledgeBase object
    - planner: an LimbPlanner object, which *you will implement and use*
    - state: either 'ready', or 'holding'
    - configuration: the robot's current configuration
    - active_limb: the limb currently active, either holding or viewing a state
    - current_bin: the name of the bin where the camera is viewing or the gripper is located
    - held_object: the held object, if one is held, or None otherwise

    External modules can call viewBinAction(), graspAction(), ungraspAction(),
    and placeInOrderBinAction()
    """
    def __init__(self,world,robotController):
        self.world = world
        self.robot = world.robot(0)
        self.controller = robotController
        self.knowledge = KnowledgeBase()
        self.planner = LimbPlanner(self.world,self.knowledge)
        global global_planner
        global_planner = self.planner
        self.state = 'ready'
        self.active_limb = None
        self.active_grasp = None
        self.current_bin = None
        self.held_object = None
        #these may be helpful
        self.left_camera_link = self.robot.getLink(left_camera_link_name)
        self.right_camera_link = self.robot.getLink(right_camera_link_name)
        self.left_gripper_link = self.robot.getLink(left_gripper_link_name)
        self.right_gripper_link = self.robot.getLink(right_gripper_link_name)
        self.left_arm_links = [self.robot.getLink(i) for i in left_arm_link_names]
        self.right_arm_links = [self.robot.getLink(i) for i in right_arm_link_names]
        id_to_index = dict([(self.robot.getLink(i).getID(),i) for i in range(self.robot.numLinks())])
        self.left_arm_indices = [id_to_index[i.getID()] for i in self.left_arm_links]
        self.right_arm_indices = [id_to_index[i.getID()] for i in self.right_arm_links]

    def waitForMove(self,timeout = None, pollRate = 0.5):
        """Waits for the move to complete, or timeout seconds is elapsed,
        before terminating."""
        iters = 0
        t = 0
        while self.controller.isMoving():
            if iters % 10 == 0:
                print "Waiting for move to complete..."
            time.sleep(pollRate)
            t += pollRate
            if timeout != None and t > timeout:
                return False
            iters += 1
        return True

    def viewBinAction(self,b):
        self.waitForMove()
            
        if self.state != 'ready':
            print "Already holding an object, can't move to bin"
            return False
        else:
            if b in apc.bin_names:
                if self.move_camera_to_bin(b):
                    self.waitForMove()
                    self.current_bin = b
                    run_perception_on_bin(self.knowledge,b)
                    print "Sensed bin",b,"with camera",self.active_limb
                else:
                    print "Move to bin",b,"failed"
                    return False
            else:
                print "Invalid bin",b
                return False
        return True
        
    def graspAction(self):
        self.waitForMove()
        self.controller.commandGripper(self.active_limb,[1])
        self.waitForMove()
            
        if self.current_bin == None:
            print "Not located at a bin"
            return False
        elif self.state != 'ready':
            print "Already holding an object, can't grasp another"
            return False
        elif len(self.knowledge.bin_contents[self.current_bin])==0:
            print "The current bin is empty"
            return False
        else:
            if self.move_to_grasp_object(self.knowledge.bin_contents[self.current_bin][0]):
                self.waitForMove()
                #now close the gripper
                self.controller.commandGripper(self.active_limb,self.active_grasp.gripper_close_command)
                self.waitForMove()
                
                self.held_object = self.knowledge.bin_contents[self.current_bin].pop(0)
                self.state = 'holding'
                print "Holding object",self.held_object.info.name,"in hand",self.active_limb
                return True
            else:
                print "Grasp failed"
                return False

    def ungraspAction(self):
        self.waitForMove()
            
        if self.state != 'holding':
            print "Not holding an object"
            return False
        else:
            if self.move_to_ungrasp_object(self.held_object):
                self.waitForMove()
                #now open the gripper
                self.controller.commandGripper(self.active_limb,self.active_grasp.gripper_open_command)
                self.waitForMove()
                
                print "Object",self.held_object.info.name,"placed back in bin"
                self.knowledge.bin_contents[self.current_bin].append(self.held_object)
                self.state = 'ready'
                self.held_object = None
                return True
            else:
                print "Ungrasp failed"
                return False
        
    def placeInOrderBinAction(self):
        self.waitForMove()

        if self.state != 'holding':
            print "Not holding an object"
        else:
            if self.move_to_order_bin(self.held_object):
                self.waitForMove()                
                #now open the gripper
                self.controller.commandGripper(self.active_limb,self.active_grasp.gripper_open_command)
                self.waitForMove()
                print "Successfully placed",self.held_object.info.name,"into order bin"
                self.knowledge.order_bin_contents.append(self.held_object)
                self.held_object.xform = None
                self.held_object.bin_name = 'order_bin'
                self.state = 'ready'
                self.held_object = None
                return True
            else:
                print "Move to order bin failed"
                return False

    def fulfillOrderAction(self,objectList):
        """Given a list of objects to be put in the order bin, run
        until completed."""
        remainingObjects = objectList
        for b in apc.bin_names:
            if self.knowledge.bin_contents[b]==None:
                if not self.viewBinAction(b):
                    print "Could not view bin",b
                    continue

            donextbin = False
            while any(o.info.name in remainingObjects for o in self.knowledge.bin_contents[b]) and not donextbin:
                #pick up and put down objects until you are holding one that is in the remainingObjects list
                if not self.graspAction():
                    print "Error grasping object"
                    donextbin = True
                    break
                while not donextbin and (self.held_object == None or self.held_object.info.name not in remainingObjects):
                    #cycle through objects by putting down and picking up the next object
                    if not self.ungraspAction():
                        print "Error putting down object"
                        return False
                    if not self.graspAction():
                        print "Error grasping object"
                        donextbin = True
                        break
                obj = self.held_object
                if self.placeInOrderBinAction():
                    remainingObjects.remove(obj.info.name)
                else:
                    print "Error putting object into order bin"
                    return False
            if len(remainingObjects)==0:
                return True
        print "These items are remaining from the order:",remainingObjects
        return False

    def randomize_limb_position(self,limb,range=None):
        """Helper: randomizes the limb configuration in self.robot.
        limb can be 'left' or 'right'.  If range is provided, then
        this samples in a range around the current commanded config"""
        qmin,qmax = self.robot.getJointLimits()
        if range == None:
            q = baxter_rest_config[:]
            if limb == 'left':
                for j in self.left_arm_indices:
                    q[j] = random.uniform(qmin[j],qmax[j])
            else:
                for j in self.right_arm_indices:
                    q[j] = random.uniform(qmin[j],qmax[j])
            self.robot.setConfig(q)
        else:
            q = self.controller.getCommandedConfig()
            if limb == 'left':
                for j in self.left_arm_indices:
                    q[j] = max(qmin[j],min(qmax[j],random.uniform(q[j]-range,q[j]+range)))
            else:
                for j in self.right_arm_indices:
                    q[j] = max(qmin[j],min(qmax[j],random.uniform(q[j]-range,q[j]+range)))
            self.robot.setConfig(q)
        return

    def move_camera_to_bin(self,bin_name):
        """Starts a motion so the camera has a viewpoint that
        observes bin_name.  Will also change self.active_limb to the
        appropriate limb.

        If successful, sends the motion to the low-level controller and
        returns True.
        
        Otherwise, does not modify the low-level controller and returns False.
        """
        world_offset = self.knowledge.bin_vantage_point(bin_name)
        
        #place +z in the +x axis, y in the +z axis, and x in the -y axis
        left_goal = ik.objective(self.left_camera_link,R=[0,0,-1,1,0,0,0,1,0],t=world_offset)
        right_goal = ik.objective(self.right_camera_link,R=[0,0,-1,1,0,0,0,1,0],t=world_offset)
        qcmd = self.controller.getCommandedConfig()
        old_config = [c for c in qcmd]
        for i in range(100):
            if random.random() < 0.5:
                if i == 0:
                    self.robot.setConfig(qcmd)
                else:
                    self.randomize_limb_position('left')
                result = ik.solve(left_goal)
                if result:
                    # TODO: plan a path, DONE
                    # change print_detail to True if you want to know what parts are colliding
                    if self.planner.is_in_self_collision(print_detail=False):
                        #print "collided"
                        continue
                    #try:
                    #    self.planner.plan(old_config, self.robot.getConfig())
                    #except:
                    #    continue
                    #print "Left planning done"
                    self.controller.setMilestone(self.robot.getConfig())
                    self.active_limb = 'left'
                    return True
            else:
                if i == 0:
                    self.robot.setConfig(qcmd)
                else:
                    self.randomize_limb_position('right')
                result = ik.solve(right_goal)
                if result:
                    # TODO: plan a path, DONE
                    # change print_detail to True if you want to know what parts are colliding
                    if self.planner.is_in_self_collision(print_detail=False):
                        #print "collided"
                        continue
                    #try:
                    #    self.planner.plan(old_config, self.robot.getConfig())
                    #except:
                    #    print "Right Failed"
                    #    continue
                    #print "Right planning done"
                    self.controller.setMilestone(self.robot.getConfig())
                    self.active_limb = 'right'
                    return True
        return False

    def move_to_grasp_object(self,object):
        """Sets the robot's configuration so the gripper grasps object at
        one of its potential grasp locations.  Might change self.active_limb
        to the appropriate limb.  Must change self.active_grasp to the
        selected grasp.

        If successful, sends the motion to the low-level controller and
        returns True.
        
        Otherwise, does not modify the low-level controller and returns False.
        """
        grasps = self.knowledge.grasp_xforms(object)
        qmin,qmax = self.robot.getJointLimits()
        qcmd = self.controller.getCommandedConfig()
        #phase 1: init IK from the commanded config, search among grasps
        for (grasp,gxform) in grasps:
            if self.active_limb == 'left':
                Tg = se3.mul(gxform,se3.inv(left_gripper_center_xform))
                goal = ik.objective(self.left_gripper_link,R=Tg[0],t=Tg[1])
            else:
                Tg = se3.mul(gxform,se3.inv(right_gripper_center_xform))
                goal = ik.objective(self.right_gripper_link,R=Tg[0],t=Tg[1])
            self.robot.setConfig(qcmd)
            if ik.solve(goal):
                if self.planner.is_in_self_collision(print_detail=False):
                    #print "collided"
                    continue
                self.controller.setMilestone(self.robot.getConfig())
                self.active_grasp = grasp
                return True
        #Phase 2: that didn't work, now try random sampling
        for i in range(100):
            #pick a config at random
            self.randomize_limb_position(self.active_limb)
            #pick a grasp at random
            (grasp,gxform) = random.choice(grasps)
            if self.active_limb == 'left':
                Tg = se3.mul(gxform,se3.inv(left_gripper_center_xform))
                goal = ik.objective(self.left_gripper_link,R=Tg[0],t=Tg[1])
            else:
                Tg = se3.mul(gxform,se3.inv(right_gripper_center_xform))
                goal = ik.objective(self.right_gripper_link,R=Tg[0],t=Tg[1])
            if ik.solve(goal):
                if self.planner.is_in_self_collision(print_detail=False):
                    #print "collided"
                    continue
                self.active_grasp = grasp
                #TODO: plan a path
                self.controller.setMilestone(self.robot.getConfig())
                return True
        return False

    def move_to_ungrasp_object(self,object):
        """Sets the robot's configuration so the gripper ungrasps the object.

        If successful, sends the motion to the low-level controller and
        returns True.
        
        Otherwise, does not modify the low-level controller and returns False.
        """
        assert len(object.info.grasps) > 0,"Object doesn't define any grasps"
        return True

    def move_to_order_bin(self,object):
        """Sets the robot's configuration so the gripper is over the order bin

        If successful, sends the motion to the low-level controller and
        returns True.
        
        Otherwise, does not modify the low-level controller and returns False.
        """
        left_target = se3.apply(order_bin_xform,[0.0,0.2,order_bin_bounds[1][2]+0.1])
        right_target = se3.apply(order_bin_xform,[0.0,-0.2,order_bin_bounds[1][2]+0.1])
        qcmd = self.controller.getCommandedConfig()
        for i in range(100):
            if self.active_limb == 'left':
                goal = ik.objective(self.left_gripper_link,local=left_gripper_center_xform[1],world=left_target)
            else:
                goal = ik.objective(self.right_gripper_link,local=right_gripper_center_xform[1],world=right_target)
            #set IK solver initial configuration
            if i==0:
                self.robot.setConfig(qcmd)
            else:
                self.randomize_limb_position(self.active_limb)
            #solve
            if ik.solve(goal,tol=0.1):
                #TODO: plan a path
                if self.planner.is_in_self_collision(print_detail=False):
                    continue
                self.controller.setMilestone(self.robot.getConfig())
                return True
        return False


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
    run_perception_on_shelf(controller.knowledge)
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
        self.draw_grasps = False
        self.draw_gripper_and_camera = True

        #initialize controllers
        self.low_level_controller = LowLevelController(simworld.robot(0),self.sim.getController(0))
        self.command_queue = Queue()
        self.picking_controller = PickingController(planworld,self.low_level_controller)
        self.picking_thread = Thread(target=run_controller,args=(self.picking_controller,self.command_queue))
        self.picking_thread.start()

    def idle(self):
        if self.simulate:
            self.sim.simulate(self.dt)
            #for Q2
            if self.simworld.numRigidObjects() >= len(ground_truth_items):
                ofs = self.simworld.numRigidObjects()-len(ground_truth_items)
                for i,item in enumerate(ground_truth_items):
                    T = self.sim.getBody(self.simworld.rigidObject(ofs+i)).getTransform()
                    item.xform = T
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
            q = self.sim.getController(i).getCommandedConfig()
            r.setConfig(q)
            r.drawGL(False)
        glDisable(GL_BLEND)
        
        global ground_truth_items
        
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
        for i in ground_truth_items:
            if i.xform == None:
                continue
            #if perceived, draw in solid color
            if self.picking_controller.knowledge.bin_contents[i.bin_name]!=None and i in self.picking_controller.knowledge.bin_contents[i.bin_name]:
                glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,0.5,0,1])
                draw_oriented_box(i.xform,i.info.bmin,i.info.bmax)
            else:
                #otherwise, draw in wireframe
                glDisable(GL_LIGHTING)
                glColor3f(1,0.5,0)
                draw_oriented_wire_box(i.xform,i.info.bmin,i.info.bmax)
                glEnable(GL_LIGHTING)
            if self.draw_grasps:
                #draw grasps, if available
                g = self.picking_controller.knowledge.grasp_xforms(i)
                if g:
                    for xform in g:
                        gldraw.xform_widget(xform,0.05,0.005)

        #show gripper and camera frames
        if self.draw_gripper_and_camera:
            left_camera_link = self.simworld.robot(0).getLink(left_camera_link_name)
            right_camera_link = self.simworld.robot(0).getLink(right_camera_link_name)
            left_gripper_link = self.simworld.robot(0).getLink(left_gripper_link_name)
            right_gripper_link = self.simworld.robot(0).getLink(right_gripper_link_name)
            gldraw.xform_widget(left_camera_link.getTransform(),0.1,0.01)
            gldraw.xform_widget(right_camera_link.getTransform(),0.1,0.01)
            gldraw.xform_widget(se3.mul(left_gripper_link.getTransform(),left_gripper_center_xform),0.05,0.005)
            gldraw.xform_widget(se3.mul(right_gripper_link.getTransform(),right_gripper_center_xform),0.05,0.005)

        #draw order box 
        glDisable(GL_LIGHTING)
        glColor3f(1,0,0)
        draw_oriented_wire_box(order_bin_xform,order_bin_bounds[0],order_bin_bounds[1])
        glEnable(GL_LIGHTING)
        return

    def keyboardfunc(self,c,x,y):
        c = c.lower()
        if c=='s':
            self.simulate = not self.simulate
            print "Simulating:",self.simulate
        if c=="v":
            # print "Pressed v"
            is_collide = global_planner.is_in_self_collision()
            print "Colliding status:", is_collide
        else:
            self.command_queue.put(c)
            if c=='q':
                self.picking_thread.join()
                exit(0)
        glutPostRedisplay()

def load_apc_world():
    """Produces a world with only the Baxter, shelf, and ground plane in it."""
    world = WorldModel()
    #uncomment these lines and comment out the next 2 if you want to use the
    #full Baxter model
    #print "Loading full Baxter model (be patient, this will take a minute)..."
    #world.loadElement(os.path.join(model_dir,"baxter.rob"))
    print "Loading simplified Baxter model..."
    world.loadElement(os.path.join(model_dir,"baxter_with_parallel_gripper_col.rob"))
    print "Loading Kiva pod model..."
    world.loadElement(os.path.join(model_dir,"kiva_pod/meshes/pod_lowres.stl"))
    print "Loading plane model..."
    world.loadElement(os.path.join(model_dir,"plane.env"))
    
    #shift the Baxter up a bit (95cm)
    Rbase,tbase = world.robot(0).getLink(0).getParentTransform()
    world.robot(0).getLink(0).setParentTransform(Rbase,(0,0,0.95))
    world.robot(0).setConfig(world.robot(0).getConfig())
    
    #translate pod to be in front of the robot, and rotate the pod by 90 degrees 
    reorient = ([1,0,0,0,0,1,0,-1,0],[0,0,0.01])
    Trel = (so3.rotation((0,0,1),-math.pi/2),[1.2,0,0])
    T = reorient
    world.terrain(0).geometry().transform(*se3.mul(Trel,T))

    #initialize the shelf xform for the visualizer and object
    #xform initialization
    global ground_truth_shelf_xform
    ground_truth_shelf_xform = se3.mul(Trel,T)
    return world

def load_baxter_only_world():
    """Produces a world with only the Baxter in it."""
    world = WorldModel()
    print "Loading simplified Baxter model..."
    world.loadElement(os.path.join(model_dir,"baxter_with_parallel_gripper_col.rob"))
    #shift the Baxter up a bit (95cm)
    Rbase,tbase = world.robot(0).getLink(0).getParentTransform()
    world.robot(0).getLink(0).setParentTransform(Rbase,(0,0,0.95))
    world.robot(0).setConfig(world.robot(0).getConfig())

    global global_robot
    global_robot = world.robot(0)
    print "Robot ready..."
    # print_instance_var(global_robot)
    from klampt.robotsim import RobotModel
    '''
    @type global_robot: RobotModel
    '''
    global_robot
    return world

def spawn_objects_from_ground_truth(world):
    """For all ground_truth_items, spawns RigidObjects in the world
    according to their sizes / mass properties"""
    global ground_truth_items
    print "Initializing world objects"
    for item in ground_truth_items:
        obj = world.makeRigidObject(item.info.name)
        bmin,bmax = item.info.bmin,item.info.bmax
        center = vectorops.div(vectorops.add(bmin,bmax),2.0)
        m = obj.getMass()
        m.setMass(item.info.mass)
        m.setCom([0,0,0])
        m.setInertia(vectorops.mul([bmax[0]-bmin[0],bmax[1]-bmin[1],bmax[2]-bmin[2]],item.info.mass/12.0))
        obj.setMass(m)
        c = obj.getContactParameters()
        c.kFriction = 0.6
        c.kRestitution = 0.1;
        c.kStiffness = 100000
        c.kDamping = 100000
        obj.setContactParameters(c)
        cube = obj.geometry()
        if not cube.loadFile(os.path.join(model_dir,"cube.tri")):
            print "Error loading cube file",os.path.join(model_dir,"cube.tri")
            exit(1)
        scale = [bmax[0]-bmin[0],0,0,0,bmax[1]-bmin[1],0,0,0,bmax[2]-bmin[2]]
        translate = vectorops.sub(bmin,center)
        cube.transform(scale,translate)
        mesh = cube.getTriangleMesh()
        obj.setTransform(item.xform[0],item.xform[1])
    return

def main():
    """The main loop that loads the planning / simulation models and
    starts the OpenGL visualizer."""
    world = load_apc_world()
   
    init_ground_truth()

    #Question 1,2,3: below line should be uncommented
    simworld = load_baxter_only_world()
    #Question 4: comment out line above, uncomment lines below
    #simworld = load_apc_world()
    #spawn_objects_from_ground_truth(simworld)

    #load the resting configuration from klampt_models/baxter_rest.config
    global baxter_rest_config
    f = open(model_dir+'baxter_with_parallel_gripper_rest.config','r')
    baxter_rest_config = loader.readVector(f.readline())
    f.close()
    simworld.robot(0).setConfig(baxter_rest_config)

    
    #run the visualizer
    visualizer = MyGLViewer(simworld,world)
    visualizer.run()

if __name__ == "__main__":
    main()

###############################################################
# WRITTEN ANSWERS / COMMENTS:
# Q1. First, there will be self-collision. The way to solve it is to check for self-collision
#     (both during path execution and at end point) and reject such paths.
#     Second, robot will collide the shelf. This is similarly solved by checking robot configuration
#     and shelf geometry for collision.
#     Third, during item retrieval, even if robot gripper does not collide with the shelf, the item
#     may, because the item has a larger geometry than the gripper. There are two ways to solve it:
#     first, because the object is bound to the gripper by the grasp_xform, we can check for collision
#     between object and shelf and make sure that the whole retrieval path is collision free. Second,
#     we can hard-code a retrieval path for each gripper and each arm. This is a pretty large
#     amount of work, though.
#     Fourth, when grasping item in bin D with right camera viewing it, the failure rate is
#     particularly high. I think this is due to the strange grasping transform for that item. The
#     solution is to retry for more times.
#
#
# Q2. I put a check for "self.planner.is_in_self_collision()" after each "if ik.solve(goal):".
#     If the returned result is true (meaning colliding), I directly continue to
#     next iteration of the for loop. If not, the milestone for controller is set. I think what I
#     did is correct since I did not observe any self-collision in final configuration. However,
#     it seems that the success rate of grasping is not very high, particularly when the right hand
#     is viewing at bin D. I see that to grasp the item in bin D, the gripper need to grasp from
#     the top. Maybe this complicates stuff, but it is not my self-collision checking's fault. Maybe
#     having more retry iterations can help.
#     For collision detection, I tested for pairwise self-links except for those that are not
#     "selfCollisionEnabled". The enabling status is specified in the robot file and can be queried
#     as an instance method on robot object.
#     I found that about 1 to 4 collisions happen before the first collision-free path appears.
#     I got the number by printing out something if collision-checking fails.
#     Thus approximately 30% kinematically feasible paths are self-collision-free.
#
#
# Q3.
#
#
# Q4.
#
#
# Q5 (bonus question).
#
#
