#!/usr/bin/python
# this forces the executable file to be interpreted with this python

# TODO: simplePlannerCollision.py : Implement Collision Detection
# TODO: complexPlannerCollision.py : Add spatula model, picking from above

from klampt import robotsim
from klampt.glprogram import *
from klampt import vectorops, se3, so3, loader, gldraw, ik
from klampt.robotsim import Geometry3D
from baxter import *
from hw4_planner_impl import *
import apc
import os
import math
import random
import copy
from threading import Thread,Lock
from Queue import Queue


# configuration variables
# Question 1,2,3: set NO_SIMULATION_COLLISIONS = 1
# Question 4: set NO_SIMULATION_COLLISIONS = 0
NO_SIMULATION_COLLISIONS = 0
#Turn this on to help fast prototyping of later stages
FAKE_SIMULATION = 1
SKIP_PATH_PLANNING = 0



# The path of the klampt_models directory
model_dir = "../klampt_models/"


# The transformation of the order bin
# = identity rotation and translation in x
order_bin_xform = (so3.identity(),[0.5,0,0])
# the local bounding box of the order bin
order_bin_bounds = ([-0.2,-0.4,0],[0.2,0.4,0.7])

# global variable for baxter's resting configuration
# baxter_rest_config = [0.0]*54  # no need to declare as this because we load from file in main()
global baxter_rest_config

# Declare the shelf xform variable
global ground_truth_shelf_xform

# Order list. Can be parsed from JSON input
global orderList
orderList = ['med_item','tall_item']

# Declare the knowledge base
global knowledge

def init_ground_truth():
    global ground_truth_items
    ground_truth_items = [apc.ItemInBin(apc.tall_item,'bin_C'),
                          apc.ItemInBin(apc.small_item,'bin_A'),
                          apc.ItemInBin(apc.med_item,'bin_E')]
    ground_truth_items[0].set_in_bin_xform(ground_truth_shelf_xform,0.2,0.2,0.0)
    ground_truth_items[1].set_in_bin_xform(ground_truth_shelf_xform,0.5,0.1,math.pi/4)
    ground_truth_items[2].set_in_bin_xform(ground_truth_shelf_xform,0.6,0.4,math.pi/2)

# KnowledgeBase Class
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
        # dictionary of sensed bin contents. Append the item name to
        # the value (a list) to the bin_names key when an item is sensed
        self.bin_contents = dict((n,None) for n in apc.bin_names)
        self.order_bin_contents = []

    def bin_front_center(self,bin_name):
        bmin,bmax = apc.bin_bounds[bin_name]
        local_center = [(bmin[0]+bmax[0])*0.5, (bmin[1]+bmax[1])*0.5, bmax[2]]
        world_center = se3.apply(ground_truth_shelf_xform, local_center)
        return world_center

    def bin_vantage_point(self,bin_name):
        world_center = self.bin_front_center(bin_name)
        # Vantage point has 20cm offset from bin center
        world_offset = so3.apply(ground_truth_shelf_xform[0],[0,0,0.2])
        return vectorops.add(world_center,world_offset)

    def grasp_xforms(self,object):
        if object.xform == None: return None
        res = []
        for g in object.info.grasps:
            # NOTE: g.grasp_xform: the transformation of the gripper fingers
            #                      relative to the object's local frame
            #        object.xform: the transformation of the object center
            #                      relative to the world frame
            #   grasp_xform_world: the transformation of the gripper fingers
            #                      relative to the world frame
            grasp_xform_world = se3.mul(object.xform, g.grasp_xform)
            res.append(grasp_xform_world)
        return res

class MyController():
    """Maintains the robot's internal state. (KnowledgeBase is maintained by
       the global knowledgeBase object)
       Members include:
    - state: either 'ready' or 'holding'
    - configuration: the robot's current configuration
    - active_limb: the limb currently active, either holding or viewing a state
    - current_bin: the name of the bin where the camera is viewing or the gripper is located
    - held_object: the held object, if one is held, or None otherwise

    External modules can call viewBinAction(), graspAction(), ungraspAction(),
    and placeInOrderBinAction()
    """
    def __init__(self,world):
        self.world = world
        self.robot = world.robot(0)
        self.state = 'ready'  # or 'holding'
        self.config = self.robot.getConfig()
        self.active_limb = None
        self.current_bin = None
        self.held_object = None

        # define camera and joint link "objects"
        self.left_camera_link   = self.robot.link(left_camera_link_name)
        self.right_camera_link  = self.robot.link(right_camera_link_name)
        self.left_gripper_link  = self.robot.link(left_gripper_link_name)
        self.right_gripper_link = self.robot.link(right_gripper_link_name)
        self.left_arm_links     = [self.robot.link(i) for i in left_arm_link_names]
        self.right_arm_links    = [self.robot.link(i) for i in right_arm_link_names]

        # define mapping from "link ID" to "link index"
        id_to_index = dict([(self.robot.link(index).getID(), index) for index in range(self.robot.numLinks())])

        # define a list of link indices on both arms
        self.left_arm_indices  = [id_to_index[i.getID()] for i in self.left_arm_links]
        self.right_arm_indices = [id_to_index[i.getID()] for i in self.right_arm_links]

    def viewBinAction(self, bin):
        if self.state != 'ready' :
            print "Error: Holding an object, robot cannot view bin"
            return False
        else:
            # If a valid bin name
            if bin in apc.bin_names:
                print "Valid bin, ",

                if self.move_camera_to_bin(bin):
                    self.current_bin = bin
                    self.run_perception_on_bin(knowledge, bin)
                    print "sensed bin", bin, " with camera", self.active_limb
                else:
                    print "but move to bin", bin, " failed"
                    return False
            else:
                print "Error: Invalid bin"
                return False
        return True

    def graspAction(self):
        if self.current_bin == None:
            print "Not located at a bin"
            return False
        elif self.state != 'ready':
            print "Already holding an object, can't grasp another"
            return False
        elif len(knowledge.bin_contents[self.current_bin])==0:
            print "The current bin is empty"
            return False
        else:
            if self.move_to_grasp_object(knowledge.bin_contents[self.current_bin][0]):
                self.held_object = knowledge.bin_contents[self.current_bin].pop()
                # self.held_object = knowledge.bin_contents[self.current_bin][0]
                self.state = 'holding'
                print "Holding object",self.held_object.info.name,"in hand",self.active_limb
                return True
            else:
                print "Grasp failed"
                return False

    def ungraspAction(self):
        if self.state != 'holding':
            print "Not holding an object"
            return False
        else:
            if self.move_to_ungrasp_object(self.held_object):
                print "Object",self.held_object.info.name,"placed back in bin"
                self.state = 'ready'
                self.held_object = None
                return True
            else:
                print "Ungrasp failed"
                return False

    def placeInOrderBinAction(self):
        if self.state != 'holding':
            print "Not holding an object"
        else:
            if self.move_to_order_bin(self.held_object):
                self.drop_in_order_bin(self.held_object)
                print "Successfully placed",self.held_object.info.name,"into order bin"
                knowledge.order_bin_contents.append(self.held_object)
                self.held_object.bin_name = 'order_bin'
                self.state = 'ready'
                self.held_object = None
                return True
            else:
                print "Move to order bin failed"
                return False

    def fulfillOrderAction(self,objectList):
        # go through all bins
        for b in apc.bin_names:
            # if the bin is empty
            if knowledge.bin_contents[b] == None:
                # try to view the bin
                if not self.viewBinAction(b):
                    continue

            doNextBin = False
            # if any of the objects in the bin is in the "remaining objets" list, and we want to keep searching in current bin
            while any(obj.info.name in objectList for obj in knowledge.bin_contents[b]) and not doNextBin:
                # grasp failed
                if not self.graspAction():
                    doNextBin = True
                    break
                # if robot is not holding on to something, or the object it is holding is not in order
                while (self.held_object == None or self.held_object.info.name not in objectList) and not doNextBin:
                    if not self.ungraspAction():
                        return False
                    if not self.graspAction():
                        doNextBin = True
                        break

                # temporarily store held object because placeInOrderBinAction sets held object to None
                heldObject = self.held_object
                if not self.placeInOrderBinAction():
                    return False

        if len(objectList) == 0:
            return True
        print "Items Remaining in Order: ", objectList
        return False

    def move_camera_to_bin(self,bin):
        # Camera rotation and translation relative to world frame
        R_camera = [0,0,-1, 1,0,0, 0,1,0]
        t_camera = knowledge.bin_vantage_point(bin)

        # Setup ik objectives for both arms
        left_goal  = ik.objective(self.left_camera_link,  R=R_camera, t=t_camera)
        right_goal = ik.objective(self.right_camera_link, R=R_camera, t=t_camera)

        # Joint Limits
        qmin,qmax = self.robot.getJointLimits()

        for i in range(100):
            # initialize to the resting pose
            # NOTE: that this (without [:]) doesn't work because the list
            #       must be copied by value, not reference.
            #       See python shallow copy vs. deep copy for more details
            # q = baxter_rest_config
            q = baxter_rest_config[:]

            # use left hand
            if random.random() < 0.5:
                # randomly initialize each joint in the arm within joint limits
                for jointIndex in self.left_arm_indices:
                    q[jointIndex] = random.uniform(qmin[jointIndex], qmax[jointIndex])
                self.robot.setConfig(q)

                # attempt to solve ik objective
                if ik.solve(left_goal):
                    self.config = self.robot.getConfig()
                    self.active_limb = 'left'
                    return True

            # use right hand
            else:
                # randomly initialize each joint in the arm within joint limits
                for jointIndex in self.right_arm_indices:
                    q[jointIndex] = random.uniform(qmin[jointIndex], qmax[jointIndex])
                self.robot.setConfig(q)

                # attempt to solve ik objective
                if ik.solve(right_goal):
                    self.config = self.robot.getConfig()
                    self.active_limb = 'right'
                    return True
        return False

    def move_to_grasp_object(self,object):
        # Candidate grasps for the object
        grasps = knowledge.grasp_xforms(object)

        # Joint Limits
        qmin,qmax = self.robot.getJointLimits()

        for i in range(100):
            # initialize to the resting pose
            # NOTE: that this (without [:]) doesn't work because the list
            #       must be copied by value, not reference.
            #       See python shallow copy vs. deep copy for more details
            #       ( q = baxter_rest_config ) doesn't work!
            q = baxter_rest_config[:]

            g = random.choice(grasps)

            # use left hand
            if self.active_limb == 'left':
                # randomly initialize each joint in the arm within joint limits
                for jointIndex in self.left_arm_indices:
                    q[jointIndex] = random.uniform(qmin[jointIndex], qmax[jointIndex])
                self.robot.setConfig(q)

                # Setup ik objective using 3 points around the grasps and gripper to align gripper to grasp points
                l, w = list(), list()
                l.append(left_gripper_center_xform[1])
                w.append(g[1])

                # align red-axis on gripper with red-axis on object
                l.append([left_gripper_center_xform[1][0], left_gripper_center_xform[1][1]+0.01, left_gripper_center_xform[1][2]])
                offset = se3.apply((g[0], [0,0,0]), [0.01,0,0])
                w.append( [g[1][0]+offset[0], g[1][1]+offset[1], g[1][2]+offset[2]] )

                left_goal  = ik.objective(self.left_gripper_link, local=[l[0],l[1]], world=[w[0],w[1]])

                # attempt to solve ik objective
                if ik.solve(left_goal):
                    self.config = self.robot.getConfig()
                    self.active_limb = 'left'
                    return True

            # use right hand
            else:
                # randomly initialize each joint in the arm within joint limits
                for jointIndex in self.right_arm_indices:
                    q[jointIndex] = random.uniform(qmin[jointIndex], qmax[jointIndex])
                self.robot.setConfig(q)

                # Setup ik objective using 3 points around the grasps and gripper to align gripper to grasp points
                l, w = list(), list()
                l.append(right_gripper_center_xform[1])
                w.append(g[1])

                l.append([right_gripper_center_xform[1][0], right_gripper_center_xform[1][1]+0.01, right_gripper_center_xform[1][2]])
                offset = se3.apply((g[0], [0,0,0]), [0.01,0,0])
                w.append( [g[1][0]+offset[0], g[1][1]+offset[1], g[1][2]+offset[2]] )

                right_goal  = ik.objective(self.right_gripper_link, local=[l[0],l[1]], world=[w[0],w[1]])

                # attempt to solve ik objective
                if ik.solve(right_goal):
                    self.config = self.robot.getConfig()
                    self.active_limb = 'right'
                    return True
        return False

    def move_to_ungrasp_object(self,object):
        knowledge.bin_contents[self.current_bin].append(object)
        return self.move_camera_to_bin(object.bin_name)

    def move_to_order_bin(self,object):
        # apply the order_bin_xform to the given point
        left_target_point  = se3.apply(order_bin_xform, [0.0, 0.2, order_bin_bounds[1][2]+0.1])
        right_target_point = se3.apply(order_bin_xform, [0.0, -0.2, order_bin_bounds[1][2]+0.1])

        # Setup ik objectives for both arms
        left_goal  = ik.objective(self.left_gripper_link,  local=left_gripper_center_xform[1],  world=left_target_point)
        right_goal = ik.objective(self.right_gripper_link, local=right_gripper_center_xform[1], world=right_target_point)

        qmin, qmax = self.robot.getJointLimits()

        for i in range(100):
            # initialize to the resting pose
            # NOTE: that this (without [:]) doesn't work because the list
            #       must be copied by value, not reference.
            #       See python shallow copy vs. deep copy for more details
            #       ( q = baxter_rest_config ) doesn't work!
            q = baxter_rest_config[:]

            # use left hand
            if self.active_limb == 'left':
                # randomly initialize each joint in the arm within joint limits
                for jointIndex in self.left_arm_indices:
                    q[jointIndex] = random.uniform(qmin[jointIndex], qmax[jointIndex])
                self.robot.setConfig(q)

                # attempt to solve ik objective
                if ik.solve(left_goal):
                    self.config = self.robot.getConfig()
                    return True

            # use right hand
            else:
                # randomly initialize each joint in the arm within joint limits
                for jointIndex in self.right_arm_indices:
                    q[jointIndex] = random.uniform(qmin[jointIndex], qmax[jointIndex])
                self.robot.setConfig(q)

                # attempt to solve ik objective
                if ik.solve(right_goal):
                    self.config = self.robot.getConfig()
                    return True
        self.config = self.robot.getConfig()
        return False

    def drop_in_order_bin(self,object):
        R, t = order_bin_xform
        tRandom = [random.uniform(-0.05,0.1),random.uniform(-0.4,.4),0]
        objHeight = object.info.bmax[2] - object.info.bmin[2]
        object.xform = (R, [t[0]+tRandom[0], t[1]+tRandom[1], t[2]+objHeight/2])
        if object.info.name in orderList:
            print "OrderList:", orderList
            print "Picked Item:", object.info.name
            orderList.remove(object.info.name)
        else:
            print "OrderList:", orderList
            print "Wrongly Picked Item:", object.info.name
        return True

    def run_perception_on_bin(self,knowledge,bin_name):
        """This is a fake perception module that simply reveals all the items
        the given bin."""
        # if the dictionary "bin_contents" doesn't contain any values for the key "bin_name"
        if knowledge.bin_contents[bin_name]==None:
            # not sensed yet
            knowledge.bin_contents[bin_name] = []
            for item in ground_truth_items:
                if (item.bin_name == bin_name) :
                    # add the item to the list of sensed items for the bin
                    knowledge.bin_contents[bin_name].append(item)
        return

# Define methods for drawing objects in scene
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
def draw_oriented_box(xform,bmin,bmax):
    """Helper: draws an oriented box"""
    draw_xformed(xform,lambda:gldraw.box(bmin,bmax))
def draw_oriented_wire_box(xform,bmin,bmax):
    """Helper: draws an oriented wireframe box"""
    draw_xformed(xform,lambda:draw_wire_box(bmin,bmax))

class MyGLViewer(GLNavigationProgram):
    """This class is used to interact with the world model in hw2.

    Pressing 'a-l' runs the view_bin method which should set the robot to a
    configuration that places a hand camera such that it points inside the
    bin.

    Pressing 's' should "sense" the currently pointed-to bin, running a fake
    perception module.

    Pressing 'x' should "grasp" an object in the currently pointed-to-bin
    with either one of the hands at the designated grasp point.

    Pressing 'u' should "ungrasp" an object currently grasped inside a bin.

    Pressing 'p' should "put down" an object in the order bin
    """
    def __init__(self,world):
        GLNavigationProgram.__init__(self,"My GL program")
        self.world = world
        self.controller = MyController(world)

        # you can set these to true to draw the bins, grasps, and/or gripper/camera frames
        self.draw_bins = False
        self.draw_grasps = True
        self.draw_gripper_and_camera = True

        init_ground_truth()

    def display(self):
        # you may run auxiliary openGL calls, if you wish to visually debug
        # self.world.robot(0).setConfig(self.controller.config)
        self.world.drawGL()

        # show bin boxes
        if self.draw_bins:
            glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,1,0,1])
            for b in apc.bin_bounds.values():
               draw_oriented_box(ground_truth_shelf_xform,b[0],b[1])

            for b in apc.bin_names:
                c = knowledge.bin_front_center(b)
                if c:
                    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,1,0.5,1])
                    r = 0.01
                    gldraw.box([c[0]-r,c[1]-r,c[2]-r],[c[0]+r,c[1]+r,c[2]+r])
                c = knowledge.bin_vantage_point(b)
                if c:
                    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0.5,1,0.5,1])
                    r = 0.01
                    gldraw.box([c[0]-r,c[1]-r,c[2]-r],[c[0]+r,c[1]+r,c[2]+r])

        # show object state
        for i in ground_truth_items:
            if i.xform == None:
                continue

            if i.bin_name == 'order_bin':
            # draw in wireframe
                glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,0.5,0,1])
                draw_oriented_box(i.xform,i.info.bmin,i.info.bmax)
                continue

            # if perceived bin has an item, draw it in solid color
            if knowledge.bin_contents[i.bin_name]!=None and i in knowledge.bin_contents[i.bin_name]:
                glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,0.5,0,1])
                draw_oriented_box(i.xform,i.info.bmin,i.info.bmax)
            else:
                # otherwise, draw in wireframe
                glDisable(GL_LIGHTING)
                glColor3f(1,0.5,0)
                draw_oriented_wire_box(i.xform,i.info.bmin,i.info.bmax)
                glEnable(GL_LIGHTING)

            if self.draw_grasps:
                # draw grasps, if available
                g = knowledge.grasp_xforms(i)
                if g:
                    for xform in g:
                        gldraw.xform_widget(xform,0.02,0.002)

        # Show gripper and camera frames
        if self.draw_gripper_and_camera:
            left_camera_link = self.world.robot(0).link(left_camera_link_name)
            right_camera_link = self.world.robot(0).link(right_camera_link_name)
            left_gripper_link = self.world.robot(0).link(left_gripper_link_name)
            right_gripper_link = self.world.robot(0).link(right_gripper_link_name)
            gldraw.xform_widget(left_camera_link.getTransform(),0.1,0.01)
            gldraw.xform_widget(right_camera_link.getTransform(),0.1,0.01)
            gldraw.xform_widget(se3.mul(left_gripper_link.getTransform(),left_gripper_center_xform),0.05,0.005, lighting=False, fancy=True)
            gldraw.xform_widget(se3.mul(right_gripper_link.getTransform(),right_gripper_center_xform),0.05,0.005, lighting=False, fancy=True)

        # Show world frame and shelf frame
        gldraw.xform_widget(ground_truth_shelf_xform, 0.1, 0.015, lighting=False, fancy=True)
        gldraw.xform_widget(se3.identity(), 0.2, 0.037, lighting=False, fancy=True)

        # Draw order box
        glDisable(GL_LIGHTING)
        glColor3f(1,0,0)
        draw_oriented_wire_box(order_bin_xform,order_bin_bounds[0],order_bin_bounds[1])
        glEnable(GL_LIGHTING)
        return

    def keyboardfunc(self,c,x,y):
        c = c.lower()
        if c >= 'a' and c <= 'l':
            self.controller.viewBinAction('bin_'+c.upper())
        elif c == 'x':
            self.controller.graspAction()
        elif c == 'u':
            self.controller.ungraspAction()
        elif c == 'p':
            self.controller.placeInOrderBinAction()
        elif c == 'o':
            self.controller.fulfillOrderAction(orderList)
        elif c == 'r':
            restart_program()
        glutPostRedisplay()

def restart_program():
    """Restarts the current program.
    Note: this function does not return. Any cleanup action (like
    saving data) must be done before calling this function."""
    python = sys.executable
    os.execl(python, python, * sys.argv)

if __name__ == "__main__":
    world = robotsim.WorldModel()
    # or,
    # from klampt import *; world = WorldModel()

    # Instantiate global knowledge base
    knowledge = KnowledgeBase()

    # Load the robot model
    # 1) full Baxter model
    # print "Loading full Baxter model (be patient, this will take a minute)..."
    # world.loadElement(os.path.join(model_dir,"baxter.rob"))
    # 2) simplified Baxter model
    print "\n<Loading simplified Baxter model...>"
    world.loadElement(os.path.join(model_dir,"baxter_col.rob"))

    # Load the shelves
    # NOTE: world.loadRigidObject(~) works too because the shelf model is a rigid object
    #       and loadElement automatically detects the object type
    print "\n<Loading Kiva pod model...>"
    world.loadElement(os.path.join(model_dir,"kiva_pod/model.obj"))

    # Load the floor plane
    print "\n<Loading plane model...>"
    world.loadElement(os.path.join(model_dir,"plane.env"))

    # Shift the Baxter up a bit (95cm)
    Rbase,tbase = world.robot(0).link(0).getParentTransform()
    world.robot(0).link(0).setParentTransform(Rbase,(0,0,0.95))
    world.robot(0).setConfig(world.robot(0).getConfig())

    # Translate the shelf to be in front of the robot, and rotate it by 90 degrees
    Trel = (so3.rotation((0,0,1),-math.pi/2),[1.2,0,0])  # desired shelf's xform
    T = world.rigidObject(0).getTransform()              # shelf model's xform
    world.rigidObject(0).setTransform(*se3.mul(Trel,T))  # combine the two xforms

    # Up until this point, the baxter model is extending its arms
    # Load the resting configuration from klampt_models/baxter_rest.config
    f = open(model_dir+'baxter_rest.config','r')
    baxter_rest_config = loader.readVector(f.readline())
    f.close()
    world.robot(0).setConfig(baxter_rest_config)

    # Orient bin boxes correctly w.r.t. the shelf
    ground_truth_shelf_xform = world.rigidObject(0).getTransform()
    R = so3.mul(apc.Xto_Z,ground_truth_shelf_xform[0])
    ground_truth_shelf_xform = (R, [1.2,0,0])

    # run the visualizer
    visualizer = MyGLViewer(world)
    visualizer.run()
