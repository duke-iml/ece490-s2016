#!/usr/bin/python
# this forces the executable file to be interpreted with this python

from klampt import robotsim
from klampt.glprogram import *
from klampt import so3, loader, gldraw
# from klampt import vectorops,se3,ik
import apc
# import os
# import math
# import random

# The path of the klampt_models directory
model_dir = "../klampt_models/"

# indices of the left and right cameras in the Baxter robot file
left_camera_link_name = 'left_hand_camera'
right_camera_link_name = 'right_hand_camera'

# indices of the left and right grippers in the Baxter robot file
left_gripper_link_name = 'left_gripper'
right_gripper_link_name = 'right_gripper'

# indices of the left and right arms in the Baxter robot file
left_arm_link_names = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
right_arm_link_names = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']

# local transformations (rotation, translation pairs) of the grasp center
# = identity rotation, and translation in y, and z
left_gripper_center_xform = (so3.identity(),[0,-0.04,0.1])
right_gripper_center_xform = (so3.identity(),[0,-0.04,0.1])

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

# Declare the knowledge base
global knowledge

# List of actual items -- this is only used for the fake perception module, and your
# code should not use these items directly
# ground_truth_items = []
# ground_truth_shelf_xform = se3.identity()


def init_ground_truth():
    global ground_truth_items
    ground_truth_items = [apc.ItemInBin(apc.tall_item,'bin_G'),
                          apc.ItemInBin(apc.small_item,'bin_A'),
                          apc.ItemInBin(apc.med_item,'bin_D')]
    # apy.ItemInBin.set_in_bin_xform(self,shelf_xform,ux,uy,theta)
    ground_truth_items[0].set_in_bin_xform(ground_truth_shelf_xform,0.2,0.2,0.0)
    ground_truth_items[1].set_in_bin_xform(ground_truth_shelf_xform,0.5,0.1,math.pi/4)
    ground_truth_items[2].set_in_bin_xform(ground_truth_shelf_xform,0.6,0.4,math.pi/2)

# TODO:
# def run_perception_on_bin(knowledge,bin_name):
#     """This is a fake perception module that simply reveals all the items
#     the given bin."""
#     global ground_truth_items
#     if knowledge.bin_contents[bin_name]==None:
#         #not sensed yet
#         knowledge.bin_contents[bin_name] = []
#         for item in ground_truth_items:
#             if item.bin_name == bin_name:
#                 #place it in the bin
#                 knowledge.bin_contents[bin_name].append(item)
#     return


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
        self.bin_contents = dict((n,None) for n in apc.bin_names)
        self.order_bin_contents = []
        # Use global variable ground_truth_shelf_xform instead
        # self.shelf_xform = se3.identity()

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
            grasp_xform_world = se3.mul(object.xform, g.grasp_xform)
            res.append(grasp_xform_world)
        return res

# TODO: Controller

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
        # self.controller = PickingController(world)

        # you can set these to true to draw the bins, grasps, and/or gripper/camera frames
        self.draw_bins = False
        self.draw_grasps = True
        self.draw_gripper_and_camera = True

        init_ground_truth()

    def display(self):
        # you may run auxiliary openGL calls, if you wish to visually debug
        # self.world.robot(0).setConfig(self.controller.config)
        self.world.drawGL()
        global ground_truth_items

        # show bin boxes
        if self.draw_bins:
            glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,1,0,1])
            for b in apc.bin_bounds.values():
               # draw_oriented_box(self.controller.knowledge.shelf_xform,b[0],b[1])
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
                        gldraw.xform_widget(xform,0.05,0.005)

        # # Show gripper and camera frames
        if self.draw_gripper_and_camera:
            left_camera_link = self.world.robot(0).link(left_camera_link_name)
            right_camera_link = self.world.robot(0).link(right_camera_link_name)
            left_gripper_link = self.world.robot(0).link(left_gripper_link_name)
            right_gripper_link = self.world.robot(0).link(right_gripper_link_name)
            gldraw.xform_widget(left_camera_link.getTransform(),0.1,0.01)
            gldraw.xform_widget(right_camera_link.getTransform(),0.1,0.01)
            gldraw.xform_widget(se3.mul(left_gripper_link.getTransform(),left_gripper_center_xform),0.05,0.005)
            gldraw.xform_widget(se3.mul(right_gripper_link.getTransform(),right_gripper_center_xform),0.05,0.005)

        # Draw order box
        glDisable(GL_LIGHTING)
        glColor3f(1,0,0)
        draw_oriented_wire_box(order_bin_xform,order_bin_bounds[0],order_bin_bounds[1])
        glEnable(GL_LIGHTING)
        return

    # TODO:
    # def keyboardfunc(self,c,x,y):
        # c = c.lower()
        # if c >= 'a' and c <= 'l':
        #     self.controller.viewBinAction('bin_'+c.upper())
        # elif c == 'x':
        #     self.controller.graspAction()
        # elif c == 'u':
        #     self.controller.ungraspAction()
        # elif c == 'p':
        #     self.controller.placeInOrderBinAction()
        # elif c == 'o':
        #     self.controller.fulfillOrderAction(['med_item','small_item'])
        # glutPostRedisplay()





if __name__ == "__main__":
   # or, from klampt import *; world = WorldModel()
   world = robotsim.WorldModel()

   knowledge = KnowledgeBase()

   # Load the robot model
   # 1) full Baxter model
   # print "Loading full Baxter model (be patient, this will take a minute)..."
   # world.loadElement(os.path.join(model_dir,"baxter.rob"))
   # 2) simplified Baxter model
   print "Loading simplified Baxter model..."
   world.loadElement(os.path.join(model_dir,"baxter_col.rob"))

   # Load the shelves
   # NOTE: world.loadRigidObject(~) works too because the shelf model is a rigid object
   #       and loadElement automatically detects the object type
   print "Loading Kiva pod model..."
   world.loadElement(os.path.join(model_dir,"kiva_pod/model.obj"))

   # Load the floor plane
   print "Loading plane model..."
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
