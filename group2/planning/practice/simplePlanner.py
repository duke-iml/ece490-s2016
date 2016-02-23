#!/usr/bin/python
# this forces the executable file to be interpreted with this python

# TODO: finish implementing MyGLViewer (order bin, object, etc.)

from klampt import robotsim
from klampt.glprogram import *
from klampt import so3, loader
# from klampt import vectorops,so3,se3,gldraw,ik,loader
# import apc
import os
# import math
import random

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

# global variable for baxter's resting configuration
# baxter_rest_config = [0.0]*54  # no need to declare as this because we load from file in main()
global baxter_rest_config

# the transformation of the order bin
# = identity rotation and translation in x
order_bin_xform = (so3.identity(),[0.5,0,0])
# the local bounding box of the order bin
order_bin_bounds = ([-0.2,-0.4,0],[0.2,0.4,0.7])

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

        # initialize the shelf xform for the visualizer and object
        # xform initialization
        global ground_truth_shelf_xform
        ground_truth_shelf_xform = world.rigidObject(0).getTransform()

        # TODO: Merge these four lines to hw2_imply.py ?
        # Added these lines to fix the object orientations
        R1 = so3.rotation([0,-1,0], math.pi/2)
        R2 = so3.rotation([0,0,-1], math.pi/2)
        R = so3.mul(R1,R2)
        ground_truth_shelf_xform = (R, [1.2,0,0])

        # init_ground_truth()
        # self.controller.knowledge.shelf_xform = ground_truth_shelf_xform

    def display(self):
        #you may run auxiliary openGL calls, if you wish to visually debug
        # self.world.robot(0).setConfig(self.controller.config)
        self.world.drawGL()
        global ground_truth_items

        # #show bin boxes
        # if self.draw_bins:
        #     glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,1,0,1])
        #     for b in apc.bin_bounds.values():
        #         draw_oriented_box(self.controller.knowledge.shelf_xform,b[0],b[1])
        #     for b in apc.bin_names:
        #         c = self.controller.knowledge.bin_front_center(b)
        #         if c:
        #             glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,1,0.5,1])
        #             r = 0.01
        #             gldraw.box([c[0]-r,c[1]-r,c[2]-r],[c[0]+r,c[1]+r,c[2]+r])
        #         c = self.controller.knowledge.bin_vantage_point(b)
        #         if c:
        #             glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0.5,1,0.5,1])
        #             r = 0.01
        #             gldraw.box([c[0]-r,c[1]-r,c[2]-r],[c[0]+r,c[1]+r,c[2]+r])

        # #show object state
        # for i in ground_truth_items:
        #     if i.xform == None:
        #         continue
        #     #if perceived, draw in solid color
        #     if self.controller.knowledge.bin_contents[i.bin_name]!=None and i in self.controller.knowledge.bin_contents[i.bin_name]:
        #         glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,0.5,0,1])
        #         draw_oriented_box(i.xform,i.info.bmin,i.info.bmax)
        #     else:
        #         #otherwise, draw in wireframe
        #         glDisable(GL_LIGHTING)
        #         glColor3f(1,0.5,0)
        #         draw_oriented_wire_box(i.xform,i.info.bmin,i.info.bmax)
        #         glEnable(GL_LIGHTING)
        #     if self.draw_grasps:
        #         #draw grasps, if available
        #         g = self.controller.knowledge.grasp_xforms(i)
        #         if g:
        #             for xform in g:
        #                 gldraw.xform_widget(xform,0.05,0.005)

        # #show gripper and camera frames
        # if self.draw_gripper_and_camera:
        #     left_camera_link = self.world.robot(0).getLink(left_camera_link_name)
        #     right_camera_link = self.world.robot(0).getLink(right_camera_link_name)
        #     left_gripper_link = self.world.robot(0).getLink(left_gripper_link_name)
        #     right_gripper_link = self.world.robot(0).getLink(right_gripper_link_name)
        #     gldraw.xform_widget(left_camera_link.getTransform(),0.1,0.01)
        #     gldraw.xform_widget(right_camera_link.getTransform(),0.1,0.01)
        #     gldraw.xform_widget(se3.mul(left_gripper_link.getTransform(),left_gripper_center_xform),0.05,0.005)
        #     gldraw.xform_widget(se3.mul(right_gripper_link.getTransform(),right_gripper_center_xform),0.05,0.005)

        # #draw order box
        # glDisable(GL_LIGHTING)
        # glColor3f(1,0,0)
        # draw_oriented_wire_box(order_bin_xform,order_bin_bounds[0],order_bin_bounds[1])
        # glEnable(GL_LIGHTING)
        return

    # def keyboardfunc(self,c,x,y):
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
            self.controller.fulfillOrderAction(['med_item','small_item'])
        glutPostRedisplay()


if __name__ == "__main__":
    # or, from klampt import *; world = WorldModel()
    world = robotsim.WorldModel()

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
    Rbase,tbase = world.robot(0).getLink(0).getParentTransform()
    world.robot(0).getLink(0).setParentTransform(Rbase,(0,0,0.95))
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

    # run the visualizer
    visualizer = MyGLViewer(world)
    visualizer.run()
