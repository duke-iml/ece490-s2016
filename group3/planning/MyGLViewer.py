from klampt.glprogram import *
from util.constants import *
from klampt import gldraw

class MyGLViewer(GLNavigationProgram):
    def __init__(self,world):
        GLNavigationProgram.__init__(self,"double shine grab visualizer")
        self.world = world
        self.draw_frames = True

    def display(self):
        self.world.drawGL()

        if self.draw_frames:
            left_camera_link = self.world.robot(0).link(LEFT_CAMERA_LINK_NAME)
            right_camera_link = self.world.robot(0).link(RIGHT_CAMERA_LINK_NAME)
            left_gripper_link = self.world.robot(0).link(LEFT_GRIPPER_LINK_NAME)
            right_gripper_link = self.world.robot(0).link(RIGHT_GRIPPER_LINK_NAME)
            gldraw.xform_widget(left_camera_link.getTransform(),0.1,0.01)
            gldraw.xform_widget(right_camera_link.getTransform(),0.1,0.01)
            gldraw.xform_widget(se3.mul(left_gripper_link.getTransform(),LEFT_GRIPPER_CENTER_XFORM),0.05,0.005)
            gldraw.xform_widget(se3.mul(right_gripper_link.getTransform(),RIGHT_GRIPPER_CENTER_XFORM),0.05,0.005)
