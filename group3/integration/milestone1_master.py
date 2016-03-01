import sys
sys.path.insert(0, "..")

import rospy
import baxter_interface
from klampt import *
from klampt.glprogram import *
from klampt import vectorops,so3,se3,gldraw,ik,loader
from util.constants import *
from perception.pc import PCProcessor

class MyGLViewer(GLNavigationProgram):
    def __init__(self,world):
        GLNavigationProgram.__init__(self,"My GL program")
        self.world = world
        self.draw_gripper_and_camera = True

    def display(self):
        self.world.drawGL()

        #show gripper and camera frames
        if self.draw_gripper_and_camera:
            left_camera_link = self.world.robot(0).link(LEFT_CAMERA_LINK_NAME)
            right_camera_link = self.world.robot(0).link(RIGHT_CAMERA_LINK_NAME)
            left_gripper_link = self.world.robot(0).link(LEFT_GRIPPER_LINK_NAME)
            right_gripper_link = self.world.robot(0).link(RIGHT_GRIPPER_LINK_NAME)
            gldraw.xform_widget(left_camera_link.getTransform(),0.1,0.01)
            gldraw.xform_widget(right_camera_link.getTransform(),0.1,0.01)
            gldraw.xform_widget(se3.mul(left_gripper_link.getTransform(),LEFT_GRIPPER_CENTER_XFORM),0.05,0.005)
            gldraw.xform_widget(se3.mul(right_gripper_link.getTransform(),RIGHT_GRIPPER_CENTER_XFORM),0.05,0.005)

class Milestone1Master:
    def __init__(self):
        self.state = 'waiting'

    def setupWorld(self):
        world = WorldModel()

        print "Loading simplified Baxter model..."
        world.loadElement(os.path.join(KLAMPT_MODELS_DIR,"baxter_col.rob"))
        print "Loading Kiva pod model..."
        world.loadElement(os.path.join(KLAMPT_MODELS_DIR,"kiva_pod/model.obj"))
        print "Loading plane model..."
        world.loadElement(os.path.join(KLAMPT_MODELS_DIR,"plane.env"))

        Rbase,tbase = world.robot(0).link(0).getParentTransform()
        world.robot(0).link(0).setParentTransform(Rbase,(0,0,0.95))
        world.robot(0).setConfig(world.robot(0).getConfig())
    
        #translate pod to be in front of the robot, and rotate the pod by 90 degrees 
        Trel = (so3.rotation((0,0,1),-math.pi/2),[1.2,0,0])
        T = world.rigidObject(0).getTransform()
        world.rigidObject(0).setTransform(*se3.mul(Trel,T))

        #load the resting configuration from klampt_models/baxter_rest.config
        global baxter_rest_config
        f = open(KLAMPT_MODELS_DIR+'baxter_rest.config','r')
        baxter_rest_config = loader.readVector(f.readline())
        f.close()
        world.robot(0).setConfig(baxter_rest_config)

        #run the visualizer
        visualizer = MyGLViewer(world)
        visualizer.run()

    def start(self):
        rospy.init_node("milestone1_master", anonymous=True)
        limb_left = baxter_interface.Limb('left')
        limb_right = baxter_interface.Limb('right')
        pc_processor = PCProcessor()
        #rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        #rospy.spin()

        # print "Moving Left limb to 0"
        # limb_left.move_to_joint_positions(Q_LEFT_ZEROS)

        # print "Moving right limb to 0"
        # limb_right.move_to_joint_positions(Q_RIGHT_ZEROS)

        print "Scanning bin for point cloud"
        limb_right.move_to_joint_positions(Q_SCAN_BIN)
        cloud = "hi"
        cloud = pc_processor.subtractShelf(cloud)
        centroid = pc_processor.getCentroid(cloud)

        # print "Moving to centroid of cloud"
        # Calculate IK for cloud centroid
        # Move to the IK solution

        # print "Moving spatula to bin"
        # limb_left.move_to_joint_positions(Q_SPATULA_AT_BIN)

        # print "Scanning spatula"
        # limb_right.move_to_joint_positions(Q_SCAN_SPATULA)
        

if __name__ == '__main__':
    print "Starting Milestone1Master"
    master = Milestone1Master()
    master.setupWorld()
    #master.start()
