import sys
sys.path.insert(0, "..")

import rospy
import baxter_interface
import klampt
from util.constants import *
from perception.pc import PCProcessor

class Milestone1Master:
    def __init__(self):
        self.state = 'waiting'

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
    Milestone1Master().start()