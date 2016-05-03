#!/usr/bin/python
import rospy
import baxter_interface
import json

'''
Joint Reader Utility:
This utility is designed to provide a clean readout of the joint angles for the baxter
limbs, as they are currently positioned.  The purpose of this utility is to allow
for postured poses to be cleanly copied into any other utilities that might need the
ability to command the baxter arms to move to specific positions.

The following is the list of appropriate file names to use for the right arm of baxter and 
the motion_state_machine class.  PLEASE USE THESE NAMES, otherwise the motion state machine
will not work without modification to the motion_state_machien code!!

Intermediate motion states are named as follows:
    rbin_inter_rx_y, where
        x = the row number we are trying to move to (row 1-3, from top to bottom)
        y = a or b, where a should come before b when moving from the tote to a shelf

Shelf motion states are named as follows:
    rbin_shelf_x, where
        x= the shelf number we are trying to move to (1-12, left to right and top to bottom)

The following are used in motion_state_machine (all with the right arm, in utility prompt hit enter to skip naming left arm)

Tote file names:
bin_right

Intermediate file names:
rbin_inter_r1_a
rbin_inter_r1_b
rbin_inter_r2_a
rbin_inter_r2_b
rbin_inter_r3_a
rbin_inter_r3_b
rbin_inter_r4_a
rbin_inter_r4_b

Shelf file names:
rbin_shelf_1
rbin_shelf_2
rbin_shelf_3
rbin_shelf_4
rbin_shelf_5
rbin_shelf_6
rbin_shelf_7
rbin_shelf_8
rbin_shelf_9
rbin_shelf_10
rbin_shelf_11
rbin_shelf_12

Dennis Lynch
2-29-16
'''

names = ["bin_right",
        "rbin_shelf_1",
        "rbin_shelf_2",
        "rbin_shelf_3",
        "rbin_inter_r1_a",
        "rbin_inter_r1_b",
        "rbin_shelf_4",
        "rbin_shelf_5",
        "rbin_shelf_6",
        "rbin_inter_r2_a",
        "rbin_inter_r2_b",
        "rbin_shelf_7",
        "rbin_shelf_8",
        "rbin_shelf_9",
        "rbin_inter_r3_a",
        "rbin_inter_r3_b",
        "rbin_shelf_10",
        "rbin_shelf_11",
        "rbin_shelf_12",
        "rbin_inter_r4_a",
        "rbin_inter_r4_b"]


#Initialize a new ROS node
rospy.init_node('retriever_node')

#Initialize skipping variables
#skip_left = False
#skip_right = False

#Specify the file names
#print "Please specify the limb file names (leave blank to not create a file)"
#left_arm_filename = raw_input("Left arm filename:")
#right_arm_filename = raw_input("Right arm filename:")

#if left_arm_filename == "":
#    skip_left = True

#if right_arm_filename == "":
#    skip_right = True

#full_left_filename = "state_json_files/" + left_arm_filename + ".json"
#left_arm = baxter_interface.Limb('left')
#left_joints = left_arm.joint_names()
#left_angles = [left_arm.joint_angle(j) for j in left_joints]
#with open(full_left_filename, "w") as file:
#    json.dump({left_joints[i]:left_angles[i] for i in range(0, len(left_angles))}, file, indent=4)

for name in names:
    print "Setting", name
    print "Press Enter..."
    raw_input()
    full_right_filename = "state_json_files/" + name + ".json"
    right_arm = baxter_interface.Limb('right')
    right_joints = right_arm.joint_names()
    right_angles = [right_arm.joint_angle(j) for j in right_joints]
    with open(full_right_filename, "w") as file:
        json.dump({right_joints[i]:right_angles[i] for i in range(0, len(right_angles))}, file, indent=4)

quit()
