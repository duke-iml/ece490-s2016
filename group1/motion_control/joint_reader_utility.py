import rospy
import baxter_interface
import json

'''
Joint Reader Utility:
This utility is designed to provide a clean readout of the joint angles for the baxter
limbs, as they are currently positioned.  The purpose of this utility is to allow
for postured poses to be cleanly copied into any other utilities that might need the
ability to command the baxter arms to move to specific positions.

Dennis Lynch
2-29-16
'''

#Initialize a new ROS node
rospy.init_node('retriever_node')

#Initialize the variables for baxter's arms
left_arm = baxter_interface.Limb('left')
right_arm = baxter_interface.Limb('right')

#Initialize an array of joint names - these are the labels that each value should be associated with
left_joints = left_arm.joint_names()
right_joints = right_arm.joint_names()

#Retrieve the current angle (in radians?) for each of the current joints on each baxter arm
left_angles = [left_arm.joint_angle(j) for j in left_joints]
right_angles = [right_arm.joint_angle(j) for j in right_joints]

#This prints out the angles, allowing for a user to copy and paste the terminal output cleanly to the desired python module
print "Left Arm Angles (radians):"
print "* Copy and paste everything in the terminal between the dotted lines directly into the python module"
print "--------------------"
print "{\'" + left_joints[0] + "\'",":",left_angles[0], ","
for i in range(1, len(left_angles) - 1):
    print "\'" + left_joints[i] + "\'",":",left_angles[i],","

print "\'" + left_joints[len(left_angles) - 1] + "\'",":",left_angles[len(left_angles) - 1], "}"
print "--------------------"
print ""
print ""
print "Right Arm Angles (radians):"
print "* Copy and paste everything in the terminal between the dotted lines directly into the python module"
print "--------------------"
print "{\'" + right_joints[0] + "\'",":",right_angles[0], ","
for i in range(1, len(right_angles) - 1):
    print "\'" + right_joints[i] + "\'",":",right_angles[i],","

print "\'" + right_joints[len(right_angles) - 1] + "\'",":",right_angles[len(right_angles) - 1], "}"
print "--------------------"

quit()
