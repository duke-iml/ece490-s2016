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

#Initialize skipping variables
skip_left = False
skip_right = False

#Specify the file names
print "Please specify the limb file names (leave blank to not create a file)"
left_arm_filename = raw_input("Left arm filename:")
right_arm_filename = raw_input("Right arm filename:")

if left_arm_filename == "":
    skip_left = True

if right_arm_filename == "":
    skip_right = True

if not skip_left:
    full_left_filename = "state_json_files/" + left_arm_filename + ".json"
    left_arm = baxter_interface.Limb('left')
    left_joints = left_arm.joint_names()
    left_angles = [left_arm.joint_angle(j) for j in left_joints]
    with open(full_left_filename, "w") as file:
        json.dump({left_joints[i]:left_angles[i] for i in range(0, len(left_angles))}, file, indent=4)

if not skip_right:
    full_right_filename = "state_json_files/" + right_arm_filename + ".json"
    right_arm = baxter_interface.Limb('right')
    right_joints = right_arm.joint_names()
    right_angles = [right_arm.joint_angle(j) for j in right_joints]
    with open(full_right_filename, "w") as file:
        json.dump({right_joints[i]:right_angles[i] for i in range(0, len(right_angles))}, file, indent=4)

quit()
