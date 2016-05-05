#contains constants for Baxter
from klampt import vectorops,so3
import math

#indices of the left and right cameras in the Baxter robot file
left_camera_link_name = 'left_hand_camera'
right_camera_link_name = 'right_hand_camera'

#indices of the left and right grippers in the Baxter robot file
left_gripper_link_name = 'left_gripper'
right_gripper_link_name = 'right_gripper'

left_arm_geometry_indices = [15,16,17,18,19,21,22]
right_arm_geometry_indices = [35,36,37,38,39,41,42]
# left_hand_geometry_indices = [54,55,56,57]
left_hand_geometry_indices = [54]
right_hand_geometry_indices = [59,60]


#indices of the left and right arms in the Baxter robot file
left_arm_link_names = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
right_arm_link_names = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']


# gripper
# left_gripper_center_xform = (so3.from_axis_angle(((0,0,1),math.pi*0.5)),[0,0.0,0.11])
right_gripper_center_xform = (so3.from_axis_angle(((0,0,1),math.pi*0.5)),[0,0.0,0.11])

#spatula
left_gripper_center_xform = (so3.from_axis_angle(((0,0,1),math.pi*0.5)),[0.025,0,0.31])


def set_model_gripper_command(robot,limb,command,spatulaPart = None):
    """Given the Baxter RobotModel 'robot' at its current configuration,
    this will set the configuration so the gripper on limb 'limb' is
    placed at the gripper command values 'command'.

    Currently handles rethink parallel jaw gripper commands, range [0] (closed)
    to [1] (open).
    """
    value = command[0]
    print limb, spatulaPart
    if limb=='left':
        # print "Opening left gripper to",value
        if spatulaPart == 1:
            robot.driver(15).setValue(value*0.4) # wide base
        elif spatulaPart == 2:
            robot.driver(16).setValue(value*0.385) # narrow base
        elif spatulaPart == 3:
            robot.driver(17).setValue(value*0.375) # fence
    else:
        # print "Opening right gripper to",value
        robot.driver(18).setValue(value*0.03)
        robot.driver(19).setValue(-value*0.03)
