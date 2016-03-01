from klampt import vectorops,so3

Q_LEFT_ZEROS = {'left_s0': 0.0, 'left_s1':0.0, 'left_e0':0.0, 'left_e1':0.0, 'left_w0':0.0, 'left_w1':0.0, 'left_w2':0.0}
Q_RIGHT_ZEROS = {'right_s0': 0.0, 'right_s1':0.0, 'right_e0':0.0, 'right_e1':0.0, 'right_w0':0.0, 'right_w1':0.0, 'right_w2':0.0}
Q_SCAN_BIN = {'right_s0': 0.6519418341064454, 'right_s1': -0.008820389520263672, 'right_w0': -0.056373793890380865, 'right_w1': -1.1485681136169434, 'right_w2': 0.24620391617431642, 'right_e0': 1.342233187866211, 'right_e1': 1.5010001992309572}
Q_SPATULA_AT_BIN = {'left_w0': -2.828277074432373, 'left_w1': -1.3989904769531252, 'left_w2': -1.193053556414795, 'left_e0': -1.4894953433349611, 'left_e1': 0.830650595690918, 'left_s0': 0.08053399127197267, 'left_s1': 0.5610534725280762}
Q_SCAN_SPATULA = {'right_s0': 1.0864418917785645, 'right_s1': -0.8402379756042481, 'right_w0': -0.19251458865966797, 'right_w1': 1.5585244787109376, 'right_w2': 0.3367087825561524, 'right_e0': 0.2876213973999024, 'right_e1': 0.8034224367370606}

KLAMPT_MODELS_DIR = "/home/group3/ece490-s2016/apc2015/klampt_models/"

#indices of the left and right cameras in the Baxter robot file
LEFT_CAMERA_LINK_NAME = 'left_hand_camera'
RIGHT_CAMERA_LINK_NAME = 'right_hand_camera'

#indices of the left and right grippers in the Baxter robot file
LEFT_GRIPPER_LINK_NAME = 'left_gripper'
RIGHT_GRIPPER_LINK_NAME = 'right_gripper'

#indices of the left and right arms in the Baxter robot file
LEFT_ARM_LINK_NAMES = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
RIGHT_ARM_LINK_NAMES = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']

#local transformations (rotation, translation pairs) of the grasp center
LEFT_GRIPPER_CENTER_XFORM = (so3.identity(),[0,-0.04,0.1])
RIGHT_GRIPPER_CENTER_XFORM = (so3.identity(),[0,-0.04,0.1])
