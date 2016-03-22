import math
from klampt import vectorops,so3

INITIAL_STATE = "START"

# Old configuration for scanning bin when the camera was mounted on the right_lower_forearm rather than the right_lower_elbow
#Q_SCAN_BIN = [0.6519418341064454, -0.008820389520263672, 1.342233187866211, 1.5010001992309572, -0.056373793890380865, -1.1485681136169434, 0.24620391617431642]

# Right arm configurations
Q_INTERMEDIATE_1 = [-0.1606844873474121, -1.2072428786865235, -0.12003399651489259, 2.585908110223389, -0.17487380961914065, -1.5251603966125489, 0.0]
Q_INTERMEDIATE_2 = [0.6722670795227051, -1.1171215075012209, 0.14917963145141602, 2.1640633940368654, -0.34783014325561523, -0.9387962411132813, -0.002684466375732422]
Q_SCAN_BIN = [0.9648739144775391, -0.930742841986084, -0.0007669903930664063, 1.9638789014465334, -0.12962137642822266, 0.17180584804687501, 0.0]

# Left arm configurations
Q_SPATULA_AT_BIN = [0.08053399127197267, 0.5610534725280762, -1.4894953433349611, 0.830650595690918, -2.828277074432373, -1.3989904769531252, 0.0]

# Directories
REPO_ROOT = "/home/group3/ece490-s2016"
KLAMPT_MODELS_DIR = REPO_ROOT + "/apc2015/klampt_models/"
LIBPATH = REPO_ROOT + "/common/"
SHELF_NPZ_FILE = REPO_ROOT + "/group3/perception/shelf.npz"

# Indices in Baxter robot file link
LEFT_CAMERA_LINK_NAME = 'left_hand_camera'
RIGHT_CAMERA_LINK_NAME = 'right_hand_camera'
LEFT_GRIPPER_LINK_NAME = 'left_gripper'
RIGHT_GRIPPER_LINK_NAME = 'right_gripper'
LEFT_ARM_LINK_NAMES = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
RIGHT_ARM_LINK_NAMES = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']

# Local transforms from right_wrist to F200
# Old naive one
RIGHT_F200_CAMERA_XFORM = (so3.mul(so3.rotation([1, 0, 0], math.pi / 2), so3.rotation([0, 0, 1], 3 * math.pi / 2)), [.07, -.1, 0])
    
# Calibrated one thanks to Hayden + William
RIGHT_F200_CAMERA_CALIBRATED_XFORM = ([-0.13894635239615533, -0.9886236474475394, 0.05759509409079229, 0.460160658969247, -0.012955497176371563, 0.8877411351457709, -0.8768957059381646, 0.14985138905074202, 0.45672582815817564], [0.07, -0.1, 0])

# Local transform from right_wrist to vacuum point
VACUUM_POINT_XFORM = (so3.identity(), [.07, 0, .43])

ROS_DEPTH_TOPIC = "/camera/depth/points"

# Time to wait (seconds) for shaking to stop before scanning bin
SCAN_WAIT_TIME = 4