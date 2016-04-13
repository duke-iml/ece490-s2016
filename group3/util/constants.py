import math
from klampt import vectorops,so3

INITIAL_STATE = "SCANNING_BIN"

REAL_PERCEPTION = True
REAL_VACUUM = False
SHOW_PLOT = False
CALIBRATE = False
REAL_PLANNING = False


# Right arm configurations
Q_INTERMEDIATE_1 = [0.6, -1.1, 0.0, 2.3, 0.0, -.9, 0.0]
Q_INTERMEDIATE_2 = [0.8, -1.1, 0.0, 2.3, 0.0, 0.0, 0.0]
Q_SCAN_BIN = [0.9844321695007325, -0.9165535197143555, -0.16336895372314456, 1.2858593939758303, 0.08973787598876953, 1.3334127983459474, 0.0]
Q_STOW = [1.0, 0.28, 0.0, 0.9, 0.0, 0.0, 0.0]

GRASP_INTERMEDIATE1 = [ 0.9767622655700684, -0.19865051180419924, -0.15531555459594729,  1.73838372588501, -0.07171360175170899,  -0.24773789696044923, 0.019174759826660157]
GRASP_INTERMEDIATE2 = [ 0.5963350306091308,  -1.0519273240905762, -0.122718462890625, 0.22127672839965823, 2.284097390551758, -1.2824079372070314, 0.04947088035278321]

# Constants for IK - used to constrain solutions
IK_OFFSET = 0.8

# 
SHELF_MODEL_XFORM = [1.5,.12,-.08]


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
#RIGHT_F200_CAMERA_CALIBRATED_XFORM = ([-0.13894635239615533, -0.9886236474475394, 0.05759509409079229, 0.08235676176722337, 0.04642207640971825, 0.9955211472455158, -0.986869431559728, 0.1430673775859965, 0.07496966402249436], [0.07, -0.1, 0])
RIGHT_F200_CAMERA_CALIBRATED_XFORM =([0.06999104199626158, 0.051730884285471124, -0.9962053852752061, 0.9863560672880546, 0.145583039455246, 0.07685887812748585, 0.14900658564077301, -0.9879926589980396, -0.04083556295269509], [0.039999999999999834, -0.1, 0])

# Local transform from right_wrist to vacuum point
#VACUUM_POINT_XFORM = (so3.identity(), [.07, 0, .43])
VACUUM_POINT_XFORM = (so3.identity(), [.083+0.0635, 0, .32])

# Sys admin stuff
ARDUINO_SERIAL_PORT = "/dev/ttyACM0"
ROS_DEPTH_TOPIC = "/realsense/pc"

# Distances (meters)
GRASP_MOVE_DISTANCE = .03
BACK_UP_DISTANCE = .2

# Times (seconds)
MOVE_TIME = 2
SCAN_WAIT_TIME = 4 if REAL_PERCEPTION else 0
GRASP_WAIT_TIME = 2 if REAL_VACUUM else 0



