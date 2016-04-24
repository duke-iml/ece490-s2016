import math
from klampt import vectorops,so3

# Constants that affect the state machine
INITIAL_STATE = "START"
REAL_VACUUM = False
REAL_PERCEPTION = False
SEGMENT = False
CALIBRATE = False

# Constants related to printing output
PRINT_BLOBS = False
PRINT_LABELS_AND_SCORES = True
VERBOSE = True

# Downsample ratio
STEP = 15

# Right arm configurations
Q_SCAN_BIN_H = [0.979830227142334, -0.8114758358642579, -0.13153885241088867, 1.5163400070922852, 0.0977912751159668, 0.8387039948181153, 0.0]
Q_IK_SEED_H = [0.9019807022460938, -0.8348690428527833, -0.09242234236450196, 2.0056798778686526, 0.04525243319091797, -1.2279516192993165, 0.0]
Q_AFTER_SCAN = [0.5740923092102052, -1.1692768542297365, 0.12923788123168947, 2.5866751006164552, -0.06941263057250976, 0.02185922620239258, 0.0]
Q_AFTER_SCAN2 = [0.22933012752685547, -1.3019661922302248, 0.45674277907104494, 2.476611979211426, -0.21629129084472656, -1.0653496559692384, 0.0]

Q_STOW = [1.0, 0.28, 0.0, 0.9, 0.0, 0.0, 0.0]

# Right arm calibration configs, see pics on Google Drive
Q_CALIBRATE_BIN_E = [1.198805984362793, -0.551082597418213, -0.4406359808166504, 1.0392719826049805, 0.42069423059692385, -0.5196359913024903, -0.14189322271728516]
Q_CALIBRATE_BIN_H = [0.9088836157836915, -0.1096796262084961, -0.46364569260864263, 0.6841554306152344, 0.5288398760192872, -0.3531990760070801, -0.15531555459594729]

# 7 list, each element is a 2 list of lower then upper acceptable bound
ELBOW_UP_BOUNDS = [[-10, 10], [-10, 10], [-.3, .3], [-10, 10], [-.5, .5], [-10, 10], [-10, 10]]

# Number of histograms per object
NUM_HIST_PER_OBJECT = 4

# Left arm configurations
Q_SPATULA_AT_BIN = [0.08053399127197267, 0.5610534725280762, -1.4894953433349611, 0.830650595690918, -2.828277074432373, -1.3989904769531252, 0.0]

# Paths
REPO_ROOT = "/home/group3/ece490-s2016"
KLAMPT_MODELS_DIR = REPO_ROOT + "/apc2015/klampt_models/"
LIBPATH = REPO_ROOT + "/common/"
VACUUM_PCD_FILE = REPO_ROOT + "/group3/planning/custom_vacuum.pcd"
PICK_JSON_PATH = REPO_ROOT + '/group3/integration/apc_pick_task.json'
PERCEPTION_DIR = REPO_ROOT + "/group3/perception"
SHELF_NPZ_FILE = PERCEPTION_DIR + "/shelf.npz"
MAT_PATH = PERCEPTION_DIR + "/matpcl/"
CLOUD_MAT_PATH = MAT_PATH + "cloud.mat"
CHENYU_GO_PATH = MAT_PATH + "chenyugo.txt"
CHENYU_DONE_PATH = MAT_PATH + "chenyudone.txt"
ARDUINO_SERIAL_PORT = "/dev/ttyACM1"
ROS_DEPTH_TOPIC = "/realsense/pc"

# Indices in Baxter robot file link
LEFT_CAMERA_LINK_NAME = 'left_hand_camera'
RIGHT_CAMERA_LINK_NAME = 'right_hand_camera'
LEFT_GRIPPER_LINK_NAME = 'left_gripper'
RIGHT_GRIPPER_LINK_NAME = 'right_gripper'
LEFT_ARM_LINK_NAMES = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
RIGHT_ARM_LINK_NAMES = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']

# Local transform from right_wrist to vacuum point
VACUUM_POINT_XFORM = (so3.identity(), [.07, 0.02, .38])

# Local transform from right_lower_forearm to F200
RIGHT_F200_CAMERA_CALIBRATED_XFORM =([-0.02003573380863745, 0.008438993066848435, -0.9997636484523572, 0.9007673343323835, 0.4340635880118531, -0.014387875521042898, 0.43383957722928207, -0.9008427082228592, -0.01629835179470207], [0.07999999999999981, -0.1, -0.05])

# Transform of shelf relative to world
SHELF_MODEL_XFORM = [1.43,-.23,-.02]

# Distances (meters)
GRASP_MOVE_DISTANCE = .05
BACK_UP_DISTANCE = .2

# Times (seconds)
MOVE_TIME = 2
SCAN_WAIT_TIME = 5 if REAL_PERCEPTION else 0
GRASP_WAIT_TIME = 2 if REAL_VACUUM else 0
