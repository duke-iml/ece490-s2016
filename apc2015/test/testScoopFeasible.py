class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

import logging, traceback
logging.basicConfig(level=logging.DEBUG, format='%(filename)-15s:%(lineno)-4d %(levelname)s: %(message)s')

logging.getLogger('OpenGL').setLevel(99)
logging.getLogger('integration.jobs').setLevel(logging.WARNING)
logging.getLogger('integration.visualization').setLevel(logging.WARNING)
logging.getLogger('integration.interface').setLevel(logging.WARNING)
logging.getLogger('integration.interface.pcd').setLevel(logging.WARNING)
logging.getLogger('integration.interface.real').setLevel(logging.WARNING)
logging.getLogger('integration.interface.fake').setLevel(logging.WARNING)
logging.getLogger('integration.control_server').setLevel(logging.ERROR)
logging.getLogger('integration.master').setLevel(logging.WARNING)
logging.getLogger('planning.control').setLevel(logging.INFO)

# allow importing from the repository root
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from integration.planning_server import PlanningServer
from integration.interface import PlanningInterface, PerceptionInterface
from integration.io import pcd
from api.shared import KnowledgeBase, RobotState
from apc.motion import FakeLowLevelController as LowLevelController

from klampt.robotsim import WorldModel
from time import sleep
from klampt import se3, so3
import math

import json
apc_order = json.load(open(os.path.join(os.path.dirname(__file__), 'example.json')))

knowledge_base_dir = './kb/'


world = WorldModel()
model = world.loadRobot('klampt_models/baxter_with_reflex_gripper_col.rob')
robot = LowLevelController(model)
#initial config
robot.setMilestone([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.1, 0.0, 2.358, 0.0, 0.0, 0.3, -0.98, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.18, 0.0, 2.118, 0.04, 0.0, 0.62, 0.901, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


kb = KnowledgeBase()
kb.robot_state = RobotState(robot)
kb.active_limb = 'left'
kb.bin_contents = apc_order['bin_contents']
kb.target_bin = 'bin_D'
kb.withdraw_point_xforms = json.load(open('kb/withdraw_point_xforms.json'))
kb.bin_bounds = json.load(open('kb/bin_bounds.json'))
# kb.object_xforms['cheezit_bit_original'] = (se3.identity()[0], (0.7, 0.5, 1))


# TODO: Proper shelf transform needs created here
r = PerceptionInterface(kb)

# make fake bin vantage points, need to update to real vantage points at some point
for k in [ 'bin_vantage_points', 'vantage_point_xforms' ]:
    path = os.path.join(knowledge_base_dir, '{}.json'.format(k))
    data = json.load(open(path))
    setattr(kb, k, data)

r = PerceptionInterface(kb)

kb.shelf_xform = r.localizeShelf()
kb.order_bin_xform = r.localizeOrderBin()


p = PlanningInterface(kb)

p.planMoveToInitialPose()

for letter in 'ABCDEFGHIJKL':
	plan = p.planGraspObjectInBin('bin_'+letter,'kong_air_dog_squeakair_tennis_ball')
	if(plan is None):
		print bcolors.FAIL+'=================================bin_'+letter+' failed================================'+bcolors.ENDC
		continue
		
	p.robot.setConfig(plan[0][-1][1][0])
	kb.robot_state.sensed_config = plan[0][-1][1][0]

	kb.grasped_object = 'kong_air_dog_squeakair_tennis_ball'

	plan = p.planMoveObjectToOrderBin('right')
	if plan is None:
		print bcolors.FAIL+'=================================bin_'+letter+' failed================================'+bcolors.ENDC
	else:
		print bcolors.OKGREEN+'+++++++++++++++++++++++++++++++++bin_'+letter+' success++++++++++++++++++++++++++++++++'+bcolors.ENDC