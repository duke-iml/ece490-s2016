class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
import numpy
def localizeShelf():
        T = numpy.array([[ 0, -1, 0,    1.03 ],  # distance from front of robot
                         [ 1,  0, 0,  -0.055 ],  # left/right
                         [ 0,  0, 1, -0.9046 ],  # height
                         [ 0,  0, 0,       1 ]])

        return (list(T[:3, :3].T.flat), list(T[:3, 3].flat))

def localizeOrderBin():
    T = numpy.array([[ 1, 0, 0,   0.7 ],
                     [ 0, 1, 0,     0 ],
                     [ 0, 0, 1, -0.90 ],
                     [ 0, 0, 0,     1 ]])

    return (list(T[:3, :3].T.flat), list(T[:3, 3].flat))

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
from integration.interface import PlanningInterface
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

robot.setMilestone([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.1, 0.0, 2.358, 0.0, 0.0, 0.3, -0.98, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.18, 0.0, 2.118, 0.04, 0.0, 0.62, 0.901, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# robot.setMilestone([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0999999999999996, 0.0, 2.358, 0.0, 0.0, 0.3, -0.9800000000000004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06679184745163541, 0.365135941112352, 2.5041869579036335, 1.5646287249338475, 0.8377567543461182, 0.0, -1.097366825056886, 2.493786406695833, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])




kb = KnowledgeBase()
kb.robot_state = RobotState(robot)
kb.active_limb = 'right'
kb.withdraw_point_xforms = json.load(open('kb/withdraw_point_xforms.json'))
kb.bin_bounds = json.load(open('kb/bin_bounds.json'))
# kb.object_xforms['cheezit_bit_original'] = (se3.identity()[0], (0.7, 0.5, 1))


# make fake bin vantage points, need to update to real vantage points at some point
for k in [ 'bin_vantage_points', 'vantage_point_xforms' ]:
    path = os.path.join(knowledge_base_dir, '{}.json'.format(k))
    data = json.load(open(path))
    setattr(kb, k, data)


kb.shelf_xform = localizeShelf()
kb.order_bin_xform = localizeOrderBin()


p = PlanningInterface(kb)

p.planMoveToInitialPose()

# cache planning code for move to bin
for letter in 'ACDFGIJLKHEB':
    plan = p.planMoveToVantagePoint('bin_'+letter, 'bin_'+letter+'_center')
    p.robot.setConfig(plan[0][0][1][-1])
    kb.robot_state.sensed_config = plan[0][0][1][-1]
    # plan = p.makeCachePlans('bin_'+letter, 'bin_'+letter+'_center')

    # path = plan[0]

    # limb = plan[1]
    # #write it out
    # filename = 'planning/cachedPlans/'+limb+'_from_home_to_bin_' + letter + '.json'
    # file = open(filename,'w')
    # json.dump(path,file)
    # file.close()

    # p.planMoveToInitialPose()

# plan = p.planMoveToInitialPose()
# locations = ['home','A','C','D','F','G','I','J','L','K','H','E']
# plan = plan[0][0][1][-1]
# for i,letter in enumerate('ACDFGIJLKHEB'):

#     print bcolors.OKGREEN+'Planing to move from bin_' + locations[i] + ' to bin_' + letter + bcolors.ENDC
#     if(i==0):
#         plan = p.makeCachePlans('bin_'+letter, 'bin_'+letter+'_center',startingConfig=plan)
#     else:
#         plan = p.makeCachePlans('bin_'+letter, 'bin_'+letter+'_center',startingConfig=plan[0][-1])

#     path = plan[0]

#     limb = plan[1]

#     #write it out
#     filename = 'planning/cachedPlans/'+limb+'_from_bin_' + locations[i] + '_to_bin_' + letter + '.json'
#     file = open(filename,'w')
#     json.dump(path,file)
#     file.close()


#go to bin A and bin C

#cache planning code for moving from bin to withdraw point and then to order bin

# for letter in 'CFIL':

# 	if(letter == 'C'):
# 		p.robot.setConfig([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0999999999999996, 0.0, 2.358, 0.0, 0.0, 0.3, -0.9800000000000004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3088284215681351, -0.040525194886402716, 2.990575585189482, 1.3360194679450996, 0.7469085812669755, 0.0, -1.5708000000000002, 2.36765689388818, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# 		kb.robot_state.sensed_config = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0999999999999996, 0.0, 2.358, 0.0, 0.0, 0.3, -0.9800000000000004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3088284215681351, -0.040525194886402716, 2.990575585189482, 1.3360194679450996, 0.7469085812669755, 0.0, -1.5708000000000002, 2.36765689388818, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# 		kb.target_bin = 'bin_C'
# 	elif(letter == 'F'):
# 		#bin F scooped item
# 		p.robot.setConfig([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0999999999999996, 0.0, 2.358, 0.0, 0.0, 0.3, -0.9800000000000004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.30158718316595046, 0.1653559427347274, 2.2363132059607045, 1.834400059027026, 1.0639366601020779, 0.0, -1.0132113159072906, 2.7856916491114374, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# 		kb.robot_state.sensed_config = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0999999999999996, 0.0, 2.358, 0.0, 0.0, 0.3, -0.9800000000000004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.30158718316595046, 0.1653559427347274, 2.2363132059607045, 1.834400059027026, 1.0639366601020779, 0.0, -1.0132113159072906, 2.7856916491114374, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# 		kb.target_bin = 'bin_F'
# 	elif(letter == 'I'):
# 		p.robot.setConfig([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0999999999999996, 0.0, 2.358, 0.0, 0.0, 0.3, -0.9800000000000004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.48758930277672796, -0.4791435220152396, 1.3019986072291876, 2.0261847241170328, 3.059, 0.0, 0.38477484250099353, 1.3669227770014718, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# 		kb.robot_state.sensed_config = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0999999999999996, 0.0, 2.358, 0.0, 0.0, 0.3, -0.9800000000000004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.48758930277672796, -0.4791435220152396, 1.3019986072291876, 2.0261847241170328, 3.059, 0.0, 0.38477484250099353, 1.3669227770014718, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# 		kb.target_bin = 'bin_I'
# 	elif(letter == 'L'):
# 		p.robot.setConfig([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0999999999999996, 0.0, 2.358, 0.0, 0.0, 0.3, -0.9800000000000004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5459489486965996, 0.3966207137077951, 1.2334197022368156, 1.7995842493636613, 2.806168682648472, 0.0, 0.3290612094497131, 0.7556578376322631, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# 		kb.robot_state.sensed_config = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0999999999999996, 0.0, 2.358, 0.0, 0.0, 0.3, -0.9800000000000004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5459489486965996, 0.3966207137077951, 1.2334197022368156, 1.7995842493636613, 2.806168682648472, 0.0, 0.3290612094497131, 0.7556578376322631, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# 		kb.target_bin = 'bin_L'

# 	tuple = p.planMoveObjectToOrderBin('right')
# 	if(tuple is not None):
# 		plan = tuple[0]
# 		#write it out
# 		filename = 'planning/cachedPlans/right_from_bin_' + letter + '_to_order_bin.json'
# 		file = open(filename,'w')
# 		json.dump(plan,file)

