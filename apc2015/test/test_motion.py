import logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)-15s %(filename)s:%(lineno)d %(levelname)s: %(message)s')

# allow importing from the repository root
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from integration.planning_server import PlanningServer
from integration.control_server import ControlServer
from integration.interface import PlanningInterface, ControlInterface, PerceptionInterface
from api.shared import KnowledgeBase, RobotState
from apc.motion import PhysicalLowLevelController as LowLevelController

from klampt.robotsim import WorldModel
from klampt import se3,so3
from time import sleep

import json
apc_order = json.load(open(os.path.join(os.path.dirname(__file__), 'example.json')))

knowledge_base_dir = './kb/'

world = WorldModel()
model = world.loadRobot('klampt_models/baxter_with_reflex_gripper_col.rob')
print 'before'
robot = LowLevelController(model, 'baxter_with_reflex_gripper_col.rob')
print 'here'

kb = KnowledgeBase()
kb.robot_state = RobotState(robot)
print kb.robot_state
kb.active_limb = 'left'
kb.bin_contents = apc_order['bin_contents']

# make fake bin vantage points, need to update to real vantage points at some point
for k in [ 'bin_vantage_points', 'vantage_point_xforms' ]:
    path = os.path.join(knowledge_base_dir, '{}.json'.format(k))
    data = json.load(open(path))
    setattr(kb, k, data)

r = PerceptionInterface(kb)
c = ControlInterface(robot, kb)
p = PlanningInterface(kb)

kb.shelf_xform = r.localizeShelf()

plan = p.planMoveToInitialPose()
c.execute(plan[0], sleep)
kb.robot_state = RobotState(robot)
sleep(5)

plan = p.planMoveToVantagePoint('bin_A', 'bin_A_center')
c.execute(plan[0], sleep)
kb.robot_state = RobotState(robot)
sleep(5)

kb.object_xforms['crayola_64_ct'] = (so3.rotation([0,0,1], 3.1415/4+3.1415), (1.0, 0.15, 0.70))
plan = p.planGraspObjectInBin('bin_A', 'crayola_64_ct')
c.execute(plan[0], sleep)
# print 'going to control interface'
# print 'made control interface'


print robot.isMoving()

print 'waiting...'
sleep(100)
