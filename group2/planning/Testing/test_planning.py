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
import sys, os, csv
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from integration.planning_server import PlanningServer
from integration.interface import PlanningInterface, PerceptionInterface
from integration.io import pcd
from api.shared import KnowledgeBase, RobotState
from apc.motion import FakeLowLevelController as LowLevelController

from klampt.robotsim import WorldModel
from time import sleep
from klampt import se3, so3, resource
from integration import visualization
import math

import json
apc_order = json.load(open(os.path.join(os.path.dirname(__file__), 'example.json')))

knowledge_base_dir = './kb/'


world = WorldModel()
model = world.loadRobot('klampt_models/baxter_with_reflex_gripper_col.rob')
robot = LowLevelController(model)
#initial config
robot.setMilestone([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.1, 0.0, 2.358, 0.0, 0.0, 0.3, -0.98, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.18, 0.0, 2.118, 0.04, 0.0, 0.62, 0.901, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

#bin f config
# robot.setMilestone([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0999999999999996, 0.0, 2.358, 0.0, 0.0, 0.3, -0.9800000000000004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.30158718316595046, 0.1653559427347274, 2.2363132059607045, 1.834400059027026, 1.0639366601020779, 0.0, -1.0132113159072906, 2.7856916491114374, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


kb = KnowledgeBase()
kb.robot_state = RobotState(robot)
kb.active_limb = 'left'
kb.bin_contents = apc_order['bin_contents']
kb.withdraw_point_xforms = json.load(open('kb/withdraw_point_xforms.json'))
kb.bin_bounds = json.load(open('kb/bin_bounds.json'))

try:
    reader = csv.reader(open(os.path.join(knowledge_base_dir,'Object Dimensions.csv')))
    header = None
    for line in reader:
        if header == None:
            header = line
        else:
            kb.object_minimum_dims[line[0]] = float(line[1])*0.01
            kb.object_maximum_dims[line[0]] = float(line[2])*0.01
            kb.object_median_dims[line[0]] = float(line[3])*0.01
except IOError:
    logger.info("Object dimensions unknown, try filling in Object Dimensions.csv")
    pass
# kb.object_xforms['cheezit_bit_original'] = (se3.identity()[0], (0.7, 0.5, 1))


# TODO: Proper shelf transform needs created here
r = PerceptionInterface(kb)

# make fake bin vantage points, need to update to real vantage points at some point
for k in [ 'bin_vantage_points', 'vantage_point_xforms' ]:
    path = os.path.join(knowledge_base_dir, '{}.json'.format(k))
    data = json.load(open(path))
    setattr(kb, k, data)
#test: visualize result
#for v in kb.vantage_point_xforms:
#	(save,result) = resource.edit(v,kb.vantage_point_xforms[v],'RigidTransform',description="vantage point for bin "+v,world=world)

r = PerceptionInterface(kb)

kb.shelf_xform = r.localizeShelf()
kb.order_bin_xform = r.localizeOrderBin()
#kb.object_xforms['crayola_64_ct'] = (so3.rotation([0,0,1], 3.1415/4+3.1415), (1.0, 0.15, 0.70))

# kb.object_xforms['kong_sitting_frog_dog_toy'] = se3.identity()
# kb.object_clouds['kong_sitting_frog_dog_toy'] = pcd.read(open('/tmp/kong_sitting_frog_dog_toy_bin_C.pcd'))[1]


kb.object_xforms['safety_works_safety_glasses'] = se3.identity()
kb.object_clouds['safety_works_safety_glasses'] = pcd.read(open('/tmp/safety_works_safety_glasses_bin_G.pcd'))[1]


kb.object_xforms['stanley_66_052'] = se3.identity()
kb.object_clouds['stanley_66_052'] = pcd.read(open('/tmp/stanley_66_052_bin_F.pcd'))[1]


p = PlanningInterface(kb)

p.planMoveToInitialPose()


kb.target_bin = 'bin_F'

plan = p.planGraspObjectInBin('bin_F','stanley_66_052')
plan = plan[0]
paths = [ p1[1] for p1 in plan if p1[0] in [ 'path', 'fast_path' ] ]
visualization.debug_plan(kb, sum(paths, [ kb.robot_state.sensed_config ]))
p.robot.setConfig(paths[-1][-1])
kb.robot_state.commanded_config = paths[-1][-1]


plan = p.planMoveObjectToOrderBin('right')[0]
paths = [ p1[1] for p1 in plan if p1[0] in [ 'path', 'fast_path' ] ]
visualization.debug_plan(kb, sum(paths, [ kb.robot_state.commanded_config ]))

#plan = p.failsafeplan('bin_G','safety_works_safety_glasses')[0]
#paths = [ p1[1] for p1 in plan if p1[0] in [ 'path', 'fast_path' ] ]
#p.robot.setConfig(paths[-1][-1])
#kb.robot_state.sensed_config = paths[-1][-1]
#visualization.debug_plan(kb, sum(paths, [ kb.robot_state.sensed_config ]))
#kb.grasped_object = 'safety_works_safety_glasses'


#plan = p.planMoveObjectToOrderBin('left')

