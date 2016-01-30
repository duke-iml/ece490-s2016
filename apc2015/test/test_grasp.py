import logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)-15s %(filename)s:%(lineno)d %(levelname)s: %(message)s')

# allow importing from the repository root
import sys, os, readline
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from integration.master import Master
from integration.visualization import debug_world

logging.getLogger('OpenGL').setLevel(99)
logging.getLogger('integration.jobs').setLevel(logging.WARNING)
logging.getLogger('integration.visualization').setLevel(logging.WARNING)
logging.getLogger('integration.interface.pcd').setLevel(logging.WARNING)
logging.getLogger('integration.interface.real').setLevel(logging.WARNING)
logging.getLogger('integration.interface.fake').setLevel(logging.WARNING)
logging.getLogger('integration.control_server').setLevel(logging.ERROR)
logging.getLogger('integration.master').setLevel(logging.INFO)
logging.getLogger('planning.control').setLevel(logging.INFO)

import json
apc_order = json.load(open(os.path.join(os.path.dirname(__file__), 'top_shelf.json')))

from klampt import se3, so3, vectorops
from klampt.robotsim import WorldModel

master = Master(apc_order)
from time import sleep

from planning.planner import LimbPlanner
import baxter_scoop as baxter

# master._set_target('bin_A', 'crayola_64_ct')
# master._set_target('bin_B', 'kong_duck_dog_toy')
master._localize_shelf_and_order_bin()
# master._localize_object('unused')
# master.knowledge_base.active_limb = 'left'
# master.knowledge_base.grasped_object = 'crayola_64_ct'
# master.knowledge_base.active_grasp = [[0.8449032475481839, 0.5332477717385703, -0.0422530024777488, -0.053105482443014856, 0.005018582272109273, -0.9985762973185823, -0.532276535286903, 0.8459442226103591, 0.03255859663963079], [0.1039670743637746, -0.1590815806021303, 0.07642602363877261]]

task = master.manager.control.update()
while not task.done: sleep(0.1)

kb = master.knowledge_base

world = WorldModel()
robot = world.loadRobot('klampt_models/baxter_with_reflex_gripper_col.rob')

shelf = world.loadRigidObject('klampt_models/north_shelf/shelf_with_bins.obj')
shelf.setTransform(*kb.shelf_xform)

order_bin = world.loadRigidObject('klampt_models/north_order_bin/order_bin.obj')
order_bin.setTransform(*kb.order_bin_xform)

while True:
    print '{} >'.format(
        # master.knowledge_base.active_limb or '-',
        # master.knowledge_base.target_bin or '-',
        master.knowledge_base.target_object or '-',
    ),

    try:
        parts = str(raw_input()).split(' ')
    except (EOFError, KeyboardInterrupt):
        print
        break

    try:
        
        cmd = parts[0]
        args = parts[1:]

        if cmd == 'load' and len(args) == 1:
            kb.target_object = args[0]
            kb.object_xforms[kb.target_object] = se3.identity()

        elif cmd == 'move' and len(args) <= 3:
            (x, y, z) = map(float, args) + ([0] * (3 - len(args)))
            xform = kb.object_xforms[kb.target_object]
            kb.object_xforms[kb.target_object] = (xform[0], [ x, y, z ])

        elif cmd == 'mover' and len(args) <= 3:
            (x, y, z) = map(float, args) + ([0] * (3 - len(args)))
            xform = kb.object_xforms[kb.target_object]
            kb.object_xforms[kb.target_object] = (xform[0], vectorops.add(xform[1], [ x, y, z ]))

        elif cmd == 'rot' and len(args) <= 3:
            (rx, ry, rz) = map(lambda a: float(a) * 3.141592/180, args) + ([0] * (3 - len(args)))
            xform = kb.object_xforms[kb.target_object]
            kb.object_xforms[kb.target_object] = (
                reduce(so3.mul, [ so3.rotation(v, a) for (v, a) in zip([ [1,0,0], [0,1,0], [0,0,1] ], [ rx, ry, rz ]) ]),
                xform[1]
            )

        elif cmd == 'rotl' and len(args) <= 3:
            (rx, ry, rz) = map(float, args) + ([0] * (3 - len(args)))
            xform = kb.object_xforms[kb.target_object]
            kb.object_xforms[kb.target_object] = (
                reduce(so3.mul, [ xform[0] ] + [ so3.rotation(v, a) for (v, a) in zip([ [1,0,0], [0,1,0], [0,0,1] ], [ rx, ry, rz ]) ]),
                xform[1]
            )

        elif cmd == 'check':
            while not master.manager.control.update().done: sleep(0.1)
            while not master.manager.control.update().done: sleep(0.1)
            robot.setConfig(kb.robot_state.sensed_config)
            debug_world(world)

            planner = LimbPlanner(world, None)
            if planner.check_collision_free('left', verbose=True):
                print 'no collisions'

        elif cmd == 'save' and len(args) == 1:
            name = args[0]
            #path = os.path.join(os.path.expanduser('~'), 'ece590-s2015','klampt_models','items',kb.target_object,'grasps','reflex', '{}.json'.format(name))

            #if os.path.exists(path):
            #    print 'Save file', path, 'already exists. Overwrite? [y/N]',
            #    prompt = raw_input()
            #    if prompt.lower() != 'y':
            #        continue

            #while not master.manager.control.update().done: sleep(0.1)
            #while not master.manager.control.update().done: sleep(0.1)
            robot.setConfig(kb.robot_state.sensed_config)
            #debug_world(world)

            #object_xform = kb.object_xforms[kb.target_object]
            gripper_xform = se3.mul(robot.getLink('left_gripper').getTransform(), baxter.left_gripper_center_xform)
	    print gripper_xform
            grasp = {
                'name': name,
                'grasp_xform': se3.mul(se3.inv(object_xform), gripper_xform),
                'gripper_close_command': [0.8, 0.8, 0.0, 0.1],
                'gripper_open_command': [0.3, 0.3, 0.0, 0.1]
            }

            print json.dumps(grasp, indent=4)

            json.dump(grasp, open(path, 'w'), indent=4)

        elif cmd == 'gripper' and len(args) == 1:
            task = None

            if args[0] == 'parallel':
                task = master.manager.control.execute([
                    ( 'left_gripper', [0.5, 0.5, 0.5, 1], 0, 0),
                    ( 'right_gripper', [0.5, 0.5, 0.5, 1], 0, 0),
                ])
            elif args[0] == 'pinch':
                task = master.manager.control.execute([
                    ( 'left_gripper', [0.3, 0.3, 0.0, 0.1], 0, 0),
                    ( 'right_gripper', [0.3, 0.3, 0.0, 0.1], 0, 0),
                ])
            elif args[0] == 'unpinch':
                task = master.manager.control.execute([
                    ( 'left_gripper', [0.6, 0.6, 0.0, 0.1], 0, 0),
                    ( 'right_gripper', [0.6, 0.6, 0.0, 0.1], 0, 0),
                ])
            elif args[0] == 'open':
                task = master.manager.control.execute([
                    ( 'left_gripper', [0.8, 0.8, 0.8, 1], 0, 0),
                    ( 'right_gripper', [0.8, 0.8, 0.8, 1], 0, 0),
                ])
            elif args[0] == 'close':
                task = master.manager.control.execute([
                    ( 'left_gripper', [0.2, 0.2, 0.2, 1], 0, 0),
                    ( 'right_gripper', [0.2, 0.2, 0.2, 1], 0, 0),
                ])

            while task and not task.done:
                sleep(0.1)
        elif cmd == 'gripper' and len(args) == 4:
                q = [ float(a) for a in args ]
                task = master.manager.control.execute([
                    ( 'left_gripper', q, 0, 0),
                    ( 'right_gripper', q, 0, 0),
                ])
                while not task.done:
                    sleep(0.1)

        elif cmd == 'robot' and len(args) == 1:
            if args[0] == 'enable':
                os.system('rosrun baxter_tools enable_robot.py -e')
            elif args[0] == 'disable':
                os.system('rosrun baxter_tools enable_robot.py -d')

        elif cmd == 'reset' and len(args) == 1:
            if args[0] == 'control':
                print master.manager._restart_control()

        elif cmd in [ 'quit', 'exit' ]:
            break

    except Exception as e:
        # print 'excution error:', e
        raise

    task = master.manager.control.update()
    while not task.done: sleep(0.1)
