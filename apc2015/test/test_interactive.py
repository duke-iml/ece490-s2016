import logging, traceback, sys
from rainbow_logging_handler import RainbowLoggingHandler
#logging.basicConfig(level=logging.DEBUG, format='%(filename)-15s:%(lineno)-4d %(levelname)s: %(message)s')
handler = RainbowLoggingHandler(sys.stderr)
handler.setFormatter(logging.Formatter('%(filename)s:%(lineno)d\t%(levelname)s: %(message)s'))
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
logger.addHandler(handler)

logging.getLogger('OpenGL').setLevel(99)
logging.getLogger('integration.jobs').setLevel(logging.WARNING)
logging.getLogger('integration.visualization').setLevel(logging.WARNING)
logging.getLogger('integration.interface').setLevel(logging.WARNING)
#logging.getLogger('integration.interface.fake').setLevel(logging.WARNING)
logging.getLogger('integration.io.pcd').setLevel(logging.WARNING)
logging.getLogger('integration.camera.client').setLevel(logging.WARNING)
logging.getLogger('integration.camera.packet').setLevel(logging.WARNING)
logging.getLogger('integration.interface.fake').setLevel(logging.WARNING)
logging.getLogger('integration.control_server').setLevel(logging.ERROR)
logging.getLogger('integration.master').setLevel(logging.WARNING)
logging.getLogger('perception.segmentation.shelf').setLevel(logging.WARNING)
logging.getLogger('perception.segmentation.icp').setLevel(logging.WARNING)
logging.getLogger('perception.segmentation.blob').setLevel(logging.WARNING)
logging.getLogger('planning.control').setLevel(logging.WARNING)

# allow importing from the repository root
import sys, os, readline
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from integration.master import Master

from integration.io import pcd

import json
apc_order = json.load(open(os.path.join(os.path.dirname(__file__), 'example2.json')))

from klampt import se3, so3, vectorops

master = Master(apc_order)
from time import sleep

for request in master.order:
    print '  {}: {}'.format(request['bin'], request['item'])

# master._set_target('bin_B', 'genuine_joe_plastic_stir_sticks')
#master._set_target('bin_F','stanley_66_052')
master._set_target('bin_C','champion_copper_plus_spark_plug')
#master._set_target('bin_E','mead_index_cards')
#master._set_target('bin_A', 'elmers_washable_no_run_school_glue')
# master._set_target('bin_B', 'kong_duck_dog_toy')
# master._set_target('bin_F', 'mead_index_cards')
#master._set_target('bin_F', 'kong_sitting_frog_dog_toy')
#master._set_target('bin_F', 'champion_copper_plus_spark_plug')
# master._set_target('bin_C', 'first_years_take_and_toss_straw_cups')
# master.knowledge_base.object_xforms['crayola_64_ct'] = ( so3.mul(so3.rotation([1,0,0], 3.1415), so3.rotation([0,0,1], 3.1415/4+3.1415)), (0.995, 0.27, 0.84))
master.knowledge_base.active_limb = 'right'
master._localize_shelf_and_order_bin()
#master._localize_object('unused')
# master.knowledge_base.grasped_object = 'champion_copper_plus_spark_plug'
# master.knowledge_base.object_xforms['first_years_take_and_toss_straw_cups'] = se3.identity()
# master.knowledge_base.object_clouds['first_years_take_and_toss_straw_cups'] = pcd.read(open('/tmp/first_years_take_and_toss_straw_cups_bin_C.pcd'))[1]
# master.knowledge_base.object_xforms['safety_works_safety_glasses'] = se3.identity()
# master.knowledge_base.object_clouds['safety_works_safety_glasses'] = pcd.read(open('/tmp/safety_works_safety_glasses_bin_G.pcd'))[1]
#master.knowledge_base.object_xforms['mead_index_cards'] = se3.identity()
#master.knowledge_base.object_clouds['mead_index_cards'] = pcd.read(open('/tmp/mead_index_cards_bin_E.pcd'))[1]
# master.knowledge_base.object_xforms['champion_copper_plus_spark_plug'] = se3.identity()
# master.knowledge_base.object_clouds['champion_copper_plus_spark_plug'] = pcd.read(open('/tmp/champion_copper_plus_spark_plug_bin_C.pcd'))[1]

task = master.manager.control.update()
while not task.done: sleep(0.1)

while True:
    print '{} {} {} >'.format(
        master.knowledge_base.active_limb or '-',
        master.knowledge_base.target_bin or '-',
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

        if cmd == 'active' and len(args) == 1:
            master.knowledge_base.active_limb = args[0]

        elif cmd == 'target':
            if len(args) == 2:
                print master._set_target(args[0], args[1])
            elif len(args) == 1:
                print master._set_target(args[0], filter(lambda o: o['bin'] == args[0], master.order)[0]['item'])

        elif cmd == 'move' and len(args) >= 1:
            if args[0] == 'initial':
                print master._move_initial()
                sleep(2)
            elif args[0] == 'vantage' and len(args) >= 2:
                print master._move_vantage_point(args[1])
                sleep(2)
            elif args[0] == 'withdraw' and len(args) >= 1:
                name = args[1] if len(args) >= 2 else master.knowledge_base.target_bin
                limb = args[2] if len(args) >= 3 else master.knowledge_base.active_limb
                print master.manager.moveToXform(se3.mul(master.knowledge_base.shelf_xform, master.knowledge_base.withdraw_point_xforms[name]), limb)
                sleep(2)
            elif args[0] == 'pickTray':
                print master.manager.PickUpOrderTray(True)
            elif args[0] == 'placeTray':
                print master.manager.PickUpOrderTray(False)
            elif args[0] == 'moveTray' and len(args) > 1:
                if(args[1].startswith('bin')):
                    print master.manager.MoveTrayToBin(args[1])
                elif(args[1].startswith('order')):
                    print master.manager.MoveTrayToOrderBin()
            else:
                print master._move_vantage_point(args[0])
                sleep(2)

        elif cmd == 'localize' and len(args) >= 1:
            if args[0] in [ 'shelf', 'order_bin', 'both' ]:
                print master._localize_shelf_and_order_bin()
            elif args[0] == 'object':
                print master._localize_object('unused')

        elif cmd == 'find':
            print master._find_object()

        elif cmd == 'grasp':
            print master._grasp_object()

        elif cmd == 'retrieve':
            print master._retrieve_object()

        elif cmd == 'move_cartesian' and len(args) == 4:
            task = None
            limb = args[0]
            amount = [float(args[1]),float(args[2]),float(args[3])]
            maxspeed = 0.01
            command = {'limb':limb,'velocity':vectorops.div(amount,vectorops.norm(amount)/maxspeed),'duration':vectorops.norm(amount)/maxspeed}
            print "Trying cartesian_drive low level command"
            task = master.manager.control.execute([
                ( 'cartesian_drive', command, 0, 0),
            ])

            while task and not task.done:
                sleep(0.1)

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

        # elif cmd == 'collide' and len(args) == 1:
        #     if args[0] == 'enable':
        #         motion.robot.left_limb.enableSelfCollisionAvoidance(True)
        #         motion.robot.right_limb.enableSelfCollisionAvoidance(True)
        #         print 'Baxter self-collision avoidance enabled'
        #     elif args[0] == 'disable':
        #         motion.robot.left_limb.enableSelfCollisionAvoidance(False)
        #         motion.robot.right_limb.enableSelfCollisionAvoidance(False)
        #         print '!!! Baxter self-collision avoidance disabled'
        
        elif cmd == 'undo':
            master._undo_failed_retrieve()
            
        elif cmd == 'shelf' and len(args) > 1:
            cmd = args[0]
            args = args[1:]

            if cmd == 'move' and len(args) <= 3:
                (x, y, z) = map(float, args) + ([0] * (3 - len(args)))
                xform = master.knowledge_base.shelf_xform
                master.knowledge_base.shelf_xform = (xform[0], [ x, y, z ])

            elif cmd == 'mover' and len(args) <= 3:
                (x, y, z) = map(float, args) + ([0] * (3 - len(args)))
                xform = master.knowledge_base.shelf_xform
                master.knowledge_base.shelf_xform = (xform[0], vectorops.add(xform[1], [ x, y, z ]))

            elif cmd == 'rot' and len(args) <= 3:
                (rx, ry, rz) = map(lambda a: float(a) * 3.141592/180, args) + ([0] * (3 - len(args)))
                xform = master.knowledge_base.shelf_xform
                master.knowledge_base.shelf_xform = (
                    reduce(so3.mul, [ so3.rotation(v, a) for (v, a) in zip([ [1,0,0], [0,1,0], [0,0,1] ], [ rx, ry, rz ]) ]),
                    xform[1]
                )

            elif cmd == 'rotl' and len(args) <= 3:
                (rx, ry, rz) = map(float, args) + ([0] * (3 - len(args)))
                xform = master.knowledge_base.shelf_xform
                master.knowledge_base.shelf_xform = (
                    reduce(so3.mul, [ xform[0] ] + [ so3.rotation(v, a) for (v, a) in zip([ [1,0,0], [0,1,0], [0,0,1] ], [ rx, ry, rz ]) ]),
                    xform[1]
                )

            print so3.matrix(master.knowledge_base.shelf_xform[0]), master.knowledge_base.shelf_xform[1]

        elif cmd == 'reset' and len(args) == 1:
            if args[0] == 'control':
                print master.manager._restart_control()
            elif args[0] == 'order':
                print master._load_apc_order(apc_order)
            elif args[0] == 'kb':
                print master._create_knowledge_base()
                print master._load_apc_order(apc_order)

        elif cmd == 'run' and len(args) == 2:
            print master.run(args[0], args[1])
            for request in master.order:
                print '  {}: {}'.format(request['bin'], request['item'])

        elif cmd in [ 'quit', 'exit' ]:
            break
        else:
            print "Invalid command",cmd

    except Exception as e:
        print 'excution error:', e
        print traceback.format_exc()

    task = master.manager.control.update()
    while not task.done: sleep(0.1)
