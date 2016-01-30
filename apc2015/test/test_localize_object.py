import logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)-15s %(filename)s:%(lineno)d %(levelname)s: %(message)s')

# allow importing from the repository root
import sys, os, readline
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from integration.master import Master
from integration.visualization import debug_cloud

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

from klampt import se3

master = Master(apc_order)
from time import sleep

for request in master.order:
    print request['bin'], request['item']

master._set_target('bin_A', 'crayola_64_ct')
# master._set_target('bin_B', 'kong_duck_dog_toy')
master._localize_shelf_and_order_bin()
# master._localize_object('unused')
master.knowledge_base.active_limb = 'left'
# master.knowledge_base.grasped_object = 'crayola_64_ct'
# master.knowledge_base.active_grasp = [[0.8449032475481839, 0.5332477717385703, -0.0422530024777488, -0.053105482443014856, 0.005018582272109273, -0.9985762973185823, -0.532276535286903, 0.8459442226103591, 0.03255859663963079], [0.1039670743637746, -0.1590815806021303, 0.07642602363877261]]

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

        elif cmd == 'localize' and len(args) >= 1:
            if args[0] in [ 'shelf', 'order_bin', 'both' ]:
                print master._localize_shelf_and_order_bin()
            elif args[0] == 'object':
                print master._localize_object('unused')

        elif cmd == 'robot' and len(args) == 1:
            if args[0] == 'enable':
                os.system('rosrun baxter_tools enable_robot.py -e')
            elif args[0] == 'disable':
                os.system('rosrun baxter_tools enable_robot.py -d')

        elif cmd == 'reset' and len(args) == 1:
            if args[0] == 'control':
                print master.manager._restart_control()
            elif args[0] == 'order':
                print master._load_apc_order(apc_order)
            elif args[0] == 'kb':
                print master._create_knowledge_base()
                print master._load_apc_order(apc_order)

        elif cmd in [ 'quit', 'exit' ]:
            break

    except Exception as e:
        print 'excution error:', e

    task = master.manager.control.update()
    while not task.done: sleep(0.1)
