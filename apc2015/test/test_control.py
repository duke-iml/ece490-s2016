import logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)-15s %(filename)s:%(lineno)d %(levelname)s: %(message)s')

# allow importing from the repository root
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from integration.control_server import ControlServer
from integration.interface import ControlInterface
from api.shared import KnowledgeBase, RobotState
from apc.motion import FakeLowLevelController as LowLevelController

from time import sleep
from klampt.robotsim import WorldModel

kb = KnowledgeBase()
parallel = False

if parallel:
    cs = ControlServer(kb)

    task = cs.start()
    while not task.done:
        print 'starting...'
        sleep(1)
    print kb.robot_state.sensed_config

    for plan in [ 'test plan', 'test plan 2' ]:
        task = cs.execute(plan)
        if not task:
            print 'restarting ControlServer'
            cs.start()
            continue

        while not task.done:
            print 'waiting...'
            sleep(1)

        if not task.error:
            print 'success:', task.result
            print kb.robot_state.sensed_config
        else:
            print 'failure:', task.error

    cs.close()

else:
    model = WorldModel().loadRobot('klampt_models/baxter_with_parallel_gripper_col.rob')
    robot = LowLevelController(model)

    kb.robot_state = RobotState(robot)

    c = ControlInterface(robot, kb)
    c.execute('test plan', sleep)
