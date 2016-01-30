import logging, traceback
logger = logging.getLogger(__name__)

from time import sleep
from random import random, randint

from klampt import se3

from api.perception import PerceptionInterface
from api.control import ControlInterface
from api.planning import PlanningInterface

from integration.io import pcd

delay_scale = 1
fail_scale = 1

def fake_wait(duration, rate):
        sleep((randint(1, duration) + random())*delay_scale)
        # randomly fail
        return random() <= rate * fail_scale

class FakeControlInterface(ControlInterface):
    def execute(self, plan, sleep):
        # fake plan execution
        self.robot.setMilestone(range(5) + [ random() ])
#         print self.robot.getSensedConfig()
        logger.info('executing plan {}'.format(plan))
        sleep(0.01)
        return fake_wait(5, 0.95)

class FakePerceptionInterface(PerceptionInterface):
    def localizeShelf(self):
        logger.info('localizing shelf')
        if fake_wait(10, 0.95):
            return se3.identity()
        else:
            return None

    def localizeOrderBin(self):
        logger.info('localizing order bin')
        if fake_wait(10, 0.95):
            return se3.identity()
        else:
            return None

    def findAllObjects(self, bin):
        logger.info('finding all objects in {}'.format(bin))
        if fake_wait(10, 0.8):
            import random
            return filter(lambda x: random() < 0.8, self.knowledge_base.bin_contents.get(bin, []))
        else:
            return None

    def localizeSpecificObject(self, bin, object):
        logger.info('localizing {} in {}'.format(bin, object))
        if fake_wait(10, 0.8):
            return (se3.identity(), pcd.read(open('/tmp/expo_dry_erase_board_eraser_bin_D.pcd'))[1], dict(zip(self.knowledge_base.bin_contents[bin], [1.0 / len(self.knowledge_base.bin_contents[bin])] * len(self.knowledge_base.bin_contents))))
        else:
            return None

class FakePlanningInterface(PlanningInterface):
    def planMoveToVantagePoint(self, bin, vantage_point):
        logger.info('planning to view {} from {}'.format(bin, vantage_point))
        if fake_wait(10, 0.9):
            return ('-'.join([ 'plan move', bin, vantage_point ]), 1, 'right', None)
        else:
            return None

    def planGraspObjectInBin(self, bin, object):
        logger.info('planning to grasp {} from {}'.format(object, bin))
        if fake_wait(10, 0.9):
            return ('-'.join([ 'plan grasp', bin, object ]), 1, 'right', 'activeGrasp')
        else:
            return None

    def planMoveObjectToOrderBin(self, limb):
        logger.info('planning to move {} to order bin'.format(limb))
        if fake_wait(10, 0.9):
            return ('-'.join([ 'plan order', limb ]), 1, 'right', None)
        else:
            return None

    def planMoveToInitialPose(self):
        logger.info('planning to initial pose')
        if fake_wait(10, 0.9):
            return ('plan initial', 1, None, None)
        else:
            return None
            
    def planPickUpOrderTray(self, pickup):
        logger.info('planning to pick up order tray')
        if fake_wait(10, 0.9):
            return ('plan pick up tray ' + str(pickup), 1, None, None)
        else:
            return None
    
    def planMoveTrayToOrderBin(self):
        logger.info('planning to move tray to order bin')
        if fake_wait(10, 0.9):
            return ('plan move tray', 1, None, None)
        else:
            return None

