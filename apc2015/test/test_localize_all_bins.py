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
logging.getLogger('integration.interface.fake').setLevel(logging.WARNING)
logging.getLogger('integration.io.pcd').setLevel(logging.WARNING)
logging.getLogger('integration.camera.client').setLevel(logging.WARNING)
logging.getLogger('integration.camera.packet').setLevel(logging.WARNING)
logging.getLogger('integration.interface.fake').setLevel(logging.WARNING)
logging.getLogger('integration.control_server').setLevel(logging.ERROR)
#logging.getLogger('integration.master').setLevel(logging.WARNING)
logging.getLogger('perception.segmentation.shelf').setLevel(logging.WARNING)
logging.getLogger('perception.segmentation.icp').setLevel(logging.WARNING)
logging.getLogger('perception.segmentation.blob').setLevel(logging.WARNING)
logging.getLogger('planning.control').setLevel(logging.WARNING)

# allow importing from the repository root
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from integration.master import Master

import json
apc_order = json.load(open(os.path.join(os.path.dirname(__file__), 'all_bins_order.json')))

master = Master(apc_order)
from time import sleep

for request in master.order:
    print '  {}: {}'.format(request['bin'], request['item'])

task = master.manager.control.update()
while not task.done: sleep(0.1)

master._move_initial()

while master.order:
    request = master.order.pop(0)

    master._set_target(request['bin'], request['item'])
    master._move_vantage_point(master.knowledge_base.bin_vantage_points[request['bin']][0])
    master._localize_object(master.knowledge_base.bin_vantage_points[request['bin']][0])

    task = master.manager.control.update()
    while not task.done: sleep(0.1)
