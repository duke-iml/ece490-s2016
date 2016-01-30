import logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)-15s %(filename)s:%(lineno)d %(levelname)s: %(message)s')

logging.getLogger('OpenGL').setLevel(99)
logging.getLogger('integration.jobs').setLevel(logging.WARNING)
logging.getLogger('integration.visualization').setLevel(logging.WARNING)
logging.getLogger('integration.interface').setLevel(logging.WARNING)
logging.getLogger('integration.interface.fake').setLevel(logging.WARNING)
logging.getLogger('integration.io.pcd').setLevel(logging.WARNING)
logging.getLogger('integration.camera.client').setLevel(logging.WARNING)
logging.getLogger('integration.camera.packet').setLevel(logging.WARNING)
logging.getLogger('integration.interface.fake').setLevel(logging.WARNING)
logging.getLogger('integration.control_server').setLevel(logging.WARNING)
logging.getLogger('integration.master').setLevel(logging.INFO)
logging.getLogger('perception.shelf').setLevel(logging.WARNING)
logging.getLogger('perception.icp').setLevel(logging.WARNING)
logging.getLogger('perception.blob').setLevel(logging.WARNING)

# allow importing from the repository root
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from integration.master import Master

import json
apc_order = json.load(open(os.path.join(os.path.dirname(__file__), 'test_shelf.json')))

# speed up the tests
from integration.interface import fake
fake.delay_scale = 1
fake.fail_scale = 100

m = Master(apc_order)

while m.order:
    request = m.order.pop(0)
    if not m.run(request['bin'], request['item']):
        m.order.append(request)
