import logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)-15s %(filename)s:%(lineno)d %(levelname)s: %(message)s')

# allow importing from the repository root
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from integration.perception_server import PerceptionServer
from integration.interface import PerceptionInterface
from api.shared import KnowledgeBase

from time import sleep

kb = KnowledgeBase()
parallel = False

if parallel:
    ps = PerceptionServer(kb)

    task = ps.localizeOrderBin()
    while task.count_success() < 1 and not task.all_done():
        print 'waiting...'
        sleep(1)
    # cancel tasks if timed out
    task.cancel()

    if task.count_success() >= 1:
        (xform, cloud) = task.get_success()[0].result

        print xform
        print cloud
    else:
        print 'failed'

else:
    p = PerceptionInterface(kb)

    (xform, cloud) = p.localizeSpecificObject('bin', 'object')

    print xform
    print cloud
