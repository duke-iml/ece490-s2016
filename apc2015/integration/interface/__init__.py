import logging, traceback
logger = logging.getLogger(__name__)

import os

if 'FAKE' in os.environ:
    logger.warn('using fake interfaces by request')

    from fake import FakePlanningInterface as PlanningInterface
    from fake import FakeControlInterface as ControlInterface
    from fake import FakePerceptionInterface as PerceptionInterface

else:

    try:
        from planning.interface import RealPlanningInterface as PlanningInterface
    except ImportError:
        logger.warn('failed to load RealPlanningInterface')
        raise
    else:
        logger.debug('loaded RealPlanningInterface')

    try:
        from planning.control import RealControlInterface as ControlInterface
    except ImportError:
        logger.warn('failed to load RealControlInterface')
        raise
    else:
        logger.debug('loaded RealControlInterface')

    try:
        from perception.interface import RealPerceptionInterface as PerceptionInterface
    except ImportError:
        logger.warn('failed to load RealPerceptionInterface')
        raise
    else:
        logger.debug('loaded RealPerceptionInterface')
