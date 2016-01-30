from interface import PlanningInterface
from jobs import BaseJob, JobServer

class PlanningJob(BaseJob):
    def _get_interface(self):
        return PlanningInterface

class PlanningServer(JobServer):
    def __init__(self, knowledge_base, parallelism=10):
        JobServer.__init__(
            self,
            (knowledge_base,),
            [ 'planMoveToVantagePoint',
              'planGraspObjectInBin',
              'planMoveObjectToOrderBin',
              'planMoveToInitialPose',
              'planMoveToXform',
              'planPickUpOrderTray',
              'planMoveTrayToBin',
              'planMoveTrayToOrderBin'],
            PlanningJob,
            parallelism
        )
