from interface import PerceptionInterface
from jobs import BaseJob, JobServer

class PerceptionJob(BaseJob):
    def _get_interface(self):
        return PerceptionInterface

class PerceptionServer(JobServer):
    def __init__(self, knowledge_base, parallelism=1):
        JobServer.__init__(
            self,
            (knowledge_base,),
            [ 'localizeShelf',
              'localizeOrderBin',
              'findAllObjects',
              'localizeSpecificObject' ],
            PerceptionJob,
            parallelism
        )
