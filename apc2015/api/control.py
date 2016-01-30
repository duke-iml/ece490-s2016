class ControlInterface:
    '''
    Interface class for executing robot motions.

    This class is provided the robot command interface, the knowledge base,
    and a plan to run.  The plan is executed until completion or an error
    occurs.  The implementation may assume that the object runs only
    a single plan during its lifetime (i.e., the objcet is reinitialized
    to run the next plan).
    '''
    def __init__(self, robot, knowledge_base):
        '''
        robot = the command interface for the robot (e.g., LowLevelController)
        knowledge_base = a read-only copy of the most recent
            KnowledgeBase
        '''
        self.robot = robot
        self.knowledge_base = knowledge_base

    def execute(self, plan, sleep):
        '''
        Run the plan to completion or until an error occurs.  This method blocks
        until the plan execution is finished.

        sleep = method to call to perform a wait

        Returns True if the plan was run successfully.
        Returns False if an error occurred.
        '''
        pass
