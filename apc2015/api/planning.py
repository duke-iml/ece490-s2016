class PlanningInterface:
    '''
    Interface class for planning robot motions.

    A plan as referred to below is a Python dictionary/list or any constrution
    of basic types.  This plan contains all the information the ControlInterface
    needs to execute a plan, such as joint configurations, arm/gripper to use,
    goal positions or other details.  The plan must be easily serialized to JSON.
    Using Python dictionaries/list/primitive/numpy types will help this
    happen most readily.

    '''
    def __init__(self, knowledge_base):
        '''
        knowledge_base = a read-only copy of the most recent
            KnowledgeBase
        '''
        self.knowledge_base = knowledge_base
        self.robot_state = knowledge_base.robot_state

    def planMoveToVantagePoint(self, bin, vantage_point):
        '''
        Plans for any arm to view the bin at the given vantage point.
        The robot must move any of its end effectors to the vantage
        point within vantage_point_tolerance.

        Preconditions:
            - neither limb is grasping an object
        Postconditions:
            - plan viewing bin at vantage point with either arm within
              vantage_point_tolerance

        bin = a string name of the bin corresponding to a bin entry
            in the KnowledgeBase
        vantage_point = a string name of a vantage point for the
            specified bin in the KnowledgeBase

        Returns (plan, goodness, limb) if planning is successful.
        Returns None if the operation failed for any reason.
        '''
        pass

    def planGraspObjectInBin(self, bin, object):
        '''
        Plans for any limb to grasp the specified object in the specified
        bin.

        bin = a string name of the bin corresponding to a bin entry
            in the KnowledgeBase
        object = a string name of the object to grasp in the given
            bin

        Preconditions:
            - neither limb is grasping an object
            - target object pose in KnowledgeBase
            - target object configuration (e.g., folded) in KnowledgeBase
            - target object model (e.g., point cloud) in KnowledgeBase
            - point cloud of bin in KnowledgeBase
            - non-target object poses in KnowledgeBase if detected

        Postconditions:
            - plan with one gripper securely grasping the target object
            - plan ends with gripper within bin
            - may plan to move any non-target objects in bin

        Returns (plan, goodness, limb) if planning successful.
        Returns None if the operation failed for any reason.
        '''
        pass

    #def planMoveToInspectObject(self):
    #    pass

    #def planReplaceObjectInBin(self):
    #    pass

    def planMoveObjectToOrderBin(self, limb):
        '''
        Plans for object grasped by the given limb to the order
        within order_bin_tolerance.  The object must not be dropped
        from a height that exceeds max_drop_height.

        limb = a string name of limb corresponding to entry in
            KnowledgeBase

        Preconditions:
            - given limb has grasped target object

        Postconditions:
            - grasped object planned released less than max_drop_height
              above order bin
            - given limb planned within order_bin_tolerance of order bin goal

        Returns (plan, goodness, limb) if planning successful.
        Returns None if the operation failed for any reason.
        '''
        pass

    def planMoveToInitialPose(self):
        '''
        Plans a robot arm reset to the initial pose.  This is intended
        primarily for error recovery.  Both graspers do not change
        state.

        Precondition:
            - any
        Postcondition:
            - both robot arms planed at initial pose
            - both grapsers have not changed state

        Returns (plan, goodness) if planning successful.
        Returns None if the operation failed for any reason.
        '''
        pass

