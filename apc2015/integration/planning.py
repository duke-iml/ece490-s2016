class PlanningInterface:
    def __init__(self, robot, knowledge_base):
        '''
        robot = interface for commanding the robot
        knowledge_base = a read-only copy of the most recent
            KnowledgeBase
        '''
        pass

    def moveToVantagePoint(self, bin, vantage_point):
        '''
        Moves any arm to view the bin at the given vantage point.
        The robot must move any of its end effectors to the vantage
        point within vantage_point_tolerance.
        
        Preconditions:
            - neither limb is grasping an object
        Postconditions:
            - robot viewing bin at vantage point with either arm
        
        bin = a string name of the bin corresponding to a bin entry
            in the KnowledgeBase
        vantage_point = a string name of a vantage point for the 
            specified bin in the KnowledgeBase
        tolerance = a floating point distance in meters in the world
            for how close the end-effector must be positioned with
            respect to the vantage point
        
        Returns the limb name used to view the bin if successful.
        Returns None if the operation failed for any reason.
        '''
        pass

    def graspObjectInBin(self, bin, object):
        '''
        Moves any limb to grasp the specified object in the specified
        bin.

        bin = a string name of the bin corresponding to a bin entry
            in the KnowledgeBase
        object = a string name of the object to grasp in the given
            bin
        
        Preconditions:
            - neither limb is grasping an object
            - target object pose is available
            - target object configuration (e.g., folded) is available
            - target object model (e.g., point cloud) is available
            - point cloud of bin is available
            - non-target object poses available if detected
            
        Postconditions:
            - one gripper has securely grasped the target object
            - gripper within bin
            - non-target objects in bin may move
        
        Returns the limb used to grasp the object if successful.
        Returns None if the operation failed for any reason.
        '''
        pass
    
    #def moveToInspectObject(self):
    #    pass
    
    #def replaceObjectInBin(self):
    #    pass
        
    def moveObjectToOrderBin(self, limb):
        '''
        Moves the object grasped by the given limb to the order
        within order_bin_tolerance.  The object must not be dropped
        from a height that exceeds max_drop_height.
        
        limb = a string name of limb corresponding to entry in
            KnowledgeBase
        
        Preconditions:
            - given limb has grasped target object
            
        Postconditions:
            - grasped object released less than max_drop_height above
              order bin
            - given limb within given tolerance of order bin goal
        
        Returns True if gripper was opened over/within the order bin.
        Returns False if the operation failed for any reason.
        '''
        pass
    
    def moveToInitialPose(self):
        '''
        Resets the robot arms to the initial pose.  This is intended
        primarily for error recovery.  Both graspers do not change
        state.
        
        Precondition:
            - any
        Postcondition:
            - both robot arms at initial pose
            - both grapsers have not changed state
            
        Returns True when initial arm poses achieved.
        Returns False if the operation failed for any reason.
        '''
        pass
    
