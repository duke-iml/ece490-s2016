class PerceptionInterface:
    def __init__(self, knowledge_base):
        '''
        knowledge_base = a writeable copy of the KnowledgeBase
        '''
        pass
        
    def localizeShelf(self):
        '''
        Finds the pose of the shelf with respect to the robot.  The
        robot is located at world coordinate origin aligned with the
        axes.  The reference point defining the shelf local origin
        is contained within the shelf model in the KnowledgeBase.
        
        Preconditions:
            - none
        Postconditions:
            - none
        
        Returns the shelf transform as (R, t) if found.
        Returns None if the operation failed for any reason.
        '''
        pass
       
    def localizeOrderBin(self):
        '''
        Finds the pose of the order bin with respect to the robot.
        The robot is located at world coordinate origin aligned with
        the axes.  The reference point defining the shelf local
        origin is contained within the shelf model in the
        KnowledgeBase.
        
        Preconditions:
            - none
        Postconditions:
            - none
        
        Returns the order bin transform as (R, t) if found.
        Returns None if the operation failed for any reason.
        '''
        pass
        
    def findAllObjects(self, bin):
        '''
        Determines the presence or absence of all objects within the
        given bin.  Pose information is not requested.  This
        function is intended to be used primarily in error recovery.
        
        bin = a string name of the bin corresponding to a bin entry
            in the KnowledgeBase
        
        Preconditions:
            - shelf and bin poses are available in the KnowledgeBase
            - list of objects expected to be within the bin is 
              available in the KnowledgeBase
        Postconditions:
            - none
        
        Returns a Python list of objects detected in the bin or an
            empty Python list if no objects are detected.
        Returns None if the operation failed for any reason.
        '''
        pass
    
    def localizeSpecificObject(self, bin, object):
        '''
        Determines the pose of the given object within the given bin.
        Pose is reported in the world coordinate system which is
        defined with respect to the robot.
        
        bin = a string name of the bin corresponding to a bin entry
            in the KnowledgeBase
        object = a string name of the object corresponding to an
            object entry in the KnowledgeBase
    
        Preconditions:
            - shelf and bin poses are available in the KnowledgeBase
        Postconditions:
            - none
        
        Returns the pose of the object as (R, t) if
            successful.        
        Returns None if the operation failed for any reason.
        '''
        pass
       
       
       
