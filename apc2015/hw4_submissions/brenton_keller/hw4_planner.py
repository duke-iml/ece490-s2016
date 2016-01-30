from klampt import *
from klampt import vectorops,so3,se3,gldraw
from klampt.cspace import CSpace,MotionPlan
from klampt.robotsim import Geometry3D
from klampt.robotcollide import WorldCollider
import time
from baxter import *

class LimbCSpace (CSpace):
    """Much of your code for HW4 will go here.  This class
    defines hooks for a motion planner.  Primarily you must define a sampling
    bound and feasibility test.  See klampt/cspace.py for more details
    on what else you can tweak.

    The configuration space is the 7-DOF configuration space of a
    single limb's joint angles.

    Attributes:
        - planner: the LimbPlanner object.  Useful for calling the
          get/set/check methods.
        - limb: the moving limb

    Attributes inherited from CSpace:
        - bounds: configuration sampling bounds, a list of pairs indicating
          the minimum/maximum range on each dimension.
        - eps: edge collision tolerance
    """
    def __init__(self,planner,limb):
        CSpace.__init__(self)
        self.planner = planner
        self.limb = limb
        #TODO: what Cartesian bounding box should the planner sample from?
        self.bound = [(-3.14159,3.14159),(-3.14159,3.14159)]
        #TODO: what collision tolerance to use?
        self.eps = 1e-2

    def feasible(self,q):
        #TODO: implement me
        #the default simply checks if q is within the bounds
        #check if the q is within the bounds
        if CSpace.feasible(self,q) :
            if(self.planner.check_limb_collision_free(self.limb,q,True)):
                return True
            else:
                return False
        else:
            return False


class LimbPlanner:
    """Much of your code for HW4 will go here.
    
    Attributes:
        - world: the WorldModel
        - robot: the RobotModel
        - knowledge: the KnowledgeBase objcet
        - collider: a WorldCollider object (see the klampt.robotcollide module)
    """
    def __init__(self,world,knowledge):
        self.world = world
        self.robot = world.robot(0)
        self.knowledge = knowledge
        self.collider = WorldCollider(world)
        self.id_to_index = {}
        #these may be helpful
        self.left_camera_link = self.robot.getLink(left_camera_link_name)
        self.right_camera_link = self.robot.getLink(right_camera_link_name)
        self.left_gripper_link = self.robot.getLink(left_gripper_link_name)
        self.right_gripper_link = self.robot.getLink(right_gripper_link_name)
        self.left_arm_links = [self.robot.getLink(i) for i in left_arm_link_names]
        self.right_arm_links = [self.robot.getLink(i) for i in right_arm_link_names]
        self.id_to_index = dict([(self.robot.getLink(i).getID(),i) for i in range(self.robot.numLinks())])
        self.left_arm_indices = [self.id_to_index[i.getID()] for i in self.left_arm_links]
        self.right_arm_indices = [self.id_to_index[i.getID()] for i in self.right_arm_links]

        #links to check for self collisions
        #a dictionary that defines the links to check for collisions given a link
        self.linkColDict = {}
        for x in range(self.robot.numLinks()):
            self.linkColDict[x] = []

        for j in range(self.robot.numLinks()):
            for i in range(self.robot.numLinks()):
                #test to see if we need to check collisions with this link
                if(self.robot.selfCollisionEnabled(i,j)):
                    self.linkColDict[j].append(i)


    def set_limb_config(self,limb,limbconfig,q):
        """Helper: Sets the 7-DOF configuration of the given limb in
        q.  Other DOFs are not touched."""
        assert len(q) == self.robot.numLinks()
        if limb=='left':
            for (i,v) in zip(self.left_arm_indices,limbconfig):
                q[i] = v
        else:
            for (i,v) in zip(self.right_arm_indices,limbconfig):
                q[i] = v
                
    def get_limb_config(self,q,limb):
        """Helper: Gets the 7-DOF configuration of the given limb given
        a full-robot configuration q"""
        qlimb = [0.0]*len(self.left_arm_indices)
        if limb=='left':
            for (i,j) in enumerate(self.left_arm_indices):
                qlimb[i] = q[j]
        else:
            for (i,j) in enumerate(self.right_arm_indices):
                qlimb[i] = q[j]
        return qlimb

    def check_collision_free(self,limb,checkObj):
        """Checks whether the given limb is collision free at the robot's
        current configuration"""
        links = []
        #t1 = time.clock()
        #left or right arm?
        if (limb == 'left'):
            links = list(self.left_arm_indices)
            links.append(self.left_gripper_link.getID()) 
            for l in left_hand_geometry_indices:
                links.append(l)
        elif(limb == 'right'):
            links = list(self.right_arm_indices)
            links.append(self.right_gripper_link.getID()) 
            for l in right_hand_geometry_indices:
                links.append(l)

        for l in links:
            #loop through all links 
            for i in self.linkColDict[l]:
               #check to see if there is a collision
                if(self.robot.getLink(l).geometry().collides(self.robot.getLink(i).geometry())):
                    #there was a collision
                    #print 'Self collision'
                    return False
        
        
            #check to see if this limb collides with the shelf
            if(self.robot.getLink(l).geometry().collides(  self.world.terrain(0).geometry())  ):
                #there was a collision
                #print 'Shelf collision',l
                return False
           
            if(self.robot.getLink(l).geometry().collides(  self.world.terrain(1).geometry())  ):
                #there was a collision
                #print 'Shelf collision',l
                return False

            if checkObj:
                #check for an object collision
                #check to see if there are any objects in the world
                for i in range(self.world.numRigidObjects()):
                    if( self.robot.getLink(l).geometry().collides(  self.world.rigidObject(i).geometry())  ):
                        #there was a collision
                        #print 'Object collision'
                        return False

            
            
                
      
        #print time.clock()-t1
        return True

    def check_limb_collision_free(self,limb,limbconfig,checkObj):
        """Checks whether the given 7-DOF limb configuration is collision
        free, keeping the rest of self.robot fixed."""
        q = self.robot.getConfig()
        self.set_limb_config(limb,limbconfig,q)
        self.robot.setConfig(q)
        return self.check_collision_free(limb,checkObj)

    def plan_limb(self,limb,limbstart,limbgoal):
        """Returns a 7-DOF milestone path for the given limb to move from the
        start to the goal, or False if planning failed"""
        #TODO: implement me.
        #Hint: you should implement the hooks in LimbCSpace above, and consult/
        #use the MotionPlan class in klampt.cspace to run an existing
        #planner.
        space = LimbCSpace(self,limb)
        MotionPlan.setOptions(type="rrt",connectionThreshold=5,bidirectional=True)
        motionPlan = MotionPlan(space)
        motionPlan.setEndpoints(limbstart,limbgoal);
        t1 = time.clock();
        motionPlan.planMore(500);
        print time.clock()-t1
        path = motionPlan.getPath()
        motionPlan.close()
        space.close()
        if path == None:
            return False
        else:
            print len(path)
            return path

    def plan(self,start,goal,order=['left','right']):
        """Plans a motion for the robot to move from configuration start
        to configuration goal.  By default, moves the left arm first,
        then the right.  To move the right first, set the 'order' argument
        to ['right','left']"""
        limbstart = {}
        limbgoal = {}
        for l in ['left','right']:
            limbstart[l] = self.get_limb_config(start,l)
            limbgoal[l] = self.get_limb_config(goal,l)
        path = [start]
        curconfig = start[:]
        for l in order:
            if limbstart[l] != limbgoal[l]:
                self.robot.setConfig(curconfig)
                #do the limb planning               
                limbpath = self.plan_limb(l,limbstart[l],limbgoal[l])             
                if limbpath == False:
                    print "Failed to plan for limb",l
                    return None
          
                print "Planned successfully for limb",l
                #concatenate whole body path
                for qlimb in limbpath[1:]:
                    q = path[-1][:]
                    self.set_limb_config(l,qlimb,q)
                    path.append(q)
                self.set_limb_config(l,limbgoal[l],curconfig)
                return path

