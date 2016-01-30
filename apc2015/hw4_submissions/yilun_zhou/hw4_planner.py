from klampt import *
from klampt import vectorops,so3,se3,gldraw
from klampt.cspace import CSpace,MotionPlan
from klampt.robotsim import Geometry3D
from klampt.robotcollide import WorldCollider
from baxter import *
from custom_util import print_instance_var
from math import pi
import random

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
        '''
        :type self.planner: LimbPlanner
        '''
        self.limb = limb
        #TODO: what Cartesian bounding box should the planner sample from?
        self.bound = [(-pi,pi),(-pi,pi),(-pi,pi),(-pi,pi),(-pi,pi),(-pi,pi),(-pi,pi)]
        #TODO: what collision tolerance to use?
        self.eps = 1e-2

    def feasible(self,q):
        #TODO: implement me
        #the default simply checks if q is within the bounds
        #print "Called"
        #print q, CSpace.feasible(self,q)
        return CSpace.feasible(self,q) and self.planner.check_limb_collision_free(self.limb, q)


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

        #these may be helpful
        self.left_camera_link = self.robot.getLink(left_camera_link_name)
        self.right_camera_link = self.robot.getLink(right_camera_link_name)
        self.left_gripper_link = self.robot.getLink(left_gripper_link_name)
        self.right_gripper_link = self.robot.getLink(right_gripper_link_name)
        self.left_arm_links = [self.robot.getLink(i) for i in left_arm_link_names]
        self.right_arm_links = [self.robot.getLink(i) for i in right_arm_link_names]
        id_to_index = dict([(self.robot.getLink(i).getID(),i) for i in range(self.robot.numLinks())])
        self.left_arm_indices = [id_to_index[i.getID()] for i in self.left_arm_links]
        self.right_arm_indices = [id_to_index[i.getID()] for i in self.right_arm_links]
        print "Planner Initialized"

    def is_in_self_collision(self, print_detail=False):
        """
        :rtype : bool
        """
        is_collide = False
        for m in xrange(self.robot.numLinks()):
            for n in xrange(self.robot.numLinks()):
                i = self.robot.getLink(m)
                j = self.robot.getLink(n)
                if m == n or not self.robot.selfCollisionEnabled(m, n):
                    continue
                if i.geometry().collides(j.geometry()):
                    if print_detail:
                        print i.getName(), j.getName(), "is colliding"
                        is_collide = True
                    else:
                        return True
                else:
                    # print i.getName(), j.getName(), "is not colliding"
                    pass
        return is_collide

    def is_colliding_with_shelf(self):
        shelf_geom = self.world.terrain(0).geometry()
        for m in xrange(self.robot.numLinks()):
            i = self.robot.getLink(m)
            if i.geometry().collides(shelf_geom):
                return True
        return False


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

    def check_collision_free(self,limb):
        """Checks whether the given limb is collision free at the robot's
        current configuration"""
        # TODO: implement me, DONE
        # print  "Checking"
        return not self.is_in_self_collision()

    def check_limb_collision_free(self,limb,limbconfig):
        """Checks whether the given 7-DOF limb configuration is collision
        free, keeping the rest of self.robot fixed."""
        q = self.robot.getConfig()
        self.set_limb_config(limb,limbconfig,q)
        self.robot.setConfig(q)
        return self.check_collision_free(limb)

    def plan_limb(self,limb,limbstart,limbgoal):
        """Returns a 7-DOF milestone path for the given limb to move from the
        start to the goal, or False if planning failed"""
        #TODO: implement me.
        #Hint: you should implement the hooks in ArmCSpace above, and consult
        #use the MotionPlan class in klampt.cspace to run an existing
        #planner.
        rrt_planner = MotionPlan(LimbCSpace(self, limb), type="rrt")
        rrt_planner.setEndpoints(limbstart, limbgoal)
        print "Planning more"
        while rrt_planner.getPath() is None:
            print "iter...",random.random()
            rrt_planner.planMore(10)
        print "Planning more done"
        print rrt_planner.getPath()
        return [limbstart,limbgoal]

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
