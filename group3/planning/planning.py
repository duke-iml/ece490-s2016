import sys
sys.path.insert(0, "..")
sys.path.insert(0, "../../common")

from klampt import *
from klampt import vectorops,so3,se3,gldraw
from klampt.cspace import CSpace,MotionPlan
from klampt.robotsim import Geometry3D
from klampt.robotcollide import WorldCollider
import math
import os
from util.constants import *

def cleanJointConfig(q):
    if len(q) != 7:
        print "This function is not being used as intended"
    else:
        q[6] = 0.0
    return q

def getBinScore(bin_state, bin_to_score):
    """
    Inputs: self.bin_state from master and the bin letter to score
    Output: The score
    """
    score = 0
    for item in bin_state[bin_to_score]['contents']:
        score = score + ITEM_SCORES[item]
    score = score * len(bin_state[bin_to_score]['contents'])
    return score

def selectBin(bin_state):
    """
    Input: self.bin_state from master
    Output: bin letter
    """
    if not SELECT_REAL_BIN:
        return HARDCODED_BIN

    best_bin = None
    lowest_score = 99999
    for k in bin_state:
        this_score = getBinScore(bin_state, k)
        if this_score < lowest_score and not bin_state[k]['done']:
            best_bin = k
            lowest_score = this_score

    return best_bin

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
        self.robot = self.planner.robot
        id_to_index = dict([(self.robot.getLink(i).getID(),i) for i in range(self.robot.numLinks())])
        if limb=='left':
            self.limb_indices = [id_to_index[i.getID()] for i in planner.left_arm_links]
        else:
            self.limb_indices = [id_to_index[i.getID()] for i in planner.right_arm_links]
        qmin,qmax = self.robot.getJointLimits()
        self.bound = [(qmin[i]-1e-6,qmax[i]+1e-6) for i in self.limb_indices]
        self.eps = 1e-1

    def feasible(self,q):
        if not CSpace.feasible(self,q):
            #print "LimbCSpace.feasible: Configuration is out of bounds"
            return False
        if not self.planner.check_limb_collision_free(self.limb,q):
            #print "LimbCSpace.feasible: Configuration is in collision"
            return False

        for i in xrange(self.planner.world.numRigidObjects()):
            o = self.planner.world.rigidObject(i)
            g = o.geometry()
            if g != None and g.type()!="":
                if self.planner.vacuumPc.withinDistance(g, .03):
                    return False
        return True


class LimbPlanner:
    """Much of your code for HW4 will go here.
    
    Attributes:
        - world: the WorldModel
        - robot: the RobotModel
        - knowledge: the KnowledgeBase objcet
        - collider: a WorldCollider object (see the klampt.robotcollide module)
    """

    def __init__(self,world, vacuumPc):
        self.world = world
        self.robot = world.robot(0)
        self.collider = WorldCollider(world)
        self.vacuumPc = vacuumPc

        #these may be helpful
        self.left_camera_link = self.robot.getLink(LEFT_CAMERA_LINK_NAME)
        self.right_camera_link = self.robot.getLink(RIGHT_CAMERA_LINK_NAME)
        self.left_arm_links = [self.robot.getLink(i) for i in LEFT_ARM_LINK_NAMES]
        self.right_arm_links = [self.robot.getLink(i) for i in RIGHT_ARM_LINK_NAMES]
        id_to_index = dict([(self.robot.getLink(i).getID(),i) for i in range(self.robot.numLinks())])
        self.left_arm_indices = [id_to_index[i.getID()] for i in self.left_arm_links]
        self.right_arm_indices = [id_to_index[i.getID()] for i in self.right_arm_links]

        self.dynamic_objects = []

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
        armfilter = None


        left_arm_geometry_indices = [15,16,17,18,19,21,22,23,30,31]
        right_arm_geometry_indices = [35,36,37,38,39,41,42,43,50,51]
        left_hand_geometry_indices = [54,55,56]
        right_hand_geometry_indices = [57,58,59]

        
        if limb=='left':
            collindices = set(left_arm_geometry_indices+left_hand_geometry_indices)
        else:
            collindices = set(right_arm_geometry_indices+right_hand_geometry_indices)
        armfilter = lambda x:isinstance(x,RobotModelLink) and (x.index in collindices)
        #check with objects in world model
        for o1,o2 in self.collider.collisionTests(armfilter,lambda x:True):
            if o1[1].collides(o2[1]):
                #print "Collision between",o1[0].getName(),o2[0].getName()
                return False
        for obj in self.dynamic_objects:
            assert obj.info.geometry != None
            for link in collindices:
                if self.robot.getLink(link).geometry().collides(obj.info.geometry):
                    print "Collision between link",self.robot.getLink(link).getName()," and dynamic object"
                    return False

        for i in xrange(self.world.numRigidObjects()):
            o = self.world.rigidObject(i)
            g = o.geometry()
            if g != None and g.type()!="":
                if self.vacuumPc.withinDistance(g, .03):
                    return False

        return True

    def rebuild_dynamic_objects(self):
        pass
    #     self.dynamic_objects = []
    #     #check with objects in knowledge
    #     for (k,objList) in self.knowledge.bin_contents.iteritems():
    #         if objList == None:
    #             #not sensed
    #             continue
    #         for item in objList:
    #             assert item.info.geometry != None
    #             item.info.geometry.setCurrentTransform(*item.xform)
    #             self.dynamic_objects.append(item)
    #     return

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
        self.rebuild_dynamic_objects()
        cspace = LimbCSpace(self,limb)
        cspace.setup()
        if not cspace.feasible(limbstart):
            print "  Start configuration is infeasible!"
            return False
        if not cspace.feasible(limbgoal):
            print "  Goal configuration is infeasible!"
            return False
        MotionPlan.setOptions(connectionThreshold=5.0)
        MotionPlan.setOptions(shortcut=1)
        plan = MotionPlan(cspace,'sbl')
        plan.setEndpoints(limbstart,limbgoal)
        maxPlanIters = 200
        maxSmoothIters = 10
        for iters in xrange(maxPlanIters):
            plan.planMore(1)
            path = plan.getPath()
            valid_path = True
            if path != None:
                for milestone in path:
                    if not cspace.feasible(milestone):
                        valid_path = False

            if path != None and valid_path:
                print "  Found a path on iteration",iters
                if len(path) > 2:
                    print "  Smoothing..."
                    plan.planMore(min(maxPlanIters-iters,maxSmoothIters))
                    path = plan.getPath()
                cspace.close()
                plan.close()
                return path
        cspace.close()
        plan.close()
        print "  No path found"
        return False

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
            diff = sum((a-b)**2 for a,b in zip(limbstart[l],limbgoal[l]))
            if diff > 1e-8:
                print "Planning for limb",l
                print "  Euclidean distance:",math.sqrt(diff)
                self.robot.setConfig(curconfig)
                #do the limb planning
                limbpath = self.plan_limb(l,limbstart[l],limbgoal[l])
                if limbpath == False:
                    print "  Failed to plan for limb",l
                    return None
                print "   Planned successfully for limb",l
                #concatenate whole body path
                for qlimb in limbpath[1:]:
                    q = path[-1][:]
                    self.set_limb_config(l,qlimb,q)
                    path.append(q)
                self.set_limb_config(l,limbgoal[l],curconfig)
        return path
