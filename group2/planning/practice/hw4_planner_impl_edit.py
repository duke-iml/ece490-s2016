from klampt import *
from klampt import vectorops,so3,se3,gldraw
from klampt.cspace import CSpace,MotionPlan
from klampt.robotsim import Geometry3D
from klampt.robotcollide import WorldCollider
from baxter import *
import math
import os

# NOTE: testing code starts
# from klampt.robotcspace import RobotCSpace
# class RobotCSpaceTest (RobotCSpace):
#     def __init__(self, planner, limb, collider=None):
#         RobotCSpace.__init__(self, planner.robot, collider=None)
#         self.planner = planner
#         self.limb = limb
#         self.robot = planner.robot

#         id_to_index = dict([(self.robot.link(i).getID(),i) for i in range(self.robot.numLinks())])
#         if limb=='left':
#             # self.limb_indices = [id_to_index[i.getID()] for i in planner.left_arm_links]
#             self.limb_indices = left_arm_geometry_indices + left_hand_geometry_indices
#         else:
#             # self.limb_indices = [id_to_index[i.getID()] for i in planner.right_arm_links]
#             self.limb_indices = right_arm_geometry_indices + right_hand_geometry_indices
#         qmin,qmax = self.robot.getJointLimits()
#         self.bound = [(qmin[i]-1e-6,qmax[i]+1e-6) for i in self.limb_indices]
#         self.eps = 1e-1

#     def feasible(self,q):
#         if not RobotCSpace.feasible(self,q):
#             print "LimbCSpace.feasible: Configuration is out of bounds"
#             return False
#         if not self.planner.check_limb_collision_free(self.limb,q):
#                 print "LimbCSpace.feasible: Configuration is in collision"
#                 return False
#         return True

from klampt.robotcspace import ClosedLoopRobotCSpace
class ClosedLoopCSpaceTest (ClosedLoopRobotCSpace):
    def __init__(self, planner, limb, iks, collider=None):
        ClosedLoopRobotCSpace.__init__(self,planner.robot, iks, collider=None)
        self.planner = planner
        self.limb = limb
        self.robot = planner.robot

        id_to_index = dict([(self.robot.link(i).getID(),i) for i in range(self.robot.numLinks())])
        if limb=='left':
            # self.limb_indices = [id_to_index[i.getID()] for i in planner.left_arm_links]
            self.limb_indices = left_arm_geometry_indices + left_hand_geometry_indices
        else:
            # self.limb_indices = [id_to_index[i.getID()] for i in planner.right_arm_links]
            self.limb_indices = right_arm_geometry_indices + right_hand_geometry_indices
        qmin,qmax = self.robot.getJointLimits()
        self.bound = [(qmin[i]-1e-6,qmax[i]+1e-6) for i in self.limb_indices]
        self.eps = 1e-1

    def feasible(self,q):
        if len(q) < self.robot.numLinks:
            qLimb = q
            q = self.robot.getConfig()
            for i in range(len(self.limb_indices)):
                q[self.limb_indices[i]] = qLimb[i]

        qmin,qmax = self.robot.getJointLimits()
        for i in range(len(q)):
            if qmin[i] > q[i] or qmax[i] < q[i]:
                print "joint out of range"
                print i, qmin[i], q[i], qmax[i]
                return False

        # if not ClosedLoopRobotCSpace.feasible(self,q):
        #     print "ClosedLoopRobotCSpace.feasible: Configuration is out of bounds"
        #     return False

        # for i in range(len(self.limb_indices)):
        #     # qlimbs[i] = q[self.limb_indices[i]]
        #     print i, self.limb_indices[i], len(q)
        # print len(q), len(self.bound)
        # for i in range(len(q)):
        #     print i
            # print self.bound[i][0]

            # if self.bound[i][0] > q[i] or self.bound[i][1] < q[i]:
            #     print "joint out of range"
            #     return False
            # # print q[i], self.bound[i]

        # if self.inJointLimits(q):
        #     print "within joint limits"

        if not self.planner.check_limb_collision_free(self.limb,q):
                print "ClosedLoopRobotCSpace.feasible: Configuration is in collision"
                return False
        return True

# NOTE: testing code ends

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
        id_to_index = dict([(self.robot.link(i).getID(),i) for i in range(self.robot.numLinks())])
        if limb=='left':
            # self.limb_indices = [id_to_index[i.getID()] for i in planner.left_arm_links]
            # self.limb_indices = self.limb_indices + [54,55]
            self.limb_indices = left_arm_geometry_indices + left_hand_geometry_indices
        else:
            # self.limb_indices = [id_to_index[i.getID()] for i in planner.right_arm_links]
            self.limb_indices = right_arm_geometry_indices + right_hand_geometry_indices
        qmin,qmax = self.robot.getJointLimits()
        self.bound = [(qmin[i]-1e-6,qmax[i]+1e-6) for i in self.limb_indices]
        self.eps = 1e-1

    def feasible(self,q):
        print len(q)
        if not CSpace.feasible(self,q):
            print "LimbCSpace.feasible: Configuration is out of bounds"
            return False
        # for i in range(len(q)):
        #     # print "q", i, q[i], self.bound[i][0], self.bound[i][1]
        #     if not ((q[i] >= self.bound[i][0]) and (q[i] <= self.bound[i][1])):
        #         print "Joint #",self.limb_indices[i],"(",q[i],") out of limits (min:",self.bound[i][0],", max:", self.bound[i][1],")"
        #         return False

        if not self.planner.check_limb_collision_free(self.limb,q):
                # print "LimbCSpace.feasible: Configuration is in collision"
                return False
        return True

class TransferCSpace (CSpace):
    def __init__(self,planner,limb,obj,grasp):
        CSpace.__init__(self)
        self.planner = planner
        self.limb = limb
        self.object = obj
        self.grasp = grasp
        self.objectGeom = obj.info.geometry
        self.robot = self.planner.robot
        id_to_index = dict([(self.robot.link(i).getID(),i) for i in range(self.robot.numLinks())])
        if limb=='left':
            self.limb_indices = [id_to_index[i.getID()] for i in planner.left_arm_links]
            self.limb_indices = left_arm_geometry_indices + left_hand_geometry_indices
        else:
            self.limb_indices = [id_to_index[i.getID()] for i in planner.right_arm_links]
            self.limb_indices = right_arm_geometry_indices + right_hand_geometry_indices
        qmin,qmax = self.robot.getJointLimits()
        self.bound = [(qmin[i]-1e-6,qmax[i]+1e-6) for i in self.limb_indices]
        self.eps = 1e-1

    def feasible(self,q):
        if not CSpace.feasible(self,q):
            #print "LimbCSpace.feasible: Configuration is out of bounds"
            return False
        qrob = self.robot.getConfig()
        self.planner.set_limb_config(self.limb,q,qrob)
        self.robot.setConfig(qrob)
        if not self.planner.check_collision_free_with_object(self.limb,self.objectGeom,self.grasp):
            #print "LimbCSpace.feasible: Configuration is in collision"
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
    def __init__(self,world,knowledge):
        self.world = world
        self.robot = world.robot(0)
        self.knowledge = knowledge
        self.collider = WorldCollider(world)

        #these may be helpful
        self.left_camera_link = self.robot.link(left_camera_link_name)
        self.right_camera_link = self.robot.link(right_camera_link_name)
        self.left_gripper_link = self.robot.link(left_gripper_link_name)
        self.right_gripper_link = self.robot.link(right_gripper_link_name)
        self.left_arm_links = [self.robot.link(i) for i in left_arm_link_names]
        self.right_arm_links = [self.robot.link(i) for i in right_arm_link_names]
        id_to_index = dict([(self.robot.link(i).getID(),i) for i in range(self.robot.numLinks())])
        # self.left_arm_indices = [id_to_index[i.getID()] for i in self.left_arm_links]
        # self.right_arm_indices = [id_to_index[i.getID()] for i in self.right_arm_links]
        # print "LEFT ARM INDICES", self.left_arm_indices

        self.left_arm_indices = left_arm_geometry_indices + left_hand_geometry_indices
        self.right_arm_indices = right_arm_geometry_indices + right_hand_geometry_indices
        # print "LEFT ARM INDICES", self.left_arm_indices

        self.dynamic_objects = []

        # NOTE: added from lab3e.py
        self.roadmap = ([],[])
        self.limb_indices = []
        self.pathToDraw = None


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
            qlimb = [0.0]*len(self.left_arm_indices)
            for (i,j) in enumerate(self.left_arm_indices):
                qlimb[i] = q[j]
        else:
            qlimb = [0.0]*len(self.right_arm_indices)
            for (i,j) in enumerate(self.right_arm_indices):
                qlimb[i] = q[j]
        return qlimb

    def check_collision_free(self,limb):
        """Checks whether the given limb is collision free at the robot's
        current configuration"""
        armfilter = None
        if limb=='left':
            collindices = set(left_arm_geometry_indices+left_hand_geometry_indices)
        else:
            collindices = set(right_arm_geometry_indices+right_hand_geometry_indices)
        armfilter = lambda x:isinstance(x,RobotModelLink) and (x.index in collindices)
        #check with objects in world model
        for o1,o2 in self.collider.collisionTests(armfilter,lambda x:True):   # NOTE: what is bb_reject??
            if o1[1].collides(o2[1]):
                # print "Collision between",o1[0].getName(),o2[0].getName()
                return False

        for obj in self.dynamic_objects:
            # print "checking collision with objects"
            # print obj.info.geometry
            assert obj.info.geometry != None
            for link in collindices:
                if self.robot.link(link).geometry().collides(obj.info.geometry):
                    # NOTE: Uncomment this line to show collision warnings
                    # print "Collision between link",self.robot.link(link).getName()," and dynamic object"
                    return False

        for link in collindices:
            shelfGeometry = self.world.terrain(0).geometry()
            linkGeometry = self.robot.link(link).geometry()

            shelfGeometry.setCollisionMargin(0.05)
            # linkGeometry.setCollisionMargin(0.05)
            if shelfGeometry.collides(linkGeometry):
                # print "link #",link,"collides with terrain"
                return False


        return True

    def check_collision_free_with_object(self,limb,objectGeom,grasp):
        """Checks whether the given limb, holding an object at the given
        grasp, is collision free at the robot's current configuration"""
        if not self.check_collision_free(limb):
            return False
        objToGripperXForm = se3.mul(left_gripper_center_xform,se3.inv(grasp.grasp_xform))
        #assume robot configuration is updated
        if limb=='left':
            gripperLink = self.robot.link(left_gripper_link_name)
        else:
            gripperLink = self.robot.link(right_gripper_link_name)

        Tgripper = gripperLink.getTransform();
        Tobj = se3.mul(Tgripper,objToGripperXForm)
        objectGeom.setCurrentTransform(*Tobj)
        for t in xrange(self.world.numTerrains()):
            if self.world.terrain(t).geometry().collides(objectGeom):
                # print "Held object-shelf collision"
                return False
        for o in self.dynamic_objects:
            if o.info.geometry.collides(objectGeom):
                # print "Held object-object collision"
                return False
        return True

    def rebuild_dynamic_objects(self):
        self.dynamic_objects = []
        #check with objects in knowledge
        for (k,objList) in self.knowledge.bin_contents.iteritems():
            if objList == None:
                #not sensed
                continue
            for item in objList:
                # print item.info.name
                # print k, item.info.name, item.info.geometry
                assert item.info.geometry != None
                item.info.geometry.setCurrentTransform(*item.xform)
                self.dynamic_objects.append(item)
        return

    def check_limb_collision_free(self,limb,limbconfig):
        """Checks whether the given 7-DOF limb configuration is collision
        free, keeping the rest of self.robot fixed."""
        q = self.robot.getConfig()
        self.set_limb_config(limb,limbconfig,q)
        self.robot.setConfig(q)
        return self.check_collision_free(limb)

    def plan_limb(self,limb,limbstart,limbgoal, printer=True, iks = None):
        """Returns a 7-DOF milestone path for the given limb to move from the
        start to the goal, or False if planning failed"""
        self.rebuild_dynamic_objects()

        # NOTE:
        cspace = LimbCSpace(self,limb)
        if iks != None:
            print "Initializing ClosedLoopCSPace"
            cspace = ClosedLoopCSpaceTest(self,limb,iks)

        if not cspace.feasible(limbstart):
            print "  Start configuration is infeasible!"
            return 1
        if not cspace.feasible(limbgoal):
            print "  Goal configuration is infeasible!"
            return 2


        # MotionPlan.setOptions(connectionThreshold=5.0)
        # MotionPlan.setOptions(shortcut=1)
        # plan = MotionPlan(cspace,'sbl')

        # MotionPlan.setOptions(type='rrt*')
        # MotionPlan.setOptions(type="prm",knn=10,connectionThreshold=0.01,shortcut=True)
        # MotionPlan.setOptions(type='fmm*')

        # MotionPlan.setOptions(bidirectional = 1)

        # MotionPlan.setOptions(type="sbl", perturbationRadius = 0.5, connectionThreshold=2.0, bidirectional = True)
        # MotionPlan.setOptions(type="rrt",perturbationRadius=0.1,bidirectional=True)
        # MotionPlan.setOptions(type="rrt", perturbationRadius = 0.25, connectionThreshold=2, bidirectional = True, shortcut = True, restart=True)
        # MotionPlan.setOptions(type="rrt*", perturbationRadius = 0.05, connectionThreshold=2, bidirectional = True)
        MotionPlan.setOptions(type="rrt", perturbationRadius = 0.2, connectionThreshold=2, bidirectional = True, shortcut = True, restart=True)
        # MotionPlan.setOptions(type="prm",knn=1,connectionThreshold=0.01)
        # plan = MotionPlan(cspace, type='sbl')

        plan = MotionPlan(cspace)



        # NOTE: added from lab3e.py
        # self.roadmap = ([],[])







        plan.setEndpoints(limbstart,limbgoal)
        maxPlanIters = 20
        maxSmoothIters = 10
        for iters in xrange(maxPlanIters):
            print iters
            plan.planMore(100)

            if limb=='left':
                self.limb_indices = self.left_arm_indices
            else:
                self.limb_indices = self.right_arm_indices

            self.roadmap = plan.getRoadmap()
            V,E =self.roadmap
            print len(V),"feasible milestones sampled,",len(E),"edges connected"



            path = plan.getPath()
            if path != None:
                if printer:
                    print "  Found a path on iteration",iters
                if len(path) > 2:
                    print "  Smoothing..."
                    plan.planMore(min(maxPlanIters-iters,maxSmoothIters))
                    path = plan.getPath()
                cspace.close()
                plan.close()

                # testing
                self.pathToDraw = path

                return path
        cspace.close()
        plan.close()
        if printer:
            print "  No path found"
        return False

    def plan(self,start,goal,order=['left','right'],printer=True, iks = None):
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
                if printer:
                    print "< Planning for limb",l,">"
                    print "  Euclidean distance:",math.sqrt(diff)
                self.robot.setConfig(curconfig)
                #do the limb planning
                limbpath = self.plan_limb(l,limbstart[l],limbgoal[l],printer=printer, iks=iks)
                if limbpath == 1 or limbpath == 2 or limbpath == False:
                    if printer:
                        print "  Failed to plan for limb",l,"\n"
                    return limbpath
                if printer:
                    print "   Planned successfully for limb",l, "\n"
                #concatenate whole body path
                for qlimb in limbpath[1:]:
                    q = path[-1][:]
                    self.set_limb_config(l,qlimb,q)
                    path.append(q)
                self.set_limb_config(l,limbgoal[l],curconfig)
        return path

    def plan_limb_transfer(self,limb,limbstart,limbgoal,heldobject,grasp):
        """Returns a 7-DOF milestone path for the given limb to move from the
        start to the goal while holding the given object.
        Returns False if planning failed"""
        self.rebuild_dynamic_objects()
        cspace = TransferCSpace(self,limb,heldobject,grasp)
        if not cspace.feasible(limbstart):
            print "  Start configuration is infeasible!"
            return False
        if not cspace.feasible(limbgoal):
            print "  Goal configuration is infeasible!"
            return False
        MotionPlan.setOptions(connectionThreshold=5.0)
        MotionPlan.setOptions(shortcut=1)
        plan = MotionPlan(cspace,'sbl')

        # MotionPlan.setOptions(type='rrt*')
        # MotionPlan.setOptions(type="prm",knn=10,connectionThreshold=0.1,shortcut=True)
        # MotionPlan.setOptions(type='fmm*')


        plan = MotionPlan(cspace)

        plan.setEndpoints(limbstart,limbgoal)
        maxPlanIters = 200
        maxSmoothIters = 10
        for iters in xrange(maxPlanIters):
            plan.planMore(1)
            path = plan.getPath()
            if path != None:
                print "  Found a path on iteration",iters,'/',maxPlanIters
                if len(path) > 2:
                    print "  Smoothing..."
                    plan.planMore(min(maxPlanIters-iters,maxSmoothIters))
                    path = plan.getPath()
                cspace.close()
                plan.close()
                return path
        cspace.close()
        plan.close()
        print "No path found"
        return False

    def plan_transfer(self,start,goal,limb,heldobject,grasp):
        """Plans a motion for the robot to move from configuration start
        to configuration goal while holding the given object."""
        limbstart = self.get_limb_config(start,limb)
        limbgoal = self.get_limb_config(goal,limb)
        path = [start]
        print "Planning transfer path for limb",limb
        self.robot.setConfig(start)
        #do the limb planning
        limbpath = self.plan_limb_transfer(limb,limbstart,limbgoal,heldobject,grasp)
        if limbpath == False:
            print "  Failed to plan transfer path for limb",limb
            return None
        print "  Planned transfer path successfully for limb",limb
        #concatenate whole body path
        for qlimb in limbpath[1:]:
            q = path[-1][:]
            self.set_limb_config(limb,qlimb,q)
            path.append(q)
        return path
