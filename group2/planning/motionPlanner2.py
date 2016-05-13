from klampt import *
from klampt import vectorops,so3,se3,gldraw,trajectory,visualization,robotplanning
from klampt.cspace import CSpace,MotionPlan
from klampt.robotsim import Geometry3D
from klampt.robotcollide import WorldCollider
from baxter import *
import math
import os
import random

# NOTE: testing code starts
from klampt.robotcspace import ClosedLoopRobotCSpace
class ClosedLoopCSpaceTest (ClosedLoopRobotCSpace):
    def __init__(self, planner, limb, iks, collider=None):
        ClosedLoopRobotCSpace.__init__(self,planner.robot, iks, collider=None)
        self.planner = planner
        self.limb = limb
        self.robot = planner.robot

        id_to_index = dict([(self.robot.link(i).getID(),i) for i in range(self.robot.numLinks())])
        if limb=='left':
            self.limb_indices = left_arm_geometry_indices + left_hand_geometry_indices
        else:
            self.limb_indices = right_arm_geometry_indices + right_hand_geometry_indices
        qmin,qmax = self.robot.getJointLimits()
        self.bound = [(qmin[i]-1e-6,qmax[i]+1e-6) for i in self.limb_indices]
        self.eps = 1e-1

    def feasible(self,q):
        # set robot to given q
        # get joint 55 orientation
        # if rotation is not right, return false

        qcurrent = self.robot.getConfig()
        if len(q) < len(qcurrent):
            if len(self.limb_indices) != len(q):
                return False
            else:
                for i in range(len(self.limb_indices)):
                    qcurrent[self.limb_indices[i]] = q[i]
                q = qcurrent

        # self.robot.setConfig(q)
        # Rcur = self.robot.link(55).getTransform()[0]
        # # Rgoal


        # if len(q)>len(self.limb_indices): return False
        # for i in range(len(q)):
        #     # print "q", i, q[i], self.bound[i][0], self.bound[i][1]
        #     # print self.limb, self.limb_indices, i, len(q)
        #     if (q[i] < self.bound[i][0]) :
        #         print "Joint #",self.limb_indices[i],"(",q[i],") out of limits (min:",self.bound[i][0],")"
        #         print "Changed joint value to its minimum"
        #         q[i] = self.bound[i][0]

        #     if (q[i] > self.bound[i][1]) :
        #         print "Joint #",self.limb_indices[i],"(",q[i],") out of limits (max:",self.bound[i][1],")"
        #         print "Changed joint value to its maximum"
        #         q[i] = self.bound[i][1]

        # print 1
        # # self.robot.setConfig(q)
        # # return AdaptiveCSpace.feasible(self,q)

        # print 2
        # if not self.inJointLimits(q): return False

        # print 3
        # self.robot.setConfig(q)
        # if not self.closedLoop(): return False;

        # #check extras
        # print 4
        # for f in self.extraFeasibilityTesters:
        #     if not f(q): return False

        # print 5
        # #check collisions
        # if self.selfCollision(): return False

        # print 6
        # if self.envCollision(): return False
        # return True



        if not CSpace.feasible(self,q):
            print "CSpace.feasible: Configuration is out of bounds"
            return False

        if not self.planner.check_collision_free(self.limb,q):
            # print "ClosedLoopRobotCSpace.feasible: Configuration is in collision"
            return False
        return True


    # def sample(self):
    #     qmin,qmax = self.robot.getJointLimits()
    #     q = self.robot.getConfig()

    #     count = 1
    #     while(not self.closedLoop()):
    #         for i in range(len(self.limb_indices)):
    #             j = self.limb_indices[i]
    #             q[j] = random.uniform(qmin[j], qmax[j])
    #         self.robot.setConfig(q)
    #         print count
    #         count += 1

    #     return q


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
            self.limb_indices = left_arm_geometry_indices + left_hand_geometry_indices
        else:
            self.limb_indices = right_arm_geometry_indices + right_hand_geometry_indices
        qmin,qmax = self.robot.getJointLimits()
        self.bound = [(qmin[i]-1e-6,qmax[i]+1e-6) for i in self.limb_indices]
        self.eps = 1e-1

    def feasible(self,q):
        for i in range(len(q)):
            # print "q", i, q[i], self.bound[i][0], self.bound[i][1]
            if (q[i] < self.bound[i][0]) :
                print "Joint #",self.limb_indices[i],"(",q[i],") out of limits (min:",self.bound[i][0],")"
                print "Changed joint value to its minimum"
                q[i] = self.bound[i][0]

            if (q[i] > self.bound[i][1]) :
                print "Joint #",self.limb_indices[i],"(",q[i],") out of limits (max:",self.bound[i][1],")"
                print "Changed joint value to its maximum"
                q[i] = self.bound[i][1]

        if not CSpace.feasible(self,q):
            # print "LimbCSpace.feasible: Configuration is out of bounds"
            return False

        # cond = self.planner.check_limb_collision_free(self.limb,q)
        # if not cond:
        if not self.planner.check_limb_collision_free(self.limb,q):
            # print "LimbCSpace.feasible: Configuration is in collision"
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

        self.left_arm_indices = left_arm_geometry_indices + left_hand_geometry_indices
        self.right_arm_indices = right_arm_geometry_indices + right_hand_geometry_indices

        self.dynamic_objects = []

        # NOTE: added from lab3e.py
        self.roadmap = ([],[])
        self.limb_indices = []
        self.pathToDraw = []
        self.colMargin = 0
        self.activeLimb = None


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

        ignoreList = ["Amazon_Picking_Shelf", "bin_"]
        ignoreList = '\t'.join(ignoreList)
        spatulaIgnoreList = [55,56,57]

        #check with objects in world model
        for o1,o2 in self.collider.collisionTests(armfilter,lambda x:True):   # NOTE: what is bb_reject??
            # print "Collision Test: Collision between",o1[0].getName(),o2[0].getName()
            if o1[0].getName()[0:3] in ignoreList and o2[0].index in spatulaIgnoreList:
                # print "ignoring collision between shelf and spautla"
                continue
            if o2[0].getName()[0:3] in ignoreList and o1[0].index in spatulaIgnoreList:
                # print "ignoring collision between shelf and spautla"
                continue
            if o1[0].getName()[0:3] in ignoreList:
                for obj in self.dynamic_objects:
                    assert obj.info.geometry != None
                    if o1[1].collides(obj.info.geometry):
                        continue
            if o2[0].getName()[0:3] in ignoreList:
                for obj in self.dynamic_objects:
                    assert obj.info.geometry != None
                    if o1[1].collides(obj.info.geometry):
                        continue

            if o1[1].collides(o2[1]):
                # print "Collision between",o1[0].getName(),o2[0].getName()
                return False

        # for obj in self.dynamic_objects:
        #     # print "checking collision with objects"
        #     # print obj.info.geometry
        #     assert obj.info.geometry != None

        #     for link in collindices:
        #         if self.robot.link(link).geometry().collides(obj.info.geometry):
        #             print "Collision between link",self.robot.link(link).getName()," and dynamic object"
        #             return False

        # for link in collindices:
        #     # print "Collision Test: Collision between",self.robot.link(link).getName(),"shelf"
        #     shelfGeometry = self.world.terrain(0).geometry()
        #     linkGeometry = self.robot.link(link).geometry()

        #     if shelfGeometry.collides(linkGeometry):
        #         print "link #",link,"collides with terrain"
        #         return False

        # print self.robot.getConfig()
        return True

    def rebuild_dynamic_objects(self):
        self.dynamic_objects = []
        #check with objects in knowledge
        for (k,objList) in self.knowledge.bin_contents.iteritems():
            if objList == None:
                #not sensed
                continue
            for item in objList:
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

        # NOTE: Not sure, but I think this is what's happening
        # Need to reset pathToDraw here because the MyGLViewer.draw() is called
        # concurrently, it uses an old path (ex. of left arm) while using the
        # new limb (ex. right limb) for drawing the trajectory. This throws an
        # Index-Out-of-Range exception for drawing the path
        self.pathToDraw = []

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

        # MotionPlan.setOptions(type="sbl", perturbationRadius = 0.25, connectionThreshold=2.0, bidirectional = True)
        # MotionPlan.setOptions(type="rrt",perturbationRadius=0.1,bidirectional=True)
        # MotionPlan.setOptions(type="rrt", perturbationRadius = 0.25, connectionThreshold=2, bidirectional = True, shortcut = True, restart=True)
        # MotionPlan.setOptions(type="rrt*", perturbationRadius = 0.25, connectionThreshold=2, bidirectional = True)
        MotionPlan.setOptions(type="rrt", perturbationRadius = 1, connectionThreshold=2, bidirectional = True, shortcut = True, restart=True)
        # MotionPlan.setOptions(type="prm",knn=1,connectionThreshold=0.01)
        # MotionPlan.setOptions(type="rrt", perturbationRadius = 1, connectionThreshold=2, bidirectional = True, shortcut = True, restart=True)
        # plan = MotionPlan(cspace, type='sbl')


        # works best for non-ClosedLoopRobotCSpace
        # MotionPlan.setOptions(type="rrt", perturbationRadius = 1, connectionThreshold=2, bidirectional = True, shortcut = True, restart=True)

        plan = MotionPlan(cspace)

        plan.setEndpoints(limbstart,limbgoal)
        maxPlanIters = 20
        maxSmoothIters = 100

        print "  Planning.",
        for iters in xrange(maxPlanIters):
            # print iters

            if limb=='left':
                self.limb_indices = self.left_arm_indices
                self.activeLimb = 'left'
            else:
                self.limb_indices = self.right_arm_indices
                self.activeLimb = 'right'

            self.roadmap = plan.getRoadmap()
            # print plan.getRoadmap()
            V,E =self.roadmap
            # print "iter:",iters,"--",len(V),"feasible milestones sampled,",len(E),"edges connected"
            # print iters,"/V",len(V),"/E",len(E),
            print ".",

            plan.planMore(10)                                       # 100

            path = plan.getPath()
            if path != None:
                if printer:
                    print "\n  Found a path on iteration",iters
                if len(path) > 2:
                    print "  Smoothing path"
                    # plan.planMore(min(maxPlanIters-iters,maxSmoothIters))
                    plan.planMore(maxSmoothIters)
                    path = plan.getPath()
                cspace.close()
                plan.close()

                # testing
                # print "  Returning path (limb", self.activeLimb,", (",len(self.limb_indices),"/",len(path[0]),"))"


                self.pathToDraw = path

                return path
        cspace.close()
        plan.close()
        if printer:
            print "  No path found"

        return False

    def plan(self,start,goal,limb,printer=True, iks = None):
        """Plans a motion for the robot to move from configuration start
        to configuration goal.  By default, moves the left arm first,
        then the right.  To move the right first, set the 'order' argument
        to ['right','left']"""
        limbstart = {}
        limbgoal = {}
        # for l in ['left','right']:
        l =limb
        limbstart[l] = self.get_limb_config(start,l)
        limbgoal[l] = self.get_limb_config(goal,l)
        path = [start]
        curconfig = start[:]

        diff = sum((a-b)**2 for a,b in zip(limbstart[l],limbgoal[l]))
        if diff > 1e-8:
            if printer:
                print "< Planning for limb",l,">"
                # print "  Euclidean distance:",math.sqrt(diff)
            self.robot.setConfig(curconfig)
            #do the limb planning
            if iks == None:
                limbpath = self.plan_limb(l,limbstart[l],limbgoal[l],printer=printer, iks=iks)


            else:
                # limbpath = self.plan_limb(l,limbstart[l],limbgoal[l],printer=printer, iks=iks)
                trajectory = self.plan_closedLoop(start,goal,iks=iks)
                return trajectory

            if limbpath == 1 or limbpath == 2 or limbpath == False:
                if printer:
                    print "  Failed to plan for limb",l,"\n"
                return limbpath
            if printer:
                print "  Planned successfully for limb",l, "\n"
            #concatenate whole body path
            for qlimb in limbpath[1:]:
                q = path[-1][:]
                self.set_limb_config(l,qlimb,q)
                path.append(q)
            self.set_limb_config(l,limbgoal[l],curconfig)
        return path

    def plan_closedLoop(self,qstart,qgoal,iks,printer=False):
        if printer:
            print "starting config: ", qstart
            print "goal config: ", qgoal

        # vis = visualization.add("robot",  self.robot)
        # vis = visualization.add("IK goal", iks)
        self.world.terrain(0).geometry().setCollisionMargin(0.075)

        cspace = robotcspace.ClosedLoopRobotCSpace(self.robot, iks, self.collider)
        cspace.eps = 1e-1
        cspace.setup()

        configs=[]
        configs.append(qstart)
        configs.append(qgoal)

        configs[0] = cspace.solveConstraints(configs[0])
        configs[1] = cspace.solveConstraints(configs[1])

        settings = { 'type':"sbl", 'perturbationRadius':0.5, 'bidirectional':1, 'shortcut':1, 'restart':1, 'restartTermCond':"{foundSolution:1,maxIters:1000}" }

        wholepath = [configs[0]]
        for i in range(len(configs)-1):
            #this code uses the robotplanning module's convenience functions
            self.robot.setConfig(configs[i])
            plan = robotplanning.planToConfig(self.world,self.robot,configs[i+1],
                                              movingSubset='all',
                                              **settings)

            if plan is None:
                return False
            print "Planning..."
            plan.planMore(1000)
            #this code just gives some debugging information. it may get expensive
            V,E = plan.getRoadmap()
            # self.roadmap = plan.getRoadmap()
            print len(V),"feasible milestones sampled,",len(E),"edges connected"
            path = plan.getPath()
            if path is None or len(path)==0:
                print "Failed to plan path between configuration",i,"and",i+1
                #debug some sampled configurations
                print V[0:max(10,len(V))]
                return False

            # maxSmoothIters = 200
            # plan.planMore(maxSmoothIters)
            # path = plan.getPath()

            self.pathToDraw = path
            print len(path)


            #the path is currently a set of milestones: discretize it so that it stays near the contact surface
            path = cspace.discretizePath(path,epsilon=1e-4)
            wholepath += path[1:]


            #to be nice to the C++ module, do this to free up memory
            plan.space.close()
            plan.close()

        return wholepath
