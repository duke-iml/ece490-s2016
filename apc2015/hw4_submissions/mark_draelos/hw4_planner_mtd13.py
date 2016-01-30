from klampt import *
from klampt import vectorops,so3,se3,gldraw
from klampt.cspace import CSpace,MotionPlan
from klampt.robotsim import Geometry3D
from klampt.robotcollide import WorldCollider
import os
import numpy
from math import ceil, pi
import time

from baxter import *

#The path of the klampt_models directory
model_dir = "../klampt_models/"

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
        # set up a search space based on the robot 1 m x 2 m x 2 m
        (R, t) = self.planner.robot.getLink(0).getParentTransform()
        self.bound = self.bounds = [(t[0]-1, t[0]+2), (t[1]-2, t[2]+2), (0, 3)]
        self.eps = 1e-2
        self.ignore = None

    def feasible(self,q):
        # check to see if the limb is in collision
        return self.planner.check_limb_collision_free(self.limb, q, self.ignore)


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
        self.id_to_index = id_to_index = dict([(self.robot.getLink(i).getID(),i) for i in range(self.robot.numLinks())])
        self.left_arm_indices = [id_to_index[i.getID()] for i in self.left_arm_links]
        self.right_arm_indices = [id_to_index[i.getID()] for i in self.right_arm_links]

        self.arm_links = {
            'right': [ (i, self.robot.getLink(i)) for i in right_arm_geometry_indices + right_hand_geometry_indices ],
            'left': [ (i, self.robot.getLink(i)) for i in left_arm_geometry_indices + left_hand_geometry_indices ]
        }
        self.robot_links = [ (i, self.robot.getLink(i)) for i in range(self.robot.numLinks()) ]


        # plan a path using RRT
        MotionPlan.setOptions(type="rrt",shortcut=1,restart=1)

        self.cs = {}
        for limb in [ 'left', 'right' ]:
            # set up C-space
            self.cs[limb] = LimbCSpace(self, limb)
            self.cs[limb].setup()

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

    def check_collision_free(self,limb,ignore):
        """Checks whether the given limb is collision free at the robot's
        current configuration"""

        ignore = ignore or []

        # check if the robot collides with itself
        for (robot_link_index, robot_link) in self.robot_links:
            for (arm_link_index, arm_link) in self.arm_links[limb]:
                # don't check a link against itself
                if arm_link_index == robot_link_index:
                    continue

                # don't check a link if self-collision isn't enabled
                if not self.robot.selfCollisionEnabled(arm_link_index, robot_link_index):
                    continue

                # now check for a collision
                if arm_link.geometry().collides(robot_link.geometry()):
                    # collision found
#                     print 'collision between', arm_link.getName(), 'and', robot_link.getName()
                    return False

        # update planning model of the world
        self.update_world_with_knowledge()

        # check collisions with rigid objects and terrains
        objs = [ self.world.rigidObject(i) for i in range(self.world.numRigidObjects()) ]
        terrains = [ self.world.terrain(i) for i in range(self.world.numTerrains()) ]

        # remove all the ignored objects
        ignore_ids = [ o.klampt_index for o in ignore ]
        objs = filter(lambda o: o.getID() not in ignore_ids, objs)

        # now do collision checking
        for collidable in objs + terrains:
            for (_, arm_link) in self.arm_links[limb]:
#                 if collidable.geometry().collides(arm_link.geometry()):
                if arm_link.geometry().collides(collidable.geometry()):
#                     print 'collision between', arm_link.getName(), 'and', collidable.getName()
                    return False

        return True

    def check_limb_collision_free(self,limb,limbconfig,ignore=None):
        """Checks whether the given 7-DOF limb configuration is collision
        free, keeping the rest of self.robot fixed."""
        q = self.robot.getConfig()
        self.set_limb_config(limb,limbconfig,q)
        self.robot.setConfig(q)
        return self.check_collision_free(limb,ignore)

    def plan_limb(self,limb,limbstart,limbgoal,ignore=None):
        """Returns a 7-DOF milestone path for the given limb to move from the
        start to the goal, or False if planning failed"""

        # update the ignored opbjects list
        self.cs[limb].ignore = ignore

        mp = MotionPlan(self.cs[limb])

        try:
            mp.setEndpoints(limbstart, limbgoal)
        # need this to catch the RuntimeError raised when the goal is infeasible
        except RuntimeError as e:
            print 'Error:', e
            return None

        # try 10 rounds of 10 iterations
        # if a path isn't found in 50 iterations, report failure
        # so a new IK solution is attempted
        for i in range(10):
            mp.planMore(10)
            path = mp.getPath()
            # give the simulation a chance to keep up
            time.sleep(0.01)
            if path:
                break

        # clean up
        mp.close()

        if path:
            # ensure the path starts with the initial position
            if path[0] != limbstart:
                path.insert(0, limbstart)

            for q in path:
                if len(q) < len(limbstart):
                    print 'fix length'
                    q.extend([0]*(len(limbstart) - len(q)))
                if not self.check_limb_collision_free(limb, q, ignore):
                    print 'collision (coarse)!'
                    return None

            # the robot can't keep up if the path is too coarsely sampled
            # thus, linearly interpolate between "distant" poses in joint space
            fine_path = self.resample_path(path, pi/50)

            for q in fine_path:
                # it seems that the path planner sometimes produces motion plans
                # that when finely linearly interpolated have collision states
                # thus, check for those collisions and reject the plan
                if not self.check_limb_collision_free(limb, q, ignore):
                    print 'collision (fine)!'
                    return None

            return fine_path
        else:
            return None

    def resample_path(self, path, granularity):
        path = numpy.array(path)

        fine_path = []
        for i in range(len(path)-1):
            # find the maximum distance in any joint coordinate
            steps = ceil(numpy.amax(path[i+1] - path[i]) / granularity)
            # interpolate the waypoint such that the largest joint change is small
            mini_path = numpy.array([ numpy.linspace(qi, qg, steps, False) for (qi, qg) in zip(path[i], path[i+1]) ]).T
            # add this to the plan
            fine_path.extend(mini_path)

        # add the final goal position
        fine_path.append(path[-1])

        #print len(path), '->', len(fine_path)
        return list(fine_path)

    def plan(self,start,goal,order=['left','right'],ignore=None):
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
                limbpath = self.plan_limb(l,limbstart[l],limbgoal[l],ignore)
                if limbpath is None:
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

    def update_world_with_knowledge(self):
        '''Create objects in the planning for those added to the
        knowledge base.'''

        # look at all the bins
        for (name, contents) in self.knowledge.bin_contents.items():
            # only care above perceived for full bins
            if contents:

                # look at all the items in the bin
                for item in contents:
                    # check which ones are missing planning objects
                    if item.klampt_index is None:
                        # create an object for it
                        # code borrowed from spawn_objects_from_ground_truth
                        obj = self.world.makeRigidObject(item.info.name)
                        bmin,bmax = item.info.bmin,item.info.bmax
                        center = vectorops.div(vectorops.add(bmin,bmax),2.0)
                        cube = obj.geometry()
                        if not cube.loadFile(os.path.join(model_dir,"cube.tri")):
                            print "Error loading cube file",os.path.join(model_dir,"cube.tri")
                            exit(1)
                        scale = [bmax[0]-bmin[0],0,0,0,bmax[1]-bmin[1],0,0,0,bmax[2]-bmin[2]]
                        translate = vectorops.sub(bmin,center)
                        cube.transform(scale,translate)
                        obj.setTransform(item.xform[0],item.xform[1])

                        # note this item's corresponding object
                        item.klampt_index = obj.getID()

                        # update the collider
                        self.collider = WorldCollider(self.world)

                        print 'spawned planning model', obj.getName()

