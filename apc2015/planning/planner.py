from klampt import *
from klampt import vectorops,so3,se3,gldraw
from klampt.cspace import CSpace,MotionPlan
from klampt.robotsim import Geometry3D
from klampt.robotcollide import WorldCollider
import baxter_scoop as baxter
import math
from integration import visualization
import os
import time
import sys

from integration import visualization

class CustomCSpace(CSpace):
    def __init__(self):
        CSpace.__init__(self)
        self.extra_feasibility_tests = []
    def feasible(self,q):
        verbose = False#len(self.extra_feasibility_tests)>0
        if verbose: print "Custom CSpace test"
        if not CSpace.feasible(self,q):
            if verbose: print "Failed CSpace test"
            return False
        for index,test in enumerate(self.extra_feasibility_tests):
            if not test(q):
                if verbose: print "Extra feasibility test",index,"returned false, returning configuration infeasible"
                return False
        if verbose: print "Passed"
        return True

class LimbCSpace (CustomCSpace):
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
        CustomCSpace.__init__(self)
        self.planner = planner
        self.limb = limb
        self.robot = self.planner.robot
        id_to_index = dict([(self.robot.getLink(i).getID(),i) for i in range(self.robot.numLinks())])
        if limb=='left':
            self.limb_indices = [id_to_index[i.getID()] for i in planner.left_arm_links]
        else:
            self.limb_indices = [id_to_index[i.getID()] for i in planner.right_arm_links]
        qmin,qmax = self.robot.getJointLimits()
        self.bound = [(qmin[i]-1e-3,qmax[i]+1e-3) for i in self.limb_indices]
        #Tweak me: edge collision checking tolerance
        self.eps = 0.5e-1
        self.numCollisionChecks = 0
        self.collisionCheckTime = 0

    def feasible(self,q):
        qrob = self.robot.getConfig()
        self.planner.set_limb_config(self.limb,q,qrob)
        self.robot.setConfig(qrob)
        if not CustomCSpace.feasible(self,q):
            #print "LimbCSpace.feasible: Configuration is out of bounds"
            return False
        if not self.planner.check_limb_collision_free(self.limb,q):
            #print "LimbCSpace.feasible: Configuration is in collision"
            return False
        if (not self.inbounds(q)):
            return False
        return True

    def inbounds(self,q):
        xloc = self.robot.getLink('left_gripper').getTransform()[1][0]
        yloc = self.robot.getLink('left_gripper').getTransform()[1][1]
        if( math.sqrt(xloc**2 + yloc**2) > 1):
            # print abs(self.robot.getLink('right_gripper').getTransform()[1][1] - self.xval),'leftright'
            return False
        return True

class TransferCSpace (CustomCSpace):
    def __init__(self,planner,limb,object,graspXform,height=None):
        CustomCSpace.__init__(self)
        self.planner = planner
        self.limb = limb
        self.grasp = graspXform
        self.object = object
        self.robot = self.planner.robot
        self.height = height
        id_to_index = dict([(self.robot.getLink(i).getID(),i) for i in range(self.robot.numLinks())])
        if limb=='left':
            self.limb_indices = [id_to_index[i.getID()] for i in planner.left_arm_links]
        else:
            self.limb_indices = [id_to_index[i.getID()] for i in planner.right_arm_links]
        qmin,qmax = self.robot.getJointLimits()
        self.bound = [(qmin[i]-1e-3,qmax[i]+1e-3) for i in self.limb_indices]
        #Tweak me: edge collision checking tolerance
        self.eps = 1e-1

    def feasible(self,q):
        if not CustomCSpace.feasible(self,q):
            # print "TransferCSpace.feasible: Configuration is out of bounds"
            return False
        qrob = self.robot.getConfig()
        self.planner.set_limb_config(self.limb,q,qrob)
        self.robot.setConfig(qrob)

        if (not self.heightOK(q)):
            # print 'height not ok'
            return False

        if not self.planner.check_collision_free_with_object(self.limb,self.object,self.grasp):
            # print "TransferCSpace.feasible: Configuration is in collision"
            return False
        return True

    def heightOK(self,config):
        armLoc = self.robot.getLink('left_gripper').getTransform()[1]
        if(self.height is not None and armLoc[0] > 0.8): #front of shelf
            if( abs(self.robot.getLink('left_gripper').getTransform()[1][2] - self.height) < 0.005):
                # print armLoc,self.height
                return False
        xloc = self.robot.getLink('left_gripper').getTransform()[1][0]
        yloc = self.robot.getLink('left_gripper').getTransform()[1][1]
        if( math.sqrt(xloc**2 + yloc**2) > 1):
            # print abs(self.robot.getLink('right_gripper').getTransform()[1][1] - self.xval),'leftright'
            return False
        return True
class TransferCSpaceRestrict (CustomCSpace):
    def __init__(self,planner,limb,object,graspXform,xrestrict=None):
        CustomCSpace.__init__(self)
        self.planner = planner
        self.limb = limb
        self.grasp = graspXform
        self.object = object
        self.robot = self.planner.robot
        self.xrestrict = xrestrict
        id_to_index = dict([(self.robot.getLink(i).getID(),i) for i in range(self.robot.numLinks())])
        if limb=='left':
            self.limb_indices = [id_to_index[i.getID()] for i in planner.left_arm_links]
        else:
            self.limb_indices = [id_to_index[i.getID()] for i in planner.right_arm_links]
        qmin,qmax = self.robot.getJointLimits()
        self.bound = [(qmin[i]-1e-3,qmax[i]+1e-3) for i in self.limb_indices]
        #Tweak me: edge collision checking tolerance
        self.eps = 1e-1

    def feasible(self,q):
        if not CustomCSpace.feasible(self,q):
            # print "TransferCSpace.feasible: Configuration is out of bounds"
            return False
        qrob = self.robot.getConfig()
        self.planner.set_limb_config(self.limb,q,qrob)
        self.robot.setConfig(qrob)
        if not self.planner.check_collision_free_with_object_base(self.limb,self.object,self.grasp):
            # print "TransferCSpace.feasible: Configuration is in collision"
            return False
        if (not self.isScoopInBounds(q)):
            return False
        return True

    def isScoopInBounds(self,config):
        #check to see if the rotation matrix from the last link is within
        q = self.robot.getConfig()
        self.planner.set_limb_config('left',config,q)
        self.robot.setConfig(q)
        xloc = self.robot.getLink('left_gripper').getTransform()[1][0]
        yloc = self.robot.getLink('left_gripper').getTransform()[1][1]
        itemLoc = se3.mul(self.robot.getLink('left_gripper').getTransform(),([0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0], [-0.04, 0.0, 0.2]))
        if( math.sqrt(xloc**2 + yloc**2) > 1):
            # print abs(self.robot.getLink('right_gripper').getTransform()[1][1] - self.xval),'leftright'
            return False
        if(self.xrestrict is not None):#TODO:this will break when we actually need it to work
            if( itemLoc[1][0] - self.xrestrict < -0.01):
                if(self.robot.getLink('left_gripper').getTransform()[1][2] > -.3):
                    # print 'xloclation',itemLoc[1][0]-self.xrestrict
                    # print 'zlocation',self.robot.getLink('left_gripper').getTransform()[1][2]
                    return False

        if(itemLoc[1][2] < -0.5 and (itemLoc[1][1] > 0.40 or itemLoc[1][1] < -0.4)): #keep the item above the bin
            # print 'item below bin'
            return False
        return True

class CSpaceRestrictX (CustomCSpace):
    def __init__(self,planner,limb,xrestrict=None):
        CustomCSpace.__init__(self)
        self.planner = planner
        self.limb = limb
        self.robot = self.planner.robot
        self.xrestrict = xrestrict
        id_to_index = dict([(self.robot.getLink(i).getID(),i) for i in range(self.robot.numLinks())])
        if limb=='left':
            self.limb_indices = [id_to_index[i.getID()] for i in planner.left_arm_links]
        else:
            self.limb_indices = [id_to_index[i.getID()] for i in planner.right_arm_links]
        qmin,qmax = self.robot.getJointLimits()
        self.bound = [(qmin[i]-1e-3,qmax[i]+1e-3) for i in self.limb_indices]
        #Tweak me: edge collision checking tolerance
        self.eps = 1e-1

    def feasible(self,q):
        if not CustomCSpace.feasible(self,q):
            # print "TransferCSpace.feasible: Configuration is out of bounds"
            return False
        qrob = self.robot.getConfig()
        self.planner.set_limb_config(self.limb,q,qrob)
        self.robot.setConfig(qrob)
        if not self.planner.check_limb_collision_free(self.limb,q):
            # print "TransferCSpace.feasible: Configuration is in collision"
            return False
        if (not self.isScoopInBounds(q)):
            return False
        return True

    def isScoopInBounds(self,config):
        #check to see if the rotation matrix from the last link is within
        q = self.robot.getConfig()
        self.planner.set_limb_config('left',config,q)
        self.robot.setConfig(q)
        xloc = self.robot.getLink('left_gripper').getTransform()[1][0]
        yloc = self.robot.getLink('left_gripper').getTransform()[1][1]
        if( math.sqrt(xloc**2 + yloc**2) > 1):
            # print abs(self.robot.getLink('right_gripper').getTransform()[1][1] - self.xval),'leftright'
            return False
        if(self.xrestrict is not None):
            if( abs(self.robot.getLink('left_gripper').getTransform()[1][0] - self.xrestrict) > 0.015):
                return False
        return True

class TransferCSpaceUpright (CustomCSpace):
    def __init__(self,planner,limb,object,graspXform):
        CustomCSpace.__init__(self)
        self.planner = planner
        self.limb = limb
        self.grasp = graspXform
        self.object = object
        self.robot = self.planner.robot
        id_to_index = dict([(self.robot.getLink(i).getID(),i) for i in range(self.robot.numLinks())])
        if limb=='left':
            self.limb_indices = [id_to_index[i.getID()] for i in planner.left_arm_links]
        else:
            self.limb_indices = [id_to_index[i.getID()] for i in planner.right_arm_links]
        qmin,qmax = self.robot.getJointLimits()
        self.bound = [(qmin[i]-1e-3,qmax[i]+1e-3) for i in self.limb_indices]
        #Tweak me: edge collision checking tolerance
        self.eps = 1e-1

    def feasible(self,q):
        if not CustomCSpace.feasible(self,q):
            # print "TransferCSpace.feasible: Configuration is out of bounds"
            return False
        qrob = self.robot.getConfig()
        self.planner.set_limb_config(self.limb,q,qrob)
        self.robot.setConfig(qrob)
        if not self.planner.check_collision_free_with_object_base(self.limb,self.object,self.grasp):
            # print "TransferCSpace.feasible: Configuration is in collision"
            return False
        if (not self.isScoopInBounds(q)):
            # print 'tray not upright'
            return False

        return True

    def isScoopInBounds(self,config):
        #check to see if the rotation matrix from the last link is within
        q = self.robot.getConfig()
        self.planner.set_limb_config('left',config,q)
        self.robot.setConfig(q)
        xloc = self.robot.getLink('left_gripper').getTransform()[1][0]
        yloc = self.robot.getLink('left_gripper').getTransform()[1][1]
        if( math.sqrt(xloc**2 + yloc**2) > 1):
            # print abs(self.robot.getLink('right_gripper').getTransform()[1][1] - self.xval),'leftright'
            return False
        if(self.robot.getLink('left_gripper').getTransform()[0][4] > 0.85 and self.robot.getLink('left_gripper').getTransform()[0][4] < 0.95):
            # print self.robot.getLink('left_gripper').getTransform()[0]
            return False
        return True

class ScoopCSpace (CustomCSpace):
    def __init__(self,planner,limb):
        CustomCSpace.__init__(self)
        self.planner = planner
        self.limb = limb
        self.robot = self.planner.robot
        id_to_index = dict([(self.robot.getLink(i).getID(),i) for i in range(self.robot.numLinks())])
        if limb=='left':
            self.limb_indices = [id_to_index[i.getID()] for i in planner.left_arm_links]
        else:
            self.limb_indices = [id_to_index[i.getID()] for i in planner.right_arm_links]
        qmin,qmax = self.robot.getJointLimits()
        self.bound = [(qmin[i]-1e-3,qmax[i]+1e-3) for i in self.limb_indices]
        #Tweak me: edge collision checking tolerance
        self.eps = 1e-1
        self.numCollisionChecks = 0
        self.collisionCheckTime = 0

    def feasible(self,q):
        if not CustomCSpace.feasible(self,q):
            #print "LimbCSpace.feasible: Configuration is out of bounds"
            return False
        if not self.planner.check_limb_collision_free(self.limb,q):
            # print "LimbCSpace.feasible: Configuration is in collision"
            return False
        if not self.isScoopUpright(q):
            # print 'Scoop is not upright'
            return False

        return True
    def isScoopUpright(self,config):
        #check to see if the rotation matrix of the scoop is correct
        q = self.robot.getConfig()
        self.planner.set_limb_config('right',config,q)
        self.robot.setConfig(q)
        slop = 0.1
        rotMatrix = self.robot.getLink('right_gripper').getTransform()[0]
        # print 'rotation matrix',rotMatrix
        #[0,1,0,0,0,1,1,0,0]
        
        if(rotMatrix[5] > 1 + slop or rotMatrix[5] < 1 - slop):
            return False
        
        return True

class HeightCSpace (CustomCSpace):
    def __init__(self,planner,limb,height,xval):
        CustomCSpace.__init__(self)
        self.planner = planner
        self.limb = limb
        self.robot = self.planner.robot
        self.minHeight = height
        self.xval = xval
        id_to_index = dict([(self.robot.getLink(i).getID(),i) for i in range(self.robot.numLinks())])
        if limb=='left':
            self.limb_indices = [id_to_index[i.getID()] for i in planner.left_arm_links]
        else:
            self.limb_indices = [id_to_index[i.getID()] for i in planner.right_arm_links]
        qmin,qmax = self.robot.getJointLimits()
        self.bound = [(qmin[i]-1e-3,qmax[i]+1e-3) for i in self.limb_indices]
        #Tweak me: edge collision checking tolerance
        self.eps = 1e-1
        self.numCollisionChecks = 0
        self.collisionCheckTime = 0

    def feasible(self,q):
        if not CustomCSpace.feasible(self,q):
           #print "LimbCSpace.feasible: Configuration is out of bounds"
           return False
        if not self.planner.check_limb_collision_free(self.limb,q):
            # print "LimbCSpace.feasible: Configuration is in collision"
            return False
        if not self.isScoopHighEnough(q):
            # print 'Scoop is not high enough'
            return False
        # if not self.isScoopUpright(q):
        #     print 'Scoop is not upright'
        #     return False

        return True
    def isScoopUpright(self,config):
        #check to see if the rotation matrix from the last link is within
        q = self.robot.getConfig()
        self.planner.set_limb_config('right',config,q)
        self.robot.setConfig(q)
        if(self.robot.getLink('right_gripper').getTransform()[0][2] < .85):
            # print self.robot.getLink('right_gripper').getTransform()[0][2],'upright'
            return False
            #straight line code 
        if( abs(self.robot.getLink('right_gripper').getTransform()[1][1] - self.xval) >= 0.01):
            # print abs(self.robot.getLink('right_gripper').getTransform()[1][1] - self.xval),'leftright'
            return False
        return True

    def isScoopHighEnough(self,config):
        #check to see if the rotation matrix from the last link is within
        q = self.robot.getConfig()
        self.planner.set_limb_config('right',config,q)
        self.robot.setConfig(q)
        if(self.robot.getLink('right_gripper').getTransform()[1][2] +.01 < self.minHeight):
            # print self.minHeight,self.robot.getLink('right_gripper').getTransform()[1][2]
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
        self.left_camera_link = self.robot.getLink(baxter.left_camera_link_name)
        self.right_camera_link = self.robot.getLink(baxter.right_camera_link_name)
        self.left_gripper_link = self.robot.getLink(baxter.left_gripper_link_name)
        self.right_gripper_link = self.robot.getLink(baxter.right_gripper_link_name)
        self.left_arm_links = [self.robot.getLink(i) for i in baxter.left_arm_link_names]
        self.right_arm_links = [self.robot.getLink(i) for i in baxter.right_arm_link_names]
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

    def check_collision_free(self,limb,exclude=None,verbose=False):
        """Checks whether the given limb is collision free at the robot's
        current configuration"""
        exclude = exclude or []
        armfilter = None
        if limb=='left':
            collindices = set(baxter.left_arm_geometry_indices+baxter.left_hand_geometry_indices)
            handindices = baxter.left_hand_geometry_indices
        else:
            collindices = set(baxter.right_arm_geometry_indices+baxter.right_hand_geometry_indices)
            handindices = baxter.right_hand_geometry_indices
        armfilter = lambda x:isinstance(x,RobotModelLink) and (x.index in collindices)
        #check with objects in world model
        for o1,o2 in self.collider.collisionTests(armfilter,lambda x:True):
            #ignore hand self collisions
            if isinstance(o1[0],RobotModelLink) and o1[0].index in handindices and isinstance(o2[0],RobotModelLink) and o2[0].index in handindices:
                continue
            if o1[0].getID() not in exclude and o2[0].getID() not in exclude:
                if o1[1].collides(o2[1]):
                    if verbose:
                        print "Collision between",o1[0].getName(),o2[0].getName()
                    return False
        
        return True

    def check_collision_free_with_object(self,limb,object,grasp):
        """Checks whether the given limb, holding an object at the given
        grasp, is collision free at the robot's current configuration"""
        objToGripperXForm = se3.mul(baxter.left_gripper_center_xform,se3.inv(grasp))
        #assume robot configuration is updated
        if limb=='left':
            gripperLink = self.robot.getLink(baxter.left_gripper_link_name)
        else:
            gripperLink = self.robot.getLink(baxter.right_gripper_link_name)

        if not isinstance(object,list):
            if object==None:
                object = []
            else:
                object = [object]

        Tgripper = gripperLink.getTransform()
        Tobj = se3.mul(Tgripper,objToGripperXForm)
        for o in object:
            o.setTransform(*Tobj)
        
        excludeids = [ o.getID() for o in object]
        if not self.check_collision_free(limb,exclude=excludeids):
            #print 'Limb is not collision free'
            return False

        for t in xrange(self.world.numRigidObjects()):
            other = self.world.rigidObject(t)
            if(other.getID() not in excludeids):
                if any(other.geometry().collides(o.geometry()) for o in object):
                    #visualization.debug(self.world)
                    #print "Held object-shelf collision"
                    return False
        return True

    def check_limb_collision_free(self,limb,limbconfig):
        """Checks whether the given 7-DOF limb configuration is collision
        free, keeping the rest of self.robot fixed."""
        q = self.robot.getConfig()
        self.set_limb_config(limb,limbconfig,q)
        self.robot.setConfig(q)
        return self.check_collision_free(limb)

    def plan_limb(self,limb,limbstart,limbgoal,Cspace='Normal',argsTuple=None,extra_feasibility_tests=[]):
        """Returns a 7-DOF milestone path for the given limb to move from the
        start to the goal, or False if planning failed"""
        if(Cspace == 'scoop'):
            cspace = ScoopCSpace(self,limb)
        elif(Cspace == 'height'):
            cspace = HeightCSpace(self,limb,argsTuple[0],argsTuple[1])
        elif(Cspace == 'xrestrict'):
            cspace = CSpaceRestrictX(self,limb,xrestrict=argsTuple[0])
        else:
            cspace = LimbCSpace(self,limb)
        cspace.extra_feasibility_tests = extra_feasibility_tests
        if not cspace.feasible(limbstart):
            print "  Start configuration is infeasible!"
            visualization.debug(self.world)
            return False
        if not cspace.feasible(limbgoal):
            print "  Goal configuration is infeasible!"
            visualization.debug(self.world)
            return False
        MotionPlan.setOptions(connectionThreshold=3.5,perturbationRadius=1)
        MotionPlan.setOptions(shortcut=1)
        plan = MotionPlan(cspace,'sbl')
        plan.setEndpoints(limbstart,limbgoal)
        maxPlanIters = 2000
        maxSmoothIters = 50
        for iters in xrange(maxPlanIters):
            plan.planMore(1)
            path = plan.getPath()
            if path != None:
                #print "  Found a path on iteration",iters
                if len(path) >= 2 and maxSmoothIters > 0:
                    #print "  Smoothing..."
                    plan.planMore(min(maxPlanIters-iters,maxSmoothIters))
                    path = plan.getPath()
                cspace.close()
                plan.close()
                #print "Average collision check time",cspace.collisionCheckTime/cspace.numCollisionChecks
                #print cspace.numCollisionChecks,"collision checks"
                return path
        #print "Average collision check time",cspace.collisionCheckTime/cspace.numCollisionChecks
        #print cspace.numCollisionChecks,"collision checks"
        cspace.close()
        plan.close()
        #print "  No path found"
        return False

    def plan(self,start,goal,order=['left','right'],Cspace='Normal',argsTuple=None,extra_feasibility_tests=[]):
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
                #print "Planning for limb",l
                #print "  Euclidean distance:",math.sqrt(diff)
                self.robot.setConfig(curconfig)
                #do the limb planning
                if(Cspace == 'scoop'):
                    limbpath = self.plan_limb(l,limbstart[l],limbgoal[l],Cspace='scoop',extra_feasibility_tests=extra_feasibility_tests)
                elif(Cspace == 'height'):
                    limbpath = self.plan_limb(l,limbstart[l],limbgoal[l],Cspace='height',argsTuple=argsTuple,extra_feasibility_tests=extra_feasibility_tests)
                elif(Cspace == 'xrestrictNobj'):
                    limbpath = self.plan_limb(l,limbstart[l],limbgoal[l],Cspace='xrestrict',argsTuple=argsTuple,extra_feasibility_tests=extra_feasibility_tests)
                else:
                    limbpath = self.plan_limb(l,limbstart[l],limbgoal[l],extra_feasibility_tests=extra_feasibility_tests)
                if limbpath == False:
                    #print "  Failed to plan for limb",l
                    return None
                #print "   Planned successfully for limb",l
                #concatenate whole body path
                for qlimb in limbpath[1:]:
                    q = path[-1][:]
                    self.set_limb_config(l,qlimb,q)
                    path.append(q)
                self.set_limb_config(l,limbgoal[l],curconfig)
        return path

    def plan_limb_transfer(self,limb,limbstart,limbgoal,heldobject,grasp,type='Normal',xrestrict=None,extra_feasibility_tests=[]):
        """Returns a 7-DOF milestone path for the given limb to move from the
        start to the goal while holding the given object.
        Returns False if planning failed"""
        if(type == 'Normal'):
            cspace = TransferCSpace(self,limb,heldobject,grasp,height=xrestrict)
        elif(type == 'restrict'):
            cspace = TransferCSpaceRestrict(self,limb,heldobject,grasp)
        elif(type == 'upright'):
            cspace = TransferCSpaceUpright(self,limb,heldobject,grasp)
        elif(type == 'xrestrict'):
            cspace = TransferCSpaceRestrict(self,limb,heldobject,grasp,xrestrict)
        cspace.extra_feasibility_tests = extra_feasibility_tests
        if not cspace.feasible(limbstart):
            print "  Start configuration is infeasible!"
            return False
        if not cspace.feasible(limbgoal):
            print "  Goal configuration is infeasible!"
            return False
        MotionPlan.setOptions(connectionThreshold=3.5,perturbationRadius=1)
        MotionPlan.setOptions(shortcut=1)
        plan = MotionPlan(cspace,'sbl')
        plan.setEndpoints(limbstart,limbgoal)
        maxPlanIters = 2000
        maxSmoothIters = 50
        for iters in xrange(maxPlanIters):
            plan.planMore(1)
            path = plan.getPath()
            if path != None:
                #print "  Found a path on iteration",iters
                if len(path) > 2 and maxSmoothIters > 0:
                    #print "  Smoothing..."
                    plan.planMore(min(maxPlanIters-iters,maxSmoothIters))
                    path = plan.getPath()
                cspace.close()
                plan.close()
                return path
        cspace.close()
        plan.close()
        #print "No path found"
        return False

    def plan_transfer(self,start,goal,limb,heldobject,grasp,type='Normal',xrestrict=None,extra_feasibility_tests=[]):
        """Plans a motion for the robot to move from configuration start
        to configuration goal while holding the given object."""
        limbstart = self.get_limb_config(start,limb)
        limbgoal = self.get_limb_config(goal,limb)
        path = [start]
        #print "Planning transfer path for limb",limb
        self.robot.setConfig(start)
        #do the limb planning
        limbpath = self.plan_limb_transfer(limb,limbstart,limbgoal,heldobject,grasp,type,xrestrict,extra_feasibility_tests=extra_feasibility_tests)
        if limbpath == False:
            #print "  Failed to plan transfer path for limb",limb
            return None
        #print "  Planned transfer path successfully for limb",limb
        #concatenate whole body path
        for qlimb in limbpath[1:]:
            q = path[-1][:]
            self.set_limb_config(limb,qlimb,q)
            path.append(q)
        return path


    def updateWorld(self,new_world):
        self.world = new_world
        self.collider = WorldCollider(new_world)

    def check_collision_free_with_object_base(self,limb,object,grasp):
        """Checks whether the given limb, holding an object at the given
        grasp, is collision free at the robot's current configuration"""
        objToGripperXForm = se3.mul(baxter.left_gripper_center_xform,se3.inv(grasp))
        #assume robot configuration is updated
        if limb=='left':
            gripperLink = self.robot.getLink(baxter.left_gripper_link_name)
        else:
            gripperLink = self.robot.getLink(baxter.right_gripper_link_name)

        if not isinstance(object,list):
            if object==None:
                object = []
            else:
                object = [object]

        Tgripper = gripperLink.getTransform()
        Tobj = se3.mul(Tgripper,objToGripperXForm)
        for o in object:
            o.setTransform(*Tobj)
        
        excludeids = [ o.getID() for o in object]
        if not self.check_collision_free_hand(limb,exclude=excludeids):
            #print 'Limb is not collision free'
            return False

        #if not self.check_collision_free(limb,exclude=excludeids):
        #    #print 'Limb is not collision free'
        #    return False

        for t in xrange(self.world.numRigidObjects()):
            other = self.world.rigidObject(t)
            if(other.getID() not in excludeids):
                if any(other.geometry().collides(o.geometry()) for o in object):
                    #visualization.debug(self.world)
                    #print "Held object-shelf collision"
                    return False
        return True

    def check_collision_free_hand(self,limb,exclude=None,verbose=False):
        """Checks whether the given limb is collision free at the robot's
        current configuration"""
        exclude = exclude or []
        armfilter = None
        if limb=='left':
            collindices = set(baxter.left_arm_geometry_indices+baxter.left_hand_geometry_indices)
            handindices = baxter.left_hand_geometry_indices
        else:
            collindices = set(baxter.right_arm_geometry_indices+baxter.right_hand_geometry_indices)
            handindices = baxter.right_hand_geometry_indices
        
        #add in the base
        collindices.add(0)
        collindices.add(1)
        collindices.add(2)
        collindices.add(3)
        collindices.add(4)
        collindices.add(5)

        armfilter = lambda x:isinstance(x,RobotModelLink) and (x.index in collindices)

        #check with objects in world model
        for o1,o2 in self.collider.collisionTests(armfilter,lambda x:True):
            #ignore hand self collisions
            
            if isinstance(o1[0],RobotModelLink) and o1[0].index in handindices and isinstance(o2[0],RobotModelLink) and o2[0].index in handindices:
                continue
            if o1[0].getID() not in exclude and o2[0].getID() not in exclude:
                if o1[1].collides(o2[1]):
                    if verbose:
                        print "Collision between",o1[0].getName(),o2[0].getName()
                    return False



            if(not isinstance(o1[0],RobotModelLink) and o2[0].index not in handindices):
                print 'checking object with base'
                if o1[1].collides(o2[1]):
                    if verbose:
                        print "Collision between",o1[0].getName(),o2[0].getName()
                    return False
            if(not isinstance(o2[0],RobotModelLink) and o1[0].index not in handindices):
               
                if o1[1].collides(o2[1]):
                    if verbose:
                        print "Collision between",o1[0].getName(),o2[0].getName()
                    return False        
        return True
