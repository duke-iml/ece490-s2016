import sys
import logging, traceback
from integration import visualization
logger = logging.getLogger(__name__)

import os
import baxter_scoop as baxter

sys.path.insert(0, '/home/motion/ece590-s2015/apc')
sys.path.insert(0, '')

from klampt import WorldModel
from api.planning import PlanningInterface
from planner import LimbPlanner, LimbCSpace, TransferCSpace
from klampt import ik, vectorops, se3, so3, resource, GeometricPrimitive, IKObjective
import json
import random
import math
import time
import numpy

from integration.io import pcd

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class RealPlanningInterface(PlanningInterface):
    def __init__(self, knowledge_base):
        # Super init
        PlanningInterface.__init__(self, knowledge_base)
        self.objectDict = {}
        self.virtualObjectDict = {}
        # Create planning world
        self.world = self._populate_world()
        self.robot = self.world.robot(0)

        #set up the collision planner
        self.planner = LimbPlanner(self.world, self.knowledge_base)

        self.left_camera_link = self.robot.getLink(baxter.left_camera_link_name)
        self.right_camera_link = self.robot.getLink(baxter.right_camera_link_name)
        self.left_gripper_link = self.robot.getLink(baxter.left_gripper_link_name)
        self.right_gripper_link = self.robot.getLink(baxter.right_gripper_link_name)
        self.left_arm_links = [self.robot.getLink(i) for i in baxter.left_arm_link_names]
        self.right_arm_links = [self.robot.getLink(i) for i in baxter.right_arm_link_names]
        self.left_arm_indices = [i.index for i in self.left_arm_links]
        self.right_arm_indices = [i.index for i in self.right_arm_links]
        graspRot = so3.mul([0, 0, 1, 0, 1, 0, 1, 0, 0],so3.rotation([0,1,0],math.pi/180*-20))
        self.trayGraspXform = graspRot,[-.04,0,0]

    def _populate_world(self):
        model_dir = "klampt_models/"


        """Produces a world with only the Baxter in it."""
        world = WorldModel()
        logger.debug("Loading simplified Baxter model...")
        world.loadElement(os.path.join(model_dir, baxter.klampt_model_name))
        Rbase, tbase = world.robot(0).getLink(0).getParentTransform()
        world.robot(0).setConfig(self.robot_state.commanded_config)

        if self.knowledge_base.shelf_xform:
            shelf = world.loadRigidObject('klampt_models/north_shelf/shelf_with_bins.obj')
            shelf.setTransform(*self.knowledge_base.shelf_xform)
            # logger.debug('spawned shelf for planning')
            self.shelf = shelf
        else:
            self.shelf = None

        if self.knowledge_base.order_bin_xform:
            order_bin = world.loadRigidObject('klampt_models/north_order_bin/order_bin.obj')
            order_bin.setTransform(*self.knowledge_base.order_bin_xform)
            # logger.debug('spawned order bin for planning')

        for (name, xform) in self.knowledge_base.object_xforms.items():
            #put all objects that are in the bin into the world model so that we can do collision checking
            #object = world.loadRigidObject('klampt_models/items/' + name + '/meshes/poisson.ply')
            object = world.makeRigidObject('{}_cloud'.format(name))

            pcd.write(self.knowledge_base.object_clouds[name], '/tmp/object.pcd')
            object.geometry().loadFile('/tmp/object.pcd')
            object.geometry().setCollisionMargin(0.005)
            #transform the object onto the 
            self.objectDict[name] = object
            object.setTransform(*xform)
            # logger.debug('spawned {} for planning'.format(name))

            #put a hallucinated bounding box around each object
            objectbbox = world.makeRigidObject('{}_estimated_bbox'.format(name))
            objectPCloud = self.knowledge_base.object_clouds[name]
            objectPCloud = map(lambda p: p[:3], objectPCloud)

            # #find the min, max, and mean for x,y,z
            mean = [0,0,0]
            minPt = [1000,1000,1000]
            maxPt = [-1000,-1000,-1000]
            for p in objectPCloud:
                mean = vectorops.add(mean,p)
                if p[0] < minPt[0]:
                    minPt[0]=p[0]
                elif p[0] > maxPt[0]:
                    maxPt[0]=p[0]
                if p[1] < minPt[1]:
                    minPt[1]=p[1]
                elif p[1] > maxPt[1]:
                    maxPt[1]=p[1]
                if p[2] < minPt[2]:
                    minPt[2]=p[2]
                elif p[2] > maxPt[2]:
                    maxPt[2]=p[2]
            dims = vectorops.sub(maxPt,minPt)
            maxindex = max((d,i) for i,d in enumerate(dims))[1]
            if dims[maxindex] > self.knowledge_base.object_maximum_dims[name]:
                excess = dims[maxindex] - self.knowledge_base.object_maximum_dims[name]
                minPt[maxindex] += excess*0.5
                maxPt[maxindex] -= excess*0.5
            for i in range(3):
                if i != maxindex:
                    excess = dims[i] - self.knowledge_base.object_minimum_dims[name]
                    minPt[i] += excess*0.5
                    maxPt[i] -= excess*0.5
            objectbbox.geometry().loadFile("klampt_models/cube.tri")
            objectbbox.geometry().transform(vectorops.mul(so3.identity(),1),[-0.5]*3)
            objectbbox.geometry().transform([maxPt[0]-minPt[0],0,0,0,maxPt[1]-minPt[1],0,0,0,maxPt[2]-minPt[2]],[0,0,0])
            objectbbox.geometry().translate(vectorops.mul(vectorops.add(minPt,maxPt),0.5))
            self.virtualObjectDict[name] = objectbbox
 
        #add the tray to the gripper
        object = world.loadRigidObject('klampt_models/tray/tray.ply')
        self.objectDict['tray'] = object
        trayRotMatrix = so3.mul([0,0,-1,1,0,0,0,-1,0],so3.rotation([0,1,0],math.pi/180.0*-10))
        xform = (trayRotMatrix,[0.17, 0.552327180434089, -0.82])
        object.setTransform(*xform)

        #DEBUG THE BOXES
        #visualization.debug(world)
        return world

    def planMoveToVantagePoint(self, bin, vantage_point):
        """
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
        """
        # #check to see if we are in any initial configurations
        # startConfig = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.1, 0.0, 2.358, 0.0, 0.0, 0.3, -0.98, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.18, 0.0, 2.118, 0.04, 0.0, 0.62, 0.901, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 

        if bin == 'bin_C' or bin == 'bin_F' or bin == 'bin_I' or bin == 'bin_L' or bin=='bin_B' or bin=='bin_E':
            limb = 'right'
            limbind = self.right_arm_indices
        else:
            limb = 'left'
            limbind = self.left_arm_indices
        logger.info(limb)

        # atStart = True
        # logger.debug('Checking current configuration')
        # for i,val in enumerate(self.robot_state.commanded_config):

        #     if(val - startConfig[i] > 0.05 and (i in limbind) ):
        #         #not in start config dont do cached planning
        #         logger.debug('Not in start configuration')
        #         atStart = False
        #         break

        # if(atStart):
        #     logger.debug('In start configuration. Looking for cached plans')
        #     #which bin are we moving to?
        #     if bin == 'bin_C' or bin == 'bin_F' or bin == 'bin_I' or bin == 'bin_L':
        #         limb = 'right'
        #     else:
        #         limb = 'left'
        #     #we are at the initial config. look up cached plans
        #     #check if a cached plan exists
        #     fileLocation = 'planning/cachedPlans/'+limb+'_from_home_to_'+bin + '.json'
        #     path = None

        #     try:
        #         pathFile = open(fileLocation)
        #         path = json.load(pathFile)
        #         goodness = 1
        #         logger.debug('Found a cached plan. Checking for collisions')
        #         noCollision = True
        #         #check if this path is in collision
        #         for config in path:
        #             #set the planenr's robot config
        #             self.planner.robot.setConfig(config)
        #             noCollision = self.planner.check_collision_free(limb)
        #             if(noCollision == False):
        #                 #there was a collision in the cached solution
        #                 #must plan from scratch
        #                 logger.debug('Cached plan has collision. Planning from scratch...')
        #                 break

        #         if(noCollision):
        #             logger.debug('Cached plan was collision free. Returning cached plan')
        #             return ([('path', path,0, 0),('left_gripper',[0.8, 0.8, 0.8, 1],0,0)],1,limb,None)


        #     except IOError:
        #         logger.debug('IO Error Could not find cached plan: {}. Planning from scratch...'.format(fileLocation))
        #         pass

        # #check to see if we are in another config
        # startConfigs = []
        # fileLocations = ['left_from_bin_home_to_bin_A.json','right_from_bin_A_to_bin_C.json',
        #                 'left_from_bin_C_to_bin_D.json','right_from_bin_D_to_bin_F.json',
        #                 'left_from_bin_F_to_bin_G.json','right_from_bin_G_to_bin_I.json',
        #                 'left_from_bin_I_to_bin_J.json','right_from_bin_J_to_bin_L.json',
        #                 'left_from_bin_L_to_bin_K.json','left_from_bin_K_to_bin_H.json',
        #                 'left_from_bin_H_to_bin_E.json','left_from_bin_E_to_bin_B.json']
        # for file in fileLocations:
        #     pathFile = open('planning/cachedPlans/'+file)
        #     path = json.load(pathFile)
        #     startConfigs.append(path)


        # for j,cachedConfig in enumerate(startConfigs):
        #     atStart = True
        #     for i,val in enumerate(self.robot_state.commanded_config):
        #         if(val - cachedConfig[0][i] > 0.05 ):
        #             #not in start config dont do cached planning
        #             logger.debug('Not in cached configuration %i',j)
        #             atStart = False
        #             break
        #     if(atStart):
        #         #return the cached plan
        #         for config in cachedConfig:
        #             #set the planenr's robot config
        #             self.planner.robot.setConfig(config)
        #             noCollision = self.planner.check_collision_free(limb)
        #             if(noCollision == False):
        #                 #there was a collision in the cached solution
        #                 #must plan from scratch
        #                 logger.debug('Cached plan has collision. Planning from scratch...')
        #                 break

        #         if(noCollision):
        #             logger.debug(bcolors.OKBLUE+'Cached plan was collision free. Returning cached plan %s'+bcolors.ENDC,j)
        #             return ([('path', cachedConfig,0, 0),('left_gripper',[0.8, 0.8, 0.8, 1],0,0)],1,limb,None)

        path = None
        goodness = None

        #world offset is where we want the camera to go in x,y,z
        #world_offset = se3.apply(self.knowledge_base.shelf_xform, self.knowledge_base.vantage_point_xforms[vantage_point][1])

        #set up the goal
        #goal_xform = ([0,0,-1,1,0,0,0,1,0],world_offset);

        goal_xform = se3.mul(self.knowledge_base.shelf_xform, self.knowledge_base.vantage_point_xforms[vantage_point])

        #find the plan if it exists
        pathToVantagePoint = self.planMoveToXform(goal_xform=goal_xform,
                                    limb=limb,
                                    object=None,
                                    link='camera',
                                    colMargin=0.05,
                                    collisionchecker=None,
                                    initialConfig=None,
                                    plan=True)

        #visualization.debug(self.world,[self.left_camera_link.getTransform()])
        if(pathToVantagePoint is None):
            logger.error('I failed at my only job. Trying to move to %s ',bin)
            return None
        if limb=='left':
            return ([('path', pathToVantagePoint,0, 0),('left_gripper',[0.8, 0.8, 0.8, 1],0,0)],1,limb,None)
        else:
            return ([('path', pathToVantagePoint,0, 0)],1,limb,None)


    def planGraspObjectInBin(self, bin, object):
        """
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
        """
        
        #[0,-15,15,-30,30,-60.60]
        r = 0.07
        vertPinchOffset = 0.03
        power_pregrasp_shifts = [[0,0.0,r], #7cm in front of the object
                           [0,r*math.sin(math.pi/180.0*-15),r*math.cos(math.pi/180.0*-15)],
                           [0,r*math.sin(math.pi/180.0*15),r*math.cos(math.pi/180.0*15)],
                           [0,r*math.sin(math.pi/180.0*-30),r*math.cos(math.pi/180.0*-30)],
                           [0,r*math.sin(math.pi/180.0*30),r*math.cos(math.pi/180.0*30)],
                           [0,r*math.sin(math.pi/180.0*-60),r*math.cos(math.pi/180.0*-60)],
                           [0,r*math.sin(math.pi/180.0*60),r*math.cos(math.pi/180.0*60)]]

        pinch_pregrasp_shifts = [[0,-vertPinchOffset,r], #7cm in front of the object
                           [r*math.sin(math.pi/180.0*-15),-vertPinchOffset,r*math.cos(math.pi/180.0*-15)],
                           [r*math.sin(math.pi/180.0*15),-vertPinchOffset,r*math.cos(math.pi/180.0*15)],
                           [r*math.sin(math.pi/180.0*-30),-vertPinchOffset,r*math.cos(math.pi/180.0*-30)],
                           [r*math.sin(math.pi/180.0*30),-vertPinchOffset,r*math.cos(math.pi/180.0*30)],
                           [r*math.sin(math.pi/180.0*-60),-vertPinchOffset,r*math.cos(math.pi/180.0*-60)],
                           [r*math.sin(math.pi/180.0*60),-vertPinchOffset,r*math.cos(math.pi/180.0*60)]]



        graspType = scoopOrGrasp(object,bin,self.knowledge_base)

        if(graspType == 'scoop'):
            return self.planPointCloudScoop(object,bin)
        elif (graspType == 'pinch' or graspType == 'power' or graspType == 'tilt'):
            (gxforms,openConfigs,closedConfigs) = self.planPointCloudGrasp(object,bin,graspType)
        else:
            return None

        if(graspType == 'pinch'):
            pregrasp_shifts = pinch_pregrasp_shifts
        else:
            pregrasp_shifts = power_pregrasp_shifts
        x = range(len(gxforms))
        # random.shuffle(x)
        for i in x:
            openConfig = openConfigs[i]
            closedConfig = closedConfigs[i]
            pregrasp_shift = pregrasp_shifts[i]
            gxform = gxforms[i]
            self.robot.setConfig(self.robot_state.commanded_config)
            
            #store the current robot config
            baxter.set_model_gripper_command(self.robot,self.knowledge_base.active_limb,openConfig)

            #get the config with the gripper open
            gripperOpenConfig = self.robot.getConfig()

                        
            #try to solve ik solution where the open gripper is placed just in front the object

            #set up the grasp goal and pregrasp goal (pregoal)
            Tg = se3.mul(gxform,se3.inv(baxter.left_gripper_center_xform))
            Tpg = se3.mul(gxform,se3.inv((baxter.left_gripper_center_xform[0],vectorops.add(baxter.left_gripper_center_xform[1],pregrasp_shift))))
            Tg = (so3.from_moment(so3.moment(Tg[0])),Tg[1])
            Tpg = (so3.from_moment(so3.moment(Tpg[0])),Tpg[1])
            goal = ik.objective(self.left_gripper_link,R=Tg[0],t=Tg[1])
            pregoal = ik.objective(self.left_gripper_link,R=Tpg[0],t=Tpg[1])
            
            pathToPreGrasp = self.planMoveToXform(goal_xform=Tpg,
                                        limb=self.knowledge_base.active_limb,
                                        link='gripper',
                                        object=None,
                                        initialConfig=gripperOpenConfig,
                                        colMargin=0.005,
                                        collisionchecker=None,
                                        checkCols=True,
                                        plan=True)

            #check if planning failed
            if pathToPreGrasp is None and i != len(gxforms)-1:
                logger.warn("Grasp planning failed -- trying another grasp")
                continue
            elif pathToPreGrasp is None and i == len(gxforms)-1:
                logger.warn("Grasp planning failed")
                break

            ##################debug####################
            # self.robot.setConfig(pathToPreGrasp[-1])
            # self.robot.setConfig(pathToPreGrasp[0][0])
            # objectPCloud = self.knowledge_base.object_clouds[object]
            # objectPCloud = map(lambda p: p[:3], objectPCloud)
            # visualization.debug_cloud([objectPCloud],world=self.world, xforms=[ Tpg, self.right_gripper_link.getTransform(), self.left_gripper_link.getTransform()])
            
            ###########################################
            
            object_id = self.objectDict[object].getID()
            virtual_object_id = self.virtualObjectDict[object].getID()
            # lambda x:self.planner.check_collision_free_with_object(x,object,self.knowledge_base.active_grasp)
            collisionchecker = lambda x:self.planner.check_collision_free(x,exclude=[object_id,virtual_object_id],verbose=False) 

            logger.info('Pre-grasp plan found')
            #plan was found to pre-goal. Find IK solution to goal
            ikSolution = self.planMoveToXform(goal_xform=Tg,
                                         limb=self.knowledge_base.active_limb,
                                         link='gripper',
                                         object=None,
                                         initialConfig=pathToPreGrasp[-1],
                                         colMargin=0,
                                         collisionchecker=collisionchecker,
                                         cspace=('Normal',-10),
                                         checkCols=True,
                                         plan=False)

            if(ikSolution is not None):
                #found an ik solution to the goal. have a grasp
                # logger.info('Found grasp! {}'.format(fileN))
                logger.info(bcolors.BOLD+bcolors.UNDERLINE+bcolors.OKGREEN+'FOUND GRASP!'+bcolors.ENDC)
                
                ###########debug#################
                # self.robot.setConfig(ikSolution[0][0])
                # visualization.debug(self.world)
                #################################
                if self.knowledge_base.active_limb == 'left':
                    gripper_name = 'left_gripper'
                else:
                    gripper_name = 'right_gripper'

                plan = [
                            (gripper_name, openConfig, 0, 0),
                            ('path', pathToPreGrasp, 0, 1),
                            ('path', [ ikSolution[0][0] ], 0, 1),
                            (gripper_name, closedConfig, 0, 0)
                        ]
                return plan,1,self.knowledge_base.active_limb,gxform
            else:
                logger.warn("Couldn't find solution from pregrasp to grasp")


        logger.warn("Grasp Planning failed")
        # #add in the failsafe move to center grasp
        # return self.failsafeplan(bin,object)
        return None


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
        

        #create a goal based on the limb
        if(limb == 'left'):
            #might need to change this order bin Z
            goal = se3.apply(self.knowledge_base.order_bin_xform,[-0.1,0.15,self.knowledge_base.max_drop_height+0.20])
            gripperlink = self.left_gripper_link
        else:
            #get the plans to move tray to order bin and put tray back
            #moveTrayToOrderBinPlan = self.planMoveTrayToOrderBin()
            # putDownTrayPlan = self.planPickUpOrderTray(False)


            #might need to change this order bin Z
            goal = se3.apply(self.knowledge_base.order_bin_xform,[-0.1,-0.15,self.knowledge_base.max_drop_height+0.30])
            gripperlink = self.right_gripper_link

            #see if there is a cached plan
          #   startConfig = 71*[0]
          # #  if(self.knowledge_base.target_bin == 'bin_C'):
          # #      startConfig = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0999999999999996, 0.0, 2.358, 0.0, 0.0, 0.3, -0.9800000000000004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3088284215681351, -0.040525194886402716, 2.990575585189482, 1.3360194679450996, 0.7469085812669755, 0.0, -1.5708000000000002, 2.36765689388818, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
          # #  elif(self.knowledge_base.target_bin == 'bin_F'):
          # #      startConfig = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0999999999999996, 0.0, 2.358, 0.0, 0.0, 0.3, -0.9800000000000004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.30158718316595046, 0.1653559427347274, 2.2363132059607045, 1.834400059027026, 1.0639366601020779, 0.0, -1.0132113159072906, 2.7856916491114374, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
          # #  elif(self.knowledge_base.target_bin == 'bin_I'):
          # #      startConfig = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0999999999999996, 0.0, 2.358, 0.0, 0.0, 0.3, -0.9800000000000004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.48758930277672796, -0.4791435220152396, 1.3019986072291876, 2.0261847241170328, 3.059, 0.0, 0.38477484250099353, 1.3669227770014718, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
          # #  elif(self.knowledge_base.target_bin == 'bin_L'):
          # #      startConfig = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0999999999999996, 0.0, 2.358, 0.0, 0.0, 0.3, -0.9800000000000004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5459489486965996, 0.3966207137077951, 1.2334197022368156, 1.7995842493636613, 2.806168682648472, 0.0, 0.3290612094497131, 0.7556578376322631, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

          #   atStart = True
          #   logger.debug('Checking current configuration')
          #   for i,val in enumerate(self.robot_state.sensed_config):
          #       if(val - startConfig[i] > 0.05 and (i in self.right_arm_indices) ):
          #           #not in start config dont do cached planning
          #           # print i
          #           # print self.right_arm_indices
          #           # print val,startConfig[i]
          #           logger.debug('Not in correct configuration skip cached plans')
          #           atStart = False
          #           break

          #   if(atStart):
          #       logger.debug('In correct configuration. Looking for cached plans')
          #       #which bin are we moving from?
          #       #we are at the initial config. look up cached plans
          #       #check if a cached plan exists
          #       fileLocation = 'planning/cachedPlans/right_from_' + self.knowledge_base.target_bin + '_to_order_bin.json'

          #       try:
          #           pathFile = open(fileLocation)
          #           planArray = json.load(pathFile)
          #           goodness = 1
          #           logger.debug('Cached plan was found. Returning cached plan')
          #           return (planArray,1,limb,None)


          #       except IOError:
          #           logger.debug('IO Error Could not find cached plan: {}. Planning from scratch...'.format(fileLocation))
          #           pass
        
        #find how far to lift the object off of the shelf
        liftVector = [0,0,0.05]

        soft_objects = ['feline_greenies_dental_treats','kong_duck_dog_toy','kong_sitting_frog_dog_toy','kygen_squeakin_eggs_plush_puppies']
        tall_objects = ['cheezit_big_original','oreo_mega_stuf','feline_greenies_dental_treats','first_years_take_and_toss_straw_cups']

        #stores the current config of the robot
        curconfig = self.robot.getConfig()
        if(limb == 'left'):
            nameOfGraspedObject = self.knowledge_base.grasped_object
            grasped_object = self.objectDict[nameOfGraspedObject]
            #get the object 
            object = [grasped_object,self.virtualObjectDict[nameOfGraspedObject]]
            lowz = se3.apply(self.knowledge_base.shelf_xform, self.knowledge_base.bin_bounds[self.knowledge_base.target_bin][0])
            cspace = ('Object',0)
            collisionchecker = lambda x:self.planner.check_collision_free_with_object(x,object,self.knowledge_base.active_grasp)    
            exclude = []
            if nameOfGraspedObject in soft_objects:
                collisionchecker = None
                self.objectDict[nameOfGraspedObject].geometry().transform(vectorops.mul(so3.identity(),0.01),[0,0,0])
                self.virtualObjectDict[nameOfGraspedObject].geometry().transform(vectorops.mul(so3.identity(),0.01),[0,0,0])
            gripper_start = gripperlink.getTransform()
            #move the object
            objToGripperXForm = se3.mul(baxter.left_gripper_center_xform,se3.inv(self.knowledge_base.active_grasp))
            Tobj = se3.mul(gripper_start,objToGripperXForm)
            grasped_object.setTransform(*Tobj)
            #try to move the object off of the shelf in decreasing height
            while(liftVector[2] > 0):
                liftgoal = (gripper_start[0],vectorops.add(gripper_start[1],liftVector))
                if nameOfGraspedObject in tall_objects:
                    #try rotating about the x axis
                    Rtilt = so3.from_axis_angle(([1,0,0],45.0*math.pi/180.0))
                    Rtilt2 = so3.from_axis_angle(([1,0,0],-45.0*math.pi/180.0))
                    liftgoal1 = (so3.mul(Rtilt,gripper_start[0]),liftgoal[1])
                    liftSolns = self.planMoveToXform(goal_xform=liftgoal1,
                                            limb=limb,
                                            link='gripper',
                                            object=object,
                                            initialConfig=self.robot_state.commanded_config,
                                            colMargin=0,
                                            collisionchecker=collisionchecker,
                                            plan=False)
                    if liftSolns == None:
                        liftgoal2 = (so3.mul(Rtilt2,gripper_start[0]),liftgoal[1])
                        liftSolns = self.planMoveToXform(goal_xform=liftgoal2,
                                                         limb=limb,
                                                         link='gripper',
                                                         object=object,
                                                         initialConfig=self.robot_state.commanded_config,
                                                         colMargin=0,
                                                         collisionchecker=collisionchecker,
                                                         plan=False)

                else:

                    liftSolns = self.planMoveToXform(goal_xform=liftgoal,
                                            limb=limb,
                                            link='gripper',
                                            object=object,
                                            initialConfig=self.robot_state.commanded_config,
                                            colMargin=0,
                                            collisionchecker=collisionchecker,
                                            plan=False)

                #find a plan to move the object away from the shelf

                if (liftSolns == None):
                    liftVector = [0,0,liftVector[2] -0.01]
                    logger.warn('Could not find a way to lift object off of the shelf. Decreasing lift height...')

                    ################debug#################
                    # visualization.debug(self.world, [gripper_start, (gripper_start[0], vectorops.add(gripper_start[1],liftVector))])
                    ######################################
                    continue
                else:
                    break
        

            if (liftSolns == None and limb == 'left'):
                logger.warn('Could not find a way to lift object off of the shelf')

                ################debug#################
                # visualization.debug(self.world, [gripper_start, (gripper_start[0], vectorops.add(gripper_start[1],liftVector))])
                ######################################

                if nameOfGraspedObject in soft_objects:
                    self.objectDict[nameOfGraspedObject].geometry().transform(vectorops.mul(so3.identity(),100),[0,0,0])
                    self.virtualObjectDict[nameOfGraspedObject].geometry().transform(vectorops.mul(so3.identity(),100),[0,0,0])
                return None

        #store this plan
        if(limb == 'left'):
            logger.info('Found a way to lift item off the shelf')
            liftOffShelfPlan = liftSolns[0][0]
            cspace = ('Object',lowz[2]+liftVector[2])
            #move the object off of the shelf
            objToGripperXForm = se3.mul(baxter.left_gripper_center_xform,se3.inv(self.knowledge_base.active_grasp))
            Tgripper = gripperlink.getTransform();
            Tobj = se3.mul(Tgripper,objToGripperXForm)
            for o in object:
                o.setTransform(*Tobj)

            withdrawTransform = se3.mul(self.knowledge_base.shelf_xform, self.knowledge_base.withdraw_point_xforms[self.knowledge_base.target_bin])
        
            #use the current rotation
            self.robot.setConfig(liftOffShelfPlan)
            withRotation = self.left_gripper_link.getTransform()[0]
            withdrawTransform = (withRotation,withdrawTransform[1])

            moveOutPlan = self.planMoveToXform(goal_xform=withdrawTransform,
                                          limb=limb,
                                          link='gripper',
                                          object=object,
                                          initialConfig=liftOffShelfPlan,
                                          colMargin=0,
                                          collisionchecker=collisionchecker,
                                          cspace=cspace,
                                          checkCols=True,
                                          plan=True)
            
            if nameOfGraspedObject in soft_objects:
                self.objectDict[nameOfGraspedObject].geometry().transform(vectorops.mul(so3.identity(),100),[0,0,0])
                self.virtualObjectDict[nameOfGraspedObject].geometry().transform(vectorops.mul(so3.identity(),100),[0,0,0])

            if (moveOutPlan is None):
                logger.warn('Could not find a way to move object away from the shelf.')
                return None
            
            
            logger.info('Found a way to move the object away from the shelf')
             
            #update the robot
            self.robot.setConfig(moveOutPlan[-1])

            #move the object away from the shelf
            Tgripper = gripperlink.getTransform();
            Tobj = se3.mul(Tgripper,objToGripperXForm)
            for o in object:
                o.setTransform(*Tobj)
            
            ####################Debug##################
            #logger.debug('Object should be at withdraw point')
            #visualization.debug(self.world)
            ###########################################


        #find a plan to place the object in the bin
        if(limb == 'left'):
            rotation = so3.mul(
                so3.rotation([0,1,0], math.pi),
                so3.rotation([0,0,1], math.pi/2)
            )
            cspace = ('xrestrict',withdrawTransform[1][0])
        else:
            ############################################
            # visualization.debug(self.world,[self.right_gripper_link.getTransform()])
            ###########################################
            rotation = [0,1,0,0,0,1,1,0,0]
            cspace = ('scoop',0)
            collisionchecker = None
            # if object != None:
            #     for o in object:
            #         o.setCurrentTransform(so3.identity(),[0,0,-2])
            object = None
            moveOutPlan = [self.robot_state.commanded_config]

        placeInBinXform = (rotation,goal)
        moveOrderBinPlan = self.planMoveToXform(goal_xform=placeInBinXform,
                                      limb=limb,
                                      link='gripper',
                                      object=object,
                                      initialConfig=moveOutPlan[-1],
                                      colMargin=0.05,
                                      collisionchecker=collisionchecker,
                                      cspace=cspace,
                                      checkCols=True,
                                      plan=True)

        if(moveOrderBinPlan is None):
            logger.warn('Unable to find a plan to move to the order bin')
            return None

        ####################Debug##################
        # if(limb == 'left'):
        #     #update the robot
        #     self.robot.setConfig(moveOrderBinPlan[-1])
        #     Tgripper = gripperlink.getTransform();
        #     Tobj = se3.mul(Tgripper,objToGripperXForm)
        #     object.setTransform(*Tobj)

        # self.robot.setConfig(moveOrderBinPlan[-1])
        # logger.debug('Object should be above order bin')
        # visualization.debug(self.world)
        ###########################################
        logger.debug("Found a way to move object to order bin")

        #move object in world model out of the way
        if object != None:
            for o in object:
                o.setTransform(so3.identity(),[0,0,-2])

        if(limb == 'left'):
            aboveBinXform = (rotation,vectorops.add(goal,[0,0,.1]))
        else:
            aboveBinXform = ([0,-1,0,0,0,-1,1,0,0],vectorops.add(goal,[0,0,.1]))
            object=None

        moveUpAboveBinPlan = self.planMoveToXform(goal_xform=aboveBinXform,
                                      limb=limb,
                                      link='gripper',
                                      object=object,
                                      initialConfig=moveOrderBinPlan[-1],
                                      colMargin=0.05,
                                      collisionchecker=collisionchecker,
                                      plan=False)


        #if the limb is right dump the contents out of the scooper
        if(limb == 'right'):
            
            dumpItemXform = ([0,-1,0,0,0,-1,1,0,0],(goal))
            dumpItemPlan = self.planMoveToXform(goal_xform=dumpItemXform,
                                      limb=limb,
                                      link='gripper',
                                      object=None,
                                      initialConfig=moveOrderBinPlan[-1],
                                      colMargin=0,
                                      collisionchecker=collisionchecker,
                                      plan=True)
            if(dumpItemPlan is None):
                logger.warn('Could not find a way to dump the item.')
                return None

            ####################Debug##################
            # self.robot.setConfig(dumpItemPlan[-1])
            # logger.debug('Scoop should be upside down')
            # visualization.debug(self.world)
            ###########################################

            #shake the item 




        if moveOrderBinPlan != None:
            #found a plan(
            logger.debug(bcolors.BOLD+bcolors.UNDERLINE+bcolors.OKGREEN+'FOUND A PLAN!'+bcolors.ENDC)
            gripper_name = '{}_gripper'.format(self.knowledge_base.active_limb)
            if(limb == 'left'):
                plan = [
                    ('path', [ liftOffShelfPlan ], 0, 1),
                    ('path', moveOutPlan, 0, 1),
                    ('path', moveOrderBinPlan, 0, 0),
                    (gripper_name, [1.0,1.0,1.0,1], 0, 0),
                    ('path', [moveUpAboveBinPlan[0][0]], 0, 1)
                ]
            else:
                logger.debug("FOUND A PLAN!")
                shakePlan = []
                shakePlan.append(moveUpAboveBinPlan[0][0])
                shakePlan.append(dumpItemPlan[-1])
                shakePlan.append(moveUpAboveBinPlan[0][0])
                shakePlan.append(dumpItemPlan[-1])
                shakePlan.append(moveUpAboveBinPlan[0][0])
                shakePlan.append(dumpItemPlan[-1])
                shakePlan.append(moveUpAboveBinPlan[0][0])
                shakePlan.append(dumpItemPlan[-1])
                shakePlan.append(moveUpAboveBinPlan[0][0])
                shakePlan.append(dumpItemPlan[-1])
                shakePlan.append(moveUpAboveBinPlan[0][0])
                shakePlan.append(dumpItemPlan[-1])

                plan = [
                    ('path', moveOrderBinPlan, 0, 0),
                    ('path', dumpItemPlan, 0, 0),
                    ('fast_path', shakePlan, 0, 0),
                    ('path', [moveUpAboveBinPlan[0][0]], 0, 1)
                ]

                # plan = moveTrayToOrderBinPlan[0] + putDownTrayPlan[0] + plan

            return (plan,1,limb,None)


        logger.warn('Could not find a way to move object to order bin. No good plans')
        return None

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
        #set collision bounds of shelf
        if(self.shelf):
            self.shelf.geometry().setCollisionMargin(0.0)

        qcmd = self.robot_state.commanded_config
        solution = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.1,0.0,2.358,0.0,0.0,0.3,-0.98,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.18,0.0,2.118,0.04,0.0,0.62,0.901,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]


        #make sure start configuration is valid
        leftUnstuckConfig = self.getUnstuck('left', qcmd, self.left_gripper_link , object=None, collisionchecker=None)
        if leftUnstuckConfig==None: return None
        allUnstuckConfig = self.getUnstuck('right', leftUnstuckConfig, self.right_gripper_link , object=None, collisionchecker=None)
        if allUnstuckConfig==None: return None


        ##################debug###################
        # self.robot.setConfig(allUnstuckConfig)
        # visualization.debug(self.world)
        ##########################################

        path = self.planner.plan(allUnstuckConfig,solution)
        if path != None:
            limb = None
            goodness = 1
        else:
            #failed to create a plan
            logger.warn('Failed to create a plan from scratch')
            return None

        path = [allUnstuckConfig] + path
        plan = [
            ('path', path, 0, 0),
            ('left_gripper', [0.2,0.2,0.2,1.0], 0, 0),
            ('right_gripper', [0.2,0.2,0.2,1.0], 0, 0)
        ]
        return (plan, 1, None, None)


    #KH: added the ability to limit movement of link via the motionlimit keyword
    def planMoveToXform(self, goal_xform, limb, link='gripper', initialConfig=None, object=None, colMargin=0, collisionchecker=None, checkCols=True, cspace =('Default',0), plan=True, extra_feasibility_tests=[]):

        if limb == 'left':
            if(link == 'gripper'):
                link = self.left_gripper_link
            elif(link == 'camera'):
                goal_xform = se3.mul(goal_xform,se3.inv(baxter.left_camera_center_xform))
                link = self.left_camera_link
            else:
                logger.error('Invalid link name given to plan move to transform');
                return None
        elif(limb == 'right'):
            if(link == 'gripper'):
                link = self.right_gripper_link
            elif(link == 'camera'):
                goal_xform = se3.mul(goal_xform,se3.inv(baxter.right_camera_center_xform))
                link = self.right_camera_link
            else:
                logger.error('Invalid link name given to plan move to transform');
                return None
        else:
            logger.error('Invalid limb name given to plan move to transform');
            return None

        #set collision bounds of shelf
        self.shelf.geometry().setCollisionMargin(colMargin)

        #set the goal
        if isinstance(goal_xform,IKObjective):
            goal = goal_xform
        else:
            goal = ik.objective(link,R=goal_xform[0],t=goal_xform[1])

        #get the current config
        if(initialConfig == None):
            curconfig = self.robot_state.commanded_config
        else:
            curconfig = initialConfig

        if len(extra_feasibility_tests) > 0:
            oldcollisionchecker = collisionchecker
            def new_collision_checker(x):
                for i,f in enumerate(extra_feasibility_tests):
                    if f(x)==False:
                        print "Extra feasibility test",i,"failed"
                        return False
                if oldcollisionchecker: return oldcollisionchecker(x)
                return True
            collisionchecker = new_collision_checker

        ####################debug#####################
        # logger.debug('Trying to move to this goal pose')
        # visualization.debug(self.world,[goal_xform, self.right_gripper_link.getTransform(), self.left_gripper_link.getTransform()])
        ##############################################

        sortedSolutions = self.get_ik_solutions([goal],[limb],curconfig,maxIters=2000,validity_checker=collisionchecker,checkCols=checkCols)
        if len(sortedSolutions) == 0:
            logger.warn('Could not find a way to move arm to pose. No IK solutions')
            return None
        #make sure that the start configuration is feasible
        curconfig = self.getUnstuck(limb=limb,
                        link=link,
                        object=object,
                        currentConfig=curconfig,
                        collisionchecker=collisionchecker,
                        extra_feasibility_tests=extra_feasibility_tests)
        if curconfig==None: return None

        if(plan):
            #make a plan
            #print bcolors.OKBLUE +'Cspace:',cspace,bcolors.ENDC
            for solution in sortedSolutions:
                if(cspace[0] == 'scoop'):
                    #print bcolors.OKBLUE +'scoop:'+bcolors.ENDC
                    path = self.planner.plan(curconfig,solution[0],Cspace='scoop',extra_feasibility_tests=extra_feasibility_tests)
                elif(cspace[0] == 'scoopHeight'):
                    #print bcolors.OKBLUE +'scoopHeight:'+bcolors.ENDC
                    path = self.planner.plan(curconfig,solution[0],Cspace='height',argsTuple=cspace[1],extra_feasibility_tests=extra_feasibility_tests)

                elif(cspace[0] == 'trayUpright'):
                    #print bcolors.OKBLUE +'trayUpright:'+bcolors.ENDC
                    # object = self.objectDict['tray']
                    path = self.planner.plan_transfer(curconfig,solution[0],limb,object,self.knowledge_base.active_grasp,type='upright',extra_feasibility_tests=extra_feasibility_tests)

                elif(cspace[0] == 'xrestrict'):
                    #print bcolors.OKBLUE +'xrestrict:'+bcolors.ENDC
                    # object = self.objectDict['tray']
                    path = self.planner.plan_transfer(curconfig,solution[0],limb,object,self.knowledge_base.active_grasp,type='xrestrict', xrestrict=cspace[1],extra_feasibility_tests=extra_feasibility_tests) #0.17

                elif(cspace[0] == 'xrestrictNoobj'):
                    #print bcolors.OKBLUE +'xrestrictNoobj:'+bcolors.ENDC
                    path = self.planner.plan(curconfig,solution[0],Cspace='xrestrictNobj',argsTuple=(cspace[1],cspace[2]),extra_feasibility_tests=extra_feasibility_tests) #0.17,0

                elif(object is not None):

                    is_tray = False
                    if isinstance(object,list):
                        is_tray = False
                    else:
                        is_tray = (object == self.objectDict['tray'])
    
                    if is_tray:
                        print bcolors.OKBLUE +'tray_restrict'+bcolors.ENDC
                        path = self.planner.plan_transfer(curconfig,solution[0],limb,object,self.knowledge_base.active_grasp,type='restrict',extra_feasibility_tests=extra_feasibility_tests)
                    else:
                        print bcolors.OKBLUE +'regular item transfer'+bcolors.ENDC
                        path = self.planner.plan_transfer(curconfig,solution[0],limb,object,self.knowledge_base.active_grasp,xrestrict=cspace[1],extra_feasibility_tests=extra_feasibility_tests)
                else:
                    #print bcolors.OKBLUE +'default'+bcolors.ENDC
                    path = self.planner.plan(curconfig,solution[0],extra_feasibility_tests=extra_feasibility_tests)
                if path != None:
                    #found a plan return
                    return path
                if(solution != sortedSolutions[-1]):
                    logger.debug('No plan found for IK solution. Trying another')
            return None
        else:
            #just return the ik solution
            return sortedSolutions

#########################################################################################################
#point cloud grasping

#########################################################################################################
    def planPointCloudGrasp(self,object,bin,graspType='tilt'):
        #get the point cloud of the object in the bin
        #comes in as list of (x,y,z)
        objectPCloud = self.knowledge_base.object_clouds[object]
        objectPCloud = map(lambda p: p[:3], objectPCloud)

        #find the min, max, and mean for x,y,z
        
        mean = [0,0,0]
        minPt = [1000,1000,1000]
        maxPt = [-1000,-1000,-1000]
        for p in objectPCloud:
            mean = vectorops.add(mean,p)
            if p[0] < minPt[0]:
                minPt[0]=p[0]
            elif p[0] > maxPt[0]:
                maxPt[0]=p[0]
            if p[1] < minPt[1]:
                minPt[1]=p[1]
            elif p[1] > maxPt[1]:
                maxPt[1]=p[1]
            if p[2] < minPt[2]:
                minPt[2]=p[2]
            elif p[2] > maxPt[2]:
                maxPt[2]=p[2]


        mean = vectorops.mul(mean,1.0/len(objectPCloud))            
        median = vectorops.mul(vectorops.add(minPt,maxPt),0.5)
        width = maxPt[1]-minPt[1]
        depth = maxPt[0]-minPt[0]
        height = maxPt[2]-minPt[2]
        #KH: modify dimensions by the known minimum dimension
        known_minimum_dimension = self.knowledge_base.object_minimum_dims[object] if object in self.knowledge_base.object_minimum_dims else 0.0
        known_maximum_dimension = self.knowledge_base.object_maximum_dims[object] if object in self.knowledge_base.object_maximum_dims else float('inf')
        width = max(known_minimum_dimension,min(known_maximum_dimension,width))
        height = max(known_minimum_dimension,min(known_maximum_dimension,height))
        depth = max(known_minimum_dimension,min(known_maximum_dimension,depth))

        binBound0 = se3.apply(self.knowledge_base.shelf_xform, self.knowledge_base.bin_bounds[bin][0])
        
        # print se3.apply(self.knowledge_base.shelf_xform, self.knowledge_base.bin_bounds[bin][1])
        ##########################################debug##################################################
        # logger.debug(bcolors.OKBLUE + 'width %s, height %s, depth %s' + bcolors.ENDC,width,height,depth)
        # visualization.debug_cloud([ objectPCloud ], world=self.world, xforms=[ (so3.identity(), mean) ])
        ################################################################################################

        #make a check here for objects that we want to scoop if they are not standing up
        ##################################################################################
        
        #create numpy array from the pc
        pcArray = numpy.array(objectPCloud)
        powerOpenConfigs = []
        powerCloseConfigs = []

        pinchOpenConfigs = []
        pinchCloseConfigs = []
        #compute the width for all of the angles
        for angle in [0,-15,15,-30,30]:
        #rotate the ptcloud by the negative of the angle and find the width
        #demean 
            pcArray -= mean
            pcArray = pcArray.dot(so3.matrix(so3.rotation([0,0,1],math.pi/180.0*-angle)))
            width = pcArray[:,1].max()-pcArray[:,1].min()
            width = max(known_minimum_dimension,min(known_maximum_dimension,width))
            print "Dimensions at angle",angle,":",width,height,depth
            #print bcolors.OKGREEN + 'Angle + Width',angle, width, bcolors.ENDC
            #set the gripper to open up to a few cm outside of the object
            gripperOpenCmd = 0.491*math.exp(2.24351361*(width+.05))
            if(gripperOpenCmd > .9):
                gripperOpenCmd = .9
            elif(gripperOpenCmd < .45):
                gripperOpenCmd = 0.45

            gripperCloseCmd = 0.2#*math.exp(3.24351361*closeDist)
            if(gripperCloseCmd > .9):
                gripperCloseCmd = .9
            elif(gripperCloseCmd < 0.1):
                gripperCloseCmd = 0.1
        
            powerOpenConfigs.append([gripperOpenCmd,gripperOpenCmd,gripperOpenCmd,1])
            powerCloseConfigs.append([gripperCloseCmd,gripperCloseCmd,gripperCloseCmd,1])
        
            gripperOpenCmd = 100*0.0166004766*(width+0.05)+0.4669
            if(gripperOpenCmd > .8):
                gripperOpenCmd = .8
            elif(gripperOpenCmd < .45):
                gripperOpenCmd = 0.45            

            pinchOpenConfigs.append([gripperOpenCmd,gripperOpenCmd,0,0.1])
            pinchCloseConfigs.append([0.3,0.3,0,0.1])

        defaultGraspRot = [0, 1, 0, 0, 0, 1, 1, 0, 0]

        pinchMean = mean[:]
        mean[0] = minPt[0]+0.07##TODO change at competition how far to move the gripper back
        if object == 'feline_greenies_dental_treats':
            #push farther
            mean[0] = minPt[0] + 0.1

        gxforms = []  

        if(graspType == 'power' or graspType == 'tilt'):      
            #add in other rotations for the power 
            if(graspType=='tilt'):
                #power tilt
                #min height is binbounds0 + 0.1 #TODO:Change this at competition
                #max dist back is 1.06
                tilt = 15
                if(mean[0] > 1.06):
                    mean[0] = 1.06
                if(mean[2] < (binBound0[2] + 0.1)):
                    mean[2] = binBound0[2] + 0.1
            elif(graspType=='power'):
                #for power
                #anything further back than 1m in x needs a tilt #TODO:Change this at competition
                #max distance back is 1 for non tilt grasp
                #anything shorter than binbound0[2]+0.09 cm needs to be moved up to binbounds0[2]+0.09 for power
                tilt = 0
                if(mean[2] < binBound0[2]+0.1):
                    mean[2] = binBound0[2]+0.1

            object_Xform = [so3.identity(),mean]
            for angle in [0,-15,15,-30,30]:
                rotGraspXform = so3.mul(defaultGraspRot,so3.rotation([0,1,0],math.pi/180.0*angle))
                rotGraspXform = so3.mul(rotGraspXform,so3.rotation([1,0,0],math.pi/180.0*tilt))
                gxforms.append( se3.mul(object_Xform,(rotGraspXform,[0,0,0] )) )

        else:#pinch
            #add in pinch grasps
            #rotate -20 around z 90 around x
            defaultPinchRot = [0, 1, 0, 0, 0, 1, 1, 0, 0]

            #for pinch
            #anything shorter than binbound0[2]+0.07 cm needs to be moved up to binbounds0[2]+0.07  #TODO:Change this at competition

            #anything further back than 1.05m needs to be set at 1.05 m    #TODO:Change this at competition

            # if(pinchMean[2] < binBound0[2] + 0.07):
            pinchMean[2] = binBound0[2] + 0.075
        
            if(pinchMean[0] > 1.05): #TODO:Change this at competition
                pinchMean[0] = 1.05

            defaultPinchRot = so3.mul(defaultPinchRot,so3.rotation([0,0,1],math.pi/180.0*-90))  
            defaultPinchRot = so3.mul(defaultPinchRot,so3.rotation([0,1,0],math.pi/180.0*20))                

            object_Xform = [so3.identity(),pinchMean];
            #add in other rotations for the pinch grasp
            for angle in [0,-15,15,-30,30]:
                rotPinch = so3.mul(defaultPinchRot,so3.rotation([1,0,0],math.pi/180.0*angle))
                gxforms.append( se3.mul(object_Xform,(rotPinch,[0,0,.05] )) )      




        #object specific code
        if(object == 'kong_air_dog_squeakair_tennis_ball' or object =='feline_greenies_dental_treats' or object == 'highland_6539_self_stick_notes'):
            #openConfigs = 7*[[0.6,0.6,0.6,1]]
            openConfigs = powerOpenConfigs 
            closedConfigs = powerCloseConfigs 
            return gxforms,openConfigs,closedConfigs

        if(graspType == 'tilt' or graspType == 'power'):  
            openConfigs = powerOpenConfigs
            closedConfigs = powerCloseConfigs
        else:
            openConfigs = pinchOpenConfigs
            closedConfigs = pinchCloseConfigs

        return gxforms,openConfigs,closedConfigs

#########################################################################################################

#failsafe point cloud grasp
#moved the item away from the sides of the bin and then grasps from above
    def failsafeplan(self,binLoc,object):

        rigid_object = self.objectDict[object]
        objectPCloud = self.knowledge_base.object_clouds[object]
        objectPCloud = map(lambda p: p[:3], objectPCloud)
        pcArray = numpy.array(objectPCloud)

        
        
        #find the min, max, and mean for x,y,z
        mean = [0,0,0]
        minPt = [1000,1000,1000]
        maxPt = [-1000,-1000,-1000]
        for p in objectPCloud:
            mean = vectorops.add(mean,p)
            if p[0] < minPt[0]:
                minPt[0]=p[0]
            elif p[0] > maxPt[0]:
                maxPt[0]=p[0]
            if p[1] < minPt[1]:
                minPt[1]=p[1]
            elif p[1] > maxPt[1]:
                maxPt[1]=p[1]
            if p[2] < minPt[2]:
                minPt[2]=p[2]
            elif p[2] > maxPt[2]:
                maxPt[2]=p[2]


        mean = vectorops.mul(mean,1.0/len(objectPCloud))            
        median = vectorops.mul(vectorops.add(minPt,maxPt),0.5)
        width = maxPt[1]-minPt[1]
        depth = maxPt[0]-minPt[0]
        height = maxPt[2]-minPt[2]

        #############################################################
    
        # visualization.debug_cloud([ objectPCloud ], world=self.world, xforms=[ (so3.identity(), mean), self.left_gripper_link.getTransform() ])
        ######################################################


        #KH: modify dimensions by the known minimum dimension
        known_minimum_dimension = self.knowledge_base.object_minimum_dims[object] if object in self.knowledge_base.object_minimum_dims else 0.0
        known_maximum_dimension = self.knowledge_base.object_maximum_dims[object] if object in self.knowledge_base.object_maximum_dims else float('inf')
        width = max(known_minimum_dimension,min(known_maximum_dimension,width))
        height = max(known_minimum_dimension,min(known_maximum_dimension,height))
        depth = max(known_minimum_dimension,min(known_maximum_dimension,depth))
        
        #is the object in the left half or right half of the bin?
        binBound0 = se3.apply(self.knowledge_base.shelf_xform, self.knowledge_base.bin_bounds[binLoc][0])
        binBound1 = se3.apply(self.knowledge_base.shelf_xform, self.knowledge_base.bin_bounds[binLoc][1])

        centerBin = [(a+b)/2.0 for a,b in zip(binBound0,binBound1)]
        self.shelf.geometry().setCollisionMargin(0)
        aboveHeight = 0.09  #TODO change this
        slideDist = 0.08    ##TODO change this
        goalXYZ = mean[:]
        goalXYZ[2] += aboveHeight
        goalXYZ[0]-=0.1                  #####################
        goalXYZ[1]-=0.05
        changeOffset = 0
        if(mean[1] > centerBin[1]):
            #object is left of center need to move right (neg)
            changeOffset = -0.01
            rotDegree = -20
        else:
            #object is right of center need to move left (pos)
            changeOffset = 0.01
            rotDegree = 20

        handTilt = 35
        #are we power, tilt, or pinch?
        #graspType = scoopOrGrasp(object,binLoc,self.knowledge_base)
        graspType = 'pinch'
        #if(graspType == 'scoop' or graspType is None):
        #    logger.warn('Tried to run a fail safe grasp on a scoop object or an object that had no grasps')
        #    return None
        #elif(graspType == 'pinch'):
        defaultGraspRot = [0, 1, 0, 0, 0, 1, 1, 0, 0]
        defaultGraspRot = so3.mul(defaultGraspRot,so3.rotation([0,0,1],math.pi/180.0*-180))  
        defaultGraspRot = so3.mul(defaultGraspRot,so3.rotation([0,1,0],math.pi/180.0*-handTilt)) 
        defaultGraspRot = so3.mul(defaultGraspRot,so3.rotation([1,0,0],math.pi/180.0*rotDegree))
        thumbConfig = 0
        preshapeConfig = 0.1
        gripperOpenCmd = 100*0.0166004766*(width+0.05)+0.2926
        if(gripperOpenCmd > .8):
            gripperOpenCmd = .8
        elif(gripperOpenCmd < .45):
            gripperOpenCmd = 0.45   
        gripperCloseCmd = 0.3
        thumbClose = 0

        #elif(graspType == 'power'):
        #    defaultGraspRot = [0, 0, -1, 0, 1, 0, 1, 0, 0]
        #    defaultGraspRot = so3.mul(defaultGraspRot,so3.rotation([1,0,0],math.pi/180.0*rotDegree))
        #    defaultGraspRot = so3.mul(defaultGraspRot,so3.rotation([0,1,0],math.pi/180.0*handTilt))
        #    preshapeConfig = 1
        #    #open the gripper to the corect distance based on the object width
        #    gripperOpenCmd = 0.3076188863*math.exp(3.24351361*(width+.05))
        #    if(gripperOpenCmd > .9):
        #        gripperOpenCmd = .9
        #    elif(gripperOpenCmd < .45):
        #        gripperOpenCmd = 0.45

        #    thumbConfig = gripperOpenCmd 
        #    gripperCloseCmd = 0.28
        #    thumbClose = 0.28

        #elif(graspType == 'tilt'):
        #    defaultGraspRot = [0, 0, -1, 0, 1, 0, 1, 0, 0]
        #    defaultGraspRot = so3.mul(defaultGraspRot,so3.rotation([1,0,0],math.pi/180.0*rotDegree))
        #    defaultGraspRot = so3.mul(defaultGraspRot,so3.rotation([0,1,0],math.pi/180.0*handTilt))
        #    thumbConfig = 0
        #    preshapeConfig = 1
        #    #open the gripper to the corect distance based on the object width
        #    gripperOpenCmd = 0.3076188863*math.exp(3.24351361*(width+.05))
        #    if(gripperOpenCmd > .9):
        #        gripperOpenCmd = .9
        #    elif(gripperOpenCmd < .45):
        #        gripperOpenCmd = 0.45

        #    thumbConfig = gripperOpenCmd 
        #    gripperCloseCmd = 0.28
        #    thumbClose = 0.28

        moveAboveObjectPlan = 0

        foundPlan = False

        #set the gripper closed
        baxter.set_model_gripper_command(self.robot,'left',[0.35,0.35,0,0.1])
        collisionchecker = lambda x:self.planner.check_collision_free(x,exclude=[],verbose=False)
        cnt = 0
        while(moveAboveObjectPlan == 0 and cnt < 10):
            print defaultGraspRot
            baxter.set_model_gripper_command(self.robot,'left',[0.35,0.35,0,0.1])
            moveAboveObjectXform = (defaultGraspRot,goalXYZ)
            moveAboveObjectPlan = self.planMoveToXform(goal_xform=moveAboveObjectXform,
                                      limb='left',
                                      link='gripper',
                                      object=None,
                                      initialConfig=self.robot_state.commanded_config,
                                      colMargin=0,
                                      collisionchecker=collisionchecker,
                                      checkCols=False,
                                      plan=False)

            if(moveAboveObjectPlan is None):
                logger.debug('Plan to move above object failed, tyring another.')
                #change the goal by change offset 
                goalXYZ = vectorops.add(goalXYZ,[0,changeOffset,0])
                moveAboveObjectPlan = 0
                cnt += 1
            else:
                foundPlan = True
                ##################################################################
                self.robot.setConfig(moveAboveObjectPlan[0][0])
                baxter.set_model_gripper_command(self.robot,'left',[0.35,0.35,0.35,1])
                visualization.debug(self.world)
                ####################################################################
                #break
                goalXYZ = vectorops.add(goalXYZ,[0,changeOffset,0])
                moveAboveObjectPlan = 0
                

        if(foundPlan == False):
            logger.warn('Could not find a way to move the gripper above the item. Object is either too deep or too tall')
            return None


        #move the hand down with ik solution not checking collisions
        moveOnObjectXform = (moveAboveObjectXform[0],vectorops.add(moveAboveObjectXform[1],[0,0,-aboveHeight]))
        moveHandDownOnObject = self.planMoveToXform(goal_xform=moveOnObjectXform,
                                      limb='left',
                                      link='gripper',
                                      object=None,
                                      initialConfig=moveAboveObjectPlan[-1],
                                      colMargin=0,
                                      collisionchecker=None,
                                      checkCols=False,
                                      plan=False)

        if(moveHandDownOnObject is None):
            logger.warn('Could not find a way to move the gripper on top of the item.')
            return None

        ##################################################################
        # self.robot.setConfig(moveHandDownOnObject[0][0])
        # baxter.set_model_gripper_command(self.robot,'left',[0.35,0.35,0.35,1])
        # visualization.debug(self.world)
        ####################################################################


        #slide the object towards the middle ignoring collisions
        slideObjectMiddleXform = (moveOnObjectXform[0],vectorops.add(moveOnObjectXform[1],[0,slideDist*changeOffset*100,0]))
        slideMiddlePlan = self.planMoveToXform(goal_xform=slideObjectMiddleXform,
                                      limb='left',
                                      link='gripper',
                                      object=None,
                                      initialConfig=moveHandDownOnObject[0][0],
                                      colMargin=0,
                                      collisionchecker=None,
                                      checkCols=False,
                                      plan=False)

        if(slideMiddlePlan is None):
            logger.warn('Could not find a way to slide the item away fromt the edge.')
            return None
        
        ##################################################################
        # logger.debug('SLide over')
        # self.robot.setConfig(slideMiddlePlan[0][0])
        # baxter.set_model_gripper_command(self.robot,'left',[0.35,0.35,0.35,1])
        # visualization.debug(self.world)
        ####################################################################
        

        #lift the gripper back up
        moveAboveObjectXRot = so3.mul(defaultGraspRot,so3.rotation([1,0,0],math.pi/180.0*-rotDegree))
        moveAboveObjectXform2 = (moveAboveObjectXRot,vectorops.add(slideObjectMiddleXform[1],[0,0,aboveHeight]))
        moveAboveObjectPlan2 = self.planMoveToXform(goal_xform=moveAboveObjectXform2,
                                      limb='left',
                                      link='gripper',
                                      object=None,
                                      initialConfig=slideMiddlePlan[0][0],
                                      colMargin=0,
                                      collisionchecker=collisionchecker,
                                      checkCols=False,
                                      plan=False)

        if(moveAboveObjectPlan2 is None):
            logger.warn('Could not find a way to lift the gripper above the item after sliding.')
            return None
        
        ##################################################################
        # logger.debug('Gripper above ')
        # self.robot.setConfig(moveAboveObjectPlan2[0][0])
        # baxter.set_model_gripper_command(self.robot,'left',[0.35,0.35,0.35,1])
        # visualization.debug(self.world)
        ####################################################################

        baxter.set_model_gripper_command(self.robot,'left',[gripperOpenCmd,gripperOpenCmd,thumbConfig,preshapeConfig])

        #move the gripper back down 
        moveAroundObjectXform = (moveAboveObjectXform2[0],vectorops.add(moveAboveObjectXform2[1],[0,0,-aboveHeight]))
        moveHandAroundObject = self.planMoveToXform(goal_xform=moveAroundObjectXform,
                                      limb='left',
                                      link='gripper',
                                      object=None,
                                      initialConfig=moveAboveObjectPlan2[0][0],
                                      colMargin=0,
                                      collisionchecker=None,
                                      checkCols=False,
                                      plan=False)

        if(moveHandAroundObject is None):
            logger.warn('Could not find a way to move around the object for grasping')
            return None



        #else we found a plan
        logger.debug(bcolors.BOLD+bcolors.UNDERLINE+bcolors.OKGREEN+'FOUND A PLAN!'+bcolors.ENDC)

        plan = [
            ('path',moveAboveObjectPlan,0,1),

            ('path', [ moveHandDownOnObject[0][0] ], 0 , 1),

            ('path', [ slideMiddlePlan[0][0] ], 0 , 1),

            ('path', [ moveAboveObjectPlan2[0][0] ], 0 , 1),

            ('left_gripper', [gripperOpenCmd,gripperOpenCmd,thumbConfig,preshapeConfig],0,1),

            ('path', [ moveHandAroundObject[0][0] ], 0 , 1),

            ('left_gripper', [gripperCloseCmd,gripperCloseCmd,thumbClose,preshapeConfig], 0, 1)
        ]
        
        return (plan, 1, 'left', None) 

            
#########################################################################################################  


#########################################################################################################

#point cloud scooping


#########################################################################################################        

    def planPointCloudScoop(self,object,bin):

        #move the tray in position
        moveToPickUpTray = self.planPickUpOrderTray(True)
        moveTrayToBinPlan = self.planMoveTrayToBin(bin)
        if(moveTrayToBinPlan is None or moveToPickUpTray is None):
            logger.warn('Could not find a way to pick up the tray')
            return None

        #get the point cloud of the object in the bin
        #comes in as list of (x,y,z)
        objectPCloud = self.knowledge_base.object_clouds[object]
        objectPCloud = map(lambda p: p[:3], objectPCloud)

        # #find the min, max, and mean for x,y,z

        
        mean = [0,0,0]
        minPt = [1000,1000,1000]
        maxPt = [-1000,-1000,-1000]
        for p in objectPCloud:
            mean = vectorops.add(mean,p)
            if p[0] < minPt[0]:
                minPt[0]=p[0]
            elif p[0] > maxPt[0]:
                maxPt[0]=p[0]
            if p[1] < minPt[1]:
                 minPt[1]=p[1]
            elif p[1] > maxPt[1]:
                maxPt[1]=p[1]
            if p[2] < minPt[2]:
                minPt[2]=p[2]
            elif p[2] > maxPt[2]:
                maxPt[2]=p[2]


        mean = vectorops.mul(mean,1.0/len(objectPCloud))            
        median = vectorops.mul(vectorops.add(minPt,maxPt),0.5)
        width = maxPt[1]-minPt[1]
        depth = maxPt[0]-minPt[0]
        height = maxPt[2]-minPt[2]

        #KH: modify dimensions by the known minimum dimension
        known_minimum_dimension = self.knowledge_base.object_minimum_dims[object] if object in self.knowledge_base.object_minimum_dims else 0.0
        known_maximum_dimension = self.knowledge_base.object_maximum_dims[object] if object in self.knowledge_base.object_maximum_dims else float('inf')
        known_median_dimension = self.knowledge_base.object_median_dims[object] if object in self.knowledge_base.object_median_dims else float('inf')
        width = max(known_minimum_dimension,min(known_maximum_dimension,width))
        height = max(known_minimum_dimension,min(known_maximum_dimension,height))
        depth = max(known_minimum_dimension,min(known_maximum_dimension,depth))

        ##########################################debug##################################################
        #logger.debug('width %s, height %s, depth %s',width,depth,height)
        #visualization.debug(world=self.world, xforms=[  self.right_gripper_link.getTransform()])
        ################################################################################################


        #at the end of the scoop rock back 
        #move the gripper to the back of the shelf at the highest possible point it can
        #align with the object but slightly offset to the right to avoid the closed surface
        #only plan for collisions with the shelf and not the object

        #height of the shelf #TODO:Change this at competition
        shelfHeight = 0.02

        #how far to lift up the scooper in between scoops #TODO:Change this at competition
        liftHeight = 0.08

        #depth of the shelf#TODO:Change this at competition
        if(object in ['kong_duck_dog_toy','kong_sitting_frog_dog_toy']):
            shelfDepth = 0.1 #move all the way to the front
        else:
            #get an idea of the depth of the object
            #is the width closer to the max dim or the median dim?
            if(abs(width - known_maximum_dimension) < abs(width - known_median_dimension)):
                #it is close to the max dim base off of the median dim
                dim = known_median_dimension
            else:
                #it is closer to the median dim, base off of the max dim
                dim = known_maximum_dimension
            shelfDepth = 0.1 - (dim-0.09) #subtract the depth of the object that is longer than the depth of the scooper

        binBound0 = se3.apply(self.knowledge_base.shelf_xform, self.knowledge_base.bin_bounds[bin][0])
        binBound1 = se3.apply(self.knowledge_base.shelf_xform, self.knowledge_base.bin_bounds[bin][1])

        centerBin = [(a+b)/2.0 for a,b in zip(binBound0,binBound1)]

        #see if the median plus half of the scoop width is going to collide
        scoopOffset = 0.02
        medianHitsBin = True
        while(medianHitsBin):
            if(median[1] - scoopOffset + 0.07 > binBound1[1]):
                logger.debug('Median of point cloud + scoop too far left (pos) make more negative')
                median[1] -= 0.01
            elif(median[1] - scoopOffset - 0.1 < binBound0[1]):
                logger.debug('Median of point cloud + scoop too far right (neg) make more positive')
                median[1] += 0.01
            else:
                logger.debug('Median ok')
                medianHitsBin = False
                median[1] -= scoopOffset

        centerBin = [(a+b)/2.0 for a,b in zip(binBound0,binBound1)]
        maxBinClear = centerBin[2]

        #rotation matrix for the scoop to be flat on the shelf
        scoopDownRotMatrix = [0,0,1,0,-1,0,1,0,0];

        scoopDownRotMatrix = so3.mul(scoopDownRotMatrix,so3.rotation([0,0,1],math.pi/180.0*180))
        scoopDownRotMatrix = so3.mul(scoopDownRotMatrix,so3.rotation([1,0,0],math.pi/180.0*-45))


        #rotation matrix for the scoop to be tilted with the teeth towards the bottom of the shelf
        teethDownRotMatrix = so3.mul(scoopDownRotMatrix,so3.rotation(vectorops.unit([0,1,1]),math.pi/180.0*-2))
        
        #rotation matrix for the scoop to be tilted with the teeth towards the bottom of the shelf
        teethUpRotMatrix = so3.mul(scoopDownRotMatrix,so3.rotation(vectorops.unit([0,1,1]),math.pi/180.0*7))

        #rotation matrix for the scoop to be tilted down moving the shelf in
        moveBackRotMatrix = so3.mul(scoopDownRotMatrix,so3.rotation(vectorops.unit([0,1,1]),math.pi/180.0*-20))
        moveBackRotMatrix = so3.mul(moveBackRotMatrix,so3.rotation([1,0,0],math.pi/180.0*30))

        currentGripper =  self.right_gripper_link.getTransform()[1]

        
        #front of shelf xform
        topFrontOfShelfXform = (scoopDownRotMatrix,[centerBin[0]-0.37,median[1],maxBinClear])

        #location of the back of the shelf
        backOfShelfCoordinates = [max(binBound0[0],binBound1[0])-0.26,median[1],maxBinClear]
        
        #set up the goal
        backofShelfXform = (moveBackRotMatrix,backOfShelfCoordinates);

        #move the scoop up to the height of the back of shelfxform
        moveScoopUP = self.planMoveToXform(goal_xform=topFrontOfShelfXform,
                                      limb='right',
                                      link='gripper',
                                      object=None,
                                      initialConfig=moveTrayToBinPlan[0][-1][1][-1],
                                      colMargin=0.05,
                                      collisionchecker=None,
                                      checkCols=True,
                                      plan=True)
        if(moveScoopUP is None):
            logger.warn('No plan available to move the scoop up')
            return None
        ##########################################debug##################################################
        # self.robot.setConfig(moveScoopUP[0][0])
        # visualization.debug_cloud([ objectPCloud ], world=self.world, xforms=[ (so3.identity(), median), self.right_gripper_link.getTransform() ])
        ################################################################################################

        self.robot.setConfig(moveScoopUP[-1])
        moveScoopBackShelf = self.planMoveToXform(goal_xform=backofShelfXform,
                                      limb='right',
                                      link='gripper',
                                      object=None,
                                      initialConfig=moveScoopUP[-1],
                                      colMargin=0.0,
                                      collisionchecker=None,
                                      cspace=('scoopHeight',(maxBinClear,median[1])),
                                      checkCols=True,
                                      plan=True)


        if(moveScoopBackShelf is None):
            logger.warn('No plan available to move the scoop to the back of the shelf')
            return None

        logger.info('Found plan to move the scoop to the back of the shelf.')
        ##########################################debug##################################################
        #self.robot.setConfig(moveScoopBackShelf[0][0])
        #visualization.debug(self.world)
        ################################################################################################

        #put the gripper on the bottom of the shelf
        #find ik solution to move down shelf height cm from end of move to back of shelf plan

        bottomOfShelfXform = (teethDownRotMatrix,vectorops.sub(backOfShelfCoordinates,[0,0,shelfHeight]))

        moveScoopBotShelf = self.planMoveToXform(goal_xform=bottomOfShelfXform,
                                      limb='right',
                                      link='gripper',
                                      object=None,
                                      initialConfig=moveScoopBackShelf[-1],
                                      colMargin=0,
                                      collisionchecker=None,
                                      checkCols=False,
                                      plan=False)
        if(moveScoopBotShelf is None):
            logger.warn('No plan available to move the scoop to the bottom of the shelf')
            return None

        logger.info('Found plan to move the scoop to the bottom of the shelf')
        ##########################################debug##################################################
        # self.robot.setConfig(moveScoopBotShelf[0][0])
        # visualization.debug_cloud([ objectPCloud ], world=self.world, xforms=[ (so3.identity(), mean), self.right_gripper_link.getTransform() ])
        ################################################################################################
        
        #rake fwd shelf depth
        frontOfShelfXform = (teethDownRotMatrix, vectorops.sub(bottomOfShelfXform[1],[shelfDepth,0,0]))

        print frontOfShelfXform
        print shelfDepth
        moveScoopFrontShelf = self.planMoveToXform(goal_xform=frontOfShelfXform,
                                      limb='right',
                                      link='gripper',
                                      object=None,
                                      initialConfig=moveScoopBotShelf[0][0],
                                      colMargin=0,
                                      collisionchecker=None,
                                      checkCols=False,
                                      plan=False)
        if(moveScoopFrontShelf is None):
            logger.warn('No plan available to move the scoop to the front of the shelf')
            return None

        logger.info('Found plan to move the scoop to the front of the shelf')
        ##########################################debug##################################################
        # self.robot.setConfig(moveScoopFrontShelf[0][0])
        # visualization.debug_cloud([ objectPCloud ], world=self.world, xforms=[ (so3.identity(), mean), self.right_gripper_link.getTransform() ])
        ################################################################################################

        #lift the scooper up again so when we move it back it doesnt drag the object with it
        liftScoopUpXform =  (teethUpRotMatrix, vectorops.add(frontOfShelfXform[1],[0,0,liftHeight]))
        moveScoopUpFrontShelf = self.planMoveToXform(goal_xform=liftScoopUpXform,
                                      limb='right',
                                      link='gripper',
                                      object=None,
                                      initialConfig=moveScoopFrontShelf[0][0],
                                      colMargin=0,
                                      collisionchecker=None,
                                      checkCols=False,
                                      plan=False)
        if(moveScoopUpFrontShelf is None):
            logger.warn('No plan available to lift the scoop off of the front of the shelf')
            return None
        ##########################################debug##################################################
        #self.robot.setConfig(moveScoopUpFrontShelf[0][0])
        #visualization.debug_cloud([ objectPCloud ], world=self.world, xforms=[ (so3.identity(), mean), self.right_gripper_link.getTransform() ])
        ################################################################################################


        logger.info('Found plan to lift the scoop off of the shelf')

        #set the current config to the last plan in the move to front of shelf to run the move back of shelf again
        moveScoopBackShelf2 = self.planMoveToXform(goal_xform=backofShelfXform,
                                      limb='right',
                                      link='gripper',
                                      object=None,
                                      initialConfig=moveScoopUpFrontShelf[0][0],
                                      colMargin=0,
                                      collisionchecker=None,
                                      cspace=('scoopHeight',(maxBinClear,median[1])),
                                      plan=True)
        if(moveScoopBackShelf2 is None):
            logger.warn('No plan available to move the scoop to the back of the shelf2')
            return None

        logger.info('Found plan to move the scoop to the back of the shelf...again')

        #move the tray down
        trayobject = self.objectDict['tray']
        goal_xform = (self.left_gripper_link.getTransform()[0],vectorops.add(self.left_gripper_link.getTransform()[1],[0,0,-.15]))
        #get ik solutions for the bin
        #set the collision checker
        collisionchecker = lambda x:self.planner.check_collision_free_with_object(x,trayobject,self.trayGraspXform)
        pickUpTrayPlan = self.planMoveToXform(goal_xform=goal_xform,
                                    limb='left',
                                    object=trayobject,
                                    link='gripper',
                                    colMargin=0,
                                    collisionchecker=collisionchecker,
                                    initialConfig=moveScoopUpFrontShelf[0][0],
                                    checkCols=True,
                                    plan=True)
        if(pickUpTrayPlan is None):
            logger.warn('No way to move the tray to down')
            return None



        safeHoldRotation = [0,1,0,0,0,1,1,0,0]
        safeHoldRotation = so3.mul(safeHoldRotation,so3.rotation([0,1,0],math.pi/180.0*65))
        liftFrontOfShelfXform = (safeHoldRotation, [centerBin[0]-0.35,centerBin[1]-.05,centerBin[2]])

        self.robot.setConfig(pickUpTrayPlan[-1])
        #######################################
        #visualization.debug(self.world)
        #######################################
        motion_limit = 0.5
        motion_range = (vectorops.add(self.right_gripper_link.getTransform()[1],[-motion_limit*0.5]*3),vectorops.add(self.right_gripper_link.getTransform()[1],[motion_limit*0.5]*3))
        def gripper_within_motion_range(x):
            pt = self.right_gripper_link.getTransform()[1]
            return all(a <= v <= b for (a,v,b) in zip(motion_range[0],pt,motion_range[1]))
        rotScoopUpFrontShelf = self.planMoveToXform(goal_xform=liftFrontOfShelfXform,
                                      limb='right',
                                      link='gripper',
                                      object=None,
                                      initialConfig=pickUpTrayPlan[-1],
                                      colMargin=0,
                                      collisionchecker=None,
                                      checkCols=True,
                                      plan=True,
                                      extra_feasibility_tests=[gripper_within_motion_range])
        if(rotScoopUpFrontShelf is None):
            logger.warn('No plan available to lift and rotate the scoop off of the shelf')
            return None

        ##########################################debug##################################################
        #self.robot.setConfig(rotScoopUpFrontShelf[-1])
        #visualization.debug_cloud([ objectPCloud ], world=self.world, xforms=[ (so3.identity(), mean), self.right_gripper_link.getTransform() ])
        ################################################################################################

        logger.info(bcolors.OKGREEN+'Found plan to scoop the object'+bcolors.ENDC)
        plan = [
            ('path',moveScoopUP,0,1),
            ('path', moveScoopBackShelf, 0, 1),
            ('path', [ moveScoopBotShelf[0][0] ], 0 , 1),
            ('path', [moveScoopFrontShelf[0][0]], 0 , 1),
            ('path', [ moveScoopUpFrontShelf[0][0] ], 0 , 1),
            ('path', moveScoopBackShelf2, 0 , 1),
            ('path', [ moveScoopBotShelf[0][0] ], 0 , 1),
            ('path', [moveScoopFrontShelf[0][0]], 0 , 1),
            ('path', [ moveScoopUpFrontShelf[0][0] ], 0 , 1),
            ('path', pickUpTrayPlan, 0 , 0),
            ('path',  rotScoopUpFrontShelf, 0 , 1)
        ]
        plan = moveToPickUpTray[0] + moveTrayToBinPlan[0] + plan
        return (plan, 1, 'right', None) 


#########################################################################################################
#get the robot out of infeasible configurations

#########################################################################################################

    def getUnstuck(self, limb, currentConfig, link, object=None, collisionchecker=None, extra_feasibility_tests=[]):
        
        self.robot.setConfig(currentConfig)

        #check if the current configuration is feasible
        feasible = True;
        if(collisionchecker is None):
            cspace = LimbCSpace(self.planner,limb)
            cspace.extra_feasibility_tests = extra_feasibility_tests
            limbConfig = self.planner.get_limb_config(currentConfig,limb)
            feasible = cspace.feasible(limbConfig)
        elif (collisionchecker is not None and self.knowledge_base.active_grasp is None):
            cspace = LimbCSpace(self.planner,limb)
            cspace.extra_feasibility_tests = extra_feasibility_tests
            limbConfig = self.planner.get_limb_config(currentConfig,limb)
            feasible = cspace.feasible(limbConfig)
        else:
            cspace = TransferCSpace(self.planner,limb,object,self.knowledge_base.active_grasp)
            cspace.extra_feasibility_tests = extra_feasibility_tests
            limbConfig = self.planner.get_limb_config(currentConfig,limb)
            feasible = cspace.feasible(limbConfig)

        if(feasible):
            #dont need to do anything
            cspace.close()
            return currentConfig
        else:

            #make sure that the joint limits are not exceeded by some small number
            qmin,qmax = self.robot.getJointLimits()
            for n,jointAngle in enumerate(currentConfig):
                if(jointAngle < qmin[n]):
                    jointAngle = qmin[n]
                elif(jointAngle > qmax[n]):
                    jointAngle = qmax[n]

                currentConfig[n] = jointAngle


            self.robot.setConfig(currentConfig)

            #recheck to see if this fixed the problem
            limbConfig = self.planner.get_limb_config(currentConfig,limb)
            feasible = cspace.feasible(limbConfig)
            if(feasible):
                cspace.close()
                return currentConfig

            #perturb the config until it is feasible
            counter = 0
            for d in range(1,6):
                d *= .01
                for dx in [0, -d, d]:
                    for dy in [0, -d, d]:
                        for dz in [0, -d, d]:
                            #find an ik solution that moves the arm dx,dy,dz
                            goal = ik.objective(link,R=link.getTransform()[0],t=vectorops.add(link.getTransform()[1],[dx,dy,dz]))
                            sortedSolutions = self.get_ik_solutions([goal],[limb],currentConfig,maxIters=100,validity_checker=collisionchecker,verbose=False)
                            if len(sortedSolutions) != 0:
                                for ikSoln in sortedSolutions:
                                    #check if this soln is feasible
                                    limbConfig = self.planner.get_limb_config(ikSoln[0],limb)
                                    feasible = cspace.feasible(limbConfig)
                                    if(feasible):
                                        cspace.close()
                                        return ikSoln[0]

                    
                            self.robot.setConfig(currentConfig)

                counter = counter + 1
        
        cspace.close()
        logger.warn('Could not find a way to perturb the starting config to make it feasible!')

        return None

##################################################################################################
#pick up the order tray method
#tray does not need to be upright. Should not have an item in it.
##################################################################################################
    def planPickUpOrderTray(self, pickUp):

        if(not pickUp):

            self.knowledge_base.active_grasp = self.trayGraspXform
            
            object = self.objectDict['tray']
            
            #get the object 

            #set the collision checker
            collisionchecker = lambda x:self.planner.check_collision_free_with_object(x,object,self.trayGraspXform)

            #move the object to where ever the left gripper is
            objToGripperXForm = se3.mul(baxter.left_gripper_center_xform,se3.inv(self.trayGraspXform))
            Tgripper = self.left_gripper_link.getTransform();
            Tobj = se3.mul(Tgripper,objToGripperXForm)
            object.setTransform(*Tobj)

        else:
            object = None
            collisionchecker = None
            baxter.set_model_gripper_command(self.robot,'left',[0.8,0.8,0.8,1])

        if(not pickUp):
            goal_xform = ([1,0,0,0,-1,0,0,0,-1], [0.17, 0.56, -0.42])
        else:
            goal_xform = ([1,0,0,0,-1,0,0,0,-1], [0.17, 0.56, -0.52])

         #get ik solutions for the bin
        planMoveTrayUp = self.planMoveToXform(goal_xform=([1,0,0,0,-1,0,0,0,-1], [0.17, 0.552327180434089, -0.1]),
                                    limb='left',
                                    object=object,
                                    link='gripper',
                                    colMargin=0,
                                    collisionchecker=collisionchecker,
                                    initialConfig=self.robot_state.commanded_config,
                                    checkCols=True,
                                    plan=True)
        if(planMoveTrayUp is None):
            logger.warn('No way to move the tray above home location')
            return None

        self.robot.setConfig(planMoveTrayUp[-1])
        #move the tray to where the gripper is
        if(not pickUp):
            Tgripper = self.left_gripper_link.getTransform()
            Tobj = se3.mul(Tgripper,objToGripperXForm)
            object.setTransform(*Tobj)
            cspace = ('xrestrict',0.17)
        else:
            cspace = ('xrestrictNoobj',0.17,0)


        object = self.objectDict['tray']

        #get ik solutions for the bin
        pickUpTrayPlan = self.planMoveToXform(goal_xform=goal_xform,
                                    limb='left',
                                    object=object,
                                    link='gripper',
                                    colMargin=0,
                                    collisionchecker=collisionchecker,
                                    initialConfig=planMoveTrayUp[-1],
                                    checkCols=True,
                                    cspace=cspace,
                                    plan=True)
        if(pickUpTrayPlan is None):
            logger.warn('No way to move the tray to the home location')
            return None

        #self.robot.setConfig(pickUpTrayPlan[-1])
        #visualization.debug(self.world)


        if(pickUp):
            #close gripper
            plan = [
                ('left_gripper', [0.8,0.8,0.8,1.0], 0, 0),
                ('path',planMoveTrayUp,0,0),
                ('path',pickUpTrayPlan,0,1),
                ('left_gripper', [0.2,0.2,0.2,1.0], 0, 0)
            ]
        else: 
            #opengripper
            plan = [
                ('path',planMoveTrayUp,0,0),
                ('path',pickUpTrayPlan,0,1),
                ('left_gripper', [0.8,0.8,0.8,1.0], 0, 0)
            ]


        

        return (plan, 1, 'left', None) 
        

##################################################################################################
#move the order tray to under the target bin
#tray does not have to be upright
##################################################################################################
    def planMoveTrayToBin(self, binLoc):

        #move the object to where ever the left gripper is and close the gripper
        object = self.objectDict['tray']
        objToGripperXForm = se3.mul(baxter.left_gripper_center_xform,se3.inv(self.trayGraspXform))
        Tgripper = self.left_gripper_link.getTransform();
        Tobj = se3.mul(Tgripper,objToGripperXForm)
        object.setTransform(*Tobj)
        baxter.set_model_gripper_command(self.robot,'left',[0.2,0.2,0.2,1])

        goalRotMatrix = [1,0,0,
                         0,0,1,
                         0,-1,0]

        goalRotMatrix = so3.mul(goalRotMatrix,so3.rotation([0,1,0],math.pi/180.0*10))
        goalRotMatrix = so3.mul(goalRotMatrix,so3.rotation([1,0,0],math.pi/180*25))
        binBound0 = se3.apply(self.knowledge_base.shelf_xform, self.knowledge_base.bin_bounds[binLoc][0])
        binBound1 = se3.apply(self.knowledge_base.shelf_xform, self.knowledge_base.bin_bounds[binLoc][1])
        centerBin = [(a+b)/2.0 for a,b in zip(binBound0,binBound1)]

        goalXYZ = vectorops.add(centerBin,[-0.28,0.2,-0.23])

        goal_xform = (goalRotMatrix,goalXYZ)

        self.knowledge_base.active_grasp = self.trayGraspXform
        
        object = self.objectDict['tray']
        
        #get the object 

        #set the collision checker
        collisionchecker = lambda x:self.planner.check_collision_free_with_object(x,object,self.trayGraspXform)

        #get ik solutions for the bin
        planMoveTrayUp = self.planMoveToXform(goal_xform=([1,0,0,0,-1,0,0,0,-1], [0.21905182823132738, 0.552327180434089, -0.1]),
                                    limb='left',
                                    object=object,
                                    link='gripper',
                                    colMargin=0,
                                    collisionchecker=collisionchecker,
                                    initialConfig=self.robot_state.commanded_config,
                                    checkCols=True,
                                    plan=True)
        if(planMoveTrayUp is None):
            logger.warn('No way to move the tray up')
            return None

        #get ik solutions for the bin
        planMoveTrayToBin = self.planMoveToXform(goal_xform=goal_xform,
                                    limb='left',
                                    object=object,
                                    link='gripper',
                                    colMargin=0,
                                    collisionchecker=collisionchecker,
                                    initialConfig=planMoveTrayUp[-1],
                                    checkCols=True,
                                    plan=True)

        if(planMoveTrayToBin is None):
            logger.warn('No way to move the tray to %s',binLoc)
            return None
        logger.debug('Found a way to move the tray to %s',binLoc)


        #self.robot.setConfig(planMoveTrayToBin[-1])
        #self.robot.setConfig(planMoveTrayToBin[0][0])
        objToGripperXForm = se3.mul(baxter.left_gripper_center_xform,se3.inv(self.trayGraspXform))
        Tgripper = self.left_gripper_link.getTransform();
        Tobj = se3.mul(Tgripper,objToGripperXForm)
        object.setTransform(*Tobj)
        #visualization.debug(self.world)

        return ([('path',planMoveTrayUp,0,0),('path',planMoveTrayToBin,0,0)], 1, 'left', None)


##################################################################################################
#move the order tray to the order bin
#tray needs to be upright
##################################################################################################
    def planMoveTrayToOrderBin(self):

        #move the object to where ever the left gripper is
        object = self.objectDict['tray']
        objToGripperXForm = se3.mul(baxter.left_gripper_center_xform,se3.inv(self.trayGraspXform))
        Tgripper = self.left_gripper_link.getTransform();
        Tobj = se3.mul(Tgripper,objToGripperXForm)
        object.setTransform(*Tobj)


        goal = se3.apply(self.knowledge_base.order_bin_xform,[-0.14,0.2,self.knowledge_base.max_drop_height+0.32])
        goalRotMatrix = [1,0,0,
                         0,0,1,
                         0,-1,0]
        goalRotMatrix = so3.mul(goalRotMatrix,so3.rotation([0,1,0],math.pi/180.0*45))
        goalRotMatrix = so3.mul(goalRotMatrix,so3.rotation([1,0,0],math.pi/180.0*15))
        goal_xform = (goalRotMatrix,goal)




        self.knowledge_base.active_grasp = self.trayGraspXform
        
        #get the object 

        #set the collision checker
        collisionchecker = lambda x:self.planner.check_collision_free_with_object(x,object,self.trayGraspXform)



        #get ik solutions for the bin
        planMoveTrayToOrderBin = self.planMoveToXform(goal_xform=goal_xform,
                                    limb='left',
                                    object=object,
                                    link='gripper',
                                    colMargin=0,
                                    collisionchecker=collisionchecker,
                                    initialConfig=self.robot_state.commanded_config,
                                    cspace=('trayUpright',0),
                                    checkCols=True,
                                    plan=True)
        if(planMoveTrayToOrderBin is None):
            logger.warn('No way to move the tray to the order bin')
            return None
        
        ################debug#####################
        self.robot.setConfig(planMoveTrayToOrderBin[-1])
        # self.robot.setConfig(planMoveTrayToOrderBin[0][0])
        Tgripper = self.left_gripper_link.getTransform();
        Tobj = se3.mul(Tgripper,objToGripperXForm)
        object.setTransform(*Tobj)
        # visualization.debug(self.world)
        ##########################################

        goalRotMatrix = so3.mul(goalRotMatrix,so3.rotation([0,0,1],math.pi/180.0*180))
        goal_xform = (goalRotMatrix,goal)
        planFlipTray = self.planMoveToXform(goal_xform=goal_xform,
                                    limb='left',
                                    object=object,
                                    link='gripper',
                                    colMargin=0,
                                    collisionchecker=collisionchecker,
                                    initialConfig=planMoveTrayToOrderBin[-1],
                                    checkCols=True,
                                    plan=True)

        ################debug#####################
        self.robot.setConfig(planFlipTray[-1])
        Tgripper = self.left_gripper_link.getTransform();
        Tobj = se3.mul(Tgripper,objToGripperXForm)
        object.setTransform(*Tobj)
        # visualization.debug(self.world)
        ##########################################
        if(planFlipTray is None):
            logger.warn('No way to flip the tray into the order bin')
            return None
            
        
        #move the tray above the home location to get out of the way of the scoop
        planMoveTrayUp = self.planMoveToXform(goal_xform=([1,0,0,0,-1,0,0,0,-1], [0.17, 0.552327180434089, -0.1]),
                                    limb='left',
                                    object=object,
                                    link='gripper',
                                    colMargin=0,
                                    collisionchecker=collisionchecker,
                                    initialConfig=planFlipTray[-1],
                                    checkCols=True,
                                    plan=True)
        if(planMoveTrayUp is None):
            logger.warn('No way to move the tray above home location')
            return None

        ################debug#####################
        self.robot.setConfig(planMoveTrayUp[-1])
        Tgripper = self.left_gripper_link.getTransform();
        Tobj = se3.mul(Tgripper,objToGripperXForm)
        object.setTransform(*Tobj)
        # visualization.debug(self.world)
        ##########################################

        return ([('path',planMoveTrayToOrderBin,0,0),('path',planFlipTray,0,0),('path',planMoveTrayUp,0,0)], 1, 'left', None)

##################################################################################################
#cached plans method

##################################################################################################        

    def makeCachePlans(self, bin, vantage_point,startingConfig=[]):
        #which bin are we moving to?
        if bin == 'bin_C' or bin == 'bin_F' or bin == 'bin_I' or bin == 'bin_L':
            limb = 'right'
        else:
            limb = 'left'

        path = None
        goodness = None
        if(len(startingConfig) == 0):
            startConfig = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.1, 0.0, 2.358, 0.0, 0.0, 0.3, -0.98, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.18, 0.0, 2.118, 0.04, 0.0, 0.62, 0.901, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            startConfig = startingConfig

        #world offset is where we want the camera to go in x,y,z
        world_offset = se3.apply(self.knowledge_base.shelf_xform, self.knowledge_base.vantage_point_xforms[vantage_point][1])

        #set up the goal
        goal_xform = ([0,0,-1,1,0,0,0,1,0],world_offset);

        #get ik solutions for the bin
        # if(bin == 'bin_I'):
        #     vantagePointIKSolns = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.1, 0.0, 2.358, 0.0, 0.0, 0.3, -0.98, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.9925407819079073, 0.6140116922364682, 1.3112753456352777, 2.618, -0.1773940650099881, 0.0, -0.8853581297489894, -2.1055396564328515, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        # else:
        vantagePointIKSolns = self.planMoveToXform(goal_xform=goal_xform,
                                    limb=limb,
                                    object=None,
                                    link='camera',
                                    colMargin=0.05,
                                    collisionchecker=None,
                                    initialConfig=startConfig,
                                    checkCols=True,
                                    plan=False) 

        chosenIK = -1
        for i,iksol in enumerate(vantagePointIKSolns):
            logger.debug('IK solution %i of %i',i+1,len(vantagePointIKSolns))
            #ask if the user likes this iksoln
            self.robot.setConfig(iksol[0])
            ##########################################
            visualization.debug(self.world)
            ans = raw_input('Is this a good ik solution? (y/n)')
            if(ans == 'y' or ans == 'Y'):
                chosenIK = iksol[0]
            else:
                print 'else'

            if(chosenIK is not -1):
                break


        path = self.planner.plan(startConfig,chosenIK)
        if path != None:
            return path,limb
        else:
            #failed to create a plan
            logger.warn('Failed to create a plan from scratch')
            return None


    def get_ik_solutions(self,goals,limbs,initialConfig=None,maxResults=10,maxIters=100,tol=1e-3,validity_checker=None,checkCols=True,verbose=True):
        """Given a list of goals and their associated limbs, returns a list
        of (q,index) pairs, where q is an IK solution for goals[index].
        The results are sorted by distance from initialConfig.

        Arguments:
            - goals: a list of IKObjectives
            - limbs: a list of 'left'/'right', one for each goal, corresponding
              to the limb that the goal is defined on
            - initialConfig: optionally, a configuration for the IK seed
            - maxResults: the maximum number of IK results to return
            - maxIters: the maximum number of random samples to draw
            - tol: the ik solving tolerance
            - validity_checker: optionally, a special collision checker f(limb)
              that returns True if the robot's current limb configuration is valid.
              If None, the standard planner collision checker is used.

        Returns: a list [(solution1,index1),...,(solutionn,indexn)] of up to
        maxResults collision-free solutions.
        """
        collides = False
        if initialConfig == None:
            initialConfig = self.robot_state.commanded_config
        if validity_checker == None:
            validity_checker = self.planner.check_collision_free
        numTrials = [0]*len(goals)
        ikSolutions = []
        for i in range(maxIters):
            index = random.randint(0,len(goals)-1)
            goal = goals[index]
            limb = limbs[index]
            numTrials[index] += 1
            if numTrials[index] == 1:
                self.robot.setConfig(initialConfig)
            else:
                self.randomize_limb_position(limb,center=initialConfig,range=0.05*(numTrials[index]-1))
            if ik.solve(goal,tol=tol):
                if(checkCols):
                    if validity_checker(limb):
                        ikSolutions.append((self.robot.getConfig(),index))
                        if len(ikSolutions) >= maxResults: break
                    else:
                        collides = True
                else:
                    ikSolutions.append((self.robot.getConfig(),index))
                    if len(ikSolutions) >= maxResults: break
        if len(ikSolutions)==0:
            if(collides and verbose):
                logger.debug(bcolors.FAIL+"No collision free IK solution"+bcolors.ENDC)
            elif(verbose):    
                logger.debug(bcolors.FAIL+"IK solution infeasible"+bcolors.ENDC)
            return []
        sortedSolutions = sorted([(vectorops.distanceSquared(solution[0],initialConfig),solution) for solution in ikSolutions])

        if(checkCols==False and len(sortedSolutions)>0 and verbose):
            print 'IK solution might have collisions here'
            self.robot.setConfig(sortedSolutions[0][1][0])
            self.planner.check_collision_free(limb,verbose=True)

        return [s[1] for s in sortedSolutions]

    def randomize_limb_position(self,limb,center=None, range=None):
        """Helper: randomizes the limb configuration in self.robot.
        limb can be 'left' or 'right'.  If range is provided, then
        this samples in a range around the config center.  If center is not
        provided, it uses the current commanded config"""
        qmin,qmax = self.robot.getJointLimits()
        q = center[:] #baxter.rest_config[:]
        if range == None:
            if limb == 'left':
                for j in self.left_arm_indices:
                    q[j] = random.uniform(qmin[j],qmax[j])
            else:
                for j in self.right_arm_indices:
                    q[j] = random.uniform(qmin[j],qmax[j])
            self.robot.setConfig(q)
        else:
            if center==None:
                center = self.robot_state.commanded_config
            if limb == 'left':
                for j in self.left_arm_indices:
                    q[j] = random.uniform(max(qmin[j],center[j]-range),min(qmax[j],center[j]+range))
            else:
                for j in self.right_arm_indices:
                    q[j] = random.uniform(max(qmin[j],center[j]-range),min(qmax[j],center[j]+range))
            self.robot.setConfig(q)
        return


def scoopOrGrasp(object,bin,knowledge_base):
    #decide whether this is a point cloud scoop or point cloud grasp
    #based on the object and the bin


    #create numpy array from the pc
    objectPCloud = knowledge_base.object_clouds[object]
    objectPCloud = map(lambda p: p[:3], objectPCloud)
    pcArray = numpy.array(objectPCloud)
    width = pcArray[:,1].max()-pcArray[:,1].min()
    depth = pcArray[:,0].max()-pcArray[:,0].min()
    height = pcArray[:,2].max()-pcArray[:,2].min()

    #which bin are we in and can we scoop/grasp?
    scoopBins=['bin_B','bin_E','bin_H','bin_K','bin_C','bin_F','bin_I','bin_L']
    graspBins=['bin_A','bin_D','bin_F','bin_G','bin_I','bin_J','bin_B','bin_E','bin_H','bin_K']

    canScoop = bin in scoopBins
    canGrasp = bin in graspBins

    if object == 'champion_copper_plus_spark_plug':
        if(canScoop):
            #do scoop
            return 'scoop'
        else:
            #do pinch
            return 'pinch'

    elif object == 'cheezit_big_original':
        if(canGrasp):
            #do power
            return 'power'
        else:
            return None

    elif object == 'crayola_64_ct':
        if(canGrasp):
            if(height < 0.09):
                #do tilt power
                return 'tilt'
            else:
                #do power
                return 'power'
        else:
            if(height < 0.09):
                #do scoop
                return 'scoop'

    elif object == 'dr_browns_bottle_brush':
        return 'tilt'

    elif object == 'elmers_washable_no_run_school_glue':
        if(canGrasp):
            if(height < 0.12):
                #do tilt power
                return 'tilt'
            else:
                #do power
                return 'power'
        else:
            #do scoop
            return 'scoop'

    elif object == 'expo_dry_erase_board_eraser':
        if(canGrasp):
            if(height < 0.1):
                #do tilt power
                return 'tilt'
            else:
                #do power
                return 'power'
        else:
            #do scoop
            return 'sccop'

    elif object == 'feline_greenies_dental_treats':
        if(canGrasp):
            if(height < 0.17):
                #do tilt power
                return 'tilt'
            else:
                #do power
                return 'power'
        else:
            #do scoop
            return 'scoop'

    elif object == 'first_years_take_and_toss_straw_cups':
        if(canGrasp):
            if(height < .13):
                #do power
                return 'power'
            else:
                #do tilt power
                return 'tilt'
        else:
            return None

    elif object == 'genuine_joe_plastic_stir_sticks':
        if(canGrasp):
            #do power
            return 'power'
        else:
            return None

    elif object == 'highland_6539_self_stick_notes':
        if(canGrasp):
            if (height < 0.09):
                #do tilt power
                return 'tilt'
            else:
                #do power
                return 'power'
        else:
            #do scoop
            return 'scoop'

    elif object == 'kong_air_dog_squeakair_tennis_ball':
        if(canGrasp):
            #do power tilt
            return 'tilt'
        else:
            return None

    elif object == 'kong_duck_dog_toy':
        if(canScoop):
            #do scoop
            return 'scoop'
        else:
            #do pinch
            return 'pinch'

    elif object == 'kong_sitting_frog_dog_toy':
        if(canScoop):
            #do scoop
            return 'scoop'
        else:
            #do pinch
            return 'pinch'

    elif object == 'kygen_squeakin_eggs_plush_puppies':
        if(canGrasp):
            #do tilt power
            return 'tilt'
        elif(canScoop):
            #do scoop
            return 'scoop'

    elif object == 'mark_twain_huckleberry_finn':
        if(canScoop):
            #do scoop
            return 'scoop'
        elif(canGrasp):
            #do pinch
            return 'pinch'

    elif object == 'mead_index_cards':
        if(height < 0.1 and canScoop):
            #do scoop
            return 'scoop'
        elif(height < 0.1 and canGrasp):
            #do tilt power
            return 'pinch'
        elif(canGrasp):
            #do power
            return 'power'

    elif object == 'mommys_helper_outlet_plugs':
        if(canGrasp):
            #do tilt power
            return 'tilt'
        else:
            #cant scoop
            return None

    elif object == 'munchkin_white_hot_duck_bath_toy':
        if(canGrasp):
            #do power
            return 'power'
        else:
            #cant scoop
            return None

    elif object == 'oreo_mega_stuf':
        if(canGrasp):
            if(width < 0.08):
                #do power
                return 'power'
        else:
            return None
    
    elif object == 'paper_mate_12_count_mirado_black_warrior':
        if(canScoop):
            #do scoop
            return 'scoop'
        elif(canGrasp):
            #do pinch
            return 'pinch'
    
    elif object == 'rollodex_mesh_collection_jumbo_pencil_cup':
        if(canGrasp):
            #do power
            return 'power'
        else:
            #cant scoop
            return None

    elif object == 'safety_works_safety_glasses':
        if(canGrasp):
            #do power tilt
            return 'tilt'
        elif(canScoop):
            #do scoop
            return 'scoop'

    elif object == 'sharpie_accent_tank_style_highlighters':
        if(canScoop):
            #do scoop
            return 'scoop'
        elif(canGrasp):
            #do pinch
            return 'pinch'

    elif object == 'stanley_66_052':
        if(canScoop):
            #do scoop
            return 'scoop'
        elif(canGrasp):
            #do pinch
            return 'pinch'
    else:
        raise RuntimeError("Unknown object "+object)


