from klampt import *
from klampt import vectorops,so3,se3,gldraw
from klampt.cspace import CSpace,MotionPlan
from klampt.robotsim import Geometry3D
from klampt.robotcollide import WorldCollider
from baxter import *
import apc

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
        get_limb_config(planner.robot.getConfig(), self.limb)


        self.setup()
        self.milestone.getPath

        self.bound = [(0,1),(0,1)]
        self.setBounds(self.bound)
        #TODO: what collision tolerance to use?
        self.eps = 1e-2

    def feasible(self,q):
        #TODO: implement me
        #the default simply checks if q is within the bounds
        return CSpace.feasible(self,q)


class LimbPlanner:
    """Much of your code for HW4 will go here.
    
    Attributes:
        - world: the WorldModel
        - robot: the RobotModel
        - knowledge: the KnowledgeBase object
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

    def set_limb_config(self,limb,limbconfig,q):
        """Helper: Sets the 7-DOF configuration of the given limb in
        q.  Other DOFs are not touched."""
        assert len(q) == self.robot.numLinks()
        if limb=='left':
            print "******** q ********"
            print q
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
        #TODO: implement me
        #robot self-collision check
        rob = self.robot
        nl = rob.numLinks()
        for i in xrange(nl):
            for j in xrange(i):
                if rob.selfCollisionEnabled(i,j):
                    coll_test = rob.getLink(i).geometry().collides(rob.getLink(j).geometry())
                    if coll_test != False:
                        return False
        
        # shelf collision-free check
        # within the world = load_apc_world()
        numT = self.collider.world.numTerrains()
        if numT > 0:
            for i in xrange(numT):
                g = self.collider.world.terrain(i).geometry()
                for j in xrange(nl):
                    if g != None:
                        coll_with_shelf = g.collides(rob.getLink(j).geometry()) ### output "NearbyTriangles: Collider for type error not known"
                        if coll_with_shelf != False:
                            return False

        # object collision-free check
        for b in apc.bin_names: # b is bin_name
            it_in_bin = self.knowledge.bin_contents[b]
            if it_in_bin == None:
                continue
            else:
                for it_num in xrange(len(it_in_bin)):
                    obj = self.world.makeRigidObject(it_in_bin[it_num].info.name) # RigideObjectModel class, it_in_bin is ItemInBin class
                    cube = obj.geometry() # Geometry3D class
                    for j in xrange(nl):
                        coll_with_objects = cube.collides(rob.getLink(j).geometry())
                        if coll_with_objects != False:
                            return False   
        """
        for t in self.terrains:
            if t < 0: continue
            for o in self.rigidObjects:
                if o < 0: continue
                self.mask[t].add(o)
                self.mask[o].add(t)
            for r in self.robots:
                for l in r:
                    if l < 0: continue
                    #test for fixed links
                    if self.geomList[l][0].getParent() >= 0:
                        self.mask[l].add(t)
                        self.mask[t].add(l)
                    else:
                        #print "Ignoring fixed link..."
                        pass
        """
        # world.makeRigidObject
        """
        for item in ground_truth_items:
            obj = self.world.makeRigidObject(item.info.name)
            bmin,bmax = item.info.bmin,item.info.bmax
            center = vectorops.div(vectorops.add(bmin,bmax),2.0)
            m = obj.getMass()
            m.setMass(item.info.mass)
            m.setCom([0,0,0])
            m.setInertia(vectorops.mul([bmax[0]-bmin[0],bmax[1]-bmin[1],bmax[2]-bmin[2]],item.info.mass/12.0))
            obj.setMass(m)
            c = obj.getContactParameters()
            c.kFriction = 0.6
            c.kRestitution = 0.1;
            c.kStiffness = 100000
            c.kDamping = 100000
            obj.setContactParameters(c)
            # call a collision check
            self.robot.getLink(i).geometry().collides(obj.geometry())
            """
        return True
    """
        flag = 0
        left_N = len(self.left_arm_links)
        right_N = len(self.right_arm_links)
        coll_test = self.left_camera_link.geometry().collides(self.right_camera_link.geometry())
        links_list = [self.left_camera_link, self.right_camera_link, self.left_gripper_link, self.right_gripper_link, self.left_arm_links, self.right_arm_links]
        
        # len(links_list) == 6
        for i in xrange(0,len(links_list) - 3):
            for j in xrange(i+1,len(links_list) - 2):
                coll_test = links_list[i].geometry().collides(links_list[j].geometry())
                if coll_test != False:
                    flag = 1
                    break
            if flag == 1:
                break
        for i in xrange(len(links_list)-2,len(links_list)):
            if flag == 1:
                break
            if i == 4:
                ind_N = left_N
            if i == 5:
                ind_N = right_N
            for j in xrange(0,ind_N):
                if links_list[0].geometry().collides(links_list[i][j].geometry()) | links_list[1].geometry().collides(links_list[i][j].geometry()) | links_list[2].geometry().collides(links_list[i][j].geometry()) | links_list[3].geometry().collides(links_list[i][j].geometry()):
                    flag = 1
                    break
        for i in xrange(0,left_N):
            if flag == 1:
                break
            for j in xrange(0,right_N):
                coll_test = links_list[4][i].geometry().collides(links_list[5][j].geometry())
                if coll_test == True:
                    flag = 1
                    break
        if flag == 1:
            return False
        for i in xrange(0,left_N - 2):
            if flag == 1:
                break
            for j in xrange(i+1,left_N - 1):
                coll_test = links_list[4][i].geometry().collides(links_list[4][j].geometry())
                if coll_test == True:
                    print ("line")
                    print (106)
                    print(i,j)
                    flag = 1
                    break
        for i in xrange(0,right_N - 2):
            if flag == 1:
                break
            for j in xrange(i+1,right_N - 1):
                coll_test = links_list[5][i].geometry().collides(links_list[5][j].geometry())
                if coll_test == True:
                    flag = 1
                    break
    """
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
        self.planner = motionplanning.PlannerInterface(self)
        return [start,goal]

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
