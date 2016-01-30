#!/usr/bin/python

from klampt import *
from klampt.glprogram import *
from klampt import vectorops,so3,se3,gldraw,ik,loader
import apc
import os
import math
import random

#The path of the klampt_models directory
model_dir = "../klampt_models/"

#indices of the left and right cameras in the Baxter robot file
left_camera_link_name = 'left_hand_camera'
right_camera_link_name = 'right_hand_camera'

#indices of the left and right grippers in the Baxter robot file
left_gripper_link_name = 'left_gripper'
right_gripper_link_name = 'right_gripper'

#indices of the left and right arms in the Baxter robot file
left_arm_link_names = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
right_arm_link_names = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']

#local transformations (rotation, translation pairs) of the grasp center
left_gripper_center_xform = (so3.identity(),[0,-0.04,0.1])
right_gripper_center_xform = (so3.identity(),[0,-0.04,0.1])

#resting configuration
baxter_rest_config = [0.0]*54

#the transformation of the order bin
order_bin_xform = (so3.identity(),[0.5,0,0])
#the local bounding box of the order bin
order_bin_bounds = ([-0.2,-0.4,0],[0.2,0.4,0.7])

class KnowledgeBase:
    """A structure containing the robot's dynamic knowledge about the world.
    Members:
    - bin_contents: a map from bin names to lists of known items in
      the bin.  Items are given by apc.ItemInBin objects.
    - order_bin_contents: the list of objects already in the order bin.
      also given by apc.ItemInBin objects
    - shelf_xform: the transformation (rotation, translation) of the bottom
      center of the shelf in world coordinates.  The x coordinate increases
      from left to right, the y coordinate increases from bottom to top,
      and the z coordinate increases from back to front.
      this will be loaded dynamically either from perception or hard coded.

    (in this homework assignment we will use the fake perception module
    to populate the bin contents, and assume the shelf xform is
    estimated perfectly.)
    """
    def __init__(self):
        self.bin_contents = dict((n,[]) for n in apc.bin_names)
        self.order_bin_contents = []
        self.shelf_xform = se3.identity()
    
    def bin_front_center(self,bin_name):
        #you may want to implement this.  Returns the world position of the
        #front-center of the bin.  The visualizer will draw these points if
        #'draw_bins' is turned to True.
        #TODO:
	point = [(apc.bin_bounds[bin_name][0][0]+apc.bin_bounds[bin_name][1][0])/2,(apc.bin_bounds[bin_name][0][1]+apc.bin_bounds[bin_name][1][1])/2,.42]
	return se3.apply(self.shelf_xform,point)

    def bin_vantage_point(self,bin_name):
        #you may want to implement this.  Returns the world position of the
        #vantage point for viewing the bin.  The visualizer will draw these points if
        #'draw_bins' is turned to True.
        #TODO:
	bin = self.bin_front_center(bin_name=bin_name)
	bin[0]=bin[0]-.2
        return bin 

    def grasp_xforms(self,object):
        #you may want to implement this.  Returns a list of world transformations
        #for the grasps defined for the given object (an ItemInBin instance).
        #The visualizer will draw these transforms if 'draw_grasps' is turned to True.
        #TODO:
	objGrasps = object.info.grasps
	
	return [se3.mul(object.xform,grasp.grasp_xform) for grasp in objGrasps]
#a list of actual items -- this is only used for the fake perception module, and your
#code should not use these items directly
ground_truth_items = []
ground_truth_shelf_xform = se3.identity()
def init_ground_truth():
    global ground_truth_items
    ground_truth_items = [apc.ItemInBin(apc.tall_item,'bin_B'),
                          apc.ItemInBin(apc.small_item,'bin_D'),
                          apc.ItemInBin(apc.med_item,'bin_H')]
    ground_truth_items[0].set_in_bin_xform(ground_truth_shelf_xform,0.2,0.2,0.0)
    ground_truth_items[1].set_in_bin_xform(ground_truth_shelf_xform,0.5,0.1,math.pi/4)
    ground_truth_items[2].set_in_bin_xform(ground_truth_shelf_xform,0.6,0.4,math.pi/2)

def run_perception_on_bin(knowledge,bin_name):
    """This is a fake perception module that simply reveals all the items
    the given bin."""
    global ground_truth_items
    if knowledge.bin_contents[bin_name]==None:
        #not sensed yet
        knowledge.bin_contents[bin_name] = []
        for item in ground_truth_items:
            if item.bin_name == bin_name:
                #place it in the bin
                knowledge.bin_contents[bin_name].append(item)
    return

class PickingController:
    """Maintains the robot's knowledge base and internal state.  Most of
    your code will go here.  Members include:
    - knowledge: a KnowledgeBase object
    - state: either 'ready' or 'holding'
    - configuration: the robot's current configuration
    - active_limb: the limb currently active, either holding or viewing a state
    - current_bin: the name of the bin where the camera is viewing or the gripper is located
    - held_object: the held object, if one is held, or None otherwise

    External modules can call viewBinAction(), graspAction(), ungraspAction(),
    and placeInOrderBinAction()
    """
    def __init__(self,world):
        self.world = world
        self.robot = world.robot(0)
        self.knowledge = KnowledgeBase()
        self.state = 'ready'
        self.config = self.robot.getConfig()
        self.active_limb = None
        self.current_bin = None
        self.held_object = None
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

    def viewBinAction(self,b):
        if self.state != 'ready':
            print "Already holding an object, can't move to bin"
            return False
        else:
            if b in apc.bin_names:
                if self.move_camera_to_bin(b):
                    self.current_bin = b
                    run_perception_on_bin(self.knowledge,b)
                    print "Sensed bin",b,"with camera",self.active_limb
                else:
                    print "Move to bin",b,"failed"
                    return False
            else:
                print "Invalid bin",b
                return False
        return True
        
    def graspAction(self):
        if self.current_bin == None:
            print "Not located at a bin"
            return False
        elif self.state != 'ready':
            print "Already holding an object, can't grasp another"
            return False
        elif len(self.knowledge.bin_contents[self.current_bin])==0:
            print "The current bin is empty"
            return False
        else:
            if self.move_to_grasp_object(self.knowledge.bin_contents[self.current_bin][0]):
                self.held_object = self.knowledge.bin_contents[self.current_bin].pop(0)
                self.state = 'holding'
                print "Holding object",self.held_object.info.name,"in hand",self.active_limb
                return True
            else:
                print "Grasp failed"
                return False

    def ungraspAction(self):
        if self.state != 'holding':
            print "Not holding an object"
            return False
        else:
            if self.move_to_ungrasp_object(self.held_object):
                print "Object",self.held_object.info.name,"placed back in bin"
                self.knowledge.bin_contents[self.current_bin].append(self.held_object)
                self.state = 'ready'
                self.held_object = None
                return True
            else:
                print "Ungrasp failed"
                return False
        
    def placeInOrderBinAction(self):
        if self.state != 'holding':
            print "Not holding an object"
        else:
            if self.move_to_order_bin(self.held_object):
                print "Successfully placed",self.held_object.info.name,"into order bin"
                self.knowledge.order_bin_contents.append(self.held_object)
                self.held_object.xform = None
                self.held_object.bin_name = 'order_bin'
                self.state = 'ready'
                self.held_object = None
                return True
            else:
                print "Move to order bin failed"
                return False

    def fulfillOrderAction(self,objectList):
        """Given a list of objects to be put in the order bin, run
        until completed."""
	orders = {}
	have = {}
	for item in objectList:
	   orders[item] = 0
           have[item] = 0
	for item in objectList:
	   orders[item] =  orders[item]+1
	for bin_name in apc.bin_names:
		self.viewBinAction(bin_name)
	for bin_name in apc.bin_names:
		for item in self.knowledge.bin_contents[bin_name]:
			have[item.info.name] = 0;
		for itemname in self.knowledge.bin_contents[bin_name]:
			have[item.info.name] = have[item.info.name]+1;
	for item in objectList:
		if(orders[item]>have[item]):
			return False
	for item in objectList:
		count = 0;
		while orders[item]!=0:
			print(str(orders[item])+str(item))
			for bin in apc.bin_names:
				for itemobject in self.knowledge.bin_contents[bin]:
					if itemobject.info.name==item:
						self.move_to_grasp_object(itemobject)
						self.move_to_order_bin(itemobject)
						orders[item]=orders[item]-1
	return True	
    def move_camera_to_bin(self,bin_name):
        """Sets the robot's configuration to a viewpoint that
        observes bin_name.  Will also change self.active_limb to the
        appropriate limb.

        If successful, changes self.config and returns True.
        Otherwise, does not change self.config and returns False.
        """
        #TODO: put my code here
	self.robot.setConfig(baxter_rest_config)
	# load k -> e, then target if not c or f. 
	pworld = self.knowledge.bin_vantage_point(bin_name=bin_name)
	kworld = self.knowledge.bin_vantage_point(bin_name="bin_K")
	eworld = self.knowledge.bin_vantage_point(bin_name="bin_E")
	fworld = self.knowledge.bin_vantage_point(bin_name="bin_F")
	fworld[1] = fworld[1]+.4
	dworld = self.knowledge.bin_vantage_point(bin_name="bin_D")
	dworld[1] = dworld[1]-.4
	if(bin_name != "bin_A" and bin_name != "bin_D" and bin_name!="bin_G" and bin_name!="bin_J"):
		self.active_limb=self.right_camera_link	
		if(bin_name=="bin_C" or bin_name =="bin_F"):
			fobj = ik.objective(self.right_camera_link,R=[0,1,0,0,0,1,1,0,0],t=fworld)
			ik.solve(fobj)
			self.config=self.robot.getConfig()
			pobj = ik.objective(self.right_camera_link,R=[0,1,0,0,0,1,1,0,0],t=pworld)
			if(ik.solve(pobj)):
				self.config=self.robot.getConfig()
				self.active_limb=self.right_gripper_link
				return True
		kobj = ik.objective(self.right_camera_link,R=[0,1,0,0,0,1,1,0,0],t=kworld)
		ik.solve(kobj)
		self.config=self.robot.getConfig()
		if(bin_name=="bin_B"):
			eobj = ik.objective(self.right_camera_link,R=[0,1,0,0,0,1,1,0,0],t=eworld)
			ik.solve(eobj)
			self.config=self.robot.getConfig()
		pobj = ik.objective(self.right_camera_link,R=[0,1,0,0,0,1,1,0,0],t=pworld)
		if(ik.solve(pobj)):
			self.config=self.robot.getConfig()
			self.active_limb=self.right_gripper_link
			return True
	else:	
		self.active_limb=self.left_camera_link	
		if(bin_name=="bin_A" or bin_name =="bin_D"):
			dobj = ik.objective(self.left_camera_link,R=[0,1,0,0,0,1,1,0,0],t=dworld)
			ik.solve(dobj)
			self.config=self.robot.getConfig()
			pobj = ik.objective(self.left_camera_link,R=[0,1,0,0,0,1,1,0,0],t=pworld)
			if(ik.solve(pobj)):
				self.config=self.robot.getConfig()
				self.active_limb=self.right_gripper_link
				return True
		kobj = ik.objective(self.left_camera_link,R=[0,1,0,0,0,1,1,0,0],t=kworld)
		ik.solve(kobj)
		self.config=self.robot.getConfig()
		pobj = ik.objective(self.left_camera_link,R=[0,1,0,0,0,1,1,0,0],t=pworld)
		if(ik.solve(pobj)):
			self.config=self.robot.getConfig()
			self.active_limb=self.left_gripper_link
			return True
	return False
    def move_to_grasp_object(self,object):
        """Sets the robot's configuration so the gripper grasps object at
        one of its potential grasp locations.  Might change self.active_limb
        to the appropriate limb.
        If successful, changes self.config and returns True.
        Otherwise, does not change self.config and returns False.
        """
        # get list of possible grasps
	poss = self.knowledge.grasp_xforms(object=object)
	target = [se3.apply(transform,[0,0,0]) for transform in poss]
	for i in range(len(target)):
		self.robot.setConfig(baxter_rest_config)
		if(ik.solve(ik.objective(self.active_limb,R=poss[i][0],t=target[i]))):
			self.config=self.robot.getConfig()
			return True
	return False


    def move_to_ungrasp_object(self,object):
        """Sets the robot's configuration so the gripper ungrasps the object.

        If successful, changes self.config and returns True.
        Otherwise, does not change self.config and returns False.
        """
        assert len(object.info.grasps) > 0,"Object doesn't define any grasps"
        #right now there's no gripper model.
        #future implementations should set gripper fingers to object.info.grasps[0].gripper_open_command
        return True

    def move_to_order_bin(self,object):
        """Sets the robot's configuration so the gripper is over the order bin

        If successful, changes self.config and returns True.
        Otherwise, does not change self.config and returns False.
        """
	self.robot.setConfig(baxter_rest_config)
	target = se3.apply(order_bin_xform, [0,0,.7])
	if(ik.solve(ik.objective(self.active_limb,R=[0,1,0,1,0,0,0,0,-1],t=target))):
		self.config = self.robot.getConfig()
		return True
	self.config = self.robot.getConfig()
	return False


def draw_xformed(xform,localDrawFunc):
    """Draws something given a se3 transformation and a drawing function
    that draws the object in its local frame.

    E.g., draw_xformed(xform,lambda:gldraw.box([ax,ay,az],[bx,by,bz])) draws
    a box oriented and translated by xform."""
    mat = zip(*se3.homogeneous(xform))
    mat = sum([list(coli) for coli in mat],[])

    glPushMatrix()
    glMultMatrixf(mat)
    localDrawFunc()
    glPopMatrix()

def draw_oriented_box(xform,bmin,bmax):
    """Helper: draws an oriented box"""
    draw_xformed(xform,lambda:gldraw.box(bmin,bmax))

def draw_wire_box(bmin,bmax):
    """Helper: draws a wireframe box"""
    glBegin(GL_LINE_LOOP)
    glVertex3f(bmin[0],bmin[1],bmin[2])
    glVertex3f(bmin[0],bmin[1],bmax[2])
    glVertex3f(bmin[0],bmax[1],bmax[2])
    glVertex3f(bmin[0],bmax[1],bmin[2])
    glEnd()
    glBegin(GL_LINE_LOOP)
    glVertex3f(bmax[0],bmin[1],bmin[2])
    glVertex3f(bmax[0],bmin[1],bmax[2])
    glVertex3f(bmax[0],bmax[1],bmax[2])
    glVertex3f(bmax[0],bmax[1],bmin[2])
    glEnd()
    glBegin(GL_LINES)
    glVertex3f(bmin[0],bmin[1],bmin[2])
    glVertex3f(bmax[0],bmin[1],bmin[2])
    glVertex3f(bmin[0],bmin[1],bmax[2])
    glVertex3f(bmax[0],bmin[1],bmax[2])
    glVertex3f(bmin[0],bmax[1],bmax[2])
    glVertex3f(bmax[0],bmax[1],bmax[2])
    glVertex3f(bmin[0],bmax[1],bmin[2])
    glVertex3f(bmax[0],bmax[1],bmin[2])
    glEnd()

def draw_oriented_wire_box(xform,bmin,bmax):
    """Helper: draws an oriented wireframe box"""
    draw_xformed(xform,lambda:draw_wire_box(bmin,bmax))


class MyGLViewer(GLNavigationProgram):
    """This class is used to interact with the world model in hw2.
    
    Pressing 'a-l' runs the view_bin method which should set the robot to a
    configuration that places a hand camera such that it points inside the
    bin.

    Pressing 's' should "sense" the currently pointed-to bin, running a fake
    perception module.

    Pressing 'x' should "grasp" an object in the currently pointed-to-bin
    with either one of the hands at the designated grasp point.

    Pressing 'u' should "ungrasp" an object currently grasped inside a bin.

    Pressing 'p' should "put down" an object in the order bin
    """
    def __init__(self,world):
        GLNavigationProgram.__init__(self,"My GL program")
        self.world = world
        self.controller = PickingController(world)

        #you can set these to true to draw the bins, grasps, and/or gripper/camera frames
        self.draw_bins = True
        self.draw_grasps = True
        self.draw_gripper_and_camera = True

        #initialize the shelf xform for the visualizer and object
        #xform initialization
        global ground_truth_shelf_xform
        ground_truth_shelf_xform = world.rigidObject(0).getTransform()
        init_ground_truth()
        self.controller.knowledge.shelf_xform = ground_truth_shelf_xform

    def display(self):
        #you may run auxiliary openGL calls, if you wish to visually debug
        self.world.robot(0).setConfig(self.controller.config)
        self.world.drawGL()
        global ground_truth_items
        
        #show bin boxes
        if self.draw_bins:
            glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,1,0,1])
            #for b in apc.bin_bounds.values():
             #   draw_oriented_box(self.controller.knowledge.shelf_xform,b[0],b[1])
            for b in apc.bin_names:
                c = self.controller.knowledge.bin_front_center(b)
                if c:
                    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,1,0.5,1])                    
                    r = 0.01
                    gldraw.box([c[0]-r,c[1]-r,c[2]-r],[c[0]+r,c[1]+r,c[2]+r])
                c = self.controller.knowledge.bin_vantage_point(b)
                if c:
                    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0.5,1,0.5,1])
                    r = 0.01
                    gldraw.box([c[0]-r,c[1]-r,c[2]-r],[c[0]+r,c[1]+r,c[2]+r])

        
        #show object state
        for i in ground_truth_items:
            if i.xform == None:
                continue
            #if perceived, draw in solid color
            if self.controller.knowledge.bin_contents[i.bin_name]!=None and i in self.controller.knowledge.bin_contents[i.bin_name]:
                glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,0.5,0,1])
                draw_oriented_box(i.xform,i.info.bmin,i.info.bmax)
            else:
                #otherwise, draw in wireframe
                glDisable(GL_LIGHTING)
                glColor3f(1,0.5,0)
                draw_oriented_wire_box(i.xform,i.info.bmin,i.info.bmax)
                glEnable(GL_LIGHTING)
            if self.draw_grasps:
                #draw grasps, if available
                g = self.controller.knowledge.grasp_xforms(i)
                if g:
                    for xform in g:
                        gldraw.xform_widget(xform,0.05,0.005)

        #show gripper and camera frames
        if self.draw_gripper_and_camera:
            left_camera_link = self.world.robot(0).getLink(left_camera_link_name)
            right_camera_link = self.world.robot(0).getLink(right_camera_link_name)
            left_gripper_link = self.world.robot(0).getLink(left_gripper_link_name)
            right_gripper_link = self.world.robot(0).getLink(right_gripper_link_name)
            gldraw.xform_widget(left_camera_link.getTransform(),0.1,0.01)
            gldraw.xform_widget(right_camera_link.getTransform(),0.1,0.01)
            gldraw.xform_widget(se3.mul(left_gripper_link.getTransform(),left_gripper_center_xform),0.05,0.005)
            gldraw.xform_widget(se3.mul(right_gripper_link.getTransform(),right_gripper_center_xform),0.05,0.005)

        #draw order box 
        glDisable(GL_LIGHTING)
        glColor3f(1,0,0)
        draw_oriented_wire_box(order_bin_xform,order_bin_bounds[0],order_bin_bounds[1])
        glEnable(GL_LIGHTING)
        return

    def keyboardfunc(self,c,x,y):
        c = c.lower()
        if c >= 'a' and c <= 'l':
            self.controller.viewBinAction('bin_'+c.upper())
        elif c == 'x':
            self.controller.graspAction()
        elif c == 'u':
            self.controller.ungraspAction()
        elif c == 'p':
            self.controller.placeInOrderBinAction()
        elif c == 'o':
            self.controller.fulfillOrderAction(['med_item','small_item'])
        glutPostRedisplay()


def main():
    """The main loop that loads the models and starts the OpenGL visualizer"""
    
    world = WorldModel()
    #uncomment these lines and comment out the next 2 if you want to use the
    #full Baxter model
    #print "Loading full Baxter model (be patient, this will take a minute)..."
    #world.loadElement(os.path.join(model_dir,"baxter.rob"))
    print "Loading simplified Baxter model..."
    world.loadElement(os.path.join(model_dir,"baxter_col.rob"))
    print "Loading Kiva pod model..."
    world.loadElement(os.path.join(model_dir,"kiva_pod/model.obj"))
    print "Loading plane model..."
    world.loadElement(os.path.join(model_dir,"plane.env"))
    
    #shift the Baxter up a bit (95cm)
    Rbase,tbase = world.robot(0).getLink(0).getParentTransform()
    world.robot(0).getLink(0).setParentTransform(Rbase,(0,0,0.95))
    world.robot(0).setConfig(world.robot(0).getConfig())
    
    #translate pod to be in front of the robot, and rotate the pod by 90 degrees 
    Trel = (so3.rotation((0,0,1),-math.pi/2),[1.2,0,0])
    T = world.rigidObject(0).getTransform()
    world.rigidObject(0).setTransform(*se3.mul(Trel,T))

    #load the resting configuration from klampt_models/baxter_rest.config
    global baxter_rest_config
    f = open(model_dir+'baxter_rest.config','r')
    baxter_rest_config = loader.readVector(f.readline())
    f.close()
    world.robot(0).setConfig(baxter_rest_config)
    
    #run the visualizer
    visualizer = MyGLViewer(world)
    visualizer.run()

if __name__ == "__main__":
    main()
