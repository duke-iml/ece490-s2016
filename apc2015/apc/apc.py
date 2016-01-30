"""Data structures for constant elements in the Amazon Picking Challenge. """

from klampt import so3,se3,vectorops
from klampt.robotsim import Geometry3D
import math
import os
import json

#Global: names of bins
bin_names = ['bin_'+c for c in ['A','B','C','D','E','F','G','H','I','J','K','L']]

#shelf coordinates are centered at the bottom center of the shelf.
#x: left to right
#y: bottom to top
#z: back to front.

#Global: map of bin names to bounding boxes ([ax,ay,az],[bx,by,bz]) in local
#frame of shelf object.  
bin_bounds = {
    'bin_A' : ([-0.41,1.55,0],[-0.158,1.78,0.42]),
    'bin_B' : ([-0.149,1.55,0],[0.149,1.78,0.42]),
    'bin_C' : ([0.158,1.55,0],[0.41,1.78,0.42]),
    'bin_D' : ([-0.41,1.32,0],[-0.158,1.52,0.42]),
    'bin_E' : ([-0.149,1.32,0],[0.149,1.52,0.42]),
    'bin_F' : ([0.158,1.32,0],[0.41,1.52,0.42]),
    'bin_G' : ([-0.41,1.09,0],[-0.158,1.29,0.42]),
    'bin_H' : ([-0.149,1.09,0],[0.149,1.29,0.42]),
    'bin_I' : ([0.158,1.09,0],[0.41,1.29,0.42]),
    'bin_J' : ([-0.41,0.82,0],[-0.158,1.06,0.42]),
    'bin_K' : ([-0.149,0.82,0],[0.149,1.06,0.42]),
    'bin_L' : ([0.158,0.82,0],[0.41,1.06,0.42]),
    }

#Global: the transformation of the order bin relative to the robot-centric
#frame
order_bin_xform = (so3.identity(),[0.5,0,0])
#Global: the local bounding box of the order bin
order_bin_bounds = ([-0.2,-0.4,0],[0.2,0.4,0.7])


class ItemGrasp:
    """An ItemGrasp defines where the gripper should go for a given object.
    It has the following members:
        - name: if provided, a name of the grasp
        - grasp_xform: the location of the grasp center, relative to the
          object's local frame
        - gripper_close_command: a command to be sent to the hand's fingers
          as a pregrasp configuration
        - gripper_close_command: a command to be sent to the hand's fingers to
          close on the object
    """
    def __init__(self,xform = se3.identity()):
        #a
        self.name = None
        #the transformation of the gripper fingers relative to the object's
        #local coordinates
        self.grasp_xform = xform
        #the command to be sent to the gripper's fingers to close it properly
        #around this object
        self.gripper_close_command = [0]
        #the command to be sent to the gripper's fingers to open it back
        #after grasping this object
        self.gripper_open_command = [1]

    def readFromDict(self,obj):
        """Reads from a dictionary"""
        if 'name' in obj:
            self.name = obj['name']
        self.grasp_xform = obj['grasp_xform']
        self.gripper_close_command = obj['gripper_close_command']
        self.gripper_open_command = obj['gripper_open_command']
    def readFromJson(self,string):
        """Reads from a JSON string"""
        obj = json.loads(string)
        self.readFromDict(obj)
    def writeToDict(self):
        """Writes to a dictionary"""
        res= {'grasp_xform':self.grasp_xform,
              'gripper_close_command':self.gripper_close_command,
              'gripper_open_command':self.gripper_open_command}
        if self.name != None:
            res['name'] = self.name
        return res
    def writeToJson(self):
        """Writes to a JSON string"""
        return json.dumps(self.writeToDict())


class ItemInfo:
    """Constant information about an object.
    Members include:
    - name: the identifier for this item
    - mass: the item's mass
    - inertia: the item's inertia matrix (either 3 diagonal entries or 9
      entries of matrix)
    - bmin, bmax: the axis-aligned bounding box of the object
    - geometryFile: a file name pointing to the object geometry
    - geometry: the Geometry3D of the object
    - grasps: a list of candidate ItemGrasps that can be used on the object
      (with transforms specified in the local coordinates of the object)
    """
    def __init__(self,name):
        self.name = name
        self.mass = 0.1
        self.bmin = [-0.1,-0.1,-0.1]
        self.bmax = [0.1,0.1,0.1]
        self.autoInertia()
        self.geometryFile = None
        self.geometry = None
        self.grasps = []

    def autoInertia(self):
        bmin,bmax = self.bmin,self.bmax
        self.inertia = vectorops.mul([bmax[0]-bmin[0],bmax[1]-bmin[1],bmax[2]-bmin[2]],self.mass/12.0)

class ItemInBin:
    """An item inside a bin at a given location.
    Members include:
    - info: an ItemInfo structure
    - bin_name: the bin the item is located in
    - xform: the transformation of the object from local to world coordinates.
      This may be None if it is known that an object is in a bin, but not
      where it is located.  In this case, the robot needs to run the pose
      estimator to localize it.
    - klampt_index: if not None, the rigidObject index in the Klamp't
      planning world.
    """
    def __init__(self,info,bin_name):
        self.info = info
        self.bin_name = bin_name
        self.xform = None
        self.klampt_index = None
    def is_localized(self):
        return self.xform != None
    def set_in_bin_xform(self,shelf_xform,ux,uy,theta):
        """A utility convenience function for setting up virtual shelves.
        
        Sets self.xform assuming the object is resting in the bin
        with its center's translational parameters (ux,uy) in the
        range [0,1]x[0,1] and orientation theta about the z axis.
        ux goes from left to right and uy goes from front to back."""
        global bin_bounds
        bmin,bmax = bin_bounds[self.bin_name]
        tlocal = [bmin[0]+ux*(bmax[0]-bmin[0]),bmin[1]-self.info.bmin[2],bmax[2]+uy*(bmin[2]-bmax[2])]
        Rlocal = so3.from_axis_angle(([0,1,0],theta))
        toShelfCoords = (so3.from_axis_angle(([1,0,0],-math.pi/2)),[0,0,0])
        self.xform = se3.mul(shelf_xform,se3.mul((Rlocal,tlocal),toShelfCoords))



item_model_path = '../klampt_models/items'

#Global: a dict mapping possible item names to ItemInfo structures
item_info = {}


def load_fake_items():
    """Loads the fake items from the homework assignments"""
    global item_info
    #set up some items and some potential grasps
    tall_item = ItemInfo('tall_item')
    tall_item.bmin = [-0.03,-0.05,-0.1]
    tall_item.bmax = [0.03,0.05,0.1]
    tall_item.geometryFile = 'box'
    XtoY = so3.from_axis_angle(([0,0,1],math.pi*0.5))
    Xto_Y = so3.from_axis_angle(([0,0,1],-math.pi*0.5))
    XtoZ = so3.from_axis_angle(([0,1,0],math.pi*0.5))
    Xto_Z = so3.from_axis_angle(([0,1,0],-math.pi*0.5))
    ZtoY = so3.from_axis_angle(([1,0,0],math.pi*0.5))
    Zto_Y = so3.from_axis_angle(([1,0,0],-math.pi*0.5))
    XYflip = so3.from_axis_angle(([0,0,1],math.pi))
    XZflip = so3.from_axis_angle(([0,1,0],math.pi))
    YZflip = so3.from_axis_angle(([1,0,0],math.pi))
    tall_item.grasps.append(ItemGrasp((Zto_Y,[0,-0.03,0])))
    tall_item.grasps.append(ItemGrasp((Zto_Y,[0,-0.03,-0.02])))
    tall_item.grasps.append(ItemGrasp((Zto_Y,[0,-0.03,-0.04])))
    tall_item.grasps.append(ItemGrasp((Zto_Y,[0,-0.03,0.02])))
    tall_item.grasps.append(ItemGrasp((Zto_Y,[0,-0.03,0.04])))
    tall_item.grasps.append(ItemGrasp((so3.mul(Zto_Y,XYflip),[0,-0.03,0])))
    tall_item.grasps.append(ItemGrasp((so3.mul(Zto_Y,XYflip),[0,-0.03,-0.02])))
    tall_item.grasps.append(ItemGrasp((so3.mul(Zto_Y,XYflip),[0,-0.03,-0.04])))
    tall_item.grasps.append(ItemGrasp((so3.mul(Zto_Y,XYflip),[0,-0.03,0.02])))
    tall_item.grasps.append(ItemGrasp((so3.mul(Zto_Y,XYflip),[0,-0.03,0.04])))
    tall_item.grasps.append(ItemGrasp((ZtoY,[0,0.03,0])))
    tall_item.grasps.append(ItemGrasp((ZtoY,[0,0.03,-0.02])))
    tall_item.grasps.append(ItemGrasp((ZtoY,[0,0.03,-0.04])))
    tall_item.grasps.append(ItemGrasp((ZtoY,[0,0.03,0.02])))
    tall_item.grasps.append(ItemGrasp((ZtoY,[0,0.03,0.04])))
    tall_item.grasps.append(ItemGrasp((so3.mul(ZtoY,XYflip),[0,0.03,0])))
    tall_item.grasps.append(ItemGrasp((so3.mul(ZtoY,XYflip),[0,0.03,-0.02])))
    tall_item.grasps.append(ItemGrasp((so3.mul(ZtoY,XYflip),[0,0.03,-0.04])))
    tall_item.grasps.append(ItemGrasp((so3.mul(ZtoY,XYflip),[0,0.03,0.02])))
    tall_item.grasps.append(ItemGrasp((so3.mul(ZtoY,XYflip),[0,0.03,0.04])))

    small_item = ItemInfo('small_item')
    small_item.bmin = [-0.01,-0.04,-0.01]
    small_item.bmax = [0.01,0.04,0.01]
    small_item.geometryFile = 'box'
    small_item.grasps.append(ItemGrasp((so3.identity(),[0,-0.02,0])))
    small_item.grasps.append(ItemGrasp((so3.identity(),[0,0.0,0])))
    small_item.grasps.append(ItemGrasp((so3.identity(),[0,0.02,0])))
    small_item.grasps.append(ItemGrasp((XtoZ,[0,-0.02,0])))
    small_item.grasps.append(ItemGrasp((XtoZ,[0,0.0,0])))
    small_item.grasps.append(ItemGrasp((XtoZ,[0,0.02,0])))
    small_item.grasps.append(ItemGrasp((Xto_Z,[0,-0.02,0])))
    small_item.grasps.append(ItemGrasp((Xto_Z,[0,0.0,0])))
    small_item.grasps.append(ItemGrasp((Xto_Z,[0,0.02,0])))
    small_item.grasps.append(ItemGrasp((XZflip,[0,-0.02,0])))
    small_item.grasps.append(ItemGrasp((XZflip,[0,0.0,0])))
    small_item.grasps.append(ItemGrasp((XZflip,[0,0.02,0])))
    
    med_item = ItemInfo('med_item')
    med_item.bmin = [-0.03,-0.03,-0.03]
    med_item.bmax = [0.03,0.03,0.03]
    med_item.geometryFile = 'box'
    #med_item.grasps.append(ItemGrasp((so3.identity(),[0,0,0])))
    med_item.grasps.append(ItemGrasp((so3.mul(XtoY,ZtoY),[0,0,0])))
    #med_item.grasps.append(ItemGrasp((Xto_Y,[0,0,0])))
    #med_item.grasps.append(ItemGrasp((XYflip,[0,0,0])))
    #med_item.grasps.append(ItemGrasp((XtoZ,[0,0,0])))
    #med_item.grasps.append(ItemGrasp((Xto_Z,[0,0,0])))
    #med_item.grasps.append(ItemGrasp((XZflip,[0,0,0])))
    #med_item.grasps.append(ItemGrasp((ZtoY,[0,0,0])))
    #med_item.grasps.append(ItemGrasp((Zto_Y,[0,0,0])))
    #med_item.grasps.append(ItemGrasp((YZflip,[0,0,0])))
    #gripper local rotational axis is x, perturb +/- along that axis
    for g in med_item.grasps[:]:
        angle = 15.0/180.0*math.pi
        med_item.grasps.append(ItemGrasp((so3.mul(g.grasp_xform[0],so3.from_axis_angle(([1,0,0],angle))),g.grasp_xform[1])))
        med_item.grasps.append(ItemGrasp((so3.mul(g.grasp_xform[0],so3.from_axis_angle(([1,0,0],-angle))),g.grasp_xform[1])))
        angle = 30.0/180.0*math.pi
        med_item.grasps.append(ItemGrasp((so3.mul(g.grasp_xform[0],so3.from_axis_angle(([1,0,0],angle))),g.grasp_xform[1])))
        med_item.grasps.append(ItemGrasp((so3.mul(g.grasp_xform[0],so3.from_axis_angle(([1,0,0],-angle))),g.grasp_xform[1])))
        angle = 45.0/180.0*math.pi
        med_item.grasps.append(ItemGrasp((so3.mul(g.grasp_xform[0],so3.from_axis_angle(([1,0,0],-angle))),g.grasp_xform[1])))
    #flip 180 about z axis
    for g in med_item.grasps[:]:
        med_item.grasps.append(ItemGrasp((so3.mul(g.grasp_xform[0],XYflip),g.grasp_xform[1])))
    
    item_info = dict((i.name,i) for i in [tall_item, small_item, med_item])



def load_item(name,gripper='rethink_electric_gripper'):
    """Returns an ItemInfo with the given name, filling in the mesh name,
    grasps, and mass characteristics.
    Does NOT load the geometry."""
    item = ItemInfo(name)
    item.geometryFile = os.path.join(item_model_path,name,"meshes/poisson.ply")

    #load grasps from files
    graspDir = os.path.join(item_model_path,name,"grasps",gripper)
    if not os.path.exists(graspDir):
        os.makedirs(graspDir)
    for fn in filter(lambda x: x.endswith('json'), os.listdir(graspDir)):
        with open(os.path.join(graspDir,fn),'r') as f:
            obj = json.load(f)
            if isinstance(obj,list):
                #grasp lists
                for i,g in enumerate(obj):
                    gr = ItemGrasp()
                    gr.name = os.path.splitext(fn)[0]+'['+str(i)+']'
                    gr.readFromDict(g)
                    item.grasps.append(gr)
            elif isinstance(obj,dict):
                gr = ItemGrasp()
                gr.name = os.path.splitext(fn)[0]
                gr.readFromDict(obj)
                item.grasps.append(gr)
            else:
                print fn,"not a valid grasp in JSON dict / list format"

    #load physics information
    physicsFile = os.path.join(item_model_path,name,"physics.json")
    try:
        with open(physicsFile,'r') as f:
            obj = json.load(f)
            item.mass = obj["mass"]
            if 'inertia' in obj:
                item.inertia = obj["inertia"]
            else:
                item.autoInertia()
    except IOError:
        print "Warning, couldn't load physics for",item.name,"from",physicsFile
        pass
    item_info[name]=item
    return item

def load_item_geometry(item,geometry_ptr = None):
    """Loads the geometry of the given ItemInfo and returns it.
    If geometry_ptr is provided, then it is assumed to be a Geometry3D
    object and the object geometry is loaded into it."""
    if geometry_ptr == None:
        geometry_ptr = Geometry3D()
    if item.geometryFile == None:
        return None
    elif item.geometryFile == 'box':
        fn = "../klampt_models/cube.tri"
        if not geometry_ptr.loadFile(fn):
            print "Error loading cube file",fn
            exit(1)
        bmin,bmax = item.bmin,item.bmax
        center = vectorops.mul(vectorops.add(bmin,bmax),0.5)
        scale = [bmax[0]-bmin[0],0,0,0,bmax[1]-bmin[1],0,0,0,bmax[2]-bmin[2]]
        translate = vectorops.sub(bmin,center)
        geometry_ptr.transform(scale,translate)
        return geometry_ptr
    else:
        if not geometry_ptr.loadFile(item.geometryFile):
            print "Error loading geometry file",item.geometryFile
            exit(1)           
        item.bmin,item.bmax = geometry_ptr.getBB()
        return geometry_ptr

def load_all_items(gripper='rethink_electric_gripper'):
    """Loads all items into the item_info structure"""
    global item_info
    print "Loading all items in",item_model_path,"..."
    for fn in os.listdir(item_model_path):
        print "   ",fn
        item = load_item(fn,gripper=gripper)

def spawn_item_in_world(item,world):
    """Given an ItemInBin instance, spawns a RigidObject in the Klamp't
    world with its same geometry / size / mass properties.
    Returns the new object."""
    obj = world.makeRigidObject(item.info.name)
    bmin,bmax = item.info.bmin,item.info.bmax
    center = vectorops.div(vectorops.add(bmin,bmax),2.0)
    m = obj.getMass()
    m.setMass(item.info.mass)
    m.setCom([0,0,0])
    m.setInertia(vectorops.mul([bmax[0]-bmin[0],bmax[1]-bmin[1],bmax[2]-bmin[2]],item.info.mass/12.0))
    obj.setMass(m)
    c = obj.getContactParameters()
    #TODO: make these parameters dynamic
    c.kFriction = 0.6
    c.kRestitution = 0.1;
    c.kStiffness = 100000
    c.kDamping = 100000
    obj.setContactParameters(c)
    simgeometry = obj.geometry()
    #either copy directly (this line)...
    simgeometry.set(item.info.geometry)
    #or reload from file (this line)
    #load_item_geometry(item.info,simgeometry)
    obj.setTransform(item.xform[0],item.xform[1])
    return obj

    
def spawn_items_in_world(items,world):
    """Given a list of ItemInBin items, spawns RigidObjects in the Klamp't
    world according to their geometries / sizes / mass properties"""
    print "Spawning Klamp't rigid objects from a list of ItemInBins"
    for item in items:
        spawn_item_in_world(item,world)
    return

class KnowledgeBase:
    """A structure containing the robot's dynamic knowledge about the world.
    Members:
    - bin_contents: a map from bin names to lists of known items in
      the bin.  Items are given by ItemInBin objects.
    - order_bin_contents: the list of objects already in the order bin.
      also given by ItemInBin objects
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
        self.bin_contents = dict((n,None) for n in bin_names)
        self.order_bin_contents = []
        self.shelf_xform = se3.identity()
    
    def bin_front_center(self,bin_name):
        bmin,bmax = bin_bounds[bin_name]
        local_center = [(bmin[0]+bmax[0])*0.5,(bmin[1]+bmax[1])*0.5,bmax[2]]
        world_center = se3.apply(self.shelf_xform,local_center)
        return world_center


    def bin_vantage_point(self,bin_name):
        world_center = self.bin_front_center(bin_name)
        #20cm offset
        world_offset = so3.apply(self.shelf_xform[0],[0,0,0.2])
        #world_center = so3.apply(Rlocal,world_center)
        return vectorops.add(world_center,world_offset)


    def grasp_xforms(self,object):
        if object.xform == None: return None
        res = []
        for g in object.info.grasps:
            grasp_xform_world = se3.mul(object.xform,g.grasp_xform)
            res.append((g,grasp_xform_world))
        return res

if __name__ == "__main__":
    #by default this will load all of the items and their geometry
    #just as a test
    print "Testing... loading all items..."
    load_all_items()
    for item in item_info:
        item.geometry = load_item_geometry(item)
