""" Holds data structures for shared constant elements in the APC """

# so3 = rotations; se3 = rigid transformations (R,T)
from klampt import so3, se3
import math

# bin names
bin_names = ['bin_'+c for c in ['A','B','C','D','E','F','G','H','I','J','K','L']]

# shelf coordinates are centered at the bottom center of the shelf.
# x: left to right
# y: bottom to top
# z: back to front.

# maps bin names to bounding boxes ([ax,ay,az],[bx,by,bz]) in local frame of shelf object.
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

    'bin_J' : ([-0.41,0.825,0],[-0.158,1.06,0.42]),
    'bin_K' : ([-0.149,0.825,0],[0.149,1.06,0.42]),
    'bin_L' : ([0.158,0.825,0],[0.41,1.06,0.42]),

    }

# rotation matrices
# NOTE: XtoZ ~ Zto_Y are flipped?
XtoY = so3.from_axis_angle(([0,0,1],math.pi*0.5))
Xto_Y = so3.from_axis_angle(([0,0,1],-math.pi*0.5))
XtoZ = so3.from_axis_angle(([0,1,0],math.pi*0.5))
Xto_Z = so3.from_axis_angle(([0,1,0],-math.pi*0.5))
ZtoY = so3.from_axis_angle(([1,0,0],math.pi*0.5))
Zto_Y = so3.from_axis_angle(([1,0,0],-math.pi*0.5))
XYflip = so3.from_axis_angle(([0,0,1],math.pi))
XZflip = so3.from_axis_angle(([0,1,0],math.pi))
YZflip = so3.from_axis_angle(([1,0,0],math.pi))


class ItemInfo:
    """Constant information about an object.
    Members include:
    - name: the identifier for this item
    - mass: the item's mass
    - bmin, bmax: the axis-aligned bounding box of the object
    - geometryFile: a file name pointing to the object geometry
    - geometry: the Geometry3D of the object
    - grasps: a list of candidate ItemGrasps that can be used on the object
      (with transforms specified in the local coordinates of the object)"""
    def __init__(self,name):
        self.name = name
        self.mass = 1.1
        self.bmin = [-0.1,-0.1,-0.1]
        self.bmax = [0.1,0.1,0.1]
        self.geometryFile = None
        self.geometry = None
        self.grasps = []

class ItemGrasp:
    """An ItemGrasp defines where the gripper should go for a given object.
    It has two members:
        - grasp_xform: the location of the grasp center, relative to the
          object's local frame
        - gripper_finger_command: a command to be sent to the hand's fingers to
          grasp the object (not used in this assignment)
    """
    def __init__(self,xform = se3.identity()):
        #the transformation of the gripper fingers relative to the object's
        #local coordinates
        self.grasp_xform = xform
        #the command to be sent to the gripper's fingers to close it properly
        #around this object
        self.gripper_close_command = [0]
        #the command to be sent to the gripper's fingers to open it back
        #after grasping this object
        self.gripper_open_command = [1]

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
        toShelfCoords = (so3.from_axis_angle(([1,0,0],math.pi/2)),[0,0,0])
        self.xform = se3.mul(shelf_xform,se3.mul((Rlocal,tlocal),toShelfCoords))



# set up some items and some potential grasps
# instantiate "tall_item" as an ItemInfo class
# shelf = ItemInfo('SHELF_BOUNDING_BOX')
# shelf.mass = 10
# # shelf.bmin = [-0.435,0,-1.0]
# # shelf.bmax = [0.435,0.01,1.0]
# shelf.bmin = [-0.635,0,-1.0]
# shelf.bmax = [0.635,0.01,1.0]

# shelf.geometryFile = 'box'
# shelf.grasps.append(ItemGrasp((Zto_Y,[0,0,0])))


tall_item = ItemInfo('tall_item')
tall_item.bmin = [-0.03,-0.05,-0.08]
tall_item.bmax = [0.03,0.05,0.08]
tall_item.geometryFile = 'box'

# append these ItemGrasp objects to tall_item.grasps list
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
med_item.grasps.append(ItemGrasp((so3.mul(XtoY,ZtoY),[0,0,0])))

# gripper local rotational axis is x, perturb +/- along that axis
for g in med_item.grasps[:]:
    angle = 15.0/180.0*math.pi
    med_item.grasps.append(ItemGrasp((so3.mul(g.grasp_xform[0],so3.from_axis_angle(([1,0,0],angle))),g.grasp_xform[1])))
    med_item.grasps.append(ItemGrasp((so3.mul(g.grasp_xform[0],so3.from_axis_angle(([1,0,0],-angle))),g.grasp_xform[1])))
    angle = 30.0/180.0*math.pi
    med_item.grasps.append(ItemGrasp((so3.mul(g.grasp_xform[0],so3.from_axis_angle(([1,0,0],angle))),g.grasp_xform[1])))
    med_item.grasps.append(ItemGrasp((so3.mul(g.grasp_xform[0],so3.from_axis_angle(([1,0,0],-angle))),g.grasp_xform[1])))
    angle = 45.0/180.0*math.pi
    med_item.grasps.append(ItemGrasp((so3.mul(g.grasp_xform[0],so3.from_axis_angle(([1,0,0],-angle))),g.grasp_xform[1])))
# flip 180 about z axis
for g in med_item.grasps[:]:
    med_item.grasps.append(ItemGrasp((so3.mul(g.grasp_xform[0],XYflip),g.grasp_xform[1])))

# a list of possible items
item_info = [tall_item, small_item, med_item]
