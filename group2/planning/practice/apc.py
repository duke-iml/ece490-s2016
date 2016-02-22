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
    'bin_J' : ([-0.41,0.81,0],[-0.158,1.06,0.42]),
    'bin_K' : ([-0.149,0.81,0],[0.149,1.06,0.42]),
    'bin_L' : ([0.158,0.81,0],[0.41,1.06,0.42]),
    }


# set up some items and some potential grasps
tall_item = ItemInfo('tall_item')
tall_item.bmin = [-0.03,-0.05,-0.1]
tall_item.bmax = [0.03,0.05,0.1]
tall_item.geometryFile = 'box'

# rotation matrices
XtoY = so3.from_axis_angle(([0,0,1],math.pi*0.5))
Xto_Y = so3.from_axis_angle(([0,0,1],-math.pi*0.5))
XtoZ = so3.from_axis_angle(([0,1,0],math.pi*0.5))
Xto_Z = so3.from_axis_angle(([0,1,0],-math.pi*0.5))
ZtoY = so3.from_axis_angle(([1,0,0],math.pi*0.5))
Zto_Y = so3.from_axis_angle(([1,0,0],-math.pi*0.5))
XYflip = so3.from_axis_angle(([0,0,1],math.pi))
XZflip = so3.from_axis_angle(([0,1,0],math.pi))
YZflip = so3.from_axis_angle(([1,0,0],math.pi))
