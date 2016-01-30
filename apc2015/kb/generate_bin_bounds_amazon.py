#!/usr/bin/env python

from klampt import so3
import json
from math import pi
import numpy

bbbs = {
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

for (k, bs) in bbbs.items():
    bs = [ so3.apply(so3.mul(so3.rotation([0,0,1], 3.141592), so3.rotation([1,0,0], 3.141592/2)), b) for b in bs ]
    bmin = [ min(*x) for x in zip(*bs) ]
    bmax = [ max(*x) for x in zip(*bs) ]

    bbbs[k] = (bmin, bmax)
    print k, bbbs[k]

json.dump(bbbs, open('kb/shelf_dims.json', 'w'), indent=4)
json.dump(bbbs, open('kb/bin_bounds.json', 'w'), indent=4)
