#!/usr/bin/env python

from klampt import so3
import json
from math import pi
import numpy

bbbs = {}

rows = [ (0.4315, 0.1463), (0.1413, -0.1414), (-0.1464, -0.4315) ]
cols = [ (1.78, 1.588), (1.55, 1.368), (1.33, 1.139), (1.10, 0.873) ]

for b in [ 'bin_'+c for c in 'ABCDEFGHIJKL' ]:    
    if b[4] in 'ADGJ':
        x = rows[0]
    elif b[4] in 'BEHK':
        x = rows[1]
    elif b[4] in 'CFIL':
        x = rows[2]
    
    y = [ -0.1315, 0.1391 ]

    if b[4] in 'ABC':
        z = cols[0]
    elif b[4] in 'DEF':
        z = cols[1]
    elif b[4] in 'GHI':
        z = cols[2]
    elif b[4] in 'JKL':
        z = cols[3]

    bbbs[b] = zip(x[::-1], y, z[::-1])
    print b, bbbs[b]

json.dump(bbbs, open('shelf_dims.json', 'w'), indent=4)
json.dump(bbbs, open('bin_bounds.json', 'w'), indent=4)
