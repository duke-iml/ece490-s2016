#!/usr/bin/env python

from klampt import so3
import json
from math import pi

vps = {}
            
for b in [ 'bin_'+c for c in 'ABCDEFGHIJKL' ]:
    z = 0.2
    y = 0
    downtilt = 0

    if b[4] in 'ADGJHK':
        #z -= 0.02
        pass
    else:
        downtilt = 3
    if b[4] in 'ADGJ':
        x = 0.3
        downtilt = 0.0
    elif b[4] in 'BEHK':
        x = 0
        downtilt = 0.0
    elif b[4] in 'CFIL':
        x = -0.3
    #if b[4] == 'H':
    #    y += 0.05

    y += 0.4

    if b[4] in 'ABC':
        z += 1.50
    elif b[4] in 'DEF':
        #z += 1.50 - 6*0.0254
        z += 1.50 - 9*0.0254
    elif b[4] in 'GHI':
        z += 1.50 - 18*0.0254
    elif b[4] in 'JKL':
        z += 1.50 - 29*0.0254

    rdowntilt = downtilt*pi / 180
    rhighdowntilt = (downtilt+10)*pi/180.0
    vps[b+'_center'] = ( so3.mul(so3.from_axis_angle(([-1,0,0],-pi/2-rdowntilt)), 
            so3.from_axis_angle(([0,0,1],pi))), 
            [ x, y, z ])
    vps[b+'_high'] = ( so3.mul(so3.from_axis_angle(([-1,0,0],-pi/2-rhighdowntilt)), 
            so3.from_axis_angle(([0,0,1],pi))), 
            [ x, y, z + 0.08 ])

    print b, vps[b+'_center'][1], vps[b+'_high'][1]

json.dump(vps, open('vantage_point_xforms.json', 'w'), indent=4)

