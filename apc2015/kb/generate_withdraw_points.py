#!/usr/bin/env python

from klampt import so3
import json
from math import pi

bins = json.load(open('bin_vantage_points.json')).keys()
bbs = json.load(open('bin_bounds.json'))
wps = {}

for b in bins:
    #center_vp = vps[b + '_center']
    bbc = [ (x + y)/2.0 for (x, y) in zip(*bbs[b]) ]
    
    # for the left and center bins using the left hand
    #if b[4] in 'ABDEGHJK':
        #wps[b] = (
        #    so3.mul(
        #        so3.rotation([0,0,1], -pi/4-pi/2),
        #        so3.rotation([0,1,0], pi/2)
        #    ),
        #    [ 0.50, 0.50, center_vp[1][2] ]
        #)
     
    # for the right bins using the right hand
    #elif b[4] in 'CFIL':
        #wps[b] = (
        #    so3.mul(
        #        so3.rotation([0,0,1], -pi/4),
        #        so3.rotation([0,1,0], pi/2)
        #    ),
        #    [ -0.50, 0.50, center_vp[1][2] ]
        #)
        
    #else:
    #    print 'skipping invalid bin', b
    #    continue

    wps[b] = (
        so3.mul(
            so3.rotation([0,0,1],-pi/2),
            so3.rotation([0,1,0], pi/2)
        ),
        [ bbc[0], bbs[b][1][1] + 0.25, bbc[2] ]
    )

    print b, wps[b][1]

json.dump(wps, open('withdraw_point_xforms.json', 'w'), indent=4)

