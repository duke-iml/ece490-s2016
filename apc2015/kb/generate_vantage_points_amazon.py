#!/usr/bin/env python

from klampt import so3
import json
from math import pi

bbs = json.load(open('kb/bin_bounds.json'))
vps = {}

for (b, (bmin, bmax)) in bbs.items():
    (x, y, z) = [ (l + u)/2.0 for (l, u) in zip(bmin, bmax) ]

    y = bmax[1] + 0.20

    downtilt = 0

    if b[4] in 'ADGJHK':
        pass
    else:
        downtilt = 3

    rdowntilt = downtilt*pi / 180
    rhighdowntilt = (downtilt+10)*pi/180.0
    rleftturn = pi/10
    vps[b+'_center'] = ( so3.mul(so3.from_axis_angle(([-1,0,0],-pi/2-rdowntilt)),
            so3.from_axis_angle(([0,0,1],pi))),
            [ x, y, z ])
    vps[b+'_high'] = ( so3.mul(so3.from_axis_angle(([-1,0,0],-pi/2-rhighdowntilt)),
            so3.from_axis_angle(([0,0,1],pi))),
            [ x, y, z + 0.08 ])
    vps[b+'_left'] = (
        so3.mul(so3.rotation([0,0,1], -rleftturn), vps[b+'_center'][0]),
        [ x + 0.08, y, z ]
    )
    vps[b+'_right'] = (
        so3.mul(so3.rotation([0,0,1], rleftturn), vps[b+'_center'][0]),
        [ x - 0.08, y, z ]
    )

    print b, vps[b+'_center'][1], vps[b+'_high'][1]

json.dump(vps, open('kb/vantage_point_xforms.json', 'w'), indent=4)

