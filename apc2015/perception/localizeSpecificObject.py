#!/usr/bin/env python

import numpy, sys, os
from subprocess import call
import os.path
import subprocess
import uuid

if False:
    T = numpy.eye(4)

    from klampt import so3

    if sys.argv[2] == 'crayola_64_ct':
        T[:3,:3] = numpy.array(
            so3.mul(
                so3.rotation([1,0,0], 3.1415),
                so3.rotation([0,0,1], 3.1415/4+3.1415)
            )
        ).reshape((3,3))
        T[:3,3] = (0.995, 0.27, 0.84)
    elif sys.argv[2] == 'kong_duck_dog_toy':
        T[:3,:3] = numpy.array(
            # so3.mul(
                # so3.rotation([1,0,0], 3.1415),
                so3.rotation([0,0,1], -3.1415/4)
            # )
        ).reshape((3,3))
        T[:3,3] = (0.94, -0.05, 0.70)
    else:
        raise SystemExit

    # print xform
    sys.stdout.write(' '.join(map(str, T.flat)) + '\n')
    sys.stdout.flush()

    # print point cloud
    path = os.path.join(os.path.dirname(__file__), 'demo.pcd')
    sys.stdout.write(open(path, 'rb').read())
    raise SystemExit

if "-v" in sys.argv:
    pr = True
else:
    pr = False


"================================Begin actual code==================================="

serial = str(uuid.uuid4())
# those files are to be created
point_cloud_file = 'perception/single_utils/point_cloud'+serial+'.pcd'
object_file = 'perception/single_utils/object_only'+serial+'.pcd'
aligned_file = 'perception/single_utils/aligned'+serial+'.pcd'
xform_matrix = 'perception/single_utils/xform_matrix'+serial+'.txt'

def ply_file_name(obj):
    return 'klampt_models/items/' + obj + '/textured_meshes/optimized_tsdf_textured_mesh.ply'

if pr:
    print "calling generate_naive_cloud.py"
call(['python', 'perception/single_utils/src/generate_naive_cloud.py', point_cloud_file])
if not os.path.isfile(point_cloud_file):
    print "Error point_cloud!"
    quit()

if pr:
    print "calling remove_all_sides.py"
call(['python', 'perception/single_utils/src/remove_all_sides.py', point_cloud_file, object_file, '-silence'])

if not os.path.isfile(object_file):
    print "Error object segmented cloud!"
    quit()

if pr:
    print "calling ICP"
call(['perception/single_utils/bin/ICP', ply_file_name(sys.argv[2]), object_file, aligned_file, xform_matrix])

if not os.path.isfile(xform_matrix):
    print "Error transformation matrix file!"
    quit()
matrix = open(xform_matrix)
OK = True
i = 0
for l in matrix:
    if l.strip()=='':
        continue
    i += 1
    try:
        map(float, l.strip().split())
        assert len(l.strip().split())==4
    except:
        OK = False

if i!=4:
    OK = False
    
if not OK:
    print "Error parsing matrix!"
    quit()

matrix = open(xform_matrix)
for l in matrix:
    if l.strip()=='':
        continue
    sys.stdout.write(l.strip()+' ')
sys.stdout.write('\n')
sys.stdout.flush()
 
f = open(object_file)
for l in f:
    sys.stdout.write(l)
sys.stdout.flush()
