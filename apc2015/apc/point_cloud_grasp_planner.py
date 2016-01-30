import apc
from klampt import vectorops,so3,se3
from klampt import *
from klampt import PointCloud
from klampt import resource
import os
import sys
import math
if '../klampt_models' not in sys.path:
    sys.path.append('../klampt_models')
import reflex

gripper = "reflex"
#maximum gripper depth (estimated at 10cm?)
depthmax = 0.10
#maximum gripper width for power grasp (estimated at 15cm?)
powerwidthmax = 0.15
#maximum gripper width for pinch grasp (estimated at 15cm?)
pinchwidthmax = 0.15

#translation from saved grasp center point to fingertip
center_to_finger = 0.05

def show(world,desc=""):
    resource.edit(None,value=world.robot(0).getConfig(),type='Config',description=desc,world=world)

def power_grasp_medial_axis(pc,bin,object,knowledgeBase):
    """Returns a power grasp for the Reflex gripper to grasp the medial axis
    of the  point cloud given in the Baxter's frame (i.e., x is forward, z
    is up).

    Input:
        - pc: a klampt.PointCloud object
        - bin: a bin name
        - object: an object name
        - knowledgeBase: an apc.KnowledgeBase object
        
    Returns a pair (g,s):
        - g: an apc.ItemGrasp member defining the grasp.  Its xform member is
          given in the Baxter's local frame.
        - s: a confidence score, with 0 = unconfident, 1 = confident
    """
    global gripper,depthmax,powerwidthmax
    points = []
    for i in xrange(0,len(pc.vertices),3):
        points.append([pc.vertices[i],pc.vertices[i+1],pc.vertices[i+2]])
    mean = [0,0,0]
    for p in points:
        mean = vectorops.add(mean,p)
    mean = vectorops.mul(mean,1.0/len(points))
    bmin,bmax = points[0][:],points[0][:]
    for p in points:
        if p[0] < bmin[0]: bmin[0]=p[0]
        elif p[0] > bmax[0]: bmax[0]=p[0]
        if p[1] < bmin[1]: bmin[1]=p[1]
        elif p[1] > bmax[1]: bmax[1]=p[1]
        if p[2] < bmin[2]: bmin[2]=p[2]
        elif p[2] > bmax[2]: bmax[2]=p[2]
    median = vectorops.mul(vectorops.add(bmin,bmax),0.5)
    width = bmax[1]-bmin[1]
    depth = bmax[0]-bmin[0]
    height = bmax[2]-bmin[2]

    #determine an adequate center
    graspcenter = median[:]
    if depth*0.5 > depthmax - center_to_finger:
        graspcenter[0] = bmin[0] + depthmax - center_to_finger
    
    graspfile = os.path.join("../resources",gripper,object,'default_power_grasp.json')
    try:
        f = open(graspfile,'r')
        contents = ''.join(f.readlines())
        g = apc.ItemGrasp()
        g.readFromJson(contents)
        #translate to center of point cloud
        g.grasp_xform = (g.grasp_xform[0],vectorops.add(g.grasp_xform[1],graspcenter))
        return (g,1)
    except IOError:
        print "No default grasp defined for object",object,"trying default"
        graspfile = os.path.join("../resources",gripper,'default_power_grasp.json')
        try:
            f = open(graspfile,'r')
            contents = ''.join(f.readlines())
            g = apc.ItemGrasp()
            g.readFromJson(contents)
            #translate to center of point cloud
            g.grasp_xform = (g.grasp_xform[0],vectorops.add(g.grasp_xform[1],graspcenter))
            #adjust grasp to size of object
            ratio = width/powerwidthmax
            if ratio > 1.0:
                g.gripper_close_command[0] = g.gripper_close_command[1] = g.gripper_close_command[2] = 0.7
                g.gripper_open_command[0] = g.gripper_open_command[1] = g.gripper_open_command[2] = g.gripper_close_command[2] = 1
                return (g,0.1)
            else:
                #simple approximation: angular interpolation
                angle = math.asin(ratio)
                g.gripper_close_command[0] = g.gripper_close_command[1] = g.gripper_close_command[2] = 0.7*angle/(math.pi*0.5)
                g.gripper_open_command[0] = g.gripper_open_command[1] = g.gripper_open_command[2] = g.gripper_open_command[2] = g.gripper_close_command[0]+0.2
            return (g,0.5)
        except IOError:
            print "Default power grasp file",graspfile,"not found"
            return (None,0)

def pinch_grasp_medial_axis(pc,bin,object,knowledgeBase):
    """Returns a power grasp for the Reflex gripper to grasp the medial axis
    of the  point cloud given in the Baxter's frame (i.e., x is forward, z
    is up).

    Input:
        - pc: a klampt.PointCloud object
        - bin: a bin name
        - object: an object name
        - knowledgeBase: an apc.KnowledgeBase object
        
    Returns a pair (g,s):
        - g: an apc.ItemGrasp member defining the grasp.  Its xform member is
          given in the Baxter's local frame.
        - s: a confidence score, with 0 = unconfident, 1 = confident
    """
    global gripper,depthmax,widthmax
    points = []
    for i in xrange(0,len(pc.vertices),3):
        points.append([pc.vertices[i],pc.vertices[i+1],pc.vertices[i+2]])
    mean = [0,0,0]
    for p in points:
        mean = vectorops.add(mean,p)
    mean = vectorops.mul(mean,1.0/len(points))
    bmin,bmax = points[0][:],points[0][:]
    for p in points:
        if p[0] < bmin[0]: bmin[0]=p[0]
        elif p[0] > bmax[0]: bmax[0]=p[0]
        if p[1] < bmin[1]: bmin[1]=p[1]
        elif p[1] > bmax[1]: bmax[1]=p[1]
        if p[2] < bmin[2]: bmin[2]=p[2]
        elif p[2] > bmax[2]: bmax[2]=p[2]
    median = vectorops.mul(vectorops.add(bmin,bmax),0.5)
    width = bmax[1]-bmin[1]
    depth = bmax[0]-bmin[0]
    height = bmax[2]-bmin[2]

    #determine an adequate center
    graspcenter = median[:]
    if depth*0.5 > depthmax - center_to_finger:
        graspcenter[0] = bmin[0] + depthmax - center_to_finger
    
    graspfile = os.path.join("../resources",gripper,object,'default_pinch_grasp.json')
    try:
        f = open(graspfile,'r')
        contents = ''.join(f.readlines())
        g = apc.ItemGrasp()
        g.readFromJson(contents)
        #translate to center of point cloud
        g.grasp_xform = (g.grasp_xform[0],vectorops.add(g.grasp_xform[1],graspcenter))
        return (g,1)
    except IOError:
        print "No default grasp defined for object",object,"trying default"
        graspfile = os.path.join("../resources",gripper,'default_pinch_grasp.json')
        try:
            f = open(graspfile,'r')
            contents = ''.join(f.readlines())
            g = apc.ItemGrasp()
            g.readFromJson(contents)
            #translate to center of point cloud
            g.grasp_xform = (g.grasp_xform[0],vectorops.add(g.grasp_xform[1],graspcenter))
            #adjust pinch to size of object
            ratio = width/pinchwidthmax
            if ratio >= 1:
                g.gripper_close_command[0] = g.gripper_close_command[1] = 0.7
                g.gripper_open_command[0] = g.gripper_open_command[1] = 1.0
                return (g,0.1)
            else:
                #simple approximation: angle
                angle = math.asin(ratio)
                g.gripper_close_command[0] = g.gripper_close_command[1] = 0.7*angle/(math.pi*0.5)
                g.gripper_open_command[0] = g.gripper_open_command[1] = g.gripper_close_command[0] + 0.2
            return (g,0.5)
        except IOError:
            print "Default pinch grasp file",graspfile,"not found"
            return (None,0)

def point_cloud_grasps(pc,bin,object,knowledgeBase):
    """Returns a list of grasp for the Reflex gripper to grasp the point
    cloud given in the Baxter's frame (i.e., x is forward, z is up).

    Input:
        - pc: a klampt.PointCloud object
        - bin: a bin name
        - object: an object name
        - knowledgeBase: an apc.KnowledgeBase object
        
    Returns a list of pairs [(g1,s1),...,(gn,sn)]:
        - gi: an apc.ItemGrasp member defining the grasp.  Its xform member is
          given in the Baxter's local frame.
        - si: a confidence score, with 0 = unconfident, 1 = confident
    Return list is sorted in order of decreasing confidence.
    """
    grasps = []
    g1 = power_grasp_medial_axis(pc,bin,object,knowledgeBase)
    if g1[0] != None: grasps.append(g1)
    g2 = pinch_grasp_medial_axis(pc,bin,object,knowledgeBase)
    if g2[0] != None: grasps.append(g2)
    return sorted(grasps,key=lambda x:-x[1])

def self_test(fn,object=None):
    print "Loading",fn
    world = WorldModel()
    if not world.loadTerrain(fn):
        print "Unable to load point cloud file",fn
        return False

    #HACK camera frame to baxter frame transform?
    #z->x, y->-z, x->-y
    print "Performing virtual camera -> baxter frame transform hack..."
    world.terrain(0).geometry().transform(so3.from_matrix([[0,0,1],[-1,0,0],[0,-1,0]]),[0.6,0,1])
    
    world.loadElement(os.path.join("../klampt_models",reflex.klampt_model_name))
    show(world)
    geom = world.terrain(0).geometry()
    pc = None
    if geom.type() == 'PointCloud':
        pc = geom.getPointCloud()
    else:
        print "Loaded object",fn,"was not a PointCloud, type",geom.type()
        if geom.type() == 'TriangleMesh':
            print "   Extracting vertices..."
            pc = PointCloud()
            mesh = geom.getTriangleMesh()
            for i in xrange(len(mesh.vertices)):
                pc.vertices.append(mesh.vertices[i])
        else:
            print "Unable to process geometries of that type"
            return False
        
    knowledge = apc.KnowledgeBase()
    if object != None:
        grasps = point_cloud_grasps(pc,'bin_A',object,knowledge)
    else:
        grasps = point_cloud_grasps(pc,'bin_A','none',knowledge)
    if len(grasps) == 0:
        print "No grasps found..."
        return False
    print
    for i in xrange(len(grasps)):
        print "Showing grasp %d, confidence %f"%(i+1,grasps[i][1])
        g = grasps[i][0]
        print "   Closed hand finger values",g.gripper_close_command
        print "   Open hand finger values",g.gripper_open_command
        world.robot(0).setConfig(reflex.commandToConfig(g.gripper_close_command))
        world.robot(0).getLink(0).setParentTransform(*g.grasp_xform)
        show(world,"Grasp %d"%(i+1,))
    return True

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "USAGE: point_cloud_grasp_planner.py PCD_file [object name]"
        exit(0)
    obj = None
    if len(sys.argv) >= 3:
        obj = sys.argv[2]
    self_test(sys.argv[1],obj)
