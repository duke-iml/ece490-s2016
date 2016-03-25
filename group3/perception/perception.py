import sys
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2
from util.constants import *
from scipy.spatial import KDTree, cKDTree

def isCloudValid(data):
    """
    Returns whether a cloud is valid to be processed by this perception module
    """
    for point in pc2.read_points(cloud, skip_nans=True):
        return True
        break
    return False

def getObjectCOM(data):
    """
    Subtracts points probably belonging to the shelf and returns the new cloud
    """
    STEP = 1 # Plot every STEPth point for speed, set to 1 to plot all

    # Load cloud data
    xs = []
    ys = []
    zs = []
    for point in pc2.read_points(data, skip_nans=True):
        xs.append(point[0])
        ys.append(point[1])
        zs.append(point[2])
    print "Point cloud count: " + str(len(xs))
    cloud = np.array([xs,ys,zs])
    cloud = cloud.transpose()
    cloud = cloud[::STEP]

    # Load shelf data
    data_shelf = np.load(SHELF_NPZ_FILE)
    dshel = np.mat([data_shelf['arr_0'],data_shelf['arr_1'],data_shelf['arr_2']])
    dshel = dshel.transpose()

    # Construct kd-tree and remove nearest neighbors
    t = cKDTree(cloud)
    d, idx = t.query(dshel, k=20, eps=0, p=2, distance_upper_bound=0.1)
    print 'finish kdtree'
    Points_To_Delete = []
    for i in range(0,len(idx)):
       for j in range(0,19):
            if d[i,j] <= 0.1:
               Points_To_Delete.append(idx[i,j])
    print len(cloud)            #cloud[idx[i,j],:]  = [0,0,0]
    cloud = np.delete(cloud,Points_To_Delete,axis = 0)
    com = np.mean(cloud,axis = 0)
    print len(cloud)
    print 'finish NN removal'
    cloud = cloud.transpose()

    # Show a plot
    if SHOW_PLOT:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(cloud[0,:], cloud[1,:], cloud[2,:])
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        plt.show()

    return (cloud.transpose(), com)
