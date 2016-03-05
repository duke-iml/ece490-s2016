import sys
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2
from util.constants import *
from scipy.spatial import KDTree, cKDTree

class PCProcessor:
    def __init__(self):
        pass

    # Subtracts points probably belonging to the shelf
    # and returns the new cloud
    def subtractShelf(self, data):
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
        dobj = np.array([xs,ys,zs])
        dobj = dobj.transpose()
        dobj = dobj[::STEP]

        # Load shelf data
        data_shelf = np.load(SHELF_NPZ_FILE)
        dshel = np.mat([data_shelf['arr_0'],data_shelf['arr_1'],data_shelf['arr_2']])
        dshel = dshel.transpose()

        # Construct kd-tree and remove nearest neighbors
        t = cKDTree(dobj)
        d, idx = t.query(dshel, k=20, eps=0, p=2, distance_upper_bound=0.1)
        print 'finish kdtree'
        Points_To_Delete = []
        for i in range(0,len(idx)):
           for j in range(0,19):
                if d[i,j] <= 0.03:
                   Points_To_Delete.append(idx[i,j])
        print len(dobj)            #dobj[idx[i,j],:]  = [0,0,0]
        dobj = np.delete(dobj,Points_To_Delete,axis = 0)
            
        print len(dobj)
        print 'finish NN removal'
        dobj = dobj.transpose()

        # Show a plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(dobj[0,:], dobj[1,:], dobj[2,:])
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        plt.show()

        return dobj
