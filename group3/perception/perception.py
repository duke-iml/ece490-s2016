import sys
sys.path.insert(0, "..")

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2
from util.constants import *
from scipy.spatial import KDTree, cKDTree

def isCloudValid(cloud):
    """
    Returns whether a cloud is valid to be processed by this perception module
    """
    for point in pc2.read_points(cloud, skip_nans=True):
        return True
        break
    return False


def calPointCloud(cloud):
    """
    Input: Point cloud
    Output: (Calibrated cloud, COM that was subtracted)
    Calculates the COM of the two shelf edges and
    subtracts their COM from every point in the cloud
    """
    pointsortt = cloud[cloud[:,2].argsort(),]
    pointth = min(np.mean(pointsortt[1:500,2])*2,max(pointsortt[1:500,2]))
    pointsort = np.array([pointsortt[1,:]])
    begin = 0
    while pointsortt[begin,2] <= pointth:
        pointsort = np.append(pointsort,[pointsortt[begin,:]],axis =0)
        begin = begin+1
    pointmean = np.mean(pointsort,axis = 0)
    print ("COM of shelf edges: ", pointmean)
    cloud = cloud - pointmean
    return (cloud, pointmean)

def convertPc2ToNp(data):
    """
    Input: PointCloud2 message
    Output: NumPy array of point cloud
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

    return cloud

def subtractShelf(cloud):
    """
    Input: Numpy array of points
    Output: Numpy array of points without shelf points
    """
    # Load shelf data
    data_shelf = np.load(SHELF_NPZ_FILE)
    dshel = np.mat([data_shelf['arr_0'],data_shelf['arr_1'],data_shelf['arr_2']])
    dshel = dshel.transpose()

    # Construct kd-tree and remove nearest neighbors
    t = cKDTree(cloud)
    d, idx = t.query(dshel, k=20, eps=0, p=2, distance_upper_bound=0.1)
    Points_To_Delete = []
    for i in range(0,len(idx)):
       for j in range(0,19):
            if d[i,j] <= 0.1:
               Points_To_Delete.append(idx[i,j])
    print ("cloud length before subtraction: ", len(cloud))
    cloud = np.delete(cloud,Points_To_Delete,axis = 0)
    print ("cloud length after subtraction: ", len(cloud))
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

    return cloud.transpose()

def segmentation(object):
    """
    Input: Numpy array of point cloud
    Output: List of planes (Np arrays)
    """
    print 'start segmentation'
    STEP = 50
    o = 1
    oo = 1
    ooo = 1
    tree = cKDTree(object)
    dist1,idx1 = tree.query(object,k=20, eps=0, p=2, distance_upper_bound=5)
    dmin = round(0.5*len(object))
    point_list = np.array([dmin])#index number of points in each iteration
    sellist = np.array([dmin])#selected points for one area
    edthall = np.median(dist1)
    leftpoint = np.ones(len(object))
    listall = [point_list]
    show = 1
    while max(leftpoint) > 0 and ooo <= 1:
        r = np.zeros((1,3))
        while len(point_list) >0 and oo<=5000:
            print "segmentation loop..."
            number = len(point_list)
            for k in range(0,number):
                dmin = point_list[0]
                edth = np.median(dist1[dmin-1])
                n,v,p = np.linalg.svd(object[idx1[dmin-1,:],:])
                dminmean = np.mean(object[idx1[dmin-1,:],:], axis=0)
                odth1 = np.zeros((20,1))
                for i in range(0,20):
                    odth1[i] = abs(np.dot((object[idx1[dmin-1,i],:] - dminmean),p[:,2])) 
                odth = np.median(odth1)
                pointss = np.zeros((20,3))
                pointnum = np.zeros(20)
                for i in range(0,20): 
                    if dist1[dmin-1,i]<=min(edth,edthall) and np.dot((object[idx1[dmin-1,i],:] - dminmean),p[:,2])<odth:
                        pointss [i,:] = object[idx1[dmin-1,i],:]
                        pointnum[i] = idx1[dmin-1,i]
                        leftpoint[idx1[dmin-1,i]] = 0
                        object[idx1[dmin-1,i],0:3] = 0
                r = np.append(r,pointss,axis =0)
                pointnum1 = pointnum
                mask = (pointnum1 !=0)
                pointnum1 = pointnum1[mask]
                for i in range(0,len(pointnum1)):
                    if len((point_list == pointnum1[i]).ravel().nonzero()[0]):
                        if len((sellist == pointnum1[i]).ravel().nonzero()[0]):
                            point_list = point_list[(point_list != pointnum1[i])]
                            pointnum1[i] = 0
                    if len((sellist == pointnum1[i]).ravel().nonzero()[0]):
                        point_list = point_list[(point_list != pointnum1[i])]
                        pointnum1[i] = 0
                    if pointnum1[0] == dmin:
                        pointnum1[0] = 0
                pointnum1 = pointnum1[(pointnum1 != 0)]
                point_list =np.append(point_list,pointnum1,axis =0)
                point_list = point_list[1:(len(point_list))]
                point_list = point_list[(point_list != 0)]
                sellist = np.append(sellist,[dmin],axis =0)
                o = o+1
                if len(point_list) == 0:
                    break
            if len(point_list) == 0:
                break
            else:
                oo = oo+1
                listall.append(point_list)
        if len(r) > 1:
            print np.size(r,axis= 0)
            print np.size(r,axis= 1)
            np.savez('plane1', r[:,0],r[:,1],r[:,2])
            show = 0
        o =1
        oo =1
        object = object[object[:,2]!=0]
        tree = cKDTree(object)
        dist1,idx1 = tree.query(object,k=20, eps=0, p=2, distance_upper_bound=1)
        leftpoint = np.ones(len(object))
        dmin = round(0.5*len(object))
        point_list = np.array([dmin])
        sellist = np.array([dmin])
        edthall = np.median(dist1)
        listall = [point_list]# why this???
        ooo = ooo+1

    print ("plane from segmentation: ", r)
    return r

def com(cloud):
    return np.mean(cloud,axis = 0)
