import sys
sys.path.insert(0, "..")

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2
from util.constants import *
from scipy.spatial import KDTree, cKDTree
import copy
import ctypes
import struct

def isCloudValid(cloud):
    """
    Returns whether a cloud is valid to be processed by this perception module
    """
    for point in pc2.read_points(cloud, skip_nans=True):
        return True
        break
    return False

def isCloudValidtest(cloud):
    """
    Returns whether a cloud is valid to be processed by this perception module
    """
    for point in pc2.read_points(cloud,skip_nans=False,field_names=("rgb","x","y","z")):
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
    pointth = min(np.mean(pointsortt[0:500,2])*2,max(pointsortt[0:500,2]))
    pointsort = np.array([pointsortt[0,:]])
    begin = 0
    while pointsortt[begin,2] <= pointth:
        pointsort = np.append(pointsort,[pointsortt[begin,0:3]],axis =0)
        begin = begin+1
    pointmean = np.mean(pointsort,axis = 0)
    print ("COM of shelf edges: ", pointmean)
    cloud[:,0:3] = cloud[:,0:3] - pointmean
    return (cloud, pointmean)

def convertPc2ToNptest(data):
    """
    Input: PointCloud2 message
    Output: NumPy array of point cloud
    updata added index as the forth number
    """
    STEP = 15 # Plot every STEPth point for speed, set to 1 to plot all

    # Load cloud data
    xs = []
    ys = []
    zs = []
    idx = []
    # r = []
    # g = []
    # b = []
    i = 1
    for point in pc2.read_points(data,skip_nans=False,field_names=("rgb","x","y","z")):
        # print point
        xs.append(point[0])
        ys.append(point[1])
        zs.append(point[2])
        test = point[3]
        # print point[3]
        # cast float32 to int so that bitwise operations are possible
        s = struct.pack('>f' ,test)
        i = struct.unpack('>l',s)[0]
        # you can get back the float value by the inverse operations
        pack = ctypes.c_uint32(i).value
        a = (pack & 0xFF000000)>> 24
        r = (pack & 0x00FF0000)>> 16
        g = (pack & 0x0000FF00)>> 8
        b = (pack & 0x000000FF)
        idx.append(i)
        i = i+1
    print "Point cloud count: " + str(len(xs))
    print r,g,b,a
    cloud = np.array([xs,ys,zs,idx])
    cloud = cloud.transpose()
    # nans = np.isnan(cloud)
    # cloud[nans] = 0
    print cloud
    cloud = cloud[::STEP]
    return cloud

def convertPc2ToNp(data):
    """
    Input: PointCloud2 message
    Output: NumPy array of point cloud
    updata added index as the forth number
    """
    STEP = 15 # Plot every STEPth point for speed, set to 1 to plot all

    # Load cloud data
    xs = []
    ys = []
    zs = []
    idx = []
    i = 1
    for point in pc2.read_points(data, skip_nans=False):
        xs.append(point[0])
        ys.append(point[1])
        zs.append(point[2])
        idx.append(i)
        i = i+1
    print "Point cloud count: " + str(len(xs))
    cloud = np.array([xs,ys,zs,idx])
    cloud = cloud.transpose()
    nans = np.isnan(cloud)
    cloud[nans] = 0
    cloud = cloud[::STEP]
    return cloud

def subtractShelf(cloud):
    """
    Input: Numpy array of points with index as the forth number
    Output: Numpy array of points without shelf points
    """
    # Load shelf data
    data_shelf = np.load(SHELF_NPZ_FILE)
    dshel = np.mat([data_shelf['arr_0'],data_shelf['arr_1'],data_shelf['arr_2']])
    dshel = dshel.transpose()

    # Construct kd-tree and remove nearest neighbors
    pCloud = copy.copy(cloud[:,0:3])
    t = cKDTree(pCloud)
    d, idx = t.query(dshel, k=20, eps=0, p=2, distance_upper_bound=0.1)
    Points_To_Delete = []
    for i in range(0,len(idx)):
       for j in range(0,19):
            if d[i,j] <= 0.03:
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

def segmentation(cloud):
    """
    Input: Numpy array of point cloud with index as the forth number
    Output: List of planes (Np arrays)
    object is cloud without index now
    """
    print 'start segmentation'
    STEP = 50
    o = 1
    oo = 1
    ooo = 1
    object = copy.deepcopy(cloud[:,0:3])
    print cloud
    print object
    tree = cKDTree(object)
    dist1,idx1 = tree.query(object,k=20, eps=0, p=2, distance_upper_bound=5)
    dmin = round(0.5*len(object))
    point_list = np.array([dmin])#index number of points in each iteration
    sellist = np.array([dmin])#selected points for one area
    edthall = np.median(dist1)
    leftpoint = np.ones(len(object))
    listall = [point_list]
    show = 1
    while max(leftpoint) > 0 and ooo <= 3:
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


def segmentationtest(cloud):
    """
    Input: Numpy array of point cloud with index as the forth number
    Output: List of planes (Np arrays) with index as the forth number
    object is cloud without index now
    """
    print 'start segmentation'
    STEP = 50
    o = 1
    oo = 1
    ooo = 1
    object = copy.copy(cloud[:,0:3])
    tree = cKDTree(object)
    dist1,idx1 = tree.query(object,k=20, eps=0, p=2, distance_upper_bound=5)
    dmin = round(0.5*len(object))
    point_list = np.array([dmin])#index number of points in each iteration
    sellist = np.array([dmin])#selected points for one area
    edthall = np.median(dist1)
    leftpoint = np.ones(len(object))
    listall = [point_list]
    show = 1
    while max(leftpoint) > 0 and ooo <= 5:
        print("segment ",ooo)
        r = np.zeros((1,4))
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
                pointss = np.zeros((20,4))
                pointnum = np.zeros(20)
                for i in range(0,20): 
                    if dist1[dmin-1,i]<=min(edth,edthall) and np.dot((object[idx1[dmin-1,i],:] - dminmean),p[:,2])<odth:
                        pointss [i,:] = cloud[idx1[dmin-1,i],:]
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
                    print "break"
                    break
            if len(point_list) == 0:
                print "break"
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
        r = r[r[:,1]!=0 , :]
        print len(r)
        print ("plane from segmentation: ", r)
        return r

def com(cloud):
    return np.mean(cloud,axis = 0)
