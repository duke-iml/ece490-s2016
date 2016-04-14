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
    for point in pc2.read_points(cloud,skip_nans=False,field_names=("rgb","x","y","z")):
        return True
        break
    return False

def calPointCloud(cloud):
    """
    Not currently used

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

def convertPc2ToNp(data):
    """
    Input: PointCloud2 message
    Output: NumPy array of point cloud [x,y,z,index]
    """
    STEP = 1 # Plot every STEPth point for speed, set to 1 to plot all

    # Load cloud data
    xs = []
    ys = []
    zs = []
    idx = []
    r = []
    g = []
    b = []
    idxnum = 0
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
        # a = (pack & 0xFF000000)>> 24
        r1 = (pack & 0x00FF0000)>> 16
        g1 = (pack & 0x0000FF00)>> 8
        b1 = (pack & 0x000000FF)
        r.append(r1)
        g.append(g1)
        b.append(b1)
        idx.append(idxnum)
        idxnum = idxnum+1
    print "Point cloud count: " + str(len(xs))
    # print r,g,b,a
    cloud = np.array([xs,ys,zs,idx,r,g,b])
    cloud = cloud.transpose()
    nans = np.isnan(cloud)
    cloud[nans] = 0
    print "Original cloud in convertPc2ToNp"
    print cloud
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
    d, idx = t.query(dshel, k=20, eps=0, p=2, distance_upper_bound=10)
    Points_To_Delete = []
    for i in range(0,len(idx)):
       for j in range(0,19):
            if d[i,j] <= 0.05:
               Points_To_Delete.append(idx[i,j])
    print ("cloud length before subtraction: ", len(cloud))
    cloud = np.delete(cloud,Points_To_Delete,axis = 0)
    print ("cloud length after subtraction: ", len(cloud))
    cloud = cloud[cloud[:,0] != 0 , :]
    return cloud



def segmentation(cloud):
    """
    Input: Numpy array of point cloud with index as the forth number
    Output: List of planes (Np arrays) with index as the forth number
    object is cloud without index now
    """
    print 'start segmentation'
    STEP = 15
    o = 1 # how many points are searched
    oo = 1 # number of iterations done for each plane
    ooo = 1 # how many planes found
    cloudcopy = copy.copy(cloud[:,0:3]) # delete the fourth column (index column)
    tree = cKDTree(cloudcopy)
    dist1,idx1 = tree.query(cloudcopy,k=20, eps=0, p=2, distance_upper_bound=5)
    # print idx1
    print len(cloudcopy)
    dmin = len(cloudcopy)/2
    print "dmin"
    print dmin
    point_list = np.array([dmin])#index number of points in each iteration
    sellist = np.array([dmin])#selected points for one area
    edthall = np.median(dist1)
    print "edthall is "
    print edthall
    leftpoint = np.ones(len(cloudcopy))
    listall = [point_list]
    show = 1
    while max(leftpoint) > 0 and ooo <= 10:
        print("segment ",ooo)
        r = np.zeros((1,4))
        while len(point_list) >0 and oo<=5000:
            print "segmentation loop..."
            number = len(point_list)
            print("length of point list", number)
            for k in range(0,number):
                dmin = point_list[0]
                edth = np.median(dist1[dmin-1])
                n,v,p = np.linalg.svd(cloudcopy[idx1[dmin-1,:],:])
                p = p.transpose()
                dminmean = np.mean(cloudcopy[idx1[dmin-1,:],:], axis=0)
                odth1 = np.zeros((20,1))
                for i in range(0,20):
                    odth1[i] = abs(np.dot((cloudcopy[idx1[dmin-1,i],:] - dminmean),p[:,2])) 
                odth = np.median(odth1)
                pointss = np.zeros((20,4))
                pointnum = np.zeros(20)
                pointnum1 = np.zeros(20)
                for i in range(0,20): 
                    if (dist1[dmin-1,i]<=min(0.003,abs(edthall)) and dist1[dmin-1,i] <= abs(edth)and np.dot((cloudcopy[idx1[dmin-1,i],:] - dminmean),p[:,2])<odth/3):
                        pointss [i,:] = cloud[idx1[dmin-1,i],:]
                        pointnum[i] = idx1[dmin-1,i]
                        leftpoint[idx1[dmin-1,i]] = 0
                        cloudcopy[idx1[dmin-1,i],0:3] = 0
                pointss = pointss[pointss[:,1]!=0,:]
                r = np.append(r,pointss,axis =0)
                pointnum1 = pointnum
                mask = (pointnum1 !=0)
                pointnum1 = pointnum1[mask]
                for i in range(0,len(pointnum1)):
                    if len((point_list == pointnum1[i]).ravel().nonzero()[0]):
                        curr_number = pointnum1[i]
                        pointnum1[i] =0
                        if len((sellist == curr_number).ravel().nonzero()[0]):
                            point_list = point_list[(point_list != curr_number)]
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
                if not len((sellist == dmin).ravel().nonzero()[0]):
                    sellist = np.append(sellist,[dmin],axis =0)
                o = o+1
                if len(point_list) == 0:
                    print "break"
                    break# end of for loop
            if len(point_list) == 0:
                print "break"
                break                  
            else:
                oo = oo+1
                listall.append(point_list)# end of inside while loop
        if len(r) > 50:
            print np.size(r,axis= 0)
            print np.size(r,axis= 1)
            a = 'plane'+str(ooo)
            np.savez(a, r[:,0],r[:,1],r[:,2])
        o =1
        oo =1
        #clear these three in case
        pointss = np.zeros((20,4))
        pointnum = np.zeros(20)
        pointnum1 = np.zeros(20)
        cloudcopy = cloudcopy[cloudcopy[:,2]!=0]
        tree = cKDTree(cloudcopy)
        dist1,idx1 = tree.query(cloudcopy,k=20, eps=0, p=2, distance_upper_bound=5)
        leftpoint = np.ones(len(cloudcopy))
        dmin = len(cloudcopy)/2
        point_list = np.array([dmin])
        sellist = np.array([dmin])
        edthall = np.median(dist1)
        print "edthall is "
        print edthall
        listall = [point_list]# why this???
        ooo = ooo+1
        r = r[r[:,1]!=0 , :]
        print len(r)
        print ("plane from segmentation: ")
    return r

def com(cloud):
    return np.mean(cloud,axis = 0)
