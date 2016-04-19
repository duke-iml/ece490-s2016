
import sys
# sys.path.insert(0, "..")
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial import KDTree, cKDTree
import perception
import color
import copy
import json
import scipy.io as sio
# from util.constants import *
REPO_ROOT = "/home/group3/ece490-s2016"
CLOUD_MAT_PATH = REPO_ROOT + "/group3/perception/matpcl/cloud.mat"
CHENYU_GO_PATH = REPO_ROOT + "/group3/perception/matpcl/chenyugo.txt"
CHENYU_DONE_PATH = REPO_ROOT + "/group3/perception/matpcl/chenyudone.txt"


ROS_DEPTH_TOPIC = "/camera/depth/points"
ROS_DEPTH_TOPIC = "/realsense/pc"
# ROS_DEPTH_TOPIC = "/cloud_pcd"

if len(sys.argv) >= 2:
    SAVE_LOCATION = sys.argv[1]
else:
    print "ERROR: You must supply a path (no need for .npz extension) to save to"
    print "Usage: python save_calibrated_shelf.py myfile"
    sys.exit(0)

rospy.init_node("listener", anonymous=True)
STEP = 15
cloud = rospy.wait_for_message(ROS_DEPTH_TOPIC, PointCloud2)
if perception.isCloudValid(cloud):
    cloud = perception.convertPc2ToNp(cloud)
    print cloud[:,3]

    np_cloud = cloud[::STEP]
    # json_str = json.dumps(np_cloud)
    # data = json.loads(json_str)
    # with open('data.json','w') as f:
    #     json.dumps(data,f)
    # np_cloud, pointmean = perception.calPointCloud(np_cloud)
    np_cloud = perception.subtractShelf(np_cloud)
    np_cloud =  np_cloud[np_cloud[:,0] != 0 , :]
    np.savez('sub', np_cloud[:,0],np_cloud[:,1],np_cloud[:,2])
    np_cloud = np_cloud[np_cloud[:,0] != 0 , :]

    sio.savemat(CLOUD_MAT_PATH, {'cloud':np_cloud})
    fo = open(CHENYU_GO_PATH, "w")
    fo.write("chenyu go")
    fo.close()
    # cloud = cloud[cloud[:,5] != 255,:]
    final_cloud = perception.resample(cloud,np_cloud,3)
    # final_cloud = np_cloud

    # print len(cloud)
    # print cloud
    # color_idx = np_cloud[:,3]
    # for i in range(1,7):
    #     color_idx = np.append(color_idx, np_cloud[:,3]+i, axis = 0)
    #     color_idx = np.append(color_idx, np_cloud[:,3]-i, axis = 0)   
    # print color_idx 
    # color_idx =  color_idx[color_idx[:] < 307200]
    # print len(color_idx)
    # # mask = np.zeros((307200,1))
    # # for i in range(0,len(color_idx)):
    # #     if color_idx[i] > 0:
    # #         mask[color_idx[i]] = 1
    # final_cloud = cloud[color_idx.astype(int)]
    # print final_cloud
    # print len(final_cloud)


    np.savez('test', final_cloud[:,0],final_cloud[:,1],final_cloud[:,2])







    # np.savez('object', cloud[:,0],cloud[:,1],cloud[:,2])
    # y,u,v = color.rgb_to_yuv(final_cloud[:,4],final_cloud[:,5],final_cloud[:,6])
    # print 'y', y
    # print 'u', u
    # print 'v', v
    uv = [color.rgb_to_yuv(*rgb)[1:3] for rgb in final_cloud[:,4:7]]
    # print uv
    # # uv = [u,v]
    # print uv
    hist = color.make_uv_hist(uv)
    # print hist
    # a = hist.sum()
    # print a
    # old_hist = np.load('hist.npz')
    # old_hist = old_hist['arr_0']
    # # print old_hist.sum()
    np.savez(SAVE_LOCATION, hist)
    # # # print old_hist
    # score = 2*np.minimum(hist,old_hist).sum()-np.maximum(hist,old_hist).sum()
    # print score
    # # print len(cloud)
    # # plane1 = perception.segmentation(cloud)
    # # np.savez('subfinished', plane1[:,0],plane1[:,1],plane1[:,2])
    # # plane = perception.segmentation(np_cloud)
    # print'finish sub '
    # # print plane