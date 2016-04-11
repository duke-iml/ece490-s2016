import sys
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial import KDTree, cKDTree
import perception
import color

ROS_DEPTH_TOPIC = "/camera/depth/points"
ROS_DEPTH_TOPIC = "/realsense/pc"
# ROS_DEPTH_TOPIC = "/cloud_pcd"

rospy.init_node("listener", anonymous=True)
STEP = 25
cloud = rospy.wait_for_message(ROS_DEPTH_TOPIC, PointCloud2)
if perception.isCloudValid(cloud):
    np_cloud = perception.convertPc2ToNp(cloud)
    # np_cloud, pointmean = perception.calPointCloud(np_cloud)
    np_cloud = perception.subtractShelf(np_cloud)
    np_cloud =  np_cloud[np_cloud[:,0] != 0 , :]
    np.savez('sub', np_cloud[:,0],np_cloud[:,1],np_cloud[:,2])
    cloud = np_cloud[np_cloud[:,0] != 0 , :]
    # cloud = cloud[cloud[:,5] != 255,:]
    print len(cloud)
    print cloud 
    # np.savez('object', cloud[:,0],cloud[:,1],cloud[:,2])
    y,u,v = color.rgb_to_yuv(cloud[:,4],cloud[:,5],cloud[:,6])
    print 'y', y
    print 'u', u
    print 'v', v
    uv = [color.rgb_to_yuv(*rgb)[1:3] for rgb in cloud[:,4:7]]
    # uv = [u,v]
    print uv
    hist = color.make_uv_hist(uv)
    print hist
    a = hist.sum()
    print a
    old_hist = np.load('hist.npz')
    old_hist = old_hist['arr_0']
    print old_hist.sum()
    # np.savez('hist', hist)
    # print old_hist
    score = 2*np.minimum(hist,old_hist).sum()-np.maximum(hist,old_hist).sum()
    print score
    # print len(cloud)
    # plane1 = perception.segmentation(cloud)
    # np.savez('subfinished', plane1[:,0],plane1[:,1],plane1[:,2])
    # plane = perception.segmentation(np_cloud)
    print'finish sub '
    # print plane