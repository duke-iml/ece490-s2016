import sys
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial import KDTree, cKDTree
import perception

ROS_DEPTH_TOPIC = "/camera/depth/points"

rospy.init_node("listener", anonymous=True)
STEP = 25
cloud = rospy.wait_for_message(ROS_DEPTH_TOPIC, PointCloud2)
if perception.isCloudValid(cloud):
    np_cloud = perception.convertPc2ToNp(cloud)
    # np_cloud, pointmean = perception.calPointCloud(np_cloud)
    np_cloud = perception.subtractShelf(np_cloud)
    cloud = np_cloud[np_cloud[:,0] != 0 , :]
    plan1 = perception.segmentation(cloud)
    np.savez('subfinished', np_cloud[:,0],np_cloud[:,1],np_cloud[:,2])
    # plane = perception.segmentation(np_cloud)
    print'finish sub '
    # print plane