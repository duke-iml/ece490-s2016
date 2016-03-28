# Reads a point cloud from ROS_TOPIC and then saves it to a numpy file

import sys
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import perception

ROS_TOPIC = "/camera/depth/points"

class SaveF200DepthSnapshot:
    def __init__(self):
        self.show = True
    def callback(self, data):
        if self.show:
            print data.fields
            xs = []
            ys = []
	    zs = []
            for point in pc2.read_points(data, skip_nans=True):
                xs.append(point[0])
                ys.append(point[1])
                zs.append(point[2])
            print "Length: " + str(len(xs))
            #add start
            cloud = np.array([xs,ys,zs])
            cloud = cloud.transpose()
            print cloud[0]
            cloud = perception.calPointCloud(cloud)
            #add end
            #cloud = cloud.transpose()
            xs = cloud[:,0]
            ys = cloud[:,1]
            zs = cloud[:,2]
            #segmentation(cloud)
            # print xs
            # print ys
            # print zs
            if len(xs) > 0:
                self.show = False
                np.savez('shelf', xs, ys, zs)
                print 'done saving, you can exit now'

    def listener(self):
        rospy.init_node("listener", anonymous=True)
        rospy.Subscriber(ROS_TOPIC, PointCloud2, self.callback)
        rospy.spin()

if __name__ == '__main__':
    SaveF200DepthSnapshot().listener()

