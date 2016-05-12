# Put the robot in Q_SCAN_BIN, make sure shelf is empty,
# and run this file to put the empty shelf point cloud 
# in the place full_integration_master expects

# Usage: python save_calibrated_shelf.py shelf

import sys
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import perception

STEP = 25
ROS_TOPIC = "/realsense/pc"

print sys.argv

if len(sys.argv) >= 2:
    SAVE_LOCATION = sys.argv[1]
else:
    print "ERROR: You must supply a path (no need for .npz extension) to save to"
    print "Usage: python save_calibrated_shelf.py myfile"
    sys.exit(0)

class SaveF200DepthSnapshot:
    def __init__(self):
        self.show = True
    def callback(self, data):
        if self.show:
            print data.fields
            xs = []
            ys = []
            zs = []
            # For some reason, returns x in [0], y in  [1], z in [2], rgb in [3]
            for point in pc2.read_points(data,skip_nans=False,field_names=("rgb","x","y","z")):
                xs.append(point[0])
                ys.append(point[1])
                zs.append(point[2])
            print "Length: " + str(len(xs))
            #add start
            cloud = np.array([xs,ys,zs])
            nans = np.isnan(cloud)
            cloud[nans] = 0
            cloud = cloud.transpose()
            print cloud[0]
            # cloud, _ = perception.calPointCloud(cloud)
            #add end
            #cloud = cloud.transpose()
            cloud = cloud[::STEP]
            print cloud
            xs = cloud[:,0]
            ys = cloud[:,1]
            zs = cloud[:,2]
            #segmentation(cloud)
            # print xs
            # print ys
            print "length after down sample: "+str(len(zs))
            if len(xs) > 0:
                self.show = False
                print "Saving to file: " + SAVE_LOCATION
                np.savez(SAVE_LOCATION, xs, ys, zs)
                print 'done saving, you can exit now with CTRL-C'

    def listener(self):
        rospy.init_node("listener", anonymous=True)
        rospy.Subscriber(ROS_TOPIC, PointCloud2, self.callback)
        rospy.spin()

if __name__ == '__main__':
    SaveF200DepthSnapshot().listener()

