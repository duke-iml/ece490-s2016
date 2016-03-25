import sys
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial import KDTree, cKDTree


class SaveF200DepthSnapshot:
    def __init__(self):
        self.show = True

    def callback(self, data):
        STEP = 1
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
            if len(xs) > 0:
                self.show = False
                np.savez('pc', xs, ys, zs)
                print 'done saving, you can exit now'
            NPZ_FILE = 'shelf.npz'
            STEP = 10 # Plot every STEPth point for speed, set to 1 to plot all
            data1 = np.load(NPZ_FILE)#data['arr_0'][::STEP], data['arr_1'][::STEP], data['arr_2'][::STEP]
            dobj = np.array([xs,ys,zs])
            dobj = dobj.transpose()
            dshel = np.mat([data1['arr_0'],data1['arr_1'],data1['arr_2']])
            dshel = dshel.transpose()
            #d2 = np.concatenate((xs,ys,zs), axis=1)
            #d2 = [xs ys zs]
            #d1[:,0] = data['arr_0'];d1[:,1] = data['arr_1'];d1[:,2] = data['arr_2'];
            #d2[:,0] = xs;d1[:,1] = ys;d2[:,2] = zs;
            t = cKDTree(dobj)
            d, idx = t.query(dshel, k=20, eps=0, p=2, distance_upper_bound=0.2)
            print 'finish kdtree'
            Points_To_Delete = []
            for i in range(0,len(idx)):
                for j in range(0,19):
                    if d[i,j] <= 0.15:
                        Points_To_Delete.append(idx[i,j])
            # print Points_To_Delete
            print len(dobj)            #dobj[idx[i,j],:]  = [0,0,0]
            dobj = np.delete(dobj,Points_To_Delete,axis = 0)
            
            com = np.mean(dobj,axis = 0)
            print com
            print len(dobj)
            print'finish sub '
            self.show = False
           
            np.savez('substracted', dobj[:,0],dobj[:,1],dobj[:,2])
    def listener(self):
        rospy.init_node("listener", anonymous=True)
        rospy.Subscriber("/camera/depth/points", PointCloud2, self.callback)
        rospy.spin()

if __name__ == '__main__':
    SaveF200DepthSnapshot().listener()