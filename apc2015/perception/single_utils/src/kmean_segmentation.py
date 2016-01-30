
from __future__ import division
import numpy as np
import matplotlib as plt
from common_utils import *
from sklearn.cluster import KMeans
import sys


def kmean_segment(point_cloud, num_clusters, dist_to_ideal):
    '''
    run kmean on point_cloud with specified number of clusters. point_cloud is a segmented-out cloud that can contain multiple objects as well as some noise. 
    dist_to_ideal a function that takes a point and returns its distance to something. lower distance is better. here the distance should be the distance to a line. 
    return a list of indices of points that are in the best cluster
    '''
    kmeans = KMeans(n_clusters=int(sys.argv[2]))
    labels = kmeans.fit_predict(xyz)
    min_idx = None
    min_dist = None    
    for i in xrange(num_clusters):
        center = list(np.array(kmeans.cluster_centers_[i,:]))
        cur_dist = dist_to_ideal(min_dist)
        if min_idx is None or min_dist>cur_dist:
            min_idx = i
            min_dist = cur_dist
    return 
    


if len(sys.argv)<3:
    print "Usage: program arg1 arg2 where arg1 is point cloud file and arg2 is number of clusters"
xyzrgb, xyz, x, y, z = read_pcd_file(sys.argv[1], ['xyzrgb', 'xyz', 'x', 'y', 'z'])

kmeans = KMeans(n_clusters=int(sys.argv[2]))
labels = kmeans.fit_predict(xyz)
print kmeans.cluster_centers_
color = []
for l in labels:
    if l==0:
        color.append((1,0,0))
    elif l==1:
        color.append((0,1,0))
    else:
        color.append((0,0,1))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
every = 10
x_selected = [x for i,(x,_,_,_) in enumerate(xyzrgb) if i%every==0]
y_selected = [y for i,(_,y,_,_) in enumerate(xyzrgb) if i%every==0]
z_selected = [z for i,(_,_,z,_) in enumerate(xyzrgb) if i%every==0]
c_selected = [c for i,c in enumerate(color) if i%every==0]
ax.scatter(x_selected, y_selected, zs=z_selected, c=c_selected, linewidths=0)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_aspect('equal')
plt.show()
