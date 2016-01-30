
from __future__ import division
import numpy as np
from scipy.spatial import KDTree
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from common_utils import *
import sys

def generate_plane(xmin, xmax, ymin, ymax, z, xres=30, yres=30):
    points = []
    for i in np.linspace(xmin, xmax, xres):
        for j in np.linspace(ymin, ymax, yres):
            points.append([i,j,z])
    return points

def avg_perpendicular_dist(plane_points, point_cloud, xy_kdtree=None, proportion_used=0.5):
    '''
    plane_points and point_cloud are both list of lists (points) i.e. [[x1,y1,z1], [x2,y2,z2], ..., [xn,yn,zn]]
    all points in plane_points have the same z-coordinate
    '''
    plane_z = plane_points[0][2]
    z_vals = [z for _,_,z in point_cloud]
    if xy_kdtree is None:
        pointsxy = [[x,y] for x,y,_ in point_cloud]
        xy_mat = np.matrix(pointsxy)
        xy_kdtree = KDTree(xy_mat)
    plane_xy_np = np.matrix([[x,y] for x,y,_ in plane_points])
    _, idx = xy_kdtree.query(plane_xy_np)
    all_z_coords = [z_vals[i] for i in idx] # z-index of the selected nearest neighbors
    abs_dists = map(abs, [z-plane_z for z in all_z_coords])
    abs_dists.sort()
    kept = abs_dists[0:int(len(abs_dists)*proportion_used)]
    return sum(kept)/len(kept)

def remove_back(xyz, xy, x, y, z, expected_z_loc=None, tol=0.03, num_windows=50):
    '''
    remove a plane perpendicular to z-axis
    xyz must NOT be xyzrgb
    '''
    x_sorted = sorted(x)
    y_sorted = sorted(y)
    z_sorted = sorted(z)
    x10 = quantile(x_sorted, 0.1)
    x90 = quantile(x_sorted, 0.9)
    y10 = quantile(y_sorted, 0.1)
    y90 = quantile(y_sorted, 0.9)
    z10 = quantile(z_sorted, 0.1)
    z90 = quantile(z_sorted, 0.9)
    kdtree = KDTree(xy)
    xmin = x10
    xmax = x90
    ymin = y10
    ymax = y90
    zs = np.linspace(z_sorted[0], z_sorted[-1], num_windows)
    avg_dists = []
    for z in zs:
        avg_dists.append(avg_perpendicular_dist(generate_plane(xmin, xmax, ymin, ymax, z), xyz, kdtree))
    min_dist_z = zs[argmin(avg_dists)]
    min_dist = min(avg_dists)
    if expected_z_loc is not None:
        if abs(expected_z_loc - min_dist_z)>tol:
            return None
    return min_dist_z, min_dist, zs, avg_dists


if __name__=="__main__":
    if len(sys.argv)<3:
        print "Usage: \"program arg1 arg2\", where arg1 is input file name and arg2 is output file name (to be created)"
        quit()

    pointsxyzrgb, pointsxyz, pointsxy, all_x, all_y, all_z = read_pcd_file(sys.argv[1], ['xyzrgb', 'xyz', 'xy', 'x', 'y', 'z'])

    all_x.sort()
    all_y.sort()
    all_z.sort()

    x10 = quantile(all_x, 0.1)
    x90 = quantile(all_x, 0.9)
    y10 = quantile(all_y, 0.1)
    y90 = quantile(all_y, 0.9)
    z10 = quantile(all_z, 0.1)
    z90 = quantile(all_z, 0.9)
        
    xyz_mat = np.matrix(pointsxyz)
    xy_mat = np.matrix(pointsxy)
    print xy_mat.shape
    kdtree = KDTree(xy_mat)


    print len(pointsxyz)

    xmin = x10
    xmax = x90
    ymin = y10
    ymax = y90
    zs = np.linspace(all_z[0], all_z[-1], 50)
    avg_dists = []
    for z in zs:
        avg_dists.append(avg_perpendicular_dist(generate_plane(xmin, xmax, ymin, ymax, z), pointsxyz, kdtree))

    plt.figure()
    plt.plot(zs, avg_dists)
    min_dist = zs[argmin(avg_dists)]
    print min_dist
    points_no_plane = remove_plane_rgb(pointsxyzrgb, min_dist, "xy")

    write_pcd_file(points_no_plane, sys.argv[2])
    print len(pointsxyz)
    print len(points_no_plane)

    ax_full = render_3d_scatter(pointsxyz, 0.01)
    ax_seg = render_3d_scatter(points_no_plane, 0.05)
    ax_seg.set_xlim3d(ax_full.get_xlim3d())
    ax_seg.set_ylim3d(ax_full.get_ylim3d())
    ax_seg.set_zlim3d(ax_full.get_zlim3d())
    plt.show()

