
from __future__ import division
import numpy as np
from scipy.spatial import KDTree
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from common_utils import *
import sys

def generate_plane(ymin, ymax, zmin, zmax, x, yres=30, zres=30):
    points = []
    for i in np.linspace(ymin, ymax, yres):
        for j in np.linspace(zmin, zmax, zres):
            points.append([x,i,j])
    return points

def avg_perpendicular_dist(plane_points, point_cloud, yz_kdtree=None, proportion_used=0.5):
    '''
    plane_points and point_cloud are both list of lists (points) i.e. [[x1,y1,z1], [x2,y2,z2], ..., [xn,yn,zn]]
    all points in plane_points have the same x-coordinate
    '''
    plane_x = plane_points[0][0]
    x_vals = [x for x,_,_ in point_cloud]
    if yz_kdtree is None:
        pointsyz = [[y,z] for _,y,z in point_cloud]
        yz_mat = np.matrix(pointsyz)
        yz_kdtree = KDTree(yz_mat)
    plane_yz_np = np.matrix([[y,z] for _,y,z in plane_points])
    _, idx = yz_kdtree.query(plane_yz_np)
    all_x_coords = [x_vals[i] for i in idx] # x-index of the selected nearest neighbors
    abs_dists = map(abs, [x-plane_x for x in all_x_coords])
    abs_dists.sort()
    kept = abs_dists[0:int(len(abs_dists)*proportion_used)]
    return sum(kept)/len(kept)

def remove_side(xyz, yz, x, y, z, expected_x_loc=None, tol=0.05):
    x_sorted = sorted(x)
    y_sorted = sorted(y)
    z_sorted = sorted(z)
    x10 = quantile(x_sorted, 0.1)
    x90 = quantile(x_sorted, 0.9)
    y10 = quantile(y_sorted, 0.1)
    y90 = quantile(y_sorted, 0.9)
    z10 = quantile(z_sorted, 0.1)
    z90 = quantile(z_sorted, 0.9)
    kdtree = KDTree(yz)
    ymin = y10
    ymax = y90
    zmin = z10
    zmax = z90
    xs = np.linspace(x_sorted[0], x_sorted[-1], 50)
    avg_dists = []
    for x in xs:
        avg_dists.append(avg_perpendicular_dist(generate_plane(ymin, ymax, zmin, zmax, x), xyz, kdtree))
    min_dist_x = xs[argmin(avg_dists)]
    min_dist = min(avg_dists)
    if expected_x_loc is not None:
        if abs(expected_x_loc - min_dist_x)>tol:
            return None
    return min_dist_x, min_dist, xs, avg_dists



if __name__ == '__main__':
    if len(sys.argv)<3:
        print "Usage: \"program arg1 arg2\", where arg1 is input file name and arg2 is output file name (to be created)"
        quit()

    pointsxyzrgb, pointsxyz, pointsyz, all_x, all_y, all_z = read_pcd_file(sys.argv[1], ['xyzrgb', 'xyz', 'yz', 'x', 'y', 'z'])

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
    yz_mat = np.matrix(pointsyz)
    kdtree = KDTree(yz_mat)

    print "Total point number:", len(pointsxyz)

    ymin = y10
    ymax = y90
    zmin = z10
    zmax = z90
    xs = np.linspace(all_x[0], all_x[-1], 50)
    print x10, x90, y10, y90, z10, z90
    avg_dists = []
    for x in xs:
        avg_dists.append(avg_perpendicular_dist(generate_plane(ymin, ymax, zmin, zmax, x), pointsxyz, kdtree))
        print "."

    plt.figure()
    plt.plot(xs, avg_dists)
    min_dist = xs[argmin(avg_dists)]
    print min_dist

    points_no_plane = remove_plane(pointsxyzrgb, min_dist, "yz")
    write_pcd_file(points_no_plane, sys.argv[2])
    print len(pointsxyz)
    print len(points_no_plane)

    ax_full = render_3d_scatter(pointsxyzrgb, 0.01)
    ax_seg = render_3d_scatter(points_no_plane, 0.1)
    print ax_full.get_xlim3d()
    ax_seg.set_xlim3d(ax_full.get_xlim3d())
    ax_seg.set_ylim3d(ax_full.get_ylim3d())
    ax_seg.set_zlim3d(ax_full.get_zlim3d())
    plt.show()

