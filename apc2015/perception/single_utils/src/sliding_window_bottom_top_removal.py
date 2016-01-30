
from __future__ import division
import numpy as np
from scipy.spatial import KDTree
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from common_utils import *
import sys

def generate_plane(xmin, xmax, zmin, zmax, y, yres=30, zres=30):
    points = []
    for i in np.linspace(xmin, xmax, yres):
        for j in np.linspace(zmin, zmax, zres):
            points.append([i,y,j])
    return points

def avg_perpendicular_dist(plane_points, point_cloud, xz_kdtree=None, proportion_used=0.5):
    '''
    plane_points and point_cloud are both list of lists (points) i.e. [[x1,y1,z1], [x2,y2,z2], ..., [xn,yn,zn]]
    all points in plane_points have the same x-coordinate
    '''
    plane_y = plane_points[0][1]
    y_vals = [y for _,y,_ in point_cloud]
    if xz_kdtree is None:
        pointsxz = [[x,z] for x,_,z in point_cloud]
        xz_mat = np.matrix(pointsxz)
        xz_kdtree = KDTree(xz_mat)
    plane_xz_np = np.matrix([[x,z] for x,_,z in plane_points])
    _, idx = xz_kdtree.query(plane_xz_np)
    all_y_coords = [y_vals[i] for i in idx] # y-index of the selected nearest neighbors
    abs_dists = map(abs, [y-plane_y for y in all_y_coords])
    abs_dists.sort()
    kept = abs_dists[0:int(len(abs_dists)*proportion_used)]
    return sum(kept)/len(kept)

def remove_bottom_top(xyz, xz, x, y, z, expected_y_loc=None, tol=0.05):
    x_sorted = sorted(x)
    y_sorted = sorted(y)
    z_sorted = sorted(z)
    x10 = quantile(x_sorted, 0.1)
    x90 = quantile(x_sorted, 0.9)
    y10 = quantile(y_sorted, 0.1)
    y90 = quantile(y_sorted, 0.9)
    z10 = quantile(z_sorted, 0.1)
    z90 = quantile(z_sorted, 0.9)
    kdtree = KDTree(xz)
    xmin = x10
    xmax = x90
    zmin = z10
    zmax = z90
    ys = np.linspace(y_sorted[0], y_sorted[-1], 50)
    avg_dists = []
    for y in ys:
        avg_dists.append(avg_perpendicular_dist(generate_plane(xmin, xmax, zmin, zmax, y), xyz, kdtree))
    min_dist_y = ys[argmin(avg_dists)]
    min_dist = min(avg_dists)
    if expected_y_loc is not None:
        if abs(expected_y_loc - min_dist_y)>tol:
            print "Expect:", expected_y_loc, "Got:", min_dist_y
            return None
    return min_dist_y, min_dist, ys, avg_dists


if __name__=="__main__":
    if len(sys.argv)<3:
        print "Usage: \"program arg1 arg2\", where arg1 is input file name and arg2 is output file name (to be created)"
        quit()

    pointsxyzrgb, pointsxyz, pointsxz, all_x, all_y, all_z = read_pcd_file(sys.argv[1], ['xyzrgb', 'xyz', 'xz', 'x', 'y', 'z'])

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
    xz_mat = np.matrix(pointsxz)
    kdtree = KDTree(xz_mat)

    print "Total point number:", len(pointsxyz)

    xmin = x10
    xmax = x90
    zmin = z10
    zmax = z90
    ys = np.linspace(all_y[0], all_y[-1], 50)
    print x10, x90, y10, y90, z10, z90
    avg_dists = []
    for y in ys:
        avg_dists.append(avg_perpendicular_dist(generate_plane(xmin, xmax, zmin, zmax, y), pointsxyz, kdtree))
        print "."

    plt.figure()
    plt.plot(ys, avg_dists)
    min_dist = ys[argmin(avg_dists)]
    print min_dist

    points_no_plane = remove_plane(pointsxyzrgb, min_dist, "xz")
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

