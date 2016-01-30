
from __future__ import division
from common_utils import *
import cv2
import numpy as np
import sys
import colorsys
import sliding_window_back_removal as sw_back
from math import pi, tan, sqrt
import matplotlib.pyplot as plt

if len(sys.argv) < 3:
	print "Usage: 'program arg1 arg2' where arg1 is the RGB image looking at the order bin from top down and arg2 is the PCD file taken at the same time"

def is_correct_color(h):
	# the line below is for the red order bin used in the competition
	return 0<h<10 or 165<h<180
	# the line below is for the yellowish card board box used during practice
	# return 10<h<90

def get_h_from_rgb(r, g, b):
	h, _, _ = colorsys.rgb_to_hsv(r/255, g/255, b/255)
	return int(h*180)

width = 640
height = 480
diag_ang = 74*pi/180
lift = sqrt(width**2+height**2)/2 / tan(diag_ang/2)
def get_xy_pixel_from_xyz_depth(x,y,z):
	'''
	return the (x,y) coordinate in the unit of pixel on the image that correspond to (x,y,z) in the point cloud
	'''
	x_image = x/z*lift + width/2
	y_image = y/z*lift + height/2
	if not 0<=x_image<=width:
		print "x!", x, y, z, x_image
	if not 0<=y_image<=height:
		print "y!", x, y, z, y_image
	return x_image, y_image



img = cv2.imread(sys.argv[1])
hsv = cv2.cvtColor(img, cv2.cv.CV_BGR2HSV)
hue = hsv[:,:,0]
xyzrgb, xyz, xy, x, y, z = read_pcd_file(sys.argv[2], ['xyzrgb', 'xyz', 'xy', 'x', 'y', 'z'])
min_z, min_dist, zs, dists = sw_back.remove_back(xyz, xy, x, y, z, num_windows=10)
_, tmp_points_idx = filter_val_idx(lambda z:abs(z-min_z)<0.03, z)
xyzrgb_tmp = select(xyzrgb, tmp_points_idx)
xyz_tmp = select(xyz, tmp_points_idx)
xy = select(xy, tmp_points_idx)
x = select(x, tmp_points_idx)
y = select(y, tmp_points_idx)
z = select(z, tmp_points_idx)
a, b, c, d = fit_plane(xyz_tmp)
_, all_points_idx = filter_val_idx(lambda p:dist_point_plane(p, a, b, c, d)<0.03, xyz)
xyzrgb_final = select(xyzrgb, all_points_idx)


eligible_points = []
for p in xyzrgb_final:
	r, g, b = pcl_float_to_rgb(p[3])
	if is_correct_color(get_h_from_rgb(r,g,b)):
		eligible_points.append(p)

finalhs = [get_h_from_rgb(*pcl_float_to_rgb(rgb)) for _,_,_,rgb in xyzrgb_final]

render_3d_scatter(xyzrgb, 0.1)
render_3d_scatter(xyzrgb_final, 0.3)
plt.figure()
plt.hist(finalhs, 30)
render_3d_scatter(eligible_points)

eligible_xs = [x for x,_,_,_ in eligible_points]
eligible_ys = [y for _,y,_,_ in eligible_points]
eligible_zs = [z for _,_,z,_ in eligible_points]

x_median = np.median(eligible_xs)
y_median = np.median(eligible_ys)
z_median = np.median(eligible_zs)

print x_median, y_median, z_median

plt.show()
