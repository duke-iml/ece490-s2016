
from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from realsense_utils.common_utils import render_3d_scatter
from scipy.spatial import KDTree
from scipy.stats import gaussian_kde

def subtract_model(model_cloud, scene_cloud=None, scene_tree=None, threshold=0.01):
	'''
	subtract model from scene. return indices of scene points that should be kept (beyond a threshold of any points in the model)
	'''
	assert scene_cloud is not None or scene_tree is not None, 'Either scene_model or scene_tree must be provided'
	model_cloud = np.array(model_cloud)
	if scene_tree is None:
		scene_cloud = np.array(scene_cloud)
		assert scene_cloud.shape[1]==3, 'scene_cloud dimension must be Nx3'
		print "Making KDTree with %d points"%scene_cloud.shape[0]
		scene_tree = KDTree(scene_cloud)
	print "Querying neighbors within %f of %d points"%(threshold, model_cloud.shape[0])
	idxs = scene_tree.query_ball_point(model_cloud, threshold)
	print "Done"
	idxs = [i for group in idxs for i in group] # flattened indices
	discard_idxs = set(idxs)
	all_idxs = set(range(scene_cloud.shape[0]))
	keep_idxs = all_idxs - discard_idxs
	return list(keep_idxs)

tote_cloud = np.load('tote.npy') / 1000
keep_idxs = np.where(tote_cloud[:,2]!=0)[0]
tote_cloud = tote_cloud[keep_idxs, :]
tote_with_objects_cloud = np.load('tote_with_objects.npy') / 1000
keep_idxs = np.where(tote_with_objects_cloud[:,2]!=0)[0]
tote_with_objects_cloud = tote_with_objects_cloud[keep_idxs, :]
tote_cloud = tote_cloud[::50, :]
tote_with_objects_cloud = tote_with_objects_cloud[::50, :]
# render_3d_scatter(tote_cloud)
# render_3d_scatter(tote_with_objects_cloud)
# plt.show()

idxs = subtract_model(tote_cloud, scene_cloud=tote_with_objects_cloud)
tote_content_cloud = tote_with_objects_cloud[idxs, :]

xs = tote_content_cloud[:,0]
ys = tote_content_cloud[:,1]
zs = tote_content_cloud[:,2]
keep_idxs = np.logical_and(np.logical_and(-0.23<xs, xs<0.2), np.logical_and(-0.2<ys, ys<0.15))
tote_content_cloud = tote_content_cloud[keep_idxs, :]
xs = tote_content_cloud[:,0]
ys = tote_content_cloud[:,1]
zs = tote_content_cloud[:,2]
render_3d_scatter(tote_content_cloud)
# plt.show()

plt.figure()
plt.scatter(xs, ys)

'''
ASSUMING UP IS Z DIRECTION
'''
xs_sorted = sorted(list(xs.flat))
ys_sorted = sorted(list(ys.flat))
N = len(xs_sorted)
xmin, xmax = xs_sorted[int(N*0.1)], xs_sorted[int(N*0.9)] # in the unit of meter
ymin, ymax = ys_sorted[int(N*0.1)], ys_sorted[int(N*0.9)]
x_inrange = np.logical_and(xs >= xmin, xs <= xmax)
y_inrange = np.logical_and(ys >= ymin, ys <= ymax)
inrange = np.logical_and(x_inrange, y_inrange)
tote_content_cloud_inrange = tote_content_cloud[inrange, :]
cloud_inrange_xy = tote_content_cloud_inrange[:, 0:2]
gaussian_kernel = gaussian_kde(cloud_inrange_xy.T)
x_grid, y_grid = np.meshgrid(np.linspace(xmin, xmax, 100), np.linspace(ymin, ymax, 100), indexing='ij')
positions = np.vstack([x_grid.ravel(), y_grid.ravel()])
densities = np.reshape(gaussian_kernel(positions).T, x_grid.shape)
max_idx = densities.argmax()
maxx, maxy = x_grid.flatten()[max_idx], y_grid.flatten()[max_idx]
plt.hold(True)
plt.scatter(maxx, maxy, color='r')
plt.figure()
plt.imshow(densities)
plt.show()
# -0.23<x<0.2, -0.2<y<0.15 