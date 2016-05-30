
from __future__ import division
import numpy as np
from realsense_utils.client import RemoteCamera
from realsense_utils.common_utils import render_3d_scatter
from realsense_utils import colorize_point_cloud
# def colorize_point_cloud(cloud, color, depth_uv, rgb_table=None)
import struct, icp, sys, threading
from scipy.spatial import KDTree
from scipy.sparse import dok_matrix
from scipy.stats import gaussian_kde
import matplotlib.pyplot as plt
from klampt import se3
from segmentation import distance_label
from scipy.sparse.csgraph import connected_components

sys.setrecursionlimit(10000)
downsample_rate = 100

class CameraData:
	def __init__(self, color, cloud, depth_uv, color_uv):
		self.color = color
		self.cloud = cloud
		self.depth_uv = depth_uv
		self.color_uv = color_uv

class Perceiver:
	'''
	DONE def __init__(self, camera_connect=True)
	DONE def read_loop_in_background(self)
	DONE def subtract_model(self, model_cloud, scene_cloud=None, scene_tree=None, threshold=0.01)
	DONE def get_current_bin_content_cloud(self, bin_letter, cur_camera_R, cur_camera_t, colorful=True, fit=False, threshold=0.01)
	DONE def get_current_point_cloud(self, colorful=False, tolist=True)
	DONE def get_current_tote_content_cloud(self, scene_cloud=None, threshold=0.02, fit=False) bin_letter, cur_camera_R, cur_camera_t)
	DONE def icp_get_bin_transform(self, bin_letter, cur_camera_R, cur_camera_t)
	DONE def load_model_bin_cloud(self, bin_letter, downsample=False)
	DONE def read_once(self, unit='meter', Nx3_cloud=False, clean=None)
	DONE def get_canonical_bin_cloud_path(self, bin_letter)
	DONE def get_bin_viewing_camera_xform_path(self, bin_letter)
	DONE def save_canonical_bin_point_cloud(self, bin_letter, R, t)
	DONE def save_canonical_tote_cloud(self, R, t)
	DONE def save_R_t(self, filename, R, t)
	DONE def load_R_t(self, filename, nparray=True)
	UNFINISHED def get_position_of_item_in_bin(self, item_name, possible_items=None, return_normal=False)
	UNFINISHED def get_picking_position_for_stowing(self)
	UNFINISHED def perceive_single_object(self, possible_items=None)
	UNFINISHED def get_bin_empty_location(self)
	'''

	def __init__(self, camera_connect=True):
		# self.prefix = ''
		self.prefix = '/home/group3/ece490-s2016/apc2016/Integration/perception/'
		self.shelf_perturb_R = None
		self.shelf_perturb_t = None
		# if camera_connect:
		# 	self.camera = RemoteCamera('10.236.66.147', 30000)
		# 	self.cd = None
		# 	self.looping_thread = threading.Thread(target=self.read_loop_in_background)
		# 	self.looping_thread.daemon = True
		# 	self.looping_thread.start()

	def read_loop_in_background(self):
		while True:
			color, cloud, depth_uv, color_uv = self.camera.read()
			cd = CameraData(color, cloud, depth_uv, color_uv)
			self.cd = cd

	def subtract_model(self, model_cloud, scene_cloud=None, scene_tree=None, threshold=0.01):
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
		else:
			scene_cloud = scene_tree.data
		print "Querying neighbors within %f of %d points"%(threshold, model_cloud.shape[0])
		idxs = scene_tree.query_ball_point(model_cloud, threshold)
		print "Done"
		idxs = [i for group in idxs for i in group] # flattened indices
		discard_idxs = set(idxs)
		all_idxs = set(range(scene_cloud.shape[0]))
		keep_idxs = all_idxs - discard_idxs
		return list(keep_idxs)

	def get_current_bin_content_cloud(self, bin_letter, cur_camera_R, cur_camera_t, colorful=True, fit=False, threshold=0.01):
		'''
		return a point cloud (either colored or not) of the current bin content in the global frame. shelf is removed. 
		if fit is True, a new round of ICP is done on the model shelf
		cur_camera_R and cur_camera_t are the current camera transformation in world
		'''
		color, scene_cloud, depth_uv, _ = self.read_once(unit='meter', Nx3_cloud=False)
		if colorful:
			scene_cloud = colorize_point_cloud(scene_cloud, color, depth_uv).reshape(-1, 4)
		else:
			scene_cloud = scene_cloud.reshape(-1, 3)
		keep_idxs = np.where(scene_cloud[:,2].flatten()!=0)[0]
		scene_cloud = scene_cloud[keep_idxs, :]
		scene_cloud = scene_cloud[::downsample_rate, :]
		scene_cloud_xyz = scene_cloud[:, 0:3]
		cur_camera_R = np.array(cur_camera_R).reshape(3,3).T
		cur_camera_t = np.array(cur_camera_t)
		scene_cloud_xyz = scene_cloud_xyz.dot(cur_camera_R)+cur_camera_t # colorless cloud
		scene_cloud[:, 0:3] = scene_cloud_xyz # transform scene_cloud
		scene_tree = KDTree(scene_cloud_xyz)
		model_cloud = self.load_model_bin_cloud(bin_letter, downsample=True)
		model_xform_R, model_xform_t = self.load_R_t(self.get_bin_viewing_camera_xform_path(bin_letter), nparray=True)
		model_cloud = model_cloud.dot(model_xform_R) + model_xform_t # apply saved transformation
		assert (self.shelf_perturb_R is not None) and (self.shelf_perturb_t is not None), 'Shelf perturbation has not been calibrated'
		model_cloud = model_cloud.dot(self.shelf_perturb_R) + self.shelf_perturb_t # apply perturbation transformation found by ICP
		if fit:
			R, t = icp.match(model_cloud, scene_tree)
			model_cloud = model_cloud.dot(R.T) + t
		keep_idxs = self.subtract_model(model_cloud, scene_tree=scene_tree, threshold=threshold)
		bin_content_cloud = scene_cloud[keep_idxs, :]
		return bin_content_cloud

	def crop_cloud(self, cloud, bin_letter):
		'''
		UNIMPLEMENTED
		TAKE CARE OF COLOR CHANNEL
		'''
		return cloud

	def crop_and_segment(self, bin_content_cloud, bin_letter, threshold=0.01):
		cropped_cloud = self.crop_cloud(bin_content_cloud, bin_letter)
		N = cropped_cloud.shape[0]
		colorless_cloud = cropped_cloud[:,0:3]
		print 'Begin making sparse adjacency matrix...'
		adj_matrix = dok_matrix((N, N), dtype='bool_')
		for i in xrange(N):
			for j in xrange(i+1, N):
				if np.linalg.norm(colorless_cloud[i,:]-colorless_cloud[j,:])<threshold:
					adj_matrix[i,j] = 1
		adj_matrix = adj_matrix + adj_matrix.T + np.eye(N)
		print 'Done. Start computing connected component... '
		n_components, labels = connected_components(adj_matrix, directed=False, return_labels=True)
		print 'Done. Found %d components. '%n_components
		return cropped_cloud, labels, n_components

	def get_current_bin_content_cc_cloud(self, bin_letter, cur_camera_R, cur_camera_t, fit=False, 
		shelf_subtraction_threshold=0.01, cc_threshold=0.01):
		bin_content_cloud = self.get_current_bin_content_cloud(bin_letter, cur_camera_R, cur_camera_t, 
			colorful=False, fit=fit, threshold=shelf_subtraction_threshold)
		cropped_cloud, labels, n_components = self.crop_and_segment(bin_content_cloud, bin_letter, cc_threshold)
		cropped_cloud_with_color = np.hstack( (cropped_cloud, np.zeros( (cropped_cloud.shape[0],1) ) ) )
		for i in xrange(n_components):
			h = i / n_components
			r, g, b = map(lambda x: int(x*255), colorsys.hsv_to_rgb(h, 1, 1))
			idxs = np.where(labels==i)[0]
			pcl_float = rgb_to_pcl_float(r, g, b)
			cropped_cloud_with_color[idxs, 3] = pcl_float
		return cropped_cloud_with_color

	def get_current_point_cloud(self, cur_camera_R, cur_camera_t, colorful=False, tolist=True):
		'''
		physical pre-condition: place the camera to the desired viewing angle. 

		return the point cloud that the camera is currently perceiving. 

		if colorful is True, a colorized point cloud is returned, (it may take longer);
		else, a black point cloud is returned. 
		
		the black point cloud is a numpy matrix of h x w x 3, where point_cloud[i,j,:] 
		gives the 3D coordinate of that point in camera frame in meter. 

		the colored point cloud is a numpy matrix of h x w x 4, where the last layer is the rgb color, 
		packed in PCL format into a single float. RGB can be retrieved by pcl_float_to_rgb(f) function
		defined above. 
		
		if there is an error in retrieving data, None is returned
		'''
		color, cloud, depth_uv, _ = self.read_once(unit='meter', Nx3_cloud=False)
		if cloud is None:
			return None
		if colorful:
			if color is None or depth_uv is None:
				return None
			cloud = colorize_point_cloud(cloud, color, depth_uv)
		ch = cloud.shape[-1]
		cloud = cloud.reshape(-1, ch)
		keep_idxs = np.where(cloud[:,2].flatten()!=0)[0]
		cloud = cloud[keep_idxs, :]
		cur_camera_R = np.array(cur_camera_R).reshape(3,3).T
		cur_camera_t = np.array(cur_camera_t)
		cloud = cloud.dot(cur_camera_R.T) + cur_camera_t
		if tolist:
			cloud = cloud.tolist()
		return cloud

	def crop_tote_cloud(self, cloud):
		return cloud

	def get_picking_position_for_stowing(self, cur_camera_R, cur_camera_t, fit=False, return_tote_content_cloud=False):
		'''
		physical pre-condition: place the camera to the hard-coded tote-viewing position. 

		return (x,y,z) coordinate in global frame of position that has something to suck. 
		'''
		tote_content_cloud = self.get_current_tote_content_cloud(cur_camera_R, cur_camera_t)
		tote_content_cloud = self.crop_tote_cloud(tote_content_cloud)
		xs = tote_content_cloud[:,0]
		ys = tote_content_cloud[:,1]
		zs = tote_content_cloud[:,2]
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
		if not return_tote_content_cloud:
			return x_grid.flatten()[max_idx], y_grid.flatten()[max_idx]
		else:
			return (x_grid.flatten()[max_idx], y_grid.flatten()[max_idx]), tote_content_cloud

	def get_current_tote_content_cloud(self, cur_camera_R, cur_camera_t, scene_cloud=None, threshold=0.02, fit=False):
		'''
		subtract tote model from current point cloud of tote (with object in it)
		if fit is True, transform the model to align with the scene first. it will take some time
		return the point cloud of the remaining scene in global frame
		'''
		if scene_cloud is None:
			_, scene_cloud, _, _ = self.read_once(unit='meter', Nx3_cloud=True, clean=True)
		model_cloud = self.load_model_tote_cloud(downsample=True)
		assert model_cloud is not None, 'Error loading tote model'
		
		model_xform_R, model_xform_t = self.load_R_t(self.get_tote_viewing_camera_xform_path(), nparray=True)
		model_cloud = model_cloud.dot(model_xform_R) + model_xform_t

		scene_cloud = scene_cloud[::downsample_rate, :]
		cur_camera_R = np.array(cur_camera_R).reshape(3,3).T
		cur_camera_t = np.array(cur_camera_t)
		scene_cloud = scene_cloud.dot(cur_camera_R.T) + cur_camera_t # transform scene cloud
		
		print "Making scene cloud with %i points"%scene_cloud.shape[0]
		scene_tree = KDTree(scene_cloud)
		print "Done"
		
		if fit:
			R, t = icp.match(model_cloud, scene_tree)
			model_cloud = model_cloud.dot(R.T) + t

		keep_idxs = self.subtract_model(model_cloud, scene_tree=scene_tree, threshold=threshold)
		content_cloud = scene_cloud[keep_idxs, :]

		# print 'Dirty content cloud has %i points'%content_cloud.shape[0]

		num_content_cloud_pts = content_cloud.shape[0]
		print "Making content cloud with %i points"%content_cloud.shape[0]
		content_tree = KDTree(content_cloud)
		print "Done. Querying neighbors within 0.03 m"
		idxs = content_tree.query_ball_point(content_cloud, 0.03)
		unisolated_flag = map(lambda x: len(x)>=3, idxs)
		content_cloud = content_cloud[np.where(unisolated_flag)[0], :]
		print "Clean cloud has %i points"%content_cloud.shape[0]

		print "Making content cloud with %i points"%content_cloud.shape[0]
		content_tree = KDTree(content_cloud)
		print "Done. Querying neighbors within 0.03 m"
		idxs = content_tree.query_ball_point(content_cloud, 0.03)
		unisolated_flag = map(lambda x: len(x)>=4, idxs)
		content_cloud = content_cloud[np.where(unisolated_flag)[0], :]
		print "Clean cloud has %i points"%content_cloud.shape[0]

		print "Making content cloud with %i points"%content_cloud.shape[0]
		content_tree = KDTree(content_cloud)
		print "Done. Querying neighbors within 0.03 m"
		idxs = content_tree.query_ball_point(content_cloud, 0.03)
		unisolated_flag = map(lambda x: len(x)>=5, idxs)
		content_cloud = content_cloud[np.where(unisolated_flag)[0], :]
		print "Clean cloud has %i points"%content_cloud.shape[0]
		return content_cloud
		


	def get_current_tote_content_cloud_fast(self, cur_camera_R, cur_camera_t, scene_cloud=None, threshold=0.02, fit=False):
		'''
		subtract tote model from current point cloud of tote (with object in it)
		if fit is True, transform the model to align with the scene first. it will take some time
		return the point cloud of the remaining scene in global frame
		'''
		assert fit is False, 'ICP is too time-consuming'

		if scene_cloud is None:
			scene_cloud = self.read_cloud_avg(ite=10, unit='meter')
		cur_camera_R = np.array(cur_camera_R).reshape(3,3).T
		cur_camera_t = np.array(cur_camera_t)
		scene_cloud = scene_cloud.dot(cur_camera_R.T) + cur_camera_t # transform scene cloud
		scene_depth = scene_cloud[:,:,2]
		
		model_cloud = self.load_model_tote_cloud_avg()
		assert model_cloud is not None, 'Error loading tote model'
		model_xform_R, model_xform_t = self.load_R_t(self.get_tote_viewing_camera_xform_path(), nparray=True)
		model_cloud = model_cloud.dot(model_xform_R) + model_xform_t
		model_depth = model_cloud[:,:,2]

		depth_diff = np.abs(scene_depth - model_depth)
		depth_diff[np.isnan(depth_diff)] = 0

		print 'Maximum depth difference is:', depth_diff.max()
		plt.figure()
		plt.imshow(depth_diff)
		plt.title('Abs diff in depth')
		plt.colorbar()
		plt.figure()
		plt.imshow(depth_diff == 0)
		plt.show(block=False)

		x_select, y_select = np.where(depth_diff > threshold)
		content_cloud = scene_cloud[x_select, y_select, :]

		return content_cloud

	def get_shelf_transformation(self, bin_letter, cur_camera_R, cur_camera_t):
		'''
		physical pre-condition: place the camera to the hard-coded position of viewing bin_A

		bin_letter is one of ['A', 'B', ..., 'L']

		return the shelf transformation as (R, t), where R is a column-major order flattened array of rotation matrix, 
		and t is an array of 3 numbers for transformation. 
		'''
		shelf_perturb_R, shelf_perturb_t = self.icp_get_bin_transform(bin_letter, cur_camera_R, cur_camera_t)
		# save perturbation transformation for shelf subtraction during perception
		self.shelf_perturb_R = shelf_perturb_R
		self.shelf_perturb_t = shelf_perturb_t
		return shelf_perturb_R, shelf_perturb_t

	def icp_get_bin_transform(self, bin_letter, cur_camera_R, cur_camera_t):
		'''
		(private) get the current bin transform relative to canonical position for the specified bin
		bin_letter is one of ['A', 'B', ..., 'L'] and is used to load the pre-computed shelf cloud

		return (R, t), where R is a column-major order flattened array of rotation matrix, 
		and t is an array of 3 numbers for transformation. 
		'''
		cur_camera_R = np.array(cur_camera_R).reshape(3,3).T
		cur_camera_t = np.array(cur_camera_t).flatten()
		_, scene_cloud, _, _ = self.read_once(unit='meter', Nx3_cloud=True, clean=True)
		model_cloud = self.load_model_bin_cloud(bin_letter, downsample=True)
		assert model_cloud is not None, 'Cannot find model cloud'
		scene_cloud = scene_cloud[::downsample_rate,:]
		model_xform_R, model_xform_t = self.load_R_t(self.get_bin_viewing_camera_xform_path(bin_letter), nparray=True)
		model_cloud_xformed = model_cloud.dot(model_xform_R.T) + model_xform_t
		scene_cloud_xformed = scene_cloud.dot(cur_camera_R.T) + cur_camera_t
		np.save(self.prefix+'tmp/model_cloud_xformed.npy', model_cloud_xformed)
		np.save(self.prefix+'tmp/scene_cloud_xformed.npy', scene_cloud_xformed)
		scene_tree = KDTree(scene_cloud_xformed)
		relative_R, relative_t = icp.match(model_cloud_xformed, scene_tree)
		relative_R = list(relative_R.flatten(order='F'))
		relative_t = list(relative_t.flat)
		return relative_R, relative_t

	def load_model_bin_cloud(self, bin_letter, downsample=False):
		try:
			model_cloud = np.load(self.get_canonical_bin_cloud_path(bin_letter))
		except:
			print 'Cannot find bin model for bin_%s at %s'%(bin_letter, self.get_canonical_bin_cloud_path(bin_letter))
			return None
		if downsample:
			model_cloud = model_cloud[::downsample_rate, :]
		return model_cloud

	def load_model_tote_cloud(self, downsample=False):
		try:
			model_cloud = np.load(self.get_tote_cloud_path())
		except:
			print 'Cannot find tote model at %s'%self.get_tote_cloud_path()
			return None
		assert len(model_cloud.shape)==2, 'Size mismatch, expecting Nx3, current size is ' + str(model_cloud.shape)
		if downsample:
			model_cloud = model_cloud[::downsample_rate, :]
		return model_cloud

	def load_model_tote_cloud_avg(self, downsample=False):
		try:
			model_cloud = np.load(self.get_tote_cloud_path())
		except:
			print 'Cannot find tote model at %s'%self.get_tote_cloud_path()
			return None
		assert len(model_cloud.shape)==3, 'Size mismatch, expecting HxWx3, current size is ' + str(model_cloud.shape)
		if downsample:
			model_cloud = model_cloud[::downsample_rate, :]
		return model_cloud

	def read_once_poll(self, unit='meter', Nx3_cloud=False, clean=None):
		'''
		(private) read from CameraData object
		'''
		cd = self.cd
		color = cd.color
		cloud = cd.cloud
		depth_uv = cd.depth_uv
		color_uv = cd.color_uv
		if unit in ['meter', 'm']:
			cloud /= 1000
		elif unit in ['centimeter', 'centi-meter', 'cm']:
			cloud = cloud
		else:
			raise Exception('Unrecognized unit '+str(unit))
		if Nx3_cloud:
			cloud = cloud.reshape(-1, 3)
		if Nx3_cloud:
			assert clean is not None, 'clean must be a boolean when Nx3_cloud is True'
			if clean:
				keep_idxs = np.where(cloud[:,2].flatten()!=0)[0]
				cloud = cloud[keep_idxs,:]
		return color, cloud, depth_uv, color_uv

	def read_once(self, unit='meter', Nx3_cloud=False, clean=None):
		camera = RemoteCamera('10.236.66.147', 30000)
		color, cloud, depth_uv, color_uv = camera.read()
		camera.close()
		if unit in ['meter', 'm']:
			cloud /= 1000
		elif unit in ['centimeter', 'centi-meter', 'cm']:
			cloud = cloud
		else:
			raise Exception('Unrecognized unit '+str(unit))
		if Nx3_cloud:
			cloud = cloud.reshape(-1, 3)
		if Nx3_cloud:
			assert clean is not None, 'clean must be a boolean when Nx3_cloud is True'
			if clean:
				keep_idxs = np.where(cloud[:,2].flatten()!=0)[0]
				cloud = cloud[keep_idxs,:]
		return color, cloud, depth_uv, color_uv

	def read_average_depth(self, ite=10, unit='meter'):
		camera = RemoteCamera('10.236.66.147', 30000)
		all_depth = np.zeros((480, 640, ite))
		for i in xrange(ite):
			all_depth[:,:,i] = camera.read()[1][:,:,2] # [1] to index cloud, [:,:,2] to index z axis
		camera.close()
		all_depth[all_depth==0]=np.nan
		depth_avg = np.nanmean(all_depth, axis=2)
		print 'Shape of average depth is', depth_avg.shape
		if unit in ['meter', 'm']:
			depth_avg /= 1000
		elif unit in ['centimeter', 'centi-meter', 'cm']:
			depth_avg = depth_avg
		else:
			raise Exception('Unrecognized unit '+str(unit))
		return depth_avg

	def read_cloud_avg(self, ite=10, unit='meter', Nx3_cloud=False, clean=None):
		camera = RemoteCamera('10.236.66.147', 30000)
		all_cloud = np.zeros((480, 640, 3, ite))
		for i in xrange(ite):
			all_cloud[:,:,:,i] = camera.read()[1] # [1] to index cloud
		camera.close()
		all_cloud[all_cloud==0]=np.nan
		cloud_avg = np.nanmean(all_cloud, axis=3)
		print 'Shape of average cloud is', cloud_avg.shape
		if unit in ['meter', 'm']:
			cloud_avg /= 1000
		elif unit in ['centimeter', 'centi-meter', 'cm']:
			cloud_avg = cloud_avg
		else:
			raise Exception('Unrecognized unit '+str(unit))
		if Nx3_cloud:
			cloud_avg = cloud_avg.reshape(-1, 3)
		if Nx3_cloud:
			assert clean is not None, 'clean must be a boolean when Nx3_cloud is True'
			if clean:
				keep_idxs = np.where(cloud_avg[:,2].flatten()!=np.nan)[0]
				cloud_avg = cloud_avg[keep_idxs,:]
		return cloud_avg

	def get_canonical_bin_cloud_path(self, bin_letter):
		return self.prefix+'canonical_model_cloud/bin_%s.npy'%bin_letter

	def get_tote_cloud_path(self):
		return self.prefix+'canonical_model_cloud/tote.npy'

	def get_bin_viewing_camera_xform_path(self, bin_letter):
		return self.prefix+'canonical_model_cloud/bin_%s_xform.txt'%bin_letter

	def get_tote_viewing_camera_xform_path(self):
		return self.prefix+'canonical_model_cloud/tote_xform.txt'

	def save_canonical_bin_point_cloud(self, bin_letter, R, t):
		'''
		physical pre-condition: place the camera to the position viewing specified bin
		when the shelf is NOT perturbed. 

		bin_letter is one of ['A', 'B', ..., 'L']

		It will return None, but the model will be saved upon return
		'''
		assert isinstance(R, list), 'R must be a list but now it is '+str(R.__class__)
		assert isinstance(t, list), 't must be a list but now it is '+str(R.__class__)
		bin_letter = bin_letter.upper()
		assert 'A'<=bin_letter<='L', 'bin_letter must be between "A" and "L"'
		_, cloud, _, _ = self.read_once(unit='meter', Nx3_cloud=True, clean=True)
		np.save(self.get_canonical_bin_cloud_path(bin_letter), cloud)
		self.save_R_t(self.get_bin_viewing_camera_xform_path(bin_letter), R, t)
		print 'Successfully saved model for bin_'+bin_letter

	def save_canonical_tote_cloud(self, R, t):
		assert isinstance(R, list), 'R must be a list but now it is '+str(R.__class__)
		assert isinstance(t, list), 't must be a list but now it is '+str(R.__class__)
		_, cloud, _, _ = self.read_once(unit='meter', Nx3_cloud=True, clean=True)
		np.save(self.get_tote_cloud_path(), cloud)
		self.save_R_t(self.get_tote_viewing_camera_xform_path(), R, t)
		print 'Successfully saved model for tote'

	def save_canonical_tote_cloud_avg(self, R, t):
		assert isinstance(R, list), 'R must be a list but now it is '+str(R.__class__)
		assert isinstance(t, list), 't must be a list but now it is '+str(R.__class__)
		cloud = self.read_cloud_avg(ite=10, unit='meter')
		np.save(self.get_tote_cloud_path(), cloud)
		self.save_R_t(self.get_tote_viewing_camera_xform_path(), R, t)
		print 'Successfully saved averaged model for tote'

	def save_R_t(self, filename, R, t):
		'''
		save rotation and translation to file
		'''
		if isinstance(filename, basestring):
			try:
				filename = open(filename, 'w')
			except IOError:
				print 'Loading transformation file %s failed'%filename
				return
		filename.write(','.join(map(str, R))+'\n')
		filename.write(','.join(map(str, t))+'\n')
		filename.close()

	def load_R_t(self, filename, nparray=True):
		'''
		load rotation and translation from file
		'''
		if isinstance(filename, basestring):
			filename = open(filename)
		lines = filename.readlines()
		R = map(float, lines[0].split(','))
		t = map(float, lines[1].split(','))
		filename.close()
		if nparray:
			R = np.array(R).reshape(3,3).T
			t = np.array(t)
		return R, t

	def get_position_of_item_in_bin(self, item_name, possible_items=None, return_normal=False):
		'''
		physical pre-condition: place the camera to the calibrated bin-viewing position.
		calibrated position is the IK-solved position that is constant ***relative to the shelf***

		***setting item_name to None*** suggests that there is only one item in the bin. Thus, no vision
		algorithm is used. 

		possible_items is a list of item names that can possibly be in the bin.
		if None is provided, it will try to choose among all items. 

		return (x,y,z) coordinate of the center of detected item. If return_normal is True, return local 
		normal vector possibly for choosing suction direction (this operation may be slow). 
		'''
		raise NotImplemented

	def perceive_single_object(self, possible_items=None):
		'''
		physical pre-condition: place the camera in a position to view an object being picked up 
		by another object

		this method is called if the scale is not enough to discriminate object (e.g. when two or more 
		objects have similar weights). 

		possible_items is a list of item names that can possibly be the item being picked (similar weights).
		if None is provided, it will try to choose among all items. 
	
		return the official name of predicted item. 
		'''
		raise NotImplemented

	def get_bin_empty_location(self):
		'''
		physical pre-condition: place the camera to the calibrated bin-viewing position. 

		return (x,y,z) coordinate of with deepest z value. useful for determining stowing location. 
		'''
		raise NotImplemented

def remove_invalid_points(cloud):
	'''
	cloud is an numpy array of size N x 3
	remove all rows with 3rd entry being 0
	'''
	cloud = np.array(cloud)
	N = cloud.shape[0]
	keep_idxs = np.where(cloud[:,2].flatten()!=0)[0]
	return cloud[keep_idxs,:]

def f_addr_to_i(f):
	return struct.unpack('I', struct.pack('f', f))[0]
	
def i_addr_to_f(i):
	return struct.unpack('f', struct.pack('I', i))[0]

def rgb_to_pcl_float(r, g, b):
	i = r<<16 | g<<8 | b
	return i_addr_to_f(i)

def pcl_float_to_rgb(f):
	i = f_addr_to_i(f)
	r = i >> 16 & 0x0000ff
	g = i >> 8 & 0x0000ff
	b = i >> 0 & 0x0000ff
	return r,g,b

def test_icp():
	pts_model = np.load('canonical_model_cloud/binA.npy')
	pts_scene = np.load('canonical_model_cloud/binA_perturbed.npy')
	pts_model = pts_model[np.where(pts_model[:,2]!=0)]
	pts_scene = pts_scene[np.where(pts_scene[:,2]!=0)]
	pts_model = pts_model[::downsample_rate, :]/1000
	pts_scene = pts_scene[::downsample_rate, :]/1000
	# ax = render_3d_scatter(pts_scene.tolist())
	# ax.set_xlim([-500,500])
	# ax.set_ylim([-500,500])
	# ax.set_zlim([0,1200])
	# ax.view_init(elev=0, azim=90)
	# plt.show()
	print 'pts_model shape after downsampling:', pts_model.shape
	print 'pts_scene shape after downsampling:', pts_scene.shape
	print 'Making KD Tree'
	scene_tree = KDTree(pts_scene)
	print 'Done making KD Tree'
	R, t = icp.match(pts_model, scene_tree, iterations=20)
	print R, t

	pts_model_xformed = pts_model.dot(R.T) + t

	pts_model_xformed = np.hstack((pts_model_xformed, np.ones((pts_model_xformed.shape[0],1))*rgb_to_pcl_float(255,0,0)))
	pts_scene = np.hstack((pts_scene, np.ones((pts_scene.shape[0],1))*rgb_to_pcl_float(255,255,0)))

	ax = render_3d_scatter(np.vstack((pts_model_xformed, pts_scene)).tolist())
	ax.set_xlim([-0.5,0.5])
	ax.set_ylim([-0.5,0.5])
	ax.set_zlim([0,1.2])
	ax.view_init(elev=0, azim=90)
	plt.title('Shelf Overlay')

	# ax = render_3d_scatter(pts_model.tolist())
	# ax.set_xlim([-500,500])
	# ax.set_ylim([-500,500])
	# ax.set_zlim([0,1200])
	# ax.view_init(elev=0, azim=90)
	# plt.title('Model Original')
	# ax = render_3d_scatter(pts_model_xformed.tolist())
	# ax.set_xlim([-500,500])
	# ax.set_ylim([-500,500])
	# ax.set_zlim([0,1200])
	# ax.view_init(elev=0, azim=90)
	# plt.title('Model Xformed')
	# ax = render_3d_scatter(pts_scene.tolist())
	# ax.set_xlim([-500,500])
	# ax.set_ylim([-500,500])
	# ax.set_zlim([0,1200])
	# ax.view_init(elev=0, azim=90)
	# plt.title('Scene')
	plt.show()

def test_tote_subtraction():
	perceiver = Perceiver(False)
	scene_cloud = np.load('canonical_model_cloud/tote_with_objects.npy')
	model_cloud = np.load('canonical_model_cloud/tote.npy')
	clean_cloud = perceiver.get_current_tote_content_cloud(scene_cloud, fit=True)
	scene_cloud = remove_invalid_points(scene_cloud)[::downsample_rate,:]
	if scene_cloud.max() > 50:
		scene_cloud /= 1000
	model_cloud = remove_invalid_points(model_cloud)[::downsample_rate,:]
	if model_cloud.max() > 50:
		model_cloud /= 1000
	render_3d_scatter(model_cloud.tolist())
	render_3d_scatter(scene_cloud.tolist())
	render_3d_scatter(clean_cloud.tolist())
	plt.show()

def save_tote():
	perceiver = Perceiver()
	raw_input('Press to save empty tote model')
	perceiver.save_canonical_bin_point_cloud('_tote')
	raw_input('Press to save tote with object scene')
	perceiver.save_canonical_bin_point_cloud('_tote_with_objects')

def save_bin():
	perceiver = Perceiver()
	raw_input('Press to save bin_A model')
	perceiver.save_canonical_bin_point_cloud('A')
	raw_input('Press to save new bin_A model')
	perceiver.save_canonical_bin_point_cloud('A_perturbed')
	

if __name__ == '__main__':
	test_tote_subtraction()

