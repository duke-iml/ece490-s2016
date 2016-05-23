
from __future__ import division
import numpy as np
from realsense_utils.client import RemoteCamera
from realsense_utils.common_utils import render_3d_scatter
from realsense_utils import colorize_point_cloud
# def colorize_point_cloud(cloud, color, depth_uv, rgb_table=None)
import struct, icp, sys, threading
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
from klampt import se3

sys.setrecursionlimit(10000)
downsample_rate = 100

class CameraData:
	def __init__(self, color, cloud, depth_uv, color_uv):
		self.color = color
		self.cloud = cloud
		self.depth_uv = depth_uv
		self.color_uv = color_uv

class Perceiver:
	def __init__(self, camera_connect=True):
		# self.prefix = ''
		self.prefix = '/home/group3/ece490-s2016/apc2016/Integration/perception/'
		if camera_connect:
			self.camera = RemoteCamera('10.236.66.147', 30000)
			self.cd = None
			self.looping_thread = threading.Thread(target=self.read_loop_in_background)
			self.looping_thread.daemon = True
			self.looping_thread.start()

	def read_loop_in_background(self):
		while True:
			color, cloud, depth_uv, color_uv = self.camera.read()
			cd = CameraData(color, cloud, depth_uv, color_uv)
			self.cd = cd

	def save_canonical_bin_point_cloud(self, bin_letter):
		'''
		physical pre-condition: place the camera to the position viewing specified bin
		when the shelf is NOT perturbed. 

		bin_letter is one of ['A', 'B', ..., 'L']

		It will return None, but the model will be saved upon return
		'''
		_, cloud, _, _ = self.read_once(unit='meter', Nx3_cloud=True, clean=True)
		np.save(self.prefix+'cached_bin_cloud/bin_'+bin_letter, cloud)
		print 'Successfully saved model for bin_'+bin_letter

	def save_tote_cloud(self):
		_, cloud, _, _ = self.read_once(unit='meter', Nx3_cloud=True, clean=True)
		np.save(self.prefix+'cached_bin_cloud/tote', cloud)
		print 'Successfully saved model for tote'

	def subtract_model(self, model_cloud, scene_cloud=None, scene_tree=None, threshold=0.01):
		'''
		subtract model from scene. return indices of scene points that should be kept (beyond a threshold of any points in the model)
		'''
		assert scene_cloud is not None or scene_tree is not None, 'Either scene_model or scene_tree must be provided'
		model_cloud = np.array(model_cloud)
		if scene_tree is None:
			scene_cloud = np.array(scene_cloud)
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

	def get_current_tote_content_cloud(self, threshold=0.02, fit=False):
		'''
		subtract tote model from current point cloud of tote (with object in it)
		if fit is True, transform the model to align with the scene first. it will take some time
		return the point cloud of the remaining scene
		'''
		if scene_cloud is None:
			_, scene_cloud, _, _ = self.read_once(unit='meter', Nx3_cloud=True, clean=True)
		try:
			model_cloud = np.load(self.prefix+'cached_bin_cloud/tote.npy')
		except:
			print "Tote model cloud not found"
			return None
		'''
		need to find points in the scene that are within threshold distance of any points in the model
		'''
		scene_cloud = scene_cloud[::downsample_rate, :]
		print "Making scene cloud with %i points"%scene_cloud.shape[0]
		scene_tree = KDTree(scene_cloud)
		print "Done"
		if fit:
			R, t = icp.match(model_cloud, scene_tree)
			model_cloud = model_cloud.dot(R.T) + t
		keep_idxs = subtract_model(model_cloud, scene_tree=scene_tree)
		return scene_cloud[keep_idxs, :]

	def subtract_tote(self, scene_cloud=None, threshold=0.02, fit=False):
		'''
		subtract tote model from current point cloud of tote (with object in it)
		if fit is True, transform the model to align with the scene first. it will take some time
		return the point cloud of the remaining scene
		'''
		if scene_cloud is None:
			_, scene_cloud, _, _ = self.read_once(unit='meter', Nx3_cloud=True, clean=True)
		try:
			model_cloud = np.load(self.prefix+'cached_bin_cloud/tote.npy')
		except:
			print "Tote model cloud not found"
			return None
		'''
		need to find points in the scene that are within threshold distance of any points in the model
		'''
		scene_cloud = scene_cloud[::downsample_rate, :]
		print "Making scene cloud with %i points"%scene_cloud.shape[0]
		scene_tree = KDTree(scene_cloud)
		print "Done"
		if fit:
			R, t = icp.match(model_cloud, scene_tree)
			model_cloud = model_cloud.dot(R.T) + t
		keep_idxs = subtract_model(model_cloud, scene_tree=scene_tree)
		return scene_cloud[keep_idxs, :]

	def get_current_point_cloud(self, colorful=False, tolist=True):
		'''
		physical pre-condition: place the camera to the desired viewing angle. 

		return the point cloud of current camera viewing. 

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
		if tolist:
			cloud = cloud.tolist()
		return cloud

	def get_shelf_transformation(self, bin_letter, camera_to_global_R, camera_to_global_t):
		'''
		physical pre-condition: place the camera to the hard-coded position of viewing bin_A

		bin_letter is one of ['A', 'B', ..., 'L']

		return the shelf transformation as (R, t), where R is a column-major order flattened array of rotation matrix, 
		and t is an array of 3 numbers for transformation. 
		'''
		shelf_perturb_R, shelf_perturb_t = self.icp_get_bin_transform(bin_letter, camera_to_global_R, camera_to_global_t)
		self.shelf_perturb_R = shelf_perturb_R
		self.shelf_perturb_t = shelf_perturb_t
		return shelf_perturb_R, shelf_perturb_t

	def get_current_bin_content_cloud(self, bin_letter, camera_to_global_R, camera_to_global_t, colorful=True, fit=False, threshold=0.01):
		'''
		return a point cloud (either colored or not) of the current bin content. shelf is removed. 
		if fit is True, a new round of ICP is done on the model shelf
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
		scene_tree = KDTree(scene_cloud_xyz)
		model_cloud = self.load_model_bin_cloud(bin_letter, downsample=True)
		if fit:
			R, t = icp.match(model_cloud, scene_tree)
			model_cloud = model_cloud.dot(R.T) + t
		keep_idxs = subtract_model(model_cloud, scene_tree=scene_tree)
		bin_content_cloud = scene_cloud[keep_idxs, :]
		return bin_content_cloud

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

	def get_picking_position_for_stowing(self):
		'''
		physical pre-condition: place the camera to the hard-coded tote-viewing position. 

		return (x,y) coordinate of a normalized position (in the range of 0 to 1) in the 
		2D tote plane that has something to suck. Coordinate system convention TBD. 
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

	def icp_get_bin_transform(self, bin_letter, camera_to_global_R, camera_to_global_t):
		'''
		(private) get the current bin transform relative to canonical position for the specified bin
		bin_letter is one of ['A', 'B', ..., 'L'] and is used to load the pre-computed shelf cloud

		return (R, t), where R is a column-major order flattened array of rotation matrix, 
		and t is an array of 3 numbers for transformation. 
		'''
		camera_to_global_R = np.array([ [camera_to_global_R[0], camera_to_global_R[3], camera_to_global_R[6]] , [camera_to_global_R[1], camera_to_global_R[4], camera_to_global_R[7]] , [camera_to_global_R[2], camera_to_global_R[5], camera_to_global_R[8]] ])
		camera_to_global_t = np.array(camera_to_global_t).flatten()
		_, scene_cloud, _, _ = self.read_once(unit='meter', Nx3_cloud=True, clean=True)
		model_cloud = self.load_model_bin_cloud(bin_letter, downsample=True)
		if model_cloud is None:
			return None
		scene_cloud = scene_cloud[::downsample_rate,:]
		model_cloud_xformed = model_cloud.dot(camera_to_global_R.T) + camera_to_global_t
		scene_cloud_xformed = scene_cloud.dot(camera_to_global_R.T) + camera_to_global_t
		np.save(self.prefix+'cached_bin_cloud/model_cloud_xformed.npy', model_cloud_xformed)
		np.save(self.prefix+'cached_bin_cloud/scene_cloud_xformed.npy', scene_cloud_xformed)
		scene_tree = KDTree(scene_cloud_xformed)
		relative_R, relative_t = icp.match(model_cloud_xformed, scene_tree)
		relative_R = list(relative_R.flatten(order='F'))
		relative_t = list(relative_t.flat)
		return relative_R, relative_t
		
	def subtract(self, bin_letter):
		'''
		(private) 
		'''
		raise NotImplemented

	def load_model_bin_cloud(self, bin_letter, downsample=False):
		try:
			model_cloud = np.load(self.prefix+'cached_bin_cloud/bin_'+bin_letter+'.npy')
		except:
			print 'Cannot find bin model for bin_%s at %scached_bin_cloud/bin_%s.npy '%(bin_letter, self.prefix, bin_letter)
			return None
		if downsample:
			model_cloud = model_cloud[::downsample, :]
		return model_cloud

	def read_once(self, unit='meter', Nx3_cloud=False, clean=None):
		'''
		(private) (OLD) read after flushing the queue
		(CURRENT) read from CameraData object
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
				keep_idxs = keep_idxs = np.where(cloud[:,2].flatten()!=0)[0]
				cloud = cloud[keep_idxs,:]
		return color, cloud, depth_uv, color_uv

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
	pts_model = np.load('cached_bin_cloud/binA.npy')
	pts_scene = np.load('cached_bin_cloud/binA_perturbed.npy')
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
	scene_cloud = np.load('cached_bin_cloud/tote_with_objects.npy')
	model_cloud = np.load('cached_bin_cloud/tote.npy')
	clean_cloud = perceiver.subtract_tote(scene_cloud, fit=True)
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
