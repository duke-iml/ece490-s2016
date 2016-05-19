
from __future__ import division
import numpy as np
from realsense_utils.client import RemoteCamera
from realsense_utils.common_utils import render_3d_scatter
from realsense_utils import colorize_point_cloud
# def colorize_point_cloud(cloud, color, depth_uv, rgb_table=None)
import struct, icp, sys, threading
from scipy.spatial import KDTree
import matplotlib.pyplot as plt

sys.setrecursionlimit(10000)

class CameraData:
	def __init__(self, color, cloud, depth_uv, color_uv):
		self.color = color
		self.cloud = cloud
		self.depth_uv = depth_uv
		self.color_uv = color_uv

class Perceiver:
	def __init__(self):
		self.camera = RemoteCamera('10.236.66.147', 30000)
		self.cd = None
		self.looping_thread = threading.Thread(target=self.read_loop_in_background)
		self.looping_thread.daemon = True
		self.looping_thread.start()
		self.prefix = '/home/group3/ece490-s2016/apc2016/Integration/perception/'

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
		_, cloud, _, _ = self.read_once()
		flattened_cloud = cloud.reshape(-1,3)
		np.save(self.prefix+'cached_bin_cloud/bin'+bin_letter, flattened_cloud)
		print 'Successfully saved model for bin_'+bin_letter

	def save_tote_cloud(self):
		_, cloud, _, _ = self.read_once()
		flattened_cloud = cloud.reshape(-1,3)
		flattened_cloud = flattened_cloud / 1000
		np.save(self.prefix+'cached_bin_cloud/tote', flattened_cloud)
		print 'Successfully saved model for tote'

	def subtract_tote(self, threshold=0.01, fit=False):
		'''
		subtract tote model from current point cloud of tote (with object in it)
		if fit is True, transform the model to align with the scene first. it will take some time
		return the point cloud of the remaining scene
		'''
		_, scene_cloud, _, _ = self.read_once()
		try:
			model_cloud = np.load(self.prefix+'cached_bin_cloud/tote.npy')
		except:
			print "Tote model cloud not found"
			return None
		'''
		need to find points in the scene that are within threshold distance of any points in the model
		'''
		if model_cloud.max()>50:
			model_cloud = model_cloud[::100, :] / 1000
		else:
			model_cloud = model_cloud[::100, :]
		scene_cloud = scene_cloud[::100, :] / 1000
		scene_tree = KDTree(scene_cloud)
		if fit:
			R, t, _ = icp.match(model_cloud, scene_tree)
			model_cloud = model_cloud.dot(R.T) + t
		idxs = scene_tree.query_ball_point(model_cloud, threshold)
		idxs = [i for i in group for group in idxs] # flattened index
		discard_idxs = set(idxs)
		all_idxs = set(range(scene_cloud.shape[0]))
		keep_idxs = all_idxs - discard_idxs
		return scene_cloud[list(keep_idxs), :]

	def get_current_point_cloud(self, colorful=True):
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
		color, cloud, depth_uv, color_uv = self.camera.read()
		if cloud is None:
			return None
		if colorful:
			if color is None or depth_uv is None:
				return None
			return colorize_point_cloud(cloud, color, depth_uv)
		else:
			return cloud

	def get_shelf_transformation(self, bin_letter):
		'''
		physical pre-condition: place the camera to the hard-coded position of viewing bin_A

		bin_letter is one of ['A', 'B', ..., 'L']

		return the shelf transformation as (R, t), where R is a column-major order flattened array of rotation matrix, 
		and t is an array of 3 numbers for transformation. 
		'''
		raise self.icp_get_bin_transform(bin_letter)

	def get_position_of_item_in_bin(self, item_name, return_normal=False):
		'''
		physical pre-condition: place the camera to the calibrated bin-viewing position.
		calibrated position is the IK-solved position that is constant ***relative to the shelf***

		***setting item_name to None*** suggests that there is only one item in the bin. Thus, no vision
		algorithm is used. 

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

	def icp_get_bin_transform(self, bin_letter):
		'''
		(private) get the current bin transform relative to canonical position for the specified bin
		bin_letter is one of ['A', 'B', ..., 'L'] and is used to load the pre-computed shelf cloud

		return (R, t), where R is a column-major order flattened array of rotation matrix, 
		and t is an array of 3 numbers for transformation. 
		'''
		color, cloud, depth_uv, color_uv = self.camera.read()
		try:
			cached_bin_cloud = np.load(self.prefix+'cached_bin_cloud/bin_'+bin_letter+'.npy')
		except:
			print 'Cannot find bin model for bin_%s at %scached_bin_cloud/bin_%s.npy '%(bin_letter, self.prefix, bin_letter)
			return None
		cloud = cloud / 1000
		if cached_bin_cloud.max() > 50:
			cached_bin_cloud = cached_bin_cloud / 1000
		scene_tree = KDTree(cloud.reshape(-1,3))
		R, t, _ = icp.match(cached_bin_cloud, scene_tree)
		return list(R.flatten(order='F')), t


	def subtract(self, bin_letter):
		'''
		(private) 
		'''
		raise NotImplemented

	def read_once(self):
		'''
		(private) (OLD) read after flushing the queue
		(CURRENT) read from CameraData object
		'''
		cd = self.cd
		return cd.color, cd.cloud, cd.depth_uv, cd.color_uv


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
	pts_model = pts_model[::100, :]/1000
	pts_scene = pts_scene[::100, :]/1000
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
	R, t, _ = icp.match(pts_model, scene_tree, iterations=20)
	print "R:", R
	print "t:", t

	print "column-major R:", ', '.join(map(str, R.T.flatten()))
	print "t:", ', '.join(map(str, t.flatten()))

	print "=================Inverse================="
	T = np.vstack((np.hstack((R, t.reshape(3,1))), np.array([0,0,0,1]).reshape(1,4)))
	T = np.linalg.inv(T)
	print "column-major R_inv:", ', '.join(map(str, T[0:3,0:3].T.flatten()))
	print "t_inv:", ', '.join(map(str, T[0:3, 3].flatten()))
	quit()

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
	raise NotImplemented

def save_tote():
	perceiver = Perceiver()
	raw_input('Press to save bin_A model')
	perceiver.save_canonical_bin_point_cloud('A')
	raw_input('Press to save new bin_A model')
	perceiver.save_canonical_bin_point_cloud('A_perturbed')
	quit()
	while True:
		pass

def save_bin():
	perceiver = Perceiver()
	raw_input('Press to save bin_A model')
	perceiver.save_canonical_bin_point_cloud('A')
	raw_input('Press to save new bin_A model')
	perceiver.save_canonical_bin_point_cloud('A_perturbed')
	quit()
	

if __name__ == '__main__':
	test_icp()
