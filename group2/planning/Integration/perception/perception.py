
from __future__ import division
import numpy as np
from realsense_utils.client import RemoteCamera
from realsense_utils.common_utils import render_3d_scatter
from realsense_utils import colorize_point_cloud
# def colorize_point_cloud(cloud, color, depth_uv, rgb_table=None)
import struct, icp, sys
from scipy.spatial import KDTree
import matplotlib.pyplot as plt

sys.setrecursionlimit(10000)

class Perceiver:
	def __init__(self):
		self.camera = RemoteCamera('10.236.66.147', 30000)

	def save_canonical_bin_point_cloud(self, bin_letter):
		'''
		physical pre-condition: place the camera to the position viewing specified bin
		when the shelf is NOT perturbed. 

		bin_letter is one of ['A', 'B', ..., 'L']

		It will return None, but the model will be saved upon return
		'''
		_, cloud, _, _ = self.read_once()
		flattened_cloud = cloud.reshape(-1,3)
		np.save('cached_bin_cloud/bin'+bin_letter, flattened_cloud)
		print 'Successfully saved model for bin_'+bin_letter


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
		cached_bin_cloud = np.load('cached_bin_cloud/bin_'+bin_letter+'.npy')
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
		(private) read after flushing the queue
		'''
		for _ in xrange(100):
			self.camera.read()
		return self.camera.read()


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

if __name__ == '__main__':

	pts_model = np.load('cached_bin_cloud/binA.npy')
	pts_scene = np.load('cached_bin_cloud/binA_perturbed.npy')
	pts_model = pts_model[np.where(pts_model[:,2]!=0)]
	pts_scene = pts_scene[np.where(pts_scene[:,2]!=0)]
	pts_model = pts_model[::20, :]
	pts_scene = pts_scene[::20, :]
	# ax = render_3d_scatter(pts_scene.tolist())
	# ax.set_xlim([-500,500])
	# ax.set_ylim([-500,500])
	# ax.set_zlim([0,1200])
	# ax.view_init(elev=0, azim=90)
	# plt.show()
	print 'pts_model shape after downsampling:', pts_model.shape
	print 'pts_scene shape after downsampling:', pts_scene.shape
	# print 'Making KD Tree'
	# scene_tree = KDTree(pts_scene)
	# print 'Done making KD Tree'
	# R, t, _ = icp.match(pts_model, scene_tree)
	# print R, t

	R = np.array([[ 0.99192696, -0.00888253, -0.12649961], [ 0.02110102,  0.99519809,  0.09557982], [ 0.12504318, -0.09747747,  0.98735122]]) 

	t = np.array([[ 26.7398304,  -14.59224354,  42.7170067]])

	pts_model_xformed = pts_model.dot(R.T) + t

	pts_model_xformed = np.hstack((pts_model_xformed, np.ones((pts_model_xformed.shape[0],1))*rgb_to_pcl_float(255,0,0)))
	pts_scene = np.hstack((pts_scene, np.ones((pts_scene.shape[0],1))*rgb_to_pcl_float(255,255,0)))

	ax = render_3d_scatter(np.vstack((pts_model_xformed, pts_scene)).tolist())
	ax.set_xlim([-500,500])
	ax.set_ylim([-500,500])
	ax.set_zlim([0,1200])
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
	# perceiver = Perceiver()


	# raw_input('Press to save bin_A model')
	# perceiver.save_canonical_bin_point_cloud('A')
	# raw_input('Press to save new bin_A model')
	# perceiver.save_canonical_bin_point_cloud('B')
	# quit()
	# while True:
	# 	pass
