
from __future__ import division

FAST_NN = False
KMEAN_OVER_SEGMENT = False
COLOR_WEIGHING = False

import numpy as np
from realsense_utils.client import RemoteCamera
from realsense_utils.common_utils import render_3d_scatter
from realsense_utils import colorize_point_cloud
# def colorize_point_cloud(cloud, color, depth_uv, rgb_table=None)
import struct, icp, sys, threading, colorsys, time, traceback
from scipy.spatial import KDTree
if FAST_NN:
	print 'WARNING: Using Grid Nearest Neighbor'
	from grid_nn import GridPointLocation as KDTree
from scipy.sparse import dok_matrix
from scipy.stats import gaussian_kde
import matplotlib.pyplot as plt
from klampt import se3
from segmentation import distance_label
from scipy.sparse.csgraph import connected_components
# from sklearn.cluster import KMeans

sys.setrecursionlimit(10000)
downsample_rate = 50

if FAST_NN:
	initial_threshold = 0.1
else:
	initial_threshold = None

RIGHT_CAMERA_IP = '169.254.65.112'
LEFT_CAMERA_IP = '169.254.97.254'

class CameraData:
	def __init__(self, color, cloud, depth_uv, color_uv):
		self.color = color
		self.cloud = cloud
		self.depth_uv = depth_uv
		self.color_uv = color_uv

class Perceiver(object):
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
	'''

	def __init__(self):
		# self.prefix = ''
		self.prefix = '/home/hskim/Documents/GitHub/ece490-s2016/apc2016/Integration/perception/'
		self.shelf_perturb_R = np.eye(3)
		self.shelf_perturb_t = np.array([0,0,0])
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
		print 'subtract_model: keep %i points out of %i points'%(len(keep_idxs), len(all_idxs))
		return list(keep_idxs)

	def get_current_bin_content_cloud(self, bin_letter, cur_camera_R, cur_camera_t, limb, colorful=True, fit=False, threshold=0.01,
		crop=False, bin_bound=None, perturb_xform=None):
		'''
		return a point cloud (either colored or not) of the current bin content in the global frame. shelf is removed.
		if fit is True, a new round of ICP is done on the model shelf
		cur_camera_R and cur_camera_t are the current camera transformation in world
		'''
		color, scene_cloud, depth_uv, _ = self.read_once(limb, unit='meter', Nx3_cloud=False)
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
		scene_cloud_xyz = scene_cloud_xyz.dot(cur_camera_R.T)+cur_camera_t # colorless cloud
		scene_cloud[:, 0:3] = scene_cloud_xyz # transform scene_cloud
		scene_tree = KDTree(scene_cloud_xyz)
		model_cloud = self.load_model_bin_cloud(bin_letter, limb, downsample=True)
		model_xform_R, model_xform_t = self.load_R_t(self.get_bin_viewing_camera_xform_path(bin_letter, limb), nparray=True)
		model_cloud = model_cloud.dot(model_xform_R.T) + model_xform_t # apply saved transformation
		assert (self.shelf_perturb_R is not None) and (self.shelf_perturb_t is not None), 'Shelf perturbation has not been calibrated'
		model_cloud = model_cloud.dot(self.shelf_perturb_R.T) + self.shelf_perturb_t # apply perturbation transformation found by ICP
		if fit:
			R, t = icp.match(model_cloud, scene_tree, initial_threshold=initial_threshold) # initial threshold of 1 meter
			model_cloud = model_cloud.dot(R.T) + t
		keep_idxs = self.subtract_model(model_cloud, scene_tree=scene_tree, threshold=threshold)
		bin_content_cloud = scene_cloud[keep_idxs, :]
		if crop:
			if perturb_xform is None:
				perturb_xform = (list(self.shelf_perturb_R.T.flat), list(self.shelf_perturb_t.flat))
			bin_content_cloud = self.crop_cloud(bin_content_cloud, bin_bound, perturb_xform)
		return bin_content_cloud

	def crop_cloud(self, cloud, bin_bound, perturb_xform):
		'''
		crop the cloud to keep only those that are in the bin_bound transformed by perturb_xform

		bin_bound is a list of two lists of three numbers: [[xmin, ymin, zmin], [xmax, ymax, zmax]]

		solution: rather than transforming the bin_bound and test for inside-ness of skewed cube, inverse transform
		the cloud and test for inside-ness of axis-parallel cube.
		'''
		assert len(cloud.shape)==2, 'cloud shape must be N x 3 or N x 4 (with color)'
		if bin_bound is None:
			return cloud

		min_list, max_list = bin_bound
		xmin, ymin, zmin = min_list
		xmax, ymax, zmax = max_list

		if perturb_xform is None:
			perturb_xform = [[1,0,0,0,1,0,0,0,1],[0,0,0]] # identity xform
		cloud_xyz = cloud[:,0:3]
		inv_perturb_R, inv_perturb_t = se3.inv(perturb_xform)
		inv_perturb_R = np.array(inv_perturb_R).reshape(3,3).T
		inv_perturb_t = np.array(inv_perturb_t)
		cloud_xyz_inv_xformed = cloud_xyz.dot(inv_perturb_R.T) + inv_perturb_t
		xmin_flag = cloud_xyz_inv_xformed[:,0] >= xmin
		xmax_flag = cloud_xyz_inv_xformed[:,0] <= xmax
		ymin_flag = cloud_xyz_inv_xformed[:,1] >= ymin
		ymax_flag = cloud_xyz_inv_xformed[:,1] <= ymax
		zmin_flag = cloud_xyz_inv_xformed[:,2] >= zmin
		zmax_flag = cloud_xyz_inv_xformed[:,2] <= zmax
		# print cloud_xyz_inv_xformed.mean(axis=0)
		# print xmin, ymin, zmin
		# print xmax, ymax, zmax
		in_flag = reduce(np.logical_and, [xmin_flag, xmax_flag, ymin_flag, ymax_flag, zmin_flag, zmax_flag])
		cropped_cloud = cloud[in_flag, :]
		print 'Cropped cloud from %i points to %i points'%(cloud.shape[0], cropped_cloud.shape[0])
		return cropped_cloud

	def segment_cloud(self, bin_content_cloud, threshold=0.01):
		N = bin_content_cloud.shape[0]
		cloud_xyz = bin_content_cloud[:,0:3]
		print 'Begin making sparse adjacency matrix...'
		adj_matrix = dok_matrix((N, N), dtype='bool_')
		for i in xrange(N):
			for j in xrange(i+1, N):
				if np.linalg.norm(cloud_xyz[i,:]-cloud_xyz[j,:])<threshold:
					adj_matrix[i,j] = 1
		adj_matrix = adj_matrix + adj_matrix.T + np.eye(N)
		print 'Done. Start computing connected component... '
		n_components, labels = connected_components(adj_matrix, directed=False, return_labels=True)
		print 'Done. Found %d components. '%n_components
		return labels, n_components

	def crop_and_segment(self, bin_content_cloud, bin_bound=None, perturb_xform=None, threshold=0.01):
		cropped_cloud = self.crop_cloud(bin_content_cloud, bin_bound=bin_bound, perturb_xform=perturb_xform)
		labels, n_components = self.segment_cloud(cropped_cloud, threshold=threshold)
		return cropped_cloud, labels, n_components

	def get_current_bin_content_cc_cloud(self, bin_letter, cur_camera_R, cur_camera_t, limb, crop=False, bin_bound=None,
		perturb_xform=None, fit=False, shelf_subtraction_threshold=0.01, cc_threshold=0.02, n=None):
		bin_content_cloud = self.get_current_bin_content_cloud(bin_letter, cur_camera_R, cur_camera_t, limb,
			colorful=False, fit=fit, threshold=shelf_subtraction_threshold, crop=crop, bin_bound=bin_bound, perturb_xform=perturb_xform)
		labels, n_components = self.segment_cloud(bin_content_cloud, cc_threshold)
		sizes = [len(np.where(labels==i)[0]) for i in xrange(n_components)]
		sorted_idxs = [ i for _,i in sorted([(v,i) for i,v in enumerate(sizes)]) ][::-1]
		if n is None:
			n = float('inf')
		cc_cloud = np.zeros((0,4))
		for i in xrange(min(n, n_components)):
			cur_idx = sorted_idxs[i]
			h = i / min(n, n_components)
			r, g, b = map(lambda x: int(x*255), colorsys.hsv_to_rgb(h, 1, 1))
			idxs = np.where(labels==cur_idx)[0]
			print 'CC %i has %i points'%(i, len(idxs))
			pcl_float = rgb_to_pcl_float(r, g, b)
			cur_cc_cloud = np.hstack( [ bin_content_cloud[idxs,:], np.ones((len(idxs), 1))*pcl_float ] )
			cc_cloud = np.vstack( [cc_cloud, cur_cc_cloud] )
		return cc_cloud

	def get_picking_position_for_single_item_bin(self, bin_letter, cur_camera_R, cur_camera_t, limb,
		colorful=True, fit=False, threshold=0.01, crop=False, bin_bound=None, perturb_xform=None,
		return_bin_content_cloud=False, return_dir_suggestion=False):
		'''
		physical pre-condition: place the camera to the hard-coded tote-viewing position.

		return (x,y,z) coordinate in global frame of position that has something to suck.
		'''
		bin_content_cloud = self.get_current_bin_content_cloud(bin_letter, cur_camera_R, cur_camera_t, limb,
			colorful=colorful, fit=fit, threshold=threshold, crop=crop, bin_bound=bin_bound, perturb_xform=perturb_xform)
		pos = self.find_max_density_3d(bin_content_cloud)
		bin_content_tree = KDTree(bin_content_cloud[:,0:3])
		neighbor_idxs = bin_content_tree.query_ball_point(pos, r=0.002)
		# print 'neighbor_idxs is of type', neighbor_idxs.__class__
		return_list = [pos]
		if return_bin_content_cloud:
			return_list += [bin_content_cloud]
		if return_dir_suggestion:
			assert bin_bound is not None, 'bin_bound must be given to give direction suggestion'
			cloud_zs = list(bin_content_cloud[:,2].flat)
			z_top = sorted(cloud_zs)[int(len(cloud_zs)*0.8)]
			zmin, zmax = bin_bound[0][2], bin_bound[1][2]
			if z_top<zmin+0.5*(zmax-zmin):
				direction = 'from top'
			else:
				ymin, ymax = bin_bound[0][1], bin_bound[1][1]
				pos_y = pos[1]
				y_center = (ymin + ymax ) / 2
				if pos_y < y_center:
					direction = 'from left'
				else:
					direction = 'from right'
			return_list += [direction]
		if len(return_list)==1:
			return return_list[0]
		else:
			return return_list

	def cc_label_to_list(self, labels, n_components, cloud, max_num_comp=None):
		'''
		convert a cloud that is already being cc-ed to list of separate clouds
		'''
		if max_num_comp is None:
			max_num_comp = float('inf')
		sizes = [len(np.where(labels==i)[0]) for i in xrange(n_components)]
		sorted_idxs = [ i for _,i in sorted([(v,i) for i,v in enumerate(sizes)]) ][::-1]
		sorted_idxs = sorted_idxs[0:min(n_components, max_num_comp)]
		cc_cloud_list = [cloud[np.where(labels==i)[0], :] for i in sorted_idxs]
		return cc_cloud_list

	def kmean_over_segment(self, cc_cloud_list, target_num_comp):
		'''
		recursively segement the largest cluster into 2 until there are target_num_comp clouds
		'''
		print cc_cloud_list.__class__
		cloud = cc_cloud_list[0]
		cc_cloud_list = cc_cloud_list[1:]
		cloud_xyz = cloud[0:3]
		kmeans = KMeans(n_clusters=2)
		labels = kmeans.fit_predict(cloud_xyz)
		cloud0 = cloud[np.where(labels==0)[0], :]
		cloud1 = cloud[np.where(labels==1)[0], :]
		cc_cloud_list += [cloud0, cloud1]
		cc_cloud_list = sorted(cc_cloud_list, key=lambda x:-x.shape[0])
		if len(cc_cloud_list)<target_num_comp:
			return self.kmean_over_segment(cc_cloud_list, target_num_comp)
		else:
			return cc_cloud_list

	def get_picking_position_for_multi_item_bin(self, target_item, possible_items, bin_letter, cur_camera_R, cur_camera_t, limb,
		colorful=True, fit=False, shelf_subtraction_threshold=0.01, cc_threshold=0.02, crop=False, bin_bound=None, perturb_xform=None,
		return_bin_content_cloud=False, return_dir_suggestion=False):
		'''
		return (x,y,z) coordinate in global frame of position that has something to suck.
		'''
		bin_content_cloud = self.get_current_bin_content_cloud(bin_letter, cur_camera_R, cur_camera_t, limb,
			colorful=colorful, fit=fit, threshold=shelf_subtraction_threshold, crop=crop, bin_bound=bin_bound, perturb_xform=perturb_xform)
		assert bin_content_cloud.shape[1]==4, 'Color channel seems to be missing'
		labels, n_components = self.segment_cloud(bin_content_cloud, cc_threshold)

		'''
		Better way: return cloud that maximizes f-g,
		where f is correlation with target object, and g is max correlation with non-target object.
		'''
		cc_cloud_list = self.cc_label_to_list(labels, n_components, bin_content_cloud, max_num_comp=len(possible_items)+1)
		cc_cloud_list = sorted(cc_cloud_list, key=lambda x:-x.shape[0]) # this should do nothing because cc_label_to_list has sorted, but just in case...
		if KMEAN_OVER_SEGMENT and len(possible_items)>=4:
			cc_cloud_list = self.kmean_over_segment(cc_cloud_list, len(possible_items)*2+1)
		best_score = None
		best_cloud = None
		for i, cur_cloud in enumerate(cc_cloud_list):
			cur_uv_hist = self.make_uv_hist_from_cloud(cur_cloud)
			scoring_func = lambda item: self.calculate_uv_matching(cur_uv_hist, self.load_uv_hist_for_item(item), suppress_center=True)
			f_score = scoring_func(target_item)
			g_score = max([scoring_func(non_target_item) for non_target_item in possible_items if non_target_item!=target_item])
			print f_score
			print g_score
			score_diff = f_score - g_score
			print 'CC %i with %i points has score diff of %f'%(i, len(np.where(labels==i)[0]), score_diff)
			if score_diff>0 and (best_score is None or score_diff > best_score):
				best_score = score_diff
				best_cloud = cur_cloud
		if best_cloud is None:
			print 'No positive score difference for item %s among items %s'%(target_item, str(possible_items))
			return None
		if COLOR_WEIGHING:
			pos = self.find_max_density_3d(best_cloud, with_color=True, uv_hist=self.load_uv_hist_for_item(item))
		else:
			pos = self.find_max_density_3d(best_cloud)
		return_list = [pos]
		if return_bin_content_cloud:
			return_list += [best_cloud]
		if return_dir_suggestion:
			assert bin_bound is not None, 'bin_bound must be given to give direction suggestion'
			zmin, zmax = bin_bound[0][2], bin_bound[1][2]
			z_cloud_max = max(list(best_cloud[:,2].flat))
			if z_cloud_max<zmin+0.6*(zmax-zmin):
				direction = 'from top'
			else:
				ymin, ymax = bin_bound[0][1], bin_bound[1][1]
				pos_y = pos[1]
				y_center = (ymin + ymax ) / 2
				if pos_y < y_center:
					direction = 'from left'
				else:
					direction = 'from right'
			return_list += [direction]
		if len(return_list)==1:
			return return_list[0]
		else:
			return return_list

	def long_name_to_short(self, name):
		'''
		translate official APC item name to short name stored in uv_hist
		'''
		short_names = ['baseball', 'bear', 'bones', 'bowl', 'brush', 'bulb', 'bunny',
		'coffee', 'cord', 'crayola24', 'cup', 'curtain', 'dumbbell', 'dvd', 'eggs',
		'expo', 'gloves', 'glucose', 'glue', 'glue_stick', 'hooks', 'index_card',
		'joke', 'mailer', 'paper_towel', 'pencil', 'plug', 'rolodex', 'scissor', 'shirt',
		'soap', 'socks', 'stems', 'tape', 'tissue', 'toothbrush_green', 'toothbrush_red',
		'util_brush', 'water']
		long_names = ['rawlings_baseball', 'cloud_b_plush_bear', 'barkely_hide_bones', 'platinum_pets_dog_bowl',
		'dr_browns_bottle_brush', 'soft_white_lightbulb', 'i_am_a_bunny_book', 'folgers_classic_roast_coffee', 'woods_extension_cord',
		'crayola_24_ct', 'easter_turtle_sippy_cup', 'peva_shower_curtain_liner', 'fitness_gear_3lb_dumbbell', 'jane_eyre_dvd',
		'kyjen_squeakin_eggs_plush_puppies', 'expo_dry_erase_board_eraser', 'womens_knit_gloves', 'up_glucose_bottle', 'elmers_washable_no_run_school_glue',
		'cool_shot_glue_sticks', 'command_hooks', 'staples_index_cards', 'laugh_out_loud_joke_book', 'scotch_bubble_mailer',
		'kleenex_paper_towels', 'ticonderoga_12_pencils', 'safety_first_outlet_plugs', 'rolodex_jumbo_pencil_cup',
		'fiskars_scissors_red', 'cherokee_easy_tee_shirt', 'dove_beauty_bar', 'hanes_tube_socks', 'creativity_chenille_stems',
		'scotch_duct_tape', 'kleenex_tissue_box', 'oral_b_toothbrush_green', 'oral_b_toothbrush_red', 'clorox_utility_brush', 'dasani_water_bottle']
		assert name in (long_names+short_names), 'name: %s must be either long form or short form'%name
		if name in long_names:
			name = short_names[long_names.index(name)]
		return name

	def get_picking_position_for_bin(self, target_item, possible_items, bin_letter, cur_camera_R, cur_camera_t, limb,
		colorful=True, fit=False, shelf_subtraction_threshold=0.01, cc_threshold=0.025, crop=False, bin_bound=None, perturb_xform=None,
		return_bin_content_cloud=False, return_dir_suggestion=False):
		'''
		perception API for picking from bin
		will return None if perception failed (e.g. not able to find specified object)
		'''
		try:
			assert target_item in possible_items, 'target item must be in the list of possible items'
			if len(possible_items)==1: # single object localization
				return self.get_picking_position_for_single_item_bin(bin_letter, cur_camera_R, cur_camera_t, limb,
					colorful=colorful, fit=fit, threshold=shelf_subtraction_threshold, crop=crop, bin_bound=bin_bound,
					perturb_xform=perturb_xform, return_bin_content_cloud=return_bin_content_cloud,
					return_dir_suggestion=return_dir_suggestion)
			else:
				target_item = self.long_name_to_short(target_item)
				possible_items = map(self.long_name_to_short, possible_items)
				return self.get_picking_position_for_multi_item_bin(target_item, possible_items, bin_letter, cur_camera_R, cur_camera_t, limb,
					colorful=colorful, fit=fit, shelf_subtraction_threshold=shelf_subtraction_threshold, cc_threshold=cc_threshold,
					crop=crop, bin_bound=bin_bound, perturb_xform=perturb_xform, return_bin_content_cloud=return_bin_content_cloud,
					return_dir_suggestion=return_dir_suggestion)
		except:
			print 'PERCEPTION EXCEPTION!!!!!!!!!!!!!'
			traceback.print_exc()
			return None


	def rgb_to_uv(self, rgb):
		r, g, b = rgb
		u = 128 -.168736*r -.331364*g + .5*b
		v = 128 +.5*r - .418688*g - .081312*b
		return u, v

	def make_uv_hist_from_cloud(self, cloud, binsize=4):
		assert cloud.shape[1]==4, 'cloud must have size N x 4'
		colors = cloud[:,3]
		rgbs = map(pcl_float_to_rgb, colors.tolist())
		hist = np.zeros((int(256/binsize), int(256/binsize)))
		vals = map(self.rgb_to_uv, rgbs)
		try:
			vals = vals.tolist()
		except:
			pass
		for val in vals:
			hist[int(val[0]/binsize), int(val[1]/binsize)] += 1
		hist = hist / len(vals)
		return hist

	def suppress(self, uv_hist):
		side_len = uv_hist.shape[0]
		center = side_len / 2 - 0.5
		for i in xrange(side_len):
			for j in xrange(side_len):
				if (i - center)**2+(j-center)**2<16:
					uv_hist[i,j] = 0

	def calculate_uv_matching(self, hist1, hist2, suppress_center=False):
		'''
		size of intersection of normalized histogram
		if suppress_center is True, the center of the histogram (corresponding to mostly black region) is erased to 0
		'''
		assert hist1.shape==hist2.shape, 'Shape not equal!'
		if suppress_center:
			self.suppress(hist1)
			self.suppress(hist2)
		sum1 = hist1.sum()
		sum2 = hist2.sum()
		hist1 = hist1 / sum1
		hist2 = hist2 / sum2
		min_hist = np.minimum(hist1, hist2)
		return min_hist.sum()

	def load_uv_hist_for_item(self, item):
		filename = self.prefix + 'uv_hist/%s.npy'%item
		try:
			hist = np.load(filename)
			return hist
		except:
			print 'Cannot find uv hist for %s at %s'%(item, filename)
			return None

	def get_current_point_cloud(self, cur_camera_R, cur_camera_t, limb, colorful=False, tolist=True):
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
		color, cloud, depth_uv, _ = self.read_once(limb, unit='meter', Nx3_cloud=False)
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
		cloud_xyz = cloud[:,0:3]
		cloud_xyz = cloud_xyz.dot(cur_camera_R.T) + cur_camera_t
		cloud[:,0:3] = cloud_xyz
		if tolist:
			cloud = cloud.tolist()
		return cloud

	def crop_tote_cloud(self, cloud):
		return cloud

	def get_picking_position_for_stowing(self, cur_camera_R, cur_camera_t, limb, fit=False, return_tote_content_cloud=False):
		'''
		physical pre-condition: place the camera to the hard-coded tote-viewing position.

		return (x,y) coordinate in global frame of position that has something to suck.
		'''
		tote_content_cloud = self.get_current_tote_content_cloud(cur_camera_R, cur_camera_t, limb, fit=fit)
		tote_content_cloud = self.crop_tote_cloud(tote_content_cloud)
		return self.find_max_density_xy(tote_content_cloud, return_tote_content_cloud)

	def get_candidate_picking_positions_for_stowing(self, cur_camera_R, cur_camera_t, limb, fit=False, return_tote_content_cloud=False, avoid_poss=[], avoid_radius=0.03):
		'''
		physical pre-condition: place the camera to the hard-coded tote-viewing position.
		avoid_poss is a list of list of two numbers, specifying list of points to avoid

		return a list of (x,y) coordinate in global frame of position that has something to suck.
		'''
		avoid_poss = [[x,y] for (x,y) in avoid_poss]
		print "avoid_poss", avoid_poss
		tote_content_cloud = self.get_current_tote_content_cloud(cur_camera_R, cur_camera_t, limb, fit=fit)
		tote_content_cloud = self.crop_tote_cloud(tote_content_cloud)
		xs = tote_content_cloud[:,0]
		ys = tote_content_cloud[:,1]
		zs = tote_content_cloud[:,2]
		xs_sorted = sorted(list(xs.flat))
		ys_sorted = sorted(list(ys.flat))
		N = len(xs_sorted)
		if N==0:
			print 'No points in cloud! '
			return None
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
		all_idxs = self.get_all_local_maxima(densities)
		# all_idxs = sorted(all_idxs, key=lambda idx:-densities[idx[0], idx[1]]) # sort in descending order of density
		all_locs = [[x_grid[x_idx,y_idx], y_grid[x_idx,y_idx]] for x_idx,y_idx in all_idxs]
		safe_locs = []
		for loc in all_locs:
			keep = True
			for avoid_pos in avoid_poss:
				# print 'loc:', loc
				# print 'avoid_pos', avoid_pos
				# print 'dist', (loc[0]-avoid_pos[0])**2+(loc[0]-avoid_pos[0])**2
				if (loc[0]-avoid_pos[0])**2+(loc[1]-avoid_pos[1])**2<=avoid_radius**2:
					# print 'avoided'
					keep = False
				break
			if keep:
				avoid_poss.append(loc)
				safe_locs.append(loc)
		print '%i safe locs remain from %i total points'%(len(safe_locs), len(all_locs))
		if not return_tote_content_cloud:
			return safe_locs[0:5], max(list(zs.flat))
		else:
			return safe_locs[0:5], tote_content_cloud, max(list(zs.flat))

	def remove_local_max(self, mat, x, y):
		h, w = mat.shape
		if x<=0 or x>=h-1 or y<=0 or y>=w-1:
			return
		if mat[x,y]==-1:
			return
		if mat[x,y]<max(mat[x-1,y], mat[x+1,y], mat[x,y-1], mat[x,y+1]):
			return
		mat[x,y] = -1
		self.remove_local_max(mat, x-1, y)
		self.remove_local_max(mat, x+1, y)
		self.remove_local_max(mat, x, y-1)
		self.remove_local_max(mat, x, y+1)

	def get_all_local_maxima(self, mat):
		h, w = mat.shape
		new_mat = np.zeros((h+2, w+2))
		new_mat[1:h+1, 1:w+1] = mat
		mat = new_mat
		all_locs = []
		while (mat==-1).sum()!=(mat.shape[0]-2)*(mat.shape[1]-2):
			max_loc = np.unravel_index(mat.argmax(), mat.shape)
			all_locs.append([max_loc[0]-1, max_loc[1]-1])
			self.remove_local_max(mat, *max_loc)
		return all_locs

	def get_current_tote_content_cloud(self, cur_camera_R, cur_camera_t, limb, colorful=True, threshold=0.02, fit=False):
		'''
		subtract tote model from current point cloud of tote (with object in it)
		if fit is True, transform the model to align with the scene first. it will take some time
		return the point cloud of the remaining scene in global frame
		'''
		color, scene_cloud, depth_uv, _ = self.read_once(limb, unit='meter', Nx3_cloud=False)
		if colorful:
			scene_cloud = colorize_point_cloud(scene_cloud, color, depth_uv).reshape(-1, 4)
		else:
			scene_cloud = scene_cloud.reshape(-1, 3)
		keep_idxs = np.where(scene_cloud[:,2].flatten()!=0)[0]
		scene_cloud = scene_cloud[keep_idxs, :]
		scene_cloud = scene_cloud[::downsample_rate, :]
		scene_cloud_xyz = scene_cloud[:, 0:3]

		model_cloud = self.load_model_tote_cloud(limb, downsample=True)
		assert model_cloud is not None, 'Error loading tote model'

		model_xform_R, model_xform_t = self.load_R_t(self.get_tote_viewing_camera_xform_path(limb), nparray=True)
		model_cloud = model_cloud.dot(model_xform_R) + model_xform_t

		cur_camera_R = np.array(cur_camera_R).reshape(3,3).T
		cur_camera_t = np.array(cur_camera_t)
		scene_cloud_xyz = scene_cloud_xyz.dot(cur_camera_R.T) + cur_camera_t # transform scene cloud
		scene_cloud[:, 0:3] = scene_cloud_xyz

		print "Making scene cloud with %i points"%scene_cloud.shape[0]
		scene_tree = KDTree(scene_cloud_xyz)
		print "Done"

		if fit:
			R, t = icp.match(model_cloud, scene_tree, iterations=10, initial_threshold=initial_threshold)
			model_cloud = model_cloud.dot(R.T) + t

		keep_idxs = self.subtract_model(model_cloud, scene_tree=scene_tree, threshold=threshold)
		content_cloud = scene_cloud[keep_idxs, :]

		# print 'Dirty content cloud has %i points'%content_cloud.shape[0]

		num_content_cloud_pts = content_cloud.shape[0]
		print "Making content cloud with %i points"%content_cloud.shape[0]
		content_tree = KDTree(content_cloud[:, 0:3])
		print "Done. Querying neighbors within 0.03 m"
		idxs = content_tree.query_ball_point(content_cloud[:, 0:3], 0.03)
		unisolated_flag = map(lambda x: len(x)>=3, idxs)
		content_cloud = content_cloud[np.where(unisolated_flag)[0], :]
		print "Clean cloud has %i points"%content_cloud.shape[0]

		if content_cloud.shape[0] > 1000:
			return content_cloud

		print "Making content cloud with %i points"%content_cloud.shape[0]
		content_tree = KDTree(content_cloud[:, 0:3])
		print "Done. Querying neighbors within 0.03 m"
		idxs = content_tree.query_ball_point(content_cloud[:, 0:3], 0.03)
		unisolated_flag = map(lambda x: len(x)>=4, idxs)
		content_cloud = content_cloud[np.where(unisolated_flag)[0], :]
		print "Clean cloud has %i points"%content_cloud.shape[0]

		if content_cloud.shape[0] > 500:
			return content_cloud

		print "Making content cloud with %i points"%content_cloud.shape[0]
		content_tree = KDTree(content_cloud[:, 0:3])
		print "Done. Querying neighbors within 0.03 m"
		idxs = content_tree.query_ball_point(content_cloud[:, 0:3], 0.03)
		unisolated_flag = map(lambda x: len(x)>=5, idxs)
		content_cloud = content_cloud[np.where(unisolated_flag)[0], :]
		print "Clean cloud has %i points"%content_cloud.shape[0]
		return content_cloud


	def get_shelf_transformation(self, bin_letter, cur_camera_R, cur_camera_t, limb):
		'''
		physical pre-condition: place the camera to the hard-coded position of viewing bin_A

		bin_letter is one of ['A', 'B', ..., 'L']

		return the shelf transformation as (R, t), where R is a column-major order flattened array of rotation matrix,
		and t is an array of 3 numbers for transformation.
		'''
		shelf_perturb_R, shelf_perturb_t = self.icp_get_bin_transform(bin_letter, cur_camera_R, cur_camera_t, limb)
		# save perturbation transformation for shelf subtraction during perception

		#self.shelf_perturb_R = shelf_perturb_R
		#self.shelf_perturb_t = shelf_perturb_t
		return shelf_perturb_R, shelf_perturb_t

	def icp_get_bin_transform(self, bin_letter, cur_camera_R, cur_camera_t, limb):
		'''
		(private) get the current bin transform relative to canonical position for the specified bin
		bin_letter is one of ['A', 'B', ..., 'L'] and is used to load the pre-computed shelf cloud

		return (R, t), where R is a column-major order flattened array of rotation matrix,
		and t is an array of 3 numbers for transformation.
		'''
		cur_camera_R = np.array(cur_camera_R).reshape(3,3).T
		cur_camera_t = np.array(cur_camera_t).flatten()
		_, scene_cloud, _, _ = self.read_once(limb, unit='meter', Nx3_cloud=True, clean=True)
		model_cloud = self.load_model_bin_cloud(bin_letter, limb, downsample=True)
		assert model_cloud is not None, 'Cannot find model cloud'
		scene_cloud = scene_cloud[::downsample_rate,:]
		model_xform_R, model_xform_t = self.load_R_t(self.get_bin_viewing_camera_xform_path(bin_letter, limb), nparray=True)
		model_cloud_xformed = model_cloud.dot(model_xform_R.T) + model_xform_t
		scene_cloud_xformed = scene_cloud.dot(cur_camera_R.T) + cur_camera_t
		np.save(self.prefix+'tmp/model_cloud_xformed.npy', model_cloud_xformed)
		np.save(self.prefix+'tmp/scene_cloud_xformed.npy', scene_cloud_xformed)
		scene_tree = KDTree(scene_cloud_xformed)
		relative_R, relative_t = icp.match(model_cloud_xformed, scene_tree, initial_threshold=initial_threshold)
		relative_R = list(relative_R.flatten(order='F'))
		relative_t = list(relative_t.flat)
		return relative_R, relative_t

	def load_model_bin_cloud(self, bin_letter, limb, downsample=False):
		filename = self.get_canonical_bin_cloud_path(bin_letter, limb)
		try:
			model_cloud = np.load(filename)
		except:
			print 'Cannot find bin model for bin_%s at %s'%(bin_letter, filename)
			return None
		if downsample:
			model_cloud = model_cloud[::downsample_rate, :]
		return model_cloud

	def load_model_tote_cloud(self, limb, downsample=False):
		filename = self.get_tote_cloud_path(limb)
		try:
			model_cloud = np.load(filename)
		except:
			print 'Cannot find tote model at %s'%filename
			return None
		assert len(model_cloud.shape)==2, 'Size mismatch, expecting Nx3, current size is ' + str(model_cloud.shape)
		if downsample:
			model_cloud = model_cloud[::downsample_rate, :]
		return model_cloud


	def read_once(self, limb, unit='meter', Nx3_cloud=False, clean=None):
		while True:
			try:
				if limb=='right':
					camera = RemoteCamera(RIGHT_CAMERA_IP, 30000)
				elif limb=='left':
					camera = RemoteCamera(LEFT_CAMERA_IP, 30000)
				else:
					raise Exception('Unrecognized limb '+str(limb))
				color, cloud, depth_uv, color_uv = camera.read()
				camera.close()
				break
			except:
				print 'Camera Connection Error. Retry in 1 second...'
				time.sleep(1)
		print 'image taken'
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

	def get_canonical_bin_cloud_path(self, bin_letter, limb):
		assert limb in ['left', 'right'], 'limb must be either "left" or "right"'
		return self.prefix+'canonical_model_cloud/bin_%s_%s.npy'%(bin_letter, limb)

	def get_tote_cloud_path(self, limb):
		assert limb in ['left', 'right'], 'limb must be either "left" or "right"'
		return self.prefix+'canonical_model_cloud/tote_%s.npy'%limb

	def get_bin_viewing_camera_xform_path(self, bin_letter, limb):
		return self.prefix+'canonical_model_cloud/bin_%s_xform_%s.txt'%(bin_letter, limb)

	def get_tote_viewing_camera_xform_path(self, limb):
		return self.prefix+'canonical_model_cloud/tote_xform_%s.txt'%limb

	def save_canonical_bin_point_cloud(self, bin_letter, R, t, limb):
		'''
		physical pre-condition: place the camera to the position viewing specified bin
		when the shelf is NOT perturbed.

		bin_letter is one of ['A', 'B', ..., 'L']

		It will return None, but the model will be saved upon return
		'''
		assert isinstance(R, list), 'R must be a list but now it is '+str(R.__class__)
		assert isinstance(t, list), 't must be a list but now it is '+str(t.__class__)
		bin_letter = bin_letter.upper()
		assert 'A'<=bin_letter<='L', 'bin_letter must be between "A" and "L"'
		_, cloud, _, _ = self.read_once(limb, unit='meter', Nx3_cloud=True, clean=True)
		np.save(self.get_canonical_bin_cloud_path(bin_letter, limb), cloud)
		self.save_R_t(self.get_bin_viewing_camera_xform_path(bin_letter, limb), R, t)
		print 'Successfully saved model for bin_'+bin_letter

	def save_canonical_tote_cloud(self, R, t, limb):
		assert isinstance(R, list), 'R must be a list but now it is '+str(R.__class__)
		assert isinstance(t, list), 't must be a list but now it is '+str(t.__class__)
		_, cloud, _, _ = self.read_once(limb, unit='meter', Nx3_cloud=True, clean=True)
		np.save(self.get_tote_cloud_path(limb), cloud)
		self.save_R_t(self.get_tote_viewing_camera_xform_path(limb), R, t)
		print 'Successfully saved model for tote'

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

	def find_max_density_3d(self, cloud, return_cloud=False, with_color=False, uv_hist=None):
		if with_color:
			assert cloud.shape[1]==4 and uv_hist is not None, 'no color channel detected for finding color-weighted max-density'
			assert uv_hist.shape==(64,64), 'wrong uv histogram shape. currently only 64 x 64 shape is supported. '
		xs = cloud[:,0]
		ys = cloud[:,1]
		zs = cloud[:,2]
		xs_sorted = sorted(list(xs.flat))
		ys_sorted = sorted(list(ys.flat))
		zs_sorted = sorted(list(zs.flat))
		N = len(xs_sorted)
		if N==0:
			print 'No points in cloud! '
			return None
		xmin, xmax = xs_sorted[int(N*0.05)], xs_sorted[int(N*0.95)] # in the unit of meter
		ymin, ymax = ys_sorted[int(N*0.05)], ys_sorted[int(N*0.95)]
		zmin, zmax = zs_sorted[int(N*0.05)], zs_sorted[int(N*0.95)]
		x_inrange = np.logical_and(xs >= xmin, xs <= xmax)
		y_inrange = np.logical_and(ys >= ymin, ys <= ymax)
		z_inrange = np.logical_and(zs >= zmin, zs <= zmax)
		inrange = reduce(np.logical_and, [x_inrange, y_inrange, z_inrange])
		bin_content_cloud_inrange = cloud[inrange, :]
		cloud_inrange_xyz = bin_content_cloud_inrange[:, 0:3]
		print 'cloud in range shape!!!', cloud_inrange_xyz.shape
		gaussian_kernel = gaussian_kde(cloud_inrange_xyz.T)
		x_bins = np.linspace(xmin, xmax, 30)
		y_bins = np.linspace(ymin, ymax, 30)
		z_bins = np.linspace(zmin, zmax, 30)
		print 'making grid'
		x_grid, y_grid, z_grid = np.meshgrid(x_bins, y_bins, z_bins, indexing='ij')
		positions = np.vstack([x_grid.ravel(), y_grid.ravel(), z_grid.ravel()])
		print 'calculating density'
		densities = np.reshape(gaussian_kernel(positions).T, x_grid.shape)
		if with_color:
			try:
				# weigh densities by color
				self.suppress(uv_hist)
				uv_hist = uv_hist / uv_hist.sum()
				multiplier = np.ones(densities.shape)
				for p in cloud:
					count_dict = {}
					x, y, z, pcl_float = list(p.flat)
					r, g, b = pcl_float_to_rgb(pcl_float)
					u, v = rgb_to_uv([r,g,b])
					x_idx = int((x-xmin)/((xmax-xmin)/29))
					y_idx = int((y-ymin)/((ymax-ymin)/29))
					z_idx = int((z-zmin)/((zmax-zmin)/29))
					u_idx = int(u/4)
					v_idx = int(v/4)
					hist_val = uv_hist[u_idx, v_idx]
					if (xidx,yidx,zidx) not in count_dict:
						count_dict[(xidx,yidx,zidx)] = 0
					count_dict[(xidx,yidx,zidx)] += 1
					multiplier[xidx, yidx, zidx] *= (hist_val+1)
				for xidx, yidx, zidx in count_dict:
					multiplier[xidx, yidx, zidx] = multiplier[xidx, yidx, zidx]**(1/count_dict[(xidx,yidx,zidx)])
				densities = np.multiply(densities, multiplier)
			except:
				traceback.print_exc()
		print 'argmax-ing'
		max_idx = densities.argmax()
		print 'done'
		if not return_cloud:
			return x_grid.flatten()[max_idx], y_grid.flatten()[max_idx], z_grid.flatten()[max_idx]
		else:
			return (x_grid.flatten()[max_idx], y_grid.flatten()[max_idx], z_grid.flatten()[max_idx]), cloud

	def find_max_density_xy(self, cloud, return_cloud=False):
		xs = tote_content_cloud[:,0]
		ys = tote_content_cloud[:,1]
		xs_sorted = sorted(list(xs.flat))
		ys_sorted = sorted(list(ys.flat))
		N = len(xs_sorted)
		if N==0:
			print 'No points in cloud! '
			return None
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
		if not return_cloud:
			return x_grid.flatten()[max_idx], y_grid.flatten()[max_idx]
		else:
			return (x_grid.flatten()[max_idx], y_grid.flatten()[max_idx]), tote_content_cloud


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
	perceiver = Perceiver()
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



	# def get_normal(self, pts):
	# 	'''
	# 	Normal vector is the eigenvector corresponding to the smallest eigenvalue of the covariance matrix
	# 	'''
	# 	pts = np.array(pts)
	# 	assert pts.shape[1]==3, 'pts must have shape N x 3'
	# 	cov_matrix = np.cov(pts, rowvar=False)
	# 	w, v = np.linalg.eig(cov_matrix)
	# 	w = list(w.flat)
	# 	ind = w.index(min(w))
	# 	normal = -v[:,ind]/v[0,ind] # normalize to have x==-1
	# 	return normal
