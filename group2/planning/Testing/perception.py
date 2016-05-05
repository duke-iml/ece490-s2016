
from __future__ import division

import numpy as np
import tensorflow as tf
import sys, struct, random, cv2, traceback, operator, rospy, scipy, time
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from skimage import measure
import matplotlib.pyplot as plt
import scipy.misc
import scipy.ndimage.filters as filters


class Perceiver():

	'''====================PUBLIC INTERFACE===================='''
	def perceive(self, name):
		'''
		detect center location for specified object on the scoop
		name is the official APC item name
		return either a tuple of normalized location in range of 0 to 1 or None if detection failed
		'''
		if self.rgb is None:
			return None
		label = self.get_label_for_name(name)
		img = self.rgb
		img = self.crop_image(img)
		# img = scipy.ndimage.imread("testimg1.png")
		binarized, down_ratio, side_len = self.sliding_window(img, label)
		plt.figure()
		plt.imshow(binarized)
		
		if binarized.min()==binarized.max():
			print "binarized min:", binarized.min(), "about to return from perceive()"
			return None
		cc = self.get_largest_cc(binarized)
		if cc is None:
			return None
		x, y = self.get_median_of_cc(cc)
		recovered_x, recovered_y = int((x+side_len/2)*down_ratio), int((y+side_len/2)*down_ratio)
		print "original position in score_map:", x, y
		print "restored position:", recovered_x, recovered_y
		if self.show:
			plt.figure()
			plt.imshow(img[::-1,:,:])
			plt.hold(True)
			plt.plot(recovered_y, img.shape[0]-recovered_x, 'ro')
			plt.gca().set_aspect('equal', adjustable='box')
			plt.axis([0,640,0,480])
			plt.show(block=False)
		return recovered_x/img.shape[0], recovered_y/img.shape[1] # the order is correct

	def detect_background(self, sigma=10):
		'''sigma is the parameter of blurring'''
		img = self.rgb
		if img is None:
			print "not yet received data"
			return None
		self.cropped_blurred_background_img = self.blur_image(self.crop_image(img))

	def perceive_single_object(self, tol=50):
		'''
		detect center location for an object when the object is the only one on the scoop
		return either a tuple of normalized location in range of 0 to 1 or None if detection failed
		this function does NOT verify the identity of object
		'''
		assert self.cropped_blurred_background_img is not None
		img = self.rgb
		cropped_blurred_img = self.blur_image(self.crop_image(img))
		binarized = self.binarize_foreground(cropped_blurred_img)
		cc = self.get_largest_cc(binarized)
		if cc is None:
			return None
		x, y = self.get_median_of_cc(cc)
		print "Single object perception x:", x, "y:", y
		x_normalized, y_normalized = x/img.shape[0], y/img.shape[1]
		print "Normalized x:", x_normalized, "y:", y_normalized
		return x_normalized, y_normalized



	'''==================END PUBLIC INTERFACE=================='''


	def __init__(self, ckpt_file='iter525500.ckpt', arch_file='architecture_spec.txt', show=True):
		self.show = show
		self.init_ros()
		self.init_tf(ckpt_file, arch_file)
		self.cropped_blurred_background_img = None
		time.sleep(3)

	def init_ros(self):
		self.node_name = "cv_bridge_demo"
		rospy.init_node(self.node_name)
		rospy.on_shutdown(self.cleanup)
		self.bridge = CvBridge()
		self.rgb = None
		self.image_sub = rospy.Subscriber("/realsense/rgb", Image, self.image_callback)

		rospy.loginfo("Waiting for image topics...")

	def init_tf(self, ckpt_file, arch_file):
		self.sess = tf.InteractiveSession()
		self.x_net, self.y_net, self.keep_prob = make_net(arch_file)
		self.sess.run(tf.initialize_all_variables()) # start running
		self.saver = tf.train.Saver() # saver object for restoring result
		self.saver.restore(self.sess, ckpt_file)

	def get_label_for_name(self, name):
		return item_dict[official_to_short_mapping[name]]

	def to_uv_color(self, img):
		return cv2.cvtColor(img, cv2.cv.CV_RGB2YCrCb)[:,:,1:3]

	def crop_image(self, img):
		'''
		crop img to only the part with the scoop
		'''
		angle = -3
		rotated = scipy.misc.imrotate(img, angle)
		cropped = rotated[69:390,88:580,:]
		return cropped

	def diff_image(self, img1, img2):
		diff = img1.astype(np.double) - img2.astype(np.double)
		return np.linalg.norm(diff, axis=2)

	def binarize_foreground(self, img_cropped_blurred, threshold=30):
		'''
		img_cropped_blurred is the image after cropping and blurring
		'''
		assert self.cropped_blurred_background_img is not None
		background_blurred_uv = self.to_uv_color(self.cropped_blurred_background_img)
		img_blurred_uv = self.to_uv_color(img_cropped_blurred)
		diff_original = self.diff_image(self.cropped_blurred_background_img, img_cropped_blurred)/5
		diff_uv = self.diff_image(background_blurred_uv, img_blurred_uv)
		diff = diff_original + diff_uv
		binarized = (diff > threshold)
		if self.show:
			plt.figure()
			plt.imshow(self.cropped_blurred_background_img)
			plt.title('Background Blurred')
			plt.figure()
			plt.imshow(img_cropped_blurred)
			plt.title('Current View Blurred')
			plt.figure()
			plt.imshow(diff)
			plt.title('Total RGB and UV Difference')
			plt.colorbar()
			plt.figure()
			plt.title('Binarized')
			plt.imshow(binarized)
			plt.show(block=False)
		return binarized.astype('uint8')

	def dist3(self, p1, p2):
		return ((p1[0]-p2[0])**2+(p1[1]-p2[1])**2+(p1[2]-p2[2])**2)**0.5

	
	def show_image(self):
		img = self.rgb
		# print "Image shape is:", img.shape
		scipy.misc.imsave("saved_"+str(time.time())+".png", img)
		plt.figure()
		plt.imshow(img)
		plt.show(block=False)

	def sliding_window(self, img, label):
		assert label is not None, "label cannot be None"
		h, w, _ = img.shape
		side_len = h/2.5
		down_ratio = side_len / 50
		# return np.load('scoremap.npy')>0.5, down_ratio, 50
		new_h = int(h / down_ratio)
		new_w = int(w / down_ratio)
		img = scipy.misc.imresize(img, (new_h, new_w))
		score_map = np.zeros((new_h-50+1,new_w-50+1))
		print "making patches"
		all_patches = []
		for i in xrange(new_h-50+1):
			for j in xrange(new_w-50+1):
				all_patches.append(img[i:i+50, j:j+50, :])
		all_patches = np.array(all_patches)
		print "all patches size", all_patches.shape
		print "done making patches. predicting"
		divs = range(0, all_patches.shape[0], 100)
		divs.append(all_patches.shape[0]+1)
		label_scores = []
		for i in xrange(len(divs)-1):
			all_scores = self.get_score_many_imgs(all_patches[divs[i]:divs[i+1], :, :, :])
			label_scores += list(all_scores[:,label].flat)
		print "done predicting"
		score_map = np.array(label_scores).reshape(new_h-50+1, new_w-50+1)
		np.save("scoremap", score_map)
		print "down_ratio:", down_ratio, "side length:", side_len
		print "score map size: ", score_map.shape
		print "score_map min:", score_map.min(), "score_map max:", score_map.max()
		if self.show:
			plt.figure()
			plt.imshow(score_map)
		return (score_map>0.5).astype('uint8'), down_ratio, 50

	def get_score_many_imgs(self, imgs):
		'''
		imgs is N x h x w x 3
		if label is None, return the index and value of the largest predicted value
		else, return the value at label
		'''
		N,h,w,_ = imgs.shape
		assert h==50 and w==50, "image is not 50 x 50"
		result = self.y_net.eval(feed_dict={self.x_net:imgs, self.keep_prob: 1.0})
		# print "shape of result should be N x 39:", result.shape, "where N is", N
		return result


	def get_median_of_cc(self, idxs):
		# print idxs.__class__, idxs[0], len(idxs)
		xs = sorted([x for (x,y) in idxs])
		ys = sorted([y for (x,y) in idxs])
		# print xs.__class__
		# print ys.__class__
		return xs[len(xs)//2], ys[len(ys)//2]


	def get_largest_cc(self, binary_img, label=1): # CHECK FOR NONE BEING RETURNED
		'''
		return a list of coordinates that are in the largest connected component with specified label
		'''
		binary_img[3,4] = 0
		binary_img[3,5] = 0
		binary_img[3,6] = 0
		cc_labeled = measure.label(binary_img)
		# print cc_labeled.min(), cc_labeled.max()
		max_label = cc_labeled.max()
		largest_size = -1
		largest_idxs = None
		for l in xrange(max_label+1):
			# print cc_labeled
			idxs = map(lambda x:list(x.flat), np.where(cc_labeled==l))
			# print idxs
			idxs = zip(*idxs)
			# print idxs
			if binary_img[idxs[0][0], idxs[0][1]]!=label:
				continue
			if len(idxs)>largest_size:
				largest_size = len(idxs)
				largest_idxs = idxs
		return largest_idxs

	

	def cleanup(self):
		print "Shutting down vision node."
		cv2.destroyAllWindows()


	def image_callback(self, ros_image):
		# print "received ROS"
		# Use cv_bridge() to convert the ROS image to OpenCV format
		try:
			rgb_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
		except CvBridgeError, e:
			print e
		self.rgb = rgb_image

	def pad_to_square(self, img):
		'''
		img is an image in numpy array format
		padding with random color
		'''
		h, w, _ = img.shape
		s = max(w, h)
		new_img = np.zeros((s, s, 3))
		new_img[:,:,0] = np.ones((s,s))*(int(random.random()*255))
		new_img[:,:,1] = np.ones((s,s))*(int(random.random()*255))
		new_img[:,:,2] = np.ones((s,s))*(int(random.random()*255))
		if h<w:
			new_img[int((w-h)/2):int((w-h)/2+h), :, :] = img
		else:
			new_img[:, int((h-w)/2):int((h-w)/2+w), :] = img
		return new_img

	def blur_image(self, img, sigma=10):
		return filters.gaussian_filter(img, [sigma, sigma, 0])


def weight_variable(shape):
	initial = tf.truncated_normal(shape, stddev=0.1)
	return tf.Variable(initial)

def bias_variable(shape):
	initial = tf.constant(0.1, shape=shape)
	return tf.Variable(initial)

def conv2d_layer(x, W):
	return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='SAME')

def max_pool_layer(x, sz1, sz2):
	return tf.nn.max_pool(x, ksize=[1, sz1, sz2, 1], strides=[1, sz1, sz2, 1], padding='SAME')

def norm_layer(x, radius):
	return tf.nn.local_response_normalization(x, depth_radius=radius)

def parse_spec(f):
	'''
	input filename
	return input and output dimension, and layer specification as a dictionary
	'''
	if isinstance(f, basestring):
		f = open(f)
	cur_layer = None
	layers = dict()
	for l in f:
		if l.strip()=='' or l.strip().startswith('#'):
			continue
		l = l.strip()
		if l.startswith('input_size'):
			dim_in = map(int, l.split(':')[1].strip().split('x'))
			assert len(dim_in)==3, "input dimension must be 3. currently dimension is: "+str(dim_in)
		if l.startswith('output_size'):
			dim_out = int(l.split(':')[1].strip())
		if l.startswith('layer'):
			cur_layer = dict()
			layer_index = int(l.split(':')[0].strip().replace('layer_',''))
			layers[layer_index] = cur_layer
		if l.startswith('type'):
			cur_layer['type'] = l.split(':')[1].strip()
		if l.startswith('activation'):
			cur_layer['activation'] = l.split(':')[1].strip()
		if l.startswith('kernel'):
			cur_layer['kernel'] = map(int, l.split(':')[1].strip().split('x'))
			assert len(cur_layer['kernel'])==4, "kernel dimension must be 4. currently dimension is: "+str(cur_layer['kernel'])
		if l.startswith('size'):
			size = l.split(':')[1].strip()
			if size=='same':
				cur_layer['size'] = size
			elif cur_layer['type']=='maxpool':
				cur_layer['size'] = map(int, size.split('x'))
				assert len(cur_layer['size'])==2, "size dimension must be 2. currently dimension is: "+str(cur_layer['size'])
			elif cur_layer['type']=='fully_connect':
				cur_layer['size'] = int(l.split(':')[1].strip())
		if l.startswith('radius'):
			cur_layer['radius'] = int(l.split(':')[1].strip())
		if l.startswith('keep_prob'):
			cur_layer['keep_prob'] = float(l.split(':')[1].strip())
	return dim_in, dim_out, layers

def get_total_size(t):
	dims = map(lambda x: x._value, t.get_shape())
	return reduce(operator.mul, dims[1:], 1)

def make_net(f, verbose=True):
	'''
	input filename
	return x and y that are input and output to the neural net, connected by layers specified by the file
	'''
	dim_in, dim_out, layer_spec = parse_spec(f)
	# print layer_spec
	x = tf.placeholder("float", shape=[None, dim_in[0], dim_in[1], dim_in[2]])
	prev_layer = x
	i = 1
	while True:
		try:
			cur_spec = layer_spec[i]
			i += 1
		except:
			# print "error layer:", i
			# traceback.print_exc()
			break
		if verbose:
			print prev_layer.get_shape()
			print cur_spec
		if cur_spec['type']=='conv':
			assert cur_spec['activation']=='relu' and 'kernel' in cur_spec, "errornous spec"+str(cur_spec)
			kernel_dim = cur_spec['kernel']
			w = weight_variable(kernel_dim)
			b = weight_variable([kernel_dim[-1]])
			prev_layer = tf.nn.relu(conv2d_layer(prev_layer, w)+b)
		elif cur_spec['type']=='maxpool':
			assert 'size' in cur_spec, "errornous spec"+str(cur_spec)
			prev_layer = max_pool_layer(prev_layer, *cur_spec['size'])
		elif cur_spec['type']=='norm':
			assert 'radius' in cur_spec, "errornous spec"+str(cur_spec)
			prev_layer = norm_layer(prev_layer, cur_spec['radius'])
		elif cur_spec['type']=='flatten':
			prev_layer = tf.reshape(prev_layer, [-1, get_total_size(prev_layer)])
		elif cur_spec['type']=='fully_connect':
			assert cur_spec['activation']=='relu' and 'size' in cur_spec, "errornous spec"+str(cur_spec)
			prev_num = get_total_size(prev_layer)
			if cur_spec['size']=='same':
				next_num = prev_num
			else:
				next_num = cur_spec['size']
			w = weight_variable([prev_num, next_num])
			b = bias_variable([next_num])
			prev_layer = tf.nn.relu(tf.matmul(prev_layer, w)+b)
		elif cur_spec['type']=='dropout':
			assert 'keep_prob' in cur_spec, "errornous spec"+str(cur_spec)
			keep_prob = tf.placeholder("float")
			prev_layer = tf.nn.dropout(prev_layer, keep_prob)
			# prev_layer = tf.nn.dropout(prev_layer, cur_spec['keep_prob'])
		elif cur_spec['type']=='readout':
			w = weight_variable([get_total_size(prev_layer), dim_out])
			b = bias_variable([dim_out])
			y_out = tf.nn.softmax(tf.matmul(prev_layer, w)+b)
		else:
			raise Exception("Unrecognized type: "+str(cur_spec['type']))
	if verbose:
		print y_out.get_shape()
	return x, y_out, keep_prob




item_dict = {
	"background":0,
	"balls":1,
	"baseball":2,
	"bear":3,
	"bones":4,
	"bowl":5,
	"brush":6,
	"bubble_mailer":7,
	"bunny_book":8,
	"coffee":9,
	"crayon":10,
	"cup":11,
	"dove":12,
	"dumbbell":13,
	"dvd":14,
	"expo":15,
	"extension_cord":16,
	"gloves":17,
	"glue":18,
	"glue_sticks":19,
	"hooks":20,
	"indexcards":21,
	"jokes_book":22,
	"light_bulb":23,
	"outlet_plugs":24,
	"papertowel":25,
	"pencil_holder":26,
	"pencils":27,
	"scissors":28,
	"shower_curtain":29,
	"socks":30,
	"stems":31,
	"tablets":32,
	"tape":33,
	"tissue":34,
	"toothbrush":35,
	"tshirt":36,
	"utility_brush":37,
	"water":38
}

official_to_short_mapping = {
	"i_am_a_bunny_book":"bunny_book",
	"laugh_out_loud_joke_book":"jokes_book",
	"scotch_bubble_mailer":"bubble_mailer",
	"up_glucose_bottle":"tablets",
	"dasani_water_bottle":"water",
	"rawlings_baseball":"baseball",
	"folgers_classic_roast_coffee":"coffee",
	"elmers_washable_no_run_school_glue":"glue",
	"hanes_tube_socks":"socks",
	"womens_knit_gloves":"gloves",
	"cherokee_easy_tee_shirt":"tshirt",
	"peva_shower_curtain_liner":"shower_curtain",
	"cloud_b_plush_bear":"bear",
	"barkely_hide_bones":"bones",
	"kyjen_squeakin_eggs_plush_puppies":"balls",
	"cool_shot_glue_sticks":"glue_sticks",
	"creativity_chenille_stems":"stems",
	"soft_white_lightbulb":"light_bulb",
	"safety_first_outlet_plugs":"outlet_plugs",
	"oral_b_toothbrush_green":"toothbrush",
	"oral_b_toothbrush_red":"toothbrush",
	"dr_browns_bottle_brush":"brush",
	"command_hooks":"hooks",
	"easter_turtle_sippy_cup":"cup",
	"fiskars_scissors_red":"scissors",
	"scotch_duct_tape":"tape",
	"woods_extension_cord":"extension_cord",
	"platinum_pets_dog_bowl":"bowl",
	"fitness_gear_3lb_dumbbell":"dumbbell",
	"rolodex_jumbo_pencil_cup":"pencil_holder",
	"clorox_utility_brush":"utility_brush",
	"kleenex_paper_towels":"papertowel",
	"expo_dry_erase_board_eraser":"expo",
	"kleenex_tissue_box":"tissue",
	"ticonderoga_12_pencils":"pencils",
	"crayola_24_ct":"crayon",
	"jane_eyre_dvd":"dvd",
	"dove_beauty_bar":"dove",
	"staples_index_cards":"indexcards"
}

if __name__ == '__main__':
	perceiver = Perceiver()
	time.sleep(1)
	perceiver.perceive('laugh_out_loud_joke_book')



	# def perceive_single_object(self):
	# 	assert self.background_color is not None
	# 	img = self.rgb.astype('uint8')
	# 	h, w, _ = img.shape
	# 	binarized = np.zeros((h,w))
	# 	for i in xrange(h):
	# 		for j in xrange(w):
	# 			if self.dist3(self.background_color, img[i,j,0:3])>self.background_dist_tol:
	# 				binarized[i,j] = 1
	# 	plt.figure()
	# 	plt.imshow(binarized)
	# 	plt.show()
	
	# def detect_background_color(self):
	# 	img = self.rgb
	# 	h, w, _ = img.shape
	# 	r_list = list(img[h//4:3*h//4, w//4:3*w//4, 0].flat)
	# 	r = sorted(r_list)[len(r_list)//2]
	# 	g_list = list(img[h//4:3*h//4, w//4:3*w//4, 1].flat)
	# 	g = sorted(g_list)[len(g_list)//2]
	# 	b_list = list(img[h//4:3*h//4, w//4:3*w//4, 2].flat)
	# 	b = sorted(b_list)[len(b_list)//2]
	# 	self.background_color = map(int, [r,g,b])
	# 	print r, g, b
	# 	background_pure = np.zeros((50,50,3)).astype(np.uint8)
	# 	background_pure[:,:,0] = r
	# 	background_pure[:,:,1] = g
	# 	background_pure[:,:,2] = b
	# 	plt.figure()
	# 	plt.imshow(background_pure)
	# 	# plt.show()