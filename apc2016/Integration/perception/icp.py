import logging, traceback
# logger = logging.getLogger(__name__)

import numpy
import scipy.spatial, scipy.ndimage
from time import time

np = numpy

def match(object_cloud, scene_tree, iterations=20, threshold_max=0.05, initial_threshold=None, return_fit_metric=False):
	total_t = numpy.zeros((1,3))
	total_R = numpy.identity(3)
	last_error = float('inf')

	# copy the points since they get transformed
	obj = numpy.copy(object_cloud)
	#matplot_points(scene_tree.data, obj)

	mark = time()
	for n in range(iterations):
		# get correspondences using distance
		if n == 0:
			# run the first query with infinite distance range in case the object is distant
			# print initial_threshold
			threshold = initial_threshold or float('inf')
			# print threshold
			# raw_input()
		else:
			# apply the distance threshold for subsequent iterations
			# reduce threshold as iterations progress
			# this affectively "anneals" the object to the scene candidate
			threshold = (threshold_max*(5/float(n)) + 0.0005)# * 1000
		mark2 = time()
		# threshold = float('inf')
		print 'querying using threshold:', threshold
		d, ind = scene_tree.query(obj, distance_upper_bound=threshold)
		print 'querying done, using time:', time() - mark2

		# filtered out point without correspondences
		mask = ind < len(scene_tree.data)
		print "Number of valid correspondences:", np.sum(mask)
		obj = obj[mask]
		correspondences = scene_tree.data[ind[mask]]

		if len(obj) == 0:
			# logger.warn('no correspondences')
			print "no correspondences"
			break

		# compute minimizing translation
		obj_mean = obj.mean(axis=0)
		correspondences_mean = correspondences.mean(axis=0)

		# demean the object and the correspondences
		obj -= obj_mean
		correspondences -= correspondences_mean

		# compute the covariance matrix
		H = [ [ obj[:,i].dot(correspondences[:,j]) for j in range(3) ] for i in range(3) ]

		# use SVD for the minimizing rotation
		(U, S, V) = numpy.linalg.svd(H)
		# this works instead of numpy.dot(U, V.T) for some reason
		R = numpy.dot(U, V)

		# check which rotation is minimizing
		sse_normal = ((correspondences - obj.dot(R.T))**2).sum()
		sse_transpose = ((correspondences - obj.dot(R))**2).sum()

		# compute translations and rotations depending upon which of R or R.T minimizes SSE
		if sse_normal < sse_transpose:
			total_t = total_t.dot(R.T) - obj_mean.dot(R.T) + correspondences_mean
			total_R = R.dot(total_R)
			last_error = sse_normal
			print "Incremental R:", R
			print "Total t:", total_t
		else:
			total_t = total_t.dot(R) - obj_mean.dot(R) + correspondences_mean
			total_R = R.T.dot(total_R)
			last_error = sse_transpose
			print "Incremental R:", R
			print "Total t:", total_t

		# transform the object from the original for the next round
		obj = object_cloud.dot(total_R.T) + total_t
		# logger.debug('{} {:.3f} {} {} {}'.format(n, time() - mark, threshold, len(correspondences), 1e9*last_error / len(correspondences)**2))

	if not return_fit_metric:
		return total_R, total_t

	# compute the goodness of fit
	object_tree = scipy.spatial.KDTree(obj)
	fit_metric = compute_fit_metric(object_tree, scene_tree)

	#matplot_points(scene_tree.data, obj)
	return total_R, total_t, fit_metric

def compute_fit_metric(object_tree, scene_tree):
	# how well does the object fit the candidate?
	# ideally every point in the candidate is fit
	# find the nearest neighbor of all candidate points and compute SSE
	_, ind = object_tree.query(scene_tree.data)
	# compute the SSE of all candidate points weighted by the number of candidate points
	# square the number of candidate points since the error term itself is squared
	sse_term = ((scene_tree.data - object_tree.data[ind])**2).sum() / len(scene_tree.data)**2

	# how well does the candidate fit the object?
	# the candidate should "nicely" fit a substantial portion of the object
	# so find how many object points are close to a candidate point
	_, ind = scene_tree.query(object_tree.data, distance_upper_bound=0.005)
	unfit_term = numpy.count_nonzero(ind == len(scene_tree.data))

	# scale by 10 so the "goodness" threshold can be 1
	# square unfit_term to bias towards fitting more points rather than minimizing SSE
	fit_metric = 10 * sse_term * unfit_term**2
	logger.info('fit metric {} {} {}'.format(sse_term, unfit_term, fit_metric))

	return fit_metric
