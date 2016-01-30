
from __future__ import division

from klampt import se3
from utils import *
from scipy.spatial import KDTree
import numpy as np
import random

#this is the object that we know is in the scene
#TODO: play around with this object, it can be "highlighters", "kong_duck", "kygen_eggs", and "school_glue"
object = "highlighters"

# Read the model and the scene point cloud
model_file = "data/models/"+object+"_model.json"
depth_file = "data/processed_depth/"+object+".json"

def icp(object,scene):
	#return se3.identity()
	"""Computes a rotation and translation of the object to the scene
	using the ICP algorithm.  Returns the transform (R,t) in Klamp't se3
	format.  
	"""
	kd = KDTree(scene)
	print "kd tree done"

	#TODO: implement me
	# Sample both maps to make the closest point computations faster

	# Compute the minimum distance between the points



	# Reject the outliers, for example, using a threshold

	# Compute the R and t matrices. The procedure can be similar as the one for the
	# 2D images.
	#ret = se3.identity()
	#return ret
	#ret[1][2]+=0.2
	#ret[1][1]+=0.3
	#return ret
	sampled = random.sample(object, 1000)
	threshold = 0.75
	net = se3.identity()
	#net =[net[0], (0.2,0.05,0)]
	#return net
	sampled = [se3.apply(net, o) for o in sampled]
	#print object[0]
	for _ in xrange(10):
		print "start finding correspondances"
		correspondings = []
		for i in sampled:
			try:
				(dist, idx) = kd.query(i)
			except:
				print i
				raise Exception()
			correspondings.append((i, scene[idx], dist))
		correspondings.sort(key=lambda x: x[2])
		correspondings = correspondings[0:int(len(correspondings)*threshold)]
		#print "\n".join(map(str,correspondings))
		print "done finding correspondances"
		object_points = [i[0] for i in correspondings]
		scene_points = [i[1] for i in correspondings]
		(R,t) = one_round_icp(scene_points, object_points, 'list')
		#(R,t) = one_round_icp(object_points, scene_points, 'matrix')
		sampled = [se3.apply((R,t),i) for i in sampled]
		net = se3.mul((R,t), net)
		#print R,t
		#print net
	return net

def one_round_icp(p, r, type):
	assert len(p)==len(r)
	N = len(p)

	p_colvec = [np.matrix(i).reshape(3,1) for i in p]
	r_colvec = [np.matrix(i).reshape(3,1) for i in r]
	mu_p = sum(p_colvec)/N;
	mu_r = sum(r_colvec)/N;


	p_tilde = [i-mu_p for i in p_colvec]
	r_tilde = [i-mu_r for i in r_colvec]

	H = sum([p_tilde[i]*r_tilde[i].transpose() for i in xrange(N)])
	#print H
	(U,S,Vt) = np.linalg.svd(H)
	V = Vt.transpose()

	mu_p = np.matrix(mu_p).reshape(3,1)
	mu_r = np.matrix(mu_r).reshape(3,1)

	R1 = U*V.transpose()
	#R1 = V*U.transpose()
	t1 = mu_p-R1*mu_r
	R1_list = R1.flatten('F').tolist()[0]
	t1_list = t1.flatten('F').tolist()[0]
	transformed_r1 = [R1*i+t1 for i in r_colvec]
	tot_dist1 = sum([dist_sq(p_colvec[i],transformed_r1[i]) for i in xrange(N)])

	R2 = V*U.transpose()
	t2 = mu_p-R2*mu_r
	R2_list = R2.flatten('F').tolist()[0]
	t2_list = t2.flatten('F').tolist()[0]
	transformed_r2 = [R2*i+t2 for i in r_colvec]
	tot_dist2 = sum([dist_sq(p_colvec[i], transformed_r2[i]) for i in xrange(N)])

	if tot_dist1<tot_dist2:
		print "Total distance is: "+str(tot_dist1)
		if type=="list":
			return R1_list, t1_list
		elif type=="matrix":
			return R1, t1
		else:
			raise Exception("Type must be either list or matrix")
	else:
		print "Total distance is: "+str(tot_dist2)
		if type=="list":
			return R2_list, t2_list
		elif type=="matrix":
			return R2, t2
		else:
			raise Exception("Type must be either list or matrix")
	

def dist_sq(p1, p2):
	# p1 and p2 needs to be numpy column vector represented as matrix
	d = p1-p2
	d.transpose()*d
	result = d.transpose()*d
	return result[0,0]

def main():
	"""
	Main loop.  Run ICP on the given model and scene file, display the results.
	"""
	model_full = get_reconstructed_model(model_file,True)
	scene_full = get_raw_depth(depth_file)

	#run the ICP
	R,t = icp(model_full['positions'],scene_full)

	#apply the ICP results to the model
	transformed_points = [se3.apply((R,t),p) for p in model_full['positions']]
	model_full['positions'] = transformed_points

	#visualize the results
	opengl_plot = OpenGLPlot(model_full, scene_full)
	opengl_plot.initialize_main_loop()

	# If matplot is available you can use it to visualize the points (but it not
	# required) as in the commented line
	# matplot_points(model_full, clpoints_out)

if __name__ == "__main__":
	main()
	#p = [(random.random(), random.random(), random.random()) for _ in xrange(100)]
	#p = [(i, i+1, i+2) for i in xrange(100)]
	#r = [i for i in p]
	#print "\n".join(map(str,one_round_icp(p,r,'list')))
