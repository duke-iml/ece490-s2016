from klampt import se3
from utils import *
from match_3d import icp
from scipy.spatial import KDTree
import random

objects = ["highlighters","kong_duck","kygen_eggs","school_glue"]
#TODO: play around with the chosen scene to see if your method works well
scene = "highlighters"

# Read the model and the raw point cloud
model_files = ["data/models/"+o+"_model.json" for o in objects]
depth_file = "data/processed_depth/"+scene+".json"

def matching_error(object,scene):
	#TODO: implement me
	kd = KDTree(scene)
	sampled = random.sample(object, 3000)
	correspondings = []
	for i in sampled:
		try:
			(dist, idx) = kd.query(i)
		except:
			print i
			raise Exception()
		correspondings.append((i, scene[idx], dist))
	tot_dist = sum([i[2] for i in correspondings])/3000
	return tot_dist

def main():
	models = [get_reconstructed_model(f,True) for f in model_files]
	scene_full = get_raw_depth(depth_file)
	errors = []
	for i in range(len(objects)):
		R,t = icp(models[i]['positions'],scene_full)
		#apply the ICP results to the model
		transformed_points = [se3.apply((R,t),p) for p in models[i]['positions']]
		errors.append(matching_error(transformed_points,scene_full))
		print "Object",objects[i],"matching error:",errors[-1]
		models[i]['positions'] = transformed_points

	#pick one of the models to visualize
	visualization_model = 0

	opengl_plot = OpenGLPlot(models[visualization_model], scene_full)
	opengl_plot.initialize_main_loop()

	# If matplot is available you can use it to visualize the points (but it not
	# required) as in the commented line
	# matplot_points(models[visualization_model], scene_full)

if __name__=="__main__":
	main()


#TODO: Task 6 answer goes here.  Report the empirical performance
# of your matching_error() function on each of the four scenes
#
# Matching error table
# scene \ object| highlighter   | kong_duck | kygen_eggs    | school glue   | best      |
# highlighter   |0.0248119671515|0.036727384|0.0432481839699|0.0169442712752|school_glue|
# kong_duck     |0.0137569372684|0.010323547|0.0171693258794|0.0152117209452| kong_duck |
# kygen_eggs    |0.0078014174689|0.009203760|0.0159139142194|0.0089829349434|highlighter|
# school_glue   |0.0360363564091|0.033545815|0.0392677318369|0.0302874453085|school_glue|
# 
# So the algorithm is correct in two cases, out of a total of four cases.
