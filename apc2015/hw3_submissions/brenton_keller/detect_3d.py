from klampt import se3
from utils import *
from match_3d import icp_plus_segment
import numpy


MAXCLOSESTPTS = 20
objects = ["highlighters","kong_duck","kygen_eggs","school_glue"]
#TODO: play around with the chosen scene to see if your method works well
scene = "school_glue"

# Read the model and the raw point cloud
model_files = ["data/models/"+o+"_model.json" for o in objects]
depth_file = "data/processed_depth/"+scene+".json"

def euclidean_distance_3(P1, P2):
    """
    Compute the Squared Euclidean distance between 2 3D points
    """
    return (P1[0]-P2[0])**2+(P1[1]-P2[1])**2+(P1[2]-P1[2])**2

def sum_of_squared_errors(P1,P2):
    """For the corresponding points P1 and P2, returns the sum of squared
    errors"""
    s = sum(euclidean_distance_3(p1,p2) for (p1,p2) in zip(P1,P2))
    l = [euclidean_distance_3(p1,p2) for (p1,p2) in zip(P1,P2)]
    return s,numpy.var(l)

def matching_error(obj,scene):

    numPoints = int(0.1*len(obj))
    nth = int(len(obj)/numPoints)
    sampled_obj = obj[::nth]
    numPoints = int(0.015*len(scene))
    nth = int(len(scene)/numPoints)
    sampled_scene = scene[::nth]
    closest_points = []
    min_pt_cnt = {}

    #loop through  all of the points in sampled_obj
    for point in sampled_scene:
	#find the point in sampled_scene that has the lowest euclidean distance
	minPoint = [-1,-1,-1]
	minDist = 100000;

	for point2 in sampled_obj:
	    #check to see if this point has been matched too many times
	    if(euclidean_distance_3(point,point2) < minDist):
		tmp = min_pt_cnt.get( (point2[0],point2[1],point2[2]) )
		if(tmp is None or tmp < MAXCLOSESTPTS):
	            minDist = euclidean_distance_3(point,point2)
		    minPoint = point2;

	tu_pt = (minPoint[0],minPoint[1],minPoint[2])
	#add the min point to the dist vector
	if (min_pt_cnt.get( tu_pt ) is not None):
		min_pt_cnt[ tu_pt ] = min_pt_cnt[tu_pt] + 1
	else:
		min_pt_cnt[ tu_pt ] = 1

	#add the minPoint to the list
	closest_points.append(minPoint)


    #find the sum of squared errors
    sum_sqr_err,var_sqr_err = sum_of_squared_errors(sampled_scene,closest_points)

    #print var_sqr_err,sum_sqr_err
    ptMatchRatio = (float(len(scene))/float(len(obj)))
    print ptMatchRatio
    return sum_sqr_err

def main():
    models = [get_reconstructed_model(f,True) for f in model_files]
    scene_full = get_raw_depth(depth_file)

    errors = []
    for i in range(len(objects)):
        ans = icp_plus_segment(scene_full,models[i]['positions'])
	R = ans[0]
	t = ans[1]
	sampled_scene = ans[2]
        #apply the ICP results to the model
        transformed_points = [se3.apply((R,t),p) for p in models[i]['positions']]
        errors.append(matching_error(transformed_points,sampled_scene))
        print "Object",objects[i],"matching error:",errors[-1]
        print '.................ICP finished................'
        models[i]['positions'] = transformed_points

    #pick one of the models to visualize
    visualization_model = errors.index(min(errors))
    

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
# scene \ object|  highlighter   |  kong_duck   |  kygen_eggs    |  school glue   | best        |
# highlighter   |  87.208        |  68.478      |  51.260        |  97.006        | kygen_eggs  |
# kong_duck     |  254.306       |  0.00115     |  213.857       |  250.413       | kong_duck  |
# kygen_eggs    |  294.773       |  0.00085     |  8.0223e-05    |  282.489       | kygen_eggs  |
# school_glue   |  0.000115      |  81.537      |  67.825        |  5.8162e-05    | glue       |
#
