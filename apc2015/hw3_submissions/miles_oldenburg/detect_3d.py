from klampt import se3
from utils import *
from match_3d import icp, euclidean_distance_3
import random
import math

objects = ["highlighters","kong_duck","kygen_eggs","school_glue"]
#TODO: play around with the chosen scene to see if your method works well
scene = "highlighters"

# Read the model and the raw point cloud
model_files = ["data/models/"+o+"_model.json" for o in objects]
depth_file = "data/processed_depth/"+scene+".json"

def getDistanceOfClosestPoint(point, scene):
    minimumDistance = euclidean_distance_3(point, scene[0])

    # Check distance to every point in P2
    for index in range(1, len(scene)):
        # Calculate to every point in
        distance = euclidean_distance_3(point, scene[index])

        # If distance is more than existing minimum, set new minimum
        if distance < minimumDistance:
            minimumDistance = distance

    return minimumDistance

def matching_error(object,scene):
    sumSquaredErrors = 0

    # Sample points on the object
    objectSamplePercentage = .01

    for i in range(1, int(math.floor((len(object) * objectSamplePercentage)))):
        # Get sample point
        sample = object[random.randint(0, len(object) - 1)]

        # Add squared error
        sumSquaredErrors += getDistanceOfClosestPoint(sample, scene) ** 2

    return sumSquaredErrors

def main():
    models = [get_reconstructed_model(f,True) for f in model_files]
    scene_full = get_raw_depth(depth_file)
    errors = []
    for i in range(len(objects)):
        print "started main"
        R,t = icp(models[i]['positions'],scene_full)
        print "exited icp"
        #apply the ICP results to the model
        transformed_points = [se3.apply((R,t),p) for p in models[i]['positions']]
        print "transformed points"
        errors.append(matching_error(transformed_points,scene_full))
        print "appended errors"
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

# Matching error table
# scene \ object| highlighter   | kong_duck     | kygen_eggs    | school_glue    | best        |
# highlighter   | 6.03846557464 | 0.04396811827 | 0.09668429704 | 0.103672097849 | kong_duck   |
# kong_duck     | 21.4152366918 | 33.1587173421 | 5.21282151809 | 1.64209554633  | school_glue |
# kygen_eggs    | 5.57626239023 | 13.208353543  | 10.9606010228 | 15.5252150486  | highlighter |
# school_glue   | 124297.61784  | 13.4749216091 | 12031869.155  | 0.747510314841 | school_glue |
#
# The results are not very accurate. I believe that the largest source of error comes from not properly filtering the background out of the scene. Other problems may be introduced by the sampling, or that there is no limit on a many to one point matching. The algorithm itself appears to work as designed, with the model moving toward the scene, but in reality it does not move toward the object part of the scene.
#
# To improve the results I would first set out to remove the walls from the background using a plane detection algorithm and try to reduce the other noise present in the scene cloud. Then I would try to eliminate the many to one point matching problem. I would also look into a kind of uniform sampling instead of a completely random one using a grid data structure so that there is hopefully a more evenly distributed cloud for the model.
#
# I started research on how to use PCL to do some filtering on the scene files before processing but have thus far not been able to use the library on my machine.
#
# The files can easily take over 20 minutes to run. I've found that the iterations themselves are not bad but calculating the final matching_error can take a long time.
