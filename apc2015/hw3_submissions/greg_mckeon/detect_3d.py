from klampt import se3
from utils import *
from match_3d import icp

objects = ["highlighters","kong_duck","kygen_eggs","school_glue"]
#TODO: play around with the chosen scene to see if your method works well
scene = "school_glue"

# Read the model and the raw point cloud
model_files = ["data/models/"+o+"_model.json" for o in objects]
depth_file = "data/processed_depth/"+scene+".json"

def matching_error(object,scene):
    return sum_of_squared_errors(object,scene)

def euclidean_distance_3(P1, P2):
    """
    Compute the Squared Euclidean distance between 2 3D points
    """
    return (P1[0]-P2[0])**2+(P1[1]-P2[1])**2+(P1[2]-P2[2])**2

def sum_of_squared_errors(P1,P2):
    """For the corresponding points P1 and P2, returns the sum of squared
    errors"""
    return sum(euclidean_distance_3(p1,p2) for (p1,p2) in zip(P1,P2))

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
    visualization_model = 2

    #opengl_plot = OpenGLPlot(models[visualization_model], scene_full)
    #opengl_plot.initialize_main_loop()

    # If matplot is available you can use it to visualize the points (but it not
    # required) as in the commented line
    #matplot_points(models[visualization_model], scene_full)

if __name__=="__main__":
    main()


#TODO: Task 6 answer goes here.  Report the empirical performance
# of your matching_error() function on each of the four scenes
#
# Matching error table
# scene \ object| highlighter   | kong_duck | kygen_eggs    | school glue   | best  |
# highlighter   | 26235.2       |   32885   |    29850      |      22782    |  glue |
# kong_duck     |  16384        |   11465   |    18496      |      8307.05  |  glue |
# kygen_eggs    |   28676       |   27086   |    28067      |      16639    |  glue |
# school_glue   |    63248      |   71790   |    66149      |      43122    |  glue |
#
#  I've implemented a mean-squared error function, but its use is limited by problems with my match_3d function.
#  The match_3d function seems to be moving further from, rather than closer to, the scene.  I think my sampling methodology is poor, and that there might be a problem with
# the computation of my t and R components.  I tried redoing my code, but got similar results, which leads me to believe there is a flaw in I am computing these.
#
# As for the error function, I think it is an easier time when the scene is large and the object is small.  I believe the errors are correct, but are skewed by the poor performance of the detection algorithm
#
