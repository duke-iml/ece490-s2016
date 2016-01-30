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

def sum_of_squared_errors(P1,P2):
    """For the corresponding points P1 and P2, returns the sum of squared
    errors"""
    return sum(eu_dist_2(p1,p2) for (p1,p2) in zip(P1,P2))

def eu_dist_2(pointA, pointB):
    """
    Returns Euclidean distance between two points of the same dimension.
    """
    if(len(pointA) != len(pointB)):
        print "Points are not of same dimension."
        return None
    return sum([(pointA[i]-pointB[i])**2 for i in range(len(pointA))])

def main():
    models = [get_reconstructed_model(f,True) for f in model_files]
    scene_full = get_raw_depth(depth_file)
    errors = []
    for i in range(len(objects)):
        R,t = icp(models[i]['positions'],scene_full,True)
        #apply the ICP results to the model
        transformed_points = [se3.apply((R,t),p) for p in models[i]['positions']]
        errors.append(matching_error(transformed_points,scene_full))
        print "Object",objects[i],"matching error:",errors[-1]
        models[i]['positions'] = transformed_points

    bestIndex = 0
    bestError = min(errors)
    for i in range(len(objects)):
        if errors[i] == bestError:
            bestIndex = i
            break

    #pick one of the models to visualize
    visualization_model = bestIndex
    print scene, " scene's best match was the object: ", objects[bestIndex]

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
# scene \ object| highlighter   | kong_duck | kygen_eggs    | school glue   | best  |
# highlighter   |               |     X     |               |               |       |
# kong_duck     |      X        |           |               |               |       |
# kygen_eggs    |               |           |               |       X       |       |
# school_glue   |               |           |               |       X       |       |
#
# I put an X for the identified object of each scene. The code works 1/4 of the time,
# but really the main loop of this program only calls one iteration of the ICP method.
# I don't want to refactor match_3d.py to reflect this structure due to the fact that
# the answer given by match_3d's convergence is also unsatisfactory. I tried to remove
# bad conditions by allowing only a 1-to-x map in the correspondences, but the algorithm
# still hones in on local minimum (like putting the glue bottle against the giant point cloud)
# wall. Future versions of this algorithm will try to identify interesting points
# and the error function will such that ICP method will stop throwing the model 
#against the highest concentration of points.
