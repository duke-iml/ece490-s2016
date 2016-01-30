from klampt import se3
from utils import *
from match_3d import icp

objects = ["highlighters","kong_duck","kygen_eggs","school_glue"]
#TODO: play around with the chosen scene to see if your method works well
scene = "highlighters"

# Read the model and the raw point cloud
model_files = ["data/models/"+o+"_model.json" for o in objects]
depth_file = "data/processed_depth/"+scene+".json"

def matching_error(object,scene):
    #TODO: implement me
    return 0

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
# highlighter   |               |           |               |               |       |
# kong_duck     |               |           |               |               |       |
# kygen_eggs    |               |           |               |               |       |
# school_glue   |               |           |               |               |       |
#
