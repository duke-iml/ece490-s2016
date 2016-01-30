from klampt import se3
from utils import *
from match_3d_mtd13 import icp, compute_fit_metric

import scipy.spatial

objects = ["highlighters","kong_duck","kygen_eggs","school_glue"]
#TODO: play around with the chosen scene to see if your method works well
scene = "school_glue"

# Read the model and the raw point cloud
model_files = ["data/models/"+o+"_model.json" for o in objects]
depth_file = "data/processed_depth/"+scene+".json"

def matching_error(object,scene):
    #return compute_fit_metric(scipy.spatial.KDTree(object), scipy.spatial.KDTree(scene))
    # see match_3d_mtd13.compute_fit_metric
    # this code will run but it is very slow since it doesn't benefit from the connected components
    # analysis performed as when compute_fit_metric is run as part of match_3d_mtd13.icp
    pass

def main():
    models = [get_reconstructed_model(f,True) for f in model_files]
    scene_full = get_raw_depth(depth_file)
    errors = []
    for i in range(len(objects)):
        #R,t = icp(models[i]['positions'],scene_full)
        # the icp returns the fit error
        (R,t), error = icp(models[i]['positions'],scene_full)
        #apply the ICP results to the model
        transformed_points = [se3.apply((R,t),p) for p in models[i]['positions']]
        #errors.append(matching_error(transformed_points,scene_full))
        errors.append(error)
        print "Object",objects[i],"matching error:",errors[-1]
        models[i]['positions'] = transformed_points

    #pick one of the models to visualize
    visualization_model = 3

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
# scene \ object| highlighter    | kong_duck      | kygen_eggs     | school glue     | best        |
#---------------+----------------+----------------+----------------+-----------------+-------------|
# highlighter   | 1.60246916584  | 7.09313446886  | inf            | 0.677389394396  | school_glue |
# kong_duck     | 0.76240453055  | 0.164073962775 | 3.5844171756   | 0.656917977082  | kong_duck   |
# kygen_eggs    | 0.639993303644 | 0.842592042384 | 0.331236198227 | 0.370730420902  | kygen_eggs  |
# school_glue   | 0.866138761098 | 2.3766331626   | 3.79604892729  | 0.0619391177256 | school_glue |
#
