from klampt import se3
from utils import *

#this is the object that we know is in the scene
#TODO: play around with this object, it can be "highlighters", "kong_duck", "kygen_eggs", and "school_glue"
object = "kygen_eggs"

# Read the model and the scene point cloud
model_file = "data/models/"+object+"_model.json"
depth_file = "data/processed_depth/"+object+".json"

def icp(object,scene):
    """Computes a rotation and translation of the object to the scene
    using the ICP algorithm.  Returns the transform (R,t) in Klamp't se3
    format.  
    """
    #TODO: implement me
    # Sample both maps to make the closest point computations faster

    # Compute the minimum distance between the points

    # Reject the outliers, for example, using a threshold

    # Compute the R and t matrices. The procedure can be similar as the one for the
    # 2D images.
    return se3.identity()

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
