from klampt import se3
from utils import *
from match_3d import ApplyICP
from scipy import spatial


objects = ["highlighters","kong_duck","kygen_eggs","school_glue"]
obs = ["highlighters","school_glue"]
scene = "school_glue"

# Read the model and the raw point cloud
model_files = ["data/models/"+o+"_model.json" for o in objects]
depth_file = "data/processed_depth/"+scene+".json"

def euclidean_distance_2(P1, P2):
    """Compute the Squared Euclidean distance between 2 3D points"""
    dist = (P1[0]-P2[0])**2+(P1[1]-P2[1])**2+(P1[2]-P2[2])**2
    return dist

def sum_of_squared_errors(P1,P2):
    """For the corresponding points P1 and P2, returns the sum of squared
    errors"""
    return sum(euclidean_distance_2(p1,p2) for (p1,p2) in zip(P1,P2))

def matching_error(object,scene):

    return sum_of_squared_errors(object, scene)

def main():


    for x in range (0,len(obs)):

        model_files = ["data/models/"+o+"_model.json" for o in objects]
        depth_file = "data/processed_depth/"+obs[x]+".json"
        print depth_file
        models = [get_reconstructed_model(f,True) for f in model_files]
        scene_full = get_raw_depth(depth_file)
        errors = []

        for i in range(len(objects)):
            print model_files[i]
            R,t = ApplyICP(models[i]['positions'],scene_full)
            #apply the ICP results to the model
            if len(R) != 0:
                transformed_points = [se3.apply((R,t),p) for p in models[i]['positions']]
                errors.append(matching_error(transformed_points,scene_full))
                print "Object",objects[i],"matching error:",errors[-1]
                models[i]['positions'] = transformed_points
            else:
                print "ERROR"
        print depth_file
        print "END"
        print ""

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
# scene / object|  |       highlighter       |        kong_duck        |      kygen_eggs      |       school glue      |         best          |
# highlighter      |    7672.2675232    |    25307.1022496    |   68820.4125845   |    23816.543182    |   Highlighter    |
# kong_duck     |   17023.7909766   |   5666.8196733538   |   13709.5053741   |   6976.10624154   |   Kong Duck    |
# kygen_eggs    |   29865.8122438   |    27904.3700675    |   19434.8562633   |   23286.3805922   |   Kygen Eggs   |
# school_glue    |   53637.3161366   |   68185.3311397   |   90196.2482889    |    4149.9421717|    |   School Glue   |
#The matching algorithm appeared to work well as it matched the correct model in all cases, however further investigation is required to ensure
#that the algorithm correctly transforms visually. Visually from MatPlotLib  the algorithm object does not match the scene however I believe that with
# further iterations the algorithm would converge successfully. Based on this further development of thresholding may be required to ensure a more efficient
#convergence.
