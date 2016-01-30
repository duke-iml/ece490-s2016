from klampt import se3
from utils import *
import random
import numpy
#this is the object that we know is in the scene
#TODO: play around with this object, it can be "highlighters", "kong_duck", "kygen_eggs", and "school_glue"
object = "school_glue"

# Read the model and the scene point cloud
model_file = "data/models/"+object+"_model.json"
depth_file = "data/processed_depth/"+object+".json"
def icp(object,scene):
    """Computes a rotation and translation of the object to the scene
    using the ICP algorithm.  Returns the transform (R,t) in Klamp't se3
    format.  
    """
    #TODO: implement me
    newObject = []
    newScene = []
    # Sample both maps to make the closest point computations faster
    for i in range((int)(len(object)*.01)) :
        newObject.append(object[(int)(random.random()*len(object))])
        newScene.append(scene[(int)(random.random()*len(scene))])
    #Compute the minimum distance between the points
    minDist = get_closest_points_3d(newObject,newScene)
    # Reject the outliers, for example, using a threshold
    listy = []
    for i in range(len(newObject)):
        listy.append(euclidean_distance_3(newObject[i], minDist[i]))
    sort = sorted(listy)
    threshold = sort[(int)(len(newObject)*.05)]
    S1,S2 = threshold_closest_points(newObject, minDist, threshold)
    S1_shift, mean1 = subtract_mean(S1)
    S2_shift, mean2 = subtract_mean(S2)
    # Compute the R and t matrices. The procedure can be similar as the one for the
    # 2D images.
    R = compute_error_minimizing_rotation(S1_shift,S2_shift)
    R = R.tolist()
    #find the t such that R*p+t = R*(p-mean2)+mean1
    Rmean2 = [R[0][0]*mean2[0]+R[0][1]*mean2[1]+R[0][2]*mean2[2],
              R[1][0]*mean2[0]+R[1][1]*mean2[1]+R[1][2]*mean2[2],
              R[2][0]*mean2[0]+R[2][1]*mean2[1]+R[2][2]*mean2[2]]
    rNew = [R[0][0],R[1][0],R[2][0],R[0][1],R[1][1],R[2][1],R[0][2],R[1][2],R[2][2]]
    return rNew,[mean1[0]-Rmean2[0],mean1[1]-Rmean2[1], mean1[2]-Rmean2[2]]

def subtract_mean(point_list):
    """
    Input list: ((x1,y1),(x2,y2),...)
    Returns a pair ([p1-mean,...,pn-mean], mean)
    """
    mean_x = 0
    mean_y = 0
    mean_z = 0
    point_list_minus_mean = []
    for i in range(len(point_list)):
        mean_x = mean_x+point_list[i][0]
        mean_y = mean_y+point_list[i][1]
        mean_z = mean_z+point_list[i][2]
    mean_x = mean_x/len(point_list)
    mean_y = mean_y/len(point_list)
    mean_z = mean_z/len(point_list)
    for j in range(len(point_list)):
        point_list_minus_mean.append([point_list[j][0]-mean_x,point_list[j][1]-mean_y,point_list[j][2]-mean_z])
    print "means: ", mean_x, mean_y, mean_z
    return point_list_minus_mean, [mean_x, mean_y, mean_z]


def compute_error_minimizing_rotation(Points1, Points2):
    """
    Compute the rotation matrix that rotates the points in Points2 to minimize
    squared error to the points in Points1
    """
    spot11=0
    spot12=0
    spot13=0
    spot21=0 
    spot22=0
    spot23=0
    spot31=0 
    spot32=0
    spot33=0
    for i in range(len(Points1)):
        spot11 = spot11+Points1[i][0]*Points2[i][0]
        spot12 = spot12+Points1[i][0]*Points2[i][1]
        spot13 = spot13+Points1[i][0]*Points2[i][2]
        spot21 = spot21+Points1[i][1]*Points2[i][0]
        spot22 = spot22+Points1[i][1]*Points2[i][1]
        spot23 = spot23+Points1[i][1]*Points2[i][2]
        spot31 = spot31+Points1[i][2]*Points2[i][0]
        spot32 = spot32+Points1[i][2]*Points2[i][1]
        spot33 = spot33+Points1[i][2]*Points2[i][2]
    U, s, V = numpy.linalg.svd(numpy.matrix([[spot11, spot12, spot13],[spot21,spot22,spot23],[spot31,spot32,spot33]]))
    return numpy.dot(numpy.transpose(V),U)
def euclidean_distance_3(P1, P2):
    """
    Compute the Squared Euclidean distance between 2 2D points
    """
    return (P1[0]-P2[0])**2+(P1[1]-P2[1])**2+(P1[2]-P2[2])**2

def sum_of_squared_errors(P1,P2):
    """For the corresponding points P1 and P2, returns the sum of squared
    errors"""
    return sum(euclidean_distance_3(p1,p2) for (p1,p2) in zip(P1,P2))

def get_closest_points_3d(P1, P2):
    """
    Compute the closest 2D points searching exhaustively. The output has
    the same dimension as P1 and produces the closest point in P2 to P1.
    Input points: [[x1,y1],[x2,y2],...]
    """
    out = []
    for point in range(len(P1)):
        myMin = 99999999999
        smallest = []
        for point2 in range(len(P2)):
            if euclidean_distance_3(P1[point],P2[point2])<myMin:
                myMin = euclidean_distance_3(P1[point],P2[point2])
                smallest = P2[point2]
        P2.remove(smallest)
        out.append(smallest)
    return out

def threshold_closest_points(P1, P2, threshold):
    """
    Discard the pairs of points for which their mutual distance exceeds the
    predefined threshold
    Input points: [[x1,y1],[x2,y2],...]
    """
    #TODO: implement me
    newP1 = []
    newP2 = []
    for point in range(len(P1)):
        if euclidean_distance_3(P1[point],P2[point])<threshold:
            newP1.append(P1[point])
            newP2.append(P2[point])
    return newP1, newP2
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
    print "iter",i,len(transformed_points),"correspondences, mean squared error",float(sum_of_squared_errors(transformed_points,scene_full))/len(transformed_points)

    # If matplot is available you can use it to visualize the points (but it not
    # required) as in the commented line
    # matplot_points(model_full, clpoints_out)

if __name__ == "__main__":
    main()
