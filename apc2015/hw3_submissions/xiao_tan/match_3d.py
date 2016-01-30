from klampt import se3
from utils import *
import numpy 
from scipy import spatial
import math

#this is the object that we know is in the scene
#TODO: play around with this object, it can be "highlighters", "kong_duck", "kygen_eggs", and "school_glue"
object = "school_glue"

# Read the model and the scene point cloud
model_file = "data/models/"+object+"_model.json"
depth_file = "data/processed_depth/"+object+".json"

def euclidean_distance(P1, P2):
    return (P1[0]-P2[0])**2+(P1[1]-P2[1])**2+(P1[2]-P2[2])**2

def sum_of_squared_errors(P1,P2):
    return sum(euclidean_distance(p1,p2) for (p1,p2) in zip(P1,P2))

def getClosestPoints(object, scene):
    pointArray = []
    # build a tree
    tree = spatial.KDTree(scene)
    # find the nearset
    # define a tuple to store the # of times for each 'scene' point
    Rng = len(object)
    tup = range(len(scene))
    for i in xrange(Rng):
        tup[i] = 0

    for i in xrange(Rng):
        dist, numi = tree.query(object[i])
        tup[numi] = tup[numi] + 1
        while tup[numi] >= 10: #  is the threshold for enforcing a limit on the number of points that matched to a single point
            numi = numi + 1
            if numi >= Rng:
                numi = numi - 1
                break
        pointArray.append(tree.data[numi]) # or try scene[numi]
    return pointArray

def thresholdClosestPoints(P1, P2):
    threshold = 0.02
    pointArr = []
    S1 = []
    S2 = []

    assert(len(P1) == len(P2))

    for i in xrange(len(P1)):
        # print "euclidean_distance(P1[",i,"], P2[",i,"]):", euclidean_distance(P1[i], P2[i])
        if euclidean_distance(P1[i], P2[i]) <= threshold:
            pointArr.append([P1[i], P2[i]])

    for j in xrange(len(pointArr)):
        S1.append(pointArr[j][0])
        S2.append(pointArr[j][1])
    # print "object length:", len(P1)
    # print "after threshold:", len(S2)
    return (S1, S2)

def getCorrespondences(P1, P2):
    P2matches = getClosestPoints(P1, P2)
    # print "P2matches", P2matches
    S1, S2 = thresholdClosestPoints(P1,P2matches)
    return S1, S2

def subtractMean(point_list):
    """
    Input list: ((x1,y1),(x2,y2),...)
    Returns a pair ([p1-mean,...,pn-mean], mean)
    """
    PL = len(point_list)
    mean_x = 0
    mean_y = 0
    mean_z = 0
    
    for i in xrange(PL):
        mean_x = mean_x + point_list[i][0]
        mean_y = mean_y + point_list[i][1]
        mean_z = mean_z + point_list[i][2]
    mean_x = mean_x / PL
    mean_y = mean_y / PL
    mean_z = mean_z / PL

    point_list_minus_mean = []
    for i in xrange(PL):
        point_list_minus_mean.append([point_list[i][0]-mean_x, point_list[i][1]-mean_y, point_list[i][2]-mean_z])

    return point_list_minus_mean, [mean_x, mean_y, mean_z]

def compute_error_minimizing_rotation(Points1, Points2):
    """
    Compute the rotation matrix that rotates the points in Points2 to minimize
    squared error to the points in Points1
    """
    assert(len(Points1) == len(Points2))
    H11 = 0
    H12 = 0
    H13 = 0
    H21 = 0
    H22 = 0
    H23 = 0
    H31 = 0
    H32 = 0
    H33 = 0
    PL = len(Points1)

    for i in xrange(PL):
        H11 = H11 + Points1[i][0] * Points2[i][0]
        H12 = H12 + Points1[i][0] * Points2[i][1]
        H13 = H13 + Points1[i][0] * Points2[i][2]
        H21 = H21 + Points1[i][1] * Points2[i][0]
        H22 = H22 + Points1[i][1] * Points2[i][1]
        H23 = H23 + Points1[i][1] * Points2[i][2]
        H31 = H31 + Points1[i][2] * Points2[i][0]
        H32 = H32 + Points1[i][2] * Points2[i][1]
        H33 = H33 + Points1[i][2] * Points2[i][2]

    H = [[H11, H12, H13], [H21, H22, H23], [H31, H32, H33]]

    U, S, V = numpy.linalg.svd(H)
    R = U.dot(V.T)

    tri2transformed = transform_points(Points2,R,[0,0,0])
    rot1 = sum_of_squared_errors(Points1, tri2transformed)

    R = (U.dot(V.T)).T
    tri2transformed = transform_points(Points2,R,[0,0,0])
    rot2 = sum_of_squared_errors(Points1, tri2transformed)

    if rot1 < rot2:
        return U.dot(V.T)
    else:
        return (U.dot(V.T)).T

def transform_point(p,R,t):
    x = R[0][0]*p[0]+R[0][1]*p[1]+R[0][2]*p[2]+t[0]
    y = R[1][0]*p[0]+R[1][1]*p[1]+R[1][2]*p[2]+t[1]
    z = R[2][0]*p[0]+R[2][1]*p[1]+R[2][2]*p[2]+t[2]
    return [x,y,z]

def transform_points(Points,R,t):
    return [transform_point(p,R,t) for p in Points]

def icp(object,scene):
    """Computes a rotation and translation of the object to the scene
    using the ICP algorithm.  Returns the transform (R,t) in Klamp't se3
    format.  
    """
    #TODO: implement me
    # Sample both maps to make the closest point computations faster
    # prefer to use kdtree instead of sampling, since hard to decide how many points to sample;

    #get the correspondences
    S1,S2 = getCorrespondences(object,scene)
    # Center the resulting pairs substracting their means
    S1_shift, mean1 = subtractMean(S1)
    S2_shift, mean2 = subtractMean(S2)
    # Compute the minimum distance between the points

    # Reject the outliers, for example, using a threshold

    # Compute the R and t matrices. The procedure can be similar as the one for the
    # 2D images.

    #calculate the error-minimizing rotation
    R = compute_error_minimizing_rotation(S1_shift,S2_shift)
    #find the t such that R*p+t = R*(p-mean2)+mean1
    Rmean2 = [R[0][0]*mean2[0]+R[0][1]*mean2[1]+R[0][2]*mean2[2],
              R[1][0]*mean2[0]+R[1][1]*mean2[1]+R[1][2]*mean2[2],
              R[2][0]*mean2[0]+R[2][1]*mean2[1]+R[2][2]*mean2[2]]
    t = [mean1[0]-Rmean2[0],mean1[1]-Rmean2[1],mean1[2]-Rmean2[2]]
    # return R, t
    #print "se3.identity()", se3.identity()

    #print "(R, t),", (R, t)

    rArray = []
    rArray.append(R[0][0])
    rArray.append(R[1][0])
    rArray.append(R[2][0])
    rArray.append(R[0][1])
    rArray.append(R[1][1])
    rArray.append(R[2][1])
    rArray.append(R[0][2])
    rArray.append(R[1][2])
    rArray.append(R[2][2])

    return rArray, t 
    #return se3.identity()

def main():
    """
    Main loop.  Run ICP on the given model and scene file, display the results.
    """
    model_full = get_reconstructed_model(model_file,True)
    scene_full = get_raw_depth(depth_file)
    for iter in xrange(10):
        R,t = icp(model_full['positions'], scene_full)
        # R,t = icp(scene_full, model_full['positions'])
        transformed_point = [se3.apply((R,t),p) for p in scene_full]
        scene_full = transformed_point
        # transformed_point = [se3.apply((R,t),p) for p in model_full['positions']]
        # model_full['positions'] = transformed_point
        #visualize the results
        S1,S2 = getCorrespondences(model_full['positions'], scene_full)
        print "iter", iter+1, "mean squared error",float(sum_of_squared_errors(S1,S2))/len(S1)

    opengl_plot = OpenGLPlot(model_full, scene_full)
    opengl_plot.initialize_main_loop()

        # S1,S2 = getCorrespondences(object, scene_full)
        # print "iter", iter+1, "correspondences, mean squared error",float(sum_of_squared_errors(S1,S2))/len(S1)

    matplot_points(model_full, clpoints_out)

    
"""
    for iter in range(1):

        Rdelta,tdelta = icp(model_full['positions'], scene_full)

        sceneTransformed = transform_points(scene_full,Rdelta,tdelta)

        S1,S2 = getCorrespondences(object, sceneTransformed)

        scene_full = sceneTransformed

        # Rdelta,tdelta = icp(model_full['positions'],sceneTransformed)

        print "iter",iter+1, len(S1), "correspondences, mean squared error",float(sum_of_squared_errors(S1,S2))/len(S1)
    #run the ICP
    # R,t = icp(model_full['positions'],scene_full)

        #apply the ICP results to the model
        transformed_point = [se3.apply((Rdelta,tdelta),p) for p in model_full['positions']]
        model_full['positions'] = transformed_point

        #visualize the results
        opengl_plot = OpenGLPlot(model_full, scene_full)
        opengl_plot.initialize_main_loop()

        #compose the new rotation with the old rotation
"""

    # If matplot is available you can use it to visualize the points (but it not
    # required) as in the commented line
    # matplot_points(model_full, clpoints_out)

if __name__ == "__main__":
    main()
