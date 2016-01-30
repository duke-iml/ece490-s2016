from klampt import se3
from utils import *
from scipy import spatial
from scipy import cluster
import numpy
import heapq
import random

#this is the object that we know is in the scene
#TODO: play around with this object, it can be "highlighters", "kong_duck", "kygen_eggs", and "school_glue"
object = "school_glue"

# Read the model and the scene point cloud
model_file = "data/models/"+object+"_model.json"
depth_file = "data/processed_depth/"+object+".json"

def euclidean_distance_2(P1, P2):
    """Compute the Squared Euclidean distance between 2 3D points"""
    dist = (P1[0]-P2[0])**2+(P1[1]-P2[1])**2+(P1[2]-P2[2])**2
    return dist

def sum_of_squared_errors(P1,P2):
    """For the corresponding points P1 and P2, returns the sum of squared
    errors"""
    return sum(euclidean_distance_2(p1,p2) for (p1,p2) in zip(P1,P2))

def get_correspondences(P1, P2):

    P2matches = get_closest_points_3d(P1, P2)

    return P1,P2matches

def get_closest_points_3d(P1, P2):
    """Compute the closest 2D points searching exhaustively. The output has
    the same dimension as P1 and produces the closest point in P2 to P1.
    Input points: [[x1,y1],[x2,y2],...]"""

    A_1 = []
    A_2 = []

    for p1_id in range(0,len(P1)):
        D_1 = 1000000
        D_2 = 0
        P_1 = [0,0]

        for p2_id in range(0,len(P2)):
            distance2 = euclidean_distance_2(P1[p1_id],P2[p2_id])

            if D_2 < D_1:
                P_1 = P2[p2_id]
                D_1 = D_2

        A_1.append(P_1)
        A_2.append(D_1)

    return A_1

def subtract_mean(point_list):
    """Input list: ((x1,y1),(x2,y2),...)
    Returns a pair ([p1-mean,...,pn-mean], mean)"""

    sumX = 0
    sumY = 0
    sumZ = 0
    n = 0;

    for point in point_list:
        sumX = sumX + point[0]
        sumY = sumY + point[1]
        sumZ = sumZ + point[2]
        n = n + 1

    mean_x = (1/(float(n)))*sumX
    mean_y = (1/(float(n)))*sumY
    mean_z = (1/(float(n)))*sumZ


    point_list_minus_mean = []
    for t in range(0, len(point_list)):
        point_list_minus_mean.append([point_list[t][0] - mean_x, point_list[t][1] - mean_y, point_list[t][2] - mean_z])

    return point_list_minus_mean, [mean_x, mean_y, mean_z]


def compute_error_minimizing_rotation(Points1, Points2):
    """Compute the rotation matrix that rotates the points in Points2 to minimize
    squared error to the points in Points1"""

    H_1_1 = 0
    H_1_2 = 0
    H_1_3 = 0
    H_2_1 = 0
    H_2_2 = 0
    H_2_3 = 0
    H_3_1 = 0
    H_3_2 = 0
    H_3_3 = 0

    for i in range(1, len(Points1)):
        H_1_1 = H_1_1 + (Points1[i][0] * Points2[i][0])
        H_1_2 = H_1_2 + (Points1[i][0] * Points2[i][1])
        H_1_3 = H_1_2 + (Points1[i][0] * Points2[i][2])

        H_2_1 = H_1_1 + (Points1[i][1] * Points2[i][0])
        H_2_2 = H_1_2 + (Points1[i][1] * Points2[i][1])
        H_2_3 = H_1_2 + (Points1[i][1] * Points2[i][2])

        H_3_1 = H_1_1 + (Points1[i][2] * Points2[i][0])
        H_3_2 = H_1_2 + (Points1[i][2] * Points2[i][1])
        H_3_3 = H_1_2 + (Points1[i][2] * Points2[i][2])

    H = [[H_1_1,H_1_2,H_1_3],[H_2_1,H_2_2,H_2_3],[H_3_1,H_3_2,H_3_3]]

    U, S, V = numpy.linalg.svd(H)

    V = numpy.transpose(V)

    R11 = (U[0][0] * V[0][0]) +((U[0][1] * V[1][0]))+ ((U[0][2] * V[2][0]))
    R12 = (U[0][0] * V[0][1]) +((U[0][1] * V[1][1]))+ ((U[0][2] * V[2][1]))
    R13 = (U[0][0] * V[0][2]) +((U[0][1] * V[1][2]))+ ((U[0][2] * V[2][2]))

    R21 = (U[1][0] * V[0][0]) +((U[1][1] * V[1][0]))+ ((U[1][2] * V[2][0]))
    R22 = (U[1][0] * V[0][1]) +((U[1][1] * V[1][1]))+ ((U[1][2] * V[2][1]))
    R23 = (U[1][0] * V[0][2]) +((U[1][1] * V[1][2]))+ ((U[1][2] * V[2][2]))

    R31 = (U[2][0] * V[0][0]) +((U[2][1] * V[1][0]))+ ((U[2][2] * V[2][1]))
    R32 = (U[2][0] * V[0][1]) +((U[2][1] * V[1][1]))+ ((U[2][2] * V[2][1]))
    R33 = (U[2][0] * V[0][2]) +((U[2][1] * V[1][2]))+ ((U[2][2] * V[2][2]))

    R = [[R11,R12,R13],[R21,R22,R23],[R31,R32,R33]]

    return R

def B(P1,P2):
    return numpy.sum(euclidean_distance_2(P1,P2) for (p1,p1) in zip(P1,P2))

def icp(object,scene):
    """Computes a rotation and translation of the object to the scene
    using the ICP algorithm.  Returns the transform (R,t) in Klamp't se3
    format.  
    """
    sampleObject = random.sample(object,int(float(len(object))*0.05))
    sampleScene = random.sample(scene,int(float(len(scene))*0.05))

    S1,S2 = get_correspondences(sampleScene, sampleObject)
    S1_shift, mean1 = subtract_mean(S1)
    S2_shift, mean2 = subtract_mean(S2)
    R = compute_error_minimizing_rotation(S1_shift,S2_shift)
    RSe3 = [R[0][0],R[0][1],R[0][2],R[1][0],R[1][1],R[1][2],R[2][0],R[2][1],R[2][2]]
    RM1 = (R[0][0] * mean2[0]) + (R[0][1] * mean2[1]) + (R[0][2] * mean2[2])
    RM2 = (R[1][0] * mean2[0]) + (R[1][1] * mean2[1]) + (R[1][2] * mean2[2])
    RM3 = (R[2][0] * mean2[0]) + (R[2][1] * mean2[1]) + (R[2][2] * mean2[2])
    t = [-(mean1[0] - RM1), -(mean1[1] - RM2), -(mean1[2] - RM3)]

    return RSe3, t

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


    #I was not able to finish this because I have four projects for my research that I am in charge of!
