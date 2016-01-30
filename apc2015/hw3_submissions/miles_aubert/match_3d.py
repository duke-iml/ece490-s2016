from klampt import se3
from utils import *
from scipy import spatial
from scipy import cluster
import numpy
import heapq
import random

#this is the object that we know is in the scene
#TODO: play around with this object, it can be "highlighters", "kong_duck", "kygen_eggs", and "school_glue"
object = "highlighters"

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

def threshold_closest_points(P1, P2, threshold):
    """Discard the pairs of points for which their mutual distance exceeds the
    predefined threshold
    Input points: [[x1,y1],[x2,y2],...]"""

    #Initialize output array
    filteredPointsP1 = []
    filteredPointsP2 = []

    #Filter based on distance threshold
    for index in range(0,len(P1)):
        if euclidean_distance_2(P1[index],P2[index]) <= threshold:
            filteredPointsP2.append(P2[index])
            filteredPointsP1.append(P1[index])

    return filteredPointsP2, filteredPointsP1

def get_correspondences(P1, P2):

    P2matches, P2distances = get_closest_points_3d(P1, P2)

    threshold = numpy.var(P2distances)*4

    S1, S2 = threshold_closest_points(P1,P2matches,threshold)

    return S1,S2

def get_closest_points_3d(P1, P2):
    """Compute the closest 2D points searching exhaustively. The output has
    the same dimension as P1 and produces the closest point in P2 to P1.
    Input points: [[x1,y1],[x2,y2],...]"""

    outArray = []
    distArray = []

    for p1_id in range(0,len(P1)):
        distance1 = 100000
        distance2 = 0
        closePoint = [0,0]

        for p2_id in range(0,len(P2)):
            distance2 = euclidean_distance_2(P1[p1_id],P2[p2_id])

            if distance2 < distance1:
                closePoint = P2[p2_id]
                distance1 = distance2

        outArray.append(closePoint)
        distArray.append(distance1)

    if len(outArray) == len(P1):
        return outArray, distArray
    else:
        return -1

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

    H11 = 0
    H12 = 0
    H13 = 0
    H21 = 0
    H22 = 0
    H23 = 0
    H31 = 0
    H32 = 0
    H33 = 0

    for i in range(1, len(Points1)):
        H11 = H11 + (Points1[i][0] * Points2[i][0])
        H12 = H12 + (Points1[i][0] * Points2[i][1])
        H13 = H12 + (Points1[i][0] * Points2[i][2])

        H21 = H11 + (Points1[i][1] * Points2[i][0])
        H22 = H12 + (Points1[i][1] * Points2[i][1])
        H23 = H12 + (Points1[i][1] * Points2[i][2])

        H31 = H11 + (Points1[i][2] * Points2[i][0])
        H32 = H12 + (Points1[i][2] * Points2[i][1])
        H33 = H12 + (Points1[i][2] * Points2[i][2])

    H = [[H11,H12,H13],[H21,H22,H23],[H31,H32,H33]]

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

    U = numpy.transpose(U)
    V = numpy.transpose(V)

    mR11 = (V[0][0] * U[0][0]) +((V[0][1] * U[1][0]))+ ((V[0][2] * U[2][0]))
    mR12 = (V[0][0] * U[0][1]) +((V[0][1] * U[1][1]))+ ((V[0][2] * U[2][1]))
    mR13 = (V[0][0] * U[0][2]) +((V[0][1] * U[1][2]))+ ((V[0][2] * U[2][2]))

    mR21 = (V[1][0] * U[0][0]) +((V[1][1] * U[1][0]))+ ((V[1][2] * U[2][0]))
    mR22 = (V[1][0] * U[0][1]) +((V[1][1] * U[1][1]))+ ((V[1][2] * U[2][1]))
    mR23 = (V[1][0] * U[0][2]) +((V[1][1] * U[1][2]))+ ((V[1][2] * U[2][2]))

    mR31 = (V[2][0] * U[0][0]) +((V[2][1] * U[1][0]))+ ((V[2][2] * U[2][1]))
    mR32 = (V[2][0] * U[0][1]) +((V[2][1] * U[1][1]))+ ((V[2][2] * U[2][1]))
    mR33 = (V[2][0] * U[0][2]) +((V[2][1] * U[1][2]))+ ((V[2][2] * U[2][2]))

    mR = [[mR11,mR12,mR13],[mR21,mR22,mR23],[mR31,mR32,mR33]]

    return R,mR

def icp(object,scene):
    """Computes a rotation and translation of the object to the scene
    using the ICP algorithm.  Returns the transform (R,t) in Klamp't se3
    format.  
    """

    sampleObject = random.sample(object,int(float(len(object))*0.01))

    S1,S2 = get_correspondences(scene, sampleObject)

    if len(S1) == 0:
        return [-1],[-1],[-1],[-1]

    if len(S2) == 0:
        return [-1],[-1],[-1],[-1]


    S1_shift, mean1 = subtract_mean(S1)
    S2_shift, mean2 = subtract_mean(S2)

    R, mR = compute_error_minimizing_rotation(S1_shift,S2_shift)
    RSe3 = [R[0][0],R[0][1],R[0][2],R[1][0],R[1][1],R[1][2],R[2][0],R[2][1],R[2][2]]
    RM1 = (R[0][0] * mean2[0]) + (R[0][1] * mean2[1]) + (R[0][2] * mean2[2])
    RM2 = (R[1][0] * mean2[0]) + (R[1][1] * mean2[1]) + (R[1][2] * mean2[2])
    RM3 = (R[2][0] * mean2[0]) + (R[2][1] * mean2[1]) + (R[2][2] * mean2[2])
    t = [-(mean1[0] - RM1), -(mean1[1] - RM2), -(mean1[2] - RM3)]

    mRSe3 = [mR[0][0],mR[0][1],mR[0][2],mR[1][0],mR[1][1],mR[1][2],mR[2][0],mR[2][1],mR[2][2]]
    mRM1 = (mR[0][0] * mean2[0]) + (mR[0][1] * mean2[1]) + (mR[0][2] * mean2[2])
    mRM2 = (mR[1][0] * mean2[0]) + (mR[1][1] * mean2[1]) + (mR[1][2] * mean2[2])
    mRM3 = (mR[2][0] * mean2[0]) + (mR[2][1] * mean2[1]) + (mR[2][2] * mean2[2])
    mt = [-(mean1[0] - mRM1), -(mean1[1] - mRM2), -(mean1[2] - mRM3)]

    return RSe3, t, mRSe3, mt

def ApplyICP(object, scene):

    outR = [];
    outT = [];

    SSEm1 = 1000000000

    for i in range(0,5):

        SSE = 1000000
        SSE_min = 10000000
        SSE_max = 10000000

        #run the ICP
        R,t,mR,mt = icp(object, scene)

        if len(R) > 1:
            #apply the ICP results to the model
            maxModel = [se3.apply((R,t),p) for p in object]
            maxSample = random.sample(object,int(float(len(object))*0.01))
            SSE_max = sum_of_squared_errors(maxSample,scene)
        elif R == -1:
            SSE_max = 10000000

        if len(mR) > 1:
            minModel = [se3.apply((mR,mt),p) for p in object]
            minSample = random.sample(object,int(float(len(object))*0.01))
            SSE_min = sum_of_squared_errors(minSample,scene)
        else:
            SSE_min = 10000000

        if SSE_min == 10000000 and SSE_max == 10000000:
            SSE = 1000000
            break
        elif SSE_max <= SSE_min:
            object = maxModel
            print SSE_max
            SSE = SSE_max
            tempR = R
            tempT = t
        else:
            object = minModel
            print SSE_min
            SSE = SSE_min
            tempR = mR
            tempT = mt

        if SSEm1 > SSE:
            SSEm1 = SSE
            outR = tempR
            outT = tempT
            print SSE

    return outR, outT

def main():
    """
    Main loop.  Run ICP on the given model and scene file, display the results.
    """
    model_full = get_reconstructed_model(model_file,True)
    scene_full = get_raw_depth(depth_file)

    R,t = ApplyICP(model_full['positions'],scene_full)


    #apply the ICP results to the model
    model = [se3.apply((R,t),p) for p in model_full['positions']]

    #visualize the results
    opengl_plot = OpenGLPlot(model, scene_full)
    opengl_plot.initialize_main_loop()

    # If matplot is available you can use it to visualize the points (but it not
    # required) as in the commented line
    # matplot_points(model_full, clpoints_out)

if __name__ == "__main__":
    main()
