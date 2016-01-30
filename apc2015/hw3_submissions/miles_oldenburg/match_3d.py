from klampt import se3
import numpy
from utils import *
import random
import math
import copy

#this is the object that we know is in the scene
#TODO: play around with this object, it can be "highlighters", "kong_duck", "kygen_eggs", and "school_glue"
object = "highlighters"

# Read the model and the scene point cloud
model_file = "data/models/"+object+"_model.json"
depth_file = "data/processed_depth/"+object+".json"

def euclidean_distance_3(P1, P2):
    """
    Compute the Squared Euclidean distance between 2 3D points
    """
    return (P1[0] - P2[0]) ** 2 + (P1[1] - P2[1]) ** 2 + (P1[2] - P2[2]) ** 2

def get_closest_points_3d(P1, P2):
    """
    Compute the closest 2D points searching exhaustively. The output has
    the same dimension as P1 and produces the closest point in P2 to P1.
    Input points: [[x1,y1],[x2,y2],...]
    """

    # Initialize list of closest points
    closestPoints = []

    # Loop every point in P1
    for point in P1:
        # Initially choose the minimum distance to the first pair
        minimumIndex = 0
        minimumDistance = euclidean_distance_3(point, P2[0])

        # Check distance to every point in P2
        for index in range(1, len(P2)):
            # Calculate to every point in
            distance = euclidean_distance_3(point, P2[index])

            # If distance is more than existing minimum, set new minimum
            if distance < minimumDistance:
                minimumIndex = index
                minimumDistance = distance

        # Now that we have searched for the minimum distance add that point from P2 to our list
        closestPoints.append(P2[minimumIndex])

    return closestPoints

def calculate_mean_and_median(P1):
    """
    Returns a tuple of the mean and median of float list P1
    """

    mean = numpy.mean(P1)
    median = numpy.median(P1)

    return mean, median

def calculate_median_absolute_deviation(P1):
    """
    Returns the median absolute deviation (MAD) of the list
    :param P1: the list
    :return: MAD
    """

    # Get mean and median of distances
    mean, median = calculate_mean_and_median(P1)

    # Calculate absolute deviation of distances
    absoluteDeviations = []
    for distance in P1:
        absoluteDeviations.append(math.fabs(distance - median))

    # Calculate median absolute deviation
    absoluteMean, absoluteMedian = calculate_mean_and_median(absoluteDeviations)

    return absoluteMedian

def threshold_closest_points(P1, P2, threshold):
    """
    Discard the pairs of points for which their mutual distance exceeds the
    predefined threshold
    Input points: [[x1,y1],[x2,y2],...]
    """

    # Examine every point in P1
    for index, point in enumerate(P1):
        # Calculate distance
        distance = euclidean_distance_3(P1[index], P2[index])

        # If distance between points exceeds our threshold, delete the pair from both lists
        if distance > threshold:
            del P1[index]
            del P2[index]

    return P1, P2

def get_correspondences(P1, P2):

    """Returns a pair of lists  (S1,S2) where S1[i] is an element of P1 that
    corresponds to S2[i], which is an element of P2."""
    # Find the closest points to triangle 1 by exhaustive search using the
    # squared Euclidean distance
    closestPoints = get_closest_points_3d(P1, P2)

    # The matching pairs may contain irrelevant data. Keep only the matching
    # points that are close enough within a threshold parameter

    # Reject the outliers, for example, using a threshold
    # Create list of euclidean distances between for closest points
    distances = []
    for index in range(0, len(P1)):
        distance = euclidean_distance_3(P1[index], closestPoints[index])
        distances.append(distance)

    # Calculate MAD
    mad = calculate_median_absolute_deviation(distances)

    # Prune distances that are already close to matching
    prunedDistances = []
    for distance in distances:
        if distance > mad:
            prunedDistances.append(distance)

    # Calculate SD on pruned distances
    std = numpy.std(prunedDistances)

    # Set threshold
    threshold = 40000.0 * std

    # Determine a good threshold.  Set the 'threshold' variable
    # to be your best choice, and write your justification to this answer
    # in the below:
    #
    # After some experimentation I used the standard deviation to set the threshold. The problem was that some points
    # converged quickly while others were still far away. I tried to solve this by pruning out some of the close points
    # before trying to detect outliers

    S1, S2 = threshold_closest_points(P1, closestPoints, threshold)

    return S1,S2

def subtract_mean(point_list):
    """
    Input list: ((x1,y1),(x2,y2),...)
    Returns a pair ([p1-mean,...,pn-mean], mean)
    """

    # Calculate mean of x and y
    mean = numpy.mean(point_list, axis=0)

    mean_x = mean[0]
    mean_y = mean[1]
    mean_z = mean[2]

    # Subtract mean from every point
    point_list_minus_mean = point_list
    for point in point_list_minus_mean:
        point = [point[0] - mean_x, point[1] - mean_y, point[2] - mean_z]

    return point_list_minus_mean, [mean_x, mean_y, mean_z]

def compute_centroid(points):
    xsum = reduce(lambda sum, (x,y,z): sum + x, points, 0)
    ysum = reduce(lambda sum, (x,y,z): sum + y, points, 0)
    zsum = reduce(lambda sum, (x,y,z): sum + z, points, 0)

    return xsum / len(points), ysum / len(points), zsum / len(points)

def compute_error_minimizing_rotation(Points1, Points2):
    """
    Compute the rotation matrix that rotates the points in Points2 to minimize
    squared error to the points in Points1
    """

    # Compute centroids
    p1x, p1y, p1z = compute_centroid(Points1)
    p2x, p2y, p2z = compute_centroid(Points2)

    # Subtact centroids from initial points
    p1tilde = []
    for p1i in Points1:
        p1tilde.append([p1i[0] - p1x, p1i[1] - p1y, p1i[2] - p1z])

    p2tilde = []
    for p2i in Points2:
        p2tilde.append([p2i[0] - p2x, p2i[1] - p2y, p2i[2] - p2z])

    # Compute values for covariance matrix
    c = [0] * 9

    for index in range(0, len(Points1)):
        pix = p1tilde[index][0]
        piy = p1tilde[index][1]
        piz = p1tilde[index][2]
        rix = p2tilde[index][0]
        riy = p2tilde[index][1]
        riz = p2tilde[index][2]

        c[0] += pix * rix
        c[1] += pix * riy
        c[2] += pix * riz
        c[3] += piy * rix
        c[4] += piy * riy
        c[5] += piy * riz
        c[6] += piz * rix
        c[7] += piz * riy
        c[8] += piz * riz

    # Construct covariance matrix
    H = numpy.array([[c[0], c[1], c[2]], [c[3], c[4], c[5]], [c[6], c[7], c[8]]])

    # Compute SVD decomposition
    svd = numpy.linalg.svd(H)

    # Find rotation using values from SVD decomp
    R = svd[0].dot(svd[2].transpose())
    R2 = svd[2].dot(svd[0].transpose())

    return R, R2

def computeSamples(object, scene):
    # Sample both maps to make the closest point computations faster
    objectSamplePercentage = .01
    sceneSamplePercentage = .01

    objectSamples = []
    sceneSamples = []

    for i in range(1, int(math.floor((len(object) * objectSamplePercentage)))):
        objectSamples.append(object[random.randint(0, len(object) - 1)])

    for i in range(1, int(math.floor((len(scene) * sceneSamplePercentage)))):
        sceneSamples.append(scene[random.randint(0, len(scene) - 1)])

    return objectSamples, sceneSamples

def sum_of_squared_errors(P1,P2):
    """For the corresponding points P1 and P2, returns the sum of squared
    errors"""
    return sum(euclidean_distance_3(p1,p2) for (p1,p2) in zip(P1,P2))

def icp(object,scene):
    """Computes a rotation and translation of the object to the scene
    using the ICP algorithm.  Returns the transform (R,t) in Klamp't se3
    format.  
    """

    # Compute samples
    #objectSamples, sceneSamples = computeSamples(object, scene)

    #Run ICP for 20 iterations
    for iters in range(20):
        #print "iter",iters

        # Compute samples
        objectSamples, sceneSamples = computeSamples(object, scene)

        #get the correspondences
        S1,S2 = get_correspondences(objectSamples, sceneSamples)

        print "iter",iters,len(S1),"correspondences, mean squared error",float(sum_of_squared_errors(S1,S2))/len(S1)

        # Center the resulting pairs substracting their means
        S1_shift, mean1 = subtract_mean(S1)
        S2_shift, mean2 = subtract_mean(S2)

        #calculate the error-minimizing rotation
        R, R2 = compute_error_minimizing_rotation(S1_shift,S2_shift)

        Rk = [R[0][0], R[1][0], R[2][0], R[0][1], R[1][1], R[2][1], R[0][2], R[1][2], R[2][2]]
        Rk2 = [R2[0][0], R2[1][0], R2[2][0], R2[0][1], R2[1][1], R2[2][1], R2[0][2], R2[1][2], R2[2][2]]

        #find the t such that R*p+t = R*(p-mean2)+mean1
        Rmean2 = [R[0][0] * mean2[0] + R[0][1] * mean2[1] + R[0][2] * mean2[2],
                  R[1][0] * mean2[0] + R[1][1] * mean2[1] + R[1][2] * mean2[2],
                  R[2][0] * mean2[0] + R[2][1] * mean2[1] + R[2][2] * mean2[2]]
        
        #find the t such that R*p+t = R*(p-mean2)+mean1
        Rmean22 = [R2[0][0] * mean2[0] + R2[0][1] * mean2[1] + R2[0][2] * mean2[2],
                  R2[1][0] * mean2[0] + R2[1][1] * mean2[1] + R2[1][2] * mean2[2],
                  R2[2][0] * mean2[0] + R2[2][1] * mean2[1] + R2[2][2] * mean2[2]]

        t = [mean1[0] - Rmean2[0], mean1[1] - Rmean2[1], mean1[2] - Rmean2[2]]
        t2 = [mean1[0] - Rmean22[0], mean1[1] - Rmean22[1], mean1[2] - Rmean22[2]]

        tran1 = copy.deepcopy(object)
        tran2 = copy.deepcopy(object)

        #apply the ICP results to the model
        transformed_points = [se3.apply((Rk,t),p) for p in tran1]
        transformed_points2 = [se3.apply((Rk2,t2),p) for p in tran2]

        # Compute samples
        objectSamples1, sceneSamples1 = computeSamples(transformed_points, scene)
        objectSamples2, sceneSamples2 = computeSamples(transformed_points2, scene)

        #get the correspondences
        S1a,S2a = get_correspondences(objectSamples1, sceneSamples1)
        S1b,S2b = get_correspondences(objectSamples2, sceneSamples2)

        R1error = float(sum_of_squared_errors(S1a,S2a))/len(S1a)
        R2error = float(sum_of_squared_errors(S1b,S2b))/len(S1b)

        print "mean squared error 1",R1error
        print "mean squared error 2",R2error

        if R1error < R2error:
            object = transformed_points
        else:
            object = transformed_points2
            Rk = Rk2
            t = t2

    return Rk, t

def main():
    """
    Main loop.  Run ICP on the given model and scene file, display the results.
    """
    model_full = get_reconstructed_model(model_file,True)
    scene_full = get_raw_depth(depth_file)

    #run the ICP
    R,t = icp(model_full['positions'], scene_full)

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
