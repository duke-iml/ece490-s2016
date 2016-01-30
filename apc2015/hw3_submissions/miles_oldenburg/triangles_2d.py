from PIL import Image
import numpy 
import math
import copy

#if matplotlib is available on your system, use it to display the iterations
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    print "Matplotlib is not available."
    print "You won't get the advantage of watching the ICP convergence..."

def compute_centroid(points):
    xsum = reduce(lambda sum, (x,y): sum + x, points, 0)
    ysum = reduce(lambda sum, (x,y): sum + y, points, 0)

    return xsum / len(points), ysum / len(points)

def image_to_points(numpy_image):
    """
    Given an PIL Image object, extract the 2D point cloud.
    The result should be a list of points [[x1,y1],...,[xn,yn]]
    """
    res = []
    for i in range(numpy_image.shape[0]):
        for j in range(numpy_image.shape[1]):
            if numpy_image[i,j]==0:
                res.append([i,j])
    return res

def euclidean_distance_2(P1, P2):
    """
    Compute the Squared Euclidean distance between 2 2D points
    """
    return (P1[0]-P2[0])**2+(P1[1]-P2[1])**2

def sum_of_squared_errors(P1,P2):
    """For the corresponding points P1 and P2, returns the sum of squared
    errors"""
    return sum(euclidean_distance_2(p1,p2) for (p1,p2) in zip(P1,P2))

def get_closest_points_2d(P1, P2):
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
        minimumDistance = euclidean_distance_2(point, P2[0])

        # Check distance to every point in P2
        for index in range(1, len(P2)):
            # Calculate to every point in
            distance = euclidean_distance_2(point, P2[index])

            # If distance is more than existing minimum, set new minimum
            if distance < minimumDistance:
                minimumIndex = index
                minimumDistance = distance

        # Now that we have searched for the minimum distance add that point from P2 to our list
        closestPoints.append(P2[minimumIndex])

    return closestPoints

def threshold_closest_points(P1, P2, threshold):
    """
    Discard the pairs of points for which their mutual distance exceeds the
    predefined threshold
    Input points: [[x1,y1],[x2,y2],...]
    """

    # Examine every point in P1
    for index, point in enumerate(P1):
        # Calculate distance
        distance = euclidean_distance_2(P1[index], P2[index])

        # If distance between points exceeds our threshold, delete the pair from both lists
        if distance > threshold:
            del P1[index]
            del P2[index]

    return P1, P2

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

def get_correspondences(P1, P2):

    """Returns a pair of lists  (S1,S2) where S1[i] is an element of P1 that
    corresponds to S2[i], which is an element of P2."""
    # Find the closest points to triangle 1 by exhaustive search using the
    # squared Euclidean distance
    P2matches = get_closest_points_2d(P1, P2)

    # The matching pairs may contain irrelevant data. Keep only the matching
    # points that are close enough within a threshold parameter

    # Create list of euclidean distances between for closest points
    distances = []
    for index in range(0, len(P1)):
        distance = euclidean_distance_2(P1[index], P2matches[index])
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
    threshold = 4.0 * std

    # Determine a good threshold.  Set the 'threshold' variable
    # to be your best choice, and write your justification to this answer
    # in the below:
    #
    # After some experimentation I used the standard deviation to set the threshold. The problem was that some points
    # converged quickly while others were still far away. I tried to solve this by pruning out some of the close points
    # before trying to detect outliers
    
    S1, S2 = threshold_closest_points(P1,P2matches,threshold)

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

    # Subtract mean from every point
    point_list_minus_mean = point_list
    for point in point_list_minus_mean:
        point = [point[0] - mean_x, point[1] - mean_y]

    return point_list_minus_mean, [mean_x, mean_y]

def compute_error_minimizing_rotation(Points1, Points2):
    """
    Compute the rotation matrix that rotates the points in Points2 to minimize
    squared error to the points in Points1
    """

    # Compute centroids
    p1x, p1y = compute_centroid(Points1)
    p2x, p2y = compute_centroid(Points2)

    # Subtact centroids from initial points
    p1tilde = []
    for p1i in Points1:
        p1tilde.append([p1i[0] - p1x, p1i[1] - p1y])

    p2tilde = []
    for p2i in Points2:
        p2tilde.append([p2i[0] - p2x, p2i[1] - p2y])

    # Compute values for covariance matrix
    p1tildexp2tildex = 0
    p1tildexp2tildey = 0
    p1tildeyp2tildex = 0
    p1tildeyp2tildey = 0

    for index in range(0, len(Points1)):
        p1tildexp2tildex += p1tilde[index][0] * p2tilde[index][0]
        p1tildexp2tildey += p1tilde[index][0] * p2tilde[index][1]
        p1tildeyp2tildex += p1tilde[index][1] * p2tilde[index][0]
        p1tildeyp2tildey += p1tilde[index][1] * p2tilde[index][1]

    # Construct covariance matrix
    H = numpy.array([[p1tildexp2tildex, p1tildexp2tildey], [p1tildeyp2tildex, p1tildeyp2tildey]])

    # Compute SVD decomposition
    svd = numpy.linalg.svd(H)

    # Find rotation using values from SVD decomp
    R = svd[0].dot(svd[2].transpose())
    R2 = svd[2].dot(svd[0].transpose())

    return R, R2

def transform_point(p,R,t):
    """Compute the point p transformed by the rotation R and translation t"""
    x = R[0][0]*p[0]+R[0][1]*p[1]+t[0]
    y = R[1][0]*p[0]+R[1][1]*p[1]+t[1]
    return [x,y]

def transform_points(Points,R,t):
    """Compute the points transformed by the rotation R and translation t"""
    return [transform_point(p,R,t) for p in Points]

def icp_step(Points1,Points2):
    """
    Performs 1 step of ICP, given two point clouds.

    Return value should be a pair (R,t) containing the
    new rotation and translation of Points2.
    """
    #get the correspondences
    S1,S2 = get_correspondences(Points1,Points2)

    # Center the resulting pairs substracting their means
    S1_shift, mean1 = subtract_mean(S1)
    S2_shift, mean2 = subtract_mean(S2)

    #calculate the error-minimizing rotation
    R, R2 = compute_error_minimizing_rotation(S1_shift,S2_shift)

    #find the t such that R*p+t = R*(p-mean2)+mean1
    Rmean2 = [R[0][0]*mean2[0]+R[0][1]*mean2[1],
              R[1][0]*mean2[0]+R[1][1]*mean2[1]]

    Rmean22 = [R2[0][0]*mean2[0]+R2[0][1]*mean2[1],
              R2[1][0]*mean2[0]+R2[1][1]*mean2[1]]

    t = [mean1[0]-Rmean2[0],mean1[1]-Rmean2[1]]
    t2 = [mean1[0]-Rmean22[0],mean1[1]-Rmean22[1]]

    return R, t, R2, t2

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#                 MAIN PROGRAM
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

file_triangle1 = "data/2d/image_1.gif"
file_triangle2 = "data/2d/image_2.gif"

# Read the images and convert to gray scale
imageraw1 = numpy.array( Image.open(file_triangle1).convert('L') )
imageraw2 = numpy.array( Image.open(file_triangle2).convert('L') )

# Threshold the images
image1 = 1*(imageraw1>128)
image2 = 1*(imageraw2>128)

# We want to match the triangles (black points with values 0), so we need to get
# their coordinates and discard the points corresponding to the background
# (white points with value 1).
# triangle1 and triangle2 should contain the selected points
triangle1 = image_to_points(image1)
triangle2 = image_to_points(image2)


#set up an initial guess
theta = -1.0
R = [[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]]
t = [0,0]
R2 = [[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]]
t2 = [0,0]

#Run ICP for 20 iterations
for iters in range(20):
    triangle2b = copy.deepcopy(triangle2)

    tri2transformed = transform_points(triangle2,R,t)
    tri2transformed2 = transform_points(triangle2b,R2,t2)

    S1,S2 = get_correspondences(triangle1,tri2transformed)
    S1b,S2b = get_correspondences(triangle1,tri2transformed2)

    error1 = float(sum_of_squared_errors(S1,S2))/len(S1)
    error2 = float(sum_of_squared_errors(S1b,S2b))/len(S1b)

    print "iter", iters, "MSE1:", error1, "MSE2:", error2

    if error2 < error1:
        S1 = S1b
        S2 = S2b
        tri2transformed = tri2transformed2

    if MATPLOTLIB_AVAILABLE:
        #plot the transformed triangles and their correspondences
        plt.scatter([p[0] for p in triangle1],[p[1] for p in triangle1],
                marker="o")
        plt.scatter([p[0] for p in tri2transformed],[p[1] for p in tri2transformed],
                marker="*")

        for p1,p2 in zip(S1,S2):
            plt.plot([p1[0],p2[0]],[p1[1],p2[1]],color='r',linestyle='-')

        plt.show()
        
    Rdelta, tdelta, Rdelta2, tdelta2 = icp_step(triangle1,tri2transformed)

    print "iter",iters,len(S1),"correspondences, mean squared error",float(sum_of_squared_errors(S1,S2))/len(S1)
    
    #compose the new rotation with the old rotation
    r1 = transform_point([R[0][0],R[1][0]],Rdelta,[0,0])
    r2 = transform_point([R[0][1],R[1][1]],Rdelta,[0,0])
    R[0][0],R[1][0] = r1
    R[0][1],R[1][1] = r2
    t = transform_point(t,Rdelta,tdelta)

    r1 = transform_point([R2[0][0],R2[1][0]],Rdelta2,[0,0])
    r2 = transform_point([R2[0][1],R2[1][1]],Rdelta2,[0,0])
    R2[0][0],R2[1][0] = r1
    R2[0][1],R2[1][1] = r2
    t2 = transform_point(t2,Rdelta2,tdelta2)
