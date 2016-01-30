from PIL import Image
import numpy 
import math

#if matplotlib is available on your system, use it to display the iterations
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    print "Matplotlib is not available."
    print "You won't get the advantage of watching the ICP convergence..."

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

    A_1 = []
    A_2 = []

    for p1_id in range(0,len(P1)):
        d_i = 100000
        d_f = 0

        Point = []

        for p2_id in range(0,len(P2)):
            d_f = euclidean_distance_2(P1[p1_id],P2[p2_id])

            if d_f < d_i:
                Point = P2[p2_id]
                d_i = d_f

        A_1.append(Point)
        A_2.append(d_i)

    return A_1, A_2

def threshold_closest_points(P1, P2, threshold):
    """
    Discard the pairs of points for which their mutual distance exceeds the
    predefined threshold
    Input points: [[x1,y1],[x2,y2],...]
    """

    discard_1 = []
    discard_2 = []

    for index in range(0,len(P1)):
        if euclidean_distance_2(P1[index],P2[index]) <= threshold:
            discard_2.append(P2[index])
            discard_1.append(P1[index])

    return discard_2, discard_1

def get_correspondences(P1, P2):
    """Returns a pair of lists  (S1,S2) where S1[i] is an element of P1 that
    corresponds to S2[i], which is an element of P2."""
    # Find the closest points to triangle 1 by exhaustive search using the
    # squared Euclidean distance
    P2matches, A_2 = get_closest_points_2d(P1, P2)

    # The matching pairs may contain irrelevant data. Keep only the matching
    # points that are close enough within a threshold parameter

    threshold = numpy.std(A_2)*2
    # TODO: determine a good threshold.  Set the 'threshold' variable
    # to be your best choice, and write your justification to this answer
    # in the below:
    #
    # [Task 4 answer goes here]

    # Question 4:  In selecting to use the standard deviation as how to evaluate the threshold, my code ensures that
    # the threshold is strictly dependent on any array of points (i.e. will work regardless of images geometry). Also,
    # by taking two standard deviations as the threshold, some uncertainty in the calculations is accounted for.
    # Finally, this threshold was found to minimize the mean squared error over 20 iterations for this specific
    # problem. The final MSE should be 5.35.

    S1, S2 = threshold_closest_points(P1,P2matches,threshold)

    return S1,S2

def subtract_mean(point_list):
    """
    Input list: ((x1,y1),(x2,y2),...)
    Returns a pair ([p1-mean,...,pn-mean], mean)
    """
    #TODO: implement me
    mean_x = 0
    mean_y = 0

    sigma_x = 0
    sigma_y = 0

    n = 0

    for point in point_list:
        sigma_x = sigma_x + point[0]
        sigma_y = sigma_y + point[1]
        n = n + 1

    mean_x = sigma_x/(float(n))
    mean_y = sigma_y/(float(n))

    point_list_minus_mean = []
    for i in range(0, len(point_list)):
        point_list_minus_mean.append([point_list[i][0] - mean_x, point_list[i][1] - mean_y])

    return point_list_minus_mean, [mean_x, mean_y]

def compute_error_minimizing_rotation(Points1, Points2):
    """
    Compute the rotation matrix that rotates the points in Points2 to minimize
    squared error to the points in Points1
    """
    #TODO: implement me

    H_1_1 = 0
    H_1_2 = 0
    H_2_1 = 0
    H_2_2 = 0

    for t in range(1, len(Points1)):
        H_1_1 = H_1_1 + (Points1[t][0] * Points2[t][0])
        H_1_2 = H_1_2 + (Points1[t][1] * Points2[t][0])
        H_2_1 = H_2_1 + (Points1[t][0] * Points2[t][1])
        H_2_2 = H_2_2 + (Points1[t][1] * Points2[t][1])

    H = [[H_1_1,H_1_2],[H_2_1,H_2_2]]

    U, S, V = numpy.linalg.svd(H)

    V = numpy.transpose(V)

    R_1_1 = (U[0][0] * V[0][0]) +((U[0][1] * V[1][0]))
    R_1_2 = (U[0][0] * V[0][1]) +((U[0][1] * V[1][1]))
    R_2_1 = (U[1][0] * V[0][0]) +((U[1][1] * V[1][0]))
    R_2_2 = (U[1][0] * V[0][1]) +((U[1][1] * V[1][1]))

    R = [[R_1_1,R_1_2],[R_2_1,R_2_2]]

    return R

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
    R = compute_error_minimizing_rotation(S1_shift,S2_shift)
    #find the t such that R*p+t = R*(p-mean2)+mean1
    Rmean2 = [R[0][0]*mean2[0]+R[0][1]*mean2[1],
              R[1][0]*mean2[0]+R[1][1]*mean2[1]]

    return R,[-(mean1[0]-Rmean2[0]),-(mean1[1]-Rmean2[1])]


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

#Run ICP for 20 iterations
for iters in range(20):
    tri2transformed = transform_points(triangle2,R,t)
    S1,S2 = get_correspondences(triangle1,tri2transformed)
    if MATPLOTLIB_AVAILABLE:
        #plot the transformed triangles and their correspondences
        print triangle1
        print tri2transformed
        plt.scatter([p[0] for p in triangle1],[p[1] for p in triangle1],
                marker="o")
        plt.scatter([p[0] for p in tri2transformed],[p[1] for p in tri2transformed],
                marker="*")
        for p1,p2 in zip(S1,S2):
            plt.plot([p1[0],p2[0]],[p1[1],p2[1]],color='r',linestyle='-')
        plt.show()
        
    Rdelta,tdelta = icp_step(triangle1,tri2transformed)
    print "iter",iters,len(S1),"correspondences, mean squared error",float(sum_of_squared_errors(S1,S2))/len(S1)
    
    #compose the new rotation with the old rotation
    r1 = transform_point([R[0][0],R[1][0]],Rdelta,[0,0])
    r2 = transform_point([R[0][1],R[1][1]],Rdelta,[0,0])
    R[0][0],R[1][0] = r1
    R[0][1],R[1][1] = r2
    t = transform_point(t,Rdelta,tdelta)

#compose the new rotation with the old rotation
r1 = transform_point([R[0][0],R[1][0]],Rdelta,[0,0])
r2 = transform_point([R[0][1],R[1][1]],Rdelta,[0,0])
R[0][0],R[1][0] = r1
R[0][1],R[1][1] = r2
t = transform_point(t,Rdelta,tdelta)
