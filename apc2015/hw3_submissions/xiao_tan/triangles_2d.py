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
   # print "Euclidean: ", sum(euclidean_distance_2(p1,p2) for (p1,p2) in zip(P1,P2))
    return sum(euclidean_distance_2(p1,p2) for (p1,p2) in zip(P1,P2))

def get_closest_points_2d(P1, P2):
    """
    Compute the closest 2D points searching exhaustively. The output has
    the same dimension as P1 and produces the closest point in P2 to P1.
    Input points: [[x1,y1],[x2,y2],...]
    """
    #TODO: implement me
    pointPairTemp = []
    pointArray = []
    for i in range(len(P1)):
        dist = 100000000
        pointPairTemp = []
        for j in range(len(P2)):
            d = euclidean_distance_2(P1[i], P2[j])
            if d < dist:
                """
                # check if P2[j] is already in pointArray
                if i == 197:
                    pointPairTemp = P2[j]
                    dist = d
                if pj_in_pointArray(pointArray, P2[j]) == False:
                    pointPairTemp = P2[j]
               # print "pointPairTemp", pointPairTemp
                    dist = d
                """
                pointPairTemp = P2[j]
                dist = d
        pointArray.append(pointPairTemp)
    return pointArray
"""
def pj_in_pointArray(pointArray, pj):
    if len(pointArray) <= 0:
        return False
    for x in range(len(pointArray)):
        if pointArray[x] == pj:
            return True
    return False
"""
def threshold_closest_points(P1, P2, threshold):
    """
    Discard the pairs of points for which their mutual distance exceeds the
    predefined threshold
    Input points: [[x1,y1],[x2,y2],...]
    """
    #TODO: implement me
    pointArr = []
    S1 = []
    S2 = []
    # print "P1:", len(P1)
    # print "P2:", len(P2)
    for i in range(len(P1)):
        if euclidean_distance_2(P1[i], P2[i]) <= threshold:
            pointArr.append([P1[i], P2[i]])

    #print "pointArr---", pointArr

    for j in range(len(pointArr)):
        S1.append(pointArr[j][0])
        S2.append(pointArr[j][1])
    #print "S1---", S1
    #print "S2---", S2
    return (S1, S2)

def get_correspondences(P1, P2, iters):
    """Returns a pair of lists  (S1,S2) where S1[i] is an element of P1 that
    corresponds to S2[i], which is an element of P2."""
    # Find the closest points to triangle 1 by exhaustive search using the
    # squared Euclidean distance
    P2matches = get_closest_points_2d(P1, P2)

    # The matching pairs may contain irrelevant data. Keep only the matching
    # points that are close enough within a threshold parameter
    threshold = 1200
    # TODO: determine a good threshold.  Set the 'threshold' variable
    # to be your best choice, and write your justification to this answer
    # in the below:
    # 
    ### Why the threshold is 1200:
        # test threshold = [300:100:3000];
        # when threshold increases, the error after 20 iterations decreases;
        # when threshold is greater than 1200, the difference between the errors of 
            # the last two iterations is less than 1 (although keep increasing the threshold 
            # may get a lower error after 20 iterations, the difference is not huge)
    ###
    # [Task 4 answer goes here]
    #
    
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
    for i in range(len(point_list)):
        mean_x = mean_x + point_list[i][0]
        mean_y = mean_y + point_list[i][1]
    mean_x = mean_x / len(point_list)
    mean_y = mean_y / len(point_list)

    # point_list_minus_mean = point_list
    #print "test:", point_list[0][0] - mean_x
    #print "point_list,", point_list

    point_list_minus_mean = []
    for i in range(len(point_list)):
        point_list_minus_mean.append([point_list[i][0] - mean_x, point_list[i][1] - mean_y])

        #print "point_list[",i,"] = ", point_list[i]
        #print "i-mean:",  point_list[i][0] - mean_x,  point_list[i][1] - mean_y
        #print "point_list_minus_mean[",i,"]", point_list_minus_mean[i][0], point_list_minus_mean[i][1]

  #  meanTol = numpy.mean(point_list, axis = 0)
  #  print "meanTol, line 111:", meanTol
  #  mean_x = meanTol[0]
  #  mean_y = meanTol[1]
   # mean_x = 0
   # mean_y = 0

    #print "point_list_minus_mean, after", point_list_minus_mean
    return point_list_minus_mean, [mean_x, mean_y]

def mulSum(num1, num2):
    assert(len(num1) == len(num2))
    anss = 0
    for i in range(len(num1)):
        anss = anss + num1[i] * num2[i]
    return anss

def compute_error_minimizing_rotation(Points1, Points2):
    """
    Compute the rotation matrix that rotates the points in Points2 to minimize
    squared error to the points in Points1
    """
    iters = 1 # dummy
    #TODO: implement me
    assert(len(Points1) == len(Points2))
    H11 = 0
    H12 = 0
    H21 = 0
    H22 = 0
    #print "Points1:", Points1
    #print "Points2:", Points2
    for i in range(len(Points1)):
        H11 = H11 + Points1[i][0] * Points2[i][0]
        H12 = H12 + Points1[i][0] * Points2[i][1]
        H21 = H21 + Points1[i][1] * Points2[i][0]
        H22 = H22 + Points1[i][1] * Points2[i][1]

    H = [[H11, H12], [H21, H22]]

    U, S, V = numpy.linalg.svd(H)
    R = U.dot(V.T)

    return R #

    tri2transformed = transform_points(Points2,R,[0,0])
    rot1 = sum_of_squared_errors(Points1, tri2transformed)

    R = (U.dot(V.T)).T
    tri2transformed = transform_points(Points2,R,[0,0])
    rot2 = sum_of_squared_errors(Points1, tri2transformed)

    # return U.dot(V.T)

    if rot1 < rot2:
        return U.dot(V.T)
    else:
        return (U.dot(V.T)).T

    # R = V * U.T
    # print "R", R
    #return R.T #[[1,0],[0,1]]

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
    S1,S2 = get_correspondences(Points1,Points2, iters)

    # Center the resulting pairs substracting their means
    S1_shift, mean1 = subtract_mean(S1)
    S2_shift, mean2 = subtract_mean(S2)

    #calculate the error-minimizing rotation
    R = compute_error_minimizing_rotation(S1_shift,S2_shift)
    #find the t such that R*p+t = R*(p-mean2)+mean1
    Rmean2 = [R[0][0]*mean2[0]+R[0][1]*mean2[1],
              R[1][0]*mean2[0]+R[1][1]*mean2[1]]
    return R,[mean1[0]-Rmean2[0],mean1[1]-Rmean2[1]]


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
    S1,S2 = get_correspondences(triangle1,tri2transformed, iters)
    """
    if MATPLOTLIB_AVAILABLE:
        #plot the transformed triangles and their correspondences
        plt.scatter([p[0] for p in triangle1],[p[1] for p in triangle1],
                marker="o")
        plt.scatter([p[0] for p in tri2transformed],[p[1] for p in tri2transformed],
                marker="*")
        for p1,p2 in zip(S1,S2):
            plt.plot([p1[0],p2[0]],[p1[1],p2[1]],color='r',linestyle='-')
        plt.show()
    """
    Rdelta,tdelta = icp_step(triangle1,tri2transformed)
    print "iter",iters+1,len(S1),"correspondences, mean squared error",float(sum_of_squared_errors(S1,S2))/len(S1)
    
    #compose the new rotation with the old rotation
    r1 = transform_point([R[0][0],R[1][0]],Rdelta,[0,0])
    r2 = transform_point([R[0][1],R[1][1]],Rdelta,[0,0])
    R[0][0],R[1][0] = r1
    R[0][1],R[1][1] = r2
    t = transform_point(t,Rdelta,tdelta)

if MATPLOTLIB_AVAILABLE:
    #plot the transformed triangles and their correspondences
    plt.scatter([p[0] for p in triangle1],[p[1] for p in triangle1],
            marker="o")
    plt.scatter([p[0] for p in tri2transformed],[p[1] for p in tri2transformed],
            marker="*")
    for p1,p2 in zip(S1,S2):
        plt.plot([p1[0],p2[0]],[p1[1],p2[1]],color='r',linestyle='-')
    plt.show()
