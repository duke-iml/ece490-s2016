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
    
    closest_points = [];
    #loop through  all of the points in P1
    for point in P1:
        #find the point in P2 that has the lowest euclidean distance
        minPoint = [-1,-1]
        minDist = 100000;
        for point2 in P2:
            if(euclidean_distance_2(point,point2) < minDist):
                minDist = euclidean_distance_2(point,point2)
                minPoint = point2;

        #add the minPoint to the list
        closest_points.append(minPoint);

    return closest_points;

def threshold_closest_points(P1, P2, threshold):
    """
    Discard the pairs of points for which their mutual distance exceeds the
    predefined threshold
    Input points: [[x1,y1],[x2,y2],...]
    """

    thresholded_P1 = [];
    thresholded_P2 = [];
    #loop through  all of the points in P1 and P2
    for i in range(0,len(P1)):
        #if the distance is less than the threshold keep it
        if(euclidean_distance_2(P1[i],P2[i]) < threshold):
           #add the points to the list
           thresholded_P1.append(P1[i]);
           thresholded_P2.append(P2[i]);
           
    return thresholded_P1,thresholded_P2

def get_correspondences(P1, P2):
    """Returns a pair of lists  (S1,S2) where S1[i] is an element of P1 that
    corresponds to S2[i], which is an element of P2."""
    # Find the closest points to triangle 1 by exhaustive search using the
    # squared Euclidean distance
    P2matches = get_closest_points_2d(P1, P2)
    # The matching pairs may contain irrelevant data. Keep only the matching
    # points that are close enough within a threshold parameter
    
    # TODO: determine a good threshold.  Set the 'threshold' variable
    # to be your best choice, and write your justification to this answer
    # in the below:
    #
    # [Task 4 answer goes here]
    # I set the threshold based on a percentile cutoff of the sorted distance
    # vector between P1 and the matching P2 points. This way each time the get_correspondences
    # method is called the threshold chnages based on the the new matches.
    # I empirically found that a high cuttoff worked better than a low cutoff because
    # even though the error starts out high, the triangle moves more when more points are selected
    # and so the error decreases rapidly after the first few itterations. If the cutoff is set too low
    # the algorithm gets stuck in a local minima
    distVector = []
    for i in range(0,len(P1)):
        #creates a vector of the distances between points
	t = euclidean_distance_2(P1[i],P2matches[i])
        distVector.append(t)

    #sort the distance vector
    distVector.sort()

    #set the threshold to the be the lowest 90%
    percentile = int(0.90*len(distVector))
    threshold = distVector[percentile]
    #print 'Threshold',threshold


    S1, S2 = threshold_closest_points(P1,P2matches,threshold)
    return S1,S2

def subtract_mean(point_list):
    """
    Input list: ((x1,y1),(x2,y2),...)
    Returns a pair ([p1-mean,...,pn-mean], mean)
    """
    
    mean_x = 0
    mean_y = 0

    for point in point_list:
	mean_x = mean_x + point[0]
	mean_y = mean_y + point[1];


    mean_x = mean_x/len(point_list)
    mean_y = mean_y/len(point_list)
    point_list_minus_mean = []
    for i in range(0,len(point_list)):
	point_list_minus_mean.append([point_list[i][0]-mean_x,point_list[i][1]-mean_y])

    
    return point_list_minus_mean, [mean_x, mean_y]


def compute_error_minimizing_rotation(Points1, Points2, matrix):
    """
    Compute the rotation matrix that rotates the points in Points2 to minimize
    squared error to the points in Points1
    """

    #compute the H matrix
    h_00 = 0
    h_01 = 0
    h_10 = 0
    h_11 = 0
    for i in range(0,len(Points1)):
	h_00 = h_00 + Points1[i][0]*Points2[i][0]
	h_01 = h_01 + Points1[i][0]*Points2[i][1]
	h_10 = h_10 + Points1[i][1]*Points2[i][0]
	h_11 = h_11 + Points1[i][1]*Points2[i][1]

    #compute the SVD
    U,s,V = numpy.linalg.svd([[h_00,h_01],[h_10,h_11]])

    #compute the transpose
    #Vt = [ [V[0][0],V[1][0]], [ V[0][1],V[1][1] ] ]
    #Ut = [ [U[0][0],U[1][0]], [ U[0][1],U[1][1] ] ]

    #compute R = UV^T
#    R_00 =  U[0][0]*Vt[0][0] + U[0][1]*Vt[1][0]
#    R_01 =  U[0][0]*Vt[0][1] + U[0][1]*Vt[1][1]
#    R_10 =  U[1][0]*Vt[0][0] + U[1][1]*Vt[0][1]
#    R_11 =  U[1][0]*Vt[0][1] + U[1][1]*Vt[1][1]

#    R_UV = [ [R_00,R_01], [R_10,R_11] ]

    #compute R = VU^T
#    R_00 =  V[0][0]*Ut[0][0] + V[0][1]*Ut[1][0]
#    R_01 =  V[0][0]*Ut[0][1] + V[0][1]*Ut[1][1]
#    R_10 =  V[1][0]*Ut[0][0] + V[1][1]*Ut[0][1]
#    R_11 =  V[1][0]*Ut[0][1] + V[1][1]*Ut[1][1]

#    R_VU = [ [R_00,R_01], [R_10,R_11] ]

    R_VU = numpy.matrix(V)*numpy.matrix.getT(numpy.matrix(U))
    R_VU = R_VU.tolist()
    
    R_UV = numpy.matrix(U)*numpy.matrix.getT(numpy.matrix(V))
    R_UV = R_UV.tolist()
    
    if(matrix == 1):
	return R_UV
    else:
	return R_VU


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

    #copy S2 twice
    S2_1 = S2[:]
    S2_2 = S2[:]
    #calculate the error-minimizing rotation
    
    #try the R_UV first
    R1 = compute_error_minimizing_rotation(S1_shift,S2_shift,0)
    #find the t such that R*p+t = R*(p-mean2)+mean1
    R1mean2 = [R1[0][0]*mean2[0]+R1[0][1]*mean2[1],
              R1[1][0]*mean2[0]+R1[1][1]*mean2[1]]
    t1 = [mean1[0]-R1mean2[0],mean1[1]-R1mean2[1]]

    #perform the transform
    S2_1 = transform_points(S2_1,R1,t1)

    #get the error
    err1 = float(sum_of_squared_errors(S1,S2_1))/len(S1)

    #try the R_VU
    R2 = compute_error_minimizing_rotation(S1_shift,S2_shift,1)
    #find the t such that R*p+t = R*(p-mean2)+mean1
    R2mean2 = [R2[0][0]*mean2[0]+R2[0][1]*mean2[1],
              R2[1][0]*mean2[0]+R2[1][1]*mean2[1]]
    t2 = [mean1[0]-R2mean2[0],mean1[1]-R2mean2[1]]
    #perform the transform
    S2_2 = transform_points(S2_2,R2,t2)

    #get the error
    err2 = float(sum_of_squared_errors(S1,S2_2))/len(S1)
    #print err1,'error UV',err2,'error VU'
    if(err1 < err2):
	return R1,t1
    else:
	return R2,t2
#    return R,[mean1[0]-Rmean2[0],mean1[1]-Rmean2[1]]


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
        plt.scatter([p[0] for p in triangle1],[p[1] for p in triangle1],
                marker="o")
        plt.scatter([p[0] for p in tri2transformed],[p[1] for p in tri2transformed],
                marker="*")
        for p1,p2 in zip(S1,S2):
            plt.plot([p1[0],p2[0]],[p1[1],p2[1]],color='r',linestyle='-')
        #plt.show()
        
    Rdelta,tdelta = icp_step(triangle1,tri2transformed)
    print "iter",iters,len(S1),"correspondences, mean squared error",float(sum_of_squared_errors(S1,S2))/len(S1)
    
    #compose the new rotation with the old rotation
    r1 = transform_point([R[0][0],R[1][0]],Rdelta,[0,0])
    r2 = transform_point([R[0][1],R[1][1]],Rdelta,[0,0])
    R[0][0],R[1][0] = r1
    R[0][1],R[1][1] = r2
    t = transform_point(t,Rdelta,tdelta)
    #print t, tdelta

