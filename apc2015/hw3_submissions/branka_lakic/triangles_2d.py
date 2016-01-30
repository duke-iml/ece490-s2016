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
    n1 = len(P1)
    n2 = len(P2)

    out = []

    for i in range(n1):
	CP = []
	cd = float("inf")

	for j in range(n2):
	    d = euclidean_distance_2(P1[i], P2[j])

	    if d < cd:
  	       cd = d
	       CP = P2[j]
 	
        out.append(CP)

        #print "Checking distances between matched points: ", P1[i], out[i], cd

    return out



def threshold_closest_points(P1, P2, threshold):
    """
    Discard the pairs of points for which their mutual distance exceeds the
    predefined threshold
    Input points: [[x1,y1],[x2,y2],...]
    """
    n = len(P1)
    n2 = len(P2)

    out = []
    out2 = []

    for i in range(n):
        d = euclidean_distance_2(P1[i], P2[i])    
        
        if d < threshold:
           out.append(P1[i])  
           out2.append(P2[i])	           

    return out, out2



def get_correspondences(P1, P2):
    """Returns a pair of lists  (S1,S2) where S1[i] is an element of P1 that
    corresponds to S2[i], which is an element of P2."""
    # Find the closest points to triangle 1 by exhaustive search using the
    # squared Euclidean distance
    P2matches = get_closest_points_2d(P1, P2)

    # The matching pairs may contain irrelevant data. Keep only the matching
    # points that are close enough within a threshold parameter
    
    threshold = float("inf")
    # TODO: determine a good threshold.  Set the 'threshold' variable
    # to be your best choice, and write your justification to this answer
    # in the below:
    #
    # [Task 4 answer goes here]
    #
    
    S1, S2 = threshold_closest_points(P1,P2matches,threshold)
    return S1,S2

def subtract_mean(point_list):
    """
    Input list: ((x1,y1),(x2,y2),...)
    Returns a pair ([p1-mean,...,pn-mean], mean)
    """
    
    n = len(point_list)

    if n == 0:
	return point_list;

    x = [col[i] for i in range(1) for col in point_list]
    y =	[col[i] for i in range(1,2) for col in point_list]	
    
    mean_x = sum(x)/n 
    mean_y = sum(y)/n  
   
    x[:] = [tmp - mean_x for tmp in x]
    y[:] = [tmp - mean_y for tmp in y]

    point_list_minus_mean = [x, y];
    point_list_minus_mean = zip(*point_list_minus_mean);
	
    return point_list_minus_mean, [mean_x, mean_y]

def compute_error_minimizing_rotation(Points1, Points2):
    """
    Compute the rotation matrix that rotates the points in Points2 to minimize
    squared error to the points in Points1
    """
    x1 = [col[i] for i in range(1) for col in Points1]
    y1 = [col[i] for i in range(1,2) for col in Points1]

    x2 = [col[i] for i in range(1) for col in Points2]
    y2 = [col[i] for i in range(1,2) for col in Points2]

    xx = sum([a*b for a,b in zip(x1,x2)])
    xy = sum([a*b for a,b in zip(x1,y2)])

    yx = sum([a*b for a,b in zip(y1,x2)])
    yy = sum([a*b for a,b in zip(y1,y2)])

    H = [[xx,xy], [yx,yy]]

    U, s, V = numpy.linalg.svd(H)

    R1 = numpy.inner(U,V)
    R2 = R1.transpose()

    tp2 = transform_points(Points2, R2, [0,0])
    tp1 = transform_points(Points2, R1, [0,0])
    
    p1_c, tp1_c = get_correspondences(Points1, tp1)
    p2_c, tp2_c = get_correspondences(Points1, tp2)

    error1 = sum_of_squared_errors(p1_c,tp1_c)
    error2 = sum_of_squared_errors(p2_c,tp2_c)

    if error1 > error2:
	R = R2
    else:
	R = R1

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
    S1,S2 = get_correspondences(triangle1,tri2transformed)
    if MATPLOTLIB_AVAILABLE:
        #plot the transformed triangles and their correspondences
        plt.scatter([p[0] for p in triangle1],[p[1] for p in triangle1],
                marker="o")
        plt.scatter([p[0] for p in tri2transformed],[p[1] for p in tri2transformed],
                marker="*")
       # for p1,p2 in zip(S1,S2):
       #     plt.plot([p1[0],p2[0]],[p1[1],p2[1]],color='r',linestyle='-')
        plt.show()
        
    Rdelta,tdelta = icp_step(triangle1,tri2transformed)
    print "iter",iters,len(S1),"correspondences, mean squared error",float(sum_of_squared_errors(S1,S2))/len(S1)
    
    #compose the new rotation with the old rotation
    r1 = transform_point([R[0][0],R[1][0]],Rdelta,[0,0])
    r2 = transform_point([R[0][1],R[1][1]],Rdelta,[0,0])
    R[0][0],R[1][0] = r1
    R[0][1],R[1][1] = r2
    t = transform_point(t,Rdelta,tdelta)

