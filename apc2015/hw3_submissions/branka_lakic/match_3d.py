from klampt import se3
from utils import *
import numpy
import math

#this is the object that we know is in the scene
#TODO: play around with this object, it can be "highlighters", "kong_duck", "kygen_eggs", and "school_glue"
object = "school_glue"

# Read the model and the scene point cloud
model_file = "data/models/"+object+"_model.json"
depth_file = "data/processed_depth/"+object+".json"

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
    Compute the closest 3D points searching exhaustively. The output has
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
	    d = euclidean_distance_3(P1[i], P2[j])

	    if d < cd:
  	       cd = d
	       CP = P2[j]
 	
        out.append(CP)

    return out

def threshold_closest_points(P1, P2, threshold):
    """
    Discard the pairs of points for which their mutual distance exceeds the
    predefined threshold
    Input points: [[x1,y1,z1],[x2,y2,z2],...]
    """
    n = len(P1)
    n2 = len(P2)

    out = []
    out2 = []

    for i in range(n):
        d = euclidean_distance_3(P1[i], P2[i])    
        
        if d < threshold:
           out.append(P1[i])  
           out2.append(P2[i])	         

    return out, out2


def get_correspondences(P1, P2):
    """Returns a pair of lists  (S1,S2) where S1[i] is an element of P1 that
    corresponds to S2[i], which is an element of P2."""
    # Find the closest points to triangle 1 by exhaustive search using the
    # squared Euclidean distance
    P2matches = get_closest_points_3d(P1, P2)

    # The matching pairs may contain irrelevant data. Keep only the matching
    # points that are close enough within a threshold parameter
    
    #threshold = float("inf")
    threshold = 5000

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
    z =	[col[i] for i in range(2,3) for col in point_list]	

    mean_x = sum(x)/n 
    mean_y = sum(y)/n  
    mean_z = sum(z)/n  
   
    x[:] = [tmp - mean_x for tmp in x]
    y[:] = [tmp - mean_y for tmp in y]
    z[:] = [tmp - mean_z for tmp in z]

    point_list_minus_mean = [x, y, z];
    point_list_minus_mean = zip(*point_list_minus_mean);
	
    return point_list_minus_mean, [mean_x, mean_y, mean_z]


def compute_error_minimizing_rotation(Points1, Points2):

    n = len(Points1)

    H = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

    for i in range(n):
    	H = H + numpy.outer(Points1[i],Points2[i])

    U, s, V = numpy.linalg.svd(H)

    R1 = numpy.inner(U,V)
  #  R2 = R1.transpose()

    R1 = numpy.inner(U,V)
   # R2 = R1.transpose()

   # tp2 = transform_points(Points2, R2, [0,0,0])
   # tp1 = transform_points(Points2, R1, [0,0,0])
    
   # p1_c, tp1_c = get_correspondences(Points1, tp1)
   # p2_c, tp2_c = get_correspondences(Points1, tp2)

   # error1 = sum_of_squared_errors(p1_c,tp1_c)
   # error2 = sum_of_squared_errors(p2_c,tp2_c)

   # if error1 > error2:
   # 	R = R2
   # else:
    R = R1

    print "Determinant of the rotation matrix that minimizes rotation ", numpy.linalg.det(R)

    return R	


def transform_point(p,R,t):
    """Compute the point p transformed by the rotation R and translation t"""
    x = R[0][0]*p[0]+R[0][1]*p[1]+R[0][2]*p[2]+t[0]
    y = R[1][0]*p[0]+R[1][1]*p[1]+R[1][2]*p[2]+t[1]
    z = R[2][0]*p[0]+R[2][1]*p[1]+R[2][2]*p[2]+t[2]

    return [x,y,z]

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
    Rmean2 = [R[0][0]*mean2[0]+R[0][1]*mean2[1]+R[0][2]*mean2[2],
              R[1][0]*mean2[0]+R[1][1]*mean2[1]+R[1][2]*mean2[2],
	      R[2][0]*mean2[0]+R[2][1]*mean2[1]+R[2][2]*mean2[2]]

    print "Determinant of the rotation matrix in the icp step: ", numpy.linalg.det(R) 

    return R,[mean1[0]-Rmean2[0],mean1[1]-Rmean2[1],mean2[2]-Rmean2[2]]

def sample_map_random(points, pct):

    n = pct*len(points)
    
    samples = numpy.random.random_integers(0,len(points)-1,n)

    points = numpy.array(points)
    points = points[samples]
 
    return points 

def sample_map_uniform(points, delta):
    """Samples points so that the minimum distance between each two points 
    in each direction is at least delta"""	
    """Not finished."""
    
    multiplier = 5000
    
    x = [int(col[i]*multiplier) for i in range(1) for col in points]
    y = [int(col[i]*multiplier) for i in range(1,2) for col in points]
    z = [int(col[i]*multiplier) for i in range(2,3) for col in points]

    max_x = max(x)+abs(min(x))
    max_y = max(y)+abs(min(y))
    max_z = max(z)+abs(min(z))

    x = [col[i]+min(x) for i in range(len(x))]
    y = [col[i]+min(y) for i in range(len(y))]
    z = [col[i]+min(z) for i in range(len(z))]

    print max_x, max_y, max_z, min(x), min(y), min(z)

    subsample_x = numpy.arange(0,delta,max_x+1)
    subsample_y = numpy.arange(0,delta,max_y+1)
    subsample_z = numpy.arange(0,delta,max_z+1)

    space = numpy.zeros((max_x+1,max_y+1,max_z+1))
    space[numpy.array(x), numpy.array(y), numpy.array(z)] = 1
    space[subsample_x, subsample_y, subsample_z] = 0

    points_out = list(numpy.nonzero(space))

    x = [(col(i)-min_x)/multiplier for i in range(1) for col in points_out]
    y = [(col[i]-min_y)/multiplier for i in range(1,2) for col in points_out]
    z = [(col[i]-min_z)/multiplier for i in range(2,3) for col in points_out]

    x = numpy.array(x)
    y = numpy.array(y)
    z = numpy.array(z)

    points_out = numpy.concatenate((x,y,z), axis=1)
  
    return points_out

def icp(object,scene):
    """Computes a rotation and translation of the object to the scene
    using the ICP algorithm.  Returns the transform (R,t) in Klamp't se3
    format.  
    """ 

    object1 = numpy.array(object)
    object1 = object1[:,0:3]
    scene1 = numpy.array(scene)
    scene1 = scene1[:,0:3]
    
    object1 = object1.tolist()
    scene1 = scene1.tolist()

    # Sample both maps to make the closest point computations faster
    
    # Percentage of points to be implemented
    pct = 0.05
    object_sampled = sample_map_random(object1, pct)
    scene_sampled = sample_map_random(scene1, pct)
    
    Points1 = object_sampled
    Points2 = scene_sampled

    #set up an initial guess
    theta = 1.0
   
    R = [[1,0,0],[0,math.cos(theta),-math.sin(theta)],[0,math.sin(theta),math.cos(theta)]]
    
    t = [0,0,0]

    #Run ICP for 20 iterations
    for iters in range(10):
        print "Iteration number ", iters

    	tPoints2 = transform_points(Points2,R,t)
    	S1,S2 = get_correspondences(Points1,tPoints2)
          
    	Rdelta,tdelta = icp_step(Points1,tPoints2)
  
    	#compose the new rotation with the old rotation
    	R = numpy.matrix(Rdelta)*numpy.matrix(R)
        R = R.tolist()
  
    	t = transform_point(t,Rdelta,tdelta)
       
    return R,t


def main():
    """
    Main loop.  Run ICP on the given model and scene file, display the results.
    """
    model_full = get_reconstructed_model(model_file,True)
 
    scene_full = get_raw_depth(depth_file)
 
    #run the ICP
    R,t = icp(model_full['positions'],scene_full)

    R = zip(*R)
    R = [num for elem in R for num in elem]

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
