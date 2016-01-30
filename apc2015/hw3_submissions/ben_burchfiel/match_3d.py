import se3
from utils import *

#this is the object that we know is in the scene
#TODO: play around with this object, it can be "highlighters", "kong_duck", "kygen_eggs", and "school_glue"
object = "school_glue"

# Read the model and the scene point cloud
model_file = "data/models/"+object+"_model.json"
depth_file = "data/processed_depth/"+object+".json"

def icp(object,scene):
    """Computes a rotation and translation of the object to the scene
    using the ICP algorithm.  Returns the transform (R,t) in Klamp't se3
    format.  
    """
    #TODO: Sample both maps to make the closest point computations faster
    objectTrans = object
    R = [[1,0,0],[0,1,0],[0,0,1]]
    t = [0,0,0]
    for iters in range(20):
        print "beginning iter",iters
        transform_points(objectTrans,R,t)
        Rdelta,tdelta = icp_step(scene, objectTrans)

        r1 = transform_point([R[0][0],R[1][0],R[2][0]],Rdelta,[0,0,0])
        r2 = transform_point([R[0][1],R[1][1],R[2][1]],Rdelta,[0,0,0])
        r2 = transform_point([R[0][2],R[1][2],R[2][2]],Rdelta,[0,0,0])
        R[0][0],R[1][0] = r1
        R[0][1],R[1][1] = r2
        t = transform_point(t,Rdelta,tdelta)

        # Compute the minimum distance between the points

        # Reject the outliers, for example, using a threshold

        # Compute the R and t matrices. The procedure can be similar as the one for the
        # 2D images.
    return se3.identity()

def euclidean_distance_3(P1, P2):
    """
    Compute the Squared Euclidean distance between 2 3D points
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
    Input points: [[x1,y1,z1],[x2,y2,z2],...]
    """

    closestPoints = []
    listIndex = 0
    for point in P1:
        currClosest = []
        currDist = -1
        for canPoint in P2:
            dist = euclidean_distance_3(point, canPoint)
            if currDist == -1 or dist < currDist:
                currClosest = canPoint
                currDist = euclidean_distance_3(point, canPoint)

        closestPoints.append(currClosest)
        listIndex += 1

    return closestPoints

def threshold_closest_points(P1, P2, threshold):
    """
    Discard the pairs of points for which their mutual distance exceeds the
    predefined threshold
    Input points: [[x1,y1,z1],[x2,y2,z2],...]
    """

    S1 = []
    S2 = []
    place = 0
    for point1 in P1:
        point2 = P2[place]
        if not euclidean_distance_3(point1, point2) > threshold:
            S1.append(point1)
            S2.append(point2)

        place += 1

    return S1, S2

def get_correspondences(P1, P2):
    """Returns a pair of lists  (S1,S2) where S1[i] is an element of P1 that
    corresponds to S2[i], which is an element of P2."""
    # Find the closest points to triangle 1 by exhaustive search using the
    # squared Euclidean distance
    P2matches = get_closest_points_3d(P1, P2)

    # The matching pairs may contain irrelevant data. Keep only the matching
    # points that are close enough within a threshold parameter

    threshold = 1000

    S1, S2 = threshold_closest_points(P1,P2matches,threshold)
    return S1,S2

def subtract_mean(point_list):
    """
    Input list: ((x1,y1,z1),(x2,y2,z1),...)
    Returns a pair ([p1-mean,...,pn-mean], mean)
    """

    mean_x = 0
    mean_y = 0
    mean_z = 0
    rtnList = []

    for point in point_list:
        mean_x += float(1)/len(point_list) * point[0]
        mean_y += float(1)/len(point_list) * point[1]
        mean_z += float(1)/len(point_list) * point[2]

    for point in point_list:
        rtnList.append([point[0] - mean_x, point[1] - mean_y, point[2] - mean_z])

    return rtnList, [mean_x, mean_y, mean_z]


def compute_error_minimizing_rotation(Points1, Points2):
    """
    Compute the rotation matrix that rotates the points in Points2 to minimize
    squared error to the points in Points1
    """

    s = (3,3)
    H = numpy.zeros(s)

    spot = 0
    for point1 in Points1:
        point2 = Points2[spot]

        p1 = numpy.array(point1)
        p2 = numpy.array(point2)
        H = H + numpy.outer(p1, p2)

        spot += 1

    U, s, V = numpy.linalg.svd(H)
    R = numpy.dot(U, V)
    return R

def transform_point(p,R,t):
    """Compute the point p transformed by the rotation R and translation t"""
    x = R[0][0]*p[0]+R[0][1]*p[1]+R[0][2]*p[2]+t[0]
    y = R[1][0]*p[0]+R[1][1]*p[1]+R[1][2]*p[2]+t[1]
    z = R[2][0]*p[0]+R[2][1]*p[1]+R[2][2]*p[2]+t[2]
    return [x,y, z]

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
    return R,[mean1[0]-Rmean2[0],mean1[1]-Rmean2[1],mean1[2]-Rmean2[2]]


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
