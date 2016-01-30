from klampt import se3
from utils import *
import random
import numpy

#this is the object that we know is in the scene
#TODO: play around with this object, it can be "highlighters", "kong_duck", "kygen_eggs", and "school_glue"


# Read the model and the scene point cloud


def icp(object,scene, r_alt_bool):
    """Computes a rotation and translation of the object to the scene
    using the ICP algorithm.  Returns the transform (R,t) in Klamp't se3
    format.  
    """
    #TODO: implement me
    # Sample both maps to make the closest point computations faster
    new_object, new_scene = sample_points(object,scene,.01)
    print len(new_scene)
    scrape_scene(new_scene, .02, 40)
    print len(new_scene), "NOWWWW"


    # Compute the minimum distance between the points
    new_scene_matches = get_closest_points_3d(new_object, new_scene)


    test = [eu_dist_2(new_object[i],new_scene_matches[i]) for i in range(len(new_object))]
    sort = sorted(test)
    threshold = sort[(int)(len(new_object)*.8)]
    # Reject the outliers, for example, using a threshold
    S1, S2 = threshold_closest_points(new_object,new_scene_matches,threshold)

    # Compute the R and t matrices. The procedure can be similar as the one for the
    # 2D images.
        # Center the resulting pairs substracting their means
    S1_shift, mean1 = subtract_mean(S1)
    S2_shift, mean2 = subtract_mean(S2)

    #calculate the error-minimizing rotation
    R = compute_error_minimizing_rotation(S1_shift,S2_shift, r_alt_bool)
    R_so3 = [R[i][j] for j in range(3) for i in range(3)]

    #find the t such that R*p+t = R*(p-mean2)+mean1
    Rmean2 = [R[0][0]*mean2[0]+R[0][1]*mean2[1]+R[0][2]*mean2[2],
              R[1][0]*mean2[0]+R[1][1]*mean2[1]+R[1][2]*mean2[2],
                R[2][0]*mean2[0]+R[2][1]*mean2[1]+R[2][2]*mean2[2]]
    return R_so3,[mean1[0]-Rmean2[0],mean1[1]-Rmean2[1],mean1[2]-Rmean2[2]]

def compute_error_minimizing_rotation(Points1, Points2, r_alt_bool):
    """
    Compute the rotation matrix that rotates the points in Points2 to minimize
    squared error to the points in Points1
    """
    #TODO: implement me
    tl = sum([Points1[i][0] * Points2[i][0] for i in range(len(Points1))])
    tc = sum([Points1[i][0] * Points2[i][1] for i in range(len(Points1))])
    tr = sum([Points1[i][0] * Points2[i][2] for i in range(len(Points1))])
    cl = sum([Points1[i][1] * Points2[i][0] for i in range(len(Points1))])
    cc = sum([Points1[i][1] * Points2[i][1] for i in range(len(Points1))])
    cr = sum([Points1[i][1] * Points2[i][2] for i in range(len(Points1))])
    bl = sum([Points1[i][2] * Points2[i][0] for i in range(len(Points1))])
    bc = sum([Points1[i][2] * Points2[i][1] for i in range(len(Points1))])
    br = sum([Points1[i][2] * Points2[i][2] for i in range(len(Points1))])

    cov = [[tl,tc,tr],[cl,cc,cr],[bl,bc,br]]
    U, s, V = numpy.linalg.svd(cov)
    R = numpy.dot(U,V.transpose())
    R_alt = numpy.dot(V,U.transpose())
    if r_alt_bool:
        return R_alt
    else:
        return R

def sum_of_squared_errors(P1,P2):
    """For the corresponding points P1 and P2, returns the sum of squared
    errors"""
    return sum(eu_dist_2(p1,p2) for (p1,p2) in zip(P1,P2))

def threshold_closest_points(P1, P2, threshold):
    """
    Discard the pairs of points for which their mutual distance exceeds the
    predefined threshold
    Input points: [[x1,y1],[x2,y2],...]
    """
    #TODO: implement me
    P1ret = []
    P1matchesret = []
    for i in range(len(P1)):
        if eu_dist_2(P1[i],P2[i]) <= threshold:
            P1ret.append(P1[i])
            P1matchesret.append(P2[i])
    return P1ret, P1matchesret

def subtract_mean(point_list):
    """
    Input list: ((x1,y1),(x2,y2),...)
    Returns a pair ([p1-mean,...,pn-mean], mean)
    """
    #TODO: implement me
    mean_x = sum([point_list[i][0] for i in range(len(point_list))])/len(point_list)
    mean_y = sum([point_list[i][1] for i in range(len(point_list))])/len(point_list)
    mean_z = sum([point_list[i][2] for i in range(len(point_list))])/len(point_list)
    point_list_minus_mean = [[point_list[i][0]-mean_x,point_list[i][1]-mean_y,point_list[i][2]-mean_z] for i in range(len(point_list))]
    return point_list_minus_mean, [mean_x, mean_y, mean_z]

def sample_points(object,scene,percentage):
    object_indexes = [(int)(random.random()*len(object)) for i in range((int)(len(object)*percentage))]
    scene_indexes = [(int)(random.random()*len(scene)) for i in range((int)(len(object)*percentage))]

    new_object = [object[i] for i in object_indexes]
    new_scene = [scene[i] for i in scene_indexes]

    return new_object, new_scene

def get_closest_points_3d(P1, P2):
    #TODO: kaighn
    """
    Compute the closest 3D points searching exhaustively. The output has
    the same dimension as P1 and produces the closest point in P2 to P1.
    Input points: [[x1,y1,z1],[x2,y2,z2],...]
    """
    ret = []
    P2prime = list(P2)
    p2uses = [0 for i in range(len(P2prime))]
    for i in range(len(P1)):
        bestDist = 922337203685477580
        bestPoint = []
        for j in range(len(P2prime)):
            if eu_dist_2(P1[i],P2prime[j]) < bestDist:
                bestDist = eu_dist_2(P1[i],P2prime[j])
                bestPoint = P2prime[j]
                bestIndex = j
        p2uses[bestIndex] += 1
        if p2uses[bestIndex] == 1:
            P2prime.remove(bestPoint)
        ret.append(bestPoint)
    print "len P1: ", len(P1)
    print "len P2: ", len(P2)
    print "len P2matches: ", len(ret)
    return ret 
def scrape_scene(scene,threshold,numNeighbors):
    """
    Attempt to remove points that are not close to an amount of other points.
    This is to remove the random single blotches of points near the model.
    """

    for p in scene:
        neighborCount = 0
        for q in scene:
            if(eu_dist_2(p,q) < threshold):
                neighborCount += 1
                if(neighborCount == numNeighbors):
                    break
        if(neighborCount != numNeighbors):
            scene.remove(p)
            


def eu_dist_2(pointA, pointB):
    """
    Returns Euclidean distance between two points of the same dimension.
    """
    if(len(pointA) != len(pointB)):
        print "Points are not of same dimension."
        return None
    return sum([(pointA[i]-pointB[i])**2 for i in range(len(pointA))])

def main2(object, scene):
    """
    Main loop.  Run ICP on the given model and scene file, display the results.
    """
    
    model_f = "data/models/"+object+"_model.json"
    depth_f = "data/processed_depth/"+scene+".json"
    model_full = get_reconstructed_model(model_f,True)
    scene_full = get_raw_depth(depth_f)


    oldError = 9999999999999999
    currentError = 9999999999999999
    r_alt_bool = False
    for iters in range(5):
        bool = True
        while bool:
            #run the ICP
            R,t = icp(model_full['positions'],scene_full, r_alt_bool)

            #apply the ICP results to the model
            transformed_points = [se3.apply((R,t),p) for p in model_full['positions']]

            #visualize the results
           
            # If matplot is available you can use it to visualize the points (but it not
            # required) as in the commented line
            # matplot_points(model_full, clpoints_out)

            #errors
            print "iter",iters,len(transformed_points),"correspondences, mean squared error",float(sum_of_squared_errors(transformed_points,scene_full))/len(transformed_points)
            currentError = float(sum_of_squared_errors(transformed_points,scene_full))/len(transformed_points)

            if currentError < oldError:
                model_full['positions'] = transformed_points
                bool = False
                oldError = currentError
            else:
                r_alt_bool = ~r_alt_bool
            
            if(iters == 4):
                opengl_plot = OpenGLPlot(model_full, scene_full)
                opengl_plot.initialize_main_loop()
            


if __name__ == "__main__":
    main2("school_glue","school_glue")
