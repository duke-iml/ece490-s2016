from klampt import se3
from klampt import so3
from utils import *
import numpy
import matplotlib.pyplot as plt
from scipy import ndimage

#this is the object that we know is in the scene
#TODO: play around with this object, it can be "highlighters", "kong_duck", "kygen_eggs", and "school_glue"
obj = "highlighters"

# Read the model and the scene point cloud
model_file = "data/models/"+obj+"_model.json"
depth_file = "data/processed_depth/"+obj+".json"

###########################################################

# Algorithm parameters
windowMult = 0.6 #blurrier for higher numbers
numPtsLow = 0.25
numPtsHigh = 0.5
closeness = 0.03
sizeLow = 0.45
sizeHigh = 1.25
numPtsLow2 = .07

outlier_thresh = 0.9
MAXCLOSESTPTS = 200
###########################################################

def euclidean_distance_3(P1, P2):
    """
    Compute the Squared Euclidean distance between 2 3D points
    """
    return (P1[0]-P2[0])**2+(P1[1]-P2[1])**2+(P1[2]-P2[2])**2

def euclidean_distance_2(P1, P2):
    """
    Compute the Squared Euclidean distance between 2 3D points
    """
    return (P1[0]-P2[0])**2+(P1[1]-P2[1])**2

def subtract_mean(point_list):
    """
    Input list: ((x1,y1,z1),(x2,y2,z2),...)
    Returns a pair ([p1-mean,...,pn-mean], mean)
    """

    mean_x = 0
    mean_y = 0
    mean_z = 0

    for point in point_list:
	mean_x = mean_x + point[0]
	mean_y = mean_y + point[1];
	mean_z = mean_z + point[2];

    mean_x = mean_x/len(point_list)
    mean_y = mean_y/len(point_list)
    mean_z = mean_z/len(point_list)

    point_list_minus_mean = []
    for i in range(len(point_list)):
	point_list_minus_mean.append([point_list[i][0]-mean_x,point_list[i][1]-mean_y,point_list[i][2]-mean_z])

    
    return point_list_minus_mean, [mean_x, mean_y, mean_z]


def segmentScene(obj,scene):
   #uniformly sample the obj vector and the scene vector with x% of points
    numPoints = int(0.5*len(obj))
    nth = int(len(obj)/numPoints)
    sampled_obj = obj[::nth]
    numPoints = int(0.70*len(scene))
    nth = int(len(scene)/numPoints)
    sampled_scene = scene[::nth]

    #get a histogram of the x y and z locations in the object
    hist_x_obj = numpy.histogram([z[0] for z in sampled_obj],bins=500)
    hist_y_obj = numpy.histogram([z[1] for z in sampled_obj],bins=500)
    hist_z_obj = numpy.histogram([z[2] for z in sampled_obj],bins=500)   

    obj_x_dim = max(list(hist_x_obj[1]))-min(list(hist_x_obj[1]))
    print 'Object xdim =',obj_x_dim
    obj_y_dim = max(list(hist_y_obj[1]))-min(list(hist_y_obj[1]))
    print 'Object ydim =',obj_y_dim
    obj_z_dim = max(list(hist_z_obj[1]))-min(list(hist_z_obj[1]))
    print 'Object zdim =',obj_z_dim
  

    #get the centroid of the object
    obj_centroid = [sum(list(hist_x_obj[1]))/len(list(hist_x_obj[1])),sum(list(hist_y_obj[1]))/len(list(hist_y_obj[1])),sum(list(hist_z_obj[1]))/len(list(hist_z_obj[1]))]
    
    #get a histogram of the x y and z locations in the scene
    hist_x = numpy.histogram([z[0] for z in sampled_scene],bins=500)
    hist_y = numpy.histogram([z[1] for z in sampled_scene],bins=500)
    hist_z = numpy.histogram([z[2] for z in sampled_scene],bins=500)

    #plot the histograms
    hist_xx = list(hist_x[0])
    hist_xy = list(hist_x[1])
    hist_xy = hist_xy[0:len(hist_xy)-1]
    #plt.scatter([p for p in hist_xy],[p for p in hist_xx],marker="o")
    #plt.show()

    hist_yx = list(hist_y[0])
    hist_yy = list(hist_y[1])
    hist_yy = hist_yy[0:len(hist_yy)-1]
    #plt.scatter([p for p in hist_yy],[p for p in hist_yx],marker="o")
    #plt.show()

    hist_zx = list(hist_z[0])
    hist_zy = list(hist_z[1])
    hist_zy = hist_zy[0:len(hist_zy)-1]
    #plt.scatter([p for p in hist_zy],[p for p in hist_zx],marker="o")
    #plt.show()


    # make a sliding window that is the width/height/depth of the object and find
    # a point in the histogram where the number of points is close to the number of 
    # points in the object
    x_window = (windowMult+2.5*obj_x_dim)*(max([z[0] for z in sampled_obj]) - min([z[0] for z in sampled_obj]))
    y_window = (windowMult+2.5*obj_y_dim)*(max([z[1] for z in sampled_obj]) - min([z[1] for z in sampled_obj]))
    z_window = (windowMult+2.5*obj_z_dim)*(max([z[2] for z in sampled_obj]) - min([z[2] for z in sampled_obj]))

    num_bins_x = int(x_window/(hist_xy[1]-hist_xy[0]))
    num_bins_y = int(y_window/(hist_yy[1]-hist_yy[0]))
    num_bins_z = int(z_window/(hist_zy[1]-hist_zy[0]))

    #slide the window in the x direction 
    scene_x_range = []   
    scene_y_range = []   
    scene_z_range = []   
    for i in range(len(hist_xy)-num_bins_x-1):
	numpts = sum(hist_xx[i:i+num_bins_x])
	if(numpts > numPtsLow*len(sampled_obj) and numpts < numPtsHigh*len(sampled_obj)): 
	    doAdd = True        	#use the set of points in the window for the icp
	    for el in scene_x_range:
		if abs(el[0]-hist_xy[i]) < closeness:
			doAdd = False
	    if(doAdd):
 	        scene_x_range.append([hist_xy[i],hist_xy[i+num_bins_x]])

    print 'Found', len(scene_x_range), 'X ranges to check'

    #slide the window in the y direction    
    for i in range(len(hist_yy)-num_bins_y-1):
	numpts = sum(hist_yx[i:i+num_bins_y])
	
	if(numpts > numPtsLow*len(sampled_obj) and numpts < numPtsHigh*len(sampled_obj)):
	    doAdd = True        	#use the set of points in the window for the icp
	    for el in scene_y_range:
		if abs(el[0]-hist_yy[i]) < closeness:
			doAdd = False
	    if(doAdd):
		#use the set of points in the window for the icp
		scene_y_range.append([hist_yy[i],hist_yy[i+num_bins_y]])

    print 'Found', len(scene_y_range), 'Y ranges to check'


    #slide the window in the z direction    
    for i in range(len(hist_zy)-num_bins_z-1):
	numpts = sum(hist_zx[i:i+num_bins_z])
	#print hist_zy[i],hist_zy[i+num_bins_z]
	if(numpts > numPtsLow*len(sampled_obj) and numpts < numPtsHigh*len(sampled_obj)):
	    doAdd = True        	#use the set of points in the window for the icp
	    for el in scene_z_range:
		if abs(el[0]-hist_zy[i]) < closeness:
			doAdd = False
	    if(doAdd):	#use the set of points in the window for the icp
		scene_z_range.append([hist_zy[i],hist_zy[i+num_bins_z]])

    print 'Found', len(scene_z_range), 'Z ranges to check'


    #see if the points within the ranges have similar dimensions to the object
    #in the other dimensions
    x_obj_forICP = []
    for r in scene_x_range:
	#find the points in the scene that are in this range
	ptList = []
	for p in sampled_scene:
	    if(p[0] > r[0] and p[0] < r[1]):
		ptList.append(p);

	#get the distance of those points from the centroid of the object
	dist_from_obj = [euclidean_distance_2([x[1],x[2]],[obj_centroid[1],obj_centroid[2]]) for x in ptList]
	#get the histogram of the distances
	dist_hist =  numpy.histogram(dist_from_obj,bins=500)

    	#split this histogram into parts using connected components
	label_vol,nd_labels = ndimage.label(dist_hist[0])
	#check all objects
	for l in range(nd_labels):
	    obj = []
	    rangeXptsY = []
    	    rangeXptsZ = []
	    #find the bins with the label l
	    valid_bins = [None]
	    for i in range(len(label_vol)):
		if(label_vol[i] == l):
		    valid_bins.append(dist_hist[1][i])
	    #get the points that are in the dist range
	    for p in ptList:
		d = euclidean_distance_2([p[1],p[2]],[obj_centroid[1],obj_centroid[2]])
		if d > valid_bins[0] and d < valid_bins[-1]:
		    obj.append(p)
		    rangeXptsY.append(p[1])
		    rangeXptsZ.append(p[2])
	    if len(obj) > 0 and len(obj) > numPtsLow2*len(sampled_obj):
		  #  matplot_points(sampled_obj,obj)
	    #check the object 
		ysize = max(rangeXptsY)-min(rangeXptsY)
		zsize = max(rangeXptsZ)-min(rangeXptsZ)
	    	#print 'Object ydim =',ysize, 'compared to' , obj_y_dim
		if(ysize > sizeLow*obj_y_dim and ysize < sizeHigh*obj_y_dim ):
	            #print 'Object zdim =', zsize, 'compared to' , obj_z_dim
		    if(zsize > sizeLow*obj_z_dim and zsize < sizeHigh*obj_z_dim):
			x_obj_forICP.append(obj)
			print 'Object added'


    print 'Objects from x match to check', len(x_obj_forICP)

    y_obj_forICP = []
    for r in scene_y_range:
	#find the points in the scene that are in this range
	ptList = []
	for p in sampled_scene:
	    if(p[1] > r[0] and p[1] < r[1]):
		ptList.append(p);

	#get the distance of those points from the centroid of the object
	dist_from_obj = [euclidean_distance_2([x[0],x[2]],[obj_centroid[0],obj_centroid[2]]) for x in ptList]
	#get the histogram of the distances
	dist_hist =  numpy.histogram(dist_from_obj,bins=500)

    	#split this histogram into parts using connected components
	label_vol,nd_labels = ndimage.label(dist_hist[0])
	#check all objects
	for l in range(nd_labels):
	    obj = []
	    rangeYptsX = []
    	    rangeYptsZ = []
	    #find the bins with the label l
	    valid_bins = [None]
	
	    for i in range(len(label_vol)):
		if(label_vol[i] == l):
		    valid_bins.append(dist_hist[1][i])
	    #get the points that are in the dist range
	    for p in ptList:
		d = euclidean_distance_2([p[0],p[2]],[obj_centroid[0],obj_centroid[2]])
		if d > valid_bins[0] and d < valid_bins[-1]:
		    obj.append(p)
		    rangeYptsX.append(p[0])
		    rangeYptsZ.append(p[2])
	    if len(obj) > 0 and len(obj) > numPtsLow2*len(sampled_obj):
		  #  matplot_points(sampled_obj,obj)
	    #check the object 
		xsize = max(rangeYptsX)-min(rangeYptsX)
		zsize = max(rangeYptsZ)-min(rangeYptsZ)
	    	#print 'Object xdim =',xsize, 'compared to' , obj_x_dim
		if(xsize > sizeLow*obj_x_dim and xsize < sizeHigh*obj_x_dim ):
	            #print 'Object zdim =',zsize, 'compared to' , obj_z_dim
		    if(zsize > sizeLow*obj_z_dim and zsize < sizeHigh*obj_z_dim):
			y_obj_forICP.append(obj)
			print 'Object added'


    print 'Objects from y match to check', len(y_obj_forICP)

    z_obj_forICP = []
    for r in scene_z_range:
	#find the points in the scene that are in this range
	ptList = []
	for p in sampled_scene:
	    if(p[2] > r[0] and p[2] < r[1]):
		ptList.append(p);


	#get the distance of those points from the centroid of the object
	dist_from_obj = [euclidean_distance_2([x[0],x[1]],[obj_centroid[0],obj_centroid[1]]) for x in ptList]
	#get the histogram of the distances
	dist_hist =  numpy.histogram(dist_from_obj,bins=500)

    	#split this histogram into parts using connected components
	label_vol,nd_labels = ndimage.label(dist_hist[0])
	#check all objects
	for l in range(nd_labels):
	    obj = []
	    rangeZptsX = []
    	    rangeZptsY = []
	    #find the bins with the label l
	    valid_bins = [None]
	    for i in range(len(label_vol)):
		if(label_vol[i] == l):
		    valid_bins.append(dist_hist[1][i])
	    #get the points that are in the dist range
	    for p in ptList:
		d = euclidean_distance_2([p[0],p[1]],[obj_centroid[0],obj_centroid[1]])
		if d > valid_bins[0] and d < valid_bins[-1]:
		    obj.append(p)
		    rangeZptsX.append(p[0])
		    rangeZptsY.append(p[1])
	    if len(obj) > 0 and len(obj) > numPtsLow2*len(sampled_obj):
		  #  matplot_points(sampled_obj,obj)
	    #check the object 
		xsize = max(rangeZptsX)-min(rangeZptsX)
		ysize = max(rangeZptsY)-min(rangeZptsY)
	    	#print 'Object xdim =',xsize, 'compared to' , obj_x_dim
		if(xsize > sizeLow*obj_x_dim and xsize < sizeHigh*obj_x_dim):
	            #print 'Object ydim =',ysize, 'compared to' , obj_y_dim
		    if(ysize > sizeLow*obj_y_dim and ysize < sizeHigh*obj_y_dim):
			z_obj_forICP.append(obj)
			print 'Object added'


    
    print 'Objects from z match to check', len(z_obj_forICP)

#    for i in range(len(x_obj_forICP)):
#	matplot_points(sampled_obj, x_obj_forICP[i])

#    for i in range(len(y_obj_forICP)):
#	matplot_points(sampled_obj, y_obj_forICP[i])

#    for i in range(len(z_obj_forICP)):
#	matplot_points(sampled_obj, z_obj_forICP[i])

    return x_obj_forICP + y_obj_forICP + z_obj_forICP

def icp(scene,obj,sample_num):
    #set up an initial guess
    R = [1,0,0,0,1,0,0,0,1]
    t = [0,0,0]
#####################################################################
    #uniformly sample the obj vector and the scene vector with x% of points
#######################################################################
    numPoints = int(0.1*len(obj))
    nth = int(len(obj)/numPoints)
    sampled_obj = obj[::nth]

    if sample_num > 0:
    	numPoints = int(sample_num*len(scene))
   	nth = int(len(scene)/numPoints)
    	sampled_scene = scene[::nth]
    else:
    	numPoints = int(0.5*len(scene))
        nth = int(len(scene)/numPoints)
      	sampled_scene = scene[::nth]
 
    object_centroid = subtract_mean(sampled_obj)
    object_centroid = object_centroid[1]
    sampled_obj_orig = sampled_obj[:]
    #print object_centroid
    for iters in range(10):
	    sampled_obj = [se3.apply((R,t),p) for p in sampled_obj_orig]
	    #matplot_points(sampled_obj, sampled_scene)

	    """Computes a rotation and translation of the object to the scene
	    using the ICP algorithm.  Returns the transform (R,t) in Klamp't se3
	    format.  
	    """

	#####################################################################
	    # Compute the minimum distance between the points
	#####################################################################
	    closest_points = []
	    distVector = []
	    min_pt_cnt = {}

	    #loop through  all of the points in sampled_obj
	    for point in sampled_scene:
		#find the point in sampled_scene that has the lowest euclidean distance
		minPoint = [-1,-1,-1]
		minDist = 100000;
	
		for point2 in sampled_obj:
		    #check to see if this point has been matched too many times
		    if(euclidean_distance_3(point,point2) < minDist):
			tmp = min_pt_cnt.get( (point2[0],point2[1],point2[2]) )
			if(tmp is None or tmp < MAXCLOSESTPTS):
		            minDist = euclidean_distance_3(point,point2)
			    minPoint = point2;
			#else:
			    #print 'too many'
		tu_pt = (minPoint[0],minPoint[1],minPoint[2])
		#add the min point to the dist vector
		if (min_pt_cnt.get( tu_pt ) is not None):
			min_pt_cnt[ tu_pt ] = min_pt_cnt[tu_pt] + 1
		else:
			min_pt_cnt[ tu_pt ] = 1

		#add the minPoint to the list
		closest_points.append(minPoint)
		distVector.append(minDist)

	######################################################################
	    # Reject the outliers using a threshold
	#####################################################################
	    #sort the distance vector
	    distVector.sort()

	    #set the threshold to the be the lowest 90%
	    percentile = int((outlier_thresh)*len(distVector))
	    threshold = distVector[percentile]
	    thresholded_P1 = [];
	    thresholded_P2 = [];
	    #loop through  all of the points in sampled_obj and closest points
	    for i in range(len(sampled_scene)):
		#if the distance is less than the threshold keep it
		if(euclidean_distance_3(sampled_scene[i],closest_points[i]) < threshold):
		   #add the points to the list
		   thresholded_P1.append(sampled_scene[i]);
		   thresholded_P2.append(closest_points[i]);
	######################################################################           
	    # Compute the R and t matrices. 
	###################################################################### 
	    # Center the resulting pairs substracting their means
	    S1_shift, mean1 = subtract_mean(thresholded_P1)
	    S2_shift, mean2 = subtract_mean(thresholded_P2)


	    #compute the H matrix
	    h_00 = 0
	    h_01 = 0
	    h_02 = 0
	    h_10 = 0
	    h_11 = 0
	    h_12 = 0
	    h_20 = 0
	    h_21 = 0
	    h_22 = 0
	   
	    for i in range(len(S1_shift)):
		h_00 = h_00 + S1_shift[i][0]*S2_shift[i][0]
		h_01 = h_01 + S1_shift[i][0]*S2_shift[i][1]
		h_02 = h_02 + S1_shift[i][0]*S2_shift[i][2]

		h_10 = h_10 + S1_shift[i][1]*S2_shift[i][0]
		h_11 = h_11 + S1_shift[i][1]*S2_shift[i][1]
		h_12 = h_12 + S1_shift[i][1]*S2_shift[i][2]

		h_20 = h_20 + S1_shift[i][2]*S2_shift[i][0]
		h_21 = h_21 + S1_shift[i][2]*S2_shift[i][1]
		h_22 = h_22 + S1_shift[i][2]*S2_shift[i][2]

	    #compute the SVD
	    U,s,V = numpy.linalg.svd([[h_00,h_01,h_02],[h_10,h_11,h_12],[h_20,h_21,h_22]])

	    #compute R = UV^T
	    #r = numpy.matrix(U)*numpy.matrix.getT(numpy.matrix(V))
	    r = numpy.matrix(U)*numpy.matrix(V)
	    R_UV = r.tolist()

	    Rmean2 = [R_UV[0][0]*mean2[0]+R_UV[0][1]*mean2[1]+R_UV[0][2]*mean2[2],
		      R_UV[1][0]*mean2[0]+R_UV[1][1]*mean2[1]+R_UV[1][2]*mean2[2],
		      R_UV[2][0]*mean2[0]+R_UV[2][1]*mean2[1]+R_UV[2][2]*mean2[2] ]

	    tdelta = [mean1[0]-Rmean2[0],mean1[1]-Rmean2[1],mean1[2]-Rmean2[2]]
	    Rdelta = [R_UV[0][0],R_UV[1][0],R_UV[2][0],R_UV[0][1],R_UV[1][1],R_UV[2][1],R_UV[0][2],R_UV[1][2],R_UV[2][2]]
	    #Rdelta = [R_UV[0][0],R_UV[0][1],R_UV[0][2],R_UV[1][0],R_UV[1][1],R_UV[1][2],R_UV[2][0],R_UV[2][1],R_UV[2][2]]
 	    
	    #compose the new rotation with the old rotation
	    r1 = se3.apply((Rdelta,[0,0,0]),[R_UV[0][0],R_UV[1][0],R_UV[2][0]])
	    r2 = se3.apply((Rdelta,[0,0,0]),[R_UV[0][1],R_UV[1][1],R_UV[2][1]])
	    r3 = se3.apply((Rdelta,[0,0,0]),[R_UV[0][2],R_UV[1][2],R_UV[2][2]])

	    R[0],R[1],R[2] = r1
	    R[3],R[4],R[5] = r2
	    R[6],R[7],R[8] = r3
	    

	    #rotate mean of object
	    t = list(se3.apply_rotation((R,[0,0,0]),object_centroid))
	    #print t
	    #subtract t from scene centroind
	    t = [mean1[0]-t[0],mean1[1]-t[1],mean1[2]-t[2]]
	    
	    #t = se3.apply_rotation((R,tdelta),t)
	    #t = se3.apply((Rdelta,tdelta),t)
	    #print t

	    #t = se3.apply((Rdelta,tdelta),t)
	    print 'Iteration', iters,'done'
    return (R,t)    


def icp_plus_segment(scene_full,obj):
    print 
    print '.................Segmenting data.................'
    sampled_scene = segmentScene(obj,scene_full)
    #run the ICP
    print '.................Done segmenting.................'
    print '.................Running ICP.....................'
    if len(sampled_scene) > 0:
	R,t = icp(sampled_scene[0],obj,0)
	return [R,t,sampled_scene[0]]
    else:
	R,t = icp(scene_full,obj,0.025)
	return [R,t,scene_full]

    

def main():
    """
    Main loop.  Run ICP on the given model and scene file, display the results.
    """
    model_full = get_reconstructed_model(model_file,True)
    scene_full = get_raw_depth(depth_file)
    
    #segment the scene
    print
    print '.................Segmenting data.................'
    sampled_scene = segmentScene(model_full['positions'],scene_full)
    #run the ICP
    print '.................Done segmenting.................'
    print '.................Running ICP.....................'
    if len(sampled_scene) > 0:
	R,t = icp(sampled_scene[0],model_full['positions'],0)
    else:
	R,t = icp(scene_full,model_full['positions'],0.05)
    print '.................ICP finished....................'
    #apply the ICP results to the model
    transformed_points = [se3.apply((R,t),p) for p in model_full['positions']]
    model_full['positions'] = transformed_points
	
    #visualize the results
    opengl_plot = OpenGLPlot(model_full, scene_full)
    opengl_plot.initialize_main_loop()


if __name__ == "__main__":
    main()
