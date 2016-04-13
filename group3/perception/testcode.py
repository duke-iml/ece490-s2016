import numpy as np

pointnum1 = np.array([1,2,3,4,9])
point_list = np.array([1,2,3,4,5,6,7,8])
dmin = 1 
sellist = np.array([1,2])
mask = (pointnum1 !=0)
pointnum1 = pointnum1[mask]
print len(pointnum1)
for i in range(0,len(pointnum1)):
    print "loop" , i
    if len((point_list == pointnum1[i]).ravel().nonzero()[0]):
        curr_number = pointnum1[i]
        pointnum1[i] =0
        if len((sellist == curr_number).ravel().nonzero()[0]):
            point_list = point_list[(point_list != curr_number)]
    if len((sellist == pointnum1[i]).ravel().nonzero()[0]):
        point_list = point_list[(point_list != pointnum1[i])]
        print i
        print ("point list: ", point_list)
        pointnum1[i] = 0
    if pointnum1[0] == dmin:
        pointnum1[0] = 0
pointnum1 = pointnum1[(pointnum1 != 0)]
point_list =np.append(point_list,pointnum1,axis =0)
print ("point list: ", point_list)
print ("point num1", pointnum1)
# print("length of point list after append", number)
point_list = point_list[1:(len(point_list))]
point_list = point_list[(point_list != 0)]