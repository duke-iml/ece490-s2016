
import sys
# sys.path.insert(0, "..")
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial import KDTree, cKDTree
import perception
import color
import copy
import json
import scipy.io as sio
import subprocess
# from util.constants import *
REPO_ROOT = "/home/group3/ece490-s2016"
CLOUD_MAT_PATH = REPO_ROOT + "/group3/perception/matpcl/cloud.mat"
CHENYU_GO_PATH = REPO_ROOT + "/group3/perception/matpcl/chenyugo.txt"
CHENYU_DONE_PATH = REPO_ROOT + "/group3/perception/matpcl/chenyudone.txt"


ROS_DEPTH_TOPIC = "/camera/depth/points"
ROS_DEPTH_TOPIC = "/realsense/pc"
# ROS_DEPTH_TOPIC = "/cloud_pcd"

if len(sys.argv) >= 2:
    SAVE_LOCATION = sys.argv[1]
else:
    print "ERROR: You must supply a path (no need for .npz extension) to save to"
    print "Usage: python save_calibrated_shelf.py myfile"
    sys.exit(0)

rospy.init_node("listener", anonymous=True)
STEP = 15
cloud = rospy.wait_for_message(ROS_DEPTH_TOPIC, PointCloud2)
if perception.isCloudValid(cloud):
    cloud = perception.convertPc2ToNp(cloud)
    print cloud[:,3]

    np_cloud = cloud[::STEP]
    # json_str = json.dumps(np_cloud)
    # data = json.loads(json_str)
    # with open('data.json','w') as f:
    #     json.dumps(data,f)
    # np_cloud, pointmean = perception.calPointCloud(np_cloud)
    np_cloud = perception.subtractShelf(np_cloud,'A')
    np_cloud =  np_cloud[np_cloud[:,0] != 0 , :]
    np.savez('sub', np_cloud[:,0],np_cloud[:,1],np_cloud[:,2])
    np_cloud = np_cloud[np_cloud[:,0] != 0 , :]

    # sio.savemat(CLOUD_MAT_PATH, {'cloud':np_cloud})
    # fo = open(CHENYU_GO_PATH, "w")
    # fo.write("chenyu go")
    # fo.close()


    # # Wait for a file to be made "chenyudone.txt"
    # while not os.path.isfile(CHENYU_DONE_PATH):
    #     sleep(1);
    # subprocess.call(["rm",CHENYU_GO_PATH])
    # subprocess.call(["rm",CHENYU_DONE_PATH])
    # # Delete chenyustart.txt first
    # # Delete chenyudone.txt
    # # Try to load seg1.mat and seg20.mat. Some files may not exist
    # object_blobs = []
    # for i in range(1,20)
    #     name  = MAT_PATH+"seg"+str(i)+".m"
    #     if os.path.isfile(name):
    #         mat_contents = sio.loadmat(name)
    #         object_blobs.append(mat_contents['r'])
    # print "============="
    # print "object blobs"
    # print object_blobs
    # print "============="
    # # after matlab segmentation, a list contains segmented cloud will be returned
    # histogram_dict = perception.loadHistogram(object_list) #object list is 
    # cloud_label = {} # key is the label of object, value is cloud points
    # label_score = {} # key is the label, value is the current score for the object 
    # for object_cloud in object_blobs:
    #     object_cloud = perception.resample(cloud,object_cloud,3)
    #     label,score = perception.objectMatch(cloud,histogram_dict)
    #     if label in cloud_label:
    #         if label_score[label] < score:
    #             label_score[label] = score
    #             cloud_label[label] = object_cloud
    #     else:
    #         cloud_label[label] = object_cloud
    #         label_score[label] = score
    # # for each cloud in the list, resample, create a dict that key is the label of object, value is cloud points
    # # create another list that that the key is the label, value is the current score for the object 
    # # if have duplicate label, save the one that have higher score.
    # if target in cloud_label:
    #     com = perception.com(cloud_label[target])
    # else:
    #     cloud_score = {}
    #     histogram_dict = perception.loadHistogram([target])
    #     for object_cloud in object_blobs:
    #         object_cloud = perception.resample(cloud,object_cloud,3)
    #         label,score = perception.objectMatch(cloud,histogram_dict)
    #         cloud_score[score] = object_cloud
    #     sorted_cloud = sorted(scores.items(), key=operator.itemgetter(0))
    #     score  = sorted_cloud.keys()[0]
    #     com = perception.com(sorted_cloud[score])


    # # get the value of target index, calculate center of mass to grab.
    
    # # if target index does not exists as the key, load the histogram of all compare all the histogram with the 









    # cloud = cloud[cloud[:,5] != 255,:]
    np_cloud = np_cloud[np_cloud[:,6]!=255,:]
    final_cloud = perception.resample(cloud,np_cloud,7)
    final_cloud = final_cloud[final_cloud[:,6] != 255 , :]

    # final_cloud = np_cloud

    # print len(cloud)
    # print cloud
    # color_idx = np_cloud[:,3]
    # for i in range(1,7):
    #     color_idx = np.append(color_idx, np_cloud[:,3]+i, axis = 0)
    #     color_idx = np.append(color_idx, np_cloud[:,3]-i, axis = 0)   
    # print color_idx 
    # color_idx =  color_idx[color_idx[:] < 307200]
    # print len(color_idx)
    # # mask = np.zeros((307200,1))
    # # for i in range(0,len(color_idx)):
    # #     if color_idx[i] > 0:
    # #         mask[color_idx[i]] = 1
    # final_cloud = cloud[color_idx.astype(int)]
    # print final_cloud
    # print len(final_cloud)


    np.savez('test', final_cloud[:,0],final_cloud[:,1],final_cloud[:,2])







    np.savez('object', final_cloud[:,0],final_cloud[:,1],final_cloud[:,2])
    y,u,v = color.rgb_to_yuv(final_cloud[:,4],final_cloud[:,5],final_cloud[:,6])
    # print 'y', y
    # print 'u', u
    # print 'v', v
    uv = [color.rgb_to_yuv(*rgb)[1:3] for rgb in final_cloud[:,4:7]]
    # print uv
    # # uv = [u,v]
    # print uv
    hist = color.make_uv_hist(uv)
    # print hist
    # a = hist.sum()
    # print a
    old_hist = np.load('ref2.npz')
    old_hist = old_hist['arr_0']
    # # print old_hist.sum()
    np.savez(SAVE_LOCATION, hist)
    # # # print old_hist
    score = 2*np.minimum(hist,old_hist).sum()-np.maximum(hist,old_hist).sum()
    print score
    # # print len(cloud)
    # # plane1 = perception.segmentation(cloud)
    # # np.savez('subfinished', plane1[:,0],plane1[:,1],plane1[:,2])
    # # plane = perception.segmentation(np_cloud)
    # print'finish sub '
    # # print plane