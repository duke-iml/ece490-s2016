import sys
sys.path.insert(0, "..")

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2
from util.constants import *
from scipy.spatial import KDTree, cKDTree
import copy
import ctypes
import struct
import os
import color
import operator

def isCloudValid(cloud):
    """
    Returns whether a cloud is valid to be processed by this perception module
    """
    for point in pc2.read_points(cloud,skip_nans=False,field_names=("rgb","x","y","z")):
        return True
        break
    return False

def calPointCloud(cloud):
    """
    Not currently used

    Input: Point cloud
    Output: (Calibrated cloud, COM that was subtracted)
    Calculates the COM of the two shelf edges and
    subtracts their COM from every point in the cloud
    """
    pointsortt = cloud[cloud[:,2].argsort(),]
    pointth = min(np.mean(pointsortt[0:500,2])*2,max(pointsortt[0:500,2]))
    pointsort = np.array([pointsortt[0,:]])
    begin = 0
    while pointsortt[begin,2] <= pointth:
        pointsort = np.append(pointsort,[pointsortt[begin,0:3]],axis =0)
        begin = begin+1
    pointmean = np.mean(pointsort,axis = 0)
    print ("COM of shelf edges: ", pointmean)
    cloud[:,0:3] = cloud[:,0:3] - pointmean
    return (cloud, pointmean)

def convertPc2ToNp(data):
    """
    Input: PointCloud2 message
    Output: NumPy array of point cloud [x,y,z,index]
    """
    STEP = 1 # Plot every STEPth point for speed, set to 1 to plot all

    # Load cloud data
    xs = []
    ys = []
    zs = []
    idx = []
    r = []
    g = []
    b = []
    idxnum = 0
    for point in pc2.read_points(data,skip_nans=False,field_names=("rgb","x","y","z")):
        # print point
        xs.append(point[0])
        ys.append(point[1])
        zs.append(point[2])
        test = point[3]
        # print point[3]
        # cast float32 to int so that bitwise operations are possible
        s = struct.pack('>f' ,test)
        i = struct.unpack('>l',s)[0]
        # you can get back the float value by the inverse operations
        pack = ctypes.c_uint32(i).value
        # a = (pack & 0xFF000000)>> 24
        r1 = (pack & 0x00FF0000)>> 16
        g1 = (pack & 0x0000FF00)>> 8
        b1 = (pack & 0x000000FF)
        r.append(r1)
        g.append(g1)
        b.append(b1)
        idx.append(idxnum)
        idxnum = idxnum+1
    print "Point cloud count: " + str(len(xs))
    # print r,g,b,a
    cloud = np.array([xs,ys,zs,idx,r,g,b])
    cloud = cloud.transpose()
    nans = np.isnan(cloud)
    cloud[nans] = 0
    print "Original cloud in convertPc2ToNp"
    print cloud
    cloud = cloud[::STEP]
    return cloud

def subtractShelf(cloud):
    """
    Input: Numpy array of points with index as the forth number
    Output: Numpy array of points without shelf points
    """
    # Load shelf data
    data_shelf = np.load(SHELF_NPZ_FILE)
    dshel = np.mat([data_shelf['arr_0'],data_shelf['arr_1'],data_shelf['arr_2']])
    dshel = dshel.transpose()

    # Construct kd-tree and remove nearest neighbors
    pCloud = copy.copy(cloud[:,0:3])
    t = cKDTree(pCloud)
    d, idx = t.query(dshel, k=20, eps=0, p=2, distance_upper_bound=10)
    Points_To_Delete = []
    for i in range(0,len(idx)):
       for j in range(0,19):
            if d[i,j] <= 0.05:
               Points_To_Delete.append(idx[i,j])
    print ("cloud length before subtraction: ", len(cloud))
    cloud = np.delete(cloud,Points_To_Delete,axis = 0)
    print ("cloud length after subtraction: ", len(cloud))
    cloud = cloud[cloud[:,0] != 0 , :]
    return cloud

def resample(cloud, np_cloud, n = 5):
    """
    Input: Numpy array of point cloud without downsample, 
            Numpy array of downsampled point cloud
    Method: take the left n points and right n points in downsampled point cloud
            add the to the final point cloud;
    Output: resampled point cloud
    """

    color_idx = np_cloud[:,3]
    for i in range(1,n):
        color_idx = np.append(color_idx, np_cloud[:,3]+i, axis = 0)
        color_idx = np.append(color_idx, np_cloud[:,3]-i, axis = 0) 
    color_idx =  color_idx[color_idx[:] < 307200]
    color_idx =  color_idx[color_idx[:] > 0]
    return cloud[color_idx.astype(int)]

def objectMatch(cloud,histogram_dict):
    """
    Input: Numpy array of cloud with RGB of current object, dictionary of all the object in the shelf
    Output: ID of specific point cloud 
    """
    uv = [color.rgb_to_yuv(*rgb)[1:3] for rgb in cloud[:,4:7]]
    hist = color.make_uv_hist(uv)
    scores = dict([ (obj, 2 * np.minimum(hist, histogram).sum() - np.maximum(hist, histogram).sum()) for (obj, histogram) in histogram_dict.items()])
    sorted_score = sorted(scores.items(), key=operator.itemgetter(1),reverse = True)
    obj = sorted_score[0][0]
    score = sorted_score[0][1]
    if VERBOSE:
        print 'found object ' + str((obj-1)/NUM_HIST_PER_OBJECT+1) + ' score is ' + str(score)+ 'cloud com is '
        print com(cloud) 
    return (obj-1)/NUM_HIST_PER_OBJECT+1,score

def loadHistogram(objects):
    """
    Input: list of index of objects
    Output: dictionary of histogram, (index -1)*3+1,2,3 as Key, histogram as value
    """
    histo_dict = {}
    for i in objects:
        for j in range(1, NUM_HIST_PER_OBJECT+1):
            idx = (i-1) * NUM_HIST_PER_OBJECT + j
            if VERBOSE:
                print 'load histgram ' + str(idx)
            histo_dict[idx] = np.load(PERCEPTION_DIR + '/{}.npz'.format(idx))['arr_0']
    return histo_dict

def build_hist_dict(objects):
    my_dict = {
        "i_am_a_bunny_book":1,
        "laugh_out_loud_joke_book":2,
        "scotch_bubble_mailer":3,
        "up_glucose_bottle":4,
        "dasani_water_bottle":5,
        "rawlings_baseball":6,
        "folgers_classic_roast_coffee":7,
        "elmers_washable_no_run_school_glue":8,
        "hanes_tube_socks":9,
        "womens_knit_gloves":10,
        "cherokee_easy_tee_shirt":11,
        "peva_shower_curtain_liner":12,
        "cloud_b_plush_bear":13,
        "barkely_hide_bones":14,
        "kyjen_squeakin_eggs_plush_puppies":15,
        "cool_shot_glue_sticks":16,
        "creativity_chenille_stems":17,
        "soft_white_lightbulb":18,
        "safety_first_outlet_plugs":19,
        "oral_b_toothbrush_green":20,
        "oral_b_toothbrush_red": 21,
        "dr_browns_bottle_brush":22,
        "command_hooks":23,
        "easter_turtle_sippy_cup":24,
        "fiskars_scissors_red":25,
        "scotch_duct_tape":26,
        "woods_extension_cord":27,
        "platinum_pets_dog_bowl":28,
        "fitness_gear_3lb_dumbbell":29,
        "rolodex_jumbo_pencil_cup":30,
        "clorox_utility_brush":31,
        "kleenex_paper_towels":32,
        "expo_dry_erase_board_eraser":33,
        "kleenex_tissue_box":34,
        "ticonderoga_12_pencils":35,
        "crayola_24_ct":36,
        "jane_eyre_dvd":37,
        "dove_beauty_bar":38,
        "staples_index_cards":39
        }
    items = []
    for item in objects:
        items.append(my_dict[item])
    return items


def com(cloud):
    return np.mean(cloud,axis = 0)
