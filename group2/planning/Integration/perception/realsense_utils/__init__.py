
from __future__ import division
from common_utils import *

def unnormalize_uv_map(uv_map, target_shape):
    '''
    Convert uv_map from 0-1 scale to true scale
    uv_map is an (m x n x 2) matrix of mapping from (m x n) matrix A to (p x q) matrix B. 
    A[i,j] corresponds to B[uv_map[i,j,0]*p, uv_map[i,j,1]*q]
    Thus, by unnormalizing uv_map into uv_map2, A[i,j] corresponds to B[uv_map2[i,j,0], uv_map2[i,j,1]]
    '''
    uv_map2 = uv_map.copy()
    for i in range(2):
        uv_map2[:,:,i] = np.clip(uv_map2[:,:,i] * target_shape[i], 0, target_shape[i]-1)
    uv_map2 = uv_map2.round().astype(np.int)
    return uv_map2

def spread_color_to_depth(color, depth_uv):
    '''
    Colorize depth image. Color image is of size (m x n x 3). Depth image is of size (p x q). 
    depth_uv is of size (p x q x 2) and maps each point on depth to a point on color image. 
    However, depth_uv[i,j,0] gives the *column index* of the corresponding point on color image. 
    Thus, depth_uv needs to be inverted along the last dimension. 
    This method returns a warped color image that can be overlayed on depth image and return an image of size (p x q x 3). 
    For realsense, the depth image has a wider field of view so there are some points in the depth image that are outside of the range of the color camera. 
    For those points, pure green (0, 255, 0) is assigned. 
    '''
    # lookup value in source image
    matched = color[depth_uv[:,:,0], depth_uv[:,:,1]]
    # set invalid pixels to green
    invalid_mask = (depth_uv[:,:,0] <= 0) | (depth_uv[:,:,1] <= 0) | (depth_uv[:,:,0] >= color.shape[0]-1) | (depth_uv[:,:,1] >= color.shape[1]-1)
    matched[invalid_mask] = [ 0, 255, 0 ]
    return matched

def colorize_point_cloud(cloud, color, depth_uv, rgb_table=None):
    '''
    Colorize a point cloud. cloud is of size (p x q x 3) where (p x q) is the size of the depth image. 
    It is generated by "protruding" each point on the depth image to 3D camera coordinate (done in native realsense SDK). 
    depth_uv is a mapping from depth image to color image as described in spread_color_to_depth. 
    color is color image. rgb_table is a lookup table that converts (r,g,b) to a floating point in PCL format. 
    If rgb_table is None, conversion is done at runtime. 
    Return a matrix PC of (p x q x 4) where where PC[i,j,:] gives the colored point (x,y,z,rgb_float) corresponding to point (i,j) in the depth image. 
    '''
    matched = spread_color_to_depth(color, depth_uv)
    if rgb_table is None:
        matched_pcl_float = np.apply_along_axis(lambda p: rgb_to_pcl_float(*p.tolist()), 2, matched)
    else:
        matched_pcl_float = np.apply_along_axis(lambda p: rgb_table[p[0], p[1], p[2]], 2, matched)
    return np.dstack([cloud, matched_pcl_float])

def get_3d_camera_coord_for_rgb_coord(i, j, cloud, color_uv):
    '''
    Find the 3d coordinate in camera frame of a point in rgb image. i is the row index and j is the column index. 
    color_uv is the already unnormalized uv mapping from color image to depth image. 
    '''
    return cloud[color_uv[i, j, 0], color_uv[i, j, 1], :]