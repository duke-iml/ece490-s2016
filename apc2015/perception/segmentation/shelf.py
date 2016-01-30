#!/usr/bin/env python

import logging, traceback
logger = logging.getLogger(__name__)

# allow importing from this folder
import sys, os
sys.path.append(os.path.dirname(__file__))

import numpy
import scipy.spatial
from time import time
import json

from klampt import se3, so3
from klampt.robotsim import WorldModel, GeometricPrimitive, Geometry3D

from integration.visualization import debug_cloud
from integration.io import pcd
from perception.segmentation import icp

def shelf_subtract(shelf_cloud, camera_cloud, shelf_model, bounds, downsample=100):
    # generate the shelf kdtree
    mark = time()
    shelf_tree = scipy.spatial.KDTree(shelf_cloud)
    logger.info('generated shelf tree in {:.3f}s'.format(time() - mark))
    
    # align the camera cloud with ICP
    mark = time()
    diff_xform = icp.match(camera_cloud[::downsample], shelf_tree, initial_threshold=0.05)
    camera_cloud_aligned = camera_cloud.dot(diff_xform[0].T) + diff_xform[1]
    logger.info('aligned camera cloud in {:.3f}s'.format(time() - mark))

    # remove all points near the shelf
    mark = time()
    
    # _, ind = shelf_tree.query(camera_cloud_aligned, distance_upper_bound=0.005)
    # mask = ind >= len(shelf_tree.data)
    # segmented_cloud = camera_cloud_aligned[mask]

    test_point = GeometricPrimitive()
    test_point.setPoint((0, 0, 0))
    
    test_geom = Geometry3D()
    test_geom.setGeometricPrimitive(test_point)
    
    def collides(point):
        test_geom.setCurrentTransform(so3.identity(), point)
        return test_geom.withinDistance(shelf_model.geometry(), 0.01)

    mask = map(lambda p: not collides(p), camera_cloud_aligned)
    # segmented_cloud = [ p for (i, p) in enumerate(camera_cloud_aligned) if mask[i] ]
    
    camera_cloud_aligned_local = (camera_cloud_aligned - shelf_model.getTransform()[1]).dot(so3.matrix(shelf_model.getTransform()[0]))

    mask2 = reduce(numpy.bitwise_and, [ (bounds[0][i] < camera_cloud_aligned_local[:, i]) & (bounds[1][i] > camera_cloud_aligned_local[:, i]) for i in range(3) ])
    segmented_cloud = camera_cloud_aligned[mask & mask2]

    logger.info('segmented camera cloud in {:.3f}s: {} -> {}'.format(time() - mark, len(camera_cloud_aligned), len(segmented_cloud)))

    return diff_xform, camera_cloud_aligned, segmented_cloud, mask & mask2

def fit_object_model(model_cloud, object_cloud):
    # generate the model kdtree
    mark = time()
    object_tree = scipy.spatial.KDTree(object_cloud)
    logger.info('generated object tree in {:.3f}s'.format(time() - mark))
    
    # align the object cloud with ICP
    mark = time()
    diff_xform = icp.match(model_cloud[::100], object_tree, iterations=50, initial_threshold=0.10)
    model_cloud_aligned = model_cloud.dot(diff_xform[0].T) + diff_xform[1]
    logger.info('aligned model cloud in {:.3f}s'.format(time() - mark))

    return diff_xform, model_cloud_aligned

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG, format='%(filename)s:%(lineno)d %(levelname)s: %(message)s')
    logging.getLogger('integration.visualization').setLevel(logging.WARNING)
    logging.getLogger('pcd').setLevel(logging.WARNING)

    SHELF_DIMS_PATH = os.path.join('kb', 'shelf_dims.json')
    SHELF_CLOUD_PATH = os.path.join('perception', 'segmentation', 'models', 'shelf_10000.npy')
    CAMERA_CLOUD_PATH = sys.argv[1]
    CAMERA_XFORM_PATH = sys.argv[2]
    CAMERA_XFORM_INDEX = int(sys.argv[3])
    BIN_NAME = sys.argv[4]
    OBJECT_MODEL_PATH = sys.argv[5]

    # load the shelf model point cloud
    mark = time()
    shelf_cloud = numpy.load(open(SHELF_CLOUD_PATH))
    logging.info('loaded {} shelf points in {:.3f}s'.format(shelf_cloud.shape[0], time() - mark))

    # load the shelf xform
    shelf_xform = (
        numpy.array([[  0,  1,  0 ],
                     [ -1,  0,  0 ],
                     [  0,  0,  1 ]]),
        [ 1.03, -0.055, -0.9046 ]
    )

    # transform the shelf point cloud
    shelf_cloud = shelf_cloud.dot(shelf_xform[0]) + shelf_xform[1]

    # load the camera point cloud
    mark = time()
    camera_cloud_color = pcd.parse(open(CAMERA_CLOUD_PATH))[1]
    camera_cloud = numpy.array([ p[:3] for p in camera_cloud_color ])
    logging.info('loaded {} camera points in {:.3f}s'.format(camera_cloud.shape[0], time() - mark))

    # load the camera xform
    gripper_xform = json.load(open(CAMERA_XFORM_PATH))[CAMERA_XFORM_INDEX]['xforms']['left_gripper']
    camera_xform = se3.mul(gripper_xform, ( so3.rotation([0,0,1], -3.141592/2), [ -0.10, 0, 0 ] ))

    # transform the camera point cloud
    camera_cloud = camera_cloud.dot(numpy.array(camera_xform[0]).reshape((3, 3))) + camera_xform[1]

    # load the object model point cloud
    mark = time()
    model_cloud_color = pcd.parse(open(OBJECT_MODEL_PATH))[1]
    model_cloud = numpy.array([ p[:3] for p in model_cloud_color ])
    logging.info('loaded {} model points in {:.3f}s'.format(model_cloud.shape[0], time() - mark))

    # set up the bin bounds
    bin_bounds = json.load(open(SHELF_DIMS_PATH))[BIN_NAME]

    # for (i, p) in enumerate(camera_cloud_color):
        # p[:3] = camera_cloud[i]
    # debug_cloud([ shelf_cloud, camera_cloud ])

    world = WorldModel()
    shelf = world.loadRigidObject('klampt_models/north_shelf/shelf.obj')
    shelf.setTransform(list(shelf_xform[0].flat), shelf_xform[1])

    _, camera_cloud_aligned, object_cloud, mask = shelf_subtract(shelf_cloud, camera_cloud, shelf, bin_bounds)

    # debug_cloud([ shelf_cloud, camera_cloud, camera_cloud_aligned ])
    # debug_cloud([ object_cloud ], world=world)
    debug_cloud([ shelf_cloud, camera_cloud, camera_cloud_aligned ])
    debug_cloud([ object_cloud, camera_cloud_aligned ])

    object_cloud_color = []
    for i in range(len(mask)):
        if mask[i]:
            object_cloud_color.append(camera_cloud_color[i])
    for (i, p) in enumerate(object_cloud_color):
        p[:3] = object_cloud[i]
    debug_cloud([ object_cloud_color ], world=world)

    pcd.write(object_cloud_color, '/tmp/object.pcd')

    _, model_cloud_aligned = fit_object_model(model_cloud, object_cloud)

    debug_cloud([ model_cloud, object_cloud, model_cloud_aligned ])
