import logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)-15s %(filename)s:%(lineno)d %(levelname)s: %(message)s')
logging.getLogger('OpenGL').setLevel(99)

# allow importing from the repository root
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy, json
from klampt.robotsim import WorldModel
from klampt import se3, so3

import apc.baxter_scoop as baxter
from api.shared import KnowledgeBase
from apc.motion import PhysicalLowLevelController as LowLevelController
from integration.visualization import debug_cloud
from integration.camera import RemoteCamera
from integration.camera import uvtexture, invalid_mask
from integration.io import pcd
from perception.segmentation.shelf import shelf_subtract

limb = sys.argv[1]
if limb not in [ 'right', 'left' ]:
    logger.error('invalid limb: {}'.format(limb))
    raise SystemExit
bin = sys.argv[2]

world = WorldModel()
model = world.loadRobot(os.path.join('klampt_models', baxter.klampt_model_name))
shelf = world.loadRigidObject(os.path.join('klampt_models', 'north_shelf', 'shelf_with_bins.obj'))

robot = LowLevelController(model, baxter.klampt_model_name)
model.setConfig(robot.getSensedConfig())

shelf_xform = (
    numpy.array([[  0,  1,  0 ],
                 [ -1,  0,  0 ],
                 [  0,  0,  1 ]]),
    [ 1.03, -0.055, -0.9046 ]
)
shelf.setTransform(list(shelf_xform[0].flat), shelf_xform[1])

SHELF_DIMS_PATH = os.path.join('kb', 'shelf_dims.json')
SHELF_CLOUD_PATH = os.path.join('perception', 'segmentation', 'models', 'shelf_thin_10000.npy')
    
# load the shelf subtraction data
bin_bounds = json.load(open(SHELF_DIMS_PATH))[bin]
# load and transform the shelf point cloud
shelf_cloud = numpy.load(open(SHELF_CLOUD_PATH))
shelf_cloud = shelf_cloud.dot(shelf_xform[0]) + shelf_xform[1]

if limb == 'left':
    realsense_pc = '192.168.0.103'
    base_xform = model.getLink('left_gripper').getTransform()
else:
    realsense_pc = '192.168.0.104'
    base_xform = model.getLink('right_gripper').getTransform()

# camera = RemoteRawCamera(realsense_pc)
# (color, cloud, depth_uv) = camera.read()
# camera.close()

# # flip everything up/down based on camera mounting
# color = color[::-1,:,:]
# cloud = cloud[::-1,:,:]
# depth_uv = depth_uv[::-1,:,:]
# # the point cloud and the depth UV map actually need to have their values changed
# # because the Y spatial direction is reversed
# cloud[:,:,1] *= -1
# depth_uv[:,:,1] = 1 - depth_uv[:,:,1]

# # compute the invalid mask early because world coordinates make
# # the z = 0 invalid points non-zero
# mask = invalid_mask(cloud)
# print mask

# # convert point cloud to meters
# cloud /= 1000

# apply the camera xform
if limb == 'left':
    camera_xform = se3.mul(base_xform, KnowledgeBase.left_camera_offset_xform)
else:
    camera_xform = se3.mul(base_xform, KnowledgeBase.right_camera_offset_xform)

# cloud = cloud.dot(numpy.array(camera_xform[0]).reshape((3, 3))) + camera_xform[1]

# # filter out the invalid points
# colorized = uvtexture(color, depth_uv)[mask]
# cloud = cloud[mask]
# cloud_color = zip(cloud.tolist(), colorized.tolist())
# print sum(mask.astype(numpy.int).flat)

camera = RemoteCamera(realsense_pc, xform=camera_xform)
camera.read()
camera.close()

# clean up the point cloud
(clean_mask, clean_cloud) = camera.clean()
_, clean_cloud_aligned, _, clean_object_mask = shelf_subtract(shelf_cloud, clean_cloud, shelf, bin_bounds, downsample=1000)
object_mask = numpy.zeros(camera.cloud.shape[:2], dtype=numpy.bool)
object_mask[clean_mask] = clean_object_mask

clean_cloud_aligned_color = map(lambda x: x[0] + [ x[1] ], zip(clean_cloud_aligned[clean_object_mask].tolist(), camera.colorize()[clean_mask][clean_object_mask].tolist()))
debug_cloud([ shelf_cloud, clean_cloud, clean_cloud_aligned ])
debug_cloud([ clean_cloud_aligned_color ], [ base_xform, camera_xform ], world=world)      
#debug_cloud([ cloud_color ], xforms=[ base_xform, camera_xform ], world=world)

pcd.write(clean_cloud_aligned_color, '/tmp/test.pcd')
