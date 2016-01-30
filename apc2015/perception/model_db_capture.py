# allow importing from the repository root
import sys, os, json
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import apc.baxter_scoop as baxter
from api.shared import KnowledgeBase
from apc.motion import PhysicalLowLevelController as LowLevelController
from integration.visualization import debug_cloud
from integration.camera.client import RemoteRawCamera
from integration.camera import uvtexture, invalid_mask, RemoteCamera
from integration.io import pcd

from klampt.robotsim import WorldModel
from klampt import se3,so3
from time import sleep, time
import numpy

world = WorldModel()
model = world.loadRobot(os.path.join('klampt_models', baxter.klampt_model_name))
shelf = world.loadRigidObject(os.path.join('klampt_models', 'north_shelf', 'shelf_with_bins.obj'))

robot = LowLevelController(model, baxter.klampt_model_name)

limb = 'left'
bin = 'bin_A'
base_path = os.path.join('perception', 'models_db2')

objects = json.load(open(os.path.join('perception', 'null_match_values.json'))).keys()

for obj in objects:
    print 'swtich to', obj
    
    for n in range(6):
        print 'take', n,

        try:
            raw_input()
        except KeyboardInterrupt:
            raise SystemExit

        qs = robot.getSensedConfig()
        model.setConfig(qs)

        if limb == 'left':
            realsense_pc = '192.168.0.104'
            base_xform = model.getLink('left_gripper').getTransform()
            camera_xform = se3.mul(base_xform, KnowledgeBase.left_camera_offset_xform)
        else:
            realsense_pc = '192.168.0.104'
            base_xform = model.getLink('right_gripper').getTransform()
            camera_xform = se3.mul(base_xform, KnowledgeBase.right_camera_offset_xform)

        camera = RemoteCamera(realsense_pc, xform=camera_xform)
        camera.read()
        camera.close()

        for f in [ 'color', 'cloud', 'depth_uv' ]:
            numpy.save(os.path.join(base_path, '{}_{}_{}.npy'.format(obj, n, f)), getattr(camera, f))

        print 'captured'

print 'done'
