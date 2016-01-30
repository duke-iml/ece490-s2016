# allow importing from the repository root
import sys, os, json
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import apc.baxter_scoop as baxter
from api.shared import KnowledgeBase
from apc.motion import PhysicalLowLevelController as LowLevelController
from integration.visualization import debug_cloud
from integration.camera.client import RemoteRawCamera
from integration.camera import uvtexture, invalid_mask
from integration.io import pcd
from klampt.robotsim import WorldModel
from klampt import se3,so3
from time import sleep, time


try:
	filename = sys.argv[1]
	link_names = sys.argv[2:]
except IndexError:
	print 'missing argument'
	print sys.argv[0], '[save base name] [link names]...'
	raise SystemExit

if os.path.exists(filename):
	print 'Save file', filename, 'already exists. Overwrite? [y/N]',
	prompt = raw_input()
	if prompt.lower() != 'y':
		raise SystemExit

world = WorldModel()
model = world.loadRobot(os.path.join('klampt_models', baxter.klampt_model_name))
shelf = world.loadRigidObject(os.path.join('klampt_models', 'north_shelf', 'shelf_with_bins.obj'))

robot = LowLevelController(model, baxter.klampt_model_name)

limb = 'left'
bin = 'bin_A'

if limb == 'left':
    realsense_pc = '192.168.0.103'
    base_xform = model.getLink('left_gripper').getTransform()
    camera_xform = se3.mul(base_xform, KnowledgeBase.left_camera_offset_xform)
else:
    realsense_pc = '192.168.0.104'
    base_xform = model.getLink('right_gripper').getTransform()
    camera_xform = se3.mul(base_xform, KnowledgeBase.right_camera_offset_xform)

save_file = open('{}.json'.format(filename), 'w')

print 'ready',

n = 0

save_file.write('[\n')

while True:
	try:
		raw_input()
	except KeyboardInterrupt:
		break

	qs = robot.getSensedConfig()
	model.setConfig(qs)
	xforms = [ model.getLink(l).getTransform() for l in link_names ]

	pcd_path = '{}_{}.pcd'.format(filename, n)
    camera = RemoteCamera(address, xform=self.camera_xform)
    camera.read()
    camera.close()

    
	
	if n > 0:
		save_file.write(',\n')

	save_file.write(json.dumps({
		'time': time(),
		'id': n,
		'pcd': pcd_path,
		'xforms': dict(zip(link_names, xforms)),
		'config': qs
	}, indent=4))
	save_file.flush()

	n += 1
	print 'captured', n,

save_file.write(']\n')
save_file.close()
