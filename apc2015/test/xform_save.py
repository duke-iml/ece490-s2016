# allow importing from the repository root
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from apc.motion import PhysicalLowLevelController as LowLevelController

from klampt.robotsim import WorldModel
from klampt import se3,so3
from time import sleep, time

import sys
import json

try:
	filename = sys.argv[1]
	link_names = sys.argv[2:]
except IndexError:
	print 'missing argument'
	print sys.argv[0], '[save file] [link names]...'
	raise SystemExit

if os.path.exists(filename):
	print 'Save file', filename, 'already exists. Overwrite? [y/N]',
	prompt = raw_input()
	if prompt.lower() != 'y':
		raise SystemExit

world = WorldModel()
model = world.loadRobot('klampt_models/baxter_with_reflex_gripper_col.rob')
robot = LowLevelController(model, 'baxter_with_reflex_gripper_col.rob')

save_file = open(filename, 'w')

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

	save_file.write(json.dumps({
		'time': time(),
		'id': n,
		'xforms': dict(zip(link_names, xforms)),
		'config': qs
	}, indent=4) + ',\n')
	save_file.flush()

	n += 1
	print 'captured', n,

save_file.write(']\n')
save_file.close()
