import rospy
import baxter_interface
#from constants import *

import sys

sys.path.insert(0,"..")

from Trajectories.camera_to_bin import *
from Trajectories.view_to_grasp import *
from Trajectories.grasp_to_stow_90 import *
from Trajectories.grasp_to_stow_straight import *
from Trajectories.grasp_to_rest import *
from Trajectories.Human_Testing import *

if(len(sys.argv) == 2):
	print sys.argv[1]
	q_target = eval(sys.argv[1])
	print 'Moving to ' , q_target
elif (len(sys.argv) == 3):
	q_target = eval(sys.argv[1])
	q_target = q_target[eval(sys.argv[2])]
else:
	q_target = Q_STOW
	print 'Moving to ', q_target
# Change me!


rospy.init_node('oeu')
limb = baxter_interface.Limb('right')
link_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
new_q = dict([(link_name, q_target[index]) for index,link_name in enumerate(link_names)])
print new_q
limb.move_to_joint_positions(new_q)