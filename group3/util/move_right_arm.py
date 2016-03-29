import rospy
import baxter_interface

# Change me!
q_target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

rospy.init_node('oeu')
limb = baxter_interface.Limb('right')
link_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
new_q = dict([(link_name, q_target[index]) for index,link_name in enumerate(link_names)])
print new_q
limb.move_to_joint_positions(new_q)
