import rospy
import baxter_interface

# Change me!
q_target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
q_target = [1.1508690847961427, -0.7558690323669434, -0.3704563598510742, 1.1922865660217286, 0.19596604542846682, 1.2455923983398438, -0.07708253450317383]


rospy.init_node('oeu')
limb = baxter_interface.Limb('right')
link_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
new_q = dict([(link_name, q_target[index]) for index,link_name in enumerate(link_names)])
print new_q
limb.move_to_joint_positions(new_q)
