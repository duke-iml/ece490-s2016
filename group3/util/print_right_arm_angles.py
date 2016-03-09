import rospy
import baxter_interface

rospy.init_node('oeu')
limb = baxter_interface.Limb('right')
q = limb.joint_angles()
link_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
new_q = [q[name] for name in link_names]
print new_q