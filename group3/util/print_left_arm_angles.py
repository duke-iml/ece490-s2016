import rospy
import baxter_interface

rospy.init_node('oeu')
limb = baxter_interface.Limb('left')
q = limb.joint_angles()
link_names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
new_q = [q[name] for name in link_names]
print new_q
