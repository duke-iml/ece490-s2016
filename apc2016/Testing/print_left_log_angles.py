import rospy
import baxter_interface

rospy.init_node('oeu')
limb = baxter_interface.Limb('left')
q = limb.joint_angles()
link_names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']

while(1):
	try:
		bin = raw_input("Enter Bin name \n")

		new_q = [q[name] for name in link_names]
		print bin, new_q

		if bin == 'q':
			break

	except KeyboardInterrupt:
		break