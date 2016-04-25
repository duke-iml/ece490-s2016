import rospy
import baxter_interface

rospy.init_node('oeu')
head = baxter_interface.Head()
head.set_pan(1.0)