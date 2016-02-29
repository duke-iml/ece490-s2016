import rospy
import baxter_interface
import time
rospy.init_node('right_arm_motion_node')

''' 
Sample of a motion state machine, using info spit out from the joint_reader_utility 

Dennis Lynch
2-29-16
'''

limb = baxter_interface.Limb('right')

standard_hold = {'right_s0' : 0.651174843713 ,
'right_s1' : -1.13131082977 ,
'right_e0' : -0.108529140619 ,
'right_e1' : -0.0483203947632 ,
'right_w0' : -2.77113629015 ,
'right_w1' : -1.57424778177 ,
'right_w2' : 3.04993729803 }

top_right = {'right_s0' : 0.866315648969 ,
'right_s1' : -0.774276801801 ,
'right_e0' : -0.17257283844 ,
'right_e1' : 0.464412683002 ,
'right_w0' : -2.94179165261 ,
'right_w1' : -0.349364124042 ,
'right_w2' : 3.02424311986 }


bin_position = {'right_s0' : 1.08797587256 ,
'right_s1' : 0.165669924902 ,
'right_e0' : -0.011504855896 ,
'right_e1' : 1.3832671739 ,
'right_w0' : -2.8980732002 ,
'right_w1' : -0.023009711792 ,
'right_w2' : 2.76730133818 }



'''The 'state machine' that will move back and forth'''
limb.move_to_joint_positions(standard_hold)
time.sleep(2.0)
limb.move_to_joint_positions(top_right)
time.sleep(2.0)
limb.move_to_joint_positions(standard_hold)
time.sleep(2.0)
limb.move_to_joint_positions(bin_position)
time.sleep(2.0)
