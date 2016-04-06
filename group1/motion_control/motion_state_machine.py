import rospy
import baxter_interface
import time
import json




''' 
Sample of a motion state machine, using info spit out from the joint_reader_utility 

Dennis Lynch
2-29-16
'''

class motion_state_machine:

    def __init__:
        rospy.init_node('right_arm_motion_node')
        load_configs()
        right_limb = baxter_interface.Limb('right')
        left_limb = baxter_interface.Limb('left')


    def load_configs():
        global bin_right
        global rbin_inter_r1_a
        global rbin_inter_r1_b
        global rbin_inter_r2_a
        global rbin_inter_r2_b
        global rbin_inter_r3_a
        global rbin_inter_r3_b
        global rbin_inter_r4_a
        global rbin_inter_r4_b
        global rbin_shelf_1
        global rbin_shelf_2
        global rbin_shelf_3
        global rbin_shelf_4
        global rbin_shelf_5
        global rbin_shelf_6
        global rbin_shelf_7
        global rbin_shelf_8
        global rbin_shelf_9
        global rbin_shelf_10
        global rbin_shelf_11
        global rbin_shelf_12
        global rbin_shelf_13
        global rbin_shelf_14
        global rbin_shelf_15
        global rbin_shelf_16

        with open("state_json_files/bin_right.json") as file:
            bin_right = json.load(file)

        with open("state_json_files/rbin_inter_r1_a.json") as file:
            rbin_inter_r1_a = json.load(file)

        with open("state_json_files/rbin_inter_r1_b.json") as file:
            rbin_inter_r1_b = json.load(file)

        with open("state_json_files/rbin_inter_r2_a.json") as file:
            rbin_inter_r2_a = json.load(file)

        with open("state_json_files/rbin_inter_r2_b.json") as file:
            rbin_inter_r2_b = json.load(file)

        with open("state_json_files/rbin_inter_r3_a.json") as file:
            rbin_inter_r3_a = json.load(file)

        with open("state_json_files/rbin_inter_r3_b.json") as file:
            rbin_inter_r3_b = json.load(file)

        with open("state_json_files/rbin_inter_r4_a.json") as file:
            rbin_inter_r4_a = json.load(file)

        with open("state_json_files/rbin_inter_r4_b.json") as file:
            rbin_inter_r4_b = json.load(file)

        with open("state_json_files/rbin_shelf_1.json") as file:
            rbin_shelf_1 = json.load(file)

        with open("state_json_files/rbin_shelf_2.json") as file:
            rbin_shelf_2 = json.load(file)

        with open("state_json_files/rbin_shelf_3.json") as file:
            rbin_shelf_3 = json.load(file)

        with open("state_json_files/rbin_shelf_4.json") as file:
            rbin_shelf_4 = json.load(file)

        with open("state_json_files/rbin_shelf_5.json") as file:
            rbin_shelf_5 = json.load(file)

        with open("state_json_files/rbin_shelf_6.json") as file:
            rbin_shelf_6 = json.load(file)

        with open("state_json_files/rbin_shelf_7.json") as file:
            rbin_shelf_7 = json.load(file)

        with open("state_json_files/rbin_shelf_8.json") as file:
            rbin_shelf_8 = json.load(file)

        with open("state_json_files/rbin_shelf_9.json") as file:
            rbin_shelf_9 = json.load(file)

        with open("state_json_files/rbin_shelf_10.json") as file:
            rbin_shelf_10 = json.load(file)

        with open("state_json_files/rbin_shelf_11.json") as file:
            rbin_shelf_11 = json.load(file)

        with open("state_json_files/rbin_shelf_12.json") as file:
            rbin_shelf_12 = json.load(file)

        with open("state_json_files/rbin_shelf_13.json") as file:
            rbin_shelf_13 = json.load(file)

        with open("state_json_files/rbin_shelf_14.json") as file:
            rbin_shelf_14 = json.load(file)

        with open("state_json_files/rbin_shelf_15.json") as file:
            rbin_shelf_15 = json.load(file)

        with open("state_json_files/rbin_shelf_16.json") as file:
            rbin_shelf_16 = json.load(file)

    def start_move_to_config(row):
        if row==1:
            right_limb.move_to_joint_positions(rbin_inter_r1_a)
            right_limb.move_to_joint_positions(rbin_inter_r1_b)
        elif row==2:
            right_limb.move_to_joint_positions(rbin_inter_r2_a)
            right_limb.move_to_joint_positions(rbin_inter_r2_b)
        elif row==3:
            right_limb.move_to_joint_positions(rbin_inter_r3_a)
            right_limb.move_to_joint_positions(rbin_inter_r3_b)
        else:
            right_limb.move_to_joint_positions(rbin_inter_r4_a)
            right_limb.move_to_joint_positions(rbin_inter_r4_b)

    def start_move_to_bin_from_config(row):
        if row==1:
            right_limb.move_to_joint_positions(rbin_inter_r1_b)
            right_limb.move_to_joint_positions(rbin_inter_r1_a)
            right_limb.move_to_joint_positions(bin_right)
        elif row==2:
            right_limb.move_to_joint_positions(rbin_inter_r2_b)
            right_limb.move_to_joint_positions(rbin_inter_r2_a)
            right_limb.move_to_joint_positions(bin_right)
        elif row==3:
            right_limb.move_to_joint_positions(rbin_inter_r3_b)
            right_limb.move_to_joint_positions(rbin_inter_r3_a)
            right_limb.move_to_joint_positions(bin_right)
        else:
            right_limb.move_to_joint_positions(rbin_inter_r4_b)
            right_limb.move_to_joint_positions(rbin_inter_r4_a)
            right_limb.move_to_joint_positions(bin_right)

    def move_to_bin(bin):
        if bin == 1:
            start_move_to_config(1)
            right_limb.move_to_joint_positions(rbin_shelf_1)
            start_move_to_bin_from_config(1)
        elif bin == 2:
            start_move_to_config(1)
            right_limb.move_to_joint_positions(rbin_shelf_2)
            start_move_to_bin_from_config(1)
        elif bin == 3:
            start_move_to_config(1)
            right_limb.move_to_joint_positions(rbin_shelf_3)
            start_move_to_bin_from_config(1)
        elif bin == 4:
            start_move_to_config(1)
            right_limb.move_to_joint_positions(rbin_shelf_4)
            start_move_to_bin_from_config(1)
        elif bin == 5:
            start_move_to_config(2)
            right_limb.move_to_joint_positions(rbin_shelf_5)
            start_move_to_bin_from_config(2)
        elif bin == 6:
            start_move_to_config(2)
            right_limb.move_to_joint_positions(rbin_shelf_6)
            start_move_to_bin_from_config(2)
        elif bin == 7:
            start_move_to_config(2)
            right_limb.move_to_joint_positions(rbin_shelf_7)
            start_move_to_bin_from_config(2)
        elif bin == 8:
            start_move_to_config(2)
            right_limb.move_to_joint_positions(rbin_shelf_8)
            start_move_to_bin_from_config(2)
        elif bin == 9:
            start_move_to_config(3)
            right_limb.move_to_joint_positions(rbin_shelf_9)
            start_move_to_bin_from_config(3)
        elif bin == 10:
            start_move_to_config(3)
            right_limb.move_to_joint_positions(rbin_shelf_10)
            start_move_to_bin_from_config(3)
        elif bin == 11:
            start_move_to_config(3)
            right_limb.move_to_joint_positions(rbin_shelf_11)
            start_move_to_bin_from_config(3)
        elif bin == 12:
            start_move_to_config(3)
            right_limb.move_to_joint_positions(rbin_shelf_12)
            start_move_to_bin_from_config(3)
        elif bin == 13:
            start_move_to_config(4)
            right_limb.move_to_joint_positions(rbin_shelf_13)
            start_move_to_bin_from_config(4)
        elif bin == 14:
            start_move_to_config(4)
            right_limb.move_to_joint_positions(rbin_shelf_14)
            start_move_to_bin_from_config(4)
        elif bin == 15:
            start_move_to_config(4)
            right_limb.move_to_joint_positions(rbin_shelf_15)
            start_move_to_bin_from_config(4)
        else:    
            start_move_to_config(4)
            right_limb.move_to_joint_positions(rbin_shelf_16)
            start_move_to_bin_from_config(4)
        
