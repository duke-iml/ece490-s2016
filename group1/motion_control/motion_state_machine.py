import rospy
import baxter_interface
import time
import json

''' 
Motion State Machine
The motion state machine is the funcitonal broad-strokes class to use when moving from the tote to a shelf, or moving from
a shelf to the tote.  This class uses specific file names in the folder "state_json_files" to map configurations read
from the joint_reader_utility into the specific global variables.

To change the joint angles associated with intermediate motions, shelf positions, or tote positions, please use the 
joint_reader_utility - provided the files are named correctly when using the joint_reader_utility, there should be
no need to change the code in this class

Shelf diagram:
------------------------------
|Shelf 1 | Shelf 2 | Shelf 3 | -> Row 1
|        |         |         |
------------------------------
|Shelf 4 | Shelf 5 | Shelf 6 | -> Row 2
|        |         |         |
------------------------------
|Shelf 7 | Shelf 8 | Shelf 9 | -> Row 3
|        |         |         |
------------------------------
|Shelf 10| Shelf 11| Shelf 12| -> Row 4
|        |         |         |
------------------------------

Motion State Machine API:
    load_configs():
        Inputs -> None
        Outputs -> None
        Description -> Loads configurations from pre-named JSON files to global variables
                        (This is done when the motion state class is initialized)

    start_move_to_config(row):
        Inputs -> [Row] Row number identifying which row the shelf the arm is moving to is in
        Outputs -> None
        Description -> Intiates a movement from the tote to the input row by performing two intermediate motions
                        These intermediate motions help 1) To move the arm up the appropriate height and 2) to
                        position the hand to face the shelf without knocking into the shelf.  These intermediate
                        motions are defined by calibrating the arm with the joint_reader_utility

    move_to_bin_from_config(row):
        Inputs -> [Row] Row number identifying which row the shelf the arm is moving from is in
        Outputs -> None
        Description -> Initiates a movement from an individual shelf to the tote by perforing two intermediate motions
                        These intermediate motions help 1) To move the arm back from a particular shelf and 2) to
                        position the hand so that it can reach down into the tote without knocking into the shelf.  These 
                        intermediate motions are defined by calibrating the arm with the joint_reader_utility

    move to shelf(shelf):
        Inputs -> [Shelf] Shelf number identifying which shelf the arm should move to
        Outputs -> None
        Description -> Completes a movement from the tote to a particular shelf, using the number scheme shown above.
                        This will first call the function start_move_to_config to identify which set of intermediate
                        positions are necessary, and then completes the movement to a generalized center location for
                        a shelf.  The specific 'generalized' locations are defined by calibrating the arm with the joint_reader_utility
        

Dennis Lynch
2-29-16
'''

class motion_state_machine:

    # The default intilization method for the motion state machine class
    def __init__(self):
        # Initializes a ROS node to use baxter commands
        rospy.init_node('motion_state_machine_node')
        
        # Load configurations into appropriate variable names
        self.load_configs()

        # Assign Baxter's limbs to variables
        global right_limb
        global left_limb
        right_limb = baxter_interface.Limb('right')
        left_limb = baxter_interface.Limb('left')


    def load_configs(self):
        # Define the global variables used by the motion functions
        # The tote
        global bin_right
        # The intermediate motions, named rbin_inter_r[rownumber]_[a/b]
        global rbin_inter_r1_a  
        global rbin_inter_r1_b
        global rbin_inter_r2_a
        global rbin_inter_r2_b
        global rbin_inter_r3_a
        global rbin_inter_r3_b
        global rbin_inter_r4_a
        global rbin_inter_r4_b
        # The generalized center points for each of the twelve shelves
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

        # Open up the joint angle json files and load them into appropriate variables
        import os
        print os.getcwd()
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

    # Begins a move from the tote to a specific row configuration - This will NOT bring the arm right
    # up to the generalized center-point for a shelf, but a generalized location for a particular row
    def start_move_to_config(self, row):
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

    # Completes a full move from a particular shelf to the tote.  Need to only define the row, however,
    # as every shelf on a particular row shares the same intermediate positions before going to the
    # single tote
    def move_to_bin_from_config(self, row):
        print "Moving to the bin from the config"
        if row==1:
            right_limb.move_to_joint_positions(rbin_inter_r1_b)
            right_limb.move_to_joint_positions(rbin_inter_r1_a)
            right_limb.move_to_joint_positions(bin_right)
        elif row==2:
            print "Moving to the intermediate b"
            print rbin_inter_r2_b
            right_limb.move_to_joint_positions(rbin_inter_r2_b)
            print "Moving to the intermediate a"
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

    # Completes a full move from the tote to one of the 12 shelves in the APC configuration.  This calls
    # start_move_to_config based on the row associated with a shelf, and finalizes the motion by calling
    # the config directly associated with a generalized center point.  Moving to the shelf config directly
    # will increase the probabilty of hitting the shelf.
    def move_to_shelf(self, shelf):
        if shelf == 1:
            self.start_move_to_config(1)
            right_limb.move_to_joint_positions(rbin_shelf_1)
        elif shelf == 2:
            self.start_move_to_config(1)
            right_limb.move_to_joint_positions(rbin_shelf_2)
        elif shelf == 3:
            self.start_move_to_config(1)
            right_limb.move_to_joint_positions(rbin_shelf_3)
        elif shelf == 4:
            self.start_move_to_config(2)
            right_limb.move_to_joint_positions(rbin_shelf_4)
        elif shelf == 5:
            self.start_move_to_config(2)
            right_limb.move_to_joint_positions(rbin_shelf_5)
        elif shelf == 6:
            self.start_move_to_config(2)
            right_limb.move_to_joint_positions(rbin_shelf_6)
        elif shelf == 7:
            self.start_move_to_config(3)
            right_limb.move_to_joint_positions(rbin_shelf_7)
        elif shelf == 8:
            self.start_move_to_config(3)
            right_limb.move_to_joint_positions(rbin_shelf_8)
        elif shelf == 9:
            self.start_move_to_config(3)
            right_limb.move_to_joint_positions(rbin_shelf_9)
        elif shelf == 10:
            self.start_move_to_config(4)
            right_limb.move_to_joint_positions(rbin_shelf_10)
        elif shelf == 11:
            self.start_move_to_config(4)
            right_limb.move_to_joint_positions(rbin_shelf_11)
        else:
            self.start_move_to_config(4)
            right_limb.move_to_joint_positions(rbin_shelf_12)
        
