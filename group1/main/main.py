#The "supervisor" module for the Amazing Stowing Team
import time
import json
import sys
import random
import time

sys.path.insert(0, '/home/group1/ece490-s2016/group1/motion_control')
import motion_state_machine as msm
import bump

class Supervisor:
    def __init__(self):
        print "Initialized new supervisor"
        #Uncomment only if connected to baxter
        self.state_machine = msm.motion_state_machine()
        #self.bumper = bump.Bumper()
        self.start_time = time.time()
        self.items_to_stow = 20
        # Potentially more to load here with everything else going on?

    def load_config_file(self, file_name):
        self.orig_config = file_name
        # Need to integrate JSON functionality into supervisor

    def done(self):
        if ((time.time() - self.start_time) > 60*3): #  Should be 20, but we're testing
            return True
        elif (self.items_to_stow == 0):
            return True

        return False # Didn't meet any of our end conditions

    def compete(self):
        print "Competing"
        while(True):
            # Go to the bin
            print "Going to the tote"
            self.state_machine.move_to_bin_from_config(2)
            # Get Perception Data
            print "Getting Perception Data ..."
            # Send perception data to bump function


            # TODO: For demo, pick a random coordinate set and bump to there
            print "Bumping to coordinates ..."

            # Pick up item
            # TODO: For demo, close the hand and turn on the vacuum
            print "Picking up item ..."

            # Based on item, pick a shelf to go to
            self.current_item = "DUMMY ITEM"
            # TODO: Add in Craig's bin picker instead of picking random shelf
            self.target_shelf = random.randrange(1,13) 
            print "Going to shelf", self.target_shelf
            # Go to that shelf
            self.state_machine.move_to_shelf(self.target_shelf)
            # Figure out if need to bump
            print "Getting perception data to figure out any bumps"


            # Bump to that location
            # TODO: For demo, pick a random coordinate set and bump to there
            print "Bumping to coordinates ..."

            # Drop item
            # TODO: For Demo, open the hand and turn off the vacuum
            print "Dropping item ..."
            # Update JSON file and count

            print "Updating JSON File"
            self.items_to_stow = self.items_to_stow - 1
            # If time is up or we've finished all items, break
            # Otherwise, loop
            if (self.done()):
                break


''' Main code '''
#Initialize the supervisor
sup = Supervisor()

#Ensure that a JSON file has been specified
if (len(sys.argv) < 2):
    print "Please specify a JSON file to load"
else:
    sup.load_config_file(sys.argv[1])

# Begin competing!
sup.compete()


