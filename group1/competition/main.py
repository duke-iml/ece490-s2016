#!/usr/bin/python
#The "supervisor" module for the Amazing Stowing Team
import time
import json
import sys
import random
import time, os
from json_handler import jsonHandler 
from bin_select import binSelector
import motion_state_machine as msm
import bump
from hand_handler import HandHandler
from vacuum import vacuum

class Supervisor:
    def __init__(self):
        print "Initialized new supervisor"
        self.time_limit = 3
        self.state_machine = msm.motion_state_machine()
        self.bumper = bump.Bumper((0.161, 0, 0))
        self.start_time = time.time()
        self.vac=vacuum()
        self.hand = HandHandler()
        self.handler=jsonHandler()
        (self.origMap,self.workOrder)=self.handler.readInFile("RandomTestB.json")
        self.items_to_stow = len(self.workOrder[0])
        ##Load bin Selector
        self.binSelect=binSelector()
        ##Initialize bin Selector 
        self.binSelect.initialize("RandomTestB.json")
        # Potentially more to load here with everything else going on?

    def load_config_file(self, file_name):
        self.orig_config = file_name
        # Need to integrate JSON functionality into supervisor

    def done(self):
        if ((time.time() - self.start_time) > 60*self.time_limit): #  Should be 20, but we're testing
            return True
        elif (self.items_to_stow == 0):
            return True

        return False # Didn't meet any of our end conditions

    def compete(self):
        
        print "Competing"
        print "Going to the tote"
        self.state_machine.move_to_bin_from_config(2)

        # If time is up or we've finished all items, break
        # Otherwise, loop
        while(not self.done()):
            # Go to the bin 
            # Get Perception Data
            print "Getting Perception Data ..."
            # Send perception data to bump function
            pos = (random.uniform(-0.114, 0.013), random.uniform(-0.127, 0.127), random.uniform(-0.15, 0))

            print "Bumping to coordinates", pos
            self.bumper.bumpRight(pos)

            # Pick up item
            # TODO: For demo, close the hand
            print "Picking up item ..."
            self.vac.on()
            self.hand.closeHand()
            # Based on item, pick a shelf to go to
            self.current_item = self.workOrder[0][0]
            # TODO: Add in Craig's bin picker instead of picking random shelf
            (self.target_bin,self.target_shelf) = self.binSelect.chooseBin(self.current_item);
            print "Going to shelf", self.target_shelf
            # Go to that shelf
            self.state_machine.move_to_shelf(self.target_shelf)
            # Figure out if need to bump
            print "Getting perception data to figure out any bumps"

            # Bump to that location
            # TODO: For demo, pick a random coordinate set and bump to there
            print "Bumping to coordinates ..."

            # Drop item
            # TODO: For Demo, open the hand 
            print "Dropping item ..."
            self.hand.openHand()
            self.vac.off()
            # Update JSON file and count
            self.binSelect.addtoBin(self.current_item,self.target_bin)
            self.workOrder[0].remove(self.current_item)
            self.origMap[self.target_bin].append(self.current_item)
            print "Updating JSON File"
            self.items_to_stow = self.items_to_stow - 1
            

            if self.target_shelf >= 1 and self.target_shelf <= 3:
                self.state_machine.move_to_bin_from_config(1)
            if self.target_shelf >= 4 and self.target_shelf <= 6:
                self.state_machine.move_to_bin_from_config(2)
            if self.target_shelf >= 7 and self.target_shelf <= 9:
                self.state_machine.move_to_bin_from_config(3)
            if self.target_shelf >= 10 and self.target_shelf <= 12:
                self.state_machine.move_to_bin_from_config(4)

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
sup.handler.writeOutFile('Cheese.json',sup.origMap,sup.workOrder)
