#The classthat selects which bin to put items in
# Inputs:
# Returns 

import time
import json

class binSelect:
    bin_dict={}

    def __init__(self,init_bin_dict):
        bin_dict=init_bin_dict


    def chooseBin(self):
        #This method selects a bin and returns that to the caller.
        placeholder=1
