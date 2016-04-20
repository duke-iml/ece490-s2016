#The classthat selects which bin to put items in
# Inputs:
# Returns 

import time
import json

class binSelect:
    bin_dict={}
    #Keys are the number of the bin and the value is a list
    #The list contains a list of items, number of items, and total volume 

    def __init__(self):
        bin_dict[bin1]=[[],0,0];
        bin_dict[bin2]=[[],0,0];
        bin_dict[bin3]=[[],0,0];
        bin_dict[bin4]=[[],0,0];
        bin_dict[bin5]=[[],0,0];
        bin_dict[bin6]=[[],0,0];
        bin_dict[bin7]=[[],0,0];
        bin_dict[bin8]=[[],0,0];
        bin_dict[bin9]=[[],0,0];
        bin_dict[bin10]=[[],0,0];
        bin_dict[bin11]=[[],0,0];
        bin_dict[bin12]=[[],0,0];
        
    def addtoBin(self,targetitem,targetbin):
        
    def chooseBin(self):
        #This method selects a bin and returns that to the caller.
        placeholder=1
