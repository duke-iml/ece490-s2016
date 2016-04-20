#The classthat selects which bin to put items in
# Inputs:
# Returns 

import time
import json

class binSelect:
    bin_dict={}
    #Keys are the number of the bin and the value is a list
    #The list contains a list of items, number of items, and total volume 
    maxbinvolume=20 #total volume constant for each bin 
    def __init__(self):
        #Create empty bin objects
        bin_dict[1]=[[],0,0];
        bin_dict[2]=[[],0,0];
        bin_dict[3]=[[],0,0];
        bin_dict[4]=[[],0,0];
        bin_dict[5]=[[],0,0];
        bin_dict[6]=[[],0,0];
        bin_dict[7]=[[],0,0];
        bin_dict[8]=[[],0,0];
        bin_dict[9]=[[],0,0];
        bin_dict[10]=[[],0,0];
        bin_dict[11]=[[],0,0];
        bin_dict[12]=[[],0,0];
        
    def addtoBin(self,itemid,itembin,itemvol):
        binlist=bin_dict[itembin]; #convert to temp local var 
        binlist[0].append(itemid); #add itemid to contents list
        binlist[1]=binlist[1]+1; #add item to number of items 
        binlist[2]=binlist[2]+itemvol; #add volume to total volume
        binlist[3]=binlist[2]/maxbinvolume; #recalculate percentage full
        
    def chooseBin(self):
        #This method selects a bin and returns that to the caller.
        placeholder=1
