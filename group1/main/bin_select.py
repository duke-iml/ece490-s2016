#The class that selects which bin to put items in
#  Description: Mantains a python dictionary which notes where every item is stored and quick statistics about the bins

import time
import json

class binSelector:
    
    
    def __init__(self):
        #Create empty bin objects
        self.bin_dict={};
        #Keys are the number of the bin and the value is a list
        #The list contains a list of items, current number of items,original number of items, total volume, and percent free
        self.bin_dict[1]=[[],0,0,0,100];
        self.bin_dict[2]=[[],0,0,0,100];
        self.bin_dict[3]=[[],0,0,0,100];
        self.bin_dict[4]=[[],0,0,0,100];
        self.bin_dict[5]=[[],0,0,0,100]; 
        self.bin_dict[6]=[[],0,0,0,100];
        self.bin_dict[7]=[[],0,0,0,100]; 
        self.bin_dict[8]=[[],0,0,0,100];
        self.bin_dict[9]=[[],0,0,0,100]; 
        self.bin_dict[10]=[[],0,0,0,100];
        self.bin_dict[11]=[[],0,0,0,100]; 
        self.bin_dict[12]=[[],0,0,0,100];
        self.maxbinvolume=20 #total volume constant for each bin 
        
    def addtoBin(self,itemid,itembin,itemvol):
        binlist=self.bin_dict[itembin]; #convert to temp local var 
        binlist[0].append(itemid); #add itemid to contents list
        binlist[1]=binlist[1]+1; #add item to number of items 
        binlist[2]=binlist[2]+itemvol; #add volume to total volume
        binlist[3]=100-binlist[2]*100/self.maxbinvolume; #recalculate percentage free
        self.bin_dict[itembin]=binlist;
        
    def chooseBin(self,itemvol):
        #This method selects a bin and returns that to the caller.
        placeholder=1
        
    def printBin(self):
        for key in self.bin_dict.keys():
            binlist=self.bin_dict[key];
            print "bin ",key," contains ",binlist[0]," with ",binlist[1]," items taking up ",binlist[2]," units or ",binlist[3],"% free";
            
