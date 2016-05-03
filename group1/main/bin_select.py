#The class that selects which bin to put items in
#  Description: Mantains a python dictionary which notes where every item is stored and quick statistics about the bins

import time
import json

class binSelector:
    
    
    def __init__(self):
        #Create empty bin objects
        self.bin_dict={};
        #Keys are the number of the bin and the value is a list
        #The list contains a list of items, current number of items,points score, total volume, percent free, strikes
        self.bin_dict['1']=[[],0,0,0,100,0];
        self.bin_dict['2']=[[],0,0,0,100,0];
        self.bin_dict['3']=[[],0,0,0,100,0];
        self.bin_dict['4']=[[],0,0,0,100,0];
        self.bin_dict['5']=[[],0,0,0,100,0]; 
        self.bin_dict['6']=[[],0,0,0,100,0];
        self.bin_dict['7']=[[],0,0,0,100,0]; 
        self.bin_dict['8']=[[],0,0,0,100,0];
        self.bin_dict['9']=[[],0,0,0,100,0]; 
        self.bin_dict['10']=[[],0,0,0,100,0];
        self.bin_dict['11']=[[],0,0,0,100,0]; 
        self.bin_dict['12']=[[],0,0,0,100,0];
        self.maxbinvolume=1485 #total volume constant for each bin 
        
    def addtoBin(self,itemid,itembin):
        itemvol=self.item_lookup[itemid];
        binlist=self.bin_dict[str(itembin)]; #convert to temp local var 
        binlist[0].append(itemid); #add itemid to contents list
        binlist[1]=binlist[1]+1; #add item to number of items 
        binlist[3]=binlist[3]+itemvol; #add volume to total volume
        binlist[4]=100-binlist[3]*100/self.maxbinvolume; #recalculate percentage free
        self.bin_dict[str(itembin)]=binlist;
        
    def chooseBin(self,itemvol):
        #This method selects a bin and returns that to the caller.
        validlist=[];
        for key in self.bin_dict.keys():
            prediction=self.addPredict(key,itemvol);
            if prediction[3]>20: #If there is less than 10% free space in the bin don't add it to the list to be considered 
                validlist.append(prediction);
        validlist.sort(cmp=self.comparator);
        return validlist[0][0]

    
    def comparator(self, a,b):
        #Comparator for the bin sort module
        if b[4]-a[4]!=0: #Compare strikes
            return b[4]-a[4];
        if b[2]-a[2]!=0: #Compare score
            return b[2]-a[2];
        if b[3]-a[3]!=0: #Compare freedom
            return b[3]-a[3];
        if b[1]-a[1]!=0: #compare number of elements 
            return a[1]-b[1];
        return 0
        
    def addPredict(self,itembin,itemvol):
        #Predicts the free space in a bin after a particular item is added to a specified bin. Returns total volume and percentage free 
        estnumel=self.bin_dict[itembin][1]+1;#number of elements after addition
        points=self.bin_dict[itembin][2];#points level (higher is better)
        estvol=self.bin_dict[itembin][3]+itemvol;
        estpf=100-estvol*100/self.maxbinvolume;
        strikes=self.bin_dict[itembin][5]; #gets strikes
        return itembin, estnumel, points, estpf,strikes

    def start(self):
        #copy the value of Calculate score of each bin and write it to position 2
        for key in self.bin_dict.keys():
            orgnum=self.bin_dict[key][1];
            if orgnum>=5:
                self.bin_dict[key][2]=20;
                continue;
            elif orgnum>=3 and orgnum<5:
                self.bin_dict[key][2]=15;
                continue;
            elif orgnum>=1 and orgnum<3:
                self.bin_dict[key][2]=10;
                continue;
            self.bin_dict[key][2]=0
        print("Bin Selector has been intialized")
            
    def printBin(self):
        for key in self.bin_dict.keys():
            binlist=self.bin_dict[key];
            print "bin ",key," contains ",binlist[0]," w/ ",binlist[1]," items ",binlist[4],"% free for ",binlist[2],"points with ",binlist[5]," strikes";
            
    def test(self):
        self.addtoBin("folgers_classic_roast_coffee",1)
        self.addtoBin("elmers_glue_all",1)
        self.addtoBin("hanes_tube_socks",1)
        self.addtoBin("womens_knit_gloves",1)
        self.addtoBin(5,1)
        self.addtoBin(6,1)
        self.addtoBin(7,2)
        self.addtoBin(8,2)
        self.addtoBin(9,4)
        self.addtoBin(10,5)
        self.addtoBin(11,5)
        self.addtoBin(12,5)
        self.addtoBin(13,5)
        self.addtoBin(14,5)
        self.addtoBin(15,7)
        self.addtoBin(16,7)
        self.addtoBin(17,7)
        self.addtoBin(18,7)
        self.addtoBin(19,7)
        self.addtoBin(20,11)

    def itemlookup(self):
        self.item_lookup={};
        self.item_lookup["i_am_a_bunny_hardcover_book"]=200;
        self.item_lookup["laugh_out_loud_jokes_for_kids_paperback_book"]=200;
        self.item_lookup["scotch_bubble_mailer"]=200;
        self.item_lookup["up_and_up_glucose_tablets"]=200;
        self.item_lookup["dasani_water_bottle"]=200;
        self.item_lookup["rawlings_recreational_use_baseball"]=200;
        self.item_lookup["folgers_classic_roast_coffee"]=200;
        self.item_lookup["elmers_glue_all"]=200;
        self.item_lookup["hanes_tube_socks"]=200;
        self.item_lookup["womens_knit_gloves"]=200;
        self.item_lookup["cherokee_easy_tee_t_shirt"]=200;
        self.item_lookup["peva_shower_curtain_liner"]=200;
        self.item_lookup["cloud_b_plush_bear"]=200;
        self.item_lookup["boots_and_barkley_beefhide_bones"]=200;
        self.item_lookup["plush_puppies_squeakin_eggs"]=200;
        self.item_lookup["cool_shot_mini_glue_sticks"]=200;
        self.item_lookup["creativity_street_chenille_stems_100_count"]=200;
        self.item_lookup["40w_soft_white_lightbulb"]=200;
        self.item_lookup["safety_1st_ultra_clear_outlet_plugs"]=200;
        self.item_lookup["oral_b_cavity_defense_toothbrush"]=200;
        self.item_lookup["dr_browns_natural_flow_bottle_brush"]=200;
        self.item_lookup["command_hooks"]=200;
        self.item_lookup["easter_turtle_figural_sippy_cup"]=200;
        self.item_lookup["fiskars_scissors_5inch_blunt_tip_red"]=200;
        self.item_lookup["scotch_cloth_duct_tape"]=200;
        self.item_lookup["woods_6ft_extension_cord"]=200;
        self.item_lookup["platinum_pets_dog_bowl"]=200;
        self.item_lookup["fitness_gear_3lb_neoprene_dumbbell"]=200;
        self.item_lookup["rolodex_wire_mesh_jumbo_pencil_holder"]=200;
        self.item_lookup["clorox_utility_brush"]=200;
        self.item_lookup["kleenex_viva_paper_towels"]=200;
        self.item_lookup["expo_white_board_care_eraser"]=200;
        self.item_lookup["kleenex_cool_touch_tissue_box"]=200;
        self.item_lookup["ticonderoga_#2_pencils_12_count"]=200;
        self.item_lookup["crayola_crayons"]=200;
        self.item_lookup["jane_eyre_dvd"]=200;
        self.item_lookup["dove_dry_oil_beauty_bar"]=200;
        self.item_lookup["staples_3x5_ruled_index_cards"]=200;
        
