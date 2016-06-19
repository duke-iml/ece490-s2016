CONST_BIN_NAMES = ['bin_A',
'bin_B',
'bin_C',
'bin_D',
'bin_E',
'bin_F',
'bin_G',
'bin_H',
'bin_I',
'bin_J',
'bin_K',
'bin_L']
CONST_ITEM_NAMES = ["i_am_a_bunny_book",
"laugh_out_loud_joke_book",
"scotch_bubble_mailer",
"scotch_bubble_mailer",
"up_glucose_bottle",
"dasani_water_bottle",
"dasani_water_bottle",
"rawlings_baseball",
"folgers_classic_roast_coffee",
"elmers_washable_no_run_school_glue",
"elmers_washable_no_run_school_glue",
"hanes_tube_socks",
"womens_knit_gloves",
"cherokee_easy_tee_shirt",
"peva_shower_curtain_liner",
"cloud_b_plush_bear",
"barkely_hide_bones",
"kyjen_squeakin_eggs_plush_puppies",
"cool_shot_glue_sticks",
"creativity_chenille_stems",
"creativity_chenille_stems",
"soft_white_lightbulb",
"safety_first_outlet_plugs",
"oral_b_toothbrush_green",
"oral_b_toothbrush_red",
"dr_browns_bottle_brush",
"command_hooks",
"easter_turtle_sippy_cup",
"fiskars_scissors_red",
"scotch_duct_tape",
"scotch_duct_tape",
"woods_extension_cord",
"platinum_pets_dog_bowl",
"fitness_gear_3lb_dumbbell",
"rolodex_jumbo_pencil_cup",
"clorox_utility_brush",
"kleenex_paper_towels",
"expo_dry_erase_board_eraser",
"expo_dry_erase_board_eraser",
"kleenex_tissue_box",
"ticonderoga_12_pencils",
"crayola_24_ct",
"jane_eyre_dvd",
"dove_beauty_bar",
"staples_index_cards",
"staples_index_cards"]

import sys
import time
import copy
sys.path.insert(0, "../")
# from Sensors import scale
import json
import json_parser_stow
from bin_select import binSelector
from Sensors import scale

class stowHandler:
    def __init__(self,filename=None):
        self.counter = 0
        self.parser=json_parser_stow.json_parser_stow()
        self.binSelector=binSelector();
        self.binSelector.initialize(filename)
        self.bin=[None]*24

        self.scale = None
        self.scale=scale.Scale_Measurement()
        self.currentWeight=float(self.scale.readData(10).split(' ')[0])
        print 'Initial Reading', self.currentWeight
        # self.currentWeight = self.currentWeight.split(' ')[0]
        print 'Numeric Reading', self.currentWeight
        if filename is not None:
            (self.binContents, self.toteContents)=self.parser.readInFile(filename)
            print self.toteContents
        # self.weightClass=[["oral_b_toothbrush_green", "oral_b_toothbrush_red","expo_dry_erase_board_eraser","expo_dry_erase_board_eraser","scotch_bubble_mailer","scotch_bubble_mailer"],
        # ["fiskars_scissors_red"],
        # ["cloud_b_plush_bear","womens_knit_gloves"],
        # ["safety_first_outlet_plugs","platinum_pets_dog_bowl"],
        # ["kyjen_squeakin_eggs_plush_puppies"],
        # ["cherokee_easy_tee_shirt"],
        # ["cool_shot_glue_sticks"],
        # ["dr_browns_bottle_brush","soft_white_lightbulb"],
        # ["ticonderoga_12_pencils","barkely_hide_bones","laugh_out_loud_joke_book","command_hooks","jane_eyre_dvd"],
        # ["rolodex_jumbo_pencil_cup","creativity_chenille_stems","creativity_chenille_stems"],
        # ["i_am_a_bunny_book"],
        # ["dove_beauty_bar"],
        # ["staples_index_cards","staples_index_cards"],
        # ["crayola_24_ct"],
        # ["easter_turtle_sippy_cup","woods_extension_cord"],
        # ["rawlings_baseball","clorox_utility_brush","elmers_washable_no_run_school_glue","elmers_washable_no_run_school_glue","scotch_duct_tape","scotch_duct_tape"],
        # ["kleenex_tissue_box"],
        # ["peva_shower_curtain_liner"],
        # ["up_glucose_bottle"],
        # ["kleenex_paper_towels"],
        # ["folgers_classic_roast_coffee"],
        # ["hanes_tube_socks"],
        # ["dasani_water_bottle","dasani_water_bottle"],
        # ["fitness_gear_3lb_dumbbell"]]

        self.weightClass=[["expo_dry_erase_board_eraser","oral_b_toothbrush_green", "oral_b_toothbrush_red","scotch_bubble_mailer"],
        ["fiskars_scissors_red"],
        ["cloud_b_plush_bear","womens_knit_gloves"],
        ["safety_first_outlet_plugs","platinum_pets_dog_bowl"],
        ["kyjen_squeakin_eggs_plush_puppies"],
        ["cherokee_easy_tee_shirt"],
        ["cool_shot_glue_sticks"],
        ["dr_browns_bottle_brush","soft_white_lightbulb"],
        ["barkely_hide_bones","command_hooks","jane_eyre_dvd","ticonderoga_12_pencils","laugh_out_loud_joke_book"],
        ["rolodex_jumbo_pencil_cup","creativity_chenille_stems"],
        ["i_am_a_bunny_book"],
        ["dove_beauty_bar"],
        ["staples_index_cards"],
        ["crayola_24_ct"],
        ["easter_turtle_sippy_cup","woods_extension_cord"],
        ["clorox_utility_brush","rawlings_baseball","elmers_washable_no_run_school_glue","scotch_duct_tape"],
        ["kleenex_tissue_box"],
        ["peva_shower_curtain_liner"],
        ["up_glucose_bottle"],
        ["kleenex_paper_towels"],
        ["folgers_classic_roast_coffee"],
        ["hanes_tube_socks"],
        ["dasani_water_bottle"],
        ["fitness_gear_3lb_dumbbell"]]
        self.overlap = [filter(lambda x: x in sublist, self.toteContents) for sublist in self.weightClass]
        print self.overlap

    def mostEmptyBin(self):
        numObj=[len(self.binMap[CONST_BIN_NAMES[i]]) for i in range(len(CONST_BIN_NAMES))]
        sortedNumObj=sorted(range(len(numObj)), key=lambda k: numObj[k])
        return CONST_BIN_NAMES[sortedNumObj[0]]

    def pickWhichObj(self, limb, debug=False):

        newWeight = float(self.scale.readData(10).split(' ')[0])

        objWeight=abs((self.currentWeight-newWeight))
        
        if (debug):
            print "Object Weight is ",objWeight
            print "Because old weight was ", self.currentWeight, " & new weight is ", newWeight

        self.currentWeight=newWeight
        item = []
        classidx=0

        if objWeight<10:
            return [None,None,False]
        elif objWeight<23:
            classidx=0
            item = self.overlap[0]
        elif objWeight<28:
            classidx=1
            item = self.overlap[1]
        elif objWeight<37:
            classidx=2
            item = self.overlap[2]
        elif objWeight<47:
            classidx=3
            item = self.overlap[3]
        elif objWeight<55:
            classidx=4
            item = self.overlap[4]
        elif objWeight<61:
            classidx=5
            item = self.overlap[5]
        elif objWeight<66:
            classidx=6
            item = self.overlap[6]
        elif objWeight<76:
            classidx=7
            item = self.overlap[7]
        elif objWeight<90:
            classidx=8
            item = self.overlap[8]
        elif objWeight<105:
            classidx=9
            item = self.overlap[9]
        elif objWeight<116:
            classidx=10
            item = self.overlap[10]
        elif objWeight<122:
            classidx=11
            item = self.overlap[11]
        elif objWeight<128:
            classidx=12
            item = self.overlap[12]
        elif objWeight<132:
            classidx=13
            item = self.overlap[13]
        elif objWeight<137:
            classidx=14
            item = self.overlap[14]
        elif objWeight<156:
            classidx=15
            item = self.overlap[15]
        elif objWeight<211:
            classidx=16
            item = self.overlap[16]
        elif objWeight<262:
            classidx=17
            item = self.overlap[17]
        elif objWeight<296:
            classidx=18
            item = self.overlap[18]
        elif objWeight<354:
            classidx=19
            item = self.overlap[19]
        elif objWeight<402:
            classidx=20
            item = self.overlap[20]
        elif objWeight<514:
            classidx=21
            item = self.overlap[21]
        elif objWeight<998:
            classidx=22
            item = self.overlap[22]
        else:
            classidx=23
            item = self.overlap[23]
        self.counter = self.counter+1

        tilt=(classidx==8)
        if len(item)==0:
            return None, None, False
        copieditem=copy.copy(item)
        print "Possible items are:"+str(copieditem)

        if tilt:
            self.updateTote(item[0])
            self.overlap[classidx].remove(item[0])
            return copieditem[0], 'bin_B', tilt
        if(len(item)>1):
            if self.bin[classidx]==None:
                (self.bin[classidx],binidx) = self.binSelector.chooseCenterBin(item[0])
        else:
            if self.bin[classidx]==None:
                if classidx==5:
                    self.bin[classidx]='bin_K'
                    binidx=11
                else:
                    (self.bin[classidx],binidx) = self.binSelector.chooseBin(item[0],limb)
        
        self.updateTote(item[0])
        self.overlap[classidx].remove(item[0])
        return copieditem[0], self.bin[classidx], tilt

    def updateWeight(self):
        self.currentWeight=float(self.scale.readData(10).split(' ')[0])
        return self.currentWeight

    def getToteContents(self):
        return self.toteContents

    def updateBin(self, bin, item):
        self.binSelector.addtoBin(item,bin)
        self.binContents[bin].append(item)
        return True

    def updateTote(self,objRemoved):
        print "Object removed:"+objRemoved
        if objRemoved in self.toteContents: 
            self.toteContents.remove(objRemoved)
            return True
        else:
            return False

    def jsonOutput(self, filename):
        self.parser.writeOutFile(filename, self.binContents, self.toteContents)

# if __name__ == "__main__":
#     JSON_FILES=["StowTestA.json","StowTestB.json","StowTestC.json","StowTestD.json","StowTestE.json"]
#     # for i in range(len(JSON_FILES)):
#     a=stowHandler("../JSON_FILES/"+JSON_FILES[0])
#     print a.mostEmptyBin()
    # for i in range(50):
    #     # time.sleep(5)
    #     (targetitem,targetbin)=a.pickWhichObj('left',True);
    #     print "TI"
    #     print targetitem
    #     if targetitem!=None:
    #         a.updateBin(targetbin,targetitem)
    #     a.jsonOutput("../JSON_FILES/StowTestBOut.json")
