import json
import json_parser_pick
# from bin_select_pick import binSelector
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
CONST_ITEM_NAMES = [
["laugh_out_loud_joke_book",
"jane_eyre_dvd",
"cherokee_easy_tee_shirt",
"womens_knit_gloves",
"i_am_a_bunny_book",
"scotch_bubble_mailer",
"soft_white_lightbulb",
"barkely_hide_bones",
"staples_index_cards",
"dove_beauty_bar",
"expo_dry_erase_board_eraser",
"ticonderoga_12_pencils",
"crayola_24_ct",
"elmers_washable_no_run_school_glue",
"creativity_chenille_stems",
"cloud_b_plush_bear",
"kleenex_tissue_box",
"command_hooks",
"safety_first_outlet_plugs",
"cool_shot_glue_sticks",
"platinum_pets_dog_bowl"],
["woods_extension_cord",
"dr_browns_bottle_brush",
"rawlings_baseball",
"kyjen_squeakin_eggs_plush_puppies",
"up_glucose_bottle",
"peva_shower_curtain_liner",
"folgers_classic_roast_coffee",
"clorox_utility_brush",
"scotch_duct_tape",
"oral_b_toothbrush_green",
"oral_b_toothbrush_red",
"easter_turtle_sippy_cup"],
["dasani_water_bottle",
"kleenex_paper_towels",
"hanes_tube_socks",
"fiskars_scissors_red",
"rolodex_jumbo_pencil_cup",
"fitness_gear_3lb_dumbbell"]]
class pickHandler:
    def __init__(self,filename=None):
        self.filename=filename
        self.parser=json_parser_pick.json_parser_pick()
        (self.binMap,self.toteContents, self.workOrder)=self.parser.readInFile(filename)
        self.targetOrder=[]
        self.easyTargetOrder=[]
        self.medTargetOrder=[]
        self.hardTargetOrder=[]
        self.easyBinInd=[]
        self.medBinInd=[]
        self.hardBinInd=[]
        self.easymedBinInd=[]
        self.easymedTargetOrder=[]

        for i in range(len(CONST_BIN_NAMES)):
            self.targetOrder.append(self.workOrder[i]["item"])
            if(CONST_ITEM_NAMES[0].count(self.workOrder[i]["item"])+CONST_ITEM_NAMES[1].count(self.workOrder[i]["item"])>0):
                self.easymedTargetOrder.append(self.workOrder[i]["item"])
                self.easymedBinInd.append(i)
            else:
                self.hardTargetOrder.append(self.workOrder[i]["item"])
                self.hardBinInd.append(i)

        # for i in range(len(CONST_BIN_NAMES)):
        #     self.targetOrder.append(self.workOrder[i]["item"])
        #     if(CONST_ITEM_NAMES[0].count(self.workOrder[i]["item"])>0):
        #         self.easyTargetOrder.append(self.workOrder[i]["item"])
        #         self.easyBinInd.append(i)
        #     elif(CONST_ITEM_NAMES[1].count(self.workOrder[i]["item"])>0):
        #         self.medTargetOrder.append(self.workOrder[i]["item"])
        #         self.medBinInd.append(i)
        #     else:
        #         self.hardTargetOrder.append(self.workOrder[i]["item"])
        #         self.hardBinInd.append(i)
        # self.binSelector=binSelector()
        # self.binSelector.initialize(filename)
    def mostEmptyBin(self):
        numObj=[len(self.binMap[CONST_BIN_NAMES[i]]) for i in range(len(CONST_BIN_NAMES))]
        sortedNumObj=sorted(range(len(numObj)), key=lambda k: numObj[k])
        return CONST_BIN_NAMES[sortedNumObj[0]]

    def workBinOrder(self):
        # filenames=['pickA.json','pickB.json','pickC.json','pickD.json','pickE.json']
        # for l in range(5):
        BinOrder=CONST_BIN_NAMES
        # filename=filenames[l]
        numObject=[len(self.binMap[BinOrder[i]]) for i in range(len(BinOrder))]
        # easyNumObject=[numObject[self.easyBinInd[i]] for i in range(len(self.easyBinInd))]
        # medNumObject=[numObject[self.medBinInd[i]] for i in range(len(self.medBinInd))]
        hardNumObject=[numObject[self.hardBinInd[i]] for i in range(len(self.hardBinInd))]
        easymedNumObject=[numObject[self.easymedBinInd[i]] for i in range(len(self.easymedBinInd))]
        easymedFirstGroup=[easymedNumObject[i]<5 for i in range(len(easymedNumObject))]
        # print easyNumObject
        # print medNumObject
        print hardNumObject
        # sortedEasyNumInd=sorted(range(len(easyNumObject)), key=lambda k: easyNumObject[k])
        # sortedMedNumInd=sorted(range(len(medNumObject)), key=lambda k: medNumObject[k])
        sortedHardNumInd=sorted(range(len(hardNumObject)), key=lambda k: hardNumObject[k])
        sortedEasymedNumInd=sorted(range(len(easymedNumObject)), key=lambda k: easymedNumObject[k])
        # sortedEasyBinInd=[self.easyBinInd[sortedEasyNumInd[i]] for i in range(len(sortedEasyNumInd))]
        # sortedMedBinInd=[self.medBinInd[sortedMedNumInd[i]] for i in range(len(sortedMedNumInd))]
        sortedHardBinInd=[self.hardBinInd[sortedHardNumInd[i]] for i in range(len(sortedHardNumInd))]
        sortedEasymedBinInd=[self.easymedBinInd[sortedEasymedNumInd[i]] for i in range(len(sortedEasymedNumInd))]
        # sortedBinInd=sortedEasyBinInd+sortedMedBinInd+sortedHardBinInd
        sortedBinInd=sortedEasymedBinInd+sortedHardBinInd
        print sortedBinInd
        sortedBinOrder=[BinOrder[sortedBinInd[i]] for i in range(len(sortedBinInd))]
        # sortedEasyTargetOrder=[self.easyTargetOrder[sortedEasyNumInd[i]] for i in range(len(sortedEasyNumInd))]
        # sortedMedTargetOrder=[self.medTargetOrder[sortedMedNumInd[i]] for i in range(len(sortedMedNumInd))]
        sortedHardTargetOrder=[self.hardTargetOrder[sortedHardNumInd[i]] for i in range(len(sortedHardNumInd))]
        sortedEasymedTargetOrder=[self.easymedTargetOrder[sortedEasymedNumInd[i]] for i in range(len(sortedEasymedNumInd))]
        # sortedTargetOrder=sortedEasyTargetOrder+sortedMedTargetOrder+sortedHardTargetOrder
        sortedTargetOrder=sortedEasymedTargetOrder+sortedHardTargetOrder
        sortedRight=[sortedBinInd[i]%3!=0 for i in range(len(sortedBinInd))]
        binEasiness=[0]*easymedFirstGroup.count(True)+[2]*easymedFirstGroup.count(False)+[4]*len(sortedHardTargetOrder)
        return (sortedBinOrder, sortedTargetOrder, sortedRight, binEasiness)

    def getToteContents(self):
        return self.toteContents

    def updateBin(self, bin, item):
        binidx=CONST_BIN_NAMES.index(bin)
        print binidx
        self.binMap[bin].remove(item)
        self.workOrder.remove(self.workOrder[binidx])
        return True

    def updateTote(self,objAdded):
        print "Object added:"+objAdded
        self.toteContents.append(objAdded)
        return True
        
    def jsonOutput(self, filename):
        self.parser.writeOutFile(filename, self.binMap, self.toteContents, self.workOrder)

# if __name__ == "__main__":
#     JSON_FILES=["PickTestA.json","PickTestB.json","PickTestC.json","PickTestD.json","PickTestE.json"]
#     for i in range(len(JSON_FILES)):
#         a=pickHandler("../JSON_FILES/"+JSON_FILES[i])
#         print a.mostEmptyBin()
#         print a.workBinOrder()
        # a.updateBin(border[0],torder[0])
        # a.updateBin(border[1],torder[1])
        # a.updateBin(border[2],torder[2])
        # a.updateTote(torder[0])
        # a.updateTote(torder[1])
        # a.updateTote(torder[2])
        # a.jsonOutput("../JSON_FILES/Out"+JSON_FILES[i])