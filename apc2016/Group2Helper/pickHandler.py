import json
import json_parser_pick
import sys
sys.path.insert(0, "..")
from Sensors import multiScale2 as scale
import cPickle as pickle
import time, copy

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
"platinum_pets_dog_bowl",
"woods_extension_cord",
"dr_browns_bottle_brush",
"rawlings_baseball",
"kyjen_squeakin_eggs_plush_puppies"],
["up_glucose_bottle",
"peva_shower_curtain_liner",
"folgers_classic_roast_coffee",
"clorox_utility_brush",
"easter_turtle_sippy_cup"],
["dasani_water_bottle",
"scotch_duct_tape",
"oral_b_toothbrush_green",
"oral_b_toothbrush_red",
"kleenex_paper_towels",
"hanes_tube_socks",
"fiskars_scissors_red",
"rolodex_jumbo_pencil_cup",
"fitness_gear_3lb_dumbbell"]]

THRESHOLD = 20
# lightest object is 22 g

class pickHandler:
    def __init__(self,filename=None,real_scale=1):
        self.real_scale = real_scale
        self.filename=filename
        self.parser=json_parser_pick.json_parser_pick()
        (self.binMap,self.toteContents, self.workOrder)=self.parser.readInFile(filename)

        # print "\n\n*****", self.workOrder, "*******\n\n"

        self.targetOrder=[]
        self.easyTargetOrder=[]
        self.medTargetOrder=[]
        self.hardTargetOrder=[]
        self.easyBinInd=[]
        self.medBinInd=[]
        self.hardBinInd=[]
        self.easymedBinInd=[]
        self.easymedTargetOrder=[]


        self.weightClass=[["scotch_bubble_mailer", "expo_dry_erase_board_eraser","oral_b_toothbrush_green", "oral_b_toothbrush_red"],
        ["fiskars_scissors_red"],
        ["womens_knit_gloves", "cloud_b_plush_bear"],
        ["safety_first_outlet_plugs","platinum_pets_dog_bowl"],
        ["kyjen_squeakin_eggs_plush_puppies"],
        ["cherokee_easy_tee_shirt"],
        ["cool_shot_glue_sticks"],
        ["dr_browns_bottle_brush","soft_white_lightbulb"],
        ["jane_eyre_dvd","ticonderoga_12_pencils","laugh_out_loud_joke_book","barkely_hide_bones","command_hooks"],
        ["creativity_chenille_stems", "rolodex_jumbo_pencil_cup"],
        ["i_am_a_bunny_book"],
        ["dove_beauty_bar"],
        ["staples_index_cards"],
        ["crayola_24_ct"],
        ["easter_turtle_sippy_cup","woods_extension_cord"],
        ["elmers_washable_no_run_school_glue","clorox_utility_brush","rawlings_baseball","scotch_duct_tape"],
        ["kleenex_tissue_box"],
        ["peva_shower_curtain_liner"],
        ["up_glucose_bottle"],
        ["kleenex_paper_towels"],
        ["folgers_classic_roast_coffee"],
        ["hanes_tube_socks"],
        ["dasani_water_bottle"],
        ["fitness_gear_3lb_dumbbell"]]

        self.scale = None
        # self.scale=scale.Scale_Measurement()
        self.currentWeight=self.readScale()

        # for i in range(len(CONST_BIN_NAMES)):
        #     self.targetOrder.append(self.workOrder[i]["item"])
        #     if(CONST_ITEM_NAMES[0].count(self.workOrder[i]["item"])+CONST_ITEM_NAMES[1].count(self.workOrder[i]["item"])>0):
        #         self.easymedTargetOrder.append(self.workOrder[i]["item"])
        #         self.easymedBinInd.append(i)
        #     else:
        #         self.hardTargetOrder.append(self.workOrder[i]["item"])
        #         self.hardBinInd.append(i)

        for i in range(len(CONST_BIN_NAMES)):
            self.targetOrder.append(self.workOrder[i]["item"])
            if(CONST_ITEM_NAMES[0].count(self.workOrder[i]["item"])>0):
                self.easyTargetOrder.append(self.workOrder[i]["item"])
                self.easyBinInd.append(i)
            elif(CONST_ITEM_NAMES[1].count(self.workOrder[i]["item"])>0):
                self.medTargetOrder.append(self.workOrder[i]["item"])
                self.medBinInd.append(i)
            else:
                self.hardTargetOrder.append(self.workOrder[i]["item"])
                self.hardBinInd.append(i)

    def mostEmptyBin(self):
        # bins=copy.copy(CONST_BIN_NAMES)
        bins=[bin for bin in CONST_BIN_NAMES]
        bins.remove('bin_J')
        bins.remove('bin_A')
        bins.remove('bin_D')
        bins.remove('bin_G')
        numObj=[len(self.binMap[bins[i]]) for i in range(len(bins))]
        sortedNumObj=sorted(range(len(numObj)), key=lambda k: numObj[k])
        return bins[sortedNumObj[0]]

    def gotRightObject(self, itemDropAttempt, bin, debug = True):

        #itemDrop attempt is the item I attempt to drop

        newWeight = self.readScale()

        objWeight=abs((self.currentWeight-newWeight))

        if (debug):
            print "Object Weight is ",objWeight
            print "Because old weight was ", self.currentWeight, " & new weight is ", newWeight

        self.currentWeight=newWeight
        item = []
        classidx=0

        if objWeight<10:
            return [False,None]
        elif objWeight<23:
            classidx=0
            item = self.weightClass[0]
        elif objWeight<28:
            classidx=1
            item = self.weightClass[1]
        elif objWeight<37:
            classidx=2
            item = self.weightClass[2]
        elif objWeight<47:
            classidx=3
            item = self.weightClass[3]
        elif objWeight<55:
            classidx=4
            item = self.weightClass[4]
        elif objWeight<61:
            classidx=5
            item = self.weightClass[5]
        elif objWeight<66:
            classidx=6
            item = self.weightClass[6]
        elif objWeight<77:
            classidx=7
            item = self.weightClass[7]
        elif objWeight<90:
            classidx=8
            item = self.weightClass[8]
        elif objWeight<105:
            classidx=9
            item = self.weightClass[9]
        elif objWeight<116:
            classidx=10
            item = self.weightClass[10]
        elif objWeight<122:
            classidx=11
            item = self.weightClass[11]
        elif objWeight<128:
            classidx=12
            item = self.weightClass[12]
        elif objWeight<134:
            classidx=13
            item = self.weightClass[13]
        elif objWeight<137:
            classidx=14
            item = self.weightClass[14]
        elif objWeight<156:
            classidx=15
            item = self.weightClass[15]
        elif objWeight<211:
            classidx=16
            item = self.weightClass[16]
        elif objWeight<262:
            classidx=17
            item = self.weightClass[17]
        elif objWeight<296:
            classidx=18
            item = self.weightClass[18]
        elif objWeight<354:
            classidx=19
            item = self.weightClass[19]
        elif objWeight<402:
            classidx=20
            item = self.weightClass[20]
        elif objWeight<514:
            classidx=21
            item = self.weightClass[21]
        elif objWeight<998:
            classidx=22
            item = self.weightClass[22]
        else:
            classidx=23
            item = self.weightClass[23]

        print 'attempted to drop', itemDropAttempt

        if itemDropAttempt in item:
            return True, itemDropAttempt
        else:
            for bin_item in self.binMap[bin]:
                if bin_item in item:
                    return False, bin_item
            else:
                return False, None

    def workBinOrder(self):
        # filenames=['pickA.json','pickB.json','pickC.json','pickD.json','pickE.json']
        # for l in range(5):
        BinOrder=CONST_BIN_NAMES
        # filename=filenames[l]
        numObject=[len(self.binMap[BinOrder[i]]) for i in range(len(BinOrder))]
        easyNumObject=[numObject[self.easyBinInd[i]] for i in range(len(self.easyBinInd))]
        medNumObject=[numObject[self.medBinInd[i]] for i in range(len(self.medBinInd))]
        hardNumObject=[numObject[self.hardBinInd[i]] for i in range(len(self.hardBinInd))]
        # easymedNumObject=[numObject[self.easymedBinInd[i]] for i in range(len(self.easymedBinInd))]
        # easymedFirstGroup=[easymedNumObject[i]<5 for i in range(len(easymedNumObject))]
        print easyNumObject
        print medNumObject
        print hardNumObject
        sortedEasyNumInd=sorted(range(len(easyNumObject)), key=lambda k: easyNumObject[k])
        sortedMedNumInd=sorted(range(len(medNumObject)), key=lambda k: medNumObject[k])
        sortedHardNumInd=sorted(range(len(hardNumObject)), key=lambda k: hardNumObject[k])
        # sortedEasymedNumInd=sorted(range(len(easymedNumObject)), key=lambda k: easymedNumObject[k])
        sortedEasyBinInd=[self.easyBinInd[sortedEasyNumInd[i]] for i in range(len(sortedEasyNumInd))]
        sortedMedBinInd=[self.medBinInd[sortedMedNumInd[i]] for i in range(len(sortedMedNumInd))]
        sortedHardBinInd=[self.hardBinInd[sortedHardNumInd[i]] for i in range(len(sortedHardNumInd))]
        # sortedEasymedBinInd=[self.easymedBinInd[sortedEasymedNumInd[i]] for i in range(len(sortedEasymedNumInd))]
        sortedBinInd=sortedEasyBinInd+sortedMedBinInd+sortedHardBinInd
        # sortedBinInd=sortedEasymedBinInd+sortedHardBinInd
        print sortedBinInd
        sortedBinOrder=[BinOrder[sortedBinInd[i]] for i in range(len(sortedBinInd))]
        sortedEasyTargetOrder=[self.easyTargetOrder[sortedEasyNumInd[i]] for i in range(len(sortedEasyNumInd))]
        sortedMedTargetOrder=[self.medTargetOrder[sortedMedNumInd[i]] for i in range(len(sortedMedNumInd))]
        sortedHardTargetOrder=[self.hardTargetOrder[sortedHardNumInd[i]] for i in range(len(sortedHardNumInd))]
        # sortedEasymedTargetOrder=[self.easymedTargetOrder[sortedEasymedNumInd[i]] for i in range(len(sortedEasymedNumInd))]
        sortedTargetOrder=sortedEasyTargetOrder+sortedMedTargetOrder+sortedHardTargetOrder
        # sortedTargetOrder=sortedEasymedTargetOrder+sortedHardTargetOrder
        sortedRight=[sortedBinInd[i]%3!=0 for i in range(len(sortedBinInd))]
        binEasiness=[0]*len(sortedEasyTargetOrder)+[2]*len(sortedMedTargetOrder)+[4]*len(sortedHardTargetOrder)
        # binEasiness=[0]*easymedFirstGroup.count(True)+[2]*easymedFirstGroup.count(False)+[4]*len(sortedHardTargetOrder)
        return (sortedBinOrder, sortedTargetOrder, sortedRight, binEasiness)

    def getToteContents(self):
        return self.toteContents

    def updateWeight(self):
        self.currentWeight=self.readScale()
        return self.currentWeight

    def testIncrease(self):
        if self.readScale() > self.currentWeight + THRESHOLD:
            return True
        else:
            return False

    def testDecrease(self):
        if self.readScale() < self.currentWeight - THRESHOLD:
            return True
        else:
            return False

    def updateBin(self, bin, item):
        binidx=CONST_BIN_NAMES.index(bin)
        # print binidx
        self.binMap[bin].remove(item)
        # self.workOrder.remove(self.workOrder[binidx])
        self.workOrder[binidx] = []

        return True

    def updateTote(self,objAdded):
        print "\nPickHandler: Object added:"+objAdded
        self.toteContents.append(objAdded)
        return True

    def jsonOutput(self, filename):
        self.parser.writeOutFile(filename, self.binMap, self.toteContents, self.workOrder)

    def readScale(self):
        if not self.real_scale:
            return 0
        # total = 0
        # data = self.scale.readData(10)
        # for entry in data:
        #     total = total + float(entry.split(' ')[0])
        totalWeight = None
        while totalWeight == None:
            try:
                with open("../Sensors/weight.pkl", "rb") as f:
                    weight = 0
                    valList = pickle.load(f)
                    for i in range(len(valList)):
                        weight = weight + float(valList[i])
                # print weight
            except EOFError:
                print "error reading"
                pass
            time.sleep(0.5)
            if weight != 0:
                totalWeight = weight
            else:
                print "total weight is 0. Check your system"
        return totalWeight

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
if __name__ == "__main__":
    ph = pickHandler("../JSON_FILES/apc_pick_task.json")
    # while True:
    #     try:
    #         print ph.readScale()
    #     except EOFError:
    #         pass
    #     time.sleep(1)

    while True:
        print ph.readScale()
