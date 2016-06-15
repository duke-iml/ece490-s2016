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
CONST_ITEM_WEIGHTS=[3.2, 2.9, 1.6, 1.6, 11.2, 16.9, 16.9,
6, 11.3, 4, 4, 16, 2,
1, 11.8, 3.2, 4, 3.2,
2.4, 4, 4, 3, 0.3, 0.5, 0.5,
4.2, 1.6, 12, 1.4, 2, 2,
6.4, 1.6, 48, 3.2, 7,
12.8, 0.3, 0.3, 10, 2.1, 8,
5.12, 4, 3, 3] 
class pickHandler:
    def __init__(self,filename=None):
        self.filename=filename
        self.parser=json_parser_pick.json_parser_pick()
        (self.binMap, self.workOrder)=self.parser.readInFile(filename)
        self.targetOrder=[]
        self.easyTargetOrder=[]
        self.medTargetOrder=[]
        self.hardTargetOrder=[]
        self.easyBinInd=[]
        self.medBinInd=[]
        self.hardBinInd=[]

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
        # self.binSelector=binSelector()
        # self.binSelector.initialize(filename)
    def workBinOrder(self):
        # filenames=['pickA.json','pickB.json','pickC.json','pickD.json','pickE.json']
        # for l in range(5):
        BinOrder=CONST_BIN_NAMES
        # filename=filenames[l]
        numObject=[len(self.binMap[BinOrder[i]]) for i in range(len(BinOrder))]
        easyNumObject=[numObject[self.easyBinInd[i]] for i in range(len(self.easyBinInd))]
        medNumObject=[numObject[self.medBinInd[i]] for i in range(len(self.medBinInd))]
        hardNumObject=[numObject[self.hardBinInd[i]] for i in range(len(self.hardBinInd))]
        print easyNumObject
        print medNumObject
        print hardNumObject
        sortedEasyNumInd=sorted(range(len(easyNumObject)), key=lambda k: easyNumObject[k])
        sortedMedNumInd=sorted(range(len(medNumObject)), key=lambda k: medNumObject[k])
        sortedHardNumInd=sorted(range(len(hardNumObject)), key=lambda k: hardNumObject[k])
        sortedEasyBinInd=[self.easyBinInd[sortedEasyNumInd[i]] for i in range(len(sortedEasyNumInd))]
        sortedMedBinInd=[self.medBinInd[sortedMedNumInd[i]] for i in range(len(sortedMedNumInd))]
        sortedHardBinInd=[self.hardBinInd[sortedHardNumInd[i]] for i in range(len(sortedHardNumInd))]
        sortedBinInd=sortedEasyBinInd+sortedMedBinInd+sortedHardBinInd
        print sortedBinInd
        sortedBinOrder=[BinOrder[sortedBinInd[i]] for i in range(len(sortedBinInd))]
        sortedEasyTargetOrder=[self.easyTargetOrder[sortedEasyNumInd[i]] for i in range(len(sortedEasyNumInd))]
        sortedMedTargetOrder=[self.medTargetOrder[sortedMedNumInd[i]] for i in range(len(sortedMedNumInd))]
        sortedHardTargetOrder=[self.hardTargetOrder[sortedHardNumInd[i]] for i in range(len(sortedHardNumInd))]
        sortedTargetOrder=sortedEasyTargetOrder+sortedMedTargetOrder+sortedHardTargetOrder
        sortedRight=[sortedBinInd[i]%3!=0 for i in range(len(sortedBinInd))]
        # objectWeight=dict()
        # binWeightDict=dict()
        # binWeight=[]
        # sortedBinIndex=[]
        # objectOrder=[]
        # threshold=10
        # notTry=0
        # giveup=True
        # for i in range(len(CONST_ITEM_NAMES)):
        #     objectWeight[CONST_ITEM_NAMES[i]]=CONST_ITEM_WEIGHTS[i]
        # print i
        # for i in range(len(CONST_BIN_NAMES)):
        #     print i
        #     target=workOrder[i]["item"]
        #     objectOrder=objectOrder+[target]
        #     numObject=numObject+[len(binMap[CONST_BIN_NAMES[i]])]
        #     if i==9 or i==10 or i==11:
        #         notTry+=1
        #         numObject[i]=50
        #         continue
        #     weight=0
        #     print target
        #     if target=="rawlings_baseball" or target=="rolodex_jumbo_pencil_cup" or target=="scotch_duct_tape":
        #         numObject[i]=50
        #         notTry+=1
        #         continue
        #     for j in range(len(binMap[CONST_BIN_NAMES[i]])):
        #         binList=binMap[CONST_BIN_NAMES[i]]
        #         weight=weight+objectWeight[binList[j]]
        #         if binList[j]=="creativity_chenille_stems":
        #             numObject[i]=40
        #         if objectWeight[binList[j]]==16.9 or objectWeight[binList[j]]==48:
        #             numObject[i]=50
        #             giveup=False
        #             notTry+=1
        #             continue
        #     binWeightDict[CONST_BIN_NAMES[i]]=weight
        #     binWeight=binWeight+[weight]
        #     if weight>threshold and giveup:
        #         numObject[i]=50
        #         notTry+=1
        # print numObject
        # # print binWeight
        # sortedBinIndex= sorted(range(len(numObject)), key=lambda k: numObject[k])
        # sortedBinIndex=[sortedBinIndex[i] for i in range(len(sortedBinIndex)-notTry)]
        # centerBins=[]
        # sideBins=[]
        # print sortedBinIndex
        # for i in sortedBinIndex:
        #     if i%3==1:
        #         centerBins.append(i)
        #     else:
        #         sideBins.append(i)
        # sortedBinIndex=sideBins+centerBins
        # print sortedBinIndex
        # BinOrder=[BinOrder[sortedBinIndex[i]] for i in range(len(sortedBinIndex))]
        # oneOrNot=[numObject[i]==1 for i in sortedBinIndex]
        # objectOrder=[objectOrder[i] for i in sortedBinIndex]
        # return (BinOrder, objectOrder, oneOrNot)
        return (sortedBinOrder, sortedTargetOrder, sortedRight)
if __name__ == "__main__":
    JSON_FILES=["PickTestA.json","PickTestB.json","PickTestC.json","PickTestD.json","PickTestE.json"]
    for i in range(len(JSON_FILES)):
        a=pickHandler("../JSON_FILES/"+JSON_FILES[i])
        [border,torder, aorder]=a.workBinOrder()
        print border
        print torder
        print aorder
