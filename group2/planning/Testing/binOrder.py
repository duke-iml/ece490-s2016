import json
import json_parser
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
CONST_ITEM_WEIGHTS=[3.2, 2.9, 1.6, 1.6, 11.2, 16.9, 16.9,
6, 11.3, 4, 4, 16, 2,
1, 11.8, 3.2, 4, 3.2,
2.4, 4, 4, 3, 0.3, 0.5, 0.5,
4.2, 1.6, 12, 1.4, 2, 2,
6.4, 1.6, 48, 3.2, 7,
12.8, 0.3, 0.3, 10, 2.1, 8,
5.12, 4, 3, 3] 
class binOrder:
    def __init__(self):
        self.parser=json_parser.json_parser()
        self.BinOrder=CONST_BIN_NAMES
    def workBinOrder(self,filename):
        # filename="apc_pick_task.json"
        (binMap, workOrder)=self.parser.readInFile(filename)
        numObject=[]
        objectWeight=dict()
        binWeightDict=dict()
        binWeight=[]
        sortedBinIndex=[]
        objectOrder=[]
        threshold=48
        notTry=0
        giveup=True
        for i in range(len(CONST_ITEM_NAMES)):
            objectWeight[CONST_ITEM_NAMES[i]]=CONST_ITEM_WEIGHTS[i]

        for i in range(len(CONST_BIN_NAMES)):
            target=workOrder[i]["item"]
            objectOrder=objectOrder+[target]
            numObject=numObject+[len(binMap[CONST_BIN_NAMES[i]])]
            weight=0
            # print target
            if target=="rawlings_baseball" or target=="rolodex_jumbo_pencil_cup":
                numObject[i]=50
                notTry+=1
                continue
            for j in range(len(binMap[CONST_BIN_NAMES[i]])):
                binList=binMap[CONST_BIN_NAMES[i]]
                weight=weight+objectWeight[binList[j]]
                if objectWeight[binList[j]]==16.9 or objectWeight[binList[j]]==48:
                    numObject[i]=50
                    giveup=False
                    notTry+=1
                    continue

            binWeightDict[CONST_BIN_NAMES[i]]=weight
            binWeight=binWeight+[weight]
            if weight>threshold and giveup:
                numObject[i]=50
                notTry+=1
        # print numObject
        # print binWeight
        sortedBinIndex= sorted(range(len(numObject)), key=lambda k: numObject[k])
        sortedBinIndex=[sortedBinIndex[i] for i in range(len(sortedBinIndex)-notTry)]
        centerBins=[]
        sideBins=[]
        for i in sortedBinIndex:
            if i%3==1:
                centerBins.append(i)
            else:
                sideBins.append(i)
        sortedBinIndex=sideBins+centerBins
        # print sortedBinIndex
        self.BinOrder=[self.BinOrder[sortedBinIndex[i]] for i in range(len(sortedBinIndex))]
        oneOrNot=[numObject[i]==1 for i in sortedBinIndex]
        objectOrder=[objectOrder[i] for i in sortedBinIndex]
        return (self.BinOrder, objectOrder, oneOrNot)
# if __name__ == "__main__":
#     a=binOrder()