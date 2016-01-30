import json
import csv
from collections import defaultdict
from probability import *

bins = ['bin_'+c for c in ['A','B','C','D','E','F','G','H','I','J','K','L']]
objects = ["oreo_mega_stuf",
                    "champion_copper_plus_spark_plug",
                    "expo_dry_erase_board_eraser",
                    "kong_duck_dog_toy",
                    "genuine_joe_plastic_stir_sticks",
                    "munchkin_white_hot_duck_bath_toy",
                    "crayola_64_ct",
                    "mommys_helper_outlet_plugs",
                    "sharpie_accent_tank_style_highlighters",
                    "kong_air_dog_squeakair_tennis_ball",
                    "stanley_66_052",
                    "safety_works_safety_glasses",
                    "dr_browns_bottle_brush",
                    "laugh_out_loud_joke_book",
                    "cheezit_big_original",
                    "paper_mate_12_count_mirado_black_warrior",
                    "feline_greenies_dental_treats",
                    "elmers_washable_no_run_school_glue",
                    "mead_index_cards",
                    "rolodex_jumbo_pencil_cup",
                    "first_years_take_and_toss_straw_cup",
                    "highland_6539_self_stick_notes",
                    "mark_twain_huckleberry_finn",
                    "kyjen_squeakin_eggs_plush_puppies",
                    "kong_sitting_frog_dog_toy"]

#this is used by the planner to define argument domains
domains = {'bin':bins,'object':objects}

#these is used by the action definitions
bin_contents = {}
order = []
objects_on_shelf = {}

#these are used by the reward function definitions
bonus_items = {
    'mark_twain_huckeberry_finn' :		3,
    'kygen_squeakin_eggs_plush_puppies':		1,
    'kong_air_dog_squeakair_tennis_ball':		1,
    'dr_browns_bottle_brush':		2,
    'kong_duck_dog_toy':		1,
    'kong_sitting_frog_dog_toy':		1,
    'laugh_out_loud_joke_book':		1,
    'munchkin_white_hot_duck_bath_toy':		1,
    'safety_works_safety_glasses':		1,
    'stanley_66_052':		3,
    'rolodex_jumbo_pencil_cup':		2
    }


def load_from_apc_file(fn):
    """Loads the bin_contents and order from an APC json file"""
    global bin_contents,order,objects_on_shelf
    # open the json file
    print "Loading JSON data from",fn
    json_data = open(fn)
    data = json.load(json_data)
    bin_contents = data['bin_contents']
    order = [v['item'] for v in data['work_order']]
    objects_on_shelf = sum(bin_contents.values(),[])
    objects_on_shelf = list(set(objects_on_shelf))
    
def num_initial_items(bin):
    """Returns the number of items originally in a bin.  Used in action /
    reward definitions"""
    if bin not in bin_contents: return 0
    return len(bin_contents[bin])

def object_initially_in_bin(object,bin):
    """Returns True if the *original* object is in the given bin 'bin'.  Used
    in action definition"""
    if bin not in bin_contents: return False
    return object in bin_contents[bin]

def score(reward,cost):
    """Used by the high-level planner to define the score of a given reward
    / cost pair."""
    if cost == 0:
        return 0
    return float(reward)/cost



p_reachable_prior = dict((b,1.0) for b in bins)

#do some hacky things -- "high_level_planning/Strategy Optimization.csv" will
#have more data
p_detect_fail = dict((o,0.1) for o in objects)
p_detect_fail["rolodex_jumbo_pencil_cup"] = 0.9

p_graspable_prior = dict((o,0.5) for o in objects)
p_graspable_prior["oreo_mega_stuf"] = 0.3
p_graspable_prior["stanley_66_052"] = 0
p_graspable_prior["laugh_out_loud_joke_book"] = 0
p_graspable_prior["mark_twain_huckleberry_finn"] = 0

p_grasp_plan_fail = dict((o,0.1) for o in objects)
p_drop = dict((o,0.25) for o in objects)

def confusion_matrix(bin,object):
    global bin_contents
    assert object in bin_contents[bin],"Trying to pick object "+object+" not in bin "+bin
    res = dict()
    if len(bin_contents[bin])==1:
        res[object] = 1
    elif len(bin_contents[bin])==2:
        for o in bin_contents[bin]:
            res[o] = 0.3
        res[object] = 0.7
    else:
        for o in bin_contents[bin]:
            res[o] = 0.6/(len(bin_contents[bin])-1)
        res[object] = 0.4
    return res

def load_strategy_constants(fn = "Strategy Optimization.csv"):
    try:
        reader = csv.reader(open(fn,'r'))
    except IOError:
        raise
    header = None
    pinchcol = None
    powercol = None
    perceivecol = None
    for line in reader:
        if header==None:
            header = line
            assert 'Item' == header[0]
            assert 'Power Grasp' in header
            assert 'Pinch Grasp' in header
            assert 'Perceive' in header
            powercol = header.index('Power Grasp')
            pinchcol = header.index('Pinch Grasp')
            perceivecol = header.index('Perceive')
        else:
            assert len(line)>=5
            assert line[0] in objects,'Object "'+line[0]+'" appears invalid'
            obj = line[0]
            p_graspable_prior[obj] = max(float(line[pinchcol]),float(line[powercol]))
            p_detect_fail[obj] = 1.0 - float(line[perceivecol])

#Uncomment this to have a default APC JSON file
#load_from_apc_file('apc.json')

#Used by the application to define a default start state
def start_state():
    global order,objects_on_shelf
    load_strategy_constants()
    return {'bin':None,'at':'home','done':False,
            'p_reachable':p_reachable_prior,
            'is_localized':dict((o,False) for o in objects_on_shelf),
            'p_graspable':p_graspable_prior,
            'p_in_bin':dict((o,1.0) for o in objects_on_shelf),
            'p_in_order_bin':dict((o,0.0) for o in objects_on_shelf),
            'p_detected_object':dict((o,0.0) for o in objects_on_shelf),
            }


