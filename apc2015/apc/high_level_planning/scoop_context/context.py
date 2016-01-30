import json
import csv
from collections import defaultdict

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

def probability_of(ptable,event):
    if isinstance(event,list):
        pevent = 1.0
        for e in event:
            if e in ptable:
                pevent += ptable[e]
        return pevent
    if event not in ptable:
        return 0.0
    else:
        return ptable[event]

def probability_complement(ptable,event):
    return 1.0 - probability_of(ptable,event)

def probability_mul(ptable1,value):
    """Returns a copy of ptable1 scaled by value"""
    res = {}
    for p in ptable1:
        res[p] = ptable1[p]*value
    return res

def probability_dot(ptable1,ptable2):
    """Element-wise dot product of probability table ptable2 to ptable1"""
    res = 0.0
    for p in ptable1:
        if p in ptable2:
            res += ptable1[p]*ptable2[p]
    return res

def probability_product(ptable1,ptable2):
    """Element-wise multiply probability table ptable2 to ptable1"""
    res = {}
    for p in ptable1:
        if p in ptable2:
            res[p] = ptable1[p]*ptable2[p]
    return res

def probability_add(ptable1,ptable2):
    """Add probability table ptable2 to ptable1"""
    res = {}
    for p in ptable1:
        res[p] = ptable1[p]
    for p in ptable2:
        if p in res:
            res[p] += ptable2[p]
        else:
            res[p] = ptable2[p]
    return res


def probability_sub(ptable1,ptable2):
    """Subtract probability table ptable2 from ptable1"""
    res = {}
    for p in ptable1:
        res[p] = ptable1[p]
    for p in ptable2:
        if p in res:
            res[p] -= ptable2[p]
            if res[p] < 0:
                print "probability_sub: warning, element",p,"became negative"
        else:
            print "probability_sub: warning, element",p,"in second arg not in first arg"
    return res

def probability_normalize(ptable):
    """Inplace normalize of probability table"""
    psum = 0.0
    for p in ptable:
        psum += ptable[p]
    if psum == 0:
        return
    scale = 1.0/psum
    for p in ptable:
        ptable[p] *= scale


p_reachable_prior = dict((b,1.0) for b in bins)
p_reachable_prior['bin_A'] = 0
p_reachable_prior['bin_D'] = 0
p_reachable_prior['bin_G'] = 0
p_reachable_prior['bin_J'] = 0

#do some hacky things -- "high_level_planning/Strategy Optimization.csv" will
#have more data
p_detect_fail = dict((o,0.1) for o in objects)
p_detect_fail["rolodex_jumbo_pencil_cup"] = 0.9

p_scoopable_prior = dict((o,0.5) for o in objects)
p_scoopable_prior["oreo_mega_stuf"] = 0
p_scoopable_prior["feline_greenies_dental_treats"] = 0

p_scoop_plan_fail = dict((o,0.1) for o in objects)
p_drop = dict((o,0.1) for o in objects)

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
    scoopcol = None
    perceivecol = None
    for line in reader:
        if header==None:
            header = line
            assert 'Item' == header[0]
            assert 'Scoop' in header
            assert 'Perceive' in header
            scoopcol = header.index('Scoop')
            perceivecol = header.index('Perceive')
        else:
            assert len(line)>=5
            assert line[0] in objects,'Object "'+line[0]+'" appears invalid'
            obj = line[0]
            p_scoopable_prior[obj] = float(line[scoopcol])
            p_detect_fail[obj] = 1.0 - float(line[perceivecol])

#Uncomment this to have a default APC JSON file
#load_from_apc_file('apc.json')

#Used by the application to define a default start state
def start_state():
    global order,objects_on_shelf
    load_strategy_constants()
    return {'bin':None,'at':'home','scoop_done':False,
            'p_reachable':p_reachable_prior,
            'is_localized':dict((o,False) for o in objects_on_shelf),
            'p_scoopable':p_scoopable_prior,
            'p_in_bin':dict((o,1.0) for o in objects_on_shelf),
            'p_in_order_bin':dict((o,0.0) for o in objects_on_shelf),
            'p_detected_object':dict((o,0.0) for o in objects_on_shelf),
            }


