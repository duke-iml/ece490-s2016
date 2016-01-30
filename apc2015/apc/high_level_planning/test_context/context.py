import json

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
    global bin_contents,order
    # open the json file
    print "Loading JSON data from",fn
    json_data = open(fn)
    data = json.load(json_data)
    bin_contents = data['bin_contents']
    order = [v['item'] for v in data['work_order']]

def num_items(bin):
    """Returns the number of items in a bin.  Used in action / reward
    definitions"""
    if bin not in bin_contents: return 0
    return len(bin_contents[bin])

def object_in_bin(object,bin):
    """Returns True if the object is in the given bin bin.  Used in
    action definition"""
    if bin not in bin_contents: return False
    return object in bin_contents[bin]

def score(reward,cost):
    """Used by the high-level planner to define the score of a given reward
    / cost pair."""
    if cost == 0:
        return 0
    return float(reward)/cost

#Uncomment this to have a default APC JSON file
#load_from_apc_file('apc.json')

#Used by the application to define a default start state
def start_state():
    return {'bin':None,'at_bin':False,'pick_done':False,
               'grasp_failures':0,'pc_grasp_failures':0}
