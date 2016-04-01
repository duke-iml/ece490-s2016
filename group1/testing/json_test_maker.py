import json
import random
import readline

class autoCompleter(object):
        
    def __init__(self, options):
        self.options = sorted(options)

    def complete(self, text, state):
        if state==0:
            if text:
                self.matches = [s for s in self.options
                                    if s and s.startswith(text)]
            else:
                self.matches = self.options[:]

        try:
            return self.matches[state]
        except IndexError:
            return None

bin_list = ["bin_A", "bin_B", "bin_C", "bin_D", "bin_E", "bin_F",
            "bin_G", "bin_H", "bin_I", "bin_J", "bin_K", "bin_L"]

item_list = ["i_am_a_bunny_hardcover_book",
             "laugh_out_loud_jokes_for_kids_paperback_book",
             "scotch_bubble_mailer",
            "up_and_up_glucose_tablets",
            "dasani_water_bottle",
            "rawlings_recreational_use_baseball",
            "folgers_classic_roast_coffee",
            "elmers_glue_all",
            "hanes_tube_socks",
            "womens_knit_gloves",
            "cherokee_easy_tee_t_shirt",
            "peva_shower_curtain_liner",
            "cloud_b_plush_bear",
            "boots_and_barkley_beefhide_bones",
            "plush_puppies_squeakin_eggs",
            "cool_shot_mini_glue_sticks",
            "creativity_street_chenille_stems_100_count",
            "40w_soft_white_lightbulb",
            "safety_1st_ultra_clear_outlet_plugs",
            "oral_b_cavity_defense_toothbrush",
            "dr_browns_natural_flow_bottle_brush",
            "command_hooks",
            "easter_turtle_figural_sippy_cup",
            "fiskars_scissors_5inch_blunt_tip_red",
            "scotch_cloth_duct_tape",
            "woods_6ft_extension_cord",
            "platinum_pets_dog_bowl",
            "fitness_gear_3lb_neoprene_dumbbell",
            "rolodex_wire_mesh_jumbo_pencil_holder",
            "clorox_utility_brush",
            "kleenex_viva_paper_towels",
            "expo_white_board_care_eraser",
            "kleenex_cool_touch_tissue_box",
            "ticonderoga_#2_pencils_12_count",
            "crayola_crayons",
            "jane_eyre_dvd",
            "dove_dry_oil_beauty_bar",
            "staples_3x5_ruled_index_cards"
            ]

not_used = item_list

used = []

bin_map = dict()

filename="";

def create_space():
    print ""
    print ""
    print ""
    print ""
    print ""

def parse_filename(tempfile):
    global filename
    if (tempfile.endswith(".json") == False):
        tempfile = tempfile + ".json"
    tempfile = "json_output_files/"+tempfile
    filename = tempfile

def choose_mode():
    create_space()
    print "*****************************"
    print "Please choose which mode you want to use:"
    print "(0) - Random Mode: Create a JSON file from random assignment"
    print "(1) - Interactive Mode: Create a JSON file using an interactive console prompt"
    print "(2) - Quit"
    choice = int(raw_input("Mode choice: "))
    if choice==0:
        random_mode()
    elif choice==1:
        interactive_mode()
    elif choice==2:
        pass
    else:
        print "Please choose an available mode (use 0 or 1)"
        choose_mode()

def make_file():
    with open(filename, "w") as file:
        json.dump({"bin_contents":{bin_list[i]:bin_map[bin_list[i]] for i in range(0, len(bin_list))}, "work_order":[]}, file, indent=4, sort_keys=True)

def initialize_map():
    for i in range(0, len(bin_list)):
        bin_map[bin_list[i]] = []

def random_mode():
    number_of_args = int(raw_input("Number of items you want on shelf: "))
    if number_of_args > len(item_list):
        print "You requested", number_of_args,"items, but there are only",len(item_list),"items available.  Capping the number of items added to the shelf at",len(item_list)

    initialize_map()

    for i in range(0, number_of_args):
        bin_name = random.choice(bin_list)
        item_name = random.choice(not_used)
        not_used.remove(item_name)
        used.append(item_name)
        bin_map[bin_name].append(item_name)

    make_file()

def print_remaining_items():
    print "Items remaining:"
    for i in range(0, len(not_used)):
        print not_used[i]

def add_new_item_to_shelf():
    completer = autoCompleter(not_used)
    readline.set_completer(completer.complete)
    readline.parse_and_bind('tab: complete')

    bin_name = "bin_" + raw_input("Enter a bin between A and L: ")
    if bin_name in bin_list:
        item_name = raw_input("Enter an item name: ")
        if item_name in not_used:
            bin_map[bin_name].append(item_name)
        else:
            print "The item you requested is not available"
    else:
        print "The only available bins are bin_A to bin_L"

def see_current_shelf_contents():
    print "Current shelf contents: "
    for i in range(0, len(bin_list)):
        print bin_list[i],":",bin_map[bin_list[i]]

def remove_item_from_shelf():
    if (len(used) == 0):
        print "There's nothing to remove!"
    else:
        item_name = raw_input("What item do you wish to remove?: ")
        if item_name in used:
            for i in range(0, len(bin_list)):
                if item_name in bin_map[bin_list[i]]:
                    # Remove the item
                    bin_map[bin_list[i]].remove(item_name)
                    not_used.append(item_name)
                    used.remove(item_name)
                    break
                else:
                    continue
                
        else:
            print "That item currently is not on the shelf"

def give_interactive_options():
    print "*********************"
    print "*********************"
    print "Options: "
    print "(0) - Add new item to shelf"
    print "(1) - See current shelf contents"
    print "(2) - See available items to add"
    print "(3) - Remove item from shelf"
    print "(4) - Finish making file"
    print "(5) - Quit"
    

def interactive_mode():
    initialize_map()
    while(True):
        give_interactive_options()
        option = int(raw_input("Please choose a menu item: "))
        if option==0:
            add_new_item_to_shelf()
        elif option==1:
            see_current_shelf_contents()
        elif option==2:
            print_remaining_items()
        elif option==3:
            remove_item_from_shelf()
        elif option==4:
            make_file()
            break
        elif option==5:
            break
        else:
            print "Not a valid option"


if __name__ == "__main__":
    create_space()
    tempfile = raw_input("Please enter the desired filename: ")
    parse_filename(tempfile)
    choose_mode()

    
