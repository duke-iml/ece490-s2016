import json

def object_pick_difficulty(bin_order):
    """
    Input: Object name as string
    Output: Difficulty score to pick object
    """
    return 1

def bin_difficulty(bin_contents, bin_order):
    """
    Input: objects as a list of strings, object name to pick as string
    Output: The difficulty score
    """
    return len(bin_contents) + object_pick_difficulty(bin_order)

def get_bin_order(raw_json_data, bin):
    """
    Inputs: Dictionary and bin name as string
    Output: Returns item to pick for that bin name
    """
    for obj in raw_json_data['work_order']:
        if obj['bin'] == bin:
            return obj['item']
    return None

with open(PICK_JSON_PATH) as pick_json_file:
    raw_json_data = json.load(pick_json_file)

bins = ['bin_A', 'bin_B', 'bin_C', 'bin_D', 'bin_E', 'bin_F', 'bin_G', 'bin_H', 'bin_I', 'bin_J', 'bin_K', 'bin_L']
for bin in bins:
    print bin_difficulty(raw_json_data['bin_contents'][bin], get_bin_order(raw_json_data, bin))
