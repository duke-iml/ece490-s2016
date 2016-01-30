from klampt import vectorops,so3,se3

#a list of actual items -- this is only used for the fake perception module, and your
#code should not use these items directly
_ground_truth_items = []
_ground_truth_shelf_xform = se3.identity()

def set_ground_truth(shelf_xform,items):
    """Must be configured for simulated perception to work"""
    global _ground_truth_shelf_xform,_ground_truth_items
    _ground_truth_shelf_xform = shelf_xform
    _ground_truth_items = items



def run_perception_on_shelf(knowledge):
    """This is a fake perception module that simply reveals the shelf
    xform."""
    knowledge.shelf_xform = _ground_truth_shelf_xform

def run_perception_on_bin(knowledge,bin_name):
    """This is a fake perception module that simply reveals all the items
    the given bin."""
    global _ground_truth_items
    if knowledge.bin_contents[bin_name]==None:
        #not sensed yet
        knowledge.bin_contents[bin_name] = []
        for item in _ground_truth_items:
            if item.bin_name == bin_name:
                #place it in the bin
                knowledge.bin_contents[bin_name].append(item)
    return

