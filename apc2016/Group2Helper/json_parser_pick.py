import json
from pprint import pprint

class json_parser_pick(object):
    def __init__(self):
        self.bin_list = ["bin_A", "bin_B", "bin_C", "bin_D", "bin_E",
            "bin_F","bin_G", "bin_H", "bin_I", "bin_J", "bin_K", "bin_L"]

    # Parses the input filename, and returns a tuple of the inital shelf mapping (bin name maps to bin contents) and the workOrder (all of the items in the tote)
    def readInFile(self, filename):
        with open(filename) as data_file:
            data = json.load(data_file)
        # print data
        binMap = dict()
        workOrder = []
        toteContents=[]

        for binName in self.bin_list:
            bin_contents = data["bin_contents"][binName]
            binMap[binName] = bin_contents

        workOrder = data["work_order"]

        return (binMap,toteContents, workOrder)
    def writeOutFile(self, filename, shelfMap, toteContents, workOrder):
        with open(filename, "w") as file:
            json.dump({"bin_contents":{self.bin_list[i]:shelfMap[self.bin_list[i]] for i in 
                range(0, len(self.bin_list))}, "tote_contents":toteContents, "work_order":workOrder}, file, indent=4, sort_keys=True)