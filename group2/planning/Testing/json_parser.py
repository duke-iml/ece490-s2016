import json
from pprint import pprint

class json_parser(object):
    def __init__(self):
        self.bin_list = ["bin_A", "bin_B", "bin_C", "bin_D", "bin_E",    "bin_F","bin_G", "bin_H", "bin_I", "bin_J", "bin_K", "bin_L"]

    # Parses the input filename, and returns a tuple of the inital shelf mapping (bin name maps to bin contents) and the workOrder (all of the items in the tote)
    def readInFile(self, filename):
        with open(filename) as data_file:
            data = json.load(data_file)
        # print data
        binMap = dict()
        workOrder = []

        for binName in self.bin_list:
            bin_contents = data["bin_contents"][binName]
            binMap[binName] = bin_contents

        workOrder = data["work_order"]

        return (binMap, workOrder)