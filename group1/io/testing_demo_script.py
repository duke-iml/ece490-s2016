import json_handler

''' Main Code '''
newHandler = json_handler.jsonHandler()

(origMap, workOrder) = newHandler.readInFile('RandomTestA.json')


# Test 'moving items' from tote to shelf
random_bin = ['bin_B', 'bin_G', 'bin_A', 'bin_D', 'bin_F']
for i in range(0,5):
    item = workOrder[0][0]
    # Place item in map
    origMap[random_bin[i]].append(item)
    workOrder[0].remove(item)

newHandler.writeOutFile('RandomTestAResults.json', origMap, workOrder)

