import sys
sys.path.insert(0, "../")

print sys.path

from Sensors import multiScale

import time

myScale = multiScale.Scale_Measurement()

first = False

start = time.clock()

while(1):

	time.sleep(1)
	print("Scale reading")
	try:	
		sum = 0
		totalWeight = myScale.readData(10)
		for index in range(len(totalWeight)):
			print( 'scale ', index, 'is ', totalWeight[index])
			sum += float(totalWeight[index].split(' ')[0])
		print ('Total weight is ', sum)
	
	except KeyboardInterrupt:
		break 

	except:
		if not first:
			first = True
			end = time.clock()

		print ('Scales stayed on for', end-start)
		traceback.print_stack()
