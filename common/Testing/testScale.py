import sys
sys.path.insert(0, "../")

print sys.path

from Sensors import scale
import time

myScale = scale.Scale_Measurement()

while(1):
	
	time.sleep(1)
	print("Scale reading")
	try:	
		print( myScale.readData(10))

	except KeyboardInterrupt:
		break 