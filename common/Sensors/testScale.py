import scale
import time

myScale = scale.Scale()

while(1):
	
	time.sleep(1)
	print("Scale reading")
	try:	
		print myScale.readData(10)
		print 'some'

	except KeyboardInterrupt:
		break
