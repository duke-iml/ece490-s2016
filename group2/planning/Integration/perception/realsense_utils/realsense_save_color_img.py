
from client import RemoteCamera
import cv2
import time
import sys
assert len(sys.argv)>=2, "Usage: python realsense_save_color_img.py filename.png [\"time\" countdown]"
if len(sys.argv)>2 and sys.argv[2]=="time":
	countdown = int(sys.argv[3])
	cur_elapsed = 0
else:
	countdown = None
cur_time = time.time()
camera = RemoteCamera('localhost')
cv2.namedWindow("Press Enter to Save")
while True:
	color, _, _, _ = camera.read()
	color = cv2.cvtColor(color, cv2.cv.CV_BGR2RGB)
	cv2.imshow("Press Enter to Save", color)
	key = cv2.waitKey(1)
	if countdown is not None and time.time()-cur_time>cur_elapsed:
		print countdown - cur_elapsed
		cur_elapsed += 1
	if countdown is not None and time.time()-cur_time>countdown:
		cv2.imwrite(sys.argv[1], color)
		break
	if key==13:
		cv2.imwrite(sys.argv[1], color)
		break
	if key==27:
		break
