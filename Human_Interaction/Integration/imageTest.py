#send Image to baxter

import os
import sys
import argparse
 
import rospy
 
import cv2
import cv_bridge
 
from sensor_msgs.msg import (
    Image,
)

def send_image(path):
	"""
	Send the image located at the specified path to the head
	display on Baxter.

	@param path: path to the image file to load and send
	"""

	img = cv2.imread(path)
	msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")

	pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
	pub.publish(msg)
	# Sleep to allow for image to be published.
	rospy.sleep(1)



def main():
	"""RSDK Xdisplay Example: Image Display

		Displays a given image file on Baxter's face.

		Pass the relative or absolute file path to an image file on your
		computer, and the example will read and convert the image using
		cv_bridge, sending it to the screen as a standard ROS Image Message.
		"""
	epilog = """
	Notes:
		Max screen resolution is 1024x600.
		Images are always aligned to the top-left corner.
		Image formats are those supported by OpenCv - LoadImage().
		"""
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt,description=main.__doc__, epilog=epilog)
	required = parser.add_argument_group('required arguments')
	required.add_argument(
	    '-f', '--file', metavar='PATH', required=True,
	    help='Path to image file to send'
	)
	parser.add_argument(
	    '-d', '--delay', metavar='SEC', type=float, default=0.0,
	    help='Time in seconds to wait before publishing image'
	)
	args = parser.parse_args(rospy.myargv()[1:])

	rospy.init_node('rsdk_xdisplay_image', anonymous=True)

	if not os.access(args.file, os.R_OK):
	    rospy.logerr("Cannot read file at '%s'" % (args.file,))
	    return 1

	if args.delay > 0:
		rospy.loginfo(
	    "Waiting for %s second(s) before publishing image to face" %
	    (args.delay,)
	)
	rospy.sleep(args.delay)

	send_image(args.file)
	return 0
 
if __name__ == '__main__':
	sys.exit(main())