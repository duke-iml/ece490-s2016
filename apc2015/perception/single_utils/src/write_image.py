#!/usr/bin/env python

#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import subprocess
import time
import psutil
import sys
import os

pid = None

class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/realsense/rgb",Image,self.callback, queue_size=1)

  def callback(self,data):
    exited = False
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      if len(sys.argv)>=2:
        name = sys.argv[1]
        if '.' not in name:
          name = name+'.jpg'
        cv2.imwrite('perception/single_utils/'+name, cv_image)
      else:
        cv2.imwrite('perception/single_utils/captured.jpg', cv_image)
      print "saved successfully"
      p = psutil.Process(pid)
      p.kill()
      os._exit(0)
    except CvBridgeError, e:
      print e


def main():
  global pid
  process = subprocess.Popen('hardware_layer/RealSense_ROS_Emitter', stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  pid = process.pid
  time.sleep(3)
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main()
    
