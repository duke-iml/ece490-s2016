#!/usr/bin/env python

from __future__ import division

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
import matplotlib.pyplot as plt
import numpy as np
from common_utils import *
from math import pi, sin, cos, tan, atan, sqrt


pid = None
file_name = None
rgb_mat = None
depth_mat = None
bridge = CvBridge()

w = 320
h = 240
diag_ang = 74/180*pi
diag = sqrt(w**2+h**2)
lift = diag/2 / tan(diag_ang/2)

def receive_rgb(data):
  global rgb_mat
  rgb_mat = bridge.imgmsg_to_cv2(data, "bgr8")
  if depth_mat is not None:
    process()

def receive_depth(data):
  global depth_mat
  depth_mat = bridge.imgmsg_to_cv2(data, "mono16")
  depth_mat = depth_mat[:,:,0]
  if rgb_mat is not None:
    process()

def process():
  psutil.Process(pid).kill()
  cv2.imwrite(file_name+".bmp", rgb_mat)
  cv2.imwrite(file_name+".depth.bmp", depth_mat)
  assert depth_mat.shape == (h, w)
  point_cloud = []
  for i in range(h):
    for j in range(w):
      depth = depth_mat[i, j]
      b1, g1, r1 = list(rgb_mat[i*2, j*2, :].flatten())
      b2, g2, r2 = list(rgb_mat[i*2+1, j*2, :].flatten())
      b3, g3, r3 = list(rgb_mat[i*2, j*2+1, :].flatten())
      b4, g4, r4 = list(rgb_mat[i*2+1, j*2+1, :].flatten())
      b1 = int(b1)
      b2 = int(b2)
      b3 = int(b3)
      b4 = int(b4)
      g1 = int(g1)
      g2 = int(g2)
      g3 = int(g3)
      g4 = int(g4)
      r1 = int(r1)
      r2 = int(r2)
      r3 = int(r3)
      r4 = int(r4)
      r = int((r1+r2+r3+r4)/4)
      g = int((g1+g2+g3+g4)/4)
      b = int((b1+b2+b3+b4)/4)
      rgb = rgb_to_pcl_float(r1, g1, b1)
      if depth==32001:
        continue
      assert depth<20000
      coord = (j+0.5-w/2, i+0.5-h/2)
      real_x = coord[0]/lift*depth
      real_y = coord[1]/lift*depth
      point_cloud.append([real_x/1000, real_y/1000, depth/1000, rgb])
  write_pcd_file(point_cloud, file_name)

  rospy.signal_shutdown("Point cloud made, shutting down...\n")
  

def main():
  global file_name
  if len(sys.argv)>=2:
    file_name = sys.argv[1]
  else:
    file_name = 'point_cloud.pcd'

  global pid
  process = subprocess.Popen('hardware_layer/RealSense_ROS_Emitter', stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  pid = process.pid
  time.sleep(3)
  
  rospy.init_node('naive_point_cloud', disable_signals=True)
  rgb_sub = rospy.Subscriber("/realsense/rgb", Image, receive_rgb, queue_size=1)
  depth_sub = rospy.Subscriber("/realsense/depth", Image, receive_depth, queue_size=1)
  
  rospy.spin()

if __name__ == '__main__':
    main()
    
