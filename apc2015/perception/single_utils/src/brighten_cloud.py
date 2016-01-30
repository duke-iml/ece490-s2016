
from __future__ import division
from common_utils import *
import matplotlib.pyplot as plt
import sys



xyzrgb = read_pcd_file(sys.argv[1], ['xyzrgb'])[0]
write_pcd_file(xyzrgb, sys.argv[2])
