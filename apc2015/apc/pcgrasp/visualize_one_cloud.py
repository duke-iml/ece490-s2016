from common_utils import *
import matplotlib.pyplot as plt
import sys

xyzrgb= read_pcd_file(sys.argv[1],['xyzrgb'])[0]
if len(sys.argv)==3:
    proportion = float(sys.argv[2])
else:
    proportion = 1

render_3d_scatter(xyzrgb, proportion)
plt.show()

