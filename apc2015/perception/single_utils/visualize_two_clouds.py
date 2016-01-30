from __future__ import division
import sys
import matplotlib.pyplot as plt
from common_utils import *

if len(sys.argv)<3:
	print "Usage: 'program arg1 arg2' where arg1 is the file of real segmented point cloud and arg2 is the model point cloud"
	quit()

rgb1 = read_pcd_file(sys.argv[1], ['xyzrgb'])[0]
rgb2 = read_pcd_file(sys.argv[2], ['xyzrgb'])[0]

# render_3d_scatter(rgb1)
# plt.show()
# quit()

proportion = 0.03

every = int(1/proportion)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

rgb = [c for _,_,_,c in rgb1]
rgb_int = [struct.unpack('I', struct.pack('f', c))[0] for c in rgb]
r = [c >> 16 & 0x0000ff for c in rgb_int]
g = [c >> 8 & 0x0000ff for c in rgb_int]
b = [c >> 0 & 0x0000ff for c in rgb_int]
rgb = [[r[i]/255, g[i]/255, b[i]/255] for i in xrange(len(r))]
x_selected = [x for i,(x,_,_,_) in enumerate(rgb1) if i%every==0]
y_selected = [y for i,(_,y,_,_) in enumerate(rgb1) if i%every==0]
z_selected = [z for i,(_,_,z,_) in enumerate(rgb1) if i%every==0]
rgb_selected = [c for i,c in enumerate(rgb) if i%every==0]
ax.scatter(x_selected, y_selected, zs=z_selected, c=rgb_selected, linewidths=0)


proportion = 0.003

rgb = [c for _,_,_,c in rgb2]
rgb_int = [struct.unpack('I', struct.pack('f', c))[0] for c in rgb]
r = [c >> 16 & 0x0000ff for c in rgb_int]
g = [c >> 8 & 0x0000ff for c in rgb_int]
b = [c >> 0 & 0x0000ff for c in rgb_int]
rgb = [[r[i]/255, g[i]/255, b[i]/255] for i in xrange(len(r))]
x_selected = [x for i,(x,_,_,_) in enumerate(rgb2) if i%every==0]
y_selected = [y for i,(_,y,_,_) in enumerate(rgb2) if i%every==0]
z_selected = [z for i,(_,_,z,_) in enumerate(rgb2) if i%every==0]
rgb_selected = [c for i,c in enumerate(rgb) if i%every==0]
ax.scatter(x_selected, y_selected, zs=z_selected, linewidths=0, alpha=0.5)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_aspect('equal')

plt.show()
