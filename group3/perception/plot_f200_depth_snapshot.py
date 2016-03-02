import sys
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

NPZ_FILE = 'cup_pc.npz'
STEP = 100 # Plot every STEPth point for speed, set to 1 to plot all


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

data = np.load(NPZ_FILE)
ax.scatter(data['arr_0'][::STEP], data['arr_1'][::STEP], data['arr_2'][::STEP])
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()
# plt.savefig('a.png')