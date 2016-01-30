import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import numpy as np
from sklearn.cluster import KMeans

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

data = []

f = open("normals.txt")
for l in f:
    x, y, z = l.strip().split()
    data.append([float(x), float(y), float(z)])
f.close()

data_mat = np.matrix(data)
kmean = KMeans(n_clusters=3, n_jobs=-1)
print "Fitting..."
kmean.fit(data_mat)
print "Fitted"

xs = []
ys = []
zs = []
colors = []
f = open("normals.txt")
for l in f:
    if random.random()>0.01:
        continue
    x, y, z = l.strip().split()
    xs.append(float(x))
    ys.append(float(y))
    zs.append(float(z))
    group = kmean.predict([float(x), float(y), float(z)])[0]
    if group==0:
        colors.append('r');
    elif group==1:
        colors.append('g');
    else:
        colors.append('b');
f.close()

ax.scatter(xs, ys, zs, c=colors, s=20)
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
plt.show()

