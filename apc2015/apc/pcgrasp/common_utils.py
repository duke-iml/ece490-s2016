
from __future__ import division

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import struct
import networkx as nx
from networkx import Graph
from math import sqrt

def argmin(list):
    '''
    return the index of the smallest item in the list
    '''
    return sorted([[val, idx] for idx, val in enumerate(list)])[0][1]

def argmax(list):
    '''
    return the index of the largest item in the list
    '''
    return sorted([[val, idx] for idx, val in enumerate(list)], reverse=True)[0][1]

def local_min(list, idx):
    '''
    return the index and the value of the local minimum following descent initialized at the specified index
    '''
    if idx==0 and list[0]>list[1]:
        idx += 1
    length = len(list)
    if idx==length-1 and list[length-2]<list[length-1]:
        idx -= 1
    while (not idx==0) and (not idx==length-1) and not(list[idx-1]>=list[idx] and list[idx+1]>=list[idx]):
        if list[idx]>list[idx-1]:
            idx -= 1
        else:
            idx += 1
    return idx, list[idx]

def select(list, idxs):
    '''
    idxs: a list of indices
    return a list that consists of only those in the original list with indices in idxs
    '''
    return [list[i] for i in idxs]

def select_each(list_of_list, idxs):
    '''
    apply select on each list in list_of_list
    return the new list_of_list with each list being selected by idxs
    '''
    return [select(l, idxs) for l in list_of_list]

def quantile(list, q):
    '''
    q: a float between 0 and 1 specifying the quantile
    return the element in the list at the (q*100)th quantile
    '''
    return list[int(len(list)*q)]

def f_addr_to_i(f):
    return struct.unpack('I', struct.pack('f', f))[0]
    
def i_addr_to_f(i):
    return struct.unpack('f', struct.pack('I', i))[0]

def rgb_to_pcl_float(r, g, b):
    i = r<<16 | g<<8 | b
    return i_addr_to_f(i)

def render_3d_scatter(points, proportion=1, xlabel="x", ylabel="y", zlabel="z"):
    '''
    render 3d points. the points are represented by a list of lists (points) i.e. [[x1,y1,z1],[x2,y2,z2],...,[xn,yn,zn]]
    return the axis handler of the image (which, for example, can be used to change window limit by set_xlim, set_ylim, set_zlim)
    '''
    if len(points[0])==4:
        ax = render_3d_scatter_with_rgb(points, proportion, xlabel, ylabel, zlabel)
        return ax
    every = int(1/proportion)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter([x for i,(x,_,_) in enumerate(points) if i%every==0], 
        [y for i,(_,y,_) in enumerate(points) if i%every==0], zs=[z for i,(_,_,z) in enumerate(points) if i%every==0])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_aspect('equal')
    return ax

def render_3d_scatter_with_rgb(points, proportion=1, xlabel="x", ylabel="y", zlabel="z"):
    '''
    render 3d points. the points are represented by a list of lists (points with rgb) i.e. [[x1,y1,z1,rgb1],[x2,y2,z2,rgb2],...,[xn,yn,zn,rgbn]]
    return the axis handler of the image (which, for example, can be used to change window limit by set_xlim, set_ylim, set_zlim)
    '''
    every = int(1/proportion)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    rgb = [c for _,_,_,c in points]
    rgb_int = [struct.unpack('I', struct.pack('f', c))[0] for c in rgb]
    r = [c >> 16 & 0x0000ff for c in rgb_int]
    g = [c >> 8 & 0x0000ff for c in rgb_int]
    b = [c >> 0 & 0x0000ff for c in rgb_int]
    rgb = [[r[i]/255, g[i]/255, b[i]/255] for i in xrange(len(r))]
    x_selected = [x for i,(x,_,_,_) in enumerate(points) if i%every==0]
    y_selected = [y for i,(_,y,_,_) in enumerate(points) if i%every==0]
    z_selected = [z for i,(_,_,z,_) in enumerate(points) if i%every==0]
    rgb_selected = [c for i,c in enumerate(rgb) if i%every==0]
    ax.scatter(x_selected, y_selected, zs=z_selected, c=rgb_selected, linewidths=0)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_aspect('equal')
    return ax

def remove_plane(point_cloud, coord, plane, tolerance=0.03):
    '''
    point_cloud format: [[x1,y1,z1],[x2,y2,z2],...,[xn,yn,zn]]
    '''
    if plane=="xy":
        return [(x,y,z) for x,y,z in point_cloud if abs(z-coord)>tolerance]
    elif plane=="yz":
        return [(x,y,z) for x,y,z in point_cloud if abs(x-coord)>tolerance]
    elif plane=="xz":
        return [(x,y,z) for x,y,z in point_cloud if abs(y-coord)>tolerance]
    else:
        raise Exception("Unrecognized plane name")

def remove_plane_rgb(point_cloud, coord, plane, tolerance=0.03):
    '''
    point_cloud format: [[x1,y1,z1,rgb1],[x2,y2,z2,rgb2],...,[xn,yn,zn,rgbn]]
    rgb is a float packed from three numbers using PCL's encoding scheme
    '''
    if plane=="xy":
        return [(x,y,z,rgb) for x,y,z,rgb in point_cloud if abs(z-coord)>tolerance]
    elif plane=="yz":
        return [(x,y,z,rgb) for x,y,z,rgb in point_cloud if abs(x-coord)>tolerance]
    elif plane=="xz":
        return [(x,y,z,rgb) for x,y,z,rgb in point_cloud if abs(y-coord)>tolerance]
    else:
        raise Exception("Unrecognized plane name")

def remove_plane_idx(point_cloud, coord, plane, tolerance=0.03):
    '''
    return the indices of the points on the plane to be removed
    '''
    if plane=="xy":
        return [i for i,p in enumerate(point_cloud) if abs(p[2]-coord)>tolerance]
    elif plane=="yz":
        return [i for i,p in enumerate(point_cloud) if abs(p[0]-coord)>tolerance]
    elif plane=="xz":
        return [i for i,p in enumerate(point_cloud) if abs(p[1]-coord)>tolerance]
    else:
        raise Exception("Unrecognized plane name")

def read_pcd_file(f, data):
    if isinstance(f, basestring):
        f = open(f)
    pointsxyzrgb = []
    pointsxyz = []
    pointsxy = []
    pointsyz = []
    pointsxz = []
    all_x = []
    all_y = []
    all_z = []
    for l in f:
        try:
            float(l.strip().split()[0])
        except:
            continue
        x, y, z, rgb = map(float, l.strip().split())
        pointsxyzrgb.append([x,y,z,rgb])
        pointsxyz.append([x,y,z])
        pointsxy.append([x,y])
        pointsyz.append([y,z])
        pointsxz.append([x,z])
        all_x.append(x)
        all_y.append(y)
        all_z.append(z)
    ret = []
    for d in data:
        if d=='rgb' or d=='xyzrgb':
            ret.append(pointsxyzrgb)
        elif d=='xyz':
            ret.append(pointsxyz)
        elif d=='xy':
            ret.append(pointsxy)
        elif d=='yz':
            ret.append(pointsyz)
        elif d=='xz':
            ret.append(pointsxz)
        elif d=='x':
            ret.append(all_x)
        elif d=='y':
            ret.append(all_y)
        elif d=='z':
            ret.append(all_z)
        else:
            raise Exception("Unrecgonized data format"+str(d))
    return ret

def write_pcd_file(point_cloud, f):
    if isinstance(f, basestring):
        f = open(f, 'w')
    tot_num = len(point_cloud)
    has_rgb = len(point_cloud[0])==4
    if has_rgb:
        f.write("VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n")
        f.write("WIDTH "+str(tot_num)+"\n")
        f.write("HEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        f.write("POINTS "+str(tot_num)+"\n")
        f.write("DATA ascii\n")
    else:
        f.write("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write("WIDTH "+str(tot_num)+"\n")
        f.write("HEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        f.write("POINTS "+str(tot_num)+"\n")
        f.write("DATA ascii\n")
    for p in point_cloud:
        f.write(" ".join(map(str,p))+"\n")
    f.close()

def euclidian_3d_dist(p1, p2):
    return sqrt( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2 )

def make_graph(points, neighbor_max_dist=0.01):
    graph = Graph()
    graph.add_nodes_from(range(len(points)))
    for i in xrange(len(points)):
        for j in xrange(i+1, len(points)):
            if euclidian_3d_dist(points[i], points[j])<neighbor_max_dist:
                graph.add_edge(i,j)
    return graph

def get_largest_cc(points, neighbor_max_dist=0.01):
    graph = make_graph(points, neighbor_max_dist)
    idxs = sorted(nx.connected_components(graph), key=len, reverse=True)[0]
    return select(points, idxs)
    
