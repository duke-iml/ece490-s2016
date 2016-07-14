import itertools
import numpy as np

class GridPointLocation:
  def __init__(self,points,cellsize=0.1):
    points = np.array(points)
    self.data = points
    self.grid = {}
    self.cellsizeinv = 1.0/cellsize
    for i in xrange(points.shape[0]):
      c = self.index(points[i,:])
      try:
        self.grid[c].append(i)
      except KeyError:
        self.grid[c] = [i]
  def index(self,pt):
    # print pt.__class__
    # print self.cellsizeinv
    return tuple(int(v*self.cellsizeinv) for v in pt)
  def query(self,pts,distance_upper_bound=1):
    print "1"
    if distance_upper_bound == None:
      raise ValueError("TODO: closest point without distance bound")
    d = []
    ind = []
    print "2"
    print "pts shape", pts.shape
    for i in xrange(pts.shape[0]):
      if i%200==0:
        print i
      di = float('inf')
      indi = None
      for j in self.pts_within_tol(pts[i,:],distance_upper_bound):
        dj = np.linalg.norm(pts[i,:]-self.data[j,:])
        if dj < di and dj <= distance_upper_bound:
          di = dj
          indi = j
      d.append(di)
      ind.append(indi)
    return d,ind

  def query_ball_point(self, pts, radius):
    return self.query_ball_pt(pts, radius)
  def query_ball_pt(self,pts,radius):
    d = []
    ind = []
    for i in xrange(pts.shape[0]):
      indi = []
      for j in self.pts_within_tol(pts[i,:],radius):
        dj = np.linalg.norm(pts[i,:]-self.data[j,:])
        if dj <= radius:
          indi.append(j)
      ind.append(indi)
    return ind
  def pts_within_tol(self,pt,dist):
    """Returns indices of all data points within the cells within +/- dist of pt"""
    bmin = pt - np.ones(pt.shape)*dist
    bmax = pt + np.ones(pt.shape)*dist
    cmin = self.index(bmin)
    cmax = self.index(bmax)
    res = []
    # print "within 1"
    # print len(list(itertools.product(*[range(a,b+1) for (a,b) in zip(cmin,cmax)])))
    for index in itertools.product(*[range(a,b+1) for (a,b) in zip(cmin,cmax)]):
      # print tuple(index)
      res += self.grid.get(tuple(index),[])
    return res


if __name__ == '__main__':
  A = np.array([[1,0],[0.9,0],[0.5,0.8]])
  g = GridPointLocation(A,0.1)
  print g.query_ball_point(np.array([[0.95,0.05]]),0.1)
