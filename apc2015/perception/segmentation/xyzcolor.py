from color import rgb_to_yuv,make_uv_hist,eval_uv_hist
import random
import numpy as np

def _compute_means(points,labels,K):
    means = [np.zeros(points.shape[1]) for i in xrange(K)]
    counts = [0]*K
    for i in xrange(points.shape[0]):
        means[labels[i]] += points[i,:]
        counts[labels[i]] += 1
    for i in xrange(K):
        if counts[i] != 0:
            means[i] /= counts[i]
        else:
            means[i] = None
    return means

def _assign_closest(points,centers):
    res = []
    for i in xrange(points.shape[0]):
        cindex = -1
        #print "Point i:",points[i,:]
        diffs = [c-points[i,:] for c in centers]
        #print diffs
        dmin,cindex = min((np.dot(diff,diff),j) for (j,diff) in enumerate(diffs))
        res.append(cindex)
    return res


def kmeans(points,k,iters=20,initial='random'):
    centers = None
    labels = None
    if initial == 'random':
        centers = random.sample(points,k)
        #print "Centers:",centers
        labels = _assign_closest(points,centers)
    else:
        labels = initial[:]
    #run Kmeans algorithm
    for iteration in xrange(iters):
        centers = _compute_means(points,labels,k)
        #fix up 
        for i,c in enumerate(centers):
            if c == None:
                c = random.choice(points)
        labels = _assign_closest(points,centers)
    #return notion of cluster relative size
    sum_cluster_distance = 0
    max_cluster_distance = 0
    dsum = [0.0]*k
    dcnt = [0]*k
    for i in xrange(points.shape[0]):
        dsum[labels[i]] += np.linalg.norm(points[i,:]-centers[labels[i]])
        dcnt[labels[i]] += 1
    for i in xrange(k):
        try:
            dsum[i] /= dcnt[i]
        except ZeroDivisionError:
            dsum[i] = 0
        max_cluster_distance = max(dsum[i],max_cluster_distance)
        sum_cluster_distance += dsum[i]
    cluster_size_deviation = 1.0 - sum_cluster_distance/(k*max_cluster_distance)
    return labels,cluster_size_deviation

def xyzrgb_segment_kmeans(pc,colors,histograms,hist_bin_size=4):
    """Input:
    - pc is a numpy array of points of size N * 3
    - colors is an array of colors of size N * 3
    - historgrams is a list of uv histograms
    Output (labels,M,cost):
    - labels: an N*1 vector of integers 0,...,M
    - M: the number of histograms
    - cost: a number denoting cost of the histogram.
      better values are lower.
    """
    assert pc.shape[1]==3
    xscale = 2.0
    yscale = 2.0
    zscale = 1.0
    hscale = np.amax(np.amax(pc,0)-np.amin(pc,0))
    assert colors.shape[1]==3
    features = np.zeros((pc.shape[0],3+len(histograms)))
    features[:,0] = pc[:,0]*xscale
    features[:,1] = pc[:,1]*yscale
    features[:,2] = pc[:,2]*zscale
    h = []
    for i in xrange(pc.shape[0]):
        uv = rgb_to_yuv(*colors[i,:])[1:3]
        hi = [eval_uv_hist(hj,uv,hist_bin_size)*hscale for hj in histograms]
        h.append(hi)
    features[:,3:] = np.array(h)
    naive_labeling = []
    for i in xrange(pc.shape[0]):
        hi,index = max((v,j) for (j,v) in enumerate(h[i]))
        naive_labeling.append(index)
    labels,quality = kmeans(features,len(histograms),initial=naive_labeling)
    return (labels,len(histograms),quality)

if __name__ == '__main__':
    N1 = 3000
    N2 = 2000
    pc1 = [[random.uniform(0.1,0.2),random.uniform(0.5,0.7),random.uniform(1.3,1.7)] for i in xrange(N1)]
    pc2 = [[random.uniform(0.13,0.18),random.uniform(0.7,0.8),random.uniform(1.3,1.7)] for i in xrange(N2)]
    rgb1 = [[random.uniform(0.4,0.8),random.uniform(0.2,0.3),random.uniform(0.0,1.0)] for i in xrange(N1)]
    rgb2 = [[random.uniform(0.2,0.5),random.uniform(0.1,0.8),random.uniform(0.3,0.6)] for i in xrange(N2)]
    hist1 = make_uv_hist([rgb_to_yuv(*c)[1:3] for c in rgb1])
    hist2 = make_uv_hist([rgb_to_yuv(*c)[1:3] for c in rgb2])
    testxyz = pc1 + pc2
    testrgb = rgb1 + rgb2
    
    labels,K,cost = xyzrgb_segment_kmeans(np.array(testxyz),np.array(testrgb),[hist1,hist2])
    print "Cost:",cost
    TP,TN,FP,FN = 0,0,0,0
    for i in xrange(0,N1):
        if labels[i] == 0:
            TP += 1
        else:
            FN += 1
    for i in xrange(N1,N1+N2):
        if labels[i] == 1:
            TN += 1
        else:
            FP += 1
    print "Obj1 labeled as Obj1:",TP
    print "Obj2 labeled as Obj2:",TN
    print "Obj2 labeled as Obj1:",FP
    print "Obj1 labeled as Obj2:",FN
    
