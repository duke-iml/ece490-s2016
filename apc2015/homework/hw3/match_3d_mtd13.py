import numpy
import scipy.spatial, scipy.ndimage
from sys import stdout

from klampt import se3, so3
from utils import *

#this is the object that we know is in the scene
#TODO: play around with this object, it can be "highlighters", "kong_duck", "kygen_eggs", and "school_glue"
object = "kong_duck"

# Read the model and the scene point cloud
model_file = "optimized_tsdf_textured_mesh.json"
depth_file = "cup_only.json"

cached = None

def multiscale_connected_components(scene, scene_tree, resolution, divisors):
    # ensure all divisors are integers
    divisors = [ int(d) for d in divisors ]
    # compute the ratio of divisors for use in connectivity masking

    mask = None
    relative_divisors = [ divisors[i] / divisors[i-1] for i in range(1, len(divisors)) ]
    for n in range(len(divisors)):
        # run connected components at the new resolution
        (grids, connectivity) = connected_components(scene, scene_tree, float(resolution) / divisors[n], mask)

        if n < len(relative_divisors):
            # only check cells that were occupied last time
            # compute the scaling factor that maps the next round to the mask from this one
            s = relative_divisors[n]
            #mask = lambda i, j, k: connectivity[i / s, j / s, k / s]
            def f(shape):
                (i, j, k) = numpy.indices(shape, numpy.uint) / int(s)
                return connectivity[i, j, k]
            mask = f

    return grids, connectivity, float(resolution) / d

def connected_components(scene, scene_tree, resolution, mask=None):
    print 'connected components at', resolution*100, 'cm'

    # create a sampling space that contains the entire scene
    grids = bounding_grid(scene, resolution)

    # initialize the connectivity matrix
    connectivity = numpy.zeros_like(grids[0], dtype=numpy.bool_)

    if mask is None:
        # provide a complete mask if none specified
        mask = numpy.ones_like(connectivity, dtype=numpy.bool_)
    else:
        # get a mask of the proper shape
        mask = mask(connectivity.shape)

    ind = sample_cloud(scene_tree, grids, mask, resolution)

    # convert the query results into 3D connectivity mask
    connectivity[mask] = (ind < len(scene_tree.data))

    return grids, connectivity

def thin_connectivity(src):
    # the outer rim of the connectivity grid becomes unconnected
    dest = numpy.zeros_like(src)

    # reject connected cells that aren't bounded by neighbors in at least 1 dimension
    # this is in contrast to scipy's binary_erosion which will remove cells not connected in all 3 dimensions
    dest[1:-1,1:-1,1:-1] = src[1:-1,1:-1,1:-1] & ( ( src[2:,1:-1,1:-1] & src[:-2,1:-1,1:-1] ) | ( src[1:-1,2:,1:-1] & src[1:-1,:-2,1:-1] ) | ( src[1:-1,1:-1,2:] & src[1:-1,1:-1,:-2] ) )

    return dest

def sample_cloud(scene_tree, grids, mask, resolution, k=1):
    # compute kdtree query parameters
    max_distance = resolution*2**0.5
    eps = resolution / 10.0

    # format the query points to evaluate them all at once
    sample = numpy.concatenate([ g[...,numpy.newaxis] for g in grids ], axis=3)
    _, ind = scene_tree.query(sample[mask], k=k, eps=eps, distance_upper_bound=max_distance)

    return ind

def clean_cloud(cloud, resolution):
    cloud_tree = scipy.spatial.KDTree(cloud)
    # create a sampling space that contains the entire cloud
    grids = bounding_grid(cloud, resolution)

    # sample with only 1 nearest neighbor allowed
    mask = numpy.ones_like(grids[0], dtype=numpy.bool_)
    ind = sample_cloud(cloud_tree, grids, mask, resolution)

    connectivity = (ind < len(cloud_tree.data))

    # filter out point without nearest neighbors
    return cloud[ind[ind < len(cloud_tree.data)]], connectivity

def bounding_grid(cloud, resolution):
    # find the cloud bounds
    ubounds = numpy.amax(cloud, axis=0)
    lbounds = numpy.amin(cloud, axis=0)
    ranges = [ numpy.arange(l - resolution/2.0, u + resolution, resolution) for (l, u) in zip(lbounds, ubounds) ]

    # make a grid of the request resolution
    # use row-column matrix dimension ordering
    grids = numpy.meshgrid(*ranges, indexing='ij')

    return grids

def extract_scene_candidate(scene_tree, grids, mask, resolution, new_resolution):
    # get the point indices using k=10 to ensure there are enough to find good bounds
    ind = sample_cloud(scene_tree, grids, mask, resolution, k=10)
    # remove missing values and duplicates
    ind2 = numpy.unique(ind[ind < len(scene_tree.data)])
    # retrieve the actual points
    rough_candidate = scene_tree.data[ind2]

    # create a sampling space that contains the candidate at the new resolution
    grids = bounding_grid(rough_candidate, new_resolution)

    # get the point indices
    ind = sample_cloud(scene_tree, grids, None, new_resolution)
    # remove missing values and duplicates
    ind2 = numpy.unique(ind[ind < len(scene_tree.data)])
    # retrieve the actual points
    fine_candidate = scene_tree.data[ind2]

    # convert the query results into 3D connectivity mask
    connectivity = (ind < len(scene_tree.data))

    return fine_candidate, connectivity

def match_candidate(obj_orig, candidate_tree, iterations=20):
    obj_orig = numpy.array(obj_orig)

    total_t = numpy.zeros((1,3))
    total_R = numpy.identity(3)
    last_error = float('inf')

    # copy the points since they get transformed
    obj = numpy.copy(obj_orig)
    #matplot_points(candidate_tree.data, obj)

    for n in range(iterations):
        # get correspondences using distance
        if n == 0:
            # run the first query with infinite distance range in case the object is distant
            threshold = float('inf')
        else:
            # apply the distance threshold for subsequent iterations
            # reduce threshold as iterations progress
            # this affectively "anneals" the object to the scene candidate
            threshold = 0.05*(5/float(n)) + 0.0001

        _, ind = candidate_tree.query(obj, distance_upper_bound=threshold)

        # filtered out point without correspondences
        mask = ind < len(candidate_tree.data)
        obj = obj[mask]
        correspondences = candidate_tree.data[ind[mask]]

        if len(obj) == 0:
            print 'no correspondences...'
            break

        # compute minimizing translation
        obj_mean = obj.mean(axis=0)
        correspondences_mean = correspondences.mean(axis=0)

        # demean the object and the correspondences
        obj -= obj_mean
        correspondences -= correspondences_mean

        # compute the covariance matrix
        H = [ [ obj[:,i].dot(correspondences[:,j]) for j in range(3) ] for i in range(3) ]

        # use SVD for the minimizing rotation
        (U, S, V) = numpy.linalg.svd(H)
        # this works instead of numpy.dot(U, V.T) for some reason
        R = numpy.dot(U, V)

        # check which rotation is minimizing
        sse_normal = ((correspondences - obj.dot(R.T))**2).sum()
        sse_transpose = ((correspondences - obj.dot(R))**2).sum()

        # compute translations and rotations depending upon which of R or R.T minimizes SSE
        if sse_normal < sse_transpose:
            total_t = total_t.dot(R.T) - obj_mean.dot(R.T) + correspondences_mean
            total_R = R.dot(total_R)
            last_error = sse_normal
        else:
            total_t = total_t.dot(R) - obj_mean.dot(R) + correspondences_mean
            total_R = R.T.dot(total_R)
            last_error = sse_transpose

        # transform the object from the original for the next round
        obj = obj_orig.dot(total_R.T) + total_t
        print n, threshold, len(correspondences), 1e9*last_error / len(correspondences)**2

    # compute the goodness of fit
    obj_tree = scipy.spatial.KDTree(obj)
    fit_metric = compute_fit_metric(obj_tree, candidate_tree)

    #matplot_points(candidate_tree.data, obj)
    return total_R, total_t, fit_metric

def compute_fit_metric(obj_tree, candidate_tree):
    # how well does the object fit the candidate?
    # ideally every point in the candidate is fit
    # find the nearest neighbor of all candidate points and compute SSE
    _, ind = obj_tree.query(candidate_tree.data)
    # compute the SSE of all candidate points weighted by the number of candidate points
    # square the number of candidate points since the error term itself is squared
    sse_term = ((candidate_tree.data - obj_tree.data[ind])**2).sum() / len(candidate_tree.data)**2

    # how well does the candidate fit the object?
    # the candidate should "nicely" fit a substantial portion of the object
    # so find how many object points are close to a candidate point
    _, ind = candidate_tree.query(obj_tree.data, distance_upper_bound=0.005)
    unfit_term = numpy.count_nonzero(ind == len(candidate_tree.data))

    # scale by 10 so the "goodness" threshold can be 1
    # square unfit_term to bias towards fitting more points rather than minimizing SSE
    fit_metric = 10 * sse_term * unfit_term**2
    print 'fit metric', sse_term, unfit_term, fit_metric

    return fit_metric

def icp(object,scene,reuse=True):
    """Computes a rotation and translation of the object to the scene
    using the ICP algorithm.  Returns the transform (R,t) in Klamp't se3
    format.
    """

    extract_resolution = 0.01
    (obj, obj_connectivity) = clean_cloud(numpy.array(object), 0.01)
    print 'sampled object', 100*extract_resolution, 'cm:', len(object), '->', len(obj)

    # get the object properties
    obj_dims = numpy.amax(object, axis=0) - numpy.amin(object, axis=0)
    obj_volume = numpy.count_nonzero(obj_connectivity) * extract_resolution**3
    print 'object', '%.1fx%.1fx%.1f' % tuple(100*obj_dims), 'cm', '(%.1f' % (100**3*obj_volume,), 'cm^3)'

    # this caching code is to make detect_3d run much faster
    # otherwise it would need to run connected components each time which is slow
    global cached
    if reuse and cached:
        (scene, scene_tree, grids, scene_connectivity, resolution) = cached
    else:
        # fast point cloud lookups
        print 'generate scene kdtree'
        scene = numpy.array(scene)
        scene_tree = scipy.spatial.KDTree(scene[::])

        # run connected components
        (grids, scene_connectivity, resolution) = multiscale_connected_components(scene, scene_tree, 0.10, [1, 4, 8, 16])

        # cache the results
        cached = (scene, scene_tree, grids, scene_connectivity, resolution)

    inflated_resolution = resolution*2**0.5

    # thin the connectivity grids
    # this is needed to separate the kygen_eggs and kong_duck from the table
    # unfortunately, it also destroys the highlighters...
    for i in range(6):
        scene_connectivity = thin_connectivity(scene_connectivity)

    labeled, label_count = scipy.ndimage.label(scene_connectivity)

    candidates = []

    # compute candidate coarse bounds
    index_bounds = scipy.ndimage.find_objects(labeled)
    for label in range(label_count):
        ib = index_bounds[label]
        lbounds = numpy.array([ grids[i][ib[0].start, ib[1].start, ib[2].start] for i in range(3) ]) - inflated_resolution
        ubounds = numpy.array([ grids[i][ib[0].stop-1, ib[1].stop-1, ib[2].stop-1] for i in range(3) ]) + inflated_resolution

        # compute rough dimensions and volume
        dim = ubounds - lbounds
        rough_volume = numpy.count_nonzero(labeled == label) * resolution**3

        candidates.append({ 'label': label+1,
                            'rough_bounds': numpy.vstack((lbounds, ubounds)),
                            'rough_volume': rough_volume
                          })

        print 'found candidate', '~%.1fx%.1fx%.1f' % tuple(100*dim), 'cm', '(~%.1f' % (100**3*rough_volume,), 'cm^3)'

    if len(candidates) == 0:
        print 'no candidates found!'
        return se3.identity, float('inf')

    # sort candidates such that the one with estimated volume closest to the target object is matched first
    candidates.sort(key=lambda c: abs(c['rough_volume'] - obj_volume))

    # extra candidate point clouds and get finer bounds
    for candidate in candidates:
        # reject large candidates before extraction to save time
        if candidate['rough_volume'] > obj_volume * 10:
            print 'candidate too large for good match -- skipping'
            continue

        (candidate['points'], candidate['connectivity']) = extract_scene_candidate(scene_tree, grids, labeled == candidate['label'], resolution, extract_resolution)

        ubounds = numpy.amax(candidate['points'], axis=0)
        lbounds = numpy.amin(candidate['points'], axis=0)

        # compute fine dimensions and volume
        dim = ubounds - lbounds
        fine_volume = numpy.count_nonzero(candidate['connectivity']) * extract_resolution**3

        candidate['fine_bounds'] = numpy.vstack((lbounds, ubounds))
        candidate['fine_volume'] = fine_volume

        print 'extracted candidate', 100*extract_resolution, 'cm:', '%.1fx%.1fx%.1f' % tuple(100*dim), 'cm', '(%.1f' % (100**3*fine_volume,), 'cm^3)'

        if fine_volume < obj_volume / 10:
            print 'candidate too small for good match -- skipping'
            continue

        # time to attempt a match to this candidate
        R, t, metric = match_candidate(obj, scipy.spatial.KDTree(candidate['points']), 100)
        # save results for later
        candidate.update({ 'transform': (R.T.flat[:], t[0]), # needed output format
                           'metric': metric
                         })
        if metric < 1:
            print 'good enough fit found:', metric
            return candidate['transform'], metric #[ c.get('fine_bounds', c['rough_bounds']) for c in candidates ]
        else:
            print 'candidate is poor fit -- trying another'


    # no good candidates found so just pick the one with the best metric
    candidates.sort(key=lambda c: c.get('metric', float('inf')))
    best_candidate = candidates[0]
    print 'best fit:', best_candidate.get('metric', float('inf'))

    return best_candidate.get('transform', se3.identity()), best_candidate.get('metric', float('inf'))#, [ c.get('fine_bounds', c['rough_bounds']) for c in candidates ]

def main():
    """
    Main loop.  Run ICP on the given model and scene file, display the results.
    """
    model_full = get_reconstructed_model(model_file,True)
    scene_full = get_raw_depth(depth_file)

    #run the ICP
    (R, t), _ = icp(model_full['positions'],scene_full)

    #apply the ICP results to the model
    transformed_points = [se3.apply((R,t),p) for p in model_full['positions']]
    model_full['positions'] = transformed_points

    #visualize the results
    opengl_plot = OpenGLPlot(model_full, scene_full)
    opengl_plot.initialize_main_loop()

    # If matplot is available you can use it to visualize the points (but it not
    # required) as in the commented line
    # matplot_points(model_full, clpoints_out)

if __name__ == "__main__":
    main()
