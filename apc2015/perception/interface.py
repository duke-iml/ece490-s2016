import logging, traceback
logger = logging.getLogger(__name__)

import subprocess, os, json
import numpy

from time import time
from klampt.robotsim import WorldModel
from klampt import se3, so3, vectorops, resource

import apc.baxter_scoop as baxter
from api.perception import PerceptionInterface

from integration.visualization import debug_cloud   
from integration.camera import RemoteCamera, invalid_mask
from integration.io import pcd

from perception.segmentation.shelf import shelf_subtract
from perception.segmentation.blob import distance_label
from perception.segmentation.color import rgb2yuv, make_uv_hist
from perception.segmentation.xyzcolor import xyzrgb_segment_kmeans
from perception.segmentation.template import match_rollodex_template

base_path = 'perception'

class RealPerceptionInterface(PerceptionInterface):
    def __init__(self, knowledge_base):
        PerceptionInterface.__init__(self, knowledge_base)

        world = WorldModel()
        robot = world.loadRobot(os.path.join('klampt_models', baxter.klampt_model_name))
        robot.setConfig(self.knowledge_base.robot_state.sensed_config)

        self.base_xform = robot.getLink('{}_gripper'.format(self.knowledge_base.active_limb)).getTransform()
        if self.knowledge_base.active_limb == 'left':
            self.camera_xform = se3.mul(self.base_xform, self.knowledge_base.left_camera_offset_xform)
        else:
            self.camera_xform = se3.mul(self.base_xform, self.knowledge_base.right_camera_offset_xform)

    # def _localize_task(self, name, args=None, skip_cloud=False):
    #     args = args or []

    #     # get the full executable path
    #     exe = os.path.join(os.path.abspath(base_path), 'localize{}.py'.format(name))
    #     logger.info('running {} {}'.format(exe, args))

    #     # start the task
    #     child = subprocess.Popen(
    #         [ exe ] + args,
    #         stdout=subprocess.PIPE,
    #         stderr=subprocess.PIPE
    #     )
    #     (output, error) = child.communicate()
    #     # output = open('/tmp/output.txt').read()
    #     # error = None
    #     # open('/tmp/output.txt', 'w').write(output)
    #     # logging.debug('child stdout: {}'.format(output))
    #     # if error:
    #         # logging.warn('child stderr: {}'.format(error))

    #     logger.debug('child {} finished'.format(child.pid))

    #     # check for error by return code
    #     if child.returncode == 0:
    #         # output format is first line xform as 4x4 flat array
    #         # remaining lines are PCL point cloud
    #         idx = output.index('\n')

    #         # parse the first line
    #         line = output[:idx].strip()
    #         xform_matrix = array(map(float, line.split())).reshape((4,4))
    #         xform = (list(xform_matrix[:3, :3].T.flat), list(xform_matrix[:3, 3].flat))

    #         if skip_cloud:
    #             cloud = []
    #         else:
    #             # parse the PCL point cloud
    #             (_, cloud) = pcd.parse(StringIO(output[idx+1:]))
        
    #         return (xform, cloud)
    #     else:
    #         # throw an exception with the error text
    #         raise Exception(error)

    # def _find_task(self, name, args):
    #     raise NotImplemented

    def localizeShelf(self):
    										#distance away
    	shelf_xform = ([0,1,0,-1,0,0,0,0,1],[1.1,-0.055,-.9])
    	resource.setDirectory("resources/")
    	return resource.get("calibrated_shelf.xform",type="RigidTransform",default=shelf_xform,doedit=False)


        T = numpy.array([[ 0, -1, 0,    1.1 ],  # distance from front of robot
                         [ 1,  0, 0,  -0.055 ],  # left/right
                         [ 0,  0, 1, -0.9 ],  # height
                         [ 0,  0, 0,       1 ]])

        return (list(T[:3, :3].T.flat), list(T[:3, 3].flat))

    def localizeOrderBin(self):
    	order_bin_xform = (so3.identity(),[0.57,0,-0.87])
    	resource.setDirectory("resources/")
    	return resource.get("calibrated_order_bin.xform",type="RigidTransform",default=order_bin_xform,doedit=False)

        T = numpy.array([[ 1, 0, 0,   0.57 ],
                         [ 0, 1, 0,     0 ],
                         [ 0, 0, 1, -0.87 ],
                         [ 0, 0, 0,     1 ]])

        return (list(T[:3, :3].T.flat), list(T[:3, 3].flat))        

    def findAllObjects(self, bin):
        raise NotImplemented
    
    def _localizeRollodexCups(self, world, color, clean_cloud_aligned, depth_uv, clean_mask, object_mask):    
        # copy the uv map because it is modified below
        # also flip x/y since numpy is row/column
        uv = depth_uv[:,:,::-1].copy()

        # scale the uv coordinates by the source image size
        for i in range(2):
            uv[:,:,i] = numpy.clip(uv[:,:,i] * color.shape[i], 0, color.shape[i]-1)

        # round to the nearest pixel
        uv = uv.round().astype(numpy.int)

        # filter the uv mapping using the bounding box coordinates
        ((bb_top, bb_left), (bb_bottom, bb_right)) = match_rollodex_template(color)
        from scipy.misc import imsave
        imsave('/tmp/cups.png', color[bb_top:bb_bottom, bb_left:bb_right])
        
        logger.debug('rollodex cups template match bounding box: {}'.format(((bb_top, bb_left), (bb_bottom, bb_right))))
        cups_mask = reduce(numpy.bitwise_and, [ uv[:,:,0] > bb_top, uv[:,:,0] < bb_bottom, uv[:,:,1] > bb_left, uv[:,:,1] < bb_right ])
        #imsave('/tmp/cups_uv.png', uv)
        imsave('/tmp/cups_mask.png', cups_mask)
        
        valid_depth_pixels = numpy.count_nonzero(cups_mask)
        if valid_depth_pixels == 0:
            logger.error('rollodex cups template has no valid depth pixels')
            return None
            
        logger.debug('rollodex cups template has {} valid depth pixels'.format(valid_depth_pixels))
        
        # get the point cloud corresponding to the cups bounding box
        mask = object_mask & cups_mask
        cups_cloud = clean_cloud_aligned[mask[clean_mask]]
        
        cups_front = cups_cloud[:,0].min()
        cups_mean = cups_cloud.mean(axis=0)
        
        known_cups_width = 0.10 - 0.02
        known_cups_height = 0.13 - 0.02
                
        # generate a fake point cloud for the cups
        n = 32
        fake_cloud = numpy.dstack((
            cups_front * numpy.ones((n, n)),           
            numpy.dstack(numpy.meshgrid(
                known_cups_width * numpy.linspace(-0.5, 0.5, n) + cups_mean[1],
                known_cups_height * numpy.linspace(-0.5, 0.5, n) + cups_mean[2],
                indexing='ij'
            ))
        ))
        
        xform = se3.identity()
        object_cloud = fake_cloud.reshape((-1, 3))
                
        (target_bin, target_object) = (self.knowledge_base.target_bin, self.knowledge_base.target_object)
        bin_contents = self.knowledge_base.bin_contents[target_bin] if target_bin in self.knowledge_base.bin_contents else []
        
        confusion_row = dict([ (obj, [0, 1][obj == 'rollodex_mesh_collection_jumbo_pencil_cup']) for obj in bin_contents ])
        logger.info('confusion row for {}: {}'.format(target_object, confusion_row))
        
        object_cloud_color = [ (p + [ [0, 0, 0] ]) for p in object_cloud.tolist() ]

        #debug_cloud([ object_cloud_color ], [ (so3.identity(), cups_mean) ], world=world)
        pcd.write(object_cloud_color, open('/tmp/{}_{}.pcd'.format(target_object, target_bin), 'w'))
        
        return (xform, object_cloud_color, confusion_row)
    
    def localizeSpecificObject(self, bin, object):
        SHELF_DIMS_PATH = os.path.join('kb', 'shelf_dims.json')
        SHELF_CLOUD_PATH = os.path.join('perception', 'segmentation', 'models', 'shelf_thin_10000.npy')
    
        # acquire the camera image
        address = { 'left': '192.168.0.103', 'right': '192.168.0.104' }[self.knowledge_base.active_limb]
        #address = { 'left': '192.168.0.103', 'right': '192.168.0.103' }[self.knowledge_base.active_limb]
        camera = RemoteCamera(address, xform=self.camera_xform)
        camera.read()
        camera.close()

        # load the shelf subtraction data
        bin_bounds = json.load(open(SHELF_DIMS_PATH))[bin]
        # load and transform the shelf point cloud
        shelf_cloud = numpy.load(open(SHELF_CLOUD_PATH))
        shelf_cloud = shelf_cloud.dot(numpy.array(self.knowledge_base.shelf_xform[0]).reshape((3,3))) + self.knowledge_base.shelf_xform[1]
        # load and transform the shelf model
        world = WorldModel()
        shelf = world.loadRigidObject(os.path.join('klampt_models', 'north_shelf', 'shelf_with_bins.obj'))
        shelf.setTransform(*self.knowledge_base.shelf_xform)

        # clean up the point cloud
        (clean_mask, clean_cloud) = camera.clean()
        _, clean_cloud_aligned, _, clean_object_mask = shelf_subtract(shelf_cloud, clean_cloud, shelf, bin_bounds, downsample=1000)
        object_mask = numpy.zeros(camera.cloud.shape[:2], dtype=numpy.bool)
        object_mask[clean_mask] = clean_object_mask

        #clean_cloud_aligned_color = map(lambda x: x[0] + [ x[1] ], zip(clean_cloud_aligned[clean_object_mask].tolist(), camera.colorize()[clean_mask][clean_object_mask].tolist()))
        #debug_cloud([ clean_cloud_aligned_color ], [ self.base_xform, self.camera_xform ])      
        #debug_cloud([ shelf_cloud, clean_cloud, clean_cloud_aligned ])
        #pcd.write(clean_cloud_aligned_color, open('/tmp/{}_{}.pcd'.format(self.knowledge_base.target_object, self.knowledge_base.target_bin), 'w'))

        (target_bin, target_object) = (self.knowledge_base.target_bin, self.knowledge_base.target_object)
        bin_contents = self.knowledge_base.bin_contents[target_bin] if target_bin in self.knowledge_base.bin_contents else []

        # perform special handling
        if target_object == 'rollodex_mesh_collection_jumbo_pencil_cup':
            logger.info('running specialized rollodex cups detection')
            return self._localizeRollodexCups(world, camera.color, clean_cloud_aligned, camera.depth_uv, clean_mask, object_mask)

        # dilate the object mask
        # from scipy.misc import imsave
        from scipy.ndimage.morphology import binary_dilation
        object_mask_dilated = binary_dilation(object_mask, iterations=2)
        # imsave('/tmp/test.png', object_mask)

        # label connected components
        (object_labeled, label_count) = distance_label(camera.cloud, object_mask_dilated, threshold=0.01)
        logger.info('found {} connected components'.format(label_count))       

        #KH: added to help debugging without changing order bin contents
        if object not in bin_contents:
            logger.warning("Warning, trying to detect object "+object+" not in bin contents, adding a temporary entry")
            bin_contents = bin_contents + [object]

        bin_contents_with_shelf = bin_contents + [ 'shelf' ]
        
        if not label_count:
            logger.warning('found no connected components')
            mask = object_mask
    
            bin_contents = self.knowledge_base.bin_contents[self.knowledge_base.target_bin]
            confusion_row = dict(zip(bin_contents, [0]*len(bin_contents)))
        else:
            object_blobs = sorted([ object_labeled == (l+1) for l in range(label_count) ], key=lambda b: -numpy.count_nonzero(b))
            #for (i, blob) in enumerate(object_blobs):
            #    logger.debug('blob {}: {} points'.format(i, numpy.count_nonzero(blob)))

            # filter out blobs with fewer than 1000 points
            min_point_count = 1000
            object_blobs = filter(lambda blob: numpy.count_nonzero(blob[clean_mask]) >= min_point_count, object_blobs)
                
            #debug_cloud([ clean_cloud_aligned[(object_mask & blob_mask)[clean_mask]] for blob_mask in object_blobs ], [ self.base_xform, self.camera_xform ], wait=False) 

            known_object_count = len(bin_contents)
            if known_object_count == 0:
                # return all large blobs
                n = len(object_blobs)
    
                # take only the largest n blobs
                blob_mask = reduce(numpy.bitwise_or, object_blobs[:n])
                mask = object_mask & blob_mask
                
                logger.warning('no objects in bin... -> returning all blobs')
            else:
                # load the matching statistics
                match_stats = json.load(open(os.path.join('perception', 'segmentation', 'match_stats.json')))            
                
                # load the object hue histograms
                known_histograms = {}
                for obj in bin_contents_with_shelf:
                    #array = numpy.load(os.path.join('perception', 'hue_array', '{}.npz'.format(obj)))['arr_0']
                    #known_histograms[obj] = numpy.histogram(array, bins=range(0,181), density=True)[0].reshape(-1,1).astype(numpy.float32)
                    known_histograms[obj] = numpy.load(os.path.join('perception', 'uv_hist2', '{}.npz'.format(obj)))['arr_0']
                                    
                # compute the color validity mask
                color_valid_mask = ~invalid_mask(camera.depth_uv)

                #KH addition 5/23: test to see whether blobs should be broken up
                if len(object_blobs)==1 and len(known_histograms) > 2:
                    logger.info("Trying KMeans segmentation to break up large blob")
                    blob_mask = reduce(numpy.bitwise_or, object_blobs)
                    mask = object_mask & blob_mask & color_valid_mask
                    labels,k,score = xyzrgb_segment_kmeans(camera.cloud[mask],camera.colorize()[mask],known_histograms.values())
                    labels = numpy.array(labels)
                    object_blobs = []
                    for i in xrange(len(known_histograms)):
                        blank = numpy.zeros_like(object_mask,dtype=numpy.bool)
                        blank[mask] = (labels==i)
                        object_blobs.append(blank)

                # score each blob for each object in the bin 
                blob_scores = []
                blob_accepts = []
                blob_rejects = []
                matched_blobs = {}
                confusion_matrix = []
                for (i, blob) in enumerate(object_blobs):
                    # compute the blob histogram
                    blob_uv = [ rgb2yuv(*rgb)[1:3] for rgb in camera.colorize()[object_mask & blob & color_valid_mask] ]
                    blob_histogram = make_uv_hist(blob_uv)
              
                    # compare the blob to each possible object                    
                    scores = dict([ (obj, 2 * numpy.minimum(blob_histogram, histogram).sum() - numpy.maximum(blob_histogram, histogram).sum()) for (obj, histogram) in known_histograms.items() ])                
                    
                    logger.debug('blob {}:'.format(i))
                    
                    blob_scores.append(scores)
                    logger.debug('  scores: {}'.format([ '{}={}'.format(*x) for x in scores.items() ]))
                    
                    # apply the cutoff to each score and associate with positive confidence
                    accepts = dict([ (obj, match_stats[obj]['ppv']) for obj in bin_contents_with_shelf if scores[obj] >= match_stats[obj]['cutoff'] ])
                    blob_accepts.append(accepts)
                    logger.debug('  accepts: {}'.format([ '{}={}'.format(*x) for x in accepts.items() ]))
                    
                    # apply the cutoff to each score and associate with negative confidence
                    rejects = dict([ (obj, match_stats[obj]['npv']) for obj in bin_contents_with_shelf if scores[obj] < match_stats[obj]['cutoff'] ])
                    blob_rejects.append(rejects)                    
                    logger.debug('  rejects: {}'.format([ '{}={}'.format(*x) for x in rejects.items() ]))

                    # populate the confusion matrix
                    shelf_probability = 0.1
                    S = lambda o: match_stats[o]['specificity']
                    iS = lambda o: 1 - S(o)
                    total_probability = sum(map(S, accepts)) + sum(map(iS, rejects)) + shelf_probability
                    confusion_matrix.append([ [ iS(o), S(o) ][ o in accepts ] / total_probability for o in bin_contents ])

                    # resolve multiple assignments
                    if len(accepts) > 1:
                        # choose the object with the highest matched confidence    
                        best_match_object = sorted(accepts.items(), key=lambda a: -a[1])[0][0]
                        # remove all other objects
                        for (obj, confidence) in accepts.items():
                            if obj != best_match_object:
                                del accepts[obj]
                                logger.warn('blob {} multiple accept ignored: {}={}'.format(i, obj, confidence))
                    
                    if len(accepts) == 1:
                        # a single match so record it
                        matched_blobs.setdefault(accepts.keys()[0], []).append(i)
                        logger.info('blob {} assigned to {} by single match'.format(i, accepts.keys()[0]))
                    
                confusion_matrix = numpy.array(confusion_matrix)
                logger.debug('confusion matrix:\n{}'.format(numpy.array(confusion_matrix)))
                    
                # assign blobs by least threshold difference until all objects are assigned (excluding shelf) or all blobs are assigned
                while len([ k for k in matched_blobs.keys() if k != 'shelf']) < len(bin_contents) and len(sum(matched_blobs.values(), [])) < len(object_blobs):
                    best_match_objects = []
                
                    for (i, scores) in enumerate(blob_scores):
                        # only consider unmatched blobs
                        if i not in sum(matched_blobs.values(), []):
                            threshold_differences = [ [obj, match_stats[obj]['cutoff'] - score]  for (obj, score) in scores.items() ]
                            best_match_objects.append([ i ] + sorted(threshold_differences, key=lambda d: d[1])[0])
                            
                    # choose the least threshold difference across all blobs and objects
                    best_blob, best_object, _ = sorted(best_match_objects, key=lambda b: b[2])[0]
                    matched_blobs.setdefault(best_object, []).append(best_blob)                    
                    logger.warn('blob {} assigned to {} by least threshold difference'.format(best_blob, best_object))
                
                # report unmatched blobs and objects (excluding shelf)
                for obj in bin_contents:
                    if obj not in matched_blobs:
                        logger.warn('no blobs matched {}'.format(obj))
                for i in range(len(object_blobs)):
                    if i not in sum(matched_blobs.values(), []):
                        logger.warn('blob {} is unassigned'.format(i))
                
                for obj in bin_contents_with_shelf:
                    if obj in matched_blobs:
                        print obj
                        mask = reduce(numpy.bitwise_or, [ object_blobs[i] for i in matched_blobs[obj] ], numpy.zeros_like(object_mask))
                        object_cloud = clean_cloud_aligned[mask[clean_mask]]
                        xform = (so3.identity(), object_cloud.mean(axis=0).tolist())
                        object_cloud_color = map(lambda x: x[0] + [ x[1] ], zip(object_cloud.tolist(), camera.colorize()[mask].tolist()))
                        #debug_cloud([ object_cloud_color ], [ self.base_xform, self.camera_xform, xform ])      
                
                # check that the target object was found
                if target_object not in matched_blobs:
                    logger.error('no blobs assigned to target object')
                    return None
                
                # return blob for the selected object
                mask = reduce(numpy.bitwise_or, [ object_blobs[i] for i in matched_blobs[target_object] ], numpy.zeros_like(object_mask))

                # return the confusion matrix rows for the selected object
                confusion_rows = confusion_matrix[matched_blobs[target_object], :]
                confusion_row = dict([ (obj, confusion_rows[:, i].mean()) for (i, obj) in enumerate(bin_contents) ])
                logger.info('confusion row for {}: {}'.format(target_object, confusion_row))

        object_cloud = clean_cloud_aligned[mask[clean_mask]]
        
        #xform = (so3.identity(), object_cloud.mean(axis=0).tolist())
        xform = se3.identity()
        object_cloud_color = map(lambda x: x[0] + [ x[1] ], zip(object_cloud.tolist(), camera.colorize()[mask].tolist()))
        #debug_cloud([ object_cloud_color ], [ self.base_xform, self.camera_xform, xform ])      
        
        # downsample the cloud to about 1000 points
        ratio = int(len(object_cloud) / 1000)
        if ratio > 1:
            object_cloud_color = object_cloud_color[::ratio]
        
        pcd.write(object_cloud_color, open('/tmp/{}_{}.pcd'.format(self.knowledge_base.target_object, self.knowledge_base.target_bin), 'w'))

        return (xform, object_cloud_color, confusion_row)

    def localizeSpecificObject_old(self, bin, object):
        (xform, cloud) = self._localize_task('SpecificObject', [ bin, object ])

        # apply the camera transform
        xform = se3.mul(self.camera_xform, xform)
        cloud = map(lambda p: se3.apply(self.camera_xform, p[:3]) + [ p[3] ], cloud)

        debug_cloud(cloud, [ self.base_xform, self.camera_xform, xform ])

        return (xform, cloud)

