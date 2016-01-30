import logging, traceback
logging.basicConfig(level=logging.DEBUG, format='%(filename)-15s:%(lineno)-4d %(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

# allow importing from the repository root
import sys, os, json
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import apc.baxter_scoop as baxter
from integration.visualization import debug_cloud
from integration.camera import uvtexture, invalid_mask
from integration.io import pcd
from perception.segmentation.shelf import shelf_subtract
from perception.segmentation.blob import distance_label

from klampt.robotsim import WorldModel
from klampt import se3,so3
from time import sleep, time
import numpy

limb = 'left'
bin = 'bin_A'
base_path = os.path.join('perception', 'models_db2')

objects = json.load(open(os.path.join('perception', 'null_match_values.json'))).keys()

SHELF_DIMS_PATH = os.path.join('perception', 'shelf_subtract', 'shelf_dims.json')
SHELF_CLOUD_PATH = os.path.join('perception', 'shelf_subtract', 'shelf_10000.npy')

shelf_xform = (
    numpy.array([[  0,  1,  0 ],
                 [ -1,  0,  0 ],
                 [  0,  0,  1 ]]),
    [ 1.03, -0.055, -0.9046 ]
)

 # load the shelf subtraction data
bin_bounds = json.load(open(SHELF_DIMS_PATH))[bin]
# load and transform the shelf point cloud
shelf_cloud = numpy.load(open(SHELF_CLOUD_PATH))
shelf_cloud = shelf_cloud.dot(shelf_xform[0]) + shelf_xform[1]

# load and transform the shelf model
world = WorldModel()
shelf = world.loadRigidObject('klampt_models/north_shelf/shelf_with_bins.obj')
shelf_xform[1][2] -= 0.01
shelf.setTransform(list(shelf_xform[0].flat), shelf_xform[1])

for obj in objects:
    print 'swtich to', obj
    
    for n in range(6):
        print 'take', n,

        color = numpy.load(os.path.join(base_path, '{}_{}_color.npy'.format(obj, n)))
        depth_uv = numpy.load(os.path.join(base_path, '{}_{}_depth_uv.npy'.format(obj, n)))
        cloud = numpy.load(os.path.join(base_path, '{}_{}_cloud.npy'.format(obj, n)))
       
        # clean up the point cloud
        clean_mask = invalid_mask(cloud)
        clean_cloud = cloud[clean_mask]
        _, clean_cloud_aligned, _, clean_object_mask = shelf_subtract(shelf_cloud, clean_cloud, shelf, bin_bounds, downsample=1000)
        object_mask = numpy.zeros(cloud.shape[:2], dtype=numpy.bool)
        object_mask[clean_mask] = clean_object_mask

        clean_cloud_aligned_color = map(lambda x: x[0] + [ x[1] ], zip(clean_cloud_aligned[clean_object_mask].tolist(), uvtexture(color, depth_uv)[clean_mask][clean_object_mask].tolist()))
        #debug_cloud([ clean_cloud_aligned_color ], world=world)      
        #debug_cloud([ shelf_cloud, clean_cloud, clean_cloud_aligned ])
        # pcd.write(clean_cloud_aligned_color, open('/tmp/{}_{}.pcd'.format(self.knowledge_base.target_object, self.knowledge_base.target_bin), 'w'))

        # dilate the object mask
        from scipy.misc import imsave
        from scipy.ndimage.morphology import binary_dilation
        object_mask_dilated = binary_dilation(object_mask, iterations=2)

        # label connected components
        from scipy.ndimage.measurements import label
        #(object_labeled, label_count) = label(object_mask_dilated)
        (object_labeled, label_count) = distance_label(cloud, object_mask_dilated)
        logger.info('found {} connected components'.format(label_count))       

        if not label_count:
            logger.warning('found no connected components')
            mask = object_mask
        else:
            object_blobs = sorted([ object_labeled == (l+1) for l in range(label_count) ], key=lambda b: -numpy.count_nonzero(b))
            # for (i, blob) in enumerate(object_blobs):
                # logger.info('blob {}: {} points'.format(i, numpy.count_nonzero(blob)))

            # filter out blobs with fewer than 1000 points
            object_blobs = filter(lambda blob: numpy.count_nonzero(blob[clean_mask]) >= 1000, object_blobs)
                
            # take only the largest blobs
            if object_blobs:
                blob_mask = reduce(numpy.bitwise_or, object_blobs)
                mask = object_mask & blob_mask
            else:
                mask = numpy.zeros_like(object_mask)
        
        object_cloud = clean_cloud_aligned[mask[clean_mask]]    
        xform = (so3.identity(), object_cloud.mean(axis=0).tolist())
        object_cloud_color = map(lambda x: x[0] + [ x[1] ], zip(object_cloud.tolist(), uvtexture(color, depth_uv)[mask].tolist()))
        try:
            pcd.write(object_cloud_color, os.path.join(base_path, '{}_{}_segmented.pcd'.format(obj, n)))
        except IndexError:
            print 'error writing PCD'
            pass
        numpy.save(os.path.join(base_path, '{}_{}_mask.npy'.format(obj, n)), mask)
        imsave(os.path.join(base_path, '{}_{}_mask.png'.format(obj, n)), mask)
        imsave(os.path.join(base_path, '{}_{}_color.png'.format(obj, n)), color)
        color2 = uvtexture(color, depth_uv)
        color2[~mask] = [0, 0, 0]
        imsave(os.path.join(base_path, '{}_{}_segmented.png'.format(obj, n)), color2)
                
        print 'segmented'
        #object_cloud_color = map(lambda x: x[0] + [ x[1] ], zip(10*(object_cloud - object_cloud.mean(axis=0)).tolist(), uvtexture(color, depth_uv)[mask].tolist()))
        #debug_cloud([ object_cloud_color ], [ xform ])
               
print 'done'
