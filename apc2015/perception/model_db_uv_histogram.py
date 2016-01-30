import logging, traceback, sys
from rainbow_logging_handler import RainbowLoggingHandler
#logging.basicConfig(level=logging.DEBUG, format='%(filename)-15s:%(lineno)-4d %(levelname)s: %(message)s')
handler = RainbowLoggingHandler(sys.stderr)
handler.setFormatter(logging.Formatter('%(filename)s:%(lineno)d\t%(levelname)s: %(message)s'))
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
logger.addHandler(handler)

# allow importing from the repository root
import sys, os, json
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import apc.baxter_scoop as baxter
from integration.visualization import debug_cloud
from integration.camera import uvtexture, invalid_mask
from integration.io import pcd
from perception.segmentation.color import rgb2yuv, make_uv_hist

from klampt.robotsim import WorldModel
from klampt import se3,so3
from time import sleep, time
import numpy

def load_histogram(obj, n):
    hist_path = os.path.join(base_path, '{}_{}_hist.npy'.format(obj, n))
    
    if os.path.exists(hist_path):
        blob_histogram = numpy.load(hist_path)    
    else:
        color = numpy.load(os.path.join(base_path, '{}_{}_color.npy'.format(obj, n)))
        depth_uv = numpy.load(os.path.join(base_path, '{}_{}_depth_uv.npy'.format(obj, n)))

        if obj == 'shelf':
            mask = numpy.ones((color.shape[0], color.shape[1]), dtype=numpy.bool)
        else:
            mask = numpy.load(os.path.join(base_path, '{}_{}_mask.npy'.format(obj, n)))
       
        color2 = uvtexture(color, depth_uv)
        color_valid_mask = ~invalid_mask(depth_uv)
        blob_uv = [ rgb_to_yuv(*rgb)[1:3] for rgb in color2[mask & color_valid_mask] ]
        blob_histogram = make_uv_hist(blob_uv)
        
        numpy.save(hist_path, blob_histogram)

    return blob_histogram

base_path = os.path.join('perception', 'models_db2')

objects = json.load(open(os.path.join('perception', 'segmentation', 'match_stats.json'))).keys()

for obj in objects:
    print 'swtich to', obj

    histograms = []
    
    for n in range(6):
        print 'take', n,
               
        color = numpy.load(os.path.join(base_path, '{}_{}_color.npy'.format(obj, n)))
        depth_uv = numpy.load(os.path.join(base_path, '{}_{}_depth_uv.npy'.format(obj, n)))
        if obj == 'shelf':
            mask = numpy.ones((color.shape[0], color.shape[1]), dtype=numpy.bool)
        else:
            mask = numpy.load(os.path.join(base_path, '{}_{}_mask.npy'.format(obj, n)))
       
        color2 = uvtexture(color, depth_uv)
        color_valid_mask = ~invalid_mask(depth_uv)
        blob_uv = [ rgb2yuv(*rgb)[1:3] for rgb in color2[mask & color_valid_mask] ]

        if blob_uv:
            blob_histogram = make_uv_hist(blob_uv)
            histograms.append(blob_histogram)

            hist_path = os.path.join(base_path, '{}_{}_hist.npy'.format(obj, n))
            numpy.save(hist_path, blob_histogram)
            print 'histogram'
        else:
            print 'skipped'
    
    if histograms:
        composite_histogram = sum(histograms) / len(histograms)
        print composite_histogram
    else:
        composite_histogram = -1e3 * numpy.ones((64, 64))
        print 'empty'
        
    numpy.savez_compressed('perception/uv_hist2/{}.npz'.format(obj), composite_histogram)
               
print 'done'
