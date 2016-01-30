import logging, traceback, sys
from rainbow_logging_handler import RainbowLoggingHandler
#logging.basicConfig(level=logging.DEBUG, format='%(filename)-15s:%(lineno)-4d %(levelname)s: %(message)s')
handler = RainbowLoggingHandler(sys.stderr)
handler.setFormatter(logging.Formatter('%(filename)s:%(lineno)d\t%(levelname)s: %(message)s'))
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
logger.addHandler(handler)

logging.getLogger('OpenGL').setLevel(99)

# allow importing from the repository root
import sys, os, json
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import apc.baxter_scoop as baxter
from integration.visualization import debug_cloud
from integration.camera import uvtexture, invalid_mask
from integration.io import pcd
from perception.segmentation.color import rgb2yuv, make_uv_hist

from klampt.robotsim import WorldModel
from klampt import se3, so3
from time import sleep, time
import numpy, math
from matplotlib import pyplot

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
        blob_uv = [ rgb2yuv(*rgb)[1:3] for rgb in color2[mask & color_valid_mask] ]
        blob_histogram = make_uv_hist(blob_uv)

        numpy.save(hist_path, blob_histogram)

    return blob_histogram

base_path = os.path.join('perception', 'models_db2')

objects = json.load(open(os.path.join('perception', 'segmentation', 'null_match_values.json'))).keys()

for obj in objects:
    positive_scores = []
    negative_scores = []
    
    # load the composite histogram
    composite_histogram = numpy.load(os.path.join('perception', 'uv_hist2', '{}.npz'.format(obj)))['arr_0']

    # check against the positives
    for n in range(6):
        blob_histogram = load_histogram(obj, n)
        score = 2 * numpy.minimum(blob_histogram, composite_histogram).sum() - numpy.maximum(blob_histogram, composite_histogram).sum()
        #score = (score + 2) / 3

        if not math.isnan(score):
            positive_scores.append(score)

        print obj, 'positive:', n, '->', score

    # check against the negatives
    for obj2 in objects:
        if obj == obj2:
            continue

        for n in range(6):
            blob_histogram = load_histogram(obj2, n)
            score = 2 * numpy.minimum(blob_histogram, composite_histogram).sum() - numpy.maximum(blob_histogram, composite_histogram).sum()
            #score = (score + 2) / 3
            
            if not math.isnan(score):
                negative_scores.append(score)

            print obj, 'negative:', obj2, n, '->',  score

    numpy.save(os.path.join(base_path, '{}_pos.npy'.format(obj)), positive_scores)
    numpy.save(os.path.join(base_path, '{}_neg.npy'.format(obj)), negative_scores)

print 'done'
