import logging, traceback
logger = logging.getLogger(__name__)

import numpy
from time import time

def distance_label(cloud, mask, threshold=0.005):
    threshold *= threshold
    mark = time()
    
    label = 0
    labeled = numpy.zeros(cloud.shape[:2], dtype=numpy.int)

    neighborhood = [ (0, -1), (-1, 0), (0, 1), (1, 0) ]
    
    stack = []
    for r in range(cloud.shape[0]):
        for c in range(cloud.shape[1]):
            if mask[r, c]:
                stack.append((r, c))

    while stack:
        (r, c) = stack.pop()

        if not labeled[r, c]:
            label += 1
            labeled[r, c] = label
            logger.debug('created label {}'.format(label))         
         
        # now check all the neighbors
        for (nr, nc) in [ (r + dr, c + dc) for (dr, dc) in neighborhood ]:
            if not (0 <= nr < cloud.shape[0] and 0 <= nc < cloud.shape[1]) or not mask[r, c]:
                continue

            # check if this neighbor is connected
            d = sum((cloud[r, c] - cloud[nr, nc])**2)
            if d > threshold:                
                continue

            # recurse
            if not labeled[nr, nc]:
                # label the neighbor
                labeled[nr, nc] = label
                stack.append((nr, nc))
    
    logger.info('distance connected components in {:.3f}s'.format(time() - mark))

    return (labeled, label)

# def distance_label2(cloud, mask, threshold=0.005**2):
#     horz_connected = mask[1:-1,1:-1] & ((cloud[0:-1,1:-1] - cloud[1:,1:-1])**2).sum() <= threshold
#     vert_connected = mask[1:-1,1:-1] & ((cloud[1:-1,0:-1] - cloud[1:-1,1:])**2).sum() <= threshold

#     from scipy.ndimage.measurements import label
#     (horz_labeled, horz_count) = label(horz_connected)
#     (vert_labeled, vert_count) = label(vert_connected)

#     vert_labeled[vert_labeled > 0] += horz_count

#     equivalence = {}
#     for r in range(0, cloud.shape[0]-2):
#         for c in range(0, cloud.shape[1]-2):
#             if horz_labeled[r, c] and vert_labeled[r, c]:
#                 equivalence.setdefault(horz_labeled[r, c], set()).add(vert_labeled[r, c])

#     labeled = horz_labeled.copy()

# def distance_label3(cloud, mask, threshold=0.005**2):
#     mark = time()

#     label_count = 0
#     equivalences = {}
#     labeled = numpy.zeros(cloud.shape[:2], dtype=numpy.int)

#     neighborhood = [ (0, -1), (-1, 0) ]
    
#     for r in range(cloud.shape[0]):
#         for c in range(cloud.shape[1]):
#             if not mask[r, c]:
#                 continue

#             for (nr, nc) in [ (r + dr, c + dc) for (dr, dc) in neighborhood ]:
#                 if nr >= 0 and nc >= 0 and not mask[nr, nc]:
#                     continue

#                 d = sum((cloud[r, c] - cloud[nr, nc])**2)
#                 if d > threshold:                
#                     # this neighbor is not connected
#                     continue

#                 if labeled[r, c] == 0:
#                     # pixel is unlabeled so use the neighbor's label
#                     labeled[r, c] = labeled[nr, nc]
#                 else:
#                     # pixel is already labeled so set an equivalence
#                     equivalences.setdefault(
#                         min(labeled[r, c], labeled[nr, nc]),
#                         set()
#                     ).add(max(labeled[r, c], labeled[nr, nc]))
#                     logger.debug('found equivalence {} <-> {}'.format(min(labeled[r, c], labeled[nr, nc]), max(labeled[r, c], labeled[nr, nc])))

#             # check that the pixel got labeled
#             if labeled[r, c] == 0:
#                 # create a new label for it
#                 label_count += 1
#                 labeled[r, c] = label_count
#                 logger.debug('created label {}'.format(label_count))
                    
#     # resolve equivalences
#     for (label, equivalents) in equivalences.items():
#         for equivalent in equivalents:
#             labeled[labeled == equivalent] = label                                    
    
#     logger.debug('distance connected components in {:.3f}s'.format(time() - mark))

#     return (labeled, labeled.max())
#     
