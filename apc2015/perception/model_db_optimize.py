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

import numpy, math
from matplotlib import pyplot

base_path = os.path.join('perception', 'models_db2')

objects = json.load(open(os.path.join('perception', 'segmentation', 'null_match_values.json'))).keys()

match_stats = {}

for obj in objects:
    positive_scores = numpy.load(os.path.join(base_path, '{}_pos.npy'.format(obj)))
    negative_scores = numpy.load(os.path.join(base_path, '{}_neg.npy'.format(obj)))
    
    if len(positive_scores) + len(negative_scores) == 0:
        print 'skip', obj
        continue
    
    thresholds = numpy.linspace(-2, 1, 101)

    sensitivity = numpy.zeros_like(thresholds)
    specificity = numpy.zeros_like(thresholds)
    ppv = numpy.zeros_like(thresholds)
    npv = numpy.zeros_like(thresholds)

    for (i, threshold) in enumerate(thresholds):
        true_positives = numpy.count_nonzero(positive_scores >= threshold)
        false_positives = numpy.count_nonzero(negative_scores >= threshold)
        true_negatives = numpy.count_nonzero(negative_scores < threshold)
        false_negatives = numpy.count_nonzero(positive_scores < threshold)

        #print threshold, 'TP', true_positives, 'FP', false_positives, 'TN', true_negatives, 'FP', false_negatives

        sensitivity[i] = float(true_positives) / (true_positives + false_negatives + 1e-10)
        specificity[i] = float(true_negatives) / (true_negatives + false_positives + 1e-10)
        ppv[i] = float(true_positives) / (true_positives + false_positives + 1e-10)
        npv[i] = float(true_negatives) / (true_negatives + false_negatives + 1e-10)

    # find the intersection
    difference = numpy.absolute(sensitivity - specificity)
    intersection = difference.argmin()
    cutoff = thresholds[difference == difference[intersection]].mean() - 0.25
    index = numpy.absolute(thresholds - cutoff).argmin()

    match_stats[obj] = {
        'cutoff': cutoff,
        'ppv': ppv[index],
        'npv': npv[index],
        'specificity': specificity[index],
        'sensitivity': sensitivity[index]
    }
    print obj, cutoff, ppv[index], npv[index]

    #pyplot.axis([-2, 1, 0, 1.1])
    #pyplot.plot(thresholds, sensitivity, label='sensitivity')
    #pyplot.plot(thresholds, specificity, label='specificity')
    #pyplot.plot([ cutoff, cutoff ], [ 0, 1.1 ], label='cutoff')
    #pyplot.xticks(numpy.linspace(-2, 1, 21))
    #pyplot.grid()
    #pyplot.legend()
    #pyplot.title(obj)
    #pyplot.show()

stats = sorted(match_stats.items(), key=lambda x: x[1]['ppv'])

pyplot.axis([0, len(stats)-1, -2, 1.1])
for k in [ 'cutoff', 'ppv', 'npv' ]:
    pyplot.plot([ s[1][k] for s in stats ], label=k)
pyplot.xticks(numpy.arange(len(stats)), [ s[0] for s in stats ], rotation='vertical')
pyplot.subplots_adjust(bottom=0.3)
pyplot.legend()
pyplot.grid()
pyplot.show()

json.dump(match_stats, open(os.path.join('perception', 'segmentation', 'match_stats.json'), 'w'), indent=4)
print 'done'
