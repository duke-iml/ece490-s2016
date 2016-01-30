#!/usr/bin/env python

import numpy, sys

data = numpy.loadtxt(sys.stdin)
numpy.save(sys.stdout, data)