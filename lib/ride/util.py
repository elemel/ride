from __future__ import division

import sys

def sign(x):
    return -1 if x < 0 else 1

def log(message):
    sys.stderr.write('ride: %s\n' % str(message))
