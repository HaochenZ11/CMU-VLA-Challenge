
# The following is for compatibility issue with PyTorch 1.8 on 
# Jetson Xavier NX with Jetpack 4.6, Python 3.6.

import numpy as np


np.meshgrid( np.random.rand(3), np.random.rand(3), indexing='ij' )

# No error.
def np_meshgrid(x, y, indexing='xy'):
    res = np.meshgrid(x, y, indexing=indexing)
    return [ np.ascontiguousarray(r) for r in res ]
