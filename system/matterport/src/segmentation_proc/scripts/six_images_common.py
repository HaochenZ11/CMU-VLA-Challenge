
import numpy as np

# -->       +---------+
# |  x      |0,W      |0,2W
# v         |    4    |
#  y        |   top   |
# +---------+---------+---------+---------+
# |H,0      |H,W      |H,2W     |H,3W     |H,4W
# |    3    |    0    |    1    |    5    |
# |   left  |  front  |  right  |   back  |
# +---------+---------+---------+---------+
#  2H,0     |2H,W     |2H,2W     2H,3W     2H,4W
#           |    2    |
#           |  bottom |
#           +---------+
#           3H,W     3H,2W

FRONT  = 'front'
BACK   = 'back'
LEFT   = 'left'
RIGHT  = 'right'
TOP    = 'top'
BOTTOM = 'bottom'

OFFSETS = offsets=np.array( 
            [ [1, 2, 1, 0, 1, 3],
              [1, 1, 2, 1, 0, 1] ], dtype=np.int32)

def make_image_cross_npy(imgs):
    '''
    Arguments:
    imgs (dict of arrays): The six images with the keys as front, back, left, right, top, and bottom.

    Returns:
    A image cross with shape (3*H, 4*W).
    '''
    
    global FRONT, BACK, LEFT, RIGHT, TOP, BOTTOM
    
    H, W = imgs[FRONT].shape[:2]
    d_type = imgs[FRONT].dtype

    if ( imgs[FRONT].ndim == 3 ):
        # Get the last dimension of the input image.
        last_dim = imgs[FRONT].shape[2]
        canvas = np.zeros( ( 3*H, 4*W, last_dim ), dtype=d_type )
    elif ( imgs[FRONT].ndim == 2 ):
        canvas = np.zeros( ( 3*H, 4*W ), dtype=d_type )
    else:
        raise Exception(f'Wrong dimension of the input images. imgs[FRONT].shape = {imgs[FRONT].shape}')

    canvas[  H:2*H,   W:2*W, ...] = imgs[FRONT]  # Front.
    canvas[  H:2*H, 2*W:3*W, ...] = imgs[RIGHT]  # Right.
    canvas[2*H:3*H,   W:2*W, ...] = imgs[BOTTOM] # Bottom.
    canvas[  H:2*H,   0:W,   ...] = imgs[LEFT]   # Left.
    canvas[  0:H,     W:2*W, ...] = imgs[TOP]    # Top.
    canvas[  H:2*H, 3*W:4*W, ...] = imgs[BACK]   # Top.

    # Padding.
    # Right.
    canvas[ H-1, 2*W:3*W, ... ] = imgs[TOP][    ::-1, -1, ... ]
    canvas[ 2*H, 2*W:3*W, ... ] = imgs[BOTTOM][    :, -1, ... ]
    # Bottom.
    canvas[ 2*H:3*H, 2*W, ... ] = imgs[RIGHT][ -1,    :, ... ]
    canvas[ 2*H:3*H, W-1, ... ] = imgs[LEFT][  -1, ::-1, ... ]
    # Left.
    canvas[ H-1, 0:W, ... ] = imgs[TOP][       :, 0, ...]
    canvas[ 2*H, 0:W, ... ] = imgs[BOTTOM][ ::-1, 0, ...]
    # Top.
    canvas[ 0:H, W-1, ... ] = imgs[LEFT][  0,    :, ...]
    canvas[ 0:H, 2*W, ... ] = imgs[RIGHT][ 0, ::-1, ...]
    # Back.
    canvas[ H-1, 3*W:4*W, ... ] = imgs[TOP][     0, ::-1, ... ]
    canvas[ 2*H, 3*W:4*W, ... ] = imgs[BOTTOM][ -1, ::-1, ... ]

    return canvas
