
import numpy as np

_LOCAL_2_PI = 2 * np.pi

def check_valid_range(r0, r1, raise_exception=False):
    '''
    Return True if:
    - r0 and r1 are in the range of [-2pi, 2pi]
    - r0 < r1
    - r1 - r0 <= 2pi
    '''
    
    global _LOCAL_2_PI
    
    flag = True
    
    if r0 >= r1:
        flag = False
        reason = f'r0 ({r0}) >= r1 ({r1}). '
    elif r0 < -_LOCAL_2_PI or r1 > _LOCAL_2_PI:
        flag = False
        reason = f'r0 ({r0}) or r1 ({r1}) has a wrong value. The value must be in [-2pi, 2pi]. '
    elif r1 - r0 > _LOCAL_2_PI:
        flag = False
        reason = f'r1 ({r1}) - r0 ({r0}) = {r1-r0} > 2pi. '
    
    if not flag and raise_exception:
        raise ValueError(reason)
    
    return flag