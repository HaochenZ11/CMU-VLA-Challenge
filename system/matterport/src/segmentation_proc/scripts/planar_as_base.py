
# Author: 
# Yaoyu Hu <yaoyuh@andrew.cmu.edu>
# Date:
# 2021-05-06

import copy
import cv2
import numpy as np

INTER_MAP_OCV = {
    'linear': cv2.INTER_LINEAR,
    'nearest': cv2.INTER_NEAREST
}

INTER_MAP = {
    'nearest': 'nearest',
    'linear': 'bilinear',
}

INTER_BLENDED = 'blended'

class PlanarAsBase(object):
    def __init__(self, 
                 fov, 
                 camera_model, 
                 cached_raw_shape=(1024, 2048),
                 default_invalid_value=0):
        '''
        NOTE: If convert_output=False, then the output is a Tensor WITH the batch dimension.
        That is, the output is a 4D Tensor no matter whether the input is a single image
        or a collection of images.
        
        Arguments:
        fov (float): Full FoV of the lens in degrees.
        camera_model (camera_model.CameraModel): Target camera model. 
        cached_raw_shape (two-element): The tentative shape of the support raw image. Use some positive values if not sure.
        convert_output (bool): True if the output needs to be converted to NumPy (OpenCV).
        default_invalid_value (scalar): The default value for the invalid output pixels.
        '''
        super(PlanarAsBase, self).__init__()
        
        self.fov = fov # Degree.

        self.camera_model = copy.deepcopy(camera_model)
        self.shape = self.camera_model.shape

        # The rotation matrix of the fisheye camera.
        # The notation is R_<to>_<from> or R_<measured in>_<be measured>.
        # This rotation matrix is the orientation of the fisheye camera w.r.t
        # the frame where we take the raw images. And the orientation is measured
        # in the raw image frame.
        
        self.cached_raw_shape = cached_raw_shape
            
        self.default_invalid_value = default_invalid_value

    def is_same_as_cached_shape(self, new_shape):
        return new_shape[0] == self.cached_raw_shape[0] and new_shape[1] == self.cached_raw_shape[1]

    def get_xyz(self, back_shift_pixel=False):
        '''
        Compute the ray vectors for all valid pixels in the fisheye image.
        A ray vector is represented as a unit vector.
        All ray vectors will be transformed such that their coordiantes are
        measured in the raw frame where z-forward, x-right, and y-downward.
        
        Some pixels are not going to have valid rays. There is a mask of valid
        pixels that is also returned by this function.

        Returns:
            xyz (Tensor): 3xN, where N is the number of pixels.
            valid_mask (Tensor): 1xN, where N is the number of pixels. A binary mask.
        '''
        # The pixel coordinates.
        # # xx, yy = self.mesh_grid_pixels(self.shape, flag_flatten=True) # 1D.
        # xx, yy = self.camera_model.pixel_meshgrid( flatten=True )
        
        # if back_shift_pixel:
        #     xx -= 0.5
        #     yy -= 0.5
        
        # pixel_coor = torch.stack( (xx, yy), dim=0 ) # 2xN

        # xyz, valid_mask = \
        #     self.camera_model.pixel_2_ray(pixel_coor)
        
        shift = 0 if back_shift_pixel else 0.5
        xyz, valid_mask = self.camera_model.get_rays_wrt_sensor_frame(shift=shift)
        
        # xyz and valid_mask are torch.Tensor.
        # xyz = xyz.astype(np.float32)

        return xyz, valid_mask