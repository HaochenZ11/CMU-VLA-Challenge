
import copy
import math
import sys
import numpy as np

from compatible_np import np_meshgrid
from radian import check_valid_range
from shape_struct import ShapeStruct

CAMERA_MODELS = dict()
LIDAR_MODELS = dict()

LOCAL_PI = math.pi

def deg2rad(deg):
    global LOCAL_PI
    return deg / 180.0 * LOCAL_PI

def register(dst):
    '''Register a class to a destination dictionary. '''
    def dec_register(cls):
        dst[cls.__name__] = cls
        return cls
    return dec_register


class SensorModel(object):
    def __init__(self, name, shape_struct):
        super().__init__()
        
        self.name = name
        
        self._ss = None # Initial value.
        self.ss = shape_struct # Update self._ss.

    
    @staticmethod
    def make_shape_struct_from_repr(shape_struct):
        if isinstance( shape_struct, dict ):
            return ShapeStruct( **shape_struct )
        elif isinstance( shape_struct, ShapeStruct ):
            return shape_struct
        else:
            raise Exception(f'shape_struct must be a dict or ShapeStruct object. Get {type(shape_struct)}')
    
    @property
    def ss(self):
        return self._ss

    @ss.setter
    def ss(self, shape_struct):
        self._ss = SensorModel.make_shape_struct_from_repr(shape_struct)
    
    @property
    def shape(self):
        return self.ss.shape

    def out_wrap(self, x):
        return x

    def __deepcopy__(self, memo):
        '''
        https://stackoverflow.com/questions/57181829/deepcopy-override-clarification#:~:text=In%20%22How%20to%20override%20the%20copy%2Fdeepcopy%20operations%20for,setattr%20%28result%2C%20k%2C%20deepcopy%20%28v%2C%20memo%29%29%20return%20result
        '''
        cls = self.__class__ # Extract the class of the object
        result = cls.__new__(cls) # Create a new instance of the object based on extracted class
        memo[ id(self) ] = result
        for k, v in self.__dict__.items():
            setattr(result, k, copy.deepcopy(v, memo)) # Copy over attributes by copying directly or in case of complex objects like lists for exaample calling the `__deepcopy()__` method defined by them. Thus recursively copying the whole tree of objects.
        return result
    
    def get_rays_wrt_sensor_frame(self, shift=0.5):
        '''
        This function returns the rays shoting from the sensor and a valid mask.
        '''
        raise NotImplementedError()
        
class CameraModel(SensorModel):
    def __init__(self, name, fx, fy, cx, cy, fov_degree, shape_struct):
        super(CameraModel, self).__init__(
            name=name, shape_struct=shape_struct)

        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.fov_degree = fov_degree 
        self.fov_rad = deg2rad( self.fov_degree )
        
        self.padding_mode_if_being_sampled = 'zeros'
        
        # Will be populated once get_valid_mask() is called for the first time.
        self.valid_mask = None

    def _f(self):
        assert self.fx == self.fy
        return self.fx

    @property
    def f(self):
        # _f() is here to be called by child classes.
        return self._f()


    def pixel_meshgrid(self, shift=0.5, normalized=False, flatten=False):
        '''
        Get the meshgrid of the pixel centers.
        shift is applied along the x and y directions.
        If normalized is True, then the pixel coordinates are normalized to [-1, 1].
        '''
        
        H, W = self.shape
        
        x = np.arange(W, dtype=np.float32) + shift
        y = np.arange(H, dtype=np.float32) + shift
        
        xx, yy = np_meshgrid(x, y, indexing='xy')
        
        xx, yy = np.ascontiguousarray(xx), np.ascontiguousarray(yy)
        
        if normalized:
            xx = xx / W * 2 - 1
            yy = yy / H * 2 - 1
        
        if flatten:
            xx = xx.reshape((-1))
            yy = yy.reshape((-1))
        
        return xx, yy
    
    def pixel_coordinates(self, shift=0.5, normalized=False, flatten=False):
        '''
        Get the pixel coordinates.
        shift is appllied along the x and y directions.
        If normalized is True, then the pixel coordinates are normalized to [-1, 1].
        '''
        xx, yy = self.pixel_meshgrid(shift=shift, normalized=normalized, flatten=flatten)
        return np.ascontiguousarray(np.stack( (xx, yy), axis=0 ))

    def pixel_2_ray(self, pixel_coor):
        '''
        Arguments:
        pixel_coor (Tensor): A 2xN Tensor contains the pixel coordinates. 
        
        NOTE: pixel_coor can also have a dimension of Bx2xN, where B is the 
        batch number.
        
        Returns:
        A 3xN Tensor representing the 3D rays. Bx3xN if batched.
        A (N,) Tensor representing the valid mask. BxN if batched.
        '''
        raise NotImplementedError()
    
    def get_rays_wrt_sensor_frame(self, shift=0.5):
        '''
        This function returns the rays shooting from the sensor and a valid mask.
        '''
        pixel_coor = self.pixel_coordinates(shift=shift, flatten=True)
        return self.pixel_2_ray( pixel_coor )
        
@register(CAMERA_MODELS)
class Equirectangular(CameraModel):
    def __init__(self, 
        shape_struct, 
        longitude_span=(-LOCAL_PI, LOCAL_PI), 
        latitude_span=(-LOCAL_PI/2, LOCAL_PI/2), 
        open_span=False):
        '''
        Used primarily for generating a panorama image from six pinhole images or ratating a
        panorama image for training.
        
        Since it is a camera model, the frame of the image is similar to other camera models.
        The z-axis is the optical axis and pointing forward. The x-axis is to the right. The 
        y-axis is downwards. 

        The principle point of a equirectangular camera is always the zero longitude and zero
        latitude angle. For extreme cases, the principle point may locate outside the image.
        
        To represent a panorama image generated by Unreal Engine, the longitude_span should be
        [-3/2 pi, pi/2].
        '''

        # The only difference is that to generate a panorama image similar
        # to the ones generated by the Unreal Engine, we need to shift the longitude, normally 
        # by -pi/2 angle. By "shift", we mean addition.

        # NOTE: Currently, this is not the frame attached to the panorama obtained from Unreal Engine.
        # When lon_shift is 0, the frame is the same as the normal definition, where z-axis is in the
        # forward direction and the middle of the image is the zero longitude angle.

        # These two members are used during the call to set_members_by_shape_struct() and _resize().
        self.init_longitude_span = longitude_span
        self.init_latitude_span  = latitude_span
        
        self.open_span = open_span

        super(Equirectangular, self).__init__(
            'equirectangular', 1, 1, 0.5, 0.5, 360, shape_struct)
        # cx, cy will be updated.
        self.set_members_by_shape_struct(self.ss)

        # # Since lon_shift is applied by adding to the longitude span, the shifted frame has a measured
        # # rotation of -lon_shift, w.r.t. the original frame. Thus, the shifted frame has a rotation 
        # # that is measured in the original frame as
        # a = -self.lon_shift
        # self.R_ori_shifted = torch.Tensor(
        #     [ [ math.cos(a), -math.sin(a) ], 
        #       [ math.sin(a),  math.cos(a) ] ]
        #     ).to(dtype=torch.float32)
        
        # Override parent's variable.
        self.padding_mode_if_being_sampled = 'border'

    def set_members_by_shape_struct(self, shape_struct):
        # Full longitude span is [-pi, pi], with possible shift or crop.
        # Full latitude span is [-pi/2, pi/2], with possible crop. No shift.
        # The actual longitude span that all the pixels cover.
        check_valid_range( *self.init_longitude_span, raise_exception=True )
        self.lon_span_pixel = self.init_longitude_span[1] - self.init_longitude_span[0]        
        
        # open_span is True means the last column of pixels do not have the same longitude angle as the first column.
        if self.open_span:
            # TODO: Potential bug if cx is not at the center of the image.
            # self.lon_span_pixel = 2*self.cx / ( 2*self.cx + 1 ) * self.lon_span_pixel
            self.lon_span_pixel = ( shape_struct.W - 1 ) / shape_struct.W * self.lon_span_pixel
        
        assert self.init_latitude_span[0] >= -LOCAL_PI / 2 and self.init_latitude_span[1] <= LOCAL_PI / 2, \
            f'latitude_span is wrong: {self.init_latitude_span}. '
        
        self.longitude_span = np.array( [ self.init_longitude_span[0], self.init_longitude_span[1] ], dtype=np.float32)
        self.latitude_span  = np.array( [ self.init_latitude_span[0],  self.init_latitude_span[1]  ], dtype=np.float32)

        # Figure out the virtual image center.
        # self.cx = ( 0 - self.init_longitude_span[0] ) / self.lon_span_pixel * ( shape_struct.W - 1 )
        # self.cy = ( 0 - self.init_latitude_span[0] ) / ( self.init_latitude_span[1] - self.init_latitude_span[0] ) * ( shape_struct.H - 1 )
        # Yaoyu 20230212: The following looks more resonable.
        self.cx = ( 0 - self.init_longitude_span[0] ) / self.lon_span_pixel * shape_struct.W
        self.cy = ( 0 - self.init_latitude_span[0] ) / \
            ( self.init_latitude_span[1] - self.init_latitude_span[0] ) * shape_struct.H

    # TODO: Which is better: direct scale or calling set_members_by_shape_struct()?
    def _resize(self, new_shape_struct):
        self.set_members_by_shape_struct(new_shape_struct)
        self.ss = new_shape_struct

    def pixel_2_ray(self, pixel_coor):
        '''
        This function assumes that all coordinates in pixel_coor are valid.

        Arguments:
        pixel_coor (Tensor): A 2xN Tensor contains the pixel coordinates. 
        
        NOTE: pixel_coor can also have a dimension of Bx2xN, where B is the 
        batch number. 
        
        Returns:
        ray: A 3xN Tensor representing the 3D rays. Bx3XN if batched.
        valid_mask: A (N,) Tensor representing the valid mask. BxN if batched.
        '''
        
        
        pixel_space_shape = \
            np.array([ self.ss.W, self.ss.H ], dtype=np.float32).reshape((2, 1))

        angle_start = \
            np.array([ self.longitude_span[0], self.latitude_span[0] ], dtype=np.float32).reshape((2, 1))
        
        angle_span = np.array(
            [ self.lon_span_pixel, self.latitude_span[1] - self.latitude_span[0] ], dtype=np.float32).reshape((2, 1))
        
        lon_lat = pixel_coor / pixel_space_shape * angle_span + angle_start
        
        # Bx1xN after calling np.split.
        longitute, latitute = np.split( lon_lat, 2, axis=-2 )
        
        c = np.cos(latitute)
        
        x = c * np.sin(longitute)
        y =     np.sin(latitute)
        z = c * np.cos(longitute)
               
        return np.concatenate( (x, y, z), axis=-2 ), np.ones_like(x.squeeze(-2), dtype=bool)