import numpy
from klampt import so3, se3

from client import RemoteRawCamera

def uvtexture(src, uv2):
    # copy the uv map because it is modified below
    # also flip x/y since numpy is row/column
    uv = uv2[:,:,::-1].copy()

    # scale the uv coordinates by the source image size
    for i in range(2):
        uv[:,:,i] = numpy.clip(uv[:,:,i] * src.shape[i], 0, src.shape[i]-1)

    # round to the nearest pixel
    uv = uv.round().astype(numpy.int)

    # lookup value in source image
    dst = src[uv[:,:,0], uv[:,:,1]]
    
    # set invalid pixels to green
    dst[invalid_mask(uv2)] = [ 0, 255, 0 ]

    return dst

def invalid_mask(image):
    if len(image.shape) == 3 and image.shape[-1] == 3:
        # a point cloud
        return image[:,:,2] > 0
    elif len(image.shape) == 3 and image.shape[-1] == 2:
        # a UV map
        return (image[:,:,0] < 0) | (image[:,:,1] < 0) | (image[:,:,0] > 1) | (image[:,:,1] > 1)
    else:
        raise NotImplementedError('unsuppored image format')

class RemoteCamera:
    def __init__(self, address, port=10000, xform=None):
        self.raw = RemoteRawCamera(address, port)
        self.xform = xform or se3.identity()
        
    def read(self, clean=True):
        (color, cloud, depth_uv) = self.raw.read()
        
        # flip everything up/down based on camera mounting
        color = color[::-1,:,:]
        cloud = cloud[::-1,:,:]
        if depth_uv is not None:
            depth_uv = depth_uv[::-1,:,:]

        # the point cloud and the depth UV map actually need to have their values changed
        # because the Y spatial direction is reversed
        cloud[:,:,1] *= -1
        if depth_uv is not None:
            depth_uv[:,:,1] = 1 - depth_uv[:,:,1]

        # convert point cloud to meters
        cloud /= 1000

        # compute the invalid mask early because world coordinates make
        # the z = 0 invalid points non-zero
        self.mask = invalid_mask(cloud)

        # apply the camera xform
        cloud = cloud.dot(numpy.array(self.xform[0]).reshape((3, 3))) + self.xform[1]
        
        self.color = color
        self.cloud = cloud
        self.depth_uv = depth_uv

        return (color, cloud, depth_uv)
        
    def clean(self):
        return (self.mask, self.cloud[self.mask])

    def colorize(self):
        if self.depth_uv is not None:
            colorized = uvtexture(self.color, self.depth_uv)
            return colorized
        else:
            return None

    def close(self):
        self.raw.close()
