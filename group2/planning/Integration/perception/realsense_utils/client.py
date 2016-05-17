import logging, traceback
logger = logging.getLogger(__name__)
logging.getLogger('packet').setLevel(logging.WARNING)

import socket
from time import time
from __init__ import unnormalize_uv_map

import packet

class RemoteCamera:
    def __init__(self, address, port=10000):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.connect((address, port))

    def read(self):
        images = packet.parse(self._receive)
        logger.debug('received {} images'.format(len(images)))

        if len(images) == 2:
            (color, cloud) = images
            depth_uv = None
            color_uv = None
        elif len(images) == 3:
            (color, cloud, depth_uv) = images
            color_uv = None
        elif len(images) == 4:
            (color, cloud, depth_uv, color_uv) = images
        else:
            logger.warn('unrecognized number of images from camera')
            raise RuntimeError('unrecognized number of images from camera')
        if cloud is not None:
            cloud[:,:,1] *= -1 # flip y axis
        if depth_uv is not None:
            depth_uv = depth_uv[:,:,::-1]
            depth_uv = unnormalize_uv_map(depth_uv, color.shape)
        if color_uv is not None:
            color_uv = color_uv[:,:,::-1]
            color_uv = unnormalize_uv_map(color_uv, (cloud.shape[0], cloud.shape[1]))
        return (color, cloud, depth_uv, color_uv)

    def _receive(self, n):
        buf = []

        while n > 0:
            data = self._socket.recv(min(n, 4096))
            n -= len(data)
            buf.append(data)

        if len(buf) == 1:
            return buf[0]
        else:
            return ''.join(buf)

    def close(self):
        self._socket.close()

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)-15s %(filename)s:%(lineno)d %(levelname)s: %(message)s')
    logging.getLogger('OpenGL').setLevel(99)

    import sys, numpy, matplotlib.cm
    from PySide.QtGui import QApplication

    from ui.numpy_widget import NumpyWidget

    camera = RemoteCamera('localhost', 10000)

    if "show" in sys.argv or "-show" in sys.argv or "--show" in sys.argv:
        show = True
    else:
        show = False
    while "show" in sys.argv:
        sys.argv.remove("show")
    while "-show" in sys.argv:
        sys.argv.remove("-show")
    while "--show" in sys.argv:
        sys.argv.remove("--show")
        
    if show:
        app = QApplication(sys.argv)
        color_window = NumpyWidget()
        color_window.show()
        depth_window = NumpyWidget()
        depth_window.show()

    colormap = numpy.float32(matplotlib.cm.jet(numpy.arange(1001) / 1000.0))

    n = 0

    try:
        mark = time()
        while True:
            (color, cloud, depth_uv, color_uv) = camera.read()

            if show:
                color_window.update_frame(color)
                depth_window.update_frame(colormap[numpy.clip(cloud[:,:,2], 0, len(colormap) - 1).astype(numpy.int)])
                app.processEvents()
                
            n += 1

            if n % 30 == 0:
                # print some debug stats
                fps = 30 / (time() - mark)
                mark = time()
                logger.debug('{:.1f} fps'.format(fps))

    except KeyboardInterrupt:
        pass

    camera.close()
