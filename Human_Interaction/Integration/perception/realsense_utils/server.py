
try:
    from PyRealSenseLib import Camera, StreamType, PixelFormat
except:
    print "Error loading DLL files. Are you using 32-bit Python? This program can only be run by 64-bit Python on Windows OS. "
    quit()

import logging, traceback
logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG, format='%(asctime)-15s %(filename)s:%(lineno)d %(levelname)s: %(message)s')
logging.getLogger('OpenGL').setLevel(99)
logging.getLogger('packet').setLevel(logging.WARNING)

import sys, numpy, matplotlib.cm, struct, socket
from time import time, sleep
from PySide.QtGui import QApplication

from ui.numpy_widget import NumpyWidget
import packet

def parse_args():
    try:
        device_identifier = int(sys.argv[1])
    except ValueError:
        device_identifier = sys.argv[1]
    except IndexError:
        device_identifier = 0

    try:
        bind_address = sys.argv[2]

        if ':' in bind_address:
            address, port = bind_address.split(':')
        else:
            address = bind_address
            port = 30000

        port = int(port)
    except IndexError:
        address = '10.236.66.147'
        port = 30000

    return (device_identifier, address, port)

def setup_socket(address, port):
    # bind the socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # increase buffer size
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8*2**20)
    # set non-blocking
    sock.setblocking(0)
    sock.bind((address, port))
    sock.listen(5)

    return sock

def setup_camera(device_identifier):
    # enumerate all devices
    try:
        devices = list(Camera.list(StreamType.COLOR | StreamType.DEPTH))
    except:
        logger.error('unable to enumerate devices')
        logger.error(traceback.format_exc())

    if devices:
        for (i, device) in enumerate(devices):
            logger.debug('found device {}: {} {} {}'.format(i, device, device.serial, device.deviceId))
    else:
        logger.debug('found no devices')

    if isinstance(device_identifier, int):
        # select the device based on device index
        try:
            device = devices[device_identifier]
        except IndexError:
            logger.error('invalid device index: {}'.format(device_identifier))
            raise SystemExit
    else:
        # select the device based on serial number
        try:
            device = filter(lambda d: d.serial == device_identifier, devices)[0]
        except IndexError:
            logger.error('invalid device serial: {}'.format(device_identifier))
            raise SystemExit

    logger.info('using device {} {}'.format(device, device.serial))
    camera = Camera(device)

    depth_formats = camera.formats(StreamType.DEPTH)
    for df in depth_formats:
        logger.debug('found depth format {}'.format(df))

    # find the highest resolution 30 fps depth format
    depth_formats = filter(lambda df: df.fps == 30 and df.format == PixelFormat.DEPTH, depth_formats)
    if depth_formats:
        depth_format = max(depth_formats, key=lambda cf: cf.width * cf.height)
        logger.debug('using depth format {}'.format(depth_format))
    else:
        logger.error('unable to find suitable depth format')
        raise SystemExit

    color_formats = camera.formats(StreamType.COLOR)
    for cf in color_formats:
        logger.debug('found color format {}'.format(cf))

    # find a 30 fps color format of the same resolution
    color_formats = filter(lambda cf: cf.fps == 30 and cf.format == PixelFormat.RGB24 and cf.width == depth_format.width and cf.height == depth_format.height, color_formats)
    if color_formats:
        color_format = color_formats[0]
        logger.debug('using color format {}'.format(color_format))
    else:
        logger.error('unable to find color format matching {}'.format(depth_format))
        raise SystemExit

    try:
        camera.open(color_format, depth_format)
    except Exception as e:
        logger.error('error opening camera')
        logger.error(traceback.format_exc())
        raise SystemExit
    else:
        logger.info('camera open')

    return camera

def check_connections(sock, clients):
    while True:
        try:
            (conn, addr) = sock.accept()
        except socket.error as e:
            if e.errno in [ 11, 10035 ]:
                # non-blocking operation did not complete -- ignore
                pass
            else:
                logger.error('error accepting socket connection')
                logger.error(traceback.format_exc())
            break
        else:
            clients.append((conn, addr))
            logger.info('client connected {}'.format(addr))

def send_frame(clients, images):
    data = packet.create(images)

    for client in clients:
        (conn, addr) = client

        try:
            conn.send(data)
        except socket.error as e:
            if e.errno in [ 11, 10035 ]:
                # non-blocking operation did not complete -- ignore
                pass
            else:
                if e.errno == 10054:
                    # remote host closed connection
                    logger.info('client disconnected {}'.format(addr, e))
                else:
                    logger.warn('lost connection {}: {}'.format(addr, e))
                conn.close()
                clients.remove(client)

    logger.debug('frame sent: {:.1f} MiB x{}'.format(len(data) / 2.0**20, len(clients)))

def loop(sock, camera, app, show=True):
    if show:
        color_window = NumpyWidget()
        color_window.show()
        depth_window = NumpyWidget()
        depth_window.show()

        colormap = numpy.float32(matplotlib.cm.jet(numpy.arange(1001) / 1000.0))

    n = 0
    clients = []

    try:
        mark = time()
        while True:
            # check for new clients
            check_connections(sock, clients)

            # read and process the camera frame
            try:
                frame = camera.read()
            except RuntimeError as e:
                logger.error('error while capturing frame')
                logger.error(traceback.format_exc())
                break

            color_buffer = frame.color.open(PixelFormat.RGB24)
            color_image = numpy.frombuffer(color_buffer.data(), dtype=numpy.uint8).reshape((color_buffer.height, color_buffer.width, -1))
            # BGR -> RGB
            color_image = color_image[:,:,::-1]

            try:
                cloud_buffer = frame.computePointCloud()
            except RuntimeError as e:
                logger.warn('{} -> skipping frame'.format(e))
                continue
            cloud = numpy.frombuffer(cloud_buffer, dtype=numpy.float32).reshape((frame.depth.height, frame.depth.width, 3))

            try:
                depth_uv_buffer = frame.computeDepthUVMap()
            except RuntimeError as e:
                logger.warn('{} -> skipping frame'.format(e))
                continue
            depth_uv = numpy.frombuffer(depth_uv_buffer, dtype=numpy.float32).reshape((frame.depth.height, frame.depth.width, 2))

            try:
                color_uv_buffer = frame.computeColorUVMap()
            except RuntimeError as e:
                logger.warn('{} -> skipping frame'.format(e))
                continue
            color_uv = numpy.frombuffer(color_uv_buffer, dtype=numpy.float32).reshape((frame.color.height, frame.color.width, 2))

            # send the frame
            if clients:
                send_frame(clients, (color_image, cloud, depth_uv, color_uv))

            # update the view
            if show and (not clients or n % 10 == 0):
                color_window.update_frame(color_image)
                depth_window.update_frame(colormap[numpy.clip(cloud[:,:,2], 0, len(colormap) - 1).astype(numpy.int)])
                # process events for the windows
                app.processEvents()

            n += 1

            if n % 30 == 0:
                # print some debug stats
                fps = 30 / (time() - mark)
                mark = time()
                logger.debug('{:.1f} fps'.format(fps))

    except KeyboardInterrupt:
        pass

    for (conn, addr) in clients:
        conn.close()
        logger.info('closed connection with {}'.format(addr))

if __name__ == '__main__':
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

    try:
        device_identifier, address, port = parse_args()
    except Exception:
        print sys.argv[0], '[device index | serial]', '[bind address[:port]]'
        raise SystemExit

    sock = setup_socket(address, port)

    camera = setup_camera(device_identifier)
    app = QApplication(sys.argv)

    logger.info('waiting for clients')

    try:
        loop(sock, camera, app, show=show)
    finally:
        try:
            camera.close()
        except Exception as e:
            logger.warn('ignored error while closing camera')
            logger.warn(traceback.format_exc())

    logger.info('exiting')

