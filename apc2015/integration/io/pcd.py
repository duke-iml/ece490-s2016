import logging, traceback
logger = logging.getLogger(__name__)

from struct import unpack_from, unpack, pack, calcsize
from cStringIO import StringIO

def _try_parse(s):
    try:
        v = int(s)
    except ValueError:
        pass
    else:
        return v

    try:
        v = float(s)
    except ValueError:
        pass
    else:
        return v

    return s

format_map = { '4f': 'f',
               '8f': 'd',
               '1u': 'B',
               '2u': 'H'
             }

def write(point_cloud, s):
    if isinstance(s, str):
        s = open(s, 'wb')

    if len(point_cloud[0]) == 2:
        point_cloud = map(lambda p: [ p[0][0], p[0][1], p[0][2], p[1] ], point_cloud)
    if len(point_cloud[0]) != 4 or len(point_cloud[0][3]) != 3:
        raise Exception('invalid PCD point format')

    s.write('VERSION .7\n')
    s.write('FIELDS x y z rgb\n')
    s.write('SIZES 4 4 4 4\n')
    s.write('TYPE F F F F\n')
    s.write('COUNT 1 1 1 1\n')
    s.write('WIDTH {}\n'.format(len(point_cloud)))
    s.write('POINTS {}\n'.format(len(point_cloud)))
    s.write('HEIGHT 1\n')
    s.write('DATA ascii\n')

    point_cloud = encode_color(point_cloud)

    for point in point_cloud:
        s.write(' '.join(map(str, point)) + '\n')

def read(s):
    return parse(s)

def parse(raw):
    header = parse_header(raw)

    if header['fields'][:3] != [ 'x', 'y', 'z' ]:
        raise Exception('invalid PCD point format')

    if header['data'] == 'binary':
        point_cloud =  parse_binary_data(header, raw)
    else:
        point_cloud = parse_ascii_data(header, raw)

    if len(header['fields']) > 3 and header['fields'][3] == 'rgb':
        point_cloud = decode_color(point_cloud)
        logger.debug('decoded PCD color channel')

    return (header, point_cloud)

def parse_header(raw):
    # read in header
    header = {}
    while 'data' not in header:
        p = raw.readline().strip().split()
        header[p[0].lower()] = [ s.lower() for s in p[1:] ]

    # parse the header
    for k, v in header.items():
        # collapse single-element lists
        if isinstance(v, list):
            header[k] = [ _try_parse(x) for x in v ]

            if len(v) == 1:
                header[k] = header[k][0]

        # convert to integer/float types
        elif isinstance(v, str):
            header[k] = _try_parse(s)

    logger.debug('parsed PCD header: {}'.format(header))
    return header

def parse_binary_data(header, raw):
    # build the point format
    point_format = ''
    for w, t in zip(header['size'], header['type']):
        point_format += format_map['{0}{1}'.format(w, t)]

    point_size = calcsize(point_format)

    # read in points
    point_cloud = []
    for i in range(0, header['points']):
        point = unpack_from(point_format, raw.read(point_size))
        point_cloud.append(point)

    logger.debug('parsed binary PCD: {} points'.format(len(point_cloud)))
    return point_cloud

def parse_ascii_data(header, raw):
    # read in points
    point_cloud = []
    for i in range(0, header['points']):
        point = map(float, raw.readline().strip().split(' '))
        point_cloud.append(point)

    logger.debug('parsed ASCII PCD: {} points'.format(len(point_cloud)))
    return point_cloud

def decode_color(point_cloud):
    for point in point_cloud:
        point[3] = unpack('BBBx', pack('f', point[3]))[::-1]

    return point_cloud

def encode_color(point_cloud):
    return [ p[0:3] + [ unpack('f', pack('BBBx', *p[3][::-1]))[0] ] for p in point_cloud ]