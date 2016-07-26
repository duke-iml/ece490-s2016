import logging, traceback
logger = logging.getLogger(__name__)

import struct, numpy

dtype2char = {
    'uint8':    'B',
    'int8':     'b',
    'uint16':   'I',
    'int16':    'i',
    'uint32':   'L',
    'int32':    'l',
    'float32':  'f',
    'float64':  'F',
}

char2dtype = dict(zip(dtype2char.values(), dtype2char.keys()))

def create(images):
    payloads = []
    header = struct.pack('<L', len(images))

    for image in images:
        header += struct.pack('<L', len(image.shape))
        header += struct.pack('<' + 'L'*len(image.shape), *image.shape)

        header += dtype2char[str(image.dtype)]

        data = image.tostring()
        payloads.append(data)
        header += struct.pack('<L', len(data))

    packet = ''.join([ header ] + payloads)

    logger.debug('encoded packet: {}'.format([ '{} {}'.format(i.shape, str(i.dtype)) for i in images ]))

    return packet

def parse(consume):
    def _read(fmt):
        return struct.unpack(fmt, consume(struct.calcsize(fmt)))

    formats = []

    count = _read('<L')[0]
    for i in range(count):
        dims = _read('<L')[0]
        shape = _read('<' + 'L' * dims)
        dtype = char2dtype[consume(1)]
        size = _read('<L')[0]

        formats.append((shape, dtype, size))

    images = []
    for (shape, dtype, size) in formats:
        image = numpy.fromstring(consume(size), numpy.dtype(dtype)).reshape(shape)

        images.append(image)

    logger.debug('decoded packet: {}'.format([ '{} {}'.format(i.shape, str(i.dtype)) for i in images ]))

    return images

