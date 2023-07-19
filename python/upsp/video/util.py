"""Image frame packing/unpacking utilities."""

import numpy


def unpack_10bpp(buf):
    """Unpack 10-bit pixels into a 1D uint16 ndarray."""
    # pixel MSBits first
    # byte 0: [ <- pix 0 - | ---------- ]
    # byte 1: [ ----><---- | -- pix 1 - ]
    # byte 2: [ ---------> | <--pix 2 - ]
    # byte 3: [ ---------- | ----><---- ]
    # byte 4: [ - pix 3 -- | ---------> ]
    pix = numpy.zeros(len(buf) * 8 // 10, dtype=numpy.uint16)
    b = numpy.frombuffer(buf, dtype=numpy.uint8).astype(numpy.uint16)
    pix[::4] = (b[::5] << 2) | (b[1::5] >> 6)
    pix[1::4] = ((b[1::5] & 0x3F) << 4) | (b[2::5] >> 4)
    pix[2::4] = ((b[2::5] & 0x0F) << 6) | (b[3::5] >> 2)
    pix[3::4] = ((b[3::5] & 0x03) << 8) | b[4::5]
    return pix


def unpack_12bpp(buf):
    """Unpack 12-bit pixels into a 1D uint16 ndarray."""
    # pixel MSBits first
    # byte 0: [ <- pix 0 - | ---------- ]
    # byte 1: [ ---------> | <- pix 1 - ]
    # byte 2: [ ---------- | ---------> ]
    pix = numpy.zeros(len(buf) * 8 // 12, dtype=numpy.uint16)
    b = numpy.frombuffer(buf, dtype=numpy.uint8).astype(numpy.uint16)
    pix[::2] = (b[::3] << 4) | (b[1::3] >> 4)
    pix[1::2] = ((b[1::3] & 0x0F) << 8) | b[2::3]
    return pix


def pack_12bpp(pix):
    """Pack a 1D array of pixel values into a 12-bit packed buffer."""
    # byte 0: [ <- pix 0 - | ---------- ]
    # byte 1: [ ---------> | <- pix 1 - ]
    # byte 2: [ ---------- | ---------> ]
    # condition for interpreting as 12-bit unsigned values
    pix[pix < 0] = 0
    pix = pix.astype(numpy.uint16)
    pix[pix > 2**12 - 1] = 2**12 - 1

    buf = numpy.zeros(pix.size * 12 // 8, dtype=numpy.uint8)
    buf[0::3] = pix[0::2] >> 4
    buf[1::3] = ((pix[0::2] & 0x0F) << 4) | (pix[1::2] >> 8)
    buf[2::3] = pix[1::2]

    return buf
