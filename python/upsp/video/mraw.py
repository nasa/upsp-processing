"""Reader for Photron MRAW video files."""
import os

from .base import VideoReader
from .util import unpack_10bpp, unpack_12bpp


class MrawReader(VideoReader):
    """Photron MRAW file reader.

    Parameters
    ----------
    path : path-like
        Path to the .mraw file
    cih_path : path-like, optional
        Path to the corresponding .cih file. If not provided, a file with the same base
        name and path as the .mraw file is used.

    Attributes
    ----------
    cih_info : dict
        Information parsed from the .cih file is stored in a dictionary with key names
        equivalent to the field names in the file.
    """

    def __init__(self, path, cih_path=None):
        super().__init__(path)
        self.cih_path = cih_path
        if cih_path is None:
            base, _ = os.path.splitext(path)
            self.cih_path = f"{base}.cih"
        self.cih_info = {}

    def initialize(self):
        """Read the CIH file"""
        self.cih_info = parse_cih(self.cih_path)

        if self.bit_depth not in (10, 12):
            raise NotImplementedError(
                "Unpacking {}-bit imgs not supported yet".format(self.bpp)
            )

    @property
    def width(self):
        return self.cih_info["Image Width"]

    @property
    def height(self):
        return self.cih_info["Image Height"]

    @property
    def bit_depth(self):
        return self.cih_info["Color Bit"]

    @property
    def frame_rate(self):
        return self.cih_info["Record Rate(fps)"]

    @property
    def frame_count(self):
        return self.cih_info["Total Frame"]

    def read_frame(self, idx):
        self._validate_index(idx)

        npix = self.width * self.height
        img_size = self.bit_depth * npix // 8

        seekpos = idx * img_size
        if self.fd.tell() != seekpos:
            self.fd.seek(seekpos)

        buf = self.fd.read(img_size)

        if self.bit_depth == 12:
            pix = unpack_12bpp(buf)
        else:
            pix = unpack_10bpp(buf)
        img = pix.reshape((self.height, self.width))
        return img


def parse_cih(path):
    """Parse a CIH file

    Assumes the following file format::

        # comment
        Field Name : Value

    Values are strings unless they can be converted to an int or float.

    Parameters
    ----------
    path : path-like
        Path to the file to parse.

    Returns
    -------
    data : dict
        Key-value pairs in the CIH file.
    """
    data = {}
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#") and not line.startswith("END"):
                key, val = line.split(":", 1)
                key = key.strip()
                val = val.strip()
                try:
                    val = int(val)
                except ValueError:
                    try:
                        val = float(val)
                    except ValueError:
                        pass
                    pass
                data[key] = val
    return data
