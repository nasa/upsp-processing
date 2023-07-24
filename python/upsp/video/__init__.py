"""Video file reading and parsing

Typical usage of this subpackage involves the use of the high-level :func:`video_reader`
context manager, which accepts a path to a video file and yields an appropriate reader
object.

For a Phantom Cine file (:class:`~upsp.video.cine.CineReader`)::

    from upsp.video import video_reader

    with video_reader("video.cine") as reader:
        nframes = reader.frame_count
        imgs = reader.read_frames(10)

Or for a Photron MRAW file (:class:`~upsp.video.mraw.MrawReader`)::

    with video_reader("video.mraw") as reader:
        bpp = reader.bit_depth
        img = reader.read_frame(0)

This subpackage also functions as a tool for inspecting a video, displaying some of the
video properties and (optionally) displaying the first frame::

    $ python -m upsp.video video.cine
"""

import contextlib
from pathlib import Path

from .base import VideoReader
from .cine import CineReader
from .mraw import MrawReader


@contextlib.contextmanager
def video_reader(path):
    """Retrieve an appropriate video file reader for a given video file

    Parameters
    ----------
    path : path-like
        Path to the video.

    Yields
    ------
    reader : ~upsp.video.base.VideoReader
        An open video reader instance.
    """
    path = path if isinstance(path, Path) else Path(path)
    ext = path.suffix

    if ext == ".cine":
        cls = CineReader
    elif ext in {".mraw", ".cih"}:
        cls = MrawReader
    else:
        raise ValueError(f"Video file with extension {ext} not supported")

    with cls(path) as reader:
        yield reader
