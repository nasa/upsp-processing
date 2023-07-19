"""Generic video reader abstraction layer"""

import abc

import numpy


class VideoReader(abc.ABC):
    """Video reader interface.

    This provides filetype-specific readers with default implementations of file
    handling, so they can uniformly be used with the context manager pattern, e.g.::

        with ReaderImpl("some_file.ext") as vid:
            print(vid.frame_count)
            vid.read_frame(0)

    Parameters
    ----------
    path : path-like
        Path to the video file to read.
    """

    dtype = numpy.uint16

    def __init__(self, path):
        self.path = path
        self.fd = None

    def open(self):
        """Open the video to begin reading

        This is normally handled via context manager.
        """
        self.fd = open(self.path, "rb")
        self.initialize()

    def close(self):
        """Close the video, preventing further reading

        This is normally handled via context manager.
        """
        if self.fd is not None:
            self.fd.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def _validate_index(self, idx):
        """Check if the video has a frame with a given index

        Raises
        ------
        ValueError : `idx` is out of range
        """
        if idx >= self.frame_count:
            raise ValueError(
                f"Invalid frame index {idx}. File has {self.frame_count} frames"
            )

    @property
    @abc.abstractmethod
    def frame_count(self):
        """Total number of frames in the file"""
        return

    @property
    @abc.abstractmethod
    def frame_rate(self):
        """Video frame rate, in Hz"""
        return

    @property
    @abc.abstractmethod
    def width(self):
        """Video frame width, in pixels"""
        return

    @property
    @abc.abstractmethod
    def height(self):
        """Video frame height, in pixels"""
        return

    @property
    @abc.abstractmethod
    def bit_depth(self):
        """Bit depth of frame data, following any post-processing

        This property should be used for determining the range of values possible in
        frame data returned by :meth:`read_frame` (i.e.  :math:`[0, 2^d - 1]`), since it
        represents the bit depth following any post-processing operations such as a
        lookup table expansion.

        See Also
        --------
        VideoReader.raw_bit_depth
            Bit depth as stored in the file, prior to any processing.
        """
        return

    @property
    def raw_bit_depth(self):
        """Bit depth of frame data as stored in the file"""
        return self.bit_depth

    @abc.abstractmethod
    def initialize(self):
        """One-time initialization performed on file open

        Concrete implementations should perform any operations necessary to obtain video
        properties such as bit depth, frame size, etc.
        """
        return

    @abc.abstractmethod
    def read_frame(self, idx):
        """Read a single frame by index.

        Parameters
        ----------
        idx : int
            Index of the frame to read.

        Returns
        -------
        img : ndarray, shape (height, width)
        """
        return

    def read_frames(self, nframes, start=0, out=None):
        """Read multiple consectutive frames from the video

        Parameters
        ----------
        nframes : int
            Number of frames to read.
        start : int, optional
            First frame to read. Default is the first frame in the file.
        out : ndarray, optional
            Array to place results in. Must have the correct shape (nframes, height,
            width). The dtype may be different from that of the reader, in which case
            casting is done on the fly (per frame).

        Returns
        -------
        out : ndarray, shape (nframes, height, width)

        Notes
        -----
        Subclasses may want to override this method for efficiency (e.g. one contiguous
        read). The base implementation simply calls :meth:`read_frames` in a loop.
        """
        # subclasses may want to override for efficiency
        self._validate_index(start + nframes - 1)
        shape = (nframes, self.height, self.width)
        if out is None:
            out = numpy.empty((nframes, self.height, self.width), dtype=self.dtype)
        elif out.shape != shape:
            raise ValueError(f"Expected shape {shape}, got {out.shape}")

        for i, f in enumerate(range(start, start + nframes)):
            out[i] = self.read_frame(f)

        return out

    def iter_frames(self, nframes, start=0, frames_per_chunk=100, dtype=None):
        """Iterate over chunks of frames in a video

        Parameters
        ----------
        nframes : int
            Total number of frames to read.
        start : int, optional
            First frame to read. Default is the first frame in the file.
        frames_per_chunk : int, optional
            Number of frames to read per iteration. The final iteration may produce
            fewer than `frames_per_chunk` if `nframes` is not an integer multiple.
        dtype : dtype, optional
            Data type to return. See :meth:`read_frames` for handling of conversion.

        Yields
        ------
        imgs : ndarray, shape (nframes_in_chunk, height, width)
            Image frames. This is a view of an array that is recycled on each read, so
            it may need to be copied each time, depending on usage.
        """
        self._validate_index(start + nframes - 1)
        shape = (min(frames_per_chunk, self.frame_count), self.height, self.width)
        dtype = self.dtype if dtype is None else dtype
        out_buf = numpy.empty(shape, dtype=dtype)
        for iframe in range(start, start + nframes, frames_per_chunk):
            iframe_end = min(iframe + frames_per_chunk, start + nframes)
            frames_to_read = min(frames_per_chunk, iframe_end - iframe)
            yield self.read_frames(
                frames_to_read, start=iframe, out=out_buf[:frames_to_read]
            )
