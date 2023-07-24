import unittest
from pathlib import Path

from upsp.video import MrawReader, video_reader

# test/python -> cpp/test/mraw
here = Path(__file__).parent
mraw_dir = here.parent.parent / "cpp" / "test" / "mraw"


class VideoReaderTest(unittest.TestCase):
    def test_video_reader_mraw(self):
        with video_reader(mraw_dir / "12bitMRAW.mraw") as vid:
            self.assertEqual(vid.frame_count, 2)
            vid.read_frame(0)


class MrawTest(unittest.TestCase):
    def test_12bit(self):
        with MrawReader(mraw_dir / "12bitMRAW.mraw") as vid:
            # properties
            self.assertEqual(vid.height, 1024)
            self.assertEqual(vid.width, 1024)
            self.assertEqual(vid.bit_depth, 12)
            self.assertEqual(vid.frame_count, 2)

            # frame reading
            f0 = vid.read_frame(1)
            vid.read_frames(2)

            self.assertEqual(f0.shape, (1024, 1024))


if __name__ == "__main__":
    unittest.main()
