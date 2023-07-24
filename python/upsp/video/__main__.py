import argparse
import matplotlib.pyplot as plt

from . import video_reader


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("video", help="Path to a video file")
    parser.add_argument("--no-plot", action="store_true", help="Only print metadata")
    args = parser.parse_args(argv)

    with video_reader(args.video) as vid:
        print("Video Properties")
        print(30 * "-")
        for attr, unit in [
            ("frame_count", ""),
            ("frame_rate", "Hz"),
            ("width", "px"),
            ("height", "px"),
            ("bit_depth", "bits"),
            ("raw_bit_depth", "bits"),
        ]:
            val = getattr(vid, attr)
            name = attr.replace("_", " ")
            print(f"{name:20s} {val} {unit}")

        if not args.no_plot:
            img = vid.read_frame(0)

            plt.matshow(
                img,
                vmin=0,
                vmax=2**vid.bit_depth - 1,
                interpolation="none",
                cmap="magma",
            )
            plt.colorbar()
            plt.show()


if __name__ == "__main__":
    main()
