import ast
import glob
import logging
import os
import numpy as np
import pandas as pd
import cv2

from ..processing import context

log = logging.getLogger(__name__)

# avoid importing all of matplotlib...
color_choices = [
    np.array(c[::-1], dtype=int)
    for c in [
        (141, 211, 199),
        (255, 255, 179),
        (190, 186, 218),
        (251, 128, 114),
        (128, 177, 211),
        (253, 180, 98),
        (179, 222, 105),
        (252, 205, 229),
        (217, 217, 217),
        (188, 128, 189),
        (204, 235, 197),
        (255, 237, 111),
    ]
]
kulite_color_choices = [c - 70 for c in color_choices]
kulite_color_idxs = {}


def highlight_pixels(image: np.ndarray, pixels: list, bgr: tuple):
    for y, x in pixels:
        image[y, x, :] = bgr
    return image


def make_selection_area_images(df: pd.DataFrame, out_dir=None):
    """Highlight selected Kulite comparison pixels in raw uPSP images

    Parameters
    ----------
    df : pandas.DataFrame
        Records with the following columns:

        - 'Pipeline Directory' : data pipeline top-level directory
        - 'Datapoint' : datapoint identifier
        - 'Kulite Nearest Vertex' : index of vertex nearest to Kulite
        - 'Kulite Name' : name of kulite
        - 'Selection Vertices' : string repr of list of indices of selected vertices,
          e.g., "[124,231,101]"
    out_dir : str or None
        Directory for writing output images. If None, defaults to subdirectories of the
        current working directory named according to the 'Pipeline Directory' names.
    """
    if out_dir is None:
        out_dir = os.getcwd()

    cache = {}
    color_choice_idx = 0
    for _, row in df.iterrows():
        root_dir = row["Pipeline Directory"]
        datapoint_name = row["Datapoint"]
        kulite_vertex = row["Kulite Nearest Vertex"]
        kulite_name = row["Kulite Name"]
        vertices = ast.literal_eval(row["Selection Vertices"])
        if root_dir not in cache:
            cache[root_dir] = {"ctx": context.Pipeline(root_dir), "plots": {}}
        ctx = cache[root_dir]["ctx"]
        plot_cache = cache[root_dir]["plots"]
        dp_ctx = ctx.datapoint(datapoint_name)
        image_flist = glob.glob(dp_ctx.output_path("psp_process", "cam*_raw.png"))
        for image_filename in image_flist:
            camera_number = int(os.path.basename(image_filename)[3:5], base=10)
            image_filename = dp_ctx.output_path(
                "psp_process", "cam%02d_raw.png" % camera_number
            )
            plot_dir = os.path.basename(os.path.normpath(ctx.root_dir))
            plot_filename = os.path.join(
                plot_dir,
                "%s-cam%0d-selected-pixels.png" % (dp_ctx.name, camera_number),
            )
            if plot_filename not in plot_cache:
                im = cv2.imread(image_filename)
                plot_cache[plot_filename] = im
            im = plot_cache[plot_filename]
            height, width, depth = im.shape
            uv_filename = dp_ctx.output_path(
                "psp_process", "cam%02d_uv" % camera_number
            )
            uv = np.fromfile(uv_filename, dtype=np.float32)

            def highlight_vertex_pixels(vertex_indices, bgr):
                vertex_uv = [(uv[2 * ii], uv[2 * ii + 1]) for ii in vertex_indices]
                vertex_yx = []
                for (
                    _idx,
                    _uv,
                ) in zip(vertex_indices, vertex_uv):
                    if _uv[0] > 0 and _uv[1] > 0:
                        _yx = round(height * _uv[1]), round(width * _uv[0])
                        log.info(
                            "%s, Camera %02d: Vertex %d -> (u,v) = %s, (y, x) = %s",
                            datapoint_name,
                            camera_number,
                            _idx,
                            _uv,
                            _yx,
                        )
                        vertex_yx.append(_yx)
                return highlight_pixels(im, vertex_yx, bgr)

            if kulite_name not in kulite_color_idxs:
                kulite_color_idxs[kulite_name] = color_choice_idx
                color_choice_idx = (color_choice_idx + 1) % len(color_choices)
            idx = kulite_color_idxs[kulite_name]
            im = highlight_vertex_pixels(vertices, color_choices[idx])
            im = highlight_vertex_pixels([kulite_vertex], kulite_color_choices[idx])

    for cache_data in cache.values():
        plot_cache = cache_data["plots"]
        for plot_filename, im in plot_cache.items():
            os.makedirs(os.path.dirname(plot_filename), exist_ok=True)
            cv2.imwrite(plot_filename, im)
            log.info("Wrote %s", plot_filename)
