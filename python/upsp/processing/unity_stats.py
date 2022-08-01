#!/usr/bin/env python
#
# Read in the data files needed by Unity. Calculate and write out
# relevant statistics for visualization.
import glob
import logging
import os
import struct
import math
from . import io

import upsp.processing.context as context  # noqa

log = logging.getLogger(__name__)


def write_unity_dataset_with_stats(
    vertex_file, src_dataset_name, dst_dataset_name,
    max_number_frames=100
):
    """Visualization: data and stats files from binary data and write to disk"""

    FLOAT_SIZE_BYTES = 4

    # 1. Figure out the number of model nodes from vertex file size
    number_nodes = os.path.getsize(vertex_file) / FLOAT_SIZE_BYTES
    log.info("Inspected %s to find %d model nodes", vertex_file, number_nodes)

    # 2. Given a number of frames, compute total number of bytes we want
    total_number_bytes = os.path.getsize(src_dataset_name)
    total_number_frames = total_number_bytes / (number_nodes * FLOAT_SIZE_BYTES)
    number_frames = max_number_frames \
        if total_number_frames > max_number_frames \
        else total_number_frames
    number_bytes = int(number_frames * (number_nodes * FLOAT_SIZE_BYTES))

    log.info("[%d frames] x [%d nodes] x [%d bytes per float] = [%d bytes] ~ [%d MB]",
             number_frames, number_nodes, FLOAT_SIZE_BYTES, number_bytes,
             int(number_bytes / 1024.0 / 1024.0))

    # 3. Read, process and dump those bytes to a new file
    input_filename = glob.glob(src_dataset_name)[0]
    output_data_filename = dst_dataset_name + ".bytes"
    output_stats_filename = dst_dataset_name + "_stats.txt"

    log.info("Reading %d bytes from %s, writing to %s", number_bytes,
             input_filename, output_data_filename)

    # chunk_size = 100 * 1024 * 1024
    nchunks, remainder = divmod(number_bytes, FLOAT_SIZE_BYTES)
    chunks = [FLOAT_SIZE_BYTES] * nchunks
    if remainder != 0:
        log.warning("Extra remainder data size: " + remainder)

    max = 0
    min = float("inf")
    total = 0
    number_floats = 0
    mean_est = 0
    M2 = 0

    # Iterate once to get maximum, minimum and total value of raw data
    # and write it out to the output file
    with open(input_filename, "rb") as fi:
        with open(output_data_filename, "wb") as fdo:
            for ch in chunks:
                byte_val = fi.read(ch)
                float_val = struct.unpack("f", byte_val)[0]

                if float_val < min:
                    min = float_val
                if float_val > max:
                    max = float_val

                if not math.isnan(float_val):
                    # log.info("float_val: %f" % float_val)
                    total = total + float_val
                    number_floats = number_floats + 1

                    (mean_est, M2) = update(mean_est, M2, number_floats,
                                            float_val)

                fdo.write(byte_val)

    if number_floats == 0:
        os.remove(output_data_filename)
        return

    stdev = math.sqrt(M2 / number_floats)
    mean = total / number_floats

    # Write statistics results to text file
    with open(output_stats_filename, "w") as fso:
        fso.write(str(max) + "\n")
        fso.write(str(min) + "\n")
        fso.write(str(mean) + "\n")
        fso.write(str(stdev) + "\n")

    os.chmod(output_data_filename, io._CHMOD_URW_GR__O__)
    os.chmod(output_stats_filename, io._CHMOD_URW_GR__O__)


def update(mean, M2, count, newValue):
    delta = newValue - mean
    mean += delta / count
    delta2 = newValue - mean
    M2 += delta * delta2
    return (mean, M2)
