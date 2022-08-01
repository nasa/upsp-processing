import numpy as np
from . import grids

###############################################################################


def read_p3d_grid(filename):
    """Read an unformatted plot3d grid

    Args:
        filename (str)  : unformatted plot3d grid file

    Returns:
        (StructGrid)
    """

    grid = grids.StructGrid()

    with open(filename, "rb") as f:

        # Read in the number of zones
        buf = np.fromfile(f, dtype=np.int32, count=1)
        n_zones = np.fromfile(f, dtype=np.int32, count=1)
        n_zones = n_zones[0]
        buf = np.fromfile(f, dtype=np.int32, count=1)

        # Read in the size of each zone
        buf = np.fromfile(f, dtype=np.int32, count=1)
        zone_sz = np.fromfile(f, dtype=np.int32, count=n_zones * 3)
        buf = np.fromfile(f, dtype=np.int32, count=1)

        total_size = 0
        for i in range(0, n_zones):
            grid.sz.append(zone_sz[3 * i : 3 * i + 3])
            total_size += np.product(grid.sz[i])

        # Read in the data for each zone
        grid.x = np.zeros((total_size,))
        grid.y = np.zeros((total_size,))
        grid.z = np.zeros((total_size,))
        zone_list = []
        curr_idx = 0
        for i in range(0, n_zones):
            zone_size = np.product(grid.sz[i])
            buf = np.fromfile(f, dtype=np.int32, count=1)
            xyz = np.fromfile(f, dtype=np.float32, count=3 * zone_size)
            buf = np.fromfile(f, dtype=np.int32, count=1)  # noqa

            grid.x[curr_idx : curr_idx + zone_size] = xyz[:zone_size]
            grid.y[curr_idx : curr_idx + zone_size] = xyz[zone_size : 2 * zone_size]
            grid.z[curr_idx : curr_idx + zone_size] = xyz[2 * zone_size :]

            # Note the vertex-zone mapping
            for j in range(0, zone_size):
                zone_list.append(i)

            curr_idx += zone_size

        grid.zones = np.array(zone_list, dtype=np.float32)

    return grid


def read_p3d_function(filename):
    """Read in the plot3d function file (first function)

    Args:
        filename (str)  : plot3d binary function file

    Return:
        (np.array) first function in the file (Cp)
    """

    # Open the file for reading
    data = None
    with open(filename, "rb") as f:

        # Read the number of zones
        n_zones = np.fromfile(f, dtype=np.int32, count=1)
        n_zones = n_zones[0]

        # Read the size of each zone
        total_size = 0
        for i in range(0, n_zones):
            ijk = np.fromfile(f, dtype=np.int32, count=3)
            total_size += np.product(ijk)
            n_vars = np.fromfile(f, dtype=np.int32, count=1)  # noqa

        # Read the data
        data = np.fromfile(f, dtype=np.float32, count=total_size)

    return data
