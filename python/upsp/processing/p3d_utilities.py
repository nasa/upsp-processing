import math
import numpy as np

################################################################################

def to_cylindrical(y,z):
    """
    moves y,z to cylindrical (using x,r,theta)
    theta returned in degrees
    """
    r = np.sqrt(y ** 2 + z ** 2)
    theta = np.arctan2(y, z) * 180.0 / np.pi
    return (r, theta)

def to_cartesian(r,theta):
    """
    moves r,theta (in degrees) to cartesian y,z
    """
    y = r * np.sin(theta * np.pi / 180.0)
    z = r * np.cos(theta * np.pi / 180.0)
    return (y, z)

class StructGrid:
    """Manage plot3d-style structured grid and write formatted to file

    Attributes
    ----------
    x, y, z : array_like
        Cartesian coordinates [N]
    r, theta : array_like
        Cylindrical coordinates [N]
    """
    
    def __init__(self):
        self.sz = []
        self.x = []
        self.y = []
        self.z = []
        self.r = []
        self.theta = []

    def reduce_grid(self,good_indices):
        """
        try to reduce all member variables to those covered by good_indices. I have no idea what this does to the sz variable :(
        NOT CURRENTLY ROBUST TO OOB ERRORS :(
        """
        self.x = self.x[good_indices]
        self.y = self.y[good_indices]
        self.z = self.z[good_indices]
        self.r = self.r[good_indices]
        self.theta = self.theta[good_indices]


    def load_grid(self, grid_file):
        """ Read a formatted p3d file

        Parameters
        ----------
        grid_file : str
            formatted plot3d file
        """
        with open(grid_file, 'r') as f:
            sz = []
            n_zones = int(f.readline())
            for i in range(0,n_zones):
                zone_sz = [int(x) for x in f.readline().split()]
                sz.append(zone_sz)
   
            for z in range(0, n_zones): 
                total_sz = sz[z][0] * sz[z][1] * sz[z][2]

                # Load all XYZ values
                vals = []
                curr_sz = 0
                for line in f:
                    new_vals = [float(x) for x in line.split()]
                    vals.extend(new_vals)
                    curr_sz += len(new_vals)
                    if curr_sz >= 3*total_sz:
                        break

                self.x.extend(vals[:total_sz])
                self.y.extend(vals[total_sz:2*total_sz])
                self.z.extend(vals[2*total_sz:])
        self.r, self.theta = to_cylindrical(self.y,self.z)

def read_p3d_grid(filename):
    """Read an unformatted plot3d grid

    Parameters
    ----------
    filename : str
        unformatted plot3d grid file

    Returns
    -------
    StructGrid
        grid
    """

    grid = StructGrid()

    with open(filename, 'rb') as f:

        # Read in the number of zones
        buf = np.fromfile(f, dtype=np.int32, count=1)
        n_zones = np.fromfile(f, dtype=np.int32, count=1)
        n_zones = n_zones[0]
        buf = np.fromfile(f, dtype=np.int32, count=1)

        # Read in the size of each zone
        buf = np.fromfile(f, dtype=np.int32, count=1)
        zone_sz = np.fromfile(f, dtype=np.int32, count=n_zones*3)
        buf = np.fromfile(f, dtype=np.int32, count=1)

        total_size = 0
        for i in range(0, n_zones):
            grid.sz.append(zone_sz[3*i:3*i+3])
            total_size += np.product(grid.sz[i])

        # Read in the data for each zone
        grid.x = np.zeros((total_size,))
        grid.y = np.zeros((total_size,))
        grid.z = np.zeros((total_size,))
        curr_idx = 0
        for i in range(0, n_zones):
            zone_size = np.product(grid.sz[i])
            buf = np.fromfile(f, dtype=np.int32, count=1)
            xyz = np.fromfile(f, dtype=np.float32, count=3*zone_size)
            buf = np.fromfile(f, dtype=np.int32, count=1)
   
            grid.x[curr_idx:curr_idx+zone_size] = xyz[:zone_size]
            grid.y[curr_idx:curr_idx+zone_size] = xyz[zone_size:2*zone_size]
            grid.z[curr_idx:curr_idx+zone_size] = xyz[2*zone_size:]

            curr_idx += zone_size
    grid.r, grid.theta = to_cylindrical(grid.y,grid.z)

    return grid

def read_p3d_function(filename):
    """Read in the plot3d function file (first function)

    Parameters
    ----------
    filename : str
        plot3d binary function file

    Returns
    -------
    np.ndarray
        first function in the file (Cp)
    """

    # Open the file for reading
    data = None
    with open(filename, 'rb') as f:
        
        # Read the number of zones
        n_zones = np.fromfile(f, dtype=np.int32, count=1)
        n_zones = n_zones[0]

        # Read the size of each zone
        total_size = 0
        for i in range(0, n_zones):
            ijk = np.fromfile(f, dtype=np.int32, count=3)
            total_size += np.product(ijk)

            n_vars = np.fromfile(f, dtype=np.int32, count=1)

        # Read the data
        data = np.fromfile(f, dtype=np.float32, count=total_size)

    return data

