import math
import numpy as np


###############################################################################
class StructGrid:
    """Manage plot3d-style structured grid and write formatted to file

    Attributes:
        sz    : array-like, sizes of each grid [3xN]
        x     : array-like, x-position [N]
        y     : array-like, y-position [N]
        z     : array-like, z-position [N]
        zones : list of vertices ordered by zones [N]
    """

    def __init__(self):
        """ Create empty StructGrid """

        self.sz = []
        self.x = []
        self.y = []
        self.z = []
        self.zones = np.array([], dtype=np.float32)

    def load_grid(self, grid_file):
        """Read a formatted p3d file

        Args:
            grid_file (str) : formatted plot3d file

        Returns:
            None
        """

        with open(grid_file, "r") as f:
            n_zones = int(f.readline())

            for i in range(0, n_zones):
                zone_sz = [int(x) for x in f.readline().split()]
                self.sz.append(zone_sz)

            zone_list = []

            for z in range(0, n_zones):
                total_sz = self.sz[z][0] * self.sz[z][1] * self.sz[z][2]

                vals = []
                curr_sz = 0
                for line in f:
                    # Note the vertex-zone mapping
                    zone_list.append(z)
                    # Load all XYZ values
                    new_vals = [float(x) for x in line.split()]
                    vals.extend(new_vals)
                    curr_sz += len(new_vals)
                    if curr_sz >= 3 * total_sz:
                        break

                self.x.extend(vals[:total_sz])
                self.y.extend(vals[total_sz : 2 * total_sz])
                self.z.extend(vals[2 * total_sz :])

            self.zones = np.array(zone_list, dtype=np.float32)

    def num_zones(self):
        """ Return the number of grids (or zones) """

        return len(self.sz)

    def size(self):
        """ Return the number of grid nodes """

        total_size = 0
        for i in range(0, self.num_zones()):
            total_size += np.product(self.sz[i])

        return total_size

    def num_faces(self, zone=None):
        """Return the number of faces in a zone

        Args:
            zone (int): zone number (0-based)

        Returns:
            (int) if zone = None, number of faces in grid
            else, number of faces in zone

        Raises:
            RuntimeError: invalid zone number
        """

        n_faces = 0

        if zone is None:
            for z in range(0, self.num_zones()):
                n_faces += self.num_faces(zone=z)
        else:
            # Make sure it is a real zone
            if zone < 0 or zone >= len(self.sz):
                raise RuntimeError("Invalid zone number")

            # Get the size (handle surface grids in j,k,or l)
            n_faces = (
                max(1, self.sz[zone][0] - 1)
                * max(1, self.sz[zone][1] - 1)
                * max(1, self.sz[zone][2] - 1)
            )

        return n_faces

    def write_p3d(self, fileout):
        """Write formatted p3d file

        Args:
            fileout (str)   : output file

        Returns:
            None
        """

        with open(fileout, "w") as f:
            f.write("{}\n".format(len(self.sz)))
            for z in range(0, len(self.sz)):
                f.write("{} {} {}\n".format(*self.sz[z]))
            idx = 0
            for z in range(0, len(self.sz)):
                num_pts = self.sz[z][0] * self.sz[z][1] * self.sz[z][2]
                for i in range(0, num_pts):
                    f.write("{:7.4e} ".format(self.x[idx + i]))
                for i in range(0, num_pts):
                    f.write("{:7.4e} ".format(self.y[idx + i]))
                for i in range(0, num_pts):
                    f.write("{:7.4e} ".format(self.z[idx + i]))
                f.write("\n")
                idx += num_pts

    def write_zones_mapping(self, fileout):
        """Write out the binary vertex zones mapping from a plot3d grid

        Args:
            fileout (str)   : output file

        Returns:
            None
        """

        self.zones.tofile(fileout)


class UnstructGrid:
    """Manages triangulated unstructured grid

    Attributes:
        n_comps : number of components
        tris    : array-like, node ids in each triangle [3,T]
        comps   : array-like, component id for each node [N]
        x       : array-like, x-position of each node [N]
        y       : array-like, y-position of each node [N]
        z       : array-like, z-position of each node [N]
    """

    def __init__(self):
        """ Create an empty UnstructGrid """

        self.x = []
        self.y = []
        self.z = []

        self.tris = []  # 2d array of sets of indices
        self.comps = []  # len(comps) == len(tri)

        self.n_comps = 0

    def num_comps(self):
        """ Return the number of components """
        return self.n_comps

    def num_nodes(self):
        """ Return the number of nodes """
        return len(self.x)

    def num_faces(self, comp=None):
        """Return the number of faces in a component

        Args:
            comp (int): component number (id)

        Returns:
            (int) if comp = None, number of faces in grid, else number of
                  faces in component
        """

        n_faces = 0

        if comp is None:
            n_faces = len(self.tris)
        else:
            for i in range(0, len(self.comps)):
                if self.comps[i] == comp:
                    n_faces += 1

        return n_faces

    def extract_comp(self, comp):
        """Extract a sub-grid containing just the component of interest

        Args:
            comp (int): component id

        Returns:
            (UnstructGrid) with just the selected component and mapping of
            old nodes to new nodes
        """

        # Initialize the new grid
        g = UnstructGrid()

        # Create a mapping from nodes to triangles, count the
        # number of faces in the component, and copy over the remaining
        # faces
        n2t = [set() for i in range(self.num_nodes())]

        n_faces = 0
        for t in range(0, len(self.tris)):
            if self.comps[t] == comp:
                n2t[self.tris[t][0]].add(n_faces)
                n2t[self.tris[t][1]].add(n_faces)
                n2t[self.tris[t][2]].add(n_faces)

                g.tris.append(self.tris[t])
                g.comps.append(comp)

                n_faces += 1

        # Remap nodes to new numbering
        n2n = []

        n_nodes = 0
        for n in range(0, len(n2t)):
            if len(n2t[n]) != 0:
                g.x.append(self.x[n])
                g.y.append(self.y[n])
                g.z.append(self.z[n])

                for t in n2t[n]:
                    for i in range(3):
                        if g.tris[t][i] == n:
                            g.tris[t][i] = n_nodes

                n2n.append(n)
                n_nodes += 1

        return g, n2n

    def get_area(self, t):
        """Return the area of a triangle

        Args:
            t (int) : triangle index

        Returns:
            (float) area of the triangle

        Raises:
            RuntimeError    : The triangle index is invalid
        """

        if t < 0 or t >= self.num_faces():
            raise RuntimeError("Invalid triangle, cannot compute area")

        # Get points
        p1 = np.array(
            [self.x[self.tris[t][0]], self.y[self.tris[t][0]], self.z[self.tris[t][0]]]
        )
        p2 = np.array(
            [self.x[self.tris[t][1]], self.y[self.tris[t][1]], self.z[self.tris[t][1]]]
        )
        p3 = np.array(
            [self.x[self.tris[t][2]], self.y[self.tris[t][2]], self.z[self.tris[t][2]]]
        )

        # Use numerically stable Heron's formula
        a = np.linalg.norm(p2 - p1)
        b = np.linalg.norm(p3 - p2)
        c = np.linalg.norm(p3 - p1)

        if b > a:
            tmp = a
            a = b
            b = tmp
        if c > a:
            tmp = a
            a = c
            c = b
        elif c > b:
            tmp = b
            b = c
            c = tmp

        # could be negative due to small numerical error, so shortcut
        pos_neg = abs(c - (a - b))

        return 0.25 * math.sqrt((a + (b + c)) * pos_neg * (c + (a - b)) * (a + (b - c)))
