import numpy as np
import scipy.spatial


class VertexSearch:
    """Vertex nearest-neighbor queries for structured/unstructured grids"""
    def __init__(self, grid):
        grid_vertices = np.transpose(np.asarray([grid.x, grid.y, grid.z]))
        self.grid = grid
        self.tree = scipy.spatial.cKDTree(grid_vertices)

    def query_ball_point(self, *args, **kwargs):
        return self.tree.query_ball_point(*args, **kwargs)

    def query(self, *args, **kwargs):
        return self.tree.query(*args, **kwargs)
