import logging
import numpy as np
import scipy.spatial.transform as tf

from ..processing import plot3d
from ..processing import kulite_utilities
from . import spatial_queries

log = logging.getLogger(__name__)


class KuliteNeighborhoodSearch:
    def __init__(self, ctx):
        self.grid = plot3d.read_p3d_grid(ctx.inputs["grid_file"])
        self.search = spatial_queries.VertexSearch(self.grid)
        log.info('Indexed grid file "%s"', ctx.inputs["grid_file"])
        self.tgts = kulite_utilities.read_tgts(ctx.inputs["targets_file"])
        log.info('Loaded tgts file "%s"', ctx.inputs["targets_file"])

    def query(
        self,
        kulite_name,
        direction,
        displacement,
        number_vertices,
        duplicate_vertex_tol=2e-2,
    ):
        kulite_position = self.tgts[kulite_name]
        log.info(
            "%s position: %s, duplicate_vertex_tol=%f",
            kulite_name, str(kulite_position), duplicate_vertex_tol
        )
        selection_position = selection_area_center(
            kulite_position, direction, displacement
        )
        _, kulite_nn_index = self.search.query(kulite_position, k=1)
        # If duplicate_vertex_tol is not None, then we should
        # skip any vertices that are within that radius of another vertex
        # in the neighbor search.
        #
        # Implementation is not immediately obvious because we must
        # specify the number of neighbors as input to the NN search.
        # Current solution is to always query slightly more vertices than
        # what the user requests, and then increase incrementally until
        # we find enough unique vertices.
        search_increase_factor = 1.5
        number_model_vertices = self.grid.x.size
        number_query_vertices = int(number_vertices * search_increase_factor)
        selection_nn_indices = []
        while number_query_vertices < number_model_vertices:
            number_query_vertices = int(number_query_vertices * search_increase_factor)
            _, selection_nn_indices = self.search.query(
                selection_position, k=number_query_vertices
            )
            selection_nn_indices = [int(v) for v in np.atleast_1d(selection_nn_indices)]
            if duplicate_vertex_tol is not None:
                selection_nn_indices = self.filter_duplicates(
                    selection_nn_indices,
                    duplicate_vertex_tol=duplicate_vertex_tol
                )
            if len(selection_nn_indices) >= number_vertices:
                selection_nn_indices = selection_nn_indices[:number_vertices]
                break

        return {
            "Kulite Nearest Vertex": int(kulite_nn_index),
            "Selection Vertices": selection_nn_indices,
        }

    def filter_duplicates(self, vertices, duplicate_vertex_tol=2e-2):
        # Initialize output list: []
        # Iterating through node idxs 0 ... N
        #  - Find all neighbor nodes within tolerance radius of node
        #    - Use existing KDtree
        #  - If any neighbors are already in output list, skip this node
        #  - Else, add this node to output list
        pts = [
            np.array([self.grid.x[ii], self.grid.y[ii], self.grid.z[ii]])
            for ii in vertices
        ]
        results = self.search.query_ball_point(pts, duplicate_vertex_tol)
        filtered = []
        for ii, neighbor_list in zip(vertices, results):
            is_in_filtered_list = [idx in filtered for idx in neighbor_list]
            if any(is_in_filtered_list):
                continue
            filtered.append(ii)
        return filtered


def selection_area_center(p0, direction, surface_displacement):
    # port + starboard wrt direction of nose of vehicle
    # By definition, +X model axis is aligned with freestream,
    # so port := +rotation around X axis (right-handed)
    forward = np.array([-1.0, 0.0, 0.0])
    aftward = np.array([+1.0, 0.0, 0.0])
    distance_from_centerline = np.sqrt(p0[1] ** 2 + p0[2] ** 2)
    rotation_degrees = np.degrees(surface_displacement / distance_from_centerline)

    def rotate_around(p, axis, degrees):
        radians = np.radians(degrees)
        rotation = tf.Rotation.from_rotvec(radians * axis)
        return rotation.apply(p)

    offset_positions = {
        "up": p0 + surface_displacement * forward,
        "down": p0 + surface_displacement * aftward,
        "starboard": rotate_around(p0, forward, rotation_degrees),
        "port": rotate_around(p0, forward, -rotation_degrees),
    }
    return offset_positions[direction.lower()]
