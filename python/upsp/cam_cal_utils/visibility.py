import logging
import os
import numpy as np

import upsp.processing.p3d_utilities as p3d
import upsp.processing.p3d_conversions as p2g
import upsp.raycast

log = logging.getLogger(__name__)


class VisibilityChecker:
    """Visibilty object class for occlusion and viewing angle checking

    This class creates an object for checking visibility of a list of points with
    associated normals. The intended use is for the external calibration to check target
    visibility, and to check visibility of the the model mesh nodes for node-pixel
    mapping.

    For the visibility check, it is assumes that the entire model is within the field of
    view of the camera. This simplifies the check for 2 reasons:

    1. We don't need to check if a node is within the field of view
    2. We don't need to check if an intersection is behind the camera

        - A ray is drawn from a given point to the camera. If that ray intersects a
          node, it is deemed not visible. However, if there is a node behind the camera
          that intersects with the ray, this will still be seen as an intersection and
          flagged as not visible. Even though the intersection behind the camera does
          not truly occlude the given point.

    Parameters
    ----------
    grid_path : str
        Filepath to the grid file
    oblique_angle : float, optional
        Maximum allowable oblique viewing angle. Viewing angle for a node pointed
        directly at the camera and in the center of the field of view of the camera
        is 0 degrees
    epsilon : float, optional
        Intersection tolerance. For the ray tracing intersection check, the origin of
        the ray is offset from the model by a distance of epsilon in the direction of
        the normal
    debug : bool, optional
        Debug parameter to speed up grid loading. It is suggested to use this debug for
        all development, and to leave it False for all non-development case.  If True,
        looks for ``'{{filename}}_primitives.npy'`` in '.cache', where ``filename``
        is the filename from `grid_path`. If it finds it, loads from the ``.npy`` file
        rather than reading and processing the grid file. If it doesn't find it, it will
        read and process the grid file as normal, and save the numpy array as
        ``'{{filename}}_primitives.npy'`` in '.cache'
    debug_nogrid : bool, optional
        Debug parameter to speed up computation. It is suggested to use this debug for
        development if occlusions are not needed for development work. There is no
        reason to leave this parameter for any non-development work. If True, instead of
        loading a real grid file, it loads a fake one with a single, extremely small
        primitive (effectively having 'no grid'). This makes BVH operations much faster.
    """

    def __init__(self, grid_path, oblique_angle=70, epsilon=1e-4, debug=False, debug_nogrid=False):
        # If debug_nogrid is used, create a fake BVH with one very small primitive
        if debug_nogrid:
            primitives = np.array([0.00001, 0.0, 0.0, 0.0, 0.00001, 0.0, 0.0, 0.0, 0.00001])

        # When debug is True (and debug_nogrid is False), we are allowed to cache & load
        #   the grid. This makes debugging faster, but cached items can become stale
        elif debug:
            # Get the filename that we can save the primitives as
            filename = os.path.splitext(os.path.basename(grid_path))[0]
            filename = filename + '_primitives.npy'

            # If the primitives are available, load them
            CACHE_DIR = os.path.join(os.path.expanduser('~'), '.cache', 'upsp')
            CACHE_FILENAME = os.path.join(CACHE_DIR, filename)
            if os.path.exists(CACHE_FILENAME):
                primitives = np.load(CACHE_FILENAME)
                log.debug('Loaded BVH primitives cache "%s"', CACHE_FILENAME)

            # If the primitives are not available, load the grid from scratch and cache it
            else:
                t = self.load_mesh(grid_path)

                # Since the primitives weren't available, package the vertices
                #   and indices into the primitives. Then save them
                primitives = self.package_primitives(t)
                os.makedirs(CACHE_DIR, exist_ok=True)
                np.save(CACHE_FILENAME, primitives)
                log.debug('Cached BVH primitives in "%s"', CACHE_FILENAME)

        # When debug and debug_nogrid are False, load and calculate everything from
        #   scratch. Don't cache any of the results
        else:
            # Load the mesh
            t = self.load_mesh(grid_path)
            primitives = self.package_primitives(t)

        # Store the grid path used for this object
        self.grid_path = grid_path

        # Convert primitives to a list (potentially from a np array)
        primitives = primitives.tolist()

        # Save the epsilon for occlusion checks
        self.epsilon = epsilon

        # Save the square of the cosine of the oblique angle
        #   This will be used for back face culling
        self.update_oblique_angle(oblique_angle)

        # Save the BVH to the instance
        # Simple scene with several triangle primitives
        # [t0p0x, t0p0y, t0p0z, t0p1x, t0p1y, t0p1z, t0p2x, t0p2y, t0p2z, ...]
        # t0 = [0, 0, 1, 1, 0, 1, 0, 1, 1]
        # t1 = [0, 0, 2, 1, 0, 2, 0, 1, 2]
        # primitives = t0 + t1
        self.scene = upsp.raycast.CreateBVH(primitives, 3)

    def update_oblique_angle(self, oblique_angle):
        """Updates object elements related to the oblique viewing angle

        Parameters
        ----------
        oblique_angle : float
            Maximum allowable oblique viewing angle. Viewing angle for a node pointed
            directly at the camera and in the center of the field of view of the camera
            is 0 degrees
        """
        self.oblique_angle = oblique_angle
        self.squared_cos_angle = np.cos(np.deg2rad(oblique_angle))**2

    def load_mesh(self, grid_path):
        """Loads the grid file into vertices and indices

        Parameters
        ----------
        grid_path : string
            Filepath to the grid file

        Returns
        -------
        t : dict
            Dict with keys "vertices" and "indices". ``t["vertices"]`` is a (N, 3) array
            of the model vertices. ``t["indices"]`` is a (N, 3) array of ints where each
            row refers to a model face ``i``. The vertices that make up face ``i`` are
            ``t["vertices"][n]``, where ``n`` is (3, 1) from ``t["indices"][i]``
        """
        grd = p3d.read_p3d_grid(grid_path)
        t = p2g.p3d_to_gltf_triangles(grd)
        t["vertices"] = np.array(t["vertices"])
        t["indices"] = np.array(t["indices"])
        return t

    def package_primitives(self, t):
        """Packages the primitives of the grid file into the BVH format

        Converts the vertices and indices into a list of primitives in the form::

            [t0p0x, t0p0y, t0p0z, t0p1x, t0p1y, t0p1z, t0p2x, t0p2y, t0p2z, ...]

        Parameters
        ----------
        t : dict
            Dict with keys "vertices" and "indices" from :meth:`load_mesh`.

        Returns
        -------
        primitives : np.ndarray
            Packaged primitives
        """

        # Get vertex and face information
        nverts = int(t["vertices"].size / 3)
        nfaces = int(t["indices"].size / 3)
        verts = np.reshape(t["vertices"], (nverts, 3))

        # This is the faces, with the indexes corresponding to the elements of verts
        # Note: faces are counter-clockwise ordered so normals are implicit in faces
        faces_v_idx = np.reshape(t["indices"], (nfaces, 3))

        # Packaged for the BVH
        primitives = []
        for face_v_idx in faces_v_idx:
            # Get the face of the primitive tri
            face = np.array([verts[face_v_idx[0]],
                             verts[face_v_idx[1]],
                             verts[face_v_idx[2]]])

            # If the face does not have 3 unique points, do not include it
            if len(np.unique(face, axis=0)) != 3:
                continue

            # Append the face to the primitive
            primitives.extend(face.ravel().tolist())

        # Convert the list of primitives to a numpy array
        primitives = np.array(primitives)

        return primitives

    def unit_vector(self, vector):
        """Returns the unit vector of the vector

        Helper function for :meth:`angle_between`

        Parameters
        ----------
        vector : np.ndarray, shape (n, 3) float
            Array of vectors

        Returns
        -------
        unit_vector : np.ndarray
            Unit vector(s) corresponding to `vector`

        See Also
        --------
        angle_between : Returns the angle in radians between vectors 'v1' and 'v2'
        """
        return np.divide(vector, np.expand_dims(np.linalg.norm(vector, axis=1), 1))

    def angle_between(self, v1, v2):
        """Returns the angle in radians between vectors `v1` and `v2`

        Parameters
        ----------
        v1 : np.ndarray, shape (n, 3), float
            Vector 1
        v2 : np.ndarray, shape (n, 3), float
            Vector 2

        Returns
        -------
        angle : float
            Angle (in radians) between `v1` and `v2`

        See Also
        --------
        unit_vector : Returns the unit vector of the vector
        is_back_facing : This is the 'slow' version of the back face culling check
        """
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.rad2deg(np.arccos(np.clip(np.sum(np.multiply(v1_u, v2_u), axis=1), -1.0, 1.0)))

    def is_back_facing(self, t, n):
        """This is the 'slow' version of the back face culling check

        This function is mostly for legacy/regression purposes

        TODO: This does not filter points on the horizon of the model (~90 degrees
        oblique viewing angle). It does seem to filter those above 70 degrees (might not
        though, very little testing was done). It might have something to do with the
        clip in angle between

        Parameters
        ----------
        t : np.ndarray, shape (3,) or (3, n), float
            Translation vector from camera to node
        n : np.ndarray, shape (3,) or (3, n), float
            Normal of the node

        Returns
        -------
        back_facing : bool
            True if the node is back facing. False if it is not back facing

        See Also
        --------
        angle_between : Returns the angle in radians between vectors 'v1' and 'v2'
        is_back_facing : This is the 'slow' version of the back face culling check
        is_back_facing_fast_vectorized : Returns array of booleans for which nodes are
            back facing
        """
        if self.angle_between(t, n) > self.oblique_angle:
            return True
        else:
            return False

    def is_back_facing_fast(self, t, n):
        r"""Returns True if the node is back facing, otherwise returns False

        This re-write is significantly faster than the naive approach, but not as fast
        as the vectorized approach. This function is mostly for legacy/regression
        purposes

        The node is back facing if angle between vectors < oblique angle

        Mathematically this is done as:

        .. math::
            \arccos\left(\frac{t \cdot n}{\|t\| \|n\|}\right) < \textrm{oblique angle}

        Since ``arccos`` is an expensive operation, re-write as:

        .. math::
            \frac{t \cdot n}{\|t\| \|n\|} < \cos(\textrm{oblique angle})

        Since division is more expensive than multiplication, re-write as:

        .. math::
              t \cdot n < \|t\| \|n\| \cos(\textrm{oblique angle})

        Using :math:`\|a\|^2 = a \cdot a`, we get:

        .. math::
              (t \cdot n) (|t \cdot n|) <
              (t \cdot t) (n \cdot n) \cos^2(\textrm{oblique angle})

        Where the absolute value  is required on the LHS to preserve sign. All elements
        of RHS are positive since ``t`` and ``n`` contain only real numbers

        :math:`\cos^2(\textrm{oblique angle})` is calculated and cached when the oblique
        angle changes, giving an additional speedup

        Parameters
        ----------
        t : np.ndarray, shape (3, 1) or (3,), float
            Translation vector from camera to node
        n : np.ndarray, shape (3, 1) or (3,), float
            Normal of the node

        Returns
        -------
        back_facing : bool
            True if input is back facing (viewing angle > maximum oblique angle). False
            if input is not back facing (viewing angle <= maximum oblique angle)

        See Also
        --------
        is_back_facing : This is the 'slow' version of the back face culling check
        is_back_facing_fast_vectorized : Returns array of booleans for which nodes are
            back facing
        """
        proj = np.dot(t, n)

        # The RHS is always positive, so if dot(t, n) < 0, then the node is
        #   back-facing
        if (proj < 0):
            return True

        # If the viewing angle is less than 90 degrees, check if it is less than the
        #   max allowable viewing angle
        # Since proj is now non-negative, we can neglect the alsolute sign from step #6
        else:
            return proj * proj < self.squared_cos_angle * np.dot(t, t) * np.dot(n, n)

    def is_back_facing_fast_vectorized(self, t, n):
        """Returns array of booleans for which nodes are back facing

        See :meth:`is_back_facing_fast` for explaination of math. To vectorize
        :math:`a \\cdot b` we use ``np.sum(a*b, axis=1)``

        Parameters
        ----------
        t : np.ndarray, shape (N, 3), float
            Array of translation vectors from camera to nodes
        n : np.ndarray, shape (N, 3), float
            Array of normal vectors of the nodes

        Returns
        -------
        back_facing : np.ndarray, bool
            ``output[i]`` is True if ``node[i]`` is backfacing (viewing angle > maximum
            oblique angle). ``output[i]`` is False if ``node[i]`` is not backfacing
            (viewing angle <= maximum oblique angle).

        See Also
        --------
        is_back_facing : This is the 'slow' version of the back face culling check
        is_back_facing_fast : Returns True if the node is back facing, otherwise returns
            False
        """

        proj = np.sum(t*n, axis=-1)
        return np.where(proj * np.abs(proj) < self.squared_cos_angle * np.sum(t*t, axis=-1) * np.sum(n*n, axis=-1), True, False)

    def does_intersect(self, origin, direction, return_pos=False):
        """Function that determines if a point is occluded by the object mesh

        Creates a ray from origin with given direction. Checks for intersection of ray
        with the BVH

        Parameters
        ----------
        origin : np.ndarray, shape (3, 1), float
            start of ray
        direction : np.ndarray, shape (3, 1), float
            direction of ray

        Returns
        -------
        result : bool
            True if node is occluded and False if node is not occluded
        """
        r = upsp.raycast.Ray(*origin, *direction)
        h = upsp.raycast.Hit()
        result = self.scene.intersect(r, h)

        # If the position was requested, return it
        if return_pos:
            return (result, np.expand_dims(h.pos, 1))

        # Otherwise just return the boolean
        else:
            return result

    def is_visible(self, tvec_model_to_camera, nodes, normals, return_angles=False):
        """Returns list of nodes that are visible

        Currently only checks for oblique viewing angle and occlusion, assumes all nodes
        are within FOV of camera

        Parameters
        ----------
        tvec_model_to_camera : np.ndarray, shape (3, 1), float
            translation vector from model to camera
        nodes : np.ndarray, shape (N, 3), float
            X, Y, Z position of nodes to be checked. ``nodes[i]`` is associated with
            ``normals[i]``
        normals : np.ndarray, shape (N, 3), float
            Normal vectors. ``normals[i]`` is associated with ``nodes[i]``
        return_angles : bool, optional
            If True, return angles for each visible node as well.

        Returns
        ----------
        visible : np.ndarray
            Numpy array of the indices of the nodes that are visible
        angles : np.ndarray
            Angles of visible nodes. Only returned when ``return_angles=True``

        See Also
        ----------
        is_back_facing_fast_vectorized : Returns array of booleans for which nodes are
            back facing
        does_intersect : Function that determines if a point is occluded by the object
            mesh
        """
        # Get the tvecs from the camera to each node
        tvecs = tvec_model_to_camera.T - nodes

        # Get the unit_tvecs and unit_normals
        tvec_norms = np.linalg.norm(tvecs, axis=-1)
        unit_tvecs = tvecs / np.expand_dims(tvec_norms, 1)
        normal_norms = np.linalg.norm(normals, axis=1)
        unit_normals = normals / np.expand_dims(normal_norms, 1)

        # Determine what nodes are back-facing
        #   or nearly back-facing based on oblique_angle
        # If the angles values are not needed, use is_back_facing_fast_vectorized
        # If angles are needed, use the slower
        if not return_angles:
            back_facings = self.is_back_facing_fast_vectorized(unit_tvecs, unit_normals)
        else:
            angles = self.angle_between(unit_tvecs, unit_normals)
            back_facings = np.where(angles > self.oblique_angle, True, False)

        # Get the offset node positions
        epsilon_normals = self.epsilon * unit_normals
        origins = nodes + epsilon_normals

        visible = []
        for i in range(len(nodes)):
            # If it is back-facing, move on to the next node
            if back_facings[i]:
                continue

            # If there is an occlusion, move on to the next node
            #   origin is the ray origin offset so it is close to, but not on the model
            #   tvecs[i] is the ray direction vector that goes from the camera to node i
            if self.does_intersect(origins[i], unit_tvecs[i]):
                continue

            # If the node was not back-facing, and was not occluded, mark it visible
            visible.append(i)

        if return_angles:
            return np.array(visible), np.array(angles[visible])

        return np.array(visible)

    def tri_normal(self, face):
        """Returns the normal of a face with vertices ordered counter-clockwise

        Parameters
        ----------
        face : np.ndarray, shape (3, 3), float
            X, Y, Z positions of the face vertices. ``face[i]`` contains x, y, z of
            vertex ``i``

        Returns
        ----------
        normal : np.ndarray, shape (3,)
            Normal vector of input face
        """
        # https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/geometry-of-a-triangle
        A = face[1] - face[0]  # Edge 1
        B = face[2] - face[1]  # Edge 2
        C = np.cross(A, B)
        return C.reshape(3, 1)

    def get_faces_and_face_normals(self):
        """Returns the faces and face normals of all grid nodes

        Returns
        -------
        faces : np.ndarray, shape (n, 3, 3)
            Each face is a (3, 3) with (i, 3) corresponding to a node, and each node
            having an (x, y, z). Faces are ordered counter-clockwise.
        face_normals : np.ndarray, shape (n, 3)
            The face normal is (x, y, z). ``face_normals[i]`` corresponds to ``face[i]``
        """
        # Get the mesh
        t = self.load_mesh(self.grid_path)

        # Get vertex and face information
        nverts = int(t["vertices"].size / 3)
        nfaces = int(t["indices"].size / 3)
        verts = np.reshape(t["vertices"], (nverts, 3))

        # This is the faces, with the indexes corresponding to the elements of verts
        # Note: faces are counter-clockwise ordered so normals are implicit in faces
        faces_v_idx = np.reshape(t["indices"], (nfaces, 3))

        # Get all the faces and the face normals
        faces = []
        face_normals = []
        for face_v_idx in faces_v_idx: #[np.random.choice(range(len(faces_v_idx)), 25000)]:
            face = np.array([verts[face_v_idx[0]],
                            verts[face_v_idx[1]],
                            verts[face_v_idx[2]]])

            if ((face[0] == face[1]).all() or (face[0] == face[2]).all()
                    or (face[1] == face[2]).all()):
                continue

            faces.append(face)
            face_normals.append(self.tri_normal(face))

        return np.array(faces), np.array(face_normals).reshape(-1, 3)

    def get_tvecs_and_norms(self):
        """Returns the tvecs and tvec normals of all grid nodes

        Returns
        -------
        tvecs : np.ndarray, shape (n, 3)
            Each tvec is corresponds to the (x, y, z) of a node.
        norms : np.ndarray, shape (n, 3)
            Each norm is an (x, y, z) of the node's normal vector ``norms[i]``
            corresponds to ``tvecs[i]``. The norm is the normal of the first face to
            contain the tvec. Each tvec can appear in multiple faces, so the first is
            used. In practice, each face that contains the node will have a different
            normal vector, but in general the difference is a relatively small angle.
        """
        # Get the faces and face normals from the vis_checker
        faces, face_normals = self.get_faces_and_face_normals()

        # Calculate a normal for every node. If a node is on several faces, assume the
        #   normals of all those faces are close enough and pick any one of them
        tvecs, norms = [], []
        node_set = set()
        for face, normal in zip(faces, face_normals):
            for node in face:
                if not tuple(node.tolist()) in node_set:
                    node_set.add(tuple(node.tolist()))
                    tvecs.append(np.array(node))
                    norms.append(np.array(normal))

        # Save the nodes and normals as np arrays
        tvecs = np.array(tvecs)
        norms = np.array(norms)
        return tvecs, norms
