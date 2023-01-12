import cv2
import numpy as np
import copy
from scipy.spatial.transform import Rotation as scipy_Rotation

np.set_printoptions(linewidth=180, precision=4, threshold=np.inf)

#---------------------------------------------------------------------------------------
# General Photogrammetry Utility Functions


def rot(angle, axis):
    """Rotation matrix for angle-axis transformation

    An identity matrix is rotated about the given axis by the given angle (in degrees)

    Parameters
    ----------
    angle : float
        angle of rotation (in degrees) about the given axis
    axis : {'x', 'y', 'z'}
        axis to rotate about

    Returns
    ----------
    np.ndarray, shape (3, 3), float
        Rotation matrix of an identity matrix rotated about the given axis by the given
        amount
    """

    assert_error_msg = "Error in photogrammetry.rot. Axis must be 'x', 'y', or 'z'."
    assert axis in ['x', 'y', 'z'], assert_error_msg

    angle_rad = np.deg2rad(angle)

    if axis == 'x':
        return np.array([[1,                 0,                  0],
                         [0, np.cos(angle_rad), -np.sin(angle_rad)],
                         [0, np.sin(angle_rad),  np.cos(angle_rad)]])

    if axis == 'y':
        return np.array([[np.cos(angle_rad), 0,   np.sin(angle_rad)],
                        [0,                  1,                   0],
                        [-np.sin(angle_rad), 0,   np.cos(angle_rad)]])

    if axis == 'z':
        return np.array([[np.cos(angle_rad), -np.sin(angle_rad), 0],
                         [np.sin(angle_rad),  np.cos(angle_rad), 0],
                         [0,                  0,                 1]])


def invTransform(R, t):
    """Returns the inverse transformation of the given `R` and `t`

    Parameters
    ----------
    R : np.ndarray, shape (3, 3), float
        Rotation Matrix
    t : np.ndarray (3, 1), float
        Translation Vector

    Returns
    ----------
    rmat_i : np.ndarray, shape (3, 3), float
        Inverse rotation matrix
    tvec_i : np.ndarray, shape (3, 1), float
        Inverse translation vector
    """

    R_transpose = R.transpose()
    return (R_transpose, -np.matmul(R_transpose, t))


def isRotationMatrix(R):
    """Checks if a matrix is a valid rotation matrix

    Parameters
    ----------
    R : np.ndarray, shape (3, 3), float
        Rotation Matrix

    Returns
    ----------
    bool
        True if R is a valid rotation matrix, and False if it is not
    """

    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    n = np.linalg.norm(np.identity(3, dtype=R.dtype) - shouldBeIdentity)
    return n < 1e-6


def isRotationMatrixRightHanded(R):
    """ Returns True if matrix is right handed, and False it is not

    Parameters
    ----------
    R : np.ndarray (3, 3), float
        Rotation Matrix

    Returns
    ----------
    boolean
        True if R is a right handed rotation matrix, and False if it is not
    """
    return np.dot(np.cross(R[:,0], R[:,1]), R[:,2]) > 0.


def rotationMatrixToTunnelAngles(R):
    """Converts rotation matrix to tunnel angles

    Parameters
    ----------
    R : np.ndarray, shape (3, 3), float
        Rotation Matrix

    Returns
    ----------
    np.ndarray (3, 1), float
        Array of tunnel angle (alpha, beta, phi)

    See Also
    ----------
    isRotationMatrix : Checks if a matrix is a valid rotation matrix
    """
    # Check that the given matrix is a valid roation matrix
    assert(isRotationMatrix(R))

    # Use scipy to convert to yzx euler angles
    r = scipy_Rotation.from_matrix(R)
    alpha, beta, neg_phi = np.array(r.as_euler('yzx', degrees=True))

    # Return the tunnel angles
    return np.expand_dims([alpha, beta, -neg_phi], 1)


def transform_3d_point(rmat, tvec, obj_pts):
    """Transform 3D points from the object frame to the camera frame.

    `rmat` and `tvec` are the transformation from the camera to the object's frame.
    ``obj_pts[i]`` is the position of point ``i`` relative to the object frame. The
    function returns the points relative to the camera.

    Parameters
    ----------
    rmat : np.ndarray, shape (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray (3, 1), float
        Translation vector from camera to object
    obj_pts : np.ndarray (n, 3), float
        List-like of 3D positions. The positions are relative to the object frame
        defined by rmat and tvec

    Returns
    ----------
    np.ndarray (n, 3) of floats
        Array of transformed points. return[i] is the transformed obj_pts[i]
    """
    obj_pts_tf = np.squeeze(np.array([np.matmul(rmat, np.expand_dims(pt, 1)) + tvec for pt in obj_pts]), axis=2)
    return obj_pts_tf


def project_3d_point(rmat, tvec, cameraMatrix, distCoeffs, obj_pts, ret_jac=False, ret_full_jac=False):
    """Projects targets into an image.

    Parameters
    ----------
    rmat : np.ndarray, shape (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray, shape (3, 1), float
        Translation vector from camera to object
    cameraMatrix : np.ndarray, shape (3, 3), float
        The (openCV formatted) camera matrix for the camera
    distCoeffs : np.ndarray (1, 5), float
        The (openCV formatted) distortion coefficients for the camera
    obj_pts : np.ndarray, shape (n, 3), float
        The 3D position of the points on the object (relative to the object frame) to
        be projected
    ret_jac : bool, optional, default=False
        If True, returns the Jacobian. If False only the projection is returned
    ret_full_jac : bool, optional, default=False
        If True, returns full 15 term Jacobian (delta rvec, delta tvec, delta focal
        length, delta principal point, delta distortion). If False, returns the 6 term
        Jacobian (delta rvec, delta tvec)

    Returns
    ----------
    projs, np.ndarray, shape (n, 2), float
        Projected pixel positions
    jacs : np.ndarray, shape (n, 2, 6) or (n, 2, 15), float
        Jacobian of the projected pixel location. Only returned if `ret_jac` is True.
        ``jacs[i]`` is associated with ``projs[i]``. ``jacs[:, 0, :]`` is the x axis,
        ``jacs[:, 1, :]`` is the y axis.  Axis 2 of `jacs` is the partial derivative
        related to the inputs (delta rvec, delta tvec, etc).
    """
    if not len(obj_pts):
        return np.array([])

    # Convert the rmat into an rvec
    rvec, _ = cv2.Rodrigues(rmat)

    # Project the points onto the image
    obj_pts = np.array(obj_pts).astype(np.float64)
    projs, jacs = cv2.projectPoints(obj_pts,
                                    rvec=rvec, tvec=tvec,
                                    cameraMatrix=cameraMatrix, distCoeffs=distCoeffs)

    # Squeeze the extra dimension out
    projs = projs.squeeze(axis=1)

    # Add the jacobian if desired
    if ret_jac:
        if not ret_full_jac:
            # Crop just the delta-rotation-vector and delta-translation-vector terms
            jacs = jacs[:, :6]

        # Reshape jac so it has a dimension for pixel (u, v)
        jacs = jacs.reshape(jacs.shape[0]//2, 2, jacs.shape[1])

        return projs, jacs

    # If the jacobian is not required, return just he projected pixel locations
    else:
        return projs


#---------------------------------------------------------------------------------------
# Target Functions

def transform_targets(rmat, tvec, tgts):
    """Transform the targets by the given transformation values.

    `rmat` and `tvec` are the transformation from the camera to the object's frame.
    ``tgts[i]['tvec']`` is the position of target ``i`` relative to the object frame.
    ``tgts[i]['norm']`` is the normal of target ``i`` relative to the object frame. The
    function returns the targets such that 'tvec' and 'norm' are relative to the camera.

    Parameters
    ----------
    rmat : np.ndarray, shape (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray, shape (3, 1), float
        Translation vector from camera to object
    tgts : list of dict
        Each target is a dict with (at a minimum) a 'tvec', and 'norm' attribute. 'tvec'
        is the position of the target relative to the object frame. 'norm' is he normal
        of the target relative to the object frame. All other attributes will be copied
        to the transformed targets.

    Returns
    ----------
    tgts_tf : list of dict
        List of targets. Each target has the attributes of the input targets, except
        the transformed targets are transformed to be relative to the camera
    """

    tgts_tf = []
    for tgt in tgts:
        # Transform the target tvec and norm to be relative to the camera
        tgt_tf_tvec = transform_3d_point(rmat, tvec, tgt['tvec'].T).T
        tgt_tf_norm = np.matmul(rmat, tgt['norm'])

        tgt_copy = copy.deepcopy(tgt)
        tgt_copy['tvec'] = tgt_tf_tvec
        tgt_copy['norm'] = tgt_tf_norm

        tgts_tf.append(tgt_copy)

    return tgts_tf


def project_targets(rmat, tvec, cameraMatrix, distCoeffs, tgts, dims=None):
    """Projects targets into an image.

    Parameters
    ----------
    rmat : np.ndarray, shape (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray (3, 1), float
        Translation vector from camera to object
    cameraMatrix : np.ndarray, shape (3, 3), float
        The (openCV formatted) camera matrix for the camera
    distCoeffs : np.ndarray, shape (5, 1) or (5,), float
        The (openCV formatted) distortion coefficients for the camera
    tgts : list of dict
        Each target is a dict with (at a minimum) 'tvec' and 'target_type' attributes.
        The 'tvec' attribute gives the target's location and 'target_type' is a string
        denoting the type of target (most commonly 'dot' or 'kulite')
    dims : array_like, length 2, optional
        Dimensions of image in (width, height). If not None, any targets that get
        projected outside the image (x < 0, x > dims[1], y < 0, y > dims[0]) will be
        None rather than having a value

    Returns
    ----------
    list of dict
        List of target projections. Each target projections is a dict with the
        attributes 'target_type' and 'proj'. If `dims` is not None, some target
        projections may be None instead
    """
    if (len(tgts) == 0):
        return []

    # Package the 3D point translation vectors
    obj_pts = np.array([tgt['tvec'] for tgt in tgts])

    # Project the points onto the image
    projs = project_3d_point(rmat, tvec, cameraMatrix, distCoeffs, obj_pts)

    # Repackage the projected points with their target type
    tgt_projs = []
    for proj, tgt, in zip(projs, tgts):
        tgt_projs.append({'target_type': tgt['target_type'], 'proj': proj})

    # If dims is given, filter projections outside the image
    if dims is not None:
        tgt_projs_temp = []
        for tgt_proj in tgt_projs:
            pt = tgt_proj['proj']
            # If the point is within the image bounds given, add it to the temp list
            if (0.0 <= pt[0] <= dims[1] - 1) and (0.0 <= pt[1] <= dims[0] - 1):
                tgt_projs_temp.append(tgt_proj)
            else:
                tgt_projs_temp.append(None)
        tgt_projs = tgt_projs_temp

    return tgt_projs


def get_occlusions_targets(rmat, tvec, tgts, vis_checker):
    """Wrapper around :mod:`~upsp.cam_cal_utils.visibility` methods

    Parameters
    ----------
    rmat : np.ndarray, shape (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray (3, 1), float
        Translation vector from camera to object
    tgts : list of dict
        Each target is a dict with (at a minimum) a 'tvec', and 'norm' attribute. The
        'tvec' attribute gives the target's location and the 'norm' attribute gives the
        target's normal vector
    vis_checker : ~upsp.cam_cal_utils.visibility.VisibilityChecker
        Visibility checker object with the relevant BVH and oblique viewing angle

    Returns
    ----------
    occlusions : list of tuple
        The first element of each tuple is a boolean. This boolean denotes if there was
        an occlusion or not. The second element of each tuple is the point that the
        occlusion happens. If there was no occlusion (the boolean from the first element
        would be False), the value [0, 0, 0] is returned.
    """
    # Get the position and orientation of the camera in the tgts frame
    rmat_model2camera, tvec_model2camera = invTransform(rmat, tvec)

    # Package the tvecs and normals of the targets
    tvecs = []
    norms = []
    for tgt in tgts:
        tvecs.append(tgt['tvec'])
        norms.append(tgt['norm'])

    occlusions = []
    for i, node_data in enumerate(zip(tvecs, norms)):
        # Unpackage the data
        node, normal = node_data

        # Check for occlusion

        # Get the direction of the ray
        direction = tvec_model2camera - node
        direction /= np.linalg.norm(direction)

        # Get the origin at a point differentiably close the the vertex
        origin = node + vis_checker.epsilon * normal

        # If it is occluded, mark it occluded and move on to the next node
        occlusions.append(vis_checker.does_intersect(origin, direction, return_pos=True))

    return occlusions


def get_visible_targets(rmat, tvec, tgts, vis_checker):
    """Wrapper around :meth:`~upsp.cam_cal_utils.visibility.VisibilityChecker.is_visible`

    Parameters
    ----------
    rmat : np.ndarray, shape (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray (3, 1), float
        Translation vector from camera to object
    tgts : list of dict
        Each target is a dict with (at a minimum) a 'tvec', and 'norm' attribute.  The
        'tvec' attribute gives the target's location and the 'norm' attribute gives the
        target's normal vector. Both are np.ndarrays (n, 3) of floats. Returned targets
        will have the same additionalk attributes as the input targets
    vis_checker : ~upsp.cam_cal_utils.visibility.VisibilityChecker
        Visibility checker object with the relevant BVH and oblique viewing angle

    Returns
    ----------
    list of dict
        List of visible targets. Returned targets are references to input targets.
        Returned list is the subset of the input targets that are visible to the camera
    """

    # Get the position and orientation of the camera in the tgts frame
    rmat_model2camera, tvec_model2camera = invTransform(rmat, tvec)

    # Package the tvecs and normals of the targets
    tvecs = []
    norms = []
    for tgt in tgts:
        tvecs.append(np.array(tgt['tvec']))
        norms.append(np.array(tgt['norm']))
    tvecs = np.squeeze(np.array(tvecs), 2)
    norms = np.squeeze(np.array(norms), 2)

    # Get the visible targets and return
    visible_indices = vis_checker.is_visible(tvec_model2camera, tvecs, norms)
    tgts_visibles = [tgts[x] for x in visible_indices]
    return tgts_visibles


def reprojection_error(rmat, tvec, cameraMatrix, distCoeffs, tgts, img_targets):
    """Calculates RMS reprojection error

    Parameters
    ----------
    rmat : np.ndarray, shape (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray, shape (3, 1), float
        Translation vector from camera to object
    cameraMatrix : np.ndarray, shape (3, 3), float
        The (openCV formatted) camera matrix for the camera
    distCoeffs : np.ndarray, shape (1, 5), float
        The (openCV formatted) distortion coefficients for the camera
    tgts : list of dict
        Each target is a dict with (at a minimum) 'tvec', 'norm', and 'target_type'
        attributes. The 'tvec' attribute gives the target's location and the 'norm'
        attribute gives the target's normal vector. 'target_type' is a string denoting
        the type of target (most commonly 'dot' or 'kulite')
    img_targets : list of dict
        Each ``img_target`` is a dict with (at a minimum) a 'center' attribute. The
        'center' attribute gives the pixel position of the target's center in the image.
        ``img_targets[i]`` is associated with ``tgts[i]``

    Returns
    ----------
    rms, max_dist : float
        RMS distance and maximum distance
    """

    if len(tgts) == 0:
        return np.inf, np.inf

    # Project inlier targets into image
    tgt_projs = project_targets(
        rmat, tvec, cameraMatrix, distCoeffs, tgts)

    # Calculate the reprojection error
    rms = 0
    max_dist = -np.inf
    for proj, img_pt in zip(tgt_projs, img_targets):
        # Get the Euclidean distance between the projection and detection img target
        dist = np.linalg.norm([proj['proj'][0] - img_pt['center'][0],
                               proj['proj'][1] - img_pt['center'][1]])

        # Check if this is the largest distance thus far
        max_dist = max(dist, max_dist)

        # Add the square of the distance to the sum
        rms += dist**2

    # Divide by the number of targets to get the mean of the squares
    rms /= len(tgts)

    # Take the square root to get the room mean square error
    rms = np.sqrt(rms)

    return rms, max_dist
