import cv2
import numpy as np
import copy
from scipy.spatial.transform import Rotation as scipy_Rotation

import visualization

np.set_printoptions(suppress=True)

#---------------------------------------------------------------------------------------
# General Photogrammetry Utility Functions


def rot(angle, axis):
    """Returns rotation matrix when rotating about the given axis by angle degrees
    
    Parameters
    ----------
    angle : float
        angle of rotation (in degrees) about the given axis
    axis : 
        axis to rotate about

    Returns
    ----------
    np.ndarray (3, 3), float
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
    """Returns the inverse transformation of the given R and t
        
    Parameters
    ----------
    R : np.ndarray (3, 3), float
        Rotation Matrix
    t : np.ndarray (3, 1) or (3,), float
        Translation Vector

    Returns
    ----------
    tuple
        (rmat_i, tvec_i) where rmat_i is the inverse rotation matrix (np.ndarray (3,3),
        float) and tvec_i is the inverse translation vector (np.ndarray (3,1), float)
    """

    R_transpose = R.transpose()
    return (R_transpose, -np.matmul(R_transpose, t))


def isRotationMatrix(R):
    """Checks if a matrix is a valid rotation matrix
        
    Parameters
    ----------
    R : np.ndarray (3, 3), float
        Rotation Matrix

    Returns
    ----------
    Boolean
        True if R is a valid rotation matrix, and False if it is not
    """

    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    n = np.linalg.norm(np.identity(3, dtype=R.dtype) - shouldBeIdentity)
    return n < 1e-6


def isRotationMatrixRightHanded(R):
    """
    Returns True if matrix is right handed, and False is left handed
    """

    return np.dot(np.cross(R[:,0], R[:,1]), R[:,2]) > 0.


def rotationMatrixToTunnelAngles(R) :
    """Converts rotation matrix to tunnel angles
        
    Parameters
    ----------
    R : np.ndarray (3, 3), float
        Rotation Matrix
    
    Returns
    ----------
    np.ndarray (3,), float
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
    return np.array([alpha, beta, -neg_phi])


def project_3d_point(rmat, tvec, cameraMatrix, distCoeffs, obj_pts, ret_jac=False, ret_full_jac=False):
    """Projects targets into an image.
    
    Parameters
    ----------
    rmat : np.ndarray (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray (3, 1) or (3,), float
        Translation vector from camera to object
    cameraMatrix : np.ndarray (3x3), float
        The (openCV formatted) camera matrix for the camera
    distCoeffs : np.ndarray (5x1) or (5,), float
        The (openCV formatted) distortion coefficients for the camera
    ret_jac : Boolean, optional default=False
        If True, returns the jacobian. If False only the projection is returned
    ret_full_jac : Boolean, optional default=False
        If True, returns full 15 term jacobian (delta rvec, delta tvec, delta focal
        length, delta principal point, delta distortion). If False, returns the 6 term
        jacobian (delta rvec, delta tvec)

    Returns
    ----------
    projs or (projs, jacs)
        If ret_jac is False, returns just projs. If ret_jac is True, returns projs and
        jacs as a tuple.
        projs is np.ndarray (N,2), float and the projected pixel positions
        jacs is np.ndarray (N, 2, 6) or (N, 2, 15), float and is the jacobian of the
        projected pixel location. jacs[i] is associated with projs[i]. jacs[:, 0, :] is
        the x axis, jacs[:, 1, :] is the y axis. Axis 2 of jacs is the partial
        derivative related to the inputs (delta rvec, delta tvec, etc)
    """
    # Convert the rmat into an rvec
    rvec, _ = cv2.Rodrigues(rmat)

    # Project the points onto the image
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
    """Transform the targets by the given transformation values

    Parameters
    ----------
    rmat : np.ndarray (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray (3, 1) or (3,), float
        Translation vector from camera to object
    tgts : list of targets
        Each target is a dict with (at a minimum) a 'tvec', and 'norm' attribute.  All
        other attributes will be copied to the transformed targets. The 'tvec' attribute
        gives the target's location and the 'norm' attribute gives the target's normal
        vector
        
    Returns
    ----------
    transformed targets
        list of targets. Each target has the attributes of the input targets, except
        the transformed targets are transformed by the given rmat and tvec transformation
    """

    tgts_tf = []
    for tgt in tgts:
        # Transform the target tvec and norm to be relative to the camera
        tgt_tf_tvec = np.matmul(rmat, tgt['tvec']) + tvec
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
    rmat : np.ndarray (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray (3, 1) or (3,), float
        Translation vector from camera to object
    tgts : list of targets
        Each target is a dict with (at a minimum) 'tvec' and 'target_type' attributes.
        The 'tvec' attribute gives the target's location and 'target_type' is a string
        denoting the type of target (most commonly 'dot' or 'kulite')
    cameraMatrix : np.ndarray (3x3), float
        The (openCV formatted) camera matrix for the camera
    distCoeffs : np.ndarray (5x1) or (5,), float
        The (openCV formatted) distortion coefficients for the camera
    dims : np.ndarray (2, 1) or (2,), float, or equivalent or None. Optional default=None
        Dimensions of image in (width, height). If not None, any targets that get
        projected outside the image (x < 0, x > dims[1], y < 0, y > dims[0]) will be
        None rather than having a value

    Returns
    ----------
    list
        list of target projections. Each target projections is a dict with the attributes
        'target_type' and 'proj'. If dims is not None, some target projections may be
        None instead
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
        tgt_projs.append({'target_type': tgt['target_type'], 'proj' : proj.tolist()})

    # If dims is given, filter projections outside the image
    if dims is not None:
        tgt_projs_temp = []
        for tgt_proj in tgt_projs:
            pt = tgt_proj['proj']
            # If the point is within the image bounds given, add it to the temp list
            if (0.0 <= pt[0] <= dims[1] - 1) and (0.0 <= pt[1] <= dims[0]):
                tgt_projs_temp.append(tgt_proj)
            else:
                tgt_projs_temp.append(None)
        tgt_projs = tgt_projs_temp

    return tgt_projs


def get_occlusions_targets(rmat, tvec, tgts, vis_checker):
    """Wrapper around visibility.py methods

    Parameters
    ----------
    rmat : np.ndarray (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray (3, 1) or (3,), float
        Translation vector from camera to object
    tgts : list of targets
        Each target is a dict with (at a minimum) a 'tvec', and 'norm' attribute. The
        'tvec' attribute gives the target's location and the 'norm' attribute gives the
        target's normal vector
    vis_checker : VisibilityChecker from upsp/python/upsp/cam_cal_utils/visibility.py
        VisibilityChecker object with the relevant BVH and oblique viewing angle

    Returns
    ----------
    A list of (boolean, occlusion_pt)
        The first element of each tuple is a boolean. This boolean denotes if there
            was an occlusion or not
        The second element of each tuple is the point that the occlusion happens. If
            there was no occlusion (the boolean from the first element would be
            False), the value [0, 0, 0] is returned.
    """
    # Get the position and orientation of the camera in the tgts frame
    rmat_model2camera, tvec_model2camera = invTransform(rmat, tvec)
    tvec_model2camera = tvec_model2camera.ravel()

    # Package the tvecs and normals of the targets
    tvecs = []
    norms = []
    for tgt in tgts:
        tvecs.append(np.array(tgt['tvec']))
        norms.append(np.array(tgt['norm']))
    
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
    """Wrapper around visibility.py's is_visible

    Parameters
    ----------
    rmat : np.ndarray (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray (3, 1) or (3,), float
        Translation vector from camera to object
    tgts : list of targets
        Each target is a dict with (at a minimum) a 'tvec', and 'norm' attribute.  The
        'tvec' attribute gives the target's location and the 'norm' attribute gives the
        target's normal vector. Returned targets will have the same attributes as the
        input targets
    vis_checker : VisibilityChecker from upsp/python/upsp/cam_cal_utils/visibility.py
        VisibilityChecker object with the relevant BVH and oblique viewing angle

    Returns
    ----------
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

    # Get the visible targets and return
    visible_indices = vis_checker.is_visible(tvec_model2camera, tvecs, norms)
    tgts_visibles = [tgts[x] for x in visible_indices]
    return tgts_visibles


def reprojection_error(rmat, tvec, cameraMatrix, distCoeffs, tgts, img_targets):
    """Calculates RMS reprojection error

    Parameters
    ----------
    rmat : np.ndarray (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray (3, 1) or (3,), float
        Translation vector from camera to object
    cameraMatrix : np.ndarray (3x3), float
        The (openCV formatted) camera matrix for the camera
    distCoeffs : np.ndarray (5x1) or (5,), float
        The (openCV formatted) distortion coefficients for the camera
    tgts : list of targets
        Each target is a dict with (at a minimum) 'tvec', 'norm', and 'target_type'
        attributes. The 'tvec' attribute gives the target's location and the 'norm'
        attribute gives the target's normal vector. 'target_type' is a string denoting
        the type of target (most commonly 'dot' or 'kulite')
    img_targets : list of img_targets
        Each img_target is a dict with (at a minimum) a 'center' attribute. The 'center'
        attribute gives the pixel position of the target's center in the image.
        img_targets[i] is associated with tgts[i]
    
    Returns
    ----------
    tuple (float, float)
    rms distance and the maximum distance
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


#---------------------------------------------------------------------------------------
# Wind Tunnel Specific Function

def tunnel_transform(ALPHA, BETA, PHI, STRUTZ, tvec__cor_tgts__tgts_frame):
    """Calculates the transformation from the tunnel coordinate frame to the tgts frame
    
    Note: This is not necessarily to the tunnel origin, just some fixed point in the
        tunnel (fixed within a tunnel test). If STRUTZ = STRUTZ_abs then it will be the
        tunnel origin

    Parameters
    ----------
    alpha : float
        Tunnel alpha in degrees
    beta : float
        Tunnel beta in degrees
    phi : float
        Tunnel phi in degrees
    tvec__tgts_trot__tgts_frame : np.ndarray (3, 1) or (3,), float
        Translation vector from the tunnel center of rotation to the tgts frame
        in the tgts frame. The tgts frame is a fixed distance from the tunnel point of
        rotation from the tgts frame's point of view, that translation vector is always
        along the x axis

    Returns
    ----------
        tuple of (np.ndarray (3, 3) float, np.ndarray (3, 1) float)
        Rotation matrix and translation vector from tgts frame to tunnel frame
    """

    # Get the component rotation matrices

    # UPWT Tunnel Coordinates are RHS Aircraft Coordinates Pitched 180 degrees
    # See UPWT AIAA Coordinate Systems Training Manual For Details
    # (uPSP Teams > General > 4.2 Calibration >
    #       Stereo Calibration > AIAA Coordinate Systems Training.pdf)
    
    # Positive Alpha is Positive Pitch
    pitch = rot(-ALPHA, 'y')

    # Positive Beta is Negative Yaw
    yaw = rot(-BETA, 'z')

    # Positive Phi is Positive Roll
    roll = rot(PHI, 'x')

    # Combine into one rotation matrix
    #   Matrix Multiplication Order is [P][Y][R]
    rotation_matrix = np.matmul(pitch, np.matmul(yaw, roll))

    # We want the transformation from tunnel to tgts, so get the inverse
    rotation_matrix = np.linalg.inv(rotation_matrix)

    # tvec__tgts_tunnel__tunnel_frame is the translation vector from the tunnel
    #   frame to the tgts frame, in the tunnel frame
    # I.e. Since the knuckle sleeve rotates relative to the tunnel, the model
    #   translation vector somewhere in the cone of allowable rotation. This
    #   calculates that translation. Also, the strutz can move up and down in
    #   the z axis, so that needs to be taken into account as well
    tvec__knuckle_tgts = np.matmul(rotation_matrix, tvec__cor_tgts__tgts_frame)
    tvec__tunnel_tgts__tunnel_frame = tvec__knuckle_tgts + np.array([0, 0, STRUTZ])

    return rotation_matrix, tvec__tunnel_tgts__tunnel_frame


# TODO: This refers specifically to a model on the sting. We will need a function for
#   a floor mounted model, and ideally something in the test_config file to specify
def tf_camera_tgts_thru_tunnel(camera_cal, wtd, test_config):
    """Returns the transformation from the camera to the model (tgts frame)

    Parameters
    ----------
    camera_cal : list
        camera calibration in the form:
        [cameraMatrix, distCoeffs, rmat__camera_tunnel, tvec__camera_tunnel]
    wtd : dict
        wind tunnel data as a dict with (at a minimum) the keys 'ALPHA', 'BETA', 'PHI',
        and 'STRUTZ'. ALPHA, BETA, and PHI are tunnel angles in degrees. STRUTZ is the
        offset of the tunnel center of rotation for the z axis in inches
    test_config : dict
        test configuration data as a dict with (at a minimum) the key 
        'tunnel-cor_to_tgts_tvec' representing the translation vector from the tunnel
        center of rotation to the model frame
    
    Returns
    ----------
    tuple (np.ndarray (3, 3) float, np.ndarray (3, 1) float)
        First element is the rotation matrix, and second element is the translation
        vector.
    """

    # Turn the wind tunnel data into the transformation from tunnel to targets
    wtd_transform = tunnel_transform(wtd['ALPHA'], wtd['BETA'], wtd['PHI'], 
                                     wtd['STRUTZ'], test_config['tunnel-cor_to_tgts_tvec'])
    rmat_tunnel_tgts, tvec_tunnel_tgts = wtd_transform

    # Transformation from tgts frame to tunnel frame
    rmat_tgts_tunnel = np.linalg.inv(rmat_tunnel_tgts)
    tvec_tgts_tunnel = -np.matmul(rmat_tgts_tunnel, tvec_tunnel_tgts)

    # Decompose the camera calibration into its parts
    cameraMatrix, distCoeffs, rmat__camera_tunnel, tvec__camera_tunnel = camera_cal

    # Combine the transformations to get the transformation from `camera to tgts frame
    rmat__camera_tgts = np.matmul(rmat__camera_tunnel, np.linalg.inv(rmat_tgts_tunnel))
    tvec__camera_tgts = tvec__camera_tunnel + np.matmul(rmat__camera_tunnel,
                                                        tvec_tunnel_tgts)

    return rmat__camera_tgts, tvec__camera_tgts
