import numpy as np
import cv2

from upsp.cam_cal_utils import (
    photogrammetry,
    internal_calibration,
)

np.set_printoptions(linewidth=np.inf, precision=4, threshold=np.inf)


def node_to_pixel_mapping_keyframe(
    rmat,
    tvec,
    cameraMatrix,
    distCoeffs,
    vis_checker,
    nodes,
    normals,
    get_pts_inside_incal_inputs=None
):
    """Returns data on the projected visible nodes for a keyframe

    Returns a numpy array of the pixel positions of the projected locations of the
    visible nodes. Additionally returns the gradient of the pixel position with respect
    to the rotation vector and translation vector. Non-visible nodes are given a pixel
    position and gradient value of NAN. Additionally returns a sorted numpy array of the
    indices of the nodes that are visible.

    Nodes are also checked for 'safety' with regards to the internal calibration. Points
    outside the 'safe' region of the internal calibration are rejected. This is most
    often the case for poorly-defined internal calibrations. But can happen to high
    quality internal calibrations if the nodes given are very far outside the frame of
    the image. See internal_calibration.get_pts_inside_incal for more information.

    Parameters
    ----------
    rmat : np.ndarray, shape (3, 3), float
        The rotation matrix from the camera to the model
    tvec : np.ndarray, shape (3, 1), float
        The translation vector from the camera to the model
    cameraMatrix : np.ndarray, shape (3, 3), float
        The (openCV formatted) camera matrix for the camera
    distCoeffs : np.ndarray, shape (5, 1) or (5,), float
        The (openCV formatted) distortion coefficients for the camera
    vis_checker : ~upsp.cam_cal_utils.visibility.VisibilityChecker
        ``VisibilityChecker`` object with the relevant BVH and oblique viewing angle
    nodes : np.ndarray, shape (N, 3), float
        A numpy array of the X, Y, and Z values of the nodes. ``nodes[n]`` is associated
        with ``normals[n]``
    normals : np.ndarray, shape (N, 3), float
        A numpy array of the i, j, and k values of the node normals. ``normals[n]`` is
        associated with ``nodes[n]``
    get_safe_pts_inputs : dict. Optional, default={}
        *get_safe_pts_inputs is given to internal_calibration.get_safe_pts when finding
        the nodes inside the safe region of the internal calibration. 
    
    Returns
    -------
    projections : np.ndarray, shape (N, 2), float
        Pixel locations of the projected nodes. Non-visibles nodes will be NAN.
        ``projections[i]`` is associated with ``nodes[i]`` and ``jacobian[i]``
    jacobians : np.ndarray, shape (N, 2, 6), float
        Jacobian of pixel locations of the projected nodes. Non-visibles nodes will
        be NAN. Jacobian axis 2 refers to (`rvec` | `tvec`). ``jacobian[i]`` is
        associated with ``nodes[i]`` and ``projections[i]``
    visible_indices : np.ndarray, shape (V,), int
        Sorted numpy array of the indices of the visibles nodes

    See Also
    --------
    node_to_pixel_mapping_non_keyframe : use outputs to quickly map non-keyframes
    """
    if get_pts_inside_incal_inputs is None:
        get_pts_inside_incal_inputs = {'critical_pt':'first'}    
    
    # Get the visible nodes
    rmat_model2camera, tvec_model2camera = photogrammetry.invTransform(rmat, tvec)

    in_incal_and_vis_idxs = vis_checker.is_visible_and_inside_incal(
        rmat,
        tvec,
        cameraMatrix,
        distCoeffs,
        nodes,
        normals,
        get_pts_inside_incal_inputs
    )
    in_incal_and_vis_idxs_nodes = nodes[in_incal_and_vis_idxs]

    # Project all visible nodes
    projs, jacs = photogrammetry.project_3d_point(
        rmat, tvec, cameraMatrix, distCoeffs, in_incal_and_vis_idxs_nodes, ret_jac=True
    )

    # Initialize an output of all NANs
    projs_fin = np.full((len(nodes), 2), np.NAN)
    jacs_fin  = np.full((len(nodes), 2, 6), np.NAN)

    # For visible nodes, replace the NANs with data
    projs_fin[in_incal_and_vis_idxs] = projs
    jacs_fin[in_incal_and_vis_idxs] = jacs

    return projs_fin, jacs_fin, in_incal_and_vis_idxs


def node_to_pixel_mapping_non_keyframe(
    rmat_key,
    tvec_key,
    rmat_curr,
    tvec_curr,
    projs,
    jacs,
    vis_idxs
):
    """Returns data on the projected visible nodes for a non-keyframe

    Returns a numpy array of the pixel positions of the projected locations of the
    visible nodes. Non-visible nodes are given a pixel position of NAN. Assumes set of
    nodes visible in keyframe is same for nearby non-keyframes. Uses the Jacobian to
    quickly approximate the projected location

    Parameters
    ----------
    rmat_key : np.ndarray, shape (3, 3), float
        The rotation matrix from the camera to the model of the keyframe
    tvec_key : np.ndarray, shape (3, 1), float
        The translation vector from the camera to the model of the keyframe
    rmat_curr : np.ndarray, shape (3, 3), float
        The rotation matrix from the camera to the model of the current non-keyframe
    tvec_curr : np.ndarray, shape (3, 1) or (3,), float
        The translation vector from the camera to the model of the current non-keyframe
    projs : np.ndarray, shape (N, 2), float
        Projected locations of the nodes in the keyframe
    jacs : np.ndarray, shape (N, 2, 6),  float
        Jacobians of the projected locations
    vis_idxs : np.ndarray, shape (V,), float
        Indices of the visible nodes.

    Returns
    -------
    updated_projections : np.ndarray, shape (N, 2), float
        Approximate pixel locations of the projected nodes. Non-visibles nodes will be
        NAN. ``updated_projections[i]`` is associated with ``projs[i]``

    See Also
    --------
    node_to_pixel_mapping_keyframe : create keyframe inputs
    """

    # Convert the rotation matrix to a rotation vector
    rvec_key  = np.array(cv2.Rodrigues(rmat_key)[0])
    rvec_curr = np.array(cv2.Rodrigues(rmat_curr)[0])

    # Get the delta Transformation
    dr = rvec_curr - rvec_key
    dt = tvec_curr - tvec_key
    dT = np.concatenate((dr, dt))

    projs_partial = projs[vis_idxs]
    jacs_partial  = jacs[vis_idxs]

    # Get the pixel updates
    projs_updated = projs_partial + np.sum(jacs_partial * dT.T, axis=2)

    projs_fin = np.full((len(projs), 2), np.NAN)
    projs_fin[vis_idxs] = projs_updated

    return projs_fin


def node_to_pixel_mapping_non_keyframe_full(
    rmat,
    tvec,
    cameraMatrix,
    distCoeffs,
    nodes,
    vis_idxs
):
    """Returns data on the projected visible nodes for a non-keyframe

    Returns a numpy array of the pixel positions of the projected locations of the
    visible nodes. Non-visible nodes are given a pixel position of NAN. Assumes set of
    nodes visible in keyframe is same for nearby non-keyframes. Fully computes the
    projection

    Parameters
    ----------
    rmat : np.ndarray, shape (3, 3), float
        The rotation matrix from the camera to the model of the current non-keyframe
    tvec : np.ndarray, shape (3, 1), float
        The translation vector from the camera to the model of the current non-keyframe
    cameraMatrix : np.ndarray, shape (3, 3), float
        The (openCV formatted) camera matrix for the camera
    distCoeffs : np.ndarray, shape (5, 1) or (5,), float
        The (openCV formatted) distortion coefficients for the camera
    nodes : np.ndarray, shape (N, 3), float
        A numpy array of the X, Y, and Z values of the nodes. ``nodes[n]`` is associated
        with ``normals[n]``
    vis_idxs : np.ndarray, shape (V,), float
        Indices of the visible nodes

    Returns
    -------
    updated_projections : np.ndarray, shape (N, 2), float
        Pixel locations of the projected nodes. Non-visibles nodes will be NAN.
        ``updated_projections[i]`` is associated with ``nodes[i]``

    See Also
    --------
    node_to_pixel_mapping_keyframe : create `vis_idxs` based on a keyframe
    """
    # Get just the visible nodes
    vis_nodes = nodes[vis_idxs]

    # Project all visible nodes
    projs = photogrammetry.project_3d_point(
        rmat, tvec, cameraMatrix, distCoeffs, vis_nodes, ret_jac=False
    )

    # Initialize an output of all NANs
    projs_fin = np.full((len(nodes), 2), np.NAN)
    projs_fin[vis_idxs] = projs

    return projs_fin
