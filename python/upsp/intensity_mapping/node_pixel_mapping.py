import numpy as np
import os
import sys
import cv2

np.set_printoptions(linewidth=np.inf, precision=4, threshold=np.inf)

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)

utils = os.path.join(parent_dir, 'cam_cal_utils')
sys.path.append(utils)

import visualization
import visibility
import photogrammetry
import patching


def node_to_pixel_mapping_keyframe(rmat, tvec, cameraMatrix, distCoeffs, vis_checker, nodes, normals):
    """Returns data on the projected visible nodes for a keyframe
    
    Returns a numpy array of the pixel positions of the projected locations of the
    visible nodes. Additionally returns the gradient of the pixel position with respect
    to the rotation vector and translation vector. Non-visible nodes are given a pixel 
    position and gradient value of NAN. Additionally returns a sorted numpy array of the
    indices of the nodes that are visible.

    Parameters
    ----------
    rmat : np.ndarray (3x3), float
        The rotation matrix from the camera to the model
    tvec : np.ndarray (3x1) or (3,), float
        The translation vector from the camera to the model
    cameraMatrix : np.ndarray (3x3), float
        The (openCV formatted) camera matrix for the camera
    distCoeffs : np.ndarray (5x1) or (5,), float
        The (openCV formatted) distortion coefficients for the camera
    vis_checker : VisibilityChecker from upsp/python/upsp/cam_cal_utils/visibility.py
        VisibilityChecker object with the relevant BVH and oblique viewing angle
    nodes : np.ndarray (N, 3), float
        A numpy array of the X, Y, and Z values of the nodes. nodes[n] is associated
        with normals[n]
    normals : np.ndarray (N, 3), float
        A numpy array of the i, j, and k values of the node normals. normals[n] is
        associated with nodes[n]
    
    Returns
    ----------
    tuple of length 3 : (projections, jacobians, visible_indices)
        projections : np.ndarray (N, 2), float
            pixel locations of the projected nodes. Non-visibles nodes will be NAN. 
            projections[i] is associated with nodes[i] and jacobian[i]
        jacobians : np.ndarray (N, 2, 6), float
            Jacobian of pixel locations of the projected nodes. Non-visibles nodes will
            be NAN. Jacobian axis 2 refers to (rvec | tvec). jacobian[i] is associated
            with nodes[i] and projections[i]
        visible_indices : np.ndarray (V,), int
            sorted numpy array of the indices of the visibles nodes
    """

    # Get the visible nodes
    rmat_model2camera, tvec_model2camera = photogrammetry.invTransform(rmat, tvec)
    vis_idxs = vis_checker.is_visible(tvec_model2camera, nodes, normals)
    vis_nodes = nodes[vis_idxs]
    
    # Project all visible nodes
    projs, jacs = photogrammetry.project_3d_point(rmat, tvec, cameraMatrix, distCoeffs, vis_nodes, ret_jac=True)

    jacs_swapped = np.swapaxes(jacs, 0, 1)

    # Initialize an output of all NANs
    projs_fin = np.full((len(nodes), 2), np.NAN)
    jacs_fin  = np.full((len(nodes), 2, 6), np.NAN)
    
    # For visible nodes, replace the NANs with data
    projs_fin[vis_idxs] = projs
    jacs_fin[vis_idxs] = jacs

    return projs_fin, jacs_fin, np.sort(vis_idxs)


def node_to_pixel_mapping_non_keyframe(rmat_key, tvec_key, rmat_curr, tvec_curr, projs, jacs, vis_idxs):
    """Returns data on the projected visible nodes for a non-keyframe
    
    Returns a numpy array of the pixel positions of the projected locations of the
    visible nodes. Non-visible nodes are given a pixel position of NAN. Assumes set of
    nodes visible in keyframe is same for nearby non-keyframes. Uses the jacobian to
    quickly approximate the projected location

    Parameters
    ----------
    rmat_key : np.ndarray (3x3), float
        The rotation matrix from the camera to the model of the keyframe
    tvec_key : np.ndarray (3x1) or (3,), float
        The translation vector from the camera to the model of the keyframe
    rmat_curr : np.ndarray (3x3), float
        The rotation matrix from the camera to the model of the current non-keyframe
    tvec_curr : np.ndarray (3x1) or (3,), float
        The translation vector from the camera to the model of the current non-keyframe
    projs : np.ndarray (N, 2), float
        Projected locations of the nodes in the keyframe
    jacs : np.ndarray (N, 2, 6),  float
        Jacobians of the projected locations
    vis_idxs : np.ndarray (V,), float
        Indices of the visible nodes. 

    Returns
    ----------
    updated_projections : np.ndarray (N, 2), float
        Approximate pixel locations of the projected nodes. Non-visibles nodes will be
            NAN. updated_projections[i] is associated with projs[i]
    """

    # Convert the rotation matrix to a rotation vector
    rvec_key  = np.array(cv2.Rodrigues(rmat_key)[0])
    rvec_curr = np.array(cv2.Rodrigues(rmat_curr)[0])

    # Get the delta Transformation
    dr = np.squeeze(rvec_curr - rvec_key, axis=1)
    dt = tvec_curr - tvec_key
    dT = np.concatenate((dr, dt))
    
    projs_partial = projs[vis_idxs]
    jacs_partial  = jacs[vis_idxs]

    # Get the pixel updates
    projs_updated = projs_partial + np.sum(jacs_partial * dT, axis=2)

    projs_fin = np.full((len(projs), 2), np.NAN)
    projs_fin[vis_idxs] = projs_updated

    return projs_fin


def node_to_pixel_mapping_non_keyframe_full(rmat, tvec, cameraMatrix, distCoeffs, nodes, vis_idxs):
    """Returns data on the projected visible nodes for a non-keyframe
    
    Returns a numpy array of the pixel positions of the projected locations of the
    visible nodes. Non-visible nodes are given a pixel position of NAN. Assumes set of
    nodes visible in keyframe is same for nearby non-keyframes. Fully computes the
    projection

    Parameters
    ----------
    rmat : np.ndarray (3x3), float
        The rotation matrix from the camera to the model of the current non-keyframe
    tvec : np.ndarray (3x1) or (3,), float
        The translation vector from the camera to the model of the current non-keyframe
    cameraMatrix : np.ndarray (3x3), float
        The (openCV formatted) camera matrix for the camera
    distCoeffs : np.ndarray (5x1) or (5,), float
        The (openCV formatted) distortion coefficients for the camera
    nodes : np.ndarray (N, 3), float
        A numpy array of the X, Y, and Z values of the nodes. nodes[n] is associated
        with normals[n]
    vis_idxs : np.ndarray (V,), float
        Indices of the visible nodes

    Returns
    ----------
    updated_projections : np.ndarray (N, 2), float
        Pixel locations of the projected nodes. Non-visibles nodes will be NAN.
            updated_projections[i] is associated with nodes[i]
    """
    # Get just the visible nodes
    vis_nodes = nodes[vis_idxs]

    # Project all visible nodes
    projs = photogrammetry.project_3d_point(rmat, tvec, cameraMatrix, distCoeffs, vis_nodes, ret_jac=False)

    # Initialize an output of all NANs
    projs_fin = np.full((len(nodes), 2), np.NAN)
    projs_fin[vis_idxs] = projs

    return projs_fin

