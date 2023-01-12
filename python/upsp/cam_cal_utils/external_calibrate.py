import cv2
import logging
import matplotlib.pyplot as plt
import numpy as np
import copy
import warnings

from upsp.target_localization import gaussian_fitting_methods
from upsp.cam_cal_utils import (
    visualization,
    visibility,
    camera_tunnel_calibrate,
    photogrammetry,
    img_utils,
)

gauss_fit = gaussian_fitting_methods.gauss_fitter_func("super")

log = logging.getLogger(__name__)

# Debugs
debug_raw_matches = True
debug_coarse_optimization = True
debug_refined_matches = True
debug_visible_projections = True
debug_refined_optimization = True
debug_show_localizations = False

# ---------------------------------------------------------------------------------------
# Other Functions


def compare_poses(pose0, pose1, more_info=False):
    """Returns the angle and distance in which the two poses differ

    Any two rotation matrices are related by a single rotation of theta about a given
    axis. Any two translation vectors are related by an [X, Y, Z] translation vector.
    This function returns the angle between the two poses, as well as the distance
    formatted as [theta, dist] where theta is in degrees and dist is in the units of
    tvec. If `more_info` is True, [theta, axis, tvec_rel] is returned where axis is the
    axis of rotation.

    Parameters
    ----------
    pose0, pose1 : tuple
        Pose to compare: ``(rmat, tvec)``, where ``rmat`` is the rotation matrix from
        camera to object (:class:`numpy.ndarray` with shape (3, 3)) and ``tvec`` is the
        translation vector from camera to object (:class:`numpy.ndarray` with shape (3,
        1)).
    more_info : bool, optional
        Changes the return value. Function returns (theta, distance) if `more_info` is
        False, function returns (rvec, tvec) if `more_info` is True

    Returns
    -------
    tuple
        If `more_info` is False, returns ``(theta, distance)``. If `more_info` is True,
        return is ``(rvec, tvec)``
    """
    # Unpack the rmats and tvecs
    rmat0, tvec0 = pose0
    rmat1, tvec1 = pose1

    # Get relative transformations
    rmat_rel = np.matmul(rmat1, rmat0.T)
    tvec_rel = tvec1 - np.matmul(rmat_rel, tvec0)

    # Get the axis and theta for rmat_rel
    rvec_rel, _ = cv2.Rodrigues(rmat_rel)

    # OpenCV's Rodrigues vector is a compact representation where theta is the magnitude
    #   of the vector (convert from radians to degrees)
    theta = np.linalg.norm(rvec_rel)
    theta = np.rad2deg(theta)

    # If more_info was not requested, return only the angle and distance
    if not more_info:
        return (theta, np.linalg.norm(tvec_rel))

    # If more_info was requested, return the Rodrigues vector and full tvec
    else:
        return (rvec_rel, tvec_rel)


# ---------------------------------------------------------------------------------------
# Calibration Functions


# TODO: make this return a num_matches, all targets, and all unique img_targets just
#   like to other targets functions
def subpixel_localize(img, tgts, img_targets, test_config, max_localize_delta=None):
    """Find the sub-pixel localized position of the image targets

    Find the location of the image target centers with sub-pixel accuracy. This method
    filters common bad localization solutions. I.e The localized position is too far
    initial guess to make sense, invalid optimizer solution (None flag), outside the
    cropped region or outside the image

    Parameters
    ----------
    img : np.ndarray, shape (h, w)
        Numpy 2D array of the image
    tgts : list
        Matched 3D targets. Each target should be a dict. The only strict requirement
        set by this function is ``tgts[i]`` is associated with ``img_targets[i]``
    img_targets : list
        Matched image targets. Each dict has, at a minimum, keys 'center', and
        'target_type'. 'center' has a value of the image target center location
        (tuple/np.ndarray of length 2 of floats) and 'target_type' has the key of the
        type of target (string). ``img_targets[i]`` is associated with ``tgts[i]``
    test_config : dict
        Processing parameters with, at a minimum, a key for each target type in
        `targets` and `img_targets`.  The key is `target_type` + '_pad'. This is the
        padding around the img target center location to use to sub-pixel localize
    max_localize_delta : float, optional
        The maximum allowable distance that subpixel_localize can change the
        `img_target` position. If None, the max allowable distance will be set to the
        padding distance minus 2.

    Returns
    -------
    targets : list
        Target positions (`tgts`) that have not been filtered out.
    img_targets : list
        Refined (sub-pixel localized) target positions in the image.

    Notes
    -----
    ``targets[i]`` is associated with ``img_targets[i]``. Return lists may not be the
    same length as input `tgts` and/or `img_targets` inputs (some
    `targets`/`img_targets` from the input may be rejected and thus are not included in
    the output)
    """
    if max_localize_delta is not None:
        filter_dist = max_localize_delta

    out_of_bounds = set()
    img_targets_refined = []
    for i, img_target in enumerate(img_targets):
        # Integer of center pixel
        center_pixel = np.rint(
            (img_target["center"][0], img_target["center"][1])
        ).astype(np.int32)

        target_pad = test_config[img_target["target_type"] + "_pad"]

        # cropped region around target (x1, y1), (x2, y2)
        bbox = [
            [center_pixel[0] - target_pad, center_pixel[1] - target_pad],
            [center_pixel[0] + target_pad + 1, center_pixel[1] + target_pad + 1],
        ]

        # If bbox goes out of bounds of the image, ignore it
        if (
            (bbox[0][0] < 0)
            or (bbox[0][1] < 0)
            or (bbox[1][0] >= img.shape[1])
            or (bbox[1][1] >= img.shape[0])
        ):
            out_of_bounds.add(i)
            img_targets_refined.append(
                {"target_type": img_target["target_type"], "center": (None, None)}
            )
            continue

        # Cropped image around target
        img_cropped = img[bbox[0][1] : bbox[1][1], bbox[0][0] : bbox[1][0]]

        # Perform the sub-pixel localization
        gauss_center = gauss_fit(
            img_cropped,
            target_type=img_target["target_type"],
            center=img_target["center"],
            img_offset=bbox[0],
        )[0]

        # If the optimizer failed, continue
        if gauss_center[0] is None:
            out_of_bounds.add(i)
            img_targets_refined.append(
                {
                    "target_type": img_target["target_type"],
                    "center": np.array((None, None)),
                }
            )
            continue

        # Create a new img_target with the updated center location
        img_target_refined = copy.deepcopy(img_target)
        img_target_refined["center"] = gauss_center

        dist = np.linalg.norm(
            [
                gauss_center[0] - img_target_refined["center"][0],
                gauss_center[1] - img_target_refined["center"][1],
            ]
        )

        # A distance of target_pad - 1 would imply the center is on a pixel in the edge
        #   of the crop. And since the taret is larger than 1 pixel, by definition this
        #   is a bad localization. Not even to mention it is likely bad since it is so
        #   far off from the expected position. We do target_pad - 2 to add a margin of
        #   safety
        if max_localize_delta is None:
            filter_dist = target_pad - 2

        if dist > filter_dist:
            out_of_bounds.add(i)
            img_targets_refined.append(
                {
                    "target_type": img_target["target_type"],
                    "center": np.array((None, None)),
                }
            )
            continue

        if debug_show_localizations:
            plt.imshow(img_cropped, cmap="gray")
            plt.scatter(
                [gauss_center[0] - bbox[0][0]],
                [gauss_center[1] - bbox[0][1]],
                c="g",
                s=2,
            )
            plt.scatter(
                [img_target["center"][0] - bbox[0][0]],
                [img_target["center"][1] - bbox[0][1]],
                c="r",
                s=2,
            )
            plt.savefig(
                str(i).rjust(3, "0")
                + "_"
                + str(np.round(dist, 3))
                + "_"
                + str(center_pixel[0])
                + "_"
                + str(center_pixel[1])
                + ".png"
            )
            plt.close()

        img_targets_refined.append(img_target_refined)

    # Remove the targets that had a bad localization
    tgts_loc = []
    img_targets_loc = []
    for i, (tgt, img_target) in enumerate(zip(tgts, img_targets_refined)):
        if i not in out_of_bounds:
            tgts_loc.append(tgt)
            img_targets_loc.append(img_target)

    return tgts_loc, img_targets_loc


# TODO: make this return a num_matches, all targets, and all unique img_targets just
#   like to other targets functions
def filter_partially_occluded(
    rmat, tvec, focal_length, tgts, img_targets, vis_checker, test_config
):
    """Checks corners of cropped area used for sub-pixel localization for occlusion

    If the corners of the crop used for sub-pixel localization jumps surfaces, then the
    target is likely partially occluded. This most commonly occurs when a booster
    partially occludes a target on the core of a launch vehicle

    To get the 3D positions of corners of the cropped area, start at the target tvec.
    Approximate the physical distance (in inches) of the cropped area (which is
    done in pixels) using the focal length and distance from camera to model. Take
    steps along the camera image plane to get to the approximate corner locations in 3D.

    With the corner locations, ensure that they are not inside the model (since the
    model is curved the steps along the image plane may put the corners slightly inside
    the model. Then check for occlusions. If the corner is occluded, the target is
    deemed partially occluded. If none of the corners are occluded, the target is deemed
    not occluded (but is still potentially partially occluded). Only the corners are
    checked to reduce computation

    Parameters
    ----------
    rmat : np.ndarray, shape (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray, shape (3, 1), float
        Translation vector from camera to object
    focal_length : float
        Focal length of the camera. Most easily accessible from a ``cameraMatrix[0][0]``
    tgts : list
        3D targets. Each target is a dict and has, at a minimum, the keys 'tvec',
        'target_type', 'norm'. 'tvec' has a :class:`numpy.ndarray` (3, 1) representing the
        position of the target relative to the model origin for its associated value.
        'norm' has a :class:`numpy.ndarray` (3, 1) representing the normal vector of the
        target relative to the model origin for its associated value. 'target_type' has
        a string representing the type of target (most commonly 'dot') for its
        associated value. ``tgts[i]`` is associated with ``img_targets[i]``
    img_targets : list
        Image targets. Each image target should be a dict, but this function does not
        set any strict requirements other than that ``img_targets[i]`` is associated
        with ``tgts[i]``
    vis_checker : ~upsp.cam_cal_utils.visibility.VisibilityChecker
        Visibility checker object with the relevant BVH and oblique viewing angle
    test_config : dict
        Processign parameters with, at a minimum, a key for each target type of the
        targets in `tgts` and `img_targets`. They key is `target_type` + '_pad'. This is
        the padding around the img target center location to use to sub-pixel localize

    Returns
    -------
    tgts_filtered : list
        Target positions (`tgts`) that have not been filtered out
    img_targets_filtered : list
        Target positions in the image.
    """
    # Get the direction for the u and v of pixel space
    rmat_inv, tvec_inv = photogrammetry.invTransform(rmat, tvec)

    # Check each target for partial occlusions
    tgts_filtered = []
    img_targets_filtered = []
    for tgt, img_target in zip(tgts, img_targets):
        # Get the scale factor
        obj_dist = np.linalg.norm(tvec.T - tgt["tvec"])

        # For the scale factor, we use the similar triangles of the object distance by
        #   actual distance (in) vs focal length by pixel distance (px)
        # The scale factor is the object distance divided by the focal length, times the
        #   pixel distance. We add one to the pad distance for a margin of safety
        step_sz = (
            obj_dist * (test_config[tgt["target_type"] + "_pad"] + 1) / focal_length
        )

        # Get the step vector for u and v for the corners
        # Get the u and v steps
        uv = step_sz * rmat_inv[:, 0:2]
        u = uv[:, 0]
        v = uv[:, 1]

        # Package the corners. We have 1 corner for steps in the given directions
        corners = []
        for x, y in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
            corners.append({"tvec": tgt["tvec"] + x * u + y * v, "norm": tgt["norm"]})

        # Check for occlusions. Each corner has a boolean and an occlusion position
        occlusions = photogrammetry.get_occlusions_targets(
            rmat, tvec, corners, vis_checker
        )

        # Check that all corners are visible
        are_all_corners_visible = True
        for corner, occlusion in zip(corners, occlusions):
            # If there was no occlusion, continue
            if not occlusion[0]:
                continue

            # If there was an occlusion, check the distance from the occlusion to corner
            # If that distance is less than sqrt(2) * step_sz (since we step in both
            #   x and y) it is possible that the corner is inside the model so it is
            #   unfairly occluded.
            # If the distance is greater this corner is just occluded, no questions
            dist = np.linalg.norm(occlusion[1] - corner["tvec"])
            if dist > step_sz * np.sqrt(2):
                are_all_corners_visible = False
                break

            # To fairly check this corner, bump it to the occluded location
            bumped_corner = copy.copy(corner)
            bumped_corner["tvec"] = occlusion[1]

            # Check this bumped corner for occlusion
            bumped_occlusion = photogrammetry.get_occlusions_targets(
                rmat, tvec, [bumped_corner], vis_checker
            )[0]

            # If there is still an occlusion, this corner is likely occluded
            # There is a chance that the distance between the bumped occlusion and
            #   the original corner is still less than step_sz * np.sqrt(2). However
            #   the occurrence seems small and not worthy of further checks.
            # This is a potential TODO to set up as a while loop
            if bumped_occlusion[0]:
                are_all_corners_visible = False
                break

        # If all corners passed the visibility check, this corner is fine and we can
        #   move onto the next corner
        if are_all_corners_visible:
            tgts_filtered.append(tgt)
            img_targets_filtered.append(img_target)

    return tgts_filtered, img_targets_filtered


def filter_min_dist(tgts, img_targets, num_matches, min_dist=8):
    """Filters the targets and image target that are too close together in the image

    If the image locations of any two targets are below the min_dist threshold, remove
    both targets from the set of matched targets. This is to avoid ambiguity in target
    matching.

    Parameters
    ----------
    tgts : list
        3D targets. Each target should be a dict. The only strict requirement set by
        this function is ``tgts[i]`` is associated with ``img_targets[i]`` for i from 0
        to `num_matches`
    img_targets : list
        Matched image targets. Each dict has, at a minimum, the key 'center' which has a
        value of the image target center location (tuple/np.ndarray of floats).
        ``img_targets[i]`` is associated with ``tgts[i]`` for i from 0 to `num_matches`
    num_matches : int
        An integer of the number of matches. Must be less than or equal to
        ``min(len(tgts), len(img_targets))``
    min_dist : float, optional
        The minimum distance between two image targets. Any image targets closer than
        this distance are filtered out.

    Returns
    -------
    tgts_filtered : list
        Target positions (`tgts`) that have been filtered so that their image locations
        are separated by a distance larger than min_dist.
    img_targets_filtered : list
        Target positions in the image.
    num_matches_filtered : int

    Notes
    -----
    ``tgts_filtered[i]`` is associated with ``img_targets_filtered[i]`` for i from 0 to
    `num_matches_filtered`.
    """
    # Find the image targets that are too close together
    supressed = set()
    for i in range(num_matches):
        for j in range(i + 1, num_matches):
            if (i in supressed) and (j in supressed):
                continue

            # Calculate the distance between point i and j
            dist = np.linalg.norm(
                [
                    img_targets[i]["center"][0] - img_targets[j]["center"][0],
                    img_targets[i]["center"][1] - img_targets[j]["center"][1],
                ]
            )

            # If the distance is below the given threshold, add both i and j to the list
            #   of supressed points
            if dist < min_dist:
                supressed.add(i)
                supressed.add(j)
                continue

    # First add targets that pass the min_dist check
    tgts_filtered = []
    img_targets_filtered = []
    for i in range(num_matches):
        if i in supressed:
            continue
        tgts_filtered.append(tgts[i])
        img_targets_filtered.append(img_targets[i])

    # The number of matches is the number after the filter
    num_matches_filtered = len(tgts_filtered)

    # Then add the remaining targets and image targets
    tgts_filtered, img_targets_filtered = post_filter_append(
        tgts, tgts_filtered, img_targets, img_targets_filtered
    )

    return (tgts_filtered, img_targets_filtered, num_matches_filtered)


def filter_bifilter(
    rmat, tvec, cameraMatrix, distCoeffs, tgts, img_targets, num_matches, max_dist
):
    """Filters the targets and image target for ambiguity

    Check that only 1 img target is within a radius of max_dist from every projected
    target location. If more than 1 img target is near a target, do not use that target

    Check that only 1 projected target location is within a radius of max_dist from
    every img target. If more than 1 projected target location is near an img target,
    do not use that img target

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
    tgts : list
        3D targets. Each target is a dict and has, at a minimum, the keys 'tvec', and
        'target_type'. 'tvec' has a :class:`numpy.ndarray` (3, 1) representing the
        position of the target relative to the model origin for its associated value.
        'target_type' has a string representing the type of target (most commonly 'dot')
        for its associated value. ``tgts[i]`` is associated with ``img_targets[i]`` for
        i from 0 to `num_matches`
    img_targets : list
        Matched image targets. Each dict has, at a minimum, the key 'center' which has a
        value of is the image target center location (tuple/np.ndarray of floats).
        ``img_targets[i]`` is associated with ``tgts[i]`` for i from 0 to `num_matches`
    num_matches : int
        An integer of the number of matches. Must be less than or equal to
        ``min(len(tgts), len(img_targets))``
    max_dist : float
        The maximum distance between two image targets. Any image targets farther than
        this distance are filtered out.

    Returns
    -------
    tgts_bifilter : list
        Target positions (`tgts`) that have been bifiltered.
    img_targets_bifilter : list
        Target positions in the image.
    num_matches_bifilter : int

    Notes
    -----
    ``tgts_matched_bifilter[i]`` is associated with ``img_targets_bifilter[i]`` for i
    from 0 to `num_matches_bifilter`.
    """
    # Project the points into the image
    tgt_projs = photogrammetry.project_targets(
        rmat, tvec, cameraMatrix, distCoeffs, tgts
    )

    # Check that all targets have at most 1 img target nearby
    tgts_matched_temp, img_targets_temp = [], []
    num_matches_temp = 0
    for i in range(num_matches):
        tgt_proj = tgt_projs[i]

        bifilter_key = True
        for j, img_tgt in enumerate(img_targets):
            if i == j:
                continue

            dist = np.linalg.norm(tgt_proj["proj"] - img_tgt["center"])

            # If the distance is less than max_dist, this target fails the bifilter
            if dist < max_dist:
                bifilter_key = False
                break

        if bifilter_key:
            tgts_matched_temp.append(tgts[i])
            img_targets_temp.append(img_targets[i])
            num_matches_temp += 1

    tgts_matched_temp, img_targets_temp = post_filter_append(
        tgts, tgts_matched_temp, img_targets, img_targets_temp
    )

    # Project the points into the image
    tgt_projs = photogrammetry.project_targets(
        rmat, tvec, cameraMatrix, distCoeffs, tgts_matched_temp
    )

    # Check that all img targets have at most 1 target nearby
    tgts_bifilter, img_targets_bifilter = [], []
    num_matches_bifilter = 0
    for i in range(num_matches_temp):
        img_tgt = img_targets_temp[i]

        bifilter_key = True
        for j, tgt_proj in enumerate(tgt_projs):
            if i == j:
                continue

            dist = np.linalg.norm(
                np.array(img_tgt["center"]) - np.array(tgt_proj["proj"])
            )

            # If the distance is less than max_dist, this target fails the bifilter
            if dist < max_dist:
                bifilter_key = False
                break

        if bifilter_key:
            tgts_bifilter.append(tgts_matched_temp[i])
            img_targets_bifilter.append(img_targets_temp[i])
            num_matches_bifilter += 1

    tgts_bifilter, img_targets_bifilter = post_filter_append(
        tgts, tgts_bifilter, img_targets, img_targets_bifilter
    )

    return tgts_bifilter, img_targets_bifilter, num_matches_bifilter


def filter_one2one(tgts, img_targets, num_matches):
    """Filters the targets and image target to ensure matches are one-to-one

    Ensures that each target is matched with at most 1 img target, and that each img
    target is matched with at most 1 target. Remove anything that is not 1:1 as stated

    Parameters
    ----------
    tgts : list
        3D targets. Each target should be a dict. The only strict requirement set by
        this function is ``tgts[i]`` is associated with ``img_targets[i]`` for i from 0
        to `num_matches`
    img_targets : list
        Image targets. Each image target should be a dict, but this function does not
        set any strict requirements other than that ``img_targets[i]`` is associated
        with ``tgts[i]`` for i from 0 to `num_matches`
    num_matches : int
        An integer of the number of matches. Must be less than or equal to
        ``min(len(tgts), len(img_targets))``

    Returns
    -------
    tgts_filtered_one2one : list
        Target positions (`tgts`) that have been filtered for pairs that are one-to-one.
    img_targets_one2one : list
        Target positions in the image.
    num_matches_one2one : int

    Notes
    -----
    ``tgts_one2one[i]`` is associated with ``img_targets_one2one[i]`` for i from 0 to
    `num_matches_one2one`.
    """

    # Check that each img_target is used at most once
    tgts_filtered_temp = []
    img_targets_temp = []
    num_matches_temp = 0
    for i in range(num_matches):
        # Unpack the target and match
        tgt, match = tgts[i], img_targets[i]

        # Create a copy of the all matching points minus the current
        img_targets_subset = copy.deepcopy(img_targets[:num_matches])
        del img_targets_subset[i]

        # Convert the numpy arrays to lists for comparisons
        match_temp = copy.deepcopy(match)
        match_temp["center"] = match_temp["center"].flatten().tolist()
        for j in range(len(img_targets_subset)):
            img_targets_subset[j]["center"] = (
                img_targets_subset[j]["center"].flatten().tolist()
            )

        # If match is not in the list, there is a 1:1 matching and it can be included
        if match_temp not in img_targets_subset:
            tgts_filtered_temp.append(tgt)
            img_targets_temp.append(match)
            num_matches_temp += 1

    img_targets_subset = None  # deleteme

    # Check that each img_target is used at most once
    tgts_filtered_one2one = []
    img_targets_one2one = []
    num_matches_one2one = 0
    for i in range(num_matches_temp):
        # Unpack the target and match
        tgt, match = tgts_filtered_temp[i], img_targets_temp[i]

        # Create a copy of the all matching points minus the current
        tgts_subset = copy.deepcopy(tgts_filtered_temp[:num_matches_temp])
        del tgts_subset[i]

        # Convert the numpy arrays to lists for comparisons
        tgt_temp = copy.deepcopy(tgt)
        tgt_temp["tvec"] = tgt_temp["tvec"].flatten().tolist()
        tgt_temp["norm"] = tgt_temp["norm"].flatten().tolist()
        for j in range(len(tgts_subset)):
            tgts_subset[j]["tvec"] = tgts_subset[j]["tvec"].flatten().tolist()
            tgts_subset[j]["norm"] = tgts_subset[j]["norm"].flatten().tolist()

        # If match is not in the list, there is a 1:1 matching and it can be included
        if tgt_temp not in tgts_subset:
            tgts_filtered_one2one.append(tgt)
            img_targets_one2one.append(match)
            num_matches_one2one += 1

    tgts_filtered_one2one, img_targets_one2one = post_filter_append(
        tgts, tgts_filtered_one2one, img_targets, img_targets_one2one
    )

    return tgts_filtered_one2one, img_targets_one2one, num_matches_one2one


def filter_max_dist(
    rmat, tvec, cameraMatrix, distCoeffs, tgts, img_targets, num_matches, max_dist
):
    """Filters the targets and image target to pairs are not very far apart

    Any match where the distance between the projected target pixel position and the
    img target center is greater than max_dist is removed.

    Parameters
    ----------
    rmat : np.ndarray, shape (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray, shape (3, 1), float
        Translation vector from camera to object
    cameraMatrix : np.ndarray (3, 3), float
        The (openCV formatted) camera matrix for the camera
    distCoeffs : np.ndarray, shape (1, 5), float
        The (openCV formatted) distortion coefficients for the camera
    tgts : list
        3D targets. Each target is a dict and has, at a minimum, the keys 'tvec', and
        'target_type'. 'tvec' has a :class:`np.ndarray` (3, 1) representing the position
        of the target relative to the model origin for its associated value.
        'target_type' has a string representing the type of target (most commonly 'dot')
        for its associated value. ``tgts[i]`` is associated with ``img_targets[i]`` for
        i from 0 to `num_matches`.
    img_targets : list
        Matched image targets. Each dict has, at a minimum, the key 'center' which has a
        value of is the image target center location (tuple/np.ndarray of floats).
        ``img_targets[i]`` is associated with ``tgts[i]`` for i from 0 to num_matches
    num_matches : int
        An integer of the number of matches. Must be less than or equal to
        ``min(len(tgts), len(img_targets))``
    max_dist : float
        The maximum matching distance between a 3D target's project and an image targets
        center. Pairs with matching distance greater this distance are filtered out.

    Returns
    -------
    tgts_filtered : list
        Target positions (`tgts`) that have been filtered such that the matching
        distance is less than max_dist.
    img_targets_filtered : list
        Target positions in the image.
    num_matches_filtered : int

    Notes
    -----
    ``tgts_filtered[i]`` is associated with ``img_targets_filtered[i]`` for i from
    0 to `num_matches_filtered`.
    """
    # Project the points into the image
    tgt_projs = photogrammetry.project_targets(
        rmat, tvec, cameraMatrix, distCoeffs, tgts
    )

    # First, add the targets that are within max_dist from their image targets
    tgts_filtered, img_targets_filtered = [], []
    for i in range(num_matches):
        tgt_proj, img_target = tgt_projs[i], img_targets[i]

        dist = np.linalg.norm(
            np.array(tgt_proj["proj"]) - np.array(img_target["center"])
        )
        if dist < max_dist:
            tgts_filtered.append(tgts[i])
            img_targets_filtered.append(img_targets[i])

    num_matches_filtered = len(tgts_filtered)

    tgts_filtered, img_targets_filtered = post_filter_append(
        tgts, tgts_filtered, img_targets, img_targets_filtered
    )

    return tgts_filtered, img_targets_filtered, num_matches_filtered


def filter_nones(tgts, img_targets, num_matches):
    """Filters the targets and image target to remove None objects

    :func:`match_obj_and_img_pts` matches each target to the closest img target.
    Therefore, it is possible that not every img target will have a match. Additionally,
    if ``max_dist`` is given to :func:`match_obj_and_img_pts`, it is possible that not
    every target will have a match. :func:`match_obj_and_img_pts` solves this by
    matching unmatched items to None.

    That causes something to fail, so this function reformats that output. Instead of
    using Nones, this function reorders the list of targets to the first n target have
    a match, and correspond to the first n items of the img targets list. n is
    additionally given as num_matches

    Parameters
    ----------
    tgts : list
        3D targets. Each target should be a dict. The only strict requirement set by
        this function is ``tgts[i]`` is associated with ``img_targets[i]`` for i from 0
        to `num_matches`
    img_targets : list
        Image targets. Each image target should be a dict, but this function does not
        set any strict requirements other than that ``img_targets[i]`` is associated
        with ``tgts[i]``
    num_matches : int
        An integer of the number of matches. Must be less than or equal to
        ``min(len(tgts), len(img_targets))``

    Returns
    -------
    tgts_filtered : list
        Target positions (`tgts`) that have been filtered to remove None values.
    img_targets_filtered : list
        Target positions in the image.
    num_matches_filtered : int

    Notes
    -----
    ``tgts_filtered[i]`` is associated with ``img_targets_filtered[i]`` for i from
    0 to `num_matches_filtered`.
    """
    # Initialized filtered lists and a count
    tgts_filtered = []
    img_targets_filtered = []
    num_matches_filtered = 0

    # First add the matches where the img_targets are not None
    #   Increment the count of matched targets
    for i in range(num_matches):
        tgt, img_target = tgts[i], img_targets[i]
        if img_target is not None:
            tgts_filtered.append(tgt)
            img_targets_filtered.append(img_target)
            num_matches_filtered += 1

    tgts_filtered, img_targets_filtered = post_filter_append(
        tgts, tgts_filtered, img_targets, img_targets_filtered
    )

    return tgts_filtered, img_targets_filtered, num_matches_filtered


def filter_matches(
    rmat,
    tvec,
    cameraMatrix,
    distCoeffs,
    tgts,
    img_targets,
    num_matches,
    test_config,
    debug=None,
):
    """Wrapper to run multiple filtering functions

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
    tgts : list
        3D targets. Each target is a dict that has, at a minimum, the keys 'tvec', and
        'target_type'. 'tvec' has a :class:`numpy.ndarray` (3, 1) representing the
        position of the target relative to the model origin for its associated value.
        'target_type' has a string representing the type of target (most commonly 'dot')
        for its associated value. ``tgts[i]`` is associated with ``img_targets[i]`` for
        i from 0 to `num_matches`
    img_targets : list
        Matched image targets. Each dict has, at a minimum, the key 'center' which has a
        value of is the image target center location (tuple/np.ndarray of floats).
        ``img_targets[i]`` is associated with ``tgts[i]`` for i from 0 to `num_matches`
    num_matches : int
        An integer of the number of matches. Must be less than or equal to
        ``min(len(tgts), len(img_targets))``
    test_config : dict
        Processing parameters with, at a minimum, a key for 'min_dist', 'max_dist'.
        'min_dist' is the minimum distance between two image targets. 'max_dist' is the
        maximum allowable distance between an image target and the projection of a 3D
        target
    debug : tuple, optional
        tuple of length 2. First item is the image to use as the background. Second item
        is the name of the debug image

    Returns
    -------
    tgts_filtered: list
        Targets filtered to remove None values, matches greater than the maximum
        matching distance, matches that are not one-to-one, matches that do not pass a
        "bifilter" test, and matches that have image locations that are too close
        together
    img_targets_filtered: list
        Associated image locations of `tgts_filtered`
    """
    # If the targets aren't empty, perform all the filtering operations
    if (num_matches != 0) and (len(tgts) != 0) and (len(img_targets) != 0):
        tgts_a, img_targets_a, num_matches_a = filter_nones(
            tgts, img_targets, num_matches
        )

        tgts_b, img_targets_b, num_matches_b = filter_max_dist(
            rmat,
            tvec,
            cameraMatrix,
            distCoeffs,
            tgts_a,
            img_targets_a,
            num_matches_a,
            test_config["max_dist"],
        )

        tgts_c, img_targets_c, num_matches_c = filter_one2one(
            tgts_b, img_targets_b, num_matches_b
        )

        tgts_d, img_targets_d, num_matches_d = filter_bifilter(
            rmat,
            tvec,
            cameraMatrix,
            distCoeffs,
            tgts_c,
            img_targets_c,
            num_matches_c,
            test_config["max_dist"],
        )

        tgts_e, img_targets_e, num_matches_e = filter_min_dist(
            tgts_d, img_targets_d, num_matches_d, test_config["min_dist"]
        )

        tgts_filtered = tgts_e[:num_matches_e]
        img_targets_filtered = img_targets_e[:num_matches_e]

    # If the targets are empty, populate the stages of filtering with the initialized
    #   values
    else:
        tgts_a, img_targets_a, num_matches_a = tgts, img_targets, num_matches
        tgts_b, img_targets_b, num_matches_b = tgts, img_targets, num_matches
        tgts_c, img_targets_c, num_matches_c = tgts, img_targets, num_matches
        tgts_d, img_targets_d, num_matches_d = tgts, img_targets, num_matches
        tgts_e, img_targets_e, num_matches_e = tgts, img_targets, num_matches
        tgts_filtered, img_targets_filtered = [], []

    # If the debug input is not None (default), create an image of the tgts and
    #   img_targets at each stage of the filtering operation
    if debug is not None:
        tgts_img_targets_num_matches = [
            [tgts, img_targets, num_matches],
            [tgts_a, img_targets_a, num_matches_a],
            [tgts_b, img_targets_b, num_matches_b],
            [tgts_c, img_targets_c, num_matches_c],
            [tgts_d, img_targets_d, num_matches_d],
            [tgts_e, img_targets_e, num_matches_e],
        ]

        for i, data in enumerate(tgts_img_targets_num_matches):
            tgts_temp, img_targets_temp, num_matches_temp = data

            tgt_projs = photogrammetry.project_targets(
                rmat, tvec, cameraMatrix, distCoeffs, tgts_temp
            )

            proj_pts = np.array([tgt_proj["proj"] for tgt_proj in tgt_projs])
            img_centers = np.array(
                [img_target["center"] for img_target in img_targets_temp]
            )

            suffix = {
                0: "Original_match",
                1: "filtered_a",
                2: "filtered_b",
                3: "filtered_c",
                4: "filtered_d",
                5: "filtered_e",
            }[i]
            name = debug[1] + "_" + suffix

            visualization.show_projection_matching(
                debug[0],
                proj_pts,
                img_centers,
                num_matches=num_matches_temp,
                name=name,
                bonus_pt=None,
                scale=1,
                ax=None,
            )

    return tgts_filtered, img_targets_filtered


def match_obj_and_img_pts(
    rmat, tvec, cameraMatrix, distCoeffs, tgts, img_targets, max_dist=np.inf
):
    """Matches 3D targets to the image targets

    Projects the 3D targets into the image, then finds the closest image target. If the
    closest image target is less than max_dist pixels away, it is matched. If it is
    farther than `max_dist` pixels, it is matched to None. This matching scheme does not
    ensure matches are one-to-one.

    Parameters
    ----------
    rmat : np.ndarray, shape (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray, shape (3,), float
        Translation vector from camera to object
    cameraMatrix : np.ndarray, shape (3, 3), float
        The (openCV formatted) camera matrix for the camera
    distCoeffs : np.ndarray, shape (5,), float
        The (openCV formatted) distortion coefficients for the camera
    tgts : np.ndarray, shape (n, 3)
        List of 3D targets.
    img_targets : list of dict
        Matched image target information. Each dict has, at a minimum, keys 'center',
        and 'target_type'. 'center' has a value of the image target center location
        (tuple/np.ndarray of length 2 of floats) and 'target_type' has the key of the
        type of target (string). ``img_targets[i]`` is associated with ``tgts[i]``
    max_dist : float
        The maximum matching distance between a 3D target's project and an image targets
        center. Pairs with matching distance greater this distance are filtered out.

    Returns:
    -----------
    matching_img_targets : list
        List of the items from the `img_targets` input such that
        ``matching_img_targets[i]`` is the closest image target to the projected
        position of ``tgts[i]``. If the closest image target is farther than `max_dist`,
        ``matching_img_targets[i]`` is None
    """

    # Project the points into the image
    tgt_projs = photogrammetry.project_targets(
        rmat, tvec, cameraMatrix, distCoeffs, tgts
    )

    # For each target, find the matching image point of the same type
    # Matches are assumed to be the closest point
    matching_img_targets = []
    for tgt_proj in tgt_projs:
        match = None
        match_dist = max_dist
        for img_target in img_targets:
            # Check that the target types match
            if img_target["target_type"] != tgt_proj["target_type"]:
                continue

            # Calculate the distance
            dist = np.linalg.norm(
                [
                    img_target["center"][0] - tgt_proj["proj"][0],
                    img_target["center"][1] - tgt_proj["proj"][1],
                ]
            )

            # If the distance is over the distance threshold and less than current
            #   lowest distance, make this the new best match
            if dist < match_dist:
                match_dist = dist
                match = copy.deepcopy(img_target)

        matching_img_targets.append(match)

    # Since the target ordering is not changes, we can return the visibles and
    #   projected as was given
    return matching_img_targets


def match_targets(
    rmat, tvec, cameraMatrix, distCoeffs, tgts, img_targets, max_dist=np.inf, debug=None
):
    """Matches each target to the closest img target

    If `max_dist` is given, distance between target and img target must be less than
    max_dist. By default this value is infinite, so any match is valid.

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
    tgts : list
        3D targets. Each target is a dict and has, at a minimum, the keys 'tvec', and
        'target_type'.  'tvec' has a :class:`numpy.ndarray` (3, 1) representing the
        position of the target relative to the model origin for its associated value.
        'target_type' has a string representing the type of target (most commonly 'dot')
        for its associated value. ``tgts[i]`` is associated with ``img_targets[i]`` for
        i from 0 to `num_matches`
    img_targets : list of dict
        Each dict has, at a minimum, keys 'center', and 'target_type'. 'center' has a
        value of the image target center location (tuple/np.ndarray of length 2 of
        floats) and 'target_type' has the key of the type of target (string).
        ``img_targets[i]`` is associated with ``tgts[i]``
    max_dist : float
        The maximum matching distance between a 3D target's project and an image targets
        center. Pairs with matching distance greater this distance are filtered out.
    debug : tuple, optional
        Debug option. For no debugging, give None. To generate debugging images, give
        a tuple of length 3. First item is the image to use as the background.
        Second item is the name of the debug image. Third item can be a dict or None. If
        dict, matches will be filtered before the debug image is created. Dict needs to
        follow test_config input requirements of filter_matches. If None, matches will
        not be filtered in the debug image

    Returns
    -------
    tgts_matched: list
        3D targets that are matched
    matching_img_targets: list
        Image locations of matched targets
    num_matches: int

    Notes
    -----
    ``tgts_matched[i]`` is matched with ``matching_img_targets[i]`` for i from 0 to
    `num_matches`.
    """

    # Match the visible targets to the closest image point
    matching_img_targets = match_obj_and_img_pts(
        rmat, tvec, cameraMatrix, distCoeffs, tgts, img_targets, max_dist
    )

    tgts_matched, matching_img_targets, num_matches = filter_nones(
        tgts, matching_img_targets, len(tgts)
    )
    tgts_matched, matching_img_targets = post_filter_append(
        tgts, tgts_matched, img_targets, matching_img_targets
    )

    if debug is not None:
        if debug[2] is not None:
            tgts_matched_temp, img_targets_temp = filter_matches(
                rmat,
                tvec,
                cameraMatrix,
                distCoeffs,
                tgts_matched,
                matching_img_targets,
                num_matches,
                debug[2],
                debug[:2],
            )
            num_matches_temp = len(tgts_matched_temp)
        else:
            tgts_matched_temp, img_targets_temp = tgts_matched, matching_img_targets
            num_matches_temp = num_matches

        tgts_matched_temp_temp = copy.deepcopy(tgts_matched_temp)
        tgts_matched_temp_temp = [copy.deepcopy(tgt) for tgt in tgts_matched_temp]
        for tgt in tgts_matched_temp_temp:
            tgt["tvec"] = tgt["tvec"].flatten().tolist()
            tgt["norm"] = tgt["norm"].flatten().tolist()

        tgts_unmatched = []
        for tgt in tgts:
            tgt_temp = copy.deepcopy(tgt)
            tgt_temp["tvec"] = tgt_temp["tvec"].flatten().tolist()
            tgt_temp["norm"] = tgt_temp["norm"].flatten().tolist()
            if tgt_temp not in tgts_matched_temp_temp:
                tgts_unmatched.append(tgt)

        img_targets_temp_temp = copy.deepcopy(img_targets_temp)
        img_targets_temp_temp = [copy.deepcopy(tgt) for tgt in img_targets_temp]
        for img_tgt in img_targets_temp_temp:
            img_tgt["center"] = img_tgt["center"].flatten().tolist()

        img_targets_unmatched = []
        for img_tgt in img_targets:
            img_tgt_temp = copy.deepcopy(img_tgt)
            img_tgt_temp["center"] = img_tgt_temp["center"].flatten().tolist()
            if img_tgt_temp not in img_targets_temp_temp:
                img_targets_unmatched.append(img_tgt)

        tgt_projs = photogrammetry.project_targets(
            rmat, tvec, cameraMatrix, distCoeffs, tgts_matched_temp + tgts_unmatched
        )

        proj_pts = np.array([tgt_proj["proj"] for tgt_proj in tgt_projs])
        img_centers = np.array(
            [
                img_target["center"]
                for img_target in img_targets_temp + img_targets_unmatched
            ]
        )

        visualization.show_projection_matching(
            debug[0],
            proj_pts,
            img_centers,
            num_matches=num_matches_temp,
            name=debug[1],
            bonus_pt=None,
            scale=1,
            ax=None,
        )

    return tgts_matched, matching_img_targets, num_matches


def post_filter_append(tgts, tgts_filtered, img_targets, img_targets_filtered):
    """Adds tgts that were filtered back into the list

    The match-and-filter scheme we use is to have two ordered lists, where
    ``tgts_filtered[i]`` is matched to ``img_targets_filtered[i]`` for i from 0 to
    `num_matches`. For i greater than num_matches, the tgt and img_target are not
    matched. We leave all unmatched targets in the list, because some filtering
    operations need them. Ex the the filters for min_dist and bi_filter.

    This function takes in the whole (randomly ordered) `tgts` and `img_targets`, as
    well as the (ordered) `tgts_filtered` and `img_targets_filtered`. Any `tgts` not in
    `tgts_filtered` are appended at the end, and similar for the `img_targets`

    Parameters
    ----------
    tgts : list of dict
        List of all targets, order is not important. Each target is a dict that has, at
        a minimum, the keys 'tvec', and 'target_type'. If isMatched is False, 'norm' is
        additionally needed as a key. 'tvec' has a :class:`numpy.ndarray` (3, 1)
        representing the position of the target relative to the model origin for its
        associated value. 'target_type' has a string representing the type of target
        (most commonly 'dot') for its associated value.
    tgts_filtered : list of dict
        List of filtered targets, order is important. ``tgts_filtered[i]`` is associated
        with ``img_targets_filtered[i]`` for i from 0 to `num_matches`. Each target is a
        dict that has, at a minimum, the keys 'tvec', and 'target_type'. If isMatched is
        False, 'norm' is additionally needed as a key. 'tvec' has a
        :class:`numpy.ndarray` (3, 1) representing the position of the target relative
        to the model origin for its associated value. 'target_type' has a string
        representing the type of target (most commonly 'dot') for its associated value.
    img_targets : list of dict
        List of all matched image targets, order is not important. Each dict has, at a
        minimum, keys 'center', and 'target_type'. 'center' has a value of the image
        target center location (tuple/np.ndarray of length 2 of floats) and
        'target_type' has the key of the type of target (string).
    img_targets_filtered : list of dict
        List of filtered image targets, order is important. ``img_targets_filtered[i]``
        is associated with ``tgts_filtered[i]`` for i from 0 to `num_matches`. Each dict
        has, at a minimum, keys 'center', and 'target_type'. 'center' has a value of the
        image target center location (tuple/np.ndarray of length 2 of floats) and
        'target_type' has the key of the type of target (string).
    """
    tgts_filtered_temp = copy.deepcopy(tgts_filtered)
    img_targets_filtered_temp = copy.deepcopy(img_targets_filtered)

    tgts_filtered_list = copy.deepcopy(tgts_filtered)
    for i in range(len(tgts_filtered_list)):
        tgts_filtered_list[i]["tvec"] = tgts_filtered_list[i]["tvec"].flatten().tolist()
        tgts_filtered_list[i]["norm"] = tgts_filtered_list[i]["norm"].flatten().tolist()

    # Add tgts that were not matched
    for tgt in tgts:
        tgt_list = copy.deepcopy(tgt)
        tgt_list["tvec"] = tgt_list["tvec"].flatten().tolist()
        tgt_list["norm"] = tgt_list["norm"].flatten().tolist()
        if tgt_list not in tgts_filtered_list:
            tgts_filtered_temp.append(tgt)

    img_targets_filtered_list = copy.deepcopy(img_targets_filtered)

    for i in range(len(img_targets_filtered_list)):
        img_targets_filtered_list[i]["center"] = (
            img_targets_filtered_list[i]["center"].flatten().tolist()
        )

    # Add img_target that were not matched
    for img_target in img_targets:
        if img_target is not None:
            img_target_list = copy.deepcopy(img_target)
            img_target_list["center"] = img_target_list["center"].flatten().tolist()
            if img_target_list not in img_targets_filtered_list:
                img_targets_filtered_temp.append(img_target)

    return tgts_filtered_temp, img_targets_filtered_temp


# ---------------------------------------------------------------------------------------
# External Calibration Wrappers


def external_calibrate(
    img,
    rmat,
    tvec,  # Frame specific
    cameraMatrix,
    distCoeffs,  # Camera specific
    tgts,
    img_targets,
    vis_checker,
    test_config,  # Config specific
    isMatched=False,
    max_localize_delta=None,  # Test specific
    reprojectionError=6.0,  # Test specific (should be stable between tests)
):
    """Set up and run solvePnPRansac to get the external calibration and inlier targets

    Parameters
    ----------
    img : np.ndarray, shape (height, width)
        Image to use for calibration
    rmat : np.ndarray, shape (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray, shape (3, 1), float
        Translation vector from camera to object
    cameraMatrix : np.ndarray, shape (3, 3), float
        The (openCV formatted) camera matrix for the camera
    distCoeffs : np.ndarray, shape (1, 5), float
        The (openCV formatted) distortion coefficients for the camera
    tgts : list of dict
        Each target is a dict that has, at a minimum, the keys 'tvec', and
        'target_type'. If isMatched is False, 'norm' is additionally needed as a key.
        'tvec' has a np.ndarray (3, 1) representing the position of the target relative
        to the model origin for its associated value. 'target_type' has a string
        representing the type of target (most commonly 'dot') for its associated value.
        ``tgts[i]`` is associated with ``img_targets[i]`` for i from 0 to `num_matches`.
        'norm' has a :class:`numpy.ndarray` (3, 1) representing the normal vector of the
        target relative to the model coordinate system for its associated value
    img_targets : list of dict
        Matched image targets. Each dict has, at a minimum, keys 'center', and
        'target_type'. 'center' has a value of the image target center location
        (tuple/np.ndarray of length 2 of floats) and 'target_type' has the key of the
        type of target (string). ``img_targets[i]`` is associated with ``tgts[i]``
    vis_checker : ~upsp.cam_cal_utils.visibility.VisibilityChecker
        Visibility checker object with the relevant BVH and oblique viewing angle
    test_config : dict
        dict with, at a minimum, a key for 'min_dist', 'max_dist', and each target type
        in targets and img_targets. 'min_dist' is the minimum distance between two image
        targets. 'max_dist' is the maximum allowable distance between an image target
        and the projection of a 3D target. The key for each target type is
        `target_type` + '_pad'. This is the padding around the img target center
        location to use to sub-pixel localize
    isMatched : bool
        If True, denotes that ``tgts[i]`` is associated with ``img_targets[i]`` and all
        targets are visible to the camera. If False, denotes `tgts` and `img_targets`
        are not in any particular order. If False, targets are checked for visibility
        and :func:`match_targets` is used to match the tgts to the image targets
    max_localize_delta : float, optional
        Parameter passed to :func:`subpixel_localize`
    reprojectionError : float, optional
        Maximum reprojection error between a target and image target to be considered an
        inlier. ReprojectionError is often smaller than ``test_config['max_dist']``
        since it is the optimized distance between the target and image target.

    Returns
    -------
    rmat_opt
        optimized rotation matrix from the camera to the model
    tvec_opt
        optimized translation vector from the camera to the model
    tgt_inliers
        list of inlier targets of the optimization
    img_target_inliers
        list of the inlier image targets of the optimization

    Notes
    -----
    ``tgt_inliers[i]`` is associated with ``img_target_inliers[i]``
    """
    # If the inputs are not matched, get the visible targets and match them.
    if not isMatched:
        # Determine which targets are visible
        visibles_tgts = photogrammetry.get_visible_targets(
            rmat, tvec, tgts, vis_checker
        )

        # Match the projected locations to the image locations
        tgts_matched, img_targets_matched, num_matches_init = match_targets(
            rmat,
            tvec,
            cameraMatrix,
            distCoeffs,
            visibles_tgts,
            img_targets,
            test_config["max_dist"],
        )

    # If the input is matched, short circuit the tgt_matched and
    #   img_targets_matched with the inputs
    else:
        tgts_matched = tgts
        img_targets_matched = img_targets
        num_matches_init = len(tgts)

    # Filter the matched targets
    tgts_filtered, img_targets_filtered = filter_matches(
        rmat,
        tvec,
        cameraMatrix,
        distCoeffs,
        tgts_matched,
        img_targets_matched,
        num_matches_init,
        test_config,
    )

    # Sub-pixel localize the image targets
    tgts_subpixel, img_targets_subpixel = subpixel_localize(
        img,
        tgts_filtered,
        img_targets_filtered,
        test_config,
        max_localize_delta=max_localize_delta,
    )

    # If there are less than 4 matches, raise an error
    if len(tgts_subpixel) < 4:
        raise ValueError(
            "Less than 4 matches were found in external_calibrate. "
            + "This can be due to blob detection finding too few targets, too few "
            + "visible targets, a bad matching scheme due to a bad starting pose "
            + "(rmat and tvec), and/or too many targets rejected during the sub-pixel "
            + "localization."
        )

    # Package the target tvecs
    tgt_tvecs = []
    for tgt in tgts_subpixel:
        tgt_tvecs.append(tgt["tvec"])
    tgt_tvecs = np.array(tgt_tvecs)

    # Package the image points
    img_centers = []
    for target in img_targets_subpixel:
        img_centers.append(target["center"])
    img_centers = np.array(img_centers)

    # Convert rmat to rvec
    rvec, _ = cv2.Rodrigues(rmat)

    # Solve for the new tvec and rvec
    retval, rvec_opt, tvec_opt, inliers = cv2.solvePnPRansac(
        tgt_tvecs,
        img_centers,
        cameraMatrix,
        distCoeffs,
        copy.copy(rvec),
        copy.copy(tvec),
        reprojectionError=reprojectionError,
        useExtrinsicGuess=True,
    )

    if len(inliers):
        inliers = np.squeeze(inliers, axis=1)

    # Package all the inlier matching points
    tgt_inliers = []
    img_target_inliers = []
    for i in inliers:
        tgt_inliers.append(tgts_subpixel[i])
        img_target_inliers.append(img_targets_subpixel[i])

    # Convert rvec_opt to rmat_opt
    rmat_opt, _ = cv2.Rodrigues(rvec_opt)

    return rmat_opt, tvec_opt, tgt_inliers, img_target_inliers


def check_external_calibrate_two_stage_inputs(
    img, rmat_init_guess, tvec_init_guess, incal, tgts, test_config, vis_checker, debug
):
    """Check that the inputs to external calibrate are valid

    Helper function to :func:`external_calibrate_two_stage`. Checks that the input are
    of the proper type, size, format, and have the relevant attributes.

    Parameters
    ----------
    img : np.ndarray, shape (h, w)
        Numpy 2D array of the image
    rmat_init_guess : np.ndarray (3, 3), float
        Initial guess of rotation matrix from camera to object
    tvec_init_guess : np.ndarray (3, 1), float
        Initial guess of translation vector from camera to object
    incal : tuple
        Camera internal calibration.

        - ``cameraMatrix`` (:class:`numpy.ndarray`, shape (3, 3)): The (OpenCV
          formatted) camera matrix for the camera
        - ``distCoeffs`` (:class:`numpy.ndarray`, shape (1, 5): The (OpenCV formatted)
          distortion coefficients for the camera
    tgts : list of dict
        Each target is a dict and has, at a minimum, the keys 'tvec', 'target_type',
        'norm'. 'tvec' has a :class:`numpy.ndarray` (3, 1) representing the position of
        the target relative to the model origin for its associated value. 'norm' has a
        :class:`numpy.ndarray` (3, 1) representing the normal vector of the target
        relative to the model origin for its associated value. 'target_type' has a
        string representing the type of target (most commonly 'dot') for its associated
        value. ``tgts[i]`` is associated with ``img_targets[i]``
    test_config : dict
        Processing parameters with, at a minimum, a key for 'min_dist', 'max_dist', and
        2 keys for the primary target type in targets and img_targets. 'min_dist' is the
        minimum distance between two image targets. 'max_dist' is the maximum allowable
        distance between an image target and the projection of a 3D target. The primary
        target type is 'dot' if there are 4+ dots in tgts. Otherwise the primary target
        type is 'kulite'. The keys for the primary target type are target_type + '_pad'
        and `target_type` + '_blob_parameters'. The first is the padding around the img
        target center location to use to sub-pixel localize. The second is the blob
        detection parameters for that type of target
    vis_checker : ~upsp.cam_cal_utils.visibility.VisibilityChecker
        Visibility checker object with the relevant BVH and oblique viewing angle
    debug : string, optional
        Name for all debug images (potentially camera name, test an camera name, etc).

    Returns
    -------
    valid : bool
        True if the all inputs are valid, False if any input is invalid
    """
    # TODO: Any time "isinstance(OBJ, list)" is called, np.array(OBJ) should be inside
    #   of a try-except in case it has degenerate objects

    # -----------------------------------------------------------------------------------
    # Check img input
    if isinstance(img, np.ndarray):
        if img.dtype != np.uint8:
            print("img dtype should be uint8 not", img.dtype)
            return False

        # If img is not 2 dimensional, it should be 3 dimensional, but single channel
        if len(img.shape) != 2:
            if len(img.shape) == 3:
                if img.shape[2] != 1:
                    print("img should be single channel, not", img.shape[2], "channel")
                    return False
            else:
                # img is not 2 dimensional or 3 dimension. Must be an error
                print(
                    "img should be 2 dimensional, or 3 dimensional and single channel. Not ",
                    len(img.shape),
                )
                return False

    else:
        print("img type should be np.ndarray. Not ", type(img))
        return False

    # -----------------------------------------------------------------------------------
    # Check rmat_init_guess input

    # Check rmat_init_guess is a numpy array of shape (3, 3)
    if isinstance(rmat_init_guess, np.ndarray):
        if rmat_init_guess.dtype != np.float32 and rmat_init_guess.dtype != np.float64:
            print(
                "rmat_init_guess.dtype should be float32 or float64 not",
                rmat_init_guess.dtype,
            )
            return False
        if rmat_init_guess.shape != (3, 3):
            print("rmat_init_guess.shape should be (3, 3). Not ", rmat_init_guess.shape)
            return False
        if not photogrammetry.isRotationMatrix(rmat_init_guess):
            print("rmat_init_guess is not a valid rotation matrix")
            return False

    else:
        print("rmat_init_guess should be np.ndarray. Not", type(rmat_init_guess))
        return False

    # -----------------------------------------------------------------------------------
    # Check tvec_init_guess input

    # Check tvec_init_guess is a numpy array of shape (3, 1)
    if isinstance(tvec_init_guess, np.ndarray):
        if tvec_init_guess.dtype != np.float32 and tvec_init_guess.dtype != np.float64:
            print(
                "tvec_init_guess.dtype should be float32 or float64 not",
                tvec_init_guess.dtype,
            )
            return False
        if tvec_init_guess.shape != (3, 1):
            print("tvec_init_guess.shape should be (3, 1). Not ", tvec_init_guess.shape)
            return False

    else:
        print("tvec_init_guess should be np.ndarray. Not", type(tvec_init_guess))
        return False

    # -----------------------------------------------------------------------------------
    # Check incal input

    # if incal[0] is a list, convert it to a numpy array to check the data
    if isinstance(incal[0], list):
        incal[0] = np.array(incal[0])

    if isinstance(incal[0], np.ndarray):
        if incal[0].dtype != np.float32 and incal[0].dtype != np.float64:
            print("incal[0].dtype should be float32 or float64 not", incal[0].dtype)
            return False
        if incal[0].shape != (3, 3):
            print("incal[0].shape should be (3, 3). Not ", incal[0].shape)
            return False

        # incal should be of the form [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
        if (incal[0][2] != [0.0, 0.0, 1.0]).any():
            print("incal[0][2] must be [0, 0, 1]")
            return False
        if incal[0][0][1] != 0:
            print("incal[0][0][1] must be 0")
            return False
        if incal[0][1][0] != 0:
            print("incal[0][1][0] must be 0")
            return False

    else:
        print("incal[0] should be np.ndarray or list. Not", type(incal[0]))
        return False

    # if incal[1] is a list, convert it to a numpy array to check the data
    if isinstance(incal[1], list):
        incal[1] = np.array(incal[1])

    if isinstance(incal[1], np.ndarray):
        if incal[1].dtype != np.float32 and incal[1].dtype != np.float64:
            print("incal[1].dtype should be float32 or float64 not", incal[1].dtype)
            return False
        if incal[1].shape != (1, 5):
            print("incal[1].shape should be (1, 5). Not ", incal[1].shape)
            return False

    else:
        print("incal[1] should be np.ndarray or list. Not", type(incal[1]))
        return False

    # -----------------------------------------------------------------------------------
    # Check tgts input

    # Check that each target is as expected
    for tgt in tgts:
        if "tvec" not in tgt or "norm" not in tgt or "target_type" not in tgt:
            print(
                "Each item in tgts must have a tvec, norm, and target_type. One or more of the items in tgts did not meet this criteria"
            )
            return False
        if isinstance(tgt["tvec"], np.ndarray) and isinstance(tgt["norm"], np.ndarray):
            if (tgt["tvec"].shape != (3, 1)) or (tgt["norm"].shape != (3, 1)):
                print(
                    "tgt tvec and norm must have shape (3, 1). One or more of the items in tgts did not meet this criteria"
                )
                return False
        else:
            print(
                "tgt tvec and norm must be np.ndarray. One or more of the items in tgts did not meet this criteria"
            )
            return False

    if len(tgts) < 4:
        print("tgts must have at least 4 items. Only", len(tgts), "were found")
        return False

    # -----------------------------------------------------------------------------------
    # Check test_config input

    if ("dot_blob_parameters" not in test_config.keys()) and (
        "kulite_blob_parameters" not in test_config.keys()
    ):
        print(
            "Neither 'dot_blob_parameters' nor 'kulite_blob_parameters' in test_config.keys()"
        )
        return False

    if "dot_blob_parameters" in test_config.keys():
        if not isinstance(test_config["dot_blob_parameters"], list):
            print(
                "test_config['dot_blob_parameters'] should be a list, not",
                type(test_config["dot_blob_parameters"]),
            )
            return False
    if "kulite_blob_parameters" in test_config.keys():
        if not isinstance(test_config["kulite_blob_parameters"], list):
            print(
                "test_config['kulite_blob_parameters'] should be a list, not",
                type(test_config["kulite_blob_parameters"]),
            )
            return False

    # TODO: Need to check that test_config['blob_parameters'] contains all valid
    #   items. But that is a lot of work, and it fails early enough in
    #   external_calibration_two_stage that it doesn't matter much

    if "max_dist" not in test_config.keys():
        print("'max_dist' not in test_config.keys()")
        return False

    if not isinstance(test_config["max_dist"], (int, float)):
        print("test_config['max_dist'] must be numeric")
        return False

    if "min_dist" not in test_config.keys():
        print("'min_dist' not in test_config.keys()")
        return False

    if not isinstance(test_config["min_dist"], (int, float)):
        print("test_config['min_dist'] must be numeric")
        return False

    # -----------------------------------------------------------------------------------
    # Check vis_checker input

    if not type(vis_checker).__name__ == "VisibilityChecker":
        print(
            "vis_checker should be visibility.VisibilityChecker object. Not",
            type(vis_checker),
        )
        return False

    if not type(vis_checker.scene).__name__ == "BVH":
        print(
            "vis_checker.scene should be upsp.raycast.CreateBVH object. Not",
            type(vis_checker.scene),
        )
        return False

    # -----------------------------------------------------------------------------------
    # Check debug input

    # If debug is not None object, a string, or integer then it is improper
    if debug != None and not isinstance(debug, str) and not isinstance(debug, int):
        print("debug should be None, string, or int. Not", type(debug))
        return False

    return True


def external_calibrate_two_stage_from_wtd(
    img,  # Frame specific
    tunnel_vals,  # Datapoint specific
    camera_tunnel_cal,  # Camera specific
    tgts,
    test_config,
    vis_checker,  # Test/configuration specific
    debug=None,
):
    """Wrapper function to :func:`external_calibrate_two_stage`.

    The `tunnel_vals` plus `test_config` are used to estimate an initial guess of `rmat`
    and `tvec`. That initial guess should project each target's within ~5 pixels of the
    associated image target

    Parameters
    ----------
    img : np.ndarray, shape (h, w)
        Numpy 2D array of the image
    tunnel_vals : dict
        Wind tunnel data as a dict with (at a minimum) the keys 'ALPHA', 'BETA', 'PHI',
        and 'STRUTZ'. ALPHA, BETA, and PHI are tunnel angles in degrees. STRUTZ is the
        offset of the tunnel center of rotation for the z axis in inches
    camera_tunnel_cal : tuple
        Camera-tunnel calibration

        - ``rmat_camera_tunnel`` (:class:`numpy.ndarray`, shape (3, 3)): Rotation matrix
          from camera to tunnel at wind off condition
        - ``tvec_camera_tunnel`` (:class:`numpy.ndarray`, shape (3, 1)): Translation
          vector from camera to tunnel at wind off condition
        - ``cameraMatrix`` (:class:`numpy.ndarray`, shape (3, 3)): The (OpenCV
          formatted) camera matrix for the camera
        - ``distCoeffs`` (:class:`numpy.ndarray`, shape (1, 5)): The (OpenCV formatted
          distortion coefficients for the camera
    tgts : list of dict
        3D targets. Each target is a dict and has, at a minimum, the keys 'tvec',
        'target_type', 'norm'. 'tvec' has a np.ndarray (3, 1) representing the position
        of the target relative to the model origin for its associated value. 'norm' has
        a :class:`numpy.ndarray` (3, 1) representing the normal vector of the target
        relative to the model origin for its associated value. 'target_type' has a
        string representing the type of target (most commonly 'dot') for its associated
        value. ``tgts[i]`` is associated with ``img_targets[i]``
    test_config : dict
        Processing parameters with, at a minimum, a key for 'min_dist', 'max_dist', and
        2 keys for the primary target type in `targets` and `img_targets`. 'min_dist' is
        the minimum distance between two image targets. 'max_dist' is the maximum
        allowable distance between an image target and the projection of a 3D target.
        The primary target type is 'dot' if there are 4+ dots in tgts. Otherwise the
        primary target type is 'kulite'. The keys for the primary target type are
        `target_type` + '_pad' and `target_type` + '_blob_parameters'. The first is the
        padding around the img target center location to use to sub-pixel localize. The
        second is the blob detection parameters for that type of target
    vis_checker : ~upsp.cam_cal_utils.visibility.VisibilityChecker
        Visibility checker object with the relevant BVH and oblique viewing angle
    debug : string, optional
        Name for all debug images (potentially camera name, test an camera name, etc)

    Returns
    -------
    rmat: np.ndarray, shape (3, 3), float
        camera-to-model external calibration rotation matrix
    tvec: np.ndarray, shape (3, 1), float
        camera-to-model external calibration translation vector
    """

    # Check that the inputs are valid
    rmat_camera_tunnel, tvec_camera_tunnel, cameraMatrix, distCoeffs = camera_tunnel_cal
    check_bool = check_external_calibrate_two_stage_inputs(
        img,
        rmat_camera_tunnel,
        tvec_camera_tunnel,
        [cameraMatrix, distCoeffs],
        tgts,
        test_config,
        vis_checker,
        debug,
    )

    tunnel_vals_check_bool = True
    if isinstance(tunnel_vals, dict):
        keys = ['ALPHA', 'BETA', 'PHI', 'STRUTZ']
        if len(set(keys).intersection(tunnel_vals.keys())) != 4:
            tunnel_vals_check_bool = False
            print(
                "tunnel_vals is missing 1 or more keys. tunnel_vals.keys():", 
                tunnel_vals.keys()
            )

        else:
            for key in keys:
                if not isinstance(tunnel_vals[key], float):
                    tunnel_vals_check_bool = False
                    print(
                        "tunnel_vals values should have type float. Instead", 
                        "tunnel_vals[" + str(key) + "] is ",
                        str(type(tunnel_vals[key]))
                    )
                    break
    else:
        tunnel_vals_check_bool = False
        print(
            "tunnel_vals should have type dict. Instead tunnel_vals has type ",
            str(type(tunnel_vals))
        )

    if not (check_bool and tunnel_vals_check_bool):
        raise ValueError(
            "One or more bad inputs were given to external_calibrate_two_stage_from_wtd"
        )

    (
        rmat_init_guess,
        tvec_init_guess,
    ) = camera_tunnel_calibrate.tf_camera_tgts_thru_tunnel(
        camera_tunnel_cal, tunnel_vals, test_config
    )

    rmat, tvec = external_calibrate_two_stage(
        img,
        rmat_init_guess,
        tvec_init_guess,
        camera_tunnel_cal[2:],
        tgts,
        test_config,
        vis_checker,
        debug,
    )

    return rmat, tvec


def external_calibrate_two_stage(
    img,  # Frame specific
    rmat_init_guess,
    tvec_init_guess,  # Datapoint specific
    incal,  # Camera specific
    tgts,
    test_config,
    vis_checker,  # Test/configuration specific
    debug=None,
):
    """Performs external calibration from an inaccurate initial guess

    Runs blob detection to find the img targets, then matches and filters the 3D targets
    to the img targets. Performs a coarse optimization to improve the intial guess. Then
    calls :func:`external_calibrate_one_step` to get the refined optimization

    The initial guess of rmat and tvec should project each target's within ~5 pixels of
    the associated image target

    Parameters
    ----------
    img : np.ndarray, shape (h, w)
        Numpy 2D array of the image
    rmat_init_guess : np.ndarray (3, 3), float
        Initial guess of rotation matrix from camera to object
    tvec_init_guess : np.ndarray (3, 1), float
        Initial guess of translation vector from camera to object
    incal : tuple
        Camera internal calibration.

        - ``cameraMatrix`` (:class:`numpy.ndarray`, shape (3, 3)): The (OpenCV
          formatted) camera matrix for the camera
        - ``distCoeffs`` (:class:`numpy.ndarray`, shape (1, 5): The (OpenCV formatted)
          distortion coefficients for the camera
    tgts : list of dict
        Each target is a dict and has, at a minimum, the keys 'tvec', 'target_type',
        'norm'. 'tvec' has a :class:`numpy.ndarray` (3, 1) representing the position of
        the target relative to the model origin for its associated value. 'norm' has a
        :class:`numpy.ndarray` (3, 1) representing the normal vector of the target
        relative to the model origin for its associated value. 'target_type' has a
        string representing the type of target (most commonly 'dot') for its associated
        value. ``tgts[i]`` is associated with ``img_targets[i]``
    test_config : dict
        Processing parameters with, at a minimum, a key for 'min_dist', 'max_dist', and
        2 keys for the primary target type in targets and img_targets. 'min_dist' is the
        minimum distance between two image targets. 'max_dist' is the maximum allowable
        distance between an image target and the projection of a 3D target. The primary
        target type is 'dot' if there are 4+ dots in tgts. Otherwise the primary target
        type is 'kulite'. The keys for the primary target type are target_type + '_pad'
        and target_type + '_blob_parameters'. The first is the padding around the img
        target center location to use to sub-pixel localize. The second is the blob
        detection parameters for that type of target
    vis_checker : ~upsp.cam_cal_utils.visibility.VisibilityChecker
        Visibility checker object with the relevant BVH and oblique viewing angle
    debug : string or None. Optional, default=None
        Name for all debug images (potentially camera name, test an camera name, etc)

    Returns
    -------
    rmat : np.ndarray, shape (3, 3)
        Valid solution camera-to-model rotation matrix
    tvec : np.ndarray, shape (3, 1)
        Valid solution camera-to-model translation vector
    """
    # Check that the inputs are valid
    check_bool = check_external_calibrate_two_stage_inputs(
        img,
        rmat_init_guess,
        tvec_init_guess,
        incal,
        tgts,
        test_config,
        vis_checker,
        debug,
    )

    # Scale the image
    img = img_utils.scale_image_max_inlier(img)

    # Check the inputs
    if not check_bool:
        raise ValueError(
            "One or more bad inputs were given to external_calibrate_two_stage"
        )

    # Unpack the intrinsics
    cameraMatrix, distCoeffs = (
        incal[0],
        incal[1],
    )

    # Check if there are enough dots to use for targets
    dots_found = True
    primary_tgts = []
    for tgt in tgts:
        if tgt["target_type"] == "dot":
            primary_tgts.append(tgt)

    # If there were enough dots, use the dots blob parameters
    if len(primary_tgts) > 4:
        blob_parameters = test_config["dot_blob_parameters"]

    # If there were not enough dots, collect the kulites and the kulite blob parameters
    else:
        # Set the dot_found flag to False
        dots_found = False
        blob_parameters = test_config["kulite_blob_parameters"]

        primary_tgts = []
        for tgt in tgts:
            if tgt["target_type"] == "kulite":
                primary_tgts.append(tgt)

    # Define the blob detector based on the parameters in the test_config file
    params = cv2.SimpleBlobDetector_Params()
    for blob_param in blob_parameters:
        setattr(params, blob_param[0], blob_param[1])

    # Run the blob detector
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(img)

    # Repackage the blob detector keypoints to function like taret image locations
    init_img_targets = []
    for keypoint in keypoints:
        init_img_target = {
            "target_type": {True: "dot", False: "kulite"}[
                dots_found
            ],  # target_type depends on if enough dots were found
            "center": np.array(keypoint.pt),
        }
        init_img_targets.append(init_img_target)

    # Debug for raw matching information
    if debug_raw_matches:
        tgts_visible = photogrammetry.get_visible_targets(
            rmat_init_guess, tvec_init_guess, primary_tgts, vis_checker
        )

        tgts_match_raw, img_targets_match_raw, num_matches_raw = match_targets(
            rmat_init_guess,
            tvec_init_guess,
            cameraMatrix,
            distCoeffs,
            tgts_visible,
            init_img_targets,
            test_config["max_dist"],
        )

        tgts_filtered_raw, img_targets_filtered_raw = filter_matches(
            rmat_init_guess,
            tvec_init_guess,
            cameraMatrix,
            distCoeffs,
            tgts_match_raw,
            img_targets_match_raw,
            num_matches_raw,
            test_config,
        )

        tgts_subpixel_raw, img_targets_subpixel_raw = subpixel_localize(
            img,
            tgts_filtered_raw,
            img_targets_filtered_raw,
            test_config,
            max_localize_delta=None,
        )
        num_matches = len(img_targets_subpixel_raw)

        tgts_subpixel_raw_temp = [copy.deepcopy(tgt) for tgt in tgts_subpixel_raw]
        for tgt in tgts_subpixel_raw_temp:
            tgt["tvec"] = tgt["tvec"].flatten().tolist()
            tgt["norm"] = tgt["norm"].flatten().tolist()

        rms, max_dist = photogrammetry.reprojection_error(
            rmat_init_guess,
            tvec_init_guess,
            cameraMatrix,
            distCoeffs,
            tgts_subpixel_raw,
            img_targets_subpixel_raw,
        )

        log.info(
            "Raw Num Points: %d RMS: %f Max Error: %f",
            len(tgts_subpixel_raw),
            rms,
            max_dist,
        )

        # Get a list of the targets that are visible, but weren't matched
        visible_but_not_matched = []
        for tgt in tgts_visible:
            tgt_temp = copy.deepcopy(tgt)
            tgt_temp["tvec"] = tgt_temp["tvec"].flatten().tolist()
            tgt_temp["norm"] = tgt_temp["norm"].flatten().tolist()
            if tgt_temp not in tgts_subpixel_raw_temp:
                visible_but_not_matched.append(tgt)

        # Get projected location the visible target centers in the image
        tgt_projs = photogrammetry.project_targets(
            rmat_init_guess,
            tvec_init_guess,
            cameraMatrix,
            distCoeffs,
            tgts_subpixel_raw + visible_but_not_matched,
        )
        proj_pts = np.array([tgt_proj["proj"] for tgt_proj in tgt_projs])

        found_but_not_matched = []
        for tgt_match_raw, img_target_match_raw in zip(
            tgts_match_raw, img_targets_match_raw
        ):
            tgt_temp = copy.deepcopy(tgt_match_raw)
            tgt_temp["tvec"] = tgt_temp["tvec"].flatten().tolist()
            tgt_temp["norm"] = tgt_temp["norm"].flatten().tolist()
            if tgt_temp not in tgts_subpixel_raw_temp:
                found_but_not_matched.append(img_target_match_raw)

        all_img_targets = img_targets_subpixel_raw + found_but_not_matched
        img_centers = [inlier_pt["center"] for inlier_pt in all_img_targets]
        img_centers = np.array(img_centers)

        # Get the debug name. If debug was given, use it otherwise don't
        debug_name = str(debug) + "_raw" if debug is not None else "raw"

        # Output a debug image of the projected locations and image target center locations
        visualization.show_projection_matching(
            img,
            proj_pts,
            img_centers,
            num_matches=num_matches,
            name=debug_name,
            scale=2.0,
        )

    # Check that enough blobs were found
    if len(keypoints) < 4:
        raise ValueError(
            "Less than 4 blobs were found in external_calibrate_two_stage."
        )

    # Do the coarse external calibration
    coarse_outputs = external_calibrate(
        img,
        rmat_init_guess,
        tvec_init_guess,
        cameraMatrix,
        distCoeffs,
        primary_tgts,
        init_img_targets,
        vis_checker,
        test_config,
        isMatched=False,
        max_localize_delta=None,
        reprojectionError=test_config["max_dist"],
    )

    # Unpack the output variables
    (
        rmat_coarse,
        tvec_coarse,
        tgts_inliers_coarse,
        img_target_inliers_coarse,
    ) = coarse_outputs

    if debug_coarse_optimization:
        rms, max_dist = photogrammetry.reprojection_error(
            rmat_coarse,
            tvec_coarse,
            cameraMatrix,
            distCoeffs,
            tgts_inliers_coarse,
            img_target_inliers_coarse,
        )

        log.info(
            "Coarse Num Points: %d RMS: %f Max Error: %f",
            len(tgts_inliers_coarse),
            rms,
            max_dist,
        )

        # Get projected location the visible targets in the image
        tgt_projs = photogrammetry.project_targets(
            rmat_coarse, tvec_coarse, cameraMatrix, distCoeffs, tgts_inliers_coarse
        )

        proj_pts = np.array([tgt_proj["proj"] for tgt_proj in tgt_projs])
        img_pts = np.array(
            [inlier_pt["center"] for inlier_pt in img_target_inliers_coarse]
        )

        # Get the debug name. If debug was given, use it otherwise don't
        debug_name = str(debug) + "_coarse" if debug is not None else "coarse"

        visualization.show_projection_matching(
            img, proj_pts, img_pts, name=debug_name, scale=2
        )

    # Call the one step function for the refined optimization
    rmat_refined, tvec_refined = external_calibrate_one_step(
        img,
        rmat_coarse,
        tvec_coarse,
        incal,
        primary_tgts,
        test_config,
        vis_checker,
        debug,
    )

    return rmat_refined, tvec_refined


def external_calibrate_one_step(
    img,  # Frame specific
    rmat_coarse,
    tvec_coarse,  # Datapoint specific
    incal,  # Camera specific
    tgts,
    test_config,
    vis_checker,  # Test/configuration specific
    debug=None,
):
    """Performs external calibration from a seim-accurate coarse guess

    The coarse guess of `rmat` and `tvec` should project each target's within 1 pixel of
    the associated image target

    Parameters
    ----------
    img : np.ndarray, shape (h, w)
        Numpy 2D array of the image
    rmat_coarse : np.ndarray, shape (3, 3), float
        Coarsely refined rotation matrix from camera to object
    tvec_coarse : np.ndarray, shape (3, 1), float
        Coarsely refined translation vector from camera to object
    incal : tuple
        Camera internal calibration.

        - ``cameraMatrix`` (:class:`numpy.ndarray`, shape (3, 3)): The (OpenCV
          formatted) camera matrix for the camera
        - ``distCoeffs`` (:class:`numpy.ndarray`, shape (1, 5): The (OpenCV formatted)
          distortion coefficients for the camera
    tgts : list of dict
        Each target is a dict and has, at a minimum, the keys 'tvec', 'target_type',
        'norm'. 'tvec' has a :class:`numpy.ndarray` (3, 1) representing the position of
        the target relative to the model origin for its associated value. 'norm' has a
        :class:`numpy.ndarray` (3, 1) representing the normal vector of the target
        relative to the model origin for its associated value. 'target_type' has a
        string representing the type of target (most commonly 'dot') for its associated
        value. ``tgts[i]`` is associated with ``img_targets[i]``
    test_config : dict
        Processing parameters with, at a minimum, a key for 'min_dist', 'max_dist', and
        2 keys for the primary target type in targets and img_targets. 'min_dist' is the
        minimum distance between two image targets. 'max_dist' is the maximum allowable
        distance between an image target and the projection of a 3D target. The primary
        target type is 'dot' if there are 4+ dots in tgts. Otherwise the primary target
        type is 'kulite'. The keys for the primary target type are target_type + '_pad'
        and target_type + '_blob_parameters'. The first is the padding around the img
        target center location to use to sub-pixel localize. The second is the blob
        detection parameters for that type of target
    vis_checker : ~upsp.cam_cal_utils.visibility.VisibilityChecker
        Visibility checker object with the relevant BVH and oblique viewing angle
    debug : string or None. Optional, default=None
        Name for all debug images (potentially camera name, test an camera name, etc)

    Returns
    -------
    rmat: np.ndarray, shape (3, 3), float
        camera-to-model external calibration rotation matrix
    tvec: np.ndarray, shape (3, 1), float
        camera-to-model external calibration translation vector
    """
    # Scale the image
    img = img_utils.scale_image_max_inlier(img)

    # Unpack the intrinsics
    cameraMatrix, distCoeffs = (
        incal[0],
        incal[1],
    )

    # Get the visible targets
    visible_tgts = photogrammetry.get_visible_targets(
        rmat_coarse, tvec_coarse, tgts, vis_checker
    )

    # Get the projections of the visible targets
    tgt_projs = photogrammetry.project_targets(
        rmat_coarse, tvec_coarse, cameraMatrix, distCoeffs, visible_tgts
    )

    # Package the target projections into img targets
    img_targets = []
    for tgt_proj in tgt_projs:
        img_target = {
            "target_type": tgt_proj["target_type"],
            "center": tgt_proj["proj"],
        }
        img_targets.append(img_target)

    # Debuf for refined matches
    if debug_refined_matches:
        # Filter the matched targets
        tgts_filtered, img_targets_filtered = filter_matches(
            rmat_coarse,
            tvec_coarse,
            cameraMatrix,
            distCoeffs,
            visible_tgts,
            img_targets,
            len(visible_tgts),
            test_config,
        )

        # Subpixel localize the image targets
        tgts_subpixel, img_targets_subpixel = subpixel_localize(
            img,
            tgts_filtered,
            img_targets_filtered,
            test_config,
            max_localize_delta=None,
        )
        num_matches = len(img_targets_subpixel)

        tgts_subpixel_temp = [copy.deepcopy(tgt) for tgt in tgts_subpixel]
        for tgt in tgts_subpixel_temp:
            tgt["tvec"] = tgt["tvec"].flatten().tolist()
            tgt["norm"] = tgt["norm"].flatten().tolist()

        # Get a list of the targets that are visible, but weren't matched
        visible_but_not_matched = []
        for tgt in visible_tgts:
            tgt_temp = copy.deepcopy(tgt)
            tgt_temp["tvec"] = tgt_temp["tvec"].flatten().tolist()
            tgt_temp["norm"] = tgt_temp["norm"].flatten().tolist()
            if tgt_temp not in tgts_subpixel_temp:
                visible_but_not_matched.append(tgt)

        # Get projected location the visible target centers in the image
        tgt_projs = photogrammetry.project_targets(
            rmat_coarse,
            tvec_coarse,
            cameraMatrix,
            distCoeffs,
            tgts_subpixel + visible_but_not_matched,
        )
        proj_pts = np.array([tgt_proj["proj"] for tgt_proj in tgt_projs])

        img_centers = np.array(
            [img_target["center"] for img_target in img_targets_subpixel]
        )

        rms, max_dist = photogrammetry.reprojection_error(
            rmat_coarse,
            tvec_coarse,
            cameraMatrix,
            distCoeffs,
            tgts_subpixel,
            img_targets_subpixel,
        )

        # Get the debug name. If debug was given, use it otherwise don't
        debug_name = str(debug) + "_refined" if debug is not None else "refined"

        # Output a debug image of the projected locations and image target center locations
        visualization.show_projection_matching(
            img,
            proj_pts,
            img_centers,
            num_matches=num_matches,
            name=debug_name,
            scale=2.0,
        )

    # Run the refined external calibration
    refined_outputs = external_calibrate(
        img,
        rmat_coarse,
        tvec_coarse,
        cameraMatrix,
        distCoeffs,
        visible_tgts,
        img_targets,
        vis_checker,
        test_config,
        isMatched=True,
        max_localize_delta=None,
        reprojectionError=test_config["max_dist"],
    )

    # Unpack the refined results
    (
        rmat_refined,
        tvec_refined,
        tgts_inliers_refined,
        img_target_inliers_refined,
    ) = refined_outputs

    # Debug for refined optimization
    if debug_refined_optimization:
        rms, max_dist = photogrammetry.reprojection_error(
            rmat_refined,
            tvec_refined,
            cameraMatrix,
            distCoeffs,
            tgts_inliers_refined,
            img_target_inliers_refined,
        )

        log.info(
            "Refined Num Points: %d RMS: %f Max Error: %f",
            len(tgts_inliers_refined),
            rms,
            max_dist,
        )

        # Get projected location the visible targets in the image
        tgt_projs = photogrammetry.project_targets(
            rmat_refined, tvec_refined, cameraMatrix, distCoeffs, tgts_inliers_refined
        )

        # Plot the kulites in blue and dots in red
        plt.imshow(img, cmap="gray")
        for tgt_proj in tgt_projs:
            plt.scatter(
                [tgt_proj["proj"][0]],
                [tgt_proj["proj"][1]],
                c={"kulite": "b", "dot": "r"}[tgt_proj["target_type"]],
                marker="o",
                s=0.05,
            )

        # Get the debug name. If debug was given, use it otherwise don't
        debug_name = (
            str(debug) + "_refined_optimization.png"
            if debug != None
            else "refined_optimization.png"
        )

        plt.savefig(debug_name, dpi=400)
        plt.close()

    # Secondary debug to project all visible targets, not just the targets used in the
    #   optimization
    if debug_visible_projections:
        # Get visible targets
        visible_tgts = photogrammetry.get_visible_targets(
            rmat_refined, tvec_refined, tgts, vis_checker
        )

        # Get projected location the visible targets in the image
        tgt_projs = photogrammetry.project_targets(
            rmat_refined, tvec_refined, cameraMatrix, distCoeffs, visible_tgts
        )

        # Plot the kulites in blue and dots in red
        plt.imshow(img, cmap="gray")
        for i, tgt_proj in enumerate(tgt_projs):
            plt.scatter(
                [tgt_proj["proj"][0]],
                [tgt_proj["proj"][1]],
                c={"kulite": "b", "dot": "r"}[tgt_proj["target_type"]],
                marker="o",
                s=0.05,
            )

        # Get the debug name. If debug was given, use it otherwise don't
        debug_name = (
            str(debug) + "_visible_projection.png"
            if debug is not None
            else "visible_projection.png"
        )

        plt.savefig(debug_name, dpi=400)
        plt.close()

    return rmat_refined, tvec_refined


def external_calibrate_RANSAC(
    incal, tgts, img_targets, vis_checker, max_iter=0.999, max_dist=8, match_thresh=0.80
):
    """Use RANSAC to find an external calibration with brute force

    To find an external calibration, select at random 3 targets and 3 img targets. Solve
    the associated P3P problem to get the external calibration(s). For each of the
    external calibrations (P3P can yield up to 4), find the visible targets, project
    them, and match them to the image targets. If there is sufficient consensus amoung
    the matches (i.e. the randomly selected targets and image targets yields a solution
    where many other unselected targets project to a location close to an image target),
    return that external calibration. If there is not sufficient consensus, repeat this
    process.

    Parameters
    ----------
    incal : tuple
        Camera internal calibration.

        - ``cameraMatrix`` (:class:`numpy.ndarray`, shape (3, 3)): The (OpenCV
          formatted) camera matrix for the camera
        - ``distCoeffs`` (:class:`numpy.ndarray`, shape (1, 5): The (OpenCV formatted)
          distortion coefficients for the camera
    tgts : list of dict
        Each target is a dict with (at a minimum) 'tvec', 'norm', and 'target_type'
        attributes. The 'tvec' attribute gives the target's location and the 'norm'
        attribute gives the target's normal vector. 'target_type' is a string denoting
        the type of target (most commonly 'dot' or 'kulite')
    img_targets : list of dict
        Each img_target is a dict with (at a minimum) a 'center' attribute. The 'center'
        attribute gives the pixel position of the target's center in the image.
        ``img_targets[i]`` is associated with ``tgts[i]``
    vis_checker : ~upsp.cam_cal_utils.visibility.VisibilityChecker
        VisibilityChecker object with the relevant BVH and oblique viewing angle
    max_iter : int or float, optional
        If int, it represents the maximum number of iterations for RANSAC. If float must
        be greater than 0 but less than 1. Float means it represents the number of
        estimate number of iterations in order to have that probability of finding a
        solution should one exist (0.999 means there's a 99.9% chance of finding a
        solution should one exist)
    max_dist : int or float, optional
        Maximum matching distance between image targets and projected target location
    match_thresh : int or float, optional
        If int, this is the number of matches that must be found to deem it a consensus.
        If float, must be greater than 0 but less than 1. If float, this is the
        proportion of tgts or img_targets (whichever is lower) that must be matched.
        I.e. If match_thresh is 0.8 and there are 50 target and 20 img_targets, there
        must be 16+ matches for it to be deemed a consensus.

    Returns
    -------
    rmat : np.ndarray, shape (3, 3)
        Rotation matrix. May be ``None`` if the maximum number of iterations is met
        without finding a valid solution.
    tvec : np.ndarray, shape (3, 1)
        Translation vector. May be ``None`` if the maximum number of iterations is met
        without finding a valid solution.
    """
    # Seed the random number generator at the start of the routine
    np.random.seed(0)

    cameraMatrix, distCoeffs = incal

    # 'Fake' visibility checker object with no grid which is used to speed up computation
    vis_checker_nogrid = visibility.VisibilityChecker(
        None,
        oblique_angle=vis_checker.oblique_angle,
        epsilon=vis_checker.epsilon,
        debug=False,
        debug_nogrid=True,
    )

    # If match_thresh is between 0 and 1, get the number of matches required
    if 0 < match_thresh <= 1:
        # Take the minimum of the proportion of tgts and img_targets
        match_thresh = min(
            np.floor(len(tgts) * match_thresh).astype(np.int32),
            np.floor(len(img_targets) * match_thresh).astype(np.int32),
        )

        # match_thresh must be at least 4 for any consensus (3 are used for P3P, so at
        #   least 1 additional is required for the smallest consensus)
        match_thresh = int(max(match_thresh, 4))

    # If max_iter is a float of 1.0, user wanted probability of 1, so repeat infinite times
    if isinstance(max_iter, float) and (max_iter == 1.0):
        max_iter = np.inf

    # If max_iter is None, find the value of max_iter such that there is a 99% chance of
    #   find the solution if it exists
    if 0 < max_iter < 1:
        # Estimated number of visible targets based on oblique viewing angle and
        #   assuming targets are equally distributed
        est_vis_targs = len(tgts) * vis_checker.oblique_angle / 180

        # Expected number of iterations to find 3 visible targets
        exp_3_vis_num = len(tgts) * (len(tgts) - 1) * (len(tgts) - 2)
        exp_3_vis_den = est_vis_targs * (est_vis_targs - 1) * (est_vis_targs - 2)
        exp_3_vis = exp_3_vis_num / exp_3_vis_den

        # Expected number of iterations to choose all 3 correct image target
        exp_img = len(img_targets) * (len(img_targets) - 1) * (len(img_targets) - 2)

        # Probability of find a solution per iteration
        P = 1 / (exp_3_vis * exp_img)

        # Binomial equations is: b(x, n, p) = [n! / (x! (n - x)!)] * P^x * (1-P)^(n-x)
        # Using that equation, simply knowing we want the probabily of x=0 to be
        # (1-max_iter)
        # (1-max_iter) = (1-P)^n => n = log(1-max_iter) / log(1-P)
        # Where n is max_iter
        max_iter = np.rint(np.log(1 - max_iter) / np.log(1 - P)).astype(np.int32)

    # Get a list of indicies of the tgts
    tgt_tvecs = np.array([tgt["tvec"] for tgt in tgts])
    tgt_tvecs_idxs = np.arange(len(tgt_tvecs))

    # Get a list of indicies of the image targets
    img_target_projs = np.array([img_tgt["center"] for img_tgt in img_targets])
    img_target_projs_idxs = np.arange(len(img_target_projs))

    print(
        "tgt_tvecs:", str(len(tgt_tvecs)) + ", img_target_projs:", len(img_target_projs)
    )
    print("match_thresh:", str(match_thresh) + ", max_iter:", max_iter)

    # RANSAC Operation:
    #   Select at random 3 tgts and 3 img targats. Solve the corresponding P3P problem,
    #   and check for consensus with the other target projections and image targets.
    #   If there is good consensus, leave the loop
    # We do not cache random guesses because there are likely hundreds of millions of
    #   possible guesses (depending on length of tgts and img_targets), and it takes on
    #   the order of a couple hundred thousand to find a solution (again depending on
    #   length of tgts and img_targets). So randomly generating the same guess twice
    #   will be relatively rare, and it is faster to generate the same guess twice
    #   rather than have to cache each guess and check each guess against the cache
    max_num_matches = 0
    max_rmat, max_tvec = None, None
    n = 0
    while n < max_iter:
        # Select 3 random targets
        rand_tgts_idxs = np.random.choice(tgt_tvecs_idxs, 3, replace=False)
        rand_tgts = tgt_tvecs[rand_tgts_idxs]

        # Select 3 random image targets
        rand_img_targets_idxs = np.random.choice(
            img_target_projs_idxs, 3, replace=False
        )
        rand_img_targets = img_target_projs[rand_img_targets_idxs]

        # Solve the associated P3P problem
        retval, rvecs, tvecs = cv2.solveP3P(
            rand_tgts,
            rand_img_targets,
            cameraMatrix,
            distCoeffs,
            flags=cv2.SOLVEPNP_P3P,
        )

        # Check each potential solution from the P3P solutions
        for i in range(retval):
            # Get the rmat and tvec
            rvec, tvec = rvecs[i], tvecs[i]
            rmat, _ = cv2.Rodrigues(rvec)

            # Get the visible targets
            tgts_vis = photogrammetry.get_visible_targets(
                rmat, tvec, tgts, vis_checker_nogrid
            )

            # If there are too few visible targets, continue
            if len(tgts_vis) < match_thresh:
                continue

            # Match the visible targets
            tgts_matched, matching_points, num_matches = match_targets(
                rmat,
                tvec,
                cameraMatrix,
                distCoeffs,
                tgts_vis,
                img_targets,
                max_dist=max_dist,
            )

            # Filter the visible targets
            tgts_matched, matching_points, num_matches = filter_one2one(
                tgts_matched, matching_points, num_matches
            )

            if num_matches > max_num_matches:
                max_num_matches = num_matches
                print("\r\tmax number of matches found:", num_matches, "\t\t\t", end="")
                max_rmat, max_tvec = rmat, tvec

            # If there are enough matches, repeat checking process with vis_checker
            #   to ensure this solution is valid
            if num_matches >= match_thresh:
                # Get the visible targets
                tgts_vis = photogrammetry.get_visible_targets(
                    rmat, tvec, tgts, vis_checker
                )

                # Match the visible targets
                tgts_matched, matching_points, num_matches = match_targets(
                    rmat,
                    tvec,
                    cameraMatrix,
                    distCoeffs,
                    tgts_vis,
                    img_targets,
                    max_dist=max_dist,
                )

                # Filter the visible targets
                tgts_matched, matching_points, num_matches = filter_one2one(
                    tgts_matched, matching_points, num_matches
                )

                # Package the target tvecs and image points
                tgt_tvecs = np.array(
                    [tgts_matched[k]["tvec"] for k in range(num_matches)]
                )
                img_centers = np.array(
                    [matching_points[k]["center"] for k in range(num_matches)]
                )

                # SolvePnP but with all targets
                retval, rvec_opt, tvec_opt = cv2.solvePnP(
                    tgt_tvecs,
                    img_centers,
                    cameraMatrix,
                    distCoeffs,
                    copy.copy(rvec),
                    copy.copy(tvec),
                    useExtrinsicGuess=True,
                )
                rmat_opt, _ = cv2.Rodrigues(rvec_opt)

                # If there are still enough matches, return the solution found
                #   Otherwise just go back into the loop
                if num_matches >= match_thresh:
                    print()
                    return rmat_opt, tvec_opt

        n += 1
    print()

    warn_str = str(
        "Maximum number of iterations exceeded in "
        + "external_calibrate_RANSAC. Please check that inputs are correct. "
        + "If they are, increase max_iter, increase max_dist, or decrease "
        + "match_thresh. Returning the best found solution"
    )

    warnings.warn(warn_str, RuntimeWarning)
    return max_rmat, max_tvec
