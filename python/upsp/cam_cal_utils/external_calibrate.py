import cv2
import logging
import matplotlib.pyplot as plt
import numpy as np
import copy
import os
import sys
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)

target_loc_methods = os.path.join(parent_dir, 'target_localization')
sys.path.append(target_loc_methods)

import gaussian_fitting_methods
# Define the gaussian fit function
gauss_fit = gaussian_fitting_methods.gauss_fitter_func('super')

utils = os.path.join(parent_dir, 'cam_cal_utils')
sys.path.append(utils)

import visualization
import visibility
import photogrammetry
import img_utils

log = logging.getLogger(__name__)

# Debugs
debug_raw_matches = True
debug_coarse_optimization = True
debug_refined_matches = True
debug_visible_projections = True
debug_refined_optimization = True
debug_show_localizations = False

#---------------------------------------------------------------------------------------
# Other Functions

 
def compare_poses(pose0, pose1, more_info=False):
    """Returns the angle and distance in which the two poses differ
    
    Any two rotation matrices are related by a single rotation of theta about a given axis
    This function returns the angle between the two poses, as well as the distance
    formatted as [theta, dist] where theta is in degrees. If more_info is True, 
    [theta, axis, tvec_rel] is returned where axis is the axis of rotation.
    
    Parameters
    ----------
    pose0 :
    pose1 :
    more_info : 
    
    Returns
    ----------
    
    """
    # Unpack the rmats and tvecs
    rmat0, tvec0 = pose0
    rmat1, tvec1 = pose1

    # Get true relative transformations
    rmat_rel = np.matmul(rmat1, rmat0.T)
    tvec_rel = tvec1 - np.matmul(rmat_rel, tvec0)
    
    # Get the axis and theta for rmat_rel
    rvec_rel, _ = cv2.Rodrigues(rmat_rel)
    
    # OpenCV's Rodrigues vector is a compact representation where theta is the magnitude
    #   of the vector (convert from radians to degrees)
    theta = np.linalg.norm(rvec_rel)
    theta = np.rad2deg(theta)

    # If more_info was not requested, return the angle and distance
    if not more_info:
        return [theta, np.linalg.norm(tvec_rel)]
    
    # If more_info was requested, return the angle, axis, and full tvec
    else:
        # In the compact OpenCV representation, the axis is the Rodrigues vector divided
        #   by the angle
        axis = rvec_rel / theta
        return [theta, axis, tvec_rel]
                    
#---------------------------------------------------------------------------------------
# Calibration Functions


def subpixel_localize_robust(img, tgts, img_targets, test_config,
                             max_localize_delta=None):
    """Find the sub-pixel localized position of the targets in a robust manner

    Find the location of the target centers with sub-pixel accuracy. This method filters
    common bad localization solutions. I.e Localized position is too far initial guess
    to make sense, invalid optimizer solution, outside crop or outside image

    Parameters
    ----------
    img
    tgts
    img_targets
    test_config
    max_localize_delta
    
    Returns
    ----------
    
    """
    if max_localize_delta is not None:
        filter_dist = max_localize_delta

    out_of_bounds = set()
    img_targets_refined = []
    for i, img_target in enumerate(img_targets):
        # Integer of center pixel
        center_pixel = np.rint((img_target['center'][0], img_target['center'][1])).astype(np.int32)
        
        target_pad = test_config[img_target['target_type'] + '_pad']

        # cropped region around target (x1, y1), (x2, y2)
        bbox = [[center_pixel[0] - target_pad, center_pixel[1] - target_pad],
                [center_pixel[0] + target_pad + 1, center_pixel[1] + target_pad + 1]]

        # If bbox goes out of bounds of the image, ignore it
        if ((bbox[0][0] < 0) or (bbox[0][1] < 0) or (bbox[1][0] >= img.shape[1]) or (bbox[1][1] >= img.shape[0])):
            out_of_bounds.add(i)
            img_targets_refined.append(
                    {'target_type' : img_target['target_type'], 'center' : (-1, -1)})            
            continue

        # Cropped image around target
        img_cropped = img[bbox[0][1] : bbox[1][1], bbox[0][0] : bbox[1][0]]
        
        fake_keypoint = cv2.KeyPoint(img_target['center'][0], img_target['center'][1], _size=1)
        
        gauss_center = gauss_fit(img_cropped, target_type=img_target['target_type'],
                                    keypoint=fake_keypoint, img_offset=bbox[0])[0]
        
        img_target_refined = {'target_type' : img_target['target_type'], 
                              'center' : gauss_center}

        dist = np.linalg.norm([gauss_center[0] - img_target_refined['center'][0], 
                               gauss_center[1] - img_target_refined['center'][1]])
        
        # A distance of target_pad - 1 would imply the center is on a pixel in the edge
        #   of the crop. And since the taret is larger than 1 pixel, by definition this
        #   is a bad localization. Not even to mention it is likely bad since it is so
        #   far off from the expected position. We do target_pad - 2 to add a margin of
        #   safety
        if max_localize_delta is None:
            filter_dist = target_pad - 2
        
        if (dist > filter_dist) or (gauss_center[0] == -1):
            out_of_bounds.add(i)
            img_targets_refined.append(
                    {'target_type' : img_target['target_type'], 'center' : (-1, -1)}) 
        
        if debug_show_localizations:
            plt.imshow(img_cropped, cmap='gray')
            plt.scatter([gauss_center[0] - bbox[0][0]], [gauss_center[1] - bbox[0][1]], c='g', s=2)
            plt.scatter([img_target['center'][0] - bbox[0][0]], [img_target['center'][1] - bbox[0][1]], c='r', s=2)
            plt.savefig(str(i).rjust(3, '0') + '_' + str(np.round(dist, 3)) + '_' 
                        + str(center_pixel[0]) + '_' + str(center_pixel[1]) + '.png')
            plt.close()
            
        img_targets_refined.append(img_target_refined)
    
    # Remove the targets that had a bad localization
    tgts_temp = []
    img_targets_temp = []
    for i, (tgt, img_target) in enumerate(zip(tgts, img_targets_refined)):
        if i not in out_of_bounds:
            tgts_temp.append(tgt)
            img_targets_temp.append(img_target)
    
    return tgts_temp, img_targets_temp


# TODO: make this return a num_matches, all targets, and all unique img_targets just
#   like to other targets functions
def filter_partially_occluded(rmat, tvec, focal_length, tgts, img_targets, vis_checker, test_config):
    """Checks if corners of crop used for localization are occluded.
    
    If the corners of the crop used for sub-pixel localization jumps surfaces, the
    target, then the target is likely at least partially occluded.
    
    To get 3D positions of corners of the cropped image, start at the target tvec.
    Approximate the physical distance (in inches) of test_config['*_pad'] (which is
    given in pixels) using the focal length and distance from camera to model. Take
    steps along the focal plane to get to the approximate corner locations in 3D.

    With those locations, ensure the targets are not inside the model (since the model
    is curved the planar focal plane may put the corners slightly inside the model. Then
    check for occlusions. If the corner is occluded, the target is partially occluded.
    If none of the corners are occluded, the target is likely not occluded (but is still
    potentially partially occluded).

    Parameters
    ----------
    rmat
    tvec
    focal_length
    tgts
    img_targets
    vis_checker
    test_config
    
    Returns
    ----------
    
    """
    # Get the direction for the u and v of pixel space
    rmat_inv, tvec_inv = photogrammetry.invTransform(rmat, tvec)    
    tvec = tvec.ravel()

    tgts_filtered = []
    img_targets_filtered = []
    for tgt, img_target in zip(tgts, img_targets):
        # Get the scale factor
        obj_dist = np.linalg.norm(tvec - tgt['tvec'])

        # For the scale factor, we use the similar triangles of the object distance by
        #   actual distance (in) vs focal length by pixel distance (px)
        # The scale factor is the object distance divided by the focal length, times the
        #   pixel distance. We add one to the pad distance for a margin of safety
        step_sz = obj_dist * (test_config[tgt['target_type'] + '_pad']  + 1) / focal_length

        # Get the step vector for u and v for the corners
        # Get the u and v steps
        uv = step_sz * rmat_inv[:, 0:2]
        u = uv[:, 0]
        v = uv[:, 1]

        # Package the corners. We have 1 corner for steps in the given directions
        corners = []
        for x, y in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
            corners.append({'tvec' : tgt['tvec'] + x * u + y * v,
                            'norm' : tgt['norm']})

        # We need to check if the corners are visible. It is a relatively safe assumption
        #   that since the targets are visible (and pass the back-face culling) that
        #   the corners pass the back face culling as well. So we only check for true
        #   occlusions

        # Get the occlusions. For each corner this has a boolean and an occlusion position
        occlusions = photogrammetry.get_occlusions_targets(rmat, tvec, corners, vis_checker)

        # Check that all corners are visible
        are_all_corners_visible = True
        for corner, occlusion in zip(corners, occlusions):
            # If there was no occlusion, continue
            if not occlusion[0]:
                continue

            # If there was an occlusion, check distance from the occlusion to the corner
            # If that distance is less than sqrt(2) * step_sz (since we step in both
            #   x and y) it is possible that the corner is inside the model so it is
            #   unfairly occluded.
            # If the distance is greater this corner is just occluded, no questions
            dist = np.linalg.norm(occlusion[1] - corner['tvec'])
            if (dist > step_sz * np.sqrt(2)):
                are_all_corners_visible = False
                break

            # To fairly check this corner, bump it to the occluded location
            bumped_corner = copy.copy(corner)
            bumped_corner['tvec'] = occlusion[1]

            # Check this bumped corner for occlusion
            bumped_occlusion = photogrammetry.get_occlusions_targets(
                rmat, tvec, [bumped_corner], vis_checker)[0]

            # If there is still an occlusion, this corner is likely occluded
            # There is a chance that the distance between the bumped occlusion and 
            #   the original corner is still less than step_sz * np.sqrt(2). However
            #   the frequency seems small and not worthy of further checks.
            # This is a potential TODO to set up as a while loop
            if bumped_occlusion[0]:
                are_all_corners_visible = False
                break
            
            # Otherwise, this corner is fine and we can move onto the next corner

        if are_all_corners_visible:
            tgts_filtered.append(tgt)
            img_targets_filtered.append(img_target)

    return tgts_filtered, img_targets_filtered


def filter_min_dist(tgts, matching_points, num_matches, min_dist=8, kwd='center'):
    """Filters target projections that are too close together in the image

    If the image locations of any two targets are below the min_dist threshold, remove
    both targets from the set of matched targets.

    Parameters
    ----------
    tgts
    matching_points
    num_matches
    min_dist
    kwd
    
    Returns
    ----------
    
    """
    supressed = set()
    for i in range(num_matches):
        for j in range(i+1, num_matches):
            if (i in supressed) and (j in supressed):
                continue

            # Calculate the distance between point i and j
            dist = np.linalg.norm([matching_points[i][kwd][0] - matching_points[j][kwd][0], 
                                   matching_points[i][kwd][1] - matching_points[j][kwd][1]]) 

            # If the distance is below the given threshold, add both i and j to the list
            #   of supressed points
            if dist < min_dist:
                supressed.add(i)
                supressed.add(j)
                continue
    
    tgts_temp = []
    matching_points_temp = []
    for i in range(num_matches):
        if i in supressed:
            continue
        tgts_temp.append(tgts[i])
        matching_points_temp.append(matching_points[i])
    
    num_matches_temp = len(tgts_temp)
    for tgt in tgts:
        if tgt not in tgts_temp:
            tgts_temp.append(tgt)
    for mp in matching_points:
        if mp not in matching_points_temp:
            matching_points_temp.append(mp)

    return tgts_temp, matching_points_temp, num_matches_temp


def filter_bifilter(rmat, tvec, cameraMatrix, distCoeffs, tgts, matching_points, num_matches, max_dist):
    """Consistency check to filer potential mismatches

    Check that only 1 img target is within a radius of max_dist from every projected 
    target location. If more than 1 img target is near a target, do not use that target
    
    Check that only 1 projected target location is within a radius of max_dist from
    every img target. If more than 1 projected target location is near an img target,
    do not use that img target

    Parameters
    ----------
    rmat
    tvec
    cameraMatrix
    distCoeffs
    tgts
    matching_points
    num_matches
    max_dist
    
    Returns
    ----------

    """

    # Project the points into the image
    tgt_projs = photogrammetry.project_targets(rmat, tvec, cameraMatrix, distCoeffs, tgts)

    # Check that all targets have at most 1 img target nearby
    tgts_matched_temp, matching_points_temp = [], []
    num_matches_temp = 0
    for i in range(num_matches):
        tgt_proj = tgt_projs[i]
        
        bifilter_key = True
        for j, mp in enumerate(matching_points):
            if i == j:
                continue

            dist = np.linalg.norm(np.array(tgt_proj['proj']) - np.array(mp['center']))
            
            # If the distance is less than max_dist, this target fails the bifilter
            if dist < max_dist:
                bifilter_key = False
                break
        
        if bifilter_key:
            tgts_matched_temp.append(tgts[i])
            matching_points_temp.append(matching_points[i])
            num_matches_temp += 1

    for tgt in tgts:
        if tgt not in tgts_matched_temp:
            tgts_matched_temp.append(tgt)
    for mp in matching_points:
        if mp not in matching_points_temp:
            matching_points_temp.append(mp)

    # Project the points into the image
    tgt_projs = photogrammetry.project_targets(
            rmat, tvec, cameraMatrix, distCoeffs, tgts_matched_temp)

    # Check that all img targets have at most 1 target nearby
    tgts_matched_bifilter, matching_points_bifilter = [], []
    num_matches_bifilter = 0
    for i in range(num_matches_temp):
        mp = matching_points_temp[i]
        
        bifilter_key = True
        for j, tgt_proj in enumerate(tgt_projs):
            if i == j:
                continue

            dist = np.linalg.norm(np.array(mp['center']) - np.array(tgt_proj['proj']))
            
            # If the distance is less than max_dist, this target fails the bifilter
            if dist < max_dist:
                bifilter_key = False
                break
        
        if bifilter_key:
            tgts_matched_bifilter.append(tgts_matched_temp[i])
            matching_points_bifilter.append(matching_points_temp[i])
            num_matches_bifilter += 1

    for tgt in tgts:
        if tgt not in tgts_matched_bifilter:
            tgts_matched_bifilter.append(tgt)
    for mp in matching_points:
        if mp not in matching_points_bifilter:
            matching_points_bifilter.append(mp)

    return tgts_matched_bifilter, matching_points_bifilter, num_matches_bifilter


def filter_one2one(tgts, matching_points, num_matches):
    """Ensures that each target is matched with at most 1 img target, and that each img
    target is matched with at most 1 target. Remove anything that is not 1:1 as stated
    
    Parameters
    ----------
    tgts
    matching_points
    num_matches
    
    Returns
    ----------
    
    """

    # Check that each img_target is used at most once
    tgts_filtered_temp = []
    matching_points_temp = []
    num_matches_temp = 0
    for i in range(num_matches):
        # Unpack the target and match
        tgt, match = tgts[i], matching_points[i]

        # Create a copy of the all matching points minus the current
        matching_points_subset = copy.deepcopy(matching_points[:num_matches])
        del matching_points_subset[i]

        # If match is not in the list, there is a 1:1 matching and it can be included
        if match not in matching_points_subset:
            tgts_filtered_temp.append(tgt)
            matching_points_temp.append(match)
            num_matches_temp += 1

    matching_points_subset = None # deleteme

    # Check that each img_target is used at most once
    tgts_filtered_one2one = []
    matching_points_one2one = []
    num_matches_one2one = 0
    for i in range(num_matches_temp):
        # Unpack the target and match
        tgt, match = tgts_filtered_temp[i], matching_points_temp[i]

        # Create a copy of the all matching points minus the current
        tgts_subset = copy.deepcopy(tgts_filtered_temp[:num_matches_temp])
        del tgts_subset[i]

        # If match is not in the list, there is a 1:1 matching and it can be included
        if tgt not in tgts_subset:
            tgts_filtered_one2one.append(tgt)
            matching_points_one2one.append(match)
            num_matches_one2one += 1

    for tgt in tgts:
        if tgt not in tgts_filtered_one2one:
            tgts_filtered_one2one.append(tgt)
    for mp in matching_points:
        if mp not in matching_points_one2one:
            matching_points_one2one.append(mp)

    return tgts_filtered_one2one, matching_points_one2one, num_matches_one2one


def filter_max_dist(rmat, tvec, cameraMatrix, distCoeffs, tgts_matched, matching_points, num_matches, max_dist):
    """Filter matches where the projected target and imag target are too far.

    Any match where the distance between the projected target pixel position and the
    img target center is greater than max_dist is removed.
    
    Parameters
    ----------
    rmat
    tvec
    cameraMatrix
    distCoeffs
    tgts_matched
    matching_points
    num_matches
    max_dist
    
    Returns
    ----------
    
    """
    # Project the points into the image
    tgt_projs = photogrammetry.project_targets(
            rmat, tvec, cameraMatrix, distCoeffs, tgts_matched
        )

    tgts_matched_temp, matching_points_temp = [], []
    for i in range(num_matches):
        tgt_proj, img_target = tgt_projs[i], matching_points[i]
        
        dist = np.linalg.norm(np.array(tgt_proj['proj']) - np.array(img_target['center']))
        if dist < max_dist:
            tgts_matched_temp.append(tgts_matched[i])
            matching_points_temp.append(matching_points[i])

    new_num_matches = len(tgts_matched_temp)

    for tgt in tgts_matched:
        if tgt not in tgts_matched_temp:
            tgts_matched_temp.append(tgt)
    
    for mp in matching_points:
        if mp not in matching_points_temp:
            matching_points_temp.append(mp)

    return tgts_matched_temp, matching_points_temp, new_num_matches

    
def filter_nones(tgts, img_targets, num_matches):
    """Format matches to use a count instead of matching target or image target to None
    
    match_obj_and_img_pts matches each target to the closest img target. Therefore, it
    is possible that not every img target will have a match. Additionally, if max_dist
    is given to match_obj_and_img_pts, it is possible that not every target will have
    a match. match_obj_and_img_pts solves this by matching unmatched items to None.

    That causes something to fail, so this function reformats that output. Instead of
    using Nones, this function reorders the list of targets to the first n target have
    a match, and correspond to the first n items of the img targets list. n is
    additionally given as num_matches

    Parameters
    ----------
    tgts
    img_targets
    num_matches
    
    Returns
    ----------
    tuple (tgts, img_targets, num_matches)    
    """

    tgts_temp = []
    img_targets_temp = []
    after_num_matches = 0
    for i in range(num_matches):
        tgt, match = tgts[i], img_targets[i]
        if match is not None:
            tgts_temp.append(tgt)
            img_targets_temp.append(match)
            after_num_matches += 1
        
    for tgt in tgts:
        if tgt not in tgts_temp:
            tgts_temp.append(tgt)
    
    for img_target in img_targets:
        if (img_target not in img_targets_temp) and (img_target is not None):
            img_targets_temp.append(img_target)

    return tgts_temp, img_targets_temp, after_num_matches


def filter_matches(rmat, tvec, cameraMatrix, distCoeffs, tgts, img_targets, num_matches, test_config, debug=None):
    """Wrapper to run multiple filtering functions

    Parameters
    ----------
    
    Returns
    ----------
    
    
    num_matches - number of matches
    all visible tgts
    all img_targets
    - first n tgts match with first n img_targets
    TODO: function description
    """

    if (num_matches != 0) and (len(tgts) != 0) and (len(img_targets) != 0):
        tgts_a, img_targets_a, num_matches_a = filter_nones(tgts, img_targets, num_matches)

        tgts_b, img_targets_b, num_matches_b = filter_max_dist(
                rmat, tvec, cameraMatrix, distCoeffs,
                tgts_a, img_targets_a, num_matches_a, test_config['max_dist']
            )

        tgts_c, img_targets_c, num_matches_c = filter_one2one(tgts_b, img_targets_b, num_matches_b)
        
        tgts_d, img_targets_d, num_matches_d = filter_bifilter(
                rmat, tvec, cameraMatrix, distCoeffs,
                tgts_c, img_targets_c, num_matches_c, test_config['max_dist']
            )
        
        tgts_e, img_targets_e, num_matches_e = filter_min_dist(tgts_d, img_targets_d, num_matches_d, test_config['min_dist'])

        tgts_filtered = tgts_e[:num_matches_e]
        img_targets_filtered = img_targets_e[:num_matches_e]
    
    else:
        tgts_a, img_targets_a, num_matches_a = tgts, img_targets, num_matches
        tgts_b, img_targets_b, num_matches_b = tgts, img_targets, num_matches
        tgts_c, img_targets_c, num_matches_c = tgts, img_targets, num_matches
        tgts_d, img_targets_d, num_matches_d = tgts, img_targets, num_matches
        tgts_e, img_targets_e, num_matches_e = tgts, img_targets, num_matches
        tgts_filtered, img_targets_filtered = [], []

    if debug is not None:
        tgts_img_targets_num_matches = [[tgts, img_targets, num_matches],
                                        [tgts_a, img_targets_a, num_matches_a],
                                        [tgts_b, img_targets_b, num_matches_b],
                                        [tgts_c, img_targets_c, num_matches_c],
                                        [tgts_d, img_targets_d, num_matches_d],
                                        [tgts_e, img_targets_e, num_matches_e]]

        for i, data in enumerate(tgts_img_targets_num_matches):
            tgts_temp, img_targets_temp, num_matches_temp = data
            
            tgt_projs = photogrammetry.project_targets(rmat, tvec, cameraMatrix, distCoeffs, tgts_temp)
            
            proj_pts = np.array([tgt_proj['proj'] for tgt_proj in tgt_projs])
            img_centers = np.array([img_target['center'] for img_target in img_targets_temp])

            suffix = {0: 'Original_match',
                     1: 'filtered_a',
                     2: 'filtered_b',
                     3: 'filtered_c',
                     4: 'filtered_d',
                     5: 'filtered_e',}[i]
            name = debug[1] + '_' + suffix

            visualization.show_projection_matching(
                debug[0], proj_pts, img_centers,
                num_matches=num_matches_temp, name=name,
                bonus_pt=None, scale=1, ax=None)

    return tgts_filtered, img_targets_filtered


def match_obj_and_img_pts(rmat, tvec, cameraMatrix, distCoeffs, 
                          tgts, img_targets, max_dist=np.inf):
    """
    Parameters
    ----------
    
    Returns
    ----------
    
    Given a pose from rmat and tvec and intrinsics cameraMatrix and distCoeffs, match
        the 3D targets given with tgts to the 2D image locations of
        img_targets

    Current matching scheme is to just match with the closest target. The image location
        and projected location may be very far from each other. The matching may not be
        one-to-one

    Inputs:
        rmat and tvec are transformation from camera to the targets frame
            rmat - rotation matrix
            tvec - translation vector
        tgts, img_targets - target information
            tgts - 3D position of targets in tgts frame
            img_targets - image location of targets
        cameraMatrix, disCoeffs - intrinsics
            cameraMatrix - camera matrix
            distCoeffs - distortion coefficients
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
            if img_target['target_type'] != tgt_proj['target_type']:
                continue

            # Calculate the distance
            dist = np.linalg.norm([img_target['center'][0] - tgt_proj['proj'][0], 
                                   img_target['center'][1] - tgt_proj['proj'][1]])

            # If the distance is over the distance threshold and less than current
            #   lowest distance, make this the new best match
            if (dist < match_dist):
                match_dist = dist
                match = img_target
        
        matching_img_targets.append(match)

    # Since the target ordering is not changes, we can return the visibles and
    #   projected as was given
    return tgts, matching_img_targets


def match_targets(rmat, tvec, cameraMatrix, distCoeffs, tgts, img_targets, max_dist=np.inf, debug=None):
    """Matches each target to the closest img target

    If max_dist is given, distance between target and img target must be less than
    max_dist. By default this value is infinite, so any match is valid.

    Parameters
    ----------
    rmat
    tvec
    cameraMatrix
    distCoeffs
    tgts
    img_targets
    max_dist
    debug
    
    Returns
    ----------
    
    Wrapper to match the 3D targets to the 2D image targets
    
    debug is [camera frame, figure name, optional] if debug images are to be generated
        optional can be None or test_config
    """

    # Match the visible targets to the closest image point
    tgts_matched, matching_points = match_obj_and_img_pts(
        rmat, tvec, cameraMatrix, distCoeffs, tgts, img_targets, max_dist)

    tgts_matched, matching_points, num_matches = filter_nones(tgts_matched, matching_points, len(tgts_matched))

    for tgt in tgts:
        if tgt not in tgts_matched:
            tgts_matched.append(tgt)
    for img_target in img_targets:
        if img_target not in matching_points:
            matching_points.append(img_target)

    if debug is not None:
        if debug[2] is not None:
            tgts_matched_temp, img_targets_temp = filter_matches(
                rmat, tvec, cameraMatrix, distCoeffs, tgts_matched, matching_points,
                num_matches, debug[2], debug[:2])
            num_matches_temp = len(tgts_matched_temp)
        else:
            tgts_matched_temp, img_targets_temp = tgts_matched, matching_points
            num_matches_temp = num_matches

        tgts_unmatched = []
        for tgt in tgts:
            if tgt not in tgts_matched_temp:
                tgts_unmatched.append(tgt)

        img_targets_unmatched = []
        for img_tgt in img_targets:
            if img_tgt not in img_targets_temp:
                img_targets_unmatched.append(img_tgt)

        tgt_projs = photogrammetry.project_targets(
                rmat, tvec, cameraMatrix, distCoeffs, tgts_matched_temp + tgts_unmatched
            )
        
        proj_pts = np.array([tgt_proj['proj'] for tgt_proj in tgt_projs])
        img_centers = np.array([img_target['center'] for img_target in img_targets_temp + img_targets_unmatched])

        visualization.show_projection_matching(
            debug[0], proj_pts, img_centers,
            num_matches = num_matches_temp, name=debug[1], bonus_pt=None, scale=1, ax=None)

    return tgts_matched, matching_points, num_matches


#---------------------------------------------------------------------------------------
# External Calibration Wrappers


def external_calibrate(img, rmat, tvec, # Frame specific
                       cameraMatrix, distCoeffs, # Camera specific
                       tgts, img_targets, vis_checker, test_config, # Config specific
                       isMatched=False, max_localize_delta=None, # Test specific
                       reprojectionError=6.0 # Test specific (should be stable between tests)
                       ):
    """Set up and run solvePnPRansac to get the external calibration and inlier targets

    Parameters
    ----------
    img
    rmat
    tvec
    cameraMatrix
    distCoeffs
    tgts
    img_targets
    vis_checker
    test_config
    isMatched
    max_localize_delta
    reprojectionError

    Returns
    ----------
    
    """
    # If the inputs are not matched, get the visible targets and match them. 
    if not isMatched:
        # Determine which targets are visible
        visibles_tgts = photogrammetry.get_visible_targets(rmat, tvec, tgts, vis_checker)

        # Match the projected locations to the image locations
        tgts_matched, img_targets_matched, num_matches_init = match_targets(
            rmat, tvec, cameraMatrix, distCoeffs, visibles_tgts, img_targets,
            test_config['max_dist'])
            
    # If the input is matched, short circuit the tgt_matched and
    #   img_targets_matched with the inputs
    else:
        tgts_matched = tgts
        img_targets_matched = img_targets
        num_matches_init = len(tgts)

    # Filter the matched targets
    tgts_filtered, img_targets_filtered = filter_matches(
            rmat, tvec, cameraMatrix, distCoeffs,
            tgts_matched, img_targets_matched, num_matches_init,
            test_config
        )
    
    # Subpixel localize the image targets
    tgts_subpixel, img_targets_subpixel = subpixel_localize_robust(
        img, tgts_filtered, img_targets_filtered, test_config,
        max_localize_delta=max_localize_delta)

    # If there are less than 4 matches, raise an error
    if len(tgts_subpixel) < 4:
        raise ValueError("Less than 4 matches were found in external_calibrate. " + 
            "This can be due to blob detection finding too few targets, too few " +
            "visible targets, a bad matching scheme due to a bad starting pose " + 
            "(rmat and tvec), and/or too many targets rejected during the sub-pixel " +
            "localization.")

    # Package the target tvecs
    tgt_tvecs = []
    for tgt in tgts_subpixel:
        tgt_tvecs.append(tgt['tvec'])
    tgt_tvecs = np.array(tgt_tvecs)

    # Package the image points
    img_centers = []
    for target in img_targets_subpixel:
        img_centers.append(target['center'])
    img_centers = np.array(img_centers)

    # Convert rmat to rvec
    rvec, _ = cv2.Rodrigues(rmat)

    # Solve for the new tvec and rvec
    retval, rvec_opt, tvec_opt, inliers = cv2.solvePnPRansac(
        tgt_tvecs, img_centers, cameraMatrix, distCoeffs,
        copy.copy(rvec), copy.copy(tvec), reprojectionError=reprojectionError,
        useExtrinsicGuess=True)

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


def check_external_calibrate_two_stage_inputs(img, rmat_init_guess, tvec_init_guess,
                                              camera_cal, tgts, test_config, vis_checker,
                                              debug):
    """Check that the inputs to external calibrate are valid

    Helper function to external_calibrate_two_stage. Checks that the input are of the
    proper type, size, format, and have the relevant attributes.

    Parameters
    ----------
    img
    rmat_init_guess
    tvec_init_guess
    camera_cal
    tgts
    test_config
    vis_checker
    debug
    
    Returns
    ----------
    
    """
    # TODO: Any time "isinstance(OBJ, list)" is called, np.array(OBJ) should be inside
    #   of a try-except in case it has degenerate objects

    #-----------------------------------------------------------------------------------
    # Check img input
    if isinstance(img, np.ndarray):
        if img.dtype != np.uint8:
            print('img dtype should be uint8 not', img.dtype)
            return False

        # If img is not 2 dimensional, it should be 3 dimensional, but single channel
        if len(img.shape) != 2:
            if len(img.shape) == 3:
                if img.shape[2] != 1:
                    print('img should be single channel, not', img.shape[2], 'channel')
                    return False
            else:
                # img is not 2 dimensional or 3 dimension. Must be an error
                print('img should be 2 dimensional, or 3 dimensional and single channel. Not ', len(img.shape))
                return False

    else:
        print('img type should be np.ndarray. Not ', type(img))
        return False

    #-----------------------------------------------------------------------------------
    # Check rmat_init_guess input

    # if rmat_init_guess is a list, convert it to a numpy array to check the data
    if isinstance(rmat_init_guess, list):
        rmat_init_guess = np.array(rmat_init_guess)

    if isinstance(rmat_init_guess, np.ndarray):
        if rmat_init_guess.dtype != np.float32 and rmat_init_guess.dtype != np.float64:
            print('rmat_init_guess.dtype should be float32 or float64 not', rmat_init_guess.dtype)
            return False
        if rmat_init_guess.shape != (3, 3):
            print('rmat_init_guess.shape should be (3, 3). Not ', rmat_init_guess.shape)
            return False
        if not photogrammetry.isRotationMatrix(rmat_init_guess):
            print('rmat_init_guess is not a valid rotation matrix')
            return False

    else:
        print('rmat_init_guess should be np.ndarray or list. Not', type(rmat_init_guess))
        return False
    
    #-----------------------------------------------------------------------------------
    # Check tvec_init_guess input

    # if tvec_init_guess is a list, convert it to a numpy array to check the data
    if isinstance(tvec_init_guess, list):
        tvec_init_guess = np.array(tvec_init_guess)

    if isinstance(tvec_init_guess, np.ndarray):
        if tvec_init_guess.dtype != np.float32 and tvec_init_guess.dtype != np.float64:
            print('tvec_init_guess.dtype should be float32 or float64 not', tvec_init_guess.dtype)
            return False
        if tvec_init_guess.shape != (3,) and tvec_init_guess.shape != (3, 1):
            print('tvec_init_guess.shape should be (3,) or (3, 1). Not ', tvec_init_guess.shape)
            return False
        
    else:
        print('tvec_init_guess should be np.ndarray or list. Not', type(tvec_init_guess))
        return False
    
    #-----------------------------------------------------------------------------------
    # Check camera_cal input
    # Only need to check camera_cal[0] and camera_cal[1]
    
    # if camera_cal[0] is a list, convert it to a numpy array to check the data
    if isinstance(camera_cal[0], list):
        camera_cal[0] = np.array(camera_cal[0])

    if isinstance(camera_cal[0], np.ndarray):
        if camera_cal[0].dtype != np.float32 and camera_cal[0].dtype != np.float64:
            print('camera_cal[0].dtype should be float32 or float64 not', camera_cal[0].dtype)
            return False
        if camera_cal[0].shape != (3, 3):
            print('camera_cal[0].shape should be (3, 3). Not ', camera_cal[0].shape)
            return False
        
        # camera_cal should be of the form [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
        if (camera_cal[0][2] != [0.0, 0.0, 1.0]).any():
            print('camera_cal[0][2] must be [0, 0, 1]')
            return False
        if camera_cal[0][0][1] != 0:
            print('camera_cal[0][0][1] must be 0')
            return False
        if camera_cal[0][1][0] != 0:
            print('camera_cal[0][1][0] must be 0')
            return False
        
    else:
        print('camera_cal[0] should be np.ndarray or list. Not', type(camera_cal[0]))
        return False
    
    # if camera_cal[1] is a list, convert it to a numpy array to check the data
    if isinstance(camera_cal[1], list):
        camera_cal[1] = np.array(camera_cal[1])

    if isinstance(camera_cal[1], np.ndarray):
        if camera_cal[1].dtype != np.float32 and camera_cal[1].dtype != np.float64:
            print('camera_cal[1].dtype should be float32 or float64 not', camera_cal[1].dtype)
            return False
        if camera_cal[1].shape != (1, 5):
            print('camera_cal[1].shape should be (1, 5). Not ', camera_cal[1].shape)
            return False
        
    else:
        print('camera_cal[1] should be np.ndarray or list. Not', type(camera_cal[1]))
        return False

    #-----------------------------------------------------------------------------------
    # Check tgts input
    
    # Check that each target is as expected
    for tgt in tgts:
        if 'tvec' not in tgt or 'norm' not in tgt or 'target_type' not in tgt:
            print('Each item in tgts must have a tvec, norm, and target_type. One or more of the items in tgts did not meet this criteria')
            return False
    
    if len(tgts) < 4:
        print('tgts must have at least 4 items. Only', len(tgts), 'were found')
        return False

    #-----------------------------------------------------------------------------------
    # Check test_config input
        
    if ('dot_blob_parameters' not in test_config.keys()) and ('kulite_blob_parameters' not in test_config.keys()):
        print("Neither 'dot_blob_parameters' nor 'kulite_blob_parameters' in test_config.keys()")
        return False

    if 'dot_blob_parameters' in test_config.keys():
        if not isinstance(test_config['dot_blob_parameters'], list):
            print("test_config['dot_blob_parameters'] should be a list, not", type(test_config['dot_blob_parameters']))
            return False
    if 'kulite_blob_parameters' in test_config.keys():
        if not isinstance(test_config['kulite_blob_parameters'], list):
            print("test_config['kulite_blob_parameters'] should be a list, not", type(test_config['kulite_blob_parameters']))
            return False

    # TODO: Need to check that test_config['blob_parameters'] contains all valid
    #   items. But that is a lot of work, and it fails early enough in
    #   external_calibration_two_stage that it doesn't matter much

    if 'max_dist' not in test_config.keys():
        print("'max_dist' not in test_config.keys()")
        return False

    if not isinstance(test_config['max_dist'], (int, float)):
        print("test_config['max_dist'] must be numeric")
        return False

    if 'min_dist' not in test_config.keys():
        print("'min_dist' not in test_config.keys()")
        return False

    if not isinstance(test_config['min_dist'], (int, float)):
        print("test_config['min_dist'] must be numeric")
        return False

    #-----------------------------------------------------------------------------------
    # Check vis_checker input
    
    if not type(vis_checker).__name__ == 'VisibilityChecker':
        print('vis_checker should be visibility.VisibilityChecker object. Not', type(vis_checker))
        return False

    if not type(vis_checker.scene).__name__ == 'BVH':
        print('vis_checker.scene should be upsp.raycast.CreateBVH object. Not', type(vis_checker.scene))
        return False

    #-----------------------------------------------------------------------------------
    # Check debug input

    # If debug is not None object, a string, or integer then it is improper
    if debug != None and not isinstance(debug, str) and not isinstance(debug, int):
        print('debug should be None, str, or int. Not', type(debug))
        return False

    return True


def external_calibrate_two_stage_from_wtd(img,  # Frame specific
                                          tunnel_vals,  # Datapoint specific
                                          camera_cal,  # Camera specific
                                          tgts, test_config, vis_checker, # Test/configuration specific
                                          debug=None):
    """Wrapper function to external_calibrate_two_stage. Parses wtd into rmat and tvec

    Parameters
    ----------
    img
    tunnel_vals
    camera_cal
    tgts
    test_config
    vis_checker
    debug
    
    Returns
    ----------

    """
    
    rmat_init_guess, tvec_init_guess = photogrammetry.tf_camera_tgts_thru_tunnel(
            camera_cal, tunnel_vals, test_config
        )
    
    rmat, tvec = external_calibrate_two_stage(
            img, rmat_init_guess, tvec_init_guess, camera_cal, tgts, test_config, vis_checker, debug
        )
    
    return rmat, tvec


def external_calibrate_two_stage(img,  # Frame specific
                                 rmat_init_guess, tvec_init_guess,  # Datapoint specific
                                 camera_cal,  # Camera specific
                                 tgts, test_config, vis_checker, # Test/configuration specific
                                 debug=None):
    """Performs external calibration from an inaccurate initial guess

    Runs blob detection to find the img targets, then matches and filters the 3D targets
    to the img targets. Performs a coarse optimization to improve the intial guess. Then
    calls external_calibrate_one_step to get the refined optimization
    
    Parameters
    ----------
    img
    rmat_init_guess
    tvec_init_guess
    camera_cal
    tgts
    test_config
    vis_checker
    debug
    
    Returns
    ----------
    
    """
    # Check that the inputs are valid
    check_bool = check_external_calibrate_two_stage_inputs(
        img, rmat_init_guess, tvec_init_guess, camera_cal, tgts, test_config, vis_checker, debug
    )
    
    # Scale the image
    img = img_utils.scale_image_max_inlier(img)
    
    # Check the inputs
    if not check_bool:
        raise ValueError("One or more bad inputs were given to external_calibrate_two_stage")

    # Unpack the intrinsics
    cameraMatrix, distCoeffs = camera_cal[0], camera_cal[1],

    # Check if there are enough dots to use for targets
    dots_found = True
    init_tgts = []
    for tgt in tgts:
        if tgt['target_type'] == 'dot':
            init_tgts.append(tgt)
    
    # If there were enough dots, use the dots blob parameters
    if len(init_tgts) > 4:
        blob_parameters = test_config["dot_blob_parameters"]
    
    # If there were not enough dots, collect the kulites and the kulite blob parameters
    else:
        # Set the dot_found flag to False
        dots_found = False
        blob_parameters = test_config["kulite_blob_parameters"]
    
        for tgt in tgts:
            if tgt['target_type'] == 'kulite':
                init_tgts.append(tgt)

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
        init_img_target = {'target_type' : {True:'dot', False:'kulite'}[dots_found], # target_type depends on if enough dots were found
                          'center' : keypoint.pt}
        init_img_targets.append(init_img_target)

    # Debug for raw matching information
    if debug_raw_matches:
        tgts_visible = photogrammetry.get_visible_targets(
            rmat_init_guess, tvec_init_guess, init_tgts, vis_checker)

        tgts_match_raw, img_targets_match_raw, num_matches_raw = match_targets(
            rmat_init_guess, tvec_init_guess, cameraMatrix, distCoeffs,
            tgts_visible, init_img_targets, test_config['max_dist'])
        
        tgts_filtered_raw, img_targets_filtered_raw = filter_matches(
            rmat_init_guess, tvec_init_guess, cameraMatrix, distCoeffs,
            tgts_match_raw, img_targets_match_raw, num_matches_raw,
            test_config)
        
        tgts_subpixel_raw, img_targets_subpixel_raw = subpixel_localize_robust(
            img, tgts_filtered_raw, img_targets_filtered_raw, test_config,
            max_localize_delta=None)
        num_matches = len(img_targets_subpixel_raw)

        rms, max_dist = photogrammetry.reprojection_error(
            rmat_init_guess, tvec_init_guess, cameraMatrix, distCoeffs,
            tgts_subpixel_raw, img_targets_subpixel_raw)

        log.info(
            'Raw Num Points: %d RMS: %f Max Error: %f',
            len(tgts_subpixel_raw), rms, max_dist
        )
        
        # Get a list of the targets that are visible, but weren't matched
        visible_but_not_matched = []
        for tgt in tgts_visible:
            if tgt not in tgts_subpixel_raw:
                visible_but_not_matched.append(tgt)

        # Get projected location the visible target centers in the image
        tgt_projs = photogrammetry.project_targets(
            rmat_init_guess, tvec_init_guess, cameraMatrix, distCoeffs,
            tgts_subpixel_raw + visible_but_not_matched)
        proj_pts = np.array([tgt_proj['proj'] for tgt_proj in tgt_projs])
        
        found_but_not_matched = []
        for tgt_match_raw, img_target_match_raw in zip(tgts_match_raw, img_targets_match_raw):
            if tgt_match_raw not in tgts_subpixel_raw:
                found_but_not_matched.append(img_target_match_raw)
        
        all_img_targets = img_targets_subpixel_raw + found_but_not_matched
        img_centers = [inlier_pt['center'] for inlier_pt in all_img_targets]
        img_centers = np.array(img_centers)

        # Get the debug name. If debug was given, use it otherwise don't
        debug_name = str(debug)+'_raw' if debug != None else 'raw'

        # Output a debug image of the projected locations and image target center locations
        visualization.show_projection_matching(img, proj_pts, img_centers, num_matches=num_matches,
                                               name=debug_name, scale=2.)
    
    # Check that enough blobs were found
    if len(keypoints) < 4:
        raise ValueError("Less than 4 blobs were found in external_calibrate_two_stage.")

    # Do the coarse external calibration    
    coarse_outputs = external_calibrate(
        img, rmat_init_guess, tvec_init_guess, cameraMatrix, distCoeffs, 
        init_tgts, init_img_targets, vis_checker, test_config,
        isMatched=False, max_localize_delta=None, reprojectionError=test_config['max_dist'])
    
    # Unpack the output variables
    rmat_coarse, tvec_coarse, tgts_inliers_coarse, img_target_inliers_coarse = coarse_outputs

    if debug_coarse_optimization:
        rms, max_dist = photogrammetry.reprojection_error(
            rmat_coarse, tvec_coarse, cameraMatrix, distCoeffs, 
            tgts_inliers_coarse, img_target_inliers_coarse)

        log.info(
            'Coarse Num Points: %d RMS: %f Max Error: %f',
            len(tgts_inliers_coarse), rms, max_dist
        )
    
        # Get projected location the visible targets in the image
        tgt_projs = photogrammetry.project_targets(
            rmat_coarse, tvec_coarse, cameraMatrix, distCoeffs,
            tgts_inliers_coarse)

        proj_pts = np.array([tgt_proj['proj'] for tgt_proj in tgt_projs])
        img_pts = np.array([inlier_pt['center'] for inlier_pt in img_target_inliers_coarse])
        
        # Get the debug name. If debug was given, use it otherwise don't
        debug_name = str(debug)+'_coarse' if debug != None else 'coarse'
        
        visualization.show_projection_matching(img, proj_pts, img_pts,
                                               name=debug_name, scale=2)

    # Call the one step function for the refined optimization
    rmat_refined, tvec_refined = external_calibrate_one_step(
        img, rmat_coarse, tvec_coarse, camera_cal, init_tgts, test_config, vis_checker, debug)
    
    return rmat_refined, tvec_refined


def external_calibrate_one_step(img,  # Frame specific
                                rmat_coarse, tvec_coarse,  # Datapoint specific
                                camera_cal,  # Camera specific
                                init_tgts, test_config, vis_checker, # Test/configuration specific
                                debug=None):
    """Performs external calibration from a seim-accurate coarse guess
    
    Parameters
    ----------
    img
    rmat_coarse
    tvec_coarse
    camera_cal
    init_tgts
    test_config
    vis_checker
    debug
    
    Returns
    ----------
    
    """
    # Scale the image
    img = img_utils.scale_image_max_inlier(img)

    # Unpack the intrinsics
    cameraMatrix, distCoeffs = camera_cal[0], camera_cal[1],

    # Get the visible targets
    visible_tgts = photogrammetry.get_visible_targets(rmat_coarse, tvec_coarse,
                                                      init_tgts, vis_checker)
    
    # Get the projections of the visible targets
    tgt_projs = photogrammetry.project_targets(rmat_coarse, tvec_coarse,
                                               cameraMatrix, distCoeffs,
                                               visible_tgts)

    # Package the target projections into img targets
    img_targets = []
    for tgt_proj in tgt_projs:
        img_target = {'target_type' : tgt_proj['target_type'],
                      'center' : tgt_proj['proj']}
        img_targets.append(img_target)
    
    # Debuf for refined matches
    if debug_refined_matches:
        # Filter the matched targets
        tgts_filtered, img_targets_filtered = filter_matches(
            rmat_coarse, tvec_coarse, cameraMatrix, distCoeffs,
            visible_tgts, img_targets, len(visible_tgts),
            test_config
            )
    
        # Subpixel localize the image targets
        tgts_subpixel, img_targets_subpixel = subpixel_localize_robust(
            img, tgts_filtered, img_targets_filtered, test_config,
            max_localize_delta=None)
        num_matches = len(img_targets_subpixel)
        
        # Get a list of the targets that are visible, but weren't matched
        visible_but_not_matched = []
        for tgt in visible_tgts:
            if tgt not in tgts_subpixel:
                visible_but_not_matched.append(tgt)

        # Get projected location the visible target centers in the image
        tgt_projs = photogrammetry.project_targets(
            rmat_coarse, tvec_coarse, cameraMatrix, distCoeffs,
            tgts_subpixel + visible_but_not_matched)
        proj_pts = np.array([tgt_proj['proj'] for tgt_proj in tgt_projs])
        
        img_centers = np.array([img_target['center'] for img_target in img_targets_subpixel])

        rms, max_dist = photogrammetry.reprojection_error(
            rmat_coarse, tvec_coarse, cameraMatrix, distCoeffs,
            tgts_subpixel, img_targets_subpixel)

        # Get the debug name. If debug was given, use it otherwise don't
        debug_name = str(debug)+'_refined' if debug != None else 'refined'

        # Output a debug image of the projected locations and image target center locations
        visualization.show_projection_matching(img, proj_pts, img_centers, num_matches=num_matches,
                                               name=debug_name, scale=2.)

    # Run the refined external calibration
    refined_outputs = external_calibrate(
        img, rmat_coarse, tvec_coarse, cameraMatrix, distCoeffs,
        visible_tgts, img_targets, vis_checker, test_config,
        isMatched=True, max_localize_delta=None, reprojectionError=test_config['max_dist'])
    
    # Unpack the refined results
    rmat_refined, tvec_refined, tgts_inliers_refined, img_target_inliers_refined = refined_outputs

    # Debug for refined optimization
    if debug_refined_optimization:
        rms, max_dist = photogrammetry.reprojection_error(
            rmat_refined, tvec_refined, cameraMatrix, distCoeffs, 
            tgts_inliers_refined, img_target_inliers_refined)

        log.info(
            'Refined Num Points: %d RMS: %f Max Error: %f',
            len(tgts_inliers_refined), rms, max_dist
        )
        
        # Get projected location the visible targets in the image
        tgt_projs = photogrammetry.project_targets(rmat_refined, tvec_refined,
                                                   cameraMatrix, distCoeffs,
                                                   tgts_inliers_refined)

        # Plot the kulites in blue and dots in red
        plt.imshow(img, cmap='gray')
        for tgt_proj in tgt_projs:
            plt.scatter([tgt_proj['proj'][0]], [tgt_proj['proj'][1]],
                        c={'kulite':'b', 'dot':'r'}[tgt_proj['target_type']],
                        marker='o', s=0.05)

        # Get the debug name. If debug was given, use it otherwise don't
        debug_name = str(debug)+'_refined_optimization.png' if debug != None else 'refined_optimization.png'

        plt.savefig(debug_name, dpi = 400)
        plt.close()

    # Secondary debug to project all visible targets, not just the targets used in the
    #   optimization
    if debug_visible_projections:
        # Get visible targets
        visible_tgts = photogrammetry.get_visible_targets(rmat_refined, tvec_refined,
                                                          init_tgts, vis_checker)
    
        # Get projected location the visible targets in the image
        tgt_projs = photogrammetry.project_targets(rmat_refined, tvec_refined,
                                                   cameraMatrix, distCoeffs,
                                                   visible_tgts)

        # Plot the kulites in blue and dots in red
        plt.imshow(img, cmap='gray')
        for i, tgt_proj in enumerate(tgt_projs):
            plt.scatter([tgt_proj['proj'][0]], [tgt_proj['proj'][1]],
                        c={'kulite':'b', 'dot':'r'}[tgt_proj['target_type']], 
                        marker='o', s=0.05)
        
        # Get the debug name. If debug was given, use it otherwise don't
        debug_name = str(debug)+'_visible_projection.png' if debug != None else 'visible_projection.png'
        
        plt.savefig(debug_name, dpi = 400)
        plt.close()

    return rmat_refined, tvec_refined

