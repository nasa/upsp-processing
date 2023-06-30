import cv2
import numpy as np
from shapely.geometry import Polygon
import copy

from upsp.cam_cal_utils import (
    photogrammetry,
)

from upsp.target_operations import gaussian_localization_methods
gauss_fit = gaussian_localization_methods.gauss_fitter_func("super")

debug_show_localizations = False

num_poly_pts = 100


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


def pixel_2_polygon(u, v):
    """ Returns a square Polygon object of length 1 centered at (u, v)

    Parameters
    ----------
    u : float
        Pixel x position
    v : float
        Pixel y position

    Returns
    ----------
    Polygon object
        Polygon of square pixel returned as a shapely polygon object
    """

    return Polygon([[u - 0.5, v - 0.5], [u + 0.5, v - 0.5],
                    [u + 0.5, v + 0.5], [u - 0.5, v + 0.5]])


def area_overlap(target, pixel_coord):
    """Calculates area overlap between target and pixel

    Parameters
    ----------
    target : Polygon
        Shapely Polygon object. Suggested to create target using ellipse_2_polygon
    pixel_coords : tuple
        Length 2 iterable (tuple, np.ndarray, etc). First index is x coordinate (u) and
        second index is y coordinate (v)

    Returns
    ----------
    float
        Returns the area overlap between the target and pixel. Effectively the amount
        that the pixel is covered by the ellipse (assumes area of pixel is 1)
    """
    px = pixel_2_polygon(*pixel_coord)
    return px.intersection(target).area


def template_detection(
    img, rmat, tvec, cameraMatrix, distCoeffs, visible_tgts, test_config
):
    """ This function uses template matching to detect targets

    The function needs the internal calibration (cameraMatrix and distCoeffs), the 3D
    positions of the targets assumed to be visible (visible_tgts), and an initial guess
    of the external calibration (rmat and tvec). Hyper parameters given in test_config
    (max_dist, target_pad, and crosscorr_coeff) tune the search parameters.

    test_config[tgt["target_type"] + "_pad"] is the radius of the target for tgt with
    tgt["target_type"]. The external calibration initial guess needs to be close enough
    to the true external calibration that the projected target position within a pixel
    distance of test_config["max_dist"] for the detection to occur. 

    For each target:
    1) using the internal cand external calibrations, a template of the target is
    generated. Using the 3D target diameter given in visible_tgts[i]['size'], the points
    on the edge of the target (in 3D space) are projected into the image to make a
    polygon approximation of the target in image space. That polygon is used to make an
    image of the target, which acts as the template. Two templates are made. One using
    the projected points, and one with the projected points shifted diagonally by 0.5
    pixels in the x and y direction. Since the template is very small, shifting by 0.5
    of a pixel can significantly change the template.
    2) The image is cropped to the region around the projected position of the target.
    The crop is a square with a side length of 2 * (test_config["max_dist"] +
    test_config[tgt["target_type"] + "_pad"]). This region is used so that the crop
    contains all allowable positions of the target, and contains the entire target. If
    only 2 * test_config["max_dist"] used, the image target's center could be on the
    edge of the crop, but half the image target would be outside the crop so the
    template wouldn't match. The crop is square so that OpenCV's methods can be used
    (since OpenCV is highly optimized).
    3) Template matching is used to find all sites within the crop where the cross
    correlation coefficient is above test_config["crosscorr_coeff"]. This step uses the
    union of the sets of target sites found with both templates (original and shifted
    template)
    3.5) If there are no target sites found, skip to the next target
    4) Target sites farther than a distance of test_config["max_dist"] from the the
    center of the crop are removed. This makes is so the image targets are within a 
    distance of test_config["max_dist"], and also targets farther than 
    test_config["max_dist"] do not invalidate the detection in the next step
    4.5) If there is exactly 1 target site remaining, that is used for the detection
    of this target
    5) If there are more than 1 target sites remaining, we will use the detection with
    the highest cross correlation coefficient (amoung both templates). The caveat is we
    can only use that detection if all target sites are for the same target. I.e. if the
    detections are on opposite sides of the crop, they are likely for different targets.
    This is computed by finding the bounding box for the target sites, then checking if
    the bounding box fits within a circle with radius
    test_config[tgt["target_type"] + "_pad"]. If it does, then we can use the detection.

    After all detections are made, the targets of visible_tgts that have an associated
    detectiona are copied into tgts_matched. The targets of visible_tgts that do not
    have an associated detectiona are copied into tgts_unmatched. The output
    tgts_detected is then tgts_matched + tgts_unmatched. And num_matches is
    len(tgts_matched). That way the first num_matches elements of tgts_detected are
    associated 1-1 with the first num_matches elements of img_targets

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
    visible_tgts : list of dict
        Each target is a dict that has, at a minimum, the keys 'tvec', and
        'target_type'. If isMatched is False, 'norm' is additionally needed as a key.
        'tvec' has a np.ndarray (3, 1) representing the position of the target relative
        to the model origin for its associated value. 'target_type' has a string
        representing the type of target (most commonly 'dot') for its associated value.
        ``tgts[i]`` is associated with ``img_targets[i]`` for i from 0 to `num_matches`.
        'norm' has a :class:`numpy.ndarray` (3, 1) representing the normal vector of the
        target relative to the model coordinate system for its associated value
    test_config : dict
        dict with, at a minimum, a key for 'max_dist', 'crosscorr_coeff' and
        '[target_type] + "_pad"' for each target type of the targets in tgts. 'max_dist'
        is the search distance used for the template detection. 'max_dist' is the max
        pixel distance between the projected position of the target using the given
        external calibration, and the discovered image location of the target.
        'crosscorr_coeff' is the crosscorrelation coefficient threshold for thetemplate
        matching. '[target_type] + "_pad"' is the distance used to determine if multiple
        template matches correspond to one target, or to multiple targets

    Returns
    -------
    tgts_detected : list of dict
        Reordered version of visible_targets so that ``img_targets[i]`` is associated
        with ``tgts_detected[i]`` for i from 0 to num_matches
    img_targets : list of dict
        Matched image targets. Each dict has, the keys 'center', and
        'target_type'. 'center' is the image target center location (tuple/np.ndarray of
        length 2 of floats) and 'target_type' has the key of the type of target
        (string). ``img_targets[i]`` is associated with ``tgts_detected[i]`` for i from
        0 to num_matches
    num_matches : int
        Number of detections/matches found
    """

    # Assume the target is small relative to the curvature of the model
    # Assume the target is a perfect circle
    # Find the plane relatively to the camera that the target lies on
    # Project points from the target into the camera
    # Use those points to make a Polygon

    img_targets = []
    for tgt in visible_tgts:
        # Basis vectors of plane based on a pt at the origin and normal (a, b, c)
        #   Step 1, find another point on the plane
        #       Equation of plane and pt (x0, y0, z0)
        #   a(x-x0) + b(y-y0) + c (z-z0) = 0
        #   ax + by + cz = ax0 + by0 + cz0
        #   x and y are free variables, so use (1, 1)
        #   cz = ax0 + by0 + cz0 - a - b
        #       (x0, y0, z0) = (0, 0, 0)
        #   z = -(a + b) / c
        #   Normalize Vector - Basis Vector 1
        #   Basis Vector 2 - Cross Product between Normal and Basis Vector 1

        # z location of point on plane with x = 1 and y = 1
        z0 = -(tgt['norm'][0][0] + tgt['norm'][1][0]) / tgt['norm'][2][0]
        
        # Normalize x, y, and z
        basis_v0 = [1, 1, z0] / np.linalg.norm([1, 1, z0])

        # Get second basis vector
        basis_v1 = np.cross(tgt['norm'][:, 0], basis_v0)
        basis_v1 /= np.linalg.norm(basis_v1)
        
        # Scale based on the target size (size is diameter, we want radius)
        basis_v0 = basis_v0 * tgt['size'] / 2
        basis_v1 = basis_v1 * tgt['size'] / 2

        # Convert to (3, 1) vectors
        basis_v0 = basis_v0.reshape(3, 1)
        basis_v1 = basis_v1.reshape(3, 1)

        # Get points along the edge of the target
        thetas = np.linspace(0, 2*np.pi, num_poly_pts).reshape(1, -1)
        target_edge_points = np.cos(thetas) * basis_v0 + np.sin(thetas) * basis_v1
        target_edge_points += tgt['tvec']

        # Project those points into the image
        img_points = photogrammetry.project_3d_point(
            rmat, tvec, cameraMatrix, distCoeffs, target_edge_points
        )

        # Get a shapely polygon representation of the polyon
        tgt_poly0 = Polygon(img_points)

        # Get the bounding box of the projected points
        x_min, x_max = min(img_points[:, 0]+0.5), max(img_points[:, 0]-0.5)
        y_min, y_max = min(img_points[:, 1]+0.5), max(img_points[:, 1]-0.5)
        
        projected_center = [(x_min + x_max)/2, (y_min + y_max)/2]
        
        x_min, y_min = np.floor(x_min).astype(int), np.floor(y_min).astype(int)
        x_max, y_max = np.ceil(x_max).astype(int), np.ceil(y_max).astype(int)
        
        # Get the crop of the image near the target
        crop_radius = test_config["max_dist"] + test_config[tgt["target_type"] + "_pad"]
        bbox = np.array([
            [
                np.floor(projected_center[0] - crop_radius),
                np.floor(projected_center[1] - crop_radius)
            ],
            [
                np.ceil(projected_center[0] + crop_radius),
                 np.ceil(projected_center[1] + crop_radius)
            ],
        ]).astype(int)
        
        # If bbox goes out of bounds of the image, ignore it
        if (
            (bbox[0][0] < 0)
            or (bbox[0][1] < 0)
            or (bbox[1][0] >= img.shape[1])
            or (bbox[1][1] >= img.shape[0])
        ):
            img_targets.append(None)
            continue

        img_cropped = copy.deepcopy(
            img[bbox[0][1] : bbox[1][1], bbox[0][0] : bbox[1][0]]
        )
        img_cropped -= img_cropped.min()
        img_cropped = 255 * (img_cropped / img_cropped.max())
        img_cropped = np.rint(img_cropped).astype(np.uint8)

        # For each pixel in the bounding box, find the overlap
        # This creates a template of the target
        template0 = np.zeros((y_max - y_min + 1, x_max - x_min + 1))
        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                template0[y - y_min][x - x_min] = area_overlap(tgt_poly0, (x, y))
        template0 = ~np.rint(255 * template0).astype(np.uint8)

        # Get a shapely representation of the polygon shifted by 1/2 a pixel
        img_points_shifted = img_points + 0.5
        tgt_poly0pt5 = Polygon(img_points_shifted)
        
        # Get the bounding box of the projected points
        x_min = min(img_points_shifted[:, 0]+0.5)
        x_max = max(img_points_shifted[:, 0]-0.5)
        
        y_min = min(img_points_shifted[:, 1]+0.5)
        y_max = max(img_points_shifted[:, 1]-0.5)
        
        x_min = np.floor(x_min).astype(int)
        y_min = np.floor(y_min).astype(int)
        
        x_max = np.ceil(x_max).astype(int)
        y_max = np.ceil(y_max).astype(int)
        
        # Shift the projected points by half a pixel and create a second template
        template0pt5 = np.zeros((y_max - y_min + 1, x_max - x_min + 1))
        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                template0pt5[y - y_min][x - x_min] = area_overlap(tgt_poly0pt5, (x, y))
        template0pt5 = ~np.rint(255 * template0pt5).astype(np.uint8)

        # Perform template matching to find the target
        # All the methods but cv2.TM_CORR gave the same results on small test set
        # cv2.TM_CCOEFF_NORMED has nice theoretical properties because it corrects for
        #   brightness (looks for correlation) and is normalized to correct variations
        #   in template size
        # cv2.CORR and its normed variant latch into the brightest part of the image
        # cv2.TM_SQDIFF and its normed variant are wildly inconsistent with the
        #   threshold, potentially due to noise (camera and surface texture)
        res0 = cv2.matchTemplate(img_cropped, template0, cv2.TM_CCOEFF_NORMED)
        target_sites0 = np.argwhere(res0 > test_config["crosscorr_coeff"])

        res0pt5 = cv2.matchTemplate(img_cropped, template0pt5, cv2.TM_CCOEFF_NORMED)
        target_sites0pt5 = np.argwhere(res0pt5 > test_config["crosscorr_coeff"])

        # Combine the found target sites
        target_sites = np.concatenate([target_sites0, target_sites0pt5], 0)

        # If none above the threshold were found, ignore this target, we missed it
        if len(target_sites) == 0:
            center = None
        
        # If 1 or more site were found, get the 'correct' site
        else:
            # Filter the target sites for those less than test_config["max_dist"] from
            #   the center of the image
            crop_center = (np.array(img_cropped.shape) - (1, 1)) / 2
            dists = np.linalg.norm(target_sites - crop_center, axis=1)
            valid_sites = np.argwhere(dists < test_config["max_dist"])[:, 0]
            target_sites = np.array([target_sites[i, :] for i in valid_sites])

            # If none are within the required distance, ignore this target, we missed it
            if len(target_sites) == 0:
                center = None

            # If exactly one target site was found, that's the spot we think it is in
            elif len(target_sites) == 1:
                center = target_sites[0] + np.array(template0.shape[::-1]) / 2
            
            # If more than 1 target site was found, check if they are all for the same
            #   target. If they are, use the one with the largest correlation
            else:
                x_max = max(target_sites[:, 1])
                x_min = min(target_sites[:, 1])
                y_max = max(target_sites[:, 0])
                y_min = min(target_sites[:, 0])
                
                target_pad = test_config[tgt["target_type"] + "_pad"]
                if np.linalg.norm([[x_max - x_min], [y_max - y_min]]) < target_pad:
                    # Get the res with the largest correlation
                    res = res0 if res0.argmax() > res0pt5.argmax() else res0pt5
                    
                    # Get the position in that res of the largest correlation
                    top_left = np.unravel_index(res.argmax(), res.shape)[::-1]
                    
                    # Get center of the template (both templates have the same shape)
                    center = top_left + np.array(template0.shape[::-1]) / 2
                
                else:
                    center = None

        # If a valid target site was not found, there was no detection
        if center is None:
            img_targets.append(None)
        
        # If a valid target site was found, add it to img_targets
        else:
            # 'Center' is given in image crop coordinates. Need to offset that by the
            # top left corner of the bounding box to put it into image coordinates
            center += (bbox[0][0], bbox[0][1])
            img_targets.append({'target_type' : tgt['target_type'], 'center' : center})
    
    # Reorder the targets and image targets
    # The first n items of tgts corresponds to the first n items of img_targets 1:1
    #   where n is num_matches. After that, they are unmatched tgts
    #   (The length of img_targets will always be equal to or less the length of tgts
    #   due to the way this routine is set up)
    tgts_matched, img_targets_matched = [], []
    tgts_unmatched = []
    for tgt, img_target in zip(visible_tgts, img_targets):
        if img_target is None:
            tgts_unmatched.append(tgt)
        else:
            tgts_matched.append(tgt)
            img_targets_matched.append(img_target)

    num_matches = len(tgts_matched)
    tgts_detected = tgts_matched + tgts_unmatched

    return tgts_detected, img_targets_matched, num_matches 


def filter_dist_filter(
    rmat,
    tvec,
    cameraMatrix,
    distCoeffs,
    tgts,
    img_targets,
    num_matches,
    intra_dist,
    inter_dist
):
    """Filters the targets and image target that are too close

    Conditions Enforced:
    1) The projected location of each target has at most 1 img_target within a radius of
       inter_dist from it
    2) Each img_target has at most 1 projected target location within a radius of
       inter_dist from it
    3) The projected location of each target is seperated from the projected location of
       each other target by a distance of at least intra_dist
    4) Each img_target is seperated from each other img_target by a distance of at least
       intra_dist

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
    tgts_bi_filter : list
        Target positions (`tgts`) that have been bifiltered.
    img_targets_bi_filter : list
        Target positions in the image.
    num_matches_bi_filter : int

    Notes
    -----
    ``tgts_matched_bi_filter[i]`` is associated with ``img_targets_bi_filter[i]`` for i
    from 0 to `num_matches_bi_filter`.
    """

    def inter_filter(tgts, img_targets, num_matches):
        """ Helper function that enforces conditions 1 and 2
        """        
        # Project the points into the image
        tgt_projs = photogrammetry.project_targets(
            rmat, tvec, cameraMatrix, distCoeffs, tgts
        ) 
    
        # Get the indices of the matched targets whose projected image location has 
        #   exactly 1 img target within a distance of inter_dist
        targets_inter_pass = set()
        for i in range(num_matches):
            inter_target_key = True
            for j in range(len(img_targets)):
                if i == j:
                    continue

                dist = np.linalg.norm(tgt_projs[i]["proj"] - img_targets[j]["center"])

                # If the distance is less than inter_dist, this target fails the filter
                if dist < inter_dist:
                    inter_target_key = False
                    break

            # If this target passed the filter, add it to the pass lists
            if inter_target_key:
                targets_inter_pass.add(i)

        # Get the indices of the matched img_targets that have exactly 1 target with a
        #   projected image location within a distance of inter_dist
        targets_intra_pass = set()
        for i in range(num_matches):
            intra_target_key = True
            for j in range(len(tgt_projs)):
                if i == j:
                    continue

                dist = np.linalg.norm(img_targets[i]["center"] - tgt_projs[j]["proj"])

                # If the distance is less than intra_dist, this target fails the filter
                if dist < intra_dist:
                    intra_target_key = False
                    break

            # If this target passed the filter, add it to the pass lists
            if intra_target_key:
                targets_intra_pass.add(i)
        
        # Get the intersection of the two sets
        inter_pass = targets_inter_pass.intersection(targets_intra_pass)

        # Add the matched tgts and matched img_targets that pass the filter to new lists
        tgts_inter_filter, img_targets_inter_filter = [], []
        for i in range(num_matches):
            if i in inter_pass:
                tgts_inter_filter.append(tgts[i])
                img_targets_inter_filter.append(img_targets[i])
        
        # Add the tgts and img_targets that were not matched and/or did pass the filter.
        #   Since they have indices greater than the return value of num_matches, they
        #   are known to be not matched
        for i in range(max(len(tgts), len(img_targets))):
            if i not in inter_pass:
                if i < len(tgts):
                    tgts_inter_filter.append(tgts[i])
                if i < len(img_targets):
                    img_targets_inter_filter.append(img_targets[i])

        return tgts_inter_filter, img_targets_inter_filter, len(inter_pass)
    
    def intra_filter(tgts, img_targets, num_matches):
        """ Helper function that enforces conditions 3 and 4
        """           
        # Project the points into the image
        tgt_projs = photogrammetry.project_targets(
            rmat, tvec, cameraMatrix, distCoeffs, tgts
        )
        
        # Find the projected target locations that are too close together
        tgts_intra_pass = set()
        for i in range(len(tgts)):
            intra_target_key = True
            for j in range(len(tgts)):
                if i == j:
                    continue
                
                # Calculate the distance between point i and j
                dist = np.linalg.norm(
                    [
                        tgt_projs[i]["proj"][0] - tgt_projs[j]["proj"][0],
                        tgt_projs[i]["proj"][1] - tgt_projs[j]["proj"][1],
                    ]
                )

                # If the distance is less than intra_dist, this target fails the filter
                if dist < intra_dist:
                    intra_target_key = False
                    break
            
            if intra_target_key:
                tgts_intra_pass.add(i)

        # Find the projected target locations that are too close together
        img_targets_intra_pass = set()
        for i in range(len(img_targets)):
            intra_target_key = True
            for j in range(len(img_targets)):
                if i == j:
                    continue
                
                # Calculate the distance between point i and j
                dist = np.linalg.norm(
                    [
                        img_targets[i]["center"][0] - img_targets[j]["center"][0],
                        img_targets[i]["center"][1] - img_targets[j]["center"][1],
                    ]
                )

                # If the distance is less than intra_dist, this target fails the filter
                if dist < intra_dist:
                    intra_target_key = False
                    break
            
            if intra_target_key:
                img_targets_intra_pass.add(i)
        
        # Get the intersection of the two sets
        intra_pass = tgts_intra_pass.intersection(img_targets_intra_pass)

        # Add the matched tgts and matched img_targets that pass the filter to new lists
        tgts_inter_filter, img_targets_inter_filter = [], []
        for i in range(num_matches):
            if i in intra_pass:
                tgts_inter_filter.append(tgts[i])
                img_targets_inter_filter.append(img_targets[i])
        
        # Add the tgts and img_targets that were not matched and/or did pass the filter.
        #   Since they have indices greater than the return value of num_matches, they
        #   are known to be not matched
        for i in range(max(len(tgts), len(img_targets))):
            if i not in intra_pass:
                if i < len(tgts):
                    tgts_inter_filter.append(tgts[i])
                if i < len(img_targets):
                    img_targets_inter_filter.append(img_targets[i])

        return tgts_inter_filter, img_targets_inter_filter, len(intra_pass)

    # Perform the inter filter (enforce conditions 1 and 2)
    inter_filer_retval = inter_filter(tgts, img_targets, num_matches)
    tgts_inter, img_targets_inter, num_matched_inter = inter_filer_retval
    
    # Perform the intra filter (enforce conditions 3 and 4)
    filtered_retval = intra_filter(tgts_inter, img_targets_inter, num_matched_inter)
    tgts_filtered, img_targets_filtered, num_matched_filtered = filtered_retval

    return tgts_filtered, img_targets_filtered, num_matched_filtered


# TODO: Implement this function properly
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
        'target_type', 'norm'. 'tvec' has a :class:`numpy.ndarray` (3, 1) representing
        the position of the target relative to the model origin for its associated
        value. 'norm' has a :class:`numpy.ndarray` (3, 1) representing the normal vector
        of the target relative to the model origin for its associated value.
        'target_type' has a string representing the type of target (most commonly 'dot')
        for its associated value. ``tgts[i]`` is associated with ``img_targets[i]``
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


def filter_matches(
    rmat, tvec, cameraMatrix, distCoeffs, tgts, img_targets, num_matches, test_config
):
    tgts_filtered, img_targets_filtered, num_matched_filtered = filter_dist_filter(
        rmat,
        tvec,
        cameraMatrix,
        distCoeffs,
        tgts,
        img_targets,
        num_matches,
        test_config["max_dist"],
        test_config["min_dist"]
    )

    return tgts_filtered, img_targets_filtered, num_matched_filtered
