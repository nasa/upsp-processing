import numpy as np
import os
import json

from upsp.cam_cal_utils import (
    external_calibrate,
    photogrammetry,
    parsers,
    visualization,
)

debug_show_3D_targets = False
debug_show_img_targets = False
debug_show_matches = True


def camera_to_tunnel_calibrate(
    ctc_dir,
    imgs,  # Camera specific
    internal_cals,  # Camera specific
    manual_detections,  # Camera specific
    tunnel_vals,  # Datapoint specific
    tgts_all,  # Test specific
    test_config,  # Test specific
    vis_checker,  # Test specific
    match_thresh=0.8,  # Optional
):
    """Performs camera to tunnel calibration for the given cameras

    Generates ``camera_to_tunnel`` json files for each given camera and saved in
    ``ctc_dir``.

    Any number of cameras can be given to this function. Each camera needs its own entry
    in `imgs`, `internal_cals`, and `manual_detections`. If there are 10 cameras, each
    of those inputs will be a list of length 10.

    If `debug_show_matches` or `debug_show_img_targets` are True, the debug images will
    be saved to the current directory. Each image will be appended with '_?' where ? is
    the index of the camera (If ? is 0, it corresponds to ``imgs[0]``,
    ``internal_cals[0]``, and ``manual_detections[0]``)

    All inputs should correspond to the same test configuration. The inputs
    `tunnel_vals`, `tgts_all`, `test_config`, and `vis_checker` will be used across all
    cameras.

    Parameters
    ----------
    ctc_dir : string
        Directory to save the camera-to-tunnel calibrations
    imgs : list
        Each image should be ``np.array`` of shape (height, width) and 8-bit.
        ``imgs[i]`` should correspond to ``internal_cals[i]`` and
        ``manual_detections[i]``
    internal_cals : list
        Each internal calibration should be of the form::

            [cameraMatrix, distCoeffs, sensor_resolution, sensor_size]

        - ``cameraMatrix`` is the (openCV formatted) camera matrix for the camera
        - ``distCoeffs`` is the (openCV formatted) distortion coefficients for the
          camera
        - ``sensor_resolution`` is a tuple of the full pixel resolution of the camera
          (which can be larger than the images of the `imgs` input)
        - ``sensor_size`` is a tuple of the physical sensor size in inches

        ``internal_cals[i]`` should correspond to ``imgs[i]`` and
        ``manual_detections[i]``
    manual_detections : list
        Each manual detection using PASCAL VOC format. Each manual detection is a
        dict with following the keys:

            - 'class' denoting the target_type
            - 'x1' denoting the left edge of the bounding box
            - 'y1' denoting the top edge of the bounding box
            - 'x2' denoting the right edge of the bounding box
            - 'y2' denoting the bottom edge of the bounding box

        ``manual_detections[i]`` should correspond to ``imgs[i]`` and
        ``internal_cals[i]``
    tunnel_vals : dict
        `tunnel_vals` has the keys ALPHA, BETA, PHI, and STRUTZ which denote the model's
        commanded position in the UPWT. For tests without this type of model positioning
        mechanism, set all values to 0.0
    tgts_all : list
        Each target is a dict with (at a minimum) a 'target_type', 'tvec', and 'norm'
        attribute.  The 'target_type' is a string for the type of target, usually 'dot'.
        Currently, only targets with the 'dot' type are used. The 'tvec' attribute gives
        the target's location and the 'norm' attribute gives the target's normal vector
        Both are ``np.array`` vectors with shape (3, 1)
    test_config : dict
        The dict must contain the following keys and values:
            - 'oblique_angle' : maximum allowable oblique viewing angle
            - 'max_match_dist' : maximum allowable matching distance between tgt
              projection and image target
            - 'dot_pad' : pixel padding distance around center of dot target to use for
              sub-pixel localization
            - 'tunnel-cor_to_tgts_tvec' : translation vector from center of rotation to
              tgts origin. For tests that do not have a center of rotation like the
              UPWT, set this value to [0.0, 0.0, 0.0]
            - 'tunnel-cor_to_tgts_rmat' : rotation matrix from center of rotation to
              tgts origin. For tests that do not have a center of rotation like the
              UPWT, set this value to [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
            - 'tunnel-cor_to_tunnel-origin_tvec' : translation vector from center of
              rotation to tunnel origin. This can be [0.0, 0.0, 0.0]
            - 'tunnel-cor_to_tunnel-origin_rmat' : rotation matrix from center of
              rotation to tunnel origin. This can be [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    vis_checker : ~upsp.cam_cal_utils.visibility.VisibilityChecker
        Visibility checker object with the relevant BVH and oblique viewing angle
    match_thresh : int or float, default = 0.80
        Proportion of matched needed to form a consensus, optional default=0.8
    """
    # Make the camera-to-tunnel directory if it does not exist
    os.makedirs(ctc_dir, exist_ok=True)

    # For this optimization, only take only the dots from the target inputs if possible
    target_type = "dot"
    tgts = []
    for tgt in tgts_all:
        if tgt["target_type"] == "dot":
            tgts.append(tgt)

    # If there were not enough dots, use kulites
    if len(tgts) < 4:
        # Set the dot_found flag to False
        target_type = "kulite"
        for tgt in tgts_all:
            if tgt["target_type"] == "kulite":
                tgts.append(tgt)

    if debug_show_3D_targets:
        import matplotlib.pyplot as plt
        import target_bumping

        fig = plt.figure("3D Targets")
        ax = fig.add_subplot(projection="3d")

        tvecs, norms = vis_checker.get_tvecs_and_norms()
        ax.scatter(tvecs[:, 0], tvecs[:, 1], tvecs[:, 2])

        internals = target_bumping.tgts_get_internals(tgts, vis_checker)
        internal_names = [internal[0] for internal in internals]
        external_tgts = []
        for tgt in tgts:
            if tgt["name"] not in internal_names:
                external_tgts.append(tgt)
        visualization.plot_pts_and_norms(external_tgts, ax)

        ax.view_init(elev=45, azim=-90)
        visualization.axisEqual3D(ax)
        plt.savefig("Targets_3D.png")
        plt.close("3D Targets")

    # Get transformation from tunnel frame to tgts frame
    rmat_tunnel_tgts, tvec_tunnel_tgts = tunnel_transform(
        **tunnel_vals, tvec__cor_tgts__tgts_frame=test_config["tunnel-cor_to_tgts_tvec"]
    )
    rmat_tgts_tunnel, tvec_tgts_tunnel = photogrammetry.invTransform(
        rmat_tunnel_tgts, tvec_tunnel_tgts
    )

    # Run the camera-tunnel calibration for each camera
    for num, img, internal_cal, manual_detection in zip(
        list(range(1, len(imgs) + 1)), imgs, internal_cals, manual_detections
    ):
        # Unpackage the internal calibration
        cameraMatrix, distCoeffs, sensor_resolution, sensor_size = internal_cal

        # Get the sub-pixel localized imag_targets
        img_targets = []
        for bbox in manual_detection:
            # If it is a target of the appropriate class, and not flagged as difficult
            #   add it to the targets list
            if (bbox["class"] == target_type) and not bbox["difficult"]:
                x1, y1, x2, y2 = bbox["x1"], bbox["y1"], bbox["x2"], bbox["y2"]
                center = ((x1 + x2) / 2, (y1 + y2) / 2)
                img_target = {"target_type": target_type, "center": center}
                img_targets.append(img_target)

        # Sub-pixel localize the manual img_targets
        __, img_targets = external_calibrate.subpixel_localize(
            img, img_targets, img_targets, test_config
        )

        # If debug_show_img_targets is turned on, generate the debug image
        if debug_show_img_targets:
            img_centers = np.array([img_target["center"] for img_target in img_targets])
            img_centers = np.squeeze(img_centers)
            visualization.show_image_locations(
                img, img_centers, str(num) + "_Image_Center_Locations"
            )

        # Using RANSAC, find the external calibration
        rmat_opt, tvec_opt = external_calibrate.external_calibrate_RANSAC(
            [cameraMatrix, distCoeffs],
            tgts,
            img_targets,
            vis_checker,
            max_dist=test_config["max_dist"],
            match_thresh=match_thresh,
        )

        # If debug_show_matches is turned on, generate the debug image
        if debug_show_matches:
            visible_init_tgts = photogrammetry.get_visible_targets(
                rmat_opt, tvec_opt, tgts, vis_checker
            )
            external_calibrate.match_targets(
                rmat_opt,
                tvec_opt,
                cameraMatrix,
                distCoeffs,
                visible_init_tgts,
                img_targets,
                max_dist=test_config["max_dist"],
                debug=[img, str(num), None],
            )

        # Transform from camera to tgts to tunnel -> Get camera to tunnel transformation
        rmat_camera_tunnel = np.matmul(rmat_opt, rmat_tgts_tunnel)
        tvec_camera_tunnel = tvec_opt + np.matmul(rmat_opt, tvec_tgts_tunnel)

        # Package the camera calibration data
        uPSP_cameraMatrix = parsers.convert_cv2_cm_to_uPSP_cm(cameraMatrix, img.shape)
        datum = {
            "uPSP_cameraMatrix": uPSP_cameraMatrix.tolist(),
            "distCoeffs": distCoeffs.tolist(),
            "rmat": rmat_camera_tunnel.tolist(),
            "tvec": tvec_camera_tunnel.reshape(3,).tolist(),
            "sensor_resolution": sensor_resolution.tolist(),
            "sensor_size": sensor_size.tolist(),
        }

        # Export the calibration as a json file
        cal_file = "camera" + str(num).rjust(2, "0") + ".json"
        with open(os.path.join(ctc_dir, cal_file), "w") as f:
            json.dump(datum, f)


# TODO: need to implement use of rmat__cor_tgts__tgts_frame as an input
def tunnel_transform(ALPHA, BETA, PHI, STRUTZ, tvec__cor_tgts__tgts_frame):
    """Calculates the transformation from the tunnel coordinate frame to the tgts frame

    Note: This is not necessarily to the tunnel origin, just some fixed point in the
    tunnel (fixed within a tunnel test). If `STRUTZ` = `STRUTZ_abs` then it will be the
    tunnel origin

    Parameters
    ----------
    ALPHA : float
        Tunnel alpha in degrees
    BETA : float
        Tunnel beta in degrees
    PHI : float
        Tunnel phi in degrees
    STRUTZ : float
        STRUTZ location of the UPWT strut
    tvec__cor_tgts__tgts_frame : np.ndarray, shape (3, 1), dtype float
        Translation vector from the tunnel center of rotation to the tgts frame
        in the tgts frame. The tgts frame is a fixed distance from the tunnel point of
        rotation from the tgts frame's point of view, that translation vector is always
        along the x axis

    Returns
    ----------
    rotation_matrix : np.ndarray, shape (3, 3)
        Rotation matrix from tgts frame to tunnel frame
    tvec__tunnel_tgts__tunnel_frame : np.ndarray, shape (3, 1)
        Translation vector from tgts frame to tunnel frame
    """

    # Get the component rotation matrices

    # UPWT Tunnel Coordinates are RHS Aircraft Coordinates Pitched 180 degrees
    # See UPWT AIAA Coordinate Systems Training Manual For Details

    # Positive Alpha is Positive Pitch
    pitch = photogrammetry.rot(-ALPHA, "y")

    # Positive Beta is Negative Yaw
    yaw = photogrammetry.rot(-BETA, "z")

    # Positive Phi is Positive Roll
    roll = photogrammetry.rot(PHI, "x")

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
    tvec__tunnel_tgts__tunnel_frame = tvec__knuckle_tgts + np.array(
        [[0], [0], [STRUTZ]]
    )

    return rotation_matrix, tvec__tunnel_tgts__tunnel_frame


# TODO: This refers specifically to a model on the sting. We will need a function for
#   a floor mounted model, and ideally something in the test_config file to specify
# TODO: need to implement use of tunnel-cor_to_tgts_rmat from test_config
def tf_camera_tgts_thru_tunnel(camera_tunnel_cal, wtd, test_config):
    """Returns the transformation from the camera to the model (tgts frame)

    Parameters
    ----------
    camera_cal : list
        camera calibration in the form::

            [rmat__camera_tunnel, tvec__camera_tunnel, cameraMatrix, distCoeffs]
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
    rmat__camera_tgts : np.ndarray, shape (3, 3)
        Rotation matrix
    tvec__camera_tgts : np.ndarray, shape (3, 1)
        Translation vector
    """

    # Turn the wind tunnel data into the transformation from tunnel to targets
    wtd_transform = tunnel_transform(
        wtd["ALPHA"],
        wtd["BETA"],
        wtd["PHI"],
        wtd["STRUTZ"],
        test_config["tunnel-cor_to_tgts_tvec"],
    )
    rmat_tunnel_tgts, tvec_tunnel_tgts = wtd_transform

    # Transformation from tgts frame to tunnel frame
    rmat_tgts_tunnel = np.linalg.inv(rmat_tunnel_tgts)
    tvec_tgts_tunnel = -np.matmul(rmat_tgts_tunnel, tvec_tunnel_tgts)

    # Decompose the camera calibration into its parts
    (
        rmat__camera_tunnel,
        tvec__camera_tunnel,
        cameraMatrix,
        distCoeffs,
    ) = camera_tunnel_cal

    # Combine the transformations to get the transformation from `camera to tgts frame
    rmat__camera_tgts = np.matmul(rmat__camera_tunnel, np.linalg.inv(rmat_tgts_tunnel))
    tvec__camera_tgts = tvec__camera_tunnel + np.matmul(
        rmat__camera_tunnel, tvec_tunnel_tgts
    )

    return rmat__camera_tgts, tvec__camera_tgts
