import numpy as np
import json
import os
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
import cv2

from upsp.cam_cal_utils import photogrammetry, parsers, visualization

debug_show_cal_bounds = True
meter2inch = 39.3701

# NOTE: This is implemented for Calib.io version v1.6.5a


def get_pts_inside_incal(
    rmat,
    tvec,
    cameraMatrix,
    distCoeffs,
    obj_pts,
    cal_area_is_safe=None,
    cal_vol_is_safe=None,
    critical_pt=None,
):
    """Returns `obj_pts` that are inside the well behaved region of the internal cal

    Performs 3 checks on each of the `obj_pts`. The first check and second check is that
    each obj_pt and its projection are within the safe volume and safe area as defined
    by cal_area_is_safe and cal_vol_is_safe. These checks are always performed. The
    third check is not always necessary, and is more involved.

    If the highest order (non-zero) radial distortion term of `distCoeffs` is negative,
    then at some real world distance from the camera optical axis, increasing the real
    world distance from the camera optical axis will decrease pixel distance of that
    point's projection. I.e. moving farther out of frame will move the point closer to
    the image center. This is not a bug, but is not an accurate model of the physical
    system. This means that some object points very far out of the field of view can
    be projected into the frame. These points have a 'negative derivative' of the
    projection curve. As the input (real world distance from camera optical axis)
    increases, the output (projected distance from the image center) decreases.

    The points need not be outside the field of view to have a negative derivative. So a
    simple FOV check is unfortunately insufficient. The third check is unnecessary if
    `cal_area_is_safe` and `cal_vol_is_safe` are well constructed. If they are not well
    constructed, the third check ensures the object points are in the well-behaved
    region of the internal calibration. For most cases, this means the object points
    are in the positive region of the internal calibration.

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
    obj_pts : np.ndarray, shape (n, 3), float
        List of 3d points. Points are relative to the object frame
    cal_area_is_safe : callable or None. Optional, default=None
        If ``None``, it is assumed all points are safe. If cal_area_is_safe is a
        callable, it must be an ``alpha_shape_is_safe`` function. It takes `img_pts` as
        an array, (shape ``(n, 2)``) of floats and returns an array-like object of
        booleans. ``return[i]`` of that function corresonds to ``img_pts[i]``. If
        ``return[i]`` is True, then ``img_pts[i]`` is within the safe internal
        calibration image area. If it is False, it is not within that safe area
    cal_area_is_safe : callable or None. Optional, default=None
        If ``None``, it is assumed all points are safe. If cal_area_is_safe is a
        callable, it must be an ``alpha_shape_is_safe`` function. It takes `obj_pts` as
        an array, (shape ``(n, 3)``) of floats and returns an array-like object of
        booleans. ``return[i]`` of that function corresonds to ``obj_pts[i]``. If
        ``return[i]`` is True, then ``obj_pts[i]`` is within the safe internal
        calibration image area. If it is False, it is not within that safe area
    critical_pt : str, optional
        Criteria for step 3. Must be one of:

        - If 'first' then `obj_pts` cannot have a homogeneous coordinate with a
          magnitude past the first maxima of the distortion curve. This is the most
          useful option for the majority of lenses.
        - If 'final', the magnitude must be less than the final maxima of the
          distortion curve, and the distortion curve must be decreasing after the final
          maxima. This is the most useful option for mustache or otherwise complex
          distortion, if the mustache distortion effect is accurate
        - If ``None``, step 3 will be omitted. If `cal_area_is_safe` and
          `cal_vol_is_safe` are available (and not dummy return True functions from
          :func:`incal_calibration_bounds_debug`), the step 3 check is likely
          unnecessary

    Returns
    -------
    obj_pts_safe : np.ndarray, shape (n,), bool
        Array of indices ``obj_pts[i]``. If i is in return, then ``obj_pts[i]`` is well
        behaved. Otherwise, ``obj_pts[i]`` is not well behaved
    
    See Also
    --------
    incal_calibration_bounds : creates ``alpha_shape_is_safe`` functions
    """
    assert critical_pt in ["first", "final", None]

    # Populate the cal area objects if they were not given
    if cal_area_is_safe is None:
        cal_area_is_safe = incal_calibration_bounds_debug()[0]
    
    if cal_vol_is_safe is None:
        cal_vol_is_safe = incal_calibration_bounds_debug()[1]

    # Define a list of booleans to denote if a variable is safe
    obj_pts_safe = np.full(len(obj_pts), True)

    # Step 1: Check that the points project to within the safe image area region
    img_pts = photogrammetry.project_3d_point(
        rmat, tvec, cameraMatrix, distCoeffs, obj_pts
    )
    obj_pts_safe *= cal_area_is_safe(img_pts)

    # Step 2: Check that the points are within the safe 3D volume
    pts_rel_cam = photogrammetry.transform_3d_point(rmat, tvec, obj_pts)
    obj_pts_safe *= cal_vol_is_safe(pts_rel_cam)

    # Step 3a: Check for shortcuts to determine if the full step 3 is necessary

    # First shortcut skips if step 3 is omitted
    if critical_pt is None:
        return np.argwhere(obj_pts_safe==True).flatten()

    # Second shortcut skips if all of the distortion coefficients are non-negative,
    #   since there is no maxima
    if (distCoeffs >= 0).all():
        return np.argwhere(obj_pts_safe==True).flatten()

    # Third shortcut skips if critical_pt == 'final' and the projection curve has a
    #   positive derivative as the magnitude of the homogeneous coordinates goes to inf
    # To determine that, we find the highest order, nonzero term. The highest order term
    #   for both the x projected position and y projected position is k3, then k2, then
    #   k1. After that, the highest order term for the x projected position is p2 and
    #   the highest order term for the y projected position is p1. Since step 2 checked
    #   for no distortion, if k3, k2, and k1 are zero then p1 or (or both) p2 must be
    #   nonzero. If the minimum of those 2 is positive, they are both positive. After p1
    #   and p2, The remainining terms are linear and a constant offset. Neither of which
    #   cause the issue step 3 addresses so neither need to be checked.
    highest_nonzero_term = (
        distCoeffs[0][4]
        if distCoeffs[0][4] != 0.0
        else distCoeffs[0][1]
        if distCoeffs[0][1] != 0.0
        else distCoeffs[0][0]
        if distCoeffs[0][0] != 0.0
        else min(p2, p1)
    )
    if (critical_pt == "final") and highest_nonzero_term > 0.0:
        return np.argwhere(obj_pts_safe==True).flatten()

    # Step 3b: Perform the full step 3

    # First step is to get the location of the points relative to the camera.
    obj_pts_rel_cam = photogrammetry.transform_3d_point(rmat, tvec, obj_pts)

    # Define the homogenous coordiantes
    x_homo_vals = (obj_pts_rel_cam[:, 0] / obj_pts_rel_cam[:, 2]).astype(complex)
    y_homo_vals = (obj_pts_rel_cam[:, 1] / obj_pts_rel_cam[:, 2]).astype(complex)

    # Define the distortion terms, and vectorize calculating of powers of x_homo_vals
    #   and y_homo_vals
    k1, k2, p1, p2, k3 = distCoeffs[0]
    x_homo_vals_2 = np.power(x_homo_vals, 2)
    y_homo_vals_2 = np.power(y_homo_vals, 2)
    x_homo_vals_4 = np.power(x_homo_vals, 4)
    y_homo_vals_4 = np.power(y_homo_vals, 4)
    x_homo_vals_6 = np.power(x_homo_vals, 6)
    y_homo_vals_6 = np.power(y_homo_vals, 6)

    # This step cannot be vectorized since the numpy roots function is numerical
    # Find the bounds on the x_homo coordinate to ensure it is closer than the
    #   inflection point of x_proj as a function of x_homo
    x_homo_min = np.full(x_homo_vals.shape, -np.inf)
    x_homo_max = np.full(x_homo_vals.shape, np.inf)
    for i in range(len(y_homo_vals)):
        # If obj_pts_safe[i] is False already, no need to calculate the roots
        if obj_pts_safe[i] == False:
            continue

        # Expanded projection function polynomial coefficients
        x_proj_coeffs = np.array(
            [
                k3,
                0,
                k2 + 3 * k3 * y_homo_vals_2[i],
                0,
                k1 + 2 * k2 * y_homo_vals_2[i] + 3 * k3 * y_homo_vals_4[i],
                3 * p2,
                1
                + k1 * y_homo_vals_2[i]
                + k2 * y_homo_vals_4[i]
                + k3 * y_homo_vals_6[i]
                + 2 * p1 * y_homo_vals[i],
                p2 * y_homo_vals_2[i],
            ]
        )

        # Projection function derivative polynomial coefficients
        x_proj_der_coeffs = np.polyder(x_proj_coeffs)

        # Find the root of the derivative
        roots = np.roots(x_proj_der_coeffs)

        # Get the real roots
        # Approximation of real[np.where(np.isreal(roots))]
        real_roots = np.real(roots[np.where(np.abs(np.imag(roots)) < 1e-10)])

        for real_root in real_roots:
            if critical_pt == "first":
                if real_root > 0.0:
                    x_homo_max[i] = np.minimum(x_homo_max[i], real_root)
                else:
                    x_homo_min[i] = np.maximum(x_homo_min[i], real_root)
            else:
                x_homo_min[i] = np.minimum(x_homo_min[i], real_root)
                x_homo_max[i] = np.maximum(x_homo_max[i], real_root)

    # Check that the x_homo values are within the bounds
    obj_pts_safe *= np.where(x_homo_vals > x_homo_min, True, False)
    obj_pts_safe *= np.where(x_homo_vals < x_homo_max, True, False)

    # Find the bounds on the y_homo coordinate to ensure it is closer than the
    #   inflection point of y_proj as a function of y_homo
    y_homo_min = np.full(y_homo_vals.shape, -np.inf)
    y_homo_max = np.full(y_homo_vals.shape, np.inf)
    for i in range(len(x_homo_vals)):
        # If obj_pts_safe[i] is False already, no need to calculate the roots
        if obj_pts_safe[i] == False:
            continue

        # Expanded projection function polynomial coefficients
        y_proj_coeffs = np.array(
            [
                k3,
                0,
                k2 + 3 * k3 * x_homo_vals_2[i],
                0,
                k1 + 2 * k2 * x_homo_vals_2[i] + 3 * k3 * x_homo_vals_4[i],
                3 * p1,
                1
                + k1 * x_homo_vals_2[i]
                + k2 * x_homo_vals_4[i]
                + k3 * x_homo_vals_6[i]
                + 2 * p2 * x_homo_vals[i],
                p1 * x_homo_vals_2[i],
            ]
        )

        # Projection function derivative polynomial coefficients
        y_proj_der_coeffs = np.polyder(y_proj_coeffs)

        # Find the root of the derivative
        roots = np.roots(y_proj_der_coeffs)

        # Get the real roots
        # Approximation of real[np.where(np.isreal(roots))]
        real_roots = np.real(roots[np.where(np.abs(np.imag(roots)) < 1e-10)])

        for real_root in real_roots:
            if critical_pt == "first":
                if real_root > 0.0:
                    y_homo_max[i] = np.minimum(y_homo_max[i], real_root)
                else:
                    y_homo_min[i] = np.maximum(y_homo_min[i], real_root)
            else:
                y_homo_min[i] = np.minimum(y_homo_min[i], real_root)
                y_homo_max[i] = np.maximum(y_homo_max[i], real_root)

    # Check that the x_homo values are within the bounds
    obj_pts_safe *= np.where(y_homo_vals > y_homo_min, True, False)
    obj_pts_safe *= np.where(y_homo_vals < y_homo_max, True, False)

    # Return the list of is_safe booleans
    return np.argwhere(obj_pts_safe==True).flatten()


def incal_from_calibio(calibio_path):
    """Returns the internal calibration values from the calib.io output json

    Parameters
    ----------
    calibio_path : str
        Path to the calib.io saved calibration json

    Returns
    -------
    img_size : np.ndarray, shape (2,)
        Image size (height, width)
    uPSP_cameraMatrix : np.ndarray, shape (3, 3)
        Camera matrix for uPSP applications (same as openCV ``cameraMatrix``, but cx and
        cy are vectors from image center to principal point rather than the principal
        point itself).
    distCoeffs : np.ndarray, shape (5, 1)
        openCV distortion coefficients
    """
    # Read the internal calibration data
    calibio_data = parsers.read_json(calibio_path)["calibration"]["cameras"][0][
        "model"
    ]["ptr_wrapper"]["data"]

    # Get the image size
    img_size = calibio_data["CameraModelCRT"]["CameraModelBase"]["imageSize"]
    img_size = np.array((img_size["height"], img_size["width"]))

    # Read the internal calibration parameters
    params = calibio_data["parameters"]

    # Parse the camera matrix
    cameraMatrix = np.array(
        [
            [params["f"]["val"], 0.0, params["cx"]["val"]],
            [0.0, params["f"]["val"], params["cy"]["val"]],
            [0.0, 0.0, 1.0],
        ]
    )
    uPSP_cameraMatrix = parsers.convert_cv2_cm_to_uPSP_cm(cameraMatrix, img_size)

    # Parse the distortion coefficients
    distCoeffs = np.array(
        [
            [
                params["k1"]["val"],
                params["k2"]["val"],
                params["p1"]["val"],
                params["p2"]["val"],
                params["k3"]["val"],
            ]
        ]
    )

    return img_size, uPSP_cameraMatrix, distCoeffs


def write_incal_from_calibio(calibio_path, camera_name, sensor_size, save_dir=None):
    """Writes the internal calibration to a json file

    Saves internal calibration as ``'{camera_name}.json'`` to `save_dir`

    Parameters
    ----------
    calibio_path : str
        Path to the calib.io saved calibration json
        (Calib.io: File > Save Calibration Project)
    camera_name : str
        Name of the camera
    sensor_size : np.ndarray, shape (2,) of floats
        Physical size of the image sensor in inches
    save_dir : str, optional
        Path of directory to save the internal calibration json file. If None, save_dir
        is set to the directory containing the Calib.io json

    """
    img_size, uPSP_cameraMatrix, distCoeffs = incal_from_calibio(calibio_path)
    sensor_size = np.array(sensor_size)

    incal = {
        "uPSP_cameraMatrix": uPSP_cameraMatrix.tolist(),
        "distCoeffs": distCoeffs.tolist(),
        "sensor_resolution": img_size.tolist(),
        "sensor_size": sensor_size.tolist(),
    }

    if save_dir is None:
        save_dir = os.basename(calibio_path)

    # Export the internal calibration as a json file
    cal_file = camera_name + ".json"
    with open(os.path.join(save_dir, cal_file), "w") as f:
        json.dump(incal, f)


def uncertainties_from_calibio(calibio_path):
    """Returns uncertainties for OpenCV terms

    Parameters
    ----------
    calibio_path : str
        Path to the calib.io saved calibration json

    Returns
    -------
    tuple
        Standard deviation of calibration terms: (focal length, principal point x,
        principal point y, k1, k2, p1, p2, k3)
    """

    covariance_data = parsers.read_json(calibio_path)["covariance"]["covarianceMatrix"]
    size = (covariance_data["size"]["height"], covariance_data["size"]["width"])
    covarianceMatrix = np.array(covariance_data["pixels"]).reshape(size)

    f_stddev = np.sqrt(covarianceMatrix[0][0])
    cx_stddev = np.sqrt(covarianceMatrix[2][2])
    cy_stddev = np.sqrt(covarianceMatrix[3][3])
    k1_stddev = np.sqrt(covarianceMatrix[4][4])
    k2_stddev = np.sqrt(covarianceMatrix[5][5])
    p1_stddev = np.sqrt(covarianceMatrix[10][10])
    p2_stddev = np.sqrt(covarianceMatrix[11][11])
    k3_stddev = np.sqrt(covarianceMatrix[6][6])

    return (
        f_stddev,
        cx_stddev,
        cy_stddev,
        k1_stddev,
        k2_stddev,
        p1_stddev,
        p2_stddev,
        k3_stddev,
    )


def incal_calibration_bounds(
    calibio_path, cal_vol_alpha, cal_area_alpha, dof=None, num_vol_figs=10
):
    """Creates objects that determines if a point is inside the calibration regions

    From the `calibio_path`, the 3D points from the calibration board are used to define
    the safe 3D volume. The 2D points of the image detections are used to define the
    safe image area. The area/volume are defined using an alpha shape (technically an
    alpha complex). An alpha shape is similar to a convex hull, but sets a limit on the
    distance between two vertices for them to be connected. The max distances for the
    volume and area is 1 / `cal_vol_alpha` and  1 / `cal_area_alpha` respectively.

    If the global variable `debug_show_cal_bounds` is True, this method also creates
    debug images.

    One debug image is created for the safe image area. That image is green for pixels
    inside the 'safe' region and red for pixels outside the safe region. The region is
    not discretized to pixels, so the image is a (good) approximation.

    Several debug images are created for the safe 3D volume. Each debug image is a slice
    of the 3D safe object. The slices are done at planes progressively farther from the
    camera (not spherical shells of constant distance from the camera). Points in the
    field of view (not accounting for lens distortion) are given. Points colored red are
    unsafe. Points colored green are safe.

    Generates the following images when `debug_show_cal_bounds` (global) is True:

        ``3d_cal_points_camera.png``
            A scatter plot of the internal calibration points as viewed from the camera
            position. X is the real world position relative to the camera horizontal
            axis. Z is the real world position relative to the camera optical axis.
        ``3d_cal_points_side.png``
            A scatter plot of the internal calibration points as viewed from a location
            to the side of the camera. X is the real world position relative to the
            camera horizontal axis. Y is the real world position relative to the camera
            vertical axis.
        ``image_area.png``
            The safe and unsafe locations in the image based on the internal calibration
            points and the cal_area_alpha parameter
        ``unsafe_volume_Z=*_inches.png``
            Where * is Z location of the planar slice in inches for the given image.
            These images are like a wedding cake stack of the 3D calibration volume.
            Each image is a slice of the volume at a given Z distance from the camera.
            The image shows the safe and unsafe locations in that planar slice.

    Parameters
    ----------
    calibio_path : str
        Path to the calib.io saved calibration json
    cal_vol_alpha : float
        Used to define the 3D volume alpha shape. Alpha parameter is 1 / `cal_vol_alpha`
    cal_area_alpha : float
        Used to define the 3D image area alpha shape. Alpha parameter is 1 /
        `cal_vol_alpha`
    dof : tuple, optional
        Depth of field. Optional, but must be given if global variable
        `debug_show_cal_bounds` is True since it is used to generate debug images. The
        first item of tuple is distance to the first slice of the 3D volume. The second
        item is distance to the last slice. The slices are done in planes (not in
        spherical shells of constant distance to the camera).
    num_vol_figs : int, optional
        Number of volume figures. Optional, but must be given if global variable
        `debug_show_cal_bounds` is True since it is used to generate debug images.  The
        debug images of the calibration volume slice the volume into planes at
        progressively farther distances. This input determines the number of slices

    Returns
    -------
    cal_area_is_safe : callable
        Function that takes image points in an array-like object and returns a
        corresponding array of bools indicating which points are safe.
    cal_vol_is_safe : callable
        Function that takes 3D points in an array-like object and returns a
        corresponding array of bools indicating which points are safe.
    """
    # If the debug_show_cal_bounds is True, dof is required
    assert (not debug_show_cal_bounds) or (debug_show_cal_bounds and dof is not None)

    calibio = parsers.read_json(calibio_path)

    # Read in the internal calibration from the file
    img_size, uPSP_cameraMatrix, distCoeffs = incal_from_calibio(calibio_path)
    cameraMatrix = parsers.convert_uPSP_cm_to_cv2_cm(uPSP_cameraMatrix, img_size)

    # Define the calibrated image area
    detections = calibio["detections"]
    img_pts = []
    for detection in detections:
        featurePoints = detection["featurePoints"]
        for pt in featurePoints:
            img_pts.append((pt["point"]["x"], pt["point"]["y"]))

    img_pts = np.array(img_pts)
    cal_area_is_safe = alpha_shape_is_safe(img_pts, cal_area_alpha)

    if debug_show_cal_bounds:
        pixels = np.array(list(np.ndindex(img_size[1], img_size[0])))
        pixel_bools = cal_area_is_safe(pixels)
        unsafe_points = np.array(pixels[np.where(~pixel_bools)[0]])
        safe_points = np.array(pixels[np.where(pixel_bools)[0]])

        plt.figure("area")
        ax = plt.gca()
        ax.scatter(unsafe_points[:, 0], unsafe_points[:, 1], s=1, c="r", label="Unsafe")
        ax.scatter(safe_points[:, 0], safe_points[:, 1], s=1, c="g", label="Safe")
        ax.scatter(
            img_pts[:, 0], img_pts[:, 1], s=0.25, c="b", label="Calibration Points"
        )
        ax.invert_yaxis()
        plt.legend(loc="upper left")
        plt.xlim(0, img_size[1])
        plt.ylim(img_size[0], 0)
        plt.xlabel("X (pixels)")
        plt.ylabel("Y (pixels)")
        plt.savefig("image_area.png")
        plt.close("area")

    # Get the calibration target positions relative to the board frame
    cal_pts_metric = []
    for cal_pt_metric in calibio["targets"][0]["objectPoints"]:
        cal_pts_metric.append(
            [cal_pt_metric["x"], cal_pt_metric["y"], cal_pt_metric["z"]]
        )
    cal_pts_metric = np.array(cal_pts_metric)
    cal_pts = cal_pts_metric * meter2inch

    # Get the locations of all 3D targets
    poses = calibio["calibration"]["poses"]
    cal_pts_rel_cam = []
    for pose in poses:
        transformation = pose["transform"]

        # Get the rotation matrix from the quaternions
        rvec = np.array(
            [
                transformation["rotation"]["rx"],
                transformation["rotation"]["ry"],
                transformation["rotation"]["rz"],
            ]
        )
        rmat, __ = cv2.Rodrigues(rvec)

        # Get the translation vector
        tvec_metric = np.array(
            [
                transformation["translation"]["x"],
                transformation["translation"]["y"],
                transformation["translation"]["z"],
            ]
        )
        tvec = (tvec_metric * meter2inch).reshape(3, 1)

        cal_pts_rel_cam += photogrammetry.transform_3d_point(
            rmat, tvec, cal_pts
        ).tolist()
    cal_pts_rel_cam = np.array(cal_pts_rel_cam)
    cal_vol_is_safe = alpha_shape_is_safe(cal_pts_rel_cam, cal_vol_alpha)

    if debug_show_cal_bounds:
        plt.figure("volume")
        ax = plt.axes(projection="3d")
        ax.scatter(
            cal_pts_rel_cam[:, 0],
            cal_pts_rel_cam[:, 1],
            cal_pts_rel_cam[:, 2],
            s=1,
            c="b",
            marker="o",
            depthshade=False,
        )
        ax.set_xlabel("X (inches)")
        ax.set_ylabel("Y (inches)")

        ax.view_init(elev=-90.0, azim=-90)
        visualization.axisEqual3D(ax)
        plt.savefig("3d_cal_points_ceiling.png")

        ax.set_ylabel("")
        ax.set_zlabel("Z (inches)")
        ax.view_init(elev=0.0, azim=-90)
        plt.savefig("3d_cal_points_camera.png")
        plt.close("volume")

    if debug_show_cal_bounds:
        pixel_extremes = [
            max(np.abs((pixels[:, 0] - cameraMatrix[0][2]))),
            max(np.abs((pixels[:, 1] - cameraMatrix[1][2]))),
        ]
        max_reach = max(pixel_extremes) / cameraMatrix[0][0] * dof[1]
        max_reach *= 1.1

        for z in np.linspace(dof[0], dof[1], num_vol_figs):
            xs = (pixels[:, 0] - cameraMatrix[0][2]) / cameraMatrix[0][0] * z
            ys = (pixels[:, 1] - cameraMatrix[1][2]) / cameraMatrix[0][0] * z
            zs = np.full(xs.shape, z)
            pts = np.array([xs, ys, zs]).T

            pt_bools = cal_vol_is_safe(pts)
            unsafe_points = np.array(pts[np.where(~pt_bools)[0]])
            safe_points = np.array(pts[np.where(pt_bools)[0]])

            plt.figure("volume")
            ax = plt.axes(projection="3d")
            ax.scatter(
                unsafe_points[:, 0],
                unsafe_points[:, 1],
                unsafe_points[:, 2],
                s=1,
                c="r",
                marker="o",
                depthshade=False,
                label="Unsafe",
            )
            ax.scatter(
                safe_points[:, 0],
                safe_points[:, 1],
                safe_points[:, 2],
                s=1,
                c="g",
                marker="o",
                depthshade=False,
                label="Safe",
            )
            ax.set_xlim(-max_reach, max_reach)
            ax.set_ylim(-max_reach, max_reach)
            ax.view_init(elev=-90.0, azim=-90)
            visualization.axisEqual3D(ax)

            digits = np.floor(np.log10(dof[1])).astype(np.int64) + 1
            fig_idx = "{:.2f}".format(z).rjust(digits + 3, "0")
            plt.xlabel("X (inches)")
            plt.ylabel("Y (inches)")
            plt.legend()
            plt.savefig("unsafe_volume_Z=" + fig_idx + "_inches.png")
            plt.close("volume")

    return cal_area_is_safe, cal_vol_is_safe


def incal_calibration_bounds_debug():
    """Returns debugging :func:`incal_calibration_bounds` objects

    The returned :func:`incal_calibration_bounds` functions True for every input point
    """

    def is_always_safe(pts):
        return np.full(pts.shape[0], True)

    return is_always_safe, is_always_safe


def alpha_shape_is_safe(pts, alpha):
    """Returns an alpha shape generated from points and the `alpha` parameter

    Returns an alpha shape constructed from the points and the given alpha parameters.
    An alpha shape is similar to a convex hull, but sets a maximum on the distance
    between two vertices. The distances is 1 / `alpha`. An alpha shape is a convex hull
    if the alpha parameter is 0.

    The shape may be more accurately referred to as an alpha complex (rather than alpha
    shape) since it has poligonal edges, but many sources use alpha shape to refer to
    both.

    Parameters
    ----------
    pts : np.ndarray, shape (n, k) of floats
        Calibration points to create the alpha shape from. ``n`` is the number of points
        ``k`` is the dimensionality of the points (2 for image points, 3 for real world
        points)
    alpha : float
        Alpha parameter of the alpha shape. Larger means more points are rejected. Must
        be non-negative. Negative values are clipped to 0.

    Returns
    -------
    is_safe : callable
        Function that accepts an array-like object (list, np.array, etc) of points and
        returns a list of booleans of the same length. ``return[i]`` corresponds to
        ``point[i]`` and True means that point is inside the alpha shape (False means it
        is not).
    """
    # For numerical stability, clip alpha to a very small value
    alpha = np.clip(alpha, 1e-30, None)

    # Get the Delaunay tessellation of the pts
    tess = Delaunay(pts)

    # Scipy Delaunay returns tessellation as incides, get tessellation as points
    tess_pts = np.take(pts, tess.simplices, axis=0)

    # Get the number of dimensions
    num_dims = tess.simplices.shape[1] - 1

    # Get radius of the circumsphere (circumcircle in 2D) for each of the tessellation
    # Equations for circumcircle and circumsphere radius from wolframalpha:
    #   https://mathworld.wolfram.com/Circumcircle.html
    #   https://mathworld.wolfram.com/Circumsphere.html
    ones = np.ones((tess_pts.shape[0], tess_pts.shape[1], 1))
    normsq = np.sum(tess_pts ** 2, axis=2)[:, :, None]
    a = np.concatenate((tess_pts, ones), axis=2)
    D = np.concatenate((normsq, a), axis=2)
    a_det = np.linalg.det(a)
    c_det = np.power(-1, num_dims + 1) * np.linalg.det(np.delete(D, -1, axis=2))

    det_sum = 0
    for k in range(num_dims):
        sign = np.power(-1, num_dims + k + 1)
        D_k = sign * np.linalg.det(np.delete(D, k + 1, axis=2))
        det_sum += np.power(D_k, 2)

    # Calculate the discriminant
    # Clip values that are negative due to machine precision
    disc = det_sum - 4 * a_det * c_det
    disc = np.where(
        (disc < 0.0) & (np.abs(disc) < np.finfo(np.float64).eps),
        np.finfo(np.float32).eps,
        disc,
    )

    # Calculate the denominator, but clip the abs value of a_det to avoid 0/0 issues
    # a_det is clipped 10 orders of magnitude larger than disc so 0/0 = 0
    #   That way 0/0 tessellations are rejected
    den = 2 * np.clip(np.abs(a_det), 1e-20, None)

    # Calculate the radius of each Delaunay tessellation
    tess_radii = np.divide(np.sqrt(disc), den)

    # Get a set of the indices of the tessellation that are a part of the alpha shape
    alpha_shape_tess = np.where(tess_radii < np.divide(1.0, alpha))[0]

    # Return a fuction that determines if a point is safe
    #   is_safe returns True if the point is inside the alpha shape boundary, and
    #   returns False if it is outside the alpha shape boundary
    def is_safe(pts):
        # Find the simplex that contains the point (if it exists)
        simplex_idx = tess.find_simplex(pts)

        # Check if the index is in the set of safe indexes
        #   If it is, return True. Otherwise return False
        return np.where(np.isin(simplex_idx, alpha_shape_tess), True, False)

    # Return the is_safe function
    return is_safe
