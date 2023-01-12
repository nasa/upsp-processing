import xml.etree.ElementTree as ET
import json
import csv
import numpy as np


def read_tgts(tgts_file_path, output_target_types=None):
    """Returns the targets in the tgts file

    Parameters
    ----------
    tgts_file_path : str
        File path to the tgts file
    output_target_types : str, list of str, optional
        If not None, only targets with a type of or in `output_target_types` will be
        read. If None, all target types will be read

    Returns
    -------
    targets : list of dict
        List of targets. Each target is of the form::

            {
                'target_type' : class_string, 'tvec' : [x, y, z], 'norm', : [x, y, z],
                'size': float, 'name': str, 'idx': int, 'zones': [i, j, k]
            }
    """
    # Package output_target_types
    if (output_target_types is not None) and (type(output_target_types) is not list):
        output_target_types = [output_target_types]

    targets = []
    with open(tgts_file_path, "r") as f:
        csv_reader = csv.reader(f, delimiter=" ")
        line_type = None
        for row in csv_reader:
            # Populate the line from the csv
            line = []
            for item in row:
                if item != "":
                    line.append(item)

            # If the length of the line is greater than 1, attempt to populate the
            #   the list of targets
            if len(line) > 1:
                # Read in items listed under '*Targets'
                if line_type == "*Targets":
                    # If the last element has 'st' for 'sharpie target' it is a dot
                    if "st" in line[-1]:
                        target_type = "dot"

                    # If the last element has 'mK' for 'masked Kulite' it is a Kulite
                    #   (and visible to the camera)
                    elif "mK" in line[-1]:
                        target_type = "kulite"

                    # If the last element has 'pK' for 'painted Kulite' it is a Kulite
                    #   (and not visible to the camera)
                    elif "pK" in line[-1]:
                        target_type = "painted_kulite"

                    # Otherwise this item is unknown
                    else:
                        target_type = line[-1]

                # Ignore items in other categories of the tgts file
                else:
                    continue

                # If output_target_types was not given, or the target_type is one of the
                #   output_target_types given, grab this target
                if (output_target_types is None) or (
                    target_type in output_target_types
                ):
                    targets.append(
                        {
                            "target_type": target_type,
                            "tvec": np.expand_dims([float(x) for x in line[1:4]], 1),
                            "norm": np.expand_dims([float(x) for x in line[4:7]], 1),
                            "size": float(line[7]),
                            "name": line[-1],
                            "idx": int(line[0]),
                            "zones": (int(line[8]), int(line[9]), int(line[10])),
                        }
                    )

            # If the length of the line is 1 or 0, set the line_type to the line's item
            #   if it has one, or set it to None if it has no items
            else:
                line_type = line[0] if len(line) == 1 else None

    return targets


def read_pascal_voc(annot_path):
    """Return image objects from a PASCAL VOC XML file

    Reads the input file(s) and returns an list of dicts, where each dict contains the
    class, bounding box, and flags of an image object.

    Parameters
    ----------
    annot_path : list or str
        This can be a string, or a list of strings (or list-like). Each string is the
        path to a label

    Returns
    -------
    out : dict or list
        If `annot_path` is a single label path, the output is a list of dicts. If
        `annot_path` is a list of label paths (or list-like), the output is a list of
        lists, where each inner list if a list of dicts.
        Each dict is represents an image object and has the keys 'class', 'x1',
        'x2', 'y1', 'y2', 'difficult', and 'truncated'.
    """

    # Assert that annot_path is of the right form
    error_msg = "Error in read_pascal_voc. Input 'annot_path' should be a filepath\
    string or list (or list-like) of filepath strings."
    assert type(annot_path) is str or type(annot_path[0]) is str, error_msg

    # Check if annot_path is a string. If it is not, it should be a list (or list-like)
    if type(annot_path) is not str:
        output = []
        for path in annot_path:
            output.append(read_pascal_voc(path))
        return output

    # Converts the .xml into a tree file
    et = ET.parse(annot_path)
    element = et.getroot()

    # Parses PASCAL VOC data into python objects
    element_objs = element.findall("object")

    # Populate the annotation_data with the filepath, and image
    #   width & height
    targets = []

    for element_obj in element_objs:
        # Populate the class_count and class_mapping return variables
        class_name = element_obj.find("name").text

        # Populate bounding box information
        obj_bbox = element_obj.find("bndbox")
        x1 = int(round(float(obj_bbox.find("xmin").text)))
        y1 = int(round(float(obj_bbox.find("ymin").text)))
        x2 = int(round(float(obj_bbox.find("xmax").text)))
        y2 = int(round(float(obj_bbox.find("ymax").text)))
        difficulty = int(element_obj.find("difficult").text)
        truncated = int(element_obj.find("truncated").text)

        #   LabelImg used the same coordinate system as OpenCV
        #   LabelImg clips x1 and y1 to 1, even if the true value is 0
        #   For future work, that should be fixed in the annotations.
        if truncated:
            if x1 == 1:
                x1 = 0
            if y1 == 1:
                y1 = 0

        # Populate annotation_data's bounding box field
        targets.append(
            {
                "class": class_name,
                "x1": x1,
                "x2": x2,
                "y1": y1,
                "y2": y2,
                "difficult": difficulty,
                "truncated": truncated,
            }
        )

    return targets


def read_wind_tunnel_data(wtd_file_path, items=("ALPHA", "BETA", "PHI", "STRUTZ")):
    """Read specified wind tunnel data from `wtd_file_path`

    Parameters
    ----------
    wtd_file_path : str
        Filepath to wind tunnel data file
    items : container, optional
        Items requested from the file. By default ALPHA, BETA, PHI and STRUTZ are
        returned.

    Returns
    -------
    tunnel_vals : dict
        Dictionary with keys of specified items and values of associated wtd values.
    """

    # Read in the wind tunel data file
    with open(wtd_file_path, "r") as f:
        f.readline()
        csv_reader = csv.DictReader(f, delimiter="\t")
        tunnel_vals = next(csv_reader)

    # Convert values to floats
    tunnel_vals = {k: float(v) for k, v in tunnel_vals.items()}

    # Remove all but the relevant
    for k in list(tunnel_vals.keys()):
        if k not in items:
            del tunnel_vals[k]

    return tunnel_vals


def convert_cv2_cm_to_uPSP_cm(cameraMatrix, dims):
    """Converts a standard camera matrix to a uPSP camera matrix

    OpenCV (and the standard) cameraMatrix uses the absolute position of the optical
    principal point. Since uPSP often crops images, the absoulte position varies
    from configuration to configuration even if the optics haven't changed. So instead,
    the position of the principal point relative to the image center is saved.

    Parameters
    ----------
    cameraMatrix : np.ndarray, shape (3, 3)
        Camera matrix of the form: ``[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]``
        where f is the focal length and (cx, cy) is
        the absolute position of the optical principal point
    dims : tuple, length 2
        Image dimensions (width, height)

    Returns
    -------
    uPSP_cameraMatrix : np.ndarray, shape (3, 3)
        Converted camera matrix of the form: ``[[fx, 0, dcx], [0, fy, dcy], [0, 0, 1]]``
        cx = w/2 + dcx and cy = h/2 + dcy where w and h are the image width and height

    See Also
    --------
    convert_uPSP_cm_to_cv2_cm : inverse conversion
    """

    # Cx (Principal Point X)
    cx = cameraMatrix[0][2]

    # Delta Cx (Principal Point Y)
    cy = cameraMatrix[1][2]

    # Get offset from image center and principal point
    dcx = cx - (dims[1] / 2)
    dcy = cy - (dims[0] / 2)

    # Return a copy of the uPSP cameraMatrix with the modified values
    uPSP_cameraMatrix = np.copy(cameraMatrix)
    uPSP_cameraMatrix[0][2] = dcx
    uPSP_cameraMatrix[1][2] = dcy

    return uPSP_cameraMatrix


def convert_uPSP_cm_to_cv2_cm(uPSP_cameraMatrix, dims):
    """Converts a uPSP camera matrix to a standard camera matrix

    OpenCV (and the standard) cameraMatrix uses the absolute position of the optical
    principal point. Since uPSP often crops images, the absoulte position varies
    from configuration to configuration even if the optics haven't changed. So instead,
    the position of the principal point relative to the image center is saved. To use
    OpenCV functions, it has to be converted back to the standard OpenCV camera matrix.

    Parameters
    ----------
    uPSP_cameraMatrix : np.ndarray, shape (3, 3)
        Camera matrix of the form: ``[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]``
        where f is the focal length and (cx, cy) is
        the absolute position of the optical principal point
    dims : tuple, length 2
        Image dimensions (width, height)

    Returns
    -------
    cameraMatrix : np.ndarray, shape (3, 3)
        Converted camera matrix of the form: ``[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]``
        cx = w/2 + dcx and cy = h/2 + dcy where w and h are the image width and height

    See Also
    --------
    convert_uPSP_cm_to_cv2_cm : inverse conversion
    """

    # Delta Cx (Principal Point X)
    dcx = uPSP_cameraMatrix[0][2]

    # Delta Cx (Principal Point Y)
    dcy = uPSP_cameraMatrix[1][2]

    # Offset the image center by (dcx, dcy)
    image_center = (dims[1] / 2, dims[0] / 2)
    principal_point = (image_center[0] + dcx, image_center[1] + dcy)

    # Return a copy of the uPSP cameraMatrix with the modified values
    cameraMatrix = np.copy(uPSP_cameraMatrix)
    cameraMatrix[0][2] = principal_point[0]
    cameraMatrix[1][2] = principal_point[1]

    return cameraMatrix


def read_internal_params(internal_cal_path, dims, read_sensor=False):
    """ Returns the internal (intrinsic) camera parameters

    Parameters
    ----------
    internal_cal_dir : str
        Path to internal calibrations
    dims : tuple
        Image dimensions (image height, image width)
    read_sensor: bool, optional
        If True, additionally read and return the sensor resolution and size

    Returns
    -------
    cameraMatrix : np.ndarray, shape (3, 3)
        Camera matrix of the form ``[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]``
        where f is the focal length and (cx, cy)
        is the position of the optical principal point relative to the image center
    distCoeffs : np.ndarray, shape (5,)
        Distortion coefficients of the form ``[k1, k2, p1, p2, k3]``
        where k1, k2, and k3 are the radial distortion terms and p1 and p2 are the
        tangential distortion terms
    sensor_resolution : np.ndarray, shape (3,)
        Camera sensor resolution, only returned if `read_sensor` is True
    sensor_size : np.ndarray, shape (2,)
        Camera sensor physical size, only returned if `read_sensor` is True
    """
    # Read the internal calibration parameters
    with open(internal_cal_path, "r") as f:
        incal = json.load(f)
    uPSP_cameraMatrix = np.array(incal["uPSP_cameraMatrix"])
    distCoeffs = np.array(incal["distCoeffs"])

    # Convert the written uPSP_cameraMatrix to the OpenCV cameraMatrix
    cameraMatrix = convert_uPSP_cm_to_cv2_cm(uPSP_cameraMatrix, dims)

    # If read_sensor is False, just return the cameraMatrix and distCoeffs
    if not read_sensor:
        return cameraMatrix, distCoeffs

    # Otherwise return the sensor parameters as well
    else:
        sensor_resolution = np.array(incal["sensor_resolution"])
        sensor_size = np.array(incal["sensor_size"])

    return cameraMatrix, distCoeffs, sensor_resolution, sensor_size


def read_camera_tunnel_cal(cal_path, dims, read_sensor=False):
    """ Returns the internal (intrinsic) and external (extrinsic) camera calibration parameters

    Parameters
    ----------
    cal_path : str
        Path to camera calibration
    dims : tuple
        Image dimensions (image height, image width)
    read_sensor: bool, optional
        If True, additionally read and return the sensor resolution and size

    Returns
    -------
    cameraMatrix : np.ndarray, shape (3, 3)
        Camera matrix of the form ``[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]``
        where f is the focal length and (cx, cy)
        is the position of the optical principal point relative to the image center
    distCoeffs : np.ndarray, shape (5,)
        Distortion coefficients of the form ``[k1, k2, p1, p2, k3]``
        where k1, k2, and k3 are the radial distortion terms and p1 and p2 are the
        tangential distortion terms
    rmat : np.ndarray, shape (3, 3)
        Rotation matrix from camera to tunnel
    tvec : np.ndarray, shape (3,)
        Translation vector from camera to tunnel
    sensor_resolution : np.ndarray, shape (2,)
        Sensor size of camera in pixels, only returned if `read_sensor` is True
    sensor_size : np.ndarray, shape (2,)
        Sensor size of camera in inches, only returned if `read_sensor` is True
    """
    # Read the camera calibration parameters
    with open(cal_path, "r") as f:
        cal = json.load(f)
    uPSP_cameraMatrix = np.array(cal["uPSP_cameraMatrix"])
    distCoeffs = np.array(cal["distCoeffs"])
    tvec = np.array(cal["tvec"]).reshape(3, 1)
    rmat = np.array(cal["rmat"])

    # Convert the written uPSP_cameraMatrix to the OpenCV cameraMatrix
    cameraMatrix = convert_uPSP_cm_to_cv2_cm(uPSP_cameraMatrix, dims)

    # If read_sensor is False, just return the cameraMatrix and distCoeffs
    if not read_sensor:
        return rmat, tvec, cameraMatrix, distCoeffs

    # Otherwise return the sensor parameters as well
    else:
        sensor_resolution = np.array(cal["sensor_resolution"])
        sensor_size = np.array(cal["sensor_size"])

    return rmat, tvec, cameraMatrix, distCoeffs, sensor_resolution, sensor_size


def read_json(path):
    """Safely reads a json file and returns the associated dict

    Parameters
    ----------
    path : path-like
        File path to the tgts file

    Returns
    -------
    dict
        dict of json file items
    """
    # Safely read the json file
    with open(path, "r") as f:
        return json.load(f)


def read_test_config(path):
    """Safely reads a test config file and returns the associated dict with arrays

    Parameters
    ----------
    path : path-like
        File path to the tgts file

    Returns
    -------
    dict
        dict of json file items
    """
    # Safely read the json file
    with open(path, "r") as f:
        test_config = json.load(f)

        # Transcribe the test config into numpy arrays if possible
        test_config_np = {}
        for key, val in test_config.items():
            try:
                # If the length is 3, it is a (3, 1) or (3, 3)
                if len(val) == 3:
                    if type(val[0]) == float:
                        test_config_np[key] = np.expand_dims(val, 1)
                    elif len(val[0]) == 3:
                        test_config_np[key] = np.array(val)
                    else:
                        test_config_np[key] = val

                # If the length if not 3, it is a float or long array
                else:
                    test_config_np[key] = val

            # If an exception is raised, just add the actual value for the key
            except Exception as e:
                test_config_np[key] = val

    return test_config_np
