import xml.etree.ElementTree as ET
import json
import csv
import os
import numpy as np


def read_tgts(tgts_file_path, output_target_types=None, read_all_fields=False, read_pK=False):
    """
    Read in the tgts file at the specified tgts_file_path
    
    Inputs:
        tgts_file_path - file path to the tgts file
        output_target_types - Options. Can be given as a string or list of strings.
                              If given, only targets with a type in output_target_types
                              will be in the output
    Outputs:
        targets - list of targets. Each target is of the form
                  {'type' : class_string, 'tvec' : [x, y, z], 'norm', : [x, y, z]}
        
    """

    if (output_target_types is not None) and (type(output_target_types) is not list):
        output_target_types = [output_target_types]

    targets = []
    with open(tgts_file_path, 'r') as f:
        csv_reader = csv.reader(f, delimiter=' ')
        line_type = None
        for row in csv_reader:
            line = []
            for item in row:
                if (item != ''):
                    line.append(item)

            if (len(line) != 1):
                # target block contains all sharpie dots and masked kulites
                if (line_type == 'target'):
                    # If the last element has 'st' for 'sharpie target' it is a dot
                    if 'st' in line[-1]:
                        target_type = 'dot'
                    
                    # If the last element has 'mK' for 'masked Kulite' it is a Kulite
                    elif 'mK' in line[-1]:
                        target_type = 'kulite'
                    
                    elif (read_pK and 'pK' in line[-1]):
                        target_type = 'kulite'

                    # Otherwise this item is junk (unmasked Kulite, misc type, etc)
                    else:
                        continue

                # Otherwise the target type is unknown
                else:
                    continue
                
                # TODO: Read in tvec and norm as numpy arrays
                # If output_target_types was not given, or the target_type is one of the
                #   output_target_types given, grab this target
                if (output_target_types is None) or (target_type in output_target_types):
                    target = {'target_type' : target_type,
                              'tvec' : [float(x) for x in line[1:4]],
                              'norm' : [float(x) for x in line[4:7]],
                              'size' : float(line[7]),
                              'name' : line[-1]}
                
                    if read_all_fields:
                        target['idx'] = int(line[0])
                        target['zones'] = (int(line[8]), int(line[9]), int(line[10]))

                    targets.append(target)
                    
            else:
                if line[0] == '*Targets':
                    line_type = 'target'
                else:
                    line_type = None

    return targets


def read_pascal_voc(annot_path, read_full_annot=False):
    """
    read_pascal_voc inputs the path(s) to label(s) and returns an list of dicts. Where
        each dict contains the class, bounding box, and flags of an image object
    Input:
        annot_path - this can be a string, or a list of strings (or list-like). Each
                     string is the path to a label
    Output:
        If annot_path is a single label path, the output is a list of dicts
        If annot_path is a list of label paths (or list-like), the output is a list of
            lists. Each inner list if a list of dicts.
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
    element_objs = element.findall('object')

    # Read the annotation information if applicable
    if read_full_annot:
        annotation_data = {
            'filename': element.find('filename').text,
            'width': int(element.find('size').find('width').text),
            'height': int(element.find('size').find('height').text),
            'targets': []}
        
    # Populate the annotation_data with the filepath, and image
    #   width & height
    targets = []

    for element_obj in element_objs:
        # Populate the class_count and class_mapping return variables
        class_name = element_obj.find('name').text

        # Populate bounding box information
        obj_bbox = element_obj.find('bndbox')
        x1 = int(round(float(obj_bbox.find('xmin').text)))
        y1 = int(round(float(obj_bbox.find('ymin').text)))
        x2 = int(round(float(obj_bbox.find('xmax').text)))
        y2 = int(round(float(obj_bbox.find('ymax').text)))
        difficulty = int(element_obj.find('difficult').text)
        truncated = int(element_obj.find('truncated').text)

        #   LabelImg used the same coordinate system as OpenCV
        #   LabelImg clips x1 and y1 to 1, even if the true value is 0
        #   For future work, that should be fixed in the annotations.
        if truncated:
            if (x1 == 1):
                x1 = 0
            if (y1 == 1):
                y1 = 0
    
        # Populate annotation_data's bounding box field
        targets.append({'class': class_name,
                        'x1': x1, 'x2': x2, 'y1': y1, 'y2': y2,
                        'difficult': difficulty,
                        'truncated': truncated})

    # Return the annotation information if applicable
    if read_full_annot:
        annotation_data['targets'] = targets
        return annotation_data

    else:
        return targets


def read_wind_tunnel_data(wtd_file_path, items=['ALPHA', 'BETA', 'PHI', 'STRUTZ']):
    """
    Returns specified wind tunnel data from wtd_file_path

    Inputs:
        wtd_file_path - filepath to wind tunnel data file
        items - items requested from that file. By default ALPHA, BETA, PHI and STRUTZ
            are returned

    Output:
        dictionary with keys of specified items and values of associated wtd values
    """

    # Read in the wind tunel data file
    with open(wtd_file_path, 'r') as f:
        f.readline()
        csv_reader = csv.DictReader(f, delimiter='\t')
        tunnel_vals = next(csv_reader)

    # Convert values to floats
    tunnel_vals = {k : float(v) for k, v in tunnel_vals.items()}

    # Remove all but the relevant
    for k in list(tunnel_vals.keys()):
        if k not in items:
            del tunnel_vals[k]

    return tunnel_vals


def convert_cv2_cm_to_uPSP_cm(cameraMatrix, dims):
    """
    # Converts standard camera matrix to uPSP 

    Inputs:
        cameraMatrix - [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]        
        dims - image dimensions

    Output:
        uPSP_cameraMatrix - [[fx, 0, dcx], [0, fy, dcy], [0, 0, 1]]

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
    """
    The standard method of storing camera intrinsic parameters is by saving the absolute
        position of the principal point. This is the case because in the standard use
        case the image used from a camera has a constant resolution
    uPSP uses variable resolution to save storage/memory when possible. The resolution
        change comes from a center from of the image center. Therefore, instead of the
        absolute position of the principal point reamining constant, the offset from
        the image center remains constant
    This function converts between the uPSP offset and the industry standard absolute
        position
    
    Inputs:
        uPSP_cameraMatrix - [[fx, 0, dcx], [0, fy, dcy], [0, 0, 1]]
        dims - image dimensions

    Output:
        cameraMatrix - [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
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


def read_internal_params(camera, internal_cal_dir, dims, read_sensor=False):
    """
    Given the internal calibraiton directory and camera number, returns the internal
        (intrinsic) camera parameters

    Input:
        Camera - camera number (int or str)
        internal_cal_dir - path to internal calibrations
        dims - (image height, image width)

    Output:
        [cameraMatrix, distCoeffs] - intrinsics in openCV format
            cameraMatrix - [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
            distCoeffs - [k1, k2, p1, p2, k3]
    """

    camera = str(camera)
    with open(os.path.join(internal_cal_dir, 'camera0' + camera + '.json'), 'r') as f:
        incal = json.load(f)
    uPSP_cameraMatrix = np.array(incal['uPSP_cameraMatrix'])
    distCoeffs = np.array(incal['distCoeffs'])

    cameraMatrix = convert_uPSP_cm_to_cv2_cm(uPSP_cameraMatrix, dims)

    # If read_sensor is False, just return the cameraMatrix and distCoeffs
    if not read_sensor:
        return cameraMatrix, distCoeffs
    
    # Otherwise return the sensor parameters as well
    else:
        sensor_resolution = np.array(incal['sensor_resolution'])
        sensor_size = np.array(incal['sensor_size'])
    return cameraMatrix, distCoeffs, sensor_resolution, sensor_size


def read_camera_params(camera, cal_dir, dims, read_sensor=False):
    """
    Given calibration directory and camera number, returns the camera calibration
        parameters

    Input:
        Camera - camera number (int or str)
        cal_dir - path to camera calibrations
        dims - (image height, image width)

    Output:
        [cameraMatrix, distCoeffs, rmat, tvec] - calibration parameters in openCV format
            cameraMatrix - [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
            distCoeffs - [k1, k2, p1, p2, k3]
            rmat - (3x3) rotation matrix from camera to tunnel
            tvec - (3,) translation vector from camera to tunnel
    """

    camera = str(camera).rjust(2, '0')
    with open(os.path.join(cal_dir, 'camera' + camera + '.json'), 'r') as f:
        cal = json.load(f)

    uPSP_cameraMatrix = np.array(cal['uPSP_cameraMatrix'])
    cameraMatrix = convert_uPSP_cm_to_cv2_cm(uPSP_cameraMatrix, dims)

    distCoeffs = np.array(cal['distCoeffs'])
    tvec = np.array(cal['tvec'])
    rmat = np.array(cal['rmat'])

    # If read_sensor is False, just return the cameraMatrix and distCoeffs
    if not read_sensor:
        return cameraMatrix, distCoeffs, rmat, tvec
    
    # Otherwise return the sensor parameters as well
    else:
        sensor_resolution = np.array(cal['sensor_resolution'])
        sensor_size = np.array(cal['sensor_size'])
    return cameraMatrix, distCoeffs, rmat, tvec, sensor_resolution, sensor_size

# TODO: Convert this into a read proc-default.json
def read_test_config(test_config_path):
    with open(test_config_path, 'r') as f:
        return json.load(f)
