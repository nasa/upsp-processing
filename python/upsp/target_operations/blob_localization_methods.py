import numpy as np
import cv2
import os
import sys

from upsp.cam_cal_utils import img_utils


#---------------------------------------------------------------------------------------
# Generic Blob Detector Parameters


# Define the parameters for the Simple Blob Detectors for the target detection
# These need to be manually tuned per test
# Currently set up to have decent performance on both dots and kulites
params_all = cv2.SimpleBlobDetector_Params()

params_all.minThreshold = 0
params_all.maxThreshold = 225

# Cannot filter by these criteria
params_all.filterByCircularity = False
params_all.filterByInertia = False

# Filtering by area is tough, even for the larger dots since the aspect
#   ratio can be very low. The smallest dot is only ~3px in area
# Additionally, the size parameter for blob detection seems to be screwy
params_all.filterByArea = False

# We should be able to filter by convexity well, but this may be an artifact
#   of the assumption that the targets are perfect ellispes
params_all.filterByConvexity = True
params_all.minConvexity = 0.9

# We should be able to filter by color, but at this time it is not implemented
params_all.filterByColor = False

# Create the detector
detector_all = cv2.SimpleBlobDetector_create(params_all)

# Create the detectors
target_detectors = {'detector_all': detector_all}


def blob_func(detection_method, decide_method=None):
    """Returns a function that acts as a wrapper around openCV's blob detection function
    to act as a target center localizer

    Parameters
    ----------
    detection_method : string
        String must be a key in the global dict target_detectors. The key selects the
        blob detection parameters to use
    decide_method : {'biggest', 'smallest', 'first', 'last', 'random', None}, optional
        decision method if more than 1 blob is detected in the image. Biggest and
        smallest selects the largest and smallest (respectfully) based on the keypoint
        size parameter. First and last select the first and last (respectfully) based on
        the order returned by the blob detector. Random selects a random blob. None uses
        the default method, which is last

    Returns
    -------
    callable
        Blob detector localization function - Wrapper around a blob detector to act as a
        localization function. The function has the signature::

            func(img, target_type=None, return_keypoint=False) -> keypoint

        where ``img`` is a 2D image array containing one target, ``target_type`` does
        nothing, and ``return_keypoint`` specifies whether the keypoint itself is
        returned (``True``) or just the center position (x, y).
    """

    if (detection_method in list(target_detectors.keys())):
        detector = target_detectors[detection_method]
    else:
        print("Specified detector does not exist.")
        print("detector must be one of the following:", *list(target_detectors.keys()))
        quit()

    # If no method was given, use 'last' method
    if decide_method is None:
        decide_method = 'last'

    valid_methods = ['biggest', 'smallest', 'first', 'last', 'random']

    # Decide method must be in the valid_methods list
    assert (decide_method in valid_methods), "Error in blob_func. Input 'decide_method \
must be one of the following: 'biggest', 'smallest', 'first', 'last', \
or 'random'"

    # Blob Localization Function
    # return_keypoint is a flag to return the entire keypoint, and no confidence value
    def fit_blob(img, target_type=None, return_keypoint=False):
        """Wrapper around a blob detector to act as a localization function

        Parameters
        ----------
            img : np.ndarray, 2D, uint8 or uint16
                Image containing one target
            target_type : optional, default=None
                This input does nothing. Localizer functions need this input due to
                stanardization of how localizer functions should be implemented for
                uPSP to make them modular. However, the blob detector function does not
                need this input to function.
            return_keypoint : optional, boolean.
                If True returns openCV keypoint object. If False, returns only estimated
                target center location
        
        Returns
        ----------
        Tuple if return_keypoint is False, and keypoint is return_keypoint is True
            Tuple's first index is the center. The center is a length 2 tuple of floats.
            The first item of the center is the x coordiante and the second item is the
            y coordinate
        """
        # If it is not an 8 bit image, convert it to an 8 bit image to use
        if (img.dtype != np.uint8):
            img_8bit = img_utils.scale_image_max_inlier(img)
        else:
            img_8bit = img

        # Detect the keypoints in the image
        keypoints = detector.detect(img_8bit)
        
        # If there is exactly one blob found, make best_keypoint as the only keypoint
        if (len(keypoints) == 1):
            best_keypoint = keypoints[0]

        # If there were multiple found, use the decider method to get the best one
        elif (len(keypoints) > 1):
            # Select the biggest keypoint based on keypoint.size
            if (decide_method == 'biggest'):
                biggest_size = -np.inf
                biggest_keypoint = None
                for keypoint in keypoints:
                    if (keypoint.size > biggest_size):
                        biggest_size, biggest_keypoint = keypoint.size, keypoint
                best_keypoint = biggest_keypoint

            # Select the smallest keypoint based on keypoint.size
            elif (decide_method == 'smallest'):
                smallest_size = np.inf
                smallest_keypoint = None
                for keypoint in keypoints:
                    if (keypoint.size < smallest_size):
                        smallest_size, smallest_keypoint = keypoint.size, keypoint
                best_keypoint = smallest_keypoint

            # For first, chose the first keypoint in the list
            elif (decide_method == 'first'):
                best_keypoint = keypoints[0]

            # For last, chose the last keypoint in the list
            elif (decide_method == 'last'):
                best_keypoint = keypoints[-1]

            # Pick a random keypoint in the list
            elif (decide_method == 'random'):
                best_keypoint = np.random.choice(keypoints, 1)[0]

        # If there is no Keypoint, it will return None
        else:
            best_keypoint = None

        # If the return_keypoint flag is on, only return the keypoint, and
        #   return the whole keypoint
        if return_keypoint:
            return best_keypoint

        # If a keypoint was not found, return (-1, -1) as the center
        # Otherwise return the keypoint location
        if best_keypoint is None:
            center = (-1, -1)
            conf = -1
        else:
            center = best_keypoint.pt
            conf = best_keypoint.size

        return (center,)

    return fit_blob
