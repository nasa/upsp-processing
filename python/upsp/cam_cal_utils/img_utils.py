import numpy as np
import scipy.stats

def convert_12bit_to_8bit(img):
    return scale_image(img, scale=(2**12) - 1)


def scale_image(img, scale=(2**12) - 1):
    """
    Proportionally scales image values between 0 and scale to 0 to 255
        I.e. If scale is 50, all pixel values are multiplied by 5.1 (255/50) then
        rounded to an integer
    Returns unsigned, 8 bit scaled version of input image
    """

    # Ensure the image is scaled properly
    img_temp = np.minimum(img, scale)

    # Normalize the image
    img_temp = img_temp.astype(np.float64)
    img_temp = img_temp / scale

    # Convert to an 8 bit image
    img_temp = img_temp * ((2**8) - 1)
    img_temp = np.rint(img_temp).astype(np.uint8)

    return img_temp


def scale_image_max_inlier(img):
    """
    Convert from 12 bit image to 8 bit image by scaling by the max inlier image value
    """

    # Get the maximum value, ignoring outliers
    img_flat = img.flatten()
    img_flat.sort()

    # Find the maximum, inlier value
    #   Considered an inlier if a pixel substantially far down the sorted list is still
    #       within 90% of the the current value
    #    Substantially far is defined as int_round(i * 0.999)
    i = len(img_flat) - 1
    while (0.9 * img_flat[i] > img_flat[min(int(np.rint(i * 0.999)), i-1)]):
        i -= 1

    return scale_image(img, scale=img_flat[i])


# TODO: see external_calibration_monte_carlo.interpolation_based_association for
#   inspiration to implement a fast version that only checks neighbors within max distance
def interp(pt, neighbors, method, scale=1, max=2):
    method_funcs = {'nearest' : interp_nearest,
                    'lanczos' : interp_lanczos,
                    'inv_prop' : interp_inv_prop,
                    'gauss' : interp_gauss,
                    'expo_decay' : interp_expo_decay,
                    'logistic' : interp_logistic}

    assert(method in method_funcs.keys()), "method must be in " + str(list(method_funcs.keys()))

    scoring_func = method_funcs[method]

    # Calculate the distance between the point and its neighbors
    dists = np.linalg.norm(neighbors - pt, axis=1)

    # Get coefficients based on given method
    coeffs = scoring_func(dists, scale)

    # Remove points farther than max
    coeffs = np.where(dists <= max, coeffs, 0.0)

    total_sum = np.sum(coeffs)
    
    # If the sum is 0 (usually only happens when max is too small), use interp_nearest
    if (total_sum == 0.0):
        coeffs = interp_nearest(dists, scale)
        total_sum = 1.0
    
    # Scale so sum of coeffs is 1
    coeffs /= total_sum

    # Return the final coefficients
    return coeffs


def interp_nearest(dists, scale):
    """Returns 1 for closest neighbor and 0 for all else
    scaling does nothing, but is kept so all interp functions have the same inputs
    """
    return np.where(dists == dists.min(), 1.0, 0.0)


def interp_lanczos(dists, scale):
    """lanczos function
    Note: some coefficients are negative
    """
    # Calculate the lanczos parameters
    return np.sinc(dists) * np.sinc(dists / scale)
    

def interp_inv_prop(dists, scale):
    """inverse proportional
    """
    # Calculate the inv proportional parameters
    return 1 / (scale + dists)


def interp_gauss(dists, scale):
    """normal distribution
    """
    return scipy.stats.norm.pdf(dists, scale=scale)


def interp_expo_decay(dists, scale):
    """exponential decay
    """
    return np.exp(-dists / scale)


def interp_logistic(dists, scale):
    """logistic function
    """
    return scipy.stats.logistic.pdf(dists, scale=scale)

