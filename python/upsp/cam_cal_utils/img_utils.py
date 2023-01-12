import numpy as np
import scipy.interpolate


def convert_12bit_to_8bit(img):
    """Proportionally scales image values from (0, 4095) to (0, 255)

    Scales input image values from (0, 4095) to (0, 255). Values in input image that
    are greater than scale will be clipped to 255 in the output image

    Parameters
    ----------
    img : np.ndarray, shape (y, x), np.uint8
        Input image

    Returns
    -------
    scaled_img : np.ndarray, shape (y, x), np.uint8
        8 bit scaled version of input image
    """
    return scale_image(img, scale=(2**12) - 1)


def scale_image(img, scale=(2**12) - 1):
    """Proportionally scales image values from (0, `scale`) to (0, 255)

    Scales input image values from (0, `scale`) to (0, 255). Values in input image that
    are greater than `scale` will be clipped to 255 in the output image

    Parameters
    -----------
    img : np.ndarray, shape (y, x)
        Input image
    scale : float or int, optional
        Maximum value for scaling input image. Default=4095 (max int for 12 bit)

    Returns
    -------
    scaled_img : np.ndarray, shape (y, x), np.uint8
        8 bit scaled version of input image
    """

    # Clip the image to the max value
    img_temp = np.minimum(img, scale)

    # Normalize the image
    img_temp = img_temp.astype(np.float64)
    img_temp = img_temp / scale

    # Convert to an 8 bit image
    img_temp = img_temp * ((2**8) - 1)
    img_temp = np.rint(img_temp).astype(np.uint8)

    return img_temp


def scale_image_max_inlier(img):
    """Proportionally scales image values from (0, `max_inlier`) to (0, 255)

    Scales input image values from (0, `max_inlier`) to (0, 255). Values in input image
    that are greater than the max inlier will be clipped to 255 in the output image

    In a sorted list of the pixel intensities, the max inlier is the largest value that
    satisfies the following condition: ``intensity[i] * 0.9 <= intensity[i*0.999]``

    Parameters
    ----------
    img : np.ndarray, shape (y, x), np.unit8
        Input image

    Returns
    -------
    scaled_img : np.ndarray, shape (y, x), np.unit8
        8 bit scaled version of input image
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


def interp(pts, img, method='nearest'):
    """Interpolate values for `pts` from `img`

    Interpolates `img` at the locations defined by `pts` using the given method

    Parameters
    ----------
    pts : np.ndarray, shape (n, 2), float
        n points to be interpolated
    img : np.ndarray, shape (y, x), np.unit8
        Input image
    method : {'linear', 'nearest', 'slinear', 'cubic', 'quintic'}, optional
        Interpolation method. See :class:`scipy.interpolate.RegularGridInterpolator` for
        details

    Returns
    -------
    out : np.ndarray, shape (n, 2), float
        Value of pts interpolated from img
    """
    # Ensure the method is one supported by scipy
    assert method in ['linear', 'nearest', 'slinear', 'cubic', 'quintic']

    # Define the X and Y domains
    X, Y = np.arange(img.shape[1]), np.arange(img.shape[0])

    # Define the scipy interpolation function
    f = scipy.interpolate.RegularGridInterpolator((X, Y), img.T, method=method)

    # Return the interpolated values for the points
    return f(pts)
