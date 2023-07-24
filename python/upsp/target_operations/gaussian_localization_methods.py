import matplotlib.pyplot as plt
import scipy.optimize as opt
import numpy as np
import warnings
from scipy.optimize import OptimizeWarning

from upsp.target_operations import blob_localization_methods

np.set_printoptions(linewidth=180, precision=8, threshold=np.inf)

super_gauss_power_upper_bound = 20

#---------------------------------------------------------------------------------------
# 2D Gaussian Functions


def super_twoD_Gaussian(pt, amplitude, xo, yo, sx, sy, theta, offset, p):
    """Returns the value of a super 2D Gaussian function at the given point

    Parameters
    ----------
    pt : np.ndarray, shape (N, 2), float
        2D Gaussian function point input. Length of output is equal to length of pt
    amplitude : float
        Amplitude of super 2D Gaussian
    xo, yo : float
        mean position of super 2D Gaussian (center of target)
    sx, sy: float
        standard deviations of super 2D Gaussian
    theta : float
        Angle of super 2D Gaussian ellipse
    offset : float
        Floor of super 2D Gaussian
    p : float
        Power of super 2D Gaussian. Higher power is more platykurtic

    Returns
    -------
    float or np.ndarray, shape (N, 1)
        Value of super 2D Gaussian function with given parameters at given point(s).
        Length of return is equal to length of the pt input
    """

    # Seperate the point into x an y
    x, y = pt

    # Get the x and y distance to the distribution's center
    xval = x - xo
    yval = y - yo

    # Calculate the Quadratic Coefficients
    cos_sq = np.cos(theta)**2
    sin_sq = np.sin(theta)**2
    sin2 = np.sin(2 * theta)
    sx_sq = sx**2
    sy_sq = sy**2

    a = (cos_sq / (2 * sx_sq)) + (sin_sq / (2 * sy_sq))
    b = - (sin2 / (4 * sx_sq)) +   (sin2 / (4 * sy_sq))
    c = (sin_sq / (2 * sx_sq)) + (cos_sq / (2 * sy_sq))

    # Calculate the Gaussian value and scale by the amplitude
    quad = a * (xval**2) + 2 * b * (xval * yval) + c * (yval**2)

    # Exp the quadratic
    g = np.exp(-np.power(quad, p))
    g *= amplitude

    # Add an offset
    g += offset

    return g


def twoD_Gaussian(pt, amplitude, xo, yo, sx, sy, theta, offset):
    """Wrapper to call :func:`super_twoD_Gaussian` with a power of 1

    See Also
    ----------
    super_twoD_Gaussian :
        Returns the value of a super 2D Gaussian function at the given point
    """
    return super_twoD_Gaussian(pt, amplitude, xo, yo, sx, sy, theta, offset, 1)


def super_twoD_Gaussian_nobounds(
    pt,
    ln_amplitude,
    xo,
    yo,
    ln_sx,
    ln_sy,
    theta,
    offset,
    ln_p
):
    """Wrapper to call :func:`super_twoD_Gaussian` with no bounds on inputs

    For :func:`super_twoD_Gaussian`, the function does not make physical sense if
    amplitude, the standard deviations, or power are non-positive. (power must be > 1
    for the function to be platykurtic)

    :func:`super_twoD_Gaussian_nobounds` is for optimizers that require no bounds. The
    natural logarithm of those values are passed to this function, and those values are
    passed through ``exp()`` before being passed to super_twoD_Gaussian. Those
    parameters can then vary from -inf to +inf, and will be mapped to (0, np.inf]

    Parameters
    ----------
    pt : np.ndarray, shape (N, 2), float
        2D Gaussian function point input. Length of output is equal to length of pt
    ln_amplitude : float
        Natural log of the amplitude of super 2D Gaussian
    xo, yo : float
        mean position of super 2D Gaussian (center of target)
    ln_sx, ln_sy: float
        Natural log of standard deviations of super 2D Gaussian
    theta : float
        Angle of super 2D Gaussian ellipse
    offset : float
        Floor of super 2D Gaussian
    ln_p : float
        Natural log of the power of super 2D Gaussian minus 1. ``p = exp(ln_p) + 1``
        Higher power is more platykurtic.

    Returns
    ----------
    float or np.ndarray, shape (N, 1)
        Value of super 2D Gaussian function with given parameters at given point(s).
        Length of return is equal to length of the pt input

    See Also
    ----------
    super_twoD_Gaussian :
        Returns the value of a super 2D Gaussian function at the given point
    """
    return super_twoD_Gaussian(
        pt,
        np.exp(ln_amplitude),
        xo,
        yo,
        np.exp(ln_sx),
        np.exp(ln_sy),
        theta,
        offset,
        np.exp(ln_p) + 1
    )


#---------------------------------------------------------------------------------------
# Gaussian Target Localization


def gauss_fitter_func(fit, get_rms=False, curve_fit_kwds=None, debug=False):
    """Returns a function that acts as a wrapper around a 2D Gaussian function to act as
    a target center localizer

    Parameters
    ----------
    fit : {'super', 'normal'}
        Specifies fitting function to use, super or standard 2D Gaussian
    get_rms : bool, optional
        If True, adds rms to output. If False does not
    curve_fit_kwds : dict, optional
        Keyword arguments for scipy curve fit optimizer. If None, the following keywords
        are used: ``{'maxfev' : 25000, 'ftol' : 1e-4, 'xtol' : 1e-4}``
    debug : bool, optional
        If True, adds full optimization parameters and covariance matrix to output. If
        False does not

    Returns
    -------
    callable
        2D Gaussian Fitting Function - returns center of target given an image

        Wrapper around optimizer of 2D Gaussian functions to act as target localization
        function.

        Arguments:

        - ``img`` (:class:`numpy.ndarray`, 2 dimensional (grayscale), unint8) : Image
          with one target. Typically small
        - ``center`` (tuple of length 2, float): Initialization for target localization
        - ``target_type`` (``{'dot', 'kulite', None}``): Type of target in image. Used
          to initialize target finding parameters
        - ``img_offset`` (tuple or None): Since the center location is from the whole
          image, and the localizer is a local function, the offset tells where the
          center location is in the local image. Additionally, output is offset by
          `img_offset` to give the target's location in the whole image. If None, do
          none of that.

        Returns:

            If `debug` input is False, return tuple contains center as first item.
            Center is a tuple of 2 floats denoting the target x and y position.

            If `get_rms` is True, return tuple contains rms error as second item. If
            `get_rms` is False, return tuple has no second item.

            If `debug` input is True, return tuple contains an inner tuple as the first
            item. The inner tuple contains the full set of optimizer parameters as the
            first item, and the covariance matrix as the second item.

            If there is an error in the optimization, all None's will be returned for the
            center, optimizer parameters, covatiance matrix, and rms as needed.
    """

    if (fit == 'super'):
        fitter = super_twoD_Gaussian
    elif (fit == 'normal'):
        fitter = twoD_Gaussian
    else:
        print("Specified fitter does not exist.")
        print("fitter must be 'super' or 'normal'")
        quit()

    blob_detector = blob_localization_methods.blob_func('detector_all', 'last')
    warnings.filterwarnings(
        "ignore",
        category=OptimizeWarning,
        message=".*Covariance of the parameters could not be estimated*."
    )

    if curve_fit_kwds is None:
        curve_fit_kwds = {'maxfev': 25000, 'ftol': 1e-4, 'xtol': 1e-4}

    def fit_gaussian(img, center=None, target_type=None, img_offset=None):
        """Returns center of target given an image

        Wrapper around optimizer of 2D Gaussian functions to act as target localization
        function.

        Parameters
        ----------
        img : np.ndarray, 2 dimensional (grayscale), unint8
            Image with one target. Typically small
        center : tuple of length 2, float or None. Optional, default=None
            Initialization for target localization. If None, blob detection is performed
            to initalize the target position
        target_type : {'dot', 'kulite', None}, default=None
            Type of target in image. Used to initialize target finding parameters
        img_offset : tuple or None, default=None
            Since the center location is from the whole image, and the localizer
            is a local function, the offset tells where the center location is in
            the local image. Additionally, output is offset by img_offset to give the
            target's location in the whole image. If None, do none of that.

        Returns
        ----------
        tuple
            If gauss_fitter_func's debug input is False, return tuple contains center as
            first item. Center is a tuple of 2 floats denoting the target x and y
            position. If gauss_fitter_func's get_rms is True, return tuple contains
            rms error as second item. If get_rms is False, return tuple has no second
            item.
            If gauss_fitter_func's debug input is True, return tuple contains an inner
            tuple as the first itme. The inner tuple contains the full set of optimizer
            parameters as the first item, and the covariance matrix as the second item.
            If gauss_fitter_func's get_rms is True, return tuple contains rms error as
            the second item. If get_rms is False, return tuple has no second item.
            If there is an error in the optimization, all None's will be returned for
            the center, optimizer parameters, covatiance matrix, and rms as needed.
        """
        # If a keypoint was not given as input, find one with a blob detector
        if center is None:
            keypoint = blob_detector(img, return_keypoint=True)

            # If the blob detector didn't find a keypoint, initialize one based
            #   on a reasonable guess
            if keypoint is None:
                center = ((img.shape[1] - 1) / 2, (img.shape[0] - 1) / 2)
            else:
                center = keypoint.pt

        # Populate initial guess information based on the center input
        center_x = center[0]
        center_y = center[1]

        # img_offset serves to help if the center input than the input image
        #   (as is the case with cropping)
        if img_offset is not None:
            center_x -= img_offset[0]
            center_y -= img_offset[1]

        # Populate the initial guess for size_x and size_y based on target type
        if target_type is None:
            size_x = 0.75
            size_y = 0.75
        else:
            if (target_type == 'kulite'):
                size_x = 0.8
                size_y = 0.8
            if (target_type == 'dot'):
                size_x = 1.2
                size_y = 1.2

        # Initialize the guess for angle to 0
        angle = 0

        # For scipy.curve_fit, img needs to be a float
        img = img.astype(np.float32)

        # Targets are dark dots (dips in intensity relative to background)
        # To make the gaussian fit more intuitive, multiply the image by -1
        #   so they are now bright dots relative to the background (raises in
        #   intensity relative to background)
        img = -img

        # Create the grid of pixel locations as a two 2D numpy arrays
        x = np.arange(0, img.shape[1])
        y = np.arange(0, img.shape[0])
        x, y = np.meshgrid(x, y)

        # Unravel the grid points and image into corresponding
        pts = np.vstack((x.ravel(), y.ravel()))
        z = img.ravel()

        # Populate an initial guess estimate for the curve fit
        initial_guess = [img.max() - np.average(img),
                         center_x, center_y,
                         size_x, size_y, angle,
                         np.average(img)]

        # Set the bounds for the inputs
        bounds = [
                    (0.0, np.inf),  # Amplitude bounds
                    (0.0, img.shape[1]), # center x location bounds
                    (0.0, img.shape[0]), # center y location bounds 
                    (np.finfo(float).eps, np.inf), # x std dev bound
                    (np.finfo(float).eps, np.inf), # y std dev bound
                    (-np.inf, np.inf),  # angle bounds
                    (min(img.flatten()), max(img.flatten())) # image offset bounds
                ]

        # If this is using a super gauss add the additional p term for the initial
        #   guess and bounds
        if (fit == 'super'):
            # Populate the initial guess for power based on target type
            if target_type is None:
                init_guess_p = 2.6
            else:
                if (target_type == 'kulite'):
                    init_guess_p = 1.8
                if (target_type == 'dot'):
                    init_guess_p = 3.2

            initial_guess.append(init_guess_p)
            bounds.append((1.0, super_gauss_power_upper_bound))

        # Re-arange the bounds into the form that scipy accepts
        bounds = [[bound[0] for bound in bounds], [bound[1] for bound in bounds]]

        # Create an return value object to store the output
        retval = []

        try:
            popt, pcov = opt.curve_fit(fitter, pts, z, p0=initial_guess,
                                       method='trf', bounds=bounds, **curve_fit_kwds)

            # If debug is on, append the full popt and pconv
            if debug:
                retval.append((popt, pcov))

            # If debug is off, append the center location as a tuple to the return value
            else:
                x_opt, y_opt = popt[1], popt[2]

                if img_offset is not None:
                    x_opt += img_offset[0]
                    y_opt += img_offset[1]

                retval.append(np.array((x_opt, y_opt)))

            # If retval is on, calculate the rms and append
            if get_rms or debug:
                z_hat = fitter(pts, *popt)

                # Debug visualizations
                if debug:
                    plt.figure('img')
                    plt.imshow(img, cmap='gray', aspect='equal')
                    plt.colorbar()
                    plt.savefig('debug_img.png')

                    plt.figure('z_hat')
                    plt.imshow(z_hat.reshape(img.shape), cmap='gray', aspect='equal')
                    plt.colorbar()

                    plt.savefig('debug_zhat.png')
                    plt.show()

                if get_rms:
                    error = z - z_hat
                    rms = np.sqrt(np.sum(np.power(error, 2)) / error.size)
                    retval.append(rms)

        except RuntimeError as e:
            # Since there was a runtime error, throw a warning and return the None flags
            warnings.warn(str(e), RuntimeWarning)

            # If debug is on, append arrays of the appropriate size for popt and pconv
            if debug:
                retval.append((np.full(len(initial_guess), None),
                               np.full((len(popt), len(popt)), None)))

            # If debug is off, just append (None, None) as the center location
            else:
                retval.append(np.array((None, None)))

            # If get_rms is on, append None
            if get_rms:
                retval.append(None)

        except ValueError as e:
            # Since there was a runtime error, throw a warning and return the None flags
            warnings.warn(str(e), RuntimeWarning)

            # If debug is on, append arrays of the appropriate size for popt and pconv
            if debug:
                retval.append(np.full(len(initial_guess), None))
                retval.append(np.full((len(initial_guess), len(initial_guess)), None))

            # If debug is off, just append (None, None) as the center location
            else:
                retval.append((None, None))

            # If get_rms is on, append None
            if get_rms:
                retval.append(None)

            return tuple(retval)

        # Return a tuple of the return value (converting from list)
        return tuple(retval)

    return fit_gaussian
