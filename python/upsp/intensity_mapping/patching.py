import numpy as np
import copy
import cv2
import time
import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)

utils = os.path.join(parent_dir, 'cam_cal_utils')
sys.path.append(utils)

import photogrammetry
import visibility

np.set_printoptions(linewidth=180, precision=4, threshold=np.inf)

# Degree of polynomial fit for patch
degree = 3

# TODO: Another potential method to look into is a 2D taylor expansion
#   F(t_0 + delta_t) = sum n from 0 to inf: 1/n! (n-th derivative of F w.r.t. t) * delta_t^n
#   Where F(t) = f(x_o + t * delta_x, y_o + t * delta_y)
#   (f is the function we want, F(t) is easy to take a taylor series of)
#   n-th derivative of F(t):
#       sum m from 0 to n:
#           (n choose m) * (m-th partial derivative w.r.t. x and (n-m)-th partial
#           derivative w.r.t. y of f) evaluated at (x_o, y_o) * delta_x^m * delta_y^(n-m)

# TODO: Another thing to try (with the current method or Taylor expansion) is weighting
#   Each pixel gets exponentially less weight on the current patch based on distance
#   Typically exp(-d^2) where d is distance


def get_target_node_idxs(nodes, tgts, buffer_thickness_in):
    """Returns the indices of all nodes that are inside of targets

    Returns list of indices of nodes that fall within the patch of a target. This is
    calculated by finding the distance from every node to every target. All nodes that
    are a distance less than tgt['size'] from the target are marked as 'invalid' and the
    indices of those nodes are returned

    This is calculated in a two step procedure for a speedup. It is based on the fact
    that euclidean distance <= manhattan distance <= sqrt(3) * euclidean distance. And
    that manhattan distance is much faster to calculate than euclidean distance since
    there is no square or square root operation.

    Step 1 calculates the manhattan distance (L1 norm) from every node to every target.
    If the manhattan distance is greater than sqrt(3) * tgt['size'], we know for certain
    that the node is at least tgt['size'] units of euclidean distance from the target.
    This will be the vast majority of nodes (all but ~300 for example launch vehicle
    w/ ~1 million nodes).

    Step 2 checks the euclidean distance of each node that failed the manhattan distance
    check. Nodes that fall within tgt['size'] of any target are marked as invalid and
    their index is returned.

    If a large portion of the nodes are near a target, this process would be slow since
    it is effectively double calculating any node near a target. However, since the
    targets are relatively small compared to the surface area of the, it results in a
    roughly 2X speedup.

    Parameters
    ----------
    nodes : np.ndarray (N, 3), float
        A numpy array of the X, Y, and Z values of the nodes
    tgts : list of targets
        Each target is a dictionary with (at a minimum) a 'tvec' and 'size' attribute.
        The 'tvec' attribute gives the target's location and the 'size' gives the 
        euclidean distance from the center of the target to the perimeter
    buffer_thickness_in : float
        Buffer (in inches) to add to fiducials when determining internals applied radially
        (increases effective radius of fiducial by buffer_thickness_in)

    Returns
    ----------
    sorted numpy array (np.int32) of the indices of the nodes that are inside of a target
    """

    # Heuristic - Nodes within np.sqrt(3) * tgt['size'] units of manhattan distance are
    #   flagged as potential invalid nodes
    heuristic_invalid_nodes = set()
    for tgt in tgts:
        manhattan_dist = np.linalg.norm(np.absolute(nodes - tgt['tvec']), ord=1, axis=1)
        heuristic_invalid_nodes.update(np.squeeze(np.argwhere(manhattan_dist < (np.sqrt(3) * (tgt['size'] / 2 + buffer_thickness_in))), axis=1).tolist())
    heuristic_invalid_nodes = list(heuristic_invalid_nodes)

    # Final Check - Of the nodes flagged, check if any are within of euclidean distance
    invalid_nodes = set()
    for tgt in tgts:
        dist = np.linalg.norm(nodes[heuristic_invalid_nodes] - tgt['tvec'], axis=1)
        invalid_nodes.update(set(heuristic_invalid_nodes[i] for i in np.squeeze(np.argwhere(dist < (tgt['size'] / 2 + buffer_thickness_in)), axis=1)))
    
    return np.array(sorted(invalid_nodes), dtype=np.int32)


def patchFiducials(fiduals_visible, inp_img, rmat, tvec, cameraMatrix, distCoeffs, boundary_thickness, buffer_thickness_in):
    """ Patches clusters in the inp_img

    Parameters:
    -----------
    fiduals_visible : list of fiducials
        Each fiducial is a dict with (at a minimum) 'tvec' and 'target_type' attributes.
        The 'tvec' attribute gives the fiducial's location and 'target_type' is a string
        denoting the type of fiducial (most commonly 'dot' or 'kulite')
    inp_img : np.uint8 np.ndarray (image)
        Input image to be patched. rmat and tvec should be aligned to image
    rmat : np.ndarray (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray (3, 1) or (3,), float
        Translation vector from camera to object
    cameraMatrix : np.ndarray (3x3), float
        The (openCV formatted) camera matrix for the camera
    distCoeffs : np.ndarray (5x1) or (5,), float
        The (openCV formatted) distortion coefficients for the camera
    boundary_thickness : int
        thickness of boundary (in pixels)
    buffer_thickness_in : float
        Buffer (in inches) to add to fiducials when determining internals applied radially
        (increases effective radius of fiducial by buffer_thickness_in)

    Returns:
    --------------
    float np.ndarray (image)
        Image with patched fiducials

    See Also:
    -----------
    get_fiducial_internal_and_boundary :
        Return internal and boundary pixel positions for the input fiducial
    get_cluster_internal_and_boundary :
        Returns a list of internal and boundary pixels for the input cluster
    polyfit2D :
        Finds the polynomial fit using the boundary pixels
    polyval2D :
        Finds the value for the internal pixels using the polynomial fit
    """
    # Create an output copy of the input image to work with
    out_img = copy.deepcopy(inp_img).astype(np.float32)

    # Cluster the fiducials
    clusters = clusterFiducials(fiduals_visible, rmat, tvec, cameraMatrix, distCoeffs, boundary_thickness, buffer_thickness_in)

    # Patch every cluster in the output image
    for cluster in clusters:
        # Get the internal and boundary points of the cluster
        if (len(cluster) == 1):
            internals, bounds = get_fiducial_internal_and_boundary(cluster[0], rmat, tvec, cameraMatrix, distCoeffs, boundary_thickness, buffer_thickness_in)
        else:
            internals, bounds = get_cluster_internal_and_boundary(cluster, rmat, tvec, cameraMatrix, distCoeffs, boundary_thickness, buffer_thickness_in)

        # Skip if there are too few points to perform patching
        if (len(bounds) < ((degree+2)*(degree+1)/2)):
            continue

        # Convert the positions to local positions
        min_bounds = (np.min(bounds[:,0]), np.min(bounds[:,1]))
        local_internals = internals - min_bounds
        local_bounds = bounds - min_bounds

        # Get the intensities of the boundary intensities
        Is = [inp_img[bound[1]][bound[0]] for bound in bounds]

        # Fit a 2D polynomial to the boundary intensities
        coeffs = polyfit2D(local_bounds, Is)
                
        # Using the fit, find values for the internal intensities
        new_internal_intensities = polyval2D(local_internals, coeffs)

        # Enter the new intensity values into the array
        internals = internals[:, [1, 0]]
        out_img[tuple(internals.T)] = new_internal_intensities

    return out_img


def clusterFiducials(fiduals_visible, rmat, tvec, cameraMatrix, distCoeffs, boundary_thickness, buffer_thickness_in):
    """ Clusters input fiducials based on image location and image size

    A cluster is made of all fiducials with a path of overlap between them. I.e. A is 
    overlapping B which is overlapping C and D. Therefore A, B, C, and D are all in the
    same cluster. It is considered overlap if the internal pixels to one fiducial
    overlap the internal or boundary pixels of another fiducial. A cluster can be a
    single fiducial if there is no overlap.

    This function clusters targets, then returns the clusters.

    Parameters:
    -----------
    fiduals_visible : list of fiducials
        Each fiducial is a dict with (at a minimum) 'tvec' and 'target_type' attributes.
        The 'tvec' attribute gives the fiducial's location and 'target_type' is a string
        denoting the type of fiducial (most commonly 'dot' or 'kulite')
    rmat : np.ndarray (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray (3, 1) or (3,), float
        Translation vector from camera to object
    cameraMatrix : np.ndarray (3x3), float
        The (openCV formatted) camera matrix for the camera
    distCoeffs : np.ndarray (5x1) or (5,), float
        The (openCV formatted) distortion coefficients for the camera
    boundary_thickness : int
        thickness of boundary (in pixels)
    buffer_thickness_in : float
        Buffer (in inches) to add to fiducials when determining internals applied radially
        (increases effective radius of fiducial by buffer_thickness_in)
    """
    # Represent fiducials as circles with center and radius
    unclustered_fiducial_sets = [] #deleteme?
    # unclustered_image_fiducials = [] #deleteme?
    for fiducial in fiduals_visible:
        internals, bounds = get_fiducial_internal_and_boundary(fiducial, rmat, tvec, cameraMatrix, distCoeffs, boundary_thickness, buffer_thickness_in)
        internal_set = set(tuple(internal) for internal in internals)
        internal_and_boundary_set = set(set(tuple(boundary) for boundary in bounds)).union(internal_set)
        unclustered_fiducial_sets.append([internal_set, internal_and_boundary_set])

        # tgt_proj, targ_size_px, __, __ = get_fiducial_pixel_properties(fiducial, rmat, tvec, cameraMatrix, distCoeffs, boundary_thickness, buffer_thickness_in)
        # # tgt_proj['proj'] is center, targ_size_px is diameter
        # unclustered_image_fiducials.append((tgt_proj['proj'], targ_size_px / 2))

    # unclustered_image_fiducials_orig = copy.deepcopy(unclustered_image_fiducials)
    unclustered_fiducial_sets_orig = copy.deepcopy(unclustered_fiducial_sets)

    # Represent fiducals as nodes on a graph (the abstract data structure)
    # Nodes are connected if the set of pixels internal to the fiducial overlap with the
    #   set of internal or set of boundary pixels of another
    # Clusters are connected components of that graph
    clusters = []
    
    # 1) Add the an unclustered fiducal to a cluster. Remove it from the
    #     unclustered list
    # 2) For each unclusted fiducial, find the distance to each fiducial in the
    #     cluster (not the center of the fidcual, the perimieter of the fiducial)
    # 3) If any of the distances is small enough that the boundaries of one can overlap
    #    with the internals of another, add it to the cluster. Additionally, remove
    #    it from the unclustered and go back to step 2.
    #       (This is defined as the distance being less than the boundary_thickness plus
    #       sqrt(2). If the distance is larger than this it cannot overlap. Anything
    #       less than that is possible)
    # 4) Once an iteration has been completed without adding an unclustered fiducial,
    #     add this cluster to the clusters list. If there are unclustered fiducials,
    #       go back to step 1.
    while len(unclustered_fiducial_sets):
        cluster = [unclustered_fiducial_sets.pop(-1)]
        # Repeat until nothing is added to the cluster
        was_added = True
        while was_added:
            was_added = False
            for unclustered in unclustered_fiducial_sets:
                for clustered in cluster:
                    # If the internals of the targets overlap, add the unclustered to
                    #   the cluster
                    if clustered[0].intersection(unclustered[0]):
                        cluster.append(unclustered)
                        unclustered_fiducial_sets.remove(unclustered)
                        was_added = True
                        break
                    
                    # If the boundaries of one overlap the internals of thee other, add
                    #   the unclustered to the cluster
                    if clustered[1].intersection(unclustered[0]) or clustered[0].intersection(unclustered[1]):
                        cluster.append(unclustered)
                        unclustered_fiducial_sets.remove(unclustered)
                        was_added = True
                        break
                
                if was_added:
                    break
        
        clusters.append(cluster)

    # Repackage as fiducial cluster
    fiducial_clusters = []
    for cluster in clusters:
        fiducial_cluster = []
        
        for image_fiducial in cluster:
            i = unclustered_fiducial_sets_orig.index(image_fiducial)
            fiducial_cluster.append(fiduals_visible[i])
        
        fiducial_clusters.append(fiducial_cluster)

    return fiducial_clusters


# TODO: automatically find fiducials that are close together to group into clusters
def get_cluster_internal_and_boundary(cluster, rmat, tvec, cameraMatrix, distCoeffs, boundary_thickness, buffer_thickness_in):
    """ Returns a list of internal and boundary pixels for the input cluster

    An internal pixel is either directly a part of the fiducials or in between fiducials.
    A boundary pixel is any pixel within buffer pixels of an internal pixel and is not
    an internal itself
    
    Parameters:
    -----------
    cluster : list of fiducials
        Each fiducial is a dict with (at a minimum) 'tvec' and 'target_type' attributes.
        The 'tvec' attribute gives the fiducial's location and 'target_type' is a string
        denoting the type of fiducial (most commonly 'dot' or 'kulite')
    rmat : np.ndarray (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray (3, 1) or (3,), float
        Translation vector from camera to object
    cameraMatrix : np.ndarray (3x3), float
        The (openCV formatted) camera matrix for the camera
    distCoeffs : np.ndarray (5x1) or (5,), float
        The (openCV formatted) distortion coefficients for the camera
    boundary_thickness : int
        thickness of boundary (in pixels)
    buffer_thickness_in : float
        Buffer (in inches) to add to fiducials when determining internals applied radially
        (increases effective radius of fiducial by buffer_thickness_in)
    
    Returns:
    -----------
    tuple
        Both items are np.ndarrays. First is the internals with shape (n, 2) for the
        position (x, y) of n internal pixels. Second is the boundary pixels with shape
        (m, 2) for the positin (x, y) of m boundary pixels

    See Also:
    -----------
    get_fiducial_boundary_map_from_internal_map :
        Determines boundary pixels from bit mask of internal pixels
    get_fiducial_pixel_properties :
        Returns the pixel properties of the input fiducial
    """
    # First stage: Get the minimum, axis aligned rectangle that contains all fiducials
    #   of this cluster
    t_min = np.array([np.iinfo(np.int32).max, np.iinfo(np.int32).max], dtype=np.int32)
    t_max = np.array([0, 0], dtype=np.int32)

    # Iterate over every fiducial, and take the leftmost, topmost, rightmost, and
    #   bottommost fiducial point. Save them in t_min and t_max respectively
    for tgt in cluster:
        # Get the minimum, axis-aligned bounding box for this fiducial
        tgt_proj, targ_size_px, t_min_temp, t_max_temp = get_fiducial_pixel_properties(tgt, rmat, tvec, cameraMatrix, distCoeffs, boundary_thickness, buffer_thickness_in)

        # Update t_min and t_max
        t_min = np.minimum(t_min, t_min_temp)
        t_max = np.maximum(t_max, t_max_temp)
    
    # Second Stage: make a mini images the size of that minimum axis aligned rectangle
    #   and mark internal and boundary pixels. An internal pixel is either directly a
    #   part of the fiducials or in between fiducials. A boundary pixel is any pixel within
    #   buffer pixels of an internal pixel and is not an internal itself

    # Mini image to contain the internal points
    internal_map = np.zeros((t_max[1] - t_min[1], t_max[0] - t_min[0]))
    
    # Mark all points that are directly a part of the fiducials as internal
    for tgt in cluster:
        internal_points, __ = get_fiducial_internal_and_boundary(tgt, rmat, tvec, cameraMatrix, distCoeffs, boundary_thickness, buffer_thickness_in)
        
        # Mark every internal point as internal
        for x, y in internal_points:
            internal_map[y - t_min[1]][x - t_min[0]] = 1

    # Mark all points between fiducials as internal
    
    # Start with a dilation then erosion to fill in any small gaps between fiducials
    #   that might not otherwise get filled. If there are no gaps, this is an identity
    #   operation
    kernel = np.full((3, 3), 1)
    internal_map = cv2.dilate(internal_map, kernel)
    internal_map = cv2.erode(internal_map, kernel)

    # For each column, find the topmost and bottommost internal pixel. Fill in
    #   everything between them
    for x in range(t_max[0] - t_min[0]):
        min_y = t_max[1] - 1
        max_y = 0

        # Find the topmost internal pixel of this row
        for y in range(t_max[1] - t_min[1]):
            if internal_map[y][x]:
                min_y = y
                break
        
        # Find the bottommost internal pixel of this row
        for y in reversed(range(t_max[1] - t_min[1])):
            if internal_map[y][x]:
                max_y = y
                break
        
        # Fill in everything in between
        for y in range(min_y, max_y):
            internal_map[y][x] = 1

    # For each row, find the leftmost and rightmost internal pixel. Fill in everything
    #   between them
    for y in range(t_max[1] - t_min[1]):
        min_x = t_max[0] - 1
        max_x = 0

        # Find the topmost internal pixel of this row
        for x in range(t_max[0] - t_min[0]):
            if internal_map[y][x]:
                min_x = x
                break
        
        # Find the bottommost internal pixel of this row
        for x in reversed(range(t_max[0] - t_min[0])):
            if internal_map[y][x]:
                max_x = x
                break
        
        # Fill in everything in between
        for x in range(min_x, max_x):
            internal_map[y][x] = 1

    # Get all pixels that are not internal and are within boundary_thickness of an internal
    boundary_map = get_fiducial_boundary_map_from_internal_map(internal_map, boundary_thickness)

    # Third Stage: Turn those maps into a list of points

    # Get all internal and boundary points
    internals = np.argwhere(internal_map == 1)[:, [1,0]]
    bounds = np.argwhere(boundary_map == 1)[:, [1,0]]

    # Offset by the region corner to get absolute image coordinates
    internals += (t_min[0], t_min[1])
    bounds += (t_min[0], t_min[1])

    return internals, bounds
    

def get_fiducial_internal_and_boundary(tgt, rmat, tvec, cameraMatrix, distCoeffs, boundary_thickness, buffer_thickness_in):
    """ Return internal and boundary pixel positions for the input fiducial

    An internal pixel a part of the fiducials (minimum axis aligned bounding rectangle).
    A boundary pixel is any pixel within buffer pixels of an internal pixel and is not
    an internal itself

    Parameters:
    -----------
    tgt : dict
        dict with (at a minimum) 'tvec' and 'target_type' attributes. The 'tvec'
        attribute gives the fiducial's location and 'target_type' is a string denoting the
        type of fiducial (most commonly 'dot' or 'kulite')
    rmat : np.ndarray (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray (3, 1) or (3,), float
        Translation vector from camera to object
    cameraMatrix : np.ndarray (3x3), float
        The (openCV formatted) camera matrix for the camera
    distCoeffs : np.ndarray (5x1) or (5,), float
        The (openCV formatted) distortion coefficients for the camera
    boundary_thickness : int
        thickness of boundary (in pixels)
    buffer_thickness_in : float
        Buffer (in inches) to add to fiducials when determining internals applied radially
        (increases effective radius of fiducial by buffer_thickness_in)
    
    Returns:
    -----------
    tuple
        Both items are np.ndarrays. First is the internals with shape (n, 2) for the
        position (x, y) of n internal pixels. Second is the boundary pixels with shape
        (m, 2) for the positin (x, y) of m boundary pixels

    See Also:
    -----------
    get_fiducial_boundary_map_from_internal_map :
        Determines boundary pixels from bit mask of internal pixels
    get_fiducial_pixel_properties :
        Returns the pixel properties of the input fiducial
    """
    assert type(boundary_thickness) is int # delteme

    # Get the tgt_proj and the minimum, axis-aligned bounding box for this fiducial
    tgt_proj, targ_size_px, t_min, t_max = get_fiducial_pixel_properties(tgt, rmat, tvec, cameraMatrix, distCoeffs, boundary_thickness, buffer_thickness_in)

    # Make a mini image the size of the region covered by the fiducial
    # In this image, 0 is safe and 1 is internals
    internal_map = np.zeros([t_max[1] - t_min[1], t_max[0] - t_min[0]])

    # Get the fiducial center location within the local map
    tgt_proj_local = [tgt_proj['proj'][0]-t_min[0], tgt_proj['proj'][1]-t_min[1]]
    
    # For each pixel, find the point on the pixel closest to the center of the fiducial
    # The expression to find the x coordinate of that point is max(X1, min(Xc, X2))
    #   where X1 is the left edge of the pixel, X2 is the right edge, and Xc is the
    #   center of the circle. Similar for the y coordinate
    # The x coordinate of that point can be one of three things:
    #   1) If the center of the fiducial is to the right of the pixel, then
    #       min(Xc, X2) will return X2 and max(X1, X2) will also return X2
    #   2) If the center of the fiducial is inside pixel, then min(Xc, X2) will return
    #       Xc and max(X1, Xc) will also return Xc
    #   3) If the center of the fiducial is to the left of the pixel, then min(Xc, X2)
    #       will return Xc but max(X1, Xc) will return X1
    nx = t_max[0] - t_min[0]
    ny = t_max[1] - t_min[1]
    xs, ys = np.meshgrid(np.arange(nx), np.arange(ny))
    xs, ys = xs.ravel(), ys.ravel()
    Xns = np.maximum(xs - 0.5, np.minimum(tgt_proj_local[0], xs + 0.5))
    Yns = np.maximum(ys - 0.5, np.minimum(tgt_proj_local[1], ys + 0.5))
    
    # For each of the closest points, if it is within the radius of the center it is
    #   internal and needs to be marked as such in the internal_map
    closest_distances = np.linalg.norm([Xns - tgt_proj_local[0], Yns - tgt_proj_local[1]], axis=0)
    internals_idxs = np.argwhere(closest_distances < (targ_size_px / 2))[:, 0]
    internals = np.stack((ys[internals_idxs], xs[internals_idxs]), axis=1)
    internal_map[tuple(internals.T)] = 1

    # Get all pixels that are not internal and are within boundary_thickness of an internal
    boundary_map = get_fiducial_boundary_map_from_internal_map(internal_map, boundary_thickness)

    # Get all internal and boundary points
    internals = np.argwhere(internal_map == 1)[:, [1,0]]
    bounds = np.argwhere(boundary_map == 1)[:, [1,0]]

    # Offset by the region corner to get absolute image coordinates
    internals += (t_min[0], t_min[1])
    bounds += (t_min[0], t_min[1])
    
    return internals, bounds


def get_fiducial_boundary_map_from_internal_map(internal_map, boundary_thickness):
    """ Determines boundary pixels from bit mask of internal pixels

    internal_map is a bitwise mask (image) where 1 is an internal and 0 is not. 
    internal_map needs to be big enough to contain all boundary pixels. That means it
    needs to have a buffer of 0's around it that is boundary_thickness thick on all
    sides (a number of columns of 0's left of leftmost internal equal to
    boundary_thickness. Same for rightmost. And similarly rows above and below)

    Performs n dilation operations on the internal_map with a 3x3 square kernel where
    n is equal to boundary_thickness. boundary_map (return value) is the result minus
    internal_map

    Parameters:
    -----------
    internal_map : np.ndarray (image)
        bitwise mask of internal pixels
    boundary_thickness : int
        thickness of boundary (in pixels)

    Returns:
    -----------
    np.ndarray (image)
        Bitmask of boundary pixels. Same dimensions of internal_map input

    See Also:
    -----------
    get_fiducial_internal_and_boundary :
        Return internal and boundary pixel positions for the input fiducial
    get_cluster_internal_and_boundary :
        Returns a list of internal and boundary pixels for the input cluster
    """
    # Dilate the internals image to get an image of the internals and boundary
    kernel = np.full((3, 3), 1)
    internals_and_boundary_map = copy.deepcopy(internal_map).astype(dtype=np.uint8)
    for i in range(boundary_thickness):
        internals_and_boundary_map = cv2.dilate(internals_and_boundary_map, kernel)

    # Get the boundary by subtracting the internals
    boundary_map = internals_and_boundary_map - internal_map
    return boundary_map


def get_fiducial_pixel_properties(tgt, rmat, tvec, cameraMatrix, distCoeffs, boundary_thickness, buffer_thickness_in):
    """ Returns the pixel properties of the input fiducial

    Pixel properties refers to projected location, pixel size (adjusted for focal
    length, distance to camera, and diameter), and minimum axis-aligned bounding box

    Parameters:
    -----------
    tgt : dict
        dict with (at a minimum) 'tvec' and 'target_type' attributes. The 'tvec'
        attribute gives the fiducial's location and 'target_type' is a string denoting the
        type of fiducial (most commonly 'dot' or 'kulite')
    rmat : np.ndarray (3, 3), float
        Rotation matrix from camera to object
    tvec : np.ndarray (3, 1) or (3,), float
        Translation vector from camera to object
    cameraMatrix : np.ndarray (3x3), float
        The (openCV formatted) camera matrix for the camera
    distCoeffs : np.ndarray (5x1) or (5,), float
        The (openCV formatted) distortion coefficients for the camera
    boundary_thickness : int
        thickness of boundary (in pixels)
    buffer_thickness_in : float
        Buffer (in inches) to add to fiducials when determining internals applied radially
        (increases effective radius of fiducial by buffer_thickness_in)

    Returns:
    -----------
    tuple
        First item is fiducial projection which is a dict with keys 'target_type', and 
        'proj' which map to a string and list of positions (length 2, (x, y))
        respectfully. 'target_type' matches the input 'target_type'. Second item is 
        a float for the fiducial size in pixels accounting for the focal length, distance
        to camera, and diameter. Does not take model geometry into account, and is
        either exactly accurate or an over estimate (likely a mild overestimate). Last
        two items are t_min and t_max. t_min is the upper left corner of the minimum
        axis aligned bounding box of the fiducial plus boundary pixels. t_max is the
        bottom right corner for that bounding box. Both are int np.ndarrays of (2,)

    See Also:
    -----------
    get_fiducial_internal_and_boundary :
        Return internal and boundary pixel positions for the input fiducial
    get_cluster_internal_and_boundary :
        Returns a list of internal and boundary pixels for the input cluster
    """
    # Effective size of fiducial in inches
    targ_size_in = tgt['size'] + 2 * buffer_thickness_in

    # Calculate pixel size of fiducial (assuming planar fiducial normal to camera)
    #   This pixel size is >= the actual pixel size (likely a mild overestimate)
    rmat_model2camera, tvec_model2camera = photogrammetry.invTransform(rmat, tvec)
    cam2tgt_dist = np.linalg.norm(tvec_model2camera - tgt['tvec'])
    targ_size_px = cameraMatrix[1][1] * targ_size_in / cam2tgt_dist

    # Get the tgt projection
    tgt_proj = photogrammetry.project_targets(rmat, tvec, cameraMatrix, distCoeffs, [tgt])[0]

    # Get the an axis aligned minimum rectangle for this fiducial (plus buffer)
    t_min = [tgt_proj['proj'][0] - 0.5 * targ_size_px - boundary_thickness - 1,
             tgt_proj['proj'][1] - 0.5 * targ_size_px - boundary_thickness - 1]
    t_max = [tgt_proj['proj'][0] + 0.5 * targ_size_px + boundary_thickness + 1,
             tgt_proj['proj'][1] + 0.5 * targ_size_px + boundary_thickness + 1]

    # Take the floor and ceiling respectively to convert to an integer
    t_min = np.floor(t_min).astype(np.int32)
    t_max = np.ceil(t_max).astype(np.int32)
    
    return tgt_proj, targ_size_px, t_min, t_max


def polyfit2D(bounds, Is):
    """ Finds the polynomial fit using the boundary pixels
    
    Parameters:
    -----------
    bounds : np.ndarray vector (n, 2) of floats
        (x, y) position of boundary pixels in local coordinates (i.e. leftmost boundary
        pixel has x coordinate of 0. Topmost has coordinate of 0). bounds[n] corresponds
        to Is[n]
    Is : np.ndarray vector (n,) of floats
        Intensity value of boundary pixels pixels. Is[n] corresponds to bounds[n]

    Returns:
    -----------
    np.ndarray vector (n, 1)
        coeffs polynomial fit coefficients

    See Also:
    -----------
    polyval2D : Finds the value for the internal pixels using the polynomial fit
    """
    assert((len(bounds) == len(Is)))
    
    # Determine the number of coefficients
    # Terms are 1 constant, 2 linear (x, y), 3 quadratic (xx, xy, yy), ...
    # Equivalent to 1 + 2 + ... + degree + (degree+1) = (degree+2)*(degree+1)/2
    num_coeffs = int((degree + 2) * (degree + 1) / 2)

    # Number of boundary terms must be greater than or equal to the number of coefficients
    assert(len(bounds) >= num_coeffs), "Not enough boundary terms, please increase boundary_thickness or decrease degree"

    # Initialize the least squares input matrix
    #   Ax ~= b where ~= is least squares solution
    # Can turn least squares polynomial fit into linear solution by solving linearly
    #   w.r.t. poly terms. I.e. [1, x, xx, xy, yy, ...] where (x, y) is pixel position
    #   and b is pixel intensity
    A = np.zeros((len(bounds), num_coeffs), dtype=np.float32)
    count = 0
    for i in range(degree + 1):
        for j in range(degree + 1):
            if (i + j) <= degree:
                A[:, count] = pow(bounds[:, 1], i) * pow(bounds[:, 0], j)
                count += 1
    
    # Solution to linear equation is just pixel values
    b = np.array(Is)

    # Initialize polynomial coefficient output vector
    poly = np.zeros(num_coeffs, dtype=np.float32)
    
    # Solve least squares problem
    coeffs = np.linalg.lstsq(A, b, rcond=None)[0]
    
    return np.array(coeffs)


def polyval2D(internals, coeffs):
    """ Finds the value for the internal pixels using the polynomial fit
    
    Parameters:
    -----------
    internals : np.ndarray vector (n, 2) of floats
        (x, y) position of internal pixels in local coordinates (i.e. leftmost boundary
        pixel has x coordinate of 0. Topmost has y coordinate of 0)
    coeffs : np.ndarray vector (n,)
        Polynomial fit coefficients
    
    Returns:
    -----------
    float np.ndarray (n,)
        Estimated intensity for internals

    See Also:
    -----------
    polyfit2D : Finds the polynomial fit using the boundary pixels
    """
    # Determine the number of coefficients
    # Terms are 1 constant, 2 linear (x, y), 3 quadratic (xx, xy, yy), ...
    # Equivalent to 1 + 2 + ... + degree + (degree+1) = (degree+2)*(degree+1)/2
    num_coeffs = int((degree + 2) * (degree + 1) / 2)

    # Initialize the input matrix
    #   Ax = b where A is the polynomial terms, x is the coefficients, and b are the
    #   resulting intensity estimates
    # Can polynomial function into linear solution by solving linearly w.r.t. poly terms
    #   I.e. [1, x, xx, xy, yy, ...] where (x, y) is pixel position
    A = np.zeros((len(internals), num_coeffs), dtype=np.float32)
    count = 0
    for i in range(degree + 1):
        for j in range(degree + 1):
            if (i + j) <= degree:
                A[:, count] = pow(internals[:, 1], i) * pow(internals[:, 0], j)
                count += 1

    Is = np.matmul(A, coeffs)
    return Is

