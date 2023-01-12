import os
import numpy as np
import csv
import copy

from upsp.cam_cal_utils import parsers

np.set_printoptions(suppress=True)

debug_print_bumping = True
debug_bump_distance = True
max_bump_iterations = 10  # debug to stop infinite loops


def get_bumping_occlusion(tgt, vis_checker):
    """Helper function to :func:`tgts_get_internals` that finds the point on the model
    surface that intersects the `tgt` normal vector if the target is occluded by the
    model

    Parameters
    ----------
    tgt : dict
        A dict and has, at a minimum, the keys 'tvec', 'target_type', 'norm'. 'tvec' has
        a :class:`numpy.ndarray` (3, 1) representing the position of the target relative
        to the model origin for its associated value. 'norm' has a
        :class:`numpy.ndarray` (3, 1) representing the normal vector of the target
        relative to the model origin for its associated value. 'target_type' has a
        string representing the type of target (most commonly 'dot') for its associated
        value.
    vis_checker : ~upsp.cam_cal_utils.visibility.VisibilityChecker
        BVH to check for visibilty of nodes

    Returns
    -------
    occlusion : bool
        Whether or not there is an occlusion. True means there is, False means there is
        not.
    location : np.ndarray, shape (3,)
        Location where the `tgt` normal vector intersects the model surface
    dist : float
        Distance from the target position to the point of intersection
    """
    # Set the epsilon (occlusion check bumps) distance to 0
    #   This way any occlusion will be returned
    eps = vis_checker.epsilon
    vis_checker.epsilon = 0

    # Get the target tvec and unit normal
    tgt_tvec = np.array(tgt['tvec'], dtype=np.float64)
    tgt_norm = np.array(tgt['norm'], dtype=np.float64)
    tgt_norm /= np.linalg.norm(tgt_norm)

    # Check for occlusions of this target along its normal
    occlusion = vis_checker.does_intersect(tgt_tvec, tgt_norm, return_pos=True)

    # Reset vis_checker.epsilon to the original value
    vis_checker.epsilon = eps

    # If there is an occlusion, investigate
    if occlusion[0]:
        # Calculate the distance from the target to the occlusion location
        dist = np.linalg.norm(tgt_tvec - occlusion[1])

        return (True, occlusion[1], dist)

    else:
        return (False, np.array([0, 0, 0]), -1)


# Helper function to check if the occlusion found is outside the tolerance
def is_real_occlusion(bumping_occlusion, tgts_tol, grid_tol):
    """Helper function to :func:`tgts_get_internals` that determines if an occlusion is
    due to real geometry, or is a numerical/processing error in generating the tgts file

    Parameters
    ----------
    bumping_occlusion : tuple
        Return value of :func:`get_bumping_occlusion` for the target associated with the
        function call
    vis_checker : ~upsp.cam_cal_utils.visibility.VisibilityChecker
        BVH to check for visibilty of nodes
    tgts_tol : float, optional
        Tolerance of the tgts file. Used to determine if target internal-ness is real,
        or if it is an artifact due to the non-water tightness of the model grid
    grid_tol : float, optional
        Tolerance of the grid file. Used to determine if target internal-ness is real,
        or if it is an artifact due to the non-water tightness of the model grid

    Returns
    -------
    bool
        True if the occlusion is due to real geometry
    """
    # The x, y, and z of the target was rounded to 1e-4 by DOTS
    #   So the max distance a target could be rounded was 1e-4 in all 3 axes
    #   This is our tolerance
    dots_tol = np.sqrt(3) * tgts_tol

    # Tim Sanstrom's bvh seems to have a tolerance of 1e-3 built into it
    #   The code is confusing
    intersect_tol = np.sqrt(3) * grid_tol

    # Calculate the total tolerance
    tol = np.linalg.norm([dots_tol, intersect_tol])

    # Return True if the distance to the occlusion is greater than (or equal to) the
    #   tolerance. Otherwise return False
    return (bumping_occlusion[2] >= tol)


def tgts_get_internals(tgts, vis_checker, tgts_tol=1e-4, grid_tol=1e-3):
    """Helper function to :func:`ltgt_bump_internals` that returns all internal targets

    Parameters
    -----------
    tgts : list of dict
        Each target is a dict and has, at a minimum, the keys 'tvec', 'target_type',
        'norm'. 'tvec' has a :class:`numpy.ndarray` (3, 1) representing the position of
        the target relative to the model origin for its associated value. 'norm' has a
        :class:`np.ndarray` (3, 1) representing the normal vector of the target relative
        to the model origin for its associated value. 'target_type' has a string
        representing the type of target (most commonly 'dot') for its associated value.
    vis_checker : ~upsp.cam_cal_utils.visibility.VisibilityChecker
        BVH to check for visibilty of nodes
    tgts_tol : float, optional
        Tolerance of the tgts file. Used to determine if target internal-ness is real,
        or if it is an artifact due to the non-water tightness of the model grid
    grid_tol : float, optional
        Tolerance of the grid file. Used to determine if target internal-ness is real,
        or if it is an artifact due to the non-water tightness of the model grid

    Returns
    -------
    list of tuple
        Each tuple is an occluded targets. The first item in the tuple is the target
        name. The second element is a boolean to denote if it is a real occlusion (like
        another part of the model is blocking the target so it should not be bumped) or
        just some accidental occlusion due to being differentiably inside the model grid
    """
    internals = []
    for tgt in tgts:
        bumping_occlusion = get_bumping_occlusion(tgt, vis_checker)
        if bumping_occlusion[0]:
            internals.append((tgt['name'],
                              bumping_occlusion[2],
                              is_real_occlusion(bumping_occlusion, tgts_tol, grid_tol)))
    return internals


def tgt_bump_internals(tgts, vis_checker, bump_eps=1e-5, tgts_tol=1e-1, grid_tol=1e-3):
    """Bumps all internal targets along their normal to be slightly external. Slightly
    external defined by `bump_eps`

    Parameters
    ----------
    tgts : list of dict
        Each target is a dict and has, at a minimum, the keys 'tvec', 'target_type',
        'norm'. 'tvec' has a :class:`numpy.ndarray` (3, 1) representing the position of
        the target relative to the model origin for its associated value. 'norm' has a
        :class:`np.ndarray` (3, 1) representing the normal vector of the target relative
        to the model origin for its associated value. 'target_type' has a string
        representing the type of target (most commonly 'dot') for its associated value.
    vis_checker : ~upsp.cam_cal_utils.visibility.VisibilityChecker
        BVH to check for visibilty of nodes
    bump_eps : float, optional
        Distance to bump the targets outside the model. Should be small so targets are
        just barely external to the model
    tgts_tol : float, optional
        Tolerance of the tgts file. Used to determine if target internal-ness is real,
        or if it is an artifact due to the non-water tightness of the model grid
    grid_tol : float, optional
        Tolerance of the grid file. Used to determine if target internal-ness is real,
        or if it is an artifact due to the non-water tightness of the model grid

    Returns
    -------
    tgts_bumped : list of dict
        Copy of `tgts` with some target positions bumped along their normal to be
        slightly external.
    was_bumped : bool
        Denotes if any targets were bumped
    """
    # Create a new list to store the bumped targets
    tgts_bumped = []

    # Flag for if at least one target was bumped
    was_bumped = False

    # Iterate through all targets
    for tgt in tgts:
        bumping_occlusion = get_bumping_occlusion(tgt, vis_checker)

        # If there was an occlusion, check the status
        if bumping_occlusion[0]:

            if debug_bump_distance:
                dist = np.linalg.norm(np.array(bumping_occlusion[1]) - np.array(tgt['tvec']))
                print('\tInternal Target:', tgt['name'], 'Dist:', dist)

            # If the occlusion is artificial (differentiably inside the mode), bump the
            #   target to be just outside the model
            if not is_real_occlusion(bumping_occlusion, tgts_tol, grid_tol):
                # Set the was_bumped flag to True
                was_bumped = True

                # Create a copy of the original target
                bumped_tgt = copy.copy(tgt)

                # Normalize the target normal vector
                bumped_tgt['norm'] = np.array(tgt['norm'], dtype=np.float64)
                bumped_tgt['norm'] /= np.linalg.norm(tgt['norm'])

                # Bump the target to the occlusion point plus some small tolerance along the normal
                bumped_tgt['tvec'] = np.array(bumping_occlusion[1]) + bump_eps * bumped_tgt['norm']

                # Add the bumped target to the new list of targets
                tgts_bumped.append(bumped_tgt)

                if debug_print_bumping:
                    print('\tInternal Target:', tgt['name'])
                    print('\t\tOriginal tvec:', tgt['tvec'])
                    print('\t\tNew tvec:', bumped_tgt['tvec'])

            # If the occlusion is real (truely occluded by some model surface),
            #   then just add the original target to the new list of targets
            else:
                tgts_bumped.append(tgt)

        # If there is no occlusion, just add the original target to the original list
        else:
            tgts_bumped.append(tgt)

    return tgts_bumped, was_bumped


def tgt_bump_externals(tgts, vis_checker, bump_eps=1e-5):
    """Bumps very external targets to be slightly external. Slightly external defined by
    `bump_eps`

    Parameters
    ----------
    tgts : list of dict
        Each target is a dict and has, at a minimum, the keys 'tvec', 'target_type',
        'norm'. 'tvec' has a :class:`numpy.ndarray` (3, 1) representing the position of
        the target relative to the model origin for its associated value. 'norm' has a
        :class:`np.ndarray` (3, 1) representing the normal vector of the target relative
        to the model origin for its associated value. 'target_type' has a string
        representing the type of target (most commonly 'dot') for its associated value.
    vis_checker : ~upsp.cam_cal_utils.visibility.VisibilityChecker
        BVH to check for visibilty of nodes
    bump_eps : float, optional
        Distance to bump the targets outside the model. Should be small so targets are
        just barely external to the model

    Returns
    -------
    tgts_bumped : list of dict
        Copy of tgts with some target positions bumped along their normal to be slightly
        external
    """
    tgts_bumped = []

    for tgt in tgts:
        # Create a copy of the target, but with an inverted normal
        tgt_inv_norm  = {'tvec': np.array(tgt['tvec']) + 100 * bump_eps * np.array(tgt['norm']),
                         'norm': -np.array(tgt['norm'])}

        # Get the point on the model surface just behind the target
        bumping_occlusion = get_bumping_occlusion(tgt_inv_norm, vis_checker)

        # Create a copy of the original target
        bumped_tgt = copy.copy(tgt)

        # Normalize the target normal vector
        bumped_tgt['norm'] = np.array(tgt['norm'], dtype=np.float64)
        bumped_tgt['norm'] /= np.linalg.norm(tgt['norm'])

        # Bump the target to the occlusion point plus some small tolerance along the normal
        bumped_tgt['tvec'] = np.array(bumping_occlusion[1]) + bump_eps * bumped_tgt['norm']

        # Add the bumped target to the new list of targets
        tgts_bumped.append(bumped_tgt)

        if debug_print_bumping:
            print('External Bump:', tgt['name'])
            print('\tOriginal tvec:', tgt['tvec'])
            print('\tOcclusion Site:', bumping_occlusion[1])
            print('\tNew tvec:', bumped_tgt['tvec'])

        if debug_bump_distance:
            dist = np.linalg.norm(np.array(bumped_tgt['tvec']) - np.array(tgt['tvec']))
            print('\tExternal Target:', tgt['name'], 'Dist:', dist)

    return tgts_bumped


def tgts_bumper(tgts, vis_checker, bump_eps=1e-5, tgts_tol=1e-1, grid_tol=1e-3):
    """Bumps all internal targets along their normal to be slightly external. Bumps any
    targets very external to be slightly external

    Parameters
    ----------
    tgts : list of dict
        Each target is a dict and has, at a minimum, the keys 'tvec', 'target_type',
        'norm'. 'tvec' has a :class:`numpy.ndarray` (3, 1) representing the position of
        the target relative to the model origin for its associated value. 'norm' has a
        :class:`np.ndarray` (3, 1) representing the normal vector of the target relative
        to the model origin for its associated value. 'target_type' has a string
        representing the type of target (most commonly 'dot') for its associated value.
    vis_checker : ~upsp.cam_cal_utils.visibility.VisibilityChecker
        BVH to check for visibilty of nodes
    bump_eps : float, optional
        Distance to bump the targets outside the model. Should be small so targets are
        just barely external to the model
    tgts_tol : float, optional
        Tolerance of the tgts file. Used to determine if target internal-ness is real,
        or if it is an artifact due to the non-water tightness of the model grid
    grid_tol : float, optional
        Tolerance of the grid file. Used to determine if target internal-ness is real,
        or if it is an artifact due to the non-water tightness of the model grid

    Returns
    -------
    bumped_external_targets : list of dict
        Copy of tgts input with the 'tvec' values modified so that each tgt is outside
        the grid of `vis_checker`
    """
    # Bump the internal targets until nothing needs to be bumped
    # Python doesn't have a Do-While Loop so I have to run it once then throw it into
    #   the loop
    was_bumped = True
    internal_bump_count = 0
    bumped_internal_targets = tgts
    if debug_print_bumping:
        print('Internal Bump Iteration', internal_bump_count)
    while was_bumped:
        if internal_bump_count > max_bump_iterations:
            print('Number of internal bumps exceeds max allowed. Skiping to external bump.' +
                  'Targets are still be internal. Please modify bump_eps, tgts_tol, or grid_tol')
            break

        bumped_internal_targets, was_bumped = tgt_bump_internals(
                bumped_internal_targets, vis_checker, bump_eps, tgts_tol, grid_tol
        )
        internal_bump_count += 1
        if was_bumped and debug_print_bumping:
            print('Internal Bump Iteration', internal_bump_count)

    # Bump the external targets and ensure all targets are external
    are_internal = True
    external_bump_count = 0
    while are_internal:
        if external_bump_count > max_bump_iterations:
            print('Number of external bumps exceeds max allowed. Writing bumped file' +
                  'anyway. Targets are still be internal. Please modify bump_eps, ' +
                  'tgts_tol, or grid_tol')
            break

        # Bump the external targets
        bumped_external_targets = tgt_bump_externals(bumped_internal_targets, vis_checker, bump_eps)
        bumped_internal_targets = bumped_external_targets

        _, was_bumped = tgt_bump_internals(
                bumped_internal_targets, vis_checker, bump_eps, tgts_tol, grid_tol)

        # If there are no internal targets, set are_interal to False
        if not was_bumped:
            are_internal = False

        external_bump_count += 1

    return bumped_external_targets


def tgts_file_bumper(tgts_file_path, vis_checker, bump_eps=1e-5, tgts_tol=1e-1, grid_tol=1e-3):
    """Creates a tgts file with all internal targets bumped along their normal to be
    slightly external and very external targets to be slightly external

    Creates a new tgts file in the same directory as tgts_file_path with the same name,
    but with the suffix '_bumped' attached to the filename

    Parameters
    ----------
    tgts_file_path : path-like
        Path to the tgts file
    vis_checker : ~upsp.cam_cal_utils.visibility.VisibilityChecker
        BVH to check for visibilty of nodes
    bump_eps : float, optional
        Distance to bump the targets outside the model. Should be small so targets are
        just barely external to the model
    tgts_tol : float, optional
        Tolerance of the tgts file. Used to determine if target internal-ness is real,
        or if it is an artifact due to the non-water tightness of the model grid
    grid_tol : float, optional
        Tolerance of the grid file. Used to determine if target internal-ness is real,
        or if it is an artifact due to the non-water tightness of the model grid
    """
    # Read in the targets
    tgts = parsers.read_tgts(tgts_file_path)

    # Bump the targets
    bumped_targets = tgts_bumper(tgts, vis_checker, bump_eps, tgts_tol, grid_tol)

    # Get the filepath of the new tgts file
    filename, file_ext = os.path.splitext(os.path.basename(tgts_file_path))
    new_tgts_file_path = os.path.join(os.path.dirname(tgts_file_path),
                                      filename + '_bumped' + file_ext)

    # Open the new tgts file
    with open(new_tgts_file_path, 'w') as f_write:
        csv_writer = csv.writer(f_write, delimiter=' ', quoting=csv.QUOTE_MINIMAL)

        # Open the old tgts file
        with open(tgts_file_path, 'r') as f_read:
            # Read the old tgts file
            csv_reader = csv.reader(f_read, delimiter=' ')
            is_in_targets_section = False

            # For each line in the old tgts file
            for row in csv_reader:
                line = []
                for item in row:
                    if (item != ''):
                        line.append(item)

                # Check for when we enter the targets section (denoted with *Targets)
                #   When we do, write all the bumped targets
                if not is_in_targets_section and (line[0] == '*Targets'):
                    # Write the *Targets header
                    csv_writer.writerow(line)
                    is_in_targets_section = True

                    # Write the bumped targets
                    for tgt in bumped_targets:
                        tgt_line = [str(tgt['idx']).ljust(5, ' '),
                                    "{:>14}".format("{:3.9f}".format(tgt['tvec'][0][0])),
                                    "{:>14}".format("{:3.9f}".format(tgt['tvec'][1][0])),
                                    "{:>14}".format("{:3.9f}".format(tgt['tvec'][2][0])),
                                    "{:>14}".format("{:3.9f}".format(tgt['norm'][0][0])),
                                    "{:>14}".format("{:3.9f}".format(tgt['norm'][1][0])),
                                    "{:>14}".format("{:3.9f}".format(tgt['norm'][2][0])) + ' ',
                                    str(tgt['size']).ljust(6, ' '),
                                    str(tgt['zones'][0]).ljust(5, ' '),
                                    str(tgt['zones'][1]).ljust(5, ' '),
                                    str(tgt['zones'][2]).ljust(5, ' '),
                                    str(tgt['name']).ljust(6, ' ')]

                        for item in tgt_line:
                            f_write.write(item)
                        f_write.write('\n')

                # If we did not just enter the targets section, check if we are still
                #   chugging through it. If we are, look for the next section break
                #   denoted by something with a * (like *Fiducials or *Virtuals)
                elif is_in_targets_section and '*' in line[0]:
                    is_in_targets_section = False

                # Regardless of if this is before or after the Targets section, if we
                #   are not in the targets section, write the line verbatim
                if not is_in_targets_section:
                    csv_writer.writerow(row)
    return
