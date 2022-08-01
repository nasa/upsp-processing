import visibility
import parsers
import os
import numpy as np
import csv
import copy

np.set_printoptions(suppress=True)

debug_print_bumping = True
debug_bump_distance = True

# Helper function that gets occlusion for target bumping
def get_bumping_occlusion(tgt, vis_checker):
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
def is_real_occlusion(bumping_occlusion, tgts_tol, bvh_tol):
    # The x, y, and z of the target was rounded to 1e-4 by DOTS
    #   So the max distance a target could be rounded was 1e-4 in all 3 axes
    #   This is our tolerance
    dots_tol = np.sqrt(3) * tgts_tol

    # Tim Sanstrom's bvh seems to have a tolerance of 1e-3 built into it
    #   The code is confusing
    intersect_tol = np.sqrt(3) * bvh_tol

    # Calculate the total tolerance
    tol = np.linalg.norm([dots_tol, intersect_tol])

    # Return True if the distance to the occlusion is greater than (or equal to) the
    #   tolerance. Otherwise return False
    return (bumping_occlusion[2] >= tol)


# Helper function that returns all internal targets.
#   Returns a list of internal targets. Each element in the list is a tuple
#   The first item in the tuple is the target name
#   The second element is a boolean to denote if it is a real occlusion or just some
#       accidental occlusion due to being differentiably inside the model grid
def tgts_get_internals(tgts, vis_checker, tgts_tol=1e-4, bvh_tol=1e-3):
    internals = []
    for tgt in tgts:
        bumping_occlusion = get_bumping_occlusion(tgt, vis_checker)
        if bumping_occlusion[0]:
            internals.append((tgt['name'],
                              bumping_occlusion[2],
                              is_real_occlusion(bumping_occlusion, tgts_tol, bvh_tol)))
    return internals


# Check if any targets are differntiably inside the model. If they are, bump them to be
#   differentiably outside the model
def tgt_bump_internals(tgts, vis_checker, bump_eps=1e-5, tgts_tol=1e-1, bvh_tol=1e-3):    
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
            if not is_real_occlusion(bumping_occlusion, tgts_tol, bvh_tol):        
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


# Assumes all targets are outside the model. Returns a list of bumped targets such that
#   each one is differntiably outside the model (by a distance of bump_eps
def tgt_bump_externals(tgts, vis_checker, bump_eps=1e-5):
    tgts_bumped = []

    for tgt in tgts:
        # Create a copy of the target, but with an inverted normal
        tgt_inv_norm  = {'tvec' : np.array(tgt['tvec']) + 100 * bump_eps * np.array(tgt['norm']),
                         'norm' : -np.array(tgt['norm'])}
        
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


# Bump the internal targets, then bump all targets to be just at the surface of the model
def tgts_bumper(tgts, vis_checker, bump_eps=1e-5, tgts_tol=1e-1, bvh_tol=1e-3):
    # Bump the internal targets until nothing needs to be bumped
    # Python doesn't have a Do-While Loop so I have to run it once then throw it into
    #   the loop
    was_bumped = True
    internal_bump_count = 1
    bumped_internal_targets = tgts
    if debug_print_bumping:
        print('Internal Bump Iteration', internal_bump_count)
    while was_bumped:
        bumped_internal_targets, was_bumped = tgt_bump_internals(
                bumped_internal_targets, vis_checker, bump_eps, tgts_tol, bvh_tol
        )
        internal_bump_count += 1
        if was_bumped and debug_print_bumping:
            print('Internal Bump Iteration', internal_bump_count)

    # Uncomment this when tgt_bump_externals is working properly
    # Bump the external targets
    bumped_targets = tgt_bump_externals(bumped_internal_targets, vis_checker, bump_eps)

    return bumped_targets


# Takes a targets file and generates a 'bumped' version where all targets are moved
#   a differential distance along their normal such that they are 1e-5 inches outside
#   the model (+/- some for numerical tolerances)
def tgts_file_bumper(tgts_file_path, vis_checker, bump_eps=1e-5, tgts_tol=1e-1, bvh_tol=1e-3):
    # Read in the targets
    tgts = parsers.read_tgts(tgts_file_path, read_all_fields=True, read_pK=True)
    
    # Bump the targets
    bumped_targets = tgts_bumper(tgts, vis_checker, bump_eps, tgts_tol, bvh_tol)
    
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
                                    "{:>14}".format("{:3.9f}".format(tgt['tvec'][0])),
                                    "{:>14}".format("{:3.9f}".format(tgt['tvec'][1])),
                                    "{:>14}".format("{:3.9f}".format(tgt['tvec'][2])),
                                    "{:>14}".format("{:3.9f}".format(tgt['norm'][0])),
                                    "{:>14}".format("{:3.9f}".format(tgt['norm'][1])),
                                    "{:>14}".format("{:3.9f}".format(tgt['norm'][2])) + ' ',
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

