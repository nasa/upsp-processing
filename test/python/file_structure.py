import os
import csv


def setUp(data_dir):
    """
    Checks files are set up properly in the given data_dir

    Parameters
    ----------
    data_dir : string
        Absolute path to data directory
    """
    # walk through upsp-proc/test/data and get the file structure
    #   Get paths relative to this file
    relevant_file_setup = []
    file_setup = os.walk(data_dir)
    for f in file_setup:
        f = list(f)
        f[0] = os.path.relpath(f[0], os.path.dirname(__file__))
        relevant_file_setup.append(f)
    
    # convert the file structure to strings to match what is written in the csv
    relevant_file_setup = [[str(x) for x in f] for f in relevant_file_setup]

    # Read the ground truth file structure csv file
    relevant_ground_truth = []
    file_ground_truth = os.path.join(data_dir, os.path.basename(data_dir) + '.csv')
    with open(file_ground_truth, 'r') as fp:
        read = csv.reader(fp)
        for line in read:
            relevant_ground_truth.append(line)

    # Check that all files in the ground truth are in upsp-proc/test/data
    for rgt in relevant_ground_truth:
        try:
            assert (rgt in relevant_file_setup)
        except AssertionError as e:
            message = "File structure error in " + data_dir
            e.args = (message,)
            raise
    
    # Check that all files in upsp-proc/test/data are in the ground truth
    for rfs in relevant_file_setup:
        try:
            assert (rfs in relevant_ground_truth)
        except AssertionError as e:
            message = "File structure error in " + data_dir
            e.args = (message,)
            raise


    # If all ground truth are in upsp-proc/test/data, and vice versa, they are the same
    return


def file_setup_writer(data_dir):
    """
    Writes a csv file of all files in data_dir, recursively

    Parameters
    ----------
    data_dir : string
        Absolute path to data directory
    """
    
    with open(os.path.join(data_dir, 'data.csv'), 'w') as fp:
        wri = csv.writer(fp)
        for files in os.walk(files_dir):
            files = list(files)
            files[0] = os.path.relpath(files[0], os.path.dirname(__file__))
            wri.writerow(files)


if __name__ == '__main__':
    files_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')
    file_setup_writer(files_dir)
