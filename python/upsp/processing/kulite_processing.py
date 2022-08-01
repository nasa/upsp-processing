import logging
from . import kulite_utilities

log = logging.getLogger(__name__)


def write_kulite_binary(output_file, data_array):
    """Generate a binary file with kulite pressure-time histories

    Args:
        output_file (str)       : output binary file location
        binary_data (bytearray) : binary array of data

    Returns:
        None
    """
    data_array.tofile(output_file)


def write_matrix_txt(output_file, kul_mat, test):
    """Generate a text file with input matrix

    Args:
        output_file (str)        : output text file location
        kul_mat (List of Lists)  : kulite matrix of data to write
        test (str)               : test number

    Returns:
        None
    """
    # if t11-0377 test, manually make the header - this information is not
    # present in the targets file for this experiment
    if test == "t11-0377":
        fields = ['Kulite Name', 'X Position', 'Y Position', 'Z Position',
                  'Radius', 'Azimuthal Angle', 'Normal i', 'Normal j',
                  'Normal k', 'Kulite Diameter', 'Zone Number']
        fieldsStr = '\t'.join(fields)

        with open(output_file, 'w') as txtfile:
            txtfile.write(fieldsStr)
            txtfile.write('\n')

            for row in kul_mat:
                rowStr = '\t'.join(row)
                txtfile.write(rowStr)
                txtfile.write('\n')

    # Otherwise, just write the data without a header
    else:
        with open(output_file, 'w') as txtfile:
            for row in kul_mat:
                rowStr = '\t'.join(row)
                txtfile.write(rowStr)
                txtfile.write('\n')


def process_data(kulite_files_path, datapoint, kulite_targets_file,
                 output_kulite_data_file):

    log.info("Targets file location: %s", kulite_targets_file)
    log.info("Data files location: %s", kulite_files_path)

    # Process the kulite data information
    run = int(datapoint[0 : 4])
    seq = int(datapoint[4 : 6])
    kuls_data_struct = kulite_utilities.Kulites(kulite_files_path, run, seq,
                                                kulites='all', psf=True)

    # Process the kulite location information
    kul_mat = kulite_utilities.read_targets_matrix(kulite_targets_file,
                                                   kul_strs='all')

    # Compare the location and data information to validate them
    kuls = run_check_kulite_data_location(kuls_data_struct, kul_mat)

    # Extract the location kulites list to inform the data extracted
    kuls_data_selected = kulite_utilities.Kulites(kulite_files_path, run, seq,
                                                  kuls, psf=True)

    # Convert the data to binary and write it
    kuls_data = kulite_utilities.create_pressure_array(kuls_data_selected)
    write_kulite_binary(output_kulite_data_file, kuls_data)


def run_check_kulite_data_location(kuls_data, kuls_loc):

    kul_list = set()

    # Check lengths of kulite lists
    kulite_data_len = len(kuls_data.data)
    kulite_loc_len = len(kuls_loc[:, 0])
    kuls_name_data = list(kuls_data.data.keys())
    kuls_name_loc = kuls_loc[:, 0]

    if (kulite_data_len != kulite_loc_len):
        log.warn("The kulite lists in the target and data files have different lengths")
        log.warn("The data files contain %s kulites", kulite_data_len)
        log.warn("The targets file contains %s kulites", kulite_loc_len)

    # Check individual kulites from the targets file
    for i in range(0, kulite_loc_len, 1):
        kul_name_loc = kuls_name_loc[i]

        if kul_name_loc in kuls_name_data:
            # Only add the kulites in the target file that have data
            kul_list.add(kul_name_loc)
        else:
            log.warn("Kulite %s found in targets file but not in data files",
                     kul_name_loc)

    # Check individual kulites from the data files
    for i in range(0, kulite_data_len, 1):
        kul_name_data = kuls_name_data[i]

        if (kul_name_data not in kuls_name_loc):
            log.warn("Kulite %s found in data files but not in targets file",
                     kul_name_data)

    return kul_list
