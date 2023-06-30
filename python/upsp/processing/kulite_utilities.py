"""Kulites class and helpers"""
import numpy as np
import os
import re
import scipy.io
from scipy import signal
from glob import glob
import pandas as pd
import csv
from collections import namedtuple
from collections import OrderedDict
import copy
import logging

from . import p3d_utilities

log = logging.getLogger(__name__)


class Kulites:
    """Load and manage kulite data

    Data is in PSI unless optional psf argument is True

    Parameters
    ----------
    data_dir : str
        directory with ``*.info``, ``*.slow``, ``*.fast`` files
    run : int
        run number
    seq : int
        sequence number
    kulites : list
        kulites to load
    data_type : numpy.dtype
        data type of array elements
    f_type : str
        'slow' or 'fast'

    Attributes
    ----------
    data : dict
        Dictionary with Kulite names as keys and numnpy arrays with pressure time
        histoires as values.
    """

    def __init__(self, data_dir, run, seq, kulites='all',
                 data_type=np.float32, psf=False, f_type='slow'):
        # Create dictionary to hold data
        self.raw, raw = dict(), dict()
        self.native, native = dict(), dict()
        self.data, data = dict(), dict()

        # Get a list of all info files. There are a couple variations,
        # auto-detect using regexes.
        info_files = self.__autodetect_info_files(data_dir, run, seq)
        log.debug('Autodetected info files: %s', str(info_files))

        # Check each info file for the kulites of interest
        for info_file in sorted(info_files):

            # Load the header information
            info = self.__read_info(info_file)

            # Search for kulites of interest, or grab them all
            idcs, kuls = [], []
            for idx, kul in enumerate(info.chanconfig.index):
                if (kulites == 'all' and 'K' in kul) or (kul in kulites):
                    idcs.append(idx)
                    kuls.append(kul)

            # If there is data of interest in this file, load it
            if len(idcs) > 0:
                vraw, vnative, vpsi = self.__read_data(info, idcs, f_type)
                for i in range(0, len(idcs)):
                    if psf:
                        vpsi[i] = vpsi[i] * 144
                    data[kuls[i]] = np.array(vpsi[i], dtype=data_type)
                    raw[kuls[i]] = np.array(vraw[i], dtype=data_type)
                    native[kuls[i]] = np.array(vnative[i], dtype=data_type)
                self.data = OrderedDict(sorted(data.items()))
                self.raw = OrderedDict(sorted(raw.items()))
                self.native = OrderedDict(sorted(native.items()))

            # Get frequency information
            if f_type == 'fast':
                row_idx = 0
            elif f_type == 'slow':
                row_idx = 1
            self.sample_rate = info.statistics['Sampling Rate'][row_idx]

    def __autodetect_info_files(self, data_dir, run: int, seq: int):
        # Find all *.info files corresponding to this datapoint (run, seq).
        # Searches based on filenames in the Kulite data folder.
        #
        # (If there are more Kulites than there are channels for a single PXI
        # unit, than there may be more than one *.info file).
        fnames = glob(os.path.join(data_dir, "*.info"))
        prefixes = [os.path.splitext(os.path.basename(fn))[0] for fn in fnames]
        patterns = [
            r'T(?P<run>\d+)p(?P<seq>\d+)t\d+$',
            r'r(?P<run>\d{4})s(?P<seq>\d{3})t\d+_S\dC\d$'
        ]

        def match_run_seq(_p, _prefix):
            m = re.search(_p, _prefix)
            if m:
                return int(m.group('run'), base=10), int(m.group('seq'), base=10)
            return None

        # Assume if the first info filename matches the pattern, then ALL of them will.
        for p in patterns:
            first_prefix_match = match_run_seq(p, prefixes[0])
            if first_prefix_match:
                return [
                    fn for fn, pf in zip(fnames, prefixes)
                    if match_run_seq(p, pf) == (run, seq)
                ]

        return []

    def __read_info(self, info_file):
        """ Read the info file and load in the header information

        Args:
            info_file (str): unitary info filename

        Returns:
            (namedtuple) with the full header information

        Raises:
            Exception: unexpected file format
        """

        # Read all of the data into a list
        d = None
        with open(info_file, "r") as fheader:
            reader = csv.reader(fheader, delimiter="\t")
            d = list(reader)

        # Split the header file into its four parts:
        # filepaths, statistics, daqconfig, channelconfig

        # 1st line, 2nd column is the number of lines in the block (zero indexed)
        n_filepaths = int(d[0][1])
        # pop all lines one at a time, don't keep the line numbers at the beginning
        group_filepaths = [d.pop(0)[2:] for line in range(n_filepaths + 1)]

        n_statistics = int(d[0][1])
        group_statistics = [d.pop(0)[2:] for line in range(n_statistics + 1)]

        n_daqconfig = int(d[0][1])
        group_daqconfig = [d.pop(0)[2:] for line in range(n_daqconfig + 1)]

        n_channelconfig = int(d[0][1])
        group_channelconfig = [d.pop(0)[2:] for line in range(n_channelconfig + 1)]

        # Set up the output named tuple of dataframes
        filepaths_df = pd.DataFrame(group_filepaths, columns=group_filepaths.pop(0))

        statistics_df = pd.DataFrame(group_statistics, columns=group_statistics.pop(0))
        statistics_df = statistics_df.apply(pd.to_numeric, errors='ignore')

        daqconfig_df = pd.DataFrame(group_daqconfig, columns=group_daqconfig.pop(0))
        daqconfig_df = daqconfig_df.apply(pd.to_numeric, errors='ignore')

        chanconfig_df = pd.DataFrame(
            group_channelconfig,
            columns=group_channelconfig.pop(0)
        )

        # TODO discuss with Nate about this convention. We're trying to
        # bypass having to manually tell our processing code what Kulites
        # were plugged into what channel, and instead infer it from the
        # raw data.
        #
        # - Ideally, the "User Name" column is present, and contains the
        #   name of the Kulite that was plugged into that channel. (As is the
        #   case in the UPWT SLS data.)
        #
        # - In practice (e.g., in the TC3 2022-02 output data), the "User Name"
        #   column may not be present. In this case, we need some other way to
        #   find the Kulite/channel mapping.
        #
        # - For now, we assume that Kulites are named 'K01', 'K02', ..., where
        #   the numbering corresponds to what channel they are plugged into.
        #
        if 'User Name' not in chanconfig_df.columns:
            log.debug(''.join([
                '"User Name" column not found in %s. ',
                'Will infer Kulite-to-channel mapping: ',
                'K01 -> Channel 1, K02 -> Channel 2, etc.',
            ]), info_file)
            chanconfig_df['User Name'] = [
                'K%02d' % int(s) for s in chanconfig_df['User Channel']
            ]

        chanconfig_df = chanconfig_df.set_index('User Name')
        chanconfig_df = chanconfig_df.apply(pd.to_numeric, errors='ignore')

        # There are several iterations on the content contained in the
        # *.info file channel info block. In general, the number of
        # columns in the block indicates the "version."
        #
        # Most importantly, we need to distinguish whether the sensor
        # calibration information is present. When it *is* present, we
        # don't need to load it via the separate *.mat or *.zPXImat file.
        #
        # NOTE: column counts here differ from the actual # columns in
        # the *.info file:
        # - two leading columns containing line/section numbers are removed
        # - the 'index' is the "User Name" column, so it is also removed
        if len(chanconfig_df.columns) == 18:
            Version = 4
        elif len(chanconfig_df.columns) == 17:
            Version = 3
        else:
            if chanconfig_df.columns[1] == "User Channel":
                Version = 2
            elif chanconfig_df.columns[1] == "Physical Channel":
                Version = 1
            else:
                raise Exception(
                    "Can't determine file format. Did Kevin change it again?"
                )

        log.debug('Detected info file version: %d', Version)
        accepted_versions = [1, 4]
        if Version not in accepted_versions:
            raise Exception(
                "%s is version %d; only versions %s are accepted"
                % (info_file, Version, str(accepted_versions))
            )

        # Set up the output
        HeaderInfo = namedtuple(
            'HeaderInfo',
            ['filename', 'filepaths', 'statistics', 'daqconfig', 'chanconfig']
        )
        info = HeaderInfo(
            info_file, filepaths_df, statistics_df, daqconfig_df, chanconfig_df
        )

        return info

    def __read_data(self, info, channels, f_type):
        """ Read in the unitary data for the channels of interest
            and multiply by coefficients to dimensionalize

        Args:
            info (namedtuple)   : info header data
            channels (List)     : list of channel numbers (0-based) within file
            f_type (str)        : 'fast' or 'slow'

        Returns:
            (List) of numpy arrays with data in dB?

        Raises:
            RuntimeError: invalid channel id
        """

        # Identify data filename based on f_type:
        # 1. Load entry from info file for given f_type.
        # 2. If entry is not a valid path, then infer based on info filename.
        if f_type == 'fast':
            row_idx = 1
        elif f_type == 'slow':
            row_idx = 2
        filename = os.path.join(
            os.path.dirname(info.filename),
            info.filepaths['File Name'][row_idx]
        )
        if not os.path.exists(filename):
            info_prefix, _ = os.path.splitext(info.filename)
            inferred_filename = info_prefix + '.' + f_type
            # log.warning(
            #     '*.info points to non-existent *.%s file (%s); inferring: %s',
            #     f_type, filename, inferred_filename
            # )
            filename = inferred_filename

        def try_load_cal():
            prefix = os.path.splitext(filename)[0]
            cal_file = prefix + '.zPXImat'
            if not os.path.exists(cal_file):
                return None
            return scipy.io.loadmat(cal_file)

        raw = []
        native = []
        data = []

        # Get the frequency ID for the type of file
        if f_type == 'fast':
            freqID = 0
        elif f_type == 'slow':
            freqID = 1

        # shortcut to remove a bunch of typing
        stats = info.statistics
        chan = info.chanconfig

        nChannels = stats.loc[freqID, 'Number Channels']
        nSamples = stats.loc[freqID, 'Samples Acquired']

        # Read the full binary file
        bin_data = None
        with open(filename, 'rb') as f:
            dtypestr = '(1,)i4'
            dt = np.dtype(dtypestr)
            bin_data = np.fromfile(f, dtype=dt)
            bin_data = np.reshape(bin_data, (nSamples, nChannels))

        cal = try_load_cal()
        if cal:
            log.debug('calibration matrix: %s', str(cal['calibrationmatrix'].shape))

        # Perform import via numpy array
        # type is nChannel columns of int32 (i4)
        for channel in channels:
            if channel > nChannels:
                raise RuntimeError('Invalid channel ID, cannot read')

            vraw = bin_data[:, channel]

            # apply native scaling and native offset (counts to volts)
            vnative = vraw * chan['Coeff k1'][channel] + chan['Coeff k0'][channel]

            if cal:  # Use calibration data from separate file
                # % V/psi with gain sens.  Convert to Pa/V
                cal_term = cal['calibrationmatrix'][channel, 2]
                # TODO confirm magic number is conversion from psi to Pa.
                # 1 psi = 6894.757 Pa
                # We don't need the conversion because we want to keep data in psi.
                # micsens = 1 / (cal_term / 6894.757)
                micsens = 1 / cal_term
                vdata = vnative * micsens
            else:  # Use calibration data from chanconfig
                # Apply instrument sensitivity and offset (volts to engineering units)
                # I think this is in psi
                vdata = vnative * chan['EU C1'][channel] / \
                    (10**(chan['EU Gain (dB)'][channel] / 20)) + \
                    chan['EU C0'][channel]

            log.debug(
                'C[%02d]' % (channel + 1,), vraw.shape,
                'Counts:', f'{np.min(vraw):+10d}', f'{np.max(vraw):+10d}',
                'V:', f'{np.min(vnative):.3f}', f'{np.max(vnative):.3f}',
                'psi:', f'{np.min(vdata):.3f}', f'{np.max(vdata):.3f}',
            )
            raw.append(vraw)
            native.append(vnative)
            data.append(vdata)

        return raw, native, data


###############################################################################

def read_tgts(tgts_file, kulites='all'):
    """Read in the tgts file data and return the coordinates of the kulites

    Parameters
    ----------
    tgts_file : str
        targets data file
    kulites : list, optional
        list of kulite strings. If "all" (default), all targets identified as Kulites
        are returned.

    Returns
    -------
    pos : dict of list
        (x,y,z) positions for each kulite keyed by name
    """

    pos = dict()

    with open(tgts_file, 'r') as f:
        for line in f:
            vals = line.strip().split()

            # Fiducial marks do not have an extra column with the kulite name
            if len(vals) > 11 and 'K' in vals[11] and 'pK' not in vals[11] \
                    and 'mK' not in vals[11]:
                if kulites != 'all':
                    if vals[11] in kulites:
                        pos[vals[11]] = [float(vals[1]), float(vals[2]), float(vals[3])]
                else:
                    pos[vals[11]] = [float(vals[1]), float(vals[2]), float(vals[3])]
    return pos


def compute_delta_rms(kulites, sosfilter=None):
    """
    calculate rms values for each kulite by subtracting off the mean, and return a dict.
    if sosfilter is provided, applies it to the timeseries before calculating rms
    """

    def calc_rms(a):
        return np.sqrt(np.mean(a ** 2))

    rms_dict = dict()

    for name, data in kulites.data.items():
        if sosfilter is not None:
            filtered = signal.sosfilt(sosfilter, data)
        else:
            filtered = data

        rms_dict[name] = calc_rms(filtered - np.mean(filtered))

    return rms_dict


def compute_rms_from_psd(kulites, startfreq=None):
    """
    calculate rms values for each kulite  in the input collection by integrating its psd
    if startfreq is provided, integrates from startfreq upwards
    """
    rms_dict = dict()
    psds = compute_psd(kulites)
    freqs = psds['freq']

    for name, data in psds.items():
        if name == 'freq':
            continue

        if startfreq is None:
            rms_dict[name] = np.sum(data)
        else:
            i_desired = np.where(freqs >= startfreq)
            rms_dict[name] = np.sum(data[i_desired])

    return rms_dict


def apply_filter(kulites, sosfilter):
    """
    Apply an sos filter to the given kulite collection, and return a new kulites object.

    Creates a deep copy in case you want to use this for comparing filtered
    vs unfiltered collections, and want to hang on to an unmodified version too
    """
    filtered_kulites = copy.deepcopy(kulites)
    for name, timeseries in kulites.data.items():
        filtered_kulites.data[name] = signal.sosfilt(sosfilter, timeseries)
    return filtered_kulites


def compute_psd(kulites, w_len=1024):
    """Compute PSDs for kulite data

    Parameters
    ----------
    kulites : Kulites
        Kulites object
    w_len : int, optional
        window length

    Returns
    -------
    dict
        Dictionary of PSDs for each kulite, with an additional entry "freq" for the
        frequency bins
    """

    data = dict()

    # Compute PSD
    freq = None
    for key, val in kulites.data.items():
        a = val
        if len(val.shape) > 1:
            if val.shape[1] == 1:
                a = np.transpose(val)

        srate = kulites.sample_rate

        freq, data[key] = signal.welch(a, fs=srate,
                                       window='hanning', nperseg=w_len,
                                       detrend='linear')

    data['freq'] = freq

    return data


# FIXME this is broken?
def create_kulite_grid(tgts_file, kul_strs):
    """Generate a structured grid of kulite locations

    Parameters
    ----------
    tgts_file : str
        TGTS kulite location file
    kul_strs : list
        kulite names to include in plots

    Returns
    -------
    kul_grid : StructGrid
        kulite positions in cartesion and cylindrical form
    """
    # Load the kulite positions
    kulite_pos = kul_strs.read_tgts(tgts_file, kul_strs)

    # Generate grid
    kul_grid = tgts_file.StructGrid()
    kul_grid.sz.append([len(kul_strs), 1, 1])

    for kul, pos in kulite_pos.items():
        kul_grid.name.append(kul)
        kul_grid.x.append(pos[0])
        kul_grid.y.append(pos[1])
        kul_grid.z.append(pos[2])
    kul_grid.r, kul_grid.theta = p3d_utilities.to_cylindrical(kul_grid.y, kul_grid.z)

    return kul_grid


def write_kulite_positions_p3d(output_file, tgts_file, kul_strs):
    """Generate a 1D plot3d grid with kulite locations

    Parameters
    ----------
    output_file : str
        output p3d file
    tgts_file : str
        TGTS kulite location file
    kul_strs : list
        kulite names to include in plots
    """
    kul_grid = create_kulite_grid(tgts_file, kul_strs)
    kul_grid.write_p3d(output_file)


def read_targets_matrix(tgts_file, kul_strs):
    """Generate a list of kulite data

    Parameters
    ----------
    tgts_file : str
        TGTS kulite location file
    kul_strs : list
        kulite names to include in plots.

    Returns
    -------
    list of list
        Kulite data of the form: ``name, x, y, z, r, theta, i, j, k, diam, zone``
    """
    if kul_strs != 'all':
        mat = np.empty([len(kul_strs), 11], dtype=object)
        idx = 0
    else:
        mat = np.empty([0, 11], dtype=object)

    with open(tgts_file, 'r') as f:
        for line in f:
            vals = line.strip().split()

            # Only lines with all entries, and only kulites (not painted or masked)
            if len(vals) > 11 and 'K' in vals[11] and 'pK' not in vals[11] \
                    and 'mK' not in vals[11]:

                # Extract information
                x = vals[1]
                y = vals[2]
                z = vals[3]
                r_val, theta_val = p3d_utilities.to_cylindrical(float(y), float(z))
                r = str(r_val)
                theta = str(theta_val)
                i = vals[4]
                j = vals[5]
                k = vals[6]
                d = vals[7]
                zone = vals[8]

                if kul_strs != 'all':
                    if vals[11] in kul_strs:
                        mat[idx] = [vals[11], x, y, z, r, theta, i, j, k, d, zone]
                        idx = idx + 1
                else:
                    if vals[11] not in mat[:, 0]:
                        newrow = [vals[11], x, y, z, r, theta, i, j, k, d, zone]
                        mat = np.vstack([mat, newrow])

    # sort list of lists by first column
    mat_sort = mat[mat[:, 0].argsort()]

    return mat_sort


def create_pressure_array(kuls, data_type=np.float32):
    """Generate a numpy array with kulite pressure-time histories

    Parameters
    ----------
    kuls : dict
        kulite names linked to array of pressure values
    data_type : numpy.dtype
        data type of array elements

    Returns
    -------
    np.ndarray
        kulite data
    """
    size = len(kuls.data) * len(kuls.data[list(kuls.data.keys())[0]])
    leng = len(kuls.data[list(kuls.data.keys())[0]])
    data_array = np.empty(size, dtype=data_type)

    log.debug("Creating kulite pressure file of %d kulites, history of %d,"
             + " size %d, and type %s..",
             len(kuls.data), leng, size, data_type)

    idx = 0
    for kul in kuls.data:
        data_array[idx : idx + leng] = kuls.data[kul]
        idx = idx + leng

    return data_array
