"""Python module for reading cine files."""

import struct
from ctypes import (
    Structure,
    c_char,
    c_double,
    c_float,
    c_int16,
    c_int32,
    c_uint8,
    c_uint16,
    c_uint32,
    sizeof,
)

import numpy

from .base import VideoReader
from .util import unpack_10bpp, unpack_12bpp

MAXLENDESCRIPTION_OLD = 121
MAXLENDESCRIPTION = 4096
OLDMAXFILENAME = 65


class PrintableStructure(Structure):
    def __repr__(self):
        return "\n".join(
            "{}: {}".format(f[0], getattr(self, f[0])) for f in self._fields_
        )


class PrintableSubStructure(Structure):
    def __repr__(self):
        return "{}\n".format(self.__class__.__name__) + "\n".join(
            "\t{}: {}".format(f[0], getattr(self, f[0])) for f in self._fields_
        )


class TIME64(PrintableSubStructure):
    _fields_ = [
        ("fractions", c_uint32),
        ("seconds", c_uint32),
    ]


class TC(PrintableSubStructure):
    _fields_ = [
        ("framesU", c_uint8, 4),
        ("framesT", c_uint8, 2),
        ("dropFrameFlag", c_uint8, 1),
        ("colorFrameFlag", c_uint8, 1),
        ("secondsU", c_uint8, 4),
        ("secondsT", c_uint8, 3),
        ("flag1", c_uint8, 1),
        ("minutesU", c_uint8, 4),
        ("minutesT", c_uint8, 3),
        ("flag2", c_uint8, 1),
        ("hoursU", c_uint8, 4),
        ("hoursT", c_uint8, 2),
        ("flag3", c_uint8, 1),
        ("flag4", c_uint8, 1),
        ("userBitData", c_uint32),
    ]


class WBGAIN(PrintableSubStructure):
    _fields_ = [
        ("R", c_float),
        ("B", c_float),
    ]


class RECT(PrintableSubStructure):
    _fields_ = [
        ("left", c_int32),
        ("top", c_int32),
        ("right", c_int32),
        ("bottom", c_int32),
    ]


class IMFILTER(PrintableSubStructure):
    _fields_ = [
        ("dim", c_int32),
        ("shifts", c_int32),
        ("bias", c_int32),
        ("Coef", c_int32 * 25),
    ]


class CINEFILEHEADER(PrintableStructure):
    _fields_ = [
        ("Type", c_uint16),
        ("Headersize", c_uint16),
        ("Compression", c_uint16),
        ("Version", c_uint16),
        ("FirstMovieImage", c_int32),
        ("TotalImageCount", c_uint32),
        ("FirstImageNo", c_int32),
        ("ImageCount", c_uint32),
        ("OffImageHeader", c_uint32),
        ("OffSetup", c_uint32),
        ("OffImageOffsets", c_uint32),
        ("TriggerTime", TIME64),
    ]


class BITMAPINFOHEADER(PrintableStructure):
    _fields_ = [
        ("biSize", c_uint32),  # header size, 40
        ("biWidth", c_int32),  # bitmap width, pixels
        ("biHeight", c_int32),  # bitmap height, pixels
        ("biPlanes", c_uint16),  # always 1
        ("biBitCount", c_uint16),  # not real depth
        ("biCompression", c_uint32),  #
        ("biSizeImage", c_uint32),  # image size, bytes
        ("biXPelsPerMeter", c_int32),  # x pixels per meter
        ("biYPelsPerMeter", c_int32),  # y pixels per meter
        ("biClrUsed", c_uint32),  #
        ("biClrImportant", c_uint32),  # color indices for display
    ]


class SETUP(PrintableStructure):
    _pack_ = 1
    _fields_ = [
        ("FrameRate16", c_uint16),
        ("Shutter16", c_uint16),
        ("PostTrigger16", c_uint16),
        ("FrameDelay16", c_uint16),
        ("AspectRatio", c_uint16),
        ("Res7", c_uint16),
        ("Res8", c_uint16),
        ("Res9", c_uint8),
        ("Res10", c_uint8),
        ("Res11", c_uint8),
        ("TrigFrame", c_uint8),
        ("Res12", c_uint8),
        ("DescriptionOld", c_char * MAXLENDESCRIPTION_OLD),
        ("Mark", c_uint16),
        ("Length", c_uint16),
        ("Res13", c_uint16),
        ("SigOption", c_uint16),
        ("BinChannels", c_int16),
        ("SamplesPerImage", c_uint8),
        ("BinName", c_char * (8 * 11)),
        ("AnaOption", c_uint16),
        ("AnaChannels", c_int16),
        ("Res6", c_uint8),
        ("AnaBoard", c_uint8),
        ("ChOption", c_int16 * 8),
        ("AnaGain", c_float * 8),
        ("AnaUnit", c_char * (8 * 6)),
        ("AnaName", c_char * (8 * 11)),
        ("lFirstImage", c_int32),
        ("dwImageCount", c_uint32),
        ("nQFactor", c_int16),
        ("wCineFileType", c_uint16),
        ("szCinePath", c_char * (4 * OLDMAXFILENAME)),
        ("Res14", c_uint16),
        ("Res15", c_uint8),
        ("Res16", c_uint8),
        ("Res17", c_uint16),
        ("Res18", c_double),
        ("Res19", c_double),
        ("Res20", c_uint16),
        ("Res1", c_int32),
        ("Res2", c_int32),
        ("Res3", c_int32),
        ("ImWidth", c_uint16),
        ("ImHeight", c_uint16),
        ("EDRShutter16", c_uint16),
        ("Serial", c_uint32),
        ("Saturation", c_int32),
        ("Res5", c_uint8),
        ("AutoExposure", c_uint32),
        ("bFlipH", c_uint32),
        ("bFlipV", c_uint32),
        ("Grid", c_uint32),
        ("FrameRate", c_uint32),
        ("Shutter", c_uint32),
        ("EDRShutter", c_uint32),
        ("PostTrigger", c_uint32),
        ("FrameDelay", c_uint32),
        ("bEnableColor", c_uint32),
        ("CameraVersion", c_uint32),
        ("FirmwareVersion", c_uint32),
        ("SoftwareVersion", c_uint32),
        ("RecordingTimeZone", c_int32),
        ("CFA", c_uint32),
        ("Bright", c_int32),
        ("Contrast", c_int32),
        ("Gamma", c_int32),
        ("Res21", c_uint32),
        ("AutoExpLevel", c_uint32),
        ("AutoExpSpeed", c_uint32),
        ("AutoExpRect", RECT),
        ("WBGain", WBGAIN * 4),
        ("Rotate", c_int32),
        ("WBView", WBGAIN),
        ("RealBPP", c_uint32),
        ("Conv8Min", c_uint32),
        ("Conv8Max", c_uint32),
        ("FilterCode", c_int32),
        ("FilterParam", c_int32),
        ("UF", IMFILTER),
        ("BlackCalSVer", c_uint32),
        ("WhiteCalSVer", c_uint32),
        ("GrayCalsVer", c_uint32),
        ("bStampTime", c_uint32),
        ("SoundDest", c_uint32),
        ("FRPSteps", c_uint32),
        ("FRPImgNr", c_int32 * 16),
        ("FRPRate", c_uint32 * 16),
        ("FRPExp", c_uint32 * 16),
        ("MCCnt", c_int32),
        ("MCPercent", c_float * 64),
        ("CICAlib", c_uint32),
        ("CalibWidth", c_uint32),
        ("CalibHeight", c_uint32),
        ("CalibRate", c_uint32),
        ("CalibExp", c_uint32),
        ("CalibEDR", c_uint32),
        ("CalibTemp", c_uint32),
        ("HeadSerial", c_uint32 * 4),
        ("RangeCode", c_uint32),
        ("RangeSize", c_uint32),
        ("Decimation", c_uint32),
        ("MasterSerial", c_uint32),
        ("Sensor", c_uint32),
        ("ShutterNs", c_uint32),
        ("EDRShutterNs", c_uint32),
        ("FrameDelayNs", c_uint32),
        ("ImPosXAcq", c_uint32),
        ("ImPosYAcq", c_uint32),
        ("ImWidthAcq", c_uint32),
        ("ImHeightAcq", c_uint32),
        ("Description", c_char * MAXLENDESCRIPTION),
        ("RisingEdge", c_uint32),
        ("FilterTime", c_uint32),
        ("LongReady", c_uint32),
        ("ShutterOff", c_uint32),
        ("Res4", c_uint8 * 16),
        ("bMetaWB", c_uint32),
        ("Hue", c_int32),
        ("BlackLevel", c_int32),
        ("WhiteLevel", c_int32),
        ("LensDescription", c_char * 256),
        ("LensAperture", c_float),
        ("LensFocusDistance", c_float),
        ("LensFocalLength", c_float),
        ("fOffset", c_float),
        ("fGain", c_float),
        ("fSaturation", c_float),
        ("fHue", c_float),
        ("fGamma", c_float),
        ("fGammaR", c_float),
        ("fGammaB", c_float),
        ("fFlare", c_float),
        ("fPedestalR", c_float),
        ("fPedestalG", c_float),
        ("fPedestalB", c_float),
        ("fChroma", c_float),
        ("ToneLabel", c_char * 256),
        ("TonePoints", c_int32),
        ("fTone", c_float * (32 * 2)),
        ("UserMatrixLabel", c_char * 256),
        ("EnableMatrices", c_uint32),
        ("fUserMatrix", c_float * 9),
        ("EnableCrop", c_uint32),
        ("CropRect", RECT),
        ("EnableResample", c_uint32),
        ("ResampleWidth", c_uint32),
        ("ResampleHeight", c_uint32),
        ("fGain16_8", c_float),
        ("FRPShape", c_uint32 * 16),
        ("TrigTC", TC),
        ("fPbRate", c_float),
        ("fTcRate", c_float),
        ("CineName", c_char * 256),
    ]


class CineReader(VideoReader):
    """Phantom Cine file reader.

    Attributes
    ----------
    cfinfo : CINEFILEHEADER
        All data contained in the cine file header.
    bminfo : BITMAPINFOHEADER
        All data contained in the bitmap info header.
    setup : SETUP
        All data contained in the setup header.
    tagged_info : dict
        Dictionary of tagged information blocks. Types depend on the data stored in the
        file when the cine was captured.
    """

    # tagged information block type code -> name, data type (per frame)
    tinfo_types = {
        1002: ("time_only", TIME64),
        1003: ("exposure", c_uint32),
        1007: ("timecode", TC),
    }

    def initialize(self):
        """Read cine file headers, annotations, and offsets"""
        self._read_headers()
        if self.cfinfo.OffSetup + self.setup.Length < self.cfinfo.OffImageOffsets:
            self._read_annotations()
        self._read_offsets()

    @property
    def width(self):
        return self.bminfo.biWidth

    @property
    def height(self):
        return self.bminfo.biHeight

    @property
    def bit_depth(self):
        # special case 10 bit with LUT expansion
        return 12 if self.raw_bit_depth == 10 else self.raw_bit_depth

    @property
    def raw_bit_depth(self):
        return self.setup.RealBPP

    @property
    def frame_rate(self):
        return self.setup.FrameRate

    @property
    def frame_count(self):
        return self.cfinfo.ImageCount

    @property
    def _packed(self):
        """Whether pixel values in the file are packed or not"""
        # doc says packed if this field == 256, but some have 1024
        return self.bminfo.biCompression != 0

    def _read_headers(self):
        """Read header data from the beginning of the file.

        Fields are read into 3 different structs (cfinfo, bminfo, and setup). These are
        documented in the Vision Research Cine File Format document.
        """
        self.cfinfo = CINEFILEHEADER()
        self.bminfo = BITMAPINFOHEADER()
        self.setup = SETUP()

        self.fd.readinto(self.cfinfo)
        self.fd.readinto(self.bminfo)
        self.fd.readinto(self.setup)

        bpp = self.setup.RealBPP
        if bpp not in (10, 12):
            raise NotImplementedError(
                f"Unpacking frames with bit depth {bpp} not supported yet"
            )

    def _read_annotations(self):
        """Read all tagged information blocks into a dict."""
        self.tagged_info = {}
        self.fd.seek(self.cfinfo.OffSetup + self.setup.Length)
        while self.fd.tell() < self.cfinfo.OffImageOffsets:
            bsize = struct.unpack("<I", self.fd.read(4))[0]
            btype = struct.unpack("<H", self.fd.read(2))[0]
            self.fd.read(2)  # reserved
            dsize = bsize - 8

            if btype not in self.tinfo_types:
                self.fd.seek(dsize, 1)
                continue

            name, typ = self.tinfo_types[btype]
            data = (self.cfinfo.ImageCount * typ)()

            if sizeof(data) != dsize:
                self.fd.seek(dsize, 1)
                continue

            self.fd.readinto(data)
            self.tagged_info[name] = data

    def _read_offsets(self):
        """Read image offsets table. Each offset value is 64 bits."""
        self.fd.seek(self.cfinfo.OffImageOffsets)
        nimgs = self.cfinfo.ImageCount
        # q : unsigned long long
        self._img_offsets = struct.unpack("<{}q".format(nimgs), self.fd.read(8 * nimgs))

    def read_frame(self, idx):
        """Read a single frame by index.

        Parameters
        ----------
        idx : int
            Index of the frame to read.

        Returns
        -------
        img : np.ndarray, shape (height, width)
        """
        self._validate_index(idx)

        # seek to image annotation
        self.fd.seek(self._img_offsets[idx])
        # read image annotation size as a uint32
        annot_size = struct.unpack("<I", self.fd.read(4))[0]
        # skip the annotation to img size, relative to current position
        # annotation size already read
        self.fd.seek(annot_size - 8, 1)
        # read image size
        img_size = struct.unpack("<I", self.fd.read(4))[0]
        # read image data
        buf = self.fd.read(img_size)

        if self._packed:
            if self.raw_bit_depth == 12:
                pix = unpack_12bpp(buf)
            else:
                pix = _LUT_10BIT[unpack_10bpp(buf)]
        else:
            pix = numpy.frombuffer(buf, dtype=numpy.uint16)

        img = pix.reshape((self.height, self.width))
        return img


# lookup table to convert from 10-bit packed to 12-bit linear
# fmt: off
_LUT_10BIT = numpy.array([
       2,    5,    6,    7,    8,    9,   10,   11,   12,   13,   14,   15,   16,   17,   17,   18,
      19,   20,   21,   22,   23,   24,   25,   26,   27,   28,   29,   30,   31,   32,   33,   33,
      34,   35,   36,   37,   38,   39,   40,   41,   42,   43,   44,   45,   46,   47,   48,   48,
      49,   50,   51,   52,   53,   54,   55,   56,   57,   58,   59,   60,   61,   62,   63,   63,
      64,   65,   66,   67,   68,   69,   70,   71,   72,   73,   74,   75,   76,   77,   78,   79,
      79,   80,   81,   82,   83,   84,   85,   86,   87,   88,   89,   90,   91,   92,   93,   94,
      94,   95,   96,   97,   98,   99,  100,  101,  102,  103,  104,  105,  106,  107,  108,  109,
     110,  110,  111,  112,  113,  114,  115,  116,  117,  118,  119,  120,  121,  122,  123,  124,
     125,  125,  126,  127,  128,  129,  130,  131,  132,  133,  134,  135,  136,  137,  137,  138,
     139,  140,  141,  142,  143,  144,  145,  146,  147,  148,  149,  150,  151,  152,  153,  154,
     156,  157,  158,  159,  160,  161,  162,  163,  164,  165,  167,  168,  169,  170,  171,  172,
     173,  175,  176,  177,  178,  179,  181,  182,  183,  184,  186,  187,  188,  189,  191,  192,
     193,  194,  196,  197,  198,  200,  201,  202,  204,  205,  206,  208,  209,  210,  212,  213,
     215,  216,  217,  219,  220,  222,  223,  225,  226,  227,  229,  230,  232,  233,  235,  236,
     238,  239,  241,  242,  244,  245,  247,  249,  250,  252,  253,  255,  257,  258,  260,  261,
     263,  265,  266,  268,  270,  271,  273,  275,  276,  278,  280,  281,  283,  285,  287,  288,
     290,  292,  294,  295,  297,  299,  301,  302,  304,  306,  308,  310,  312,  313,  315,  317,
     319,  321,  323,  325,  327,  328,  330,  332,  334,  336,  338,  340,  342,  344,  346,  348,
     350,  352,  354,  356,  358,  360,  362,  364,  366,  368,  370,  372,  374,  377,  379,  381,
     383,  385,  387,  389,  391,  394,  396,  398,  400,  402,  404,  407,  409,  411,  413,  416,
     418,  420,  422,  425,  427,  429,  431,  434,  436,  438,  441,  443,  445,  448,  450,  452,
     455,  457,  459,  462,  464,  467,  469,  472,  474,  476,  479,  481,  484,  486,  489,  491,
     494,  496,  499,  501,  504,  506,  509,  511,  514,  517,  519,  522,  524,  527,  529,  532,
     535,  537,  540,  543,  545,  548,  551,  553,  556,  559,  561,  564,  567,  570,  572,  575,
     578,  581,  583,  586,  589,  592,  594,  597,  600,  603,  606,  609,  611,  614,  617,  620,
     623,  626,  629,  632,  635,  637,  640,  643,  646,  649,  652,  655,  658,  661,  664,  667,
     670,  673,  676,  679,  682,  685,  688,  691,  694,  698,  701,  704,  707,  710,  713,  716,
     719,  722,  726,  729,  732,  735,  738,  742,  745,  748,  751,  754,  758,  761,  764,  767,
     771,  774,  777,  781,  784,  787,  790,  794,  797,  800,  804,  807,  811,  814,  817,  821,
     824,  828,  831,  834,  838,  841,  845,  848,  852,  855,  859,  862,  866,  869,  873,  876,
     880,  883,  887,  890,  894,  898,  901,  905,  908,  912,  916,  919,  923,  927,  930,  934,
     938,  941,  945,  949,  952,  956,  960,  964,  967,  971,  975,  979,  982,  986,  990,  994,
     998, 1001, 1005, 1009, 1013, 1017, 1021, 1025, 1028, 1032, 1036, 1040, 1044, 1048, 1052, 1056,
    1060, 1064, 1068, 1072, 1076, 1080, 1084, 1088, 1092, 1096, 1100, 1104, 1108, 1112, 1116, 1120,
    1124, 1128, 1132, 1137, 1141, 1145, 1149, 1153, 1157, 1162, 1166, 1170, 1174, 1178, 1183, 1187,
    1191, 1195, 1200, 1204, 1208, 1212, 1217, 1221, 1225, 1230, 1234, 1238, 1243, 1247, 1251, 1256,
    1260, 1264, 1269, 1273, 1278, 1282, 1287, 1291, 1295, 1300, 1304, 1309, 1313, 1318, 1322, 1327,
    1331, 1336, 1340, 1345, 1350, 1354, 1359, 1363, 1368, 1372, 1377, 1382, 1386, 1391, 1396, 1400,
    1405, 1410, 1414, 1419, 1424, 1428, 1433, 1438, 1443, 1447, 1452, 1457, 1462, 1466, 1471, 1476,
    1481, 1486, 1490, 1495, 1500, 1505, 1510, 1515, 1520, 1524, 1529, 1534, 1539, 1544, 1549, 1554,
    1559, 1564, 1569, 1574, 1579, 1584, 1589, 1594, 1599, 1604, 1609, 1614, 1619, 1624, 1629, 1634,
    1639, 1644, 1649, 1655, 1660, 1665, 1670, 1675, 1680, 1685, 1691, 1696, 1701, 1706, 1711, 1717,
    1722, 1727, 1732, 1738, 1743, 1748, 1753, 1759, 1764, 1769, 1775, 1780, 1785, 1791, 1796, 1801,
    1807, 1812, 1818, 1823, 1828, 1834, 1839, 1845, 1850, 1856, 1861, 1867, 1872, 1878, 1883, 1889,
    1894, 1900, 1905, 1911, 1916, 1922, 1927, 1933, 1939, 1944, 1950, 1956, 1961, 1967, 1972, 1978,
    1984, 1989, 1995, 2001, 2007, 2012, 2018, 2024, 2030, 2035, 2041, 2047, 2053, 2058, 2064, 2070,
    2076, 2082, 2087, 2093, 2099, 2105, 2111, 2117, 2123, 2129, 2135, 2140, 2146, 2152, 2158, 2164,
    2170, 2176, 2182, 2188, 2194, 2200, 2206, 2212, 2218, 2224, 2231, 2237, 2243, 2249, 2255, 2261,
    2267, 2273, 2279, 2286, 2292, 2298, 2304, 2310, 2317, 2323, 2329, 2335, 2341, 2348, 2354, 2360,
    2366, 2373, 2379, 2385, 2392, 2398, 2404, 2411, 2417, 2423, 2430, 2436, 2443, 2449, 2455, 2462,
    2468, 2475, 2481, 2488, 2494, 2501, 2507, 2514, 2520, 2527, 2533, 2540, 2546, 2553, 2559, 2566,
    2572, 2579, 2586, 2592, 2599, 2605, 2612, 2619, 2625, 2632, 2639, 2645, 2652, 2659, 2666, 2672,
    2679, 2686, 2693, 2699, 2706, 2713, 2720, 2726, 2733, 2740, 2747, 2754, 2761, 2767, 2774, 2781,
    2788, 2795, 2802, 2809, 2816, 2823, 2830, 2837, 2844, 2850, 2857, 2864, 2871, 2878, 2885, 2893,
    2900, 2907, 2914, 2921, 2928, 2935, 2942, 2949, 2956, 2963, 2970, 2978, 2985, 2992, 2999, 3006,
    3013, 3021, 3028, 3035, 3042, 3049, 3057, 3064, 3071, 3078, 3086, 3093, 3100, 3108, 3115, 3122,
    3130, 3137, 3144, 3152, 3159, 3166, 3174, 3181, 3189, 3196, 3204, 3211, 3218, 3226, 3233, 3241,
    3248, 3256, 3263, 3271, 3278, 3286, 3294, 3301, 3309, 3316, 3324, 3331, 3339, 3347, 3354, 3362,
    3370, 3377, 3385, 3393, 3400, 3408, 3416, 3423, 3431, 3439, 3447, 3454, 3462, 3470, 3478, 3486,
    3493, 3501, 3509, 3517, 3525, 3533, 3540, 3548, 3556, 3564, 3572, 3580, 3588, 3596, 3604, 3612,
    3620, 3628, 3636, 3644, 3652, 3660, 3668, 3676, 3684, 3692, 3700, 3708, 3716, 3724, 3732, 3740,
    3749, 3757, 3765, 3773, 3781, 3789, 3798, 3806, 3814, 3822, 3830, 3839, 3847, 3855, 3863, 3872,
    3880, 3888, 3897, 3905, 3913, 3922, 3930, 3938, 3947, 3955, 3963, 3972, 3980, 3989, 3997, 4006,
    4014, 4022, 4031, 4039, 4048, 4056, 4064, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
])
# fmt: on
