/** @file
 *  @brief  Cine File Reader
 *  @date   Apr 13, 2016
 *  @author smurman
 */

#include "CineReader.h"

#include <fstream>
#include <ctime>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

#include "vr_cine.h"

namespace upsp {

/*****************************************************************************/
uint16_t CINE2_LUT[1024] = {
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
4014, 4022, 4031, 4039, 4048, 4056, 4064, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};

/*****************************************************************************/
CineReader::CineReader(const std::string& filepath)
    : filepath_(filepath),
      ifs_(filepath, std::ios::binary) {
    if (!ifs_) {
        throw std::invalid_argument("Video File is invalid");
    }
}

/*****************************************************************************/
CineReader::~CineReader() {
    if (ifs_.is_open()) {
        ifs_.close();
    }
}

/*****************************************************************************/
void CineReader::read_offsets() {
    ifs_.seekg(cine_header_.OffImageOffsets);
    image_offsets_.resize(cine_header_.ImageCount);
    uint64_t nbytes = sizeof(uint64_t) * cine_header_.ImageCount;
    ifs_.read((char*)&image_offsets_[0], nbytes);

    curr_offset_ = cine_header_.OffImageOffsets + nbytes;

    // we assume identical frames
    if (cine_header_.ImageCount > 1)
    CV_Assert((image_offsets_[1] - image_offsets_[0]) ==
        (image_offsets_[2] - image_offsets_[1]));

    int frame_size = (properties_->width * properties_->height * bits_per_pixel_) / 8;
    size_t tot_size = image_offsets_[cine_header_.ImageCount - 1] - image_offsets_[0];
    size_t exp_tot_size = (cine_header_.ImageCount - 1) *
      (frame_size + 2 * sizeof(uint32_t));
    CV_Assert(tot_size == exp_tot_size);

    // leave setup to start reading frames
    ifs_.seekg(image_offsets_[0] + 2 * sizeof(uint32_t));
    wanted_offset_ = image_offsets_[0] + 2 * sizeof(uint32_t);
}


/*****************************************************************************/
void CineReader::load_properties(VideoProperties &props) {
    properties_ = &props;
    ifs_.read((char*)&cine_header_, sizeof(cine_header_));
    ifs_.read((char*)&bitmap_header_, sizeof(bitmap_header_));
    ifs_.read((char*)&setup_, sizeof(setup_));

    properties_->num_frames = cine_header_.ImageCount;
    properties_->frame_rate = setup_.FrameRate16;
    properties_->aperture = setup_.LensAperture;
    properties_->width = setup_.ImWidth;
    properties_->height = setup_.ImHeight;
    bits_per_pixel_ = setup_.RealBPP;

    // read tagged blocks if available
    if ( (cine_header_.OffSetup + setup_.Length) < cine_header_.OffImageOffsets) {
        ifs_.seekg(cine_header_.OffSetup + setup_.Length);
        uint32_t BlockSize;
        uint16_t Type;
        uint16_t Reserved;

        while (ifs_.tellg() < cine_header_.OffImageOffsets) {
            ifs_.read((char*)&BlockSize, sizeof(BlockSize));
            ifs_.read((char*)&Type, sizeof(Type));
            ifs_.read((char*)&Reserved, sizeof(Reserved));

            if (Type == 0x3eb) {
                int len = (BlockSize - 8) / 4;
                uint32_t* Exposures = new uint32_t[len];
                ifs_.read((char*) Exposures, BlockSize - 8);

                properties_->exposure = Exposures[0] / (float) std::pow(2,32);
                properties_->exposure *= 1e6; // seconds to microseconds

                delete[] Exposures;
            } else {
                ifs_.seekg(BlockSize - 8, std::ios_base::cur);
            }
        }
    }

    read_offsets();

    // compressed 10 bit -> uncompressed 12 bit
    properties_->bit_depth = (bits_per_pixel_ == 10) ? 12 : bits_per_pixel_;
}

void CineReader::summarize() {
    std::cout << "ImageCount " << cine_header_.ImageCount << std::endl;
    time_t tigger = (time_t)cine_header_.TriggerTime.seconds;
    std::cout << "TriggerTime " << ctime(&tigger) << std::endl;
    std::cout << "OffSetup " << cine_header_.OffSetup << std::endl;
    std::cout << "sizeof(SETUP) " << sizeof(SETUP) << std::endl;
    std::cout << "OffImageOffsets " << cine_header_.OffImageOffsets << std::endl;

    std::cout << "Width " << bitmap_header_.biWidth << std::endl;
    std::cout << "Height " << bitmap_header_.biHeight << std::endl;
    std::cout << "Bits " << bitmap_header_.biBitCount << std::endl;
    std::cout << "ClrImportant " << bitmap_header_.biClrImportant << std::endl;

    std::cout << "RealBPP " << setup_.RealBPP << std::endl;
    std::cout << "ResampleWidth " << setup_.ResampleWidth << std::endl;
    std::cout << "ResampleHeight " << setup_.ResampleHeight << std::endl;
    std::cout << "fGain16_8 " << setup_.fGain16_8 << std::endl;
    std::cout << "CameraVersion " << setup_.CameraVersion << std::endl;
    std::cout << "FrameRate " << setup_.FrameRate << std::endl;
    std::cout << "ShutterNs " << setup_.ShutterNs << std::endl;
    std::cout << "LensAperture " << setup_.LensAperture << std::endl;
    std::cout << "CineName " << setup_.CineName << std::endl;

    std::cout << "Exposure " << properties_->exposure << std::endl;
}

/*****************************************************************************/
void CineReader::write_header(std::ofstream& ofs) {
    ofs.write((char*)&cine_header_, sizeof(cine_header_));
    ofs.write((char*)&bitmap_header_, sizeof(bitmap_header_));
    ofs.write((char*)&setup_, sizeof(setup_));
}

unsigned int CineReader::get_bits_per_pixel() const {
    return bits_per_pixel_;
}

/*****************************************************************************/
void CineReader::dump_header(std::ostream& out) const {

    // arrays and strings are not currently printed
    char buf[1000];

    // CINEFILEHEADER;
    out << "Type: " << cine_header_.Type << '\n' <<
        "Headersize: " << cine_header_.Headersize << '\n' <<
        "Compression: " << cine_header_.Compression << '\n' <<
        "Version: " << cine_header_.Version << '\n' <<
        "FirstMovieImage: " << cine_header_.FirstMovieImage << '\n' <<
        "TotalImageCount: " << cine_header_.TotalImageCount << '\n' <<
        "FirstImageNo: " << cine_header_.FirstImageNo << '\n' <<
        "ImageCount: " << cine_header_.ImageCount << '\n' <<
        "OffImageHeader: " << cine_header_.OffImageHeader << '\n' <<
        "OffSetup: " << cine_header_.OffSetup << '\n' <<
        "OffImageOffsets: " << cine_header_.OffImageOffsets << std::endl;

    // print out time in user-friendly format with nanosecond precision
    // first use a stringstream to get the fraction then skip the leading 0
    std::ostringstream oss;
    oss.precision(9);
    oss << std::fixed << std::showpoint;
    oss << (cine_header_.TriggerTime.fractions & ~0b11) / 4294967296.0;
    const std::string s = oss.str();
    const char* p = s.c_str();
    const char* fract_str = p+1;
    // get user-friendly date and time, print it with fractional seconds
        time_t trigger = (time_t)cine_header_.TriggerTime.seconds;
    struct tm* ltime = localtime(&trigger);
    strftime(buf, 1000, "%a, %d %b %Y %T", ltime);
    out << "TriggerTime: " << buf << fract_str;
    // get & print time zone offset by itself
    strftime(buf, 1000, " %z", ltime);
    out << buf << " sync " << (cine_header_.TriggerTime.fractions & 0b11) << std::endl;

    // BITMAPINFOHEADER;
    out << "biSize: " << bitmap_header_.biSize << '\n' <<
        "biWidth: " << bitmap_header_.biWidth << '\n' <<
        "biHeight: " << bitmap_header_.biHeight << '\n' <<
        "biPlanes: " << bitmap_header_.biPlanes << '\n' <<
        "biBitCount: " << bitmap_header_.biBitCount << '\n' <<
        "biCompression: " << bitmap_header_.biCompression << '\n' <<
        "biSizeImage: " << bitmap_header_.biSizeImage << '\n' <<
        "biXPelsPerMeter: " << bitmap_header_.biXPelsPerMeter << '\n' <<
        "biYPelsPerMeter: " << bitmap_header_.biYPelsPerMeter << '\n' <<
        "biClrUsed: " << bitmap_header_.biClrUsed << '\n' <<
        "biClrImportant: " << bitmap_header_.biClrImportant << std::endl;

    // SETUP
    // Note: fields marked "UPDF" - Updated Field and "TBI" - to be ignored
    // are not printed.
    out << "TrigFrame: " << int(setup_.TrigFrame) << '\n' <<
        "Mark: " << setup_.Mark << '\n' <<
        "Length: " << setup_.Length << '\n' <<
        "SigOption: " << setup_.SigOption << '\n' <<
        "BinChannels: " << setup_.BinChannels << '\n' <<
        "SamplesPerImage: " << int(setup_.SamplesPerImage) << '\n' <<
        //char BinName[8][11]; // Names for the first 8 binary signals having
        "AnaOption: " << setup_.AnaOption << '\n' <<
        "AnaChannels: " << setup_.AnaChannels << '\n' <<
        "AnaBoard: " << int(setup_.AnaBoard) << '\n' <<
        //int16_t ChOption[8]; // Per channel analog options;
        //float AnaGain[8];    // User gain correction for conversion from voltage
        //char AnaUnit[8][6];  // Measurement unit for analog channels: max 5
        //char AnaName[8][11]; // Channel name for the first 8 analog channels:
        "lFirstImage: " << setup_.lFirstImage << '\n' <<
        "dwImageCount: " << setup_.dwImageCount << '\n' <<
        "nQFactor: " << setup_.nQFactor << '\n' <<
        "wCineFileType: " << setup_.wCineFileType << '\n' <<
        //char szCinePath[4][OLDMAXFILENAME];
        "ImWidth: " << setup_.ImWidth << '\n' <<
        "ImHeight: " << setup_.ImHeight << '\n' <<
        "EDRShutter16: " << setup_.EDRShutter16 << '\n' <<
        "Serial: " << setup_.Serial << '\n' <<
        "Saturation: " << setup_.Saturation << '\n' <<
        "AutoExposure: " << setup_.AutoExposure << '\n' <<
        "bFlipH: " << setup_.bFlipH << '\n' <<
        "bFlipV: " << setup_.bFlipV << '\n' <<
        "Grid: " << setup_.Grid << '\n' <<
        "FrameRate: " << setup_.FrameRate << '\n' <<
        "Shutter: " << setup_.Shutter << '\n' <<
        "EDRShutter: " << setup_.EDRShutter << '\n' <<
        "PostTrigger: " << setup_.PostTrigger << '\n' <<
        "FrameDelay: " << setup_.FrameDelay << '\n' <<
        "bEnableColor: " << setup_.bEnableColor << '\n' <<
        "CameraVersion: " << setup_.CameraVersion << '\n' <<
        "FirmwareVersion: " << setup_.FirmwareVersion << '\n' <<
        "SoftwareVersion: " << setup_.SoftwareVersion << '\n' <<
        "RecordingTimeZone: " << setup_.RecordingTimeZone << '\n' <<
        "CFA: " << setup_.CFA << '\n' <<
        "Bright: " << setup_.Bright << '\n' <<
        "Contrast: " << setup_.Contrast << '\n' <<
        "Gamma: " << setup_.Gamma << '\n' <<
        "AutoExpLevel: " << setup_.AutoExpLevel << '\n' <<
        "AutoExpSpeed: " << setup_.AutoExpSpeed << '\n' <<
        //RECT AutoExpRect; // Rectangle for autoexposure control
        //WBGAIN WBGain[4]; // Gain adjust on R,B components, for white balance,
        "Rotate: " << setup_.Rotate << '\n' <<
        //WBGAIN WBView;  // White balance to apply on color interpolated Cines
        "RealBPP: " << setup_.RealBPP << '\n' <<
        "Conv8Max: " << setup_.Conv8Max << '\n' <<
        "FilterCode: " << setup_.FilterCode << '\n' <<
        "FilterParam: " << setup_.FilterParam << '\n' <<
        //IMFILTER UF; // User filter: a 3x3 or 5x5 user convolution filter
        "BlackCalSVer: " << setup_.BlackCalSVer << '\n' <<
        "WhiteCalSVer: " << setup_.WhiteCalSVer << '\n' <<
        "GrayCalSVer: " << setup_.GrayCalSVer << '\n' <<
        "bStampTime: " << setup_.bStampTime << '\n' <<
        "SoundDest: " << setup_.SoundDest << '\n' <<
        "FRPSteps: " << setup_.FRPSteps << '\n' <<
        //int32_t FRPImgNr[16]; // Image number where to change the rate and/or
        //uint32_t FRPRate[16]; // New value for frame rate (fps)
        //uint32_t FRPExp[16]; // New value for exposure
        //Multicine partition
        "MCCnt: " << setup_.MCCnt << '\n' <<
        //float MCPercent[64]; // Percent of memory used for partitions
        "CICalib: " << setup_.CICalib << '\n' <<
        "CalibWidth: " << setup_.CalibWidth << '\n' <<
        "CalibHeight: " << setup_.CalibHeight << '\n' <<
        "CalibRate: " << setup_.CalibRate << '\n' <<
        "CalibExp: " << setup_.CalibExp << '\n' <<
        "CalibEDR: " << setup_.CalibEDR << '\n' <<
        "CalibTemp: " << setup_.CalibTemp << '\n' <<
        //uint32_t HeadSerial[4]; // Head serials for ethernet multihead cameras
        "RangeCode: " << setup_.RangeCode << '\n' <<
        "RangeSize: " << setup_.RangeSize << '\n' <<
        "Decimation: " << setup_.Decimation << '\n' <<
        "MasterSerial: " << setup_.MasterSerial << '\n' <<
        "Sensor: " << setup_.Sensor << '\n' <<
        "ShutterNs: " << setup_.ShutterNs << '\n' <<
        "EDRShutterNs: " << setup_.EDRShutterNs << '\n' <<
        "FrameDelayNs: " << setup_.FrameDelayNs << '\n' <<
        "ImPosXAcq: " << setup_.ImPosXAcq << '\n' <<
        "ImPosYAcq: " << setup_.ImPosYAcq << '\n' <<
        "ImWidthAcq: " << setup_.ImWidthAcq << '\n' <<
        "ImHeightAcq: " << setup_.ImHeightAcq << '\n' <<
        //char Description[MAXLENDESCRIPTION];//User description or comments
        "RisingEdge: " << setup_.RisingEdge << '\n' <<
        "FilterTime: " << setup_.FilterTime << '\n' <<
        "LongReady: " << setup_.LongReady << '\n' <<
        "ShutterOff: " << setup_.ShutterOff << '\n' <<
        "bMetaWB: " << setup_.bMetaWB << '\n' <<
        "Hue: " << setup_.Hue << '\n' <<
        "BlackLevel: " << setup_.BlackLevel << '\n' <<
        "WhiteLevel: " << setup_.WhiteLevel << '\n' <<
        //char LensDescription[256];// text with the producer, model,
        "LensAperture: " << setup_.LensAperture << '\n' <<
        "LensFocusDistance: " << setup_.LensFocusDistance << '\n' <<
        "LensFocalLength: " << setup_.LensFocalLength << '\n' <<
        "fOffset: " << setup_.fOffset << '\n' <<
        "fGain: " << setup_.fGain << '\n' <<
        "fSaturation: " << setup_.fSaturation << '\n' <<
        "fHue: " << setup_.fHue << '\n' <<
        "fGamma: " << setup_.fGamma << '\n' <<
        "fGammaR: " << setup_.fGammaR << '\n' <<
        "fGammaB: " << setup_.fGammaB << '\n' <<
        "fFlare: " << setup_.fFlare << '\n' <<
        "fPedestalR: " << setup_.fPedestalR << '\n' <<
        "fPedestalG: " << setup_.fPedestalG << '\n' <<
        "fPedestalB: " << setup_.fPedestalB << '\n' <<
        "fChroma: " << setup_.fChroma << '\n' <<
        //char  ToneLabel[256];
        "TonePoints: " << setup_.TonePoints << '\n' <<
        //float fTone[32*2]; // up to 32  points  + 0.0,0.0  1.0,1.0
        //char UserMatrixLabel[256];
        "EnableMatrices: " << setup_.EnableMatrices << '\n' <<
        //float fUserMatrix[9]
        "EnableCrop: " << setup_.EnableCrop << '\n' <<
        //RECT CropRect;
        "EnableResample: " << setup_.EnableResample << '\n' <<
        "ResampleWidth: " << setup_.ResampleWidth << '\n' <<
        "ResampleHeight: " << setup_.ResampleHeight << '\n' <<
        "fGain16_8: " << setup_.fGain16_8 << '\n' <<
        //uint32_t FRPShape[16];// 0: flat, 1 ramp
        "TrigTC: " << // Trigger frame SMPTE time code and user bits
          int(setup_.TrigTC.hoursT) << int(setup_.TrigTC.hoursU) << ':' <<
          int(setup_.TrigTC.minutesT) << int(setup_.TrigTC.minutesU) << ':' <<
          int(setup_.TrigTC.secondsT) << int(setup_.TrigTC.secondsU) << ':' <<
          int(setup_.TrigTC.framesT) << int(setup_.TrigTC.framesU) << '\n' <<
        "fPbRate: " << setup_.fPbRate << '\n' <<
        "fTcRate: " << setup_.fTcRate << std::endl;
        //char CineName[256]; // Cine name
}

/*****************************************************************************/
cv::Mat CineReader::read_packed() {
    int num_unpacked = properties_->width * properties_->height;
    int num_packed = (bits_per_pixel_*num_unpacked)/8;
    uint8_t *packed = new uint8_t[num_packed];
    read(packed, num_packed*sizeof(uint8_t));
    cv::Mat unpacked = cv::Mat::zeros(properties_->height, properties_->width, CV_16U);

    if (bits_per_pixel_ == 10) {
        unpack_10bit(packed, unpacked, num_packed);

        // uncompress the 10-bit to 12-bit data
        // note that OpenCV does not have a 16-bit lookup table function so we
        // do it by hand
        cv::Mat A(properties_->height, properties_->width, CV_16U);
        cv::MatIterator_<uint16_t> it = A.begin<uint16_t>();
        cv::MatIterator_<uint16_t> end = unpacked.end<uint16_t>();
        cv::MatIterator_<uint16_t> dst;
        for (dst = unpacked.begin<uint16_t>(); dst != end; ++dst, ++it) {
            *it = CINE2_LUT[*dst];
        }
        // Do not flip any version, checked against Buffet data and TC3 data, jmp
        delete[] packed;
        return(A);
    }
    else {
        CV_Assert(bits_per_pixel_ == 12);
        unpack_12bit(packed, unpacked, num_packed);
        delete[] packed;
        return unpacked;
    }

    return cv::Mat();
}

/*****************************************************************************/
cv::Mat CineReader::read_linear() {
    // read 16-bit words directly into Mat
    unsigned nbytes = properties_->width * properties_->height * sizeof(uint16_t);
    cv::Mat A(properties_->height, properties_->width, CV_16U);
    uchar *p = A.data;

    read(p, nbytes);

    // flip frame vertically
    cv::Mat B(properties_->height, properties_->width, CV_16U);
    flip(A, B, 0);

    return(B);
}


/*****************************************************************************/
cv::Mat CineReader::read_frame(unsigned int n) {
    cv::Mat src;
    wanted_offset_ = image_offsets_[n-1] + 2 * sizeof(uint32_t);

    if (bits_per_pixel_ != 8) {
        src = read_packed();
    }
    else {
        src = read_linear();
    }

    return src;
}

/*****************************************************************************/
void CineReader::read(void* buf, size_t N) {
    if (N == 0)
       return;

    if (curr_offset_ != wanted_offset_) {
    try {
        ifs_.seekg(wanted_offset_);
    } catch(std::exception const& e) {
        std::cerr << "There was an error seeking to offset " << wanted_offset_;
        std::cerr << ": " << e.what() << std::endl;
    }
    curr_offset_ = wanted_offset_;
    }

    ifs_.exceptions( std::ifstream::eofbit | std::ifstream::failbit |
            std::ifstream::badbit );

    try {
    ifs_.read((char*)(buf), N);
    } catch(std::exception const& e) {
    std::cerr << "There was an error in CineReader::read: " << e.what() << std::endl;
    }
    curr_offset_ += N;
    wanted_offset_ = curr_offset_;
    wanted_offset_ += 2 * sizeof(uint32_t);
}

}  // namespace upsp
