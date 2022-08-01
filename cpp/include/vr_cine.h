/*
 * vr_cine.h
 *
 *  Created on: May 17, 2016
 *      Author: smurman
 *
 *  Header format as defined by Virtual Research
 */

#ifndef UFML_VR_CINE_H_
#define UFML_VR_CINE_H_

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

typedef int bool32_t;

/*****************************************************************************/
#if !defined(_TIMEDEFINED_)
#define _TIMEDEFINED_
//A format for small intervals of time: [250 picosecond ... 1 second)
//It is fixed point 0.32 or, in other words, the time in seconds is
//stored multiplied by 4Gig i.e. 4294967296.0 and rounded to int.
typedef uint32_t FRACTIONS, *PFRACTIONS;

//The absolute time format used in PC software is TIME64
typedef struct tagTIME64
{
    FRACTIONS fractions;     // Fractions of seconds
        // (resolution 1/4Gig i.e. cca. 1/4 ns)
        // The fractions of the second are stored here
        // multiplied by 2**32. Least significant 2 bits
        // store info about IRIG synchronization
        // bit0 = 0 IRIG synchronized
        // bit0 = 1 not synchronized
        // bit1 = 0 Event input=0 (short to ground)
        // bit1 = 1 Event input=1 (open)

    uint32_t seconds;     // Seconds from Jan 1 1970, compatible with the C
        // library routines
        // (max year: 2038 signed, 2106 unsigned)
        // VS2005 changed the default time_t to 64 bits;
        // here we have to maintain the 32 bits size to
        // remain compatible with the stored file format
        // and the public interfaces

} TIME64, *PTIME64;

/*****************************************************************************/
//Time code according to the standard SMPTE 12M-1999
typedef struct tagTC
{
    uint8_t framesU:4; // Units of frames
    uint8_t framesT:2; // Tens of frames
    uint8_t dropFrameFlag:1; // Dropframe flag
    uint8_t colorFrameFlag:1; // Colorframe flag
    uint8_t secondsU:4; // Units of seconds
    uint8_t secondsT:3; // Tens of seconds
    uint8_t flag1:1; // Flag 1
    uint8_t minutesU:4; // Units of minutes
    uint8_t minutesT:3; // Tens of minutes
    uint8_t flag2:1; // Flag 2
    uint8_t hoursU:4; // Units of hours
    uint8_t hoursT:2; // Tens of hours
    uint8_t flag3:1; // Flag 3
    uint8_t flag4:1; // Flag 4
    uint32_t userBitData; // 32 user bits
}TC, *PTC;

// Unpacked representation of SMPTE 12M-1999 Time Code
typedef struct tagTCU
{
    uint32_t frames;
    uint32_t seconds;
    uint32_t minutes;
    uint32_t hours;
    bool32_t dropFrameFlag;
    bool32_t colorFrameFlag;
    bool32_t flag1;
    bool32_t flag2;
    bool32_t flag3;
    bool32_t flag4;
    uint32_t userBitData;
}TCU, *PTCU;

#endif  /* _TIMEDEFINED_ */


#if !defined(_WBGAIN_)
#define _WBGAIN_
//Color channels adjustment
//intended for the White balance adjustment on color camera
//by changing the gains of the red and blue channels
typedef struct tagWBGAIN
{
    float R; //White balance, gain correction for red
    float B; //White balance, gain correction for blue
} WBGAIN, *PWBGAIN;
#endif

/*****************************************************************************/
#if !defined(_WINDOWS)
//Rectangle with well defined fields size
typedef struct tagRECT
{
    int32_t left;
    int32_t top;
    int32_t right;
    int32_t bottom;
} RECT, *PRECT;
#endif // _WINDOWS

#define OLDMAXFILENAME 65 // maximum file path size for the continuous recording
                          // to keep compatibility with old setup files

#define MAXLENDESCRIPTION_OLD 121 //maximum length for setup description
                                  //(before Phantom 638)

#define MAXLENDESCRIPTION 4096 // maximum length for new setup description

/*****************************************************************************/
// Image processing: Filtering
typedef struct tagIMFILTER
{
    int32_t dim; //square kernel dimension 3,5
    int32_t shifts; //right shifts of Coef (8 shifts means divide by 256)
    int32_t bias; //bias to add at end
    int32_t Coef[5*5]; //maximum alocation for a 5x5 filter
}
IMFILTER, *PIMFILTER;

typedef struct tagCINEFILEHEADER
{
    uint16_t Type;
    uint16_t Headersize;
    uint16_t Compression;
    uint16_t Version;
    int32_t FirstMovieImage;
    uint32_t TotalImageCount;
    int32_t FirstImageNo;
    uint32_t ImageCount;
    uint32_t OffImageHeader; // Offset of the BITMAPINFOHEADER structure in the cine file.
    uint32_t OffSetup; // Offset of the SETUP structure in the cine file.
    uint32_t OffImageOffsets; //Offset in the cine file of an array with the positions of each image stored in the file.
    TIME64 TriggerTime;
} CINEFILEHEADER;

typedef struct tagBITMAPINFOHEADER
{
    uint32_t biSize;
    int32_t biWidth;
    int32_t biHeight;
    uint16_t biPlanes;
    uint16_t biBitCount;
    uint32_t biCompression;
    uint32_t biSizeImage;
    int32_t biXPelsPerMeter;
    int32_t biYPelsPerMeter;
    uint32_t biClrUsed;
    uint32_t biClrImportant;
} BITMAPINFOHEADER;


/*****************************************************************************/
// SETUP structure - camera setup parameters
// It started to be used in 1992 during the 16 bit compilers era;
// the fields are arranged compact with alignment at 1 byte - this was
// the compiler default at that time. New fields were added, some of them
// replace old fields but a compatibility is maintained with the old versions.
//
// ---UPDF = Updated Field. This field is maintained for compatibility with old
// versions but a new field was added for that information. The new field can
// be larger or may have a different measurement unit. For example FrameRate16
// was a 16 bit field to specify frame rate up to 65535 fps (frames per second).
// When this was not enough anymore, a new field was added: FrameRate (32 bit    ￼￼
// integer, able to store values up to 4 billion fps). Another example: Shutter
// field (exposure duration) was specified initially in microseconds,
// later the field ShutterNs was added to store the value in nanoseconds.
// The UF can be considered outdated and deprecated; they are updated in the
// Phantom libraries but the users of the SDK can ignore them.
//
// ---TBI - to be ignored, not used anymore
//
// Use the definition from stdint.h with known size for the integer types
//
#pragma pack(1)
typedef struct tagSETUP
{
uint16_t FrameRate16; // ---UPDF replaced by FrameRate
uint16_t Shutter16; // ---UPDF replaced by ShutterNs
uint16_t PostTrigger16; // ---UPDF replaced by PostTrigger
uint16_t FrameDelay16; // ---UPDF replaced by FrameDelayNs
uint16_t AspectRatio; // ---UPDF replaced by ImWidth, ImHeight
uint16_t Res7; // ---TBI Contrast16
               // (analog controls, not available after
               // Phantom v3)
uint16_t Res8; // ---TBI Bright16
uint8_t Res9;  // ---TBI Rotate16
uint8_t Res10; // ---TBI TimeAnnotation
               // (time always comes from camera )
uint8_t Res11; // ---TBI TrigCine (all cines are triggered)
uint8_t TrigFrame; // Sync imaging mode:
                   // 0, 1, 2 = internal, external, locktoirig
uint8_t Res12; // ---TBI ShutterOn (the shutter is always on)
char DescriptionOld[MAXLENDESCRIPTION_OLD]; // ---UPDF replaced by larger Description able to
                                            // store 4k of user comments
uint16_t Mark; // "ST" - marker for setup file
uint16_t Length; // Length of the current version of setup
uint16_t Res13; // ---TBI Binning (binning factor)
uint16_t SigOption; // Global signals options:
                    // MAXSAMPLES = records the max possible samples
int16_t BinChannels; // Number of binary channels read from the
                     // SAM (Signal Acquisition Module)
uint8_t SamplesPerImage; // Number of samples acquired per image, both
                         // binary and analog;
char BinName[8][11]; // Names for the first 8 binary signals having
                     // maximum 10 chars/name; each string ended by a
                     // byte = 0
uint16_t AnaOption; // Global analog options single ended, bipolar
int16_t AnaChannels; // Number of analog channels used (16 bit 2's
                     // complement per channel)
uint8_t Res6;        // ---TBI (reserved)
uint8_t AnaBoard;    // Board type 0=none, 1=dsk (DSP system kit),
                     // 2 dsk+SAM
                     // 3 Data Translation DT9802
                     // 4 Data Translation DT3010
int16_t ChOption[8]; // Per channel analog options;
                     // now:bit 0...3 analog gain (1,2,4,8)
float AnaGain[8];    // User gain correction for conversion from voltage
                     // to real units , per channel
char AnaUnit[8][6];  // Measurement unit for analog channels: max 5
                     // chars/name ended each by a byte = 0
char AnaName[8][11]; // Channel name for the first 8 analog channels:
                     // max 10 chars/name ended each by a byte = 0
int32_t lFirstImage; // Range of images for continuous recording:
                     // first image
uint32_t dwImageCount; // Image count for continuous recording;
                       // used also for signal recording
int16_t nQFactor;   // Quality - for saving to compressed file at
                    // continuous recording; range 2...255


uint16_t wCineFileType; // Cine file type - for continuous recording
char szCinePath[4][OLDMAXFILENAME]; //4 paths to save cine files - for
                                    // continuous recording. After upgrading to Win32
                                    // this still remained 65 bytes long each
                                    // GetShortPathName is used for the filenames
                                    // saved here

uint16_t Res14; // ---TBI bMainsFreq (Mains frequency:
                // TRUE = 60Hz USA, FALSE = 50Hz
                // Europe, for signal view in DSP)

                // Time board - settings for PC104 irig board
                // used in Phantom v3 not used anymore after v3
uint8_t Res15;  // ---TBI bTimeCode;
                // Time code: IRIG_B, NASA36, IRIG-A

uint8_t Res16;  // ---TBI bPriority
                // Time code has priority over PPS

uint16_t Res17;  // ---TBI wLeapSecDY
                 // Next day of year with leap second

double Res18; // ---TBI dDelayTC Propagation delay for time code
double Res19; // ---TBI dDelayPPS Propagation delay for PPS

uint16_t Res20; // ---TBI  GenBits
int32_t Res1; // ---TBI
int32_t Res2; // ---TBI
int32_t Res3; // ---TBI

uint16_t ImWidth; // Image dimensions in v4 and newer cameras: Width
uint16_t ImHeight; // Image height
uint16_t EDRShutter16; // ---UPDF replaced by EDRShutterNs
uint32_t Serial; // Camera serial number. For firewire cameras you
                 // have a translated value here:
                 // factory serial + 0x58000

int32_t Saturation; // ---UPDF replaced by float fSaturation
                    // Color saturation adjustmment [-100, 100] neutral 0
uint8_t Res5;  // ---TBI

uint32_t AutoExposure; // Autoexposure enable 0=disable, 1=lock at trigger,
                       // 3=active after trigger

bool32_t bFlipH; // Flips image horizontally
bool32_t bFlipV; // Flips image vertically;
uint32_t Grid; // Displays a crosshair or a grid in setup, 0=no grid
               // 2=cross hair, 8= grid with 8 intervals
uint32_t FrameRate; // Frame rate in frames per seconds
uint32_t Shutter; // ---UPDF replaced by ShutterNs
                  // (here the value is in microseconds)

uint32_t EDRShutter; // ---UPDF replaced by EDRShutterNs
                     // (here the value is in microseconds)

uint32_t PostTrigger; // Post trigger frames, measured in frames
uint32_t FrameDelay; // ---UPDF replaced by FrameDelayNs
                     // (here the value is in microseconds)
bool32_t bEnableColor; // User option: when 0 forces gray images from
                       // color cameras

uint32_t CameraVersion; // The version of camera hardware (without decimal
                        // point). Examples of cameras produced after the
                        // year 2000
                        // Firewire: 4, 5, 6
                        // Ethernet: 42 43 51 7 72 73 9 91 10
                        // 650 (p65) 660 (hd) ....

uint32_t FirmwareVersion;// Firmware version
uint32_t SoftwareVersion;// Phantom software version
                         // End of SETUP in software version 551 (May 2001)

int32_t RecordingTimeZone;// The time zone active during the recording of
                          // the cine
                          // End of SETUP in software version 552 (May 2001)

uint32_t CFA; // Code for the Color Filter Array of the sensor
              // CFA_NONE=0,(gray) CFA_VRI=1(gbrg/rggb),
              // CFA_VRIV6=2(bggr/grbg), CFA_BAYER=3(gb/rg)
              // CFA_BAYERFLIP=4 (rg/gb)
              // high byte carries info about color/gray heads at
              // v6 and v6.2
              // Masks: 0x80000000: TLgray 0x40000000: TRgray
              // 0x20000000: BLgray 0x10000000: BRgray

//Final adjustments after image processing:
int32_t Bright; // ---UPDF replaced by fOffset
                // Brightness -100...100 neutral:0
int32_t Contrast; // ---UPDF replaced by fGain
                  // -100...100 neutral:0
int32_t Gamma; // ---UPDF replaced by fGamma
               // -100...100 neutral:0

uint32_t Res21; // ---TBI

uint32_t AutoExpLevel; // Level for autoexposure control
uint32_t AutoExpSpeed; // Speed for autoexposure control
RECT AutoExpRect; // Rectangle for autoexposure control
WBGAIN WBGain[4]; // Gain adjust on R,B components, for white balance,
                  // at Recording
                  // 1.0 = do nothing,
                  // index 0: all image for v4,5,7...
                  // and TL head for v6, v6.2 (multihead)
                  // index 1, 2, 3 : TR, BL, BR for multihead
int32_t Rotate;  // Rotate the image 0=do nothing
                 // +90=counterclockwise -90=clockwise

                 // End of SETUP in software version 578 (Nov 2002)

WBGAIN WBView;  // White balance to apply on color interpolated Cines
uint32_t RealBPP; // Real number of bits per pixel
                  // 8 on 8 bit cameras
                  // (v3, 4, 5, 6, 42, 43, 51, 62,
                  // Phantom v7:  8 or 12
                  // 14 bit cameras  8, 10, 12, 14
                  // Pixels will be stored on 8 or 16 bit in files
                  // and in PC memory
                  // (if RealBPP>8 the storage will be on 16 bits)

//First degree function to convert 16 bit pixels to 8 bit
//(for display or file convert)
uint32_t Conv8Min;// ---TBI
                  // Minimum value when converting to 8 bits
uint32_t Conv8Max; // ---UPDF replaced by fGain16_8
                   // Max value when converting to 8 bits

int32_t FilterCode; // ImageProcessing: area processing code
int32_t FilterParam; // ImageProcessing: optional parameter
IMFILTER UF; // User filter: a 3x3 or 5x5 user convolution filter
uint32_t BlackCalSVer; // Software Version used for Black Reference
uint32_t WhiteCalSVer; // Software Version used for White Calibration
uint32_t GrayCalSVer; // Software Version used for Gray Calibration
bool32_t bStampTime;// Stamp time (in continuous recording)
                    // 1 = absolute time, 3 = from trigger
                    // End of SETUP in software version 605 (Nov 2003)

uint32_t SoundDest; // Sound device 0: none, 1: Speaker, 2: sound board

//Frame rate profile
uint32_t FRPSteps; // Suplimentary steps in frame rate profile
                   // 0 means no frame rate profile

int32_t FRPImgNr[16]; // Image number where to change the rate and/or
                      // exposure allocated for 16 points (4 available
                      // in v7)

uint32_t FRPRate[16]; // New value for frame rate (fps)
uint32_t FRPExp[16]; // New value for exposure
                     // (nanoseconds, not implemented in cameras)

//Multicine partition
int32_t MCCnt; // Partition count (= cine count - 1)
               // Preview cine does not need a partition

float MCPercent[64]; // Percent of memory used for partitions
                     // Allocated for 64 partitions, 15 used in the
                     // current cameras
                     // End of SETUP in software version 606 (May 2004)

// CALIBration on Current Image (CSR, current session reference)
uint32_t CICalib; // This cine or this stg is the result of
                  // a current image calibration
                  // masks: 1 BlackRef, 2 WhiteCalib, 4 GrayCheck
                  // Last cicalib done at the acqui params:

uint32_t CalibWidth; // Image dimensions
uint32_t CalibHeight;
uint32_t CalibRate; // Frame rate (frames per second)
uint32_t CalibExp; // Exposure duration (nanoseconds)
uint32_t CalibEDR; // EDR (nanoseconds)
uint32_t CalibTemp; // Sensor Temperature
uint32_t HeadSerial[4]; // Head serials for ethernet multihead cameras
                        // (v6.2) When multiple heads are saved in a file,
                        // the serials for existing heads are not zero
                        // When one head is saved in a file its serial is
                        // in HeadSerial[0] and the other head serials
                        // are 0xFFffFFff
                        // End of SETUP in software version 607 (Oct 2004)
uint32_t RangeCode; // Range data code: describes the range data format
uint32_t RangeSize; // Range data, per image size
uint32_t Decimation; // Factor to reduce the frame rate when sending
                     // the images to i3 external memory by fiber
                     // End of SETUP in software version 614 (Feb 2005)

uint32_t MasterSerial; // Master camera Serial for external sync. 0 means
                       // none (this camera is not a slave of another camera)
                       // End of SETUP in software version 624 (Jun 2005)

uint32_t Sensor; // Camera sensor code
                // End of SETUP in software version 625 (Jul 2005)

//Acquisition parameters in nanoseconds
uint32_t ShutterNs; // Exposure, in nanoseconds
uint32_t EDRShutterNs; // EDRExp, in nanoseconds
uint32_t FrameDelayNs; // FrameDelay, in nanoseconds
                       // End of SETUP in software version 631 (Oct 2005)

//Stamp outside the acquired image
//(this increases the image size by adding a border with text information)
uint32_t ImPosXAcq; // Acquired image horizontal offset in
                    // sideStamped image

uint32_t ImPosYAcq;// Acquired image vertical offset in sideStamped
                   // image

uint32_t ImWidthAcq; // Acquired image width (different value from
                     // ImWidth if sideStamped file)

uint32_t ImHeightAcq;// Acquired image height (different value from
                     // ImHeight if sideStamped file)

char Description[MAXLENDESCRIPTION];//User description or comments
                                   //(enlarged to 4096 characters)
                                   // End of SETUP in software version 637 (Jul 2006)
bool32_t RisingEdge; // TRUE rising, FALSE falling
uint32_t FilterTime; // time constant
bool32_t LongReady; // If TRUE the Ready is 1 from the start
                    // to the end of recording (needed for signal
                    // acquisition)

bool32_t ShutterOff;// Shutter off - to force maximum exposure for PIV
                    // End of SETUP in software version 658 (Mar 2008)

uint8_t Res4[16]; // ---TBI
                  // End of SETUP in software version 663 (May 2008)

bool32_t bMetaWB; // pixels does not have WB applied
                  // (or any other processing)

int32_t Hue; // ---UPDF replaced by float fHue
             // hue corection (degrees: -180 ...180)
             // End of SETUP in software version 671 (May 2009)

int32_t BlackLevel; // Black level in the raw pixels
int32_t WhiteLevel; // White level in the raw pixels

char LensDescription[256];// text with the producer, model,
                          // focal range etc ...
float LensAperture; // aperture f number
float LensFocusDistance;// distance where the objects are in focus in
                        // meters, not available from Canon motorized lens
float LensFocalLength; // current focal length; (zoom factor)
                       // End of SETUP in software version 691 (Jul 2010)

//image adjustment
float fOffset;// [-1.0, 1.0], neutral 0.0;
              // 1.0 means shift by the maximum pixel value
float fGain; // [0.0, Max], neutral 1.0;
float fSaturation; // [0.0, Max], neutral 1.0;
float fHue; // [-180.0, 180.0] neutral 0;
            // degrees and fractions of degree to rotate the hue

float fGamma;// [0.0, Max], neutral 1.0; global gamma
             // (or green gamma)
float fGammaR; // per component gammma (to be added to
               // Gamma)
               // 0 means neutral

float fGammaB;
float fFlare; // [-1.0, 1.0], neutral 0.0;
              // 1.0 means shift by the maximum pixel
              // pre White Balance offset

float fPedestalR; // [-1.0, 1.0], neutral 0.0;
                  // 1.0 means shift by the maximum pixel
float fPedestalG; // after gamma offset
float fPedestalB;

float fChroma; // [0.0, Max], neutral 1.0;
               // chrominance adjustment (after gamma)

char  ToneLabel[256];
int32_t   TonePoints;
float fTone[32*2]; // up to 32  points  + 0.0,0.0  1.0,1.0
                   // defining a LUT using spline curves

char UserMatrixLabel[256];
bool32_t EnableMatrices;
float fUserMatrix[9]; // RGB color matrix

bool32_t EnableCrop; // The Output image will contains only a rectangle
                     // portion of the input image
RECT CropRect;
bool32_t EnableResample;// Resample image to a desired output Resolution
uint32_t ResampleWidth;
uint32_t ResampleHeight;

float fGain16_8; // Gain coefficient used when converting to 8bps
                 // Input pixels (bitdepth>8) are multiplied by
                 // the factor: fGain16_8 * (2**8 / 2**bitdepth)
                 // End of SETUP in software version 693 (Oct 2010)

uint32_t FRPShape[16];// 0: flat, 1 ramp
TC TrigTC; // Trigger frame SMPTE time code and user bits
float fPbRate; // Video playback rate (fps) active when the cine
               // was captured
float fTcRate; // Playback rate (fps) used for generating SMPTE
               // time code
               // End of SETUP in software version 701 (Apr 2011)

char CineName[256]; // Cine name
                    // End of SETUP in software version 702 (May 2011)


//VRI internal note: Size checked structure.
//Update oldcomp.c if new fields are added
//------------------------------------------------------------------------
} SETUP, *PSETUP;
/*****************************************************************************/

typedef struct tagINFORMATIONBLOCK
{
    uint32_t BlockSize;
    uint16_t Type;
    uint16_t Reserved;
    uint8_t *Data; // allocate to [BlockSize-8];
} INFORMATIONBLOCK;

typedef struct tagANNOTATIONBLOCK
{
    uint32_t AnnotationSize;
    uint8_t *Annotation;  // allocate to size [AnnotationSize - 8];
    uint32_t ImageSize;
} ANNOTATIONBLOCK;

#pragma pack()
#ifdef __cplusplus
}
#endif

#include <boost/fusion/adapted/struct/adapt_struct.hpp>
#include <boost/fusion/include/adapt_struct.hpp>

// Generate an adapter allowing to view as a Boost.Fusion sequence
BOOST_FUSION_ADAPT_STRUCT(
    tagCINEFILEHEADER,
    (    uint16_t, Type)
    (    uint16_t, Headersize)
    (    uint16_t, Compression)
    (    uint16_t, Version)
    (    int32_t, FirstMovieImage)
    (    uint32_t, TotalImageCount)
    (    int32_t, FirstImageNo)
    (    uint32_t, ImageCount)
    (    uint32_t, OffImageHeader)
    (    uint32_t, OffSetup)
    (    uint32_t, OffImageOffsets)
//    (TIME64, TriggerTime)
)

#endif /* UFML_VR_CINE_H_ */
