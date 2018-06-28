#ifndef __LUCAM_DEF__H__
#define __LUCAM_DEF__H__

#define REQUEST_LUCAM               0x12

#define LUCAM_INITIALIZE            0x0000
#define LUCAM_USB_HIGH_SPEED        0x0004
#define LUCAM_FPGA_MODE             0x0008
#define LUCAM_FIRMFPGA_VERSION      0x000c
#define LUCAM_SPECIFICATION         0x0010
// steven [begin]
#define LUCAM_FLASH_MODE            0x0014
#define LUCAM_FLASH_OFFSET          0x0018
#define LUCAM_FLASH_UPDATE_SIZE     0x001c
#define LUCAM_FLASH_TYPE            0x0020
// steven [end]
#define LUCAM_FORMAT_COUNT          0x019c
#define LUCAM_TRUE_PIXEL_DEPTH      0x01a0
#define LUCAM_VIDEO_EN              0x0214
#define LUCAM_TRIGGER_CTRL          0x0218

#define LUCAM_FLAGS                 0x0280

#define LUCAM_BRIGHTNESS            0x0400
#define LUCAM_CONTRAST              0x0410
#define LUCAM_HUE                   0x0420
#define LUCAM_SATURATION            0x0430
#define LUCAM_SHARPNESS             0x0440
#define LUCAM_GAMMA                 0x0450
#define LUCAM_PAN                   0x0460
#define LUCAM_TILT                  0x0470
#define LUCAM_ROLL                  0x0480
#define LUCAM_ZOOM                  0x0490
#define LUCAM_FO_EXPOSURE           0x04a0
#define LUCAM_IRIS                  0x04b0
#define LUCAM_FOCUS                 0x04c0
#define LUCAM_WHITEBAL_U            0x04d0
#define LUCAM_WHITEBAL_V            0x04e0
#define LUCAM_FO_GAIN               0x04f0
#define LUCAM_FO_GAIN_RED           0x0500
#define LUCAM_FO_GAIN_GREEN1        0x0510
#define LUCAM_FO_GAIN_GREEN2        0x0520
#define LUCAM_FO_GAIN_BLUE          0x0530
#define LUCAM_STILL_EXPOSURE        0x0540
#define LUCAM_STILL_GAIN            0x0550
#define LUCAM_STILL_GAIN_RED        0x0560
#define LUCAM_STILL_GAIN_GREEN1     0x0570
#define LUCAM_STILL_GAIN_GREEN2     0x0580
#define LUCAM_STILL_GAIN_BLUE       0x0590
#define LUCAM_STILL_STROBE_DELAY    0x05a0
#define LUCAM_STILL_DYN_RANGE       0x05b0
#define LUCAM_STILL_KNEE1_EXPOSURE  0x05c0
#define LUCAM_STILL_KNEE2_EXPOSURE  0x05d0
#define LUCAM_STILL_KNEE3_EXPOSURE  0x05e0
#define LUCAM_VIDEO_KNEE            0x05f0
#define LUCAM_STILL_SHUTTER_TYPE    0x0600
#define LUCAM_STILL_EXPOSURE_DELAY  0x0610
#define LUCAM_THRESHOLD             0x0620
#define LUCAM_SPECIAL_MODE          0x0630
#define LUCAM_AUTO_EXP_TARGET       0x0640
#define LUCAM_UART_CTRL             0x0650
#define LUCAM_TIMESTAMPS            0x0660
#define LUCAM_SNAPSHOT_SETTING      0x0670
#define LUCAM_AUTO_EXP_MAX          0x0680
#define LUCAM_TEMPERATURE           0x0690
#define LUCAM_BPC                   0x06a0
#define LUCAM_TRIGGER               0x06b0
#define LUCAM_TRIGGER_PIN           0x06b0
#define LUCAM_DIGITAL_GAIN          0x06c0
#define LUCAM_FRAME_GATE            0x06d0
#define LUCAM_EXPOSURE_INTERVAL     0x06e0
#define LUCAM_PWM                   0x06f0
#define LUCAM_MEMORY                0x0700
#define LUCAM_STILL_STROBE_DURATION 0x0710
#define LUCAM_COLOR_LUT_ACCESS      0x0720
#define LUCAM_FAN                   0x0730
#define LUCAM_SYNC_MODE             0x0740
#define LUCAM_SNAPSHOT_COUNT        0x0750
#define LUCAM_LSC_X                 0x0760
#define LUCAM_LSC_Y                 0x0770
#define LUCAM_AUTO_IRIS_MAX         0x0780
#define LUCAM_LENS_STABILIZATION    0x0790
#define LUCAM_VIDEO_TRIGGER         0x07a0
#define LUCAM_VIDEO_SETTING         0x07b0
#define LUCAM_BLANKING_H            0x07c0
#define LUCAM_BLANKING_V            0x07d0
#define LUCAM_KNEE2_LEVEL           0x07e0
#define LUCAM_KNEE3_LEVEL           0x07f0
#define LUCAM_THRESHOLD_LOW         0x0800
#define LUCAM_THRESHOLD_HIGH        0x0810
#define LUCAM_TEMPERATURE2          0x0820
#define LUCAM_LIGHT_FREQUENCY       0x0830
#define LUCAM_LUMINANCE             0x0840
#define LUCAM_AUTO_GAIN_MAX         0x0850
#define LUCAM_TOBERENAMED1          0x0860
#define LUCAM_STROBE_PIN            0x0870
#define LUCAM_TRIGGER_MODE          0x0880
#define LUCAM_FOCAL_LENGTH          0x0890
#define LUCAM_IRIS_LATENCY          0x08a0
#define LUCAM_BLACK_LEVEL           0x08b0
#define LUCAM_VIDEO_MIN_FRAME_TIME  0x08c0
#define LUCAM_STILL_MIN_FRAME_TIME  0x08d0
#define LUCAM_BLACK_LEVEL_RAW       0x08e0
#define LUCAM_VIDEO_GAIN_RAW        0x08f0
#define LUCAM_STILL_GAIN_RAW        0x0900
#define LUCAM_GAINHDR               0x0910
#define LUCAM_STILL_GAINHDR         0x0920
#define LUCAM_VIDEO_GAINHDR_RAW     0x0930
#define LUCAM_STILL_GAINHDR_RAW     0x0940
#define LUCAM_STILL_LIGHTING_PHASE_SYNC      0x0950
#define LUCAM_AUTO_GAIN_MIN         0x0960
#define LUCAM_TIMESTAMP_HW_RESET    0x0970
#define LUCAM_IRIS_STEPS            0x0980
#define LUCAM_TOBERENAMED5          0x0990
#define LUCAM_TOBERENAMED6          0x09a0
#define LUCAM_TOBERENAMED7          0x09b0
#define LUCAM_TOBERENAMED8          0x09c0
#define LUCAM_TOBERENAMED9          0x09d0
#define LUCAM_TOBERENAMED10         0x09e0
#define LUCAM_TOBERENAMED11         0x09f0
#define LUCAM_TOBERENAMED12         0x0a00
#define LUCAM_TOBERENAMED13         0x0a10
#define LUCAM_TOBERENAMED14         0x0a20
#define LUCAM_TOBERENAMED15         0x0a30

#define LUCAM_FIRST_PROP            LUCAM_BRIGHTNESS // Keep this up!!
#define LUCAM_LAST_PROP             LUCAM_TOBERENAMED15 // Keep this up!!

#define LUCAM_PROP_SIZE             0x0010


#define LUCAM_REALTIMESTAMP         0x0d00
#define LUCAM_REALTIMESTAMP_FREQUENCY 0x0d20


#define LUCAM_FO_COLOR_ID           0x1010
#define LUCAM_COLOR_INQ             0x1014
#define LUCAM_MAX_SIZE              0x1000
#define LUCAM_UNIT_SIZE             0x1004
#define LUCAM_FO_POSITION           0x1008
#define LUCAM_FO_SIZE               0x100c
#define LUCAM_FO_SUBSAMPLING        0x1018
#define LUCAM_FO_SUBSAMPLING_INQ    0x101c
#define LUCAM_FO_KPS_MINMAX_INQ     0x1040
#define LUCAM_FO_KPS                0x1048
#define LUCAM_FO_KPS_ARRAY_CNT      0x104c
#define LUCAM_FO_KPS_ARRAY          0x1050
#define LUCAM_FO_VALIDATE              0x1060
#define LUCAM_FO_TAP_CONFIGURATION     0x1068
#define LUCAM_FO_TAP_CONFIGURATION_INQ 0x106c

#define LUCAM_SEQUENCING_FRAME_COUNT_MAX     0x2000
#define LUCAM_SEQUENCING_FRAME_COUNT         0x2004
#define LUCAM_SEQUENCING_FRAME_CURRENT       0x2008
#define LUCAM_SEQUENCING_FRAME_INDEX         0x200c
#define LUCAM_SEQUENCING_STATE               0x2010
#define LUCAM_SEQUENCING_RESERVED            0x2014
#define LUCAM_SEQUENCING_EDIT_FLAGS          0x2038
#define LUCAM_SEQUENCING_EDIT_FRAME          0x203c

#define LUCAM_SEQUENCING_EDIT_BASE           0x2040

#define LUCAM_SEQUENCING_STATE_ON_SNAP       0x00000002
#define LUCAM_SEQUENCING_STATE_CIRCULAR      0x80000000

#define LUCAM_STILL_POSITION           0x4008
#define LUCAM_STILL_SIZE               0x400c
#define LUCAM_STILL_COLOR_ID           0x4010
#define LUCAM_STILL_SUBSAMPLING        0x4018
#define LUCAM_STILL_VALIDATE           0x4060
#define LUCAM_STILL_TAP_CONFIGURATION  0x4068

#define LUCAM_PROP_MESSAGE_SUPPORT0 0x4FF8
#define LUCAM_PROP_MESSAGE_NUMBER   0x4FFC
#define LUCAM_PROP_MESSAGE_BASE     0x5000

/***************************************************
 Available color IDS for these regs:
  LUCAM_FO_COLOR_ID
  LUCAM_COLOR_INQ (8 lsb)
  LUCAM_STILL_COLOR_ID
****************************************************/

#define LUCAM_COLOR_MONO8           0
#define LUCAM_COLOR_MONO16          5
#define LUCAM_COLOR_MONO61          6

#define LUCAM_COLOR_BAYER_RGGB      8
#define LUCAM_COLOR_BAYER_GRBG      9
#define LUCAM_COLOR_BAYER_GBRG      10
#define LUCAM_COLOR_BAYER_BGGR      11

#define LUCAM_COLOR_BAYER16_RGGB    12
#define LUCAM_COLOR_BAYER16_GRBG    13
#define LUCAM_COLOR_BAYER16_GBRG    14
#define LUCAM_COLOR_BAYER16_BGGR    15

#define LUCAM_COLOR_XENA_CYYM       16
#define LUCAM_COLOR_XENA_YCMY       17
#define LUCAM_COLOR_XENA_YMCY       18
#define LUCAM_COLOR_XENA_MYYC       19

#define LUCAM_COLOR_XENA16_CYYM     20
#define LUCAM_COLOR_XENA16_YCMY     21
#define LUCAM_COLOR_XENA16_YMCY     22
#define LUCAM_COLOR_XENA16_MYYC     23

#define LUCAM_COLOR_BAYER61_RGGB    24
#define LUCAM_COLOR_BAYER61_GRBG    25
#define LUCAM_COLOR_BAYER61_GBRG    26
#define LUCAM_COLOR_BAYER61_BGGR    27

#define LUCAM_COLOR_XENA61_CYYM     28
#define LUCAM_COLOR_XENA61_YCMY     29
#define LUCAM_COLOR_XENA61_YMCY     30
#define LUCAM_COLOR_XENA61_MYYC     31


/*******************************************************
 Related to the LUCAM_PROP_MESSAGE_* registers.
*******************************************************/

#define LUCAM_PROP_MSG0_LUT8        0
#define LUCAM_PROP_MSG0_LUT16       2


/******************************************************
 Not related to a reg in particular.
******************************************************/

#define LUCAM_EXT_CMD_GPIO          0x7
#define LUCAM_EXT_CMD_GPIO_SELECT   0x9
#define LUCAM_EXT_CMD_LED           0xf
#define LUCAM_EXT_CMD_GPIO_CONFIGURE 0x10


/******************************************************
 Flags of LUCAM_FLAGS register
*******************************************************/
#define LUCAM_FLAGS_FORMAT_VALIDATION           0x00000002
#define LUCAM_FLAGS_FLASH_SUPPORTED             0x00000004
#define LUCAM_FLAGS_FORMAT_COMMIT_SUPPORTED     0x00000010 /* _FO_FORMAT_COMMIT is valid */
#define LUCAM_FLAGS_KFREQ32                     0x00000020 /* KFREQS are 32 bits (PARA_INQ unused) and represents pixels, not bytes */
#define LUCAM_FLAGS_FLASH_SUBSECTORS_SUPPORTED  0x00000080
#define LUCAM_FLAGS_CONTINUOUS_KFREQS           0x00000100
#define LUCAM_FLAGS_MODELDTA_PRESENT            0x00000800


// steven [begin]
//----------------------------------------------------------
// LUCAM_FLASH_TYPE values
#define FLASH_INTEL_28F640J3A                   (0) // 64*128K=8MB
#define FLASH_SPANSION_S25FL016A                (1) // 32*64K=2MB
#define FLASH_MICRON_M25P40                     (2) // 8*64K=512KB

//----------------------------------------------------------
// LUCAM_FLASH_MODE values
#define LUCAM_FLASH_MODE_NORMAL                 0x00000000 // W
#define LUCAM_FLASH_MODE_PROGRAM                0x00000001 // W
#define LUCAM_FLASH_MODE_SECTOR_ERASE           0x00000002 // W
#define LUCAM_FLASH_MODE_BULK_ERASE             0x00000004 // W
#define LUCAM_FLASH_MODE_READ                   0x00000008 // W
#define LUCAM_FLASH_MODE_SUBSECTOR_ERASE        0x00000020 // W
#define LUCAM_FLASH_MODE_DONE                   0x00000010 // R
// steven [end]

/****************************************************************************
* MODEL DATA STUFF
****************************************************************************/

#define LUCAM_MODEL_DATA_MATRICES                        1
struct _model_data_matrix
{
   short matrixIndex;
   short matrixCoefficients[9];
};

#define LUCAM_MODEL_DATA_FLAGS                           2
#define LUCAM_MODEL_DATA_FLAG_MONO_GRID      0x00000001
#define LUCAM_MODEL_DATA_FLAG_GREENS_UNEQUAL 0x00000002
#define LUCAM_MODEL_DATA_FLAG_MULTI_TAPS     0x00000004

#define LUCAM_PROP_SEQUENCABLE_FLAG          0x08000000

#define LUCAM_MODEL_DATA_PERMANENT_STORAGE_MAPPING       4

// Default LUTF settings. {ULONG Version ; ULONG Reserved ; struct _lutf_setting_??? Mask ; struct _lutf_setting_??? Value; }
// Version 0: ??? is short 
// Version 1: ??? is no_lut
#define MODEL_DTA_LUTF_DEFAULT_SETTINGS                  6


#endif // __LUCAM_DEF__H__

