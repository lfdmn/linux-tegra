/*  Copyright Lumenera Corporation 2005
 *  All Rights Reserved
 *
 */

#ifndef LUCAM_H
#define LUCAM_H

/*
* If ALLOW_RGB24_OUTPUT is 1 then the driver can convert frames to RGB24 but more memory is required.
* If ALLOW_RGB24_OUTPUT is 0 then the driver cannot convert frames to RGB24. This saves some memory.
*/
#ifndef ALLOW_RGB24_OUTPUT
#define ALLOW_RGB24_OUTPUT             0
#endif

/*
* The driver allocates a number of buffer to work with.
*
*/
#ifndef MAX_IMAGE_BUFS
#define MAX_IMAGE_BUFS	               2
#endif

/*
* Most cameras supports enhanced buffering.
*/
#ifndef ENHANCED_BUFFERING
#define ENHANCED_BUFFERING             0
#endif

/*
 * USB3 cameras uses a different endpoint layout.
 */
#ifndef USB3_ENDPOINT_CONFIGURATION
#define USB3_ENDPOINT_CONFIGURATION    0
#endif


#define MEM_IN_BULK_BUF                0

#define LUCAM_VIDEO_READ_QUEUE_ALL     1

#define USE_VMALLOC32                  1


#ifdef __KERNEL__
#ifndef VIDEODEV_VERSION_CODE
   #define VIDEODEV_VERSION_CODE LINUX_VERSION_CODE
#endif

#include <linux/version.h>
#include <linux/usb.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
#include <linux/videodev2.h>
#else
#include <linux/videodev.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
#include <media/v4l2-dev.h>
#endif
#include <linux/kernel.h>                         /* printk() */
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>

#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#if  LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
#include <media/v4l2-common.h>
#endif
#if  LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <media/v4l2-ioctl.h>
#endif

#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(3,11,0)
#include <media/v4l2-device.h>
#endif

#if  LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
#if USE_VMALLOC32
#include "videobuf-vmalloc_32.h"
#else
#include <media/videobuf-vmalloc.h>
#endif
#else
#include <media/video-buf.h>
#endif

#ifdef CONFIG_VIDEO_V4L1_COMPAT
/* Include V4L1 specific functions. Should be removed soon */
#include <linux/videodev.h>
#endif

#endif // __KERNEL__

#include "lucam_def.h"


/**************************************************************************
 * LUCAM_MODULE_VERSION
 * 2,0,0,7: LIN-86: Fix issue with saving tiff on 64-bits platforms, wrt to XRESOLUTION and YRESOLUTION tags.
 * 2,0,0,8: LIN-132: Fix endianness issue when converting to RGB using the non-ex functions
 * 2,0,0,9: Added LucamSetTimeout.
 *          Added several backdoor functionnalities for use with testing.
 * 2,0,0,10: LIN-137: Fixed api correcting the Hardware bad pixel map instead of the software.
 *           Fixed StopStreaming not cancelling any pending TakeFrame.
 * 2,0,0,11: LIN-156: Fixed bug where LUCAM_PROP_UNIT_WIDTH and LUCAM_PROP_UNIT_HEIGHT were hard-coded
 * 2,0,0,12: LIN-159 Added support for LUCAM_PROP_AUTO_EXP_TARGET
 * 2,0,0,13: LIN-190 Added camera timestamp functions and properties to linux driver and SDK
 * 2,0,0,14: LIN-198:Fixed bug where video buffer queue sometimes didnn't get emptied on camera close
 * 2.0.0.15: LIN-201:API pixel correction fix
 *           LIN-205:Added driver support for Infinity 3S-1 camera
 *           LIN-204:Fixed the bug where image timestamps were wiped on host tap correction.
 * 2.0.0.16: LIN-209:Added support for LUCAM_PROP_AUTO_EXP_MAXIMUM, LUCAM_PROP_AUTO_GAIN_MINIMUM
 *                   and LUCAM_PROP_AUTO_GAIN_MAXIMUM
 * 2.0.0.17: IPL-370: Fix for init-auto-focus not reforcing init of focus.
 * 2.0.0.18: LIN-220: Added API and driver support for GPIO timestamp clock reset feature
 *           LIN-223: Added support for LUCAM_PROP_FOCAL_LENGTH
 * 2.0.0.19: LIN-245: Fix for a bug where API was incorrectly setting pixel format value on init
 * 2.0.0.20: LIN-249: Added support for Pregius cameras
 *           LIN-250: Added support for LUCAM_PROP_IRIS_STEPS_COUNT
 *           LIN-246:Fixed the bug that caused the API segfault when doing ROI stream
 *                   autofocus after enable/disable fast frames.
 *           LIN-245: Fixed the bug that caused the API segfault when doing autofocus
 *                    while streaming 16bit video
 * 2.0.0.21: LIN-235: OneShotAuto fix for Lt29
 *           LIN-240: Fixed bug where SW bad pixel/column correction corrupted timestamps
 *           LIN-242: Added LucamInitAutoLens to autofocus example
 *           LIN-244: Fixed bug where autofocus didn't work properly on linux
 *           LIN-252: Updated api/driver version for the linux package for lin-245 fix.
 *           LIN-254: Added support for 64-bit ARM
 *           LIN-224,LIN-241: Added support for binning in SW BPC on linux
 *           LIN-192,LIN-259: Fixed bug where still exposure wrapped arround at 71 minutes
 *           LIN-194: Fixed bug where reset of gamma, contrast, brightness and digital gain
 *                    didn't work for cameras that didn't have this feature implemented in
 *                    camera such as Veyron
 *           LIN-228: Fixed the bug where Cannon 50mm auto focus didn't work on linux
 *           LIN-200,LIN-260: Support for system architecture check on SDK package install
 *           LIN-263: Fixed OneShotAutoExposure bug when frame is too dark
 *           LIN-262: Fixed bug that caused pregius camera hw trigger failure
 *           LIN-264: Fixed bug where veyron 16bit oneshotautoexposure didn't work properly
 *           LIN-258: Fixed bug where LucamSaveImageEx() created corrupt BMP files on ARM 64-bit
 * 2.0.0.22: LIN-263: Update API and driver version to 2.0.0.22, and dso version to 2.3
 *                    for Linux_SDK_2.3 release
 *           LIN-251: Disabled LUCAM_DM_HIGHER_QUALITY demosaicing method when setting 
 *                    LUCAM_PROP_DEMOSAICING_METHOD
 *           LIN-197: Added support for lens shading correction properties LSC_X and LSC_Y
 *           LIN-261: Fixed gamma rounding error
 *           LIN-257: Fixed the bug where images were bogus depending on filename
 *           PREGU-74: Fix for init auto lens not working on Pregius models
 *           LIN-210: Fix for LucamConvertFrameToRgb24 sometimes being different then
 *                    LucamConvertFrameToRgb24Ex
 *           LIN-265: Added API support for downloading of CCM from the camera
 * 2.0.0.23: LIN-208: Fixed the bug where driver couldn't read camera serial number
 *                    for linux kernel version 4.10.0 and higher.
 * 2.0.0.24: LIN-293: Reverted the fix for LIN-210 in SVN#9611.
 */
#define MAKE_VERSION(a,b,c,d) ((((a)&0xff)<<24) | (((b)&0xff)<<16) | (((c)&0xff) << 8) | ((d)&0xff))

#define LUCAM_MODULE_VERSION MAKE_VERSION(2,0,0,23)

//#define DEBUG




/*
* Minor number assignation default
*/
#ifndef USBPORT_NR_DEFAULT
#define USBPORT_NR_DEFAULT       0
#endif


#ifndef O_NOIO
#define O_NOIO  O_TRUNC
#endif





/*************************************************
TO DO LIST:
- Implement FPGA pipe reset. (not necessary)
- Implement reset.
- Implement version.
*************************************************/



#ifndef _BYTE_DEFINED
#define _BYTE_DEFINED
typedef __u8 BYTE;
#endif                                            // !_BYTE_DEFINED

#ifndef _BOOL_DEFINED
#define _BOOL_DEFINED
typedef int BOOL;

#define FALSE           0
#define TRUE            1

#endif                                            // !_BOOL_DEFINED

#ifndef _WORD_DEFINED
#define _WORD_DEFINED
typedef __u16 WORD;
#endif                                            // !_WORD_DEFINED

typedef struct
{
    WORD w, h;
}lucam_point;

typedef struct _lucam_prop_r
{
   __u32 flags;
   __s32 value;
   __s32 min;
   __s32 max;
} lucam_prop, lucam_prop_r;

typedef struct _lucam_prop_w
{
   __u32 flags;
   __s32 value;
} lucam_prop_w;

typedef struct _lucam_format_subsampling
{
   __u16 subsamplingX;
   __u16 subsamplingY;
} lucam_format_subsampling;

typedef struct _lucam_format_startpos
{
   __u16 startX;
   __u16 startY;
} lucam_format_startpos;

typedef struct _lucam_color_id_2
{
   __u32 colorId;
   __u32 tapConfig;
} lucam_color_id_2;

typedef enum
{
   LUCAM_VIDEO_MODE                =0,
   LUCAM_STILL_HW_TRIGGER_MODE     =1,
   LUCAM_STILL_SW_TRIGGER_MODE     =2
}LUCAM_MODE;

typedef struct _lucam_version
{
   __u32 camera_id;
   __u32 serial_number;
   __u32 hardware_version;
   __u32 driver_version;
   __u32 firmware_version;
   __u32 usb_speed;
} lucam_version;

typedef struct _lucam_sgl_thru
{
   __u16 index;
   __u16 value;
   __u16 length;
   __u16 req_or_reserved;
}lucam_sgl_thru;

typedef struct _lucam_event_bits
{
   __u32 events[2];
}lucam_event_bits;

typedef struct _lucam_reg_access
{
   __u32 reg;
   __s32 value;
} lucam_reg_access;

// steven [begin]
// the macros are used as command in struct lucam_flash_access
enum FLASH_COMMAND
{
   FLASH_COMMAND_IDENTIFY            = 0,
   FLASH_COMMAND_READ_WRITE          = 1,  // for read and write
   FLASH_COMMAND_ERASE_BULK,         
   FLASH_COMMAND_ERASE_SECTOR,       
   FLASH_COMMAND_ERASE_SUBSECTOR     
};

typedef struct _lucam_flash_access
{
   __u32 command;  // 1 = r/w; 2,3,4=erase
   __u32 offset;
   __u32 len;
} lucam_flash_access;

// steven [end]

struct lucam_counters
{
   int frame_completions;
   int timestamps;
   int frame_errors;
   int frame_lost;
   int bh_count;
   int bh_success;
   int bh_inprogress;
   int bh_buffererror;
   int bh_stalled;
   int bh_babble;
   int bh_bitstuff;
   int bh_crctimeout;
   int bh_nak;
   int bh_unknownerror;
   int bh_desyncherror;
   int bh_cancelled;
   int bh_remoteioerror; /* not considered an error */
   int bh_shortpacketerror;
   int restarts_scheduled;
   int bulkbufs_submitted;
   int bulkbufs_submit_failures;
   
   int pending_frames;
   int pending_usbtransfers;
};

#ifdef __KERNEL__

/*
 * in systems where not all memory is DMA addressable, use a bounce
 * buffer for each image packet URB's incoming data.
 * The videobuf-vmalloc32 alternative has not been implemented yet.
 */
#ifndef MEM_IN_BULK_BUF
#define MEM_IN_BULK_BUF       0
#endif


#define CLAMP(x,min,max) ((x>max)?max:((x<min)?min:x))

#define LUCAM_NAME              "Lucam"
#define LUCAM_DRIVER_VERSION    KERNEL_VERSION(4,8,0)
#define VID_HARDWARE_LUCAM      0x5354                //vendor ID
#define VID_HARDWARE_LUCAM2     0x1724                //vendor ID

//request
#define LUMENERA_REQUEST    0x12                  //for access camera register
#define LUMENERA_EXT_CMD    0x13

#define FPGA_OUT_ENDPOINT
#define VIDEO_IN_ENDPOINT
#define STILL_IN_ENDPOINT

#define IDLE_ALT_SETTING     0
#define FPGA_ALT_SETTING     (plucam->fpga_setting)
#define DATA_ALT_SETTING     (plucam->data_setting)

#define MAX_CAMERA_NUMBER   127

#define FRAME_ERROR_REQUIRE_RESTART     2

#if  LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,12)
   // timeout must be in msecs
   #define USB_CTRL_MSG_TIMEOUT             (5000)
   #define USB_BULK_MSG_TIMEOUT             (5000)
#else
   // Timeout must be in jiffies
   #define USB_CTRL_MSG_TIMEOUT             (HZ*5)
   #define USB_BULK_MSG_TIMEOUT             (HZ*5)
#endif

#define CAMERA_MONO  1
#define CAMERA_COLOR 2

#define SWAP16(u)          ((((u) & 0xff) << 8) | (((u) & 0xff00) >> 8))


static inline BOOL is_prop_supported (lucam_prop* v)
{
    return (v->flags >>31 );
};

static inline BOOL is_prop_auto_supported(lucam_prop* v)
{
    return is_prop_supported(v) && (v->flags >> 30);
};

static inline BOOL is_prop_enabled(lucam_prop* v)
{
    return is_prop_supported(v) && (v->flags >> 15);
};

static inline BOOL is_prop_auto(lucam_prop* v)
{
    return is_prop_auto_supported(v) && (v->flags >>14 );
};

#if 0
#define MAX_INTEL_HEX_RECORD_LENGTH 16
typedef struct _INTEL_HEX_RECORD
{
    BYTE  Length;
    WORD  Address;
    BYTE  Type;
    BYTE  Data[MAX_INTEL_HEX_RECORD_LENGTH];
} INTEL_HEX_RECORD, *PINTEL_HEX_RECORD;

typedef struct _ALTERA_RBF_RECORD
{
  __u32 Length;
  BYTE Data[];
} ALTERA_RBF_RECORD, *PALTERA_RBF_RECORD;
#endif

struct _FpgaConfigDescriptor2
{
   const unsigned char *FpgaConfig;
   unsigned long Size;
   unsigned long ProgramCode;
};
struct _FpgaList2
{
   unsigned short Pid;
   unsigned short Did;
   struct _FpgaConfigDescriptor2 *FpgaDescriptors;
   int FpgaDescriptorsCount;
};



struct _ModelDta2
{
   unsigned short Pid;
   unsigned short Did;
   const unsigned char *Pointer;
   int Size;
};



#define DEBUG_COUNT(ptr, name) ptr->counters.name++


/****camera registers****************/

typedef struct _lucam_param
{
   __u16 reg_index;
   lucam_prop value;
} lucam_param, *plucam_param;

typedef struct _lucam_reg32
{
   __u16 reg_index;
   __u32  value;
} lucam_reg32;

typedef struct
{
   lucam_reg32   sensor_init, usb_speed, fpga_mode, format_count, video_en, trigger_ctl, color_inq;
   lucam_reg32   max_size, unit_size, fo_kps_minmax, fo_kps, fo_kps_cnt, fo_kps_array;
   lucam_param   gamma, contrast, brightness;
   lucam_reg32   fo_color_id, fo_position, fo_size, fo_subsampling, fo_subsampling_inq;
   lucam_reg32   st_position, st_size, st_color_id, st_subsampling;
   lucam_reg32   fo_tap_configuration, st_tap_configuration;
   lucam_param   fo_exposure, fo_gain, fo_gain_red, fo_gain_green1, fo_gain_green2, fo_gain_blue;
   lucam_param   iris, focus;
   lucam_param   still_knee1_exposure, still_knee2_exposure, still_knee3_exposure, video_knee, knee2_level;
   lucam_param   st_exposure, st_gain, st_gain_red, st_gain_green1, st_gain_green2, st_gain_blue;
   lucam_param   st_strobe_delay, st_exposure_delay, st_strobe_duration, auto_exposure_max, auto_gain_min, auto_gain_max;
   lucam_param   snapshot_count, snapshot_setting, st_shutter_type, timestamps, bpc, video_settings,lsc_x,lsc_y,trigger_mode;
   lucam_param   trigger_pin, strobe_pin,auto_exp_target,realtimestamp,realtimestamp_frequency,timestamp_hw_reset,focal_length,iris_steps_count;
   lucam_reg32   message_support;
} lucam_registers;



struct _lucam_device;
struct _frame_buf;
struct _image_buf;
struct _bulk_buf;


typedef struct _image_buf
{
	/* common v4l buffer stuff -- must be first */
	struct videobuf_buffer vb;

	struct _lucam_device *plucam; /* the device model */

	void*     data;  /* the image data */
	volatile __u32 offset; /* where to DMA in the next RAW data
				* packet */
#if ALLOW_RGB24_OUTPUT
	int needs_conversion; /* if true then after the capture the
			       * image needs to be converted to a
			       * RGB24 format. */
	void *raw_data;  /* if transformation is needed this holds the
			  * raw image data. points into
			  * *raw_alloc_data on a page aligned
			  * boundary */
	void *raw_alloc_data; 
#endif
	int status;
	int error_code; // used for completion

} lucam_image_buf, *plucam_image_buf;

/* A bulk_buf represents a bulk transaction
 */
typedef struct _bulk_buf
{
	struct _image_buf *pimage_buf; // ptr to corresponding raw image buffer
	int index;   // in the array of bulk_buf
	int is_first; // true if it is the first transaction of a image_buf
	int is_last; // true if it is the last transaction of a image_buf
	int submitted; // true if the usbcore owns it

#if MEM_IN_BULK_BUF
	void *data;
	size_t offset;
#endif

	struct urb *urb;
} lucam_bulk_buf, *plucam_bulk_buf;

#define MAX_BULK_PACKETS      (15)

//#define BULK_PACKET_SIZE     4096
#define CTRL_PACKET_SIZE     64

struct lucam_fh {
	struct _lucam_device            *dev;
	enum v4l2_buf_type         type;
};

#define LUCAM_STREAM_STATE_STOP         0
#define LUCAM_STREAM_STATE_PAUSE        3
#define LUCAM_STREAM_STATE_RUN2PAUSE    4
#define LUCAM_STREAM_STATE_PAUSE2RUN    5
#define LUCAM_STREAM_STATE_RUN          6

#define LUCAM_CURRENT_SPECIFICATION     2


/****Lucam Device*****/
typedef struct _lucam_device
{
   struct usb_device   *udev;
   struct video_device *vdev;

#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(3,11,0)
   struct v4l2_device v4l2_dev;
#endif

   __u32 specification;
   __u32 lucam_flags;
   __u32 EmbeddedVersion;

   unsigned long*  kpsarray;
   int kpsarraycnt;

   lucam_registers     cam_registers;

   void *lut8;
   int lut8len;

   volatile LUCAM_MODE camera_mode;              //video or snapshot ??
   volatile int opencount;                         // how many times it is opened?
   volatile struct file *file_that_set_fmt;
   volatile struct file *file_holding_eeprom_lock;
   int eeprom_lock_count;
   volatile BYTE unplugged;                      //is the device unplugged?
   wait_queue_head_t remove_ok;

   BYTE index;                                   //which slot the current device is in --multiple device support
   struct semaphore mutex;

   //__u32  strobe_delay;

   signed long imagesize_max;                    // = max_w*max_h*3

   //spinlock_t        lock;

   struct semaphore control_mutex; // for accessing control endpoint 0

#if USB3_ENDPOINT_CONFIGURATION
   unsigned int miscdataout_pipe;
   unsigned int video_pipe;
   unsigned int miscdatain_pipe;
#else
   int alt_setting;

   int fpga_setting; // normally 1 for USB2, 0 for USB1
   int data_setting; // normally 2 for USB2, 0 for USB1

   unsigned int fpga_pipe;
   unsigned int video_pipe;
   unsigned int still_pipe;
#endif

   int stream_count; // normally 2, can be 1

   // struct proc_dir_entry *proc_entry;
   unsigned long serial_number;

   /* from here it is all stuff related to streaming and its mechanism */

   __u32 pixelformat;
   size_t framesize;  // of the raw data, not necessarly valid if the stream is stopped
   int outwidth;
   int outheight;

   //size_t imagesize;  // of the image w.r.t. pixelformat and output dimensions
   size_t bitcount;      // of the raw data, not necessarly valid if the stream is stopped

   int current_stream_state; // 0: stopped, 1: paused, 2: running
   int wanted_stream_state; // wanted from app: 0: stopped, 1: paused, 2: running
   int frame_error;         // if not 0 then a frame error was detected

   int last_bulk_index;
   int bulk_bufs_used; // inited when allocating stuff, always <= MAX_BULK_PACKETS
   lucam_bulk_buf    bulk_bufs[MAX_BULK_PACKETS];
#if ENHANCED_BUFFERING
   atomic_t bulk_buf_idle_count;
   int bulk_buf_idle_first;
#endif
   unsigned long bulk_lock;

	/****************************
	 * The USB packet pump fills frames. When they are full, they
	 * are delivered as images after any transformations required
	 * have been done. 
	 *
	 * If no transformation is needed, then the data is delivered
	 * direct to the image buffers from the USB dma engine.
	 *
	 * If a transformation is needed, then the frame's storage is
	 * allocated lazily, the USB DMA engine delivers to there, and
	 * the transformation happens from the frame storage to the
	 * image buffer.
	 *****************************/
#if !ENHANCED_BUFFERING
	lucam_image_buf   blank_image_buf; // if there is no image_buf available then this image_buf is used.
#endif

	/************************
	 * images queued for the usb packet pump to fill
	 ************************/
	struct list_head active_frames;

	/*****************************************
	 * used by the usb packet pump
	 *****************************************/

	// image_buf currently being filled with bulk data but still
	// not entirely being passed to the usb layer
	plucam_image_buf current_image_buf; 

	// When an error occurs while a image_buf is owned by the usb
	// layer this will be set to the errored frame buf during the
	// cancellation (bulk_buf->is_last)
	plucam_image_buf error_image_buf;

	/***********************
	 * a queue of image_buf that needs to be filled with bulk data
	 ***********************/
	struct videobuf_queue vb_vidq;
	spinlock_t slock;
#if  VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,37)
   struct mutex vb_vidq_ext_mutex;
#endif

   size_t frames;          // number of the frames that have been taken

   /*The stream state (running, stopped, ... ) may be changed from an app command; or
    * from an error in the stream. In the last case the stream needs to be restarted.
    * This can only be done from a scheduled work/task. For synchronization purposes all
    * stream state changes are done from a scheduled work/task.
    */
   wait_queue_head_t state_changed_queue;
   struct work_struct change_state_task;
   struct semaphore change_state_task_mutex;

   int camera_enabled; // true is camera is pushing data thru usb

   __u32 timeout;

   __u64 timestampLatched;

   wait_queue_head_t event_queue;
   lucam_event_bits event_pending;

	struct lucam_counters counters;

#ifdef DEBUG
   int debug;
#endif
} lucam_device, *plucam_device;

/****************************************************************************
 *
 *  Trival helper functions
 *
 *
 ***************************************************************************/

static inline WORD LO_WORD(__u32 i)
{
    return i & 0xffff;
}


static inline WORD HI_WORD(__u32 i)
{
    return (i>>16) & 0xffff;
}


static inline __u32 words_to_u32(WORD l, WORD h)
{
    __u32 value=h;
    value=l+(value<<16);
    return value;
}


static inline WORD bytes_to_word(BYTE l, BYTE h)
{
    WORD value=h;
    value=l+(value<<8);
    return value;
}


#ifndef abs
#define abs(x) (x>=0?(x):(-x))
#endif

#if 0
typedef enum{RED=0, GREEN1=1, GREEN2=2, BLUE=3}
BAYER_RGB;

void lucam_bayer_to_rgb8( BYTE* rgb_buf, const BYTE *rawdata_buf, int width, int height, BAYER_RGB topleft);
void mono8_to_rgb8(BYTE* rgb_buf, BYTE *rawdata_buf, int width, int height);
void mono16_to_rgb8(BYTE* rgb_buf, BYTE *rawdata_buf, int width, int height);
void mono61_to_rgb8(BYTE* rgb_buf, BYTE *rawdata_buf, int width, int height);
#endif

extern int lucam_ext_cmd_rw(plucam_device plucam, __u16 usbindex, __u16 usbvalue, void* val, int len, BYTE dir);

#ifdef DRIVER_INITRESET
extern int CustomInitReset(plucam_device plucam);
#endif

#ifdef DRIVER_SOFT_FORMAT_VALIDATION
extern int CustomSoftFormatValidate(plucam_device plucam, int stillMode, unsigned int colorid, unsigned int width, unsigned int height);
extern int CustomSoftFormatAdjustStartPos(plucam_device pExt, int stillMode);
extern int CustomSoftFormatAdjustColorId(plucam_device pExt, int stillMode);
#endif


#endif // __KERNEL__
#endif

