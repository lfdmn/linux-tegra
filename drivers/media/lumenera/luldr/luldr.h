/*  Copyright Lumenera Corporation 2005
 *  All Rights Reserved
 *
 */

#ifndef LULDR_H
#define LULDR_H

#include <linux/version.h>
#include <linux/usb.h>
#include <linux/kernel.h>                         /* printk() */
#include <linux/spinlock.h>


/*
 * If set then the driver will attempt to
 * verify the download.
 */
#define VERIFY_FIRMWARE                     0

/* Only applicable if VERIFY_FIRMWARE is 1 */
#define MAX_RETRIES                         3

#define MAX_DOWNLOAD_PACKET_SIZE            (16)

//#define LULDR_DEBUG



#if LINUX_VERSION_CODE > KERNEL_VERSION(2,4,18)
typedef struct urb urb_t, *purb_t;
#endif


#ifndef _BYTE_DEFINED
#define _BYTE_DEFINED
typedef __u8 BYTE;
#endif                                            // !_BYTE_DEFINED

#ifndef _BOOL_DEFINED
#define _BOOL_DEFINED
typedef enum {FALSE=0, TRUE=1}
BOOL;
#endif                                            // !_BOOL_DEFINED

#ifndef _WORD_DEFINED
#define _WORD_DEFINED
typedef __u16 WORD;
#endif                                            // !_WORD_DEFINED


#define LULDR_NAME          "luldr"
#define LULDR_VERSION       "0.01"
#define VID_HARDWARE_LULDR  0x5354                //vendor ID
#define VID_HARDWARE_LULDR2 0x1724                //vendor ID

//request
#define FX2_FIRMWARE_LOAD   0xA0                  //for firmware load

#define FX2_REG_CPUCS       0xE600                //registers for reseting 8051
#define FX2_RESET_CPU       0x01
#define FX2_SET_CPU         0x00

#define MAX_CAMERA_NUMBER   4

#define TIMEOUT             HZ*10

#define MAX_INTEL_HEX_RECORD_LENGTH 16
typedef struct _INTEL_HEX_RECORD
{
    BYTE  Length;
    WORD  Address;
    BYTE  Type;
    BYTE  Data[MAX_INTEL_HEX_RECORD_LENGTH];
} INTEL_HEX_RECORD, *PINTEL_HEX_RECORD;

struct _FirmwareList2
{
   unsigned short Pid;
   unsigned short Did;
   INTEL_HEX_RECORD *Record;
};

typedef struct
{
    struct usb_device   *udev;

    volatile BYTE opened;                         //is the device opened?
    volatile BYTE unplugged;                      //is the device unplugged?
    wait_queue_head_t remove_ok;

    BYTE index;                                   //which slot the current device is in --multiple device support
    struct semaphore mutex;

    spinlock_t        lock;
    wait_queue_head_t reading_queue;

} luldr_device, *pluldr_device;

/*****************Message routines**********************/
#define TRACE_MODULE    0x0001
#define TRACE_PROBE     0x0002
#define TRACE_OPEN      0x0004
#define TRACE_READ      0x0008
#define TRACE_MEMORY    0x0010
#define TRACE_FLOW      0x0020
#define TRACE_SIZE      0x0040
#define TRACE_REG       0x0080
#define TRACE_SEQUENCE  0x1000
#define TRACE_ERROR     0x2000
#define TRACE_INFO      0x4000

#ifdef DEBUG
#  ifdef __KERNEL__
/* Trace certain actions in the driver */
/* This one if debugging is on, and kernel space */
#define Trace(R, A...) printk(KERN_ERR LULDR_NAME " " A)
#  else
/* This one for user space */
#define Trace(R, A...) if (lucam_trace_state & R) fprintf(stdout, A)
#  endif
#else
#define Trace(R, A...)
#endif

#define Info(A...)  Trace(TRACE_ERROR, A)
#define Err(A...)   Trace(TRACE_ERROR, A)

#define SWAP16(u)    ((((u) & 0xff) << 8) | (((u) & 0xff00) >> 8))

#endif
