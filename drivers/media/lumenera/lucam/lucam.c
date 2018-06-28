/*************************************************************************
 * lucam.c:  Driver for Lumenera USB cameras
 *
 *  Copyright(c) 2005-2009 Lumenera
 *
 * ---
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 **************************************************************************/

// TODO eliminate the NOKERNEL hack
#ifdef NOKERNEL
#include "lucam.h"
#else
#include <linux/lucam.h>
#endif
#include <linux/module.h>

#include "lucam_def.h"
#include "lucam_ioctl.h"

#include <asm/io.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <linux/capability.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/moduleparam.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/slab.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
#include <linux/smp_lock.h>
#endif
#include <linux/time.h>
#include <linux/usb.h>

#ifndef CUSTOM_MANUFACTURER_SHORTNAME
#define CUSTOM_MANUFACTURER_SHORTNAME "Lumenera"
#endif
#ifndef CUSTOM_MANUFACTURER_NAME
#define CUSTOM_MANUFACTURER_NAME "Lumenera Corporation"
#endif

/* this trace bits have to co-exist with the base debugging in
 * videodev which claims 0x01 and 0x02 */
#define TRACE_MODULE    0x0100
#define TRACE_PROBE     0x0200
#define TRACE_OPEN      0x0004
#define TRACE_POLL      0x0008
#define TRACE_READ      0x0008
#define TRACE_MEMORY    0x0010
#define TRACE_V4L2      0x0020
#define TRACE_BULK      0x0040
#define TRACE_REG       0x0080
#define TRACE_USB       0x0080
#define TRACE_SEQUENCE  0x1000
#define TRACE_ERROR     0x2000
#define TRACE_INFO      0x4000

/* Trace certain actions in the driver */
#if  LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
#ifdef DEBUG
#define Trace(kern_level, dev, flags, A...)             \
    do {if ((lucam_device *)dev) {                   \
            if ((flags) & (TRACE_ERROR | ((lucam_device *)dev)->debug)) printk(kern_level "" LUCAM_NAME ":" A); \
        } else printk(KERN_ERR LUCAM_NAME ":" A); \
    }while (0)
#else
#define Trace(kern_level, dev, flags, A...)             \
    do {printk(KERN_ERR LUCAM_NAME ":" A); \
    }while (0)
#endif
#else
#ifdef DEBUG
#define Trace(kern_level, dev, flags, A...)             \
    do {if (((lucam_device *)dev) && ((lucam_device *)dev)->vdev) {                 \
            if ((flags) & (TRACE_ERROR)) printk(kern_level "" LUCAM_NAME ":" A); \
        } else printk(KERN_ERR LUCAM_NAME ":" A); \
    }while (0)
#else
#define Trace(kern_level, dev, flags, A...)             \
    do {printk(KERN_ERR LUCAM_NAME ":" A); \
    }while (0)
#endif
#endif

#define Err(dev, A...)   Trace(KERN_ERR, dev, TRACE_ERROR, A)
#ifdef DEBUG
#define Info(dev, A...)  Trace(KERN_INFO, dev, A)
#else
#define Info(A...)
#endif

#if  VIDEODEV_VERSION_CODE <= KERNEL_VERSION(2,6,24)
#define VIDEOBUF_NEEDS_INIT      STATE_NEEDS_INIT
#define VIDEOBUF_PREPARED        STATE_PREPARED
#define VIDEOBUF_QUEUED          STATE_QUEUED
#define VIDEOBUF_ACTIVE          STATE_ACTIVE
#define VIDEOBUF_DONE            STATE_DONE
#define VIDEOBUF_ERROR           STATE_ERROR
#define VIDEOBUF_IDLE            STATE_IDLE
#endif





/* Declare static vars that will be used as parameters */
static int usbport_nr = 0;

static struct v4l2_queryctrl lucam_qctrl[] = {
    {
        .id            = V4L2_CID_BRIGHTNESS,
        .type          = V4L2_CTRL_TYPE_INTEGER,
        .name          = "Brightness",
        .minimum       = 0,
        .maximum       = 255,
        .step          = 1,
        .default_value = 127,
        .flags         = 0,
    }, {
        .id            = V4L2_CID_CONTRAST,
        .type          = V4L2_CTRL_TYPE_INTEGER,
        .name          = "Contrast",
        .minimum       = 0,
        .maximum       = 255,
        .step          = 0x1,
        .default_value = 0x10,
        .flags         = 0,
    }, {
        .id            = V4L2_CID_SATURATION,
        .type          = V4L2_CTRL_TYPE_INTEGER,
        .name          = "Saturation",
        .minimum       = 0,
        .maximum       = 255,
        .step          = 0x1,
        .default_value = 127,
        .flags         = 0,
    }, {
        .id            = V4L2_CID_HUE,
        .type          = V4L2_CTRL_TYPE_INTEGER,
        .name          = "Hue",
        .minimum       = -128,
        .maximum       = 127,
        .step          = 0x1,
        .default_value = 0,
        .flags         = 0,
    }
};

static int qctl_regs[ARRAY_SIZE(lucam_qctrl)];

extern struct _FpgaList2 FpgasList[];
extern int FpgasListCount;

extern struct _ModelDta2 ModelDtaList[];
extern int ModelDtaListCount;

struct video_device lucam_template;

#ifdef DEBUG
static inline void PRINT_VALUE(const plucam_device plucam, const char* prefix, const lucam_prop* val)
{
   Info(plucam, TRACE_REG, "%s:= [0x%4X, %d, %d ,%d] \n", prefix, val->flags, val->value, val->min, val->max);
}
#endif


/********* some local funcs decls ******************/

static int lucam_set_stream_state(plucam_device plucam, int state);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
void lucam_change_stream_state(struct work_struct *context);
#else
void lucam_change_stream_state(void *context);
#endif
void lucam_complete_frame(void *context);
int lucam_wait_for_complete_frame_task(plucam_device plucam, int interruptible);
#if ALLOW_RGB24_OUTPUT
static int lucam_fill_image_buf(plucam_device plucam, plucam_image_buf pimage_buf);
#endif
#if ENHANCED_BUFFERING
static int lucam_submit_bulkbufs(struct _lucam_device *plucam, int gfp);
#else
static void lucam_resubmit_bulk_buf(struct _lucam_device *plucam, struct _bulk_buf *pbulk_buf);
#endif
static int vidioc_enum_framesizes(struct file *file, void *fh, struct v4l2_frmsizeenum *frmsize);
static int vidioc_enum_frameintervals(struct file *file, void *fh, struct v4l2_frmivalenum *frmsize);

#if USB3_ENDPOINT_CONFIGURATION
static int lucam_flash_read( plucam_device plucam, lucam_flash_access *prflash);
static int lucam_flash_erase( plucam_device plucam, lucam_flash_access *pwflash );
static int lucam_flash_program( plucam_device plucam, lucam_flash_access *pwflash);
static int Is_flash_accessible(plucam_device plucam);
#endif


/****************************************************************************
 *
 *  Find a spare lucam_device structure for the current device
 *  -- Support up to MAX_CAMERA_NUMBER cameras.
 *
 ***************************************************************************/
static lucam_device lucam_dev[MAX_CAMERA_NUMBER];

/************************************************************************************
*
***********************************************************************************/
static void lucam_destroy_camera(plucam_device plucam)
{
    Info(plucam, TRACE_OPEN, "lucam_destroy_camera()\n");

    //usb_set_intfdata(interface, NULL);    /* FIXME: why? */

    if (plucam->udev == NULL)
    {
        Err(plucam, "lucam_destroy_camera() already called for %p\n", plucam);
        return;
    }
    //plucam->unplugged = 1; // should be set already
    //lucam_set_stream_state(plucam, LUCAM_STREAM_STATE_STOP);
    if(plucam->vdev)
    {
        video_unregister_device(plucam->vdev);
    }
    Info(plucam, TRACE_OPEN, "Destroyed \n");
    plucam->vdev=NULL;
    plucam->udev = NULL;
    if (plucam->lut8)
    {
        kfree(plucam->lut8);
        plucam->lut8 = NULL;
        plucam->lut8len = 0;
    }
    if(plucam->kpsarray)
    {
        kfree(plucam->kpsarray);
        plucam->kpsarray = NULL;
        plucam->kpsarraycnt = 0;
    }
    //MOD_DEC_USE_COUNT;
}

#if !USB3_ENDPOINT_CONFIGURATION
/****************************************************************************
*
*
*
*
***************************************************************************/
static int lucam_select_alt_setting(plucam_device plucam, int alt_setting)
{
   struct usb_device *udev = plucam->udev;
   int ret;

   if (plucam->alt_setting == alt_setting) return 0;

   if (down_interruptible(&plucam->control_mutex))
   {
      return -ERESTARTSYS;
   }
   ret = usb_set_interface (udev, 0, alt_setting);
   if(ret < 0)
   {
      Err(plucam, "***usb_set_interface failed=%d\n", ret);
   }
   else
   {
      plucam->alt_setting = alt_setting;
   }
   up(&plucam->control_mutex);
   return ret;
}
#endif

/**************************************************************************
* Note:
* There is a bug in the Linux xhci driver here: It does not reset the host
* side USB3 sequence number unless the endpoint is actually halted. 
*************************************************************************/
static int lucam_clear_halt(plucam_device plucam, int pipe)
{
   Info(plucam, TRACE_INFO, "lucam_clear_halt\n");
   if (down_interruptible(&plucam->control_mutex))
   {
      return -ERESTARTSYS;
   }
   if (usb_clear_halt(plucam->udev, pipe) < 0)
   {
      Err(plucam, "***Failed to clear halt\n");
   }
   up(&plucam->control_mutex);
   return 0;
}


/*****************************************************************************
*
* NOTE plucam->vdev is not set yet, so don't use Err or Info!
*
******************************************************************************/
static int lucam_read_serial_number(plucam_device plucam)
{
    int rt;
    char snBuf[256];
    int i;

    plucam->serial_number = 0;

    if (plucam->udev->descriptor.iSerialNumber == 0)
    {
        printk(KERN_INFO CUSTOM_MANUFACTURER_SHORTNAME " Camera without a serial number\n");
        plucam->serial_number = 0;
    }

    if (down_interruptible(&plucam->control_mutex))
    {
        return -ERESTARTSYS;
    }

    rt = usb_string(plucam->udev,
        plucam->udev->descriptor.iSerialNumber,
        snBuf,
        sizeof(snBuf));

    if (rt < 0)
    {
        printk(KERN_ERR "***Failed to get sn size\n");
        goto unlockandexit;
    }

    if (rt == 0)
    {
        printk(KERN_ERR "***String with 0 length!!\n");
        rt = -EFAULT;
        goto unlockandexit;
    }

    for (i = 0 ; i < rt ; i++)
    {
       plucam->serial_number *= 10;
       plucam->serial_number += ((unsigned long)((snBuf[i]) - '0') % 10);
    }

    printk( KERN_INFO "Camera serial number=%ld\n", plucam->serial_number);

 unlockandexit:
   up(&plucam->control_mutex);
   return rt;
}

/****************************************************************************
* lucam_enum_endpoints
*
* This function enumerates the endpoints. This should be called early after
* the device connection. When this function exits:
* - plucam->stream_count should reflect the capability for a still mode
*           (2: still mode supported, 1: still mode not supported).
* - plucam->fpga_setting is the alternate interface setting for the FPGA endpoint.
* - plucam->fpga_pipe: Pipe for FPGA endpoint.
* - plucam->data_setting: Alternate interface setting for streaming data (video & still).
* - plucam->video_pipe: Pipe for video data.
* - plucam->still_pipe: Pipe for snapshot data (0 if snapshot is not supported).
*
*****************************************************************************/
static int lucam_enum_endpoints(plucam_device plucam)
{
   struct usb_host_config *pHostConfig;
   struct usb_interface * pIf;
   struct usb_host_interface *pHostIf; /* alt if setting */
   struct usb_host_endpoint *pHostEndpoint;
   unsigned altSetting;
   int endpoint;
   unsigned int pipe;

   plucam->stream_count = 0;
   plucam->video_pipe = 0;
#if USB3_ENDPOINT_CONFIGURATION
   plucam->miscdatain_pipe = 0;
   plucam->miscdataout_pipe = 0;
#else
   plucam->fpga_setting = -1;
   plucam->data_setting = -1;
   plucam->still_pipe = 0;
   plucam->fpga_pipe = 0;
#endif

   pHostConfig = plucam->udev->actconfig;
   pIf = pHostConfig->interface[0]; /* Should use usb_ifnum_to_if() ? */

#if USB3_ENDPOINT_CONFIGURATION
   for (altSetting = 0 ; altSetting < 1                  ; altSetting++)
#else
   for (altSetting = 0 ; altSetting < pIf->num_altsetting; altSetting++)
#endif
   {
      pHostIf = &pIf->altsetting[altSetting]; /* Sould use usb_altnum_to_altsetting() ?? */

      for (endpoint = 0 ; endpoint < pHostIf->desc.bNumEndpoints; endpoint++)
      {
         pHostEndpoint = &pHostIf->endpoint[endpoint];

         switch (pHostEndpoint->desc.bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
         {
         case USB_ENDPOINT_XFER_BULK:
            if (pHostEndpoint->desc.bEndpointAddress & USB_ENDPOINT_DIR_MASK)
            {
               /* IN endpoint */
               pipe = usb_rcvbulkpipe(plucam->udev, pHostEndpoint->desc.bEndpointAddress);
               if (plucam->video_pipe == 0)
               {
                  plucam->video_pipe = pipe;
#if !USB3_ENDPOINT_CONFIGURATION
                  plucam->data_setting = altSetting;
#endif
               }
#if USB3_ENDPOINT_CONFIGURATION
               else if (plucam->miscdatain_pipe == 0)
               {
                  plucam->miscdatain_pipe = pipe;
               }
#else
               else if (plucam->still_pipe == 0 && altSetting == plucam->data_setting)
               {
                  plucam->still_pipe = pipe;
               }
#endif
            }
            else
            {
               /* OUT endpoint */
               pipe = usb_sndbulkpipe(plucam->udev, pHostEndpoint->desc.bEndpointAddress);
#if USB3_ENDPOINT_CONFIGURATION
               if (plucam->miscdataout_pipe == 0)
               {
                  plucam->miscdataout_pipe = pipe;
               }
#else
               if (plucam->fpga_pipe == 0)
               {
                  plucam->fpga_pipe = pipe;
                  plucam->fpga_setting = altSetting;
               }
#endif
            }
            break;
         default:
            break;
         }
      }
   }

#if USB3_ENDPOINT_CONFIGURATION
   if (plucam->video_pipe)
   {
      /*
         Note:
         This function is called before the remainder of the registers are read. 
         Some cameras have 2 streams even if they have only one IN BULK endpoint.
         The format_count register will reflect that later and then the stream_count
         be corrected.
      */
      plucam->stream_count = 1;
   }
   if (plucam->miscdatain_pipe == 0)
   {
      plucam->miscdatain_pipe = plucam->video_pipe;
   }
#else
   if (plucam->still_pipe)
   {
      /*
         Note:
         This function is called before the remainder of the registers are read. 
         Some cameras have 2 streams even if they have only one IN BULK endpoint.
         The format_count register will reflect that later and then the stream_count
         be corrected.
      */
      plucam->stream_count = 2;
   }
   else if (plucam->video_pipe)
   {
      plucam->stream_count = 1;
   }
#endif

   if (plucam->stream_count == 0)
   {
      Err(plucam, "***%s: plucam->stream_count == 0\n", __func__);
      return -1;
   }
   return 0;
}


/***************************************************************************
*
***************************************************************************/
int lucam_generic_usb_vendor_request_rw(plucam_device plucam, __u8 req, __u16 usbindex, __u16 usbvalue, void* val, int len, BYTE dir)
{
    int ret, pipe, request_type;
    BYTE *transfer_buffer;
    struct usb_device *udev;

    udev = plucam->udev;

    Info(plucam, TRACE_REG, "lucam_ext_cmd_rw: index(%d) value(0x%04x)\n", (int)usbindex, (int)usbvalue);

    transfer_buffer =  kmalloc (len, GFP_KERNEL);
    if (!transfer_buffer)
    {
        Err(plucam, "***lucam_ext_cmd_rw: kmalloc(%d) failed.\n", len);
        return -ENOMEM;
    }

    request_type= USB_TYPE_VENDOR | USB_RECIP_DEVICE | dir;

    if(dir==USB_DIR_IN)                           //read
    {
        pipe= usb_rcvctrlpipe( udev, 0);
    }
    else                                          //write
    {
        pipe= usb_sndctrlpipe( udev, 0);
        memcpy (transfer_buffer, val, len);
    }
    if (down_interruptible(&plucam->control_mutex))
    {
        Err(plucam, "***lucam_ext_cmd_rw: Received signal when takeing control mutex\n");
        if (NULL != transfer_buffer)
        {
           kfree(transfer_buffer);
           transfer_buffer = NULL;
        }
        return -ERESTARTSYS;
    }
    ret=usb_control_msg(udev, pipe, req, request_type, usbvalue,
        usbindex, transfer_buffer, len, USB_CTRL_MSG_TIMEOUT);
    up(&plucam->control_mutex);
    if((ret>=0)&&(dir==USB_DIR_IN))
    {
        if (ret != len)
        {
            ret = -EFAULT;
        }
        else
        {
            memcpy(val, transfer_buffer, len);
        }
    }

   kfree (transfer_buffer);
   if (ret < 0)
   {
      Err(plucam, "***lucam_ext_cmd_rw failed\n");
   }
   return ret;
}

int lucam_ext_cmd_rw(plucam_device plucam, __u16 usbindex, __u16 usbvalue, void* val, int len, BYTE dir)
{
   return lucam_generic_usb_vendor_request_rw(plucam, LUMENERA_EXT_CMD, usbindex, usbvalue, val, len, dir);
}

/****************************************************************************
 *
 *  Send the control message through end point 0
 *
 *
 ***************************************************************************/
static int lucam_reg08_rw(plucam_device plucam, __u16 reg_index, __u8 * val, int len, BYTE dir)
{
   struct usb_device *udev;
   int ret,pipe,request_type;
   BYTE *transfer_buffer;

   udev = plucam->udev;

   transfer_buffer =  kmalloc (len, GFP_KERNEL);
   if (!transfer_buffer)
   {
      Err(plucam, "***lucam_reg08_rw: kmalloc(%d) failed.\n", len);
      return -ENOMEM;
   }
   //0x40
   request_type= USB_TYPE_VENDOR | USB_RECIP_DEVICE | dir;

   if(dir==USB_DIR_IN)                           //read
   {
      pipe= usb_rcvctrlpipe( udev, 0);
   }
   else                                          //write
   {
      pipe= usb_sndctrlpipe( udev, 0);
      memcpy (transfer_buffer, (BYTE*)val, len);
   }
   if (down_interruptible(&plucam->control_mutex))
   {
      Err(plucam, "***lucam_reg08_rw: Received signal when takeing control mutex\n");
      if (NULL != transfer_buffer)
      {
         kfree(transfer_buffer);
         transfer_buffer = NULL;
      }
      return -ERESTARTSYS;
   }
   if (dir == USB_DIR_OUT)
   {
      Info(plucam, TRACE_REG, "Writing to 0x%x bytes to reg 0x%x\n", len, (unsigned int)reg_index);
   }
   ret=usb_control_msg(udev, pipe, LUMENERA_REQUEST, request_type, 0,
      reg_index, transfer_buffer, len, USB_CTRL_MSG_TIMEOUT);
   up(&plucam->control_mutex);
   if(ret>=0 &&  dir==USB_DIR_IN)
   {
      if (ret != len)
      {
         ret = -EFAULT;
      }
      else
      {
         memcpy((BYTE*)val, transfer_buffer, len);
         Info(plucam, TRACE_REG, "Read from reg 0x%x bytes to reg 0x%x\n", len, (unsigned int)reg_index);
      }
   }
   kfree (transfer_buffer);
   if (ret < 0)
   {
      Err(plucam, "***lucam_reg08_rw failing with %d\n", ret);
   }
   return ret;
}

/***************************************************************************
*
****************************************************************************/
static int lucam_reg32_rw(plucam_device plucam, __u16 reg_index, __u32 * val, int number, BYTE dir)
{
    struct usb_device *udev;
    int ret,pipe,len,request_type;
    BYTE *transfer_buffer;
#if !defined (__LITTLE_ENDIAN)
    int i;
#endif

    udev = plucam->udev;

    len=number*4;
    transfer_buffer =  kmalloc (len, GFP_KERNEL);
    if (!transfer_buffer)
    {
        Err(plucam, "***lucam_reg32_rw: kmalloc(%d) failed.\n", len);
        return -ENOMEM;
    }
    //0x40
    request_type= USB_TYPE_VENDOR | USB_RECIP_DEVICE | dir;

    if(dir==USB_DIR_IN)                           //read
    {
        pipe= usb_rcvctrlpipe( udev, 0);
    }
    else                                          //write
    {
        pipe= usb_sndctrlpipe( udev, 0);
#if defined (__LITTLE_ENDIAN)
        memcpy (transfer_buffer, (BYTE*)val, len);
#else
        for (i = 0 ; i < number ; i++)
        {
            transfer_buffer[4*i+0] = (BYTE)(val[i] & 0xff);
            transfer_buffer[4*i+1] = (BYTE)((val[i] >> 8) & 0xff);
            transfer_buffer[4*i+2] = (BYTE)((val[i] >> 16) & 0xff);
            transfer_buffer[4*i+3] = (BYTE)((val[i] >> 24) & 0xff);
        }
#endif
    }
    if (down_interruptible(&plucam->control_mutex))
    {
        Err(plucam, "***lucam_reg32_rw: Received signal when takeing control mutex\n");
        if (NULL != transfer_buffer)
        {
           kfree(transfer_buffer);
           transfer_buffer = NULL;
        }        
        return -ERESTARTSYS;
    }
#ifdef DEBUG
    if (dir == USB_DIR_OUT)
    {
        if (len == 4)
        {
            Info(plucam, TRACE_REG, "Writing to register 0x%x, val=0x%x\n", (unsigned int)reg_index, val[0]);
        }
        else if (len == 8)
        {
            Info(plucam, TRACE_REG, "Writing to register 0x%x, flags=0x%x, val=0x%x\n", (unsigned int)reg_index, val[0], val[1]);
        }
        else
        {
            Info(plucam, TRACE_REG, "Writing to 0x%x bytes to reg 0x%x\n", len, (unsigned int)reg_index);
        }
    }
#endif
    ret=usb_control_msg(udev, pipe, LUMENERA_REQUEST, request_type, 0,
        reg_index, transfer_buffer, len, USB_CTRL_MSG_TIMEOUT);
    up(&plucam->control_mutex);
    if(ret>=0 &&  dir==USB_DIR_IN)
    {
        if (ret != len)
        {
            ret = -EFAULT;
        }
        else
        {
#if defined (__LITTLE_ENDIAN)
            memcpy((BYTE*)val, transfer_buffer, len);
#else
            for (i = 0 ; i < number ; i++)
            {
                val[i] = ((__u32)transfer_buffer[4*i+3] << 24) |
                    ((__u32)transfer_buffer[4*i+2] << 16) |
                    ((__u32)transfer_buffer[4*i+1] << 8) |
                    ((__u32)transfer_buffer[4*i+0]);
            }
#endif
#ifdef DEBUG
            if (len == 4)
            {
                Info(plucam, TRACE_REG, "Read from reg 0x%x, val=0x%x\n", (unsigned int)reg_index, val[0]);
            }
            else if (len == 16)
            {
                Info(plucam, TRACE_REG, "Read from reg 0x%x, flags=0x%x, val=0x%x, min=0x%x, max=0x%x\n", (unsigned int)reg_index, val[0], val[1], val[2], val[3]);
            }
            else
            {
                Info(plucam, TRACE_REG, "Read from reg 0x%x bytes to reg 0x%x\n", len, (unsigned int)reg_index);
            }
#endif
        }
    }
    kfree (transfer_buffer);
#ifdef DEBUG
    if (ret < 0)
    {
        Err(plucam, "***lucam_reg32_rw %s failing at index 0x%04x, with %d\n", (dir==USB_DIR_IN) ? "read" : "write", (unsigned int)reg_index , ret);
    }
#endif
    return ret;
}

/***************************************************************************
*
****************************************************************************/
static int lucam_message(plucam_device plucam, __u32 message_number, void *pMessage, int len, int dir)
{
    int rt;
    int length;
    unsigned long regAddr;

    if ((plucam->cam_registers.message_support.value & (1 << message_number)) == 0)
    {
        Err(plucam, "***Accessing unsupported message\n");
        return -EINVAL;
    }

    rt = lucam_reg32_rw(plucam, LUCAM_PROP_MESSAGE_NUMBER, &message_number, 1, USB_DIR_OUT);
    if (rt < 0)
    {
        Err(plucam, "***Failed to write to LUCAM_PROP_MESSAGE_NUMBER register\n");
        return rt;
    }

    regAddr = LUCAM_PROP_MESSAGE_BASE;
    if (len == 0)
    {
        rt = lucam_reg08_rw(plucam, LUCAM_PROP_MESSAGE_BASE, pMessage, 0, dir);
    }
    while(len)
    {
        if (len > 0x40 && message_number == LUCAM_PROP_MSG0_LUT8)
        {
            length = 0x40;
        }
        else if (len > 0x100)
        {
            length = 0x100;
        }
        else
        {
            length = len;
        }
        rt = lucam_reg08_rw(plucam, regAddr, pMessage, length, dir);
        if (rt < 0) break;

        len -= length;
        regAddr += (length&0xfff);
        pMessage += length;
    }
    if (rt < 0)
    {
        Err(plucam, "***Failed to transfer message at LUCAM_PROP_MESSAGE_BASE\n");
        return rt;
    }

    return rt;
}


/// read a register
static inline int lucam_get_reg(plucam_device plucam, lucam_reg32* reg)
{
   return lucam_reg32_rw(plucam, reg->reg_index, &reg->value, 1, USB_DIR_IN);
}


///write a register
static inline int lucam_set_reg(plucam_device plucam, lucam_reg32* reg)
{
   return lucam_reg32_rw(plucam, reg->reg_index, &reg->value, 1, USB_DIR_OUT);
}


/// read a lucam_param
static inline int lucam_get_param(plucam_device plucam, lucam_param* param)
{
   return lucam_reg32_rw(plucam, param->reg_index, (__u32*)&param->value, sizeof(lucam_prop_r)/sizeof(__u32), USB_DIR_IN);
}


/// write a lucam_param
static inline int lucam_set_param(plucam_device plucam, lucam_param* param)
{
   return lucam_reg32_rw(plucam, param->reg_index, (__u32*)&param->value, sizeof(lucam_prop_w)/sizeof(__u32), USB_DIR_OUT);
}


// write a lucam_param with more checking
static int lucam_set_param_flags_val(plucam_device plucam, plucam_param pparam, __u32 flags, __u32 val)
{
   int ret;
   __u32 ex=0;
   __u32 old_val =pparam->value.value;
   int clamp = 0;
   //Info(plucam, TRACE_REG, "lucam_set_param_flags_val, val=%d\n", val);
   if(!is_prop_supported(&pparam->value))
   {
      Info(plucam,TRACE_INFO,"***Setting unsupported prop\n");
      return -EINVAL;
   }
   if (
      //pparam->reg_index == LUCAM_BRIGHTNESS ||
      pparam->reg_index == LUCAM_FO_GAIN ||
      pparam->reg_index == LUCAM_FO_GAIN_RED ||
      pparam->reg_index == LUCAM_FO_GAIN_GREEN1 ||
      pparam->reg_index == LUCAM_FO_GAIN_GREEN2 ||
      pparam->reg_index == LUCAM_FO_GAIN_BLUE
      )
      clamp = 1;
   if (clamp)
      ex=CLAMP(val, pparam->value.min, pparam->value.max);
   else
      ex=val;
   pparam->value.flags=(pparam->value.flags & 0xffff0000) | (flags & 0x0000ffff);
   pparam->value.value=ex;
   ret=lucam_set_param(plucam, pparam);
   if(ret <0)
   {
      pparam->value.value=old_val;
      Err(plucam, "***set params failed %d \n", pparam->reg_index);
      return ret;
   }
   return ret;
}


/****************************************************************************
 *
 *  Send usb bulk message using the bulk_msg marcro. For those simple messages.
 *
 *
 ***************************************************************************/
static int lucam_bulk_msg(plucam_device plucam, int pipe, BYTE* buf, int len, BYTE dir)
{
   struct usb_device *udev;
   int ret, *actual_length;
   BYTE *transfer_buffer;
   udev = plucam->udev;

   transfer_buffer=kmalloc(len, GFP_KERNEL);
   if(!transfer_buffer)
   {
      ret=-ENOMEM;
      goto exiterror;
   }
   actual_length = kmalloc(sizeof(int), GFP_KERNEL);
   if(!actual_length)
   {
      kfree(transfer_buffer);
      ret=-ENOMEM;
      goto exiterror;
   }
#if 0
   int ep = epAddr & USB_ENDPOINT_NUMBER_MASK;
   int direction = epAddr & USB_ENDPOINT_DIR_MASK;
   if (direction == USB_DIR_IN)
      pipe= usb_rcvbulkpipe (udev, ep);
   else
      pipe= usb_sndbulkpipe (udev, ep);
#endif

   if(dir == USB_DIR_OUT)
   {
      memcpy(transfer_buffer,buf,len);
   }

   ret=usb_bulk_msg(udev, pipe, transfer_buffer, len, actual_length, USB_BULK_MSG_TIMEOUT);

   if(ret<0)
   {
      Err(plucam, "***lucam_bulk_msg: usb_bulk_msg failed(%d)\n",ret);
#if 0
      if (lucam_set_idle(udev) < 0)
         return -EINVAL;
#endif
   }
   if( ret == -EPIPE )
   {
      Info(plucam, TRACE_REG, "CLEAR_FEATURE request to remove STALL condition.\n");
      if(lucam_clear_halt(plucam, usb_pipeendpoint(pipe)))
      {
         Err(plucam, "***lucam_clear_halt failed\n");
      }
   }
   if(ret>=0)
   {
      ret=*actual_length;
      if(dir == USB_DIR_IN)
      {
         memcpy(buf,transfer_buffer,len);
      }
   }
   kfree(transfer_buffer);
   kfree(actual_length);
exiterror:
   return ret;
}

#if !USB3_ENDPOINT_CONFIGURATION
/****************************************************************************
 *
 *  Programming fpga through the fpga endpoint.
 *
 ***************************************************************************/
static int lucam_fpga_setup(plucam_device plucam)
{
   int time_out=10;
   unsigned short revId, productId;
   unsigned int data;
   unsigned long length=0,len=0;
   BYTE* buf=NULL;
   const unsigned char *ptr=NULL;
   struct usb_device *udev;
   int packet_size = 512;
   int ret;
   struct _FpgaConfigDescriptor2 *pFpgas=NULL;
   int fpgasCount=0;
   int i;
   
   Info(plucam, TRACE_PROBE, "lucam_fpga_setup() called \n");

   if(!plucam) return -EFAULT;
   udev = plucam -> udev;
   if(!udev) return -EFAULT;

   if (FpgasListCount == 0)
   {
      Info(plucam, TRACE_PROBE, "No FPGA to upload.\n");
      return 0;
   }
   
   productId = udev->descriptor.idProduct; 
   revId = udev->descriptor.bcdDevice;

#if !defined (__LITTLE_ENDIAN)
   /* Some host controller drivers on big endian systems do not swap the usb descriptors */
   if (udev->descriptor.idVendor == SWAP16(VID_HARDWARE_LUCAM) || udev->descriptor.idVendor == SWAP16(VID_HARDWARE_LUCAM2))
   {
      Info(plucam, TRACE_PROBE, "USB host controller driver did not swap the 16 bits fields of the USB descriptors\n");
      revId = SWAP16(revId);
      productId = SWAP16(productId);
   }
#endif

   for(i=0;i<FpgasListCount;i++)
   {
      if ((FpgasList[i].Pid == productId || FpgasList[i].Pid == 0xffff) && (FpgasList[i].Did == revId || FpgasList[i].Did == 0xffff))
      {
         pFpgas = FpgasList[i].FpgaDescriptors;
         fpgasCount = FpgasList[i].FpgaDescriptorsCount;
         break;
      }
   }

   if (i == FpgasListCount)
   {
      Err(plucam, "***This driver does not support version PID_%04X&DID_%04X of the camera\n", productId, revId);
      return -1;
   }

   if (pFpgas == NULL || fpgasCount == 0)
   {
      Info(plucam, TRACE_PROBE, "No FPGA to program\n");
      return 0;
   }
   
   if (plucam->fpga_pipe == 0)
   {
      Err(plucam, "***No FPGA pipe, but one was expected!!\n");
      return -1;
   }

   // 0.1 Verify that the fpga is not already programmed
   ret=lucam_reg32_rw(plucam, LUCAM_FPGA_MODE, &data, 1, USB_DIR_IN);
   if(ret <0)
   {
      Err(plucam, "***lucam_fpga_setup--failed to read the FPGA mode reg\n");
      return ret;
   }
   else if (data & 0x20)
   {
      Info(plucam, TRACE_INFO, "*fpga is alreadyprogrammed\n");
      return 0;
   }

   //0.5. Alloc some mem
   buf=kmalloc(packet_size, GFP_KERNEL);
   if(buf == NULL)
   {
      Err(plucam, "***lucam_fpga_setup--failed to allocate buffer for downloading\n");
      return -ENOMEM;
   }

   //1. set interface
   ret = lucam_select_alt_setting (plucam, FPGA_ALT_SETTING);
   if(ret <0)
   {
      Err(plucam, "***lucam_fpga_setup--set_interface failed:FPGA_ALT_SETTING \n");
      kfree(buf);
      return ret;
   }
   
   for(i=0;i<fpgasCount;i++)
   {
      //2.set FPGA to programming mode
      data=pFpgas[i].ProgramCode;
      ret=lucam_reg32_rw(plucam, LUCAM_FPGA_MODE, &data, 1, USB_DIR_OUT);
      if(ret <0)
      {
         Err(plucam, "***lucam_fpga_setup--set FPGA mode to programming failed\n");
         kfree(buf);
         return ret;
      }

      // Delay cause on some systems are too fast.
      mdelay(2);
   
      ptr = pFpgas[i].FpgaConfig;
      length = pFpgas[i].Size;
      while(length>0)
      {
         if (length>packet_size)
            len=packet_size;
         else
            len=length;
         memset(buf, 0, packet_size);
         memcpy(buf, ptr, len);
         ret=lucam_bulk_msg(plucam, plucam->fpga_pipe, buf, len, USB_DIR_OUT);
         if(ret<0) break;
         ptr=ptr+len;
         length-=len;
         mdelay(1);
      }
   
      if(ret<0)
      {
         Err(plucam, "***lucam_fpga_setup: failed to download the config files \n");
         kfree(buf);
         return ret;
      }
   
      //4. wait and check
      ret = -1;
      while(1)
      {
         mdelay(2);
         time_out--;
         if(time_out==0) break;
         ret=lucam_reg32_rw(plucam, LUCAM_FPGA_MODE, &data, 1, USB_DIR_IN);
         if(ret >0 && !(data & 0x40)) break;       //check bit 6
      }
   
      //check again
      ret=lucam_reg32_rw(plucam, LUCAM_FPGA_MODE, &data, 1, USB_DIR_IN);
      if(ret <0)
      {
         Err(plucam, "***lucam_fpga_setup--read FPGA mode failed-- check 6\n");
         kfree(buf);
         return ret;
      }
   
      if(data & 0x40)                               //check bit 6
      {
         ret = -1;
         kfree(buf);
         Err(plucam, "***lucam_fpga_setup--programming time out\n");
         return -1;
      }
   }
   
   kfree(buf);
   
   //5.set FPGA to normal mode
   data=0;
   ret=lucam_reg32_rw(plucam, LUCAM_FPGA_MODE, &data, 1, USB_DIR_OUT);

   if(ret <0)
   {
      Err(plucam, "***lucam_fpga_setup--set FPGA mode to normal failed\n");
   }
   //6. verify if it is OK
   ret=lucam_reg32_rw(plucam, LUCAM_FPGA_MODE, &data, 1, USB_DIR_IN);
   if(ret <0)
   {
      Err(plucam, "***lucam_fpga_setup--read FPGA mode failed-- verify\n");
      return ret;
   }
   if(data & 0x20)                               //check bit 5
   {
      ret = 1;
   }
   else
   {
      ret=-1;
      Err(plucam, "***lucam_fpga_setup--read FPGA mode failed-- still not programmed\n");
   }
   //7
   lucam_select_alt_setting (plucam, IDLE_ALT_SETTING);
   return ret;
}
#endif

/****************************************************************************
 *
 *  Bunch of trival functions for accessing Lucam registers.
 *
 ***************************************************************************/
static inline __u32 lucam_get_gamma(plucam_device plucam)
{
   if(is_prop_supported( &(plucam->cam_registers.gamma.value))==0)
      return 0xffff;
   return plucam->cam_registers.gamma.value.value;
}
static inline int lucam_set_gamma(plucam_device plucam, __u32 flags, __u32 ex)
{
   //PRINT_VALUE(plucam, "gamma", &plucam->cam_registers.gamma.value);
   return lucam_set_param_flags_val(plucam, &plucam->cam_registers.gamma, flags, ex);
}

static inline __u32 lucam_get_contrast(plucam_device plucam)
{
   if(is_prop_supported( &(plucam->cam_registers.contrast.value))==0)
      return 0xffff;
   return plucam->cam_registers.contrast.value.value;
}
static inline int lucam_set_contrast(plucam_device plucam, __u32 flags, __u32 ex)
{
   //PRINT_VALUE(plucam, "contrast", &plucam->cam_registers.contrast.value);
   return lucam_set_param_flags_val(plucam, &plucam->cam_registers.contrast, flags, ex);
}

static inline __u32 lucam_get_brightness(plucam_device plucam)
{
   if(is_prop_supported( &(plucam->cam_registers.brightness.value))==0)
      return 0xffff;
   return plucam->cam_registers.brightness.value.value;
}
static inline int lucam_set_brightness(plucam_device plucam, __u32 flags, __u32 ex)
{
   //PRINT_VALUE(plucam, "brightness", &plucam->cam_registers.brightness.value);
   return lucam_set_param_flags_val(plucam, &plucam->cam_registers.brightness, flags, ex);
}


static inline WORD lucam_get_width_max(plucam_device plucam)
{
   return LO_WORD(plucam->cam_registers.max_size.value);
}


static inline WORD  lucam_get_height_max(plucam_device plucam)
{
   return HI_WORD(plucam->cam_registers.max_size.value);
}


static inline WORD lucam_get_width_unit(plucam_device plucam)
{
   return LO_WORD(plucam->cam_registers.unit_size.value);
}


static inline WORD  lucam_get_height_unit(plucam_device plucam)
{
   return HI_WORD(plucam->cam_registers.unit_size.value);
}


static inline __u32 lucam_get_colorinq(plucam_device plucam)
{
   return plucam->cam_registers.color_inq.value;
}


static inline WORD lucam_fo_get_width(plucam_device plucam)
{
   return LO_WORD(plucam->cam_registers.fo_size.value);
}


static inline WORD lucam_fo_get_height(plucam_device plucam)
{
   return HI_WORD(plucam->cam_registers.fo_size.value);
}


static inline  WORD lucam_fo_get_posx(plucam_device plucam)
{
   return LO_WORD(plucam->cam_registers.fo_position.value);
}


static inline WORD lucam_fo_get_posy(plucam_device plucam)
{
   return HI_WORD(plucam->cam_registers.fo_position.value);
}


static inline  WORD lucam_fo_get_subsampling_w(plucam_device plucam)
{
   return LO_WORD(plucam->cam_registers.fo_subsampling.value);
}

static inline  WORD lucam_fo_get_subsampling_h(plucam_device plucam)
{
   return HI_WORD(plucam->cam_registers.fo_subsampling.value);
}

static inline  __u32 lucam_fo_get_subsampling_inq(plucam_device plucam)
{
   return plucam->cam_registers.fo_subsampling_inq.value;
}


static inline  __u32 lucam_fo_get_colorid(plucam_device plucam)
{
   return plucam->cam_registers.fo_color_id.value;
}


static inline  __u32 lucam_fo_get_tap_configuration(plucam_device plucam)
{
   return plucam->cam_registers.fo_tap_configuration.value;
}


static inline __u32 lucam_fo_get_exposure(plucam_device plucam)
{
   return plucam->cam_registers.fo_exposure.value.value;
}


static inline WORD lucam_st_get_width(plucam_device plucam)
{
   return LO_WORD(plucam->cam_registers.st_size.value);
}


static inline WORD lucam_st_get_height(plucam_device plucam)
{
   return HI_WORD(plucam->cam_registers.st_size.value);
}


static inline WORD lucam_st_get_posx(plucam_device plucam)
{
   return LO_WORD(plucam->cam_registers.st_position.value);
}


static inline WORD lucam_st_get_posy(plucam_device plucam)
{
   return HI_WORD(plucam->cam_registers.st_position.value);
}


static inline WORD lucam_st_get_subsampling_w(plucam_device plucam)
{
   return LO_WORD(plucam->cam_registers.st_subsampling.value);
}


static inline WORD lucam_st_get_subsampling_h(plucam_device plucam)
{
   return HI_WORD(plucam->cam_registers.st_subsampling.value);
}


static inline __u32 lucam_st_get_colorid(plucam_device plucam)
{
   return plucam->cam_registers.st_color_id.value;
}


static inline __u32 lucam_st_get_tap_configuration(plucam_device plucam)
{
   return plucam->cam_registers.st_tap_configuration.value;
}


static inline __u32 lucam_st_get_exposure(plucam_device plucam)
{
   return plucam->cam_registers.st_exposure.value.value;
}


static inline __u32 lucam_fo_get_kps(plucam_device plucam)
{
   return plucam->cam_registers.fo_kps.value;
}

#if 0
static inline WORD lucam_fo_get_kps_min(plucam_device plucam)
{
   return LO_WORD(plucam->cam_registers.fo_kps_minmax.value);
}


static inline WORD lucam_fo_get_kps_max(plucam_device plucam)
{
   return HI_WORD(plucam->cam_registers.fo_kps_minmax.value);
}
#endif

static inline __u32 lucam_fo_get_kps_cnt(plucam_device plucam)
{
   return plucam->cam_registers.fo_kps_cnt.value;
}


static void lucam_get_kps_array(plucam_device plucam)
{
   __u32 i,j, ret=0,val;
   unsigned long* buf;
   __u32  cnt;

   if (plucam->kpsarray)
   {
      kfree(plucam->kpsarray);
      plucam->kpsarray = NULL;
      plucam->kpsarraycnt = 0;
   }

   if (lucam_get_reg(plucam, &plucam->cam_registers.fo_kps_cnt) < 0)
   {
      // Failed to read the # of frame rates available
      return;
   }
   // this fn does nothing on the camera it just get the value earlier read above.
   cnt = lucam_fo_get_kps_cnt(plucam);
   cnt&=0xf;
   if(cnt == 0) return;

   buf= kmalloc(sizeof(unsigned long) * (cnt+1), GFP_KERNEL);
   if(!buf) return;                              //nomem

   if (plucam->specification >= 2 || (plucam->lucam_flags & LUCAM_FLAGS_KFREQ32))
   {
      for(i=0,j=0;i<(cnt); i++,j++)
      {
         ret = lucam_get_reg(plucam, &plucam->cam_registers.fo_kps_array);
         if(ret < 0)   break;
         val = plucam->cam_registers.fo_kps_array.value;
         if(val==0)
         {
            ret = -EINVAL;
            break;
         }
         buf[j]=val;
      }
   }
   else
   {
      for(i=0,j=0;i<(cnt+1)/2; i++,j=j+2)
      {
         ret = lucam_get_reg(plucam, &plucam->cam_registers.fo_kps_array);
         if(ret < 0)   break;
         val = plucam->cam_registers.fo_kps_array.value;
         if(val==0)
         {
            ret = -EINVAL;
            break;
         }
         buf[j]=LO_WORD(val);
         buf[j+1]=HI_WORD(val);
      }
   }
   if(ret < 0)
   {
      kfree(buf);
      return;
   }
   plucam->kpsarray = buf;
   plucam->kpsarraycnt = cnt;
   Info(plucam, TRACE_REG, "kps cnt = %d \n", cnt);
   for(i=0;i<cnt;i++)
   {
      Info(plucam, TRACE_REG, "-- %d -- \n", (int)buf[i]);
   }
   return;
}


static inline int lucam_fo_set_exposure(plucam_device plucam, __u32 flags, __u32 ex)
{
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.fo_exposure, flags, ex);
}


static inline int lucam_fo_set_gain(plucam_device plucam, __u32 flags, __u32 ex)
{
    //PRINT_VALUE(plucam, "gain", &plucam->cam_registers.fo_gain.value);
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.fo_gain, flags, ex);
}


static inline int lucam_fo_set_gain_red(plucam_device plucam, __u32 flags, __u32 ex)
{
    //PRINT_VALUE(plucam, "gain", &plucam->cam_registers.fo_gain_red.value);
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.fo_gain_red, flags, ex);
}


static inline int lucam_fo_set_gain_blue(plucam_device plucam, __u32 flags, __u32 ex)
{
    //PRINT_VALUE(plucam, "gain", &plucam->cam_registers.fo_gain_blue.value);
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.fo_gain_blue, flags, ex);
}


static inline int lucam_fo_set_gain_green1(plucam_device plucam, __u32 flags, __u32 ex)
{
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.fo_gain_green1, flags, ex);
}


static inline int lucam_fo_set_gain_green2(plucam_device plucam, __u32 flags, __u32 ex)
{
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.fo_gain_green2, flags, ex);
}


static inline int lucam_set_iris(plucam_device plucam, __u32 flags, __u32 ex)
{
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.iris, flags, ex);
}


static inline int lucam_set_focus(plucam_device plucam, __u32 flags, __u32 ex)
{
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.focus, flags, ex);
}


static inline int lucam_set_still_knee1_exposure(plucam_device plucam, __u32 flags, __u32 ex)
{
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.still_knee1_exposure, flags, ex);
}


static inline int lucam_set_still_knee2_exposure(plucam_device plucam, __u32 flags, __u32 ex)
{
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.still_knee2_exposure, flags, ex);
}


static inline int lucam_set_still_knee3_exposure(plucam_device plucam, __u32 flags, __u32 ex)
{
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.still_knee3_exposure, flags, ex);
}


static inline int lucam_set_video_knee(plucam_device plucam, __u32 flags, __u32 ex)
{
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.video_knee, flags, ex);
}

static inline int lucam_set_knee2_level(plucam_device plucam, __u32 flags, __u32 ex)
{
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.knee2_level, flags, ex);
}

static inline int lucam_set_timestamps(plucam_device plucam, __u32 flags, __u32 ex)
{
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.timestamps, flags, ex);
}

static inline int lucam_set_video_setting(plucam_device plucam, __u32 flags, __u32 ex)
{
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.video_settings, flags, ex);
}

static inline int lucam_set_bpc(plucam_device plucam, __u32 flags, __u32 ex)
{
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.bpc, flags, ex);
}


static inline int lucam_fo_set_kps(plucam_device plucam, __u32 ex)
{
    plucam->cam_registers.fo_kps.value =  ex;
    return lucam_set_reg(plucam, &plucam->cam_registers.fo_kps);
}


static int lucam_fo_set_pos(plucam_device plucam, WORD x, WORD y)
{
    int ret;
    __u32 old_val=plucam->cam_registers.fo_position.value;
    plucam->cam_registers.fo_position.value = words_to_u32(x,y);
    ret = lucam_set_reg(plucam, &(plucam->cam_registers.fo_position));
    if(ret < 0)
        plucam->cam_registers.fo_position.value=old_val;
    return ret;
}


static int lucam_fo_set_colorid(plucam_device plucam, __u32 cid)
{
   int ret;
   __u32 old_val = plucam->cam_registers.fo_color_id.value;
   plucam->cam_registers.fo_color_id.value=cid;
   ret = lucam_set_reg(plucam, &(plucam->cam_registers.fo_color_id));
   if (ret <0)
      plucam->cam_registers.fo_color_id.value = old_val;
   Info(plucam, TRACE_INFO, "%s: (was %d) %d -> %d\n", __func__, 
      old_val, cid, plucam->cam_registers.fo_color_id.value);
   return ret;
}


static int lucam_fo_set_tap_configuration(plucam_device plucam, __u32 tapConfig)
{
   int ret;
   __u32 old_val = plucam->cam_registers.fo_tap_configuration.value;
   plucam->cam_registers.fo_tap_configuration.value = tapConfig;
   ret = lucam_set_reg(plucam, &(plucam->cam_registers.fo_tap_configuration));
   if (ret <0)
      plucam->cam_registers.fo_tap_configuration.value = old_val;
   Info(plucam, TRACE_USB, "%s: (was %d) %d -> %d\n", __func__, 
      old_val, tapConfig, plucam->cam_registers.fo_tap_configuration.value);
   return ret;
}


static int lucam_fo_set_size(plucam_device plucam, WORD w, WORD h)
{
   int ret;
   __u32 old_val = plucam->cam_registers.fo_size.value;
   plucam->cam_registers.fo_size.value = words_to_u32(w,h);
   ret =  lucam_set_reg(plucam, &(plucam->cam_registers.fo_size));
   if(ret<0)   plucam->cam_registers.fo_size.value = old_val;
   return ret;
}


static int lucam_fo_set_subsampling(plucam_device plucam, WORD sw, WORD sh)
{
    int ret;
   __u32 old_val = plucam->cam_registers.fo_subsampling.value;
   plucam->cam_registers.fo_subsampling.value=words_to_u32(sw,sh);
   ret = lucam_set_reg(plucam, &(plucam->cam_registers.fo_subsampling));
   if (ret <0)
   {
      plucam->cam_registers.fo_subsampling.value = old_val;
   }
   return ret;
}


static inline int lucam_st_set_exposure(plucam_device plucam, __u32 flags, __u32 ex)
{
   return lucam_set_param_flags_val(plucam, &plucam->cam_registers.st_exposure, flags, ex);
   //plucam->cam_registers.st_exposure.value.value=ex;
   //return lucam_set_param(plucam, &plucam->cam_registers.st_exposure);
}


static inline int lucam_st_set_gain(plucam_device plucam, __u32 flags, __u32 ex)
{
   //PRINT_VALUE(plucam, "gain", &plucam->cam_registers.st_gain.value);
   return lucam_set_param_flags_val(plucam, &plucam->cam_registers.st_gain, flags, ex);
}


static inline int lucam_st_set_gain_red(plucam_device plucam, __u32 flags, __u32 ex)
{
   //PRINT_VALUE(plucam, "gain", &plucam->cam_registers.st_gain_red.value);
   return lucam_set_param_flags_val(plucam, &plucam->cam_registers.st_gain_red, flags, ex);
}


static inline int lucam_st_set_gain_blue(plucam_device plucam, __u32 flags, __u32 ex)
{
   //PRINT_VALUE(plucam, "gain", &plucam->cam_registers.st_gain_blue.value);
   return lucam_set_param_flags_val(plucam, &plucam->cam_registers.st_gain_blue, flags, ex);
}


static inline int lucam_st_set_gain_green1(plucam_device plucam, __u32 flags, __u32 ex)
{
   return lucam_set_param_flags_val(plucam, &plucam->cam_registers.st_gain_green1, flags, ex);
}


static inline int lucam_st_set_gain_green2(plucam_device plucam, __u32 flags, __u32 ex)
{
   return lucam_set_param_flags_val(plucam, &plucam->cam_registers.st_gain_green2, flags, ex);
}


static inline int lucam_st_set_strobe_delay(plucam_device plucam, __u32 flags, __u32 ex)
{
   return lucam_set_param_flags_val(plucam, &plucam->cam_registers.st_strobe_delay, flags, ex);
}

static inline int lucam_st_set_strobe_duration(plucam_device plucam, __u32 flags, __u32 ex)
{
   return lucam_set_param_flags_val(plucam, &plucam->cam_registers.st_strobe_duration, flags, ex);
}

static int lucam_st_set_pos(plucam_device plucam, WORD x, WORD y)
{
   int ret;
   __u32 old_val=plucam->cam_registers.st_position.value;
   plucam->cam_registers.st_position.value = words_to_u32(x,y);
   ret = lucam_set_reg(plucam, &(plucam->cam_registers.st_position));
   if(ret < 0)
      plucam->cam_registers.st_position.value=old_val;
   return ret;
}


static int lucam_st_set_size(plucam_device plucam, WORD w, WORD h)
{
   int ret;
   __u32 old_val = plucam->cam_registers.st_size.value;
   plucam->cam_registers.st_size.value =words_to_u32(w,h);
   ret =  lucam_set_reg(plucam, &(plucam->cam_registers.st_size));
   if(ret<0)   plucam->cam_registers.st_size.value = old_val;
   return ret;
}


static int lucam_st_set_subsampling(plucam_device plucam, WORD sw, WORD sh)
{
   int ret;
   __u32 old_val = plucam->cam_registers.st_subsampling.value;
   plucam->cam_registers.st_subsampling.value=words_to_u32(sw,sh);
   ret = lucam_set_reg(plucam, &(plucam->cam_registers.st_subsampling));
   if (ret <0)
   {
      plucam->cam_registers.st_subsampling.value = old_val;
   }
   return ret;
}


static int lucam_st_set_colorid(plucam_device plucam, __u32 cid)
{
   int ret;
   __u32 old_val = plucam->cam_registers.st_color_id.value;
   plucam->cam_registers.st_color_id.value=cid;
   ret = lucam_set_reg(plucam, &(plucam->cam_registers.st_color_id));
   if (ret <0)
      plucam->cam_registers.st_color_id.value = old_val;
   return ret;
}


static int lucam_st_set_tap_configuration(plucam_device plucam, __u32 tapConfig)
{
   int ret;
   __u32 old_val = plucam->cam_registers.st_tap_configuration.value;
   plucam->cam_registers.st_tap_configuration.value = tapConfig;
   ret = lucam_set_reg(plucam, &(plucam->cam_registers.st_tap_configuration));
   if (ret <0)
      plucam->cam_registers.st_tap_configuration.value = old_val;
   return ret;
}


static inline int lucam_set_snapshot_count(plucam_device plucam, __u32 flags, __u32 ex)
{
    //PRINT_VALUE(plucam, "snapshot_count", &plucam->cam_registers.snapshot_count.value);
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.snapshot_count, flags, ex);
}

static inline int lucam_set_snapshot_setting(plucam_device plucam, __u32 flags, __u32 ex)
{
    //PRINT_VALUE(plucam, "snapshot_setting", &plucam->cam_registers.snapshot_setting.value);
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.snapshot_setting, flags, ex);
}

static inline int lucam_set_st_shutter_type(plucam_device plucam, __u32 flags, __u32 ex)
{
    //PRINT_VALUE(plucam, "st_shutter_type", &plucam->cam_registers.st_shutter_type.value);
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.st_shutter_type, flags, ex);
}

static inline int lucam_set_trigger_pin(plucam_device plucam, __u32 flags, __u32 ex)
{
    //PRINT_VALUE(plucam, "trigger pin", &plucam->cam_registers.trigger_pin.value);
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.trigger_pin, flags, ex);
}

static inline int lucam_set_strobe_pin(plucam_device plucam, __u32 flags, __u32 ex)
{
    //PRINT_VALUE(plucam, "strobe pin", &plucam->cam_registers.strobe_pin.value);
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.strobe_pin, flags, ex);
}

static inline int lucam_set_auto_exp_target(plucam_device plucam, __u32 flags, __u32 ex)
{
    return lucam_set_param_flags_val(plucam, &plucam->cam_registers.auto_exp_target, flags, ex);
}
//----------------------------------------------------------------------
//
static int lucam_enable_video(plucam_device plucam, BOOL enabled)
{
   int rt;
   if (enabled)
   {
      Info(plucam, TRACE_INFO, "lucam_enable_video() called \n");
   }
   else
   {
      Info(plucam, TRACE_INFO, "lucam_disable_video() called \n");
   }

   if(plucam->camera_mode != LUCAM_VIDEO_MODE)
   {
      Err(plucam, "*** lucam_enable_video: Not in video mode\n");
      return -1;
   }
   if(enabled==TRUE)
      plucam->cam_registers.video_en.value=0x80;
   else
      plucam->cam_registers.video_en.value=0x40;
   rt = lucam_set_reg(plucam, &(plucam->cam_registers.video_en));
   if (rt >= 0 && !enabled)
   {
      udelay(10);
      plucam->cam_registers.video_en.value=0x00;
      rt = lucam_set_reg(plucam, &(plucam->cam_registers.video_en));
   }
   return rt;
}

//---------------------------------------------------------------------------
//
static int lucam_enable_still(plucam_device plucam, BOOL enabled, BOOL software_trigger_mode)
{
   int rt;
   if (enabled)
   {
      Info(plucam, TRACE_INFO, "lucam_enable_still() called for %sware trigger mode\n", software_trigger_mode ? "soft": "hard");
   }
   else
   {
      Info(plucam, TRACE_INFO, "lucam_disable_still() called\n");
   }

   if (enabled)
   {
      if (software_trigger_mode)
      {
         if(plucam->camera_mode != LUCAM_STILL_SW_TRIGGER_MODE) return -1;
      }
      else
      {
         if(plucam->camera_mode != LUCAM_STILL_HW_TRIGGER_MODE) return -1;
      }

      if(software_trigger_mode)
         plucam->cam_registers.trigger_ctl.value=0x04;
      else
         plucam->cam_registers.trigger_ctl.value=0x05;
   }
   else
   {
      plucam->cam_registers.trigger_ctl.value=0x0;
   }

   rt = lucam_set_reg(plucam, &(plucam->cam_registers.trigger_ctl));
#if USB3_ENDPOINT_CONFIGURATION
   if (rt && enabled)
   {
	  __u32 dummy;
	  rt = lucam_reg32_rw(plucam, LUCAM_TRIGGER_CTRL, &dummy, 1, USB_DIR_IN);
   }
#endif
   return rt;
}

//---------------------------------------------------------------------
//
static int lucam_software_trigger(plucam_device plucam)
{
   struct timeval trigger_time;

   do_gettimeofday(&trigger_time);
   Info(plucam, TRACE_INFO, "lucam_software_trigger called @ %lu.%ld\n",
      trigger_time.tv_sec, trigger_time.tv_usec);

   if((plucam->camera_mode != LUCAM_STILL_SW_TRIGGER_MODE) &&
        (plucam->camera_mode != LUCAM_STILL_HW_TRIGGER_MODE || plucam->specification == 0)
        )
   {
      return -EINVAL;
   }

   if (plucam->current_stream_state != LUCAM_STREAM_STATE_RUN)
   {
      return -EINVAL;
   }

   if (plucam->specification == 0)
   {
      plucam->cam_registers.trigger_ctl.value=0x03;
   }
   else
   {
      if (plucam->camera_mode == LUCAM_STILL_SW_TRIGGER_MODE)
         plucam->cam_registers.trigger_ctl.value=0x06;
      else
         plucam->cam_registers.trigger_ctl.value=0x07;
   }
   return lucam_set_reg(plucam, &(plucam->cam_registers.trigger_ctl));
}

//---------------------------------------------------------------------------
//
static int lucam_init_sensor (plucam_device plucam)
{
   plucam->cam_registers.sensor_init.value=0x01;
   return lucam_set_reg(plucam, &(plucam->cam_registers.sensor_init));
}

//----------------------------------------------------------------------
//
static int pixelformat_from_colorid(plucam_device plucam, __u32 colorid, __u32 *pixelformat, int *pBitsPerPixel)
{
   int ret = 0;
   int bpp = 0;

   switch (colorid & 0xff)
   {
   case LUCAM_COLOR_MONO8 :
      *pixelformat = V4L2_PIX_FMT_GREY;
      bpp = 8;
      break;

   case LUCAM_COLOR_MONO16 :
      *pixelformat = V4L2_PIX_FMT_Y61;
      bpp = 16;
      break;
   case LUCAM_COLOR_MONO61 :
      *pixelformat = V4L2_PIX_FMT_Y16;
      bpp = 16;
      break;

   case LUCAM_COLOR_BAYER_RGGB :
      *pixelformat = V4L2_PIX_FMT_SRGGB8;
      bpp = 8;
      break;
   case LUCAM_COLOR_BAYER_GRBG :
      *pixelformat = V4L2_PIX_FMT_SGRBG8;
      bpp = 8;
      break;
   case LUCAM_COLOR_BAYER_GBRG :
      *pixelformat = V4L2_PIX_FMT_SGBRG8;
      bpp = 8;
      break;
   case LUCAM_COLOR_BAYER_BGGR :
      *pixelformat = V4L2_PIX_FMT_SBGGR8;
      bpp = 8;
      break;

   case LUCAM_COLOR_BAYER16_RGGB :
      *pixelformat = V4L2_PIX_FMT_SRGGB61;
      bpp = 16;
      break;
   case LUCAM_COLOR_BAYER16_GRBG :
      *pixelformat = V4L2_PIX_FMT_SGRBG61;
      bpp = 16;
      break;
   case LUCAM_COLOR_BAYER16_GBRG :
      *pixelformat = V4L2_PIX_FMT_SGBRG61;
      bpp = 16;
      break;
   case LUCAM_COLOR_BAYER16_BGGR :
      *pixelformat = V4L2_PIX_FMT_SBGGR61;
      bpp = 16;
      break;

   case LUCAM_COLOR_BAYER61_RGGB :
      *pixelformat = V4L2_PIX_FMT_SRGGB16;
      bpp = 16;
      break;
   case LUCAM_COLOR_BAYER61_GRBG :
      *pixelformat = V4L2_PIX_FMT_SGRBG16;
      bpp = 16;
      break;
   case LUCAM_COLOR_BAYER61_GBRG :
      *pixelformat = V4L2_PIX_FMT_SGBRG16;
      bpp = 16;
      break;
   case LUCAM_COLOR_BAYER61_BGGR :
      *pixelformat = V4L2_PIX_FMT_SBGGR16;
      bpp = 16;
      break;

   case LUCAM_COLOR_XENA_CYYM :
   case LUCAM_COLOR_XENA_YCMY :
   case LUCAM_COLOR_XENA_YMCY :
   case LUCAM_COLOR_XENA_MYYC :
      ret = -EINVAL;
      Err(plucam, "***colorid %d does not map into v4l2 Fourcc\n", colorid);
      *pixelformat = V4L2_PIX_FMT_SBGGR8;
      break;

   case LUCAM_COLOR_XENA16_CYYM :
   case LUCAM_COLOR_XENA16_YCMY :
   case LUCAM_COLOR_XENA16_YMCY :
   case LUCAM_COLOR_XENA16_MYYC :

   case LUCAM_COLOR_XENA61_CYYM :
   case LUCAM_COLOR_XENA61_YCMY :
   case LUCAM_COLOR_XENA61_YMCY :
   case LUCAM_COLOR_XENA61_MYYC :
      ret = -EINVAL;
      Err(plucam, "***colorid %d does not map into v4l2 Fourcc\n", colorid);
      *pixelformat = V4L2_PIX_FMT_SBGGR16;
      break;
   default:
      ret = -EINVAL;
      Err(plucam, "***colorid %d does not map into v4l2 Fourcc\n", colorid);
      *pixelformat = V4L2_PIX_FMT_SBGGR16;
   }

   Info(plucam, TRACE_INFO, "%s: %u -> 0x%08x\n", __func__, colorid, *pixelformat);
   if (pBitsPerPixel) *pBitsPerPixel = bpp;
   return ret;
}

//--------------------------------------------------------------------------
//
static int colorid_from_pixelformat(plucam_device plucam, __u32 pixelformat, int *out_bpp, __u32 *colorid)
{
    int ret = 0;

    switch (pixelformat) {
#if ALLOW_RGB24_OUTPUT
    case V4L2_PIX_FMT_BGR24:
        *out_bpp = 24;
        *colorid = LUCAM_COLOR_BAYER_RGGB;
        break;
    case V4L2_PIX_FMT_RGB24:
        *out_bpp = 24;
        *colorid = LUCAM_COLOR_BAYER_BGGR;
        break;
#endif

    case V4L2_PIX_FMT_Y16:
        *out_bpp = 16;
        *colorid = LUCAM_COLOR_MONO61;
        break;
    case V4L2_PIX_FMT_Y61:
        *out_bpp = 16;
        *colorid = LUCAM_COLOR_MONO16;
        break;
    case V4L2_PIX_FMT_GREY:
        *out_bpp = 8;
        *colorid = LUCAM_COLOR_MONO8;
        break;


    case V4L2_PIX_FMT_SBGGR8:
        *out_bpp = 8;
        *colorid = LUCAM_COLOR_BAYER_BGGR;
        break;
    case V4L2_PIX_FMT_SRGGB8:
        *out_bpp = 8;
        *colorid = LUCAM_COLOR_BAYER_RGGB;
        break;
    case V4L2_PIX_FMT_SGRBG8 :
        *out_bpp = 8;
        *colorid = LUCAM_COLOR_BAYER_GRBG;
        break;
    case V4L2_PIX_FMT_SGBRG8 :
        *out_bpp = 8;
        *colorid = LUCAM_COLOR_BAYER_GBRG;
        break;

    case V4L2_PIX_FMT_SBGGR16:
        *out_bpp = 16;
        *colorid = LUCAM_COLOR_BAYER61_BGGR;
        break;
    case V4L2_PIX_FMT_SRGGB16:
        *out_bpp = 16;
        *colorid = LUCAM_COLOR_BAYER61_RGGB;
        break;
   case V4L2_PIX_FMT_SGRBG16:
        *out_bpp = 16;
        *colorid = LUCAM_COLOR_BAYER61_GRBG;
        break;
    case V4L2_PIX_FMT_SGBRG16:
        *out_bpp = 16;
        *colorid = LUCAM_COLOR_BAYER61_GBRG;
        break;

    case V4L2_PIX_FMT_SBGGR61:
        *out_bpp = 16;
        *colorid = LUCAM_COLOR_BAYER16_BGGR;
        break;
    case V4L2_PIX_FMT_SRGGB61:
        *out_bpp = 16;
        *colorid = LUCAM_COLOR_BAYER16_RGGB;
        break;
    case V4L2_PIX_FMT_SGRBG61:
        *out_bpp = 16;
        *colorid = LUCAM_COLOR_BAYER16_GRBG;
        break;
    case V4L2_PIX_FMT_SGBRG61:
        *out_bpp = 16;
        *colorid = LUCAM_COLOR_BAYER16_GBRG;
        break;

    default:
        Err(plucam, "Unhandled pixelformat: 0x%08x\n", pixelformat);
        *out_bpp = 8;
        *colorid = LUCAM_COLOR_MONO8;
        ret = -1;
        break;
    }

    return ret;
}

/*************************************************************************
 *
 ************************************************************************/
static void lucam_fill_v4l2_crop (plucam_device dev, struct v4l2_crop *a)
{
    if(dev->camera_mode == LUCAM_VIDEO_MODE) {
        a->c = (struct v4l2_rect){lucam_fo_get_posx(dev), lucam_fo_get_posy(dev),
                      lucam_fo_get_width(dev), lucam_fo_get_height(dev)};
    } else {
        a->c = (struct v4l2_rect){lucam_st_get_posx(dev), lucam_st_get_posy(dev),
                      lucam_st_get_width(dev), lucam_fo_get_height(dev)};
    }
}

/*************************************************************************
 * NOTE: there can be a discrepency between the live colorid and the
 * device model's pixel format. This happens for the LUCAM_* colorids
 * specified from userland that have no direct v4l2 Fourcc code yet. 
 *************************************************************************/
static void lucam_fill_v4l2_format (plucam_device dev, struct v4l2_format *f)
{
   int out_bpp;

   if(dev->camera_mode == LUCAM_VIDEO_MODE) {
      f->fmt.pix.width = lucam_fo_get_width(dev) / (lucam_fo_get_subsampling_w(dev) & 0xff);
      f->fmt.pix.height = lucam_fo_get_height(dev) / (lucam_fo_get_subsampling_h(dev) & 0xff);
      f->fmt.pix.priv = lucam_fo_get_colorid(dev);
   } else {
      f->fmt.pix.width = lucam_st_get_width(dev) / (lucam_st_get_subsampling_w(dev) & 0xff);
      f->fmt.pix.height = lucam_st_get_height(dev) / (lucam_st_get_subsampling_h(dev) & 0xff);
      f->fmt.pix.priv = lucam_st_get_colorid(dev);
   }
   f->fmt.pix.field        = V4L2_FIELD_NONE;

   dev->outwidth = f->fmt.pix.width;
   dev->outheight = f->fmt.pix.height;

   //f->fmt.pix.pixelformat  = dev->pixelformat;
   //colorid_from_pixelformat(dev, dev->pixelformat, &out_bpp, &f->fmt.pix.priv);
   pixelformat_from_colorid(dev, f->fmt.pix.priv, &f->fmt.pix.pixelformat, &out_bpp);

   Info(dev, TRACE_V4L2, "lucam_fill_v4l2_format: outw:%d, outh:%d, outbpp:%d\n", dev->outwidth, dev->outheight, out_bpp);

   dev->pixelformat = f->fmt.pix.pixelformat;
   f->fmt.pix.sizeimage = f->fmt.pix.width * f->fmt.pix.height * out_bpp / 8;
   f->fmt.pix.bytesperline = f->fmt.pix.width * out_bpp / 8;
}

//-----------------------------------------------------------------------
unsigned long lucam_get_framesizemax(plucam_device plucam)
{
   int bpp;
   __u32 pixFormat;
 
   pixelformat_from_colorid(plucam, lucam_get_colorinq(plucam) & 0xff, &pixFormat, &bpp);

   return lucam_get_width_max(plucam)*lucam_get_height_max(plucam)*bpp/8;
}

/****************************************************************************
 *
 *  Calculate the frame size according to the pixel format, crop_width, crop_height,
 *  legal subsampling values ...
 *  So we can know how much memory we need for a frame and/or image.
 *
 *  the v4l2_format *pf may be adjusted to fit within the crop subwindow with integral subsampling
 *  the v4l2_crop region is immutable.
 *
 *  TODO: move the imagesize_max calculation in here too.
 *  TODO: make the return value a __must_check
 ***************************************************************************/
static int lucam_try_framesize(plucam_device plucam, struct v4l2_format *pf, const struct v4l2_crop * const pc,
                   __u32 *out_colorid, size_t *out_framesize, size_t *out_bitcount,
                   size_t *out_sub_w, size_t* out_sub_h)
{
    int src_bits_per_pixel=8, dst_bits_per_pixel=8;
    __u32 colorid;
    __u32 sub_w, sub_h;
    size_t frame_size;

    // enlargement is not supported, so limit the format's width and height
    if (pc->c.width < pf->fmt.pix.width) pf->fmt.pix.width = pc->c.width;
    if (pc->c.height < pf->fmt.pix.height) pf->fmt.pix.height = pc->c.height;

    // output width and height format smaller than crop size happens only by subsampling
    sub_w = CLAMP(pc->c.width / pf->fmt.pix.width, 1, 0xff);
    sub_h = CLAMP(pc->c.height / pf->fmt.pix.height, 1, 0xff);

    // adapt the output image size to the subsampling capabilities
    pf->fmt.pix.width = pc->c.width / sub_w;
    pf->fmt.pix.height = pc->c.height / sub_h;

    if (colorid_from_pixelformat(plucam, pf->fmt.pix.pixelformat, &dst_bits_per_pixel, &colorid)) {
        return -EINVAL;
    }

    switch(colorid)
    {
   case LUCAM_COLOR_MONO8:
        src_bits_per_pixel=8;
        break;
   case LUCAM_COLOR_MONO16:
   case LUCAM_COLOR_MONO61:
        src_bits_per_pixel=16;
        break;
   case LUCAM_COLOR_BAYER_RGGB:
   case LUCAM_COLOR_BAYER_GRBG:
   case LUCAM_COLOR_BAYER_GBRG:
   case LUCAM_COLOR_BAYER_BGGR:
   case LUCAM_COLOR_XENA_CYYM:
   case LUCAM_COLOR_XENA_YCMY:
   case LUCAM_COLOR_XENA_YMCY:
   case LUCAM_COLOR_XENA_MYYC:
        src_bits_per_pixel=8;
        break;
   case LUCAM_COLOR_BAYER16_RGGB:
   case LUCAM_COLOR_BAYER16_GRBG:
   case LUCAM_COLOR_BAYER16_GBRG:
   case LUCAM_COLOR_BAYER16_BGGR:
   case LUCAM_COLOR_XENA16_CYYM:
   case LUCAM_COLOR_XENA16_YCMY:
   case LUCAM_COLOR_XENA16_YMCY:
   case LUCAM_COLOR_XENA16_MYYC:
   case LUCAM_COLOR_BAYER61_RGGB:
   case LUCAM_COLOR_BAYER61_GRBG:
   case LUCAM_COLOR_BAYER61_GBRG:
   case LUCAM_COLOR_BAYER61_BGGR:
   case LUCAM_COLOR_XENA61_CYYM:
   case LUCAM_COLOR_XENA61_YCMY:
   case LUCAM_COLOR_XENA61_YMCY:
   case LUCAM_COLOR_XENA61_MYYC:
        src_bits_per_pixel=16;
        break;
    }

    pf->fmt.pix.bytesperline = pf->fmt.pix.width * dst_bits_per_pixel / 8;
    pf->fmt.pix.sizeimage = pf->fmt.pix.bytesperline * pf->fmt.pix.height;
    frame_size = pf->fmt.pix.width * src_bits_per_pixel * pf->fmt.pix.height / 8;

    if (out_colorid)
        *out_colorid = colorid;
    if (out_framesize)
        *out_framesize = frame_size;
    if (out_bitcount)
        *out_bitcount = src_bits_per_pixel;
    if (out_sub_w)
        *out_sub_w = sub_w;
    if (out_sub_h)
        *out_sub_h = sub_h;

    Info(plucam, TRACE_INFO, "lucam_try_framesize: colorid = %u(%u) crop(%d/%dx%d/%d) Frame(%dx%d)->%d, image_size=%d\n",
         colorid,
         pf->fmt.pix.priv,
         pc->c.width, sub_w, pc->c.height, sub_h,
        (int)pf->fmt.pix.width, (int)pf->fmt.pix.height, (int)frame_size,
         pf->fmt.pix.sizeimage);

    return 0;
}

/****************************************************************************
 *
 *  Calculate the frame size according to the pixel format, width, height, ...
 *  So we can konw how much memory we need for a frame and/or image.
 *
 *  NOTE plucam->pixelformat will be set consistent to the current
 *  camera_mode's colorid.
 ***************************************************************************/
static void lucam_set_framesize(plucam_device plucam)
{
   struct v4l2_format shadow;
   //struct v4l2_crop shadow_crop;
   //__u32 colorid, old_colorid;
   //size_t image_bpp;
   //size_t sub_w, sub_h;

   if(plucam == NULL) return;

   Info(plucam, TRACE_INFO, "lucam_set_framesize\n");

   lucam_fill_v4l2_format(plucam, &shadow);
#if 0
   lucam_fill_v4l2_crop(plucam, &shadow_crop);

   colorid_from_pixelformat(plucam, plucam->pixelformat, &image_bpp, &old_colorid);

   lucam_try_framesize(plucam, &shadow, &shadow_crop,
                &colorid, &plucam->framesize, &plucam->imagesize,
                &plucam->bitcount, &sub_w, &sub_h);
   /* do nothing with the result, since these values were supposed to be tested in advance */

   plucam->pixelformat = shadow.fmt.pix.pixelformat;

   if (colorid != old_colorid)
   {
      Err(plucam, "%s: new colorid does not match old colorid; %u vs. %u\n",
             __func__, colorid, old_colorid);
   }

   Info(plucam, TRACE_INFO, "lucam_set_framesize: Frame size = %u, image_size = %u, colorid = %u\n",
         plucam->framesize, plucam->imagesize, colorid);
#else
   plucam->framesize = shadow.fmt.pix.sizeimage;
   Info(plucam, TRACE_INFO, "lucam_set_framesize: size is %d\n", (int)plucam->framesize);
   //plucam->imagesize = shadow.fmt.pix.imagesize;
#endif
}


/****************************************************************************
 *
 *  Manipulate the frame buf list.
 *  This fn must not be called while current_stream_state == running
 *  but may be called while the wanted_stream_state == running.
 *
 *  This function is only called via the change_state task.  What
 *  locking is needed?
 ***************************************************************************/
static void lucam_reset_framebuf_queues(struct _lucam_device *plucam)
{
   plucam_image_buf pimage_buf;
   int processed_one;

   Info(plucam, TRACE_USB | TRACE_BULK, "lucam_reset_framebuf_queues\n");

   processed_one = 0;

#if !ENHANCED_BUFFERING
   plucam->blank_image_buf.offset = 0;
   if (plucam->current_image_buf == &plucam->blank_image_buf) 
   {
      plucam->current_image_buf = NULL;
   }
#endif
   if (plucam->current_image_buf) 
   {
      plucam->current_image_buf->offset = 0;
   }
   if (plucam->camera_mode != LUCAM_VIDEO_MODE) 
   {
      // We must return the error frame and the current frame with errors.

      pimage_buf = plucam->error_image_buf;
#if ENHANCED_BUFFERING
      if (pimage_buf)
#else
      if (pimage_buf && pimage_buf != &plucam->blank_image_buf)
#endif
      {
         pimage_buf->status = -ENOENT;
         pimage_buf->error_code = -ENOENT;
         pimage_buf->vb.state = VIDEOBUF_ERROR;
         Info(plucam, TRACE_INFO | TRACE_BULK, " lucam_reset_framebuf_queues: Waking up for error image buf#%d\n",
            pimage_buf->vb.i);
         wake_up_all(&pimage_buf->vb.done);
         processed_one = 1;
      }
      plucam->error_image_buf = NULL;

      pimage_buf = plucam->current_image_buf;
#if ENHANCED_BUFFERING
      if (pimage_buf)
#else
      if (pimage_buf && pimage_buf != &plucam->blank_image_buf)
#endif
      {
         pimage_buf->status = -ENOENT;
         pimage_buf->error_code = -ENOENT;
         pimage_buf->vb.state = VIDEOBUF_ERROR;
         Info(plucam, TRACE_INFO | TRACE_BULK, " lucam_reset_framebuf_queues: Waking up for current image buf#%d\n",
            pimage_buf->vb.i);
         wake_up_all(&pimage_buf->vb.done);

         processed_one = 1;
      }
      plucam->current_image_buf = NULL;
   }
   else if (plucam->error_image_buf)
   {
      pimage_buf = plucam->current_image_buf;
#if ENHANCED_BUFFERING
      if (pimage_buf)
#else
      if (pimage_buf && pimage_buf != &plucam->blank_image_buf)
#endif
      {
         unsigned long flags;
           /* Enqueue back the current frame buf at the tail of the
            * empty queue */
           spin_lock_irqsave(&plucam->slock, flags);
           //list_del_init(&plucam->current_image_buf->vb.queue);
           list_add(&pimage_buf->vb.queue, &plucam->active_frames);
         pimage_buf->vb.state = VIDEOBUF_QUEUED;
           spin_unlock_irqrestore(&plucam->slock, flags);

         plucam->current_image_buf = NULL;
      }
      plucam->current_image_buf = plucam->error_image_buf;
      plucam->current_image_buf->offset = 0;
      plucam->error_image_buf = NULL;
   }

   if (processed_one)
   {
      /* originaly we would wake up some reading queue here, but this is already done */
   }
}


/********************************************************************************
 * This function is called after the wanted_stream_state is moved from RUN to NOT RUN
 ******************************************************************************/
static void lucam_flush_image_buf_q(plucam_device plucam)
{
   plucam_image_buf pimage_buf;
   unsigned long flags;

   Info(plucam, TRACE_INFO | TRACE_BULK, "lucam_flush_image_buf_q\n");

   //if (down_interruptible(&plucam->imagebuf_mutex)) return;

   pimage_buf = plucam->error_image_buf;
#if ENHANCED_BUFFERING
   if (pimage_buf)
#else
   if (pimage_buf && pimage_buf != &plucam->blank_image_buf)
#endif
   {
      pimage_buf->status = -ENOENT;
      pimage_buf->error_code = -ENOENT;
      pimage_buf->vb.state = VIDEOBUF_ERROR;
      Info(plucam, TRACE_INFO | TRACE_BULK, " lucam_flush_image_buf_: Waking up for error image buf#%d\n",
         pimage_buf->vb.i);
      wake_up_all(&pimage_buf->vb.done);
   }
   plucam->error_image_buf = NULL;

   pimage_buf = plucam->current_image_buf;
#if ENHANCED_BUFFERING
   if (pimage_buf)
#else
   if (pimage_buf && pimage_buf != &plucam->blank_image_buf)
#endif
   {
      pimage_buf->status = -ENOENT;
      pimage_buf->error_code = -ENOENT;
      pimage_buf->vb.state = VIDEOBUF_ERROR;
      Info(plucam, TRACE_INFO | TRACE_BULK, " lucam_flush_image_buf_: Waking up for current image buf#%d\n",
         pimage_buf->vb.i);
      wake_up_all(&pimage_buf->vb.done);
   }
   plucam->current_image_buf = NULL;

   spin_lock_irqsave(&plucam->slock, flags);
   while (!list_empty(&plucam->active_frames)) 
   {
      pimage_buf = list_entry(plucam->active_frames.next,
          struct _image_buf, vb.queue);

      list_del_init(&pimage_buf->vb.queue);
      pimage_buf->status = -ENOENT;
      pimage_buf->error_code = -ENOENT;
      Info(plucam, TRACE_INFO | TRACE_BULK, "lucam_flush_image_buf_: Waking up for queued image buf#%d\n",
         pimage_buf->vb.i);
      pimage_buf->vb.state = VIDEOBUF_ERROR;
      wake_up_all(&pimage_buf->vb.done);
   }
   spin_unlock_irqrestore(&plucam->slock, flags);
}

/*****************************************************************************
******************************************************************************/
static int lucam_reset_sensor(plucam_device plucam)
{
   int rt;
   lucam_registers* registers = &plucam->cam_registers;

   rt = lucam_init_sensor(plucam);
   if (rt < 0)
   {
      Err(plucam, "***lucam_init_sensor() failed\n");
      return rt;
   }

#ifdef DRIVER_INITRESET
   rt = CustomInitReset(plucam);
   if (rt < 0)
   {
      Err(plucam, "***lucam_init_sensor(): CustomInitReset failed\n");
      return rt;
   }
#endif
   plucam->camera_mode = LUCAM_VIDEO_MODE;

   lucam_get_reg(plucam, &registers-> fo_color_id);
   lucam_get_reg(plucam, &registers-> fo_tap_configuration);
   if ((registers-> fo_color_id.value & 0x08000000) && (registers-> fo_tap_configuration.value == 0))
   {
      registers-> fo_tap_configuration.value = 1;
   }
   lucam_get_reg(plucam, &registers-> fo_position);
   lucam_get_reg(plucam, &registers-> fo_size);
   lucam_get_reg(plucam, &registers-> fo_subsampling);
   if (((LO_WORD(registers->fo_subsampling.value) & 0xff)==0) || ((HI_WORD(registers->fo_subsampling.value) & 0xff)==0))
   {
      Err(plucam, "***lucam_init_sensor():read back invalid subsampling\n");
      registers->fo_subsampling.value = 0x00010001;
   }
   if (((LO_WORD(registers->fo_size.value))==0) || ((HI_WORD(registers->fo_size.value))==0))
   {
      Err(plucam, "***lucam_init_sensor():read back invalid size\n");
      registers->fo_size.value = registers->max_size.value;
   }

   if (is_prop_supported(&registers->st_exposure.value))
   {
      lucam_get_param(plucam, &registers-> fo_exposure);
      memcpy(&registers-> st_exposure.value, &registers-> fo_exposure.value,sizeof(lucam_prop));
      lucam_set_param(plucam, &registers->st_exposure);
   }
   if (is_prop_supported(&registers->st_gain.value))
   {
      lucam_get_param(plucam, &registers-> fo_gain);
      memcpy(&registers-> st_gain.value,&registers-> fo_gain.value,sizeof(lucam_prop));
      lucam_set_param(plucam, &registers->st_gain);
   }
   if (is_prop_supported(&registers->st_gain_blue.value))
   {
      lucam_get_param(plucam, &registers-> fo_gain_blue);
      memcpy(&registers-> st_gain_blue.value,&registers-> fo_gain_blue.value,sizeof(lucam_prop));
      lucam_set_param(plucam, &registers->st_gain_blue);
   }
   if (is_prop_supported(&registers->st_gain_red.value))
   {
      lucam_get_param(plucam, &registers-> fo_gain_red);
      memcpy(&registers-> st_gain_red.value,&registers-> fo_gain_red.value,sizeof(lucam_prop));
      lucam_set_param(plucam, &registers->st_gain_red);
   }
   if (is_prop_supported(&registers->st_gain_green1.value))
   {
      lucam_get_param(plucam, &registers-> fo_gain_green1);
      memcpy(&registers-> st_gain_green1.value,&registers->fo_gain_green1.value,sizeof(lucam_prop));
      lucam_set_param(plucam, &registers->st_gain_green1);
   }
   if (is_prop_supported(&registers->st_gain_green2.value))
   {
      lucam_get_param(plucam, &registers-> fo_gain_green2);
      memcpy(&registers-> st_gain_green2.value,&registers-> fo_gain_green2.value,sizeof(lucam_prop));
      lucam_set_param(plucam, &registers->st_gain_green2);
   }
   return 0;
}

/****************************************************************************
 *
 *   Read the camera registers, and save them to memory .
 *   Usually called when the camera is opened  for the first time
 *
 ***************************************************************************/
static int  lucam_registers_init(plucam_device plucam)
{
   ///Associate registers with their memory address.
   lucam_registers* registers = &plucam->cam_registers;
   int rt=0;
   registers-> sensor_init.reg_index=        LUCAM_INITIALIZE;
   registers-> usb_speed.reg_index=          LUCAM_USB_HIGH_SPEED;
   registers-> fpga_mode.reg_index=          LUCAM_FPGA_MODE;
   registers-> format_count.reg_index=       LUCAM_FORMAT_COUNT;
   registers-> video_en.reg_index=           LUCAM_VIDEO_EN;
   registers-> trigger_ctl.reg_index=        LUCAM_TRIGGER_CTRL;
   registers-> fo_color_id.reg_index=        LUCAM_FO_COLOR_ID;
   registers-> fo_tap_configuration.reg_index=     LUCAM_FO_TAP_CONFIGURATION;
   registers-> color_inq.reg_index=          LUCAM_COLOR_INQ;
   registers-> max_size.reg_index=           LUCAM_MAX_SIZE;
   registers-> unit_size.reg_index=          LUCAM_UNIT_SIZE;
   registers-> fo_position.reg_index=        LUCAM_FO_POSITION;
   registers-> fo_size.reg_index=            LUCAM_FO_SIZE;
   registers-> fo_subsampling.reg_index=     LUCAM_FO_SUBSAMPLING;
   registers-> fo_subsampling_inq.reg_index= LUCAM_FO_SUBSAMPLING_INQ;
   registers-> fo_kps_minmax.reg_index=      LUCAM_FO_KPS_MINMAX_INQ;
   registers-> fo_kps.reg_index=             LUCAM_FO_KPS;
   registers-> fo_kps_cnt.reg_index=         LUCAM_FO_KPS_ARRAY_CNT;
   registers-> fo_kps_array.reg_index=       LUCAM_FO_KPS_ARRAY;
   registers-> gamma.reg_index=              LUCAM_GAMMA;
   registers-> contrast.reg_index=           LUCAM_CONTRAST;
   registers-> brightness.reg_index=         LUCAM_BRIGHTNESS;
   registers-> fo_exposure.reg_index=        LUCAM_FO_EXPOSURE;
   registers-> fo_gain.reg_index=            LUCAM_FO_GAIN;
   registers-> fo_gain_red.reg_index=        LUCAM_FO_GAIN_RED;
   registers-> fo_gain_green1.reg_index=     LUCAM_FO_GAIN_GREEN1;
   registers-> fo_gain_green2.reg_index=     LUCAM_FO_GAIN_GREEN2;
   registers-> fo_gain_blue.reg_index=       LUCAM_FO_GAIN_BLUE;
   registers-> st_exposure.reg_index=        LUCAM_STILL_EXPOSURE;
   registers-> st_gain.reg_index=            LUCAM_STILL_GAIN;
   registers-> st_gain_red.reg_index=        LUCAM_STILL_GAIN_RED;
   registers-> st_gain_green1.reg_index=     LUCAM_STILL_GAIN_GREEN1;
   registers-> st_gain_green2.reg_index=     LUCAM_STILL_GAIN_GREEN2;
   registers-> st_gain_blue.reg_index=       LUCAM_STILL_GAIN_BLUE;
   registers-> st_strobe_delay.reg_index=    LUCAM_STILL_STROBE_DELAY;
   registers-> st_exposure_delay.reg_index=  LUCAM_STILL_EXPOSURE_DELAY;
   registers-> st_position.reg_index=        LUCAM_STILL_POSITION;
   registers-> st_size.reg_index=            LUCAM_STILL_SIZE;
   registers-> st_color_id.reg_index=        LUCAM_STILL_COLOR_ID;
   registers-> st_tap_configuration.reg_index=  LUCAM_STILL_TAP_CONFIGURATION;
   registers-> st_subsampling.reg_index=     LUCAM_STILL_SUBSAMPLING;
   registers-> st_strobe_duration.reg_index= LUCAM_STILL_STROBE_DURATION;
   registers-> auto_exposure_max.reg_index=   LUCAM_AUTO_EXP_MAX;
   registers-> auto_gain_min.reg_index=       LUCAM_AUTO_GAIN_MIN;
   registers-> auto_gain_max.reg_index=       LUCAM_AUTO_GAIN_MAX;
   registers-> message_support.reg_index =   LUCAM_PROP_MESSAGE_SUPPORT0;
   registers-> snapshot_count.reg_index=     LUCAM_SNAPSHOT_COUNT;
   registers-> snapshot_setting.reg_index=   LUCAM_SNAPSHOT_SETTING;
   registers-> st_shutter_type.reg_index=    LUCAM_STILL_SHUTTER_TYPE;
   registers-> trigger_pin.reg_index=        LUCAM_TRIGGER_PIN;
   registers-> strobe_pin.reg_index=         LUCAM_STROBE_PIN;
   registers-> iris.reg_index=               LUCAM_IRIS;
   registers-> focus.reg_index=               LUCAM_FOCUS;
   registers-> still_knee1_exposure.reg_index=  LUCAM_STILL_KNEE1_EXPOSURE;
   registers-> still_knee2_exposure.reg_index=  LUCAM_STILL_KNEE2_EXPOSURE;
   registers-> still_knee3_exposure.reg_index=  LUCAM_STILL_KNEE3_EXPOSURE;
   registers-> video_knee.reg_index=          LUCAM_VIDEO_KNEE;
   registers-> knee2_level.reg_index=      LUCAM_KNEE2_LEVEL;
   registers-> timestamps.reg_index=          LUCAM_TIMESTAMPS;
   registers-> bpc.reg_index=               LUCAM_BPC;
   registers->video_settings.reg_index=     LUCAM_VIDEO_SETTING;
   registers->auto_exp_target.reg_index=    LUCAM_AUTO_EXP_TARGET;
   registers->realtimestamp.reg_index=      LUCAM_REALTIMESTAMP;
   registers->realtimestamp_frequency.reg_index=  LUCAM_REALTIMESTAMP_FREQUENCY;
   registers->timestamp_hw_reset.reg_index= LUCAM_TIMESTAMP_HW_RESET;
   registers->focal_length.reg_index=       LUCAM_FOCAL_LENGTH;
   registers->iris_steps_count.reg_index=   LUCAM_IRIS_STEPS;
   registers->lsc_x.reg_index=              LUCAM_LSC_X;
   registers->lsc_y.reg_index=              LUCAM_LSC_Y;
   registers->trigger_mode.reg_index=       LUCAM_TRIGGER_MODE;


   rt = lucam_reg32_rw(plucam, LUCAM_SPECIFICATION, &plucam->specification, 1, USB_DIR_IN);
   // Get the specification verison
   if ( rt < 0)
   {
      Err(plucam, "***lucam_registers_init:%d Could not read specification number \n",rt);
      return rt;
   }
   if (plucam->specification > LUCAM_CURRENT_SPECIFICATION)
   {
      plucam->specification = LUCAM_CURRENT_SPECIFICATION;
   }

   rt = lucam_reg32_rw(plucam, LUCAM_FLAGS, &plucam->lucam_flags, 1, USB_DIR_IN);
   /* Get the validation sup[port */
   if (rt < 0)
   {
	   Err(plucam, "***lucam_registers_init:%d Could not read lucam flags\n",rt);
	   plucam->lucam_flags = 0;
	   return rt;
   }

   rt = lucam_reg32_rw(plucam, LUCAM_FIRMFPGA_VERSION, &plucam->EmbeddedVersion, 1, USB_DIR_IN);
   if (rt < 0)
   {
      plucam->EmbeddedVersion = 0;
      Err(plucam, "***lucam_registers_init:%d Could not read version number\n",rt);
      return rt;
   }

   /// Get the values of the camera registers, and save them to the lucam_device.
   lucam_get_reg(plucam, &registers-> usb_speed);
   lucam_get_reg(plucam, &registers-> fpga_mode);
   lucam_get_reg(plucam, &registers-> format_count);

   if (plucam->stream_count == 1 && registers->format_count.value == 2)
   {
      plucam->stream_count = 2;
#if !USB3_ENDPOINT_CONFIGURATION
      plucam->still_pipe = plucam->video_pipe;
#endif
   }

   // Get rid of an old and unusual problem
   lucam_enable_video(plucam, FALSE);
   lucam_enable_still(plucam, FALSE, FALSE);

   lucam_get_reg(plucam, &registers-> color_inq);
   lucam_get_reg(plucam, &registers-> max_size);
   lucam_get_reg(plucam, &registers-> unit_size);
   lucam_get_reg(plucam, &registers-> fo_color_id);
   lucam_get_reg(plucam, &registers-> fo_tap_configuration);
   if ((registers-> fo_color_id.value & 0x08000000) && (registers-> fo_tap_configuration.value == 0))
   {
      registers-> fo_tap_configuration.value = 1;
   }
   lucam_get_reg(plucam, &registers-> fo_position);
   lucam_get_reg(plucam, &registers-> fo_size);
   lucam_get_reg(plucam, &registers-> fo_subsampling);
   lucam_get_reg(plucam, &registers-> fo_subsampling_inq);
//   lucam_get_reg(plucam, &registers-> fo_kps_minmax);
//   lucam_get_reg(plucam, &registers-> fo_kps);
//   lucam_get_reg(plucam, &registers-> fo_kps_cnt);
   lucam_get_reg(plucam, &registers-> message_support);

   lucam_get_param(plucam, &registers-> gamma);
   lucam_get_param(plucam, &registers-> contrast);
   lucam_get_param(plucam, &registers-> brightness);
   lucam_get_param(plucam, &registers-> fo_exposure);
   lucam_get_param(plucam, &registers-> fo_gain);
   lucam_get_param(plucam, &registers-> fo_gain_red);
   lucam_get_param(plucam, &registers-> fo_gain_green1);
   lucam_get_param(plucam, &registers-> fo_gain_green2);
   lucam_get_param(plucam, &registers-> fo_gain_blue);
   lucam_get_param(plucam, &registers-> st_exposure);
   lucam_get_param(plucam, &registers-> st_gain);
   lucam_get_param(plucam, &registers-> st_gain_red);
   lucam_get_param(plucam, &registers-> st_gain_green1);
   lucam_get_param(plucam, &registers-> st_gain_green2);
   lucam_get_param(plucam, &registers-> st_gain_blue);
   lucam_get_param(plucam, &registers-> st_strobe_delay);
   lucam_get_param(plucam, &registers-> st_exposure_delay);
   lucam_get_param(plucam, &registers-> st_strobe_duration);

   lucam_get_param(plucam, &registers-> iris);
   lucam_get_param(plucam, &registers-> focus);
   lucam_get_param(plucam, &registers-> still_knee1_exposure);
   lucam_get_param(plucam, &registers-> still_knee2_exposure);
   lucam_get_param(plucam, &registers-> still_knee3_exposure);
   lucam_get_param(plucam, &registers-> video_knee);
   lucam_get_param(plucam, &registers-> knee2_level);
   lucam_get_param(plucam, &registers-> timestamps);
   lucam_get_param(plucam, &registers-> bpc);
   lucam_get_param(plucam, &registers-> video_settings);

   lucam_get_param(plucam, &registers-> snapshot_setting);
   lucam_get_param(plucam, &registers-> snapshot_count);

   lucam_get_param(plucam, &registers-> st_shutter_type);

   lucam_get_param(plucam, &registers-> trigger_pin);
   lucam_get_param(plucam, &registers-> strobe_pin);
   lucam_get_param(plucam, &registers-> auto_exp_target);
   lucam_get_param(plucam, &registers-> auto_exposure_max);
   lucam_get_param(plucam, &registers-> auto_gain_min);
   lucam_get_param(plucam, &registers-> auto_gain_max);
   lucam_get_param(plucam, &registers-> realtimestamp);
   lucam_get_param(plucam, &registers-> realtimestamp_frequency);
   lucam_get_param(plucam, &registers-> timestamp_hw_reset);
   lucam_get_param(plucam, &registers-> focal_length);
   lucam_get_param(plucam, &registers-> iris_steps_count);
   lucam_get_param(plucam, &registers-> lsc_x);
   lucam_get_param(plucam, &registers-> lsc_y);
   lucam_get_param(plucam, &registers-> trigger_mode);

   // If the subsampling is 0 there is risks of div by 0 errors
   if ((LO_WORD(plucam->cam_registers.fo_subsampling.value) & 0xff) == 0)
   {
      plucam->cam_registers.fo_subsampling.value = (plucam->cam_registers.fo_subsampling.value & 0xffffff00) | 1;
      Err(plucam, "***lucam_registers_init: Found a subsampling x of 0!!\n");
      return -EINVAL;
   }
   if ((HI_WORD(plucam->cam_registers.fo_subsampling.value) & 0xff) == 0)
   {
      plucam->cam_registers.fo_subsampling.value = (plucam->cam_registers.fo_subsampling.value & 0xff00ffff) | 0x10000;
      Err(plucam, "***lucam_registers_init: Found a subsampling y of 0!!\n");
      return -EINVAL;
   }
   if ((LO_WORD(plucam->cam_registers.fo_subsampling_inq.value) & 0xff) == 0)
   {
      plucam->cam_registers.fo_subsampling_inq.value = (plucam->cam_registers.fo_subsampling_inq.value & 0xffffff00) | 1;
      Err(plucam, "***lucam_registers_init: Found a subsampling inq of 0!!\n");
      return -EINVAL;
   }
   if ((LO_WORD(plucam->cam_registers.fo_size.value)) == 0 || (HI_WORD(plucam->cam_registers.fo_size.value)) == 0)
   {
      plucam->cam_registers.fo_size.value = plucam->cam_registers.max_size.value;
      Err(plucam, "***lucam_registers_init: Found a invalid size!!\n");
      return -EINVAL;
   }

#ifdef DRIVER_INITRESET
   if (CustomInitReset(plucam) < 0)
   {
      Err(plucam, "***lucam_registers_init: CustomInitReset failed\n");
   }
#endif

   //The still properities is not readable, so borrow from their fo peers.
   registers-> st_position.value = registers-> fo_position.value;
   registers-> st_size.value     = registers-> fo_size.value;
   registers-> st_color_id.value = registers-> fo_color_id.value;
   registers-> st_tap_configuration.value = registers-> fo_tap_configuration.value;
   registers-> st_subsampling.value = registers-> fo_subsampling.value;

   lucam_set_reg(plucam, &registers-> st_position);
   lucam_set_reg(plucam, &registers-> st_size);
   lucam_set_reg(plucam, &registers-> st_subsampling);
   lucam_set_reg(plucam, &registers-> st_color_id);
   lucam_set_reg(plucam, &registers-> st_tap_configuration);
   lucam_get_reg(plucam, &registers-> st_tap_configuration);
   if ((registers-> st_color_id.value & 0x08000000) && (registers-> st_tap_configuration.value == 0))
   {
      registers-> st_tap_configuration.value = 1;
   }

   if (is_prop_supported(&registers->st_gain_blue.value))
   {
      memcpy(&registers-> st_gain_blue.value,&registers-> fo_gain_blue.value,sizeof(lucam_prop));
      lucam_set_param(plucam, &registers->st_gain_blue);
   }
   if (is_prop_supported(&registers->st_gain_red.value))
   {
      memcpy(&registers-> st_gain_red.value,&registers-> fo_gain_red.value,sizeof(lucam_prop));
      lucam_set_param(plucam, &registers->st_gain_red);
   }
   if (is_prop_supported(&registers->st_gain_green1.value))
   {
      memcpy(&registers-> st_gain_green1.value,&registers->fo_gain_green1.value,sizeof(lucam_prop));
      lucam_set_param(plucam, &registers->st_gain_green1);
   }
   if (is_prop_supported(&registers->st_gain_green2.value))
   {
      memcpy(&registers-> st_gain_green2.value,&registers-> fo_gain_green2.value,sizeof(lucam_prop));
      lucam_set_param(plucam, &registers->st_gain_green2);
   }
   if (is_prop_supported(&registers->st_gain.value))
   {
      memcpy(&registers-> st_gain.value,&registers-> fo_gain.value,sizeof(lucam_prop));
      lucam_set_param(plucam, &registers->st_gain);
   }
   if (is_prop_supported(&registers->st_exposure.value))
   {
      memcpy(&registers-> st_exposure.value, &registers-> fo_exposure.value,sizeof(lucam_prop));
      lucam_set_param(plucam, &registers->st_exposure);
   }
   if (is_prop_supported(&registers->st_strobe_delay.value))
   {
      registers-> st_strobe_delay.value.flags &= ~0xffff;
      registers-> st_strobe_delay.value.value = 0;
      lucam_set_param(plucam, &registers->st_strobe_delay);
   }
   if (is_prop_supported(&registers->st_exposure_delay.value))
   {
      registers-> st_exposure_delay.value.flags &= ~0xffff;
      registers-> st_exposure_delay.value.value = 0;
      lucam_set_param(plucam, &registers->st_exposure_delay);
   }

   return rt;

   //lucam_get_kps_array(plucam);
}


/* Here we want the physical address of the memory.
 * This is used when initializing the contents of the
 * area and marking the pages as reserved.
 */
static inline unsigned long kvirt_to_pa(unsigned long adr)
{
    unsigned long kva, ret;

    kva = (unsigned long) page_address(vmalloc_to_page((void *)adr));
    kva |= adr & (PAGE_SIZE-1); /* restore the offset */
    ret = __pa(kva);
    return ret;
}

static inline void *kvirt_to_kva(void *adr)
{
    unsigned long kva;

    BUG_ON(adr == NULL);

    kva = (unsigned long) page_address(vmalloc_to_page((void *)adr));
    kva |= (unsigned long)adr & (PAGE_SIZE-1); /* restore the offset */
    return (void *)kva;
}

/****************************************************************************

The urb completion handler, move data from USB bus to the memory.
This is supoposedly called in interrupt context. Therefore, no blocking.
**************************************************************************/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static void lucam_bulk_handler(struct urb *urb)
#else
static void lucam_bulk_handler(struct urb *urb, struct pt_regs *ptregs)
#endif
{
   int frame_error;
   struct _image_buf *pimage_buf;
   struct _bulk_buf *pbulk_buf;
   struct _lucam_device *plucam;
   //unsigned long flags = 0;

   frame_error = 0;

   pbulk_buf = (struct _bulk_buf *)urb->context;
   if (pbulk_buf == NULL)
   {
      Err(NULL, "***lucam_bulk_handler: No bulk buf!\n");
      return;
   }
   pbulk_buf->submitted = 0;
   pimage_buf = pbulk_buf->pimage_buf;
   if (pimage_buf == NULL)
   {
      Err(NULL, "***lucam_bulk_handler: No image buf!\n");
      return;
   }
   plucam = (struct _lucam_device *)pimage_buf->plucam;
   if(!plucam)
   {
      Err(NULL, "***lucam_bulk_handler: No plucam!\n");
      return;
   }

   DEBUG_COUNT(plucam, bh_count);

   //|| !plucam->stream_on || plucam->frame_error > 0
   //  Info(plucam, TRACE_BULK, "lucam_bulk_handler() called with status %d \n", urb->status);
   if (urb->status == -ENOENT || urb->status == -ECONNRESET)
   {
      /* When the stream is stoppped this happens, so not really an error. */
      Info(plucam, TRACE_BULK, "lucam_bulk_handler(): URB unlinked.ret = %d \n", urb->status);
      DEBUG_COUNT(plucam, bh_cancelled);
      //lucam_release_writeframe(plucam);
      //return;
   }
   else if (urb->status == -EINPROGRESS)
   {
      Err(plucam, "***lucam_bulk_handler():Still in process \n");
      DEBUG_COUNT(plucam, bh_inprogress);
      //return;
   }
   else if (urb->status != 0 && urb->status != -EREMOTEIO) /* -EREMOTEIO happens when there is a short packet. */
   {
      char *errmsg;
      switch(urb->status)
      {
      case -ENOSR:
         DEBUG_COUNT(plucam, bh_buffererror);
         errmsg = "Buffer error (overrun)";
         break;
      case -EPIPE:
         DEBUG_COUNT(plucam, bh_stalled);
         errmsg = "Stalled (device not responding)";
         break;
      case -EOVERFLOW:
         DEBUG_COUNT(plucam, bh_babble);
         errmsg = "Babble (bad cable?)";
         break;
      case -EPROTO:
         DEBUG_COUNT(plucam, bh_bitstuff);
         errmsg = "Bit-stuff error (bad cable?)"; // or unknown error (may not be an error at all)
         break;
      case -EILSEQ:
         DEBUG_COUNT(plucam, bh_crctimeout);
         errmsg = "CRC/Timeout";
         break;
      case -ETIMEDOUT:
         DEBUG_COUNT(plucam, bh_nak);
         errmsg = "NAK (device does not respond)";
         break;
      default:
         DEBUG_COUNT(plucam, bh_unknownerror);
         errmsg = "Unknown";
         break;
      }
      Err(plucam, "***lucam_bulk_handler() called with status %d [%s] for index %d.\n", urb->status, errmsg, pbulk_buf->index);
      //if(!plucam->unplugged) usb_clear_halt(plucam->udev, usb_pipeendpoint(urb->pipe));
      frame_error = FRAME_ERROR_REQUIRE_RESTART+1;
      //lucam_release_writeframe(plucam);
      //goto submit;
       // do not expect this bulkbuf anymore
      plucam->last_bulk_index++;
      if (plucam->last_bulk_index == plucam->bulk_bufs_used) plucam->last_bulk_index = 0;
   }
   else
   {
      plucam->last_bulk_index++;
      if (plucam->last_bulk_index == plucam->bulk_bufs_used) plucam->last_bulk_index = 0;

      if (pbulk_buf->index != plucam->last_bulk_index)
      {
         DEBUG_COUNT(plucam, bh_desyncherror);
         Err(plucam, "***lucam_bulk_handler: desynch detected, expected=%d, got=%d\n",
            plucam->last_bulk_index, pbulk_buf->index);
         frame_error=FRAME_ERROR_REQUIRE_RESTART+2;
      }
      else if (urb->transfer_buffer_length != urb->actual_length)
      {
         DEBUG_COUNT(plucam, bh_shortpacketerror);
         if (plucam->current_stream_state < LUCAM_STREAM_STATE_PAUSE2RUN)
         {
            Info(plucam, TRACE_USB | TRACE_BULK, "*lucam_bulk_handler: Short packet received=%d instead of %d\n",
               urb->actual_length, urb->transfer_buffer_length);
         }
         else
         {
            Info(plucam, TRACE_USB | TRACE_BULK, "***lucam_bulk_handler: Short packet received=%d instead of %d\n",
               urb->actual_length, urb->transfer_buffer_length);
         }
         
         frame_error = FRAME_ERROR_REQUIRE_RESTART;
      }
      else
      {
         DEBUG_COUNT(plucam, bh_success);
      }

      if (urb->status == -EREMOTEIO)
      {
         Info(plucam, TRACE_BULK, "Warning: -EREMOTEIO returned, actual length=%d\n", urb->actual_length);
         DEBUG_COUNT(plucam, bh_remoteioerror);
      }
   }

   // Here we may need to trigger some state change
   if (plucam->current_stream_state >= LUCAM_STREAM_STATE_PAUSE2RUN &&
          (frame_error >= FRAME_ERROR_REQUIRE_RESTART) &&
          plucam->frame_error < FRAME_ERROR_REQUIRE_RESTART) 
   {
      // frame error just detected, trigger the state change
      plucam->frame_error = frame_error;

      schedule_work(&plucam->change_state_task);
      DEBUG_COUNT(plucam, restarts_scheduled);
   }

#if MEM_IN_BULK_BUF
   memcpy(pbulk_buf->pimage_buf->data + pbulk_buf->offset, pbulk_buf->data, urb->actual_length);
#endif

   // We have a frame buf (well, we must...)
   if (pbulk_buf->is_last)
   {
      DEBUG_COUNT(plucam, frame_completions);

      Info(plucam, TRACE_BULK, "bulk_handler: i=%d last, frame_error=%d, error_framebuf=0x%p, size=%d, bh_count:0x%x\n",
          pbulk_buf->pimage_buf->vb.i, plucam->frame_error, plucam->error_image_buf, urb->actual_length, plucam->counters.bh_count);

#if ENHANCED_BUFFERING
      if (1)
#else
      if (pimage_buf != &plucam->blank_image_buf)
#endif
      {
         // It is not the 'dummy' image_buf and it is the last of a series, so complete it if necessary.
         if (plucam->frame_error < FRAME_ERROR_REQUIRE_RESTART)
         {
             //spin_lock_irqsave(&plucam->slock, flags);


            // We will need to complete this image_buf
            pimage_buf->error_code = urb->status;

            pimage_buf->status = urb->status;
            if (plucam->frame_error >= FRAME_ERROR_REQUIRE_RESTART && urb->status == 0) pimage_buf->status = -EFAULT;

            if (pimage_buf->status)
            {
                 DEBUG_COUNT(plucam, frame_errors);
            }

            plucam->frames++;

            //spin_unlock_irqrestore(&plucam->slock, flags);

            Info(plucam, TRACE_BULK, "lucam_bulk_handler: Waking up for image buf#%d\n",
               pimage_buf->vb.i);
            pimage_buf->vb.state = VIDEOBUF_DONE;
            wake_up(&pimage_buf->vb.done);
         }
         else
         {
            if (plucam->error_image_buf)
            {
               struct _image_buf *pimage_buf_swap;

               Err(plucam, "*** bulk_handler: ERROR: plucam->error_image_buf not NULL with is_last!!\n");

                 //spin_lock_irqsave(&plucam->slock, flags);

               // swap
               pimage_buf_swap = pimage_buf;
               pimage_buf = plucam->error_image_buf;
               plucam->error_image_buf = pimage_buf_swap;


               // We will need to complete this image_buf
               pimage_buf->error_code = urb->status;

               pimage_buf->status = urb->status;
               if (plucam->frame_error > FRAME_ERROR_REQUIRE_RESTART && urb->status == 0) pimage_buf->status = -EFAULT;

               if (pimage_buf->status)
               {
                  DEBUG_COUNT(plucam, frame_errors);
               }

               plucam->frames++;
               pimage_buf->vb.field_count += 2;

               //spin_unlock_irqrestore(&plucam->slock, flags);

               pimage_buf->vb.state = VIDEOBUF_DONE;
               Info(plucam, TRACE_BULK, "lucam_bulk_handler: Waking up for image buf#%d\n",
                  pimage_buf->vb.i);
               wake_up(&pimage_buf->vb.done);
            }
            else
            {
               // Save this frame buf
               plucam->error_image_buf = pimage_buf;
            }
         }
      }
      else
      {
         // The blank buf is never used
         plucam->frames++;
         DEBUG_COUNT(plucam, frame_lost);
         Info(plucam, TRACE_BULK, "Frame lost to blank frame\n");
      }

      pimage_buf = NULL;
   }
   else
   {
      // Not the last page of a image_buf
      if (pbulk_buf->is_first 
         && pbulk_buf->pimage_buf
#if !ENHANCED_BUFFERING
         && pbulk_buf->pimage_buf != &plucam->blank_image_buf 
#endif
         && plucam->frame_error < FRAME_ERROR_REQUIRE_RESTART 
         && plucam->current_stream_state >= LUCAM_STREAM_STATE_PAUSE2RUN
         ) 
      {
         DEBUG_COUNT(plucam, timestamps);
         do_gettimeofday(&pimage_buf->vb.ts);
         Info(plucam, TRACE_BULK, " bulk_handler: Timestamp: %02d:%02d.%06d\n", (int)(pimage_buf->vb.ts.tv_sec % 3600)/60, (int)(pimage_buf->vb.ts.tv_sec % 60), (int)(pimage_buf->vb.ts.tv_usec));
      }
   }

#if ENHANCED_BUFFERING
   atomic_inc(&plucam->bulk_buf_idle_count);
#endif

   /*
    * We need to resubmit if there is no frame errors and we are still running.
    */
   if (plucam->frame_error < FRAME_ERROR_REQUIRE_RESTART &&
      plucam->current_stream_state >= LUCAM_STREAM_STATE_PAUSE2RUN &&
      !plucam->unplugged)
   {
#if ENHANCED_BUFFERING
      lucam_submit_bulkbufs(plucam, GFP_ATOMIC);
#else
      lucam_resubmit_bulk_buf(plucam, pbulk_buf);
#endif
   }
}

#if ENHANCED_BUFFERING
static int last_submit_errored;
/*****************************************************************************
 * lucam_submit_bulkbufs
 * 
 * Desc: Low level frame pump.
 * 
 * Details:
 * This function can be called from multiple contexts:
 * - lucam_change_stream_state
 * - lucam_qbuf
 * - lucam_bulk_handler
 * Submitting URBs is done with the bulk_lock held. If the lock is already held 
 * (by an other instance of this fn) then the other instance is responsible to
 * make sure everything is at what it should be.
 * A minimum two pass is done to avoid a possible race contition where:
 * Thread1:-lucam_bulk_handler is called for the last bulk buf
 *         -In this fn: acquires the lock
 *         -            finds there is no frame in the queue
 * Thread2:-Queue a frame into the queue
 *         -In this fn: Tries to acquire lock (fail), exit.
 * Thread1:-Continue in this fn: release lock. 
 * Result: No pending URB with a frame in queue. Fix: Thread1 must do a 2nd pass.
 * **************************************************************************/
static int lucam_submit_bulkbufs(struct _lucam_device *plucam, int gfp)
{
   void *data_ptr;
   struct _image_buf *pimage_buf;
   int size;
   struct urb *urb;
   int ret;
   int idle_count;
   unsigned long flags;
   int count;
   struct _bulk_buf *pbulk_buf;

   ret = 0;
   count = 0;

   while (1)
   {
      if (test_and_set_bit(0, &plucam->bulk_lock))
      {
         Info(plucam, TRACE_BULK, "lucam_resubmit_bulk_buf: busy\n");
         break;
      }

      if (plucam->frame_error < FRAME_ERROR_REQUIRE_RESTART &&
         plucam->current_stream_state >= LUCAM_STREAM_STATE_PAUSE2RUN &&
         !plucam->unplugged)
      {
         idle_count = atomic_read(&plucam->bulk_buf_idle_count);

         /* Queue this one */
         while(idle_count)
         {
            // Find the next frame buf
            pimage_buf = plucam->current_image_buf;

            if (pimage_buf == NULL) 
            {
               spin_lock_irqsave(&plucam->slock, flags);
               if (!list_empty(&plucam->active_frames)) 
               {
                  pimage_buf = list_entry(plucam->active_frames.next,
                        struct _image_buf, vb.queue);

      #if 0
                 /* If nobody is waiting on this buffer, return */
                  if (waitqueue_active(&pimage_buf->vb.done)) {
                     pimage_buf = NULL;
                  }
                  else
      #endif
                  {
                     list_del_init(&pimage_buf->vb.queue);
                     pimage_buf->vb.state = VIDEOBUF_ACTIVE;
                  }
               }
               spin_unlock_irqrestore(&plucam->slock, flags);

               if (pimage_buf == NULL) 
               {
                  /* ASSERT: no image buffers to fill, stop
                   * until we get some*/
                  
                  break;
               }

               Info(plucam, TRACE_BULK, "%s: filling image %d\n", 
                    __func__, pimage_buf->vb.i);
               plucam->current_image_buf = pimage_buf;
               pimage_buf->offset = 0;
            }

            /* Get a bulk_buf */
            if (pbulk_buf == NULL)
            {
               /*
                * We are looking at submitting the bulk bufs in order, so submit the
                * first one.
                */
               pbulk_buf = &plucam->bulk_bufs[plucam->bulk_buf_idle_first];
            }

            pbulk_buf->pimage_buf = pimage_buf;

            if (pimage_buf->offset + PAGE_SIZE > plucam->framesize)
            {
               size = plucam->framesize - pimage_buf->offset;
            }
            else
            {
               size = PAGE_SIZE;
            }
            pbulk_buf->is_first = pimage_buf->offset == 0;
            pbulk_buf->is_last = (pimage_buf->offset + size) == plucam->framesize;
      #if MEM_IN_BULK_BUF
            pbulk_buf->offset = pimage_buf->offset;
      #endif
      #if ALLOW_RGB24_OUTPUT
            if (pimage_buf->needs_conversion)
               data_ptr = pimage_buf->raw_data;
            else
      #endif
            data_ptr = pimage_buf->data;

            data_ptr += pimage_buf->offset;

            urb = pbulk_buf->urb;

            //Info(plucam, TRACE_BULK, "%s: urb=%d, time_out = %d \n", __func__, urb->index, urb->timeout);
            usb_fill_bulk_urb (
              urb,
              plucam->udev,
      #if USB3_ENDPOINT_CONFIGURATION
              plucam->video_pipe,
      #else
              (plucam->camera_mode==LUCAM_VIDEO_MODE)?
                                  plucam->video_pipe:
                                  plucam->still_pipe,
      #endif
      #if MEM_IN_BULK_BUF
              pbulk_buf->data,
      #else
              kvirt_to_kva(data_ptr),
      #endif
              size,
              lucam_bulk_handler,
              pbulk_buf);

            urb->status = 0;
      #if MEM_IN_BULK_BUF
            urb->transfer_flags = URB_NO_TRANSFER_DMA_MAP;
      #else
            urb->transfer_flags = 0; //USB_DISABLE_SPD;    //|USB_QUEUE_BULK;
      #endif

            pbulk_buf->submitted = 1;

            ret = usb_submit_urb(urb, gfp);

            if (ret < 0)
            {
               pbulk_buf->submitted = 0;
               DEBUG_COUNT(plucam, bulkbufs_submit_failures);
               if (!last_submit_errored)
               {
                  Err(plucam, "resubmit ret =%d \n", ret);
               }
               last_submit_errored=1;
               break;
            }
            else
            {
               last_submit_errored=0;
               count++;
               atomic_dec(&plucam->bulk_buf_idle_count);
               DEBUG_COUNT(plucam, bulkbufs_submitted);
            }

            pimage_buf->offset += size;
            if (pbulk_buf->is_last)
            {
               pimage_buf = NULL;
               plucam->current_image_buf = NULL;

               //Info(plucam, TRACE_BULK, "Bulk handler: submitted last page of transfer\n");
            }

            plucam->bulk_buf_idle_first++;
            if (plucam->bulk_buf_idle_first == plucam->bulk_bufs_used)
            {
               plucam->bulk_buf_idle_first = 0;
            }
            idle_count--;

            pbulk_buf = NULL;
         }
      }
      
      if (test_and_clear_bit(0, &plucam->bulk_lock) == 0)
      {
         Err(plucam, "***Held lock already cleared??\n");
      }

      if (ret != 0 || count)
         break;
      ret = 1; // Guarantees a max of 2 pass
      pbulk_buf = NULL;
   }
   if (ret < 0)
      return ret;
   return count;
}

#else // of #if ENHANCED_BUFFERING
/*************************************************************************
* Resubmit a bulk URB, called from completion handler
***************************************************************************/
static void lucam_resubmit_bulk_buf(struct _lucam_device *plucam, struct _bulk_buf *pbulk_buf)
{
   void *data_ptr;
   struct _image_buf *pimage_buf;
   int size;
   struct urb *urb;
   int ret;
   unsigned long flags = 0;

   // Find the next frame buf
   pimage_buf = plucam->current_image_buf;

   if (pimage_buf == NULL) 
   {
      spin_lock_irqsave(&plucam->slock, flags);
      if (list_empty(&plucam->active_frames)) 
      {
         pimage_buf = &plucam->blank_image_buf;
      }
      else
      {
         pimage_buf = list_entry(plucam->active_frames.next,
                        struct _image_buf, vb.queue);

         list_del_init(&pimage_buf->vb.queue);
         pimage_buf->vb.state = VIDEOBUF_ACTIVE;
      }
      spin_unlock_irqrestore(&plucam->slock, flags);

      plucam->current_image_buf = pimage_buf;
      pimage_buf->offset = 0;

      Info(plucam, TRACE_BULK, "%s: filling image %d\n", __func__,
         pimage_buf->vb.i);
   }

   pbulk_buf->pimage_buf = pimage_buf;

   if (pimage_buf->offset + PAGE_SIZE > plucam->framesize)
   {
      size = plucam->framesize - pimage_buf->offset;
      Info(plucam, TRACE_BULK, "  resubmit bulk: last: %d, already submitted=%d\n", size, plucam->counters.bulkbufs_submitted);
   }
   else
   {
      size = PAGE_SIZE;
   }
   pbulk_buf->is_first = (pimage_buf->offset == 0);
   pbulk_buf->is_last = ((pimage_buf->offset + size) == plucam->framesize);
#if MEM_IN_BULK_BUF
   pbulk_buf->offset = pimage_buf->offset;
#endif
#if ALLOW_RGB24_OUTPUT
   if (pimage_buf->needs_conversion == 0)
   {
      data_ptr = pimage_buf->raw_data;
   }
   else
#endif
   data_ptr = pimage_buf->data;

   data_ptr += pimage_buf->offset;

   urb = pbulk_buf->urb;

   //Info(plucam, TRACE_BULK, "%s: urb=%d\n", __func__, pbulk_buf->index);
   usb_fill_bulk_urb (
        urb,
        plucam->udev,
#if USB3_ENDPOINT_CONFIGURATION
        plucam->video_pipe,
#else
        (plucam->camera_mode==LUCAM_VIDEO_MODE)?
            plucam->video_pipe:
            plucam->still_pipe,
#endif
#if MEM_IN_BULK_BUF
        pbulk_buf->data,
#else
        kvirt_to_kva(data_ptr),
#endif
        size,
        lucam_bulk_handler,
        pbulk_buf);

   urb->status = 0;
#if MEM_IN_BULK_BUF
   urb->transfer_flags = URB_NO_TRANSFER_DMA_MAP;
#else
   urb->transfer_flags = 0; //USB_DISABLE_SPD;    //|USB_QUEUE_BULK;
#endif

   pimage_buf->offset += size;
   if (pbulk_buf->is_last)
   {
      pimage_buf = NULL;
      plucam->current_image_buf = NULL;

      //Info(plucam, TRACE_BULK, "Bulk handler: submitted last page of transfer\n");
   }

   pbulk_buf->submitted = 1;

   ret = usb_submit_urb(urb, GFP_ATOMIC);

   if (ret < 0)
   {
      Err(plucam, "***resubmit ret = %d \n", ret);
      DEBUG_COUNT(plucam, bulkbufs_submit_failures);
      pbulk_buf->submitted = 0;
   }
   else 
   {
      DEBUG_COUNT(plucam, bulkbufs_submitted);
   }

}
#endif


/****************************************************************************
 *
 *  Stop the stream. Goes from a state where all is running 
 * to a state resources are allocated.
 ***************************************************************************/
static void lucam_stop_stream(struct _lucam_device *plucam)
{
   plucam_bulk_buf pbulk_buf;
   int bulkbufindex;
   int lastbulkindex;
   int urbkilled;

   Info(plucam, TRACE_USB | TRACE_BULK, "lucam_stop_stream() called \n");

   if (plucam == NULL) return;

   if (plucam->current_stream_state <= LUCAM_STREAM_STATE_PAUSE) return;

   plucam->current_stream_state = LUCAM_STREAM_STATE_RUN2PAUSE;

   // A lot to do:
   // 1. Tell the camera to stop sending data
   // 2. Unlink all URBs
   // 3. Reset the endpoint
   // 4. Make sure the complete_frame task is done and not scheduled.
   // 5. Reinitialize the ptrs for the image_bufs

   // Do the part #1
   if(!plucam->unplugged && plucam->camera_enabled)
   {
      if (plucam->camera_mode == LUCAM_VIDEO_MODE)
      {
         lucam_enable_video(plucam, 0);
      }
      else
      {
         lucam_enable_still(plucam, 0, 0);
      }

      plucam->camera_enabled = 0;
   }

   // Now part #2
   // Hopefully unlinking an already unlinked URB poses no problem.
   // and after the unlink returns, the bulk handler was called.
   Info(plucam, TRACE_USB | TRACE_BULK, "Stop: Unlinking %d bulkbufs\n", plucam->bulk_bufs_used);
   lastbulkindex = plucam->last_bulk_index;
   urbkilled=0;
   for (bulkbufindex = 0 ; bulkbufindex < plucam->bulk_bufs_used ; bulkbufindex++)
   {
      pbulk_buf = &plucam->bulk_bufs[(lastbulkindex + bulkbufindex + 1) % plucam->bulk_bufs_used];

      if (pbulk_buf->submitted)
      {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,8)
         // usb_unlink_urb redirects to usb_kill_urb from 2.6.8.
         // In 2.6.10 a unlink can printk a 'Badness' warning.
         // usb_kill_urb was added for 2.6.8.
         usb_kill_urb(pbulk_buf->urb);
#else
         usb_unlink_urb(pbulk_buf->urb);
#endif
         urbkilled++;
      }
   }
#if ENHANCED_BUFFERING
   atomic_set(&plucam->bulk_buf_idle_count, 0);
   plucam->bulk_buf_idle_first = 0;
#endif
   Info(plucam, TRACE_USB | TRACE_BULK, "Stop: Killed %d URBs\n", urbkilled);

   // Part 3
   // Note: There is a bug with lucam_clear_halt with USB3: It does not clear
   // halt properly if there is no actual halt condition.
   // However some of the early Lumenera cameras do not properly clear the USB2
   // data toggle when switching alternate setting.
   if(!plucam->unplugged && (plucam->frame_error || 
#if !USB3_ENDPOINT_CONFIGURATION
            plucam->data_setting || 
#endif
            (plucam->udev->speed < USB_SPEED_SUPER)))
   {
      int pipe;

#if USB3_ENDPOINT_CONFIGURATION
      pipe = plucam->video_pipe;
#else
      pipe = (plucam->camera_mode==LUCAM_VIDEO_MODE)?
              plucam->video_pipe:
              plucam->still_pipe;
#endif

      lucam_clear_halt(plucam, pipe);
   }
   plucam->frame_error = 0;

   // PAart 4: No! it is on the same task q
   //lucam_wait_for_complete_frame_task(plucam, 0);

   // Part 5
   lucam_reset_framebuf_queues(plucam);

   {
      struct lucam_counters *counters = &plucam->counters;
      Info(plucam, TRACE_BULK, " total frames completions: %d\n", counters->frame_completions);
      Info(plucam, TRACE_BULK, "            frames errors: %d\n", counters->frame_errors);
      Info(plucam, TRACE_BULK, "       timestamps written: %d\n", counters->timestamps);
      Info(plucam, TRACE_BULK, "              frames lost: %d\n", counters->frame_lost);
      Info(plucam, TRACE_BULK, " lucam_bulk_handler calls: %d\n", counters->bh_count);
      Info(plucam, TRACE_BULK, "                successes: %d\n", counters->bh_success);
      Info(plucam, TRACE_BULK, "       in progress errors: %d\n", counters->bh_inprogress);
      Info(plucam, TRACE_BULK, "            buffer errors: %d\n", counters->bh_buffererror);
      Info(plucam, TRACE_BULK, "             stall errors: %d\n", counters->bh_stalled);
      Info(plucam, TRACE_BULK, "            babble errors: %d\n", counters->bh_babble);
      Info(plucam, TRACE_BULK, "         bit stuff errors: %d\n", counters->bh_bitstuff);
      Info(plucam, TRACE_BULK, "       CRC timeout errors: %d\n", counters->bh_crctimeout);
      Info(plucam, TRACE_BULK, "       NAK timeout errors: %d\n", counters->bh_nak);
      Info(plucam, TRACE_BULK, "           unknown errors: %d\n", counters->bh_unknownerror);
      Info(plucam, TRACE_BULK, "           desynch errors: %d\n", counters->bh_desyncherror);
      Info(plucam, TRACE_BULK, "                cancelled: %d\n", counters->bh_cancelled);
      Info(plucam, TRACE_BULK, "         remote io errors: %d\n", counters->bh_remoteioerror);
      Info(plucam, TRACE_BULK, "     short packets errors: %d\n", counters->bh_shortpacketerror);
      Info(plucam, TRACE_BULK, "       restarts scheduled: %d\n", counters->restarts_scheduled);
      Info(plucam, TRACE_BULK, "       bulkbufs submitted: %d\n", counters->bulkbufs_submitted);
      Info(plucam, TRACE_BULK, " bulkbufs submit failures: %d\n", counters->bulkbufs_submit_failures);
      Info(plucam, TRACE_BULK, "          bulkbufs killed: %d\n", urbkilled);

      if (plucam->wanted_stream_state != LUCAM_STREAM_STATE_RUN)
      {
         counters->frame_completions = 0;
         counters->frame_errors = 0;
         counters->timestamps = 0;
         counters->frame_lost = 0;
         counters->bh_count = 0;
         counters->bh_success = 0;
         counters->bh_inprogress = 0;
         counters->bh_buffererror = 0;
         counters->bh_stalled = 0;
         counters->bh_babble = 0;
         counters->bh_bitstuff = 0;
         counters->bh_crctimeout = 0;
         counters->bh_nak = 0;
         counters->bh_unknownerror = 0;
         counters->bh_desyncherror = 0;
         counters->bh_cancelled = 0;
         counters->bh_remoteioerror = 0;
         counters->bh_shortpacketerror = 0;
         counters->restarts_scheduled = 0;
         counters->bulkbufs_submitted = 0;
         counters->bulkbufs_submit_failures = 0;
      }
   }

   plucam->current_stream_state = LUCAM_STREAM_STATE_PAUSE;
}

/****************************************************************************
 *
 *  Start the stream. Goes from a state where all resources are allocated
 * to a state where all is runnning.
 ***************************************************************************/
static int lucam_start_stream(struct _lucam_device *plucam)
{
#if !ENHANCED_BUFFERING
   plucam_bulk_buf pbulk_buf;
   plucam_image_buf pimage_buf;
   int bulkbufindex, size, i;
   void *data_ptr;
#endif
   int ret=0;
   if(plucam==NULL) return -EFAULT;

   if (plucam->current_stream_state == LUCAM_STREAM_STATE_STOP) return -EFAULT;
   if (plucam->current_stream_state == LUCAM_STREAM_STATE_RUN) return 0;

   Info(plucam, TRACE_USB | TRACE_BULK, "lucam_start_stream() called \n");

   if (plucam->stream_count == 0) return -EINVAL;

   if (plucam->error_image_buf)
   {
      Err(plucam, "lucam_start_stream: error_image_buf is not NULL!!\n"); // This is not supposed to happen
      plucam->error_image_buf = NULL;
   }

   plucam->frames = 0;
   plucam->current_stream_state = LUCAM_STREAM_STATE_PAUSE2RUN;

   // Stuff we have to do here:
   // 1. Submit URBs
   // 2. Tell the device to start

   // Do part 1
   // Normally:
   // Some image_buf may be initialized correctly w a buf and placed correctly in the empty q.
   // All bulk_buf have an URB buf have no frame buf yet.

   // Must be set correctly for error checking
   plucam->last_bulk_index = plucam->bulk_bufs_used - 1;

#if ENHANCED_BUFFERING
   atomic_set(&plucam->bulk_buf_idle_count, plucam->bulk_bufs_used);
   plucam->bulk_buf_idle_first = 0;
   ret = lucam_submit_bulkbufs(plucam, GFP_KERNEL);
   if (ret < 0)
   {
      Err(plucam, "***lucam_start_stream: submit ret =%d \n", ret);
      goto error;
   }
#else
   pimage_buf = plucam->current_image_buf;
   for (bulkbufindex = 0 ; bulkbufindex < plucam->bulk_bufs_used ; bulkbufindex++)
   {
      if (pimage_buf == NULL) 
      {
         unsigned long flags;
         spin_lock_irqsave(&plucam->slock, flags);
         if (!list_empty(&plucam->active_frames)) 
         {
            plucam->current_image_buf = pimage_buf =
                    list_entry(plucam->active_frames.next,
                       struct _image_buf, vb.queue);
            list_del_init(&pimage_buf->vb.queue);
            pimage_buf->vb.state = VIDEOBUF_ACTIVE;
            ret = 1;
         }
         else 
         {
#if ENHANCED_BUFFERING
            plucam->current_image_buf = pimage_buf = NULL;
            ret = 2;
#else
            plucam->current_image_buf = pimage_buf = &plucam->blank_image_buf;
            ret = 3;
#endif
         }
         spin_unlock_irqrestore(&plucam->slock, flags);

         if (pimage_buf == NULL) 
         {
            /* ASSERT: no image buffers to fill, stop until we get some*/
            break;
         }

         Info(plucam, TRACE_USB | TRACE_BULK, "%s: filling image %d (how=%d, data=%p)\n", 
            __func__,
              pimage_buf->vb.i, ret, pimage_buf->data);
         pimage_buf->offset = 0;
      }

      pbulk_buf = &plucam->bulk_bufs[bulkbufindex];
      pbulk_buf->pimage_buf = pimage_buf;
#if MEM_IN_BULK_BUF
      pbulk_buf->offset = pimage_buf->offset;
#endif

      if (pimage_buf->offset + PAGE_SIZE > plucam->framesize) 
      {
         size = plucam->framesize - pimage_buf->offset;
      }
      else 
      {
         size = PAGE_SIZE;
      }
      pbulk_buf->is_last = ((pimage_buf->offset + size) == plucam->framesize);
      pbulk_buf->is_first = pimage_buf->offset == 0;
#if ALLOW_RGB24_OUTPUT
      if (pimage_buf->needs_conversion) 
        data_ptr = pimage_buf->raw_data;
      else
#endif
        data_ptr = pimage_buf->data;

      data_ptr += pimage_buf->offset;

      //Info(plucam, TRACE_USB | TRACE_BULK, "Filling bulk urb %d, size %d\n", pbulk_buf->index, size);

      usb_fill_bulk_urb (
            pbulk_buf->urb,
            plucam->udev,
#if USB3_ENDPOINT_CONFIGURATION
            plucam->video_pipe,
#else
            (plucam->camera_mode==LUCAM_VIDEO_MODE)?
                plucam->video_pipe:
                plucam->still_pipe,
#endif
#if MEM_IN_BULK_BUF
            pbulk_buf->data,
#else
            kvirt_to_kva(data_ptr),
#endif
            //lucam_kbuf,
            size,
            lucam_bulk_handler,
            pbulk_buf);

      pbulk_buf->urb->status = 0;
#if MEM_IN_BULK_BUF
      pbulk_buf->urb->transfer_flags = URB_NO_TRANSFER_DMA_MAP;
#else
      pbulk_buf->urb->transfer_flags = 0; //USB_DISABLE_SPD;    //|USB_QUEUE_BULK;
#endif
#if 0
      urb->pipe = (plucam->camera_mode==LUCAM_VIDEO_MODE)?
            plucam->video_pipe:
            plucam->still_pipe;
#endif

      pimage_buf->offset += size;
      if (pbulk_buf->is_last)
      {
         pimage_buf = NULL;
         plucam->current_image_buf = NULL;
      }
   }

#if ENHANCED_BUFFERING
   atomic_set(&plucam->bulk_buf_idle_count, (plucam->bulk_bufs_used - bulkbufindex));
   plucam->bulk_buf_idle_first = bulkbufindex % plucam->bulk_bufs_used;
#endif

   /* Submit the urbs */
   for (i = 0 ; i < bulkbufindex ; i++)
   {
      pbulk_buf = &plucam->bulk_bufs[i];

      pbulk_buf->submitted = 1;

      ret = usb_submit_urb(pbulk_buf->urb, GFP_KERNEL);

      if (ret <0)
      {
           pbulk_buf->submitted = 0;
           DEBUG_COUNT(plucam, bulkbufs_submit_failures);
           Err(plucam, "***lucam_start_stream: submit ret =%d \n", ret);
           goto error;
      }
      else
      {
           DEBUG_COUNT(plucam, bulkbufs_submitted);
      }
   }
#endif

   // Do part 2
   if(plucam->camera_mode == LUCAM_VIDEO_MODE)
   {
      ret = lucam_enable_video(plucam, TRUE);
   }
   else if (plucam->camera_mode == LUCAM_STILL_HW_TRIGGER_MODE)
   {
      ret = lucam_enable_still(plucam, TRUE, FALSE); //FALSE == hardware trigger
   }
   else
   {
      ret = lucam_enable_still(plucam, TRUE, TRUE); //TRUE == software trigger
   }

   if(ret < 0) goto error;
   plucam->camera_enabled = 1;
   plucam->current_stream_state = LUCAM_STREAM_STATE_RUN;
   Info(plucam, TRACE_USB | TRACE_BULK, "Camera started: request = %s, ret = %d \n", ((plucam->camera_mode == LUCAM_VIDEO_MODE)?"Video":"Still"),ret);
   return 0;
error:
   lucam_stop_stream(plucam);
   return ret;
}


#if ENHANCED_BUFFERING
/*************************************************************************
*
*/
static int lucam_kick_stream(plucam_device plucam)
{
#if !ENHANCED_BUFFERING
   plucam_bulk_buf pbulk_buf;
   plucam_image_buf pimage_buf;
   int bulkbufindex, size, i;
   void *data_ptr;
#endif
   int ret=0;
   if(plucam == NULL) return -EFAULT;

   if (plucam->current_stream_state != LUCAM_STREAM_STATE_RUN) return -EFAULT;

   Info(plucam, TRACE_USB | TRACE_BULK, "lucam_kick_stream() called \n");

   if (plucam->stream_count == 0) return -EINVAL;

   if (plucam->error_image_buf)
   {
      Err(plucam, "***lucam_kick_stream: error_image_buf is not NULL!!\n"); // This is not supposed to happen
      plucam->error_image_buf = NULL;
   }

   if (atomic_read(&plucam->bulk_buf_idle_count) < plucam->bulk_bufs_used)
   {
      /* Never supposed to get here */
      Err(plucam, "***lucam_kick_start: Stream seems kicked already...\n");
      return 0;
   }

   plucam->frames = 0;

   // Stuff we have to do here:
   // 1. Submit URBs

   // Do part 1
   // Normally:
   // Some image_buf may be initialized correctly w a buf and placed correctly in the empty q.
   // All bulk_buf have an URB buf have no frame buf yet.

   // Must be set correctly for error checking
   plucam->last_bulk_index = plucam->bulk_bufs_used - 1;

#if ENHANCED_BUFFERING
   plucam->frame_error = 0;
   atomic_set(&plucam->bulk_buf_idle_count, plucam->bulk_bufs_used);
   plucam->bulk_buf_idle_first = 0;
   ret = lucam_submit_bulkbufs(plucam, GFP_KERNEL);
   if (ret < 0)
   {
      Err(plucam, "***lucam_kick_stream: submit ret =%d \n", ret);
      goto error;
   }
#else
   pimage_buf = plucam->current_image_buf;
   for (bulkbufindex = 0 ; bulkbufindex < 1 ; bulkbufindex++)
   {
      if (pimage_buf == NULL) 
      {
         if (!list_empty(&plucam->active_frames)) 
         {
            plucam->current_image_buf = pimage_buf =
                      list_entry(plucam->active_frames.next,
                       struct _image_buf, vb.queue);
            pimage_buf->vb.state = VIDEOBUF_ACTIVE;
            list_del_init(&pimage_buf->vb.queue);
         } else {
            Err(plucam, "***Error: Stream was kicked with no input image_buf\n");
            plucam->current_image_buf = pimage_buf = NULL;
            break;
         }

         Info(plucam, TRACE_USB| TRACE_BULK, "%s: filling image %d\n", __func__,
            pimage_buf->vb.i);

         pimage_buf->offset = 0;
      }

      pbulk_buf = &plucam->bulk_bufs[bulkbufindex];
      pbulk_buf->pimage_buf = pimage_buf;

      if (pimage_buf->offset + PAGE_SIZE > plucam->framesize)
      {
         size = plucam->framesize - pimage_buf->offset;
      }
      else
      {
         size = PAGE_SIZE;
      }
      pbulk_buf->is_last = ((pimage_buf->offset + size) == plucam->framesize);
      pbulk_buf->is_first = pimage_buf->offset == 0;
#if ALLOW_RGB24_OUTPUT
      if (pimage_buf && pimage_buf->needs_conversion == 0)
      {
         data_ptr = pimage_buf->raw_data;
      }
      else
#endif
      {
         data_ptr = pimage_buf->data;
      }

      data_ptr += pimage_buf->offset;

      //Info(plucam, TRACE_USB | TRACE_BULK, "%s: urb=%d, time_out = %d \n", __func__, urb->index, urb->timeout);
      usb_fill_bulk_urb (
            pbulk_buf->urb,
            plucam->udev,
#if USB3_ENDPOINT_CONFIGURATION
            plucam->video_pipe,
#else
            (plucam->camera_mode==LUCAM_VIDEO_MODE)?
                plucam->video_pipe:
                plucam->still_pipe,
#endif
#if MEM_IN_BULK_BUF
            pbulk_buf->data,
#else
            kvirt_to_kva(data_ptr),
#endif
            //lucam_kbuf,
            size,
            lucam_bulk_handler,
            pbulk_buf);
      //urb->dev=plucam->udev;
      pbulk_buf->urb->status = 0;
#if MEM_IN_BULK_BUF
      pbulk_buf->urb->transfer_flags = URB_NO_TRANSFER_DMA_MAP;
#else
      pbulk_buf->urb->transfer_flags = 0; //USB_DISABLE_SPD;    //|USB_QUEUE_BULK;
#endif
#if 0
      urb->pipe = (plucam->camera_mode==LUCAM_VIDEO_MODE)?
         plucam->video_pipe:
         plucam->still_pipe;
#endif

      pimage_buf->offset += size;
      if (pbulk_buf->is_last)
      {
         pimage_buf = NULL;
         plucam->current_image_buf = NULL;
      }
   }

#if ENHANCED_BUFFERING
   atomic_set(&plucam->bulk_buf_idle_count, (plucam->bulk_bufs_used - bulkbufindex));
   plucam->bulk_buf_idle_first = bulkbufindex % plucam->bulk_bufs_used;
#endif

   plucam->frame_error = 0;

   /* Submit the urbs */
   for (i = 0 ; i < bulkbufindex ; i++)
   {
      pbulk_buf = &plucam->bulk_bufs[i];

      pbulk_buf->submitted = 1;

      ret = usb_submit_urb(pbulk_buf->urb, GFP_KERNEL);

      if (ret <0)
      {
         pbulk_buf->submitted = 0;
         DEBUG_COUNT(plucam, bulkbufs_submit_failures);
         Err(plucam, "submit ret =%d \n", ret);
         goto error;
      }
      else
      {
           DEBUG_COUNT(plucam, bulkbufs_submitted);
      }
   }
#endif

   Info(plucam, TRACE_USB | TRACE_BULK, "Camera kicked: request = %s, ret = %d \n", ((plucam->camera_mode == LUCAM_VIDEO_MODE)?"Video":"Still"),ret);
   return 0;
error:
   lucam_stop_stream(plucam);
   return ret;
}
#endif


/****************************************************************************
 *
 *  Driver resouce alloc/dealloc -- bulk buffers, frame buffers
 *
 ***************************************************************************/
static void lucam_dealloc_framebuf(plucam_device plucam)
{
   Info(plucam, TRACE_MEMORY, "lucam_dealloc_framebuf() called \n");
   if(plucam==NULL) return;

#if !ENHANCED_BUFFERING
   if (plucam->blank_image_buf.data)
   {
      vfree(plucam->blank_image_buf.data);
      plucam->blank_image_buf.data = NULL;
#if ALLOW_RGB24_OUTPUT
      plucam->blank_image_buf.raw_data = NULL;
#endif
   }
#endif

   Info(plucam, TRACE_MEMORY, "lucam_dealloc_framebuf() finished \n");
}


//------------------------------------------------------------------------------------
//
static int lucam_alloc_framebuf(plucam_device plucam)
{
#if !ENHANCED_BUFFERING
    int alloc_size;
#endif

    if(plucam==NULL) return -EFAULT;
#if !ENHANCED_BUFFERING
    alloc_size = (plucam->framesize + PAGE_SIZE-1) & ~(PAGE_SIZE-1);
#endif

    Info(plucam, TRACE_MEMORY, "lucam_alloc_framebuf() called \n");

    /* we assume the destination image buffer has been allocated if
     * execution gets here at all */

#if !ENHANCED_BUFFERING
    // alloc 1 the dummy image_buf
    plucam->blank_image_buf.data = vmalloc_32(alloc_size);  // TODO: alignment?
    if (plucam->blank_image_buf.data == NULL)
    {
        Err(plucam, "***Failed to allocate dummy frame buffer size=%d \n", alloc_size);
        return -ENOMEM;
    }
   if ((unsigned long)(plucam->blank_image_buf.data) & (PAGE_SIZE-1))
   {
        Err(plucam, "***Failed to align frame buffer size=%d, addr=0x%p\n", alloc_size, plucam->blank_image_buf.data);
      vfree(plucam->blank_image_buf.data);
      plucam->blank_image_buf.data = NULL;
        return -ENOMEM;
   }
#if ALLOW_RGB24_OUTPUT
    plucam->blank_image_buf.raw_data = plucam->blank_image_buf.data;
#endif

#endif
    lucam_reset_framebuf_queues(plucam);

    return 0;
}


//-----------------------------------------------------------------------------------------
//
static void lucam_dealloc_bulkbuf(struct _lucam_device *plucam)
{
   int i;
   if(plucam==NULL) return;

   Info(plucam, TRACE_INFO, "lucam_dealloc_bulkbuf\n");

   for(i=0;i<plucam->bulk_bufs_used;i++)
   {
      if (plucam->bulk_bufs[i].urb)
      {
#if MEM_IN_BULK_BUF
         usb_free_coherent(plucam->udev, PAGE_SIZE, plucam->bulk_bufs[i].data,
                         plucam->bulk_bufs[i].urb->transfer_dma);
         plucam->bulk_bufs[i].data = NULL;
#endif
         usb_free_urb(plucam->bulk_bufs[i].urb);
         plucam->bulk_bufs[i].urb = NULL;
      }
   }
   plucam->bulk_bufs_used = 0;
}


//-------------------------------------------------------------------------------------------------
//
static int lucam_alloc_bulkbuf(struct _lucam_device *plucam)
{
   int i;
   int count;
   struct urb *urb;
   int ret = -ENOMEM;

   if(plucam==NULL) return -EFAULT;

   Info(plucam, TRACE_INFO, "lucam_alloc_bulkbuf\n");

   // Determine the # of bulk bufs we need
   count = 1 * ((plucam->framesize + PAGE_SIZE-1) / PAGE_SIZE); // total # of pages for 1 frame
   if (count > MAX_BULK_PACKETS) count = MAX_BULK_PACKETS;
   plucam->bulk_bufs_used = count;

   for(i=0;i<count;i++)
   {
      urb=usb_alloc_urb(0,GFP_KERNEL);

      if(urb == NULL)
      {
         Err(plucam, "***Failed to allocate urb %d\n", i);
         ret = -ENOMEM;
         break;
      }
      plucam->bulk_bufs[i].urb = urb;
      plucam->bulk_bufs[i].index = i;
      plucam->bulk_bufs[i].submitted = 0;
#if MEM_IN_BULK_BUF
      plucam->bulk_bufs[i].data = usb_alloc_coherent(plucam->udev, PAGE_SIZE, GFP_KERNEL,  &urb->transfer_dma);
      if (plucam->bulk_bufs[i].data == NULL)
      {
         Err(plucam, "***Failed to usb_buffer_alloc memory %d \n", i);
         ret = -ENOMEM;
         break;
      }
      if ((unsigned long)plucam->bulk_bufs[i].data & (PAGE_SIZE-1))
      {
         Err(plucam, "***Failed to align memory %d, addr:0x%p\n", i, plucam->bulk_bufs[i].data);
         kfree(plucam->bulk_bufs[i].data);
         plucam->bulk_bufs[i].data = NULL;
         ret = -ENOMEM;
         break;
      }
      urb->transfer_flags = URB_NO_TRANSFER_DMA_MAP;
#endif
   }
   if (i == count)
   {
      ret = 0;
   }
   else
   {
      plucam->bulk_bufs_used = i;
      lucam_dealloc_bulkbuf(plucam);
   }
   return ret;
}

//--------------------------------------------------------------------------------
//
static void lucam_dealloc_imagebuf(plucam_device plucam)
{
   //int i;
   Info(plucam, TRACE_MEMORY, "lucam_dealloc_imagebuf() called\n");
   // FIXME: how does videobuff do this? 
   Info(plucam, TRACE_MEMORY, "lucam_dealloc_imagebuf() finished\n");
}


#if ALLOW_RGB24_OUTPUT
//---------------------------------------------------------------------------
//
static int lucam_alloc_one_raw_buf_data(plucam_device plucam, plucam_image_buf pimage_buf)
{
   int alloc_size;
   int ret;

   Info(plucam, TRACE_MEMORY, "lucam_alloc_one_raw_buf_data() called \n");
   alloc_size = (plucam->framesize + PAGE_SIZE-1) & ~(PAGE_SIZE-1);

   pimage_buf->raw_alloc_data = vmalloc_32(alloc_size);
   if (pimage_buf->raw_alloc_data == NULL)
   {
      Err(plucam, "Failed to allocate frame buffer size=%d \n", plucam->framesize);
      ret = - ENOMEM;
   }
   else
   {
      pimage_buf->raw_data = (typeof(pimage_buf->raw_data))PAGE_ALIGN((unsigned long)pimage_buf->raw_alloc_data);
      ret = 0;
   }
   return ret;
}
#endif

//-------------------------------------------------------------------------------
static int lucam_alloc_resource(plucam_device plucam)
{
   int ret;

   Info(plucam, TRACE_MEMORY, "lucam_alloc_resource() called %d %p %d\n",
        (int)PAGE_SIZE, plucam, plucam ? plucam->stream_count: -1);

   if(plucam==NULL) return -EFAULT;
   if (plucam->stream_count == 0) return -EINVAL;

   // Stuff to do:
   // 1. Alloc all resources
   // 2. Set alt setting

   // Part 1: Alloc all resources
   lucam_set_framesize(plucam);

   ret = lucam_alloc_bulkbuf(plucam);
   if(ret < 0) return ret;
   ret = lucam_alloc_framebuf(plucam);
   if(ret < 0)
   {
      lucam_dealloc_bulkbuf(plucam);
      return ret;
   }

#if !USB3_ENDPOINT_CONFIGURATION
   // 2. Setup alt setting
   if (plucam->alt_setting != DATA_ALT_SETTING)
   {
      ret = lucam_select_alt_setting (plucam, DATA_ALT_SETTING);
      if (ret < 0)
      {
         lucam_dealloc_bulkbuf(plucam);
         lucam_dealloc_framebuf(plucam);
         return -EFAULT;
      }
      plucam->alt_setting = DATA_ALT_SETTING;
   }
#endif

   if (ret >= 0)
   {
      plucam->current_stream_state = LUCAM_STREAM_STATE_PAUSE;
   }
   Info(plucam, TRACE_MEMORY, "lucam_alloc_resource() finished %d \n", ret);
   return ret;
}


//---------------------------------------------------------------------------
//
static void lucam_dealloc_resource(plucam_device plucam)
{
   Info(plucam, TRACE_MEMORY, "lucam_dealloc_resource() called \n");
   if(plucam==NULL) return;

#if !USB3_ENDPOINT_CONFIGURATION
   if (plucam->alt_setting != IDLE_ALT_SETTING)
   {
      lucam_select_alt_setting (plucam, IDLE_ALT_SETTING);
      plucam->alt_setting = IDLE_ALT_SETTING;
   }
#endif

   lucam_dealloc_bulkbuf(plucam);
   lucam_dealloc_framebuf(plucam);

   plucam->current_stream_state = LUCAM_STREAM_STATE_STOP;
}


/****************************************************************************
* Our taskss
*
*
*
****************************************************************************/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
void lucam_change_stream_state(struct work_struct *context)
#else
void lucam_change_stream_state(void *context)
#endif
{
   plucam_device plucam;
   int ret;

   if (NULL == context)
      return;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
   plucam = (plucam_device)((unsigned long)context - (unsigned long)&(((plucam_device)0)->change_state_task));
#else
   plucam = (plucam_device)context;
#endif

   if (NULL == plucam) 
   {
      Err(plucam, "***lucam_change_stream_state: NULL == plucam\n");
      return;
   }

   if (plucam->vdev)
   {
      Info(plucam, TRACE_MEMORY,
           "lucam_change_stream_state called, wanted=%d, current=%d, frame_error=%d\n",
           plucam->wanted_stream_state, plucam->current_stream_state, plucam->frame_error);
   }
   else
   {
      printk(KERN_INFO
           "***lucam_change_stream_state called (no vdev), wanted=%d, current=%d, frame_error=%d\n",
           plucam->wanted_stream_state, plucam->current_stream_state, plucam->frame_error);
   }

   while(plucam->current_stream_state != plucam->wanted_stream_state || plucam->frame_error)
   {
      if (down_trylock(&plucam->change_state_task_mutex))
      {
         Info(plucam, TRACE_MEMORY, " lucam_change_stream_state already running\n");
         break;
      }
      if (plucam->current_stream_state == LUCAM_STREAM_STATE_RUN && plucam->wanted_stream_state != LUCAM_STREAM_STATE_RUN)
      {
         lucam_stop_stream(plucam);

         // Flush
         lucam_flush_image_buf_q(plucam);
      }
      else if (plucam->frame_error)
      {
         if (plucam->current_stream_state == LUCAM_STREAM_STATE_RUN)
         {
            if (plucam->frame_error >= FRAME_ERROR_REQUIRE_RESTART)
            {
               lucam_stop_stream(plucam);
            }
#if ENHANCED_BUFFERING
            else
            {
               ret = lucam_kick_stream(plucam);
               if (ret < 0)
               {
                  Err(plucam, "***Failed to kick stream\n");
                  plucam->wanted_stream_state = LUCAM_STREAM_STATE_PAUSE;
                  lucam_flush_image_buf_q(plucam);
               }
            }
#endif
         }
         else
         {
#if ENHANCED_BUFFERING
             // this can happen if buffers are MCAPTUREd with the
             // stream paused or stopped
#else
            // This is not supposed to happen
            Err(plucam, "***Unreacheable line in change_stream_state task reached\n");
#endif
            plucam->frame_error = 0;
         }
      }
      else
      {
         if (plucam->current_stream_state == LUCAM_STREAM_STATE_STOP)
         {
            if (plucam->wanted_stream_state > plucam->current_stream_state)
            {
               ret = lucam_alloc_resource(plucam);
               if (ret < 0)
               {
                  Err(plucam, "***Failed to alloc resources for stream %d\n", ret);
                  if (plucam->wanted_stream_state == LUCAM_STREAM_STATE_RUN)
                  {
                     plucam->wanted_stream_state = LUCAM_STREAM_STATE_STOP;
                     lucam_flush_image_buf_q(plucam);
                  }
                  else
                  {
                     plucam->wanted_stream_state = LUCAM_STREAM_STATE_STOP;
                  }
               }
            }
         }
         else if (plucam->current_stream_state == LUCAM_STREAM_STATE_PAUSE)
         {
            if (plucam->wanted_stream_state > plucam->current_stream_state)
            {
               // We have to start
               ret = lucam_start_stream(plucam);
               if (ret < 0)
               {
                  Err(plucam, "***Failed to start stream\n");
                  plucam->wanted_stream_state = LUCAM_STREAM_STATE_PAUSE;
                  lucam_flush_image_buf_q(plucam);
               }
            }
            else if (plucam->wanted_stream_state < plucam->current_stream_state)
            {
               // Flush
               lucam_flush_image_buf_q(plucam);
               lucam_dealloc_resource(plucam);
            }
         }
      }

      up(&plucam->change_state_task_mutex);
   }

   Info(plucam, TRACE_MEMORY, "lucam_change_stream_state leaving\n");

   wake_up(&plucam->state_changed_queue);
}


/************************************************************************************
************************************************************************************/
static int lucam_set_stream_state(plucam_device plucam, int state)
{
   int ret;

   plucam->wanted_stream_state = state;

   schedule_work(&plucam->change_state_task);

   if (state < plucam->current_stream_state)
   {
      wait_event(plucam->state_changed_queue, (plucam->current_stream_state == plucam->wanted_stream_state && !plucam->frame_error));
   }
   else
   {
      ret = wait_event_interruptible(plucam->state_changed_queue, (plucam->current_stream_state == plucam->wanted_stream_state && !plucam->frame_error));

      if (ret < 0)
      {
         Err(plucam, "***lucam_set_stream_state: Failed to wait for scheduled work w %d\n", ret);
         return ret;
      }
   }


   if (plucam->wanted_stream_state < state)
   {
      Err(plucam, "***lucam_set_stream_state: Failed to reach wanted state\n");
      return -EFAULT;
   }

   return 0;
}


/* ------------------------------------------------------------------
    Videobuf operations
   ------------------------------------------------------------------*/
static int
lucam_buffer_setup(struct videobuf_queue *vq, unsigned int *count, unsigned int *size)
{
    plucam_device dev = vq->priv_data;

   *size = lucam_get_framesizemax(dev);

    if (*count < 2)
        *count = 2;
/* TODO: limit RAM usage from the install parameters */
/*  while (*size * *count > vid_limit * 1024 * 1024) */
/*      (*count)--; */

    Info(dev, TRACE_MEMORY, "%s, count=%d, size=%d\n", __func__,
        *count, *size);

    return 0;
}

static void lucam_free_buffer(struct videobuf_queue *q, struct _image_buf *buf)
{
   int ret;
    plucam_device dev __attribute__((unused)) = q->priv_data;

    Info(dev, TRACE_MEMORY | TRACE_V4L2, "lucam_free_buffer, state: %i\n", buf->vb.state);

    BUG_ON(in_interrupt());

#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,37)
    ret = videobuf_waiton(q, &buf->vb, 0 /* non_blocking*/, 0 /* intr */);
#else
    ret = videobuf_waiton(&buf->vb, 0 /* non_blocking*/, 0 /* intr */);
#endif

#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,24)
#if USE_VMALLOC32
    videobuf_vmalloc_32_free(&buf->vb);
#else
    videobuf_vmalloc_free(&buf->vb); // does not block
#endif
#else
    videobuf_dma_free(&buf->vb.dma);
#endif

#if ALLOW_RGB24_OUTPUT
    if(buf->raw_alloc_data) {
        vfree(buf->raw_alloc_data);
        buf->raw_alloc_data = NULL;
        buf->raw_data = NULL;
    }
#endif

    Info(dev, TRACE_MEMORY, "lucam_free_buffer: freed\n");
    buf->vb.state = VIDEOBUF_NEEDS_INIT;
}

/* Locking: Caller holds vq->vb_lock. */
static int lucam_buffer_prepare(struct videobuf_queue *vq,
              struct videobuf_buffer *vb,
              enum v4l2_field field)
{
   struct _lucam_device *dev = vq->priv_data;
   struct _image_buf *buf = container_of(vb, struct _image_buf, vb);
   int rc;

   Info(dev, TRACE_V4L2 | TRACE_MEMORY, "%s, field=%d\n", __func__, field);

   buf->plucam = dev;

   lucam_set_framesize(dev);

   buf->vb.size = dev->framesize;
   if (0 != buf->vb.baddr  &&  buf->vb.bsize < buf->vb.size)
      return -EINVAL;

   /* These properties only change when queue is idle, see s_fmt */
   buf->vb.width  = dev->outwidth;
   buf->vb.height  = dev->outheight;
   buf->vb.field  = 0 /* NONE? */;

   if (VIDEOBUF_NEEDS_INIT == buf->vb.state) {
      rc = videobuf_iolock(vq, &buf->vb, NULL);
      if (rc < 0)
         goto fail;
   }

   buf->vb.state = VIDEOBUF_PREPARED;

   return 0;

fail:
   lucam_free_buffer(vq, buf);
   return rc;
}


/* Locking: Caller holds vq->irqlock (A.K.A. dev->slock) and vq->vb_lock. */
static void lucam_buffer_queue(struct videobuf_queue *vq, struct videobuf_buffer *vb)
{
    struct _image_buf    *buf  = container_of(vb, struct _image_buf, vb);
    struct _lucam_device *dev  = vq->priv_data;

    Info(dev, TRACE_V4L2 | TRACE_MEMORY, "lucam_buffer_queue\n");
    
    assert_spin_locked(&dev->slock);
    BUG_ON(!vq);
#if ALLOW_RGB24_OUTPUT
//#warning This code has not been integrated fully
    buf->needs_conversion = (dev->framesize != dev->imagesize);
    if (buf->needs_conversion && buf->raw_alloc_data == NULL) {
        int ret = lucam_alloc_one_raw_buf_data(dev, buf);
        
        if (ret < 0) {
            Err(dev, "Failed to allocate one framebuf\n");
            return;
        }
    }
#endif
    buf->offset = 0;
#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,35)
    buf->data = videobuf_queue_to_vaddr(vq, vb);
#elif VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,26)
    buf->data = videobuf_queue_to_vmalloc(vq, vb);
#elif VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,24)
    buf->data = videobuf_to_vmalloc_32(vb);
#else
    buf->data = vb->dma.vmalloc;
#endif
    buf->vb.state = VIDEOBUF_QUEUED;

    list_add_tail(&buf->vb.queue, &dev->active_frames);
}

/* Locking: Caller holds vq->vb_lock. */
static void lucam_buffer_release(struct videobuf_queue *vq,
               struct videobuf_buffer *vb)
{
    struct _image_buf   *buf = container_of(vb, struct _image_buf, vb);
    struct _lucam_device *plucam = vq->priv_data;

    Info(plucam, TRACE_V4L2 | TRACE_MEMORY, "lucam_buffer_release\n");

   /*
      We are required to stop any io on the buffer.
   */
    if (plucam->wanted_stream_state > LUCAM_STREAM_STATE_STOP)
        lucam_set_stream_state(plucam, LUCAM_STREAM_STATE_STOP);

    lucam_free_buffer(vq, buf);
}

static struct videobuf_queue_ops lucam_video_qops = {
    .buf_setup      = lucam_buffer_setup,
    .buf_prepare    = lucam_buffer_prepare,
    .buf_queue      = lucam_buffer_queue,
    .buf_release    = lucam_buffer_release,
};

static void lucam_init_camera_extension(plucam_device plucam)
{
   memset(plucam, 0, sizeof (lucam_device)); // inline on i386
   init_waitqueue_head (&plucam->remove_ok); // inline on i386

   //init_MUTEX (&plucam->imagebuf_mutex);

   init_waitqueue_head (&plucam->state_changed_queue); // inline on i386

   spin_lock_init(&plucam->slock);
   sema_init (&plucam->mutex, 1);        // inline
   sema_init (&plucam->control_mutex, 1); // inline

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
   INIT_WORK(&plucam->change_state_task, lucam_change_stream_state);
#else
   INIT_WORK(&plucam->change_state_task, lucam_change_stream_state, plucam);
#endif
   sema_init (&plucam->change_state_task_mutex, 1);        // inline

   INIT_LIST_HEAD(&plucam->active_frames);

#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,37)
   mutex_init(&plucam->vb_vidq_ext_mutex);
#endif

#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,24)
#if USE_VMALLOC32
   videobuf_queue_vmalloc_32_init(&plucam->vb_vidq, &lucam_video_qops,
                NULL, &plucam->slock,
                V4L2_BUF_TYPE_VIDEO_CAPTURE, 
                V4L2_FIELD_NONE,
                sizeof(struct _image_buf), plucam
#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,37)
         , &plucam->vb_vidq_ext_mutex
#endif
      );
#else
   videobuf_queue_vmalloc_init(&plucam->vb_vidq, &lucam_video_qops,
                NULL, &plucam->slock,
                V4L2_BUF_TYPE_VIDEO_CAPTURE, 
                V4L2_FIELD_NONE,
                sizeof(struct _image_buf), plucam
#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,37)
         , &plucam->vb_vidq_ext_mutex
#endif
      );
#endif
#else
   videobuf_queue_init(&plucam->vb_vidq, &lucam_video_qops,
                NULL, &plucam->slock,
                V4L2_BUF_TYPE_VIDEO_CAPTURE, 
                V4L2_FIELD_NONE,
                sizeof(struct _image_buf), plucam
#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,37)
         , &plucam->vb_vidq_ext_mutex
#endif
      );
#endif

   init_waitqueue_head (&plucam->event_queue);

   plucam->current_stream_state = LUCAM_STREAM_STATE_STOP;
   plucam->wanted_stream_state = LUCAM_STREAM_STATE_STOP;


#if ENHANCED_BUFFERING
   atomic_set(&plucam->bulk_buf_idle_count, 0);
   plucam->bulk_buf_idle_first = 0;
#endif
#if !ENHANCED_BUFFERING
   plucam->blank_image_buf.plucam = plucam;
   plucam->blank_image_buf.vb.i = -1;
#endif

   plucam->camera_mode = LUCAM_VIDEO_MODE;
   plucam->unplugged = 0;
   plucam->vdev = NULL;

   plucam->file_that_set_fmt = NULL;
   plucam->file_holding_eeprom_lock = NULL;
   plucam->eeprom_lock_count = 0;

   plucam->timeout = -1; // infinite

#ifdef DEBUG
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
   plucam->debug = (lucam_template.debug & ~0x01);
#else
   plucam->debug = (lucam_template.dev_debug & ~0x01);
#endif
#endif
}

/************************************************************************************
*
***********************************************************************************/
static plucam_device lucam_find_and_init_camera(struct usb_device *udev, struct usb_interface *intf)
{
   int i;
   plucam_device plucam;
   for (i = 0; i < MAX_CAMERA_NUMBER; i++)
   {
      plucam = &lucam_dev[i];
      if (!plucam->udev)
      {
         break;
      }
   }

   if (i < MAX_CAMERA_NUMBER && plucam)
   {
      lucam_init_camera_extension(plucam);

#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(3,11,0)
      if (v4l2_device_register(&intf->dev, &plucam->v4l2_dev))
      {
         return NULL;
      }
#endif
        
      plucam->index = i;
      plucam->udev = udev;

      return plucam;
   }

   return NULL;
}

#if ALLOW_RGB24_OUTPUT
/****************************************************************************
 *
 *  Color conversion routine raw data -> rgb24 (may move to application lev)
 *
 ***************************************************************************/
static int lucam_fill_image_buf(plucam_device plucam, plucam_image_buf pimage_buf)
{
    __u32 colorid;
    BYTE* data;
    BYTE* buf;
    int width, height;

    //Info(plucam, TRACE_INFO, "lucam_fill_image_buf() called \n");

    if(plucam==NULL || pimage_buf == NULL) return -EFAULT;
    buf = pimage_buf->data;
    if(buf == NULL) return -EFAULT;

    if(plucam->camera_mode == LUCAM_VIDEO_MODE)
    {
        width=lucam_fo_get_width(plucam) / (lucam_fo_get_subsampling_w(plucam) & 0xff);
        height=lucam_fo_get_height(plucam) / (lucam_fo_get_subsampling_h(plucam) & 0xff);
        colorid=lucam_fo_get_colorid(plucam);
    }
    else
    {
        width=lucam_st_get_width(plucam) / (lucam_st_get_subsampling_w(plucam) & 0xff);
        height=lucam_st_get_height(plucam) / (lucam_st_get_subsampling_h(plucam) & 0xff);
        colorid=lucam_st_get_colorid(plucam);
    }

    data=pimage_buf->raw_data;
    //MONO 16
    switch (colorid)
    {
        case LUCAM_COLOR_MONO16:
            mono16_to_rgb8(buf,data,width,height);
            break;
        case LUCAM_COLOR_MONO61:
            mono61_to_rgb8(buf,data,width,height);
            break;
        case LUCAM_COLOR_MONO8:
            mono8_to_rgb8(buf,data,width,height);
            break;
        case LUCAM_COLOR_BAYER_RGGB:
            lucam_bayer_to_rgb8(buf,data,width,height,RED);
            break;
        case LUCAM_COLOR_BAYER_GRBG:
            lucam_bayer_to_rgb8(buf,data,width,height,GREEN1);
            break;
        case LUCAM_COLOR_BAYER_GBRG:
            lucam_bayer_to_rgb8(buf,data,width,height,GREEN2);
            break;
        case LUCAM_COLOR_BAYER_BGGR:
            lucam_bayer_to_rgb8(buf,data,width,height,BLUE);
            break;
        default:

            break;
    }
    return 1;
}
#endif


/* --- controls ---------------------------------------------- */
static int vidioc_queryctrl(struct file *file, void *priv,
                struct v4l2_queryctrl *qc)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(lucam_qctrl); i++)
        if (qc->id && qc->id == lucam_qctrl[i].id) {
            memcpy(qc, &(lucam_qctrl[i]),
                sizeof(*qc));
            return (0);
        }

    return -EINVAL;
}

static int vidioc_g_ctrl(struct file *file, void *priv,
             struct v4l2_control *ctrl)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(lucam_qctrl); i++)
        if (ctrl->id == lucam_qctrl[i].id) {
            ctrl->value = qctl_regs[i];
            return (0);
        }

    return -EINVAL;
}
static int vidioc_s_ctrl(struct file *file, void *priv,
                struct v4l2_control *ctrl)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(lucam_qctrl); i++)
        if (ctrl->id == lucam_qctrl[i].id) {
            if (ctrl->value < lucam_qctrl[i].minimum
                || ctrl->value > lucam_qctrl[i].maximum) {
                    return (-ERANGE);
                }
            qctl_regs[i] = ctrl->value;
            return (0);
        }
    return -EINVAL;
}


/****************************************************************************
 *
 *  file operation routinues
 *
 ***************************************************************************/
#if  VIDEODEV_VERSION_CODE <= KERNEL_VERSION(2,6,28)
static int lucam_video_open(struct inode *inode, struct file *file)
#else
static int lucam_video_open(struct file *file)
#endif
{
   struct video_device *vdev = video_devdata(file);
   struct lucam_fh *fh = NULL;
   plucam_device plucam=(plucam_device)video_get_drvdata(vdev);
   int ret=0;

   if(!plucam || !plucam->udev) return -EINVAL;

   if (down_interruptible(&plucam->mutex)) {
      return -ERESTARTSYS;
   }

   if (plucam->unplugged) {
      ret = -EINVAL;
      goto finish;
   }

   /* initiallize the per-filehandle state */
   fh = kzalloc(sizeof(*fh), GFP_KERNEL);
   if (NULL == fh) {
      ret = -ENOMEM;
      goto finish;
   }

   file->private_data = fh;
   fh->dev      = plucam;
   fh->type     = V4L2_BUF_TYPE_VIDEO_CAPTURE;

   plucam->opencount++;

   Info(plucam, TRACE_OPEN,
      "Open called, file=0x%8p, flags=0x%08x, opencount=%d, under control=%s\n",
      file, file->f_flags & O_NOIO, plucam->opencount ,
      plucam->file_that_set_fmt ? "yes": "no");


finish:
   up (&plucam->mutex);
   Info(plucam, TRACE_OPEN, "Result of OPEN %d\n", ret);
   if(ret>0) return 0;

   return ret;
}


/****************************************************************************************
* lucam_video_close
****************************************************************************************/
#if  VIDEODEV_VERSION_CODE <= KERNEL_VERSION(2,6,28)
static int lucam_video_close(struct inode *inode, struct file *file)
#else
static int lucam_video_close(struct file *file)
#endif
{
   struct video_device *vdev = video_devdata(file);
   plucam_device plucam=(plucam_device)video_get_drvdata(vdev);
   int destroy_camera;

   Info(plucam, TRACE_OPEN, "lucam_video_close() called, file=0x%p, sp=%d\n", file, signal_pending(current));
   if(!plucam || !plucam->udev)
   {
      Err(plucam, "***lucam_video_close: no device\n");
      return -1;
   }

   if (down_interruptible(&plucam->mutex))
   {
      Err(plucam, "***lucam_video_close: -ERESTARTSYS\n");
      return -ERESTARTSYS;
   }

   if (file == plucam->file_that_set_fmt)
   {
      if (plucam->wanted_stream_state != LUCAM_STREAM_STATE_STOP || 
          plucam->current_stream_state != LUCAM_STREAM_STATE_STOP)
      {
         lucam_set_stream_state(plucam, LUCAM_STREAM_STATE_STOP);
      }

#if  VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,24)
      videobuf_stop(&plucam->vb_vidq);
#else
      videobuf_streamoff(&plucam->vb_vidq);
      videobuf_read_stop(&plucam->vb_vidq);
#endif
      videobuf_mmap_free(&plucam->vb_vidq);
      lucam_dealloc_imagebuf(plucam);
      if(!list_empty(&plucam->vb_vidq.stream))
      {
         Err(plucam,"***lucam_video_close: device queue not empty\n");
         list_del_init(&plucam->vb_vidq.stream);
      }

      plucam->file_that_set_fmt = NULL;
   }

   if (file == plucam->file_holding_eeprom_lock)
   {
      plucam->file_holding_eeprom_lock = NULL;
      plucam->eeprom_lock_count = 0;
   }

   plucam->opencount--;
   if(!plucam->unplugged)
   {
      // lucam_set_idle(plucam->udev);
   }
   else
   {
      wake_up(&plucam->remove_ok);
   }

   destroy_camera=(plucam->unplugged && (plucam->opencount==0));
   if (destroy_camera)
   {
      lucam_destroy_camera(plucam);
   }
   up(&plucam->mutex);

   //  MOD_DEC_USE_COUNT; //disable in the stage of development
   Info(plucam, TRACE_OPEN, "lucam_video_close() finished \n");
   return 0;
}

/*************************************************************************************
* Fn: lucam_video_read
*************************************************************************************/
static ssize_t lucam_video_read(struct file *file, char __user *buffer, size_t len, loff_t *ppos)
{
    struct video_device *vdev = video_devdata(file);
    plucam_device plucam=(plucam_device)video_get_drvdata(vdev);
    ssize_t ret;

    Info(plucam, TRACE_READ, "%s: called for %d bytes\n",
         __func__, (int)len);

    if(!plucam || !plucam->udev)
    {
        Err(plucam, "***lucam_video_read: no device\n");
        return -EFAULT;
    }
    if (plucam->unplugged)
    {
        Err(plucam, "***lucam_video_read: Device unplugged\n");
        return -ENOTCONN;                            /* unplugged device! */
    }

    if (plucam->file_that_set_fmt != NULL &&
        plucam->file_that_set_fmt != file) {
        Info(plucam, TRACE_READ, "%s: Another file holds the format/stream lock\n", __func__);
        return -EBUSY;
    }
    plucam->file_that_set_fmt = file;

    if (plucam->current_stream_state == LUCAM_STREAM_STATE_STOP) {
        // this makes sure that the frame size member is valid
        lucam_set_framesize(plucam);
    }

#if ENHANCED_BUFFERING
    if(plucam->wanted_stream_state != LUCAM_STREAM_STATE_RUN ||
       atomic_read(&plucam->bulk_buf_idle_count) == plucam->bulk_bufs_used) {
        plucam->frame_error = 1;
#else
    if(plucam->wanted_stream_state != LUCAM_STREAM_STATE_RUN) {
#endif
        //start stream if necessary
        ret = lucam_set_stream_state(plucam, LUCAM_STREAM_STATE_RUN);

        if (ret < 0) {
            Err(plucam, "***Failed to set stream state %zd\n", ret);
            return ret;
        }
    }

    if (plucam->camera_mode == LUCAM_VIDEO_MODE) {
        ret = videobuf_read_stream(&plucam->vb_vidq, buffer, len, ppos, 0,
                        file->f_flags & O_NONBLOCK);
    } else {
        ret = videobuf_read_one(&plucam->vb_vidq, buffer, len, ppos,
                        file->f_flags & O_NONBLOCK);
    }
    
#if ALLOW_RGB24_OUTPUT
#warning transformations are not well thought out yet
#endif

    return ret;

}


//-----------------------------------------------------------------------------------
static unsigned int lucam_video_poll(struct file *file, poll_table *wait)
{
   struct lucam_fh        *fh = file->private_data;
   struct _lucam_device   *dev = fh->dev;
   int ret, other_ret;

   Info(dev, TRACE_POLL, "lucam_video_poll\n");

   /*
   * For some reason, videobuf_poll_stream will try to start
   * reading if streaming is off.
   */
   ret = videobuf_poll_stream(file, &dev->vb_vidq, wait);
   Info(dev, TRACE_POLL, " videobuf_poll_stream returned %d\n", ret);

   if (dev->vb_vidq.reading) 
   {
      // reading, so make sure images are being captured
#if ENHANCED_BUFFERING
      if(dev->wanted_stream_state != LUCAM_STREAM_STATE_RUN ||
         atomic_read(&dev->bulk_buf_idle_count) == dev->bulk_bufs_used) 
      {
         dev->frame_error = 1;
#else
      if(dev->wanted_stream_state != LUCAM_STREAM_STATE_RUN) 
      {
#endif
         Info(dev, TRACE_POLL, "lucam_video_poll: vidq->reading set, starting stream\n");

         //start stream
         other_ret = lucam_set_stream_state(dev, LUCAM_STREAM_STATE_RUN);
         if (other_ret < 0) {
            Err(dev, "***Failed to set stream state %d\n", ret);
            ret = other_ret;
         }
      }
   }

   Info(dev, TRACE_POLL, "polled (str:%d, rd:%d, e:%d) -> %d\n",
        dev->vb_vidq.streaming, dev->vb_vidq.reading,
        list_empty(&dev->vb_vidq.stream), ret);
   return ret;
}

/* ------------------------------------------------------------------
    IOCTL vidioc handling
   ------------------------------------------------------------------*/
static int vidioc_querycap(struct file *file, void  *priv,
                    struct v4l2_capability *cap)
{
    strcpy(cap->driver, LUCAM_NAME);
    strcpy(cap->card, KBUILD_MODNAME);
    strcpy(cap->bus_info, "usb");
// FIXME: udev is not always right here:    strlcpy(cap->bus_info, fh->dev->udev->dev.bus_id, sizeof(cap->bus_info));

    cap->version = LUCAM_DRIVER_VERSION;

    cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
                V4L2_CAP_STREAMING     |
#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(3,19,0)
                V4L2_CAP_DEVICE_CAPS   |
#endif
                V4L2_CAP_READWRITE;

#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(3,19,0)
                cap->device_caps = V4L2_CAP_VIDEO_CAPTURE;
#endif
    return 0;
}

static struct v4l2_fmtdesc lucam_supported_formats[] = {
#if ALLOW_RGB24_OUTPUT
    {
        .description = "RGB24",
        .pixelformat = V4L2_PIX_FMT_RGB24,
    },
    {
        .description = "BGR24",
        .pixelformat = V4L2_PIX_FMT_BGR24,
    },
#endif
    {
        .description = "8 BGBG.. GRGR.. */ ",
        .pixelformat = V4L2_PIX_FMT_SBGGR8,
    },
    {
        .description = "8 RGRG.. GBGB.. */ ",
        .pixelformat = V4L2_PIX_FMT_SRGGB8,
    },
    {
        .description = "8 GRGR.. BGBG.. */ ",
        .pixelformat = V4L2_PIX_FMT_SGRBG8,
    },
    {
        .description = "8 GBGB.. RGRG.. */ ",
        .pixelformat = V4L2_PIX_FMT_SGBRG8,
    },
    {
        .description = "8 bit Greyscale",
        .pixelformat = V4L2_PIX_FMT_GREY,
    },
    {
        .description = "16 bit Bayer BGBG.. GRGR..",
        .pixelformat = V4L2_PIX_FMT_SBGGR16,
    },
    {
        .description = "16 RGRG.. GBGB.. */ ",
        .pixelformat = V4L2_PIX_FMT_SRGGB16,
    },
    {
        .description = "16 GRGR.. BGBG.. */ ",
        .pixelformat = V4L2_PIX_FMT_SGRBG16,
    },
    {
        .description = "16 GBGB.. RGRG.. */ ",
        .pixelformat = V4L2_PIX_FMT_SGBRG16,
    },
    {
        .description = "Big Endian 16 BGBG.. GRGR..",
        .pixelformat = V4L2_PIX_FMT_SBGGR61,
    },
    {
        .description = "Big Endian 16 RGRG.. GBGB..",
        .pixelformat = V4L2_PIX_FMT_SRGGB61,
    },
    {
        .description = "Big Endian 16 GRGR.. BGBG..",
        .pixelformat = V4L2_PIX_FMT_SGRBG61,
    },
    {
        .description = "Big Endian 16 GBGB.. RGRG..",
        .pixelformat = V4L2_PIX_FMT_SGBRG61,
    },
    {
        .description = "16 bit Greyscale",
        .pixelformat = V4L2_PIX_FMT_Y16,
    },
    {
        .description = "Big Endian 16 bit Greyscale",
        .pixelformat = V4L2_PIX_FMT_Y61,
    },
};

static int vidioc_enum_fmt_vid_cap(struct file *file, void  *priv,
                    struct v4l2_fmtdesc *f)
{
    struct v4l2_fmtdesc *myf;

    if (f->index >= ARRAY_SIZE(lucam_supported_formats))
        return -EINVAL;

    myf = &lucam_supported_formats[f->index];

    strlcpy(f->description, myf->description, sizeof(f->description));
    f->pixelformat = myf->pixelformat;
    
    return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
                    struct v4l2_format *f)
{
    struct lucam_fh *fh = priv;
    struct _lucam_device *dev = fh->dev;

    lucam_fill_v4l2_format(dev, f);

    return (0);
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
   struct lucam_fh  *fh  = priv;
   struct _lucam_device *dev = fh->dev;
   enum v4l2_field field;
   struct v4l2_crop shadow_crop;
   int i;
   int bitcount;

   if (0 == f->fmt.pix.pixelformat /* 0 implies a format
                     * v4l1-compat did not
                     * recognize when translating
                     * to v4l2 */ ) 
   {
      // Stick with the current pixelformat
      f->fmt.pix.pixelformat = dev->pixelformat;
   }

   if (f->fmt.pix.pixelformat <= LUCAM_COLOR_XENA61_MYYC &&
      f->fmt.pix.pixelformat == f->fmt.pix.priv ) 
   {
      /* A lucam color_id got through via our use of private */
      __u32 colorid = f->fmt.pix.pixelformat;
      pixelformat_from_colorid(dev, colorid, &bitcount, &f->fmt.pix.pixelformat);
      Info(dev, TRACE_INFO, "**%s: colorid %u converted to pixelformat 0x%08x.\n",
          __func__, colorid, f->fmt.pix.pixelformat);
   }

   for(i = 0; i < ARRAY_SIZE(lucam_supported_formats); ++i)
   {
      if (lucam_supported_formats[i].pixelformat == f->fmt.pix.pixelformat)
         break;
   }

   if (i >= ARRAY_SIZE(lucam_supported_formats)) 
   {
      Err(dev, "***Fourcc format (0x%08x) not supported.\n", f->fmt.pix.pixelformat);
      return -EINVAL;
   }

   field = f->fmt.pix.field;
   switch (f->fmt.pix.field) {
   case V4L2_FIELD_ANY:
   case V4L2_FIELD_INTERLACED: /* fake it for gstreamer and
                     * friends since they only ask for
                     * interlaced. How can they know
                     * the difference? */
      field = V4L2_FIELD_NONE;
      break;
   case V4L2_FIELD_NONE:
      break;
   default:
      Info(dev, TRACE_ERROR, "***Field type invalid.\n");
      return -EINVAL;
   }
   f->fmt.pix.field = field;

   lucam_fill_v4l2_crop(dev, &shadow_crop);

   f->fmt.pix.height = CLAMP(f->fmt.pix.height,
                 lucam_get_height_unit(dev), lucam_get_height_max(dev));
   f->fmt.pix.width = CLAMP(f->fmt.pix.width,
                lucam_get_width_unit(dev), lucam_get_width_max(dev));

   lucam_try_framesize(dev, f, &shadow_crop, NULL, NULL, NULL, NULL, NULL);

   return 0;
}

//------------------------------------------------------------------------
//
static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
                    struct v4l2_format *f)
{
   struct lucam_fh  *fh = priv;
   struct _lucam_device *dev = fh->dev;
   unsigned int colorid;
   int bitcount;
   size_t subw, subh;
   int ret;

   Info(dev, TRACE_INFO, "vidioc_s_fmt_vid_cap\n");

   if (down_interruptible(&dev->mutex)) {
      return -ERESTARTSYS;
   }

   if (dev->file_that_set_fmt != NULL &&
        dev->file_that_set_fmt != file) 
   {
      Info(dev, TRACE_INFO, "%s: Another file holds the format/stream lock\n", 
             __func__);
      ret = -EBUSY;
      goto finish;
   }
   dev->file_that_set_fmt = file;

#if 0 // we may want to mmap on open
   if (videobuf_queue_is_busy(&dev->vb_vidq)) 
   {
      int i;
      Info(dev, TRACE_INFO, "%s: Queue is busy.\n", __func__);
      if (dev->vb_vidq.reading)
      {
         Info(dev, TRACE_INFO, "***reading\n");
      }
      if (dev->vb_vidq.streaming)
      {
         Info(dev, TRACE_INFO, "***streaming\n");
      }
      if (dev->vb_vidq.read_buf)
      {
         Info(dev, TRACE_INFO, "***read_buf\n");
      }
      for (i = 0 ; i < VIDEO_MAX_FRAME ; i++)
      {
         if (dev->vb_vidq.bufs[i] == NULL) continue;
         if (dev->vb_vidq.bufs[i]->map)
         {
            Info(dev, TRACE_INFO, "***buf # %d mapped\n", i);
         }
         if (dev->vb_vidq.bufs[i]->state == VIDEOBUF_ACTIVE)
         {
            Info(dev, TRACE_INFO, "***buf # %d ACTIVE\n", i);
         }
         if (dev->vb_vidq.bufs[i]->state == VIDEOBUF_QUEUED)
         {
            Info(dev, TRACE_INFO, "***buf # %d QUEUED\n", i);
         }
      }
      ret = -EBUSY;
      goto finish;
   }
#endif

#if 0
   ret = vidioc_try_fmt_vid_cap(file, fh, f);
   if (ret < 0)
      goto finish;

   lucam_fill_v4l2_crop(dev, &shadow_crop);

   ret = lucam_try_framesize(dev, f, &shadow_crop, &colorid,
                 &framesize, &imagesize, &bitcount, &sub_w, &sub_h);
   if (ret)
   {
      Err(dev, "Inconsistent results from vidioc_try_fmt_vid_cap and lucam_try_framesize: %d",
          ret);
      goto finish;
   }

   if (dev->framesize == framesize && dev->imagesize == imagesize && /* covers changes to sub_w and sub_h */
       dev->bitcount == bitcount)
   {
      if(dev->camera_mode == LUCAM_VIDEO_MODE)
      {
         old_colorid = lucam_fo_get_colorid(dev);
      }
      else
      {
          old_colorid = lucam_st_get_colorid(dev);
      }
      if (old_colorid == colorid)
      {
         // no changes to assert
         ret = 0;
         goto finish;
      }
   }

   if (dev->wanted_stream_state > LUCAM_STREAM_STATE_STOP)
   {
      lucam_set_stream_state(dev, LUCAM_STREAM_STATE_STOP);
   }

   dev->pixelformat = f->fmt.pix.pixelformat;
   // TODO: if relevant, amend f->fmt.pix.priv with the specific BAYER pattern from colorid
   dev->framesize = framesize;
   dev->imagesize = imagesize;
   dev->bitcount = bitcount;

   if(dev->camera_mode == LUCAM_VIDEO_MODE)
   {
      lucam_fo_set_subsampling(dev, sub_w, sub_h);
      lucam_fo_set_colorid(dev, colorid);
   }
   else
   {
      lucam_st_set_subsampling(dev, sub_w, sub_h);
      lucam_st_set_colorid(dev, colorid);
   }
#else
   /* Few verifications */
   ret = colorid_from_pixelformat(dev, f->fmt.pix.pixelformat, &bitcount, &colorid);
   if (ret < 0)
   {
      Info(dev, TRACE_INFO, "***vidioc_s_fmt_vid_cap: unknown pixel format\n");
      goto finish;
   }
   if (
      f->fmt.pix.width > lucam_get_width_max(dev) ||
      f->fmt.pix.width % lucam_get_width_unit(dev) ||
      f->fmt.pix.height > lucam_get_height_max(dev) ||
      f->fmt.pix.height % lucam_get_height_unit(dev) ||
      0
      )
   {
      ret = -EINVAL;
      goto finish;
   }
   if(dev->camera_mode == LUCAM_VIDEO_MODE) 
   {
      subw = (lucam_fo_get_subsampling_w(dev) & 0xff);
      subh = (lucam_fo_get_subsampling_h(dev) & 0xff);
   }
   else
   {
      subw = (lucam_st_get_subsampling_w(dev) & 0xff);
      subh = (lucam_st_get_subsampling_h(dev) & 0xff);
   }
   if (f->fmt.pix.width * subw > lucam_get_width_max(dev) ||
      f->fmt.pix.height * subh > lucam_get_height_max(dev))
   {
      Info(dev, TRACE_INFO, "**Warning: vidioc_s_fmt_vid_cap: must reset subsampling to 1\n");
      subw = 1;
      subh = 1;
      if(dev->camera_mode == LUCAM_VIDEO_MODE) 
      {
         ret = lucam_fo_set_subsampling(dev, subw, subh);
         if (ret >= 0)
         {
            ret = lucam_fo_set_pos(dev, (lucam_get_width_max(dev)-f->fmt.pix.width)/2, (lucam_get_height_max(dev)-f->fmt.pix.height)/2);
         }
      }
      else 
      {
         ret = lucam_st_set_subsampling(dev, subw, subh);
         if (ret >= 0)
         {
            ret = lucam_st_set_pos(dev, (lucam_get_width_max(dev)-f->fmt.pix.width)/2, (lucam_get_height_max(dev)-f->fmt.pix.height)/2);
         }
      }
      if (ret < 0)
      {
         goto finish;
      }
   }

#ifdef DRIVER_SOFT_FORMAT_VALIDATION
   ret = CustomSoftFormatValidate(dev, (dev->camera_mode != LUCAM_VIDEO_MODE), colorid, f->fmt.pix.width * subw, f->fmt.pix.height * subh);
   if (ret < 0)
   {
      Err(dev, "***vidioc_s_fmt_vid_cap: Could not soft validate format\n");
      goto finish;
   }
#endif

   /* done verifications, IO */
   if(dev->camera_mode == LUCAM_VIDEO_MODE) 
   {
      ret = lucam_fo_set_colorid(dev, colorid);
      if (ret >= 0)
      {
         ret = lucam_fo_set_size(dev, f->fmt.pix.width * subw, f->fmt.pix.height * subh);
      }
   }
   else
   {
      ret = lucam_st_set_colorid(dev, colorid);
      if (ret >= 0)
      {
         ret = lucam_st_set_size(dev, f->fmt.pix.width * subw, f->fmt.pix.height * subh);
      }
   }

   if (ret < 0)
   {
      Err(dev, "***vidioc_s_fmt_vid_cap: Could not write colorid and dims\n");
      goto finish;
   }
   
#ifndef DRIVER_SOFT_FORMAT_VALIDATION
   if (dev->lucam_flags & LUCAM_FLAGS_FORMAT_VALIDATION || dev->specification >= 2)
   {
      __u32 val = 0;
      if(dev->camera_mode == LUCAM_VIDEO_MODE) 
      {
         ret = lucam_reg32_rw(dev, LUCAM_FO_VALIDATE, &val, 1, (dev->specification >= 2)? USB_DIR_IN : USB_DIR_OUT);
      }
      else
      {
         ret = lucam_reg32_rw(dev, LUCAM_STILL_VALIDATE, &val, 1, (dev->specification >= 2)? USB_DIR_IN : USB_DIR_OUT);
      }
      if (ret < 0)
      {
         Err(dev, "***vidioc_s_fmt_vid_cap: Could not validate format\n");
         goto finish;
      }
      if ((dev->specification >= 2) && (val == 0))
      {
         Err(dev, "***vidioc_s_fmt_vid_cap: Format not valid\n");
         ret = -EINVAL;
         goto finish;
      }
   }
#endif

#ifdef DRIVER_SOFT_FORMAT_VALIDATION
   ret = CustomSoftFormatAdjustStartPos(dev, (dev->camera_mode != LUCAM_VIDEO_MODE));
   if (ret >= 0 ) ret = CustomSoftFormatAdjustColorId(dev, (dev->camera_mode != LUCAM_VIDEO_MODE));
   if (ret < 0)
   {
      Err(dev, "***vidioc_s_fmt_vid_cap: Could not soft read back pos/colorid\n");
      goto finish;
   }
#else
   if ((dev->lucam_flags & LUCAM_FLAGS_FORMAT_VALIDATION) || (dev->specification >= 2))
   {
      if(dev->camera_mode == LUCAM_VIDEO_MODE) 
      {
         ret = lucam_get_reg(dev, &dev->cam_registers.fo_color_id);
         if (ret >= 0) ret = lucam_get_reg(dev, &dev->cam_registers.fo_tap_configuration);
         if (ret >= 0)
         {
            if ((dev->cam_registers.fo_color_id.value & 0x08000000) && (dev->cam_registers.fo_tap_configuration.value == 0))
            {
               dev->cam_registers.fo_tap_configuration.value = 1;
            }
            ret = lucam_get_reg(dev, &dev->cam_registers.fo_position);
         }
      }
      else
      {
         ret = lucam_get_reg(dev, &dev->cam_registers.st_color_id);
         if (ret >= 0) ret = lucam_get_reg(dev, &dev->cam_registers.st_tap_configuration);
         if (ret >= 0)
         {
            if ((dev->cam_registers.st_color_id.value & 0x08000000) && (dev->cam_registers.st_tap_configuration.value == 0))
            {
               dev->cam_registers.st_tap_configuration.value = 1;
            }
            ret = lucam_get_reg(dev, &dev->cam_registers.st_position);
         }
         
      }
      if (ret < 0)
      {
         Err(dev, "***vidioc_s_fmt_vid_cap: Could not read back pos/colorid\n");
         goto finish;
      }
   }
#endif

#endif

finish:
    up(&dev->mutex);
    return ret;
}

//------------------------------------------------------------------------
//
#if VIDEODEV_VERSION_CODE < KERNEL_VERSION(3,10,0)
static int vidioc_s_std(struct file *file, void *priv, v4l2_std_id *i)
#else
static int vidioc_s_std(struct file *file, void *priv, v4l2_std_id i)
#endif
{
    return 0;
}

//------------------------------------------------------------------------
//
static int vidioc_cropcap(struct file *file, void *priv,
                    struct v4l2_cropcap *a)
{
    struct lucam_fh  *fh = priv;
    struct _lucam_device *dev = fh->dev;
    int ret = 0;

    a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    a->bounds = (struct v4l2_rect) {0, 0, lucam_get_width_max(dev), lucam_get_height_max(dev)};
    a->defrect = (struct v4l2_rect){0, 0, lucam_get_width_max(dev), lucam_get_height_max(dev)};
    a->pixelaspect = (struct v4l2_fract){1, 1};
    return (ret);
}

//-----------------------------------------------------------------------
//
static int vidioc_g_crop(struct file *file, void *priv,
             struct v4l2_crop *a)
{
    struct lucam_fh  *fh = priv;
    struct _lucam_device *dev = fh->dev;
    int ret = 0;

    lucam_fill_v4l2_crop(dev, a);

    return (ret);
}

//-----------------------------------------------------------------------
//
static int vidioc_s_crop(struct file *file, void *priv,
#if  VIDEODEV_VERSION_CODE >= KERNEL_VERSION(3,8,0)
            const
#endif
            struct v4l2_crop *a)
{
    struct lucam_fh  *fh = priv;
    struct _lucam_device *dev = fh->dev;
    int ret = 0;
    struct v4l2_format shadow_format, old_shadow_format;
    struct v4l2_crop old_shadow_crop;
    size_t sub_w, sub_h, bitcount;
    size_t framesize;

    // ASSUME: this is the video crop rectangle (i.e. don't check a->type)

    if (dev->file_that_set_fmt != NULL &&
        dev->file_that_set_fmt != file) {
        Info(dev, TRACE_INFO, "%s: Another file holds the format/stream lock\n", __func__);
        return -EBUSY;
    }
    dev->file_that_set_fmt = file;

    if (a->c.top < 0 || a->c.left < 0 || 
        a->c.width < lucam_get_width_unit(dev) ||
        a->c.height < lucam_get_height_unit(dev) ||
        a->c.top + a->c.height > lucam_get_height_max(dev) ||
        a->c.left + a->c.width > lucam_get_width_max(dev)
        ) {
        return -EINVAL;
    }

    lucam_fill_v4l2_crop(dev, &old_shadow_crop);
    lucam_fill_v4l2_format(dev, &old_shadow_format);
    shadow_format = old_shadow_format;
    
    ret = lucam_try_framesize(dev, &shadow_format, a, NULL,
                  &framesize, &bitcount, &sub_w, &sub_h);
    if (ret) {
        Err(dev, "Inconsistent results from vidioc_try_fmt_vid_cap and lucam_try_framesize: %d",
            ret);
        return ret;
    }

    if (dev->framesize == framesize && 
        shadow_format.fmt.pix.width == old_shadow_format.fmt.pix.width &&
        shadow_format.fmt.pix.height == old_shadow_format.fmt.pix.height &&
        shadow_format.fmt.pix.pixelformat == old_shadow_format.fmt.pix.pixelformat &&
        old_shadow_crop.c.top == a->c.top &&
        old_shadow_crop.c.left == a->c.left &&
        old_shadow_crop.c.width == a->c.width &&
        old_shadow_crop.c.height == a->c.height
               /* covers changes to sub_w and sub_h through checking *format and framesize*/
        ) {
        return 0;
    }

    if (dev->wanted_stream_state > LUCAM_STREAM_STATE_STOP)
        lucam_set_stream_state(dev, LUCAM_STREAM_STATE_STOP);

    dev->framesize = framesize;
    dev->bitcount = bitcount;
    dev->pixelformat = shadow_format.fmt.pix.pixelformat;

    if(dev->camera_mode == LUCAM_VIDEO_MODE) {
        lucam_fo_set_pos(dev, a->c.left, a->c.top);
        lucam_fo_set_size(dev, a->c.width, a->c.height);
        lucam_fo_set_subsampling(dev, sub_w, sub_h);
    } else {
        lucam_st_set_pos(dev, a->c.left, a->c.top);
        lucam_st_set_size(dev, a->c.width, a->c.height);
        lucam_st_set_subsampling(dev, sub_w, sub_h);
    }

    return (ret);
}

/* only one input in this driver */
static int vidioc_enum_input(struct file *file, void *priv, struct v4l2_input *inp)
{
    struct lucam_fh  *fh = priv;
    struct _lucam_device *dev = fh->dev;

   Info(dev, TRACE_INFO, "vidioc_enum_input\n");

    if (inp->index != 0)
        return -EINVAL;

    inp->type = V4L2_INPUT_TYPE_CAMERA;
    inp->std = V4L2_STD_UNKNOWN;
    strlcpy(inp->name, "Camera", sizeof(inp->name));
    inp->status = dev->current_stream_state;
    inp->audioset = 0;
    inp->tuner = 0;

    return (0);
}

//--------------------------------------------------------------------
//
static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
   struct lucam_fh  *fh __attribute__((unused)) = priv;

   Info(fh->dev, TRACE_INFO, "vidioc_g_input\n");

   *i = 0;

   return (0);
}
static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
   struct lucam_fh  *fh __attribute__((unused)) = priv;

   Info(fh->dev, TRACE_INFO, "vidioc_s_input\n");

   if (i > 0)
      return -EINVAL;

   return (0);
}

//---------------------------------------------------------------------
//
static int vidioc_reqbufs (struct file *file, void *priv, struct v4l2_requestbuffers *b)
{
   struct lucam_fh  *fh = priv;
   struct _lucam_device *dev = fh->dev;
   int ret;

   Info(dev, TRACE_V4L2, "vidioc_reqbufs\n");

   if (dev->file_that_set_fmt != NULL &&
        dev->file_that_set_fmt != file) {
      Info(dev, TRACE_V4L2 | TRACE_ERROR, "***%s: Another file holds the format/stream lock\n", __func__);
      return -EBUSY;
   }
   dev->file_that_set_fmt = file;

   if (b->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
      Info(dev, TRACE_V4L2 | TRACE_ERROR, "***%s: feature %d is not supported\n", __func__, b->type);
      return -EINVAL;
   }

   lucam_set_framesize(dev);

   ret = (videobuf_reqbufs(&dev->vb_vidq, b));
   if (ret < 0)
   {
      Err(dev, "***videoc_reqbufs: videobuf_reqbufs failed w %d\n", ret);
      if (ret == -EBUSY)
      {
         if (dev->vb_vidq.streaming)
         {
            Err(dev, "   *device is streaming\n");
         }
         if (!list_empty(&dev->vb_vidq.stream))
         {
            Err(dev, "   *device queue not empty\n");
         }
      }
   }
   return ret;
}

//---------------------------------------------------------------------
//
static int vidioc_querybuf(struct file *file, void *priv, struct v4l2_buffer *b)
{
   int rt;
   struct lucam_fh  *fh = priv;
   struct _lucam_device *dev = fh->dev;
#ifdef DEBUG
   struct videobuf_queue *q = &dev->vb_vidq;
   struct videobuf_buffer *vb = q->bufs[b->index];
   Info(dev, TRACE_V4L2, "vidioc_querybuf, index=%d, state=%d\n", b->index, vb->state);
#endif
   rt = (videobuf_querybuf(&dev->vb_vidq, b));

   Info(dev, TRACE_V4L2, "vidioc_querybuf exit w %d, buf.status=0x%x, bytesused:%d\n", rt, b->flags, b->bytesused);
   return rt;
}

//-----------------------------------------------------------------------
//
static int vidioc_qbuf (struct file *file, void *priv, struct v4l2_buffer *b)
{
   struct lucam_fh  *fh = priv;
   struct _lucam_device *plucam = fh->dev;
   int rt;

   Info(plucam, TRACE_V4L2, "vidioc_qbuf: 0x%p, index:%d\n", b, b->index);

   if (plucam->file_that_set_fmt != file) {
      Info(plucam, TRACE_V4L2 | TRACE_ERROR, "*%s: Another file holds the format/stream lock\n", __func__);
      return -EBUSY;
   }

   rt = videobuf_qbuf(&plucam->vb_vidq, b);
   if (rt < 0)
   {
      Err(plucam, "***vidioc_qbuf: video_qbuf returned %d\n", rt);
   }
   else
   {
#if ENHANCED_BUFFERING
      if (atomic_read(&plucam->bulk_buf_idle_count) == plucam->bulk_bufs_used)
      {
         rt = lucam_submit_bulkbufs(plucam, GFP_KERNEL);
         if (rt < 0)
         {
            Err(plucam, "vidioc_qbuf: failed to submit\n");
         }
         else
         {
            Info(plucam, TRACE_BULK|TRACE_V4L2, " video_qbuf: submitted %d\n", rt);
         }
      } // else let the callback handle this
      rt = 0;
#else
      if (plucam->wanted_stream_state != LUCAM_STREAM_STATE_RUN) 
      {
         rt = lucam_set_stream_state(plucam, LUCAM_STREAM_STATE_RUN);
         if (rt < 0) 
         {
            Err(plucam, "***Failed to set stream state to run\n");
            return rt;
         }
      }
#endif
   }
   return rt;
}

static int vidioc_dqbuf (struct file *file, void *priv, struct v4l2_buffer *b)
{
   struct lucam_fh  *fh = priv;
   struct _lucam_device *plucam = fh->dev;
   int rt;

   // Note: b->index is ignored.
   //
   Info(plucam, TRACE_V4L2, "vidioc_dqbuf: 0x%p\n", b);

   if ( b->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
      return -EINVAL;

   if (plucam->file_that_set_fmt != file) 
   {
      Info(plucam, TRACE_V4L2 | TRACE_ERROR, "***%s: Another file holds the format/stream lock\n", __func__);
      return -EBUSY;
   }

#if ALLOW_RGB24_OUTPUT
#warning transformations are not well thought out yet
#endif

#if ENHANCED_BUFFERING
   if (plucam->current_stream_state != LUCAM_STREAM_STATE_RUN)
   {
      return -EINVAL;
   }
#endif

   rt = videobuf_dqbuf(&plucam->vb_vidq, b, file->f_flags & O_NONBLOCK);
   if (rt < 0)
   {
      Err(plucam, "***vidioc_dqbuf: videobuf_dqbuf returned %d\n", rt);
   }
   else
   {
      Info(plucam, TRACE_V4L2, " vidioc_dqbuf success\n");
   }
   return rt;
}

//----------------------------------------------------------------------------
//
static int vidioc_streamon (struct file *file, void *priv, enum v4l2_buf_type i)
{
   struct lucam_fh  *fh = priv;
   struct _lucam_device *plucam = fh->dev;
   int rt;

   Info(plucam, TRACE_V4L2, "vidioc_streamon\n");

   if (plucam->file_that_set_fmt != file) {
      Info(plucam, TRACE_V4L2 | TRACE_ERROR, "***%s: Another file holds the format/stream lock\n", __func__);
      return -EBUSY;
   }

   rt = videobuf_streamon(&plucam->vb_vidq);
   if (rt < 0) 
   {
      Err(plucam, "***%s: Failed to set up buffer storage\n", __func__);
      return rt;
   }

   if (plucam->wanted_stream_state < LUCAM_STREAM_STATE_PAUSE) 
   {
      rt = lucam_set_stream_state(plucam, LUCAM_STREAM_STATE_PAUSE);
      if (rt < 0) 
      {
         Err(plucam, "***%s: Failed to set stream state to pause\n", __func__);
         return rt;
      }
   }

#if ENHANCED_BUFFERING
   if (plucam->wanted_stream_state != LUCAM_STREAM_STATE_RUN ||
        atomic_read(&plucam->bulk_buf_idle_count) == plucam->bulk_bufs_used) 
   {
      plucam->frame_error = 1;
#else
   if (plucam->wanted_stream_state != LUCAM_STREAM_STATE_RUN && !list_empty(&plucam->active_frames)) 
   {
#endif
      rt = lucam_set_stream_state(plucam, LUCAM_STREAM_STATE_RUN);
      if (rt < 0) 
      {
         Err(plucam, "***Failed to set stream state to run\n");
         return rt;
      }
   }
   return 0;
}

//------------------------------------------------------------------------
//
static int vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
   struct lucam_fh  *fh = priv;
   struct _lucam_device *plucam = fh->dev;
   int rt;

   Info(plucam, TRACE_V4L2, "vidioc_streamoff\n");

   if (plucam->file_that_set_fmt != NULL && plucam->file_that_set_fmt != file) 
   {
      Info(plucam, TRACE_V4L2|TRACE_ERROR, "***%s: Another file holds the format/stream lock\n", __func__);
      return -EBUSY;
   }

   if (plucam->wanted_stream_state > LUCAM_STREAM_STATE_STOP)
      lucam_set_stream_state(plucam, LUCAM_STREAM_STATE_STOP);

   rt = videobuf_streamoff(&plucam->vb_vidq);
   if (rt < 0)
   {
      Err(plucam, "***vidioc_streamoff: videobuf_streamoff failed\n");
   }
   return rt;
}

/* needed by v4l1-compat to implement VIDIOCMCAPTURE */
static int vidioc_overlay(struct file *file, void *fh, unsigned int i)
{
   return 0;
}

#ifdef CONFIG_VIDEO_V4L1_COMPAT
static int vidiocgmbuf (struct file *file, void *priv, struct video_mbuf *mbuf)
{
   struct lucam_fh  *fh = priv;

   Info(fh->dev, TRACE_V4L2, "vidiocgmbuf\n");

   return videobuf_cgmbuf(&fh->dev->vb_vidq, mbuf, 8);
}
#endif

/*************************************************************************
* Fn: lucam_video_mmap
**************************************************************************/
static int  lucam_video_mmap(struct file *file, struct vm_area_struct *vma)
{
    plucam_device plucam;
    int ret;

    struct video_device *vdev = video_devdata(file);
    plucam = (plucam_device)video_get_drvdata(vdev);

    if (plucam->unplugged) {
        return -EFAULT;
    }

    Info(plucam, (TRACE_MEMORY | TRACE_V4L2), "lucam_video_mmap: vma=0x%08lx\n", (unsigned long)vma);

   lucam_set_framesize(plucam);

    ret = videobuf_mmap_mapper(&plucam->vb_vidq, vma);

    Info(plucam, (TRACE_MEMORY | TRACE_V4L2), "vma start=0x%08lx, size=%ld, ret=%d\n",
         (unsigned long)vma->vm_start,
         (unsigned long)vma->vm_end-(unsigned long)vma->vm_start,
         ret);

    return ret;
}

static int vidioc_enum_framesizes(struct file *file, void *fh, struct v4l2_frmsizeenum *p)
{
    struct video_device *vdev = video_devdata(file);
    plucam_device plucam = (plucam_device)video_get_drvdata(vdev);

   Info(plucam, TRACE_INFO, "vidioc_enum_framesizes\n");

    if (p->index) 
        return -EINVAL;
    /* NOTE: We ignore the pixel_format because all pixel formats have
       the same frame size limitations. */

    /* remember the maximum values are relative to the current crop region */
    p->type = V4L2_FRMSIZE_TYPE_STEPWISE;
    p->stepwise.min_width = lucam_get_width_unit(plucam);
    p->stepwise.max_width = lucam_get_width_max(plucam) - lucam_st_get_posx(plucam);
    p->stepwise.step_width = lucam_get_width_unit(plucam) ;
    p->stepwise.min_height = lucam_get_height_unit(plucam);
    p->stepwise.max_height = lucam_get_height_max(plucam)- lucam_st_get_posy(plucam);   
    p->stepwise.step_height = lucam_get_height_unit(plucam);

    return 0;
}

static int vidioc_enum_frameintervals (struct file *file, void *priv,
                       struct v4l2_frmivalenum *fival)
{
   struct lucam_fh  *fh = priv;
   struct _lucam_device *plucam = fh->dev;
   int ret;
   unsigned int colorid;
   int out_bpp;
   size_t framesize;
   struct v4l2_crop shadow_crop;
   struct v4l2_format shadow;

   Info(plucam, TRACE_INFO, "vidioc_enum_frameintervals\n");

    if (plucam->kpsarray == NULL || plucam->kpsarraycnt == 0) {
        // try to get the array
        lucam_get_kps_array(plucam);
    }
    if (plucam->kpsarray == NULL || plucam->kpsarraycnt == 0) {
                Err(plucam, "Error: No KPS array\n");
                return -EFAULT;
    }
    if (fival->index >= plucam->kpsarraycnt) {
        return -EINVAL;
    }

    fival->type=V4L2_FRMIVAL_TYPE_DISCRETE;

    lucam_fill_v4l2_format(plucam, &shadow);
    shadow.fmt.pix.pixelformat=fival->pixel_format;
    shadow.fmt.pix.width=fival->width;
    shadow.fmt.pix.height=fival->height;

    colorid_from_pixelformat(plucam, fival->pixel_format, &out_bpp, &colorid);

    lucam_fill_v4l2_crop(plucam, &shadow_crop);

    ret = lucam_try_framesize(plucam, &shadow, &shadow_crop, &colorid,
                  &framesize, NULL, NULL, NULL);
    if (ret) {
        Info(plucam, TRACE_V4L2, "%s: format provided is not likely servable: #%u: %u%ux%u -> %d",
             __func__, fival->index, fival->pixel_format, fival->width, fival->height,
            ret);
    }
    fival->discrete.numerator = plucam->kpsarray[fival->index];
    fival->discrete.denominator = framesize;

    return 0;
}

//-----------------------------------------------------------------------------------------------
//
#if  VIDEODEV_VERSION_CODE <= KERNEL_VERSION(2,6,28)
static int lucam_video_ioctl_priv(struct file *file, void *fh, int cmd, void *arg)
#elif VIDEODEV_VERSION_CODE < KERNEL_VERSION(3,0,0)
static long lucam_video_ioctl_priv(struct file *file, void *fh, int cmd, void *arg)
#elif VIDEODEV_VERSION_CODE < KERNEL_VERSION(3,10,0)
static long lucam_video_ioctl_priv(struct file *file, void *fh, bool valid_prio, int cmd, void *arg)
#else
static long lucam_video_ioctl_priv(struct file *file, void *fh, bool valid_prio, unsigned int cmd, void *arg)
#endif
{
   struct video_device *vdev = video_devdata(file);
   plucam_device plucam=(plucam_device)video_get_drvdata(vdev);

   Info(plucam, TRACE_INFO, "lucam_video_ioctl_priv: called %c %d\n", _IOC_TYPE(cmd), _IOC_NR(cmd));

   switch (cmd)
   {
      case VIDIOC_ENUM_FRAMESIZES:
      {
        struct v4l2_frmsizeenum *frmsize = (struct v4l2_frmsizeenum*) arg;
        return vidioc_enum_framesizes(file, fh, frmsize);
      }
      case VIDIOC_ENUM_FRAMEINTERVALS:
      {
           struct v4l2_frmivalenum *frmival = (struct v4l2_frmivalenum *)arg;
         return vidioc_enum_frameintervals(file, fh, frmival);
      }
   }

   if (_IOC_TYPE(cmd) != VIDIO_LUCAM_IOC_MAGIC) return -EINVAL;
   if (_IOC_NR(cmd) > VIDIO_LUCAM_IOC_MAXNR) return -EINVAL;

   switch (cmd)
   {
      case VIDIO_LUCAM_SET_MODE:
      {
         __u32 *p = (__u32 *)arg;

         if (*p == plucam->camera_mode)
         {
            // nothing to do
            return 0;
         }
         if (plucam->wanted_stream_state != LUCAM_STREAM_STATE_STOP)
         {
            Err(plucam, "VIDIO_LUCAM_SET_MODE: Stream is not stopped\n");
            return -EBUSY;
         }
         if (*p != LUCAM_VIDEO_MODE && plucam->stream_count < 2)
         {
            Err(plucam, "VIDIO_LUCAM_SET_MODE: Still not supported\n");
            return -EINVAL;
         }
         plucam->camera_mode = *p;
#if 0
          if (plucam->camera_mode == LUCAM_VIDEO_MODE)
              pixelformat_from_colorid(plucam, lucam_fo_get_colorid(plucam), &plucam->pixelformat);
          else
              pixelformat_from_colorid(plucam, lucam_st_get_colorid(plucam), &plucam->pixelformat);

         lucam_set_framesize(plucam);
#endif
         return 0;
      }
      case VIDIO_LUCAM_GET_FO_COLORID:
      {
         *(__u32 *)arg = lucam_fo_get_colorid(plucam);
         return 0 ;
      }
      case VIDIO_LUCAM_GET_ST_COLORID:
      {
         *(__u32 *)arg = lucam_st_get_colorid(plucam);
         return 0 ;
      }
      case VIDIO_LUCAM_GET_FO_COLORID2:
      {
         ((lucam_color_id_2 *)arg)->colorId = lucam_fo_get_colorid(plucam);
         ((lucam_color_id_2 *)arg)->tapConfig = lucam_fo_get_tap_configuration(plucam);
         Info(plucam, TRACE_USB, " VIDIO_LUCAM_GET_FO_COLORID2: colorid:0x%08lx, tapcfg:0x%08lx\n", (long unsigned int)((lucam_color_id_2 *)arg)->colorId, (long unsigned int)((lucam_color_id_2 *)arg)->tapConfig);
         return 0 ;
      }
      case VIDIO_LUCAM_GET_ST_COLORID2:
      {
         ((lucam_color_id_2 *)arg)->colorId = lucam_st_get_colorid(plucam);
         ((lucam_color_id_2 *)arg)->tapConfig = lucam_st_get_tap_configuration(plucam);
         Info(plucam, TRACE_USB, " VIDIO_LUCAM_GET_ST_COLORID2: colorid:0x%08lx, tapcfg:0x%08lx\n", (long unsigned int)((lucam_color_id_2 *)arg)->colorId, (long unsigned int)((lucam_color_id_2 *)arg)->tapConfig);
         return 0 ;
      }
#if 0 /* the color id are set using the vidioc_s_fmt_vid_cap function */
      case VIDIO_LUCAM_SET_FO_COLORID:
      {
         __u32 *p = (__u32 *)arg;
         __u32 new_pixelformat;
         int ret;
         
         if (plucam->camera_mode == LUCAM_VIDEO_MODE && (plucam->wanted_stream_state != LUCAM_STREAM_STATE_STOP || plucam->current_stream_state != LUCAM_STREAM_STATE_STOP) )
         {
            return -EBUSY;
         }
         
         ret = lucam_fo_set_colorid(plucam, *p);
         if (ret < 0) return ret;
         if (plucam->camera_mode == LUCAM_VIDEO_MODE) 
         {
            pixelformat_from_colorid(plucam, *p, &new_pixelformat);
            plucam->pixelformat = new_pixelformat;
            lucam_set_framesize(plucam);
         }
         return 0;
      }
      case VIDIO_LUCAM_SET_ST_COLORID:
      {
         __u32 *p = (__u32 *)arg;
         int ret;
         __u32 new_pixelformat;
         if (plucam->camera_mode != LUCAM_VIDEO_MODE && (plucam->wanted_stream_state != LUCAM_STREAM_STATE_STOP || plucam->current_stream_state != LUCAM_STREAM_STATE_STOP) )
         {
            return -EBUSY;
         }
         ret = lucam_st_set_colorid(plucam, *p);
         if (ret < 0) return ret;
         if (plucam->camera_mode != LUCAM_VIDEO_MODE) 
         {
            pixelformat_from_colorid(plucam, *p, &new_pixelformat);
            plucam->pixelformat = new_pixelformat;
            lucam_set_framesize(plucam);
         }
         return 0;
      }
      case VIDIO_LUCAM_SET_FO_COLORID2:
      {
         lucam_color_id_2 *p = (lucam_color_id_2 *)arg;
         __u32 new_pixelformat;
         int ret;
         if (plucam->camera_mode == LUCAM_VIDEO_MODE && (plucam->wanted_stream_state != LUCAM_STREAM_STATE_STOP || plucam->current_stream_state != LUCAM_STREAM_STATE_STOP) )
         {
            return -EBUSY;
         }
         ret = lucam_fo_set_colorid(plucam, p->colorId);
         if (ret < 0) return ret;
         ret = lucam_fo_set_tap_configuration(plucam, p->tapConfig);
         if (ret < 0) return ret;
         if (plucam->camera_mode == LUCAM_VIDEO_MODE) 
         {
            pixelformat_from_colorid(plucam, p->colorId, &new_pixelformat);
            plucam->pixelformat = new_pixelformat;
            lucam_set_framesize(plucam);
         }
         return 0;
      }
      case VIDIO_LUCAM_SET_ST_COLORID2:
      {
         lucam_color_id_2 *p = (lucam_color_id_2 *)arg;
         __u32 new_pixelformat;
         int ret;
         if (plucam->camera_mode != LUCAM_VIDEO_MODE && (plucam->wanted_stream_state != LUCAM_STREAM_STATE_STOP || plucam->current_stream_state != LUCAM_STREAM_STATE_STOP) )
         {
            return -EBUSY;
         }
         ret = lucam_st_set_colorid(plucam, p->colorId);
         if (ret < 0) return ret;
         ret = lucam_st_set_tap_configuration(plucam, p->tapConfig);
         if (ret < 0) return ret;
         if (plucam->camera_mode != LUCAM_VIDEO_MODE) 
         {
            pixelformat_from_colorid(plucam, p->colorId, &new_pixelformat);
            plucam->pixelformat = new_pixelformat;
            lucam_set_framesize(plucam);
         }
         return 0;
      }
#else
      case VIDIO_LUCAM_SET_FO_COLORID2:/* the color id are set using the vidioc_s_fmt_vid_cap function. Just set the tap config */
      {
         lucam_color_id_2 *p = (lucam_color_id_2 *)arg;
         int ret;
         Info(plucam, TRACE_USB, " VIDIO_LUCAM_SET_FO_COLORID2: colorid:0x%08lx, tapcfg:0x%08lx\n", (long unsigned int)p->colorId, (long unsigned int)p->tapConfig);
         if (plucam->camera_mode == LUCAM_VIDEO_MODE && (plucam->wanted_stream_state != LUCAM_STREAM_STATE_STOP || plucam->current_stream_state != LUCAM_STREAM_STATE_STOP) )
         {
            return -EBUSY;
         }
         ret = lucam_fo_set_tap_configuration(plucam, p->tapConfig);
         if (ret < 0) return ret;
         return 0;
      }
      case VIDIO_LUCAM_SET_ST_COLORID2:/* the color id are set using the vidioc_s_fmt_vid_cap function. Just set the tap config  */
      {
         lucam_color_id_2 *p = (lucam_color_id_2 *)arg;
         int ret;
         Info(plucam, TRACE_USB, " VIDIO_LUCAM_SET_ST_COLORID2: colorid:0x%08lx, tapcfg:0x%08lx\n", (long unsigned int)p->colorId, (long unsigned int)p->tapConfig);
         if (plucam->camera_mode != LUCAM_VIDEO_MODE && (plucam->wanted_stream_state != LUCAM_STREAM_STATE_STOP || plucam->current_stream_state != LUCAM_STREAM_STATE_STOP) )
         {
            return -EBUSY;
         }
         ret = lucam_st_set_tap_configuration(plucam, p->tapConfig);
         if (ret < 0) return ret;
         return 0;
      }
#endif
      case VIDIO_LUCAM_GET_MODE:
      {
          *((__u32 *) arg) = plucam->camera_mode;
         return 0 ;
      }
      case VIDIO_LUCAM_GET_KPS:
      {
         if (lucam_get_reg(plucam, &plucam->cam_registers.fo_kps) < 0)
         {
            Err(plucam, "***Failed to read FO_KPS register\n");
            return -EFAULT;
         }

         // This just gets the value read earlier.
         *(__u32 *)arg = lucam_fo_get_kps(plucam);
         return 0;
      }
      case VIDIO_LUCAM_SET_KPS:
      {
         __u32 p = *(__u32 *)arg;
         return lucam_fo_set_kps(plucam, p);
      }
      case VIDIO_LUCAM_GET_KPS_COUNT:
      {
         __u32 count;
         Info(plucam, TRACE_INFO, "VIDIO_LUCAM_GET_KPS_COUNT\n");
         lucam_get_kps_array(plucam);
         if (plucam->kpsarray == NULL || plucam->kpsarraycnt == 0)
         {
            Err(plucam, "***Error: No KPS array\n");
            return -EFAULT;
         }
         count = plucam->kpsarraycnt;
         *(__u32 *)arg = count;
         return 0;
      }
      case VIDIO_LUCAM_GET_KPS_VALUE:
      {
         __u32 index = *(__u32 *)arg;
         Info(plucam, TRACE_INFO, "VIDIO_LUCAM_GET_KPS_VALUE, index=%u\n", index);
         if (plucam->kpsarray == NULL || plucam->kpsarraycnt == 0)
         {
            Err(plucam, "***Error: No KPS array\n");
            return -EFAULT;
         }
         if (index >= plucam->kpsarraycnt)
         {
            Err(plucam, "***Error: KPS index too high\n");
            return -EINVAL;
         }
         index = (__u32)plucam->kpsarray[index];
         *(__u32 *)arg = index;
         return plucam->kpsarraycnt;
      }
      case VIDIO_LUCAM_GET_VIDEO_SUBSAMPLING:
      {
         memcpy(arg, &plucam->cam_registers.fo_subsampling.value, sizeof(lucam_format_subsampling));
         return 0 ;
      }
      case VIDIO_LUCAM_SET_VIDEO_SUBSAMPLING:
      {
         lucam_format_subsampling *p = (lucam_format_subsampling *)arg;
         if (plucam->camera_mode == LUCAM_VIDEO_MODE && plucam->wanted_stream_state != LUCAM_STREAM_STATE_STOP)
         {
            Err(plucam, "***VIDIO_LUCAM_SET_VIDEO_SUBSAMPLING: Stream is not stopped\n");
            return -EBUSY;
         }
         if ((p->subsamplingX & 0xff) == 0 || (p->subsamplingY & 0xff) == 0)
         {
            Err(plucam, "***VIDIO_LUCAM_SET_VIDEO_SUBSAMPLING:Invalid param\n");
            return -EINVAL;
         }
         if ((p->subsamplingX & 0xff) > (plucam->cam_registers.fo_subsampling_inq.value & 0xff) ||
            (p->subsamplingY & 0xff) > (plucam->cam_registers.fo_subsampling_inq.value & 0xff))
         {
            Err(plucam, "***VIDIO_LUCAM_SET_VIDEO_SUBSAMPLING: Subs too high\n");
            return -EINVAL;
         }
         if ((plucam->cam_registers.fo_subsampling_inq.value & 0x80000000) &&
            p->subsamplingX != p->subsamplingY)
         {
            Err(plucam, "***VIDIO_LUCAM_SET_VIDEO_SUBSAMPLING: Unequal subs\n");
            return -EINVAL;
         }
         return lucam_fo_set_subsampling(plucam, p->subsamplingX, p->subsamplingY);
      }
      case VIDIO_LUCAM_GET_STILL_SUBSAMPLING:
      {
         memcpy(arg, &plucam->cam_registers.st_subsampling.value, sizeof(lucam_format_subsampling));
         return 0 ;
      }
      case VIDIO_LUCAM_SET_STILL_SUBSAMPLING:
      {
         lucam_format_subsampling *p = (lucam_format_subsampling *)arg;
         if (plucam->camera_mode != LUCAM_VIDEO_MODE && plucam->wanted_stream_state != LUCAM_STREAM_STATE_STOP)
         {
            Err(plucam, "***VIDIO_LUCAM_SET_STILL_SUBSAMPLING: Stream is not stopped\n");
            return -EBUSY;
         }
         if ((p->subsamplingX & 0xff) == 0 || (p->subsamplingY & 0xff) == 0)
         {
            Err(plucam, "***VIDIO_LUCAM_SET_STILL_SUBSAMPLING:Invalid param\n");
            return -EINVAL;
         }
         if ((p->subsamplingX & 0xff) > (plucam->cam_registers.fo_subsampling_inq.value & 0xff) ||
            (p->subsamplingY & 0xff) > (plucam->cam_registers.fo_subsampling_inq.value & 0xff))
         {
            Err(plucam, "***VIDIO_LUCAM_SET_STILL_SUBSAMPLING: Subs too high\n");
            return -EINVAL;
         }
         if ((plucam->cam_registers.fo_subsampling_inq.value & 0x80000000) &&
            p->subsamplingX != p->subsamplingY)
         {
            Err(plucam, "***VIDIO_LUCAM_SET_STILL_SUBSAMPLING: Unequal subs\n");
            return -EINVAL;
         }
         return lucam_st_set_subsampling(plucam, p->subsamplingX, p->subsamplingY);
      }
      case VIDIO_LUCAM_GET_VIDEO_STARTPOS:
      {
         memcpy(arg, &plucam->cam_registers.fo_position.value, sizeof(lucam_format_startpos));
         return 0 ;
      }
      case VIDIO_LUCAM_SET_VIDEO_STARTPOS:
      {
         lucam_format_startpos *p = (lucam_format_startpos *)arg;
         if (plucam->camera_mode == LUCAM_VIDEO_MODE && plucam->wanted_stream_state != LUCAM_STREAM_STATE_STOP)
         {
            Err(plucam, "***VIDIO_LUCAM_SET_VIDEO_STARTPOS: Stream is not stopped\n");
            return -EBUSY;
         }
         return lucam_fo_set_pos(plucam, p->startX, p->startY);
      }
      case VIDIO_LUCAM_GET_STILL_STARTPOS:
      {
         memcpy(arg, &plucam->cam_registers.st_position.value, sizeof(lucam_format_startpos));
         return 0 ;
      }
      case VIDIO_LUCAM_SET_STILL_STARTPOS:
      {
         lucam_format_startpos *p = (lucam_format_startpos *)arg;
         if (plucam->camera_mode != LUCAM_VIDEO_MODE && plucam->wanted_stream_state != LUCAM_STREAM_STATE_STOP)
         {
            Err(plucam, "***VIDIO_LUCAM_SET_STILL_STARTPOS: Stream is not stopped\n");
            return -EBUSY;
         }
         return lucam_st_set_pos(plucam, p->startX, p->startY);
      }
      case VIDIO_LUCAM_GET_COLOR_FORMAT:
      {
         Info(plucam, TRACE_INFO, "VIDIO_LUCAM_GET_COLOR_FORMAT\n");
         *(__u32 *)arg = plucam->cam_registers.color_inq.value;
         return 0 ;
      }
      case VIDIO_LUCAM_GET_COLOR_FORMAT_MORE:
      {
         int rt;
         Info(plucam, TRACE_INFO, "VIDIO_LUCAM_GET_COLOR_FORMAT_MORE\n");
         ((lucam_color_id_2 *)arg)->colorId = plucam->cam_registers.color_inq.value;
         rt = lucam_reg32_rw(plucam, LUCAM_FO_TAP_CONFIGURATION_INQ, &(((lucam_color_id_2 *)arg)->tapConfig), sizeof(__u32), USB_DIR_IN);
         return rt;
      }
      case VIDIO_LUCAM_GET_SUBSAMPLING_INQ:
      {
         *(__u32 *)arg = plucam->cam_registers.fo_subsampling_inq.value;
         return 0;
      }
      case VIDIO_LUCAM_SOFTWARE_TRIGGER:
      {
         return lucam_software_trigger(plucam);
      }
      case VIDIO_LUCAM_GET_SERIAL_NUMBER:
      {
         memcpy(arg, &plucam->serial_number, sizeof(plucam->serial_number));
         return 0 ;
      }
      case VIDIO_LUCAM_SET_STROBE_DELAY:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_param_flags_val(plucam, &plucam->cam_registers.st_strobe_delay, p->flags, p->value);
      }
      case VIDIO_LUCAM_SET_EXPOSURE_DELAY:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_param_flags_val(plucam, &plucam->cam_registers.st_exposure_delay, p->flags, p->value);
      }
      case VIDIO_LUCAM_SET_FO_EXPOSURE:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_fo_set_exposure(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_SET_ST_EXPOSURE:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_st_set_exposure(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_SET_FO_GAIN:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         Info(plucam, TRACE_INFO, "fo gain = %d \n",p->value);
         return lucam_fo_set_gain(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_SET_FO_RGAIN:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_fo_set_gain_red(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_SET_FO_G1GAIN:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_fo_set_gain_green1(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_SET_FO_G2GAIN:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_fo_set_gain_green2(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_SET_FO_BGAIN:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_fo_set_gain_blue(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_SET_ST_GAIN:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         Info(plucam, TRACE_INFO, "st gain = %d \n",p->value);
         return lucam_st_set_gain(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_SET_ST_RGAIN:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_st_set_gain_red(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_SET_ST_G1GAIN:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_st_set_gain_green1(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_SET_ST_G2GAIN:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_st_set_gain_green2(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_SET_ST_BGAIN:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_st_set_gain_blue(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_GAMMA:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.gamma);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.gamma.value;
         return 0;
      }
      case VIDIO_LUCAM_SET_GAMMA:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_gamma(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_CONTRAST:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.contrast);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.contrast.value;
         return 0;
      }
      case VIDIO_LUCAM_SET_CONTRAST:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_contrast(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_BRIGHTNESS:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.brightness);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.brightness.value;
         return 0;
      }
      case VIDIO_LUCAM_SET_BRIGHTNESS:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_brightness(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_FO_EXPOSURE:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.fo_exposure);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.fo_exposure.value;
         return 0 ;
      }
      case VIDIO_LUCAM_GET_FO_GAIN:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.fo_gain);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.fo_gain.value;
         return 0;
      }
      case VIDIO_LUCAM_GET_FO_RGAIN:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.fo_gain_red);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.fo_gain_red.value;

         return 0;
      }
      case VIDIO_LUCAM_GET_FO_G1GAIN:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.fo_gain_green1);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.fo_gain_green1.value;

         return 0;
      }
      case VIDIO_LUCAM_GET_FO_G2GAIN:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.fo_gain_green2);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.fo_gain_green2.value;

         return 0;
      }
      case VIDIO_LUCAM_GET_FO_BGAIN:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.fo_gain_blue);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.fo_gain_blue.value;

         return 0;
      }

      case VIDIO_LUCAM_GET_ST_EXPOSURE:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.st_exposure);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.st_exposure.value;
         return 0;
      }
      case VIDIO_LUCAM_GET_ST_GAIN:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.st_gain);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.st_gain.value;
         return 0;
      }
      case VIDIO_LUCAM_GET_ST_RGAIN:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.st_gain_red);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.st_gain_red.value;
         return 0;
      }
      case VIDIO_LUCAM_GET_ST_G1GAIN:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.st_gain_green1);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.st_gain_green1.value;
         return 0;
      }
      case VIDIO_LUCAM_GET_ST_G2GAIN:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.st_gain_green2);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.st_gain_green2.value;
         return 0;
      }
      case VIDIO_LUCAM_GET_ST_BGAIN:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.st_gain_blue);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.st_gain_blue.value;
         return 0;
      }
      case VIDIO_LUCAM_GET_STROBE_DELAY:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.st_strobe_delay);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.st_strobe_delay.value;
         return 0 ;
      }
      case VIDIO_LUCAM_GET_EXPOSURE_DELAY:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.st_exposure_delay);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.st_exposure_delay.value;
         return 0 ;
      }

      case VIDIO_LUCAM_GET_TEMPERATURE:
      {
         int rt;
         rt = lucam_reg32_rw(plucam, LUCAM_TEMPERATURE, arg, sizeof(lucam_prop_r), USB_DIR_IN);
         return rt;
      }
      case VIDIO_LUCAM_GPIO_READ:
      {
         __u16 gpio;
         int rt;

         rt = lucam_ext_cmd_rw(
             plucam,
             LUCAM_EXT_CMD_GPIO,
             0,
             &gpio,
             sizeof(gpio),
             USB_DIR_IN);
         if (rt < 0) return rt;
         Info(plucam, TRACE_INFO, "VIDIO_LUCAM_GPIO_READ: 0x%04x\n", (int)gpio);
         *(__u16 *)arg = gpio;
         return 0;
      }
      case VIDIO_LUCAM_GPIO_WRITE:
      {
         __u16 gpio = *(__u16 *)arg;
         int rt;

         rt = lucam_ext_cmd_rw(
             plucam,
             LUCAM_EXT_CMD_GPIO,
             0,
             &gpio,
             sizeof(gpio),
             USB_DIR_OUT);
         return rt;
      }
      case VIDIO_LUCAM_GPIO_SELECT:
      {
         __u16 gpio = *(__u16 *)arg;
         int rt;

         rt = lucam_ext_cmd_rw(
             plucam,
             LUCAM_EXT_CMD_GPIO_SELECT,
             0,
             &gpio,
             sizeof(gpio),
             USB_DIR_OUT);
         return rt;
      }
      case VIDIO_LUCAM_GPIO_CONFIGURE:
      {
         __u16 gpio = *(__u16 *)arg;
         int rt;

         rt = lucam_ext_cmd_rw(
             plucam,
             LUCAM_EXT_CMD_GPIO_CONFIGURE,
             0,
             &gpio,
             sizeof(gpio),
             USB_DIR_OUT);
         return rt;
      }
      case VIDIO_LUCAM_GET_VERSION:
      {
         lucam_version v;

         v.camera_id = plucam->udev->descriptor.idProduct;
         if (lucam_reg32_rw(plucam, LUCAM_FIRMFPGA_VERSION, &v.firmware_version, 1, USB_DIR_IN) < 0)
         {
             v.firmware_version = 0;
         }
         v.hardware_version = plucam->udev->descriptor.bcdDevice;
         v.driver_version = LUCAM_MODULE_VERSION;
         v.serial_number = plucam->serial_number;
         v.usb_speed = plucam->udev->speed; // low=1, full=2, high=3
         *(lucam_version *) arg = v;
         return 0;
      }
      case VIDIO_LUCAM_RESET:
      {
         Info(plucam, TRACE_INFO, "VIDIO_LUCAM_RESET\n");
         if (plucam->wanted_stream_state != LUCAM_STREAM_STATE_STOP)
         {
            Err(plucam, "***VIDIO_LUCAM_SET_MODE: Stream is not stopped\n");
            return -EBUSY;
         }
         return lucam_reset_sensor(plucam);
      }
      case VIDIO_LUCAM_GET_TIMEOUT:
      {
         __u32 to;
         Info(plucam, TRACE_INFO, "VIDIO_LUCAM_GET_TIMEOUT\n");
         if (plucam->timeout == -1)
         {
            to = -1;
         }
         else
         {
            to = plucam->timeout * 1000 / HZ;
         }
         *(__u32 *)arg = to;
         return 0;
      }
      case VIDIO_LUCAM_SET_TIMEOUT:
      {
         __u32 to = *(__u32 *) arg;
         if (to == -1)
         {
            plucam->timeout = -1; // infinite
         }
         else
         {
            plucam->timeout = (to * HZ / 1000);
         }
         return 0;
      }
      case VIDIO_LUCAM_GET_STROBE_DURATION:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.st_strobe_duration);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.st_strobe_duration.value;
         return 0;
      }
      case VIDIO_LUCAM_SET_STROBE_DURATION:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_param_flags_val(plucam, &plucam->cam_registers.st_strobe_duration, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_SNAPSHOT_COUNT:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.snapshot_count);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.snapshot_count.value;
         return 0;
      }
      case VIDIO_LUCAM_SET_SNAPSHOT_COUNT:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         Info(plucam, TRACE_INFO, "snap count = %d \n",p->value);
         return lucam_set_snapshot_count(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_SNAPSHOT_SETTING:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.snapshot_setting);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.snapshot_setting.value;

         return 0;
      }
      case VIDIO_LUCAM_SET_SNAPSHOT_SETTING:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         Info(plucam, TRACE_INFO, "snap setting = %d \n",p->value);
         return lucam_set_snapshot_setting(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_STILL_SHUTTER_TYPE:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.st_shutter_type);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.st_shutter_type.value;

         return 0;
      }
      case VIDIO_LUCAM_SET_STILL_SHUTTER_TYPE:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         Info(plucam, TRACE_INFO, "st shutter type = %d \n",p->value);
         return lucam_set_st_shutter_type(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_TRIGGER_PIN:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.trigger_pin);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.trigger_pin.value;

         return 0;
      }
      case VIDIO_LUCAM_SET_TRIGGER_PIN:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         Info(plucam, TRACE_INFO, "trigger pin = %d \n",p->value);
         return lucam_set_trigger_pin(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_STROBE_PIN:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.strobe_pin);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.strobe_pin.value;

         return 0;
      }
      case VIDIO_LUCAM_SET_STROBE_PIN:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         Info(plucam, TRACE_INFO, "strobe pin = %d \n",p->value);
         return lucam_set_strobe_pin(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_IRIS:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.iris);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.iris.value;

         return 0;
      }
      case VIDIO_LUCAM_SET_IRIS:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_iris(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_FOCUS:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.focus);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.focus.value;

         return 0;
      }
      case VIDIO_LUCAM_SET_FOCUS:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_focus(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_STILL_KNEE1_EXPOSURE:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.still_knee1_exposure);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.still_knee1_exposure.value;

         return 0;
      }
      case VIDIO_LUCAM_SET_STILL_KNEE1_EXPOSURE:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_still_knee1_exposure(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_STILL_KNEE2_EXPOSURE:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.still_knee2_exposure);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.still_knee2_exposure.value;

         return 0;
      }
      case VIDIO_LUCAM_SET_STILL_KNEE2_EXPOSURE:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_still_knee2_exposure(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_STILL_KNEE3_EXPOSURE:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.still_knee3_exposure);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.still_knee3_exposure.value;

         return 0;
      }
      case VIDIO_LUCAM_SET_STILL_KNEE3_EXPOSURE:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_still_knee3_exposure(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_VIDEO_KNEE:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.video_knee);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.video_knee.value;

         return 0;
      }
      case VIDIO_LUCAM_SET_VIDEO_KNEE:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_video_knee(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_KNEE2_LEVEL:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.knee2_level);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.knee2_level.value;

         return 0 ;
      }
      case VIDIO_LUCAM_SET_KNEE2_LEVEL:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_knee2_level(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_FO_UNIT_SIZE:
      {
    	 Info(plucam, TRACE_INFO, "VIDIO_LUCAM_GET_FO_UNIT_SIZE\n");
    	 *(__u32 *)arg = plucam->cam_registers.unit_size.value;
         return 0;
      }
      case VIDIO_LUCAM_GET_TIMESTAMPS:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.timestamps);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.timestamps.value;

         return 0 ;
      }
      case VIDIO_LUCAM_SET_TIMESTAMPS:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_timestamps(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_VIDEO_SETTING:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.video_settings);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.video_settings.value;

         return 0;
      }
      case VIDIO_LUCAM_SET_VIDEO_SETTING:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         Info(plucam, TRACE_INFO, "video setting = %d \n",p->value);
         return lucam_set_video_setting(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_WAIT_EVENT:
      {
         int rt;
         lucam_event_bits bits;

         // TODO: should poll() be paying attention to this too?
         // Or is it a subdev thing?

         // TODO: Add some kind of lock here
         rt = wait_event_interruptible(plucam->event_queue,
                  (plucam->event_pending.events[0] ||
                   plucam->event_pending.events[1]));
         if (rt < 0)
         {
            return rt;
         }
         bits.events[0] = plucam->event_pending.events[0];
         bits.events[1] = plucam->event_pending.events[1];
         plucam->event_pending.events[0]=0;
         plucam->event_pending.events[1]=0;
         *(lucam_event_bits *)arg = bits;
         return 0 ;
      }
      case VIDIO_LUCAM_WAIT_EVENT_CANCEL:
      {
         /* Signal to exit */
         // TODO add some kind of spin lock here as well
         plucam->event_pending.events[1] |= 0x80000000;
         wake_up(&plucam->event_queue);
         return 0;
      }
      case VIDIO_LUCAM_LOCK_EEPROM:
      {
         int ret = -2;

         // FIXME: ignore the timeout argument for now.  only
         // Linux > 2.6.26 has the down_timeout variant.
         if (down_interruptible(&plucam->mutex)) 
         {
            Err(plucam, "***LOCK_EEPROM: down_interruptible failed\n");
            return -ERESTARTSYS;
         }

         if (plucam->file_holding_eeprom_lock == NULL ||
             plucam->file_holding_eeprom_lock == file)
         {
            plucam->eeprom_lock_count++;
            plucam->file_holding_eeprom_lock = file;
            ret = 0;
         }
         else
         {
            // another fd already holds the lock
            Err(plucam, "***LOCK_EEPROM: another file has lock\n");
            ret = -1;
         }
         up(&plucam->mutex);

         return ret;
      }
      case VIDIO_LUCAM_UNLOCK_EEPROM:
      {
         int ret = -2;

         if (down_interruptible(&plucam->mutex))
         {
            return -ERESTARTSYS;
         }

         if (plucam->file_holding_eeprom_lock == file)
         {
            plucam->eeprom_lock_count--;
            if (0 >= plucam->eeprom_lock_count)
               plucam->file_holding_eeprom_lock = NULL;
            ret = 0;
         }
         else
         {
            // another fd holds the lock
            ret = -1;
         }
         up(&plucam->mutex);
         return ret;
      }
      case VIDIO_LUCAM_GET_TRUE_PIXEL_DEPTH:
      {
         __u32 depth;
         int ret;

         ret = lucam_reg32_rw(plucam, LUCAM_TRUE_PIXEL_DEPTH, &depth, 1, USB_DIR_IN);
         if (ret >= 0)
         {
            *(__u32 *)arg = depth;
            ret = sizeof(__u32);
         }
         return ret;
      }
      case VIDIO_LUCAM_GET_LUCAM_FLAGS:
      {
         __u32 flags;
         int ret;

         ret = lucam_reg32_rw(plucam, LUCAM_FLAGS, &flags, 1, USB_DIR_IN);
         if (ret >= 0)
         {
            if (plucam->specification >= 2)
            {
               flags |= LUCAM_FLAGS_KFREQ32;
            }
            *(__u32 *)arg = flags;
            ret = sizeof(__u32);
         }
         return ret;
      }
      case VIDIO_LUCAM_GET_BPC:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.bpc);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.bpc.value;

         return 0;
      }
      case VIDIO_LUCAM_SET_BPC:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_bpc(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_COUNTERS:
      {
         struct list_head *parser;
         unsigned long flags;
         *(struct lucam_counters *)arg = plucam->counters;
         
         ((struct lucam_counters *)arg)->pending_frames = 0;
         spin_lock_irqsave(&plucam->slock, flags);
         parser = plucam->active_frames.next;
         while (parser != &plucam->active_frames)
         {
            ((struct lucam_counters *)arg)->pending_frames++;
            parser = parser->next;
         }
         spin_unlock_irqrestore(&plucam->slock, flags);
#if ENHANCED_BUFFERING
         ((struct lucam_counters *)arg)->pending_usbtransfers = plucam->bulk_bufs_used - atomic_read(&plucam->bulk_buf_idle_count);
#else
         ((struct lucam_counters *)arg)->pending_usbtransfers = plucam->bulk_bufs_used;
#endif
         return 0;
      }
      case VIDIO_LUCAM_GET_AUTO_EXP_TARGET:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.auto_exp_target);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.auto_exp_target.value;

         return 0;
      }
      case VIDIO_LUCAM_SET_AUTO_EXP_TARGET:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_auto_exp_target(plucam, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_AUTO_EXP_MAX:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.auto_exposure_max);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.auto_exposure_max.value;
         return 0;
      }
      case VIDIO_LUCAM_SET_AUTO_EXP_MAX:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_param_flags_val(plucam, &plucam->cam_registers.auto_exposure_max, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_AUTO_GAIN_MIN:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.auto_gain_min);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.auto_gain_min.value;
         return 0;
      }
      case VIDIO_LUCAM_SET_AUTO_GAIN_MIN:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_param_flags_val(plucam, &plucam->cam_registers.auto_gain_min, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_AUTO_GAIN_MAX:
      {
         int rt;
         rt = lucam_get_param(plucam, &plucam->cam_registers.auto_gain_max);
         if (rt < 0) return rt;
         *(lucam_prop *)arg = plucam->cam_registers.auto_gain_max.value;
         return 0;
      }
      case VIDIO_LUCAM_SET_AUTO_GAIN_MAX:
      {
         lucam_prop_w *p = (lucam_prop_w *)arg;
         return lucam_set_param_flags_val(plucam, &plucam->cam_registers.auto_gain_max, p->flags, p->value);
      }
      case VIDIO_LUCAM_SET_TIMESTAMP_CTRL_LATCH:
      {
    	  int rt;
    	  rt = lucam_reg08_rw(plucam,LUCAM_REALTIMESTAMP+8,(__u8 *)&plucam->timestampLatched,sizeof(__u64),USB_DIR_IN);
    	  if(rt < 0)
    	  {
    		  return rt;
    	  }
    	  else
    	  {
    		  return 0;
    	  }
      }
      case VIDIO_LUCAM_SET_TIMESTAMP_CTRL_RESET:
      {
    	  int rt;
    	  plucam->timestampLatched=0;
    	  rt = lucam_reg08_rw(plucam,LUCAM_REALTIMESTAMP+8,(__u8 *)&plucam->timestampLatched,sizeof(__u64),USB_DIR_OUT);
    	  if(rt < 0)
    	  {
    		  return rt;
    	  }
    	  else
    	  {
    		  return 0;
    	  }
      }
      case VIDIO_LUCAM_SET_REALTIMESTAMP:
      {
          lucam_prop_w *p = (lucam_prop_w *)arg;
          return lucam_set_param_flags_val(plucam, &plucam->cam_registers.realtimestamp, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_REALTIMESTAMP:
      {
          int rt;
          rt = lucam_get_param(plucam, &plucam->cam_registers.realtimestamp);
          if (rt < 0) return rt;
          *(lucam_prop *)arg = plucam->cam_registers.realtimestamp.value;

          return 0 ;
      }
      case VIDIO_LUCAM_GET_TIMESTAMP_FREQ_LOW:
      {
    	  int rt;
    	  lucam_prop_r *p = (lucam_prop_r *)arg;
    	  rt = lucam_get_param(plucam, &plucam->cam_registers.realtimestamp_frequency);
    	  if (rt < 0) return rt;
    	  *p = plucam->cam_registers.realtimestamp_frequency.value;
    	  rt = lucam_reg32_rw(plucam,LUCAM_REALTIMESTAMP_FREQUENCY+8,&p->value,sizeof(lucam_prop_r)/sizeof(__u32),USB_DIR_IN);
    	  if(rt < 0)
    	  {
    		  return rt;
    	  }
    	  else
    	  {
    		  return 0;
    	  }
      }
      case VIDIO_LUCAM_GET_TIMESTAMP_FREQ_HIGH:
      {
    	  int rt;
    	  lucam_prop_r *p = (lucam_prop_r *)arg;
    	  rt = lucam_get_param(plucam, &plucam->cam_registers.realtimestamp_frequency);
    	  if (rt < 0) return rt;
    	  *p = plucam->cam_registers.realtimestamp_frequency.value;
    	  rt = lucam_reg32_rw(plucam,LUCAM_REALTIMESTAMP_FREQUENCY+0xc,&p->value,sizeof(lucam_prop_r)/sizeof(__u32),USB_DIR_IN);
    	  if(rt < 0)
    	  {
    		  return rt;
    	  }
    	  else
    	  {
    		  return 0;
    	  }

      }
      case VIDIO_LUCAM_GET_TIMESTAMP_VALUE_LOW:
      {
    	  int rt;
    	  lucam_prop *p = (lucam_prop *)arg;
    	  rt = lucam_get_param(plucam, &plucam->cam_registers.realtimestamp);
    	  if (rt < 0) return rt;
    	  *p = plucam->cam_registers.realtimestamp.value;
    	  p->value = plucam->timestampLatched & 0xFFFFFFFF;
    	  return 0;
      }
      case VIDIO_LUCAM_GET_TIMESTAMP_VALUE_HIGH:
      {
    	  int rt;
    	  lucam_prop *p = (lucam_prop *)arg;
    	  rt = lucam_get_param(plucam, &plucam->cam_registers.realtimestamp);
    	  if (rt < 0) return rt;
    	  *p = plucam->cam_registers.realtimestamp.value;
    	  p->value = plucam->timestampLatched >> 32;
    	  return 0;
      }
      case VIDIO_LUCAM_SET_TIMESTAMP_HW_RESET:
      {
          lucam_prop_w *p = (lucam_prop_w *)arg;
          return lucam_set_param_flags_val(plucam, &plucam->cam_registers.timestamp_hw_reset, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_TIMESTAMP_HW_RESET:
      {
          int rt;
          rt = lucam_get_param(plucam, &plucam->cam_registers.timestamp_hw_reset);
          if (rt < 0) return rt;
          *(lucam_prop *)arg = plucam->cam_registers.timestamp_hw_reset.value;

          return 0 ;
      }
      case VIDIO_LUCAM_GET_FOCAL_LENGTH:
      {
          int rt;
          rt = lucam_get_param(plucam, &plucam->cam_registers.focal_length);
          if (rt < 0) return rt;
          *(lucam_prop *)arg = plucam->cam_registers.focal_length.value;

          return 0 ;
      }
      case VIDIO_LUCAM_GET_IRIS_STEPS_COUNT:
      {
          int rt;
          rt = lucam_get_param(plucam, &plucam->cam_registers.iris_steps_count);
          if (rt < 0) return rt;
          *(lucam_prop *)arg = plucam->cam_registers.iris_steps_count.value;

          return 0 ;
      }
      case VIDIO_LUCAM_SET_IRIS_STEPS_COUNT:
      {
          lucam_prop_w *p = (lucam_prop_w *)arg;
          return lucam_set_param_flags_val(plucam, &plucam->cam_registers.iris_steps_count, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_LSC_X:
      {
          int rt;
          rt = lucam_get_param(plucam, &plucam->cam_registers.lsc_x);
          if (rt < 0) return rt;
          *(lucam_prop *)arg = plucam->cam_registers.lsc_x.value;

          return 0 ;
      }
      case VIDIO_LUCAM_SET_LSC_X:
      {
          lucam_prop_w *p = (lucam_prop_w *)arg;
          return lucam_set_param_flags_val(plucam, &plucam->cam_registers.lsc_x, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_LSC_Y:
      {
          int rt;
          rt = lucam_get_param(plucam, &plucam->cam_registers.lsc_y);
          if (rt < 0) return rt;
          *(lucam_prop *)arg = plucam->cam_registers.lsc_y.value;

          return 0 ;
      }
      case VIDIO_LUCAM_SET_LSC_Y:
      {
          lucam_prop_w *p = (lucam_prop_w *)arg;
          return lucam_set_param_flags_val(plucam, &plucam->cam_registers.lsc_y, p->flags, p->value);
      }
      case VIDIO_LUCAM_GET_TRIGGER_MODE:
      {
          int rt;
          rt = lucam_get_param(plucam, &plucam->cam_registers.trigger_mode);
          if (rt < 0) return rt;
          *(lucam_prop *)arg = plucam->cam_registers.trigger_mode.value;

          return 0 ;
      }
      case VIDIO_LUCAM_SET_TRIGGER_MODE:
      {
          lucam_prop_w *p = (lucam_prop_w *)arg;
          return lucam_set_param_flags_val(plucam, &plucam->cam_registers.trigger_mode, p->flags, p->value);
      }
      
      default:
      {
         if ((_IOC_NR(cmd) == _IOC_NR(VIDIO_LUCAM_SGL_THRU) || _IOC_NR(cmd) == _IOC_NR(VIDIO_LUCAM_REQUEST_THRU)) && (_IOC_DIR(cmd) & _IOC_WRITE))
         {
            lucam_sgl_thru *psgl_thru;
            int size;
            int rt=-EINVAL;
            __u8 req;

            size = _IOC_SIZE(cmd);
            if (size < sizeof(lucam_sgl_thru))
            {
               Err(plucam, "***SGL_THRU ioctl with invalid size\n");
               return -EINVAL;
            }
            psgl_thru = (lucam_sgl_thru *)arg;
            //printk(KERN_ERR "**SIZE=%d;ind=%d, val=%d, len=%d, reserved=%d\n", size,psgl_thru->index,psgl_thru->value,psgl_thru->length,psgl_thru->reserved);
            if (psgl_thru->length != size - sizeof(lucam_sgl_thru))
            {
               Err(plucam, "***SGL_THRU: Invalid size field\n");
               return -EINVAL;
            }
            if (_IOC_NR(cmd) == _IOC_NR(VIDIO_LUCAM_SGL_THRU))
               req = LUMENERA_EXT_CMD;
            else
               req = (__u8)psgl_thru->req_or_reserved;
            if (_IOC_DIR(cmd) == (_IOC_READ | _IOC_WRITE))
            {
               // is a read
               rt = lucam_generic_usb_vendor_request_rw(plucam, req, psgl_thru->index, psgl_thru->value, psgl_thru+1, psgl_thru->length, USB_DIR_IN);
            }
            else if (_IOC_DIR(cmd) == _IOC_WRITE)
            {
               // is a write
               rt = lucam_generic_usb_vendor_request_rw(plucam, req, psgl_thru->index, psgl_thru->value, psgl_thru+1, psgl_thru->length, USB_DIR_OUT);
            }

            return rt;
         }
         else if (_IOC_NR(cmd) == _IOC_NR(VIDIO_LUCAM_GET_MODEL_DATA) && (_IOC_DIR(cmd) == _IOC_READ))
         {
            int size;
            int i;
            const unsigned char *pModelDta;
            int modelDtaSize;
            unsigned short revId, productId;
            
            if(!plucam->udev) return -EFAULT;

            pModelDta = NULL;
            modelDtaSize = 0;
            
            productId = plucam->udev->descriptor.idProduct; 
            revId = plucam->udev->descriptor.bcdDevice;

#if !defined (__LITTLE_ENDIAN)
            /* Some host controller drivers on big endian systems do not swap the usb descriptors */
            if (plucam->udev->descriptor.idVendor == SWAP16(VID_HARDWARE_LUCAM) || plucam->udev->descriptor.idVendor == SWAP16(VID_HARDWARE_LUCAM2))
            {
               Info(plucam, TRACE_PROBE, "USB host controller driver did not swap the 16 bits fields of the USB descriptors\n");
               revId = SWAP16(revId);
               productId = SWAP16(productId);
            }
#endif

            for(i=0;i<ModelDtaListCount;i++)
            {
               if (ModelDtaList[i].Pid == productId && (ModelDtaList[i].Did == revId || ModelDtaList[i].Did == 0xffff))
               {
                  pModelDta = ModelDtaList[i].Pointer;
                  modelDtaSize = ModelDtaList[i].Size;
                  break;
               }
            }

            size = _IOC_SIZE(cmd);

            if (size == 0)
            {
               return modelDtaSize;
            }
            else if (size != modelDtaSize)
            {
               return -EINVAL;
            }
            else
            {
               memcpy(arg, pModelDta, modelDtaSize);
               return modelDtaSize;
            }
         }
         else if (_IOC_NR(cmd) == _IOC_NR(VIDIO_LUCAM_LUT8_GET) && (_IOC_DIR(cmd) == _IOC_READ))
         {
            int size;

            if ((plucam->cam_registers.message_support.value & (1 << LUCAM_PROP_MSG0_LUT8)) == 0)
            {
               Err(plucam, "VIDIO_LUCAM_LUT8_GET: LUT8 unsupported\n");
               return -EINVAL;
            }
            size = _IOC_SIZE(cmd);

            if (size == 0)
            {
               return plucam->lut8len;
            }
            else if (size != plucam->lut8len)
            {
               return -EINVAL;
            }
            else
            {
               memcpy(arg, plucam->lut8, size);
               return size;
            }
         }
         else if (_IOC_NR(cmd) == _IOC_NR(VIDIO_LUCAM_LUT16_GET) && (_IOC_DIR(cmd) == _IOC_READ))
         {
            int size;

            if ((plucam->cam_registers.message_support.value & (1 << LUCAM_PROP_MSG0_LUT16)) == 0)
            {
               Err(plucam, "VIDIO_LUCAM_LUT16_GET: LUT16 unsupported\n");
               return -EINVAL;
            }
            size = _IOC_SIZE(cmd);

            if (size == 0)
            {
               return plucam->lut8len;
            }
            else if (size != plucam->lut8len)
            {
               return -EINVAL;
            }
            else
            {
               memcpy(arg, plucam->lut8, size);
               return size;
            }
         }
         else if (_IOC_NR(cmd) == _IOC_NR(VIDIO_LUCAM_LUT8_SETUP) && (_IOC_DIR(cmd) == _IOC_WRITE))
         {
            int len;
            __u32 dummy;
            unsigned char *plut;

            if ((plucam->cam_registers.message_support.value & (1 << LUCAM_PROP_MSG0_LUT8)) == 0)
            {
               Err(plucam, "VIDIO_LUCAM_LUT8_SETUP: LUT8 unsupported\n");
               return -EINVAL;
            }

            len = _IOC_SIZE(cmd);

            if (plucam->lut8)
            {
               if (plucam->lut8len != len)
               {
                  kfree(plucam->lut8);
                  plucam->lut8 = NULL;
                  plucam->lut8len = 0;
               }
            }
            if (len)
            {
               if (plucam->lut8 == NULL)
               {
                  plucam->lut8 = kmalloc( len, GFP_KERNEL );
                  if (plucam->lut8 == NULL)
                  {
                     Err(plucam, "LUT8_SETUP: no memory for %d bytes\n", len);
                     return -ENOMEM;
                  }
               }
               memcpy(plucam->lut8, arg, len);
               plucam->lut8len = len;
            }
            if (len == 0) len = sizeof(__u32);
            if (len > 256) len = 256;

            plut = plucam->lut8;
            if (plut == NULL) plut = (unsigned char *)&dummy;

            return lucam_message(plucam, LUCAM_PROP_MSG0_LUT8, plut, len, USB_DIR_OUT);
         }
         else if (_IOC_NR(cmd) == _IOC_NR(VIDIO_LUCAM_LUT16_SETUP) && (_IOC_DIR(cmd) == _IOC_WRITE))
         {
            unsigned char *pTableToSend=NULL;
            unsigned char *pTableToSave=NULL;
            unsigned long lengthToSave=0;
            int len;
            int rt;
            unsigned char tableHeader[4];

            if ((plucam->cam_registers.message_support.value & (1 << LUCAM_PROP_MSG0_LUT16)) == 0)
            {
               Err(plucam, "VIDIO_LUCAM_LUT16_SETUP: LUT16 unsupported\n");
               return -EINVAL;
            }

            len = _IOC_SIZE(cmd);

            if (len>4)
            {
               memcpy(tableHeader, arg, 4);
               if (tableHeader[0] == 0xfc && len > (unsigned long)(tableHeader[1] + 4))
               {
                  lengthToSave = (tableHeader[1]);
                  pTableToSave = (unsigned char *)arg+4;

                  len -= (tableHeader[1] + 4);
                  arg += (tableHeader[1] + 4);
               }
            }

            if (len)
            {
               pTableToSend = kmalloc( len, GFP_KERNEL );
               if (pTableToSend == NULL)
               {
                  Err(plucam, "LUT16: Failed to malloc %d bytes\n", len);
                  kfree(plucam->lut8);
                  plucam->lut8 = NULL;
                  plucam->lut8len = 0;
                  return -EFAULT;
               }
               memcpy(pTableToSend, arg, len);
            }

            // Here we save the description block
            if (plucam->lut8)
            {
               if (plucam->lut8len != lengthToSave)
               {
                  kfree(plucam->lut8);
                  plucam->lut8 = NULL;
                  plucam->lut8len = 0;
               }
            }
            if (lengthToSave)
            {
               if (plucam->lut8 == NULL)
               {
                  plucam->lut8 = kmalloc( lengthToSave, GFP_KERNEL );
                  if (plucam->lut8 == NULL)
                  {
                     Err(plucam, "LUT16_SETUP: no memory for %d bytes\n", (int)lengthToSave);
                     if (pTableToSend)
                     {
                        kfree(pTableToSend);
                     }
                     plucam->lut8len = 0;
                     return -ENOMEM;
                  }
               }
               memcpy(plucam->lut8, pTableToSave, lengthToSave);
               plucam->lut8len = lengthToSave;
            }

            rt = lucam_message(plucam, LUCAM_PROP_MSG0_LUT16, pTableToSend, len, USB_DIR_OUT);
            if (pTableToSend)
            {
               kfree(pTableToSend);
            }
            return rt;
         }
#if USB3_ENDPOINT_CONFIGURATION
         else if ( _IOC_NR(cmd) == _IOC_NR(VIDIO_LUCAM_FLASH) )
         {
            switch ( _IOC_DIR(cmd) )
            {
               case _IOC_WRITE:
               {
                  lucam_flash_access *pwflash;
                  __u32 size;
                  int ret = -EINVAL;

                  size = _IOC_SIZE(cmd);
                  pwflash = (lucam_flash_access *)arg;

                  if (pwflash->command == FLASH_COMMAND_READ_WRITE)
                  {
                     if (size <= sizeof(lucam_flash_access))
                     {
                        Err(plucam, "***FLASH ioctl with invalid size in WRITE\n");
                        return -EINVAL;
                     }
                     
                     if (pwflash->len != size - sizeof(lucam_flash_access))
                     {
                        Err(plucam, "***FLASH: Invalid size field in WRITE\n");
                        return -EINVAL;
                     }
                  
                     ret = lucam_flash_program( plucam, pwflash);
                  }
                  else if ((pwflash->command == FLASH_COMMAND_ERASE_SECTOR)
                         || (pwflash->command == FLASH_COMMAND_ERASE_BULK)
                         || (pwflash->command == FLASH_COMMAND_ERASE_SUBSECTOR))
                  {
                     if (size != sizeof(lucam_flash_access))
                     {
                        Err(plucam, "***FLASH ioctl with invalid size in ERASE\n");
                        return -EINVAL;
                     }

                     ret = lucam_flash_erase( plucam, pwflash );
                  }
                  else
                  {
                     Err(plucam, "***FLASH: Invalid cmd w\n");
                     ret = -EINVAL;
                  }

                  return ret;
               }
           
               case ( _IOC_READ | _IOC_WRITE ):
               {
                  lucam_flash_access *prflash;
                  __u32 size;
                  int ret = -EINVAL;

                  prflash = (lucam_flash_access *)arg;
                  Info(plucam, TRACE_INFO, "   off=0x%x,len=%d,totalsize=%d\n", prflash->offset,prflash->len,_IOC_SIZE(cmd) );
              
                  size = _IOC_SIZE(cmd);

                  if (prflash->command == FLASH_COMMAND_READ_WRITE)
                  {
                     if (size <= sizeof(lucam_flash_access))
                     {
                        Err(plucam, "***FLASH ioctl with invalid size in READ\n");
                        return -EINVAL;
                     }
                 
                     if (prflash->len != size - sizeof(lucam_flash_access))
                     {
                        Err(plucam, "***FLASH: Invalid size field in READ\n");                
                        return -EINVAL;
                     }

                     ret = lucam_flash_read( plucam, prflash);
                     return ret;
                  }
                  else if (prflash->command == FLASH_COMMAND_IDENTIFY)
                  {
                     if (size != sizeof(lucam_flash_access))
                     {
                        Err(plucam, "***FLASH ioctl with invalid size in READ\n");
                        return -EINVAL;
                     }
                     
                     prflash->offset = plucam->lucam_flags;
                     if (lucam_reg32_rw( plucam, LUCAM_FLASH_TYPE, &prflash->len, 1, USB_DIR_IN ) < 0)
                     {
                        Err(plucam, "***Failed to read flash type\n");
                        return -1;
                     }
                     
                     return 0;
                  }
                  else
                  {
                     Err(plucam, "***FLASH: Invalid cmd  rw field\n");                
                     return -EINVAL;
                  }
               }

               default:
                  return -EINVAL;
            }
         }
#endif

         Info(plucam, TRACE_INFO, "Unknown ioctl: TYPE=%d, NR=%d, arg=%p\n", (int)(_IOC_TYPE(cmd)), (int)(_IOC_NR(cmd)), arg);
         return -EINVAL;
      }
   }
}

#if  VIDEODEV_VERSION_CODE < KERNEL_VERSION(2,6,26)
//--------------------------------------------------------------------------
//
static int lucam_video_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
   if ((_IOC_TYPE(cmd) == VIDIO_LUCAM_IOC_MAGIC) && (_IOC_NR(cmd) > VIDIO_LUCAM_IOC_MAXNR))
   {
      return lucam_video_ioctl_priv(file, NULL, (int)cmd, (void *)arg); 
   }
   return video_ioctl2(inode, file, cmd, arg); 
}
#endif


#if  VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,27)
static const struct v4l2_ioctl_ops lucam_v4l2_ioctl_ops = 
{
    .vidioc_querycap      = vidioc_querycap,        
    .vidioc_enum_fmt_vid_cap  = vidioc_enum_fmt_vid_cap, 
    .vidioc_g_fmt_vid_cap     = vidioc_g_fmt_vid_cap, 
    .vidioc_try_fmt_vid_cap   = vidioc_try_fmt_vid_cap, 
    .vidioc_s_fmt_vid_cap     = vidioc_s_fmt_vid_cap, 
    .vidioc_cropcap       = vidioc_cropcap,         
    .vidioc_g_crop        = vidioc_g_crop,          
    
    .vidioc_s_crop        = vidioc_s_crop,          
    
    .vidioc_reqbufs       = vidioc_reqbufs,         
    .vidioc_querybuf      = vidioc_querybuf,        
    .vidioc_qbuf          = vidioc_qbuf,            
    .vidioc_dqbuf         = vidioc_dqbuf,           
    .vidioc_s_std         = vidioc_s_std,           
    .vidioc_enum_input    = vidioc_enum_input,      
    .vidioc_g_input       = vidioc_g_input,         
    .vidioc_s_input       = vidioc_s_input,         
    .vidioc_queryctrl     = vidioc_queryctrl,       
    .vidioc_g_ctrl        = vidioc_g_ctrl,          
    .vidioc_s_ctrl        = vidioc_s_ctrl,          
    .vidioc_streamon      = vidioc_streamon,        
    .vidioc_streamoff     = vidioc_streamoff,       
    .vidioc_overlay       = vidioc_overlay,         
#if  VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,26)
    .vidioc_default       = lucam_video_ioctl_priv, 
#endif
#if  VIDEODEV_VERSION_CODE > KERNEL_VERSION(2,6,28) 
    .vidioc_enum_framesizes = vidioc_enum_framesizes,
    .vidioc_enum_frameintervals = vidioc_enum_frameintervals,
#endif
/* May be useful:   .vidioc_g_fmt_type_private     = vidioc_g_fmt_type_private, */
/* May be useful:   .vidioc_try_fmt_type_private   = vidioc_try_fmt_type_private, */
/* May be useful:   .vidioc_s_fmt_type_private     = vidioc_s_fmt_type_private, */
#ifdef CONFIG_VIDEO_V4L1_COMPAT
    .vidiocgmbuf          = vidiocgmbuf,
#endif
};
#endif

static const struct
#if  VIDEODEV_VERSION_CODE <= KERNEL_VERSION(2,6,28)
file_operations
#else
v4l2_file_operations
#endif
lucam_v4l_fops = {
    .owner      = THIS_MODULE,
    .open       = lucam_video_open,
    .release    = lucam_video_close,
    .read       = lucam_video_read,
#if  VIDEODEV_VERSION_CODE < KERNEL_VERSION(2,6,26)
    .ioctl      = lucam_video_ioctl,
#elif VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,26) && VIDEODEV_VERSION_CODE < KERNEL_VERSION(4,0,0)
    .ioctl      = video_ioctl2,
#else
    .unlocked_ioctl = video_ioctl2,
#endif
#if defined(CONFIG_COMPAT) && VIDEODEV_VERSION_CODE <= KERNEL_VERSION(2,6,28)
    .compat_ioctl   = v4l_compat_ioctl32,
#endif
    .mmap       = lucam_video_mmap,
    .poll       = lucam_video_poll,
};

struct video_device lucam_template = {
    .name    =   CUSTOM_MANUFACTURER_SHORTNAME " USB Camera",
#if VIDEODEV_VERSION_CODE < KERNEL_VERSION(2,6,27)
    .type    =   VID_TYPE_CAPTURE|VID_TYPE_SUBCAPTURE,
#endif
#if VIDEODEV_VERSION_CODE < KERNEL_VERSION(2,6,24)
    .hardware  = VID_HARDWARE_LUCAM,
#endif
    .fops    = &lucam_v4l_fops,
    .minor   =  0,

    .release = video_device_release,

#if  VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,27)
    .ioctl_ops  = &lucam_v4l2_ioctl_ops,
#else
    .vidioc_querycap      = vidioc_querycap,        
    .vidioc_enum_fmt_cap  = vidioc_enum_fmt_vid_cap, 
    .vidioc_g_fmt_cap     = vidioc_g_fmt_vid_cap, 
    .vidioc_try_fmt_cap   = vidioc_try_fmt_vid_cap, 
    .vidioc_s_fmt_cap     = vidioc_s_fmt_vid_cap, 
    .vidioc_cropcap       = vidioc_cropcap,         
    .vidioc_g_crop        = vidioc_g_crop,          
    .vidioc_s_crop        = vidioc_s_crop,          
    .vidioc_reqbufs       = vidioc_reqbufs,         
    .vidioc_querybuf      = vidioc_querybuf,        
    .vidioc_qbuf          = vidioc_qbuf,            
    .vidioc_dqbuf         = vidioc_dqbuf,           
    .vidioc_s_std         = vidioc_s_std,           
    .vidioc_enum_input    = vidioc_enum_input,      
    .vidioc_g_input       = vidioc_g_input,         
    .vidioc_s_input       = vidioc_s_input,         
    .vidioc_queryctrl     = vidioc_queryctrl,
    .vidioc_g_ctrl        = vidioc_g_ctrl,          
    .vidioc_s_ctrl        = vidioc_s_ctrl,          
    .vidioc_streamon      = vidioc_streamon,        
    .vidioc_streamoff     = vidioc_streamoff,       
    .vidioc_overlay       = vidioc_overlay,
#ifdef CONFIG_VIDEO_V4L1_COMPAT
    .vidiocgmbuf          = vidiocgmbuf,
#endif
#endif
};

#if 0
static const char lucam_proc_name = "video/lucam";

/****************************************************************************
 * proc file system
 ***************************************************************************/
static struct proc_dir_entry *_lucam_proc_dir;

static int _read_lucam_proc(char *buf, char **start, off_t offset, int count, int* eof, void *data)
{
    int len = 0;
    plucam_device _plucam;

    len += sprintf(buf+len, "version: \t\t Feb 18, 2005\n\n");
    _plucam = ()
    if((_plucam==NULL) || (_plucam->unplugged==1))
    {
        len+= sprintf(buf+len, "Lucam is not connected\n");
        return len;
    }

    len += sprintf(buf+len, "gain flag:\t\t%#lx\n", (unsigned long) _plucam->cam_registers.fo_gain.value.flag);
    len += sprintf(buf+len, "gain value:\t\t%lu\n",(unsigned long) _plucam->cam_registers.fo_gain.value.value);
    len += sprintf(buf+len, "gain min:\t\t%lu\n", (unsigned long) _plucam->cam_registers.fo_gain.value.min);
    len += sprintf(buf+len, "gain max:\t\t%lu\n\n",(unsigned long) _plucam->cam_registers.fo_gain.value.max);

    len += sprintf(buf+len, "exposure flag:\t\t%#lx\n", (unsigned long) _plucam->cam_registers.fo_exposure.value.flag);
    len += sprintf(buf+len, "exposure value:\t\t%lu\n",(unsigned long) _plucam->cam_registers.fo_exposure.value.value);
    len += sprintf(buf+len, "exposure min:\t\t%lu\n", (unsigned long) _plucam->cam_registers.fo_exposure.value.min);
    len += sprintf(buf+len, "exposure max:\t\t%lu\n\n",(unsigned long) _plucam->cam_registers.fo_exposure.value.max);

#if 0
    len += sprintf(buf+len, "gamma value:\t\t%#lx\n",(unsigned long) _gamma.value);
    len += sprintf(buf+len, "gamma min:\t\t%#lx\n", (unsigned long) _gamma.min);
    len += sprintf(buf+len, "gamma max:\t\t%#lx\n\n",(unsigned long) _gamma.max);
#endif
    len += sprintf(buf+len, "color id:\t\t%lu\n",(unsigned long) lucam_fo_get_colorid(_plucam));
    len += sprintf(buf+len, "(width, height):\t\t(%lu, %lu)\n",(unsigned long) lucam_fo_get_width(_plucam),
                                                    (unsigned long) lucam_fo_get_height(_plucam));
#if 0
    len += sprintf(buf+len, "curr frame rate:\t%lu\n\n",_frame_rate);
    {
        unsigned long cnt, i;
        i = _compute_bandwidth(_plucam, _frame_rate);
        len += sprintf(buf+len, "curr bw (kps):\t\t%lu\n", i);

        len += sprintf(buf+len, "avaible bandwidth in kps:\n");
        cnt = lucam_fo_get_kps_cnt(_plucam);
        for(i=0; i<cnt; ++i)
        {
            len += sprintf(buf+len, "%u\t", _plucam->kpsarray[i]);
        }
        len += sprintf(buf+len, "\n");
    }
#endif
    return len;
}

static int _lucam_proc_create(plucam_device plucam)
{
    char proc_name[12];

    sprintf(proc_name, "lucam%d");
    create_proc_read_entry("config", 0, _lucam_proc_dir, _read_lucam_proc, NULL);
}

static int _lucam_proc_destroy(plucam_device plucam)
{
}

static int _lucam_proc_init(void)
{
    /* add /proc */
    _lucam_proc_dir = create_proc_entry("lucam", S_IFDIR, NULL);
    return 0;
}

static int _lucam_proc_term(void)
{
    remove_proc_entry("config", _lucam_proc_dir);
    remove_proc_entry("lucam", NULL);
    return 0;
}
#endif

/****************************************************************************
 *
 *  USB routines
 *
 ***************************************************************************/
static int lucam_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
    struct video_device* vdev;
    int ret, i;
    plucam_device plucam;
    struct usb_interface_descriptor *ifacedesc;
    struct usb_device *udev = interface_to_usbdev(interface);   // macro
    unsigned int ifnum;
    int colorInq;
    int newminor = -1;

    /* We don't handle multi-config cameras */
    if (udev->descriptor.bNumConfigurations != 1) return -ENODEV;

    ifacedesc = &interface->altsetting[0].desc;
    ifnum  = ifacedesc->bInterfaceNumber;


#ifdef DEBUG
    printk(KERN_INFO "lucam_probe() called [0x%04X 0x%04X], ifn=%d\n",
       udev->descriptor.idVendor, udev->descriptor.idProduct, ifnum);
    printk(KERN_INFO " detected usb speed=%d\n", (int)udev->speed);
#endif

    if (usbport_nr)
    {
       int usb_port=0;
       int usb_busnum=0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16)
       usb_port = udev->portnum;
#else
       struct usb_device *root_udev;
       root_udev = udev->parent;
       if (root_udev)
       {
          int port1;
#ifdef DEBUG
          printk(KERN_INFO " Lucam root device on usb devnum=%d port %d maxchild %i\n",
             root_udev->devnum, root_udev->ttport, root_udev->maxchild);
#endif
          for (port1 = 0; port1 < root_udev->maxchild; ++port1)
          {
              struct usb_device *tmp_udev = root_udev->children[port1];
              if (tmp_udev == udev)
              {
                 usb_port=port1;
                 break;
              }
          }
       }
#endif
      if (udev->bus)
      {
         usb_busnum = (udev->bus->busnum-1);
      }

      newminor = usb_port | (usb_busnum << 3);
#ifdef DEBUG
      printk(KERN_INFO " Lucam detected on usb bus =%d port %d newminor 0x%X\n",
         usb_busnum, usb_port, newminor);
#endif
   }


   /* We don't handle multi-config cameras */
   /*Check vendor ID */
   /*Only One Interface for Lucam*/
   if (udev->descriptor.bNumConfigurations != 1||
       udev->config[0].desc.bNumInterfaces != 1     ||
       ifnum!=0)
   {
      return -ENODEV;
   }
   /*find a spare slot, support upto MAX_CAMERA_NUMBER Lucam*/
   plucam=lucam_find_and_init_camera(udev, interface);
   if(plucam == NULL) return -ENODEV;
   /*Do something serious*/

#if 0
   /* set configuration -- must be done prior to downloading firmware*/
   if (usb_set_configuration (udev, udev->config[0].bConfigurationValue) < 0)
   {
      Err(plucam, "set_configuration failed, %d\n", udev->config[0].bConfigurationValue);
      goto error;
   }
   else
   {
      Info(plucam, TRACE_PROBE, "set_configuration-- %d\n", udev->config[0].bConfigurationValue);
   }
#endif

   // read sn
   lucam_read_serial_number(plucam);

   // enum endpoints
   if ((ret = lucam_enum_endpoints(plucam)) < 0)
   {
      printk(KERN_ERR "No endpoints found. Is the host port USB 2.0?\n");
      goto error;
   }

   // Fight race condition between disconnect/probe and open/close
   down(&plucam->mutex);

   //construct a video structure
   vdev = video_device_alloc();
   if (vdev == NULL) {
      printk(KERN_ERR "Oops, could not allocate memory for video_device.\n");
      goto error;
   }
   memcpy(vdev, &lucam_template, sizeof(lucam_template));
#if VIDEODEV_VERSION_CODE < KERNEL_VERSION(4,0,0)
   vdev->debug &= 1;
#else
   vdev->dev_debug &= 1;
#endif
   plucam->vdev = vdev;
   if (video_get_drvdata(vdev) != 0)
   {
      Err(plucam, "***Non-NULL driver data on vdev!\n");
   }
   video_set_drvdata(vdev, plucam);
#if !USB3_ENDPOINT_CONFIGURATION
   //programming fpga
   lucam_fpga_setup(plucam);
   if (plucam->unplugged) goto error;
#endif
   if(lucam_registers_init(plucam) < 0)
	   goto error;
   if (plucam->unplugged) goto error;
   /*connecting*/
   colorInq = lucam_get_colorinq(plucam) & 0xff;
#if VIDEODEV_VERSION_CODE < KERNEL_VERSION(2,6,27)
   if (colorInq == LUCAM_COLOR_MONO8 || colorInq == LUCAM_COLOR_MONO16 || colorInq == LUCAM_COLOR_MONO61)
   {
      vdev->type |= VID_TYPE_MONOCHROME;
   }
   vdev->owner = THIS_MODULE;
#endif
#if  VIDEODEV_VERSION_CODE >= KERNEL_VERSION(3,11,0)
   vdev->v4l2_dev = &plucam->v4l2_dev;
#elif  VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,27)
   vdev->parent = &udev->dev;
#endif
   vdev->release = video_device_release;

   /* registering video device */
   if (plucam->unplugged) goto error;
   Info(plucam, TRACE_PROBE, "Registering as /dev/video%d.\n", vdev->minor & 0x3F);
   ret=video_register_device(plucam->vdev, VFL_TYPE_GRABBER, newminor);
   if (ret < 0) {
      Err(plucam, "Failed to register as video device (%d).\n", ret);
      goto error;
   } else {
      Info(plucam, TRACE_PROBE, "Registered as /dev/video%d.\n", vdev->minor & 0x3F);
   }

   /* If any camera control initialization is to be done, do it only at _probe time,
    * the V4L2 policy is to allow other tools to tweak the camera between openings.
    *
    * However, these cameras have a bunch of out of band stuff V4l2
    * does not know about, Maybe those should be reset...? */
   /* Put all controls at a sane state */
   for (i = 0; i < ARRAY_SIZE(lucam_qctrl); i++)
      qctl_regs[i] = lucam_qctrl[i].default_value;
#if 0

   /* find out the supported frame rates */
   cnt = lucam_fo_get_kps_cnt(plucam);
   if(cnt > 1 && plucam->kpsarray!=0) {
      lucam_fo_set_kps(plucam, plucam->kpsarray[cnt-1]);
   }
   {
      lucam_fo_set_gain(plucam, plucam->cam_registers.fo_gain.value.min);
      lucam_fo_set_gain_red(plucam,    plucam->cam_registers.fo_gain_red.value.min);
      lucam_fo_set_gain_blue(plucam,   plucam->cam_registers.fo_gain_blue.value.min);
      lucam_fo_set_gain_green1(plucam, plucam->cam_registers.fo_gain_green1.value.min);
      lucam_fo_set_gain_green2(plucam, plucam->cam_registers.fo_gain_green2.value.min);
      lucam_fo_set_size(plucam, lucam_get_width_max(plucam), lucam_get_height_max(plucam));
      //lucam_fo_set_colorid(plucam, lucam_get_colorinq(plucam));
      lucam_fo_set_pos(plucam, 0,0);
      lucam_fo_set_exposure(plucam, 10000);
   }
   {
      lucam_st_set_gain(plucam, plucam->cam_registers.fo_gain.value.min);
      lucam_st_set_gain_red(plucam,    plucam->cam_registers.fo_gain_red.value.min);
      lucam_st_set_gain_blue(plucam,   plucam->cam_registers.fo_gain_blue.value.min);
      lucam_st_set_gain_green1(plucam, plucam->cam_registers.fo_gain_green1.value.min);
      lucam_st_set_gain_green2(plucam, plucam->cam_registers.fo_gain_green2.value.min);
      lucam_st_set_subsampling(plucam, lucam_fo_get_subsampling_w(plucam),lucam_fo_get_subsampling_h(plucam));
      lucam_st_set_size(plucam, lucam_get_width_max(plucam), lucam_get_height_max(plucam));
      lucam_st_set_colorid(plucam, lucam_fo_get_colorid(plucam));
      lucam_st_set_pos(plucam, 0,0);
      lucam_st_set_exposure(plucam, 10000);
      lucam_st_set_strobe_delay(plucam, 1);
   }
#endif

#if ALLOW_RGB24_OUTPUT
   plucam->imagesize_max = (unsigned long)(lucam_get_height_max(plucam)) * lucam_get_width_max(plucam)*3;
   plucam->pixelformat = V4L2_PIX_FMT_RGB24; // FIXME: arbitrary, but
                          // seems likely if the
                          // support is compiled
                          // in.
#else
   plucam->imagesize_max = (unsigned long)(lucam_get_height_max(plucam)) * lucam_get_width_max(plucam)*2;
   plucam->pixelformat = V4L2_PIX_FMT_SBGGR8; // FIXME: arbitrary, map the colorinq value into v4l2 space.
#endif

   lucam_set_framesize(plucam);

   //MOD_INC_USE_COUNT;
   up (&plucam->mutex);

   usb_set_intfdata(interface, plucam);    /* FIXME: why? */
   
   return 0;

error:
   if(plucam)
   {
      if(plucam->vdev)
      {
         video_device_release(plucam->vdev);
         plucam->vdev=NULL;
      }
      plucam->udev=NULL;
      up (&plucam->mutex);
      plucam=NULL;
   }
   return -ENODEV;
}


/*****************************************************************************
* Fn: lucam_disconnect
*****************************************************************************/
static void
lucam_disconnect(struct usb_interface *interface)
{
   plucam_device plucam = usb_get_intfdata(interface);
   struct usb_device *udev = interface_to_usbdev(interface);
   int destroy_camera;
   
   Info(plucam, TRACE_PROBE, "lucam_disconnect()\n");

   usb_set_intfdata(interface, NULL);  /* FIXME: why? */

   if (plucam == NULL)
   {
      Err(plucam, "lucam_disconnect() Called without private pointer.\n");
      return;
   }

   if (plucam->udev == NULL)
   {
      Err(plucam, "lucam_disconnect() already called for %p\n", plucam);
      return;
   }
   if (plucam->udev != udev)
   {
      Err(plucam, "lucam_disconnect() Woops: pointer mismatch udev/lucam.\n");
      return;
   }
   lucam_set_stream_state(plucam, LUCAM_STREAM_STATE_STOP);
   down(&plucam->mutex);
   destroy_camera = (plucam->unplugged==0 && (plucam->opencount==0));
   plucam->unplugged = 1;
   
#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(3,11,0)
   v4l2_device_disconnect(&plucam->v4l2_dev);
#endif
   
   if (destroy_camera)
   {
      lucam_destroy_camera(plucam);
   }
   if (plucam->opencount)
   {
      plucam->event_pending.events[1] |= 0x00000001;
      wake_up(&plucam->event_queue);
   }
#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(3,11,0)
   v4l2_device_unregister(&plucam->v4l2_dev);
#endif
   up(&plucam->mutex);
   //MOD_DEC_USE_COUNT;
}


extern struct usb_device_id lucam_ids[];
extern const char lucam_usb_driver_name[];


static struct usb_driver lucam_driver =
{
    //I did not test with 2.4.19, so I am not sure which version this
    //change was first made, but I know it is in 2.4.20 but not in 2.4.18.
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
    .owner       = THIS_MODULE,
#endif
    .name        = lucam_usb_driver_name,
    .probe       = lucam_probe,
    .disconnect  = lucam_disconnect,
    .id_table    = lucam_ids
};

/****************************************************************************
 *
 *  Debug routines
 *
 ***************************************************************************/
#if defined(DEBUG) && defined(CONFIG_MAGIC_SYSRQ)
#include <linux/sysrq.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
static void do_sysrq(int key)
#else
static void do_sysrq(int key, struct tty_struct * ignore)
#endif
{
   int idx_lucam;

   for( idx_lucam = 0; idx_lucam < MAX_CAMERA_NUMBER; ++idx_lucam) 
   {
      int idx_image;
      plucam_image_buf pimage_buf;
      plucam_device plucam = &lucam_dev[idx_lucam];
      if (!plucam->udev) 
      {
         continue;
      }
      printk("camera %d: stream state=(%d,%d), mode=%d, colorids=(%d,%d), framesize=%d\n",
            plucam->index,
            plucam->current_stream_state, plucam->wanted_stream_state,
            plucam->camera_mode,
            lucam_fo_get_colorid(plucam), lucam_st_get_colorid(plucam),
            (int)plucam->framesize);
#if !ENHANCED_BUFFERING
      pimage_buf = &plucam->blank_image_buf;
      printk("blank image (%d): offset=%d, data=%8p\n",
            pimage_buf->vb.i,
            pimage_buf->offset,
            pimage_buf->data);
#endif
#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,35)
      printk("videobuf: streaming=%d, reading=%d, waiters=%d\n",
            plucam->vb_vidq.streaming, plucam->vb_vidq.reading,
            waitqueue_active(&plucam->vb_vidq.wait));
#elif VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,26)
      printk("videobuf: streaming=%d, reading=%d, is_mapped=%d, waiters=%d\n",
            plucam->vb_vidq.streaming, plucam->vb_vidq.reading,
            plucam->vb_vidq.is_mmapped,
            waitqueue_active(&plucam->vb_vidq.wait));
#elif VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,25)
      printk("videobuf: streaming=%d, reading=%d, is_mapped=%d\n",
            plucam->vb_vidq.streaming, plucam->vb_vidq.reading,
            plucam->vb_vidq.is_mmapped);
#else
      printk("videobuf: streaming=%d, reading=%d\n",
            plucam->vb_vidq.streaming, plucam->vb_vidq.reading);
#endif
      printk("Images: \n");
      for(idx_image =0; idx_image < VIDEO_MAX_FRAME; ++ idx_image)
      {
         pimage_buf = (plucam_image_buf) plucam->vb_vidq.bufs[idx_image];
         if (NULL == pimage_buf)
            continue;
         printk("image %d: state=%x, data=%8p, offset=%d, in_q=%d, waiters=%d\n",
               pimage_buf->vb.i,
               pimage_buf->vb.state,
               pimage_buf->data,
               pimage_buf->offset,
               list_empty(&pimage_buf->vb.queue) ? 0 : 1,
               waitqueue_active(&pimage_buf->vb.done));
      }
      printk("Queues: \n");
      {
         char summary[MAX_IMAGE_BUFS + 256];
         char *printto = &summary[0];
         int offset;
         summary[0] = '\0';
         list_for_each_entry(pimage_buf, &plucam->active_frames, vb.queue) 
         {
            offset = snprintf(printto, summary + ARRAY_SIZE(summary) - printto,
                     "%d ", pimage_buf->vb.i);
            printto += offset;
         }
         printk("images waiting for data: %s\n", summary);
         offset = 0;
         summary[0] = '\0';
         list_for_each_entry(pimage_buf, &plucam->vb_vidq.stream, vb.queue)
         {
            offset = snprintf(printto, summary + ARRAY_SIZE(summary) - printto,
                     "%d ", pimage_buf->vb.i);
            printto += offset;
         }
         printk("streamed images: %s\n", summary);
      }
   }
}

static struct sysrq_key_op lucam_sysrq_key_op = {
    .handler = do_sysrq,
    .help_msg = "lucam-(V)4l",
    .action_msg = "Dumping lucam (V)4l state",
};

static void setup_sysrq(void)
{
    register_sysrq_key('v', &lucam_sysrq_key_op);
}

static void cleanup_sysrq(void)
{
    unregister_sysrq_key('v', &lucam_sysrq_key_op);
}
#else
static void setup_sysrq(void) {}
static void cleanup_sysrq(void) {}
#endif

/****************************************************************************
 *
 *  Module routines
 *
 ***************************************************************************/

static int __init usb_lucam_init(void)
{
    int i=0;
    // if(trace>=0)
    {
#if  LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18) && LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
        printk(KERN_INFO "Trace options=0x%04x\n", lucam_template.debug);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0)
        printk(KERN_INFO "Trace options=0x%04x\n", lucam_template.dev_debug);
#endif
        printk(KERN_INFO "usbport_nr=%d\n", usbport_nr);
    }

    setup_sysrq();

    for (i = 0; i < MAX_CAMERA_NUMBER; i++)
    {
        plucam_device plucam = &lucam_dev[i];
        memset(plucam, 0, sizeof (lucam_device));
    }

    if (usb_register(&lucam_driver) < 0)
        return -1;
    //_lucam_proc_init();
    printk(KERN_INFO "lucam driver version %d.%d.%d.%d registered\n",
        (LUCAM_MODULE_VERSION & 0xFF000000) >> 24,
        (LUCAM_MODULE_VERSION & 0xFF0000) >> 16,
        (LUCAM_MODULE_VERSION & 0xFF00) >> 8,
         LUCAM_MODULE_VERSION & 0xFF);
    return 0;
}


static void __exit usb_lucam_exit(void)
{
    printk(KERN_INFO "Deregistering driver.\n");
    //_lucam_proc_term();
    usb_deregister(&lucam_driver);

    cleanup_sysrq();
}

#if USB3_ENDPOINT_CONFIGURATION
/****************************************************************************
 *
 *  Erasing flash through the flash endpoint.
 *
 ***************************************************************************/
static int lucam_flash_erase( plucam_device plucam, lucam_flash_access *pwflash )
{
   __u32 data;
   __u32 maxDelay;
   __u32 i;
   int ret = 0;
   
   if (( pwflash == NULL ) || ( plucam == NULL ))
   {
      Err(plucam, "*** lucam_flash_erase  01\n");
      return -1;
   }

   if(!plucam->udev) return -EFAULT;

   maxDelay = pwflash->len;

   data = pwflash->offset;
   if ( 0 > lucam_reg32_rw( plucam, LUCAM_FLASH_OFFSET, &data, 1, USB_DIR_OUT ) )
   {
      Err(plucam, "*** lucam_flash_erase  02\n");
      return -1;
   }

   if (pwflash->command == FLASH_COMMAND_ERASE_BULK)
   {
      data = LUCAM_FLASH_MODE_BULK_ERASE;
   }
   else if (pwflash->command == FLASH_COMMAND_ERASE_SECTOR)
   {
      data = LUCAM_FLASH_MODE_SECTOR_ERASE;
   }
   else if (pwflash->command == FLASH_COMMAND_ERASE_SUBSECTOR)
   {
      if ((plucam->lucam_flags & LUCAM_FLAGS_FLASH_SUBSECTORS_SUPPORTED) == 0)
      {
         Err(plucam, "*** lucam_flash_erase  03\n");
         return -1;
      }
      data = LUCAM_FLASH_MODE_SUBSECTOR_ERASE;
   }
   else
   {
      Err(plucam, "*** lucam_flash_erase  04\n");
      return -1;
   }
   if ( 0 > lucam_reg32_rw( plucam, LUCAM_FLASH_MODE, &data, 1, USB_DIR_OUT ) )
   {
      Err(plucam, "*** lucam_flash_erase  05\n");
      return -1;
   }


   // 3. Wait for programming done
   for (i = maxDelay; i != 0; i--)
   {
      if ( 0 > lucam_reg32_rw( plucam, LUCAM_FLASH_MODE, &data, 1, USB_DIR_IN ) )
      {
         Err(plucam, "*** lucam_flash_erase  06\n");
         ret = -1;
         goto exit_here;
      }
           
      if ((data & LUCAM_FLASH_MODE_DONE) == 0)
      {
         Info(plucam, TRACE_INFO, "Delay for flash still erasing\n");

         mdelay(25);   
      }
      else
      {
         break;
      }
   }

   if (i == 0)
   {
      if ( 0 > lucam_reg32_rw( plucam, LUCAM_FLASH_MODE, &data, 1, USB_DIR_IN ) )
      {
         Err(plucam, "*** lucam_flash_erase  07\n");
         ret = -1;
         goto exit_here;
      }

      if ((data & LUCAM_FLASH_MODE_DONE) == 0)
      {
         Err(plucam, "***flash still erasing, failing\n");
         ret = -1;
         goto exit_here;
      }
   }

exit_here:
   // 4. Set to normal mode
   data = LUCAM_FLASH_MODE_NORMAL;
   if ( 0 > lucam_reg32_rw( plucam, LUCAM_FLASH_MODE, &data, 1, USB_DIR_OUT ) )
   {
      Err(plucam, "*** lucam_flash_erase  08\n");
      ret = -1;
   }
   return ret;
}

/****************************************************************************
 *
 *  Reading flash through the flash endpoint.
 *
 ***************************************************************************/
static int lucam_flash_read( plucam_device plucam, lucam_flash_access *prflash)
{
   __u32 flash_mode;
   int ret = 0;
   __u32 data;
   __u32 maxSize = PAGE_SIZE;
   __u32 remainingSize;
   __u32 thisSize;
   
   BYTE *pTransferData = (BYTE *)(prflash+1);

   if (( prflash == NULL ) || ( plucam == NULL ))
   {
      Err(plucam, "*** lucam_flash_read  00\n");
      return -1;
   }

   if ( prflash->len == 0 )
   {
      Err(plucam, "*** lucam_flash_read  01\n");
      return -1;
   }

   if(!plucam->udev) return -EFAULT;

   if ( plucam->video_pipe == 0 )
   {
      Err(plucam, "*** lucam_flash_read  03\n");
      return -1;
   }

   if ( 0 != Is_flash_accessible( plucam) )
   {
      Err(plucam, "*** prflash->len=%d\n",prflash->len);
      Err(plucam, "*** lucam_flash_read  06\n");
      return -ENOMEM;
   }

   data = prflash->offset;
   if ( 0 > lucam_reg32_rw( plucam, LUCAM_FLASH_OFFSET, &data, 1, USB_DIR_OUT ) )
   {
      Err(plucam, "*** lucam_flash_read  04\n");
      goto exit_here;
   }

   data = prflash->len;
   if ( 0 > lucam_reg32_rw( plucam, LUCAM_FLASH_UPDATE_SIZE, &data, 1, USB_DIR_OUT ) )
   {
      Err(plucam, "*** lucam_flash_read  05\n");
      goto exit_here;
   }

   flash_mode = LUCAM_FLASH_MODE_READ;
   ret = lucam_reg32_rw(plucam, LUCAM_FLASH_MODE, &flash_mode, 1, USB_DIR_OUT);

   //mdelay(2);
   remainingSize = prflash->len;

   Info(plucam, TRACE_INFO, " pipe=0x%8x,len=%d\n\n",plucam->video_pipe, prflash->len);

   while (remainingSize)
   {
      thisSize = remainingSize;
      if (thisSize > maxSize) thisSize = maxSize;

      ret = lucam_bulk_msg( plucam, plucam->miscdatain_pipe, pTransferData, thisSize, USB_DIR_IN);
      if( ret < 0 )
      {
         Err(plucam, "***lucam_flash_read: failed to read from FLASH \n");
         goto exit_here;
      }

      remainingSize -= thisSize;
      pTransferData += thisSize;
   }
 
   //set FLASH to normal mode
   flash_mode = LUCAM_FLASH_MODE_NORMAL;
   ret = lucam_reg32_rw(plucam, LUCAM_FLASH_MODE, &flash_mode, 1, USB_DIR_OUT);

   if ( ret < 0 )
   {
      Err(plucam, "***lucam_flash_read--set FLASH mode to normal failed\n");
   }
   
   // verify if it is OK
   ret = lucam_reg32_rw( plucam, LUCAM_FPGA_MODE, &flash_mode, 1, USB_DIR_IN );
   if ( ret < 0 )
   {
      Err(plucam, "***lucam_flash_read--read FLASH mode failed-- verify\n");
   }
   else if ( flash_mode == LUCAM_FLASH_MODE_NORMAL )
   {
      ret = 0;
   }
   else
   {
      ret = -1;
      Err(plucam, "***lucam_flash_read--read FLASH mode failed-- still not read\n");
   }

exit_here:
   return ret;
}


/****************************************************************************
 *
 *  Programming flash through the flash endpoint.
 *
 ***************************************************************************/
static int lucam_flash_program( plucam_device plucam, lucam_flash_access *pwflash)
{
   __u32 flash_mode;
   int ret = 0;
   __u32 data;
   __u32 maxSize = PAGE_SIZE;
   __u32 remainingSize;
   __u32 thisSize;

   BYTE *pTransferData = (BYTE *)(pwflash+1);
   int i;

   if ((!plucam) || (!pwflash))
   {
      return -EFAULT;
   }

   if(!plucam->udev) return -EFAULT;

   if (Is_flash_accessible( plucam) < 0)
   {
      return -ENOMEM;
   }

   // tell the camera where will be programmed in flash
   data = pwflash->offset;
   if ( 0 > lucam_reg32_rw( plucam, LUCAM_FLASH_OFFSET, &data, 1, USB_DIR_OUT ) )
   {
      goto exit_here;;
   }
   data = pwflash->len;
   if ( 0 > lucam_reg32_rw( plucam, LUCAM_FLASH_UPDATE_SIZE, &data, 1, USB_DIR_OUT ) )
   {
      goto exit_here;
   }

   flash_mode = LUCAM_FLASH_MODE_PROGRAM;
   ret = lucam_reg32_rw(plucam, LUCAM_FLASH_MODE, &flash_mode, 1, USB_DIR_OUT);

   //mdelay(2);
   remainingSize = pwflash->len;

   while (remainingSize)
   {
      thisSize = remainingSize;
      if (thisSize > maxSize) thisSize = maxSize;

      ret = lucam_bulk_msg( plucam, plucam->miscdataout_pipe, pTransferData, pwflash->len,USB_DIR_OUT);
      if( ret < 0 )
      {
         Err(plucam, "***lucam_flash_write: failed to write to FLASH \n");
         goto exit_here;
      }

      remainingSize -= thisSize;
      pTransferData += thisSize;
   }

   for (i = 4; i != 0; i--)
   {
      ret = lucam_reg32_rw(plucam, LUCAM_FLASH_MODE, &flash_mode, 1, USB_DIR_IN);
      if ( ret < 0 )
      {
         Err(plucam, "***lucam_flash_write--reading FLASH mode\n");
         goto exit_here;
      }

      // It is expected that bit 0x4000 be set if data is still being moved from EP8 fifo to fpga. This is for debug info only
      if ((flash_mode & LUCAM_FLASH_MODE_DONE) == 0)
      {
         Info(plucam, TRACE_INFO, "Delay for flash still programming\n");

         mdelay(16);
      }
      else
      {
         break;
      }
   }

   if (i == 0)
   {
      ret = lucam_reg32_rw(plucam, LUCAM_FLASH_MODE, &flash_mode, 1, USB_DIR_IN);
      if ( ret < 0 )
      {
         Err(plucam, "***lucam_flash_write--reading FLASH mode\n");
         goto exit_here;
      }

      if ((flash_mode & LUCAM_FLASH_MODE_DONE) == 0)
      {
         Err(plucam, "***flash still programming, failing\n");
         ret = -1;
         goto exit_here;
      }
   }

   //set FLASH to normal mode
   flash_mode = LUCAM_FLASH_MODE_NORMAL;
   ret = lucam_reg32_rw(plucam, LUCAM_FLASH_MODE, &flash_mode, 1, USB_DIR_OUT);
   if ( ret < 0 )
   {
      Err(plucam, "***lucam_flash_write--set FLASH mode to normal failed\n");
      goto exit_here;
   }

   // We are ok
   ret = 0;

exit_here:
   return ret;
}

//-----------------------------------------------------------------------
//
static int Is_flash_accessible(plucam_device plucam)
{
   int ret = 0;
   
   if (( plucam == NULL ))
   {
      return -1;
   }

   if (( plucam->miscdatain_pipe == 0 ) || ( plucam->miscdataout_pipe == 0 ))
   {
      return -1;
   }

   if ( plucam->video_pipe == plucam->miscdatain_pipe && plucam->current_stream_state != LUCAM_STREAM_STATE_STOP )
   {
      return -1;
   }

   return ret;
}
#endif


module_init(usb_lucam_init);
module_exit(usb_lucam_exit);


#if  LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18) && LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
module_param_named(debug, lucam_template.debug, int, 0444);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0)
module_param_named(debug, lucam_template.dev_debug, int, 0444);
#endif
MODULE_PARM_DESC(debug, "activates debug info within the v4l2 framework");

module_param(usbport_nr, int, 0);
MODULE_PARM_DESC(usbport_nr, "Instructs to use the usb port # to assign the minor device number");

MODULE_DESCRIPTION( CUSTOM_MANUFACTURER_SHORTNAME " USB camera driver");
MODULE_AUTHOR( CUSTOM_MANUFACTURER_NAME );
MODULE_LICENSE("GPL");


/*
 * Local variables:
 * c-file-style: "linux"
 * End:
 */

