/****************************************************************
 * luldr.c : Lumenera camera firmware loading module
 *
 * Copyright 2005-2009 Lumenera Corporation
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
 *****************************************************************/
#ifdef NOKERNEL
#include "luldr.h"
#else
#include <linux/luldr.h>
#endif
#include <linux/module.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#include <linux/moduleparam.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,0)
#include <linux/slab.h>
#else
#include <linux/malloc.h>
#endif
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/proc_fs.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
#include <linux/smp_lock.h>
#endif
#include <linux/vmalloc.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/page.h>
#include <linux/capability.h>
#include <linux/poll.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#include <linux/moduleparam.h>
#endif


static int luldr_trace_state = TRACE_MODULE | TRACE_PROBE |TRACE_REG | TRACE_OPEN|TRACE_ERROR|TRACE_INFO;

extern struct _FirmwareList2 FirmwareList[];
extern int FirmwareListCount;

#ifndef CUSTOM_MANUFACTURER_SHORTNAME
#define CUSTOM_MANUFACTURER_SHORTNAME "Lumenera"
#endif
#ifndef CUSTOM_MANUFACTURER_NAME
#define CUSTOM_MANUFACTURER_NAME "Lumenera Corporation"
#endif

/****************************************************************************
 *
 *  Find a spare luldr_device structure for the current device
 *  -- Support up to MAX_CAMERA_NUMBER cameras.
 *
 ***************************************************************************/
static luldr_device luldr_dev[MAX_CAMERA_NUMBER];

static int luldr_find_place (void)
{
    int i;
    for (i = 0; i < MAX_CAMERA_NUMBER; i++)
    {
        pluldr_device s = &luldr_dev[i];
        if (!s->udev)
            return i;
    }
    return -1;
}

//-----------------------------------------------------------------------------
//
PINTEL_HEX_RECORD LucamFindHexRecord(unsigned short pid, unsigned short did)
{
   int i;

   for(i=0;i<FirmwareListCount ; i++)
   {
      if (FirmwareList[i].Pid == pid && FirmwareList[i].Did == did)
      {
         return FirmwareList[i].Record;
      }
   }
   return NULL;
}

/**************************************************************************************************
 *
 *  Access fx2 memory. Similar to register access despite different format of the control message
 *
 **************************************************************************************************/

static int luldr_fx2mem_rw(struct usb_device *udev,  __u16 start_addr, BYTE* buf, int len, BYTE dir)
{
    int ret,request_type,pipe;
    BYTE *transfer_buffer =  kmalloc (len, GFP_KERNEL);

    if (!transfer_buffer)
    {
        Err("luldr_fx2mem_rw: kmalloc(%d) failed.", len);
        return -ENOMEM;
    }
    //0x40
    request_type= USB_TYPE_VENDOR | USB_RECIP_DEVICE | dir;

    if(dir==USB_DIR_IN)                           //read
        pipe= usb_rcvctrlpipe( udev, 0);
    else                                          //write
    {
        pipe= usb_sndctrlpipe( udev, 0);
        memcpy (transfer_buffer, (BYTE*)buf, len);
    }

    ret=usb_control_msg(udev, pipe,
        FX2_FIRMWARE_LOAD, request_type,
        start_addr,                               //<--- value = address
        0, transfer_buffer, len, TIMEOUT);

    if(ret>=0 && dir==USB_DIR_IN)
        memcpy ((BYTE*)buf, transfer_buffer, len);
    kfree (transfer_buffer);
    return ret;
}


/********************************************************************************
Reset or set CPU by writing to CPUCS register
**********************************************************************************/
static int luldr_cpu_reset(struct usb_device *udev,BYTE state)
{
    int rc;
    BYTE value;
#if VERIFY_FIRMWARE
    int retries = MAX_RETRIES;
#endif

    Info("luldr_cpu_reset() called:0x%04x\n",state);
    if(state!=FX2_SET_CPU)
        value= FX2_RESET_CPU;
    else
        value= FX2_SET_CPU;
#if VERIFY_FIRMWARE
    do
    {
       rc=luldr_fx2mem_rw(udev,  FX2_REG_CPUCS, &value, 1, USB_DIR_OUT);
       if (rc >= 0) break;
       Err("reset/set CPU failed: %d, retrying\n", rc);
    } while (retries--);
#else
    rc=luldr_fx2mem_rw(udev,  FX2_REG_CPUCS, &value, 1, USB_DIR_OUT);
#endif
    if(rc<0) Err("reset/set CPU failed: %d\n", rc);
    return rc;
}


/******************************************************************************
download firmware to fx2 ?
******************************************************************************/
static int luldr_firmware_download(pluldr_device pluldr)
{
    int ret;
    unsigned short revId, productId;
#if VERIFY_FIRMWARE
    int retries;
    int i;
    BYTE firmware_chunk[MAX_DOWNLOAD_PACKET_SIZE+1];
    int reverify;
#endif

    PINTEL_HEX_RECORD ptrToStartOfRecord;
    PINTEL_HEX_RECORD ptr;
    struct usb_device *udev = pluldr->udev;
    Info("luldr_firmware_download() called \n");
    if(!pluldr || !udev) return -1;

    revId = udev->descriptor.bcdDevice;
    productId = udev->descriptor.idProduct;

#if !defined (__LITTLE_ENDIAN)
    /* Some host controller drivers on big endian systems do not swap the usb descriptors */
    if (udev->descriptor.idVendor == SWAP16(VID_HARDWARE_LULDR) || udev->descriptor.idVendor == SWAP16(VID_HARDWARE_LULDR2))
    {
        Info("USB host controller driver did not swap the 16 bits fields of the USB descriptors\n");
        revId = SWAP16(revId);
        productId = SWAP16(productId);
    }
#endif

   ptrToStartOfRecord = LucamFindHexRecord(productId, revId);
   if (ptrToStartOfRecord == NULL)
   {
      Err("Board rev id unsupported\n");
      return -1;
      //revId = 0;
   }

    ret= luldr_cpu_reset(udev,FX2_RESET_CPU);     //reset cpu
    if(ret<0) return ret;

   ptr = ptrToStartOfRecord;
   
    //warning !!! you must add an item, whose Type!=0, at the end of HEX_RECORD array.
    while (ptr->Type == 0)
    {
#if VERIFY_FIRMWARE
        retries = MAX_RETRIES;
        do
        {
           ret = luldr_fx2mem_rw(udev, ptr->Address, ptr->Data, ptr->Length,USB_DIR_OUT);
           if (ret >= 0) break;
           Err("error downloading chunk of firmware with %d, retrying\n", ret);
        } while (retries--);
#else
        ret = luldr_fx2mem_rw(udev, ptr->Address, ptr->Data, ptr->Length,USB_DIR_OUT);
#endif
        if (ret < 0)
        {
            Err("luldr_firmware_download failed (%d %04X %p %d)", ret, ptr->Address, ptr->Data, ptr->Length);
            break;
        }
        ptr++;
    }

#if VERIFY_FIRMWARE
    /*
     * Here not only we verify that the firmware was downlaoded correctly but
     * in the case of a failure we retry again.
     */
    Info("Verifying the firmware\n");
    reverify = 0;
   ptr = ptrToStartOfRecord;
    while (ptr->Type == 0)
    {
        retries = MAX_RETRIES;
        if (ptr->Length > MAX_DOWNLOAD_PACKET_SIZE)
        {
           Err("Compilation error: MAX_DOWNLOAD_PACKET_SIZE too small\n");
           break;
        }

        do
        {
           /* BUG in IC: If the address is odd it will clamp it even. */
           if (ptr->Address & 1)
           {
              ret = luldr_fx2mem_rw(udev, ptr->Address & ~1, firmware_chunk, ptr->Length+1, USB_DIR_IN);
           }
           else
           {
              ret = luldr_fx2mem_rw(udev, ptr->Address, firmware_chunk, ptr->Length, USB_DIR_IN);
           }
           if (ret >= 0) break;
           Err("error loading back chunk of firmware with %d, retrying\n", ret);
        } while (retries--);

        if (ret < 0)
        {
           Err("luldr_firmware_download readback failed (%d %04X %p %d)", ret, ptr->Address, ptr->Data, ptr->Length);
           break;
        }

        for (i = 0 ; i < ptr->Length ; i++)
        {
           if (ptr->Data[i] != firmware_chunk[i+(ptr->Address&1)])
           {
              break;
           }
        }

        if (i != ptr->Length)
        {
             Err("Firmware verify mismatch, addr:0x%04x, to program:0x%02x, readback:0x%02x\n", ptr->Address+i, ptr->Data[i],firmware_chunk[i]);
             retries = 3;
             do
             {
                 ret = luldr_fx2mem_rw(udev, ptr->Address, ptr->Data, ptr->Length, USB_DIR_OUT);
                 if (ret >= 0) break;
                 Err("error redownloading chunk of firmware with %d, retrying\n", ret);
             } while (retries--);
             if (ret < 0)
             {
                 Err("luldr_firmware_redownload failed (%d %04X %p %d)", ret, ptr->Address, ptr->Data, ptr->Length);
                 break;
             }
             if (reverify)
             {
                Err("Reverification failed, givin up\n");
                break;
             }
             else
             {
                reverify = 1;
                continue; // reverify
             }
        }
        ptr++;
        reverify = 0;
    }
#endif

    ret=luldr_cpu_reset(udev,FX2_SET_CPU);
    return ret;
}




/****************************************************************************
 *
 *  USB routines
 *
 ***************************************************************************/
extern struct usb_device_id luldr_ids[];


/****************************************************************************/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
static int luldr_probe(struct usb_interface *interface, const struct usb_device_id *id)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,0)
static void *luldr_probe(struct usb_device *udev, unsigned int ifnum, const struct usb_device_id *id)
#else /* 2.2.x */
static void *luldr_probe(struct usb_device *udev, unsigned int ifnum)
#endif
{
    int ret;
    struct usb_interface_descriptor *ifacedesc;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    unsigned int ifnum;
    struct usb_device *udev = interface_to_usbdev(interface);
    static const int ERROR_CODE = -ENODEV;
#else
    static void * const ERROR_CODE = NULL;
#endif
    pluldr_device pluldr=NULL;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,0)
	/* Check if the device has a product number that we support */
	struct usb_device_id *i;
	for (i=luldr_ids; i->idVendor; i++) {
		if (udev->descriptor.idVendor == i->idVendor &&
		    udev->descriptor.idProduct == i->idProduct) break;
	}
	if (!i->idVendor) return ERROR_CODE;
#endif

	/* We don't handle multi-config cameras */
	if (udev->descriptor.bNumConfigurations != 1) return ERROR_CODE;

	/*
	 * Checking vendor/product is not enough
	 * In case on QuickCam Web the audio is at class 1 and subclass 1/2.
	 * one /dev/dsp and one /dev/mixer
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
	ifacedesc = &interface->altsetting[0].desc;
	ifnum  = ifacedesc->bInterfaceNumber;
#else
	ifacedesc = &udev->actconfig->interface[ifnum].altsetting[0];
#endif

    Info("luldr_probe() called [0x%04X 0x%04X], ifn=%d\n", udev->descriptor.idVendor, udev->descriptor.idProduct, ifnum);
    /* We don't handle multi-config cameras */
    /*Check vendor ID */
    /*Only One Interface for Lucam*/
    if (udev->descriptor.bNumConfigurations != 1||
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
        udev->config[0].desc.bNumInterfaces != 1     ||
#else
        udev->config[0].bNumInterfaces != 1     ||
#endif
        ifnum!=0)
    {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
        return ERROR_CODE;
#else
        return NULL;
#endif
    }
    /*find a spare slot, support upto MAX_CAMERA_NUMBER Lucam*/
    ret=luldr_find_place();
    if(ret<0)
    {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
        return ERROR_CODE;
#else
        return NULL;
#endif
    }
    /*Do something serious*/
    pluldr=&luldr_dev[ret];
    pluldr->unplugged = 0;
                                                  //lock
    down (&pluldr->mutex);
    pluldr->udev = udev;
    pluldr->index=ret;

#if 0
    /* set configuration -- must be done prior to downloading firmware*/
    if (usb_set_configuration (udev, udev->config[0].desc.bConfigurationValue) < 0)
    {
        Err("set_configuration failed, %d\n", udev->config[0].desc.bConfigurationValue);
        goto error;
    }
    else
    {
        Err("set_configuration-- %d\n", udev->config[0].desc.bConfigurationValue);
    }
#endif

    //to check if the camera mono or color
    if (luldr_firmware_download(pluldr) < 0)
    {
        goto error;
    }
    up (&pluldr->mutex);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    if (!pluldr) return ERROR_CODE;
    usb_set_intfdata(interface, pluldr);	/* FIXME: why? */
    return 0;
#else
    return pluldr;
#endif
error:
    if(pluldr)
    {
        pluldr->udev=NULL;
        up (&pluldr->mutex);
        pluldr=NULL;
    }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    return ERROR_CODE;
#else
    return NULL;
#endif
}


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
static void
luldr_disconnect(struct usb_interface *interface)
#else
static void
luldr_disconnect(struct usb_device *usbdev, void *ptr)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
	pluldr_device pluldr = usb_get_intfdata(interface);
	struct usb_device *usbdev = interface_to_usbdev(interface);
#else
    pluldr_device pluldr = (pluldr_device) ptr;
#endif
    Info("luldr_disconnect() called \n");
    //DECLARE_WAITQUEUE(wait, current);
    if (pluldr == NULL)
    {
        Err("luldr_disconnect() Called without private pointer.\n");
        return;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
	usb_set_intfdata(interface, NULL);	/* FIXME: why? */
#endif
    if (pluldr->udev == NULL)
    {
        Err("lucam_disconnect() already called for %p\n", pluldr);
        return;
    }
    if (pluldr->udev != usbdev)
    {
        Err("lucam_disconnect() Woops: pointer mismatch udev/lucam.\n");
        return;
    }
    pluldr->unplugged = 1;
    wake_up_interruptible(&pluldr->reading_queue);
    Info("Disconnected \n");
    pluldr-> udev = NULL;
    //MOD_DEC_USE_COUNT;
}


extern const char luldr_usb_driver_name[];

static struct usb_driver luldr_driver =
{
    //I did not test with 2.4.19, so I am not sure which version this
    //change was first made, but I know it is in 2.4.20 but not in 2.4.18.
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,4,18) && LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
    .owner       = THIS_MODULE,
#endif
    .name        = luldr_usb_driver_name,
    .probe       = luldr_probe,
    .disconnect  = luldr_disconnect,
    .id_table    = luldr_ids
};

/****************************************************************************
 *
 *  Module routines
 *
 ***************************************************************************/
static int trace=-1;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
module_param(trace, int, 0);
#else
MODULE_PARM(trace, "i");
#endif
MODULE_PARM_DESC(trace, "For debugging");

MODULE_DESCRIPTION( CUSTOM_MANUFACTURER_NAME " USB2 camera loader driver");
MODULE_AUTHOR( CUSTOM_MANUFACTURER_NAME );

MODULE_LICENSE("GPL");


static int __init luldr_init(void)
{
    int i=0;
    if(trace>=0)
    {
        Info("Trace options:0x%04x\n", trace);
        luldr_trace_state=trace;
    }
    for (i = 0; i < MAX_CAMERA_NUMBER; i++)
    {
        pluldr_device pluldr = &luldr_dev[i];
        memset(pluldr, 0, sizeof (luldr_device));
        pluldr->udev=NULL;
        sema_init (&pluldr->mutex, 1);
        init_waitqueue_head (&pluldr->remove_ok);
        init_waitqueue_head (&pluldr->reading_queue);
    }

    if (usb_register(&luldr_driver) < 0)
        return -1;
    Info("luldr driver version %s registered\n", LULDR_VERSION);
    return 0;
}




static void __exit luldr_exit(void)
{
    usb_deregister(&luldr_driver);		/* Will also call luldr_usb_disconnect() if necessary */
    Info("luldr driver version %s unregistered\n", LULDR_VERSION);
}

module_init(luldr_init);
module_exit(luldr_exit);
