#ifdef NOKERNEL
#include "../lucam.h"
#else
#include <linux/lucam.h>
#endif
#include <linux/usb.h>
#include <linux/kernel.h>                         /* printk() */
#include <linux/spinlock.h>

#include <linux/module.h>

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

struct usb_device_id lucam_ids[]=
{
    {USB_DEVICE(VID_HARDWARE_LUCAM2, 0x01A7)},
    {}
};

MODULE_DEVICE_TABLE(usb, lucam_ids);

const char lucam_usb_driver_name[] = "Infinity USB 2.0 Camera 1A7";
