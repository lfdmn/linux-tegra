/*
 * helper functions for vmalloc capture buffers
 *
 * The functions expect the hardware being able to scatter gatter
 * (i.e. the buffers are not linear in physical memory, but fragmented
 * into PAGE_SIZE chunks).  They also assume the driver does not need
 * to touch the video data.
 *
 * (c) 2007 Mauro Carvalho Chehab, <mchehab@infradead.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2
 */
#ifndef _VIDEOBUF_VMALLOC_32_H
#define _VIDEOBUF_VMALLOC_32_H

#ifdef NOKERNEL
#include "lucam.h"
#else
#include <linux/lucam.h>
#endif

#include <media/videobuf-core.h>

#if USE_VMALLOC32
/* --------------------------------------------------------------------- */

struct videobuf_vmalloc_32_memory
{
	u32                 magic;

	void                *vmalloc_32;

	/* remap_vmalloc_32_range seems to need to run after mmap() on some cases */
	struct vm_area_struct *vma;
};

void videobuf_queue_vmalloc_32_init(struct videobuf_queue* q,
			 const struct videobuf_queue_ops *ops,
			 struct device *dev,
			 spinlock_t *irqlock,
			 enum v4l2_buf_type type,
			 enum v4l2_field field,
			 unsigned int msize,
			 void *priv
#if  VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,37)
         , struct mutex *ext_lock
#endif
      );

void *videobuf_to_vmalloc_32 (struct videobuf_buffer *buf);

void videobuf_vmalloc_32_free (struct videobuf_buffer *buf);
#endif

#endif
