/*
 * helper functions for vmalloc video4linux capture buffers
 *
 * (c) 2007 Mauro Carvalho Chehab, <mchehab@infradead.org>
 * (c) 2010 Albert den Haan, <albert.denhaan@lumenera.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2
 */
/***************************************************
* This version updated to 2.6.38
***************************************************/
#ifdef NOKERNEL
#include "lucam.h"
#else
#include <linux/lucam.h>
#endif

#include "videobuf-vmalloc_32.h"

#if USE_VMALLOC32

#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include <linux/pci.h>
#include <linux/vmalloc.h>
#include <linux/pagemap.h>
#include <asm/page.h>
#include <asm/pgtable.h>

#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,37)
#include <linux/videodev2.h>
#else
#include <linux/videodev.h>
#endif
#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,18)
#include <media/v4l2-dev.h>
#endif

#define MAGIC_VMAL_32_MEM 0x18231223

#define MAGIC_CHECK(is,should)	if (unlikely((is) != (should))) \
	{ printk(KERN_ERR "magic mismatch: %x (expected %x)\n",is,should); BUG(); }

extern struct video_device lucam_template;

#if VIDEODEV_VERSION_CODE < KERNEL_VERSION(4,0,0)
#define dprintk(level, fmt, arg...)	if (lucam_template.debug >= level) \
	printk(KERN_DEBUG "Lucam: " fmt , ## arg)
#else
#define dprintk(level, fmt, arg...)	if (lucam_template.dev_debug >= level) \
	printk(KERN_DEBUG "Lucam: " fmt , ## arg)
#endif


/***************************************************************************/

static void
videobuf_vm_open(struct vm_area_struct *vma)
{
	struct videobuf_mapping *map = vma->vm_private_data;

	dprintk(2,"videobuf_vm_open %p [count=%u,vma=%08lx-%08lx]\n",map,
		map->count,vma->vm_start,vma->vm_end);

	map->count++;
}

static void videobuf_vm_close(struct vm_area_struct *vma)
{
	struct videobuf_mapping *map = vma->vm_private_data;
	struct videobuf_queue *q = map->q;
	int i;

	dprintk(2,"videobuf_vm_close %p [count=%u,vma=%08lx-%08lx]\n", map,
		map->count, vma->vm_start, vma->vm_end);

	map->count--;
	if (0 == map->count) {
		struct videobuf_vmalloc_32_memory *mem;

		dprintk(1, "munmap %p q=%p\n", map, q);
#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,37)
      videobuf_queue_lock(q);
#else
		mutex_lock(&q->vb_lock);
#endif

		/* We need first to cancel streams, before unmapping */
		if (q->streaming)
			videobuf_queue_cancel(q);

		for (i = 0; i < VIDEO_MAX_FRAME; i++) {
			if (NULL == q->bufs[i])
				continue;

			if (q->bufs[i]->map != map)
				continue;

			mem = q->bufs[i]->priv;
			if (mem) {
				/* This callback is called only if kernel has
				   allocated memory and this memory is mmapped.
				   In this case, memory should be freed,
				   in order to do memory unmap.
				 */

				MAGIC_CHECK(mem->magic, MAGIC_VMAL_32_MEM);

				/* vfree is not atomic - can't be
				   called with IRQ's disabled
				 */
				dprintk(1, "%s: buf[%d] freeing (%p)\n",
					__func__, i, mem->vmalloc_32);

				vfree(mem->vmalloc_32);
				mem->vmalloc_32 = NULL;
			}

			q->bufs[i]->map   = NULL;
			q->bufs[i]->baddr = 0;
		}

		kfree(map);

#if VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,37)
      videobuf_queue_unlock(q);
#else
		mutex_unlock(&q->vb_lock);
#endif
	}

	return;
}

static const struct vm_operations_struct videobuf_vm_ops =
{
	.open     = videobuf_vm_open,
	.close    = videobuf_vm_close,
};

/* ---------------------------------------------------------------------
 * vmalloc_32 handlers for the generic methods
 */

/* Allocated area consists on 3 parts:
	struct video_buffer
	struct <driver>_buffer (cx88_buffer, saa7134_buf, ...)
	struct videobuf_dma_sg_memory
 */
#if  VIDEODEV_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void *__videobuf_alloc_vb(size_t size)
#else
static struct videobuf_buffer *__videobuf_alloc_vb(size_t size)
#endif
{
	struct videobuf_vmalloc_32_memory *mem;
	struct videobuf_buffer *vb;

	vb = kzalloc(size+sizeof(*mem),GFP_KERNEL);

   if (!vb)
      return vb;

	mem = vb->priv = ((char *)vb)+size;
	mem->magic=MAGIC_VMAL_32_MEM;

	dprintk(1,"%s: allocated at %p(%ld+%ld) & %p(%ld)\n",
		__func__,vb,(long)sizeof(*vb),(long)size-sizeof(*vb),
		mem,(long)sizeof(*mem));

	return vb;
}

static int __videobuf_iolock (struct videobuf_queue* q,
			      struct videobuf_buffer *vb,
			      struct v4l2_framebuffer *fbuf)
{
	struct videobuf_vmalloc_32_memory *mem = vb->priv;
	int pages;

	BUG_ON(!mem);

	MAGIC_CHECK(mem->magic, MAGIC_VMAL_32_MEM);

	switch (vb->memory) {
	case V4L2_MEMORY_MMAP:
		dprintk(1, "%s memory method MMAP\n", __func__);

		/* All handling should be done by __videobuf_mmap_mapper() */
		if (!mem->vmalloc_32) {
			printk(KERN_ERR "memory is not alloced/mmapped.\n");
			return -EINVAL;
		}
		break;
	case V4L2_MEMORY_USERPTR:
		pages = PAGE_ALIGN(vb->size);

		dprintk(1, "%s memory method USERPTR\n", __func__);

#if 1
		if (vb->baddr) {
			printk(KERN_ERR "USERPTR is currently not supported\n");
			return -EINVAL;
		}
#endif

		/* The only USERPTR currently supported is the one needed for
		   read() method.
		 */

		mem->vmalloc_32 = vmalloc_32_user(pages);
		if (!mem->vmalloc_32) {
			printk(KERN_ERR "vmalloc_32 (%d pages) failed\n", pages);
			return -ENOMEM;
		}
		dprintk(1, "vmalloc_32 is at addr %p (%d pages)\n",
			mem->vmalloc_32, pages);

#if 0
		int rc;
		/* Kernel userptr is used also by read() method. In this case,
		   there's no need to remap, since data will be copied to user
		 */
		if (!vb->baddr)
			return 0;

		/* FIXME: to properly support USERPTR, remap should occur.
		   The code bellow won't work, since mem->vma = NULL
		 */
		/* Try to remap memory */
		rc = remap_vmalloc_32_range(mem->vma, (void *)vb->baddr, 0);
		if (rc < 0) {
			printk(KERN_ERR "mmap: remap failed with error %d. ", rc);
			return -ENOMEM;
		}
#endif

		break;
	case V4L2_MEMORY_OVERLAY:
	default:
		dprintk(1, "%s memory method OVERLAY/unknown\n", __func__);

		/* Currently, doesn't support V4L2_MEMORY_OVERLAY */
		printk(KERN_ERR "Memory method currently unsupported.\n");
		return -EINVAL;
	}

	return 0;
}

#if  VIDEODEV_VERSION_CODE < KERNEL_VERSION(2,6,35)
static int __videobuf_sync(struct videobuf_queue *q,
			   struct videobuf_buffer *buf)
{
	return 0;
}

static int __videobuf_mmap_free(struct videobuf_queue *q)
{
	unsigned int i;

	dprintk(1, "%s\n", __func__);
	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		if (q->bufs[i]) {
			if (q->bufs[i]->map)
				return -EBUSY;
		}
	}

	return 0;
}

static int __videobuf_mmap_mapper(struct videobuf_queue *q,
			 struct vm_area_struct *vma)
{
	struct videobuf_vmalloc_32_memory *mem;
	struct videobuf_mapping *map;
	unsigned int first;
	int retval, pages;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;

	dprintk(1, "%s\n", __func__);
	if (!(vma->vm_flags & VM_WRITE) || !(vma->vm_flags & VM_SHARED))
		return -EINVAL;

	/* look for first buffer to map */
	for (first = 0; first < VIDEO_MAX_FRAME; first++) {
		if (NULL == q->bufs[first])
			continue;

		if (V4L2_MEMORY_MMAP != q->bufs[first]->memory)
			continue;
		if (q->bufs[first]->boff == offset)
			break;
	}
	if (VIDEO_MAX_FRAME == first) {
		dprintk(1,"mmap app bug: offset invalid [offset=0x%lx]\n",
			(vma->vm_pgoff << PAGE_SHIFT));
		return -EINVAL;
	}

	/* create mapping + update buffer list */
	map = kzalloc(sizeof(struct videobuf_mapping), GFP_KERNEL);
	if (NULL == map)
		return -ENOMEM;

	q->bufs[first]->map = map;
	map->start = vma->vm_start;
	map->end   = vma->vm_end;
	map->q     = q;

	q->bufs[first]->baddr = vma->vm_start;

	mem = q->bufs[first]->priv;
	BUG_ON(!mem);
	MAGIC_CHECK(mem->magic, MAGIC_VMAL_32_MEM);

	pages = PAGE_ALIGN(vma->vm_end - vma->vm_start);
	mem->vmalloc_32 = vmalloc_32_user(pages);
	if (!mem->vmalloc_32) {
		printk(KERN_ERR "vmalloc_32 (%d pages) failed\n", pages);
		goto error;
	}
	dprintk(1, "vmalloc_32 is at addr %p (%d pages)\n",
		mem->vmalloc_32, pages);

	/* Try to remap memory */
	retval = remap_vmalloc_range(vma, mem->vmalloc_32, 0);
	if (retval < 0) {
		printk(KERN_ERR "mmap: remap failed with error %d. ", retval);
		vfree(mem->vmalloc_32);
		goto error;
	}

	vma->vm_ops          = &videobuf_vm_ops;
	vma->vm_flags       |= VM_DONTEXPAND | VM_RESERVED;
	vma->vm_private_data = map;

	dprintk(1,"mmap %p: q=%p %08lx-%08lx (%lx) pgoff %08lx buf %d\n",
		map, q, vma->vm_start, vma->vm_end,
		(long int) q->bufs[first]->bsize,
		vma->vm_pgoff, first);

	videobuf_vm_open(vma);

	return 0;

error:
	mem = NULL;
	kfree(map);
	return -ENOMEM;
}
#else
static int __videobuf_mmap_mapper(struct videobuf_queue *q,
				  struct videobuf_buffer *buf,
				  struct vm_area_struct *vma)
{
	struct videobuf_vmalloc_32_memory *mem;
	struct videobuf_mapping *map;
	int retval, pages;

	dprintk(1, "%s\n", __func__);

	/* create mapping + update buffer list */
	map = kzalloc(sizeof(struct videobuf_mapping), GFP_KERNEL);
	if (NULL == map)
		return -ENOMEM;

	buf->map = map;
#if  VIDEODEV_VERSION_CODE < KERNEL_VERSION(2,6,36)
	map->start = vma->vm_start;
	map->end   = vma->vm_end;
#endif
	map->q     = q;

	buf->baddr = vma->vm_start;

	mem = buf->priv;
	BUG_ON(!mem);
	MAGIC_CHECK(mem->magic, MAGIC_VMAL_32_MEM);

	pages = PAGE_ALIGN(vma->vm_end - vma->vm_start);
	mem->vmalloc_32 = vmalloc_32_user(pages);
	if (!mem->vmalloc_32) {
		printk(KERN_ERR "vmalloc_32 (%d pages) failed\n", pages);
		goto error;
	}
	dprintk(1, "vmalloc_32 is at addr %p (%d pages)\n", mem->vmalloc_32, pages);

	/* Try to remap memory */
	retval = remap_vmalloc_range(vma, mem->vmalloc_32, 0);
	if (retval < 0) {
		printk(KERN_ERR "mmap: remap failed with error %d. ", retval);
		vfree(mem->vmalloc_32);
		goto error;
	}

	vma->vm_ops          = &videobuf_vm_ops;
#if  LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
	vma->vm_flags       |= VM_DONTEXPAND | VM_RESERVED;
#else
	vma->vm_flags       |= VM_DONTEXPAND | VM_DONTDUMP;
#endif
	vma->vm_private_data = map;

	dprintk(1, "mmap %p: q=%p %08lx-%08lx (%lx) pgoff %08lx buf %d\n",
		map, q, vma->vm_start, vma->vm_end,
		(long int)buf->bsize,
		vma->vm_pgoff, buf->i);

	videobuf_vm_open(vma);

	return 0;

error:
	mem = NULL;
	kfree(map);
	return -ENOMEM;
}
#endif

#if  VIDEODEV_VERSION_CODE < KERNEL_VERSION(2,6,35)
static int __videobuf_copy_to_user ( struct videobuf_queue *q,
				char __user *data, size_t count,
				int nonblocking )
{
	struct videobuf_vmalloc_32_memory *mem=q->read_buf->priv;
	BUG_ON (!mem);
	MAGIC_CHECK(mem->magic,MAGIC_VMAL_32_MEM);

	BUG_ON (!mem->vmalloc_32);

	/* copy to userspace */
	if (count > q->read_buf->size - q->read_off)
		count = q->read_buf->size - q->read_off;

	if (copy_to_user(data, mem->vmalloc_32+q->read_off, count))
		return -EFAULT;

	return count;
}

static int __videobuf_copy_stream ( struct videobuf_queue *q,
				char __user *data, size_t count, size_t pos,
				int vbihack, int nonblocking )
{
	unsigned int  *fc;
	struct videobuf_vmalloc_32_memory *mem=q->read_buf->priv;
	BUG_ON (!mem);
	MAGIC_CHECK(mem->magic,MAGIC_VMAL_32_MEM);

	if (vbihack) {
		/* dirty, undocumented hack -- pass the frame counter
			* within the last four bytes of each vbi data block.
			* We need that one to maintain backward compatibility
			* to all vbi decoding software out there ... */
		fc  = (unsigned int*)mem->vmalloc_32;
		fc += (q->read_buf->size>>2) -1;
		*fc = q->read_buf->field_count >> 1;
		dprintk(1,"vbihack: %d\n",*fc);
	}

	/* copy stuff using the common method */
	count = __videobuf_copy_to_user (q,data,count,nonblocking);

	if ( (count==-EFAULT) && (0 == pos) )
		return -EFAULT;

	return count;
}
#endif

static struct videobuf_qtype_ops qops = {
	.magic        = MAGIC_QTYPE_OPS,

#if  VIDEODEV_VERSION_CODE < KERNEL_VERSION(2,6,36)
	.alloc        = __videobuf_alloc_vb,
#else
	.alloc_vb     = __videobuf_alloc_vb,
#endif
	.iolock       = __videobuf_iolock,
#if  VIDEODEV_VERSION_CODE < KERNEL_VERSION(2,6,35)
	.sync         = __videobuf_sync,
	.mmap_free    = __videobuf_mmap_free,
#endif
	.mmap_mapper  = __videobuf_mmap_mapper,
#if  VIDEODEV_VERSION_CODE < KERNEL_VERSION(2,6,35)
	.video_copy_to_user = __videobuf_copy_to_user,
	.copy_stream  = __videobuf_copy_stream,
#endif
#if  VIDEODEV_VERSION_CODE < KERNEL_VERSION(2,6,35)
	.vmalloc      = videobuf_to_vmalloc_32,
#else
	.vaddr        = videobuf_to_vmalloc_32,
#endif
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
      )
{
	videobuf_queue_core_init(q, ops, dev, irqlock, type, field, msize,
				 priv, &qops
#if  VIDEODEV_VERSION_CODE >= KERNEL_VERSION(2,6,37)
         , ext_lock
#endif
      );
}


void *videobuf_to_vmalloc_32 (struct videobuf_buffer *buf)
{
	struct videobuf_vmalloc_32_memory *mem=buf->priv;
	BUG_ON (!mem);
	MAGIC_CHECK(mem->magic,MAGIC_VMAL_32_MEM);

	return mem->vmalloc_32;
}

void videobuf_vmalloc_32_free (struct videobuf_buffer *buf)
{
	struct videobuf_vmalloc_32_memory *mem = buf->priv;

	/* mmapped memory can't be freed here, otherwise mmapped region
	   would be released, while still needed. In this case, the memory
	   release should happen inside videobuf_vm_close().
	   So, it should free memory only if the memory were allocated for
	   read() operation.
	 */
	if ((buf->memory != V4L2_MEMORY_USERPTR) || (buf->baddr == 0))
		return;

	if (!mem)
		return;

	MAGIC_CHECK(mem->magic, MAGIC_VMAL_32_MEM);

	vfree(mem->vmalloc_32);
	mem->vmalloc_32 = NULL;

	return;
}

/*
 * Local variables:
 * c-basic-offset: 8
 * End:
 */
#endif

