#include <linux/init.h>
#include <linux/linkage.h>
#include <linux/const.h>
#include <asm/page.h>

	__PAGE_ALIGNED_DATA

	.globl vdso32_start, vdso32_end
	.balign PAGE_SIZE
vdso32_start:
	.incbin "arch/arm/kernel/vdso32/vdso32.so"
	.balign PAGE_SIZE
vdso32_end:

	.previous
