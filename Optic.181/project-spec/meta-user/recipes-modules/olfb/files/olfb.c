/*
 *  olfb.c -- Virtual frame buffer device
 *
 *      Copyright (C) 2002 James Simmons
 *
 *	Copyright (C) 1997 Geert Uytterhoeven
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <linux/fb.h>
#include <linux/init.h>

#include "debug.h"

    /*
     *  RAM we reserve for the frame buffer. This defines the maximum screen
     *  size
     *
     *  The default can be overridden if the driver is compiled as a module
     */

#define VFB_OL							1
#define VFB_OL_PHYSICAL_BASE			0x40000000
#define VFB_OL_SIZE			            0x800000
#define ENABLE_MEM_REGION				0

static void *videomemory;
#if VFB_OL
#define VIDEOMEMSIZE    VFB_OL_SIZE
#else
#define VIDEOMEMSIZE	(1*1024*1024)	/* 1 MB */
#endif
static u_long videocount = 1;
module_param(videocount, ulong, 0);
MODULE_PARM_DESC(videocount, "Frame Buffer Count");

static u_long videomemorysize = VIDEOMEMSIZE;

/**********************************************************************
 *
 * Memory management
 *
 **********************************************************************/
#if VFB_OL
#else
static void *rvmalloc(unsigned long size)
{
	void *mem;
	unsigned long adr;

	size = PAGE_ALIGN(size);
	mem = vmalloc_32(size);
	if (!mem)
		return NULL;

	memset(mem, 0, size); /* Clear the ram out, no junk to the user */
	adr = (unsigned long) mem;
	while (size > 0) {
		SetPageReserved(vmalloc_to_page((void *)adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	return mem;
}

static void rvfree(void *mem, unsigned long size)
{
	unsigned long adr;

	if (!mem)
		return;

	adr = (unsigned long) mem;
	while ((long) size > 0) {
		ClearPageReserved(vmalloc_to_page((void *)adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	vfree(mem);
}
#endif /* VFB_OL */

static struct fb_var_screeninfo vfb_default = {
	.xres =		1920,
	.yres =		1080,
	.xres_virtual =	1920,
	.yres_virtual =	1080,
	.bits_per_pixel = 32,
	.red =		{ 0, 8, 0 },
    .green =	{ 8, 8, 0 },
    .blue =		{ 16, 8, 0 },
//	.activate =	FB_ACTIVATE_TEST,
	.activate =	FB_ACTIVATE_NOW,
    .height =	1920,
    .width =	1080,
//	.pixclock =	20000,
//	.left_margin =	64,
//	.right_margin =	64,
//	.upper_margin =	32,
//	.lower_margin =	32,
//	.hsync_len =	64,
//	.vsync_len =	2,
	.vmode =	FB_VMODE_NONINTERLACED,
};

static struct fb_fix_screeninfo vfb_fix = {
	.id =		"olfb",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
//	.visual =	FB_VISUAL_PSEUDOCOLOR,
//	.xpanstep =	1,
//	.ypanstep =	1,
//	.ywrapstep =	1,
	.accel =	FB_ACCEL_NONE,
};

static bool vfb_enable __initdata = 1;	/* disabled by default */

static int vfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info);
static int vfb_set_par(struct fb_info *info);
static int vfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue, u_int transp, struct fb_info *info);
static int vfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info);

static struct fb_ops vfb_ops = {
//	.fb_read        = fb_sys_read,
//	.fb_write       = fb_sys_write,
	.fb_check_var	= vfb_check_var,
	.fb_set_par		= vfb_set_par,
	.fb_setcolreg	= vfb_setcolreg,
//	.fb_pan_display	= vfb_pan_display,
	.fb_fillrect	= sys_fillrect,
	.fb_copyarea	= sys_copyarea,
	.fb_imageblit	= sys_imageblit,
};

    /*
     *  Internal routines
     */

static u_long get_line_length(int xres_virtual, int bpp)
{
	u_long length;

#if VFB_OL
	length = xres_virtual * bpp;
	length = (length + 31) & ~31;
	length >>= 3;
#else
	length = xres_virtual * bpp;
	length >>= 3;
#endif
	return (length);
}

    /*
     *  Setting the video mode has been split into two parts.
     *  First part, xxxfb_check_var, must not write anything
     *  to hardware, it should only verify and adjust var.
     *  This means it doesn't alter par but it does use hardware
     *  data from it to check this var. 
     */

static int vfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	u_long line_length;

	/*
	 *  FB_VMODE_CONUPDATE and FB_VMODE_SMOOTH_XPAN are equal!
	 *  as FB_VMODE_SMOOTH_XPAN is only used internally
	 */

	if (var->vmode & FB_VMODE_CONUPDATE) {
		var->vmode |= FB_VMODE_YWRAP;
		var->xoffset = info->var.xoffset;
		var->yoffset = info->var.yoffset;
	}

	/*
	 *  Some very basic checks
	 */
	if (!var->xres)
		var->xres = 1;
	if (!var->yres)
		var->yres = 1;
	if (var->xres > var->xres_virtual)
		var->xres_virtual = var->xres;
	if (var->yres > var->yres_virtual)
		var->yres_virtual = var->yres;
	if (var->bits_per_pixel <= 1)
		var->bits_per_pixel = 1;
	else if (var->bits_per_pixel <= 8)
		var->bits_per_pixel = 8;
	else if (var->bits_per_pixel <= 16)
		var->bits_per_pixel = 16;
	else if (var->bits_per_pixel <= 24)
		var->bits_per_pixel = 24;
	else if (var->bits_per_pixel <= 32)
		var->bits_per_pixel = 32;
	else
		return -EINVAL;

	if (var->xres_virtual < var->xoffset + var->xres)
		var->xres_virtual = var->xoffset + var->xres;
	if (var->yres_virtual < var->yoffset + var->yres)
		var->yres_virtual = var->yoffset + var->yres;

	/*
	 *  Memory limit
	 */
	line_length = get_line_length(var->xres_virtual, var->bits_per_pixel);
#if VFB_OL
#else
	if (line_length * var->yres_virtual > videomemorysize)
		return -ENOMEM;
#endif

	/*
	 * Now that we checked it we alter var. The reason being is that the video
	 * mode passed in might not work but slight changes to it might make it 
	 * work. This way we let the user know what is acceptable.
	 */
	switch (var->bits_per_pixel) {
	case 1:
	case 8:
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 0;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 16:		/* RGBA 5551 */
		if (var->transp.length) {
			var->red.offset = 0;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 5;
			var->blue.offset = 10;
			var->blue.length = 5;
			var->transp.offset = 15;
			var->transp.length = 1;
		} else {	/* RGB 565 */
			var->red.offset = 0;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 6;
			var->blue.offset = 11;
			var->blue.length = 5;
			var->transp.offset = 0;
			var->transp.length = 0;
		}
		break;
	case 24:		/* RGB 888 */
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 16;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 32:		/* RGBA 8888 */
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 16;
		var->blue.length = 8;
		var->transp.offset = 24;
		var->transp.length = 8;
		break;
	}
	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;
	var->transp.msb_right = 0;

	return 0;
}

/* This routine actually sets the video mode. It's in here where we
 * the hardware state info->par and fix which can be affected by the 
 * change in par. For this driver it doesn't do much. 
 */
static int vfb_set_par(struct fb_info *info)
{
	info->fix.line_length = 
		get_line_length(info->var.xres_virtual, info->var.bits_per_pixel);
	return 0;
}

    /*
     *  Set a single color register. The values supplied are already
     *  rounded down to the hardware's capabilities (according to the
     *  entries in the var structure). Return != 0 for invalid regno.
     */

static int vfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			 u_int transp, struct fb_info *info)
{
	if (regno >= 256)	/* no. of hw registers */
		return 1;
	/*
	 * Program hardware... do anything you want with transp
	 */

	/* grayscale works only partially under directcolor */
	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue =
		    (red * 77 + green * 151 + blue * 28) >> 8;
	}

	/* Directcolor:
	 *   var->{color}.offset contains start of bitfield
	 *   var->{color}.length contains length of bitfield
	 *   {hardwarespecific} contains width of RAMDAC
	 *   cmap[X] is programmed to (X << red.offset) | (X << green.offset) | (X << blue.offset)
	 *   RAMDAC[X] is programmed to (red, green, blue)
	 *
	 * Pseudocolor:
	 *    var->{color}.offset is 0 unless the palette index takes less than
	 *                        bits_per_pixel bits and is stored in the upper
	 *                        bits of the pixel value
	 *    var->{color}.length is set so that 1 << length is the number of available
	 *                        palette entries
	 *    cmap is not used
	 *    RAMDAC[X] is programmed to (red, green, blue)
	 *
	 * Truecolor:
	 *    does not use DAC. Usually 3 are present.
	 *    var->{color}.offset contains start of bitfield
	 *    var->{color}.length contains length of bitfield
	 *    cmap is programmed to (red << red.offset) | (green << green.offset) |
	 *                      (blue << blue.offset) | (transp << transp.offset)
	 *    RAMDAC does not exist
	 */
#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		red = CNVT_TOHW(red, info->var.red.length);
		green = CNVT_TOHW(green, info->var.green.length);
		blue = CNVT_TOHW(blue, info->var.blue.length);
		transp = CNVT_TOHW(transp, info->var.transp.length);
		break;
	case FB_VISUAL_DIRECTCOLOR:
		red = CNVT_TOHW(red, 8);	/* expect 8 bit DAC */
		green = CNVT_TOHW(green, 8);
		blue = CNVT_TOHW(blue, 8);
		/* hey, there is bug in transp handling... */
		transp = CNVT_TOHW(transp, 8);
		break;
	}
#undef CNVT_TOHW
	/* Truecolor has hardware independent palette */
	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
		u32 v;

		if (regno >= 16)
			return 1;

		v = (red << info->var.red.offset) |
		    (green << info->var.green.offset) |
		    (blue << info->var.blue.offset) |
		    (transp << info->var.transp.offset);
		switch (info->var.bits_per_pixel) {
		case 8:
			break;
		case 16:
			((u32 *) (info->pseudo_palette))[regno] = v;
			break;
		case 24:
		case 32:
			((u32 *) (info->pseudo_palette))[regno] = v;
			break;
		}
		return 0;
	}
	return 0;
}

    /*
     *  Pan or Wrap the Display
     *
     *  This call looks only at xoffset, yoffset and the FB_VMODE_YWRAP flag
     */

static int vfb_pan_display(struct fb_var_screeninfo *var,
			   struct fb_info *info)
{
	if (var->vmode & FB_VMODE_YWRAP) {
		if (var->yoffset >= info->var.yres_virtual ||
		    var->xoffset)
			return -EINVAL;
	} else {
		if (var->xoffset + info->var.xres > info->var.xres_virtual ||
		    var->yoffset + info->var.yres > info->var.yres_virtual)
			return -EINVAL;
	}
	info->var.xoffset = var->xoffset;
	info->var.yoffset = var->yoffset;
	if (var->vmode & FB_VMODE_YWRAP)
		info->var.vmode |= FB_VMODE_YWRAP;
	else
		info->var.vmode &= ~FB_VMODE_YWRAP;
	return 0;
}


#ifndef MODULE
/*
 * The virtual framebuffer driver is only enabled if explicitly
 * requested by passing 'video=olfb:' (or any actual options).
 */
static int __init vfb_setup(char *options)
{
	char *this_opt;

	vfb_enable = 0;

	if (!options)
		return 1;

	vfb_enable = 1;

	if (!*options)
		return 1;

	while ((this_opt = strsep(&options, ",")) != NULL) {
		if (!*this_opt)
			continue;
		/* Test disable for backwards compatibility */
		if (!strcmp(this_opt, "disable"))
			vfb_enable = 0;
	}
	return 1;
}
#endif  /*  MODULE  */

    /*
     *  Initialisation
     */

static int vfb_probe(struct platform_device *dev)
{
    struct fb_info **_info = NULL;
	struct fb_info *info = NULL;
	int retval = -ENOMEM;
    int idx, ei;

    videomemorysize = VIDEOMEMSIZE * videocount;
    printk(KERN_INFO "Video Count=%ld, Memory Size:0x%08lX\n", videocount, videomemorysize);
    _info = vmalloc(sizeof(struct fb_info *)*videocount);
    memset(_info, 0, sizeof(struct fb_info *)*videocount);


	/*
	 * For real video cards we use ioremap.
	 */
#if VFB_OL
# if ENABLE_MEM_REGION
	if (!request_mem_region(VFB_OL_PHYSICAL_BASE, VFB_OL_SIZE, "olfb")) {
		DBG("request_mem_region (failed)\n");
		return -ENXIO;
	}
# endif
	videomemory = ioremap_nocache (VFB_OL_PHYSICAL_BASE, videomemorysize);
//	videomemory = ioremap (VFB_OL_PHYSICAL_BASE, videomemorysize);
	if (videomemory == NULL) {
		DBG("ioremap_nocache(failed)\n");
# if ENABLE_MEM_REGION
		release_mem_region (VFB_OL_SIZE, videomemorysize);
# endif
		return retval;
	}
#else
	if (!(videomemory = rvmalloc(videomemorysize)))
		return retval;
#endif

	/*
	 * VFB must clear memory to prevent kernel info
	 * leakage into userspace
	 * VGA-based drivers MUST NOT clear memory if
	 * they want to be able to take over vgacon
	 */
#if 0
	memset(videomemory, 0, videomemorysize);
#endif
    for (idx=0; idx<videocount; idx++) {
    	_info[idx] = info = framebuffer_alloc(sizeof(u32) * 256, &dev->dev);
    	if (!info)
    		goto err;
    
    	info->screen_base = (char __iomem *)((uint32_t)videomemory + idx*VIDEOMEMSIZE);
    	info->fbops = &vfb_ops;
    
    #if VFB_OL
    	info->var = vfb_default;
    #else
    	retval = fb_find_mode(&info->var, info, NULL, NULL, 0, NULL, 8);
    	if (!retval || (retval == 4))
    		info->var = vfb_default;
    #endif
    
    #if VFB_OL
    	vfb_fix.smem_start = (unsigned long) VFB_OL_PHYSICAL_BASE + idx*VIDEOMEMSIZE;
    	// vfb_fix.smem_start = (unsigned long) videomemory + idx*VIDEOMEMSIZE;
    	vfb_fix.smem_len = VIDEOMEMSIZE;
    #else
    	vfb_fix.smem_start = (unsigned long) videomemory;
    	vfb_fix.smem_len = videomemorysize;
    #endif
    	info->fix = vfb_fix;
    	info->pseudo_palette = info->par;
    	info->par = NULL;
    	info->flags = FBINFO_FLAG_DEFAULT;
    
    	retval = fb_alloc_cmap(&info->cmap, 256, 0);
    	if (retval < 0) {
    		goto err1;
    	}
    
    	retval = register_framebuffer(info);
    	if (retval < 0) {
    		goto err2;
    	}
        fb_info(info, "Frame buffer Address:%p, Size:%X\n", info->screen_base, VIDEOMEMSIZE);
    }    
    platform_set_drvdata(dev, _info);
    fb_info(info, "OL frame buffer device, using %ldK of video memory\n", videomemorysize >> 10);
    return 0;

err2:
    for (ei=0; ei<=idx; ei++) {
        info = _info[idx];
	    fb_dealloc_cmap(&info->cmap);
    }
err1:
    for (ei=0; ei<=idx; ei++) {
        info = _info[idx];
    	framebuffer_release(info);
    }
err:
#if VFB_OL
	iounmap (videomemory);
# if ENABLE_MEM_REGION
	release_mem_region (VFB_OL_PHYSICAL_BASE, videomemorysize);
# endif
#else
	rvfree(videomemory, videomemorysize);
#endif
    vfree(_info);
	return retval;
}

static int vfb_remove(struct platform_device *dev)
{
	struct fb_info **_info = platform_get_drvdata(dev);
    struct fb_info *info;
    int idx;

    for (idx=0; idx<videocount; idx++) {
        info = _info[idx];
    	if (info) {
    		unregister_framebuffer(info);
    		fb_dealloc_cmap(&info->cmap);
    		framebuffer_release(info);
    	}
    }
#if VFB_OL
    iounmap (videomemory);
# if ENABLE_MEM_REGION
    release_mem_region (VFB_OL_PHYSICAL_BASE, videomemorysize);
# endif
#else
    rvfree(videomemory, videomemorysize);
#endif
    vfree(_info);
	return 0;
}

static struct platform_driver vfb_driver = {
	.probe	= vfb_probe,
	.remove = vfb_remove,
	.driver = {
		.name	= "olfb",
	},
};

static struct platform_device *vfb_device = NULL;

static int __init vfb_init(void)
{
	int ret = 0;

	DBG(__DATE__ " " __TIME__ "\n");
#ifndef MODULE
	char *option = NULL;

	if (fb_get_options("olfb", &option))
		return -ENODEV;
	vfb_setup(option);
#endif

	if (!vfb_enable)
		return -ENXIO;

	ret = platform_driver_register(&vfb_driver);

	if (!ret) {
		vfb_device = platform_device_alloc("olfb", 0);

		if (vfb_device)
			ret = platform_device_add(vfb_device);
		else
			ret = -ENOMEM;

		if (ret) {
			platform_device_put(vfb_device);
			platform_driver_unregister(&vfb_driver);
		}
	}

	return ret;
}

module_init(vfb_init);

#ifdef MODULE
static void __exit vfb_exit(void)
{
	platform_device_unregister(vfb_device);
	platform_driver_unregister(&vfb_driver);
    vfb_device = NULL;
}

module_exit(vfb_exit);
#endif				/* MODULE */

MODULE_AUTHOR("hslee");
MODULE_LICENSE("GPL");