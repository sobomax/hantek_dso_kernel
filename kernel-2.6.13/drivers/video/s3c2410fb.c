/*
 * linux/drivers/video/s3c2410fb.c
 *	Copyright (c) Arnaud Patard, Ben Dooks
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	    S3C2410 LCD Controller Frame Buffer Driver
 *	    based on skeletonfb.c, sa1100fb.c and others
 *
 * ChangeLog
 *
 * 2005-12-26   : godori(www.aesop-embedded.org)<ghcstop@gmail.com>
 *      - pixclock routine change(why?: because 6.4", 4", 7" lcd )
 *
 * 2005-12   : godori(www.aesop-embedded.org)<ghcstop@gmail.com>
 *      - LCD use static memory(no dynamic dma alloc)
 *      - register setting control
 * 
 * 2005-04-07: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - u32 state -> pm_message_t state
 *      - S3C2410_{VA,SZ}_LCD -> S3C24XX
 *
 * 2005-03-15: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - Removed the ioctl
 *      - use readl/writel instead of __raw_writel/__raw_readl
 *
 * 2004-12-04: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - Added the possibility to set on or off the
 *      debugging mesaages
 *      - Replaced 0 and 1 by on or off when reading the
 *      /sys files
 *
 * 2005-03-23: Ben Dooks <ben-linux@fluff.org>
 *	- added non 16bpp modes
 *	- updated platform information for range of x/y/bpp
 *	- add code to ensure palette is written correctly
 *	- add pixel clock divisor control
 *
 * 2004-11-11: Arnaud Patard <arnaud.patard@rtp-net.org>
 * 	- Removed the use of currcon as it no more exist
 * 	- Added LCD power sysfs interface
 *
 * 2004-11-03: Ben Dooks <ben-linux@fluff.org>
 *	- minor cleanups
 *	- add suspend/resume support
 *	- s3c2410fb_setcolreg() not valid in >8bpp modes
 *	- removed last CONFIG_FB_S3C2410_FIXED
 *	- ensure lcd controller stopped before cleanup
 *	- added sysfs interface for backlight power
 *	- added mask for gpio configuration
 *	- ensured IRQs disabled during GPIO configuration
 *	- disable TPAL before enabling video
 *
 * 2004-09-20: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - Suppress command line options
 *
 * 2004-09-15: Arnaud Patard <arnaud.patard@rtp-net.org>
 * 	- code cleanup
 *
 * 2004-09-07: Arnaud Patard <arnaud.patard@rtp-net.org>
 * 	- Renamed from h1940fb.c to s3c2410fb.c
 * 	- Add support for different devices
 * 	- Backlight support
 *
 * 2004-09-05: Herbert Pötzl <herbert@13thfloor.at>
 *	- added clock (de-)allocation code
 *	- added fixem fbmem option
 *
 * 2004-07-27: Arnaud Patard <arnaud.patard@rtp-net.org>
 *	- code cleanup
 *	- added a forgotten return in h1940fb_init
 *
 * 2004-07-19: Herbert Pötzl <herbert@13thfloor.at>
 *	- code cleanup and extended debugging
 *
 * 2004-07-15: Arnaud Patard <arnaud.patard@rtp-net.org>
 *	- First version
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>

#include <asm/mach/map.h>
#include <asm/arch/regs-lcd.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/fb.h>
#include <asm/hardware/clock.h>

#include <asm/arch/sbc2440v3-map.h> // ghcstop MMU setting

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#include "s3c2410fb.h"


static struct s3c2410fb_info	   info;


static struct s3c2410fb_mach_info *mach_info; 

/* Debugging stuff */
#ifdef CONFIG_FB_S3C2410_DEBUG
static int debug	   = 1;
#else
static int debug	   = 0;
#endif

#define dprintk(msg...)	if (debug) { printk(KERN_DEBUG "s3c2410fb: " msg); }


//#define GDEBUG // ghcstop
#ifdef  GDEBUG
#    define gprintk( x... )  printk( x )
#else
#    define gprintk( x... )
#endif



/* useful functions */

static inline struct s3c2410fb_info *fb_to_s3cfb(struct fb_info *info)
{
	return container_of(info, struct s3c2410fb_info, fb);
}

/* s3c2410fb_set_lcdaddr
 *
 * initialise lcd controller address pointers
*/

static void s3c2410fb_set_lcdaddr(struct s3c2410fb_info *fbi)
{
	struct fb_var_screeninfo *var = &fbi->fb.var;
	unsigned long saddr1, saddr2, saddr3;

	saddr1  = fbi->fb.fix.smem_start >> 1;
	saddr2  = fbi->fb.fix.smem_start;
	saddr2 += (var->xres * var->yres * var->bits_per_pixel)/8;
	saddr2>>= 1;

	saddr3 =  S3C2410_OFFSIZE(0) | S3C2410_PAGEWIDTH(var->xres);

	gprintk("LCDSADDR1 = 0x%08lx\n", saddr1);
	gprintk("LCDSADDR2 = 0x%08lx\n", saddr2);
	gprintk("LCDSADDR3 = 0x%08lx\n", saddr3);

	writel(saddr1, S3C2410_LCDSADDR1);
	writel(saddr2, S3C2410_LCDSADDR2);      
	writel(saddr3, S3C2410_LCDSADDR3);
}

/* s3c2410fb_calc_pixclk()
 *
 * calculate divisor for clk->pixclk
*/
static unsigned int s3c2410fb_calc_pixclk(struct s3c2410fb_info *fbi,
					  unsigned long pixclk)
{
	unsigned long clk = clk_get_rate(fbi->clk); // hclk get
	unsigned long long div;

	div = (unsigned long long)clk / pixclk;
	return div;
}

/*
 *	s3c2410fb_check_var():
 *	Get the video params out of 'var'. If a value doesn't fit, round it up,
 *	if it's too big, return -EINVAL.
 *
 */
static int s3c2410fb_check_var(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	struct s3c2410fb_info *fbi = fb_to_s3cfb(info);

	gprintk("check_var(var=%p, info=%p)\n", var, info);

	/* validate x/y resolution */

	if (var->yres > fbi->mach_info->yres.max)
		var->yres = fbi->mach_info->yres.max;
	else if (var->yres < fbi->mach_info->yres.min)
		var->yres = fbi->mach_info->yres.min;

	if (var->xres > fbi->mach_info->xres.max)
		var->yres = fbi->mach_info->xres.max;
	else if (var->xres < fbi->mach_info->xres.min)
		var->xres = fbi->mach_info->xres.min;

	/* validate bpp */

	if (var->bits_per_pixel > fbi->mach_info->bpp.max)
		var->bits_per_pixel = fbi->mach_info->bpp.max;
	else if (var->bits_per_pixel < fbi->mach_info->bpp.min)
		var->bits_per_pixel = fbi->mach_info->bpp.min;

	/* set r/g/b positions */

	if (var->bits_per_pixel == 16) {
		var->red.offset		= 11;
		var->green.offset	= 5;
		var->blue.offset	= 0;
		var->red.length		= 5;
		var->green.length	= 6;
		var->blue.length	= 5;
		var->transp.length	= 0;
	} else {
		var->red.length		= 8;
		var->red.offset		= 0;
		var->green.length	= 0;
		var->green.offset	= 8;
		var->blue.length	= 8;
		var->blue.offset	= 0;
		var->transp.length	= 0;
	}

	return 0;
}

/* s3c2410fb_activate_var
 *
 * activate (set) the controller from the given framebuffer
 * information
*/

static int s3c2410fb_activate_var(struct s3c2410fb_info *fbi,
				  struct fb_var_screeninfo *var)
{
	fbi->regs.lcdcon1 &= ~S3C2410_LCDCON1_MODEMASK;

	gprintk("%s: var->xres  = %d\n", __FUNCTION__, var->xres);
	gprintk("%s: var->yres  = %d\n", __FUNCTION__, var->yres);
	gprintk("%s: var->bpp   = %d\n", __FUNCTION__, var->bits_per_pixel);

	switch (var->bits_per_pixel) {
	case 1:
		fbi->regs.lcdcon1 |= S3C2410_LCDCON1_TFT1BPP;
		break;
	case 2:
		fbi->regs.lcdcon1 |= S3C2410_LCDCON1_TFT2BPP;
		break;
	case 4:
		fbi->regs.lcdcon1 |= S3C2410_LCDCON1_TFT4BPP;
		break;
	case 8:
		fbi->regs.lcdcon1 |= S3C2410_LCDCON1_TFT8BPP;
		break;
	case 16:
		fbi->regs.lcdcon1 |= S3C2410_LCDCON1_TFT16BPP;
		break;

	default:
		/* invalid pixel depth */
		dev_err(fbi->dev, "invalid bpp %d\n", var->bits_per_pixel);
	}

	/* check to see if we need to update sync/borders */

	if (!fbi->mach_info->fixed_syncs) {
		gprintk("setting vert: up=%d, low=%d, sync=%d\n",
			var->upper_margin, var->lower_margin,
			var->vsync_len);

		gprintk("setting horz: lft=%d, rt=%d, sync=%d\n",
			var->left_margin, var->right_margin,
			var->hsync_len);

		fbi->regs.lcdcon2 = 
			S3C2410_LCDCON2_VBPD(var->upper_margin - 1) |
			S3C2410_LCDCON2_VFPD(var->lower_margin - 1) |
			S3C2410_LCDCON2_VSPW(var->vsync_len - 1);

		fbi->regs.lcdcon3 = 
			S3C2410_LCDCON3_HBPD(var->right_margin - 1) |
			S3C2410_LCDCON3_HFPD(var->left_margin - 1);

		fbi->regs.lcdcon4 &= ~S3C2410_LCDCON4_HSPW(0xff);
		fbi->regs.lcdcon4 |=  S3C2410_LCDCON4_HSPW(var->hsync_len - 1);
	}

	/* update X/Y info */

	fbi->regs.lcdcon2 &= ~S3C2410_LCDCON2_LINEVAL(0x3ff);
	fbi->regs.lcdcon2 |=  S3C2410_LCDCON2_LINEVAL(var->yres - 1);

	fbi->regs.lcdcon3 &= ~S3C2410_LCDCON3_HOZVAL(0x7ff);
	fbi->regs.lcdcon3 |=  S3C2410_LCDCON3_HOZVAL(var->xres - 1);

	gprintk("%s: pixclock = %u\n", __FUNCTION__, var->pixclock);
	
	if (var->pixclock > 0) {
		int clkdiv = s3c2410fb_calc_pixclk(fbi, var->pixclock);

		clkdiv = (clkdiv / 2) -1;
		if (clkdiv < 0)
			clkdiv = 0;


		gprintk("CLKVAL = %d\n", clkdiv);

		fbi->regs.lcdcon1 &= ~S3C2410_LCDCON1_CLKVAL(0x3ff);
		fbi->regs.lcdcon1 |=  S3C2410_LCDCON1_CLKVAL(clkdiv);
	}

	/* write new registers */

	gprintk("new register set:\n");
	gprintk("lcdcon[1] = 0x%08lx\n", fbi->regs.lcdcon1);
	gprintk("lcdcon[2] = 0x%08lx\n", fbi->regs.lcdcon2);
	gprintk("lcdcon[3] = 0x%08lx\n", fbi->regs.lcdcon3);
	gprintk("lcdcon[4] = 0x%08lx\n", fbi->regs.lcdcon4);
	gprintk("lcdcon[5] = 0x%08lx\n", fbi->regs.lcdcon5);

	writel(fbi->regs.lcdcon1 & ~S3C2410_LCDCON1_ENVID, S3C2410_LCDCON1);
	writel(fbi->regs.lcdcon2, S3C2410_LCDCON2);
	writel(fbi->regs.lcdcon3, S3C2410_LCDCON3);
	writel(fbi->regs.lcdcon4, S3C2410_LCDCON4);
	writel(fbi->regs.lcdcon5, S3C2410_LCDCON5);

	/* set lcd address pointers */
	s3c2410fb_set_lcdaddr(fbi);

	writel(fbi->regs.lcdcon1, S3C2410_LCDCON1);
	
	return 0;
}


/*
 *      s3c2410fb_set_par - Optional function. Alters the hardware state.
 *      @info: frame buffer structure that represents a single frame buffer
 *
 */
static int s3c2410fb_set_par(struct fb_info *info)
{
	struct s3c2410fb_info *fbi = (struct s3c2410fb_info *)info;
	struct fb_var_screeninfo *var = &info->var;

	if (var->bits_per_pixel == 16)
		fbi->fb.fix.visual = FB_VISUAL_TRUECOLOR;
	else
		fbi->fb.fix.visual = FB_VISUAL_PSEUDOCOLOR;

	fbi->fb.fix.line_length     = (var->width*var->bits_per_pixel)/8;


	/* activate this new configuration */

	s3c2410fb_activate_var(fbi, var);
	return 0;
}

static void schedule_palette_update(struct s3c2410fb_info *fbi,
				    unsigned int regno, unsigned int val)
{
	unsigned long flags;
	unsigned long irqen;

	local_irq_save(flags);

	fbi->palette_buffer[regno] = val;

	if (!fbi->palette_ready) {
		fbi->palette_ready = 1;

		/* enable IRQ */
		irqen = readl(S3C2410_LCDINTMSK);
		irqen &= ~S3C2410_LCDINT_FRSYNC;
		writel(irqen, S3C2410_LCDINTMSK);
	}

	local_irq_restore(flags);
}

/* from pxafb.c */
static inline unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int s3c2410fb_setcolreg(unsigned regno,
			       unsigned red, unsigned green, unsigned blue,
			       unsigned transp, struct fb_info *info)
{
	struct s3c2410fb_info *fbi = (struct s3c2410fb_info *)info;
	unsigned int val;

	switch (fbi->fb.fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/* true-colour, use pseuo-palette */

		if (regno < 16) {
			u32 *pal = fbi->fb.pseudo_palette;

			val  = chan_to_field(red,   &fbi->fb.var.red);
			val |= chan_to_field(green, &fbi->fb.var.green);
			val |= chan_to_field(blue,  &fbi->fb.var.blue);

			pal[regno] = val;
		}

	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		if (regno < 256) {
			/* currently assume RGB 5-6-5 mode */

			val  = ((red   >>  0) & 0xf800);
			val |= ((green >>  5) & 0x07e0);
			val |= ((blue  >> 11) & 0x001f);
			
			writel(val, S3C2410_TFTPAL(regno));
			schedule_palette_update(fbi, regno, val);
		}

		break;

	default:
		return 1;   /* unknown type */
	}

	return 0;
}


/**
 *	s3c2410fb_pan_display
 *	@var: frame buffer variable screen structure
 *	@info: frame buffer structure that represents a single frame buffer
 *
 *	Pan (or wrap, depending on the `vmode' field) the display using the
 *	`xoffset' and `yoffset' fields of the `var' structure.
 *	If the values don't fit, return -EINVAL.
 *
 *	Returns negative errno on error, or zero on success.
 */
static int s3c2410fb_pan_display(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	gprintk("pan_display(var=%p, info=%p)\n", var, info);

	gprintk("pan_display: xoffset=%d\n", var->xoffset);
	gprintk("pan_display: yoffset=%d\n", var->yoffset);

	return 0;
}

/**
 *      s3c2410fb_blank
 *	@blank_mode: the blank mode we want.
 *	@info: frame buffer structure that represents a single frame buffer
 *
 *	Blank the screen if blank_mode != 0, else unblank. Return 0 if
 *	blanking succeeded, != 0 if un-/blanking failed due to e.g. a
 *	video mode which doesn't support it. Implements VESA suspend
 *	and powerdown modes on hardware that supports disabling hsync/vsync:
 *	blank_mode == 2: suspend vsync
 *	blank_mode == 3: suspend hsync
 *	blank_mode == 4: powerdown
 *
 *	Returns negative errno on error, or zero on success.
 *
 */
static int s3c2410fb_blank(int blank_mode, struct fb_info *info)
{
	gprintk("blank(mode=%d, info=%p)\n", blank_mode, info);

	if (mach_info == NULL)
		return -EINVAL;

	if (blank_mode == FB_BLANK_UNBLANK)
		writel(0x0, S3C2410_TPAL);
	else {
		gprintk("setting TPAL to output 0x000000\n");
		writel(S3C2410_TPAL_EN, S3C2410_TPAL);
	}

	return 0;
}

static int s3c2410fb_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", debug ? "on" : "off");
}
static int s3c2410fb_debug_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t len)
{
	if (mach_info == NULL)
		return -EINVAL;

	if (len < 1)
		return -EINVAL;

	if (strnicmp(buf, "on", 2) == 0 ||
	    strnicmp(buf, "1", 1) == 0) {
		debug = 1;
		printk(KERN_DEBUG "s3c2410fb: Debug On");
	} else if (strnicmp(buf, "off", 3) == 0 ||
		   strnicmp(buf, "0", 1) == 0) {
		debug = 0;
		printk(KERN_DEBUG "s3c2410fb: Debug Off");
	} else {
		return -EINVAL;
	}

	return len;
}

static DEVICE_ATTR(debug, 0666,
		   s3c2410fb_debug_show,
		   s3c2410fb_debug_store);

static struct fb_ops s3c2410fb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= s3c2410fb_check_var,
	.fb_set_par	= s3c2410fb_set_par,
	.fb_blank	= s3c2410fb_blank,
	.fb_pan_display	= s3c2410fb_pan_display,
	.fb_setcolreg	= s3c2410fb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_cursor	= soft_cursor,
};


/* Fake monspecs to fill in fbinfo structure */
/* Don't know if the values are important    */
static struct fb_monspecs monspecs __initdata = {
	.hfmin	= 30000,
	.hfmax	= 70000,
	.vfmin	= 50,
	.vfmax	= 65,
};

/*
 * s3c2410fb_map_video_memory():
 *	Allocates the DRAM memory for the frame buffer.  This buffer is
 *	remapped into a non-cached, non-buffered, memory region to
 *	allow palette and pixel writes to occur without flushing the
 *	cache.  Once this area is remapped, all virtual memory
 *	access to the video memory should occur at the new region.
 */
static int __init s3c2410fb_map_video_memory(struct s3c2410fb_info *fbi)
{
	gprintk("map_video_memory(fbi=%p)\n", fbi);

	fbi->map_size = PAGE_ALIGN(fbi->fb.fix.smem_len + PAGE_SIZE);
	fbi->map_cpu  = dma_alloc_writecombine(fbi->dev, fbi->map_size,
					       &fbi->map_dma, GFP_KERNEL);

	fbi->map_size = fbi->fb.fix.smem_len;

	if (fbi->map_cpu) {
		/* prevent initial garbage on screen */
		gprintk("map_video_memory: clear %p:%08x\n",
			fbi->map_cpu, fbi->map_size);
		memset(fbi->map_cpu, 0xf0, fbi->map_size);

		fbi->screen_dma		= fbi->map_dma;
		fbi->fb.screen_base	= fbi->map_cpu;
		fbi->fb.fix.smem_start  = fbi->screen_dma;

		gprintk("map_video_memory: dma=%08x cpu=%p size=%08x\n",
			fbi->map_dma, fbi->map_cpu, fbi->fb.fix.smem_len);
	}

	return fbi->map_cpu ? 0 : -ENOMEM;
}

static inline void modify_gpio(void __iomem *reg,
			       unsigned long set, unsigned long mask)
{
	unsigned long tmp;

	tmp = readl(reg) & ~mask;
	writel(tmp | set, reg);
}


/*
 * s3c2410fb_init_registers - Initialise all LCD-related registers
 */

int s3c2410fb_init_registers(struct s3c2410fb_info *fbi)
{
	unsigned long flags;

	/* Initialise LCD with values from haret */

	local_irq_save(flags);

	/* modify the gpio(s) with interrupts set (bjd) */

	modify_gpio(S3C2410_GPCUP,  mach_info->gpcup,  mach_info->gpcup_mask);
	modify_gpio(S3C2410_GPCCON, mach_info->gpccon, mach_info->gpccon_mask);
	modify_gpio(S3C2410_GPDUP,  mach_info->gpdup,  mach_info->gpdup_mask);
	modify_gpio(S3C2410_GPDCON, mach_info->gpdcon, mach_info->gpdcon_mask);

	local_irq_restore(flags);

	writel(fbi->regs.lcdcon1, S3C2410_LCDCON1);
	writel(fbi->regs.lcdcon2, S3C2410_LCDCON2);
	writel(fbi->regs.lcdcon3, S3C2410_LCDCON3);
	writel(fbi->regs.lcdcon4, S3C2410_LCDCON4);
	writel(fbi->regs.lcdcon5, S3C2410_LCDCON5);
  
 	s3c2410fb_set_lcdaddr(fbi); //ghcstop: LCD frame buffer address setting

	gprintk("LPCSEL    = 0x%08lx\n", mach_info->lpcsel);
	writel(mach_info->lpcsel, S3C2410_LPCSEL);

	gprintk("replacing TPAL %08x\n", readl(S3C2410_TPAL));

	/* ensure temporary palette disabled */
	writel(0x00, S3C2410_TPAL);
	
	
	/* ghcstop modified */
	s3c2410_gpio_cfgpin(S3C2410_GPC5, S3C2410_GPC5_OUTP); // lcd display enable/disable
	s3c2410_gpio_cfgpin(S3C2410_GPB1, S3C2410_GPB1_OUTP); // back light control
	s3c2410_gpio_cfgpin(S3C2410_GPH6, S3C2410_GPH6_OUTP); 
	
	s3c2410_gpio_pullup(S3C2410_GPC5, 0); 
	s3c2410_gpio_pullup(S3C2410_GPB1, 0);
	s3c2410_gpio_pullup(S3C2410_GPH6, 0);

	s3c2410_gpio_setpin(S3C2410_GPC5, 1);
	s3c2410_gpio_setpin(S3C2410_GPH6, 1); 
	s3c2410_gpio_setpin(S3C2410_GPB1, 1);

	
	/* probably not required */
	msleep(10);		

	/* Enable video by setting the ENVID bit to 1 */
	fbi->regs.lcdcon1 |= S3C2410_LCDCON1_ENVID;
	writel(fbi->regs.lcdcon1, S3C2410_LCDCON1);
	return 0;
}

static void s3c2410fb_write_palette(struct s3c2410fb_info *fbi)
{
	unsigned int i;
	unsigned long ent;

	fbi->palette_ready = 0;
	
	for (i = 0; i < 256; i++) {		
		if ((ent = fbi->palette_buffer[i]) == PALETTE_BUFF_CLEAR)
			continue;

		writel(ent, S3C2410_TFTPAL(i));
		
		/* it seems the only way to know exactly
		 * if the palette wrote ok, is to check
		 * to see if the value verifies ok
		 */
		
		if (readw(S3C2410_TFTPAL(i)) == ent)
			fbi->palette_buffer[i] = PALETTE_BUFF_CLEAR;
		else
			fbi->palette_ready = 1;   /* retry */
	}
}

static char driver_name[]="s3c2410fb";
 
int __init s3c2410fb_probe(struct device *dev)
{
	struct s3c2410fb_hw *mregs;
	int ret;
	int i;

	mach_info = dev->platform_data;
	if (mach_info == NULL) {
		dev_err(dev,"no platform data for lcd, cannot attach\n");
		return -EINVAL;
	}

	mregs = &mach_info->regs;

	strcpy(info.fb.fix.id, driver_name);

	memcpy(&info.regs, &mach_info->regs, sizeof(info.regs));

	info.mach_info		    = dev->platform_data;

	info.fb.fix.type	    = FB_TYPE_PACKED_PIXELS;
	info.fb.fix.type_aux	    = 0;
	info.fb.fix.xpanstep	    = 0;
	info.fb.fix.ypanstep	    = 0;
	info.fb.fix.ywrapstep	    = 0;
	info.fb.fix.accel	    = FB_ACCEL_NONE;

	info.fb.var.nonstd	    = 0;
	info.fb.var.activate	    = FB_ACTIVATE_NOW;
#if 1	
	info.fb.var.height	    = mach_info->height;
	info.fb.var.width	    = mach_info->width;
#else	
	info.fb.var.height	    = -1;
	info.fb.var.width	    = -1;
#endif	
	info.fb.var.accel_flags     = 0;
	info.fb.var.vmode	    = FB_VMODE_NONINTERLACED;

	info.fb.fbops		    = &s3c2410fb_ops;
	info.fb.flags		    = FBINFO_FLAG_DEFAULT;
	info.fb.monspecs	    = monspecs;
#if 1	
	info.fb.pseudo_palette      = &info.pseudo_pal;
#else
	addr = &info;
        addr = addr + sizeof(struct s3c2410fb_info);
	info.fb.pseudo_palette      = addr;
#endif	

	info.fb.var.xres	    = mach_info->xres.defval;
	info.fb.var.xres_virtual    = mach_info->xres.defval;
	info.fb.var.yres	    = mach_info->yres.defval;
	info.fb.var.yres_virtual    = mach_info->yres.defval;
	info.fb.var.bits_per_pixel  = mach_info->bpp.defval;
	info.fb.var.pixclock        = mach_info->pixclock; 
	
	info.fb.var.upper_margin    = S3C2410_LCDCON2_GET_VBPD(mregs->lcdcon2) +1;
	info.fb.var.lower_margin    = S3C2410_LCDCON2_GET_VFPD(mregs->lcdcon2) +1;
	info.fb.var.vsync_len	    = S3C2410_LCDCON2_GET_VSPW(mregs->lcdcon2) + 1;

	info.fb.var.left_margin	    = S3C2410_LCDCON3_GET_HFPD(mregs->lcdcon3) + 1;
	info.fb.var.right_margin    = S3C2410_LCDCON3_GET_HBPD(mregs->lcdcon3) + 1;
	info.fb.var.hsync_len	    = S3C2410_LCDCON4_GET_HSPW(mregs->lcdcon4) + 1;

	info.fb.var.red.offset      = 11;
	info.fb.var.green.offset    = 5;
	info.fb.var.blue.offset     = 0;
	info.fb.var.transp.offset   = 0;
	info.fb.var.red.length      = 5;
	info.fb.var.green.length    = 6;
	info.fb.var.blue.length     = 5;
	info.fb.var.transp.length   = 0;
	info.fb.fix.smem_len        =	mach_info->xres.max *
					mach_info->yres.max *
					mach_info->bpp.max / 8;

	for (i = 0; i < 256; i++)
		info.palette_buffer[i] = PALETTE_BUFF_CLEAR;

	
	info.clk = clk_get(NULL, "lcd");
	if (!info.clk || IS_ERR(info.clk)) {
		printk(KERN_ERR "failed to get lcd clock source\n");
		return -ENOENT;
	}

	clk_use(info.clk);
	clk_enable(info.clk);
	gprintk("got and enabled clock\n");

	msleep(1);

	/* Initialize video memory */
	ret = s3c2410fb_map_video_memory(&info);
	if (ret) {
		printk( KERN_ERR "Failed to allocate video RAM: %d\n", ret);
		ret = -ENOMEM;
		goto failed;
	}
	gprintk("got video memory\n");

	ret = s3c2410fb_init_registers(&info); // ¿Ö ¶Ç ÀÌÁö¶öÀÌÁö?

	ret = s3c2410fb_check_var(&info.fb.var, &info.fb);

	ret = register_framebuffer(&info.fb);
	if (ret < 0) {
		printk(KERN_ERR "Failed to register framebuffer device: %d\n", ret);
		goto failed;
	}

	/* create device files */
	device_create_file(dev, &dev_attr_debug);

	printk(KERN_INFO "S3C24X0 fb%d: %s frame buffer device initialize done\n",
		info.fb.node, info.fb.fix.id);

	return 0;
failed:
	return ret;
}

/* s3c2410fb_stop_lcd
 *
 * shutdown the lcd controller
*/

static void s3c2410fb_stop_lcd(void)
{
	unsigned long flags;
	unsigned long tmp;

	local_irq_save(flags);

	tmp = readl(S3C2410_LCDCON1);
	writel(tmp & ~S3C2410_LCDCON1_ENVID, S3C2410_LCDCON1);

	local_irq_restore(flags);
}

/*
 *  Cleanup
 */
static void __exit s3c2410fb_cleanup(void)
{
	s3c2410fb_stop_lcd();
	msleep(1);

 	if (info.clk) {
 		clk_disable(info.clk);
 		clk_unuse(info.clk);
 		clk_put(info.clk);
 		info.clk = NULL;
	}

	unregister_framebuffer(&info.fb);
}

#ifdef CONFIG_PM

/* suspend and resume support for the lcd controller */

static int s3c2410fb_suspend(struct device *dev, pm_message_t state, u32 level)
{
	if (level == SUSPEND_DISABLE || level == SUSPEND_POWER_DOWN) {
		s3c2410fb_stop_lcd();

		/* sleep before disabling the clock, we need to ensure
		 * the LCD DMA engine is not going to get back on the bus
		 * before the clock goes off again (bjd) */

		msleep(1);
		clk_disable(info.clk);
	}

	return 0;
}

static int s3c2410fb_resume(struct device *dev, u32 level)
{
	if (level == RESUME_ENABLE) {
		clk_enable(info.clk);
		msleep(1);

		s3c2410fb_init_registers(&info);

	}

	return 0;
}

#else
#define s3c2410fb_suspend NULL
#define s3c2410fb_resume  NULL
#endif

static struct device_driver s3c2410fb_driver = {
	.name		= "s3c2410-lcd",
	.bus		= &platform_bus_type,
	.probe		= s3c2410fb_probe,
	.suspend	= s3c2410fb_suspend,
	.resume		= s3c2410fb_resume,
};

int __devinit s3c2410fb_init(void)
{
	return driver_register(&s3c2410fb_driver);
}

module_init(s3c2410fb_init);
module_exit(s3c2410fb_cleanup);

MODULE_AUTHOR("Arnaud Patard <arnaud.patard@rtp-net.org>, Ben Dooks <ben-linux@fluff.org>, godori<www.aesop-embedded.org>");
MODULE_DESCRIPTION("Framebuffer driver for the s3c2410");
MODULE_LICENSE("GPL");
