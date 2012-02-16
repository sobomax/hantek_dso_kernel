/* linux/arch/arm/mach-s3c2410/mach-h1940.c
 *
 * Copyright (c) 2003-2005 Simtec Electronics
 *   Ben Dooks <ben@simtec.co.uk>
 *
 * http://www.handhelds.org/projects/h1940.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Modifications:
 *     16-May-2003 BJD  Created initial version
 *     16-Aug-2003 BJD  Fixed header files and copyright, added URL
 *     05-Sep-2003 BJD  Moved to v2.6 kernel
 *     06-Jan-2003 BJD  Updates for <arch/map.h>
 *     18-Jan-2003 BJD  Added serial port configuration
 *     17-Feb-2003 BJD  Copied to mach-ipaq.c
 *     21-Aug-2004 BJD  Added struct s3c2410_board
 *     04-Sep-2004 BJD  Changed uart init, renamed ipaq_ -> h1940_
 *     18-Oct-2004 BJD  Updated new board structure name
 *     04-Nov-2004 BJD  Change for new serial clock
 *     04-Jan-2005 BJD  Updated uart init call
 *     10-Jan-2005 BJD  Removed include of s3c2410.h
 *     14-Jan-2005 BJD  Added clock init
 *     03-May-2005 DaeHo Jung<aplus43@hanmail.net> has modified for aesop2440 board -jdh-
 
 2005.06 godori(ghcstop) from www.aesop-embedded.org
         fix for s3c2440a sound driver

 2005.12 godori(ghcstop) from www.aesop-embedded.org
         RTC, usb gadget(not working), backlight control, LCD on/off, nand+mtd+yaffs
         
 2005.12.26 godori(ghcstop) from www.aesop-embedded.org
         usb gadget(working), 7" Hitachi LCD(800x480) add, framebuffer configuration parameter scheme change
         
*/



#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/device.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/hardware.h>
#include <asm/hardware/iomd.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

//#include <asm/debug-ll.h>
#include <asm/arch/regs-serial.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-lcd.h>
#include <asm/arch/regs-timer.h>

#include <asm/arch/fb.h>
#include <asm/arch/udc.h>
#include <asm/arch/ts.h>
#include <asm/arch/lcd.h>

#include <linux/serial_core.h>

#include <asm/arch/map.h> // ghcstop add
#include <asm/arch/aesop-map.h>

// ghcstop for nand
#include <asm/arch/nand.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>


#include "clock.h"
#include "devs.h"
#include "cpu.h"


// ghcstop fix
static struct map_desc aesop2440_iodesc[] __initdata = {
  { AESOP_VA_CS8900A, AESOP_PA_CS8900A, AESOP_SZ_CS8900A, MT_DEVICE }, 
  { (unsigned long)S3C24XX_VA_IIS, S3C2410_PA_IIS, S3C24XX_SZ_IIS, MT_DEVICE },   
};

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

static struct s3c2410_uartcfg aesop2440_uartcfgs[] = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	/* IR port */
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.uart_flags  = UPF_CONS_FLOW,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x43,
		.ufcon	     = 0x51,
	}
};


static void pullup(unsigned char cmd)
{
	printk(KERN_DEBUG "udc: pullup(%d)\n",cmd);
	switch (cmd)
	{
		case S3C2410_UDC_P_ENABLE :
			break;
		case S3C2410_UDC_P_DISABLE :
			break;
		case S3C2410_UDC_P_RESET :
			break;
		default: break;
	}
}

static struct s3c2410_udc_mach_info aesop2440_udc_cfg __initdata = {
		.udc_command = pullup,
};


static struct s3c2410_ts_mach_info aesop2440_ts_cfg __initdata = {
		.delay = 10000,
		.presc = 49,
		.oversampling_shift = 2,
};


 
#if defined( CONFIG_FB_S3C2410_640X480 )

// ghcstop: aesop2440 use 640x480 6.4" LCD panel
static struct s3c2410fb_mach_info aesop2440_lcdcfg __initdata = {
	.fixed_syncs = 1,
	
	.width       =	640,
	.height      =	480,

	.xres=		{640,640,640},
	.yres=		{480,480,480},
	.bpp=		{16,16,16},
	
	
	.regs={ 
		.lcdcon1=	S3C2410_LCDCON1_TFT16BPP | \
				S3C2410_LCDCON1_TFT | \
				S3C2410_LCDCON1_CLKVAL(4),
		
		.lcdcon2=	S3C2410_LCDCON2_VBPD(1) | \
				S3C2410_LCDCON2_LINEVAL(480-1) | \
				S3C2410_LCDCON2_VFPD(2) | \
				S3C2410_LCDCON2_VSPW(1),
				  
		.lcdcon3=	S3C2410_LCDCON3_HBPD(6) | \
				S3C2410_LCDCON3_HOZVAL(640-1) | \
				S3C2410_LCDCON3_HFPD(2),
				  
		.lcdcon4= 	S3C2410_LCDCON4_HSPW(4),
		
		.lcdcon5=	S3C2410_LCDCON5_FRM565 | \
				S3C2410_LCDCON5_INVVLINE | \
				S3C2410_LCDCON5_INVVFRAME | \
				S3C2410_LCDCON5_HWSWP,
	},
	.lpcsel=	0xE0,
	
	.gpccon     =	0xAAAA56A9,
	.gpccon_mask=	0xffffffff,
	.gpcup      =	0x0000ffff,
	.gpcup_mask =	0xffffffff,
	.gpdcon     =	0xaaaaaaaa,
	.gpdcon_mask=	0xffffffff,
	.gpdup      =	0x0000ffff,
	.gpdup_mask =	0xffffffff,

};



#elif defined( CONFIG_FB_S3C2410_480X272 )

#define H_FP	1		/* front porch */
#define H_SW	40		/* Hsync width  */
#define H_BP	1		/* Back porch */

#define V_FP	1		/* front porch */
#define V_SW	9		/* Vsync width */
#define V_BP	1		/* Back porch */

#define H_RESOLUTION	480	/* x resolition */
#define V_RESOLUTION	272	/* y resolution  */
#define VFRAME_FREQ     60	/* frame rate freq. */

#define LCD_PIXEL_CLOCK (VFRAME_FREQ *(H_FP+H_SW+H_BP+H_RESOLUTION) * (V_FP+V_SW+V_BP+V_RESOLUTION))
#define PIXEL_CLOCK     (VFRAME_FREQ * LCD_PIXEL_CLOCK)	/*  vclk = frame * pixel_count */
#define PIXEL_BPP       16	/*  RGB 5-6-5 format */


// ghcstop: aesop2440 use 480x272 4" LCD panel
static struct s3c2410fb_mach_info aesop2440_lcdcfg __initdata = {
	.fixed_syncs = 1,
	.pixclock    = LCD_PIXEL_CLOCK,   
	
	.width       =	H_RESOLUTION,
	.height      =	V_RESOLUTION,

	.xres=		{H_RESOLUTION, H_RESOLUTION, H_RESOLUTION},
	.yres=		{V_RESOLUTION, V_RESOLUTION, V_RESOLUTION},
	.bpp=		{PIXEL_BPP, PIXEL_BPP, PIXEL_BPP},
	
	
	.regs={ 
		.lcdcon1=	S3C2410_LCDCON1_TFT16BPP | \
				S3C2410_LCDCON1_TFT ,
		
		.lcdcon2=	S3C2410_LCDCON2_VBPD(V_BP) | \
				S3C2410_LCDCON2_LINEVAL(V_RESOLUTION-1) | \
				S3C2410_LCDCON2_VFPD(V_FP) | \
				S3C2410_LCDCON2_VSPW(V_SW),
				  
		.lcdcon3=	S3C2410_LCDCON3_HBPD(H_BP) | \
				S3C2410_LCDCON3_HOZVAL(H_RESOLUTION-1) | \
				S3C2410_LCDCON3_HFPD(H_FP),
				  
		.lcdcon4= 	S3C2410_LCDCON4_HSPW(H_SW),
		
		.lcdcon5=	S3C2410_LCDCON5_FRM565 | \
				S3C2410_LCDCON5_INVVLINE | \
				S3C2410_LCDCON5_INVVFRAME | \
				S3C2410_LCDCON5_HWSWP,
	},
	.lpcsel=	0xE0,
	
	.gpccon     =	0xAAAA56A9,
	.gpccon_mask=	0xffffffff,
	.gpcup      =	0x0000ffff,
	.gpcup_mask =	0xffffffff,
	.gpdcon     =	0xaaaaaaaa,
	.gpdcon_mask=	0xffffffff,
	.gpdup      =	0x0000ffff,
	.gpdup_mask =	0xffffffff,

};



#endif

//===================================================================
// backlight & lcd power control: ghcstop add
//===================================================================

//#define GDEBUG // ghcstop add
#ifdef  GDEBUG
#    define gprintk( x... )  printk( x )
#else
#    define gprintk( x... )
#endif


static void aesop2440_backlight_power(int on)
{
	gprintk("%s\n", __FUNCTION__);
	s3c2410_gpio_setpin(S3C2410_GPB1, 0);
	s3c2410_gpio_pullup(S3C2410_GPB1, 0);

	s3c2410_gpio_cfgpin(S3C2410_GPB1,
			    (on) ? S3C2410_GPB1_TOUT1 : S3C2410_GPB1_OUTP);
}

static void aesop2440_lcd_power(int on)
{
	// lcd en/disable pin제어...이것을 끄게되면 그냥 하얗게 나와 버린다. 쩝....이상하다. lcd 메뉴얼 따라서 틀리게 해야겠당.
	gprintk("A %s: on: %d\n", __FUNCTION__, on);
	s3c2410_gpio_setpin(S3C2410_GPC5, on);
}

static void aesop2440_set_brightness(int tcmpb0)
{
	unsigned long tcfg0;
	unsigned long tcfg1;
	unsigned long tcon;

	gprintk("%s: tcmpb0: %d\n", __FUNCTION__, tcmpb0);

	aesop2440_backlight_power(tcmpb0 ? 1 : 0);


	tcfg0=readl(S3C2410_TCFG0);
	tcfg1=readl(S3C2410_TCFG1);

	tcfg0 &= ~S3C2410_TCFG_PRESCALER0_MASK; 
	tcfg0 |= 0x18; 

	tcfg1 &= ~S3C2410_TCFG1_MUX1_MASK; 
	tcfg1 |= S3C2410_TCFG1_MUX1_DIV2;

	writel(tcfg0, S3C2410_TCFG0);
	writel(tcfg1, S3C2410_TCFG1);
	
	writel(0x31, S3C2410_TCNTB(1));

	tcon = readl(S3C2410_TCON);
	tcon &= ~0x00000f00;                   
	tcon |= S3C2410_TCON_T1RELOAD;         
	tcon |= S3C2410_TCON_T1MANUALUPD;      

	writel(tcon, S3C2410_TCON);      
	 
	writel(0x31, S3C2410_TCNTB(1));  
	writel(tcmpb0, S3C2410_TCMPB(1));

	/* start the timer running */
	tcon |= S3C2410_TCON_T1START;      
	tcon &= ~S3C2410_TCON_T1MANUALUPD; 
	writel(tcon, S3C2410_TCON);
}

static struct s3c2410_bl_mach_info aesop2440_blcfg __initdata = {

	.backlight_max          = 0x2c,
	.backlight_default      = 0x16,
	.backlight_power	= aesop2440_backlight_power,
	.set_brightness		= aesop2440_set_brightness,
	.backlight_power	= aesop2440_backlight_power,
	.lcd_power		= aesop2440_lcd_power
};


//===================================================================
// nand flash: ghcstop add
//===================================================================

/* ghcstop: NAND Flash on AESOP2440 board */
struct mtd_partition aesop_default_nand_part[1] = {
	[0] = {
		.name	= "Total nand",
		.size	= 64*SZ_1M,
		.offset	= 0
	},
};

/* the aesop has 1 selectable slots for nand-flash, the three
 * on-board chip areas, as well as the external SmartMedia
 * slot.
 *
 * Note, there is no current hot-plug support for the SmartMedia
 * socket.
*/

static struct s3c2410_nand_set aesop_nand_sets[] = {
	[0] = {
		.name		= "SMC",
		.nr_chips	= 1,
		.nr_map		= NULL,
		.nr_partitions	= ARRAY_SIZE(aesop_default_nand_part),
		.partitions	= aesop_default_nand_part
	},
};


static struct s3c2410_platform_nand aesop_nand_info = {
	.tacls		= 10,
	.twrph0		= 60,
	.twrph1		= 30,
	.nr_sets	= ARRAY_SIZE(aesop_nand_sets),
	.sets		= aesop_nand_sets,
	.select_chip	= NULL,
};



static struct platform_device *aesop2440_devices[] __initdata = {
	&s3c_device_usb,
	&s3c_device_lcd,
	&s3c_device_bl, // ghcstop: backlight
	&s3c_device_wdt,
	&s3c_device_i2c,
	&s3c_device_iis,
	&s3c_device_sdi,
	&s3c_device_usbgadget,
	&s3c_device_ts,
	&s3c_device_nand,
	&s3c_device_sound, // ghcstop: sound
	&s3c_device_buttons, // ghcstop add for button driver
	&s3c_device_rtc, // ghcstop rtc
};

static struct s3c24xx_board aesop2440_board __initdata = {
	.devices       = aesop2440_devices,
	.devices_count = ARRAY_SIZE(aesop2440_devices)
};

void __init aesop2440_map_io(void)
{
	s3c24xx_init_io(aesop2440_iodesc, ARRAY_SIZE(aesop2440_iodesc));
// ghcstop add
#ifdef CONFIG_S3C2440_INCLK12
	s3c24xx_init_clocks(12000000);
#else
	s3c24xx_init_clocks(16934400);
#endif
	s3c24xx_init_uarts(aesop2440_uartcfgs, ARRAY_SIZE(aesop2440_uartcfgs));
	s3c24xx_set_board(&aesop2440_board);
	
	// ghcstop add
	s3c_device_nand.dev.platform_data = &aesop_nand_info;	
}

void __init aesop2440_init_irq(void)
{
	s3c24xx_init_irq();

}

void __init aesop2440_init(void)
{
	set_s3c2410ts_info(&aesop2440_ts_cfg);
	set_s3c2410udc_info(&aesop2440_udc_cfg);
	set_s3c2410fb_info(&aesop2440_lcdcfg); 
	
	/* lcd on/off & backlight brightness control driver */
	set_s3c2410bl_info(&aesop2440_blcfg);
	
	s3c2410_gpio_cfgpin(S3C2410_GPH0, S3C2410_GPH0_OUTP);
	s3c2410_gpio_pullup(S3C2410_GPH0, 0); 
	s3c2410_gpio_setpin(S3C2410_GPH0, 0); 
}
		
MACHINE_START(AESOP2440, "aESOP-2440")
	/* Maintainer: godori(ghcstop@gmail.com) www.aesop-embedded.org */
	.phys_ram	= S3C2410_SDRAM_PA,
	.phys_io	= S3C2410_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C2410_SDRAM_PA + 0x100,
	
	.init_irq	= aesop2440_init_irq,
	.map_io		= aesop2440_map_io,
	.init_machine	= aesop2440_init,
	.timer		= &s3c24xx_timer,	
MACHINE_END
