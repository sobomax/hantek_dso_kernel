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
 *     10-Mar-2005 LCVR Changed S3C2410_VA to S3C24XX_VA
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

#include "clock.h"
#include "devs.h"
#include "cpu.h"

static struct map_desc h1940_iodesc[] __initdata = {
	/* nothing here yet */
};

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

static struct s3c2410_uartcfg h1940_uartcfgs[] = {
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
		.ucon	     = 0x245,
		.ulcon	     = 0x03,
		.ufcon	     = 0x00,
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

static struct s3c2410_udc_mach_info h1940_udc_cfg __initdata = {
		.udc_command = pullup,
};


static struct s3c2410_ts_mach_info h1940_ts_cfg __initdata = {
		.delay = 10000,
		.presc = 49,
		.oversampling_shift = 2,
};

/**
 * Set lcd on or off
 **/

static void h1940_backlight_power(int on)
{
	s3c2410_gpio_setpin(S3C2410_GPB0, 0);
	s3c2410_gpio_pullup(S3C2410_GPB0, 0);

	s3c2410_gpio_cfgpin(S3C2410_GPB0,
			    (on) ? S3C2410_GPB0_TOUT0 : S3C2410_GPB0_OUTP);
}

static void h1940_lcd_power(int on)
{
	s3c2410_gpio_setpin(S3C2410_GPC0, on);
}

static struct s3c2410fb_mach_info h1940_lcdcfg __initdata = {
	.fixed_syncs=		1,
	.regs={ 
		.lcdcon1=	S3C2410_LCDCON1_TFT16BPP | \
				S3C2410_LCDCON1_TFT | \
				S3C2410_LCDCON1_CLKVAL(0x0C),

		.lcdcon2=	S3C2410_LCDCON2_VBPD(7) | \
				S3C2410_LCDCON2_LINEVAL(319) | \
				S3C2410_LCDCON2_VFPD(6) | \
				S3C2410_LCDCON2_VSPW(0),

		.lcdcon3=	S3C2410_LCDCON3_HBPD(19) | \
				S3C2410_LCDCON3_HOZVAL(239) | \
				S3C2410_LCDCON3_HFPD(7),

		.lcdcon4=	S3C2410_LCDCON4_MVAL(0) | \
				S3C2410_LCDCON4_HSPW(3),

		.lcdcon5=	S3C2410_LCDCON5_FRM565 | \
				S3C2410_LCDCON5_INVVLINE | \
				S3C2410_LCDCON5_HWSWP,
	},
	.lpcsel=	0x02,
	.gpccon=	0xaa940659,
	.gpccon_mask=	0xffffffff,
	.gpcup=		0x0000ffff,
	.gpcup_mask=	0xffffffff,
	.gpdcon=	0xaa84aaa0,
	.gpdcon_mask=	0xffffffff,
	.gpdup=		0x0000faff,
	.gpdup_mask=	0xffffffff,

	.width=		240,
	.height=	320,
	.xres=		{240,240,240},
	.yres=		{320,320,320},
	.bpp=		{16,16,16},
};

static void h1940_set_brightness(int tcmpb0)
{
	unsigned long tcfg0;
	unsigned long tcfg1;
	unsigned long tcon;

	/* configure power on/off */
	h1940_backlight_power(tcmpb0 ? 1 : 0);


	tcfg0=readl(S3C2410_TCFG0);
	tcfg1=readl(S3C2410_TCFG1);

	tcfg0 &= ~S3C2410_TCFG_PRESCALER0_MASK;
	tcfg0 |= 0x18;

	tcfg1 &= ~S3C2410_TCFG1_MUX0_MASK;
	tcfg1 |= S3C2410_TCFG1_MUX0_DIV2;

	writel(tcfg0, S3C2410_TCFG0);
	writel(tcfg1, S3C2410_TCFG1);
	writel(0x31, S3C2410_TCNTB(0));

	tcon = readl(S3C2410_TCON);
	tcon &= ~0x0F;
	tcon |= S3C2410_TCON_T0RELOAD;
	tcon |= S3C2410_TCON_T0MANUALUPD;

	writel(tcon, S3C2410_TCON);
	writel(0x31, S3C2410_TCNTB(0));
	writel(tcmpb0, S3C2410_TCMPB(0));

	/* start the timer running */
	tcon |= S3C2410_TCON_T0START;
	tcon &= ~S3C2410_TCON_T0MANUALUPD;
	writel(tcon, S3C2410_TCON);
}

static struct s3c2410_bl_mach_info h1940_blcfg __initdata = {

	.backlight_max          = 0x2c,
	.backlight_default      = 0x16,
	.backlight_power	= h1940_backlight_power,
	.set_brightness		= h1940_set_brightness,
	.backlight_power	= h1940_backlight_power,
	.lcd_power		= h1940_lcd_power
};

static struct platform_device *h1940_devices[] __initdata = {
	&s3c_device_usb,
	&s3c_device_lcd,
	&s3c_device_bl,
	&s3c_device_wdt,
	&s3c_device_i2c,
	&s3c_device_iis,
	&s3c_device_sdi,
	&s3c_device_usbgadget,
	&s3c_device_ts,
	&s3c_device_buttons,
	&s3c_device_nand,
};

static struct s3c24xx_board h1940_board __initdata = {
	.devices       = h1940_devices,
	.devices_count = ARRAY_SIZE(h1940_devices)
};

void __init h1940_map_io(void)
{
	s3c24xx_init_io(h1940_iodesc, ARRAY_SIZE(h1940_iodesc));
	s3c24xx_init_clocks(0);
	s3c24xx_init_uarts(h1940_uartcfgs, ARRAY_SIZE(h1940_uartcfgs));
	s3c24xx_set_board(&h1940_board);
}

void __init h1940_init_irq(void)
{
	s3c24xx_init_irq();

}

void __init h1940_init(void)
{
	set_s3c2410ts_info(&h1940_ts_cfg);
 	set_s3c2410udc_info(&h1940_udc_cfg);
 	/* Set pad to usb device and usbsuspend to 'normal' */
 	__raw_writel(__raw_readl(S3C2410_MISCCR)&~0x3008,S3C2410_MISCCR);
	set_s3c2410fb_info(&h1940_lcdcfg);
	set_s3c2410bl_info(&h1940_blcfg);
}

MACHINE_START(H1940, "IPAQ-H1940")
	/* Maintainer: Ben Dooks <ben@fluff.org> */
	.phys_ram	= S3C2410_SDRAM_PA,
	.phys_io	= S3C2410_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C2410_SDRAM_PA + 0x100,
	.map_io		= h1940_map_io,
	.init_irq	= h1940_init_irq,
	.init_machine	= h1940_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
