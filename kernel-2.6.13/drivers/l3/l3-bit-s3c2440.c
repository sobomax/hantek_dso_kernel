/*
 * drivers/l3/l3-bit-elfin.c
 * 
 * $Id: l3-bit-elfin.c,v 1.2 2004/05/12 06:28:52 laputa Exp $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * 2004-04-28 : Kwanghyun la <nala.la@samsung.com>
 *   - modified for sharing module device driver of samsung  arch
 *
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/l3/algo-bit.h>

#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>

#include <asm/arch/regs-gpio.h>

//#define GDEBUG
#ifdef GDEBUG
#  define dprintk(x...) printk(x)
#else
#  define dprintk(x...)
#endif



#define NAME "l3-bit-24x0-gpio"

struct bit_data {
	unsigned int	sda;
	unsigned int	scl;
	unsigned int	l3_mode;
};

static int getsda(void *data)
{
	struct bit_data *bits = data;

	return (int)s3c2410_gpio_getpin(bits->sda);
}

static DECLARE_MUTEX(l3_lock);
#define LOCK		&l3_lock

/*
 * iPAQs need the clock line driven hard high and low.
 */
static void l3_setscl(void *data, int state)
{
	struct bit_data *bits = data;
	unsigned long flags;

	local_irq_save(flags);
	if (state)
	{
		s3c2410_gpio_setpin(bits->scl, 1);
	}
	else
	{
		s3c2410_gpio_setpin(bits->scl, 0);
	}
		
	s3c2410_gpio_cfgpin( (bits->scl), S3C2410_GPB4_OUTP);
	s3c2410_gpio_pullup( (bits->scl), 1); 
	
	
	local_irq_restore(flags);
}

static void l3_setsda(void *data, int state)
{
	struct bit_data *bits = data;

	if (state)
	{
		s3c2410_gpio_setpin(bits->sda, 1);
	}
	else
	{
		s3c2410_gpio_setpin(bits->sda, 0);
	}
}

static void l3_setdir(void *data, int in)
{
	struct bit_data *bits = data;
	unsigned long flags;

	local_irq_save(flags);
	if (in)
	{
		s3c2410_gpio_cfgpin( (bits->sda), S3C2410_GPB3_INP);
    	s3c2410_gpio_pullup( (bits->sda), 1); 
	}
	else
	{
		s3c2410_gpio_cfgpin( (bits->sda), S3C2410_GPB3_OUTP);
    	s3c2410_gpio_pullup( (bits->sda), 1); 
	}
	local_irq_restore(flags);
}

static void l3_setmode(void *data, int state)
{
	struct bit_data *bits = data;

	if (state)
	{
		s3c2410_gpio_setpin(bits->l3_mode, 1);
	}
	else
	{
		s3c2410_gpio_setpin(bits->l3_mode, 0);
	}
}

static struct l3_algo_bit_data l3_bit_data = {
	data:		NULL,
	setdat:		l3_setsda,
	setclk:		l3_setscl,
	setmode:	l3_setmode,
	setdir:		l3_setdir,
	getdat:		getsda,
	data_hold:	1,
	data_setup:	1,
	clock_high:	1,
	mode_hold:	1,
	mode_setup:	1,
};

static struct l3_adapter l3_adapter = {
	owner:		THIS_MODULE,
	name:		NAME,
	algo_data:	&l3_bit_data,
	lock:		LOCK,
};

static int inline l3_start(struct bit_data *bits)
{
	l3_bit_data.data = bits;
	return l3_bit_add_bus(&l3_adapter);
}

static void inline l3_end(void)
{
	l3_bit_del_bus(&l3_adapter);
}

static struct bit_data bit_data;

static int __init bus_init(void)
{
	struct bit_data *bit = &bit_data;
	unsigned long flags;
	int ret;

#if defined( CONFIG_ARCH_QQ2440) || defined (CONFIG_BOARD_S3C2440_SMDK)

dprintk("l3_2440 0\n");

	bit->sda     = S3C2410_GPB3;
	bit->scl     = S3C2410_GPB4;
	bit->l3_mode = S3C2410_GPB2;
#endif

	if (!bit->sda)
		return -ENODEV;

	/*
	 * Default level for L3 mode is low.
	 */

	local_irq_save(flags);

	/* L3 gpio interface set */
dprintk("l3_2440 1\n");	
    s3c2410_gpio_setpin(bit->l3_mode, 1); 
	s3c2410_gpio_setpin(bit->scl, 1);

	s3c2410_gpio_cfgpin( (bit->scl), S3C2410_GPB4_OUTP);
	s3c2410_gpio_pullup( (bit->scl), 1); 
	s3c2410_gpio_cfgpin( (bit->sda), S3C2410_GPB3_OUTP);
	s3c2410_gpio_pullup( (bit->sda), 1); 
	s3c2410_gpio_cfgpin( (bit->l3_mode), S3C2410_GPB2_OUTP);
	s3c2410_gpio_pullup( (bit->l3_mode), 1); 
     

#if defined( CONFIG_ARCH_QQ2440) || defined (CONFIG_BOARD_S3C2440_SMDK)
	/* IIS gpio interface set */
	
dprintk("l3_2440 2\n");		
        /* GPE 0: I2SLRCK */
        s3c2410_gpio_cfgpin( S3C2410_GPE0, S3C2410_GPE0_I2SLRCK);
        s3c2410_gpio_pullup( S3C2410_GPE0, 0); 
        /* GPE 1: I2SSCLK */
        s3c2410_gpio_cfgpin( S3C2410_GPE1, S3C2410_GPE1_I2SSCLK);
        s3c2410_gpio_pullup( S3C2410_GPE1, 0); 
        /* GPE 2: CDCLK */
        s3c2410_gpio_cfgpin( S3C2410_GPE2, S3C2410_GPE2_CDCLK);
        s3c2410_gpio_pullup( S3C2410_GPE2, 0); 
        /* GPE 3: I2SSDI */
        s3c2410_gpio_cfgpin( S3C2410_GPE3, S3C2410_GPE3_I2SSDI);
        s3c2410_gpio_pullup( S3C2410_GPE3, 0); 
        /* GPE 4: I2SSDO */
        s3c2410_gpio_cfgpin( S3C2410_GPE4, S3C2410_GPE4_I2SSDO);
        s3c2410_gpio_pullup( S3C2410_GPE4, 0); 


#endif

	local_irq_restore(flags);

	ret = l3_start(bit);
	if (ret)
		l3_end();

	printk("GPIO L3 bus interface for S3C2440, installed\n");

	return ret;
}

static void __exit bus_exit(void)
{
	l3_end();
	printk("GPIO L3 bus interface for S3C2440, uninstalled\n");
}

module_init(bus_init);
module_exit(bus_exit);
