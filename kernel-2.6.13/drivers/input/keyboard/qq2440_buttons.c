/*
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (c) 2005 Arnaud Patard <arnaud.patard@rtp-net.org>
 * Samsung S3C2410 keyboard support
 *
 * Based on various pxa ipaq drivers.
 * 
 * ChangeLog
 *
 * 2005-07-24: Arnaud Patard <arnaud.patard@rtp-net.org>
 * 	- Added key repeat
 *
 * 2005-07-23: Arnaud Patard <arnaud.patard@rtp-net.org>
 * 	- Renamed all kbd occurences to something like buttons
 * 	- Added debounce (specially usefull for iPAQ power button)
 *
 * 2005-06-21: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - Initial version
 *
 */

#include <linux/config.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <asm/arch/regs-gpio.h>

/* For id.version */
#define S3C2410BUTVERSION	0x0001
#define DRV_NAME		"s3c2410-buttons"

#define REPEAT_DELAY		HZ/10

#ifdef DEBUG
#define dprintk(msg...) printk(KERN_DEBUG "s3c2410_buttons: " msg);
#else
#define dprintk(msg...)
#endif

MODULE_AUTHOR("Arnaud Patard <arnaud.patard@rtp-net.org>");
MODULE_DESCRIPTION("s3c2410 buttons driver");
MODULE_LICENSE("GPL");

struct s3c2410_button {
	int irq;
	int pin;
	int pin_setting;
	int keycode;
	char *name;
	int last_state;
	struct timer_list timer;
};

/* To be moved later to a better place */
static struct s3c2410_button s3c2410_buttons[] = {
	{  IRQ_EINT1,  S3C2410_GPF1,   S3C2410_GPF1_EINT1,    KEY_ENTER, 	     "Select", 0 },
	{  IRQ_EINT3,  S3C2410_GPF3,   S3C2410_GPF3_EINT3,   KEY_RECORD,	     "Record", 0 },
	{  IRQ_EINT4,  S3C2410_GPF4,   S3C2410_GPF4_EINT4, KEY_CALENDAR,	   "Calendar", 0 },
	{ IRQ_EINT5,  S3C2410_GPF5,  S3C2410_GPF5_EINT5,   KEY_COFFEE,	   "Contacts", 0 }, /* TODO: find a better key :P */
	{ IRQ_EINT6,  S3C2410_GPF6,  S3C2410_GPF6_EINT6,     KEY_MAIL,	       "Mail", 0 },
	{ IRQ_EINT7,  S3C2410_GPF7,  S3C2410_GPF7_EINT7, KEY_HOMEPAGE,	       "Home", 0 },
	{ IRQ_EINT8,  S3C2410_GPG0,  S3C2410_GPG0_EINT8,     KEY_LEFT,	 "Left_arrow", 0 },
	{ IRQ_EINT11,  S3C2410_GPG3,  S3C2410_GPG3_EINT11,    KEY_RIGHT,	"Right_arrow", 0 },
	{ IRQ_EINT13,  S3C2410_GPG5,  S3C2410_GPG5_EINT13,       KEY_UP,	   "Up_arrow", 0 },
	{ IRQ_EINT14, S3C2410_GPG6, S3C2410_GPG6_EINT14,     KEY_DOWN,	 "Down_arrow", 0 },
};

struct s3c2410_buttons_private {
	struct input_dev	dev;
	spinlock_t		lock;
	int 			count;
	int			shift;	
	char			phys[32];
};

static struct s3c2410_buttons_private priv;

static irqreturn_t s3c2410but_keyevent(int irq, void *dev_id, struct pt_regs *regs)
{
	struct s3c2410_button *button = (struct s3c2410_button *)dev_id;
	int down;

	if (!button)
		return IRQ_HANDLED;

	down = !(s3c2410_gpio_getpin(button->pin));

	/* the power button of the ipaq are tricky. They send 'released' events even
	 * when the button are already released. The work-around is to proceed only 
	 * if the state changed.
	 **/
	if (button->last_state == down)
		return IRQ_HANDLED;

	button->last_state = down;
	
	dprintk("%s button %s\n",button->name, down ? "pressed" : "released");

	input_report_key(&priv.dev, button->keycode, down);
	input_sync(&priv.dev);

	if (down)
		mod_timer(&button->timer, jiffies + REPEAT_DELAY);

	return IRQ_HANDLED;
}

static void s3c2410but_timer_callback(unsigned long data)
{
		struct s3c2410_button *button = (struct s3c2410_button *) data;
		int down;

		down = !(s3c2410_gpio_getpin(button->pin));
		
		if (down) {
			dprintk("Timer: %s button %s\n",button->name, down ? "pressed" : "released");
			input_report_key(&priv.dev, button->keycode, down);
			input_sync(&priv.dev);
			mod_timer(&button->timer, jiffies + REPEAT_DELAY);
		}
}

static int __init s3c2410but_probe(struct device *dev)
{
	int i;

	/* Initialise input stuff */
	memset(&priv, 0, sizeof(struct s3c2410_buttons_private));
	init_input_dev(&priv.dev);
	priv.dev.evbit[0] = BIT(EV_KEY);
	sprintf(priv.phys, "input/s3c2410_buttons0");

	priv.dev.private = &priv;
	priv.dev.name = DRV_NAME;
	priv.dev.phys = priv.phys;
	priv.dev.id.bustype = BUS_HOST;
	priv.dev.id.vendor = 0xDEAD;
	priv.dev.id.product = 0xBEEF;
	priv.dev.id.version = S3C2410BUTVERSION;


	for (i = 0; i < ARRAY_SIZE (s3c2410_buttons); i++) {
		set_bit(s3c2410_buttons[i].keycode, priv.dev.keybit);
		s3c2410_gpio_cfgpin(s3c2410_buttons[i].pin,s3c2410_buttons[i].pin_setting);
		request_irq (s3c2410_buttons[i].irq, s3c2410but_keyevent,\
				SA_SAMPLE_RANDOM, s3c2410_buttons[i].name, &s3c2410_buttons[i]);
		set_irq_type(s3c2410_buttons[i].irq, IRQT_BOTHEDGE);
		
		init_timer(&s3c2410_buttons[i].timer);
		s3c2410_buttons[i].timer.function = s3c2410but_timer_callback;
		s3c2410_buttons[i].timer.data     = (unsigned long)&s3c2410_buttons[i];
	}
	
	printk(KERN_INFO "%s successfully loaded\n", DRV_NAME);

	/* All went ok, so register to the input system */
	input_register_device(&priv.dev);

	return 0;
}

static int s3c2410but_remove(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE (s3c2410_buttons); i++) {
		disable_irq(s3c2410_buttons[i].irq);
		free_irq(s3c2410_buttons[i].irq,&priv.dev);
	}

	input_unregister_device(&priv.dev);

	return 0;
}


static struct device_driver s3c2410but_driver = {
       .name           = DRV_NAME,
       .bus            = &platform_bus_type,
       .probe          = s3c2410but_probe,
       .remove         = s3c2410but_remove,
};


int __init s3c2410but_init(void)
{
	return driver_register(&s3c2410but_driver);
}

void __exit s3c2410but_exit(void)
{
	driver_unregister(&s3c2410but_driver);
}

module_init(s3c2410but_init);
module_exit(s3c2410but_exit);




