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
 
 2005.10.25: godori(www.aesop-embedded.org)
 
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
#define DRV_NAME		"s3c2440-buttons"

#define REPEAT_DELAY		HZ/10

#ifdef DEBUG
#    define dprintk(msg...) printk(KERN_DEBUG "s3c2410_buttons: " msg);
#else
#    define dprintk(msg...)
#endif

MODULE_AUTHOR("Arnaud Patard <arnaud.patard@rtp-net.org>");
MODULE_DESCRIPTION("s3c2410 buttons driver");
MODULE_LICENSE("GPL");

struct s3c2410_button
{
    int             irq;
    int             pin;
    int             pin_setting;
    int             keycode;
    char           *name;
    int             last_state;
    struct timer_list timer;
};

#if 0 // This is for aESOP2440 button, from 2.4.20
    #define IRQ_AESOP_BT0               IRQ_EINT0
    #define IRQ_AESOP_BT1               IRQ_EINT1
    #define IRQ_AESOP_BT2               IRQ_EINT2
    #define IRQ_AESOP_BT3               IRQ_EINT3
    #define IRQ_AESOP_BT4               IRQ_EINT4
    #define IRQ_AESOP_BT5               IRQ_EINT5
    #define IRQ_AESOP_BT6               IRQ_EINT6
    #define IRQ_AESOP_BT7               IRQ_EINT7
    #define IRQ_AESOP_BT8               IRQ_EINT12
    #define IRQ_AESOP_BT9               IRQ_EINT13
    #define IRQ_AESOP_BT10              IRQ_EINT14
    #define IRQ_AESOP_BT11              IRQ_EINT15
    
/* button interrupt check structure: now used */
BUTTON_TYPE     gpio_buttons[] = {
    {IRQ_AESOP_BT0, GPIO_AESOP_BT0,   SCANCODE_ENTER  , KEY_RELEASED, GPIO_FALLING_EDGE, GPIO_PULLUP_EN, "Enter", keypad_handler},
    {IRQ_AESOP_BT1, GPIO_AESOP_BT1,   SCANCODE_UP     , KEY_RELEASED, GPIO_FALLING_EDGE, GPIO_PULLUP_EN, "Up"   , keypad_handler},
    {IRQ_AESOP_BT2, GPIO_AESOP_BT2,   SCANCODE_3      , KEY_RELEASED, GPIO_FALLING_EDGE, GPIO_PULLUP_EN, "3"    , keypad_handler},
    {IRQ_AESOP_BT3, GPIO_AESOP_BT3,   SCANCODE_RIGHT  , KEY_RELEASED, GPIO_FALLING_EDGE, GPIO_PULLUP_EN, "Right", keypad_handler},
    {IRQ_AESOP_BT4, GPIO_AESOP_BT4,   SCANCODE_2      , KEY_RELEASED, GPIO_FALLING_EDGE, GPIO_PULLUP_EN, "2"    , keypad_handler},
    {IRQ_AESOP_BT5, GPIO_AESOP_BT5,   SCANCODE_SPACE  , KEY_RELEASED, GPIO_FALLING_EDGE, GPIO_PULLUP_EN, "Space", keypad_handler},
    {IRQ_AESOP_BT6, GPIO_AESOP_BT6,   SCANCODE_1      , KEY_RELEASED, GPIO_FALLING_EDGE, GPIO_PULLUP_EN, "1"    , keypad_handler},
    {IRQ_AESOP_BT7, GPIO_AESOP_BT7,   SCANCODE_DOWN   , KEY_RELEASED, GPIO_FALLING_EDGE, GPIO_PULLUP_EN, "Down" , keypad_handler},
    {IRQ_AESOP_BT8, GPIO_AESOP_BT8,   SCANCODE_4      , KEY_RELEASED, GPIO_FALLING_EDGE, GPIO_PULLUP_EN, "4"    , keypad_handler},
    {IRQ_AESOP_BT9, GPIO_AESOP_BT9,   SCANCODE_LEFT   , KEY_RELEASED, GPIO_FALLING_EDGE, GPIO_PULLUP_EN, "Left" , keypad_handler},
    {IRQ_AESOP_BT10, GPIO_AESOP_BT10, SCANCODE_0      , KEY_RELEASED, GPIO_FALLING_EDGE, GPIO_PULLUP_EN, "0"    , keypad_handler},
    {IRQ_AESOP_BT11, GPIO_AESOP_BT11, SCANCODE_5      , KEY_RELEASED, GPIO_FALLING_EDGE, GPIO_PULLUP_EN, "5"    , keypad_handler},
    

    {END_OF_LIST, END_OF_LIST, 0, 0, 0, 0, NULL, NULL}
};

// This is for aESOP2440 button, from 2.4.20
    #define GPIO_AESOP_BT0		GPIO_F0
    #define GPIO_AESOP_BT1		GPIO_F1
    #define GPIO_AESOP_BT2		GPIO_F2
    #define GPIO_AESOP_BT3		GPIO_F3
    #define GPIO_AESOP_BT4		GPIO_F4
    #define GPIO_AESOP_BT5		GPIO_F5
    #define GPIO_AESOP_BT6		GPIO_F6
    #define GPIO_AESOP_BT7		GPIO_F7
    #define GPIO_AESOP_BT8		GPIO_G4
    #define GPIO_AESOP_BT9		GPIO_G5
    #define GPIO_AESOP_BT10		GPIO_G6
    #define GPIO_AESOP_BT11		GPIO_G7


#endif


/* To be moved later to a better place */
/* ghcstop: key definition */
static struct s3c2410_button s3c2410_buttons[] = {
    {IRQ_EINT0 , S3C2410_GPF0, S3C2410_GPF0_EINT0 , KEY_ENTER, "Enter", 0},
    {IRQ_EINT1 , S3C2410_GPF1, S3C2410_GPF1_EINT1 , KEY_UP   , "Up", 0},
    {IRQ_EINT2 , S3C2410_GPF2, S3C2410_GPF2_EINT2 , KEY_3    , "3", 0},
    {IRQ_EINT3 , S3C2410_GPF3, S3C2410_GPF3_EINT3 , KEY_RIGHT, "Right", 0},
    {IRQ_EINT4 , S3C2410_GPF4, S3C2410_GPF4_EINT4 , KEY_2    , "2", 0}, /* TODO: find a better key :P */
    {IRQ_EINT5 , S3C2410_GPF5, S3C2410_GPF5_EINT5 , KEY_SPACE, "Space", 0},
    {IRQ_EINT6 , S3C2410_GPF6, S3C2410_GPF6_EINT6 , KEY_1    , "1", 0},
    {IRQ_EINT7 , S3C2410_GPF7, S3C2410_GPF7_EINT7 , KEY_DOWN , "Down", 0},
    {IRQ_EINT12, S3C2410_GPG4, S3C2410_GPG4_EINT12, KEY_4    , "4", 0},
    {IRQ_EINT13, S3C2410_GPG5, S3C2410_GPG5_EINT13, KEY_LEFT , "Left", 0},
    {IRQ_EINT14, S3C2410_GPG6, S3C2410_GPG6_EINT14, KEY_0    , "0", 0},
    {IRQ_EINT15, S3C2410_GPG7, S3C2410_GPG7_EINT15, KEY_5    , "5", 0},
};

struct s3c2410_buttons_private
{
    struct input_dev dev;
    spinlock_t      lock;
    int             count;
    int             shift;
    char            phys[32];
};

static struct s3c2410_buttons_private priv;

static irqreturn_t
aesop2440but_keyevent(int irq, void *dev_id, struct pt_regs *regs)
{
    struct s3c2410_button *button = (struct s3c2410_button *) dev_id;
    int             down;

    if (!button)
        return IRQ_HANDLED;

    down = !(s3c2410_gpio_getpin(button->pin));

    /*
     * the power button of the ipaq are tricky. They send 'released' events even when the button are already released. The work-around is to proceed only if the state changed. 
     */
    if (button->last_state == down)
        return IRQ_HANDLED;

    button->last_state = down;

    dprintk("%s button %s\n", button->name, down ? "pressed" : "released");

    input_report_key(&priv.dev, button->keycode, down);
    input_sync(&priv.dev);

    if (down)
        mod_timer(&button->timer, jiffies + REPEAT_DELAY);

    return IRQ_HANDLED;
}

static void
aesop2440but_timer_callback(unsigned long data)
{
    struct s3c2410_button *button = (struct s3c2410_button *) data;
    int             down;

    /*
     * button이 눌리면 low active이므로 gpio에 대한 read값은 0이다.
     * 이녀석을 !로 해야 down이 1이 된다.
     *
     * 즉, low일때 down이므로 ! 를 해줘야 함.
     */
    down = !(s3c2410_gpio_getpin(button->pin));

    if (down)
    {
        dprintk("Timer: %s button %s\n", button->name, down ? "pressed" : "released");
        input_report_key(&priv.dev, button->keycode, down);
        input_sync(&priv.dev);
        mod_timer(&button->timer, jiffies + REPEAT_DELAY);
    }
}

static int __init
aesop2440but_probe(struct device *dev)
{
    int             i;

    /*
     * Initialise input stuff 
     */
    memset(&priv, 0, sizeof(struct s3c2410_buttons_private));
    init_input_dev(&priv.dev);
    priv.dev.evbit[0] = BIT(EV_KEY);
    sprintf(priv.phys, "input/s3c2440_buttons0");

    priv.dev.private = &priv;
    priv.dev.name = DRV_NAME;
    priv.dev.phys = priv.phys;
    priv.dev.id.bustype = BUS_HOST;
    priv.dev.id.vendor = 0xDEAD;
    priv.dev.id.product = 0xBEEF;
    priv.dev.id.version = S3C2410BUTVERSION;


    for (i = 0; i < ARRAY_SIZE(s3c2410_buttons); i++)
    {
        set_bit(s3c2410_buttons[i].keycode, priv.dev.keybit);
        s3c2410_gpio_cfgpin(s3c2410_buttons[i].pin, s3c2410_buttons[i].pin_setting);
        request_irq(s3c2410_buttons[i].irq, aesop2440but_keyevent, SA_SAMPLE_RANDOM, s3c2410_buttons[i].name, &s3c2410_buttons[i]);
        set_irq_type(s3c2410_buttons[i].irq, IRQT_BOTHEDGE);

        init_timer(&s3c2410_buttons[i].timer);
        s3c2410_buttons[i].timer.function = aesop2440but_timer_callback;
        s3c2410_buttons[i].timer.data = (unsigned long) &s3c2410_buttons[i];
    }

    printk(KERN_INFO "%s successfully loaded\n", DRV_NAME);

    /*
     * All went ok, so register to the input system 
     */
    input_register_device(&priv.dev);

    return 0;
}

static int
aesop2440but_remove(struct device *dev)
{
    int             i;

    for (i = 0; i < ARRAY_SIZE(s3c2410_buttons); i++)
    {
        disable_irq(s3c2410_buttons[i].irq);
        free_irq(s3c2410_buttons[i].irq, &priv.dev);
    }

    input_unregister_device(&priv.dev);

    return 0;
}


static struct device_driver aesop2440but_driver = {
    .name = DRV_NAME,
    .bus = &platform_bus_type,
    .probe = aesop2440but_probe,
    .remove = aesop2440but_remove,
};


int __init
aesop2440but_init(void)
{
    return driver_register(&aesop2440but_driver);
}

void __exit
aesop2440but_exit(void)
{
    driver_unregister(&aesop2440but_driver);
}

module_init(aesop2440but_init);
module_exit(aesop2440but_exit);
