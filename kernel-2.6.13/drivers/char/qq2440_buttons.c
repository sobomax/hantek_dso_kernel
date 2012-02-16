#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <asm/irq.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <asm/arch/regs-gpio.h>
#include <asm/hardware.h>

#define DEVICE_NAME     "buttons"   /* 加载模式后，执行”cat /proc/devices”命令看到的设备名称 */
#define BUTTON_MAJOR    232         /* 主设备号 */

struct button_irq_desc {
    int irq;
	int pin;
	int pin_setting;
	int number;
    char *name;	
};

/* 用来指定按键所用的外部中断引脚及中断触发方式, 名字 */
static struct button_irq_desc button_irqs [] = {
    {IRQ_EINT19, S3C2410_GPG11, S3C2410_GPG11_EINT19, 0, "KEY1"}, /* K1 */
    {IRQ_EINT11, S3C2410_GPG3,  S3C2410_GPG3_EINT11,  1, "KEY2"}, /* K2 */
    {IRQ_EINT2,  S3C2410_GPF2,  S3C2410_GPF2_EINT2,   2, "KEY3"}, /* K3 */
    {IRQ_EINT0,  S3C2410_GPF0,  S3C2410_GPF0_EINT0,   3, "KEY4"}, /* K4 */
};

/* 按键被按下的次数(准确地说，是发生中断的次数) */
static volatile int key_values [] = {0, 0, 0, 0};

/* 等待队列: 
 * 当没有按键被按下时，如果有进程调用qq2440_buttons_read函数，
 * 它将休眠
 */
static DECLARE_WAIT_QUEUE_HEAD(button_waitq);

/* 中断事件标志, 中断服务程序将它置1，qq2440_buttons_read将它清0 */
static volatile int ev_press = 0;


static irqreturn_t buttons_interrupt(int irq, void *dev_id)
{
    struct button_irq_desc *button_irqs = (struct button_irq_desc *)dev_id;
    int up = s3c2410_gpio_getpin(button_irqs->pin);

	if (up)
		key_values[button_irqs->number] = (button_irqs->number + 1) + 0x80;
	else
		key_values[button_irqs->number] = (button_irqs->number + 1);
	
    ev_press = 1;                  /* 表示中断发生了 */
    wake_up_interruptible(&button_waitq);   /* 唤醒休眠的进程 */
    
    return IRQ_RETVAL(IRQ_HANDLED);
}


/* 应用程序对设备文件/dev/buttons执行open(...)时，
 * 就会调用qq2440_buttons_open函数
 */
static int qq2440_buttons_open(struct inode *inode, struct file *file)
{
    int i;
    int err;
    
    for (i = 0; i < sizeof(button_irqs)/sizeof(button_irqs[0]); i++) {
        // 注册中断处理函数
		s3c2410_gpio_cfgpin(button_irqs[i].pin,button_irqs[i].pin_setting);
        err = request_irq(button_irqs[i].irq, buttons_interrupt, NULL, 
                          button_irqs[i].name, (void *)&button_irqs[i]);
		set_irq_type(button_irqs[i].irq, IRQT_BOTHEDGE);
        if (err)
            break;
    }

    if (err) {
        // 释放已经注册的中断
        i--;
        for (; i >= 0; i--) {
			disable_irq(button_irqs[i].irq);
            free_irq(button_irqs[i].irq, (void *)&button_irqs[i]);
        }
        return -EBUSY;
    }
    
    return 0;
}


/* 应用程序对设备文件/dev/buttons执行close(...)时，
 * 就会调用qq2440_buttons_close函数
 */
static int qq2440_buttons_close(struct inode *inode, struct file *file)
{
    int i;
    
    for (i = 0; i < sizeof(button_irqs)/sizeof(button_irqs[0]); i++) {
        // 释放已经注册的中断
		disable_irq(button_irqs[i].irq);
        free_irq(button_irqs[i].irq, (void *)&button_irqs[i]);
    }

    return 0;
}


/* 应用程序对设备文件/dev/buttons执行read(...)时，
 * 就会调用qq2440_buttons_read函数
 */
static int qq2440_buttons_read(struct file *filp, char __user *buff, 
                                         size_t count, loff_t *offp)
{
    unsigned long err;

	if (!ev_press) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		else
			/* 如果ev_press等于0，休眠 */
			wait_event_interruptible(button_waitq, ev_press);
	}
    
    /* 执行到这里时，ev_press等于1，将它清0 */
    ev_press = 0;

    /* 将按键状态复制给用户，并清0 */
    err = copy_to_user(buff, (const void *)key_values, min(sizeof(key_values), count));
    memset((void *)key_values, 0, sizeof(key_values));

    return err ? -EFAULT : min(sizeof(key_values), count);
}

/**************************************************
* 当用户程序调用select函数时，本函数被调用
* 如果有按键数据，则select函数会立刻返回
* 如果没有按键数据，本函数使用poll_wait等待
**************************************************/
static unsigned int qq2440_buttons_poll(
        struct file *file,
        struct poll_table_struct *wait)
{
	unsigned int mask = 0;
    poll_wait(file, &button_waitq, wait);
    if (ev_press)
        mask |= POLLIN | POLLRDNORM;
    return mask;
}


/* 这个结构是字符设备驱动程序的核心
 * 当应用程序操作设备文件时所调用的open、read、write等函数，
 * 最终会调用这个结构中的对应函数
 */
static struct file_operations qq2440_buttons_fops = {
    .owner   =   THIS_MODULE,    /* 这是一个宏，指向编译模块时自动创建的__this_module变量 */
    .open    =   qq2440_buttons_open,
    .release =   qq2440_buttons_close, 
    .read    =   qq2440_buttons_read,
	.poll	 =   qq2440_buttons_poll,
};

/*
 * 执行“insmod qq2440_buttons.ko”命令时就会调用这个函数
 */
static int __init qq2440_buttons_init(void)
{
    int ret;

    /* 注册字符设备驱动程序
     * 参数为主设备号、设备名字、file_operations结构；
     * 这样，主设备号就和具体的file_operations结构联系起来了，
     * 操作主设备为BUTTON_MAJOR的设备文件时，就会调用qq2440_buttons_fops中的相关成员函数
     * BUTTON_MAJOR可以设为0，表示由内核自动分配主设备号
     */
    ret = register_chrdev(BUTTON_MAJOR, DEVICE_NAME, &qq2440_buttons_fops);
    if (ret < 0) {
      printk(DEVICE_NAME " can't register major number\n");
      return ret;
    }
	devfs_mk_cdev(MKDEV(BUTTON_MAJOR, 0), S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP, DEVICE_NAME);
    
    printk(DEVICE_NAME " initialized\n");
    return 0;
}

/*
 * 执行”rmmod qq2440_buttons.ko”命令时就会调用这个函数 
 */
static void __exit qq2440_buttons_exit(void)
{
    /* 卸载驱动程序 */
	devfs_remove(DEVICE_NAME);
    unregister_chrdev(BUTTON_MAJOR, DEVICE_NAME);
}

/* 这两行指定驱动程序的初始化函数和卸载函数 */
module_init(qq2440_buttons_init);
module_exit(qq2440_buttons_exit);

/* 描述驱动程序的一些信息，不是必须的 */
MODULE_AUTHOR("http://www.arm9.net");             // 驱动程序的作者
MODULE_DESCRIPTION("S3C2410/S3C2440 BUTTON Driver");   // 一些描述信息
MODULE_LICENSE("GPL");                              // 遵循的协议

