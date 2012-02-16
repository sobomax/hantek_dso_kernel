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

#define DEVICE_NAME     "buttons"   /* ����ģʽ��ִ�С�cat /proc/devices����������豸���� */
#define BUTTON_MAJOR    232         /* ���豸�� */

struct button_irq_desc {
    int irq;
	int pin;
	int pin_setting;
	int number;
    char *name;	
};

/* ����ָ���������õ��ⲿ�ж����ż��жϴ�����ʽ, ���� */
static struct button_irq_desc button_irqs [] = {
    {IRQ_EINT19, S3C2410_GPG11, S3C2410_GPG11_EINT19, 0, "KEY1"}, /* K1 */
    {IRQ_EINT11, S3C2410_GPG3,  S3C2410_GPG3_EINT11,  1, "KEY2"}, /* K2 */
    {IRQ_EINT2,  S3C2410_GPF2,  S3C2410_GPF2_EINT2,   2, "KEY3"}, /* K3 */
    {IRQ_EINT0,  S3C2410_GPF0,  S3C2410_GPF0_EINT0,   3, "KEY4"}, /* K4 */
};

/* ���������µĴ���(׼ȷ��˵���Ƿ����жϵĴ���) */
static volatile int key_values [] = {0, 0, 0, 0};

/* �ȴ�����: 
 * ��û�а���������ʱ������н��̵���qq2440_buttons_read������
 * ��������
 */
static DECLARE_WAIT_QUEUE_HEAD(button_waitq);

/* �ж��¼���־, �жϷ����������1��qq2440_buttons_read������0 */
static volatile int ev_press = 0;


static irqreturn_t buttons_interrupt(int irq, void *dev_id)
{
    struct button_irq_desc *button_irqs = (struct button_irq_desc *)dev_id;
    int up = s3c2410_gpio_getpin(button_irqs->pin);

	if (up)
		key_values[button_irqs->number] = (button_irqs->number + 1) + 0x80;
	else
		key_values[button_irqs->number] = (button_irqs->number + 1);
	
    ev_press = 1;                  /* ��ʾ�жϷ����� */
    wake_up_interruptible(&button_waitq);   /* �������ߵĽ��� */
    
    return IRQ_RETVAL(IRQ_HANDLED);
}


/* Ӧ�ó�����豸�ļ�/dev/buttonsִ��open(...)ʱ��
 * �ͻ����qq2440_buttons_open����
 */
static int qq2440_buttons_open(struct inode *inode, struct file *file)
{
    int i;
    int err;
    
    for (i = 0; i < sizeof(button_irqs)/sizeof(button_irqs[0]); i++) {
        // ע���жϴ�����
		s3c2410_gpio_cfgpin(button_irqs[i].pin,button_irqs[i].pin_setting);
        err = request_irq(button_irqs[i].irq, buttons_interrupt, NULL, 
                          button_irqs[i].name, (void *)&button_irqs[i]);
		set_irq_type(button_irqs[i].irq, IRQT_BOTHEDGE);
        if (err)
            break;
    }

    if (err) {
        // �ͷ��Ѿ�ע����ж�
        i--;
        for (; i >= 0; i--) {
			disable_irq(button_irqs[i].irq);
            free_irq(button_irqs[i].irq, (void *)&button_irqs[i]);
        }
        return -EBUSY;
    }
    
    return 0;
}


/* Ӧ�ó�����豸�ļ�/dev/buttonsִ��close(...)ʱ��
 * �ͻ����qq2440_buttons_close����
 */
static int qq2440_buttons_close(struct inode *inode, struct file *file)
{
    int i;
    
    for (i = 0; i < sizeof(button_irqs)/sizeof(button_irqs[0]); i++) {
        // �ͷ��Ѿ�ע����ж�
		disable_irq(button_irqs[i].irq);
        free_irq(button_irqs[i].irq, (void *)&button_irqs[i]);
    }

    return 0;
}


/* Ӧ�ó�����豸�ļ�/dev/buttonsִ��read(...)ʱ��
 * �ͻ����qq2440_buttons_read����
 */
static int qq2440_buttons_read(struct file *filp, char __user *buff, 
                                         size_t count, loff_t *offp)
{
    unsigned long err;

	if (!ev_press) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		else
			/* ���ev_press����0������ */
			wait_event_interruptible(button_waitq, ev_press);
	}
    
    /* ִ�е�����ʱ��ev_press����1��������0 */
    ev_press = 0;

    /* ������״̬���Ƹ��û�������0 */
    err = copy_to_user(buff, (const void *)key_values, min(sizeof(key_values), count));
    memset((void *)key_values, 0, sizeof(key_values));

    return err ? -EFAULT : min(sizeof(key_values), count);
}

/**************************************************
* ���û��������select����ʱ��������������
* ����а������ݣ���select���������̷���
* ���û�а������ݣ�������ʹ��poll_wait�ȴ�
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


/* ����ṹ���ַ��豸��������ĺ���
 * ��Ӧ�ó�������豸�ļ�ʱ�����õ�open��read��write�Ⱥ�����
 * ���ջ��������ṹ�еĶ�Ӧ����
 */
static struct file_operations qq2440_buttons_fops = {
    .owner   =   THIS_MODULE,    /* ����һ���ָ꣬�����ģ��ʱ�Զ�������__this_module���� */
    .open    =   qq2440_buttons_open,
    .release =   qq2440_buttons_close, 
    .read    =   qq2440_buttons_read,
	.poll	 =   qq2440_buttons_poll,
};

/*
 * ִ�С�insmod qq2440_buttons.ko������ʱ�ͻ�����������
 */
static int __init qq2440_buttons_init(void)
{
    int ret;

    /* ע���ַ��豸��������
     * ����Ϊ���豸�š��豸���֡�file_operations�ṹ��
     * ���������豸�žͺ;����file_operations�ṹ��ϵ�����ˣ�
     * �������豸ΪBUTTON_MAJOR���豸�ļ�ʱ���ͻ����qq2440_buttons_fops�е���س�Ա����
     * BUTTON_MAJOR������Ϊ0����ʾ���ں��Զ��������豸��
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
 * ִ�С�rmmod qq2440_buttons.ko������ʱ�ͻ����������� 
 */
static void __exit qq2440_buttons_exit(void)
{
    /* ж���������� */
	devfs_remove(DEVICE_NAME);
    unregister_chrdev(BUTTON_MAJOR, DEVICE_NAME);
}

/* ������ָ����������ĳ�ʼ��������ж�غ��� */
module_init(qq2440_buttons_init);
module_exit(qq2440_buttons_exit);

/* �������������һЩ��Ϣ�����Ǳ���� */
MODULE_AUTHOR("http://www.arm9.net");             // �������������
MODULE_DESCRIPTION("S3C2410/S3C2440 BUTTON Driver");   // һЩ������Ϣ
MODULE_LICENSE("GPL");                              // ��ѭ��Э��

