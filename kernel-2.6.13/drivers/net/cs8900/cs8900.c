
/*
 * linux/drivers/net/cs8900.c
 *
 * Author: Abraham van der Merwe <abraham at 2d3d.co.za>
 *
 * A Cirrus Logic CS8900A driver for Linux
 * based on the cs89x0 driver written by Russell Nelson,
 * Donald Becker, and others.
 *
 * This source code is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * History:
 *    22-May-2002  Initial version (Abraham vd Merwe)
 *    30-May-2002  Added char device support for eeprom (Frank Becker)
 *    24-Jan-2004  Fixups for 2.6 (Frank Becker)
 *	  15-July-2004 Modified for SMDK2410 (Roc Wu pwu at jadechip.com)
 */
 
#define VERSION_STRING "Cirrus Logic CS8900A driver for Linux (Modified for SMDK2410)"
/*
 * At the moment the driver does not support memory mode operation.
 * It is trivial to implement this, but not worth the effort.
 */

/*
 * TODO:
 *
 *   1. Sort out ethernet checksum
 *   2. If !ready in send_start(), queue buffer and send it in interrupt handler
 *      when we receive a BufEvent with Rdy4Tx, send it again. dangerous!
 *   3. how do we prevent interrupt handler destroying integrity of get_stats()?
 *   4. Change reset code to check status.
 *   5. Implement set_mac_address and remove fake mac address
 *   7. Link status detection stuff
 *   8. Write utility to write EEPROM, do self testing, etc.
 *   9. Implement DMA routines (I need a board w/ DMA support for that)
 *  10. Power management
 *  11. Add support for multiple ethernet chips
 */

// added BSt
#include <linux/config.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>

// Added BSt
#include <asm/mach-types.h>

#ifdef CONFIG_SA1100_CERF
#include "asm/arch/cerf.h"
#endif

#ifdef CONFIG_ARCH_QQ2440
#include "asm/arch/sbc2440v3-map.h"
#include <asm/arch/regs-mem.h>
#endif

#ifdef CONFIG_ARCH_SMDK2410
//#include "asm/arch/smdk2410.h"
#include <asm/arch/regs-mem.h>
#endif

#include "cs8900.h"

//#define FULL_DUPLEX
//#define DEBUG
static char cirrus_mac[18] = "08:00:3e:21:c7:f7";

typedef struct {
	struct net_device_stats stats;
	u16 txlen;
	int char_devnum;

        spinlock_t lock;
} cs8900_t;

int cs8900_probe (struct net_device *dev);
static struct net_device cs8900_dev =
{
        init: cs8900_probe
};

/* 
 * There seems to be no way to determine the exact size of the eeprom, 
 * so we use the largest size.
 * FIXME: Verify it's safe to read/write past the end of a 64/128
 *        byte eeprom. 
 *
 * Possible eeprom sizes: 
 * Cx46 -  64 bytes
 * Cx56 - 128 bytes
 * Cx66 - 256 bytes
 */
#define MAX_EEPROM_SIZE		256

static int cs8900_eeprom_fopen(struct inode *inode, struct file *file);
static int cs8900_eeprom_frelease(struct inode *inode, struct file *file);
static loff_t cs8900_eeprom_fllseek(struct file * file,loff_t offset, int flags);
static ssize_t cs8900_eeprom_fread(struct file *file, char *buf, size_t count, loff_t *f_pos);
static ssize_t cs8900_eeprom_fwrite(struct file *file, const char *buf, size_t count, loff_t *f_pos);
static struct file_operations cs8900_eeprom_fops = {
        owner:          THIS_MODULE,
        open:           cs8900_eeprom_fopen,
        release:        cs8900_eeprom_frelease,
        llseek:         cs8900_eeprom_fllseek,
        read:           cs8900_eeprom_fread,
        write:          cs8900_eeprom_fwrite,
};      

static u16 cs8900_eeprom_cache[MAX_EEPROM_SIZE/2];

/*
 * I/O routines
 */

static inline u16 cs8900_read (struct net_device *dev,u16 reg)
{
	outw (reg,dev->base_addr + PP_Address);
	return (inw (dev->base_addr + PP_Data));
}

static inline void cs8900_write (struct net_device *dev,u16 reg,u16 value)
{
	outw (reg,dev->base_addr + PP_Address);
	outw (value,dev->base_addr + PP_Data);
}

static inline void cs8900_set (struct net_device *dev,u16 reg,u16 value)
{
	cs8900_write (dev,reg,cs8900_read (dev,reg) | value);
}

static inline void cs8900_clear (struct net_device *dev,u16 reg,u16 value)
{
	cs8900_write (dev,reg,cs8900_read (dev,reg) & ~value);
}

static inline void cs8900_frame_read (struct net_device *dev,struct sk_buff *skb,u16 length)
{
	insw (dev->base_addr,skb_put (skb,length),(length + 1) / 2);
}

static inline void cs8900_frame_write (struct net_device *dev,struct sk_buff *skb)
{
	outsw (dev->base_addr,skb->data,(skb->len + 1) / 2);
}

/*
 * EEPROM I/O routines
 */

static int cs8900_eeprom_wait (struct net_device *dev)
{
	int i;

	for (i = 0; i < 3000; i++) {
		if (!(cs8900_read (dev,PP_SelfST) & SIBUSY))
			return (0);
		udelay (1);
	}

	return (-1);
}

static int cs8900_eeprom_read (struct net_device *dev,u16 *value,u16 offset)
{
	if (cs8900_eeprom_wait (dev) < 0)
		return (-1);

	cs8900_write (dev,PP_EEPROMCommand,offset | EEReadRegister);

	if (cs8900_eeprom_wait (dev) < 0)
		return (-1);

	*value = cs8900_read (dev,PP_EEPROMData);

	return (0);
}

static int cs8900_eeprom_write (struct net_device *dev,u16 *value,u16 offset)
{
	cs8900_eeprom_wait(dev);
        cs8900_write(dev, PP_EEPROMCommand, (EEWriteEnable));
	cs8900_eeprom_wait(dev);
        cs8900_write(dev, PP_EEPROMData, *value);
	cs8900_eeprom_wait(dev);
        cs8900_write(dev, PP_EEPROMCommand, (offset | EEWriteRegister));
	cs8900_eeprom_wait(dev);
        cs8900_write(dev, PP_EEPROMCommand, (EEWriteDisable));
	cs8900_eeprom_wait(dev);

        return 0;
}

/*
 * Debugging functions
 */

#ifdef DEBUG
static inline int printable (int c)
{
	return ((c >= 32 && c <= 126) ||
			(c >= 174 && c <= 223) ||
			(c >= 242 && c <= 243) ||
			(c >= 252 && c <= 253));
}

static void dump16 (struct net_device *dev,const u8 *s,size_t len)
{
	int i;
	char str[128];

	if (!len) return;

	*str = '\0';

	for (i = 0; i < len; i++) {
		if (i && !(i % 4)) strcat (str," ");
		sprintf (str,"%s%.2x ",str,s[i]);
	}

	for ( ; i < 16; i++) {
		if (i && !(i % 4)) strcat (str," ");
		strcat (str,"   ");
	}

	strcat (str," ");
	for (i = 0; i < len; i++) sprintf (str,"%s%c",str,printable (s[i]) ? s[i] : '.');

	printk (KERN_DEBUG "%s:     %s\n",dev->name,str);
}

static void hexdump (struct net_device *dev,const void *ptr,size_t size)
{
	const u8 *s = (u8 *) ptr;
	int i;
	for (i = 0; i < size / 16; i++, s += 16) dump16 (dev,s,16);
	dump16 (dev,s,size % 16);
}

static void dump_packet (struct net_device *dev,struct sk_buff *skb,const char *type)
{
	printk (KERN_INFO "%s: %s %d byte frame %.2x:%.2x:%.2x:%.2x:%.2x:%.2x to %.2x:%.2x:%.2x:%.2x:%.2x:%.2x type %.4x\n",
			dev->name,
			type,
			skb->len,
			skb->data[0],skb->data[1],skb->data[2],skb->data[3],skb->data[4],skb->data[5],
			skb->data[6],skb->data[7],skb->data[8],skb->data[9],skb->data[10],skb->data[11],
			(skb->data[12] << 8) | skb->data[13]);
	if (skb->len < 0x100) hexdump (dev,skb->data,skb->len);
}

static void eepromdump( struct net_device *dev)
{
	u16 buf[0x80];
	u16 i;
	int count;
	int total;

	if( cs8900_read( dev, PP_SelfST) & EEPROMpresent)
	{
		printk (KERN_INFO "%s: EEPROM present\n",dev->name);
	}
	else
	{
		printk (KERN_INFO "%s: NO EEPROM present\n",dev->name);
		return;
	}

	if( cs8900_read( dev, PP_SelfST) & EEPROMOK)
	{
		printk (KERN_INFO "%s: EEPROM OK\n",dev->name);
	}
	else
	{
		printk (KERN_INFO "%s: EEPROM checksum mismatch - fixing...\n",dev->name);
	}

	printk (KERN_INFO "%s: Hexdump\n",dev->name);
	for( i=0; i<0x80; i++)
	{
		cs8900_eeprom_read( dev, &buf[i], i);
	}
	hexdump( dev, buf, 0x100);

	if( buf[0] & 0x0100)
	{
		printk (KERN_INFO "%s: non-sequential EEPROM\n",dev->name);
	}
	else
	{
		printk (KERN_INFO "%s: sequential EEPROM\n",dev->name);
	}

	if( (buf[0] & 0xe000) == 0xa000)
	{
		printk (KERN_INFO "%s: Found reset configuration block\n",dev->name);
	}
	else
	{
		printk (KERN_INFO "%s: Reset configuration block not found\n",dev->name);
		return;
	}

	count = 2;
	total = buf[0] & 0xff;
	printk (KERN_INFO "%s: Reset configuration block size = %d bytes\n",dev->name, total);

	while( count < total)
	{
		int groupsize = (buf[count/2] >> 12) + 1;
		int basereg = (buf[count/2] &0x1ff);
		printk (KERN_INFO "%s: Group size = %d words\n",dev->name, groupsize);
		printk (KERN_INFO "%s:  Base register = %x\n",dev->name, basereg);
		count += (groupsize + 1)*2;
	}
}

#endif	/* #ifdef DEBUG */

/*
 * Driver functions
 */

static void cs8900_receive (struct net_device *dev)
{
	cs8900_t *priv = (cs8900_t *) dev->priv;
	struct sk_buff *skb;
	u16 status,length;

	status = cs8900_read (dev,PP_RxStatus);
	length = cs8900_read (dev,PP_RxLength);

	if (!(status & RxOK)) {
		priv->stats.rx_errors++;
		if ((status & (Runt | Extradata))) priv->stats.rx_length_errors++;
		if ((status & CRCerror)) priv->stats.rx_crc_errors++;
		return;
	}

	if ((skb = dev_alloc_skb (length + 4)) == NULL) {
		priv->stats.rx_dropped++;
		return;
	}

	skb->dev = dev;
	skb_reserve (skb,2);

	cs8900_frame_read (dev,skb,length);

#ifdef FULL_DUPLEX
	dump_packet (dev,skb,"recv");
#endif	/* #ifdef FULL_DUPLEX */

	skb->protocol = eth_type_trans (skb,dev);

	netif_rx (skb);
	dev->last_rx = jiffies;

	priv->stats.rx_packets++;
	priv->stats.rx_bytes += length;
}

static int cs8900_send_start (struct sk_buff *skb,struct net_device *dev)
{
	cs8900_t *priv = (cs8900_t *) dev->priv;
	u16 status;

	spin_lock_irq(&priv->lock);
	netif_stop_queue (dev);

	cs8900_write (dev,PP_TxCMD,TxStart (After5));
	cs8900_write (dev,PP_TxLength,skb->len);

	status = cs8900_read (dev,PP_BusST);

	if ((status & TxBidErr)) {
		spin_unlock_irq(&priv->lock);
		printk (KERN_WARNING "%s: Invalid frame size %d!\n",dev->name,skb->len);
		priv->stats.tx_errors++;
		priv->stats.tx_aborted_errors++;
		priv->txlen = 0;
		return (1);
	}

	if (!(status & Rdy4TxNOW)) {
		spin_unlock_irq(&priv->lock);
		printk (KERN_WARNING "%s: Transmit buffer not free!\n",dev->name);
		priv->stats.tx_errors++;
		priv->txlen = 0;
		/* FIXME: store skb and send it in interrupt handler */
		return (1);
	}

	cs8900_frame_write (dev,skb);
	spin_unlock_irq(&priv->lock);

#ifdef DEBUG
	dump_packet (dev,skb,"send");
#endif	/* #ifdef DEBUG */

	dev->trans_start = jiffies;

	dev_kfree_skb (skb);

	priv->txlen = skb->len;

	return (0);
}

static irqreturn_t cs8900_interrupt (int irq,void *id,struct pt_regs *regs)
{
	struct net_device *dev = (struct net_device *) id;
	cs8900_t *priv;
	volatile u16 status;
	irqreturn_t handled = 0;
		
	if (dev->priv == NULL) {
		printk (KERN_WARNING "%s: irq %d for unknown device.\n",dev->name,irq);
		return 0;
	}

	priv = (cs8900_t *) dev->priv;
	
	while ((status = cs8900_read (dev, PP_ISQ))) {
		handled = 1;

		switch (RegNum (status)) {
		case RxEvent:
			cs8900_receive (dev);
			break;

		case TxEvent:
			priv->stats.collisions += ColCount (cs8900_read (dev,PP_TxCOL));
			if (!(RegContent (status) & TxOK)) {
				priv->stats.tx_errors++;
				if ((RegContent (status) & Out_of_window)) priv->stats.tx_window_errors++;
				if ((RegContent (status) & Jabber)) priv->stats.tx_aborted_errors++;
				break;
			} else if (priv->txlen) {
				priv->stats.tx_packets++;
				priv->stats.tx_bytes += priv->txlen;
			}
			priv->txlen = 0;
			netif_wake_queue (dev);
			break;

		case BufEvent:
			if ((RegContent (status) & RxMiss)) {
				u16 missed = MissCount (cs8900_read (dev,PP_RxMISS));
				priv->stats.rx_errors += missed;
				priv->stats.rx_missed_errors += missed;
			}
			if ((RegContent (status) & TxUnderrun)) {
				priv->stats.tx_errors++;
				priv->stats.tx_fifo_errors++;

				priv->txlen = 0;
				netif_wake_queue (dev);
			}
			/* FIXME: if Rdy4Tx, transmit last sent packet (if any) */
			break;

		case TxCOL:
			priv->stats.collisions += ColCount (cs8900_read (dev,PP_TxCOL));
			break;

		case RxMISS:
			status = MissCount (cs8900_read (dev,PP_RxMISS));
			priv->stats.rx_errors += status;
			priv->stats.rx_missed_errors += status;
			break;
		}
	}
	return IRQ_RETVAL(handled);
}

static void cs8900_transmit_timeout (struct net_device *dev)
{
	cs8900_t *priv = (cs8900_t *) dev->priv;
	priv->stats.tx_errors++;
	priv->stats.tx_heartbeat_errors++;
	priv->txlen = 0;
	netif_wake_queue (dev);
}

static int cs8900_start (struct net_device *dev)
{
	int result;

#if defined(CONFIG_ARCH_SMDK2410)|defined(CONFIG_ARCH_QQ2440)
	set_irq_type(dev->irq, IRQT_RISING);

	/* enable the ethernet controller */
	cs8900_set (dev,PP_RxCFG,RxOKiE | BufferCRC | CRCerroriE | RuntiE | ExtradataiE);
	cs8900_set (dev,PP_RxCTL,RxOKA | IndividualA | BroadcastA);
	cs8900_set (dev,PP_TxCFG,TxOKiE | Out_of_windowiE | JabberiE);
	cs8900_set (dev,PP_BufCFG,Rdy4TxiE | RxMissiE | TxUnderruniE | TxColOvfiE | MissOvfloiE);
	cs8900_set (dev,PP_LineCTL,SerRxON | SerTxON);
	cs8900_set (dev,PP_BusCTL,EnableRQ);

#ifdef FULL_DUPLEX
	cs8900_set (dev,PP_TestCTL,FDX);
#endif	/* #ifdef FULL_DUPLEX */
	udelay(200);	
	/* install interrupt handler */
	if ((result = request_irq (dev->irq, &cs8900_interrupt, 0, dev->name, dev)) < 0) {
		printk ("%s: could not register interrupt %d\n",dev->name, dev->irq);
		return (result);
	}
#else
	
	/* install interrupt handler */
	if ((result = request_irq (dev->irq, &cs8900_interrupt, 0, dev->name, dev)) < 0) {
		printk ("%s: could not register interrupt %d\n",dev->name, dev->irq);
		return (result);
	}

	set_irq_type(dev->irq, IRQT_RISING);

	/* enable the ethernet controller */
	cs8900_set (dev,PP_RxCFG,RxOKiE | BufferCRC | CRCerroriE | RuntiE | ExtradataiE);
	cs8900_set (dev,PP_RxCTL,RxOKA | IndividualA | BroadcastA);
	cs8900_set (dev,PP_TxCFG,TxOKiE | Out_of_windowiE | JabberiE);
	cs8900_set (dev,PP_BufCFG,Rdy4TxiE | RxMissiE | TxUnderruniE | TxColOvfiE | MissOvfloiE);
	cs8900_set (dev,PP_LineCTL,SerRxON | SerTxON);
	cs8900_set (dev,PP_BusCTL,EnableRQ);

#ifdef FULL_DUPLEX
	cs8900_set (dev,PP_TestCTL,FDX);
#endif	/* #ifdef FULL_DUPLEX */
	
#endif /* #if defined(CONFIG_ARCH_SMDK2410) */

	/* start the queue */
	netif_start_queue (dev);

	return (0);
}

static int cs8900_stop (struct net_device *dev)
{
	/* disable ethernet controller */
	cs8900_write (dev,PP_BusCTL,0);
	cs8900_write (dev,PP_TestCTL,0);
	cs8900_write (dev,PP_SelfCTL,0);
	cs8900_write (dev,PP_LineCTL,0);
	cs8900_write (dev,PP_BufCFG,0);
	cs8900_write (dev,PP_TxCFG,0);
	cs8900_write (dev,PP_RxCTL,0);
	cs8900_write (dev,PP_RxCFG,0);

	/* uninstall interrupt handler */
	free_irq (dev->irq,dev);

	/* stop the queue */
	netif_stop_queue (dev);

	return (0);
}

                                                                                                 
static int cs8900_set_mac_address (struct net_device *dev, void *p)
{
        cs8900_t *priv = netdev_priv(dev);
        struct sockaddr *addr = (struct sockaddr *)p;
        int i;
                  
        if (netif_running(dev))
                return -EBUSY;
                                                                                                 
        spin_lock(&priv->lock);
                                                                                                 
        memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);
                                                                                                 
        /* configure MAC address */
        for (i = 0; i < ETH_ALEN; i += 2)
                cs8900_write (dev,PP_IA + i,dev->dev_addr[i] | (dev->dev_addr[i + 1] << 8));
                                                                                                 
        spin_unlock(&priv->lock);
                                                                                                 
        return 0;
}

static struct net_device_stats *cs8900_get_stats (struct net_device *dev)
{
	cs8900_t *priv = (cs8900_t *) dev->priv;
	return (&priv->stats);
}

static void cs8900_set_receive_mode (struct net_device *dev)
{
	if ((dev->flags & IFF_PROMISC))
		cs8900_set (dev,PP_RxCTL,PromiscuousA);
	else
		cs8900_clear (dev,PP_RxCTL,PromiscuousA);

	if ((dev->flags & IFF_ALLMULTI) && dev->mc_list)
		cs8900_set (dev,PP_RxCTL,MulticastA);
	else
		cs8900_clear (dev,PP_RxCTL,MulticastA);
}

static int cs8900_eeprom (struct net_device *dev)
{
	cs8900_t *priv = (cs8900_t *) dev->priv;
	int i;

	/* SMDK2410 CS8900A without EEPROM at all */
#if defined(CONFIG_ARCH_SMDK2410)|defined(CONFIG_ARCH_QQ2440)
	return (-ENODEV);
#endif 

#ifdef DEBUG
	eepromdump (dev);
#endif

	if( (cs8900_read( dev, PP_SelfST) & EEPROMpresent) == 0)
	{
		/* no eeprom */
		return (-ENODEV);
	}

	/* add character device for easy eeprom programming */
	if( (priv->char_devnum=register_chrdev(0,"cs8900_eeprom",&cs8900_eeprom_fops)) != 0)
		printk (KERN_INFO "%s: Registered cs8900_eeprom char device (major #%d)\n",
			dev->name, priv->char_devnum);
	else
		printk (KERN_WARNING "%s: Failed to register char device cs8900_eeprom\n",dev->name);

	if( (cs8900_read( dev, PP_SelfST) & EEPROMOK) == 0) 
	{
		/* bad checksum, invalid config block */
		return (-EFAULT);
	}

	/* If we get here, the chip will have initialized the registers
	 * that were specified in the eeprom configuration block
	 * We assume this is at least the mac address.
	 */
	for (i = 0; i < ETH_ALEN; i += 2)
	{
		u16 mac = cs8900_read (dev,PP_IA + i);
		dev->dev_addr[i] = mac & 0xff;
		dev->dev_addr[i+1] = (mac>>8) & 0xff;
	}

	return (0);
}

/*
 * EEPROM Charater device
 */

static int cs8900_eeprom_fopen(struct inode *inode, struct file *file)
{
	u16 i;
	for( i=0; i<MAX_EEPROM_SIZE/2; i++)
	{
		cs8900_eeprom_read( &cs8900_dev, &cs8900_eeprom_cache[i],i);
	}

	return 0;
}

static int cs8900_eeprom_frelease(struct inode *inode, struct file *file)
{
	return 0;
}

static loff_t cs8900_eeprom_fllseek(struct file * file,loff_t offset, int whence)
{
	long newpos;

	switch(whence)
	{
		case 0: /* SEEK_SET */
			newpos = offset;
			break;
		case 1: /* SEEK_CUR */
			newpos = file->f_pos + offset;
			break;
		case 2: /* SEEK_END */
			newpos = (MAX_EEPROM_SIZE-1) - offset;
			break;
		default: /* can't happen */
			return -EINVAL;

	}

	if( (newpos<0) || (newpos>=MAX_EEPROM_SIZE)) return -EINVAL;

	file->f_pos = newpos;
	return newpos;
}

static ssize_t cs8900_eeprom_fread(struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	unsigned char *temp = (unsigned char *)cs8900_eeprom_cache;

        if (*f_pos >= MAX_EEPROM_SIZE)
                return 0;

        if (*f_pos + count > MAX_EEPROM_SIZE)
            count = MAX_EEPROM_SIZE - *f_pos;

        if (count<1)
                return 0;

        if (copy_to_user(buf, &temp[*f_pos], count)){
                return -EFAULT;
        }
        *f_pos += count;
        return count;
}

static ssize_t cs8900_eeprom_fwrite(struct file *file, const char *buf, size_t count, loff_t *f_pos)
{
	u16 i;
	unsigned char *temp = (unsigned char *)cs8900_eeprom_cache;

        if (*f_pos >= MAX_EEPROM_SIZE)
                return 0;

        if (*f_pos + count > MAX_EEPROM_SIZE)
            count = MAX_EEPROM_SIZE - *f_pos;

        if (count<1)
                return 0;

	/* FIXME: lock critical section */

	/* update the cache */
        if (copy_from_user(&temp[*f_pos], buf, count)){
                return -EFAULT;
        }

	/* not concerned about performance, so write the entire thing */
	for( i=0; i<MAX_EEPROM_SIZE/2; i++)
	{
		cs8900_eeprom_write( &cs8900_dev, &cs8900_eeprom_cache[i],i);
	}

        *f_pos += count;
        return count;
}

/*
 * Architecture dependant code
 */

#ifdef CONFIG_SA1100_FRODO
static void frodo_reset (struct net_device *dev)
{
	int i;
	volatile u16 value;

	/* reset ethernet controller */
	FRODO_CPLD_ETHERNET |= FRODO_ETH_RESET;
	mdelay (50);
	FRODO_CPLD_ETHERNET &= ~FRODO_ETH_RESET;
	mdelay (50);

	/* we tied SBHE to CHIPSEL, so each memory access ensure the chip is in 16-bit mode */
	for (i = 0; i < 3; i++) value = cs8900_read (dev,0);

	/* FIXME: poll status bit */
}
#endif	/* #ifdef CONFIG_SA1100_FRODO */

/*
 * Driver initialization routines
 */
void cirrus_parse_mac (const char *macstr, char *mac)
{
        int i;
        if (strlen(macstr) != 17)
                printk("invalid MAC string format\n");
        for (i = 0; i < 6; i++) {
                mac[i] = simple_strtoul(macstr + i * 3, NULL, 16);
        }
}

int __init cs8900_probe (struct net_device *dev)
{
	static cs8900_t priv;
	int i,result;
	u16 value;

	printk (VERSION_STRING"\n");

	memset (&priv,0,sizeof (cs8900_t));

	__raw_writel(0x2211d110,S3C2410_BWSCON);
	__raw_writel(0x1f7c,S3C2410_BANKCON3);
	ether_setup (dev);

	dev->open               = cs8900_start;
	dev->stop               = cs8900_stop;
	dev->hard_start_xmit    = cs8900_send_start;
	dev->get_stats          = cs8900_get_stats;
	dev->set_multicast_list = cs8900_set_receive_mode;
	dev->set_mac_address    = cs8900_set_mac_address;
	dev->tx_timeout         = cs8900_transmit_timeout;
	dev->watchdog_timeo     = HZ;

#if defined(CONFIG_ARCH_SMDK2410)|defined(CONFIG_ARCH_QQ2440)
	//dev->dev_addr[0] = 0x08;
	//dev->dev_addr[1] = 0x00;
	//dev->dev_addr[2] = 0x3e;
	//dev->dev_addr[3] = 0x26;
	//dev->dev_addr[4] = 0x0a;
	//dev->dev_addr[5] = 0x5c;
	cirrus_parse_mac(cirrus_mac, dev->dev_addr);
#else
	dev->dev_addr[0] = 0x00;
    dev->dev_addr[1] = 0x12;
    dev->dev_addr[2] = 0x34;
    dev->dev_addr[3] = 0x56;
    dev->dev_addr[4] = 0x78;
    dev->dev_addr[5] = 0x9a;
#endif

	dev->if_port   = IF_PORT_10BASET;
	dev->priv      = (void *) &priv;

	spin_lock_init(&priv.lock);

	SET_MODULE_OWNER (dev);

#ifdef CONFIG_SA1100_FRODO
	dev->base_addr = FRODO_ETH_IO + 0x300;
	dev->irq = FRODO_ETH_IRQ;
	frodo_reset (dev);
#endif	/* #ifdef CONFIG_SA1100_FRODO */

#if defined(CONFIG_SA1100_CERF)
	dev->base_addr = CERF_ETH_IO + 0x300;
	dev->irq = CERF_ETH_IRQ;
#endif /* #if defined(CONFIG_SA1100_CERF) */

#if defined(CONFIG_ARCH_SMDK2410)|defined(CONFIG_ARCH_QQ2440)
	dev->base_addr = vSMDK2410_ETH_IO + 0x300;
	dev->irq = SMDK2410_ETH_IRQ;
#endif /* #if defined(CONFIG_ARCH_SMDK2410) */ 

	if ((result = check_mem_region (dev->base_addr, 16))) {
		printk (KERN_ERR "%s: can't get I/O port address 0x%lx\n",dev->name,dev->base_addr);
		return (result);
	}
	request_mem_region (dev->base_addr, 16, dev->name);
	
	/* verify EISA registration number for Cirrus Logic */
	if ((value = cs8900_read (dev,PP_ProductID)) != EISA_REG_CODE) {
		printk (KERN_ERR "%s: incorrect signature 0x%.4x\n",dev->name,value);
		return (-ENXIO);
	}

	/* verify chip version */
	value = cs8900_read (dev,PP_ProductID + 2);
	if (VERSION (value) != CS8900A) {
		printk (KERN_ERR "%s: unknown chip version 0x%.8x\n",dev->name,VERSION (value));
		return (-ENXIO);
	}
	/* setup interrupt number */
	cs8900_write (dev,PP_IntNum,0);


	/* If an EEPROM is present, use it's MAC address. A valid EEPROM will 
	 * initialize the registers automatically.
	 */
	result = cs8900_eeprom (dev);

	printk (KERN_INFO "%s: CS8900A rev %c at %#lx irq=%d",
		dev->name,'B' + REVISION (value) - REV_B, dev->base_addr, dev->irq);
	if (result == -ENODEV) {
		/* no eeprom or invalid config block, configure MAC address by hand */
		for (i = 0; i < ETH_ALEN; i += 2)
			cs8900_write (dev,PP_IA + i,dev->dev_addr[i] | (dev->dev_addr[i + 1] << 8));
		printk (", no eeprom ");
	}
	else if( result == -EFAULT)
	{
#if defined(CONFIG_SA1100_CERF)
	    /* The default eeprom layout doesn't follow the cs8900 layout 
		 * that enables automatic cs8900 initialization. Doh!
		 * Read the mac address manually.
		 */
		u16 MAC_addr[3] = {0, 0, 0};

		if (cs8900_eeprom_read(dev, &MAC_addr[0], 0x1c) == -1)
			printk("\ncs8900: [CERF] EEPROM[0] read failed\n");
		if (cs8900_eeprom_read(dev, &MAC_addr[1], 0x1d) == -1)
			printk("\ncs8900: [CERF] EEPROM[1] read failed\n");
		if (cs8900_eeprom_read(dev, &MAC_addr[2], 0x1e) == -1)
			printk("\ncs8900: [CERF] EEPROM[2] read failed\n");

		for (i = 0; i < ETH_ALEN / 2; i++)
		{
			dev->dev_addr[i*2]	= MAC_addr[i] & 0xff;
			dev->dev_addr[i*2+1]	= (MAC_addr[i] >> 8) & 0xff;

			cs8900_write (dev,PP_IA + i*2,dev->dev_addr[i*2] | (dev->dev_addr[i*2 + 1] << 8));
		}
		printk (", eeprom (smdk2410 layout)");
#else
		printk (", eeprom (invalid config block)");
#endif /* #if defined(CONFIG_SA1100_CERF) */
	}
	else
	{
		printk (", eeprom ok");
	}

	printk (", addr:");
	for (i = 0; i < ETH_ALEN; i += 2)
	{
		u16 mac = cs8900_read (dev,PP_IA + i);
		printk ("%c%02X:%2X", (i==0)?' ':':', mac & 0xff, (mac >> 8));
	}
	printk ("\n");

        /* configure MAC address */
        for (i = 0; i < ETH_ALEN; i += 2)
                cs8900_write (dev,PP_IA + i,dev->dev_addr[i] | (dev->dev_addr[i + 1] << 8));
	return (0);
}

static int __init cs8900_init (void)
{
	strcpy(cs8900_dev.name, "eth%d");

	return (register_netdev (&cs8900_dev));
}

static void __exit cs8900_cleanup (void)
{
	cs8900_t *priv = (cs8900_t *) cs8900_dev.priv;
	if( priv->char_devnum)
	{
		unregister_chrdev(priv->char_devnum,"cs8900_eeprom");
	}
	release_mem_region (cs8900_dev.base_addr,16);
	unregister_netdev (&cs8900_dev);
}

MODULE_AUTHOR ("Abraham van der Merwe <abraham at 2d3d.co.za>");
MODULE_DESCRIPTION (VERSION_STRING);
MODULE_LICENSE ("GPL");

module_init (cs8900_init);
module_exit (cs8900_cleanup);
