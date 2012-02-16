/*
 * linux/drivers/usb/gadget/s3c2410_udc.c
 * Samsung on-chip full speed USB device controllers
 *
 * Copyright (C) 2004 Herbert PÃ¶tzl - Arnaud Patard
 *
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *   2005-12: godori(www.aesop-embedded.org) <ghcstop@gmail.com>
 *			=> port to S3C2440A, bug fix for s3c2410
 *
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/version.h>

#include <linux/usb.h>
#include <linux/usb_gadget.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <asm/arch/irqs.h>

#include <asm/arch/hardware.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-udc.h>
#include <asm/arch/udc.h>
#include <asm/hardware/clock.h>

#include "s3c2410_udc.h"


//#define GDEBUG // ghcstop
#ifdef  GDEBUG
#    define gprintk( x... )  printk( x )
#else
#    define gprintk( x... )
#endif

#define ENABLE_SYSFS

#define DRIVER_DESC     "S3C2410 USB Device Controller Gadget"
#define DRIVER_VERSION  "28 Aug 2005"
#define DRIVER_AUTHOR	"Herbert PÃ¶tzl <herbert@13thfloor.at>, Arnaud Patard <arnaud.patard@rtp-net.org>"

static const char gadget_name[] = "s3c2410_udc";
static const char driver_desc[] = DRIVER_DESC;

static struct s3c2410_udc *the_controller;
static struct clk *udc_clock;
static void __iomem *base_addr;
static u64      rsrc_start;
static u64      rsrc_len;

static inline u32
udc_readl(u32 reg)
{
    return readl(base_addr + reg);
}
static inline void
udc_writel(u32 value, u32 reg)
{
    writel(value, base_addr + reg);
}

static struct s3c2410_udc_mach_info *udc_info;

/*************************** DEBUG FUNCTION ***************************/
#define DEBUG_NORMAL	1
#define DEBUG_VERBOSE	2

#ifdef CONFIG_USB_S3C2410_DEBUG
#    define USB_S3C2410_DEBUG_LEVEL 1
uint32_t        s3c2410_ticks = 0;
static int
dprintk(int level, const char *fmt, ...)
{
    static char     printk_buf[1024];
    static long     prevticks;
    static int      invocation;
    va_list         args;
    int             len;

    if (level > USB_S3C2410_DEBUG_LEVEL)
        return 0;

    if (s3c2410_ticks != prevticks)
    {
        prevticks = s3c2410_ticks;
        invocation = 0;
    }

    len = scnprintf(printk_buf, sizeof(printk_buf), "%1lu.%02d USB: ", prevticks, invocation++);

    va_start(args, fmt);
    len = vscnprintf(printk_buf + len, sizeof(printk_buf) - len, fmt, args);
    va_end(args);

    return printk("%s", printk_buf);
}
#else
static int
dprintk(int level, const char *fmt, ...)
{
    return 0;
}
#endif
#ifdef ENABLE_SYSFS
static ssize_t
s3c2410udc_regs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    u32             addr_reg,
                    pwr_reg,
                    ep_int_reg,
                    usb_int_reg;
    u32             ep_int_en_reg,
                    usb_int_en_reg,
                    ep0_csr;
    u32             ep1_i_csr1,
                    ep1_i_csr2,
                    ep1_o_csr1,
                    ep1_o_csr2;
    u32             ep2_i_csr1,
                    ep2_i_csr2,
                    ep2_o_csr1,
                    ep2_o_csr2;

    addr_reg = udc_readl(S3C2410_UDC_FUNC_ADDR_REG);
    pwr_reg = udc_readl(S3C2410_UDC_PWR_REG);
    ep_int_reg = udc_readl(S3C2410_UDC_EP_INT_REG);
    usb_int_reg = udc_readl(S3C2410_UDC_USB_INT_REG);
    ep_int_en_reg = udc_readl(S3C2410_UDC_EP_INT_EN_REG);
    usb_int_en_reg = udc_readl(S3C2410_UDC_USB_INT_EN_REG);
    udc_writel(0, S3C2410_UDC_INDEX_REG);
    ep0_csr = udc_readl(S3C2410_UDC_IN_CSR1_REG);
    udc_writel(1, S3C2410_UDC_INDEX_REG);
    ep1_i_csr1 = udc_readl(S3C2410_UDC_IN_CSR1_REG);
    ep1_i_csr2 = udc_readl(S3C2410_UDC_IN_CSR2_REG);
    ep1_o_csr1 = udc_readl(S3C2410_UDC_IN_CSR1_REG);
    ep1_o_csr2 = udc_readl(S3C2410_UDC_IN_CSR2_REG);
    udc_writel(2, S3C2410_UDC_INDEX_REG);
    ep2_i_csr1 = udc_readl(S3C2410_UDC_IN_CSR1_REG);
    ep2_i_csr2 = udc_readl(S3C2410_UDC_IN_CSR2_REG);
    ep2_o_csr1 = udc_readl(S3C2410_UDC_IN_CSR1_REG);
    ep2_o_csr2 = udc_readl(S3C2410_UDC_IN_CSR2_REG);


    return snprintf(buf, PAGE_SIZE,
                    "FUNC_ADDR_REG  : 0x%04X\n"
                    "PWR_REG        : 0x%04X\n"
                    "EP_INT_REG     : 0x%04X\n"
                    "USB_INT_REG    : 0x%04X\n"
                    "EP_INT_EN_REG  : 0x%04X\n"
                    "USB_INT_EN_REG : 0x%04X\n"
                    "EP0_CSR        : 0x%04X\n"
                    "EP1_I_CSR1     : 0x%04X\n"
                    "EP1_I_CSR2     : 0x%04X\n"
                    "EP1_O_CSR1     : 0x%04X\n"
                    "EP1_O_CSR2     : 0x%04X\n"
                    "EP2_I_CSR1     : 0x%04X\n"
                    "EP2_I_CSR2     : 0x%04X\n"
                    "EP2_O_CSR1     : 0x%04X\n"
                    "EP2_O_CSR2     : 0x%04X\n",
                    addr_reg, pwr_reg, ep_int_reg, usb_int_reg, ep_int_en_reg, usb_int_en_reg, ep0_csr, ep1_i_csr1, ep1_i_csr2, ep1_o_csr1, ep1_o_csr2, ep2_i_csr1, ep2_i_csr2, ep2_o_csr1, ep2_o_csr2);
}

static          DEVICE_ATTR(regs, 0444, s3c2410udc_regs_show, NULL);
#endif
/*------------------------- I/O ----------------------------------*/
static void
nuke(struct s3c2410_udc *udc, struct s3c2410_ep *ep)
{
    /*
     * Sanity check 
     */
    if (&ep->queue != NULL)
        while (!list_empty(&ep->queue))
        {
            struct s3c2410_request *req;
            req = list_entry(ep->queue.next, struct s3c2410_request, queue);

            list_del_init(&req->queue);
            req->req.status = -ESHUTDOWN;
            req->req.complete(&ep->ep, &req->req);
        }
}

/*
 * 	done
 */
static void
done(struct s3c2410_ep *ep, struct s3c2410_request *req, int status)
{
    list_del_init(&req->queue);

    if (likely(req->req.status == -EINPROGRESS))
        req->req.status = status;
    else
        status = req->req.status;

    req->req.complete(&ep->ep, &req->req);
}

static inline void
clear_ep_state(struct s3c2410_udc *dev)
{
    unsigned        i;

    /*
     * hardware SET_{CONFIGURATION,INTERFACE} automagic resets endpoint fifos, and pending transactions mustn't be continued in any case. 
     */
    for (i = 1; i < S3C2410_ENDPOINTS; i++)
        nuke(dev, &dev->ep[i]);
}

static inline int
fifo_count_out(void)
{
    int             tmp;

    tmp = udc_readl(S3C2410_UDC_OUT_FIFO_CNT2_REG) << 8;
    tmp |= udc_readl(S3C2410_UDC_OUT_FIFO_CNT1_REG);

    return tmp & 0xffff;
}

/*
 * 	write_packet
 */
static inline int
write_packet(int fifo, struct s3c2410_request *req, unsigned max)
{
    unsigned        len;
    u8             *buf;

    buf = req->req.buf + req->req.actual;
    len = min(req->req.length - req->req.actual, max);
    dprintk(DEBUG_VERBOSE, "write_packet %d %d %d ", req->req.actual, req->req.length, len);
    req->req.actual += len;
    dprintk(DEBUG_VERBOSE, "%d\n", req->req.actual);

    max = len;
    while (max--)
        udc_writel(*buf++, fifo);
    return len;
}

/*
 * 	write_fifo
 */
// return:  0 = still running, 1 = completed, negative = errno
static int
write_fifo(struct s3c2410_ep *ep, struct s3c2410_request *req)
{
    u8             *buf;
    unsigned        count;
    int             is_last;
    u32             idx;
    int             fifo_reg;
    u32             ep_csr;


    switch (ep->bEndpointAddress & 0x7F)
    {
    default:
    case 0:
        idx = 0;
        fifo_reg = S3C2410_UDC_EP0_FIFO_REG;
        break;
    case 1:
        idx = 1;
        fifo_reg = S3C2410_UDC_EP1_FIFO_REG;
        break;
    case 2:
        idx = 2;
        fifo_reg = S3C2410_UDC_EP2_FIFO_REG;
        break;

    case 3:
        idx = 3;
        fifo_reg = S3C2410_UDC_EP3_FIFO_REG;
        break;

    case 4:
        idx = 4;
        fifo_reg = S3C2410_UDC_EP4_FIFO_REG;
        break;
    }

    buf = req->req.buf + req->req.actual;
    prefetch(buf);

    count = ep->ep.maxpacket;
    count = write_packet(fifo_reg, req, count);

    /*
     * last packet is often short (sometimes a zlp) 
     */
    if (count != ep->ep.maxpacket)
        is_last = 1;
    else if (req->req.length == req->req.actual && !req->req.zero)
        is_last = 2;
    else
        is_last = 0;

    /*
     * Only ep0 debug messages are interesting 
     */
    if (!idx)
        dprintk(DEBUG_NORMAL, "Written ep%d %d.%d of %d b [last %d,z %d]\n", idx, count, req->req.actual, req->req.length, is_last, req->req.zero);

    if (is_last)
    {
        /*
         * The order is important. It prevents to send 2 packet at the same time 
         */
        if (!idx)
        {
            /*
             * If we got a reset signal, no need to say 'data sent' 
             */
            if (!(udc_readl(S3C2410_UDC_USB_INT_REG) & S3C2410_UDC_USBINT_RESET))
                set_ep0_de_in(base_addr);
            ep->dev->ep0state = EP0_IDLE;
        }
        else
        {
            udc_writel(idx, S3C2410_UDC_INDEX_REG);
            ep_csr = udc_readl(S3C2410_UDC_IN_CSR1_REG);
            udc_writel(idx, S3C2410_UDC_INDEX_REG);
            udc_writel(ep_csr | S3C2410_UDC_ICSR1_PKTRDY, S3C2410_UDC_IN_CSR1_REG);
        }
        done(ep, req, 0);
        if (!list_empty(&ep->queue))
        {
            is_last = 0;
            req = container_of(ep->queue.next, struct s3c2410_request, queue);
        }
        else
            is_last = 1;
    }
    else
    {
        if (!idx)
        {
            /*
             * If we got a reset signal, no need to say 'data sent' 
             */
            if (!(udc_readl(S3C2410_UDC_USB_INT_REG) & S3C2410_UDC_USBINT_RESET))
                set_ep0_ipr(base_addr);
        }
        else
        {
            udc_writel(idx, S3C2410_UDC_INDEX_REG);
            ep_csr = udc_readl(S3C2410_UDC_IN_CSR1_REG);
            udc_writel(idx, S3C2410_UDC_INDEX_REG);
            udc_writel(ep_csr | S3C2410_UDC_ICSR1_PKTRDY, S3C2410_UDC_IN_CSR1_REG);
        }
    }


    return is_last;
}

static inline int
read_packet(int fifo, u8 * buf, struct s3c2410_request *req, unsigned avail)
{
    unsigned        len;

    len = min(req->req.length - req->req.actual, avail);
    req->req.actual += len;
    avail = len;

    while (avail--)
        *buf++ = (unsigned char) udc_readl(fifo);
    return len;
}

// return:  0 = still running, 1 = queue empty, negative = errno
static int
read_fifo(struct s3c2410_ep *ep, struct s3c2410_request *req)
{
    u8             *buf;
    u32             ep_csr;
    unsigned        bufferspace;
    int             is_last = 1;
    unsigned        avail;
    int             fifo_count = 0;
    u32             idx;
    int             fifo_reg;


    switch (ep->bEndpointAddress & 0x7F)
    {
    default:
    case 0:
        idx = 0;
        fifo_reg = S3C2410_UDC_EP0_FIFO_REG;
        break;
    case 1:
        idx = 1;
        fifo_reg = S3C2410_UDC_EP1_FIFO_REG;
        break;
    case 2:
        idx = 2;
        fifo_reg = S3C2410_UDC_EP2_FIFO_REG;
        break;

    case 3:
        idx = 3;
        fifo_reg = S3C2410_UDC_EP3_FIFO_REG;
        break;

    case 4:
        idx = 4;
        fifo_reg = S3C2410_UDC_EP4_FIFO_REG;
        break;

    }


    buf = req->req.buf + req->req.actual;
    bufferspace = req->req.length - req->req.actual;
    if (!bufferspace)
    {
        dprintk(DEBUG_NORMAL, "read_fifo: Buffer full !!\n");
        return -1;
    }

    udc_writel(idx, S3C2410_UDC_INDEX_REG);

    fifo_count = fifo_count_out();
    dprintk(DEBUG_VERBOSE, "fifo_read fifo count : %d\n", fifo_count);

    if (fifo_count > ep->ep.maxpacket)
        avail = ep->ep.maxpacket;
    else
        avail = fifo_count;

    fifo_count = read_packet(fifo_reg, buf, req, avail);

    if (fifo_count < ep->ep.maxpacket)
    {
        is_last = 1;
        /*
         * overflowed this request? flush extra data 
         */
        if (fifo_count != avail)
        {
            req->req.status = -EOVERFLOW;
        }
    }
    else
    {
        if (req->req.length == req->req.actual)
            is_last = 1;
        else
            is_last = 0;
    }

    udc_writel(idx, S3C2410_UDC_INDEX_REG);
    fifo_count = fifo_count_out();

    /*
     * Only ep0 debug messages are interesting 
     */
    if (!idx)
        dprintk(DEBUG_VERBOSE, "fifo_read fifo count : %d [last %d]\n", fifo_count, is_last);


    if (is_last)
    {
        if (!idx)
        {
            set_ep0_de_out(base_addr);
            ep->dev->ep0state = EP0_IDLE;
        }
        else
        {
            udc_writel(idx, S3C2410_UDC_INDEX_REG);
            ep_csr = udc_readl(S3C2410_UDC_OUT_CSR1_REG);
            udc_writel(idx, S3C2410_UDC_INDEX_REG);
            udc_writel(ep_csr & ~S3C2410_UDC_OCSR1_PKTRDY, S3C2410_UDC_OUT_CSR1_REG);
        }
        done(ep, req, 0);
        if (!list_empty(&ep->queue))
        {
            is_last = 0;
            req = container_of(ep->queue.next, struct s3c2410_request, queue);
        }
        else
            is_last = 1;

    }
    else
    {
        if (!idx)
        {
            clear_ep0_opr(base_addr);
        }
        else
        {
            udc_writel(idx, S3C2410_UDC_INDEX_REG);
            ep_csr = udc_readl(S3C2410_UDC_OUT_CSR1_REG);
            udc_writel(idx, S3C2410_UDC_INDEX_REG);
            udc_writel(ep_csr & ~S3C2410_UDC_OCSR1_PKTRDY, S3C2410_UDC_OUT_CSR1_REG);
        }
    }


    return is_last;
}


static int
read_fifo_crq(struct usb_ctrlrequest *crq)
{
    int             bytes_read = 0;
    int             fifo_count = 0;
    int             i;


    unsigned char  *pOut = (unsigned char *) crq;

    udc_writel(0, S3C2410_UDC_INDEX_REG);

    fifo_count = fifo_count_out();

    BUG_ON(fifo_count > 8);

    dprintk(DEBUG_VERBOSE, "read_fifo_crq(): fifo_count=%d\n", fifo_count);
    while (fifo_count--)
    {
        i = 0;

        do
        {
            *pOut = (unsigned char) udc_readl(S3C2410_UDC_EP0_FIFO_REG);
            i++;
        }
        while ((fifo_count_out() != fifo_count) && (i < 10));

        if (i == 10)
        {
            dprintk(DEBUG_NORMAL, "read_fifo(): read failure\n");
        }

        pOut++;
        bytes_read++;
    }

    dprintk(DEBUG_VERBOSE, "read_fifo_crq: len=%d %02x:%02x {%x,%x,%x}\n", bytes_read, crq->bRequest, crq->bRequestType, crq->wValue, crq->wIndex, crq->wLength);

    return bytes_read;
}

/*------------------------- usb state machine -------------------------------*/
static void
handle_ep0(struct s3c2410_udc *dev)
{
    u32             ep0csr;
    struct s3c2410_ep *ep = &dev->ep[0];
    struct s3c2410_request *req;
    struct usb_ctrlrequest crq;

    if (list_empty(&ep->queue))
        req = 0;
    else
        req = list_entry(ep->queue.next, struct s3c2410_request, queue);


    udc_writel(0, S3C2410_UDC_INDEX_REG);
    ep0csr = udc_readl(S3C2410_UDC_IN_CSR1_REG);
    dprintk(DEBUG_NORMAL, "ep0csr %x\n", ep0csr);

    /*
     * clear stall status 
     */
    if (ep0csr & S3C2410_UDC_EP0_CSR_SENTSTL)
    {
        /*
         * FIXME 
         */
        nuke(dev, ep);
        dprintk(DEBUG_NORMAL, "... clear SENT_STALL ...\n");
        clear_ep0_sst(base_addr);
        dev->ep0state = EP0_IDLE;
        return;
    }

    /*
     * clear setup end 
     */
    if (ep0csr & S3C2410_UDC_EP0_CSR_SE
        /*
         * && dev->ep0state != EP0_IDLE 
         */ )
    {
        dprintk(DEBUG_NORMAL, "... serviced SETUP_END ...\n");
        nuke(dev, ep);
        clear_ep0_se(base_addr);
        dev->ep0state = EP0_IDLE;
    }


    switch (dev->ep0state)
    {
    case EP0_IDLE:
        /*
         * start control request? 
         */
        if (ep0csr & S3C2410_UDC_EP0_CSR_OPKRDY)
        {
            int             len,
                            ret,
                            tmp;

            nuke(dev, ep);

            len = read_fifo_crq(&crq);
            if (len != sizeof(crq))
            {
                dprintk(DEBUG_NORMAL, "setup begin: fifo READ ERROR" " wanted %d bytes got %d. Stalling out...\n", sizeof(crq), len);
                set_ep0_ss(base_addr);
                return;
            }

            dprintk(DEBUG_NORMAL, "bRequest = %d wLength = %d\n", crq.bRequest, crq.wLength);

            /*
             * cope with automagic for some standard requests. 
             */
            dev->req_std = (crq.bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD;
            dev->req_config = 0;
            dev->req_pending = 1;
            switch (crq.bRequest)
            {
                /*
                 * hardware restricts gadget drivers here! 
                 */
            case USB_REQ_SET_CONFIGURATION:
                dprintk(DEBUG_NORMAL, "USB_REQ_SET_CONFIGURATION ... \n");
                if (crq.bRequestType == USB_RECIP_DEVICE)
                {
                  config_change:
                    dev->req_config = 1;
                    clear_ep_state(dev);
                    set_ep0_de_out(base_addr);
                }
                break;
                /*
                 * ... and here, even more ... 
                 */
            case USB_REQ_SET_INTERFACE:
                dprintk(DEBUG_NORMAL, "USB_REQ_SET_INTERFACE ... \n");
                if (crq.bRequestType == USB_RECIP_INTERFACE)
                {
                    goto config_change;
                }
                break;

                /*
                 * hardware was supposed to hide this 
                 */
            case USB_REQ_SET_ADDRESS:
                dprintk(DEBUG_NORMAL, "USB_REQ_SET_ADDRESS ... \n");
                if (crq.bRequestType == USB_RECIP_DEVICE)
                {
                    tmp = crq.wValue & 0x7F;
                    dev->address = tmp;
                    udc_writel((tmp | 0x80), S3C2410_UDC_FUNC_ADDR_REG);
                    set_ep0_de_out(base_addr);
                    return;
                }
                break;
            default:
                clear_ep0_opr(base_addr);
                break;
            }

            if (crq.bRequestType & USB_DIR_IN)
                dev->ep0state = EP0_IN_DATA_PHASE;
            else
                dev->ep0state = EP0_OUT_DATA_PHASE;
            ret = dev->driver->setup(&dev->gadget, &crq);
            if (ret < 0)
            {
                if (dev->req_config)
                {
                    dprintk(DEBUG_NORMAL, "config change %02x fail %d?\n", crq.bRequest, ret);
                    return;
                }
                if (ret == -EOPNOTSUPP)
                    dprintk(DEBUG_NORMAL, "Operation not supported\n");
                else
                    dprintk(DEBUG_NORMAL, "dev->driver->setup failed. (%d)\n", ret);

                set_ep0_ss(base_addr);
                set_ep0_de_out(base_addr);
                dev->ep0state = EP0_IDLE;
                /*
                 * deferred i/o == no response yet 
                 */
            }
            else if (dev->req_pending)
            {
                dprintk(DEBUG_VERBOSE, "dev->req_pending... what now?\n");
                dev->req_pending = 0;
            }
            dprintk(DEBUG_VERBOSE, "ep0state %s\n", ep0states[dev->ep0state]);
        }
        break;
    case EP0_IN_DATA_PHASE:    /* GET_DESCRIPTOR etc */
        dprintk(DEBUG_NORMAL, "EP0_IN_DATA_PHASE ... what now?\n");
        if (!(ep0csr & 2) && req)
        {
            write_fifo(ep, req);
        }
        break;
    case EP0_OUT_DATA_PHASE:   /* SET_DESCRIPTOR etc */
        dprintk(DEBUG_NORMAL, "EP0_OUT_DATA_PHASE ... what now?\n");
        if ((ep0csr & 1) && req)
        {
            read_fifo(ep, req);
        }
        break;
    case EP0_END_XFER:
        dprintk(DEBUG_NORMAL, "EP0_END_XFER ... what now?\n");
        dev->ep0state = EP0_IDLE;
        break;
    case EP0_STALL:
        dev->ep0state = EP0_IDLE;
        break;
    }
}

/*
 * 	handle_ep - Manage I/O endpoints
 */
static void
handle_ep(struct s3c2410_ep *ep)
{
    struct s3c2410_request *req;
    int             is_in = ep->bEndpointAddress & USB_DIR_IN;
    u32             ep_csr1;
    u32             idx;

    if (likely(!list_empty(&ep->queue)))
        req = list_entry(ep->queue.next, struct s3c2410_request, queue);

    else
        req = 0;

    idx = (u32) (ep->bEndpointAddress & 0x7F);

    if (is_in)
    {
        udc_writel(idx, S3C2410_UDC_INDEX_REG);
        ep_csr1 = udc_readl(S3C2410_UDC_IN_CSR1_REG);
        dprintk(DEBUG_VERBOSE, "ep%01d write csr:%02x %d\n", idx, ep_csr1, req ? 1 : 0);

        if (ep_csr1 & S3C2410_UDC_ICSR1_SENTSTL)
        {
            dprintk(DEBUG_VERBOSE, "st\n");
            udc_writel(idx, S3C2410_UDC_INDEX_REG);
            udc_writel(0x00, S3C2410_UDC_IN_CSR1_REG);
            return;
        }

        if (!(ep_csr1 & S3C2410_UDC_ICSR1_PKTRDY) && req)
        {
            write_fifo(ep, req);
        }
    }
    else
    {
        udc_writel(idx, S3C2410_UDC_INDEX_REG);
        ep_csr1 = udc_readl(S3C2410_UDC_OUT_CSR1_REG);
        dprintk(DEBUG_VERBOSE, "ep%01d read csr:%02x\n", idx, ep_csr1);

        if (ep_csr1 & S3C2410_UDC_OCSR1_SENTSTL)
        {
            udc_writel(idx, S3C2410_UDC_INDEX_REG);
            udc_writel(0x00, S3C2410_UDC_OUT_CSR1_REG);
            return;
        }
        if ((ep_csr1 & S3C2410_UDC_OCSR1_PKTRDY) && req)
        {
            read_fifo(ep, req);
        }
    }
}

#include <asm/arch/regs-irq.h>
/*
 *      s3c2410_udc_irq - interrupt handler
 */
static          irqreturn_t
s3c2410_udc_irq(int irq, void *_dev, struct pt_regs *r)
{
    struct s3c2410_udc *dev = _dev;
    int             usb_status;
    int             usbd_status;
    int             pwr_reg;
    int             ep0csr;
    int             i;
    u32             idx;
    unsigned long   flags;


    /*
     * Driver connected ? 
     */
    if (!dev->driver)
    {
        /*
         * Clear interrupts 
         */
        udc_writel(udc_readl(S3C2410_UDC_USB_INT_REG), S3C2410_UDC_USB_INT_REG);
        udc_writel(udc_readl(S3C2410_UDC_EP_INT_REG), S3C2410_UDC_EP_INT_REG);
    }
    spin_lock_irqsave(&dev->lock, flags);

    /*
     * Save index 
     */
    idx = udc_readl(S3C2410_UDC_INDEX_REG);

    /*
     * Read status registers 
     */
    usb_status = udc_readl(S3C2410_UDC_USB_INT_REG);
    usbd_status = udc_readl(S3C2410_UDC_EP_INT_REG);
    pwr_reg = udc_readl(S3C2410_UDC_PWR_REG);

    S3C2410_UDC_SETIX(base_addr, EP0);
    ep0csr = udc_readl(S3C2410_UDC_IN_CSR1_REG);

    //dprintk(DEBUG_NORMAL, "usbs=%02x, usbds=%02x, pwr=%02x ep0csr=%02x\n", usb_status, usbd_status, pwr_reg,ep0csr);
   

    /*
     * Now, handle interrupts. There's two types :
     * - Reset, Resume, Suspend coming -> usb_int_reg
     * - EP -> ep_int_reg
     */

    /*
     * RESET 
     */
    if (usb_status & S3C2410_UDC_USBINT_RESET)
    {
        dprintk(DEBUG_NORMAL, "USB reset csr %x pwr %x\n", ep0csr, pwr_reg);

        udc_writel(0x00, S3C2410_UDC_INDEX_REG);
        udc_writel((dev->ep[0].ep.maxpacket & 0x7ff) >> 3, S3C2410_UDC_MAXP_REG);

        dev->gadget.speed = USB_SPEED_FULL;
        dev->ep0state = EP0_IDLE;
        nuke(dev, &dev->ep[0]);

        /*
         * clear interrupt 
         */
        udc_writel(S3C2410_UDC_USBINT_RESET, S3C2410_UDC_USB_INT_REG);

        udc_writel(idx, S3C2410_UDC_INDEX_REG);
        spin_unlock_irqrestore(&dev->lock, flags);
        return IRQ_HANDLED;
    }

    /*
     * RESUME 
     */
    if (usb_status & S3C2410_UDC_USBINT_RESUME)
    {
        dprintk(DEBUG_NORMAL, "USB resume\n");

        /*
         * clear interrupt 
         */
        udc_writel(S3C2410_UDC_USBINT_RESUME, S3C2410_UDC_USB_INT_REG);

        if (dev->gadget.speed != USB_SPEED_UNKNOWN && dev->driver && dev->driver->resume)
            dev->driver->resume(&dev->gadget);
    }

    /*
     * SUSPEND 
     */
    if (usb_status & S3C2410_UDC_USBINT_SUSPEND)
    {
        dprintk(DEBUG_NORMAL, "USB suspend\n");

        /*
         * clear interrupt 
         */
        udc_writel(S3C2410_UDC_USBINT_SUSPEND, S3C2410_UDC_USB_INT_REG);

        if (dev->gadget.speed != USB_SPEED_UNKNOWN && dev->driver && dev->driver->suspend)
            dev->driver->suspend(&dev->gadget);

        dev->ep0state = EP0_IDLE;
    }

    /*
     * EP 
     */
    /*
     * control traffic 
     */
    /*
     * check on ep0csr != 0 is not a good idea as clearing in_pkt_ready generate an interrupt 
     */
    if (usbd_status & S3C2410_UDC_INT_EP0)
    {
        dprintk(DEBUG_VERBOSE, "USB ep0 irq\n");
        /*
         * Clear the interrupt bit by setting it to 1 
         */
        udc_writel(S3C2410_UDC_INT_EP0, S3C2410_UDC_EP_INT_REG);
        handle_ep0(dev);
    }
    /*
     * endpoint data transfers 
     */
    for (i = 1; i < S3C2410_ENDPOINTS; i++)
    {
        u32             tmp = 1 << i;

        if (usbd_status & tmp)
        {
            dprintk(DEBUG_VERBOSE, "USB ep%d irq\n", i);

            /*
             * Clear the interrupt bit by setting it to 1 
             */
            udc_writel(tmp, S3C2410_UDC_EP_INT_REG);
            handle_ep(&dev->ep[i]);
        }
    }


    dprintk(DEBUG_VERBOSE, "irq: %d done.\n", irq);

    /*
     * Restore old index 
     */
    udc_writel(idx, S3C2410_UDC_INDEX_REG);

    spin_unlock_irqrestore(&dev->lock, flags);

    return IRQ_HANDLED;
}

/*------------------------- s3c2410_ep_ops ----------------------------------*/

/*
 * 	s3c2410_ep_enable
 */
static int
s3c2410_ep_enable(struct usb_ep *_ep, const struct usb_endpoint_descriptor *desc)
{
    struct s3c2410_udc *dev;
    struct s3c2410_ep *ep;
    u32             max,
                    tmp;
    unsigned long   flags;
    u32             csr1,
                    csr2;
    u32             int_en_reg;


    ep = container_of(_ep, struct s3c2410_ep, ep);

    if (!_ep || !desc || ep->desc || _ep->name == ep0name || desc->bDescriptorType != USB_DT_ENDPOINT)
        return -EINVAL;
    dev = ep->dev;
    if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
        return -ESHUTDOWN;

    max = le16_to_cpu(desc->wMaxPacketSize) & 0x1fff;

    spin_lock_irqsave(&dev->lock, flags);
    _ep->maxpacket = max & 0x7ff;
    ep->desc = desc;
    ep->bEndpointAddress = desc->bEndpointAddress;

    /*
     * set max packet 
     */
    udc_writel(ep->num, S3C2410_UDC_INDEX_REG);
    udc_writel(max >> 3, S3C2410_UDC_MAXP_REG);


    /*
     * set type, direction, address; reset fifo counters 
     */
    if (desc->bEndpointAddress & USB_DIR_IN)
    {
    	/* 2440 2.4.20 driver 
   		EP2: usb specÀÇ in mode·Î ¼¼ÆÃ 
   		
		UD_INDEX = UD_INDEX_EP2;           
		UD_MAXP = (EP2_FIFO_SIZE >> 3);
		
		UD_INDEX = UD_INDEX_EP2;
		UD_ICSR1 = UD_ICSR1_FFLUSH | UD_ICSR1_CLRDT; // fifo flush, data toggle
		
		UD_INDEX = UD_INDEX_EP2;
		UD_ICSR2 = UD_ICSR2_MODEIN | UD_ICSR2_DMAIEN; // input mode, IN_PKT_RDY dis 
		
		#ifdef USE_USBD_DMA
			UD_ICSR2 &= ~UD_ICSR2_DMAIEN;
		#endif
		*/
		
		/* fw code, ep1 bulk in mode
		
	    rINDEX_REG=1;
	    #if (EP1_PKT_SIZE==32)
    	    rMAXP_REG=FIFO_SIZE_32;	//EP1:max packit size = 32
    	#else
			rMAXP_REG=FIFO_SIZE_64;	//EP1:max packit size = 64
    	#endif	
    	rIN_CSR1_REG=EPI_FIFO_FLUSH|EPI_CDT;	
    	rIN_CSR2_REG=EPI_MODE_IN|EPI_IN_DMA_INT_MASK|EPI_BULK; //IN mode, IN_DMA_INT=masked    
    	rOUT_CSR1_REG=EPO_CDT;   	
    	rOUT_CSR2_REG=EPO_BULK|EPO_OUT_DMA_INT_MASK;   	
    	*/

gprintk("%s: in ep num: %d\n", __FUNCTION__, ep->num);

        csr1 = S3C2410_UDC_ICSR1_FFLUSH | S3C2410_UDC_ICSR1_CLRDT;
        csr2 = S3C2410_UDC_ICSR2_MODEIN | S3C2410_UDC_ICSR2_DMAIEN;

        udc_writel(ep->num, S3C2410_UDC_INDEX_REG);
        udc_writel(csr1, S3C2410_UDC_IN_CSR1_REG);
        udc_writel(ep->num, S3C2410_UDC_INDEX_REG);
        udc_writel(csr2, S3C2410_UDC_IN_CSR2_REG);
    }
    else
    {
    	/* 2440 2.4.20 driver 
    	EP1: usb specÀÇ out mode·Î ¼¼ÆÃ 
		UD_INDEX = UD_INDEX_EP1;            
		UD_MAXP = (EP1_FIFO_SIZE >> 3);
		
		UD_INDEX = UD_INDEX_EP1;
		UD_ICSR1 = UD_ICSR1_FFLUSH | UD_ICSR1_CLRDT; // fifo flush, data toggle
		
		UD_INDEX = UD_INDEX_EP1;
		UD_ICSR2 = 0x0; // output mode
		
		UD_INDEX = UD_INDEX_EP1;
		UD_OCSR1 = UD_OCSR1_FFLUSH | UD_OCSR1_CLRDT; // fifo flush
		
		UD_INDEX = UD_INDEX_EP1;
		UD_OCSR2 = UD_OCSR2_DMAIEN; // OUT_PKT_RDY interrupt disable
		#ifdef USE_USBD_DMA
			UD_OCSR2 &= ~UD_OCSR2_DMAIEN; // OUT_PKT_RDY interrupt disable
		#endif
		*/
		
		/* fw code, ep3 bulk out mode
	    rINDEX_REG=3;
    	#if (EP3_PKT_SIZE==32)
        	rMAXP_REG=FIFO_SIZE_32;	//EP3:max packit size = 32
    	#else
			rMAXP_REG=FIFO_SIZE_64;	//EP3:max packit size = 64
    	#endif	
    	
    	rIN_CSR1_REG=EPI_FIFO_FLUSH|EPI_CDT|EPI_BULK;
    	rIN_CSR2_REG=EPI_MODE_OUT|EPI_IN_DMA_INT_MASK; //OUT mode, IN_DMA_INT=masked    
    	rOUT_CSR1_REG=EPO_CDT;   	
    			//clear OUT_PKT_RDY, data_toggle_bit.
				//The data toggle bit should be cleared when initialization.
    	rOUT_CSR2_REG=EPO_BULK|EPO_OUT_DMA_INT_MASK;   	
    	*/
    	
gprintk("%s: out ep num: %d\n", __FUNCTION__, ep->num);    	
    	
        /*
         * don't flush he in fifo or there will be an interrupt for that endpoint 
         */
        csr1 = S3C2410_UDC_ICSR1_CLRDT;
        #if 0
        csr2 = S3C2410_UDC_ICSR2_DMAIEN; 
        #else
        csr2 = 0;
       	csr2 |= S3C2410_UDC_ICSR2_DMAIEN; 
       	#endif

        udc_writel(ep->num, S3C2410_UDC_INDEX_REG);
        udc_writel(csr1, S3C2410_UDC_IN_CSR1_REG);
        udc_writel(ep->num, S3C2410_UDC_INDEX_REG);
        udc_writel(csr2, S3C2410_UDC_IN_CSR2_REG);

        csr1 = S3C2410_UDC_OCSR1_FFLUSH | S3C2410_UDC_OCSR1_CLRDT;
        csr2 = S3C2410_UDC_OCSR2_DMAIEN;

        udc_writel(ep->num, S3C2410_UDC_INDEX_REG);
        udc_writel(csr1, S3C2410_UDC_OUT_CSR1_REG);
        udc_writel(ep->num, S3C2410_UDC_INDEX_REG);
        udc_writel(csr2, S3C2410_UDC_OUT_CSR2_REG);
    }


    /*
     * enable irqs 
     */
    int_en_reg = udc_readl(S3C2410_UDC_EP_INT_EN_REG);
    udc_writel(int_en_reg | (1 << ep->num), S3C2410_UDC_EP_INT_EN_REG);


    /*
     * print some debug message 
     */
    tmp = desc->bEndpointAddress;
    dprintk(DEBUG_NORMAL, "enable %s(%d) ep%x%s-blk max %02x\n", _ep->name, ep->num, tmp, desc->bEndpointAddress & USB_DIR_IN ? "in" : "out", max);

    spin_unlock_irqrestore(&dev->lock, flags);

    return 0;
}

/*
 * s3c2410_ep_disable
 */
static int
s3c2410_ep_disable(struct usb_ep *_ep)
{
    struct s3c2410_ep *ep = container_of(_ep, struct s3c2410_ep, ep);
    unsigned long   flags;
    u32             int_en_reg;


    if (!_ep || !ep->desc)
    {
        dprintk(DEBUG_NORMAL, "%s not enabled\n", _ep ? ep->ep.name : NULL);
        return -EINVAL;
    }

    spin_lock_irqsave(&ep->dev->lock, flags);
    ep->desc = 0;

    nuke(ep->dev, ep);

    /*
     * disable irqs 
     */
    int_en_reg = udc_readl(S3C2410_UDC_EP_INT_EN_REG);
    udc_writel(int_en_reg & ~(1 << ep->num), S3C2410_UDC_EP_INT_EN_REG);

    spin_unlock_irqrestore(&ep->dev->lock, flags);

    dprintk(DEBUG_NORMAL, "%s disabled\n", _ep->name);

    return 0;
}

/*
 * s3c2410_alloc_request
 */
static struct usb_request *
s3c2410_alloc_request(struct usb_ep *_ep, unsigned int mem_flags)
{
    struct s3c2410_ep *ep;
    struct s3c2410_request *req;

    dprintk(DEBUG_VERBOSE, "s3c2410_alloc_request(ep=%p,flags=%d)\n", _ep, mem_flags);

    ep = container_of(_ep, struct s3c2410_ep, ep);

    if (!_ep)
        return 0;

    req = kmalloc(sizeof *req, mem_flags);
    if (!req)
        return 0;
    memset(req, 0, sizeof *req);
    INIT_LIST_HEAD(&req->queue);
    return &req->req;
}

/*
 * s3c2410_free_request
 */
static void
s3c2410_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
    struct s3c2410_ep *ep;
    struct s3c2410_request *req;

    dprintk(DEBUG_VERBOSE, "s3c2410_free_request(ep=%p,req=%p)\n", _ep, _req);

    ep = container_of(_ep, struct s3c2410_ep, ep);

    if (!ep || !_req || (!ep->desc && _ep->name != ep0name))
        return;

    req = container_of(_req, struct s3c2410_request, req);
    WARN_ON(!list_empty(&req->queue));
    kfree(req);
}

/*
 * 	s3c2410_alloc_buffer
 */
static void    *
s3c2410_alloc_buffer(struct usb_ep *_ep, unsigned bytes, dma_addr_t * dma, unsigned int mem_flags)
{
    char           *retval;

    dprintk(DEBUG_VERBOSE, "s3c2410_alloc_buffer()\n");

    if (!the_controller->driver)
        return 0;
    retval = kmalloc(bytes, mem_flags);
    *dma = (dma_addr_t) retval;
    return retval;
}

/*
 * s3c2410_free_buffer
 */
static void
s3c2410_free_buffer(struct usb_ep *_ep, void *buf, dma_addr_t dma, unsigned bytes)
{
    dprintk(DEBUG_VERBOSE, "s3c2410_free_buffer()\n");

    if (bytes)
        kfree(buf);
}

/*
 * 	s3c2410_queue
 */
static int
s3c2410_queue(struct usb_ep *_ep, struct usb_request *_req, unsigned int gfp_flags)
{
    struct s3c2410_request *req;
    struct s3c2410_ep *ep;
    struct s3c2410_udc *dev;
    u32             ep_csr = 0;
    int             fifo_count = 0;
    unsigned long   flags;


    ep = container_of(_ep, struct s3c2410_ep, ep);

    if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name)))
    {
        dprintk(DEBUG_NORMAL, "s3c2410_queue: inval 2\n");
        return -EINVAL;
    }

    dev = ep->dev;
    if (unlikely(!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN))
    {
        return -ESHUTDOWN;
    }

    spin_lock_irqsave(&dev->lock, flags);

    req = container_of(_req, struct s3c2410_request, req);
    if (unlikely(!_req || !_req->complete || !_req->buf || !list_empty(&req->queue)))
    {
        if (!_req)
            dprintk(DEBUG_NORMAL, "s3c2410_queue: 1 X X X\n");
        else
        {
            dprintk(DEBUG_NORMAL, "s3c2410_queue: 0 %01d %01d %01d\n", !_req->complete, !_req->buf, !list_empty(&req->queue));
        }
        spin_unlock_irqrestore(&dev->lock, flags);
        return -EINVAL;
    }
#if 0
    /*
     * iso is always one packet per request, that's the only way we can report per-packet status.  that also helps with dma. 
     */
    if (unlikely(ep->bmAttributes == USB_ENDPOINT_XFER_ISOC && req->req.length > le16_to_cpu(ep->desc->wMaxPacketSize)))
        return -EMSGSIZE;
#endif
    _req->status = -EINPROGRESS;
    _req->actual = 0;

    if (ep->bEndpointAddress)
    {
        udc_writel(ep->bEndpointAddress & 0x7F, S3C2410_UDC_INDEX_REG);
        ep_csr = udc_readl(ep->bEndpointAddress & USB_DIR_IN ? S3C2410_UDC_IN_CSR1_REG : S3C2410_UDC_OUT_CSR1_REG);
        fifo_count = fifo_count_out();
    }
    /*
     * kickstart this i/o queue? 
     */
    if (list_empty(&ep->queue))
    {
        if (ep->bEndpointAddress == 0 /* ep0 */ )
        {

            switch (dev->ep0state)
            {
            case EP0_IN_DATA_PHASE:
                if (write_fifo(ep, req))
                {
                    req = 0;
                }
                break;

            case EP0_OUT_DATA_PHASE:
                /*
                 * nothing to do here 
                 */
                dev->ep0state = EP0_IDLE;
                break;

            default:
                spin_unlock_irqrestore(&dev->lock, flags);
                return -EL2HLT;
            }
        }
        else if ((ep->bEndpointAddress & USB_DIR_IN) != 0 && (!(ep_csr & S3C2410_UDC_OCSR1_PKTRDY)) && write_fifo(ep, req))
        {
            req = 0;
        }
        else if ((ep_csr & S3C2410_UDC_OCSR1_PKTRDY) && fifo_count && read_fifo(ep, req))
        {
            req = 0;
        }

    }

    /*
     * pio or dma irq handler advances the queue. 
     */
    if (likely(req != 0))
        list_add_tail(&req->queue, &ep->queue);

    spin_unlock_irqrestore(&dev->lock, flags);

    dprintk(DEBUG_VERBOSE, "s3c2410_queue normal end\n");
    return 0;
}

/*
 * 	s3c2410_dequeue
 */
static int
s3c2410_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
    struct s3c2410_ep *ep;
    struct s3c2410_udc *udc;
    int             retval = -EINVAL;
    unsigned long   flags;
    struct s3c2410_request *req = 0;

    dprintk(DEBUG_VERBOSE, "s3c2410_dequeue(ep=%p,req=%p)\n", _ep, _req);

    if (!the_controller->driver)
        return -ESHUTDOWN;

    if (!_ep || !_req)
        return retval;
    ep = container_of(_ep, struct s3c2410_ep, ep);
    udc = container_of(ep->gadget, struct s3c2410_udc, gadget);

    spin_lock_irqsave(&udc->lock, flags);
    list_for_each_entry(req, &ep->queue, queue)
    {
        if (&req->req == _req)
        {
            list_del_init(&req->queue);
            _req->status = -ECONNRESET;
            retval = 0;
            break;
        }
    }
    spin_unlock_irqrestore(&udc->lock, flags);

    if (retval == 0)
    {
        dprintk(DEBUG_VERBOSE, "dequeued req %p from %s, len %d buf %p\n", req, _ep->name, _req->length, _req->buf);

        _req->complete(_ep, _req);
        done(ep, req, -ECONNRESET);
    }
    return retval;

    return 0;
}


/*
 * s3c2410_set_halt
 */
static int
s3c2410_set_halt(struct usb_ep *_ep, int value)
{
    return 0;
}


static const struct usb_ep_ops s3c2410_ep_ops = {
    .enable = s3c2410_ep_enable,
    .disable = s3c2410_ep_disable,

    .alloc_request = s3c2410_alloc_request,
    .free_request = s3c2410_free_request,

    .alloc_buffer = s3c2410_alloc_buffer,
    .free_buffer = s3c2410_free_buffer,

    .queue = s3c2410_queue,
    .dequeue = s3c2410_dequeue,

    .set_halt = s3c2410_set_halt,
};

/*------------------------- usb_gadget_ops ----------------------------------*/

/*
 * 	s3c2410_g_get_frame
 */
static int
s3c2410_g_get_frame(struct usb_gadget *_gadget)
{
    int             tmp;

    dprintk(DEBUG_VERBOSE, "s3c2410_g_get_frame()\n");

    tmp = udc_readl(S3C2410_UDC_FRAME_NUM2_REG) << 8;
    tmp |= udc_readl(S3C2410_UDC_FRAME_NUM1_REG);

    return tmp & 0xffff;
}

/*
 * 	s3c2410_wakeup
 */
static int
s3c2410_wakeup(struct usb_gadget *_gadget)
{

    dprintk(DEBUG_NORMAL, "s3c2410_wakeup()\n");

    return 0;
}

/*
 * 	s3c2410_set_selfpowered
 */
static int
s3c2410_set_selfpowered(struct usb_gadget *_gadget, int value)
{
    struct s3c2410_udc *udc;

    dprintk(DEBUG_NORMAL, "s3c2410_set_selfpowered()\n");

    udc = container_of(_gadget, struct s3c2410_udc, gadget);

    if (value)
        udc->devstatus |= (1 << USB_DEVICE_SELF_POWERED);
    else
        udc->devstatus &= ~(1 << USB_DEVICE_SELF_POWERED);

    return 0;
}



static const struct usb_gadget_ops s3c2410_ops = {
    .get_frame = s3c2410_g_get_frame,
    .wakeup = s3c2410_wakeup,
    .set_selfpowered = s3c2410_set_selfpowered,
};


/*------------------------- gadget driver handling---------------------------*/
/*
 * udc_disable
 */
static void
udc_disable(struct s3c2410_udc *dev)
{
    dprintk(DEBUG_NORMAL, "udc_disable called\n");

    /*
     * Disable all interrupts 
     */
    udc_writel(0x00, S3C2410_UDC_USB_INT_EN_REG);
    udc_writel(0x00, S3C2410_UDC_EP_INT_EN_REG);

    /*
     * Clear the interrupt registers 
     */
    udc_writel(S3C2410_UDC_USBINT_RESET | S3C2410_UDC_USBINT_RESUME | S3C2410_UDC_USBINT_SUSPEND, S3C2410_UDC_USB_INT_REG);
    udc_writel(0x1F, S3C2410_UDC_EP_INT_REG);


    /*
     * Good bye, cruel world 
     */
    if (udc_info && udc_info->udc_command)
        udc_info->udc_command(S3C2410_UDC_P_DISABLE);

    /*
     * Set address to 0 
     */
    /*
     * udc_writel( 0x80, S3C2410_UDC_FUNC_ADDR_REG);
     */

    /*
     * Set speed to unknown 
     */
    dev->gadget.speed = USB_SPEED_UNKNOWN;
}

/*
 * udc_reinit
 */
static void
udc_reinit(struct s3c2410_udc *dev)
{
    u32             i;

    /*
     * device/ep0 records init 
     */
    INIT_LIST_HEAD(&dev->gadget.ep_list);
    INIT_LIST_HEAD(&dev->gadget.ep0->ep_list);
    dev->ep0state = EP0_IDLE;


    for (i = 0; i < S3C2410_ENDPOINTS; i++)
    {
        struct s3c2410_ep *ep = &dev->ep[i];

        if (i != 0)
            list_add_tail(&ep->ep.ep_list, &dev->gadget.ep_list);

        ep->dev = dev;
        ep->desc = 0;
        INIT_LIST_HEAD(&ep->queue);
    }
}

/*
 * udc_enable
 */
static void
udc_enable(struct s3c2410_udc *dev)
{
    int             i;
    int 			ep0csr;

    dprintk(DEBUG_NORMAL, "udc_enable called\n");

    /*
     * dev->gadget.speed = USB_SPEED_UNKNOWN; 
     */
    dev->gadget.speed = USB_SPEED_FULL;

    /*
     * Set MAXP for all endpoints 
     */
    for (i = 0; i < S3C2410_ENDPOINTS; i++)
    {

        udc_writel(i, S3C2410_UDC_INDEX_REG);
        udc_writel((dev->ep[i].ep.maxpacket & 0x7ff) >> 3, S3C2410_UDC_MAXP_REG);
    }

	S3C2410_UDC_SETIX(base_addr, EP0);
	udc_writel( (S3C2410_UDC_EP0_CSR_SOPKTRDY|S3C2410_UDC_EP0_CSR_SSE), S3C2410_UDC_EP0_CSR_REG );
    S3C2410_UDC_SETIX(base_addr, EP0);
    ep0csr = udc_readl(S3C2410_UDC_EP0_CSR_REG);

    /*
     * Set default power state 
     */
    udc_writel(DEFAULT_POWER_STATE, S3C2410_UDC_PWR_REG);

    /*
     * Enable reset and suspend interrupt interrupts 
     */
    udc_writel(1 << 2 | 1 << 0, S3C2410_UDC_USB_INT_EN_REG);

    /*
     * Enable ep0 interrupt 
     */
    udc_writel(0x01, S3C2410_UDC_EP_INT_EN_REG);

    /*
     * time to say "hello, world" 
     */
    if (udc_info && udc_info->udc_command)
        udc_info->udc_command(S3C2410_UDC_P_ENABLE);
}


/*
 * 	nop_release
 */
static void
nop_release(struct device *dev)
{
    dprintk(DEBUG_NORMAL, "%s %s\n", __FUNCTION__, dev->bus_id);
}

/*
 *	usb_gadget_register_driver
 */
int
usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
    struct s3c2410_udc *udc = the_controller;
    int             retval;

    dprintk(DEBUG_NORMAL, "usb_gadget_register_driver() '%s'\n", driver->driver.name);

    /*
     * Sanity checks 
     */
    if (!udc)
        return -ENODEV;
    if (udc->driver)
        return -EBUSY;
    if (!driver->bind || !driver->unbind || !driver->setup || driver->speed == USB_SPEED_UNKNOWN)
        return -EINVAL;

    /*
     * Hook the driver 
     */
    udc->driver = driver;
    udc->gadget.dev.driver = &driver->driver;

    /*
     * Bind the driver 
     */
    device_add(&udc->gadget.dev);
    dprintk(DEBUG_NORMAL, "binding gadget driver '%s'\n", driver->driver.name);
    if ((retval = driver->bind(&udc->gadget)) != 0)
    {
        device_del(&udc->gadget.dev);
        udc->driver = 0;
        udc->gadget.dev.driver = 0;
        return retval;
    }

    /*
     * driver->driver.bus = 0; 
     */

    /*
     * Enable udc 
     */
    udc_enable(udc);

    return 0;
}


/*
 * 	usb_gadget_unregister_driver
 */
int
usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
    struct s3c2410_udc *udc = the_controller;

    if (!udc)
        return -ENODEV;
    if (!driver || driver != udc->driver)
        return -EINVAL;

    dprintk(DEBUG_NORMAL, "usb_gadget_register_driver() '%s'\n", driver->driver.name);

    driver->unbind(&udc->gadget);
    device_del(&udc->gadget.dev);
    udc->driver = 0;

    device_release_driver(&udc->gadget.dev);
    driver_unregister(&driver->driver);

    /*
     * Disable udc 
     */
    udc_disable(udc);

    return 0;
}

/*---------------------------------------------------------------------------*/
static struct s3c2410_udc memory = {
    .gadget = {
               .ops = &s3c2410_ops,
               .ep0 = &memory.ep[0].ep,
               .name = gadget_name,
               .dev = {
                       .bus_id = "gadget",
                       .release = nop_release,
                       },
               },

    /*
     * control endpoint 
     */
    .ep[0] = {
              .num = 0,
              .ep = {
                     .name = ep0name,
                     .ops = &s3c2410_ep_ops,
                     .maxpacket = EP0_FIFO_SIZE,
                     },
              .dev = &memory,
              },

    /*
     * first group of endpoints 
     */
    .ep[1] = {
              .num = 1,
              .ep = {
                     .name = "ep1-bulk",
                     .ops = &s3c2410_ep_ops,
                     .maxpacket = EP_FIFO_SIZE,
                     },
              .dev = &memory,
              .fifo_size = EP_FIFO_SIZE,
              .bEndpointAddress = 1,
              .bmAttributes = USB_ENDPOINT_XFER_BULK,
              },
    .ep[2] = {
              .num = 2,
              .ep = {
                     .name = "ep2-bulk",
                     .ops = &s3c2410_ep_ops,
                     .maxpacket = EP_FIFO_SIZE,
                     },
              .dev = &memory,
              .fifo_size = EP_FIFO_SIZE,
              .bEndpointAddress = 2,
              .bmAttributes = USB_ENDPOINT_XFER_BULK,
              },
    .ep[3] = {
              .num = 3,
              .ep = {
                     .name = "ep3-bulk",
                     .ops = &s3c2410_ep_ops,
                     .maxpacket = EP_FIFO_SIZE,
                     },
              .dev = &memory,
              .fifo_size = EP_FIFO_SIZE,
              .bEndpointAddress = 3,
              .bmAttributes = USB_ENDPOINT_XFER_BULK,
              },
    .ep[4] = {
              .num = 4,
              .ep = {
                     .name = "ep4-bulk",
                     .ops = &s3c2410_ep_ops,
                     .maxpacket = EP_FIFO_SIZE,
                     },
              .dev = &memory,
              .fifo_size = EP_FIFO_SIZE,
              .bEndpointAddress = 4,
              .bmAttributes = USB_ENDPOINT_XFER_BULK,
              }

};

/*
 *	probe - binds to the platform device
 */
static int
s3c2410_udc_probe(struct device *_dev)
{
    struct s3c2410_udc *udc = &memory;
    int             retval;


    dprintk(DEBUG_NORMAL, "s3c2410_udc_probe\n");

    spin_lock_init(&udc->lock);
    udc_info = _dev->platform_data;

    rsrc_start = S3C2410_PA_USBDEV;
    rsrc_len = S3C24XX_SZ_USBDEV;

    if (!request_mem_region(rsrc_start, rsrc_len, gadget_name))
        return -EBUSY;

    base_addr = ioremap(rsrc_start, rsrc_len);
    if (!base_addr)
    {
        retval = -ENOMEM;
        goto err_mem;
    }


    device_initialize(&udc->gadget.dev);
    udc->gadget.dev.parent = _dev;
    udc->gadget.dev.dma_mask = _dev->dma_mask;

    the_controller = udc;
    dev_set_drvdata(_dev, udc);

    udc_disable(udc);
    udc_reinit(udc);

    /*
     * irq setup after old hardware state is cleaned up 
     */
    retval = request_irq(IRQ_USBD, s3c2410_udc_irq, SA_INTERRUPT, gadget_name, udc);
    if (retval != 0)
    {
        printk(KERN_ERR "%s: can't get irq %i, err %d\n", gadget_name, IRQ_USBD, retval);
        retval = -EBUSY;
        goto err_map;
    }
    dprintk(DEBUG_VERBOSE, "%s: got irq %i\n", gadget_name, IRQ_USBD);

#ifdef ENABLE_SYSFS
    /*
     * create device files 
     */
    device_create_file(_dev, &dev_attr_regs);
#endif
    return 0;

  err_map:
    iounmap(base_addr);
  err_mem:
    release_mem_region(rsrc_start, rsrc_len);

    return retval;
}

/*
 * 	s3c2410_udc_remove
 */
static int
s3c2410_udc_remove(struct device *_dev)
{
    struct s3c2410_udc *udc = _dev->driver_data;

    dprintk(DEBUG_NORMAL, "s3c2410_udc_remove\n");
    usb_gadget_unregister_driver(udc->driver);

    if (udc->got_irq)
    {
        free_irq(IRQ_USBD, udc);
        udc->got_irq = 0;
    }

    iounmap(base_addr);
    release_mem_region(rsrc_start, rsrc_len);

    dev_set_drvdata(_dev, 0);
    kfree(udc);

    return 0;
}

static struct device_driver udc_driver = {
    .name = "s3c2410-usbgadget",
    .bus = &platform_bus_type,
    .probe = s3c2410_udc_probe,
    .remove = s3c2410_udc_remove,
};


static int __init udc_init(void)
{
	u32 tmp;
	//dprintk(DEBUG_NORMAL, "%s: version %s\n", gadget_name, DRIVER_VERSION);


	printk("%s: version %s\n", gadget_name, DRIVER_VERSION);


    tmp = readl(S3C2410_MISCCR);
    tmp &= ~(1<<13 | 1<<3); 
    writel(tmp, S3C2410_MISCCR);
    
    tmp = readl(S3C2410_MISCCR);


	udc_clock = clk_get(NULL, "usb-device");
	if (!udc_clock) {
		printk(KERN_INFO "failed to get udc clock source\n");
		return -ENOENT;
	}
	clk_use(udc_clock);

	clk_disable(udc_clock);

	//ghcstop: input clock 12Mhz
	#ifdef CONFIG_S3C2440_INCLK12
		// 48Mhz
	    tmp = (0x38 << S3C2410_PLLCON_MDIVSHIFT) | (0x02 << S3C2410_PLLCON_PDIVSHIFT) | (0x02 << S3C2410_PLLCON_SDIVSHIFT);
    	writel(tmp, S3C2410_UPLLCON);
	#else  // 16.9344 Mhz
		// 47.98Mhz 
	    tmp = (0x3C << S3C2410_PLLCON_MDIVSHIFT) | (0x04 << S3C2410_PLLCON_PDIVSHIFT) | (0x02 << S3C2410_PLLCON_SDIVSHIFT);
    	writel(tmp, S3C2410_UPLLCON);
	#endif


	clk_enable(udc_clock);

	mdelay(10);
	

	dprintk(DEBUG_VERBOSE, "got and enabled clock\n");

	return driver_register(&udc_driver);
}

static void __exit
udc_exit(void)
{
    if (udc_clock)
    {
        clk_disable(udc_clock);
        clk_unuse(udc_clock);
        clk_put(udc_clock);
        udc_clock = NULL;
    }

    driver_unregister(&udc_driver);
}


EXPORT_SYMBOL(usb_gadget_unregister_driver);
EXPORT_SYMBOL(usb_gadget_register_driver);

module_init(udc_init);
module_exit(udc_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
