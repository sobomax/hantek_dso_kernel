/*
 *  linux/drivers/mmc/s3c2410mci.h - Samsung S3C2410 SDI Interface driver
 *
 *  Copyright (C) 2004 Thomas Kleffel, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <linux/mmc/protocol.h>

#include <asm/dma.h>
#include <asm/dma-mapping.h>
#include <asm/arch/dma.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/hardware/amba.h>
#include <asm/hardware/clock.h>
#include <asm/mach/mmc.h>

#include <asm/arch/regs-sdi.h>
#include <asm/arch/regs-gpio.h>

//#define S3C2410SDI_DMA_BACKBUF

#ifdef CONFIG_MMC_DEBUG
#define DBG(x...)       printk(KERN_DEBUG x)
#else
#define DBG(x...)       do { } while (0)
#endif

#include "s3c2410mci.h"

#define DRIVER_NAME "mmci-s3c2410"
#define PFX DRIVER_NAME ": "

#define RESSIZE(ressource) (((ressource)->end - (ressource)->start)+1)

// #define KERN_DEBUG KERN_INFO

typedef enum {
	DMAP_READ,
	DMAP_WRITE,
} eDMAPurpose_t;

static struct s3c2410_dma_client s3c2410sdi_dma_client = {
	.name		= "s3c2410-sdi",
};



/*
 * ISR for SDI Interface IRQ
 * Communication between driver and ISR works as follows:
 *   host->mrq 			points to current request
 *   host->complete_what	tells the ISR when the request is considered done
 *     COMPLETION_CMDSENT	  when the command was sent
 *     COMPLETION_RSPFIN          when a response was received
 *     COMPLETION_XFERFINISH	  when the data transfer is finished
 *     COMPLETION_XFERFINISH_RSPFIN both of the above.
 *   host->complete_request	is the completion-object the driver waits for
 *
 * 1) Driver sets up host->mrq and host->complete_what
 * 2) Driver prepares the transfer
 * 3) Driver enables interrupts
 * 4) Driver starts transfer
 * 5) Driver waits for host->complete_rquest
 * 6) ISR checks for request status (errors and success)
 * 6) ISR sets host->mrq->cmd->error and host->mrq->data->error
 * 7) ISR completes host->complete_request
 * 8) ISR disables interrupts
 * 9) Driver wakes up and takes care of the request
*/

static irqreturn_t s3c2410sdi_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	struct s3c2410sdi_host *host;
	u32 sdi_csta, sdi_dsta, sdi_dcnt;
	u32 sdi_cclear, sdi_dclear;
	unsigned long iflags;

	host = (struct s3c2410sdi_host *)dev_id;

	//Check for things not supposed to happen
	if(!host) return IRQ_HANDLED;
	
	sdi_csta 	= readl(host->base + S3C2410_SDICMDSTAT);
	sdi_dsta 	= readl(host->base + S3C2410_SDIDSTA);
	sdi_dcnt 	= readl(host->base + S3C2410_SDIDCNT);
	
	DBG(PFX "IRQ csta=0x%08x dsta=0x%08x dcnt:0x%08x\n", sdi_csta, sdi_dsta, sdi_dcnt);
		
	spin_lock_irqsave( &host->complete_lock, iflags);
	
	if( host->complete_what==COMPLETION_NONE ) {
		goto clear_imask;
	}
	
	if(!host->mrq) { 
		goto clear_imask;
	}

	
	sdi_csta 	= readl(host->base + S3C2410_SDICMDSTAT);
	sdi_dsta 	= readl(host->base + S3C2410_SDIDSTA);
	sdi_dcnt 	= readl(host->base + S3C2410_SDIDCNT);
	sdi_cclear	= 0;
	sdi_dclear	= 0;
	
	
	if(sdi_csta & S3C2410_SDICMDSTAT_CMDTIMEOUT) {
		host->mrq->cmd->error = MMC_ERR_TIMEOUT;
		goto transfer_closed;
	}

	if(sdi_csta & S3C2410_SDICMDSTAT_CMDSENT) {
		if(host->complete_what == COMPLETION_CMDSENT) {
			host->mrq->cmd->error = MMC_ERR_NONE;
			goto transfer_closed;
		}

		sdi_cclear |= S3C2410_SDICMDSTAT_CMDSENT;
	}

	if(sdi_csta & S3C2410_SDICMDSTAT_CRCFAIL) {
		if(host->mrq->cmd->flags & MMC_RSP_CRC) {
			host->mrq->cmd->error = MMC_ERR_BADCRC;
			goto transfer_closed;
		}

		sdi_cclear |= S3C2410_SDICMDSTAT_CRCFAIL;
	}

	if(sdi_csta & S3C2410_SDICMDSTAT_RSPFIN) {
		if(host->complete_what == COMPLETION_RSPFIN) {
			host->mrq->cmd->error = MMC_ERR_NONE;
			goto transfer_closed;
		}

		if(host->complete_what == COMPLETION_XFERFINISH_RSPFIN) {
			host->mrq->cmd->error = MMC_ERR_NONE;
			host->complete_what = COMPLETION_XFERFINISH;
		}

		sdi_cclear |= S3C2410_SDICMDSTAT_RSPFIN;
	}

	if(sdi_dsta & S3C2410_SDIDSTA_FIFOFAIL) {
		host->mrq->cmd->error = MMC_ERR_NONE;
		host->mrq->data->error = MMC_ERR_FIFO;
		goto transfer_closed;
	}

	if(sdi_dsta & S3C2410_SDIDSTA_RXCRCFAIL) {
		host->mrq->cmd->error = MMC_ERR_NONE;
		host->mrq->data->error = MMC_ERR_BADCRC;
		goto transfer_closed;
	}

	if(sdi_dsta & S3C2410_SDIDSTA_CRCFAIL) {
		host->mrq->cmd->error = MMC_ERR_NONE;
		host->mrq->data->error = MMC_ERR_BADCRC;
		goto transfer_closed;
	}

	if(sdi_dsta & S3C2410_SDIDSTA_DATATIMEOUT) {
		host->mrq->cmd->error = MMC_ERR_NONE;
		host->mrq->data->error = MMC_ERR_TIMEOUT;
		goto transfer_closed;
	}

	if(sdi_dsta & S3C2410_SDIDSTA_XFERFINISH) {
		if(host->complete_what == COMPLETION_XFERFINISH) {
			host->mrq->cmd->error = MMC_ERR_NONE;
			host->mrq->data->error = MMC_ERR_NONE;
			goto transfer_closed;
		}

		if(host->complete_what == COMPLETION_XFERFINISH_RSPFIN) {
			host->mrq->data->error = MMC_ERR_NONE;
			host->complete_what = COMPLETION_RSPFIN;
		}

		sdi_dclear |= S3C2410_SDIDSTA_XFERFINISH;
	}

	writel(sdi_cclear, host->base + S3C2410_SDICMDSTAT);
	writel(sdi_dclear, host->base + S3C2410_SDIDSTA);

	spin_unlock_irqrestore( &host->complete_lock, iflags);
	DBG(PFX "IRQ still waiting.\n");
	return IRQ_HANDLED;


transfer_closed:
	host->complete_what = COMPLETION_NONE;
	complete(&host->complete_request);
	writel(0, host->base + S3C2410_SDIIMSK);
	spin_unlock_irqrestore( &host->complete_lock, iflags);
	DBG(PFX "IRQ transfer closed.\n");
	return IRQ_HANDLED;
	
clear_imask:
	writel(0, host->base + S3C2410_SDIIMSK);
	spin_unlock_irqrestore( &host->complete_lock, iflags);
	DBG(PFX "IRQ clear imask.\n");
	return IRQ_HANDLED;

}


/*
 * ISR for the CardDetect Pin
*/

static irqreturn_t s3c2410sdi_irq_cd(int irq, void *dev_id, struct pt_regs *regs)
{
	struct s3c2410sdi_host *host = (struct s3c2410sdi_host *)dev_id;
	mmc_detect_change(host->mmc);

	return IRQ_HANDLED;
}



void s3c2410sdi_dma_done_callback(s3c2410_dma_chan_t *dma_ch, void *buf_id,
	int size, s3c2410_dma_buffresult_t result)
{	unsigned long iflags;
	u32 sdi_csta, sdi_dsta,sdi_dcnt;
	struct s3c2410sdi_host *host = (struct s3c2410sdi_host *)buf_id;
	
	sdi_csta 	= readl(host->base + S3C2410_SDICMDSTAT);
	sdi_dsta 	= readl(host->base + S3C2410_SDIDSTA);
	sdi_dcnt 	= readl(host->base + S3C2410_SDIDCNT);
	
	DBG(PFX "DMAD csta=0x%08x dsta=0x%08x dcnt:0x%08x result:0x%08x\n", sdi_csta, sdi_dsta, sdi_dcnt, result);
	
	spin_lock_irqsave( &host->complete_lock, iflags);
	
	if(!host->mrq) goto out;
	if(!host->mrq->data) goto out;
	
	
	sdi_csta 	= readl(host->base + S3C2410_SDICMDSTAT);
	sdi_dsta 	= readl(host->base + S3C2410_SDIDSTA);
	sdi_dcnt 	= readl(host->base + S3C2410_SDIDCNT);
		
	if( result!=S3C2410_RES_OK ) {
		goto fail_request;
	}
	
	
	if(host->mrq->data->flags & MMC_DATA_READ) {
		if( sdi_dcnt>0 ) {
			goto fail_request;
		}
	}
	
out:	
	complete(&host->complete_dma);
	spin_unlock_irqrestore( &host->complete_lock, iflags);
	return;


fail_request:
	host->mrq->data->error = MMC_ERR_DMA;
	host->complete_what = COMPLETION_NONE;
	complete(&host->complete_dma);
	complete(&host->complete_request);
	writel(0, host->base + S3C2410_SDIIMSK);
	goto out;

}


void s3c2410sdi_dma_setup(struct s3c2410sdi_host *host, eDMAPurpose_t purpose) {
	s3c2410_dmasrc_t source;

	switch(purpose) {
		default:
		case DMAP_READ:
			source  = S3C2410_DMASRC_HW;
			break;

		case DMAP_WRITE:
			source  = S3C2410_DMASRC_MEM;
			break;
	}

	s3c2410_dma_devconfig(host->dma, source, 3, host->mem->start + S3C2410_SDIDATA);
	s3c2410_dma_config(host->dma, 4, (1<<23) | (2<<24));
	s3c2410_dma_set_buffdone_fn(host->dma, s3c2410sdi_dma_done_callback);
	s3c2410_dma_setflags(host->dma, S3C2410_DMAF_AUTOSTART);
}

static void s3c2410sdi_request(struct mmc_host *mmc, struct mmc_request *mrq) {
 	struct s3c2410sdi_host *host = mmc_priv(mmc);
	u32 sdi_carg, sdi_ccon, sdi_timer;
	u32 sdi_bsize, sdi_dcon, sdi_imsk;

	DBG(KERN_DEBUG PFX "request: [CMD] opcode:0x%02x arg:0x%08x flags:%x retries:%u\n",
		mrq->cmd->opcode, mrq->cmd->arg, mrq->cmd->flags, mrq->cmd->retries);


	sdi_ccon = mrq->cmd->opcode & S3C2410_SDICMDCON_INDEX;
	sdi_ccon|= S3C2410_SDICMDCON_SENDERHOST;
	sdi_ccon|= S3C2410_SDICMDCON_CMDSTART;

	sdi_carg = mrq->cmd->arg;

	//FIXME: Timer value ?!
	sdi_timer= 0xF000;

	sdi_bsize= 0;
	sdi_dcon = 0;
	sdi_imsk = 0;

	//enable interrupts for transmission errors
	sdi_imsk |= S3C2410_SDIIMSK_RESPONSEND;
	sdi_imsk |= S3C2410_SDIIMSK_CRCSTATUS;


	host->complete_what = COMPLETION_CMDSENT;

	if (mrq->cmd->flags & MMC_RSP_MASK) {
		host->complete_what = COMPLETION_RSPFIN;

		sdi_ccon |= S3C2410_SDICMDCON_WAITRSP;
		sdi_imsk |= S3C2410_SDIIMSK_CMDTIMEOUT;

	} else {
		//We need the CMDSENT-Interrupt only if we want are not waiting
		//for a response
		sdi_imsk |= S3C2410_SDIIMSK_CMDSENT;
	}

	if(mrq->cmd->flags & MMC_RSP_LONG) {
		sdi_ccon|= S3C2410_SDICMDCON_LONGRSP;
	}

	if(mrq->cmd->flags & MMC_RSP_CRC) {
		sdi_imsk |= S3C2410_SDIIMSK_RESPONSECRC;
	}


	if (mrq->data) {
		host->complete_what = COMPLETION_XFERFINISH_RSPFIN;



		sdi_bsize = (1 << mrq->data->blksz_bits);

		sdi_dcon  = (mrq->data->blocks & S3C2410_SDIDCON_BLKNUM_MASK);
		sdi_dcon |= S3C2410_SDIDCON_DMAEN;

		sdi_imsk |= S3C2410_SDIIMSK_FIFOFAIL;
		sdi_imsk |= S3C2410_SDIIMSK_DATACRC;
		sdi_imsk |= S3C2410_SDIIMSK_DATATIMEOUT;
		sdi_imsk |= S3C2410_SDIIMSK_DATAFINISH;
		sdi_imsk |= 0xFFFFFFE0;

		DBG(PFX "request: [DAT] bsize:%u blocks:%u bytes:%u\n",
			sdi_bsize, mrq->data->blocks, mrq->data->blocks * sdi_bsize);

		if(mrq->data->flags & MMC_DATA_WIDE) {
			sdi_dcon |= S3C2410_SDIDCON_WIDEBUS;
		}

		if(!(mrq->data->flags & MMC_DATA_STREAM)) {
			sdi_dcon |= S3C2410_SDIDCON_BLOCKMODE;
		}

		if(mrq->data->flags & MMC_DATA_WRITE) {
			sdi_dcon |= S3C2410_SDIDCON_TXAFTERRESP;
			sdi_dcon |= S3C2410_SDIDCON_XFER_TXSTART;
			s3c2410sdi_dma_setup(host, DMAP_WRITE);
#ifdef S3C2410SDI_DMA_BACKBUF			
			memcpy(host->dmabuf_log, mrq->data->req->buffer, mrq->data->blocks * sdi_bsize);
#endif
		}

		if(mrq->data->flags & MMC_DATA_READ) {
			sdi_dcon |= S3C2410_SDIDCON_RXAFTERCMD;
			sdi_dcon |= S3C2410_SDIDCON_XFER_RXSTART;
			s3c2410sdi_dma_setup(host, DMAP_READ);
		}
		
		

		//start DMA
#ifdef S3C2410SDI_DMA_BACKBUF
		s3c2410_dma_enqueue(host->dma, (void *) host,
			host->dmabuf_phys,
			(mrq->data->blocks << mrq->data->blksz_bits) );
#else
		s3c2410_dma_enqueue(host->dma, (void *) host,
			virt_to_phys(mrq->data->req->buffer),
			(mrq->data->blocks << mrq->data->blksz_bits) );
#endif
	}

	host->mrq = mrq;

	init_completion(&host->complete_request);
	init_completion(&host->complete_dma);

	//Clear command and data status registers
	writel(0xFFFFFFFF, host->base + S3C2410_SDICMDSTAT);
	writel(0xFFFFFFFF, host->base + S3C2410_SDIDSTA);

	// Setup SDI controller
	writel(sdi_bsize,host->base + S3C2410_SDIBSIZE);
	writel(sdi_timer,host->base + S3C2410_SDITIMER);
	writel(sdi_imsk,host->base + S3C2410_SDIIMSK);

	// Setup SDI command argument and data control
	writel(sdi_carg, host->base + S3C2410_SDICMDARG);
	writel(sdi_dcon, host->base + S3C2410_SDIDCON);

	// This initiates transfer
	writel(sdi_ccon, host->base + S3C2410_SDICMDCON);

	// Wait for transfer to complete
	wait_for_completion(&host->complete_request);
	DBG("[CMD] request complete.\n");
	if(mrq->data) {
		wait_for_completion(&host->complete_dma);
		DBG("[DAT] DMA complete.\n");
	}
	
	// Wait for DMA to complete
//	if(mrq->data) wait_for_completion(&host->complete_dma);

	//Cleanup controller
	writel(0, host->base + S3C2410_SDICMDARG);
	writel(0, host->base + S3C2410_SDIDCON);
	writel(0, host->base + S3C2410_SDICMDCON);
	writel(0, host->base + S3C2410_SDIIMSK);

	// Read response
	mrq->cmd->resp[0] = readl(host->base + S3C2410_SDIRSP0);
	mrq->cmd->resp[1] = readl(host->base + S3C2410_SDIRSP1);
	mrq->cmd->resp[2] = readl(host->base + S3C2410_SDIRSP2);
	mrq->cmd->resp[3] = readl(host->base + S3C2410_SDIRSP3);

	host->mrq = NULL;

	DBG(PFX "request done.\n");

	// If we have no data transfer we are finished here
	if (!mrq->data) goto request_done;

	// Calulate the amout of bytes transfer, but only if there was
	// no error
	if(mrq->data->error == MMC_ERR_NONE) {
		mrq->data->bytes_xfered = (mrq->data->blocks << mrq->data->blksz_bits);
		if(mrq->data->flags & MMC_DATA_READ);
#ifdef S3C2410SDI_DMA_BACKBUF
		memcpy(mrq->data->req->buffer, host->dmabuf_log, mrq->data->bytes_xfered);
#endif
	} else {
		mrq->data->bytes_xfered = 0;
	}

	// If we had an error while transfering data we flush the
	// DMA channel to clear out any garbage
	if(mrq->data->error != MMC_ERR_NONE) {
		s3c2410_dma_ctrl(host->dma, S3C2410_DMAOP_FLUSH);
		DBG(PFX "flushing DMA.\n");		
	}
	// Issue stop command
	if(mrq->data->stop) mmc_wait_for_cmd(mmc, mrq->data->stop, 3);


request_done:

	mrq->done(mrq);
}

static void s3c2410sdi_set_ios(struct mmc_host *mmc, struct mmc_ios *ios) {
	struct s3c2410sdi_host *host = mmc_priv(mmc);
	u32 sdi_psc, sdi_con;

	//Set power
	sdi_con = readl(host->base + S3C2410_SDICON);
	switch(ios->power_mode) {
		case MMC_POWER_ON:
		case MMC_POWER_UP:
			s3c2410_gpio_setpin(S3C2410_GPA17, 1); // card power on

			s3c2410_gpio_cfgpin(S3C2410_GPE5, S3C2410_GPE5_SDCLK);
			s3c2410_gpio_cfgpin(S3C2410_GPE6, S3C2410_GPE6_SDCMD);
			s3c2410_gpio_cfgpin(S3C2410_GPE7, S3C2410_GPE7_SDDAT0);
			s3c2410_gpio_cfgpin(S3C2410_GPE8, S3C2410_GPE8_SDDAT1);
			s3c2410_gpio_cfgpin(S3C2410_GPE9, S3C2410_GPE9_SDDAT2);
			s3c2410_gpio_cfgpin(S3C2410_GPE10, S3C2410_GPE10_SDDAT3);

			sdi_con|= S3C2410_SDICON_FIFORESET;
			break;

		case MMC_POWER_OFF:
		default:
			s3c2410_gpio_setpin(S3C2410_GPA17, 0); // card power off

			s3c2410_gpio_setpin(S3C2410_GPE5, 0);
			s3c2410_gpio_cfgpin(S3C2410_GPE5, S3C2410_GPE5_OUTP);
			break;
	}

	//Set clock
	for(sdi_psc=0;sdi_psc<255;sdi_psc++) {
		if( (clk_get_rate(host->clk) / (2*(sdi_psc+1))) <= ios->clock) break;
	}

	if(sdi_psc > 255) sdi_psc = 255;
	writel(sdi_psc, host->base + S3C2410_SDIPRE);

	//Set CLOCK_ENABLE
	if(ios->clock) 	sdi_con |= S3C2410_SDICON_CLOCKTYPE;
	else		sdi_con &=~S3C2410_SDICON_CLOCKTYPE;

	writel(sdi_con, host->base + S3C2410_SDICON);
}

static struct mmc_host_ops s3c2410sdi_ops = {
	.request	= s3c2410sdi_request,
	.set_ios	= s3c2410sdi_set_ios,
};

static int s3c2410sdi_probe(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct mmc_host 	*mmc;
	struct s3c2410sdi_host 	*host;

	int ret;

	mmc = mmc_alloc_host(sizeof(struct s3c2410sdi_host), dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto probe_out;
	}

	host = mmc_priv(mmc);

	spin_lock_init( &host->complete_lock );
	host->complete_what 	= COMPLETION_NONE;
	host->mmc 		= mmc;
	host->dma		= S3C2410SDI_DMA;
	host->irq_cd		= IRQ_EINT2;


	host->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!host->mem) {
		printk(KERN_INFO PFX "failed to get io memory region resouce.\n");
		ret = -ENOENT;
		goto probe_free_host;
	}

	host->mem = request_mem_region(host->mem->start,
		RESSIZE(host->mem), pdev->name);

	if (!host->mem) {
		printk(KERN_INFO PFX "failed to request io memory region.\n");
		ret = -ENOENT;
		goto probe_free_host;
	}

	host->base = ioremap(host->mem->start, RESSIZE(host->mem));
	if (host->base == 0) {
		printk(KERN_INFO PFX "failed to ioremap() io memory region.\n");
		ret = -EINVAL;
		goto probe_free_mem_region;
	}

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq == 0) {
		printk(KERN_INFO PFX "failed to get interrupt resouce.\n");
		ret = -EINVAL;
		goto probe_iounmap;
	}

	if(request_irq(host->irq, s3c2410sdi_irq, 0, DRIVER_NAME, host)) {
		printk(KERN_INFO PFX "failed to request sdi interrupt.\n");
		ret = -ENOENT;
		goto probe_iounmap;
	}

	s3c2410_gpio_cfgpin(S3C2410_GPF2, S3C2410_GPF2_EINT2);
	set_irq_type(host->irq_cd, IRQT_BOTHEDGE);

	if(request_irq(host->irq_cd, s3c2410sdi_irq_cd, 0, DRIVER_NAME, host)) {
		printk(KERN_WARNING PFX "failed to request card detect interrupt.\n" );
		ret = -ENOENT;
		goto probe_free_irq;
	}

	if(s3c2410_dma_request(S3C2410SDI_DMA, &s3c2410sdi_dma_client, NULL)) {
		printk(KERN_WARNING PFX "unable to get DMA channel.\n" );
		ret = -EBUSY;
		goto probe_free_irq_cd;
	}

	host->clk = clk_get(dev, "sdi");
	if (IS_ERR(host->clk)) {
		printk(KERN_INFO PFX "failed to find clock source.\n");
		ret = PTR_ERR(host->clk);
		host->clk = NULL;
		goto probe_free_host;
	}

	if((ret = clk_use(host->clk))) {
		printk(KERN_INFO PFX "failed to use clock source.\n");
		goto clk_free;
	}

	if((ret = clk_enable(host->clk))) {
		printk(KERN_INFO PFX "failed to enable clock source.\n");
		goto clk_unuse;
	}


	mmc->ops 	= &s3c2410sdi_ops;
	mmc->ocr_avail	= MMC_VDD_32_33;
	mmc->flags      = MMC_HOST_WIDEMODE;
	mmc->f_min 	= clk_get_rate(host->clk) / 512;
	mmc->f_max 	= clk_get_rate(host->clk) / 2;


	//HACK: There seems to be a hardware bug in TomTom GO.
	if(mmc->f_max>3000000) mmc->f_max=3000000;


	/*
	 * Since we only have a 16-bit data length register, we must
	 * ensure that we don't exceed 2^16-1 bytes in a single request.
	 * Choose 64 (512-byte) sectors as the limit.
	 */
	mmc->max_sectors = 64;

	/*
	 * Set the maximum segment size.  Since we aren't doing DMA
	 * (yet) we are only limited by the data length register.
	 */

	mmc->max_seg_size = mmc->max_sectors << 9;
#ifdef S3C2410SDI_DMA_BACKBUF	
	host->dmabuf_log = dma_alloc_coherent(NULL, mmc->max_seg_size ,&host->dmabuf_phys, GFP_KERNEL|GFP_DMA);
	
	if(!host->dmabuf_log) {
		printk(KERN_INFO PFX "failed to allocate DMA buffer.\n");
		goto clk_disable;
	}
	
	host->dmabuf_size = mmc->max_seg_size;
	
	printk(KERN_INFO PFX "probe: mapped sdi_base=%p irq=%u irq_cd=%u dma=%u dmabuf_l=%p dmabuf_p=%p.\n", 
		host->base, host->irq, host->irq_cd, host->dma, host->dmabuf_log, host->dmabuf_phys);
#else
	printk(KERN_INFO PFX "probe: mapped sdi_base=%p irq=%u irq_cd=%u dma=%u.\n", 
		host->base, host->irq, host->irq_cd, host->dma);
#endif	
	if((ret = mmc_add_host(mmc))) {
		printk(KERN_INFO PFX "failed to add mmc host.\n");
		goto free_dmabuf;
	}

	dev_set_drvdata(dev, mmc);

	printk(KERN_INFO PFX "initialisation done.\n");
	return 0;
	
 free_dmabuf:
#ifdef S3C2410SDI_DMA_BACKBUF
 	dma_free_coherent(NULL, host->dmabuf_size, host->dmabuf_log, host->dmabuf_phys);
#endif

	clk_disable(host->clk);

 clk_unuse:
	clk_unuse(host->clk);

 clk_free:
	clk_put(host->clk);

 probe_free_irq_cd:
 	free_irq(host->irq_cd, host);

 probe_free_irq:
 	free_irq(host->irq, host);

 probe_iounmap:
	iounmap(host->base);

 probe_free_mem_region:
	release_mem_region(host->mem->start, RESSIZE(host->mem));

 probe_free_host:
	mmc_free_host(mmc);
 probe_out:
	return ret;
}

static int s3c2410sdi_remove(struct device *dev)
{
	struct mmc_host 	*mmc  = dev_get_drvdata(dev);
	struct s3c2410sdi_host 	*host = mmc_priv(mmc);

	mmc_remove_host(mmc);
#ifdef S3C2410SDI_DMA_BACKBUF
 	dma_free_coherent(NULL, host->dmabuf_size, host->dmabuf_log, host->dmabuf_phys);
#endif
	clk_disable(host->clk);
	clk_unuse(host->clk);
	clk_put(host->clk);
 	free_irq(host->irq_cd, host);
 	free_irq(host->irq, host);
	iounmap(host->base);
	release_mem_region(host->mem->start, RESSIZE(host->mem));
	mmc_free_host(mmc);

	return 0;
}

static struct device_driver s3c2410sdi_driver =
{
        .name           = "s3c2410-sdi",
        .bus            = &platform_bus_type,
        .probe          = s3c2410sdi_probe,
        .remove         = s3c2410sdi_remove,
};

static int __init s3c2410sdi_init(void)
{
	return driver_register(&s3c2410sdi_driver);
}

static void __exit s3c2410sdi_exit(void)
{
	driver_unregister(&s3c2410sdi_driver);
}

module_init(s3c2410sdi_init);
module_exit(s3c2410sdi_exit);

MODULE_DESCRIPTION("Samsung S3C2410 Multimedia Card Interface driver");
MODULE_LICENSE("GPL");
