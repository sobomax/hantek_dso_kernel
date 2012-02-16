/*
 *  linux/drivers/mmc/s3c2410mci.h - Samsung S3C2410 SDI Interface driver
 *
 *  Copyright (C) 2004 Thomas Kleffel, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 
 
 2005.07.09 godori from www.aesop-embedded.org(ghcstop@empal.com)
            porting sdi interface(DMA mode) to S3C2440A
            ==> irq mode?: see s3c2410mci.c.irq_mode_single_block_fault 
                           & this file's s3c2410sdi_irq()(fsta & dsta control routine)
            DMA mode transfer rate(10MBytes file tranfer time is 7 sec with single block mode)
            ==> HIGH system load with media player
 
 2005.07.10 godori from www.aesop-embedded.org(ghcstop@empal.com)
            porting sdi interface(IRQ mode) to S3C2440A
            ==> dma mode?: s3c2410mci.c.dma
            IRQ mode transfer rate: slow than DMA mode
            ==> Low system load with media player: 
 
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
	
	#ifdef CONFIG_CPU_S3C2440
		u32 sdi_fsta;
		int i, txfifocnt, datalen;
		u32 sdi_data;
	#endif	

	host = (struct s3c2410sdi_host *)dev_id;

	//Check for things not supposed to happen
	if(!host) return IRQ_HANDLED;
	
	sdi_csta 	= readl(host->base + S3C2410_SDICMDSTAT);
	sdi_dsta 	= readl(host->base + S3C2410_SDIDSTA);
	sdi_dcnt 	= readl(host->base + S3C2410_SDIDCNT);
	#ifdef CONFIG_CPU_S3C2440
		sdi_fsta = readl(host->base + S3C2410_SDIFSTA);
	#endif
	
		
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

	#ifdef CONFIG_CPU_S3C2440
		if( sdi_fsta & S3C2410_SDIFSTA_FF_FAIL ) 
		{
			host->mrq->cmd->error = MMC_ERR_NONE;
			host->mrq->data->error = MMC_ERR_FIFO;
			goto transfer_closed;
		}
		else if(sdi_fsta & S3C2410_SDIFSTA_FF_LASTXFER) 
		{
			host->mrq->cmd->error = MMC_ERR_NONE;
			host->mrq->data->error = MMC_ERR_FIFO;
    		sdi_fsta |= S3C2410_SDIFSTA_FIFORESET;
    		writel(sdi_fsta, host->base + S3C2410_SDIFSTA);
			goto transfer_closed;
		}
	#else
		if(sdi_dsta & S3C2410_SDIDSTA_FIFOFAIL) {
			host->mrq->cmd->error = MMC_ERR_NONE;
			host->mrq->data->error = MMC_ERR_FIFO;
			goto transfer_closed;
		}
	#endif

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

    if( host->mrq->data ) 
    {
    	datalen = host->mrq->data->blocks << host->mrq->data->blksz_bits; 
		if( host->mrq->data->flags & MMC_DATA_READ ) 
		{
		    if( (sdi_fsta&S3C2410_SDIFSTA_RFLAST) == S3C2410_SDIFSTA_RFLAST )
        	{
				for(i=(sdi_fsta & 0x7f)/4;i>0;i--) 
				{
					sdi_data = readl(host->base + S3C2410_SDIDATA);
					*((u32 *)(host->mrq->data->req->buffer + host->mrq->data->bytes_process)) = sdi_data;
					host->mrq->data->bytes_process += 4;
				}
				
				
				if(host->complete_what == COMPLETION_XFERFINISH) {
					host->mrq->cmd->error = MMC_ERR_NONE;
					host->mrq->data->error = MMC_ERR_NONE;
			
					sdi_fsta = 0;
   	   				sdi_fsta |= S3C2410_SDIFSTA_RFLAST;
					writel(sdi_fsta, host->base + S3C2410_SDIFSTA);
			
					sdi_dsta   = 0;
					sdi_dsta |= S3C2410_SDIDSTA_XFERFINISH;
					writel(sdi_dsta, host->base + S3C2410_SDIDSTA);
			
					goto transfer_closed;
				}
        
        	}
        	else if( (sdi_fsta&S3C2410_SDIFSTA_RFFULL) == S3C2410_SDIFSTA_RFFULL )
        	{
        		for(i=0;i<8;i++)
        		{
					sdi_data = readl(host->base + S3C2410_SDIDATA);
					*((u32 *)(host->mrq->data->req->buffer + host->mrq->data->bytes_process)) = sdi_data;
					host->mrq->data->bytes_process += 4;
				}
        	}
        }
        else 
        {
		    if (sdi_fsta & S3C2410_SDIFSTA_TXEMPTY) 
		    {
            	txfifocnt = 16; 
            	
            	while (txfifocnt && host->mrq->data->bytes_process < datalen)
            	{
                	sdi_data = *((unsigned int *)(host->mrq->data->req->buffer + host->mrq->data->bytes_process));
                	writel(sdi_data, host->base + S3C2410_SDIDATA);
                	host->mrq->data->bytes_process += 4;
                	txfifocnt--;
            	}
        	}
        }
    }
	


	if(sdi_dsta & S3C2410_SDIDSTA_XFERFINISH) {
		if(host->complete_what == COMPLETION_XFERFINISH) {
			host->mrq->cmd->error = MMC_ERR_NONE;
			host->mrq->data->error = MMC_ERR_NONE;
			
			sdi_fsta = 0;
   			sdi_fsta |= S3C2410_SDIFSTA_RFLAST;
	    	writel(sdi_fsta, host->base + S3C2410_SDIFSTA);
	    	
	    	sdi_dsta   = 0;
	    	sdi_dsta |= S3C2410_SDIDSTA_XFERFINISH;
	    	writel(sdi_dsta, host->base + S3C2410_SDIDSTA);
			
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
	return IRQ_HANDLED;


transfer_closed:
	host->complete_what = COMPLETION_NONE;
	complete(&host->complete_request);
	writel(0, host->base + S3C2410_SDIIMSK);
	spin_unlock_irqrestore( &host->complete_lock, iflags);
	return IRQ_HANDLED;
	
clear_imask:
	writel(0, host->base + S3C2410_SDIIMSK);
	spin_unlock_irqrestore( &host->complete_lock, iflags);
	return IRQ_HANDLED;

}


/*
 * ISR for the CardDetect Pin
*/

static irqreturn_t s3c2410sdi_irq_cd(int irq, void *dev_id, struct pt_regs *regs)
{
	struct s3c2410sdi_host *host = (struct s3c2410sdi_host *)dev_id;
	mmc_detect_change(host->mmc);

//printk("ghc: interrupt occur ***********************\n");

	return IRQ_HANDLED;
}




static void s3c2410sdi_request(struct mmc_host *mmc, struct mmc_request *mrq) {
 	struct s3c2410sdi_host *host = mmc_priv(mmc);
	u32 sdi_carg, sdi_ccon, sdi_timer;
	u32 sdi_bsize, sdi_dcon, sdi_imsk, sdi_fsta;


	sdi_ccon = mrq->cmd->opcode & S3C2410_SDICMDCON_INDEX;
	sdi_ccon|= S3C2410_SDICMDCON_SENDERHOST;
	sdi_ccon|= S3C2410_SDICMDCON_CMDSTART;

	sdi_carg = mrq->cmd->arg;

	#ifdef CONFIG_ARCH_SMDK2410
	    //FIXME: Timer value ?!
	    sdi_timer= 0xF000;
	#elif defined(CONFIG_CPU_S3C2440)
	    sdi_timer= 0x7fffff;
	#endif
	

	sdi_bsize= 0;
	sdi_dcon = 0;
	sdi_imsk = 0;

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
		
		sdi_dcon &= ~S3C2410_SDIDCON_DMAEN; 
		mrq->data->bytes_process = 0; 
		
		sdi_imsk |= S3C2410_SDIIMSK_FIFOFAIL;
		sdi_imsk |= S3C2410_SDIIMSK_DATACRC;
		sdi_imsk |= S3C2410_SDIIMSK_DATATIMEOUT;
		sdi_imsk |= S3C2410_SDIIMSK_DATAFINISH;
		sdi_imsk |= 0xFFFFFFE0;

		if(mrq->data->flags & MMC_DATA_WIDE) {
			sdi_dcon |= S3C2410_SDIDCON_WIDEBUS;
		}
		
		if(!(mrq->data->flags & MMC_DATA_STREAM)) {
			sdi_dcon |= S3C2410_SDIDCON_BLOCKMODE;
		}

		if(mrq->data->flags & MMC_DATA_WRITE) {
			sdi_dcon |= S3C2410_SDIDCON_TXAFTERRESP;
			sdi_dcon |= S3C2410_SDIDCON_XFER_TXSTART;
			
			sdi_dcon |= S3C2410_SDIDCON_DSIZE_WORD; 
            sdi_dcon |= S3C2410_SDIDCON_TRANSSTART; 
			
			sdi_imsk |= S3C2410_SDIIMSK_TXFIFOEMPTY;
		}

		if(mrq->data->flags & MMC_DATA_READ) {
			sdi_dcon |= S3C2410_SDIDCON_RXAFTERCMD;
			sdi_dcon |= S3C2410_SDIDCON_XFER_RXSTART;
			
			sdi_dcon |= S3C2410_SDIDCON_DSIZE_WORD; 
            sdi_dcon |= S3C2410_SDIDCON_TRANSSTART; 
			
			sdi_imsk |= S3C2410_SDIIMSK_RXFIFOLAST;
			sdi_imsk |= S3C2410_SDIIMSK_RXFIFOFULL;
		}
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

	// If we have no data transfer we are finished here
	if (!mrq->data) goto request_done;

	// Calulate the amout of bytes transfer, but only if there was
	// no error
	if(mrq->data->error == MMC_ERR_NONE) {
		mrq->data->bytes_xfered = (mrq->data->blocks << mrq->data->blksz_bits);
		if(mrq->data->flags & MMC_DATA_READ);
	} else {
		mrq->data->bytes_xfered = 0;
	}

	// If we had an error while transfering data we reset the
	// FIFO to clear out any garbage
	if(mrq->data->error != MMC_ERR_NONE) {
		sdi_fsta = 0;
   		sdi_fsta |= S3C2410_SDIFSTA_FIFORESET;
   		writel(sdi_fsta, host->base + S3C2410_SDIFSTA);
	}
	// Issue stop command
	if(mrq->data->stop) mmc_wait_for_cmd(mmc, mrq->data->stop, 3);


request_done:
	mrq->done(mrq);
}

static void s3c2410sdi_set_ios(struct mmc_host *mmc, struct mmc_ios *ios) {
	struct s3c2410sdi_host *host = mmc_priv(mmc);
	
	#ifdef CONFIG_ARCH_SMDK2410 
		u32 sdi_psc, sdi_con;
	#elif defined(CONFIG_CPU_S3C2440)
		u32 sdi_psc, sdi_con, sdi_fsta;
	#endif

	//Set power
	#ifdef CONFIG_ARCH_SMDK2410 
		sdi_con = readl(host->base + S3C2410_SDICON);
	#elif defined(CONFIG_CPU_S3C2440)
		sdi_con  = readl(host->base + S3C2410_SDICON);
		sdi_fsta = readl(host->base + S3C2410_SDIFSTA);
	#endif
	
	switch(ios->power_mode) {
		case MMC_POWER_ON:
		case MMC_POWER_UP:

			s3c2410_gpio_cfgpin(S3C2410_GPE5, S3C2410_GPE5_SDCLK);
			s3c2410_gpio_cfgpin(S3C2410_GPE6, S3C2410_GPE6_SDCMD);
			s3c2410_gpio_cfgpin(S3C2410_GPE7, S3C2410_GPE7_SDDAT0);
			s3c2410_gpio_cfgpin(S3C2410_GPE8, S3C2410_GPE8_SDDAT1);
			s3c2410_gpio_cfgpin(S3C2410_GPE9, S3C2410_GPE9_SDDAT2);
			s3c2410_gpio_cfgpin(S3C2410_GPE10, S3C2410_GPE10_SDDAT3);

			#ifdef CONFIG_ARCH_SMDK2410 
	    		sdi_con|= S3C2410_SDICON_FIFORESET;
			#elif defined(CONFIG_CPU_S3C2440)
	    		sdi_fsta |= S3C2410_SDIFSTA_FIFORESET;
			#endif
	
			
			break;

		case MMC_POWER_OFF:
		default:
			s3c2410_gpio_setpin(S3C2410_GPE5, 0);
			s3c2410_gpio_cfgpin(S3C2410_GPE5, S3C2410_GPE5_OUTP);
			break;
	}

	for(sdi_psc=0;sdi_psc<255;sdi_psc++) {
		if( (clk_get_rate(host->clk) / (2*(sdi_psc+1))) <= ios->clock) break;
	}

	if(sdi_psc > 255) sdi_psc = 255;
	writel(sdi_psc, host->base + S3C2410_SDIPRE);

	#ifdef CONFIG_ARCH_SMDK2410 
		if(ios->clock) 	sdi_con |= S3C2410_SDICON_CLOCKTYPE;
		else		sdi_con &=~S3C2410_SDICON_CLOCKTYPE;
		writel(sdi_con, host->base + S3C2410_SDICON);
	#elif defined(CONFIG_CPU_S3C2440)
		if(ios->clock) 	
		{
			sdi_con &= ~S3C2410_SDICON_CLOCKTYPE; 
			sdi_con |= S3C2410_SDICON_ENCLK;
		}
		else		
		{
			sdi_con &=~S3C2410_SDICON_CLOCKTYPE;
			sdi_con &=~S3C2410_SDICON_ENCLK;
		}
	
   		writel(sdi_con, host->base + S3C2410_SDICON);
   		writel(sdi_fsta, host->base + S3C2410_SDIFSTA);
	#endif


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
	
	host->irq_cd		= IRQ_EINT16; // ghcstop, card detect irq


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

	#if 0 // ghcstop
	s3c2410_gpio_cfgpin(S3C2410_GPF2, S3C2410_GPF2_EINT2);
	set_irq_type(host->irq_cd, IRQT_BOTHEDGE);
	#else // aesop board
	s3c2410_gpio_cfgpin(S3C2410_GPG8, S3C2410_GPG8_EINT16);
	set_irq_type(host->irq_cd, IRQT_BOTHEDGE);
	#endif
	

	if(request_irq(host->irq_cd, s3c2410sdi_irq_cd, 0, DRIVER_NAME, host)) {
		printk(KERN_WARNING PFX "failed to request card detect interrupt.\n" );
		ret = -ENOENT;
		goto probe_free_irq;
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
	printk(KERN_INFO PFX "probe: mapped sdi_base=%p irq=%u irq_cd=%u dma=%u.\n", 
		host->base, host->irq, host->irq_cd, host->dma);

	if((ret = mmc_add_host(mmc))) {
		printk(KERN_INFO PFX "failed to add mmc host.\n");
		goto free_dmabuf;
	}

	dev_set_drvdata(dev, mmc);

	printk(KERN_INFO PFX "initialisation done.\n");
	return 0;
	
 free_dmabuf:

	clk_disable(host->clk);

 clk_unuse:
	clk_unuse(host->clk);

 clk_free:
	clk_put(host->clk);

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
