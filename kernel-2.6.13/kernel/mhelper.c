
#include <linux/config.h>
#include <linux/linkage.h>
#include <asm/errno.h>

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <asm/mach/map.h>

#include "mhelper.h"



#define IODESC_ENT(x) { (unsigned long)S3C24XX_VA_##x, S3C2410_PA_##x, S3C24XX_SZ_##x, MT_DEVICE }

struct map_desc mhelper_iodesc[] = {
        IODESC_ENT(IRQ),
        IODESC_ENT(MEMCTRL),
        IODESC_ENT(USBHOST),
        IODESC_ENT(DMA),
        IODESC_ENT(CLKPWR),
        IODESC_ENT(LCD),
        IODESC_ENT(NAND),
        IODESC_ENT(UART),
        IODESC_ENT(TIMER),
        IODESC_ENT(USBDEV),
        IODESC_ENT(WATCHDOG),
        IODESC_ENT(IIC),
        IODESC_ENT(IIS),
        IODESC_ENT(GPIO),
        IODESC_ENT(RTC),
        IODESC_ENT(ADC),
        IODESC_ENT(SPI),
        IODESC_ENT(SDI),
};

#define       IODESC_SIZE     (sizeof(mhelper_iodesc)/sizeof(struct map_desc))
	
static inline unsigned long
map_io_to_va(unsigned long addr)
{
	int index;

      for (index=0; index<IODESC_SIZE; index++) {
               unsigned long phys = mhelper_iodesc[index].physical;
               unsigned long len = mhelper_iodesc[index].length;

		if ((addr >= phys) & (addr < phys+len))
			break;
	}
      if (index < IODESC_SIZE) {
               unsigned long phys = mhelper_iodesc[index].physical;
               unsigned long virt = mhelper_iodesc[index].virtual;

		return (addr - phys + virt);
	}
	return 0;
}


extern asmlinkage long
sys_mhelper(uint32_t cmd, uint32_t addr, uint32_t value)
{
	unsigned long vaddr = 0;
	int verb;

	if (!(vaddr = map_io_to_va(addr)))
		return -ENXIO;


	verb = (cmd & OPT_VERB);	
	cmd &= ~OPT_VERB;

        switch (cmd) {

	case CMD_READ_B:
		value = __raw_readb(vaddr);
		if (verb)
			printk("0x%08x: 0x%02x (%d)\n", 
				addr, value, value);
		break;
	case CMD_READ_W:
		value = __raw_readw(vaddr);
		if (verb)
			printk("0x%08x: 0x%04x (%d)\n", 
				addr, value, value);
		break;
	case CMD_READ_L:
		value = __raw_readl(vaddr);
		if (verb)
			printk("0x%08x: 0x%08x (%d)\n", 
				addr, value, value);
		break;

	case CMD_WRITE_B:
		__raw_writeb(value, vaddr);
		break;
	case CMD_WRITE_W:
		__raw_writew(value, vaddr);
		break;
	case CMD_WRITE_L:
		__raw_writel(value, vaddr);
		break;

	default:
		return -EINVAL;
		break;
	}

	return verb?0:value;
}

