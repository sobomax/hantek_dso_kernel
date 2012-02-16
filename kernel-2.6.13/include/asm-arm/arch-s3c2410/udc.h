/* linux/include/asm/arch-s3c2410/udc.h
 *
 * Copyright (c) 2005 Arnaud Patard <arnaud.patard@rtp-net.org>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 *  Changelog:
 *     14-Mar-2005     RTP     Created file
 *     02-Aug-2005     RTP     File rename
 */

#ifndef __ASM_ARM_UDC_H
#define __ASM_ARM_UDC_H

#include <asm/arch/regs-udc.h>

#define S3C2410_UDC_P_ENABLE	1	/* Pull-up enable        */
#define S3C2410_UDC_P_DISABLE	2	/* Pull-up disable       */
#define S3C2410_UDC_P_RESET	3	/* UDC reset, in case of */

struct s3c2410_udc_mach_info {
	void            (*udc_command)(unsigned char);

};

void __init set_s3c2410udc_info(struct s3c2410_udc_mach_info *hard_s3c2410udc_info);

#endif /* __ASM_ARM_UDC_H */
