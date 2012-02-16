/* linux/include/asm-arm/arch-s3c2410/aesop-map.h */

/* Date: 2005.05.03
 * This source file is written by DaeHo Jung<aplus43@hanmail.net>
 * -jdh-
 */  

#ifndef __ASM_ARCH_AESOPMAP_H
#define __ASM_ARCH_AESOPMAP_H

#define AESOP_VA_IOBASE	0xF8000000

#define AESOP_VA_CS8900A    AESOP_VA_IOBASE	 	/* 0xF8000000 */
#define AESOP_PA_CS8900A    (S3C2410_CS3 + 0x1000000)	/* 0x19000000 */
#define AESOP_SZ_CS8900A    SZ_1M

#endif /* __ASM_ARCH_AESOPMAP_H */
