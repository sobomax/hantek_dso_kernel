#
# Makefile for the kernel mmc device drivers.
#

#
# Core
#
obj-$(CONFIG_MMC)		+= mmc_core.o

#
# Media drivers
#
obj-$(CONFIG_MMC_BLOCK)		+= mmc_block.o

#
# Host drivers
#
obj-$(CONFIG_MMC_ARMMMCI)	+= mmci.o
obj-$(CONFIG_MMC_PXA)		+= pxamci.o
obj-$(CONFIG_MMC_WBSD)		+= wbsd.o
obj-$(CONFIG_MMC_S3C2410)	+= s3c2410mci.o
obj-$(CONFIG_MMC_S3C2440)	+= s3c2440mci.o

mmc_core-y := mmc.o mmc_queue.o mmc_sysfs.o
