#
# Makefile for USB Media drivers
#

sn9c102-objs	:= sn9c102_core.o sn9c102_hv7131d.o sn9c102_mi0343.o sn9c102_ov7630.o sn9c102_pas106b.o sn9c102_pas202bcb.o sn9c102_tas5110c1b.o sn9c102_tas5130d1b.o

obj-$(CONFIG_USB_DABUSB)	+= dabusb.o
obj-$(CONFIG_USB_DSBR)		+= dsbr100.o
obj-$(CONFIG_USB_IBMCAM)	+= ibmcam.o usbvideo.o ultracam.o
obj-$(CONFIG_USB_KONICAWC)	+= konicawc.o usbvideo.o
obj-$(CONFIG_USB_OV511)		+= ov511.o
obj-$(CONFIG_USB_SE401)		+= se401.o
obj-$(CONFIG_USB_SN9C102)	+= sn9c102.o
obj-$(CONFIG_USB_STV680)	+= stv680.o
obj-$(CONFIG_USB_VICAM)		+= vicam.o usbvideo.o
obj-$(CONFIG_USB_W9968CF)	+= w9968cf.o
obj-$(CONFIG_USB_PWC)           += pwc/
obj-$(CONFIG_USB_GSPCA)       += gspca/
