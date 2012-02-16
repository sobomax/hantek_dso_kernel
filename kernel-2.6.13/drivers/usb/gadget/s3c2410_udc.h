#ifndef _S3C2410_UDC_H
#define _S3C2410_UDC_H

struct s3c2410_ep {
	struct list_head		queue;
	unsigned long			last_io;	/* jiffies timestamp */
	struct usb_gadget		*gadget;
	struct s3c2410_udc		*dev;
	const struct usb_endpoint_descriptor *desc;
	struct usb_ep			ep;
	u8				num;

	unsigned short			fifo_size;
	u8				bEndpointAddress;
	u8				bmAttributes;

	unsigned			halted : 1;
	unsigned			already_seen : 1;
	unsigned			setup_stage : 1;
};


/* Warning : ep0 has a fifo of 16 bytes */
/* Don't try to set 32 or 64            */
#define EP0_FIFO_SIZE		16
#define EP_FIFO_SIZE		64
#define DEFAULT_POWER_STATE	0x00

static const char ep0name [] = "ep0";

static const char *const ep_name[] = {
	ep0name,                                /* everyone has ep0 */
	/* s3c2410 four bidirectional bulk endpoints */
	"ep1-bulk", "ep2-bulk", "ep3-bulk", "ep4-bulk",
};

#define S3C2410_ENDPOINTS       ARRAY_SIZE(ep_name)

struct s3c2410_request {
	struct list_head		queue;		/* ep's requests */
	struct usb_request		req;
};

enum ep0_state {
        EP0_IDLE,
        EP0_IN_DATA_PHASE,
        EP0_OUT_DATA_PHASE,
        EP0_END_XFER,
        EP0_STALL,
};

static const char *ep0states[]= {
        "EP0_IDLE",
        "EP0_IN_DATA_PHASE",
        "EP0_OUT_DATA_PHASE",
        "EP0_END_XFER",
        "EP0_STALL",
};

struct s3c2410_udc {
	spinlock_t			lock;

	struct s3c2410_ep		ep[S3C2410_ENDPOINTS];
	int				address;
	struct usb_gadget		gadget;
	struct usb_gadget_driver	*driver;
	struct s3c2410_request		fifo_req;
	u8				fifo_buf[EP_FIFO_SIZE];
	u16				devstatus;

	u32				port_status;
    	int 	    	    	    	ep0state;

	unsigned			got_irq : 1;

	unsigned			req_std : 1;
	unsigned			req_config : 1;
	unsigned			req_pending : 1;
};

/****************** MACROS ******************/
/* #define BIT_MASK	BIT_MASK*/
#define BIT_MASK	0xFF

#define maskb(base,v,m,a)      \
	        writeb((readb(base+a) & ~(m))|((v)&(m)), (base+a))

#define maskw(base,v,m,a)      \
	        writew((readw(base+a) & ~(m))|((v)&(m)), (base+a))

#define maskl(base,v,m,a)      \
	        writel((readl(base+a) & ~(m))|((v)&(m)), (base+a))

#define clear_ep0_sst(base) do {			\
    	S3C2410_UDC_SETIX(base,EP0); 			\
	writel(0x00, base+S3C2410_UDC_EP0_CSR_REG); 	\
} while(0)

#define clear_ep0_se(base) do {				\
    	S3C2410_UDC_SETIX(base,EP0); 			\
	maskl(base,S3C2410_UDC_EP0_CSR_SSE,		\
	    	BIT_MASK, S3C2410_UDC_EP0_CSR_REG); 	\
} while(0)

#define clear_ep0_opr(base) do {			\
   	S3C2410_UDC_SETIX(base,EP0);			\
	maskl(base,S3C2410_UDC_EP0_CSR_SOPKTRDY,	\
		BIT_MASK, S3C2410_UDC_EP0_CSR_REG); 	\
} while(0)

#define set_ep0_ipr(base) do {				\
   	S3C2410_UDC_SETIX(base,EP0);			\
	maskl(base,S3C2410_UDC_EP0_CSR_IPKRDY,		\
		BIT_MASK, S3C2410_UDC_EP0_CSR_REG); 	\
} while(0)

#define set_ep0_de(base) do {				\
   	S3C2410_UDC_SETIX(base,EP0);			\
	maskl(base,S3C2410_UDC_EP0_CSR_DE,		\
		BIT_MASK, S3C2410_UDC_EP0_CSR_REG);	\
} while(0)

#define set_ep0_ss(base) do {				\
   	S3C2410_UDC_SETIX(base,EP0);				\
	maskl(base,S3C2410_UDC_EP0_CSR_SENDSTL|S3C2410_UDC_EP0_CSR_SOPKTRDY,	\
		BIT_MASK, S3C2410_UDC_EP0_CSR_REG);	\
} while(0)

#define set_ep0_de_out(base) do {			\
   	S3C2410_UDC_SETIX(base,EP0);			\
	maskl(base,(S3C2410_UDC_EP0_CSR_SOPKTRDY 	\
		| S3C2410_UDC_EP0_CSR_DE),		\
		BIT_MASK, S3C2410_UDC_EP0_CSR_REG);	\
} while(0)

#define set_ep0_sse_out(base) do {			\
   	S3C2410_UDC_SETIX(base,EP0);			\
	maskl(base,(S3C2410_UDC_EP0_CSR_SOPKTRDY 	\
		| S3C2410_UDC_EP0_CSR_SSE),		\
		BIT_MASK, S3C2410_UDC_EP0_CSR_REG);	\
} while(0)

#define set_ep0_de_in(base) do {			\
   	S3C2410_UDC_SETIX(base,EP0);			\
	maskl(base,(S3C2410_UDC_EP0_CSR_IPKRDY		\
		| S3C2410_UDC_EP0_CSR_DE),		\
		BIT_MASK, S3C2410_UDC_EP0_CSR_REG);		\
} while(0)



#define clear_stall_ep1_out(base) do {			\
   	S3C2410_UDC_SETIX(base,EP1);			\
	orl(0,base+S3C2410_UDC_OUT_CSR1_REG);		\
} while(0)


#define clear_stall_ep2_out(base) do {			\
   	S3C2410_UDC_SETIX(base,EP2);			\
	orl(0, base+S3C2410_UDC_OUT_CSR1_REG);		\
} while(0)


#define clear_stall_ep3_out(base) do {			\
   	S3C2410_UDC_SETIX(base,EP3);			\
	orl(0,base+S3C2410_UDC_OUT_CSR1_REG);		\
} while(0)


#define clear_stall_ep4_out(base) do {			\
   	S3C2410_UDC_SETIX(base,EP4);			\
	orl(0, base+S3C2410_UDC_OUT_CSR1_REG);		\
} while(0)

#endif


