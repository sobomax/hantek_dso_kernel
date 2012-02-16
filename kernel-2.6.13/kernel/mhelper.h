
#define CAT_READ	0x10
#define CAT_WRITE	0x20

#define CAT_BYTE	0x01
#define CAT_WORD	0x02
#define CAT_LONG	0x03

#define	OPT_VERB	0x80


enum {
	CMD_VERSION = 0,

	CMD_READ_B  = 0x11,
	CMD_READ_W,
	CMD_READ_L,

	CMD_WRITE_B = 0x21,
	CMD_WRITE_W,
	CMD_WRITE_L,

};

