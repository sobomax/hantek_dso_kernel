/*
 * 
 * (C) Samsung Electronics 2004
 *
 * Philips UDA1341 Audio Device Driver for SMDK board
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 * 2004-04-28 Kwanghyun La <nala.la@samsung.com>
 *   - modified for sharing module device driver of samsung arch
 *
 * 2004-07: SW.LEE
 *         comment : Originally made by MIZI Research For S3C2410
 *	            ported to S3C2440A
 
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/l3/l3.h>
#include <linux/l3/uda1341.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/semaphore.h>
#include <asm/dma.h>
#include <asm/hardware/clock.h>
#include <asm/arch/dma.h>
#include <asm/arch/regs-iis.h>

#include "qq2440-audio.h"


#undef USE_SYSFS  
#define USE_SYSFS 

#ifdef  GDEBUG
#    define dprintk( x... )  printk( x )
#else
#    define dprintk( x... )
#endif

#define AUDIO_NAME		"SBC2440_UDA1341"
#define AUDIO_NAME_VERBOSE	"SBC2440 UDA1341 audio driver"

#define AUDIO_FMT_MASK          (AFMT_S16_LE)
#define AUDIO_FMT_DEFAULT       (AFMT_S16_LE)

/* the UDA1341 is stereo only */
#define AUDIO_CHANNELS_DEFAULT	2
#define AUDIO_RATE_DEFAULT	44100

#define AUDIO_NBFRAGS_DEFAULT	8
//#define AUDIO_FRAGSIZE_DEFAULT    8192
#define AUDIO_FRAGSIZE_DEFAULT	16384

typedef struct
{
    int             size;       /* buffer size */
    char           *start;      /* point to actual buffer */
    dma_addr_t      dma_addr;   /* physical buffer address */
    struct semaphore sem;       /* down before touching the buffer */
    int             master;     /* owner for buffer allocation, contain size when true */
} audio_buf_t;


typedef struct
{
    audio_buf_t    *buffers;    /* pointer to audio buffer structures */
    audio_buf_t    *buf;        /* current buffer used by read/write */
    u_int           buf_idx;    /* index for the pointer above */
    u_int           fragsize;   /* fragment i.e. buffer size */
    u_int           nbfrags;    /* nbr of fragments */
    int             bytecount;  /* nbr of processed bytes */
    int             fragcount;  /* nbr of fragment transitions */
    u_int           channels;   /* audio channels 1:mono, 2:stereo */
    u_int           rate;       /* audio rate */
    dmach_t         dma_ch;     /* DMA channel (channel2 for audio) */
    int             active:1;   /* actually in progress */
    int             stopped:1;  /* might be active but stopped */
    wait_queue_head_t frag_wq;  /* for poll(), etc. */
    s3c2410_dma_client_t dmaclient; /* kernel 2.6 dma client */
} audio_stream_t;

/*
 * Mixer (UDA1341) interface 
 */
static struct l3_client uda1341;

static audio_stream_t output_stream;
static audio_stream_t input_stream;

static int ao_dcon = 0, ai_dcon = 0;



#define NEXT_BUF(_s_,_b_) { \
	(_s_)->_b_##_idx++; \
	(_s_)->_b_##_idx %= (_s_)->nbfrags; \
	(_s_)->_b_ = (_s_)->buffers + (_s_)->_b_##_idx; }

static u_int    audio_rate;
static int      audio_channels;
static int      audio_fmt;

//static u_int audio_fragsize;
//static u_int audio_nbfrags;

static int      audio_rd_refcount;
static int      audio_wr_refcount;

#define audio_active		(audio_rd_refcount | audio_wr_refcount)


static void     start_sbc2440_iis_bus_tx(void);
static void     start_sbc2440_iis_bus_rx(void);



static void
audio_clear_buf(audio_stream_t * s)
{
    dprintk("\n");

    s->active = 0;
    s->stopped = 0;

    /*
     * ensure DMA won't run anymore 
     */
    //sbc2440_dma_flush_all(s->dma_ch);
    s3c2410_dma_ctrl(s->dma_ch, S3C2410_DMAOP_FLUSH);

    if (s->buffers)
    {
        int             frag;

        for (frag = 0; frag < s->nbfrags; frag++)
        {
            if (!s->buffers[frag].master)
                continue;
            dma_free_coherent( NULL, s->buffers[frag].master, s->buffers[frag].start,  s->buffers[frag].dma_addr);
        }
        kfree(s->buffers);
        s->buffers = NULL;
    }

    s->buf_idx = 0;
    s->buf = NULL;
}

/*
 * This function allocates the buffer structure array and buffer data space
 * according to the current number of fragments and fragment size.
 */
static int
audio_setup_buf(audio_stream_t * s)
{
    int             frag;
    int             dmasize = 0;
    char           *dmabuf = NULL;
    dma_addr_t      dmaphys = 0;

    if (s->buffers)
        return -EBUSY;

//printk("audio_setup_buf:=================\n"); 
    s->buffers = kmalloc(sizeof(audio_buf_t) * s->nbfrags, GFP_KERNEL);
    if (!s->buffers)
        goto err;
    memzero(s->buffers, sizeof(audio_buf_t) * s->nbfrags);

    for (frag = 0; frag < s->nbfrags; frag++)
    {
        audio_buf_t    *b = &s->buffers[frag];

        /*
         * Let's allocate non-cached memory for DMA buffers.
         * We try to allocate all memory at once.
         * If this fails (a common reason is memory fragmentation),
         * then we allocate more smaller buffers.
         */
        if (!dmasize)
        {
            dmasize = (s->nbfrags - frag) * s->fragsize;
            do
            {
            	dmabuf = dma_alloc_coherent(NULL, dmasize, &dmaphys, GFP_KERNEL);
                if (!dmabuf)
                    dmasize -= s->fragsize;
            }
            while (!dmabuf && dmasize);
            if (!dmabuf)
                goto err;
            b->master = dmasize;
            memzero(dmabuf, dmasize);
        }

        b->start = dmabuf;
        b->dma_addr = dmaphys;
        // b->stream = s;
        sema_init(&b->sem, 1);
        dprintk("buf %d: start %p dma %ld\n", frag, b->start, (unsigned long)b->dma_addr);

        dmabuf += s->fragsize;
        dmaphys += s->fragsize;
        dmasize -= s->fragsize;
    }

    s->buf_idx = 0;
    s->buf = &s->buffers[0];

    return 0;

  err:
    printk(AUDIO_NAME ": unable to allocate audio memory\n ");
    audio_clear_buf(s);
    return -ENOMEM;
}


/*
 * This function yanks all buffers from the DMA code's control and
 * resets them ready to be used again.
 */

static void
audio_reset_buf(audio_stream_t * s)
{
    int             frag;

    s->active = 0;
    s->stopped = 0;
    s3c2410_dma_ctrl(s->dma_ch, S3C2410_DMAOP_FLUSH);
    if (s->buffers)
    {
        for (frag = 0; frag < s->nbfrags; frag++)
        {
            audio_buf_t    *b = &s->buffers[frag];

            b->size = 0;
            sema_init(&b->sem, 1);
        }
    }
    s->bytecount = 0;
    s->fragcount = 0;
}

static void
audio_dmaout_done_callback(s3c2410_dma_chan_t *chp, void *buf, int size,  s3c2410_dma_buffresult_t result)
{
    audio_buf_t    *b = (audio_buf_t *) buf;

    up(&b->sem);
    wake_up(&output_stream.frag_wq);

	if( result != S3C2410_RES_OK )
	{
		dprintk("%s: tranter error\n", __FUNCTION__);
	}

}


static void
audio_dmain_done_callback(s3c2410_dma_chan_t *chp, void *buf, int size,  s3c2410_dma_buffresult_t result)
{
    audio_buf_t    *b = (audio_buf_t *) buf;

    b->size = size;
    up(&b->sem);
    wake_up(&input_stream.frag_wq);
	
	if( result != S3C2410_RES_OK )
	{
		dprintk("%s: tranter error\n", __FUNCTION__);
	}
}


static int
audio_sync(struct file *file)
{
    audio_stream_t *s = &output_stream;
    audio_buf_t    *b = s->buf;

    dprintk("audio_sync\n");

    if (!s->buffers)
        return 0;

    if (b->size != 0)
    {
        down(&b->sem);
        s3c2410_dma_enqueue(s->dma_ch, (void *) b, b->dma_addr, b->size);
        b->size = 0;
        NEXT_BUF(s, buf);
    }

    b = s->buffers + ((s->nbfrags + s->buf_idx - 1) % s->nbfrags);
    if (down_interruptible(&b->sem))
        return -EINTR;
    up(&b->sem);

    return 0;
}

static inline int
copy_from_user_mono_stereo(char *to, const char *from, int count)
{
    u_int          *dst = (u_int *) to;
    const char     *end = from + count;

    if( !access_ok(VERIFY_READ, from, count) ) // access_ok가 0이면
        return -EFAULT;

    if ((int) from & 0x2)
    {
        u_int           v;

        __get_user(v, (const u_short *) from);
        from += 2;
        *dst++ = v | (v << 16);
    }

    while (from < end - 2)
    {
        u_int           v,
                        x,
                        y;

        __get_user(v, (const u_int *) from);
        from += 4;
        x = v << 16;
        x |= x >> 16;
        y = v >> 16;
        y |= y << 16;
        *dst++ = x;
        *dst++ = y;
    }

    if (from < end)
    {
        u_int           v;

        __get_user(v, (const u_short *) from);
        *dst = v | (v << 16);
    }

    return 0;
}

static          ssize_t
sbc2440_audio_write(struct file *file, const char *buffer, size_t count, loff_t * ppos)
{
    const char     *buffer0 = buffer;
    audio_stream_t *s = &output_stream;
    int             chunksize,
                    ret = 0;

    dprintk("audio_write : start count=%d\n", count);

    switch (file->f_flags & O_ACCMODE)
    {
    case O_WRONLY:
    case O_RDWR:
        break;
    default:
        return -EPERM;
    }

    if (!s->buffers && audio_setup_buf(s))
        return -ENOMEM;

    count &= ~0x03;

    while (count > 0)
    {
        audio_buf_t    *b = s->buf;

        if (file->f_flags & O_NONBLOCK)
        {
            ret = -EAGAIN;
            if (down_trylock(&b->sem))
                break;
        }
        else
        {
            ret = -ERESTARTSYS;
            if (down_interruptible(&b->sem))
                break;
        }

        if (s->channels == 2)
        {
            chunksize = s->fragsize - b->size;
            if (chunksize > count)
                chunksize = count;
            dprintk("write %d to %d\n", chunksize, s->buf_idx);
            if (copy_from_user(b->start + b->size, buffer, chunksize))
            {
                up(&b->sem);
                return -EFAULT;
            }
            b->size += chunksize;
        }
        else
        {
            chunksize = (s->fragsize - b->size) >> 1;

            if (chunksize > count)
                chunksize = count;
            dprintk("write %d to %d\n", chunksize * 2, s->buf_idx);
            if (copy_from_user_mono_stereo(b->start + b->size, buffer, chunksize))
            {
                up(&b->sem);
                return -EFAULT;
            }

            b->size += chunksize * 2;
        }

        buffer += chunksize;
        count -= chunksize;
        if (b->size < s->fragsize)
        {
            up(&b->sem);
            break;
        }
        s->active = 1;          // ghcstop add
        s3c2410_dma_enqueue(s->dma_ch, (void *) b, b->dma_addr, b->size);
       
        b->size = 0;
        NEXT_BUF(s, buf);
    }

    if ((buffer - buffer0))
        ret = buffer - buffer0;

    dprintk("audio_write : end count=%d\n\n", ret);

    return ret;
}

static          ssize_t
sbc2440_audio_read(struct file *file, char *buffer, size_t count, loff_t * ppos)
{
    const char     *buffer0 = buffer;
    audio_stream_t *s = &input_stream;
    int             chunksize,
                    ret = 0;

    dprintk("audio_read: count=%d\n", count);

    if (ppos != &file->f_pos)
        return -ESPIPE;

    if (!s->buffers)
    {
        int             i;

        if (audio_setup_buf(s))
            return -ENOMEM;

        for (i = 0; i < s->nbfrags; i++)
        {
            audio_buf_t    *b = s->buf;

            down(&b->sem);
            s3c2410_dma_enqueue(s->dma_ch, (void *) b, b->dma_addr, s->fragsize);
            NEXT_BUF(s, buf);
        }
    }

    while (count > 0)
    {
        audio_buf_t    *b = s->buf;

        /*
         * Wait for a buffer to become full 
         */
        if (file->f_flags & O_NONBLOCK)
        {
            ret = -EAGAIN;
            if (down_trylock(&b->sem))
                break;
        }
        else
        {
            ret = -ERESTARTSYS;
            if (down_interruptible(&b->sem))
                break;
        }

        chunksize = b->size;
        if (chunksize > count)
            chunksize = count;
        dprintk("read %d from %d\n", chunksize, s->buf_idx);
        if (copy_to_user(buffer, b->start + s->fragsize - b->size, chunksize))
        {
            up(&b->sem);
            return -EFAULT;
        }

        b->size -= chunksize;

        buffer += chunksize;
        count -= chunksize;
        if (b->size > 0)
        {
            up(&b->sem);
            break;
        }

        /*
         * Make current buffer available for DMA again 
         */
        s3c2410_dma_enqueue(s->dma_ch, (void *) b, b->dma_addr, s->fragsize);

        NEXT_BUF(s, buf);
    }

    if ((buffer - buffer0))
        ret = buffer - buffer0;

    dprintk("audio_read: return=%d\n", ret);

    return ret;
}

static unsigned int
sbc2440_audio_poll(struct file *file, struct poll_table_struct *wait)
{
    unsigned int    mask = 0;
    int             i;

    dprintk("audio_poll(): mode=%s\n", (file->f_mode & FMODE_WRITE) ? "w" : "");

    if (file->f_mode & FMODE_READ)
    {
        if (!input_stream.active)
        {
            if (!input_stream.buffers && audio_setup_buf(&input_stream))
                return -ENOMEM;
        }
        poll_wait(file, &input_stream.frag_wq, wait);

        for (i = 0; i < input_stream.nbfrags; i++)
        {
            if (atomic_read(&input_stream.buffers[i].sem.count) > 0)
                mask |= POLLIN | POLLWRNORM;
            break;
        }
    }

    if (file->f_mode & FMODE_WRITE)
    {
        if (!output_stream.active)
        {
            if (!output_stream.buffers && audio_setup_buf(&output_stream))
                return -ENOMEM;
            poll_wait(file, &output_stream.frag_wq, wait);
        }


        for (i = 0; i < output_stream.nbfrags; i++)
        {
            if (atomic_read(&output_stream.buffers[i].sem.count) > 0)
                mask |= POLLOUT | POLLWRNORM;
            break;
        }
    }

    dprintk("audio_poll() returned mask of %s\n", (mask & POLLOUT) ? "w" : "");
    return mask;
}

static          loff_t
sbc2440_audio_llseek(struct file *file, loff_t offset, int origin)
{
    return -ESPIPE;
}

static int
sbc2440_mixer_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
    /*
     * We only accept mixer (type 'M') ioctls.
     */
    if (_IOC_TYPE(cmd) != 'M')
        return -EINVAL;

    return l3_command(&uda1341, cmd, (void *) arg);
}


static int
iispsr_value(int sample_rate)
{
    int             i;
    unsigned long   fact0 = clk_get_rate(clk_get(NULL, "pclk")) / S_CLOCK_FREQ;
    unsigned long   r0_sample_rate,
                    r1_sample_rate = 0,
        r2_sample_rate;
    int             prescaler = 0;

    dprintk("requested sample_rate = %d\n", sample_rate);

    for (i = 1; i < 32; i++)
    {
        r1_sample_rate = fact0 / i;
        if (r1_sample_rate < sample_rate)
            break;
    }

    r0_sample_rate = fact0 / (i + 1);
    r2_sample_rate = fact0 / (i - 1);

    dprintk("calculated (%d-1) freq = %ld, error = %d\n", i + 1, r0_sample_rate, abs(r0_sample_rate - sample_rate));
    dprintk("calculated (%d-1) freq = %ld, error = %d\n", i, r1_sample_rate, abs(r1_sample_rate - sample_rate));
    dprintk("calculated (%d-1) freq = %ld, error = %d\n", i - 1, r2_sample_rate, abs(r2_sample_rate - sample_rate));

    prescaler = i;
    if (abs(r0_sample_rate - sample_rate) < abs(r1_sample_rate - sample_rate))
        prescaler = i + 1;
    if (abs(r2_sample_rate - sample_rate) < abs(r1_sample_rate - sample_rate))
        prescaler = i - 1;

    prescaler = max_t(int, 0, (prescaler - 1));

    dprintk("selected prescale value = %d, freq = %ld, error = %d\n", prescaler, fact0 / (prescaler + 1), abs((fact0 / (prescaler + 1)) - sample_rate));

    return prescaler;
}

static long
audio_set_dsp_speed(long val)
{
    unsigned long   tmp;
    int             prescaler = 0;

    tmp = clk_get_rate(clk_get(NULL, "pclk")) / S_CLOCK_FREQ;
    dprintk("requested = %ld, limit = %ld\n", val, tmp);
    if (val > (tmp >> 1))
        return -1;
    prescaler = iispsr_value(val);
    IISPSR = IISPSR_A(prescaler) | IISPSR_B(prescaler);

    audio_rate = val;
    output_stream.rate = input_stream.rate = audio_rate;    // ghcstop fix
    dprintk("return audio_rate = %ld\n", (unsigned long) audio_rate);

    return audio_rate;
}


static int
audio_set_fragments(audio_stream_t * s, int val)
{
    if (s->active)
        return -EBUSY;
    if (s->buffers)
        audio_clear_buf(s);
    s->nbfrags = (val >> 16) & 0x7FFF;
    val &= 0xffff;
    if (val < 4)
        val = 4;
    if (val > 15)
        val = 15;
    s->fragsize = 1 << val;
    if (s->nbfrags < 2)
        s->nbfrags = 2;
    if (s->nbfrags * s->fragsize > 128 * 1024)
        s->nbfrags = 128 * 1024 / s->fragsize;
    if (audio_setup_buf(s))
        return -ENOMEM;
    return val | (s->nbfrags << 16);
}


static int
sbc2440_audio_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
    long            val;

//printk("a_ioctl = 0x%08x, SNDCTL_DSP_GETOSPACE=0x%08x\n", cmd, SNDCTL_DSP_GETOSPACE);
    switch (cmd)
    {
    case SNDCTL_DSP_SETFMT:
        get_user(val, (long *) arg);
        if (val & AUDIO_FMT_MASK)
        {
            audio_fmt = val;
            break;
        }
        else
            return -EINVAL;
    case SNDCTL_DSP_CHANNELS:
    case SNDCTL_DSP_STEREO:
        get_user(val, (long *) arg);
        if (cmd == SNDCTL_DSP_STEREO)
            val = val ? 2 : 1;
        if (val != 1 && val != 2)
            return -EINVAL;
        audio_channels = val;
        break;
    case SOUND_PCM_READ_CHANNELS:
        put_user(audio_channels, (long *) arg);
        break;
    case SNDCTL_DSP_SPEED:
        get_user(val, (long *) arg);
        val = audio_set_dsp_speed(val);
        if (val < 0)
            return -EINVAL;
        put_user(val, (long *) arg);
        break;
    case SOUND_PCM_READ_RATE:
        put_user(audio_rate, (long *) arg);
        break;
    case SNDCTL_DSP_GETFMTS:
        put_user(AUDIO_FMT_MASK, (long *) arg);
        break;
    case SNDCTL_DSP_GETBLKSIZE:
        if (file->f_mode & FMODE_WRITE)
            return put_user(output_stream.fragsize, (long *) arg);
        else
            return put_user(input_stream.fragsize, (int *) arg);

    case SNDCTL_DSP_SETFRAGMENT:

        if (get_user(val, (long *) arg))
            return -EFAULT;
        if (file->f_mode & FMODE_READ)
        {
            int             ret = audio_set_fragments(&input_stream, val);

            if (ret < 0)
                return ret;
            ret = put_user(ret, (int *) arg);
            if (ret)
                return ret;
        }
        if (file->f_mode & FMODE_WRITE)
        {
            int             ret = audio_set_fragments(&output_stream, val);

            if (ret < 0)
                return ret;
            ret = put_user(ret, (int *) arg);
            if (ret)
                return ret;
        }
        return 0;
    case SNDCTL_DSP_SYNC:
        return audio_sync(file);
    case SNDCTL_DSP_GETOSPACE:
        {
            audio_stream_t *s = &output_stream;
            audio_buf_info *inf = (audio_buf_info *) arg;
            
            int             err = !access_ok(VERIFY_WRITE, inf,
                                              sizeof(*inf));
            int             i;
            int             frags = 0,
                            bytes = 0,
                            dma_send_bytes = 0;                

            if (!(file->f_mode & FMODE_WRITE))
                return -EINVAL;
            if (err)
                return err;
            if (!s->buffers && audio_setup_buf(s))
                return -ENOMEM;
            for (i = 0; i < s->nbfrags; i++)
            {
                if (atomic_read(&s->buffers[i].sem.count) > 0)
                {
                    if (s->buffers[i].size == 0)
                        frags++;
                    bytes += s->fragsize - s->buffers[i].size;
                }
            }
            
            put_user(frags, &inf->fragments);
            put_user(s->nbfrags, &inf->fragstotal);
            put_user(s->fragsize, &inf->fragsize);
            put_user(bytes, &inf->bytes);
            break;
        }

    case SNDCTL_DSP_GETISPACE:
        {
            audio_stream_t *s = &input_stream;
            audio_buf_info *inf = (audio_buf_info *) arg;
            int             err = !access_ok(VERIFY_WRITE, inf,
                                              sizeof(*inf));
            int             i;
            int             frags = 0,
                bytes = 0;

            if (!(file->f_mode & FMODE_READ))
                return -EINVAL;

            if (err)
                return err;

            if (!s->buffers && audio_setup_buf(s))
                return -ENOMEM;

            for (i = 0; i < s->nbfrags; i++)
            {
                if (atomic_read(&s->buffers[i].sem.count) > 0)
                {
                    if (s->buffers[i].size == s->fragsize)
                        frags++;
                    bytes += s->buffers[i].size;
                }
            }
            put_user(frags, &inf->fragments);
            put_user(s->nbfrags, &inf->fragstotal);
            put_user(s->fragsize, &inf->fragsize);
            put_user(bytes, &inf->bytes);
            break;
        }

    case SNDCTL_DSP_RESET:
        if (file->f_mode & FMODE_READ)
        {
            audio_reset_buf(&input_stream);
        }
        if (file->f_mode & FMODE_WRITE)
        {
            audio_reset_buf(&output_stream);
        }
        return 0;
    case SNDCTL_DSP_NONBLOCK:
        file->f_flags |= O_NONBLOCK;
        return 0;
    case SNDCTL_DSP_POST:
    case SNDCTL_DSP_SUBDIVIDE:
    case SNDCTL_DSP_GETCAPS:
    case SNDCTL_DSP_GETTRIGGER:
    case SNDCTL_DSP_SETTRIGGER:
    case SNDCTL_DSP_GETIPTR:
    case SNDCTL_DSP_GETOPTR:
    case SNDCTL_DSP_MAPINBUF:
    case SNDCTL_DSP_MAPOUTBUF:
    case SNDCTL_DSP_SETSYNCRO:
    case SNDCTL_DSP_SETDUPLEX:
        printk("request IOCTL %d \n", cmd);
        return -ENOSYS;
    default:
        return sbc2440_mixer_ioctl(inode, file, cmd, arg);
    }
    return 0;
}

static int
sbc2440_audio_open(struct inode *inode, struct file *file)
{
    int             cold = !audio_active;

    dprintk("audio_open\n");

    if ((file->f_flags & O_ACCMODE) == O_RDONLY)
    {
        if (audio_rd_refcount || audio_wr_refcount)
            return -EBUSY;
        audio_rd_refcount++;
    }
    else if ((file->f_flags & O_ACCMODE) == O_WRONLY)
    {
        if (audio_wr_refcount)
            return -EBUSY;
        audio_wr_refcount++;
    }
    else if ((file->f_flags & O_ACCMODE) == O_RDWR)
    {
        if (audio_rd_refcount || audio_wr_refcount)
            return -EBUSY;
        audio_rd_refcount++;
        audio_wr_refcount++;
    }
    else
        return -EINVAL;

    if (cold)
    {
        audio_rate = AUDIO_RATE_DEFAULT;
        audio_channels = AUDIO_CHANNELS_DEFAULT;

        /*
         * the UDA1341 is stereo only ==> 2 channels
         */
        if ((file->f_mode & FMODE_WRITE))
        {
            output_stream.fragsize = AUDIO_FRAGSIZE_DEFAULT;
            output_stream.nbfrags = AUDIO_NBFRAGS_DEFAULT;
            output_stream.channels = audio_channels;

            start_sbc2440_iis_bus_tx();
            audio_clear_buf(&output_stream);
	    if (!output_stream .buffers && audio_setup_buf(&output_stream)) 
		return -ENOMEM;
            init_waitqueue_head(&output_stream.frag_wq);
        }
        if ((file->f_mode & FMODE_READ))
        {
            input_stream.fragsize = AUDIO_FRAGSIZE_DEFAULT;
            input_stream.nbfrags = AUDIO_NBFRAGS_DEFAULT;
            input_stream.channels = audio_channels;

            start_sbc2440_iis_bus_rx();
            audio_clear_buf(&input_stream);
            init_waitqueue_head(&input_stream.frag_wq);
        }
    }


    return 0;
}

static int
sbc2440_audio_release(struct inode *inode, struct file *file)
{
    dprintk("audio_release\n");

    if (file->f_mode & FMODE_READ)
    {
        if (audio_rd_refcount == 1)
            audio_clear_buf(&input_stream);
        audio_rd_refcount = 0;
    }

    if (file->f_mode & FMODE_WRITE)
    {
        if (audio_wr_refcount == 1)
        {
            audio_sync(file);
            audio_clear_buf(&output_stream);
            audio_wr_refcount = 0;
        }
    }

    return 0;
}

static void
start_uda1341(void)
{
#if 0
    struct uda1341_cfg cfg;

    cfg.format = FMT_MSB;
    cfg.fs = S_CLOCK_FREQ;
    l3_command(&uda1341, L3_UDA1341_CONFIGURE, &cfg);
#endif
}

static void
start_sbc2440_iis_bus_rx(void)
{
    IISCON = 0;
    IISMOD = 0;
    IISFIFOC = 0;

    /*
     * 44 KHz , 384fs 
     */
    IISPSR = (IISPSR_A(iispsr_value(AUDIO_RATE_DEFAULT)) | IISPSR_B(iispsr_value(AUDIO_RATE_DEFAULT)));

    IISCON = (IISCON_RX_DMA     /* Transmit DMA service request */
              | IISCON_TX_IDLE  /* Receive Channel idle */
              | IISCON_PRESCALE);   /* IIS Prescaler Enable */

    IISMOD = (IISMOD_SEL_MA     /* Master mode */
              | IISMOD_SEL_RX | IISMOD_CH_RIGHT /* Low for left channel */
              | IISMOD_FMT_MSB  /* MSB-justified format */
              | IISMOD_BIT_16   /* Serial data bit/channel is 16 bit */
#if (S_CLOCK_FREQ == 384)
              | IISMOD_FREQ_384 /* Master clock freq = 384 fs */
#else
              | IISMOD_FREQ_256 /* Master clock freq = 256 fs */
#endif
              | IISMOD_SFREQ_32);   /* 32 fs */

    IISFIFOC = (IISFCON_RX_DMA  /* Transmit FIFO access mode: DMA */
                | IISFCON_RX_EN);   /* Transmit FIFO enable */

    IISCON |= IISCON_EN;        /* IIS enable(start) */
}

void
start_sbc2440_iis_bus_tx(void)
{
    IISCON = 0;
    IISMOD = 0;
    IISFIFOC = 0;

    IISPSR = (IISPSR_A(iispsr_value(AUDIO_RATE_DEFAULT)) | IISPSR_B(iispsr_value(AUDIO_RATE_DEFAULT)));

    IISCON = (IISCON_TX_DMA     /* Transmit DMA service request */
              | IISCON_RX_IDLE  /* Receive Channel idle */
              | IISCON_PRESCALE);   /* IIS Prescaler Enable */

    IISMOD = (IISMOD_SEL_MA     /* Master mode */
              | IISMOD_SEL_TX   /* Transmit */
              | IISMOD_CH_RIGHT /* Low for left channel */
              | IISMOD_FMT_MSB  /* MSB-justified format */
              | IISMOD_BIT_16   /* Serial data bit/channel is 16 bit */
#if (S_CLOCK_FREQ == 384)
              | IISMOD_FREQ_384 /* Master clock freq = 384 fs */
#else
              | IISMOD_FREQ_256 /* Master clock freq = 256 fs */
#endif
              | IISMOD_SFREQ_32);   /* 32 fs */

    IISFIFOC = (IISFCON_TX_DMA  /* Transmit FIFO access mode: DMA */
                | IISFCON_TX_EN);   /* Transmit FIFO enable */

    IISCON |= IISCON_EN;        /* IIS enable(start) */

}

#ifdef USE_SYSFS // sysfs사용할 경우.
static int audio_init_dma(audio_stream_t * s, char *desc)
#else
static int __init audio_init_dma(audio_stream_t * s, char *desc)
#endif
{
	int ret;
	
    if (s->dma_ch == S3C2410_DMA_CH2)
    {
        ret = s3c2410_dma_request(s->dma_ch, &(s->dmaclient), NULL);
        if( ret )
        {
        	dprintk("%s: dma request err\n", __FUNCTION__ );
        	return ret;
        }
        ao_dcon = S3C2410_DCON_HANDSHAKE|S3C2410_DCON_SYNC_PCLK|S3C2410_DCON_TSZUNIT|S3C2410_DCON_SSERVE|S3C2410_DCON_CH2_I2SSDO|S3C2410_DCON_NORELOAD|
        s3c2410_dma_config(s->dma_ch, 2, ao_dcon); // a out, halfword
        s3c2410_dma_setflags(s->dma_ch, S3C2410_DMAF_AUTOSTART); // a out

        s3c2410_dma_set_buffdone_fn(s->dma_ch, audio_dmaout_done_callback);
        s3c2410_dma_devconfig(s->dma_ch, S3C2410_DMASRC_MEM, BUF_ON_APB, 0x55000010);

        dprintk("%s: dma request done audio out channel\n", __FUNCTION__ );
        
        
        
        return 0;
    }
    else if (s->dma_ch == S3C2410_DMA_CH1)
    {
        ret = s3c2410_dma_request(s->dma_ch, &(s->dmaclient), NULL);
        if( ret )
        {
        	dprintk("%s: dma request err\n", __FUNCTION__ );
        	return ret;
        }
        ai_dcon = S3C2410_DCON_HANDSHAKE|S3C2410_DCON_SYNC_PCLK|S3C2410_DCON_TSZUNIT|S3C2410_DCON_SSERVE|S3C2410_DCON_CH1_I2SSDI|S3C2410_DCON_NORELOAD|
        s3c2410_dma_config(s->dma_ch, 2, ai_dcon); // a in, halfword
        s3c2410_dma_setflags(s->dma_ch, S3C2410_DMAF_AUTOSTART); // a in
        
        s3c2410_dma_set_buffdone_fn(s->dma_ch, audio_dmain_done_callback);
        s3c2410_dma_devconfig(s->dma_ch, S3C2410_DMASRC_HW, BUF_ON_APB, 0x55000010);          
        
        dprintk("%s: dma request done audio in channel\n", __FUNCTION__ );
        return 0;
    }
    else
        return 1;
}

static int
audio_clear_dma(audio_stream_t * s)
{
	//extern int s3c2410_dma_free(dmach_t channel, s3c2410_dma_client_t *);
	s3c2410_dma_free(s->dma_ch, &(s->dmaclient) );
    //sbc2440_free_dma(s->dma_ch);

    return 0;
}





// ghcstop: audio driver common routine ==> device driver register routine ===================


static struct file_operations sbc2440_audio_fops = {
  llseek:  sbc2440_audio_llseek,
  write:   sbc2440_audio_write,
  read:    sbc2440_audio_read,
  poll:    sbc2440_audio_poll,
  ioctl:   sbc2440_audio_ioctl,
  open:    sbc2440_audio_open,
  release: sbc2440_audio_release,
  owner:	THIS_MODULE
};

static struct file_operations sbc2440_mixer_fops = {
  ioctl: sbc2440_mixer_ioctl,
  owner: THIS_MODULE
};


#if 0 // if use sa1100-audio driver style, todo this
static int h3600_audio_open(struct inode *inode, struct file *file)
{
	return sa1100_audio_attach(inode, file, &audio_state);
}

/*
 * Missing fields of this structure will be patched with the call
 * to sa1100_audio_attach().
 */
static struct file_operations h3600_audio_fops = {
	open:		h3600_audio_open,
	owner:		THIS_MODULE
};
#endif

static inline void
sbc2440_uda1341_enable(void)
{
    start_uda1341();
    start_sbc2440_iis_bus_tx();
}

#ifdef CONFIG_PM
static int
sbc2440_audio_suspend(struct device *dev, u32 state, u32 level)
{
    switch (level)
    {
    case SUSPEND_POWER_DOWN:
        break;
    }

    return 0;
}

static int
sbc2440_audio_resume(struct device *dev, u32 level)
{
    switch (level)
    {
    case RESUME_POWER_ON:
        printk("%s\n", __FUNCTION__);
        sbc2440_uda1341_enable();
        break;
    }

    return 0;
}
#else
#define sbc2440_audio_suspend	NULL
#define sbc2440_audio_resume	NULL
#endif


static int      audio_dev_dsp, audio_dev_mixer;



static int sbc2440_audio_probe(struct device *_dev)
{
    int             ret = 0;

    printk("SBC2440 SOUND driver probe!\n");

    ret = l3_attach_client(&uda1341, "l3-bit-24x0-gpio", "uda1341");
    if (ret)
    {
        printk("l3_attach_client() failed.\n");
        return ret;
    }
    l3_open(&uda1341);
    start_uda1341();

    output_stream.dma_ch = S3C2410_DMA_CH2;
    output_stream.dmaclient.name = "audio_out";
    if (audio_init_dma(&output_stream, "UDA1341 out"))
    {
        audio_clear_dma(&output_stream);
        printk(KERN_WARNING AUDIO_NAME_VERBOSE ": unable to get DMA channels\n");
        return -EBUSY;
    }

    input_stream.dma_ch = S3C2410_DMA_CH1;
    input_stream.dmaclient.name = "audio_in";

    if (audio_init_dma(&input_stream, "UDA1341 in"))
    {
        audio_clear_dma(&input_stream);
        printk(KERN_WARNING AUDIO_NAME_VERBOSE ": unable to get DMA channels\n");
        return -EBUSY;
    }
    audio_dev_dsp = register_sound_dsp(&sbc2440_audio_fops, -1);
    audio_dev_mixer = register_sound_mixer(&sbc2440_mixer_fops, -1);

    printk(AUDIO_NAME_VERBOSE " initialized\n");

    return 0;
}


static int sbc2440_audio_remove(struct device *_dev)
{
    unregister_sound_dsp(audio_dev_dsp);
    unregister_sound_mixer(audio_dev_mixer);
    audio_clear_dma(&output_stream);
    audio_clear_dma(&input_stream);
    l3_close(&uda1341);
    l3_detach_client(&uda1341);
    printk(AUDIO_NAME_VERBOSE " unloaded\n");
    
    return 0;
}

static struct device_driver sbc2440_audio_driver = {
	.name		= "s3c2440-sound",
	.bus		= &platform_bus_type,
	.probe		= sbc2440_audio_probe,
	.remove		= sbc2440_audio_remove,
	.suspend	= sbc2440_audio_suspend,
	.resume		= sbc2440_audio_resume,
};

#ifdef USE_SYSFS
static int __init sbc2440_uda1341_init(void)
{
	int ret = -ENODEV;
printk("SBC2440 SOUND driver register\n");
	//if (!machine_is_h3600() || machine_is_h3100() || machine_is_h3800())
	ret = driver_register(&sbc2440_audio_driver);
	if( ret )
	{
		printk("S3C2440 SOUND driver un registered, %d\n", ret);
	}

	return ret;
}

static void __exit sbc2440_uda1341_exit(void)
{
	driver_unregister(&sbc2440_audio_driver);
}
#else
int __init sbc2440_uda1341_init(void)
{
    int             ret = 0;

//printk("ghcstop.........probe\n");

    ret = l3_attach_client(&uda1341, "l3-bit-24x0-gpio", "uda1341");
    if (ret)
    {
        printk("l3_attach_client() failed.\n");
        return ret;
    }
    l3_open(&uda1341);
    start_uda1341();

    output_stream.dma_ch = S3C2410_DMA_CH2;
    output_stream.dmaclient.name = "audio_out";
    if (audio_init_dma(&output_stream, "UDA1341 out"))
    {
        audio_clear_dma(&output_stream);
        printk(KERN_WARNING AUDIO_NAME_VERBOSE ": unable to get DMA channels\n");
        return -EBUSY;
    }

    input_stream.dma_ch = S3C2410_DMA_CH1;
    input_stream.dmaclient.name = "audio_in";

    if (audio_init_dma(&input_stream, "UDA1341 in"))
    {
        audio_clear_dma(&input_stream);
        printk(KERN_WARNING AUDIO_NAME_VERBOSE ": unable to get DMA channels\n");
        return -EBUSY;
    }
    audio_dev_dsp = register_sound_dsp(&sbc2440_audio_fops, -1);
    audio_dev_mixer = register_sound_mixer(&sbc2440_mixer_fops, -1);

    printk(AUDIO_NAME_VERBOSE " initialized\n");

    return 0;
}

void __exit sbc2440_uda1341_exit(void)
{
    unregister_sound_dsp(audio_dev_dsp);
    unregister_sound_mixer(audio_dev_mixer);
    audio_clear_dma(&output_stream);
    audio_clear_dma(&input_stream);
    l3_close(&uda1341);
    l3_detach_client(&uda1341);
    printk(AUDIO_NAME_VERBOSE " unloaded\n");
    
    //return 0;
}


#endif


module_init(sbc2440_uda1341_init);
module_exit(sbc2440_uda1341_exit);

MODULE_AUTHOR("Kwanghyun La <nala.la@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IIS sound driver for S3C2440");
