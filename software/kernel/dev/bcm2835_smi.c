#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_data/dma-bcm2708.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_dma.h>

#include "virt-dma.h"


#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/pagemap.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/io.h>

#include <linux/delay.h>

/* misc. constants */
#define BUFFER_SIZE 4*100000
#define N_BUFFER 8
#define MMAP_SIZE (BUFFER_SIZE*N_BUFFER)


/******************************************************************************
 * Stream buffers
 ******************************************************************************/

struct bcm2835_cb_entry {
	struct bcm2835_dma_cb *cb;
	dma_addr_t paddr;
};

typedef struct
{
	char *buf_virt;
	dma_addr_t buf_phys;
	struct bcm2835_dma_cb *control_block;
	dma_addr_t paddr;
	uint8_t idx;

	struct bcm2835_cb_entry cb000;
	struct bcm2835_cb_entry cb00;
	struct bcm2835_cb_entry cb01;
	struct bcm2835_cb_entry cb02;
	struct bcm2835_cb_entry cb03;

	struct bcm2835_cb_entry cb1;
	struct bcm2835_cb_entry cb2;
	struct bcm2835_cb_entry cb3;
	struct list_head node;
} stream_fifo_item_t;

typedef struct
{
	struct list_head headlist;
	unsigned int length;
	spinlock_t lock;
	unsigned long flags;
} stream_fifo_t;

static int stream_fifo_init(stream_fifo_t *fifo)
{
	spin_lock_init(&(fifo->lock));
	INIT_LIST_HEAD(&(fifo->headlist));
	fifo->length = 0;
	return 0;
}

static int stream_fifo_deinit(stream_fifo_t *fifo)
{
	struct list_head *pos;
	struct list_head *q;
	stream_fifo_item_t *item;
	int count;

	count = 0;
	list_for_each_safe(pos, q, &(fifo->headlist))
	{
		item = list_entry(pos, stream_fifo_item_t, node);
		kfree(item);
		list_del(pos);
		count++;
	}
	fifo->length = 0;
	return count;
}

static int stream_fifo_push(stream_fifo_t *fifo, stream_fifo_item_t *item)
{
	if (NULL != item)
	{

		list_add_tail(&(item->node), &(fifo->headlist));
		fifo->length++;
		return 0;
	}
	else
	{
		printk(KERN_ALERT "%s> pushing NULL element\n", __func__);
		return -1;
	}
}

stream_fifo_item_t *stream_fifo_pop(stream_fifo_t *fifo)
{
	stream_fifo_item_t *elt;

	if (list_empty(&(fifo->headlist)))
	{
		elt = NULL;
	}
	else
	{
		elt = list_first_entry(&(fifo->headlist), stream_fifo_item_t, node);
		list_del(&elt->node);
		fifo->length--;
	}
	return elt;
}

static int stream_fifo_empty(stream_fifo_t *fifo)
{
	return list_empty(&(fifo->headlist));
}

inline static void stream_fifo_lock(stream_fifo_t *fifo)
{
	spin_lock_irqsave(&(fifo->lock), fifo->flags);
}

inline static void stream_fifo_unlock(stream_fifo_t *fifo)
{
	spin_unlock_irqrestore(&(fifo->lock), fifo->flags);
}


#define BCM2835_SMI_IMPLEMENTATION
#include <linux/broadcom/bcm2835_smi.h>

static dma_addr_t smi_regs_busaddr;
static uint8_t *buf_virt;
static struct bcm2835_smi_instance *inst;
static struct bcm2835_dma_cb *last_control_block = NULL;

struct bcm2835_dma_cb {
	uint32_t info; /* TODO */
	uint32_t src;
	uint32_t dst;
	uint32_t length;
	uint32_t stride;
	uint32_t next;
	uint32_t pad[2];
};


struct bcm2835_chan {
	struct virt_dma_chan vc;

	struct dma_slave_config	cfg;
	unsigned int dreq;

	int ch;
	struct bcm2835_desc *desc;
	struct dma_pool *cb_pool;

	void __iomem *chan_base;
	int irq_number;
	unsigned int irq_flags;

	bool is_lite_channel;
	bool is_40bit_channel;
};

struct bcm2835_desc {
	struct bcm2835_chan *c;
	struct virt_dma_desc vd;
	enum dma_transfer_direction dir;

	unsigned int frames;
	size_t size;

	bool cyclic;

	struct bcm2835_cb_entry cb_list[];
};

#define BCM2835_DMA_CS		0x00
#define BCM2835_DMA_ADDR	0x04
#define BCM2835_DMA_TI		0x08
#define BCM2835_DMA_SOURCE_AD	0x0c
#define BCM2835_DMA_DEST_AD	0x10
#define BCM2835_DMA_LEN		0x14
#define BCM2835_DMA_STRIDE	0x18
#define BCM2835_DMA_NEXTCB	0x1c
#define BCM2835_DMA_DEBUG	0x20

/* DMA CS Control and Status bits */
#define BCM2835_DMA_ACTIVE	BIT(0)  /* activate the DMA */
#define BCM2835_DMA_END		BIT(1)  /* current CB has ended */
#define BCM2835_DMA_INT		BIT(2)  /* interrupt status */
#define BCM2835_DMA_DREQ	BIT(3)  /* DREQ state */
#define BCM2835_DMA_ISPAUSED	BIT(4)  /* Pause requested or not active */
#define BCM2835_DMA_ISHELD	BIT(5)  /* Is held by DREQ flow control */
#define BCM2835_DMA_WAITING_FOR_WRITES BIT(6) /* waiting for last
					       * AXI-write to ack
					       */
#define BCM2835_DMA_ERR		BIT(8)
#define BCM2835_DMA_PRIORITY(x) ((x & 15) << 16) /* AXI priority */
#define BCM2835_DMA_PANIC_PRIORITY(x) ((x & 15) << 20) /* panic priority */
/* current value of TI.BCM2835_DMA_WAIT_RESP */
#define BCM2835_DMA_WAIT_FOR_WRITES BIT(28)
#define BCM2835_DMA_DIS_DEBUG	BIT(29) /* disable debug pause signal */
#define BCM2835_DMA_ABORT	BIT(30) /* Stop current CB, go to next, WO */
#define BCM2835_DMA_RESET	BIT(31) /* WO, self clearing */

/* Transfer information bits - also bcm2835_cb.info field */
#define BCM2835_DMA_INT_EN	BIT(0)
#define BCM2835_DMA_TDMODE	BIT(1) /* 2D-Mode */
#define BCM2835_DMA_WAIT_RESP	BIT(3) /* wait for AXI-write to be acked */
#define BCM2835_DMA_D_INC	BIT(4)
#define BCM2835_DMA_D_WIDTH	BIT(5) /* 128bit writes if set */
#define BCM2835_DMA_D_DREQ	BIT(6) /* enable DREQ for destination */
#define BCM2835_DMA_D_IGNORE	BIT(7) /* ignore destination writes */
#define BCM2835_DMA_S_INC	BIT(8)
#define BCM2835_DMA_S_WIDTH	BIT(9) /* 128bit writes if set */
#define BCM2835_DMA_S_DREQ	BIT(10) /* enable SREQ for source */
#define BCM2835_DMA_S_IGNORE	BIT(11) /* ignore source reads - read 0 */
#define BCM2835_DMA_BURST_LENGTH(x) ((x & 15) << 12)
#define BCM2835_DMA_CS_FLAGS(x) (x & (BCM2835_DMA_PRIORITY(15) | \
				      BCM2835_DMA_PANIC_PRIORITY(15) | \
				      BCM2835_DMA_WAIT_FOR_WRITES | \
				      BCM2835_DMA_DIS_DEBUG))
#define BCM2835_DMA_PER_MAP(x)	((x & 31) << 16) /* REQ source */
#define BCM2835_DMA_WAIT(x)	((x & 31) << 21) /* add DMA-wait cycles */
#define BCM2835_DMA_NO_WIDE_BURSTS BIT(26) /* no 2 beat write bursts */

/* A fake bit to request that the driver doesn't set the WAIT_RESP bit. */
#define BCM2835_DMA_NO_WAIT_RESP BIT(27)
#define WAIT_RESP(x) ((x & BCM2835_DMA_NO_WAIT_RESP) ? \
		      0 : BCM2835_DMA_WAIT_RESP)

/* A fake bit to request that the driver requires wide reads */
#define BCM2835_DMA_WIDE_SOURCE BIT(24)
#define WIDE_SOURCE(x) ((x & BCM2835_DMA_WIDE_SOURCE) ? \
		      BCM2835_DMA_S_WIDTH : 0)

/* A fake bit to request that the driver requires wide writes */
#define BCM2835_DMA_WIDE_DEST BIT(25)
#define WIDE_DEST(x) ((x & BCM2835_DMA_WIDE_DEST) ? \
		      BCM2835_DMA_D_WIDTH : 0)

/* Valid only for channels 0 - 14, 15 has its own base address */
#define BCM2835_DMA_CHAN_SIZE	0x100
#define BCM2835_DMA_CHAN(n)	((n) * BCM2835_DMA_CHAN_SIZE) /* Base address */
#define BCM2835_DMA_CHANIO(base, n) ((base) + BCM2835_DMA_CHAN(n))

/* the max dma length for different channels */
#define MAX_DMA_LEN SZ_1G
#define MAX_LITE_DMA_LEN (SZ_64K - 4)

static struct bcm2835_desc *bcm2835_dma_create_cb_chain2(
	struct bcm2835_chan *c,
	enum dma_transfer_direction direction,
	bool cyclic, u32 info, u32 finalextrainfo, size_t frames,
	gfp_t gfp, size_t count);

static inline struct bcm2835_chan *to_bcm2835_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct bcm2835_chan, vc.chan);
}

static inline struct bcm2835_desc *to_bcm2835_dma_desc(
		struct dma_async_tx_descriptor *t)
{
	return container_of(t, struct bcm2835_desc, vd.tx);
}

uint32_t *smics_disable;
uint32_t *smics_enable; 
uint32_t *smics_enable_w; 
uint32_t *smil; 
uint32_t *smil_w; 
uint32_t *smics_start; 
uint32_t *smics_start_w; 
uint32_t *smics_enable_clear; 
uint32_t *smics_enable_clear_w; 

uint32_t *dummy_enable; 
uint32_t *dummy_addr; 
uint32_t *dummy_addr2; 



static struct bcm2835_desc *bcm2835_dma_create_cb_chain(
	struct bcm2835_chan *c,
	enum dma_transfer_direction direction,
	bool cyclic, u32 info, u32 finalextrainfo, size_t frames,
	gfp_t gfp)
{
	size_t frame = 0;
	struct bcm2835_desc *d;
	struct bcm2835_cb_entry *cb_entry;
	struct bcm2835_dma_cb *control_block;

	/* allocate and setup the descriptor. */
	d = kzalloc(struct_size(d, cb_list, 20), gfp);
	if (!d)
		return NULL;

	d->c = c;
	d->dir = direction;
	d->cyclic = cyclic;

	cb_entry = &d->cb_list[15];
	cb_entry->cb = dma_pool_alloc(c->cb_pool, gfp, &cb_entry->paddr);
	control_block = cb_entry->cb;
	control_block->info = BCM2835_DMA_INT_EN;
	control_block->dst = (uint32_t) buf_virt;
	control_block->src = (uint32_t) &buf_virt[0x4000];
	control_block->length = 0x0;
	control_block->stride = 0;
	control_block->next = 0;

	*dummy_addr = cb_entry->paddr;
	
	cb_entry = &d->cb_list[16];
	cb_entry->cb = dma_pool_alloc(c->cb_pool, gfp, &cb_entry->paddr);
	control_block = cb_entry->cb;
	control_block->info = BCM2835_DMA_INT_EN;
	control_block->dst = (uint32_t) buf_virt;
	control_block->src = (uint32_t) &buf_virt[0x5000];
	control_block->length = 0x0;
	control_block->stride = 0;
	control_block->next = 0;

	*dummy_addr2 = cb_entry->paddr;

	cb_entry = &d->cb_list[frame];
	cb_entry->cb = dma_pool_alloc(c->cb_pool, gfp,
			&cb_entry->paddr);

	/* fill in the control block */
	control_block = cb_entry->cb;
	control_block->info = BCM2835_DMA_WAIT(31) | BCM2835_DMA_WAIT_RESP;
	control_block->dst = smi_regs_busaddr + SMICS;
	control_block->src = (uint32_t) smics_disable;
	control_block->length = 4;
	control_block->stride = 0;
	control_block->next = 0;
	printk("ici %x %x %x", control_block->src, control_block->dst, control_block->info);
	if (frame)
		d->cb_list[frame - 1].cb->next = cb_entry->paddr;
	d->frames += 1;
	frame += 1;

	cb_entry = &d->cb_list[frame];
	cb_entry->cb = dma_pool_alloc(c->cb_pool, gfp,
			&cb_entry->paddr);

	/* fill in the control block */
	control_block = cb_entry->cb;
	control_block->info = BCM2835_DMA_WAIT(31) | BCM2835_DMA_WAIT_RESP;
	control_block->dst = smi_regs_busaddr + SMIL;
	control_block->src = (uint32_t) smil;
	control_block->length = 4;
	control_block->stride = 0;
	control_block->next = 0;
	printk("ici %x %x %x", control_block->src, control_block->dst, control_block->info);
	if (frame)
		d->cb_list[frame - 1].cb->next = cb_entry->paddr;
	d->frames += 1;
	frame += 1;

	cb_entry = &d->cb_list[frame];
	cb_entry->cb = dma_pool_alloc(c->cb_pool, gfp,
			&cb_entry->paddr);

	/* fill in the control block */
	control_block = cb_entry->cb;
	control_block->info = BCM2835_DMA_WAIT(31) | BCM2835_DMA_WAIT_RESP;
	control_block->dst = smi_regs_busaddr + SMICS;
	control_block->src = (uint32_t) smics_enable;
	control_block->length = 4;
	control_block->stride = 0;
	control_block->next = 0;
	printk("ici %x %x %x => %llx", control_block->src, control_block->dst, control_block->info, cb_entry->paddr);
	if (frame)
		d->cb_list[frame - 1].cb->next = cb_entry->paddr;
	d->frames += 1;
	frame += 1;

	cb_entry = &d->cb_list[frame];
	cb_entry->cb = dma_pool_alloc(c->cb_pool, gfp,
			&cb_entry->paddr);

	/* fill in the control block */
	control_block = cb_entry->cb;
	control_block->info = BCM2835_DMA_WAIT(31) | BCM2835_DMA_WAIT_RESP;
	control_block->dst = smi_regs_busaddr + SMICS;
	control_block->src = (uint32_t) smics_enable_clear;
	control_block->length = 4;
	control_block->stride = 0;
	control_block->next = 0;
	printk("ici %x %x %x", control_block->src, control_block->dst, control_block->info);
	if (frame)
		d->cb_list[frame - 1].cb->next = cb_entry->paddr;
	d->frames += 1;
	frame += 1;

	cb_entry = &d->cb_list[frame];
	cb_entry->cb = dma_pool_alloc(c->cb_pool, gfp,
			&cb_entry->paddr);

	/* fill in the control block */
	control_block = cb_entry->cb;
	control_block->info = BCM2835_DMA_WAIT(31) | BCM2835_DMA_WAIT_RESP;
	control_block->dst = smi_regs_busaddr + SMICS;
	control_block->src = (uint32_t) smics_start;
	control_block->length = 4;
	control_block->stride = 0;
	control_block->next = 0;
	printk("ici %x %x %x", control_block->src, control_block->dst, control_block->info);
	if (frame)
		d->cb_list[frame - 1].cb->next = cb_entry->paddr;
	d->frames += 1;
	frame += 1;

	cb_entry = &d->cb_list[frame];
	cb_entry->cb = dma_pool_alloc(c->cb_pool, gfp,
			&cb_entry->paddr);

	/* fill in the control block */
	control_block = cb_entry->cb;
	control_block->info = info;
	control_block->src = smi_regs_busaddr + SMID;
	control_block->dst = (uint32_t) buf_virt;
	control_block->length = 0x40000;
	control_block->stride = 0;
	control_block->next = 0;
	printk("ici %x %x %x", control_block->src, control_block->dst, control_block->info);
	if (frame)
		d->cb_list[frame - 1].cb->next = cb_entry->paddr;
	d->frames += 1;
	frame += 1;

	cb_entry = &d->cb_list[frame];
	cb_entry->cb = dma_pool_alloc(c->cb_pool, gfp,
			&cb_entry->paddr);

	/* fill in the control block */
	control_block = cb_entry->cb;
	control_block->info = BCM2835_DMA_WAIT(31);
	control_block->dst = 0x7e007000 + 7*0x100 + BCM2835_DMA_ADDR;
	control_block->src = (uint32_t) dummy_addr;
	control_block->length = 4;
	control_block->stride = 0;
	control_block->next = 0;
	printk("ici %x %x %x %x", *(uint32_t*)control_block->src, control_block->src, control_block->dst, control_block->info);
	if (frame)
		d->cb_list[frame - 1].cb->next = cb_entry->paddr;
	d->frames += 1;
	frame += 1;

	cb_entry = &d->cb_list[frame];
	cb_entry->cb = dma_pool_alloc(c->cb_pool, gfp,
			&cb_entry->paddr);

	/* fill in the control block */
	control_block = cb_entry->cb;
	control_block->info = BCM2835_DMA_WAIT(31);
	control_block->dst = 0x7e007000 + 7*0x100 + BCM2835_DMA_CS;
	control_block->src = (uint32_t) dummy_enable;
	control_block->length = 4;
	control_block->stride = 0;
	control_block->next = 0;
	printk("ici %x %x %x", control_block->src, control_block->dst, control_block->info);
	if (frame)
		d->cb_list[frame - 1].cb->next = cb_entry->paddr;
	d->frames += 1;
	frame += 1;

	cb_entry = &d->cb_list[frame];
	cb_entry->cb = dma_pool_alloc(c->cb_pool, gfp,
			&cb_entry->paddr);

	/* fill in the control block */
	control_block = cb_entry->cb;
	control_block->info = info;
	control_block->src = smi_regs_busaddr + SMID;
	control_block->dst = (uint32_t) &buf_virt[0x40000];
	control_block->length = 0x40000;
	control_block->stride = 0;
	control_block->next = 0;
	printk("ici %x %x %x", control_block->src, control_block->dst, control_block->info);
	if (frame)
		d->cb_list[frame - 1].cb->next = cb_entry->paddr;
	d->frames += 1;
	frame += 1;

	cb_entry = &d->cb_list[frame];
	cb_entry->cb = dma_pool_alloc(c->cb_pool, gfp,
			&cb_entry->paddr);

	/* fill in the control block */
	control_block = cb_entry->cb;
	control_block->info = BCM2835_DMA_WAIT(31);
	control_block->dst = 0x7e007000 + 7*0x100 + BCM2835_DMA_ADDR;
	control_block->src = (uint32_t) dummy_addr2;
	control_block->length = 4;
	control_block->stride = 0;
	control_block->next = 0;
	printk("ici %x %x %x %x", *(uint32_t*)control_block->src, control_block->src, control_block->dst, control_block->info);
	if (frame)
		d->cb_list[frame - 1].cb->next = cb_entry->paddr;
	d->frames += 1;
	frame += 1;

	cb_entry = &d->cb_list[frame];
	cb_entry->cb = dma_pool_alloc(c->cb_pool, gfp,
			&cb_entry->paddr);

	/* fill in the control block */
	control_block = cb_entry->cb;
	control_block->info = BCM2835_DMA_WAIT(31);
	control_block->dst = 0x7e007000 + 7*0x100 + BCM2835_DMA_CS;
	control_block->src = (uint32_t) dummy_enable;
	control_block->length = 4;
	control_block->stride = 0;
	control_block->next = 0;
	printk("ici %x %x %x", control_block->src, control_block->dst, control_block->info);
	if (frame)
		d->cb_list[frame - 1].cb->next = cb_entry->paddr;
	d->frames += 1;
	frame += 1;

	//{
	//	int j;
	//	for (j = 0; j< d->frames; j++) {
	//		printk("cb %d %x %x %x", j, d->cb_list[j].cb->info, d->cb_list[j].cb->src, d->cb_list[j].cb->dst);
	//	}	
	//}

	return d;
}

static struct dma_async_tx_descriptor *bcm2835_dma_prep_dma_memcpy(
	struct dma_chan *chan,
	unsigned long flags)
{
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);
	struct bcm2835_desc *d;
	struct bcm2835_cb_entry *cb_entry;
	struct bcm2835_dma_cb *control_block;

	/* allocate and setup the descriptor. */
	d = kzalloc(struct_size(d, cb_list, 1), GFP_KERNEL);

	d->c = c;
	d->dir = DMA_MEM_TO_MEM;
	d->cyclic = true;
	
	cb_entry = &d->cb_list[0];
	cb_entry->cb = dma_pool_alloc(c->cb_pool, GFP_NOWAIT,
			&cb_entry->paddr);

	if (cb_entry->cb == NULL) {
		printk("ca pue");
	}

	/* fill in the control block */
	control_block = cb_entry->cb;
	control_block->info = 0x40140|BCM2835_DMA_INT_EN;
	control_block->dst = (uint32_t) buf_virt;
	control_block->src = (uint32_t) buf_virt;
	control_block->length = 0;
	control_block->stride = 0;
	control_block->next = 0;
	d->frames += 1;

	return vchan_tx_prep(&c->vc, &d->vd, flags);
}

static struct dma_async_tx_descriptor *bcm2835_dma_prep_dma_cyclic2(
	struct dma_chan *chan,
	unsigned long flags,
	size_t count)
{
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);
	struct bcm2835_desc *d;
	u32 info = 0;
	u32 extra = 0;

	/* Setup DREQ channel */
	if (c->dreq != 0)
		info |= BCM2835_DMA_PER_MAP(c->dreq);

	info |= BCM2835_DMA_D_DREQ | BCM2835_DMA_S_INC;

	/*
	 * allocate the CB chain
	 * note that we need to use GFP_NOWAIT, as the ALSA i2s dmaengine
	 * implementation calls prep_dma_cyclic with interrupts disabled.
	 */
	d = bcm2835_dma_create_cb_chain2(c,
			DMA_MEM_TO_DEV, false, 
					info, extra,
					1, 
					GFP_NOWAIT,
					count
					);

	return vchan_tx_prep(&c->vc, &d->vd, flags);
}

static struct dma_async_tx_descriptor *bcm2835_dma_prep_dma_cyclic(
	struct dma_chan *chan,
	unsigned long flags)
{
	struct bcm2835_chan *c = to_bcm2835_dma_chan(chan);
	struct bcm2835_desc *d;
	u32 info = WAIT_RESP(c->dreq) | WIDE_SOURCE(c->dreq) | WIDE_DEST(c->dreq);
	u32 extra = 0;

	/* Setup DREQ channel */
	if (c->dreq != 0)
		info |= BCM2835_DMA_PER_MAP(c->dreq);

	printk("info2 %x", info);

	info |= BCM2835_DMA_S_DREQ | BCM2835_DMA_D_INC;

	/*
	 * allocate the CB chain
	 * note that we need to use GFP_NOWAIT, as the ALSA i2s dmaengine
	 * implementation calls prep_dma_cyclic with interrupts disabled.
	 */
	d = bcm2835_dma_create_cb_chain(c,
			DMA_DEV_TO_MEM, true, 
					info, extra,
					1, 
					GFP_NOWAIT);
#if 1
	/* wrap around into a loop */
	d->cb_list[d->frames - 1].cb->next = d->cb_list[0].paddr;
#endif

	return vchan_tx_prep(&c->vc, &d->vd, flags);
}


/**
 * Broadcom Secondary Memory Interface driver
 *
 * Written by Luke Wren <luke@raspberrypi.org>
 * Copyright (c) 2015, Raspberry Pi (Trading) Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2, as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



#define DRIVER_NAME "smi-bcm2835"

#define N_PAGES_FROM_BYTES(n) ((n + PAGE_SIZE-1) / PAGE_SIZE)

#define DMA_WRITE_TO_MEM true
#define DMA_READ_FROM_MEM false

struct dma_ctxt
{
  int wr_idx;                    /**< write pointer of the FIFO */
  u64 bam;                    /**< write pointer of the FIFO */
  struct bcm2835_smi_instance *inst;
};


struct bcm2835_smi_instance {
	struct device *dev;
	struct smi_settings settings;
	__iomem void *smi_regs_ptr;
	dma_addr_t smi_regs_busaddr;

	struct dma_chan *dma_chan;
	struct dma_chan *dma_chan2;
	char *buf_virt2;
	dma_addr_t buf_phys2;

	wait_queue_head_t wq;
	stream_fifo_t unused_buffer_fifo;
	stream_fifo_item_t *user_buffer;

	struct dma_slave_config dma_config;

	struct bcm2835_smi_bounce_info bounce;

	struct scatterlist buffer_sgl;

	struct clk *clk;

	/* Sometimes we are called into in an atomic context (e.g. by
	   JFFS2 + MTD) so we can't use a mutex */
	spinlock_t transaction_lock;

	char *buf_virt;                       /**< virtual address of the shared DMA memory */
	dma_addr_t buf_phys;                  /**< physical address of the shared DMA memory */

	struct dma_async_tx_descriptor *desc;   /**< aync DMA descriptor of the current transaction */
	struct dma_async_tx_descriptor *desc2;   /**< aync DMA descriptor of the current transaction */

	dma_cookie_t dma_cookie;                /**< DMA cookie of the current transaction */
	dma_cookie_t dma_cookie2;                /**< DMA cookie of the current transaction */
	struct dma_ctxt dma_ctxt;
	spinlock_t lock;                      /**< lock protecting fifo (wr_idx, rd_idx, n_filled_segments) */
};

/****************************************************************************
*
*   SMI peripheral setup
*
***************************************************************************/

static inline void write_smi_reg(struct bcm2835_smi_instance *inst,
	u32 val, unsigned reg)
{
	writel(val, inst->smi_regs_ptr + reg);
}

static inline u32 read_smi_reg(struct bcm2835_smi_instance *inst, unsigned reg)
{
	return readl(inst->smi_regs_ptr + reg);
}

/* Token-paste macro for e.g SMIDSR_RSTROBE ->  value of SMIDSR_RSTROBE_MASK */
#define _CONCAT(x, y) x##y
#define CONCAT(x, y) _CONCAT(x, y)

#define SET_BIT_FIELD(dest, field, bits) ((dest) = \
	((dest) & ~CONCAT(field, _MASK)) | (((bits) << CONCAT(field, _OFFS))& \
	 CONCAT(field, _MASK)))
#define GET_BIT_FIELD(src, field) (((src) & \
	CONCAT(field, _MASK)) >> CONCAT(field, _OFFS))

static void smi_dump_context_labelled(struct bcm2835_smi_instance *inst,
	const char *label)
{
	dev_err(inst->dev, "SMI context dump: %s", label);
	dev_err(inst->dev, "SMICS:  0x%08x", read_smi_reg(inst, SMICS));
	dev_err(inst->dev, "SMIL:   0x%08x", read_smi_reg(inst, SMIL));
	dev_err(inst->dev, "SMIDSR: 0x%08x", read_smi_reg(inst, SMIDSR0));
	dev_err(inst->dev, "SMIDSW: 0x%08x", read_smi_reg(inst, SMIDSW0));
	dev_err(inst->dev, "SMIDC:  0x%08x", read_smi_reg(inst, SMIDC));
	dev_err(inst->dev, "SMIFD:  0x%08x", read_smi_reg(inst, SMIFD));
	dev_err(inst->dev, " ");
}

static inline void smi_dump_context(struct bcm2835_smi_instance *inst)
{
	smi_dump_context_labelled(inst, "");
}

static void smi_get_default_settings(struct bcm2835_smi_instance *inst)
{
	struct smi_settings *settings = &inst->settings;

	settings->data_width = SMI_WIDTH_8BIT;
	settings->pack_data = true;

	settings->read_setup_time = 1;
	settings->read_hold_time = 0;
	settings->read_pace_time = 0;
	settings->read_strobe_time = 2;

	settings->write_setup_time = settings->read_setup_time;
	settings->write_hold_time = settings->read_hold_time;
	settings->write_pace_time = settings->read_pace_time;
	settings->write_strobe_time = settings->read_strobe_time;

	settings->dma_enable = true;
	settings->dma_passthrough_enable = true;
	settings->dma_read_thresh = 0x02;
	settings->dma_write_thresh = 0x2;
	settings->dma_panic_read_thresh = 0x8;
	settings->dma_panic_write_thresh = 0x8;
}

void bcm2835_smi_set_regs_from_settings(struct bcm2835_smi_instance *inst)
{
	struct smi_settings *settings = &inst->settings;
	int smidsr_temp = 0, smidsw_temp = 0, smics_temp,
	    smidcs_temp, smidc_temp = 0;

	spin_lock(&inst->transaction_lock);

	/* temporarily disable the peripheral: */
	smics_temp = read_smi_reg(inst, SMICS);
	write_smi_reg(inst, 0, SMICS);
	smidcs_temp = read_smi_reg(inst, SMIDCS);
	write_smi_reg(inst, 0, SMIDCS);

	if (settings->pack_data)
		smics_temp |= SMICS_PXLDAT;
	else
		smics_temp &= ~SMICS_PXLDAT;

	SET_BIT_FIELD(smidsr_temp, SMIDSR_RWIDTH, settings->data_width);
	SET_BIT_FIELD(smidsr_temp, SMIDSR_RSETUP, settings->read_setup_time);
	SET_BIT_FIELD(smidsr_temp, SMIDSR_RHOLD, settings->read_hold_time);
	SET_BIT_FIELD(smidsr_temp, SMIDSR_RPACE, settings->read_pace_time);
	SET_BIT_FIELD(smidsr_temp, SMIDSR_RSTROBE, settings->read_strobe_time);
	write_smi_reg(inst, smidsr_temp, SMIDSR0);

	SET_BIT_FIELD(smidsw_temp, SMIDSW_WWIDTH, settings->data_width);
	if (settings->data_width == SMI_WIDTH_8BIT)
		smidsw_temp |= SMIDSW_WSWAP;
	else
		smidsw_temp &= ~SMIDSW_WSWAP;
	SET_BIT_FIELD(smidsw_temp, SMIDSW_WSETUP, settings->write_setup_time);
	SET_BIT_FIELD(smidsw_temp, SMIDSW_WHOLD, settings->write_hold_time);
	SET_BIT_FIELD(smidsw_temp, SMIDSW_WPACE, settings->write_pace_time);
	SET_BIT_FIELD(smidsw_temp, SMIDSW_WSTROBE,
			settings->write_strobe_time);
	write_smi_reg(inst, smidsw_temp, SMIDSW0);

	SET_BIT_FIELD(smidc_temp, SMIDC_REQR, settings->dma_read_thresh);
	SET_BIT_FIELD(smidc_temp, SMIDC_REQW, settings->dma_write_thresh);
	SET_BIT_FIELD(smidc_temp, SMIDC_PANICR,
		      settings->dma_panic_read_thresh);
	SET_BIT_FIELD(smidc_temp, SMIDC_PANICW,
		      settings->dma_panic_write_thresh);
	if (settings->dma_passthrough_enable) {
		smidc_temp |= SMIDC_DMAP;
		smidsr_temp |= SMIDSR_RDREQ;
		write_smi_reg(inst, smidsr_temp, SMIDSR0);
		smidsw_temp |= SMIDSW_WDREQ;
		write_smi_reg(inst, smidsw_temp, SMIDSW0);
	} else
		smidc_temp &= ~SMIDC_DMAP;
	if (settings->dma_enable)
		smidc_temp |= SMIDC_DMAEN;
	else
		smidc_temp &= ~SMIDC_DMAEN;

	write_smi_reg(inst, smidc_temp, SMIDC);

	/* re-enable (if was previously enabled) */
	write_smi_reg(inst, smics_temp, SMICS);
	write_smi_reg(inst, smidcs_temp, SMIDCS);

	spin_unlock(&inst->transaction_lock);
}
EXPORT_SYMBOL(bcm2835_smi_set_regs_from_settings);

struct smi_settings *bcm2835_smi_get_settings_from_regs
	(struct bcm2835_smi_instance *inst)
{
	struct smi_settings *settings = &inst->settings;
	int smidsr, smidsw, smidc;

	spin_lock(&inst->transaction_lock);

	smidsr = read_smi_reg(inst, SMIDSR0);
	smidsw = read_smi_reg(inst, SMIDSW0);
	smidc = read_smi_reg(inst, SMIDC);

	settings->pack_data = (read_smi_reg(inst, SMICS) & SMICS_PXLDAT) ?
	    true : false;

	settings->data_width = GET_BIT_FIELD(smidsr, SMIDSR_RWIDTH);
	settings->read_setup_time = GET_BIT_FIELD(smidsr, SMIDSR_RSETUP);
	settings->read_hold_time = GET_BIT_FIELD(smidsr, SMIDSR_RHOLD);
	settings->read_pace_time = GET_BIT_FIELD(smidsr, SMIDSR_RPACE);
	settings->read_strobe_time = GET_BIT_FIELD(smidsr, SMIDSR_RSTROBE);

	settings->write_setup_time = GET_BIT_FIELD(smidsw, SMIDSW_WSETUP);
	settings->write_hold_time = GET_BIT_FIELD(smidsw, SMIDSW_WHOLD);
	settings->write_pace_time = GET_BIT_FIELD(smidsw, SMIDSW_WPACE);
	settings->write_strobe_time = GET_BIT_FIELD(smidsw, SMIDSW_WSTROBE);

	settings->dma_read_thresh = GET_BIT_FIELD(smidc, SMIDC_REQR);
	settings->dma_write_thresh = GET_BIT_FIELD(smidc, SMIDC_REQW);
	settings->dma_panic_read_thresh = GET_BIT_FIELD(smidc, SMIDC_PANICR);
	settings->dma_panic_write_thresh = GET_BIT_FIELD(smidc, SMIDC_PANICW);
	settings->dma_passthrough_enable = (smidc & SMIDC_DMAP) ? true : false;
	settings->dma_enable = (smidc & SMIDC_DMAEN) ? true : false;

	spin_unlock(&inst->transaction_lock);

	return settings;
}
EXPORT_SYMBOL(bcm2835_smi_get_settings_from_regs);

static inline void smi_set_address(struct bcm2835_smi_instance *inst,
	unsigned int address)
{
	int smia_temp = 0, smida_temp = 0;

	SET_BIT_FIELD(smia_temp, SMIA_ADDR, address);
	SET_BIT_FIELD(smida_temp, SMIDA_ADDR, address);

	/* Write to both address registers - user doesn't care whether we're
	   doing programmed or direct transfers. */
	write_smi_reg(inst, smia_temp, SMIA);
	write_smi_reg(inst, smida_temp, SMIDA);
}

static void smi_setup_regs(struct bcm2835_smi_instance *inst)
{

	dev_dbg(inst->dev, "Initialising SMI registers...");
	/* Disable the peripheral if already enabled */
	write_smi_reg(inst, 0, SMICS);
	write_smi_reg(inst, 0, SMIDCS);

	smi_get_default_settings(inst);
	bcm2835_smi_set_regs_from_settings(inst);
	//smi_set_address(inst, 0);
	smi_set_address(inst, 10);

	write_smi_reg(inst, read_smi_reg(inst, SMICS) | SMICS_ENABLE, SMICS);
	write_smi_reg(inst, read_smi_reg(inst, SMIDCS) | SMIDCS_ENABLE,
		SMIDCS);
}

/****************************************************************************
*
*   Low-level SMI access functions
*   Other modules should use the exported higher-level functions e.g.
*   bcm2835_smi_write_buf() unless they have a good reason to use these
*
***************************************************************************/
static inline uint32_t smi_read_single_word(struct bcm2835_smi_instance *inst)
{
	int timeout = 0;

	write_smi_reg(inst, SMIDCS_ENABLE, SMIDCS);
	write_smi_reg(inst, SMIDCS_ENABLE | SMIDCS_START, SMIDCS);
	/* Make sure things happen in the right order...*/
	mb();
	while (!(read_smi_reg(inst, SMIDCS) & SMIDCS_DONE) &&
		++timeout < 10000)
		;
	if (timeout < 10000)
		return read_smi_reg(inst, SMIDD);

	dev_err(inst->dev,
		"SMI direct read timed out (is the clock set up correctly?)");
	return 0;
}

static inline void smi_write_single_word(struct bcm2835_smi_instance *inst,
	uint32_t data)
{
	int timeout = 0;

	write_smi_reg(inst, SMIDCS_ENABLE | SMIDCS_WRITE, SMIDCS);
	write_smi_reg(inst, data, SMIDD);
	write_smi_reg(inst, SMIDCS_ENABLE | SMIDCS_WRITE | SMIDCS_START,
		SMIDCS);

	while (!(read_smi_reg(inst, SMIDCS) & SMIDCS_DONE) &&
		++timeout < 10000)
		;
	if (timeout >= 10000)
		dev_err(inst->dev,
		"SMI direct write timed out (is the clock set up correctly?)");
}


/* Initiates a programmed read into the read FIFO. It is up to the caller to
 * read data from the FIFO -  either via paced DMA transfer,
 * or polling SMICS_RXD to check whether data is available.
 * SMICS_ACTIVE will go low upon completion. */
static void smi_init_programmed_read(struct bcm2835_smi_instance *inst,
	int num_transfers)
{
	int smics_temp;

	/* Disable the peripheral: */
	smics_temp = read_smi_reg(inst, SMICS) & ~(SMICS_ENABLE | SMICS_WRITE);
	write_smi_reg(inst, smics_temp, SMICS);
	while (read_smi_reg(inst, SMICS) & SMICS_ENABLE)
		;

	/* Program the transfer count: */
	write_smi_reg(inst, num_transfers, SMIL);

	/* re-enable and start: */
	smics_temp |= SMICS_ENABLE;
	write_smi_reg(inst, smics_temp, SMICS);
	smics_temp |= SMICS_CLEAR;
	/* Just to be certain: */
	mb();
	while (read_smi_reg(inst, SMICS) & SMICS_ACTIVE)
		;
	write_smi_reg(inst, smics_temp, SMICS);
	smics_temp |= SMICS_START;
	write_smi_reg(inst, smics_temp, SMICS);
}

/* Initiates a programmed write sequence, using data from the write FIFO.
 * It is up to the caller to initiate a DMA transfer before calling,
 * or use another method to keep the write FIFO topped up.
 * SMICS_ACTIVE will go low upon completion.
 */
static void smi_init_programmed_write(struct bcm2835_smi_instance *inst,
	int num_transfers)

{
	int smics_temp;

	/* Disable the peripheral: */
	smics_temp = read_smi_reg(inst, SMICS) & ~SMICS_ENABLE;
	write_smi_reg(inst, smics_temp, SMICS);

	write_smi_reg(inst, read_smi_reg(inst, SMICS) | SMICS_CLEAR, SMICS);

	smi_set_address(inst, 1);

	// Wait as long as the SMI is still enabled
	while (read_smi_reg(inst, SMICS) & SMICS_ENABLE)
		;

	/* Program the transfer count: */
	write_smi_reg(inst, num_transfers, SMIL);

	/* setup, re-enable and start: */
	//smics_temp |= SMICS_WRITE | SMICS_ENABLE;
	smics_temp |= SMICS_WRITE | SMICS_ENABLE | SMICS_CLEAR;
	write_smi_reg(inst, smics_temp, SMICS);

	smics_temp |= SMICS_START;
	write_smi_reg(inst, smics_temp, SMICS);
}

/* Initiate a read and then poll FIFO for data, reading out as it appears. */
static void smi_read_fifo(struct bcm2835_smi_instance *inst,
	uint32_t *dest, int n_bytes)
{
	if (read_smi_reg(inst, SMICS) & SMICS_RXD) {
		smi_dump_context_labelled(inst,
			"WARNING: read FIFO not empty at start of read call.");
		while (read_smi_reg(inst, SMICS))
			;
	}

	/* Dispatch the read: */
	if (inst->settings.data_width == SMI_WIDTH_8BIT)
		smi_init_programmed_read(inst, n_bytes);
	else if (inst->settings.data_width == SMI_WIDTH_16BIT)
		smi_init_programmed_read(inst, n_bytes / 2);
	else {
		dev_err(inst->dev, "Unsupported data width for read.");
		return;
	}

	/* Poll FIFO to keep it empty */
	while (!(read_smi_reg(inst, SMICS) & SMICS_DONE))
		if (read_smi_reg(inst, SMICS) & SMICS_RXD)
			*dest++ = read_smi_reg(inst, SMID);

	/* Ensure that the FIFO is emptied */
	if (read_smi_reg(inst, SMICS) & SMICS_RXD) {
		int fifo_count;

		fifo_count = GET_BIT_FIELD(read_smi_reg(inst, SMIFD),
			SMIFD_FCNT);
		while (fifo_count--)
			*dest++ = read_smi_reg(inst, SMID);
	}

	if (!(read_smi_reg(inst, SMICS) & SMICS_DONE))
		smi_dump_context_labelled(inst,
			"WARNING: transaction finished but done bit not set.");

	if (read_smi_reg(inst, SMICS) & SMICS_RXD)
		smi_dump_context_labelled(inst,
			"WARNING: read FIFO not empty at end of read call.");

}

/* Initiate a write, and then keep the FIFO topped up. */
static void smi_write_fifo(struct bcm2835_smi_instance *inst,
	uint32_t *src, int n_bytes)
{
	int i, timeout = 0;

	/* Empty FIFOs if not already so */
	if (!(read_smi_reg(inst, SMICS) & SMICS_TXE)) 
	{
		smi_dump_context_labelled(inst, "WARNING: write fifo not empty at start of write call.");
		write_smi_reg(inst, read_smi_reg(inst, SMICS) | SMICS_CLEAR, SMICS);
	}

	/* Initiate the transfer */
	if (inst->settings.data_width == SMI_WIDTH_8BIT)
		smi_init_programmed_write(inst, n_bytes);
	else if (inst->settings.data_width == SMI_WIDTH_16BIT)
		smi_init_programmed_write(inst, n_bytes / 2);
	else {
		dev_err(inst->dev, "Unsupported data width for write.");
		return;
	}
	
	/* Fill the FIFO: */
	for (i = 0; i < (n_bytes - 1) / 4 + 1; ++i) {
		while (!(read_smi_reg(inst, SMICS) & SMICS_TXD))
			;
		write_smi_reg(inst, *src++, SMID);
	}

	/* Busy wait... */
	while (!(read_smi_reg(inst, SMICS) & SMICS_DONE) && ++timeout <
		1000000)
		;

	if (timeout >= 1000000)
		smi_dump_context_labelled(inst,
			"Timed out on write operation!");
	if (!(read_smi_reg(inst, SMICS) & SMICS_TXE))
		smi_dump_context_labelled(inst,
			"WARNING: FIFO not empty at end of write operation.");
}

/****************************************************************************
*
*   SMI DMA operations
*
***************************************************************************/

/* Disable SMI and put it into the correct direction before doing DMA setup.
   Stops spurious DREQs during setup. Peripheral is re-enabled by init_*() */
static void smi_disable(struct bcm2835_smi_instance *inst,
	enum dma_transfer_direction direction)
{
	int smics_temp = read_smi_reg(inst, SMICS) & ~SMICS_ENABLE;

	if (direction == DMA_DEV_TO_MEM)
		smics_temp &= ~SMICS_WRITE;
	else
		smics_temp |= SMICS_WRITE;
	write_smi_reg(inst, smics_temp, SMICS);
	while (read_smi_reg(inst, SMICS) & SMICS_ACTIVE)
		;
}

static struct scatterlist *smi_scatterlist_from_buffer(
	struct bcm2835_smi_instance *inst,
	dma_addr_t buf,
	size_t len,
	struct scatterlist *sg)
{
	sg_init_table(sg, 1);
	sg_dma_address(sg) = buf;
	sg_dma_len(sg) = len;
	return sg;
}

static void smi_dma_callback_user_copy(void *param)
{
	/* Notify the bottom half that a chunk is ready for user copy */
	struct bcm2835_smi_instance *inst =
		(struct bcm2835_smi_instance *)param;

	up(&inst->bounce.callback_sem);
}

/* Creates a descriptor, assigns the given callback, and submits the
   descriptor to dmaengine. Does not block - can queue up multiple
   descriptors and then wait for them all to complete.
   sg_len is the number of control blocks, NOT the number of bytes.
   dir can be DMA_MEM_TO_DEV or DMA_DEV_TO_MEM.
   callback can be NULL - in this case it is not called. */
static inline struct dma_async_tx_descriptor *smi_dma_submit_sgl(
	struct bcm2835_smi_instance *inst,
	struct scatterlist *sgl,
	size_t sg_len,
	enum dma_transfer_direction dir,
	dma_async_tx_callback callback)
{
	struct dma_async_tx_descriptor *desc;

	desc = dmaengine_prep_slave_sg(inst->dma_chan,
				       sgl,
				       sg_len,
				       dir,
				       DMA_PREP_INTERRUPT | DMA_CTRL_ACK |
				       DMA_PREP_FENCE);
	if (!desc) {
		dev_err(inst->dev, "read_sgl: dma slave preparation failed!");
		write_smi_reg(inst, read_smi_reg(inst, SMICS) & ~SMICS_ACTIVE,
			SMICS);
		while (read_smi_reg(inst, SMICS) & SMICS_ACTIVE)
			cpu_relax();
		write_smi_reg(inst, read_smi_reg(inst, SMICS) | SMICS_ACTIVE,
			SMICS);
		return NULL;
	}
	desc->callback = callback;
	desc->callback_param = inst;
	if (dmaengine_submit(desc) < 0)
		return NULL;
	return desc;
}

/* NB this function blocks until the transfer is complete */
static void
smi_dma_read_sgl(struct bcm2835_smi_instance *inst,
	struct scatterlist *sgl, size_t sg_len, size_t n_bytes)
{
	struct dma_async_tx_descriptor *desc;

	/* Disable SMI and set to read before dispatching DMA - if SMI is in
	 * write mode and TX fifo is empty, it will generate a DREQ which may
	 * cause the read DMA to complete before the SMI read command is even
	 * dispatched! We want to dispatch DMA before SMI read so that reading
	 * is gapless, for logic analyser.
	 */

	smi_disable(inst, DMA_DEV_TO_MEM);

	desc = smi_dma_submit_sgl(inst, sgl, sg_len, DMA_DEV_TO_MEM, NULL);
	dma_async_issue_pending(inst->dma_chan);

	if (inst->settings.data_width == SMI_WIDTH_8BIT)
		smi_init_programmed_read(inst, n_bytes);
	else
		smi_init_programmed_read(inst, n_bytes / 2);

	if (dma_wait_for_async_tx(desc) == DMA_ERROR)
		smi_dump_context_labelled(inst, "DMA timeout!");
}

static void
smi_dma_write_sgl(struct bcm2835_smi_instance *inst,
	struct scatterlist *sgl, size_t sg_len, size_t n_bytes)
{
	struct dma_async_tx_descriptor *desc;

	if (inst->settings.data_width == SMI_WIDTH_8BIT)
		smi_init_programmed_write(inst, n_bytes);
	else
		smi_init_programmed_write(inst, n_bytes / 2);

	desc = smi_dma_submit_sgl(inst, sgl, sg_len, DMA_MEM_TO_DEV, NULL);
	dma_async_issue_pending(inst->dma_chan);

	if (dma_wait_for_async_tx(desc) == DMA_ERROR)
		smi_dump_context_labelled(inst, "DMA timeout!");
	else
		/* Wait for SMI to finish our writes */
		while (!(read_smi_reg(inst, SMICS) & SMICS_DONE))
			cpu_relax();
}

ssize_t bcm2835_smi_user_dma(
	struct bcm2835_smi_instance *inst,
	enum dma_transfer_direction dma_dir,
	char __user *user_ptr, size_t count,
	struct bcm2835_smi_bounce_info **bounce)
{

	int chunk_no = 0, chunk_size, count_left = count;
	struct scatterlist *sgl;
	void (*init_trans_func)(struct bcm2835_smi_instance *, int);

	spin_lock(&inst->transaction_lock);

	if (dma_dir == DMA_DEV_TO_MEM)
		init_trans_func = smi_init_programmed_read;
	else
		init_trans_func = smi_init_programmed_write;

	smi_disable(inst, dma_dir);

	sema_init(&inst->bounce.callback_sem, 0);
	if (bounce)
		*bounce = &inst->bounce;
	while (count_left) {
		chunk_size = count_left > DMA_BOUNCE_BUFFER_SIZE ?
			DMA_BOUNCE_BUFFER_SIZE : count_left;
		if (chunk_size == DMA_BOUNCE_BUFFER_SIZE) {
			sgl =
			&inst->bounce.sgl[chunk_no % DMA_BOUNCE_BUFFER_COUNT];
		} else {
			sgl = smi_scatterlist_from_buffer(
				inst,
				inst->bounce.phys[
					chunk_no % DMA_BOUNCE_BUFFER_COUNT],
				chunk_size,
				&inst->buffer_sgl);
		}

		if (!smi_dma_submit_sgl(inst, sgl, 1, dma_dir,
			smi_dma_callback_user_copy
		)) {
			dev_err(inst->dev, "sgl submit failed");
			count = 0;
			goto out;
		}
		count_left -= chunk_size;
		chunk_no++;
	}
	dma_async_issue_pending(inst->dma_chan);

	if (inst->settings.data_width == SMI_WIDTH_8BIT)
		init_trans_func(inst, count);
	else if (inst->settings.data_width == SMI_WIDTH_16BIT)
		init_trans_func(inst, count / 2);
out:
	spin_unlock(&inst->transaction_lock);
	return count;
}
EXPORT_SYMBOL(bcm2835_smi_user_dma);


/****************************************************************************
*
*   High level buffer transfer functions - for use by other drivers
*
***************************************************************************/

/* Buffer must be physically contiguous - i.e. kmalloc, not vmalloc! */
void bcm2835_smi_write_buf(
	struct bcm2835_smi_instance *inst,
	const void *buf, size_t n_bytes)
{
	int odd_bytes = n_bytes & 0x3;

	n_bytes -= odd_bytes;

	spin_lock(&inst->transaction_lock);

	if (n_bytes > DMA_THRESHOLD_BYTES) {
		dma_addr_t phy_addr = dma_map_single(
			inst->dev,
			(void *)buf,
			n_bytes,
			DMA_TO_DEVICE);
		struct scatterlist *sgl =
			smi_scatterlist_from_buffer(inst, phy_addr, n_bytes,
				&inst->buffer_sgl);

		if (!sgl) {
			smi_dump_context_labelled(inst,
			"Error: could not create scatterlist for write!");
			goto out;
		}
		smi_dma_write_sgl(inst, sgl, 1, n_bytes);

		dma_unmap_single
			(inst->dev, phy_addr, n_bytes, DMA_TO_DEVICE);
	} else if (n_bytes) {
		smi_write_fifo(inst, (uint32_t *) buf, n_bytes);
	}
	buf += n_bytes;

	if (inst->settings.data_width == SMI_WIDTH_8BIT) {
		while (odd_bytes--)
			smi_write_single_word(inst, *(uint8_t *) (buf++));
	} else {
		while (odd_bytes >= 2) {
			smi_write_single_word(inst, *(uint16_t *)buf);
			buf += 2;
			odd_bytes -= 2;
		}
		if (odd_bytes) {
			/* Reading an odd number of bytes on a 16 bit bus is
			   a user bug. It's kinder to fail early and tell them
			   than to e.g. transparently give them the bottom byte
			   of a 16 bit transfer. */
			dev_err(inst->dev,
		"WARNING: odd number of bytes specified for wide transfer.");
			dev_err(inst->dev,
		"At least one byte dropped as a result.");
			dump_stack();
		}
	}
out:
	spin_unlock(&inst->transaction_lock);
}
EXPORT_SYMBOL(bcm2835_smi_write_buf);

void bcm2835_smi_read_buf(struct bcm2835_smi_instance *inst,
	void *buf, size_t n_bytes)
{

	/* SMI is inherently 32-bit, which causes surprising amounts of mess
	   for bytes % 4 != 0. Easiest to avoid this mess altogether
	   by handling remainder separately. */
	int odd_bytes = n_bytes & 0x3;

	spin_lock(&inst->transaction_lock);
	n_bytes -= odd_bytes;
	if (n_bytes > DMA_THRESHOLD_BYTES) {
		dma_addr_t phy_addr = dma_map_single(inst->dev,
						     buf, n_bytes,
						     DMA_FROM_DEVICE);
		struct scatterlist *sgl = smi_scatterlist_from_buffer(
			inst, phy_addr, n_bytes,
			&inst->buffer_sgl);
		if (!sgl) {
			smi_dump_context_labelled(inst,
			"Error: could not create scatterlist for read!");
			goto out;
		}
		smi_dma_read_sgl(inst, sgl, 1, n_bytes);
		dma_unmap_single(inst->dev, phy_addr, n_bytes, DMA_FROM_DEVICE);
	} else if (n_bytes) {
		smi_read_fifo(inst, (uint32_t *)buf, n_bytes);
	}
	buf += n_bytes;

	if (inst->settings.data_width == SMI_WIDTH_8BIT) {
		while (odd_bytes--)
			*((uint8_t *) (buf++)) = smi_read_single_word(inst);
	} else {
		while (odd_bytes >= 2) {
			*(uint16_t *) buf = smi_read_single_word(inst);
			buf += 2;
			odd_bytes -= 2;
		}
		if (odd_bytes) {
			dev_err(inst->dev,
		"WARNING: odd number of bytes specified for wide transfer.");
			dev_err(inst->dev,
		"At least one byte dropped as a result.");
			dump_stack();
		}
	}
out:
	spin_unlock(&inst->transaction_lock);
}
EXPORT_SYMBOL(bcm2835_smi_read_buf);

void bcm2835_smi_set_address(struct bcm2835_smi_instance *inst,
	unsigned int address)
{
	spin_lock(&inst->transaction_lock);
	smi_set_address(inst, address);
	spin_unlock(&inst->transaction_lock);
}
EXPORT_SYMBOL(bcm2835_smi_set_address);

struct bcm2835_smi_instance *bcm2835_smi_get(struct device_node *node)
{
	struct platform_device *pdev;

	if (!node)
		return NULL;

	pdev = of_find_device_by_node(node);
	if (!pdev)
		return NULL;

	return platform_get_drvdata(pdev);
}
EXPORT_SYMBOL(bcm2835_smi_get);

/****************************************************************************
*
*   bcm2835_smi_probe - called when the driver is loaded.
*
***************************************************************************/


static int bcm2835_smi_dma_setup2(struct bcm2835_smi_instance *inst)
{
	int rv = 0;
	printk("setup2");

	inst->dma_chan = dma_request_slave_channel(inst->dev, "rx-tx");
	inst->dma_chan2 = dma_request_slave_channel(inst->dev, "test");

	printk("DMA %p %p", inst->dma_chan, inst->dma_chan2);

	inst->dma_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	inst->dma_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	inst->dma_config.src_addr = inst->smi_regs_busaddr + SMID;
	smi_regs_busaddr = inst->smi_regs_busaddr;
	inst->dma_config.dst_addr = inst->dma_config.src_addr;

	/* Direction unimportant - always overridden by prep_slave_sg */
	inst->dma_config.direction = DMA_DEV_TO_MEM;
	dmaengine_slave_config(inst->dma_chan, &inst->dma_config);
	dmaengine_slave_config(inst->dma_chan2, &inst->dma_config);

	//printk("Allocating %u bytes (%u buffers of %u samples + timestamp)\n", MMAP_SIZE, N_BUFFER, DEFAULT_N_CAPTURED_SAMPLES);
	inst->buf_virt = dma_alloc_coherent(inst->dev, PAGE_ALIGN(DMA_BOUNCE_BUFFER_SIZE + 2000), &inst->buf_phys, GFP_KERNEL);
	buf_virt = inst->buf_virt;
	memset(inst->buf_virt, 0xaa, PAGE_ALIGN(DMA_BOUNCE_BUFFER_SIZE + 2000));

	smics_disable = (uint32_t*) &buf_virt[PAGE_ALIGN(DMA_BOUNCE_BUFFER_SIZE)]; 
	smics_enable = (uint32_t*) &buf_virt[PAGE_ALIGN(DMA_BOUNCE_BUFFER_SIZE)+4]; 
	smil = (uint32_t*) &buf_virt[PAGE_ALIGN(DMA_BOUNCE_BUFFER_SIZE)+8]; 
	smics_start = (uint32_t*) &buf_virt[PAGE_ALIGN(DMA_BOUNCE_BUFFER_SIZE)+12]; 
	smics_enable_clear = (uint32_t*) &buf_virt[PAGE_ALIGN(DMA_BOUNCE_BUFFER_SIZE)+16]; 
	dummy_enable = (uint32_t*) &buf_virt[PAGE_ALIGN(DMA_BOUNCE_BUFFER_SIZE)+24]; 
	dummy_addr = (uint32_t*) &buf_virt[PAGE_ALIGN(DMA_BOUNCE_BUFFER_SIZE)+28]; 
	dummy_addr2 = (uint32_t*) &buf_virt[PAGE_ALIGN(DMA_BOUNCE_BUFFER_SIZE)+32]; 

	smics_start_w = (uint32_t*) &buf_virt[PAGE_ALIGN(DMA_BOUNCE_BUFFER_SIZE)+36]; 
	smics_enable_clear_w = (uint32_t*) &buf_virt[PAGE_ALIGN(DMA_BOUNCE_BUFFER_SIZE)+40]; 
	smics_enable_w = (uint32_t*) &buf_virt[PAGE_ALIGN(DMA_BOUNCE_BUFFER_SIZE)+44]; 
	smil_w = (uint32_t*) &buf_virt[PAGE_ALIGN(DMA_BOUNCE_BUFFER_SIZE)+48]; 

	*dummy_enable = BCM2835_DMA_ACTIVE;

#if 0
	*smics_disable = 0x5400c002;
	*smics_enable = 0x5400c001;
	*smics_start = 0x5400c009;
	*smics_enable_clear = 0x5400c011;
#else
	*smics_disable = 0x00000000;
	*smics_enable = 0x00004001;
	*smics_enable_clear = 0x00004011;
	*smics_start = 0x00004009;

	*smics_enable_w = 0x00004021;
	*smics_enable_clear_w = 0x00004031;
	*smics_start_w = 0x00004039;

	//*smics_disable = 0x00;
	//*smics_enable = 0x01;
	//*smics_enable_clear = 0x11;
	//*smics_start = 0x09;
#endif

	//*smil = DMA_BOUNCE_BUFFER_SIZE;
	*smil = 0x40000*2;
	//*smil_w = 512;
	*smil_w = 40000;

	if (NULL == inst->buf_virt)
	{
		printk("dma_alloc_coherent failed\n");
		dma_release_channel(inst->dma_chan);
		return -ENODEV;
	}
	return rv;

}

//=========================================================================
void dump_hex(const void* data, size_t size)
{
        char ascii[17];
        size_t i, j;
        ascii[16] = '\0';
	//printk("dump_hex:\n");

        for (i = 0; i < size; ++i) {
                printk(KERN_CONT "%02X ", ((unsigned char*)data)[i]);
                if (((unsigned char*)data)[i] >= ' ' && ((unsigned char*)data)[i] <= '~')
        {
                        ascii[i % 16] = ((unsigned char*)data)[i];
                }
        else
        {
                        ascii[i % 16] = '.';
                }
                if ((i+1) % 8 == 0 || i+1 == size)
        {
                        printk(KERN_CONT " ");
                        if ((i+1) % 16 == 0)
            {
                                printk(KERN_CONT "|  %s \n", ascii);
                        }
            else if (i+1 == size)
            {
                                ascii[(i+1) % 16] = '\0';
                                if ((i+1) % 16 <= 8)
                {
                                        printk(KERN_CONT " ");
                                }
                                for (j = (i+1) % 16; j < 16; ++j)
                {
                                        printk(KERN_CONT "   ");
                                }
                                printk(KERN_CONT "|  %s \n", ascii);
                        }
                }
        }
}

static uint8_t bami[2*2048];
static uint8_t bami_old[2*2048];

static int bad = 0;

#define PDU_SIZE (256+1+2+1)*4

uint16_t const crc_ccitt_false_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

static inline uint16_t crc_ccitt_false_byte(uint16_t crc, const uint8_t c)
{
    return (crc << 8) ^ crc_ccitt_false_table[(crc >> 8) ^ c];
}

uint16_t crc_ccitt_false(uint16_t crc, uint8_t const *buffer, size_t len)
{
        while (len--)
                crc = crc_ccitt_false_byte(crc, *buffer++);
        return crc;
}


static int seqnum = -1;

static int debug = 0;

#define STATE_HEADER_CA 0
#define STATE_HEADER_FE 1
#define STATE_PAYLOAD 2
#define STATE_TRAILER_CO 3
#define STATE_TRAILER_DE 4
#define STATE_TRAILER_CRC_MSB 5
#define STATE_TRAILER_CRC_LSB 6


extern int bam(uint32_t *samples, int nb_samples, uint64_t ts_first_sample);

static void verify_all(uint8_t id, uint8_t *buf, int len) {
	printk("verify_all1 %x %x %x %x", buf[0], buf[1], buf[2], buf[3]);
	printk("verify_all2 %x %x %x %x", buf[4], buf[5], buf[6], buf[7]);
	return;
#if 1

	int i;
	static int state = STATE_HEADER_CA;
	static int remaining_data = 0;
	static int idx = 0;
	uint16_t crc, res;

	debug ++;

	for (i = 0; i< len; i++) {
		if (state == STATE_HEADER_CA) {
			memcpy(bami_old, bami, 20);
			if (buf[i] == 0xCA) {
				//printk("0xCA %d", i);
				state = STATE_HEADER_FE;
				bami[0] = 0xCA;
			}
		} else if (state == STATE_HEADER_FE) {
			if (buf[i] == 0xFE) {
				//printk("0xFE %d", i);
				state = STATE_PAYLOAD;
				remaining_data = 256*4 + 8 + 2;
				bami[1] = 0xFE;
				idx = 2;
			} else {
				state = STATE_HEADER_CA;
			}
		} else if (state == STATE_PAYLOAD) {
			if (remaining_data > 0) {
				if ((len - i) >= remaining_data) {
					memcpy(&bami[idx], &buf[i], remaining_data); 
					idx += remaining_data;
					i += (remaining_data - 1);
					remaining_data = 0;
				} else {
				//if (idx < 4) {
					bami[idx] = buf[i];
					remaining_data --;
					idx ++;
				//}
				}
			} else {
				int curseqnum = (bami[2] << 8) + bami[3]; 


				if (seqnum != -1) {
					//dump_hex(bami_old, PDU_SIZE);
					if (curseqnum != ((seqnum + 1) & 0x7fff)) {
						printk("id %d curseqnum %d last seqnum %d debug %d", id, curseqnum, seqnum, debug);
						bad ++;
						printk("previous");
						dump_hex(bami_old, 20);
						printk("last");
						dump_hex(bami, 20);
					}
				}
				seqnum = curseqnum;

				state = STATE_TRAILER_CO;

				if (buf[i] == 0xC0) {
					state = STATE_TRAILER_DE;
					bami[idx++] = 0xC0;
				} else {
					bad ++;
					printk("failed CODE %d %d %x", id, i, buf[i]);
					//dump_hex(&buf[i], 8);
					state = STATE_HEADER_CA;
						printk("previous");
						dump_hex(bami_old, 20);
						printk("last");
						dump_hex(bami, 20);
				}
				//printk("CO %d", i);
			}
		} else if (state == STATE_TRAILER_DE) {
			if (buf[i] == 0xDE) {
				bami[idx++] = 0xDE;
				//printk("DE %d", i);
				state = STATE_TRAILER_CRC_MSB;
			} else {
				bad ++;
				printk("failed CODE %d %d %x", id, i, buf[i]);
				//dump_hex(&buf[i], 8);
				state = STATE_HEADER_CA;
					//	printk("previous");
					//	dump_hex(bami_old, PDU_SIZE);
					//	printk("last");
					//	dump_hex(bami, PDU_SIZE);
			}
		} else if (state == STATE_TRAILER_CRC_MSB) {
			state = STATE_TRAILER_CRC_LSB;
			bami[idx++] = buf[i];
			crc = buf[i] << 8;
		} else if (state == STATE_TRAILER_CRC_LSB) {
			state = STATE_HEADER_CA;
			bami[idx++] = buf[i];
			crc |= buf[i];
			//dump_hex(bami, PDU_SIZE);
			res = crc_ccitt_false(0xffff, &bami[2+2+8], 256*4);
			if (res != crc) {
				printk("id %d crc %x res %x", id, crc, res);
					//	printk("previous");
			//		//	dump_hex(bami_old, PDU_SIZE);
			//		//	printk("last");
			//		//	dump_hex(bami, PDU_SIZE);
				bad ++;
			} else {
				//mychardev_bami();
				uint64_t timestamp = ((uint64_t) bami[4] << 56) + ((uint64_t)bami[5] << 48) + ((uint64_t)bami[6] << 40) + ((uint64_t)bami[7] << 32) + (bami[8] << 24) + (bami[9] << 16) + (bami[10] << 8) + bami[11] ;
				//printk("timestamp %lld", timestamp);
				//int a = bam((uint32_t*)&bami[2+2+8], 256, timestamp);
			}
		}
	}
#endif
}

#include <linux/math64.h>

static void dma_cb3(void *params) {

	static uint8_t bam_idx = 0;
	static uint32_t hea_cnt = 0;

	//printk("BAMMMM CB3 %d", hea_cnt);
	//smi_dump_context_labelled(inst, "");

	//dev_err(inst->dev, "SMICS:  0x%08x", read_smi_reg(inst, SMICS));
	//dev_err(inst->dev, "SMIL:   0x%08x", read_smi_reg(inst, SMIL));
	//dev_err(inst->dev, "SMIFD:  0x%08x", read_smi_reg(inst, SMIFD));

	hea_cnt ++;

	struct bcm2835_chan *c = to_bcm2835_dma_chan(inst->dma_chan2);
	struct bcm2835_chan *c1 = to_bcm2835_dma_chan(inst->dma_chan);

	dma_addr_t b = readl(c->chan_base + BCM2835_DMA_SOURCE_AD);
	stream_fifo_item_t *elt = phys_to_virt(b);


	/* TODO verifier que l on loupe rien */
	
	if (bam_idx != elt->idx) {
		printk("BAMMMM CB3 %d %d", elt->idx, bam_idx);
	}

	if (elt->cb000.cb)
		dma_pool_free(c1->cb_pool, elt->cb000.cb, elt->cb000.paddr);
	if (elt->cb00.cb)
		dma_pool_free(c1->cb_pool, elt->cb00.cb, elt->cb00.paddr);
	if (elt->cb01.cb)
		dma_pool_free(c1->cb_pool, elt->cb01.cb, elt->cb01.paddr);
	if (elt->cb02.cb)
		dma_pool_free(c1->cb_pool, elt->cb02.cb, elt->cb02.paddr);
	if (elt->cb03.cb)
		dma_pool_free(c1->cb_pool, elt->cb03.cb, elt->cb03.paddr);

	if (elt->cb1.cb)
		dma_pool_free(c1->cb_pool, elt->cb1.cb, elt->cb1.paddr);
	if (elt->cb2.cb)
		dma_pool_free(c1->cb_pool, elt->cb2.cb, elt->cb2.paddr);
	if (elt->cb3.cb)
		dma_pool_free(c1->cb_pool, elt->cb3.cb, elt->cb3.paddr);

	bam_idx += 1;
	bam_idx %= N_BUFFER;

	/* Recycle transmitted transaction */
	stream_fifo_lock(&inst->unused_buffer_fifo);
	stream_fifo_push(&inst->unused_buffer_fifo, elt);
	stream_fifo_unlock(&inst->unused_buffer_fifo);
	wake_up(&inst->wq);
}

static void dma_cb2(void *params) {
	static u64 prev_ts = 0;
	static void *last_b = NULL;

#if 1
	struct dma_ctxt * dma_ctxt = (struct dma_ctxt *) params;

	/* TODO hack this function */

	u64 ts = ktime_get_ns();
	int dt = div_u64(ts - prev_ts, 1000000);

	struct bcm2835_chan *c2 = to_bcm2835_dma_chan(dma_ctxt->inst->dma_chan2);

	void *b = (void*) readl(c2->chan_base + BCM2835_DMA_SOURCE_AD);

	//printk("dbg1 %p b %p dt %d ts %lld", last_b, b, dt, ts);

	if (b == last_b) {
		printk("on a loupe une IT %d", dt);
	}

#if 1
	if (b == &buf_virt[0x4000]) {
		if (last_b == &buf_virt[0x5000] && (dt >= 15 || dt <= 17)) {
			//printk("ping %d", dt);
		} else {
			printk("ca pue last_b %p b %p dt %d", last_b, b, dt);
		}
		if (last_b != NULL) {
			verify_all(dma_ctxt->wr_idx, &buf_virt[0], 0x40000);
		}
	} else {
		if (last_b == &buf_virt[0x4000] && (dt >= 15 || dt <= 17)) {
			//printk("pong %d", dt);
		} else {
			printk("ca pue last_b %p b %p dt %d", last_b, b, dt);
		}
		if (last_b != NULL) {
			verify_all(dma_ctxt->wr_idx, &buf_virt[0x40000], 0x40000);
		}
	}
#endif

	//printk("first\n");
	//dump_hex(buf_virt, 32);
	//printk("second\n");
	//dump_hex(&buf_virt[0x40000], 32);
	last_b = b;
	prev_ts = ts;	
	dma_ctxt->wr_idx += 1;
	
	if ((dma_ctxt->wr_idx % 500) == 0) {
		printk("so far so good %lld %d %x", dma_ctxt->bam, bad, bami[0]);
		dma_ctxt->wr_idx = 0;
		dma_ctxt->bam += 1;
	}
	b = (void*) readl(c2->chan_base + BCM2835_DMA_SOURCE_AD);
	uint64_t ts2 = ktime_get_ns();
	int dt2 = div_u64(ts2 - ts, 1000000);
	//printk("dbg22 %p b %p dt %d ts %lld", last_b, b, dt2 , ts);
#endif
}

static void bcm2835_dma_start_desc(struct bcm2835_chan *c)
{
	struct virt_dma_desc *vd = vchan_next_desc(&c->vc);
	struct bcm2835_desc *d;
	printk("YO %d", c->ch);

	if (!vd) {
		c->desc = NULL;
		return;
	}

	list_del(&vd->node);

	c->desc = d = to_bcm2835_dma_desc(&vd->tx);
}

int bcm2835_smi_start_tx(size_t);
//int my_testd(struct bcm2835_smi_instance *inst)
int my_test(const char*buf, size_t count)
{
	smi_set_address(inst, 6);

	struct bcm2835_chan *c = to_bcm2835_dma_chan(inst->dma_chan);

	*smil_w = count;

	copy_from_user(inst->user_buffer->buf_virt, buf, count);

	dma_addr_t cs;

	dma_addr_t cb = readl(c->chan_base + BCM2835_DMA_ADDR);
	//printk("b1 %llx", cb);

	if (cb != 0) {

		//printk("on link %d", c->desc->frames);

		struct bcm2835_dma_cb *control_block;

		dma_addr_t paddr;

		control_block = dma_pool_alloc(c->cb_pool, GFP_NOWAIT, &paddr);
		dma_addr_t fpaddr = paddr;

		inst->user_buffer->cb000.cb = control_block;
		inst->user_buffer->cb000.paddr = paddr;

		control_block->info = BCM2835_DMA_WAIT(31) | BCM2835_DMA_WAIT_RESP;
		control_block->dst = (uint32_t) buf_virt;
		control_block->src = (uint32_t) &buf_virt[0x5000];
		control_block->length = 512;
		control_block->stride = 0;
		control_block->next = 0;

		last_control_block->next = paddr;
		last_control_block = control_block;

		control_block = dma_pool_alloc(c->cb_pool, GFP_NOWAIT, &paddr);

		inst->user_buffer->cb00.cb = control_block;
		inst->user_buffer->cb00.paddr = paddr;

		control_block->info = BCM2835_DMA_WAIT(31) | BCM2835_DMA_WAIT_RESP;
		control_block->dst = smi_regs_busaddr + SMICS;
		control_block->src = (uint32_t) smics_disable;
		control_block->length = 4;
		control_block->stride = 0;
		control_block->next = 0;

		last_control_block->next = paddr;
		last_control_block = control_block;

		control_block = dma_pool_alloc(c->cb_pool, GFP_NOWAIT, &paddr);

		inst->user_buffer->cb01.cb = control_block;
		inst->user_buffer->cb01.paddr = paddr;

		control_block->info = BCM2835_DMA_WAIT(31) | BCM2835_DMA_WAIT_RESP;
		control_block->dst = smi_regs_busaddr + SMIL;
		control_block->src = (uint32_t) smil_w;
		control_block->length = 4;
		control_block->stride = 0;
		control_block->next = 0;

		last_control_block->next = paddr;
		last_control_block = control_block;

		control_block = dma_pool_alloc(c->cb_pool, GFP_NOWAIT, &paddr);

		inst->user_buffer->cb02.cb = control_block;
		inst->user_buffer->cb02.paddr = paddr;

		control_block->info = BCM2835_DMA_WAIT(31) | BCM2835_DMA_WAIT_RESP;
		control_block->dst = smi_regs_busaddr + SMICS;
		control_block->src = (uint32_t) smics_enable_w;
		control_block->length = 4;
		control_block->stride = 0;
		control_block->next = 0;

		last_control_block->next = paddr;
		last_control_block = control_block;

		control_block = dma_pool_alloc(c->cb_pool, GFP_NOWAIT, &paddr);

		inst->user_buffer->cb03.cb = control_block;
		inst->user_buffer->cb03.paddr = paddr;

		control_block->info = BCM2835_DMA_WAIT(31) | BCM2835_DMA_WAIT_RESP;
		control_block->dst = smi_regs_busaddr + SMICS;
		control_block->src = (uint32_t) smics_start_w;
		control_block->length = 4;
		control_block->stride = 0;
		control_block->next = 0;

		last_control_block->next = paddr;
		last_control_block = control_block;

		/* SMI => DISABLE, WAIT, PROGRAM SMIL, RE-ENABLE, START, verifier les pertes residuelles - fifo not emptied - */

		control_block = dma_pool_alloc(c->cb_pool, GFP_NOWAIT, &paddr);

		inst->user_buffer->cb1.cb = control_block;
		inst->user_buffer->cb1.paddr = paddr;

		if (control_block == NULL) {
			printk("ca pue");
		}


		/* fill in the control block */
		control_block->info = 0x00040140;
		control_block->dst = smi_regs_busaddr + SMID;
		control_block->src = inst->user_buffer->buf_virt;
		control_block->length = count;
		control_block->stride = 0;
		control_block->next = 0;

		last_control_block->next = paddr;
		last_control_block = control_block;

		control_block = dma_pool_alloc(c->cb_pool, GFP_NOWAIT, &paddr);

		if (control_block == NULL) {
			printk("ca pue");
		}

		inst->user_buffer->cb2.cb = control_block;
		inst->user_buffer->cb2.paddr = paddr;

		/* fill in the control block */
		control_block->info = BCM2835_DMA_WAIT(31);
		control_block->dst = 0x7e007000 + 7*0x100 + BCM2835_DMA_ADDR;
		control_block->src = virt_to_phys(&inst->user_buffer->paddr);
		control_block->length = 4;
		control_block->stride = 0;
		control_block->next = 0;
		//printk("ici %x %x %x %x", *(uint32_t*)control_block->src, control_block->src, control_block->dst, control_block->info);

		last_control_block->next = paddr;
		last_control_block = control_block;

		control_block = dma_pool_alloc(c->cb_pool, GFP_NOWAIT, &paddr);

		if (control_block == NULL) {
			printk("ca pue");
		}

		inst->user_buffer->cb3.cb = control_block;
		inst->user_buffer->cb3.paddr = paddr;

		/* fill in the control block */
		control_block->info = BCM2835_DMA_WAIT(31) | BCM2835_DMA_INT_EN;
		control_block->dst = 0x7e007000 + 7*0x100 + BCM2835_DMA_CS;
		control_block->src = (uint32_t) dummy_enable;
		control_block->length = 4;
		control_block->stride = 0;
		control_block->next = 0;
		//printk("ici %x %x %x", control_block->src, control_block->dst, control_block->info);

		last_control_block->next = paddr;
		last_control_block = control_block;

		cb = readl(c->chan_base + BCM2835_DMA_ADDR);
		//printk("alors %llx", cb);

		if (cb == 0) {
			printk("foutu !!! %llx", cb);
		}	
		
		//printk("on pause");
		cs = readl(c->chan_base + BCM2835_DMA_CS);
		cs &= ~1; 
		writel(cs, c->chan_base + BCM2835_DMA_CS);

		dma_addr_t ncb = readl(c->chan_base + BCM2835_DMA_NEXTCB);
		//printk("ncb %llx", ncb);
		if (ncb == 0) {
			writel(fpaddr, c->chan_base + BCM2835_DMA_NEXTCB);
		}

		//printk("on resume");

		cs = readl(c->chan_base + BCM2835_DMA_CS);
		cs |= 1; 
		writel(cs, c->chan_base + BCM2835_DMA_CS);
	} else {
		bcm2835_smi_start_tx(count);

	}

	stream_fifo_lock(&inst->unused_buffer_fifo);
	/* block until an unused buffer is available */
	wait_event_lock_irq(inst->wq, !(stream_fifo_empty(&inst->unused_buffer_fifo)), inst->unused_buffer_fifo.lock);
	inst->user_buffer = stream_fifo_pop(&inst->unused_buffer_fifo);
	stream_fifo_unlock(&inst->unused_buffer_fifo);

	return 0;
}

//int bcm2835_smi_start(struct bcm2835_smi_instance *inst)
int bcm2835_smi_start(void)
{
    bad = 0;
    seqnum = -1;
    debug = 0;

    inst->dma_ctxt.wr_idx = 0;
    inst->dma_ctxt.bam = 0;
    inst->dma_ctxt.inst = inst;

#if 1
    inst->desc2 = bcm2835_dma_prep_dma_memcpy(inst->dma_chan2, DMA_PREP_INTERRUPT|DMA_CTRL_ACK);
    if (NULL == inst->desc2)
    {
	    printk(KERN_ALERT "%s> dmaengine_prep_dma_cyclic failed\n", __func__);
	    return -1;
    }
    else
    {
	    printk(KERN_ALERT "%s> dmaengine_prep_dma_cyclic success\n", __func__);
    }
    inst->desc2->callback = dma_cb2;
    inst->desc2->callback_param = &inst->dma_ctxt;
    inst->dma_cookie2 = dmaengine_submit(inst->desc2);

    if (dma_submit_error(inst->dma_cookie2) != 0)
    {
	    printk("%s> dmaengine_submit failed\n", __func__);
	    return -1;
    }
    {
    struct bcm2835_chan *c = to_bcm2835_dma_chan(inst->dma_chan2);
    if (vchan_issue_pending(&c->vc) && !c->desc)
	    bcm2835_dma_start_desc(c);
    }
#endif

    
#if 1
    inst->desc = bcm2835_dma_prep_dma_cyclic(inst->dma_chan, DMA_PREP_INTERRUPT|DMA_CTRL_ACK);
    if (NULL == inst->desc)
    {
        printk(KERN_ALERT "%s> dmaengine_prep_dma_cyclic failed\n", __func__);
        return -1;
    }
    else
    {
        printk(KERN_ALERT "%s> dmaengine_prep_dma_cyclic success\n", __func__);
    }

    inst->desc->callback = NULL;
    inst->desc->callback_param = NULL;
    inst->dma_cookie = dmaengine_submit(inst->desc);
    if (dma_submit_error(inst->dma_cookie) != 0)
    {
	    printk("%s> dmaengine_submit failed\n", __func__);
	    return -1;
    }
    dma_async_issue_pending(inst->dma_chan);
#endif

    return 0;
}

int bcm2835_smi_start_tx(size_t count)
{
    struct dma_async_tx_descriptor *desc = NULL;

#if 1
    inst->desc2 = bcm2835_dma_prep_dma_memcpy(inst->dma_chan2, DMA_PREP_INTERRUPT|DMA_CTRL_ACK);
    if (NULL == inst->desc2)
    {
	    printk(KERN_ALERT "%s> dmaengine_prep_dma_cyclic failed\n", __func__);
	    return -1;
    }
    else
    {
//	    printk(KERN_ALERT "%s> dmaengine_prep_dma_cyclic success\n", __func__);
    }
    inst->desc2->callback = dma_cb3;
    inst->dma_cookie2 = dmaengine_submit(inst->desc2);

    if (dma_submit_error(inst->dma_cookie2) != 0)
    {
	    printk("%s> dmaengine_submit failed\n", __func__);
	    return -1;
    }

   // dma_async_issue_pending(inst->dma_chan2);

    if (true) {
	    struct bcm2835_chan *c = to_bcm2835_dma_chan(inst->dma_chan2);

	    if (vchan_issue_pending(&c->vc) && !c->desc) {
		    bcm2835_dma_start_desc(c);
		    printk("ca se passe normalement");
	    } else { 
		    //printk("ca se passe mal %d", c->desc);
	    }
    }
#endif

    smi_init_programmed_write(inst, count);

    desc = bcm2835_dma_prep_dma_cyclic2(inst->dma_chan, DMA_PREP_INTERRUPT|DMA_CTRL_ACK|DMA_PREP_FENCE, count);

    if (NULL == desc)
    {
        printk(KERN_ALERT "%s> dmaengine_prep_dma_cyclic failed\n", __func__);
        return -1;
    }
    else
    {
   //     printk(KERN_ALERT "%s> dmaengine_prep_dma_cyclic success\n", __func__);
    }

    inst->dma_cookie = dmaengine_submit(desc);

    if (dma_submit_error(inst->dma_cookie) != 0)
    {
	    printk("%s> dmaengine_submit failed\n", __func__);
	    return -1;
    }

    inst->desc = desc;

    dma_async_issue_pending(inst->dma_chan);

    return 0;
}

#if 0
static int bcm2835_smi_dma_setup(struct bcm2835_smi_instance *inst)
{
	int i, rv = 0;

	inst->dma_chan = dma_request_slave_channel(inst->dev, "rx-tx");

	inst->dma_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	inst->dma_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	inst->dma_config.src_addr = inst->smi_regs_busaddr + SMID;
	inst->dma_config.dst_addr = inst->dma_config.src_addr;
	/* Direction unimportant - always overridden by prep_slave_sg */
	inst->dma_config.direction = DMA_DEV_TO_MEM;
	dmaengine_slave_config(inst->dma_chan, &inst->dma_config);
	/* Alloc and map bounce buffers */
	for (i = 0; i < DMA_BOUNCE_BUFFER_COUNT; ++i) {
		inst->bounce.buffer[i] =
		dmam_alloc_coherent(inst->dev, DMA_BOUNCE_BUFFER_SIZE,
				&inst->bounce.phys[i],
				GFP_KERNEL);
		if (!inst->bounce.buffer[i]) {
			dev_err(inst->dev, "Could not allocate buffer!");
			rv = -ENOMEM;
			break;
		}
		smi_scatterlist_from_buffer(
			inst,
			inst->bounce.phys[i],
			DMA_BOUNCE_BUFFER_SIZE,
			&inst->bounce.sgl[i]
		);
	}

	return rv;
}
#endif

static int start(void)
{
	inst->user_buffer = stream_fifo_pop(&inst->unused_buffer_fifo);
	return 0;
}

static int bcm2835_smi_probe(struct platform_device *pdev)
{
	int err;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct resource *ioresource;
	const __be32 *addr;
	size_t buffer_idx;


	/* We require device tree support */
	if (!node)
		return -EINVAL;
	/* Allocate buffers and instance data */
	inst = devm_kzalloc(dev, sizeof(struct bcm2835_smi_instance),
		GFP_KERNEL);
	if (!inst)
		return -ENOMEM;


	inst->dev = dev;
	spin_lock_init(&inst->transaction_lock);

	ioresource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	inst->smi_regs_ptr = devm_ioremap_resource(dev, ioresource);
	if (IS_ERR(inst->smi_regs_ptr)) {
		err = PTR_ERR(inst->smi_regs_ptr);
		goto err;
	}
	addr = of_get_address(node, 0, NULL, NULL);
	inst->smi_regs_busaddr = be32_to_cpu(*addr);

	init_waitqueue_head(&inst->wq);

	err = bcm2835_smi_dma_setup2(inst);
	if (err)
		goto err;

	stream_fifo_init(&inst->unused_buffer_fifo);

	printk(KERN_ALERT "Allocating %u bytes (%u buffers of maximum %u samples)\n", MMAP_SIZE, N_BUFFER, BUFFER_SIZE >> 2);
	inst->buf_virt2 = dma_alloc_coherent(inst->dev, PAGE_ALIGN(MMAP_SIZE), &inst->buf_phys2, GFP_KERNEL);

	struct bcm2835_chan *c = to_bcm2835_dma_chan(inst->dma_chan2);

	for (buffer_idx = 0; buffer_idx < N_BUFFER; buffer_idx++)
	{
		stream_fifo_item_t *elt = kmalloc(sizeof(stream_fifo_item_t), GFP_KERNEL);
		elt->control_block = dma_pool_alloc(c->cb_pool, GFP_NOWAIT, &elt->paddr);

		elt->buf_virt = inst->buf_virt2 + PAGE_ALIGN(buffer_idx*BUFFER_SIZE);
		elt->buf_phys = inst->buf_phys2 + PAGE_ALIGN(buffer_idx*BUFFER_SIZE);


		elt->control_block->info = 0x40140 | BCM2835_DMA_INT_EN;
		//elt->control_block->dst = (uint32_t) buf_virt;
		elt->control_block->dst = virt_to_phys(elt);
		elt->control_block->src = virt_to_phys(elt);
		elt->control_block->length = 0x0;
		elt->control_block->stride = 0;
		elt->control_block->next = 0;
		elt->idx = buffer_idx;

		elt->cb000.cb = NULL;
		elt->cb00.cb = NULL;
		elt->cb01.cb = NULL;
		elt->cb02.cb = NULL;
		elt->cb03.cb = NULL;
		elt->cb1.cb = NULL;
		elt->cb2.cb = NULL;
		elt->cb3.cb = NULL;

		stream_fifo_push(&inst->unused_buffer_fifo, elt);
		if (elt->buf_virt == NULL)
		{
			printk(KERN_ERR "dma_alloc_coherent failed\n");
			return -ENODEV;
		}
		printk(KERN_ALERT "buf_virt = 0x%p, buf_phys = 0x%08llx, offset = 0x%lx\n", elt, elt->buf_phys, elt->buf_virt - inst->buf_virt2);
	}

	start();

	/* request clock */
	inst->clk = devm_clk_get(dev, NULL);
	if (!inst->clk)
		goto err;
	clk_prepare_enable(inst->clk);

	/* Finally, do peripheral setup */
	smi_setup_regs(inst);

	platform_set_drvdata(pdev, inst);

	dev_info(inst->dev, "initialised");

	return 0;
err:
	kfree(inst);
	return err;
}

EXPORT_SYMBOL(bcm2835_smi_start);
EXPORT_SYMBOL(my_test);

/****************************************************************************
*
*   bcm2835_smi_remove - called when the driver is unloaded.
*
***************************************************************************/

static int bcm2835_smi_remove(struct platform_device *pdev)
{
	struct bcm2835_smi_instance *inst = platform_get_drvdata(pdev);
	struct device *dev = inst->dev;

	dmaengine_terminate_sync(inst->dma_chan);
	dmaengine_terminate_sync(inst->dma_chan2);

	dma_release_channel(inst->dma_chan);
	dma_release_channel(inst->dma_chan2);

	clk_disable_unprepare(inst->clk);

	dev_info(dev, "SMI device removed - OK");
	return 0;
}

static struct bcm2835_desc *bcm2835_dma_create_cb_chain2(
	struct bcm2835_chan *c,
	enum dma_transfer_direction direction,
	bool cyclic, u32 info, u32 finalextrainfo, size_t frames,
	gfp_t gfp, size_t count)
{
	size_t frame = 0;
	struct bcm2835_desc *d;
	struct bcm2835_cb_entry *cb_entry;
	struct bcm2835_dma_cb *control_block;

	/* allocate and setup the descriptor. */
	d = kzalloc(struct_size(d, cb_list, 3), gfp);
	if (!d)
		return NULL;

	d->c = c;
	d->dir = direction;
	d->cyclic = cyclic;

	cb_entry = &d->cb_list[frame];
	cb_entry->cb = dma_pool_alloc(c->cb_pool, gfp,
			&cb_entry->paddr);

	if (cb_entry->cb == NULL) {
		printk("ca pue");
	}

	/* fill in the control block */
	control_block = cb_entry->cb;
	control_block->info = info;
	control_block->dst = smi_regs_busaddr + SMID;
	control_block->src = inst->user_buffer->buf_virt;
	control_block->length = count;
	control_block->stride = 0;
	control_block->next = 0;
	//printk("ici %x %x %x", control_block->src, control_block->dst, control_block->info);
	if (frame)
		d->cb_list[frame - 1].cb->next = cb_entry->paddr;
	d->frames += 1;
	frame += 1;

	cb_entry = &d->cb_list[frame];
	cb_entry->cb = dma_pool_alloc(c->cb_pool, gfp,
			&cb_entry->paddr);
	
	if (cb_entry->cb == NULL) {
		printk("ca pue");
	}
	/* fill in the control block */
	control_block = cb_entry->cb;
	control_block->info = BCM2835_DMA_WAIT(31);
	control_block->dst = 0x7e007000 + 7*0x100 + BCM2835_DMA_ADDR;
	control_block->src = virt_to_phys(&inst->user_buffer->paddr);
	control_block->length = 4;
	control_block->stride = 0;
	control_block->next = 0;
	//printk("ici %x %x %x %x", *(uint32_t*)control_block->src, control_block->src, control_block->dst, control_block->info);
	if (frame)
		d->cb_list[frame - 1].cb->next = cb_entry->paddr;
	d->frames += 1;
	frame += 1;

	cb_entry = &d->cb_list[frame];
	cb_entry->cb = dma_pool_alloc(c->cb_pool, GFP_NOWAIT,
			&cb_entry->paddr);

	if (cb_entry->cb == NULL) {
		printk("ca pue");
	}

	/* fill in the control block */
	control_block = cb_entry->cb;
	control_block->info = BCM2835_DMA_WAIT(31)|BCM2835_DMA_INT_EN;
	control_block->dst = 0x7e007000 + 7*0x100 + BCM2835_DMA_CS;
	control_block->src = (uint32_t) dummy_enable;
	control_block->length = 4;
	control_block->stride = 0;
	control_block->next = 0;
	//printk("ici %x %x %x", control_block->src, control_block->dst, control_block->info);
	if (frame)
		d->cb_list[frame - 1].cb->next = cb_entry->paddr;
	d->frames += 1;
	frame += 1;

	inst->user_buffer->cb000.cb = NULL;
	inst->user_buffer->cb00.cb = NULL;
	inst->user_buffer->cb01.cb = NULL;
	inst->user_buffer->cb02.cb = NULL;
	inst->user_buffer->cb03.cb = NULL;

	inst->user_buffer->cb1.cb = NULL;
	inst->user_buffer->cb2.cb = NULL;
	inst->user_buffer->cb3.cb = NULL;

	last_control_block = control_block;

	if (false) {
		int j;
		for (j = 0; j< d->frames; j++) {
			printk("cb %x %x %x %x next %x", j, d->cb_list[j].cb->info, d->cb_list[j].cb->src, d->cb_list[j].cb->dst, d->cb_list[j].cb->next);
		}	
	}

	return d;
}


/****************************************************************************
*
*   Register the driver with device tree
*
***************************************************************************/

static const struct of_device_id bcm2835_smi_of_match[] = {
	{.compatible = "brcm,bcm2835-smi",},
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, bcm2835_smi_of_match);

static struct platform_driver bcm2835_smi_driver = {
	.probe = bcm2835_smi_probe,
	.remove = bcm2835_smi_remove,
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = bcm2835_smi_of_match,
		   },
};

module_platform_driver(bcm2835_smi_driver);
MODULE_ALIAS("platform:smi-bcm2835");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Device driver for BCM2835's secondary memory interface");
MODULE_AUTHOR("Luke Wren <luke@raspberrypi.org>");
