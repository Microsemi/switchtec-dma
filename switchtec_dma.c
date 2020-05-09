// SPDX-License-Identifier: GPL-2.0
/*
 * Microchip Switchtec(tm) DMA Controller Driver
 * Copyright (c) 2019, Kelvin Cao <kelvin.cao@microchip.com>
 * Copyright (c) 2019, Microchip Corporation
 */

#include "../drivers/dma/dmaengine.h"

#include <linux/circ_buf.h>
#include <linux/dmaengine.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/delay.h>

#include "version.h"
MODULE_DESCRIPTION("Switchtec PCIe Switch DMA Engine");
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kelvin Cao");

enum switchtec_reg_offset {
	SWITCHTEC_DMAC_VERSION_OFFSET = 0,
	SWITCHTEC_DMAC_CAPABILITY_OFFSET = 0x80,
	SWITCHTEC_DMAC_STATUS_OFFSET = 0x100,
	SWITCHTEC_DMAC_CONTROL_OFFSET = 0x180,
	SWITCHTEC_DMAC_CHAN_CTRL_OFFSET = 0x1000,
	SWITCHTEC_DMAC_CHAN_CFG_STS_OFFSET = 0x160000,
};

#define SWITCHTEC_DESC_MAX_SIZE 0x100000

struct dmac_version_regs {
	u32 fw_ver;
	u32 dma_prot_ver;
	u32 hw_dmac_ver;
} __packed;

struct dmac_capability_regs {
	u32 cap;
	u32 chan_cnt;
	u32 rsvd;
	u32 cplt_tmo;
	u32 tag_limit;
	u16 chan_sts_vec;
	u16 int_err_vec;
	u16 se_buf_cnt;
	u16 se_buf_base;
} __packed;

struct dmac_status_regs {
	u32 state;
	u32 internal_err;
	u32 chan_halt_sum_lo;
	u32 chan_halt_sum_hi;
	u32 rsvd[2];
	u32 chan_paused_sum_lo;
	u32 chan_paused_sum_hi;
} __packed;

struct dmac_control_regs {
	u32 reset_halt;
} __packed;

#define SWITCHTEC_CHAN_CTRL_PAUSE     BIT(0)
#define SWITCHTEC_CHAN_CTRL_HALT      BIT(1)
#define SWITCHTEC_CHAN_CTRL_RESET     BIT(2)
#define SWITCHTEC_CHAN_CTRL_ERR_PAUSE BIT(3)

#define SWITCHTEC_CHAN_STS_HALTED     BIT(10)

struct chan_hw_regs {
	u16 cq_head;
	u16 rsvd1;
	u16 sq_tail;
	u16 rsvd2;
	u8 ctrl;
	u8 rsvd3[3];
	u16 status;
	u16 rsvd4;
} __packed;

enum {
	PERF_BURST_SCALE = 0x1,
	PERF_BURST_SIZE = 0x6,
	PERF_INTERVAL = 0x0,
	PERF_MRRS = 0x3,
	PERF_ARB_WEIGHT = 0x1,
};

enum {
	PERF_BURST_SCALE_SHIFT = 2,
	PERF_BURST_SCALE_MASK = 0x3,
	PERF_MRRS_SHIFT = 4,
	PERF_MRRS_MASK = 0x7,
	PERF_INTERVAL_SHIFT = 8,
	PERF_INTERVAL_MASK = 0x7,
	PERF_BURST_SIZE_SHIFT = 12,
	PERF_BURST_SIZE_MASK = 0x7,
	PERF_ARB_WEIGHT_SHIFT = 24,
	PERF_ARB_WEIGHT_MASK = 0xff,
};

enum {
	PERF_MIN_INTERVAL = 0,
	PERF_MAX_INTERVAL = 7,
	PERF_MIN_BURST_SIZE = 0,
	PERF_MAX_BURST_SIZE = 7,
	PERF_MIN_BURST_SCALE = 0,
	PERF_MAX_BURST_SCALE = 2,
	PERF_MIN_MRRS = 0,
	PERF_MAX_MRRS = 7,
};

enum {
	SE_BUF_BASE_SHIFT = 2,
	SE_BUF_BASE_MASK = 0x1ff,
	SE_BUF_LEN_SHIFT = 12,
	SE_BUF_LEN_MASK = 0x1ff,
	SE_THRESH_SHIFT = 23,
	SE_THRESH_MASK = 0x1ff,
};

#define SWITCHTEC_CHAN_ENABLE BIT(1)

#define SWITCHTEC_LAT_SE_FETCH   BIT(0)
#define SWITCHTEC_LAT_VDM        BIT(1)
#define SWITCHTEC_LAT_RD_IMM     BIT(2)
#define SWITCHTEC_LAT_FW_NP      BIT(3)
#define SWITCHTEC_LAT_SE_PROCESS BIT(4)

struct chan_fw_regs {
	u32 valid_en_se;
	u32 cq_base_lo;
	u32 cq_base_hi;
	u32 cq_size;
	u32 sq_base_lo;
	u32 sq_base_hi;
	u32 sq_size;
	u32 int_vec;
	u32 perf_cfg;
	u32 rsvd1;
	u32 perf_latency_selector;
	u32 perf_fetched_se_cnt_lo;
	u32 perf_fetched_se_cnt_hi;
	u32 perf_byte_cnt_lo;
	u32 perf_byte_cnt_hi;
	u32 rsvd2;
	u16 perf_se_pending;
	u16 perf_se_buf_empty;
	u32 perf_chan_idle;
	u32 perf_lat_max;
	u32 perf_lat_min;
	u32 perf_lat_last;
	u16 sq_current;
	u16 sq_phase;
	u16 cq_current;
	u16 cq_phase;
} __packed;

#define SWITCHTEC_CHAN_INTERVAL 1
#define SWITCHTEC_CHAN_BURST_SZ 1
#define SWITCHTEC_CHAN_BURST_SCALE 1
#define SWITCHTEC_CHAN_MRRS 1

struct switchtec_dma_chan {
	struct switchtec_dma_dev *swdma_dev;
	struct dma_chan dma_chan;
	struct chan_hw_regs __iomem *mmio_chan_hw;
	struct chan_fw_regs __iomem *mmio_chan_fw;

	struct tasklet_struct desc_task;
	spinlock_t ring_lock;
	bool ring_active;
	int cid;

	/* channel index and irq */
	int index;
	int irq;

	int initialized;

	/*
	 * In driver context, head is advanced by producer while
	 * tail is advanced by consumer.
	 */

	/* the head and tail for both desc_ring and hw_sq */
	int head;
	int tail;
	struct switchtec_dma_desc **desc_ring;
	struct switchtec_dma_hw_se_desc *hw_sq;
	dma_addr_t dma_addr_sq;

	/* the head and tail for both hw_cq */
	int cq_head;
	int cq_tail;
	struct switchtec_dma_hw_ce *hw_cq;
	dma_addr_t dma_addr_cq;

	struct kobject config_kobj;
	struct kobject pmon_kobj;
};

struct switchtec_dma_dev {
	struct dma_device dma_dev;
	struct pci_dev __rcu *pdev;
	struct switchtec_dma_chan **swdma_chans;
	int chan_cnt;

	int int_error_irq;
	int chan_status_irq;

	struct dmac_version_regs __iomem *mmio_dmac_ver;
	struct dmac_capability_regs __iomem *mmio_dmac_cap;
	struct dmac_status_regs __iomem *mmio_dmac_status;
	struct dmac_control_regs __iomem *mmio_dmac_ctrl;
	void __iomem *mmio_chan_hw_all;
	void __iomem *mmio_chan_fw_all;

	struct tasklet_struct int_error_task;
	struct tasklet_struct chan_status_task;

	struct kref ref;
	struct work_struct release_work;
};

static struct switchtec_dma_chan *to_switchtec_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct switchtec_dma_chan, dma_chan);
}

static struct device *to_chan_dev(struct switchtec_dma_chan *swdma_chan)
{
	return &swdma_chan->dma_chan.dev->device;
}

enum switchtec_dma_opcode {
	SWITCHTEC_DMA_OPC_MEMCPY = 0x0,
	SWITCHTEC_DMA_OPC_RDIMM = 0x1,
	SWITCHTEC_DMA_OPC_WRIMM = 0x2,
	SWITCHTEC_DMA_OPC_RHI = 0x6,
	SWITCHTEC_DMA_OPC_NOP = 0x7,
};

struct switchtec_dma_hw_se_desc {
	u8 opc;
	u8 ctrl;
	__le16 tlp_setting;
	__le16 rsvd1;
	__le16 cid;
	__le32 byte_cnt;
	__le32 saddr_widata_lo;
	__le32 saddr_widata_hi;
	__le32 daddr_lo;
	__le32 daddr_hi;
	__le16 dfid_connid;
	__le16 sfid;
};

#define SWITCHTEC_SE_LIOF               BIT(6)
#define SWITCHTEC_SE_BRR                BIT(7)
#define SWITCHTEC_SE_CID_MASK           GENMASK(15, 0)

#define SWITCHTEC_CE_SC_LEN_ERR         BIT(0)
#define SWITCHTEC_CE_SC_UR              BIT(1)
#define SWITCHTEC_CE_SC_CA              BIT(2)
#define SWITCHTEC_CE_SC_RSVD_CPL        BIT(3)
#define SWITCHTEC_CE_SC_ECRC_ERR        BIT(4)
#define SWITCHTEC_CE_SC_EP_SET          BIT(5)
#define SWITCHTEC_CE_SC_D_RD_CTO        BIT(8)
#define SWITCHTEC_CE_SC_D_RIMM_UR       BIT(9)
#define SWITCHTEC_CE_SC_D_RIMM_CA       BIT(10)
#define SWITCHTEC_CE_SC_D_RIMM_RSVD_CPL BIT(11)
#define SWITCHTEC_CE_SC_D_ECRC          BIT(12)
#define SWITCHTEC_CE_SC_D_EP_SET        BIT(13)
#define SWITCHTEC_CE_SC_D_BAD_CONNID    BIT(14)
#define SWITCHTEC_CE_SC_D_BAD_RHI_ADDR  BIT(15)
#define SWITCHTEC_CE_SC_D_INVD_CMD      BIT(16)
#define SWITCHTEC_CE_SC_MASK		GENMASK(16, 0)

struct switchtec_dma_hw_ce {
	__le32 rdimm_cpl_dw0;
	__le32 rdimm_cpl_dw1;
	__le32 rsvd1;
	__le32 cpl_byte_cnt;
	__le16 sq_head;
	__le16 rsvd2;
	__le32 rsvd3;
	__le32 sts_code;
	__le16 cid;
	__le16 phase_tag;
};

struct switchtec_dma_desc {
	struct dma_async_tx_descriptor txd;
	struct switchtec_dma_hw_se_desc *hw;
	u32 index;
	u32 orig_size;
	bool completed;
};
void __iomem *global_bar;
#if 0
static struct switchtec_dma_desc *to_switchtec_desc(
		struct dma_async_tx_descriptor *txd)
{
	return container_of(txd, struct switchtec_dma_desc, txd);
}
#endif
#define HALT_RETRY 100
static int halt_channel(struct switchtec_dma_chan *swdma_chan)
{
	u8 ctrl;
	u32 status;
	struct chan_hw_regs *chan_hw = swdma_chan->mmio_chan_hw;
	int retry = HALT_RETRY;
	struct pci_dev *pdev = swdma_chan->swdma_dev->pdev;

	ctrl = readb(&chan_hw->ctrl);

	ctrl |= SWITCHTEC_CHAN_CTRL_HALT | SWITCHTEC_CHAN_CTRL_RESET;
	pci_dbg(pdev, "chan %d: halt channel, ctrl 0x%x\n",
		swdma_chan->index, ctrl);
	writeb(ctrl, &chan_hw->ctrl);

	do {
		status = readl(&chan_hw->status);
		pci_dbg(pdev, "chan %d: status: 0x%x\n",
			swdma_chan->index, status);

		if (status & SWITCHTEC_CHAN_STS_HALTED)
			return 0;
		else
			msleep(1);
	} while (retry--);

	pci_dbg(pdev, "chan %d: halt channel failed\n", swdma_chan->index);
	return 1;
}

static int unhalt_channel(struct switchtec_dma_chan *swdma_chan)
{
	u8 ctrl;
	u32 status;
	struct chan_hw_regs *chan_hw = swdma_chan->mmio_chan_hw;
	int retry = HALT_RETRY;
	struct pci_dev *pdev = swdma_chan->swdma_dev->pdev;

	ctrl = readb(&chan_hw->ctrl);
	ctrl &= ~SWITCHTEC_CHAN_CTRL_HALT;

	pci_dbg(pdev, "chan %d: unhalt channel, ctrl 0x%x\n",
		swdma_chan->index, ctrl);

	writeb(ctrl, &chan_hw->ctrl);

	do {
		status = readl(&chan_hw->status);
		pci_dbg(pdev, "chan %d: status 0x%x\n",
			swdma_chan->index, status);
		if (!(status & SWITCHTEC_CHAN_STS_HALTED)) {
			pci_dbg(pdev, "chan %d: unhaletd\n", swdma_chan->index);
			return 0;
		} else
			msleep(1);
	} while (retry--);

	pci_dbg(pdev, "chan %d: unhalt channel failed\n", swdma_chan->index);
	return 1;
}

static int reset_channel(struct switchtec_dma_chan *swdma_chan)
{
	u8 ctrl;
	struct chan_hw_regs *chan_hw = swdma_chan->mmio_chan_hw;
	struct pci_dev *pdev = swdma_chan->swdma_dev->pdev;

	ctrl = SWITCHTEC_CHAN_CTRL_RESET;
	writel(ctrl, &chan_hw->ctrl);

	msleep(1);

	ctrl = 0;
	writel(ctrl, &chan_hw->ctrl);

	pci_dbg(pdev, "chan %d: reset channel, ctrl 0x%x\n",
		swdma_chan->index, ctrl);

	return 0;
}

static int enable_channel(struct switchtec_dma_chan *swdma_chan)
{
	u32 valid_en_se;
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	struct pci_dev *pdev = swdma_chan->swdma_dev->pdev;

	valid_en_se = readl(&chan_fw->valid_en_se);
	valid_en_se |= SWITCHTEC_CHAN_ENABLE;

	writel(valid_en_se, &chan_fw->valid_en_se);
	pci_dbg(pdev, "chan %d: enable channel, valid_en_se 0x%x\n",
		swdma_chan->index, valid_en_se);

	return 0;
}

#define SWITCHTEC_DMA_SQ_SIZE SZ_32K
#define SWITCHTEC_DMA_CQ_SIZE SZ_32K

#define SWITCHTEC_DMA_RING_SIZE SWITCHTEC_DMA_SQ_SIZE

static struct switchtec_dma_desc *switchtec_dma_get_desc(
		struct switchtec_dma_chan *swdma_chan, int i)
{
	return swdma_chan->desc_ring[i];
}

static struct switchtec_dma_hw_ce * switchtec_dma_get_ce(
		struct switchtec_dma_chan *swdma_chan, int i)
{
	return &swdma_chan->hw_cq[i];
}

static void switchtec_dma_process_desc(struct switchtec_dma_chan *swdma_chan)
{
	struct device *chan_dev = to_chan_dev(swdma_chan);
	struct dmaengine_result res;
	struct switchtec_dma_desc *desc, *cur_desc;
	static struct switchtec_dma_hw_ce *ce;
	u16 cq_head;
	int cid;
	int se_idx;
	int cnt = 0;
	int i = 0;
	int *p;

	spin_lock_bh(&swdma_chan->ring_lock);

	cq_head = readw(&swdma_chan->mmio_chan_fw->cq_current);
	dev_dbg(chan_dev, "cq_head is %x\n", cq_head);
	while ((cnt = CIRC_CNT(cq_head, swdma_chan->cq_tail,
			       SWITCHTEC_DMA_CQ_SIZE)) >= 1) {
		ce = switchtec_dma_get_ce(swdma_chan, swdma_chan->cq_tail);
		cid = le16_to_cpu(ce->cid);
		se_idx = cid & (SWITCHTEC_DMA_SQ_SIZE - 1);
		desc = switchtec_dma_get_desc(swdma_chan, se_idx);
		cur_desc = switchtec_dma_get_desc(swdma_chan, swdma_chan->tail);

		dev_dbg(to_chan_dev(swdma_chan),
			"ooo_dbg: current CE (cid: %x, cookie: %x)",
			cid, desc->txd.cookie);

		res.residue = desc->orig_size - ce->cpl_byte_cnt;
		p = (int *)ce;
		for (i = 0; i < sizeof(*ce)/4; i++) {
			dev_dbg(chan_dev, "CE DW%d: 0x%08x\n", i,
				le32_to_cpu(*p));
			p++;
		}

		if (!(ce->sts_code & SWITCHTEC_CE_SC_MASK)) {
			dev_dbg(chan_dev,"CID 0x%04x Success\n", cid);
			dev_dbg(chan_dev, "Requested byte count: 0x%08x\n",
				desc->orig_size);
			dev_dbg(chan_dev, "Completed byte count: 0x%08x\n",
				ce->cpl_byte_cnt);
			res.result = DMA_TRANS_NOERROR;
		} else if (ce->sts_code & SWITCHTEC_CE_SC_D_RD_CTO) {
			dev_err(chan_dev,
				"CID 0x%04x Read failed, SC 0x%08x\n", cid,
				(u32)(ce->sts_code & SWITCHTEC_CE_SC_MASK));
			res.result = DMA_TRANS_READ_FAILED;
		} else {
			dev_err(chan_dev,
				"CID 0x%04x Write failed, SC 0x%08x\n", cid,
				(u32)(ce->sts_code & SWITCHTEC_CE_SC_MASK));
			res.result = DMA_TRANS_WRITE_FAILED;
		}

		desc->completed = true;

		swdma_chan->cq_tail++;
		swdma_chan->cq_tail &= SWITCHTEC_DMA_CQ_SIZE - 1;
		writew(swdma_chan->cq_tail, &swdma_chan->mmio_chan_hw->cq_head);

		if (se_idx != swdma_chan->tail) {
			dev_dbg(to_chan_dev(swdma_chan),
				"ooo_dbg: out of order CE! current CE (cid: %x), current SE (cid: %x)",
				cid, le16_to_cpu(cur_desc->hw->cid));
			continue;
		}

		do {
			dma_cookie_complete(&desc->txd);
			dma_descriptor_unmap(&desc->txd);
			dmaengine_desc_get_callback_invoke(&desc->txd, &res);
			desc->txd.callback = NULL;
			desc->txd.callback_result = NULL;
			desc->completed = false;

			swdma_chan->tail++;
			swdma_chan->tail &= SWITCHTEC_DMA_SQ_SIZE - 1;
			desc = switchtec_dma_get_desc(swdma_chan,
						      swdma_chan->tail);
			if (!desc->completed)
				break;
		} while (CIRC_CNT(swdma_chan->head, swdma_chan->tail,
				  SWITCHTEC_DMA_SQ_SIZE));

		dev_dbg(to_chan_dev(swdma_chan), "ooo_dbg: next SE (cid: %x)",
			le16_to_cpu(desc->hw->cid));

	}
	spin_unlock_bh(&swdma_chan->ring_lock);
}

static void switchtec_dma_abort_desc(struct switchtec_dma_chan *swdma_chan)
{
	struct dmaengine_result res;
	struct switchtec_dma_desc *desc;

	switchtec_dma_process_desc(swdma_chan);

	spin_lock_bh(&swdma_chan->ring_lock);

	while (CIRC_CNT(swdma_chan->head, swdma_chan->tail,
			SWITCHTEC_DMA_SQ_SIZE) >= 1) {
		dev_dbg(to_chan_dev(swdma_chan), "abort descriptor: 0x%x\n",
			swdma_chan->tail);
		desc = switchtec_dma_get_desc(swdma_chan, swdma_chan->tail);

		res.residue = desc->orig_size;
		res.result = DMA_TRANS_ABORTED;

		dma_cookie_complete(&desc->txd);
		dma_descriptor_unmap(&desc->txd);
		dmaengine_desc_get_callback_invoke(&desc->txd, &res);
		desc->txd.callback = NULL;
		desc->txd.callback_result = NULL;

		swdma_chan->tail++;
	}

	spin_unlock_bh(&swdma_chan->ring_lock);
}

static void __switchtec_dma_chan_stop(struct switchtec_dma_chan *swdma_chan)
{
	dev_dbg(to_chan_dev(swdma_chan), "\n");

	writeb(SWITCHTEC_CHAN_CTRL_HALT, &swdma_chan->mmio_chan_hw->ctrl);

	writel(0, &swdma_chan->mmio_chan_fw->sq_base_lo);
	writel(0, &swdma_chan->mmio_chan_fw->sq_base_hi);
	writel(0, &swdma_chan->mmio_chan_fw->cq_base_lo);
	writel(0, &swdma_chan->mmio_chan_fw->cq_base_hi);
}

static void switchtec_dma_chan_stop(struct switchtec_dma_chan *swdma_chan)
{
	dev_dbg(to_chan_dev(swdma_chan), "\n");
	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		rcu_read_unlock();
		return;
	}

	__switchtec_dma_chan_stop(swdma_chan);

	rcu_read_unlock();
}

#if 0
static void switchtec_dma_stop(struct switchtec_dma_dev *swdma_dev)
{
	rcu_read_lock();
	if (!rcu_dereference(swdma_dev->pdev)) {
		rcu_read_unlock();
		return;
	}

	__switchtec_dma_stop(swdma_dev);

	rcu_read_unlock();
}
#endif
static void switchtec_dma_desc_task(unsigned long data)
{
	struct switchtec_dma_chan *swdma_chan = (void *)data;

	switchtec_dma_process_desc(swdma_chan);
}

static void switchtec_dma_int_error_task(unsigned long data)
{
//	struct switchtec_dma_dev *swdma_dev = (void *)data;
}

static void switchtec_dma_chan_status_task(unsigned long data)
{
//	struct switchtec_dma_dev *swdma_dev = (void *)data;
}

static struct dma_async_tx_descriptor *switchtec_dma_prep_memcpy(
		struct dma_chan *c, dma_addr_t dma_dst, dma_addr_t dma_src,
		size_t len, unsigned long flags)
	__acquires(swdma_chan->ring_lock)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(c);
	struct device *chan_dev = to_chan_dev(swdma_chan);
	struct switchtec_dma_desc *desc;

	dev_dbg(chan_dev, "\n");

	spin_lock_bh(&swdma_chan->ring_lock);
	if (!swdma_chan->ring_active)
		goto err_unlock;

	if (!CIRC_SPACE(swdma_chan->head, swdma_chan->tail,
			SWITCHTEC_DMA_RING_SIZE))
		goto err_unlock;

	if (len > SWITCHTEC_DESC_MAX_SIZE)
		goto err_unlock;

	desc = switchtec_dma_get_desc(swdma_chan, swdma_chan->head);

	desc->completed = false;
	desc->hw->opc = SWITCHTEC_DMA_OPC_MEMCPY;
	desc->hw->daddr_lo = cpu_to_le32(lower_32_bits(dma_dst));
	desc->hw->daddr_hi = cpu_to_le32(upper_32_bits(dma_dst));
	desc->hw->saddr_widata_lo = cpu_to_le32(lower_32_bits(dma_src));
	desc->hw->saddr_widata_hi = cpu_to_le32(upper_32_bits(dma_src));
	desc->hw->byte_cnt = cpu_to_le32(len);
	desc->hw->tlp_setting = 0;
	swdma_chan->cid &= SWITCHTEC_SE_CID_MASK;
	desc->hw->cid = cpu_to_le16(swdma_chan->cid++);
	desc->index = swdma_chan->head;

	dev_dbg(chan_dev, "SE SADDR : 0x%08x_%08x\n",
		desc->hw->saddr_widata_hi,
		desc->hw->saddr_widata_lo);
	dev_dbg(chan_dev, "SE DADDR : 0x%08x_%08x\n",
		desc->hw->daddr_hi, desc->hw->daddr_lo);
	dev_dbg(chan_dev, "SE BCOUNT: 0x%08x\n", desc->hw->byte_cnt);

	desc->orig_size = len;

	if (flags & DMA_PREP_INTERRUPT)
		desc->hw->ctrl |= SWITCHTEC_SE_LIOF;

	if (flags & DMA_PREP_FENCE)
		desc->hw->ctrl |= SWITCHTEC_SE_BRR;

	desc->txd.flags = flags;

	swdma_chan->head++;
	if (swdma_chan->head == SWITCHTEC_DMA_RING_SIZE)
		swdma_chan->head = 0;

	/* return with the lock held, it will be released in tx_submit */

	return &desc->txd;

err_unlock:
	/*
	 * Keep sparse happy by restoring an even lock count on
	 * this lock.
	 */
	__acquire(swdma_chan->ring_lock);

	spin_unlock_bh(&swdma_chan->ring_lock);
	return NULL;
}

struct dma_async_tx_descriptor *switchtec_dma_prep_wimm_data(
		struct dma_chan *c, dma_addr_t dst, u64 data,
		unsigned long flags)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(c);
	struct device *chan_dev = to_chan_dev(swdma_chan);
	struct switchtec_dma_desc *desc;

	dev_dbg(chan_dev, "\n");

	spin_lock_bh(&swdma_chan->ring_lock);
	if (!swdma_chan->ring_active)
		goto err_unlock;

	if (!CIRC_SPACE(swdma_chan->head, swdma_chan->tail,
			SWITCHTEC_DMA_RING_SIZE))
		goto err_unlock;

	desc = switchtec_dma_get_desc(swdma_chan, swdma_chan->head);

	desc->completed = false;
	desc->hw->opc = SWITCHTEC_DMA_OPC_WRIMM;
	desc->hw->daddr_lo = cpu_to_le32(lower_32_bits(dst));
	desc->hw->daddr_hi = cpu_to_le32(upper_32_bits(dst));
	desc->hw->saddr_widata_lo = cpu_to_le32(lower_32_bits(data));
	desc->hw->saddr_widata_hi = cpu_to_le32(upper_32_bits(data));
	desc->hw->byte_cnt = cpu_to_le32(8);
	desc->hw->tlp_setting = 0;
	swdma_chan->cid &= SWITCHTEC_SE_CID_MASK;
	desc->hw->cid = cpu_to_le16(swdma_chan->cid++);
	desc->index = swdma_chan->head;

	dev_dbg(chan_dev, "SE WIMM Data: 0x%08x_%08x\n",
		desc->hw->saddr_widata_hi,
		desc->hw->saddr_widata_lo);
	dev_dbg(chan_dev, "SE DADDR    : 0x%08x_%08x\n",
		desc->hw->daddr_hi, desc->hw->daddr_lo);
	dev_dbg(chan_dev, "SE BCOUNT   : 0x%08x\n", desc->hw->byte_cnt);

	desc->orig_size = 8;

	if (flags & DMA_PREP_INTERRUPT)
		desc->hw->ctrl |= SWITCHTEC_SE_LIOF;

	if (flags & DMA_PREP_FENCE)
		desc->hw->ctrl |= SWITCHTEC_SE_BRR;

	desc->txd.flags = flags;

	swdma_chan->head++;
	swdma_chan->head &= SWITCHTEC_DMA_RING_SIZE - 1;

	/* return with the lock held, it will be released in tx_submit */

	return &desc->txd;

err_unlock:
	/*
	 * Keep sparse happy by restoring an even lock count on
	 * this lock.
	 */
	__acquire(swdma_chan->ring_lock);

	spin_unlock_bh(&swdma_chan->ring_lock);
	return NULL;
}

static dma_cookie_t switchtec_dma_tx_submit(
		struct dma_async_tx_descriptor *desc)
	__releases(swdma_chan->ring_lock)
{
	struct switchtec_dma_chan *swdma_chan =
		to_switchtec_dma_chan(desc->chan);
	dma_cookie_t cookie;

	cookie = dma_cookie_assign(desc);

	dev_dbg(to_chan_dev(swdma_chan), "ooo_dbg: submit SE (cookie: %x)\n", cookie);

	spin_unlock_bh(&swdma_chan->ring_lock);

	return cookie;
}

static enum dma_status switchtec_dma_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	enum dma_status ret;

	dev_dbg(to_chan_dev(swdma_chan), "\n");

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_COMPLETE)
		return ret;

	switchtec_dma_process_desc(swdma_chan);

	return dma_cookie_status(chan, cookie, txstate);
}

static void switchtec_dma_issue_pending(struct dma_chan *chan)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct switchtec_dma_dev *swdma_dev = swdma_chan->swdma_dev;

	rcu_read_lock();
	if (!rcu_dereference(swdma_dev->pdev)) {
		rcu_read_unlock();
		return;
	}

	/*
	 * Ensure the desc updates are visible before starting the
	 * DMA engine.
	 */
	wmb();

	/*
	 * The sq_tail register is actually for the head of the
	 * submisssion queue. Chip has the opposite define of head/tail
	 * to the Linux kernel.
	 */
	dev_dbg(to_chan_dev(swdma_chan), "HEAD 0x%x\n", swdma_chan->head);

	writew(swdma_chan->head, &swdma_chan->mmio_chan_hw->sq_tail);
	dev_dbg(to_chan_dev(swdma_chan),
		"Update SE HEAD 0x%x to firmware.\n", swdma_chan->head);
	rcu_read_unlock();
}

static irqreturn_t switchtec_dma_isr(int irq, void *chan)
{
	struct switchtec_dma_chan *swdma_chan = chan;

	if (swdma_chan->ring_active)
		tasklet_schedule(&swdma_chan->desc_task);

	return IRQ_HANDLED;
}

static irqreturn_t switchtec_dma_int_error_isr(int irq, void *dma)
{
	struct switchtec_dma_dev *swdma_dev = dma;

	tasklet_schedule(&swdma_dev->int_error_task);

	return IRQ_HANDLED;
}

static irqreturn_t switchtec_dma_chan_status_isr(int irq, void *dma)
{
	struct switchtec_dma_dev *swdma_dev = dma;

	tasklet_schedule(&swdma_dev->chan_status_task);

	return IRQ_HANDLED;
}

static void switchtec_dma_release_work(struct work_struct *work)
{
	struct switchtec_dma_dev *swdma_dev =
		container_of(work,struct switchtec_dma_dev, release_work);

	dev_dbg(swdma_dev->dma_dev.dev, "\n");

	dma_async_device_unregister(&swdma_dev->dma_dev);
//	switchtec_kobject_del(swdma_dev);
	put_device(swdma_dev->dma_dev.dev);
}

static void switchtec_dma_release(struct kref *ref)
{
	struct switchtec_dma_dev *swdma_dev =
		container_of(ref,struct switchtec_dma_dev, ref);

	dev_dbg(swdma_dev->dma_dev.dev, "\n");
	/*
	 * The dmaengine reference counting and locking is a bit of a
	 * mess so we have to work around it a bit here. We might put
	 * the reference while the dmaengine holds the dma_list_mutex
	 * which means we can't call dma_async_device_unregister() directly
	 * here and it must be delayed.
	 */
	schedule_work(&swdma_dev->release_work);
}

static void switchtec_dma_put(struct switchtec_dma_dev *swdma_dev)
{
	dev_dbg(swdma_dev->dma_dev.dev, "\n");
	kref_put(&swdma_dev->ref, switchtec_dma_release);
}

static void switchtec_dma_free_desc(struct switchtec_dma_chan *swdma_chan)
{
	int i;
        size_t size;

	size = SWITCHTEC_DMA_SQ_SIZE * sizeof(*swdma_chan->hw_sq);
	if (swdma_chan->hw_sq)
		dmam_free_coherent(&swdma_chan->swdma_dev->pdev->dev, size,
				   swdma_chan->hw_sq, swdma_chan->dma_addr_sq);

	size = SWITCHTEC_DMA_CQ_SIZE * sizeof(*swdma_chan->hw_cq);
	if (swdma_chan->hw_cq)
		dmam_free_coherent(&swdma_chan->swdma_dev->pdev->dev, size,
				   swdma_chan->hw_cq, swdma_chan->dma_addr_cq);

	if (swdma_chan->desc_ring) {
		for (i = 0; i < SWITCHTEC_DMA_RING_SIZE; i++)
			if (swdma_chan->desc_ring[i])
				kfree(swdma_chan->desc_ring[i]);

		kfree(swdma_chan->desc_ring);
	}
}

static int switchtec_dma_alloc_desc(struct switchtec_dma_chan *swdma_chan)
{
	struct switchtec_dma_dev *swdma_dev = swdma_chan->swdma_dev;
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	size_t size;
	struct switchtec_dma_desc *desc;
	int i;

	swdma_chan->head = swdma_chan->tail = 0;
	swdma_chan->cq_head = swdma_chan->cq_tail = 0;

	size = SWITCHTEC_DMA_SQ_SIZE * sizeof(*swdma_chan->hw_sq);
	swdma_chan->hw_sq = dmam_alloc_coherent(&swdma_dev->pdev->dev, size,
						&swdma_chan->dma_addr_sq,
						GFP_KERNEL);
	if (!swdma_chan->hw_sq)
		goto free_and_exit;

	size = SWITCHTEC_DMA_CQ_SIZE * sizeof(*swdma_chan->hw_cq);
	swdma_chan->hw_cq = dmam_alloc_coherent(&swdma_dev->pdev->dev, size,
						&swdma_chan->dma_addr_cq,
						GFP_KERNEL);
	if (!swdma_chan->hw_cq)
		goto free_and_exit;

	size = sizeof(*swdma_chan->desc_ring);
	swdma_chan->desc_ring = kcalloc(SWITCHTEC_DMA_RING_SIZE,
					size, GFP_KERNEL);
	if (!swdma_chan->desc_ring)
		goto free_and_exit;

	memset(swdma_chan->desc_ring, 0, SWITCHTEC_DMA_RING_SIZE * size);

	for (i = 0; i < SWITCHTEC_DMA_RING_SIZE; i++) {
		desc = kzalloc(sizeof(*desc), GFP_KERNEL);
		if (!desc)
			goto free_and_exit;

		dma_async_tx_descriptor_init(&desc->txd, &swdma_chan->dma_chan);
		desc->txd.tx_submit = switchtec_dma_tx_submit;
		desc->hw = &swdma_chan->hw_sq[i];
		desc->completed = true;

		swdma_chan->desc_ring[i] = desc;
	}

	/* set sq/cq */
	writel(lower_32_bits(swdma_chan->dma_addr_sq), &chan_fw->sq_base_lo);
	writel(upper_32_bits(swdma_chan->dma_addr_sq), &chan_fw->sq_base_hi);
	writel(lower_32_bits(swdma_chan->dma_addr_cq), &chan_fw->cq_base_lo);
	writel(upper_32_bits(swdma_chan->dma_addr_cq), &chan_fw->cq_base_hi);

	writel(cpu_to_le16(SWITCHTEC_DMA_SQ_SIZE),
	       &swdma_chan->mmio_chan_fw->sq_size);
	writel(cpu_to_le16(SWITCHTEC_DMA_CQ_SIZE),
	       &swdma_chan->mmio_chan_fw->cq_size);

	return 0;

free_and_exit:
	switchtec_dma_free_desc(swdma_chan);

	return -ENOMEM;
}

static int switchtec_dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct switchtec_dma_dev *swdma_dev = swdma_chan->swdma_dev;
	u32 perf_cfg;
	int rc;

	dev_dbg(to_chan_dev(swdma_chan), "\n");

	rcu_read_lock();
	if (!rcu_dereference(swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	kref_get(&swdma_dev->ref);

	rc = switchtec_dma_alloc_desc(swdma_chan);
	if (rc) {
		switchtec_dma_put(swdma_dev);
		rcu_read_unlock();
		return rc;
	}

	enable_channel(swdma_chan);
	reset_channel(swdma_chan);
	unhalt_channel(swdma_chan);

	swdma_chan->ring_active = true;
	swdma_chan->cid = 0;

	dma_cookie_init(chan);

	perf_cfg = readl(&swdma_chan->mmio_chan_fw->perf_cfg);

	dev_info(&chan->dev->device, "Burst Size:  0x%x",
		 (perf_cfg >> PERF_BURST_SIZE_SHIFT) & PERF_BURST_SIZE_MASK);

	dev_info(&chan->dev->device, "Burst Scale: 0x%x",
		 (perf_cfg >> PERF_BURST_SCALE_SHIFT) & PERF_BURST_SCALE_MASK);

	dev_info(&chan->dev->device, "Interval:    0x%x",
		 (perf_cfg >> PERF_INTERVAL_SHIFT) & PERF_INTERVAL_MASK);

	dev_info(&chan->dev->device, "Arb Weight:  0x%x",
		 (perf_cfg >> PERF_ARB_WEIGHT_SHIFT) & PERF_ARB_WEIGHT_MASK);

	dev_info(&chan->dev->device, "MRRS:        0x%x",
		 (perf_cfg >> PERF_MRRS_SHIFT) & PERF_MRRS_MASK);

	rcu_read_unlock();
	return SWITCHTEC_DMA_SQ_SIZE;
}

static void switchtec_dma_free_chan_resources(struct dma_chan *chan)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 valid_en_se;
	struct pci_dev *pdev;

	dev_dbg(to_chan_dev(swdma_chan), "\n");

	spin_lock_bh(&swdma_chan->ring_lock);
	swdma_chan->ring_active = false;
	spin_unlock_bh(&swdma_chan->ring_lock);

	switchtec_dma_chan_stop(swdma_chan);

	rcu_read_lock();
	pdev = rcu_dereference(swdma_chan->swdma_dev->pdev);
	if (pdev)
		synchronize_irq(swdma_chan->irq);
	rcu_read_unlock();

	tasklet_kill(&swdma_chan->desc_task);

	switchtec_dma_abort_desc(swdma_chan);

	switchtec_dma_free_desc(swdma_chan);

	/* Disable the channle */
	valid_en_se = readl(&chan_fw->valid_en_se);
	valid_en_se &= ~SWITCHTEC_CHAN_ENABLE;

#if 0
	writel(valid_en_se, &chan_fw->valid_en_se);

#endif

	switchtec_dma_put(swdma_chan->swdma_dev);
}

#define SWITCHTEC_DMA_CHAN_HW_REGS_SIZE 0x1000
#define SWITCHTEC_DMA_CHAN_FW_REGS_SIZE 0x80

struct kobj_type switchtec_config_ktype;

static int switchtec_dma_chan_init(struct switchtec_dma_dev *swdma_dev, int i)
{
	struct dma_device *dma = &swdma_dev->dma_dev;
	struct device *dev = &swdma_dev->pdev->dev;
	struct dma_chan *chan;
	struct switchtec_dma_chan *swdma_chan;
	struct chan_fw_regs *chan_fw;
	struct chan_hw_regs *chan_hw;
	u32 perf_cfg = 0;
	u32 valid_en_se;
	u32 thresh;
	int se_buf_len;
	int irq;
	int rc;
	size_t offset;

	swdma_chan = devm_kzalloc(dev, sizeof(*swdma_chan), GFP_KERNEL);
	if (!swdma_chan)
		return -ENOMEM;

	swdma_chan->index = i;

	offset =  i * SWITCHTEC_DMA_CHAN_FW_REGS_SIZE;
	chan_fw = (struct chan_fw_regs *)(swdma_dev->mmio_chan_fw_all + offset);

	offset =  i * SWITCHTEC_DMA_CHAN_HW_REGS_SIZE;
	chan_hw = (struct chan_hw_regs *)(swdma_dev->mmio_chan_hw_all + offset);

	swdma_dev->swdma_chans[i] = swdma_chan;
	swdma_chan->mmio_chan_fw = chan_fw;
	swdma_chan->mmio_chan_hw = chan_hw;
	swdma_chan->swdma_dev = swdma_dev;

	/* halt channel first */
	rc = halt_channel(swdma_chan);
	if (rc) {
		dev_err(dev, "Chan %d: halt failed\n", i);
		return -EIO;
	}

	perf_cfg = readl(&chan_fw->perf_cfg);
	dev_dbg(dev, "Chan %d: perf_cfg 0x%x (read)\n", i, perf_cfg);

	/* init perf tuner */
	perf_cfg = PERF_BURST_SCALE << PERF_BURST_SCALE_SHIFT;
	perf_cfg |= PERF_MRRS << PERF_MRRS_SHIFT;
	perf_cfg |= PERF_INTERVAL << PERF_INTERVAL_SHIFT;
	perf_cfg |= PERF_BURST_SIZE << PERF_BURST_SIZE_SHIFT;
	perf_cfg |= PERF_ARB_WEIGHT << PERF_ARB_WEIGHT_SHIFT;

	dev_dbg(dev, "Chan %d: perf_cfg 0x%x (write)\n", i, perf_cfg);
	writel(perf_cfg, &chan_fw->perf_cfg);

	valid_en_se = readl(&chan_fw->valid_en_se);

	dev_dbg(dev, "Chan %d: valid_en_se 0x%x (read)\n", i, valid_en_se);
	dev_dbg(dev, "Chan %d: SE buf base 0x%x (read)\n",
		i, (valid_en_se >> SE_BUF_BASE_SHIFT) & SE_BUF_BASE_MASK);

	se_buf_len = (valid_en_se >> SE_BUF_LEN_SHIFT) & SE_BUF_LEN_MASK;
	dev_dbg(dev, "Chan %d: SE buf cnt  0x%x (read)\n", i, se_buf_len);

	thresh = se_buf_len / 2;
	valid_en_se |= (thresh & SE_THRESH_MASK) << SE_THRESH_SHIFT;
	writel(valid_en_se , &chan_fw->valid_en_se);

	/* request irqs */
	irq = readl(&chan_fw->int_vec);
	dev_dbg(dev, "Chan %d: irq vec 0x%x\n", i, irq);

	irq = pci_irq_vector(swdma_dev->pdev, irq);
	if (irq < 0)
		return irq;

	rc = devm_request_irq(dev, irq, switchtec_dma_isr, 0, KBUILD_MODNAME,
			      swdma_chan);
	if (rc)
		return rc;

	swdma_chan->irq = irq;

	spin_lock_init(&swdma_chan->ring_lock);
	tasklet_init(&swdma_chan->desc_task, switchtec_dma_desc_task,
		     (unsigned long)swdma_chan);

	chan = &swdma_chan->dma_chan;
	chan->device = dma;
	dma_cookie_init(chan);
	list_add_tail(&chan->device_node, &dma->channels);

	swdma_chan->initialized = 1;

	return 0;
}

void switchtec_chan_kobject_del(struct switchtec_dma_chan *swdma_chan);

static int switchtec_dma_chan_free(struct switchtec_dma_chan *swdma_chan)
{
	struct device *chan_dev = to_chan_dev(swdma_chan);
	struct device *dev = &swdma_chan->swdma_dev->pdev->dev;
	spin_lock_bh(&swdma_chan->ring_lock);
	swdma_chan->ring_active = false;
	swdma_chan->initialized = false;
	spin_unlock_bh(&swdma_chan->ring_lock);

	dev_dbg(chan_dev, "free_irq: 0x%x\n", swdma_chan->irq);
	devm_free_irq(dev, swdma_chan->irq, swdma_chan);

	switchtec_chan_kobject_del(swdma_chan);

	__switchtec_dma_chan_stop(swdma_chan);

	return 0;
}

static int switchtec_dma_chans_release(struct switchtec_dma_dev *swdma_dev)
{
	int i;

	for (i = 0; i < swdma_dev->dma_dev.chancnt; i++)
		switchtec_dma_chan_free(swdma_dev->swdma_chans[i]);

	return 0;
}

static int switchtec_dma_get_chan_cnt(struct switchtec_dma_dev *swdma_dev)
{
	return le32_to_cpu(readl(&swdma_dev->mmio_dmac_cap->chan_cnt));
}

static int switchtec_dma_chans_enumerate(struct switchtec_dma_dev *swdma_dev,
					 int chan_cnt)
{
	struct dma_device *dma = &swdma_dev->dma_dev;
	struct device *dev = &swdma_dev->pdev->dev;
	int base;
	int cnt;
	int rc;
	int i;

	swdma_dev->swdma_chans = devm_kcalloc(dev, chan_cnt,
					      sizeof(*swdma_dev->swdma_chans),
					      GFP_KERNEL);

	if (!swdma_dev->swdma_chans)
		return -ENOMEM;

	base = readw(&swdma_dev->mmio_dmac_cap->se_buf_base);
	cnt = readw(&swdma_dev->mmio_dmac_cap->se_buf_cnt);
	dev_dbg(dev, "EP SE buf base: 0x%x\n", base);
	dev_dbg(dev, "EP SE buf cnt:  0x%x\n", cnt);

	INIT_LIST_HEAD(&dma->channels);

	for (i = 0; i < chan_cnt; i++) {
		rc = switchtec_dma_chan_init(swdma_dev, i);
		if (rc) {
			dev_err(dev, "Chan %d: init failed\n", i);
			goto error_exit;
		}
	}

	dma->chancnt = chan_cnt;

	return dma->chancnt;

error_exit:
	switchtec_dma_chans_release(swdma_dev);
	return -ENXIO;
}

struct switchtec_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct dma_chan *, char *);
	ssize_t (*store)(struct dma_chan *, const char *, size_t);
};

static ssize_t burst_scale_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 perf_cfg = 0;

	perf_cfg = readl(&chan_fw->perf_cfg);
	perf_cfg >>= PERF_BURST_SCALE_SHIFT;
	perf_cfg &= PERF_BURST_SCALE_MASK;

	return sprintf(page, "0x%x\n", perf_cfg);
}

static ssize_t burst_scale_store(struct dma_chan *chan, const char *page,
				 size_t count)
{
	int burst_scale;
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 perf_cfg;
	ssize_t ret = count;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		goto err_unlock;
	}

	if (chan->client_count) {
		goto err_unlock;
	}

	if (kstrtoint(page, 0, &burst_scale) == 0) {
		if (burst_scale < PERF_MIN_BURST_SCALE ||
		    burst_scale > PERF_MAX_BURST_SCALE) {
			ret = -EINVAL;
			goto err_unlock;
		}
		perf_cfg = readl(&chan_fw->perf_cfg);
		perf_cfg &= ~(PERF_BURST_SCALE_MASK << PERF_BURST_SCALE_SHIFT);
		perf_cfg |= burst_scale << PERF_BURST_SCALE_SHIFT;
		writel(perf_cfg, &chan_fw->perf_cfg);
	}

err_unlock:
	rcu_read_unlock();

	return ret;
}
struct switchtec_sysfs_entry burst_scale_attr = __ATTR_RW(burst_scale);

static ssize_t burst_size_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 burst_size = 0;

	burst_size = readl(&chan_fw->perf_cfg);
	burst_size >>= PERF_BURST_SIZE_SHIFT;
	burst_size &= PERF_BURST_SIZE_MASK;

	return sprintf(page, "0x%x\n", burst_size);
}

static ssize_t burst_size_store(struct dma_chan *chan, const char *page,
				size_t count)
{
	int burst_size;
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 perf_cfg;
	ssize_t ret = count;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		ret = -ENODEV;
		goto err_unlock;
	}

	if (chan->client_count) {
		ret = -EBUSY;
		goto err_unlock;
	}

	if (kstrtoint(page, 0, &burst_size) == 0) {
		if (burst_size < PERF_MIN_BURST_SIZE ||
		    burst_size > PERF_MAX_BURST_SIZE) {
			ret = -EINVAL;
			goto err_unlock;
		}
		perf_cfg = readl(&chan_fw->perf_cfg);
		perf_cfg &= ~(PERF_BURST_SIZE_MASK << PERF_BURST_SIZE_SHIFT);
		perf_cfg |= burst_size << PERF_BURST_SIZE_SHIFT;
		writel(perf_cfg, &chan_fw->perf_cfg);
	}

err_unlock:
	rcu_read_unlock();

	return ret;
}


struct switchtec_sysfs_entry burst_size_attr = __ATTR_RW(burst_size);

static ssize_t arb_weight_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 weight = 0;

	weight = readl(&chan_fw->perf_cfg);
	weight >>= PERF_ARB_WEIGHT_SHIFT;
	weight &= PERF_ARB_WEIGHT_MASK;

	return sprintf(page, "0x%x\n", weight);
}

static ssize_t arb_weight_store(struct dma_chan *chan, const char *page,
				size_t count)
{
	int weight;
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 perf_cfg;
	ssize_t ret = count;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		ret = -ENODEV;
		goto err_unlock;
	}

	if (chan->client_count) {
		ret = -EBUSY;
		goto err_unlock;
	}

	if (kstrtoint(page, 0, &weight) == 0) {
		if (weight < 0) {
			ret = -EINVAL;
			goto err_unlock;
		}
		perf_cfg = readl(&chan_fw->perf_cfg);
		perf_cfg &= ~(PERF_ARB_WEIGHT_MASK << PERF_ARB_WEIGHT_SHIFT);
		perf_cfg |= weight << PERF_ARB_WEIGHT_SHIFT;
		writel(perf_cfg, &chan_fw->perf_cfg);
	}

err_unlock:
	rcu_read_unlock();

	return ret;
}
struct switchtec_sysfs_entry arb_weight_attr = __ATTR_RW(arb_weight);

static ssize_t interval_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 interval = 0;

	interval = readl(&chan_fw->perf_cfg);
	interval >>= PERF_INTERVAL_SHIFT;
	interval &= PERF_INTERVAL_MASK;

	return sprintf(page, "%x\n", interval);
}

static ssize_t interval_store(struct dma_chan *chan, const char *page,
			      size_t count)
{
	int interval;
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 perf_cfg;
	ssize_t ret = count;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		ret = -ENODEV;
		goto err_unlock;
	}

	if (chan->client_count) {
		ret = -EBUSY;
		goto err_unlock;
	}

	if (kstrtoint(page, 0, &interval) == 0) {
		if (interval < PERF_MIN_INTERVAL ||
		    interval > PERF_MAX_INTERVAL) {
			ret = -EINVAL;
			goto err_unlock;
		}
		perf_cfg = readl(&chan_fw->perf_cfg);
		perf_cfg &= ~(PERF_INTERVAL_MASK << PERF_INTERVAL_SHIFT);
		perf_cfg |= interval << PERF_INTERVAL_SHIFT;
		writel(perf_cfg, &chan_fw->perf_cfg);
	}

err_unlock:
	rcu_read_unlock();

	return ret;
}

struct switchtec_sysfs_entry interval_attr = __ATTR_RW(interval);

static ssize_t mrrs_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 mrrs = 0;

	mrrs = readl(&chan_fw->perf_cfg);
	mrrs >>= PERF_MRRS_SHIFT;
	mrrs &= PERF_MRRS_MASK;

	return sprintf(page, "0x%x\n", mrrs);
}

static ssize_t mrrs_store(struct dma_chan *chan, const char *page, size_t count)
{
	int mrrs;
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 perf_cfg;
	ssize_t ret = count;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		ret = -ENODEV;
		goto err_unlock;
	}

	if (chan->client_count) {
		ret = -EBUSY;
		goto err_unlock;
	}

	if (kstrtoint(page, 0, &mrrs) == 0) {
		if (mrrs < PERF_MIN_MRRS || mrrs > PERF_MAX_MRRS) {
			ret = -EINVAL;
			goto err_unlock;
		}
		perf_cfg = readl(&chan_fw->perf_cfg);
		perf_cfg &= ~(PERF_MRRS_MASK << PERF_MRRS_SHIFT);
		perf_cfg |= mrrs << PERF_MRRS_SHIFT;
		writel(perf_cfg, &chan_fw->perf_cfg);
	}

err_unlock:
	rcu_read_unlock();

	return ret;
}

struct switchtec_sysfs_entry mrrs_attr = __ATTR_RW(mrrs);

static ssize_t se_count_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u64 count = 0;

	count = le32_to_cpu(readl(&chan_fw->perf_fetched_se_cnt_hi));
	count <<= 32;
	count |= le32_to_cpu(readl(&chan_fw->perf_fetched_se_cnt_lo));

	return sprintf(page, "0x%llx\n", count);
}

struct switchtec_sysfs_entry se_count_attr = __ATTR_RO(se_count);

static ssize_t byte_count_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u64 count = 0;

	count = le32_to_cpu(readl(&chan_fw->perf_byte_cnt_hi));
	count <<= 32;
	count |= le32_to_cpu(readl(&chan_fw->perf_byte_cnt_lo));

	return sprintf(page, "0x%llx\n", count);
}

struct switchtec_sysfs_entry byte_count_attr = __ATTR_RO(byte_count);

static ssize_t se_pending_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u16 count = 0;

	count = le16_to_cpu(readl(&chan_fw->perf_se_pending));

	return sprintf(page, "0x%x\n", count);
}

struct switchtec_sysfs_entry se_pending_attr = __ATTR_RO(se_pending);

static ssize_t se_buf_empty_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u16 count = 0;

	count = le16_to_cpu(readl(&chan_fw->perf_se_buf_empty));

	return sprintf(page, "0x%x\n", count);
}

struct switchtec_sysfs_entry se_buf_empty_attr = __ATTR_RO(se_buf_empty);

static ssize_t chan_idle_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 ratio = 0;

	ratio = le32_to_cpu(readl(&chan_fw->perf_chan_idle));

	return sprintf(page, "0x%x\n", ratio);
}

struct switchtec_sysfs_entry chan_idle_attr = __ATTR_RO(chan_idle);

static ssize_t latency_max_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 lat = 0;

	lat = le32_to_cpu(readl(&chan_fw->perf_lat_max));

	return sprintf(page, "0x%x\n", lat);
}

struct switchtec_sysfs_entry latency_max_attr = __ATTR_RO(latency_max);

static ssize_t latency_min_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 lat = 0;

	lat = le32_to_cpu(readl(&chan_fw->perf_lat_min));

	return sprintf(page, "0x%x\n", lat);
}

struct switchtec_sysfs_entry latency_min_attr = __ATTR_RO(latency_min);

static ssize_t latency_last_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 lat = 0;

	lat = le32_to_cpu(readl(&chan_fw->perf_lat_last));

	return sprintf(page, "0x%x\n", lat);
}

struct switchtec_sysfs_entry latency_last_attr = __ATTR_RO(latency_last);

static ssize_t latency_selector_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	u32 lat = 0;

	lat = le32_to_cpu(readl(&chan_fw->perf_latency_selector));

	strcat(page, "To select a latency type, write the type number (1 ~ 5) to this file.\n\n");

        strcat(page, "Latency Types (Selected latency type is shown with trailing (*))\n");
	strcat(page, "(1) SE Fetch latency");
	if (lat & SWITCHTEC_LAT_SE_FETCH)
		strcat(page, " (*)\n");
	else
		strcat(page, "\n");

	strcat(page, "(2) VDM latency");
	if (lat & SWITCHTEC_LAT_VDM)
		strcat(page, " (*)\n");
	else
		strcat(page, "\n");

	strcat(page, "(3) Read Immediate latency");
	if (lat & SWITCHTEC_LAT_RD_IMM)
		strcat(page, " (*)\n");
	else
		strcat(page, "\n");

	strcat(page, "(4) FW NP latency");
	if (lat & SWITCHTEC_LAT_FW_NP)
		strcat(page, " (*)\n");
	else
		strcat(page, "\n");

	strcat(page, "(5) SE Processing latency");
	if (lat & SWITCHTEC_LAT_SE_PROCESS)
		strcat(page, " (*)\n");
	else
		 strcat(page, "\n");

	strcat(page, "\n");

	return strlen(page);
}

static ssize_t latency_selector_store(struct dma_chan *chan, const char *page,
				      size_t count)
{
	int lat_type;
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs *chan_fw = swdma_chan->mmio_chan_fw;
	ssize_t ret = count;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		ret = -ENODEV;
		goto err_unlock;
	}

	if (chan->client_count) {
		ret = -EBUSY;
		goto err_unlock;
	}

	if (kstrtoint(page, 0, &lat_type) == 0) {
		if (lat_type < 1 ||
		    lat_type > 5) {
			ret = -EINVAL;
			goto err_unlock;
		}
		lat_type = 1 << (lat_type - 1);
		writel(lat_type, &chan_fw->perf_latency_selector);
	}

err_unlock:
	rcu_read_unlock();

	return ret;
}

struct switchtec_sysfs_entry latency_selector_attr =__ATTR_RW(latency_selector);

static ssize_t switchtec_config_attr_show(struct kobject *kobj,
					  struct attribute *attr, char *page)
{
	struct switchtec_sysfs_entry *entry;
	struct switchtec_dma_chan *swdma_chan;

	entry = container_of(attr, struct switchtec_sysfs_entry, attr);
	swdma_chan = container_of(kobj, struct switchtec_dma_chan, config_kobj);

	if (!entry->show)
		return -EIO;
	return entry->show(&swdma_chan->dma_chan, page);
}

static ssize_t switchtec_config_attr_store(struct kobject *kobj,
					   struct attribute *attr,
					   const char *page, size_t count)
{
	struct switchtec_sysfs_entry *entry;
	struct switchtec_dma_chan *swdma_chan;

	entry = container_of(attr, struct switchtec_sysfs_entry, attr);
	swdma_chan = container_of(kobj, struct switchtec_dma_chan, config_kobj);

	if (!entry->store)
		return -EIO;
	return entry->store(&swdma_chan->dma_chan, page, count);
}

const struct sysfs_ops switchtec_config_sysfs_ops = {
	.show	= switchtec_config_attr_show,
	.store  = switchtec_config_attr_store,
};

static struct attribute *switchtec_config_attrs[] = {
	&burst_scale_attr.attr,
	&burst_size_attr.attr,
	&interval_attr.attr,
	&arb_weight_attr.attr,
	&mrrs_attr.attr,
	NULL,
};

struct kobj_type switchtec_config_ktype = {
	.sysfs_ops = &switchtec_config_sysfs_ops,
	.default_attrs = switchtec_config_attrs,
};

static ssize_t switchtec_pmon_attr_show(struct kobject *kobj,
					struct attribute *attr, char *page)
{
	struct switchtec_sysfs_entry *entry;
	struct switchtec_dma_chan *swdma_chan;

	entry = container_of(attr, struct switchtec_sysfs_entry, attr);
	swdma_chan = container_of(kobj, struct switchtec_dma_chan, pmon_kobj);

	if (!entry->show)
		return -EIO;
	return entry->show(&swdma_chan->dma_chan, page);
}

static ssize_t switchtec_pmon_attr_store(struct kobject *kobj,
					 struct attribute *attr,
					 const char *page, size_t count)
{
	struct switchtec_sysfs_entry *entry;
	struct switchtec_dma_chan *swdma_chan;

	entry = container_of(attr, struct switchtec_sysfs_entry, attr);
	swdma_chan = container_of(kobj, struct switchtec_dma_chan, pmon_kobj);

	if (!entry->store)
		return -EIO;
	return entry->store(&swdma_chan->dma_chan, page, count);
}

const struct sysfs_ops switchtec_pmon_sysfs_ops = {
	.show	= switchtec_pmon_attr_show,
	.store  = switchtec_pmon_attr_store,
};


static struct attribute *switchtec_pmon_attrs[] = {
	&se_count_attr.attr,
	&byte_count_attr.attr,
	&se_pending_attr.attr,
	&se_buf_empty_attr.attr,
	&chan_idle_attr.attr,
	&latency_max_attr.attr,
	&latency_min_attr.attr,
	&latency_last_attr.attr,
	&latency_selector_attr.attr,
	NULL,
};

struct kobj_type switchtec_pmon_ktype = {
	.sysfs_ops = &switchtec_pmon_sysfs_ops,
	.default_attrs = switchtec_pmon_attrs,
};

void switchtec_chan_kobject_add(struct switchtec_dma_chan *swdma_chan)
{
	struct kobject *parent;
	int err;

	parent = &swdma_chan->dma_chan.dev->device.kobj;
	err = kobject_init_and_add(&swdma_chan->config_kobj,
				   &switchtec_config_ktype, parent, "config");
	if (err) {
		dev_warn(to_chan_dev(swdma_chan),
			 "sysfs config init error (%d), continuing...\n", err);
		kobject_put(&swdma_chan->config_kobj);
	}

	err = kobject_init_and_add(&swdma_chan->pmon_kobj,
				   &switchtec_pmon_ktype, parent, "pmon");
	if (err) {
		dev_warn(to_chan_dev(swdma_chan),
			 "sysfs pmon init error (%d), continuing...\n", err);
		kobject_put(&swdma_chan->pmon_kobj);
	}
}

void switchtec_kobject_add(struct switchtec_dma_dev *swdma_dev)
{
	struct dma_device *dma = &swdma_dev->dma_dev;
	struct switchtec_dma_chan *swdma_chan;
	struct dma_chan *c;

	list_for_each_entry(c, &dma->channels, device_node) {
		swdma_chan = to_switchtec_dma_chan(c);
		switchtec_chan_kobject_add(swdma_chan);
	}
}

void switchtec_chan_kobject_del(struct switchtec_dma_chan *swdma_chan)
{
	if (swdma_chan->config_kobj.state_initialized) {
		kobject_del(&swdma_chan->config_kobj);
		kobject_put(&swdma_chan->config_kobj);
	}

	if (swdma_chan->pmon_kobj.state_initialized) {
		kobject_del(&swdma_chan->pmon_kobj);
		kobject_put(&swdma_chan->pmon_kobj);
	}
}

static int switchtec_dma_create(struct pci_dev *pdev)
{
	struct switchtec_dma_dev *swdma_dev;
	struct dma_device *dma;
	struct dma_chan *chan;
	struct device *dev = &pdev->dev;
	void __iomem *bar;
	int chan_cnt;
	int nr_vecs;
	int irq;
	int rc;

	/*
	 * Create the switchtec dma device
	 */
	swdma_dev = devm_kzalloc(dev, sizeof(*swdma_dev), GFP_KERNEL);
	if (!swdma_dev)
		return -ENOMEM;

	bar = pcim_iomap_table(pdev)[0];
	global_bar = bar;

	swdma_dev->mmio_dmac_ver = bar + SWITCHTEC_DMAC_VERSION_OFFSET;
	swdma_dev->mmio_dmac_cap = bar + SWITCHTEC_DMAC_CAPABILITY_OFFSET;
	swdma_dev->mmio_dmac_status = bar + SWITCHTEC_DMAC_STATUS_OFFSET;
	swdma_dev->mmio_dmac_ctrl = bar + SWITCHTEC_DMAC_CONTROL_OFFSET;
	swdma_dev->mmio_chan_hw_all = bar + SWITCHTEC_DMAC_CHAN_CTRL_OFFSET;
	swdma_dev->mmio_chan_fw_all = bar + SWITCHTEC_DMAC_CHAN_CFG_STS_OFFSET;

	RCU_INIT_POINTER(swdma_dev->pdev, pdev);

	nr_vecs = pci_msix_vec_count(pdev);
	rc = pci_alloc_irq_vectors(pdev, nr_vecs, nr_vecs, PCI_IRQ_MSIX);
	if (rc < 0) {
		goto err_exit;
	}

	irq = readw(&swdma_dev->mmio_dmac_cap->int_err_vec);
	dev_dbg(dev, "Internal error irq vec 0x%x\n", irq);

	irq = pci_irq_vector(swdma_dev->pdev, irq);
	if (irq < 0) {
		rc = irq;
		goto err_exit;
	}

	rc = devm_request_irq(dev, irq, switchtec_dma_int_error_isr, 0,
			      KBUILD_MODNAME, swdma_dev);
	if (rc)
		goto err_exit;

	swdma_dev->int_error_irq = irq;

	irq = readw(&swdma_dev->mmio_dmac_cap->chan_sts_vec);
	dev_dbg(dev, "Channel status irq vec 0x%x\n", irq);

	irq = pci_irq_vector(swdma_dev->pdev, irq);
	if (irq < 0) {
		rc = irq;
		goto err_exit;
	}

	rc = devm_request_irq(dev, irq, switchtec_dma_chan_status_isr, 0,
			      KBUILD_MODNAME, swdma_dev);
	if (rc)
		goto err_exit;

	swdma_dev->chan_status_irq = irq;

	chan_cnt = switchtec_dma_get_chan_cnt(swdma_dev);
	if (!chan_cnt) {
		pci_err(swdma_dev->pdev, "No channel configured.\n");
		rc = -ENXIO;
		goto err_exit;
	}

	if (nr_vecs != (chan_cnt + 2))
		pci_warn(pdev, "vector number (0x%x) != channel count (0x%x)\n",
			 nr_vecs, chan_cnt);

	chan_cnt = switchtec_dma_chans_enumerate(swdma_dev, chan_cnt);
	if (chan_cnt < 0) {
		pci_err(pdev, "Failed to enumerate dma channels: %d\n",
			chan_cnt);
		rc = -ENXIO;
		goto err_exit;
	}

	swdma_dev->chan_cnt = chan_cnt;

	dma = &swdma_dev->dma_dev;
	pci_info(pdev, "chan count: %d\n", dma->chancnt);
	dma->copy_align = DMAENGINE_ALIGN_1_BYTE;
	dma_cap_set(DMA_MEMCPY, dma->cap_mask);
	dma_cap_set(DMA_PRIVATE, dma->cap_mask);
	dma->dev = get_device(&pdev->dev);

	kref_init(&swdma_dev->ref);
	INIT_WORK(&swdma_dev->release_work, switchtec_dma_release_work);

	tasklet_init(&swdma_dev->int_error_task,
		     switchtec_dma_int_error_task,
		     (unsigned long)swdma_dev);

	tasklet_init(&swdma_dev->chan_status_task,
		     switchtec_dma_chan_status_task,
		     (unsigned long)swdma_dev);

	dma->device_alloc_chan_resources = switchtec_dma_alloc_chan_resources;
	dma->device_free_chan_resources = switchtec_dma_free_chan_resources;
	dma->device_prep_dma_memcpy = switchtec_dma_prep_memcpy;
	dma->device_prep_dma_imm_data = switchtec_dma_prep_wimm_data;
	dma->device_issue_pending = switchtec_dma_issue_pending;
	dma->device_tx_status = switchtec_dma_tx_status;

	rc = dma_async_device_register(dma);
	if (rc) {
		pci_err(pdev, "Failed to register dma device: %d\n", rc);
		switchtec_dma_chans_release(swdma_dev);
		goto err_exit;
	}

	list_for_each_entry(chan, &dma->channels, device_node)
		pci_dbg(pdev, "chan name: %s\n", dma_chan_name(chan));

	pci_set_drvdata(pdev, swdma_dev);

	switchtec_kobject_add(swdma_dev);

	return 0;

err_exit:
	if (swdma_dev->int_error_irq)
		devm_free_irq(dev, swdma_dev->int_error_irq, swdma_dev);

	if (swdma_dev->chan_status_irq)
		devm_free_irq(dev, swdma_dev->chan_status_irq, swdma_dev);

	return rc;
}

static int switchtec_dma_probe(struct pci_dev *pdev,
			       const struct pci_device_id *id)
{
	int rc;

	rc = pcim_enable_device(pdev);
	if (rc)
		return rc;

	rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(64));
	if (rc)
		rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (rc)
		return rc;

	rc = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(64));
	if (rc)
		rc = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
	if (rc)
		return rc;

	rc = pcim_iomap_regions(pdev, 1, KBUILD_MODNAME);
	if (rc)
		return rc;

	pci_set_master(pdev);

	rc = switchtec_dma_create(pdev);
	if (rc)
		goto err_free_irq_vectors;

	pci_info(pdev, "Switchtec DMA Channels Registered\n");

	return 0;

err_free_irq_vectors:
	pci_free_irq_vectors(pdev);
	return rc;
}

static void switchtec_dma_remove(struct pci_dev *pdev)
{
	struct switchtec_dma_dev *swdma_dev = pci_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	switchtec_dma_chans_release(swdma_dev);

	tasklet_kill(&swdma_dev->int_error_task);
	tasklet_kill(&swdma_dev->chan_status_task);

	rcu_assign_pointer(swdma_dev->pdev, NULL);
	synchronize_rcu();

	devm_free_irq(dev, swdma_dev->int_error_irq, swdma_dev);
	devm_free_irq(dev, swdma_dev->chan_status_irq, swdma_dev);

	pci_free_irq_vectors(pdev);

#if 0
	switchtec_dma_put(swdma_dev);
#else
	dma_async_device_unregister(&swdma_dev->dma_dev);
	put_device(swdma_dev->dma_dev.dev);
#endif

	pci_info(pdev, "Switchtec DMA Channels Unregistered\n");
}

#define MICROSEMI_VENDOR_ID 0x11f8

#define SWITCHTEC_PCI_DEVICE(device_id) \
	{ \
		.vendor     = MICROSEMI_VENDOR_ID, \
		.device     = device_id, \
		.subvendor  = PCI_ANY_ID, \
		.subdevice  = PCI_ANY_ID, \
		.class      = PCI_CLASS_SYSTEM_OTHER << 8, \
		.class_mask = 0xFFFFFFFF, \
	}

static const struct pci_device_id switchtec_dma_pci_tbl[] = {
	SWITCHTEC_PCI_DEVICE(0x4000),  //PFX 100XG4
	SWITCHTEC_PCI_DEVICE(0x4084),  //PFX 84XG4
	SWITCHTEC_PCI_DEVICE(0x4068),  //PFX 68XG4
	SWITCHTEC_PCI_DEVICE(0x4052),  //PFX 52XG4
	SWITCHTEC_PCI_DEVICE(0x4036),  //PFX 36XG4
	SWITCHTEC_PCI_DEVICE(0x4028),  //PFX 28XG4
	SWITCHTEC_PCI_DEVICE(0x4100),  //PSX 100XG4
	SWITCHTEC_PCI_DEVICE(0x4184),  //PSX 84XG4
	SWITCHTEC_PCI_DEVICE(0x4168),  //PSX 68XG4
	SWITCHTEC_PCI_DEVICE(0x4152),  //PSX 52XG4
	SWITCHTEC_PCI_DEVICE(0x4136),  //PSX 36XG4
	SWITCHTEC_PCI_DEVICE(0x4128),  //PSX 28XG4
	SWITCHTEC_PCI_DEVICE(0x4200),  //PAX 100XG4
	SWITCHTEC_PCI_DEVICE(0x4284),  //PAX 84XG4
	SWITCHTEC_PCI_DEVICE(0x4268),  //PAX 68XG4
	SWITCHTEC_PCI_DEVICE(0x4252),  //PAX 52XG4
	SWITCHTEC_PCI_DEVICE(0x4236),  //PAX 36XG4
	SWITCHTEC_PCI_DEVICE(0x4228),  //PAX 28XG4
	{0}
};
MODULE_DEVICE_TABLE(pci, switchtec_dma_pci_tbl);

static struct pci_driver switchtec_dma_pci_driver = {
	.name           = KBUILD_MODNAME,
	.id_table       = switchtec_dma_pci_tbl,
	.probe          = switchtec_dma_probe,
	.remove		= switchtec_dma_remove,
};
module_pci_driver(switchtec_dma_pci_driver);
