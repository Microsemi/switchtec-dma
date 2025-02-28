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

#include "linux/switchtec_fabric_dma.h"
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
	SWITCHTEC_DMAC_FABRIC_CMD_OFFSET = 0x164000,
	SWITCHTEC_DMAC_FABRIC_CTRL_OFFSET = 0x165000,
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
	u32 rsvd1;
	u32 cplt_tmo;
	u32 tag_limit;
	u16 chan_sts_vec;
	u16 rsvd2;
	u16 se_buf_cnt;
	u16 se_buf_base;
} __packed;

struct dmac_status_regs {
	u32 state;
	u32 internal_err;
	u32 chan_halt_sum_lo;
	u32 chan_halt_sum_hi;
	u32 rsvd[2];
	u32 chan_pause_sum_lo;
	u32 chan_pause_sum_hi;
} __packed;

#define IER_OB_PF_RD_ERR_I  BIT(14)
#define IER_OB_TLP_RD_ERR_I BIT(15)
#define IER_ECC_ER_0_I      BIT(20)
#define IER_ECC_ER_1_I      BIT(21)
#define IER_PARITY_ERR_I    BIT(22)
#define IER_IB_IER_I        BIT(23)

struct dmac_control_regs {
	u32 reset_halt;
} __packed;

struct dmac_fabric_cmd_regs {
	u32 input[256];
	u32 rsvd1[256];
	u16 command;
	u16 rsvd2;
} __packed;

struct dmac_fabric_control_regs {
	u16 cmd_vec;
	u16 rsvd1;
	u32 cmd_dma_addr_lo;
	u32 cmd_dma_addr_hi;
	u16 event_vec;
	u16 rsvd2;
	u32 event_dma_addr_lo;
	u32 event_dma_addr_hi;
	u32 event_dma_size;
	u32 event_queue_tail;
	u32 cmd_event_enable;
	u16 local_hfid;
	u16 rsvd3;
	u32 requestor_id;
} __packed;

#define SWITCHTEC_CHAN_CTRL_PAUSE     BIT(0)
#define SWITCHTEC_CHAN_CTRL_HALT      BIT(1)
#define SWITCHTEC_CHAN_CTRL_RESET     BIT(2)
#define SWITCHTEC_CHAN_CTRL_ERR_PAUSE BIT(3)

#define SWITCHTEC_CHAN_STS_PAUSED      BIT(9)
#define SWITCHTEC_CHAN_STS_HALTED      BIT(10)
#define SWITCHTEC_CHAN_STS_PAUSED_MASK GENMASK(29, 13)

static const char *channel_status_str[] = {
	[13] = "received a VDM with length error status",
	[14] = "received a VDM or Cpl with Unsupported Request error status",
	[15] = "received a VDM or Cpl with Completion Abort error status",
	[16] = "received a VDM with ECRC error status",
	[17] = "received a VDM with EP error status",
	[18] = "received a VDM with Reserved Cpl error status",
	[19] = "received only part of split SE CplD",
	[20] = "the ISP_DMAC detected a Completion Time Out",
	[21] = "received a Cpl with Unsupported Request status",
	[22] = "received a Cpl with Completion Abort status",
	[23] = "received a Cpl with a reserved status",
	[24] = "received a TLP with ECRC error status in its metadata",
	[25] = "received a TLP with the EP bit set in the header",
	[26] = "the ISP_DMAC tried to process a SE with an invalid Connection ID",
	[27] = "the ISP_DMAC tried to process a SE with an invalid Remote Host interrupt",
	[28] = "a reserved opcode was detected in an SE",
	[29] = "received a SE Cpl with error status",
};

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
	u16 cq_size;
	u16 rsvd1;
	u32 sq_base_lo;
	u32 sq_base_hi;
	u16 sq_size;
	u16 rsvd2;
	u32 int_vec;
	u32 perf_cfg;
	u32 rsvd3;
	u32 perf_latency_selector;
	u32 perf_fetched_se_cnt_lo;
	u32 perf_fetched_se_cnt_hi;
	u32 perf_byte_cnt_lo;
	u32 perf_byte_cnt_hi;
	u32 rsvd4;
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

enum cmd {
	CMD_GET_HOST_LIST = 1,
	CMD_REGISTER_BUF = 2,
	CMD_UNREGISTER_BUF = 3,
	CMD_GET_BUF_LIST = 4,
	CMD_GET_OWN_BUF_LIST = 5,
};

enum cmd_status {
	CMD_STATUS_IDLE = 0,
	CMD_STATUS_INPROGRESS = 1,
	CMD_STATUS_DONE = 2,
	CMD_STATUS_ERROR = 0xFF,
};

#define CMD_TIMEOUT_MSECS 200

#define SWITCHTEC_CHAN_INTERVAL 1
#define SWITCHTEC_CHAN_BURST_SZ 1
#define SWITCHTEC_CHAN_BURST_SCALE 1
#define SWITCHTEC_CHAN_MRRS 1

static LIST_HEAD(chan_list);
static LIST_HEAD(dma_list);

struct switchtec_dma_chan {
	struct switchtec_dma_dev *swdma_dev;
	struct dma_chan dma_chan;
	struct chan_hw_regs __iomem *mmio_chan_hw;
	struct chan_fw_regs __iomem *mmio_chan_fw;
	spinlock_t hw_ctrl_lock;

	struct tasklet_struct desc_task;
	spinlock_t submit_lock;
	bool ring_active;
	int cid;
	spinlock_t complete_lock;
	bool comp_ring_active;

	/* channel index and irq */
	int index;
	int irq;

	/*
	 * In driver context, head is advanced by producer while
	 * tail is advanced by consumer.
	 */

	/* the head and tail for both desc_ring and hw_sq */
	int head;
	int tail;
	int phase_tag;
	struct switchtec_dma_desc **desc_ring;
	struct switchtec_dma_hw_se_desc *hw_sq;
	dma_addr_t dma_addr_sq;

	/* the tail for hw_cq */
	int cq_tail;
	struct switchtec_dma_hw_ce *hw_cq;
	dma_addr_t dma_addr_cq;

	struct kobject config_kobj;
	struct kobject pmon_kobj;

	bool is_fabric;

	struct list_head list;
};

#define CMD_OUTPUT_SIZE 1024
struct cmd_output{
	u32 status;
	u32 cmd_id;
	u32 rtn_val;
	u32 output_size;
	u8 data[CMD_OUTPUT_SIZE];
};

#define SWITCHTEC_DMA_EQ_SIZE SZ_1K
struct fabric_event_queue {
	u32 head;
	u32 rsvd[3];
	struct switchtec_fabric_event entries[];
};

struct switchtec_dma_dev {
	struct dma_device dma_dev;
	struct pci_dev __rcu *pdev;
	struct switchtec_dma_chan **swdma_chans;
	int chan_cnt;

	int chan_status_irq;

	void __iomem *bar;
	struct dmac_version_regs __iomem *mmio_dmac_ver;
	struct dmac_capability_regs __iomem *mmio_dmac_cap;
	struct dmac_status_regs __iomem *mmio_dmac_status;
	struct dmac_control_regs __iomem *mmio_dmac_ctrl;
	struct dmac_fabric_cmd_regs __iomem *mmio_fabric_cmd;
	struct dmac_fabric_control_regs __iomem *mmio_fabric_ctrl;
	void __iomem *mmio_chan_hw_all;
	void __iomem *mmio_chan_fw_all;

	struct tasklet_struct chan_status_task;

	bool is_fabric;
	u16 hfid;

	/*
	 * Only one cmd can be executed at a time.
	 */
	struct mutex cmd_mutex;
	struct cmd_output *cmd;
	dma_addr_t cmd_dma_addr;
	int cmd_irq;
	struct atomic_notifier_head rhi_notifier_list;

	struct fabric_event_queue *eq;
	dma_addr_t eq_dma_addr;
	int eq_tail;
	int event_irq;
	struct tasklet_struct fabric_event_task;
	struct atomic_notifier_head event_notifier_list;

	struct work_struct release_work;

	struct list_head list;
};

static struct switchtec_dma_dev *to_switchtec_dma(struct dma_device *d)
{
	return container_of(d, struct switchtec_dma_dev, dma_dev);
}

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
	union {
		__le32 saddr_lo;
		__le32 widata_lo;
	};
	union {
		__le32 saddr_hi;
		__le32 widata_hi;
	};
	__le32 daddr_lo;
	__le32 daddr_hi;
	__le16 dfid;
	__le16 sfid;
};

#define SWITCHTEC_SE_DFM                BIT(5)
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
	u32 orig_size;
	bool completed;
};

#define HALT_RETRY 100
static int halt_channel(struct switchtec_dma_chan *swdma_chan)
{
	u32 status;
	struct chan_hw_regs __iomem *chan_hw = swdma_chan->mmio_chan_hw;
	int retry = HALT_RETRY;
	struct pci_dev *pdev;
	int ret;

	rcu_read_lock();
	pdev = rcu_dereference(swdma_chan->swdma_dev->pdev);
	if (!pdev) {
		ret = -ENODEV;
		goto unlock_and_exit;
	}

	spin_lock(&swdma_chan->hw_ctrl_lock);
	writeb(SWITCHTEC_CHAN_CTRL_HALT, &chan_hw->ctrl);

	ret = -EIO;
	do {
		status = readl(&chan_hw->status);

		if (status & SWITCHTEC_CHAN_STS_HALTED) {
			ret = 0;
			break;
		} else {
			udelay(1000);
		}
	} while (retry--);
	spin_unlock(&swdma_chan->hw_ctrl_lock);

unlock_and_exit:
	rcu_read_unlock();
	return ret;
}

static int unhalt_channel(struct switchtec_dma_chan *swdma_chan)
{
	u8 ctrl;
	u32 status;
	struct chan_hw_regs __iomem *chan_hw = swdma_chan->mmio_chan_hw;
	int retry = HALT_RETRY;
	struct pci_dev *pdev;
	int ret;

	rcu_read_lock();
	pdev = rcu_dereference(swdma_chan->swdma_dev->pdev);
	if (!pdev) {
		ret = -ENODEV;
		goto unlock_and_exit;
	}

	spin_lock(&swdma_chan->hw_ctrl_lock);
	ctrl = readb(&chan_hw->ctrl);
	ctrl &= ~SWITCHTEC_CHAN_CTRL_HALT;

	writeb(ctrl, &chan_hw->ctrl);

	ret = -EIO;
	do {
		status = readl(&chan_hw->status);
		if (!(status & SWITCHTEC_CHAN_STS_HALTED)) {
			ret = 0;
			break;
		} else {
			udelay(1000);
		}
	} while (retry--);
	spin_unlock(&swdma_chan->hw_ctrl_lock);

unlock_and_exit:
	rcu_read_unlock();
	return ret;
}

static int reset_channel(struct switchtec_dma_chan *swdma_chan)
{
	u8 ctrl;
	struct chan_hw_regs __iomem *chan_hw = swdma_chan->mmio_chan_hw;
	struct pci_dev *pdev;

	rcu_read_lock();
	pdev = rcu_dereference(swdma_chan->swdma_dev->pdev);
	if (!pdev) {
		rcu_read_unlock();
		return -ENODEV;
	}

	/*
	 * This function is only called during initialization, no need to
	 * protect the access to chan_hw->ctrl with hw_ctrl_lock.
	 */
	ctrl = SWITCHTEC_CHAN_CTRL_RESET;
	ctrl |= SWITCHTEC_CHAN_CTRL_ERR_PAUSE;
	writel(ctrl, &chan_hw->ctrl);

	udelay(1000);

	ctrl = SWITCHTEC_CHAN_CTRL_ERR_PAUSE;
	writel(ctrl, &chan_hw->ctrl);

	rcu_read_unlock();
	return 0;
}

static int pause_reset_channel(struct switchtec_dma_chan *swdma_chan)
{
	struct chan_hw_regs __iomem *chan_hw = swdma_chan->mmio_chan_hw;
	struct pci_dev *pdev;

	rcu_read_lock();
	pdev = rcu_dereference(swdma_chan->swdma_dev->pdev);
	if (!pdev) {
		rcu_read_unlock();
		return -ENODEV;
	}

	/*
	 * This function is only called during initialization, no need to
	 * protect the access to chan_hw->ctrl with hw_ctrl_lock.
	 */

	/* pause channel */
	writeb(SWITCHTEC_CHAN_CTRL_PAUSE, &chan_hw->ctrl);
	rcu_read_unlock();

	/* wait 60ms to ensure no pending CEs */
	msleep(60);

	/* reset channel */
	return reset_channel(swdma_chan);

}

#define PAUSE_RESUME_RETRY 100
static int switchtec_dma_pause(struct dma_chan *chan)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	u32 status;
	struct chan_hw_regs __iomem *chan_hw = swdma_chan->mmio_chan_hw;
	int retry = PAUSE_RESUME_RETRY;
	struct pci_dev *pdev;
	int ret;

	rcu_read_lock();
	pdev = rcu_dereference(swdma_chan->swdma_dev->pdev);
	if (!pdev) {
		ret = -ENODEV;
		goto unlock_and_exit;
	}

	spin_lock(&swdma_chan->hw_ctrl_lock);
	writeb(SWITCHTEC_CHAN_CTRL_PAUSE, &chan_hw->ctrl);

	ret = -EIO;
	do {
		status = readl(&chan_hw->status);

		if (status & SWITCHTEC_CHAN_STS_PAUSED) {
			ret = 0;
			break;
		} else {
			udelay(1000);
		}
	} while (retry--);
	spin_unlock(&swdma_chan->hw_ctrl_lock);

unlock_and_exit:
	rcu_read_unlock();
	return ret;
}

static int switchtec_dma_resume(struct dma_chan *chan)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	u32 status;
	struct chan_hw_regs __iomem *chan_hw = swdma_chan->mmio_chan_hw;
	int retry = PAUSE_RESUME_RETRY;
	struct pci_dev *pdev;
	int ret;

	rcu_read_lock();
	pdev = rcu_dereference(swdma_chan->swdma_dev->pdev);
	if (!pdev) {
		ret = -ENODEV;
		goto unlock_and_exit;
	}

	spin_lock(&swdma_chan->hw_ctrl_lock);
	writeb(0, &chan_hw->ctrl);

	ret = -EIO;
	do {
		status = readl(&chan_hw->status);

		if (!(status & SWITCHTEC_CHAN_STS_PAUSED)) {
			ret = 0;
			break;
		} else {
			udelay(1000);
		}
	} while (retry--);
	spin_unlock(&swdma_chan->hw_ctrl_lock);

unlock_and_exit:
	rcu_read_unlock();
	return ret;
}

static int enable_channel(struct switchtec_dma_chan *swdma_chan)
{
	u32 valid_en_se;
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
	struct pci_dev *pdev;

	rcu_read_lock();
	pdev = rcu_dereference(swdma_chan->swdma_dev->pdev);
	if (!pdev) {
		rcu_read_unlock();
		return -ENODEV;
	}

	valid_en_se = readl(&chan_fw->valid_en_se);
	valid_en_se |= SWITCHTEC_CHAN_ENABLE;

	writel(valid_en_se, &chan_fw->valid_en_se);

	rcu_read_unlock();
	return 0;
}

static int disable_channel(struct switchtec_dma_chan *swdma_chan)
{
	u32 valid_en_se;
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
	struct pci_dev *pdev;

	rcu_read_lock();
	pdev = rcu_dereference(swdma_chan->swdma_dev->pdev);
	if (!pdev) {
		rcu_read_unlock();
		return -ENODEV;
	}

	valid_en_se = readl(&chan_fw->valid_en_se);
	valid_en_se &= ~SWITCHTEC_CHAN_ENABLE;

	writel(valid_en_se, &chan_fw->valid_en_se);

	rcu_read_unlock();
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
	struct switchtec_dma_desc *desc;
	struct switchtec_dma_hw_ce *ce;
	__le16 phase_tag;
	int tail;
	int cid;
	int se_idx;
	u32 sts_code;
	int i = 0;
	int *p;

	do {
		spin_lock_bh(&swdma_chan->complete_lock);
		if (!swdma_chan->comp_ring_active) {
			spin_unlock_bh(&swdma_chan->complete_lock);
			break;
		}

		ce = switchtec_dma_get_ce(swdma_chan, swdma_chan->cq_tail);
		phase_tag = smp_load_acquire(&ce->phase_tag);
		if (le16_to_cpu(phase_tag) == swdma_chan->phase_tag) {
			spin_unlock_bh(&swdma_chan->complete_lock);
			break;
		}

		cid = le16_to_cpu(ce->cid);
		se_idx = cid & (SWITCHTEC_DMA_SQ_SIZE - 1);
		desc = switchtec_dma_get_desc(swdma_chan, se_idx);

		tail = swdma_chan->tail;

		res.residue = desc->orig_size - le32_to_cpu(ce->cpl_byte_cnt);

		sts_code = le32_to_cpu(ce->sts_code);

		if (!(sts_code & SWITCHTEC_CE_SC_MASK)) {
			res.result = DMA_TRANS_NOERROR;
		} else {
			if (sts_code & SWITCHTEC_CE_SC_D_RD_CTO)
				res.result = DMA_TRANS_READ_FAILED;
			else
				res.result = DMA_TRANS_WRITE_FAILED;

			dev_err(chan_dev, "CID 0x%04x failed, SC 0x%08x\n", cid,
				(u32)(sts_code & SWITCHTEC_CE_SC_MASK));

			p = (int *)ce;
			for (i = 0; i < sizeof(*ce)/4; i++) {
				dev_err(chan_dev, "CE DW%d: 0x%08x\n", i,
					le32_to_cpu((__force __le32)*p));
				p++;
			}
		}

		desc->completed = true;

		swdma_chan->cq_tail++;
		swdma_chan->cq_tail &= SWITCHTEC_DMA_CQ_SIZE - 1;

		rcu_read_lock();
		if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
			rcu_read_unlock();
			spin_unlock_bh(&swdma_chan->complete_lock);
			return;
		}
		writew(swdma_chan->cq_tail, &swdma_chan->mmio_chan_hw->cq_head);
		rcu_read_unlock();

		if (swdma_chan->cq_tail == 0)
			swdma_chan->phase_tag = !swdma_chan->phase_tag;

		/*  Out of order CE */
		if (se_idx != tail) {
			spin_unlock_bh(&swdma_chan->complete_lock);
			continue;
		}

		do {
			dma_cookie_complete(&desc->txd);
			dma_descriptor_unmap(&desc->txd);
			dmaengine_desc_get_callback_invoke(&desc->txd, &res);
			desc->txd.callback = NULL;
			desc->txd.callback_result = NULL;
			desc->completed = false;

			tail++;
			tail &= SWITCHTEC_DMA_SQ_SIZE - 1;
			smp_store_release(&swdma_chan->tail, tail);
			desc = switchtec_dma_get_desc(swdma_chan,
						      swdma_chan->tail);
			if (!desc->completed)
				break;
		} while (CIRC_CNT(READ_ONCE(swdma_chan->head), swdma_chan->tail,
				  SWITCHTEC_DMA_SQ_SIZE));

		spin_unlock_bh(&swdma_chan->complete_lock);
	} while (1);
}

static void switchtec_dma_abort_desc(struct switchtec_dma_chan *swdma_chan,
		int force)
{
	struct dmaengine_result res;
	struct switchtec_dma_desc *desc;

	if (!force)
		switchtec_dma_process_desc(swdma_chan);

	spin_lock_bh(&swdma_chan->complete_lock);

	while (CIRC_CNT(swdma_chan->head, swdma_chan->tail,
			SWITCHTEC_DMA_SQ_SIZE) >= 1) {
		desc = switchtec_dma_get_desc(swdma_chan, swdma_chan->tail);

		res.residue = desc->orig_size;
		res.result = DMA_TRANS_ABORTED;

		dma_cookie_complete(&desc->txd);
		dma_descriptor_unmap(&desc->txd);
		if (!force)
			dmaengine_desc_get_callback_invoke(&desc->txd, &res);
		desc->txd.callback = NULL;
		desc->txd.callback_result = NULL;

		swdma_chan->tail++;
		swdma_chan->tail &= SWITCHTEC_DMA_SQ_SIZE - 1;
	}

	spin_unlock_bh(&swdma_chan->complete_lock);
}

static void switchtec_dma_chan_stop(struct switchtec_dma_chan *swdma_chan)
{
	int rc;

	rc = halt_channel(swdma_chan);
	if (rc) {
		dev_err(to_chan_dev(swdma_chan), "stop channel failed\n");
		return;
	}

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		rcu_read_unlock();
		return;
	}

	writel(0, &swdma_chan->mmio_chan_fw->sq_base_lo);
	writel(0, &swdma_chan->mmio_chan_fw->sq_base_hi);
	writel(0, &swdma_chan->mmio_chan_fw->cq_base_lo);
	writel(0, &swdma_chan->mmio_chan_fw->cq_base_hi);

	rcu_read_unlock();
}

static int switchtec_dma_terminate_all(struct dma_chan *chan)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	int rc = 0;

	spin_lock_bh(&swdma_chan->complete_lock);
	swdma_chan->comp_ring_active = false;
	spin_unlock_bh(&swdma_chan->complete_lock);

	rc = pause_reset_channel(swdma_chan);
	if (rc)
		dev_err(to_chan_dev(swdma_chan),
			"%s: pause reset channel failed\n",
			dma_chan_name(chan));

	return rc;
}

static void switchtec_dma_synchronize(struct dma_chan *chan)
{
	struct pci_dev *pdev;
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	int rc;

	rcu_read_lock();
	pdev = rcu_dereference(swdma_chan->swdma_dev->pdev);
	if (pdev)
		synchronize_irq(swdma_chan->irq);
	rcu_read_unlock();

	switchtec_dma_abort_desc(swdma_chan, 1);

	rc = enable_channel(swdma_chan);
	if (rc)
		return;

	rc = reset_channel(swdma_chan);
	if (rc)
		return;

	rc = unhalt_channel(swdma_chan);
	if (rc)
		return;

	spin_lock_bh(&swdma_chan->submit_lock);
	swdma_chan->head = 0;
	spin_unlock_bh(&swdma_chan->submit_lock);

	spin_lock_bh(&swdma_chan->complete_lock);
	swdma_chan->comp_ring_active = true;
	swdma_chan->phase_tag = 0;
	swdma_chan->tail = 0;
	swdma_chan->cq_tail = 0;
	swdma_chan->cid = 0;
	dma_cookie_init(chan);
	spin_unlock_bh(&swdma_chan->complete_lock);
}

static void switchtec_dma_desc_task(unsigned long data)
{
	struct switchtec_dma_chan *swdma_chan = (void *)data;

	switchtec_dma_process_desc(swdma_chan);
}

static void switchtec_dma_chan_status_task(unsigned long data)
{
	struct switchtec_dma_dev *swdma_dev = (void *)data;
	struct dma_device *dma_dev = &swdma_dev->dma_dev;
	struct dma_chan *chan;
	struct switchtec_dma_chan *swdma_chan;
	struct chan_hw_regs __iomem *chan_hw;
	struct device *chan_dev;
	u32 chan_status;
	int bit;

	list_for_each_entry(chan, &dma_dev->channels, device_node) {
		swdma_chan = to_switchtec_dma_chan(chan);
		chan_dev = to_chan_dev(swdma_chan);
		chan_hw = swdma_chan->mmio_chan_hw;

		rcu_read_lock();
		if (!rcu_dereference(swdma_dev->pdev)) {
			rcu_read_unlock();
			return;
		}

		chan_status = readl(&chan_hw->status);
		chan_status &= SWITCHTEC_CHAN_STS_PAUSED_MASK;
		rcu_read_unlock();

		bit = ffs(chan_status);
		if (!bit)
			dev_dbg(chan_dev, "No pause bit set.");
		else
			dev_err(chan_dev, "Paused, %s\n",
				channel_status_str[bit - 1]);
	}
}

#define SWITCHTEC_INVALID_HFID 0xffff

enum desc_type{
	MEMCPY,
	WIMM,
	UNKNOWN_TRANSACTION,
};

static struct dma_async_tx_descriptor *switchtec_dma_prep_desc(
		struct dma_chan *c, enum desc_type type, u16 dst_fid,
		dma_addr_t dma_dst, u16 src_fid, dma_addr_t dma_src, u64 data,
		size_t len, unsigned long flags)
	__acquires(swdma_chan->submit_lock)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(c);
	struct device *chan_dev = to_chan_dev(swdma_chan);
	struct switchtec_dma_desc *desc;
	int head;
	int tail;

	spin_lock_bh(&swdma_chan->submit_lock);

	if (type >= UNKNOWN_TRANSACTION)
		goto err_unlock;

	if (type == MEMCPY)
		if (len > SWITCHTEC_DESC_MAX_SIZE)
			goto err_unlock;

	if ((type == WIMM) && (len == 8))
		if (dma_dst & ((1 << DMAENGINE_ALIGN_8_BYTES) - 1)) {
			dev_err(chan_dev,
				"QW WIMM dst addr 0x%08x_%08x not QW aligned!\n",
				upper_32_bits(dma_dst), lower_32_bits(dma_dst));
			goto err_unlock;
		}

	if (!swdma_chan->ring_active)
		goto err_unlock;

	tail = READ_ONCE(swdma_chan->tail);
	head = swdma_chan->head;

	if (!CIRC_SPACE(head, tail, SWITCHTEC_DMA_RING_SIZE))
		goto err_unlock;

	desc = switchtec_dma_get_desc(swdma_chan, head);

	if (src_fid != SWITCHTEC_INVALID_HFID &&
	    dst_fid != SWITCHTEC_INVALID_HFID)
		desc->hw->ctrl |= SWITCHTEC_SE_DFM;

	if (flags & DMA_PREP_INTERRUPT)
		desc->hw->ctrl |= SWITCHTEC_SE_LIOF;

	if (flags & DMA_PREP_FENCE)
		desc->hw->ctrl |= SWITCHTEC_SE_BRR;

	desc->txd.flags = flags;

	desc->completed = false;
	if (type == MEMCPY) {
		desc->hw->opc = SWITCHTEC_DMA_OPC_MEMCPY;
		desc->hw->saddr_lo = cpu_to_le32(lower_32_bits(dma_src));
		desc->hw->saddr_hi = cpu_to_le32(upper_32_bits(dma_src));
	} else {
		desc->hw->opc = SWITCHTEC_DMA_OPC_WRIMM;
		desc->hw->widata_lo = cpu_to_le32(lower_32_bits(data));
		desc->hw->widata_hi = cpu_to_le32(upper_32_bits(data));
	}
	desc->hw->daddr_lo = cpu_to_le32(lower_32_bits(dma_dst));
	desc->hw->daddr_hi = cpu_to_le32(upper_32_bits(dma_dst));
	desc->hw->byte_cnt = cpu_to_le32(len);
	desc->hw->tlp_setting = 0;
	desc->hw->dfid = cpu_to_le16(dst_fid);
	desc->hw->sfid = cpu_to_le16(src_fid);
	swdma_chan->cid &= SWITCHTEC_SE_CID_MASK;
	desc->hw->cid = cpu_to_le16(swdma_chan->cid++);
	desc->orig_size = len;

	head++;
	head &= SWITCHTEC_DMA_RING_SIZE - 1;
	smp_store_release(&swdma_chan->head, head);

	/* return with the lock held, it will be released in tx_submit */

	return &desc->txd;

err_unlock:
	/*
	 * Keep sparse happy by restoring an even lock count on
	 * this lock.
	 */
	__acquire(swdma_chan->submit_lock);

	spin_unlock_bh(&swdma_chan->submit_lock);
	return NULL;
}

static struct dma_async_tx_descriptor *switchtec_dma_prep_memcpy(
		struct dma_chan *c, dma_addr_t dma_dst, dma_addr_t dma_src,
		size_t len, unsigned long flags)
	__acquires(swdma_chan->submit_lock)
{

	return switchtec_dma_prep_desc(c, MEMCPY, SWITCHTEC_INVALID_HFID,
				       dma_dst, SWITCHTEC_INVALID_HFID, dma_src,
				       0, len, flags);
}

static struct dma_async_tx_descriptor *switchtec_dma_prep_wimm_data(
		struct dma_chan *c, dma_addr_t dst, u64 data,
		unsigned long flags)
	__acquires(swdma_chan->submit_lock)
{
	return switchtec_dma_prep_desc(c, WIMM, SWITCHTEC_INVALID_HFID, dst,
				       SWITCHTEC_INVALID_HFID, 0, data,
				       sizeof(data), flags);
}

static dma_cookie_t switchtec_dma_tx_submit(
		struct dma_async_tx_descriptor *desc)
	__releases(swdma_chan->submit_lock)
{
	struct switchtec_dma_chan *swdma_chan =
		to_switchtec_dma_chan(desc->chan);
	dma_cookie_t cookie;

	cookie = dma_cookie_assign(desc);

	spin_unlock_bh(&swdma_chan->submit_lock);

	return cookie;
}

static enum dma_status switchtec_dma_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	enum dma_status ret;

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

	rcu_read_lock();
	if (!rcu_dereference(swdma_dev->pdev)) {
		rcu_read_unlock();
		return;
	}

	spin_lock_bh(&swdma_chan->submit_lock);
	writew(swdma_chan->head, &swdma_chan->mmio_chan_hw->sq_tail);
	spin_unlock_bh(&swdma_chan->submit_lock);

	rcu_read_unlock();
}

static irqreturn_t switchtec_dma_isr(int irq, void *chan)
{
	struct switchtec_dma_chan *swdma_chan = chan;

	if (swdma_chan->comp_ring_active)
		tasklet_schedule(&swdma_chan->desc_task);

	return IRQ_HANDLED;
}

static irqreturn_t switchtec_dma_chan_status_isr(int irq, void *dma)
{
	struct switchtec_dma_dev *swdma_dev = dma;

	tasklet_schedule(&swdma_dev->chan_status_task);

	return IRQ_HANDLED;
}

static void switchtec_dma_free_desc(struct switchtec_dma_chan *swdma_chan)
{
	int i;
	size_t size;
	struct switchtec_dma_dev *swdma_dev = swdma_chan->swdma_dev;
	struct pci_dev *pdev;

	rcu_read_lock();
	pdev = rcu_dereference(swdma_dev->pdev);
	if (!pdev) {
		rcu_read_unlock();
		return;
	}
	rcu_read_unlock();

	size = SWITCHTEC_DMA_SQ_SIZE * sizeof(*swdma_chan->hw_sq);
	if (swdma_chan->hw_sq)
		dmam_free_coherent(&pdev->dev, size, swdma_chan->hw_sq,
				   swdma_chan->dma_addr_sq);

	size = SWITCHTEC_DMA_CQ_SIZE * sizeof(*swdma_chan->hw_cq);
	if (swdma_chan->hw_cq)
		dmam_free_coherent(&pdev->dev, size, swdma_chan->hw_cq,
				   swdma_chan->dma_addr_cq);

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
	struct pci_dev *pdev;
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
	size_t size;
	struct switchtec_dma_desc *desc;
	int rc;
	int i;

	swdma_chan->head = swdma_chan->tail = 0;
	swdma_chan->cq_tail = 0;

	rcu_read_lock();
	pdev = rcu_dereference(swdma_dev->pdev);
	if (!pdev) {
		rcu_read_unlock();
		return -ENODEV;
	}
	rcu_read_unlock();

	size = SWITCHTEC_DMA_SQ_SIZE * sizeof(*swdma_chan->hw_sq);
	swdma_chan->hw_sq = dmam_alloc_coherent(&pdev->dev, size,
						&swdma_chan->dma_addr_sq,
						GFP_KERNEL);
	if (!swdma_chan->hw_sq) {
		rc = -ENOMEM;
		goto free_and_exit;
	}

	size = SWITCHTEC_DMA_CQ_SIZE * sizeof(*swdma_chan->hw_cq);
	swdma_chan->hw_cq = dmam_alloc_coherent(&pdev->dev, size,
						&swdma_chan->dma_addr_cq,
						GFP_KERNEL);
	if (!swdma_chan->hw_cq) {
		rc = -ENOMEM;
		goto free_and_exit;
	}

	memset(swdma_chan->hw_cq, 0, size);

	/* reset host phase tag */
	swdma_chan->phase_tag = 0;

	size = sizeof(*swdma_chan->desc_ring);
	swdma_chan->desc_ring = kcalloc(SWITCHTEC_DMA_RING_SIZE,
					size, GFP_KERNEL);
	if (!swdma_chan->desc_ring) {
		rc = -ENOMEM;
		goto free_and_exit;
	}

	memset(swdma_chan->desc_ring, 0, SWITCHTEC_DMA_RING_SIZE * size);

	for (i = 0; i < SWITCHTEC_DMA_RING_SIZE; i++) {
		desc = kzalloc(sizeof(*desc), GFP_KERNEL);
		if (!desc) {
			rc = -ENOMEM;
			goto free_and_exit;
		}

		dma_async_tx_descriptor_init(&desc->txd, &swdma_chan->dma_chan);
		desc->txd.tx_submit = switchtec_dma_tx_submit;
		desc->hw = &swdma_chan->hw_sq[i];
		desc->completed = true;

		swdma_chan->desc_ring[i] = desc;
	}

	rcu_read_lock();
	pdev = rcu_dereference(swdma_dev->pdev);
	if (!pdev) {
		rcu_read_unlock();
		rc = -ENODEV;
		goto free_and_exit;
	}

	/* set sq/cq */
	writel(lower_32_bits(swdma_chan->dma_addr_sq), &chan_fw->sq_base_lo);
	writel(upper_32_bits(swdma_chan->dma_addr_sq), &chan_fw->sq_base_hi);
	writel(lower_32_bits(swdma_chan->dma_addr_cq), &chan_fw->cq_base_lo);
	writel(upper_32_bits(swdma_chan->dma_addr_cq), &chan_fw->cq_base_hi);

	writew((__force u16)cpu_to_le16(SWITCHTEC_DMA_SQ_SIZE),
	       &swdma_chan->mmio_chan_fw->sq_size);
	writew((__force u16)cpu_to_le16(SWITCHTEC_DMA_CQ_SIZE),
	       &swdma_chan->mmio_chan_fw->cq_size);

	rcu_read_unlock();
	return 0;

free_and_exit:
	switchtec_dma_free_desc(swdma_chan);
	return rc;
}

static int switchtec_dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct switchtec_dma_dev *swdma_dev = swdma_chan->swdma_dev;
	u32 perf_cfg;
	int rc;

	rc = switchtec_dma_alloc_desc(swdma_chan);
	if (rc)
		return rc;

	rc = enable_channel(swdma_chan);
	if (rc)
		return rc;

	rc = reset_channel(swdma_chan);
	if (rc)
		return rc;

	rc = unhalt_channel(swdma_chan);
	if (rc)
		return rc;

	swdma_chan->ring_active = true;
	swdma_chan->comp_ring_active = true;
	swdma_chan->cid = 0;

	dma_cookie_init(chan);

	rcu_read_lock();
	if (!rcu_dereference(swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	perf_cfg = readl(&swdma_chan->mmio_chan_fw->perf_cfg);
	rcu_read_unlock();

	dev_dbg(&chan->dev->device, "Burst Size:  0x%x",
		(perf_cfg >> PERF_BURST_SIZE_SHIFT) & PERF_BURST_SIZE_MASK);

	dev_dbg(&chan->dev->device, "Burst Scale: 0x%x",
		(perf_cfg >> PERF_BURST_SCALE_SHIFT) & PERF_BURST_SCALE_MASK);

	dev_dbg(&chan->dev->device, "Interval:    0x%x",
		(perf_cfg >> PERF_INTERVAL_SHIFT) & PERF_INTERVAL_MASK);

	dev_dbg(&chan->dev->device, "Arb Weight:  0x%x",
		(perf_cfg >> PERF_ARB_WEIGHT_SHIFT) & PERF_ARB_WEIGHT_MASK);

	dev_dbg(&chan->dev->device, "MRRS:        0x%x",
		(perf_cfg >> PERF_MRRS_SHIFT) & PERF_MRRS_MASK);

	return SWITCHTEC_DMA_SQ_SIZE;
}

static void switchtec_dma_free_chan_resources(struct dma_chan *chan)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct pci_dev *pdev;

	spin_lock_bh(&swdma_chan->submit_lock);
	swdma_chan->ring_active = false;
	spin_unlock_bh(&swdma_chan->submit_lock);

	spin_lock_bh(&swdma_chan->complete_lock);
	swdma_chan->comp_ring_active = false;
	spin_unlock_bh(&swdma_chan->complete_lock);

	switchtec_dma_chan_stop(swdma_chan);

	rcu_read_lock();
	pdev = rcu_dereference(swdma_chan->swdma_dev->pdev);
	if (pdev)
		synchronize_irq(swdma_chan->irq);
	rcu_read_unlock();

	tasklet_kill(&swdma_chan->desc_task);

	switchtec_dma_abort_desc(swdma_chan, 0);

	switchtec_dma_free_desc(swdma_chan);

	disable_channel(swdma_chan);
}

#define SWITCHTEC_DMA_CHAN_HW_REGS_SIZE 0x1000
#define SWITCHTEC_DMA_CHAN_FW_REGS_SIZE 0x80

static struct kobj_type switchtec_config_ktype;

static int switchtec_dma_chan_init(struct switchtec_dma_dev *swdma_dev, int i)
{
	struct dma_device *dma = &swdma_dev->dma_dev;
	struct pci_dev *pdev;
	struct device *dev;
	struct dma_chan *chan;
	struct switchtec_dma_chan *swdma_chan;
	struct chan_fw_regs __iomem *chan_fw;
	struct chan_hw_regs __iomem *chan_hw;
	u32 perf_cfg = 0;
	u32 valid_en_se;
	u32 thresh;
	int se_buf_len;
	int irq;
	int rc = 0;
	size_t offset;

	rcu_read_lock();
	pdev = rcu_dereference(swdma_dev->pdev);
	if (!pdev) {
		rcu_read_unlock();
		return -ENODEV;
	}
	rcu_read_unlock();

	dev = &pdev->dev;

	swdma_chan = devm_kzalloc(dev, sizeof(*swdma_chan), GFP_KERNEL);
	if (!swdma_chan)
		return -ENOMEM;

	swdma_chan->phase_tag = 0;
	swdma_chan->index = i;

	offset =  i * SWITCHTEC_DMA_CHAN_FW_REGS_SIZE;
	chan_fw = (struct chan_fw_regs __iomem *)(swdma_dev->mmio_chan_fw_all
			+ offset);

	offset =  i * SWITCHTEC_DMA_CHAN_HW_REGS_SIZE;
	chan_hw = (struct chan_hw_regs __iomem *)(swdma_dev->mmio_chan_hw_all
			+ offset);

	swdma_dev->swdma_chans[i] = swdma_chan;
	swdma_chan->mmio_chan_fw = chan_fw;
	swdma_chan->mmio_chan_hw = chan_hw;
	swdma_chan->swdma_dev = swdma_dev;

	rc = pause_reset_channel(swdma_chan);
	if (rc) {
		dev_err(dev, "Channel %d: pause and reset channel failed\n", i);
		return rc;
	}

	perf_cfg = readl(&chan_fw->perf_cfg);

	/* init perf tuner */
	perf_cfg = PERF_BURST_SCALE << PERF_BURST_SCALE_SHIFT;
	perf_cfg |= PERF_MRRS << PERF_MRRS_SHIFT;
	perf_cfg |= PERF_INTERVAL << PERF_INTERVAL_SHIFT;
	perf_cfg |= PERF_BURST_SIZE << PERF_BURST_SIZE_SHIFT;
	perf_cfg |= PERF_ARB_WEIGHT << PERF_ARB_WEIGHT_SHIFT;

	writel(perf_cfg, &chan_fw->perf_cfg);

	valid_en_se = readl(&chan_fw->valid_en_se);

	dev_dbg(dev, "Channel %d: SE buffer base %d\n",
		i, (valid_en_se >> SE_BUF_BASE_SHIFT) & SE_BUF_BASE_MASK);

	se_buf_len = (valid_en_se >> SE_BUF_LEN_SHIFT) & SE_BUF_LEN_MASK;
	dev_dbg(dev, "Channel %d: SE buffer count %d\n", i, se_buf_len);

	thresh = se_buf_len / 2;
	valid_en_se |= (thresh & SE_THRESH_MASK) << SE_THRESH_SHIFT;
	writel(valid_en_se , &chan_fw->valid_en_se);

	/* request irqs */
	irq = readl(&chan_fw->int_vec);
	dev_dbg(dev, "Channel %d: CE irq vector %d\n", i, irq);

	irq = pci_irq_vector(pdev, irq);
	if (irq < 0)
		return irq;

	rc = devm_request_irq(dev, irq, switchtec_dma_isr, 0, KBUILD_MODNAME,
			      swdma_chan);
	if (rc)
		return rc;

	swdma_chan->irq = irq;

	spin_lock_init(&swdma_chan->hw_ctrl_lock);
	spin_lock_init(&swdma_chan->submit_lock);
	spin_lock_init(&swdma_chan->complete_lock);
	tasklet_init(&swdma_chan->desc_task, switchtec_dma_desc_task,
		     (unsigned long)swdma_chan);

	chan = &swdma_chan->dma_chan;
	chan->device = dma;
	dma_cookie_init(chan);
	list_add_tail(&chan->device_node, &dma->channels);

	swdma_chan->is_fabric = swdma_dev->is_fabric;
	list_add_tail(&swdma_chan->list, &chan_list);

	return 0;
}

void switchtec_chan_kobject_del(struct switchtec_dma_chan *swdma_chan);

static int switchtec_dma_chan_free(struct switchtec_dma_chan *swdma_chan)
{
	struct pci_dev *pdev;

	rcu_read_lock();
	pdev = rcu_dereference(swdma_chan->swdma_dev->pdev);
	if (!pdev) {
		rcu_read_unlock();
		return -ENODEV;
	}
	rcu_read_unlock();

	spin_lock_bh(&swdma_chan->submit_lock);
	swdma_chan->ring_active = false;
	spin_unlock_bh(&swdma_chan->submit_lock);

	spin_lock_bh(&swdma_chan->complete_lock);
	swdma_chan->comp_ring_active = false;
	spin_unlock_bh(&swdma_chan->complete_lock);

	devm_free_irq(&pdev->dev, swdma_chan->irq, swdma_chan);

	switchtec_chan_kobject_del(swdma_chan);

	switchtec_dma_chan_stop(swdma_chan);

	return 0;
}

static int switchtec_dma_chans_release(struct switchtec_dma_dev *swdma_dev)
{
	int i;

	for (i = 0; i < swdma_dev->chan_cnt; i++)
		switchtec_dma_chan_free(swdma_dev->swdma_chans[i]);

	return 0;
}

static int switchtec_dma_chans_enumerate(struct switchtec_dma_dev *swdma_dev,
					 int chan_cnt)
{
	struct dma_device *dma = &swdma_dev->dma_dev;
	struct pci_dev *pdev;
	int base;
	int cnt;
	int rc;
	int i;

	rcu_read_lock();
	pdev = rcu_dereference(swdma_dev->pdev);
	if (!pdev) {
		rcu_read_unlock();
		return -ENODEV;
	}
	rcu_read_unlock();

	swdma_dev->swdma_chans = devm_kcalloc(&pdev->dev, chan_cnt,
					      sizeof(*swdma_dev->swdma_chans),
					      GFP_KERNEL);

	if (!swdma_dev->swdma_chans)
		return -ENOMEM;

	base = readw(&swdma_dev->mmio_dmac_cap->se_buf_base);
	cnt = readw(&swdma_dev->mmio_dmac_cap->se_buf_cnt);

	dev_dbg(&pdev->dev, "EP SE buffer base %d\n", base);
	dev_dbg(&pdev->dev, "EP SE buffer count %d\n", cnt);

	INIT_LIST_HEAD(&dma->channels);

	for (i = 0; i < chan_cnt; i++) {
		rc = switchtec_dma_chan_init(swdma_dev, i);
		if (rc) {
			dev_err(&pdev->dev, "Channel %d: init channel failed\n",
				i);
			goto err_exit;
		}
	}

	return chan_cnt;

err_exit:
	switchtec_dma_chans_release(swdma_dev);
	return rc;
}

struct switchtec_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct dma_chan *, char *);
	ssize_t (*store)(struct dma_chan *, const char *, size_t);
};

static ssize_t burst_scale_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
	u32 perf_cfg = 0;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	perf_cfg = readl(&chan_fw->perf_cfg);
	perf_cfg >>= PERF_BURST_SCALE_SHIFT;
	perf_cfg &= PERF_BURST_SCALE_MASK;

	rcu_read_unlock();
	return sprintf(page, "0x%x\n", perf_cfg);
}

static ssize_t burst_scale_store(struct dma_chan *chan, const char *page,
				 size_t count)
{
	int burst_scale;
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
	u32 perf_cfg;
	ssize_t ret = count;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		ret = -ENODEV;
		goto err_unlock;
	}

	if (chan->client_count)
		goto err_unlock;

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
static struct switchtec_sysfs_entry burst_scale_attr = __ATTR_RW(burst_scale);

static ssize_t burst_size_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
	u32 burst_size = 0;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	burst_size = readl(&chan_fw->perf_cfg);
	burst_size >>= PERF_BURST_SIZE_SHIFT;
	burst_size &= PERF_BURST_SIZE_MASK;

	rcu_read_unlock();
	return sprintf(page, "0x%x\n", burst_size);
}

static ssize_t burst_size_store(struct dma_chan *chan, const char *page,
				size_t count)
{
	int burst_size;
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
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


static struct switchtec_sysfs_entry burst_size_attr = __ATTR_RW(burst_size);

static ssize_t arb_weight_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
	u32 weight = 0;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	weight = readl(&chan_fw->perf_cfg);
	weight >>= PERF_ARB_WEIGHT_SHIFT;
	weight &= PERF_ARB_WEIGHT_MASK;

	rcu_read_unlock();
	return sprintf(page, "0x%x\n", weight);
}

static ssize_t arb_weight_store(struct dma_chan *chan, const char *page,
				size_t count)
{
	int weight;
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
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
static struct switchtec_sysfs_entry arb_weight_attr = __ATTR_RW(arb_weight);

static ssize_t interval_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
	u32 interval = 0;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	interval = readl(&chan_fw->perf_cfg);
	interval >>= PERF_INTERVAL_SHIFT;
	interval &= PERF_INTERVAL_MASK;

	rcu_read_unlock();
	return sprintf(page, "%x\n", interval);
}

static ssize_t interval_store(struct dma_chan *chan, const char *page,
			      size_t count)
{
	int interval;
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
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

static struct switchtec_sysfs_entry interval_attr = __ATTR_RW(interval);

static ssize_t mrrs_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
	u32 mrrs = 0;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	mrrs = readl(&chan_fw->perf_cfg);
	mrrs >>= PERF_MRRS_SHIFT;
	mrrs &= PERF_MRRS_MASK;

	rcu_read_unlock();
	return sprintf(page, "0x%x\n", mrrs);
}

static ssize_t mrrs_store(struct dma_chan *chan, const char *page, size_t count)
{
	int mrrs;
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
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

static struct switchtec_sysfs_entry mrrs_attr = __ATTR_RW(mrrs);

static ssize_t se_count_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
	u64 count = 0;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	count = le32_to_cpu((__force __le32)
			readl(&chan_fw->perf_fetched_se_cnt_hi));
	count <<= 32;
	count |= le32_to_cpu((__force __le32)
			readl(&chan_fw->perf_fetched_se_cnt_lo));

	rcu_read_unlock();
	return sprintf(page, "0x%llx\n", count);
}

static struct switchtec_sysfs_entry se_count_attr = __ATTR_RO(se_count);

static ssize_t byte_count_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
	u64 count = 0;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	count = le32_to_cpu((__force __le32)readl(&chan_fw->perf_byte_cnt_hi));
	count <<= 32;
	count |= le32_to_cpu((__force __le32)readl(&chan_fw->perf_byte_cnt_lo));

	rcu_read_unlock();
	return sprintf(page, "0x%llx\n", count);
}

static struct switchtec_sysfs_entry byte_count_attr = __ATTR_RO(byte_count);

static ssize_t se_pending_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
	u16 count = 0;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	count = le16_to_cpu((__force __le16)readw(&chan_fw->perf_se_pending));

	rcu_read_unlock();
	return sprintf(page, "0x%x\n", count);
}

static struct switchtec_sysfs_entry se_pending_attr = __ATTR_RO(se_pending);

static ssize_t se_buf_empty_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
	u16 count = 0;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	count = le16_to_cpu((__force __le16)readw(&chan_fw->perf_se_buf_empty));

	rcu_read_unlock();
	return sprintf(page, "0x%x\n", count);
}

static struct switchtec_sysfs_entry se_buf_empty_attr = __ATTR_RO(se_buf_empty);

static ssize_t chan_idle_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
	u32 ratio = 0;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	ratio = le32_to_cpu((__force __le32)readl(&chan_fw->perf_chan_idle));

	rcu_read_unlock();
	return sprintf(page, "0x%x\n", ratio);
}

static struct switchtec_sysfs_entry chan_idle_attr = __ATTR_RO(chan_idle);

static ssize_t latency_max_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
	u32 lat = 0;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	lat = le32_to_cpu((__force __le32)readl(&chan_fw->perf_lat_max));

	rcu_read_unlock();
	return sprintf(page, "0x%x\n", lat);
}

static struct switchtec_sysfs_entry latency_max_attr = __ATTR_RO(latency_max);

static ssize_t latency_min_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
	u32 lat = 0;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	lat = le32_to_cpu((__force __le32)readl(&chan_fw->perf_lat_min));

	rcu_read_unlock();
	return sprintf(page, "0x%x\n", lat);
}

static struct switchtec_sysfs_entry latency_min_attr = __ATTR_RO(latency_min);

static ssize_t latency_last_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
	u32 lat = 0;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	lat = le32_to_cpu((__force __le32)readl(&chan_fw->perf_lat_last));

	rcu_read_unlock();
	return sprintf(page, "0x%x\n", lat);
}

static struct switchtec_sysfs_entry latency_last_attr = __ATTR_RO(latency_last);

static ssize_t latency_selector_show(struct dma_chan *chan, char *page)
{
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
	u32 lat = 0;

	rcu_read_lock();
	if (!rcu_dereference(swdma_chan->swdma_dev->pdev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	lat = le32_to_cpu((__force __le32)
			readl(&chan_fw->perf_latency_selector));

	strcat(page, "To select a latency type, write the type number (1 ~ 4) to this file.\n\n");

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

	strcat(page, "(4) SE Processing latency");
	if (lat & SWITCHTEC_LAT_SE_PROCESS)
		strcat(page, " (*)\n");
	else
		strcat(page, "\n");

	strcat(page, "(5) FW NP TLP latency");
	if (lat & SWITCHTEC_LAT_FW_NP)
		strcat(page, " (*)\n");
	else
		strcat(page, "\n");

	strcat(page, "\n");

	rcu_read_unlock();
	return strlen(page);
}

static ssize_t latency_selector_store(struct dma_chan *chan, const char *page,
				      size_t count)
{
	int lat_type;
	struct switchtec_dma_chan *swdma_chan = to_switchtec_dma_chan(chan);
	struct chan_fw_regs __iomem *chan_fw = swdma_chan->mmio_chan_fw;
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
		switch (lat_type) {
		case 1:
			lat_type = SWITCHTEC_LAT_SE_FETCH;
			break;
		case 2:
			lat_type = SWITCHTEC_LAT_VDM;
			break;
		case 3:
			lat_type = SWITCHTEC_LAT_RD_IMM;
			break;
		case 4:
			lat_type = SWITCHTEC_LAT_SE_PROCESS;
			break;
		case 5:
			lat_type = SWITCHTEC_LAT_FW_NP;
			break;
		default:
			ret = -EINVAL;
			goto err_unlock;
		};
		writel(lat_type, &chan_fw->perf_latency_selector);
	}

err_unlock:
	rcu_read_unlock();

	return ret;
}

static struct switchtec_sysfs_entry latency_selector_attr =
__ATTR_RW(latency_selector);

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

static const struct sysfs_ops switchtec_config_sysfs_ops = {
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

static struct attribute_group switchtec_config_group = {
    .attrs = switchtec_config_attrs,
};

static const struct attribute_group *switchtec_config_groups[] = {
    &switchtec_config_group,
    NULL,
};

static struct kobj_type switchtec_config_ktype = {
	.sysfs_ops = &switchtec_config_sysfs_ops,
	.default_groups = switchtec_config_groups,
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

static struct sysfs_ops switchtec_pmon_sysfs_ops = {
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

static struct attribute_group switchtec_pmon_group = {
    .attrs = switchtec_pmon_attrs,
};

static const struct attribute_group *switchtec_pmon_groups[] = {
    &switchtec_pmon_group,
    NULL,
};

static struct kobj_type switchtec_pmon_ktype = {
	.sysfs_ops = &switchtec_pmon_sysfs_ops,
	.default_groups = switchtec_pmon_groups,
};

static void switchtec_chan_kobject_add(struct switchtec_dma_chan *swdma_chan)
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

static void switchtec_kobject_add(struct switchtec_dma_dev *swdma_dev)
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

static bool is_fabric_dma(struct dma_device *dma)
{
	struct switchtec_dma_dev *d;

	list_for_each_entry(d, &dma_list, list) {
		if (dma == &d->dma_dev)
			return d->is_fabric;
	}

	return false;
}

static int execute_cmd(struct switchtec_dma_dev *swdma_dev, u32 cmd,
		       const void *input, size_t input_size, void *output,
		       size_t *output_size)
{
	struct pci_dev *pdev;
	unsigned long wait_timeout;
	enum cmd_status status;
	size_t size;
	int ret = 0;

	mutex_lock(&swdma_dev->cmd_mutex);

	rcu_read_lock();
	pdev = rcu_dereference(swdma_dev->pdev);
	if (!pdev) {
		rcu_read_unlock();
		ret = -ENODEV;
		goto out;
	}

	swdma_dev->cmd->status = CMD_STATUS_IDLE;
	memset(swdma_dev->cmd->data, 0xFF, CMD_OUTPUT_SIZE);

	memcpy_toio(&swdma_dev->mmio_fabric_cmd->input, input, input_size);
	iowrite32(cmd, &swdma_dev->mmio_fabric_cmd->command);

	rcu_read_unlock();

	wait_timeout = jiffies + msecs_to_jiffies(CMD_TIMEOUT_MSECS);
	do {
		status = swdma_dev->cmd->status;

		if (time_after_eq(jiffies, wait_timeout)) {
			dev_err(&pdev->dev, "CMD %d: timeout!\n", cmd);
			ret = -ETIME;
			goto out;
		}
		if (status == CMD_STATUS_DONE || status == CMD_STATUS_ERROR)
			break;
		cpu_relax();
	} while (1);

	if (status != CMD_STATUS_DONE && status != CMD_STATUS_ERROR) {
		ret = -EBADMSG;
		goto out;
	}

	if (output && output_size) {
		size = *output_size > swdma_dev->cmd->output_size ?
		       swdma_dev->cmd->output_size : *output_size;
		memcpy(output, swdma_dev->cmd->data, size);

		*output_size = size;
	}

out:
	mutex_unlock(&swdma_dev->cmd_mutex);
	return ret;
}

bool is_switchtec_fabric(struct dma_chan *c)
{
	struct switchtec_dma_chan *swdma_chan;
	list_for_each_entry(swdma_chan, &chan_list, list)
		if (c == &swdma_chan->dma_chan)
			return swdma_chan->is_fabric;

	return false;
}
EXPORT_SYMBOL(is_switchtec_fabric);

struct dma_device *switchtec_fabric_get_dma_device(char *name)
{
	struct switchtec_dma_dev *d;
	char dev_name[16];

	list_for_each_entry(d, &dma_list, list) {
		sprintf(dev_name, "dma%d", d->dma_dev.dev_id);
		if (!strcmp(name, dev_name))
			return &d->dma_dev;
	}

	return NULL;
}
EXPORT_SYMBOL(switchtec_fabric_get_dma_device);

int switchtec_fabric_put_dma_device(struct dma_device *dma_dev)
{
	if (!dma_dev || !is_fabric_dma(dma_dev))
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL(switchtec_fabric_put_dma_device);

#define SWITCHTEC_LOCAL_PAX 0xff
int switchtec_fabric_get_pax_count(struct dma_device *dma_dev)
{
	struct switchtec_dma_dev *swdma_dev = to_switchtec_dma(dma_dev);
	u8 pax_id = SWITCHTEC_LOCAL_PAX;
	size_t size;
	int ret;

	struct {
		u8 pax_id;
		u8 pax_num;
		u8 local_phys_port_num;
		u8 host_port_num;
	} rsp;

	if (!dma_dev || !is_fabric_dma(dma_dev))
		return -EINVAL;

	size = sizeof(rsp);
	ret = execute_cmd(swdma_dev, CMD_GET_HOST_LIST, &pax_id, sizeof(pax_id),
			  &rsp, &size);
	if (ret < 0)
		return ret;

	return rsp.pax_num;
}
EXPORT_SYMBOL(switchtec_fabric_get_pax_count);

int switchtec_fabric_get_host_ports(struct dma_device *dma_dev, u8 pax_id,
				    int port_num,
				    struct switchtec_host_port *ports)
{
	struct switchtec_dma_dev *swdma_dev = to_switchtec_dma(dma_dev);
	size_t size;
	int rtn_port_num;
	int i;
	int ret;

	struct {
		u8 pax_id;
		u8 pax_num;
		u8 phys_pid;
		u8 host_port_num;
		u32 rsvd;
		struct {
			__le16 hfid;
			u8 phys_pid;
			u8 link_state;
		} host_ports[SWITCHTEC_HOST_PORT_NUM_PER_PAX];
	} *rsp;

	if (!dma_dev || !is_fabric_dma(dma_dev))
		return -EINVAL;

	rsp = kzalloc(sizeof(*rsp), GFP_KERNEL);
	if (!rsp)
		return -ENOMEM;

	size = sizeof(*rsp);
	ret = execute_cmd(swdma_dev, CMD_GET_HOST_LIST, &pax_id, sizeof(pax_id),
			  rsp, &size);
	if (ret < 0)
		goto out;

	rtn_port_num = port_num < rsp->host_port_num ? port_num :
		       rsp->host_port_num;

	for (i = 0; i < rtn_port_num; i++) {
		ports[i].hfid = le16_to_cpu(rsp->host_ports[i].hfid);
		ports[i].pax_id = rsp->pax_id;
		ports[i].phys_pid = rsp->host_ports[i].phys_pid;
		ports[i].link_state = rsp->host_ports[i].link_state;
	}

	ret = rtn_port_num;
out:
	kfree(rsp);
	return ret;
}
EXPORT_SYMBOL(switchtec_fabric_get_host_ports);

int switchtec_fabric_register_rhi_notify(struct dma_device *dma_dev,
					 struct notifier_block *nb)
{
	struct switchtec_dma_dev *swdma_dev;

	if (!dma_dev || !is_fabric_dma(dma_dev))
		return -EINVAL;

	swdma_dev = to_switchtec_dma(dma_dev);
	return atomic_notifier_chain_register(&swdma_dev->rhi_notifier_list,
					      nb);
}
EXPORT_SYMBOL_GPL(switchtec_fabric_register_rhi_notify);

int switchtec_fabric_unregister_rhi_notify(struct dma_device *dma_dev,
					   struct notifier_block *nb)
{
	struct switchtec_dma_dev *swdma_dev;

	if (!dma_dev || !is_fabric_dma(dma_dev) || !nb)
		return -EINVAL;

	swdma_dev = to_switchtec_dma(dma_dev);
	return atomic_notifier_chain_unregister(&swdma_dev->rhi_notifier_list,
						nb);
}
EXPORT_SYMBOL_GPL(switchtec_fabric_unregister_rhi_notify);

static irqreturn_t switchtec_dma_fabric_rhi_isr(int irq, void *dma)
{
	struct switchtec_dma_dev *swdma_dev = dma;

	atomic_notifier_call_chain(&swdma_dev->rhi_notifier_list, irq, NULL);

	return IRQ_HANDLED;
}

int switchtec_fabric_register_buffer(struct dma_device *dma_dev, u16 peer_hfid,
				     u8 buf_index, u64 buf_addr, u64 buf_size,
				     int *cookie)
{
	struct switchtec_dma_dev *swdma_dev = to_switchtec_dma(dma_dev);
	struct device *dev = dma_dev->dev;
	struct pci_dev *pdev;
	size_t size;
	int irq;
	int ret = 0;

	struct {
		__le16 hfid;
		u8 buf_index;
		u8 rsvd;
		__le32 addr_lo;
		__le32 addr_hi;
		__le32 size_lo;
		__le32 size_hi;
	} req = {
		.hfid = cpu_to_le16(peer_hfid),
		.buf_index = buf_index,
		.addr_lo = cpu_to_le32(lower_32_bits(buf_addr)),
		.addr_hi = cpu_to_le32(upper_32_bits(buf_addr)),
		.size_lo = cpu_to_le32(lower_32_bits(buf_size)),
		.size_hi = cpu_to_le32(upper_32_bits(buf_size)),
	};

	struct {
		u8 buf_index;
		u8 rsvd;
		__le16 buf_vec;
	} rsp;

	if (!dma_dev || !is_fabric_dma(dma_dev) || !cookie)
		return -EINVAL;

	size = sizeof(rsp);
	ret = execute_cmd(swdma_dev, CMD_REGISTER_BUF, &req, sizeof(req),
			  &rsp, &size);
	if (ret < 0)
		return ret;

	rcu_read_lock();
	pdev = rcu_dereference(swdma_dev->pdev);
	if (!pdev) {
		rcu_read_unlock();
		return -ENODEV;
	}

	irq = pci_irq_vector(pdev, le16_to_cpu(rsp.buf_vec));
	rcu_read_unlock();

	if (irq < 0)
		return -ENXIO;

	dev_dbg(dev, "Register Buffer (to hfid 0x%04x, index %d)\n", peer_hfid,
		buf_index);
	dev_dbg(dev, "    dma addr:     0x%08x_%08x\n", upper_32_bits(buf_addr),
		lower_32_bits(buf_addr));
	dev_dbg(dev, "    dma size:     0x%08x_%08x\n", upper_32_bits(buf_size),
		lower_32_bits(buf_size));
	dev_dbg(dev, "    vector:       %d", le16_to_cpu(rsp.buf_vec));
	dev_dbg(dev, "    irq (cookie): %d", irq);

	*cookie = irq;

	ret = devm_request_irq(dev, irq, switchtec_dma_fabric_rhi_isr, 0,
			       KBUILD_MODNAME, swdma_dev);

	return ret;
}
EXPORT_SYMBOL(switchtec_fabric_register_buffer);

int switchtec_fabric_unregister_buffer(struct dma_device *dma_dev,
				       u16 peer_hfid, u8 buf_index, int cookie)
{
	struct switchtec_dma_dev *swdma_dev = to_switchtec_dma(dma_dev);
	int ret = 0;

	struct {
		__le16 hfid;
		u8 buf_index;
		u8 rsvd;
	} req = {
		.hfid = cpu_to_le16(peer_hfid),
		.buf_index = buf_index,
	};

	if (!dma_dev || !is_fabric_dma(dma_dev))
		return -EINVAL;

	devm_free_irq(dma_dev->dev, cookie, swdma_dev);

	ret = execute_cmd(swdma_dev, CMD_UNREGISTER_BUF, &req, sizeof(req),
			  NULL, NULL);
	return ret;
}
EXPORT_SYMBOL(switchtec_fabric_unregister_buffer);

struct buffer_entry {
	__le16 hfid;
	u8 index;
	u8 rsvd1;
	__le32 addr_lo;
	__le32 addr_hi;
	__le32 size_lo;
	__le32 size_hi;
	__le16 rhi_index;
	u16 rsvd2;
	__le16 local_dfid;
	__le16 remote_dfid;
	__le16 local_rhi_dfid;
	__le16 remote_rhi_dfid;
};

int switchtec_fabric_get_peer_buffers(struct dma_device *dma_dev, u16 peer_hfid,
				      int buf_num,
				      struct switchtec_buffer *bufs)
{
	struct switchtec_dma_dev *swdma_dev = to_switchtec_dma(dma_dev);
	size_t size;
	int i;
	int rtn_buf_num;
	int ret;

	struct {
		u8 buf_num;
		u8 rsvd1[3];
		u32 rsvd2;
		struct buffer_entry bufs[SWITCHTEC_BUF_NUM_PER_HOST_PORT];
	} *rsp;

	if (!dma_dev || !is_fabric_dma(dma_dev))
		return -EINVAL;

	rsp = kzalloc(sizeof(*rsp), GFP_KERNEL);
	if (!rsp)
		return -ENOMEM;

	size = sizeof(*rsp);
	ret = execute_cmd(swdma_dev, CMD_GET_BUF_LIST, &peer_hfid,
			  sizeof(peer_hfid), rsp, &size);
	if (ret < 0)
		goto out;

	rtn_buf_num = buf_num < rsp->buf_num ? buf_num : rsp->buf_num;
	for (i = 0; i < rtn_buf_num; i++) {
		struct switchtec_buffer *buf = &bufs[i];
		struct buffer_entry *rsp_buf = &rsp->bufs[i];

		buf->from_hfid = le16_to_cpu(rsp_buf->hfid);
		buf->to_hfid = swdma_dev->hfid;
		buf->index = rsp_buf->index;
		buf->dma_addr = le32_to_cpu(rsp_buf->addr_hi);
		buf->dma_addr <<= 32;
		buf->dma_addr |= le32_to_cpu(rsp_buf->addr_lo);
		buf->dma_size = le32_to_cpu(rsp_buf->size_hi);
		buf->dma_size <<= 32;
		buf->dma_size |= le32_to_cpu(rsp_buf->size_lo);
		buf->rhi_index = le16_to_cpu(rsp_buf->rhi_index);
		buf->local_dfid = le16_to_cpu(rsp_buf->local_dfid);
		buf->remote_dfid = le16_to_cpu(rsp_buf->remote_dfid);
		buf->local_rhi_dfid = le16_to_cpu(rsp_buf->local_rhi_dfid);
		buf->remote_rhi_dfid = le16_to_cpu(rsp_buf->remote_rhi_dfid);
	}

	ret = rtn_buf_num;
out:
	kfree(rsp);
	return ret;
}
EXPORT_SYMBOL(switchtec_fabric_get_peer_buffers);

int switchtec_fabric_get_buffer_number(struct dma_device *dma_dev)
{
	struct switchtec_dma_dev *swdma_dev = to_switchtec_dma(dma_dev);
	u16 local_buf_index = 0;
	size_t size;
	int ret;

	struct {
		u8 total_buf_num;
		u8 local_buf_index;
		u8 rtn_buf_num;
		u8 rsvd;
	} rsp;

	if (!dma_dev || !is_fabric_dma(dma_dev))
		return -EINVAL;

	size = sizeof(rsp);
	ret = execute_cmd(swdma_dev, CMD_GET_OWN_BUF_LIST, &local_buf_index,
			  sizeof(local_buf_index), &rsp, &size);
	if (ret < 0)
		return ret;

	return rsp.total_buf_num;
}
EXPORT_SYMBOL(switchtec_fabric_get_buffer_number);

int switchtec_fabric_get_buffers(struct dma_device *dma_dev, int buf_num,
				 struct switchtec_buffer *bufs)
{
	struct switchtec_dma_dev *swdma_dev = to_switchtec_dma(dma_dev);
	u8 local_buf_index = 0;
	size_t size;
	int i, j;
	int rtn_buf_num = 0;
	int remain_buf_num = 0;
	int ret;

	struct {
		u16 buf_num;
		u16 buf_index;
		u8 rtn_buf_num;
		u8 rsvd[3];
		struct buffer_entry bufs[(CMD_OUTPUT_SIZE - 8) /
			sizeof(struct buffer_entry)];
	} *rsp;

	if (!dma_dev || !is_fabric_dma(dma_dev))
		return -EINVAL;

	rsp = kzalloc(sizeof(*rsp), GFP_KERNEL);
	if (!rsp)
		return -ENOMEM;

	i = 0;
	do {
		size = sizeof(*rsp);
		ret = execute_cmd(swdma_dev, CMD_GET_OWN_BUF_LIST,
				  &local_buf_index, sizeof(local_buf_index),
				  rsp, &size);
		if (ret < 0)
			goto out;

		if (!rtn_buf_num) {
			rtn_buf_num = buf_num < rsp->buf_num ? buf_num :
				      rsp->buf_num;
			remain_buf_num = rtn_buf_num;
		}

		j = 0;
		for (j = 0; j < rsp->rtn_buf_num; i++, j++) {
			struct switchtec_buffer *buf = &bufs[i];
			struct buffer_entry *rsp_buf = &rsp->bufs[j];

			buf->from_hfid = swdma_dev->hfid;
			buf->to_hfid = le16_to_cpu(rsp_buf->hfid);
			buf->index = rsp_buf->index;
			buf->dma_addr = le32_to_cpu(rsp_buf->addr_hi);
			buf->dma_addr <<= 32;
			buf->dma_addr |= le32_to_cpu(rsp_buf->addr_lo);
			buf->dma_size = le32_to_cpu(rsp_buf->size_hi);
			buf->dma_size <<= 32;
			buf->dma_size |= le32_to_cpu(rsp_buf->size_lo);
			buf->rhi_index = le16_to_cpu(rsp_buf->rhi_index);
			buf->local_dfid = le16_to_cpu(rsp_buf->local_dfid);
			buf->remote_dfid = le16_to_cpu(rsp_buf->remote_dfid);
			buf->local_rhi_dfid =
				le16_to_cpu(rsp_buf->local_rhi_dfid);
			buf->remote_rhi_dfid =
				le16_to_cpu(rsp_buf->remote_rhi_dfid);

			local_buf_index++;

			if (--remain_buf_num == 0)
				break;
		}
	} while (remain_buf_num);

	ret = rtn_buf_num;
out:
	kfree(rsp);
	return ret;
}
EXPORT_SYMBOL(switchtec_fabric_get_buffers);

static int switchtec_fabric_event_notify(struct dma_device *dma_dev,
					 struct switchtec_fabric_event *event)
{
	struct switchtec_dma_dev *swdma_dev;

	if (!dma_dev || !is_fabric_dma(dma_dev))
		return -EINVAL;

	swdma_dev = to_switchtec_dma(dma_dev);
	return atomic_notifier_call_chain(&swdma_dev->event_notifier_list,
					  event->type, event);
}

static void switchtec_dma_fabric_event_task(unsigned long data)
{
	struct switchtec_dma_dev *swdma_dev = (void *)data;
	struct switchtec_fabric_event *event;
	int cnt;

	while ((cnt = CIRC_CNT(swdma_dev->eq->head, swdma_dev->eq_tail,
			       SWITCHTEC_DMA_EQ_SIZE)) > 0) {
		event = &swdma_dev->eq->entries[swdma_dev->eq_tail];

		switchtec_fabric_event_notify(&swdma_dev->dma_dev, event);

		swdma_dev->eq_tail++;
		swdma_dev->eq_tail &= SWITCHTEC_DMA_EQ_SIZE - 1;
	}

	writel(swdma_dev->eq_tail,
	       &swdma_dev->mmio_fabric_ctrl->event_queue_tail);

	return;
}

static irqreturn_t switchtec_dma_fabric_event_isr(int irq, void *dma)
{
	struct switchtec_dma_dev *swdma_dev = dma;

	tasklet_schedule(&swdma_dev->fabric_event_task);

	return IRQ_HANDLED;
}

struct dma_async_tx_descriptor *switchtec_fabric_dma_prep_memcpy(
		struct dma_chan *c, u16 dst_dfid, dma_addr_t dma_dst,
		u16 src_dfid, dma_addr_t dma_src, size_t len,
		unsigned long flags)
	__acquires(swdma_chan->submit_lock)
{
	return switchtec_dma_prep_desc(c, MEMCPY, dst_dfid, dma_dst, src_dfid,
				       dma_src, 0, len, flags);
}
EXPORT_SYMBOL(switchtec_fabric_dma_prep_memcpy);

#define RHI_BASE_ADDR 0x135000
#define RHI_DATA 0xffffffff
struct dma_async_tx_descriptor *switchtec_fabric_dma_prep_rhi(
		struct dma_chan *c, u16 peer_dfid, u16 rhi_index,
		u16 local_dfid, unsigned long flags)
	__acquires(swdma_chan->submit_lock)
{
	dma_addr_t dst_addr = RHI_BASE_ADDR + rhi_index * 4;
	u32 data = RHI_DATA;

	return switchtec_dma_prep_desc(c, WIMM, peer_dfid, dst_addr,
				       local_dfid, 0, data, sizeof(data),
				       flags);
}
EXPORT_SYMBOL(switchtec_fabric_dma_prep_rhi);

struct dma_async_tx_descriptor *switchtec_fabric_dma_prep_wimm_data(
		struct dma_chan *c, u16 peer_dfid, dma_addr_t dma_dst,
		u16 local_dfid, u64 data, unsigned long flags)
	__acquires(swdma_chan->submit_lock)
{
	return switchtec_dma_prep_desc(c, WIMM, peer_dfid, dma_dst, local_dfid,
				       0, data, sizeof(data), flags);
}
EXPORT_SYMBOL(switchtec_fabric_dma_prep_wimm_data);

int switchtec_fabric_register_event_notify(struct dma_device *dma_dev,
					   struct notifier_block *nb)
{
	struct switchtec_dma_dev *swdma_dev;

	if (!dma_dev || !is_fabric_dma(dma_dev))
		return -EINVAL;

	swdma_dev = to_switchtec_dma(dma_dev);
	return atomic_notifier_chain_register(&swdma_dev->event_notifier_list,
					      nb);
}
EXPORT_SYMBOL_GPL(switchtec_fabric_register_event_notify);

int switchtec_fabric_unregister_event_notify(struct dma_device *dma_dev,
					     struct notifier_block *nb)
{
	struct switchtec_dma_dev *swdma_dev;

	if (!dma_dev || !is_fabric_dma(dma_dev))
		return -EINVAL;

	swdma_dev = to_switchtec_dma(dma_dev);
	return atomic_notifier_chain_unregister(&swdma_dev->event_notifier_list,
						nb);
}
EXPORT_SYMBOL_GPL(switchtec_fabric_unregister_event_notify);

static int switchtec_dma_init_fabric(struct switchtec_dma_dev *swdma_dev)
{
	struct pci_dev *pdev;
	int irq;
	size_t size;
	int rc = 0;

	if (!swdma_dev->is_fabric)
		return 0;

	rcu_read_lock();
	pdev = rcu_dereference(swdma_dev->pdev);
	if (!pdev) {
		rcu_read_unlock();
		return -ENODEV;
	}
	rcu_read_unlock();

	swdma_dev->cmd = dmam_alloc_coherent(&pdev->dev,
					     sizeof(*swdma_dev->cmd),
					     &swdma_dev->cmd_dma_addr,
					     GFP_KERNEL);
	if (!swdma_dev->cmd)
		return -ENOMEM;

	size = SWITCHTEC_DMA_EQ_SIZE * sizeof(struct switchtec_fabric_event) +
		offsetof(struct fabric_event_queue, entries);
	swdma_dev->eq = dmam_alloc_coherent(&pdev->dev, size,
					    &swdma_dev->eq_dma_addr,
					    GFP_KERNEL);
	if (!swdma_dev->eq)
		return -ENOMEM;

	swdma_dev->hfid = readw(&swdma_dev->mmio_fabric_ctrl->local_hfid);
	swdma_dev->hfid = le16_to_cpu((__force __le16)swdma_dev->hfid);

	writel((__force u32)cpu_to_le32(lower_32_bits(swdma_dev->cmd_dma_addr)),
	       &swdma_dev->mmio_fabric_ctrl->cmd_dma_addr_lo);
	writel((__force u32)cpu_to_le32(upper_32_bits(swdma_dev->cmd_dma_addr)),
	       &swdma_dev->mmio_fabric_ctrl->cmd_dma_addr_hi);

	mutex_init(&swdma_dev->cmd_mutex);

	tasklet_init(&swdma_dev->fabric_event_task,
		     switchtec_dma_fabric_event_task,
		     (unsigned long)swdma_dev);

	irq = readw(&swdma_dev->mmio_fabric_ctrl->event_vec);
	dev_dbg(&pdev->dev, "Fabric event irq vector %d\n", irq);

	irq = pci_irq_vector(pdev, irq);
	if (irq < 0) {
		rc = irq;
		goto err_exit;
	}

	rc = devm_request_irq(&pdev->dev, irq, switchtec_dma_fabric_event_isr,
			      0, KBUILD_MODNAME, swdma_dev);
	if (rc)
		goto err_exit;

	swdma_dev->event_irq = irq;

	swdma_dev->eq_tail = 0;

	writel((__force u32)cpu_to_le32(lower_32_bits(swdma_dev->eq_dma_addr)),
	       &swdma_dev->mmio_fabric_ctrl->event_dma_addr_lo);
	writel((__force u32)cpu_to_le32(upper_32_bits(swdma_dev->eq_dma_addr)),
	       &swdma_dev->mmio_fabric_ctrl->event_dma_addr_hi);

	ATOMIC_INIT_NOTIFIER_HEAD(&swdma_dev->event_notifier_list);
	ATOMIC_INIT_NOTIFIER_HEAD(&swdma_dev->rhi_notifier_list);

	writel((__force u32)cpu_to_le32(0),
	       &swdma_dev->mmio_fabric_ctrl->cmd_event_enable);
	writel((__force u32)cpu_to_le32(1),
	       &swdma_dev->mmio_fabric_ctrl->cmd_event_enable);

	return 0;

err_exit:
	if (swdma_dev->event_irq)
		devm_free_irq(&pdev->dev, swdma_dev->event_irq, swdma_dev);

	return rc;
}

static int switchtec_dma_create(struct pci_dev *pdev, bool is_fabric)
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
	int i;

	/*
	 * Create the switchtec dma device
	 */
	swdma_dev = devm_kzalloc(dev, sizeof(*swdma_dev), GFP_KERNEL);
	if (!swdma_dev)
		return -ENOMEM;

	bar = pcim_iomap_table(pdev)[0];
	swdma_dev->bar = bar;

	swdma_dev->mmio_dmac_ver = bar + SWITCHTEC_DMAC_VERSION_OFFSET;
	swdma_dev->mmio_dmac_cap = bar + SWITCHTEC_DMAC_CAPABILITY_OFFSET;
	swdma_dev->mmio_dmac_status = bar + SWITCHTEC_DMAC_STATUS_OFFSET;
	swdma_dev->mmio_dmac_ctrl = bar + SWITCHTEC_DMAC_CONTROL_OFFSET;
	swdma_dev->mmio_chan_hw_all = bar + SWITCHTEC_DMAC_CHAN_CTRL_OFFSET;
	swdma_dev->mmio_chan_fw_all = bar + SWITCHTEC_DMAC_CHAN_CFG_STS_OFFSET;

	if (is_fabric) {
		swdma_dev->mmio_fabric_cmd = swdma_dev->bar +
			SWITCHTEC_DMAC_FABRIC_CMD_OFFSET;
		swdma_dev->mmio_fabric_ctrl = swdma_dev->bar +
			SWITCHTEC_DMAC_FABRIC_CTRL_OFFSET;

		readw(&swdma_dev->mmio_fabric_ctrl->requestor_id);
		dev_info(dev, "Switchtec PAX DMA EP\n");
	} else {
		dev_info(dev, "Switchtec PSX/PFX DMA EP\n");
	}

	RCU_INIT_POINTER(swdma_dev->pdev, pdev);

	nr_vecs = pci_msix_vec_count(pdev);
	rc = pci_alloc_irq_vectors(pdev, nr_vecs, nr_vecs, PCI_IRQ_MSIX);
	if (rc < 0)
		goto err_exit;

	tasklet_init(&swdma_dev->chan_status_task,
		     switchtec_dma_chan_status_task,
		     (unsigned long)swdma_dev);

	irq = readw(&swdma_dev->mmio_dmac_cap->chan_sts_vec);
	dev_dbg(dev, "Channel pause irq vector %d\n", irq);

	irq = pci_irq_vector(pdev, irq);
	if (irq < 0) {
		rc = irq;
		goto err_exit;
	}

	rc = devm_request_irq(dev, irq, switchtec_dma_chan_status_isr, 0,
			      KBUILD_MODNAME, swdma_dev);
	if (rc)
		goto err_exit;

	swdma_dev->chan_status_irq = irq;

	chan_cnt = le32_to_cpu((__force __le32)
			       readl(&swdma_dev->mmio_dmac_cap->chan_cnt));
	if (!chan_cnt) {
		pci_err(pdev, "No channel configured.\n");
		rc = -ENXIO;
		goto err_exit;
	}

	swdma_dev->is_fabric = is_fabric;
	chan_cnt = switchtec_dma_chans_enumerate(swdma_dev, chan_cnt);
	if (chan_cnt < 0) {
		pci_err(pdev, "Failed to enumerate dma channels: %d\n",
			chan_cnt);
		rc = -ENXIO;
		goto err_exit;
	}

	swdma_dev->chan_cnt = chan_cnt;

	dma = &swdma_dev->dma_dev;
	dma->copy_align = DMAENGINE_ALIGN_1_BYTE;
	dma_cap_set(DMA_MEMCPY, dma->cap_mask);
	dma_cap_set(DMA_PRIVATE, dma->cap_mask);
	dma->dev = get_device(&pdev->dev);

	dma->device_alloc_chan_resources = switchtec_dma_alloc_chan_resources;
	dma->device_free_chan_resources = switchtec_dma_free_chan_resources;
	dma->device_prep_dma_memcpy = switchtec_dma_prep_memcpy;
	dma->device_prep_dma_imm_data = switchtec_dma_prep_wimm_data;
	dma->device_issue_pending = switchtec_dma_issue_pending;
	dma->device_tx_status = switchtec_dma_tx_status;
	dma->device_pause = switchtec_dma_pause;
	dma->device_resume = switchtec_dma_resume;
	dma->device_terminate_all = switchtec_dma_terminate_all;
	dma->device_synchronize = switchtec_dma_synchronize;

	rc = switchtec_dma_init_fabric(swdma_dev);
	if (rc) {
		pci_err(pdev, "Failed to init fabric DMA: %d\n", rc);
		goto err_chans_release_exit;
	}

	rc = dma_async_device_register(dma);
	if (rc) {
		pci_err(pdev, "Failed to register dma device: %d\n", rc);
		goto err_chans_release_exit;
	}

	pci_info(pdev, "Channel count: %d\n", chan_cnt);

	i = 0;
	list_for_each_entry(chan, &dma->channels, device_node)
		pci_info(pdev, "Channel %d: %s\n", i++, dma_chan_name(chan));

	pci_set_drvdata(pdev, swdma_dev);

	switchtec_kobject_add(swdma_dev);

	list_add_tail(&swdma_dev->list, &dma_list);

	return 0;

err_chans_release_exit:
	switchtec_dma_chans_release(swdma_dev);

err_exit:
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

	rc = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (rc)
		goto err_disable;

	rc = pcim_iomap_regions(pdev, 1, KBUILD_MODNAME);
	if (rc)
		return rc;

	pci_set_master(pdev);

	rc = switchtec_dma_create(pdev, id->driver_data);
	if (rc)
		goto err_free_irq_vectors;

	pci_info(pdev, "Switchtec DMA Channels Registered\n");

	return 0;

err_free_irq_vectors:
	pci_free_irq_vectors(pdev);

err_disable:
	pci_disable_device(pdev);
	return rc;
}

static void switchtec_dma_remove(struct pci_dev *pdev)
{
	struct switchtec_dma_dev *swdma_dev = pci_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	switchtec_dma_chans_release(swdma_dev);

	tasklet_kill(&swdma_dev->chan_status_task);

	if (swdma_dev->is_fabric) {
		tasklet_kill(&swdma_dev->fabric_event_task);
		devm_free_irq(dev, swdma_dev->event_irq, swdma_dev);

		mutex_lock(&swdma_dev->cmd_mutex);
		rcu_assign_pointer(swdma_dev->pdev, NULL);
		synchronize_rcu();
		mutex_unlock(&swdma_dev->cmd_mutex);
	} else {
		rcu_assign_pointer(swdma_dev->pdev, NULL);
		synchronize_rcu();
	}

	devm_free_irq(dev, swdma_dev->chan_status_irq, swdma_dev);

	pci_free_irq_vectors(pdev);

	dma_async_device_unregister(&swdma_dev->dma_dev);
	put_device(swdma_dev->dma_dev.dev);

	pci_info(pdev, "Switchtec DMA Channels Unregistered\n");
}

#define MICROSEMI_VENDOR_ID 0x11f8

#define SWITCHTEC_PCI_DEVICE(device_id, is_fabric) \
	{ \
		.vendor     = MICROSEMI_VENDOR_ID, \
		.device     = device_id, \
		.subvendor  = PCI_ANY_ID, \
		.subdevice  = PCI_ANY_ID, \
		.class      = PCI_CLASS_SYSTEM_OTHER << 8, \
		.class_mask = 0xFFFFFFFF, \
		.driver_data = is_fabric, \
	}

#define SWITCHTEC_PCI100X_DEVICE(device_id, is_fabric) \
	{ \
		.vendor     = PCI_VENDOR_ID_EFAR, \
		.device     = device_id, \
		.subvendor  = PCI_ANY_ID, \
		.subdevice  = PCI_ANY_ID, \
		.class      = PCI_CLASS_SYSTEM_OTHER << 8, \
		.class_mask = 0xFFFFFFFF, \
		.driver_data = is_fabric, \
	}

static const struct pci_device_id switchtec_dma_pci_tbl[] = {
	SWITCHTEC_PCI_DEVICE(0x4000, 0),  //PFX 100XG4
	SWITCHTEC_PCI_DEVICE(0x4084, 0),  //PFX 84XG4
	SWITCHTEC_PCI_DEVICE(0x4068, 0),  //PFX 68XG4
	SWITCHTEC_PCI_DEVICE(0x4052, 0),  //PFX 52XG4
	SWITCHTEC_PCI_DEVICE(0x4036, 0),  //PFX 36XG4
	SWITCHTEC_PCI_DEVICE(0x4028, 0),  //PFX 28XG4
	SWITCHTEC_PCI_DEVICE(0x4100, 0),  //PSX 100XG4
	SWITCHTEC_PCI_DEVICE(0x4184, 0),  //PSX 84XG4
	SWITCHTEC_PCI_DEVICE(0x4168, 0),  //PSX 68XG4
	SWITCHTEC_PCI_DEVICE(0x4152, 0),  //PSX 52XG4
	SWITCHTEC_PCI_DEVICE(0x4136, 0),  //PSX 36XG4
	SWITCHTEC_PCI_DEVICE(0x4128, 0),  //PSX 28XG4
	SWITCHTEC_PCI_DEVICE(0x4200, 1),  //PAX 100XG4
	SWITCHTEC_PCI_DEVICE(0x4284, 1),  //PAX 84XG4
	SWITCHTEC_PCI_DEVICE(0x4268, 1),  //PAX 68XG4
	SWITCHTEC_PCI_DEVICE(0x4252, 1),  //PAX 52XG4
	SWITCHTEC_PCI_DEVICE(0x4236, 1),  //PAX 36XG4
	SWITCHTEC_PCI_DEVICE(0x4228, 1),  //PAX 28XG4
	SWITCHTEC_PCI_DEVICE(0x4352, 0),  //PFXA 52XG4
	SWITCHTEC_PCI_DEVICE(0x4336, 0),  //PFXA 36XG4
	SWITCHTEC_PCI_DEVICE(0x4328, 0),  //PFXA 28XG4
	SWITCHTEC_PCI_DEVICE(0x4452, 0),  //PSXA 52XG4
	SWITCHTEC_PCI_DEVICE(0x4436, 0),  //PSXA 36XG4
	SWITCHTEC_PCI_DEVICE(0x4428, 0),  //PSXA 28XG4
	SWITCHTEC_PCI_DEVICE(0x4552, 1),  //PAXA 52XG4
	SWITCHTEC_PCI_DEVICE(0x4536, 1),  //PAXA 36XG4
	SWITCHTEC_PCI_DEVICE(0x4528, 1),  //PAXA 28XG4
	SWITCHTEC_PCI_DEVICE(0x5000, 0),  //PFX 100XG5
	SWITCHTEC_PCI_DEVICE(0x5084, 0),  //PFX 84XG5
	SWITCHTEC_PCI_DEVICE(0x5068, 0),  //PFX 68XG5
	SWITCHTEC_PCI_DEVICE(0x5052, 0),  //PFX 52XG5
	SWITCHTEC_PCI_DEVICE(0x5036, 0),  //PFX 36XG5
	SWITCHTEC_PCI_DEVICE(0x5028, 0),  //PFX 28XG5
	SWITCHTEC_PCI_DEVICE(0x5100, 0),  //PSX 100XG5
	SWITCHTEC_PCI_DEVICE(0x5184, 0),  //PSX 84XG5
	SWITCHTEC_PCI_DEVICE(0x5168, 0),  //PSX 68XG5
	SWITCHTEC_PCI_DEVICE(0x5152, 0),  //PSX 52XG5
	SWITCHTEC_PCI_DEVICE(0x5136, 0),  //PSX 36XG5
	SWITCHTEC_PCI_DEVICE(0x5128, 0),  //PSX 28XG5
	SWITCHTEC_PCI_DEVICE(0x5300, 0),  //PFXA 100XG5
	SWITCHTEC_PCI_DEVICE(0x5384, 0),  //PFXA 84XG5
	SWITCHTEC_PCI_DEVICE(0x5368, 0),  //PFXA 68XG5
	SWITCHTEC_PCI_DEVICE(0x5352, 0),  //PFXA 52XG5
	SWITCHTEC_PCI_DEVICE(0x5336, 0),  //PFXA 36XG5
	SWITCHTEC_PCI_DEVICE(0x5328, 0),  //PFXA 28XG5
	SWITCHTEC_PCI_DEVICE(0x5400, 0),  //PSXA 100XG5
	SWITCHTEC_PCI_DEVICE(0x5484, 0),  //PSXA 84XG5
	SWITCHTEC_PCI_DEVICE(0x5468, 0),  //PSXA 68XG5
	SWITCHTEC_PCI_DEVICE(0x5452, 0),  //PSXA 52XG5
	SWITCHTEC_PCI_DEVICE(0x5436, 0),  //PSXA 36XG5
	SWITCHTEC_PCI_DEVICE(0x5428, 0),  //PSXA 28XG5
	SWITCHTEC_PCI100X_DEVICE(0x1001, 0), //PCI1001 16XG4
	SWITCHTEC_PCI100X_DEVICE(0x1002, 0), //PCI1002 12XG4
	SWITCHTEC_PCI100X_DEVICE(0x1003, 0), //PCI1003 16XG4
	SWITCHTEC_PCI100X_DEVICE(0x1004, 0), //PCI1004 16XG4
	SWITCHTEC_PCI100X_DEVICE(0x1005, 0), //PCI1005 16XG4
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
