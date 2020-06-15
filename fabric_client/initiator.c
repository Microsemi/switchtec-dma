#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/delay.h>

#include "../linux/switchtec_fabric_dma.h"
#include "protocol.h"

MODULE_LICENSE("Dual BSD/GPL");

static unsigned long long pattern = 0x11223344aabbccdd;
module_param(pattern, ullong, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(pattern, "Data pattern to send (default: 0x11223344aabbccdd)");

static char dma_name[32];
module_param_string(dma_dev, dma_name, sizeof(dma_name), S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dev, "DMA device to use, like dma16");

static char dma_channel_name[32] = "";
module_param_string(dma_chan, dma_channel_name, sizeof(dma_channel_name),
		    S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(chan,
		 "DMA channel to use, like chan0 (default: first available on the specified DMA device)");

static int peer_pax_id = -1;
module_param(peer_pax_id, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(peer_pax_id, "Peer host port PAX ID");

static int peer_pid = -1;
module_param(peer_pid, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(peer_pid, "Peer host port physical port ID");

static unsigned int peer_hfid = 0;
module_param(peer_hfid, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(peer_hfid, "Peer host port HFID");

static unsigned int timeout = 5;
module_param(timeout, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(timeout, "Time in seconds to wait for peer buffer registration (default: 5)");

struct dma_device *dma_dev = NULL;
struct dma_chan *dma_chan = NULL;

static bool dma_client_filter(struct dma_chan *chan, void *data)
{
	char full_chan_name[64];

	if (strlen(dma_channel_name)) {
		sprintf(full_chan_name, "%s%s", dma_name, dma_channel_name);
		return !strcmp(dma_chan_name(chan), full_chan_name);
	} else {
		return is_switchtec_fabric(chan);
	}
}

enum transaction {
	MEMCPY,
	WIMM,
	RHI,
};

static void dma_copy_callback(void *data)
{
	enum transaction *t = (enum transaction *)data;

	switch (*t) {
	case (MEMCPY):
		printk("MEMCPY done.\n");
		break;
	case (WIMM):
		printk("WIMM done.\n");
		break;
	case (RHI):
		printk("RHI done.\n");
		break;
	}
}

DECLARE_COMPLETION(event_comp);
static int event_notify(struct notifier_block *self,
			unsigned long event_type,
			void *event)
{
	struct switchtec_fabric_event *e = event;

	if ((e->reg_buf_data.hfid == peer_hfid) &&
	    (event_type == SWITCHTEC_FABRIC_REGISTER_BUFFER))
		complete(&event_comp);

	return NOTIFY_OK;
}

static struct notifier_block event_nb = {
	.notifier_call = event_notify,
};

void print_buffer_info(const char *desc, struct switchtec_buffer *buf)
{
	printk("%s\n", desc);
	printk("    peer buf slot:   %x\n", buf->index);
	printk("    dma address:     0x%08x_%08x\n",
	       upper_32_bits((unsigned long)buf->dma_addr),
	       lower_32_bits((unsigned long)buf->dma_addr));
	printk("    size:            0x%llx\n", buf->dma_size);
	printk("    rhi index:       0x%x\n", buf->rhi_index);
	printk("    peer hfid:       0x%04x\n", buf->from_hfid);
	printk("    local hfid:      0x%04x\n", buf->to_hfid);
	printk("    local dfid:      0x%04x\n", buf->local_dfid);
	printk("    remote dfid:     0x%04x\n", buf->remote_dfid);
	printk("    local rhi dfid:  0x%04x\n", buf->local_rhi_dfid);
	printk("    remote rhi dfid: 0x%04x\n", buf->remote_rhi_dfid);
}

static int __init dma_client_init(void)
{
	dma_cap_mask_t dma_mask;
	bool event_notify = false;
	struct switchtec_host_port host_ports[SWITCHTEC_HOST_PORT_NUM_PER_PAX];
	size_t size;
	struct switchtec_buffer peer_bufs[SWITCHTEC_BUF_NUM_PER_HOST_PORT];
	struct switchtec_buffer *spd_buf;
	struct switchtec_buffer *data_buf;
	int host_num;
	struct dma_async_tx_descriptor *tx;
	int i;
	size_t len;
	unsigned long long *src_buf;
	dma_addr_t src_dma_addr;
	dma_cookie_t cookie;
	enum transaction t1, t2, t3, t4;
	int ret;

	/*
	 * Specify either peer host PAX ID and Peer host port PID, or peer
	 * host port hfid
	 */
	if (((peer_pax_id == -1) || (peer_pid == -1)) && !peer_hfid) {
		printk("Please specify either peer_pax_id and peer_pid, or peer_hfid.\n");
		return -EINVAL;
	}

	if (!strlen(dma_name)) {
		printk("Please specify dma_dev.\n");
		return -EINVAL;
	}

	init_completion(&event_comp);

	dma_dev = switchtec_fabric_get_dma_device(dma_name);
	if (!dma_dev) {
		printk("Failed to get DMA device.\n");
		return -ENODEV;
	}

	/*
	 * Register event notify
	 */
	ret = switchtec_fabric_register_event_notify(dma_dev, &event_nb);
	if (ret < 0) {
		printk("Failed to register event notify.\n");
		goto err_free_resource;
	}
	event_notify = true;

	/*
	 * peer host PAX ID and Peer host port PID specified, get peer host port
	 * hfid
	 */
	if (!peer_hfid) {
		/*
		 * Get the number of PAX in fabric
		 */
		ret = switchtec_fabric_get_pax_count(dma_dev);
		if (ret < 0) {
			printk("Failed to get PAX count.\n");
			goto err_free_resource;
		}

		/*
		 * Get the host port list from peer PAX
		 */
		host_num = switchtec_fabric_get_host_ports(
				dma_dev, peer_pax_id,
				SWITCHTEC_HOST_PORT_NUM_PER_PAX,
				host_ports);

		/*
		 * Get peer HFID of specified host port
		 */
		for (i = 0; i < host_num; i++) {
			if (host_ports[i].phys_pid == peer_pid)
				peer_hfid = host_ports[i].hfid;
		}

		if (!peer_hfid) {
			printk("PAX %d, host port %d doesn't exist.\n",
			       peer_pax_id, peer_pid);
			ret = -ENODEV;
			goto err_free_resource;
		}
	}

	/*
	 * Get peer host buffers
	 */
	do {
		unsigned long t;
		ret = switchtec_fabric_get_peer_buffers(
				dma_dev, peer_hfid,
				SWITCHTEC_BUF_NUM_PER_HOST_PORT,
				peer_bufs);
		if (ret < 0) {
			printk("Failed to get buffers from peer 0x%04x\n",
			       peer_hfid);
			goto err_free_resource;
		} else if (ret > 0) {
			break;
		}

		t = wait_for_completion_timeout(&event_comp,
						msecs_to_jiffies(timeout * 1000));
		if (t == 0) {
			ret = -ETIME;
			goto err_free_resource;
		}
		/*
		 * To work around a bug in the firmware
		 */
		mdelay(100);
	} while(1);

	spd_buf = &peer_bufs[SCRATCHPAD_BUFFER_INDEX];
	print_buffer_info("Peer scratchpad DMA buffer", spd_buf);

	data_buf = &peer_bufs[DATA_BUFFER_INDEX];
	print_buffer_info("Peer data DMA buffer", data_buf);

	/*
	 * Request a Switchtec DMA channel
	 */
	dma_cap_zero(dma_mask);
	dma_cap_set(DMA_MEMCPY|DMA_PRIVATE, dma_mask);
	dma_chan = dma_request_channel(dma_mask, dma_client_filter, NULL);
	if (!dma_chan) {
		printk("Failed to get DMA channel.\n");
		ret = -ENODEV;
		goto err_free_resource;
	}

	/*
	 * Allocate source data buffer
	 */
	size = PAGE_SIZE;
	src_buf = dma_alloc_coherent(dma_dev->dev, size, &src_dma_addr,
				     GFP_KERNEL);

	/*
	 * Load buffer with pattern
	 */
	for (i = 0; i < size / sizeof(pattern); i++)
		src_buf[i] = pattern;

	/*
	 * Kick off data DMA transaction
	 */
	printk("MEMCPY to data buf\n");
	len = size;
	tx = switchtec_fabric_dma_prep_memcpy(dma_chan, data_buf->remote_dfid,
					      data_buf->dma_addr,
					      data_buf->local_dfid,
					      src_dma_addr, len,
					      DMA_PREP_INTERRUPT |
					      DMA_CTRL_ACK);
	if (!tx) {
		ret = -EIO;
		goto err_free_resource;
	}

	tx->callback = dma_copy_callback;
	t1 = MEMCPY;
	tx->callback_param = &t1;

	cookie = dmaengine_submit(tx);
	ret = dma_submit_error(cookie);
	if (ret)
		goto err_free_resource;

	dma_async_issue_pending(dma_chan);

	/*
	 * Update spd->copied_size
	 */
	printk("WIMM to spd buf, update copied size.\n");
	tx = switchtec_fabric_dma_prep_wimm_data(dma_chan, spd_buf->remote_dfid,
						 spd_buf->dma_addr,
						 spd_buf->local_dfid, len,
						 DMA_PREP_INTERRUPT |
						 DMA_CTRL_ACK);
	if (!tx) {
		ret = -EIO;
		goto err_free_resource;
	}

	tx->callback = dma_copy_callback;
	t2 = WIMM;
	tx->callback_param = &t2;

	cookie = dmaengine_submit(tx);
	ret = dma_submit_error(cookie);
	if (ret)
		goto err_free_resource;

	/*
	 * Update spd->pattern
	 */
	printk("WIMM to spd buf, update pattern.\n");
	tx = switchtec_fabric_dma_prep_wimm_data(dma_chan, spd_buf->remote_dfid,
						 spd_buf->dma_addr + 8,
						 spd_buf->local_dfid, pattern,
						 DMA_PREP_INTERRUPT |
						 DMA_CTRL_ACK);
	if (!tx) {
		ret = -EIO;
		goto err_free_resource;
	}

	tx->callback = dma_copy_callback;
	t3 = WIMM;
	tx->callback_param = &t3;

	cookie = dmaengine_submit(tx);
	ret = dma_submit_error(cookie);
	if (ret)
		goto err_free_resource;

	dma_async_issue_pending(dma_chan);

	dma_sync_wait(dma_chan, cookie);

	/*
	 * Trigger RHI for spd update
	 */
	printk("RHI to spd buf.\n");
	tx = switchtec_fabric_dma_prep_rhi(dma_chan, spd_buf->remote_rhi_dfid,
					   spd_buf->rhi_index,
					   spd_buf->local_rhi_dfid);
	if (!tx) {
		ret = -EIO;
		goto err_free_resource;
	}

	tx->callback = dma_copy_callback;
	t4 = RHI;
	tx->callback_param = &t4;

	cookie = dmaengine_submit(tx);
	ret = dma_submit_error(cookie);
	if (ret)
		goto err_free_resource;

	dma_async_issue_pending(dma_chan);

	dma_sync_wait(dma_chan, cookie);

	return 0;

err_free_resource:
	if (event_notify)
		switchtec_fabric_unregister_event_notify(dma_dev, &event_nb);

	if (dma_chan)
		dma_release_channel(dma_chan);

	switchtec_fabric_put_dma_device(dma_dev);
	return ret;
}

module_init(dma_client_init);

static void __exit dma_client_exit(void)
{
	int ret;

	ret = switchtec_fabric_unregister_event_notify(dma_dev, &event_nb);
	if (ret)
		printk("Failed to unregister event notify.\n");

	dma_release_channel(dma_chan);

	switchtec_fabric_put_dma_device(dma_dev);
}

module_exit(dma_client_exit);
