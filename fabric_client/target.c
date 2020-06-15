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

static unsigned long data_buf_size = 0x400;
module_param(data_buf_size, ulong, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(server,
		 "Size of data buffer (server, default: 0x400)");

static char dma_name[32] = "";
module_param_string(dma_dev, dma_name, sizeof(dma_name), S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dev, "DMA device to use, like dma16 (client and server)");

static unsigned int peer_hfid;
module_param(peer_hfid, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(peer_hfid, "Peer host port HFID(server)");

struct fabric_dma_target {
	struct scratchpad *spd;
	unsigned long long *data;
	struct tasklet_struct data_process_task;
} target;

static void target_data_process_task(unsigned long data)
{
	struct fabric_dma_target *target = (void *)data;
	struct scratchpad *spd = target->spd;
	int i;

	printk("%s\n", __FUNCTION__);
	printk("copied size: 0x%llx\n", spd->copied_size);
	printk("pattern:     0x%llx\n", spd->pattern);
	printk("data[0]:     0x%llx\n", target->data[0]);

	for (i = 0; i < spd->copied_size / sizeof(spd->pattern); i++) {
		if (memcmp(&target->data[i], &spd->pattern,
			   sizeof(spd->pattern)))
			printk("Memory verification failed.\n");
	}

	printk("Memory verification succeeded.\n");
}

struct dma_device *dma_dev = NULL;

int spd_cookie = 0;
int data_cookie = 0;
static int rhi_notify(struct notifier_block *self,
		      unsigned long cookie, void *dev)
{
	printk("Receive RHI notification with cookie 0x%lx\n", cookie);

	if (cookie == spd_cookie)
		tasklet_schedule(&target.data_process_task);

	return NOTIFY_OK;
}

static struct notifier_block rhi_nb = {
	.notifier_call = rhi_notify,
};

static int __init dma_client_init(void)
{
	int ret;
	int *spd_buf;
	int *data_buf;
	bool rhi_notify = false;
	size_t spd_buf_size;
	dma_addr_t buf_dma_addr;
	int buf_num;
	int i;
	struct switchtec_buffer local_bufs[2];
	struct switchtec_buffer *buf;

	if (!strlen(dma_name)) {
		printk("Please specify dma_dev.\n");
		return -EINVAL;
	}

	if (!peer_hfid) {
		printk("Please specify peer_hfid.\n");
		return -EINVAL;
	}

	dma_dev = switchtec_fabric_get_dma_device(dma_name);
	if (!dma_dev) {
		printk("Failed to get DMA device.\n");
		return -ENODEV;
	}

	tasklet_init(&target.data_process_task,
		     target_data_process_task,
		     (unsigned long)&target);

	/*
	 * Register RHI notify
	 */
	ret = switchtec_fabric_register_rhi_notify(dma_dev, &rhi_nb);
	if (ret < 0) {
		printk("Failed to register RHI notify.\n");
		goto err_free_resource;
	}
	rhi_notify = true;

	/*
	 * Allocate spd DMA buffer
	 */
	spd_buf_size = PAGE_SIZE;
	spd_buf = dmam_alloc_coherent(dma_dev->dev, spd_buf_size,
				      &buf_dma_addr, GFP_KERNEL);
	if (!spd_buf) {
		printk("Failed to allocate spd DMA buffer.\n");
		ret = -ENOMEM;
		goto err_free_resource;
	}

	target.spd = (struct scratchpad *)spd_buf;
	/*
	 * Register spd DMA buffer to buffer slot 0 @ peer host
	 */
	ret = switchtec_fabric_register_buffer(dma_dev, peer_hfid, 0,
					       buf_dma_addr, spd_buf_size,
					       &spd_cookie);
	if (ret < 0) {
		printk("Failed to register spd DMA buffer to slot 0 @ peer 0x04%x.\n",
		       peer_hfid);
		ret = -ENXIO;
		goto err_free_resource;
	}

	/*
	 * Allocate data DMA buffer
	 */
	data_buf = dma_alloc_coherent(dma_dev->dev, data_buf_size, &buf_dma_addr,
				      GFP_KERNEL);
	if (!data_buf) {
		printk("Failed to allocate data DMA buffer.\n");
		return -ENOMEM;
		goto err_free_resource;
	}

	target.data = (unsigned long long *)data_buf;
	/*
	 * Register data DMA buffer to buffer slot 1 @ peer host
	 */
	ret = switchtec_fabric_register_buffer(dma_dev, peer_hfid,
					       1, buf_dma_addr,
					       data_buf_size, &data_cookie);
	if (ret < 0) {
		printk("Failed to register data DMA buffer to slot 1 @ peer 0x04%x.\n",
		       peer_hfid);
		ret = -ENXIO;
		goto err_free_resource;
	}

	buf_num = switchtec_fabric_get_buffer_number(dma_dev);

	ret = switchtec_fabric_get_buffers(dma_dev, buf_num, local_bufs);
	if (ret < 0) {
		printk("Failed to get local buffers.\n");
		ret = -ENXIO;
		goto err_free_resource;
	}

	for (i = 0; i < buf_num; i++) {
		buf = &local_bufs[i];
		printk("Local buffer #%d (%s)\n", i,
		       i == SCRATCHPAD_BUFFER_INDEX ?  "scratchpad" : "data");
		printk("    from hfid:       0x%04x\n", buf->from_hfid);
		printk("    to hfid:         0x%04x\n", buf->to_hfid);
		printk("    index:           %d\n", buf->index);
		printk("    dma address:     0x%08x_%08x\n",
		       upper_32_bits(buf->dma_addr),
		       lower_32_bits(buf->dma_addr));
		printk("    size:            0x%llx\n", buf->dma_size);
		printk("    rhi index:       %d\n", buf->rhi_index);
		printk("    local dfid:      0x%04x\n", buf->local_dfid);
		printk("    remote dfid:     0x%04x\n", buf->remote_dfid);
		printk("    local rhi dfid:  0x%04x\n", buf->local_rhi_dfid);
		printk("    remote rhi dfid: 0x%04x\n", buf->remote_rhi_dfid);
	}

	return 0;

err_free_resource:
	if (spd_cookie)
		switchtec_fabric_unregister_buffer(dma_dev, peer_hfid,
						   SCRATCHPAD_BUFFER_INDEX,
						   spd_cookie);
	if (data_cookie)
		switchtec_fabric_unregister_buffer(dma_dev, peer_hfid,
						   DATA_BUFFER_INDEX,
						   data_cookie);

	if (rhi_notify)
		switchtec_fabric_unregister_rhi_notify(dma_dev, &rhi_nb);

	switchtec_fabric_put_dma_device(dma_dev);
	return ret;
}

module_init(dma_client_init);

static void __exit dma_client_exit(void)
{
	int ret;

	ret = switchtec_fabric_unregister_buffer(dma_dev, peer_hfid,
						 SCRATCHPAD_BUFFER_INDEX,
						 spd_cookie);
	if (ret < 0)
		printk("Failed to unregister spd DMA buffer from slot %d @ peer 0x%x.\n",
		       SCRATCHPAD_BUFFER_INDEX, peer_hfid);

	ret = switchtec_fabric_unregister_buffer(dma_dev, peer_hfid,
						 DATA_BUFFER_INDEX,
						 data_cookie);
	if (ret < 0)
		printk("Failed to unregister data DMA buffer from slot %d @ peer 0x%x.\n",
		       DATA_BUFFER_INDEX, peer_hfid);

	ret = switchtec_fabric_unregister_rhi_notify(dma_dev, &rhi_nb);
	if (ret < 0)
		printk("Failed to unregister RHI notify.\n");

        switchtec_fabric_put_dma_device(dma_dev);
}

module_exit(dma_client_exit);
