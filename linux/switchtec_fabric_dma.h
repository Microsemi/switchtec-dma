// SPDX-License-Identifier: GPL-2.0
/*
 * Microchip Switchtec(tm) DMA Controller Driver
 * Copyright (c) 2020, Microchip Corporation
 */

#ifndef _SWITCHTEC_FABRIC_DMA_H
#define _SWITCHTEC_FABRIC_DMA_H

#include <linux/dmaengine.h>

#define SWITCHTEC_HOST_PORT_NUM_PER_PAX 8
#define SWITCHTEC_BUF_NUM_PER_HOST_PORT 6

/**
 * enum ltssm_major - ltssm major state
 * @LTSSM_MAJOR_DETECT: Detect
 * @LTSSM_MAJOR_POLL: Polling
 * @LTSSM_MAJOR_CFG: Config
 * @LTSSM_MAJOR_L0: L0
 * @LTSSM_MAJOR_RECOVERY: Recovery
 * @LTSSM_MAJOR_DISABLE: Disable
 * @LTSSM_MAJOR_LOOPBK: Loopback
 * @LTSSM_MAJOR_HOTRST: Hot reset
 * @LTSSM_MAJOR_L0S: TX L0s
 * @LTSSM_MAJOR_L1: L1
 * @LTSSM_MAJOR_L2: L2
 * @LTSSM_MAJOR_NUMBER:
 */
enum ltssm_major {
	LTSSM_MAJOR_DETECT,
	LTSSM_MAJOR_POLL,
	LTSSM_MAJOR_CFG,
	LTSSM_MAJOR_L0,
	LTSSM_MAJOR_RECOVERY,
	LTSSM_MAJOR_DISABLE,
	LTSSM_MAJOR_LOOPBK,
	LTSSM_MAJOR_HOTRST,
	LTSSM_MAJOR_L0S,
	LTSSM_MAJOR_L1,
	LTSSM_MAJOR_L2,
	LTSSM_MAJOR_NUMBER,
};

/**
 * struct switchtec_host_port - host port information
 * @hfid: hfid of the host port
 * @pax_id: PAX ID of the host port
 * @phys_pid: Physical port ID of the host port
 * @link_state: Link state of the host port
 */
struct switchtec_host_port {
	u16 hfid;
	u8 pax_id;
	u8 phys_pid;
	enum ltssm_major link_state;
};

/**
 * struct switchtec_buffer - buffer information
 * @from_hfid: hfid on which the buffer is allocated
 * @to_hfid: hfid will use the buffer
 * @index: buffer index, 0 to SWITCHTEC_BUF_NUM_PER_HOST_PORT
 * @dma_addr: dma address of the buffer
 * @dma_size: size of the buffer
 * @rhi_index: RHI index of the buffer
 * @local_dfid: local DMA FID
 * @remote_dfid: remote DMA FID
 * @local_rhi_dfid: local RHI DMA FID
 * @remote_rhi_dfid: remote RHI DMA FID
 */
struct switchtec_buffer {
	u16 from_hfid;
	u16 to_hfid;
	u8 index;
	u64 dma_addr;
	u64 dma_size;
	u16 rhi_index;
	u16 local_dfid;
	u16 remote_dfid;
	u16 local_rhi_dfid;
	u16 remote_rhi_dfid;
};

/**
 * enum switchtec_fabric_event_type - fabric DMA event type
 * @SWITCHTEC_FABRIC_REGISTER_BUFFER: buffer registered to local host port
 * @SWITCHTEC_FABRIC_UNREGISTER_BUFFER: buffer unregistered from local host port
 * @SWITCHTEC_FABRIC_EVENT_OVERFLOW: F/W internal event buffer overflow
 */
enum switchtec_fabric_event_type {
	SWITCHTEC_FABRIC_REGISTER_BUFFER,
	SWITCHTEC_FABRIC_UNREGISTER_BUFFER,
	SWITCHTEC_FABRIC_EVENT_OVERFLOW,
};

/**
 * struct switchtec_fabric_event - fabric event
 * @type: event type
 * @reg_buf_data: register buffer event data structure
 * @unreg_buf_data: unregister buffer event data structure
 * @data: raw event data
 */
struct switchtec_fabric_event {
	enum switchtec_fabric_event_type type;
	union {
		/**
		 * struct switchtec_fabric_event - register buffer event data
		 * @hfid: hfid from which this buffer is registered
		 * @rhi_index: RHI index for this buffer
		 * @index: buffer index
		 */
		struct register_buf_data {
			u16 hfid;
			u16 rhi_index;
			u8 index;
			u8 rsvd[3];
		} reg_buf_data;

		/**
		 * struct switchtec_fabric_event - register buffer event data
		 * @hfid: hfid from which this buffer is registered
		 * @index: buffer index
		 */
		struct unregister_buf_data {
			u16 hfid;
			u8 index;
			u8 rsvd;
		} unreg_buf_data;

		u32 data[3];
	};
};

/**
 * is_switchtec_fabric - test if a channel is a switchtec fabric channel
 * @c: DMA channel

 * Test if a channel is a switchtec fabric channel
 */
bool is_switchtec_fabric(struct dma_chan *c);

/**
 * switchtec_fabric_get_dma_device - get a fabric DMA device
 * @name: name of a fabric dma device, like 'dma16'
 *
 * Get a DMA device with given name and increase its ref count.
 */
struct dma_device *switchtec_fabric_get_dma_device(char *name);

/**
 * switchtec_fabric_put_dma_device - put a fabric DMA device
 * @dma_dev: DMA device handle
 *
 * Decrease the ref count of a DMA device.
 */
int switchtec_fabric_put_dma_device(struct dma_device *dma_dev);

/**
 * switchtec_fabric_get_pax_count - get the PAX switch count in the fabric
 * @dma_dev: DMA device handle
 *
 * Return the number of PAX switch in the fabric
 */
int switchtec_fabric_get_pax_count(struct dma_device *dma_dev);

/**
 * switchtec_fabric_get_host_ports - get host port structure of a PAX switch
 * @dma_dev:  DMA device handle
 * @pax_id:   PAX id
 * @port_num: size of host port array
 * @ports:    array of host port structures
 *
 * Get at most the specified number of host port structures of the specified PAX
 */
int switchtec_fabric_get_host_ports(struct dma_device *dma_dev, u8 pax_id,
				    int port_num,
				    struct switchtec_host_port *ports);

/**
 * switchtec_fabric_register_rhi_notify - register callback for RHI notfication
 * @dma_dev: DMA device handle
 * @nb:      callback structure
 *
 * Register callback for RHI notification. When a RHI is triggered, the callback
 * function will be called.
 */
int switchtec_fabric_register_rhi_notify(struct dma_device *dma_dev,
					 struct notifier_block *nb);

/**
 * switchtec_fabric_unregister_rhi_notify - unregister callback for RHI
 * 					    notfication
 * @dma_dev:  DMA device handle
 * @nb:       callback structure
 *
 * Unregister callback for RHI notification.
 */
int switchtec_fabric_unregister_rhi_notify(struct dma_device *dma_dev,
					   struct notifier_block *nb);

/**
 * switchtec_fabric_register_buffer - register a DMA buffer
 * @dma_dev:   DMA device handle
 * @peer_hfid: peer host FID to which the buffer is registered
 * @buf_index: DMA buffer index, 0..5
 * @buf_addr:  DMA address of the buffer
 * @buf_size:  buffer size
 * @cookie:    cookie used to distinguish buffer when a RHI callback is called
 *
 * Register a DMA buffer for peer host to use, a cookie is passed out.
 */
int switchtec_fabric_register_buffer(struct dma_device *dma_dev, u16 peer_hfid,
				     u8 buf_index, u64 buf_addr, u64 buf_size,
				     int *cookie);

/**
 * switchtec_fabric_unregister_buffer - unregister a DMA buffer
 * @dma_dev:   DMA device handle
 * @peer_hfid: peer host FID to which the buffer is registered
 * @buf_index: DMA buffer index, 0..5
 * @cookie:    cookie used to distinguish buffer when a RHI callback is called
 *
 * Unregister a DMA buffer.
 */
int switchtec_fabric_unregister_buffer(struct dma_device *dma_dev,
				       u16 peer_hfid, u8 buf_index, int cookie);

/**
 * switchtec_fabric_get_peer_buffers - get peer registered buffers
 * @dma_dev:   DMA device handle
 * @peer_hfid: peer host FID from which the buffer is registered
 * @buf_num:   size of buffer array
 * @bufs:      array of buffer structure
 *
 * Get at most the specified number of buffers registered from specified host.
 */
int switchtec_fabric_get_peer_buffers(struct dma_device *dma_dev, u16 peer_hfid,
				      int buf_num,
				      struct switchtec_buffer *bufs);

/**
 * switchtec_fabric_get_buffer_number - get locally registered buffer count
 * @dma_dev: DMA device handle
 *
 * Get locally registered buffer count.
 */
int switchtec_fabric_get_buffer_number(struct dma_device *dma_dev);

/**
 * switchtec_fabric_get_buffers - get locally registered buffers
 * @dma_dev: DMA device handle
 * @buf_num: size of buffer array
 * @bufs:    array of buffer structure
 *
 * Get at most the specified number of buffers registered buffers locally.
 */
int switchtec_fabric_get_buffers(struct dma_device *dma_dev, int buf_num,
				 struct switchtec_buffer *bufs);

/**
 * switchtec_fabric_register_event_notify - register callback for the fabric
 * 					    event
 * @dma_dev: DMA device handle
 * @nb:      callback structure
 *
 * Register callback for the fabric event. When a fabric event is triggered,
 * the callback function will be called.
 */
int switchtec_fabric_register_event_notify(struct dma_device *dma_dev,
					   struct notifier_block *nb);

/**
 * switchtec_fabric_unregister_event_notify - Unregister callback for the fabric
 *					      event
 * @dma_dev: DMA device handle
 * @nb:      callback structure
 *
 * Unregister callback for the fabric event.
 */
int switchtec_fabric_unregister_event_notify(struct dma_device *dma_dev,
					     struct notifier_block *nb);

/**
 * switchtec_fabric_dma_prep_memcpy - prepare a fabric memcpy descriptor
 * @c:       DMA channel handle
 * @dst_fid: destination FID
 * @dma_dst: destination DMA address
 * @src_fid: source FID
 * @dma_src: source DMA address
 * @len:     size of data to copy
 * @flags:   flags, refer to note.
 *
 * Prepare a fabric memcpy descriptor.
 * Note: This function is very similar to Linux DMA framework API
 * dmaengine_prep_dma_memcpy, the flags argument definition is identical to
 * its counter part of dmaengine_prep_dma_memcpy.
 */
struct dma_async_tx_descriptor *switchtec_fabric_dma_prep_memcpy(
		struct dma_chan *c, u16 dst_dfid, dma_addr_t dma_dst,
		u16 src_dfid, dma_addr_t dma_src, size_t len,
		unsigned long flags);

/**
 * switchtec_fabric_dma_prep_rhi - prepare a fabric RHI descriptor
 * @c:          DMA channel handle
 * @peer_dfid:  peer RHI DMA FID
 * @rhi_index:  RHI index which can be got from the buffer structure
 * @local_dfid: local RHI DMA FID
 * @flags:      flags, refer to note.
 *
 * Prepare a fabric descriptor.
 * Note: This function is very similar to Linux DMA framework API
 * dmaengine_prep_dma_memcpy, the flags argument definition is identical to
 * its counter part of dmaengine_prep_dma_memcpy.
 */
struct dma_async_tx_descriptor *switchtec_fabric_dma_prep_rhi(
		struct dma_chan *c, u16 peer_dfid, u16 rhi_index,
		u16 local_dfid, unsigned long flags);

/**
 * switchtec_fabric_dma_prep_wimm_data - prepare a fabric wimm data descriptor
 * @c:          DMA channel handle
 * @peer_dfid:  destination DMA FID
 * @dma_dst:    destination DMA address
 * @local_dfid: source DMA FID
 * @data:       data to write
 * @flags:      flags, refer to note.
 *
 * Prepare a fabric RHI descriptor.
 * Note: This function is very similar to Linux DMA framework API
 * device_prep_dma_imm_data, the flags argument definition is identical to
 * its counter part of device_prep_dma_imm_data.
 */
struct dma_async_tx_descriptor *switchtec_fabric_dma_prep_wimm_data(
		struct dma_chan *c, u16 peer_dfid, dma_addr_t dma_dst,
		u16 local_dfid, u64 data, unsigned long flags);
#endif
