#!/bin/bash
#
# This is an example script for loading initiator, please update before running
# on your platform.
#

#
# Update the value to 0 to run
#
need_tune=1

# Specify DMA device and channel to use
dma_device=dma16
dma_channel=chan0

# Specify either peer_pax_id and peer_port_id, or peer_hfid
peer_pax_id=0x1
peer_port_id=24
peer_hfid=0x0a01

# Specify the timeout value to wait for peer buffer registration
timeout=5

if [ $need_tune -ne 0 ]; then
	echo
	echo "Please tune script parameters before running on your platform!"
	echo
	exit 1
fi

sudo insmod ../switchtec_dma.ko dyndbg=+p
if [ -n "$peer_hfid" ]; then
	sudo insmod initiator.ko dma_dev=$dma_device dma_chan=$dma_channel peer_hfid=$peer_hfid timeout=$timeout
else
	sudo insmod initiator.ko dma_dev=$dma_device dma_chan=$dma_channel peer_pax_id=$peer_pax_id peer_pid=$peer_port_id timeout=$timeout
fi
