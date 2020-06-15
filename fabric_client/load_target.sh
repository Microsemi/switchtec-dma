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

# Specify either peer_pax_id and peer_port_id, or peer_hfid
peer_hfid=0x0a02

if [ $need_tune -ne 0 ]; then
	echo 
	echo "Please tune script parameters before running on your platform!"
	echo 
	exit 1
fi

sudo insmod ../switchtec_dma.ko dyndbg=+p
sudo insmod target.ko dma_dev=$dma_device peer_hfid=$peer_hfid
