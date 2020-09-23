# Switchtec Fabric DMA Driver Client Examples

There are two Switchtec fabric DMA clients in this directory to help users
understand the proprietary fabric APIs of the Switchtec DMA driver. They work in
initiator and target mode, which are considered the most common use cases of
Switchtec fabric DMA. The initiator produces data and copies it from the local
host to the buffers of a peer host. The target on the peer host consumes the
data.

Note: this is only for reference. A real world client would be much more
complicated with error handling and protection, and could act as initiator and
target at the same time.

## Build

First build the Switchtec DMA driver by running `sudo make` in the root of this
repository, then run `./build.sh` in this directory.

## Initiator

The initiator and target use a buffer allocated by the target as a scratchpad
for sharing metadata. When the initiator transfers data to the target buffer,
it does the following:

1. get buffers from the target and copy data to them.
2. inform the target of the data pattern copied and the data size.

## Target

The target client allocates buffers for the initiator client to use. When the
initiator has finished copying data it will notify the target, which will check
the data against the pattern in the scratchpad.

## Helper scripts

There are many parameters that can be passed to the initiator and target modules
. For a full list, run `modifo initiator.ko` or `modinfo target.ko`. The scripts
"\_load_initiator.sh" and "\_load_target.sh" give examples of using some of
these client module parameters. To run these scripts, you will need to tune
some of the scripts' parameters and update parameter "need_tune" to 0, otherwise
you will be given a warning. (To avoid confusing git by touching the original
load scripts as you don't want to commit these changes, you can make a copy of
these load scripts for your own parameter tuning by removing the prefix "\_"
from the names of the copies of the scripts.)

To unload the initiator or target, simply run `sudo ./unload_initiator.sh` or
`sudo ./unload_target.sh`.
