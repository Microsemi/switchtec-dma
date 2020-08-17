# Switchtec Fabric DMA Driver Client Examples

## Summary

There are two Switchtec fabric DMA clients in this directory for people to
understand the proprietary fabric APIs of Switchtec DMA driver. They work in
the initiator and target mode, which is considered the most common use case of
Switchtec fabric DMA. The initiator produces data and copy them from local host
to the buffers of a peer host. The target on the peer host consumes the data in
one way or another.

Note: this is only for reference. A real world client would be much complicated
with more error handlings and protections, and could act as initiator and target
at the same time.

## Build

Build Switchtec DMA driver first by running `sudo make` in the root of this
repository, then run `./build.sh` in this directory.

## Initiator

The initiator and target use a buffer allocated by the target as scratchpad
for metadata sharing. When initiator transfers data to target buffer, it does:

1. get buffers from target, and copy data to the buffers.
2. informs target what is the data pattern of copied, and the data size.

## Target

The target client allocate buffers for initiator client use, when the data
copies from initiator complete, the target will get notified and will check the
data against the pattern from scratchpad.

## Helper scripts

There are many paramters the initiator and target module takes, run `modifo
initiator.ko` or `modinfo target.ko` for a full list. The script
"\_load_initiator.sh" and "\_load_target.sh" give examples of using use some of
these client modules parameters. To run these scripts, you will need to tune
some of the scripts parameters and update parameter "need_tune" to 0, otherwise
you will be given a warning. (To avoid confusing git by touching the original
load scripts as you don't want to commit these changes, you can make a copy of
these load scripts for your own parameter tuning. And just remove the prefix "\_"
from the names of copies of the scripts.)

To unload the initiator or target, simply run `sudo ./unload_initiator.sh` or
`sudo ./unload_target.sh`
