# switchtec-dma

Switchtec Gen4 PCIe switch introduces a new hardware DMA engine to offload
expensive memory operations, such as large copies, from host processors. A
complete solution of the Switchtec DMA involves not only the hardware DMA
engine, but also embedded firmware and host software.

The Switchtec DMA kernel module exports the access to the Switchtec DMA engine
to the upper layer host software (DMA client).

## Interface

When configured, the Switchtec devices advertise a special DMA endpoint which
enables the DMA functionality. BAR0 of the endpoint exports the firmware and
hardware registers for the driver to configure and control the DMA engine.

The switchtec dma driver maintains two queues in host memory space, a submission
queue (SQ) and a completion queue (CQ), to communicate with the hardware DMA
engine. The Switchtec DMA driver adds Submission Elements to the SQ on behalf of
the DMA client when a DMA transfer is kicked off. The DMA engine adds Completion
Elements to the CQ when a DMA transfer has finished.

## DMA mode

The Switchtec DMA driver can work in two modes: basic mode and fabric mode. With
Switchtec PSX/PFX devices, the Switchtec DMA driver works in basic mode. With
Switchtec PAX devices, the Switchtec DMA driver works in fabric mode. The
Switchtec DMA driver can work in basic mode and fabric mode at the same time if
both PSX/PFX and PAX devices are present in the system.

## Basic mode

In basic mode, Switchtec DMA enables data movement within a local host.

This mode is automatically turned on with Switchtec PSX/PFX DMA endpoint
devices when they're probed. With this mode on, the Switchtec DMA driver enables
the following DMA transactions:

- Memory copy within the local host
- Write Immediate within the local host

### DMA client interface

In basic mode, the Switchtec DMA driver works as a DMA controller driver
(provider) of the Linux DMAEngine framework. For more detail, refer to
Documentation/driver-api/dmaengine/provider.rst in the Linux kernel source tree.

## Fabric mode

In fabric mode, the Switchtec DMA driver enables data movement from one host to
another within a Switchtec fabric.

This mode is automatically turned on with Switchtec PAX DMA endpoint devices
when they're probed. With this mode on, the Switchtec DMA driver enables the
following DMA transactions:

- Memory copy from the local host to a remote host
- Write Immediate to a remote host
- Remote Host Interrupt to a remote host

### DMA client interface

In fabric mode, the Switchtec DMA driver extends the provider interface of the
Linux DMAEngine framework, which means all of the current provider interfaces
can still be used. For example, a fabric DMA client can use
dma_request_channel() to get a DMA channel, dmaengine_submit() to submit a
prepared descriptor, then dma_async_issue_pending() to kick off the transfer.

The extended fabric interface enables Switchtec fabric specific functionality
which includes:

- fabric client pairing
- buffer management
- fabric operation descriptor preparation
- fabric event and notification

The fabric DMA clients work in pairs: one acts as target, and the other acts as
initiator. Typically, a target allocates buffers for an initiator, and the
initiator writes data to the buffers for the target to consume. A real world
client could act as initiator and target at the same time.

Fabric client examples are provided in the fabric_client folder.

## Channel configuration

Several sysfs files are provided for channel configuration. These files are
created under /sys/class/dma/dmaXchanY/config/. For the meaning and the use of
these files, please contact the Microchip applications team.

## Performance monitoring

Several other sysfs files are provided for performance monitoring. These files
are created under /sys/class/dma/dmaXchanY/pmon/. For the meaning and the use of
these files, please contact the Microchip applications team.
