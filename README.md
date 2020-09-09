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
engine. The Switchtec DMA driver adds Submission Elements (SE) to the SQ on
behalf of the DMA client when a DMA transfer is kicked off. The DMA engine adds
Completion Elements (CE) to the CQ when a DMA transfer has finished.

## DMA mode

The Switchtec DMA driver can work in two modes: basic mode and fabric mode. With
Switchtec PSX/PFX devices, the Switchtec DMA driver works in basic mode. With
Switchtec PAX devices, the Switchtec DMA driver works in fabric mode. The
Switchtec DMA driver can work in basic mode and fabric mode at the same time if
both PSX/PFX and PAX devices are present in the system.

### Basic mode

In basic mode, Switchtec DMA enables data movement within a local host.

This mode is automatically turned on with Switchtec PSX/PFX DMA endpoint
devices when they're probed and initialized by the driver. In this mode, the
Switchtec DMA driver enables the following DMA transactions:

- Memory copy within the local host
- Write Immediate within the local host

#### DMA client interface

In basic mode, the Switchtec DMA driver works as a DMA controller driver
(provider) of the Linux DMAEngine framework. For more detail, refer to
Documentation/driver-api/dmaengine/provider.rst in the Linux kernel source tree.

### Fabric mode

In fabric mode, the Switchtec DMA driver enables data movement from one host to
another within the same Switchtec fabric.

This mode is automatically turned on with Switchtec PAX DMA endpoint devices
when they're probed and initialized by the driver. In this mode, the Switchtec
DMA driver enables the following DMA transactions:

- Memory copy from the local host to a remote host
- Write Immediate to a remote host
- Interrupt a remote host (Remote Host Interrupt or RHI)

#### Fabric DMA client interface

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

Several sysfs files are provided for channel configuration. They are created
under /sys/class/dma/dmaXchanY/config/ where X and Y are the DMA device number
and channel number. Below is a table describing these files. For the use of
these files, please contact the Microchip applications team.

File name | Read/Write | Default value | Min value | Max value | Description
:---      |:---        |:---           |:---       |:---       |:---
arb_weight | R/W | 1 | 0 | 2^8 - 1 | This value is used to initialize the local copy of the channel weight in the WRR Arbiter.
burst_scale | R/W | 1 | 0 | 2 | The following three files are used together to determine max payload/sec for a channel:<br />1. burst_scale<br />2. burst_size<br />3. interval
burst_size | R/W | 6 | 0 | 7 | The following three files are used together to determine max payload/sec for a channel:<br />1. burst_scale <br />2. burst_size<br />3. interval
linterval | R/W | 0 | 0 | 7 | The following three files are used together to determine max payload/sec for a channel:<br />1. burst_scale <br />2. burst_size<br />3. interval
mrrs | R/W | 3| 0 | 5 | Maximum Read Request size for this channel. Defined encodings for this field are:<br />0: 128 bytes MRRS <br />1: 256 bytes MRRS<br />2: 512 bytes MRRS<br />3: 1024 bytes MRRS<br />4: 2048 bytes MRRS<br />5: 4096 bytes MRRS

## Performance monitoring

Several other sysfs files are provided for performance monitoring. They are
created under /sys/class/dma/dmaXchanY/pmon/ where X and Y are the DMA device
number and channel number. Below is a table describing these files.

File name | Read/Write | Description
:---      |:---        |:---
latency_selector | R/W | To select a latency type to measure the latency_min, latency_max and latency_last values. Valid types are:<br />1: SE Fetch latency<br />2: VDM latency<br />3: Read Immediate latency<br />4: SE Processing latency
latency_min | R/O | Minimum turnaround time for this channel since reset or the last time the register read measured in nanoseconds.
latency_max | R/O | Maximum turnaround time for this channel since reset or the last time the register read measured in nanoseconds.
latency_last | R/O | The last turnaround time for this channel measured in nanoseconds.
se_count | R/O | Number of SEs processed by this channel since reset or the last time the register read. This value will eventually saturate if it is not read.
se_buf_empty | R/O | Number of empty SE buffers available to this channel.
se_pending | R/O | Number of SEs waiting to be fetched from the SQ.
byte_count | R/O | Number of bytes sent by DMA transfers on this channel since reset or the last time the register read. This value will eventually saturate if it is not read.
chan_idle | R/O | Number of cycles a channel is idle. This is counted from the time the channel SE buffers go empty, while there are no new SEs waiting on the SQ, until the next time the SQ tail is updated. The cycle count continues to increment until the register is read at, at which point it clears and the count starts again. This value will eventually saturate if it is not read.
