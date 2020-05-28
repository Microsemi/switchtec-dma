// SPDX-License-Identifier: GPL-2.0
/*
 * Microchip Switchtec(tm) DMA Controller Driver
 * Copyright (c) 2020, Microchip Corporation
 */

#ifndef _SWITCHTEC_DMA_H
#define _SWITCHTEC_DMA_H

#include <linux/dmaengine.h>

int switchtec_fabric_get_pax_count(struct dma_device *dma_dev);

#endif
