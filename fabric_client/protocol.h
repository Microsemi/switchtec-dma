// SPDX-License-Identifier: GPL-2.0
/*
 * Microchip Switchtec(tm) DMA Controller Driver
 * Copyright (c) 2020, Microchip Corporation
 */

#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#define SCRATCHPAD_BUFFER_INDEX 0
#define DATA_BUFFER_INDEX       1

struct scratchpad {
	u64 copied_size;
	u64 pattern;
};
#endif
