/****************************************************************************
 * boards/amovlab/amov-v6xrt/src/imxrt_flexspi_nor_boot.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>

/* RT1176 ROM IVT header: tag 0xd1, big-endian length 0x0020, HAB 4.1. */
#define IVT_HEADER 0x412000d1u
#define IVT_RSVD 0u
#define DCD_ADDRESS 0u
#define CSF_ADDRESS 0u
#define PLUGIN_FLAG 0u

/* Absolute linker symbols keep the ROM header and both link scripts aligned. */
extern const uint8_t _boot_loadaddr;
extern const uint8_t _boot_size;
extern const void *const _vectors[];

#define IMAGE_DEST ((uint32_t)&_boot_loadaddr)
#define IMAGE_SIZE ((uint32_t)&_boot_size)
#define IMAGE_ENTRY_ADDRESS ((uint32_t)&_vectors)
#define BOOT_DATA_ADDRESS ((uint32_t)&g_boot_data)
#define IMAG_VECTOR_TABLE ((uint32_t)&g_image_vector_table)

struct ivt_s {
	uint32_t hdr;
	uint32_t entry;
	uint32_t reserved1;
	uint32_t dcd;
	uint32_t boot_data;
	uint32_t self;
	uint32_t csf;
	uint32_t reserved2;
};

struct boot_data_s {
	uint32_t start;
	uint32_t size;
	uint32_t plugin;
	uint32_t placeholder;
};

extern const struct boot_data_s g_boot_data;
