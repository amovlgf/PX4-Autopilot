/****************************************************************************
 * boards/amovlab/amov-v6xrt/src/imxrt_flexspi_nor_flash.c
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

#include <px4_arch/imxrt_flexspi_nor_flash.h>

/* Micron MT25QU128ABA, 16 MB, 1.8 V. FlexSPI1: GPIO_SD_B2_06..11.
 * Start at 30 MHz, standard SPI, 24-bit addresses, no dummy cycles or DQS.
 * Sequence IDs follow the i.MX RT1176 ROM NOR driver API.
 * No chip erase sequence: updates are confined to the 4 MB firmware window.
 */
locate_data(".boot_hdr.conf")
const struct flexspi_nor_config_s g_flash_config = {
	.memConfig = {
		.tag = FLEXSPI_CFG_BLK_TAG,
		.version = FLEXSPI_CFG_BLK_VERSION,
		.readSampleClkSrc = kFlexSPIReadSampleClk_LoopbackInternally,
		.csHoldTime = 3,
		.csSetupTime = 3,
		.controllerMiscOption = (1u << kFlexSpiMiscOffset_SafeConfigFreqEnable),
		.deviceType = kFlexSpiDeviceType_SerialNOR,
		.sflashPadType = kSerialFlash_1Pad,
		.serialClkFreq = kFlexSpiSerialClk_30MHz,
		.sflashA1Size = 16u * 1024u * 1024u,
		.busyOffset = 0,
		.busyBitPolarity = 0,
		.lookupTable = {
			/* READ */
			[0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x03, RADDR_SDR, FLEXSPI_1PAD, 24),
			[1] = FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_1PAD, 4, STOP_EXE, FLEXSPI_1PAD, 0),
			/* READ STATUS */
			[4 * 1] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05, READ_SDR, FLEXSPI_1PAD, 4),
			/* WRITE ENABLE */
			[4 * 3] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06, STOP_EXE, FLEXSPI_1PAD, 0),
			/* ERASE 4 KB */
			[4 * 5] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x20, RADDR_SDR, FLEXSPI_1PAD, 24),
			/* ERASE 64 KB */
			[4 * 8] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xd8, RADDR_SDR, FLEXSPI_1PAD, 24),
			/* PAGE PROGRAM */
			[4 * 9] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x02, RADDR_SDR, FLEXSPI_1PAD, 24),
			[4 * 9 + 1] = FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 4, STOP_EXE, FLEXSPI_1PAD, 0),
		},
	},
	.pageSize = 256,
	.sectorSize = 4u * 1024u,
	.blockSize = 64u * 1024u,
	.isUniformBlockSize = false,
	.ipcmdSerialClkFreq = kFlexSpiSerialClk_30MHz,
};
