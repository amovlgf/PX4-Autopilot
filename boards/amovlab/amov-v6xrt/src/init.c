/****************************************************************************
 *
 *   Copyright (c) 2018-2019, 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "board_config.h"

#include <string.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <px4_arch/imxrt_flexspi_nor_flash.h>
#include <px4_arch/imxrt_romapi.h>
#include <px4_platform_common/init.h>
#include <px4_platform_common/px4_manifest.h>
#include <px4_platform/board_dma_alloc.h>
#include <drivers/drv_board_led.h>

extern const struct flexspi_nor_config_s g_flash_config;
struct flexspi_nor_config_s g_bootConfig;

/* The boot ROM and both images use the same single-pad SDR configuration.
 * Install the complete programming LUT from RAM without changing flash mode.
 */
#ifdef BOOTLOADER
locate_code(".ramfunc")
static void noinline_function board_flash_initialize(void)
{
	ROM_API_Init();
	memcpy(&g_bootConfig, &g_flash_config, sizeof(g_bootConfig));
	irqstate_t flags = up_irq_save();
	ROM_FLEXSPI_NorFlash_Init(1, &g_bootConfig);
	ROM_FLEXSPI_NorFlash_ClearCache(1);
	up_irq_restore(flags);
}
#endif

void imxrt_boardinitialize(void)
{
#ifdef BOOTLOADER
	board_flash_initialize();
#endif
	imxrt_config_gpio(GPIO_nLED_BLUE);
	imxrt_usb_initialize();
}

const px4_mft_s *board_get_manifest(void)
{
	return NULL;
}

int board_app_initialize(uintptr_t arg)
{
#ifndef BOOTLOADER
	int ret = px4_platform_init();

	if (ret != OK) {
		return ret;
	}

	ret = board_dma_alloc_init();

	if (ret != OK) {
		return ret;
	}

	drv_led_start();
#endif
	return OK;
}
