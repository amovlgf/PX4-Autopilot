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

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <imxrt_gpio.h>
#include <imxrt_iomuxc.h>
#include <arch/board/board.h>

/* R13: GPIO_AD_02 / GPIO3_IO01, active low. */
#define GPIO_nLED_BLUE (GPIO_PORT3 | GPIO_PIN1 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE)
#define HRT_TIMER 5
#define HRT_TIMER_CHANNEL 2
#define BOARD_DMA_ALLOC_POOL_SIZE 5120
#define BOARD_HAS_ISP_BOOTLOADER 1
#define BOARD_ENABLE_CONSOLE_BUFFER
#define DIRECT_PWM_OUTPUT_CHANNELS 0
#define BOARD_HAS_NO_CAPTURE

__BEGIN_DECLS
int imxrt_usb_initialize(void);
int board_read_VBUS_state(void);
#include <px4_platform_common/board_common.h>
__END_DECLS
