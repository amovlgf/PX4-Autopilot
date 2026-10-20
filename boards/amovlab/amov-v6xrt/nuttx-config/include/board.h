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

#include <nuttx/config.h>

#define BOARD_CPU_FREQUENCY 996000000
#define IMXRT_IPG_PODF_DIVIDER 5
#define BOARD_GPT_FREQUENCY 24000000
#define BOARD_XTAL_FREQUENCY 24000000

/* Baseboard DEBUG connector, not the vendor demo's UART12. */
#define GPIO_LPUART1_RX (GPIO_LPUART1_RX_3 | IOMUX_UART_DEFAULT | PADMUX_SION) /* GPIO_DISP_B2_09 */
#define GPIO_LPUART1_TX (GPIO_LPUART1_TX_3 | IOMUX_PULL_NONE | IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_FAST) /* GPIO_DISP_B2_08 */

#define BOARD_NLEDS 1
#define BOARD_LED_BLUE 0
