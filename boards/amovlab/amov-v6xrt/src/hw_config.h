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

#define USB0_DEV 0x01
#define SERIAL0_DEV 0x02
#define SERIAL1_DEV 0x04
#define APP_LOAD_ADDRESS 0x30020000
#define APP_VECTOR_OFFSET 0x2000
#define BOOTLOADER_DELAY 5000
#define INTERFACE_USB 1
#define INTERFACE_USB_CONFIG "/dev/ttyACM0"
#define INTERFACE_USART 0
/* Local development ID; not an upstream allocation. */
#define BOARD_TYPE 65001
/* The common loader tests <=: this is the LAST 4 KB sector index. */
#define BOARD_FLASH_SECTORS 1023
#define BOARD_FIRST_FLASH_SECTOR_TO_ERASE 32
#define BOARD_FLASH_SIZE (4 * 1024 * 1024)
#define APP_RESERVATION_SIZE 0
#define OSC_FREQ 24
#define BOARD_PIN_LED_BOOTLOADER GPIO_nLED_BLUE
#define BOARD_LED_ON 0
#define BOARD_LED_OFF 1
#define SERIAL_BREAK_DETECT_DISABLED 1
#define ARCH_SN_MAX_LENGTH 12
#define USB_DATA_ALIGN
#define BOOT_DEVICES_SELECTION USB0_DEV
#define BOOT_DEVICES_FILTER_ONUSB USB0_DEV
