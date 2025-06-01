/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

// DMAMUX1 Using at most 8 Channels on DMA1 --------   Assigned
//                                                     V

#define DMAMAP_SPI1_RX    DMAMAP_DMA12_SPI1RX_0     /* 1 DMA1:37 BMI055, ICM-42688-P, MS5611, ICM-42670-P */
#define DMAMAP_SPI1_TX    DMAMAP_DMA12_SPI1TX_0     /* 2 DMA1:38 BMI055, ICM-42688-P, MS5611, ICM-42670-P  */

#define DMAMAP_SPI4_RX    DMAMAP_DMA12_SPI4RX_0     /* 5 DMA1:83 FRAM */
#define DMAMAP_SPI4_TX    DMAMAP_DMA12_SPI4TX_0     /* 6 DMA1:84 FRAM */

//#define DMAMAP_UART4_RX  DMAMAP_DMA12_UART4RX_0     /* 5 DMA1:63 GPS1 */
//#define DMAMAP_UART4_TX  DMAMAP_DMA12_UART4TX_0     /* 6 DMA1:64 GPS1 */

#define DMAMAP_USART6_RX  DMAMAP_DMA12_USART6RX_0   /* 7 DMA1:71 PX4IO */
#define DMAMAP_USART6_TX  DMAMAP_DMA12_USART6TX_0   /* 8 DMA1:72 PX4IO */

// DMAMUX2 Using at most 8 Channels on DMA2 --------   Assigned
//                                                     V

#define DMAMAP_USART2_RX  DMAMAP_DMA12_USART2RX_1   /* 1 DMA1:43 Telem1 */
#define DMAMAP_USART2_TX  DMAMAP_DMA12_USART2TX_1   /* 2 DMA1:44 Telem1 */

#define DMAMAP_USART3_RX  DMAMAP_DMA12_USART3RX_1   /* 3 DMA2:45 Telem2 */
#define DMAMAP_USART3_TX  DMAMAP_DMA12_USART3TX_1   /* 4 DMA2:46 Telem2 */

#define DMAMAP_UART8_RX   DMAMAP_DMA12_UART8RX_1    /* 7 DMA1:81  DEBUG */
#define DMAMAP_UART8_TX   DMAMAP_DMA12_UART8TX_1    /* 8 DMA1:82  DEBUG */
