/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2020 Artur Pacholec
 * Copyright (c) 2020 Konrad Beckmann
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */


#ifndef BOARD_H_
#define BOARD_H_

#include "fsl_device_registers.h"

// required since iMX RT10xx SDK include this file for board size
#define BOARD_FLASH_SIZE (16 * 1024 * 1024)
#define BOARD_FLASH_BASE FlexSPI_AMBA_BASE

#define APP_START_ADDRESS 0x6000C000

#define UF2_FAMILY 0x4FB2D5BD

#define VENDOR_NAME "xil.se"
#define PRODUCT_NAME "Pergola UF2"
#define BOARD_ID "Pergola"
#define VOLUME_LABEL "PERGOLA"
#define INDEX_URL "https://github.com/pergola-fpga"

void board_flash_flush(void);
uint32_t board_flash_read_blocks(uint8_t *dest, uint32_t block, uint32_t num_blocks);
uint32_t board_flash_write_blocks(const uint8_t *src, uint32_t lba, uint32_t num_blocks);

void board_delay_ms(uint32_t ms);

#endif /* BOARD_H_ */
