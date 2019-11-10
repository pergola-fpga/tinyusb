/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2019 Artur Pacholec
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

#include "bsp/board.h"

#include "fsl_iomuxc.h"
#include "clock_config.h"
#include "fsl_lpuart.h"
#include "fsl_gpio.h"

#define LED_MUX IOMUXC_GPIO_SD_05_GPIO2_IO05
#define LED_GPIO GPIO1
#define LED_PIN 11

#define BUTTON_MUX IOMUXC_GPIO_11_GPIOMUX_IO11
#define BUTTON_GPIO GPIO2
#define BUTTON_PIN 5

#define UART_RX_MUX IOMUXC_GPIO_09_LPUART1_RXD
#define UART_TX_MUX IOMUXC_GPIO_10_LPUART1_TXD
#define UART_BANK LPUART1

void board_init(void)
{
  BOARD_BootClockRUN();

  CLOCK_EnableClock(kCLOCK_Iomuxc);

  // Config LED GPIO
  IOMUXC_SetPinMux(LED_MUX, 0U);
  IOMUXC_SetPinConfig(LED_MUX, 0x01B0A0U);

  gpio_pin_config_t led_config = { kGPIO_DigitalOutput, 0, kGPIO_NoIntmode };
  GPIO_PinInit(LED_GPIO, LED_PIN, &led_config);

  // Config button GPIO
  IOMUXC_SetPinMux(BUTTON_MUX, 0U);
  IOMUXC_SetPinConfig(BUTTON_MUX, 0x70A0U);

  gpio_pin_config_t sw_config = { kGPIO_DigitalInput, 0, kGPIO_NoIntmode };
  GPIO_PinInit(BUTTON_GPIO, BUTTON_PIN, &sw_config);

  // Config UART GPIO and peripheral
  IOMUXC_SetPinMux(UART_RX_MUX, 0U);
  IOMUXC_SetPinConfig(UART_RX_MUX, 0x10B0U);

  IOMUXC_SetPinMux(UART_TX_MUX, 0U);
  IOMUXC_SetPinConfig(UART_TX_MUX, 0x10A0U);

  lpuart_config_t config;
  LPUART_GetDefaultConfig(&config);
  config.baudRate_Bps = 115200;
  config.enableTx = true;
  config.enableRx = true;
  LPUART_Init(UART_BANK, &config, (CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 6U) / (CLOCK_GetDiv(kCLOCK_UartDiv) + 1U));

#if CFG_TUSB_OS  == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);
#endif

  CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
  CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000U);
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  GPIO_PinWrite(LED_GPIO, LED_PIN, state);
}

uint32_t board_button_read(void)
{
  return GPIO_PinRead(BUTTON_GPIO, BUTTON_PIN) == 0;
}

int board_uart_read(uint8_t* buf, int len)
{
  LPUART_ReadBlocking(UART_BANK, buf, len);

  return len;
}

int board_uart_write(void const * buf, int len)
{
  LPUART_WriteBlocking(UART_BANK, (uint8_t*)buf, len);

  return len;
}

#if CFG_TUSB_OS  == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

void SysTick_Handler (void)
{
  system_ticks++;
}

uint32_t board_millis(void)
{
  return system_ticks;
}
#endif
