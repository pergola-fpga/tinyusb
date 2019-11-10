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

#include "tusb_option.h"

#if TUSB_OPT_DEVICE_ENABLED && (CFG_TUSB_MCU == OPT_MCU_RT10XX)

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+
#include "common/tusb_common.h"
#include "device/dcd.h"
#include "dcd_rt10xx.h"
#include "fsl_common.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

#define QHD_MAX          16
#define QTD_NEXT_INVALID 0x01

typedef struct {
  // Must be at 2K alignment
  dcd_qhd_t qhd[QHD_MAX] TU_ATTR_ALIGNED(64);
  dcd_qtd_t qtd[QHD_MAX] TU_ATTR_ALIGNED(32);
}dcd_data_t;

#if (CFG_TUSB_RHPORT0_MODE & OPT_MODE_DEVICE)
CFG_TUSB_MEM_SECTION TU_ATTR_ALIGNED(2048) static dcd_data_t dcd_data0;
#endif

#if (CFG_TUSB_RHPORT1_MODE & OPT_MODE_DEVICE)
CFG_TUSB_MEM_SECTION TU_ATTR_ALIGNED(2048) static dcd_data_t dcd_data1;
#endif

static USB_Type * const RT_USB[2] = { USB, NULL };

static dcd_data_t* const dcd_data_ptr[2] =
{
#if (CFG_TUSB_RHPORT0_MODE & OPT_MODE_DEVICE)
  &dcd_data0,
#else
  NULL,
#endif

#if (CFG_TUSB_RHPORT1_MODE & OPT_MODE_DEVICE)
  &dcd_data1
#else
  NULL
#endif
};

//--------------------------------------------------------------------+
// CONTROLLER API
//--------------------------------------------------------------------+

/// follows LPC43xx User Manual 23.10.3
static void bus_reset(uint8_t rhport)
{
  USB_Type* rt_usb = RT_USB[rhport];

  // The reset value for all endpoint types is the control endpoint. If one endpoint
  // direction is enabled and the paired endpoint of opposite direction is disabled, then the
  // endpoint type of the unused direction must bechanged from the control type to any other
  // type (e.g. bulk). Leaving an unconfigured endpoint control will cause undefined behavior
  // for the data PID tracking on the active endpoint.

  for( int i=0; i < 7; i++)
  {
    rt_usb->ENDPTCTRL[i] = (TUSB_XFER_BULK << 2) | (TUSB_XFER_BULK << 18);
  }

  //------------- Clear All Registers -------------//
  rt_usb->ENDPTNAK       = rt_usb->ENDPTNAK;
  rt_usb->ENDPTNAKEN     = 0;
  rt_usb->USBSTS         = rt_usb->USBSTS;
  rt_usb->ENDPTSETUPSTAT = rt_usb->ENDPTSETUPSTAT;
  rt_usb->ENDPTCOMPLETE  = rt_usb->ENDPTCOMPLETE;

  while (rt_usb->ENDPTPRIME);
  rt_usb->ENDPTFLUSH = 0xFFFFFFFF;
  while (rt_usb->ENDPTFLUSH);

  // read reset bit in portsc

  //------------- Queue Head & Queue TD -------------//
  dcd_data_t* p_dcd = dcd_data_ptr[rhport];
  tu_memclr(p_dcd, sizeof(dcd_data_t));

  //------------- Set up Control Endpoints (0 OUT, 1 IN) -------------//
  p_dcd->qhd[0].zero_length_termination = p_dcd->qhd[1].zero_length_termination = 1;
  p_dcd->qhd[0].max_package_size = p_dcd->qhd[1].max_package_size = CFG_TUD_ENDPOINT0_SIZE;
  p_dcd->qhd[0].qtd_overlay.next = p_dcd->qhd[1].qtd_overlay.next = QTD_NEXT_INVALID;

  p_dcd->qhd[0].int_on_setup = 1; // OUT only
}

void dcd_init(uint8_t rhport)
{
  USB_Type* const rt_usb = RT_USB[rhport];
  dcd_data_t* p_dcd = dcd_data_ptr[rhport];

  tu_memclr(p_dcd, sizeof(dcd_data_t));

  rt_usb->ENDPTLISTADDR = (uint32_t) p_dcd->qhd; // Endpoint List Address has to be 2K alignment
  rt_usb->USBSTS  = rt_usb->USBSTS;
  rt_usb->USBINTR = INT_MASK_USB | INT_MASK_ERROR | INT_MASK_PORT_CHANGE | INT_MASK_RESET | INT_MASK_SUSPEND | INT_MASK_SOF;

  rt_usb->USBMODE &= ~USBHS_USBMODE_CM_MASK;
  rt_usb->USBMODE |= USBHS_USBMODE_CM(0x02U);

  rt_usb->USBCMD &= ~0x00FF0000; // Interrupt Threshold Interval = 0
  rt_usb->USBCMD |= TU_BIT(0); // connect
}

void dcd_int_enable(uint8_t rhport)
{
  NVIC_EnableIRQ(USB_OTG1_IRQn);
}

void dcd_int_disable(uint8_t rhport)
{
  NVIC_DisableIRQ(USB_OTG1_IRQn);
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
  // Response with status first before changing device address
  dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);

  RT_USB[rhport]->DEVICEADDR = (dev_addr << 25) | TU_BIT(24);
}

void dcd_set_config(uint8_t rhport, uint8_t config_num)
{
  (void) rhport;
  (void) config_num;
  // nothing to do
}

void dcd_remote_wakeup(uint8_t rhport)
{
  (void) rhport;
}

//--------------------------------------------------------------------+
// HELPER
//--------------------------------------------------------------------+
// index to bit position in register
static inline uint8_t ep_idx2bit(uint8_t ep_idx)
{
  return ep_idx/2 + ( (ep_idx%2) ? 16 : 0);
}

static void qtd_init(dcd_qtd_t* p_qtd, void * data_ptr, uint16_t total_bytes)
{
  tu_memclr(p_qtd, sizeof(dcd_qtd_t));

  p_qtd->next        = QTD_NEXT_INVALID;
  p_qtd->active      = 1;
  p_qtd->total_bytes = p_qtd->expected_bytes = total_bytes;

  if (data_ptr != NULL)
  {
    p_qtd->buffer[0]   = (uint32_t) data_ptr;
    for(uint8_t i=1; i<5; i++)
    {
      p_qtd->buffer[i] |= tu_align4k( p_qtd->buffer[i-1] ) + 4096;
    }
  }
}

//--------------------------------------------------------------------+
// DCD Endpoint Port
//--------------------------------------------------------------------+
void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  uint8_t const epnum  = tu_edpt_number(ep_addr);
  uint8_t const dir    = tu_edpt_dir(ep_addr);

  if (epnum == 0) {
    RT_USB[rhport]->ENDPTCTRL0 |= ENDPTCTRL_MASK_STALL << (dir ? 16 : 0);
  } else {
    RT_USB[rhport]->ENDPTCTRL[epnum-1] |= ENDPTCTRL_MASK_STALL << (dir ? 16 : 0);
  }
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  uint8_t const epnum  = tu_edpt_number(ep_addr);
  uint8_t const dir    = tu_edpt_dir(ep_addr);

  // data toggle also need to be reset
  if (epnum == 0) {
    RT_USB[rhport]->ENDPTCTRL0 &= ~(ENDPTCTRL_MASK_STALL << ( dir  ? 16 : 0));
  } else {
    RT_USB[rhport]->ENDPTCTRL[epnum-1] &= ~(ENDPTCTRL_MASK_STALL << ( dir  ? 16 : 0));
    RT_USB[rhport]->ENDPTCTRL[epnum-1] |= ENDPTCTRL_MASK_TOGGLE_RESET << ( dir ? 16 : 0 );
  }
}

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * p_endpoint_desc)
{
  // TODO not support ISO yet
  TU_VERIFY ( p_endpoint_desc->bmAttributes.xfer != TUSB_XFER_ISOCHRONOUS);

  uint8_t const epnum  = tu_edpt_number(p_endpoint_desc->bEndpointAddress);
  uint8_t const dir    = tu_edpt_dir(p_endpoint_desc->bEndpointAddress);
  uint8_t const ep_idx = 2*epnum + dir;

  // USB0 has 5, USB1 has 3 non-control endpoints
  TU_ASSERT( epnum <= 7 );

  //------------- Prepare Queue Head -------------//
  dcd_qhd_t * p_qhd = &dcd_data_ptr[rhport]->qhd[ep_idx];
  tu_memclr(p_qhd, sizeof(dcd_qhd_t));

  p_qhd->zero_length_termination = 1;
  p_qhd->max_package_size        = p_endpoint_desc->wMaxPacketSize.size;
  p_qhd->qtd_overlay.next        = QTD_NEXT_INVALID;

  // Enable EP Control
  if (epnum == 0) {
    RT_USB[rhport]->ENDPTCTRL0 |= ((p_endpoint_desc->bmAttributes.xfer << 2) | ENDPTCTRL_MASK_ENABLE | ENDPTCTRL_MASK_TOGGLE_RESET) << (dir ? 16 : 0);
  } else {
    RT_USB[rhport]->ENDPTCTRL[epnum-1] |= ((p_endpoint_desc->bmAttributes.xfer << 2) | ENDPTCTRL_MASK_ENABLE | ENDPTCTRL_MASK_TOGGLE_RESET) << (dir ? 16 : 0);
  }
  return true;
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir   = tu_edpt_dir(ep_addr);
  uint8_t const ep_idx = 2*epnum + dir;

  if ( epnum == 0 )
  {
    // follows UM 24.10.8.1.1 Setup packet handling using setup lockout mechanism
    // wait until ENDPTSETUPSTAT before priming data/status in response TODO add time out
    while(RT_USB[rhport]->ENDPTSETUPSTAT & TU_BIT(0)) {}
  }

  dcd_qhd_t * p_qhd = &dcd_data_ptr[rhport]->qhd[ep_idx];
  dcd_qtd_t * p_qtd = &dcd_data_ptr[rhport]->qtd[ep_idx];

  //------------- Prepare qtd -------------//
  qtd_init(p_qtd, buffer, total_bytes);
  p_qtd->int_on_complete = true;
  p_qhd->qtd_overlay.next = (uint32_t) p_qtd; // link qtd to qhd

  // start transfer
  RT_USB[rhport]->ENDPTPRIME = TU_BIT( ep_idx2bit(ep_idx) ) ;

  return true;
}

//--------------------------------------------------------------------+
// ISR
//--------------------------------------------------------------------+
void hal_dcd_isr(uint8_t rhport)
{
  USB_Type* const rt_usb = RT_USB[rhport];

  uint32_t const int_enable = rt_usb->USBINTR;
  uint32_t const int_status = rt_usb->USBSTS & int_enable;
  rt_usb->USBSTS = int_status; // Acknowledge handled interrupt

  if (int_status == 0) return;// disabled interrupt sources


  if (int_status & INT_MASK_RESET)
  {
    bus_reset(rhport);
    dcd_event_bus_signal(rhport, DCD_EVENT_BUS_RESET, true);
  }

  if (int_status & INT_MASK_SUSPEND)
  {
    if (rt_usb->PORTSC1 & PORTSC_SUSPEND_MASK)
    {
      // Note: Host may delay more than 3 ms before and/or after bus reset before doing enumeration.
      if ((rt_usb->DEVICEADDR >> 25) & 0x0f)
      {
        dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);
      }
    }
  }

  // TODO disconnection does not generate interrupt !!!!!!
//	if (int_status & INT_MASK_PORT_CHANGE)
//	{
//	  if ( !(rt_usb->PORTSC1 & PORTSC_CURRENT_CONNECT_STATUS_MASK) )
//	  {
//      dcd_event_t event = { .rhport = rhport, .event_id = DCD_EVENT_UNPLUGGED };
//      dcd_event_handler(&event, true);
//	  }
//	}

  if (int_status & INT_MASK_USB)
  {
    uint32_t const edpt_complete = rt_usb->ENDPTCOMPLETE;
    rt_usb->ENDPTCOMPLETE = edpt_complete; // acknowledge

    dcd_data_t* const p_dcd = dcd_data_ptr[rhport];

    if (rt_usb->ENDPTSETUPSTAT)
    {
      //------------- Set up Received -------------//
      // 23.10.10.2 Operational model for setup transfers
      rt_usb->ENDPTSETUPSTAT = rt_usb->ENDPTSETUPSTAT;// acknowledge

      dcd_event_setup_received(rhport, (uint8_t*) &p_dcd->qhd[0].setup_request, true);
    }

    if ( edpt_complete )
    {
      for(uint8_t ep_idx = 0; ep_idx < QHD_MAX; ep_idx++)
      {
        if ( tu_bit_test(edpt_complete, ep_idx2bit(ep_idx)) )
        {
          // 23.10.12.3 Failed QTD also get ENDPTCOMPLETE set
          dcd_qtd_t * p_qtd = &dcd_data_ptr[rhport]->qtd[ep_idx];

          uint8_t result = p_qtd->halted  ? XFER_RESULT_STALLED :
              ( p_qtd->xact_err ||p_qtd->buffer_err ) ? XFER_RESULT_FAILED : XFER_RESULT_SUCCESS;

          uint8_t const ep_addr = (ep_idx/2) | ( (ep_idx & 0x01) ? TUSB_DIR_IN_MASK : 0 );
          dcd_event_xfer_complete(rhport, ep_addr, p_qtd->expected_bytes - p_qtd->total_bytes, result, true); // only number of bytes in the IOC qtd
        }
      }
    }
  }

  if (int_status & INT_MASK_SOF)
  {
    dcd_event_bus_signal(rhport, DCD_EVENT_SOF, true);
  }

  if (int_status & INT_MASK_NAK) {}
  if (int_status & INT_MASK_ERROR) TU_ASSERT(false, );
}

#endif
