#ifndef __THREAD_USBD_H__
#define __THREAD_USBD_H__

#include "tx_api.h"

// 事件组
#define USBD_EVENT_FLAG_IRQ (1U << 0)         // USB中断
#define USBD_EVENT_FLAG_WINUSB_OUT (1U << 1)  //
#define USBD_EVENT_FLAG_WINUSB_IN (1U << 2)   //
#define USBD_EVENT_FLAG_DAP_DONE (1U << 3)    //
#define USBD_EVENT_FLAG_CDC_OUT (1U << 4)     //
#define USBD_EVENT_FLAG_CDC_IN (1U << 5)      //
#define USBD_EVENT_FLAG_CDC_TX_DONE (1U << 6) //
#define USBD_EVENT_FLAG_CDC_RX_DONE (1U << 7) //
#define USBD_EVENT_FLAG_CDC_RX_IDLE (1U << 8) //

VOID thread_usbd_main(ULONG id);

#endif /* __THREAD_USBD_H__ */
