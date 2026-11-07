/**
  ******************************************************************************
  * @file    usb_comm.h
  * @author  Kamingo
  * @brief   USB CDC 通信封装（26 版 Comm 层风格，F407 OTG_FS 版）
  * @date    2026-11-01
  * @version 1.0
  * @note    角色：次选上行通道，主要用于 PC 侧数据收集/调试。
  *          板上 USB 座直连 PA11/PA12（USB FS device，无 USB-UART 桥），
  *          对端必须是 USB Host（PC CDC 虚拟串口）；GD32 与本板都是
  *          device 口，两者直连走不通，给 GD32 的链路走 USART1。
  *          USB 未插入/未打开串口时 Usb_Send 静默丢弃并计数，不影响串口。
  ******************************************************************************
  */

#ifndef __USB_COMM_H
#define __USB_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported function declarations --------------------------------------------*/
void     comm_usb_init(void);
void     OTG_FS_IRQ_Handler(void);
uint8_t  Usb_Send(const uint8_t *buf, uint16_t len);
uint8_t  Usb_IsConnected(void);
uint32_t Usb_GetDropped(void);

#ifdef __cplusplus
}
#endif

#endif /* __USB_COMM_H */
