/**
  ******************************************************************************
  * @file    usart_comm.h
  * @author  Kamingo
  * @brief   USART 通信中间层：接收注册/分发（26 版 Comm 写法）
  * @date    2026-10-30
  * @version 1.0
  ******************************************************************************
  */

#ifndef __USART_COMM_H
#define __USART_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bsp_usart.h"
#include "relay_config.h"

/* Exported variables --------------------------------------------------------*/
extern uint8_t Uart1_Rx_Buff[RELAY_UART_RX_BUF_SIZE];

/* Exported function declarations --------------------------------------------*/
void comm_usart_init(void);
void comm_usart_rebind(void); /* 波特率切换后重绑 RX（缓冲/回调不变） */

#ifdef __cplusplus
}
#endif

#endif /* __USART_COMM_H */
