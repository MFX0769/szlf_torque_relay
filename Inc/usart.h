/**
  ******************************************************************************
  * @file    usart.h
  * @author  Kamingo
  * @brief   USART1 初始化（PA9/PA10，115200 8N1，DMA 收发）
  * @date    2026-10-30
  * @version 1.0
  ******************************************************************************
  */

#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

void MX_USART1_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H */
