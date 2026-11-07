/**
  ******************************************************************************
  * @file    bsp_usart.h
  * @author  Kamingo
  * @brief   USART BSP：IDLE+DMA 接收、TX 环形 DMA 队列、用户回调注册
  * @date    2026-11-01
  * @version 2.0
  * @note    接口约定沿用 Taurus26 重构版 Bsp/bsp_usart.h：
  *          句柄结构体 usart_user_rx_handle_t + 回调 usart_call_back。
  *          v2 变更：TX 改环形 DMA 队列——非阻塞入队、DMA 完成中断自动续传，
  *          排队/阻塞情况可通过统计变量观测（dropped / sent）。
  ******************************************************************************
  */

#ifndef __BSP_USART_H
#define __BSP_USART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include <stdint.h>

/* Private define ------------------------------------------------------------*/
#define BSP_USART_TX_RING_SIZE      512U   /* TX 环形队列大小（字节），≥8 帧 16B 帧余量 */
#define BSP_USART_TX_MAX_ONCE       64U    /* 单次入队上限（一帧最大 16B，留倍数余量） */

/* Private type --------------------------------------------------------------*/
typedef uint32_t (*usart_call_back)(uint8_t *buf, uint16_t len);

/**
  * @brief 串口接收用户句柄（26 版同名结构）
  */
typedef struct
{
  UART_HandleTypeDef *huart;
  uint16_t rx_buffer_size;
  uint8_t *rx_buffer;
  usart_call_back callback_f;
} usart_user_rx_handle_t;

/* Exported variables --------------------------------------------------------*/
extern usart_user_rx_handle_t usart1_rx_user_handle;

/* Exported function declarations --------------------------------------------*/
void bsp_user_usart_Init(UART_HandleTypeDef *huart, uint8_t *Rxbuffer, uint32_t length, usart_call_back fun);
void bsp_user_usart_Receive_Handler(usart_user_rx_handle_t *p);

/* TX 环形队列接口（v2） */
uint8_t Uart1_SendPush(const uint8_t *msg, uint16_t len);
void    Uart1_TxPump(void);
uint32_t Uart1_GetTxDropped(void);
uint32_t Uart1_GetTxSent(void);
uint32_t Uart1_GetRxFrameCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_USART_H */
