/**
  ******************************************************************************
  * @file    usart_comm.c
  * @author  Kamingo
  * @brief   USART1 接收中间层：注册 IDLE+DMA 回调，解析入口预留
  * @date    2026-10-30
  * @version 1.0
  * @note    26 版把接收缓冲放在 .RAM_D1（H7 的 D1 SRAM）；F407 没有
  *          D1 域，这里放普通 .bss，属平台差异。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart_comm.h"
#include "status_task.h"
#include "relay_task.h"   /* Relay_SetProfile：切档命令 */
#include <string.h>

/* Private variables ---------------------------------------------------------*/
uint8_t Uart1_Rx_Buff[RELAY_UART_RX_BUF_SIZE];

/* Private function declarations ---------------------------------------------*/
static uint32_t User_USART1_RxCpltCallback(uint8_t *buf, uint16_t len);

/**
  * @brief 通信层初始化：注册 USART1 接收回调并启动 IDLE+DMA
  * @retval None
  */
void comm_usart_init(void)
{
  bsp_user_usart_Init(&huart1, Uart1_Rx_Buff, RELAY_UART_RX_BUF_SIZE,
                      User_USART1_RxCpltCallback);
}

/**
  * @brief 波特率切换后重绑 RX（缓冲与回调不变，Relay_SetProfile 调用）
  */
void comm_usart_rebind(void)
{
  bsp_user_usart_Init(&huart1, Uart1_Rx_Buff, RELAY_UART_RX_BUF_SIZE,
                      User_USART1_RxCpltCallback);
}

/**
  * @brief USART1 接收完成回调（IDLE 触发）
  * @param buf 数据
  * @param len 长度
  * @retval 0
  */
static uint32_t User_USART1_RxCpltCallback(uint8_t *buf, uint16_t len)
{
  /* 收到任意字节即认为链路活着（GD32 侧可选发心跳/命令） */
  wdg_user_set_bit(WDG_BIT_BSP_RELAY_UART);

  /* 命令通道：ASCII '1'/'2'/'3' = 切 P1/P2/P3 档（其余字节忽略） */
  if ((buf != NULL) && (len > 0U))
  {
    uint16_t i;
    for (i = 0U; i < len; i++)
    {
      if ((buf[i] >= '1') && (buf[i] <= '3'))
      {
        (void)Relay_SetProfile((uint8_t)(buf[i] - '1'));
      }
    }
    memset(buf, 0, len);
  }

  return 0;
}
