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
#include "app_brake.h"    /* Brake_RequestCal：刹车端点标定命令（v2.2） */
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

  /* 命令通道：ASCII "PR1"/"PR2"/"PR3" = 切 P1/P2/P3 档（其余忽略）。
   * 2026-11-10 改：原实现单个 '1'/'2'/'3' 就切档，而本回调在中断上下文，
   * Relay_SetProfile 里的 taskENTER_CRITICAL 会 configASSERT 死锁；且 GD 的
   * 串口文本里数字极多，会随机触发。现在：①要求 "PR"+digit 三字节序列；
   * ②ISR 里只置请求（Relay_RequestProfile），实际切换在 relay_task。
   * 2026-11-14 v2.2 加："BR1"=记录松开端 adc_max、"BR2"=记录捏死端 adc_min
   * （ Brake_Tick 任务侧执行取当前滤波值，端点标定 RAM 生效掉电丢）。
   * 2026-11-14 v2.3 加行程校准状态机命令："BRC"=进校准(LO)、"BRN"=LO→HI、
   * "BRA"=apply、"BRX"=取消、"BRQ"=查询（均回 0x05 事件帧，见 prot_relay.h）。 */
  if ((buf != NULL) && (len > 0U))
  {
    uint16_t i;
    for (i = 0U; (uint16_t)(i + 2U) < len; i++)
    {
      if ((buf[i] == 'P') && (buf[i + 1U] == 'R') &&
          (buf[i + 2U] >= '1') && (buf[i + 2U] <= '3'))
      {
        Relay_RequestProfile((uint8_t)(buf[i + 2U] - '1'));
      }
      if ((buf[i] == 'B') && (buf[i + 1U] == 'R') &&
          ((buf[i + 2U] == '1') || (buf[i + 2U] == '2')))
      {
        Brake_RequestCal((uint8_t)(buf[i + 2U] - '0'));
      }
      if ((buf[i] == 'B') && (buf[i + 1U] == 'R'))
      {
        if (buf[i + 2U] == 'C')      { Brake_RequestCalCmd(BRAKE_CAL_CMD_START); }
        else if (buf[i + 2U] == 'N') { Brake_RequestCalCmd(BRAKE_CAL_CMD_NEXT); }
        else if (buf[i + 2U] == 'A') { Brake_RequestCalCmd(BRAKE_CAL_CMD_APPLY); }
        else if (buf[i + 2U] == 'X') { Brake_RequestCalCmd(BRAKE_CAL_CMD_CANCEL); }
        else if (buf[i + 2U] == 'Q') { Brake_RequestCalCmd(BRAKE_CAL_CMD_QUERY); }
        else
        {
          /* 其余忽略 */
        }
      }
    }
    memset(buf, 0, len);
  }

  return 0;
}
