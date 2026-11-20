/**
  ******************************************************************************
  * @file    tx_sched.c
  * @author  Kamingo
  * @brief   USART1 上行单一发送调度器实现
  * @date    2026-11-15
  * @version 1.0
  * @note    队列元素=定长帧槽（len+buf），深度 32：峰值 3 帧型并发（0x01+0x04+
  *          0x05/事件）×50Hz 下 2s 缓冲，实际消费速率 11520B/s ≫ 生产 1500B/s，
  *          队列常态近似空。发送任务阻塞等队列，出队后调 Uart1_SendPush
  *          （BSP 内部仍是无阻塞环形 DMA 队列，本层不改 BSP）。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tx_sched.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "bsp_usart.h"
#include "relay_config.h"  /* RELAY_TARE_FRAME_LEN=帧长上限 23B */
#include <stddef.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define TX_SCHED_MSG_MAX        (RELAY_TARE_FRAME_LEN)  /* 23B：全帧型上限 */
#define TX_SCHED_QUEUE_DEPTH    (32U)

/* Private types -------------------------------------------------------------*/
typedef struct
{
  uint8_t len;
  uint8_t buf[TX_SCHED_MSG_MAX];
} tx_sched_msg_t;

/* Private variables ---------------------------------------------------------*/
static QueueHandle_t s_queue = NULL;
static volatile uint32_t s_dropped = 0;

/**
  * @brief 创建发送队列与发送任务
  * @note  任务由 freertos.c 的 osThreadCreate 创建；本函数只建队列。
  *        须在 relay_task 首次入队前调用（MX_FREERTOS_Init 内、建任务前）。
  */
void TxSched_Init(void)
{
  if (s_queue == NULL)
  {
    s_queue = xQueueCreate(TX_SCHED_QUEUE_DEPTH, sizeof(tx_sched_msg_t));
    if (s_queue == NULL)
    {
      Error_Handler();
    }
  }
}

/**
  * @brief 上行帧入队（非阻塞）
  * @retval 0=入队成功；1=队列满丢弃（计数）
  */
uint8_t TxSched_PushUart(const uint8_t *buf, uint16_t len)
{
  tx_sched_msg_t msg;

  if ((buf == NULL) || (len == 0U) || (len > TX_SCHED_MSG_MAX))
  {
    return 1U;
  }
  if (s_queue == NULL)
  {
    return 1U;   /* 未初始化：直接丢（初始化序错误应尽早暴露） */
  }

  msg.len = (uint8_t)len;
  memcpy(msg.buf, buf, len);

  if (xQueueSend(s_queue, &msg, 0U) != pdTRUE)
  {
    s_dropped++;
    return 1U;
  }
  return 0U;
}

/**
  * @brief 发送任务主体：阻塞等队列，出队顺序调 BSP 发送
  * @note  单一消费者 ⇒ Uart1_SendPush 调用严格串行；任务级优先级与
  *        relay 相同档（AboveNormal），消费能力远超生产，不入瓶颈。
  */
void tx_sched_task(void const *argu)
{
  tx_sched_msg_t msg;
  (void)argu;

  for (;;)
  {
    if ((s_queue != NULL) && (xQueueReceive(s_queue, &msg, osWaitForever) == pdTRUE))
    {
      (void)Uart1_SendPush(msg.buf, msg.len);
    }
  }
}

/**
  * @brief 入队丢弃帧数
  */
uint32_t TxSched_GetDropped(void)
{
  return s_dropped;
}
