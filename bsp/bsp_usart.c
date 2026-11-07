/**
  ******************************************************************************
  * @file    bsp_usart.c
  * @author  Kamingo
  * @brief   USART1 IDLE+DMA 接收 / TX 环形 DMA 队列发送
  * @date    2026-11-01
  * @version 2.0
  * @note    1. 接收沿用 26 版：IDLE 中断取 DMA 剩余计数并回调分发。
  *          2. TX v2：环形队列 + DMA 完成中断续传。
  *             - Uart1_SendPush 非阻塞入队（临界区拷贝，绕回分段）；
  *             - Uart1_TxPump 在入队后/发送完成中断里启动下一段；
  *             - 队列满时丢弃并计数（dropped），绝不在任务里阻塞；
  *             - 发送字节数/丢弃数可观测，用于对账转发延迟与丢帧。
  *          3. F407 用 DMA_Stream_TypeDef->NDTR 取剩余字节（26 版同写法）。
  *          4. 用 HAL_UART_AbortReceive 而不是 HAL_UART_DMAStop：
  *             只停接收，不影响同串口的 DMA 发送（26 版注释同结论）。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_usart.h"
#include "FreeRTOS.h"    /* taskENTER/EXIT_CRITICAL 宏 */
#include "task.h"
#include <stddef.h>

/* Private variables ---------------------------------------------------------*/
usart_user_rx_handle_t usart1_rx_user_handle =
{
  .huart = NULL,
  .rx_buffer_size = 0,
  .rx_buffer = NULL,
  .callback_f = NULL
};

static volatile uint32_t s_uart1_rx_frame_count = 0;

/* TX 环形队列状态 */
static uint8_t  s_tx_ring[BSP_USART_TX_RING_SIZE];
static volatile uint16_t s_tx_head = 0;      /* 写指针（任务侧推进） */
static volatile uint16_t s_tx_tail = 0;      /* 已确认发送完成的读指针（IRQ 侧推进） */
static volatile uint8_t  s_tx_busy = 0;      /* DMA 在途标志 */
static volatile uint32_t s_tx_dropped = 0;   /* 队列满丢弃字节数 */
static volatile uint32_t s_tx_sent = 0;      /* 已完成发送字节数 */

/**
  * @brief 初始化用户串口：使能 IDLE 中断 + 启动 DMA 接收
  * @param huart   UART 句柄
  * @param Rxbuffer 接收缓冲
  * @param length  缓冲长度
  * @param fun     用户回调（IDLE 时调用）
  * @retval None
  */
void bsp_user_usart_Init(UART_HandleTypeDef *huart, uint8_t *Rxbuffer, uint32_t length, usart_call_back fun)
{
  if ((huart == NULL) || (Rxbuffer == NULL) || (length == 0U))
  {
    Error_Handler();
  }

  if (huart->Instance == USART1)
  {
    usart1_rx_user_handle.huart = huart;
    usart1_rx_user_handle.rx_buffer = Rxbuffer;
    usart1_rx_user_handle.rx_buffer_size = (uint16_t)length;
    usart1_rx_user_handle.callback_f = fun;

    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    (void)HAL_UART_Receive_DMA(huart, Rxbuffer, length);
  }
}

/**
  * @brief IDLE 空闲中断处理：取本次 DMA 接收长度并回调
  * @param p 用户串口句柄
  * @retval None
  * @note  在 USART1_IRQHandler 中 HAL_UART_IRQHandler 之后调用。
  */
void bsp_user_usart_Receive_Handler(usart_user_rx_handle_t *p)
{
  uint16_t rx_len;

  if ((p == NULL) || (p->huart == NULL) || (p->huart->hdmarx == NULL))
  {
    return;
  }

  if (__HAL_UART_GET_FLAG(p->huart, UART_FLAG_IDLE) != RESET)
  {
    /* 清 IDLE 标志，否则会反复进中断 */
    __HAL_UART_CLEAR_IDLEFLAG(p->huart);

    /* 只中止接收 DMA，保留发送通道 */
    (void)HAL_UART_AbortReceive(p->huart);

    /* 剩余计数 -> 本次收到多少字节 */
    rx_len = (uint16_t)(p->rx_buffer_size -
              ((DMA_Stream_TypeDef *)p->huart->hdmarx->Instance)->NDTR);

    s_uart1_rx_frame_count++;

    if (p->callback_f != NULL)
    {
      (void)p->callback_f(p->rx_buffer, rx_len);
    }

    /* 重新使能 IDLE 并重启 DMA 接收 */
    __HAL_UART_ENABLE_IT(p->huart, UART_IT_IDLE);
    (void)HAL_UART_Receive_DMA(p->huart, p->rx_buffer, p->rx_buffer_size);
  }
}

/**
  * @brief TX 环形队列入队（非阻塞，任务上下文调用）
  * @param msg 数据
  * @param len 长度（≤ BSP_USART_TX_MAX_ONCE）
  * @retval 0=入队成功；1=队列满（丢弃并计数）
  */
uint8_t Uart1_SendPush(const uint8_t *msg, uint16_t len)
{
  uint16_t head;
  uint16_t free_space;
  uint16_t i;

  if ((msg == NULL) || (len == 0U) || (len > BSP_USART_TX_MAX_ONCE))
  {
    return 1U;
  }

  taskENTER_CRITICAL();
  head = s_tx_head;
  /* 空闲空间：head 与 tail 相等视为全空 */
  free_space = (uint16_t)(BSP_USART_TX_RING_SIZE - 1U -
                          ((uint16_t)(head - s_tx_tail) % BSP_USART_TX_RING_SIZE));

  if (len > free_space)
  {
    s_tx_dropped += len;
    taskEXIT_CRITICAL();
    return 1U;
  }

  for (i = 0U; i < len; i++)
  {
    s_tx_ring[head] = msg[i];
    head = (uint16_t)((head + 1U) % BSP_USART_TX_RING_SIZE);
  }
  s_tx_head = head;
  taskEXIT_CRITICAL();

  /* 入队成功，尝试启动发送（内部自会判断 busy） */
  Uart1_TxPump();
  return 0U;
}

/**
  * @brief TX 泵：若 DMA 空闲且队列非空，启动下一段连续区发送
  * @note  任务上下文与 DMA 完成中断都会调用；临界区防双重启动。
  *        2026-11-07 修复：本函数会被 HAL_UART_TxCpltCallback（DMA TC 中断）
  *        调用，原先无条件用 taskENTER_CRITICAL() —— 该 API 只允许任务上下文，
  *        在中断里会触发 configASSERT 死循环。实测卡点：PC=vPortEnterCritical、
  *        LR=Uart1_TxPump，表现为发完第 1 帧后整机停摆（LED 不闪、计数冻结）。
  *        改为按 __get_IPSR() 自动选择任务/中断版本临界区。
  */
void Uart1_TxPump(void)
{
  uint16_t tail = 0U;
  uint16_t head = 0U;
  uint16_t cont_len;
  BaseType_t need_send = pdFALSE;
  BaseType_t in_isr = (__get_IPSR() != 0U) ? pdTRUE : pdFALSE;
  UBaseType_t saved = 0U;

  if (in_isr != pdFALSE)
  {
    saved = taskENTER_CRITICAL_FROM_ISR();
  }
  else
  {
    taskENTER_CRITICAL();
  }

  if ((s_tx_busy == 0U) && (s_tx_head != s_tx_tail))
  {
    s_tx_busy = 1U;
    tail = s_tx_tail;
    head = s_tx_head;
    need_send = pdTRUE;
  }

  if (in_isr != pdFALSE)
  {
    taskEXIT_CRITICAL_FROM_ISR(saved);
  }
  else
  {
    taskEXIT_CRITICAL();
  }

  if (need_send == pdFALSE)
  {
    return;
  }

  /* 本段连续长度：绕回则先发到环形缓冲末尾 */
  cont_len = (head > tail) ? (uint16_t)(head - tail)
                           : (uint16_t)(BSP_USART_TX_RING_SIZE - tail);

  if (HAL_UART_Transmit_DMA(&huart1, &s_tx_ring[tail], cont_len) != HAL_OK)
  {
    /* 启动失败（如波特率切换中 gState 忙）：下轮 pump 重试 */
    s_tx_busy = 0U;
  }
}

/**
  * @brief HAL 发送完成回调：推进 tail 并续发（DMA TC 中断上下文）
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    /* 中断上下文：push 侧（任务）会同时读 tail，必须用 ISR 版临界区保护。
     * 2026-11-07：原先无保护直接改 tail/sent/busy。 */
    UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
    s_tx_tail = (uint16_t)((s_tx_tail + huart->TxXferSize) % BSP_USART_TX_RING_SIZE);
    s_tx_sent += huart->TxXferSize;
    s_tx_busy = 0U;
    taskEXIT_CRITICAL_FROM_ISR(saved);

    /* 队列里还有数据就在中断里接着发 */
    Uart1_TxPump();
  }
}

/**
  * @brief 获取 TX 丢弃字节数（观测排队/阻塞压力）
  */
uint32_t Uart1_GetTxDropped(void)
{
  return s_tx_dropped;
}

/**
  * @brief 获取已完成发送字节数（对账用）
  */
uint32_t Uart1_GetTxSent(void)
{
  return s_tx_sent;
}

/**
  * @brief 获取 USART1 收到的帧数（调试/看门狗用）
  * @retval 帧计数
  */
uint32_t Uart1_GetRxFrameCount(void)
{
  return s_uart1_rx_frame_count;
}
