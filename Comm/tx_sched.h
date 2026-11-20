/**
  ******************************************************************************
  * @file    tx_sched.h
  * @author  Kamingo
  * @brief   USART1 上行单一发送调度器（队列 + 专发送任务）
  * @date    2026-11-15
  * @version 1.0
  * @note    背景（2026-11-15 实车 rec_20261115_002355）：[PA] 37.8/s、CRC 错
  *          ≈24%，刹车活跃段错帧率 +73%——0x01(16B)+0x04(14B) 背靠背 30B/20ms。
  *          本刀=单变量一刀：0x01/0x02/0x03/0x04/0x05 不再在 relay_task 内各自
  *          直写 BSP 队列，统一入本调度器 FIFO，由专发送任务顺序出队调
  *          Uart1_SendPush——帧构造与字节发送彻底解耦，字节流顺序=入队顺序。
  *          不改 0x01/0x04 帧格式、不改发送率、不动 BSP/优先级（纪律）。
  *          队列满=丢帧计数（TxSched_GetDropped），绝不阻塞 relay_task。
  ******************************************************************************
  */

#ifndef __TX_SCHED_H
#define __TX_SCHED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Exported function declarations --------------------------------------------*/
/** @brief 创建发送队列与发送任务（调度器启动前调用一次） */
void TxSched_Init(void);

/** @brief 上行帧入队（非阻塞；满则丢+计数）
 *  @param buf 帧缓冲（内部拷贝）
 *  @param len 帧长（≤ TX_SCHED_MSG_MAX）
 *  @retval 0=入队；1=丢弃 */
uint8_t TxSched_PushUart(const uint8_t *buf, uint16_t len);

/** @brief 发送任务主体（FreeRTOS，阻塞等队列） */
void tx_sched_task(void const *argu);

/** @brief 入队丢弃帧数（观测压力） */
uint32_t TxSched_GetDropped(void);

#ifdef __cplusplus
}
#endif

#endif /* __TX_SCHED_H */
