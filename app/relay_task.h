/**
  ******************************************************************************
  * @file    relay_task.h
  * @author  Kamingo
  * @brief   中转任务：采样→换算→组帧→USART1 + USB CDC 双路发送
  * @date    2026-11-01
  * @version 2.0
  * @note    v2：三档性能档位（运行时可切，串口收 '1'/'2'/'3'），
  *          帧内带发送 tick，TX 走环形 DMA 队列，USB CDC 并行发送。
  ******************************************************************************
  */

#ifndef __RELAY_TASK_H
#define __RELAY_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void relay_task(void const *argu);

/* 档位切换（usart_comm 命令层调用）：idx 0/1/2 = P1/P2/P3。
 * 返回 0=成功（含波特率重配），1=参数非法。切换会短暂清空 TX 队列。 */
uint8_t Relay_SetProfile(uint8_t idx);
uint8_t Relay_GetProfile(void);
const char *Relay_ProfileName(uint8_t idx);

#ifdef __cplusplus
}
#endif

#endif /* __RELAY_TASK_H */
