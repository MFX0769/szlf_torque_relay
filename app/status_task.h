/**
  ******************************************************************************
  * @file    status_task.h
  * @author  Kamingo
  * @brief   状态任务 / 软件看门狗位图（26 版 status_task 范式）
  * @date    2026-10-30
  * @version 1.0
  ******************************************************************************
  */

#ifndef __STATUS_TASK_H
#define __STATUS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdint.h>

/* 任务状态位 */
#define WDG_BIT_TASK_RELAY          (uint32_t)(1U << 0)

/* 硬件通信状态位 */
#define WDG_BIT_BSP_RELAY_UART      (uint32_t)(1U << 1)

typedef struct
{
  uint16_t relay : 1;
  uint16_t uart  : 1;
  uint16_t rsvd  : 14;
} status_t;

extern status_t status;

void wdg_user_set_bit(uint32_t status_bit);
void status_task(void const *argu);

#ifdef __cplusplus
}
#endif

#endif /* __STATUS_TASK_H */
