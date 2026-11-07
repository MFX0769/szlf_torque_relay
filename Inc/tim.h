/**
  ******************************************************************************
  * @file    tim.h
  * @author  Kamingo
  * @brief   踏频脉冲定时器初始化（方案开关见 relay_config.h）
  * @date    2026-11-05
  * @version 1.1
  ******************************************************************************
  */

#ifndef __TIM_H
#define __TIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "relay_config.h"

extern TIM_HandleTypeDef htim4;
#if (CADENCE_CAPTURE_MODE == CAPTURE_MODE_GATED_DMA)
extern TIM_HandleTypeDef htim5;
void MX_TIM5_Init(void);
#endif

void MX_TIM4_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H */
