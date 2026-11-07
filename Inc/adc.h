/**
  ******************************************************************************
  * @file    adc.h
  * @author  Kamingo
  * @brief   ADC1 初始化（PA2 / ADC123_IN2，连续+DMA 循环）
  * @date    2026-10-30
  * @version 1.0
  ******************************************************************************
  */

#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

void MX_ADC1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H */
