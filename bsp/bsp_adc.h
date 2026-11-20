/**
  ******************************************************************************
  * @file    bsp_adc.h
  * @author  Kamingo
  * @brief   力矩+刹车 ADC BSP：连续 DMA 采样（双通道扫描）+ 块平均
  * @date    2026-11-14
  * @version 1.1
  * @note    v2.2：ADC1 规则序列 rank1=PA2 力矩、rank2=PA3 刹车霍尔，
  *          DMA 缓冲 [t,b] 交替；两路各自 32 点块平均，一次 GetSample
  *          同时取走（单块就绪标志，避免双 getter 竞态）。
  ******************************************************************************
  */

#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  uint16_t raw_avg;   /* 力矩 32 点块平均原始值 0..4095 */
  uint16_t mv;        /* raw_avg * 3300 / 4095 */
  uint8_t  saturated; /* 1=超过 3.2V 或接近 ADC 满量程 */
  uint16_t brake_raw; /* 刹车通道 32 点块平均原始值 0..4095（v2.2） */
} bsp_adc_sample_t;

/* Exported function declarations --------------------------------------------*/
void Bsp_Adc_Init(void);
uint8_t Bsp_Adc_GetSample(bsp_adc_sample_t *out);
uint16_t Bsp_Adc_GetBrakeRaw(void); /* 最新刹车块平均（hall 设备层读数入口） */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ADC_H */
