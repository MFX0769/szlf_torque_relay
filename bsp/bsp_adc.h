/**
  ******************************************************************************
  * @file    bsp_adc.h
  * @author  Kamingo
  * @brief   力矩 ADC BSP：连续 DMA 采样 + 块平均
  * @date    2026-10-30
  * @version 1.0
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
  uint16_t raw_avg;   /* 32 点块平均原始值 0..4095 */
  uint16_t mv;        /* raw_avg * 3300 / 4095 */
  uint8_t  saturated; /* 1=超过 3.2V 或接近 ADC 满量程 */
} bsp_adc_sample_t;

/* Exported function declarations --------------------------------------------*/
void Bsp_Adc_Init(void);
uint8_t Bsp_Adc_GetSample(bsp_adc_sample_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ADC_H */
