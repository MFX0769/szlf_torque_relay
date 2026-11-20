/**
  ******************************************************************************
  * @file    bsp.h
  * @author  Kamingo
  * @brief   BSP 层总入口（沿用 Taurus26 Bsp/bsp.h 聚合写法）
  * @date    2026-10-30
  * @version 1.0
  ==============================================================================
*/

#ifndef __BSP_H
#define __BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bsp_adc.h"
#include "bsp_button.h"
#include "bsp_capture.h"
#include "bsp_led.h"
#include "bsp_usart.h"
#include "bsp_iwdg.h"

/* Exported function declarations --------------------------------------------*/
void Bsp_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_H */
