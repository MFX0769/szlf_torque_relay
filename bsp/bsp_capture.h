/**
  ******************************************************************************
  * @file    bsp_capture.h
  * @author  Kamingo
  * @brief   踏频脉冲采集 BSP：32 脉冲/转，2s 超时判 0（方案开关见 relay_config.h）
  * @date    2026-11-05
  * @version 2.0
  * @note    新默认=门控 DMA（TIM4 外部时钟计数+TIM5 门控 DMA 快照，零中断）；
  *          旧=TIM4_CH1 输入捕获逐脉冲中断（CAPTURE_MODE_IC 回退）。
  *          接口签名两方案一致，上层（relay_task）无感切换。
  ******************************************************************************
  */

#ifndef __BSP_CAPTURE_H
#define __BSP_CAPTURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Exported function declarations --------------------------------------------*/
void Bsp_Capture_Init(void);
uint16_t Bsp_Capture_GetRpmX10(void);
uint8_t Bsp_Capture_IsTimeout(void);
uint32_t Bsp_Capture_GetPulseCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CAPTURE_H */
