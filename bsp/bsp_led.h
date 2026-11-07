/**
  ******************************************************************************
  * @file    bsp_led.h
  * @author  Kamingo
  * @brief   板载 LED（PA1，低有效）BSP
  * @date    2026-10-30
  * @version 1.0
  ******************************************************************************
  */

#ifndef __BSP_LED_H
#define __BSP_LED_H

#ifdef __cplusplus
extern "C" {
#endif

void Bsp_Led_Init(void);
void Bsp_Led_On(void);
void Bsp_Led_Off(void);
void Bsp_Led_Toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_LED_H */
