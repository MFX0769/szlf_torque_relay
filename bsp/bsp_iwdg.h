/**
  ******************************************************************************
  * @file    bsp_iwdg.h
  * @author  Kamingo
  * @brief   IWDG BSP 接口（默认不启用，宏 RELAY_USE_IWDG 打开）
  * @date    2026-10-30
  * @version 1.0
  ******************************************************************************
  */

#ifndef __BSP_IWDG_H
#define __BSP_IWDG_H

#ifdef __cplusplus
extern "C" {
#endif

void Bsp_Iwdg_Init(void);
void Bsp_Iwdg_Refresh(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_IWDG_H */
