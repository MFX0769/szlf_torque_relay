/**
  ******************************************************************************
  * @file    bsp_iwdg.c
  * @author  Kamingo
  * @brief   IWDG 封装：RELAY_USE_IWDG=0 时为空实现
  * @date    2026-10-30
  * @version 1.0
  * @note    IWDG 一旦启动无法关闭，调试阶段默认关闭；
  *          量产/长期运行把 Inc/relay_config.h 的 RELAY_USE_IWDG 改 1，
  *          并在 main.c 打开 MX_IWDG_Init 调用（或由 Bsp_Iwdg_Init 调用）。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_iwdg.h"
#include "iwdg.h"
#include "relay_config.h"

/**
  * @brief 初始化 IWDG（可选）
  * @retval None
  */
void Bsp_Iwdg_Init(void)
{
#if (RELAY_USE_IWDG != 0U)
  MX_IWDG_Init();
#endif
}

/**
  * @brief 喂狗（可选）
  * @retval None
  */
void Bsp_Iwdg_Refresh(void)
{
#if (RELAY_USE_IWDG != 0U)
  (void)HAL_IWDG_Refresh(&hiwdg);
#endif
}
