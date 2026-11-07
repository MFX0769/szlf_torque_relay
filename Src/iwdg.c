/**
  ******************************************************************************
  * @file    iwdg.c
  * @author  Kamingo
  * @brief   IWDG 初始化（默认不调用；RELAY_USE_IWDG 打开后由 Bsp_Iwdg_Init 调用）
  * @date    2026-10-30
  * @version 1.0
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "iwdg.h"
#include "relay_config.h"

IWDG_HandleTypeDef hiwdg;

/**
  * @brief IWDG 初始化
  * @note  LSI 32kHz / 32 = 1kHz，Reload 500 -> 500ms 超时
  * @retval None
  */
void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = RELAY_IWDG_PRESCALER;
  hiwdg.Init.Reload = RELAY_IWDG_RELOAD;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
