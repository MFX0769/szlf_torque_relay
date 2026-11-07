/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  Kamingo
  * @brief   板载 LED 心跳（PA1，低有效，510R 限流）
  * @date    2026-10-30
  * @version 1.0
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_led.h"
#include "main.h"
#include "relay_config.h"

/**
  * @brief LED 初始化（GPIO 已在 MX_GPIO_Init 配置）
  * @retval None
  */
void Bsp_Led_Init(void)
{
  Bsp_Led_Off();
}

/**
  * @brief 点亮 LED
  * @retval None
  */
void Bsp_Led_On(void)
{
#if (RELAY_LED_ACTIVE_LOW != 0U)
  HAL_GPIO_WritePin(RELAY_LED_GPIO_PORT, RELAY_LED_GPIO_PIN, GPIO_PIN_RESET);
#else
  HAL_GPIO_WritePin(RELAY_LED_GPIO_PORT, RELAY_LED_GPIO_PIN, GPIO_PIN_SET);
#endif
}

/**
  * @brief 熄灭 LED
  * @retval None
  */
void Bsp_Led_Off(void)
{
#if (RELAY_LED_ACTIVE_LOW != 0U)
  HAL_GPIO_WritePin(RELAY_LED_GPIO_PORT, RELAY_LED_GPIO_PIN, GPIO_PIN_SET);
#else
  HAL_GPIO_WritePin(RELAY_LED_GPIO_PORT, RELAY_LED_GPIO_PIN, GPIO_PIN_RESET);
#endif
}

/**
  * @brief 翻转 LED
  * @retval None
  */
void Bsp_Led_Toggle(void)
{
  HAL_GPIO_TogglePin(RELAY_LED_GPIO_PORT, RELAY_LED_GPIO_PIN);
}
