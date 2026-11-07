/**
  ******************************************************************************
  * @file    gpio.c
  * @author  Kamingo
  * @brief   GPIO 初始化：PA1 板载 LED（低有效），初始熄灭
  * @date    2026-10-30
  * @version 1.0
  * @note    PA0=板载按键 K1、PA1=LED D2（510R）均在 DevEBox 上有占用；
  *          力矩 ADC 因此选 PA2，不选 PA0/PA1。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "relay_config.h"

/**
  * @brief 配置 PA1 为推挽输出，初始高电平（LED 灭）
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELAY_LED_GPIO_PORT, RELAY_LED_GPIO_PIN, GPIO_PIN_SET);

  /* Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = RELAY_LED_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_LED_GPIO_PORT, &GPIO_InitStruct);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
