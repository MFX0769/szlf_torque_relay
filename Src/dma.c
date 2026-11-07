/**
  ******************************************************************************
  * @file    dma.c
  * @author  Kamingo
  * @brief   DMA2 时钟与中断初始化
  * @date    2026-10-30
  * @version 1.0
  * @note    Stream0=ADC1，Stream2=USART1_RX，Stream7=USART1_TX
  *          （F407 DMA2 请求映射，见参考手册 RM0090 DMA 请求表）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "dma.h"
#include "relay_config.h"

/**
  * @brief 使能 DMA2 时钟并配置三个 Stream 的 NVIC
  * @retval None
  */
void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration (ADC1) */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, RELAY_DMA_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration (USART1_RX) */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, RELAY_DMA_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration (USART1_TX) */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, RELAY_DMA_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
