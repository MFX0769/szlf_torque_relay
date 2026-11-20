/**
  ******************************************************************************
  * @file    adc.c
  * @author  Kamingo
  * @brief   ADC1 初始化：PA2(ADC123_IN2) 力矩 + PA3(ADC123_IN3) 刹车霍尔，
  *          12bit，双通道扫描连续转换，DMA2 Stream0 循环
  * @date    2026-11-14
  * @version 1.1
  * @note    1. HAL 版本与 RM25 工程一致：STM32F4xx_HAL_Driver V1.7.6。
  *          2. v2.1 起单通道连续；v2.2 加刹车霍尔为 rank2 扫描：
  *             每序列 2 次转换（84+12 周期 ×2），转换率减半至 ~109ksps，
  *             块率随之减半（力矩块平均时间跨度 9.4ms→19ms，更平滑、
  *             固定 +5ms 时延；P1 档 8 帧 160ms 滑均主导，影响可接受）。
  *          3. DMA 缓冲布局：[t0,b0,t1,b1,...] 交替，偶数下标=rank1 力矩，
  *             奇数下标=rank2 刹车（bsp_adc.c 按此拆块平均）。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"
#include "relay_config.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/**
  * @brief  ADC1 初始化
  * @note   ADC 时钟 = PCLK2/4 = 84MHz/4 = 21MHz（F407 ADC 上限 36MHz）
  *         采样 84 + 转换 12 = 96 周期 -> 约 218.75ksps
  * @retval None
  */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = TORQUE_ADC_INSTANCE;
  hadc1.Init.ClockPrescaler = TORQUE_ADC_CLOCK_PRESCALER;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ENABLE;       /* v2.2：双通道扫描 */
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;         /* v2.2：rank1 力矩 + rank2 刹车 */
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 0;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = TORQUE_ADC_CHANNEL;
  sConfig.Rank = 1;
  sConfig.SamplingTime = TORQUE_ADC_SAMPLETIME;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = BRAKE_ADC_CHANNEL;
  sConfig.Rank = 2;
  sConfig.SamplingTime = TORQUE_ADC_SAMPLETIME;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC MSP 初始化：模拟引脚 + DMA2 Stream0
  * @param hadc ADC 句柄
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (hadc->Instance == ADC1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**ADC1 GPIO Configuration
    PA2     ------> ADC1_IN2  (力矩)
    PA3     ------> ADC1_IN3  (刹车霍尔，v2.2)
    */
    GPIO_InitStruct.Pin = TORQUE_ADC_GPIO_PIN | BRAKE_ADC_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(TORQUE_ADC_GPIO_PORT, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);
  }
}

/**
  * @brief ADC MSP 反初始化
  * @param hadc ADC 句柄
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
    __HAL_RCC_ADC1_CLK_DISABLE();
    HAL_GPIO_DeInit(TORQUE_ADC_GPIO_PORT, TORQUE_ADC_GPIO_PIN | BRAKE_ADC_GPIO_PIN);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
