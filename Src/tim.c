/**
  ******************************************************************************
  * @file    tim.c
  * @author  Kamingo
  * @brief   踏频脉冲计数定时器初始化（方案开关见 relay_config.h）
  * @date    2026-11-05
  * @version 1.1
  * @note    CAPTURE_MODE_IC（旧）：TIM4_CH1 输入捕获，PB6，20kHz 计数逐脉冲中断；
  *          20kHz 下 16bit 回绕 3.27s > 2s 超时窗，软件不处理溢出。
  *          CAPTURE_MODE_GATED_DMA（新，默认）：TIM4 外部时钟模式 1（SMS=111），
  *          TI1FP1 上升沿直接拨 CNT（PSC=0，每脉冲 +1，脉冲路径零中断），
  *          ARR=0xFFFF 自然回绕由任务侧 uint16 差分消化；另加门控 TIM5
  *          10Hz UPDATE 触发 DMA 快照 CNT（DMA/差分在 bsp_capture.c）。
  *          回退：relay_config.h 把 CADENCE_CAPTURE_MODE 改回 CAPTURE_MODE_IC。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "relay_config.h"

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;
#if (CADENCE_CAPTURE_MODE == CAPTURE_MODE_GATED_DMA)
TIM_HandleTypeDef htim5;
#endif

/**
  * @brief  TIM4 初始化：按 CADENCE_CAPTURE_MODE 分方案配置
  * @retval None
  */
void MX_TIM4_Init(void)
{
#if (CADENCE_CAPTURE_MODE == CAPTURE_MODE_GATED_DMA)
  /* 新方案：外部时钟模式 1。借道 HAL_TIM_IC_Init 完成基础寄存器与 MSP
   * （时钟/PB6 复用/NVIC——门控模式下 TIM4 不产生中断，NVIC 使能无害），
   * 再用 SlaveConfigSynchro 配 SMS=111 + TI1FP1 触发（内部设 TI1 滤波、
   * 极性上升沿、CC1E=0，滤波强度沿用 CADENCE_IC_FILTER）。 */
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};

  htim4.Instance = CADENCE_CAPTURE_INSTANCE;
  htim4.Init.Prescaler = 0U; /* 外部时钟模式下内部时钟不走，置 0 保持语义清晰 */
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFFU; /* 65536 脉冲回绕，uint16 差分自然消化 */
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1; /* SMS=111 */
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;        /* PB6 滤波输入 */
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlaveConfig.TriggerFilter = CADENCE_IC_FILTER; /* 与旧方案同强度毛刺滤波 */
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
#else
  /* 旧方案：输入捕获，20kHz 内部时钟测周期 */
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim4.Instance = CADENCE_CAPTURE_INSTANCE;
  htim4.Init.Prescaler = CADENCE_TIMER_PRESCALER; /* 84MHz / 4200 = 20kHz */
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = CADENCE_TIMER_PERIOD;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = CADENCE_IC_FILTER; /* 4 个采样点滤波，抑制霍尔脉冲毛刺 */
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, CADENCE_CAPTURE_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
#endif
}

#if (CADENCE_CAPTURE_MODE == CAPTURE_MODE_GATED_DMA)
/**
  * @brief  门控定时器 TIM5 初始化（10Hz 固定窗，无中断，只出 UPDATE DMA 请求）
  * @note   时钟先开再 Init：工程未定义 HAL_TIM_Base_MspInit（HAL timebase
  *         用 TIM10 且手动开时钟），弱符号默认为空，不重复定义。
  * @retval None
  */
void MX_TIM5_Init(void)
{
  __HAL_RCC_TIM5_CLK_ENABLE();

  htim5.Instance = CADENCE_GATE_INSTANCE;
  htim5.Init.Prescaler = CADENCE_GATE_PSC;   /* 84MHz/8400 = 10kHz */
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = CADENCE_GATE_ARR;      /* 10kHz × 100ms = 1000 计数/窗 */
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif

/**
  * @brief TIM4 MSP 初始化：PB6 复用 + NVIC
  * @param tim_icHandle 定时器句柄
  * @retval None
  */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* tim_icHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (tim_icHandle->Instance == CADENCE_CAPTURE_INSTANCE)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1
    */
    GPIO_InitStruct.Pin = CADENCE_CAPTURE_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = CADENCE_CAPTURE_GPIO_AF;
    HAL_GPIO_Init(CADENCE_CAPTURE_GPIO_PORT, &GPIO_InitStruct);

    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM4_IRQn, CADENCE_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  }
}

/**
  * @brief TIM4 MSP 反初始化
  * @param tim_icHandle 定时器句柄
  * @retval None
  */
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* tim_icHandle)
{
  if (tim_icHandle->Instance == CADENCE_CAPTURE_INSTANCE)
  {
    __HAL_RCC_TIM4_CLK_DISABLE();
    HAL_GPIO_DeInit(CADENCE_CAPTURE_GPIO_PORT, CADENCE_CAPTURE_GPIO_PIN);
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
