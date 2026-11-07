/**
  ******************************************************************************
  * @file    bsp_adc.c
  * @author  Kamingo
  * @brief   力矩 ADC：ADC1 连续转换 + DMA2 Stream0 循环 + 半满/全满双块
  * @date    2026-10-30
  * @version 1.0
  * @note    1. STM32F407 的 ADC 没有软件校准位/校准 API（HAL 1.7.6 也
  *             没有 HAL_ADCEx_Calibration_Start），因此不做软件校准。
  *          2. DMA 双块：半满回调表示块 0 就绪，全满回调表示块 1 就绪；
  *             任务在另一块被 DMA 写入期间拷贝就绪块。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_adc.h"
#include "adc.h"
#include "relay_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

/* Private variables ---------------------------------------------------------*/
static uint16_t s_adc_dma_buf[TORQUE_ADC_DMA_BUF_SAMPLES];

static volatile uint8_t s_block_ready = 0;
static volatile uint8_t s_ready_idx = 0;
static bsp_adc_sample_t s_last_sample = {0, 0, 0};

/**
  * @brief 启动 ADC DMA 连续采样
  * @retval None
  */
void Bsp_Adc_Init(void)
{
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc_dma_buf,
                        TORQUE_ADC_DMA_BUF_SAMPLES) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief 获取最新一块 32 点平均结果
  * @param out 输出
  * @retval 1=本次拿到新块；0=没有新块，out 返回上一次结果
  */
uint8_t Bsp_Adc_GetSample(bsp_adc_sample_t *out)
{
  uint16_t local[TORQUE_ADC_BLOCK_SAMPLES];
  uint32_t sum = 0;
  uint8_t ready;
  uint8_t idx;
  uint8_t i;

  if (out == NULL)
  {
    return 0;
  }

  taskENTER_CRITICAL();
  ready = s_block_ready;
  idx = s_ready_idx;
  s_block_ready = 0;
  if (ready != 0U)
  {
    for (i = 0; i < TORQUE_ADC_BLOCK_SAMPLES; i++)
    {
      local[i] = s_adc_dma_buf[(uint32_t)idx * TORQUE_ADC_BLOCK_SAMPLES + i];
    }
  }
  taskEXIT_CRITICAL();

  if (ready == 0U)
  {
    *out = s_last_sample;
    return 0;
  }

  for (i = 0; i < TORQUE_ADC_BLOCK_SAMPLES; i++)
  {
    sum += local[i];
  }

  s_last_sample.raw_avg = (uint16_t)(sum / TORQUE_ADC_BLOCK_SAMPLES);
  s_last_sample.mv = (uint16_t)TORQUE_ADC_MV(s_last_sample.raw_avg);
  s_last_sample.saturated = (uint8_t)((s_last_sample.raw_avg >= TORQUE_ADC_SAT_RAW) ||
                                      (s_last_sample.mv >= TORQUE_SAT_MV));

  *out = s_last_sample;
  return 1;
}

/**
  * @brief ADC DMA 半满：块 0 就绪
  * @param hadc ADC 句柄
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == TORQUE_ADC_INSTANCE)
  {
    s_ready_idx = 0;
    s_block_ready = 1;
  }
}

/**
  * @brief ADC DMA 全满：块 1 就绪
  * @param hadc ADC 句柄
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == TORQUE_ADC_INSTANCE)
  {
    s_ready_idx = 1;
    s_block_ready = 1;
  }
}
