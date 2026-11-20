/**
  ******************************************************************************
  * @file    bsp_adc.c
  * @author  Kamingo
  * @brief   力矩+刹车 ADC：ADC1 双通道扫描 + DMA2 Stream0 循环 + 半满/全满双块
  * @date    2026-11-14
  * @version 1.1
  * @note    1. STM32F407 的 ADC 没有软件校准位/校准 API（HAL 1.7.6 也
  *             没有 HAL_ADCEx_Calibration_Start），因此不做软件校准。
  *          2. DMA 双块：半满回调表示块 0 就绪，全满回调表示块 1 就绪；
  *             任务在另一块被 DMA 写入期间拷贝就绪块。
  *          3. v2.2 布局：每块 = BLOCK_SAMPLES 个 [力矩,刹车] 对，即
  *             2×BLOCK_SAMPLES 个 halfword；偶数下标=rank1 力矩，
  *             奇数下标=rank2 刹车。块率相对单通道版减半（~1.7kHz），
  *             仍远高于 50Hz 帧率需求。
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
/* 双块 × 每块 32 对 × 每对 2 通道 */
static uint16_t s_adc_dma_buf[2U * 2U * TORQUE_ADC_BLOCK_SAMPLES];

static volatile uint8_t s_block_ready = 0;
static volatile uint8_t s_ready_idx = 0;
static bsp_adc_sample_t s_last_sample = {0, 0, 0, 0};

#define ADC_BLOCK_HALFWORDS   (2U * TORQUE_ADC_BLOCK_SAMPLES)

/**
  * @brief 启动 ADC DMA 连续采样
  * @retval None
  */
void Bsp_Adc_Init(void)
{
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc_dma_buf,
                        2U * ADC_BLOCK_HALFWORDS) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief 获取最新一块 32 点平均结果（力矩+刹车一次取走）
  * @param out 输出
  * @retval 1=本次拿到新块；0=没有新块，out 返回上一次结果
  */
uint8_t Bsp_Adc_GetSample(bsp_adc_sample_t *out)
{
  uint16_t local[ADC_BLOCK_HALFWORDS];
  uint32_t sum_torque = 0;
  uint32_t sum_brake = 0;
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
    for (i = 0; i < ADC_BLOCK_HALFWORDS; i++)
    {
      local[i] = s_adc_dma_buf[(uint32_t)idx * ADC_BLOCK_HALFWORDS + i];
    }
  }
  taskEXIT_CRITICAL();

  if (ready == 0U)
  {
    *out = s_last_sample;
    return 0;
  }

  /* [t,b] 交替：偶数下标=力矩，奇数下标=刹车 */
  for (i = 0; i < TORQUE_ADC_BLOCK_SAMPLES; i++)
  {
    sum_torque += local[2U * i];
    sum_brake += local[2U * i + 1U];
  }

  s_last_sample.raw_avg = (uint16_t)(sum_torque / TORQUE_ADC_BLOCK_SAMPLES);
  s_last_sample.mv = (uint16_t)TORQUE_ADC_MV(s_last_sample.raw_avg);
  s_last_sample.saturated = (uint8_t)((s_last_sample.raw_avg >= TORQUE_ADC_SAT_RAW) ||
                                      (s_last_sample.mv >= TORQUE_SAT_MV));
  s_last_sample.brake_raw = (uint16_t)(sum_brake / TORQUE_ADC_BLOCK_SAMPLES);

  *out = s_last_sample;
  return 1;
}

/**
  * @brief 最新刹车通道块平均（hall 设备层 hall_hw_read_adc 的读数入口）
  * @retval 刹车 raw 0..4095（无新块时为上一次结果）
  */
uint16_t Bsp_Adc_GetBrakeRaw(void)
{
  return s_last_sample.brake_raw;
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
