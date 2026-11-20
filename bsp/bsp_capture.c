/**
  ******************************************************************************
  * @file    bsp_capture.c
  * @author  Kamingo
  * @brief   踏频脉冲采集 BSP（方案开关见 relay_config.h，接口两方案通用）
  * @date    2026-11-05
  * @version 2.0
  * @note    方案一 CAPTURE_MODE_IC（旧，保留回退）：TIM4_CH1 输入捕获，PB6，
  *          20kHz 计数逐脉冲中断；rpm_x10 = 12,000,000/dt_counts。
  *          方案二 CAPTURE_MODE_GATED_DMA（新，默认）：Why——旧路径每脉冲一次
  *          中断（32 脉冲/转，最高 ~2048 次/s），CPU 与时序抖动都吃了亏；
  *          新路径脉冲由 TIM4 外部时钟模式 1（SMS=111）硬件直接计数，
  *          TIM5 10Hz 门控 UPDATE 触发 DMA1_Stream6/Ch6（TIM5_UP，
  *          RM0090 Rev3 Table 33）把 TIM4->CNT 快照进 16 深环形缓冲，
  *          全链路零中断。How——任务侧（relay_task 2-20ms 轮询）在
  *          GetRpmX10/IsTimeout 里消化新快照：k 窗计数差 Δ 对 k×100ms，
  *          rpm_x10 = Δ×600000/(32×100×k) = Δ×187.5/k；Δ=0 保持上一窗
  *          rpm（与旧路径行为一致，避免稀脉冲时输出在 0/x 间跳动），
  *          2s 无脉冲判超时清零。回退——CADENCE_CAPTURE_MODE 改回
  *          CAPTURE_MODE_IC 重编，接口签名不变，relay_task 不用改。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_capture.h"
#include "tim.h"
#include "main.h"
#include "relay_config.h"
#include <stddef.h>

#if (CADENCE_CAPTURE_MODE == CAPTURE_MODE_GATED_DMA)
/* ==================== 方案二：门控 DMA 零中断计数 ==================== */

/* Private variables ---------------------------------------------------------*/
static DMA_HandleTypeDef s_hdma_snap;             /* TIM5_UP → 环形快照 CNT */
static uint16_t s_snap_buf[CADENCE_SNAP_COUNT];   /* DMA 直接写入，无 cache 一致性问题 */
static volatile uint32_t s_snap_write = 0U;       /* DMA 下一个待写槽位（NDTR 反推） */
static volatile uint32_t s_last_pulse_tick = 0U;  /* 最近一个有脉冲的窗的时刻 */
static volatile uint16_t s_rpm_x10 = 0U;
static volatile uint8_t s_have_pulse = 0U;
static volatile uint32_t s_pulse_count = 0U;      /* 各窗 Δ 累计（兼容调试口径） */

/**
  * @brief 消化新到快照：按写指针差分出 k 个新窗，算 rpm/累计脉冲/脉冲时刻
  * @note  幂等，无新窗时几个时钟周期就返回；仅 relay_task 上下文调用，无并发。
  *       NDTR=剩余传输数 → 写指针 = SNAP_COUNT-NDTR；16 为 2 的幂，取模用位与。
  * @retval None
  */
static void Capture_ProcessSnapshots(void)
{
  uint32_t write_idx = CADENCE_SNAP_COUNT - s_hdma_snap.Instance->NDTR;
  uint32_t k = (write_idx - s_snap_write) & (CADENCE_SNAP_COUNT - 1U);
  uint16_t newest;
  uint16_t prev;
  uint16_t diff;

  if (k == 0U)
  {
    return; /* 没有新窗 */
  }

  newest = s_snap_buf[(write_idx - 1U) & (CADENCE_SNAP_COUNT - 1U)];
  prev   = s_snap_buf[(s_snap_write - 1U) & (CADENCE_SNAP_COUNT - 1U)];
  diff   = (uint16_t)(newest - prev); /* uint16 差分自然消化 CNT 65536 回绕 */

  s_snap_write = write_idx;
  s_pulse_count += diff;

  if (diff != 0U)
  {
    s_have_pulse = 1U;
    s_last_pulse_tick = HAL_GetTick();
    if (diff <= 2047U) /* 物理上限 1.6s 约 105 脉冲；超界防乘法溢出直接丢弃 */
    {
      uint32_t rpm10 = ((uint32_t)diff * RPM_GATED_NUMERATOR) /
                       ((uint32_t)PULSES_PER_REV * CADENCE_WINDOW_MS * k);
      s_rpm_x10 = (rpm10 > CADENCE_RPM_MAX_X10) ? CADENCE_RPM_MAX_X10 : (uint16_t)rpm10;
    }
  }
  /* diff==0：保持上一窗 rpm，2s 超时统一清零（见 IsTimeout） */
}

/**
  * @brief 启动门控 DMA 采集链：TIM5 门控 + DMA 快照 + TIM4 脉冲计数
  * @retval None
  */
void Bsp_Capture_Init(void)
{
  MX_TIM5_Init();
  __HAL_RCC_DMA1_CLK_ENABLE(); /* dma.c 只开了 DMA2（ADC/USART1） */

  s_hdma_snap.Instance = CADENCE_DMA_STREAM;
  s_hdma_snap.Init.Channel = CADENCE_DMA_CHANNEL;        /* TIM5_UP 请求 */
  s_hdma_snap.Init.Direction = DMA_PERIPH_TO_MEMORY;
  s_hdma_snap.Init.PeriphInc = DMA_PINC_DISABLE;
  s_hdma_snap.Init.MemInc = DMA_MINC_ENABLE;             /* 环形逐槽写 */
  s_hdma_snap.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; /* CNT 是 16bit */
  s_hdma_snap.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  s_hdma_snap.Init.Mode = DMA_CIRCULAR;                  /* 到头自动回卷 */
  s_hdma_snap.Init.Priority = DMA_PRIORITY_LOW;          /* 10Hz 半字，无带宽压力 */
  s_hdma_snap.Init.FIFOMode = DMA_FIFOMODE_DISABLE;      /* 直通模式 */
  if (HAL_DMA_Init(&s_hdma_snap) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_LINKDMA(&htim5, hdma[TIM_DMA_ID_UPDATE], s_hdma_snap);

  /* 外设地址指向 TIM4->CNT：触发源（TIM5_UP）与搬运源地址无关，DMA 只管搬 */
  if (HAL_DMA_Start(&s_hdma_snap,
                    (uint32_t)&(CADENCE_CAPTURE_INSTANCE->CNT),
                    (uint32_t)s_snap_buf,
                    CADENCE_SNAP_COUNT) != HAL_OK)
  {
    Error_Handler();
  }

  /* 先开 TIM4 脉冲计数，再让 TIM5 发一次 UG：立即打基线快照进 buf[0]，
   * 之后每 100ms 一窗；s_snap_write 对齐到基线后的写指针。
   * 本 HAL 版本无 __HAL_TIM_GENERATE_EVENT 宏，按其定义直写 EGR.UG */
  __HAL_TIM_ENABLE(&htim4);
  __HAL_TIM_ENABLE_DMA(&htim5, TIM_DMA_UPDATE);
  htim5.Instance->EGR = TIM_EVENTSOURCE_UPDATE; /* 基线快照，NDTR=15 */
  s_snap_write = CADENCE_SNAP_COUNT - s_hdma_snap.Instance->NDTR;
  __HAL_TIM_ENABLE(&htim5); /* 门控窗从此刻起算，100ms 后第一窗 */
}

#else
/* ==================== 方案一：输入捕获逐脉冲中断（回退路径） ==================== */

/* Private variables ---------------------------------------------------------*/
static volatile uint16_t s_last_capture = 0;
static volatile uint32_t s_last_pulse_tick = 0;
static volatile uint16_t s_rpm_x10 = 0;
static volatile uint8_t s_have_last = 0;
static volatile uint32_t s_pulse_count = 0;

/**
  * @brief 启动 TIM4_CH1 输入捕获中断
  * @retval None
  */
void Bsp_Capture_Init(void)
{
  if (HAL_TIM_IC_Start_IT(&htim4, CADENCE_CAPTURE_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 捕获回调（由 HAL_TIM_IRQHandler 分发）
  * @param htim 定时器句柄
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  uint16_t capture;
  uint32_t now_tick;
  uint32_t gap_ms;
  uint16_t dt;

  if ((htim->Instance == CADENCE_CAPTURE_INSTANCE) &&
      (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
  {
    capture = (uint16_t)HAL_TIM_ReadCapturedValue(htim, CADENCE_CAPTURE_CHANNEL);
    now_tick = HAL_GetTick();
    s_pulse_count++;

    if (s_have_last != 0U)
    {
      gap_ms = now_tick - s_last_pulse_tick;
      if (gap_ms <= CADENCE_TIMEOUT_MS)
      {
        dt = (uint16_t)(capture - s_last_capture);
        if (dt > 0U)
        {
          uint32_t rpm10 = RPM_X10_NUMERATOR / ((uint32_t)PULSES_PER_REV * (uint32_t)dt);
          s_rpm_x10 = (rpm10 > CADENCE_RPM_MAX_X10) ? CADENCE_RPM_MAX_X10 : (uint16_t)rpm10;
        }
      }
      else
      {
        /* 间隔超过 2s：旧数据失效，重新同步 */
        s_rpm_x10 = 0;
      }
    }

    s_last_capture = capture;
    s_last_pulse_tick = now_tick;
    s_have_last = 1;
  }
}

#endif /* CADENCE_CAPTURE_MODE */

/* ==================== 公共接口（两方案签名一致） ==================== */

/**
  * @brief 读取 rpm*10；陈旧或超时返回 0
  * @retval rpm*10
  * @note  2026-11-08：不再"保持到 2s 超时"，>CADENCE_STALE_MS 无脉冲即冲刷为 0，
  *        避免下游把旧踏频当当前值（用户实测 PA211 侧数值保持）。
  */
uint16_t Bsp_Capture_GetRpmX10(void)
{
#if (CADENCE_CAPTURE_MODE == CAPTURE_MODE_GATED_DMA)
  Capture_ProcessSnapshots();
#endif
  if (Bsp_Capture_IsTimeout() != 0U)
  {
    return 0;
  }
  if (Bsp_Capture_IsStale() != 0U)
  {
    /* 冲刷旧值：陈旧时不再输出上一窗结果 */
    s_rpm_x10 = 0;
    return 0;
  }
  if (s_rpm_x10 > CADENCE_RPM_MAX_X10)
  {
    s_rpm_x10 = CADENCE_RPM_MAX_X10; /* 物理上限门控 */
  }
  return s_rpm_x10;
}

/**
  * @brief 踏频陈旧判定（>CADENCE_STALE_MS 无新脉冲）
  * @retval 1=陈旧，值不可用
  */
uint8_t Bsp_Capture_IsStale(void)
{
#if (CADENCE_CAPTURE_MODE == CAPTURE_MODE_GATED_DMA)
  if (s_have_pulse == 0U)
  {
    return 1U;
  }
#else
  if (s_have_last == 0U)
  {
    return 1U;
  }
#endif
  return (uint8_t)((HAL_GetTick() - s_last_pulse_tick) > CADENCE_STALE_MS);
}

/**
  * @brief 2s 无脉冲判超时
  * @retval 1=超时/从未收到脉冲
  */
uint8_t Bsp_Capture_IsTimeout(void)
{
#if (CADENCE_CAPTURE_MODE == CAPTURE_MODE_GATED_DMA)
  Capture_ProcessSnapshots();
  if (s_have_pulse == 0U)
  {
    return 1;
  }
  return (uint8_t)((HAL_GetTick() - s_last_pulse_tick) > CADENCE_TIMEOUT_MS);
#else
  if (s_have_last == 0U)
  {
    return 1;
  }
  return (uint8_t)((HAL_GetTick() - s_last_pulse_tick) > CADENCE_TIMEOUT_MS);
#endif
}

/**
  * @brief 累计脉冲数（调试用）
  * @retval 脉冲计数
  */
uint32_t Bsp_Capture_GetPulseCount(void)
{
#if (CADENCE_CAPTURE_MODE == CAPTURE_MODE_GATED_DMA)
  Capture_ProcessSnapshots();
#endif
  return s_pulse_count;
}
