/**
  ******************************************************************************
  * @file    relay_task.c
  * @author  Kamingo
  * @brief   中转任务：力矩 ADC + 踏频捕获 -> 协议帧 -> USART1 / USB CDC
  * @date    2026-11-01
  * @version 2.0
  * @note    1. 任务周期用 osDelayUntil 绝对延时，避免发送耗时累积漂移；
  *             周期来自当前性能档位（Relay_SetProfile 可切）。
  *          2. ADC 恒满速采样（DMA 循环，块率 ~3.4kHz），档位只改
  *             滤波窗与发送率——"逼上限"的是时间分辨率，采样不留档位差。
  *          3. 换算：torque_mNm = (v_mV - 750) * 1000 / 35，限幅 0..70000。
  *             手册 0.5-80Nm 与 0.75-3.2V/35mV 不自洽（3.2V 只到 70Nm），
  *             常数在 Inc/relay_config.h，可整体标定替换。
  *          4. 双路上行：USART1（TX 环形 DMA 队列）为主，USB CDC 为辅；
  *             两路互不阻塞，各自丢帧计数可观测。
  *          5. 帧内 tx_tick = HAL_GetTick() 低 16 位（1ms），接收方
  *             用本地时基打点即得全程转发延迟（减去串口传输常量后
  *             为纯转发延迟），见 prot_relay.h。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "relay_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "bsp.h"
#include "bsp_usart.h"
#include "usart.h"
#include "usb_comm.h"
#include "usart_comm.h"
#include "filter.h"
#include "prot_relay.h"
#include "relay_config.h"
#include "status_task.h"

/* ============================ 性能档位 ============================ */
typedef struct
{
  uint32_t baud;            /* 波特率 */
  uint16_t period_ms;       /* 发送周期 */
  uint8_t  filter_window;   /* 滑动平均窗 */
  const char *name;
} relay_profile_t;

static const relay_profile_t s_profiles[RELAY_PROFILE_COUNT] =
{
  /* P1 兼容档：对 GD32 现网 */
  { RELAY_PROFILE1_BAUD, RELAY_PROFILE1_PERIOD_MS, RELAY_PROFILE1_FILTER, "P1-50Hz" },
  /* P2 标准档 */
  { RELAY_PROFILE2_BAUD, RELAY_PROFILE2_PERIOD_MS, RELAY_PROFILE2_FILTER, "P2-200Hz" },
  /* P3 极限档：E2 带宽实验 */
  { RELAY_PROFILE3_BAUD, RELAY_PROFILE3_PERIOD_MS, RELAY_PROFILE3_FILTER, "P3-500Hz" },
};

static volatile uint8_t s_profile = 0U; /* 当前档位索引，默认 P1 */

/**
  * @brief 切换档位（含波特率重配；usart_comm 命令层调用）
  * @param idx 0/1/2 = P1/P2/P3
  * @retval 0=成功；1=参数非法
  * @note  切波特率前清空 TX 队列并中止在途发送（丢帧由 seq 对账），
  *        切完重绑 RX（IDLE+DMA）；滤波窗由任务循环检测档位变化后重建。
  */
uint8_t Relay_SetProfile(uint8_t idx)
{
  UART_HandleTypeDef *huart = &huart1;

  if (idx >= RELAY_PROFILE_COUNT)
  {
    return 1U;
  }
  if (idx == s_profile)
  {
    return 0U;
  }

  taskENTER_CRITICAL();
  s_profile = idx;
  taskEXIT_CRITICAL();

  /* 波特率重配：DeInit 停掉收发 DMA（TX 队列状态随 DeInit 复位），
   * 配置完重绑 RX（usart_comm 层保存了缓冲与回调） */
  (void)HAL_UART_DeInit(huart);
  huart->Init.BaudRate = s_profiles[idx].baud;
  if (HAL_UART_Init(huart) != HAL_OK)
  {
    Error_Handler();
  }
  comm_usart_rebind();

  return 0U;
}

uint8_t Relay_GetProfile(void)
{
  return s_profile;
}

const char *Relay_ProfileName(uint8_t idx)
{
  if (idx >= RELAY_PROFILE_COUNT)
  {
    return "?";
  }
  return s_profiles[idx].name;
}

/**
  * @brief 中转任务主体
  * @param argu 未使用
  * @retval None
  */
void relay_task(void const *argu)
{
  uint32_t last_wake = osKernelSysTick();
  uint8_t seq = 0;
  uint8_t frame[RELAY_FRAME_LEN];
  uint16_t frame_len;
  uint8_t  profile = 0U;
  uint8_t  last_profile = 0xFFU;
  bsp_adc_sample_t adc;
  relay_payload_t payload;
  filter_movavg_t torque_filter;

  (void)argu;

  for (;;)
  {
    /* 0. 档位快照；变化时按新窗口重建滤波器（丢窗内旧值可接受） */
    profile = s_profile;
    if (profile != last_profile)
    {
      Filter_MovAvg_Init(&torque_filter, s_profiles[profile].filter_window);
      last_profile = profile;
    }

    /* 1. 采样（没有新 DMA 块时返回上一次结果） */
    (void)Bsp_Adc_GetSample(&adc);

    /* 2. mV 滑动平均 + 换算 */
    {
      int32_t mv_filtered = Filter_MovAvg_Update(&torque_filter, (int32_t)adc.mv);
      int32_t torque_mnm = (mv_filtered - TORQUE_ZERO_MV) * 1000 / TORQUE_MV_PER_NM;

      if (torque_mnm < TORQUE_MIN_MNM)
      {
        torque_mnm = TORQUE_MIN_MNM;
      }
      if (torque_mnm > TORQUE_MAX_MNM)
      {
        torque_mnm = TORQUE_MAX_MNM;
      }
      payload.torque_mnm = torque_mnm;
    }

    /* 3. 踏频与标志位 */
    payload.rpm_x10 = Bsp_Capture_GetRpmX10();
    payload.flags = 0;

    if (Bsp_Capture_IsTimeout() != 0U)
    {
      payload.flags |= RELAY_FLAG_CADENCE_TIMEOUT;
      payload.rpm_x10 = 0;
    }
    else if (payload.rpm_x10 > 0U)
    {
      payload.flags |= RELAY_FLAG_PEDALING;
    }

    if (adc.saturated != 0U)
    {
      payload.flags |= RELAY_FLAG_SATURATED;
    }

    /* 4. 组帧 + 双路发送（串口环形队列 + USB CDC） */
    payload.tx_tick = (uint16_t)(HAL_GetTick() & 0xFFFFU);
    frame_len = Prot_Relay_Pack(frame, seq, &payload);

    if (frame_len == RELAY_FRAME_LEN)
    {
      (void)Uart1_SendPush(frame, frame_len);   /* 主路：满则计数丢弃 */
      if (Usb_IsConnected() != 0U)
      {
        (void)Usb_Send(frame, frame_len);       /* 辅路：PC 收集/调试 */
      }
    }
    seq++;

    /* 5. 软件看门狗位图：relay 任务活着 */
    wdg_user_set_bit(WDG_BIT_TASK_RELAY);

    /* 6. 周期按当前档位（切换后下一轮生效） */
    (void)osDelayUntil(&last_wake, s_profiles[profile].period_ms);
  }
}
