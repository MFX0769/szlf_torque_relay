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
#include "app_brake.h"
#include "tx_sched.h"

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

/* 2026-11-10: ISR 侧只置请求；实际切换在 relay_task 里做。
 * 事故链：USART1 RX 回调（ISR）收到 '1'/'2'/'3' → Relay_SetProfile →
 * taskENTER_CRITICAL → vPortEnterCritical 的 configASSERT 死循环，整机停摆
 * （实测 PC=vPortEnterCritical+0x47、LR=Relay_SetProfile+0x1e）。 */
static volatile uint8_t s_pending_profile = 0xFFU;

void Relay_RequestProfile(uint8_t idx)
{
  if (idx < RELAY_PROFILE_COUNT)
  {
    s_pending_profile = idx;
  }
}

/* ============================ 按键去皮（v2.1） ============================ */
/* 2026-11-12：板载 K1(PA0) 按一下 → offset=当前滤波后原始读数；此后
 * cmd=0x01 帧永远发原始值，cmd=0x03 帧发 corrected=raw-offset，
 * 事件帧 cmd=0x02 携带四要素一次发齐，flags bit4 置位 1s 供下游捕边沿。
 * 仅 relay_task 上下文访问（无 ISR 共享），不需要临界区。 */
static int32_t            s_tare_offset_mnm = 0;   /* 0=未去皮 */
static relay_tare_event_t s_tare_ev;               /* 待发事件内容 */
static uint8_t            s_tare_ev_pending = 0U;  /* 1=事件帧待发 */
static uint32_t           s_tare_mark_tick = 0U;   /* bit4 标记窗起点 */
/* v2.1 辅帧（cmd=0x03/0x02）独立序号：GD 侧 pa211_link 只对 cmd=0x01 帧
 * 做 seq 连续对账、辅帧在 len/cmd 校验就被 resync 丢弃（不更新对账状态），
 * 若辅帧消耗主 seq，GD 收到的 0x01 序列会 0,2,4... 全判 gap 丢帧——
 * 因此主 seq 只属于 0x01 帧，辅帧自用 s_aux_seq。 */
static uint8_t            s_aux_seq = 0U;
/* v2.2 刹车帧（cmd=0x04）独立序号：同上纪律，不与 0x01/0x02/0x03 共用。 */
static uint8_t            s_brake_seq = 0U;
/* v2.3 校准事件帧（cmd=0x05）独立序号：同上。 */
static uint8_t            s_cal_seq = 0U;

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

  Brake_Init();   /* v2.2：刹车设备层初始化（任务上下文，ADC DMA 已在跑） */

  for (;;)
  {
    /* 0a. 处理 ISR 挂起的档位请求（任务上下文才允许切波特率） */
    if (s_pending_profile != 0xFFU)
    {
      const uint8_t pending = s_pending_profile;
      s_pending_profile = 0xFFU;
      (void)Relay_SetProfile(pending);
    }

    /* 0. 档位快照；变化时按新窗口重建滤波器（丢窗内旧值可接受） */
    profile = s_profile;
    if (profile != last_profile)
    {
      Filter_MovAvg_Init(&torque_filter, s_profiles[profile].filter_window);
      last_profile = profile;
    }

    /* 1. 采样（没有新 DMA 块时返回上一次结果） */
    (void)Bsp_Adc_GetSample(&adc);

    /* 1b. 刹车采样 + 校准命令执行（v2.2；设备层 4 点滑均在 Brake_Tick 内） */
    Brake_Tick();

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

    /* 2b. 按键去皮：取本周期限幅后的原始值为新 offset（边沿单次触发）。
     *     原始值 payload.torque_mnm 不动；corrected 只进 cmd=0x03 帧。 */
    if (Bsp_Button_PressedEdge() != 0U)
    {
      s_tare_offset_mnm   = payload.torque_mnm;
      s_tare_ev.tick_ms   = HAL_GetTick();
      s_tare_ev.raw_before = payload.torque_mnm;
      s_tare_ev.offset_mnm = s_tare_offset_mnm;
      s_tare_ev.corrected  = payload.torque_mnm - s_tare_offset_mnm;
      s_tare_mark_tick    = s_tare_ev.tick_ms;
      s_tare_ev_pending   = 1U;
    }

    /* 3. 踏频与标志位 */
    payload.rpm_x10 = Bsp_Capture_GetRpmX10();
    payload.flags = 0;

    if (Bsp_Capture_IsTimeout() != 0U)
    {
      payload.flags |= RELAY_FLAG_CADENCE_TIMEOUT;
      payload.rpm_x10 = 0;
    }
    else if (Bsp_Capture_IsStale() != 0U)
    {
      /* >300ms 无新脉冲：值已被 GetRpmX10 冲刷为 0，置陈旧位供下游判别 */
      payload.flags |= RELAY_FLAG_CADENCE_STALE;
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

    /* 3b. 去皮事件窗标记（bit4）：事件后 TARE_EVENT_FLAG_MS 内置位，
     *     无符号回绕安全减法；窗过后自动清零。 */
    if ((s_tare_mark_tick != 0U) &&
        ((HAL_GetTick() - s_tare_mark_tick) < TARE_EVENT_FLAG_MS))
    {
      payload.flags |= RELAY_FLAG_TARE_EVENT;
    }

#if !RELAY_DEBUG_PRINT
    /* 4. 组帧 + 双路发送（串口环形队列 + USB CDC）。
     *    2026-11-12 修正：**单帧**。0x01 的 torque 字段直接放去皮后的值，
     *    不再并行发 0x03 校正帧。
     *    原因（板上实测）：双帧把串口帧率从 54Hz 提到 108Hz，GD 侧
     *    PA211 链路 24% 帧 CRC 失败（13.7s 内 153/631），导致力矩样本
     *    随机缺失、助力明显偏弱。GD 只认 cmd=0x01，0x03 的字节还会
     *    逐字节流过它的同步状态机，可能造成假帧头。
     *    原始值仍可追溯：tq.csv/USB 侧与 0x02 事件帧里都有 raw_before 与
     *    offset；每周期不再重复传输。 */
    payload.torque_mnm = payload.torque_mnm - s_tare_offset_mnm;
    payload.tx_tick = (uint16_t)(HAL_GetTick() & 0xFFFFU);
    frame_len = Prot_Relay_Pack(frame, seq, &payload);

    if (frame_len == RELAY_FRAME_LEN)
    {
      (void)TxSched_PushUart(frame, frame_len);   /* 主路：单一发送任务串行出队（v2.4） */
      if (Usb_IsConnected() != 0U)
      {
        (void)Usb_Send(frame, frame_len);       /* 辅路：PC 收集/调试 */
      }
    }
    seq++;

    if (s_tare_ev_pending != 0U)
    {
      frame_len = Prot_Relay_PackTareEvent(frame, s_aux_seq, &s_tare_ev);
      if (frame_len == RELAY_TARE_FRAME_LEN)
      {
        (void)TxSched_PushUart(frame, frame_len);
        if (Usb_IsConnected() != 0U)
        {
          (void)Usb_Send(frame, frame_len);
        }
      }
      s_tare_ev_pending = 0U;
      s_aux_seq++;
    }

    /* 4b. 刹车辅帧（v2.2，cmd=0x04）：按 BRAKE_SEND_DIV 分频发送，
     *     独立 seq；布局见 prot_relay.h v2.2 段。 */
    if (BRAKE_SEND_DIV > 0U)
    {
      static uint8_t s_brake_div_cnt = 0U;
      s_brake_div_cnt++;
      if (s_brake_div_cnt >= BRAKE_SEND_DIV)
      {
        relay_brake_payload_t bpay;
        s_brake_div_cnt = 0U;
        bpay.brake_raw = Brake_GetFiltered();
        bpay.brake_map = Brake_GetMapped();
        bpay.flags     = (Brake_GetSat() != 0U) ? RELAY_FLAG_BRAKE_SAT : 0U;
        bpay.tx_tick   = (uint16_t)(HAL_GetTick() & 0xFFFFU);
        frame_len = Prot_Relay_PackBrake(frame, s_brake_seq, &bpay);
        if (frame_len == RELAY_BRAKE_FRAME_LEN)
        {
          (void)TxSched_PushUart(frame, frame_len);
          if (Usb_IsConnected() != 0U)
          {
            (void)Usb_Send(frame, frame_len);
          }
        }
        s_brake_seq++;
      }
    }
#endif /* !RELAY_DEBUG_PRINT */

    /* 4c. 校准事件帧（v2.3，cmd=0x05）：状态变化/BRQ 查询时发一帧（两模式均发，
     *     校准 UI 需要在 debug 波形模式下也能收到状态）。 */
    {
      uint8_t cal_state;
      if (Brake_TakeCalEvent(&cal_state) != 0U)
      {
        relay_cal_event_t cev;
        cev.state   = cal_state;
        cev.raw_min = hall_brake.map.adc_min;
        cev.raw_max = hall_brake.map.adc_max;
        cev.tick_ms = HAL_GetTick();
        frame_len = Prot_Relay_PackCalEvent(frame, s_cal_seq, &cev);
        if (frame_len == RELAY_CAL_FRAME_LEN)
        {
          (void)TxSched_PushUart(frame, frame_len);
          if (Usb_IsConnected() != 0U)
          {
            (void)Usb_Send(frame, frame_len);
          }
        }
        s_cal_seq++;
      }
    }

    /* 5. 软件看门狗位图：relay 任务活着 */
    wdg_user_set_bit(WDG_BIT_TASK_RELAY);

    /* 6. 周期按当前档位（切换后下一轮生效） */
    (void)osDelayUntil(&last_wake, s_profiles[profile].period_ms);
  }
}
