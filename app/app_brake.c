/**
  ******************************************************************************
  * @file    app_brake.c
  * @author  Kamingo
  * @brief   刹车量模块：HAL4395 读取 + 端点校准
  * @date    2026-11-14
  * @version 1.0
  * @note    移植自 rc_remote f8b2e9d(2026-06-11) app/app_throttle.c（HAL4395
  *          时代版）单颗霍尔分支，平台适配：
  *          - 10bit→12bit 端点初值（303/710 ×4 → 300/3600，宏可调）；
  *          - 输出分辨率 100→1000（0.1%，训练特征用）；
  *          - 删 UI/VOFA/发送死区/THROTTLE_FAKE_TEST（中继板无 UI，
  *            刹车量为标记特征恒发，靠 0x04 帧分频控制带宽）；
  *          - 原版 min/max 校准状态机改为串口单点记录（BR1/BR2），
  *            RAM 生效掉电丢——端点自动跟踪待装机有数据后再做。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_brake.h"
#include "relay_config.h"
#include "main.h"     /* HAL_GetTick */
#include <stddef.h>   /* NULL */

/* ------------------------------- 传感器实例 ------------------------------- */
hall_sensor_t hall_brake;          /* HAL4395 */
static uint16_t s_filter_buf[BRAKE_FILTER_SIZE];

/* ------------------------------- 校准请求 ------------------------------- */
/* usart_comm ISR 置请求，Brake_Tick（任务上下文）执行；0=无请求 */
static volatile uint8_t s_cal_req = 0U;

/* ----------------- v2.5 全自动行程校准（忠实移植遥控器原版三件套） -----------------
 * 原版=rc_remote app_throttle.c calib_sm_tick：移动死区判到达 + 端点保持 2s 确认
 * + 10s 超时兜底，全程零按钮。刹车版语义：LO=松开端（大值）保持追踪 max；
 * HI=捏死端（小值）保持追踪 min；apply 判范围 ≥BRAKE_CAL_MIN_SPAN。
 * 口径全为 4 点滑均 filtered（噪声 ±2~4 码）。 */
static uint8_t  s_cal_state = BRAKE_CAL_STATE_IDLE;
static uint16_t s_track_lo_max = 0U;    /* LO 阶段松开端追踪（取 max，filtered） */
static uint16_t s_track_hi_min = 4095U; /* HI 阶段捏死端追踪（取 min，filtered） */
static uint16_t s_backup_min = 0U;      /* apply 前端点备份（cancel 恢复） */
static uint16_t s_backup_max = 0U;
static uint16_t s_cal_phase_raw = 0U;   /* 进入阶段时的参考值（移动检测基准） */
static uint16_t s_cal_lo_raw = 0U;      /* LO 确认值（HI 阶段移动检测基准） */
static uint32_t s_cal_phase_start = 0U; /* 阶段开始 tick（超时用） */
static uint32_t s_cal_hold_start = 0U;  /* 保持计时起点（0xFFFFFFFF=未开始） */
static volatile uint8_t s_cal_event = 0U;       /* 1=事件待发 0x05（relay_task 取走清零） */
static volatile uint8_t s_cal_event_state = 0U; /* 事件帧 state 快照（cal_enter 时刻） */
static volatile uint8_t s_cal_cmd_req = 0U;     /* 状态机命令请求（ISR 置，任务执行） */

static void cal_enter(uint8_t state)
{
  s_cal_state = state;
  s_cal_event_state = state;   /* 快照：apply 后 s_cal_state 回 IDLE，事件帧仍报 DONE_OK */
  s_cal_event = 1U;
}

/* apply：间隔校验后写端点并回 IDLE（事件帧已带 DONE_OK/FAIL 快照） */
static void cal_apply(void)
{
  if ((s_track_lo_max > s_track_hi_min) &&
      ((s_track_lo_max - s_track_hi_min) >= BRAKE_CAL_MIN_SPAN))
  {
    hall_brake.map.adc_max = s_track_lo_max;  /* 松开端（大值） */
    hall_brake.map.adc_min = s_track_hi_min;  /* 捏死端（小值） */
    hall_filter_reset(&hall_brake.filter);
    cal_enter(BRAKE_CAL_STATE_DONE_OK);
  }
  else
  {
    cal_enter(BRAKE_CAL_STATE_FAIL);
  }
  s_cal_state = BRAKE_CAL_STATE_IDLE;
}

/**
  * @brief 刹车模块初始化
  * @note  端点初值来自宏（12bit 口径），装机后 BR1/BR2 覆盖。
  */
void Brake_Init(void)
{
  hall_hw_config_t hw = {
      .adc_channel = 0U,              /* 本板不使用（读数走 Bsp_Adc 刹车通道） */
      .en_ctrl = GPIO_CTRL_NONE,      /* Enable 硬件接地，无固件控制 */
      .power_ctrl = GPIO_CTRL_NONE,
  };

  hall_map_config_t map = {
      .adc_min = BRAKE_MAP_RAW_FULL,  /* 捏死端（小值） */
      .adc_max = BRAKE_MAP_RAW_ZERO,  /* 松开端（大值） */
      .deadzone_low = BRAKE_MAP_DEADZONE,
      .deadzone_high = BRAKE_MAP_DEADZONE,
      .throttle_min = 0U,
      .throttle_max = BRAKE_MAP_OUT_MAX,
      .map_type = HALL_MAP_LINEAR,
      .reverse = true,                /* HAL4395：捏下电压下降，raw 大→输出小 */
  };

  hall_sensor_init(&hall_brake, &hw, &map, s_filter_buf, BRAKE_FILTER_SIZE);
}

/**
  * @brief 采样更新 + 校准命令执行
  * @note  校准记录当前滤波值并清滤波窗（端点跳变后旧窗值污染映射）。
  */
void Brake_Tick(void)
{
  hall_sensor_update(&hall_brake);

  /* 单点标定（BR1/BR2，保留兼容） */
  if (s_cal_req != 0U)
  {
    uint16_t v = hall_brake.data.filtered;
    if (s_cal_req == 1U)
    {
      hall_brake.map.adc_max = v;     /* 松开端（大值） */
    }
    else if (s_cal_req == 2U)
    {
      hall_brake.map.adc_min = v;     /* 捏死端（小值） */
    }
    else
    {
      /* 未知请求，忽略 */
    }
    hall_filter_reset(&hall_brake.filter);
    s_cal_req = 0U;
  }

  /* v2.5 全自动行程校准：命令执行 + 原版三件套（移动死区/保持确认/超时兜底） */
  if (s_cal_cmd_req != 0U)
  {
    uint8_t cmd = s_cal_cmd_req;
    s_cal_cmd_req = 0U;

    if (cmd == BRAKE_CAL_CMD_START)
    {
      s_track_lo_max = 0U;                    /* 重置追踪 */
      s_track_hi_min = 4095U;
      s_backup_min = hall_brake.map.adc_min;  /* 备份现端点（cancel 恢复） */
      s_backup_max = hall_brake.map.adc_max;
      cal_enter(BRAKE_CAL_STATE_LO);
      s_cal_phase_raw = hall_brake.data.filtered;  /* 移动检测基准=进入瞬间值 */
      s_cal_phase_start = HAL_GetTick();
      s_cal_hold_start = 0xFFFFFFFFU;
    }
    else if ((cmd == BRAKE_CAL_CMD_NEXT) && (s_cal_state == BRAKE_CAL_STATE_LO))
    {
      /* 手动切 HI（v2.3 兼容，自动流程不依赖它） */
      s_cal_lo_raw = s_track_lo_max;
      cal_enter(BRAKE_CAL_STATE_HI);
      s_cal_phase_start = HAL_GetTick();
      s_cal_hold_start = 0xFFFFFFFFU;
    }
    else if ((cmd == BRAKE_CAL_CMD_APPLY) && (s_cal_state == BRAKE_CAL_STATE_HI))
    {
      cal_apply();                            /* 手动 apply（v2.3 兼容） */
    }
    else if ((cmd == BRAKE_CAL_CMD_CANCEL) && (s_cal_state != BRAKE_CAL_STATE_IDLE))
    {
      hall_brake.map.adc_min = s_backup_min;  /* 恢复原端点 */
      hall_brake.map.adc_max = s_backup_max;
      hall_filter_reset(&hall_brake.filter);
      cal_enter(BRAKE_CAL_STATE_CANCELLED);
      s_cal_state = BRAKE_CAL_STATE_IDLE;
    }
    else if (cmd == BRAKE_CAL_CMD_QUERY)
    {
      s_cal_event_state = s_cal_state;        /* 查询：按当前态发 0x05 */
      s_cal_event = 1U;
    }
    else
    {
      /* 非法阶段跳转，忽略 */
    }
  }

  /* 原版三件套状态机（50Hz；tick 换算 1 tick=20ms） */
  if (s_cal_state == BRAKE_CAL_STATE_LO)
  {
    uint16_t v = hall_brake.data.filtered;
    uint32_t phase_elapsed = HAL_GetTick() - s_cal_phase_start;

    /* 到达判定：filtered 偏离进入基准 > 死区（用户已在松开端附近活动/稳定） */
    if ((v != s_cal_phase_raw) &&
        (((v > s_cal_phase_raw) ? (v - s_cal_phase_raw) : (s_cal_phase_raw - v)) > BRAKE_CAL_MOVE_DEADZONE))
    {
      if (s_cal_hold_start == 0xFFFFFFFFU) { s_cal_hold_start = HAL_GetTick(); }
      if (v > s_track_lo_max) { s_track_lo_max = v; }

      if ((HAL_GetTick() - s_cal_hold_start) >= BRAKE_CAL_HOLD_MS)
      {
        s_cal_lo_raw = s_track_lo_max;        /* 记 LO 确认值 */
        cal_enter(BRAKE_CAL_STATE_HI);        /* 自动进 HI */
        s_cal_phase_start = HAL_GetTick();
        s_cal_hold_start = 0xFFFFFFFFU;
      }
    }
    else if (phase_elapsed >= BRAKE_CAL_TIMEOUT_MS)
    {
      cal_enter(BRAKE_CAL_STATE_FAIL);        /* 10s 未检测到端点活动 */
      s_cal_state = BRAKE_CAL_STATE_IDLE;
    }
    else
    {
      s_cal_hold_start = 0xFFFFFFFFU;         /* 未在端点区域，重置保持计时 */
    }
  }
  else if (s_cal_state == BRAKE_CAL_STATE_HI)
  {
    uint16_t v = hall_brake.data.filtered;
    uint32_t phase_elapsed = HAL_GetTick() - s_cal_phase_start;

    /* 到达判定：偏离 LO 确认值 > 死区（用户开始捏） */
    uint16_t d = (v > s_cal_lo_raw) ? (uint16_t)(v - s_cal_lo_raw)
                                    : (uint16_t)(s_cal_lo_raw - v);
    if (d > BRAKE_CAL_MOVE_DEADZONE)
    {
      if (s_cal_hold_start == 0xFFFFFFFFU) { s_cal_hold_start = HAL_GetTick(); }
      if (v < s_track_hi_min) { s_track_hi_min = v; }

      if ((HAL_GetTick() - s_cal_hold_start) >= BRAKE_CAL_HOLD_MS)
      {
        cal_apply();                          /* 保持满 2s：自动判范围并 apply */
      }
    }
    else if (phase_elapsed >= BRAKE_CAL_TIMEOUT_MS)
    {
      cal_enter(BRAKE_CAL_STATE_FAIL);
      s_cal_state = BRAKE_CAL_STATE_IDLE;
    }
    else
    {
      s_cal_hold_start = 0xFFFFFFFFU;
    }
  }
  else
  {
    /* 非校准态不追踪 */
  }
}

uint16_t Brake_GetRaw(void)      { return hall_brake.data.raw; }
uint16_t Brake_GetFiltered(void) { return hall_brake.data.filtered; }
uint16_t Brake_GetMapped(void)   { return hall_brake.data.throttle; }

uint8_t Brake_GetSat(void)
{
  /* 与力矩通道同款饱和口径：接近满量程 */
  return (uint8_t)(hall_brake.data.raw >= TORQUE_ADC_SAT_RAW ? 1U : 0U);
}

void Brake_RequestCal(uint8_t kind)
{
  if ((kind == 1U) || (kind == 2U))
  {
    s_cal_req = kind;
  }
}

void Brake_RequestCalCmd(uint8_t cmd)
{
  if ((cmd >= BRAKE_CAL_CMD_START) && (cmd <= BRAKE_CAL_CMD_QUERY))
  {
    s_cal_cmd_req = cmd;
  }
}

uint8_t Brake_GetCalState(void)      { return s_cal_state; }
uint8_t Brake_TakeCalEvent(uint8_t *out_state)
{
  uint8_t has = s_cal_event;
  if (has != 0U)
  {
    if (out_state != NULL) { *out_state = s_cal_event_state; }
    s_cal_event = 0U;
  }
  return has;   /* 事件标志与 state 分离：IDLE 查询应答也是合法事件 */
}

void Brake_GetCalTrack(uint16_t *out_lo_max, uint16_t *out_hi_min)
{
  if (out_lo_max != NULL) { *out_lo_max = s_track_lo_max; }
  if (out_hi_min != NULL) { *out_hi_min = s_track_hi_min; }
}
