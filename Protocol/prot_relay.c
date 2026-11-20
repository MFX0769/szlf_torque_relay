/**
  ******************************************************************************
  * @file    prot_relay.c
  * @author  Kamingo
  * @brief   中转帧打包/校验
  * @date    2026-10-30
  * @version 1.0
  * @note    CRC 直接用 mathlib/crc.c（RM 原文件，保留版权头）：
  *          多项式 0x1021 反射实现（表驱动 0x8408）、初值 0xFFFF、
  *          refin/refout=true、xorout=0，帧内低字节在前。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "prot_relay.h"
#include "relay_config.h"
#include "crc.h"

/**
  * @brief 组一帧力矩/踏频数据
  * @param buf     输出缓冲，至少 RELAY_FRAME_LEN 字节
  * @param seq     帧序号（0..255 回绕）
  * @param payload 负载
  * @retval 帧长（RELAY_FRAME_LEN）
  */
uint16_t Prot_Relay_Pack(uint8_t *buf, uint8_t seq, const relay_payload_t *payload)
{
  uint16_t idx = 0;
  uint16_t crc;
  uint32_t torque;

  if ((buf == 0) || (payload == 0))
  {
    return 0;
  }

  buf[idx++] = RELAY_FRAME_HEAD0;
  buf[idx++] = RELAY_FRAME_HEAD1;
  buf[idx++] = RELAY_FRAME_LEN_FIELD;
  buf[idx++] = RELAY_CMD_TORQUE;
  buf[idx++] = seq;

  torque = (uint32_t)payload->torque_mnm;
  buf[idx++] = (uint8_t)(torque & 0xFFU);
  buf[idx++] = (uint8_t)((torque >> 8) & 0xFFU);
  buf[idx++] = (uint8_t)((torque >> 16) & 0xFFU);
  buf[idx++] = (uint8_t)((torque >> 24) & 0xFFU);

  buf[idx++] = (uint8_t)(payload->rpm_x10 & 0xFFU);
  buf[idx++] = (uint8_t)((payload->rpm_x10 >> 8) & 0xFFU);

  buf[idx++] = payload->flags;

  /* v2：发送时刻 1ms tick 低 16 位（观测转发延迟，见 prot_relay.h） */
  buf[idx++] = (uint8_t)(payload->tx_tick & 0xFFU);
  buf[idx++] = (uint8_t)((payload->tx_tick >> 8) & 0xFFU);

  /* idx = 14：从 AA55 到 tx_tick */
  crc = Get_CRC16_Check_Sum(buf, idx, RELAY_CRC_INIT);
  buf[idx++] = (uint8_t)(crc & 0xFFU);
  buf[idx++] = (uint8_t)((crc >> 8) & 0xFFU);

  return idx;
}

/**
  * @brief 组一帧校正值帧（v2.1，cmd=0x03；布局与 cmd=0x01 完全一致）
  * @param buf           输出缓冲，至少 RELAY_FRAME_LEN 字节
  * @param seq           帧序号（0..255 回绕）
  * @param payload       负载（rpm/flags/tx_tick 原样复用本周期数据帧内容）
  * @param corrected_mnm 校正后力矩 = 原始 - offset（未去皮时 == 原始）
  * @retval 帧长（RELAY_FRAME_LEN）
  */
uint16_t Prot_Relay_PackCorrected(uint8_t *buf, uint8_t seq,
                                  const relay_payload_t *payload, int32_t corrected_mnm)
{
  uint16_t idx = 0;
  uint16_t crc;
  uint32_t torque;

  if ((buf == 0) || (payload == 0))
  {
    return 0;
  }

  buf[idx++] = RELAY_FRAME_HEAD0;
  buf[idx++] = RELAY_FRAME_HEAD1;
  buf[idx++] = RELAY_FRAME_LEN_FIELD;
  buf[idx++] = RELAY_CMD_TORQUE_CORR;
  buf[idx++] = seq;

  torque = (uint32_t)corrected_mnm;
  buf[idx++] = (uint8_t)(torque & 0xFFU);
  buf[idx++] = (uint8_t)((torque >> 8) & 0xFFU);
  buf[idx++] = (uint8_t)((torque >> 16) & 0xFFU);
  buf[idx++] = (uint8_t)((torque >> 24) & 0xFFU);

  buf[idx++] = (uint8_t)(payload->rpm_x10 & 0xFFU);
  buf[idx++] = (uint8_t)((payload->rpm_x10 >> 8) & 0xFFU);

  buf[idx++] = payload->flags;

  buf[idx++] = (uint8_t)(payload->tx_tick & 0xFFU);
  buf[idx++] = (uint8_t)((payload->tx_tick >> 8) & 0xFFU);

  crc = Get_CRC16_Check_Sum(buf, idx, RELAY_CRC_INIT);
  buf[idx++] = (uint8_t)(crc & 0xFFU);
  buf[idx++] = (uint8_t)((crc >> 8) & 0xFFU);

  return idx;
}

/**
  * @brief 组一帧去皮事件帧（v2.1，cmd=0x02；23B，四要素一次发齐）
  * @param buf 输出缓冲，至少 RELAY_TARE_FRAME_LEN 字节
  * @param seq 帧序号（0..255 回绕）
  * @param ev  事件内容（tick/归零前原始值/offset/归零后校正值）
  * @retval 帧长（RELAY_TARE_FRAME_LEN）
  */
uint16_t Prot_Relay_PackTareEvent(uint8_t *buf, uint8_t seq,
                                  const relay_tare_event_t *ev)
{
  uint16_t idx = 0;
  uint16_t crc;
  uint32_t v;

  if ((buf == 0) || (ev == 0))
  {
    return 0;
  }

  buf[idx++] = RELAY_FRAME_HEAD0;
  buf[idx++] = RELAY_FRAME_HEAD1;
  buf[idx++] = RELAY_TARE_LEN_FIELD;
  buf[idx++] = RELAY_CMD_TARE_EVENT;
  buf[idx++] = seq;

  v = ev->tick_ms;
  buf[idx++] = (uint8_t)(v & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 8) & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 16) & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 24) & 0xFFU);

  v = (uint32_t)ev->raw_before;
  buf[idx++] = (uint8_t)(v & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 8) & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 16) & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 24) & 0xFFU);

  v = (uint32_t)ev->offset_mnm;
  buf[idx++] = (uint8_t)(v & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 8) & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 16) & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 24) & 0xFFU);

  v = (uint32_t)ev->corrected;
  buf[idx++] = (uint8_t)(v & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 8) & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 16) & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 24) & 0xFFU);

  /* idx = 21：从 AA55 到 corrected */
  crc = Get_CRC16_Check_Sum(buf, idx, RELAY_CRC_INIT);
  buf[idx++] = (uint8_t)(crc & 0xFFU);
  buf[idx++] = (uint8_t)((crc >> 8) & 0xFFU);

  return idx;
}

/**
  * @brief 组一帧刹车数据（v2.2，cmd=0x04；14B，独立 seq 不占主帧对账）
  * @param buf     输出缓冲，至少 RELAY_BRAKE_FRAME_LEN 字节
  * @param seq     帧序号（0..255 回绕，独立于 0x01/0x02/0x03）
  * @param payload 负载（raw/map/flags/tick）
  * @retval 帧长（RELAY_BRAKE_FRAME_LEN）
  */
uint16_t Prot_Relay_PackBrake(uint8_t *buf, uint8_t seq,
                              const relay_brake_payload_t *payload)
{
  uint16_t idx = 0;
  uint16_t crc;
  uint32_t v;

  if ((buf == 0) || (payload == 0))
  {
    return 0;
  }

  buf[idx++] = RELAY_FRAME_HEAD0;
  buf[idx++] = RELAY_FRAME_HEAD1;
  buf[idx++] = RELAY_BRAKE_LEN_FIELD;
  buf[idx++] = RELAY_CMD_BRAKE;
  buf[idx++] = seq;

  v = payload->brake_raw;
  buf[idx++] = (uint8_t)(v & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 8) & 0xFFU);

  v = payload->brake_map;
  buf[idx++] = (uint8_t)(v & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 8) & 0xFFU);

  buf[idx++] = payload->flags;

  buf[idx++] = (uint8_t)(payload->tx_tick & 0xFFU);
  buf[idx++] = (uint8_t)((payload->tx_tick >> 8) & 0xFFU);

  /* idx = 12：从 AA55 到 tx_tick */
  crc = Get_CRC16_Check_Sum(buf, idx, RELAY_CRC_INIT);
  buf[idx++] = (uint8_t)(crc & 0xFFU);
  buf[idx++] = (uint8_t)((crc >> 8) & 0xFFU);

  return idx;
}

/**
  * @brief 组一帧校准事件（v2.3，cmd=0x05；16B，状态变化/查询时发一帧）
  * @param buf 输出缓冲，至少 RELAY_CAL_FRAME_LEN 字节
  * @param seq 帧序号（独立于 0x01/0x02/0x03/0x04）
  * @param ev  事件内容（state/端点/时刻）
  * @retval 帧长（RELAY_CAL_FRAME_LEN）
  */
uint16_t Prot_Relay_PackCalEvent(uint8_t *buf, uint8_t seq,
                                 const relay_cal_event_t *ev)
{
  uint16_t idx = 0;
  uint16_t crc;
  uint32_t v;

  if ((buf == 0) || (ev == 0))
  {
    return 0;
  }

  buf[idx++] = RELAY_FRAME_HEAD0;
  buf[idx++] = RELAY_FRAME_HEAD1;
  buf[idx++] = RELAY_CAL_LEN_FIELD;
  buf[idx++] = RELAY_CMD_BRAKE_CAL;
  buf[idx++] = seq;

  buf[idx++] = ev->state;

  v = ev->raw_min;
  buf[idx++] = (uint8_t)(v & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 8) & 0xFFU);

  v = ev->raw_max;
  buf[idx++] = (uint8_t)(v & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 8) & 0xFFU);

  v = ev->tick_ms;
  buf[idx++] = (uint8_t)(v & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 8) & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 16) & 0xFFU);
  buf[idx++] = (uint8_t)((v >> 24) & 0xFFU);

  /* idx = 14：从 AA55 到 tick */
  crc = Get_CRC16_Check_Sum(buf, idx, RELAY_CRC_INIT);
  buf[idx++] = (uint8_t)(crc & 0xFFU);
  buf[idx++] = (uint8_t)((crc >> 8) & 0xFFU);

  return idx;
}

/**
  * @brief 校验一帧（供 GD32 侧/串口工具对照，本固件不调用）
  * @param buf 帧
  * @param len 帧长
  * @retval 1=合法
  */
uint8_t Prot_Relay_Verify(const uint8_t *buf, uint16_t len)
{
  uint16_t crc;

  if ((buf == 0) || (len != RELAY_FRAME_LEN))
  {
    return 0;
  }
  if ((buf[0] != RELAY_FRAME_HEAD0) || (buf[1] != RELAY_FRAME_HEAD1))
  {
    return 0;
  }
  if ((buf[2] != RELAY_FRAME_LEN_FIELD) || (buf[3] != RELAY_CMD_TORQUE))
  {
    return 0;
  }

  crc = Get_CRC16_Check_Sum((uint8_t *)buf, RELAY_FRAME_LEN - 2U, RELAY_CRC_INIT);
  if ((buf[RELAY_FRAME_LEN - 2U] != (uint8_t)(crc & 0xFFU)) ||
      (buf[RELAY_FRAME_LEN - 1U] != (uint8_t)((crc >> 8) & 0xFFU)))
  {
    return 0;
  }

  return 1;
}
