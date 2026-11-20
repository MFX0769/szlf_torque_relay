/**
  ******************************************************************************
  * @file    prot_relay.h
  * @author  Kamingo
  * @brief   中转帧协议（26 版 Protocol 中间层风格）
  * @date    2026-11-01
  * @version 2.0
  * @note    帧格式 v2（16 字节，小端）：
  *          AA 55 | len=11 | cmd=0x01 | seq | torque_mNm(int32)
  *                | rpm_x10(uint16) | flags(uint8) | tx_tick(uint16)
  *                | crc16(2B, 低字节先)
  *          - tx_tick：发送时刻 HAL_GetTick() 低 16 位（1ms 单位，65.5s 回绕）。
  *            接收方用自己的 1ms 时基打点，(rx_local - tx_tick + 65536) % 65536
  *            即"发送->接收"全程延迟；串口传输时间是可预测常量
  *            （16B×10bit/baud），减掉即得纯转发延迟，实现跨设备对齐。
  *          - CRC 覆盖 AA 55 起到 tx_tick（共 14 字节）。
 *
 *          v2.1（2026-11-12，按键去皮）向后兼容扩展：
 *          - cmd=0x01 布局不变，torque_mNm 永远是原始值（去皮不改它）；
 *          - cmd=0x03 校正帧：同 16B 布局，torque_mNm 位置放
 *            corrected = raw - offset（未去皮时 corrected==raw）；
 *          - cmd=0x02 事件帧：23B（len=17），一次性携带去皮四要素：
 *            AA 55 | len=17 | cmd=0x02 | seq | tare_tick(u32)
 *                  | raw_before(i32) | offset_mnm(i32) | corrected(i32)
 *                  | crc16(2B, 覆盖前 21B)；
 *          - flags bit4 = RELAY_FLAG_TARE_EVENT：事件后 1s 内置位（保留位，
 *            老解析器忽略即可）。GD 侧 pa211_link 状态机对 len/cmd 不匹配
 *            的帧逐字节重同步丢弃，新帧不破坏现网解析（pa211_link.c:36）。
 *
 *          v2.2（2026-11-14，刹车霍尔）向后兼容扩展：
 *          - cmd=0x04 刹车帧：14B（len=9），独立 seq（不占主帧/辅帧对账）：
 *            AA 55 | len=9 | cmd=0x04 | seq | brake_raw(u16) | brake_map(u16)
 *                  | flags(u8) | tx_tick(u16) | crc16(2B, 覆盖前 12B)；
 *            brake_raw=滤波后 ADC 码（0..4095）、brake_map=刹车量 0..1000、
 *            flags bit0=饱和。发送分频 BRAKE_SEND_DIV（默认 1=跟主帧 50Hz，
 *            USART1 直连 GD 时改 5）。
 *
 *          v2.3（2026-11-14，刹车行程校准）向后兼容扩展：
 *          - cmd=0x05 校准事件帧：16B（len=11），独立 seq，状态变化/BRQ 查询
 *            时发一帧：
 *            AA 55 | len=11 | cmd=0x05 | seq | state(u8) | raw_min(u16)
 *                  | raw_max(u16) | tick(u32) | crc16(2B, 覆盖前 14B)；
 *            state=BRAKE_CAL_STATE_*（0=IDLE/1=LO/2=HI/3=DONE_OK/4=FAIL/5=CANCELLED），
 *            raw_min=捏死端、raw_max=松开端（apply 后=新端点）。
 *          - 命令（ASCII 入 RX）：BRC=进校准(LO)、BRN=LO→HI、BRA=apply、
 *            BRX=取消、BRQ=查询（回 0x05 当前态）；BR1/BR2 单点记录保留。
 ******************************************************************************
 */

#ifndef __PROT_RELAY_H
#define __PROT_RELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  int32_t  torque_mnm; /* 0..70000 mNm */
  uint16_t rpm_x10;    /* 踏频 * 10，0..65535 */
  uint8_t  flags;      /* bit0 踩踏中, bit1 饱和, bit2 踏频超时, bit3 陈旧,
                          bit4 去皮事件窗（v2.1） */
  uint16_t tx_tick;    /* 发送时刻 1ms tick 低 16 位（观测转发延迟） */
} relay_payload_t;

/** v2.1 去皮事件四要素（cmd=0x02 事件帧负载，一次按压发一帧） */
typedef struct
{
  uint32_t tick_ms;    /* 按压生效时刻 HAL_GetTick()（32bit 全量，1ms） */
  int32_t  raw_before; /* 归零前原始力矩 mNm（限幅后的原始值） */
  int32_t  offset_mnm; /* 新 offset = 按压时刻滤波后原始读数 */
  int32_t  corrected;  /* 归零后校正值 = raw_before - offset_mnm */
} relay_tare_event_t;

/** v2.2 刹车帧负载（cmd=0x04，14B 帧） */
typedef struct
{
  uint16_t brake_raw;  /* 滤波后 ADC 码 0..4095 */
  uint16_t brake_map;  /* 刹车量 0..1000（0.1% 分辨率） */
  uint8_t  flags;      /* bit0 = RELAY_FLAG_BRAKE_SAT（通道饱和） */
  uint16_t tx_tick;    /* 发送时刻 1ms tick 低 16 位（同 0x01 口径） */
} relay_brake_payload_t;

/** v2.3 刹车校准事件负载（cmd=0x05，16B 帧，状态变化/查询时发一帧） */
typedef struct
{
  uint8_t  state;      /* BRAKE_CAL_STATE_*（IDLE/LO/HI/DONE_OK/FAIL/CANCELLED） */
  uint16_t raw_min;    /* 捏死端（apply 后=新 adc_min；LO 阶段为追踪中值或原值） */
  uint16_t raw_max;    /* 松开端（apply 后=新 adc_max） */
  uint32_t tick_ms;    /* 事件时刻 HAL_GetTick()（32bit 全量） */
} relay_cal_event_t;

/* Exported function declarations --------------------------------------------*/
uint16_t Prot_Relay_Pack(uint8_t *buf, uint8_t seq, const relay_payload_t *payload);
uint16_t Prot_Relay_PackCorrected(uint8_t *buf, uint8_t seq,
                                  const relay_payload_t *payload, int32_t corrected_mnm);
uint16_t Prot_Relay_PackTareEvent(uint8_t *buf, uint8_t seq,
                                  const relay_tare_event_t *ev);
uint16_t Prot_Relay_PackBrake(uint8_t *buf, uint8_t seq,
                              const relay_brake_payload_t *payload);
uint16_t Prot_Relay_PackCalEvent(uint8_t *buf, uint8_t seq,
                                 const relay_cal_event_t *ev);
uint8_t Prot_Relay_Verify(const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __PROT_RELAY_H */
