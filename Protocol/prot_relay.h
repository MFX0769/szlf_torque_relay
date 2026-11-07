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
  uint8_t  flags;      /* bit0 踩踏中, bit1 饱和, bit2 踏频超时 */
  uint16_t tx_tick;    /* 发送时刻 1ms tick 低 16 位（观测转发延迟） */
} relay_payload_t;

/* Exported function declarations --------------------------------------------*/
uint16_t Prot_Relay_Pack(uint8_t *buf, uint8_t seq, const relay_payload_t *payload);
uint8_t Prot_Relay_Verify(const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __PROT_RELAY_H */
