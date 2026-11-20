/**
  ******************************************************************************
  * @file    debug_task.c
  * @author  Kamingo
  * @brief   调试打印任务：霍尔原始值 VOFA+ JustFloat 波形输出
  * @date    2026-11-14
  * @version 1.1
  * @note    结构参照云台工程 app/debug_task（RM_Taurus_25 A44A_gimbal），
  *          波形协议对齐 rc_remote drivers/common/debug.c vofa_senddata
  *          （JustFloat：float 小端直发 + 帧尾 00 00 80 7F）。
  *          通道（4 路）：ch0=raw、ch1=filtered、ch2=map(0..1000)、ch3=sat。
  *          通道：经 TxSched_PushUart 入单一发送队列（v2.4），RELAY_DEBUG_PRINT=1
  *          时 relay_task 停发二进制帧，同一条线内容=纯 VOFA 波形，不混流。
  *          标定命令 BR1/BR2 不受影响（RX 独立）。10Hz 波形（100ms/帧）。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "debug_task.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "usb_comm.h"
#include "app_brake.h"
#include "tx_sched.h"
#include <string.h>

/* JustFloat 帧尾：float[4]=16B + tail[4]=20B */
static const uint8_t s_vofa_tail[4] = { 0x00U, 0x00U, 0x80U, 0x7FU };

/**
  * @brief 调试打印任务主体
  * @param argu 未使用
  * @retval None
  */
void debug_task(void const *argu)
{
  static float ch[4];
  static uint8_t frame[20];
  (void)argu;

  for (;;)
  {
    ch[0] = (float)Brake_GetRaw();      /* 原始 ADC 码 0..4095 */
    ch[1] = (float)Brake_GetFiltered(); /* 4 点滑均 */
    ch[2] = (float)Brake_GetMapped();   /* 刹车量 0..1000 */
    ch[3] = (float)Brake_GetSat();      /* 饱和 0/1 */

    memcpy(frame, ch, sizeof(ch));
    memcpy(&frame[sizeof(ch)], s_vofa_tail, sizeof(s_vofa_tail));

    (void)TxSched_PushUart(frame, (uint16_t)sizeof(frame));
    if (Usb_IsConnected() != 0U)
    {
      (void)Usb_Send(frame, (uint16_t)sizeof(frame));
    }
    osDelay(100U);
  }
}
