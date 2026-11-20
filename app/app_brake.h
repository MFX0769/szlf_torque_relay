/**
  ******************************************************************************
  * @file    app_brake.h
  * @author  Kamingo
  * @brief   刹车量模块接口（HAL4395 读取 + 端点校准）
  * @date    2026-11-14
  * @version 1.0
  * @note    移植自 rc_remote f8b2e9d(2026-06-11) app/app_throttle.c/h
  *          （HAL4395 时代版）。用途=路测数据标记/训练特征（刹车量），
  *          非控制联锁——见踏频仓 knowledge/刹车信号选型_20261113.md。
  *          校准：串口命令 BR1=记录松开端（当前 filtered → adc_max）、
  *          BR2=记录捏死端（→ adc_min）；RAM 生效掉电丢，端点宏为初值。
  ******************************************************************************
  */

#ifndef __APP_BRAKE_H__
#define __APP_BRAKE_H__

#include <stdint.h>
#include "bsp_hall.h"

/*-------------------------------生命周期开始---------------------------------*/

/** @brief 刹车模块初始化：设备层（HAL4395 配置 + 滤波 + 映射端点宏） */
void Brake_Init(void);

/** @brief 采样更新（含校准命令执行），主任务每周期调用 */
void Brake_Tick(void);

/*--------------------------------生命周期结束---------------------------------*/

/*-------------------------------读取接口开始---------------------------------*/

/** @brief 当前未滤波 raw 值 */
uint16_t Brake_GetRaw(void);

/** @brief 当前滤波后 raw 值 */
uint16_t Brake_GetFiltered(void);

/** @brief 当前刹车量映射输出 0..1000（0.1% 分辨率） */
uint16_t Brake_GetMapped(void);

/** @brief 刹车通道饱和标志（1=raw 接近满量程） */
uint8_t  Brake_GetSat(void);

/*-------------------------------读取接口结束---------------------------------*/

/*-----------------------------校准命令开始-----------------------------------*/

/** @brief 校准请求（usart_comm ISR 侧调用，任务侧 Brake_Tick 执行）
 *  @param kind 1=以当前 filtered 记松开端 adc_max（捏下前发）
 *              2=以当前 filtered 记捏死端 adc_min（捏到底时发） */
void Brake_RequestCal(uint8_t kind);

/* ---- v2.3 行程校准状态机（对齐遥控器 LO→HI→DONE，UI 落平板） ----
 * 命令经 Brake_RequestCalCmd 置请求，Brake_Tick 执行；状态变化置事件标志，
 * relay_task 组 cmd=0x05 事件帧上报（UI 同步状态机）。 */
#define BRAKE_CAL_CMD_START             (1U)  /* BRC：进校准，进 LO */
#define BRAKE_CAL_CMD_NEXT              (2U)  /* BRN：LO→HI */
#define BRAKE_CAL_CMD_APPLY             (3U)  /* BRA：HI→apply→DONE */
#define BRAKE_CAL_CMD_CANCEL            (4U)  /* BRX：取消，恢复原端点 */
#define BRAKE_CAL_CMD_QUERY             (5U)  /* BRQ：查状态（回 0x05 当前态） */

/** @brief 状态机命令请求（ISR 侧调用） */
void Brake_RequestCalCmd(uint8_t cmd);

/** @brief 当前校准状态（BRAKE_CAL_STATE_*，供 0x05 组帧） */
uint8_t Brake_GetCalState(void);

/** @brief 取状态机事件（取走即清）
 *  @param out_state 输出 0x05 帧 state 快照（BRAKE_CAL_STATE_*，可为 IDLE）
 *  @retval 1=有事件（out_state 有效，含 state=IDLE 的查询应答）；0=无事件 */
uint8_t Brake_TakeCalEvent(uint8_t *out_state);

/** @brief 校准采集值（LO=松开端追踪/HI=捏死端追踪，仅校准态有意义） */
void Brake_GetCalTrack(uint16_t *out_lo_max, uint16_t *out_hi_min);

/*-----------------------------校准命令结束-----------------------------------*/

/*---------------------------------调试开始-----------------------------------*/

extern hall_sensor_t hall_brake;   /* HAL4395 传感器对象（状态查询用） */

/*---------------------------------调试结束-----------------------------------*/

#endif /* __APP_BRAKE_H__ */
