/**
  ******************************************************************************
  * @file    bsp_button.h
  * @author  Kamingo
  * @brief   板载用户按键 K1（PA0/WK_UP）轮询消抖：去皮触发用
  * @date    2026-11-12
  * @version 1.0
  * @note    DevEBox 板载 K1 接 PA0（WK_UP），按下接 3V3 → 高电平有效；
  *          PA0 原本空置（力矩 ADC 特意避开它，见 relay_config.h 注释），
  *          本模块把它配成输入 + 内部下拉，任务上下文轮询（非中断）。
  ******************************************************************************
  */

#ifndef __BSP_BUTTON_H
#define __BSP_BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @brief 按键 GPIO 初始化（PA0 输入 + 内部下拉；在 Bsp_Init 里调用）
  * @retval None
  */
void Bsp_Button_Init(void);

/**
  * @brief 消抖轮询：检测一次"稳定按下沿"（消费型，每次按压只返回一次 1）
  * @param  None
  * @retval 1=捕获到一次按压沿（调用方处理后即消费）；0=无事件
  * @note   消抖判据：高电平持续 ≥ TARE_DEBOUNCE_MS 才算按下，低电平
  *         持续 ≥ TARE_DEBOUNCE_MS 才重新武装；轮询粒度 = 调用方任务周期
  *         （P1 档 20ms / P3 档 2ms），判定时刻最迟滞后一个轮询周期。
  *         只能在任务上下文调用（内部只读 GPIO 与 tick，无临界区）。
  */
uint8_t Bsp_Button_PressedEdge(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_BUTTON_H */
