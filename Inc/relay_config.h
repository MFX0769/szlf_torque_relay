/**
  ******************************************************************************
  * @file    relay_config.h
  * @author  Kamingo
  * @brief   力矩/踏频中转固件集中配置（引脚、量程、协议、周期）
  * @date    2026-10-30
  * @version 1.0
  ==============================================================================
  * 说明：
  *   1. 本文件是全部"魔数"的唯一入口，改量程/周期/引脚只改这里。
  *   2. 引脚选择依据：DevEBox STM32F4XX_M V2.0 原理图
  *      （develop_log/knowledge/appendices/devebox_f4/STM32F407VX_M_schematics.pdf）。
  *   3. 力矩传感器手册量程 0.5-80Nm、输出 0.75-3.2V 与 35mV/Nm 不自洽：
  *      0.75-3.2V 只有 2.45V 跨度，按 35mV/Nm 只能覆盖 70Nm；
  *      若按 80Nm 反算则应为 30.625mV/Nm。此处按用户给定的 35mV/Nm 实现，
  *      常数做成宏，实测标定后可整体替换。
  ******************************************************************************
  */

#ifndef __RELAY_CONFIG_H
#define __RELAY_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================ 板级 / 时钟 ============================ */
/* DevEBox STM32F407Vx 板载 8MHz 晶振；RM25 原工程为 16MHz，已改。 */
#define RELAY_HSE_VALUE_HZ              (8000000U)

/* ============================ 力矩 ADC ============================ */
/* PA2 = ADC123_IN2，DevEBox 排针引出；PA0/PA1 被板载按键/LED 占用，禁用。 */
#define TORQUE_ADC_INSTANCE             ADC1
#define TORQUE_ADC_CHANNEL              ADC_CHANNEL_2
#define TORQUE_ADC_GPIO_PORT            GPIOA
#define TORQUE_ADC_GPIO_PIN             GPIO_PIN_2

#define TORQUE_ADC_CLOCK_PRESCALER      ADC_CLOCK_SYNC_PCLK_DIV4 /* PCLK2/4 = 21MHz */
#define TORQUE_ADC_SAMPLETIME           ADC_SAMPLETIME_84CYCLES
#define TORQUE_ADC_RESOLUTION_BITS      12U
#define TORQUE_ADC_FULL_SCALE           (4095U)
#define TORQUE_ADC_VREF_MV              (3300U)
#define TORQUE_ADC_BLOCK_SAMPLES        (32U)   /* 块平均点数（16-64 可选） */
#define TORQUE_ADC_DMA_BUF_SAMPLES      (2U * TORQUE_ADC_BLOCK_SAMPLES) /* 半满/全满双块 */
#define TORQUE_ADC_MV(raw)              ((uint32_t)(raw) * TORQUE_ADC_VREF_MV / TORQUE_ADC_FULL_SCALE)
#define TORQUE_ADC_SAT_RAW              (4090U) /* 接近满量程判饱和 */

/* 传感器换算：v_mV = raw*3300/4095; torque_mNm = (v_mV-750)*1000/35 */
#define TORQUE_ZERO_MV                  (750)   /* 0Nm 对应 0.75V */
#define TORQUE_MV_PER_NM                (35)    /* 35mV/Nm */
#define TORQUE_MAX_MNM                  (70000) /* 70Nm 限幅；80Nm 时需改 30.625mV/Nm */
#define TORQUE_MIN_MNM                  (0)
#define TORQUE_SAT_MV                   (3200U) /* 手册上限 3.2V，超过判饱和 */

/* 输出滑动平均窗口（帧级，50Hz 下 8 帧 = 160ms） */
#define TORQUE_FILTER_WINDOW            (8U)   /* 旧值保留：P1 档默认窗 */

/* ============================ 踏频捕获 ============================ */
/* PB6 = TIM4_CH1（AF2）。选它的理由见 README：
 *   - PB6 在 DevEBox 排针上，板载无器件占用；
 *   - TIM4 与 ADC/USART1/FreeRTOS 时基（TIM10）均不冲突；
 *   - 32 脉冲/转、≤64Hz，捕获中断负载可忽略。 */
#define CADENCE_CAPTURE_INSTANCE        TIM4
#define CADENCE_CAPTURE_CHANNEL         TIM_CHANNEL_1
#define CADENCE_CAPTURE_GPIO_PORT       GPIOB
#define CADENCE_CAPTURE_GPIO_PIN        GPIO_PIN_6
#define CADENCE_CAPTURE_GPIO_AF         GPIO_AF2_TIM4

/* ---- 采集方案开关（2026-11-05 升级，回退改这里重编即可，接口不变）----
 * CAPTURE_MODE_IC        ：旧方案。TIM4_CH1 输入捕获，逐脉冲中断测周期
 *                          （32 脉冲/转 × ≤64Hz 脉冲 = 最高 ~2048 中断/s）。
 * CAPTURE_MODE_GATED_DMA ：新方案（默认）。TIM4 外部时钟模式 1（SMS=111，
 *                          TI1FP1 上升沿直接拨 CNT，脉冲路径零中断）+ TIM5
 *                          固定窗门控，TIM5_UP 事件触发 DMA 把 TIM4->CNT
 *                          快照进环形缓冲，任务侧差分算 rpm（10Hz 窗）。
 * DMA 映射依据 RM0090 Rev3 Table 33（DMA1 request mapping，已对原文核实）：
 *   TIM5_UP = DMA1 Stream6 Ch6（备选 Stream0 Ch6）；TIM4_UP = DMA1 Stream6 Ch2。
 *   TIM4 计数器 65536 脉冲才回绕（≈32s@2048pps），UP 事件太稀疏不能当窗，
 *   所以固定窗由独立门控 TIM5 产生，DMA 源地址指向 TIM4->CNT。
 * 定时器占用现状：SysTick=FreeRTOS tick，TIM10=HAL tick，TIM4=脉冲计数，
 *   其余 TIM 全空闲，门控选 TIM5（APB1 定时器时钟 84MHz）。 */
#define CAPTURE_MODE_IC                 (1U)
#define CAPTURE_MODE_GATED_DMA          (2U)
#define CADENCE_CAPTURE_MODE            CAPTURE_MODE_GATED_DMA

#if (CADENCE_CAPTURE_MODE == CAPTURE_MODE_GATED_DMA)
/* 门控定时器：84MHz/8400 = 10kHz 计数，ARR=10*窗ms-1 → 100ms 一次 UPDATE */
#define CADENCE_GATE_INSTANCE           TIM5
#define CADENCE_GATE_PSC                (8400U - 1U)
#define CADENCE_WINDOW_MS               (100U)   /* 固定窗宽度，10Hz 快照 */
#define CADENCE_GATE_ARR                (10U * CADENCE_WINDOW_MS - 1U)
/* TIM5_UP 专用流；DMA1 目前全空闲（ADC/USART1 都在 DMA2），无冲突 */
#define CADENCE_DMA_STREAM              DMA1_Stream6
#define CADENCE_DMA_CHANNEL             DMA_CHANNEL_6
/* 环形快照深度：16 窗 = 1.6s。任务轮询周期 2-20ms，远快于 16 窗，
 * 只有任务停摆 >1.6s 才会丢窗（FreeRTOS 正常调度不会发生）。 */
#define CADENCE_SNAP_COUNT              (16U)
/* rpm_x10 = Δcount × 600000 / (32 × 窗ms × 窗数k)；600000 = 60×10×1000 */
#define RPM_GATED_NUMERATOR             (60U * 10U * 1000U)
#endif

/* TIM4 挂在 APB1 定时器时钟 84MHz（168MHz 系统时钟，APB1 分频 4） */
#define CADENCE_TIMER_SOURCE_HZ         (84000000U)
#define CADENCE_TIMER_CLOCK_HZ          (20000U) /* 20kHz，50us 分辨率 */
#define CADENCE_TIMER_PRESCALER         (CADENCE_TIMER_SOURCE_HZ / CADENCE_TIMER_CLOCK_HZ - 1U)
#define CADENCE_TIMER_PERIOD            (0xFFFFU) /* 16bit，20kHz 下 3.27s 回绕 */
#define CADENCE_IC_FILTER               (0x3U)    /* 4 个采样点滤波 */
#define CADENCE_IRQ_PRIORITY            (6U)

#define PULSES_PER_REV                  (32U)
#define CADENCE_TIMEOUT_MS              (2000U)
/* rpm_x10 = 60*10*f_timer / (32 * dt_counts) = 12,000,000 / dt_counts */
#define RPM_X10_NUMERATOR               (60U * 10U * CADENCE_TIMER_CLOCK_HZ)

/* ============================ USART1 ============================ */
/* PA9=TX / PA10=RX，板丝印 TXD1/RXD1，AF7；115200 8N1。 */
#define RELAY_UART_INSTANCE             USART1
#define RELAY_UART_BAUDRATE             (115200U)
#define RELAY_UART_TX_GPIO_PORT         GPIOA
#define RELAY_UART_TX_GPIO_PIN          GPIO_PIN_9
#define RELAY_UART_RX_GPIO_PORT         GPIOA
#define RELAY_UART_RX_GPIO_PIN          GPIO_PIN_10
#define RELAY_UART_GPIO_AF              GPIO_AF7_USART1
#define RELAY_UART_RX_BUF_SIZE          (32U)
#define RELAY_UART_IRQ_PRIORITY         (5U)
#define RELAY_DMA_IRQ_PRIORITY          (5U)

/* ============================ 协议 / 周期 ============================ */
/* 帧 v2（2026-11-01）：16 字节，加 tx_tick 观测转发延迟。
 * 0:AA 1:55 2:len 3:cmd 4:seq 5-8:torque_mNm(LE) 9-10:rpm_x10(LE)
 * 11:flags 12-13:tx_tick(LE, 1ms 单位回绕) 14-15:crc16(LE, 覆盖 0..13) */
#define RELAY_FRAME_HEAD0               (0xAAU)
#define RELAY_FRAME_HEAD1               (0x55U)
#define RELAY_CMD_TORQUE                (0x01U)
#define RELAY_FRAME_LEN_FIELD           (11U)   /* cmd+seq+torque+rpm+flags+tick */
#define RELAY_FRAME_LEN                 (16U)   /* AA55 + len + 11 + crc16 */
#define RELAY_CRC_INIT                  (0xFFFFU)

/* ============================ 性能档位 ============================ */
/* 三档运行时可切（串口收 ASCII '1'/'2'/'3'）。ADC 恒满速采样
 * （块率 ~3.4kHz），档位只改滤波窗与发送率；波特率必须 ≥ 帧率需求：
 * 16B×10bit/字节，500Hz 需 ≥96k，故 P3 用 921600（F407/GD32 均支持）。 */
#define RELAY_PROFILE_COUNT             (3U)
/* P1 兼容档：对 GD32 现网 50Hz/115200 */
#define RELAY_PROFILE1_BAUD             (115200U)
#define RELAY_PROFILE1_PERIOD_MS        (20U)
#define RELAY_PROFILE1_FILTER           (8U)
/* P2 标准档：200Hz/460800 */
#define RELAY_PROFILE2_BAUD             (460800U)
#define RELAY_PROFILE2_PERIOD_MS        (5U)
#define RELAY_PROFILE2_FILTER           (4U)
/* P3 极限档：500Hz/921600，最小滤波，E2 带宽实验用 */
#define RELAY_PROFILE3_BAUD             (921600U)
#define RELAY_PROFILE3_PERIOD_MS        (2U)
#define RELAY_PROFILE3_FILTER           (2U)
/* 滤波窗口上限（Filter_MovAvg 缓冲按此分配） */
#define TORQUE_FILTER_WINDOW_MAX        (16U)

#define RELAY_FLAG_PEDALING             (0x01U) /* bit0 踩踏中 */
#define RELAY_FLAG_SATURATED            (0x02U) /* bit1 传感器饱和 */
#define RELAY_FLAG_CADENCE_TIMEOUT      (0x04U) /* bit2 踏频超时 */

/* ============================ LED / 看门狗 ============================ */
/* PA1 板载 LED D2，低有效，串联 510R；心跳用。 */
#define RELAY_LED_GPIO_PORT             GPIOA
#define RELAY_LED_GPIO_PIN              GPIO_PIN_1
#define RELAY_LED_ACTIVE_LOW            (1U)

/* IWDG 先不启用，只留接口；置 1 后 status_task 负责喂狗。 */
#define RELAY_USE_IWDG                  (0U)
#define RELAY_IWDG_PRESCALER            IWDG_PRESCALER_32
#define RELAY_IWDG_RELOAD               (500U)

#ifdef __cplusplus
}
#endif

#endif /* __RELAY_CONFIG_H */
