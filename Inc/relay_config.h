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

/* ============================ 调试打印开关（v2.2，2026-11-14） ============================ */
/* 1=同一条 USART1 输出 VOFA+ JustFloat 波形（霍尔原始值调磁铁间距用），
 *   二进制帧停发；0=正常协议帧流（0x01 力矩 50Hz + 0x02 事件 + 0x04 刹车 50Hz）。
 *   一处宏改 0/1 重编即整链切换（freertos.c 任务创建 + relay_task 发帧段同步切换），
 *   两模式互斥不混流。参照云台工程 app/debug_task 模式。2026-11-14 实战验证两种模式。 */
#define RELAY_DEBUG_PRINT               (0U)

/* ============================ 刹车霍尔 ADC（v2.2，2026-11-14） ============================
 * HAL4395 线性霍尔（拆自遥控器扳机+环形磁铁），PA3 = ADC123_IN3，DevEBox 排针引出。
 * 电气口径（手册实测口径见踏频仓 knowledge/刹车信号选型_20261113.md §7）：
 *   零点（松开）≈3.0V → 捏下 ≈0.1V，捏下=电压下降 ⇒ reverse=true（raw 大=刹车量 0）。
 *   Enable 脚硬件接地常开，固件无 GPIO 控制——接线人别悬空（悬空不工作，手册 §5）。
 * 移植源=rc_remote f8b2e9d(2026-06-11) devices/hall/hall_sensor.c + app/app_throttle.c
 * （HAL4395 时代版），10bit→12bit 端点换算，装机后用串口 BR1/BR2 命令覆盖端点。 */
#define BRAKE_ADC_CHANNEL               ADC_CHANNEL_3
#define BRAKE_ADC_GPIO_PORT             GPIOA
#define BRAKE_ADC_GPIO_PIN              GPIO_PIN_3
#define BRAKE_MAP_RAW_FULL              (300U)   /* 捏死端 raw（小值，10bit 303×4 换算初值） */
#define BRAKE_MAP_RAW_ZERO              (3600U)  /* 松开端 raw（大值，10bit 710×4+余量初值） */
#define BRAKE_MAP_DEADZONE              (20U)    /* 双端死区半径（10bit 5×4） */
#define BRAKE_MAP_OUT_MAX               (1000U)  /* 刹车量输出 0..1000（0.1% 分辨率） */
#define BRAKE_FILTER_SIZE               (4U)     /* 设备层滑均窗（同遥控器 4 点） */
/* 上行：cmd=0x04 辅帧、独立 seq（沿 v2.1 辅帧纪律），跟主帧同率发送。
 * 当前 USART1 消费者=平板录制流（COM34 抓取），50Hz×14B 仅占 115200 带宽 ~6%；
 * 若未来 USART1 直连 GD（pa211_link 逐字节状态机），改 5 → 10Hz 控增量——
 * 11-12 前科：GD 链路帧率翻倍（54→108Hz）致 CRC 失败 24%。 */
#define BRAKE_SEND_DIV                  (1U)
#define RELAY_CMD_BRAKE                 (0x04U)
#define RELAY_BRAKE_LEN_FIELD           (9U)     /* cmd+seq+raw+map+flags+tick */
#define RELAY_BRAKE_FRAME_LEN           (14U)    /* AA55+len+9+crc16 */
#define RELAY_FLAG_BRAKE_SAT            (0x01U)  /* 0x04 flags bit0=刹车通道饱和 */

/* ---- 刹车行程校准（v2.5，忠实移植遥控器摇杆校准三件套，全自动零按钮） ----
 * 原版=rc_remote/app/app_throttle.c calib_sm_tick（移动死区+端点保持 2s+超时 10s）。
 * 一键 BRC 后自动走：LO（松开端：raw 稳定判到达→保持 2s 追 max）→ 自动切 HI
 * （捏死端：raw 偏离 LO>死区→保持 2s 追 min）→ 自动 apply 判范围→DONE_OK/FAIL。
 * 各阶段切换发一帧 0x05；BRX 取消恢复备份；BRN/BRA 手动命令保留兼容。
 * 参数依据：本车实测行程仅 ~49 码（11-14 拍板不调几何）、追踪口径=4 点滑均
 * filtered（噪声 ±2~4 码）——死区 15=噪声 4 倍且 <行程 1/3（原版 150 照抄=
 * 永远判不出"动了"，10bit 满量程口径不可移植）；范围门 20=真行程 40%、
 * 假噪声间隔 5 倍；v2.3 的 100 码门槛 >49 行程=必然 FAIL，是 bug（11-19 用户点破）。 */
#define BRAKE_CAL_MOVE_DEADZONE         (15U)    /* 移动检测死区（码，filtered 口径） */
#define BRAKE_CAL_HOLD_MS               (2000U)  /* 端点保持确认时长 */
#define BRAKE_CAL_TIMEOUT_MS            (10000U) /* 单阶段超时（未到端点判 FAIL） */
#define BRAKE_CAL_MIN_SPAN              (20U)    /* apply 合法最小端点间隔（码） */
#define RELAY_CMD_BRAKE_CAL             (0x05U)
#define RELAY_CAL_LEN_FIELD             (11U)    /* cmd+seq+state+min+max+tick */
#define RELAY_CAL_FRAME_LEN             (16U)    /* AA55+len+11+crc16 */
/* 0x05 state 枚举（状态变化即发一帧） */
#define BRAKE_CAL_STATE_IDLE            (0U)
#define BRAKE_CAL_STATE_LO              (1U)     /* 追踪松开端 */
#define BRAKE_CAL_STATE_HI              (2U)     /* 追踪捏死端 */
#define BRAKE_CAL_STATE_DONE_OK         (3U)     /* apply 成功 */
#define BRAKE_CAL_STATE_FAIL            (4U)     /* 端点间隔不足，未写 */
#define BRAKE_CAL_STATE_CANCELLED       (5U)

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
/* PA9=TX / PA10=RX，板丝印 TXD1/RXD1，AF7；8N1。
 * 2026-11-09：对 GD 车机板从 115200 改为 1,000,000 —— GD 的 USART2 控制台
 * 开机固定 HW_UART_BAUD=1M（platform/bsp/hw_kmo.h:249），改中继适配它，
 * 不动飞行板启动路径（GD 侧重复 sdStart 会 chSysHalt，已实机撞过）。 */
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
 * 11:flags 12-13:tx_tick(LE, 1ms 单位回绕) 14-15:crc16(LE, 覆盖 0..13)
 *
 * 帧 v2.1（2026-11-12，按键去皮）：cmd=0x01 布局一字不动（原始力矩），
 *   新增两类帧向后兼容（老解析器按 len/cmd 过滤会安全丢弃，GD 侧
 *   pa211_link 状态机逐字节 resync，不破坏流对齐）：
 *   - cmd=0x03 校正帧：与 0x01 同布局 16B，torque_mNm 位置放
 *     corrected = raw - offset（未去皮 offset=0，corrected==raw）；
 *   - cmd=0x02 事件帧：23B（len=17），一次性携带去皮四要素，
 *     0:AA 1:55 2:len 3:cmd 4:seq 5-8:tare_tick(u32) 9-12:raw_before(i32)
 *     13-16:offset_mnm(i32) 17-20:corrected(i32) 21-22:crc16(覆盖 0..20)。 */
#define RELAY_FRAME_HEAD0               (0xAAU)
#define RELAY_FRAME_HEAD1               (0x55U)
#define RELAY_CMD_TORQUE                (0x01U)
#define RELAY_CMD_TARE_EVENT            (0x02U) /* v2.1：去皮事件帧（一次） */
#define RELAY_CMD_TORQUE_CORR           (0x03U) /* v2.1：校正值帧（每周期） */
#define RELAY_FRAME_LEN_FIELD           (11U)   /* cmd+seq+torque+rpm+flags+tick */
#define RELAY_FRAME_LEN                 (16U)   /* AA55 + len + 11 + crc16 */
#define RELAY_TARE_LEN_FIELD            (17U)   /* seq+tick+raw+offset+corrected */
#define RELAY_TARE_FRAME_LEN            (23U)   /* AA55 + len + cmd + 17 + crc16 */
#define RELAY_CRC_INIT                  (0xFFFFU)

/* ============================ 性能档位 ============================ */
/* 三档运行时可切（串口收 ASCII '1'/'2'/'3'）。ADC 恒满速采样
 * （块率 ~3.4kHz），档位只改滤波窗与发送率；波特率必须 ≥ 帧率需求：
 * 16B×10bit/字节，500Hz 需 ≥96k，故 P3 用 921600（F407/GD32 均支持）。 */
#define RELAY_PROFILE_COUNT             (3U)
/* P1 兼容档：对 GD32 现网 50Hz/1M（2026-11-09 由 115200 改为 1M，适配 GD 控制台） */
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
#define RELAY_FLAG_CADENCE_TIMEOUT      (0x04U) /* bit2 踏频超时（2s 无脉冲） */
#define RELAY_FLAG_CADENCE_STALE        (0x08U) /* bit3 踏频陈旧（>300ms 未更新，值已冲刷） */
#define RELAY_FLAG_TARE_EVENT           (0x10U) /* bit4 去皮事件窗（v2.1，事件后 TARE_EVENT_FLAG_MS 内置位） */

/* 踏频数据时效与限幅（2026-11-08 用户令）：
 * 旧行为是"无新窗就保持上一窗 rpm，直到 2s 超时才清零"，下游会把
 * 一秒多前的旧踏频当当前值用（实测 PA211 侧出现 51.4 反复保持）。
 * 改为：>CADENCE_STALE_MS 无脉冲即冲刷为 0（3 个 100ms 窗），
 * 并对转速做物理上限门控（>200rpm 判无效钳到上限，不参与判据）。 */
#define CADENCE_STALE_MS                (300U)
#define CADENCE_RPM_MAX_X10             (2000U) /* 200rpm 物理上限 */

/* ============================ 去皮按键（v2.1） ============================ */
/* 板载用户按键 K1 = PA0(WK_UP)，按下接 3V3、高电平有效；PA0 此前空置
 * （力矩 ADC 选 PA2 就是为避开它，见上文 ADC 段注释），专用于去皮。
 * offset 取按压生效时刻的滤波后输出（P1 档 8 帧×20ms=160ms 滑动平均窗，
 * P2/P3 档 4/2 帧窗更短——不做中值，说明：滑窗平均即本工程既有滤波）。 */
#define TARE_BTN_GPIO_PORT              GPIOA
#define TARE_BTN_GPIO_PIN               GPIO_PIN_0
#define TARE_DEBOUNCE_MS                (60U)   /* 消抖 ≥50ms，取 60ms 整 */
#define TARE_EVENT_FLAG_MS              (1000U) /* bit4 事件标记窗口 */
#define TARE_OFFSET_MIN_MNM             (TORQUE_MIN_MNM)
#define TARE_OFFSET_MAX_MNM             (TORQUE_MAX_MNM)

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
