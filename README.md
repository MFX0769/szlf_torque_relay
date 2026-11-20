# Kamingo 力矩/踏频中转固件（STM32F407 DevEBox）

> 状态：Keil MDK 编译通过（0 error / 0 warning），**未上板验证**。
>
> 基线：`RM_Taurus_25_Aerial_Robot_f407` 的 Keil/CubeMX 工程壳
> + `taurus_26_-aerial_-robot` 的 BSP / Comm / Protocol 写法
> + 本项目新增的 `bsp_adc` / `bsp_capture` / `relay_task`。

---

## 1. 功能

- ADC1 连续 DMA 采八方力矩传感器模拟电压（PA2 / ADC123_IN2）。
- TIM4_CH1 输入捕获数踏频脉冲（PB6，32 脉冲/转）。
- FreeRTOS 两个任务：`relay_task`（50Hz 采样→换算→组帧→USART1 发送）、
  `status_task`（LED 心跳 + 软件看门狗位图 + 可选 IWDG 喂狗）。
- USART1（PA9/PA10，115200 8N1）向 GD32 PD9 发送 14 字节定长帧，
  帧内带序号，GD32 侧可统计丢帧。
- 无 USB、无 CAN、无 OTA、无 bootloader。

---

## 2. 目录

```
kamingo_torque_relay/
├─ A44A_gimbal.ioc              # CubeMX 配置（手工同步，见 §10）
├─ MDK-ARM/
│  ├─ A44A_gimbal.uvprojx       # Keil 工程（名字沿用 RM25）
│  ├─ A44A_gimbal.uvoptx
│  ├─ startup_stm32f407xx.s     # ARMCC 启动文件（RM25 原样）
│  └─ Objects/
│     ├─ kamingo_torque_relay.axf / .hex / .bin
│     └─ kamingo_torque_relay.map / .sct
├─ Inc/
│  ├─ main.h / stm32f4xx_hal_conf.h / stm32f4xx_it.h
│  ├─ FreeRTOSConfig.h
│  ├─ relay_config.h            # 全部魔数集中入口
│  └─ adc.h / tim.h / usart.h / dma.h / gpio.h / iwdg.h
├─ Src/
│  ├─ main.c / freertos.c / stm32f4xx_it.c
│  ├─ adc.c / tim.c / usart.c / dma.c / gpio.c / iwdg.c
│  ├─ stm32f4xx_hal_msp.c / stm32f4xx_hal_timebase_tim.c
│  └─ system_stm32f4xx.c
├─ bsp/                         # 26 版风格：bsp_xxx + 统一聚合
│  ├─ bsp.c / bsp.h
│  ├─ bsp_adc.c / .h            # 力矩 ADC + 双块平均
│  ├─ bsp_capture.c / .h        # 踏频捕获 + 2s 超时
│  ├─ bsp_usart.c / .h          # IDLE+DMA 收 + DMA 发 + 回调注册
│  ├─ bsp_led.c / .h
│  └─ bsp_iwdg.c / .h
├─ Comm/
│  └─ usart_comm.c / .h         # 26 版 Comm 层：注册 USART1 接收回调
├─ Protocol/
│  └─ prot_relay.c / .h         # 26 版 Protocol 层：组帧 + CRC
├─ app/
│  ├─ relay_task.c / .h         # 50Hz 主任务
│  └─ status_task.c / .h        # 心跳/软件看门狗位图
├─ mathlib/
│  ├─ crc.c / .h                # RM25 原样复制
│  └─ filter.c / .h             # 自写滑动平均
├─ Drivers/
│  ├─ CMSIS/                    # RM25 原样（删 DSP Source）
│  └─ STM32F4xx_HAL_Driver/     # HAL 1.7.6
│     └─ ADC HAL 4 个文件来自 CubeF4 v1.24.1（同 1.7.6）
└─ Middlewares/Third_Party/FreeRTOS/
   └─ Source/                   # 10.3.x，RVDS/ARM_CM4F + heap_4
```

---

## 3. 引脚定案（硬约束）

| 功能 | 引脚 | 复用/模式 | 方向 | 选型理由 |
|---|---|---|---|---|
| 力矩 ADC | **PA2** | ADC123_IN2 / 模拟 | 输入 | 排针引出；板载无占用；避开 PA0/PA1；ADC1 通道 2 |
| 踏频捕获 | **PB6** | TIM4_CH1 / AF2 | 输入 | 原理图明确标 TIM4_CH1；板载无 I2C 器件；TIM4 与 ADC/USART1/TIM10 时基均不冲突 |
| USART1 TX | **PA9** | USART1_TX / AF7 | 输出 | 板丝印 TXD1；接 GD32 PD9 |
| USART1 RX | **PA10** | USART1_RX / AF7 | 输入 | 板丝印 RXD1；可选接 GD32 TX |
| SWDIO | PA13 | SYS_JTMS-SWDIO | 双向 | 调试/烧录 |
| SWCLK | PA14 | SYS_JTCK-SWCLK | 输入 | 调试/烧录 |
| LED | **PA1** | GPIO 推挽 | 输出 | 板载 D2 + 510R，低有效；心跳 |
| 5V | 5V 排针 | 电源 | — | 力矩传感器供电 |
| GND | GND 排针 | 电源 | — | 共地 |

### 3.1 ADC 为什么选 PA2

- PA2 = ADC123_IN2，12bit 可用，排针 J3 引出。
- PA0 = 板载按键 K1（WK_UP，接 3V3），PA1 = 板载 LED D2（510R 负载），
  两者都不能做高阻模拟输入。
- 备选 PC0/PC1/PA4/PA5 也在排针上，但：
  - PA4/PA5 是 DAC/SPI1 复用，且 PA5 与 SPI1_SCK 共用；
  - PC0/PC1 也可用，但本设计只需要一路，PA2 与 USART1/捕获脚距离最远，
    走线最省心。
- 结论：ADC=PA2；PC0/PC1/PA4/PA5 作为备选，改 `Inc/relay_config.h` 即可。

### 3.2 踏频为什么选 PB6（不是 PC6/PA6）

- **PB6 = TIM4_CH1**：DevEBox 原理图引脚名直接写
  `DCMI_D5/I2C1_CLK/TIM4_CH1/PB6`，复用关系没有歧义。
- **PC6**：原理图标注的是 `PC6/TIM8_CH1/...`；虽然 F407 的 AF2 也可把
  PC6 映射为 TIM3_CH1，但原理图未直接写 TIM3_CH1，需要再查 AF 表，
  增加一次核对成本。本设计不选它。
- **PA6**：是 TIM3_CH1 的默认脚，但同时也是 SPI1_MISO 的复用脚；
  DevEBox 的 SPI Flash 用的是 PB4 作 MISO，PA6 实际空闲，但为避免
  与 SPI 复用混淆，不选。
- TIM4 是 APB1 定时器（84MHz），与 HAL 时基 TIM10、FreeRTOS SysTick、
  ADC1/USART1 都不冲突；16bit 计数器在 20kHz 下 3.27s 回绕，
  大于 2s 踏频超时窗口，软件不需要处理溢出。
- **为什么不用 EXTI 兜底**：EXTI 只能拿软件时间戳，中断延迟和任务调度
  抖动会直接变成转速误差；TIM 输入捕获是硬件锁存，20kHz 计数下量化
  误差 50us，对 ≤64Hz 脉冲足够。EXTI 只在“定时器全被占用”时才考虑。

### 3.3 原理图核对结论

依据：
`develop_log/knowledge/appendices/devebox_f4/STM32F407VX_M_schematics.pdf`
（page 1）与同目录 `README_F407VGT6.md`、`pins_F407VGT6.csv`。

- PDF 引脚表明确出现：`PA2/ADC123_IN2/USART2_TX/TIM2_CH3/TIM5_CH3`、
  `...TIM4_CH1/PB6...`、`PA9/USART1_TX/TIM1_CH2`、
  `PA10/USART1_RX/TIM1_CH2`、`PA13/JTMS/SWDIO`、`PA14/JTCK/SWCLK`。
- PDF 的 LED 电路：`D2 LED` + `R14 510R` 接 PA1，低电平点亮。
- PDF 的按键：PA0/WK_UP（K1 接 3V3）。
- `pins_F407VGT6.csv` 列出 PA2/PB6/PA9/PA10/PA13/PA14/PA1 均在排针上。
- 结论：PA2、PB6、PA9、PA10、PA1、PA13/PA14 均引出且无板载冲突；
  PA0/PA1 不作为 ADC 输入。

---

## 4. 接线

### 4.1 八方力矩传感器（4 线）

| 传感器线 | 接到 | 说明 |
|---|---|---|
| 棕 | **PA2** | 模拟电压 0.75–3.2V（手册），0.75V=0Nm |
| 绿 | **PB6** | 踏频脉冲，32 脉冲/转；内部上拉已开 |
| 橙 | **5V** | 传感器供电（不是 3.3V） |
| 黑 | **GND** | 与 STM32/GD32 共地 |
| 白/紫 | 悬空 | 绝缘包好，不接 |

注意事项：

1. **先量再接**：上电前用万用表确认传感器输出不超过 3.3V。
   PA2 是 3.3V ADC 输入，超过 VDDA 会损伤 IO。
2. 手册写 0.75–3.2V 对应 0.5–80Nm，但 0.75–3.2V 只有 2.45V 跨度：
   - 按 35mV/Nm：2.45V / 0.035 = **70Nm**（本固件按此实现，限幅 70Nm）；
   - 按 80Nm：2.45V / 80 = **30.625mV/Nm**。
   两者不自洽。若实测 80Nm 时输出接近 3.55V，则**不能直接进 PA2**，
   必须加分压/钳位；届时改 `TORQUE_MV_PER_NM` 与 `TORQUE_MAX_MNM`。
3. 若绿色线是开集输出，内部上拉约 40k 可能偏弱；长线/高频时可外加
   4.7k–10k 上拉到 3.3V。
4. 信号地与功率地单点共地，避免 5V 回流压降叠加到 ADC 地。

### 4.2 与 GD32 对接

| STM32 侧 | GD32 侧 | 说明 |
|---|---|---|
| PA9（USART1_TX，3.3V TTL） | PD9（RX） | 单向数据：STM32 → GD32 |
| PA10（USART1_RX） | GD32 TX（可选） | 若 GD32 需要发命令/心跳 |
| GND | GND | 必须共地 |

- 115200 8N1，无硬件流控。
- GD32 PD9 若是 5V 容忍脚，也建议按 3.3V 电平设计；不要接 5V 到 PA9。
- 若只做单向发送，PA10 可悬空；但固件已开 IDLE+DMA 接收，接上更利于
  后续加命令/心跳。

### 4.3 USB 4 芯线颜色陷阱

- 本设计**不用 USB 数据**（PA11/PA12 留空），USB 口只可能用于 5V 供电。
- 廉价 USB 线的线色不统一：红/黑可能对调，绿/白也可能对调。
  **严禁按颜色直接对接**；先量通断（USB A 口 VBUS/GND 对应哪两根），
  或直接用板子的 5V/GND 排针供电。
- 若用 USB 给板子供电，不要再把 USB 的 D+/D- 接到任何传感器/串口。

---

## 5. 构建（Keil MDK，无 GCC/Makefile）

> 按最新指令：保持 RM 工作流，**不新建 GCC/Makefile 工程**。

环境（本机已验证）：

- Keil MDK 5.40（`D:\Software\Keil\Keil_v5\UV4\UV4.exe`）
- ARMCC 5.06 update 7（`D:\Software\Keil\Keil_v5\ARM\ARM_Compiler_5.06u7\Bin`）
- 器件包：Keil.STM32F4xx_DFP（本机实际解析到 3.1.1）

命令行构建：

```bat
"D:\Software\Keil\Keil_v5\UV4\UV4.exe" -b ^
  "D:\Kamingo\kamingo_torque_relay\MDK-ARM\A44A_gimbal.uvprojx" ^
  -j0 -o "D:\Kamingo\kamingo_torque_relay\MDK-ARM\build.log"
```

本次结果（2026-10-30）：

```
Program Size: Code=21564 RO-data=1308 RW-data=176 ZI-data=19864
".\Objects\kamingo_torque_relay.axf" - 0 Error(s), 0 Warning(s).
```

产物：

| 文件 | 大小 | 用途 |
|---|---|---|
| `MDK-ARM/Objects/kamingo_torque_relay.axf` | 1,110,004 B | Keil 调试/下载（ELF） |
| `MDK-ARM/Objects/kamingo_torque_relay.hex` | 64,880 B | 通用烧录 |
| `MDK-ARM/Objects/kamingo_torque_relay.bin` | **23,048 B** | 裸 bin（fromelf 生成） |
| `MDK-ARM/Objects/kamingo_torque_relay.map` | 283,006 B | 符号/尺寸 |

- 目标器件是 **STM32F407VETx**（512KB Flash / 128KB RAM + 64KB CCM），
  与 RM25 工程一致；DevEBox F407VGT6 可直接运行（只用到前 512KB/128KB）。
- 若换成 VGT6 器件包，Keil 里改 Device 即可，代码不用动。
- `清理编译文件.bat` 会删除 axf/hex/map 等产物，日常不要误运行。

---

## 6. 协议（STM32 → GD32 PD9）

固定 14 字节，小端：

| 偏移 | 长度 | 字段 | 说明 |
|---|---|---|---|
| 0 | 2 | `AA 55` | 帧头 |
| 2 | 1 | `len` | 固定 9：`cmd` 到 `flags` 的字节数 |
| 3 | 1 | `cmd` | 固定 `0x01`（力矩/踏频） |
| 4 | 1 | `seq` | 帧序号，0..255 回绕 |
| 5 | 4 | `torque_mNm` | int32 LE，0..70000 |
| 9 | 2 | `rpm_x10` | uint16 LE，踏频×10 |
| 11 | 1 | `flags` | bit0=踩踏中，bit1=饱和，bit2=踏频超时 |
| 12 | 2 | `crc16` | 低字节在前 |

### 6.1 CRC 参数（必须与 GD32 一致）

`mathlib/crc.c` 是 RM25 原文件（未改），实际参数：

- 多项式：`0x1021` 的反射实现（表驱动 `0x8408`）
- 初值：`0xFFFF`
- 输入/输出反射：是
- 结果异或：`0x0000`
- 计算范围：从 `AA 55` 到 `flags`（12 字节）
- 帧内顺序：**低字节先**

> 注意：这不是常见的 CCITT-FALSE（0x1021 + init 0xFFFF + 高位先），
> 而是 RM 裁判系统同款反射 CRC16。GD32 侧若用标准库的 CCITT-FALSE，
> 会全部校验失败；请按上面参数实现。

### 6.2 示例帧

`seq=0, torque=12345mNm, rpm_x10=900, flags=0x01`：

```
AA 55 09 01 00 39 30 00 00 84 03 01 BE A9
```

CRC 计算结果 `0xA9BE`，低字节 `BE` 在前。

> 注意：本节上文是 v1（14B）旧描述；**实际帧格式以代码为准**
> （`Protocol/prot_relay.h` v2：16B 含 `tx_tick`，`relay_config.h` 协议段）。

### 6.2.1 协议 v2.1：按键去皮扩展（2026-11-12）

板上用户按键 K1（PA0/WK_UP）按一下 → 中继板把当时滤波后的原始力矩读数
记为新 `offset`；此后**cmd=0x01 帧布局一字不动，`torque_mNm` 永远是原始值**。
校正值与事件走新增帧，向后兼容：

| cmd | 帧长 | 用途 | 内容 |
|---|---|---|---|
| `0x01` | 16B | 原始帧（不变） | `torque_mNm` = 原始值，字段布局同 v2 |
| `0x03` | 16B | 校正帧（新增） | 布局与 0x01 完全一致，`torque_mNm` 位置放 `corrected = raw - offset`；未去皮时 offset=0、corrected==raw |
| `0x02` | 23B | 去皮事件帧（新增，每次按压 1 帧） | `AA 55 \| len=17 \| cmd=0x02 \| seq \| tare_tick(u32 LE) \| raw_before(i32) \| offset_mnm(i32) \| corrected(i32) \| crc16`（CRC 覆盖前 21B，参数同 §6.1） |

- `flags` 新增 **bit4 = `RELAY_FLAG_TARE_EVENT`（0x10）**：去皮事件后 1000ms
  内，0x01/0x03 帧都置位，供下游捕边沿；老解析器按保留位忽略即可。
- `seq` 约定：**主 seq 只属于 cmd=0x01 帧**（对 GD 现网对账保持连续）；
  0x03/0x02 辅帧用独立序号（互不消耗）。GD 侧 `pa211_link.c` 状态机对
  len/cmd 不匹配的帧逐字节重同步丢弃且不更新对账状态——若辅帧消耗
  主 seq，GD 会把 0x01 序列判成 0,2,4... 全部 gap 丢帧，故必须分离。
- 带宽：稳态 2×16B×帧率；P1 档（50Hz/115200）=1600B/s ≈ 14% 占用。
- 取舍：offset 数值不进 0x01 帧（16B 无空位，改布局会破坏现网），
  上位机如需持续校正值直接解析 0x03 帧即可；GD 侧现网只见
  `flags bit4`（`pa211_link.h` 对 flags 原样透传不解释）。
- offset 不持久化：断电重启后 offset=0（corrected==raw），再按一次即恢复。

### 6.3 发送节奏与丢帧统计

- 50Hz（20ms 一帧），14B/帧 → 700B/s，115200 余量很大。
- `seq` 每帧 +1，即使本次因 UART 忙跳过发送也 +1；
  GD32 用 `seq` 连续性即可统计丢帧。
- 上电第一帧 `seq=0`；`torque` 在 ADC 首块就绪前为 0，属正常。

---

## 7. 固件行为与关键参数

集中在 `Inc/relay_config.h`，改参数只改这一处。

### 7.1 力矩 ADC

- ADC1，PA2，12bit，右对齐，连续转换，DMA2 Stream0 循环。
- ADC 时钟 = PCLK2/4 = 84MHz/4 = **21MHz**（F407 上限 36MHz）。
- 采样 84 + 转换 12 = 96 周期 → 约 **218.75ksps**。
- DMA 缓冲 2×32 点：半满=块 0，全满=块 1；任务取最新就绪块，
  32 点平均后 `v_mV = raw*3300/4095`。
- 任务级再做一个 8 帧滑动平均（160ms）。

### 7.2 力矩换算

```c
torque_mNm = (v_mV - TORQUE_ZERO_MV) * 1000 / TORQUE_MV_PER_NM;
// 750mV=0Nm，35mV/Nm，限幅 0..70000mNm
```

- 饱和判据：`raw >= 4090` 或 `v_mV >= 3200` → flags bit1。
- 手册 0.5–80Nm 与 0.75–3.2V/35mV 不自洽，见 §4.1；
  常数可整体标定替换。

### 7.3 踏频

- TIM4_CH1，PB6，上升沿，IC 滤波 0x3，20kHz 计数（Prescaler=4199）。
- 32 脉冲/转：`rpm_x10 = 60*10*20000 / (32*dt) = 12,000,000 / dt`。
- 2s 无脉冲 → rpm=0，flags bit2=1。
- 16bit 计数 3.27s 回绕 > 2s 超时窗口，正常工况不会跨回绕计算。

### 7.4 RTOS 任务

| 任务 | 优先级 | 栈（osThreadDef） | 周期 | 职责 |
|---|---|---|---|---|
| `relay_task` | AboveNormal | 512 | 20ms | 采样/换算/组帧/发送 |
| `status_task` | Normal | 256 | 100ms | LED 500ms 翻转、看门狗位图、可选喂狗 |

- FreeRTOS 10.3.x，CMSIS-RTOS v1（`osThreadDef`/`osThreadCreate`），
  1ms tick，heap_4，heap 15360B。
- `relay_task` 用 `osDelayUntil` 绝对延时，避免发送耗时累积漂移。
- `status_task` 每 100ms 检查 `WDG_BIT_TASK_RELAY` 和
  `WDG_BIT_BSP_RELAY_UART`，然后清空位图；relay 位正常才喂 IWDG。
- IWDG 默认关闭（`RELAY_USE_IWDG=0`）；打开后 500ms 超时，
  由 `Bsp_Iwdg_Init/Refresh` 接管。

### 7.5 USART1 接收

- IDLE + DMA 接收，32B 缓冲；IDLE 中断里 `HAL_UART_AbortReceive`
  只停接收、不影响 DMA 发送，取 NDTR 算长度后回调，再重启接收。
- `Comm/usart_comm.c` 注册回调：收到任意字节即置
  `WDG_BIT_BSP_RELAY_UART`，命令解析预留。
- 发送用 `HAL_UART_Transmit_DMA`；发前检查 `huart1.gState == READY`，
  忙则跳过本帧（seq 仍递增）。

---

## 8. 与 RM25 的剪裁清单（文件级）

### 8.1 删除（业务/外设）

- `app/`：gimbal_task、shoot_task、modeswitch_task、debug_task、
  air_log_task、can_send_task、vision_send_task、judgesend_task
- `bsp/`：bsp_can、bsp_FricMotor、bsp_J4310、bsp_LK_Motor_MG4005、
  bsp_judge、bsp_judgesend、bsp_new_rc、bsp_vision、vision_predict、
  bsp_ws2812、bsp_A3_imu、bsp_T_imu、bsp_TriggerMotor、bsp_CoverServo、
  bsp_powerlimit、bsp_supercap、bsp_log、bsp_debug（含所有 `.orig`）
- `mathlib/`：KalmanFilter、DataScope_DP、data_fifo、data_processing、
  define_printf、func_generator、math_calcu、math_process、mytype、pid、
  printf、remote_msg、ubf、us_tim
- `control/`（control_def.h）、`dsp/`（ARM_MATH 库+头）、
  `Drivers/CMSIS/Source/` 与 `Drivers/Source/`（DSP C 源）、`History/`
- `Src/`：can.c、spi.c、usb_otg.c
- `Inc/`：can.h、spi.h、usb_otg.h
- HAL：stm32f4xx_hal_can.c、stm32f4xx_hal_spi.c、
  stm32f4xx_hal_pcd.c、stm32f4xx_hal_pcd_ex.c、stm32f4xx_ll_usb.c
  及对应头文件
- Keil 工程里的 DSP lib、CAN/SPI/USB 分组、旧的 `rm_main` 输出

### 8.2 保留（骨架）

- Keil 工程壳、`.ioc`、`startup_stm32f407xx.s`、
  `system_stm32f4xx.c`、`stm32f4xx_hal_timebase_tim.c`（TIM10 时基）
- HAL 1.7.6 的 core/dma/gpio/rcc/tim/uart/iwdg/flash/pwr/cortex/exti
- FreeRTOS 10.3.x（RVDS ARM_CM4F + heap_4 + CMSIS_RTOS）
- `mathlib/crc.c/h`（原样）
- `app/status_task.c/h`、`bsp/bsp_usart.c/h`（重写为 26 版接口）

### 8.3 新增

- `Src/adc.c`、`Inc/adc.h`（ADC1 + DMA2 Stream0，CubeMX 风格）
- `bsp/bsp_adc.*`、`bsp/bsp_capture.*`、`bsp/bsp_led.*`、
  `bsp/bsp_iwdg.*`、`bsp/bsp.c/h`
- `Comm/usart_comm.*`、`Protocol/prot_relay.*`
- `app/relay_task.*`
- `mathlib/filter.*`
- `Inc/relay_config.h`
- HAL ADC 4 个文件（来自 CubeF4 v1.24.1，同 HAL 1.7.6）
- `README.md`

### 8.4 Keil 工程

`MDK-ARM/A44A_gimbal.uvprojx` 的 `<Groups>` 已重写为：

```
Application/MDK-ARM
Drivers/CMSIS
Application/User
Application/Bsp
Application/AppCtrl
Application/Comm
Application/Protocol
Application/MathLib
Drivers/STM32F4xx_HAL_Driver
Middlewares/FreeRTOS
::CMSIS
```

Include path 已去掉 `control`、`dsp/Include`，加入 `Protocol`、`Comm`；
Define 去掉了 `ARM_MATH_CM4` 和重复项，保留
`USE_HAL_DRIVER,STM32F407xx`；输出名改为
`kamingo_torque_relay`，输出目录 `MDK-ARM/Objects/`。

---

## 9. 26 版 vs 25 版：差异与采纳

| 维度 | RM25（F407） | Taurus26（H723） | 本项目 |
|---|---|---|---|
| MCU/HAL | STM32F407，F4 HAL 1.7.6 | STM32H723，H7 HAL | **保留 25 的 F407/F4 HAL** |
| RTOS | FreeRTOS CMSIS-RTOS v1 | FreeRTOS CMSIS-RTOS v1 | 保留 25 的 F4 port；任务模型照 26 |
| BSP 串口 | `USER_UART_Init/IRQHandler` + 全局分支 | `usart_user_rx_handle_t` + 回调注册 | **采纳 26**（`bsp/bsp_usart.*`） |
| BSP 聚合 | 无 | `Bsp/bsp.c` + `Bsp/bsp.h` | **采纳 26**（`bsp/bsp.c/h`） |
| Comm 层 | 混在 bsp_usart 里 | `Comm/usart_comm.c` 注册回调 | **采纳 26**（`Comm/usart_comm.*`） |
| Protocol 层 | 散在各 bsp/app | `Protocol/prot_*.c/h` | **采纳 26**（`Protocol/prot_relay.*`） |
| 状态/看门狗 | 位图 + status_task | 位图 + status_task | 保留 25 骨架，按 26 精简 |
| 数据池 | 无 | `System/system_datapool.*` | 未单独建；资源宏集中到 `relay_config.h` + `Comm` |
| 数学层 | `mathlib/`（crc/pid/ubf…） | `Math/`（DSP/OSQP） | 保留 25 的 `mathlib/crc`，加 `filter` |
| H7 专有 | — | `.RAM_D1`、FDCAN、USB | 不采纳；F407 无 D1 域 |

### 9.1 采纳的 26 写法（file:line 证据）

- 回调式串口句柄：`Bsp/bsp_usart.h:38`（`usart_call_back`）、
  `:48-49`（`usart_user_rx_handle_t`）、`:73`（`bsp_user_usart_Init`）、
  `:75`（`bsp_user_usart_Receive_Handler`）、`:77`（`Uart1_Transmit`）。
- IDLE 处理：`Bsp/bsp_usart.c:183` 用 `HAL_UART_AbortReceive`（只停收，
  不影响发），`:187` 用 `DMA_Stream_TypeDef->NDTR` 算长度。
- BSP 聚合：`Bsp/bsp.c:13` 只 `#include "bsp.h"`，由 `bsp.h:22-23`
  汇总模块头；本项目对应 `bsp/bsp.c` + `bsp/bsp.h`。
- 状态位图：`Application/status_task.c:54`（`wdg_event_group`）、
  `:82`（`status_t status`）、`Application/status_task.h:10-47`
  （`WDG_BIT_*` + `status_t` + `wdg_user_set_bit`）。
- Comm 注册：`Comm/usart_comm.c:54-56`（`comm_usart_init` 调
  `bsp_user_usart_Init`）、`:146`（USART1 回调）。
- 任务模型：`Core/Src/freertos.c:132-198`（`MX_FREERTOS_Init` +
  `osThreadDef`/`osThreadCreate`）。
- 文件头风格：`Bsp/bsp_usart.c:1-12`（`@file/@author/@brief/@date/@version`）。

### 9.2 26 与 25 的冲突点

- **MCU 不同**：26 是 H7，25 是 F4。HAL/CMSIS/启动文件不能混用；
  本项目以 25 的 F407 工程壳为准，只借 26 的软件分层和接口。
- **RTOS**：两者都是 FreeRTOS + CMSIS-RTOS v1，无冲突；保留 25 的
  F4 RVDS port。
- **H7 专有写法**：26 的 `.RAM_D1` 段、FDCAN、USB 设备库在 F407 上
  不存在/不需要，未采纳。
- **目录名**：26 用 `Application/Bsp/Comm/Protocol/System/Math`；
  25 用 `app/bsp/mathlib`。本项目以 25 的 Keil 工程壳为准，
  保留 `app/`、`bsp/`、`mathlib/`，新增 `Comm/`、`Protocol/`。

---

## 10. 与 RM 的有意偏离

1. **中文注释统一 UTF-8**：RM25 是 GBK；GCC/Keil 混用 GBK 有编码风险。
   本工程全部新建/重写文件用 UTF-8，Keil ARMCC 5.06 实测 0 warning。
   `mathlib/crc.c/h` 是 RM 原文件，保持原样（无中文注释）。
2. **ADC HAL 是新增的**：RM25 工程原本没有 ADC 模块。为保证 HAL 版本
   一致，ADC 的 4 个文件取自 STM32CubeF4 **v1.24.1**（同 HAL 1.7.6），
   不是从更新的 1.8.x 包复制。
3. **`.ioc` 手工同步**：`A44A_gimbal.ioc` 已按本项目引脚/外设改写，
   但**没有用 CubeMX 重新生成代码**（本机 CubeMX 6.15 的无头加载未
   成功验证）。`.mxproject` 已删除，CubeMX 首次打开会重建。
   为降低再生风险，`.ioc` **故意不列 FreeRTOS 任务**（任务在
   `Src/freertos.c` 的 USER CODE 段，CubeMX 再生时会保留，不会重复
   建任务），也**不启用 IWDG**（IWDG 一旦启动不能关，量产时再手动
   把 `RELAY_USE_IWDG` 改 1 并让 `Bsp_Iwdg_Init` 调用 `MX_IWDG_Init`）。
   若你重新 Generate Code，请先 diff `Src/Inc`，确认 USER CODE 段、
   `freertos.c` 任务表、`iwdg.c` 没有被覆盖/删除。
4. **工程/目标名沿用 `A44A_gimbal`**：Keil 工程文件名、TargetName、
   `.ioc` 的 ProjectName 都没改，减少 uvoptx/RTE 迁移风险；
   输出名已改为 `kamingo_torque_relay`。
5. **目标器件是 F407VETx**：沿用 RM25 的 512KB/128KB 配置；DevEBox
   VGT6 可直接跑，只是没用满 1MB/192KB。
6. **无 GCC/Makefile**：按最新指令保持 Keil 工作流；没有生成
   `startup_stm32f407xx.s`（GCC 版）、链接脚本或 Makefile。
7. **看门狗默认关闭**：`RELAY_USE_IWDG=0`；IWDG 一旦启动不能关，
   调试阶段先留接口。

---

## 11. 待台架验证项（全部未上板）

> 以下项目**没有硬件实测**，上板时逐项确认。

1. **ADC 零点/量程**：PA2 接 0.75V 时 torque 应≈0；接 3.2V 时应≈70000。
   用万用表/信号源核对 `v_mV = raw*3300/4095` 的增益误差。
2. **传感器输出上限**：实测 80Nm 时绿色/棕色线电压；若 >3.3V，
   必须分压/钳位后再进 PA2（见 §4.1）。
3. **踏频**：手动转一圈应数到 32 个脉冲；`rpm_x10` 与实际转速对比。
   重点验证 20kHz 预分频（TIM4 时钟是否确为 84MHz）。
4. **2s 超时**：停止踩踏 2s 后 `rpm_x10=0`、flags bit2=1。
5. **USART1**：115200 8N1，PA9→GD32 PD9，示波器/逻辑分析仪看
   14 字节帧；确认 GD32 侧 CRC 参数与 §6.1 一致。
6. **帧序号**：连续收 100 帧，seq 应连续（0..255 回绕）；
   人为拔线再插，GD32 应能通过 seq 跳变统计丢帧。
7. **LED**：PA1 低有效，status_task 500ms 翻转；若常亮/常灭，
   检查 D2 极性（低有效）。
8. **5V 供电**：力矩传感器电流是否在板载 5V 能力内；共地是否可靠。
9. **IWDG（可选）**：把 `RELAY_USE_IWDG` 改 1，确认 500ms 内喂狗；
   人为卡死 relay_task 应复位。
10. **CubeMX 重新生成**：若使用 `.ioc` Generate Code，先备份本工程，
     diff `Src/Inc/freertos.c/.ioc`，确认引脚/任务表/时钟没有被改回。

---

## 12. 已知风险 / 限制

- **手册量程不自洽**：0.5–80Nm 与 0.75–3.2V/35mV 只能覆盖到 70Nm；
  80Nm 需要 3.55V，超过 3.3V ADC 基准。当前固件按 70Nm 限幅，
  上板前必须确认传感器真实输出范围。
- **PA2 无外部保护**：长时间过压/静电可能损伤 ADC；建议加
  1k 串联 + BAT54S 钳位到 3.3V/GND（按需）。
- **PB6 内部上拉偏弱**：开集输出长线时外加 4.7k–10k 上拉。
- **16bit 捕获回绕**：2s 超时 < 3.27s 回绕，当前安全；若把超时
  改到 >3.27s，必须加定时器溢出计数。
- **UART 发送缓冲在任务栈上**：20ms 周期、14B 帧、115200 下 DMA
  1.2ms 完成，余量充足；不要把周期改到 <2ms 或把帧长加大很多。
- **IDLE 中断里 `HAL_UART_AbortReceive`**：HAL 内部会等 DMA 关闭，
  ISR 中有少量阻塞；当前 115200/32B 下可接受，未上板实测。
- **`.ioc` 未经过 CubeMX 生成验证**：只做了手工同步；以 `Src/Inc`
  为准（见 §10.3）。
- **Keil 工程文件清单是手工重写的**：命令行编译已通过，但 IDE 的
  Groups 显示未逐项目视核对；打开工程后建议扫一眼文件树。
- **FreeRTOS heap 15360B**：两个任务 + idle/timer 任务约 5KB，
  余量足够；后续加任务/队列时注意 `configTOTAL_HEAP_SIZE`。

## 上板验证记录（2026-10-30，PWLINK2 + openocd）

- 探针：PWLINK2（CMSIS-DAPv2，FW 2.1.0）；cfg=`tools/openocd_f407_pwlink2.cfg`（`reset_config none`，软复位）
- 烧录：`program ... verify` → **Verified OK**
- 结果（全部通过）：CFSR/HFSR=0；tick 1500@1.5s、+1004/1s；任务 4 个（IDLE/Tmr Svc/relayTask/statusTask），relay 唤醒周期 +20ms；堆 free=min-free=12072/15360；LED PA1 500ms 翻转；ADC DMA 循环、静置 983-1032 counts（≈0.8V）；TIM4 20kHz；UART 抓在 HAL_UART_Transmit_DMA 内（gState=BUSY_TX，TxXferSize=14）；踏频静置 0
- 踩坑：①srst 下 reset halt 失败 → 用 `reset_config none`；②**FPB 断点在本探针/openocd 组合下不生效，DWT 观察点可用**；③未开 configCHECK_FOR_STACK_OVERFLOW（建议开 2）；④ADC 零点≈810mV，标定时需偏置修正

## v2 变更（2026-11-01，编译 0E/0W，bin=39536B，未烧板）

### 总目标（用户定调）
STM32=有线透传，够简单、结构与 RM/CubeMX 完全同构；串口为主直给 GD32，USB 次选可并行；档位逼传感器性能上限；转发排队/延迟可观测可抵消。

### 变更清单
1. **TX 环形 DMA 队列**（bsp_usart v2）：`Uart1_SendPush` 非阻塞入队 512B 环 → DMA TC 中断续传（`HAL_UART_TxCpltCallback`）；满则丢帧计数 `Uart1_GetTxDropped()`；不再用 gState 轮询跳帧
2. **帧 v2 = 16B**：v1 的 14B 加 `tx_tick`（u16，1ms tick 低 16 位）@offset 12-13，CRC 覆盖 0..13。
   对齐公式：`全程延迟 = (rx_local_tick − tx_tick + 65536) % 65536`，减串口传输常量 = 纯转发延迟
3. **三档位**（串口发 ASCII '1'/'2'/'3' 即切，Relay_SetProfile 含波特率重配）：

   | 档 | 发送率 | 波特率 | 滤波窗 |
   |---|---|---|---|
   | P1 默认 | 50Hz | 115200 | 8 |
   | P2 | 200Hz | 460800 | 4 |
   | P3（E2 实验） | 500Hz | 921600 | 2 |

   ADC 恒满速（块率 ~3.4kHz），档位只改滤波窗+发送率
4. **USB CDC 并行上行**：OTG_FS CDC（PA11/PA12），`Usb_Send` 同帧流并行，未连接静默丢弃；**板上无 USB-UART 桥**，USB 只对 PC host，GD32 走 USART1

### 新增文件
`Comm/usb_comm.c/h`（CDC 封装+IRQ 汇聚+丢帧计数）、`USB_DEVICE/{App,Target}/`（26 版移植 F407 化）、`Middlewares/ST/STM32_USB_Device_Library/`、HAL pcd/pcd_ex/ll_usb

### 已知事项
- AfterMake 未配 fromelf 出 bin：`fromelf --bin --output=...bin ...axf` 手动
- 档位切换瞬间 TX 队列清空（丢帧由 seq 对账）；滤波窗切换丢窗内旧值
- USB 下行（CDC_Receive_FS）预留空壳，命令走 USART1

## v2.2 变更（2026-11-14，刹车霍尔通道；编译 0E/0W，debug 版已烧验证 VOFA 出流）

### 功能增量
- **刹车霍尔通道（HAL4395，PA3=ADC123_IN3）**：拆自遥控器扳机+环形磁铁，刹车量=路测数据标记/训练特征（非控制联锁）。选型正本=踏频仓 `develop_log/knowledge/刹车信号选型_线性霍尔与STM32接入_20261113.md`（含 HAL4395 手册口径：Enable 接地才工作/单极 S 面/零点 3.0V 捏下降/CL≤0.5nF）。
- ADC1 改双通道扫描（rank1=PA2 力矩、rank2=PA3 刹车），`bsp_adc` 双路 32 点块平均（`bsp_adc_sample_t.brake_raw`）；副作用=块率 3.4→1.7kHz（力矩块平均窗 9.4→19ms，+5ms 固定时延，P1 档 160ms 滑均主导可接受）。
- `bsp/bsp_hall.c/h`（新 305 行）：设备层移植自 rc_remote f8b2e9d(2026-06-11) `devices/hall/hall_sensor.c`（HAL4395 时代版；当前工作区是 08 月后 TMAG 重构版不可作源）。硬件读数改 `Bsp_Adc_GetBrakeRaw()`，滤波/四映射/双端死区/reverse 逐行保留；AC5 不允许 struct 强转（原版 `(hall_hw_config_t)*hw_config` 编不过）→直接结构赋值。
- `app/app_brake.c/h`（新 128 行）：12bit 端点宏 `BRAKE_MAP_RAW_FULL/ZERO`（300/3600 初值），输出 0..1000；串口命令 **BR1=记松开端 adc_max / BR2=记捏死端 adc_min**（ISR 置请求沿 PRx 先例，任务侧执行，RAM 生效掉电丢）。
- 协议 **cmd=0x04 刹车帧（14B）**：`AA55|len=9|0x04|seq|brake_raw(u16,滤后码)|brake_map(u16,0..1000)|flags(bit0=饱和)|tx_tick(u16)|crc16(覆盖前12B)`；独立 seq（v2.1 辅帧纪律），发送分频 `BRAKE_SEND_DIV`（默认 1=跟主帧 50Hz，占 115200 带宽 ~6%；未来 USART1 直连 GD 改 5）。**0x01 主帧布局一字未动**。
- **调试模式 `RELAY_DEBUG_PRINT`（relay_config.h，当前=1）**：同一条 USART1 内容替换为 VOFA+ JustFloat 波形（ch0=raw/ch1=filtered/ch2=map/ch3=sat，20B 帧@100ms，帧尾 00 00 80 7F），relay_task 二进制帧停发；`app/debug_task.c/h`（结构参照云台 RM_Taurus_25 `app/debug_task`，其 bsp_debug.c 全注释死代码/DataScope 协议不可直接搬）。改回 0 重编恢复正式帧流。

### 上板实测（2026-11-14，VOFA 波形）
- 静态 raw≈926~994（磁铁耦合中段，非手册零点 3.0V≈3700 码）；**捏放全程 Δ=49 码（≈40mV）**——环形磁铁旋转式（转角仅 ~29°）+霍尔对旋转对称位=天然低灵敏几何。
- **用户拍板：49 码先这样，不调几何**——理由：①几何调优麻烦（同事之前优化过效果一般）②刹车占骑行周期短③标记用途 SNR 6–12σ 够用（阈值+滞回判决）。标记=主用途成立；连续特征 4% 噪声勉强可用；弱点=端点漂移敏感（漂 1 码=2% 特征误差，靠 BR1/BR2 重标缓解）。
- 改进项（不阻塞，训练阶段若刹车特征太粗再动）：霍尔挪到磁极分界线切向位+间隙 2–3mm+磁铁挪活动臂最大半径端，目标 Δ≥1000 码。
- raw 锯齿噪声 ±7 码/2 帧周期（VOFA I0 通道 926↔940 交替）——嫌疑=双通道采样 crosstalk 或力矩线耦合；行程 49 码下占比 >10% 为主要误差源，行程拉大后 <1.5% 可忽略；未定因，挂账。

### 烧录与探针纪律（本次实战两条）
- 探针=有线 DAP **SN 1081747132**（COM34 那颗）；GD 那颗=SN 7047224032 勿混。**烧前必须芯片自证**：`openocd -c "adapter serial <SN>" … -c "mdw 0xE0042000"`——`0x…413`=STM32F407 可烧，`0x…419`=GD32F450 拒烧（11-14 实战拦截一次误烧 GD）。
- **openocd `program … exit` 不带 reset → 目标保持 halt、串口静默（"烧录成功"≠"在跑"）**——必须显式 `-c "reset run"` 或 `program … reset exit`。本次 COM34 无输出即此坑。
- 烧录命令模板（reset_config srst_only 会让 flash 算法超时，命令行覆盖）：
  `openocd -c "adapter serial 1081747132" -f tools/openocd_f407_pwlink2.cfg -c "reset_config none" -c "program <bin> verify 0x08000000 exit" -c "init" -c "reset run" -c "shutdown"`

### 接线（本次实装线色，只对这颗拆机件有效）
红=VCC→3V3；紫=EN→**GND**（悬空/接 3.3V 不工作）；黄=Out→10k 串联→(100nF 到地)→**PA3**；绿=GND 共地。

### 遗留待办
- ⬜ BR1/BR2 实标端点（VOFA 波形下位置确认后）
- ⬜ `RELAY_DEBUG_PRINT` 改回 0 重编重烧，恢复 0x01+0x04 正式帧流
- ⬜ 解算台/录制端解析 0x04 帧（帧格式见上文协议段）
- ⬜ raw 锯齿噪声定因（crosstalk 嫌疑，行程决定不改则挂账）

## v2.3 变更（2026-11-14，刹车行程校准状态机；编译 0E/0W，已烧板上全流程 8/8 PASS）

### 功能：对齐遥控器油门行程校准（rc_remote 0500716 状态机 + 008d31f 校准页语义）
- **命令（ASCII 直发 USART1，回 0x05 事件帧）**：`BRC`=进校准(LO 追踪松开端 max)→`BRN`(切 HI 追踪捏死端 min)→`BRA`(apply：间隔≥BRAKE_CAL_MIN_SPAN=100 码才写端点，否则 FAIL 端点不动) / `BRX`(取消恢复备份端点) / `BRQ`(查询，IDLE 也回帧)。`BR1/BR2` 单点记录保留兼容。
- **cmd=0x05 校准事件帧（16B）**：`AA55|len=11|0x05|seq|state(u8)|raw_min(u16)|raw_max(u16)|tick(u32)|crc16(前14B)`；state 枚举 0=IDLE/1=LO/2=HI/3=DONE_OK/4=FAIL/5=CANCELLED；独立 seq（v2.1 辅帧纪律）。
- 实时曲线复用 0x04 帧流（50Hz brake_raw 滤波值），校准期间 UI 直接画曲线。
- apply 后端点 RAM 生效掉电丢；事件帧带新端点值供 UI 显示。

### 板上实测（2026-11-14，COM34 脚本驱动 8 项全 PASS）
`BRQ→IDLE / BRC→LO / BRN→HI / BRA→FAIL(未真捏间隔<100 防呆拒绝✓) / BRQ→IDLE / BRC→LO / BRX→CANCELLED / BRQ→IDLE`；0x04 帧流持续（raw≈902 map=821，宏端点自洽）。
- **修一真 bug**：`Brake_TakeCalEvent` 原把"state=IDLE 事件"与"无事件"混为同一返回值（均 0），IDLE 查询被吞——改为标志/状态分离（返回有无+出参 state）。
- **烧录运维坑**：openocd `program…exit` 与后续 `reset run` 会话连发时目标可能不跑（PC=0x2000002e RAM 死态）——**分两次调用+中间 sleep 2s** 后稳定；判活三法（串口/BRQ 指令/openocd 读 PC）实战有效。

### 平板校准 UI 流程（待 solver-dev 实现，语义对齐遥控器校准页）
进校准页→发 BRC（提示"松开刹把捏放数轮"）→0x04 曲线观察→发 BRN（提示"捏到底保持数轮"）→发 BRA→0x05 DONE_OK 显示新端点（FAIL 则提示重试 BRN 阶段）；随时 BRX 取消。命令均为 3 字节 ASCII 直发 COM34。

## v2.4 变更（2026-11-15，USART1 上行单一发送调度器；编译 0E/0W，未烧板）

### 问题（2026-11-15 实车 30.9min，rec_20261115_002355）
[PA] 37.8/s（应 50）、CRC 错 17,125 次≈24%；刹车活跃段 CRC 错/秒 7.93 vs 非活跃 4.59（+73%）、seq 缺口 7.39 vs 4.47。设备侧 [USB] drop/ovf/trunc 全 0、行/产=100% ⇒ 丢不在录制端。

### 改动（单变量一刀：TX 调度解耦，不改帧格式/发送率/BSP/优先级）
- **新增 `Comm/tx_sched.c/h`**：FreeRTOS 队列（深度 32×24B 定长帧槽）+ 专发送任务（AboveNormal、栈 256w、阻塞出队→Uart1_SendPush）。队列满=丢帧计数 `TxSched_GetDropped()`，绝不阻塞 relay_task。
- `relay_task.c` 4 处直写（0x01 主帧/0x02 去皮事件/0x04 刹车/0x05 校准事件）→ `TxSched_PushUart`；`debug_task.c` VOFA 帧同改。USB 路径不动（另一条链无此问题）。
- `freertos.c`：TxSched_Init 先于两任务创建（队列先于生产者就绪）+ txSchedTask 创建（debug/业务两模式都建）。
- 产物：`kamingo_torque_relay_v24_txsched.bin` 43300B，md5 `990a5eee7eb9785e92c83b6236740893`（0E/0W）。

### 与任务提示词的口径偏差（如实记）
1. "0x01 与 0x04 未串行化"**不准确**：BSP 层早有 512B TX 环形队列+s_tx_busy 标志+双侧临界区 pump（2026-11-07 修过 ISR 死锁），且业务流下唯一调用方=relay_task 单任务（顺序天然）。本刀把"帧构造"与"字节发送"彻底解耦（BSP 调用只存在于单一消费者），串行化更严，但**若真因在 BSP 之下的字节流时序，此刀可能不改善 CRC**。
2. **AI 的替代假设（未验证，不与主假设同刀测）**：0x04 插入使字节流从"16B+gap"变"30B 连续"，GD 侧 USART RX 若为逐字节中断无 DMA，IDLE gap 缩短→中断压力↑→RX overrun 丢字节→CRC 错，与刹车活跃+73% 同向。**验证法=判据已内嵌：本刀上台架后 CRC 错若不降，即指向 GD 侧接收（下一刀改 GD RX 或 BRAKE_SEND_DIV=5 降 0x04 率），勿同时调 BSP 优先级。**
3. tx_tick 语义微变：现为"构造时刻"（BSP 环形队列时代本就如此），发送任务增加 μs 级转发延迟，远小于 tick 分辨率。

### 待台架验收（板不在位，未烧）
- 门1：台架 5min（力矩 50/s+刹车每 10s 一捏）[PA] CRC 错<1/s 且 ≥47/s
- 门2：刹车活跃 vs 非活跃错帧率差 <20%（before：+73%）
- 门3：tx_tick 单调、seq 连续
- before：4.59 / 7.93（CRC 错每秒，非刹车/刹车中）
