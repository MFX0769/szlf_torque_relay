/**
  ******************************************************************************
  * @file    bsp_hall.h
  * @author  Kamingo
  * @brief   线性霍尔传感器设备层（滤波 + 映射 + 传感器对象）
  * @date    2026-11-14
  * @version 1.0
  * @note    移植自 rc_remote f8b2e9d(2026-06-11) devices/hall/hall_sensor.c/h
  *          （HAL4395 时代版），保留原结构与命名以便对照审计：
  *          - 硬件层读数改为 Bsp_Adc_GetBrakeRaw()（BK3633 ADC API 不适用）；
  *          - Enable/电源 GPIO 逻辑保留骨架（GPIO_CTRL_NONE 路径），中继板
  *            HAL4395 Enable 硬件接地，不走固件；
  *          - 字段名 data.throttle / 接口名 adc_to_throttle 沿用原版，
  *            本工程语义=刹车量 0..1000。
  *          滤波=滑动窗口平均；映射=线性/指数/对数/S 曲线 + 双端死区 + 反向。
  ******************************************************************************
  */

#ifndef __BSP_HALL_H
#define __BSP_HALL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief GPIO控制配置
 */
typedef struct {
    uint8_t gpio;           /* GPIO引脚（0xFF = 无效/未连接） */
    uint8_t active_level;   /* 有效电平（0或1） */
} gpio_ctrl_t;

/** 无效GPIO标记（用于没有使能/电源控制的霍尔） */
#define GPIO_CTRL_NONE  { 0xFF, 0 }

/**
 * @brief 霍尔传感器硬件配置
 * @note  v2.2 中继板：adc_channel/en_ctrl/power_ctrl 仅保留结构兼容，
 *        实际读数固定走 Bsp_Adc 刹车通道，GPIO 控制不使用（0xFF）。
 */
typedef struct {
    uint8_t      adc_channel;   /* ADC通道号（保留字段，本板不使用） */
    gpio_ctrl_t  en_ctrl;       /* 使能控制gpio */
    gpio_ctrl_t  power_ctrl;    /* 电源控制gpio */
} hall_hw_config_t;

/**
 * @brief 滤波器配置（滑动窗口平均）
 */
typedef struct {
    uint8_t  window_size;         /* 滑动窗口大小 */
    uint16_t *window_buf;         /* 窗口缓冲区指针 */
    uint8_t  window_index;        /* 当前窗口索引 */
    uint32_t window_sum;          /* 窗口累加和 */
} hall_filter_t;

/**
 * @brief 映射曲线类型
 */
typedef enum {
    HALL_MAP_LINEAR = 0,        /* 线性映射 */
    HALL_MAP_EXPONENTIAL,       /* 指数映射（加速响应） */
    HALL_MAP_LOGARITHMIC,       /* 对数映射（平滑响应） */
    HALL_MAP_S_CURVE,           /* S曲线映射（中间敏感） */
} hall_map_type_t;

/**
 * @brief 霍尔映射配置
 */
typedef struct {
    uint16_t adc_min;           /* ADC最小值（下限，捏死端小值） */
    uint16_t adc_max;           /* ADC最大值（上限，松开端大值） */
    uint16_t deadzone_low;      /* 下限死区半径  死区范围[adc_min - deadzone_low, adc_min + deadzone_low] */
    uint16_t deadzone_high;     /* 上限死区半径  死区范围 [adc_max - deadzone_high, adc_max + deadzone_high] */
    uint16_t throttle_min;      /* 输出最小值（通常0） */
    uint16_t throttle_max;      /* 输出最大值（本工程=1000，0.1%分辨率） */
    hall_map_type_t map_type;   /* 映射类型 */
    bool     reverse;           /* 是否反向（true: ADC大->输出小；HAL4395 松开=大值） */
} hall_map_config_t;

/**
 * @brief 霍尔传感器对象
 */
typedef struct {
    hall_hw_config_t  hw;       /* 硬件配置 */
    hall_filter_t     filter;   /* 滤波器 */
    hall_map_config_t map;      /* 映射配置 */
    bool              enabled;  /* 已使能标志位 */
    /* 采样数据（调用hall_sensor_update后更新） */
    struct data{
        uint16_t      raw;      /* 原始ADC值 */
        uint16_t      filtered; /* 滤波后ADC值 */
        uint16_t      throttle; /* 映射后输出值（本工程语义=刹车量） */
    } data;
} hall_sensor_t;

/**
 * @brief 初始化霍尔传感器对象
 * @param sensor 传感器对象指针
 * @param hw_config 硬件配置（本板传 NULL 不合法，须给配置结构）
 * @param map_config 映射配置（传NULL使用默认配置）
 * @param filter_buf 滤波器缓冲区（uint16_t数组）
 * @param filter_size 滤波器窗口大小（元素个数，建议4-16）
 */
void hall_sensor_init(hall_sensor_t *sensor,
                       hall_hw_config_t *hw_config,
                       hall_map_config_t *map_config,
                      uint16_t *filter_buf,
                      uint8_t filter_size);

/**
 * @brief 采样一次，更新 raw / filtered / throttle 三个成员
 * @param sensor 传感器对象指针
 * @note  调用后通过 sensor->raw / sensor->filtered / sensor->throttle 访问数据
 */
void hall_sensor_update(hall_sensor_t *sensor);

/**
 * @brief 重置传感器（清空滤波器）
 */
void hall_sensor_reset(hall_sensor_t *sensor);

/**
 * @brief 配置映射参数
 */
void hall_sensor_config_map(hall_sensor_t *sensor, const hall_map_config_t *map);

/**
 * @brief 滤波器接口（内部使用，校准 apply 后可 reset）
 */
void hall_filter_init(hall_filter_t *filter, uint16_t *buf, uint8_t size);
void hall_filter_reset(hall_filter_t *filter);
uint16_t hall_filter_update(hall_filter_t *filter, uint16_t raw_value);

/**
 * @brief 映射接口（内部使用）
 */
void hall_map_init_default(hall_map_config_t *map);
uint16_t hall_map_adc_to_throttle(const hall_map_config_t *map, uint16_t adc_value);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HALL_H */
