/**
  ******************************************************************************
  * @file    status_task.c
  * @author  Kamingo
  * @brief   状态任务：软件看门狗位图 + LED 心跳 + 可选喂狗
  * @date    2026-10-30
  * @version 1.0
  * @note    位图思路照 Taurus26 Application/status_task.c：
  *          各任务/中断置位，status_task 周期性检查并清空；
  *          某个位连续丢失说明对应任务卡死。
  *          IWDG 默认关闭（RELAY_USE_IWDG=0），打开后只有 relay 位
  *          正常才喂狗。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "status_task.h"
#include "bsp_led.h"
#include "bsp_iwdg.h"

/* Private define ------------------------------------------------------------*/
#define STATUS_PERIOD_MS            (100U)
#define LED_TOGGLE_TICKS            (5U)   /* 100ms * 5 = 500ms 翻转 */

/* Private variables ---------------------------------------------------------*/
status_t status;

static volatile uint32_t s_wdg_event_group = 0;

/**
  * @brief 置位待检测量
  * @param status_bit 位掩码
  * @retval None
  */
void wdg_user_set_bit(uint32_t status_bit)
{
  s_wdg_event_group |= status_bit;
}

/**
  * @brief 检查某一位是否齐全
  * @param status_bit 位掩码
  * @retval 1=齐全
  */
static uint8_t bit_check(uint32_t status_bit)
{
  return (uint8_t)((s_wdg_event_group & status_bit) == status_bit);
}

/**
  * @brief 状态任务主体
  * @param argu 未使用
  * @retval None
  */
void status_task(void const *argu)
{
  uint8_t led_tick = 0;
  uint32_t last_wake = osKernelSysTick();
  uint8_t relay_ok;
  uint8_t uart_ok;

  (void)argu;

  Bsp_Led_Off();

  for (;;)
  {
    taskENTER_CRITICAL();
    relay_ok = bit_check(WDG_BIT_TASK_RELAY);
    uart_ok = bit_check(WDG_BIT_BSP_RELAY_UART);
    s_wdg_event_group = 0; /* 清空，等待下一轮 */
    taskEXIT_CRITICAL();

    status.relay = relay_ok;
    status.uart = uart_ok;

    /* relay 任务正常才喂狗；IWDG 未启用时为空函数 */
    if (relay_ok != 0U)
    {
      Bsp_Iwdg_Refresh();
    }

    /* 500ms 心跳翻转 */
    led_tick++;
    if (led_tick >= LED_TOGGLE_TICKS)
    {
      led_tick = 0;
      Bsp_Led_Toggle();
    }

    (void)osDelayUntil(&last_wake, STATUS_PERIOD_MS);
  }
}
