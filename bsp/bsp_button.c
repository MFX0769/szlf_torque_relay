/**
  ******************************************************************************
  * @file    bsp_button.c
  * @author  Kamingo
  * @brief   板载用户按键 K1（PA0/WK_UP）：软件消抖状态机（时间基准消抖）
  * @date    2026-11-12
  * @version 1.0
  * @note    接线依据：DevEBox 原理图板载 K1=PA0(WK_UP)，按下接 3V3，
  *          抬起靠内部下拉（PULL_DOWN）保持低 —— 见 README §3.1 与
  *          relay_config.h "PA0/PA1 被板载按键/LED 占用" 的注释。
  *          消抖不用计数用 tick：任务周期随档位在 2~20ms 之间变化，
  *          时间基准消抖在所有档位下行为一致（≥TARE_DEBOUNCE_MS）。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_button.h"
#include "main.h"
#include "relay_config.h"

/* Private types ---------------------------------------------------------------*/
typedef enum
{
  BTN_ST_ARMED = 0,   /* 空闲已武装：等待稳定按下沿 */
  BTN_ST_DOWN_RAW,    /* 读到高但未满消抖时长 */
  BTN_ST_PRESSED,     /* 按压沿已产生（等待调用方消费+释放） */
  BTN_ST_RELEASE_RAW  /* 读到低但未满消抖时长 */
} btn_state_t;

/* Private variables -----------------------------------------------------------*/
static btn_state_t s_state = BTN_ST_ARMED;
static uint32_t    s_edge_tick = 0U; /* 最近一次电平翻转时刻（消抖起点） */

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief 按键 GPIO 初始化（PA0 输入 + 内部下拉）
  * @retval None
  */
void Bsp_Button_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin   = TARE_BTN_GPIO_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN; /* 按下接 3V3：内部下拉保抬起飞低 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TARE_BTN_GPIO_PORT, &GPIO_InitStruct);

  s_state     = BTN_ST_ARMED;
  s_edge_tick = HAL_GetTick();
}

/**
  * @brief 消抖轮询：检测一次稳定按下沿（见 bsp_button.h）
  * @retval 1=按压沿（消费型）；0=无
  */
uint8_t Bsp_Button_PressedEdge(void)
{
  const uint32_t now = HAL_GetTick();
  const uint8_t  level_high =
    (HAL_GPIO_ReadPin(TARE_BTN_GPIO_PORT, TARE_BTN_GPIO_PIN) == GPIO_PIN_SET) ? 1U : 0U;

  switch (s_state)
  {
  case BTN_ST_ARMED: /* 空闲：高电平出现即记录起点，进入消抖 */
    if (level_high != 0U)
    {
      s_edge_tick = now;
      s_state = BTN_ST_DOWN_RAW;
    }
    break;

  case BTN_ST_DOWN_RAW: /* 高电平持续 ≥ 消抖时长 → 按压沿成立 */
    if (level_high != 0U)
    {
      if ((now - s_edge_tick) >= TARE_DEBOUNCE_MS)
      {
        s_state = BTN_ST_PRESSED;
        return 1U;
      }
    }
    else
    {
      s_state = BTN_ST_ARMED; /* 抖动回落：重新武装 */
    }
    break;

  case BTN_ST_PRESSED: /* 等待释放：低电平出现即记录起点 */
    if (level_high == 0U)
    {
      s_edge_tick = now;
      s_state = BTN_ST_RELEASE_RAW;
    }
    break;

  case BTN_ST_RELEASE_RAW: /* 低电平持续 ≥ 消抖时长 → 重新武装（支持连续按） */
  default:
    if (level_high == 0U)
    {
      if ((now - s_edge_tick) >= TARE_DEBOUNCE_MS)
      {
        s_state = BTN_ST_ARMED;
      }
    }
    else
    {
      s_state = BTN_ST_PRESSED; /* 抖动回高：仍算按住 */
    }
    break;
  }

  return 0U;
}
