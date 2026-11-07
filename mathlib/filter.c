/**
  ******************************************************************************
  * @file    filter.c
  * @author  Kamingo
  * @brief   滑动平均滤波器
  * @date    2026-10-30
  * @version 1.0
  * @note    窗口内数据填满前按实际点数求平均，避免上电初值拖尾。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "filter.h"
#include <stddef.h>

/**
  * @brief 初始化滑动平均
  * @param f      滤波器
  * @param window 窗口点数（1..FILTER_MOVAVG_MAX_WINDOW）
  * @retval None
  */
void Filter_MovAvg_Init(filter_movavg_t *f, uint8_t window)
{
  uint8_t i;

  if (f == NULL)
  {
    return;
  }
  if (window == 0U)
  {
    window = 1U;
  }
  if (window > FILTER_MOVAVG_MAX_WINDOW)
  {
    window = FILTER_MOVAVG_MAX_WINDOW;
  }

  f->window = window;
  f->index = 0;
  f->count = 0;
  f->sum = 0;
  for (i = 0; i < FILTER_MOVAVG_MAX_WINDOW; i++)
  {
    f->buf[i] = 0;
  }
}

/**
  * @brief 清空历史
  * @param f 滤波器
  * @retval None
  */
void Filter_MovAvg_Reset(filter_movavg_t *f)
{
  if (f == NULL)
  {
    return;
  }
  Filter_MovAvg_Init(f, f->window);
}

/**
  * @brief 推入一个新值并返回当前平均
  * @param f     滤波器
  * @param value 新值
  * @retval 平均后的整数
  */
int32_t Filter_MovAvg_Update(filter_movavg_t *f, int32_t value)
{
  if (f == NULL || f->window == 0U)
  {
    return value;
  }

  if (f->count < f->window)
  {
    f->buf[f->index] = value;
    f->sum += value;
    f->count++;
  }
  else
  {
    f->sum -= f->buf[f->index];
    f->buf[f->index] = value;
    f->sum += value;
  }

  f->index++;
  if (f->index >= f->window)
  {
    f->index = 0;
  }

  return f->sum / (int32_t)f->count;
}
