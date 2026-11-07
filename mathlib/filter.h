/**
  ******************************************************************************
  * @file    filter.h
  * @author  Kamingo
  * @brief   滑动平均滤波器（自写，替代 RM25 的 Kalman/低通业务模块）
  * @date    2026-10-30
  * @version 1.0
  ******************************************************************************
  */

#ifndef __FILTER_H
#define __FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FILTER_MOVAVG_MAX_WINDOW   (16U)

typedef struct
{
  int32_t buf[FILTER_MOVAVG_MAX_WINDOW];
  int32_t sum;
  uint8_t window;
  uint8_t index;
  uint8_t count;
} filter_movavg_t;

void Filter_MovAvg_Init(filter_movavg_t *f, uint8_t window);
void Filter_MovAvg_Reset(filter_movavg_t *f);
int32_t Filter_MovAvg_Update(filter_movavg_t *f, int32_t value);

#ifdef __cplusplus
}
#endif

#endif /* __FILTER_H */
