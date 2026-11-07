/**
  ******************************************************************************
  * @file    gpio.h
  * @author  Kamingo
  * @brief   GPIO 初始化（板载 LED PA1）
  * @date    2026-10-30
  * @version 1.0
  ******************************************************************************
  */

#ifndef __GPIO_H
#define __GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void MX_GPIO_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __GPIO_H */
