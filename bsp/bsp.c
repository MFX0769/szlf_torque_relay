/**
  ******************************************************************************
  * @file    bsp.c
  * @author  Kamingo
  * @brief   所有 BSP 模块的总调用：初始化
  * @date    2026-10-30
  * @version 1.0
  ==============================================================================
*/

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"

/**
  * @brief BSP 层初始化（在 MX_xxx_Init 之后、调度器启动之前调用）
  * @retval None
  */
void Bsp_Init(void)
{
  Bsp_Led_Init();
  Bsp_Adc_Init();      /* 启动 ADC1 连续 DMA */
  Bsp_Button_Init();   /* PA0 板载 K1 输入（去皮按键，v2.1） */
  Bsp_Capture_Init();  /* 启动 TIM4_CH1 输入捕获 */
  Bsp_Iwdg_Init();     /* 默认空实现 */
}
