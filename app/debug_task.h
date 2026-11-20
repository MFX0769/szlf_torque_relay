/**
  ******************************************************************************
  * @file    debug_task.h
  * @author  Kamingo
  * @brief   调试打印任务接口（云台工程 debug_task 模式移植）
  * @date    2026-11-14
  * @version 1.0
  * @note    RELAY_DEBUG_PRINT=1 时由 freertos.c 创建；同一条 USART1
  *          输出 debug 文本行（此时 relay_task 二进制帧停发）。
  ******************************************************************************
  */

#ifndef __DEBUG_TASK_H__
#define __DEBUG_TASK_H__

/** @brief 调试打印任务：100ms 一行 [BRK] 霍尔原始/滤波/映射/端点值 */
void debug_task(void const *argu);

#endif /* __DEBUG_TASK_H__ */
