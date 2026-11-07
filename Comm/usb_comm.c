/**
  ******************************************************************************
  * @file    usb_comm.c
  * @author  Kamingo
  * @brief   USB CDC 通信封装（F407 OTG_FS 版）
  * @date    2026-11-01
  * @version 1.0
  * @note    MX_USB_DEVICE_Init 由 CubeMX 风格的 usb_device.c 提供，
  *          这里做 IRQ 汇聚、发送包装与连接检测。发送走 CDC 数据 EP，
  *          CDC 未配置（PC 未打开串口）时返回忙，由调用方决定丢弃计数。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usb_comm.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include <stddef.h>

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t s_usb_dropped = 0;

/**
  * @brief USB 设备栈初始化（枚举 CDC）
  * @note  在调度器启动前、MX_USART1_Init 之后调用；USB 时钟
  *        = PLLQ 48MHz（RM25 168MHz 主频的标准分频）。
  */
void comm_usb_init(void)
{
  (void)MX_USB_DEVICE_Init();
}

/**
  * @brief OTG_FS 中断汇聚（stm32f4xx_it.c 调用）
  */
void OTG_FS_IRQ_Handler(void)
{
  extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

/**
  * @brief USB CDC 发送（非阻塞；失败/未连接丢弃并计数）
  * @param buf 数据
  * @param len 长度（≤ APP_TX_DATA_SIZE）
  * @retval 0=已提交 CDC；1=丢弃
  */
uint8_t Usb_Send(const uint8_t *buf, uint16_t len)
{
  if ((buf == NULL) || (len == 0U))
  {
    return 1U;
  }

  /* CDC_Transmit_FS：未枚举/未配置时返回 USBD_BUSY，不阻塞 */
  if (CDC_Transmit_FS((uint8_t *)buf, len) != USBD_OK)
  {
    s_usb_dropped++;
    return 1U;
  }
  return 0U;
}

/**
  * @brief USB 是否已连接并被主机配置（CDC 打开）
  * @retval 1=已连接
  */
uint8_t Usb_IsConnected(void)
{
  extern USBD_HandleTypeDef hUsbDeviceFS;
  return (uint8_t)((hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1U : 0U);
}

/**
  * @brief 获取 USB 发送丢弃次数
  */
uint32_t Usb_GetDropped(void)
{
  return s_usb_dropped;
}
