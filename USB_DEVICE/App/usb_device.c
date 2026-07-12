/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
#define USB_RECOVERY_ENUM_TIMEOUT_MS       3000U
#define USB_RECOVERY_RECONNECT_DELAY_MS    25U
#define USB_RECOVERY_MIN_INTERVAL_MS       1000U
#define USB_RECOVERY_STABLE_MS             10000U
#define USB_RECOVERY_MAX_ATTEMPTS          3U

static volatile uint8_t s_usb_recovery_pending = 0U;
static volatile uint8_t s_usb_enumeration_active = 0U;
static volatile uint32_t s_usb_recovery_reason = 0U;
static volatile uint32_t s_usb_bus_reset_tick = 0U;
static uint32_t s_usb_last_recovery_tick = 0U;
static uint32_t s_usb_configured_since = 0U;
static uint8_t s_usb_recovery_attempts = 0U;
static uint8_t s_usb_initialized = 0U;

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */
void USB_DEVICE_RequestRecovery(uint32_t reason)
{
  s_usb_recovery_reason = reason;
  s_usb_recovery_pending = 1U;
}

void USB_DEVICE_NotifyBusReset(void)
{
  s_usb_bus_reset_tick = HAL_GetTick();
  s_usb_enumeration_active = 1U;
}

void USB_DEVICE_Process(void)
{
  uint32_t now = HAL_GetTick();

  if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
    s_usb_enumeration_active = 0U;
    if (s_usb_configured_since == 0U) {
      s_usb_configured_since = now;
    } else if ((now - s_usb_configured_since) >= USB_RECOVERY_STABLE_MS) {
      s_usb_recovery_attempts = 0U;
    }
  } else {
    s_usb_configured_since = 0U;
    if ((s_usb_enumeration_active != 0U) &&
        ((now - s_usb_bus_reset_tick) >= USB_RECOVERY_ENUM_TIMEOUT_MS)) {
      s_usb_enumeration_active = 0U;
      USB_DEVICE_RequestRecovery(USB_RECOVERY_REASON_ENUM_TIMEOUT);
    }
  }

  if (s_usb_recovery_pending == 0U) {
    return;
  }
  if (s_usb_recovery_attempts >= USB_RECOVERY_MAX_ATTEMPTS) {
    s_usb_recovery_pending = 0U;
    return;
  }
  if ((s_usb_last_recovery_tick != 0U) &&
      ((now - s_usb_last_recovery_tick) < USB_RECOVERY_MIN_INTERVAL_MS)) {
    return;
  }

  s_usb_recovery_pending = 0U;
  s_usb_enumeration_active = 0U;
  s_usb_last_recovery_tick = now;
  s_usb_recovery_attempts++;
  (void)s_usb_recovery_reason;

  if (s_usb_initialized != 0U) {
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
    HAL_NVIC_ClearPendingIRQ(OTG_FS_IRQn);
    (void)USBD_DeInit(&hUsbDeviceFS);
    s_usb_initialized = 0U;
  }
  HAL_Delay(USB_RECOVERY_RECONNECT_DELAY_MS);
  MX_USB_DEVICE_Init();
}

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */
  s_usb_initialized = 1U;
  s_usb_configured_since = 0U;

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

