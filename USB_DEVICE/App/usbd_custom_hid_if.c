/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @version        : v1.0_Cube
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t usbBuffer[0x40];


/** Usb custom HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
    /* USER CODE BEGIN 0 */
    0x06, 0x00, 0xff,              //   Usage Page(Undefined )
    0x09, 0x01,                    //   USAGE (Undefined)
    0xa1, 0x01,                    //   COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x40,                    //   REPORT_COUNT (64)
    0x09, 0x01,                    //   USAGE (Undefined)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x40,                    //   REPORT_COUNT (64)
    0x09, 0x01,                    //   USAGE (Undefined)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x09, 0x01,                    //   USAGE (Undefined)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)

    /* USER CODE END 0 */
    0xC0    /*     END_COLLECTION                */
};


extern USBD_HandleTypeDef hUsbDeviceFS;



/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t* state);

/**
  * @}
  */

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS =
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS
};


static char usbData[24+9];
static uint8_t usbSendReq = 0;

void sendbytesViausb(uint8_t* data)
{
#if 0
    memset(usbData, 0, strlen(usbData));
	sprintf(usbData,"%d-%d-%d-%d-%d-%d-%d-%d",
			data[0], data[1], data[2], data[3],
			data[4], data[5], data[6], data[7]);
#endif

	memcpy(usbData, data, 10);

	usbSendReq = 1;
}


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{
  return (USBD_OK);
}

/**
  * @brief  DeInitializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  return (USBD_OK);
}

/**
  * @brief  Manage the CUSTOM HID class events
  * @param  event_idx: Event index
  * @param  state: Event state
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t* state)
{
	printf("USB In: %s\r\n", state);
	printf("%d - %d - %d - %d - %d\r\n\n", state[0], state[1], state[2], state[3], state[4] );

	memset(usbBuffer,0,0x40);       //Set send buffer out length to 64 bytes.

	usbBuffer[0] = 10;
	memcpy(&usbBuffer[1],usbData,usbBuffer[0]);
	usbSendReq = 0;

	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,(uint8_t*)usbBuffer,0x40);


	return (USBD_OK);
}



