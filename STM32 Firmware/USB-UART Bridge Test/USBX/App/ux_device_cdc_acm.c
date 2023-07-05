/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_cdc_acm.c
  * @author  MCD Application Team
  * @brief   USBX Device applicative file
  ******************************************************************************
    * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "main.h"
#include "app_usbx_device.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define APP_RX_DATA_SIZE	2048
#define APP_TX_DATA_SIZE	2048

#define MIN_BAUDRATE     9600
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

#if defined ( __ICCARM__ ) /* IAR Compiler */
#pragma location = 0x24028000
#elif defined ( __CC_ARM ) /* MDK ARM Compiler */
__attribute__((section(".UsbxAppSection")))
#elif defined ( __GNUC__ ) /* GNU Compiler */
__attribute__((section(".UsbxAppSection")))
#endif

UX_SLAVE_CLASS_CDC_ACM  *cdc_acm;

/* Data received over USB CDC are stored in this buffer */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/* Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

uint32_t UserTxBufPtrIn;
uint32_t UserTxBufPtrOut;

UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER CDC_VCP_LineCoding =
{
  115200, /* baud rate */
  0x00,   /* stop bits-1 */
  0x00,   /* parity - none */
  0x08    /* nb. of bits 8 */
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void Error_Handler(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  USBD_CDC_ACM_Activate
  *         This function is called when insertion of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Activate */
	cdc_acm = (UX_SLAVE_CLASS_CDC_ACM*) cdc_acm_instance;

  if (ux_device_class_cdc_acm_ioctl(cdc_acm,
          UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
          &CDC_VCP_LineCoding) != UX_SUCCESS)
  {
	  Error_Handler();
  }

  /* USER CODE END USBD_CDC_ACM_Activate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_Deactivate
  *         This function is called when extraction of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Deactivate */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);

  cdc_acm = UX_NULL;
  /* USER CODE END USBD_CDC_ACM_Deactivate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_ParameterChange
  *         This function is invoked to manage the CDC ACM class requests.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */
	  UX_PARAMETER_NOT_USED(cdc_acm_instance);

	  ULONG request;
	  UX_SLAVE_TRANSFER *transfer_request;
	  UX_SLAVE_DEVICE *device;

	  /* Get the pointer to the device.  */
	  device = &_ux_system_slave -> ux_system_slave_device;

	  /* Get the pointer to the transfer request associated with the control endpoint. */
	  transfer_request = &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

	  request = *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);

	  switch (request)
	  {
	    case UX_SLAVE_CLASS_CDC_ACM_SET_LINE_CODING :

	      /* Get the Line Coding parameters */
	      if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_CODING,
	                                        &CDC_VCP_LineCoding) != UX_SUCCESS)
	      {
	        Error_Handler();
	      }

	      /* Check if baudrate < 9600) then set it to 9600 */
	      if (CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_baudrate < MIN_BAUDRATE)
	      {
	        CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_baudrate = MIN_BAUDRATE;

	        /* Set the new configuration of ComPort */

	      }

	      break;

	    case UX_SLAVE_CLASS_CDC_ACM_GET_LINE_CODING :

	      /* Set the Line Coding parameters */
	      if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
	                                        &CDC_VCP_LineCoding) != UX_SUCCESS)
	      {
	        Error_Handler();
	      }

	      break;

	    case UX_SLAVE_CLASS_CDC_ACM_SET_CONTROL_LINE_STATE :
	    default :
	      break;
	  }

  /* USER CODE END USBD_CDC_ACM_ParameterChange */

  return;
}

/* USER CODE BEGIN 1 */

/**
  * @brief  Function implementing usbx_cdc_acm_thread_entry.
  * @param  arg: Not used
  * @retval none
  */

void usbx_cdc_acm_thread_entry(ULONG arg)
{
  UX_SLAVE_DEVICE *device;
  ULONG actual_length;
  ULONG actual_length_write;
  ULONG ux_status = UX_SUCCESS;

  /* Disable warnings for unused variables */
  UX_PARAMETER_NOT_USED(arg);
  UX_PARAMETER_NOT_USED(ux_status);

  /* Get device */
  device = &_ux_system_slave->ux_system_slave_device;

  while (1)
  {
    /* Get Data interface */


    /* Check if device is configured */
    if ((device->ux_slave_device_state == UX_DEVICE_CONFIGURED) && (cdc_acm != UX_NULL))
    {

#ifndef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE

        /* Set transmission_status to UX_FALSE for the first time */
        cdc_acm -> ux_slave_class_cdc_acm_transmission_status = UX_FALSE;

#endif /* UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE */

        /* Read the received data in blocking mode */
        ux_device_class_cdc_acm_read(cdc_acm, (UCHAR *)UserRxBufferFS, 64,
                                     &actual_length);
        if (actual_length != 0)
        {
        	ux_status = ux_device_class_cdc_acm_write(cdc_acm,
        	                         (UCHAR *)(&UserRxBufferFS[0]),
									 actual_length, &actual_length_write);

        }
        else{
        	ux_status = UX_TRANSFER_DATA_LESS_THAN_EXPECTED;
        }
    }
    else
    {
      tx_thread_sleep(10);
    }
  }
}

/* USER CODE END 1 */
