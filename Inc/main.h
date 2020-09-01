/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define DEBUG 1
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define BUZZER_out_Pin GPIO_PIN_13
#define BUZZER_out_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define ready_to_drive_button_Pin GPIO_PIN_6
#define ready_to_drive_button_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
/*aviciis code BEGIN*/


#define NUTRAL 				1
#define IGNITION_TO_DRIVE 	2
#define BUZZER				3
#define DRIVE 				4
#define ERROR_state			5
//#define ERROR				5

//==== APPS DEFINE ======
#define TABLE_MODEL 1
#if TABLE_MODEL
//APPS values for table model
#define APPS_0_MAX_VAL 3787
#define APPS_0_MIN_VAL 3216
#define APPS_1_MAX_VAL 1741
#define APPS_1_MIN_VAL 19
#define APPS_2_MAX_VAL 0xFFFF
#define APPS_2_MIN_VAL 0xFFFF
#endif

//#define APPS_0_MIN		709
//#define APPS_0_MAX		2300

//#define APPS_1_MIN		0										// Need to measure the true values
//#define APPS_1_MAX		0xFFFF									// Need to measure the true values

//#define APPS_2_MIN		0
//#define APPS_2_MAX		0

//#define OUTPUT_SCALE 	100


//=========== GLOBAL ERRORS =============
#define ERROR_APPS             			0xFFFF
#define ERROR_APPS_MAXVALUE				0xDFFF
#define ERROR_BPPS          		    0xFF
#define ERROR_APPS_BPPS_TIMEOUT 		0xFF
//#define ERROR_SHRT_CIRC_TO_GRND	 		0x0050
#define ERROR_OpenSHTDWN			 	1						// Defines an error that open shutdown circuit at Safe State
#define ERROR_DontSHTDWN			 	2						// Defines an error the doesn't open shutdown circuit at Safe State







#if !TABLE_MODEL
//
#define APPS_0_MAX_VAL 3446
#define APPS_0_MIN_VAL 0
#define APPS_1_MAX_VAL 3446
#define APPS_1_MIN_VAL 0
#define APPS_2_MAX_VAL 3446
#define APPS_2_MIN_VAL 0
#endif


/*aviciis code END*/
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
