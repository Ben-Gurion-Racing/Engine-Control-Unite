
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "lwip.h"

/* USER CODE BEGIN Includes */
#include  <errno.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include  <stdlib.h>
#include  <stdio.h>
#include  <stm32f7xx_hal_can.h>
#include "can.h"
//https://www.st.com/content/ccc/resource/technical/document/reference_manual/group0/96/8b/0d/ec/16/22/43/71/DM00224583/files/DM00224583.pdf/jcr:content/translations/en.DM00224583.pdf
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


//===================== ETH ============================

ip4_addr_t destIPAddr;

//===================== CAN ============================
volatile unsigned char CAN_1_Rx_received_flg = 0x00;
volatile unsigned char CAN_1_RecData[8];
unsigned long CAN_1_specific_id_test  = 0x00;
unsigned long CAN_1_temp_id;
unsigned int  CAN_1_eid;
unsigned int  CAN_1_sid;
uint32_t CAN_1_ide;
uint32_t CAN_1_rtr;
uint32_t CAN_1_dlc;
uint32_t CAN_1_fmi;
volatile unsigned car_state;
volatile uint32_t tickstart = 0U;

//==================== TIME =========================
volatile unsigned char	Time_1_Ms_Flag = 0x00;
volatile unsigned char	Time_5_Ms_Flag = 0x00;
volatile unsigned char	Time_1_Se_Flag = 0x00;
//==================== KEEP =========================
volatile unsigned char	Keep_80[16];
volatile unsigned char	Keep_420[16];

//==================== FLAG =========================

volatile unsigned char brak_flag = 0x00;
volatile unsigned char motor_LEFT = 0x00;
volatile unsigned char motor_RIGHT = 0x00;


//==================== MAIN var =====================
volatile unsigned int count = 0;
static void init_CAN1_BGR(void);
#if 0
/*aviciis code BEGIN*/
uint32_t val;
uint32_t output;
double max_val = MAXIMUM_VAL;
double min_val = MINIMUM_VAL;
double scale = OUTPUT_SCALE;
char answer;
/*aviciis code END*/
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int _write(int file, char *ptr, int len);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	//while(1);
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
#if 0
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_LWIP_Init();
  MX_CAN1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
#endif


  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_LWIP_Init();
  MX_ADC1_Init();
  init_CAN1_BGR();

	HAL_GPIO_WritePin( GPIOB , GPIO_PIN_2|LD2_Pin , GPIO_PIN_RESET);

  //============= CAN ===================
      __HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
      HAL_CAN_Start( &hcan1 );
  #if 1
  // standart CAN message by elik
      /* Set up the Id */
      CAN1->sTxMailBox[1U].TIR =  ((  0x80   << 21U) |  0);
      /* Set up the DLC */
      CAN1->sTxMailBox[1U].TDTR &= 0xFFFFFFF0U;
      CAN1->sTxMailBox[1U].TDTR |= 0x00000008U;
      /* Set up the data field */

      CAN1->sTxMailBox[1U].TDLR =  (0xAA << 24U) |  (0x55 << 16U) |(0xAA << 8U) | (0x55 );
      CAN1->sTxMailBox[1U].TDHR =  (0xA5 << 24U) |  (0x5A << 16U) |(0xA5 << 8U) | (0x5A );
      /* Request transmission */
      CAN1->sTxMailBox[1U].TIR  |=  CAN_TI0R_TXRQ;

  #endif

//================== ETH ==================
IP4_ADDR(&destIPAddr,192,168,1,49);

Start_Motor_1(); //set motor off
Start_Motor_2(); //set motor off

car_state = DRIVE; // initial the state; TODO write the value to the flash a

#if 1 //debug
motor_LEFT = 1;
motor_RIGHT = 1;
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while(1){

	//if(can_reacive)
	//get meassage

	if( Time_5_Ms_Flag ){

		Time_5_Ms_Flag = 0x00;

 		if(Keep_420[1] == 0x00){

     		Keep_420[1] = 0x01;
 		}
 		else{
 			HAL_GPIO_WritePin( GPIOB , GPIO_PIN_2 , GPIO_PIN_SET);
 		}


		// standart CAN message by elik
		/* Set up the Id */
		CAN1->sTxMailBox[0U].TIR =  ((  0x420   << 21U) |  0);
		/* Set up the DLC */
		CAN1->sTxMailBox[0U].TDTR &= 0xFFFFFFF0U;

		CAN1->sTxMailBox[0U].TDTR |= 0x00000000U;
		/* Set up the data field */
		//CAN1->sTxMailBox[0U].TDLR =  (0xAA << 24U) |  (0x55 << 16U) |(0xAA << 8U) | (0x55 );
		//CAN1->sTxMailBox[0U].TDHR =  (0xA5 << 24U) |  (0x5A << 16U) |(0xA5 << 8U) | (0x5A );
		/* Request transmission */
		CAN1->sTxMailBox[0U].TIR  |=  CAN_TI0R_TXRQ;
	}
	if( Time_1_Se_Flag ){

 		if(Keep_80[1] == 0x00){

     		Keep_80[1] = 0x01;
 		}
 		else{
 			HAL_GPIO_WritePin( GPIOB , GPIO_PIN_2 , GPIO_PIN_SET);
 		}
		Time_1_Se_Flag = 0x00;



		// standart CAN message by elik
		/* Set up the Id */
		CAN1->sTxMailBox[1U].TIR =  ((  0x80   << 21U) |  0);
		/* Set up the DLC */
		CAN1->sTxMailBox[1U].TDTR &= 0xFFFFFFF0U;
		CAN1->sTxMailBox[1U].TDTR |= 0x00000008U;
		/* Set up the data field */
		CAN1->sTxMailBox[1U].TDLR =  (0xAA << 24U) |  (0x55 << 16U) |(0xAA << 8U) | (0x55 );
		CAN1->sTxMailBox[1U].TDHR =  (0xA5 << 24U) |  (0x5A << 16U) |(0xA5 << 8U) | (0x5A );
		/* Request transmission */
		CAN1->sTxMailBox[1U].TIR  |=  CAN_TI0R_TXRQ;


		//ask for motor state
		ASK_Motor_1();
		ASK_Motor_2();

	}

	MX_LWIP_Process();

	switch(car_state) // set the car state -- idle ready to drive
	{
	case NUTRAL:
		//if button is push and pedal value is on then move to IGNITION_TO_DRIVE
		if((HAL_GPIO_ReadPin(GPIOB,ready_to_drive_button_Pin)==1) && brak_flag) //=> Button is Pressed
			if(motor_LEFT & motor_RIGHT){
				car_state = IGNITION_TO_DRIVE;
				count = 0;
				tickstart = HAL_GetTick();
				printf("\rpress \n");
			}
		//set motor on
		break;
	case IGNITION_TO_DRIVE:
		printf("ready_to_drive_button_Pin %d brak_flag %d",HAL_GPIO_ReadPin(GPIOB,ready_to_drive_button_Pin) ,brak_flag );
		if((HAL_GPIO_ReadPin(GPIOB,ready_to_drive_button_Pin)==1) && brak_flag ){// && motor_RIGHT && motor_LEFT){ //=> Button is Pressed
			 if ((HAL_GetTick() - tickstart) > 3000U){
					car_state = BUZZER;
					tickstart = HAL_GetTick();
				}
		}
		else
		{
			printf("\DEBUG \n");
			car_state = NUTRAL;
			Start_Motor_1(); //set motor off
			Start_Motor_2(); //set motor off
		}
		//if button is push and pedal value is on and motor state is MO=1 then waite for 100 iteration , and then move to ready to drive state
		break;
	case BUZZER:
		 HAL_GPIO_WritePin(BUZZER_out_GPIO_Port, BUZZER_out_Pin, GPIO_PIN_SET);
		 if ((HAL_GetTick() - tickstart) > 2000U){
			 HAL_GPIO_WritePin(BUZZER_out_GPIO_Port, BUZZER_out_Pin, GPIO_PIN_RESET);
			 car_state = DRIVE; }
		break;
	case DRIVE:
		printf("\rDRIVE \n");
		asm("NOP");
		asm("NOP");
		asm("NOP");




		break;
	case ERROR: break;
	}



  //HAL_Delay(1000);
  //send_msg_to_start_L();
  //send_msg_to_start_R();




#if 0
  scale = 12;

  {
	HAL_ADC_Start(&hadc1);
	val=HAL_ADC_GetValue(&hadc1);
	output=( (val-min_val) / (max_val-min_val) ) * 2*scale;
	//if(output < ) output = 0;
	//output = 0;

	//answer=(char)output+48;
	answer=(char)output;
	//answer = '5';
	asm("NOP");
	printf("\rpedal value %u \n",output);
	send_msg_to_dest(answer);
	send_msg_to_dest2(answer);
	//send_msg_to_dest_R(answer);
	//send_msg_to_dest_L(answer);
#endif
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	 // MX_LWIP_Process();
	  //HAL_Delay(5);

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = ENABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USB_OTG_FS init function */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.ep0_mps = DEP0CTL_MPS_64;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_out_GPIO_Port, BUZZER_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_out_Pin */
  GPIO_InitStruct.Pin = BUZZER_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ready_to_drive_button_Pin */
  GPIO_InitStruct.Pin = ready_to_drive_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ready_to_drive_button_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static void init_CAN1_BGR(void){


CAN_FilterTypeDef FilterConfig;
	  hcan1.Instance = CAN1;
	  hcan1.Init.Prescaler = 3;
	  hcan1.Init.Mode = CAN_MODE_NORMAL;
	  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
	  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	  hcan1.Init.TimeTriggeredMode = DISABLE;
	  hcan1.Init.AutoBusOff = ENABLE;
	  hcan1.Init.AutoWakeUp = ENABLE;
	  hcan1.Init.AutoRetransmission = DISABLE;
	  hcan1.Init.ReceiveFifoLocked = DISABLE;
	  hcan1.Init.TransmitFifoPriority = DISABLE;
	  if (HAL_CAN_Init(&hcan1) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  FilterConfig.FilterIdHigh = 0xFFFFU;
	  FilterConfig.FilterIdLow  = 0xFFFFU;
	  FilterConfig.FilterMaskIdHigh = 0x0000U;
	  FilterConfig.FilterMaskIdLow = 0x0000U;
	  FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	  FilterConfig.FilterBank = 0;
	  FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	  FilterConfig.FilterActivation  = ENABLE;
	  FilterConfig.SlaveStartFilterBank  = 0;

	  HAL_CAN_ConfigFilter( &hcan1,  &FilterConfig );





}


int _write(int file, char *data, int len)
{
      HAL_UART_Transmit(&huart3, (uint8_t*)data, len, 10);
      return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	printf("\r failed to %s in line %d \n",file,line);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
