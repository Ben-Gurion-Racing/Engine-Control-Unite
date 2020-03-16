/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include  <errno.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include  <stdlib.h>
#include  <stdio.h>
#include  <stm32f7xx_hal_can.h>
#include "can.h"
//https://www.st.com/content/ccc/resource/technical/document/reference_manual/group0/96/8b/0d/ec/16/22/43/71/DM00224583/files/DM00224583.pdf/jcr:content/translations/en.DM00224583.pdf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//////////////////////////////////////////////////////// How do I know how to define these variables?

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
volatile unsigned char	Keep_80[16];				//Keep_80[i] indicate if still waiting for a response from unit 'i' to the 0x80 CAN message
volatile unsigned char	Keep_420[16];				//Keep_420[i] indicate if still waiting for a response from unit 'i' to the 0x420 CAN message

//==================== FLAG =========================

volatile unsigned char brak_flag = 0x00;             // This flag means that the brake pedal is pressed
volatile unsigned char motor_LEFT = 0x00;            // This flag means that the left motor is on
volatile unsigned char motor_RIGHT = 0x00;           // This flag means that the left motor is on
volatile int car_volt = -1;
extern uint32_t output;                              // This is a value from 0-100 that indicates how much torque is delivered from the EV pedal
int RPM_r = 0x00;
int RPM_L = 0x00;
double motor_temp_r;                                 // Motor temperature?


//==================== MAIN var =====================
volatile unsigned int count = 0;
static void init_CAN1_BGR(void);                     // Initiating the CAN module?
static void init_CAN_Filter(void);

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
int _write(int file, char *ptr, int len);            // Transmiting some data?
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	//while(1);
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
#if 1
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

#if 0
  /**
   * This is a manual initial of the STM32 devices.
   * this was done by Elik Rubin to setup the CAN1.
   * in 6/feb/2020 Avishai Vaisman changed the CAN1
   * properties so we can use the STMcube setup.
   * NOTE: Elik's CanFilter propperties added manualy
   */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_LWIP_Init();
  MX_ADC1_Init();
  init_CAN1_BGR();                                     // CAN1 initialize by the user
#endif

	HAL_GPIO_WritePin( GPIOB , GPIO_PIN_2|LD2_Pin , GPIO_PIN_RESET);  // Not sure why is it here?

  //=================== CAN =========================
  __HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);        // Enable CAN1 interrupts.
  HAL_CAN_Start( &hcan1 );                                        // Start the CAN module.
  #if 1
  // standart CAN message by elik - Open decelarations to understand meanings
      CAN1->sTxMailBox[1U].TIR =  ((  0x80   << 21U) |  0);           // Set up the Id
      /* Set up the DLC */
      CAN1->sTxMailBox[1U].TDTR &= 0xFFFFFFF0U;
      CAN1->sTxMailBox[1U].TDTR |= 0x00000008U;
      /* Set up the data field */
      CAN1->sTxMailBox[1U].TDLR =  (0xAA << 24U) |  (0x55 << 16U) |(0xAA << 8U) | (0x55 ); //TDLR=0xAA55AA55
      CAN1->sTxMailBox[1U].TDHR =  (0xA5 << 24U) |  (0x5A << 16U) |(0xA5 << 8U) | (0x5A ); //TDLR=0xA55AA55A
      /* Request transmission */
      CAN1->sTxMailBox[1U].TIR  |=  CAN_TI0R_TXRQ;
  #endif

//====================== ETH ========================
IP4_ADDR(&destIPAddr,192,168,1,49);                  // Set an IP address for the ETHERNET?

//Start_Motor_1(); //set motor off
//Start_Motor_2(); //set motor off


#if 1 //debug
car_state = NUTRAL;                                 // initial the state; TODO write the value to the flash a
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while(1){

	//if(can_reacive)
	//get meassage

	if( Time_5_Ms_Flag ){                           // Something to do every 5ms?
		Time_5_Ms_Flag = 0x00;                      // Flag reset
 		if(Keep_420[1] == 0x00){                    // Not sure?
     		Keep_420[1] = 0x01;                     // Not sure?
 		}
 		else{
 			HAL_GPIO_WritePin( GPIOB , GPIO_PIN_2 , GPIO_PIN_SET);   // Not sure why to right this pin..
 		}


		// standart CAN 420 message by elik - ***Need to write what is 420 message***
		CAN1->sTxMailBox[0U].TIR =  ((  0x420   << 21U) |  0);           // Set up the Id
		/* Set up the DLC */
		CAN1->sTxMailBox[0U].TDTR &= 0xFFFFFFF0U;
		CAN1->sTxMailBox[0U].TDTR |= 0x00000000U;
		/* Set up the data field */
		//CAN1->sTxMailBox[0U].TDLR =  (0xAA << 24U) |  (0x55 << 16U) |(0xAA << 8U) | (0x55 );
		//CAN1->sTxMailBox[0U].TDHR =  (0xA5 << 24U) |  (0x5A << 16U) |(0xA5 << 8U) | (0x5A );
		/* Request transmission */
		CAN1->sTxMailBox[0U].TIR  |=  CAN_TI0R_TXRQ;

	}
	if( Time_1_Se_Flag ){                           // Something to do every 1 Sec?
		Time_1_Se_Flag = 0x00;                      // Flag reset
 		if(Keep_80[1] == 0x00){                     // Not sure?
     		Keep_80[1] = 0x01;                      // Flag reset?
 		}
 		else{
 			HAL_GPIO_WritePin( GPIOB , GPIO_PIN_2 , GPIO_PIN_SET); // Not sure?
 		}


		// standart CAN 80 message by elik ***Need to write what is 80 message***
		CAN1->sTxMailBox[1U].TIR =  ((  0x80   << 21U) |  0);           // Set up the Id
		/* Set up the DLC */
		CAN1->sTxMailBox[1U].TDTR &= 0xFFFFFFF0U;
		CAN1->sTxMailBox[1U].TDTR |= 0x00000008U;
		/* Set up the data field */
		CAN1->sTxMailBox[1U].TDLR =  (0xAA << 24U) |  (0x55 << 16U) |(0xAA << 8U) | (0x55 );
		CAN1->sTxMailBox[1U].TDHR =  (0xA5 << 24U) |  (0x5A << 16U) |(0xA5 << 8U) | (0x5A );
		/* Request transmission */
		CAN1->sTxMailBox[1U].TIR  |=  CAN_TI0R_TXRQ;

		/* ask for motor state */
		ASK_Motor_1();
		ASK_Motor_2();


		if(car_state != DRIVE){
			send_msg_to_dest(0);
			send_msg_to_dest2(0);
		}

		if(motor_RIGHT == 0 )
			Start_Motor_1();

		if(motor_LEFT == 0 )
			Start_Motor_2();

		ASK_Motor_RPM_L();
		ASK_Motor_RPM_r();
		ASK_Motor_temp_R();




		printf("\n");
		printf("\n");
		printf("\r car_state 	= 	%d \n" ,car_state);
		printf("\r motor_RIGHT 	=	%d \n", motor_RIGHT);
		printf("\r motor_LEFT	=	%d \n", motor_LEFT);
		printf("\r output 	=	%d \n",output);
		printf("\r brak_flag 	=	%d \n",brak_flag);
		printf("\r ready_to_drive_button_Pin	=	%d \n",HAL_GPIO_ReadPin(GPIOB,ready_to_drive_button_Pin));
		printf("\r RPM_r 	=	%d \n",RPM_r);
		printf("\r RPM_L 	=	%d \n",RPM_L);
		printf("\r motor_temp_r 	=	%f \n",motor_temp_r);

		printf("\n");
		printf("\n");
	}



	MX_LWIP_Process();

	switch(car_state) // set the car state -- idle ready to drive
	{
	case NUTRAL:
		//if button is push and pedal value is on then move to IGNITION_TO_DRIVE
		if((HAL_GPIO_ReadPin(GPIOB,ready_to_drive_button_Pin)==0) && brak_flag) //=> Button is Pressed
			if(motor_LEFT  == 0 || motor_RIGHT == 0){
				car_state = NUTRAL;
				printf("\r Start_Motor \n");
				Start_Motor_1(); //set motor on
				Start_Motor_2(); //set motor on
				HAL_Delay(200);
			}
			else
			{
				//printf("\r IGNITION_TO_DRIVE \n");
				car_state = IGNITION_TO_DRIVE;
				tickstart = HAL_GetTick();
			}



		//set motor on
		break;
	case IGNITION_TO_DRIVE:
		if((HAL_GPIO_ReadPin(GPIOB,ready_to_drive_button_Pin)==0) && brak_flag ){// && motor_RIGHT && motor_LEFT){ //=> Button is Pressed
			 if ((HAL_GetTick() - tickstart) > 3000U){
					car_state = BUZZER;
					tickstart = HAL_GetTick();
				}
		}
		else
			car_state = NUTRAL;

		//if button is push and pedal value is on and motor state is MO=1 then waite for 100 iteration , and then move to ready to drive state
		break;
	case BUZZER:
		 HAL_GPIO_WritePin(BUZZER_out_GPIO_Port, BUZZER_out_Pin, GPIO_PIN_SET);			//turn buzzer on
		 if ((HAL_GetTick() - tickstart) > 2000U){										//check how much time elapse
			 HAL_GPIO_WritePin(BUZZER_out_GPIO_Port, BUZZER_out_Pin, GPIO_PIN_RESET);	//turn buzzer off
			 car_state = DRIVE; }
		break;
	case DRIVE:
		if(motor_RIGHT == 0 || motor_LEFT == 0){
			car_state = ERROR_state;
			tickstart = HAL_GetTick();
		}

		break;
	case ERROR_state:
	if(motor_RIGHT == 1 && motor_LEFT == 1)
				car_state = DRIVE;
	if ((HAL_GetTick() - tickstart) > 5000U)
		 car_state = NUTRAL;

	 break;

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
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
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
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
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
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
  /* USER CODE BEGIN CAN1_Init 2 */

  init_CAN_Filter();	//Can filter properties set by Elik Rubin

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

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
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ready_to_drive_button_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/** static void init_CAN1_BGR(void)
 * CAN1 properties set by Elik Rubin
 */
static void init_CAN1_BGR(void){                // Initiating the CAN module?

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
	  init_CAN_Filter();
}

/**static void init_CAN_Filter(void)
 * Can filter properties set by Elik Rubin
 */
static void init_CAN_Filter(void){
	CAN_FilterTypeDef FilterConfig;
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
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	//printf("\r failed to %s in line %d \n",file,line);
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
