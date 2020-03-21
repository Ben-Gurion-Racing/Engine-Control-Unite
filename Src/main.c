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

//---------------------- References ----------------------
// STM manual reference:
// https://www.st.com/content/ccc/resource/technical/document/reference_manual/group0/96/8b/0d/ec/16/22/43/71/DM00224583/files/DM00224583.pdf/jcr:content/translations/en.DM00224583.pdf
// ELMO command reference:
// https://s3.us-west-2.amazonaws.com/secure.notion-static.com/2d9ef1b5-4c83-45ac-8248-e25a7c62fbe6/MAN-G-CR.pdf?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=ASIAT73L2G45PJP3TSGT%2F20200301%2Fus-west-2%2Fs3%2Faws4_request&X-Amz-Date=20200301T215231Z&X-Amz-Expires=86400&X-Amz-Security-Token=IQoJb3JpZ2luX2VjEEMaCXVzLXdlc3QtMiJIMEYCIQDvLvdFmWJIns%2BzEcaxYaUIc3V1ZRSdHHLISQlI5pBAJQIhAIUxmJt55WN44353wgPZqdxYCo7MePuPUHpivgwMJYVJKrQDCBwQABoMMjc0NTY3MTQ5MzcwIgzMkJtShobXrR9VGuAqkQNq0MlyzJc73Cj%2BgiqVLmqP0GJ5q%2BK0og5KNiOetQDppGhYSL0VzfcPiPP5jK4ISiPGI%2BdVK6x3l8K%2Bdwj%2B%2FGRrqLfY9cVeGG94ADHTJHRHhIK9eLwSov%2FAp%2BExDJWxaIsZr%2FXy%2BOZt%2BfvEQMSvr1JjEoxvigmy7z0SgwXAYqhESKYYpCYZiE3bR9enUWf9TGXUqC1QNYj6VWW90SQA%2F5DGbzqxtHFu0pmSXtg5n%2BxBnz%2BGs5isyaXhonQkY7fOApFiuG7XIkiel5GxWmcTZ4pMue5%2FJi9V5Be5IUgK4pxriHd3aOG297lOEUSCKSrgB0KQYAYoUkRE7A%2Fe6t0l8FWlEilYm6qhoAvSZ0lc%2FWfx9pIDmN7KNlr3PyPnFQtJZZEjbK9%2FO9Guxk6GUGLmwEVZAS%2FDhtoKg7vJgFJ6m8tjJUxbvYtXyKOnDLFITf%2FBtmWzOpbAP1YYsngb2Qddm%2F87MNAaNZ4cSve9DFFmd5Y%2FrHYqCaU5Mjvob%2Fxq3ExyXJsTAQb5QV04YLouTxSTPYi1dzCfgPDyBTrqAYXPtKBg0qdtxWv4SqCyy6ubed155k80J%2BRc04edkuBfZbx%2FtO%2BMD1KEZt2Y3QkC1bcsQqWGPMXcslcoFvzlX%2BU%2BqlFJ0usmqkYIszPWYlDCSgukn9UPpYA%2F%2Fx8pNe%2FGHvfQqY5Bpe2EYHQfgLESObtX123swAhdLyaHFQmXU4obL7m8PeLLAFBxrtTGHizo%2Bg1JHnoGsH51O6Zj4CmWihxISfQQRRtd%2B0INVKjjaL7sCOnTweRR3XiLhR96iFUkQYrtQaGNxvt1tgh75Ml7Io3DptScvU%2FvTGfXUbrXddZw7tjfL5o3hlCcMQ%3D%3D&X-Amz-Signature=c1b554a835f59851f3af63e08b89099f9543d5c2238813efbbcea4a7f08eb9fb&X-Amz-SignedHeaders=host&response-content-disposition=filename%20%3D%22Command%2520Reference%2520for%2520ELMO%2520drivers.pdf%22
// https://www.st.com/content/ccc/resource/technical/document/user_manual/65/e8/20/db/16/36/45/f7/DM00103685.pdf/files/DM00103685.pdf/jcr:content/translations/en.DM00103685.pdf
// Other relevant files - lwip.c , stm32f7xx_it.c , ethernetif.c


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

ip4_addr_t destIPAddr;
//===================== CAN ============================
volatile unsigned char CAN_1_Rx_received_flg = 0x00; // Represent that a CAN message has been received
volatile unsigned char CAN_1_RecData[8];             // An array to hold the data from the CAN message
unsigned long CAN_1_specific_id_test  = 0x00; // not sure - didn't see it at other parts of the code
unsigned long CAN_1_temp_id; // not sure - didn't see it at other parts of the code
unsigned int  CAN_1_eid;                             // Represent that the CAN message has an extended identifier(look up at reference manual)
unsigned int  CAN_1_sid;							 // Represent that the CAN message has a standard identifier(look up at reference manual)
uint32_t CAN_1_ide; 								 // CAN definitions - check manual reference
uint32_t CAN_1_rtr; 								 // CAN definitions - check manual reference
uint32_t CAN_1_dlc; 								 // CAN definitions - check manual reference
uint32_t CAN_1_fmi; 								 // CAN definitions - check manual reference
volatile unsigned car_state;                         // This variable indicates the car's driving state
volatile uint32_t tickstart = 0U;                    // Defines a 32bit unsigned clock variable
//===================== TIME ===========================
volatile unsigned char	Time_1_Ms_Flag = 0x00;
volatile unsigned char	Time_5_Ms_Flag = 0x00;       // This flag elapses every 5ms - used for 420 functions(motor outputs and etc)
volatile unsigned char	Time_500_Ms_Flag = 0x00;     // This flag elapses every 0.5s - used for 80 message - checks online users
volatile unsigned int	WatchDog_420=0;
//===================== KEEP ===========================
volatile unsigned char	Keep_80[16];				 // Keep_80[i] indicate if still waiting for a response from unit 'i' to the 0x80 CAN message
volatile unsigned char	Keep_420[16];				 // Keep_420[i] indicate if still waiting for a response from unit 'i' to the 0x420 CAN message
//===================== FLAG ===========================
volatile unsigned char brak_flag = 0x00;             // This flag means that the brake pedal is pressed
volatile unsigned char motor_LEFT = 0x00;            // This flag means that the left motor is on or off
volatile unsigned char motor_RIGHT = 0x00;           // This flag means that the right motor is on or off
volatile int car_volt = -1; // not sure - didn't see it at other parts of the code
extern uint32_t output;                              // This is a value from 0-100 that indicates how much torque is delivered from the EV pedal
int RPM_r = 0x00;
int RPM_L = 0x00;
double motor_temp_r;                                 // Motor temperature
volatile int ErrorState=ERROR_DontSHTDWN;			 // There are 2 cases of errors at Safe state detailed at main.h
//===================== MAIN var =======================
volatile unsigned int count = 0; // not sure - didn't see it at other parts of the code
static void init_CAN1_BGR(void);                     // Initiating the CAN module
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
int _write(int file, char *ptr, int len); // not sure
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
  #if 1 // not sure
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

  HAL_GPIO_WritePin( GPIOB , GPIO_PIN_2|LD2_Pin , GPIO_PIN_RESET);    // not sure

  //===================== CAN ============================
  __HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);            // Enable CAN1 interrupts.
  HAL_CAN_Start( &hcan1 );                                            // Start the CAN module.
 #if 1
  // Standard CAN message by elik - Open decelerations to understand meanings
  CAN1->sTxMailBox[1U].TIR =  ((  0x80   << 21U) |  0);               // Set up the Id for an empty mailbox(mailbox 1 in this line)
  /* Set up the DLC - number of bytes of data being transmitted */
  CAN1->sTxMailBox[1U].TDTR &= 0xFFFFFFF0U;
  CAN1->sTxMailBox[1U].TDTR |= 0x00000008U;
  /* Set up the data field */
  CAN1->sTxMailBox[1U].TDLR =  (0xAA << 24U) |  (0x55 << 16U) |(0xAA << 8U) | (0x55 ); //TDLR=0xAA55AA55
  CAN1->sTxMailBox[1U].TDHR =  (0xA5 << 24U) |  (0x5A << 16U) |(0xA5 << 8U) | (0x5A ); //TDHR=0xA55AA55A
  /* Request transmission bit - look up reference manual */
  CAN1->sTxMailBox[1U].TIR  |=  CAN_TI0R_TXRQ;
 #endif

  //===================== ETH ============================
  IP4_ADDR(&destIPAddr,192,168,1,49);               // Set an IP address for the ETHERNET

  //Start_Motor_1(); //set motor off
  //Start_Motor_2(); //set motor off

  #if 1 //debug
   car_state = NUTRAL;                              // initial the state; TODO write the value to the flash a
  #endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while(1){

	//if(can_reacive)
	//get message

	if(Time_5_Ms_Flag){                             // "Driving Loop" - every 5ms check the APPS state
		Time_5_Ms_Flag = 0x00;                      // Flag reset
 		if(Keep_420[1] == 0x00){  					// If the ECU is not already waiting for a 420 message answer(421 message) => make it wait for one
     		Keep_420[1] = 0x01;   					// Set the the flag to 1 => ECU is waiting for a 420 message answer
     		WatchDog_420=0;							// Reset 420 delay watchdog
 		}
 		else{										// Else there is a delay of more than 5ms for the 420 message => open shutdown
 			HAL_GPIO_WritePin( GPIOB , GPIO_PIN_2 , GPIO_PIN_SET);   // Not sure - maybe an ERROR pin for shutdown? since its waiting and havn't gotten for 5ms?
 			// Change this pin to LED so we can identify this problem?
 			WatchDog_420++;							// Count 420 delay watchdog for every 5ms
 			if (WatchDog_420>20)					// If 100ms has elapsed => open shutdown circuit
 				OpenShutDownError();				// Enter Safe state and open shut down circuit
 	 		}

		// standard CAN 420 message by elik - Check the pedals state from the APPS's STM
		CAN1->sTxMailBox[0U].TIR =  ((  0x420   << 21U) |  0);       // Set up the Id for an empty mailbox(mailbox 0 in this line)
		/* Set up the DLC - number of bytes of data being transmitted */
		CAN1->sTxMailBox[0U].TDTR &= 0xFFFFFFF0U;
		CAN1->sTxMailBox[0U].TDTR |= 0x00000000U;
		/* Set up the data field */
		//CAN1->sTxMailBox[0U].TDLR =  (0xAA << 24U) |  (0x55 << 16U) |(0xAA << 8U) | (0x55 );
		//CAN1->sTxMailBox[0U].TDHR =  (0xA5 << 24U) |  (0x5A << 16U) |(0xA5 << 8U) | (0x5A );
		/* Request transmission bit - look up reference manual */
		CAN1->sTxMailBox[0U].TIR  |=  CAN_TI0R_TXRQ;
	}
	if(Time_500_Ms_Flag){                           // This section prints car & motor state to the console(to the user)
		Time_500_Ms_Flag = 0x00;                    // Flag reset
 		if(Keep_80[1] == 0x00){                     // not sure? maybe check if the ECU is waiting for a 80 message?
     		Keep_80[1] = 0x01;                      // not sure? maybe if its not waiting - than flag that it is waiting for a 80 message?
 		}
 		else{
 			HAL_GPIO_WritePin( GPIOB , GPIO_PIN_2 , GPIO_PIN_SET);   // Not sure - maybe an ERROR pin for shutdown? since its waiting and havn't gotten for 1s?
 			// Change this pin to LED so we can identify this problem?
 			OpenShutDownError();	   								 // Enter Safe state and open shut down circuit
 		}

		// standard CAN 80 message by elik - Check if the other STM's on the CAN network are connected
		CAN1->sTxMailBox[1U].TIR =  ((  0x80   << 21U) |  0);        // Set up the Id for an empty mailbox(mailbox 1 in this line)
		 /* Set up the DLC - number of bytes of data being transmitted */
		CAN1->sTxMailBox[1U].TDTR &= 0xFFFFFFF0U;
		CAN1->sTxMailBox[1U].TDTR |= 0x00000008U;
		/* Set up the data field */
		CAN1->sTxMailBox[1U].TDLR =  (0xAA << 24U) |  (0x55 << 16U) |(0xAA << 8U) | (0x55 ); //TDLR=0xAA55AA55
		CAN1->sTxMailBox[1U].TDHR =  (0xA5 << 24U) |  (0x5A << 16U) |(0xA5 << 8U) | (0x5A ); //TDHR=0xA55AA55A
		/* Request transmission bit - look up reference manual */
		CAN1->sTxMailBox[1U].TIR  |=  CAN_TI0R_TXRQ;

		ASK_Motor_1();                              // Ask for motor 1 state
		ASK_Motor_2();                              // Ask for motor 2 state
		if(car_state != DRIVE){                     // If the car is not in DRIVE STATE => send 0 torque to the motors
			send_msg_to_dest(0);                    // Send 0 torque to motor right
			send_msg_to_dest2(0);                   // Send 0 torque to motor left
		}
		if(motor_RIGHT == 0)                        // If motor right is off => start it(not sure why every second, what if the car is at SAFE STATE?)
			Start_Motor_1();                        // Start motor right
		if(motor_LEFT == 0)                         // If motor left is off => start it(not sure why every second, what if the car is at SAFE STATE?)
			Start_Motor_2();                        // Start motor left
		ASK_Motor_RPM_L();
		ASK_Motor_RPM_r();
		ASK_Motor_temp_R();

        // Prints to the console
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
	MX_LWIP_Process();                              // not sure

	// Car state menu
	switch(car_state) // set the car state -- idle ready to drive
	{
	case NUTRAL:
		if((HAL_GPIO_ReadPin(GPIOB,ready_to_drive_button_Pin)==0) && brak_flag)  // Check if Ready2Drive && brake pedal are pressed
			if(motor_LEFT  == 0 || motor_RIGHT == 0){                            // If one of the motor is off => start them and stay at NUTRAL for 200ms
				car_state = NUTRAL;
				printf("\r Start_Motor \n");
				Start_Motor_1(); 												 // Set motor right on
				Start_Motor_2(); 												 // Set motor left on
				HAL_Delay(200);                                                  // Creates a 200ms delay(not sure why 200)
			}
			else                                                                 // If the motors are on => move to IGNITION2DRIVE state
			{
				//printf("\r IGNITION_TO_DRIVE \n");
				car_state = IGNITION_TO_DRIVE;
				tickstart = HAL_GetTick();                                       // Start a timer for the ignition state
			}
		//set motor on
	break;
	case IGNITION_TO_DRIVE:
		if((HAL_GPIO_ReadPin(GPIOB,ready_to_drive_button_Pin)==0) && brak_flag){ // Check if Ready2Drive && brake pedal are pressed
			 if ((HAL_GetTick() - tickstart) > 3000U){                           // If Ready2Drive && brake pedal are pressed for 3s => move to BUZZER state
					car_state = BUZZER;
					tickstart = HAL_GetTick();                                   // Start a timer for the BUZZER state
				}
		}
		else                                                                     // If the buttons are released => return to NUTRAL state
			car_state = NUTRAL;
		//if button is push and pedal value is on and motor state is MO=1 then wait for 100 iteration, and then move to ready to drive state
	break;
	case BUZZER:
		 HAL_GPIO_WritePin(BUZZER_out_GPIO_Port, BUZZER_out_Pin, GPIO_PIN_SET);			// turn buzzer ON
		 if ((HAL_GetTick() - tickstart) > 2000U){										// If 2s have elapsed
			 HAL_GPIO_WritePin(BUZZER_out_GPIO_Port, BUZZER_out_Pin, GPIO_PIN_RESET);	// Turn buzzer OFF
			 car_state = DRIVE;                                                         // Proceed to DRIVE state
		 }
	break;
  // This is the where the MAIN LOOP occurs, while driving the STM receives values from the pedals
  // and transmit them to the motor by FIFO0 interrupts(look up stm32f7xx_it.c file)
	case DRIVE:
		if(motor_RIGHT == 0 || motor_LEFT == 0){                                 // If one of the motors doesn't work => go to Safe State
			car_state = SAFE_STATE;
			ErrorState=ERROR_DontSHTDWN;										 // This error doesn't open shutdown circuit
			tickstart = HAL_GetTick();                                           // Start a timer for the Safe State
		}
	break;
	case SAFE_STATE:
		if (ErrorState==ERROR_DontSHTDWN)										 // The error doesn't open shutdown circuit
		{
			if(motor_RIGHT == 1 && motor_LEFT == 1)                              // If the motors work => return to DRIVE state
				car_state = DRIVE;
			if ((HAL_GetTick() - tickstart) > 5000U)                             // If 5sec has passed in Safe State => return to NUTRAL state
				car_state = NUTRAL;
				ErrorState=ERROR_OpenSHTDWN;									 // Reset the error state flag
		}
		else																	 // Open shutdown circuit
			ErrorState=ERROR_OpenSHTDWN;
			OpenShutDownError();	   								 // Enter Safe state and open shut down circuit
			// Need to add an open shutdown circuit pin here
			// HAL_GPIO_WritePin( GPIOB , GPIO_PIN_5 , GPIO_PIN_SET); // Need to choose a free pin
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

  } // not sure that these suppose to be here
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_10|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_out_GPIO_Port, BUZZER_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|LD2_Pin;
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
static void init_CAN1_BGR(void){                // Initiating the CAN module made by elik - not sure whats in here

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
 */
static void init_CAN_Filter(void){              // Can filter properties set by Elik Rubin - not sure whats in here
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
