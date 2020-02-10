/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
  
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define APPS_0_VALUE (0xFF00 & ( CAN_1_RecData[1] << 8 )) | (0x00FF & (CAN_1_RecData[0] ) );
#define APPS_1_VALUE (0xFF00 & ( CAN_1_RecData[3] << 8 )) | (0x00FF & (CAN_1_RecData[2] ) );
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//=================== ETH ============================
static struct udp_pcb* upcb;
static struct udp_pcb* upcb2;

//=================== CAN 1 =========================
extern volatile unsigned char CAN_1_Rx_received_flg;
extern volatile unsigned char CAN_1_RecData[8];
unsigned char 	CAN_1_Time_Stemp_420[2];
unsigned char	CAN_1_Time_Stemp_80[2];
unsigned long	CAN_1_Rx_ID  = 0x00;
unsigned long 	CAN_1_Rx_temp_id;
unsigned int  	CAN_1_Rx_eid;
unsigned int  	CAN_1_Rx_sid;
unsigned long 	CAN_1_Tx_temp_id;

/**
 * CAN_1_Rx_ide is the Identifier extension in the CAN receive FIFO mailbox identifier register
 * This bit defines the identifier type of message in the mailbox.
 * 0: Standard identifier.
 * 1: Extended identifier.
 * REFERENCE MANUAL page  1566.
 */
uint32_t 		CAN_1_Rx_ide;

uint32_t 		CAN_1_Rx_rtr;	/*!< CAN receive FIFO mailbox identifier register> */

uint32_t 		CAN_1_Rx_dlc;	/*!< CAN receive FIFO mailbox data length control register> */

/**
 * CAN_1_Rx_fmi is the CAN receive FIFO mailbox Filter match index
 *	This register contains the index of the filter the message stored in the mailbox passed through.
 *	For more details on identifier filtering refer to:
 *	Section 40.7.4: Identifier filtering on REFERENCE MANUAL page 1542
 */
uint32_t 		CAN_1_Rx_fmi;




//==================== TIME =========================
extern volatile unsigned char	Time_1_Ms_Flag;
extern volatile unsigned char	Time_5_Ms_Flag;
extern volatile unsigned char	Time_1_Se_Flag;
unsigned int					Time_1_Ms_Counter = 0;
unsigned int					Time_1_Se_Counter = 0;

//==================== KEEP =========================
extern volatile unsigned char	Keep_80[16];
extern volatile unsigned char	Keep_420[16];

//==================== FLAG =========================

extern volatile unsigned char brak_flag ;
extern volatile unsigned car_state;

//=============== APPS VALIDATION ===================
/*--T11.9.2(c) Failures of sensor signals used in programmable devices--*/
const uint16_t appsMax[3] = {
		APPS_0_MAX,	//the max value of apps 0 in its mechanical range of movement
		APPS_1_MAX,	//the max value of apps 1 in its mechanical range of movement
		APPS_2_MAX
};
const uint16_t appsMin[3] = {
		APPS_0_MIN,	//the min value of apps 0 in its mechanical range of movement
		APPS_1_MIN,	//the min value of apps 1 in its mechanical range of movement
		APPS_2_MIN
};

uint32_t apps[2];							//will hold the value of apps0 and apps1

uint32_t val;
uint32_t output;
double max_val[3] = {APPS_0_MAX , APPS_1_MAX , APPS_2_MAX};
double min_val[3] = {APPS_0_MIN , APPS_1_MIN,APPS_2_MIN};
uint32_t appsOutput[3];
double scale = OUTPUT_SCALE;

/* External variables --------------------------------------------------------*/
extern ETH_HandleTypeDef heth;
extern CAN_HandleTypeDef hcan1;

int msec_5 = 0;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ETH_HandleTypeDef heth;
extern CAN_HandleTypeDef hcan1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */


	Time_1_Ms_Counter++;
	Time_1_Ms_Flag = 0x01;
	if( !(Time_1_Ms_Counter % 5) ){
		Time_5_Ms_Flag = 0x01;
	}
	if(Time_1_Ms_Counter >= 1000){
		Time_1_Ms_Counter = 0;
		Time_1_Se_Counter++;
		Time_1_Se_Flag = 0x01;
	}


  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */




  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 TX interrupts.
  */
void CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_TX_IRQn 0 */

  /* USER CODE END CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_TX_IRQn 1 */


  /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
	uint32_t errorcode = HAL_CAN_ERROR_NONE;
	uint32_t interrupts = READ_REG(CAN1->IER);
	uint32_t msrflags = READ_REG(CAN1->MSR);
	uint32_t tsrflags = READ_REG(CAN1->TSR);
	uint32_t rf0rflags = READ_REG(CAN1->RF0R);
	uint32_t rf1rflags = READ_REG(CAN1->RF1R);
	uint32_t esrflags = READ_REG(CAN1->ESR);


  /* Receive FIFO 0 message pending interrupt management *********************/  //definition in reference manual page  1552
  if ((interrupts & CAN_IT_RX_FIFO0_MSG_PENDING) != 0U)  //Check if the interrupt came due to a FIFO message interrupt on FIFO 0 register
  {
	/* Check if message is still pending */		//the FMP0 bits ([0,1]) in the FIFO0 register indicate how many messages are pending (REFERENCE MANUAL page  1558)
	if ((CAN1->RF0R & CAN_RF0R_FMP0) != 0U)
	{
		/* RIR == Receive Identifier Register: REFERANCE MANUAL page  1566*/
		CAN_1_Rx_ide = (uint8_t)0x04U & CAN1->sFIFOMailBox[0].RIR;
		if ( CAN_1_Rx_ide == CAN_ID_STD )										//if true than the message has Standard identifier
		{
			CAN_1_Rx_eid = 0x00;												//no need in extended identifier
			CAN_1_Rx_sid = 0x000007FFU & (CAN1->sFIFOMailBox[0].RIR >> 21U);	//gets the bits of the standard identifier [31:21] in RIR

			/**** Setting data for a message with standart identifier ****/
			CAN_1_Rx_rtr = (uint8_t)0x02U & CAN1->sFIFOMailBox[0].RIR;
			CAN_1_Rx_dlc = (uint8_t)0x0FU & CAN1->sFIFOMailBox[0].RDTR;
			CAN_1_Rx_fmi = (uint8_t)0xFFU & (CAN1->sFIFOMailBox[0].RDTR >> 8U);
																	// | 	0x421:		|
			CAN_1_RecData[0] = CAN1->sFIFOMailBox[0].RDLR;			// | APPS_0 LSB		|
			CAN_1_RecData[1] = CAN1->sFIFOMailBox[0].RDLR >> (8U);	// | APPS_0 MSB		|
			CAN_1_RecData[2] = CAN1->sFIFOMailBox[0].RDLR >> (16U);	// | APPS_1 LSB		|
			CAN_1_RecData[3] = CAN1->sFIFOMailBox[0].RDLR >> (24U);	// | APPS_1 MSB		|
			CAN_1_RecData[4] = CAN1->sFIFOMailBox[0].RDHR;			// | APPS_2 LSB		|
			CAN_1_RecData[5] = CAN1->sFIFOMailBox[0].RDHR >> (8U);	// | APPS_2 MSB		|
			CAN_1_RecData[6] = CAN1->sFIFOMailBox[0].RDHR >> (16U);	// | BPPS_Signal	|
			CAN_1_RecData[7] = CAN1->sFIFOMailBox[0].RDHR >> (24U);	// | valid_Apps_Bpps|
		}
		else																	//than message has extended identifier
		{
			CAN_1_Rx_sid = 0x00;												//no need in standard identifier
			CAN_1_Rx_eid = 0x1FFFFFFFU & (CAN1->sFIFOMailBox[0].RIR >> 3U);		// gets the bits of the extended identifier [31:3] in RIR
		}
		switch(CAN_1_Rx_sid)													//Handling standard identifier message
		{
			case 0x401  :

				 //TODO elik

			break;
			case 0x421 :	//Answer for the 0x420 message (getting samples of the pedals sensors from the pedals unite)

				if( CAN_1_RecData[7] == ERROR_APPS_BPPS_TIMEOUT){			 //cheak_if_pedal_valid

					/* Set up the Id */
					CAN1->sTxMailBox[2U].TIR =  ((  0x400   << 21U) |  0);
					/* Set up the DLC */
					CAN1->sTxMailBox[2U].TDTR &= 0xFFFFFFF0U;
					CAN1->sTxMailBox[2U].TDTR |= 0x00000000U;
					/* Set up the data field */
					//CAN1->sTxMailBox[0U].TDLR =  (0xAA << 24U) |  (0x55 << 16U) |(0xAA << 8U) | (0x55 );
					//CAN1->sTxMailBox[0U].TDHR =  (0xA5 << 24U) |  (0x5A << 16U) |(0xA5 << 8U) | (0x5A );
					/* Request transmission */
					CAN1->sTxMailBox[2U].TIR  |=  CAN_TI0R_TXRQ;
				}
				else
				{
					output = 0;								//START FROM SAFE MODE
					appsOutput[0]= 0;
					appsOutput[1]= 0;
					appsOutput[2]= 0;

					Keep_420[1] = 0x00;
					apps[0] = APPS_0_VALUE;
					apps[1] = APPS_1_VALUE;
					if( CAN_1_RecData[6] == 0x01 ){  // check if break pedal is pressed
						brak_flag = 1;
					}
					else{							// Check The APPS
						if(apps[0] == ERROR_APPS_MAXVALUE || apps[1] == ERROR_APPS_MAXVALUE ){
							printf("CAN1_Rx IT: ERROR_APPS_MAXVALUE");
						}


						else if (   apps[0] > appsMax[0]		//Check if the apps value is plausible
								 ||	apps[0] < appsMin[0]
								 || apps[1] > appsMax[1]
								 || apps[1] < appsMin[1] ){

							printf("CAN1_Rx IT: Plausibility check failed");
						}
						else//drive :)
						{
							#if 0	//this will be the next output calculation (by Avishai)
							// both apps are inverted to each other and have the same travel range
							// therefore they are compleating each other to 100%
							getOutput(0); 									//put the travel precentage of APPS0 in appsOutput[0]
							getOutput(1); 									//put the travel precentage of APPS1 in appsOutput[1]
							if( ( appsOutput[0] + appsOutput[1] ) < 90)		//T11.8.9 : Implausibility is defined as a deviation of more than ten percentage points pedal travel between any of the used APPSs
								output = 0;
							else
								output = appsOutput[0];
							#endif
							val = apps[0] ;
							output=( (val-min_val[0]) / (max_val[0]-min_val[0]) ) * scale;

							if(output > 100) output = 100;
							//output = output && 0x00FF;
#if 0
					CAN1->sTxMailBox[2U].TIR =  ((  0x60   << 21U) |  0);
					/* Set up the DLC */
					CAN1->sTxMailBox[2U].TDTR &= 0xFFFFFFF0U;
					CAN1->sTxMailBox[2U].TDTR |= 0x00000001U;
					/* Set up the data field */
					CAN1->sTxMailBox[2U].TDLR =  output;
					CAN1->sTxMailBox[2U].TDHR =  val;
					/* Request transmission */
					CAN1->sTxMailBox[2U].TIR  |=  CAN_TI0R_TXRQ;
#endif
						}
					}
					//push pedal to array[10];
					//get the median_of _the_array

					//if state is DRIVE then send command upcb2
					//send_TC( upcb  , destIPAddr , 5001 , output );

					if(car_state == DRIVE ){//DRIVE){
						send_msg_to_dest2(output);
						send_msg_to_dest(output);
						//send_msg_to_dest2_temp( output);
					}
				}
			break;

			case 0x81  :  //indicate there is a communication between the ECU and PU
			//HAL_GPIO_WritePin( GPIOB , GPIO_PIN_2|LD3_Pin , GPIO_PIN_SET);


			   if( (CAN_1_RecData[0] == 0x55)
					   && (CAN_1_RecData[1] == 0x55)
					   && (CAN_1_RecData[2] == 0x55)
					   && (CAN_1_RecData[3] == 0x55)
					   && (CAN_1_RecData[4] == 0x55)
					   && (CAN_1_RecData[5] == 0x55)
					   && (CAN_1_RecData[6] == 0x55)
					   && (CAN_1_RecData[7] == 0x55)  ){
				   Keep_80[1] = 0x00;
			   }

			break;


			default :
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			asm("NOP");
			}

	SET_BIT(CAN1->RF0R, CAN_RF0R_RFOM0);   //release message from FIFO Queue
	}
  }
#if 0
  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */
#endif
  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */

  /* USER CODE END CAN1_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles CAN1 SCE interrupt.
  */
void CAN1_SCE_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_SCE_IRQn 0 */

  /* USER CODE END CAN1_SCE_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_SCE_IRQn 1 */

  /* USER CODE END CAN1_SCE_IRQn 1 */
}

/**
  * @brief This function handles Ethernet global interrupt.
  */
void ETH_IRQHandler(void)
{
  /* USER CODE BEGIN ETH_IRQn 0 */

  /* USER CODE END ETH_IRQn 0 */
  HAL_ETH_IRQHandler(&heth);
  /* USER CODE BEGIN ETH_IRQn 1 */

  /* USER CODE END ETH_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
 * inline double getOutput(int apps_i)
 * return the scaled  precentage of the apps_i
 * APPS movement.
 */
inline void getOutput(int apps_i){
	appsOutput[apps_i] = ( (apps[apps_i] - min_val[apps_i]) / (max_val[apps_i] - min_val[apps_i]) ) * scale;
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
