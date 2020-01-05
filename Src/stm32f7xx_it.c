/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
#include "stm32f7xx_it.h"

/* USER CODE BEGIN 0 */
#define MINIMUM_VAL 709.0 //0x000 //480.0
#define MAXIMUM_VAL  2300.0//2481.0 //0xFFF //4095.0
#define OUTPUT_SCALE 100 //0x64    // the output value will be between 0 to OUTPUT_SCALE


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
uint32_t 		CAN_1_Rx_ide;
uint32_t 		CAN_1_Rx_rtr;
uint32_t 		CAN_1_Rx_dlc;
uint32_t 		CAN_1_Rx_fmi;


uint32_t val;
uint32_t output;
double max_val = MAXIMUM_VAL;
double min_val = MINIMUM_VAL;
double scale = OUTPUT_SCALE;

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


/* External variables --------------------------------------------------------*/
extern ETH_HandleTypeDef heth;
extern CAN_HandleTypeDef hcan1;

int msec_5 = 0;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ETH_HandleTypeDef heth;
extern CAN_HandleTypeDef hcan1;

/******************************************************************************/
/*            Cortex-M7 Processor Interruption and Exception Handlers         */ 
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
  HAL_SYSTICK_IRQHandler();
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

	  /* Receive FIFO 0 message pending interrupt management *********************/
	  if ((interrupts & CAN_IT_RX_FIFO0_MSG_PENDING) != 0U)
	  {
	    /* Check if message is still pending */
	    if ((CAN1->RF0R & CAN_RF0R_FMP0) != 0U)
	    {
	      CAN_1_Rx_ide = (uint8_t)0x04U & CAN1->sFIFOMailBox[0].RIR;
	      if ( CAN_1_Rx_ide == CAN_ID_STD )
	      {
	        CAN_1_Rx_eid = 0x00;
	        CAN_1_Rx_sid = 0x000007FFU & (CAN1->sFIFOMailBox[0].RIR >> 21U);
	      }
	      else
	      {
	        CAN_1_Rx_sid = 0x00;
	        CAN_1_Rx_eid = 0x1FFFFFFFU & (CAN1->sFIFOMailBox[0].RIR >> 3U);
	      }
	      switch(CAN_1_Rx_sid) {

	         case 0x401  :

	        	 	 //TODO elik

	            break;
	         case 0x421  :


				CAN_1_Rx_rtr = (uint8_t)0x02U & CAN1->sFIFOMailBox[0].RIR;
				CAN_1_Rx_dlc = (uint8_t)0x0FU & CAN1->sFIFOMailBox[0].RDTR;
				CAN_1_Rx_fmi = (uint8_t)0xFFU & (CAN1->sFIFOMailBox[0].RDTR >> 8U);

				CAN_1_RecData[0] = CAN1->sFIFOMailBox[0].RDLR;
				CAN_1_RecData[1] = CAN1->sFIFOMailBox[0].RDLR >> (8U);
				CAN_1_RecData[2] = CAN1->sFIFOMailBox[0].RDLR >> (16U);
				CAN_1_RecData[3] = CAN1->sFIFOMailBox[0].RDLR >> (24U);
				CAN_1_RecData[4] = CAN1->sFIFOMailBox[0].RDHR;
				CAN_1_RecData[5] = CAN1->sFIFOMailBox[0].RDHR >> (8U);
				CAN_1_RecData[6] = CAN1->sFIFOMailBox[0].RDHR >> (16U);
				CAN_1_RecData[7] = CAN1->sFIFOMailBox[0].RDHR >> (24U);

				if( CAN_1_RecData[7] == 0xFF){ //cheak_if_pedal_valid

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
				else{
					Keep_420[1] = 0x00;
					val = ( (0xFF00 & ( CAN_1_RecData[1] << 8 )) | (0x00FF & (CAN_1_RecData[0] ) ) ) ;
					output=( (val-min_val) / (max_val-min_val) ) * scale;

					if(output > 100) output = 100;
					//output = output && 0x00FF;
					brak_flag = 0;

					if( CAN_1_RecData[6] == 0x01 ){  // break
						output = 0;
						brak_flag = 1;
					}
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

	         case 0x81  :
	     		//HAL_GPIO_WritePin( GPIOB , GPIO_PIN_2|LD3_Pin , GPIO_PIN_SET);

	               CAN_1_Rx_rtr = (uint8_t)0x02U & CAN1->sFIFOMailBox[0].RIR;
	               CAN_1_Rx_dlc = (uint8_t)0x0FU & CAN1->sFIFOMailBox[0].RDTR;
	               CAN_1_Rx_fmi = (uint8_t)0xFFU & (CAN1->sFIFOMailBox[0].RDTR >> 8U);

	               CAN_1_RecData[0] = CAN1->sFIFOMailBox[0].RDLR;
	               CAN_1_RecData[1] = CAN1->sFIFOMailBox[0].RDLR >> (8U);
	               CAN_1_RecData[2] = CAN1->sFIFOMailBox[0].RDLR >> (16U);
	               CAN_1_RecData[3] = CAN1->sFIFOMailBox[0].RDLR >> (24U);
	               CAN_1_RecData[4] = CAN1->sFIFOMailBox[0].RDHR;
	               CAN_1_RecData[5] = CAN1->sFIFOMailBox[0].RDHR >> (8U);
	               CAN_1_RecData[6] = CAN1->sFIFOMailBox[0].RDHR >> (16U);
	               CAN_1_RecData[7] = CAN1->sFIFOMailBox[0].RDHR >> (24U);

	               if( (CAN_1_RecData[0] == 0x55) && (CAN_1_RecData[1] == 0x55) && (CAN_1_RecData[2] == 0xAA) && (CAN_1_RecData[3] == 0x55) && (CAN_1_RecData[4] == 0x5A) && (CAN_1_RecData[5] == 0xA5) && (CAN_1_RecData[6] == 0x5A) && (CAN_1_RecData[7] == 0xA5)  ){
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

	    SET_BIT(CAN1->RF0R, CAN_RF0R_RFOM0);
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

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
