/*
 * can.c
 *
 *  Created on: May 6, 2019
 *      Author: Eli
 */


//#include <can_data_struct.h>

#include "stm32f7xx_hal.h"
#include "can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_FilterTypeDef FilterConfig;

// Public variables
can_variables can;
can_variables canq[CAN_BUF_LEN];
can_variables *can_push_ptr;

// Private variables
can_variables *can_pop_ptr;



int can_transmit_it( void )
{
uint32_t pTxMailbox ;

if ( can_push_ptr != can_pop_ptr )
	{
		if( HAL_CAN_AddTxMessage(&hcan1, &can_pop_ptr->CAN_TX_pHeader,can_pop_ptr->CAN_TX_buffer, &pTxMailbox) != HAL_OK)
			_Error_Handler(__FILE__, __LINE__); // No data to transmit

		can_pop_ptr++;
		if (can_pop_ptr == ( canq + CAN_BUF_LEN )) can_pop_ptr = canq;
		return(1);
	}
	else
	{
		//no data to transmit
		return(-3);
	}
}


void can_push( void )
{
int flag = 0;

	if ( can_push_ptr != can_pop_ptr ) flag=1;

	__disable_irq();

	can_push_ptr++;
	if (can_push_ptr == ( canq + CAN_BUF_LEN )) can_push_ptr = canq;
	if(flag & !(HAL_CAN_IsTxMessagePending(&hcan1, (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2))))
		can_transmit_it();

	__enable_irq();

}


void can_init(void){

   __HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_TX_MAILBOX_EMPTY); //Enable CAN Tx interrupt
   __HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
   __HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_RX_FIFO1_MSG_PENDING);

   FilterConfig.FilterIdHigh = 0xFFFFU;
   FilterConfig.FilterIdLow  = 0xFFFFU;
   FilterConfig.FilterMaskIdHigh = 0x0000U;
   FilterConfig.FilterMaskIdLow = 0x0000U;
   FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
   //FilterConfig.FilterNumber = 0;
   FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
   FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
   FilterConfig.FilterActivation  = ENABLE;
   FilterConfig.FilterBank = 0;

   HAL_CAN_ConfigFilter(&hcan1, &FilterConfig);
   HAL_CAN_Start(&hcan1);

   can_push_ptr = canq;
   can_pop_ptr = can_push_ptr;

}
//CAN_TX_MAILBOX0





