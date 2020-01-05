/*
 * can.h
 *
 *  Created on: May 6, 2019
 *      Author: Eli
 */

#ifndef CAN_H_
#define CAN_H_

#include "stm32f7xx_hal.h"


typedef struct _can_variables{

uint8_t CAN_TX_buffer[8];
CAN_TxHeaderTypeDef  CAN_TX_pHeader;

}can_variables;

#define CAN_BUF_LEN		32


// Public variables
extern can_variables can;
extern can_variables canq[CAN_BUF_LEN];
extern can_variables *can_push_ptr;

// Public function
int can_transmit_it( void );
void can_push( void );
void can_init(void);


#endif /* CAN_H_ */
