/**
 ******************************************************************************
  * File Name          : LWIP.c
  * Description        : This file provides initialization code for LWIP
  *                      middleWare.
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
  
/* Includes ------------------------------------------------------------------*/
#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#if defined ( __CC_ARM )  /* MDK ARM Compiler */
#include "lwip/sio.h"
#endif /* MDK ARM Compiler */

/* USER CODE BEGIN 0 */
#include "lwip/udp.h"
#include  <math.h>

// Reference:
// https://www.st.com/resource/en/user_manual/dm00103685-developing-applications-on-stm32cube-with-lwip-tcpip-stack-stmicroelectronics.pdf

/* USER CODE END 0 */
/* Private function prototypes -----------------------------------------------*/
/* ETH Variables initialization ----------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN 1 */
static struct udp_pcb* elmo_r;                     // A udp pcb(check reference) struct to communicate with ELMO right inverter and motor
static struct udp_pcb* elmo_L;                     // A udp pcb struct to communicate with ELMO left inverter and motor
uint8_t IP_ADDRESS[4];                             // Unsigned 8bit array for the IP address
uint8_t NETMASK_ADDRESS[4];
uint8_t GATEWAY_ADDRESS[4];
struct pbuf *pp;                                   // Packet buffer struct used to send strings to the ELMOs

//==================== FLAG =========================
extern volatile unsigned char motor_RIGHT ;
extern volatile unsigned char motor_LEFT ;
extern volatile int car_volt; // not sure - didn't see it at other parts of the code
extern int RPM_r ;
extern int RPM_L ;
extern double motor_temp_r;

// From last code
//extern volatile unsigned char UI2_R;
//extern volatile unsigned char UI2_L;

/* USER CODE END 1 */

/* Variables Initialization */
struct netif gnetif;
ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;
uint8_t IP_ADDRESS[4];
uint8_t NETMASK_ADDRESS[4];
uint8_t GATEWAY_ADDRESS[4];

/* USER CODE BEGIN 2 */


// Creates a substring at the length of 'l' from a given string and the 'p' place.
void substring(char s[], char sub[], int p, int l) {
   int c = 0;
   while (c < l) {
      sub[c] = s[p+c];
      c++;
   }
   sub[c] = '\0';
}

// This function receives data from the left ELMO and copies it to the ECU global variables
void udp_receive_callback_L(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port){
	static char buffer[20];                                         // This buffer holds the data
	//printf("\r reacive \n");
    if (p != NULL)                                            // If the a struct with data received => proceed
    {
    	if(p->len > 2)                                        // If the p length>2 check it(otherwise its not the data we need)
    	{
    		pbuf_copy_partial(p, buffer, p->tot_len, 0);      // Copies the relevant data from the received buffer
    	    if(buffer[0] == 'M' && buffer[1] == 'O' && buffer[2] == '\r')  // If the data received is an "MO" command(motor on/off)
    	    {
    	    	//buff[0] = *(p->payload+3);
    	    	motor_LEFT = buffer[3] - 48;          // Motor_LEFT received 0 or 1
    	    	printf("\r mototr left = %d \n",motor_LEFT);
    	    }
    	    // From last code
    	    //else if (buffer[0] == 'U' && buffer[1] == 'I' && buffer[2] == '[' && buffer[3] == '1' && buffer[4] == ']' && buffer[5] == '\r') // If the data received is an "UI[2]", Watchdog delay register
    	    //    	          	          	          	  {
    	    //    	          	    	  	  	  	  	  	  UI2_L =  buffer[6] - 48;        // Register value
    	    //    	          	          	    	  	  	  printf("\r UI2_R = %d \n",UI2_L);
    	    //    	          	          	          	  }
    	    // If the data received is the RPM for the left motor from the buffer string
    	    if(p->len > 5 && buffer[0] == 'F' && buffer[1] == 'V' && buffer[2] == '[' && buffer[3] == '1' && buffer[4] == ']')
    	    {
    	    	int size = 0;
    	    	char num_temp[10];
    	    	RPM_L = 0;
    	    	//buff[0] = *(p->payload+3);
    	    	for(int i=0;i<10;i++)                // This loop find the size of the data at the buffer string
    	    	{
    	    		if(buffer[6+i] == ';')
    	    			break;
    	    		size++;
    	    	}//for
    	    	substring(buffer, num_temp, 6, size); // This function copies the data from the buffer to "num_temp" char array
    	    	RPM_L = atoi(num_temp);               // The RPM for the left motors is the data from the "num_temp" array converted to int
    	    }//if
    	}
        /* free pbuf */
        pbuf_free(p);
    }
}



// This function receives data from the right ELMO and copies it to the ECU global variables
void udp_receive_callback_R(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port){
	static char buffer[20];                                         // This buffer holds the data
	char num_temp[10];        // Why it wasn't defined at the left func?
	//printf("\r reacive \n");
    if (p != NULL)										    // If the a struct with data received => proceed
    {
    	if(p->len > 2)									    // If the p length>2 check it(otherwise its not the data we need)
    	{
    		pbuf_copy_partial(p, buffer, p->tot_len, 0);      // Copies the relevant data from the received buffer
    	    if(buffer[0] == 'M' && buffer[1] == 'O' && buffer[2] == '\r') // If the data received is an "MO" command(motor on/off)
    	    {
    	        //buff[0] = *(p->payload+3);
    	    	motor_RIGHT =  buffer[3] - 48;        // Motor_RIGHT received 0 or 1
    	    	printf("\r motor right = %d \n",motor_LEFT);  //Mistake here!// //Mistake here!// //Mistake here!//
    	    }
    	    //else if (buffer[0] == 'U' && buffer[1] == 'I' && buffer[2] == '[' && buffer[3] == '1' && buffer[4] == ']' && buffer[5] == '\r') // If the data received is an "UI[2]", Watchdog delay register
    	    //    	          	  {
    	    //	  	  	  	  	  	  UI2_R =  buffer[6] - 48;        // Register value
    	    //    	    	  	  	  printf("\r UI2_R = %d \n",UI2_R);
    	    //    	          	  }
    	    else  // This else didn't appear at the last function
    	    // If the data received is the RPM for the right motor from the buffer string
    	    	if(p->len > 5 && buffer[0] == 'F' && buffer[1] == 'V' && buffer[2] == '[' && buffer[3] == '1' && buffer[4] == ']')
    	        {
    	    		int size = 0;
    	        	RPM_r = 0;
    	        	//buff[0] = *(p->payload+3);
    	        	for(int i=0;i<10;i++)
    	        	{
    	        		if(buffer[6+i] == ';')		// This loop find the size of the data at the buffer string
    	        			break;
    	        	    size++;
    	        	}//for
    	        	substring(buffer, num_temp, 6, size); // This function copies the data from the buffer to "num_temp" char array
    	        	RPM_r = atoi(num_temp);               // The RPM for the left motors is the data from the "num_temp" array converted to int
    	        }//if
    	        // Suppose to get motor temp, according to ELMO command reference AN[1] gets analog input 1(not sure what this means)
    	        // There is another command for temperature - which is TI[] - not sure why it's not used
    	       	if(buffer[0] == 'A' && buffer[1] == 'N' && buffer[2] == '[' && buffer[3] == '1' && buffer[4] == ']')
    	        {
    	        	int size = 0;
    	        	motor_temp_r = 0;
    	        	//buff[0] = *(p->payload+3);
    	        	for(int i=0;i<10;i++)
    	        	{
    	        		if(buffer[6+i] == ';')
    	        	    	break;
    	        	    size++;
    	        	}//for
    	        	substring(buffer, num_temp, 6, size);
    	        	motor_temp_r = atof(num_temp);
    	        }//if
    	}
    pbuf_free(p);
    }
}

// This function creates UDP(check reference) struct to communicate with ELMO right inverter and motor
// A socket is one end-point of a two-way communication link between two programs running on the network
err_t create_udp_socket(){
	err_t err = ERR_OK;                              // Error flag
	ip4_addr_t destIPAddr;                           // A pointer to hold the IP address
	elmo_r = udp_new();                              // Create a UDP PCB
	if (elmo_r == NULL){                             // If the UDP_PCB is still NULL after it was created
	    return ERR_MEM;                              // Return Error
	}
	//IP4_ADDR(&destIPAddr,192,168,1,54);
	IP4_ADDR(&destIPAddr,192,168,1,49);              // Set an IP address given by the four byte-parts for the right ELMO
	// upcb->local_port = 5001;
	//upcb->local_port = 4004; // Set our local port to 4004
	// Should bind to the local ip and port
	err = udp_bind(elmo_r,IP4_ADDR_ANY,5002);        // Bind an UDP PCB with an IP address and port, returns ERR_OK if succeeded
	if (err != ERR_OK){                              // If an error occurred => return error
		return err;
	}
	// Connect to the other port
	//err = udp_connect(elmo_r,&destIPAddr,5001);
	err = udp_connect(elmo_r,&destIPAddr,5001);      // This will associate the UDP PCB with the remote address
	if (err != ERR_OK){                              // If an error occurred => return error
		return err;
	}
	//pp = pbuf_alloc(PBUF_TRANSPORT,5, PBUF_POOL);
	udp_recv(elmo_r,udp_receive_callback_R,NULL);    // Set the receive function - this callback will be called when receiving a datagram for the pcb
	return err;                                      // When finished => return error statues(probably an OK one)
}

// This function creates UDP(check reference) struct to communicate with ELMO right inverter and motor
// A socket is one end-point of a two-way communication link between two programs running on the network
err_t create_udp_socket2(){
    err_t err = ERR_OK;								 // Error flag
    ip4_addr_t destIPAddr;							 // A pointer to hold the IP address
    elmo_L = udp_new();								 // Create a UDP PCB
    elmo_L->next = NULL;	// Wasn't defined in the last func

    if (elmo_L == NULL){                             // If the UDP_PCB is still NULL after it was created
        return ERR_MEM;								 // Return Error
    }
    //IP4_ADDR(&destIPAddr,192,168,1,60);
    IP4_ADDR(&destIPAddr,192,168,1,48);				 // Set an IP address given by the four byte-parts for the left ELMO
    //upcb->local_port = 5001;
    //upcb->local_port = 4004; // Set our local port to 4004
    // Should bind to the local ip and port
    err = udp_bind(elmo_L,IP4_ADDR_ANY,5004);        // Bind an UDP PCB with an IP address and port, returns ERR_OK if succeeded
    if (err != ERR_OK){								 // If an error occurred => return error
        return err;
    }
    // Connect to the other port
    err = udp_connect(elmo_L,&destIPAddr,5001);      // This will associate the UDP PCB with the remote address
    if (err != ERR_OK){								 // If an error occurred => return error
        return err;
    }
    // Set the receive function
    udp_recv(elmo_L,udp_receive_callback_L,NULL);    // Set the receive function - this callback will be called when receiving a datagram for the pcb
    return err;                                      // When finished => return error statues(probably an OK one)
}

// This function send torque rated 0-100 to left motor using TC command, see ELMO command reference
err_t send_msg_to_dest2(char in ){
    struct pbuf *p;                                  // Defines pbuf
	uint8_t data[6]={0};                             // Defines an unsigned 8bit data array with zeros
	//printf("tc %d \n",in);
    //char ans[2];
   // int_to_char(ans,in);
      // Defines the data array to hold "TC=**\r"
	  data[0] = 'T';
	  data[1] = 'C';
	  data[2] = '=';
	  data[3] = in/10 + 48;
	  data[4] = in%10 + 48;
	  data[5] = '\r';
    /* allocate pbuf from pool*/
    p = pbuf_alloc(PBUF_TRANSPORT,6, PBUF_POOL);	 // Some kind of memory allocation for the pbuf
    pbuf_dechain(p);
    if (p != NULL)									 // If everything is fine - send data using the pbuf
        {
            pbuf_take(p, (char*)data, 6);            // Copy data to pbuf from the data array
            udp_send(elmo_L, p);                     // Send udp data to the ELMO
            pbuf_free(p);     						 // Free pbuf
            return ERR_OK;							 // End func if everything was fine
        }
    return ERR_OK; // not sure, I think its suppose to return error and print it to user
}
//want to replace the original colling

err_t send_msg_to_dest2_temp(char in ){	// !!! not sure how this function is different from the last one !!!
    struct pbuf *p;
	uint8_t data[6]={0};

	//printf("tc %d \n",in);
    //char ans[2];
   // int_to_char(ans,in);

	  data[0] = 'T';
	  data[1] = 'C';
	  data[2] = '=';
	  data[3] = in/10 + 48;
	  data[4] = in%10 + 48;
	  data[5] = '\r';

    /* allocate pbuf from pool*/
    p = pbuf_alloc(PBUF_TRANSPORT,6, PBUF_POOL);
    pbuf_dechain(p);

    if (p != NULL)
    {
        /* copy data to pbuf */
        pbuf_take(p, (char*)data, 6);

        /* send udp data */
        udp_send(elmo_r, p);

        /* free pbuf */
        pbuf_free(p);
        return ERR_OK;
    }
    return ERR_OK;
}

// This function send torque rated 0-100 to the right motor using TC command, see ELMO command reference
err_t send_msg_to_dest(char in ){
	struct pbuf *p;								 	 // Defines pbuf
	uint8_t data[7]={0};			// Why 7 and not 6? why the extra '-'?
	//char ans[2];
	//int_to_char(ans,in);
	// Defines the data array to hold "TC=**\r"
	data[0] = 'T';
	data[1] = 'C';
	data[2] = '=';
	data[3] = '-';                // Why?? the last one didn't need it
	data[4] = in/10 + 48;
	data[5] = in%10 + 48;
	data[6] = '\r';

	/* allocate pbuf from pool*/
	p = pbuf_alloc(PBUF_TRANSPORT,7, PBUF_POOL);	 // Some kind of memory allocation for the pbuf
	pbuf_dechain(p);
	if (p != NULL)									 // If everything is fine - send data using the pbuf
		{
		pbuf_take(p, (char*)data, 7);				 // Copy data to pbuf from the data array
		udp_send(elmo_r, p);						 // Send udp data to the ELMO
		pbuf_free(p);								 // Free pbuf
		return ERR_OK;							     // End func if everything was fine
        }
    return ERR_MEM;                                  // Else, there was an error with the pbuf
    printf("error allocate the pbuf\n");
}

// This function starts the right motor using MO command, see ELMO command reference
err_t Start_Motor_1( void ){
        struct pbuf *p;								 // Defines pbuf
    	uint8_t data[5]={0};						 // Defines an unsigned 8bit data array with zeros
    	// Defines the data array to hold "MC=1\r"
    	  data[0] = 'M';
    	  data[1] = 'O';
    	  data[2] = '=';
    	  data[3] = '1';
    	  data[4] = '\r';
        p = pbuf_alloc(PBUF_TRANSPORT,5, PBUF_POOL); // Some kind of memory allocation for the pbuf
        pbuf_dechain(p);
        if (p != NULL)								 // If everything is fine - send data using the pbuf
        {
            pbuf_take(p, (char*)data, 5);			 // Copy data to pbuf from the data array
            udp_send(elmo_r, p);					 // Send udp data to the ELMO
            pbuf_free(p);							 // Free pbuf
            return ERR_OK;							 // End func if everything was fine
        }
    return ERR_MEM;									 // Else, there was an error with the pbuf
    printf("error allocate the pbuf\n");
}

// This function starts the right motor using MO command, see ELMO command reference
err_t Start_Motor_2( void ){
        struct pbuf *p;								 // Defines a packet buffer struct - pbuf(look up at "open declaration")
    	uint8_t data[5]={0};						 // Defines an unsigned 8bit data array with zeros
    	// Defines the data array to hold "MC=1\r"
    	  data[0] = 'M';
    	  data[1] = 'O';
    	  data[2] = '=';
    	  data[3] = '1';
    	  data[4] = '\r';
        p = pbuf_alloc(PBUF_TRANSPORT,5, PBUF_POOL); // Some kind of memory allocation for the pbuf
        pbuf_dechain(p);
        if (p != NULL)								 // If everything is fine - send data using the pbuf
        {
            pbuf_take(p, (char*)data, 5);			 // Copy data to pbuf from the data array
            udp_send(elmo_L, p);					 // Send udp data to the ELMO
            pbuf_free(p);							 // Free pbuf
            return ERR_OK;							 // End func if everything was fine
        }
        return ERR_OK; // I think this isn't suppoed to be here
    return ERR_MEM;									 // Else, return an error
    printf("error allocate the pbuf\n");
}


/**
 * send "MO\r" to motor left
 *
 * @return Err_OK if send successfully
 * else return Err_MEM due to failure of allocating memory to send the message
 */
err_t ASK_Motor_1( void ){
        struct pbuf *p;                 		     // Defines a packet buffer struct - pbuf(look up at "open declaration")
    	uint8_t data[3]={0};              			 // Defines an unsigned 8bit data array with zeros
          // This data definition is sent to the ELMO for "MO" status(0 or 1) request - check ELMO command reference
    	  data[0] = 'M';
    	  data[1] = 'O';
    	  data[2] = '\r';
        p = pbuf_alloc(PBUF_TRANSPORT,3, PBUF_POOL); // Some kind of memory allocation for the pbuf
        pbuf_dechain(p);
        if (p != NULL)                               // If everything is fine - send data using the pbuf
        {
            pbuf_take(p, (char*)data, 3);            // Copy data to pbuf from the data array
            udp_send(elmo_r, p);                     // Send udp data to the ELMO
            pbuf_free(p);                            // Free pbuf
            return ERR_OK;                           // End func if everything was fine
        }
    return ERR_MEM;                                  // Else, return an error
    printf("error allocate the pbuf\n");
}

/**
 * send "MO\r" to motor left
 *
 * @return Err_OK if send successfully
 * else return Err_MEM due to failure of allocating memory to send the message
 */
err_t ASK_Motor_2( void ){
        struct pbuf *p;                   			 // Defines a packet buffer struct - pbuf(look up at "open declaration")
    	uint8_t data[3]={0};             		     // Defines an unsigned 8bit data array with zeros
    	  // This data definition is sent to the ELMO for "MO" status(0 or 1) request - check ELMO command reference
    	  data[0] = 'M';
    	  data[1] = 'O';
    	  data[2] = '\r';
        p = pbuf_alloc(PBUF_TRANSPORT,3, PBUF_POOL); // Some kind of memory allocation for the pbuf
        pbuf_dechain(p);
        if (p != NULL)                               // If everything is fine - send data using the pbuf
        {
            pbuf_take(p, (char*)data, 3);            // Copy data to pbuf from the data array
            udp_send(elmo_L, p);					 // Send udp data to the ELMO
            pbuf_free(p);                            // Free pbuf
            return ERR_OK;                           // End func if everything was fine
        }
    return ERR_MEM;                                  // Else, return an error
    printf("error allocate the pbuf\n");
}

// This function sends a request for the current RPM of motor right
err_t ASK_Motor_RPM_r( void ){
        struct pbuf *p;								 // Defines a packet buffer struct - pbuf(look up at "open declaration")
    	uint8_t data[6]={0};					     // Defines an unsigned 8bit data array with zeros
    	int len = 6;
    	// This data definition is sent to the ELMO for "FV[1]" request(socket 1 velocity request - check ELMO command reference)
    	  data[0] = 'F';
    	  data[1] = 'V';
    	  data[2] = '[';
    	  data[3] = '1';
    	  data[4] = ']';
    	  data[5] = '\r';
        p = pbuf_alloc(PBUF_TRANSPORT,len, PBUF_POOL); // Some kind of memory allocation for the pbuf
        // pbuf dechain is missing here !!
        if (p != NULL)								 // If everything is fine - send data using the pbuf
        {
            pbuf_take(p, (char*)data, len);			 // Copy data to pbuf from the data array
            udp_send(elmo_r, p);                     // Send udp data to the ELMO
            pbuf_free(p);						  	 // Free pbuf
            return ERR_OK;						 	 // End func if everything was fine
        }
    return ERR_MEM;							 		 // Else, return an error
    printf("error allocate the pbuf\n");
}

// This function sends a request for the current RPM of motor left
err_t ASK_Motor_RPM_L( void ){
        struct pbuf *p;						 		 // Defines a packet buffer struct - pbuf(look up at "open declaration")
    	uint8_t data[6]={0};						 // Defines an unsigned 8bit data array with zeros
    	int len = 6;
    	// This data definition is sent to the ELMO for "FV[1]" request(socket 1 velocity request - check ELMO command reference)
    	  data[0] = 'F';
    	  data[1] = 'V';
    	  data[2] = '[';
    	  data[3] = '1';
    	  data[4] = ']';
    	  data[5] = '\r';
        p = pbuf_alloc(PBUF_TRANSPORT,len, PBUF_POOL); // Some kind of memory allocation for the pbuf
        // dechain missing here?
        if (p != NULL)								 // If everything is fine - send data using the pbuf
        {
            pbuf_take(p, (char*)data, len);			 // Copy data to pbuf from the data array
            udp_send(elmo_L, p);					 // Send udp data to the ELMO
            pbuf_free(p);							 // Free pbuf
            return ERR_OK;							 // End func if everything was fine
        }
    return ERR_MEM;									 // Else, return an error
    printf("error allocate the pbuf\n");
}

// not sure about this function or MI[6]=7\r message
err_t Watchdog_mes_r( void ){
        struct pbuf *p;								 // Defines a packet buffer struct - pbuf(look up at "open declaration")
    	uint8_t data[8]={0};						 // Defines an unsigned 8bit data array with zeros
    	int len = 8;
    	// This data definition is sent to the ELMO for "MI[6]=7" - not sure what this means
    	  data[0] = 'M';
    	  data[1] = 'I';
    	  data[2] = '[';
    	  data[3] = '6';
    	  data[4] = ']';
    	  data[5] = '=';
    	  data[6] = '7';
    	  data[7] = '\r';
        p = pbuf_alloc(PBUF_TRANSPORT,len, PBUF_POOL); // Some kind of memory allocation for the pbuf
        if (p != NULL)
        {
            pbuf_take(p, (char*)data, len); 		 // Copy data to pbuf from the data array
            udp_send(elmo_r, p);				 	 // Send udp data to the ELMO
            pbuf_free(p);						 	 // Free pbuf
            return ERR_OK;						 	 // End func if everything was fine
        }
    return ERR_MEM;								 	 // Else, return an error
    printf("error allocate the pbuf\n");
}

// not sure about this function or MI[6]=7\r message
err_t Watchdog_mes_L( void ){
        struct pbuf *p;								 // Defines a packet buffer struct - pbuf(look up at "open declaration")
    	uint8_t data[8]={0};						 // Defines an unsigned 8bit data array with zeros
    	int len = 8;
    	// This data definition is sent to the ELMO for "MI[6]=7" - not sure what this means
    	  data[0] = 'M';
    	  data[1] = 'I';
    	  data[2] = '[';
    	  data[3] = '6';
    	  data[4] = ']';
    	  data[5] = '=';
    	  data[6] = '7';
    	  data[7] = '\r';
        p = pbuf_alloc(PBUF_TRANSPORT,len, PBUF_POOL); // Some kind of memory allocation for the pbuf
        // dechain missing here?
        if (p != NULL)								 // If everything is fine - send data using the pbuf
        {
            pbuf_take(p, (char*)data, len);			 // Copy data to pbuf from the data array
            udp_send(elmo_L, p);					 // Send udp data to the ELMO
            pbuf_free(p);							 // Free pbuf
            return ERR_OK;							 // End func if everything was fine
        }
    return ERR_MEM;									 // Else, return an error
    printf("error allocate the pbuf\n");
}

// This function sends a request for the current temperature of the right motor
err_t ASK_Motor_temp_R( void ){
        struct pbuf *p;								 // Defines a packet buffer struct - pbuf(look up at "open declaration")
    	uint8_t data[6]={0};						 // Defines an unsigned 8bit data array with zeros
    	int len = 6;
    	// Suppose to get motor temp, according to ELMO command reference AN[1] gets analog input 1(not sure what this means)
    	// There is another command for temperature - which is TI[] - not sure why it's not used
    	  data[0] = 'A';
    	  data[1] = 'N';
    	  data[2] = '[';
    	  data[3] = '1';
    	  data[4] = ']';
    	  data[5] = '\r';
        p = pbuf_alloc(PBUF_TRANSPORT,len, PBUF_POOL); // Some kind of memory allocation for the pbuf
        // dechain missing here?
        if (p != NULL)								 // If everything is fine - send data using the pbuf
        {
            pbuf_take(p, (char*)data, len);          // Copy data to pbuf from the data array
            udp_send(elmo_r, p);					 // Send udp data to the ELMO
            pbuf_free(p);							 // Free pbuf
            return ERR_OK;							 // End func if everything was fine
        }
    return ERR_MEM;									 // Else, return an error
    printf("error allocate the pbuf\n");
}

// This function sends a request for the current actual voltage supplied to the left ELMO
err_t ASK_Motor_volt( void ){
        struct pbuf *p;								 // Defines a packet buffer struct - pbuf(look up at "open declaration")
        int len = 6;								 // Defines an unsigned 8bit data array with zeros
    	uint8_t data[6]={0};
    	// This data definition is sent to the ELMO for "AN[6]" request(actual voltage supplied - check ELMO command reference)
    	  data[0] = 'A';
    	  data[1] = 'N';
    	  data[1] = '[';
    	  data[1] = '6';
    	  data[1] = ']';
    	  data[2] = '\r';
        p = pbuf_alloc(PBUF_TRANSPORT,6, PBUF_POOL); // Some kind of memory allocation for the pbuf
        pbuf_dechain(p);
        if (p != NULL)								 // If everything is fine - send data using the pbuf
        {
            pbuf_take(p, (char*)data, len);			 // Copy data to pbuf from the data array
            udp_send(elmo_L, p);					 // Send udp data to the ELMO
            pbuf_free(p);							 // Free pbuf
            return ERR_OK;							 // End func if everything was fine
        }
    return ERR_MEM;									 // Else, return an error
    printf("error allocate the pbuf\n");
}


//// This function request the value of UI[2] of the Elmo to check is there was a delay in the messages
//err_t AskDelayWatchDogR( void ){
//        struct pbuf *p;                 		     // Defines a packet buffer struct - pbuf(look up at "open declaration")
//    	uint8_t data[6]={0};              			 // Defines an unsigned 8bit data array with zeros
//          // This data definition is sent to the ELMO for "UI[2]" register value - check ELMO command reference
//    	  data[0] = 'U';
//    	  data[1] = 'I';
//    	  data[2] = '[';
//    	  data[3] = '2';
//   	  data[4] = ']';
//    	  data[5] = '\r';
//        p = pbuf_alloc(PBUF_TRANSPORT,6, PBUF_POOL); // Some kind of memory allocation for the pbuf
//        pbuf_dechain(p);
//        if (p != NULL)                               // If everything is fine - send data using the pbuf
//       {
//            pbuf_take(p, (char*)data, 6);            // Copy data to pbuf from the data array
//            udp_send(elmo_r, p);                     // Send udp data to the ELMO
//            pbuf_free(p);                            // Free pbuf
//            return ERR_OK;                           // End func if everything was fine
//        }
//    return ERR_MEM;                                  // Else, return an error
//    printf("error allocate the pbuf\n");
//}
//
//// This function request the value of UI[2] of the Elmo to check is there was a delay in the messages
//err_t AskDelayWatchDogL( void ){
//        struct pbuf *p;                   			 // Defines a packet buffer struct - pbuf(look up at "open declaration")
//   	uint8_t data[6]={0};             		     // Defines an unsigned 8bit data array with zeros
//    	// This data definition is sent to the ELMO for "UI[2]" register value - check ELMO command reference
//    	     data[0] = 'U';
//    	     data[1] = 'I';
//    	     data[2] = '[';
//    	     data[3] = '2';
//    	     data[4] = ']';
//    	     data[5] = '\r';
//        p = pbuf_alloc(PBUF_TRANSPORT,6, PBUF_POOL); // Some kind of memory allocation for the pbuf
//        pbuf_dechain(p);
//        if (p != NULL)                               // If everything is fine - send data using the pbuf
//        {
//            pbuf_take(p, (char*)data, 6);            // Copy data to pbuf from the data array
//            udp_send(elmo_L, p);					 // Send udp data to the ELMO
//            pbuf_free(p);                            // Free pbuf
//            return ERR_OK;                           // End func if everything was fine
//        }
//    return ERR_MEM;                                  // Else, return an error
//    printf("error allocate the pbuf\n");
//}
//
//// This function resets the ui[1] register at Elmo right, which is a flag for loss of connection of the ECU and the Elmo, after 500ms the Elmo sends torque 0 tu the motor
//err_t ResetElmoRFlag( void ){
//	 struct pbuf *p;                   			 // Defines a packet buffer struct - pbuf(look up at "open declaration")
//	 uint8_t data[8]={0};             		     // Defines an unsigned 8bit data array with zeros
//	 // This data definition is sent to the ELMO for reseting the "UI[1]" register value - check ELMO command reference
//	 	 data[0] = 'U';
//	 	 data[1] = 'I';
//	 	 data[2] = '[';
//	 	 data[3] = '1';
//	 	 data[4] = ']';
//	 	 data[5] = '=';
//	 	 data[6] = '0';
//	 	 data[7] = '\r';
//	 p = pbuf_alloc(PBUF_TRANSPORT,8, PBUF_POOL); // Some kind of memory allocation for the pbuf
//	 pbuf_dechain(p);
//	 if (p != NULL)                              // If everything is fine - send data using the pbuf
//	 {
//	      pbuf_take(p, (char*)data, 8);          // Copy data to pbuf from the data array
//	      udp_send(elmo_r, p);					 // Send udp data to the ELMO
//	      pbuf_free(p);                          // Free pbuf
//	      return ERR_OK;                         // End func if everything was fine
//	 }
//	 return ERR_MEM;                             // Else, return an error
//	 printf("error allocate the pbuf\n");
//}
//
//// This function resets the ui[1] register at Elmo right, which is a flag for loss of connection of the ECU and the Elmo, after 500ms the Elmo sends torque 0 tu the motor
//err_t ResetElmoLFlag( void ){
//	struct pbuf *p;                   			 // Defines a packet buffer struct - pbuf(look up at "open declaration")
//	uint8_t data[8]={0};             		     // Defines an unsigned 8bit data array with zeros
//	 // This data definition is sent to the ELMO for reseting the "UI[1]" register value - check ELMO command reference
//		data[0] = 'U';
//		data[1] = 'I';
//		data[2] = '[';
//		data[3] = '1';
//		data[4] = ']';
//		data[5] = '=';
//		data[6] = '0';
//		data[7] = '\r';
//	p = pbuf_alloc(PBUF_TRANSPORT,8, PBUF_POOL); // Some kind of memory allocation for the pbuf
//	pbuf_dechain(p);
//	if (p != NULL)                               // If everything is fine - send data using the pbuf
//	{
//	      pbuf_take(p, (char*)data, 8);          // Copy data to pbuf from the data array
//		  udp_send(elmo_r, p);					 // Send udp data to the ELMO
//		  pbuf_free(p);                          // Free pbuf
//		  return ERR_OK;                         // End func if everything was fine
//	}
//	return ERR_MEM;                              // Else, return an error
//	printf("error allocate the pbuf\n");
//}




/* USER CODE END 2 */

/**
  * LwIP initialization function
  */
void MX_LWIP_Init(void)
{
  /* IP addresses initialization */
  IP_ADDRESS[0] = 192;
  IP_ADDRESS[1] = 168;
  IP_ADDRESS[2] = 1;
  IP_ADDRESS[3] = 51;
  NETMASK_ADDRESS[0] = 255;
  NETMASK_ADDRESS[1] = 255;
  NETMASK_ADDRESS[2] = 255;
  NETMASK_ADDRESS[3] = 0;
  GATEWAY_ADDRESS[0] = 0;
  GATEWAY_ADDRESS[1] = 0;
  GATEWAY_ADDRESS[2] = 0;
  GATEWAY_ADDRESS[3] = 0;
  
  /* Initilialize the LwIP stack without RTOS */
  lwip_init();

  /* IP addresses initialization without DHCP (IPv4) */
  IP4_ADDR(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
  IP4_ADDR(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
  IP4_ADDR(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);

  /* add the network interface (IPv4/IPv6) without RTOS */
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);

  /* Registers the default network interface */
  netif_set_default(&gnetif);

  if (netif_is_link_up(&gnetif))
  {
    /* When the netif is fully configured this function must be called */
    netif_set_up(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif);
  }

/* USER CODE BEGIN 3 */

  // Setup the udp port
  create_udp_socket2();
  create_udp_socket();
  printf("\rcreate udp socket\n");





/* USER CODE END 3 */
}

#ifdef USE_OBSOLETE_USER_CODE_SECTION_4
/* Kept to help code migration. (See new 4_1, 4_2... sections) */
/* Avoid to use this user section which will become obsolete. */
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
#endif

/**
 * ----------------------------------------------------------------------
 * Function given to help user to continue LwIP Initialization
 * Up to user to complete or change this function ...
 * Up to user to call this function in main.c in while (1) of main(void) 
 *-----------------------------------------------------------------------
 * Read a received packet from the Ethernet buffers 
 * Send it to the lwIP stack for handling
 * Handle timeouts if LWIP_TIMERS is set and without RTOS
 * Handle the llink status if LWIP_NETIF_LINK_CALLBACK is set and without RTOS 
 */
void MX_LWIP_Process(void)
{
/* USER CODE BEGIN 4_1 */
 // ethernetif_set_link(&gnetif);
/* USER CODE END 4_1 */
  ethernetif_input(&gnetif);
  
/* USER CODE BEGIN 4_2 */
  //send_msg_to_dest();
 // printf("\rsend message\n");
/* USER CODE END 4_2 */  
  /* Handle timeouts */
  sys_check_timeouts();

/* USER CODE BEGIN 4_3 */

/* USER CODE END 4_3 */
}

#if defined ( __CC_ARM )  /* MDK ARM Compiler */
/**
 * Opens a serial device for communication.
 *
 * @param devnum device number
 * @return handle to serial device if successful, NULL otherwise
 */
sio_fd_t sio_open(u8_t devnum)
{
  sio_fd_t sd;

/* USER CODE BEGIN 7 */
  sd = 0; // dummy code
/* USER CODE END 7 */
	
  return sd;
}

/**
 * Sends a single character to the serial device.
 *
 * @param c character to send
 * @param fd serial device handle
 *
 * @note This function will block until the character can be sent.
 */
void sio_send(u8_t c, sio_fd_t fd)
{
/* USER CODE BEGIN 8 */
/* USER CODE END 8 */
}

/**
 * Reads from the serial device.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received - may be 0 if aborted by sio_read_abort
 *
 * @note This function will block until data can be received. The blocking
 * can be cancelled by calling sio_read_abort().
 */
u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 9 */
  recved_bytes = 0; // dummy code
/* USER CODE END 9 */	
  return recved_bytes;
}

/**
 * Tries to read from the serial device. Same as sio_read but returns
 * immediately if no data is available and never blocks.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received
 */
u32_t sio_tryread(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 10 */
  recved_bytes = 0; // dummy code
/* USER CODE END 10 */	
  return recved_bytes;
}
#endif /* MDK ARM Compiler */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
