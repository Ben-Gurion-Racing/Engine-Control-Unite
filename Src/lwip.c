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
/* USER CODE END 0 */
/* Private function prototypes -----------------------------------------------*/
/* ETH Variables initialization ----------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN 1 */
static struct udp_pcb* elmo_r;
static struct udp_pcb* elmo_L;

uint8_t IP_ADDRESS[4];
uint8_t NETMASK_ADDRESS[4];
uint8_t GATEWAY_ADDRESS[4];

struct pbuf *pp;

//==================== FLAG =========================

extern volatile unsigned char motor_RIGHT ;
extern volatile unsigned char motor_LEFT ;
extern volatile int car_volt;
extern int RPM_r ;
extern int RPM_L ;
extern double motor_temp_r;

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


void substring(char s[], char sub[], int p, int l) {
   int c = 0;

   while (c < l) {
      sub[c] = s[p+c];
      c++;
   }
   sub[c] = '\0';
}

void udp_receive_callback_L(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port){

static char buffer[20];

//printf("\r reacive \n");
      if (p != NULL)
      {
    	  if(p->len > 2)
    	  {
    	      pbuf_copy_partial(p, buffer, p->tot_len, 0);
    	      if(buffer[0] == 'M' && buffer[1] == 'O' && buffer[2] == '\r')
    	          	  {
    	          		  //buff[0] = *(p->payload+3);
    	    	  	  	  motor_LEFT = buffer[3] - 48;
    	    	  	  	  printf("\r mototr left = %d \n",motor_LEFT);
    	          	  }


    	      if(p->len > 5 && buffer[0] == 'F' && buffer[1] == 'V' && buffer[2] == '[' && buffer[3] == '1' && buffer[4] == ']')
					  {
						  int size = 0;
						  char num_temp[10];

						  RPM_L = 0;
						  //buff[0] = *(p->payload+3);
						  for(int i=0;i<10;i++){
							  if(buffer[6+i] == ';')
								  break;
							   size++;
						  }//for
						  substring(buffer, num_temp, 6, size);
						  RPM_L = atoi(num_temp);

					  }//if



    	  }




        /* copy data to pbuf */
        /* send udp data */
        /* free pbuf */
        pbuf_free(p);
      }
}



void udp_receive_callback_R(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port){

static char buffer[20];
char num_temp[10];
//printf("\r reacive \n");
      if (p != NULL)
      {
    	  if(p->len > 2)
    	  {
    	      pbuf_copy_partial(p, buffer, p->tot_len, 0);
    	      if(buffer[0] == 'M' && buffer[1] == 'O' && buffer[2] == '\r')
    	          	  {
    	          		  //buff[0] = *(p->payload+3);
    	    	  	  	  motor_RIGHT =  buffer[3] - 48;
    	    	  	  	  printf("\r mototr right = %d \n",motor_LEFT);

    	          	  }
    	      else
    	    	  if(p->len > 5 && buffer[0] == 'F' && buffer[1] == 'V' && buffer[2] == '[' && buffer[3] == '1' && buffer[4] == ']')
								  {
    	    		  	  	  	  	  int size = 0;
    	    		  	  	  	  	  RPM_r = 0;
									  //buff[0] = *(p->payload+3);
    	    		  	  	  	  	  for(int i=0;i<10;i++){
    	    		  	  	  	  		  if(buffer[6+i] == ';')
    	    		  	  	  	  			  break;
    	    		  	  	  	  		   size++;
    	    		  	  	  	  	  }//for
    	    		  	  	  	      substring(buffer, num_temp, 6, size);
    	    		  	  	  	      RPM_r = atoi(num_temp);

								  }//if




    	      	  	  if(buffer[0] == 'A' && buffer[1] == 'N' && buffer[2] == '[' && buffer[3] == '1' && buffer[4] == ']')
    	      								  {
    	          	    		  	  	  	  	  int size = 0;
    	          	    		  	  	  	  	  motor_temp_r = 0;
    	      									  //buff[0] = *(p->payload+3);
    	          	    		  	  	  	  	  for(int i=0;i<10;i++){
    	          	    		  	  	  	  		  if(buffer[6+i] == ';')
    	          	    		  	  	  	  			  break;
    	          	    		  	  	  	  		   size++;
    	          	    		  	  	  	  	  }//for
    	          	    		  	  	  	      substring(buffer, num_temp, 6, size);
    	          	    		  	  	  	      motor_temp_r = atof(num_temp);

    	      								  }//if



    	  }

        /* copy data to pbuf */
        /* send udp data */
        /* free pbuf */
        pbuf_free(p);
      }
}



err_t create_udp_socket(){
    err_t err = ERR_OK;
    ip4_addr_t destIPAddr;
    elmo_r = udp_new();

    if (elmo_r == NULL){
        return ERR_MEM;
    }
    //IP4_ADDR(&destIPAddr,192,168,1,54);
    IP4_ADDR(&destIPAddr,192,168,1,49);
   // upcb->local_port = 5001;
    //upcb->local_port = 4004; // Set our local port to 4004
    // Should bind to the local ip and port
    err = udp_bind(elmo_r,IP4_ADDR_ANY,5002);

    if (err != ERR_OK){
        return err;
    }
    // Connect to the other port
    //err = udp_connect(elmo_r,&destIPAddr,5001);
    err = udp_connect(elmo_r,&destIPAddr,5001);
    if (err != ERR_OK){
        return err;
    }

	//pp = pbuf_alloc(PBUF_TRANSPORT,5, PBUF_POOL);


    // Set the receive function
    udp_recv(elmo_r,udp_receive_callback_R,NULL);
    return err;
}


err_t create_udp_socket2(){
    err_t err = ERR_OK;
    ip4_addr_t destIPAddr;
    elmo_L = udp_new();
    elmo_L->next = NULL;

    if (elmo_L == NULL){
        return ERR_MEM;
    }
    //IP4_ADDR(&destIPAddr,192,168,1,60);
    IP4_ADDR(&destIPAddr,192,168,1,48);
   // upcb->local_port = 5001;
    //upcb->local_port = 4004; // Set our local port to 4004
    // Should bind to the local ip and port
    err = udp_bind(elmo_L,IP4_ADDR_ANY,5004);
    if (err != ERR_OK){
        return err;
    }
    // Connect to the other port
    err = udp_connect(elmo_L,&destIPAddr,5001);
    if (err != ERR_OK){
        return err;
    }


    // Set the receive function
    udp_recv(elmo_L,udp_receive_callback_L,NULL);
    return err;
}



err_t send_msg_to_dest2(char in ){
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
        udp_send(elmo_L, p);

        /* free pbuf */
        pbuf_free(p);
        return ERR_OK;
    }
    return ERR_OK;
}

//want to replace the original colling


err_t send_msg_to_dest2_temp(char in ){
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


err_t send_msg_to_dest(char in ){
        struct pbuf *p;
    	uint8_t data[7]={0};

        //char ans[2];
        //int_to_char(ans,in);

    	  data[0] = 'T';
    	  data[1] = 'C';
    	  data[2] = '=';
    	  data[3] = '-';
    	  data[4] = in/10 + 48;
    	  data[5] = in%10 + 48;
    	  data[6] = '\r';

        /* allocate pbuf from pool*/
        p = pbuf_alloc(PBUF_TRANSPORT,7, PBUF_POOL);
        pbuf_dechain(p);



        if (p != NULL)
        {
            /* copy data to pbuf */
            pbuf_take(p, (char*)data, 7);

            /* send udp data */
            udp_send(elmo_r, p);

            /* free pbuf */
            pbuf_free(p);
            return ERR_OK;
        }


    return ERR_MEM;
    printf("error allocate the pbuf\n");
}


err_t Start_Motor_1( void ){
        struct pbuf *p;
    	uint8_t data[5]={0};


    	  data[0] = 'M';
    	  data[1] = 'O';
    	  data[2] = '=';
    	  data[3] = '1';
    	  data[4] = '\r';

        /* allocate pbuf from pool*/
        p = pbuf_alloc(PBUF_TRANSPORT,5, PBUF_POOL);
        pbuf_dechain(p);



        if (p != NULL)
        {
            /* copy data to pbuf */
            pbuf_take(p, (char*)data, 5);

            /* send udp data */
            udp_send(elmo_r, p);

            /* free pbuf */
            pbuf_free(p);
            return ERR_OK;
        }


    return ERR_MEM;
    printf("error allocate the pbuf\n");
}

/**
 * send "MO\r" to motor left
 *
 * @return Err_OK if send successfully
 * else return Err_MEM due to failure of allocating memory to send the message
 */
err_t ASK_Motor_1( void ){
        struct pbuf *p;
    	uint8_t data[3]={0};

    	  data[0] = 'M';
    	  data[1] = 'O';
    	  data[2] = '\r';

        /* allocate pbuf from pool*/
        p = pbuf_alloc(PBUF_TRANSPORT,3, PBUF_POOL);
        pbuf_dechain(p);



        if (p != NULL)
        {
            /* copy data to pbuf */
            pbuf_take(p, (char*)data, 3);

            /* send udp data */
            udp_send(elmo_r, p);

            /* free pbuf */
            pbuf_free(p);
            return ERR_OK;
        }


    return ERR_MEM;
    printf("error allocate the pbuf\n");
}

/**
 * send "MO\r" to motor left
 *
 * @return Err_OK if send successfully
 * else return Err_MEM due to failure of allocating memory to send the message
 */
err_t ASK_Motor_2( void ){
        struct pbuf *p;
    	uint8_t data[3]={0};

    	  data[0] = 'M';
    	  data[1] = 'O';
    	  data[2] = '\r';

        /* allocate pbuf from pool*/
        p = pbuf_alloc(PBUF_TRANSPORT,3, PBUF_POOL);
        pbuf_dechain(p);



        if (p != NULL)
        {
            /* copy data to pbuf */
            pbuf_take(p, (char*)data, 3);

            /* send udp data */
            udp_send(elmo_L, p);

            /* free pbuf */
            pbuf_free(p);
            return ERR_OK;
        }


    return ERR_MEM;
    printf("error allocate the pbuf\n");
}

err_t ASK_Motor_RPM_r( void ){
        struct pbuf *p;
    	uint8_t data[6]={0};
    	int len = 6;

    	  data[0] = 'F';
    	  data[1] = 'V';
    	  data[2] = '[';
    	  data[3] = '1';
    	  data[4] = ']';
    	  data[5] = '\r';

        /* allocate pbuf from pool*/
        p = pbuf_alloc(PBUF_TRANSPORT,len, PBUF_POOL);

        if (p != NULL)
        {
            /* copy data to pbuf */
            pbuf_take(p, (char*)data, len);

            /* send udp data */
            udp_send(elmo_r, p);

            /* free pbuf */
            pbuf_free(p);
            return ERR_OK;
        }


    return ERR_MEM;
    printf("error allocate the pbuf\n");
}


err_t Watchdog_mes_r( void ){
        struct pbuf *p;
    	uint8_t data[8]={0};
    	int len = 8;

    	  data[0] = 'M';
    	  data[1] = 'I';
    	  data[2] = '[';
    	  data[3] = '6';
    	  data[4] = ']';
    	  data[5] = '=';
    	  data[6] = '7';
    	  data[7] = '\r';

        /* allocate pbuf from pool*/
        p = pbuf_alloc(PBUF_TRANSPORT,len, PBUF_POOL);

        if (p != NULL)
        {
            /* copy data to pbuf */
            pbuf_take(p, (char*)data, len);

            /* send udp data */
            udp_send(elmo_r, p);

            /* free pbuf */
            pbuf_free(p);
            return ERR_OK;
        }


    return ERR_MEM;
    printf("error allocate the pbuf\n");
}

err_t Watchdog_mes_L( void ){
        struct pbuf *p;
    	uint8_t data[8]={0};
    	int len = 8;

    	  data[0] = 'M';
    	  data[1] = 'I';
    	  data[2] = '[';
    	  data[3] = '6';
    	  data[4] = ']';
    	  data[5] = '=';
    	  data[6] = '7';
    	  data[7] = '\r';

        /* allocate pbuf from pool*/
        p = pbuf_alloc(PBUF_TRANSPORT,len, PBUF_POOL);

        if (p != NULL)
        {
            /* copy data to pbuf */
            pbuf_take(p, (char*)data, len);

            /* send udp data */
            udp_send(elmo_L, p);

            /* free pbuf */
            pbuf_free(p);
            return ERR_OK;
        }


    return ERR_MEM;
    printf("error allocate the pbuf\n");
}




err_t ASK_Motor_RPM_L( void ){
        struct pbuf *p;
    	uint8_t data[6]={0};
    	int len = 6;

    	  data[0] = 'F';
    	  data[1] = 'V';
    	  data[2] = '[';
    	  data[3] = '1';
    	  data[4] = ']';
    	  data[5] = '\r';

        /* allocate pbuf from pool*/
        p = pbuf_alloc(PBUF_TRANSPORT,len, PBUF_POOL);

        if (p != NULL)
        {
            /* copy data to pbuf */
            pbuf_take(p, (char*)data, len);

            /* send udp data */
            udp_send(elmo_L, p);

            /* free pbuf */
            pbuf_free(p);
            return ERR_OK;
        }


    return ERR_MEM;
    printf("error allocate the pbuf\n");
}


err_t ASK_Motor_temp_R( void ){
        struct pbuf *p;
    	uint8_t data[6]={0};
    	int len = 6;

    	  data[0] = 'A';
    	  data[1] = 'N';
    	  data[2] = '[';
    	  data[3] = '1';
    	  data[4] = ']';
    	  data[5] = '\r';

        /* allocate pbuf from pool*/
        p = pbuf_alloc(PBUF_TRANSPORT,len, PBUF_POOL);

        if (p != NULL)
        {
            /* copy data to pbuf */
            pbuf_take(p, (char*)data, len);

            /* send udp data */
            udp_send(elmo_r, p);

            /* free pbuf */
            pbuf_free(p);
            return ERR_OK;
        }


    return ERR_MEM;
    printf("error allocate the pbuf\n");
}




err_t ASK_Motor_volt( void ){
        struct pbuf *p;
        int len = 6;
    	uint8_t data[6]={0};

    	  data[0] = 'A';
    	  data[1] = 'N';
    	  data[1] = '[';
    	  data[1] = '6';
    	  data[1] = ']';
    	  data[2] = '\r';

        /* allocate pbuf from pool*/
        p = pbuf_alloc(PBUF_TRANSPORT,6, PBUF_POOL);
        pbuf_dechain(p);



        if (p != NULL)
        {
            /* copy data to pbuf */
            pbuf_take(p, (char*)data, len);

            /* send udp data */
            udp_send(elmo_L, p);

            /* free pbuf */
            pbuf_free(p);
            return ERR_OK;
        }


    return ERR_MEM;
    printf("error allocate the pbuf\n");
}



err_t Start_Motor_2( void ){
        struct pbuf *p;
    	uint8_t data[5]={0};


    	  data[0] = 'M';
    	  data[1] = 'O';
    	  data[2] = '=';
    	  data[3] = '1';
    	  data[4] = '\r';

        /* allocate pbuf from pool*/
        p = pbuf_alloc(PBUF_TRANSPORT,5, PBUF_POOL);
        pbuf_dechain(p);



        if (p != NULL)
        {
            /* copy data to pbuf */
            pbuf_take(p, (char*)data, 5);

            /* send udp data */
            udp_send(elmo_L, p);

            /* free pbuf */
            pbuf_free(p);
            return ERR_OK;
        }
        return ERR_OK;


    return ERR_MEM;
    printf("error allocate the pbuf\n");
}


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
}




/* USER CODE END 3 */


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
