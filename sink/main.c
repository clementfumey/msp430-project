#include <msp430f2274.h>

#if defined(__GNUC__) && defined(__MSP430__)
/* This is the MSPGCC compiler */
#include <msp430.h>
#include <iomacros.h>
#include <legacymsp430.h>
#elif defined(__IAR_SYSTEMS_ICC__)
/* This is the IAR compiler */
//#include <io430.h>
#endif

#include <stdio.h>

#include "isr_compat.h"
#include "leds.h"
#include "clock.h"
#include "timer.h"
#include "button.h"
#include "uart.h"
#include "adc10.h"
#include "spi.h"
#include "cc2500.h"
#include "lpm_compat.h"
#include "flash.h"
#include "watchdog.h"

#include "pt.h"

#define ID 0
#define COEFF_1_ADDR INFOD_START+1
#define COEFF_2_ADDR INFOD_START+3


#define MSG_BYTE_TYPE 0U //First Byte is type of message
#define MSG_BYTE_DEST_ROUTE 1U //Second Byte is dest id
#define MSG_BYTE_SRC_ROUTE 2U // Third is source id
#define MSG_BYTE_CONTENT 3U // Fourth and fifth is content

#define PKTLEN 5 //Packet lenght

// First Byte : Type of message

#define MSG_TYPE_TEMPERATURE 0x02

#define NUM_TIMERS 1
static uint16_t timer[NUM_TIMERS];
#define TIMER_SEND_TEMP timer[0]

int timer_reached(uint16_t timer, uint16_t count) {
    return (timer >= count);
}

#define DBG_PRINTF printf

/*
 * Radio
 */

static char radio_tx_buffer[PKTLEN];
static uint8_t radio_rx_buffer[PKTLEN];
static int radio_rx_flag;

static void radio_send_message()
{
    cc2500_utx(radio_tx_buffer, PKTLEN);
    cc2500_rx_enter();
}

void radio_cb(uint8_t* buffer, int size, int8_t rssi)
{
  led_green_switch();
  switch (size)
    {
    case 0:
      DBG_PRINTF("msg size 0\n");
      break;
    case -EEMPTY:
      DBG_PRINTF("msg empty\n");
      break;
    case -ERXFLOW:
      DBG_PRINTF("msg rx overflow\n");
      break;
    case -ERXBADCRC:
      DBG_PRINTF("msg rx bad CRC\n");
      break;
    case -ETXFLOW:
      DBG_PRINTF("msg tx overflow\n");
      break;
    default:
      if (size > 0)
	{
	  // memcpy(buffer_rx_msg, buffer, MSG_SIZE);
	  DBG_PRINTF("rssi %d\r\n", rssi);
	  
	    //memcpy(radio_rx_buffer, buffer, PKTLEN);
	    //FIXME what if radio_rx_flag == 1 already?
	    radio_rx_flag = 1;
	}
      else
	{
	  /* packet error, drop */
	  DBG_PRINTF("msg packet error size=%d\n",size);
	}
      break;
    }
  cc2500_idle();
  cc2500_rx_enter();
  led_green_switch();
}



/* Protothread contexts */

#define NUM_PT 1
static struct pt pt[NUM_PT];


static void dump_message(uint8_t *buffer)
{
    printf("message received\r\n");
    printf("\r\n  type: ");
    switch(buffer[MSG_BYTE_TYPE])
    {
        case MSG_TYPE_TEMPERATURE:
            printf("temperature");
            break;
    }
    printf("\r\n  source : %d\r\n", buffer[MSG_BYTE_SRC_ROUTE]);
    printf("  destination: %d\r\n", buffer[MSG_BYTE_DEST_ROUTE]);

//TODO SWITCH Instead
    if(buffer[MSG_BYTE_TYPE] == MSG_TYPE_TEMPERATURE)
    {
        unsigned int temperature;
        char *pt = (char *) &temperature;
        pt[0] = buffer[MSG_BYTE_CONTENT + 1];
        pt[1] = buffer[MSG_BYTE_CONTENT];
        printf("  temperature: %d,%d\r\n", temperature/10,temperature%10);
    }

}
/* ************************************************** */
/* ************************************************** */
/* ************************************************** */

static PT_THREAD(thread_process_msg(struct pt *pt))
{
    PT_BEGIN(pt);

    while(1)
    {
        PT_WAIT_UNTIL(pt, radio_rx_flag == 1);

        dump_message(radio_rx_buffer);
	
        if(radio_rx_buffer[MSG_BYTE_TYPE] == MSG_TYPE_TEMPERATURE ){
            //&& radio_rx_buffer[MSG_BYTE_DEST_ROUTE] == ID
        }else{
	
	}
        radio_rx_flag = 0;
    }

    PT_END(pt);
}

/* ************************************************** */
/* ************************************************** */
/* ************************************************** */

int main(void)
{
	watchdog_stop();

	set_mcu_speed_dco_mclk_16MHz_smclk_8MHz();
	leds_init();
	led_red_on();

	timerA_init();
	timerA_set_wakeup(1);
	timerA_start_milliseconds(1000);

	uart_init(UART_9600_SMCLK_8MHZ);
	printf("Smart GreenHouses: Sink\n\r");

	/* radio init */
	spi_init();
	cc2500_init();
	cc2500_rx_register_buffer(radio_rx_buffer, PKTLEN);
	cc2500_rx_register_cb(radio_cb);
	cc2500_rx_enter();
	radio_rx_flag = 0;

  __enable_interrupt();

  //led_green_on();
	/* protothreads init */
	int i;
	for(i = 0; i < NUM_PT; i++)
	{
		PT_INIT(&pt[i]);
	}

	while(1) {
		thread_process_msg(&pt[0]);
	}
}

/* ************************************************** */
/* ************************************************** */