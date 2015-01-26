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
#include <limits.h>

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

#define ID 1

#define MSG_BYTE_TYPE 0U //First Byte is type of message
#define MSG_BYTE_DEST_ROUTE 1U //Second Byte is dest id
#define MSG_BYTE_SRC_ROUTE 2U // Third is source id
#define MSG_BYTE_CONTENT 3U // Fourth and fifth is content

#define PKTLEN 10 //Packet lenght

// First Byte : Type of message
#define MSG_TYPE_RTS 0x01
#define MSG_TYPE_CTS 0x02
#define MSG_TYPE_TEMPERATURE 0x03

#define NUM_TIMERS 3
static uint16_t timer[NUM_TIMERS];
#define TIMER_SEND_TEMP timer[0]
#define TIMER_LED_RED_ON timer[1]
#define TIMER_LED_GREEN_ON timer[2]

void timer_tick_cb() {
    int i;
    for(i = 0; i < NUM_TIMERS; i++)
    {
        if(timer[i] != UINT_MAX) {
            timer[i]++;
        }
    }
}

int timer_reached(uint16_t timer, uint16_t count) {
    return (timer >= count);
}

#define DBG_PRINTF printf

/*
 * LEDs
 */

static int led_green_duration;
static int led_green_flag;

/* asynchronous */
static void led_green_blink(int duration)
{
    led_green_duration = duration;
    led_green_flag = 1;
}

static int led_red_duration;
static int led_red_flag;

/* asynchronous */
static void led_red_blink(int duration)
{
    led_red_duration = duration;
    led_red_flag = 1;
}


/*
 * Radio
 */
static int clear_to_send_flag;
static char radio_tx_buffer[PKTLEN];
static uint8_t radio_rx_buffer[PKTLEN];
static int radio_rx_flag;
static void printhex(char *buffer, unsigned int len)
{
    unsigned int i;
    for(i = 0; i < len; i++)
    {
        printf("%02X ", buffer[i]);
    }
}

static void radio_send_message()
{
    cc2500_utx(radio_tx_buffer, PKTLEN);
    printhex(radio_tx_buffer, PKTLEN);
    printf("\r\n");
    cc2500_rx_enter(); //If we put this one just after cc2500_utx it fail and we receveid BAD_CRC
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
	    printf("rssi %d\r\n", rssi);
	}
      else
	{
	  /* packet error, drop */
	  printf("error");
	  DBG_PRINTF("msg packet error size=%d\n",size);
	}
      break;
    }
  cc2500_idle();
  cc2500_rx_enter();
  led_green_switch();
}

/* to be called from within a protothread */
static void init_message()
{
    unsigned int i;
    for(i = 0; i < PKTLEN; i++)
    {
      /* msg[0] == id */
      radio_tx_buffer[i] = ID + i;
    }
    radio_tx_buffer[MSG_BYTE_SRC_ROUTE] = ID;
    /* useless bytes becaus cc2500 driver don't like small packets*/
    radio_tx_buffer[6] = 0x04;
    radio_tx_buffer[7] = 0x03;
    radio_tx_buffer[8] = 0x02;
    radio_tx_buffer[9] = 0x01;
}

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

/* to be called from within a protothread */
static void send_cts(uint8_t dest)
{
    led_green_blink(10);
    init_message();
    radio_tx_buffer[MSG_BYTE_TYPE] = MSG_TYPE_CTS;
    radio_tx_buffer[MSG_BYTE_DEST_ROUTE] = dest;
    radio_send_message();
}

/* Protothread contexts */

#define NUM_PT 3
static struct pt pt[NUM_PT];

/* ************************************************** */
/* ************************************************** */
/* ************************************************** */
static PT_THREAD(thread_led_green(struct pt *pt))
{
    PT_BEGIN(pt);

    while(1)
    {
        PT_WAIT_UNTIL(pt, led_green_flag);
        led_green_on();
        TIMER_LED_GREEN_ON = 0;
        PT_WAIT_UNTIL(pt, timer_reached(TIMER_LED_GREEN_ON,
          led_green_duration));
        led_green_off();
        led_green_flag = 0;
    }

    PT_END(pt);
}

static PT_THREAD(thread_led_red(struct pt *pt))
{
    PT_BEGIN(pt);

    while(1)
    {
        PT_WAIT_UNTIL(pt, led_red_flag);
        led_red_on();
        TIMER_LED_RED_ON = 0;
        PT_WAIT_UNTIL(pt, timer_reached(TIMER_LED_RED_ON,
          led_red_duration));
        led_red_off();
        led_red_flag = 0;
    }

    PT_END(pt);
}

static PT_THREAD(thread_process_msg(struct pt *pt))
{
    PT_BEGIN(pt);

    while(1)
    {
        PT_WAIT_UNTIL(pt, radio_rx_flag == 1);
	
        if(radio_rx_buffer[MSG_BYTE_TYPE] == MSG_TYPE_TEMPERATURE ){
            //&& radio_rx_buffer[MSG_BYTE_DEST_ROUTE] == ID
	    dump_message(radio_rx_buffer);
	    clear_to_send_flag = 1;
        }else if(radio_rx_buffer[MSG_BYTE_TYPE] == MSG_TYPE_RTS){
	    if (clear_to_send_flag = 1){
		printf("RTS message received from %D\n",radio_rx_buffer[MSG_BYTE_SRC_ROUTE]);
		send_cts(radio_rx_buffer[MSG_BYTE_SRC_ROUTE]);
		clear_to_send_flag = 0;
		//timer cts = 0;
	    }
	}else{
	    clear_to_send_flag = 1;
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
	timerA_register_cb(&timer_tick_cb);
	timerA_start_milliseconds(10);

	uart_init(UART_9600_SMCLK_8MHZ);
	printf("Smart GreenHouses: Sink\n\r");

	/* radio init */
	spi_init();
	cc2500_init();
	cc2500_rx_register_buffer(radio_rx_buffer, PKTLEN);
	cc2500_rx_register_cb(radio_cb);
	cc2500_rx_enter();
	radio_rx_flag = 0;
	clear_to_send_flag = 1;

  __enable_interrupt();

  //led_green_on();
	/* protothreads init */
	int i;
	for(i = 0; i < NUM_PT; i++)
	{
		PT_INIT(&pt[i]);
	}
	printf("Protothread init\n\r");
	while(1) {
	    thread_process_msg(&pt[0]);
	    thread_led_red(&pt[1]);
	    thread_led_green(&pt[2]);
	}
}

/* ************************************************** */
/* ************************************************** */