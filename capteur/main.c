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

#if defined IDDEF && IDDEF > 1
#define ID IDDEF
#else
#define ID 255
#endif

#define COEFF_1_ADDR INFOD_START+1
#define COEFF_2_ADDR INFOD_START+3


#define MSG_BYTE_TYPE 0U //First Byte is type of message
#define MSG_BYTE_DEST_ROUTE 1U //Second Byte is dest id
#define MSG_BYTE_SRC_ROUTE 2U // Third is source id
#define MSG_BYTE_CONTENT 3U // Fourth is content

#define PKTLEN 10 //Packet lenght

// First Byte : Type of message
#define MSG_TYPE_RTS 0x01
#define MSG_TYPE_CTS 0x02
#define MSG_TYPE_TEMPERATURE 0x03

#define DBG_PRINTF printf

/* ************ */
/*     Leds     */
/* ************ */

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

/* ************ */
/*    Timers    */
/* ************ */
#define NUM_TIMERS 5
static uint16_t timer[NUM_TIMERS];
#define TIMER_SEND_TEMP timer[0]
#define TIMER_ANTIBOUNCING timer[1]
#define TIMER_LED_RED_ON timer[2]
#define TIMER_LED_GREEN_ON timer[3]
#define TIMER_RTS_CTS timer[4]

#define RTS_TIMEOUT 500

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
/* ************ */
/*    Button    */
/* ************ */

#define ANTIBOUNCING_DURATION 10 /* 10 timer counts = 100 ms */
static int antibouncing_flag;
static int button_pressed_flag;
static int send_mode_flag;

void button_pressed_cb()
{
    if(antibouncing_flag == 0)
    {
        button_pressed_flag = 1;
        antibouncing_flag = 1;
        TIMER_ANTIBOUNCING = 0;
        if (send_mode_flag){
			led_red_blink(100);
			send_mode_flag = 0;
			printf("send_mode_flag = 0\n");
		}else{
			send_mode_flag = 1;
			led_green_blink(100);
			printf("send_mode_flag = 1\n");
		}
    }
}



/* ************ */
/*    Radio     */
/* ************ */
static int clear_to_send_flag;
static int rts_retry;
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
	//cc2500_idle();
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
    for(i=0; i<PKTLEN; i++)
    {
      radio_tx_buffer[i] = 0x00;
    }
    radio_tx_buffer[MSG_BYTE_SRC_ROUTE] = ID;
    /* useless bytes because cc2500 driver don't like small packets*/
    radio_tx_buffer[6] = 0x04;
    radio_tx_buffer[7] = 0x03;
    radio_tx_buffer[8] = 0x02;
    radio_tx_buffer[9] = 0x01;
}

/* to be called from within a protothread */
static void send_temperature()
{
	led_green_blink(10);
    init_message();
    radio_tx_buffer[MSG_BYTE_TYPE] = MSG_TYPE_TEMPERATURE;
    int temperature = adc10_sample_temp();
    printf("temp: %d,%d°C\n\r",temperature/10,temperature%10);
    /* msp430 is little endian, convert temperature to network order */
    char *pt = (char *) &temperature;
    radio_tx_buffer[MSG_BYTE_CONTENT] = pt[1];
    radio_tx_buffer[MSG_BYTE_CONTENT + 1] = pt[0];
    radio_send_message();
}

/* to be called from within a protothread */
static void send_rts()
{
	led_green_blink(10);
    init_message();
    radio_tx_buffer[MSG_BYTE_TYPE] = MSG_TYPE_RTS;
    radio_tx_buffer[MSG_BYTE_DEST_ROUTE] = 1;
    radio_tx_buffer[MSG_BYTE_CONTENT] = 0x05;
    radio_tx_buffer[MSG_BYTE_CONTENT + 1] = 0x06;
	radio_send_message();
}




/* Protothread contexts */

#define NUM_PT 6
static struct pt pt[NUM_PT];

/* ************************************************** */
/* ************************************************** */
/* ************************************************** */
static PT_THREAD(thread_button(struct pt *pt))
{
    PT_BEGIN(pt);

    while(1)
    {
        PT_WAIT_UNTIL(pt, button_pressed_flag == 1);
        button_pressed_flag = 0;
    }


    PT_END(pt);
}

static PT_THREAD(thread_antibouncing(struct pt *pt))
{
    PT_BEGIN(pt);

    while(1)
    {
        PT_WAIT_UNTIL(pt, antibouncing_flag
          && timer_reached(TIMER_ANTIBOUNCING, ANTIBOUNCING_DURATION));
        antibouncing_flag = 0;
    }

    PT_END(pt);
}

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

static PT_THREAD(thread_send_temp(struct pt *pt))
{
    PT_BEGIN(pt);

    while(1)
    {
        TIMER_SEND_TEMP = 0;
        rts_retry = 0;
        PT_WAIT_UNTIL(pt, send_mode_flag && timer_reached( TIMER_SEND_TEMP, 1000));
        while(rts_retry < 5 && clear_to_send_flag != 1) {
			TIMER_RTS_CTS = 0;
			printf("Try number: %d\n CTS-flag: %d\n", rts_retry, clear_to_send_flag);
			send_rts(); //TODO add randomness sleep before every retry;
			/* Wait until a cts has been received, or until the
			   timer expires or until rts_retry reach 5. If the timer expires, we should send the packet
			   again. */
			PT_WAIT_UNTIL(pt,  clear_to_send_flag || timer_reached(TIMER_RTS_CTS, RTS_TIMEOUT));
			rts_retry++;
		}
        if (clear_to_send_flag){
			send_temperature();
			clear_to_send_flag = 0;
		}else{
			printf("Too many retries");
		}
        
    }

    PT_END(pt);
}

static PT_THREAD(thread_process_msg(struct pt *pt))
{
    PT_BEGIN(pt);

    while(1)
    {
        PT_WAIT_UNTIL(pt, radio_rx_flag == 1);
	
        if(radio_rx_buffer[MSG_BYTE_TYPE] == MSG_TYPE_CTS && radio_rx_buffer[MSG_BYTE_DEST_ROUTE] == ID){
			clear_to_send_flag = 1;
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
	volatile int adc_coeff_1, adc_coeff_2 = 0;
	watchdog_stop();

	set_mcu_speed_dco_mclk_16MHz_smclk_8MHz();
	
	/* led init */
	leds_init();
	
	/* timer init */
	timerA_init();
    timerA_register_cb(&timer_tick_cb);
    timerA_start_milliseconds(10);
    
	/* uart init */
	uart_init(UART_9600_SMCLK_8MHZ);
	printf("Smart GreenHouses: temperature sensor\n\r");
	
	/* temp init */
	adc10_start();
	
	/* radio init */
	spi_init();
	cc2500_init();
	cc2500_rx_register_buffer(radio_rx_buffer, PKTLEN);
	cc2500_rx_register_cb(radio_cb);
	cc2500_rx_enter();
	radio_rx_flag = 0;
	clear_to_send_flag = 0;
	
	/* button init */
    button_init();
    button_register_cb(button_pressed_cb);
    antibouncing_flag = 0;
    button_pressed_flag = 0;
    send_mode_flag = 1;
    
	/* We search for calibration data in flash */
	if (*((char *)COEFF_1_ADDR) !=0xFFFF){
		/* Get calibration data from flash */
		char *foo = (char *) &adc_coeff_1;
		foo[0] = *((char *)COEFF_1_ADDR+1);
		foo[1] = *((char *)COEFF_1_ADDR);

		printf("Using a custom coeff : %d\n", adc_coeff_1);
		adc10_calibrate(adc_coeff_1, 0);
	}
	if (*((char *)COEFF_2_ADDR) !=0xFFFF){
		/* Get calibration data from flash */
		char *foo = (char *) &adc_coeff_2;
		foo[0] = *((char *)COEFF_2_ADDR+1);
		foo[1] = *((char *)COEFF_2_ADDR);
		adc10_calibrate(0, adc_coeff_2);
	}

	button_enable_interrupt();
     __enable_interrupt();


	/* protothreads init */
	int i;
	for(i = 0; i < NUM_PT; i++)
	{
		PT_INIT(&pt[i]);
	}

	while(1) {
		thread_send_temp(&pt[0]);
		thread_button(&pt[1]);
		thread_antibouncing(&pt[2]);
		thread_led_red(&pt[3]);
		thread_led_green(&pt[4]);
		thread_process_msg(&pt[5]);
		
	}
}

/* ************************************************** */
/* ************************************************** */