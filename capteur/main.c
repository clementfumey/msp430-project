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
#define COEFF_1_ADDR INFOD_START+1
#define COEFF_2_ADDR INFOD_START+3


#define MSG_BYTE_TYPE 0U //First Byte is type of message
#define MSG_BYTE_DEST_ROUTE 1U //Second Byte is dest id
#define MSG_BYTE_SRC_ROUTE 2U // Third is source id
#define MSG_BYTE_CONTENT 3U // Fourth is content

#define PKTLEN 10 //Packet lenght

// First Byte : Type of message

#define MSG_TYPE_TEMPERATURE 0x02

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
#define NUM_TIMERS 4
static uint16_t timer[NUM_TIMERS];
#define TIMER_SEND_TEMP timer[0]
#define TIMER_ANTIBOUNCING timer[1]
#define TIMER_LED_RED_ON timer[2]
#define TIMER_LED_GREEN_ON timer[3]

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

static char radio_tx_buffer[PKTLEN];

static void radio_send_message()
{
    cc2500_utx(radio_tx_buffer, PKTLEN);
}

/* to be called from within a protothread */
static void init_message()
{
    unsigned int i;
    for(i = 0; i < PKTLEN; i++)
    {
        radio_tx_buffer[i] = 0x00;
    }
    radio_tx_buffer[MSG_BYTE_SRC_ROUTE] = ID;
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

/* Protothread contexts */

#define NUM_PT 5
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
        PT_WAIT_UNTIL(pt, send_mode_flag && timer_reached( TIMER_SEND_TEMP, 1000));
        send_temperature();
        
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
	
	/* radio init */ //rx init missing
	spi_init();
	cc2500_init();
	
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
	}
}

/* ************************************************** */
/* ************************************************** */