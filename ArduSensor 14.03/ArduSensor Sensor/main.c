/*
 * Project Ardunsensor
 * main.c
 * Created: 2016-03-15
 * Author: Kristjan Tozen
 * Based on Cmc code
 *
TODO:	- ADC Noise reduction
		- WDT work completely
 */ 

#define F_CPU 16000000UL					// UART needs this
#define _DH_ "0013A200" 					// XBee address of the destination Supernode //0013A200
#define _DL_ "40E53A3A"						// 40AA8840   vmcas1 - 40E53A3A  VMCAS2 - 40E568D2

#define postID "6969"						// Device ID
#define MEASURE_COUNTS 6					// How many times are we measuring capacitance? This will determine how much avaraging we will be doing.
#define measInterval 20						// Measuring interval
#define WDOG_TIMEOUT WDTO_1S				// WD wont work without it... 
#define WDTIMEO 1000						// Watchdog timeout| OPTIONS: 250 = 0.25sec; 500 = 0.5sec; 1000 = 1sec; 2000 = 2sec; 4000 = 4sec; 8000 = 8sec;

#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include "Common.h"
#include "uart.h"




// Variables
uint16_t capValue = 0;
uint16_t measCounter = 0;
uint16_t measurements[MEASURE_COUNTS]; 
char buffer[64];
uint16_t sleepCounter = 0;

// Function prototypes
uint8_t confADC(void);
uint8_t confSQG(void);
uint16_t getADC(uint8_t sensor);
uint8_t confXBEE(void);




// This function configures ADC
uint8_t confADC(void)
{
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);	// Set ADC prescaler to 128x [125kHz @ 16MHz]
	SETBIT(ADMUX,REFS0);									// Set reference voltage to AVcc
	
	CLEARBIT(ADCSRA, ADATE);
	
	ADCSRA |= (1 << ADEN) | (1 << ADSC);	// Enable ADATE, ADC and start A2D conversions
	return 1;
}


// This function configures Square Wave Generator
uint8_t confSQG(void) 
{
	// Set up timer: 
	SETBIT(DDRD,PD6);				// Set PD6[Square Wave Output] as output.
	SETBIT(DDRD,PD5);				// Set PD5[Square Wave Output] as output.
	
	TCCR0A = 0b00110011;
	TCCR0B = 0b00001001;
	
	OCR0A = 15;
	OCR0B = 8;			// Set compare match value. 11 is 25% 
	TCCR0A &= ~((1 << COM0B1) | (1 << COM0B0));		// Disables SQG output
	return 1;
	
	//TCCR0A &= ~((1 << COM0B1) | (1 << COM0B0));		// Disables SQG output
	//TCCR0A |= (1 << COM0B1) | (1 << COM0B0);		// Enables SQG output
}

// This function configures xBee
uint8_t confXBEE(void) {
		/* uart_puts("POST ID:");
		 uart_puts("\r");			// Set address
		 _delay_ms(10);
		 uart_puts(postID);
		 uart_puts("\r");
		 _delay_ms(10);*/
		 
		 uart_puts("+++"); 			//Enter command mode
		 _delay_ms(2000);

		 uart_puts("ATID 4000\r"); 	// Set PAN ID //1342  VMCAS1 ID - 4000; VMCAS2 ID - 4001
		 _delay_ms(500);
		 
		 uart_puts("ATDH"); 			// Set address
		 _delay_ms(10);
		 uart_puts(_DH_); 			// Written in #Define
		 uart_puts("\r");
		 _delay_ms(100);
		 
		 uart_puts("ATDL");
		 _delay_ms(10);
		 uart_puts(_DL_);			// Written in #Define
		 uart_puts("\r");
		 _delay_ms(100);
		 
		 
		 uart_puts("ATNH 1E\r");		// Required XBee constants
		 _delay_ms(100);
		 uart_puts("ATNO 3\r");
		 _delay_ms(100);
		 uart_puts("ATSP 7D0\r");
		 _delay_ms(100);
		 uart_puts("ATSN 21C\r");
		 _delay_ms(100);
		 uart_puts("ATSM 1\r");
		 _delay_ms(100);
		 
		 uart_puts("ATWR\r");		// Save configuration
		 _delay_ms(2000);
		 
		 uart_puts("ATCN\r");		// Exit Command mode
		 _delay_ms(1000);
		 
		 return 1;
	
}

// Function for getting different ADC values
uint16_t getADC(uint8_t sensor)
{
	uint8_t i;
	capValue = 0;
	CLEARBIT(PORTD, PD2);								// Turn on peripheral mosfet!
	SETBIT(ADCSRA, ADEN);								// Enable ADC module
	SETBIT(ADCSRA, ADSC);								// Start first conversion
	_delay_ms(1);
	switch(sensor) {
		
		// Main sensor measuring
		case 0:
		TCCR0A |= (1 << COM0B1) | (1 << COM0B0);			// Enables SQG output
		
		ADMUX &= 0b11110000;								// Set ADC channel to ADC0
		_delay_ms(1);
		break;
		
		// Calibration sensor measuring
		case 1:
		TCCR0A |= (1 << COM0B1) | (1 << COM0B0);			// Enables SQG output
		
		ADMUX &= 0b11110000;
		ADMUX |= 0b00000010;								// Set ADC channel to ADC2
		_delay_ms(1);
		break;
		
		// Voltage measuring
		case 2:
		ADMUX &= 0b11110000;
		ADMUX |= 0b00000001;
		break;
		
		// External temperature IC measuring
		case 3:
		ADMUX &= 0b11110000;
		ADMUX |= 0b00000011;
		break;
		
		// Internal CPU temperature measuring
		case 4:
		ADMUX &= 0b11110000;
		ADMUX |= 0b00001000;
		break;
	}
	
	for (i = 0; i < MEASURE_COUNTS; i++){
		SETBIT(ADCSRA, ADSC);							// Start the ADC conversion
		_delay_ms(2);
		measurements[i] = 0;
		measurements[i] = ADC;
		capValue += measurements[i];
	}
	
	capValue = capValue/i;
	
	CLEARBIT(ADCSRA, ADEN);								// Disable ADC
	TCCR0A &= ~((1 << COM0B1) | (1 << COM0B0));			// Disables SQG output
	SETBIT(PORTD, PD2);									// Turn off peripheral mosfet
	
	return capValue;
	
	
	
}

ISR(WDT_vect)
{
	// Do nothing, we just woke up the monster!
}

// THIS IS OUR MAIN FUNCTION
int main(void)
{
	// SETUP
	cli();												// Disable global interrupts when configuring!
	MCUSR = 0;											// Zero all previous reset info
	confSQG();
	confADC();
	SETBIT(DDRD, PD2);									// Peripheral mosfet as output
	SETBIT(PORTD, PD2);									// Turn off that mosfet!
	SETBIT(DDRC,PC5);									// Set xBee sleep pin as output
	DIDR1=0xFF;											// Disable digital input pins just in case
	wdt_disable();										// Disable Watchdog for now
	sei();												// Enable global interrupts
	
	uart_init(UART_BAUD_SELECT(9600,F_CPU));			// Initialize UART at 9600 baudrate.
	_delay_ms(5);										// Let's wait lil bit just in case
	confXBEE();											// We will configure xBee now
	_delay_ms(2000);	
	uart_puts("HELLO, WELCOME TO ARDUSENSOR! \n");		// Let's be polite and greet our guest!
	uart_puts("ArduSensor is configured! \n");			// Let's also let our guest know that we are configured and good to go!
	_delay_ms(2000);	
	SETBIT(PORTC, PC5);									// Set xBee to sleep!
	
	
	// Main loop
	while(1)
	{
		if(sleepCounter >= measInterval){
			
			wdt_disable();								// Disable Watchdog
			CLEARBIT(PORTC, PC5);						// Wake da xBee up, yo!
			_delay_ms(4);								// Let's actually wait for xBee to wake up
			sleepCounter = 0;							// Reset the sleep counter
			
			uint16_t mainCap, calibCap, inTemp,
			exTemp, batVolt = 0;						// Initialize and null the previous main cap read
			
			mainCap = getADC(0);						// Get the main cap reading
			calibCap = getADC(1);						// Get the calibration cap reading
			batVolt = getADC(2);
			exTemp = getADC(3);
			inTemp = getADC(4);
		
		
			/*sprintf(buffer,"<%u;%u;%u;%u;%u;%u;%s> \n",	// Formatting data for UART!  THIS IS OLD!!!
			inTemp, exTemp, batVolt, calibCap,	
			mainCap, measCounter++, postID);			*/ 
			
			sprintf(buffer,"<%u;%u;%u;%u;%u>",	// Formatting data for UART!  THIS IS OLD!!!
			exTemp, batVolt, calibCap,
			mainCap, measCounter++);
					
			uart_puts(buffer);							// Sending to UART/xBee
			_delay_ms(40);								// Lil bit of delay cuz xBee is needy mo'fucka
			
			SETBIT(PORTC, PC5);							// Let xBee sleep, nigga!
		}
		
		sleepCounter++;									// We will increment the sleepcounter now!
		
		// Configuring WatchDog
		cli();
		WDTCSR |= (1 << WDCE);
		wdt_enable(WDOG_TIMEOUT);						// Pointless basically but WD does not work without it. 
		WDTCSR |= (1 << WDCE);
		WDTCSR = __WDCONF__;
		wdt_reset();
		sei();

		// Let's go to sleep now!
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sleep_enable();
		sleep_bod_disable();
		sleep_cpu();
		sleep_disable();
		
	}
	
}



