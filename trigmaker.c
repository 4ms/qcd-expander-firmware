#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/***********************
 *  COMPILE SETTINGS   *
 ***********************/

/*
DEBUG
*/
//#define DEBUG

/************************
 * Mnuemonics		*
 ************************/
//fosc=16MHz
//5ms = 40 overflows of TCNT0
#define TRIG_LENGTH 39

#define FALLTRIG 0
#define INVGATE 2
#define RISEFALLTRIG 1

/********************
 * GLOBAL VARIABLES *
 ********************/
volatile uint8_t out0_tmr;
volatile uint8_t out1_tmr;
volatile uint8_t out2_tmr;
volatile uint8_t out3_tmr;
volatile uint8_t out4_tmr;
volatile uint8_t out5_tmr;
volatile uint8_t out6_tmr;
volatile uint8_t out7_tmr;
volatile uint8_t out8_tmr;

/*******************
 * PIN DEFINITIONS *
 *******************/

#ifdef DEBUG
#define DEBUG_pin PA1
#define DEBUG_init DDRA |= (1<<DEBUG_pin)
#define DEBUGFLIP PORTA ^= (1<<DEBUG_pin)
#define DEBUGHIGH PORTA |= (1<<DEBUG_pin)
#define DEBUGLOW PORTA &= ~(1<<DEBUG_pin)
#endif

#define IN0_pin PB3
#define IN0_init DDRB &= ~(1<<IN0_pin); PORTB |= (1<<IN0_pin)
#define IN0 ((PINB & (1<<IN0_pin)))

#define IN1_pin PB2
#define IN1_init DDRB &= ~(1<<IN1_pin); PORTB |= (1<<IN1_pin)
#define IN1 ((PINB & (1<<IN1_pin)))

#define IN2_pin PB1
#define IN2_init DDRB &= ~(1<<IN2_pin); PORTB |= (1<<IN2_pin)
#define IN2 ((PINB & (1<<IN2_pin)))

#define IN3_pin PB0
#define IN3_init DDRB &= ~(1<<IN3_pin); PORTB |= (1<<IN3_pin)
#define IN3 ((PINB & (1<<IN3_pin)))

#define sense0_pin (1<<PD2)
#define sense1_pin (1<<PD3)
#define sense2_pin (1<<PD4)
#define sense3_pin (1<<PD5)
#define sensein_pins (sense0_pin | sense1_pin | sense2_pin | sense3_pin)
#define sensein_init DDRD &= ~(sensein_pins); PORTD &= ~(sensein_pins)
#define sensein_read (PIND)

#define OUT0_pin PB7
#define OUT0_init DDRB |= (1 << OUT0_pin)
#define OUT0_ON PORTB |= (1 << OUT0_pin)
#define OUT0_OFF PORTB &= ~(1 << OUT0_pin)

#define OUT1_pin PB6
#define OUT1_init DDRB |= (1 << OUT1_pin)
#define OUT1_ON PORTB |= (1 << OUT1_pin)
#define OUT1_OFF PORTB &= ~(1 << OUT1_pin)

#define OUT2_pin PB5
#define OUT2_init DDRB |= (1 << OUT2_pin)
#define OUT2_ON PORTB |= (1 << OUT2_pin)
#define OUT2_OFF PORTB &= ~(1 << OUT2_pin)

#define OUT3_pin PB4
#define OUT3_init DDRB |= (1 << OUT3_pin)
#define OUT3_ON PORTB |= (1 << OUT3_pin)
#define OUT3_OFF PORTB &= ~(1 << OUT3_pin)

#define senseout_pin PD0
#define senseout_init DDRD |= (1 << senseout_pin)
#define senseout_ON PORTD |= (1 << senseout_pin)
#define senseout_OFF PORTD &= ~(1 << senseout_pin)
#define senseout_TOGGLE PORTD ^= (1 << senseout_pin)



SIGNAL (TIMER0_OVF_vect){
	out0_tmr++;
	out1_tmr++;
	out2_tmr++;
	out3_tmr++;
}


/***************************************************
 *             MAIN() FUNCTION                     *
 *                                                 *
 ***************************************************/


int main(void){
	uint8_t t;

	uint8_t IN0_washigh=0;
	uint8_t IN1_washigh=0;
	uint8_t IN2_washigh=0;
	uint8_t IN3_washigh=0;
	uint8_t out0_high=0;
	uint8_t out1_high=0;
	uint8_t out2_high=0;
	uint8_t out3_high=0;

	uint8_t mode_read_high=0;
	uint8_t mode_read_low=0;
	uint8_t switch_state_0=RISEFALLTRIG,switch_state_1=RISEFALLTRIG,switch_state_2=RISEFALLTRIG,switch_state_3=RISEFALLTRIG;

	uint8_t sense_ctr=255;
	uint8_t sense_state=0;


	/** Initialize **/

	//Setup timer: Normal mode, TOP at 0xFF, OC0A and OC0B disconnected, Prescale @ FCK/8, Enable Overflow Interrupt
	TCCR0A=(0<<COM0A0) | (0<<COM0A1) | (0<<COM0B0) | (0<<COM0B1) | (0<<WGM01) | (0<<WGM00) ;
	TCCR0B= (0<<WGM02) | (0<<CS00) | (1<<CS01) | (0<<CS02);
	TCNT0=0;
	TIMSK |= (1<<TOIE0); 


	IN0_init;
	IN1_init;
	IN2_init;
	IN3_init;
	sensein_init;
	OUT0_init;
	OUT1_init;
	OUT2_init;
	OUT3_init;
	senseout_init;

#ifdef DEBUG_init
	DEBUG_init;
#endif

	senseout_ON;

	OUT0_OFF;
	OUT1_OFF;
	OUT2_OFF;
	OUT3_OFF;

	sei();

	/** Main loop 3-7us without senseout**/
	while(1){
		// Sense out is a cheap way to detect if the four On/Off/On switches are in the up, center, or down positions,
		// with just using one GPIO per switch (sense0-sense3) plus one extra GPIO (senseout).
		// The up and down positions of the switches are tied to Vcc and GND. The poles are connected to senseout via series resistors.
		// Periodically we set the senseout pin high, then read all the switches. Then we set senseout low and read the switches again.
		// We then know the switch position based on if it was read as high for both reads, or low for both reads, or high then low.
		if (--sense_ctr==0){
			sense_state++;

			if (sense_state==1){
				mode_read_high=sensein_read;
				senseout_OFF;
			}

			else if (sense_state==2){
				mode_read_low=sensein_read;
				senseout_ON;
			}

			else if (sense_state==3){

				if (mode_read_high & sense0_pin) {
					if (mode_read_low & sense0_pin)
						switch_state_0=RISEFALLTRIG;
					else 
						switch_state_0=INVGATE;
				} else
					switch_state_0=FALLTRIG;
			}

			else if (sense_state==4){

				if (mode_read_high & sense1_pin) {
					if (mode_read_low & sense1_pin)
						switch_state_1=RISEFALLTRIG;
					else 
						switch_state_1=INVGATE;
				} else
					switch_state_1=FALLTRIG;
			}

			else if (sense_state==5){

				if (mode_read_high & sense2_pin) {
					if (mode_read_low & sense2_pin)
						switch_state_2=RISEFALLTRIG;
					else 
						switch_state_2=INVGATE;
				} else 
					switch_state_2=FALLTRIG;
			}

			else if (sense_state==6){

				if (mode_read_high & sense3_pin) {
					if (mode_read_low & sense3_pin)
						switch_state_3=RISEFALLTRIG;
					else 
						switch_state_3=INVGATE;
				} else 
					switch_state_3=FALLTRIG;
				
				sense_state=0;
			}
		}


		if (IN0) { 					//Input 0 is high
			if (!IN0_washigh){ 		//First time detected high = Rising Edge
				IN0_washigh=1;
			
				if (switch_state_0==RISEFALLTRIG) {
					OUT0_ON;			//RISEFALLTRIG mode goes high on rising edge
					out0_high=1;
					cli();
						out0_tmr=0;
					sei();
				}
				else if (switch_state_0==INVGATE){
					OUT0_OFF;			//INVGATE mode goes low on rising edge
					out0_high=0;
				}
			}
		} else {					//Input 0 is low
			if (IN0_washigh) {		//First time detected low = Falling Edge
				IN0_washigh=0;

				OUT0_ON;			//All modes go high on falling edge
				out0_high=1;
				cli();
					out0_tmr=0;
				sei();

			}
		}


		if (IN1) { 					//Input 1 is high
			if (!IN1_washigh){ 		//First time detected high = Rising Edge
				IN1_washigh=1;
			
				if (switch_state_1==RISEFALLTRIG) {
					OUT1_ON;			//RISEFALLTRIG mode goes high on rising edge
					out1_high=1;
					cli();
						out1_tmr=0;
					sei();
				}
				else if (switch_state_1==INVGATE){
					OUT1_OFF;			//INVGATE mode goes low on rising edge
					out1_high=0;
				}
			}
		} else {					//Input 1 is low
			if (IN1_washigh) {		//First time detected low = Falling Edge
				IN1_washigh=0;

				OUT1_ON;			//All modes go high on falling edge
				out1_high=1;
				cli();
					out1_tmr=0;
				sei();

			}
		}





		if (IN2) { 					//Input 2 is high
			if (!IN2_washigh){ 		//First time detected high = Rising Edge
				IN2_washigh=1;
			
				if (switch_state_2==RISEFALLTRIG) {
					OUT2_ON;			//RISEFALLTRIG mode goes high on rising edge
					out2_high=1;
					cli();
						out2_tmr=0;
					sei();
				}
				else if (switch_state_2==INVGATE){
					OUT2_OFF;			//INVGATE mode goes low on rising edge
					out2_high=0;
				}
			}
		} else {					//Input 2 is low
			if (IN2_washigh) {		//First time detected low = Falling Edge
				IN2_washigh=0;

				OUT2_ON;			//All modes go high on falling edge
				out2_high=1;
				cli();
					out2_tmr=0;
				sei();

			}
		}

		if (IN3) { 					//Input 3 is high
			if (!IN3_washigh){ 		//First time detected high = Rising Edge
				IN3_washigh=1;
			
				if (switch_state_3==RISEFALLTRIG) {
					OUT3_ON;			//RISEFALLTRIG mode goes high on rising edge
					out3_high=1;
					cli();
						out3_tmr=0;
					sei();
				}
				else if (switch_state_3==INVGATE){
					OUT3_OFF;			//INVGATE mode goes low on rising edge
					out3_high=0;
				}
			}
		} else {					//Input 3 is low
			if (IN3_washigh) {		//First time detected low = Falling Edge
				IN3_washigh=0;

				OUT3_ON;			//All modes go high on falling edge
				out3_high=1;
				cli();
					out3_tmr=0;
				sei();

			}
		}



		if (switch_state_0!=INVGATE){
			cli();
				t=out0_tmr;
			sei();
			if (out0_high && t>TRIG_LENGTH){
				OUT0_OFF;
				out0_high=0;
			}
		}

		if (switch_state_1!=INVGATE){
			cli();
				t=out1_tmr;
			sei();
			if (out1_high && t>TRIG_LENGTH){
				OUT1_OFF;
				out1_high=0;
			}
		}

		if (switch_state_2!=INVGATE){
			cli();
				t=out2_tmr;
			sei();
			if (out2_high && t>TRIG_LENGTH){
				OUT2_OFF;
				out2_high=0;
			}
		}

		if (switch_state_3!=INVGATE){
			cli();
				t=out3_tmr;
			sei();
			if (out3_high && t>TRIG_LENGTH){
				OUT3_OFF;
				out3_high=0;
			}
		}



	} //main loop

} //void main()





