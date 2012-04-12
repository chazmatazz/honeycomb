// servo.c

#include "avr_compiler.h"
#include "servo.h"

uint8_t SERVO_MODE = QUICK_SERVO;
int16_t smart_servo_pos_deg = -30;
int16_t servo_current_position;


/*
#define PIN1_bm		0b00000001
#define PIN2_bm		0b00000010
#define PIN3_bm		0b00000100
#define PIN4_bm		0b00001000
#define PIN5_bm		0b00010000
#define PIN6_bm		0b00100000
#define PIN7_bm		0b01000000
#define PIN8_bm		0b10000000
*/

void init_servo()
{

	/* the servo code that is written here will:
		1. choose an on-board timer/counter (we will use timer/counter E1 which will drive single-slope PWM on portEpin4 and portEpin5)
		2. supply a scaled clock source that will drive this counter (hopefully 32MHz/64 = 500 kHz)
		3. initialize single-slope PWM and output it to pin 40 (OC1A, portEpin4) and/or pin 41 (OC1B, portEpin5)
	*/
	
	// SERVO: initialize the two servo data-pins as output pins
	
		PORTE.DIRSET = PIN4_bm; // (PWM_1) make pin E4 writeable, by default all initialize as readable
		PORTE.DIRSET = PIN5_bm; // (PWM_2) make pin E5 writeable, by default all initialize as readable
	
	if(SERVO_MODE == QUICK_SERVO)
	{
	// SERVO: Begin a timer-counter that will reach TOP (overflow) every 20ms (servo signal is 50 Hz)
	
		TCE1.PER = 10126; // Set period (10126 ticks@~500KHz -> ~20.0ms) ** CALIBRATED (w/ opt.lvl -O1), meas abt. 506.27 ticks/ms **
		//TC_SetPeriod( &TCE1, 9999 );  // Set period (10000 ticks@500KHz = 20ms)  ** CORRECT, but NOT CALIBRATED **
		
		TCE1.CCA = 785;	//initially fill the CCA register with a value that is calibrated to zero the servo (~1.5 ms on time)	[PWM1]
		TCE1.CCB = 785;	//initially fill the CCB register with a value that is calibrated to zero the servo (~1.5 ms on time)	[PWM2]
		TCE1.CTRLA |= 0b00000101;		// Set clock and prescaler, 32MHz/64 = 500KHz
		TCE1.CTRLB |= 0b00000011;		// enable Single Slope PWM (Waveform Generation Mode)
		TCE1.CTRLB |= 0b00010000;		// enable waveform output on OC1A (setting this in WGM of operation overrides the port output register for this output pin: portEpin4)
		TCE1.CTRLB |= 0b00100000;		// enable waveform output on OC1B (setting this in WGM of operation overrides the port output register for this output pin: portEpin5)
	}	
	
	else if(SERVO_MODE == SMART_SERVO)
	{
	// SERVO: Begin a clock counter that will trigger an overflow-interrupt (TCE1) every 20ms -> (50 Hz)
	
		TCE1.PER = 10126;  // Set period (10126 ticks@~500KHz -> ~20.0ms) ** CALIBRATED (w/ opt.lvl -O1), meas abt. 506.27 ticks/ms **
		
		//TC1_SetOverflowIntLevel( &TCE1, TC_OVFINTLVL_MED_gc );  // set a medium-level interrupt on overflow
		TCE1.INTCTRLA |= 0b00000010;	// set a medium-level interrupt on overflow
		//TC1_ConfigClockSource( &TCE1, TC_CLKSEL_DIV64_gc );    // Set clock and prescaler, 32MHz/64 = 500KHz
		TCE1.CTRLA |= 0b00000101;		// Set clock and prescaler, 32MHz/64 = 500KHz
		
		TCE1.PER = 10126;  // Set period (10126 ticks@~500KHz -> ~20.0ms) ** CALIBRATED (w/ opt.lvl -O1), meas abt. 506.27 ticks/ms **
		
		TCE1.INTCTRLB |= 0b00000010;  // set a medium-level interrupt on CCA compare
		
	}	
}	

uint16_t servo_CNT_compare_from_postion(float degrees)
{
	uint16_t compare_value;		// this value will ultimately fill the PER register of the timer/counter

	compare_value = 785 + degrees*5; // ** CALIBRATED (w/ opt.lvl -O1) for +/- 90 degrees and ZEROED when servo_position_deg==0 **
	// compare_value = 760 + (int16_t)degrees*2.7777; // ** CORRECT, but NOT CALIBRATED **

	return compare_value;
}

void set_servo_position(float degrees)
{
	if(SERVO_MODE == QUICK_SERVO)
	{
		if(servo_current_position == degrees)
			return;
		else
		{ 
			uint16_t compare_value = servo_CNT_compare_from_postion(degrees);		

			TCE1.CCABUF = compare_value;	// fill the clock-compare A-buffer with the servo position val (for pin OC1A)
											// CCABUF will be loaded into CCB on the next UPDATE event (counter value = BOTTOM)
	
			TCE1.CCBBUF = compare_value;	// fill the clock-compare B-buffer with the servo position val (for pin OC1B)
											// CCBBUF will be loaded into CCB on the next UPDATE event (counter value = BOTTOM)								
		
			servo_current_position = degrees;
		}		
	}
	
	else
		/* not yet configured for SMART_SERVO */
		return;
}	

ISR(TCE1_OVF_vect)
{
	PORTB.OUT |= PIN1_bm; //(YELLOW LED) turn the bit on (for debugging, remove this line when complete)
	
	// SERVO: Begin the HIGH-pulse to the servo:
		
		PORTE.OUT |= PIN4_bm; // (PWM_1) set pin on
		PORTE.OUT |= PIN5_bm; // (PWM_2) set pin on
	
	// SERVO: calculate how long the HIGH-pulse to the servo should last
	
	set_servo_position(smart_servo_pos_deg);
	
}

ISR(TCE1_CCA_vect)
{
    PORTB.OUT &= ~PIN1_bm; //(YELLOW LED) turn the bit off (for debugging, remove this line when complete)
	
	// SERVO: End the HIGH-pulse to the servo, clear the interrupt, stop counting on TCD1 :
	
		PORTE.OUT &= ~PIN4_bm; // (PWM_1) set pin off
		PORTE.OUT &= ~PIN5_bm; // (PWM_2) set pin off
	
		//TC1_SetOverflowIntLevel( &TCE1, TC_OVFINTLVL_OFF_gc ); // Clear Interrupt on Overflow (is this necessary??)
		//TC1_ConfigClockSource( &TCE1, TC_CLKSEL_OFF_gc );      // Stop clock
}
