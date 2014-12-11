#include <msp430.h>
#include "lab6.h"
//#include "moveRobot.c"

#include <msp430.h>

/*
 * main.c
 * Sean Bapty
 * toggles LED 1 when proximity to a wall on the center is less than ~1.5 in.
 * toggles LED 2 when proximity to a wall on the right/left is less than ~1.5 in.
 */
//-----------------------------------------------------------------
// Name:	Coulston
// File:	lab7.c
// Date:	Fall 2014
// Purp:	Collect analog samples from P1.3 and P1.4
//-----------------------------------------------------------------
#include "msp430g2553.h"
#include "lab7.h"

void initMSP430();

#define LEFT_CLOSE_BOUND		0x130							// closer the robot is to the wall the higher the ADC output
#define LEFT_FAR_BOUND			0x170
#define ABSOLUTE_DISTANCE		0x75							// blocks out stray signals by only recieving signals within maze size

#define RIGHT_CLOSE_BOUND		0x200							// closer the robot is to the wall the higher the ADC output
#define RIGHT_FAR_BOUND			0x100

#define CENTER_CLOSE_BOUND		0x150							// closer the robot is to the wall the higher the ADC output
#define CENTER_FAR_BOUND		0x100
//----------------------------------------------------------------------
//----------------------------------------------------------------------
int main(void) {

    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	unsigned short sample[16];									// Just to analyze the values
	unsigned char i = 0;										// index into sample array

	IFG1=0; 													// clear interrupt flag1

	BCSCTL1 = CALBC1_8MHZ;										// 8MHz clock
	DCOCTL = CALDCO_8MHZ;

	RED_LED_ENABLE;												// Set the red LED as output
	ORANGE_LED_ENABLE;											// Set orange to output

	ADC10CTL0 = ADC10SHT_3 + ADC10ON + ADC10IE;
	ADC10CTL1 = ADC10DIV_7;
	ADC10CTL1 |= ADC10SSEL1 | ADC10SSEL0;

	initPWM();

	P1OUT &= ~BIT6;
	P1OUT &= ~BIT0;

	int lADC;
	int rADC;
	int cADC;

	while(1) {

		P1OUT &= ~BIT6;
		P1OUT &= ~BIT0;

		initPWM();
		leftIR();
		lADC = ADC10MEM;
		rightIR();
		rADC = ADC10MEM;
		centerIR();
		cADC = ADC10MEM;

		if (cADC > CENTER_CLOSE_BOUND && lADC < LEFT_CLOSE_BOUND){
			hardLeft();
			//stop();
			P1OUT |= BIT6;
		}
		else if (cADC > CENTER_CLOSE_BOUND && rADC < RIGHT_CLOSE_BOUND){
			hardLeft();
			//stop();
			P1OUT |= BIT6;
		}
		else if (rADC < RIGHT_CLOSE_BOUND && lADC < LEFT_CLOSE_BOUND){
			moveForward();
			stop();
		}
		else if (lADC > LEFT_CLOSE_BOUND){
			P1OUT |= BIT6;
			//stop();
			hardRight();
			stop();
		}
		else if (rADC > RIGHT_CLOSE_BOUND){
			hardLeft();
			//stop();
		}

		/*
		else if (lADC < LEFT_FAR_BOUND){
			P1OUT |= BIT0;
			hardLeft();
			//moveForward();
		}
		else if (lADC > LEFT_CLOSE_BOUND){
			P1OUT |= BIT0;
			hardRight();
			//__delay_cycles(SHORT_DELAY);
		}

/*
		else if (ADC10MEM < LEFT_FAR_BOUND){
			leftIR();
			//initPWM();
			P1OUT &= ~BIT6;
			stop();
			hardLeft();
			__delay_cycles(LONG_DELAY);
			moveForward();
			__delay_cycles(LONG_DELAY*2);
		}
		__delay_cycles(SHORT_DELAY);
		P1OUT &= ~BIT6;
		P1OUT &= ~BIT0;
/*

		__delay_cycles(10000);
		centerIR();
		if(ADC10MEM < CENTER_CLOSE_BOUND){
			stop();
			moveForward();
		}
		else{
			hardRight();
			stop();
		}
		__delay_cycles(10000);
/*
		initPWM();
		centerIR();
		if (ADC10MEM > CENTER_CLOSE_BOUND){
			P1OUT |= BIT0;
			hardRight();
			//moveBackward();
			stop();
		}
		__delay_cycles(10000);


		P1OUT &= ~BIT6;
		P1OUT &= ~BIT0;
/*
		rightIR();
		if (ADC10MEM > RIGHT_CLOSE_BOUND){
			P1OUT |= BIT0;
			moveForward();
			stop();
		}
		else{
			P1OUT &= ~BIT6;

		}
		P1OUT &= ~BIT6;
		P1OUT &= ~BIT0;
		*/
	} // end main
	return 0;
}
unsigned int leftIR(){
	ADC10CTL0 &= ~ENC;								// clear ADC prior to changing
	ADC10CTL1 = INCH_5;								// left wired to pin 4
	ADC10AE0 |= BIT5;
	ADC10CTL0 |= ENC + ADC10SC;
	return ADC10MEM;
}

unsigned int centerIR(){
	ADC10CTL0 &= ~ENC;								// clear ADC prior to changing
	ADC10CTL1 = INCH_4;								// center wired to pin 4
	ADC10AE0 |= BIT4;
	ADC10CTL0 |= ENC + ADC10SC;
	return ADC10MEM;
}

unsigned int rightIR(){
	ADC10CTL0 &= ~ENC;								// clear ADC prior to changing
	ADC10CTL1 = INCH_2;								// right wired to pin 2
	ADC10AE0 |= BIT2;
	ADC10CTL0 |= ENC + ADC10SC;
	return ADC10MEM;
}

