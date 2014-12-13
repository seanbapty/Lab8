Lab8
====

Maze Solving Robot
#Objectives or purpose
During this lab, you will combine the previous laboratory assignments and program your robot to autonomously navigate through a maze.
#Preliminary design/PreLab
Prior to doing any work pseudo code was written. This was really all the prelab that needed to be done because there were no hardware changes that needed to be made other than disconnect the LEDs once full functionality was achieved as they were a waste of power.
```
*Constants specific to each IR sensor*


hieracrchy of movements-will be performed unless IR input denotes it cannot and then it will check the next function
	move forward
	move left
	move right

while can move forward
	go forward
	else go left
	else go right
```
	
#Software flow chart / algorithms
#Hardware schematic
Below is a culminating schematic of all the necessary pieces to make an autonomous moving robot.
![alt tag](https://raw.githubusercontent.com/seanbapty/Lab8/master/final%20schematic.jpg)
#Well-formatted code
See lab 6 for a description of development of robot movement functionality, and lab 7 for a description of IR sensor implementation. The only code specifically unique to this lab was the use of IR readings to drive the motors. 
Below is the main.c file which updated a robots movement direction based on IR input.
```
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
	ADC10CTL1 = INCH_5;								// left wired to pin 5
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
```
As is apparent, the move functions are the same as lab 6 and the IR functions are the same as lab 7.
#Debugging
The main error encountered while solving the maze was activating multiple move functions at the same time. When multiple move functions are active, it is difficult to determine the output because the operator does not know which function will over power the other function, or if they will cancel each other out, or if they will create a completely different movement. The scenario where multiple functions combined to do something unique was especially difficult to discern what the problem was because the LED light was being illuminated when it was hard to tell that it should have been. This problem was solved by implementing all movement functions within a single "if else" statement. That way, only one can be run at a time.
#Testing methodology / results
The primary mode of testing was to either observe how the wheels reacted to certain IR input (by moving a wall/paper toward or away from them) or place the robot in the maze and see how it performed. The first test in the maze caused the robot to accelerate quickly forward and not make the left turn in time. This was fixed by inputting stop commands to slow down the robots movement and allow the IR sensors to get better data.
Ultimately, a slow moving robot that responded whenever it got to close to a wall else went forward was created.
#Observations and Conclusions
####Documentation
