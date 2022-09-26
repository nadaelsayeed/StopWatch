/*
 * Main.c
 *
 *  Created on: Sep 15, 2022
 *      Author: Dell
 */

#include"Function.h"
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
typedef struct
{
	unsigned char sec;
	unsigned char mins;
	unsigned char hrs;
}Time;
Time x;

int main(void)
{
	/* define 4 pins in PORTC as Output that are connected to DECODER*/
	DDRC|=0X0F;

	/* define 6 pins in portA  as the enable/disable pins for the six 7-segments. */
	DDRA|=0X3F;

	/* disable 6 pins at first */
	PORTA&=0XC0;

	x.sec=0;
	x.mins=0;
	x.hrs=0;

	TIMER1_Init_CTC_Mode();
	INT0_Init();
	INT1_Init();
	INT2_Init();
	while(1)
	{
		Sev_seg_second(x.sec);
		_delay_ms(2);
		Sev_seg_mins(x.mins);
		_delay_ms(2);
		Sev_seg_hours(x.hrs);
		_delay_ms(2);

	}




}







