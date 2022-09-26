/*
 * Function.c
 *
 *  Created on: Sep 16, 2022
 *      Author: Dell
 */


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

/*
 * initialize Timer1 with CTC mode and set N=1024 to count the stop watch time
 *  each second timer Execute ISR
 *    */
void TIMER1_Init_CTC_Mode(void)
{
	/* initialize timer/counter register with value =0 */
	TCNT1=0;

	/* initialize output compare register =977
	 * because every second we want to increment STOP WATCH time
	 * Fclck=1MHZ  N=1024  Ftimer=1*10^6/1024=976.5625 HZ
	 * Ttimer=1/Ftimer=1.024ms
	 * OCR1A=1s/1.024*10^-3=977count
	 * */
	OCR1A=977;

	/* deactivate PWM mode on CR1A by set bit FOC1A
	 * SET N=1024 by set bit CS10,CS12
	 *  set CTC Mode on CR1A by set bit WGM12 (mode 4)
	 */
	TCCR1A=(1<<FOC1A);
	TCCR1B=(1<<WGM12)|(1<<CS10)|(1<<CS12);

	/* enable interrupt for Output Compare and Match Register A */
	TIMSK|=(1<<OCIE1A);

	/* enable global interrupt */
	SREG|=(1<<7);
}
/* after each second timer rise flag for interrupt
 *
 * ISR increments time     */
ISR(TIMER1_COMPA_vect)
{
	x.sec++;
	if(x.sec==60)
	{
		x.sec=0;
		x.mins++;
	}
	if(x.mins==60)
	{
		x.hrs++;
		x.mins=0;
	}

}
void Sev_seg_second(unsigned char seconds)
{
	unsigned short ones ,tens;

	ones=seconds%10;

	/* enable first seven segment and disable others */
	PORTA=(PORTA&0XC0)|0X01;

	/*display ones seconds on first seven segment */
	PORTC=(PORTC&0XF0)|(ones&0X0F);

	_delay_ms(2);
	seconds/=10;
	tens=seconds%10;

	/* enable second seven segment and disable others */
	PORTA=(PORTA&0XC0)|0X02;

	/*display tens seconds on second seven segment */
	PORTC=(PORTC&0XF0)|(tens&0X0F);

}

void Sev_seg_mins(unsigned char mins)
{
	unsigned short ones ,tens;

	ones=mins%10;

	/* enable third seven segment and disable others */
	PORTA=(PORTA&0XC0)|0X04;

	/*display ones minutes on third seven segment */
	PORTC=(PORTC&0XF0)|(ones&0X0F);

	_delay_ms(2);

	mins=mins/10;

	tens=mins%10;

	/* enable fourth seven segment and disable others */
	PORTA=(PORTA&0XC0)|0X08;

	/*display tens minutes on fourth seven segment */
	PORTC=(PORTC&0XF0)|(tens&0X0F);


}

void Sev_seg_hours(unsigned char hrs)
{
	unsigned short ones ,tens;

	ones=hrs%10;

	/* enable fifth seven segment and disable others */
	PORTA=(PORTA&0XC0)|0X10;

	/*display ones hours on third seven segment */
	PORTC=(PORTC&0XF0)|(ones&0X0F);

	_delay_ms(2);

	hrs=hrs/10;

	tens=hrs%10;

	/* enable sixth seven segment and disable others */
	PORTA=(PORTA&0XC0)|0X20;

	/*display tens minutes on fourth seven segment */
	PORTC=(PORTC&0XF0)|(tens&0X0F);

}

void INT0_Init(void)
{
	/* define pin2 at PORT D as input */
	DDRD&=~(1<<PD2);

	/* Activate internal pull up resistor for PD2*/
	PORTD|=(1<<PD2);

	/* define that falling edge of INTO generates an interrupt request */
	MCUCR=(MCUCR&~(1<<ISC00))|(1<<ISC01);

	/* ENBALE  MODULE INTERRUPT FOR INTO  */
	GICR|=(1<<INT0);

	/* Enable global interrupt flag */
	SREG|=(1<<7);
}

/*ISR for INT0
 * when the External interrupt0 flag is set (falling edge is detected )
 * stop watch  reset
 *
 * */

ISR(INT0_vect)
{
	x.sec=0;
	x.mins=0;
	x.hrs=0;
}

void INT1_Init(void)
{
	/*define PIN3 at portD as input */
	DDRD&=~(1<<PD3);

	/* Configure External Interrupt INT1 with raising edge*/
	MCUCR|=(1<<ISC10)|(1<<ISC11);

	/* ENBALE  MODULE INTERRUPT FOR INT1  */
	GICR|=(1<<INT1);

	/* Enable general interrupt bit   */
	SREG|=(1<<7);

}
/*
 * ISR for INT1
 * when the External interrupt1 flag is set (RISING edge is detected )
 * stop watch  should be paused
 *
 * */


ISR(INT1_vect)
{
	/* No clock source (Timer/Counter stopped)*/
	TCCR1B&=0XF8;

}


void INT2_Init(void)
{
	/* define pin2 at PORT B as input */
	DDRB&=~(1<<PB2);

	/* Activate internal pull up resistor for PD2*/
	PORTB|=(1<<PB2);

	/* define that falling edge of INT2 to generate an interrupt request */
	MCUCSR&=~(1<<ISC2);

	/* ENBALE  MODULE INTERRUPT FOR INT2  */
	GICR|=(1<<INT2);

	/* Enable global interrupt flag */
	SREG|=(1<<7);
}

/*
 * ISR for INT2
 *  when the External interrupt2 flag is set (falling edge is detected )
 * stop watch  should be resumed
 *
 * */


ISR(INT2_vect)
{
	/* clkI/O /1024 (From prescaler)*/
	TCCR1B|=(1<<CS10)|(1<<CS12);

}













