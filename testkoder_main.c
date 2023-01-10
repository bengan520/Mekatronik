/**** DEFINITIONS ****/

// if baudrate not defined, defines it.
#ifndef BAUDRATE
#define BAUDRATE 2400
#endif
#define F_CPU 1000000 //1 MHz clock speed
#define FOSC 1000000 //1 MHz clock speed
#define MYUBRR FOSC/16/BAUDRATE-1
#define STATE_00 0b00000000
#define STATE_01 0b00010000
#define STATE_10 0b00100000
#define STATE_11 0b00110000
#define COUNTER_BUF_SIZE 16
#define PRESCALER 8




/**** INCLUDING STUFF ****/
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h> // for printing but does not work
#include <stdlib.h>
#include <util/atomic.h>


/**** DEFINE FUNCTIONS ****/
void LED_init(void);
void LED_tester(void);
void USART_Init( unsigned int ubrr );
void Receive_Transmit_Tester (void);
unsigned char USART_Receive( void );
void USART_Transmit( unsigned char data );
void send_rpm(unsigned int value);
ISR(PCINT1_vect);
int init_INTs(void);
int updatePWM(int value);
void initPWM(void);


volatile uint8_t AB;
uint8_t ABnew;
volatile uint8_t pwmvalue =100;

int main(void)
{
    LED_init();
	USART_Init(MYUBRR);
	init_INTs();
	initPWM();
	//sei();
    while (1) 
    {
		
		
		LED_tester();			//Tests the LEDS by blinking them
		//Receive_Transmit_Tester();		//Tests full loopback to Hyperterminal by receiving a char from hyperterminal and transmitting it back
										
										
										
										
		//Activate SEI when trying the encoder but not when trying receive and led								
		//Encoder - interrupt and PWM tester is tested with empty while loop, because it uses interrupts
		//_delay_ms(250);
		
		
		
		
    }
}

//Function for blinking the LEDS
void LED_tester(void){
	
	PORTB = PORTB | (1<<PB1);
	_delay_ms(250);
	PORTD = PORTD | (1<<PD2);
	_delay_ms(250);
	PORTD = PORTD | (1<<PD3);
	_delay_ms(250);
	PORTD = PORTD | (1<<PD4);
	_delay_ms(250);
	PORTB = PORTB & ~(1<<PB1);
	_delay_ms(250);
	PORTD = PORTD & ~(1<<PD2);
	_delay_ms(250);
	PORTD = PORTD & ~(1<<PD3);
	_delay_ms(250);
	PORTD = PORTD & ~(1<<PD4);
	_delay_ms(250);
}

//Initializing the LEDS
void LED_init(void)
{
	DDRB = DDRB | (1<<PB1) | (1<<PB2);	// PB1, PB2 -> outputs
	DDRD = DDRD | (1<<PD2) | (1<<PD3) | (1<<PD4); //PD2, PD3, PD4 -> outputs
	PORTB = PORTB | (1<<PB2); //AVR-ON LED
}

//initializes USART
void USART_Init( unsigned int ubrr )
{
	
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	
	/*Enable receiver and transmitter and receiver interrupt*/
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
	
	/* Set frame format: 8data, 1stop bit , no parity*/
	// UCSR0C = [00000110] (correct from start)
	
	
}

//Transmittign function
void USART_Transmit( unsigned char data )
{
	
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	
	/* Put data into buffer, sends the data */
	UDR0 = data;
}
//Receving function
unsigned char USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) )
	;
	/* Get and return received data from buffer */
	return UDR0;
}
//Test full loopback
void Receive_Transmit_Tester (void){
	char receive = USART_Receive();
	USART_Transmit(receive);
}


int init_INTs(void)
{
	DDRC &= ~((1<<DDC4)|(1<<DDC5));	// PINC4 - PINC5 set as inputs
	PORTC &= (1<<PC4)|(1<<PC5);
	PCMSK1 |= (1<<PCINT12)|(1<<PCINT13);   // PCINT12 and PCINT13  enabled
	PCICR |= (1<<PCIE1);	// The PC interrupt group 1 (PCINT8 -> PCINT14) enabled
	return 1;
}


ISR(PCINT1_vect)
{
	
	char fram = 'f';
	char bak = 'b';
	
	ABnew = PINC & ((1<<PINC5) | (1<<PINC4));
	
	switch(ABnew)
	{

		case STATE_00 : if (AB==STATE_01){pwmvalue++;USART_Transmit(fram);} else /*AB==16*/ {pwmvalue--;USART_Transmit(bak);} break;
		case STATE_01 : if (AB==STATE_11){pwmvalue++;USART_Transmit(fram);} else /*AB==48*/ {pwmvalue--;USART_Transmit(bak);} break;
		case STATE_10 : if (AB==STATE_00){pwmvalue++;USART_Transmit(fram);} else /*AB==0*/ {pwmvalue--;USART_Transmit(bak);} break;
		case STATE_11 : if (AB==STATE_10){pwmvalue++;USART_Transmit(fram);} else /*AB==32*/ {pwmvalue--;USART_Transmit(bak);} break;

		
		default: PORTB = PORTB |(1<<PB1);
		break;
	}
	AB = ABnew;
	if(pwmvalue > 255)
	{
		pwmvalue = 255;
	}
	if(pwmvalue < 0)
	{
		pwmvalue = 0;
	}
	send_rpm(pwmvalue);
	updatePWM(pwmvalue);
		}
	
	
	/*Sends the rpm as 3 chars, one by one*/
	void send_rpm(unsigned int value) {
		char rpm[3];
		sprintf(rpm,"%d",value);
		
		for(int i=0;i<3;i++){
			
			USART_Transmit(rpm[i]);
		}
		USART_Transmit(' ');
	}



//initialize PWM
void initPWM(void)
{
	DDRD |= ((1<<DDD5)|(1<<DDD6));	//Set PIND5 and PIND6 as outputs
	TCCR0A |= 0b10110011;			//Configure fast PWM mode, non-inverted output on OCA and inverted output on OCB
	TCCR0B |= 0x01;					//Internal clock selector, no prescaler
	
}

//updates PWM registers with the value we send in..
int updatePWM(int value)
{
	OCR0A = value;
	//OCR0B = value;
	OCR0B = value;
	return value;
}