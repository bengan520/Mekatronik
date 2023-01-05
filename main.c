

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
//#define KP 3 // Proportional gain
//#define KI 1 // Integral gain
#define FIXED_POINT_SCALE 8
#define VREF 5
#define RESOLUTION 1024
//#define dt 132	// deltaT in ms

/**** INCLUDING STUFF ****/
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h> // for printing but does not work
#include <stdlib.h>
#include <util/atomic.h>
#include <stdint.h>


/**** DEFINE FUNCTIONS ****/
void USART_Init(unsigned int ubrr);
void LED_init(void);
int init_INTs(void);
void init_Ex1(void);
void initPWM(void);
void control_clock_init(void);
void ADC_init();

void USART_Transmit( unsigned char data );
unsigned char USART_Receive( void );
void send_rpm(unsigned int value);
void stringReceiver();
int ADC_input();

void all_leds_off( void );
int updatePWM(int value);

unsigned int rpm();
unsigned long ticks_sum();
int32_t fixedPointMultiply(int32_t x, int32_t y);
void filter(unsigned int i);
void ADC_tuner();


//TODO: Ta bort delayen i while loopen o ersätt med att räkna antalet overflows för PWM timern. Annars blir delayen olika lång varje gång.
//Fixa fixed point calculations i updatepwm funktionen
// Fixa ADC ingången för att fixa potentiometern
//Multiplicera med deltaT i integraldelen


/**** DEFINE VARIABLES ****/
volatile uint8_t AB;
uint8_t ABnew;
volatile uint8_t pwmvalue =50;
volatile unsigned int counter_register[COUNTER_BUF_SIZE];
volatile unsigned int cur_buff_index = 0;
int32_t integral = 0;
int n = 0;
volatile uint8_t AB;
uint8_t ABnew;
int tuner = 0;		// int to hold the value of the ADC_tuner [-10,10]
int32_t dt = 0.132 * (1 << FIXED_POINT_SCALE); //multiplying by 2^FIXED_POINT_SCALE to keep accuracy of decimals but have it in an int 
int32_t KP = 3 << FIXED_POINT_SCALE;
int32_t KI = 10 << FIXED_POINT_SCALE;

int main(void)
{
	init_Ex1();
	LED_init();
	USART_Init(MYUBRR);
	init_INTs();
	initPWM();
	control_clock_init();
	ADC_init();
	sei();    // set global interrupt flag -> enable ALL interrupts (but those that are masked won't do anything)
	
	while (1)
	{
		
		PORTD |= ((1<<PD2)|(1<<PD3)|(1<<PD4));
		_delay_ms(500);
		PORTD &= ~((1<<PD2)|(1<<PD3)|(1<<PD4));
		_delay_ms(500);
	}
	return 1;
}



/*******************************************MATH FUNCTIONS*******************************************************
*****************************************************************************************************************
*****************************************************************************************************************/

//Filters the encoder ticks to improve accuracy of the controller
void filter(unsigned int i){
	
	ABnew = PINC & ((1<<PINC5) | (1<<PINC4));
		
		// Checks the direction of the encoder ticks, so that we filter out all the ticks that we register in the wrong direction
		switch(ABnew)
		{
	
			case STATE_00 : if (AB==STATE_01){i=0;} else /*AB==16*/ {} break;
			case STATE_01 : if (AB==STATE_11){i=0;} else /*AB==48*/ {} break;
			case STATE_10 : if (AB==STATE_00){i=0;} else /*AB==0*/ {} break;
			case STATE_11 : if (AB==STATE_10){i=0;} else /*AB==32*/ {} break;
	
			
			default: PORTB = PORTB |(1<<PB1);
			break;
		}
		AB = ABnew;
		
		
	
	/* Filter out timer values outside the range 5-237 rpm atm, allowing higher end rpm values because otherwise cannot reach correct rpm*/
	if(i>330 && i < 15625){
		counter_register[cur_buff_index%COUNTER_BUF_SIZE] = i; // Store timer value in buffer
		cur_buff_index++;
	}
}

/*Adds all the timer values in the buffer*/
unsigned long ticks_sum() {
	unsigned long sum = 0;
	for(int i = 0; i < COUNTER_BUF_SIZE; i++) {
		sum += counter_register[i];
	}
	return sum;
}
/*Calculates the rpm*/
unsigned int rpm() {
	return (60*F_CPU*COUNTER_BUF_SIZE)/((long) ticks_sum()*PRESCALER*96);
}



int32_t fixedPointMultiply(int32_t x, int32_t y) {
	int64_t result = (int64_t)x * y;
	int32_t rounding = 0.5 * (1<<FIXED_POINT_SCALE);		//Makes the rounding better for the ints
	result+=rounding;
	return result >> FIXED_POINT_SCALE;		// Divides by fixed_point_scale so that we return it with just fixed_point_scale number of decimal bits and not fixed_point_scale^2 number of bits (since we multiply 2 scaled ints)
}




/*******************************************INTERRUPT FUNCTIONS**************************************************
*****************************************************************************************************************
*****************************************************************************************************************/


/*Overflow calls the control function*/
ISR(TIMER2_OVF_vect){
	cli();
	n = n+1;
	if(n == 2){
		ADC_tuner();				
		updatePWM(pwmvalue);	//Updates PWM with 66*2= 132 ms intervals
		n=0;
	}
	sei();
}

void ADC_tuner(){
	int ADC_value = ADC_input();
	int output = (ADC_value - 512) / 51;	//output is -10 when ADC_value is 0 and 10 when it is 1023
	tuner = output;
}


/*Receive interrupt function*/
ISR(USART_RX_vect){
	char c = USART_Receive();
	if(c=='r'){
		send_rpm(rpm());
	}
	else if(c=='w'){
		stringReceiver();
	}
}

/* Timer overflow interrupt function*/ISR (TIMER1_OVF_vect)
{
	/* Toggle a pin on timer overflow */
	
	USART_Transmit('o');
}


/* Encoder interrupt function*/
ISR(PCINT1_vect)
{
	cli();
	unsigned int i = TCNT1; //Read timer
	filter(i);	//sends timer value to the filter function
	TCNT1 = 0; //Timer = 0
	sei();
}

/*******************************************TRANSMIT AND RECEIVE*************************************************
*****************************************************************************************************************
*****************************************************************************************************************/




/* Receives the 3 strings sent by cygwin and puts the value into our pwm*/
void stringReceiver(){
	char stringReceive[3];
	for(int i = 0; i < 3; i++){
		stringReceive[i] = USART_Receive();
	}
	//integral = 0;	//resets the integral part
	pwmvalue = atoi(stringReceive);
	
}


/*Sends the rpm as 3 chars, one by one*/
void send_rpm(unsigned int value) {
	char rpm[3];
	sprintf(rpm,"%d",value);
	
	for(int i=0;i<3;i++){
		
		USART_Transmit(rpm[i]);
	}
	//USART_Transmit(' ');
}


/* Transmit one character */
void USART_Transmit( unsigned char data )
{
	
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

/*Receive one char*/
unsigned char USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) )
	;
	/* Get and return received data from buffer */
	return UDR0;
}


/*******************************************UPDATING FUNCTIONS***************************************************
*****************************************************************************************************************
*****************************************************************************************************************/

int ADC_input(void){
	// Start ADC conversion
	ADCSRA |= (1 << ADSC);
	// Wait for conversion to complete
	while (ADCSRA & (1 << ADSC));
	int adc_value = ADC; //0-1023
	// Calculate voltage
	//int voltage = (adc_value * VREF) / RESOLUTION;
	return adc_value;
}


/* Turns off all the leds except our power-on LED*/
void all_leds_off( void )
{
PORTB = PORTB & ~(1<<PB1); //turns off the B led
PORTD &= ~((1<<PD2)|(1<<PD3)|(1<<PD4)); //turns off all D leds

}


//updates PWM registers with the value we send in..
int updatePWM(int newrpm)
{
	
newrpm = newrpm + tuner; // changes the rpm value if we turn the potentiometer	
	
int currentrpm = rpm();
int32_t error = (newrpm - currentrpm) * (1<<FIXED_POINT_SCALE);
integral += fixedPointMultiply(error,dt);
int value = (fixedPointMultiply(KP,error) + fixedPointMultiply(KI,integral))  >> FIXED_POINT_SCALE;


if(value > 255){
	value = 255;
}

else if (value < 0){
	value = 0;
}
	
//OCR0A = value;
OCR0B = value;
return value;
}


/*******************************************INITIALIZE FUNCTIONS*************************************************
*****************************************************************************************************************
*****************************************************************************************************************/

/*initializes the control clock*/
void control_clock_init(void){
	TCCR2B = (1 << CS22) | (1<<CS21); //Prescaler 256 and 8 bit counter --> overflow every ~66 ms
	TIMSK2 = (1 << TOIE2);
}







/*initializes the ADC*/
void ADC_init(void){
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //prescaler to 128
	ADCSRA |= (1 << ADEN); //Enables the ADC
	//DDRC &= ~(1<<DDC1); //Set ADC1 as input
	DDRC &= ~(1<<DDC3); //Set ADC3 as input
	//ADMUX |= (1<< REFS0) | (1 << MUX0);  // Select AVCC with external capacitor at AREF pin and ADC1
	ADMUX |= (1<< REFS0) | (1 << MUX0)|(1 << MUX1) ;  // Select AVCC with external capacitor at AREF pin and ADC1
	
}




/*Initializes the LEDS*/
void LED_init(void)
{
	DDRB = DDRB | (1<<PB1) | (1<<PB2);	// PB1, PB2 -> outputs
	DDRD = DDRD | (1<<PD2) | (1<<PD3) | (1<<PD4); //PD2, PD3, PD4 -> outputs
	PORTB = PORTB | (1<<PB2); //AVR-ON LED
}

//initialize PWM
void initPWM(void)
{
	DDRD |= ((1<<DDD5)|(1<<DDD6));	//Set PIND5 and PIND6 as outputs
	TCCR0A |= 0b10110011;			//Configure fast PWM mode, non-inverted output on OCA and inverted output on OCB
	TCCR0B |= 0x01;					//Internal clock selector, no prescaler
	
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

/*Initialize timer*/
void init_Ex1(void)
{
	/* Timer clock = I/O clock / 8 */
	TCCR1B = (1<<CS11);
	/* Clear overflow flag */
	TIFR1 = 1<<TOV1;
	/* Enable Overflow Interrupt */
	//TIMSK1 = 1<<TOIE1;
}/* Initializes the encoder interrupt pins*/
int init_INTs(void)
{
	DDRC &= ~((1<<DDC4)|(1<<DDC5));	// PINC4 - PINC5 set as inputs
	PORTC &= (1<<PC4)|(1<<PC5);
	PCMSK1 |= (1<<PCINT12)|(1<<PCINT13);   // PCINT12 and PCINT13  enabled
	PCICR |= (1<<PCIE1);	// The PC interrupt group 1 (PCINT8 -> PCINT14) enabled
	return 1;
}
