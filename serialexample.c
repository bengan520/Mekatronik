// Compile with gcc -Wall -std=c99 -o output.exe avr_com serial*

#include "serialport.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define DEBUG 1


int sp;
char cin[4];
char cout[4];
void delay(int milliseconds);




void delay(int milliseconds)
{
	long pause;
	clock_t now,then;

	pause = milliseconds*(CLOCKS_PER_SEC/1000);
	now = then = clock();
	while( (now-then) < pause )
	now = clock();
}



int main(void)
{
	/*Declaration of variables*/
	
	/*Initialise serial port */
	sp = serial_init("/dev/ttyS0",0);
	if(sp == 0)
	{
		printf("Error! Serial port could not be opened.\n");
	}
	else
	{
		printf("Serial port open with identifier %d \n",sp);
		printf("\n");
		//printf("Hello, input the requested rpm: ");
	}
	
	
	
	while (1) {
		
		printf("Enter ""r"" to read the current rpm or ""w"" to write a new or ""a"" to see tuner");
		printf("\n");
		scanf("%s",cout);
		
		
		
		if(cout[0]=='r'){
			
			write(sp,cout,1);
			//read(sp,&cin,3);
			read(sp,&cin[0],1);
			read(sp,&cin[1],1);
			read(sp,&cin[2],1);
			printf("rpm = ");
			printf(cin);
			printf("\n");
		}
		
		if(cout[0]=='a'){
			
			write(sp,cout,1);
			//read(sp,&cin,3);
			read(sp,&cin[0],1);
			read(sp,&cin[1],1);
			read(sp,&cin[2],1);
			printf("tuner = ");
			printf(cin);
			printf("\n");
		}
		
		if(cout[0]=='w'){
			
			write(sp,cout,1);
			printf("Write rpm: ");
			scanf("%s",cout);
			
			if (4 < atoi(cout) && atoi(cout) < 121){
				write(sp,cout,3);
				printf("New rpm = %d",atoi(cout));
				printf("\n");
				
				
			}
			else if (atoi(cout) < 5){
				printf("Lowest allowed RPM is 5");
				printf("\n");
			}
			else if (atoi(cout) > 120){
				printf("Highest allowed RPM is 120");
				printf("\n");
			}
			else{
				printf("Input not allowed");
				printf("\n");
			}
		}
		
		

	}
	
	/*Close the serial port */
	serial_cleanup(sp);
	return 1;
}