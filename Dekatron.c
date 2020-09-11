/*
 * Dekatron.c
 *
 * Created: 5-10-2014
   Cleaned it up a little bit sep 2020
 *  Author: Charles van den Ouweland
 
 Lfuse: 0xF7. External xtal 20MHz, no divide
 
 This program is for making a clock out of a dekatron, A GS10 (in total 30 cathodes) or GS12 (in total 36 cathodes). 
 These dekatrons have all the (10 or 12) main cathodes connected to external pins, so they are ideal to control which cathode should light up
 Other types of dekatrons have only one (or a few of the) main cathodes exposed on a pin of their own and the rest is a common-cathode pin.
 Next to the main cathodes all dekatrons have two guide cathodes G1 and G2 between each pair of main cathodes. All the G1's are connected and all the G2's are connected.
 
 The main cathodes are connected through a 74141 to PD0..3.
 G1 is connected via MPSA42 transistor to PD7
 G2 is connected via MPSA42 transistor to PD6 (or the other way around, I don't remember)

 The very rare GS12 dekatron has even more cathodes. The 2 extra cathodes are connected via MPSA42 transistors to PD5 and PD4
 
 PC3 and PC4 are connected to a push button to ground. These serve to set the current time after power on. One button serves to move the minutes forward and the other 
 moves the minutes back. The hours follow automatically. The seconds cannot be set
 
 The clock shows a seconds, minutes and hours. The hours are the brightest dot. The minutes are two slightly less bright dots. 
 The seconds is the dot that move around once per second.
 
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define top1 3
#define top2 120
#define NUMSTATES 30 //30 for Dekatron, 36 for Dodekatron GS12
#define MINTIM 40
#define MAXTIM 240
#define SW1 3 //PC3 is SW1
#define SW2 4 //PC4 is SW1
#define KEYDELAY 3

void setDekatronState36(uint8_t state);
void setDekatronState236(uint8_t state1, uint8_t state2);
uint8_t DekatronState36(uint8_t state);
void delay_ms(uint16_t x);
void delay_10us(uint16_t x);
volatile char ds;//deciseconds
volatile char sec;//seconds
volatile char min;//minutes
volatile char hrs;//hours


int main(void){
	DDRC=0b111; //PC0 is /blank, PC1=50Hz,PC2=GND,PC3=SW1, PC4=SW2
	PORTC=0b11000; //pull up PC3 and PC4
	DDRD=0xff;
	
	//timercounter 1: interrupt every 10ms
	TCCR1A=0;
	TCCR1B=(1<<WGM12)|(1<<CS11);
	OCR1A=24999;
	TIMSK1=(1<<OCIE1A);
	sei();
	
	uint8_t state=0;
	int8_t delta;
	unsigned ctr;
	uint8_t ctr1=1;
	uint8_t state1=0;
	uint8_t state2=0;
	
	//jump from 0 to 1 to 0 to 2 etc. This is just a test of the dekatron to verify that everything works
	delta=1;
	ctr1=3;
	ctr=0;
    while(ctr1!=0){
	    state=(state+delta+NUMSTATES)%NUMSTATES;
	    setDekatronState36(state);
	    delay_ms(1+(int)ctr*ctr/150);
	    ctr=(ctr+1)%250;
	    if(ctr==0){
		    delta=-delta;
			ctr1--;
	    }
    }

	// clock
	state=0;
	char keyCountDown=KEYDELAY;
	char fresh=1;
	while(1){
		setDekatronState36(state);
		delay_10us(4);

		char s=sec;
		char m=min;
		char h=hrs;
		
		//indicator at 12h
		if(state==0)delay_10us(50);
		
		//seconds
		state1=s/2;
		state2=((s+1)/2)%NUMSTATES;
		if(state==state1)delay_10us(40);
		if(state==state2)delay_10us(40);

		//minutes
		state1=((m+1)/2)%NUMSTATES;
		state2=(state1+((m%2)?-1:1))%NUMSTATES;
		if(state==state1)delay_10us(400);
		if(state==state2)delay_10us(200);

		//hours
		//state1=(4*(h%12)+5)/10;		
		state1=((5*(h%12)+m/12+1)/2)%NUMSTATES;
		if(state==state1)delay_10us(1000);


		state=(state+1)%NUMSTATES;
		if(state==0){
			if (!(PINC&(1<<SW1))){
				keyCountDown--;
				if (keyCountDown==0){
					keyCountDown=KEYDELAY*(fresh?4:1);
					fresh=0;
					//must disable interrupts here
					cli();
					min++;
					if(min==60){
						min=0;
						hrs++;
						hrs%=24;
					}
					sei();
				}
			}
			else if (!(PINC&(1<<SW2))){
				keyCountDown--;
				if (keyCountDown==0){
					keyCountDown=KEYDELAY*(fresh?4:1);
					fresh=0;
					//must disable interrupts here
					cli();
					if(min==0){
						min=60;
						hrs=(hrs+23)%24; //hrs--;
					}
					min--;
					sei();
				}
			}else{
				keyCountDown=KEYDELAY;
				fresh=1;
			}
		}		
	}
	return 0;
}

//-------------------------------------------------------------------------------
ISR(TIMER1_COMPA_vect){	//is called once every 10ms
	PORTC^=0b10;//to verify that it works
	ds++;
	if(ds==100){
		ds=0;
		sec++;
	}
	if(sec==60){
		sec=0;
		min++;
	}
	if(min==60){
		hrs++;
		hrs%=24;
		min=0;
	}
}

void setDekatronState236(uint8_t state1, uint8_t state2){
	uint8_t s1=DekatronState36(state1);
	uint8_t s2=DekatronState36(state2);
	PORTD=((s1|s2)&0b11110000)|(s1&s2);
}

void setDekatronState36(uint8_t state){
	static uint8_t lastState;
	uint8_t distance=(state-lastState+NUMSTATES)%NUMSTATES;
	if (state%3!=0 && distance>1 && distance<(NUMSTATES-1)){
		//the states that are not divisible by 3 are not directly addressable. First briefly glow the nearest state that is divisible by 3.
		uint8_t intermediate=(state+1)/3*3;
		if((abs(intermediate-lastState)+NUMSTATES)%NUMSTATES==2){
			//exception: distance STATE - LASTSTATE is 3 and intermediate lies between new and previous state, then we must go through both intermediate steps otherwise it won't work
			PORTD=((lastState+intermediate)/2)%NUMSTATES;
			for (uint8_t y=0; y<200; y++) for (uint8_t z=0; z<5; z++) asm volatile ("nop");
		}
		PORTD=DekatronState36(intermediate);
		for (uint8_t y=0; y<200; y++) for (uint8_t z=0; z<10; z++) asm volatile ("nop");
	}
	PORTD=DekatronState36(state);
	lastState=state;
}

uint8_t DekatronState36(uint8_t state){
	//the states 0, 3, 6...27 are BDC-coded on PD3210
	//the states 30 en 33 are on PD4 and PD5
	//the states 3n+1 are on PD6
	//the states 3n+2 are on PD7
	state%=36;
	switch(state%3){
		case 0:{
			return((state==30)?0b10000:(state==33)?0b100000:0)|(state/3);
		}
		case 1:{
			return(0b1001111);
		}
		case 2:{
			return(0b10001111);
		}
	}
	return 0;
}

//General short delays
void delay_ms(uint16_t x){
	uint8_t y, z;
	for ( ; x > 0 ; x--){
		for ( y = 0 ; y < 90 ; y++){
			for ( z = 0 ; z < 6 ; z++){
				asm volatile ("nop");
			}
		}
	}
}

void delay_10us(uint16_t x){
	uint8_t y;
	for ( ; x>0; x--){
		for (y=0; y<49; y++){
				asm volatile ("nop");
		}
	}
}