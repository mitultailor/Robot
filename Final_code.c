 /********************************************************************************
 Platform: SPARK V
 Experiment: Swarm Robotics implementation code 
 Written by: Shailesh Jain,Mitul Tailor,Paras Gosalia
 Last Modification: 30th April 2012
 AVR Studio Version 4.17, Build 666
 size: 10.9 KB

 Concepts covered:  Grid exploration by multi-robot system communication using zigbee wireless module


 Use baud rate as 9600bps.
 
 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega16
 	Frequency: 7372800
 	Optimization: -Os (For more information read section: Selecting proper optimization
	              options below figure 4.22 in the hardware manual)
 *********************************************************************************/

 /********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode


********************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define N 1             		   // Direction  North
#define S 2						   // Direction  South
#define E 3						   // Direction  East
#define W 4 					   // Direction  West

#define		THRESHOLD		25        // i.e on LCD Sensor values are betwn 0 To 25 on white line  
#define		VELOCITY_MAX	100
#define		VELOCITY_MIN	60
#define 	VELOCITY_LOW	0

#define F 0x11
#define B 0x12
#define R 0x13
#define L 0x14
#define M 0x15

#define Servo2port PORTD 			// servo 2 control pin connection 
#define Servo2bit 7
#define S1 OCR2
#define S2 OCR0

unsigned int current_value =0;
volatile unsigned char ShaftCountLeft = 0; //to keep track of left position encoder 
volatile unsigned char ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning
unsigned char s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0; // varables to store sensors value
unsigned char bot_dir = N;
unsigned int get_start=0;
unsigned int dead[]={0,0,0,0};
unsigned int grid[8][8]=
{
};
unsigned int pathindex = 0;
unsigned int check_node[64];
unsigned int path[]={F,F,F,F,F,F,F,L,
							 L,F,F,F,F,F,F,R,
							 R,F,F,F,F,F,F,L,
							 L,F,F,F,F,F,F,R,
							 R,F,F,F,F,F,F,L,
							 L,F,F,F,F,F,F,R,
							 R,F,F,F,F,F,F,L,
							 L,F,F,F,F,F,F,M};
							 
unsigned int path1[]={F,F,F,L,L,F,F,R,R,F,F,L,L,F,F,M};
unsigned cond=0;				
unsigned int toggel = 0;   //used as a variable for buzzer state toggel action
unsigned char value=0,x_cord=0,y_cord=0,length=0,end=0,x_future=0,y_future=0;
unsigned int check_pos =0,to_begin = 0;
unsigned int flag[4]= {0,0,0,0};
unsigned int count =0;
unsigned int k=0,j=0;
unsigned int size = 8;
unsigned int current = 0;
unsigned int counter = 0;
unsigned int bot_id_set =0;
unsigned int cal =0;
unsigned int visited = 0;
 char store[4]={0,0,0,0};
 char tempor[4]={0,0,0,0};
 unsigned int next = 0;
 unsigned int visit_node = 0;
 unsigned int to_process = 0;
 unsigned char rx1=0,rx2=0,rx3=0,rx4=0;
 unsigned int total_turn = 0;
 unsigned int neigh=0;
 unsigned int final_done =0;
 unsigned int block_found =0;

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
unsigned char Front_IR_Sensor;
unsigned char receive_data=0;   // used to save Receiving data
unsigned char ssss=0;

unsigned char bot_id = '0';
unsigned int done[4]={0,0,0,0};

//--------------------****TIMER INTERRUPT**** ----------------------------------------------------

ISR(TIMER0_OVF_vect)
{
  // Timer 0 over flow 
  Servo2port |= (1 << Servo2bit); // set servo 2 control pin
  TCNT0 = 0x6F; // reload timer 0 initial value 
}

ISR(TIMER0_COMP_vect)
{
  // Timer 0 compare match interrupt 
  Servo2port &= ~(1 << Servo2bit); //clear servo 2 control pin 
}

//ISR for right position encoder
ISR(INT0_vect)
{
  ShaftCountRight++; //increment right shaft position count
}

//ISR for left position encoder
ISR(INT1_vect)
{
  ShaftCountLeft++; //increment left shaft position count
}


//----------------------------------****PORT INITIALIZATION****---------------------------------------------------


void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7;    //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80;  // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRA = 0x00;   //set PORTF direction as input
 PORTA = 0x00;  //set PORTF pins floating
}

void motion_pin_config (void)
{
 DDRB = DDRB | 0x0F;   //set direction of the PORTB3 to PORTB0 pins as output
 PORTB = PORTB & 0xF0; // set initial value of the PORTB3 to PORTB0 pins to logic 0
 DDRD = DDRD | 0x30;   //Setting PD4 and PD5 pins as output for PWM generation
 PORTD = PORTD | 0x30; //PD4 and PD5 pins are for velocity control using PWM
}

void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as outpt
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//Function to configure INT1 (PORTD 3) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRD  = DDRD & 0xF7;  //Set the direction of the PORTD 3 pin as input
 PORTD = PORTD | 0x08; //Enable internal pull-up for PORTD 3 pin
}

//Function to configure INT0 (PORTD 2) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRD  = DDRD & 0xFB;  //Set the direction of the PORTD 2 pin as input
 PORTD = PORTD | 0x04; //Enable internal pull-up for PORTD 2 pin
}

void port_init()
{
 lcd_port_config();
 adc_pin_config();		
 motion_pin_config();
 buzzer_pin_config();
  left_encoder_pin_config();    //left encoder pin config
 right_encoder_pin_config();   //right encoder pin config	
 
}

//-----------------------------------------------------------------------****MOTION CONTROL****------------------------------------------------------------


//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortBRestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortBRestore = PORTB; 			// reading the PORTB's original status
 PortBRestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortBRestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTB status
 PORTB = PortBRestore; 			// setting the command to the port
}

void forward (void) //both wheels forward
{
  motion_set(0x06);
}

void stop (void) //hard stop
{
  motion_set(0x00);
}

void back(void)
{
  motion_set(0x09);
}

//to move bot differential right

void right(void)
{
  motion_set(0x0A);
}

//to move bot differential left

void left(void)
{
  motion_set(0x05);
}

void velocity (unsigned char left_motor, unsigned char right_motor)
{
 OCR1AH = 0x00;
 OCR1AL = left_motor; 
 OCR1BH = 0x00;
 OCR1BL = right_motor;
}

//--------------------------------------****ENCODER FUNCTION****-----------------------------------------------------------


void linear_distance_mm(unsigned int DistanceInMM)
{
  float ReqdShaftCount = 0;
  unsigned char ReqdShaftCountInt = 0;

  ReqdShaftCount = DistanceInMM / 12.92; // division by resolution to get shaft count
  ReqdShaftCountInt = (unsigned char)ReqdShaftCount;

  ShaftCountRight = 0;
  while (1)
  {
    if (ShaftCountRight > ReqdShaftCountInt)
    {
      break;
    }
  }
// stop(); //Stop robot
}

void angle_rotate(unsigned int Degrees)
{
  float ReqdShaftCount = 0;
  unsigned char ReqdShaftCountInt = 0;

  ReqdShaftCount = (float)Degrees / 12.85; // division by resolution to get shaft count 
  ReqdShaftCountInt = (unsigned char)ReqdShaftCount;
  ShaftCountRight = 0;
  ShaftCountLeft = 0;

  while (1)
  {
    if ((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
      break;
  }
 //stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
  forward(); 
  linear_distance_mm(DistanceInMM);
}

//function below make the bot to move backward through fixed distance

void back_mm(unsigned int DistanceInMM)
{
  back();
  linear_distance_mm(DistanceInMM);
}

//function used below rotates the bot as per the angle send

void left_degrees(unsigned int Degrees)
{
  // 28 pulses for 360 degrees rotation 12.92 degrees per count
  left(); //Turn left
  angle_rotate(Degrees);
}

//function used below rotates the bot as per the angle send

void right_degrees(unsigned int Degrees)
{
  // 28 pulses for 360 degrees rotation 12.92 degrees per count
  right(); //Turn right
  angle_rotate(Degrees);
}

//------------------------------------------------------****UART INITIALIZATION****-----------------------------------------------------

void uart0_init(void)
{
 UCSRB = 0x00; //disable while setting baud rate
 UCSRA = 0x00;
 UCSRC = 0x86;
 UBRRL = 0x2F; //set baud rate lo  //67 is for 16MHz 9600 baudrate
 UBRRH = 0x00; //set baud rate hi
 UCSRB = 0x98; 
}

void USART_Transmit( unsigned char data )
{
/* Wait for empty transmit buffer */
while ( !( UCSRA & (1<<UDRE)) )
{
}
/* Put data into buffer, sends the data */
UDR = data;
}

//---------------------------------------------------****ADC CONVERSION AND LCD FUNCTION****----------------------------------

//This Function accepts the Channel Number and returns the corresponding Analog Value 
unsigned char ADC_Conversion(unsigned char Ch)
{
 unsigned char a;
 Ch = Ch & 0x07;  			
 ADMUX= 0x20| Ch;	   		
 ADCSRA = ADCSRA | 0x40;	//Set start conversion bit
 while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
 a=ADCH;
 ADCSRA = ADCSRA|0x10;      //clear ADIF (ADC Interrupt Flag) by writing 1 to it
 return a;
}

// This Function prints the Analog Value Of Corresponding Channel No. at required Row and Coloumn Location. 
void print_sensor(char row, char coloumn,unsigned char channel)
{
 ADC_Value = ADC_Conversion(channel);
 lcd_print(row, coloumn, ADC_Value, 3);
}

//Function to Initialize ADC
void adc_init()
{
 ADCSRA = 0x00;
 ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
 ACSR = 0x80;
 ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

void left_position_encoder_interrupt_init (void) //Interrupt 1 enable
{
 cli(); //Clears the global interrupt
 MCUCR = MCUCR | 0x08; // INT1 is set to trigger with falling edge
 GICR = GICR | 0x80;   // Enable Interrupt INT1 for left position encoder
 sei(); // Enables the global interrupt 
}

void right_position_encoder_interrupt_init (void) //Interrupt 0 enable
{
 cli(); //Clears the global interrupt
 MCUCR = MCUCR | 0x02; // INT0 is set to trigger with falling edge
 GICR = GICR | 0x40;   // Enable Interrupt INT5 for right position encoder
 sei(); // Enables the global interrupt 
}

//-------------------------------------------------------****TIMER INIT****------------------------------------------------
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xAA; //setup
 TCNT1L = 0x00;
 TCCR1A = 0x00;
 TCCR1B = 0x0D; //start 
}

void timer2_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFF; //setup
 TCNT1L = 0x01;
 OCR1AH = 0x00;
 OCR1AL = 0xFF;
 OCR1BH = 0x00;
 OCR1BL = 0xFF;
 ICR1H  = 0x00;
 ICR1L  = 0xFF;
 TCCR1A = 0xA1;
 TCCR1B = 0x0D; //start Timer
}
void timer0_init(void)
{
  TCCR0 |= (1 << CS02) | (1 << CS00); // Timer 0 prescaler 1024 , normal mode 
  TCNT0 = 0x6F; // Timer 0 initial vlaue 
 // TIMSK |= (1 << OCIE0) | (1 << TOIE0); // timer 0 compare match and overflow interrupt enable 
 }
 
//-------------------------------------------------------****BUZZER FUNCTION****------------------------------------------------

void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

//-------------------------------------------------------****INIT DEVICES ****------------------------------------------------

void init_devices (void)
{
 DDRA=0x00;
DDRB&=~_BV(4);
DDRD&=~_BV(6);

PORTA=0xFF;
PORTB|=_BV(4);
PORTD|=_BV(6);
 cli();    //Clears the global interrupts
  DDRA |= (1 << PA7); // making servo contol pins as output
  DDRD |= (1 << PD7);
 uart0_init();
 port_init();
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 timer1_init();
// timer2_init();
  TIMSK = 0x04;   //timer1 interrupt sources
 adc_init();
 sei();          //Enables the global interrupts
}


void init_devices2 (void)
{
 DDRA=0x00;
DDRB&=~_BV(4);
DDRD&=~_BV(6);

PORTA=0xFF;
PORTB|=_BV(4);
PORTD|=_BV(6);
 cli();    //Clears the global interrupts
  DDRA |= (1 << PA7); // making servo contol pins as output
  DDRD |= (1 << PD7);
 uart0_init();
 port_init();
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 timer2_init();
 adc_init();
 sei();          //Enables the global interrupts
}

//-------------------------------------------------------****SENSOR VALUE READ FUNCTION****------------------------------------------------

void in_sens_val()
{
  if (bit_is_set(PINA, 0))
    s5 = 1;
  else
    s5 = 0;

  if (bit_is_set(PINA, 1))
    s4 = 1;
  else
    s4 = 0;

  if (bit_is_set(PINA, 2))
    s3 = 1;
  else
    s3 = 0;

  if (bit_is_set(PIND, 6))
    s2 = 1;
  else
    s2 = 0;

  if (bit_is_set(PINB, 4))
    s1 = 1;
  else
    s1 = 0;


}

//-------------------------------------------------------****LINE FOLLOWER FUNCTION****------------------------------------------------
void follow()
{
	
				  if((bit_is_set(PINA, 1))  && (bit_is_set(PIND, 6)))	// if middle 3 on line go straight
				  {
				    velocity(VELOCITY_MAX,VELOCITY_MAX);
					lcd_print (2,1,VELOCITY_MAX,3);
 					lcd_print (2,5,VELOCITY_MAX,3);
				  }
				  else
				  if((bit_is_clear(PINA, 1))  && (bit_is_set(PIND, 6)))  
				  {
				   velocity(VELOCITY_MIN,VELOCITY_MAX);
				    lcd_print (2,1,VELOCITY_MIN,3);
 					lcd_print (2,5,VELOCITY_MAX,3);
				  }
				  else
	  
				  if((bit_is_set(PINA, 1))  && (bit_is_clear(PIND, 6)))
				  {
				   
				   velocity(VELOCITY_MAX,VELOCITY_MIN);
				  lcd_print (2,1,VELOCITY_MAX,3);
 					lcd_print (2,5,VELOCITY_MIN,3);
				  }
		
}

//-------------------------------------------------------****NODE AND ITS PROPERTIES****------------------------------------------------

int isPlus()
{
	if(bit_is_set(PINA,0) && bit_is_set(PINB,4))
		return 1;
	else
		return 0;
}

int isEnd()
{
	if(bit_is_clear(PINA,1) && bit_is_clear(PIND,6)  && bit_is_clear(PINA,2) && bit_is_clear(PINA,0) && bit_is_clear(PINB,4))
		return 1;
	else
		return 0;
}
//-------------------------------------------------------****OBSTACLE DETECTION FUNCTION****------------------------------------------------

int isObstacle()
{
 if(bit_is_clear(PINA,7))
		return 1;
	else
		return 0;
}


int isBlock1()
{
	if(bit_is_clear(PINA,1) && bit_is_clear(PIND,6)  && bit_is_clear(PINA,2) && bit_is_set(PINA,0) && bit_is_set(PINB,4))
		return 1;
	else
		return 0;
}
int isBlock2()
{
	if((bit_is_clear(PINA,1) && bit_is_set(PIND,6)  && bit_is_set(PINA,0) && bit_is_set(PINB,4)) ||
	(bit_is_set(PINA,1) && bit_is_clear(PIND,6)  && bit_is_set(PINA,0) && bit_is_set(PINB,4)) )
		return 1;
	else
		return 0;
}

//-------------------------------------------------------****TIMER OVERFLOW INTERRUPT****------------------------------------------------
//This ISR can be used to toggle the buzzer input
ISR(TIMER1_OVF_vect)
{
 if (toggel == 0)
 {
	USART_Transmit('$');
	USART_Transmit('s');
	if(counter == 0)
	{
	bot_id = 'A';
	bot_id_set = 1;
	toggel = 1;
	}
	if(counter == 1 && bot_id_set == 0)
	{
	bot_id = 'C';
	bot_id_set =1;
	toggel =1;
	}
	if(counter == 2 && bot_id_set == 0)
	{
	bot_id = 'B';
	bot_id_set =1;
	toggel = 1;
	}
	if(counter == 3 && bot_id_set == 0)
	{
	bot_id='D';
	bot_id_set = 1;
	toggel = 1;
	}
	
	return;
 }
 if(check_pos == 1)
 {
	to_begin = 1 ;
	return;
}

}

//-------------------------------------------------------****GRIPPING FUNCTION****------------------------------------------------
void pickup(void)
{
  S2 = 0x78;
  _delay_ms(500);
 
}

void drop_down(void)
{
 S2 = 0x73;
 _delay_ms(500);

}

//-------------------------------------------------------****SHARP 90 TURN FUNCTION****------------------------------------------------
void turnLeft_done(void)
{
 forward_mm(60);
  left();
 _delay_ms(350);
 in_sens_val();

 while(((bit_is_clear(PINA, 1)) && (bit_is_clear(PIND, 6))))
 {
  in_sens_val();
    left();
 }
   PORTB = 0x06;	
}

void turnRight_done()
{
 forward_mm(60);
  right();
 _delay_ms(350);
 in_sens_val();

 while(((bit_is_clear(PINA, 1)) && (bit_is_clear(PIND, 6))))
 {
  in_sens_val();
    right();
 }
 PORTB= 0x06;

}
void turnRight()
{
 forward_mm(30);
  right();
 _delay_ms(350);
 in_sens_val();

 while(((bit_is_clear(PINA, 1)) && (bit_is_clear(PIND, 6))))
 {
  in_sens_val();
    right();
 }
  PORTB = 0x06;	
}
void turnRight1()
{
  forward_mm(70);
  right();
 _delay_ms(350);
 in_sens_val();

 while(((bit_is_clear(PINA, 1)) && (bit_is_clear(PIND, 6))))
 {
  in_sens_val();
    right();
 }
PORTB = 0x06;	
}

void turnLeft(void)
{
 forward_mm(30);
  left();
 _delay_ms(350);
 in_sens_val();

 while(((bit_is_clear(PINA, 1)) && (bit_is_clear(PIND, 6))))
 {
  in_sens_val();
    left();
 }
  PORTB = 0x06;
}

void turnLeft1(void)
{
  left();
 _delay_ms(350);
 in_sens_val();

 while(((bit_is_clear(PINA, 1)) && (bit_is_clear(PIND, 6))))
 {
  in_sens_val();
    left();
 }
 PORTB = 0x06;
}


void turnBack()
{
  right();
 _delay_ms(700);
 in_sens_val();
 while(((bit_is_clear(PINA, 1)) && (bit_is_clear(PIND, 6))) )
 {
  in_sens_val();
    right();
 }
 PORTB = 0x06;

}

//-------------------------------------------------------****CHANGE DIRECTION FUNCTION****------------------------------------------------

void change_direction_to(char dir)
{
  if (bot_dir == N)
  {
    switch (dir)
    {
      case N:
        break;
      case S:
        {
          turnBack();
          bot_dir = S;
          break;
        }
      case E:
        {
          turnRight();
          bot_dir = E;
          break;
        }
      case W:
        {
          turnLeft();
          bot_dir = W;
          break;
        }
    }
  }
  else if (bot_dir == S)
  {
    switch (dir)
    {
      case N:
        {
          turnBack();
          bot_dir = N;
          break;
        }
      case S:
        {
          break;
        }
      case E:
        {
          turnLeft();
          bot_dir = E;
          break;
        }
      case W:
        {
          turnRight();
          bot_dir = W;
          break;
        }
    }
  }
 else  if (bot_dir == W)
  {
    switch (dir)
    {
      case W:
        break;
      case E:
        {
          turnBack();
          bot_dir = E;
          break;
        }
      case N:
        {
          turnRight();
          bot_dir = N;
          break;
        }
      case S:
        {
          turnLeft();
          bot_dir = S;
          break;
        }
    }
  }
 else if (bot_dir == E)
  {
    switch (dir)
    {
      case E:
        break;
      case W:
        {
          turnBack();
          bot_dir = W;
          break;
        }
      case S:
        {
          turnRight();
          bot_dir = S;
          break;
        }
      case N:
        {
          turnLeft();
          bot_dir = N;
          break;
        }
    }
  }

}
void change_dirn(unsigned char dir)
{
switch(dir)
{
case N: bot_dir = W; break;		
case E: bot_dir = N; break;
case W:bot_dir = S; break;
case S:bot_dir = E; break;
}
}

//-------------------------------------------------------****BOT AVAILABILITY FUNCTION****------------------------------------------------
void alive(unsigned char check_bit)
{
	switch(check_bit)
	{
		case 'A' : flag[0] = 1;
					break;
		case 'B' : flag[1] = 1;
					break;
		case 'C' :	flag[2] = 1;
					break;
		case 'D' :  flag[3] = 1;
					break;
	}
}
void update_node()
{
  if(to_process ==1)
		{   
		    lcd_cursor(2,13);
		    lcd_string(store);
			
			visited =atoi(store);
			
			check_node[visited]=1;
			to_process=0;
			buzzer_off();
			for(k=0;k<4;k++)
			 store[k]=0;
			forward();
			return;
		}
}

//-------------------------------------------------------****COMMUNICATION INTERRUPT****------------------------------------------------
SIGNAL(SIG_UART_RECV) 
{
	rx1= rx2;
	rx2= UDR;
  
 if(toggel == 0)
 {
	
	if(rx1 == '$' && bot_id_set == 0)
	{
	counter++;
	return;
	}

 }
 if(to_begin == 1)
 {
	if(rx1 == '*')
	{
		switch(rx2)
		{
		case 'A':	flag[0]++;break;
		case 'B':	flag[1]++;break;
		case 'C': 	flag[2]++;break;
		case 'D':	flag[3]++;break;
		}
	}
	else if(rx1 == '@')
	{
	  get_start=1;
	  
	   switch(rx2)
	   {
	   case 'q':  cond=2;
					break;
						
	   case 'w': 	cond=1;
					path[31]=M;
					break;
	   case 'e':  	 switch(bot_dir)
					 {
					 case N : cond = 1;break;
					 case S : cond =2;path[31]=M;break;
					 case E : break;
					 case W : cond = 1 ;break;
					 }
					break;
	   case 'r':  cond=1;
					break;
	   }
	}
	else if(rx1 == '%')
	{
		switch(rx2)
		{
		case 'A': done[0]=1;break;
		case 'B': done[1]=1;break;
		case 'C': done[2]=1;break;
		case 'D': done[3]=1;break;
					
		}
	}
 }
 
return; // Echo the received data plus 1
}

//-------------------------------------------------------****SET INITIAL DIRECTION****------------------------------------------------
void initial_direction(unsigned char dir)
{
	switch(dir)
	{
	case 'A':	change_direction_to(W);
				break;
				
	case 'B':	follow();
				break;
				
	case 'C':  change_direction_to(E);
				break;
				
	case 'D':  change_direction_to(S);
				break;
	 }
 }
 
 //-------------------------------------------------------****DISPERSION FUNCTION****------------------------------------------------
 
 void dispersion()
 {
  to_begin=1;
 lcd_cursor(1,1);
  lcd_string("DISPE");
  velocity(VELOCITY_MAX,VELOCITY_MAX);
		forward();
		while(1)
		{
		follow();
		lcd_print(2,16,bot_dir,1);
			if(isPlus())
			{
					follow();
					while(isPlus());
					if(value == 0 && end == 0)		
					initial_direction(bot_id);	
					
					if(value == 0 && end == 1)
					{
						turnRight();
						change_dirn(bot_dir);
					}
				value++;
						
			}
			if(isEnd())
			{
				USART_Transmit('*');
				USART_Transmit(bot_id);
				switch(bot_id)
				{
				case 'A':	flag[0]++;break;
				case 'B':	flag[1]++;break;
				case 'C': 	flag[2]++;break;
				case 'D':	flag[3]++;break;
				}
				
				if(end==0)
				{
					turnBack();
					value = 0;
				
				}
				if(end==1)
				{  
					turnBack();
					value = 0;
				}
				if(end ==2)
				{
				length =value-1;
				lcd_print(1,7,length,1);
				turnBack();
				stop();
				break;
				}
				end++;
			  
			}
		}
}

//-------------------------------------------------------****LOCALIZATION AND MAPPING FUNCTION****------------------------------------------------
void localize()
{
lcd_cursor(1,1);
  lcd_string("MAPPY");
  length =7;
	check_pos =1;
  switch(bot_dir)
   {
    case E: x_cord = 0;
			y_cord = 0;
			break;
	case N: x_cord = length;
			y_cord = 0;
			break;
	case W : x_cord = length;
			 y_cord = length;
			 break;
	case S : x_cord =0;
			 y_cord = length;
			 break;
	}

	return;
}	

//-------------------------------------------------------****INFORMATION SHARING FUNCTION****------------------------------------------------


void information_sharing()
{
 lcd_cursor(1,1);
  lcd_string("SHARE");
if(check_pos == 1 )
{
	int set = (int)bot_id -65; 
	flag[set] = 1;
} 
check_pos =0;
return;
}

//-------------------------------------------------------****INITIAL GRID COORDINATE FUNCTION****------------------------------------------------
void init_grid()
{
	for(k=0;k<size;k++)
	{
		for(j=0;j<size;j++)
		{
		grid[k][j] = (j*8)+ k;
		}
	}
}

//------------------------------------------------------------****PROCESS NODE FUNCTION****------------------------------------------------
void check_next_node()
{
	
	current = grid[x_cord][y_cord];
	
	switch(bot_dir)
	{
	case N : if(current == 63)
			{
				change_direction_to(W);
				next = grid[x_cord-1][y_cord];
			}
			else
			{
			 next = grid[x_cord][y_cord+1];
			}
			break;
			
	case E : if(current == 7)
			{
				change_direction_to(N);
				next = grid[x_cord][y_cord+1];
			}
			else
			{
			next = grid[x_cord+1][y_cord];	
			}
			break;
	case W :  if(current == 56)
			{
				change_direction_to(S);
				next = grid[x_cord][y_cord-1];
			}
			else
			{
			next = grid[x_cord-1][y_cord];
			}
			break;
	case S :  if(current == 0)
			{
				change_direction_to(E);
				next = grid[x_cord+1][y_cord];
			}
			else
			{
			next = grid[x_cord][y_cord-1]; 
			}
			break;
	}
	x_future = next % 8;
	y_future = next / 8;	
}

//-------------------------------------------------------****CHECK TASK COMPLETION FUNCTION ****------------------------------------------------

void check_task_completion()
{
				USART_Transmit('%');
				USART_Transmit(bot_id);
		
				switch(bot_id)
				{
				case 'A': done[0]=1;break;
				case 'B': done[1]=1;break;
				case 'C': done[2]=1;break;
				case 'D': done[3]=1;break;
				}
				stop();
				for(k=0;k<1000;k++)
				{
					lcd_print(1,9,k,4);
					if(neigh == 0)
					{
						switch(bot_id)
						{
						case 'A': if(done[1]==1)
									neigh=1;
									break;
						case 'B':  if(done[2]==1)
									neigh=1;
									break;
						case 'C': if(done[3]==1)
									neigh=1;
									break;
						case 'D':  if(done[0]==1)
									neigh=1;
									break;
						}
					}
					else 
					{
						break;
					}
				}

}
void block_detect()
{
			stop();	
			ssss =SREG;
			cli();
			timer0_init();
			TIMSK |= (1 << OCIE0) | (1 << TOIE0); // timer 0 compare match and overflow interrupt enable 
			sei();
			SREG = ssss;
			drop_down();
			_delay_ms(2000);
			pickup();
			//drop_down();
			
			ssss =SREG;
			cli();
			timer2_init();
			sei();
			ssss = SREG;
			forward();	
}

//-------------------------------------------------------****ALIGN ROBOT POSITION AND MOTION CONTROL FUNCTION ****------------------------------------------------

void orient(int value)
{
	switch(value)
	{

		case F:
			follow();
			while(isPlus());

				break;

		case L:
				turnLeft_done();
			if(bot_dir == N)
			{
				bot_dir = W;
			}
			else if (bot_dir == E)
			{
				bot_dir = N;
			}
			else if (bot_dir == W)
			{
				bot_dir=S;
			}
			else if (bot_dir == S)
			{
				bot_dir = E;
			}
				break;
		case R:
			turnRight_done();
		if(bot_dir == N)
			{
				bot_dir = E;
			}
			else if (bot_dir == E)
			{
				bot_dir =S;
			}
			else if (bot_dir == W)
			{
				bot_dir = N;
			}
			else if (bot_dir == S)
			{
				bot_dir = W;
			}
			break;
				
			case M: 
			
				check_task_completion();
				if(neigh ==1 || final_done ==1)
				{
					stop();
					_delay_ms(200000);
				}	
				else 
				{
					final_done=1;
					forward();
					turnRight1();
					path1[0]=R;
					pathindex=0;
					while(1)
					{
					follow();
						if(isPlus())
						{
						current_value = path1[pathindex++];
						orient(current_value);
						}
					}
				}
				break;
	}

}


//-------------------------------------------------------****TASK DISTRIBUTION FUNCTION****------------------------------------------------

void region_division()
{
for(k=0;k<300;k++)
{
	lcd_print(1,9,k,4);
	if(flag[0]>1 && flag[1]>1 && flag[2]>1 && flag[3]>1)
	{
	 USART_Transmit('@');
	 USART_Transmit('r');
	 get_start=1;
	 cond=1;
	 break;
	}
	else if(get_start==1)
	{
	  break;
	 }
}
if(get_start==0)
{
for(k=0;k<100;k++)
{
	lcd_print(1,9,k,4);
	if(flag[0]>1 && flag[2]>1 && flag[1]>1) 
	{
	 USART_Transmit('@');
	 USART_Transmit('e');
	 get_start=1;
	 switch(bot_dir)
	 {
	 case N : cond = 1;break;
	 case S : cond =2;path[31]=M;break;
	 case E : break;
	 case W : cond = 1 ;break;
	 }
	 break;
	}
	else if(get_start==1)
	{
	 break;
	 }
}
}
if(get_start==0)
{
for(k=0;k<100;k++)
{
	lcd_print(1,9,k,4);
	if((flag[0]>1 && flag[2]>1) )
	{
	 USART_Transmit('@');
	 USART_Transmit('w');
	 get_start=1;
	 cond=2;
	 path[31]=M;
	 break;
	}
	else if(get_start==1)
	{
	 break;
	 }
}
}
if(get_start==0)
{
  if(flag[0]> 1 || flag[1]> 1 || flag[2]>1 || flag[3]>1)
  {
     USART_Transmit('@');
	  USART_Transmit('q');
		cond =2;
		get_start=1;
  }
}
if(get_start==1)
{			
	forward();
	while(1)
	{
		follow();
		if(isPlus())
		{
			if(block_found == 1)
			{
				block_detect();
				block_found =0;
			}
			if(isBlock1())
			{	
				block_found = 1;
			}
			if(cond == 1)
			{
			current_value = path1[pathindex++];
			orient(current_value);
			}
			else if(cond==2)
			{
			current_value = path[pathindex++];
			orient(current_value);
			}
		}
	}
  }
}
//-------------------------------------------------------****MAIN FUNCTION ****------------------------------------------------
int main()
{
	
	init_devices();
	lcd_set_4bit();
	lcd_init();
    lcd_cursor(1,1);
	lcd_string("START");
	_delay_ms(rand());
//--*** GRIPPER INITAL POSITION ***---
	S2 = 0x73;
	cli();
	timer0_init();
	TIMSK |= (1 << OCIE0) | (1 << TOIE0); // timer 0 compare match and overflow interrupt enable 
	sei();
	pickup();
	
	while(1)
	{
	if( bot_id_set == 1)
	{	
	
		init_devices2();
		init_grid();
		dispersion();
		localize();
		region_division();
		break;
	}
	
 }
 }

