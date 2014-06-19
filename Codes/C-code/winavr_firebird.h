/** \file winavr_firebird.h
 
 *******************************************************************************
Written by: 
Group 5:

HARISH 8005052
SAMEER 8005056
RAJESH 8005041
PRADEEP 8005044

 AVR Studio Version 4.17, Build 666

 Application example: Robot control over serial port

 Concepts covered:  serial communication

 Serial Port used: UART1

 There are two components to the motion control:
 1. Direction control using pins PORTA0 to PORTA3
 2. Velocity control by PWM on pins PL3 and PL4 using OC5A and OC5B.

 In this experiment for the simplicity PL3 and PL4 are kept at logic 1.
 
 Pins for PWM are kept at logic 1.
 
 Connection Details:    
                       
  Motion control:        L-1---->PA0;        L-2---->PA1;
                           R-1---->PA2;        R-2---->PA3;
                           PL3 (OC5A) ----> Logic 1;     PL4 (OC5B) ----> Logic 1;


  Serial Communication:    PORTD 2 --> RXD1 UART1 receive for RS232 serial communication
                        PORTD 3 --> TXD1 UART1 transmit for RS232 serial communication

                        PORTH 0 --> RXD2 UART 2 receive for USB - RS232 communication
                        PORTH 1 --> TXD2 UART 2 transmit for USB - RS232 communication

                        PORTE 0 --> RXD0 UART0 receive for ZigBee wireless communication
                        PORTE 1 --> TXD0 UART0 transmit for ZigBee wireless communication

                        PORTJ 0 --> RXD3 UART3 receive available on microcontroller expainsion board
                        PORTJ 1 --> TXD3 UART3 transmit available on microcontroller expainsion board

Serial communication baud rate: 9600bps

 This experiment enables the user to control the robot motion through Serial
 Comunication from the PC Wirelessly. 

 Byte Commands for respective direction are as Follows:

 0x51 -----> FORWARD
 0x52 -----> BACKWARD
 0x53 -----> LEFT
 0x54 -----> Right
 0x50 -----> Stop

 Note:
 
 1. Make sure that in the configuration options following settings are
     done for proper operation of the code

     Microcontroller: atmega2560
     Frequency: 11059200
     Optimization: -O0 (For more information read section: Selecting proper optimization options
                        below figure 4.22 in the hardware manual)

 2. Difference between the codes for RS232 serial, USB and wireless communication is only in the serial port number.
     Rest of the things are the same.

 3. For USB communication check the Jumper 1 position on the ATMEGA2560 microcontroller adaptor board

*********************************************************************************

** *******************************************************************************

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

******************************************************************************/
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#define FCPU 11059200ul ///defined here to make sure that program works properly


unsigned long int ShaftCountLeft = 0; ///< to keep track of left position encoder 
unsigned long int ShaftCountRight = 0; ///< to keep track of right position encoder
unsigned int Degrees; ///< to accept angle in degrees for turning
unsigned char data; ///< to receive data through serial communication 

/** Function To Initialize Ports */
void motion_pin_config()
{
 DDRA = DDRA | 0xCF;   ///Motion control pins set as output
 PORTA = PORTA & 0x30; ///Inital value of the motion control pins set to 0
 DDRL = DDRL | 0x18;   ///Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; ///Setting PL3 and PL4 pins as logic 1
}


///Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  ///Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; ///Enable internal pullup for PORTE 4 pin
}

///Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  ///Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; ///Enable internal pullup for PORTE 4 pin
}

///Interrupt 4 enable
void left_position_encoder_interrupt_init (void) 
{
 cli(); ///Clears the global interrupt
 EICRB = EICRB | 0x02; /// INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; /// Enable Interrupt INT4 for left position encoder
 sei();   /// Enables the global interrupt 
}

//Interrupt 5 enable
void right_position_encoder_interrupt_init (void) 
{
 cli(); ///Clears the global interrupt
 EICRB = EICRB | 0x08; /// INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; /// Enable Interrupt INT5 for right position encoder
 sei();   /// Enables the global interrupt 
}

///Function to Initialize PORTS
void port_init()
{
 motion_pin_config();
 left_encoder_pin_config();
 right_encoder_pin_config();
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
}


///ISR for right position encoder
ISR(INT5_vect)  
{
 ShaftCountRight++;  ///increment right shaft position count
}


///TSR for left position encoder
ISR(INT4_vect)
{
 ShaftCountLeft++;  ///increment left shaft position count
}


///Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		/// removing upper nibbel for the protection
 PortARestore = PORTA; 		/// reading the PORTA original status
 PortARestore &= 0xF0; 		/// making lower direction nibbel to 0
 PortARestore |= Direction; /// adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		/// executing the command
}

///both wheels forward
void forward (void) 
{
  motion_set (0x06);
}

///both wheels backward
void back (void) 
{
  motion_set(0x09);
}

///Left wheel backward, Right wheel forward
void left (void) 
{
  motion_set(0x05);
}

///Left wheel forward, Right wheel backward
void right (void) 
{
  motion_set(0x0A);
}


void stop (void)
{
  motion_set (0x00);
}


/// Timer 5 initialised in PWM mode for velocity control
/// Prescale:64
/// PWM 8bit fast, TOP=0x00FF
/// Timer Frequency:674.988Hz
void timer5_init()
{
	TCCR5B = 0x00;	///Stop
	TCNT5H = 0xFF;	///Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	///Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	///Output compare register high value for Left Motor
	OCR5AL = 0xFF;	///Output compare register low value for Left Motor
	OCR5BH = 0x00;	///Output compare register high value for Right Motor
	OCR5BL = 0xFF;	///Output compare register low value for Right Motor
	OCR5CH = 0x00;	///Output compare register high value for Motor C1
	OCR5CL = 0xFF;	///Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/**{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionalit to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	///WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}


///Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}


///Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 4.090; /// division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0; 
 ShaftCountLeft = 0; 

 while (1)
 {
  if((ShaftCountRight > ReqdShaftCountInt) || (ShaftCountLeft > ReqdShaftCountInt))
  {
  ShaftCountRight = 0;
  ShaftCountLeft = 0;
  break;
  }
 }
 }


///Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 5.338; /// division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 ShaftCountRight = 0;
 
 while(1)
 {
  if(ShaftCountRight > ReqdShaftCountInt || ShaftCountLeft > ReqdShaftCountInt )
  {
  ShaftCountRight = 0;
  ShaftCountLeft = 0;
  break;
  }
 } 
 stop(); //Stop action
}

///move forward by specified distance
void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
}

///move backward by specified distance
void back_mm(unsigned int DistanceInMM)
{
 back();;
 linear_distance_mm(DistanceInMM);
 
}

///rotate left by specified degrees
void left_degrees(unsigned int Degrees) 
{
/// 88 pulses for 360 degrees rotation 4.090 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}


///rotate right by specified degrees
void right_degrees(unsigned int Degrees)
{
/// 88 pulses for 360 degrees rotation 4.090 degrees per count
 right(); //Turn right
 angle_rotate(Degrees);
}

///Function to configure the buzzer
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		///Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;		///Setting PORTC 3 logic low to turnoff buzzer
}

///Function to switch the buzzer on 
void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

///Function to switch the buzzer off 
void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

/** UART1 initialization */
/// desired baud rate:9600
/// actual baud rate:9600 (0.0%)
/// char size: 8 bit
/// parity: Disabled
void uart1_init(void)
{
 UCSR1B = 0x00; ///disable while setting baud rate
 UCSR1A = 0x00;
 UCSR1C = 0x06;
 UBRR1L = 0x47; ///set baud rate lo
 UBRR1H = 0x00; ///set baud rate hi
 UCSR1B = 0x98;
}

/// ISR for receive complete interrupt
SIGNAL(SIG_USART1_RECV)
{
	data = UDR1; 		///making copy of data from UDR in data variable 

	UDR1 = data; 		///echo data back to PC


DDRC = 0xFF;
		if(data == 0x51)
		{
			velocity(255,255);
			forward();  ///forward
			_delay_ms(50);
			stop();

		}

		if(data  == 0x52)
		{
			velocity(255,255);
			back(); ///back
			_delay_ms(50);
			stop();
		}

		if(data == 0x53)
		{
			velocity(120,120);
			left();  ///left
			_delay_ms(50);
			stop();
		}

		if(data == 0x54)
		{
			velocity(120,120);
			right();  ///right
			_delay_ms(50);
			stop();
		}

		if(data == 0x50)
		{
			velocity(120,120);
			stop(); ///stop
		}

		if(data == 0x42)
		{
			buzzer_on();
			_delay_ms(1000); ///buzzer
			buzzer_off();
		}

}
