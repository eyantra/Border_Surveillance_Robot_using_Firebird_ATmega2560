/** \file main.c
 
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
#include "winavr_firebird.h"

/** Function To Initialize All The Devices */	
void init_devices()
{
 cli();
 port_init();  ///Initializes all the ports
 XMCRA=0x00;
 XMCRB=0x00;
 timer5_init();
 uart1_init(); ///Initailize UART0 for serial communiaction
 EIMSK  = 0x00;
 TIMSK0 = 0x00; ///timer0 interrupt sources
 TIMSK1 = 0x00; ///timer1 interrupt sources
 TIMSK2 = 0x00; ///timer2 interrupt sources
 TIMSK3 = 0x00; ///timer3 interrupt sources
 TIMSK4 = 0x00; ///timer4 interrupt sources
 TIMSK5 = 0x00; ///timer5 interrupt sources
 buzzer_pin_config();
 sei();   /// Enables the global interrupt 
 velocity(120,120);
}


unsigned char data; 
/** Main Function */
int main(void)
{
	init_devices();
	while(1);
}
