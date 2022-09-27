/******************************************************************************

File Name:           lcd v2.c
Dependencies:  		 Microchip Peripheral Library
Processor:           PIC24F256GA110
Company:             Microchip Technology, Inc.

Copyright 2013 Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Adrian Mot			6/20/13    Modified for PIC24FJ256GA110 family
 *****************************************************************************/
#include "MCP3910_EVB.h"


// Define functions for us/ms delay
void delay_us(int n)
{
	int i;
	for(i=0;i<n;i++){}
	for(i=0;i<n;i++){}
	for(i=0;i<n;i++){}
}
void delay_ms(int n)
{
	int i;
	int j;
	for(i=0;i<1000;i++)
	{
		for(j=0;j<n;j++){}
		for(j=0;j<n;j++){}
	}
}

// Define functions for start and stop I2C communication
void start(void)
{
	SDA=0;
	delay_us(3);
	SCL=0;
	delay_us(3);	
}	

void stop(void)
{
	SDA=0;
	delay_us(3);
	SCL=1;
	delay_us(3);
	SDA=1;
	delay_us(3);
}		

// Define function to send one bit
void send_bit(int n)
{
	if (n==0) SDA=0;
	else SDA=1;
	delay_us(3);
	SCL=1;
	delay_us(6);
	SCL=0;
	delay_us(3);
}

// Define function to get acknowledge
void get_ack(void)
{
	char b;
	delay_us(3);
	SCL=1;
	delay_us(3);
	b=SDAr;
	delay_us(3);
	SCL=0;
	delay_us(3);
}	

// Define function to send one byte
void send_byte(char out)
{
	int i;
	int data;
	data=out;
	for(i=0;i<8;i++)
	{
		if (data&0x80) send_bit(1);
		else send_bit(0);
		data=data<<1;
	}
	SDA_DIR=1;      
	get_ack();
	SDA_DIR=0;
	delay_us(3);
}	

// Define function to send one command or one data byte to LCD	
void send_to_LCD(char address,char control_byte,char data_byte)
{
	start();
	send_byte(address);
	send_byte(control_byte);
	send_byte(data_byte);
	stop();
	delay_us(100);
}

// Define function to send to LCD 2 lines to display Voltage,Current,PF and Power 
void send_text_format(void)
{
	int i;
	send_to_LCD(Slave_address,Control_command,0x80);	// Set DDRAM on first character of first line
	delay_ms(1);
	for(i=0;i<16;i++)
	{
		send_to_LCD(Slave_address,Control_data,line1_format[i]);
	}
	send_to_LCD(Slave_address,Control_command,0xC0);	// Set DDRAM on first character of second line
	delay_ms(1);
	for(i=0;i<16;i++)
	{
		send_to_LCD(Slave_address,Control_data,line2_format[i]);
	}
}

// Define function for LCD initialization
void Init_LCD()
{
	SDA_DIR=0;		// Set SDA associated pin as output
	SDA=1;			// Set SDA as 1 (no communication)
	SCL_DIR=0;		// Set SCL associated pin as output
	SCL=1;			// Set SCL as 1 (no communication)
	delay_ms(100);	
	send_to_LCD(Slave_address,Control_command,0x38);	// Function set DL = 8bits N = 2 lines
	send_to_LCD(Slave_address,Control_command,0x39);	// Function set DL = 8bits N = 2 lines; DH = single line font; IS = extended instruction set
	send_to_LCD(Slave_address,Control_command,0x14);	// Internal OSC frequency BS = 1/5 bias frame; frame frequency = 183Hz
	send_to_LCD(Slave_address,Control_command,0x78);	// Contrast set 0b101000	(first 2 MSB are set in the next instruction)
	send_to_LCD(Slave_address,Control_command,0x5D);	// Power/ICON/Contrast control ICON display = on; Booster circuit = on 
	send_to_LCD(Slave_address,Control_command,0x6D);	// Follower control Follower = on; follower amplified ratio = 0b101
	delay_ms(500);
	send_to_LCD(Slave_address,Control_command,0x0C);	// Display ON/OFF entire display = on; cursor = off; cursor position = off
	send_to_LCD(Slave_address,Control_command,0x01);	// Clear display
}
