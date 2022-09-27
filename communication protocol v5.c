/******************************************************************************

File Name:           communication protocol v5.c
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


void send_UART(char *c)
{
	while(*c) send_char( *c++);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void send_char(char d)  // Transmission of a byte "d"
{
	while (!U1STAbits.TRMT);
	U1TXREG = d;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void send_internal_registers(void)
{
	unsigned char i, j;

	for(i=0;i<23;i++)
	{
		hexdec_long(internal_registers[i]);
		j=1;
		if(tx_buf[2]!=48)
		{
			j=0;
			send_char(tx_buf[2]);
		}
		if(j)
		{
			if(tx_buf[3]!=48)
			{
				j=0;
				send_char(tx_buf[3]);
			}
		}
		else send_char(tx_buf[3]);
		if(j)
		{
			if(tx_buf[4]!=48)
			{
				j=0;
				send_char(tx_buf[4]);
			}
		}
		else send_char(tx_buf[4]);
		if(j)
		{
			if(tx_buf[5]!=48)
			{
				j=0;
				send_char(tx_buf[5]);
			}
		}
		else send_char(tx_buf[5]);
		if(j)
		{
			if(tx_buf[6]!=48)
			{
				j=0;
				send_char(tx_buf[6]);
			}
		}
		else send_char(tx_buf[6]);
		if(j)
		{
			if(tx_buf[7]!=48)
			{
				j=0;
				send_char(tx_buf[7]);
			}
		}
		else send_char(tx_buf[7]);
		if(j)
		{
			if(tx_buf[8]!=48)
			{
				j=0;
				send_char(tx_buf[8]);
			}
		}
		else send_char(tx_buf[8]);
		send_char(tx_buf[9]);
		if(i<22)
		{
			send_char(',');
		}
	}
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void send_internal_registers2(void)
{
        while (!U1STAbits.TRMT);
	U1TXREG = internal_registers[0]+0x30;
        while (!U1STAbits.TRMT);
	U1TXREG = ',';
        while (!U1STAbits.TRMT);
	U1TXREG = internal_registers[1]+0x30;
        while (!U1STAbits.TRMT);
	U1TXREG = ',';
        while (!U1STAbits.TRMT);
	U1TXREG = internal_registers[2]+0x30;
        while (!U1STAbits.TRMT);
	U1TXREG = ',';
        while (!U1STAbits.TRMT);
	U1TXREG = internal_registers[3]+0x30;
        while (!U1STAbits.TRMT);
	U1TXREG = ',';
        while (!U1STAbits.TRMT);
	U1TXREG = internal_registers[4]+0x30;
        while (!U1STAbits.TRMT);
	U1TXREG = ',';
        while (!U1STAbits.TRMT);
	U1TXREG = internal_registers[5]+0x30;
        while (!U1STAbits.TRMT);
	U1TXREG = ',';
        while (!U1STAbits.TRMT);
	U1TXREG = internal_registers[6]+0x30;
        while (!U1STAbits.TRMT);
	U1TXREG = ',';
        while (!U1STAbits.TRMT);
	U1TXREG = internal_registers[7]+0x30;
        while (!U1STAbits.TRMT);
	U1TXREG = ',';
        while (!U1STAbits.TRMT);
	U1TXREG = internal_registers[8]+0x30;
        while (!U1STAbits.TRMT);
	U1TXREG = ',';
        while (!U1STAbits.TRMT);
	U1TXREG = internal_registers[9]+0x30;
        while (!U1STAbits.TRMT);
	U1TXREG = ',';
        while (!U1STAbits.TRMT);
	U1TXREG = internal_registers[10]+0x30;
        while (!U1STAbits.TRMT);
	U1TXREG = ',';
        while (!U1STAbits.TRMT);
	U1TXREG = internal_registers[11]+0x30;
        while (!U1STAbits.TRMT);
}
///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void set_timer(void)
{
	rxbuffer[0]=rx_data;
	IFS0bits.U1RXIF=0;
	while (IFS0bits.U1RXIF==0)
	{
		rxbuffer[1]=rx_data;
		IFS0bits.U1RXIF=0;
	}
	while (IFS0bits.U1RXIF==0)
	{
		rxbuffer[2]=rx_data;
		IFS0bits.U1RXIF=0;
	}
	write_timer();
	rx_function=0;
	count_rx=0;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void write_timer(void)
{
	unsigned char val;
	
if(rxbuffer[3]==0x0D)
	{
	val = 100*(rxbuffer[0]-48)+10*(rxbuffer[1]-48)+rxbuffer[2]-48;
	PR3=val;
	TMR2=0;
	TMR3=0;
		send_char('t');
		send_char(rxbuffer[0]);
		send_char(rxbuffer[1]);
		send_char(rxbuffer[2]);
		send_char(13);
	}
	else
	{
		send_char('t');
		send_char('1');
		send_char(13);
	}
}

void receive_data(void)
{
	unsigned char val;

    if (receive_counter != urxdata)
    {
        decode_buffer[decode_counter] = rxbuffer[receive_counter];
        if (rxbuffer[receive_counter] != 0x0D)
        {
            decode_counter++;
        }
        else
        {
            decode_counter = 0;         //Initialize for the next command
            switch (decode_buffer[0])
            {
                    case 'i':
                                    send_char('i');
                                    send_UART(board_ID);
                                    send_char(0x0D);
                            break;

                    case 's':
                                    send_char('s');
                                    if (Flags1.bits.Start_cycle==0)
                                    {
                                            Flags1.bits.Start_cycle=1;
                                            if (SPI1STATbits.SPIROV)
                                            {
                                                    SPI1STATbits.SPIROV	= 0;			//Clear overflow
                                            }
                                            IFS0bits.SPI1IF=0;
                                            IEC0bits.SPI1IE=1;          			//enable SPI interrupt -> ADC data reception
                                    }
                            break;

                    case 'p':
                                    send_char('p');
                                    send_char(0x0D);
                                    Flags1.bits.Start_cycle=0;
                                    TMR2=0;
                                    TMR3=0;
                                    T2CONbits.TON=0;
                            break;

                    case 'R':
                                    Read_Internal_Registers2();
                                    send_char('R');
                                    send_internal_registers2();             //The new protocol
                                    send_char(0x0D);
                            break;

                    case 'v':
                                    send_char('v');
                                    send_UART(firmware_ID);
                                    send_char(0x0D);
                            break;

                    case 't':
                                    val = 100*(decode_buffer[1]-48)+10*(decode_buffer[2]-48)+decode_buffer[3]-48;
                                    PR3=val;
                                    TMR2=0;
                                    TMR3=0;
                                    send_char('t');
                                    send_char(decode_buffer[1]);
                                    send_char(decode_buffer[2]);
                                    send_char(decode_buffer[3]);
                                    send_char(0x0D);
                                    break;

                    case 'C':
                                    val=(decode_buffer[1]-48)&0x07;
                                    channel=val>>1;
                                    switch (channel)
                                    {
                                        case 0:
                                            RPINR20bits.SDI1R = 44;                 //Switch the SDI input to Neutral
                                            break;
                                        case 1:
                                            RPINR20bits.SDI1R = 40;                 //Switch the SDI input to Phase A
                                            break;
                                        case 2:
                                            RPINR20bits.SDI1R = 39;                 //Switch the SDI input to Phase B
                                            break;
                                        case 3:
                                            RPINR20bits.SDI1R = 38;                 //Switch the SDI input to Phase C
                                            break;
                                    }
                                    send_char('C');
                                    send_char(decode_buffer[1]);
                                    send_char(decode_buffer[2]);
                                    send_char(decode_buffer[3]);
                                    send_char(0x0D);
                                    break;

                    case 'c':
                                    send_char('c');
                                    val=(channel<<1)+48;
                                    send_char(val);
                                    send_char(',');
                                    val++;
                                    send_char(val);
                                    send_char(0x0D);
                                    break;

                    default:
                            break;
            }
        }
     receive_counter++;
     if (receive_counter >= max_urxdata) receive_counter = 0;
    }
}

void transmit_data(void)
{
	union
	{
		unsigned long voltage;
		struct
		{
			unsigned char LSB : 8;
			unsigned char NSB : 8;
			unsigned char MSB : 8;
			unsigned char d : 8;
		};
	} volt;

	union
	{
		unsigned long current;
		struct
		{
			unsigned char LSB : 8;
			unsigned char NSB : 8;
			unsigned char MSB : 8;
			unsigned char d : 8;
		};
	} amp;
	
    if(counter_tx<BUFFER_LENGTH)
	{
	    if(counter_tx==0)
		{
   			while (!U1STAbits.TRMT);
			U1TXREG = '!';
 		}
		volt.d=0;
		volt.MSB=voltage_msb[counter_tx];
		volt.NSB=voltage_nsb[counter_tx];
		volt.LSB=voltage_lsb[counter_tx];
		hexdec_long(volt.voltage);
		while (!U1STAbits.TRMT);
		U1TXREG = tx_buf[2];
		while (!U1STAbits.TRMT);
		U1TXREG = tx_buf[3];
		while (!U1STAbits.TRMT);
		U1TXREG = tx_buf[4];
		while (!U1STAbits.TRMT);
		U1TXREG = tx_buf[5];
		while (!U1STAbits.TRMT);
		U1TXREG = tx_buf[6];
		while (!U1STAbits.TRMT);
		U1TXREG = tx_buf[7];
		while (!U1STAbits.TRMT);
		U1TXREG = tx_buf[8];
		while (!U1STAbits.TRMT);
		U1TXREG = tx_buf[9];
		while (!U1STAbits.TRMT);
		U1TXREG = ',';
		
		amp.d=0;
		amp.MSB=current_msb[counter_tx];
		amp.NSB=current_nsb[counter_tx];
		amp.LSB=current_lsb[counter_tx];
		hexdec_long(amp.current);
		while (!U1STAbits.TRMT);
		U1TXREG = tx_buf[2];
		while (!U1STAbits.TRMT);
		U1TXREG = tx_buf[3];
		while (!U1STAbits.TRMT);
		U1TXREG = tx_buf[4];
		while (!U1STAbits.TRMT);
		U1TXREG = tx_buf[5];
		while (!U1STAbits.TRMT);
		U1TXREG = tx_buf[6];
		while (!U1STAbits.TRMT);
		U1TXREG = tx_buf[7];
		while (!U1STAbits.TRMT);
		U1TXREG = tx_buf[8];
		while (!U1STAbits.TRMT);
		U1TXREG = tx_buf[9];
	
		if(counter_tx==BUFFER_LENGTH-1)
		{
			while (!U1STAbits.TRMT);
			U1TXREG = ':';	
			hexdec_long(internal_registers[24]);
			U1TXREG = tx_buf[0];
			while (!U1STAbits.TRMT);
			U1TXREG = tx_buf[1];
			while (!U1STAbits.TRMT);
			U1TXREG = tx_buf[2];
			while (!U1STAbits.TRMT);
			U1TXREG = tx_buf[3];
			while (!U1STAbits.TRMT);
			U1TXREG = tx_buf[4];
			while (!U1STAbits.TRMT);
			U1TXREG = tx_buf[5];
			while (!U1STAbits.TRMT);
			U1TXREG = tx_buf[6];
			while (!U1STAbits.TRMT);
			U1TXREG = tx_buf[7];
			while (!U1STAbits.TRMT);
			U1TXREG = tx_buf[8];
			while (!U1STAbits.TRMT);
			U1TXREG = tx_buf[9];
			while (!U1STAbits.TRMT);
			U1TXREG = ',';

			hexdec_long(internal_registers[25]);
			U1TXREG = tx_buf[0];
			while (!U1STAbits.TRMT);
			U1TXREG = tx_buf[1];
			while (!U1STAbits.TRMT);
			U1TXREG = tx_buf[2];
			while (!U1STAbits.TRMT);
			U1TXREG = tx_buf[3];
			while (!U1STAbits.TRMT);
			U1TXREG = tx_buf[4];
			while (!U1STAbits.TRMT);
			U1TXREG = tx_buf[5];
			while (!U1STAbits.TRMT);
			U1TXREG = tx_buf[6];
			while (!U1STAbits.TRMT);
			U1TXREG = tx_buf[7];
			while (!U1STAbits.TRMT);
			U1TXREG = tx_buf[8];
			while (!U1STAbits.TRMT);
			U1TXREG = tx_buf[9];
			while (!U1STAbits.TRMT);
			U1TXREG = 13;
			counter_tx=0;
			Flags1.bits.Start_transmision = 0;

			if(Flags1.bits.Start_cycle==1)			// if acquisition is running enable the INT2 for a new acquisition
			{
				TMR2=0;
				TMR3=0;
				T2CONbits.TON=1;
				LEDG=1;
			}	
		}
		else
		{
			while (!U1STAbits.TRMT);
			U1TXREG = ';';
			while (!U1STAbits.TRMT);
			counter_tx++;
                }
	}
}


