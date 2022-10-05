/******************************************************************************

File Name:           interrupts rev2 v13.c
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

void __attribute__((interrupt, no_auto_psv)) _SPI1Interrupt(void)
{
	Data_decoding.word_val = SPI1BUF;		//Word 0
	if (Data_decoding.byte.LB==0xA5)		//Read from SPI
	{
		IEC0bits.SPI1IE=0;					//Stop the SPI interrupt
		T4CONbits.TON=1;					//timer 4 (32bits) is used to measure the time required to acquire all 2048 samples. 
		if (Zero_words_counter>4)			//At least 5 zero words
		{
//                        Config_Neutral = Data_decoding.byte.HB;
			while (!SPI1STATbits.SPIRBF);
			Data_decoding.word_val = SPI1BUF;	//Word 1
			if (channel==0)
			{
				voltage_msb[counter_buffer]=Data_decoding.byte.HB;
				voltage_nsb[counter_buffer]=Data_decoding.byte.LB;
			}
			while (!SPI1STATbits.SPIRBF);
			Data_decoding.word_val = SPI1BUF;	//Word 2
			if (channel==0)
			{
				voltage_lsb[counter_buffer]=Data_decoding.byte.HB;
				current_msb[counter_buffer]=Data_decoding.byte.LB;
			}
			while (!SPI1STATbits.SPIRBF);
			Data_decoding.word_val = SPI1BUF;	//Word 3
			if (channel==0)
			{
				current_nsb[counter_buffer]=Data_decoding.byte.HB;
				current_lsb[counter_buffer]=Data_decoding.byte.LB;
			}
			while (!SPI1STATbits.SPIRBF);
                        if (Flags1.bits.Same_OSR==1)
                        {
                            RPINR20bits.SDI1R = 40;                 //Switch the SDI input to Phase A
                        }
			
			Data_decoding.word_val = SPI1BUF;	//Word 4
			while (!SPI1STATbits.SPIRBF);
			Data_decoding.word_val = SPI1BUF;	//Word 5
//                        Config_PhaseA = Data_decoding.byte.HB;
			while (!SPI1STATbits.SPIRBF);
			Data_decoding.word_val = SPI1BUF;	//Word 6
			if (channel==1)
			{
				voltage_msb[counter_buffer]=Data_decoding.byte.HB;
				voltage_nsb[counter_buffer]=Data_decoding.byte.LB;
			}
			while (!SPI1STATbits.SPIRBF);
			Data_decoding.word_val = SPI1BUF;	//Word 7
			if (channel==1)
			{
				voltage_lsb[counter_buffer]=Data_decoding.byte.HB;
				current_msb[counter_buffer]=Data_decoding.byte.LB;
			}
			while (!SPI1STATbits.SPIRBF);
			Data_decoding.word_val = SPI1BUF;	//Word 8
			if (channel==1)
			{
				current_nsb[counter_buffer]=Data_decoding.byte.HB;
				current_lsb[counter_buffer]=Data_decoding.byte.LB;
			}
			while (!SPI1STATbits.SPIRBF);
                        if (Flags1.bits.Same_OSR==1)
                        {
                            RPINR20bits.SDI1R = 39;                 //Switch the SDI input to Phase B
                        }
			
			Data_decoding.word_val = SPI1BUF;	//Word 9
			while (!SPI1STATbits.SPIRBF);
			Data_decoding.word_val = SPI1BUF;	//Word 10
//                        Config_PhaseB = Data_decoding.byte.HB;
			while (!SPI1STATbits.SPIRBF);
			Data_decoding.word_val = SPI1BUF;	//Word 11
			if (channel==2)
			{
				voltage_msb[counter_buffer]=Data_decoding.byte.HB;
				voltage_nsb[counter_buffer]=Data_decoding.byte.LB;
			}
			while (!SPI1STATbits.SPIRBF);
			Data_decoding.word_val = SPI1BUF;	//Word 12
			if (channel==2)
			{
				voltage_lsb[counter_buffer]=Data_decoding.byte.HB;
				current_msb[counter_buffer]=Data_decoding.byte.LB;
			}
			while (!SPI1STATbits.SPIRBF);
			Data_decoding.word_val = SPI1BUF;	//Word 13
			if (channel==2)
			{
				current_nsb[counter_buffer]=Data_decoding.byte.HB;
				current_lsb[counter_buffer]=Data_decoding.byte.LB;
			}
			while (!SPI1STATbits.SPIRBF);
                        if (Flags1.bits.Same_OSR==1)
                        {
                            RPINR20bits.SDI1R = 38;                 //Switch the SDI input to Phase C
                        }
			
			Data_decoding.word_val = SPI1BUF;	//Word 14
			while (!SPI1STATbits.SPIRBF);
			Data_decoding.word_val = SPI1BUF;	//Word 15
//                        Config_PhaseC = Data_decoding.byte.HB;
			while (!SPI1STATbits.SPIRBF);
			Data_decoding.word_val = SPI1BUF;	//Word 16
			if (channel==3)
			{
				voltage_msb[counter_buffer]=Data_decoding.byte.HB;
				voltage_nsb[counter_buffer]=Data_decoding.byte.LB;
			}
			while (!SPI1STATbits.SPIRBF);
			Data_decoding.word_val = SPI1BUF;	//Word 17
			if (channel==3)
			{
				voltage_lsb[counter_buffer]=Data_decoding.byte.HB;
				current_msb[counter_buffer]=Data_decoding.byte.LB;
			}
			while (!SPI1STATbits.SPIRBF);
			Data_decoding.word_val = SPI1BUF;	//Word 18
			if (channel==3)
			{
				current_nsb[counter_buffer]=Data_decoding.byte.HB;
				current_lsb[counter_buffer]=Data_decoding.byte.LB;
			}
			while (!SPI1STATbits.SPIRBF);
                        if (Flags1.bits.Same_OSR==1)
                        {
                            RPINR20bits.SDI1R = 44;                 //Switch the SDI input to Neutral
                        }
			
			Data_decoding.word_val = SPI1BUF;	//Word 19
			if(counter_buffer==BUFFER_LENGTH-1)
			{		
				T4CONbits.TON=0;						// when acquisition is done stop the timer 4 and save its value
				internal_registers[24]=TMR5;
				internal_registers[25]=TMR4;
				TMR4=0;
				TMR5=0;				
				//IPC1bits.OC2IP=4;		//Start TX
				if(Flags1.bits.Start_cycle==1)        
                    Flags1.bits.Start_transmision = 1;  // Buffer is full
				counter_buffer=0;
			}
			else
			{
				counter_buffer++;
			}   
		}
		Zero_words_counter=0;
//		IEC0bits.U1RXIE=1;
	}
	else
	{
		Zero_words_counter++;
		//if (Zero_words_counter>8) IEC0bits.U1RXIE=0;
	}

	if (SPI1STATbits.SPIROV)
	{
		SPI1STATbits.SPIROV	= 0;			//Clear overflow
	}
    IFS0bits.SPI1IF=0;
	if (Flags1.bits.Start_transmision==0) IEC0bits.SPI1IE=1;					//Start the SPI interrupt
 }


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>	
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void)
{
	IFS0bits.T3IF=0;
	T2CONbits.TON=0;
	LEDG=0;
	if (SPI1STATbits.SPIROV)
	{
            SPI1STATbits.SPIROV	= 0;			//Clear overflow
	}
	IFS0bits.SPI1IF=0;
	IEC0bits.SPI1IE=1;          			//enable SPI interrupt
//	IPC7bits.INT2IP=4;    								// the timer 3 is used to control the acquisition time (not sampling speed)
}	
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// UART RX receives the functions from PC
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{
	IFS0bits.U1RXIF=0;
        while (U1STAbits.URXDA == 1)
        {
            rx_data=U1RXREG;
            rxbuffer[urxdata]=rx_data;
            if (rx_data)urxdata++;
            if (urxdata>=max_urxdata) urxdata = 0;
        }
        if (U1STAbits.OERR == 1) U1STAbits.OERR = 0;

//	if (rx_data==0x0D)
//	{
//		command_received=1;
//		//IEC0bits.U1RXIE=0;
//	}
}

void __attribute__((interrupt, auto_psv)) _U1ErrInterrupt(void)
{
	IFS4bits.U1ERIF=0;
	LEDR=1;
//	while(1)
//	{}
}

// My interrupts
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// Change Notification Interrupt for the buttons
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void)
{
    newBUTTON2=CN16;
    newBUTTON3=CN15;
    newBUTTON4=CN19;
    start_flag_state=Flags1.bits.Start_cycle;
    //newLED4=PORTAbits.RA9;
    //newLED3=PORTAbits.RA10;
    if(newBUTTON2==0)
    {
        screen++;
        if(screen>4)
            screen=0;
    }
    if(newBUTTON3==0)
    {
        ch++;
        if(ch>3)
            ch=0;
    }
    if(newBUTTON4==0)
    {
        if(start_flag_state==0)
        {
            // Start cycle when it is first pressed
            Flags1.bits.Start_cycle=1;
            if (SPI1STATbits.SPIROV)
            {
                SPI1STATbits.SPIROV	= 0;			//Clear overflow
            }
            IFS0bits.SPI1IF=0;
            IEC0bits.SPI1IE=1;          			//enable SPI interrupt -> ADC data reception
        }
        else
        {   // Stop cycle when it is pressed again
            Flags1.bits.Start_cycle=0;
            TMR2=0;
            TMR3=0;
            T2CONbits.TON=0;
        }
    }
    IFS1bits.CNIF = 0;      // Clear CNI interrupt flag
}