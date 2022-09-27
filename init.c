/******************************************************************************

File Name:           init.c
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


void Init(void)
{
    INTCON1bits.NSTDIS=0;   // enable nested interupts
    SRbits.IPL=7;           // disable user interupts
    INTCON2bits.INT3EP=0;   // int3 positive edge

    OSCTUNbits.TUN=0;       // FRC frequency 8MHz

    CLKDIVbits.ROI=0;       // interrupts have no effect on the DOZEN bit
    CLKDIVbits.DOZE=0;      // processor clock reduction select bits Fcy/1
    CLKDIVbits.DOZEN=1;     // procesor clock/ peripheral clock ratio is given by DOZE bits
    CLKDIVbits.RCDIV=0;    // Internal fast oscilator postscaler FRC divide by 1

    OSCCONbits.COSC=0b011;      // Oscilator selection: XT with PLL
    OSCCONbits.NOSC=0b011;      // New Oscilator selection: XT with PLL (16MIPS)
    OSCCONbits.CLKLOCK=0;   // clock lock enable - not locked
    OSCCONbits.LPOSCEN=0;   // Secondary oscilator enable: disable
    OSCCONbits.OSWEN=0;     // Oscilator swich enable bit : complete
    OSCCONbits.CLKLOCK=1;   // clock lock enable - locked
    RCON=0x00; // Reset control register

    IFS0bits.SPI1IF=0;
    IEC0bits.SPI1IE=0;          	//Interrupt inactive until command received from PC
    IPC2bits.SPI1IP=6;

    SPI1CON1bits.MODE16=1;          //16 bits mode
    SPI1CON1bits.MSTEN=0;			//slave mode
    SPI1CON1bits.CKP=0;             //iddle clock is low level
    SPI1CON1bits.SMP=0;             //data sampled at end of data output time
    SPI1CON1bits.DISSDO=1;          //disable SDO
    
    SPI1CON2bits.SPIBEN=0;          //enhanced mode
    SPI1STATbits.SISEL0=1;          //interrupt when SPI receive buffer is full (SPI1RBF=1)
    SPI1STATbits.SISEL1=0;
    SPI1STATbits.SISEL2=0;

    IFS0bits.SPF1IF=0;
    IEC0bits.SPF1IE=0;
    IPC2bits.SPF1IP=2;

                                    // OC1 is used for controlling the sampling speed
    OC1CON1 = 0;                    // It is a good practice to clear off the control bits initially
    OC1CON2 = 0;
    OC1CON1bits.OCTSEL = 0b111;     // This selects the peripheral clock as the clock input to the OC module

    OC1CON2bits.SYNCSEL=0b11111;

    OC1R = 2;                       // This is just a typical number, user must calculate based on the waveform requirements and the system clock
    OC1RS = 3;                      // Determines the Period

    IEC0bits.OC2IE=0;

    OC2CON1 = 0;
    OC2CON2 = 0;
    OC2CON1bits.OCTSEL = 0b111;
    OC2R = 2000;					//2000;
    OC2RS = 4000;					//4000;
    OC2CON1bits.OCM = 0b111;
    OC2CON2bits.SYNCSEL=0b11111;

    IFS0bits.U1RXIF=0;              // UART RX interrupt is used to receive the data from PC, clear interrupt flag of rx
    IPC2bits.U1RXIP=5;              // Set uart1 Priority to 5
    IEC0bits.U1RXIE=1;              // enable rx recieved data interrupt
    IEC4bits.U1ERIE=1;
    IEC4bits.U1ERIE=0;
    IPC16bits.U1ERIP=2;

    U1MODEbits.UARTEN=1;
    U1MODEbits.BRGH=0;              // Default: High baud rate enabled

    U1STAbits.UTXEN=1;              // Transmit is enabled
    U1STAbits.URXISEL=0b00;         // Interrupt flag bit is set when RXBUF is filled whith 1 character
    U1BRG=6;                        // The default baud rate used is 921kbauds (3)

    AD1PCFG=0xFFFF;

    T2CONbits.TON=0;
    T2CONbits.TSIDL=0;
    T2CONbits.TGATE=0;
    T2CONbits.TCKPS1=1;
    T2CONbits.TCKPS0=0;
    T2CONbits.T32=1;
    T2CONbits.TCS=0;

    PR2=0;                          //  timer 3 is used to control the acquisition time (not sampling speed)
    PR3=1;

    IFS0bits.T3IF=0;
    IEC0bits.T3IE=1;
    IPC2bits.T3IP=4;


    T4CONbits.TON=0;                //  timer 4 (32 bits) is used to measure the sampling speed
    T4CONbits.TSIDL=0;
    T4CONbits.TGATE=0;
    T4CONbits.TCKPS1=0;
    T4CONbits.TCKPS0=0;
    T4CONbits.T32=1;
    T4CONbits.TCS=0;

    DRESET=1;
    RESET=1;

    DDR=1;
    DR=1;

    DCS=1;
    CS=1;

    DLEDR=0;
    LEDR=0;

    DLEDG=0;
    LEDG=0;

    DEN232=0;
    EN232=0;

    DSCLK=1;
    SCLK=1;

    DMOSI=1;
    MOSI=1;

    DMISO=1;
    MISO=1;

    DMOD1=0;
    MOD1=0;

    DMOD0=0;
    MOD0=0;
          
    DTX1=0;
    TX1=1;

    DRX1=1;
    RX1=1;

    CN19=1;
    CN16=1;
    CN15=1;
    DCN19=1;
    DCN16=1;
    DCN15=1;

    DMCLK=0;
    MCLK=0;
    __delay_us(1);
    MCLK=1;
    __delay_us(15);				//Keeps clock high to reset the ADCs

    counter_buffer=0;
    counter_tx=0;
    count_rx=0;
    urxdata=0;
    receive_counter=0;
    decode_counter=0;
    rx_function=0;
    types=1;
    rxcounter=0;
    
    Flags1.bits.Start_transmision=0;
    Flags1.bits.Start_cycle=0;
    write_reg_count=0;
    Zero_words_counter=0;
    channel=0;
}
