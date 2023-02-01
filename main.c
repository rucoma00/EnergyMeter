/******************************************************************************

File Name:           main.c
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
 * Rubén Concejo 		30/11/22   Added modifications for wattmeter
 *****************************************************************************/
// PIC24FJ256GA110 Configuration Bit Settings

// 'C' source line config statements

// CONFIG3
#pragma config WPFP = WPFP511           // Write Protection Flash Page Segment Boundary (Highest Page (same as page 170))
#pragma config WPDIS = WPDIS            // Segment Write Protection Disable bit (Segmented code protection disabled)
#pragma config WPCFG = WPCFGDIS         // Configuration Word Code Page Protection Select bit (Last page(at the top of program memory) and Flash configuration words are not protected)
#pragma config WPEND = WPENDMEM         // Segment Write Protection End Page Select bit (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMOD = XT             // Primary Oscillator Select (XT oscillator mode selected)
#pragma config I2C2SEL = PRI            // I2C2 Pin Select bit (Use Default SCL2/SDA2 pins for I2C2)
#pragma config IOL1WAY = OFF            // IOLOCK One-Way Set Enable bit (Unlimited Writes To RP Registers)
#pragma config OSCIOFNC = OFF           // Primary Oscillator Output Function (OSCO functions as CLKO (FOSC/2))
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-safe Clock Monitor are disabled)
#pragma config FNOSC = PRIPLL           // Oscillator Select (Primary oscillator (XT, HS, EC) with PLL module (XTPLL,HSPLL, ECPLL))
#pragma config IESO = OFF               // Internal External Switch Over Mode (IESO mode (Two-speed start-up)disabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Standard Watchdog Timer is enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config ICS = PGx2               // Comm Channel Select (Emulator functions are shared with PGEC2/PGED2)
#pragma config GWRP = OFF               // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG port is disabled)
#pragma config BKBUG = OFF

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include "MCP3910_EVB.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

//Configuration Bits Set in code
//_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & BKBUG_OFF & ICS_PGx2 & FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768)
//_CONFIG2( IESO_OFF & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF & IOL1WAY_OFF & I2C2SEL_PRI & POSCMOD_XT )
//_CONFIG3( WPCFG_WPCFGDIS & WPDIS_WPDIS )

//Globals

signed char voltage_msb[BUFFER_LENGTH], voltage_nsb[BUFFER_LENGTH], voltage_lsb[BUFFER_LENGTH], current_msb[BUFFER_LENGTH], current_nsb[BUFFER_LENGTH], current_lsb[BUFFER_LENGTH];
unsigned int counter_buffer, counter_tx, count_rx, rx_function, register_write_cnt;
unsigned char tx_buf[10], tx_bufc[3], types, rxcounter, char_to_int[3];
unsigned long internal_registers[26];
unsigned char receive_counter, urxdata, decode_counter, decode_buffer[10], rxbuffer[max_urxdata], start_acquisition, rx_data, write_reg_count;
unsigned int Zero_words_counter;
unsigned char command_received;

FLAGS Flags1;
WDATA Data_decoding;
unsigned char channel, Config_Neutral, Config_PhaseA, Config_PhaseB, Config_PhaseC;


char line1_format[]=" MCP3910 PIC24  ";
char line2_format[]="Evaluation board";

// My variables
int32_t Vmcp_CH0[BUFFER_LENGTH], Vmcp_CH1[BUFFER_LENGTH];
double Vconv_CH0[BUFFER_LENGTH], Vconv_CH1[BUFFER_LENGTH];
double Vrms_CH0=0, Vrms_CH1=0, Vrms=0, Irms=0, P=0, Q=0, S=0, PF=0, Energy=0;  
char str_Vrms[10]={0};
char str_Irms[10]={0};
char str_P[10]={0};
char str_Q[10]={0};
char str_S[10]={0};
char str_PF[10]={0};
char str_Energy[10]={0};
char send[100]=":";

volatile unsigned int screen=0, newBUTTON2=0, newBUTTON3=0, newBUTTON4=0, start_process=0;

uint32_t adc_timer_raw;
float    adc_timer_ms; 

// My functions
double Calc_RMS(double* param, int N){
    float producto=0;
    float suma=0;
    float valor_RMS=0;
    int i;
    for(i=0;i<N;i++){
    producto=param[i]*param[i];
    suma+=producto;
 }
 valor_RMS=sqrt(suma/N);
 return valor_RMS;
}

double Calc_P(double* Volts, double* Amps, int N)
{
 float producto=0;
 float suma=0;
 float P;
 int i;
 for(i=0;i<N;i++){
 producto=Volts[i]*Amps[i];
 suma+=producto;
 }
  P=suma/N;
 return P;
}

double Calc_Q(double* Volts, double* Amps, int N)
{
 float producto=0;
 float suma=0;
 float Q;
 int i;
 for(i=N/4;i<N;i++){
 producto=Volts[i-N/4]*Amps[i];
 suma+=producto;
 }
 Q=suma/N;
 return Q;
}


int main (void)
{
    unsigned char i;

    __delay_ms(100);		//Waiting for Vdd to raise
    Init();
    ioMap();
    Init_LCD();
//    __delay_ms(10);
    for (i=0;i<4;i++)           //1 second
    {
        __delay_ms(250);
    }
    
    send_text_format();

    DMCLK=1;
    MCLK=1;

    detect_jumpers();
    SRbits.IPL=3;               //enable global interrupts
    Flags1.bits.Same_OSR=0;
    if ((internal_registers[0]!=0) && (internal_registers[3]!=0) && (internal_registers[6]!=0) && (internal_registers[9]!=0))
    {
        //If OSR is the same for all channels
        if ((internal_registers[0]==internal_registers[3]) && (internal_registers[3]==internal_registers[6]) && (internal_registers[6]==internal_registers[9]))
        {
            Flags1.bits.Same_OSR=1;
        }
        SPI1STATbits.SPIEN=1;       //enable SPI module
        IFS0bits.SPI1IF=0;
        IEC0bits.SPI1IE=1;          //enable SPI interrupt

        OC1CON1bits.OCM = 0b101;    // This selects and starts the Edge Aligned PWM mode
    }
    // CN interrupt
    IPC4bits.CNIP = 6;
    DCN16=1;    // BUTTON2 input
    //DCN15=1;    // BUTTON3 input
    DCN19=1;    // BUTTON4 input
    DLEDR=0;    // LD3 output
    DLEDG=0;    // LD4 output
    
    CNEN2bits.CN16IE = 1;   // Interrupt enabled. Pin : RD7
    //CNEN1bits.CN15IE = 1;   // Interrupt enabled. Pin : RD6
    CNEN2bits.CN19IE = 1;   // Interrupt enabled. Pin : RD13
    IEC1bits.CNIE = 1;      // Enable CNI interrupt
    IFS1bits.CNIF = 0;      // Clear CNI interrupt flag
    
    while(!start_process)   // Wait for the button to be pressed to start
    {
        delay_ms(300);
        sprintf(line1_format,"%s","  Press SW4 to  ");
        sprintf(line2_format,"%s","     begin      ");  
        send_text_format();
        delay_ms(300);
        sprintf(line1_format,"%s"," MCP3910 PIC24  ");
        sprintf(line2_format,"%s","Evaluation board");
        send_text_format();
    }
    //channel=1;
    Flags1.bits.Start_cycle=1;
    while(1)
    {
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
        // When BUFFER_SIZE samples have been captured
        if (Flags1.bits.Start_transmision)  // It should be 1 when the buffer is full
        {
            //transmit_data();  // Not transmitting this data
            // Calculations when the buffer is full
            // Two periods of voltage and current
            for(i=0;i<BUFFER_LENGTH;i++)
            {
                Vmcp_CH0[i]= ((uint32_t)voltage_msb[i] << 16) | ((uint32_t)voltage_nsb[i] << 8) | (uint32_t)voltage_lsb[i];            
                Imcp_CH1[i]= ((uint32_t)current_msb[i] << 16) | ((uint32_t)current_nsb[i] << 8) | (uint32_t)current_lsb[i];
            
                Vconv_CH0[i]=Vmcp[i]*1.2/8388607;
                Iconv_CH1[i]=Imcp[i]*1.2/8388607;        
            }
            // RMS values
            Vrms=Calc_RMS(Vconv,BUFFER_LENGTH);
            Irms=Calc_RMS(Iconv,BUFFER_LENGTH);
            // P, Q, S, PF
            P=Calc_P(Vconv_CH0,Vconv_CH1,BUFFER_LENGTH);
            Q=Calc_Q(Vconv-CH0,Iconv_CH1,BUFFER_LENGTH);
            S=Vrms*Irms;
            if (S<0.001)
                PF=1;   // Protection so that there's never /0
            else
                PF=P/S;
            // Saving in strings
            sprintf(str_Vrms,"%3.2f",Vrms);
            sprintf(str_Irms,"%2.3f",Irms);
            sprintf(str_P,"%.5f",P);
            sprintf(str_Q,"%.5f",Q);
            sprintf(str_S,"%.2f",S);
            sprintf(str_PF,"%.5f",PF);
            sprintf(str_Energy,"%.1f",Energy);
            
            
            // Assemble string to send data
            snprintf(send,sizeof(send),"%s,%s,%s,%s,%s,%s,%s\n",str_Vrms,str_Irms,str_P,str_Q,str_S,str_PF,str_Energy);
            send_UART(send);                    // Send values
            
            // End of transmit_data() to enable no acquisition
            Flags1.bits.Start_transmision=0;    // Ready to capture again
            if(Flags1.bits.Start_cycle==1)      // if acquisition is running enable the INT2 for a new acquisition
			{
				TMR2=0;
				TMR3=0;
				T2CONbits.TON=1;
				LEDG=1;
			}	
        }
        
        //LCD Display
        switch(screen)
        {
            case 0:
                sprintf(line1_format,"%s"," MCP3910 PIC24  ");
                sprintf(line2_format,"%s","Evaluation board");
                break;
            case 1: // RMS
                sprintf(line1_format,"#%d: Vrms = %.3f", channel, Vrms);
                sprintf(line2_format,"#%d: Irms = %.3f", channel, Irms);
                break;
            case 2: // P, Q
                sprintf(line1_format,"#%d: P = %.5f ", channel, P);
                sprintf(line2_format,"#%d: Q = %.5f ", channel, Q);
                break;
            case 3: // S, PF
                sprintf(line1_format,"#%d: S = %.5f  ", channel, S);
                sprintf(line2_format,"#%d: PF = %.5f ", channel, PF);
                break;
            case 4:
                sprintf(line1_format,"#%d: ADCms = %f", channel, adc_timer_ms);
                sprintf(line2_format,"#%d: Eng =  %f ", channel, Energy);
                break;

        }
        send_text_format(); // Update screen
        //delay_ms(500);      // Wait
        while(start_process==0)
        {
            delay_ms(800);
            sprintf(line1_format,"%s","  Press SW4 to  ");
            sprintf(line2_format,"%s","    continue    ");  
            send_text_format();
            delay_ms(800);
            sprintf(line1_format,"%s","Process on break");
            sprintf(line2_format,"%s","   Waiting...   ");
            send_text_format();
        }
    }
    return 0;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
