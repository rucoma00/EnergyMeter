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
int32_t Vmcp[68], Imcp[68];
double Vconv[68], Iconv[68];
double Vrms, Irms, P, Q, S, PF;
char str_Vrms[10]={0};
char str_Irms[10]={0};
char str_P[10]={0};
char str_Q[10]={0};
char str_S[10]={0};
char str_PF[10]={0};
char send[100]=":";

volatile unsigned int screen=0, ch=0, newPORTD=0, newLED4=0, newBUTTON2=0, newBUTTON3=0, newLED3=0,
                    oldBUTTON2=0, oldBUTTON3=0;



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
 for(i=0;i<N;i++){
 producto=Volts[i+N/4]*Amps[i];
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
    DCN16=1;     // BUTTON2 como entrada
    DCN15=1;    // BUTTON3 como entrada
    DLEDR=0;    // LD3 como salida
    DLEDG=0;    // LD4 como salida
    
    CNEN2bits.CN16IE = 1;   // Interupt enabled. Pin : RD7
    CNEN1bits.CN15IE = 1;   // Interrupt enabled. Pin : RD6
    IEC1bits.CNIE = 1;      // Enable CNI interrupt
    IFS1bits.CNIF = 0;      // Clear CNI interrupt flag

    while(1)
    {
        receive_data();
        if (Flags1.bits.Start_transmision)  // It shoud be 1 when the buffer is full
        {
            //transmit_data();
            // Calculations should probably be done here. Needs to be tested
            if(channel != ch)
            {
                channel=ch;
            }
        }
        
        // Two periods of voltage and current
        for(i=0;i<68;i++)
        {
            Vmcp[i]= ((uint32_t)voltage_msb[i] << 16) | ((uint32_t)voltage_nsb[i] << 8) | (uint32_t)voltage_lsb[i];            
            Imcp[i]= ((uint32_t)current_msb[i] << 16) | ((uint32_t)current_nsb[i] << 8) | (uint32_t)current_lsb[i];
            
            Vconv[i]=Vmcp[i]*3.3/8388607;
            Iconv[i]=Imcp[i]*5/8388607;        
        }
        
        // RMS values
        Vrms=Calc_RMS(Vconv,68);
        Irms=Calc_RMS(Iconv,68);
        
        // P, Q, S, PF
        P=Calc_P(Vconv,Iconv,68);
        Q=Calc_Q(Vconv,Iconv,68);
        S=Vrms*Irms;
        PF=P/S;
        
        // Guardar en strings
        sprintf(str_Vrms,"%.3f",Vrms);
        sprintf(str_Irms,"%.3f",Irms);
        sprintf(str_P,"%.3f",P);
        sprintf(str_Q,"%.3f",Q);
        sprintf(str_S,"%.3f",S);
        sprintf(str_PF,"%1.3f",PF);
        
        snprintf(send,sizeof(send),"%s,%s,%s,%s,%s,%s\n",str_Vrms,str_Irms,str_P,str_Q,str_S,str_PF);
//        strcat(send,str_Vrms);
//        strcat(send,",");
//        strcat(send,str_Irms);
//        strcat(send,",");
//        strcat(send,str_P);
//        strcat(send,",");
//        strcat(send,str_Q);
//        strcat(send,",");
//        strcat(send,str_S);
//        strcat(send,",");
//        strcat(send,str_PF);
//        strcat(send,";\n");
//        
         //LCD Display
        switch(screen)
        {
            case 0:
                sprintf(line1_format,"%s"," MCP3910 PIC24  ");
                sprintf(line2_format,"%s","Evaluation board");
                break;
            case 1:
                sprintf(line1_format," Channel = %d ", ch);
                sprintf(line2_format," MCP = %d       ", channel);
                break;
            case 2: 
                sprintf(line1_format," Vrms = %.3f  ", Vrms);
                sprintf(line2_format," Irms = %.3f  ", Irms);
                break;
            case 3:
                sprintf(line1_format," P = %.3f  ", P);
                sprintf(line2_format," Q = %.3f  ", Q);
                break;
            case 4:
                sprintf(line1_format," S = %.3f  ", S);
                sprintf(line2_format," PF = %.3f ", PF);
                break; 
        }
        
        send_text_format();
//        
//        sprintf(line1_format,"%d",RCON);
//        send_text_format();
//        //send_UART("24.323,5.300,50.687,60.567,34.898,0.987");
        send_UART(send);
        delay_ms(500);
    }
    return 0;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
