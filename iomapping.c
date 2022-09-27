/*****************************************************************************
 *
 * Basic Explorer 16 I/O Mapping functionality for PPS peripherals.
 *
 *****************************************************************************
 * FileName:        iomapping.c
 * Dependencies:    system.h
 * Processor:       PIC24
 * Compiler:       	MPLAB C30
 * Linker:          MPLAB LINK30
 * Company:         Microchip Technology Incorporated
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the "Company") is intended and supplied to you, the Company's
 * customer, for use solely and exclusively with products manufactured
 * by the Company. 
 *
 * The software is owned by the Company and/or its supplier, and is 
 * protected under applicable copyright laws. All rights are reserved. 
 * Any use in violation of the foregoing restrictions may subject the 
 * user to criminal sanctions under applicable laws, as well as to 
 * civil liability for the breach of the terms and conditions of this 
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES, 
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT, 
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR 
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Brant Ivey			3/14/06    Modified for PIC24FJ64GA004 family
 *****************************************************************************/

#include "MCP3910_EVB.h"

void ioMap(){
  
//            __builtin_write_OSCCONL(OSCCON & 0xBF);
            
        //OUTPUTS *********************
            //RP17 = U1TX
            RPOR8bits.RP17R = U1TX_IO;

            //RP11 = OC1
            RPOR5bits.RP11R = OC1_IO;   //set the pin as clock output
//            RPOR5bits.RP11R = SCK1OUT_IO;   //set the pin as clock output

        //INPUTS **********************
            // U1RX = RP10
            RPINR18bits.U1RXR = 10;

            //SDI1 = RP20
            RPINR20bits.SDI1R = 44;
//            RPINR20bits.SDI1R = 38;

            //RD0 = SCK1
            RPINR20bits.SCK1R = 11;

//            __builtin_write_OSCCONL(OSCCON | 0x40);
}
