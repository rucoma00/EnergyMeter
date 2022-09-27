/******************************************************************************

File Name:           IIC_LCD.h
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
#include "p24FJ256GA110.h"

#define Slave_address 	0x7C
#define Control_command	0x80
#define Control_data	0x60

#define SDA		LATAbits.LATA3
#define	SCL		LATAbits.LATA2

#define SDAr		PORTAbits.RA3
#define SDA_DIR         TRISAbits.TRISA3
#define	SCLr		PORTAbits.RA2
#define SCL_DIR         TRISAbits.TRISA2

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void init(void);
void init_LCD(void);

extern char line1_format[];
extern char line2_format[];
extern char disp_Current[];
extern char disp_Voltage[];
extern char disp_PF[];
extern char disp_Power[];
extern char *pointer;
extern unsigned char tx_buf[10];
//>>>>>>>>>>>>>>>>>>>>>>>>>>
extern char b, b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14, b15, b16, b17, ack1, ack2, rdy;
