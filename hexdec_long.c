/******************************************************************************

File Name:           hexdec_long.c
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


	
void hexdec_long( unsigned long count )
{	
	char ones;
	char tens;
	char hundreds;
	char thousands;
	char thousand10s;
	char thousand100s;
    char mill;
    char mill10s;
    char mill100s;
    char bills;
    
	bills=0;
	mill100s=0;
	mill10s=0;			
 	mill=0;
 	thousand100s=0;
 	thousand10s=0;
	thousands=0;
	hundreds=0;
	tens  = 0;						
	ones = 0;

		while ( count >= 1000000000 )
	{
		count -= 1000000000;			// subtract 100000
		
		bills++;					// increment 10thousands
	}

		while ( count >= 100000000 )
	{
		count -= 100000000;			// subtract 100000
		
		mill100s++;					// increment 10thousands
	}

		while ( count >= 10000000 )
	{
		count -= 10000000;			// subtract 100000
		
		mill10s++;					// increment 10thousands
	}

		while ( count >= 1000000 )
	{
		count -= 1000000;			// subtract 100000
		
		mill++;					// increment 10thousands
	}
	
		while ( count >= 100000 )
	{
		count -= 100000;			// subtract 100000
		
		thousand100s++;					// increment 10thousands
	}
	
	while ( count >= 10000 )
	{
		count -= 10000;			// subtract 10000
		
		thousand10s++;					// increment 10thousands
	}


		while ( count >= 1000 )
		{
			count -= 1000;			// subtract 1000
		
			thousands++;					// increment thousands
		}

			while ( count >= 100 )
			{
				count -= 100;			// subtract 100
		
				hundreds++;					// increment hundreds
			}


				while ( count >= 10 )
					{
						count -= 10;			// subtract 10
		
						tens++;					// increment tens
					}

						ones = count;					// remaining count equals ones
				
				    tx_buf[0]=bills+0x30;
				    tx_buf[1]=mill100s+0x30;
                    tx_buf[2]=mill10s+0x30;
					tx_buf[3]=mill+0x30;
					tx_buf[4]=thousand100s+0x30;
					tx_buf[5]=thousand10s+0x30;
					tx_buf[6]=thousands+0x30;
					tx_buf[7]=hundreds+0x30;
					tx_buf[8]=tens+0x30;
					tx_buf[9]=ones+0x30;					
}				
