#include "MCP3910_EVB.h"
//>>>>>>> Read the registers from MCP3910

unsigned char Read3910(char address)
{
        char i, data;
        address=((address<<1)+1)|0x20;
        CS=0;
        i=SPI1BUF;
        SPI1BUF=address;
        while (!SPI1STATbits.SPIRBF);
        i=SPI1BUF;
        SPI1BUF=0;
        while (!SPI1STATbits.SPIRBF);
        data=SPI1BUF;
        CS=1;
        return(data);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//  Reads the all the registers from MCP3910

void Read_Internal_Registers(void)
{
	unsigned char i;
	for(i=0; i<23; i++)
	{
		internal_registers[i]=0;
	}
        i = Config_Neutral>>5;
        i &= 0x03;                  //Extracting the OSR information for Neutral channel, which should be the same on all channels
        switch (i)
        {
            case 0:
                internal_registers[4] |= 0b00000000000000000010000000000000;
                break;
            case 1:
                internal_registers[4] |= 0b00000000000000000100000000000000;
                break;
            case 2:
                internal_registers[4] |= 0b00000000000000000110000000000000;
                break;
            case 3:
                internal_registers[4] |= 0b00000000000000001000000000000000;
                break;
        }

        i = Config_Neutral>>3;
        i &= 0x03;                  //Extracting the GAIN information for Neutral channel
        switch (i)
        {
            case 0:
                break;
            case 1:
                internal_registers[2] |= 0b00000000000000000000000000000011;
                break;
            case 2:
                internal_registers[2] |= 0b00000000000000000000000000000100;
                break;
            case 3:
                internal_registers[2] |= 0b00000000000000000000000000000101;
                break;
        }
        i = Config_PhaseA>>3;
        i &= 0x03;                  //Extracting the GAIN information for Phase A channel
        switch (i)
        {
            case 0:
                break;
            case 1:
                internal_registers[2] |= 0b00000000000000000000000011000000;
                break;
            case 2:
                internal_registers[2] |= 0b00000000000000000000000100000000;
                break;
            case 3:
                internal_registers[2] |= 0b00000000000000000000000101000000;
                break;
        }
        i = Config_PhaseB>>3;
        i &= 0x03;                  //Extracting the GAIN information for Phase B channel
        switch (i)
        {
            case 0:
                break;
            case 1:
                internal_registers[2] |= 0b00000000000000000011000000000000;
                break;
            case 2:
                internal_registers[2] |= 0b00000000000000000100000000000000;
                break;
            case 3:
                internal_registers[2] |= 0b00000000000000000101000000000000;
                break;
        }
        i = Config_PhaseC>>3;
        i &= 0x03;                  //Extracting the GAIN information for Phase C channel
        switch (i)
        {
            case 0:
                break;
            case 1:
                internal_registers[2] |= 0b00000000000011000000000000000000;
                break;
            case 2:
                internal_registers[2] |= 0b00000000000100000000000000000000;
                break;
            case 3:
                internal_registers[2] |= 0b00000000000101000000000000000000;
                break;
        }

        if (Config_Neutral & 0x04) internal_registers[4] |= 0b00000000000010000000000000000000;      //Extracting the BOOST information for Neutral channel, which should be the same on all channels
        internal_registers[22] = 0xA50000;  //Registers unlocked
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

//The new protocol
void Read_Internal_Registers2(void)
{
	unsigned char i;
//	for(i=0; i<23; i++)
//	{
//		internal_registers[i]=0;
//	}

        i = Config_Neutral>>5;
        i &= 0x03;                      //Extracting the OSR information for Neutral channel
        internal_registers[0] = i;

        i = Config_PhaseA>>5;
        i &= 0x03;                      //Extracting the OSR information for Phase A channel
        internal_registers[3] = i;

        i = Config_PhaseB>>5;
        i &= 0x03;                      //Extracting the OSR information for Phase B channel
        internal_registers[6] = i;

        i = Config_PhaseC>>5;
        i &= 0x03;                      //Extracting the OSR information for Phase C channel
        internal_registers[9] = i;

        internal_registers[1] = 0;
        if (Config_Neutral & 0x04)
        {
            internal_registers[1] = 1;  //Extracting the BOOST  information for Neutral channel
        }

        i = Config_Neutral>>3;
        i &= 0x03;                      //Extracting the GAIN information for Neutral channel
        internal_registers[2] = i;

        internal_registers[4] = 0;
        if (Config_PhaseA & 0x04)
        {
            internal_registers[4] = 1;  //Extracting the BOOST  information for Phase A channel
        }

        i = Config_PhaseA>>3;
        i &= 0x03;                      //Extracting the GAIN information for Phase A channel
        internal_registers[5] = i;

        internal_registers[7] = 0;
        if (Config_PhaseB & 0x04)
        {
            internal_registers[7] = 1;  //Extracting the BOOST  information for Phase B channel
        }

        i = Config_PhaseB>>3;
        i &= 0x03;                      //Extracting the GAIN information for Phase B channel
        internal_registers[8] = i;

        internal_registers[10] = 0;
        if (Config_PhaseC & 0x04)
        {
            internal_registers[10] = 1;  //Extracting the BOOST  information for Phase C channel
        }

        i = Config_PhaseC>>3;
        i &= 0x03;                      //Extracting the GAIN information for Phase C channel
        internal_registers[11] = i;

        internal_registers[22] = 0xA50000;  //Registers unlocked
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void detect_jumpers(void)
{
        SPI1STATbits.SPIEN=1;       //enable SPI module
        
        DMCLK=1;
        MCLK=1;
        OC1CON1bits.OCM = 0b101;                        // This selects and starts the Edge Aligned PWM mode

        for (;;)
        {
            while (!SPI1STATbits.SPIRBF);                   //wait for data on SPI
            Data_decoding.word_val = SPI1BUF;		//Word 0
            if (Data_decoding.byte.LB==0xA5)		//Read from SPI
            {
                    if (Zero_words_counter>4)			//At least 5 zero words
                    {
                            Config_Neutral = Data_decoding.byte.HB;
                            Zero_words_counter=0;
                            break;
                    }
            }
            else Zero_words_counter++;
        }

        OC1CON1bits.OCM = 0b000;
        SPI1STATbits.SPIEN=0;       //disable SPI module
        DMCLK=0;
        MCLK=0;
        __delay_us(1);
        MCLK=1;
        __delay_us(15);             //Keeps clock high to reset the ADCs

        RPINR20bits.SDI1R = 40;     //Switch the SDI input to Phase A
        SPI1STATbits.SPIEN=1;       //enable SPI module

        DMCLK=1;
        MCLK=1;
        OC1CON1bits.OCM = 0b101;                        // This selects and starts the Edge Aligned PWM mode

        for (;;)
        {
            while (!SPI1STATbits.SPIRBF);                   //wait for data on SPI
            Data_decoding.word_val = SPI1BUF;		//Word 0
            if (Data_decoding.byte.LB==0xA5)		//Read from SPI
            {
                    if (Zero_words_counter>4)			//At least 5 zero words
                    {
                            Config_PhaseA = Data_decoding.byte.HB;
                            Zero_words_counter=0;
                            break;
                    }
            }
            else Zero_words_counter++;
        }

        OC1CON1bits.OCM = 0b000;
        SPI1STATbits.SPIEN=0;       //disable SPI module
        DMCLK=0;
        MCLK=0;
        __delay_us(1);
        MCLK=1;
        __delay_us(15);             //Keeps clock high to reset the ADCs

        RPINR20bits.SDI1R = 39;     //Switch the SDI input to Phase B
        SPI1STATbits.SPIEN=1;       //enable SPI module

        DMCLK=1;
        MCLK=1;
        OC1CON1bits.OCM = 0b101;                        // This selects and starts the Edge Aligned PWM mode

        for(;;)
        {
            while (!SPI1STATbits.SPIRBF);                   //wait for data on SPI
            Data_decoding.word_val = SPI1BUF;		//Word 0
            if (Data_decoding.byte.LB==0xA5)		//Read from SPI
            {
                    if (Zero_words_counter>4)			//At least 5 zero words
                    {
                            Config_PhaseB = Data_decoding.byte.HB;
                            Zero_words_counter=0;
                            break;
                    }
            }
            else Zero_words_counter++;
        }

        OC1CON1bits.OCM = 0b000;
        SPI1STATbits.SPIEN=0;       //disable SPI module
        DMCLK=0;
        MCLK=0;
        __delay_us(1);
        MCLK=1;
        __delay_us(15);             //Keeps clock high to reset the ADCs

        RPINR20bits.SDI1R = 38;     //Switch the SDI input to Phase C
        SPI1STATbits.SPIEN=1;       //enable SPI module

        DMCLK=1;
        MCLK=1;
        OC1CON1bits.OCM = 0b101;                        // This selects and starts the Edge Aligned PWM mode

        for(;;)
        {
            while (!SPI1STATbits.SPIRBF);                   //wait for data on SPI
            Data_decoding.word_val = SPI1BUF;		//Word 0
            if (Data_decoding.byte.LB==0xA5)		//Read from SPI
            {
                    if (Zero_words_counter>4)			//At least 5 zero words
                    {
                            Config_PhaseC = Data_decoding.byte.HB;
                            Zero_words_counter=0;
                            break;
                    }
            }
            else Zero_words_counter++;
        }

        OC1CON1bits.OCM = 0b000;
        SPI1STATbits.SPIEN=0;                   //disable SPI module
        DMCLK=0;
        MCLK=0;
        __delay_us(1);
        MCLK=1;
        __delay_us(15);				//Keeps clock high to reset the ADCs

        RPINR20bits.SDI1R = 44;                 //Switch the SDI input to Neutral
	if (SPI1STATbits.SPIROV)
	{
		SPI1STATbits.SPIROV	= 0;			//Clear overflow
	}
        IFS0bits.SPI1IF=0;
        
        Read_Internal_Registers2();
}
