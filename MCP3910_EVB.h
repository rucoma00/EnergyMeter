//#include "p24FJ256GA110.h"
#include "IIC_LCD.h"
#define FCY 14745600UL
#include <libpic30.h>
#include <math.h>



#define u8	unsigned char				// Unsigned 8bit memory type(0 to 255)
#define s8	char						// Signed 8bit memory type(-128 to 127)
#define u16	unsigned int				// Unsigned 16bit memory type(0 to 65,535)
#define s16	int							// Signed 16bit memory type(-32,768 to 32,767)
#define u24	unsigned short long*		// Unsigned 24bit memory type(0 to 16,777,215)
#define s24	short long*					// Signed 24bit memory type(-8,388,608 to 8,388,608)
#define u32 unsigned long*				// Unsigned 32bit memory type(0 to 4,294,967,295)
#define s32	long*						// Signed 32bit memory type(-2,147,483,648 to 2,147,483,648)
#define bool unsigned char

#define TRUE 1
#define FALSE 0
#define ON 1
#define OFF 0

typedef union
{
	struct
	{
		unsigned char LB;
		unsigned char HB;
	}byte;
	unsigned int word_val;
} WDATA;

typedef union
{ 
    struct
    {
        unsigned char Start_transmision:1;      //Start transmision
        unsigned char Start_cycle:1;            //
        unsigned char Same_OSR:1;
        unsigned char :1;
        unsigned char :1;
        unsigned char :1;
        unsigned char :1;
        unsigned char :1;
    } bits;
    unsigned char     byte_val;    		// BYTE access

} FLAGS;

typedef union
{
	struct
	{
		unsigned char:8;			//this should be 0xA5 if in sync
		unsigned char CNT0:1;
		unsigned char CNT1:1;
		unsigned char BOOST:1;
		unsigned char GAIN0:1;
		unsigned char GAIN1:1;
		unsigned char OSR0:1;
		unsigned char OSR1:1;
		unsigned char:1;
	}bits;
	struct
	{
		unsigned char LB;
		unsigned char HB;
	}byte;
	unsigned int word_val;
} SYNC;

#define _trapISR __attribute__((interrupt,no_auto_psv))

//MCP3901 Reset Pin
#define RESET 	LATAbits.LATA5 
#define DRESET	TRISAbits.TRISA5

// Chip Select
#define CS 	LATAbits.LATA4 
#define DCS	TRISAbits.TRISA4

// Data Ready Pin
#define DR 	LATEbits.LATE9
#define DDR	TRISEbits.TRISE9

#define SCLK    LATFbits.LATF6
#define DSCLK   TRISFbits.TRISF6

#define MOSI    LATFbits.LATF8
#define DMOSI   TRISFbits.TRISF8

#define MISO	LATFbits.LATF7
#define DMISO	TRISFbits.TRISF7

#define MCLK    LATDbits.LATD0
#define DMCLK   TRISDbits.TRISD0

// Modulator 0 Output Pin
#define MOD0 	LATDbits.LATD10 
#define DMOD0	TRISDbits.TRISD10

// Modulator 1 Output Pin
#define MOD1 	LATDbits.LATD11 
#define DMOD1	TRISDbits.TRISD11

//Green LED
#define LEDG 	LATAbits.LATA9      // LED 4 en la placa
#define DLEDG	TRISAbits.TRISA9 

//Red LED
#define LEDR 	LATAbits.LATA10     // LED 3 en la placa
#define DLEDR	TRISAbits.TRISA10 

//UART1 TX
#define TX1 	LATFbits.LATF5 
#define DTX1	TRISFbits.TRISF5

//UART1 RX
#define RX1 	LATFbits.LATF4
#define DRX1	TRISFbits.TRISF4

//test out
#define DRG0	TRISGbits.TRISG0
#define RG0	LATGbits.LATG0

//Pushbutton Switches
#define SW1   PORTBbits.RB8 
#define SW2   PORTBbits.RB9
#define SW3   PORTBbits.RB10
#define CN19   PORTDbits.RD13   // BUTTON 4 en la placa
#define CN16   PORTDbits.RD7    // BUTTON 2 en la placa
#define CN15   PORTDbits.RD6    // BUTTON 3 en la placa
#define DSW1   TRISBbits.TRISB8
#define DSW2   TRISBbits.TRISB9
#define DSW3   TRISBbits.TRISB10
#define DCN19  TRISDbits.TRISD13    // BUTTON 4 en la placa
#define DCN16  TRISDbits.TRISD7     // BUTTON 2 en la placa
#define DCN15  TRISDbits.TRISD6     // BUTTON 3 en la placa
//Serial Enable
#define EN232   LATCbits.LATC13 
#define DEN232   TRISCbits.TRISC13

#define OSW1   ODCBbits.ODB8
#define OSW2   ODCBbits.ODB9
#define OSW3   ODCBbits.ODB10

//>>> MCP3910 register map >>>>>>
#define phaseH  	0x07
#define phaseL  	0x08
#define gain 		0x09
#define statusH 	0x0A
#define statusL 	0x0B
#define configH 	0x0C
#define configL 	0x0D
#define CH0_OFF_H	0x0E
#define CH0_OFF_M	0x0F
#define CH0_OFF_L	0x10
#define CH0_GAIN_H	0x11
#define CH0_GAIN_M	0x12
#define CH0_GAIN_L	0x13
#define CH1_OFF_H	0x14
#define CH1_OFF_M	0x15
#define CH1_OFF_L	0x16
#define CH1_GAIN_H	0x17
#define CH1_GAIN_M	0x18
#define CH1_GAIN_L	0x19
#define V_REF		0x1A
//>>> IO mapping >>>>>>>>>>>>>>>>

        
        //PPS Outputs
        #define NULL_IO		0
        #define C1OUT_IO	1
        #define C2OUT_IO	2
        #define U1TX_IO		3
        #define U1RTS_IO	4
        #define U2TX_IO		5
        #define U2RTS_IO	6
        #define SDO1_IO		7
        #define SCK1OUT_IO	8
        #define SS1OUT_IO	9
        #define SDO2_IO		10
        #define SCK2OUT_IO	11
        #define SS2OUT_IO	12
        #define OC1_IO		18
        #define OC2_IO		19
        #define OC3_IO		20
        #define OC4_IO		21
        #define OC5_IO		22
        #define OC6_IO      23
        #define OC7_IO      24
        #define OC8_IO      25
        #define U3TX_IO     28
        #define U3RTS_IO    29
        #define U4TX_IO     30
        #define U4RTS_IO    31
        #define SDO3_IO     32
        #define SCK3OUT_IO  33
        #define SS3OUT_IO   34
        #define OC9_IO      35
  
  
#define CS_delay		3
#define BUFFER_LENGTH 	2048
#define board_ID		"ADM00425"
#define firmware_ID		"1.0.0"

#define max_urxdata 50  //204

//This is the GUI command definition

#define IDENTIFY 1
#define START 2
#define STOP 3
#define WRITE_ALL 4
#define READ_ALL 5
#define WRITE_SINGLE 6
#define READ_SINGLE 7
#define FIRMWARE_VERSION 8
#define TEMPO_ADJ 9
#define CH_WRITE 10
#define CH_READ 11

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//global variables
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

extern signed char voltage_msb[BUFFER_LENGTH], voltage_nsb[BUFFER_LENGTH], voltage_lsb[BUFFER_LENGTH], current_msb[BUFFER_LENGTH], current_nsb[BUFFER_LENGTH], current_lsb[BUFFER_LENGTH];
extern unsigned int counter_buffer, counter_tx, count_rx, rx_function, register_write_cnt;
extern unsigned char tx_buf[10], tx_bufc[3], types, rxcounter, char_to_int[3];
extern unsigned long internal_registers[26];
extern 	unsigned char receive_counter, urxdata, decode_counter, decode_buffer[10], rxbuffer[max_urxdata], start_acquisition, rx_data, write_reg_count;
extern unsigned int Zero_words_counter;
extern unsigned char command_received;

extern FLAGS Flags1;
extern WDATA Data_decoding;
extern unsigned char channel, Config_Neutral, Config_PhaseA, Config_PhaseB, Config_PhaseC;

extern volatile unsigned int screen, ch, newBUTTON2, newLED4, newBUTTON3, newLED3,
        oldBUTTON2, oldBUTTON3, newBUTTON4, start_flag_state;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
/*****************************************************
* Local Prototypes
*****************************************************/
void hexdec_long(unsigned long);
void hexdec_char(unsigned char);
unsigned char Read3910(char addres);
void Read_Internal_Registers(void);
void Read_Internal_Registers2(void);
void send_UART(char *c);
void send_char(char d);
void Init(void);
void ioMap(void);
void send_internal_registers(void);
void send_internal_registers2(void);
void set_timer(void);
void write_timer(void);

void Init_LCD(void);
void send_text_format(void);
void hexdec_long(unsigned long);
void IIC_hexdec_long( unsigned long in, char *point, int length );
void receive_data(void);
void transmit_data(void);
void detect_jumpers(void);


