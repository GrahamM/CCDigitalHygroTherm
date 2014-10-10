/* Copyright (c) 2014 Graham Murphy <graham@c0redump.net>
*
* Permission to use, copy, modify, and distribute this software for any
* purpose with or without fee is hereby granted, provided that the above
* copyright notice and this permission notice appear in all copies.
*
* THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
* WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
* ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
* WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
* ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
* OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <htc.h>            /* HiTech General Includes */
#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>        /* For true/false definition */

#include "user.h"

/******************************************************************************/
/* High level address functions                                               */
/******************************************************************************/


// Returns new address and writes it to EEPROM
uint16_t GenAddr(uint8_t *txBuff, bool type )
{
    txBuff[0]=TMR1H & 0x0F;  // Mask off high bits
    if (type)  txBuff[0] |=1<<6; // Check if we're a data sensor
    txBuff[1]=TMR1L;
    EEPROM_putc(0x00,txBuff[0]);
    EEPROM_putc(0x01,txBuff[1]);
    return (txBuff[0]<<4)+txBuff[1];
}

// Retrives address and writes it to buffer
uint16_t GetAddr(unsigned char *txBuff)
{
    txBuff[0]=EEPROM_getc(0x00);
    txBuff[1]=EEPROM_getc(0x01);
    return (txBuff[0]<<4)+txBuff[1];
}

/******************************************************************************/
/* high level radio functions                                                 */
/******************************************************************************/
// Transmit the datablock
void TXData(uint8_t *txBuff) {
    uint8_t cnt;

    StartTX();                  // power up the TX
    __delay_us(100);

    SendFSK(0xAA);              // Transmit header
    SendFSK(0x2D);              // Sync word
    SendFSK(0xD4);              // Sync word

    for (cnt=0;cnt<9;cnt++) {   // data packet
        SendManFSK(txBuff[cnt]);
    }
    WaitFSK();                  // wait for last bit to be transmitted
    StopTX();                   // Turn off TX
}

void doPair(uint8_t *txBuff, uint8_t mType, uint32_t ipu)
{
                GenAddr(txBuff,1);  // Generate/populate a data sensor address
                txBuff[2]=0x00;     // blank
                txBuff[3]=mType;    // device type
                txBuff[4]=(ipu &0xff000000UL) >>24; // impulses per unit (ipu) MSB
                txBuff[5]=(ipu &0x00ff0000UL) >>16; // impulses per unit (ipu)
                txBuff[6]=(ipu &0x0000ff00UL) >>8;  // impulses per unit (ipu)
                txBuff[7]=(ipu &0x000000ffUL);      // impulses per unit (ipu) LSB
                PairTX(txBuff);                     // Send 'em
}

void PairTX(uint8_t *txBuff) {
    uint8_t cnt;
    txBuff[0] |=1<<7;       // Set the pairing bit
    for (cnt=0;cnt<29;cnt++) {
        TXData(txBuff);
        LED=1-LED;          // Toggle LED
        __delay_ms(400);     // pause between packets
    }
    
    txBuff[0] &= ~(1<<7);   // Unset the pairing bit
    LED=0;                  // Turn LED off
}


/******************************************************************************/
/* Mid and low level radio functions                                          */
/******************************************************************************/
// Init settings for TXC101
void InitRadio(void)
{
    SendCTRL(0xCC00);       // read "status" - actually throw it away
    SendCTRL(0x88F0);       // config band/capacitance/bandwith
    SendCTRL(0xC857);       // data rate = 3918 bits/sec
    SendCTRL(0xD240);       // PLL Bandwidth set
    SendCTRL(0xA634);       // Freq set
    SendCTRL(0xC220);       // Low battery & bit sync
    SendCTRL(0xC001);       // TX off
}

// Start the RF output
void StartTX(void)
{
    SendCTRL(0xC039);        // turn on osc, synth,PA
}

// Stop the RF output
void StopTX(void)
{
    SendCTRL(0xC001);       // power down TX
    SendCTRL(0xC400);       // go to sleep
}

// Send control info to TXC101
void SendCTRL(uint16_t tosend) {
    uint8_t cnt=16;
    nSEL=0; // select the chip
    SCK=0;
    while(cnt--)
    {
        if (tosend & 0x8000) { SDI=1; } else {SDI=0;}
        SCK=1;
        __delay_us(1);
        SCK=0;
        tosend<<=1;
    }
    nSEL=1;
}

// Manchester encode and send one byte
void SendManFSK(uint8_t data) {
    uint8_t cnt=8;
    while (cnt--)
    {
        if (data & 0x80) {
            Sendbit(1);
            Sendbit(0);
        } else {
            Sendbit(0);
            Sendbit(1);
        }
        data <<=1;
    }
}

// Send raw FSK byte - not manchester encoded
void SendFSK(uint8_t data)
{
    uint8_t cnt=8;
    while (cnt--)
    {
        if (data & 0x80) {
            Sendbit(1);
        } else {
            Sendbit(0);
        }
        data <<=1;
    }
}

//  Sends one bit of data
void Sendbit(uint8_t data) {
#asm
    sla:
        BTFSS 0x5, 0x2      //while(!nIRQ);
        GOTO sla
    slb:
        BTFSC 0x5, 0x2      //while(nIRQ);
        GOTO slb
#endasm
        FSK=data;
}
// Waits for TXC101 to indicate it's ready for next bit
void WaitFSK(void)
{
#asm
    wla:
        BTFSS 0x5, 0x2      //while(!nIRQ);
        GOTO wla
    wlb:
        BTFSC 0x5, 0x2      //while(nIRQ);
        GOTO wlb
#endasm
}


/******************************************************************************/
/* EEPROM routines from http://www.mcuexamples.com/PIC-internal-EEPROM.php    */
/******************************************************************************/
void EEPROM_putc(uint8_t address, uint8_t data)
{
    uint8_t INTCON_SAVE;

    EEADR  = address;
    EEDATA = data;

    EECON1bits.EEPGD= 0;   
    EECON1bits.WREN = 1;   

    INTCON_SAVE=INTCON;     
    INTCON=0;               // Disable interrupts, Next two lines SHOULD run without interrupts

    EECON2=0x55;            // Required sequence for write to internal EEPROM
    EECON2=0xAA;            // Required sequence for write to internal EEPROM

    EECON1bits.WR=1;        // begin write to internal EEPROM
    INTCON=INTCON_SAVE;     

    NOP();

    while (PIR2bits.EEIF==0)
    {
        NOP();
    }

    EECON1bits.WREN=0;      // Disable writes to EEPROM on write complete (EEIF flag on set PIR2 )
    PIR2bits.EEIF=0;        //Clear EEPROM write complete flag. (must be cleared in software. So we do it here)

}

// This function reads data from address given in internal EEPROM of PIC
uint8_t EEPROM_getc(uint8_t address)
{
    EEADR=address;
    EECON1bits.EEPGD= 0;    // 0 = Access data EEPROM memory
    EECON1bits.RD   = 1;    
    return EEDATA;          // return data
}


/******************************************************************************/
/* Peripheral setup                                                           */
/******************************************************************************/
void InitApp(void)
{
    /* Setup analog functionality and port direction */
    ANSEL=0x00;     // No analog set
    ANSELH=0x00;    // No analog set
    TRISA=0xFC;     // NIRQ + Serial
    TRISB=0x3F;     // SDI, SCK RB4 is button
    TRISC=0x8F;     // NSEL, FSK and LED
    WPUB4=1;        // Weak pullup on button
    nRABPU=0;       // Turn weak pullups on

    /* Initialize peripherals */
    nSEL=1;
    SDI=1;
    SCK=0;
    FSK=0;

    T1CON=0x05;             // timer 1 on, using FOSC/4 - for address

    RA0=1;                  // idle soft serial
}

// Osc switcher
void ConfigureOscillator(uint8_t speed)
{
    if (speed) {
        OSCCON=0x70;        // Switches clock to 8MHz
    } else {
        OSCCON=0x00;        // Switches clock to 31kHz
    }
}
/******************************************************************************/
/* Softserial code                                                            */
/******************************************************************************/
// Write out on RA0 - ICSPDAT
void _Soft_USART_Write(uint8_t chr) {
uint8_t i;
uint16_t data;

    data = (chr << 1);      
    
    for (i=0;i<9;i++) {     
        RA0=(1 & data);
        data>>=1;           
        __delay_us(SERIAL_US_A_BIT);    
    }
    RA0=1;                  // stop bit
   __delay_us(SERIAL_BIT_TIME);         
}

void _Soft_USART_Write_Hex(unsigned char data)
{
    uint8_t tmp;
    tmp=48 + (data>>4);
    if (tmp>57) tmp=tmp+7;
    _Soft_USART_Write(tmp);
    tmp=48 + (data&0x0F);
    if (tmp>57) tmp=tmp+7;
    _Soft_USART_Write(tmp);
}

void _Soft_USART_Writes(const char * s)
{
 while (*s)   _Soft_USART_Write(*s++);
}
