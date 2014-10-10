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

#include <htc.h>           /* Global Header File */
#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>       /* For true/false definition */


#include "user.h"          /* User funct/params, such as InitApp */
 

/******************************************************************************/
/* User Global Variable Declaration                                           */
/******************************************************************************/
DataPacket_t datapacket; // Our data packet to work with
const uint8_t mType=4;   // Meter type (water)
const uint32_t mIpu=1;   // Number of impulses per unit
uint8_t DHTData[5];      // Data from DHT22
uint8_t gcnt;            // global counter for asm code

// Function that interrogates DHT22
void readDHT(void);


/******************************************************************************/
/* Feel free to change getValue to suit your purposes.                        */
/* This example just sends a 1 if the pisn are shorted, and a 0 if they're not*/
/******************************************************************************/
void getValue()
{
    uint16_t Humi;

    readDHT();
    Humi=(DHTData[0]<<8)+DHTData[1];
    Humi=Humi/10;   // Throw away fraction

    datapacket.value_high=Humi & 0xFF;      // Humidity
    datapacket.value_midh=DHTData[2];       // Temperature high
    datapacket.value_midl=DHTData[3];       // Temperature low
    datapacket.value_low=(
            datapacket.value_high+datapacket.value_midh+datapacket.value_midl
            ) & 0xff;  // checksum

}

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

void main(void)
{
    uint8_t cnt,debounce;
    
    
    ConfigureOscillator(1);         // high speed
    LED=0;
    InitApp();                      // Init I/O and peripherals
    InitRadio();                    // Setup radio

    for (cnt=0;cnt<8;cnt++) {       // init data packet
        datapacket.bytes[cnt]=0;
    }
    GetAddr(datapacket.bytes);      // Get our address from EEPROM
    datapacket.sensor_type=mType;   // Set sensor type

    // Delay two seconds to allow things to get started up
    for (cnt=0;cnt<20;cnt++) {
        __delay_ms(100);
    }

    // Main code loop
    while (1)
    {
        ConfigureOscillator(1);         // high speed

        LED=1;                          // LED on
        getValue();                     // Get the data value
        TXData(datapacket.bytes);       // Transmit the data
        LED=0;                          // LED off

        ConfigureOscillator(0);         // low speed
        
        for (cnt=0;cnt<64;cnt++)        // Idle loop - approx 80 seconds
        {
            __delay_ms(4);              // at low speed, it's a lot longer than
                                        // might think! (4 = ~1.25secs)
            // debounce switch
            if (!BTN) {debounce++; } else {debounce=0;}

            // pair if button held for ~5 seconds.
            if (debounce>4) {
                ConfigureOscillator(1); // high speed
                doPair(datapacket.bytes, mType,mIpu);// generate new address and pair
                debounce=0;
                ConfigureOscillator(0); // low speed
                }
            }
        }
}

// Reads are done by changing tristate register
void readDHT(void)
{
    uint8_t i,k;
    
    TRISC3=0;      // RC3 = output
    SENS=0;         // RC3 pulled low
    __delay_ms(30); // pull output line low 30ms
    TRISC3=1;      // data line allowed to float high
    __delay_us(35); // pull output line high 40uS

    // Look for 80uS low and 80 uS high sensor response
    // Really should be a timeout here
    while(!SENS);
    while(SENS);

    // Nab the data
    for (k=0;k<5;k++)       // k is byte number
    {
        DHTData[k]=0;
        for (i=0;i<8;i++)   // i is bit number
        {
            DHTData[k]<<=1;
            gcnt=0;

            while (!SENS);      // wait for low to high
#asm
            banksel (_gcnt)
DHTloop:
            BTFSS PORTC, 0x3    //while(SENS) {   // Measure length of high to low
            GOTO  DHTexit
            INCF  (_gcnt)       // gcnt++;
            GOTO  DHTloop
DHTexit:
#endasm

            if (gcnt>16) {
              DHTData[k] |=1;   // bit was 1.
            }
        }
    }
}