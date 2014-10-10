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
/* User Level #define Macros                                                  */
/******************************************************************************/

/* Baud rate - 19200 is probably the maximum */
#define     BAUD        9200
#define SERIAL_BIT_TIME 1000000/BAUD
#define SERIAL_US_A_BIT SERIAL_BIT_TIME-15     // Using XC8 in free mode, 15uS
                                               //is how long the loop takes once.

/* Microcontroller Freqs*/
#define _XTAL_FREQ      8000000L

/* I/O Pins */
#define     SDI         RB7
#define     SCK         RB6
#define     nSEL        RC5
#define     nIRQ        RA2
#define     FSK         RC4

#define     BTN         RB4     // Button
#define     INP         RC3     // Input
#define     LED         RC6     // LED

#define     SENS        RC3     // Sensor



/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

/**
 * Set Oscillator speed high or low
 * @param speed 0 - 31KHz (low speed), >0 - 8MHz (high speed)
 */
void ConfigureOscillator(uint8_t speed);

/**
 * Transmit data
 * @param TxBuff 8 byte data buffer containing data to transmit
 */
void TXData(uint8_t *TxBuff);

/** 
 * Generate a new address and write it to EEPROM and data buffer
 * @param txBuff 8 byte buffer that address will be written to.
 * @param type type of sensor
 * @return new 12bit sensor address
 */
uint16_t GenAddr(uint8_t *txBuff, bool type );

/** 
 * Retreive store address from EEPROM and return it in buffer
 * @param txBuff 8 byte buffer that address will be written to.
 * @return 12bit sensor address
 */
uint16_t GetAddr(uint8_t *TxBuff);

/**
 * Generates new address and transmits pairing request
 * @param txBuff 8 byte buffer
 * @param mType sensor type
 * @param ipu Impulses per unit
 */
void doPair(uint8_t *txBuff, uint8_t mType, uint32_t ipu);

/**
 * Send out txBuff 30 times with pairing flag set.
 * @param txBuff
 */
void PairTX(uint8_t *txBuff);
/**
 * Set up peripherals and I/O ports
 */
void InitApp(void);                             // I/O and Peripheral Initialization
/**
 * Configure radio parameters.
 */
void InitRadio(void);                           // Inits radio settings

/**
 * Power up TX stages.
 */
void StartTX(void);
/**
 * Power down TX stages.
 */
void StopTX(void);                              // Power down TX
/**
 * Send control messages to radio.
 * @param tosend message to send to radio
 */
void SendCTRL( unsigned int tosend);
/**
 * Send a byte with manchester encoding.
 * @param data byte to send
 */
void SendManFSK( unsigned char data);
/**
 * Send a byte as is (without manchester encoding).
 * @param data byte to send
 */
void SendFSK(unsigned char data);
/**
 * Send a bit for FSK.
 * @param data bit to send
 */
void Sendbit(unsigned char data);
/**
 * Wait for radio to finish sending bit.
 */
void WaitFSK(void);
/**
 * Write data to EEPROM at address
 * @param address address to write to
 * @param data data to write
 */
void EEPROM_putc(uint8_t address, uint8_t data); // Write to EEPROM
/**
 * Read data from EEPROM at address
 * @param address address to read from
 * @return byte in EEPROM
 */
uint8_t EEPROM_getc(uint8_t address);            // Read from EEPROM

/**
 * SoftUSART character output
 * @param chr byte to write to serial port
 */
void _Soft_USART_Write(uint8_t chr);
/**
 * SoftUSART HEX output
 * @param data byte to convert to hex to write to serial port
 */
void _Soft_USART_Write_Hex(unsigned char data);
/**
 * SoftUSART write string
 * @param s string to write to serial port
 */
void _Soft_USART_Writes(const char * s);

/******************************************************************************/
/* User Structures                                                            */
/******************************************************************************/


typedef union {
    struct {
        /** high nibble of 12bit device address */
        unsigned int address_high   :4;
        unsigned int                :2;
        unsigned int data_sensor    :1;
        /** pairing mode indicator */
        unsigned int pair_request   :1;
        /** low byte of 12bit device address */
        unsigned int address_low    :8; 
        unsigned int                :8;
        /** type of sensor
         2 = electricity
         3 = gas
         4 = water
         */
        unsigned int sensor_type    :8;
        /** MSB of 32 impulse value */
        unsigned int value_high     :8; 
        unsigned int value_midh     :8;
        unsigned int value_midl     :8;
        /** LSB of 32 impulse value */
        unsigned int value_low      :8; 
    };
    /** unsigned char representation of data packet*/
    unsigned char bytes[8];
} DataPacket_t;