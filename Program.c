// Author: Harry Moss
// Date: 05.05.2022

// All relevant includes
#include <stdio.h>
#include <cdefBF706.h>
#include <defBF706.h>
#include <math.h>
#include "stdfix.h"
#include <sys/platform.h>
#include "adi_initialize.h"
#include <services/int/adi_int.h>
#include <stdint.h>
#include <stdbool.h>
#include <services/gpio/adi_gpio.h>


// C definitions
#define codec_in ((volatile long fract *)0x2004D0C4)
#define codec_out ((volatile long fract *)0x2004D040)

#define BUFFER_SIZE 2                    // Size of buffer to transmit
#define REV_ROOM_SIZE 25000              // Reverb room size

long fract XIN[BUFFER_SIZE];                // Input buffer
long fract YOUT[BUFFER_SIZE];               // Output buffer



// GAIN
unsigned gainCoeff = 3;                 // Gain effect coefficient
char gainPWR = 0;                       // Gain ON/OFF

// DISTORTION
long fract pThres = 0.0007;             // Distortion upper bound
long fract nThres = -0.0007;            // Distortion lower bound
char heavyDistPWR = 0;                  // Distortion heavy ON/OFF
char crunchDistPWR = 0;                 // Distortion crunch ON/OFF

// ECHO
unsigned filterLengthEcho = 2000;           // Echo buffer length
long fract decayEcho = 0.8;             // Echo decay
char echoPWR = 0;

// REVERB
long fract decayReverb = 0.75;             // Reverb decay
long fract A0 = 0;                     // Accumulator register
long fract A1 = 0;                     // Delayed feedback averages
long fract A2 = 0;                     // ^^
long fract A3 = 0;                     // ^^
long fract op = 0.25;                  // Operator
long fract i[6][REV_ROOM_SIZE] = {0};          // 6 comb filter buffer
unsigned k = 0;                        // Circular buffer position
char reverbHallPWR = 0;                // Reverb hall ON/OFF
char reverbRoomPWR = 0;                // Reverb room ON/OFF

uint32_t data;                         // GPIO receive data

// Effect definitions
long fract gain (long fract input);
long fract distortion (long fract input);
long fract echo (long fract input, long fract dec, unsigned length);
long fract reverb (long fract input);

// Configuration functions
void TWI_write(uint16_t, uint8_t);
void codec_configure(void);
void sport_configure(void);
void init_SPORT_DMA(void);
void SPORT0_RX_interrupt_handler(uint32_t iid, void *handlerArg);
void memWrite(unsigned, unsigned);
void memFractWrite(unsigned, long fract);


// Flash code START
/*
Program FlashMem2 is a simple demonstration program for writing and reading data to/from
the serial flash memory on the ADSP-BF706 EZ-KIT Mini board. It does not require any
external files other than register addresses. It is very basic and contains no error
checking. However, it illustrates how to establish communication and interact with the
W25Q32 chip.
Author: Patrick Gaydecki
Date : 28.07.2021
*/

void flash_delay(void);
void flash_initialize(void);
void flash_write_byte(uint8_t);
void flash_sector_erase(int);
void flash_read_page(int, uint8_t[]);
void flash_write_page(int, uint8_t[]);
uint8_t const write_enable=0x06;
uint8_t const erase=0x20;
uint8_t const unlock=0x98;
uint8_t const write_page=0x2;
uint8_t const write_disable=0x4;
uint8_t const read_page=0x03;
uint8_t lo_addr;
uint8_t mid_addr;
uint8_t hi_addr;
int n;
// Function flash_delay is a crude delay function
void flash_delay()
{
 for(n=0;n<6000000;n++){};
}
// Function flash_initialize sets Port B as SPI, enables the flash write, and
// unlocks the global erase feature.
void flash_initialize()
{
 *pREG_PORTB_FER = 0x7c00; // Set Port B as SPI
 *pREG_PORTB_FER_SET = 0x7c00;
 *pREG_PORTB_DIR_SET = 0x8000;
 *pREG_SPI2_CLK = 0x1; // Set bit rate rate
 *pREG_SPI2_CTL = 0x3; // Enable SPI
 *pREG_PORTB_DATA_CLR = 0x8000; // Flash chip enable
flash_write_byte(write_enable); // Write enable the flash
 *pREG_PORTB_DATA_SET = 0x8000; // Flash chip disable
 *pREG_PORTB_DATA_CLR = 0x8000;
flash_write_byte(unlock); // Unlock all sectors
 *pREG_PORTB_DATA_SET = 0x8000;
}
// Function flash_write_byte writes a single byte to the flash.
void flash_write_byte(uint8_t command)
{
 *pREG_SPI2_TWC = 0x1;
 *pREG_SPI2_TFIFO = command;
 *pREG_SPI2_TXCTL = 0xD;
 while((*pREG_SPI2_STAT & 0x800)==0x0){} // Wait for TX complete
 *pREG_SPI2_TXCTL = 0;
}

// Function flash_sector_erase erases an entire sector. Here it is sector 0.
void flash_sector_erase(int sectorw)
{
 sectorw*=4096; // Decompose the address
 lo_addr=sectorw;
 mid_addr=sectorw>>8;
 hi_addr=sectorw>>16;
 *pREG_PORTB_DATA_CLR = 0x8000;
 flash_write_byte(write_enable); // Write enable the flash;
 *pREG_PORTB_DATA_SET = 0x8000;
 *pREG_PORTB_DATA_CLR = 0x8000;
 flash_write_byte(erase); // Erase the sector;
 flash_write_byte(hi_addr);
 flash_write_byte(mid_addr);
 flash_write_byte(lo_addr);
 *pREG_PORTB_DATA_SET = 0x8000;
 flash_delay(); // Lazy delay. Should check status
}
// Function flash_read_page reads an entire 256-byte page
void flash_read_page(int pager, uint8_t data_rd[])
{
 *pREG_PORTB_DATA_CLR = 0x8000;
 pager*=256; // Decompose the address
 lo_addr=pager;
 mid_addr=pager>>8;
 hi_addr=pager>>16;
 flash_write_byte(read_page); // Read page instruction
 flash_write_byte(hi_addr);
 flash_write_byte(mid_addr);
 flash_write_byte(lo_addr);
 *pREG_SPI2_RXCTL=0x5;
 for (n=0;n<256;n++)
 {
 *pREG_SPI2_RWC = 0x1;
 while((*pREG_SPI2_STAT & 0x400)==0x0){} // Wait for RX complete
 data_rd[n]=*pREG_SPI2_RFIFO;
 }
 *pREG_SPI2_RXCTL=0;
 *pREG_PORTB_DATA_SET = 0x8000;
}
// Function flash_write_page writes an entire 256-byte page
void flash_write_page(int pagew, uint8_t data_wr[])
{
 pagew*=256; // Decompose the address
 lo_addr=pagew;
 mid_addr=pagew>>8;
 hi_addr=pagew>>16;
 *pREG_PORTB_DATA_CLR = 0x8000;
 flash_write_byte(write_enable); // Write enable the flash;
 *pREG_PORTB_DATA_SET = 0x8000;
 *pREG_PORTB_DATA_CLR = 0x8000;
 flash_write_byte(write_page); // Write page instruction;
 flash_write_byte(hi_addr);
 flash_write_byte(mid_addr);
 flash_write_byte(lo_addr);
 for (n=0;n<256;n++) {flash_write_byte(data_wr[n]);} // Now write any 256 bytes
 *pREG_PORTB_DATA_SET = 0x8000;
 flash_delay();
}

// Flash code END


// MemWrite writes a 32-bit value to a memory location
// MemFractWrite does the same but with a long fract
// MemFractRead reads a long fract from memory
void memWrite(unsigned address, unsigned memvalue)
{
 unsigned *towrite;
 towrite = (unsigned*)address;
 *towrite=memvalue;
}

void memFractWrite(unsigned address, long fract memvalue)
{
 long fract *towrite;
 towrite = (long fract*)address;
 *towrite=memvalue;
}

long fract memFractRead(uint32_t address)
{
  long fract *core_address = (long fract *)address;
  return(*core_address);
}

// EFFECT FUNCTIONS

long fract gain (long fract input){
    if (gainPWR == 1){                    // Gain ON/OFF?
        return input*gainCoeff;
    }
    else{
        return input;
    }
}

long fract distortion (long fract input){
    if (heavyDistPWR == 1){                    // Distortion heavy ON/OFF?
         if (input >= pThres){                // If over upper threshold, hard clip
             return pThres*11;            // Gain = 11 for powerful sound
         }
         if (input <= nThres){
             return nThres*11;            // If under lower threshold, hard clip
         }

         else{
             return input*11;
         }
    }

    if (crunchDistPWR == 1){                // Distortion crunch ON/OFF?

         if (input >= pThres*8){            // As before, but with a higher threshold
             return (pThres*8)*2;            // Gain = 2 for crunchier sound
         }
         if (input <= nThres*8){
             return (nThres*8)*2;
         }
         else{
             return input*2;
         }
    }
    else{
            return input;                // Otherwise return input
    }
}

long fract echo (long fract input, long fract dec, unsigned length){
    if (echoPWR == 1){                    // Echo ON/OFF?
        memFractWrite(0x11906004, input);        // Write input, decay and length into memory
        memFractWrite(0x08010030, dec);
        memWrite(0x0801002c, length*4);
        asm
           (
            "P0=[0x08010024];I0=P0;"                 // Reload I0
            "P0=[0x0801002c];L0=P0;"                 // Load delay into L0
            "R1=[0x08010030];"                       // Decay coefficient
            "R0=[0x11906004];"                       // Get codec value

            "A1 = R0;"                        // Clear A0 and get data/taps
            "R0=[I0];"                 // Load into R0 the value at the start of the buffer

            "A1:0+= R0 * R1;"                        // Echo
                "R1=A1:0;[0x11906004]=R1;"
                "[I0++]=R1;"             // Put result at the start of the next buffer

            "P0=I0;[0x08010024]=P0;"                 // Load filter length into P0
        );
        long fract output = memFractRead(0x11906004);     // Get output

        return output;
    }
    else{
        return input;                     // Otherwise, return input
    }
}

long fract reverb (long fract input){
    if (reverbHallPWR == 1){                // Reverb hall ON/OFF?
        for (unsigned c = 1; c < 7; c++){        // 6 comb filters
            if (k >= REV_ROOM_SIZE){        // Start overwriting old values when at the end of circular buffer
                k = 0;
                }
            A0 = input;
            A0 += (i[c][k] * (decayReverb - 0.063*c));    // MAC with function to give different decay for each filter
            i[c][k++] = A0;                 // Place in next address of circular buffer
        }
        long fract output = (A0 + A1 + A2 + A3) * op;        // Averaging system to smooth output
        A1 = A0;
        A2 = A1;
        A3 = A2;
        return output;
    }
    else if (reverbRoomPWR == 1){                // As before
        for (unsigned c = 1; c < 7; c++){
            if (k >= (REV_ROOM_SIZE/2)){        // Circular buffer length/2 to give a smaller 'room size'
                k = 0;
            }
            A0 = input;
            A0 += (i[c][k] * (decayReverb - 0.05*c));
            i[c][k++] = A0;
        }
        long fract output = (A0 + A1 + A2 + A3) * op;
        A1 = A0;
        A2 = A1;
        A3 = A2;
        return output;
    }
    else{                            // Otherwise, return input
        return input;
    }
}

// Subroutine DMA_init initialises the SPORT0 DMA0 and DMA1 in auto-buffer mode, p19–39 and p19–49, BHRM.
void init_SPORT_DMA()
{
 *pREG_DMA0_ADDRSTART = YOUT; // Points to start of SPORT0_A buffer
 *pREG_DMA0_XCNT = BUFFER_SIZE; // No. of words to transmit
 *pREG_DMA0_XMOD = 4; // Word length, increment to find next word in the buffer
 *pREG_DMA1_ADDRSTART = XIN; // Points to start of SPORT0_B buffer
 *pREG_DMA1_XCNT = BUFFER_SIZE; // No. of words to receive
 *pREG_DMA1_XMOD = 4; // Word length, increment to find the next word in buffer
 *pREG_DMA0_CFG = 0x00001221; // SPORT0 TX, FLOW = autobuffer, MSIZE = PSIZE = 4 bytes
 *pREG_DMA1_CFG = 0x00101223; // SPORT0 RX, DMA interrupt when x count expires
}

// Function SPORT0_RX_interrupt_handler is called when a new audio sample has been received.
// Here, only a single channel is used. The other is ignored. The input is held in XIN[1] and the
// Output is YIN[1]
void SPORT0_RX_interrupt_handler(uint32_t iid, void *handlerArg)
{
 *pREG_DMA1_STAT = 0x1; // Clear interrupt
 // Declare effect functions
 long fract effect1[BUFFER_SIZE], effect2[BUFFER_SIZE], effect3[BUFFER_SIZE], effect4[BUFFER_SIZE];

 for(int i = 0; i < BUFFER_SIZE; i++){                   // Run for each channel (left and right)

     // ***************************GAIN*******************************
     effect1[0+i] = gain(XIN[0+i]);

     // ************************DISTORTION****************************
     effect2[0+i] = distortion(effect1[0+i]);

     // ***************************ECHO********************************
     effect3[0+i] = echo(effect2[0+i], decayEcho, filterLengthEcho); // Pass echo decay and filter length

     // **************************REVERB*******************************
     effect4[0+i] = reverb(effect3[0+i]);

     // **************************OUTPUT*******************************
     YOUT[0+i] = effect4[0+i];
};
}

// Function sport_configure initialises the SPORT0. Refer to pages 26-59, 26-67,
// 26-75 and 26-76 of the ADSP-BF70x Blackfin+ Processor Hardware Reference manual.
void sport_configure()
{
 *pREG_PORTC_FER=0x0003F0; // Set up Port C in peripheral mode
 *pREG_PORTC_FER_SET=0x0003F0; // Set up Port C in peripheral mode
 *pREG_SPORT0_CTL_A=0x2001973; // Set up SPORT0 (A) as TX to codec, 24 bits
 *pREG_SPORT0_DIV_A=0x400001; // 64 bits per frame, clock divisor of 1
 *pREG_SPORT0_CTL_B=0x0001973; // Set up SPORT0 (B) as RX from codec, 24 bits
 *pREG_SPORT0_DIV_B=0x400001; // 64 bits per frame, clock divisor of 1
}

// Function TWI_write is a simple driver for the TWI. Refer to page 24-15 onwards
// Of the ADSP-BF70x Blackfin+ Processor Hardware Reference manual.
void TWI_write(uint16_t reg_add, uint8_t reg_data)
{
 int n;
 reg_add=(reg_add<<8)|(reg_add>>8); // Reverse low order and high order bytes
 *pREG_TWI0_CLKDIV=0x3232; // Set duty cycle
 *pREG_TWI0_CTL=0x8c; // Set prescale and enable TWI
 *pREG_TWI0_MSTRADDR=0x38; // Address of codec
 *pREG_TWI0_TXDATA16=reg_add; // Address of register to set, LSB then MSB
 *pREG_TWI0_MSTRCTL=0xc1; // Command to send three bytes and enable transmit
 for(n=0;n<8000;n++){} // Delay since codec must respond
 *pREG_TWI0_TXDATA8=reg_data; // Data to write
 for(n=0;n<10000;n++){} // Delay
 *pREG_TWI0_ISTAT=0x050; // Clear TXERV interrupt
 for(n=0;n<10000;n++){} // Delay
 *pREG_TWI0_ISTAT=0x010; // Clear MCOMP interrupt
}

// Function codec_configure initialises the ADAU1761 codec. Refer to the control register
// Descriptions, page 51 onwards of the ADAU1761 data sheet.
void codec_configure()
{
 TWI_write(0x4000, 0x01); // Enable master clock, disable PLL
 TWI_write(0x40F9, 0x7f); // Enable all clocks
 TWI_write(0x40Fa, 0x03); // Enable all clocks
 TWI_write(0x4015, 0x01); // Set serial port master mode
 TWI_write(0x4019, 0x13); // Set ADC to on, both channels
 TWI_write(0x401c, 0x21); // Enable left channel mixer
 TWI_write(0x401e, 0x41); // Enable right channel mixer
 TWI_write(0x4029, 0x03); // Turn on power, both channels
 TWI_write(0x402A, 0x03); // Set both DACs on
 TWI_write(0x40f2, 0x01); // DAC gets L, R input from serial port
 TWI_write(0x40f3, 0x01); // ADC sends L, R input to serial port
 TWI_write(0x400a, 0x0d); // Set left line-in gain to 3 dB
 TWI_write(0x400c, 0x0d); // Set right line-in gain to 3 dB
 TWI_write(0x4023, 0xf3); // Set left headphone volume to 2 dB
 TWI_write(0x4024, 0xf3); // Set right headphone volume to 2 dB
 TWI_write(0x4017, 0x00); // Set codec default sample rate, 48 kHz
}

#pragma optimize_for_speed

int main(void) {                        // Int main flash code
 int sector;
 int page;
 uint8_t data_write[256];
 uint8_t data_read[256];
 sector=0;                          // Sector and page = 0
 page=0;
 for (n=0;n<256;n++) {data_write[n]=n;}             // Fill the data array
 flash_initialize();
 flash_sector_erase(sector);                   // Erase sector before write
 flash_write_page(page, data_write);               // Write page to flash
 flash_read_page(page, data_read);                // Read back page from flash

// Set the clock to maximum
 *pREG_CGU0_CTL=0x2000;
 *pREG_CGU0_DIV=0X42042442;

 bool my_audio = true;
 codec_configure();                     // Configure codec, sport and DMA
 sport_configure();
 init_SPORT_DMA();
 adi_int_InstallHandler(INTR_SPORT0_B_DMA, SPORT0_RX_interrupt_handler, 0, true); // Interrupt handler for incoming audio data
 *pREG_SEC0_GCTL  = 1;                                    // Enable the System Event Controller (SEC)
 *pREG_SEC0_CCTL0 = 1;                                    // Enable SEC Core Interface (SCI)

 // GPIO set up. Set all effect ON/OFF pins as inputs
 adi_gpio_SetDirection(ADI_GPIO_PORT_B, ADI_GPIO_PIN_1 | ADI_GPIO_PIN_2 | ADI_GPIO_PIN_3 | ADI_GPIO_PIN_4 | ADI_GPIO_PIN_5 | ADI_GPIO_PIN_6, ADI_GPIO_DIRECTION_INPUT);
 adi_gpio_SetDirection(ADI_GPIO_PORT_B, ADI_GPIO_PIN_0, ADI_GPIO_DIRECTION_OUTPUT);        // Power for switch LEDs = output
 adi_gpio_Set(ADI_GPIO_PORT_B, ADI_GPIO_PIN_0);                         // Turn on this power pin

 pThres = pThres >> 8;                      // Bit shift right by 8 for distortion threshold
 nThres = nThres >> 8;                      // I.e, convert 32 bit fract into 24 bit so the 24 bit codec can read it successfully
 asm
 (
         "P0=0x4B00;"                            // Circular buffer in bytes (hence X4)
         "I0=0x11800000;B0=I0;L0=P0;"                // Circular buffer for input data, Block A

         "P0=I0;[0x08010024]=P0;"                 // Save I, B and L registers
         "P0=B0;[0x08010028]=P0;"                 // Save I, B and L registers
 );

 while(my_audio){
     adi_gpio_GetData(ADI_GPIO_PORT_B, &data);       // Get GPIO data
     if (data & ADI_GPIO_PIN_1){                // Pin active? FOR ALL BELOW STATEMENTS
         heavyDistPWR = 1;
     }
     else{                          // Pin inactive. FOR ALL BELOW STATEMENTS
         heavyDistPWR = 0;
     }
     if (data & ADI_GPIO_PIN_2){                // ...
         crunchDistPWR = 1;
     }
     else{
         crunchDistPWR = 0;
     }
     if (data & ADI_GPIO_PIN_3){
         echoPWR = 1;
     }
     else{
         echoPWR = 0;
     }
     if (data & ADI_GPIO_PIN_4){
         reverbHallPWR = 1;
     }
     else{
         reverbHallPWR = 0;
     }
     if (data & ADI_GPIO_PIN_5){
         reverbRoomPWR = 1;
     }
     else{
         reverbRoomPWR = 0;
     }
     if (data & ADI_GPIO_PIN_6){
         gainPWR = 1;
     }
     else{
         gainPWR = 0;
     }
 };
 return 0;
}
