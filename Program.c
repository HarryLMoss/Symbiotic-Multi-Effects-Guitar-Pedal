

#include <stdio.h>
#include <cdefBF706.h>
#include <defBF706.h>
#include <math.h>
#include "stdfix.h"
#include <sys/platform.h>
#include "adi_initialize.h"
#include <services/int/adi_int.h>



#define codec_in ((volatile long fract *)0x2004D0C4)
#define codec_out ((volatile long fract *)0x2004D040)


#define BUFFER_SIZE 2 // Size of buffer to transmit

long fract XIN[BUFFER_SIZE]; // Input buffer
long fract YOUT[BUFFER_SIZE]; // Output buffer

long fract pThres = 0.0005;
long fract nThres = -0.0005;

unsigned filterLengthEcho = 4000;
long fract decayEcho = 0.8;

long fract decayReverb = 0.9;
long fract wdReverb = 0.7;
unsigned revRoomSize = 2000;
long fract A0, iPrev = 0;
unsigned iRev = 0;

void TWI_write(uint16_t, uint8_t);
void codec_configure(void);
void sport_configure(void);
void init_SPORT_DMA(void);
void SPORT0_RX_interrupt_handler(uint32_t iid, void *handlerArg);
void memWrite(unsigned, unsigned);
void memFractWrite(unsigned, long fract);
uint32_t memRead(unsigned);
long fract memFractRead(unsigned);

long fract distortion (long fract input);
long fract echo (long fract input, long fract dec, unsigned length, int a);
long fract reverb (long fract input);


// Function memwrite writes a 32-bit value to a memory location
// Inputs: address: the 32-bit address; memvalue: 32-bit value to write
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


uint32_t memRead(uint32_t address)
{
  uint32_t *core_address = (uint32_t *)address;
  return(*core_address);
}

long fract memFractRead(uint32_t address)
{
  long fract *core_address = (long fract *)address;
  return(*core_address);
}



long fract distortion (long fract input){



	 if (input >= pThres){
		 return pThres*10;
	 }

	 if (input <= nThres){
		 return nThres*10;
	 }

	 else{
		 return input*10;
	 }
}

long fract echo (long fract input, long fract dec, unsigned length){

	memFractWrite(0x11906004, input);
	memFractWrite(0x08010030, dec);
	memWrite(0x0801002c, length*4);



	  asm
	       (
	   		"P0=[0x08010024];I0=P0;"                              // Reload I0
	   		"P0=[0x08010028];B0=P0;"                              // Reload B0
	    	"P0=[0x0801002c];L0=P0;"                              // Load delay into L0

			"R1=[0x08010030];"                                    // Decay coefficient
			"R0=[0x11906004];"                                    // Get codec value

			"A1 = R0;"                                            // Clear A0 and get data/taps
					 "R0=[I0];"		//load into R0 whatever was the last value of prev. iteration

			"A1:0+= R0 * R1;"                                     // Echo
					 "R1=A1:0;[0x11906004]=R1;"
					 "[I0++]=R1;"

				 "P0=I0;[0x08010024]=P0;"                              // Load filter length into P0
			   );



	long fract output = memFractRead(0x11906004);

	return output;
	//return input;
}


long fract reverb (long fract input){



	//long fract c1, c2, c3, c4, c5, c6 = 0;


	/*memFractWrite(0x08000000, echo(input, 0.85, revRoomSize));
	memFractWrite(0x08000004, echo(input, 0.80, revRoomSize));
	memFractWrite(0x08000008, echo(input, 0.75, revRoomSize));
	memFractWrite(0x08000012, echo(input, 0.70, revRoomSize));
	memFractWrite(0x08000016, echo(input, 0.65, revRoomSize));
	memFractWrite(0x08000020, echo(input, 0.60, revRoomSize));*/

	/*c1 = echo(input, 0.85, revRoomSize, 1);
	c2 = echo(input, 0.85, revRoomSize, 2);
	c3 = echo(input, 0.75, revRoomSize, 3);
	c4 = echo(input, 0.70, revRoomSize, 4);
	c5 = echo(input, 0.65, revRoomSize, 5);
	c6 = echo(input, 0.55, revRoomSize, 6);*/
	
	
/*	if (i = 0){
		iPrev = input;
	}*/
	
	//else{
		for (unsigned c = 1; c < 7; c++){
			A0 = input;
			A0 += iPrev * (decayReverb - 0.05*c);
				echo[c] = A0;
			iPrev = A0;
			iRev++;
		}
	//}
	
	if (iRev >= revRoomSize){
		iRev = 0;
	}
	long fract echoes = (echo[1]+echo[2]+echo[3]+echo[4]+echo[5]+echo[6])/6;

	long fract output = input*wdReverb + echoes*(1.00000 - wdReverb);

	return output;
	//return input;
}



// Subroutine DMA_init initialises the SPORT0 DMA0 and DMA1 in auto-buffer mode, p19–39 and p19–49, BHRM.
void init_SPORT_DMA()
{
 *pREG_DMA0_ADDRSTART = YOUT; // points to start of SPORT0_A buffer
 *pREG_DMA0_XCNT = BUFFER_SIZE; // no. of words to transmit
 *pREG_DMA0_XMOD = 4; // Word length, increment to find next word in the buffer
 *pREG_DMA1_ADDRSTART = XIN; // points to start of SPORT0_B buffer
 *pREG_DMA1_XCNT = BUFFER_SIZE; // no. of words to receive
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

 int i;
 long fract effect1[BUFFER_SIZE], effect2[BUFFER_SIZE], effect3[BUFFER_SIZE];

 for(i = 0; i < BUFFER_SIZE; i++){

	 //************************DISTORTION****************************

	 effect1[0+i] = distortion(XIN[0+i]);

	 //************************ECHO*********************************

	 //effect2[0+i] = echo(effect1[0+i], decayEcho, filterLengthEcho, 0);

	 //************************REVERB*******************************

	 effect3[0+i] = reverb(effect1[0+i]);

	 //*************************OUTPUT********************************

	 YOUT[0+i] = effect3[0+i]*5;
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
// of the ADSP-BF70x Blackfin+ Processor Hardware Reference manual.
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
// descriptions, page 51 onwards of the ADAU1761 data sheet.
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
 TWI_write(0x400a, 0x0b); // Set left line-in gain to 0 dB
 TWI_write(0x400c, 0x0b); // Set right line-in gain to 0 dB
 TWI_write(0x4023, 0xe7); // Set left headphone volume to 0 dB
 TWI_write(0x4024, 0xe7); // Set right headphone volume to 0 dB
 TWI_write(0x4017, 0x00); // Set codec default sample rate, 48 kHz
}


#pragma optimize_for_speed

int main(void) {
 //unsigned filter_length=8060;
// Set the clock to maximum
 *pREG_CGU0_CTL=0x2000;
 *pREG_CGU0_DIV=0X42042442;

 bool my_audio = true;
 codec_configure();
 sport_configure();
 init_SPORT_DMA();
 adi_int_InstallHandler(INTR_SPORT0_B_DMA, SPORT0_RX_interrupt_handler, 0, true);
 *pREG_SEC0_GCTL  = 1;                                    // Enable the System Event Controller (SEC)
 *pREG_SEC0_CCTL0 = 1;                                    // Enable SEC Core Interface (SCI)



 pThres = pThres >> 8;
 nThres = nThres >> 8;
 
 revRoomSize *= 6;

 asm
 (

		 "P0=0x4B00;"                        // Circular buffer in bytes (hence X4)
		 "I0=0x11800000;B0=I0;L0=P0;"        // Circular buffer for input data, Block A

		 "P0=I0;[0x08010024]=P0;"            // Save I, B and L registers
		 "P0=B0;[0x08010028]=P0;"            // Save I, B and L registers


 );


 while(my_audio){};


 return 0;

}

