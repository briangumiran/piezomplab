
/*
 *
 * file: main.c ( DSP)
 * Brian Anthony L gumiran

 * compatibility with XC16: select large memory format, include lipdsp-elf.a
 * External Interupt: Every 5 Seconds, from RB4 (pin21) to INT0 (pin55)
 * Swept Frequency: parameters set by START FREQ, END FREQ, and PULSE LEN produced at RB5 (pin20)
 * ADC receives resonant frequency at RB2 (pin23)

 * Peripherals Used:
 * Programming pins PGEC1(pin26),PGED1 (pin27)
 * UART TX1 (pin51)/CAN TX (future)
 * External Interrupt pin INT0 (pin55)
 * GPIO: RB4 (pin21) RB5 (pin20)
 * Timers:  sampling timer Timer2
 *          Ext int Timer 4-5
 *          Sweep timer timer 6
 * ADC:     12 bit at RB2 (pin23)
 *
 
 * \file  main.c
 *
 * \author Raphael Victor L. Canseco
 * \date October 2014
 * \version 1.0
 *
 * Project: Senslope \n
 * Processor:      dspic33f256mc710a \n
 * Compiler:       c18 \n
 *
 * Version 1.0
 * This is a fork of the sensor node integration code for pic18f2585
 * This code is targeted for the dspic33f family of s microcontrollers

 */
#define FCY 7370000/2
#define START_FREQ 2500
#define END_FREQ 3100
#define PULSE_LEN 5 //number of pulses before incrementing frequency
#define TIMEOUT 10000//!< timeout to wait for sending

//identification of sensor site peizo
#define ARQ_LOGGER 1 // 0 for OLD master
#define NORMALMODE 0//0 for test mode
#define PIEZONUM 21 //actual number of PIEZO

#include <p33Fxxxx.h>
#include "dsp.h"
#include "fft.h"
#include <libpic30.h>
#include <stdio.h>
#include "senslope_can.h"
#include "PZOCalib.h"



/*CONFIG params*/
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure Segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Mode (Internal Fast RC (FRC))
#pragma config IESO = OFF                //

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
//#pragma config PLLKEN = ON              // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
//#pragma config LPOL = ON                // Low-side PWM Output Polarity (Active High)
//#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
//#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

/* Extern definitions */

extern void SquareMagnitudeCplx(int, fractcomplex*, fractional*);
//This function computes the squared magnitude of elements in a complex vector

extern fractcomplex sigCmpx[FFT_BLOCK_LENGTH] __attribute__ ((section (".ydata, data, ymemory"), aligned (FFT_BLOCK_LENGTH * 2 *2)));
// Typically, the input signal to an FFT routine is a complex array containing samples
//	of an input signal. For this example, we will provide the input signal in an
//	array declared in Y-data space.

// Global Definitions
#ifndef FFTTWIDCOEFFS_IN_PROGMEM
fractcomplex twiddleFactors[FFT_BLOCK_LENGTH/2] __attribute__ ((section (".xbss, bss, xmemory"), aligned (FFT_BLOCK_LENGTH*2)));
// Declare Twiddle Factor array in X-space

#else
extern const fractcomplex twiddleFactors[FFT_BLOCK_LENGTH/2]	// Twiddle Factor array in Program memory
__attribute__ ((space(auto_psv), aligned (FFT_BLOCK_LENGTH*2)));
#endif
#ifndef FFTTWIDCOEFFS_IN_PROGMEM								// Generate TwiddleFactor Coefficients
	TwidFactorInit (LOG2_BLOCK_LENGTH, &twiddleFactors[0], 0);	// We need to do this only once at start-up
#endif


/*===============global variables ==================*/
char buf[50];               //for UART testing
long node_id = PZO_NODE_ID;           //sensor node ID 0x29, for testing 0x01
int data[FFT_BLOCK_LENGTH]; //data for samples
unsigned char *parsed_data;        //parsed data holder
int num = 0, sampling_done = 0, check_init = 0;
int freq = START_FREQ, period, pulse_repeat;
unsigned int status = 0;
mID gCanMsg;                //!< global variable for buffer for CAN messages.
double result;              //variable for FFT result frequency
int counter = 0;
double V_therm; // for thermistor temp
int V_thermADC; // for thermistor temp
double R_therm; // for thermistor temp
/*==================================================*/

// transmit to UART
void transmit(char *sptr){
	while(*sptr != 0){			// Wait until TX buf ready for new data
	   	U1TXREG=*sptr;
		while(U1STAbits.UTXBF==1);
	   	sptr++;
	}
}

void init_uart(void){
	U1MODEbits.STSEL = 0;	// 1 Stop Bit
	U1MODEbits.PDSEL = 0;	// 8bit, No Parity
	U1MODEbits.ABAUD = 0;	// autobaud disabled
	U1MODEbits.BRGH = 0;	// BRGH=0
	U1BRG = 24;			// 24 for FCY = 7.37MHz
	U1STAbits.UTXISEL0 = 0;	// interrupt after 1 tx char is transmitted
	U1STAbits.UTXISEL1 = 0;
	IEC0bits.U1TXIE = 1;	// enable UART tx interrupt
	U1MODEbits.UARTEN = 1;	// enable UART
	U1STAbits.UTXEN = 1;	// enable UART tx
	__delay_ms(10);

	check_init++;
}

void init_timer2(void){
	T2CON = 0x0000; 		// Disable Timer, continue when idle, internal clock
	T2CONbits.TCKPS = 0;	// 1:1 pres
	TMR2 = 0x00;			// Clear timer register
	PR2 = FCY/SAMPLING_RATE;		// 6000 Hz = 5833, 1550 = 22580 (1:1)
                                        // 4755 for 1550Hz 7.37MHz
	IPC1bits.T2IP = 0x01; 	// Set Timer2 Interrupt Priority Level
	IFS0bits.T2IF = 0; 		// Clear Timer2 Interrupt Flag
	IEC0bits.T2IE = 1;		// Enable Timer2 interrupt

	check_init++;

}

void init_adc(void){
	AD1CON1 = 0x0400;		// ADC disabled, 12-bit sampling
	AD1CON2 = 0;
	AD1CON1bits.FORM = 3;	// 3=signed fractional format, 0=integer
	AD1CHS0bits.CH0SA = 4;		// AN4 as +input, VREF- as -input
        AD1CHS0bits.CH0NA = 0;
	AD1CSSL = 0;			// skip ANx for input scan
	AD1CON2bits.VCFG = 0;	// internal voltage
	AD1CON3 = 0x0002;		// clock derived from system clock

	check_init++;
}


void init_adc_therm(void){
    AD2CON1bits.ADON = 0; //adc disabled
    AD2CON1bits.AD12B = 0; //10bit operation
    AD2CON1bits.FORM = 0;  //integer
    AD2CHS0bits.CH0NA = 0; //VREFL
    AD2CHS0bits.CH0SA  = 3; //AN3 AS input
    AD2CON3 = 0x0002;
    AD2CON2 = 0;

}

double do_fft(int *input_data){
	int i=0, k=0;
	int	peakFrequencyBin = 0;				// Declare post-FFT variables to compute the
	double peakFrequency = 0;				// frequency of the largest spectral component

        fractional *p_real = &sigCmpx[0].real ;
        fractcomplex *p_cmpx = &sigCmpx[0] ;

	for (i=0,k=0; i <= (FFT_BLOCK_LENGTH/2)-1; i++,k+=2)
	{
		sigCmpx[i].real = input_data[k];			// save ADC data to sigCmpx
		sigCmpx[i].imag = input_data[k+1];
	}

	for (; i <= FFT_BLOCK_LENGTH-2; i++) {	// zero out all imaginary components
		sigCmpx[i].real = 0;
		sigCmpx[i].imag = 0;
	}

	for (i = 0; i < FFT_BLOCK_LENGTH; i++)	// The FFT function requires input data
	{										// to be in the fractional fixed-point range [-0.5, +0.5]
		*p_real = *p_real >>1 ;				// So, we shift all data samples by 1 bit to the right.
		*p_real++;							// Should you desire to optimize this process, perform
	}										// data scaling when first obtaining the time samples
											// Or within the BitReverseComplex function source code

	p_real = &sigCmpx[(FFT_BLOCK_LENGTH/2)-1].real ;	// Set up pointers to convert real array
	p_cmpx = &sigCmpx[FFT_BLOCK_LENGTH-1] ; 			// to a complex array. The input array initially has all
														// the real input samples followed by a series of zeros

	for ( i = FFT_BLOCK_LENGTH; i > 0; i-- ) 	// Convert the Real input sample array
	{											// to a Complex input sample array
		(*p_cmpx).real = (*p_real--);			// We will simpy zero out the imaginary
		(*p_cmpx--).imag = 0x0000;				// part of each data sample
	}

	// Perform FFT operation
	#ifndef FFTTWIDCOEFFS_IN_PROGMEM
		FFTComplexIP (LOG2_BLOCK_LENGTH, &sigCmpx[0], &twiddleFactors[0], COEFFS_IN_DATA);
	#else
		FFTComplexIP (LOG2_BLOCK_LENGTH, &sigCmpx[0], (fractcomplex *)
		__builtin_psvoffset(&twiddleFactors[0]), (int) __builtin_psvpage(&twiddleFactors[0]));
	#endif

	// Store output samples in bit-reversed order of their addresses
	BitReverseComplex (LOG2_BLOCK_LENGTH, &sigCmpx[0]);

	// Compute the square magnitude of the complex FFT output array to have a Real output vetor
	SquareMagnitudeCplx(FFT_BLOCK_LENGTH, &sigCmpx[0], &sigCmpx[0].real);

	for(i=0; i<5;i++)			// zero out lower frequencies, may cause error in readings
	{
		sigCmpx[i].real = 0;
		sigCmpx[i].imag = 0;
	}
	// Find the frequency Bin ( = index into the SigCmpx[] array) that has the largest energy
	// i.e., the largest spectral component
	VectorMax(FFT_BLOCK_LENGTH/2, &sigCmpx[0].real, &peakFrequencyBin);

	// Compute the frequency (in Hz) of the largest spectral component
	peakFrequency = peakFrequencyBin*((double)SAMPLING_RATE/FFT_BLOCK_LENGTH);
	peakFrequency = (double)(-1.001*peakFrequency)+3109;		// from experimental calibration

        if((peakFrequency<5000) & (peakFrequency>2000)){
            return peakFrequency;
        }
        else{
            return 0;
        }
}

void init_sweep(){
  
    T6CON = 0;
    TMR6 = 0;                   //clear timer register
    PR6 = (FCY/START_FREQ)/4;               //set period for twice the frequency for the toggle
    LATBbits.LATB5 = 1;         //initialize period

    IPC11bits.T6IP = 0x04; 	// Set Timer5 Interrupt Priority Level
    IFS2bits.T6IF = 0; 		// Clear Timer5 Interrupt Flag
    IEC2bits.T6IE = 1;		// Enable Timer5 interrupt

}

/*parse the frequency result, get the digits*/
unsigned char * getdigits(double result_freq){
    int digits[6]; //  result of digit decomposition (3 bytes)
    static unsigned char result_digits[3]; //store the result in pairs (thousands-tenths)
    long int temp = result_freq*100; //move the decimal up
        long int div = 1;
        int i=0;
        
    //set the divisor, get the highest power of 10
    for (div = 1; div <= temp; div*=10 ){}

    //get the digits by dividing by power of 10
        for(div/=10;div>0;div/=10){
            digits[i++]=temp/div;
            temp %= div;
        }
    //organize data to nibbles for can transmission
        result_digits[2]=(unsigned char)(digits[0]*10+digits[1]);
        result_digits[1]=(unsigned char)(digits[2]*10+digits[3]);
        result_digits[0]=(unsigned char)(digits[4]*10+digits[5]);
    return result_digits;
}


int main(void){
//	setup_clock();			// make clock = 8Mhz. clock output on OSC2 pin
	AD1PCFGL = 0xFFE7;		// all port B pins as digital, RB4/5 as analog (thermistor and signal)
        AD2PCFGL = 0xFFE7;
        TRISB = 0x0010;			// set all PORT pins as output
  
	init_uart();
	init_timer2();
	init_adc();
        init_sweep();
        init_adc_therm();
	//init_int0();
        


	if (check_init ==3){
		sprintf(buf, "Initialization complete.\r\n");
		transmit(buf);
	}

        node_ecan_init();//initialize CAN
        
        
	while(1){
            
            //if normal mode, data from CAN
            if (NORMALMODE)
            {
            /*Check Incoming data*/
                status = can_check_for_datain_extended(&gCanMsg);

                //check node id of the poll if same with the node ID of the sensor
                if(status & ((gCanMsg.id >> 3) == node_id)){
                    sprintf(buf, "CAN POLL RECIEVED, START SAMPLING.\r\n");
                    transmit(buf);

                    T6CONbits.TON = 1; // start sweep
     
                // start ADC
                /*wait for sampling to finish*/
                while(sampling_done==0);
		if(sampling_done==1){
                    sprintf(buf, "sampling finished.\r\n");
                    transmit(buf);
                }
                result = do_fft(data);			//sampling finished. do fft.
                //check result in UART
                sprintf(buf, "%.3f\r\n",result);
                transmit(buf);

                //piezo calib
                result = (PC_FAC*result)+PC_CON;
           
                parsed_data = getdigits(result);
                sampling_done = 0;                      //toggle sample flag


                /*Set CAN data loading depending on Sensor Column*/

                if(ARQ_LOGGER){ //for new column (CAN Broadcast)

                    gCanMsg.data[0] = 0xFF;
                    gCanMsg.data[1] = parsed_data[0];
                    gCanMsg.data[2] = 0xCC;
                    gCanMsg.data[3] = parsed_data[1];
                    gCanMsg.data[4] = 0xCC;
                    gCanMsg.data[5] = parsed_data[2];
                    gCanMsg.data[6] = 0xCC;
                    gCanMsg.data[7] = 0x00; //RESERVED FOR THERMISTOR
                    gCanMsg.data_length = 8;
                    gCanMsg.id = node_id;
                    can_send_data_with_arb_repeat_extended(&gCanMsg,TIMEOUT);
                }

                else{ //for old colum (polling)
                    //print buffer data to prevent parsing 
                    gCanMsg.data[0] = parsed_data[2];
                    gCanMsg.data[1] = 0xAB;
                    gCanMsg.data[2] = parsed_data[1];
                    gCanMsg.data[3] = 0xCD;
                    gCanMsg.data[4] = parsed_data[0];
                    gCanMsg.data[5] = 0xEF;
                    gCanMsg.data_length = 6;
                    gCanMsg.id = node_id<<3;
                    can_send_data_with_arb_repeat_extended(&gCanMsg,TIMEOUT);
                }
            }
            }

            //if test mode. print data directly to serial
            else{

                T6CONbits.TON = 1;  // start sweep
                // start ADC
                /*wait for sampling to finish*/
                while(sampling_done==0);
		if(sampling_done==1){
                }
                result = do_fft(data);			//sampling finished. do fft.
                counter += 1; 
                //check result in UART
                sprintf(buf, "%.3f \t %d  \r\n",result, counter);
                transmit(buf);

                //read thermistor resistance

                AD2CON1bits.ADON = 1;
                AD2CON1bits.SAMP = 1;	// start sampling
                __delay_us(10);
                AD2CON1bits.SAMP = 0;	// end sampling, start converting
                while (!AD2CON1bits.DONE);
                V_thermADC = ADC2BUF0;        //fixed resistor voltage, 1000 Ohms


                //compute Thermistor resistance
                R_therm = 1000*((1024.00/V_thermADC)-1); // baka mali ung data types


                sprintf(buf, "%d \t   \r\n",V_thermADC);
                transmit(buf);
                AD2CON1bits.ADON = 0;
           
                sampling_done = 0;
                __delay_ms(5000);

            }
        }
           
	


	return 0;
}


void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void){
	IFS0bits.U1TXIF = 0;	// clear tx interrput flag
}

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void){
	AD1CON1bits.SAMP = 1;	// start sampling
	__delay_us(10);
	AD1CON1bits.SAMP = 0;	// end sampling, start converting
	while (!AD1CON1bits.DONE);	// wait until conversion is complete
	data[num++] = ADC1BUF0;
	if (num == FFT_BLOCK_LENGTH){
		T2CONbits.TON = 0;
		AD1CON1bits.ADON = 0;
//		sprintf(buf, "ADC sampling done. \r\n");
//		transmit(buf);
		sampling_done = 1;
                num = 0;
	}

	IFS0bits.T2IF = 0; 		//Clear Timer2 interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _T6Interrupt(void){
    //For the sweep
    LATBbits.LATB5 = ~LATBbits.LATB5;	//initiate frequency by reversing the output


    //count number of pulses per frequency
    pulse_repeat++;

    //increase frequency after n pulses
    if (pulse_repeat == PULSE_LEN){
        freq++;
        PR6 = (FCY/freq)/2;
        pulse_repeat = 0;
    }
    //repeat frequency after reaching the last freq
    else if (freq == END_FREQ){
        freq = START_FREQ;
        pulse_repeat = 0;


        LATBbits.LATB5 = 1; // make sure the signal is turned off
        //__delay_ms(50);
        LATBbits.LATB4 = 1; //connect sweep to adc
        T6CONbits.TON = 0;  //Turn off sweep timer


        AD1CON1bits.ADON = 1;	// turn ADC on
	T2CONbits.TON = 1;		// dito magsstart mag-ADC

    }

    IFS2bits.T6IF = 0;   //clear interrupt flag
}



/******************************************************************************
 *Function Name: _C1Interrupt
 * Description:
 * handles all the interrupts that are passed by CAN
 *
 * Dependencies:
 *structure mID for rx_ecan1message
 *
 * Parameters:
 * void
 *
 * Output:
 * void
 ******************************************************************************/
void __attribute__((interrupt, no_auto_psv))_C1Interrupt(void)
{
	IFS2bits.C1IF = 0;        // clear interrupt flag
     //   debug("c1 interrupt occured");
	if(C1INTFbits.TBIF)
    {
       //     debug("transmit occured ata");
    	C1INTFbits.TBIF = 0;
    }

    if(C1INTFbits.RBIF)
    {

		// read the message
	    if(C1RXFUL1bits.RXFUL1==1)
	    {

	    }
            if(C1RXFUL1bits.RXFUL2 == 1)
            {
            
            }

		C1INTFbits.RBIF = 0;

	}

        //check for errors in bus
        if(C1INTFbits.ERRIF)
        {
          //   debug("Error ERRIF");
            if(C1VECbits.ICODE == 0b1000001)
            {//Error interrupt
        //        debug("Error interrupt received");
            }
             if(C1VECbits.ICODE == 0b1000011)
            {//Error interrupt
          //      debug("Receiver Overflow interrupt");
            }
             if(C1VECbits.ICODE == 0b1000010)
            {//Error interrupt
       //         debug("Error interrupt wake-up interrupt");
            }



            C1INTFbits.ERRIF = 0;
        }
        /*Invalid message received interrupt
         *generated for any other type of errors during message reception*/
        if(C1INTFbits.IVRIF)
        {
     //       debug("Error in received message");
            C1INTFbits.IVRIF = 0;
        }
}
