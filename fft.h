/* Constant Definitions */
#define FFT_BLOCK_LENGTH	1024     /* 256 = Number of frequency points in the FFT */
#define LOG2_BLOCK_LENGTH 	10 		 /* 8 = Number of "Butterfly" Stages in FFT processing */
#define SAMPLING_RATE		1550	/* = Rate at which input signal was sampled */
                                    /* SAMPLING_RATE is used to calculate the frequency*/
                                    /* of the largest element in the FFT output vector*/

#define FFTTWIDCOEFFS_IN_PROGMEM 	/*<---Comment out this line of the code if twiddle factors (coefficients) */
                                	/*reside in data memory (RAM) as opposed to Program Memory */
                                	/*Then remove the call to "TwidFactorInit()" and add the twiddle factor*/
                                	/*coefficient file into your Project. An example file for a 256-pt FFT*/
                                	/*is provided in this Code example */

