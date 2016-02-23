/*!
 * \file  senslope_can.c
 *
 * \author Raphael Victor L. Canseco
 * \date October 2014
 * \version 1.0
 *
 * Project:        Senslope                                                     \n
 * Processor:      dspic33f256mc710a                                                   \n
 * Compiler:       c18                                                          \n
 * Build:
 *
 * Changelog:                                                                 \n
 *
 *
 *
 */
/* Device header file */
#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #if defined(__dsPIC33E__)
    	#include <p33Exxxx.h>
    #elif defined(__dsPIC33F__)
    	#include <p33Fxxxx.h>
    #endif
#endif
//
//#if defined(__dsPIC33F__)
//#include "p33fxxxx.h"
//#elif defined(__PIC24H__)
//#include "p24hxxxx.h"
//#endif

//#include <p33FJ256GP710A.h>

#include "senslope_can.h"


extern long node_id;//!< nodeid must be unique for every node

/*Important this is for the dma mapping of the CAN */
ECAN1MSGBUF ecan1msgBuf __attribute__((space(dma),aligned(ECAN1_MSG_BUF_LENGTH*16)));



/******************************************************************************
 *Function Name:dma0init
 * Description:
 * Sets up the DMA for ecan transmission
 * Dma Initialization for ECAN1 Transmission
 * Dependencies:
 * This depends on the global variable ecan1msgBuf, a pointer to a memory
 * location for the DMA
 *
 * Parameters: VOID
 *
 *
 * Output: VOID
 *
 *
 *
 ******************************************************************************/
void dma0init(void){

	 DMACS0=0;
         DMA0CON=0x2020;
	 DMA0PAD=0x0442;	/* ECAN 1 (C1TXD)  point dma channel to CANtx*/
 	 DMA0CNT=0x0007;
	 DMA0REQ=0x0046;/* ECAN 1 Transmit Configure channel for transmit*/
	 DMA0STA=  __builtin_dmaoffset(&ecan1msgBuf[0]); //modified from example
	 DMA0CONbits.CHEN=1;

}


/******************************************************************************
 *Function Name:dma2init
 * Description:
 * Sets up the DMA for ecan reception
 * Dma Initialization for ECAN1 reception
 * Dependencies:
 * This depends on the global variable ecan1msgBuf, a pointer to a memory
 * location for the DMA
 *
 * Parameters: VOID
 *
 *
 * Output: VOID
 *
 *
 *
 ******************************************************************************/
void dma2init(void){

	 DMACS0=0;
         DMA2CON=0x0020;
	// DMA2PAD=0x0440;	/* ECAN 1 (C1RXD) */
         DMA2PAD = (int)(&C1RXD);
 	 DMA2CNT=0x0007;
	 DMA2REQ=0x0022;	/* ECAN 1 Receive */
         /*changed ecan1msgBuf to ecan1msgBuf[0] as prescribed in a forum*/
	 DMA2STA= __builtin_dmaoffset(&ecan1msgBuf[0]); //Modified from example
         _DMA2IE= 0;
   	_DMA2IF = 0;
	 DMA2CONbits.CHEN=1;

}





/******************************************************************************
 *Function Name:can_set_ports
 * Description:
 * configures the ports for can use
 * disables the analog function for the ports
 * sets up the cantx for output and canrx for input
 * unlocks the peripheral select to point CAN to the port physical pins
 *
 * Dependencies:
 * assumes specific pins for a specific micro-controller
 *
 * Parameters:
 * void
 *
 * Output:
 * void
 ******************************************************************************/
void can_set_ports(void)
{
    TRISFbits.TRISF1 = 0;     //CANtx
    TRISFbits.TRISF0 = 1;     //CANRx

}







//------------------------------------------------------------------------------
//    DMA interrupt handlers
//------------------------------------------------------------------------------

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
   IFS0bits.DMA0IF = 0;          // Clear the DMA0 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA2Interrupt(void)
{
   IFS1bits.DMA2IF = 0;          // Clear the DMA2 Interrupt Flag;
}

/*
 * ECANDRV portion
 */


/******************************************************************************
 *Function Name:ecan1WriteRxAcptFilter
 * Description:
 * Writes the acceptance filters that are needed by the CAN controller to help
 * it decide if it would accept the message with specific identifier
 * Dependencies:
 *
 *
 * Parameters:
 * int n: filter number
 * long identifier: could be standard or extended
 * unsigned int exide: dictates if standard or extended filter
 *                           0 for standard 1 for extended
 * unsigned int bufPnt: pointer for the buffer where the filtered message
 *                                                      would be stored
 * unsigned int maskSel: to select what mask would be used for the messages
 *
 *
 * Output:
 * void
 *
 *
 ******************************************************************************/
void ecan1WriteRxAcptFilter(int n, long identifier, unsigned int exide,
                            unsigned int bufPnt,unsigned int maskSel)
{
    /*
This function configures Acceptance Filter "n"

Inputs:
n-> Filter number [0-15]
identifier-> Bit ordering is given below
Filter Identifier (29-bits) : 0b000f ffff ffff ffff ffff ffff ffff ffff
				   |____________|_____________________|
					  SID10:0           EID17:0


Filter Identifier (11-bits) : 0b0000 0000 0000 0000 0000 0fff ffff ffff
						          |___________|
							       SID10:0
exide -> "0" for standard identifier
		 "1" for Extended identifier

bufPnt -> Message buffer to store filtered message [0-15]
maskSel -> Optional Masking of identifier bits [0-3]

*/


unsigned long sid10_0=0, eid15_0=0, eid17_16=0;
unsigned int *sidRegAddr,*bufPntRegAddr,*maskSelRegAddr, *fltEnRegAddr;


	C1CTRL1bits.WIN=1;

	/*Obtain the Address of CiRXFnSID, CiBUFPNTn, CiFMSKSELn and CiFEN
         * register for a given filter number "n"*/
	sidRegAddr = (unsigned int *)(&C1RXF0SID + (n << 1));
	bufPntRegAddr = (unsigned int *)(&C1BUFPNT1 + (n >> 2));
	maskSelRegAddr = (unsigned int *)(&C1FMSKSEL1 + (n >> 3));
	fltEnRegAddr = (unsigned int *)(&C1FEN1);


	// Bit-filed manupulation to write to Filter identifier register
	if(exide==1) { 	// Filter Extended Identifier
		eid15_0 = (identifier & 0xFFFF);
		eid17_16= (identifier>>16) & 0x3;
		sid10_0 = (identifier>>18) & 0x7FF;
                // Write to CiRXFnSID Register
	    *sidRegAddr=(((sid10_0)<<5) + 0x8) + eid17_16;
	    *(sidRegAddr+1)= eid15_0;	// Write to CiRXFnEID Register

	}else{	// Filter Standard Identifier
		sid10_0 = (identifier & 0x7FF);
		*sidRegAddr=(sid10_0)<<5;// Write to CiRXFnSID Register
		*(sidRegAddr+1)=0;	// Write to CiRXFnEID Register
	}

        // clear nibble
   *bufPntRegAddr = (*bufPntRegAddr) & (0xFFFF - (0xF << (4 *(n & 3))));
   // Write to C1BUFPNTn Register
   *bufPntRegAddr = ((bufPnt << (4 *(n & 3))) | (*bufPntRegAddr));
   // clear 2 bits
   *maskSelRegAddr = (*maskSelRegAddr) & (0xFFFF - (0x3 << ((n & 7) * 2)));
    // Write to C1FMSKSELn Register
   *maskSelRegAddr = ((maskSel << (2 * (n & 7))) | (*maskSelRegAddr));

   *fltEnRegAddr = ((0x1 << n) | (*fltEnRegAddr)); // Write to C1FEN1 Register

   C1CTRL1bits.WIN=0;


}


unsigned int ECANFilterSID(int sid, unsigned long eid, int exide)
{
	/* This function returns the value to be
	 * stored in CxFnSID register.
	 * SID - The Standard ID
	 * EID - The Extended ID. For Standard messages
	 * 		 EID should be zero.
	 * EXIDE - 0 for Standard messages. 1 otherwise
	 */

	unsigned int returnVal;

	returnVal = sid << 5;
	if(exide == 1)
		returnVal |= 0x8;
	else
		returnVal &= 0xFFFE;
	returnVal |= (int)(eid >> 16);
	return (returnVal);
}


void ECAN1RxFiltersConfig(int SID1,unsigned int bufpnt )
{
	/* Enable 2 filters. The ID are defined in node.h
	 * Note that using Register Indirect mode does not
	 * affect the filter and masking capability. You may
	 * not want to set up the CxBUFPNTn bits to point to
	 * FIFO. This way you can avoid the FIFO interrupts.
	 * Just point to an arbitrary RX buffer */

	/* In this case filters 0 and 4 are used */

	 C1CTRL1bits.WIN = 1;
	 C1FEN1 = 0x11;

	 C1RXF0SID = ECANFilterSID(SID1,0,0);

	// C1RXF4SID = ECANFilterSID(SID2,0,0);

	 C1RXM0SID = 0xFFEB;		/* Configure MASK 0 - All bits used in comparison*/
	 C1RXM0EID = 0xFFFF;

	 C1FMSKSEL1bits.F0MSK = 0x0;	/* User MASK 0 for all filters */
	 //C1FMSKSEL1bits.F4MSK = 0x0;	/* User MASK 0 for all filters */

	 C1BUFPNT1bits.F0BP = bufpnt;		/* Set the destination buffers to be any thing but */
	 //C1BUFPNT2bits.F4BP = 0x1;		/* configured transmit buffers */


}


/******************************************************************************
 *Function Name:ecan1WriteRxAcptMask
 * Description:
 * Configures the MASKs to be used for CAN
 * Dependencies:
 *
 *
 * Parameters:
 * int m: mask number
 * long identifier: could be standard or extended
  * unsigned int mide: 0 dont care if extended or standard a match is a match
 * 1 look at exide for matching type
 *unsigned int exide: 1 extended
 *                    0 standard
 *
 *
 * Output:
 * void
 *
 *
 ******************************************************************************/

void ecan1WriteRxAcptMask(int m, long identifier, unsigned int mide,
                                                unsigned int exide){

    /*
This function configures Acceptance Filter Mask "m"

Inputs:
m-> Mask number [0-2]
identifier-> Bit ordering is given below
Filter Mask Identifier (29-bits) : 0b000f ffff ffff ffff ffff ffff ffff ffff
	                                |____________|_____________________|
						   SID10:0           EID17:0


Filter Mask Identifier (11-bits) : 0b0000 0000 0000 0000 0000 0fff ffff ffff
							       |___________|
						                   SID10:0

mide ->  "0"  Match either standard or extended address message if filters match
         "1"  Match only message types that correpond to 'exide' bit in filter

*/


unsigned long sid10_0=0, eid15_0=0, eid17_16=0;
unsigned int *maskRegAddr;


	C1CTRL1bits.WIN=1;

	// Obtain the Address of CiRXMmSID register for given Mask number "m"
	maskRegAddr = (unsigned int *)(&C1RXM0SID + (m << 1));

	// Bit-filed manupulation to write to Filter Mask register
	if(exide==1)
	{ 	// Filter Extended Identifier
		eid15_0 = (identifier & 0xFFFF);
		eid17_16= (identifier>>16) & 0x3;
		sid10_0 = (identifier>>18) & 0x7FF;

		if(mide==1){
                    /* Write to CiRXMnSID Register*/
			*maskRegAddr=((sid10_0)<<5) + 0x0008 + eid17_16;
                }else{// Write to CiRXMnSID Register
			*maskRegAddr=((sid10_0)<<5) + eid17_16;
                }
	    *(maskRegAddr+1)= eid15_0;		// Write to CiRXMnEID Register

	}
	else
	{			// Filter Standard Identifier
		sid10_0 = (identifier & 0x7FF);
		if(mide==1){
                    // Write to CiRXMnSID Register
			*maskRegAddr=((sid10_0)<<5) + 0x0008;
                }else
			*maskRegAddr=(sid10_0)<<5;// Write to CiRXMnSID Register

		*(maskRegAddr+1)=0;	// Write to CiRXMnEID Register
	}


	C1CTRL1bits.WIN=0;

}

/******************************************************************************
 *Function Name:ecan1WriteRxAcptMask
 * Description:
 * Configures the MASKs to be used for CAN
 * Dependencies:
 *
 *
 * Parameters:
 * int m: mask number
 * long identifier: could be standard or extended
  * unsigned int mide: 0 dont care if extended or standard a match is a match
 * 1 look at exide for matching type
 *unsigned int exide: 1 extended
 *                    0 standard
 *
 *
 * Output:
 * void
 *
 *
 ******************************************************************************/
void ecan1WriteTxMsgBufId(unsigned int buf, long txIdentifier, unsigned int ide,
                                                    unsigned int remoteTransmit){


    /* ECAN Transmit Message Buffer Configuration

Inputs:
buf	-> Transmit Buffer Number

txIdentifier ->

Extended Identifier (29-bits) : 0b000f ffff ffff ffff ffff ffff ffff ffff
				      |____________|_____________________|
						SID10:0           EID17:0



Standard Identifier (11-bits) : 0b0000 0000 0000 0000 0000 0fff ffff ffff
					                    |___________|
						                 SID10:0

Standard Message Format:
						Word0 : 0b000f ffff ffff ffff
							|____________|||___
 							SID10:0   SRR   IDE

						Word1 : 0b0000 0000 0000 0000
								|____________|
								 EID17:6

						Word2 : 0b0000 00f0 0000 ffff
						  |_____||	  	 |__|
							EID5:0 RTR   	  DLC



Extended Message Format:
						Word0 : 0b000f ffff ffff ffff
							|____________|||___
 							SID10:0   SRR   IDE

						Word1 : 0b0000 ffff ffff ffff
								 |____________|
								  EID17:6

						Word2 : 0bffff fff0 0000 ffff
						  |_____||	  	 |__|
							  EID5:0 RTR   	  DLC

ide -> "0"  Message will transmit standard identifier
	   "1"  Message will transmit extended identifier



remoteTransmit -> "0" Message transmitted is a normal message
				  "1" Message transmitted is a remote message

				Standard Message Format:
					      Word0 : 0b000f ffff ffff ff1f
							  |____________|||___
 							SID10:0   SRR   IDE

						Word1 : 0b0000 0000 0000 0000
							  |____________|
								  EID17:6

						Word2 : 0b0000 0010 0000 ffff
						 |_____||	  	 |__|
							EID5:0 RTR   	  DLC



				Extended Message Format:
						Word0 : 0b000f ffff ffff ff1f
						     |____________|||___
 							SID10:0   SRR   IDE

						Word1 : 0b0000 ffff ffff ffff
							   |____________|
								  EID17:6

						Word2 : 0bffff ff10 0000 ffff
						  |_____||	  	 |__|
							  EID5:0 RTR   	  DLC

*/

unsigned long word0=0, word1=0, word2=0;
unsigned long sid10_0=0, eid5_0=0, eid17_6=0;


if(ide)
	{
		eid5_0  = (txIdentifier & 0x3F);
		eid17_6 = (txIdentifier>>6) & 0xFFF;
		sid10_0 = (txIdentifier>>18) & 0x7FF;
		word1 = eid17_6;
	}
	else
	{
		sid10_0 = (txIdentifier & 0x7FF);
	}


	if(remoteTransmit==1) { 	// Transmit Remote Frame

		word0 = ((sid10_0 << 2) | ide | 0x2);
		word2 = ((eid5_0 << 10)| 0x0200);}

	else {

		word0 = ((sid10_0 << 2) | ide);
		word2 = (eid5_0 << 10);
	     }

/* Obtain the Address of Transmit Buffer in DMA RAM for a
 * given Transmit Buffer number*/

if(ide)
	ecan1msgBuf[buf][0] = (word0 | 0x0002);
else
	ecan1msgBuf[buf][0] = word0;

	ecan1msgBuf[buf][1] = word1;
	ecan1msgBuf[buf][2] = word2;
}



/******************************************************************************
 *Function Name:ecan1WriteTxMsgBufData
 * Description:
 * Puts data to be transmitted in the buffers for transmission
 * Dependencies:
 *
 *
 * Parameters:
 * unsigned int buf: transmit buffer number
 * data length: Length of Data in Bytes to be transmitted
 * unsigned int data1-data4: the data that are to be transmitted
 *
 *
 * Output:
 * void
 *
 *
 ******************************************************************************/

void ecan1WriteTxMsgBufData(unsigned int buf, unsigned int dataLength,
        unsigned int data1, unsigned int data2, unsigned int data3,
                                                unsigned int data4){

	ecan1msgBuf[buf][2] = ((ecan1msgBuf[buf][2] & 0xFFF0) + dataLength) ;

	ecan1msgBuf[buf][3] = data1;
	ecan1msgBuf[buf][4] = data2;
	ecan1msgBuf[buf][5] = data3;
	ecan1msgBuf[buf][6] = data4;

}

/******************************************************************************
 *Function Name:ecan_write_data_bytes
 * Description:
 * Puts data in bytes format to be transmitted in the buffers for transmission
 * Dependencies:
 *
 *
 * Parameters:
 * unsigned int buf: transmit buffer number
 * data length: Length of Data in Bytes to be transmitted
 * unsigned int data1-data4: the data that are to be transmitted
 *
 *
 * Output:
 * void
 *
 *Author Rap Canseco
 ******************************************************************************/
void ecan_write_data_bytes(unsigned int buf,CAN_DATA data){


    unsigned int datachunk1,datachunk2,datachunk3,datachunk4;

    datachunk1 = data.data2;
    datachunk1 = datachunk1 << 8; // move data2 to the upper nibble
    datachunk1 ^= data.data1;          // place data1 to the lower nibble

    datachunk2 = data.data4;
    datachunk2 = datachunk2 << 8; // move data4 to the upper nibble
    datachunk2 |= data.data3;          // place data3 to the lower nibble

    datachunk3 = data.data6;
    datachunk3 = datachunk3 << 8; // move data6 to the upper nibble
    datachunk3 |= data.data5;          // place data5 to the lower nibble

    datachunk4 = data.data8;
    datachunk4 = datachunk4 << 8; // move data8 to the upper nibble
    datachunk4 |= data.data7;          // place data7 to the lower nibble



    ecan1WriteTxMsgBufData(buf,data.data_length,datachunk1,datachunk2,datachunk3,
                                                datachunk4);



}



/******************************************************************************
 *Function Name:ecan1DisableRXFilter
 * Description:
 * disables filter
 * Dependencies:
 *
 *
 * Parameters:
 * unsigned int n: filter number
 *
 * Output:
 * void
 *
 *
 ******************************************************************************/
void ecan1DisableRXFilter(int n)
{
unsigned int *fltEnRegAddr;
   C1CTRL1bits.WIN=1;
   fltEnRegAddr = (unsigned int *)(&C1FEN1);
   *fltEnRegAddr = (*fltEnRegAddr) & (0xFFFF - (0x1 << n));
   C1CTRL1bits.WIN=0;

}



/*FROM ECAN function*/

/******************************************************************************
 *Function Name:ecan1WriteMessage
 * Description:
 * ECAN1 buffer loaded with Identifiers and Data
 * just a test function that has a predefined message and source calls the
 * functions to write the message to the buffer
 * Dependencies:
 * ecan1WriteTxMsgBufId
 * ecan1WriteTxMsgBufData
 *
 *
 * Parameters:
 * void
 *
 * Output:
 * void
 *
 *
 ******************************************************************************/
void ecan1WriteMessage(void){

/* Writing the message for Transmission
ecan1WriteTxMsgBufId(unsigned int buf, long txIdentifier, unsigned int ide,
 * unsigned int remoteTransmit);
ecan1WriteTxMsgBufData(unsigned int buf, unsigned int dataLength, unsigned int
 *  data1, unsigned int data2, unsigned int data3, unsigned int data4);

buf -> Transmit Buffer number

txIdentifier -> SID<10:0> : EID<17:0>

ide = 0 -> Message will transmit standard identifier
ide = 1 -> Message will transmit extended identifier

remoteTransmit = 0 -> Normal message
remoteTransmit = 1 -> Message will request remote transmission

dataLength -> Data length can be from 0 to 8 bytes

data1, data2, data3, data4 -> Data words (2 bytes) each

*/

	ecan1WriteTxMsgBufId(0,0x1FFEFFFF,1,0);
	ecan1WriteTxMsgBufData(0,8,0x02F0,0xF0E0,0x0203,0xFF34);

}


/******************************************************************************
 *Function Name:ecan1WriteMessage
 * Description:
 * ECAN1 buffer loaded with Identifiers and Data
 * just a test function that has a predefined message and source calls the
 * functions to write the message to the buffer
 * Dependencies:
 * ecan1WriteTxMsgBufId
 * ecan1WriteTxMsgBufData
 *
 *
 * Parameters:
 * void
 *
 * Output:
 * void
 *
 *
 ******************************************************************************/
void ecan_write(unsigned int buf, long txIdentifier, unsigned int ide, unsigned int remoteTransmit
,mID *CanMsg){

/* Writing the message for Transmission
ecan1WriteTxMsgBufId(unsigned int buf, long txIdentifier, unsigned int ide,
 * unsigned int remoteTransmit);
ecan1WriteTxMsgBufData(unsigned int buf, unsigned int dataLength, unsigned int
 *  data1, unsigned int data2, unsigned int data3, unsigned int data4);

buf -> Transmit Buffer number

txIdentifier -> SID<10:0> : EID<17:0>

ide = 0 -> Message will transmit standard identifier
ide = 1 -> Message will transmit extended identifier

remoteTransmit = 0 -> Normal message
remoteTransmit = 1 -> Message will request remote transmission

dataLength -> Data length can be from 0 to 8 bytes

data1, data2, data3, data4 -> Data words (2 bytes) each

*/

        CAN_DATA data;
        data.data1 = CanMsg->data[0];
        data.data2 = CanMsg->data[1];
        data.data3 = CanMsg->data[2];
        data.data4 = CanMsg->data[3];
        data.data5 = CanMsg->data[4];
        data.data6 = CanMsg->data[5];
        data.data7 = CanMsg->data[6];
        data.data8 = CanMsg->data[7];
        data.data_length = CanMsg->data_length;

	ecan1WriteTxMsgBufId(buf,txIdentifier,ide,0);
        ecan_write_data_bytes(buf,data);
        C1TR01CONbits.TXREQ0=1;

       

}



/**
 *
 * @brief       sends the data in the CAN buffer with repeat for arbitration loss
 *
 * This function sends data to the CAN bus. It waits for the message to be
 * successfully sent but if message is not sent for a time it would timeout.
 * If a send fails due to loss in arbitration,a resend would be also done
 * assuming it is also within the specified timeout.
 * This function assumes an extended identifier
 *                                                                              \n
 * Requires:                                                                    \n
 *  Timer0
 *
 *
 * @param [in]  buffer    buffer of what to send
 * @param [in]  timeout   contains how many instruction cycles before skipping
 *
 *
 * @retval 0    successful
 * @retval 1    timed out
 * @retval 2    the module is in bus off
 */
unsigned int can_send_data_with_arb_repeat_extended(mID *CanMsg,unsigned int timeout)
{
    unsigned int buf = 0;//use buffer 0 for transmit
    unsigned int x,y,status;
    status = 0;
    ecan_write(buf, CanMsg->id, 0, 0,CanMsg);

     //wait for successful transmission


    x=0;
    y=timeout;
    while(C1TR01CONbits.TXREQ0){
        if(C1TR01CONbits.TXLARB0)   //if fail due to arbitration
            C1TR01CONbits.TXREQ0 = 1; //request again

        for(x=0;x<3690;x++);    //delay about 1 ms for 7.37mhz
        y++;
        if(y>2300){              //about 1 second trying.
            status = 1;         //timed out
            break;
        }
    }


    return status;

}





/******************************************************************************
 *Function Name:ecan_get_byte
 * Description:
 * gets the data part from the message mID
 * Dependencies:
 *
 *
 *
 * Parameters:
 * int bytenum: the byte number to be taken from the data num from 1 to 8
 * mID message: the message taken from CAN communication
 *
 * Output:
 * char bytedata
 *
 *
 ******************************************************************************/
char ecan_get_byte(int bytenum,mID *message)
{
    char thebyte;
    thebyte = 0;
    thebyte = message->data[bytenum-1];

    return thebyte;
}
/******************************************************************************
 *Function Name:ecan_bit_test
 * Description:
 * checks if the specific bit from a byte is 1
 * Dependencies:
 *
 *
 *
 * Parameters:
 * int bitnum: the bit number to be taken from the byte from 0 to 7
 * char thebyte: the byte for testing
 *
 * Output:
 * int 1 if set 0 if not set
 *
 *
 ******************************************************************************/

int ecan_bit_test(int bitnum,char thebyte)
{

    char temp;
    temp = 0;

    switch(bitnum){
        case 0:
            temp = thebyte & 0b00000001;
            break;
        case 1:
            temp = thebyte & 0b00000010;
            break;
        case 2:
            temp = thebyte & 0b00000100;
            break;
        case 3:
            temp = thebyte & 0b00001000;
            break;
        case 4:
            temp = thebyte & 0b00010000;
            break;
        case 5:
            temp = thebyte & 0b00100000;
            break;
        case 6:
            temp = thebyte & 0b01000000;
            break;
        case 7:
            temp = thebyte & 0b10000000;
            break;
        default:
            break;

    }
    if(temp){
        return 1;
    }
    return 0;
}



/******************************************************************************
 *Function Name:rxECAN1
 * Description:
 * moves the message from the DMA memory to RAM
 *
 * Dependencies:
 * This depends on the global variable ecan1msgBuf, a pointer to a memory
 * location for the DMA
 * Depends on the structure mID that constitutes the contents of a message
 * Parameters:
 * mID *message: a pointer to the message structure in  RAM
 *
 * Output:
 * void
 *
 *Other notes:
 *  Author:            Jatinder Gharoo
 ******************************************************************************/
void rxECAN1(mID *message)
{
	unsigned int ide=0;
	unsigned int srr=0;
	unsigned long id=0;

	/*
	Standard Message Format:
	Word0 : 0bUUUx xxxx xxxx xxxx
                    |____________|||
                        SID10:0   SRR IDE(bit 0)
	Word1 : 0bUUUU xxxx xxxx xxxx
                       |____________|
			  EID17:6
	Word2 : 0bxxxx xxx0 UUU0 xxxx
                  |_____||	 |__|
                  EID5:0 RTR   	  DLC
	word3-word6: data bytes
	word7: filter hit code bits

	Substitute Remote Request Bit
	SRR->	"0"	 Normal Message
			"1"  Message will request remote transmission

	Extended  Identifier Bit
	IDE-> 	"0"  Message will transmit standard identifier
	   		"1"  Message will transmit extended identifier

	Remote Transmission Request Bit
	RTR-> 	"0"  Message transmitted is a normal message
			"1"  Message transmitted is a remote message
	*/
	/* read word 0 to see the message type */
	ide=ecan1msgBuf[message->buffer][0] & 0x0001;
	srr=ecan1msgBuf[message->buffer][0] & 0x0002;

	/* check to see what type of message it is */
	/* message is standard identifier */
	if(ide==0)
	{
		message->id=(ecan1msgBuf[message->buffer][0] & 0x1FFC) >> 2;
		message->frame_type=CAN_FRAME_STD;
	}
	/* mesage is extended identifier */
	else
	{
		id=ecan1msgBuf[message->buffer][0] & 0x1FFC;
		message->id=id << 16;
		id=ecan1msgBuf[message->buffer][1] & 0x0FFF;
		message->id=message->id+(id << 6);
		id=(ecan1msgBuf[message->buffer][2] & 0xFC00) >> 10;
		message->id=message->id+id;
		message->frame_type=CAN_FRAME_EXT;
	}
	/* check to see what type of message it is */
	/* RTR message */
	if(srr==1)
	{
		message->message_type=CAN_MSG_RTR;
	}
	/* normal message */
	else
	{
		message->message_type=CAN_MSG_DATA;
		message->data[0]=(unsigned char)ecan1msgBuf[message->buffer][3];
		message->data[1]=
               (unsigned char)((ecan1msgBuf[message->buffer][3] & 0xFF00) >> 8);
		message->data[2]=(unsigned char)ecan1msgBuf[message->buffer][4];
		message->data[3]=
               (unsigned char)((ecan1msgBuf[message->buffer][4] & 0xFF00) >> 8);
		message->data[4]=(unsigned char)ecan1msgBuf[message->buffer][5];
		message->data[5]=
               (unsigned char)((ecan1msgBuf[message->buffer][5] & 0xFF00) >> 8);
		message->data[6]=(unsigned char)ecan1msgBuf[message->buffer][6];
		message->data[7]=(unsigned char)((ecan1msgBuf[message->buffer][6] & 0xFF00) >> 8);
		message->data_length=
                      (unsigned char)(ecan1msgBuf[message->buffer][2] & 0x000F);
	}
}





/*****************************************************************************

 Senslope specific functions
 ******************************************************************************/
/**
 *
 * @brief       initializes the clock for CAN module
 *
 *  Assumes internal clock of 7.37 mhz and communicating in 40khz
 * configures the clock using 23TQ
 *
 *
 *
 */
void node_ecan_clk_init(void){

            /*Timing formulas
     * FCY = FOSC/2; for FOSC = 20000000, FCY = 10000000
     * Note it is better to have a big N for TQ
     * Ftq = N*Fbaud
     * BRP = baudrate prescaler = (Fcy/(2Ftq))-1
     * assume Fbaud = 0.5Mbps, FCY = 10000000,BRP = 0
     * Then Ftq = 5000000, N = 10
     *FCY SHOULD BE DIVISIBLE BY N AS A RULE OF THUMB
     */

    /*Bit time = (Sync Segment + prop delay + phase Seg1 + phase seg2) = 10*/
    /*Tq distribution rules
     * Ntq = 8->25
     * Synch = 1, Prop segment = 1-8, phase segment 1-8, phase segment2(1-8)
     * Ps1 + PropagDelay >= PS2
     * phase segment 2 > Synch jump
     * after Seg1 => 60-70 percent of Ntq
     * phase2 = > SJW
     * Current
     * Synch = 1Tq
     * PD = 4 Tq
     * PS1 = 2 Tq
     * PS2 = 3 Tq
     *
     * SJW = 2Tq
     *
     * note laging bawas ng isa ung equivalent kasi considered 1 ang 00
     */

    /*assumes internal RC 7.37 and 40kbps*/
     /*use FCY as clock*/
    C1CTRL1bits.CANCKS = 1;


    /*Synchronization Jump width set to 4 Tq*/
    /*1Tq = 00  2Tq = 01 ... 4Tq = 11 */
    C1CFG1bits.SJW = 0b11;
    //C1CFG1bits.SJW = 0b01;


    /*Phase segment 2 is not programmable*/
    //  C1CFG2bits.SEG2PHTS = 0;
    /* 1 programmable 0 not programmable*/
    C1CFG2bits.SEG2PHTS = 1;


    /*bus line is sampled 3 times at sample point*/
    C1CFG2bits.SAM = 1;

       /*bus line is sampled 1 times at sample point*/
   // C1CFG2bits.SAM = 0;


    /*Propagation Segment time is 8 Tq*/
    C1CFG2bits.PRSEG = 0x7; //8

     /*Phase segment 1 time is set to 7 Tq*/
    /*1Tq = 000 -> 8Tq = 111 */
    C1CFG2bits.SEG1PH = 0x6;//7
  

     /*Phase segment 2 time is 7 Tq*/
    /*1Tq = 000 -> 8Tq = 111 */
    C1CFG2bits.SEG2PH = 0x6;//7
    
    
    /*AT 10 mips*/

     C1CFG1bits.BRP = 0x1;
    



}




/**
 *
 * @brief       initializes the CAN module
 *
 * Initializes the CAN1 module of the controller and uses. DMA0 and DMA2
 * as the buffer memory for all of its data. All of the messages are
 * assumed to be using extended ids
 *
 *
 *
 *
 */
void node_ecan_init(void){
        can_set_ports();
        dma0init();
	dma2init();
/* Request Configuration Mode */

	C1CTRL1bits.REQOP=4;
	while(C1CTRL1bits.OPMODE!=4);



	node_ecan_clk_init();

	C1FCTRLbits.DMABS=0b000;	/* 4 CAN Message Buffers in DMA RAM */

/*	Filter Configuration

	ecan1WriteRxAcptFilter(int n, long identifier, unsigned int exide,
                                   unsigned int bufPnt,unsigned int maskSel)

	n = 0 to 15 -> Filter number

	identifier -> SID <10:0> : EID <17:0>

	exide = 0 -> Match messages with standard identifier addresses
	exide = 1 -> Match messages with extended identifier addresses

	bufPnt = 0 to 14  -> RX Buffer 0 to 14
	bufPnt = 15 -> RX FIFO Buffer

	maskSel = 0	->	Acceptance Mask 0 register contains mask
	maskSel = 1	->	Acceptance Mask 1 register contains mask
	maskSel = 2	->	Acceptance Mask 2 register contains mask
	maskSel = 3	->	No Mask Selection

*/

        // use filter 2,accept with id 1, extended id, buffer 2,mask 0
        ecan1WriteRxAcptFilter(2,node_id*8,0,2,0);



/*	Mask Configuration

	ecan1WriteRxAcptMask(int m, long identifierMask, unsigned int mide,
                                                     unsigned int exide)

	m = 0 to 2 -> Mask Number

	identifier -> SID <10:0> : EID <17:0>

	mide = 0 -> Match either standard or extended address message
        if filters match
	mide = 1 -> Match only message types that correpond
         to 'exide' bit in filter

	exide = 0 -> Match messages with standard identifier addresses
	exide = 1 -> Match messages with extended identifier addresses

*/
        //configure mask0, look at every bit, match with standard id only
	ecan1WriteRxAcptMask(0,0x1FFFFFFF,0,0);
        //ecan1WriteRxAcptMask(0,0x00000000,0,0);


/* Enter Normal Mode */
	C1CTRL1bits.REQOP=0;
	while(C1CTRL1bits.OPMODE!=0);




/* ECAN transmit/receive message control */

	C1RXFUL1=C1RXFUL2=C1RXOVF1=C1RXOVF2=0x0000;
	C1TR01CONbits.TXEN0=1;	      /* ECAN1, Buffer 0 is a Transmit Buffer */
	C1TR01CONbits.TXEN1=0;         /* ECAN1, Buffer 1 is a Receive Buffer */
        C1TR23CONbits.TXEN2 = 0;         /*ECAN1, Buffer 2 is a receive buffer*/
	C1TR01CONbits.TX0PRI=0b11;	   /* Message Buffer 0 Priority Level */
	C1TR01CONbits.TX1PRI=0b11;         /* Message Buffer 1 Priority Level */
        C1TR23CONbits.TX2PRI = 0b11;






        IEC2bits.C1IE = 1;
	C1INTEbits.TBIE = 1;
	C1INTEbits.RBIE = 1;

        //interrupt for error handling
        C1INTEbits.ERRIE;

        //invalid message interrupt
        C1INTEbits.IVRIE;
}



/**
 *
 * @brief       checks for data in can bus
 *
 * This function checks if data is available in the CAN bus. The message is placed
 * in the buffer pointer that is passed through this function.
 *
 *
 * @param [out]  CanMsg  datatype where all of the data is placed
 *
 *
 * @retval 0    nothing received
 * @retval !0   message received
 */
unsigned char can_check_for_datain_extended(mID *CanMsg)
{
    if(C1RXFUL1bits.RXFUL2){//received something
        CanMsg->buffer = 2; //point to buffer 2
        rxECAN1(CanMsg);

         C1RXFUL1bits.RXFUL2 = 0;
       return 1;
    }
    //did not receive anything
    return 0;

  
}


/**
 *
 * @brief       decides which command is given by the master
 *
 * It checks the msgid and verifies if it is a broadcast or node specific.
 * It lets pass all of the broadcast command. If a node specific command was
 * seen, a check would be done to see if it is for the specific node. If it is
 * not it blocks it and just outputs a zero. Requires that a message is in the
 * buffer.
 *
 * @param [in]  CanMsg     contains the data that was received
 * @param [in]  nodeid      unique id of the specific CAN node
 *
 *R
 * @retval 0    command is not for the node
 * @retval !0   command given by the master
 */
unsigned char can_process_commands_extended(mID *CanMsg,unsigned long nodeid)
{
    unsigned long idH;
    unsigned long idL;
    unsigned int command;

    command = 0;
    if(CanMsg->data[0] > MAXCANBROADCAST){
        //node specific
        idH = CanMsg->data[1];
        idL = CanMsg->data[2];
        if(( idH<<8 | idL) != nodeid) //id did not match
            command = 0;
        else
            command = CanMsg->data[0];
    }else // broadcast
        command = CanMsg->data[0];

    return command;

}