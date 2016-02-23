/* 
 * File:   senslope_can.h
 * Author: rap
 *
 * Created on October 9, 2014, 4:01 PM
 */

#ifndef SENSLOPE_CAN_H
#define	SENSLOPE_CAN_H

#ifdef	__cplusplus
extern "C" {
#endif



#define CAN_MSG_DATA	0x01 // message type
#define CAN_MSG_RTR	0x02 // data or RTR
#define CAN_FRAME_EXT	0x03 // Frame type
#define CAN_FRAME_STD	0x04 // extended or standard


/* message structure in RAM */
typedef struct{
	/* keep track of the buffer status */
	unsigned char buffer_status;    //!< keeps track of buffer status
	/* RTR message or data message */
	unsigned char message_type; //!< to know if RTR or not
	/* frame type extended or standard */
	unsigned char frame_type; //! to know if extended or standad
	/* buffer being used to reference the message */
	unsigned char buffer;  //!< used to choose which buffer to use
	/* 29 bit id max of 0x1FFF FFFF
	*  11 bit id max of 0x7FF */
	unsigned long id; //!< message identifier 29 bit
	/* message data */
	unsigned char data[8]; //!< databytes of the CAN
	/* received message data length */
	unsigned char data_length; //!< number of bytes passed in the transaction
}mID;

typedef struct{
    unsigned char data_length; //!< number of bytes passed in the transaction
    unsigned char data1;  //!< databytes of the CAN
    unsigned char data2; //!< databytes of the CAN
    unsigned char data3; //!< databytes of the CAN
    unsigned char data4; //!< databytes of the CAN
    unsigned char data5; //!< databytes of the CAN
    unsigned char data6; //!< databytes of the CAN
    unsigned char data7; //!< databytes of the CAN
    unsigned char data8; //!< databytes of the CAN
}CAN_DATA;//!< helper intermediate struct









 /*ECANCONFIG PART*/
 /* CAN Baud Rate Configuration 		*/
//#define FCAN  	40000000
//#define BITRATE 1000000
//#define NTQ 	20		// 20 Time Quanta in a Bit Time
//#define BRP_VAL		((FCAN/(2*NTQ*BITRATE))-1)

/* CAN Message Buffer Configuration */
#define  ECAN1_MSG_BUF_LENGTH 	4
typedef unsigned int ECAN1MSGBUF [ECAN1_MSG_BUF_LENGTH][8];
extern ECAN1MSGBUF  ecan1msgBuf __attribute__((space(dma)));


/* Function Prototype 	*/

void dma0init(void);
 void dma2init(void);
void can_set_ports(void);
int extract_2nd_lower_10_bits(unsigned char* data);
int extract_lower_10_bits(unsigned char* data);






/*ECANDRV part*/
void ecan1WriteRxAcptFilter(int n, long identifier, unsigned int exide,unsigned int bufPnt,unsigned int maskSel);
 void ecan1WriteRxAcptMask(int m, long identifierMask, unsigned int mide,unsigned int exide);

 void ecan1WriteTxMsgBufId(unsigned int buf, long txIdentifier, unsigned int ide, unsigned int remoteTransmit);
 void ecan1WriteTxMsgBufData(unsigned int buf, unsigned int dataLength, unsigned int data1, unsigned int data2, unsigned int data3, unsigned int data4);
//extern void ecan_write_data_bytes(unsigned int buf, CAN_DATA data);
void ecan1DisableRXFilter(int n);

void ecan_write_data_bytes(unsigned int buf,CAN_DATA data);



 void rxECAN1(mID *message);
 void ecan1WriteMessage(void);
 char ecan_get_byte(int bytenum,mID *message);
void ecan_write(unsigned int buf, long txIdentifier, unsigned int ide, unsigned int remoteTransmit
,mID *CanMsg);
char ecan_get_byte(int bytenum,mID *message);
int ecan_bit_test(int bitnum,char thebyte);


/*Senslope Specific*/
void node_ecan_init(void);
void node_ecan_clk_init(void);



 #define MAXCANBROADCAST                    99//!< defines the limit for broadcast commands
unsigned char can_check_for_datain_extended(mID *CanMsg);
unsigned char can_process_commands_extended(mID *CanMsg,unsigned long nodeid);
unsigned int can_send_data_with_arb_repeat_extended(mID *CanMsg,unsigned int timeout);


#ifdef	__cplusplus
}
#endif

#endif	/* SENSLOPE_CAN_H */

