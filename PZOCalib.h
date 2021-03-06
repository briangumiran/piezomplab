/* 
 * File:   PZOCalib.h
 * Author: Brian
 *
 * Created on November 6, 2015, 1:41 PM
 *
 * Piezo calibration values in another header
 *
 * BAWAL ANG ID na nagtatapos sa B
 */

#ifndef PZOCALIB_H
#define	PZOCALIB_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef PIEZONUM
    //PIEZO5 for PUGB
    #if (PIEZONUM == 5) //5 ni ivy
        #define PC_FAC +1.01
        #define PC_CON -7.8809
        #define PZO_NODE_ID 0x11 //0x13 dapat
    //PIEZO3 for PUGT
    #elif (PIEZONUM == 3) // 3 ni ivy
        #define PC_FAC +0.9964
        #define PC_CON -0.3008
        #define PZO_NODE_ID 0xFF //0X15
    //piezo11 for sinB
    #elif (PIEZONUM == 11) // 11 ni ivy
        #define PC_FAC +1.0047
        #define PC_CON -4.2838
        #define PZO_NODE_ID 0x21 //0X21  node 33 //singit sagitna
    //PIEZO2 for MAMB
//    #elif (PIEZONUM == 2) // 2 ni ivy
//        #define PC_FAC 1.0051
//        #define PC_CON 0.134
//        #define PZO_NODE_ID 0x1F //node 30
    #elif (PIEZONUM == 90)   //arq logger, old nodes 1 dati
        #define PC_FAC 1.0051
        #define PC_CON 0.134
        #define PZO_NODE_ID 0xFF //ARQ data logger standard ID
    //PIEZO6 FOR HUMB
//    #elif (PIEZONUM == 6) // 6 ni ivy
//        #define PC_FAC 1.0045
//        #define PC_CON 3.8073
//        #define PZO_NODE_ID 0xFF   //ARQ data logger standard ID for HUMB
    //PIEZO4 FOR HUMT
    #elif (PIEZONUM == 4)
        #define PC_FAC 1.0002
        #define PC_CON -10.004
        #define PZO_NODE_ID 0xFF //ARQ data logger
        //PIEZO8 FOR OSLT
    #elif (PIEZONUM == 8) //busted oslt
        #define PC_FAC 0.9991
        #define PC_CON -5.1826
        #define PZO_NODE_ID 0x28 //5-26 kasi yung nodes, node 27
    #elif(PIEZONUM == 21)
        #define PC_FAC 0.9996
        #define PC_CON 12.809
        #define PZO_NODE_ID 0x28
    #elif(PIEZONUM == 1) // 1-RA, MAMBDUE (new)
        #define PC_FAC 1.0006
        #define PC_CON 7.6922
        #define PZO_NODE_ID 0xFF //ARQ standard piezo/message id
    #elif(PIEZONUM == 6) // 6-RA, PUGDUE (new)
        #define PC_FAC 1.0004
        #define PC_CON 3.6487
        #define PZO_NODE_ID 0xFF //ARQ standard piezo/message id
    #elif(PIEZONUM == 2) // 6-RA, PUGDUE (new)
        #define PC_FAC 1.0038
        #define PC_CON 1.7049
        #define PZO_NODE_ID 0xFF //ARQ standard piezo/message id
    #else
        #define PC_FAC +1.006
        #define PC_CON -7.8809
        #define PZO_NODE_ID 0xFF
    #endif
#endif


#ifdef	__cplusplus
}
#endif

#endif	/* PZOCALIB_H */

