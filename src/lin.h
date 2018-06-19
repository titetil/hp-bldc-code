/*********************************************************************
 *
 *              LIN-Master Include File
 *
 *********************************************************************
 * /ingroup         LIN
 * FileName:        ELINMInt.h
 * Dependencies:    ELINMInt.def
 * Processor:       18Fxxxx
 * Compilers:       Microchip C Compiler -> mcc C18 v2.20 or better
 * Assembler:       MPASMWIN 02.70.02 or higher
 * Linker:          MPLINK 2.33.00 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the "Company") is intended and supplied to you, the Company’s
 * customer, for use solely and exclusively with products manufactured
 * by the Company. 
 *
 * The software is owned by the Company and/or its supplier, and is 
 * protected under applicable copyright laws. All rights are reserved. 
 * Any use in violation of the foregoing restrictions may subject the 
 * user to criminal sanctions under applicable laws, as well as to 
 * civil liability for the breach of the terms and conditions of this 
 * license.
 
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES, 
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT, 
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR 
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *
 * LIN Master header file, for EUSART enabled 18F parts
 *
 * Author               Date            Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * CG                   09/4/03         Initial Release (Maestro 1.0)
 ********************************************************************/

#ifndef _ELINMINT_H          /* Used to avoid duplicate inclusion */ 
#define     _ELINMINT_H     

/*********************************************************************
 *
 * General purpose typedef's used by LIN Master libray.
 *
 * Remove these definations if they are already defined in one of your
 * application files.  Make sure that corresponding header file is
 * included before including this header file.
 ********************************************************************/

typedef  enum  _BOOL
         {
             FALSE = 0, 
             TRUE
         }
                      BOOL;                                             /* defintion of boolean types */ 
typedef  unsigned  char BYTE;                                           /* definition of BYTE */ 

#include     "ELINMInt.def"                                             /* .DEF file is included here - necessary! */ 


/* start position to fill TX vector - the buffer used in transmission shall be loaded
 * with the data to be transmitted from this position because position 0 is loaded with
 * SYNC character and position one contains the ID */


extern  void Transmit_LIN_8Bytes( BYTE  ID, BYTE  B0, BYTE  B1, BYTE  B2, BYTE  B3, BYTE  B4, BYTE  B5, BYTE  B6, 
                                  BYTE  B7 );
extern  void Receive_ETAT_PADD( void  );
extern  void Receive_Diag( char  id );
extern  void Transmit_LIN_Byte( BYTE  ID, BYTE  B0 );
extern  void Transmit_LIN_3Bytes( BYTE  ID, BYTE  B0, BYTE  B1, BYTE  B2 );
extern  void EnableMCP201( void  );


#define     ELIMINT_TXMSG_INIT     2


/* definitions to make Microchip C and Hitech C compatible */

#if defined  (  MCHP_C18  )

/ TXSTA bits definitions */
#define     TXSTA_TX9D         TXSTAbits.TX9D
#define     TXSTA_TRMT         TXSTAbits.TRMT
#define     TXSTA_BRGH         TXSTAbits.BRGH
#define     TXSTA_SENDB        TXSTAbits.SENDB
#define     TXSTA_SYNC         TXSTAbits.SYNC
#define     TXSTA_TXEN         TXSTAbits.TXEN
#define     TXSTA_TX9          TXSTAbits.TX9
#define     TXSTA_CSRC         TXSTAbits.CSRC

#define     RCSTA_RX9D         RCSTAbits.RX9D
#define     RCSTA_OERR         RCSTAbits.OERR
#define     RCSTA_FERR         RCSTAbits.FERR
#define     RCSTA_ADDEN        RCSTAbits.ADDEN
#define     RCSTA_CREN         RCSTAbits.CREN
#define     RCSTA_SREN         RCSTAbits.SREN
#define     RCSTA_RX9          RCSTAbits.RX9
#define     RCSTA_SPEN         RCSTAbits.SPEN

#define     BAUDCON_ABDEN      BAUDCONbits.ABDEN
#define     BAUDCON_WUE        BAUDCONbits.WUE
#define     BAUDCON_BRG16      BAUDCONbits.BRG16
#define     BAUDCON_SCKP       BAUDCONbits.SCKP
#define     BAUDCON_RCIDL      BAUDCONbits.RCIDL
#define     BAUDCON_ABDOVF     BAUDCONbits.ABDOVF

#define     INTCON_GIE         INTCONbits.GIE
#define     INTCON_GIEH        INTCONbits.GIEH
#define     INTCON_PEIE        INTCONbits.PEIE
#define     INTCON_GIEL        INTCONbits.GIEL
#define     INTCON_TMR0IE      INTCONbits.TMR0IE
#define     INTCON_TMR0IF      INTCONbits.TMR0IF
#define     INTCON_RBIF        INTCONbits.RBIF

#define     PIR1_TXIF          PIR1bits.TXIF
#define     PIR1_RCIF          PIR1bits.RCIF

#define     PIE1_TXIE          PIE1bits.TXIE
#define     PIE1_RCIE          PIE1bits.RCIE

#define     IPR1_TXIP          IPR1bits.TXIP
#define     IPR1_TXIP          IPR1bits.TXIP

#endif

#if defined  (  HITECH_C18  )

#define     TXSTA_TX9D         TX9D
#define     TXSTA_TRMT         TRMT
#define     TXSTA_BRGH         BRGH
#define     TXSTA_SENDB        SENDB
#define     TXSTA_SYNC         SYNC
#define     TXSTA_TXEN         TXEN
#define     TXSTA_TX9          TX9
#define     TXSTA_CSRC         CSRC

#define     RCSTA_RX9D         RX9D
#define     RCSTA_OERR         OERR
#define     RCSTA_FERR         FERR
#define     RCSTA_ADDEN        ADDEN
#define     RCSTA_CREN         CREN
#define     RCSTA_SREN         SREN
#define     RCSTA_RX9          RX9
#define     RCSTA_SPEN         SPEN

#define     BAUDCON_ABDEN      ABDEN
#define     BAUDCON_WUE        WUE
#define     BAUDCON_BRG16      BRG16
#define     BAUDCON_SCKP       SCKP
#define     BAUDCON_RCIDL      RCIDL
#define     BAUDCON_ABDOVF     ABDOVF

#define     INTCON_GIE         GIE
#define     INTCON_GIEH        GIEH
#define     INTCON_PEIE        PEIE
#define     INTCON_GIEL        GIEL
#define     INTCON_TMR0IE      TMR0IE
#define     INTCON_TMR0IF      TMR0IF
#define     INTCON_RBIF        RBIF

#define     PIR1_TXIF          TXIF
#define     PIR1_RCIF          RCIF

#define     PIE1_TXIE          TXIE
#define     PIE1_RCIE          RCIE

#define     IPR1_TXIP          TXIP
#define     IPR1_TXIP          TXIP

#endif


/* Constants used to set the different flags that control the protocol's behavior */

//#define       ELINMINT_TXBIT                  0x01                    /* TX state active flag */
//#define       ELINMINT_RXBIT                  0x02                    /* RX state active flag */
#define     ELINMINT_ERRORBIT              0x04                 /* ERROR detected flag */ 
#define     ELINMINT_IDLEBIT               0x08                 /* Bus IDLE state flag */ 

#define     ELINMINT_EXTENDED_ID           0x3E                 /* extended message id - this is an special ID */ 
#define     ELINMINT_EXTENDED_ID1          0x3F                 /* extended message id1 - reserved for future expansion */ 
#define     ELINMINT_WAKEUP_BYTE           0x80                 /* wake-up byte generated by a slave or the master 
                                                                  to wake-up the network from sleep */

/******************************************************************************
* !! Error Codes associated with the protocol !!
******************************************************************************/

#define     ELINMINT_NO_ERROR              0x00                 /* NO ERROR Detected ! */ 
#define     ELINMINT_THMIN_ERROR           0x10                 /* ERROR CODE: HEADER time too short */ 
#define     ELINMINT_THMAX_ERROR           0x20                 /* ERROR CODE: HEADER time too long */ 
#define     ELINMINT_TFMIN_ERROR           0x30                 /* ERROR CODE: FRAME time too short */ 
#define     ELINMINT_TFMAX_ERROR           0x40                 /* ERROR CODE: FRAME time too long */ 
#define     ELINMINT_CHECKSUM_ERROR        0x50                 /* ERROR CODE: Checksum incorrect */ 
#define     ELINMINT_DATA_ERROR            0x60                 /* ERROR CODE: Data Transmitted different from received */ 
#define     ELINMINT_FRAMING_ERROR         0x70                 /* ERROR CODE: Data received incorrectly -> Framing Error */ 

/********************************************************************
                        IMPORTANT:

The handler uses about ELINMINT_NINST_HANDLER_MIN instructions. 

The following defines check the minimum handler time against the 
interrup period and if the handler time exceeds the interrup period 
then an error message is generated.

*********************************************************************/

#define     ELINMINT_NINST_HANDLER_MIN     112L                 /* number of inst. run by the protocol during an Int. */ 

/* here the necessary time to run the protocol during an int. is calculated as a function of
 * the uC's clock frequency (CLOCK_FREQ) and the number of inst. of the LIN int. handler */

#define     ELINMINT_INT_HANDLER_TIME      ( ( 1000000L * ( 4 * ( ELINMINT_NINST_HANDLER_MIN + 5 ) ) ) / CLOCK_FREQ    \
                                              )

/* after that the processing time of the protocol is compared with the interrupt. 
 * execution time and if smaller ( interrupt period > processing time)
 * then set an error message. */

#if ELINMINT_INT_HANDLER_TIME  >  ELINMINT_INTERRUPT_PERIOD
#error "LIN TIMING NOT VIABLE - INTERRUPT PERIOD TOO SMALL !!"
#endif

/* calculate the bit time in usec. */
#define     ELINMINT_BIT_PERIOD             1000000L / ELINMINT_BAUD

/*  calculate the ratio between the int. period and the bit time (1/BAUD) */
#define     ELINMINT_INT_PER_BAUD_RATIO     ELINMINT_BIT_PERIOD / ELINMINT_INTERRUPT_PERIOD

/********************************************************************
                        IMPORTANT:

The handler calculates the limit times for header and frame based on
the relation between the number of interrupts and the number of bits
to be transmitted. Therefore for higher precision is recommended that 
the period of the interrupt where the handler is invoked to be small.

The correction factor calculates the ratio between the bit transmission 
time and the interrup period. Once it's calculated the correction factor 
is applied to the minimum and maximum header and frame times.
This is done because the LIN Interrupt handler in fact counts the number
of interrupts. Once this factor is applied the number of bits 
transmitted and/or received and the time this operations took can be 
checked.
*********************************************************************/

#define     ELINMINT_CORRECTION_FACTOR      ( 100L * ELINMINT_BIT_PERIOD ) / ELINMINT_INTERRUPT_PERIOD /* relation betwenn int. and  times */ 
#define     ELINMINT_SLEEP_TIMEOUTB         25000L                                              /* bus idle timeout (defines the sleep time) */ 
#define     ELINMINT_SLEEP_TIMEOUT          ( ( ELINMINT_SLEEP_TIMEOUTB * ELINMINT_CORRECTION_FACTOR ) / 100L ) /* idle bus in number of interrupts */ 


#define     ELINMINT_DELAY_TO_WAKEUP        5L * ELINMINT_CORRECTION_FACTOR /* from Twudel, (4+1)*(Ints to bits ratio) */ 

#define     ELINMINT_TBIT                   1000000L / ELINMINT_BAUD    /* bit time in microseconds */ 


/* here the minimum and maximum header times in number of bits is set */

#define     ELINMINT_THEADER_MINB           34L                         /* minimum number of bits in the header */ 
#define     ELINMINT_THEADER_MAXB           ( ( ( ELINMINT_THEADER_MINB + 1 ) * 14L ) / 10L ) /*  minimum number of bits in the header */ 

/* here the minimum and maximum header times in number of interrupts is calculated */

#define     ELINMINT_THEADER_MIN            ( ELINMINT_THEADER_MINB * ELINMINT_CORRECTION_FACTOR ) / 100 /* minimum header time in interrupts */ 
#define     ELINMINT_THEADER_MAX            ( ELINMINT_THEADER_MAXB * ELINMINT_CORRECTION_FACTOR ) / 100 /* maximum header time in interrupts */ 

/* serial port related constants */

#define     ELINMINT_TXSTA_INIT             0x26                        /* transmit enable+High BaudRate+TSR Empty */ 
#define     ELINMINT_RCSTA_INIT             0x90                        /* serial Port Enabled+enable receiver */ 
#define     ELINMINT_BAUDCON_INIT           0x48                        /* receiver Idle+16bit baud rate generator */ 
#define     ELINMINT_PIE1_INIT              0x00                        /* no serial interrupt */ 

/* standard values used by the protocol */

#define     ELINMINT_DUMMY_VALUE            0xFF                        /* dummy byte */ 
#define     ELINMINT_SYNC_VALUE             0x55                        /* sync value */ 

/**********************************************************************
/ ID union, used to define the ID and also allow the bit by bit access
*********************************************************************/

typedef  union  ELINMINT_ID
         {
             unsigned  char ID;

             struct
             {
                 unsigned  ID0 : 1;
                 unsigned  ID1 : 1;
                 unsigned  ID2 : 1;
                 unsigned  ID3 : 1;
                 unsigned  ID4 : 1;
                 unsigned  ID5 : 1;
                 unsigned  ID6 : 1;
                 unsigned  ID7 : 1;
             }              IDbits;
         }                             ELINMINT_ID;


/**********************************************************************
/ Message size union, used to define the size of a message and at the 
/ same time to allow the bit by bit access
*********************************************************************/

typedef  union  ELINMINT_MESSAGE_SIZE
         {
             unsigned  char SIZE;

             struct
             {
                 unsigned  SIZE0 : 1;
                 unsigned  SIZE1 : 1;
                 unsigned  SIZE2 : 1;
                 unsigned  SIZE3 : 1;
                 unsigned  SIZE4 : 1;
                 unsigned  SIZE5 : 1;
                 unsigned  SIZE6 : 1;
                 unsigned  SIZE7 : 1;
             }              SIZEbits;
         }                             ELINMINT_MESSAGE_SIZE;

/*********************************************************************
/ CRC union, used to define the checksum used by LIN and at the 
/ same time to allow the bit by bit access
*********************************************************************/

typedef  union  ELINMINT_CRC
         {
             int     CRC;
             BYTE    CRCL;
             struct
             {
                 unsigned  CRC0  : 1;
                 unsigned  CRC1  : 1;
                 unsigned  CRC2  : 1;
                 unsigned  CRC3  : 1;
                 unsigned  CRC4  : 1;
                 unsigned  CRC5  : 1;
                 unsigned  CRC6  : 1;
                 unsigned  CRC7  : 1;
                 unsigned  CRC8  : 1;
                 unsigned  CRC9  : 1;
                 unsigned  CRC10 : 1;
                 unsigned  CRC11 : 1;
                 unsigned  CRC12 : 1;
                 unsigned  CRC13 : 1;
                 unsigned  CRC14 : 1;
                 unsigned  CRC15 : 1;
             }       CRCbits;
         }                             ELINMINT_CRC;

/*********************************************************************
/ STATUS union, used to define the main status in a way to permit the 
/ bit by bit access
*********************************************************************/


#define     ELINMINT_IDLEBIT_POSITION     0x08                          /* IDLE bit position in status byte, must be kept up to date */ 

typedef  union  ELINMINT_STATUS
         {
             BYTE    ELINMIntStatusByte;
             struct
             {
                 unsigned  TX         : 1;                              /* transmitting active flag */ 
                 unsigned  RX         : 1;                              /* reception active flag */ 
                 unsigned  ERROR      : 1;                              /* error flag */ 
                 unsigned  IDLE       : 1;                              /* protocol IDLE flag */ 
                 unsigned  ERROR_BIT0 : 1;                              /* error definition nibble */ 
                 unsigned  ERROR_BIT1 : 1;
                 unsigned  ERROR_BIT2 : 1;
                 unsigned  ERROR_BIT3 : 1;
             }       ELINMINTSTS;
         }                        ELINMINT_STATUS;

/**********************************************************************
/  ELINMINT_STATUS1 union, used to define auxiliar status in a way to 
/  permit the bit by bit access
**********************************************************************/

typedef  union  ELINMINT_STATUS1
         {
             BYTE    ELINMIntStatusByte;
             struct
             {
                 unsigned  WAKEUP          : 1;                         /* wake-up in process flag */ 
                 unsigned  HEADER          : 1;                         /* header in transmission flag */ 
                 unsigned  FRAME           : 1;                         /* frame in transmission flag */ 
                 unsigned  EXTENDED        : 1;                         /* Extended message size flag */ 
                 unsigned  WAKEUP_RECEIVED : 1;                         /* wake-up signal received from slave */ 
                 unsigned  WAKEUP_SENT     : 1;                         /* wake-up signal sent to slaves */ 
                 unsigned  SLEEP_TIMEOUT   : 1;                         /* bus idle time-out flag */ 
             }       ELINMINTSTS;
         }                        ELINMINT_STATUS1;


/*********************************************************************
 * Function:    BYTE _ELINMIntInitialize(void)
 *
 * PreCondition:
 *
 * Input:

 * Output:      0=NO Error
 *              !0=ERROR CODE
 *
 * Side Effects:
 *
 * Stack Requirements:
 *
 * Overview:    This procedure Initializes:
 *                      The Baud Rate
 *                      Reception Interrupt (enabled)
 *                      Protocol Status bytes - > put protocol in IDLE 
 *                      Interbyte timing
 *                      
 ********************************************************************/
BYTE _ELINMIntInitialize( void  );

/*********************************************************************
 * Function:    void ELINMIntHandler(void)
 
 *
 * PreCondition:    
 *
 * Input:           
 *
 * Output:          
 *
 * Side Effects:    
 *
 * Stack Requirements: 
 *
 * Overview:    This procedure process LIN interrupt related tasks
 *
 ********************************************************************/
void ELINMIntHandler( void  );

/*********************************************************************
 * Function:    void _ELINMIntSendMessage(BYTE id,char size,unsigned int fmin,unsigned int fmax)
 *
 * PreCondition:mELINMIntInitialize invoked    
 *
 * Input:       The pointer to the original message to be sent
 *                      The ID
 *                      The size of the message
 *                      The minimum frame time
 *                      The maximum frame time
 *                                      
 *
 * Output:  
 *
 * Side Effects:    
 *
 * Stack Requirements: 
 *
 * Overview:    This function sends a message thru LIN 
 *
 ********************************************************************/
void _ELINMIntSendMessage( BYTE  id, char  size, unsigned int  fmin, unsigned int  fmax );

/*********************************************************************
 * Function:    void _ELINMIntResetProtocol(BYTE code)
 *
 * PreCondition:
 *
 * Input:       code - any eventual error code
 *                                      
 * Output:                      
 *
 * Side Effects:
 *
 * Stack Requirements:
 *
 * Overview:    This procedure resets the protocol and if necessary 
 *              sets the error flags
 *
 ********************************************************************/
void _ELINMIntResetProtocol( BYTE  code );

/*********************************************************************
 * Function:    void _ELINMIntReceiveMessage(BYTE tag,BYTE id,char size);
 *
 * PreCondition:mELINMIntInitialize invoked
 *
 * Input:       The tag (message identifier)
 *              The ID used in reception
 *              The size of the message
 *
 * Output:  
 *
 * Side Effects:    
 *
 * Stack Requirements: 
 *
 * Overview:    This function receive a message thru LIN 
 *
 ********************************************************************/
void  _ELINMIntReceiveMessage( BYTE  tag, BYTE  id, char  size );

/*********************************************************************
 * Function:    BYTE *_ELINMIntGetPointer(char _ELINMInt_tag, BYTE _ELINMInt_position)
 *
 * PreCondition:mELINMIntInitialize invoked
 *              TX Buffer Available (using mELINMIntTXBufferAvailable())
 *
 * Input:
 *              _ELINMInt_tag - The specific message tag 
 *              to be associated with the TX message buffer.
 *              _ELINMInt_position - the position of the buffer to be 
 *              returned. 
 *
 * Output:      A pointer to the TX data buffer to be loaded.
 *
 * Side Effects:
 *
 * Stack Requirements:
 *
 * Overview:    This function takes a tag and associates it with a
 *              TX buffer and returns a BYTE pointer that allows the user
 *              to load the message to be transmitted. The tag lets the
 *              user identifies his(hers) message in case of an error
 *
 * OBS:         This function is not invoked directly by the user but
 *              through a macro (mELINMIntGetTXPointer(tag))
 ********************************************************************/
BYTE * _ELINMIntGetPointer( char  _ELINMInt_tag, BYTE  _ELINMInt_position );

/*********************************************************************
 * Function:    BYTE _ELINMIntCalcIDParity(ELINMINT_ID ELINM_idtr)
 *
 * PreCondition:
 *
 * Input:       ID 
 *
 * Output:      ID with parity bits set
 *
 * Side Effects:    
 *
 * Stack Requirements:
 *
 * Overview:    This functions calculates the Parity.
 *
 ********************************************************************/
BYTE  _ELINMIntCalcIDParity( ELINMINT_ID  ELINM_idtr );

/*********************************************************************
 * Macro:       mELINMIntSendMessage(tag,i,s)
 *
 * PreCondition:mELINMIntInitialize() invoked. 
 *              TX Buffer available 
 *
 * Input:       tag - message tag
 *              i - ID of the message
 *              s  - size in bytes
 *                              
 * Output:              
 *
 * Side Effects:        
 *
 * Note:        This macro calculates the frame's maximum and and minimum 
 *              times and passes all this info to the related function. If 
 *              the size of the message is passed to the macro as a constant 
 *              then the calculations are done in compile time, therefore 
 *              saving both time and space, being the recommended method.
 *              It will also work when a variable is passed as a parameter,
 *              but the calculations will be done in real-time.
 *                              
 ********************************************************************/


#define     mELINMIntSendMessage( tag, i, s )     _ELINMIntSendMessage ( i, s, ( ( ( ( ( s + 3 ) * 15L ) + 44L ) * ELINMINT_CORRECTION_FACTOR    \
                                                     ) / 100L ), ( ( ( ( ( ( ( ( s + 3 ) * 15L ) + 44L ) + 1L ) *  \
                                                     14L ) / 10L ) * ELINMINT_CORRECTION_FACTOR ) / 100L ) )
/*********************************************************************
 * Macro:       mELINMIntTXBufferAvailable()
 *
 * PreCondition:mELINMIntInitialize() invoked.
 *
 * Input:
 *
 * Output:      1 = buffer available for TX
 *              0 = NO buffer available
 *
 * Side Effects:
 *
 * Note:
 *
 ********************************************************************/
#define     mELINMIntTXBufferAvailable( )     ( _ELINMIntStatus.ELINMINTSTS.IDLE )

/*********************************************************************
 * Macro:       mELINMIntRXBufferAvailable()
 *
 * PreCondition:mELINMIntInitialize() invoked.                 
 *
 * Input:               
 *
 * Output:      1 = buffer available for RX
 *              0 = NO buffer available
 *
 * Side Effects:
 *
 * Note:                        
 *
 ********************************************************************/
#define     mELINMIntRXBufferAvailable( )     ( _ELINMIntStatus.ELINMINTSTS.IDLE )

/*********************************************************************
 * Macro:       mELINMIntGetMessageTag()
 *
 * PreCondition:mELINMIntInitialize() invoked.                                 
 *
 * Input:               
 *
 * Output:      Returns the tag of the received message.
 *
 * Side Effects:
 *
 * Note:                                
 *
 ********************************************************************/
#define     mELINMIntGetMessageTag( )         ( _ELINMIntMessageTag )

/*********************************************************************
 * Macro:       mELINMIntTXErrorDetected()
 *
 * PreCondition:mELINMIntInitialize() invoked.                                  
 *
 * Input:               
 *
 * Output:      1 = error in transmission detected!
 *              0 = NO error detected
 *      
 * Side Effects:
 *
 * Note:                        
 *
 ********************************************************************/
#define     mELINMIntTXErrorDetected( )     ( _ELINMIntStatus.ELINMINTSTS.ERROR )

/*********************************************************************
 * Macro:       mELINMIntRXErrorDetected()
 *
 * PreCondition:mELINMIntInitialize() invoked.                                  
 *
 * Input:               
 *
 * Output:      0 = error in reception detected
 *                      
 * Side Effects:
 *
 * Note:                        
 *
 ********************************************************************/
#define     mELINMIntRXErrorDetected( )     ( _ELINMIntStatus.ELINMINTSTS.ERROR )

/*********************************************************************
 * Macro:       mELINMIntTXErrorCode(tag)
 *
 * PreCondition:Error been detected by mELINMIntRXErrorDetected()
 *
 * Input:       tag - the identification of the message that returned
 *              a problem.      
 *
 * Output:      The error code according to the table:
 *                      
 *                              0x00    // NO ERROR
 *      ELINMINT_THMIN_ERROR    0x10    // ERROR CODE: HEADER time too short
 *      ELINMINT_THMAX_ERROR    0x20    // ERROR CODE: HEADER time too long
 *      ELINMINT_TFMIN_ERROR    0x30    // ERROR CODE: FRAME time too short                     
 *      ELINMINT_TFMAX_ERROR    0x40    // ERROR CODE: FRAME time too long
 *      ELINMINT_CHECKSUM_ERROR 0x50    // ERROR CODE: Checksum 
 *      ELINMINT_DATA_ERROR     0x60    // ERROR CODE: Data Transmitted different from received
 *      ELINMINT_FRAMING_ERROR  0x70    // ERROR CODE: Data received incorrectly -> Framing Error
 *
 * Side Effects:
 *
 * Note:                        
 *
 ********************************************************************/

#define     mELINMIntTXErrorCode( tag )     ( _ELINMIntStatus.ELINMIntStatusByte & 0xF0 )

/*********************************************************************
 * Macro:       mELINMIntTXErrorTag()
 *
 * PreCondition:Error been detected by mELINMIntRXErrorDetected()
 *
 * Input:               
 *
 * Output:      Returns the Tag of the message that presented a problem 
 *                      
 * Side Effects:
 *
 * Note:                        
 *
 ********************************************************************/
#define     mELINMIntTXErrorTag( )          ( _ELINMIntMessageTag )

/*********************************************************************
 * Macro:       mELINMIntRXErrorTag()
 *
 * PreCondition:Error been detected by mELINMIntRXErrorDetected()
 *
 * Input:               
 *
 * Output:      Returns the Tag of the message that with error
 *                      
  * Side Effects:
 *
 * Note:                        
 *
 ********************************************************************/
#define     mELINMIntRXErrorTag( )           ( _ELINMIntMessageTag )

/*********************************************************************
 * Macro:       mELINMIntRXErrorCode(tag)
 *
 * PreCondition:Error been detected by mELINMIntRXErrorDetected() 
 *
 * Input:       tag - the identification of the message that returned
 *              a problem.      
 *
 * Output:      The error code in the HIGH nibble according to the table:
 *                      
 *                              0x00    // NO ERROR
 *      ELINMINT_THMIN_ERROR    0x10    // ERROR CODE: HEADER time too short
 *      ELINMINT_THMAX_ERROR    0x20    // ERROR CODE: HEADER time too long
 *      ELINMINT_TFMIN_ERROR    0x30    // ERROR CODE: FRAME time too short                     
 *      ELINMINT_TFMAX_ERROR    0x40    // ERROR CODE: FRAME time too long
 *      ELINMINT_CHECKSUM_ERROR 0x50    // ERROR CODE: Checksum
 *      ELINMINT_DATA_ERROR     0x60    // ERROR CODE: Data Transmitted different from received
 *      ELINMINT_FRAMING_ERROR  0x70    // ERROR CODE: Data received incorrectly -> Framing Error
 *
 * Side Effects:
 *
 * Note:                        
 *
 ********************************************************************/
#define     mELINMIntRXErrorCode( tag )      ( _ELINMIntStatus.ELINMIntStatusByte & 0xF0 )

/*********************************************************************
 * Macro:       mELINMIntGetTXPointer(tag)
 *
 * PreCondition:mELINMIntInitialize() invoked.           
 *              Available buffer detected by mELINMIntTXBufferAvailable()                                       
 *
 * Input:       tag - the message tag, an identification of the message to 
 *              be sent
 *
 * Output:      Returns a data pointer to the TX buffer 
 *
 * Side Effects:        
 *
 * Note:        This macro takes the number by which that specific 
 *              message is going to be identified (tag) and
 *              returns the data pointer to the buffer where the message 
 *              shall be loaded for transmission.               
 *
 ********************************************************************/
#define     mELINMIntGetTXPointer( tag )     _ELINMIntGetPointer ( tag, ELIMINT_TXMSG_INIT )

/*********************************************************************
 * Macro:       mELINMIntRXStatus(Tag)
 *
 * PreCondition:Reception completed, checked by mELINMIntMessageReceived
 *
 * Input:       tag - the identification of the message
 *
 * Output:      Returns error code according to the table:
 *
 *                              0x00    // NO ERROR Detected
 *      ELINMINT_THMIN_ERROR    0x10    // ERROR CODE: HEADER time too short
 *      ELINMINT_THMAX_ERROR    0x20    // ERROR CODE: HEADER time too long
 *      ELINMINT_TFMIN_ERROR    0x30    // ERROR CODE: FRAME time too short
 *      ELINMINT_TFMAX_ERROR    0x40    // ERROR CODE: FRAME time too long
 *      ELINMINT_CHECKSUM_ERROR 0x50    // ERROR CODE: Checksum
 *      ELINMINT_DATA_ERROR     0x60    // ERROR CODE: Data Transmitted different from received
 *      ELINMINT_FRAMING_ERROR  0x70    // ERROR CODE: Data received incorrectly -> Framing Error
 *
 * Side Effects:
 *
 * Note:
 *
 ********************************************************************/
#define     mELINMIntRXStatus( Tag )         ( _ELINMIntStatus.ELINMIntStatusByte & 0xF0 )


/*********************************************************************
 * Macro:       mELINMIntTXStatus(Tag)
 *
 * PreCondition:Transmission completed, checked by mELINMIntMessageSent
 *
 * Input:       tag - the identification of the message
 *
 * Output:      Returns error code according to the table:
 *
 *                              0x00    // NO ERROR Detected
 *      ELINMINT_THMIN_ERROR    0x10    // ERROR CODE: HEADER time too short
 *      ELINMINT_THMAX_ERROR    0x20    // ERROR CODE: HEADER time too long
 *      ELINMINT_TFMIN_ERROR    0x30    // ERROR CODE: FRAME time too short
 *      ELINMINT_TFMAX_ERROR    0x40    // ERROR CODE: FRAME time too long
 *      ELINMINT_CHECKSUM_ERROR 0x50    // ERROR CODE: Checksum
 *      ELINMINT_DATA_ERROR     0x60    // ERROR CODE: Data Transmitted different from received
 *      ELINMINT_FRAMING_ERROR  0x70    // ERROR CODE: Data received incorrectly -> Framing Error
 *
 * Side Effects:
 *
 * Note:
 *
 ********************************************************************/
#define     mELINMIntTXStatus( Tag )                 ( _ELINMIntStatus.ELINMIntStatusByte & 0xF0 )

/*********************************************************************
 * Macro:       mELINMIntRXMessageAvailable()
 *
 * PreCondition:mELINMIntReceiveMessage(tag,i,s) invoked
 *
 * Input:
 *
 * Output:      1 = message was received
 *              0 = no message was received 
 *
 * Side Effects:
 *
 * Note:        This macro will flag the reception of a message even
 *              if an error was encountered, therefore before reading the
 *              message is necessary to check it's integrity with
 *              mELINMIntRXErrorDetected()
 *
 ********************************************************************/
#define     mELINMIntRXMessageAvailable( )           ( _ELINMIntStatus.ELINMINTSTS.IDLE )


/*********************************************************************
 * Macro:       mELINMIntGetRXPointer(tag)
 *
 * PreCondition:mELINMIntInitialize() invoked.           
 *              Available buffer detected by mELINMIntRXBufferAvailable()                                       
 *
 * Input:       Tag - the message tag, an identification of the message to 
 *              be received
 *
 * Output:      Returns a data pointer to the buffer containing the 
 *              received message
 *
 * Side Effects:        
 *
 * Note:        This macro takes the number by which that specific 
 *              message was identified (tag) and returns 
 *              the data pointer used to read the message
 *
 ********************************************************************/
#define     mELINMIntGetRXPointer( tag )             _ELINMIntGetPointer ( tag, 0 )

/*********************************************************************
 * Macro:       mELINMIntReceiveMessage(tag,i,s)
 *
 * PreCondition:mELINMIntInitialize() invoked.           
 *              Available buffer detected by mELINMIntRXBufferAvailable()                                       
 *
 * Input:       tag of the message
 *              i = ID of the message
 *              s = size of the message
 *
 * Output:      
 *
 * Side Effects:        
 *
 * Note:        This macro sends a request to the slaves to receive 
 *              's' data bytes.                 
 *
 ********************************************************************/
#define     mELINMIntReceiveMessage( tag, i, s )     _ELINMIntReceiveMessage ( tag, i, s )

/*********************************************************************
 * Macro:       mELINMIntMessageReceived(tag)                   
 *
 * PreCondition:mELINMIntReceiveMessage(tag,i,s) invoked
 *
 * Input:       tag - the identification number the message             
 *
 * Output:      1 = message was received.       
 *              0 = the message wasn't received yet
 *
 * Side Effects:        
 *
 * Note:        This macro will flag the reception of a message even
 *              if an error was encountered, therefore before reading the
 *              message is necessary to check it's integrity
 *
 ********************************************************************/
#define     mELINMIntMessageReceived( tag )     ( _ELINMIntStatus.ELINMINTSTS.IDLE )

/*********************************************************************
 * Macro:       mELINMIntCheckWakeUPReceived()                  
 *
 * PreCondition:
 *
 * Input:                       
 *
 * Output:      1 = wake-up signal was received.        
 *              0 = NO wake-up signal was received.     
 *
 * Side Effects:        
 *
 * Note:        This macro will flag the reception of a wake-up signal
 *              This signal will remain active until a transmission or 
 *              reception is completed. The sleep timeout bit is reset.                
 *
 ********************************************************************/
#define     mELINMIntCheckWakeUPReceived( )     ( _ELINMIntStatus1.ELINMINTSTS.WAKEUP_RECEIVED )

/*********************************************************************
 * Macro:       mELINMIntSendWakeUPSignal()                     
 *
 * PreCondition:Protocol Initialized by ELINMIntInitialize().
 *
 * Input:                       
 *
 * Output:      
 *
 * Side Effects:        
 *
 * Note:        This macro sends a Wake-Up signal and resets the
 *              sleep timeout flag
 *
 ********************************************************************/
#define     mELINMIntSendWakeUPSignal( )                                                                              \
                if( _ELINMIntStatus1.ELINMIntStatusByte = 0x20 )                                                      \
                    TXREG=0x80

/*********************************************************************
 * Macro:       mELINMIntMessageSent(tag)
 *
 * PreCondition:mELINMIntSendMessage invoked with the same tag                                 
 *
 * Input:       tag - The message indentification number        
 *
 * Output:      1 = transmission completed
 *              0 = Transmission not completed
 * Side Effects:
 *
 * Note:                        
 *
 ********************************************************************/
#define     mELINMIntMessageSent( tag )     ( _ELINMIntStatus.ELINMIntStatusByte & ELINMINT_IDLEBIT_POSITION )


/*********************************************************************
 * Macro:       mELINMIntSleepTimeOut()
 *
 * PreCondition:mELINMIntInitialize() invoked.
 *
 * Input:       
 *
 * Output:      
 *              1 = Time-Out detected
 *              0 = NO Time-Out detected
 *
 * Side Effects:
 *
 * Note:        The sleep time-out is issued after 25000 bits-time
 *              of silence in the bus.                                  
 *
 ********************************************************************/
#define     mELINMIntSleepTimeOut( )        ( _ELINMIntStatus1.ELINMINTSTS.SLEEP_TIMEOUT )

/*********************************************************************
 * Macro:       mELINMIntInitialize()
 *
 * PreCondition:
 *
 * Input:       
 *
 * Output:      
 *              0 = Initialization OK
 *              !0= Initialization Error Code
 *
 * Side Effects:
 *
 * Note:        
 *
 ********************************************************************/
#define     mELINMIntInitialize( )          ( _ELINMIntInitialize ( ) )

/*********************************************************
/ to allow access to the variables by user's program
**********************************************************/

extern  ELINMINT_STATUS  _ELINMIntStatus;
extern  ELINMINT_STATUS1 _ELINMIntStatus1;
extern  BYTE             _ELINMIntMessageTag;
extern  BYTE             _ELINMIntMessageBuffer[ ];

#endif  /* #ifndef _ELINMINT_H     Used to avoid duplicate inclusion */ 

