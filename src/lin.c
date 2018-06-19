/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/**
* \defgroup  LIN       Lin communication
* \brief Configuration of the PWM for BVH2.
* This module contains all functions for PWM read and write .
* \version 1.0 - 2013-02-15
* \section pwm_hist History    
* <b>V1.0 - 2013-02-15 - ADQ - First numbered version</b>    
*/
/**
 * \ingroup   LIN
 * \file      lin.c
 * \brief     Lin-Master routine
 * \note        Target CPU    : PIC16F1936\n
 *              Compiler      : HI_TECH PICC (v9.81)
 * \copyright (c) TI Automotive - All rights reserved
 * \author    TI Automotive - JPL - Joachim Plantholt
 * \requirement \b TIR-SW-LIN-001 Lin Init
 * \requirement \b TIR-SW-LIN-002 LinHandler(void)
 * \requirement \b TIR-SW-LIN-003 Lin_Rx
 * \requirement \b TIR-SW-LIN-004 Parse Input command
 * \requirement \b TIR-SW-LIN-005 LIN_Tx
 * \test \b TC_SW-LIN-001 Lin Test
 * \details   This file configurate and do the LIN communication with extern user. Must be disable for the production.
 **/
/*~T*/

/**
 * FileName:        ELINMInt.c\n
 * Dependencies:    ELINMInt.h\n
 * Processor:       18Fxxxx\n
 * Compilers:       Microchip C Compiler -> mcc C18 v2.20 or better\n
 * Assembler:       MPASMWIN 02.70.02 or higher\n
 * Linker:          MPLINK 2.33.00 or higher\n
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
 * This FW provides the necessary functions to create a Master LIN 
 * node driver using an EUSART enabled 18F part.
 *
 * Author               Date            Comment\n
 * CG                   09/4/03         Initial Release (Maestro 1.0)
**/
/*~T*/
/********************************* Header-Files *********************************/
#include     "lin.h"
#include     "bldc.h"
#include     <htc.h>
/*~T*/
/********************************* Defines **************************************/
#define     _LINM18ESOURCE     /**< Seems to be not used */ 

#define     _CS_DATA_HIGH      RC5 = 1 /**< LIN treiber select for enabling */ 
#define     _CS_DATA_LOW       RC5 = 0 /**< LIN treiber select for disabling */ 

#define     FREQ_MULT          ( XTAL_FREQ ) / ( 4 MHZ ) /**< No usage */ 

#define     WaitMCP            DelayUs ( 200L ) /**< Wait for 200us macro */ 

#define     DelayUs( x )                                                                                              \
                {                                                                                                     \
                    unsigned char _dcnt;                                                                              \
                    if( x >= 4 )                                                                                      \
                        _dcnt = ( x * ( 32000000UL ) / 2 );                                                           \
                    else                                                                                              \
                        _dcnt = 1;                                                                                    \
                    while( --_dcnt > 0 )                                                                              \
                    {                                                                                                 \
                        asm( "nop" );                                                                                 \
                        asm( "nop" );                                                                                 \
                        continue;                                                                                     \
                    }                                                                                                 \
                } /**< Wait for parametered time macro */ 



BYTE                  _ELINMIntMessageTag;                              /**< message tag -> identifies one specific message */
ELINMINT_CRC          _ELINMIntRXCRC;                                   /**< reception CRC buffer */
ELINMINT_STATUS       _ELINMIntStatus;                                  /**< main status byte */
ELINMINT_STATUS1      _ELINMIntStatus1;                                 /**< second status byte */
ELINMINT_MESSAGE_SIZE _ELINMIntMessageSize;                             /**< message size */
ELINMINT_MESSAGE_SIZE _ELINMIntRXMessageSize;                           /**< reception message size */
BYTE                  _ELINMIntMessageBuffer[ ELINMINT_MAX_MESSAGE_SIZE + 3 ]; /**< message buffer */
BYTE                  _ELINMIntReadBack;                                /**< saves the last transmitted byte to be compared with feedback */

BYTE                  _ELINMIntMessageBufferPointer;                    /**< TX byte position pointer */
                char  _ELINMIntSpace;                                   /**< interbyte and interframe space */
unsigned        int   _ELINMIntTFrameMin;                               /**< timing control variable */
unsigned        int   _ELINMIntTFrameMax;                               /**< timing control variable */
unsigned        int   _ELINMIntTHeaderMin;                              /**< timing control variable */
unsigned        int   _ELINMIntTHeaderMax;                              /**< timing control variable */
unsigned  long        _ELINMIntSleepTimeout;                            /**< sleep time-out count variable */


/* Flags for EOL */
unsigned        char  ICT_STAMP_PRESENCE; /**< Flag for ICT stamp presence in EEPROM */
unsigned        char  FCT_STAMP_ABSENCE; /**< Flag for FCT stamp absence in EEPROM */
unsigned        char  TEMPERATURE_CALIBRATION; /**< Flag to enable the temperature calibration */
unsigned        char  VER_SW_VERSION_NUMBER; /**< Flag to enable the SW number validation */
unsigned        char  VER_SUPPLY_VOLTAGE_VALUE; /**< Flag to enable the supply voltage  verification */
unsigned        char  CURRENT_CALIBRATION; /**< Flag to enable the current phase calibration */
unsigned        char  SPEED_READING; /**< Flag to enable the reading of the speed */
unsigned        char  FCT_STAMP_WRITING; /**< Flag to enable the writing of the FCT stamp */
unsigned        char  LIN_DEACTIVATION; /**< Flag to enable the LIN deactivation */

unsigned        char  ui8_lin_calibration;


unsigned        char  ui8_b_DResServID_c; /**< First byte of LIN communication: SID */
unsigned        char  ui8_b_DResLocID_c; /**< Second byte of LIN communication: LID */
unsigned        char  ui8_b_DResB0_c; /**< Third byte of LIN communication: first sended value */
unsigned        char  ui8_b_DResB1_c; /**< Fourth byte of LIN communication: second sended value */
unsigned        char  ui8_b_DResB2_c; /**< Fifth byte of LIN communication: third sended value */
unsigned        char  ui8_b_DResB3_c; /**< Sixth byte of LIN communication: fourth sended value */
unsigned        char  ui8_b_DResB4_c; /**< Seventh byte of LIN communication: fifth sended value */
unsigned        char  ui8_b_DResB5_c; /**< Eightth byte of LIN communication: sixth sended value */

/*~I*/
#if defined  (  def_LIN  )  ||  defined  (  def_LIN_5ms  )
/*~A*/
/*~+:Transmit_LIN_8Bytes*/
/*~T*/
/**
 * \fn      void Transmit_LIN_8Bytes(BYTE ID,  BYTE B0, BYTE B1, BYTE B2, BYTE B3, BYTE B4, BYTE B5, BYTE B6, BYTE B7)
 * \brief   Send a 8-byte message to the slave
 * \param   ID BYTE
 * \param   B0 BYTE
 * \param   B1 BYTE
 * \param   B2 BYTE
 * \param   B3 BYTE
 * \param   B4 BYTE
 * \param   B5 BYTE
 * \param   B6 BYTE
 * \param   B7 BYTE
 * \pre     None
 * \post    None
 * \details Send a single message using fixed value tag, checking for the transmission of that specific message
 */
/*~T*/
BYTE  my_msg[ 8 ];                                                      /* message buffer, used for tests */
BYTE * pt;
BYTE  ErrorCode;
/*~F*/
void Transmit_LIN_8Bytes( BYTE  ID, BYTE  B0, BYTE  B1,
                          BYTE  B2, BYTE  B3, BYTE  B4,
                          BYTE  B5, BYTE  B6, BYTE  B7 )
/*~-*/
{
   /*~T*/
      /*****************************************************************************
      / Send a single message using fixed value tag
      / checking for the transmission of that specific message
      ***************************************************************************/

    while( mELINMIntTXBufferAvailable( ) == 0)  /* Wait TX buffer available */
    {
        ;
    }
    pt    = mELINMIntGetTXPointer( 5 );         /* get the available pointer and tag it's message as 3 */
    *pt++ = B0;                                 /* insert data */
    *pt++ = B1;
    *pt++ = B2;
    *pt++ = B3;
    *pt++ = B4;
    *pt++ = B5;
    *pt++ = B6;
    *pt++ = B7;

    mELINMIntSendMessage( 5, ID, 8 );           /* send message */
    if( ( ErrorCode = mELINMIntTXStatus( 5 ) ))  /* check if an TX error was detected (!=0) */
    {
        /* error handling - to be added at this point by the application */
    }


/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:Transmit_LIN_Byte*/
/*~T*/
/**
 * \fn      void Transmit_LIN_Byte(BYTE ID,  BYTE B0)
 * \brief   Send a 1-byte message to the slave
 * \param   ID BYTE
 * \param   B0 BYTE
 * \pre     None
 * \post    None
 * \details Send a single message using fixed value tag, checking for the transmission of that specific message
 */
/*~F*/
void Transmit_LIN_Byte( BYTE  ID, BYTE  B0 )
/*~-*/
{
   /*~T*/
     /*****************************************************************************
     / Send a single message using fixed value tag
     / checking for the transmission of that specific message
     *****************************************************************************/

   while( mELINMIntTXBufferAvailable( ) == 0)  /* Wait TX buffer available */
   {
       ;
   }
   pt    = mELINMIntGetTXPointer( 5 );         /* get the available pointer and tag it's message as 3 */
   *pt++ = B0;                                 /* insert data */

   mELINMIntSendMessage( 5, ID, 1 );           /* send message */
   if( ( ErrorCode = mELINMIntTXStatus( 5 ) )) /* check if an TX error was detected (!=0) */
   {
       /* error handling - to be added at this point by the application */
   }
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:Transmit_LIN_3Bytes*/
/*~T*/
/**
 * \fn      void Transmit_LIN_3Bytes(BYTE ID, BYTE B0, BYTE B1, BYTE B2)
 * \brief   Send a 3-byte message to the slave
 * \param   ID BYTE
 * \param   B0 BYTE
 * \param   B1 BYTE
 * \param   B2 BYTE
 * \pre     None
 * \post    None
 * \details Send a single message using fixed value tag, checking for the transmission of that specific message
 */
/*~F*/
void Transmit_LIN_3Bytes( BYTE  ID, BYTE  B0, BYTE  B1,
                          BYTE  B2 )
/*~-*/
{
   /*~T*/
     /*****************************************************************************
     / Send a single message using fixed value tag
     / checking for the transmission of that specific message
     ****************************************************************************/

   while( mELINMIntTXBufferAvailable( ) == 0)  /* Wait TX buffer available */
   {
       ;
   }
   pt    = mELINMIntGetTXPointer( 5 );         /* get the available pointer and tag it's message as 3 */
   *pt++ = B0;                                 /* insert data */
   *pt++ = B1;
   *pt++ = B2;


   mELINMIntSendMessage( 5, ID, 3 );           /* send message */
   if( ( ErrorCode = mELINMIntTXStatus( 5 ) )) /* check if an TX error was detected (!=0) */
   {
       /* error handling - to be added at this point by the application */
   }
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:Receive_Diag */
/*~T*/
/**
 * \fn      void Receive_Diag(char id)
 * \brief   Receive a 8-byte message from the slave.
 * \param   id char
 * \pre     None
 * \post    None
 * \sa      #ui8_b_DResServID_c 
 * \sa      #ui8_b_DResLocID_c 
 * \sa      #ui8_b_DResB0_c 
 * \sa      #ui8_b_DResB1_c
 * \sa      #ui8_b_DResB2_c 
 * \sa      #ui8_b_DResB3_c 
 * \sa      #ui8_b_DResB4_c 
 * \sa      #ui8_b_DResB5_c 
 * \sa      #my_msg 
 * \sa      #pt 
 * \details The message is stored in\n
 *              #ui8_b_DResServID_c\n
 *              #ui8_b_DResLocID_c\n
 *              #ui8_b_DResB0_c\n
 *              #ui8_b_DResB1_c\n
 *              #ui8_b_DResB2_c\n
 *              #ui8_b_DResB3_c\n
 *              #ui8_b_DResB4_c\n
 *              #ui8_b_DResB5_c\n
 */
/*~F*/
void Receive_Diag( char  id )
/*~-*/
{
   /*~T*/
    while( mELINMIntRXBufferAvailable( ) == 0)   /* if there is no RX buffer available wait*/
    {
        ;
    }
    ;
    mELINMIntReceiveMessage( 5, id, 8 );       /* request data using tag=5 (message number), ID=0x3d, size=8 */
    while( mELINMIntMessageReceived( 5 ) == 0)   /* wait until the message is received */
    {
      //asm("nop");
    }
    ;
    if( ( ErrorCode == mELINMIntRXStatus( 5 ) ))  /* check if an RX error was detected */
    {
      //asm("nop");
      //asm("nop");
    }
    else
    {
        pt          = mELINMIntGetRXPointer( 5 ); /* get the data pointer */
        my_msg[ 0 ] = *pt;                     /* read the message */
        pt++;                                  /* received message handling - to be added at this point */
        my_msg[ 1 ] = *pt;
        pt++;                                  /* received message handling - to be added at this point */
        my_msg[ 2 ] = *pt;
        pt++;                                  /* received message handling - to be added at this point */
        my_msg[ 3 ] = *pt;
        pt++;                                  /* received message handling - to be added at this point */
        my_msg[ 4 ] = *pt;
        pt++;                                  /* received message handling - to be added at this point */
        my_msg[ 5 ] = *pt;
        pt++;                                  /* received message handling - to be added at this point */
        my_msg[ 6 ] = *pt;
        pt++;                                  /* received message handling - to be added at this point */
        my_msg[ 7 ]        = *pt;

        ui8_b_DResServID_c = my_msg[ 0 ];
        ui8_b_DResLocID_c  = my_msg[ 1 ];
        ui8_b_DResB0_c     = my_msg[ 2 ];
        ui8_b_DResB1_c     = my_msg[ 3 ];
        ui8_b_DResB2_c     = my_msg[ 4 ];
        ui8_b_DResB3_c     = my_msg[ 5 ];
        ui8_b_DResB4_c     = my_msg[ 6 ];
        ui8_b_DResB5_c     = my_msg[ 7 ];

    }


/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:Receive_ETAT_PADD */
/*~T*/
/**
 * \fn      void Receive_ETAT_PADD(void)
 * \brief   Receive a 2-byte message from the slave.
 * \pre     None
 * \post    None
 * \details This function is configuring a start time where the speed is at maximum after a reset of the pump.
*/
/*~F*/
void Receive_ETAT_PADD( void  )
/*~-*/
{
   /*~T*/
    while( mELINMIntRXBufferAvailable( ) == 0)      /* if there is no RX buffer available wait */
    {
        ;
    }
    ;
    mELINMIntReceiveMessage( 5, 0x02, 2 );          /* request data using tag=5 (message number), ID=0x01, size=2 */
    while( mELINMIntMessageReceived( 5 ) == 0)      /* wait until the message is received */
    {
        ;
    }
    ;
    if( ( ErrorCode = mELINMIntRXStatus( 5 ) ))     /* check if an RX error was detected */
    {
                                                    /* error handling - to be added at this point by the application */
    }
    else
    {
        pt          = mELINMIntGetRXPointer( 5 );  /* get the data pointer */
        my_msg[ 0 ] = *pt;                         /* read the message */
        pt++;                                      /* received message handling - to be added at this point */
        my_msg[ 1 ] = *pt;
        pt++;
        my_msg[ 2 ] = *pt;
        pt++;
        my_msg[ 3 ] = *pt;
#ifdef def_update_PhaseAngle
        ui16_phase_angle = my_msg[ 1 ];
#endif
        //   ui16_start_time = (unsigned int)(my_msg[0]) + (unsigned int)(my_msg[1]<<8) ;
        ui8_lin_calibration = my_msg[ 2 ];     /* 1: temperature calibration 
                                                  2: current = 0A calibration
                                                  3: current = 6A calibration
                                                  oth: no calibration */
    }
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void EnableMCP201(void)*/
/*~T*/
/**
 * \fn      void EnableMCP201(void)
 * \brief   Enable the LIN driver
 * \pre     None
 * \post    None
 * \details The enabling of the Lin driver is explained in the microcontroller datasheet.
*/
/*~F*/
void EnableMCP201( void  )
/*~-*/
{
   /*~T*/
   /** The following routine is executed: */
   /** Disable treiber\n
    *  Wait for 200us\n
    *  Enable treiber\n
    *  Wait for 200us\n
    *  Disable treiber\n
    *  Wait for 200us\n
    *  Enable treiber */
   /** Then LIN treiber is enabled. To disable the LIN communication, the macro #_CS_DATA_LOW must be called. */
    _CS_DATA_LOW;
    WaitMCP  ;
    _CS_DATA_HIGH;
    WaitMCP  ;
    _CS_DATA_LOW;
    WaitMCP  ;
    _CS_DATA_HIGH;
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:BYTE _ELINMIntInitialize*/
/*~F*/
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
BYTE _ELINMIntInitialize( void  )

/*~-*/
{
   /*~T*/
    SPBRG                                = ELINMINT_SPBRG;                /* set Baud Rate (16 bits) */
    SPBRGH                               = ELINMINT_SPBRGH;
    TXSTA                                = ELINMINT_TXSTA_INIT;           /* initiates transmission and reception */
    RCSTA                                = ELINMINT_RCSTA_INIT;
    BAUDCON                              = ELINMINT_BAUDCON_INIT;
    PIE1                                 = ELINMINT_PIE1_INIT;
    _ELINMIntSleepTimeout                = ELINMINT_SLEEP_TIMEOUT;        /* init bus idle-to-sleep timing */
    _ELINMIntSpace                       = ELINMINT_INTERBYTE_SPACE;      /* start interbyte space */
    _ELINMIntStatus.ELINMIntStatusByte   = 0;                             /* clear status */
    _ELINMIntStatus1.ELINMIntStatusByte  = 0;                             /* clear status1 */
    _ELINMIntStatus.ELINMINTSTS.IDLE     = 1;                             /* return to idle state */
    return ( 0 );                                                         /* return 0 - OK */

/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void _ELINMIntResetProtocol*/
/*~F*/
/*********************************************************************
 * Function:    void _ELINMIntResetProtocol(BYTE code)
 *
 * PreCondition:
 *
 * Input:       BYTE code - any eventual error code
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
void _ELINMIntResetProtocol( BYTE  code )

/*~-*/
{
   /*~T*/
    _ELINMIntReadBack                   = RCREG;                          /* read the receive register to clear RCIF flag */
    _ELINMIntStatus1.ELINMIntStatusByte = 0;                              /* reset all aux. status conditions including */
    _ELINMIntRXCRC.CRC                  = 0;                              /* reset CRC */
    _ELINMIntStatus.ELINMIntStatusByte  = code;                           /* status code set. */
    _ELINMIntSleepTimeout               = ELINMINT_SLEEP_TIMEOUT;         /* init bus idle-to-sleep timing */

/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void ELINMIntHandler*/
/*~T*/
/*********************************************************************
 * Function:    void interrupt ELINMIntHandler(void)
 * PreCondition:        
 * Input:
 * Output:
 * Side Effects:
 * Stack Requirements:
 * Overview:    Interrupt Handler for EUSART based Master node LIN
 *              To be called by a timer based interrupt routine                 
 *
 ********************************************************************/

/*~F*/
void ELINMIntHandler( void  )

/*~-*/
{
   /*~I*/
    if( SENDB == 0) /* check if it is not sending break */

   /*~-*/
   {
      /*~I*/
        if( RCIF)     /* also checks if received something */

      /*~-*/
      {
         /*~A*/
         /*~+:RCIF*/
         /*~I*/
         if( _ELINMIntStatus.ELINMINTSTS.IDLE) /* check if in IDLE state */

         /*~-*/
         {
            /*~I*/
            if( ( RCSTA & 0x06 ) == 0) /* if received without error */

            /*~-*/
            {
               /*~A*/
               /*~+:(RCSTA&0x06)==0*/
               /*~I*/
               if( RCREG == ELINMINT_WAKEUP_BYTE) /* then check if it was an wake-up request */

               /*~-*/
               {
                  /*~T*/
                  _ELINMIntSleepTimeout = ELINMINT_SLEEP_TIMEOUT;            /* reinit bus-idle-to sleep timing */
                  _ELINMIntReadBack     = RCREG;                             /* read the receive register to clear RCIF flag */

                  /*~I*/
                  if( _ELINMIntStatus1.ELINMINTSTS.WAKEUP_SENT)              /* if an wake-up signal was sent from the master to the slave */

                  /*~-*/
                  {
                     /*~T*/
                     _ELINMIntStatus1.ELINMINTSTS.WAKEUP_SENT = 0; /* clear the flag */

                  /*~-*/
                  }
                  /*~O*/
                  /*~-2*/
                  else
                  {
                     /*~T*/
                     _ELINMIntStatus1.ELINMINTSTS.WAKEUP = 1; /* if wake-up signal set the wake-up flag */

                  /*~-*/
                  }
                  /*~E*/
               /*~-*/
               }
               /*~E*/
               /*~E*/
            /*~-*/
            }
            /*~E*/
         /*~-*/
         }
         /*~O*/
         /*~-2*/
         else
         {
            /*~I*/
            if( _ELINMIntStatus.ELINMINTSTS.TX) /* check if transmitting */

            /*~-*/
            {
               /*~A*/
               /*~+:_ELINMIntStatus.ELINMINTSTS.TX */
               /*~I*/
               if( ( RCSTA & 0x06 ) && _ELINMIntMessageBufferPointer)       /* if error and it's not in the BREAK byte (pointer==0) */

               /*~-*/
               {
                  /*~T*/
                  _ELINMIntResetProtocol(
                  ELINMINT_IDLEBIT + ELINMINT_ERRORBIT + ELINMINT_FRAMING_ERROR ); /* signal the error */

               /*~-*/
               }
               /*~O*/
               /*~-2*/
               else
               {
                  /*~T*/
                  /* if no error detected or error detected in BREAK (expected) */

                  /*~I*/
                  if( _ELINMIntMessageBufferPointer == 2) /* then check if it just sent the HEADER */

                  /*~-*/
                  {
                     /*~T*/
                     _ELINMIntStatus1.ELINMINTSTS.HEADER = 0; /* if so turn-off the header transmission flag (it will trigger the check of header max. time) */

                  /*~-*/
                  }
                  /*~E*/
                  /*~I*/
                  if( _ELINMIntSpace)               /* if interbyte space not zero decrement and go away */

                  /*~-*/
                  {
                     /*~T*/
                     _ELINMIntSpace--;

                  /*~-*/
                  }
                  /*~O*/
                  /*~-2*/
                  else
                  {
                     /*~I*/
                     if( _ELINMIntReadBack != RCREG) /* otherwise check if the byte read is the same as the sent one */

                     /*~-*/
                     {
                        /*~T*/
                        _ELINMIntResetProtocol(ELINMINT_IDLEBIT + ELINMINT_ERRORBIT + ELINMINT_DATA_ERROR ); /* if not signals the error */

                     /*~-*/
                     }
                     /*~O*/
                     /*~-2*/
                     else
                     {
                        /*~T*/
                        /* if byte read is the same as the sent one */

                        /*~I*/
                        if( _ELINMIntMessageSize.SIZE) /* and if there is still message to be sent */

                        /*~-*/
                        {
                           /*~T*/
                           _ELINMIntReadBack = RCREG; /* read the receive register to clear RCIF flag */
                           _ELINMIntReadBack = _ELINMIntMessageBuffer[ _ELINMIntMessageBufferPointer ];      /* load next byte to be sent */
                           TXREG = _ELINMIntReadBack; /* also load the read-back check byte */          _ELINMIntMessageSize.SIZE--; /* decrement the counter */
                           _ELINMIntMessageBufferPointer++; /* increment the pointer */
                           _ELINMIntSpace += ELINMINT_INTERBYTE_SPACE; /* start interbyte space counter */

                        /*~-*/
                        }
                        /*~O*/
                        /*~-2*/
                        else
                        {
                           /*~T*/
                           /* if all message already transmitted */

                           /*~I*/
                           if( _ELINMIntStatus.ELINMINTSTS.RX) /* and if it's set to receive */

                           /*~-*/
                           {
                              /*~T*/
                              _ELINMIntStatus.ELINMINTSTS.TX = 0; /* then clear the transmission flag */
                              _ELINMIntMessageBufferPointer  = 0; /* and reset the pointer */

                           /*~-*/
                           }
                           /*~O*/
                           /*~-2*/
                           else
                           {
                              /*~T*/
                              /* however, if it's not going to receive */

                              /*~T*/
                              _ELINMIntStatus1.ELINMINTSTS.FRAME = 0; /* then clear frame check flag (force the max. frame time to be checked) */
                              _ELINMIntResetProtocol( ELINMINT_IDLEBIT ); /* reset the protocol without error */

                           /*~-*/
                           }
                           /*~E*/
                        /*~-*/
                        }
                        /*~E*/
                     /*~-*/
                     }
                     /*~E*/
                  /*~-*/
                  }
                  /*~E*/
               /*~-*/
               }
               /*~E*/
               /*~E*/
            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~I*/
               if( _ELINMIntStatus.ELINMINTSTS.RX) /* if receiving */

               /*~-*/
               {
                  /*~A*/
                  /*~+:_ELINMIntStatus.ELINMINTSTS.RX*/
                  /*~I*/
                  if( RCSTA & 0x06) /* and if an error was detected */

                  /*~-*/
                  {
                     /*~T*/
                     _ELINMIntResetProtocol( ELINMINT_IDLEBIT + ELINMINT_ERRORBIT + ELINMINT_FRAMING_ERROR ); /* reset protocol with error */

                  /*~-*/
                  }
                  /*~O*/
                  /*~-2*/
                  else
                  {
                     /*~I*/
                     if( _ELINMIntRXMessageSize.SIZE) /* if no error detected and if there is still message to be received */

                     /*~-*/
                     {
                        /*~T*/
                        _ELINMIntMessageBuffer [ _ELINMIntMessageBufferPointer ] = RCREG; /* load message data byte in the buffer */
                        _ELINMIntRXMessageSize.SIZE--; /* decrement the counter */
                        _ELINMIntRXCRC.CRC += RCREG; /* add to CRC */

                        /*~I*/
                        if( _ELINMIntRXCRC.CRCbits.CRC8) /* if a carry bit from the lower byte */

                        /*~-*/
                        {
                           /*~T*/
                           _ELINMIntRXCRC.CRCL++; /* add the carry bit */

                        /*~-*/
                        }
                        /*~E*/
                        /*~T*/
                        _ELINMIntMessageBufferPointer++; /* increment pointer */

                     /*~-*/
                     }
                     /*~O*/
                     /*~-2*/
                     else
                     {
                        /*~T*/
                        /* if all data bytes already received */

                        /*~T*/
                        _ELINMIntStatus1.ELINMINTSTS.FRAME == 0;
                        _ELINMIntRXCRC.CRCL += RCREG + 1; /* check CRC(checksum) by XORing received CRC */

                        /*~I*/
                        if( _ELINMIntRXCRC.CRCL )

                        /*~-*/
                        {
                           /*~T*/
                           _ELINMIntResetProtocol( ELINMINT_IDLEBIT + ELINMINT_ERRORBIT + ELINMINT_CHECKSUM_ERROR );  /* if error reset protocol and signal error. if result != 0 error! */

                        /*~-*/
                        }
                        /*~O*/
                        /*~-2*/
                        else
                        {
                           /*~T*/
                           _ELINMIntResetProtocol( ELINMINT_IDLEBIT );    /* otherwise just reset protocol without error */

                        /*~-*/
                        }
                        /*~E*/
                     /*~-*/
                     }
                     /*~E*/
                  /*~-*/
                  }
                  /*~E*/
                  /*~E*/
               /*~-*/
               }
               /*~E*/
            /*~-*/
            }
            /*~E*/
         /*~-*/
         }
         /*~E*/
         /*~E*/
      /*~-*/
      }
      /*~E*/
   /*~-*/
   }
   /*~E*/
   /*~I*/
    if( _ELINMIntStatus.ELINMINTSTS.IDLE == 0 )

   /*~-*/
   {
      /*~A*/
      /*~+:_ELINMIntStatus.ELINMINTSTS.IDLE==0*/
      /*~I*/
      if( _ELINMIntStatus1.ELINMINTSTS.FRAME == 1) /* check flag bit of transmission of frame */

      /*~-*/
      {
         /*~I*/
         if( _ELINMIntStatus1.ELINMINTSTS.HEADER == 1) /* check flag bit of transmission of header */

         /*~-*/
         {
            /*~I*/
            if( _ELINMIntTHeaderMin) /* while the min. header time > 0 decrement it */

            /*~-*/
            {
               /*~T*/
               _ELINMIntTHeaderMin--;

            /*~-*/
            }
            /*~E*/
            /*~I*/
            if( _ELINMIntTHeaderMax) /* while the max. header time > 0 decrement it */

            /*~-*/
            {
               /*~T*/
               _ELINMIntTHeaderMax--;

            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~T*/
               /* however, if max. header time == 0 and header transmission not completed */
               _ELINMIntResetProtocol( ELINMINT_IDLEBIT + ELINMINT_ERRORBIT + ELINMINT_THMAX_ERROR ); /* then reset with error */

            /*~-*/
            }
            /*~E*/
         /*~-*/
         }
         /*~O*/
         /*~-2*/
         else
         {
            /*~T*/
            /* check flag bit of transmission of header */

            /*~I*/
            if( _ELINMIntTHeaderMin) /* check header minimum time */

            /*~-*/
            {
               /*~T*/
               _ELINMIntResetProtocol( ELINMINT_IDLEBIT + ELINMINT_ERRORBIT + ELINMINT_THMIN_ERROR ); /* if > 0 --> error */

            /*~-*/
            }
            /*~E*/
         /*~-*/
         }
         /*~E*/
         /*~I*/
         if( _ELINMIntTFrameMin) /* while min. frame time > 0 decrement it */

         /*~-*/
         {
            /*~T*/
            _ELINMIntTFrameMin--;

         /*~-*/
         }
         /*~E*/
         /*~I*/
         if( _ELINMIntTFrameMax) /* while max. frame time > 0 decrement it */

         /*~-*/
         {
            /*~T*/
            _ELINMIntTFrameMax--;

         /*~-*/
         }
         /*~O*/
         /*~-2*/
         else
         {
            /*~T*/
            /* however if the max. frame time ==0 and frame not completed */
            _ELINMIntResetProtocol( ELINMINT_IDLEBIT + ELINMINT_ERRORBIT + ELINMINT_TFMAX_ERROR );  /* signal error */

         /*~-*/
         }
         /*~E*/
      /*~-*/
      }
      /*~O*/
      /*~-2*/
      else
      {
         /*~T*/
         /* if frame transmission (or reception) completed */

         /*~I*/
         if( _ELINMIntTFrameMin)  /* check frame min. time */

         /*~-*/
         {
            /*~T*/
            _ELINMIntResetProtocol( ELINMINT_IDLEBIT + ELINMINT_ERRORBIT + ELINMINT_TFMIN_ERROR ); /* if >0 --> error */

         /*~-*/
         }
         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~E*/
   /*~-*/
   }
   /*~O*/
   /*~-2*/
   else
   {
      /*~I*/
        if( _ELINMIntStatus1.ELINMINTSTS.WAKEUP_SENT == 0)                  /* once the wake-up process is initiated */

      /*~-*/
      {
         /*~I*/
            if( _ELINMIntSleepTimeout) /* the sleep timeout is blocked */

         /*~-*/
         {
            /*~T*/
                _ELINMIntSleepTimeout--;

         /*~-*/
         }
         /*~O*/
         /*~-2*/
         else
         {
            /*~T*/
                _ELINMIntStatus1.ELINMINTSTS.SLEEP_TIMEOUT = 1;

         /*~-*/
         }
         /*~E*/
      /*~-*/
      }
      /*~E*/
   /*~-*/
   }
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:BYTE _ELINMIntCalcIDParity*/
/*~F*/
/*********************************************************************
 * Function:    BYTE _ELINMIntCalcIDParity(ELINMINT_ID ELINM_idtr)
 *
 * PreCondition:
 *
 * Input:       An ID 
 *
 * Output:      The same ID with parity bits set
 *
 * Side Effects:    
 *
 * Stack Requirements:
 *
 * Overview:    This functions calculates the Parity and then sends the ID.
 *              It doesn't check if the TXREG is available according to the
 *              LIN spec.
 *
 ********************************************************************/
BYTE _ELINMIntCalcIDParity( ELINMINT_ID  ELINM_idtr )

/*~-*/
{
   /*~T*/
    ELINM_idtr.ID &= 0x3F;                                                /* ensure parity bits are clean */

   /*~I*/
    if( ELINM_idtr.IDbits.ID0)                                            /* calculates the Parity bit 6 */

   /*~-*/
   {
      /*~T*/
        ELINM_idtr.ID ^= 0x40;

   /*~-*/
   }
   /*~E*/
   /*~I*/
    if( ELINM_idtr.IDbits.ID1 )

   /*~-*/
   {
      /*~T*/
        ELINM_idtr.ID ^= 0x40;

   /*~-*/
   }
   /*~E*/
   /*~I*/
    if( ELINM_idtr.IDbits.ID2 )

   /*~-*/
   {
      /*~T*/
        ELINM_idtr.ID ^= 0x40;

   /*~-*/
   }
   /*~E*/
   /*~I*/
    if( ELINM_idtr.IDbits.ID4 )

   /*~-*/
   {
      /*~T*/
        ELINM_idtr.ID ^= 0x40;

   /*~-*/
   }
   /*~E*/
   /*~T*/
    ELINM_idtr.IDbits.ID7 = 1;                                            /* calculates the Parity bit 7 */

   /*~I*/
    if( ELINM_idtr.IDbits.ID1 )

   /*~-*/
   {
      /*~T*/
        ELINM_idtr.ID ^= 0x80;

   /*~-*/
   }
   /*~E*/
   /*~I*/
    if( ELINM_idtr.IDbits.ID3 )

   /*~-*/
   {
      /*~T*/
        ELINM_idtr.ID ^= 0x80;

   /*~-*/
   }
   /*~E*/
   /*~I*/
    if( ELINM_idtr.IDbits.ID4 )

   /*~-*/
   {
      /*~T*/
        ELINM_idtr.ID ^= 0x80;

   /*~-*/
   }
   /*~E*/
   /*~I*/
    if( ELINM_idtr.IDbits.ID5 )

   /*~-*/
   {
      /*~T*/
        ELINM_idtr.ID ^= 0x80;

   /*~-*/
   }
   /*~E*/
   /*~T*/
    return ( ( BYTE )ELINM_idtr.ID );

/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void _ELINMIntSendMessage*/
/*~F*/
/*********************************************************************
 * Function:    void _ELINMIntSendMessage(BYTE _ELINM_idr,char _ELINM_size,unsigned 
 *                                        int _ELINM_fmin,unsigned int _ELINM_fmax)
 *
 * PreCondition:ELINMIntInitialize invoked
 *              TX Buffer Available (using mELINMIntTXBufferAvailable())
 *
 * Input:       The ID of the message
 *              The size of the message
 *              The minimum frame time
 *              The maximum frame time
 *
 * Output:
 *
 * Side Effects:Error Code is reset
 *
 * Stack Requirements:
 *
 * Overview:    This function sends a message. It is not to be invoked by the user directly but through the mELINMIntSendMessage
 *              macro, which calculates the frame timing.
 *
 ********************************************************************/
void _ELINMIntSendMessage( BYTE  _ELINM_idr,
                           char  _ELINM_size,
                           unsigned int  _ELINM_fmin,
                           unsigned int  _ELINM_fmax )

/*~-*/
{
   /*~T*/
              char _ELINM_i;
    ELINMINT_ID    _ELINM_tid;
    unsigned  int  _ELINM_chk;


    _ELINM_tid.ID                       = _ELINM_idr;
    _ELINMIntStatus.ELINMIntStatusByte &= 0x0F;                           /* reset error codes (High Nibble of the byte) */
                                                                          /* the low nibble contains the state (IDLE, TX,....) */
    _ELINMIntTHeaderMin                 = ELINMINT_THEADER_MIN;           /* initialize min/max time for Header Variables */
    _ELINMIntTHeaderMax                 = ELINMINT_THEADER_MAX;
    _ELINMIntTFrameMin                  = _ELINM_fmin;                    /* initialize min/max time for Frame Variables: the same for the entire frame */
    _ELINMIntTFrameMax                  = _ELINM_fmax;

   /*~I*/
   /* if the user wants to use extended message the it must set the (ELINMINT_EXTENDED_MESSAGE==1)
   / In this case a special EXTENDED flag is set when the extended frame message is used so
   / that the normal Frame timing check is disabled. This is done because depending on the size
   / of the extended message it will result in transmission times that cannot be handled by
   / the unsigned int _ELINMIntTFrameMax and _ELINMIntTFrameMin variables.
   / in this case the user must redefine these variables and measure the time/code overhead created */

    if( _ELINMIntStatus.ELINMINTSTS.IDLE)                                 /* if idle start */

   /*~-*/
   {
      /*~I*/
#if ELINMINT_EXTENDED_MESSAGE  ==  1                                    /* if extended ID is going to be used */ 

      /*~I*/
        if( _ELINM_idr == ELINMINT_EXTENDED_ID || _ELINM_idr == ELINMINT_EXTENDED_ID1) /* then check if in use and set proper flag */

      /*~-*/
      {
         /*~T*/
            _ELINMIntStatus1.ELINMINTSTS.EXTENDED = 1;

      /*~-*/
      }
      /*~E*/
      /*~-*/
#endif
      /*~E*/
      /*~T*/
        _ELINMIntMessageBuffer[ 0 ]   = ELINMINT_SYNC_VALUE;                /* load SYNC value */
        _ELINMIntMessageBuffer[ 1 ]   = _ELINMIntCalcIDParity( _ELINM_tid );/* calculates ID's Parity */
        _ELINMIntMessageBufferPointer = 0;                                  /* initiate pointer */

      /*~I*/
      /* if reception is activated the number of bytes to be received is passed as a parameter so that the
      / min. and max. frame times can be calculated, however the number of bytes to be transmitted must be 
      / set to 2 so that only the sync and ID are sent.*/

        if( _ELINMIntStatus.ELINMINTSTS.RX )

      /*~-*/
      {
         /*~T*/
            _ELINMIntMessageSize.SIZE = 2;

      /*~-*/
      }
      /*~O*/
      /*~-2*/
      else
      {
         /*~T*/
            _ELINMIntMessageSize.SIZE = _ELINM_size + 2;
            _ELINM_chk                = 0;

         /*~L*/
            for( _ELINM_i = ELIMINT_TXMSG_INIT;
                 _ELINM_i < _ELINMIntMessageSize.SIZE;
                 _ELINM_i++ )

         /*~-*/
         {
            /*~T*/
                _ELINM_chk += _ELINMIntMessageBuffer[ _ELINM_i ];               /* add data bytes for CRC calc. */

         /*~-*/
         }
         /*~E*/
         /*~T*/
            _ELINMIntMessageBuffer[ _ELINMIntMessageSize.SIZE ]
            = ( ~( _ELINM_chk + ( _ELINM_chk>>8 ) ) );                                        /* calc. Checksum */
            _ELINMIntMessageSize.SIZE++;

      /*~-*/
      }
      /*~E*/
      /*~T*/
        SENDB                                = 1;                           /* break character bit */
        TXREG                                = ELINMINT_DUMMY_VALUE;        /* load a dummy character in TX reg. (start TX) */
        _ELINMIntStatus.ELINMIntStatusByte  &= 0x0F;                        /* reset errors */
        _ELINMIntStatus.ELINMINTSTS.TX       = 1;                           /* set TX byte */
        _ELINMIntStatus1.ELINMINTSTS.HEADER  = 1;                           /* enable header timing check */
        _ELINMIntStatus1.ELINMINTSTS.FRAME   = 1;                           /* enable header timing check */
        _ELINMIntReadBack                    = 0x00;                        /* break read back (expected) */
        _ELINMIntStatus.ELINMINTSTS.IDLE     = 0;                           /* clear the IDLE bit */

   /*~-*/
   }
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void _ELINMIntReceiveMessage*/
/*~F*/
/*********************************************************************
 * Function:    void _ELINMIntReceiveMessage(BYTE _ELINM_tag,BYTE _ELINM_id,char _ELINM_size)
 *
 * PreCondition:ELINMIntInitialize invoked
 *              RX Buffer Available (using mELINMIntRXBufferAvailable())
 *
 *
 * Input:       tag - an identifier to the message
 *              id - the ID of the message
 *              size - the size of the message
 *
 * Output:
 *
 * Side Effects:
 *
 * Stack Requirements:
 *
 * Overview:    This function requests a message to be sent by a
 *              slave. It saves the tag so the user can check which message
 *              had a problem is the case of an error being detected
 *              The ID is used to create the Header used to request
 *              the data.
 *
 *
 ********************************************************************/
void _ELINMIntReceiveMessage( BYTE  _ELINM_tag,
                              BYTE  _ELINM_id,
                              char  _ELINM_size )

/*~-*/
{
   /*~I*/
    if( _ELINMIntStatus.ELINMINTSTS.IDLE)                                 /* if idle start */

   /*~-*/
   {
      /*~T*/
        _ELINMIntMessageTag            = _ELINM_tag;                        /* save tag of the message */
        _ELINMIntRXMessageSize.SIZE    = _ELINM_size;                       /* initiate the size of the received message */
        _ELINMIntStatus.ELINMINTSTS.RX = 1;                                 /* set RX flag requesting Reception */
        _ELINMIntRXCRC.CRC             = 0;                                 /* clear CRC (in fact checksum) */
        mELINMIntSendMessage( _ELINMIntMessageBuffer, _ELINM_id, 0 );       /* request the HEADER to be sent */

   /*~-*/
   }
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:BYTE *_ELINMIntGetPointer*/
/*~F*/
/*********************************************************************
 * Function:    BYTE *_ELINMIntGetPointer(char _ELINMInt_tag, BYTE _ELINMInt_position)
 *
 * PreCondition:ELINMIntInitialize invoked
 *              TX Buffer Available (using mELINMIntTXBufferAvailable())
 *
 * Input:
 *              _ELINMInt_tag -> The specific message tag  
 *              to be associated with the TX message buffer.
 *              _ELINMInt_position -> the position of the buffer to be 
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
BYTE * _ELINMIntGetPointer( char  _ELINMInt_tag,
                           BYTE  _ELINMInt_position )

/*~-*/
{
   /*~T*/
    _ELINMIntMessageTag = _ELINMInt_tag;                                  /* save the TAG */
    return ( ( BYTE * )& _ELINMIntMessageBuffer[ _ELINMInt_position ] );  /* returns the data pointer */

/*~-*/
}
/*~E*/
/*~E*/
/*~-*/
#endif
/*~E*/
