/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/**
* \defgroup  EEPROM    EEPROM Management
* \brief Functions for the managing of the EEPROM.
* This module contains functions to read and write into the EEPROM .
* \version 1.0 - 2013-02-15
* \section eeprom_hist History   
* <b>V1.0 - 2013-02-15 - ADQ - First numbered version</b>     
*/
/**
* \ingroup   EEPROM
* \file      eeprom.c
* \brief     Manages the reading and the writting of the EEPROM
* \note        Target CPU    : PIC16F1936\n
*              Compiler      : HI_TECH PICC (v9.81)            
* \copyright (c) TI Automotive - All rights reserved 
* \author    TI Automotive - JPL - Joachim Plantholt        
* \author    TI Automotive - ADQ - Aymeric de Quatrebarbes  
* \details   The functions are the translation of Assembler Code in the microcontroller datasheet.
* \requirement TIR-SW-EEPROM-001 Read in eeprom
* \requirement TIR-SW-EEPROM-002 Write data in eeprom
* \test
**/

/*~T*/
/*----------------  Includes               --------------------------------------------------------------*/
#include     <htc.h>
#include     "eeprom.h"
#include     "pwm.h"
#include     "project.h"
#include     "timer.h"
/*~T*/
/*-----------------------------  Prototypes --------------------------------------------------------------*/
unsigned  char read_eeprom_data( unsigned char  ui8_adress );
          void write_eeprom_data( unsigned char  ui8_adress, unsigned char  ui8_adress_data );

/*~A*/
/*~+:unsigned char read_eeprom_data(unsigned char ui8_adress)*/
/*~T*/
/**
* \fn      unsigned char  read_eeprom_data(unsigned char ui8_adress)
* \brief   This function reads the data ui8_adress_data at the address ui8_adress in the EEPROM.
* \param   ui8_adress  unsigned char
* \return  ui8_adress_data unsigned char
* \pre     None
* \post    None
* \details The function is translate from the Assembler Code in the PIC16F1936 datasheet.
**/
/*~F*/
unsigned  char read_eeprom_data( unsigned char  ui8_adress )
/*~-*/
{
   /*~T*/
    unsigned  char ui8_adress_data;

    WREN = 0;
    WR = 0;
    EEADRL          = ui8_adress; /* Data Memory adress to read */
    EEPGD           = 0;
    CFGS            = 0;      /* Point to DATA Memory */
    RD              = 1;      /* EEPROM Read enabled */
    ui8_adress_data = EEDATL;

    return ui8_adress_data;
   /* Returns the data read in EEPROM at the address given to the function */
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void write_eeprom_data(unsigned char ui8_adress, unsigned char ui8_adress_data)*/
/*~T*/
/**
* \fn      void write_eeprom_data(unsigned char ui8_adress, unsigned char ui8_adress_data)
* \brief   This function writes into the EEPROM at the given address ui8_adress the given data ui8_adress_data.
* \param   ui8_adress  unsigned char
* \param   ui8_adress_data  unsigned char
* \pre     None
* \post    None
* \details The function is translate from the Assembler Code in the PIC16F1936 datasheet.
**/
/*~F*/
void write_eeprom_data( unsigned char  ui8_adress,
                        unsigned char  ui8_adress_data )
/*~-*/
{
   /*~T*/

    WREN = 0;
    RD = 0;
    EEADRL = ui8_adress;          /* Data Memory address to write */
    EEDATL = ui8_adress_data;     /* Data Memory Value to write */
    CFGS   = 0;                   /* Deselect Configuration Space */
    EEPGD  = 0;                   /* Point to Data Memory */
    WREN   = 1;                   /* Enable Writes */

    GIE    = 0;                   /* Disable INTs */
    EECON2 = 0x55;                /* Write 55h */
    EECON2 = 0xAA;                /* Write AAh */
    WR     = 1;                   /* Set WR bit to begin write */
 

   /*~L*/
    while( WR )
   /*~-*/
   {
      /*~T*/
        asm( "nop" );
        asm( "nop" );
   /*~-*/
   }
   /*~E*/
   /*~T*/
      WREN   = 0;                   /* Disable Writes */
      GIE    = 1;                   /* Enable interrupts */

/*~-*/
}
/*~E*/
/*~E*/
