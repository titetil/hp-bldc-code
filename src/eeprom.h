/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/**
* \ingroup   EEPROM
* \file      eeprom.h
* \brief     Header for EEPROM management
* \note      Target CPU    : PIC16F1936\n
*            Compiler      : HI_TECH PICC (v9.81)           
* \copyright (c) TI Automotive - All rights reserved
* \author    TI Automotive - JPL - Joachim Plantholt        
* \author    TI Automotive - ADQ - Aymeric de Quatrebarbes
* \author    TI Automotive - MMK - Mohamed Moussaddak  
* 
**/
/*~I*/
#ifndef _EEPROM_H_
/*~T*/
#define     _EEPROM_H_     
/************************ global functions **************************************************************/
extern  unsigned  char read_eeprom_data( unsigned char  ui8_adress );
extern            void write_eeprom_data( unsigned char  ui8_adress, unsigned char  ui8_adress_data );
/****************************Defines*******************************************************************/
/*~-*/
#endif
/*~E*/
