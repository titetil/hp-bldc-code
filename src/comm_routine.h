/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/*--------------------------------------------------------------------------------------------------------
  Project Name  : BVH2  
  File Name     : comm_routine.h
  Module        : LIN_calibration 
  Description   : Header file LIN calibration   
  Target CPU    : PIC16F1936
  Compiler      : tbd 

  Copyright (c)  TI Automotive     All rights reserved.

 Initials     Name                      Company
 --------     ---------------------     ------------------------------------

 AQS           Aymeric de Quatrebarbes  TI Automotive
   

 
 H I S T O R Y

 Date       Version  Author  Description
 yy-mm-dd   
 12-05-09   0.00.01  AQS     created
  
-------------------------------------------------------------------------------------------------------- */

/*~I*/
#ifndef _COMM_ROUTINE_H_
/*~T*/
#define _COMM_ROUTINE_H_

/************************ global functions **************************/

extern char ICT_stamp_verification(void); // Verification of ICT stamp presence
extern char FCT_stamp_verification(void); // Verification of FCT stamp presence
extern unsigned int read_SW_version(void); // SW version
extern void write_FCT_stamp(unsigned char Prod_Year_FCT,
                     unsigned char Prod_Day_FCT_MSB,
                     unsigned char Prod_Day_FCT_LSB,
                     unsigned char Prod_Num_FCT_MSB,
                     unsigned char Prod_Num_FCT_LSB);
extern void LIN_deactivation(void);
extern void EOL_TEST_FCT(void);

/************************extern variable **************************/



/****************************Defines*********************************/


/*~-*/
#endif
/*~E*/
