/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/*--------------------------------------------------------------------------------------------------------
  Project Name  : BVH2  
  File Name     : pwm.h
  Module        : PWM 
  Description   : Header file TIMER   
  Target CPU    : PIC16
  Compiler      : tbd 

  Copyright (c)  TI Automotive     All rights reserved.

 Initials     Name                      Company
 --------     ---------------------     ------------------------------------

 MMK           Mohamed Moussaddak       TI Automotive
 JPL           Joachim Plantholt        TI Automotive   

 
 H I S T O R Y

 Date       Version  Author  Description
 yy-mm-dd   
 10-11-17   0.00.01  jpl     created
 11-10-28   0.01.00  jpl     fixed first version 
  
-------------------------------------------------------------------------------------------------------- */

/*~I*/
#ifndef _PWM1_H_

/*~T*/
#define     _PWM1_H_     

/************************ global functions **************************************************************/
extern            void PWM_Capture_init( unsigned char  CCP_Nb );
extern            void PWM_CTRL( void  );
extern            void PWM_Write_Out( unsigned char  ui8_DutyCycle_Out );
extern            void interrupt_PWMCapture( void  );
extern  unsigned  char PWMReadDC( void  );
extern  unsigned  int    ui16_PWM_Freq_In;
extern            double Duty_Cycle_In_Ratio;
extern  unsigned  char   ui8_Duty_Cycle_In_Ratio;
extern  unsigned  char   ui8_PWMoutvalue;
extern  unsigned  char   ui8_Duty_Cycle_In_Ratio;

/****************************Defines*******************************************************************/
#define     PWM_Freq10               100         /*  PWM frequenz is 10 Hz (100ms)                   */ 
#define     Capt_Every_Fall_Edge     0x04
#define     Capt_Every_Rise_Edge     0x05
#define     Capt_Mode_Off            0x00
#define     CCP_Regis_Low_Clear      0x00
#define     CCP_Regis_High_Clear     0x00


#define     Port_PWMout              LATC0 /* use lat instead of using pin because this could have side effects */ 
/*~-*/
#endif
/*~E*/
