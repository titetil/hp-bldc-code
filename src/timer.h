/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/*--------------------------------------------------------------------------------------------------------
  Project Name  : BVH2  
  File Name     : timer.h
  Module        : TIMER 
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
#ifndef _TIMER_H_

/*~T*/
#define     _TIMER_H_     

/************************ global functions **************************/
extern            void timer_init( unsigned char  ui8_TmrNb );
extern  unsigned  int  get_timer( unsigned char  ui8_TmrNb );
extern            void clear_timer( unsigned char  ui8_TmrNb );
extern            void start_timer( unsigned char  ui8_TmrNb, unsigned char  ui8_start_spec_time );
extern            void Oscill_Source_Block( void  );
/****************************Defines*********************************/

#define     def_Task5ms                253 /* 253, Task-Time = 5ms, High-byte and low-byte timer1 normal task time */ 
#define     def_Taskx                  30 /* 120, Task-Time = 5ms, High-byte and low-byte timer1 normal task time */ 
#define     def_Task1                  125 /* Task_Time with Timer1 */ 
#define     Timer1_Freq                253
#define     Periph_Interr_Off          0x00
#define     Periph_Interr_On           0xFF
#define     Periph_Interr_Flag_Off     0x00
#define     Tim_Regis_Low_Clear        0x00
#define     Tim_Regis_High_Clear       0x00
#define     OFF                        0
#define     ON                         1
/*~-*/
#endif
/*~E*/
