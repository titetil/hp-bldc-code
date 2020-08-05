/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/*! Test */
/*-------------------------------------------------------------------------------------------------------
  Project Name  : BVH2  
  File Name     : bldc.h
  Module        : BLDC 
  Description   : Header file BLDC   
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


#include     "project.h"

/*~I*/
#ifndef BLDC_H

/*~T*/
#define     BLDC_H                     


/*~T*/
#define     def_sample_NTC_Temp        1
#define     def_sample_Ubat            2
#define     def_sample_Ubemf           3
#define     def_sample_UPhase          4
#define     def_sample_I               5
#define     def_sample_CPU_Temp        9

/*~T*/


/*****************************************************************************/
/* controller specific definitions */
/* application specific definitions */

#define     FPWM                       20000UL // in fact the PWM frequency is 16Khz
//#define     FPWM                       16000UL
//#define FPWM                    10000UL
//#define BLANKINGTIME            0.0001                          /* 200µs blanking */
//#define BLANKINGTIME            0.00025                          /* 200µs blanking */
//#define BLANKINGTIME            0.00035                          /* 200µs blanking */
//#define BLANKINGTIME            0.0004                          /* 200µs blanking */
//#define BLANKINGTIME            0.0005                          /* 200µs blanking */
//#define BLANKINGTIME            0.0006                          /* 200µs blanking */
//#define BLANKINGTIME            0.0007                          /* 200µs blanking */

/* consider that the blankingtime must be seeen as a blanking timeout because the blanking period is detected automatically 
   by sampling the BEMF after the MOSFET are switched to the next state. this blanking time is only the timeout value
 
    */
//#define BLANKINGTIME            0.0010                           /* 1ms blanking */
//#define BLANKINGTIME            0.0004                             /* 600æs blanking */
//#define BLANKINGTIME            0.0006                             /* 600æs blanking */
//#define BLANKINGTIME            0.0007                             /* 600æs blanking */
#define     BLANKINGTIME               0.002                      /* 2ms blanking timeout because automatic blanking is implemented */ 




#define     FOSC_var                       32000000UL
#define     POLEPAIRS                  2

//#define def_phase_angle       110  // optimum for high speed 
#define     def_phase_angle            128 // optimum for high speed 
//#define def_phase_angle       120  // optimum for high speed 
#define     def_PhaseAdvanceStart      128

#define     def_MINSPEED               300
//#define def_MaxSPEED                9000   /* not used at the moment */
//#define def_SpeedMax               13333    /* 1/tcom     tcom = x * 12 steps * 50æs / 8 stages  **/
//#define def_SpeedMax               1666       /* 1/tcom     tcom = x * 12 steps * 50æs             **/

// All the caculated speed depends on the  

#define     def_ramp0_start_SPEED       600   // actually unused 
#define     def_ramp1_start_SPEED       200   // consider 5/4 ( 16 to 20 Khz) 1. alignment step   
//#define     def_ramp1_start_SPEED       300   // consider 5/4 ( 16 to 20 Khz) 1. alignment step   
#define     def_ramp2_start_SPEED       800   // 2. alignement step 
#define     def_ramp3_start_SPEED       1200  // 1. first speed step 
#define     def_ramp4_start_SPEED       1600  // 2. second speed step  
#define     def_ramp5_start_SPEED       2000
#define     def_ramp6_start_SPEED       2000
#define     def_ramp7_start_SPEED       2000
#define     def_ramp8_start_SPEED       2000
#define     def_ramp9_start_SPEED       2000
#define     def_ramp_end_SPEED          2000
#define     def_STARTUP_PWM             40   /*  PWM based on PWM period  */
#define     def_Align_PWM               40   /*  PWM based on PWM period  */

//#define     def_STARTUP_PWM             30   /*  PWM based on PWM period  */
#define     def_minSTARTUP_PWM          10     /*  PWM based on PWM period  */
#define     def_nb_steps_startramp      10
#define     def_nb_align_startramp      3    /* -1 number of alignement steps */

#define     def_DEFAULT_UBAT            400 /* 12V */ 



/************************************************************************************************/
/*                             1                                                                */
/* TSECTOR =      --------------------------------------                                        */
/*                   (MechRPM /60) * POLEPAIRS * 6 Step                                         */
/*                                                                                              */
/*                                         Tsector                                              */
/* COMM_TIME_MAX as number of interrupt  = --------                                             */
/*                                         TPWM                                                 */
/************************************************************************************************/

#define     def_COMM_TIME_MAX          ( unsigned short ) ( ( 10 * FPWM ) / ( def_MINSPEED * POLEPAIRS ) )
#define     def_ramp0_start            ( unsigned short ) ( ( 10 * FPWM ) / ( def_ramp0_start_SPEED * POLEPAIRS ) )
#define     def_ramp1_start            ( unsigned short ) ( ( 10 * FPWM ) / ( def_ramp1_start_SPEED * POLEPAIRS ) )
#define     def_ramp2_start            ( unsigned short ) ( ( 10 * FPWM ) / ( def_ramp2_start_SPEED * POLEPAIRS ) )
#define     def_ramp3_start            ( unsigned short ) ( ( 10 * FPWM ) / ( def_ramp3_start_SPEED * POLEPAIRS ) )
#define     def_ramp4_start            ( unsigned short ) ( ( 10 * FPWM ) / ( def_ramp4_start_SPEED * POLEPAIRS ) )
#define     def_ramp5_start            ( unsigned short ) ( ( 10 * FPWM ) / ( def_ramp5_start_SPEED * POLEPAIRS ) )
#define     def_ramp6_start            ( unsigned short ) ( ( 10 * FPWM ) / ( def_ramp6_start_SPEED * POLEPAIRS ) )
#define     def_ramp7_start            ( unsigned short ) ( ( 10 * FPWM ) / ( def_ramp7_start_SPEED * POLEPAIRS ) )
#define     def_ramp8_start            ( unsigned short ) ( ( 10 * FPWM ) / ( def_ramp8_start_SPEED * POLEPAIRS ) )
#define     def_ramp9_start            ( unsigned short ) ( ( 10 * FPWM ) / ( def_ramp9_start_SPEED * POLEPAIRS ) )
#define     def_ramp_end               ( unsigned short ) ( ( 10 * FPWM ) / ( def_ramp_end_SPEED * POLEPAIRS ) )


/* ignore comparator output for a number or PWM cycles to prevent false commutation             */
/* detection because of winding saturation after sector change                                  */
#define     def_BLANKINGCOUNT        ( unsigned char ) ( BLANKINGTIME * FPWM )

/* calculated value for PWM generator based on oscillator and required PWM frequency            */
//#define PWM_PERIOD_VALUE        (unsigned char)(FOSC / (4 * FPWM))

#define     DEF_PWM_PERIOD_VALUE     ( unsigned char ) ( FOSC_var / ( 16 * FPWM ) )
//#define PWM_PERIOD_VALUE     80?????
//#define DEF_PWM_PERIOD_VALUE        (unsigned char)(FOSC / (8 * FPWM))

/* so adding a value of 2 adjusts the result to correct value                                   */
#define     T1ADJUSTTCLOCK           2


/* Driver and sense states
* Phase               A        B        C
* Low Side Drive   P1A/RC5  P1B/RC4  P1D/RC2  
* High Side Drive    RA2      RA4      RA5
* BEMF Sense       C12IN0-  C12IN1-  C12IN2-
*/
#define     DRIVE_A_HIGH             0x04
#define     DRIVE_B_HIGH             0x10
#define     DRIVE_C_HIGH             0x20

#define     SENSE_A                  0x00
#define     SENSE_B                  0x01
#define     SENSE_C                  0x02

#define     PhaseA_Ena               LATC1
#define     PhaseB_Ena               LATC3
#define     PhaseC_Ena               LATC4

#define     PhaseX_DIS               LATC = LATC & 0b11100101
#define     PhaseX_Ena               LATC = LATC | 0b00011010
#define     State1_Ena               LATC = ( LATC & 0b11100101 ) | 0b00001010 /* Phase A and Phase B enabled */ 
#define     State2_Ena               LATC = ( LATC & 0b11100101 ) | 0b00010010 /* Phase A and Phase C enabled */ 
#define     State3_Ena               LATC = ( LATC & 0b11100101 ) | 0b00011000 /* Phase B and Phase C enabled */ 
#define     State4_Ena               LATC = ( LATC & 0b11100101 ) | 0b00001010 /* Phase A and Phase B anabled */ 
#define     State5_Ena               LATC = ( LATC & 0b11100101 ) | 0b00010010 /* Phase A and Phase C enabled */ 
#define     State6_Ena               LATC = ( LATC & 0b11100101 ) | 0b00011000 /* Phase B and Phase C enabled */ 

#define     State_dummy              LATC = LATC

#define     PhaseA_Dir               LATC2
#define     PhaseB_Dir               LATB2
#define     PhaseC_Dir               LATB1

#define     OverCurrentFlag          MotorFlags.bits.B0
#define     MOTOR_NOT_SYNC           MotorFlags.bits.B1
#define     DIRECTION                MotorFlags.bits.B2
#define     Flag_STARTUP             MotorFlags.bits.B3
#define     Flag_MotorAlign          MotorFlags.bits.B4
#define     ZC_DETECTED              MotorFlags.bits.B5
#define     Flag_RUN_MOTOR           MotorFlags.bits.B6
#define     COMM_DONE                MotorFlags.bits.B7


//#define def_Current_Limit 320     /* 18 A */
//#define     def_Current_Limit     260 /* 15 A */ 
//#define     def_Current_Limit     300 /* 15 A for hardware C0 */ 
#define     def_Current_Limit     500 /* 25 A for hardware C0 */ 


/*~T*/

/************************************************************************
* prototypes                                                            *
*************************************************************************/

extern  void init_bldc( void  );
extern  void interrrupt_bldc( void  );
extern  void InitMotorStop( void  );
extern  void InitMotorRun( void  );
extern            signed    short       phase_delay_counter_debug;
extern            unsigned         char BlankingCountdbg;

extern            unsigned  short       ui16_phase_angle;
extern            unsigned         int  B[ 8 ];
extern            _u_bits               MotorFlags; /* enables the access to the flags of the Motor */
extern            unsigned  short       ui16_speed_fil;
extern            unsigned  short       ui16_speed_rar;
extern  volatile  _u_wb                 ui16_IPhase1_bldc;
extern  volatile  _u_wb                 ui16_IPhase2_bldc;
extern  volatile  _u_wb                 ui16_IPhase3_bldc;
extern  volatile  _u_wb                 ui16_IPhase_bldc;

extern            unsigned  short       ui16_UPhase_bldc;

extern            unsigned  short       ui16_NTC_Temp_bldc;
extern            unsigned  short       ui16_CPU_Temp_bldc;

extern  volatile  _u_wb                 ui16_Ubat_bldc;
extern  volatile  _u_wb                 ui16_Ubemf_bldc;
extern            unsigned  short       ui8_zero_cros_cnt;
extern            unsigned  short       ui16_duty_cycle_BLDC;


/************************************************************************
* prototypes                                                            *
*************************************************************************/

void InitSystem( void  );
void InitDriver( void  );
void Commutate( void  );
void ControlStartUp( void  );
void StallControl( void  );
void AdcManager( void  );
void WarmUpControl( void  );
void TimeBaseManager( void  );
void CalcFilter( void  );
void ControlSlowStart( void  );





/*~-*/
#endif
/*~E*/
