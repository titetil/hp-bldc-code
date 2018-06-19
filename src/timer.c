/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/**
* \defgroup  TIMER     Timer control
* \brief Configuration of the timers of the PIC16F1936.
* This module contains all functions for .
* \version 1.1 - 2013-04-05
* \section timer_hist History
* <b>V1.1 - 2013-04-05 - ADQ - First version for C sample</b>
* -     Change the current calibration in EOL
* -     Measure the ramp and write the ramp coefficient into EEPROM in LID 0x95      
* <b>V1.0 - 2013-02-15 - ADQ - First numbered version</b>
* -     This version is related to HW B3     
*/
/**
 * \ingroup   TIMER
 * \file      timer.c
 * \brief     Timer configuration for BVH2 software
 * \note        Target CPU    : PIC16F1936\n
 *              Compiler      : HI_TECH PICC (v9.81)
 * \copyright (c) TI Automotive - All rights reserved
 * \author    TI Automotive - JPL - Joachim Plantholt
 * \author    TI Automotive - ADQ - Aymeric de Quatrebarbes
 * \requirement \b TIR-SW-Timer-001 Timer init
 * \requirement \b TIR-SW-Timer-002 Get Timer
 * \requirement \b TIR-SW-Timer-003 Free running timer for scheduler
 * \requirement \b TIR-SW-Timer-004 BLDC timer
 * \requirement \b TIR-SW-Timer-005 Timer PWM Computation
 * \requirement \b TIR-SW-Timer-007 Clear Timers
 * \test \b TC_TIR-SW-Timer-001 void timer_init(unsigned char)
 * \test \b TC_TIR-SW-Timer-002 unsigned int get_timer(unsigned char)
 * \test \b TC_TIR-SW-Timer-003 Free running timer for scheduler
 * \test \b TC_TIR-SW-Timer-004 BLDC Timer
 * \test \b TC_TIR-SW-Timer-005 Timer PWM Computation
 * \test \b TC_TIR-SW-Timer-006 void start-timer(unsigned char)
 * \details
 **/
/*~T*/
/*----------------  Includes               --------------------------------------------------------------*/
#include     "project.h"
#include     "timer.h"
#include     <htc.h>
/*~T*/
/*----------------  globale Variables      --------------------------------------------------------------*/
/*~T*/
/*----------------  Prototypes             --------------------------------------------------------------*/
          void timer_init( unsigned char  ui8_TmrNb );
unsigned  int  get_timer( unsigned char  ui8_TmrNb );
          void start_timer( unsigned char  ui8_TmrNb, unsigned char  ui8_start_spec_time );
          void clear_timer( unsigned char  ui8_TmrNb );
          void Oscill_Source_Block( void  );
/*~A*/
/*~+:void timer_init(unsigned char ui8_TmrNb)*/
/*~T*/
/**
 * \fn      void timer_init(unsigned char ui8_TmrNb)
 * \brief   Initialisation of defined timer
 * \param   ui8_TmrNb unsigned char
 * \pre     None
 * \post    None
 * \details This function is initialisating the defined timer between the timers proposed by the microcontroller given by the argument
 *          For each timer comes first the initialisation to 0 then a defined frequency of the timer. This frequency depends on the module assigned to this timer.
 *          - Timer 0 : Internal oscillator. Frequenz 32 MHz. Fcyc=Fosc/4=8MHz. Prescale 1:4 -->  Tsys = 0,5us.
 *          - Timer 1 : Timer1 clock is Fosc, prescale=1:4
 *          - Timer 2 : Motor PWM generation. Postscaler 1:1, Prescaler 1:64
 *          - Timer 3 : Not used
 *          - Timer 4 : Scheduler. Postscaler 1:2, Prescaler 1:16
 *          - Timer 5 : Not used
 *          - Timer 6 : LIN Interface and needs an Interrupt
 */
/*~F*/
void timer_init( unsigned char  ui8_TmrNb )
/*~-*/
{
   /*~C*/
   switch( ui8_TmrNb )
   /*~-*/
   {
      /*~F*/
      case 0:
      /*~-*/
      {
         /*~T*/
         /*------------------------Timer 0 initialisation------------------------------------------------------*/
         clear_timer( 0 );
         TMR0IE = 0;
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 1:
      /*~-*/
      {
         /*~T*/
         /*-------------------------Timer 1 initialisation------------------------------------------------------*/
         clear_timer( 1 );
         T1CON  = 0b01110101;           /* Timer1 clock is Fosc, prescale=1:4, Timer1 starts            */
         TMR1IE = 0;
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 2:
      /*~-*/
      {
         /*~T*/
         /*-------------------------Timer 2 is used for Motor PWM Generation -----------------------------------*/
         clear_timer( 2 );
         T2CON  = 0b00000111;           /* Postscaler 1:1, Prescaler 1:64, Timer2 is on  */
         TMR2IE = 0;
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 4:
      /*~-*/
      {
         /*~T*/
         /*-------------------------Timer 4 is used for Scheduler ----------------------------------------------*/
         clear_timer( 4 );
         T4CON  = 0b00001110;           /* Postscaler 1:2, Prescaler 1:16, Timer4 is on                 */


         /*~T*/
         TMR4IE = 1;
         /*~T*/
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 6:
      /*~-*/
      {
         /*~T*/
         /*-------------------------Timer 6 ist used for LIN Interface and needs an Intterupt ------------------*/
         clear_timer( 6 );
         T6CON = 0b00000101;

         /*~I*/
#ifdef def_LIN
         /*~T*/
         TMR6IE = 1;

         /*~O*/
         /*~-*/
#else
         /*~I*/
#ifdef def_LIN_5ms
         /*~T*/
         TMR6IE = 1;

         /*~O*/
         /*~-*/
#else
         /*~T*/
         TMR6IE = 0;
         /*~-*/
#endif
         /*~E*/
         /*~-*/
#endif
         /*~E*/
         /*~T*/
         break;
      /*~-*/
      }
      /*~E*/
      /*~O*/
      /*~-2*/
      default:
      {
         /*~T*/
         break;

      /*~-*/
      }
   /*~-*/
   }
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:unsigned int get_timer(unsigned char ui8_TmrNb)*/
/*~T*/
/**
 * \fn      unsigned int get_timer(unsigned char ui8_TmrNb)
 * \brief   Get the timer value for defined timer
 * \param   ui8_TmrNb unsigned char
 * \pre     None
 * \post    None
 * \details This function is getting the timer value for the defined timer
 */
/*~F*/
unsigned  int get_timer( unsigned char  ui8_TmrNb )
/*~-*/
{
   /*~T*/
   unsigned  char Tmr_Val;

   /*~C*/
   switch( ui8_TmrNb )
   /*~-*/
   {
      /*~F*/
      case 0:
      /*~-*/
      {
         /*~T*/
         Tmr_Val = TMR0;
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 2:
      /*~-*/
      {
         /*~T*/
         Tmr_Val = TMR2;
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 3:
      /*~-*/
      {
         /*~T*/
         Tmr_Val = 0;
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 4:
      /*~-*/
      {
         /*~T*/
         Tmr_Val = TMR4;
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 5:
      /*~-*/
      {
         /*~T*/
         Tmr_Val = 0;
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 6:
      /*~-*/
      {
         /*~T*/
         Tmr_Val = TMR6;
         break;
      /*~-*/
      }
      /*~E*/
      /*~O*/
      /*~-2*/
      default:
      {
         /*~T*/
         Tmr_Val = 0;
         break;
      /*~-*/
      }
   /*~-*/
   }
   /*~E*/
   /*~T*/
   return Tmr_Val;

/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void clear_timer(unsigned char ui8_TmrNb)*/
/*~T*/
/**
 * \fn      void clear_timer(unsigned char ui8_TmrNb)
 * \brief   Clear and restore to 0 the timer value for defined timer
 * \param   ui8_TmrNb unsigned char
 * \pre     None
 * \post    None
 * \details This function is setting the timer value to 0
 */
/*~F*/
void clear_timer( unsigned char  ui8_TmrNb )
/*~-*/
{
   /*~C*/
   switch( ui8_TmrNb )
   /*~-*/
   {
      /*~F*/
      case 0:
      /*~-*/
      {
         /*~T*/
         TMR0 = 0;
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 1:
      /*~-*/
      {
         /*~T*/
         TMR1L = Tim_Regis_Low_Clear;                   /*    Reset Timer1                             */
         TMR1H = Tim_Regis_High_Clear;
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 2:
      /*~-*/
      {
         /*~T*/
         TMR2 = 0;
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 4:
      /*~-*/
      {
         /*~T*/
         TMR4 = 0;
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 6:
      /*~-*/
      {
         /*~T*/
         TMR6 = 0;
         break;
      /*~-*/
      }
      /*~E*/
      /*~O*/
      /*~-2*/
      default:
      {
         /*~T*/
         break;

      /*~-*/
      }
   /*~-*/
   }
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void start_timer(unsigned char ui8_TmrNb, unsigned char ui16_start_spec_time)*/
/*~T*/
/**
 * \fn      void start_timer(unsigned char ui8_TmrNb, unsigned char ui8_start_spec_time)
 * \brief   Start the defined timer with a defined value
 * \param   ui8_TmrNb unsigned char
 * \param   ui8_start_spec_time unsigned char
 * \pre     None
 * \post    None
 * \details This function in starting the timer number ui8_TmrNb with a specific time, do nothing in case of timer 0 or timer 1
 */
/*~F*/
void start_timer( unsigned char  ui8_TmrNb,
                  unsigned char  ui8_start_spec_time )
/*~-*/
{
   /*~C*/
   switch( ui8_TmrNb )
   /*~-*/
   {
      /*~F*/
      case 0:
      /*~-*/
      {
         /*~T*/
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 1:
      /*~-*/
      {
         /*~T*/
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 2:
      /*~-*/
      {
         /*~T*/
         TMR2ON = OFF;
         PR2    = ui8_start_spec_time;
         TMR2ON = ON;
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 4:
      /*~-*/
      {
         /*~T*/
         TMR4ON = OFF;
         PR4    = ui8_start_spec_time;
         TMR4ON = ON;
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 6:
      /*~-*/
      {
         /*~T*/
         TMR6ON = OFF;
         PR6    = ui8_start_spec_time;
         TMR6ON = ON;
         break;
      /*~-*/
      }
      /*~E*/
      /*~O*/
      /*~-2*/
      default:
      {
         /*~T*/
         break;

      /*~-*/
      }
   /*~-*/
   }
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void Oscill_Source_Block(void)*/
/*~T*/
/**
 * \fn      void Oscill_Source_Block(void)
 * \brief   Set the oscillator source and the frequency of the Watchdog
 * \pre     None
 * \post    None
 * \details This function in setting the oscillator source as timer 1 with Fosc = 32 Mhz. The Watchdog Timer Period is 256ms => 4 Hz
 */
/*~F*/
void Oscill_Source_Block( void  )
/*~-*/
{
   /*~T*/
   OSCCON = 0xF0;        /* Timer 1 oscillator source Fosc=32 MHz, */
   WDTCON = 0b00010001;  /* Watchdog Timer Period interval 256ms typ wtachdog enabled  */ 
/*~-*/
}
/*~E*/
/*~E*/
