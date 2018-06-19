/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/**
* \defgroup  INTERRUPT Interrupt routine
* \brief Configuration of the interrupts.
* This module contains the assignement for the interrupts for the program.
* \version 1.0 - 2013-02-15
* \section interrupt_hist History   
* <b>V1.0 - 2013-02-15 - ADQ - First numbered version</b>    
*/
/**
 * \ingroup   INTERRUPT
 * \file      interrupt.c
 * \brief     Interrupt routine
 * \note        Target CPU    : PIC16F1936\n
 *              Compiler      : HI_TECH PICC (v9.81)
 * \copyright (c) TI Automotive - All rights reserved
 * \author    TI Automotive - JPL - Joachim Plantholt
 * \requirement \b TIR-SW-INT-001 interrupt_handler()
 * \test \b TC_TIR-SW-INT-001 interrupt_handler()
 * \details   These functions assigne the interrupt routines and the scheduler of the main function in the SW.
 **/
/*~T*/
/*----------------  Includes               --------------------------------------------------------------*/
#include     <htc.h>
#include     "project.h"
#include     "timer.h"
#include     "pwm.h"
#include     "bldc.h"
#include     "lin.h"
#include     "diag.h"
/*~T*/
/*----------------  globale Variables      --------------------------------------------------------------*/
unsigned  char ui8_Task_Cont1ms;  /**< Counter value incremented by Task1ms */ 
/*~T*/
/*----------------  Prototypes             --------------------------------------------------------------*/
        void           Task1ms( void  );
static  void interrupt  interrupt_handler( void  );
/*~A*/
/*~+:void Task1ms(void)*/
/*~T*/
/**
 * \fn      void Task1ms(void)
 * \brief   Counter for the 1ms main scheduler
 * \pre     None
 * \post    None
 * \variable #ui8_Task_Cont1ms
 * \details Each time this function is called, a counter represented by #ui8_Task_Cont1ms in growing. This task must be call each 1ms
 **/
/*~F*/
void Task1ms( void  )
/*~-*/
{
   /*~T*/
   ui8_Task_Cont1ms++;
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+: static void interrupt interrupt_handler*/
/*~T*/
/**
 * \fn      static void interrupt interrupt_handler(void)
 * \brief   Interrupt routine for BVH2
 * \pre     None
 * \post    None
 * \details This function manages the flags and the interrupt of timer1, timer2, timer4 and timer6, and also the interrupt capture module.\n
 *          Timer1 interrupt flag is the main interrupt routine,\n
 *          Timer2 interrupt flag manages the bldc interrupt routine,\n
 *          Timer4 interrupt flag manages the main scheduler all ms and also the diagnostic output,\n
 *          Timer6 interrupt flag manages the LIN interrupt routine,\n
 *          CCP5 interrupt flag manages the PWM interrupt routine.
 **/
/*~F*/
static  void interrupt  interrupt_handler( void  )
/*~-*/
{
   /*~I*/
   if( TMR1IF )
   /*~-*/
   {
      /*~T*/
      TMR1IF = 0;
   /*~-*/
   }
   /*~E*/
   /*~I*/
   if( TMR4IF )
   /*~-*/
   {
      /*~T*/
      TMR4IF = 0;
      /*~T*/
      PWM_CTRL( );
      /*~T*/
      Task1ms( );
   /*~-*/
   }
   /*~E*/
   /*~T*/
   //DiagPicEtatMonitor();

   /*~I*/
   if( TMR2IF )
   /*~-*/
   {
      /*~T*/
      TMR2IF = 0;
      /*~T*/
      interrrupt_bldc( );
   /*~-*/
   }
   /*~E*/
   /*~I*/
   if( TMR6IF )
   /*~-*/
   {
      /*~T*/
      TMR6IF = 0;
      /*~I*/
#if defined  (  def_LIN  )  ||  defined  (  def_LIN_5ms  )

      /*~T*/
      ELINMIntHandler( );   /* process LIN int. based protocol */ 
      /*~-*/
#endif
      /*~E*/
   /*~-*/
   }
   /*~E*/
   /*~I*/
   if( CCP5IF )
   /*~-*/
   {
      /*~T*/
      CCP5IF = 0;
      /*~T*/
      interrupt_PWMCapture( );
   /*~-*/
   }
   /*~E*/
   /*~I*/
   if( CCP2IF )
   /*~-*/
   {
      /*~T*/
      CCP2IF = 0;
   /*~-*/
   }
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
