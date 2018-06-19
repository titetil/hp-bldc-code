/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/**
* \defgroup  PWM       PWM control
* \brief Configuration of the PWM for BVH2.
* This module contains all functions for PWM read and write .
* \version 1.0 - 2013-02-15
* \section pwm_hist History    
* <b>V1.0 - 2013-02-15 - ADQ - First numbered version</b>    
*/
/**
 * \ingroup   PWM
 * \file      pwm.c
 * \brief     PWM module configuration
 * \note        Target CPU    : PIC16F1936\n
 *              Compiler      : HI_TECH PICC (v9.81)
 * \copyright (c) TI Automotive - All rights reserved
 * \author    TI Automotive - JPL - Joachim Plantholt
 * \author    TI Automotive - ADQ - Aymeric de Quatrebarbes
 * \requirement \b TIR-SW-PWM-001 CDE_MJP Detection
 * \requirement \b TIR-SW-PWM-002 CDE_MJP input frequency
 * \requirement \b TIR-SW-PWM-003 ETAT_MJP PWM out
 * \requirement \b TIR-SW-PWM-004 PWM_CTRL()
 * \requirement \b TIR-SW-PWM-005 Interrupt PWM Capture
 * \requirement \b TIR-SW-PWM-006 Capture Read
 * \test \b TC_TIR-SW-PWM-001 PWM in
 * \test \b TC_TIR-SW-PWM-002 CDE_MJP input frequency
 * \test \b TC_TIR-SW-PWM-003 PWM out
 * \test \b TC_TIR-SW-PWM-004 void PWM_CTRL()
 * \test \b TC_TIR-SW-PWM-005 void interrupt_PWMCapture()
 * \test \b TC_TIR-SW-PWM-006 unsigned char PWMReadDC()
 * \details
 **/
/*~T*/
/*----------------  Includes               --------------------------------------------------------------*/
#include     "pwm.h"
#include     "project.h"
#include     "diag.h"
#include     <htc.h>
/*~T*/
/*--------------------------  globale Variables ------------------------------------------------------------*/
          unsigned  char ui8_PWM_FreqCnt = 0; /**< PWM Frequence Counter to reach the right frequency of ETAT_MJP */
          unsigned  char ui8_PWMoutvalue = 0;   /**< PWM output value for ETAT_MJP */
          unsigned  char ui8_Pulse_State = 0; /**< Selection of the rising or falling edge for the calculation of PWM for CDE_MJP */
          unsigned  int  ui16_Duty_Cycle_In = 0; /**< Absolute value for DC in PWM input */
          unsigned  int  ui16_PWM_Freq_In = 0;/**< Absolute value for the frequency in PWM input */
volatile  _u_wb          ui16_Capt_Val0 = 0;    /**< Reference value for PWM calculation */
volatile  _u_wb          ui16_Capt_Val1 = 0;    /**< Value of CCPR on the falling edge */
volatile  _u_wb          ui16_Capt_Val2 = 0;    /**< Value of CCPR on the second rising edge */
          unsigned  char ui8_Duty_Cycle_In_Ratio = 0; /**< Calculate DC from 0 to 200, corresponding to 0 to 100%, with or without inversion by the transistor  */
          unsigned  char ui8_PWMinDC_sav; /**< Calculation of real DC read by the microcontroller. Must be inverted for all HW from B1 */
          unsigned  char ui8_PWMin_failCnt; /**< Counter for the time of bad detection of ETAT_MJP */ 
/*~T*/
/*-----------------------------  Prototypes --------------------------------------------------------------*/
          void PWM_Capture_init( unsigned char  ui8_CCP_Nb );
          void PWM_CTRL( void  );
          void PWM_Write_Out( unsigned char  ui8_DutyCycle_Out );
          void interrupt_PWMCapture( void  );
unsigned  char PWMReadDC( void  );
/*~A*/
/*~+:void PWM_Capture_init(unsigned char CaptNb)*/
/*~T*/
/**
* \fn      void PWM_Capture_init(unsigned char ui8_CCP_Nb)
* \brief   This function configures Capture Mode for PWM input signal
* \param   ui8_CCP_Nb unsigned char 
* \pre     None
* \post    None
* \details For every PWM module (CCP1 to CCP5):
*          - Switch off the CCPx module (CCPxCON)
*          - Set the CCPx register to 0 (CCPRxL and CCPRxH)
*          - Enable the capture mode interrupt (CCPxIE)
*          - Set the interrupt flag to 0 (CCPxIF)
*          - Set capture mode at every rising edge (CCPxCON)
*/
/*~F*/
void PWM_Capture_init( unsigned char  ui8_CCP_Nb )
/*~-*/
{
   /*~T*/

   ui8_PWMinDC_sav   = 0;
   ui8_PWMin_failCnt = 0;

   /** For every PWM module (CCP1 to CCP5):
   * - Switch off the CCPx module (CCPxCON)
   * - Set the CCPx register to 0 (CCPRxL and CCPRxH)
   * - Enable the capture mode interrupt (CCPxIE)
   * - Set the interrupt flag to 0 (CCPxIF)
   * - Set capture mode at every rising edge (CCPxCON) */
   /*~C*/
   switch( ui8_CCP_Nb )
   /*~-*/
   {
      /*~F*/
      case 1:
      /*~-*/
      {
         /*~T*/
         CCP1CON = Capt_Mode_Off;                  /* CCP1 Module is off */
         CCPR1L  = CCP_Regis_Low_Clear;            /* Set CCP1 register to 0 */
         CCPR1H  = CCP_Regis_High_Clear;
         CCP1IE  = 1;                              /* Capture mode Interrupt enabled */
         CCP1IF  = 0;
         CCP1CON = Capt_Every_Rise_Edge;           /* Capture mode every rising edge */
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 2:
      /*~-*/
      {
         /*~T*/
         CCP2CON = Capt_Mode_Off;              /* CCP2 Module is off */
         CCPR2L  = CCP_Regis_Low_Clear;        /* Set CCP2 register to 0 */
         CCPR2H  = CCP_Regis_High_Clear;
         CCP2IE  = 1;                          /* Capture mode Interrupt enabled */
         CCP2IF  = 0;
         CCP2CON = Capt_Every_Rise_Edge;       /* Capture mode every rising edge */
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 3:
      /*~-*/
      {
         /*~T*/
         CCP3CON = Capt_Mode_Off;               /* CCP3 Module is off */
         CCPR3L  = CCP_Regis_Low_Clear;         /* Set CCP2 register to 0 */
         CCPR3H  = CCP_Regis_High_Clear;
         CCP3IE  = 1;                           /* Capture mode Interrupt enabled */
         CCP3IF  = 0;
         CCP3CON = Capt_Every_Rise_Edge;        /* Capture mode every rising edge */
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 4:
      /*~-*/
      {
         /*~T*/
         CCP4CON = Capt_Mode_Off;             /* CCP4 Module is off */
         CCPR4L  = CCP_Regis_Low_Clear;       /* Set CCP4 register to 0 */
         CCPR4H  = CCP_Regis_High_Clear;
         CCP4IE  = 1;                         /* Capture mode Interrupt enabled */
         CCP4IF  = 0;
         CCP4CON = Capt_Every_Rise_Edge;      /* Capture mode every rising edge */
         break;
      /*~-*/
      }
      /*~E*/
      /*~F*/
      case 5:
      /*~-*/
      {
         /*~T*/
         CCP5CON = Capt_Mode_Off;            /* CCP5 Module is off */
         CCPR5L  = CCP_Regis_Low_Clear;      /* Set CCP5 register to 0 */
         CCPR5H  = CCP_Regis_High_Clear;
         CCP5IE  = 1;                        /* Capture mode Interrupt enabled */
         CCP5IF  = 0;
         CCP5CON = Capt_Every_Rise_Edge;     /* Capture mode every rising edge */
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
/*~+:void PWM_CTRL(void)*/
/*~T*/
/**
* \fn      void PWM_CTRL(void)
* \brief   This function controls the output PWM, it is called every 1ms. 
* \details This function is managing the duty cycle and period of the PWM output signal. The output signal has a frequency of 10Hz.\n
*          Due to the transistor in the HW, the direct output from the uC is inverted compared with the output of the module.
* \pre     None
* \post    None
*/
/*~F*/
void PWM_CTRL( void  )
/*~-*/
{
   /*~I*/
#ifdef def_LIN_Sync
   /*~T*/

   /*~O*/
   /*~-*/
#else
   /*~I*/
#ifdef ver_ETAT
   /*~A*/
   /*~+:Verification of ETAT_MJP*/
   /*~T*/
   ui8_PWM_FreqCnt++;
   /*~I*/
   if( ui8_PWM_FreqCnt >= PWM_Freq10 )
   /*~-*/
   {
      /*~T*/
      ui8_PWM_FreqCnt = 0;
      Port_PWMout     = 0; /* PWM output at pin is 1, Notice that there is a inverting due to the transistor */

   /*~-*/
   }
   /*~O*/
   /*~-2*/
   else
   {
      /*~I*/
      if( ( PIC_ETAT_MONITOR == 1 ) && ( ui8_PWM_FreqCnt >= ui8_PWMoutvalue + 2 ) )
      /*~-*/
      {
         /*~T*/
         Port_PWMout = 0;/* PWM output at pin is 1, Notice that there is a inverting due to the transistor */

      /*~-*/
      }
      /*~O*/
      /*~-2*/
      else
      {
         /*~I*/
         if( ui8_PWM_FreqCnt >= ui8_PWMoutvalue )
         /*~-*/
         {
            /*~T*/
            Port_PWMout = 1;
         /*~-*/
         }
         /*~O*/
         /*~-2*/
         else
         {
            /*~T*/
            //Port_PWMout=1; /* PWM out pin  is Low  */

            /*~I*/
            if( Port_PWMout == 0 )
            /*~-*/
            {
               /*~I*/
               if( PIC_ETAT_MONITOR == 0 )
               /*~-*/
               {
                  /*~T*/
                  PWM_Write_Out( 100 );
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
   /*~O*/
   /*~-*/
#else
   /*~A*/
   /*~+:No verification*/
   /*~T*/
   ui8_PWM_FreqCnt++;
   /*~I*/
   if( ui8_PWM_FreqCnt >= PWM_Freq10 )
   /*~-*/
   {
      /*~T*/
      ui8_PWM_FreqCnt = 0;
      Port_PWMout     = 0; /* PWM output at pin is 1, Notice that there is a inverting due to the transistor */

   /*~-*/
   }
   /*~O*/
   /*~-2*/
   else
   {
      /*~I*/
      if( ui8_PWM_FreqCnt >= ui8_PWMoutvalue )
      /*~-*/
      {
         /*~T*/
         Port_PWMout = 1; /* PWM out pin  is Low  */

      /*~-*/
      }
      /*~E*/
   /*~-*/
   }
   /*~E*/
   /*~E*/
   /*~-*/
#endif
   /*~E*/
   /*~-*/
#endif
   /*~E*/
   /*~T*/

/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void PWM_Write_Out(unsigned char ui8_DutyCycle)*/
/*~T*/
/**
* \fn      void PWM_Write_Out(unsigned char ui8_DutyCycle_Out)
* \brief   This function set the duty cycle value of the PWM output signal
* \param   ui8_DutyCycle_Out unsigned char
* \pre     None
* \post    None
**/
/*~F*/
void PWM_Write_Out( unsigned char  ui8_DutyCycle_Out )
/*~-*/
{
   /*~T*/
   ui8_PWMoutvalue = ui8_DutyCycle_Out;
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:unsigned char PWMReadDC(void)*/
/*~T*/
/**
* \fn      unsigned char PWMReadDC(void)
* \brief   This Routine calculates the duty cycle ratio of the PWM input signal.
* \return  ui8_Duty_Cycle_In_Ratio unsigned char
* \pre     None
* \post    None
* \details The function reads the input DC (from CDE_MJP) inverted or not (set with <tt>def_PWMinInvert</tt>)
**/
/*~F*/
unsigned  char PWMReadDC( void  )
/*~-*/
{
   /*~T*/
   /* First the PWM duty cycle is calculated */
   ui8_PWMinDC_sav  =   (unsigned char) ( 200*(unsigned  short long )(ui16_Duty_Cycle_In) / ui16_PWM_Freq_In ) ;  /* Calculation of duty cycle  */
   /*~I*/
   if( ui8_PWMinDC_sav == 0 )
   /*~-*/
   {
      /*~T*/
      /* if calculation is zero */
      /*~T*/
      ui8_PWMin_failCnt++;
      /*~I*/
      if( ui8_PWMin_failCnt >= 5 )
      /*~-*/
      {
         /*~T*/
         ui8_Duty_Cycle_In_Ratio = ui8_PWMinDC_sav;
         //def_DebugPort0 = 1 ;
         //def_DebugPort0 = 0 ;

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
      /* duty cycle is not zero */
      /*~T*/
      ui8_PWMin_failCnt       = 0;

      /*~T*/
      ui8_Duty_Cycle_In_Ratio = ui8_PWMinDC_sav;
      /*~T*/
      ui16_Duty_Cycle_In      = 0;
   /*~-*/
   }
   /*~E*/
   /*~I*/
#ifdef def_PWMinInvert
   /*~T*/
   return ( 200 - ui8_Duty_Cycle_In_Ratio );
   /*~O*/
   /*~-*/
#else
   /*~T*/
   return ui8_Duty_Cycle_In_Ratio;
   /*~-*/
#endif
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void interrupt_PWMCapture(void)*/
/*~T*/
/**
* \fn      void  interrupt_PWMCapture(void)
* \brief   This function manages the acquisition of PWM input signal.
* The PWM input uses CCP5 of PIC16F1936.  Notice that Timer 1 is assigned to the Motor Module and is as well time base for CCP5. 
* The CCP5 can only detect one edge of input signal. Therefore this function must use an interrupt to detect different edges of an input signal.
* If one edge is detected the software  reprogram the CCP for the next edge. 
* This function is very timing critical, because during interrupt execution additional interrupts are inhibited. 
* The routine writes 3 data values into a structure, which are used for the PWM calculation. 
* This Routine uses CCP5 and detects different edges of PWM input signal
*
* \pre     None
* \post    None
*/
/*~F*/
void interrupt_PWMCapture( void  )
/*~-*/
{
   /*~T*/
   /** First, the first rising edge of the input signal is detected. The value of the CCPR is stored in memory and the register is reprogramed to detect the next * falling edge.
   * 
   * Then, the negative egde is detected. The value of the CCPR is stored in the memory and the register is reprogramed to detect the next rising edge.
   *
   * Last, the second rising edge is detected. The value of the CCPR is stored in the memory and the register is reprogramed to detect the second falling edge.
   * After that, the period of the PWM input is calculated with by substracting the values stored in the memory for the two last rising edges and the duty cycled * is also calculated.
   */
   /*~C*/
    switch( ui8_Pulse_State )
   /*~-*/
   {
      /*~F*/
        case 0:
      /*~-*/
      {
         /*~A*/
         /*~+:Positive edge 1*/
         /*~T*/
                ui8_Pulse_State     = 1;                    /*  Pos_edge 1                                 */
                ui16_Capt_Val0.b.lo = CCPR5L;
                ui16_Capt_Val0.b.hi = CCPR5H;
                CCP5CON             = Capt_Every_Fall_Edge; /* Change to capture mode every falling edge    */
                break;
         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~F*/
        case 1:
      /*~-*/
      {
         /*~A*/
         /*~+:Negative edge*/
         /*~T*/
                ui8_Pulse_State     = 2;                           /*  Neg_edge                                 */
                ui16_Capt_Val1.b.lo = CCPR5L;
                ui16_Capt_Val1.b.hi = CCPR5H;
                CCP5CON             = Capt_Every_Rise_Edge;    /* Change to capture mode every rising edge */
                break;
         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~F*/
        case 2:
      /*~-*/
      {
         /*~A*/
         /*~+:Positive edge 2*/
         /*~T*/
                ui8_Pulse_State     = 1;                             /*  Pos_edge 2                            */
                ui16_Capt_Val2.b.lo = CCPR5L;
                ui16_Capt_Val2.b.hi = CCPR5H;
                ui16_PWM_Freq_In    = ui16_Capt_Val2.w - ui16_Capt_Val0.w;
                ui16_Duty_Cycle_In  = ui16_Capt_Val1.w - ui16_Capt_Val0.w;
                ui16_Capt_Val0.w    = ui16_Capt_Val2.w;
                CCP5CON             = Capt_Every_Fall_Edge;              /*  Capture mode every falling edge    */

         /*~I*/
                if( ui16_Duty_Cycle_In )
         /*~-*/
         {
            /*~T*/
            //def_DebugPort0 = 1 ;
         /*~-*/
         }
         /*~O*/
         /*~-2*/
         else
         {
            /*~T*/
            //def_DebugPort0 = 0 ;
         /*~-*/
         }
         /*~E*/
         /*~T*/
                break;
         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~O*/
      /*~-2*/
      default:
      {
         /*~A*/
         /*~+:others*/
         /*~T*/
         ui8_Pulse_State = 0;                                     /* if variable will be overwritten wrong the default state is taken */
         break;
         /*~E*/
      /*~-*/
      }
   /*~-*/
   }
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
