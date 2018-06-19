/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/**
* \defgroup  DIAG      Diagnostics of failures
* \brief Diagnostics for BVH.
* This module is initialize the module for diagnostics, convert the alarm into PWM output signal and do the EOL.
* \version 1.1 - 2013-04-05
* \section diag_hist History
* <b>V1.1 - 2013-04-05 - ADQ - First version for C sample</b>
* -     Change the current calibration in EOL
* -     Measure the ramp and write the ramp coefficient into EEPROM in LID 0x95      
* <b>V1.0 - 2013-02-15 - ADQ - First numbered version</b>
* -     This version is related to HW B3     
*/

/**
* \ingroup   DIAG
* \file      diag.c
* \brief     Diag module for BVH2 software
* \note      Target CPU    : PIC16F1936\n
*            Compiler      : HI_TECH PICC (v9.81)       
* \copyright (c) TI Automotive - All rights reserved 
* \author    TI Automotive - MMK - Mohamed Moussaddak       
* \author    TI Automotive - JPL - Joachim Plantholt        
* \author    TI Automotive - ADQ - Aymeric de Quatrebarbes  
* \details   This file regroups all flags for the diagnostics and set the ETAT_MJP of the system to the corresponding PWM value.\n
*            The root causes and the limp behavior are described in the Matlab layer
* \requirement \b TIR-SW-DIAG-002 Over Temperature
* \requirement \b TIR-SW-DIAG-003 Pump current consumption to high
* \requirement \b TIR-SW-DIAG-004 Pump current consumption to low
* \requirement \b TIR-SW-DIAG-005 Status PCU
* \requirement \b TIR-SW-DIAG-006 Status of PowerStage
* \requirement \b TIR-SW-DIAG-007 Pump/motor blocked detection
* \requirement \b TIR-SW-DIAG-008 CDE_MJP Limp mode
* \requirement \b TIR-SW-DIAG-009 CDE_MJP frequency out of specification
* \requirement \b TIR-SW-DIAG-010 SetDiagAlarm()
* \requirement \b TIR-SW-DIAG-011 Alarm set by LIN * 
* \requirement \b TIR-SW-DIAG-012 ETAT_MJP out of specification
* \requirement \b TIR-SW-DIAG-013 I Phase floating detection
* \requirement \b TIR-SW-DIAG-014 Alarm Phase I sense
* \requirement \b TIR-SW-DIAG-015 U_Batt Sense
* \test \b TC_TIR-SW-DIAG-002 Overtemperature
* \test \b TC_TIR-SW-DIAG-003 Current consumption
* \test \b TC_TIR-SW-DIAG-004 Pump current consumption to low
* \test \b TC_TIR-SW-DIAG-005 status of PCU
* \test \b TC_TIR-SW-DIAG-006 status of Mosfets and pump winding
* \test \b TC_TIR-SW-DIAG-007 Check Pump state and set alarm
* \test \b TC_TIR-SW-DIAG-008 PWM Input Limp Mode
* \test \b TC_TIR-SW-DIAG-009 PWM in out of specification
* \test \b TC_TIR-SW-DIAG-010 SetDiagAlarm()
* \test \b TC_TIR-SW-DIAG-011 Alarm set by LIN
* \test \b TC_TIR-SW-DIAG-012 PWM out
* \test \b TC_TIR-SW-DIAG-013 Ph_I_Sense floating detection
* \test \b TC_TIR-SW-DIAG-014 Alarm Ph_I_Sense
* \test \b TC_TIR-SW-DIAG-015 U_batt_sense
**/

/*~T*/
/*----------------  Includes               --------------------------------------------------------------*/
#include     <htc.h>
#include     "config.h"
#include     "project.h"
#include     "BVH2_Appl_Layer.h"
#include     "pwm.h"
#include     "adc.h"
#include     "diag.h"
#include     "bldc.h"
#include     "eeprom.h"
#include     "lin.h"
#include     "cksum.h"

/*~T*/
/*----------------  Defines      --------------------------------------------------------------*/
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



/*~T*/
/*--------------------------  globale Variables -----------------------------------------------*/
unsigned  char ui8_lin_sim_Failures; /**< Put an error signal on ETAT_MJP by LIN */
unsigned  char ui8_lin_sim_Failures_ena; /**< Enable signal for the LIN communication for error signals */
unsigned  char Error_PICetatMonitor; /**< 1 if bad ETAT_MJP, else 0 */
unsigned  char DC_pic_etat_monitor; /**< DC read by pin RA6 */
unsigned  char ui8_cnt1;/**< Not used but initialised in DiagInit(void) */
unsigned  char ui8_cnt_PWM;/**< Counter for the verification of ETAT_MJP */
unsigned  char ict_stamp = 1;            /**< Signal to enable the ICT Stamp writing in EOL */
unsigned  char wkpoint = 1; /**< Detection of the Working Point in EOL */
unsigned  char phaseCal = 0; /**< Avancement for current calibration. Set from 0 to 3. 0 is corresponding to the saving of the datas, and the other for the writting in EEPROM and the transmission of the signal */
unsigned  char ui8_failure = 7;          /**< Initialisation for the failure alarms */
unsigned char ui8_given_supply;
unsigned int ui16_ambiant_temperature;
_u_wb ui16_IPhase1_bldc_cal;
_u_wb ui16_IPhase2_bldc_cal;
_u_wb ui16_IPhase3_bldc_cal;
extern         unsigned char ui8_current_cal[3];

_u_bits        ui8_error_Flags; /**< Error flags */

extern  unsigned  char ui8_b_DResServID_c;/* EOL SID; defined in lin.c */
extern  unsigned  char ui8_b_DResLocID_c;/* EOL LID; defined in lin.c */
extern  unsigned  char ui8_b_DResB0_c;  /* EOL Data1; defined in lin.c */
extern  unsigned  char ui8_b_DResB1_c;  /* EOL Data2; defined in lin.c */
extern  unsigned  char ui8_b_DResB2_c;  /* EOL Data3; defined in lin.c */
extern  unsigned  char ui8_b_DResB3_c;  /* EOL Data4; defined in lin.c */
extern  unsigned  char ui8_b_DResB4_c;  /* EOL Data5; defined in lin.c */
extern  unsigned  char ui8_b_DResB5_c;  /* EOL Data6; defined in lin.c */
extern  unsigned  char ui8_selected_lid;


extern  _D_TYPE        sum;
extern  _D_TYPE        checksum[ 3 ];

extern  unsigned  int  ui16_I_cal_Ph1; /* Calibration value for I_Phase1 */
extern  unsigned  int  ui16_I_cal_Ph2; /* Calibration value for I_Phase2 */
extern  unsigned  int  ui16_I_cal_Ph3; /* Calibration value for I_Phase3 */
extern  unsigned  int  ui16_Temp_cal; /* Calibration value for CPU temperature */

/*~T*/
/*-----------------------------  Prototypes --------------------------------------------------------------*/
void DiagInit( void  );
void SetDiagAlarm( void  );
void NegativeAnswer(unsigned char LID);
/*~A*/
/*~+:void DiagInit(void)*/
/*~T*/
/**
* \fn      void DiagInit(void)
* \brief   Initialisation of Flags. 
* \pre     None
* \post    None
* \details This function initialises the flags for errors to 0 and force the output DC to 11% DC         
**/
/*~F*/
void DiagInit( void  )
/*~-*/
{
   /*~I*/
#ifdef def_diag_init_values
   /*~T*/
   Error_PICetatMonitor = 0;
   DC_pic_etat_monitor  = 0;
   ui8_cnt1             = 0;

   /*~-*/
#endif
   /*~E*/
   /*~T*/
   ui8_error_Flags.b = 0;/* Set all the flags to 0 */
   PWM_Write_Out( 11 ); /* Force the output DC to normal behavior */ 
   /*~T*/
   ui16_IPhase1_bldc_cal.w = 0;
   ui16_IPhase2_bldc_cal.w = 0;
   ui16_IPhase3_bldc_cal.w = 0;


/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void SetDiagAlarm(void )*/
/*~+:*/
/*~T*/
/**
* \fn      void SetDiagAlarm(void)
* \brief   Set the DC output for the alarms. 
* \pre     None
* \post    None
* \details This function force the ouptut DC to 11% if <tt>def_SW_PSA_YBS</tt> is defined\n
*          If not, and if the LIN communication is enable, the output DC can be set by LIN\n
*          Else, set the DC due to the alarm detected in the Matlab layer:\n
*                If ECU error, set ETAT_MJP DC to 88%\n
*                If Over temperature, set ETAT_MJP DC to 33%\n
*                If Dry running, set ETAT_MJP DC to 66%\n
*                If Motor stalled, set ETAT_MJP DC to 55%\n
*                If Over current, set ETAT_MJP DC to 44%\n
*                If PWM input frequency error, set ETAT_MJP DC to 77%\n
*                If PWM input DC error, set ETAT_MJP DC to 22%\n
*                Else, set ETAT_MJP DC to 11%
**/
/*~I*/
#ifdef def_SW_PSA_YBS
/*~F*/
void SetDiagAlarm( void  )
/*~-*/
{
   /*~T*/
   PWM_Write_Out( 11 ); /* Force the output DC to normal behavior */ 
/*~-*/
}
/*~E*/
/*~O*/
/*~-*/
#else
/*~F*/
void SetDiagAlarm( void  )

/*~-*/
{
   /*~A*/
   /*~+:Normal command*/
   /*~T*/
   /* Set the different output values for the diagnostics */
   /*~I*/
   if( bool_PowerStage_err_Flag )
   /*~-*/
   {
      /*~T*/
      PWM_Write_Out( 88 );

   /*~-*/
   }
   /*~O*/
   /*~-2*/
   else
   {
      /*~I*/
      if( bool_CPU_TempAlarm )

      /*~-*/
      {
         /*~T*/
         PWM_Write_Out( 33 );

      /*~-*/
      }
      /*~O*/
      /*~-2*/
      else
      {
         /*~I*/
         if( bool_DryRunningAlarm )
         /*~-*/
         {
            /*~T*/
            PWM_Write_Out( 66 );

         /*~-*/
         }
         /*~O*/
         /*~-2*/
         else
         {
            /*~I*/
            if( bool_MotorStalled )
            /*~-*/
            {
               /*~T*/
               PWM_Write_Out( 55 );

            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~I*/
               if( bool_HighCurrentAlarm )
               /*~-*/
               {
                  /*~T*/
                   PWM_Write_Out( 44 );
               /*~-*/
               }
               /*~O*/
               /*~-2*/
               else
               {
                  /*~I*/
                  if( bool_PWMin_Freq_err_Flag )

                  /*~-*/
                  {
                     /*~T*/
                     PWM_Write_Out( 77 );

                  /*~-*/
                  }
                  /*~O*/
                  /*~-2*/
                  else
                  {
                     /*~I*/
                     if( bool_PWMin_err_Flag )
                     /*~-*/
                     {
                        /*~T*/
                        PWM_Write_Out( 22 );

                     /*~-*/
                     }
                     /*~O*/
                     /*~-2*/
                     else
                     {
                        /*~T*/
                        PWM_Write_Out( 11 );
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
      /*~-*/
      }
      /*~E*/
   /*~-*/
   }
   /*~E*/
   /*~T*/
   /* If ECU error, set ETAT_MJP DC to 88%
   *   If Over temperature, set ETAT_MJP DC to 33%
   *   If Dry running, set ETAT_MJP DC to 66%
   *   If Motor stalled, set ETAT_MJP DC to 55%
   *   If Over current, set ETAT_MJP DC to 44%
   *   If PWM input frequency error, set ETAT_MJP DC to 77%
   *   If PWM input DC error, set ETAT_MJP DC to 22%
   *   Else, set ETAT_MJP DC to 11% */
   /*~E*/
/*~-*/
}
/*~E*/
/*~-*/
#endif
/*~E*/
/*~E*/
/*~A*/
/*~+:void DiagPicEtatMonitor(void)*/
/*~T*/
/**
* \fn      void DiagPicEtatMonitor(void)
* \brief   Verify if the output ETAT_MJP of the microcontroller is as expected
* \pre     None
* \post    None
* \details This function verify that the output ETAT_MJP is not short circuited
*          It is detecting if both output of microcontroller and output of the system are at the same value or not.
*          If the values are not the same, the system waits 5 calls and put the output to 0 to protect the HW.
**/
/*~F*/
void DiagPicEtatMonitor( void  )
/*~-*/
{
   /*~I*/
#ifdef ver_ETAT
   /*~I*/
   if( Port_PWMout == 1 )
   /*~-*/
   {
      /*~I*/
      if( PIC_ETAT_MONITOR == 1 )
      /*~-*/
      {
         /*~T*/
         ui8_cnt_PWM++;
         /*~I*/
         if( ui8_cnt_PWM > 5 )
         /*~-*/
         {
            /*~T*/
            Port_PWMout = 0;
            //PWM_Write_Out(100);
            //ui8_PWMoutvalue = 99;
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
         ui8_cnt_PWM = 0;
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
      if( PIC_ETAT_MONITOR == 0 )
      /*~-*/
      {
         /*~T*/
         ui8_cnt_PWM++;
         /*~I*/
         if( ui8_cnt_PWM > 5 )
         /*~-*/
         {
            /*~T*/
            Port_PWMout = 0;
            //PWM_Write_Out(100);
            //ui8_PWMoutvalue = 99;
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
         ui8_cnt_PWM = 0;
      /*~-*/
      }
      /*~E*/
   /*~-*/
   }
   /*~E*/
   /*~-*/
#endif
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void EOL(void)    LIN configuration*/
/*~T*/
/**
* \fn      void EOL(void)
* \brief   Routine to manage the EOL tests
* \pre     None
* \post    None
* \details This function is configuring the EOL test for each electronics. It is running at the first utilisation when the EEPROM is empty.
 *
**/
/*~F*/
void EOL( void  )
/*~-*/
{
   /*~T*/
   unsigned  char ui8_b_DResB0_RD;
   unsigned  char ui8_b_DResB1_RD;
   unsigned  char ui8_b_DResB2_RD;
   unsigned  char ui8_b_DResB3_RD;
   unsigned  char ui8_b_DResB4_RD;
   unsigned  char ui8_b_DResB5_RD;

   //#define transmit;
   /*~I*/
   if( ui8_b_DResServID_c == 0x3b )
   /*~-*/
   {
      /*~T*/
      /** ## If SID = 0x3b, receive the data from the tester */
      /*~I*/
#ifdef def_EOL_oneTest
      /*~T*/
      ui8_selected_lid = ui8_b_DResLocID_c;
      /*~-*/
#endif
      /*~E*/
      /*~C*/
      switch( ui8_b_DResLocID_c) /* allmost same structure */ 
      /*~-*/
      {
         /*~A*/
         /*~+:case 0x80: eol_sequence_start*/
         /*~F*/
         case ( eol_sequence_start ):
         /*~-*/
         {
            /*~I*/
            if( eol_sequence_start == ui8_selected_lid )
            /*~-*/
            {
               /*~T*/
               ui8_selected_lid = ict_stamp_presence;
               ict_stamp        = 1;

               Transmit_LIN_8Bytes( 0x10, 0x3A, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 );


            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~I*/
#ifdef transmit
               /*~T*/
               NegativeAnswer(ui8_b_DResLocID_c);
               /*~-*/
#endif
               /*~E*/
            /*~-*/
            }
            /*~E*/
            /*~T*/
            break;
         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~A*/
         /*~+:case 0x90: ict_stamp_presence*/
         /*~F*/
         case ( ict_stamp_presence ):
         /*~-*/
         {
            /*~I*/
            if( ict_stamp_presence == ui8_selected_lid )
            /*~-*/
            {
               /*~T*/
               /** <b>LID = 0x90</b>: Transmit to the tester the HW informations:
               *      - HW version MSB
               *      - HW version LSB
               *      - HW revision MSB
               *      - HW revision LSB
               *      - 0x00
               *      - 0x00\n
               * Then transmit:
               *      - YY
               *      - TT MSB
               *      - TT LSB
               *      - XX MSB
               *      - XX LSB
               *      - 0x00
               */
               /*~C*/
               switch( ict_stamp )
               /*~-*/
               {
                  /*~F*/
                  case 1:
                  /*~-*/
                  {
                     /*~T*/
                     ui8_b_DResB0_RD = read_eeprom_data( 0xf0 ); /* HW_Ver MSB */
                     ui8_b_DResB1_RD = read_eeprom_data( 0xf1 ); /* HW_Ver LSB */
                     ui8_b_DResB2_RD = read_eeprom_data( 0xf2 ); /* HW_Rev MSB */
                     ui8_b_DResB3_RD = read_eeprom_data( 0xf3 ); /* HW_Rev LSB */
                     ui8_b_DResB4_RD = 0;
                     ui8_b_DResB5_RD = 0;

                     ict_stamp       = 2;
                     break;
                  /*~-*/
                  }
                  /*~E*/
                  /*~F*/
                  case 2:
                  /*~-*/
                  {
                     /*~T*/
                     ui8_b_DResB0_RD = read_eeprom_data( 0xeb ); /* YY */
                     ui8_b_DResB1_RD = read_eeprom_data( 0xec ); /* TT_MSB */
                     ui8_b_DResB2_RD = read_eeprom_data( 0xed ); /* TT_LSB */
                     ui8_b_DResB3_RD = read_eeprom_data( 0xee ); /* XX_MSB */
                     ui8_b_DResB4_RD = read_eeprom_data( 0xef ); /* XX_LSB */
                     ui8_b_DResB5_RD = 0;

                     ui8_selected_lid = fct_stamp_absence;
                     break;
                  /*~-*/
                  }
                  /*~E*/
                  /*~O*/
                  /*~-2*/
                  default:
                  {
                     /*~T*/

                     NegativeAnswer(ict_stamp_presence);
                     break;
                  /*~-*/
                  }
               /*~-*/
               }
               /*~E*/
               /*~I*/
               if ((ui8_b_DResB0_RD != 0xFF) && (ui8_b_DResB1_RD != 0xFF) && (ui8_b_DResB2_RD != 0xFF) && (ui8_b_DResB3_RD != 0xFF) && (ui8_b_DResB4_RD != 0xFF) && (ui8_b_DResB5_RD != 0xFF))
               /*~-*/
               {
                  /*~T*/
                  Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD, ui8_b_DResB1_RD, ui8_b_DResB2_RD, ui8_b_DResB3_RD, ui8_b_DResB4_RD, ui8_b_DResB5_RD );

               /*~-*/
               }
               /*~O*/
               /*~-2*/
               else
               {
                  /*~T*/
                  Transmit_LIN_8Bytes( 0x10, 0x7F, ict_stamp_presence, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );


                  /*~I*/
#ifdef def_EOL_ShutDown
                  /*~T*/
                  write_eeprom_data(0x55,0x55); /** Write in EEPROM at address 0x55 a flag to say that the EOL are finished */
                  RC5 = 0; /* Disable LIN */
                  /*~-*/
#endif
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
#ifdef transmit
               /*~T*/

               NegativeAnswer(ui8_b_DResLocID_c);
               /*~-*/
#endif
               /*~E*/
            /*~-*/
            }
            /*~E*/
            /*~T*/
            break;
         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~A*/
         /*~+:case 0x91: fct_stamp_absence*/
         /*~F*/
         case ( fct_stamp_absence ):
         /*~-*/
         {
            /*~I*/
            if( fct_stamp_absence == ui8_selected_lid )
            /*~-*/
            {
               /*~T*/
               /** <b>LID = 0x91</b>: Transmit to the tester the FCT informations:
               *      - Prod_Year_FCT
               *      - Prod_Day_FCT MSB
               *      - Prod_Day_FCT LSB
               *      - Prod_Num_FCT MSB
               *      - Prod_Num_FCT LSB
               *      - 0x00
               */
               /*~T*/
               ui8_b_DResB0_RD = read_eeprom_data( 0xf4 ); /* Prod_Year_FCT */
               ui8_b_DResB1_RD = read_eeprom_data( 0xf5 ); /* Prod_Day_FCT MSB */
               ui8_b_DResB2_RD = read_eeprom_data( 0xf6 ); /* Prod_Day_FCT LSB */
               ui8_b_DResB3_RD = read_eeprom_data( 0xf7 ); /* Prod_Num_FCT MSB */
               ui8_b_DResB4_RD = read_eeprom_data( 0xf8 ); /* Prod_Num_FCT LSB */
               ui8_b_DResB5_RD = 0;
               /*~I*/
               if ((ui8_b_DResB0_RD == 0xFF) && (ui8_b_DResB1_RD == 0xFF) && (ui8_b_DResB2_RD == 0xFF) && (ui8_b_DResB3_RD == 0xFF) && (ui8_b_DResB4_RD == 0xFF))
               /*~-*/
               {
                  /*~T*/
                  Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD, ui8_b_DResB1_RD, ui8_b_DResB2_RD, ui8_b_DResB3_RD, ui8_b_DResB4_RD, ui8_b_DResB5_RD );

               /*~-*/
               }
               /*~O*/
               /*~-2*/
               else
               {
                  /*~T*/
                  Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );
                  /*~I*/
#ifdef def_EOL_ShutDown
                  /*~T*/
                  write_eeprom_data(0x55,0x55); /** Write in EEPROM at address 0x55 a flag to say that the EOL are finished */
                  RC5 = 0; /* Disable LIN */
                  /*~-*/
#endif
                  /*~E*/
               /*~-*/
               }
               /*~E*/
               /*~T*/
               ui8_selected_lid = temperature_calibration;

            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~T*/

               /*~I*/
#ifdef transmit
               /*~T*/
               NegativeAnswer(ui8_b_DResLocID_c);
               /*~-*/
#endif
               /*~E*/
            /*~-*/
            }
            /*~E*/
            /*~T*/
            break;
         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~I*/
#ifdef temperature_calibration_enable
         /*~A*/
         /*~+:case 0x92: temperature_calibration*/
         /*~F*/
         case ( temperature_calibration ):
         /*~-*/
         {
            /*~I*/
            if( temperature_calibration == ui8_selected_lid )
            /*~-*/
            {
               /*~T*/
               /** If <b>LID = 0x92</b>, write the temperature calibration in EEPROM (address 0x00 and 0x01)*/
               /*~T*/
               /** The ambiant temperature must be given by the tester to the electronic. The temperature is coded on 10 bits on the bytes 3 and 4 of the Lin frame. The ratio is Temperature_ADC = 1.08 * Temperature + 527
               /*~T*/
               ui16_ambiant_temperature = (ui8_b_DResB0_c<<8) | ui8_b_DResB1_c;

               /*~T*/
               write_eeprom_data( 0x00, (unsigned char)( ( ui16_ambiant_temperature - ui16_NTC_Temp_bldc_mean )>>8 ) );
               write_eeprom_data( 0x01, (unsigned char)(   ui16_ambiant_temperature - ui16_NTC_Temp_bldc_mean ) );

               ui16_Temp_cal = ( read_eeprom_data( 0x00 )<<8 ) | read_eeprom_data( 0x01 );
               /*~T*/
               /** <b>LID = 0x92</b>: Transmit to the tester the temperature information:
               *      - Calibrated temperature 8 bits
               *      - Temperature calibration value 8 bits
               *      - 0x00
               *      - 0x00
               *      - 0x00
               *      - 0x00
               */
               /*~T*/
               ui8_b_DResB0_RD = ( unsigned char )( ( ui16_NTC_Temp_bldc_mean + ui16_Temp_cal )>>2 ); /* read temperature */
               ui8_b_DResB1_RD = ( unsigned char )( ui16_Temp_cal>>2 );
               ui8_b_DResB2_RD = 0;
               ui8_b_DResB3_RD = 0;
               ui8_b_DResB4_RD = 0;
               ui8_b_DResB5_RD = 0;


               /*~I*/
               if (ui8_b_DResB0_RD == (ui16_ambiant_temperature>>2))
               /*~-*/
               {
                  /*~T*/
                  Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD, ui8_b_DResB1_RD, ui8_b_DResB2_RD,  ui8_b_DResB3_RD, ui8_b_DResB4_RD, ui8_b_DResB5_RD );

               /*~-*/
               }
               /*~O*/
               /*~-2*/
               else
               {
                  /*~T*/
                  Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );
                  /*~I*/
#ifdef def_EOL_ShutDown
                  /*~T*/
                  write_eeprom_data(0x55,0x55); /** Write in EEPROM at address 0x55 a flag to say that the EOL are finished */
                  RC5 = 0; /* Disable LIN */
                  /*~-*/
#endif
                  /*~E*/
               /*~-*/
               }
               /*~E*/
               /*~T*/
               ui8_selected_lid = sw_version_number;

            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~T*/

               /*~I*/
#ifdef transmit
               /*~T*/
               NegativeAnswer(ui8_b_DResLocID_c);
               /*~-*/
#endif
               /*~E*/
            /*~-*/
            }
            /*~E*/
            /*~T*/
            break;
         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~-*/
#endif
         /*~E*/
         /*~A*/
         /*~+:case 0x93: sw_version_number*/
         /*~F*/
         case ( sw_version_number ):
         /*~-*/
         {
            /*~I*/
            if( sw_version_number == ui8_selected_lid )
            /*~-*/
            {
               /*~T*/
               /** <b>LID = 0x93</b>: Transmit to the tester the Software information:
               *      - Software version MSB
               *      - Software version LSB
               *      - Checksum read MSB
               *      - Checksum read LSB
               *      - Checksum measured MSB
               *      - Checksum measured LSB
               */
               /*~T*/
               /* Checksum verification OK. Done just after each reset of the system. If checksum OK, pump is running else, wait... */
               sum           = cksum( );
               checksum[ 1 ] = ( FLASH_READ( 0x1FFF ) );
               checksum[ 2 ] = ( FLASH_READ( 0x1FFE ) );
               checksum[ 0 ] = ( checksum[ 1 ]<<8 ) + checksum[ 2 ];

               /*~T*/
               ui8_b_DResB0_RD = ui8_SoftwareVersion_MSB; /* SW_Ver MSB */
               ui8_b_DResB1_RD = ui8_SoftwareVersion_LSB; /* SW_Ver LSB */
               ui8_b_DResB2_RD = checksum[1] ;
               ui8_b_DResB3_RD = checksum[2] ;
               ui8_b_DResB4_RD = (unsigned int)((sum)>>8) ;
               ui8_b_DResB5_RD = (unsigned int)(sum) ;


               /*~I*/
               if (checksum[0] == sum)
               /*~-*/
               {
                  /*~T*/
                  Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD, ui8_b_DResB1_RD, ui8_b_DResB2_RD,
                  ui8_b_DResB3_RD, ui8_b_DResB4_RD, ui8_b_DResB5_RD );

               /*~-*/
               }
               /*~O*/
               /*~-2*/
               else
               {
                  /*~T*/
                  Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );
                  /*~I*/
#ifdef def_EOL_ShutDown
                  /*~T*/
                  write_eeprom_data(0x55,0x55); /** Write in EEPROM at address 0x55 a flag to say that the EOL are finished */
                  RC5 = 0; /* Disable LIN */
                  /*~-*/
#endif
                  /*~E*/
               /*~-*/
               }
               /*~E*/
               /*~T*/
               ui8_selected_lid = supply_voltage_verification;


            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~I*/
#ifdef transmit
               /*~T*/
               NegativeAnswer(ui8_b_DResLocID_c);
               /*~-*/
#endif
               /*~E*/
            /*~-*/
            }
            /*~E*/
            /*~T*/
            break;

         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~A*/
         /*~+:case 0x94: supply_voltage_verification*/
         /*~F*/
         case ( supply_voltage_verification ):
         /*~-*/
         {
            /*~I*/
            if( supply_voltage_verification == ui8_selected_lid )
            /*~-*/
            {
               /*~T*/
               /** <b>LID = 0x94</b>: Transmit to the tester the supply voltage information:
               *      - Supply voltage 
               *      - 0x00
               *      - 0x00
               *      - 0x00
               *      - 0x00
               *      - 0x00
               */
               /*~T*/
               /** The tester gives the power supply by sending on byte 3 the corresponding value. The ratio is Suppy_ADC = 8.33 * Supply(V) */
               /*~T*/
               ui8_given_supply = (ui8_b_DResB0_c);
               /*~I*/
               if (ui8_given_supply == 0)
               /*~-*/
               {
                  /*~T*/
                  ui8_given_supply = 0x6F;
               /*~-*/
               }
               /*~E*/
               /*~T*/
               ui8_b_DResB0_RD = ( unsigned char )( ui16_fir_Bat_mittel>>2 ); /* Supply Voltage MSB */
               ui8_b_DResB1_RD = 0; /* Supply Voltage LSB */
               ui8_b_DResB2_RD = 0;
               ui8_b_DResB3_RD = 0;
               ui8_b_DResB4_RD = 0;
               ui8_b_DResB5_RD = 0;



               /*~I*/
               if ((ui8_b_DResB0_RD > ui8_given_supply - 3) && (ui8_b_DResB0_RD < ui8_given_supply + 3) )
               //if ((ui8_b_DResB0_RD > 0x6C) && (ui8_b_DResB0_RD < 0x72) )
               /** Verification that for 13.5V, the ADC for voltage is detecting a value between 13.3V and 13.7V */
               /*~-*/
               {
                  /*~T*/
                  Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD, ui8_b_DResB1_RD, ui8_b_DResB2_RD,
                  ui8_b_DResB3_RD, ui8_b_DResB4_RD, ui8_b_DResB5_RD );

               /*~-*/
               }
               /*~O*/
               /*~-2*/
               else
               {
                  /*~T*/
                  Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );
                  /*~I*/
#ifdef def_EOL_ShutDown
                  /*~T*/
                  write_eeprom_data(0x55,0x55); /** Write in EEPROM at address 0x55 a flag to say that the EOL are finished */
                  RC5 = 0; /* Disable LIN */
                  /*~-*/
#endif
                  /*~E*/
               /*~-*/
               }
               /*~E*/
               /*~T*/
               ui8_selected_lid = current_calibration_ph1;

            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~T*/


               /*~I*/
#ifdef transmit
               /*~T*/
               NegativeAnswer(ui8_b_DResLocID_c);
               /*~-*/
#endif
               /*~E*/
            /*~-*/
            }
            /*~E*/
            /*~T*/
            break;
         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~A*/
         /*~+:current_calibration_ph1*/
         /*~F*/
         case ( current_calibration_ph1 ):
         /*~-*/
         {
            /*~I*/
            if( current_calibration_ph1 == ui8_selected_lid )
            /*~-*/
            {
               /*~T*/
               /* Stop the motor and enable the upper transistor for Phase1 */

               /*~C*/
               switch (phaseCal)
               /*~-*/
               {
                  /*~F*/
                  case 0:
                  /*~-*/
                  {
                     /*~T*/
                     InitMotorStop( );

                     /*~T*/
                     PEIE = 0; /** UnSet all peripheral interrupts */
                     GIE  = 0; /** UnSet all global interrupts */
                     CCP1CON = 0x00; /** Unlock the Direction Pins */

                     /*~T*/
                     /* Phase 2 calibration */

                     /*~T*/
                     PhaseB_Dir = 1;
                     PhaseB_Ena = 1;

                     /*~T*/
                     PhaseA_Ena = 0;
                     PhaseC_Ena = 0;
                     /*~T*/
                     mADC_ChanSelect(Ph2_ISen_Channel);
                     /*~A*/
                     /*~+:Wait 120 us*/
                     /*~T*/
                     DelayUs( 200 );
                     DelayUs( 200 );
                     DelayUs( 200 );
                     DelayUs( 200 );
                     DelayUs( 200 );
                     DelayUs( 200 );
                     /*~E*/
                     /*~T*/
                     ADC_Wait();
                     mADC_TrigConversion( );
                     ui16_IPhase2_bldc_cal.w = ADC_Read() - ui8_current_cal[1];

                     /*~T*/
                     /* Phase 1 calibration */

                     /*~T*/
                     PhaseA_Dir = 1;
                     PhaseA_Ena = 1;

                     /*~T*/
                     PhaseB_Ena = 0;
                     PhaseC_Ena = 0;
                     /*~T*/
                     mADC_ChanSelect(Ph1_ISen_Channel);
                     /*~A*/
                     /*~+:Wait 120 us*/
                     /*~T*/
                     DelayUs( 200 );
                     DelayUs( 200 );
                     DelayUs( 200 );
                     DelayUs( 200 );
                     DelayUs( 200 );
                     DelayUs( 200 );
                     /*~E*/
                     /*~T*/
                     ADC_Wait();
                     mADC_TrigConversion( );
                     ui16_IPhase1_bldc_cal.w = ADC_Read() - ui8_current_cal[0];

                     /*~T*/
                     /* Phase 3 calibration */

                     /*~T*/
                     PhaseC_Dir = 1;
                     PhaseC_Ena = 1;
                     /*~T*/
                     PhaseA_Ena = 0;
                     PhaseB_Ena = 0;
                     /*~T*/
                     mADC_ChanSelect(Ph3_ISen_Channel);

                     /*~A*/
                     /*~+:Wait 120 us*/
                     /*~T*/
                     DelayUs( 200 );
                     DelayUs( 200 );
                     DelayUs( 200 );
                     DelayUs( 200 );
                     DelayUs( 200 );
                     DelayUs( 200 );
                     /*~E*/
                     /*~T*/
                     ADC_Wait();
                     mADC_TrigConversion( );
                     ui16_IPhase3_bldc_cal.w = ADC_Read() - ui8_current_cal[2];

                     /*~T*/
                     PhaseX_DIS;

                     /*~T*/
                     CCP1CON = 0x0c; /** Lock Direction Pins */
                     PEIE = 1; /** Set all peripheral interrupts */
                     GIE  = 1; /** Set all global interrupts */

                     /*~T*/
                     phaseCal = 1;

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
                     /** If <b>LID = 0x95</b>, write the current of the phase 1 into the EEPROM (address 0x02 and 0x03) */
                     /*~T*/
                     /* 0x7800 corresponds to 0x78 = 120 = 6A from the nominal value for 6A. This value 0x78 is multiplied by 256 to have more resolution for the calibrated value. Then the nominal value is divided by the current value for the calibration to measure the factor for the current ramp. */
                     write_eeprom_data( 0x02, ( unsigned char )( ( 0x7800 / ui16_IPhase1_bldc_cal.w )>>8 ) );
                     write_eeprom_data( 0x03, ( unsigned char )(   0x7800 / ui16_IPhase1_bldc_cal.w      ) );

                     /*~T*/
                     ui16_I_cal_Ph1 = ( read_eeprom_data( 0x02 )<<8 ) | read_eeprom_data( 0x03 );
                     /*~T*/
                     /** <b>LID = 0x95</b>: Transmit to the tester the current of phase 1 information:
                     *      - Calibrated current of phase 1 MSB
                     *      - Calibrated current of phase 1 LSB
                     *      - 0x00
                     *      - Calibration value of the current of phase 1 MSB
                     *      - Calibration value of the current of phase 1 LSB
                     *      - 0x00
                     */
                     /*~T*/
                     /* Assignement of Data for transmission */
                     ui8_b_DResB0_RD = phaseCal;
                     ui8_b_DResB1_RD = ( unsigned char )( ( ( ui16_IPhase1_bldc_cal.w * ui16_I_cal_Ph1 )>>8 )>>8 ); /* 0x00 expected */
                     ui8_b_DResB2_RD = ( unsigned char )(   ( ui16_IPhase1_bldc_cal.w * ui16_I_cal_Ph1 )>>8 ); /* 0x78 expected */
                     ui8_b_DResB3_RD = ( unsigned char )( ui16_I_cal_Ph1 >> 8); /* If the ramp is matching to the nominal value, ui16_I_cal_Ph1 is 0x0100 */
                     ui8_b_DResB4_RD = ( unsigned char )( ui16_I_cal_Ph1     );
                     ui8_b_DResB5_RD = 0;
                     //ui8_b_DResB2_RD = ui16_IPhase1_bldc_cal.w;

                     /*~T*/
                     /* Transmission */
                     Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD,
                                          ui8_b_DResB1_RD, ui8_b_DResB2_RD,
                                          ui8_b_DResB3_RD, ui8_b_DResB4_RD,
                                          ui8_b_DResB5_RD );
                     //ui8_selected_lid = current_calibration_ph2;
                     phaseCal = 2;
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
                     /** If <b>LID = 0x96</b>, write the current of the phase 2 into the EEPROM (address 0x04 and 0x05) */
                     /*~T*/
                     write_eeprom_data( 0x04, ( unsigned char )( ( 0x7800 / ui16_IPhase2_bldc_cal.w )>>8 ) );
                     write_eeprom_data( 0x05, ( unsigned char )(   0x7800 / ui16_IPhase2_bldc_cal.w      ) );
                     /*~T*/
                     /* Calibration value */
                     ui16_I_cal_Ph2 = ( read_eeprom_data( 0x04 )<<8 ) | read_eeprom_data( 0x05 );
                     /*~T*/
                     /** <b>LID = 0x96</b>: Transmit to the tester the current of phase 2 information:
                     *      - Calibrated current of phase 2 MSB
                     *      - Calibrated current of phase 2 LSB
                     *      - 0x00
                     *      - Calibration value of the current of phase 2 MSB
                     *      - Calibration value of the current of phase 2 LSB
                     *      - 0x00
                     */
                     /*~T*/
                     /* Assignement for transmission */
                     ui8_b_DResB0_RD = phaseCal;
                     ui8_b_DResB1_RD = ( unsigned char )( ( ( ui16_IPhase2_bldc_cal.w * ui16_I_cal_Ph2 )>>8 )>>8 );
                     ui8_b_DResB2_RD = ( unsigned char )( ( ( ui16_IPhase2_bldc_cal.w * ui16_I_cal_Ph2 )>>8 )    );
                     ui8_b_DResB3_RD = ( unsigned char )( ui16_I_cal_Ph2>>8 );
                     ui8_b_DResB4_RD = ( unsigned char )( ui16_I_cal_Ph2 );
                     ui8_b_DResB5_RD = 0;
                     //ui8_b_DResB2_RD = ui16_IPhase2_bldc_cal.w;

                     /*~T*/
                     /* Transmission */
                     Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD,
                                          ui8_b_DResB1_RD, ui8_b_DResB2_RD,
                                          ui8_b_DResB3_RD, ui8_b_DResB4_RD,
                                          ui8_b_DResB5_RD );
                     //ui8_selected_lid = current_calibration_ph3;

                     phaseCal = 3;
                     /*~T*/
                     break;
                  /*~-*/
                  }
                  /*~E*/
                  /*~F*/
                  case 3:
                  /*~-*/
                  {
                     /*~T*/
                     /** If <b>LID = 0x97</b>, write the current of the phase 3 into the EEPROM (address 0x06 and 0x07) */
                     /*~T*/
                     write_eeprom_data( 0x06, ( unsigned char )( ( 0x7800 / ui16_IPhase3_bldc_cal.w )>>8 ) );
                     write_eeprom_data( 0x07, ( unsigned char )( 0x7800 / ui16_IPhase3_bldc_cal.w ) );

                     /*~T*/
                     /* calibration value */
                     ui16_I_cal_Ph3 = ( read_eeprom_data( 0x06 )<<8 ) | read_eeprom_data( 0x07 );

                     /*~T*/
                     /** <b>LID = 0x97</b>: Transmit to the tester the current of phase 3 information:
                     *      - Calibrated current of phase 3 MSB
                     *      - Calibrated current of phase 3 LSB
                     *      - 0x00
                     *      - Calibration value of the current of phase 3 MSB
                     *      - Calibration value of the current of phase 3 LSB
                     *      - 0x00
                     */
                     /*~T*/
                     /* Assignement for transmission */
                     ui8_b_DResB0_RD = phaseCal;
                     ui8_b_DResB1_RD = ( unsigned char )( ( ( ui16_IPhase3_bldc_cal.w * ui16_I_cal_Ph3 )>>8 )>>8 );
                     ui8_b_DResB2_RD = ( unsigned char )( ( ( ui16_IPhase3_bldc_cal.w * ui16_I_cal_Ph3 )>>8 )    );
                     ui8_b_DResB3_RD = ( unsigned char )( ui16_I_cal_Ph3>>8 );
                     ui8_b_DResB4_RD = ( unsigned char )( ui16_I_cal_Ph3 );
                     ui8_b_DResB5_RD = 0;
                     //ui8_b_DResB2_RD = ui16_IPhase3_bldc_cal.w;
                     /*~T*/
                     /* Transmission */
                     Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD,
                                          ui8_b_DResB1_RD, ui8_b_DResB2_RD,
                                          ui8_b_DResB3_RD, ui8_b_DResB4_RD,
                                          ui8_b_DResB5_RD );
                     ui8_selected_lid = speed_wp;

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
                     NegativeAnswer(ui8_b_DResLocID_c);
                  /*~-*/
                  }
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
               NegativeAnswer(ui8_b_DResLocID_c);
            /*~-*/
            }
            /*~E*/
            /*~T*/
            break;
         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~A*/
         /*~+:case 0x98: speed_wp*/
         /*~F*/
         case ( speed_wp ):
         /*~-*/
         {
            /*~I*/
            if( speed_wp == ui8_selected_lid )
            /*~-*/
            {
               /*~T*/
               /** <b>LID = 0x98</b>: Transmit to the tester the speed information:
               *      - PWM input DC
               *      - Speed of the motor
               *      - Speed demand
               *      - 0x00
               *      - 0x00
               *      - 0x00
               */
               /*~I*/
#ifdef def_EOL_oneTest
               /*~T*/
               wkpoint = ui8_b_DResB0_c;

               /*~-*/
#endif
               /*~E*/
               /*~C*/
               switch( ui8_b_DResB0_c )
               /*~-*/
               {
                  /*~A*/
                  /*~+:wkpoint 1*/
                  /*~F*/
                  case 1:
                  /*~-*/
                  {
                     /*~I*/
                     if( wkpoint == ui8_b_DResB0_c )
                     /*~-*/
                     {
                        /*~T*/
                        ui8_b_DResB0_RD = ui8_b_DResB0_c;
                        ui8_b_DResB1_RD = PWMReadDC( )>>1; /* PWM input DC */
                        ui8_b_DResB2_RD = ui16_speed_fil*5/4; /* Speed of the motor */
                        ui8_b_DResB3_RD = PWM_trans_table[ ui8_b_DResB1_RD ]; /* Speed request */
                        ui8_b_DResB4_RD = 0;
                        ui8_b_DResB5_RD = 0;

                        /*~I*/
                        if ((ui8_b_DResB2_RD < ui8_b_DResB3_RD + 3) && (ui8_b_DResB2_RD > ui8_b_DResB3_RD - 3))
                        /*~-*/
                        {
                           /*~T*/
                           Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD, ui8_b_DResB1_RD, ui8_b_DResB2_RD, ui8_b_DResB3_RD, ui8_b_DResB4_RD, ui8_b_DResB5_RD );

                        /*~-*/
                        }
                        /*~O*/
                        /*~-2*/
                        else
                        {
                           /*~T*/
                           Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, ui8_b_DResB0_c, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );

                           /*~I*/
#ifdef def_EOL_ShutDown
                           /*~T*/
                           write_eeprom_data(0x55,0x55); /** Write in EEPROM at address 0x55 a flag to say that the EOL are finished */
                           RC5 = 0; /* Disable LIN */
                           /*~-*/
#endif
                           /*~E*/
                        /*~-*/
                        }
                        /*~E*/
                        /*~I*/
#ifdef def_EOL_oneTest
                        /*~T*/

                        /*~O*/
                        /*~-*/
#else
                        /*~T*/
                        wkpoint = 2;

                        /*~-*/
#endif
                        /*~E*/
                     /*~-*/
                     }
                     /*~O*/
                     /*~-2*/
                     else
                     {
                        /*~T*/

                        /*~I*/
#ifdef transmit
                        /*~T*/
                        NegativeAnswer(ui8_b_DResLocID_c);

                        /*~-*/
#endif
                        /*~E*/
                     /*~-*/
                     }
                     /*~E*/
                     /*~T*/
                     break;
                  /*~-*/
                  }
                  /*~E*/
                  /*~E*/
                  /*~I*/
#ifdef speed_enable
                  /*~A*/
                  /*~+:wkpoint 2*/
                  /*~F*/
                  case 2:
                  /*~-*/
                  {
                     /*~I*/
                     if( wkpoint == ui8_b_DResB0_c )
                     /*~-*/
                     {
                        /*~T*/
                        ui8_b_DResB0_RD = ui8_b_DResB0_c;
                        ui8_b_DResB1_RD = PWMReadDC( )>>1; /* PWM input DC */
                        ui8_b_DResB2_RD = ui16_speed_fil*5/4; /* Speed of the motor */
                        ui8_b_DResB3_RD = PWM_trans_table[ ui8_b_DResB1_RD ]; /* Speed request */
                        ui8_b_DResB4_RD = 0;
                        ui8_b_DResB5_RD = 0;


                        /*~I*/
                        if ((ui8_b_DResB2_RD < ui8_b_DResB3_RD + 3) && (ui8_b_DResB2_RD > ui8_b_DResB3_RD - 3))
                        /*~-*/
                        {
                           /*~T*/
                           Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c,
                                                ui8_b_DResB0_RD, ui8_b_DResB1_RD,
                                                ui8_b_DResB2_RD, ui8_b_DResB3_RD,
                                                ui8_b_DResB4_RD,
                                                ui8_b_DResB5_RD );

                        /*~-*/
                        }
                        /*~O*/
                        /*~-2*/
                        else
                        {
                           /*~T*/
                           Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, ui8_b_DResB0_c, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );

                           /*~I*/
#ifdef def_EOL_ShutDown
                           /*~T*/
                           write_eeprom_data(0x55,0x55); /** Write in EEPROM at address 0x55 a flag to say that the EOL are finished */
                           RC5 = 0; /* Disable LIN */
                           /*~-*/
#endif
                           /*~E*/
                        /*~-*/
                        }
                        /*~E*/
                        /*~I*/
#ifdef def_EOL_oneTest
                        /*~T*/

                        /*~O*/
                        /*~-*/
#else
                        /*~T*/
                        wkpoint = 3;

                        /*~-*/
#endif
                        /*~E*/
                     /*~-*/
                     }
                     /*~O*/
                     /*~-2*/
                     else
                     {
                        /*~T*/


                        /*~I*/
#ifdef transmit
                        /*~T*/
                        NegativeAnswer(ui8_b_DResLocID_c);

                        /*~-*/
#endif
                        /*~E*/
                     /*~-*/
                     }
                     /*~E*/
                     /*~T*/
                     break;
                  /*~-*/
                  }
                  /*~E*/
                  /*~E*/
                  /*~-*/
#endif
                  /*~E*/
                  /*~A*/
                  /*~+:wkpoint 3*/
                  /*~F*/
                  case 3:
                  /*~-*/
                  {
                     /*~I*/
                     if( wkpoint == ui8_b_DResB0_c )
                     /*~-*/
                     {
                        /*~T*/
                        ui8_b_DResB0_RD = ui8_b_DResB0_c;
                        ui8_b_DResB1_RD = PWMReadDC( )>>1; /* PWM input DC */
                        ui8_b_DResB2_RD = ui16_speed_fil*5/4; /* Speed of the motor */
                        ui8_b_DResB3_RD = PWM_trans_table[ ui8_b_DResB1_RD ]; /* Speed request */
                        ui8_b_DResB4_RD = 0;
                        ui8_b_DResB5_RD = 0;


                        /*~I*/
                        if ((ui8_b_DResB2_RD < ui8_b_DResB3_RD + 3) && (ui8_b_DResB2_RD > ui8_b_DResB3_RD - 3))
                        /*~-*/
                        {
                           /*~T*/
                           Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c,
                                                ui8_b_DResB0_RD, ui8_b_DResB1_RD,
                                                ui8_b_DResB2_RD, ui8_b_DResB3_RD,
                                                ui8_b_DResB4_RD,
                                                ui8_b_DResB5_RD );

                        /*~-*/
                        }
                        /*~O*/
                        /*~-2*/
                        else
                        {
                           /*~T*/
                           Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, ui8_b_DResB0_c, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );

                           /*~I*/
#ifdef def_EOL_ShutDown
                           /*~T*/
                           write_eeprom_data(0x55,0x55); /** Write in EEPROM at address 0x55 a flag to say that the EOL are finished */
                           RC5 = 0; /* Disable LIN */
                           /*~-*/
#endif
                           /*~E*/
                        /*~-*/
                        }
                        /*~E*/
                        /*~I*/
#ifdef def_EOL_oneTest
                        /*~T*/

                        /*~O*/
                        /*~-*/
#else
                        /*~T*/
                        wkpoint = 4;
                        /*~-*/
#endif
                        /*~E*/
                        /*~T*/

                     /*~-*/
                     }
                     /*~O*/
                     /*~-2*/
                     else
                     {
                        /*~T*/

                        /*~I*/
#ifdef transmit
                        /*~T*/
                        NegativeAnswer(ui8_b_DResLocID_c);
                        /*~-*/
#endif
                        /*~E*/
                     /*~-*/
                     }
                     /*~E*/
                     /*~T*/
                     break;
                  /*~-*/
                  }
                  /*~E*/
                  /*~E*/
                  /*~A*/
                  /*~+:wkpoint 4*/
                  /*~F*/
                  case 4:
                  /*~-*/
                  {
                     /*~I*/
                     if( wkpoint == ui8_b_DResB0_c )
                     /*~-*/
                     {
                        /*~T*/
                        ui8_b_DResB0_RD = ui8_b_DResB0_c;
                        ui8_b_DResB1_RD = PWMReadDC( )>>1; /* PWM input DC */
                        ui8_b_DResB2_RD = ui16_speed_fil*5/4; /* Speed of the motor */
                        ui8_b_DResB3_RD = PWM_trans_table[ ui8_b_DResB1_RD ]; /* Speed request */
                        ui8_b_DResB4_RD = 0;
                        ui8_b_DResB5_RD = 0;


                        /*~I*/
                        if ((ui8_b_DResB2_RD < ui8_b_DResB3_RD + 3) && (ui8_b_DResB2_RD > ui8_b_DResB3_RD - 3))
                        /*~-*/
                        {
                           /*~T*/
                           Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c,
                                                ui8_b_DResB0_RD, ui8_b_DResB1_RD,
                                                ui8_b_DResB2_RD, ui8_b_DResB3_RD,
                                                ui8_b_DResB4_RD,
                                                ui8_b_DResB5_RD );

                        /*~-*/
                        }
                        /*~O*/
                        /*~-2*/
                        else
                        {
                           /*~T*/
                           Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, ui8_b_DResB0_c, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );

                           /*~I*/
#ifdef def_EOL_ShutDown
                           /*~T*/
                           write_eeprom_data(0x55,0x55); /** Write in EEPROM at address 0x55 a flag to say that the EOL are finished */
                           RC5 = 0; /* Disable LIN */
                           /*~-*/
#endif
                           /*~E*/
                        /*~-*/
                        }
                        /*~E*/
                        /*~I*/
#ifdef def_EOL_oneTest
                        /*~T*/

                        /*~O*/
                        /*~-*/
#else
                        /*~T*/
                        wkpoint = 5;
                        /*~-*/
#endif
                        /*~E*/
                     /*~-*/
                     }
                     /*~O*/
                     /*~-2*/
                     else
                     {
                        /*~T*/

                        /*~I*/
#ifdef transmit
                        /*~T*/
                        NegativeAnswer(ui8_b_DResLocID_c);
                        /*~-*/
#endif
                        /*~E*/
                     /*~-*/
                     }
                     /*~E*/
                     /*~T*/
                     break;
                  /*~-*/
                  }
                  /*~E*/
                  /*~E*/
                  /*~A*/
                  /*~+:wkpoint 5*/
                  /*~F*/
                  case 5:
                  /*~-*/
                  {
                     /*~I*/
                     if( wkpoint == ui8_b_DResB0_c )
                     /*~-*/
                     {
                        /*~T*/
                        ui8_b_DResB0_RD = ui8_b_DResB0_c;
                        ui8_b_DResB1_RD = PWMReadDC( )>>1; /* PWM input DC */
                        ui8_b_DResB2_RD = ui16_speed_fil*5/4; /* Speed of the motor */
                        ui8_b_DResB3_RD = PWM_trans_table[ ui8_b_DResB1_RD ]; /* Speed request */
                        ui8_b_DResB4_RD = 0;
                        ui8_b_DResB5_RD = 0;

                        /*~I*/
                        if ((ui8_b_DResB2_RD < ui8_b_DResB3_RD + 3) && (ui8_b_DResB2_RD > ui8_b_DResB3_RD - 3))
                        /*~-*/
                        {
                           /*~T*/
                           Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c,
                                                ui8_b_DResB0_RD, ui8_b_DResB1_RD,
                                                ui8_b_DResB2_RD, ui8_b_DResB3_RD,
                                                ui8_b_DResB4_RD,
                                                ui8_b_DResB5_RD );

                        /*~-*/
                        }
                        /*~O*/
                        /*~-2*/
                        else
                        {
                           /*~T*/
                           Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, ui8_b_DResB0_c, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );

                           /*~I*/
#ifdef def_EOL_ShutDown
                           /*~T*/
                           write_eeprom_data(0x55,0x55); /** Write in EEPROM at address 0x55 a flag to say that the EOL are finished */
                           RC5 = 0; /* Disable LIN */
                           /*~-*/
#endif
                           /*~E*/
                        /*~-*/
                        }
                        /*~E*/
                        /*~I*/
#ifdef def_EOL_oneTest
                        /*~T*/

                        /*~O*/
                        /*~-*/
#else
                        /*~T*/
                        wkpoint = 6;
                        /*~-*/
#endif
                        /*~E*/
                     /*~-*/
                     }
                     /*~O*/
                     /*~-2*/
                     else
                     {
                        /*~T*/

                        /*~I*/
#ifdef transmit
                        /*~T*/
                        NegativeAnswer(ui8_b_DResLocID_c);
                        /*~-*/
#endif
                        /*~E*/
                     /*~-*/
                     }
                     /*~E*/
                     /*~T*/
                     break;
                  /*~-*/
                  }
                  /*~E*/
                  /*~E*/
                  /*~I*/
#ifdef speed_enable
                  /*~A*/
                  /*~+:wkpoint 6*/
                  /*~F*/
                  case 6:
                  /*~-*/
                  {
                     /*~I*/
                     if( wkpoint == ui8_b_DResB0_c )
                     /*~-*/
                     {
                        /*~T*/
                        ui8_b_DResB0_RD = ui8_b_DResB0_c;
                        ui8_b_DResB1_RD = PWMReadDC( )>>1; /* PWM input DC */
                        ui8_b_DResB2_RD = ui16_speed_fil*5/4; /* Speed of the motor */
                        ui8_b_DResB3_RD = PWM_trans_table[ ui8_b_DResB1_RD ]; /* Speed request */
                        ui8_b_DResB4_RD = 0;
                        ui8_b_DResB5_RD = 0;

                        /*~I*/
                        if ((ui8_b_DResB2_RD < ui8_b_DResB3_RD + 3) && (ui8_b_DResB2_RD > ui8_b_DResB3_RD - 3))
                        /*~-*/
                        {
                           /*~T*/
                           Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c,
                                                ui8_b_DResB0_RD, ui8_b_DResB1_RD,
                                                ui8_b_DResB2_RD, ui8_b_DResB3_RD,
                                                ui8_b_DResB4_RD,
                                                ui8_b_DResB5_RD );

                        /*~-*/
                        }
                        /*~O*/
                        /*~-2*/
                        else
                        {
                           /*~T*/
                           Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, ui8_b_DResB0_c, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );

                           /*~I*/
#ifdef def_EOL_ShutDown
                           /*~T*/
                           write_eeprom_data(0x55,0x55); /** Write in EEPROM at address 0x55 a flag to say that the EOL are finished */
                           RC5 = 0; /* Disable LIN */
                           /*~-*/
#endif
                           /*~E*/
                        /*~-*/
                        }
                        /*~E*/
                        /*~I*/
#ifdef def_EOL_oneTest
                        /*~T*/

                        /*~O*/
                        /*~-*/
#else
                        /*~T*/
                        wkpoint = 7;
                        /*~-*/
#endif
                        /*~E*/
                     /*~-*/
                     }
                     /*~O*/
                     /*~-2*/
                     else
                     {
                        /*~T*/

                        /*~I*/
#ifdef transmit
                        /*~T*/
                        NegativeAnswer(ui8_b_DResLocID_c);
                        /*~-*/
#endif
                        /*~E*/
                     /*~-*/
                     }
                     /*~E*/
                     /*~T*/
                     break;
                  /*~-*/
                  }
                  /*~E*/
                  /*~E*/
                  /*~-*/
#endif
                  /*~E*/
                  /*~I*/
#ifdef speed_enable
                  /*~A*/
                  /*~+:wkpoint 7*/
                  /*~F*/
                  case 7:
                  /*~-*/
                  {
                     /*~I*/
                     if( wkpoint == ui8_b_DResB0_c )
                     /*~-*/
                     {
                        /*~T*/
                        ui8_b_DResB0_RD = ui8_b_DResB0_c;
                        ui8_b_DResB1_RD = PWMReadDC( )>>1; /* PWM input DC */
                        ui8_b_DResB2_RD = ui16_speed_fil*5/4; /* Speed of the motor */
                        ui8_b_DResB3_RD = PWM_trans_table[ ui8_b_DResB1_RD ]; /* Speed request */
                        ui8_b_DResB4_RD = 0;
                        ui8_b_DResB5_RD = 0;


                        /*~I*/
                        if ((ui8_b_DResB2_RD < ui8_b_DResB3_RD + 3) && (ui8_b_DResB2_RD > ui8_b_DResB3_RD - 3))
                        /*~-*/
                        {
                           /*~T*/
                           Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c,
                                                ui8_b_DResB0_RD, ui8_b_DResB1_RD,
                                                ui8_b_DResB2_RD, ui8_b_DResB3_RD,
                                                ui8_b_DResB4_RD,
                                                ui8_b_DResB5_RD );

                        /*~-*/
                        }
                        /*~O*/
                        /*~-2*/
                        else
                        {
                           /*~T*/
                           Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, ui8_b_DResB0_c, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );

                           /*~I*/
#ifdef def_EOL_ShutDown
                           /*~T*/
                           write_eeprom_data(0x55,0x55); /** Write in EEPROM at address 0x55 a flag to say that the EOL are finished */
                           RC5 = 0; /* Disable LIN */
                           /*~-*/
#endif
                           /*~E*/
                        /*~-*/
                        }
                        /*~E*/
                        /*~T*/
                        ui8_selected_lid = diagnostic_flags;

                     /*~-*/
                     }
                     /*~O*/
                     /*~-2*/
                     else
                     {
                        /*~T*/

                        /*~I*/
#ifdef transmit
                        /*~T*/
                        NegativeAnswer(ui8_b_DResLocID_c);
                        /*~-*/
#endif
                        /*~E*/
                     /*~-*/
                     }
                     /*~E*/
                     /*~T*/
                     break;
                  /*~-*/
                  }
                  /*~E*/
                  /*~E*/
                  /*~-*/
#endif
                  /*~E*/
                  /*~O*/
                  /*~-2*/
                  default:
                  {
                     /*~A*/
                     /*~+:negative answer*/
                     /*~T*/
                     NegativeAnswer(ui8_b_DResLocID_c);
                     break;
                     /*~E*/
                  /*~-*/
                  }
               /*~-*/
               }
               /*~E*/
            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~A*/
               /*~+:negative answer*/
               /*~T*/

               /*~I*/
#ifdef transmit
               /*~T*/
               NegativeAnswer(ui8_b_DResLocID_c);
               /*~-*/
#endif
               /*~E*/
               /*~E*/
            /*~-*/
            }
            /*~E*/
            /*~T*/
            break;
         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~A*/
         /*~+:case 0x9f: diagnostic_flags*/
         /*~F*/
         case ( diagnostic_flags ):
         /*~-*/
         {
            /*~I*/
            if( diagnostic_flags == ui8_selected_lid )
            /*~-*/
            {
               /*~T*/
               /** If <b>LID = 0x9f</b>, do nothing */
               /*~T*/
               ui8_failure = ui8_b_DResB0_c;

               /*~A*/
               /*~+:Diag with LIN*/
               /*~C*/
               switch (ui8_failure)
               /*~-*/
               {
                  /*~I*/
#ifdef flag_enable
                  /*~F*/
                  case 7:
                  /*~-*/
                  {
                     /*~A*/
                     /*~+:Power stage error (88% DC)*/
                     /*~T*/
                     PWM_Write_Out( 88 );
                     bool_PWMin_err_Flag       = 0;
                     bool_OverTemp_err_Flag    = 0;
                     bool_OverTemp_severe_Flag = 0;
                     bool_OverCurr_err_Flag    = 0;
                     bool_Pump_err_Flag        = 0;
                     bool_DryRunning_err_Flag  = 0;
                     bool_PowerStage_err_Flag  = 1;
                     bool_PWMin_Freq_err_Flag  = 0;
                     ui8_b_DResB0_RD = 2;
                     ui8_b_DResB1_RD = 88;
                     ui8_failure = 6;
                     break;
                     /*~E*/
                  /*~-*/
                  }
                  /*~E*/
                  /*~-*/
#endif
                  /*~E*/
                  /*~I*/
#ifdef flag_enable
                  /*~F*/
                  case 6:
                  /*~-*/
                  {
                     /*~A*/
                     /*~+:Over temperature flag (33% DC)*/
                     /*~T*/
                     PWM_Write_Out( 33 );
                     bool_PWMin_err_Flag       = 0;
                     bool_OverTemp_err_Flag    = 1;
                     bool_OverTemp_severe_Flag = 0;
                     bool_OverCurr_err_Flag    = 0;
                     bool_Pump_err_Flag        = 0;
                     bool_DryRunning_err_Flag  = 0;
                     bool_PowerStage_err_Flag  = 0;
                     bool_PWMin_Freq_err_Flag  = 0;
                     ui8_b_DResB0_RD = 64;
                     ui8_b_DResB1_RD = 33;
                     ui8_failure = 5;
                     break;
                     /*~E*/
                  /*~-*/
                  }
                  /*~E*/
                  /*~-*/
#endif
                  /*~E*/
                  /*~I*/
#ifdef flag_enable
                  /*~F*/
                  case 5:
                  /*~-*/
                  {
                     /*~A*/
                     /*~+:Dry running flag (66% DC)*/
                     /*~T*/
                     PWM_Write_Out( 66 );
                     bool_PWMin_err_Flag       = 0;
                     bool_OverTemp_err_Flag    = 0;
                     bool_OverTemp_severe_Flag = 0;
                     bool_OverCurr_err_Flag    = 0;
                     bool_Pump_err_Flag        = 0;
                     bool_DryRunning_err_Flag  = 1;
                     bool_PowerStage_err_Flag  = 0;
                     bool_PWMin_Freq_err_Flag  = 0;
                     ui8_b_DResB0_RD = 4;
                     ui8_b_DResB1_RD = 66;
                     ui8_failure = 4;
                     break;
                     /*~E*/
                  /*~-*/
                  }
                  /*~E*/
                  /*~-*/
#endif
                  /*~E*/
                  /*~F*/
                  case 4:
                  /*~-*/
                  {
                     /*~A*/
                     /*~+:Pump error flag (55% DC)*/
                     /*~T*/
                     PWM_Write_Out( 55 );
                     bool_PWMin_err_Flag       = 0;
                     bool_OverTemp_err_Flag    = 0;
                     bool_OverTemp_severe_Flag = 0;
                     bool_OverCurr_err_Flag    = 0;
                     bool_Pump_err_Flag        = 1;
                     bool_DryRunning_err_Flag  = 0;
                     bool_PowerStage_err_Flag  = 0;
                     bool_PWMin_Freq_err_Flag  = 0;
                     ui8_b_DResB1_RD = 55;
                     ui8_b_DResB0_RD = 8;
                     ui8_failure = 3;
                     break;
                     /*~E*/
                  /*~-*/
                  }
                  /*~E*/
                  /*~I*/
#ifdef flag_enable
                  /*~F*/
                  case 3:
                  /*~-*/
                  {
                     /*~A*/
                     /*~+:Overcurrent flag (44% DC)*/
                     /*~T*/
                     PWM_Write_Out( 44 );
                     bool_PWMin_err_Flag       = 0;
                     bool_OverTemp_err_Flag    = 0;
                     bool_OverTemp_severe_Flag = 0;
                     bool_OverCurr_err_Flag    = 1;
                     bool_Pump_err_Flag        = 0;
                     bool_DryRunning_err_Flag  = 0;
                     bool_PowerStage_err_Flag  = 0;
                     bool_PWMin_Freq_err_Flag  = 0;
                     ui8_b_DResB0_RD = 16;
                     ui8_b_DResB1_RD = 44;
                     ui8_failure = 2;
                     break;
                     /*~E*/
                  /*~-*/
                  }
                  /*~E*/
                  /*~-*/
#endif
                  /*~E*/
                  /*~F*/
                  case 2:
                  /*~-*/
                  {
                     /*~A*/
                     /*~+:PWM input frequency error (77% DC)*/
                     /*~T*/
                     PWM_Write_Out( 77 );
                     bool_PWMin_err_Flag       = 0;
                     bool_OverTemp_err_Flag    = 0;
                     bool_OverTemp_severe_Flag = 0;
                     bool_OverCurr_err_Flag    = 0;
                     bool_Pump_err_Flag        = 0;
                     bool_DryRunning_err_Flag  = 0;
                     bool_PowerStage_err_Flag  = 0;
                     bool_PWMin_Freq_err_Flag  = 1;

                     ui8_b_DResB1_RD = 77;
                     ui8_b_DResB0_RD= 1;
                     ui8_failure = 1;
                     break;
                     /*~E*/
                  /*~-*/
                  }
                  /*~E*/
                  /*~I*/
#ifdef flag_enable
                  /*~F*/
                  case 1:
                  /*~-*/
                  {
                     /*~A*/
                     /*~+:PWM input error (not in the range of DC (22%)*/
                     /*~T*/
                     PWM_Write_Out( 22 );
                     bool_PWMin_err_Flag       = 1;
                     bool_OverTemp_err_Flag    = 0;
                     bool_OverTemp_severe_Flag = 0;
                     bool_OverCurr_err_Flag    = 0;
                     bool_Pump_err_Flag        = 0;
                     bool_DryRunning_err_Flag  = 0;
                     bool_PowerStage_err_Flag  = 0;
                     bool_PWMin_Freq_err_Flag  = 0;
                     ui8_b_DResB1_RD = 22;
                     ui8_b_DResB0_RD = 128;
                     ui8_failure = 0;
                     break;
                     /*~E*/
                  /*~-*/
                  }
                  /*~E*/
                  /*~-*/
#endif
                  /*~E*/
                  /*~O*/
                  /*~-2*/
                  default:
                  {
                     /*~A*/
                     /*~+:no errror (11% DC)*/
                     /*~T*/
                     PWM_Write_Out( 11 );
                     bool_PWMin_err_Flag       = 0;
                     bool_OverTemp_err_Flag    = 0;
                     bool_OverTemp_severe_Flag = 0;
                     bool_OverCurr_err_Flag    = 0;
                     bool_Pump_err_Flag        = 0;
                     bool_DryRunning_err_Flag  = 0;
                     bool_PowerStage_err_Flag  = 0;
                     bool_PWMin_Freq_err_Flag  = 0;
                     break;
                     /*~E*/
                  /*~-*/
                  }
               /*~-*/
               }
               /*~E*/
               /*~E*/
               /*~T*/
               /** <b>LID = 0x9f</b>: Transmit to the diagnostics flags: */


               /*~T*/
               ui8_b_DResB2_RD = 0;
               ui8_b_DResB3_RD = 0;
               ui8_b_DResB4_RD = 0;
               ui8_b_DResB5_RD = 0;

               /*~T*/
               Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD,
                                    ui8_b_DResB1_RD, ui8_b_DResB2_RD,
                                    ui8_b_DResB3_RD, ui8_b_DResB4_RD,
                                    ui8_b_DResB5_RD );

            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~T*/

               /*~I*/
#ifdef transmit
               /*~T*/
               NegativeAnswer(ui8_b_DResLocID_c);
               /*~-*/
#endif
               /*~E*/
            /*~-*/
            }
            /*~E*/
            /*~T*/
            break;
         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~A*/
         /*~+:case 0xa0: fct_stamp_writing*/
         /*~F*/
         case ( fct_stamp_writing ):
         /*~-*/
         {
            /*~I*/
            if( fct_stamp_writing == ui8_selected_lid )
            /*~-*/
            {
               /*~T*/
               /** If <b>LID = 0xa0</b>, write FCT stamp into the EEPROM: 
               *      - Prod_Year_FCT in adress 0xf4
               *      - Prod_Day_FCT MSB in adress 0xf5
               *      - Prod_Day_FCT LSB in adress 0xf6
               *      - Prod_Num_FCT MSB in adress 0xf7
               *      - Prod_Num_FCT LSB in adress 0xf8
               */
               /*~T*/
               write_eeprom_data( 0xf4, ui8_b_DResB0_c ); /* Prod_Year_FCT */
               write_eeprom_data( 0xf5, ui8_b_DResB1_c ); /* Prod_Day_FCT MSB */
               write_eeprom_data( 0xf6, ui8_b_DResB2_c ); /* Prod_Day_FCT LSB */
               write_eeprom_data( 0xf7, ui8_b_DResB3_c ); /* Prod_Num_FCT MSB */
               write_eeprom_data( 0xf8, ui8_b_DResB4_c ); /* Prod_Num_FCT LSB */

               /*~T*/
               /** <b>LID = 0xa0</b>: Transmit to the tester the FCT stamp information: 
               *      - Prod_Year_FCT
               *      - Prod_Day_FCT MSB
               *      - Prod_Day_FCT LSB
               *      - Prod_Num_FCT MSB
               *      - Prod_Num_FCT LSB
               *      - 0x00
               */
               /*~T*/
               ui8_b_DResB0_RD = read_eeprom_data( 0xf4 ); /* Prod_Year_FCT */
               ui8_b_DResB1_RD = read_eeprom_data( 0xf5 ); /* Prod_Day_FCT MSB */
               ui8_b_DResB2_RD = read_eeprom_data( 0xf6 ); /* Prod_Day_FCT LSB */
               ui8_b_DResB3_RD = read_eeprom_data( 0xf7 ); /* Prod_Num_FCT MSB */
               ui8_b_DResB4_RD = read_eeprom_data( 0xf8 ); /* Prod_Num_FCT LSB */
               ui8_b_DResB5_RD = 0;

               /*~I*/
               if ((ui8_b_DResB0_RD != 0xFF) && (ui8_b_DResB1_RD != 0xFF) && (ui8_b_DResB2_RD != 0xFF) && (ui8_b_DResB3_RD != 0xFF) && (ui8_b_DResB4_RD != 0xFF))
               /*~-*/
               {
                  /*~T*/
                  Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD, ui8_b_DResB1_RD, ui8_b_DResB2_RD, ui8_b_DResB3_RD, ui8_b_DResB4_RD, ui8_b_DResB5_RD );

               /*~-*/
               }
               /*~O*/
               /*~-2*/
               else
               {
                  /*~T*/
                  Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF );
                  /*~I*/
#ifdef def_EOL_ShutDown
                  /*~T*/
                  write_eeprom_data(0x55,0x55); /** Write in EEPROM at address 0x55 a flag to say that the EOL are finished */
                  RC5 = 0; /* Disable LIN */
                  /*~-*/
#endif
                  /*~E*/
               /*~-*/
               }
               /*~E*/
               /*~T*/

               ui8_selected_lid = eol_finished;

            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~I*/
#ifdef transmit
               /*~T*/
               NegativeAnswer(ui8_b_DResLocID_c);
               /*~-*/
#endif
               /*~E*/
            /*~-*/
            }
            /*~E*/
            /*~T*/
            break;
         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~A*/
         /*~+:case 0xfa: eol_finished*/
         /*~F*/
         case ( eol_finished ):
         /*~-*/
         {
            /*~I*/
            if( eol_finished == ui8_selected_lid )
            /*~-*/
            {
               /*~T*/
               Transmit_LIN_8Bytes( 0x10, 0x3a, ui8_b_DResLocID_c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 );
               write_eeprom_data(0x55,0x55); /** Write in EEPROM at address 0x55 a flag to say that the EOL are finished */
               RC5 = 0; /* Disable LIN */
            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~I*/
#ifdef transmit
               /*~T*/
               NegativeAnswer(ui8_b_DResLocID_c);
               /*~-*/
#endif
               /*~E*/
            /*~-*/
            }
            /*~E*/
            /*~T*/
            break;
         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~O*/
         /*~-2*/
         default:
         {
            /*~A*/
            /*~+:negative answer*/
            /*~T*/
            /** In case of <b>other LID</b>, the data are not read by the program and are forced to 0xFF */
            /*~T*/
            NegativeAnswer(ui8_selected_lid);
            break;
            /*~E*/
         /*~-*/
         }
      /*~-*/
      }
      /*~E*/
   /*~-*/
   }
   /*~O*/
   /*~-2*/
   else
   {
      /*~A*/
      /*~+:negative answer*/
      /*~T*/
      /** ## If SID not recognized, transmit a negative response frame to the tester */
      /*~T*/
      NegativeAnswer(ui8_selected_lid);
      /*~E*/
   /*~-*/
   }
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void NegativeAnswer(unsigned char LID)*/
/*~F*/
void NegativeAnswer(unsigned char LID)
/*~-*/
{
   /*~T*/
   ui8_b_DResB0_c = 0xFF;
   ui8_b_DResB1_c = 0xFF;
   ui8_b_DResB2_c = 0xFF;
   ui8_b_DResB3_c = 0xFF;
   ui8_b_DResB4_c = 0xFF;
   ui8_b_DResB5_c = 0xFF;

   Transmit_LIN_8Bytes( 0x10, 0x7F, LID, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );

/*~-*/
}
/*~E*/
/*~E*/
