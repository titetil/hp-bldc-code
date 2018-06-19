/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/*! 

@file bldc.c
@brief BLDC module which drives the pump 

 FileName:       bldc.c                       <br>
 Project:        BVH2                         <br>
 Processor:      PIC16F1936                   <br>
 Compiler:       PICC HITEC 9.81              <br> 
 Company:        TI Automotive                <br>
                                              <br>
  Copyright (c)  TI Automotive     All rights reserved.

@author      J.Plantholt 
 
@version 0.01.02
@date 17.02.2012
@ingroup BLDC
*/
/*~T*/

/*--------------------------------------------------------------------------------------------------------

 Initials     Name                      Company
 --------     ---------------------     ------------------------------------
 JPL           Joachim Plantholt        TI Automotive   
 
 
 H I S T O R Y

 Date       Version  Author  Description
 yy-mm-dd   
 10-11-17   0.00.01  jpl     created
 11-10-28   0.01.00  jpl     fixed first version
 12-01-20   0.01.01  jpl     switch only on when Flag_RUN_MOTOR is set
 12-02-14   0.01.02  jpl     prepare to run pump backwards, controlled by a compiler switch     
  
-------------------------------------------------------------------------------------------------------- */

/*****************************************************************************/
#include     <htc.h>
#include     "config.h"
#include     "project.h"
#include     "bldc.h"
#include     "adc.h"
#include     "diag.h"
#include     "BVH2_Appl_Layer.h"


/*~T*/

unsigned  short       comm_time;
unsigned  short       ui16_speed_rar; /* not filtered speed */
unsigned  short       ui8_zero_cros_cnt;

unsigned         int  ui16_step_cnt;

unsigned         char ui8_StartupPWM;


unsigned  short       ui16_comm_time_max;
unsigned         char comm_state;

signed    short       phase_delay_counter;
signed    short       phase_delay_counter_debug;

unsigned  short       ui16_phase_advancement;
unsigned  short       ui16_phase_angle;

unsigned         char bemf_filter;

unsigned         char ui8_BlankingCount;
unsigned         char BlankingCountdbg;

volatile  _u_wb                 ui16_IPhase_bldc;

volatile  _u_wb                 ui16_IPhase1_bldc;
volatile  _u_wb                 ui16_IPhase2_bldc;
volatile  _u_wb                 ui16_IPhase3_bldc;

unsigned         char ui8_IPhase_sel;
unsigned         char ui8_UPhase_sel;
unsigned         char ui8_Ubemf_sel;

unsigned  short       ui16_UPhase_bldc;
volatile  _u_wb                 ui16_Ubemf_bldc;

unsigned  short       ui16_NTC_Temp_bldc;
unsigned  short       ui16_CPU_Temp_bldc;

volatile  _u_wb                 ui16_Ubat_bldc;
unsigned         char ui8_sampleState;

unsigned         char ui8_CompFlag;
unsigned         char CompFlag_prev; /* previous comp Flag  */

unsigned         int  B[ 8 ];
unsigned         char Bcnt;
unsigned  short       ui16_speed_fil;
unsigned         char ui8_duty_cycle_BLDC;
_u_bits               MotorFlags;


extern         unsigned  int  ui16_I_cal_Ph1;
extern         unsigned  int  ui16_I_cal_Ph2;
extern         unsigned  int  ui16_I_cal_Ph3;
extern         unsigned  int  ui16_Temp_cal;
extern         unsigned char ui8_current_cal[3];

extern         unsigned  int  ui16_Task_Cont500ms;


/*~T*/

const unsigned char cBEMF_FILTER[64]=
{       0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E, /* 000000,000010,000100,000110,001000,001010,001100,001110 */
        0x10,0x12,0x14,0x16,0x18,0x1A,0x1C,0x1E, /* 010000,010010,010100,010110,011000,011010,011100,011110 */
        0x20,0x22,0x24,0x26,0x28,0x2A,0x2C,0x2E, /* 100000,100010,100100,100110,101000,101010,101100,101110 */
        0x01,0x01,0x01,0x36,0x01,0x3A,0x3C,0x3E, /* 000001,000001,000001,110110,000001,111010,111100,111110 */
        0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E, /* 000000,000010,000100,000110,001000,001010,001100,001110 */
        0x01,0x01,0x01,0x16,0x01,0x1A,0x1C,0x1E, /* 000001,000001,000001,010110,000001,011010,011100,011110 */
        0x01,0x01,0x01,0x26,0x01,0x2A,0x2C,0x2E, /* 000001,000001,000001,100110,000001,111010,111100,111110 */
        0x01,0x01,0x01,0x36,0x01,0x3A,0x3C,0x3E  /* 000001,000001,000001,110110,000001,011010,011100,011110 */
};



               bit            rising_bemf_flag;


/*****************************************************************************/

void           commutate( void  );
void           init( void  );

void           InitMotorRun( void  );
void           InitMotorStop( void  );

/*~A*/
/*~+:void BLDCWait(void)*/
/*~F*/
/*-------------------------------------------------------------------------------------------------------*/
/* Function:        void BLDCWait(unsigned char ui8_Ch_number)                                           */
/* PreCondition:    None                                                                                 */
/* Input:           unsigned char                                                                        */
/* Output:          None                                                                                 */
/* Side Effects:    None                                                                                 */
/* Overview:        this function  delays the sampling point of bemf signal to ensure setup time 
                    and to avoid sampling of bemf signal on the edge                                     */
/* Note:            None                                                                                 */
/*-------------------------------------------------------------------------------------------------------*/

void BLDCWait( void  )
/*~-*/
{
   /*~T*/

    NOP( );          /* 8 NOP are app 1æs */
    NOP( );
    NOP( );
    NOP( );
    NOP( );
    NOP( );
    NOP( );
    NOP( );

    NOP( );
    NOP( );
    NOP( );
    NOP( );
    NOP( );
    NOP( );
    NOP( );
    NOP( );

    NOP( );
    NOP( );
    NOP( );
    NOP( );
    NOP( );
    NOP( );
    NOP( );
    NOP( );

    NOP( );
    NOP( );
    NOP( );
    NOP( );
    NOP( );
    NOP( );
    NOP( );
    NOP( );
                     /* measured Delay 4,5æs */

/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void init_bldc*/
/*~F*/
/* *********************************************************************************** */
/* *********************************************************************************** */
void init_bldc( void  )

/*~-*/
{
   /*~T*/
   /* weak pull-ups disabled,falling edge of INT,Prescaler assigned to WDT */
   /* TMR0 clk internal FOSC/4 */

   /* pin definition based on PIC16LF1936 MCPIM for MCLV33 developement board */

   /* only steering for ECCP1 is used for channeling low side switches */
   PSTR1CON          = 0;

   /* comparator2 used for BEMF zerocrossing detection */
   //CM2CON0 = 0x80;
   CM1CON0           = 0x80;
   //      CM2CON0 = 0x82; /* hysteresis enabled */
   //      CM2CON0 = 0x84; /* hispeed enabled */
   //      CM2CON0 = 0x86; /* hispeed & hysteresis enabled */

   //CM2CON1 = 0x00;
   CM1CON1           = 0x00;

   C1IE              = 0; /* disable interrupt */

   /* auto-shutdown: PWM restarts automatically */
   /* Deadband delay: 4/FOSC * 7 = 3.5µs ( half bridge only ) */

   PWM1CON           = 0x87;
   //PWM1CON = 0x9f;

   /*~T*/
   CCP1AS            = 0x800;
   CCP1CON           = 0x0C;

   /*~T*/
   /* Auto shut dowm configuration */
   /* P1A,B,C & D set "0" during shutdown */
   /* shutdown triggered from INTfalling edge */
   CCPR1L            = 0;

   CCP2IE            = 0; // first disable 
   CCP2CON           = 0x04;
   CCPR2L            = 0xff;
   CCPR2H            = 0xff;

   PR2               = DEF_PWM_PERIOD_VALUE;

   T2CON             = 0x05;
   //T2CON = 0x0c ;
   //T2CON = 0x06;

   /* setup blanking counter */
   ui8_BlankingCount = def_BLANKINGCOUNT;

   /* PWM period interrupt used for aquidistant comparator scan */

   TMR2IE            = 1;
   //CCP2IE  = 1 ;    // enable !!!!!  

   MotorFlags.b      = 0; /* Clearing all motor flags */

   /*~T*/
   InitMotorRun( );
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void interrrupt_bldc*/
/*~T*/
/* It must be avoided that the comparator samples during the rising or falling edge of the PWM signal. 
Timing is critical */
/*~F*/
void interrrupt_bldc( void  )

/*~-*/
{
   /*~T*/
   unsigned  long ui32_tmp ; 
   /*~T*/
    ++comm_time ;

   /*~I*/
   if( comm_time > ui16_comm_time_max )

   /*~-*/
   {
      /*~T*/
      commutate( ); /* timeout occurs force commutation */
   /*~-*/
   }
   /*~E*/
   /*~I*/
   /* if the number of startsteps are finished, the Flag_STARTUP is set */
   if( ui16_step_cnt < def_nb_steps_startramp )
   /*~-*/
   {
      /*~T*/

   /*~-*/
   }
   /*~O*/
   /*~-2*/
   else
   {
      /*~I*/
      //#define def_forced_comm_mode 
#ifdef def_forced_comm_mode /* if defined the transition to auto mode is ignored */ 
      /*~T*/

      /*~O*/
      /*~-*/
#else
      /*~T*/
      Flag_STARTUP = 0;          /* Startup finished */

      /*~-*/
#endif
      /*~E*/
   /*~-*/
   }
   /*~E*/
   /*~I*/
   if (ui16_step_cnt >= def_nb_align_startramp )
   /*~-*/
   {
      /*~T*/
      Flag_MotorAlign = 0 ; /* end the alignment phase */
   /*~-*/
   }
   /*~E*/
   /*~T*/
   BLDCWait( ); /* to ensure stable bemf signal and dont sample on the edge of the PWM signal */ 
   /*~A*/
   /*~+:sample comparator */
   /*~I*/
   /* The comparator doesn't use an interrupt. Only the Comparator Flag is used. 
      Watch out, the polarity of the comparator is reverse, i.e. a zero crossing is represented by C1OUT = 0 */
    if( C1OUT )

   /*~-*/
   {
      /*~T*/
        ui8_CompFlag = 0x00;
   /*~-*/
   }
   /*~O*/
   /*~-2*/
   else
   {
      /*~T*/
      //def_DebugPort0 = 1 ;
      /*~T*/
      /* zero crossing detected */
        ui8_CompFlag = 0x01;


   /*~-*/
   }
   /*~E*/
   /*~E*/
   /*~A*/
   /*~+:Sample Analog Values */
   /*~C*/
    switch( ui8_sampleState )
   /*~-*/
   {
      /*~F*/
        case 1:
      /*~-*/
      {
         /*~A*/
         /*~+:sample_NTC_Tempor select Ubat*/
         /*~K*/
         /*~+:The first ADC can not be included in the scheme without any risk. If the comm_timer doesn't reach the last ADC state, the channel is not prepared and a wrong data is read. */
         /*~T*/
         mADC_ChanSelect( NTC_Temper_Channel ); /* Channel is selected for the next input */

         /*~T*/
         ADC_Wait( );

         /*~T*/
         //def_DebugPort0 = 1 ; 
         mADC_TrigConversion( );
         //def_DebugPort0 = 0 ;

         /*~T*/
         def_DebugPort0=1;
         ui16_NTC_Temp_bldc = ADC_Read( ); /* Read the previous selected Channel */
         def_DebugPort0=0;

         /*~T*/
         mADC_ChanSelect( Batte_Volt_Channel ); /* Channel is selected for the next input */

         /*~T*/
         ADC_Wait( );

         /*~T*/
         //def_DebugPort0 = 1 ;
         mADC_TrigConversion( );
         //def_DebugPort0 = 0 ;   
         /*~T*/
         ui8_sampleState++;

         /*~T*/
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
         /*~+:sample_Ubat*/
         /*~T*/
         //def_DebugPort0 = 1 ;
         /*~T*/
         mADC_ChanSelect( ui8_IPhase_sel ); /* It is no error to select the channel before read the current Channel result */

         /*~T*/
         //def_DebugPort0 = 0 ;
         /*~I*/
#ifdef def_patchUbatSens
         /*~T*/
         /* patch the voltage to 13V  */
         ui16_Ubat_bldc.b.lo = 0xbb; /* Get the 8 bits LSB result */
         ui16_Ubat_bldc.b.hi = 0x01; /* Get the 2 bits MSB result */

         /*~O*/
         /*~-*/
#else
         /*~T*/
         /*! Read the AD converter without check the DONE Bit */
         def_DebugPort0=1;
         ui16_Ubat_bldc.b.lo = ADRESL; /* Get the 8 bits LSB result */
         ui16_Ubat_bldc.b.hi = ADRESH; /* Get the 2 bits MSB result */
         def_DebugPort0=0;

         /*~-*/
#endif
         /*~E*/
         /*~A*/
         /*~+:Wait for 2us*/
         /*~T*/
         NOP( );
         NOP( );
         NOP( );
         NOP( );
         NOP( );
         NOP( );
         NOP( );
         NOP( );

         NOP( );
         NOP( );
         NOP( );
         NOP( );
         NOP( );
         NOP( );
         NOP( );
         NOP( );

         /*~E*/
         /*~T*/
         ADC_Wait();
         /*~T*/
         //def_DebugPort0 = 1 ;
         mADC_TrigConversion( );
         //def_DebugPort0 = 0 ;
         /*~T*/
         ui8_sampleState++;

         /*~T*/
         break;
         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~F*/
        case 3:
      /*~-*/
      {
         /*~A*/
         /*~+:sample_Ubemf*/
         /*~K*/
         /*~+:The statemachine stays in this state untill it detects end of demag. */
         /*~+:with leaving the current state the next state clears the blanking counter and will finish the blanking period finished premature */
         /*~+:*/
         /*~T*/
         //def_DebugPort0 = 1 ;
         /*~T*/
         /*! Read the AD converter without check the DONE Bit */
         //ui16_Ubemf_bldc = ADC_Read() ;  /* Due to a timing issue the data is read without checking the status of the DONE Bit */
         def_DebugPort0=1;

         ui16_Ubemf_bldc.b.lo = ADRESL; /* Get the 8 bits LSB result */
         ui16_Ubemf_bldc.b.hi = ADRESH; /* Get the 2 bits MSB result */
         def_DebugPort0=0;

         /*~T*/
         //def_DebugPort0 = 1 ;
         mADC_TrigConversion( );
         //def_DebugPort0 = 0 ; /* retrigger conversion without knowing that demag is terminated and the next channel will be used */

         /*~I*/
#ifdef def_Auto_Demag
         /*~A*/
         /*~+:increment State if end of Demag is detected */
         /*~I*/
         if( ( Flag_RUN_MOTOR ) && ( ui8_duty_cycle_BLDC != 0 ) )
         /*~-*/
         {
            /*~I*/
            if( rising_bemf_flag )
            /*~-*/
            {
               /*~I*/
               if( ( ui16_Ubemf_bldc.w + 100 ) < ( ui16_Ubat_bldc.w )) /* 100 == 3V external */

               /*~-*/
               {
                  /*~T*/
                  /* setDemagEnd */
                  /*~T*/
                  ui8_sampleState++;

                  /*~T*/
                  /* selected next channel */
                  mADC_ChanSelect( ui8_UPhase_sel );
                  /*~T*/
                  ADC_Wait( );

                  /*~T*/
                  //def_DebugPort0 = 1 ;
                  mADC_TrigConversion( );
                  //def_DebugPort0 = 0 ;
               /*~-*/
               }
               /*~O*/
               /*~-2*/
               else
               {
                  /*~T*/
                  /*! Stay in this state select same channel adc is allready retriggered   */

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
               if( ( ui16_Ubemf_bldc.w ) > ( 30 ) )
               /*~-*/
               {
                  /*~T*/
                  /* setDemagEnd */
                  /*~T*/
                  ui8_sampleState++;

                  /*~T*/
                  /* selected next channel */
                  mADC_ChanSelect( ui8_UPhase_sel );

                  /*~T*/
                  ADC_Wait( );

                  /*~T*/
                  //def_DebugPort0 = 1 ;
                  mADC_TrigConversion( );
                  //def_DebugPort0 = 0 ;
               /*~-*/
               }
               /*~O*/
               /*~-2*/
               else
               {
                  /*~T*/
                  /*! Stay in this state select same channel adc is allready retriggered   */
               /*~-*/
               }
               /*~E*/
            /*~-*/
            }
            /*~E*/
            /*~T*/

         /*~-*/
         }
         /*~O*/
         /*~-2*/
         else
         {
            /*~T*/
            ui8_sampleState++;

            /*~T*/
            /* selected next channel */
            mADC_ChanSelect( ui8_UPhase_sel );


            /*~T*/
            ADC_Wait( );

            /*~T*/
            //def_DebugPort0 = 1 ;
            mADC_TrigConversion( );
            //def_DebugPort0 = 0 ;
            /*~T*/

         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~O*/
         /*~-*/
#else
         /*~A*/
         /*~+:allways step to next state*/
         /*~+:the end of demag is only controlled by the Blanking counter and will not cleared in advanceded*/
         /*~T*/
         ui8_sampleState++;

         /*~T*/
         mADC_ChanSelect( ui8_UPhase_sel );     /* Channel is selected for the next input */

         /*~T*/
         ADC_Wait( );

         /*~T*/
         mADC_TrigConversion( );

         /*~E*/
         /*~-*/
#endif
         /*~E*/
         /*~T*/
         break;
         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~F*/
        case 4:
      /*~-*/
      {
         /*~A*/
         /*~+:sample_U Phase */
         /*~I*/
#ifdef def_Auto_Demag
         /*~T*/
         BlankingCountdbg  = ui8_BlankingCount;
         ui8_BlankingCount = 0;            /* break blanking period The blanking counter is cleared on state after detecting end of demag  */ 
         /*~-*/
#endif
         /*~E*/
         /*~T*/
         def_DebugPort0=1;
         ui16_UPhase_bldc = ADC_Read( );
         def_DebugPort0=0;

         /*~T*/
         mADC_ChanSelect( CPU_Temp_Channel ); /* After sampling the ADC the next Channel is selected */
                                             /* for the internal Temperature Sensor for realizing a 200æs setup time */
         /*~T*/
         ADC_Wait( );

         /*~I*/
         if( bool_OverTemp_err_Flag == 1)
         /*~-*/
         {
            /*~T*/
            ui8_sampleState = 9;
         /*~-*/
         }
         /*~O*/
         /*~-2*/
         else
         {
            /*~T*/
            ui8_sampleState++;

         /*~-*/
         }
         /*~E*/
         /*~T*/
         break;
         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~F*/
        case 5:
      /*~-*/
      {
         /*~A*/
         /*~+:wait setup */
         /*~T*/
         ui8_sampleState++;

         /*~T*/
         break;
         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~F*/
        case 6:
      /*~-*/
      {
         /*~A*/
         /*~+:wait setup */
         /*~T*/
         ui8_sampleState++;

         /*~T*/
         break;
         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~F*/
        case 7:
      /*~-*/
      {
         /*~A*/
         /*~+:wait setup */
         /*~T*/
         ui8_sampleState++;

         /*~T*/
         break;
         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~F*/
        case 8:
      /*~-*/
      {
         /*~A*/
         /*~+:sample_CPU_Temp*/
         /*~T*/
         //def_DebugPort0 = 1 ;
         mADC_TrigConversion( );
         //def_DebugPort0 = 0 ;

         /*~T*/
         def_DebugPort0=1;
         ui16_CPU_Temp_bldc = ADC_Read( ); /* Read without selecting the channel */
         def_DebugPort0=0;

         /*~T*/
         ui8_sampleState++;

         /*~T*/
         break;
         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~F*/
        case 9:
      /*~-*/
      {
         /*~A*/
         /*~+:sample_I Phase */
         /*~T*/
         //ui16_IPhase_bldc.b.lo = ADRESL; /* Get the 8 bits LSB result */
         //ui16_IPhase_bldc.b.hi = ADRESH; /* Get the 2 bits MSB result */
         def_DebugPort0=1;
         ui16_IPhase_bldc.w = ADC_Read( );
         def_DebugPort0=0;

         /*~I*/
         if (Flag_RUN_MOTOR)
         /*~-*/
         {
            /*~C*/
            switch( ui8_IPhase_sel )
            /*~-*/
            {
               /*~F*/
               case Ph1_ISen_Channel:
               /*~-*/
               {
                  /*~I*/
                  if (ui16_IPhase_bldc.w > ( (unsigned  int) ui8_current_cal[0]) )
                  /*~-*/
                  {
                     /*~T*/
                     ui16_IPhase1_bldc.w = (((ui16_IPhase_bldc.w - ui8_current_cal[0])*ui16_I_cal_Ph1)>>8);
                  /*~-*/
                  }
                  /*~O*/
                  /*~-2*/
                  else
                  {
                     /*~T*/
                     ui16_IPhase1_bldc.w = 0 ; 
                  /*~-*/
                  }
                  /*~E*/
                  /*~T*/
                  break;
               /*~-*/
               }
               /*~E*/
               /*~F*/
               case Ph2_ISen_Channel:
               /*~-*/
               {
                  /*~I*/
                  if (ui16_IPhase_bldc.w >  ( (unsigned  int) ui8_current_cal[1] ) )
                  /*~-*/
                  {
                     /*~T*/
                     ui16_IPhase2_bldc.w = (((ui16_IPhase_bldc.w - ui8_current_cal[1])*ui16_I_cal_Ph2)>>8);

                  /*~-*/
                  }
                  /*~O*/
                  /*~-2*/
                  else
                  {
                     /*~T*/
                     ui16_IPhase2_bldc.w = 0 ;
                  /*~-*/
                  }
                  /*~E*/
                  /*~T*/
                  break;
               /*~-*/
               }
               /*~E*/
               /*~F*/
               case Ph3_ISen_Channel:
               /*~-*/
               {
                  /*~I*/
                  if (ui16_IPhase_bldc.w > ((unsigned  int) ui8_current_cal[2]) )
                  /*~-*/
                  {
                     /*~T*/
                     ui16_IPhase3_bldc.w = (((ui16_IPhase_bldc.w - ui8_current_cal[2])*ui16_I_cal_Ph3)>>8);
                  /*~-*/
                  }
                  /*~O*/
                  /*~-2*/
                  else
                  {
                     /*~T*/
                     ui16_IPhase3_bldc.w = 0 ;
                  /*~-*/
                  }
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
                  ui16_IPhase1_bldc.w = 0;
                  ui16_IPhase2_bldc.w = 0;
                  ui16_IPhase3_bldc.w = 0;

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
            ui16_IPhase1_bldc.w = 0;
            ui16_IPhase2_bldc.w = 0;
            ui16_IPhase3_bldc.w = 0;

         /*~-*/
         }
         /*~E*/
         /*~I*/
         //#define sf
#ifdef sf
         /*~T*/
         /* Short circuit on MOSFET detection */
         /*~I*/
         if( (ui16_IPhase1_bldc.w > def_Current_Limit) || (ui16_IPhase2_bldc.w > def_Current_Limit) || (ui16_IPhase3_bldc.w > def_Current_Limit) )
         /*~-*/
         {
            /*~T*/
            OverCurrentFlag = 1;
            /*~I*/
#ifdef def_EnableCurrentLimit
            /*~T*/
            PhaseX_DIS;

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
            //OverCurrentFlag = 0 ;
         /*~-*/
         }
         /*~E*/
         /*~O*/
         /*~-*/
#else
         /*~T*/
         /* Short circuit on MOSFET detection */
         /*~I*/
         if( (ui16_IPhase_bldc.w > def_Current_Limit+40) )
         /*~-*/
         {
            /*~T*/
            OverCurrentFlag = 1;
            /*~I*/
#ifdef def_EnableCurrentLimit
            /*~T*/
            PhaseX_DIS;

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
            //OverCurrentFlag = 0 ;
         /*~-*/
         }
         /*~E*/
         /*~-*/
#endif
         /*~E*/
         /*~T*/
         mADC_ChanSelect( ui8_Ubemf_sel ); /* Channel is selected for the next input */

         /*~T*/
         ADC_Wait( );

         /*~T*/
         //def_DebugPort0 = 1 ;
         mADC_TrigConversion( );
         //def_DebugPort0 = 0 ;
         /*~I*/
         if ( bool_OverTemp_err_Flag == 1)
         /*~-*/
         {
            /*~T*/
            ui8_sampleState=5;

         /*~-*/
         }
         /*~O*/
         /*~-2*/
         else
         {
            /*~T*/
            ui8_sampleState++;

         /*~-*/
         }
         /*~E*/
         /*~T*/
         break;
         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~F*/
        case 10:
      /*~-*/
      {
         /*~T*/
                break;
      /*~-*/
      }
      /*~E*/
   /*~-*/
   }
   /*~E*/
   /*~E*/
   /*~I*/
#ifdef def_MayorityFilter
   /*~A*/
   /*~+:Mayority Filter used */
   /*~I*/
   /* The comparator doesn't use an interrupt. Only the Comparator Flag is used. 
      Watch out, the polarity of the comparator is reverse, i.e. a zero crossing is represented by C1OUT = 0 */
    if( ui8_CompFlag )

   /*~-*/
   {
      /*~T*/

   /*~-*/
   }
   /*~O*/
   /*~-2*/
   else
   {
      /*~T*/
      /* represents no zero crossing occured */
        bemf_filter |= 0b00000001;

   /*~-*/
   }
   /*~E*/
   /*~T*/
    bemf_filter = cBEMF_FILTER[ bemf_filter ];

   /*~E*/
   /*~-*/
#endif
   /*~E*/
   /*~A*/
   /*~+:smooth update of PWM during the startup phase */
   /*~I*/
   if( Flag_RUN_MOTOR )
   /*~-*/
   {
      /*~I*/
      if( Flag_STARTUP) /* During Startupramp  */ 
      /*~-*/
      {
         /*~T*/
         bool_start_demand_mat     = 1  ;
         ui8_fixed_start_speed_mat = 65 ; /* 10000rpm */
         ui16_Task_Cont500ms       = 0  ;

         /*~T*/
         /* The PWM is updated every 4. interrupt or 200us notice at 600 rpm there are 
            The maximum PWM during start is e.g. defined as 0x40 or 160 interrupts
            The PWM is ramped during the first 160 interrupt from 0 to 40 
            1200 rpm 83 ticks  
            2400 rpm 42 ticks 
            3000 rpm 30 ticks   
         */
         /*~I*/
         if( ( comm_time % 4 ) == 0) /* Modulo function */ 
         /*~-*/
         {
            /*~I*/
            if (Flag_MotorAlign)
            /*~-*/
            {
               /*~I*/
               if (ui16_Ubat_bldc.w)
               /*~-*/
               {
                  /*~T*/
                  /* */
                  ui32_tmp = ( def_DEFAULT_UBAT * def_Align_PWM ) / ui16_Ubat_bldc.w ;
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
               if (ui16_Ubat_bldc.w)
               /*~-*/
               {
                  /*~T*/
                  /* */
                  ui32_tmp = ( def_DEFAULT_UBAT * def_STARTUP_PWM ) / ui16_Ubat_bldc.w ;
               /*~-*/
               }
               /*~E*/
            /*~-*/
            }
            /*~E*/
            /*~I*/
            if( ui8_StartupPWM > ( (unsigned char) ui32_tmp)  )
            /*~-*/
            {
               /*~T*/
               ui8_StartupPWM--;
            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~T*/
               ui8_StartupPWM++;
            /*~-*/
            }
            /*~E*/
            /*~T*/
            CCPR1L = ui8_StartupPWM ; 
         /*~-*/
         }
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
      /*~T*/
      /* PWM is zero when Motor is commanded to zero */
      CCPR1L = 0;
   /*~-*/
   }
   /*~E*/
   /*~E*/
   /*~I*/
    if( 0 == ui8_BlankingCount )

   /*~-*/
   {
      /*~I*/
#ifndef def_MayorityFilter
      /*~A*/
      /*~+:Mayority Filter not used  set variable for zerocrossing when comparator hits*/
      /*~I*/
      /* The comparator doesn't use an interrupt. Only the Comparator Flag is used. 
         Watch out, the polarity of the comparator is reverse, i.e. a zero crossing is represented by C1OUT = 0 */

      if( ui8_CompFlag )

      /*~-*/
      {
         /*~T*/
         /* zero crossing detected */

         /*~I*/
         if( ZC_DETECTED == 0 )
         /*~-*/
         {
            /*~T*/
            //def_DebugPort0 = 1 ; 
            /*~T*/
            ui8_zero_cros_cnt++;
            /*~T*/
            ZC_DETECTED = 1;   /* transition to detect terocrossing bypass the Mayority filter */

         /*~-*/
         }
         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~T*/
      CompFlag_prev = ui8_CompFlag;
      /*~E*/
      /*~-*/
#endif
      /*~E*/
      /*~T*/
      /*! if software is configured to open one MOSFET during the demagnetization phase. After the Blanking period is finished the normal pattern must be applied to the MOSFET again. this is done here. This is identical for both method (PWM on Low side or PWM on high side) */

      /*~I*/
#ifdef def_re_initState
      /*~A*/
      /*~+:def_re_initState*/
      /*~T*/
      /* The current statevariable is inclremented just after assigning the ports for this state */
      /*~C*/
        switch( comm_state )
      /*~-*/
      {
         /*~F*/
            case 1:

         /*~-*/
         {
            /*~K*/
            /*~+:State  Low   High  Comparator*/
            /*~+:  1     B     A       -C*/
            /*~+:*/
            /*~T*/
                    State6_Ena;
                    break;

         /*~-*/
         }
         /*~E*/
         /*~F*/
            case 2:

         /*~-*/
         {
            /*~K*/
            /*~+:State  Low   High  Comparator*/
            /*~+:  2     C     A        B*/
            /*~T*/
                    State1_Ena;
                    break;

         /*~-*/
         }
         /*~E*/
         /*~F*/
            case 3:

         /*~-*/
         {
            /*~K*/
            /*~+:State  Low   High  Comparator*/
            /*~+:  3     C     B       -A*/
            /*~T*/
                    State2_Ena;
                    break;

         /*~-*/
         }
         /*~E*/
         /*~F*/
            case 4:

         /*~-*/
         {
            /*~K*/
            /*~+:State  Low   High  Comparator*/
            /*~+:  4     A     B        C*/
            /*~+:*/
            /*~T*/
                    State3_Ena;
                    break;

         /*~-*/
         }
         /*~E*/
         /*~F*/
            case 5:

         /*~-*/
         {
            /*~K*/
            /*~+:State  Low   High  Comparator*/
            /*~+: 5     A     C       -B*/
            /*~+:*/
            /*~T*/
                    State4_Ena;
                    break;

         /*~-*/
         }
         /*~E*/
         /*~F*/
            case 6:

         /*~-*/
         {
            /*~K*/
            /*~+:State  Low   High  Comparator*/
            /*~+: 6     B     C        A*/
            /*~T*/
                    State5_Ena;
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
      /*~E*/
      /*~-*/
#endif
      /*~E*/
      /*~I*/
#ifdef def_MayorityFilter
      /*~A*/
      /*~+:check output of filter and set variable for zerocrossing*/
      /*~I*/
        if( bemf_filter & 0x01 )

      /*~-*/
      {
         /*~T*/
         /* zero crossing detected */
            ZC_DETECTED = 1;

      /*~-*/
      }
      /*~E*/
      /*~E*/
      /*~-*/
#endif
      /*~E*/
      /*~I*/
        if( ZC_DETECTED )

      /*~-*/
      {
         /*~A*/
         /*~+:commutate if phasedelay finishes and Flag_STARTUP is cleared  */
         /*~I*/
            if( 0 == phase_delay_counter )

         /*~-*/
         {
            /*~I*/
                if( Flag_STARTUP == 0) /* Transisiton to auto mode after startup */ 
            /*~-*/
            {
               /*~I*/
#ifdef def_SW_PSA_YBS
               /*~T*/
               /* only for yellow board 
               samples no zero crossing detection */

               /*~O*/
               /*~-*/
#else
               /*~T*/
               //def_DebugPort0 = 1 ;
               //def_DebugPort0 = 0 ; 
               /*~T*/
                    commutate( );
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
            /*~T*/
                phase_delay_counter--;
         /*~-*/
         }
         /*~E*/
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
      /*~T*/
        ui8_BlankingCount--;
        bemf_filter = 62; /* preload the comparator  */
      //bemf_filter = 0 ;/* preload the comparator  */


      /*~T*/



   /*~-*/
   }
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void commutate*/
/*~F*/
void commutate( void  )

/*~-*/
{
   /*~I*/
   if( ui16_step_cnt < def_nb_steps_startramp )
   /*~-*/
   {
      /*~T*/
      ui16_step_cnt++;
   /*~-*/
   }
   /*~E*/
   /*~T*/
   B[ Bcnt++ ]      = ( comm_time ); /*ActT1value.w; */
   Bcnt            &= ( ( sizeof( B ) / sizeof( B[ 0 ] ) ) - 1 );
   ui16_speed_fil = ( B[ 0 ] + B[ 1 ] + B[ 2 ] + B[ 3 ] + B[ 4 ] + B[ 5 ] + B[ 6 ] + B[ 7 ] );
   ui16_speed_rar   = comm_time;
   /*~T*/
   //ui16_speed_rar = (  ui16_speed_rar + comm_time)>>1  ;          /* simple filtered speed value */

   /*~T*/
   ZC_DETECTED      = 0;

   /*~T*/
   ui8_sampleState  = 1;

   /*~T*/
   //bemf_filter = 0;
   bemf_filter      = 62; /* preload the comparator mayority Filter */

   /*~T*/
   COMM_DONE        = 1;  /* function commutate() */

   /*~I*/
   if( Flag_STARTUP )
   /*~-*/
   {
      /*~A*/
      /*~+:Phase Advancement during start  */
      /*~T*/
      //ui16_phase_advancement = (comm_time * ui16_phase_angle) / 100 ;
        ui16_phase_advancement = ( ( ui16_speed_rar>>3 ) * def_PhaseAdvanceStart )>>7;
      //ui16_phase_advancement = 0 ;

      /*~I*/
        if( comm_time > ui16_phase_advancement )
      /*~-*/
      {
         /*~T*/
            phase_delay_counter = ( ( comm_time ) - ui16_phase_advancement );
      /*~-*/
      }
      /*~O*/
      /*~-2*/
      else
      {
         /*~T*/
            phase_delay_counter = 0;
      /*~-*/
      }
      /*~E*/
      /*~T*/
        phase_delay_counter_debug = phase_delay_counter;/* the phase_delay_counter is decremented in the interrupt service*/
                                                        /* phase_delay_counter_debug is not decremented and can be used for debug purposes */
      /*~E*/
      /*~A*/
      /*~+:comm_time max handling during start of pump */
      /*~T*/
      /* during the first steps of the motor the timeout is reduced by the equation x = x / 2 
         e.g. 
        def_ramp_start_SPEED 600rpm  --> 166 digit 
        def_ramp_end_SPEED   3000rpm -->  33 digit
      
        1.step 166   600 rpm actually not used because the init routine calls commutate() which  
        2.step  83  4,4ms 1200 rpm 
        3.step  41  2,2ms 2400 rpm 
        4.step  33  1,65ms ( end limitation to 3000 rpm until automode )  
             
      
      */
      /*~C*/
      switch (ui16_step_cnt)
      /*~-*/
      {
         /*~F*/
         case 0 :
         /*~-*/
         {
            /*~T*/
            ui16_comm_time_max = def_ramp0_start ;
            break;
         /*~-*/
         }
         /*~E*/
         /*~F*/
         case 1:
         /*~-*/
         {
            /*~T*/
            ui16_comm_time_max = def_ramp1_start ;
            break;
         /*~-*/
         }
         /*~E*/
         /*~F*/
         case 2:
         /*~-*/
         {
            /*~T*/
            ui16_comm_time_max = def_ramp2_start ;
            break;
         /*~-*/
         }
         /*~E*/
         /*~F*/
         case 3:
         /*~-*/
         {
            /*~T*/
            ui16_comm_time_max = def_ramp3_start ;
            break;
         /*~-*/
         }
         /*~E*/
         /*~F*/
         case 4:
         /*~-*/
         {
            /*~T*/
            ui16_comm_time_max = def_ramp4_start ;
            break;
         /*~-*/
         }
         /*~E*/
         /*~F*/
         case 5:
         /*~-*/
         {
            /*~T*/
            ui16_comm_time_max = def_ramp5_start ;
            break;
         /*~-*/
         }
         /*~E*/
         /*~F*/
         case 6:
         /*~-*/
         {
            /*~T*/
            ui16_comm_time_max = def_ramp6_start ;
            break;
         /*~-*/
         }
         /*~E*/
         /*~F*/
         case 7:
         /*~-*/
         {
            /*~T*/
            ui16_comm_time_max = def_ramp7_start ;
            break;
         /*~-*/
         }
         /*~E*/
         /*~F*/
         case 8:
         /*~-*/
         {
            /*~T*/
            ui16_comm_time_max = def_ramp8_start ;
            break;
         /*~-*/
         }
         /*~E*/
         /*~F*/
         case 9:
         /*~-*/
         {
            /*~T*/
            ui16_comm_time_max = def_ramp9_start ;
            break;
         /*~-*/
         }
         /*~E*/
         /*~O*/
         /*~-2*/
         default:
         {
            /*~T*/
            ui16_comm_time_max = def_ramp_end;
         /*~-*/
         }
      /*~-*/
      }
      /*~E*/
      /*~I*/
      // limit to def_ramp_end 
      if( ui16_comm_time_max < def_ramp_end )
      /*~-*/
      {
         /*~T*/
         ui16_comm_time_max = def_ramp_end;
      /*~-*/
      }
      /*~E*/
      /*~E*/
      /*~I*/
      if (Flag_MotorAlign)
      /*~-*/
      {
         /*~T*/
         //ui8_StartupPWM = 0 ; /* reset to zero every commutation start during alignement */
         ui8_StartupPWM =   def_minSTARTUP_PWM ; 
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
      /*~+:Phase Advancement handling */
      /*~I*/
        if( rising_bemf_flag) /* Be carefull this variable is changed later */ 
      /*~-*/
      {
         /*~T*/
         /* falling edge */
            phase_delay_counter = 0;
      /*~-*/
      }
      /*~O*/
      /*~-2*/
      else
      {
         /*~T*/
         /* rising edge */
            phase_delay_counter = comm_time>>1;


      /*~-*/
      }
      /*~E*/
      /*~I*/
#ifdef def_ZeroPhaseDelay
      /*~T*/
        phase_delay_counter = 0;
      /*~-*/
#endif
      /*~E*/
      /*~T*/
        phase_delay_counter_debug = phase_delay_counter;/* the phase_delay_counter is decremented in the interrupt service*/
                                                        /* phase_delay_counter_debug is not decremented and can be used for debug purposes */
      /*~E*/
      /*~A*/
      /*~+:comm_time max during running */
      /*~T*/
        ui16_comm_time_max        = def_COMM_TIME_MAX;
      /*~E*/
      /*~A*/
      /*~+:update of PWM duty cycle */
      /*~I*/
      if( Flag_RUN_MOTOR )
      /*~-*/
      {
         /*~I*/
#ifdef def_filterPWMtoPowerStage
         /*~I*/
         if (CCPR1L == ui8_duty_cycle_BLDC)
         /*~-*/
         {
            /*~T*/

         /*~-*/
         }
         /*~O*/
         /*~-2*/
         else
         {
            /*~I*/
            if( CCPR1L > ui8_duty_cycle_BLDC )

            /*~-*/
            {
               /*~T*/
               CCPR1L--;
               /*~I*/
               if (CCPR1L > ui8_duty_cycle_BLDC)
               /*~-*/
               {
                  /*~T*/
                  CCPR1L--;
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
               CCPR1L++;
               /*~I*/
               if (CCPR1L < ui8_duty_cycle_BLDC)
               /*~-*/
               {
                  /*~T*/
                  CCPR1L++;
               /*~-*/
               }
               /*~E*/
            /*~-*/
            }
            /*~E*/
         /*~-*/
         }
         /*~E*/
         /*~O*/
         /*~-*/
#else
         /*~T*/
         CCPR1L = ui8_duty_cycle_BLDC;
         /*~I*/
#ifdef def_fixed_PWM
         /*~T*/
         CCPR1L = def_fixed_PWM;
         /*~-*/
#endif
         /*~E*/
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
         CCPR1L = 0;
      /*~-*/
      }
      /*~E*/
      /*~E*/
   /*~-*/
   }
   /*~E*/
   /*~T*/
   comm_time = 0;
   /*~A*/
   /*~+:PWM on High side*/
   /*~I*/
    if( comm_state == 0xff ) /* this instruction avoids a compiler issue of some older version which generate a endless loop when switch case is 0xff */
   /*~-*/
   {
      /*~T*/
        comm_state = 6;
   /*~-*/
   }
   /*~E*/
   /*~C*/
    switch( comm_state )
   /*~-*/
   {
      /*~F*/
        case 1:

      /*~-*/
      {
         /*~A*/
         /*~+:State  Low   High  Comparator*/
         /*~+:  1     B     A       -C*/
         /*~+:*/
         /*~T*/
                PSTR1CON = 0b00000001; /* PWM on Channel A */

         /*~I*/
                if( Flag_RUN_MOTOR )
         /*~-*/
         {
            /*~T*/
                    State1_Ena;
         /*~-*/
         }
         /*~E*/
         /*~T*/
         /* PhaseA_Dir = 1 ;    not active because controlled by PWM */
                PhaseB_Dir     = 0;
                PhaseC_Dir     = 0;
                ui8_IPhase_sel = Ph1_ISen_Channel;
                ui8_UPhase_sel = Ph1_Motor_Channel;
                ui8_Ubemf_sel  = Ph3_Motor_Channel; /* comparator on channel 3 */
                CM1CON1        = SENSE_C; /* Prepare comparator to sense on C falling edge */

         /*~I*/
#ifdef def_forward
         /*~T*/
                CM1CON0          = 0x94;
                rising_bemf_flag = 0;
                comm_state       = 2;

         /*~O*/
         /*~-*/
#else
         /*~T*/
                CM1CON0          = 0x84;
                rising_bemf_flag = 1;
                comm_state       = 6;
         /*~-*/
#endif
         /*~E*/
         /*~T*/

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
         /*~+:State  Low   High  Comparator*/
         /*~+:  2     C     A        B*/
         /*~T*/
                PSTR1CON = 0b00000001;

         /*~I*/
                if( Flag_RUN_MOTOR )
         /*~-*/
         {
            /*~T*/
                    State2_Ena;
         /*~-*/
         }
         /*~E*/
         /*~T*/
         /* PhaseA_Dir = 1 ; not active because controlled by PWM */
                PhaseB_Dir     = 1;
                PhaseC_Dir     = 0;
                ui8_IPhase_sel = Ph1_ISen_Channel;
                ui8_UPhase_sel = Ph1_Motor_Channel;
                ui8_Ubemf_sel  = Ph2_Motor_Channel; /* comparator on channel 3 */
                CM1CON1        = SENSE_B;

         /*~I*/
#ifdef def_forward
         /*~T*/
                CM1CON0          = 0x84;
                rising_bemf_flag = 1;
                comm_state       = 3;


         /*~O*/
         /*~-*/
#else
         /*~T*/
                CM1CON0          = 0x94;
                rising_bemf_flag = 0;
                comm_state       = 1;
         /*~-*/
#endif
         /*~E*/
         /*~T*/



                break;

         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~F*/
        case 3:

      /*~-*/
      {
         /*~A*/
         /*~+:State  Low   High  Comparator*/
         /*~+:  3     C     B       -A*/
         /*~T*/
                PSTR1CON = 0b00000010;

         /*~I*/
                if( Flag_RUN_MOTOR )
         /*~-*/
         {
            /*~T*/
                    State3_Ena;
         /*~-*/
         }
         /*~E*/
         /*~T*/
                PhaseA_Dir     = 0;
         /* PhaseB_Dir = 1 ; not active because controlled by PWM */
                PhaseC_Dir     = 0;
                ui8_IPhase_sel = Ph2_ISen_Channel;
                ui8_UPhase_sel = Ph2_Motor_Channel;
                ui8_Ubemf_sel  = Ph1_Motor_Channel; /* comparator on channel 3 */
                CM1CON1        = SENSE_A;

         /*~I*/
#ifdef def_forward
         /*~T*/
                CM1CON0          = 0x94;
                rising_bemf_flag = 0;
                comm_state       = 4;


         /*~O*/
         /*~-*/
#else
         /*~T*/
                CM1CON0          = 0x84;
                rising_bemf_flag = 1;
                comm_state       = 2;
         /*~-*/
#endif
         /*~E*/
         /*~T*/
                break;

         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~F*/
        case 4:

      /*~-*/
      {
         /*~A*/
         /*~+:State  Low   High  Comparator*/
         /*~+:  4     A     B        C*/
         /*~+:*/
         /*~T*/
                PSTR1CON = 0b00000010;

         /*~I*/
                if( Flag_RUN_MOTOR )
         /*~-*/
         {
            /*~T*/
                    State4_Ena;
         /*~-*/
         }
         /*~E*/
         /*~T*/
                PhaseA_Dir     = 0;
         /* PhaseB_Dir = 1 ; // not active because controlled by PWM */
                PhaseC_Dir     = 1;
                ui8_IPhase_sel = Ph2_ISen_Channel;
                ui8_UPhase_sel = Ph2_Motor_Channel;
                ui8_Ubemf_sel  = Ph3_Motor_Channel; /* comparator on channel 3 */
                CM1CON1        = SENSE_C;

         /*~I*/
#ifdef def_forward
         /*~T*/
                CM1CON0          = 0x84;
                rising_bemf_flag = 1;
                comm_state       = 5;


         /*~O*/
         /*~-*/
#else
         /*~T*/
                CM1CON0          = 0x94;
                rising_bemf_flag = 0;
                comm_state       = 3;
         /*~-*/
#endif
         /*~E*/
         /*~T*/
                break;
         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~F*/
        case 5:

      /*~-*/
      {
         /*~A*/
         /*~+:State  Low   High  Comparator*/
         /*~+: 5     A     C       -B*/
         /*~+:*/
         /*~T*/
                PSTR1CON = 0b00000100;

         /*~I*/
                if( Flag_RUN_MOTOR )
         /*~-*/
         {
            /*~T*/
                    State5_Ena;
         /*~-*/
         }
         /*~E*/
         /*~T*/
                PhaseA_Dir     = 0;
                PhaseB_Dir     = 0;
         /* PhaseC_Dir = 1 ; not active because controlled by PWM */
                ui8_IPhase_sel = Ph3_ISen_Channel;
                ui8_UPhase_sel = Ph3_Motor_Channel;
                ui8_Ubemf_sel  = Ph2_Motor_Channel; /* comparator on channel 3 */
                CM1CON1        = SENSE_B;


         /*~I*/
#ifdef def_forward
         /*~T*/
                CM1CON0          = 0x94;
                rising_bemf_flag = 0;
                comm_state       = 6;


         /*~O*/
         /*~-*/
#else
         /*~T*/
                CM1CON0          = 0x84;
                rising_bemf_flag = 1;
                comm_state       = 4;
         /*~-*/
#endif
         /*~E*/
         /*~T*/
                break;

         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~F*/
        case 6:

      /*~-*/
      {
         /*~A*/
         /*~+:State  Low   High  Comparator*/
         /*~+: 6     B     C        A*/
         /*~T*/
                PSTR1CON = 0b00000100;

         /*~I*/
                if( Flag_RUN_MOTOR )
         /*~-*/
         {
            /*~T*/
                    State6_Ena;
         /*~-*/
         }
         /*~E*/
         /*~T*/
                PhaseA_Dir     = 1;
                PhaseB_Dir     = 0;
         /* PhaseC_Dir = 1 ; not active because controlled by PWM */
                ui8_IPhase_sel = Ph3_ISen_Channel;      /* Select Channel 3 for I sensing */
                ui8_UPhase_sel = Ph3_Motor_Channel;
                ui8_Ubemf_sel  = Ph1_Motor_Channel; /* comparator on channel 3 */
                CM1CON1        = SENSE_A;

         /*~I*/
#ifdef def_forward
         /*~T*/
                CM1CON0          = 0x84;
                rising_bemf_flag = 1;
                comm_state       = 1;


         /*~O*/
         /*~-*/
#else
         /*~T*/
                CM1CON0          = 0x94;
                rising_bemf_flag = 0;
                comm_state       = 5;
         /*~-*/
#endif
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
         /*~T*/
                PSTR1CON         = 0x00; /* Modulate off */
                CM1CON0          = 0x00;
                rising_bemf_flag = 0;
                comm_state       = 1;
                break;

      /*~-*/
      }
   /*~-*/
   }
   /*~E*/
   /*~E*/
   /*~I*/
   if( rising_bemf_flag )
   /*~-*/
   {
      /*~T*/
      ui8_BlankingCount = def_BLANKINGCOUNT;
   /*~-*/
   }
   /*~O*/
   /*~-2*/
   else
   {
      /*~T*/
      ui8_BlankingCount = def_BLANKINGCOUNT;
   /*~-*/
   }
   /*~E*/
   /*~T*/
   comm_time  = 0; /* reset the commuttation time */

/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void InitMotorRun*/
/*~F*/
void InitMotorRun( void  )

/*~-*/
{
   /*~I*/
   if( 0 == Flag_RUN_MOTOR )

   /*~-*/
   {
      /*~T*/
      ZC_DETECTED               = 0;
      ui16_comm_time_max        = def_ramp0_start; /* first init with the timeout during ramp up */
      comm_state                = 1;
      ui16_step_cnt             = 0;
      ui8_duty_cycle_BLDC       = 0;
      comm_time                 = 0;
      bemf_filter               = 0;
      phase_delay_counter       = 0;
      phase_delay_counter_debug = 0;
      PSTR1CON                  = 0;

      /*~T*/
      Flag_STARTUP              = 1 ;
      Flag_RUN_MOTOR            = 1 ; /* START */
      ui16_phase_angle          = def_phase_angle;
      ui8_StartupPWM            = 0 ;
      CCPR1L                    = 0 ;
      ui8_sampleState           = 0 ;
      ui8_zero_cros_cnt         = 0 ;
      Flag_MotorAlign           = 1 ;

      /*~T*/
      //bool_start_demand_mat = 1;
      //ui8_fixed_start_speed_mat = 73 ; /* 9000rpm */
      //ui16_Task_Cont500ms = 0;

      /*~T*/
      commutate( );
   /*~-*/
   }
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void InitMotorStop*/
/*~F*/
/* *********************************************************************************** */
/* *********************************************************************************** */

void InitMotorStop( )

/*~-*/
{
   /*~I*/
    if( 1 == Flag_RUN_MOTOR )

   /*~-*/
   {
      /*~T*/
        PhaseA_Ena     = 0; /* switch off */
        PhaseB_Ena     = 0;
        PhaseC_Ena     = 0;

        CCP1ASE        = 0;   /* Clear auto shutdown flag */
        CCPR1L         = 0;
        Flag_RUN_MOTOR = 0;   /* STOP */
        COMM_DONE      = 0;

   /*~-*/
   }
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
