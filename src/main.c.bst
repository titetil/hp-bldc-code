/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/** 
* \defgroup  MAIN      Main
* \brief Main module for BVH2.
* This module is calling all function into a scheduler.
* \version 1.1 - 2013-03-07
* \section main_hist History
* <b>V1.1 - 2013-03-07 - ADQ - First version for C sample</b>
* -     Change of temperature thresholds for the Matlab layer due to the change of the resistance R672
* -     Add function void I_calibrationInit(void) for the calibration of the current after each power switch on of the module.
* -     Calibration of current : offset and ramp factor set at each power switch on
*      
* <b>V1.0 - 2013-02-15 - ADQ - First numbered version</b>
* -     This version is related to HW B3    
* 
*/

/**
* \file      main.c
* \ingroup  MAIN
* \brief     Main module of BVH2 software
* \note        Target CPU    : PIC16F1936\n
*              Compiler      : HI_TECH PICC (v9.81)       
* \copyright (c) TI Automotive - All rights reserved 
* \author    TI Automotive - MMK - Mohamed Moussaddak       
* \author    TI Automotive - JPL - Joachim Plantholt        
* \author    TI Automotive - ADQ - Aymeric de Quatrebarbes  
* \requirement \b TIR-SW-Main-001 Scheduler
* \requirement \b TIR-SW-Main-002 Task time 1ms
* \requirement \b TIR-SW-Main-003 Task time 5ms
* \requirement \b TIR-SW-Main-004 Task time 100ms A B C D
* \requirement \b TIR-SW-Main-005 void system_init(void)
* \requirement \b TIR-SW-Main-006 CPU Load
* \requirement \b TIR-SW-Main-007 Watchdog
* \requirement \b TIR-SW-Main-008 System Task routine
* \todo
* \test \b TC_TIR-SW-Main-001 Task time verification (1ms and 5 ms)
* \test \b TC_TIR-SW-Main-002 Task Time 100ms A B C D
* \test \b TC_TIR-SW-Main-003 Init
* \test \b TC_TIR-SW-Main-004 System Task
* \test \b TC_TIR-SW-Main-006 Watchdog
**/
/*~T*/
/*----------------  Includes ----------------------------------------------------------------------------*/

#include     <htc.h>
#include     "config.h"
#include     "project.h"
#include     "timer.h"
#include     "pwm.h"
#include     "bldc.h"
#include     "lin.h"
#include     "diag.h"
#include     "adc.h"
#include     "eeprom.h"
#include     "cksum.h"
#include     "BVH2_Appl_Layer.h"


/*~A*/
/*~+:CONFIG bytes microcontroller*/
/*~I*/
#ifdef def_EnableClockout
/*~T*/
/** INTOSC oscillator: I/O function on CLKIN pin
* WDT disabled
* PWRT disabled
* MCLR/CPP pin function is MCLR ie MCLR disabled
* Program memory code protection is disabled 
* Data memory code protection is disabled
* Brown-out Reset enabled
* CLKOUT function is enabled on the CLKOUT pin
* Internal/External Switchover mode is enabled
* Fail_Safe Clock Monitor is enabled */
__CONFIG( FOSC_INTOSC & WDTE_OFF & MCLRE_OFF & CLKOUTEN_ON );
/*~O*/
/*~-*/
#else
/*~T*/
/** INTOSC oscillator: I/O function on CLKIN pin\n
* WDT disabled\n
* PWRT disabled\n
* MCLR/CPP pin function is digital port ie MCLR disabled\n
* Program memory code protection is disabled\n
* Data memory code protection is disabled\n
* Brown-out Reset enabled\n
* CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin\n
* Internal/External Switchover mode is enabled\n
* Fail_Safe Clock Monitor is enabled */
__CONFIG( FOSC_INTOSC & WDTE_ON & MCLRE_OFF & CLKOUTEN_OFF );
/*~-*/
#endif
/*~E*/
/*~T*/
/** Write protection off\n
* All VCAP pin functionality is disabled\n
* 4x PLL enabled\n
* Stack Overflow or Underflow will cause a Reset\n
* Brown-out Reset Voltage (VBOR) set to 2.5 V\n
* High-voltage on MCLR/VPP must be used for programming */
__CONFIG( PLLEN_ON & LVP_OFF & BORV_25 );

/*~E*/
/*~T*/
/*----------------  globale Variables      --------------------------------------------------------------*/

        unsigned  int  ui16_Timer_VaL1;
        unsigned  char ui8_Task_Cont3ms; /**< Counter for executing tasks every 3 ms*/
        unsigned  char ui8_Task_Cont5ms; /**< Counter for executing tasks every 5 ms*/
        unsigned  char ui8_Task_Cont100ms; /**< Counter for executing tasks every 100 ms*/
        unsigned  int  ui16_Task_Cont500ms; /**< Counter for executing tasks every 500 ms*/
        unsigned  char ui8_Sync_Cont10ms; /**< Counter for synchronisation for 10ms at CDE_MJP > 10% */
        unsigned  char ui8_lastTaskvalue; /**< Comparison with ui8_Task_Cont1ms */
        unsigned  int  ui16_I_cal_Ph1 = 1; /**< Calibration value for I_Phase1 */
        unsigned  int  ui16_I_cal_Ph2 = 1; /**< Calibration value for I_Phase2 */
        unsigned  int  ui16_I_cal_Ph3 = 1; /**< Calibration value for I_Phase3 */
        unsigned  int  ui16_Temp_cal; /**< Calibration value for CPU temperature */
        unsigned  char ui8_DebugCnt; /**< Counter for test */
        unsigned  char ui8_i_wob = 0; /**< Index of wobbling table */
        unsigned  char ui8_selected_lid = 0x80; /**< Initialisation of LID for EOL routine */
        unsigned  int  ui16_wait = 0; /**< Counter to see the end of the initialisation of the system */
        _D_TYPE        sum;
        _D_TYPE        checksum[ 3 ];

unsigned char ui8_current_cal[3];
unsigned char ui8_calib_flag;

extern  unsigned  char ui8_Task_Cont1ms;
extern  unsigned  char ui8_Ki_Lin;
extern  unsigned  char ui8_Kp_Lin;
extern  unsigned  char ui8_temp_calibration;
extern  unsigned  char ui8_b_DResServID_c;
extern  unsigned  char ui8_b_DResLocID_c;
extern  unsigned  char ui8_b_DResB0_c;
extern  unsigned  char ui8_b_DResB1_c;
extern  unsigned  char ui8_b_DResB2_c;
extern  unsigned  char ui8_b_DResB3_c;
extern  unsigned  char ui8_b_DResB4_c;
extern  unsigned  char ui8_b_DResB5_c;
extern  unsigned  char ui8_duty_cycle_mat;


/*~T*/
/*-------------------  Prototypes  ----------------------------- */
        void init_ports( void  );
        void system_init( void  );

extern  void Task1ms( void  );
/*~A*/
/*~+:void I_calibrationInit(void)*/
/*~T*/
/**
* \fn      void I_calibrationInit(void)
* \brief   Measure the offset of the current phase\n
*          Return the offset value to be substract to have the real current value to each phase.\n
*          Give the factor value for the ramp written in EEPROM by EOL.\n
* \pre     None
* \post    None
* \details The calibrate current phase must be always positive because of post treatement of the current. This function is called after all power reset of the system. 
**/
/*~F*/
void I_calibrationInit(void)
/*~-*/
{
   /*~I*/
   if ( read_eeprom_data( 0x03 ) != 0x00 && read_eeprom_data( 0x03 ) != 0xFF)
   /*~-*/
   {
      /*~T*/
      ui16_I_cal_Ph1 = ( read_eeprom_data( 0x02 )<<8 ) | read_eeprom_data( 0x03 );

   /*~-*/
   }
   /*~O*/
   /*~-2*/
   else
   {
      /*~T*/
      ui16_I_cal_Ph1 = 0x0100;

   /*~-*/
   }
   /*~E*/
   /*~I*/
   if ( read_eeprom_data( 0x05 ) != 0x00 && read_eeprom_data( 0x05 ) != 0xFF)
   /*~-*/
   {
      /*~T*/
      ui16_I_cal_Ph2 = ( read_eeprom_data( 0x04 )<<8 ) | read_eeprom_data( 0x05 );

   /*~-*/
   }
   /*~O*/
   /*~-2*/
   else
   {
      /*~T*/
      ui16_I_cal_Ph2 = 0x0100;

   /*~-*/
   }
   /*~E*/
   /*~I*/
   if ( read_eeprom_data( 0x07 ) != 0x00 && read_eeprom_data( 0x07 ) != 0xFF)
   /*~-*/
   {
      /*~T*/
      ui16_I_cal_Ph3 = ( read_eeprom_data( 0x06 )<<8 ) | read_eeprom_data( 0x07 );
   /*~-*/
   }
   /*~O*/
   /*~-2*/
   else
   {
      /*~T*/
      ui16_I_cal_Ph3 = 0x0100;

   /*~-*/
   }
   /*~E*/
   /*~I*/
#ifdef calib_after_init_bldc
   /*~A*/
   /*~+:Old calibration*/
   /*~T*/
   ui8_lastTaskvalue = 0;
   ui8_calib_flag = 1;

   unsigned int i;
   /*~L*/
   while (ui8_calib_flag == 1)
   /*~-*/
   {
      /*~L*/
      while (ui16_IPhase1_bldc.w == 0) /* Wait for the first read of the ADC Channel. While the value is not read, the variable is staying to 0. Once the ADC is read, the variable is set to a positive value because of the offset at IS on the MOSFET. The loop is exited and the calibration is done. If the read is to long or the ADC value is 0, the loop is exited after 12.5 ms and the calibration is done. */
      /*~-*/
      {
         /*~T*/
         i++;

         /*~I*/
         if( i > 5000 ) /* Wait for 25ms before forcing to exit the loop */
         /*~-*/
         {
            /*~T*/
            i = 0;
            break;
         /*~-*/
         }
         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~T*/
      ui8_current_cal[0] = (unsigned char)(ui16_IPhase1_bldc.w); /* The calibrate current phase must be always positive because of post treatement of the current. If this calibrated current is negative, the recognition is overcurrent and the corresponding limp mode is executed. To avoid this statement, the calibration value is a little higher as the real value. */
      /*~L*/
      while (ui16_IPhase2_bldc.w == 0)
      /*~-*/
      {
         /*~T*/
         i++;

         /*~I*/
         if( i > 5000 ) /* Wait for 25ms before forcing to exit the loop */
         /*~-*/
         {
            /*~T*/
            i = 0;
            break;
         /*~-*/
         }
         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~T*/
      ui8_current_cal[1] = (unsigned char)(ui16_IPhase2_bldc.w);

      /*~L*/
      while (ui16_IPhase3_bldc.w == 0)
      /*~-*/
      {
         /*~T*/
         i++;

         /*~I*/
         if( i > 5000 ) /* Wait for 25ms before forcing to exit the loop */
         /*~-*/
         {
            /*~T*/
            i = 0;
            break;
         /*~-*/
         }
         /*~E*/
      /*~-*/
      }
      /*~E*/
      /*~T*/
      ui8_current_cal[2] = (unsigned char)(ui16_IPhase3_bldc.w);
      /*~T*/
      ui8_lastTaskvalue = ui8_Task_Cont1ms;
      ui8_calib_flag = 0;
   /*~-*/
   }
   /*~E*/
   /*~E*/
   /*~O*/
   /*~-*/
#else
   /*~T*/
   PhaseA_Dir = 0;
   PhaseB_Dir = 0;
   PhaseC_Dir = 0;
   PhaseX_Ena;

   ADC_Wait();
   ADC_Wait();

   mADC_ChanSelect(Ph3_ISen_Channel);
   ADC_Wait();
   mADC_TrigConversion( );
   //ui16_IPhase3_bldc.w = ADC_Read();
   //ui8_current_cal[2] = ui16_IPhase3_bldc.w; /* Typical offset of MOSFET is 40ADC for 1k*/
   ui8_current_cal[2] = ADC_Read();

   mADC_ChanSelect(Ph2_ISen_Channel);
   ADC_Wait();
   mADC_TrigConversion( );
   //ui16_IPhase2_bldc.w = ADC_Read();
   //ui8_current_cal[1] = ui16_IPhase2_bldc.w; /* Typical offset of MOSFET is 40ADC for 1k*/
   ui8_current_cal[1] = ADC_Read();

   mADC_ChanSelect(Ph1_ISen_Channel);
   ADC_Wait();
   mADC_TrigConversion( );
   //ui16_IPhase1_bldc.w = ADC_Read();
   //ui8_current_cal[0] = ui16_IPhase1_bldc.w; /* Typical offset of MOSFET is 40ADC for 1k */
   ui8_current_cal[0] = ADC_Read();

   /*~-*/
#endif
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:Wobble tables */
/*~I*/
#ifdef def_wobb_
/*~T*/
#define     def_wobb_table1 /**< Wobbling range of 2kHz */     
//#define def_wobb_table2 /**< Wobbling range of 4kHz */
//#define def_wobb_table3 /**< Wobbling range of 1kHz */
/*~I*/
#ifdef def_wobb_table1
/*~T*/
const unsigned char wob_per[10]  = {101, 103, 104, 102, 100, 99, 97, 96, 98, 100} ; /**< Wobbling from 19.2 to 20.8 kHz */
/*~-*/
#endif
/*~E*/
/*~I*/
#ifdef def_wobb_table2
/*~T*/
const unsigned char wob_per[10]  = {102, 106, 108, 104, 100, 98, 94, 92, 96, 100} ; /**< Wobbling from 18.4 to 21.6 kHz */
/*~-*/
#endif
/*~E*/
/*~I*/
#ifdef def_wobb_table3
/*~T*/
const unsigned char wob_per[10]  = {101, 102, 102, 101, 100, 99, 98, 98, 99, 100} ; /**< Wobbling from 18.4 to 21.6 kHz */
/*~-*/
#endif
/*~E*/
/*~-*/
#endif
/*~E*/
/*~E*/
/*~A*/
/*~+:void init_ports(void)*/
/*~T*/
/**
* \fn      void init_ports(void)
* \brief   Initialisation of port assignement. 
*          This function determines if the ports are inputs or outputs and the type of assignement (ADC, CLK, PWM...)
* \pre     None
* \post    None
**/
/*~F*/
void init_ports( void  )
/*~-*/
{
   /*~T*/
   /* Definition of analog inputs */
   /** RA0, RA1, RA2, RA5 defined as analog input correspond to AN0, AN1, AN2, AN4\n
     * RB0, RB3, RB5 defined as analog input correspond to AN12, AN9, AN13 */
   ANSELA = 0b00100111;  /* RA0, RA1, RA2, RA5 defined as analog input correspond to AN0, AN1, AN2, AN4*/
   ANSELB = 0b00101001;  /* RB0, RB3, RB5 defined as analog input correspond to AN12, AN9, AN13*/

   /* Initialisation of Data Latch registers */
   LATA   = 0;
   LATB   = 0;
   LATC   = 0;

   /*~I*/
#if defined  (  def_LIN  )  ||  defined  (  def_LIN_5ms  )
   /*~I*/
   if ((read_eeprom_data(0x55)==0x55) || (RC5==0) )
   /*~-*/
   {
      /*~T*/
      TRISA = 0b01111111;
      /** Port RA0 = Ph1_U_Sense\n
      * Port RA1 = Ph2_U_Sense\n
      * Port RA2 = Ph1_I_Sense\n
      * Port RA3 = Ph_Sum_Sense\n
      * Port RA4 = CDE_MJP_PIC\n
      * Port RA5 = Ph3_I_Sense\n
      * Port RA6 = PIC_ETAT_Monitor\n
      * Port RA7 = Output*/
      TRISB = 0b00111001;
      /** Port RB0 = UbatSense Input\n
      * Port RB1 = PhaseC_Dir\n
      * Port RB2 = PhaseB_Dir\n
      * Port RB3 = Starpoint Input\n
      * Port RB4 = NTCSense Input\n
      * Port RB5 = PhaseBISense Input\n
      * Port RB6 = Output\n
      * Port RB7 = Output */
      TRISC = 0b00000000;
      /** Port RC0 = ETAT_MJP PWM output\n
      * Port RC1 = PhaseA_Ena\n
      * Port RC2 = PhaseA_Dir\n
      * Port RC3 = PhaseB_Ena\n
      * Port RC4 = PhaseC_Ena\n
      * Port RC5 = unused Debug Can be used for enable MCP201\n
      * Port RC6 = output\n
      * Port RC7 = output */

   /*~-*/
   }
   /*~O*/
   /*~-2*/
   else
   {
      /*~T*/
      TRISA = 0b01111111;
      /** Port RA0 = Ph1_U_Sense\n
      * Port RA1 = Ph2_U_Sense\n
      * Port RA2 = Ph1_I_Sense\n
      * Port RA3 = Ph_Sum_Sense\n
      * Port RA4 = CDE_MJP_PIC\n
      * Port RA5 = Ph3_I_Sense\n
      * Port RA6 = PIC_ETAT_Monitor\n
      * Port RA7 = Output*/
      TRISB = 0b00111001;
      /** Port RB0 = UbatSense Input\n
      * Port RB1 = PhaseC_Dir\n
      * Port RB2 = PhaseB_Dir\n
      * Port RB3 = Starpoint Input\n
      * Port RB4 = NTCSense Input\n
      * Port RB5 = PhaseBISense Input\n
      * Port RB6 = PRGC Input\n
      * Port RB7 = PRGD Input */
      TRISC = 0b10000000;
      /** Port RC0 = ETAT_MJP PWM output\n
      * Port RC1 = PhaseA_Ena\n
      * Port RC2 = PhaseA_Dir\n
      * Port RC3 = PhaseB_Ena\n
      * Port RC4 = PhaseC_Ena\n
      * Port RC5 = unused Debug Can be used for enable MCP201\n
      * Port RC6 = Lin TX\n
      * Port RC7 = Lin RX  */

   /*~-*/
   }
   /*~E*/
   /*~O*/
   /*~-*/
#else
   /*~A*/
   /*~+:No LIN*/
   /*~T*/
   TRISA = 0b01111111;
   /** Port RA0 = Ph1_U_Sense\n
   * Port RA1 = Ph2_U_Sense\n
   * Port RA2 = Ph1_I_Sense\n
   * Port RA3 = Ph_Sum_Sense\n
   * Port RA4 = CDE_MJP_PIC\n
   * Port RA5 = Ph3_I_Sense\n
   * Port RA6 = PIC_ETAT_Monitor\n
   * Port RA7 = Output*/
   TRISB = 0b00111001;
   /** Port RB0 = UbatSense Input\n
   * Port RB1 = PhaseC_Dir\n
   * Port RB2 = PhaseB_Dir\n
   * Port RB3 = Starpoint Input\n
   * Port RB4 = NTCSense Input\n
   * Port RB5 = PhaseBISense Input\n
   * Port RB6 = Output\n
   * Port RB7 = Output */
   TRISC = 0b00000000;
   /** Port RC0 = ETAT_MJP PWM output\n
   * Port RC1 = PhaseA_Ena\n
   * Port RC2 = PhaseA_Dir\n
   * Port RC3 = PhaseB_Ena\n
   * Port RC4 = PhaseC_Ena\n
   * Port RC5 = unused Debug Can be used for enable MCP201\n
   * Port RC6 = output\n
   * Port RC7 = output */

   /*~E*/
   /*~-*/
#endif
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void systemInit(void)*/
/*~T*/
/**
* \fn      void system_init(void)
* \brief   Initialisation of the microcontroller. 
*          This function initialises the different variables used in the program and the different registers of the microcontroller
* \pre     None
* \post    None
**/
/*~F*/
void system_init( void  )
/*~-*/
{
   /*~T*/
   OPTION_REG         = 0b10000001;
   /** 
   All weak pull-ups are disabled\n
   Interrupt on falling edge of INT pin\n
   Internal instruction cycle clock (Fosc/4)\n
   Increment on low-to-high transition on T0CKI pin\n
   Prescaler is assigned to the Timer0 module\n
   Prescaler Timer0 rate = 1:4 */
   /*~T*/
   ui8_ResetMatlab    = 1; /** ui8_ResetMatlab set to 1 */
   GIE                = 0; /** All interrupt disabled */
   INTCON             = 0; /** INTCON is normally even cleared by reset */
   PIE1               = 0; /** All peripheral interrupts are disabled */
   PIE2               = 0;
   PIE3               = 0;
   /*~T*/
   ui16_Timer_VaL1    = 0;
   ui8_Task_Cont3ms   = 0;
   ui8_Task_Cont5ms   = 0;
   ui8_Task_Cont100ms = 0;
   ui8_DebugCnt       = 0;

   ui8_Sync_Cont10ms  = 0;

   ui16_dryRun_Thresh = 73; /* 3.6A */
   ui16_Current_Thresh = 205; /* 10A */
   /*~T*/
   init_ports( ); /** Initialisation of ports */ 
   /*~I*/
#if defined  (  def_LIN  )  ||  defined  (  def_LIN_5ms  )
   /*~T*/
   EnableMCP201( );/** Enabling the LIN driver */
   _ELINMIntInitialize( );/** Initialisation the lin communication */

   /*~-*/
#endif
   /*~E*/
   /*~T*/
   Oscill_Source_Block( );
   timer_init( 1 ); /** Initialisation of Timer1 */
   timer_init( 4 ); /** Initialisation of Timer4 */
   timer_init( 6 ); /** Initialisation of Timer6 */

   /*~T*/
   PWM_Capture_init( 5 ); /** Initialisation of PWM input */
   ADC_Init( ); /** Initialisation of ADCs */
   DiagInit( ); /** Initialisation of Diagnostic flags */
   FILTER_Init( );
   /*~T*/
   //ui16_Temp_cal = ( read_eeprom_data( 0x00 )<<8 ) | read_eeprom_data( 0x01 ); // Temperature calibration
   ui16_Temp_cal = 0; // No temperature calibration
   //ui16_mat_inpTemp = 572; /* Initialisation of temperature for matlab at 25øC */
   /*~T*/
   /** Write the SW version in EEPROM */
   //write_eeprom_data( def_adr_SoftwareVersion, ui8_SoftwareVersion_MSB );
   //write_eeprom_data( def_adr_SoftwareVersion + 1, ui8_SoftwareVersion_LSB );

   /*~T*/
   /* EEPROM Initialisation */
   WREN = 0;
   WR = 0;
   RD = 0;
   /*~T*/
   I_calibrationInit(); /** Calibration of current after all power reset. This calibration must be made when the motor is off. */
   /*~T*/
   init_bldc( ); /** Initialisation of BLDC driver */ 
   /*~T*/
   PEIE = 1; /** Set all peripheral interrupts */
   GIE  = 1; /** Set all global interrupts */
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void main(void)*/
/*~T*/
/**
* \fn      void main(void)
* \brief   Main loop of the BVH2 Program 
* \pre     None
* \post    None
**/
/*~F*/
void main( void )
/*~-*/
{
   /*~I*/
#ifdef def_LIN_5ms 
   /*~T*/
   unsigned  char ui8_CCPR1L_scaled; /** used only for debug with LIN_5ms */

   /*~-*/
#endif
   /*~E*/
   /*~T*/
   system_init( );/** Initialise the system with the function system_init() */
   clear_timer( 4 );/** Clear the timer 4 which is used for the scheduler */
   ui8_lastTaskvalue = 0;

   /*~A*/
   /*~+:Signal for end of initialisation on ETAT_MJP: 0 for 2.5ms, 1 for 2.5ms and 0.*/
   /*~I*/
   //#define init_signal_etat_mjp
#ifdef init_signal_etat_mjp
   /*~T*/
   /* Signal on debug port to have the end of initialisation. This signal is recognisable by sending 0 during 5ms then 1 during 5ms then 0 again. */
    Port_PWMout = 1;
   /*~L*/
    for( ui16_wait = 0; ui16_wait < 500; ui16_wait++ )
   /*~-*/
   {
      /*~T*/
        NOP( );
        NOP( );
        NOP( );
        NOP( );
        NOP( );
        NOP( );
        NOP( );
        NOP( );
   /*~-*/
   }
   /*~E*/
   /*~T*/
    Port_PWMout = 0;
   /*~L*/
    for( ui16_wait = 0; ui16_wait < 500; ui16_wait++ )
   /*~-*/
   {
      /*~T*/
        NOP( );
        NOP( );
        NOP( );
        NOP( );
        NOP( );
        NOP( );
        NOP( );
        NOP( );
   /*~-*/
   }
   /*~E*/
   /*~T*/
    Port_PWMout = 1;
   /*~-*/
#endif
   /*~E*/
   /*~E*/
   /*~L*/
   while( 1 )
   /*~-*/
   {
      /*~I*/
      if( ( ui8_Task_Cont1ms - ui8_lastTaskvalue ) != 0) /* I taksvalue was incremented */ 
      /*~-*/
      {
         /*~A*/
         /*~+:Task 1 ms*/
         /*~T*/
         /**  ## Task 1 ms */
         /*~T*/
         ui8_debug_out0 = ~ui8_debug_out0;
         ui8_lastTaskvalue = ui8_Task_Cont1ms;

         /*~T*/
         ui8_Task_Cont3ms++;
         ui8_Task_Cont5ms++; /** Increment counter for task 5 ms */
         ui8_Task_Cont100ms++; /** Increment counter for task 100 ms */

         /*~I*/
#ifdef def_wobb_
         /*~A*/
         /*~+:Wobbling*/
         /*~T*/
         /** Use motor PWM frequency wobbling */
         /*~I*/
         if( ui8_i_wob < 9 )
         /*~-*/
         {
            /*~T*/
            ui8_i_wob++;

         /*~-*/
         }
         /*~O*/
         /*~-2*/
         else
         {
            /*~T*/
            ui8_i_wob = 0;
         /*~-*/
         }
         /*~E*/
         /*~T*/
         PR2 = wob_per[ ui8_i_wob ]; /** Force the PWM frequency */ 
         /*~I*/
#ifdef adap_DC_wobb
         /*~T*/
         /** Adapt the PWM DC with the frequency */
         ui8_duty_cycle_BLDC = ui8_duty_cycle_mat * PR2 / 100;

         /*~O*/
         /*~-*/
#else
         /*~T*/
         /** No adaptation of the PWM DC with the frequency */
         ui8_duty_cycle_BLDC = ui8_duty_cycle_mat;
         /*~-*/
#endif
         /*~E*/
         /*~T*/

         /*~E*/
         /*~O*/
         /*~-*/
#else
         /*~A*/
         /*~+:No wobbling*/
         /*~T*/
         /** Don't use motor PWM wobbling */
         /*~I*/
#ifdef HIGH_TEMPERATURE10kHz /* period and out DC assignement */ 
         /*~A*/
         /*~+:10 kHz behavior enabled*/
         /*~I*/
         /** Reduction of the frequency of PWM Motor in case of high temperature */
         if( bool_OverTemp_err_Flag == 1) /** The flag for overtemperature is set by the Matlab layer. Then the limp mode (10kHz or 16kHz is set to reduce the temperature of the system */ 
         /*~-*/
         {
            /*~I*/
#ifdef def_fixed_PWM
            /*~A*/
            /*~+:def_fixed_PWM*/
            /*~T*/
            /** With overtemperature, the PWM DC of the motor is fixed to the value #def_dixed_PWM and the frequency of this PWM is 10kHz (just if the DC is different to 0% or 100%. */
            PR2                 = DEF_PWM_PERIOD_VALUE * 2;
            ui8_duty_cycle_BLDC = def_fixed_PWM;
            /*~E*/
            /*~O*/
            /*~-*/
#else
            /*~I*/
#ifdef def_overshooting
            /*~A*/
            /*~+:Overshooting*/
            /*~I*/
            if( ui16_Task_Cont500ms < def_overshooting )
            /*~-*/
            {
               /*~T*/
               /** In case of overshooting at every start of the pump: during the start time define by #def_overshooting, the PWM DC of the pump is forced to 100%. The frequency is also forced to 10kHz, which have no really impact for this time. But there is an impact for the initialisation */
               PR2 = DEF_PWM_PERIOD_VALUE * 2;
               ui8_duty_cycle_BLDC = 125;
            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~T*/
               /** If there is still the overtemperature flag and the system is not in the starting phase, the frequency of the motor is 10kHz and the duty cycle is preserved. It has for impact a multiplication of 2 of #ui8_duty_cycle_mat because this variable is define for 20kHz. */
               PR2 = DEF_PWM_PERIOD_VALUE * 2;
               ui8_duty_cycle_BLDC = ui8_duty_cycle_mat * 2;
            /*~-*/
            }
            /*~E*/
            /*~E*/
            /*~O*/
            /*~-*/
#else
            /*~A*/
            /*~+:10kHz in case of overtemperature*/
            /*~T*/
            /** Each time the flag for over temperature is set, the frequency of the Motor PWM is set to 10kHz to reduce the temperature. The DC is always the same. But to have this same DC, the variable #ui8_duty_cycle_mat must be adapted and multiply by 2 because it is given for 20kHz. */
            PR2 = DEF_PWM_PERIOD_VALUE * 2;
            ui8_duty_cycle_BLDC = ui8_duty_cycle_mat * 2;
            /*~E*/
            /*~-*/
#endif
            /*~E*/
            /*~-*/
#endif
            /*~E*/
         /*~-*/
         }
         /*~O*/
         /*~-2*/
         else
         {
            /*~I*/
#ifdef def_fixed_PWM
            /*~A*/
            /*~+:def_fixed_PWM*/
            /*~T*/
            /** With overtemperature, the PWM DC of the motor is fixed to the value #def_dixed_PWM and the frequency of this PWM is 16kHz (just if the DC is different to 0% or 100%. */
            PR2 = DEF_PWM_PERIOD_VALUE * 5 / 4;
            ui8_duty_cycle_BLDC = def_fixed_PWM;
            /*~E*/
            /*~O*/
            /*~-*/
#else
            /*~I*/
#ifdef def_overshooting
            /*~A*/
            /*~+:Overshooting*/
            /*~I*/
            if( ui16_Task_Cont500ms < def_overshooting )
            /*~-*/
            {
               /*~T*/
               /** In case of overshooting at every start of the pump: during the start time define by #def_overshooting, the PWM DC of the pump is forced to 100%. The frequency is also forced to 16kHz, which have no really impact for this time. But there is an impact for the initialisation */
               PR2 = DEF_PWM_PERIOD_VALUE * 5 / 4;
               ui8_duty_cycle_BLDC = 125;
            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~T*/
               /** If there is still no overtemperature flag and the system is not in the starting phase, the frequency of the motor is 16kHz and the duty cycle is preserved. */
               PR2 = DEF_PWM_PERIOD_VALUE * 5 / 4;
               ui8_duty_cycle_BLDC = ui8_duty_cycle_mat * 5 / 4;

            /*~-*/
            }
            /*~E*/
            /*~E*/
            /*~O*/
            /*~-*/
#else
            /*~A*/
            /*~+:16kHz in case of normal temperature*/
            /*~T*/
            /** The Normal behavior correspond to a frequency of the system of 16kHz. Each time the flag for over temperature isn't set, the frequency of the Motor PWM is set to 16kHz. */
            PR2 = DEF_PWM_PERIOD_VALUE * 5 / 4;
            ui8_duty_cycle_BLDC = ui8_duty_cycle_mat * 5 / 4;

            /*~E*/
            /*~-*/
#endif
            /*~E*/
            /*~-*/
#endif
            /*~E*/
         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~O*/
         /*~-*/
#else
         /*~I*/
#ifdef def_fixed_PWM /* period and out DC assignement */ 
         /*~T*/
         ui8_duty_cycle_BLDC = def_fixed_PWM; /** motor DC set to def_fixed_PWM 100% */ 
         /*~O*/
         /*~-*/
#else
         /*~I*/
#ifdef def_overshooting
         /*~I*/
         if( ui16_Task_Cont500ms < def_overshooting) /* period and out DC assignement */ 
         /*~-*/
         {
            /*~T*/
            PR2 = DEF_PWM_PERIOD_VALUE * 5 / 4;
            ui8_duty_cycle_BLDC = 125;
         /*~-*/
         }
         /*~O*/
         /*~-2*/
         else
         {
            /*~T*/
            PR2 = DEF_PWM_PERIOD_VALUE * 5 / 4;
            ui8_duty_cycle_BLDC = ui8_duty_cycle_mat * 5 / 4;
         /*~-*/
         }
         /*~E*/
         /*~O*/
         /*~-*/
#else
         /*~T*/
         PR2 = DEF_PWM_PERIOD_VALUE * 5 / 4;
         ui8_duty_cycle_BLDC = ui8_duty_cycle_mat*5/4
         /*~-*/
#endif
         /*~E*/
         /*~-*/
#endif
         /*~E*/
         /*~-*/
#endif
         /*~E*/
         /*~T*/

         /*~E*/
         /*~-*/
#endif
         /*~E*/
         /*~T*/
         Get_Analog_Value( ); /** Give back all analog values */

         /*~I*/
#ifdef def_LIN_Sync
         /*~I*/
         if( ui8_PWM_dc_mat > 21) /* Input PWM > 10% */ 
         /*~-*/
         {
            /*~I*/
            if( ui8_Sync_Cont10ms < 11 )
            /*~-*/
            {
               /*~T*/
               Port_PWMout = 0; /* etat_Mjp = 1 */
               ui8_Sync_Cont10ms++;
            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~T*/
               Port_PWMout = 1; /* etat_Mjp = 0 */

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
            Port_PWMout       = 1; /* etat_Mjp = 0 */
            ui8_Sync_Cont10ms = 0;
         /*~-*/
         }
         /*~E*/
         /*~-*/
#endif
         /*~E*/
         /*~E*/
         /*~A*/
         /*~+:Task 3 ms -- For LIN communication with 1 Byte*/
         /*~I*/
         if( ui8_Task_Cont3ms > 2 )
         /*~-*/
         {
            /*~T*/
            ui8_Task_Cont3ms = 0;
            /*~A*/
            /*~+:LIN_3ms*/
            /*~I*/
#ifdef def_LIN_5ms
            /*~I*/
            if ((read_eeprom_data(0x55)==0x55) || (RC5==0) )
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
               ui8_CCPR1L_scaled = CCPR1L * 4 / 5;

               /*~T*/
               //Transmit_LIN_Byte(0x2d,(unsigned char)(!Port_PWMout<<7 | ui8_duty_cycle_mat)); /** Transmit 1 byte with LIN enabled */
               //Transmit_LIN_Byte(0x2d,(unsigned char)(!Port_PWMout<<7 | ui8_CCPR1L_scaled)); /** Transmit 1 byte with LIN enabled */
               //Transmit_LIN_Byte(0x2d,(unsigned char)(ui16_mat_inpTemp>>2)); /** Transmit 1 byte with LIN enabled. Lin value must be multiplied by 80 to have the speed of the pump */
               //Transmit_LIN_Byte(0x2f,(unsigned char)(16000/ui16_speed_fil)); /** Transmit 1 byte with LIN enabled. Lin value must be multiplied by 40 to have the speed of the pump */
               //Transmit_LIN_Byte(0x2f,(unsigned char)(ui16_speed_rar)); /** Transmit 1 byte with LIN enabled. Lin value must be multiplied by 40 to have the speed of the pump */

            /*~-*/
            }
            /*~E*/
            /*~-*/
#endif
            /*~E*/
            /*~E*/
         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~A*/
         /*~+:Task 5 ms -- Matlab routine, LIN 3 bytes transmission, Watchdog*/
         /*~I*/
         if( ui8_Task_Cont5ms > 4 )
         /*~-*/
         {
            /*~T*/
            /** ## Task 5 ms */
            /*~T*/
            ui8_Task_Cont5ms = 0;

            /*~T*/
            CLRWDT( ); /** Enables the Watchdog */ 
            /*~A*/
            /*~+:Input connection of matlab layer*/
            /*~T*/
            /** Connect the matlab layer with the program */
            ui8_PWM_dc_mat    = PWMReadDC( );          /* connect input duty cycle to Matlab ui8_duty_cycle 0 .. 200 */
            ui16_PWM_Freq_mat = ui16_PWM_Freq_In;
            ui16_Speed_mat    = ui16_speed_fil;
            ui16_mat_inpTemp = ui16_NTC_Temp_bldc_mean_cal;
            ui8_Ki_mat        = 5;
            ui8_Kp_mat        = 0;
            /*~A*/
            /*~+:ui16_Speed_demand_mat assignement to PWM_trans_table*/
            /*~I*/
#ifdef HIGH_TEMPERATURE10kHz /* Speed demand from PWM input assignement */ 
            /*~A*/
            /*~+:10kHz*/
            /*~T*/
            /** At high temperature, adapt the #PWM_trans_table to have the wanted speed.  */
            /*~I*/
            if( bool_OverTemp_err_Flag == 1 )
            /*~-*/
            {
               /*~T*/
               /* Speed assignement */
               ui16_Speed_demand_mat = ( PWM_trans_table[ ( ui8_PWM_dc_mat>>1 ) ] )>>1;

               /* Max Speed assignement */
               ui16_Speed_demand_mat_Max = ( PWM_trans_table[ 95 ] )>>1;

               /* Min Speed assignement */
               ui16_Speed_demand_mat_min = ( PWM_trans_table[ 11 ] )>>1;
            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~T*/
               /* Speed assignement */
               ui16_Speed_demand_mat = PWM_trans_table[ ( ui8_PWM_dc_mat>>1 ) ] * 4 / 5;

               /* Max Speed assignement */
               ui16_Speed_demand_mat_Max = ( PWM_trans_table[ 95 ] ) * 4 / 5;

               /* Min Speed assignement */
               ui16_Speed_demand_mat_min = ( PWM_trans_table[ 11 ] ) * 4 / 5;
            /*~-*/
            }
            /*~E*/
            /*~E*/
            /*~O*/
            /*~-*/
#else
            /*~A*/
            /*~+:Normal - 16kHz*/
            /*~T*/
            /* Speed assignement */
            ui16_Speed_demand_mat = PWM_trans_table[ ( ui8_PWM_dc_mat>>1 ) ] * 4 / 5;

            /* Max Speed assignement */
            ui16_Speed_demand_mat_Max = ( PWM_trans_table[ 95 ] ) * 4 / 5;

            /* Min Speed assignement */
            ui16_Speed_demand_mat_min = ( PWM_trans_table[ 11 ] ) * 4 / 5;
            /*~E*/
            /*~-*/
#endif
            /*~E*/
            /*~E*/
            /*~I*/
#ifdef def_LIN_Sync
            /*~T*/
            bool_mat_pic_etat = 0;
            /*~O*/
            /*~-*/
#else
            /*~T*/
            //bool_mat_pic_etat = Error_PICetatMonitor ;
            /*~-*/
#endif
            /*~E*/
            /*~T*/
            ui8_BattVolt_mat = ( unsigned char )( ui16_fir_Bat_mittel>>2 );
            ui16_mat_Current        = ui16_fir_IPhase_mean.w;
            bool_mat_currAlarm_bldc = OverCurrentFlag;
            /*~I*/
#ifdef def_OpenLoopMode
            /*~T*/
            /** Open loop mode defined */
            bool_ControlLoopMode = 1;
            /*~O*/
            /*~-*/
#else
            /*~T*/
            /** Close loop mode defined */
            bool_ControlLoopMode = 0;
            /*~-*/
#endif
            /*~E*/
            /*~E*/
            /*~A*/
            /*~+:Exectution of Matlab routine*/
            /*~T*/
            BVH2_Appl_Layer( ); /** Call the Matlab layer to generate the flags and the limp modes */

            /*~T*/
            ui8_ResetMatlab = 0; /** Clear the Matlab reset */

            /*~E*/
            /*~A*/
            /*~+:Output connection of matlab layer*/
            /*~I*/
            if( ui8_duty_cycle_mat == 0 )
            /*~-*/
            {
               /*~T*/
               InitMotorStop( ); /** initialisation of motor stopped if output DC set to 0 by Matlab layer */

            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~T*/
               InitMotorRun( ); /** Initialisation of motor run if run again after stop */

            /*~-*/
            }
            /*~E*/
            /*~T*/
            /* assign LIN error to matlab state */

            /*~T*/
            /** Connect outputs of matlab layer to the program. The different flags are treated for diagnostics output */
            bool_PWMin_err_Flag       = bool_PWMin_err_Alarm;  /* Alarm for PWM_KO22 */
            bool_OverTemp_err_Flag    = bool_CPU_TempAlarm;    /* Alarm for PWM_KO33*/
            bool_OverTemp_severe_Flag = bool_CPU_TempRedAlarm; /* Flag for Critical over temperature */
            bool_OverCurr_err_Flag    = bool_HighCurrentAlarm; /* Alarm for PWM_KO44*/
            bool_Pump_err_Flag        = bool_MotorStalled;  /* || bool_PIC_Alarm */      /* Alarm for PWM_KO55 */
            bool_DryRunning_err_Flag  = bool_DryRunningAlarm;  /* Alarm for PWM_KO66 */
            bool_PowerStage_err_Flag  = bool_UbatAlarm;        /* Alarm for PWM_KO88 */
            bool_PWMin_Freq_err_Flag  = bool_PWMin_Freq_err_Alarm; /* Alarm for PWM_KO77 */


            /*~E*/
            /*~T*/
            OverCurrentFlag           = 0; /** Once the over current alarm is computed by the application Layer this alarm is reset (Handshake). If the current is still higher than expected, the BLDC module set this flag to 1 again and stop the system if it was not stopped. The Matlab layer continue the treatment of the flag in the next 5ms  */

            /*~I*/
#ifdef def_EnableTempErrorSwitch
            /*~A*/
            /*~+:Stop motor when error occurs*/
            /*~I*/
            if( bool_CPU_TempAlarm) /* !!!! todo only Alarm or switch off the pump */ 
            /*~-*/
            {
               /*~T*/
               InitMotorStop( );
            /*~-*/
            }
            /*~O*/
            /*~-2*/
            else
            {
               /*~T*/
               InitMotorRun( );
            /*~-*/
            }
            /*~E*/
            /*~E*/
            /*~-*/
#endif
            /*~E*/
            /*~A*/
            /*~+:LIN_5ms*/
            /*~I*/
#ifdef def_LIN_5ms
            /*~I*/
            if ((read_eeprom_data(0x55)==0x55) || (RC5==0) )
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
                  Transmit_LIN_3Bytes( 0x2e, ( unsigned char )( ui16_Speed_mat ),
                                        ( unsigned char )( ui16_mat_Current ),
                                        ( unsigned char )( ui16_IPhase1_bldc.w ) );                       /** Transmit 3 byte with LIN enabled */

            /*~-*/
            }
            /*~E*/
            /*~-*/
#endif
            /*~E*/
            /*~E*/
         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~A*/
         /*~+:Task 100ms A Receive */
         /*~I*/
         if( ui8_Task_Cont100ms == 25 )
         /*~-*/
         {
            /*~T*/
            /** ## Task 100 ms at counter = 25 */
            /*~I*/
#ifdef def_LIN_Sync
            /*~T*/

            /*~O*/
            /*~-*/
#else
            /*~T*/
            SetDiagAlarm( ); /** The diagnostic alarm is checked every 100ms and set the PWM of ETAT_MJP */

            /*~T*/
            //DiagPicEtatMonitor();
            /*~I*/
#ifdef ver_ETAT
            /*~I*/
            if( ( PIC_ETAT_MONITOR == 1 && Port_PWMout == 0 ) || ( PIC_ETAT_MONITOR == 0 && Port_PWMout == 1 ) )
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
               PWM_Write_Out( 100 );

            /*~-*/
            }
            /*~E*/
            /*~O*/
            /*~-*/
#else
            /*~T*/
            //Error_PICetatMonitor = 0 ;

            /*~-*/
#endif
            /*~E*/
            /*~-*/
#endif
            /*~E*/
            /*~I*/
#ifdef def_LIN
            /*~I*/
            if ((read_eeprom_data(0x55)==0x55) || (RC5 == 0))
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
               //def_DebugPort0 = 1;
               Receive_Diag(0x11); /* Receive the data from the EOL tester */
               //def_DebugPort0 = 0;

            /*~-*/
            }
            /*~E*/
            /*~T*/
            //Receive_ETAT_PADD();
            /*~-*/
#endif
            /*~E*/
         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~A*/
         /*~+:Task 100ms A# Receive -- EOL*/
         /*~I*/
         if( ui8_Task_Cont100ms == 37 )
         /*~-*/
         {
            /*~I*/
#ifdef def_LIN
            /*~T*/
            /** ## Task 100 ms at counter = 37 */
            /*~I*/
            if ((read_eeprom_data(0x55)==0x55) || (RC5==0) )
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
               //def_DebugPort0 = 1;
               EOL(); /** Process to the EOL test and calibration */
               //def_DebugPort0 = 0;
            /*~-*/
            }
            /*~E*/
            /*~T*/

            /*~-*/
#endif
            /*~E*/
         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~A*/
         /*~+:Task 100ms B Send Status 0x2a */
         /*~I*/
         if( ui8_Task_Cont100ms == 50 )
         /*~-*/
         {
            /*~I*/
#ifdef def_LIN_status
            /*~I*/
            if ((read_eeprom_data(0x55)==0x55) || (RC5==0) )
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
               /** ## Task 100 ms at counter = 50 */
               /*~T*/
               Transmit_LIN_8Bytes( 0x2a, ( unsigned char )( ui8_error_Flags.b ),
                                        ( unsigned char )( ui8_Ki_mat ),
                                        ( unsigned char )( ui8_Kp_mat ),
                                        ( unsigned char )( ui16_speed_fil ),
                                        ( unsigned char )( MotorFlags.b ),
                                        ( unsigned char )( ui16_Speed_demand_mat ),
                                        ( unsigned char )( ui8_PWMoutvalue ),
                                        ( unsigned char )( Error_PICetatMonitor ) );
               /** Transmit 8 bytes by LIN with the ID 0x2a: \n
               * Error Flags\n
               * Debug counter\n
               * PIC_ETAT_MONITOR (RA6)\n
               * Motor speed\n
               * Motor flags\n
               * Motor speed demand\n
               * ETAT_MJP DC\n
               * Error_PICetat_Monitor */
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
         /*~+:Task 100ms C Send Status 0x2b */
         /*~I*/
         if( ui8_Task_Cont100ms == 75 )
         /*~-*/
         {
            /*~I*/
#ifdef def_LIN_status
            /*~I*/
            if ((read_eeprom_data(0x55)==0x55) || (RC5==0) )
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
               /** ## Task 100 ms at counter = 75 */
               /*~T*/
               Transmit_LIN_8Bytes( 0x2b, ( unsigned char )( PR2 ),
                                        ( unsigned char )( ui8_b_DResServID_c ),
                                        ( unsigned char )( ui16_PWM_Freq_In>>8 ),
                                        ( unsigned char )( ui8_duty_cycle_BLDC ),
                                        ( unsigned char )( ui8_PWM_dc_mat ),
                                        ( unsigned char )( ui16_NTC_Temp_bldc_mean>>2 ),
                                        ( unsigned char )( ui16_CPU_Temp_bldc_mean_cal>>2 ),
                                        ( unsigned char )( ui16_mat_inpTemp>>2 ) );
               /** Transmit 8 bytes by LIN with the ID 0x2b: \n
               * Frequency of PWM input motor\n
               * SID for EOL\n
               * Input frequency (1:256)\n
               * Motor DC\n
               * Input DC\n
               * NTC temperature (1:4)\n
               * CPU temperature (1:4)\n
               * Input temperature of Matlab layer (1:4) */
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
         /*~+:Task 100ms D Send Status 0x2c */
         /*~I*/
         if( ui8_Task_Cont100ms > 100 )
         /*~-*/
         {
            /*~T*/
            /** ## Task 100 ms at counter = 100 */
            /*~T*/
            ui8_Task_Cont100ms = 0; /** Reset the task counter */ 
            /*~I*/
#ifdef def_LIN_status
            /*~I*/
            if ((read_eeprom_data(0x55)==0x55) || (RC5==0) )
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
               Transmit_LIN_8Bytes( 0x2c, ( unsigned char )( ui16_mat_Current ),
                                        ( unsigned char )( ui16_UPhase_bldc ),
                                        ( unsigned char )( ui16_IPhase1_bldc.w ),
                                        ( unsigned char )( ui16_IPhase2_bldc.w ),
                                        ( unsigned char )( ui16_IPhase3_bldc.w ),
                                        ( unsigned char )( ui8_current_cal[0] ),
                                        ( unsigned char )( ui8_current_cal[1] ),
                                        ( unsigned char )( ui8_current_cal[2] ));
               /** Transmit 8 bytes by LIN with the ID 0x2c: \n
               * Current input of Matlab layer (1:4)\n
               * Filtered input voltage (1:4)\n
               * Input phase1 current (1:4)\n
               * Input phase2 current (1:4)\n
               * Input phase3 current (1:4)\n
               * Input voltage (1:4)\n
               * BEMF voltage (1:4)\n
               * Filtered input phase current (1:4)*/
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
         /*~+:Overshooting duration*/
         /*~I*/
#ifdef def_overshooting
         /*~I*/
         if( ui16_Task_Cont500ms < def_overshooting )
         /*~-*/
         {
            /*~T*/
            ui16_Task_Cont500ms++; /** Increment counter for task 500 ms */ 
         /*~-*/
         }
         /*~O*/
         /*~-2*/
         else
         {
            /*~T*/
            bool_start_demand_mat = 0;
         /*~-*/
         }
         /*~E*/
         /*~O*/
         /*~-*/
#else
         /*~T*/
         bool_start_demand_mat = 0;
         /*~-*/
#endif
         /*~E*/
         /*~E*/
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
   /*~-*/
   }
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
