/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/**
* \ingroup   DIAG
* \file      diag.h
* \brief     Header file for diag task
*
* \copyright (c) TI Automotive - All rights reserved 
*
* \note      Target CPU    : PIC16F1936\n
*            Compiler      : HI_TECH PICC (v9.81)       
*      
* \author    TI Automotive - JPL - Joachim Plantholt        
* \author    TI Automotive - MMK - Mohamed Moussaddak
* \author    TI Automotive - ADQ - Aymeric de Quatrebarbes
* 
* \details
*/
/*~I*/
#ifndef _DIAG_H_
/*~T*/
#define     _DIAG_H_     


/*~T*/
extern  unsigned  int  ui16_PWM_Freq_In;

extern  unsigned  char Error_PICetatMonitor;/* Error for ETAT_MJP feedback */ 
extern  unsigned  char DC_pic_etat_monitor; /* Counter to have the DC of ETAT_MJP feedback */ 
/*~T*/
/************************ global functions **************************/
extern            void DiagInit( void  );
extern            void SetDiagAlarm( void  );
extern  unsigned  char read_eeprom_data( unsigned char  ui8_adress );
extern            void ReadCal_Value( void  );
extern            void DiagPicEtatMonitor( void  );
extern            void EOL( void  );
/****************************Defines*********************************/

/*~-*/
#endif
/*~E*/
/*~T*/
#define     Status_B0                       MotorFlags.bits.B0
#define     MOTOR_NOT_SYNC                  MotorFlags.bits.B1

#define     bool_PWMin_err_Flag             ui8_error_Flags.bits.B0
#define     bool_OverTemp_err_Flag          ui8_error_Flags.bits.B1
#define     bool_OverTemp_severe_Flag       ui8_error_Flags.bits.B7
#define     bool_OverCurr_err_Flag          ui8_error_Flags.bits.B2
#define     bool_Pump_err_Flag              ui8_error_Flags.bits.B3
#define     bool_DryRunning_err_Flag        ui8_error_Flags.bits.B4
#define     bool_PWMin_Freq_err_Flag        ui8_error_Flags.bits.B5
#define     bool_PowerStage_err_Flag        ui8_error_Flags.bits.B6

#define     PIC_ETAT_MONITOR                RA6 /**< PIC_ETAT_MONITOR set at pin RA6 of the uC */ 

/*~T*/
#define     eol_sequence_start              0x80
#define     ict_stamp_presence              0x90
#define     fct_stamp_absence               0x91
#define     temperature_calibration         0x92
#define     sw_version_number               0x93
#define     supply_voltage_verification     0x94
#define     current_calibration_ph1         0x95
#define     current_calibration_ph2         0x96
#define     current_calibration_ph3         0x97
#define     speed_wp                        0x98
#define     diagnostic_flags                0x9f
#define     fct_stamp_writing               0xa0
#define     eol_finished                    0xfa
/*~T*/
extern  _u_bits ui8_error_Flags;


