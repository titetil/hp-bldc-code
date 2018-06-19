/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/**
* \ingroup   ADC
* \file      adc.h
* \brief     Header file for ADC
* \note      Target CPU    : PIC16F1936\n
*            Compiler      : HI_TECH PICC (v9.81)           
* \copyright (c) TI Automotive - All rights reserved 
* \author    TI Automotive - JPL - Joachim Plantholt
* \author    TI Automotive - MMK - Mohamed Moussaddak        
* \author    TI Automotive - ADQ - Aymeric de Quatrebarbes  
* \details
**/
/*~I*/
#ifndef _ADC_H_
/*~T*/
#define     _ADC_H_     
/************************ global functions **************************/
extern            void ADC_Init( void  );
extern  unsigned  int  ADC_Read( void  );
extern            void ADC_Wait( void  );
extern  signed    int  NTC_Temperature( void  );
extern            void FILTER1( unsigned char  ui8_channel );
extern            void FILTER2( unsigned char  ui8_channel );
extern            void Get_Analog_Value( void  );
extern            void FILTER_Init( void  );
/************************extern variable **************************/
extern  unsigned  int  ui16_fir_Bat_mittel;
extern  _u_wb          ui16_fir_IPhase_mean;
extern  unsigned  int  ui16_CPU_Temp_bldc_mean;
extern  unsigned  int  ui16_NTC_Temp_bldc_mean;
extern  unsigned  int  ui16_CPU_Temp_bldc_mean_cal;
extern  unsigned  int  ui16_NTC_Temp_bldc_mean_cal;
extern  unsigned  char ui8_IPhase_sel;
extern  unsigned  int  filter1;
/****************************Defines*********************************/
#define     ADC0_Config                 0x01 /**< Value for selecting the ADC channel */ 
#define     NTC_Temper_Channel          ( 0x0B << 2 ) | ADC0_Config /**< ADC channel for NTC */ 
#define     Batte_Volt_Channel          ( 0x0C << 2 ) | ADC0_Config /**< ADC channel for Battery Voltage */ 
#define     Ph1_ISen_Channel            ( 0x02 << 2 ) | ADC0_Config /**< ADC channel for Ph1 Current Sensor */ 
#define     Ph2_ISen_Channel            ( 0x0D << 2 ) | ADC0_Config /**< ADC channel for Ph2 Current Sensor */ 
#define     Ph3_ISen_Channel            ( 0x04 << 2 ) | ADC0_Config /**< ADC channel for Ph3 Current Sensor */ 
#define     Ph1_ISel                    0x01 /**< Phase1 current selection value */ 
#define     Ph2_ISel                    0x02 /**< Phase2 current selection value */ 
#define     Ph3_ISel                    0x03 /**< Phase3 current selection value */ 
#define     Ph1_Motor_Channel           ( 0x00 << 2 ) | ADC0_Config /**< ADC channel for Ph1 Voltage sensor */ 
#define     Ph2_Motor_Channel           ( 0x01 << 2 ) | ADC0_Config /**< ADC channel for Ph2 Voltage sensor */ 
#define     Ph3_Motor_Channel           ( 0x09 << 2 ) | ADC0_Config /**< ADC channel for Ph3 Voltage sensor */ 
#define     CPU_Temp_Channel            ( 0x1D << 2 ) | ADC0_Config /**< Virtual ADC channel for CPU temperature indicator */ 
#define     Filter_Size                 8 /**< Filter size */ 
/*********************    Macro for ADC ******************************/
#define     mADC_ChanSelect( a )       ( ADCON0 = a ) /**< Macro which select the ADC channel */ 
#define     mADC_TrigConversion( )     ( GO_nDONE = 1 ) /**< Macro which trigger the conversion of the value on the selected channel */ 
/*~-*/
#endif
/*~E*/
