/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/** 
* \defgroup  ADC       Analog Conversion
* \brief     ADC module for BVH2.
* This module is initialising the ADC register and reading the analog value coming into AN ports. It filters also these values
* \version 1.0 - 2013-02-15
* \section adc_hist History  
* <b>V1.0 - 2013-02-15 - ADQ - First numbered version</b>
* -     This version is related to uC PIC16F1936    
* 
*/
/**
 * \ingroup   ADC
 * \file          adc.c
 * \brief         ADC configuration for reading of analog values
 * \note                Target CPU        : PIC16F1936\n
 *                      Compiler          : HI_TECH PICC (v9.81)
 * \copyright (c) TI Automotive - All rights reserved
 * \author        TI Automotive - JPL - Joachim Plantholt
 * \author        TI Automotive - ADQ - Aymeric de Quatrebarbes
 * \requirement \b TIR-SW-ADC-004 Supply Voltage Detection (U_batt)
 * \requirement \b TIR-SW-ADC-005 I Phase detection
 * \requirement \b TIR-SW-ADC-006 CPU Temperature indicator
 * \requirement \b TIR-SW-ADC-007 Phase Voltage Detection
 * \requirement \b TIR-SW-ADC-008 I Phase calibration
 * \requirement \b TIR-SW-ADC-009 CPU Temperature Indicator calibration
 * \requirement \b TIR-SW-ADC-010 NTC voltage detection
 * \test \b TC_TIR-SW-ADC-004 Supply voltage detection
 * \test \b TC_TIR-SW-ADC-005 I phase detection
 * \test \b TC_TIR-SW-ADC-006 CPU Temperature indicator
 * \test \b TC_TIR-SW-ADC-007 Phase voltage detection
 * \test \b TC_TIR-SW-ADC-008 I phase calibration
 * \test \b TC_TIR-SW-ADC-009 CPU temperature indicator calibration
 * \test \b TC_TIR-SW-ADC-010 NTC voltage detection
 * \details
 **/
/*~T*/
/*----------------      Includes                           --------------------------------------------------------------*/
#include         "project.h"
#include         "adc.h"
#include         "bldc.h"
#include         "eeprom.h"
#include         "lin.h"
#include         <htc.h>
/*~T*/
/*----------------      Defines -----------------------------------------------------------------------------*/
#define         WINDOW_SIZE      8/**< Define the size for the mean filters */ 
#define         filterWindow /**< Enable the mean filter for Ubat */\

/*~T*/
/*----------------      globale Variables          --------------------------------------------------------------*/
       volatile  _u_wb    analog_value; /**< Structure for the storage of ADC value during reading of Analog inputs */
       unsigned      int  inputArray1[ WINDOW_SIZE ]; /**< Table for the storage of the Ubat values for the filter */
       unsigned      int  ui16_fir_Bat_mittel = 0; /**< Battery Voltage after filter */
       _u_wb              ui16_fir_IPhase_mean = 0; /**< Phase current value after filter */
       unsigned      int  ui16_fir_UPhase_mittel = 0; /**< Phase Voltage after filter */
       unsigned      int  ui16_NTC_Temp_bldc_mean = 0; /**< Temperature of NTC after filter */
       unsigned      int  ui16_CPU_Temp_bldc_mean = 0; /**< Temperature of CPU after filter */
       unsigned      char windowPtr1; /**< Counter for the Ubat filter */
       unsigned      int  filter1 = 0; /**< Intermediate value for the Ubat filter */
       unsigned      int  filter3 = 0; /**< Intermediate value for the IPhase filter */
       unsigned      int  filterPh = 0; /**< Intermediate value for the UPhase filter */
       unsigned      int  filterTempNTC = 0; /**< Intermediate value for the NTC value filter */
       unsigned      int  filterTempCPU = 0; /**< Intermediate value for the temperature indicator filter */
       unsigned      int  ui16_CPU_Temp_bldc_mean_cal; /**< Filtered and calibrated value for the temperature indicator */
       unsigned      int  ui16_NTC_Temp_bldc_mean_cal; /**< Filtered and calibrated value for the NTC temperature. */
       unsigned      char ui8_temp_calibration; /**< calibration value for temperature */

extern unsigned      int  ui16_Temp_cal;
extern unsigned      int  ui16_I_cal_Ph1;

/*~T*/
/*----------------      Prototypes                         --------------------------------------------------------------*/
void ADC_Init( void  );
void ADC_Wait( void  );
void Get_Analog_Value( void  );
unsigned  int  ADC_Read( void  );
void FILTER_UPhase( void      );
void FILTER_Ubat( void  );
void FILTER_IPhase( void      );
void FILTER_Temp( void  );
void FILTER_Init( void  );
/*~A*/
/*~+:void ADC_Init(void)*/
/*~T*/
/**
 * \fn          void ADC_Init(void)
 * \brief       Initialization of the ADC ports
 * \pre         None
 * \post        None
 * \variable    #ui8_temp_calibration
 * \details Select the Frequenz Fosc/32=1MHz, Right justified result\n
 *                      ADC Clock Period Tad=1us, The conversion time for 10bits is 10*tad+2*tad\n
 *                      Select ADC port chanel 0 AN0\n
 *                      Enable ADC conversion module\n
 *                      Enable internal temperature sensor - Vout = VDD-4VT  (High Range)\n
 *                      Initialise the CPU temperature indicator ADC value to 0
 **/
/*~F*/
void ADC_Init( void  )
/*~-*/
{
   /*~T*/
   ADCON1 = 0b10100000; /* Select the Frequenz Fosc/32=1MHz, Right justified result
                         * ADC Clock Period Tad=1us, The conversion time for 10bits is 10*tad+2*tad */
   ADCON0 = 0b00000000; /* Select ADC port chanel 0 AN0 */
   ADON   = 1; /* Enable ADC conversion module */
   FVRCON = FVRCON | 0b00110000; /* Enable internal temperature sensor */
   ui8_temp_calibration = 0;/* Initialisation of the CPU temperature indicator ADC value */ 
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void ADC_Wait(void)*/
/*~T*/
/**
 * \fn      void ADC_Wait(void)
 * \brief   Wait for 3us
 * \pre     None
 * \post    None
 * \details This function waits for 3us. Only NOP() are used (this function is understood by the compiler). 
 *          Each NOP() is waiting for 125ns. To wait 3us, this function must use 24 NOP().
 **/
/*~F*/
void ADC_Wait( void  )
/*~-*/
{
   /*~T*/
   /* Each NOP() is waiting for 125ns */
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
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:unsigned int ADC_Read(void)*/
/*~T*/
/**
 * \fn      unsigned int ADC_Read(void)
 * \brief   Read the value in ADC register
 * \return  #analog_value
 * \pre     None
 * \post    None
 * \details Store the ADC value in #analog_value structure from registers ADRESH and ADRESL
 **/
/*~F*/
unsigned  int ADC_Read( void  )
/*~-*/
{
   /*~T*/
   unsigned  char i;
   i = 0;
   /*~L*/
   while( GO_nDONE )
   /*~-*/
   {
      /*~T*/
      i++;

      /*~I*/
      if( i > 64 )
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
   /*~T*/
   analog_value.b.hi = ADRESH;/* Get the 2 bits MSB result */
   analog_value.b.lo = ADRESL;/* Get the 8 bits LSB result */ 
   /*~T*/
   return analog_value.w;
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void FILTER_Init(void)*/
/*~T*/
/**
 * \fn      void FILTER_Init(void)
 * \brief   Initialisation of the ADC filter values.
 * \pre     None
 * \post    None
 * \see #windowPtr1
 * \see #WINDOW_SIZE 
 * \see #Batte_Volt_Channel
 * \see #ui16_Ubat_bldc
 * \see #inputArray1
 * \details This function initialises the output of the Ubat filter to 11V (ADC value = 350). This issue is to accelerate the starting of the pump after a reset.
 *          If this initialisation is not done, the pump is starting after 200ms after reset, which is the time to grow to the start voltage detection at the ADC channel of the microcontroller.
 **/
/*~F*/
void FILTER_Init( void  )
/*~-*/
{
   /*~L*/
   for( windowPtr1 = 0; windowPtr1 < WINDOW_SIZE; windowPtr1++ )
   /*~-*/
   {
      /*~T*/
      mADC_ChanSelect( Batte_Volt_Channel );
      ADC_Wait( );
      ui16_Ubat_bldc.w          = ADC_Read( );
      inputArray1[ windowPtr1 ] = 350; /* Init values in the filter are 11V */ 
   /*~-*/
   }
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void FILTER_Ubat(void)         Calculation ui16_fir_Bat_mittel*/
/*~T*/
/**
 * \fn          void FILTER_Ubat(void)
 * \brief       Filtering the Ubat input
 * \pre         None
 * \post        None
 * \see         #filterWindow
 * \see         #inputArray1
 * \see         #windowPtr1
 * \see         #ui16_Ubat_bldc
 * \see         #filter1
 * \see         #ui16_fir_Bat_mittel
 * \details  This filter stores 8 consecutive ADC value from analog input Battery Voltage and calcul the mean filter.\n
 *                      The macro #filterWindow must be activated to filter the BatteryVoltage analog value.\n
 *                      Else, the mean filter is not implement and a numeric filter of order 64 is implemented which is really more capacitive.
 **/
/*~F*/
void FILTER_Ubat( void  )
/*~-*/
{
   /*~I*/
#ifdef filterWindow
   /*~T*/
   /* Mean filter on 8 values */
   inputArray1[ windowPtr1 ] = ui16_Ubat_bldc.w;
   filter1 = inputArray1[ 0 ] + inputArray1[ 1 ] + inputArray1[ 2 ] + inputArray1[ 3 ] + inputArray1[ 4 ] + inputArray1[ 5 ] + inputArray1[ 6 ] + inputArray1[ 7 ];
   ui16_fir_Bat_mittel = filter1>>3;
   /*~I*/
   if( ++windowPtr1 >= WINDOW_SIZE )
   /*~-*/
   {
      /*~T*/
      windowPtr1 = 0;
      filter1    = 0;
   /*~-*/
   }
   /*~E*/
   /*~O*/
   /*~-*/
#else
   /*~T*/
   /* Very capacitive filter */
   filter1             = filter1 + ui16_Ubat_bldc.w;
   ui16_fir_Bat_mittel = filter1>>6;
   filter1             = filter1 - ui16_fir_Bat_mittel;
   /*~-*/
#endif
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void FILTER_UPhase(void)       Calculation ui16_fir_UPhase_mittel*/
/*~T*/
/**
 * \fn      void FILTER_UPhase(void)
 * \brief   Filtering the Uphase input
 * \pre     None
 * \post    None
 * \see     #filter_en
 * \see     #filterPh
 * \see     #ui16_UPhase_bldc
 * \see     #ui16_fir_UPhase_mittel
 * \details This function is filtering the Uphase ADC value to be more stable. Both input and output are linked to this variable. 
 *          This numeric filter is order 64, and is relative slow.\n
 *          The filter is activated only if the #filter_en is called, else, there is no filter and the used value in the system is 
 *          the value read by the ADC.
 */
/*~F*/
void FILTER_UPhase( void  )
/*~-*/
{
   /*~I*/
#ifdef filter_en
   /*~T*/
   filterPh               = filterPh + ui16_UPhase_bldc;
   ui16_fir_UPhase_mittel = filterPh>>6;
   filterPh               = filterPh - ui16_fir_UPhase_mittel;
   /*~O*/
   /*~-*/
#else
   /*~T*/
   ui16_fir_UPhase_mittel = ui16_UPhase_bldc;
   /*~-*/
#endif
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void FILTER_IPhase(void)       Calculation ui16_fir_IPhase_mean*/
/*~T*/
/**
 * \fn      void FILTER_IPhase(void)
 * \brief   Filtering the I_Phase input
 * \pre     None
 * \post    None
 * \see     #filter_en
 * \see     #filter3
 * \see     #ui16_IPhase_bldc
 * \see     #ui16_fir_IPhase_mean
 * \details This function is filtering the Iphase ADC value to be more stable. Both input and output are linked to this variable. 
 *          This numeric filter is order 64, and is relative slow.\n
 *          The filter is activated only if the #filter_en is called, else, there is no filter and the used value in the system is 
 *          the value read by the ADC.
 **/

/*~F*/
void FILTER_IPhase( void  )
/*~-*/
{
   /*~I*/
#ifdef filter_en
   /*~T*/
   filter3                = filter3 + ui16_IPhase1_bldc.w;
   ui16_fir_IPhase_mean.w = filter3>>6;
   filter3                = filter3 - ui16_fir_IPhase_mean.w;

   /*~O*/
   /*~-*/
#else
   /*~T*/
   ui16_fir_IPhase_mean.w = (ui16_IPhase1_bldc.w + ui16_IPhase2_bldc.w + ui16_IPhase3_bldc.w)/3;
   /*~-*/
#endif
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void FILTER_Temp(void)         Calculation ui16_NTC_Temp_bldc_mean*/
/*~T*/
/**
 * \fn      void FILTER_Temp(void)
 * \brief   Filtering the Temperature values
 * \pre     None
 * \post    None
 * \see     #filter_en
 * \see     #filterTempNTC
 * \see     #ui16_NTC_Temp_bldc
 * \see     #ui16_NTC_Temp_bldc_mean
 * \see     #filterTempCPU
 * \see     #ui16_Temp_cal
 * \details This function is filtering the Temperature ADC values to be more stable. Both input and output are linked to this variable. 
 *          This numeric filter is order 64, and is relative slow.\n
 *          Both temperature sensor (NTC and CPU temperature indicator) are filtered with the filter. \n
 *          The final filtered value is offset with the temperature calibration value done in the EOL calibration.\n
 *          The filter is activated only if the #filter_en is called.
 **/
/*~F*/
void FILTER_Temp( void  )
/*~-*/
{
   /*~I*/
#ifdef filter_en
   /*~T*/
   /* NTC temp filter */
   filterTempNTC           = filterTempNTC + ui16_NTC_Temp_bldc;
   ui16_NTC_Temp_bldc_mean = filterTempNTC>>6;
   filterTempNTC           = filterTempNTC - ui16_NTC_Temp_bldc_mean;

   /* CPU temp filter. Don't used from V0.9.3 */
   /* filterTempCPU           = filterTempCPU + ui16_CPU_Temp_bldc;
   ui16_CPU_Temp_bldc_mean = filterTempCPU>>6;
   filterTempCPU           = filterTempCPU - ui16_CPU_Temp_bldc_mean;*/
   /*~O*/
   /*~-*/
#else
   /*~T*/
   ui16_NTC_Temp_bldc_mean = ui16_NTC_Temp_bldc;
   /* ui16_CPU_Temp_bldc_mean = ui16_CPU_Temp_bldc; */
   /*~-*/
#endif
   /*~E*/
   /*~T*/
   /* ui16_CPU_Temp_bldc_mean_cal = ui16_CPU_Temp_bldc_mean + ui16_Temp_cal; */
   ui16_NTC_Temp_bldc_mean_cal = ui16_NTC_Temp_bldc_mean + ui16_Temp_cal;
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void Get_Analog_Value(void)*/
/*~T*/
/**
 * \fn      void Get_Analog_Value(void)
 * \brief   Call all filtered values
 * \pre     None
 * \post    None
 * \details This function calls all wanted filters.
 **/
/*~F*/
void Get_Analog_Value( void  )
/*~-*/
{
   /*~T*/
   FILTER_Ubat( ); /** Call the input voltage filter */
   FILTER_IPhase( ); /** Call the current phases filter */
   FILTER_Temp( ); /** Call the temperature filter */ 
/*~-*/
}
/*~E*/
/*~E*/
