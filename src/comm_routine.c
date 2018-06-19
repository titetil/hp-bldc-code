/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/*!--------------------------------------------------------------------------------------------------------
  Project Name  : BVH2  
  File Name     : comm_routine.c
  Module        : LIN COMMUNICATION 
  Description   : LIN communication for EOL tests  
  Target CPU    : PIC16
  Compiler      : tbd 

  Copyright (c)  TI Automotive     All rights reserved.

 Initials     Name                      Company
 --------     ---------------------     ------------------------------------
 AQS          Aymeric de Quatrebarbes   TI Automotive   

 
 H I S T O R Y

 Date       Version  Author  Description
 yy-mm-dd   
 12-04-04   0.00.01  aqs     created
  
-------------------------------------------------------------------------------------------------------- */


/*~T*/
/*----------------  Includes               --------------------------------------------------------------*/
#include "project.h"
#include "config.h"
#include "eeprom.h"
#include "lin.h"
#include "bldc.h"
#include "T_Link/BVH2_Appl_Layer.h"
#include "comm_routine.h"
#include <htc.h>

#ifdef EOL_TEST

/*----------------  globale Variables      --------------------------------------------------------------*/
BYTE my_msg[8];

unsigned char ICT_STAMP_PRESENCE ;
unsigned char FCT_STAMP_ABSENCE ;
unsigned char TEMPERATURE_CALIBRATION ;
unsigned char VER_SW_VERSION_NUMBER ;
unsigned char VER_SUPPLY_VOLTAGE_VALUE ;
unsigned char CURRENT_CALIBRATION ;
unsigned char SPEED_READING ;
unsigned char FCT_STAMP_WRITING ;
unsigned char LIN_DEACTIVATION ;
unsigned char FLAG_OK ;
unsigned char FLAG_NOK ;
unsigned char TEMPERATURE_WRITE ;


/*----------------  Prototypes             --------------------------------------------------------------*/



/*~A*/
/*~+:char ICT_stamp_verification(void)*/
/*~K*/
/*~+:/~*!------------------------------------------------------------------------------------------------------*~/*/
/*~+:/~*! @name       ICT_stamp_verification                                                              *~/*/
/*~+:/~*! @PreCondition:    None                                                                                *~/*/
/*~+:/~*! @param[in]           None                                                                                *~/*/
/*~+:/~*! @param[out]          ICT_stamp: 0 if ICT_stamp is present, else 1                                                                        *~/*/
/*~+:/~*! @bug    None                                                                                *~/*/
/*~+:/~*! @details          Verify if stamp for ICT is written in EEPROM (in address EB to F3)*/
/*~+:*                  ICT_stamp = 0 if stamp is written, else ICT_stamp = 1                               *~/*/
/*~+:/~*!                                                                                                      *~/*/
/*~+:/~*! @warning            None                                                                                *~/*/
/*~+:/~*!------------------------------------------------------------------------------------------------------*~/*/
/*~+:*/
/*~+:*/
/*~F*/
char ICT_stamp_verification(void)
/*~-*/
{
   /*~T*/
    /* The stamp is written in BDC code also 0xFF is never reached during the writing of the stamp. The routine do a detection of 0xFF in EEPROM, which is the default value, in the addresses covered for ICT_stamp (addresses 0xEB to 0xFE). */

   char ICT_stamp = 0;

   //! Increment ICT_stamp every time the data in address is 0xFF (correspond to not written)

   /*~L*/
   for(int i = 0xEB; i <= 0xF3; i++)
   /*~-*/
   {
      /*~I*/
      if (read_eeprom_data(i) == 0xFF)
      /*~-*/
      {
         /*~T*/
         ICT_stamp++;

      /*~-*/
      }
      /*~E*/
   /*~-*/
   }
   /*~E*/
   /*~T*/

   /*! Convert ICT_temp to bit value
    *    - 1 if ICT_stamp not written
    *    - 0 if ICT_stamp is present  */
      
   /*~I*/
   if (ICT_stamp >= 1)
   /*~-*/
   {
      /*~T*/
      ICT_stamp = 1;
   /*~-*/
   }
   /*~E*/
   /*~T*/
   return ICT_stamp;

/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:char FCT_stamp_verification(void)*/
/*~+:*/
/*~K*/
/*~+:*/
/*~+:/~*!------------------------------------------------------------------------------------------------------*~/*/
/*~+:/~*! @name       FCT_stamp_verification                                                              *~/*/
/*~+:/~*! PreCondition:    None                                                                                *~/*/
/*~+:/~*! Input:           None                                                                                *~/*/
/*~+:/~*! Output:          FCT_stamp: 0 if ICT_stamp is present, else 1                                                                           *~/*/
/*~+:/~*! Side Effects:    None                                                                                *~/*/
/*~+:/~*! Overview:        Verify if stamp for FCT is written in EEPROM (in address F4 to F8)                  *~/*/
/*~+:/~*!                  FCT_stamp = 0 if stamp is written, else FCT_stamp = 1                               *~/*/
/*~+:/~*!                                                                                                      *~/*/
/*~+:/~*! Note:            None                                                                                *~/*/
/*~+:/~*!------------------------------------------------------------------------------------------------------*~/*/
/*~+:*/
/*~F*/
char FCT_stamp_verification(void)

/*~-*/
{
   /*~T*/
       /* The stamp is written in BDC code also 0xFF is never reached during the writing of the stamp. 
        * The routine do a detection of 0xFF in EEPROM, which is the default value, in the addresses covered for
        * FCT_stamp (addresses 0xF4 to 0xF8). */

   char FCT_stamp = 0;

       //! Increment FCT_stamp every time the data in address is 0xFF (correspond to not written)


   /*~L*/
   for (int i = 0xF4; i <= 0xF8; i++)
   /*~-*/
   {
      /*~I*/
      if (read_eeprom_data(i) == 0xFF)
      /*~-*/
      {
         /*~T*/
         FCT_stamp++;

      /*~-*/
      }
      /*~E*/
   /*~-*/
   }
   /*~E*/
   /*~T*/

       /*! Convert FCT_temp to bit value
        *    - 1 if FCT_stamp not written
        *    - 0 if FCT_stamp is present
        *    - 2 if FCT_stamp not fully present  */
     
   /*~I*/
   if (FCT_stamp == 5)
   /*~-*/
   {
      /*~T*/
      FCT_stamp = 1;

   /*~-*/
   }
   /*~O*/
   /*~-2*/
   else
   {
      /*~I*/
      if (FCT_stamp == 0)
      /*~-*/
      {
         /*~T*/
         FCT_stamp = 0;

      /*~-*/
      }
      /*~O*/
      /*~-2*/
      else
      {
         /*~T*/
         FCT_stamp = 2;
      /*~-*/
      }
      /*~E*/
   /*~-*/
   }
   /*~E*/
   /*~T*/
   return FCT_stamp;
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:unsigned int read_SW_version(void)*/
/*~K*/
/*~+:/~*!------------------------------------------------------------------------------------------------------*~/*/
/*~+:/~*! name      read_SW_version                                                                     *~/*/
/*~+:/~*! PreCondition:    None                                                                                *~/*/
/*~+:/~*! Input:           None                                                                                *~/*/
/*~+:/~*! Output:          SW_version                                                                          *~/*/
/*~+:/~*! Side Effects:    None                                                                                *~/*/
/*~+:/~*! Overview:        Read the software version from the EEPROM                                           *~/*/
/*~+:/~*!                                                                                                      *~/*/
/*~+:/~*! Note:            None                                                                                *~/*/
/*~+:/~*!------------------------------------------------------------------------------------------------------*~/*/
/*~+:*/
/*~F*/
unsigned int read_SW_version(void)

/*~-*/
{
   /*~T*/
    unsigned int SW_version;
       //! SW_version MSB in address F9
       //! SW version LSB in address FA
       SW_version = ( (read_eeprom_data(0xF9) << 8) | (read_eeprom_data(0xFA)) );

       return SW_version;

/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void write_FCT_stamp(Prod_Year_FCT, Prod_Day_FCT_MSB, Prod_Day_FCT_LSB, Prod_Num_FCT_MSB, Prod_Num_FCT_LSB)*/
/*~K*/
/*~+:/~*------------------------------------------------------------------------------------------------------*~/*/
/*~+:/~* Function:        write_FCT_stamp                                                                     *~/*/
/*~+:/~* PreCondition:    None                                                                                *~/*/
/*~+:/~* Input:           None                                                                                *~/*/
/*~+:/~* Output:          None                                                                                *~/*/
/*~+:/~* Side Effects:    None                                                                                *~/*/
/*~+:/~* Overview:        Write FCT stamp in EEPROM                                                           *~/*/
/*~+:/~*                                                                                                      *~/*/
/*~+:/~* Note:            None                                                                                *~/*/
/*~+:/~*------------------------------------------------------------------------------------------------------*~/*/
/*~+:*/
/*~F*/
void write_FCT_stamp(unsigned char Prod_Year_FCT,     // Year 20xx
                     unsigned char Prod_Day_FCT_MSB,  // Day 0x | -- 
                     unsigned char Prod_Day_FCT_LSB,  // Day 0- | xx 
                     unsigned char Prod_Num_FCT_MSB,  // Test xx | -- 
                     unsigned char Prod_Num_FCT_LSB)  // Test -- | xx
/*~-*/
{
   /*~K*/
   /*~+:write_eeprom_data(0xF4, Prod_Year_FCT) ;    // Write the last 2 digits of the year into EEPROM*/
   /*~+:write_eeprom_data(0xF5, Prod_Day_FCT_MSB) ; // Write the first 2 digits of the day of the year into EEPROM*/
   /*~+:write_eeprom_data(0xF6, Prod_Day_FCT_LSB) ; // Write the last 2 digits of the day of the year into EEPROM*/
   /*~+:write_eeprom_data(0xF7, Prod_Num_FCT_MSB) ; // Write the first 2 digits of the test number of the day into EEPROM*/
   /*~+:write_eeprom_data(0xF8, Prod_Num_FCT_LSB) ; // Write the last 2 digits of the test number of the day into EEPROM*/
   /*~+:*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void LIN_deactivation(void)*/
/*~+:*/
/*~K*/
/*~+:/~*------------------------------------------------------------------------------------------------------*~/*/
/*~+:/~* Function:        LIN_deactivation                                                                    *~/*/
/*~+:/~* PreCondition:    None                                                                                *~/*/
/*~+:/~* Input:           None                                                                                *~/*/
/*~+:/~* Output:          None                                                                                *~/*/
/*~+:/~* Side Effects:    None                                                                                *~/*/
/*~+:/~* Overview:        Deactivate the LIN with printing a flag into the EEPROM  */
/*~+: *                      - LIN disabled if LIN flag set to 1                 */
/*~+: *                      - LIN enabled if LIN flag set to 0                                              *~/*/
/*~+:/~*                                                                                                      *~/*/
/*~+:/~* Note:            None                                                                                *~/*/
/*~+:/~*------------------------------------------------------------------------------------------------------*~/*/
/*~+:*/
/*~F*/
void LIN_deactivation(void)

/*~-*/
{
   /*~I*/
    if (read_eeprom_data(0x10) != 0)    // LIN flag = 1 => LIN disabled

   /*~-*/
   {
      /*~T*/
      write_eeprom_data(0x10, 0);
#undef def_LIN

   /*~-*/
   }
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void Current_Calibration(void)*/
/*~+:*/
/*~K*/
/*~+:*/
/*~F*/
void Current_Calibration(void)
/*~-*/
{
   /*~T*/

/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void Init_EEPROM(void)*/
/*~+:*/
/*~K*/
/*~+:/~*------------------------------------------------------------------------------------------------------*~/*/
/*~+:/~* Function:        Init EEPROM                                                                         *~/*/
/*~+:/~* PreCondition:    None                                                                                *~/*/
/*~+:/~* Input:           None                                                                                *~/*/
/*~+:/~* Output:          None                                                                                *~/*/
/*~+:/~* Side Effects:    None                                                                                *~/*/
/*~+:/~* Overview:        Init EEPROM to FF                                                                   *~/*/
/*~+:/~*                                                                                                      *~/*/
/*~+:/~* Note:            None                                                                                *~/*/
/*~+:/~*------------------------------------------------------------------------------------------------------*~/*/
/*~+:*/
/*~F*/
void Init_EEPROM(void)

/*~-*/
{
   /*~L*/
   for (int i = 0x00; i < 0xFF; i++)
   /*~-*/
   {
      /*~T*/
       write_eeprom_data(i, 0xFF);

   /*~-*/
   }
   /*~E*/
   /*~T*/

/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void Confirm(void)*/
/*~F*/
void Confirm(void)
/*~-*/
{
   /*~T*/
   Receive_Diag(0x3b);

   /*~L*/
   while (FLAG_OK != 0 && FLAG_NOK != 0)
   /*~-*/
   {
      /*~T*/
      Receive_Diag(0x3b);
      /*~I*/
      if (FLAG_OK == 0 && FLAG_NOK == 0)
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
   /*~L*/
   while (FLAG_OK == 0 && FLAG_NOK == 0)
   /*~-*/
   {
      /*~T*/
      Receive_Diag(0x3b);
      /*~I*/
      if (FLAG_NOK == 1)
      /*~-*/
      {
         /*~T*/
         LIN_deactivation();
         break;
      /*~-*/
      }
      /*~O*/
      /*~-2*/
      else
      {
         /*~I*/
         if (FLAG_OK == 1)
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
   /*~-*/
   }
   /*~E*/
/*~-*/
}
/*~E*/
/*~E*/
/*~A*/
/*~+:void EOL_TEST_FCT(void)*/
/*~K*/
/*~+:///----------------------------------------------------*/
/*~+:/// Routine for the EOL tests. Dialog with LIN*/
/*~+:///----------------------------------------------------*/
/*~+:*/
/*~F*/
void EOL_TEST_FCT(void)

/*~-*/
{
   /*~T*/
   // ------------------------------ VARIABLES ------------------------------------
   char ICT_stamp_ver ;
   char FCT_stamp_ver ;
   char Temperature_MSB ;
   char Temperature_LSB ;
   char SW_version_LSB ;
   char SW_version_MSB ;
   char Voltage_MSB ;
   char Voltage_LSB ;
   char Speed_MSB ;
   char Speed_LSB ;
   char Current_MSB ;
   char Current_LSB ;
   /*~I*/
   if (1)//if init and read_eeprom_data(0x10) == 1; 
   /*~-*/
   {
      /*~I*/
      if (1) // if tester present 
      /*~-*/
      {
         /*~A*/
         /*~+:DETECTION OF ICT STAMP PRESENCE*/
         /*~T*/
         /// -----------------------------------------------------------------------------
         ///  # 1. ---- DETECTION OF ICT STAMP PRESENCE ----
         /// ## 1.1. Send request to show that the ICT stamp is present.
         Receive_Diag(0x3b);

         /*~L*/
         while (ICT_STAMP_PRESENCE == 1)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (ICT_STAMP_PRESENCE == 0)
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
         /*~L*/
         while (ICT_STAMP_PRESENCE == 0)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (ICT_STAMP_PRESENCE == 1)
            /*~-*/
            {
               /*~T*/
               ICT_stamp_ver = ICT_stamp_verification();

               //* To complete to have the entire answer from tester:*/
               Transmit_LIN_8Bytes(0x3a,
                           0xf1, 0x90, ICT_stamp_ver, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);
               Confirm();
               break;


            /*~-*/
            }
            /*~E*/
         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~A*/
         /*~+:DETECTION OF FCT STAMP ABSENCE*/
         /*~T*/
         /// -----------------------------------------------------------------------------
         /// # 2. ---- DETECTION OF FCT STAMP ABSENCE ----
         /// ## 2.1. Send request to show that the FCT stamp is not present.

         Receive_Diag(0x3b);
         /*~L*/
         while (FCT_STAMP_ABSENCE == 1)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (FCT_STAMP_ABSENCE == 0)
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
         /*~L*/
         while (FCT_STAMP_ABSENCE == 0)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (FCT_STAMP_ABSENCE == 1)
            /*~-*/
            {
               /*~T*/
               FCT_stamp_ver = FCT_stamp_verification();
               Transmit_LIN_8Bytes(0x3a,
                           0xf1, 0x91, FCT_stamp_ver, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);


               /*~T*/
               Confirm();
               break;
            /*~-*/
            }
            /*~E*/
         /*~-*/
         }
         /*~E*/
         /*~T*/
            
         /*~E*/
         /*~A*/
         /*~+:TEMPERATURE CALIBRATION*/
         /*~T*/
         /// -----------------------------------------------------------------------------
         /// # 3. ---- TEMPERATURE CALIBRATION ----
         /// ## 3.1. Send the temperature detected with the internal temperature indicator.


         Receive_Diag(0x3b);
         /*~L*/
         while (TEMPERATURE_CALIBRATION == 1)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (TEMPERATURE_CALIBRATION == 0)
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
         /*~L*/
         while (TEMPERATURE_CALIBRATION == 0)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (TEMPERATURE_CALIBRATION == 1)
            /*~-*/
            {
               /*~T*/
               Temperature_MSB = (ui16_CPU_Temp_bldc>>8);  //* Convert the Temperature given by ADC in real temperature */
               Temperature_LSB = ui16_CPU_Temp_bldc ;  //* 2bits for MSB, 8 for LSB */
               Transmit_LIN_8Bytes(0x3a,
                            0xf1, 0x92, Temperature_MSB, Temperature_LSB, 0xFF, 0xFF, 0xFF, 0xFF);

               /*~T*/
               break;
            /*~-*/
            }
            /*~E*/
         /*~-*/
         }
         /*~E*/
         /*~T*/

         // Wait for the writing of the temperature_wrinting flag

         /*~L*/
         while (TEMPERATURE_WRITE == 1)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (TEMPERATURE_WRITE == 0)
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

         /// ## 3.3. Write the offset into the EEPROM at address 0x00 and 0x01.

         /*~L*/
         while (TEMPERATURE_WRITE== 0)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (TEMPERATURE_WRITE == 1)
            /*~-*/
            {
               /*~T*/
               write_eeprom_data(0x00, my_msg[0]); /// Write the temperature MSB calibration value at address 0x00
               write_eeprom_data(0x01, my_msg[1]); /// Write the temperature LSB calibration value at address 0x01

               /*~T*/
               break;
            /*~-*/
            }
            /*~E*/
         /*~-*/
         }
         /*~E*/
         /*~T*/


         /*~T*/
         /// ## 3.4. Send a confirmation that the temperature calibration value is written into the EEPROM (read the calibration value into EEPROM)
         //Transmit_LIN_8Bytes(0x3d, 0xb1, 0x91, read_eeprom_data(0x00), read_eeprom_data(0x01), 0xFF, 0xFF, 0xFF, 0xFF);


         /*~T*/
         /// ## 3.5. Receive the confirmation of the reading of the signal. 
         //Receive_Diag();


         /*~E*/
         /*~A*/
         /*~+:VERIFICATION OF SOFTWARE VERSION NUMBER*/
         /*~T*/
             // -----------------------------------------------------------------------------
             // 4. ---- VERIFICATION OF SOFTWARE VERSION NUMBER ----
             // 4.1. Send the software version written in EEPROM during ICT test (in address EB to F3).

         Receive_Diag(0x3b);
         /*~L*/
         while (VER_SW_VERSION_NUMBER == 1)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (VER_SW_VERSION_NUMBER == 0)
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
         /*~L*/
         while (VER_SW_VERSION_NUMBER == 0)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (VER_SW_VERSION_NUMBER == 1)
            /*~-*/
            {
               /*~T*/
               SW_version_LSB = read_SW_version();
               SW_version_MSB = (read_SW_version()>>8);
               Transmit_LIN_8Bytes(0x3a,
                          0xf1, 0x93, SW_version_MSB, SW_version_LSB, 0xFF, 0xFF, 0xFF, 0xFF );

               Confirm();

               break;
            /*~-*/
            }
            /*~E*/
         /*~-*/
         }
         /*~E*/
         /*~T*/

         /*~E*/
         /*~A*/
         /*~+:VERIFICATION OF SUPPLY VOLTAGE VALUE*/
         /*~T*/
         // -----------------------------------------------------------------------------
         // 5. ---- VERIFICATION OF SUPPLY VOLTAGE VALUE ----
         // 5.1. Send the supply voltage value to the tester.

         Receive_Diag(0x3b);


         /*~L*/
         while (VER_SUPPLY_VOLTAGE_VALUE == 1)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (VER_SUPPLY_VOLTAGE_VALUE == 0)
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


         /*~L*/
         while (VER_SUPPLY_VOLTAGE_VALUE == 0)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (VER_SUPPLY_VOLTAGE_VALUE == 1)
            /*~-*/
            {
               /*~T*/
               Voltage_MSB = ui16_Ubat_bldc.b.hi;
               Voltage_LSB = ui16_Ubat_bldc.b.lo;

               Transmit_LIN_8Bytes(0x3a,
                          0xf1, 0x94, Voltage_MSB, Voltage_LSB, 0xFF, 0xFF, 0xFF, 0xFF );

               Confirm();

               break;
            /*~-*/
            }
            /*~E*/
         /*~-*/
         }
         /*~E*/
         /*~T*/


         // 5.2. Receive the confirmation of the reading of the signal. 
         //Receive_Diag();


         /*~E*/
         /*~A*/
         /*~+:CURRENT CALIBRATION*/
         /*~T*/
             // -----------------------------------------------------------------------------
             // 6. ---- CURRENT CALIBRATION ----
             // 6.1. Send the current value of phase 1 to the tester.

         Receive_Diag(0x3b);
         /*~L*/
         while (CURRENT_CALIBRATION != 0)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (CURRENT_CALIBRATION == 0)
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
         /*~L*/
         while (CURRENT_CALIBRATION == 0)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (CURRENT_CALIBRATION != 0)
            /*~-*/
            {
               /*~A*/
               /*~+:CURRENT_CALIBRATION == 1*/
               /*~L*/
               while (CURRENT_CALIBRATION == 1)
               /*~-*/
               {
                  /*~T*/
                  Transmit_LIN_8Bytes(0x3a,
                             0xf1, 0x95, Current_MSB, Current_LSB, 0xFF, 0xFF, 0xFF, 0xFF );

                  Confirm();
                  // 7.2. Receive the confirmation of the reading of the signal. Abort if wrong answer.

                  // 7.3. Repeat the procedure 6 times (to working point 7).

               /*~-*/
               }
               /*~E*/
               /*~E*/
               /*~A*/
               /*~+:CURRENT_CALIBRATION == 2*/
               /*~L*/
               while (CURRENT_CALIBRATION == 2)
               /*~-*/
               {
                  /*~T*/
                  Transmit_LIN_8Bytes(0x3a,
                             0xf1, 0x96, Current_MSB, Current_LSB, 0xFF, 0xFF, 0xFF, 0xFF );

                  Confirm();
                  // 7.2. Receive the confirmation of the reading of the signal. Abort if wrong answer.

                  // 7.3. Repeat the procedure 6 times (to working point 7).

               /*~-*/
               }
               /*~E*/
               /*~E*/
               /*~A*/
               /*~+:CURRENT_CALIBRATION == 3*/
               /*~L*/
               while (CURRENT_CALIBRATION == 3)
               /*~-*/
               {
                  /*~T*/
                  Transmit_LIN_8Bytes(0x3a,
                             0xf1, 0x97, Current_MSB, Current_LSB, 0xFF, 0xFF, 0xFF, 0xFF );

                  // 7.2. Receive the confirmation of the reading of the signal. Abort if wrong answer.

                  // 7.3. Repeat the procedure 6 times (to working point 7).
                  Confirm();
                  break;
               /*~-*/
               }
               /*~E*/
               /*~E*/
            /*~-*/
            }
            /*~E*/
         /*~-*/
         }
         /*~E*/
         /*~T*/
            // 6.2. Receive the difference between detected current by internal sensor and real current (3A applied at 11.5V).

             // 6.3. Write the offset into the EEPROM at address 0x01.

             // 6.4. Send a confirmation that the current calibration value for phase 1 is written into the EEPROM.

             // 6.5. Receive the confirmation of the reading of the signal. Abort if wrong answer.

             // 6.6. Repeat the process of current calibration for phase2 and phase3.



         /*~E*/
         /*~A*/
         /*~+:READ THE SPEED WITH LIN*/
         /*~T*/
         // -----------------------------------------------------------------------------
         // 7. ---- READ THE SPEED WITH LIN ----


         /*~K*/
         /*~+:Working point 1: DC = 0%   ; PFH4     ; rpm*/
         /*~+:Working point 2: DC = 5%   ; Pump off ; rpm */
         /*~+:Working point 3: DC = 10%  ; PFH1     ; rpm */
         /*~+:Working point 4: DC = tbd  ; PFH2     ; rpm */
         /*~+:Working point 5: DC = tbd  ; PFH3     ; rpm*/
         /*~+:Working point 6: DC = 95%  ; PFH4     ; rpm */
         /*~+:Working point 7: DC = 100% ; PFH4     ; rpm */
         /*~+:*/
         /*~T*/
         // 7.1. Send the speed value with Lin for working point 1.

         Speed_LSB = ui16_speed_fil;
         Speed_MSB = (ui16_speed_fil>>8);

         Receive_Diag(0x3b);
         /*~L*/
         while (SPEED_READING != 0)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (SPEED_READING == 0)
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
         /*~L*/
         while (SPEED_READING == 0)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (SPEED_READING != 0)
            /*~-*/
            {
               /*~A*/
               /*~+:SPEED_READING == 1*/
               /*~L*/
               while (SPEED_READING == 1)
               /*~-*/
               {
                  /*~T*/
                  Transmit_LIN_8Bytes(0x3a,
                             0xf1, 0x98, Speed_MSB, Speed_LSB, 0xFF, 0xFF, 0xFF, 0xFF );
                  Confirm();

                  // 7.2. Receive the confirmation of the reading of the signal. Abort if wrong answer.

                  // 7.3. Repeat the procedure 6 times (to working point 7).

               /*~-*/
               }
               /*~E*/
               /*~E*/
               /*~A*/
               /*~+:SPEED_READING == 2*/
               /*~L*/
               while (SPEED_READING == 2)
               /*~-*/
               {
                  /*~T*/
                  Transmit_LIN_8Bytes(0x3a,
                             0xf1, 0x99, Speed_MSB, Speed_LSB, 0xFF, 0xFF, 0xFF, 0xFF );

                  Confirm();
                  // 7.2. Receive the confirmation of the reading of the signal. Abort if wrong answer.

                  // 7.3. Repeat the procedure 6 times (to working point 7).

               /*~-*/
               }
               /*~E*/
               /*~E*/
               /*~A*/
               /*~+:SPEED_READING == 3*/
               /*~L*/
               while (SPEED_READING == 3)
               /*~-*/
               {
                  /*~T*/
                  Transmit_LIN_8Bytes(0x3a,
                             0xf1, 0x9A, Speed_MSB, Speed_LSB, 0xFF, 0xFF, 0xFF, 0xFF );

                  Confirm();
                  // 7.2. Receive the confirmation of the reading of the signal. Abort if wrong answer.

                  // 7.3. Repeat the procedure 6 times (to working point 7).

               /*~-*/
               }
               /*~E*/
               /*~E*/
               /*~A*/
               /*~+:SPEED_READING == 4*/
               /*~L*/
               while (SPEED_READING == 4)
               /*~-*/
               {
                  /*~T*/
                  Transmit_LIN_8Bytes(0x3a,
                             0xf1, 0x9B, Speed_MSB, Speed_LSB, 0xFF, 0xFF, 0xFF, 0xFF );

                  Confirm();
                  // 7.2. Receive the confirmation of the reading of the signal. Abort if wrong answer.

                  // 7.3. Repeat the procedure 6 times (to working point 7).

               /*~-*/
               }
               /*~E*/
               /*~E*/
               /*~A*/
               /*~+:SPEED_READING == 5*/
               /*~L*/
               while (SPEED_READING == 5)
               /*~-*/
               {
                  /*~T*/
                  Transmit_LIN_8Bytes(0x3a,
                             0xf1, 0x9C, Speed_MSB, Speed_LSB, 0xFF, 0xFF, 0xFF, 0xFF );

                  Confirm();
                  // 7.2. Receive the confirmation of the reading of the signal. Abort if wrong answer.

                  // 7.3. Repeat the procedure 6 times (to working point 7).

               /*~-*/
               }
               /*~E*/
               /*~E*/
               /*~A*/
               /*~+:SPEED_READING == 6*/
               /*~L*/
               while (SPEED_READING == 6)
               /*~-*/
               {
                  /*~T*/
                  Transmit_LIN_8Bytes(0x3a,
                             0xf1, 0x9D, Speed_MSB, Speed_LSB, 0xFF, 0xFF, 0xFF, 0xFF );

                  Confirm();
                  // 7.2. Receive the confirmation of the reading of the signal. Abort if wrong answer.

                  // 7.3. Repeat the procedure 6 times (to working point 7).

               /*~-*/
               }
               /*~E*/
               /*~E*/
               /*~A*/
               /*~+:SPEED_READING == 7*/
               /*~L*/
               while (SPEED_READING == 7)
               /*~-*/
               {
                  /*~T*/
                  Transmit_LIN_8Bytes(0x3a,
                             0xf1, 0x9E, Speed_MSB, Speed_LSB, 0xFF, 0xFF, 0xFF, 0xFF );

                  Confirm();
                  // 7.2. Receive the confirmation of the reading of the signal. Abort if wrong answer.

                  // 7.3. Repeat the procedure 6 times (to working point 7).
                  break;
               /*~-*/
               }
               /*~E*/
               /*~E*/
            /*~-*/
            }
            /*~E*/
         /*~-*/
         }
         /*~E*/
         /*~T*/

         /*~T*/


         /*~T*/

         /*~E*/
         /*~A*/
         /*~+:WRITE FCT STAMP*/
         /*~T*/
         // -----------------------------------------------------------------------------
         // 8. ---- WRITE FCT STAMP ----

         unsigned char Prod_Year_FCT;     // Year 20xx
         unsigned char Prod_Day_FCT_MSB;  // Day 0x | -- 
         unsigned char Prod_Day_FCT_LSB;  // Day 0- | xx 
         unsigned char Prod_Num_FCT_MSB;  // Test xx | -- 
         unsigned char Prod_Num_FCT_LSB;

         Receive_Diag(0x3b);
         /*~L*/
         while (FCT_STAMP_WRITING == 1)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (FCT_STAMP_WRITING == 0)
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
         /*~L*/
         while (FCT_STAMP_WRITING == 0)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (FCT_STAMP_WRITING == 1)
            /*~-*/
            {
               /*~T*/
               Prod_Year_FCT = my_msg[3];
               Prod_Day_FCT_MSB = my_msg[4];
               Prod_Day_FCT_LSB = my_msg[5];
               Prod_Num_FCT_MSB = my_msg[6];
               Prod_Num_FCT_LSB = my_msg[7];

               write_FCT_stamp(Prod_Year_FCT,     // Year 20xx
                               Prod_Day_FCT_MSB,  // Day 0x | -- 
                               Prod_Day_FCT_LSB,  // Day 0- | xx 
                               Prod_Num_FCT_MSB,  // Test xx | -- 
                               Prod_Num_FCT_LSB);  // Test -- | xx

               break;
            /*~-*/
            }
            /*~E*/
         /*~-*/
         }
         /*~E*/
         /*~E*/
         /*~A*/
         /*~+:DEACTIVATION OF LIN*/
         /*~T*/
         // -----------------------------------------------------------------------------
         // 9. ---- DEACTIVATION OF LIN ----
         // 9.1. Send the request for the deactivation of LIN.

         Receive_Diag(0x3b);
         /*~L*/
         while (LIN_DEACTIVATION == 1)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (LIN_DEACTIVATION == 0)
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
         /*~L*/
         while (LIN_DEACTIVATION == 0)
         /*~-*/
         {
            /*~T*/
            Receive_Diag(0x3b);
            /*~I*/
            if (LIN_DEACTIVATION == 1)
            /*~-*/
            {
               /*~T*/
               LIN_deactivation();
               break;
            /*~-*/
            }
            /*~E*/
         /*~-*/
         }
         /*~E*/
         /*~T*/
         //#undef EOL_TEST
         /*~E*/
      /*~-*/
      }
      /*~O*/
      /*~-2*/
      else
      {
         /*~T*/
         // try to detect tester during 5 seconds
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
/*~T*/
#endif
