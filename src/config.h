/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/**
* \defgroup  CONTROL   Matlab Layer
* \defgroup  BLDC      Motor algorithm
*
* \file      config.h
* \brief     Header for software configuration
*
* \copyright (c) TI Automotive - All rights reserved 
*
* \note      Target CPU    : PIC16F1936\n
*            Compiler      : HI_TECH PICC (v9.81)       
*      
* \author    TI Automotive - JPL - Joachim Plantholt        
* \author    TI Automotive - ADQ - Aymeric de Quatrebarbes  
* 
* \details
* \htmlonly 
* <table border cellspacing=0> <caption><b>H I S T O R Y</b>?</CAPTION> 
*  <tr> <th width=90> Date <br> yy-mm-dd </th>  <th width=90> Version </th>  <th width=90> Author </th> <th width=300> Description </th> </tr> 
*  <tr> <td> 10-11-17 </td>  <td> 0.00.01 </td>  <td> JPL </td>  <td> created</td> </tr> 
*  <tr> <td> 11-10-28 </td>  <td> 0.01.00 </td>  <td> JPL </td>  <td> fixed first version </td> </tr> 
*  <tr> <td> 12-06-19 </td>  <td> 0.09.00 </td>  <td> ADQ </td>  <td> EOL implemented</td>  </tr> 
* </table>
* \endhtmlonly 
**/
/*Date    Version    Author  Description
yy-mm-dd                        
10.11.17      0.00.01 JPL     created
11.10.28      0.01.00 JPL     fixed first version
12.06.19      0.09.00 ADQ     EOL implemented
*/
/*~A*/
/*~+:History of the project*/
/*~T*/
/** \mainpage
\section soft_hist SOFTWARE HISTORY

<b>V0.9.2 Release 131500. Checksum 0x68F4</b>
-       LIN disabled by command

<b>V0.9.0 Release 131000. Checksum 0x750E</b>
-       New Start algorithm after breaking impellers
-       First SW adapted for C samples ( current ramp / temperature recognition )

<b>V0.9. Release 130700. Checksum 0x7D95</b>
-       Current calibration implemented to be done after each power reset.
-       checksum verification after reset implemented
-       Current limitation for high protection OK
-       RC5 corresponding on CS/WAKE on the LIN driver set to 0 after EOL sequence. It shuts down the LIN communication.
-       Overshooting at start removed
-       LIN communication with status.
-       Misra checked not fully

<b>V0.8.8.1 checksum 0xF69D</b>
-       overshooting of 300ms at start
-       Freeze overshooting in code
-       in bldc.c, change the starting speed read by Matlab to 10000rpm to have no more loss in pressure and speed as demand
-       in config.h, definition with #define def_overshooting of an overshooting in starting phase of the pump to 300ms, DIAG restored by disabling def_LIN_Sync
-       in lin.c, remove the overshooting set by lin
-       in main.c, set the overshooting as #ifdef, and send by Lin the 3 values DCout, DCout_mat, speed

<b>V0.8.8.0 checksum 0x0A3B</b>
-       no overshooting

<b>V0.8.7.3 For pressure built up in chalons from v0.8.7 checksum 0x5051</b>
-       Rising of PWM output regulated by motor. When overshooting is finished after time define by LIN, the output PWM is comming from ControlLoop from Matlab.

<b>V0.8.7.2 for pressure build up tests - Checksum 0x18A1</b>
-       Diag output remove and replace by a LIN Synchronisation trigger. This trigger is 1 during 11ms when PWM input > 10%.

<b>V0.8.7.1 for pressure build up tests - Checksum 0x330E</b>
-       documentation with doxygen
-       addition of a input parameter in Lin5ms.
-       Kp/Ki, fixed_start_speed, fixed_start_speed_demand set in code instead of Matlab
-       Routine which can select the start speed for some ms before the correct speed demand.
-       Time where the maximum speed for pressure built up is selected by lin
-       Speed (real speed /100) and output dc are sended by Lin5ms.
-       Lin enabled
-       Alarm removed at low voltage. Set only when the pump is stopped because of undervoltage.
-       UE_Mini hysteresis change (8.0 -> 8.5V instead of 8.0 -> 9.3V)
-       Current Alarm set at 7A input current
-       Dry running repaired.

<b>V0.8.7</b>
-       Temperature alarm set at 110C instead of 95C. Restored the alarm at 85C
-       Filter Ubat changed to mean_filter.
-       Add a function for the initialisation of the filter
-       Change the routine Current_Analysis_High in Matlab layer after reset because of an alarm set automatically 500ms after reset
-       Change the frequency of the filter from 5ms to 1ms
-       Change of answer at 2% DC of CDE_MJP. It was stopped, it is fix to pump at PFH4

<b>V0.8.6 checksum 0xBF44</b>
-       LIN disable,
-       RA7, RC5, RC6, RC7, RB6, RB7 as output and low level
-       Temperature alarm set at 95C and stored at 85C and Motor Phase DC set to 10kHz when alarm occurs
-       Low voltage shutting down at 5,6 and restart at 6,2V
-       Phase Motor DC set to 16kHz

<b>V0.8.5 checksum 0x58A9 For EMC test at Flextronics.</b>
-       LIN disable,
-       RA7, RC5, RC6, RC7, RB6, RB7 as output and low level
-       Temperature alarm set at 95C and stored at 85C
-       Low voltage shutting down at 5,6 and restart at 6,2V

<b>V0.8.2.3 checksum 0xCEDD For Temperature tests at Flextronics</b>
-       Output PWM from 20kHz to 10kHz at 90C
-       Output PWM from 10kHz to 20kHz at 70C
-       PWM motor frequency changes to 10kHz with high temperature
-       verification of ETAT_MJP implemented
-       PWM command by CDE_MJP again
-       Wobbling enable
-       Iphase filter corrected

<b>V0.8.2.2 checksum 0x2449</b> 
-       For EMC tests
-       PWM input fixed to 25%
-       PWM input of the pump set to 120

<b>V0.8.2.1 checksum 0x4314</b>
-       For EMC tests
-       PWM input fixed to 25%

<b>V0.8.2 checksum 0x65c8</b>
-       For EMC tests
-       LIN_5ms implemented 
-       define output for def_LIN
-       LIN disabled
-       Input filters enabled
-       Lin disabled Wobble PWM input motor  
-       Not used output forced to 0 

<b>V0.8.1 BVH2 SW Release R121900 checksum 0x03d8</b>
-       change of speed table (from 3800rpm to 7300rpm)

<b>V0.8  BVH2 SW Release R121700 checksum 0x0aa3</b>
-       PWM control of the pump command by CDE_MJP

<b>V0.7.9 Release R121701 checksum 0xe7d1</b> 
-       PWM control of the pump forced to 100%
-       For Temperature Tests for FLextronics 
-       StartupPWM = 20% instead of 40% 
-       InitMotorRun at every start of the pump 
-       def_FilterPWMtoPowerStage active
-       Pump never switched off because of high temperature 
-       Change of threshold value for NTC 
-       Change of LIN enable timing init 
-       Change of speed table 
-       Update of diagnositics signals for bad signal in CMD_MJP 
-       Change in init routine with init PWM to 0% instead of 40% 
-       Reactive the NTC sensor for B2.1 sample

<b>V0.7 BVH2 BLDC R120700 Checksum 0xdb5d</b> 
-       repair the watchdog it wasn't really enabled before
-       introduce a compiler variable to switch the direction of Pump
-       introduce a variable to invert the PWM input  
-       prepare the BLDC module to run forward backwards
-       fix a problem with some special characters in the code
-       remove some unused variables
-       remove some variants in the BLDC for better overview
-       Unstable PWM output fixed
-       Filter PWM out removed (former version only allow 1 digit change for each commutation step)
-       Change back the characteristic to 4500 to 6500 rpm .
-       linear relation between PWM input and speed demand 
-       LIN disabled because it generates a rough motor run  
 
<b>V0.5 R113900</b> 
-       Version for Eric with changed characteristic Checksum 0xc94c   
-       Version for validating pump speed with extra speed table 
-       not for costumer 
-       Change only debug output checksum 
-       improvement of phase angle to compensate snubber impact  
-       Improve of timing bldc.c. 
-       The hardware supports 30% minimum pwm for Motor 

<b>V0.4   R113000</b> 
-       checksum Microchip 0xf926
-       switch LIN off  
-       Speed at 95% reduced to 6500rpm PWM input Characteristic
-       Change the speed detection to a 8 stage filter
-       Control algorithm control on new bigger value (better resolution)
-       repair a bug in the range adaption internal value 180 (4400rpm) and 124 (6450rpm) is used
-       Speed limit for PWM 11-15% is now concerning spec i.e. limited to PFH1 formerly this was linear  
-       enable filter PWM to power stage for inhibit problem with occasionally rise of pump speed
-       speed closed loop 
-       This version should solve the issues that pump occasionally stalls without recognition of the software. 
-       intermediate check in 
-       change the structure to configure Loop control outside of Matlab 
-       Change to 2ms blanking time and auto Demag detection  
-       First version with improved commutation behavior 
-       one root cause for dissymmetry is changed (advanced angle calculation)   
-       The delay of sample the comparator is delayed. This is important because there is an overshoot possible
-       Root cause are the snubbers (MOSFET Power stage)
-       sample BEMF with fixed delay first final version for Release 112200
-       sample BEMF depends on the duty cycle of the PWM signal 
-       first version with speed control (not final characteristic)  
  
<b>V0.0.7 R111900 BVH2_SW_PCU</b> 
-       This version was be optimized for high speed 550 l/h for DHD test  
-       the mayority filter is bypassed 
-       automatic demag recognition 
-       intermediate check in after optimize the software for high current appl. 
-       Mayority Filter removed i.e. made optional  

<b>V0.0.6 SW Release 111500</b>   
-       Software Version for Chalons for Motor  
 
<b>V0.0.5 SW Release 111300</b>   
-       Software Version for yellow board samples first check in 
-       Integrate Matlab 

<b>V0.0.4 Software</b> 
-       Version for yellow board samples first check in 
-       Integrate Matlab 

<b>V0.0.3 Software Version after integration at the 11.03.2011</b> 

<b>V0.0.2 Software Version after integration at the 03.03.2011</b> 
-       ADC complete  
-       PWM complete  
-       timer complete  
-       Check in for Version 0.01  
-       Clkout is defined with a compiler option (define) in hardware version 0.6 this is define as Ubat sense  In Version V0.7 RA6 Pin 10  is defined as Debug out and might be used to supply system clock to debug connector. 
-       Update ADC

<b>V0.0.1 </b>  
-       Check in after integrate Motor and PWM (in and out) 


*/
/*~E*/
/*~I*/
#ifndef CONFIG_H

/*~T*/
#define     CONFIG_H                      
/*~T*/
//#define def_Fastdemag            /**< Left in the design but actually not used after implementing the */
#define     def_EnableCurrentLimit /**< Not enabled for high current test samples. \warning To be disable for A sample  */
//#define def_MayorityFilter       /**< Actually there are not evidence for the improvement */
//#define def_re_initState         /**< Reinit the State when demag is finished */
//#define def_forced_comm_mode     /**< Only for test purposes */

//#define def_fixed_PWM    200     /**< Define a fixed output PWM value which is 100% */
//#define def_fixed_PWM    100*5/4     /**< Define a fixed output PWM value which is 100% */

//#define def_update_PhaseAngle    /**< removed from project */
//#define     def_filterPWMtoPowerStage /**< \warning To be disable for A samples */
#define     def_Auto_Demag
//#define     def_OpenLoopMode         /**< if defined the Control Loop is disabled an  Open loop is configured */
//#define def_patchUbatSens            /* only for old hardware to be able to patch the Battery voltage sensing */
/*~T*/
extern const unsigned short PWM_trans_table[202] = {
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* 0-4% */
181, 181, 181, 181, 181, 181, 181, 181, 181, 181, /* 5-9 */
181, 182, 184, 185, 186, 187, 189, 190, 191, 193, 194, 195, 196, 198, 199, 200, 202, 203, 204, 205,
207, 208, 209, 211, 212, 213, 214, 216, 217, 218, 220, 221, 222, 224, 225, 226, 227, 229, 230, 231,
233, 234, 235, 236, 238, 239, 240, 242, 243, 244, 245, 247, 248, 249, 251, 252, 253, 254, 256, 257,
258, 260, 261, 262, 263, 265, 266, 267, 269, 270, 271, 272, 274, 275, 276, 278, 279, 280, 281, 283,
284, 285, 287, 288, 289, 290, 292, 293, 294, 296, 297, 298, 300, 301, 302, 303, 305, 306, 307, 309,
310, 311, 312, 314, 315, 316, 318, 319, 320, 321, 323, 324, 325, 327, 328, 329, 330, 332, 333, 334,
336, 337, 338, 339, 341, 342, 343, 345, 346, 347, 348, 350, 351, 352, 354, 355, 356, 357, 359, 360,
361, 363, 364, 365, 367, 368, 369, 370, 372, 373, 374, 376, 377, 378, 379, 381, 382, 383, 385, 386,
387, 388, 390, 391, 392, 394, 395, 396, 397, 399, 400, 400,
400, 400, 400, 400, 400, 400, 400, 400, 400, 400 /* 96-100 */
}; /**< Assign to a PWM input value a speed for the pump */

#define     def_adr_SoftwareVersion     0xF9 /**< Defines the first address in the EEPROM of the SW version number */ 
#define     ui8_SoftwareVersion_MSB     0x02 /**< 0xXY describes the version number X.Y */ 
#define     ui8_SoftwareVersion_LSB     0x00 /**< This byte precises the version number. */ 
/*~T*/
#define     def_ZeroPhaseDelay /**< \warning Be carefull when disabling ZeroPhaseDelay, because current alternative algorithm could cause dissymetrie. Actually there is a relation to the snubber which causes dissymetrie. The snubber causes an overshoot when pwm is switched. When not symetrical value are selected for rising and falling edge then this can compensate the dissymetrie */

/*~T*/
//#define def_SW_PSA_YBS            /**< this option enables the software version for the yellowboard samples.Then any commutation due to a zerocrossing detection is inhibited. Furthermore only a valid PWM diagnsotic signal is asserted to the output */
/*~T*/
//#define ver_ETAT                  /**< #Etat_PIC_Monitor verification */
/*~T*/
//#define def_EnableTempErrorSwitch /**< Switches off when temperature exceed the maximum Temp */
//#define     HIGH_TEMPERATURE10kHz /**< Modification of the frequency of the pump at high temperature to reduce this temperature to 10kHz */

/*~T*/
//#define def_overshooting 300  /**< Overshooting at the starting of the pump of 300ms */
/*~T*/
//#define     def_LIN /**< Be careful generates a high interrupt load */
//#define  def_LIN_Sync             /**< Define for synchronisation between LIN interface and oscilloscope. */
//#define  def_LIN_5ms              /**< LIN enable for 5 ms task */
//#define  def_LIN_status             /**< LIN debugger?*/
//#define  def_Simulate_Fail        /**< Command the diag outputs with LIN */

/*~T*/
#define def_EOL_oneTest /** Define for testing only one by one the EOL tests */
/*~T*/
//#define def_wobb_                 /**< frequency woobling */
#define     adap_DC_wobb /**< DC of PWM motor wobbling */
/*~T*/
#define     filter_en /**< filter enabled for Maltab layer */
/*~T*/
#define     def_PWMinInvert /**< Input of PWM input is inverted at the uC Input */
//#define def_forward               /**< This switch controlls the direction of pump. It is introduced with the introduction of Hardware V10492/4 */

/*~-*/
#endif
/*~E*/
