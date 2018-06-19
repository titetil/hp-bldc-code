/*~-*/
/*~XSF_LANGUAGE: C/C++*/
/*~T*/
/**
* \ingroup   BVH2
* \file      project.h
* \brief     Header for structures
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
* <table border cellspacing=0> <caption><b>H I S T O R Y</b></CAPTION> 
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
/*~T*/
#include     "config.h"
/*~T*/
#define     def_DebugPort0     RA7 /**< Debug port = RA7 port in the uC */ 
/*~T*/
/** Structure which can define each bit of a char */
typedef  union
         {
             unsigned  char b;
             struct
             {
                 unsigned  B0 : 1;
                 unsigned  B1 : 1;
                 unsigned  B2 : 1;
                 unsigned  B3 : 1;
                 unsigned  B4 : 1;
                 unsigned  B5 : 1;
                 unsigned  B6 : 1;
                 unsigned  B7 : 1;

             }              bits;
         }      _u_bits;

/*~T*/
/** Structure which separate an int into two char (hi and lo) */
typedef  union
         {
             unsigned  int w;
             struct
             {
                 unsigned  char lo;
                 unsigned  char hi;
             }             b;
         }      _u_wb;

/*~T*/
/** Structure which separate a long into two int (hiw and low) */
typedef  union
         {
             unsigned  long  lng;
             struct
             {
                 _u_wb low;
                 _u_wb hiw;
             }               w;
         }      _u_lng;


