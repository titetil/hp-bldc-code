/******************************************************************************
*                                                                              
* FILE:                                                                        
*   MUL64_05.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: MUL64_05.c $ $Revision: 5 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $                           
******************************************************************************/



#include "dsfxp.h"


/******************************************************************************
*
* FUNCTION:
*   void F__U64MULU32U32(UInt32 f1, UInt32 f2, UInt32 *r_h, UInt32 *r_l)
*
* DESCRIPTION:
*   Multiplies f1 by f2 with 64bit result.
*
* PARAMETERS:
*   type    name    meaning    
*   UInt32    f1    1. factor
*   UInt32    f2    2. factor
*   UInt32  *r_h    64bit - result (high-part)
*   UInt32  *r_l    64bit - result (low-part)                                                                        
*
* RETURNS:
*
* NOTE:
* f1                          * f2 = 
* (f1H*2^16 + f1L)            * (f2H*2^16 + f2L) = 				
* f1H*f2H*2^32 + f1H*f2L*2^16 + f1L*f2H*2^16 + f1L*f2L				
*									
* shifted by  64    48    32	16    0 Bits
* |-----|-----|-----|-----|-----|-----|
*             |    f1H*f2H|     |     |
*                   |    f1H*f2L|     |
*                   |    f1L*f2H|     |
*                         |    f1L*f2L|
* |-----|-----|-----|-----|-----|-----|
*             |<   r_h   >|<   r_l   >|
*
*   
*
* HISTORY:
*
******************************************************************************/
void F__U64MULU32U32(UInt32 f1, UInt32 f2, UInt32 *r_h, UInt32 *r_l)
{
    UInt16  f1L,                            /* low-part of f1    */
            f2L,                            /* low-part of f2    */
            f1H,                            /* high-part of f1   */
            f2H;                            /* high-part of f2   */
    UInt32  temp,                            
            f1Lf2L,                          
            f1Hf2L,     
            f1Hf2H;

    f1L = (UInt16)f1;
    f2L = (UInt16)f2;
    f1H = (UInt16) ( f1 >> 16 );
    f2H = (UInt16) ( f2 >> 16 );

    f1Lf2L = (UInt32)f1L * (UInt32)f2L;    /* convenient multiplication */
    f1Hf2L = (UInt32)f1H * (UInt32)f2L;
    f1Hf2H = (UInt32)f1H * (UInt32)f2H;
                                /* add the pieces           */
    temp   = f1Hf2L + (f1Lf2L>>16);
    if(temp < f1Hf2L) f1Hf2H += 0x10000;     /* high-part + carry        */    
                            /* mid = temp + f1Lf2H         */
    f1Hf2L = temp + (UInt32)f1L * (UInt32)f2H; 
    if(f1Hf2L < temp) f1Hf2H += 0x10000;    /* high-part + carry        */
                                                    
    *r_l  = (f1Hf2L<<16) + (f1Lf2L & 0x0000FFFF);   /* low of f1Hf2L    */     
    *r_h  = f1Hf2H + (f1Hf2L>>16);            
}/*END F__U64MULU32U32()*/


