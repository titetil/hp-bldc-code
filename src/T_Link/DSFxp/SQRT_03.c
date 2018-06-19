/******************************************************************************
*                                                                              
* FILE:                                                                        
*   SQRT_03.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: SQRT_03.c $ $Revision: 6 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $                           
******************************************************************************/



#include "dsfxp.h"

/******************************************************************************
*
* FUNCTION:
*   F__U8SQRTU32(ar)
*   
* DESCRIPTION:
*   Calculates an integer square root from argument 'ar'. The result 'rslt' is 
*   the largest integer whose square is less than or equal 'ar'.
*
* PARAMETERS:
*   type    name    meaning                                			
*   IInt32  ar      input value
*
* RETURNS:
*   Int8           return value    rslt = sqrt(ar) 
*
* NOTE:
*                         
*   
* HISTORY:
*
******************************************************************************/
UInt8 F__U8SQRTU32(UInt32 ar)                                                
{     
    UInt8 rslt;                            /* result of sqrt                 */
                      
    C__U8SQRTU32(ar, rslt);
    return rslt;
}
/* END F__U8SQRTU32() */


