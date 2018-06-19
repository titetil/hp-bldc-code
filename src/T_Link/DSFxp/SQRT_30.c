/******************************************************************************
*                                                                              
* FILE:                                                                        
*   SQRT_04.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: SQRT_30.c $ $Revision: 4 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $                           
******************************************************************************/



#include "dsfxp.h"

/******************************************************************************
*
* FUNCTION:
*   F__U8SQRTI16_SAT(ar)
*   
* DESCRIPTION:
*   Calculates an integer square root from argument 'ar'. The result 'rslt' is 
*   the largest integer whose square is less than or equal 'ar'.
*
* PARAMETERS:
*   type    name    meaning                                			
*   Int16   ar      input value
*
* RETURNS:
*   UInt8           return value    rslt = sqrt(ar) 
*
* NOTE:
*                         
*   
* HISTORY:
*
******************************************************************************/
UInt8 F__U8SQRTI16_SAT(Int16 ar)
{     
    UInt8 result;                           /* result of sqrt                 */
                      
    C__U8SQRTI16_SAT(ar, result);
    return result;
}
/* END F__U8SQRTI16_SAT() */


