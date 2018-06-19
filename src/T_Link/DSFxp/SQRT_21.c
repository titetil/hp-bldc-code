/******************************************************************************
*                                                                              
* FILE:                                                                        
*   SQRT_03.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: SQRT_21.c $ $Revision: 4 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $                           
******************************************************************************/



#include "dsfxp.h"

/******************************************************************************
*
* FUNCTION:
*   F__I8SQRTI16(ar)
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
*   Int8            return value    rslt = sqrt(ar) 
*
* NOTE:
*                         
*   
* HISTORY:
*
******************************************************************************/
Int8 F__I8SQRTI16(Int16 ar)                                                
{     
    Int8 result;                           /* result of sqrt                 */
                      
    C__I8SQRTI16(ar, result);
    return result;
}
/* END F__I8SQRTI16() */


