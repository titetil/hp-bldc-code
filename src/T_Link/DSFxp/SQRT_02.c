/******************************************************************************
*                                                                              
* FILE:                                                                        
*   SQRT_02.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: SQRT_02.c $ $Revision: 6 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $                           
******************************************************************************/



#include "dsfxp.h"

/******************************************************************************
*
* FUNCTION:
*   F__U8SQRTU16(ar)
*   
* DESCRIPTION:
*   Calculates an integer square root from argument 'ar'. The result 'rslt' is 
*   the largest integer whose square is less than or equal 'ar'.
*   Use macro C__U8SQRTU16(ar, rslt).
*
* PARAMETERS:
*   type    name    meaning                                			
*   UInt16  ar      input value
*
* RETURNS:
*   UInt8          return value    rslt = sqrt(ar) 
*
* NOTE:
*                         
*   
* HISTORY:
*
******************************************************************************/
UInt8 F__U8SQRTU16(UInt16 ar)                                                
{     
    UInt8 rslt;                            /* result of sqrt                 */
                      
    C__U8SQRTU16(ar, rslt);
    return rslt;
}
/* END F__U8SQRTU16() */


