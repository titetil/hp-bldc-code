/******************************************************************************
*                                                                              
* FILE:                                                                        
*   SQRT_06.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: SQRT_18.c $ $Revision: 4 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $                           
******************************************************************************/



#include "dsfxp.h"

/******************************************************************************
*
* FUNCTION:
*   F__I16SQRTU16(ar)
*   
* DESCRIPTION:
*   Calculates an integer square root from argument 'ar'. The result 'rslt' is 
*   the largest integer whose square is less than or equal 'ar'.
*
* PARAMETERS:
*   type    name    meaning                                			
*   UInt16  ar      input value
*
* RETURNS:
*   Int16           return value    rslt = sqrt(ar) 
*
* NOTE:
*                         
*   
* HISTORY:
*
******************************************************************************/
Int16 F__I16SQRTU16(UInt16 ar)                                                
{     
    Int16 result;                           /* result of sqrt                 */
                      
    C__I16SQRTU16(ar, result);
    return result;
}
/* END F__I16SQRTU16() */


