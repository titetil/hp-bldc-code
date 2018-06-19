/******************************************************************************
*                                                                              
* FILE:                                                                        
*   SQRT_03.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: SQRT_28.c $ $Revision: 4 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $                           
******************************************************************************/



#include "dsfxp.h"

/******************************************************************************
*
* FUNCTION:
*   F__I32SQRTI32(ar)
*   
* DESCRIPTION:
*   Calculates an integer square root from argument 'ar'. The result 'rslt' is 
*   the largest integer whose square is less than or equal 'ar'.
*
* PARAMETERS:
*   type    name    meaning                                			
*   Int32   ar      input value
*
* RETURNS:
*   Int32          return value    rslt = sqrt(ar) 
*
* NOTE:
*                         
*   
* HISTORY:
*
******************************************************************************/
Int32 F__I32SQRTI32(Int32 ar)                                                
{     
    Int32 result;                           /* result of sqrt                 */
                      
    C__I32SQRTI32(ar, result);
    return result;
}
/* END F__I32SQRTI32() */


