/******************************************************************************
*                                                                              
* FILE:                                                                        
*   SQRT_02.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: SQRT_23.c $ $Revision: 4 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $                           
******************************************************************************/



#include "dsfxp.h"

/******************************************************************************
*
* FUNCTION:
*   F__I8SQRTI32(ar)
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
*   Int8          return value    rslt = sqrt(ar) 
*
* NOTE:
*                         
*
*   
* HISTORY:
*
******************************************************************************/
Int8 F__I8SQRTI32(Int32 ar)                                                
{     
    Int8 rslt;                            /* result of sqrt                 */
                      
    C__I8SQRTI32(ar, rslt);
    return rslt;
}
/* END F__I8SQRTI32() */


