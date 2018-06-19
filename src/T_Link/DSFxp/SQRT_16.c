/******************************************************************************
*                                                                              
* FILE:                                                                        
*   SQRT_01.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: SQRT_16.c $ $Revision: 4 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $                           
******************************************************************************/



#include "dsfxp.h"

/******************************************************************************
*
* FUNCTION:
*   F__I16SQRTU32(ar)
*   
* DESCRIPTION:
*   Calculates an integer square root from argument 'ar'. The result 'rslt' is 
*   the largest integer whose square is less than or equal 'ar'.
*
* PARAMETERS:
*   type    name    meaning                                			
*   UInt32  ar      input value
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
Int16 F__I16SQRTU32(UInt32 ar)                                                
{     
    Int16 rslt;                            /* result of sqrt                 */
                      
    C__I16SQRTU32(ar, rslt);
    return rslt;
}
/* END F__I16SQRTU32() */







