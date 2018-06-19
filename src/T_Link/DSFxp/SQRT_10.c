/******************************************************************************
*                                                                              
* FILE:                                                                        
*   SQRT_01.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: SQRT_10.c $ $Revision: 4 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $                           
******************************************************************************/



#include "dsfxp.h"

/******************************************************************************
*
* FUNCTION:
*   F__U16SQRTI32(ar)
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
*   UInt16          return value    rslt = sqrt(ar) 
*
* NOTE:
*                         
*   
* HISTORY:
*
******************************************************************************/
UInt16 F__U16SQRTI32(Int32 ar)                                                
{     
    UInt16 rslt;                            /* result of sqrt                 */
                      
    C__U16SQRTI32(ar, rslt);
    return rslt;
}
/* END F__U16SQRTI32() */







