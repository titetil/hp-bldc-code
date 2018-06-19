/******************************************************************************
*                                                                              
* FILE:                                                                        
*   DIV64_08.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: DIV64_08.c $ $Revision: 5 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $                           
******************************************************************************/



#include "dsfxp.h"


/******************************************************************************
*
* FUNCTION:
*   F__U32DIVU64U32(n_h, n_l, d)
*
* DESCRIPTION:
*   Divides the 64bit Value n by the 32bit Value d with a 32bit result.
*
* PARAMETERS:
*   type    name    meaning    
*   UInt32   n_h    upper 32Bit of the nominator
*   UInt32   n_l    lower 32Bit of the nominator
*   UInt32     d    denominator 
*
* RETURNS:
*   UInt32     result
*
* NOTE:
*  - "division by zero" detection (gives 0x7FFFFFFF respectively 0xFFFFFFFF)
*  - the algorithm supports only calculations with unsigned values. Therefore
*    the sign of the result is determined separately.
*
*
* HISTORY:
*
******************************************************************************/
UInt32 F__U32DIVU64U32(UInt32 n_h, UInt32 n_l, UInt32 d)
{														                        
	UInt64s tmp;										                        
	tmp.lo = n_l;										                        
	tmp.hi = n_h;										                        
	return( F__U32DIVU64sU32(&tmp, d) );
}/* END F__U32DIVU64U32() */


