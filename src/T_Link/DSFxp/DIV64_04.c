/******************************************************************************
*                                                                              
* FILE:                                                                        
*   DIV64_04.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: DIV64_04.c $ $Revision: 5 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $                           
******************************************************************************/



#include "dsfxp.h"


/******************************************************************************
*
* FUNCTION:
*   F__U64DIVU64U32(n_h, n_l, d, r_h, r_l)
*
* DESCRIPTION:
*   Divides the 64bit Value n by the 32bit Value d with a 64bit result. 
*
* PARAMETERS:
*   type    name    meaning    
*  UInt32    n_h    upper 32Bit of the nominator
*  UInt32    n_l    lower 32Bit of the nominator
*  UInt32      d    denominator 
*  UInt32    r_h    upper 32bit of the result
*  UInt32    r_l    lower 32bit of the result
*
* RETURNS:
*
* NOTE:
*  - "division by zero" detection (gives 0x7FFFFFFFFFFFFFFF respectively 
*    0xFFFFFFFFFFFFFFFF)
*  - the algorithm supports only calculations with unsigned values. Therefore
*    the sign of the result is determined separately.
*
*
* HISTORY:
*
******************************************************************************/
void F__U64DIVU64U32(UInt32 n_h, UInt32 n_l, UInt32 d, UInt32 *r_h, UInt32 *r_l)
{														                        
	UInt64s tmp;										                        
	tmp.lo = n_l;										                        
	tmp.hi = n_h;										                        
	tmp = F__U64sDIVU64sU32(&tmp, d);					                        
	*r_h = tmp.hi;										                        
	*r_l = tmp.lo;										                        
}/* END F__U64DIVU64U32() */


