/******************************************************************************
*                                                                              
* FILE:                                                                        
*   DIV64s03.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: DIV64s03.c $ $Revision: 4 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $                           
******************************************************************************/



#include "dsfxp.h"


/* mixed division of Int64s by UInt32 */
UInt64s F__I64sDIVI64sU32(const UInt64s *u, UInt32 a)
{
	UInt64s retval;
	UInt64s U;
	Int8 sign = 0;

	U = *u;	/* local copy to preserve u */

	/* check signs of arguments, force positive */
	if((Int32)U.hi < 0)
	{
		negate(&U);
		sign = !sign;
	}
	
	/* do unsigned divide */
	retval = F__U64sDIVU64sU32(&U, (UInt32)a);

	/* restore sign */
	if(sign) negate(&retval);
	return(retval);
}



