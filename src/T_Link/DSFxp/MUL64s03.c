/******************************************************************************
*                                                                              
* FILE:                                                                        
*   MUL64s03.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: MUL64s03.c $ $Revision: 4 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $                           
******************************************************************************/



#include "dsfxp.h"


/* Multiply two Int32 to yield a Int64s using Booth's algorithm */
UInt64s F__I64sMULI32I32(const Int32 x, const Int32 y)
{
	UInt64s retval;
	retval = F__U64sMULU32U32((UInt32)x, (UInt32)y);
	if(x < 0) retval.hi -= y;
	if(y < 0) retval.hi -= x;
	return(retval);
}


