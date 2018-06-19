/******************************************************************************
*                                                                              
* FILE:                                                                        
*   MUL64s02.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: MUL64s02.c $ $Revision: 4 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $                           
******************************************************************************/



#include "dsfxp.h"


/* Multiply two Int32 with UInt32 to yield a UInt64s using Booth's algorithm */
UInt64s F__I64sMULI32U32(const Int32 x, const UInt32 y)
{
	UInt64s retval;
	retval = F__U64sMULU32U32((UInt32)x, y);
	if(x < 0) retval.hi -= y;
	return(retval);
}



