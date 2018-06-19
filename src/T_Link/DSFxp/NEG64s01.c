/******************************************************************************
*                                                                              
* FILE:                                                                        
*   NEG64s01.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: NEG64s01.c $ $Revision: 6 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $                           
******************************************************************************/



#include "dsfxp.h"


void negate(UInt64s *x)
{
	x->lo = -x->lo;	/* complement */
	x->hi = ~x->hi;
	if(x->lo == 0) ++x->hi;
}


