/**
 * @file       div64_06.c
 * @brief      64-bit divisions
 *
 * @since      1999-04-16
 * @author     O. Grajetzky
 *
 * Copyright (c) 2003 by dSPACE GmbH, Paderborn, Germany
 * All Rights Reserved
 *
 */

/*
 * $Workfile: DIV64_06.c $
 * $Revision: 6 $
 * $Date: 28.10.03 14:34 $
 */


#include "dsfxp.h"


/******************************************************************************
*
* FUNCTION:
*   F__I32DIVI64U32(n_h, n_l, d)
*
* DESCRIPTION:
*   Divides the 64bit Value n by the 32bit Value d with a 32bit result.
*
* PARAMETERS:
*   type    name    meaning
*   Int32    n_h    upper 32Bit of the nominator
*   UInt32   n_l    lower 32Bit of the nominator
*   UInt32     d    denominator
*
* RETURNS:
*   Int32      result
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
Int32 F__I32DIVI64U32(Int32 n_h, UInt32 n_l, UInt32 d)
{
	UInt64s tmp;

	tmp.lo = n_l;
	tmp.hi = (UInt32)n_h;
	return( F__I32DIVI64sU32(&tmp, d) );
}/* END F__I32DIVI64U32() */
