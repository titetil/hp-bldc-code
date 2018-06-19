/**
 * @file       cos_04.c
 * @brief      cosine
 *
 * @since      2003-07-17
 * @author     D. Andoleit
 *
 * Copyright (c) 2003 by dSPACE GmbH, Paderborn, Germany
 * All Rights Reserved
 *
 */

/*
 * $Workfile: COS_04.c $
 * $Revision: 4 $
 * $Date: 28.10.03 14:34 $
 */


#include "dsfxp.h"

/******************************************************************************
*
* FUNCTION:
*   F__I32COSI32_4TERMS(v)
*
* DESCRIPTION:
*   Calculates 4 terms of cosine series.
*   cos(x) = 1 - x^2/2! + x^4/4! - x^6/6!
*
* PARAMETERS:
*   Int32   v      input value
*
* RETURNS:
*   Int32   fcn    return value
*
* NOTE:
*
*
******************************************************************************/
Int32 F__I32COSI32_4TERMS(Int32 v)
{
    Int32 fcn;
	Int32 xr;
	UInt32 x2;
	Int32 AUX_a_Int64_h;
	UInt32 AUX_a_Int64_l;

	F__I64MULI32U32(v, 0x6487ED51, &AUX_a_Int64_h, &AUX_a_Int64_l);
	xr = (Int32)( (Int32)(AUX_a_Int64_h << 2) + (UInt32)( AUX_a_Int64_l >> 30) );

	F__I64MULI32I32(xr, xr, &AUX_a_Int64_h, &AUX_a_Int64_l);
	x2 = (UInt32)( (Int32)(AUX_a_Int64_h << 2) + (UInt32)( AUX_a_Int64_l >> 30) );

    F__I64MULU32U32(x2, 0x44444444, &AUX_a_Int64_h, &AUX_a_Int64_l);
	fcn = (Int32)(AUX_a_Int64_h >> 3);
	fcn = (Int32)(0x40000000 - fcn);

	F__I64MULU32I32(x2, fcn, &AUX_a_Int64_h, &AUX_a_Int64_l);
	fcn = (Int32)( (Int32)(AUX_a_Int64_h << 2) + (UInt32)( AUX_a_Int64_l >> 30) );

	F__I64MULI32U32(fcn, 0x55555555, &AUX_a_Int64_h, &AUX_a_Int64_l);
	fcn = (Int32)(AUX_a_Int64_h >> 2);
	fcn = (Int32)(0x40000000 - fcn);

	F__I64MULU32I32(x2, fcn, &AUX_a_Int64_h, &AUX_a_Int64_l);
	fcn = (Int32)( (Int32)(AUX_a_Int64_h << 1) + (UInt32)( AUX_a_Int64_l >> 31) );
 	fcn = (Int32)(0x40000000 - fcn);

    return fcn;
}
/* END F__I32COSI32_4TERMS() */
