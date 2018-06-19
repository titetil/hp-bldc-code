/**
 * @file       sin_06.c
 * @brief      sine
 *
 * @since      2003-07-17
 * @author     D. Andoleit
 *
 * Copyright (c) 2003 by dSPACE GmbH, Paderborn, Germany
 * All Rights Reserved
 *
 */

/*
 * $Workfile: SIN_06.c $
 * $Revision: 4 $
 * $Date: 28.10.03 14:34 $
 */


#include "dsfxp.h"

/******************************************************************************
* FUNCTION:
*   F__I32SINI32_6TERMS(v)
*
* DESCRIPTION:
*   Calculates 6 terms of sine series.
*   sin(x) = x - x^3/3! + x^5/5! - x^7/7! + x^9/9! - x^11/11!
*
* PARAMETERS:
*   Int32   v      input value
*
* RETURNS:
*   Int32   v      input variable is used as return value
*
******************************************************************************/
Int32 F__I32SINI32_6TERMS(Int32 v)
{
    Int32 xr;
    UInt32 x2;
    Int32 AUX_a_Int64_h;
    UInt32 AUX_a_Int64_l;

    F__I64MULI32U32(v, 0x6487ED51, &AUX_a_Int64_h, &AUX_a_Int64_l);
    xr = (Int32)( (Int32)(AUX_a_Int64_h << 2) + (UInt32)(AUX_a_Int64_l >> 30) );
    F__I64MULI32I32(xr, xr, &AUX_a_Int64_h, &AUX_a_Int64_l);
    x2 = (UInt32)( (Int32)(AUX_a_Int64_h << 2) + (UInt32)(AUX_a_Int64_l >> 30) );

    F__I64MULU32U32(x2, 0x4A7904A7, &AUX_a_Int64_h, &AUX_a_Int64_l);
    v = (Int32)( 0x40000000 - (Int32)(AUX_a_Int64_h >> 5) );

    F__I64MULI32U32(v, x2, &AUX_a_Int64_h, &AUX_a_Int64_l);
    v = (Int32)( (Int32)(AUX_a_Int64_h << 2) + (UInt32)(AUX_a_Int64_l >> 30) );
    F__I64MULI32U32(v, 0x71C71C71, &AUX_a_Int64_h, &AUX_a_Int64_l);
    v = (Int32)( 0x40000000 - (Int32)(AUX_a_Int64_h >> 5) );

    F__I64MULI32U32(v, x2, &AUX_a_Int64_h, &AUX_a_Int64_l);
    v = (Int32)( (Int32)(AUX_a_Int64_h << 2) + (UInt32)(AUX_a_Int64_l >> 30) );
    F__I64MULI32U32(v, 0x61861862, &AUX_a_Int64_h, &AUX_a_Int64_l);
    v = (Int32)( 0x40000000 - (Int32)(AUX_a_Int64_h >> 4) );

    F__I64MULI32U32(v, x2, &AUX_a_Int64_h, &AUX_a_Int64_l);
    v = (Int32)( (Int32)(AUX_a_Int64_h << 2) + (UInt32)(AUX_a_Int64_l >> 30) );
    F__I64MULI32U32(v, 0x66666666, &AUX_a_Int64_h, &AUX_a_Int64_l);
    v = (Int32)( 0x40000000 - (Int32)(AUX_a_Int64_h >> 3) );

    F__I64MULI32U32(v, x2, &AUX_a_Int64_h, &AUX_a_Int64_l);
    v = (Int32)( (Int32)(AUX_a_Int64_h << 2) + (UInt32)(AUX_a_Int64_l >> 30) );
    F__I64MULI32U32(v, 0x55555555, &AUX_a_Int64_h, &AUX_a_Int64_l);
    v = (Int32)( 0x40000000 - (Int32)(AUX_a_Int64_h >> 1) );

    F__I64MULI32I32(xr, v, &AUX_a_Int64_h, &AUX_a_Int64_l);
    v = (Int32)( (Int32)(AUX_a_Int64_h << 2) + (UInt32)(AUX_a_Int64_l >> 30) );

    return v;
}
/* END F__I32SINI32_6TERMS() */
