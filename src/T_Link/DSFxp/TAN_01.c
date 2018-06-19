/**
 * @file       tan_01.c
 * @brief      tangent
 *
 * @since      2003-07-17
 * @author     D. Andoleit
 *
 * Copyright (c) 2003 by dSPACE GmbH, Paderborn, Germany
 * All Rights Reserved
 *
 */

/*
 * $Workfile: TAN_01.c $
 * $Revision: 8 $
 * $Date: 24.11.03 16:25 $
 */


#include "dsfxp.h"

/******************************************************************************
*
* FUNCTION:
*   F__I16TANI16(v, ysh)
*
* DESCRIPTION:
*   Calculates tan as sin/cos using table.
*   Output scaling is 2^-ysh
*
* PARAMETERS:
*   Int16   v      input value
*   Int8    ysh    exponent of output LSB negate
*
* RETURNS:
*   Int16   y      return value
*
* NOTE:
*
*
******************************************************************************/
Int16 F__I16TANI16(Int16 v, Int8 ysh)
{
	 UInt16 y_cos;
	 UInt16 y_sin;
	 Int16 i;
	 Int16 y = 1;

    if (v == -32768)
    {
 		y = 0x8000;
    }
    else
    {
	    if ( v < 0 )
	    {
	        v = (Int16)(-v);
	        y = -1;
	    }

        /* calculate table index and value of cosine */
        i = (Int16)(v >> 8);
        y_cos = cosLUT[i];
        y_cos -= (UInt16)((UInt32)((UInt16)(y_cos - cosLUT[i+1]) * (v & 0xFF)) >> 8);

        /* calculate table index and value of sine */
        i = (Int16)(128 - i);
        y_sin = cosLUT[i];
        y_sin += (UInt16)((UInt32)((UInt16)(cosLUT[i-1] - y_sin) * (v & 0xFF)) >> 8);

        if ((UInt32)y_sin < (UInt32)((UInt32)y_cos << (15 - ysh)))
        {
            y *= (Int16)((UInt32)((UInt32)y_sin << ysh) / y_cos);
        }
        else
        {
            if (y == 1)
            {
                y = 0x7fff;
            }
            else
            {
                y = 0x8000;
            }
        }
    }
    return y;
} /* END F__I16TANI16() */
