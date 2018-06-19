/**
 * @file       tan_01a.c
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
 * $Workfile: TAN_01a.c $
 * $Revision: 5 $
 * $Date: 28.10.03 14:34 $
 */


#include "dsfxp.h"

/******************************************************************************
*
* FUNCTION:
*   F__I16TANI16_ARB(v, yN, yD)
*
* DESCRIPTION:
*   Calculates tan as sin/cos using table.
*   Scaling of output is arbitrary
*
* PARAMETERS:
*   Int16   v      input value
*   Int16   yN     nominator of output LSB
*   Int16   yD     denominator of output LSB
*
* RETURNS:
*   Int16   y      return value
*
* NOTE:
*
*
******************************************************************************/
Int16 F__I16TANI16_ARB(Int16 v, Int16 yN, Int16 yD)
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
	        v = (Int16)-v;
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

        if ((Int32)y_sin < ( (Int32)((Int32)(((Int32)y_cos << 15) / (Int32)yD)) * (Int32)yN))
        {
            y *= (Int16)( ((Int32)y_sin) * ((Int32)yD) / ((Int32)yN) / ((Int32)y_cos));
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
} /* END F__I16TANI16_ARB() */
