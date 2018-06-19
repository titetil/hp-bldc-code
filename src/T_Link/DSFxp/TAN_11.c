/**
 * @file       tan_11.c
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
 * $Workfile: TAN_11.c $
 * $Revision: 4 $
 * $Date: 28.10.03 14:34 $
 */


#include "dsfxp.h"

/******************************************************************************
*
* FUNCTION:
*   F__I32TANI32_LT11(v, ysh)
*
* DESCRIPTION:
*   Calculates tan as sin/cos using table.
*   Output scaling is 2^-ysh
*
* PARAMETERS:
*   Int32   v      input value
*   Int8    ysh    exponent of output LSB negate
*
* RETURNS:
*   Int32   y      return value 
*
* NOTE:
*
*
******************************************************************************/
Int32 F__I32TANI32_LE11(Int32 v, Int8 ysh)
{
    Int32 y_sin;
    Int32 y_cos;
    Int32 y = 1;
    Int32	y_sin_h;	
    UInt32	y_sin_l;	
    Int32	y_cos_h;	
    UInt32	y_cos_l;	
       
    if (v == -2147483648)
    {
        y = 0x80000000;
    }
    else
    {
        if (v < 0)
        {
            v = -v;
            y = -1;
        }
        
        v = v >> 1;
        y_sin = F__I32SINI32_LE11(v);
        y_cos = F__I32SINI32_LE11(v + 0x40000000);
        
        C__I64COPYI32(y_sin, y_sin_h, y_sin_l);
        C__I64SHLI32C6_LT32(y_cos, 31 - ysh, 1 + ysh, y_cos_h, y_cos_l);
        
        if C__LT64(y_sin_h, y_sin_l, y_cos_h, y_cos_l)
        {
            y *= y_sin;
            C__I64SHLI32C6_LT32(y, ysh, (32-ysh), y_sin_h, y_sin_l);
            C__I32DIVI64I32(y_sin_h, y_sin_l, y_cos, y);
        }
        else
        {
            if (y == 1)
            {
                y = 0x7fffffff;
            }
            else
            {
                y = 0x80000000;
            }
        }
    }
    return y;
} /* END F__I32TANI32_LE11() */
