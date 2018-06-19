/******************************************************************************
*
* FILE:
*   TAN_11a.c         2002/11/19
*
*  Copyright (c) 2000 dSPACE GmbH, GERMANY
*
*  $Workfile: TAN_11a.c $ $Revision: 2 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* FUNCTION:
*   F__I32TANI32_LE11_ARB(v, yN, yD)
*
* DESCRIPTION:
*   Calculates tan as sin/cos using series approach.
*   Scaling of output is arbitrary
*
* PARAMETERS:
*   Int32   v      input value
*   Int16   yN     nominator of output LSB
*   Int16   yD     denominator of output LSB
*
* RETURNS:
*   Int32   y      return value 
*
* NOTE:
*
*
******************************************************************************/
Int32 F__I32TANI32_LE11_ARB(Int32 v, Int32 yN, Int32 yD)
{
    Int32 y_sin;
    Int32 y_cos;
    Int32 y = 1;
    Int32	y_sin_h;	
    UInt32	y_sin_l;	
    Int32	y_cos_h;	
    UInt32	y_cos_l;	
       
    if (v == INT32MIN)
    {
        y = INT32MIN;
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
        
        C__I64SHLI32C6_LT32(y_cos, 31, 1, y_cos_h, y_cos_l);
        C__I64DIVI64I32(y_cos_h, y_cos_l, yD, y_sin_h, y_sin_l);
        C__I64MULI64I32(y_sin_h, y_sin_l, yN, y_cos_h, y_cos_l)
        C__I64COPYI32(y_sin, y_sin_h, y_sin_l);
        
        if C__LT64(y_sin_h, y_sin_l, y_cos_h, y_cos_l)
        {
            y *= y_sin;
            C__I64MULI32I32(y,yD,y_sin_h, y_sin_l);
            C__I64DIVI64I32(y_sin_h, y_sin_l, yN, y_cos_h, y_cos_l);
            C__I32DIVI64I32(y_cos_h, y_cos_l, y_cos, y);
        }
        else
        {
            if (y == 1)
            {
                y = INT32MAX;
            }
            else
            {
                y = INT32MIN;
            }
        }
    }
    return y;
} /* END F__I32TANI32_LE11_ARB() */
