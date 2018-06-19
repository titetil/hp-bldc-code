/**
 * @file       atan_02.c
 * @brief      atan2
 *
 * @since      2003-07-17
 * @author     D. Andoleit
 *
 * Copyright (c) 2003 by dSPACE GmbH, Paderborn, Germany
 * All Rights Reserved
 *
 */

/*
 * $Workfile: ATAN_02.c $
 * $Revision: 5 $
 * $Date: 28.10.03 14:34 $
 */


#include "dsfxp.h"

/******************************************************************************
*
* FUNCTION:
*   F__I16ATAN2I16(v, c, y)
*
* DESCRIPTION:
*   Calculates atan using asin.
*   Scaling is less than Int16, 2^-14
*
* PARAMETERS:
*   Int16   v      = x / y; input value of atan
*   Int16   c      second input value
*   Int16   y      denominator of v = x/y
*
* RETURNS:
*   Int16   fcn    return value
*
* NOTE:
*
******************************************************************************/
Int16 F__I16ATAN2I16(Int16 v, Int32 c, Int16 y)
{
    Int16 v1;

       v1 = (Int16)(F__I16ATANI16(v, c) >> 1);

       if (y < 0)
       {
           if (v > 0)
           {
		       v1 -= 0x6488; /* -pi */
           }
           else
           {
		       v1 += 0x6488; /* +pi */
           }
           v1 = (Int16)-v1;
       }
       return v1;
} /* END F__I16ATAN2I16() */
