/**
 * @file       atan_01.c
 * @brief      atan
 *
 * @since      2003-07-17
 * @author     D. Andoleit
 *
 * Copyright (c) 2003 by dSPACE GmbH, Paderborn, Germany
 * All Rights Reserved
 *
 */

/*
 * $Workfile: ATAN_01.c $
 * $Revision: 5 $
 * $Date: 5.02.04 7:03 $
 */


#include "dsfxp.h"

/******************************************************************************
*
* FUNCTION:
*   F__I16ATANI16(v)
*
* DESCRIPTION:
*   Calculates atan using asin.
*   Scaling is less than Int16, 2^-14
*
* PARAMETERS:
*   Int16   v      input value
*
* RETURNS:
*   Int16   fcn    return value
*
* NOTE:
*
*
******************************************************************************/

Int16 F__I16ATANI16(Int16 x_In, Int32 c1)
{
    Int16 x2;
    Int32 x1;

    if (x_In == 0)
    {
        return 0;
    }
    else
    {
        x1 = (Int32)(x_In * ((Int32)x_In)) + c1;
    
        C__I16SQRTI32((Int32)(x1 >> 2), x2);
    
        if (x2 == 0)  /* to prevent integer division by zero */
        {
            if (x_In > 0)
            {
                return 0x517C;   /* pi/2 */
            }
            else
            {            
                return 0xAE83;   /* -pi/2 */
            }
        }
        else
        {
            x1 = (Int16)(((Int32)((Int32)x_In) << 13) / x2);
        
            return F__I16ASINI16((Int16)x1);
        }
    }
} /* END F__I16ATANI16() */