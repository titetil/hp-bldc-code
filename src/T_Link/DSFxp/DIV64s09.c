/**
 * @file       div64s09.c
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
 * $Workfile: DIV64s09.c $
 * $Revision: 6 $
 * $Date: 28.10.03 14:34 $
 */


#include "dsfxp.h"


/* mixed division of UInt64s by Int32 */
Int32 F__I32DIVU64sI32(const UInt64s *u, Int32 a)
{
    Int32 retval;
    UInt64s U;
    Int8 sign = 0;

    U = *u;    /* local copy to preserve u */

    /* check signs of arguments, force positive */
    if(a < 0)
    {
        a = -a;
        sign = !sign;
    }

    /* do unsigned divide */
    retval = (Int32)F__U32DIVU64sU32(&U, (UInt32)a);

    /* restore sign */
    if(sign) return(-retval);
    return(retval);
}
