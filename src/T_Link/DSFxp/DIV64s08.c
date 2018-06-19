/**
 * @file       div64s08.c
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
 * $Workfile: DIV64s08.c $
 * $Revision: 5 $
 * $Date: 28.10.03 14:34 $
 */


#include "dsfxp.h"


/* mixed division of Int64s by UInt32 */
Int32 F__I32DIVI64sU32(const UInt64s *u, UInt32 a)
{
    Int32 retval;
    UInt64s U;
    Int8 sign = 0;

    U = *u;    /* local copy to preserve u */

    /* check signs of arguments, force positive */
    if((Int32)U.hi < 0)
    {
        negate(&U);
        sign = !sign;
    }

    /* do unsigned divide */
    retval = (Int32)F__U32DIVU64sU32(&U, (UInt32)a);

    /* restore sign */
    if(sign) return(-retval);
    return(retval);
}
