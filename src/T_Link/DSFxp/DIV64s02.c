/******************************************************************************
*                                                                              
* FILE:                                                                        
*   DIV64s02.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: DIV64s02.c $ $Revision: 7 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $                           
******************************************************************************/



#include "dsfxp.h"


#ifdef LITTLE_ENDIAN	/* Target with little-endian architecture */

/* divide a UInt64s by an UInt16                                     */
UInt32 F__U32DIVU64sU16(const UInt64s *u, UInt16 a)
{
    Int8 i;
    UInt64s retval;
    UInt32s rem;
    UInt32 temp;
    UInt16 *up = (UInt16 *)&u->hi;
    UInt16 *qp = (UInt16 *)&retval.hi;

    /* test for divide by zero */
	if(a == 0)
    {
        retval.lo = 0x7FFFFFFFL;
        if((Int32)u->hi < 0) retval.lo |= 0x80000000L;
        return(retval.lo);
    }

	/* do first division (result may be long) */

    retval.hi = u->hi / a;
    rem.lng = u->hi - retval.hi * a;

    /* loop for next two digits */
	for(i=0; i<2; i++)
    {
        --up;
        --qp;
        rem.pr[1] = rem.pr[0];
        rem.pr[0] = *up;
        temp = rem.lng / a;
        *qp = (UInt16)temp;
        rem.lng -= *qp * a;
    }
    return(retval.lo);
}

#else

/* divide a UInt64s by an UInt16                                     */
UInt32 F__U32DIVU64sU16(const UInt64s *u, UInt16 a)
{
    Int8 i;
    UInt64s retval;
    UInt32s rem;
    UInt32 temp;
    UInt16 *up = (UInt16 *)&u->hi;
    UInt16 *qp = (UInt16 *)&retval.hi;

    /* test for divide by zero */
	if(a == 0)
    {
        retval.lo = 0x7FFFFFFFL;
        if((Int32)u->hi < 0) retval.lo |= 0x80000000L;
        return(retval.lo);
    }

	/* do first division (result may be long) */

    retval.hi = u->hi / a;
    rem.lng = u->hi - retval.hi * a;

    /* loop for next two digits */
	++up;
	++qp;
	for(i=0; i<2; i++)
    {
        ++up;
        ++qp;
        rem.pr[0] = rem.pr[1];
        rem.pr[1] = *up;
        temp = rem.lng / a;
        *qp = (UInt16)temp;
        rem.lng -= *qp * a;
    }
    return(retval.lo);
}

#endif


