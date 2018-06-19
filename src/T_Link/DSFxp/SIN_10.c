/******************************************************************************
*
* FILE:
*   SIN_10.c         2002/11/19
*
*  Copyright (c) 2000 dSPACE GmbH, GERMANY
*
*  $Workfile: SIN_10.c $ $Revision: 3 $ $Date: 28.10.03 14:34 $ $Author: Dagmara $
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* FUNCTION:
*   F__I32SINI32_LE10(v)
*
* DESCRIPTION:
*   Calculates sine using series approach. Last term is x^10/10! 
*
* PARAMETERS:
*   Int32   v      input value
*
* RETURNS:
*
* NOTE:
*
*
******************************************************************************/
Int32 F__I32SINI32_LE10(Int32 v)
{
    Int32 n;

	v += 0x20000000;
	n = v;
    v &= 0x3fffffff;
	v -= 0x20000000;	

    if (n & 0x40000000)
    {
        v = F__I32COSI32_6TERMS(v);
    }
    else
    {
        v = F__I32SINI32_5TERMS(v);
    }
    
    if (n < 0)
    {
        v = -v;
    }
    
    return v;
}
/* END F__I32SINI32_LE10() */
