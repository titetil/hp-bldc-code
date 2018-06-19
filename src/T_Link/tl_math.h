/******************************************************************************
*
* FILE:
*   hyp.h         
*
* RELATED FILES:
*   math.h
*
* DESCRIPTION:
*   Definines prototype of hypot function.
*
*   Calculates the hypotenuse.
*   C__HYPOT(x, y);

*
*  AUTHOR(S):
*    S. Hillebrand
*
*  Copyright (c) 2003 dSPACE GmbH, GERMANY
*
*  $Workfile: tl_math.h $ $Revision: 1 $ $Date: 15.08.03 13:29 $ $Author: Sebastianh $
******************************************************************************/

#ifndef __tl_math_h__
#define __tl_math_h__

#include <math.h>

/* hypot */
#define C__HYPOT(x, y)\
    sqrt((x)*(x) + (y)*(y))

#endif /* #ifndef __tl_math_h__ */
