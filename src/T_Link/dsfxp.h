/******************************************************************************
*
*  FILE:
*    dsfxp.h
* 
*  RELATED FILES:
*    adv64.h
*    cmp.h
*    copy.h
*    div.h
*    fir.h
*    fit.h
*    mul.h
*    neg.h
*    sat.h
*    shl.h
*    shr.h
*    sqrt.h
*    sum.h
*    trig.h
*    tl_math.h
* 
*  DESCRIPTION:
*    This file contains include statements for all fixed-point header files
*    C-macros are characterized by a leading "C__".
*    Ansi-C-functions are characterized by a leading "F__".
*    The macro/function-name represents the implemented operation(s):
*    (C__|F__) + output_type + OPERATION + input_type {+ input_type} [+ _SAT] 
*
*  AUTHOR(S):
*    O. Grajetzky
*
*  Copyright (c) 2002 dSPACE GmbH, GERMANY
*
*  $Workfile: dsfxp.h $ $Revision: 14 $ $Date: 11.03.04 16:44 $ $Author: Frankl $
******************************************************************************/


#ifndef __DSFXP_H__
#define __DSFXP_H__

#include "tl_types.h"

/* TargetLink header files */
#if defined(__LCC__)
 #include <.\..\..\SrcFiles\i86\LCC\tllimits.h>
#elif defined(__WCC__)
 #include <.\..\..\SrcFiles\i86\WCC\tllimits.h>
#elif defined(__MSC__)
 #include <.\..\..\SrcFiles\i86\MSVC\tllimits.h>
#else
 #include "tllimits.h"
#endif




#include "adv64.h"
#include "cmp.h"
#include "copy.h"
#include "div.h"
//#include "fir.h"
#include "fit.h"
//#include "mul.h"
#include "neg.h"
#include "sat.h"
#include "shl.h"
#include "shr.h"
//#include "sqrt.h"
//#include "sum.h"
#include "trig.h"
#include "tl_math.h"



#endif /* #ifndef __DSFXP_H__ */
