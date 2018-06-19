/**
 * @file       mul64_01.c
 * @brief      64-bit multiplication
 *
 * @since      1999-04-16
 * @author     O. Grajetzky
 *
 * Copyright (c) 2003 by dSPACE GmbH, Paderborn, Germany
 * All Rights Reserved
 *
 */

/*
 * $Workfile: MUL64_01.c $
 * $Revision: 6 $
 * $Date: 28.10.03 14:34 $
 */


#include "dsfxp.h"


/******************************************************************************
*
* FUNCTION:
*   void F__I64MULI64I32(Int32 f1_h, UInt32 f1_l Int32 f2, Int32 *r_h,
*                        UInt32 *r_l)
*
* DESCRIPTION:
*   Multiplies f1 by f2 with 64-bit result.
*
* PARAMETERS:
*   typ     name    meaning
*   Int32   f1_h    1. factor (high-part)
*   UInt32  f1_l    1. factor (low-part)
*   Int32     f2    2. factor
*   Int32   *r_h    64bit - result (high-part)
*   UInt32  *r_l    64bit - result (low-part)
*
* RETURNS:
*
* NOTE:
* f1                                                 * f2 =
* (f1_h*2^32                 + f1_l)                 * f2 =
* ((f1_hH*2^16 + f1_hL)*2^32 + f1_lH*2^16 + f1_lL)   * (f2H*2^16 + f2L) =
* (f1_hH*2^48 + f1_hL*2^32   + f1_lH*2^16 + f1_lL)   * (f2H*2^16 + f2L) =
*
* f1_hH*f2H*2^64 + f1_hL*f2H*2^48 + f1_lH*f2H*2^32 + f1_lL*f2H*2^16 +
* f1_hH*f2L*2^48 + f1_hL*f2L*2^32 + f1_lH*f2L*2^16 + f1_lL*f2L
*
* shifted by  64    48    32    16    0 Bits
* |-----|-----|-----|-----|-----|-----|
* |  f1_hH*f2H|     |     |     |     |
*       |  f1_hL*f2H|     |     |     |
*             |  f1_lH*f2H|     |     |
*                   |  f1_lL*f2H|     |
*       |  f1_hH*f2L|     |     |     |
*             |  f1_hL*f2L|     |     |
*                   |  f1_lH*f2L|     |
*                         |  f1_lL*f2L|
* |-----|-----|-----|-----|-----|-----|
*             |<   r_h   >|<   r_l   >|
*
*   Converts negative numbers with Booth-algorithm.
*
*
* HISTORY:
*
******************************************************************************/
void F__I64MULI64I32(Int32 f1_h, UInt32 f1_l, Int32 f2, Int32 *r_h, UInt32 *r_l)
{
    UInt16  f1_hH,
            f1_hL,
            f1_lH,
            f1_lL,
            f2H,
            f2L;
    UInt32  f1_lHf2H,
            f1_lLf2H,
            f1_lLf2L,
            temp;

    f1_hH = (UInt16)((UInt32)f1_h >> 16);
    f1_hL = (UInt16)f1_h;
    f1_lH = (UInt16)((UInt32)f1_l >> 16);
    f1_lL = (UInt16)f1_l;
    f2H   = (UInt16)((UInt32)f2 >> 16);
    f2L   = (UInt16)f2;

    f1_lHf2H = (UInt32)f1_lH * (UInt32)f2H;
    f1_lLf2H = (UInt32)f1_lL * (UInt32)f2H;
    f1_lLf2L = (UInt32)f1_lL * (UInt32)f2L;

    temp = f1_lLf2H + ((UInt32)f1_lH*(UInt32)f2L);
    if(temp < f1_lLf2H) f1_lHf2H += 0x10000;
    f1_lLf2H = temp + (f1_lLf2L >> 16);
    if(f1_lLf2H < temp) f1_lHf2H += 0x10000;

    *r_l = (f1_lLf2H << 16) + (f1_lLf2L & 0x0000FFFF);
    *r_h = (Int32)( (((UInt32)f1_hL*(UInt32)f2H) << 16) + f1_lHf2H +
           (((UInt32)f1_hH*(UInt32)f2L) << 16) + ((UInt32)f1_hL*(UInt32)f2L) +
           (f1_lLf2H >> 16) );

    if(f2 < 0) *r_h -= f1_l;

}/*END F__I64MULI64I32()*/
