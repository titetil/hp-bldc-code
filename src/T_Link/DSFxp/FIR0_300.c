/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR0_300.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR0_300.c $ $Revision: 7 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__U64FIR64_U32U32(Input,NTabs,DelayLine,Coeff,Counter,Result_h,Result_l)
*
* DESCRIPTION:
*   FIR - Filter calculation.
*
* PARAMETERS:
*   type    name       meaning    
*   UInt8   Input      current Input 
*   UInt16  NTabs      number of tabs
*   UInt8*  DelayLine  pointer to DelayLine vector
*   UInt8*  Coeff      pointer to Coefficients vector 
*   UInt32* Result_h   pointer to the result variable (High part of 64 bit variable)  
*   UInt32* Result_l   pointer to the result variable (Low part of 64 bit Variable)
*
* RETURNS:
*	void
*
* NOTE:
*   After return the Result_h/Result_l variables contain the 64bit accumulation result.  
*   
* HISTORY:
*
******************************************************************************/

void F__U64FIR64_U8U8(UInt8 Input,UInt16 NTabs,UInt8* DelayLine,const UInt8* Coeff,UInt32 *Result_h,UInt32 *Result_l)
{
UInt16   i;
UInt16   Mul;
UInt32   Accu_h = 0;
UInt32   Accu_l = 0;
	
	/* Update */
	for(i=0;i<NTabs-1;i++)
	{
	  *DelayLine = *(DelayLine-1);  
	   DelayLine--;
	}
	*DelayLine = Input;
	
	/* Accumulation */
	for(i=0;i<NTabs;i++)
	{
	   Mul = (UInt16)((UInt16)*DelayLine++ * (UInt16) *Coeff++);
	   C__U64ADDU64U32(Accu_h,Accu_l,(UInt32)Mul, Accu_h, Accu_l);
	}

	*Result_h = Accu_h;
	*Result_l = Accu_l;	
}


