/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR0_322.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR0_312.c $ $Revision: 4 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__U64FIR64_U16U32(Input,NTabs,DelayLine,Coeff,Counter,Result_h,Result_l)
*
* DESCRIPTION:
*   FIR - Filter calculation.
*
* PARAMETERS:
*   type    name       meaning    
*   UInt16  Input      current Input 
*   UInt16  NTabs      number of tabs
*   UInt16* DelayLine  pointer to DelayLine vector
*   UInt32* Coeff      pointer to Coefficients vector 
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

void F__U64FIR64_U16U32(UInt16 Input,UInt16 NTabs,UInt16* DelayLine,const UInt32* Coeff,UInt32 *Result_h,UInt32 *Result_l)
{
UInt16    i;
UInt32	  Mul_h;
UInt32    Mul_l;
UInt32	  Accu_h = 0;
UInt32    Accu_l = 0;
	
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
	   C__U64MULU32U32((UInt32)*DelayLine,*Coeff, Mul_h, Mul_l);
       DelayLine++;Coeff++;
       C__U64ADDU64U64(Mul_h, Mul_l,Accu_h, Accu_l, Accu_h, Accu_l);	   
	}
	*Result_h = Accu_h;
	*Result_l = Accu_l;
}


