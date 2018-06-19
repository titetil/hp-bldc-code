/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR1_746.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR1_746.c $ $Revision: 6 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__I64FIR64_SAT_I8I32(Input,NTabs,DelayLine,Coeff,Counter,Result_h,Result_l)
*
* DESCRIPTION:
*   FIR - Filter calculation with saturation.
*
* PARAMETERS:
*   type    name       meaning    
*   Int8    Input      current Input 
*   UInt16  NTabs      number of tabs
*   Int8*   DelayLine  pointer to DelayLine vector
*   Int32*  Coeff      pointer to Coefficients vector 
*   Int32*  Result_h   pointer to the result variable (High part of 64 bit variable)  
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

void F__I64FIR64_SAT_I8I32(Int8 Input,UInt16 NTabs,Int8* DelayLine,const Int32* Coeff,Int32 *Result_h,UInt32 *Result_l)
{
UInt16    i;
Int32	  Mul_h;
UInt32    Mul_l;
Int32	  Accu_h = 0;
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
	   C__I64MULI32I32(*DelayLine,*Coeff, Mul_h, Mul_l);
       DelayLine++;Coeff++;
       C__I64ADDI64I64_SAT(Accu_h, Accu_l, Mul_h, Mul_l, Accu_h, Accu_l);	   
	}
	*Result_h = Accu_h;
	*Result_l = Accu_l;
}


