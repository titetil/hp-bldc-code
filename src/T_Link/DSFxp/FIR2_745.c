/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR2_745.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR2_745.c $ $Revision: 8 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__I64FIR64_I8CBI16(Input,NTabs,DelayLine,Coeff,Counter,Result_h,Result_l)
*
* DESCRIPTION:
*   FIR - Filter calculation with circular buffer.
*
* PARAMETERS:
*   type    name       meaning    
*   Int8    Input      current Input 
*   UInt16  NTabs      number of tabs
*   Int8*   DelayLine  pointer to DelayLine vector
*   Int16*  Coeff      pointer to Coefficients vector 
*   UInt16* Counter    pointer to the Counter variable (to work in conjunction with circular buffer)
*   Int32*  Result_h   pointer to the result variable  (High part of 64 bit variable)  
*   UInt32* Result_l   pointer to the result variable  (Low part of 64 bit Variable)
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


void F__I64FIR64_I8CBI16(Int8 Input,UInt16 NTabs,Int8* DelayLine,const Int16* Coeff,UInt16* Counter,Int32 *Result_h,UInt32 *Result_l)
{
Int32     Mul;
Int32     Accu_h = 0;
UInt32    Accu_l = 0;
UInt16    i;
Int8     *AuxDelayLine;


	
	if (*Counter) (*Counter)--;
	else          (*Counter) = (UInt16)(NTabs-1);

	AuxDelayLine = DelayLine + (*Counter);

	/* Update */
	*AuxDelayLine = Input; 
	
	/* Accumulation */
	for(i=0;i<NTabs;i++)
	{
	   Mul   = (Int32)*AuxDelayLine++ * (Int32)*Coeff++;
	   C__I64ADDI64I32(Accu_h,Accu_l, Mul, Accu_h, Accu_l);
	   if(AuxDelayLine > (DelayLine + (NTabs-1))) AuxDelayLine = DelayLine;
	}
	*Result_h = Accu_h;
	*Result_l = Accu_l;
}
