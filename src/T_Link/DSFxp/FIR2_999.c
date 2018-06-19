/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR2_999.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR2_999.c $ $Revision: 7 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*  F__F64FIR64_F64CBF64(Input,NTabs,DelayLine,Coeff,Counter)
*
* DESCRIPTION:
*   FIR - Filter calculation with circular buffer.
*
* PARAMETERS:
*   type     name       meaning    
*   Float64  Input      current Input 
*   UInt16   NTabs      number of tabs
*   UInt16*  Counter    pointer to the Counter variable (to work in conjunction with circular buffer)
*   Float64* DelayLine  pointer to DelayLine vector
*   Float64* Coeff      pointer to Coefficients vector 
*
* RETURNS:
*   Float64  accumulation result 
*	
* NOTE:
*   
* HISTORY:
*
******************************************************************************/

Float64 F__F64FIR64_F64CBF64(Float64 Input,UInt16 NTabs,Float64* DelayLine,const Float64* Coeff,UInt16* Counter)
{
Float64   Accu = 0.0L;
Float64  *AuxDelayLine;
UInt16    i;
	
	if (*Counter) (*Counter)--;
	else          (*Counter) = (UInt16)(NTabs-1);

	AuxDelayLine = DelayLine + (*Counter);

	/* Update */
	*AuxDelayLine = Input; 
	
	/* Accumulation */
	for(i=0;i<NTabs;i++)
	{
	   Accu += *(AuxDelayLine++) * *(Coeff++);	    		
	   if(AuxDelayLine > (DelayLine + (NTabs-1))) AuxDelayLine = DelayLine;
	}
	return  Accu;
}


