/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR2_888.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR2_888.c $ $Revision: 7 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*  F__I32FIR32_I16CBI16(Input,NTabs,DelayLine,Coeff,Counter)
*
* DESCRIPTION:
*   FIR - Filter calculation with circular buffer.
*
* PARAMETERS:
*   type     name       meaning    
*   Float32  Input      current Input 
*   UInt16   NTabs      number of tabs
*   UInt16*  Counter    pointer to the Counter variable (to work in conjunction with circular buffer)
*   Float32* DelayLine  pointer to DelayLine vector
*   Float32* Coeff      pointer to Coefficients vector 
*
* RETURNS:
*   Float32  accumulation result 
*	
* NOTE:
*   
* HISTORY:
*
******************************************************************************/

Float32 F__F32FIR32_F32CBF32(Float32 Input,UInt16 NTabs,Float32* DelayLine,const Float32* Coeff,UInt16* Counter)
{
Float32   Accu = 0.0F;
Float32  *AuxDelayLine;
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


