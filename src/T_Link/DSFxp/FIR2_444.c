/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR2_444.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR2_444.c $ $Revision: 7 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*  F__I8FIR8_I8CBI8(Input,NTabs,DelayLine,Coeff,Counter)
*
* DESCRIPTION:
*   FIR - Filter calculation with circular buffer.
*
* PARAMETERS:
*   type    name       meaning    
*   Int8    Input      current Input 
*   UInt16  NTabs      number of tabs
*   UInt16* Counter    pointer to the Counter variable (to work in conjunction with circular buffer)
*   Int8*   DelayLine  pointer to DelayLine vector
*   Int8*   Coeff      pointer to Coefficients vector 
*
* RETURNS:
*   Int8   accumulation result 
*	
* NOTE:
*   
* HISTORY:
*
******************************************************************************/


Int8 F__I8FIR8_I8CBI8(Int8 Input,UInt16 NTabs,Int8* DelayLine,const Int8* Coeff,UInt16* Counter)
{
UInt16   i;
Int8     Accu = 0;
Int8    *AuxDelayLine;
	
	if (*Counter) (*Counter)--;
	else          (*Counter) = (UInt16)(NTabs-1);

	AuxDelayLine = DelayLine + (*Counter);

	/* Update */
	*AuxDelayLine = Input; 

	
	/* Accumulation */
	for(i=0;i<NTabs;i++)
	{
	   Accu +=(Int8)((Int16) *AuxDelayLine++ * (Int16) *Coeff++);	    		
	   if(AuxDelayLine > (DelayLine + (NTabs-1))) AuxDelayLine = DelayLine;
	}
	return  Accu;
}


