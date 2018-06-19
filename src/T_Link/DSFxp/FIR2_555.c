/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR2_555.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR2_555.c $ $Revision: 7 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*  F__I16FIR16_I16CBI16(Input,NTabs,DelayLine,Coeff,Counter)
*
* DESCRIPTION:
*   FIR - Filter calculation with circular buffer.
*
* PARAMETERS:
*   type    name       meaning    
*   Int16   Input      current Input 
*   UInt16  NTabs      number of tabs
*   UInt16* Counter    pointer to the Counter variable (to work in conjunction with circular buffer)
*   Int16*  DelayLine  pointer to DelayLine vector
*   Int16*  Coeff      pointer to Coefficients vector 
*
* RETURNS:
*   Int16   accumulation result 
*	
* NOTE:
*   
* HISTORY:
*
******************************************************************************/

Int16 F__I16FIR16_I16CBI16(Int16 Input,UInt16 NTabs,Int16* DelayLine,const Int16* Coeff,UInt16* Counter)
{
UInt16    i;
Int16     Accu = 0;
Int16    *AuxDelayLine;

	
	if (*Counter) (*Counter)--;
	else          (*Counter) = (UInt16)(NTabs-1);

	AuxDelayLine = DelayLine + (*Counter);

	/* Update */
	*AuxDelayLine = Input; 
	
	/* Accumulation */
	for(i=0;i<NTabs;i++)
	{
	   Accu +=(Int16)((Int32)*AuxDelayLine++ * (Int32)*Coeff++);	    		
	   if(AuxDelayLine > (DelayLine + (NTabs-1))) AuxDelayLine = DelayLine;
	}
	return  Accu;
}


