/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR2_212.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR2_212.c $ $Revision: 5 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*  F__U32FIR32_U16CBU32(Input,NTabs,DelayLine,Coeff,Counter)
*
* DESCRIPTION:
*   FIR - Filter calculation with circular buffer.
*
* PARAMETERS:
*   type    name       meaning    
*   UInt16  Input      current Input 
*   UInt16  NTabs      number of tabs
*   UInt16* Counter    pointer to the Counter variable (to work in conjunction with circular buffer)
*   UInt16* DelayLine  pointer to DelayLine vector
*   UInt32* Coeff      pointer to Coefficients vector 
*
* RETURNS:
*   UInt32  accumulation result 
*	
* NOTE:
*   
* HISTORY:
*
******************************************************************************/

UInt32 F__U32FIR32_U16CBU32(UInt16 Input,UInt16 NTabs,UInt16* DelayLine,const UInt32* Coeff,UInt16* Counter)
{
UInt32    Accu = 0;
UInt16    i;
UInt16   *AuxDelayLine;
	
	if (*Counter) (*Counter)--;
	else          (*Counter) = (UInt16)(NTabs-1);

	AuxDelayLine = DelayLine + (*Counter);

	/* Update */
	*AuxDelayLine = Input; 
	
	/* Accumulation */
	for(i=0;i<NTabs;i++)
	{
	   Accu += (UInt32)*AuxDelayLine++ * (UInt32)*Coeff++;	    		
	   if(AuxDelayLine > (DelayLine + (NTabs-1))) AuxDelayLine = DelayLine;
	}
	return  Accu;
}


