/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR2_101.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR2_101.c $ $Revision: 7 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__U16FIR16_U8CBU16(Input,NTabs,DelayLine,Coeff,Counter)
*
* DESCRIPTION:
*   FIR - Filter calculation with circular buffer.
*
* PARAMETERS:
*   type    name       meaning    
*   UInt8   Input      current Input 
*   UInt16  NTabs      number of tabs
*   UInt16* Counter    pointer to the Counter variable (to work in conjunction with circular buffer)
*   UInt8*  DelayLine  pointer to DelayLine vector
*   UInt16* Coeff      pointer to Coefficients vector 
*
* RETURNS:
*   UInt16  accumulation result 
*	
* NOTE:
*   
* HISTORY:
*
******************************************************************************/

UInt16 F__U16FIR16_U8CBU16(UInt8 Input,UInt16 NTabs,UInt8* DelayLine,const UInt16* Coeff,UInt16* Counter)
{
UInt16   i;
UInt16   Accu = 0;
UInt8   *AuxDelayLine;

	
	if (*Counter) (*Counter)--;
	else          (*Counter) = (UInt16)(NTabs-1);

	AuxDelayLine = DelayLine + (*Counter);

	/* Update */
	*AuxDelayLine = Input; 
	
	/* Accumulation */
	for(i=0;i<NTabs;i++)
	{
	   Accu +=(UInt16)((UInt32)*AuxDelayLine++ * (UInt32)*Coeff++);	    		
	   if(AuxDelayLine > (DelayLine + (NTabs-1))) AuxDelayLine = DelayLine;
	}
	return  Accu;
}


