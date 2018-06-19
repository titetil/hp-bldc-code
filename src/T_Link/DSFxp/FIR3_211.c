/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR3_211.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR3_211.c $ $Revision: 7 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__U32FIR32_SAT_U16CBU16(Input,NTabs,DelayLine,Coeff,Counter)
*
* DESCRIPTION:
*   FIR - Filter calculation with circular buffer and saturation.
*
* PARAMETERS:
*   type     name       meaning    
*   UInt16   Input      current Input 
*   UInt16   NTabs      number of tabs
*   UInt16*  Counter    pointer to the Counter variable (to work in conjunction with circular buffer)
*   UInt16*  DelayLine  pointer to DelayLine vector
*   UInt16*  Coeff      pointer to Coefficients vector 
*
* RETURNS:
*   UInt32   accumulation result 
*	
* NOTE:
*   
* HISTORY:
*
******************************************************************************/

UInt32 F__U32FIR32_SAT_U16CBU16(UInt16 Input,UInt16 NTabs,UInt16* DelayLine,const UInt16* Coeff,UInt16* Counter)
{
UInt32    Mul;
UInt32    Accu   = 0;
UInt32    Accu_1 = 0;
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
		Mul  = (UInt32)*AuxDelayLine++ * (UInt32)*Coeff++;	 
		Accu = Accu_1 + Mul; 

		if (  Accu < Accu_1  ) {
		  Accu = 0xFFFFFFFF;
		}
   		
	   Accu_1 = Accu;
	   if(AuxDelayLine > (DelayLine + (NTabs-1))) AuxDelayLine = DelayLine;

    }
	return  Accu;
}


