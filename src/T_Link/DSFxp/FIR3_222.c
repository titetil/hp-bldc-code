/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR3_222.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR3_222.c $ $Revision: 8 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__U32FIR32_SAT_U32CBU32(Input,NTabs,DelayLine,Coeff,Counter)
*
* DESCRIPTION:
*   FIR - Filter calculation with circular buffer and saturation.
*
* PARAMETERS:
*   type     name       meaning    
*   UInt32   Input      current Input 
*   UInt16   NTabs      number of tabs
*   UInt16*  Counter    pointer to the Counter variable (to work in conjunction with circular buffer)
*   UInt32*  DelayLine  pointer to DelayLine vector
*   UInt32*  Coeff      pointer to Coefficients vector 
*
* RETURNS:
*   UInt32   accumulation result 
*	
* NOTE:
*   
* HISTORY:
*
******************************************************************************/

UInt32 F__U32FIR32_SAT_U32CBU32(UInt32 Input,UInt16 NTabs,UInt32* DelayLine,const UInt32* Coeff,UInt16* Counter)
{
UInt32     Mul_h;
UInt32     Mul_l;
UInt32     Accu_h  = 0;
UInt32     Accu_l = 0;
UInt32    *AuxDelayLine;
UInt16     i;

	if (*Counter) (*Counter)--;
	else          (*Counter) = (UInt16)(NTabs-1);

	AuxDelayLine = DelayLine + (*Counter);

	/* Update */
	*AuxDelayLine = Input; 

	/* Accumulation */
	for(i=0;i<NTabs;i++)
	{
	   C__U64MULU32U32(*AuxDelayLine, *Coeff, Mul_h, Mul_l);
	   C__U64ADDU64U32_SAT(Mul_h, Mul_l, Accu_l, Accu_h, Accu_l);

       if(Accu_h) Accu_l = 0xFFFFFFFF;
       
       AuxDelayLine++;Coeff++;

	   if(AuxDelayLine > (DelayLine + (NTabs-1))) AuxDelayLine = DelayLine;

    }
	return  Accu_l;
}


