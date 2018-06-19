/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR2_646.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR2_646.c $ $Revision: 5 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*  F__I32FIR32_I8CBI32(Input,NTabs,DelayLine,Coeff,Counter)
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
*   Int32*  Coeff      pointer to Coefficients vector 
*
* RETURNS:
*   Int32   accumulation result 
*	
* NOTE:
*   
* HISTORY:
*
******************************************************************************/

Int32 F__I32FIR32_I8CBI32(Int8 Input,UInt16 NTabs,Int8* DelayLine,const Int32* Coeff,UInt16* Counter)
{  
Int32     Accu = 0;
Int32     Mul_h;
UInt32    Mul_l;
Int8      *AuxDelayLine;
UInt16    i;

	
	if (*Counter) (*Counter)--;
	else          (*Counter) = (UInt16)(NTabs-1);

	AuxDelayLine = DelayLine + (*Counter);

	/* Update */
	*AuxDelayLine = Input; 
	
	/* Accumulation */
	for(i=0;i<NTabs;i++)
	{
	   C__I64MULI32I32((Int32)*AuxDelayLine,*Coeff,Mul_h,Mul_l);
	   AuxDelayLine++;Coeff++;
	   Accu +=(Int32)Mul_l;	 

	   if(AuxDelayLine > (DelayLine + (NTabs-1))) AuxDelayLine = DelayLine;
	}
	return  Accu;
}


