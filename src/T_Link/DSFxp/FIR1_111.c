/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR1_111.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR1_111.c $ $Revision: 8 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__U16FIR16_SAT_U16U16(Input,NTabs,DelayLine,Coeff)
*
* DESCRIPTION:
*   FIR - Filter calculation with saturation.
*
* PARAMETERS:
*   type    name       meaning    
*   UInt16  Input      current Input 
*   UInt16  NTabs      number of tabs
*   UInt16* DelayLine  pointer to DelayLine vector
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

UInt16 F__U16FIR16_SAT_U16U16(UInt16 Input,UInt16 NTabs,UInt16* DelayLine,const UInt16* Coeff)
{
UInt16    i;
UInt32    Mul;
UInt16    Accu = 0;
	
	/* Update */
	for(i=0;i<NTabs-1;i++)
	{
	  *DelayLine = *(DelayLine-1);  
	   DelayLine--;
	}
	*DelayLine = Input;
	
	/* Accumulation */
	for(i=0;i<NTabs;i++)
	{
	   Mul   = (UInt32)*DelayLine++ * (UInt32)*Coeff++;	
	   Mul  += Accu;
	   Accu  = C__U16FITU32_SAT(Mul,65535);
	}
	return  Accu;
}


