/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR1_555.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR1_555.c $ $Revision: 9 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__I16FIR16_SAT_I16I16(Input,NTabs,DelayLine,Coeff)
*
* DESCRIPTION:
*   FIR - Filter calculation with saturation.
*
* PARAMETERS:
*   type    name       meaning    
*   Int16   Input      current Input 
*   UInt16  NTabs      number of tabs
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

Int16 F__I16FIR16_SAT_I16I16(Int16 Input,UInt16 NTabs,Int16* DelayLine,const Int16* Coeff)
{
UInt16    i;
Int32     Mul;
Int16     Accu = 0;
	
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
	   Mul   = (Int32)*DelayLine++ * (Int32)*Coeff++;	
	   Mul  += Accu;
	   Accu  = C__I16SATI32_SATb(Mul, 32767, -32768);
	}
	return  Accu;
}


