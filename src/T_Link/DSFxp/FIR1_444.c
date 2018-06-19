/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR1_444.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR1_444.c $ $Revision: 10 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__I8FIR8_SAT_I8I8(Input,NTabs,DelayLine,Coeff)
*
* DESCRIPTION:
*   FIR - Filter calculation with saturation.
*
* PARAMETERS:
*   type    name       meaning    
*   Int8    Input      current Input 
*   UInt16  NTabs      number of tabs
*   Int8*   DelayLine  pointer to DelayLine vector
*   Int8*   Coeff      pointer to Coefficients vector 
*
* RETURNS:
*   Int8    accumulation result 
*	
* NOTE:
*   
* HISTORY:
*
******************************************************************************/

Int8 F__I8FIR8_SAT_I8I8(Int8 Input,UInt16 NTabs,Int8* DelayLine,const Int8* Coeff)
{
UInt16   i;
Int16    Mul;
Int8     Accu = 0;
	
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
	   Mul  = (Int16)((Int16)*DelayLine++ * (Int16)*Coeff++);
	   Mul += Accu;	    		
	   Accu = C__I8SATI16_SATb(Mul,127,-128);
	   	
	}
	return  Accu;
}


