/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR0_888.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR0_888.c $ $Revision: 6 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*  F__I32FIR32_I32I32(Input,NTabs,DelayLine,Coeff)
*
* DESCRIPTION:
*   FIR - Filter calculation.
*
* PARAMETERS:
*   type     name       meaning    
*   Float32  Input      current Input 
*   UInt16   NTabs      number of tabs
*   Float32* DelayLine  pointer to DelayLine vector
*   Float32* Coeff      pointer to Coefficients vector 
*
* RETURNS:
*   Float32  accumulation result 
*	
* NOTE:
*   
* HISTORY:
*
******************************************************************************/

Float32 F__F32FIR32_F32F32(Float32 Input,UInt16 NTabs,Float32* DelayLine,const Float32* Coeff)
{
UInt16    i;
Float32   Accu = 0.0F;
	
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
	   Accu += *DelayLine++ * *Coeff++;	    		
	}
	return  Accu;
}


