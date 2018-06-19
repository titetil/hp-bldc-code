/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR0_999.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR0_999.c $ $Revision: 6 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
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
*   Float64  Input      current Input 
*   UInt16   NTabs      number of tabs
*   Float64* DelayLine  pointer to DelayLine vector
*   Float64* Coeff      pointer to Coefficients vector 
*
* RETURNS:
*   Float64  accumulation result 
*	
* NOTE:
*   
* HISTORY:
*
******************************************************************************/

Float64 F__F64FIR64_F64F64(Float64 Input,UInt16 NTabs,Float64* DelayLine,const Float64* Coeff)
{
UInt16    i;
Float64   Accu = 0.0L;
	
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


