/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR0_222.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR0_222.c $ $Revision: 6 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__U32FIR32_U32U32(Input,NTabs,DelayLine,Coeff)
*
* DESCRIPTION:
*   FIR - Filter calculation.
*
* PARAMETERS:
*   type    name       meaning    
*   UInt32  Input      current Input 
*   UInt16  NTabs      number of tabs
*   UInt32* DelayLine  pointer to DelayLine vector
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


UInt32 F__U32FIR32_U32U32(UInt32 Input,UInt16 NTabs,UInt32* DelayLine,const UInt32* Coeff)
{
UInt16    i;
UInt32    Accu = 0;
	
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
	   Accu += (UInt32)*DelayLine++ * (UInt32)*Coeff++;	    		
	}
	return  Accu;
}


