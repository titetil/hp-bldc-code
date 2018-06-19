/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR0_545.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR0_545.c $ $Revision: 6 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__I16FIR16_I8I16(Input,NTabs,DelayLine,Coeff)
*
* DESCRIPTION:
*   FIR - Filter calculation.
*
* PARAMETERS:
*   type    name       meaning    
*   Int8    Input      current Input 
*   UInt16  NTabs      number of tabs
*   Int8*   DelayLine  pointer to DelayLine vector
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

Int16 F__I16FIR16_I8I16(Int8 Input,UInt16 NTabs,Int8* DelayLine,const Int16* Coeff)
{
UInt16    i;
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
	   Accu +=(Int16)((Int32)*DelayLine++ * (Int32)*Coeff++);	    		
	}
	return  Accu;
}


