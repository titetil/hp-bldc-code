/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR1_100.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR1_100.c $ $Revision: 8 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__U16FIR16_SAT_U8U8(Input,NTabs,DelayLine,Coeff)
*
* DESCRIPTION:
*   FIR - Filter calculation with saturation.
*
* PARAMETERS:
*   type    name       meaning    
*   UInt8   Input      current Input 
*   UInt16  NTabs      number of tabs
*   UInt8*  DelayLine  pointer to DelayLine vector
*   UInt8*  Coeff      pointer to Coefficients vector 
*
* RETURNS:
*   UInt16  accumulation result 
*	
* NOTE:
*   
* HISTORY:
*
******************************************************************************/

UInt16 F__U16FIR16_SAT_U8U8(UInt8 Input,UInt16 NTabs,UInt8* DelayLine,const UInt8* Coeff)
{
UInt16    i;
UInt16    Mul;
UInt16    Accu   = 0;
UInt16    Accu_1 = 0;
	
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
		Mul  = (UInt16)((UInt16)*DelayLine++ * (UInt16)*Coeff++);	 
	    Accu = (UInt16)(Accu_1 + Mul); 
       
		if (  Accu < Accu_1  ) {
		  Accu = 0xFFFF;
		}
		
	   Accu_1 = Accu;
	}
	return  Accu;
}


