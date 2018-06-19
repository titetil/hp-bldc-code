/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR1_201.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR1_201.c $ $Revision: 7 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__U32FIR32_SAT_U8U16(Input,NTabs,DelayLine,Coeff)
*
* DESCRIPTION:
*   FIR - Filter calculation with saturation.
*
* PARAMETERS:
*   type    name       meaning    
*   UInt8   Input      current Input 
*   UInt16  NTabs      number of tabs
*   UInt8*  DelayLine  pointer to DelayLine vector
*   UInt16* Coeff      pointer to Coefficients vector 
*
* RETURNS:
*   UInt32  accumulation result 
*	
* NOTE:
*   
* HISTORY:
*
******************************************************************************/


UInt32 F__U32FIR32_SAT_U8U16(UInt8 Input,UInt16 NTabs,UInt8* DelayLine,const UInt16* Coeff)
{
UInt16    i;
UInt32    Mul;
UInt32    Accu   = 0;
UInt32    Accu_1 = 0;

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
	
	   Mul  = (UInt32)*DelayLine++ * (UInt32)*Coeff++;	 
	   Accu = Accu_1 + Mul; 

 	   if (  Accu < Accu_1  ) {
	      Accu = 0xFFFFFFFF;
	   }
   		
	   Accu_1 = Accu;
    }
	return  Accu;
}


