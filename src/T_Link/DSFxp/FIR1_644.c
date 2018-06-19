/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR1_644.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR1_644.c $ $Revision: 8 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__I32FIR32_SAT_I8I8(Input,NTabs,DelayLine,Coeff)
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
*   Int32   accumulation result 
*	
* NOTE:
*   
* HISTORY:
*
******************************************************************************/

Int32 F__I32FIR32_SAT_I8I8(Int8 Input,UInt16 NTabs,Int8* DelayLine,const Int8* Coeff)
{
UInt16   i;
Int16    Mul;
Int32    Accu   = 0;
Int32    Accu_1 = 0;
	
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
	    Accu = Accu_1 + (Int32)Mul; 
	    		
       if ( ( Accu_1 >= 0) && (Mul >= 0) && (Accu < 0) ) 
       {
           Accu = INT32MAX;
	   } 
	   else 
	   {
  	      if (  ( Accu_1 < 0) && (Mul < 0) && (Accu >= 0)  )
  	       {
      		 Accu = INT32MIN;
		   }
	   }

	   Accu_1 = Accu;
	}
	return  Accu;
}


