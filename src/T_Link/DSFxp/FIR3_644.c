/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR3_644.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR3_644.c $ $Revision: 7 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__I32FIR32_SAT_I8CBI8(Input,NTabs,DelayLine,Coeff,Counter)
*
* DESCRIPTION:
*   FIR - Filter calculation with circular buffer and saturation.
*
* PARAMETERS:
*   type     name       meaning    
*   Int8     Input      current Input 
*   UInt16   NTabs      number of tabs
*   UInt16*  Counter    pointer to the Counter variable (to work in conjunction with circular buffer)
*   Int8*    DelayLine  pointer to DelayLine vector
*   Int8*    Coeff      pointer to Coefficients vector 
*
* RETURNS:
*   Int32   accumulation result 
*	
* NOTE:
*   
* HISTORY:
*
******************************************************************************/

Int32 F__I32FIR32_SAT_I8CBI8(Int8 Input,UInt16 NTabs,Int8* DelayLine,const Int8* Coeff,UInt16* Counter)
{
Int32    Accu   = 0;
Int32    Accu_1 = 0;
UInt16   i;
Int16    Mul;
Int8    *AuxDelayLine;
	
	if (*Counter) (*Counter)--;
	else          (*Counter) = (UInt16)(NTabs-1);

	AuxDelayLine = DelayLine + (*Counter);

	/* Update */
	*AuxDelayLine = Input; 

	
	/* Accumulation */
	for(i=0;i<NTabs;i++)
	{
		Mul  = (Int16)((Int16)*AuxDelayLine++ * (Int16)*Coeff++);	 
	    Accu = Accu_1 + (Int32)Mul; 
	    		
       if ( ( Accu_1 >= 0) && (Mul >= 0) && (Accu < 0) ) 
       {
           Accu = 0x7FFFFFFF;
	   } 
	   else 
	   {
  	      if (  ( Accu_1 < 0) && (Mul < 0) && (Accu >= 0)  )
  	       {
      		 Accu = 0x80000000;
		   }
	   }
	   if(AuxDelayLine > (DelayLine + (NTabs-1))) AuxDelayLine = DelayLine;
	   Accu_1 = Accu;
	}
	return  Accu;
}


