/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR3_656.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR3_656.c $ $Revision: 6 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__I32FIR32_SAT_I16CBI32(Input,NTabs,DelayLine,Coeff,Counter)
*
* DESCRIPTION:
*   FIR - Filter calculation with circular buffer and saturation.
*
* PARAMETERS:
*   type     name       meaning    
*   Int16    Input      current Input 
*   UInt16   NTabs      number of tabs
*   UInt16*  Counter    pointer to the Counter variable (to work in conjunction with circular buffer)
*   Int16*   DelayLine  pointer to DelayLine vector
*   Int32*   Coeff      pointer to Coefficients vector 
*
* RETURNS:
*   Int32   accumulation result 
*	
* NOTE:
*   
* HISTORY:
*
******************************************************************************/

Int32 F__I32FIR32_SAT_I16CBI32(Int16 Input,UInt16 NTabs,Int16* DelayLine,const Int32* Coeff,UInt16* Counter)
{
Int32     Mul_h;
UInt32    Mul_l;
Int32     Accu   = 0;
Int32     Accu_h = 0;
UInt32    Accu_l = 0;
Int16    *AuxDelayLine;
UInt16    i;


	if (*Counter) (*Counter)--;
	else          (*Counter) = (UInt16)(NTabs-1);

	AuxDelayLine = DelayLine + (*Counter);

	/* Update */
	*AuxDelayLine = Input; 

	/* Accumulation */
	for(i=0;i<NTabs;i++)
	{
	   C__I64MULI32I32((Int32)*AuxDelayLine, *Coeff, Mul_h, Mul_l);
	   C__I64ADDI64I32(Mul_h, Mul_l, Accu, Accu_h, Accu_l); 

       if(Accu_h > 0)      Accu = 0x7FFFFFFF;
       else if(Accu_h < 0) Accu = 0x80000000;
	   else                Accu = (Int32)Accu_l;
	   
	   AuxDelayLine++;Coeff++;	 
	   if(AuxDelayLine > (DelayLine + (NTabs-1))) AuxDelayLine = DelayLine;

    }
	return  Accu;
}


