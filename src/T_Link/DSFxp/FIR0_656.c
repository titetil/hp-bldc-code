/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR0_656.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR0_656.c $ $Revision: 4 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*  F__I32FIR32_I16I32(Input,NTabs,DelayLine,Coeff)
*
* DESCRIPTION:
*   FIR - Filter calculation.
*
* PARAMETERS:
*   type    name       meaning    
*   Int16   Input      current Input 
*   UInt16  NTabs      number of tabs
*   Int16*  DelayLine  pointer to DelayLine vector
*   Int32*  Coeff      pointer to Coefficients vector 
*
* RETURNS:
*   Int32   accumulation result 
*	
* NOTE:
*   
* HISTORY:
*
******************************************************************************/

Int32 F__I32FIR32_I16I32(Int16 Input,UInt16 NTabs,Int16* DelayLine,const Int32* Coeff)
{
UInt16    i;
UInt32    Accu;
Int32     Mul_h;
UInt32    Mul_l;
	
	/* Update */
	for(i=0;i<NTabs-1;i++)
	{
	  *DelayLine = *(DelayLine-1);  
	   DelayLine--;
	}
	*DelayLine = Input;
	
	Accu = 0;	
	/* Accumulation */
	for(i=0;i<NTabs;i++)
	{
	   C__I64MULI32I32((Int32)*DelayLine,*Coeff,Mul_h,Mul_l);
	   DelayLine++;Coeff++;
	   Accu += Mul_l;
	}
	return  (Int32)Accu;
}


