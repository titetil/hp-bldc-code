/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR1_666.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR1_666.c $ $Revision: 9 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__I32FIR32_SAT_I32I32(Input,NTabs,DelayLine,Coeff)
*
* DESCRIPTION:
*   FIR - Filter calculation with saturation.
*
* PARAMETERS:
*   type    name       meaning    
*   Int32   Input      current Input 
*   UInt16  NTabs      number of tabs
*   Int32*  DelayLine  pointer to DelayLine vector
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

Int32 F__I32FIR32_SAT_I32I32(Int32 Input,UInt16 NTabs,Int32* DelayLine,const Int32* Coeff)
{
UInt16    i;
Int32     Mul_h;
UInt32    Mul_l;
Int32     Accu   = 0;
Int32     Accu_h = 0;
UInt32    Accu_l = 0;

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
	   C__I64MULI32I32(*DelayLine, *Coeff, Mul_h, Mul_l);
	   C__I64ADDI64I32(Mul_h, Mul_l, Accu, Accu_h, Accu_l);

       if(Accu_h > 0)      Accu = 0x7FFFFFFF;
       else if(Accu_h < 0) Accu = 0x80000000;
	   else                Accu = (Int32)Accu_l;
       DelayLine++;Coeff++; 
    }
	return  Accu;
}


