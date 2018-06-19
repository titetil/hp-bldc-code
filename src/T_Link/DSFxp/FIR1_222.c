/******************************************************************************
*                                                                              
* FILE:                                                                        
*   FIR1_222.c         2000/03/22                                              
*                                                                              
*  Copyright (c) 2000 dSPACE GmbH, GERMANY                                     
*                                                                              
*  $Workfile: FIR1_222.c $ $Revision: 7 $ $Date: 14.01.04 16:27 $ $Author: Markuss $                           
******************************************************************************/

#include "dsfxp.h"

/******************************************************************************
*
* Function:
*   F__U32FIR32_SAT_U32U32(Input,NTabs,DelayLine,Coeff)
*
* DESCRIPTION:
*   FIR - Filter calculation with saturation.
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

UInt32 F__U32FIR32_SAT_U32U32(UInt32 Input,UInt16 NTabs,UInt32* DelayLine,const UInt32* Coeff)
{
UInt32    Mul_h;
UInt32    Mul_l;
UInt32    Accu_h = 0;
UInt32    Accu_l = 0;
UInt16    i;

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
	   C__U64MULU32U32(*DelayLine, *Coeff, Mul_h, Mul_l);
	   C__U64ADDU64U32_SAT(Mul_h, Mul_l, Accu_l, Accu_h, Accu_l);

       if(Accu_h) Accu_l = 0xFFFFFFFF;
       
       DelayLine++;Coeff++;

    }
	return  Accu_l;
}


