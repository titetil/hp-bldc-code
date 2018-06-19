	
#ifndef __BVH2_App_Layer_sfun_h__
#define __BVH2_App_Layer_sfun_h__
#include <string.h>
#include <stdlib.h>
#include <math.h>
#ifndef min
#define min(a,b)    (((a) < (b)) ? (a) : (b))
#endif
#ifndef max
#define max(a,b)    (((a) > (b)) ? (a) : (b))
#endif
#define S_FUNCTION_NAME sf_sfun
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "sfcdebug.h"
#include "tmwtypes.h"



#define CALL_EVENT (255) /* Enumeration of all events for machine */
#ifndef _sfTime_
/* Stateflow time variable */
extern real_T _sfTime_;
#endif


extern uint8_T _sfEvent_;
#endif


