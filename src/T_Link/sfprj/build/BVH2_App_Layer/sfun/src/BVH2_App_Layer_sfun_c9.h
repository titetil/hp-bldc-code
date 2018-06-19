#ifndef __BVH2_App_Layer_sfun_c9_h__
#define __BVH2_App_Layer_sfun_c9_h__
#include "sfc_sf.h"
#include "sfc_mex.h"
extern void sf_BVH2_App_Layer_sfun_c9_get_check_sum(mxArray *plhs[]);
extern void BVH2_App_Layer_sfun_c9_registry(SimStruct *simStructPtr);
extern void BVH2_App_Layer_sfun_c9_sizes_registry(SimStruct *simStructPtr);
extern void BVH2_App_Layer_sfun_c9(void);

typedef struct SFBVH2_App_Layer_sfun_c9LocalDataStruct{
  uint16_T m0_c9_d1_Counter;
} SFBVH2_App_Layer_sfun_c9LocalDataStruct;
typedef struct SFBVH2_App_Layer_sfun_c9StateStruct{
  unsigned char is_active_BVH2_App_Layer_sfun_c9;
  unsigned char is_BVH2_App_Layer_sfun_c9;
  unsigned char is_m0_c9_s1_DryRunningAlarm;
} SFBVH2_App_Layer_sfun_c9StateStruct;

typedef struct S_SFBVH2_App_Layer_sfun_c9InstanceStruct {
  SFBVH2_App_Layer_sfun_c9LocalDataStruct LocalData;
  SFBVH2_App_Layer_sfun_c9StateStruct State;
  SimStruct *S;
  ChartInfoStruct chartInfo;
  unsigned int chartNumber;
  unsigned int instanceNumber;
} SFBVH2_App_Layer_sfun_c9InstanceStruct;

#endif

