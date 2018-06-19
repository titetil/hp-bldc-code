#ifndef __BVH2_App_Layer_sfun_c6_h__
#define __BVH2_App_Layer_sfun_c6_h__
#include "sfc_sf.h"
#include "sfc_mex.h"
extern void sf_BVH2_App_Layer_sfun_c6_get_check_sum(mxArray *plhs[]);
extern void BVH2_App_Layer_sfun_c6_registry(SimStruct *simStructPtr);
extern void BVH2_App_Layer_sfun_c6_sizes_registry(SimStruct *simStructPtr);
extern void BVH2_App_Layer_sfun_c6(void);

typedef struct SFBVH2_App_Layer_sfun_c6LocalDataStruct{
  uint16_T m0_c6_d1_StateCnt;
} SFBVH2_App_Layer_sfun_c6LocalDataStruct;
typedef struct SFBVH2_App_Layer_sfun_c6StateStruct{
  unsigned char is_active_BVH2_App_Layer_sfun_c6;
  unsigned char is_BVH2_App_Layer_sfun_c6;
  unsigned char is_m0_c6_s1_DryRunning;
} SFBVH2_App_Layer_sfun_c6StateStruct;

typedef struct S_SFBVH2_App_Layer_sfun_c6InstanceStruct {
  SFBVH2_App_Layer_sfun_c6LocalDataStruct LocalData;
  SFBVH2_App_Layer_sfun_c6StateStruct State;
  SimStruct *S;
  ChartInfoStruct chartInfo;
  unsigned int chartNumber;
  unsigned int instanceNumber;
} SFBVH2_App_Layer_sfun_c6InstanceStruct;

#endif

