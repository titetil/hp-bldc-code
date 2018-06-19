#ifndef __BVH2_App_Layer_sfun_c5_h__
#define __BVH2_App_Layer_sfun_c5_h__
#include "sfc_sf.h"
#include "sfc_mex.h"
extern void sf_BVH2_App_Layer_sfun_c5_get_check_sum(mxArray *plhs[]);
extern void BVH2_App_Layer_sfun_c5_registry(SimStruct *simStructPtr);
extern void BVH2_App_Layer_sfun_c5_sizes_registry(SimStruct *simStructPtr);
extern void BVH2_App_Layer_sfun_c5(void);

typedef struct SFBVH2_App_Layer_sfun_c5LocalDataStruct{
  uint8_T m0_c5_d1_StateCnt;
} SFBVH2_App_Layer_sfun_c5LocalDataStruct;
typedef struct SFBVH2_App_Layer_sfun_c5StateStruct{
  unsigned char is_active_BVH2_App_Layer_sfun_c5;
  unsigned char is_BVH2_App_Layer_sfun_c5;
} SFBVH2_App_Layer_sfun_c5StateStruct;

typedef struct S_SFBVH2_App_Layer_sfun_c5InstanceStruct {
  SFBVH2_App_Layer_sfun_c5LocalDataStruct LocalData;
  SFBVH2_App_Layer_sfun_c5StateStruct State;
  SimStruct *S;
  ChartInfoStruct chartInfo;
  unsigned int chartNumber;
  unsigned int instanceNumber;
} SFBVH2_App_Layer_sfun_c5InstanceStruct;

#endif

