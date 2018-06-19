#ifndef __BVH2_App_Layer_sfun_c1_h__
#define __BVH2_App_Layer_sfun_c1_h__
#include "sfc_sf.h"
#include "sfc_mex.h"
extern void sf_BVH2_App_Layer_sfun_c1_get_check_sum(mxArray *plhs[]);
extern void BVH2_App_Layer_sfun_c1_registry(SimStruct *simStructPtr);
extern void BVH2_App_Layer_sfun_c1_sizes_registry(SimStruct *simStructPtr);
extern void BVH2_App_Layer_sfun_c1(void);

typedef struct SFBVH2_App_Layer_sfun_c1LocalDataStruct{
  uint16_T m0_c1_d2_StateCnt;
  uint8_T m0_c1_d1_RestartCounter;
} SFBVH2_App_Layer_sfun_c1LocalDataStruct;
typedef struct SFBVH2_App_Layer_sfun_c1StateStruct{
  unsigned char is_active_BVH2_App_Layer_sfun_c1;
  unsigned char is_BVH2_App_Layer_sfun_c1;
} SFBVH2_App_Layer_sfun_c1StateStruct;

typedef struct S_SFBVH2_App_Layer_sfun_c1InstanceStruct {
  SFBVH2_App_Layer_sfun_c1LocalDataStruct LocalData;
  SFBVH2_App_Layer_sfun_c1StateStruct State;
  SimStruct *S;
  ChartInfoStruct chartInfo;
  unsigned int chartNumber;
  unsigned int instanceNumber;
} SFBVH2_App_Layer_sfun_c1InstanceStruct;

#endif

