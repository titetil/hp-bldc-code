#ifndef __BVH2_App_Layer_sfun_c4_h__
#define __BVH2_App_Layer_sfun_c4_h__
#include "sfc_sf.h"
#include "sfc_mex.h"
extern void sf_BVH2_App_Layer_sfun_c4_get_check_sum(mxArray *plhs[]);
extern void BVH2_App_Layer_sfun_c4_registry(SimStruct *simStructPtr);
extern void BVH2_App_Layer_sfun_c4_sizes_registry(SimStruct *simStructPtr);
extern void BVH2_App_Layer_sfun_c4(void);

typedef struct SFBVH2_App_Layer_sfun_c4LocalDataStruct{
  uint16_T m0_c4_d1_StateCnt;
} SFBVH2_App_Layer_sfun_c4LocalDataStruct;
typedef struct SFBVH2_App_Layer_sfun_c4StateStruct{
  unsigned char is_active_BVH2_App_Layer_sfun_c4;
  unsigned char is_BVH2_App_Layer_sfun_c4;
} SFBVH2_App_Layer_sfun_c4StateStruct;

typedef struct S_SFBVH2_App_Layer_sfun_c4InstanceStruct {
  SFBVH2_App_Layer_sfun_c4LocalDataStruct LocalData;
  SFBVH2_App_Layer_sfun_c4StateStruct State;
  SimStruct *S;
  ChartInfoStruct chartInfo;
  unsigned int chartNumber;
  unsigned int instanceNumber;
} SFBVH2_App_Layer_sfun_c4InstanceStruct;

#endif

