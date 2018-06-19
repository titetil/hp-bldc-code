#ifndef __BVH2_App_Layer_sfun_c3_h__
#define __BVH2_App_Layer_sfun_c3_h__
#include "sfc_sf.h"
#include "sfc_mex.h"
extern void sf_BVH2_App_Layer_sfun_c3_get_check_sum(mxArray *plhs[]);
extern void BVH2_App_Layer_sfun_c3_registry(SimStruct *simStructPtr);
extern void BVH2_App_Layer_sfun_c3_sizes_registry(SimStruct *simStructPtr);
extern void BVH2_App_Layer_sfun_c3(void);

typedef struct SFBVH2_App_Layer_sfun_c3LocalDataStruct{
  uint8_T m0_c3_d1_Counter;
} SFBVH2_App_Layer_sfun_c3LocalDataStruct;

typedef struct SFBVH2_App_Layer_sfun_c3ConstantDataStruct{
  uint16_T m0_c3_d9_Treshold_red;
  uint16_T m0_c3_d8_Treshold_green;
  uint16_T m0_c3_d7_Threshold_RedAlarm;
} SFBVH2_App_Layer_sfun_c3ConstantDataStruct;
typedef struct SFBVH2_App_Layer_sfun_c3StateStruct{
  unsigned char is_active_BVH2_App_Layer_sfun_c3;
  unsigned char is_BVH2_App_Layer_sfun_c3;
} SFBVH2_App_Layer_sfun_c3StateStruct;

typedef struct S_SFBVH2_App_Layer_sfun_c3InstanceStruct {
  SFBVH2_App_Layer_sfun_c3LocalDataStruct LocalData;
  SFBVH2_App_Layer_sfun_c3ConstantDataStruct ConstantData;
  SFBVH2_App_Layer_sfun_c3StateStruct State;
  SimStruct *S;
  ChartInfoStruct chartInfo;
  unsigned int chartNumber;
  unsigned int instanceNumber;
} SFBVH2_App_Layer_sfun_c3InstanceStruct;

#endif

