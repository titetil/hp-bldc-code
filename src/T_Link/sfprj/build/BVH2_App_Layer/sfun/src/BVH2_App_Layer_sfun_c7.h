#ifndef __BVH2_App_Layer_sfun_c7_h__
#define __BVH2_App_Layer_sfun_c7_h__
#include "sfc_sf.h"
#include "sfc_mex.h"
extern void sf_BVH2_App_Layer_sfun_c7_get_check_sum(mxArray *plhs[]);
extern void BVH2_App_Layer_sfun_c7_registry(SimStruct *simStructPtr);
extern void BVH2_App_Layer_sfun_c7_sizes_registry(SimStruct *simStructPtr);
extern void BVH2_App_Layer_sfun_c7(void);

typedef struct SFBVH2_App_Layer_sfun_c7LocalDataStruct{
  uint16_T m0_c7_d3_StateCnt;
  uint16_T m0_c7_d1_BadCnt;
  uint8_T m0_c7_d2_RestartCounter;
} SFBVH2_App_Layer_sfun_c7LocalDataStruct;

typedef struct SFBVH2_App_Layer_sfun_c7ConstantDataStruct{
  uint16_T m0_c7_d11_TresholdLow_Speed;
  boolean_T m0_c7_d12_false;
  boolean_T m0_c7_d13_true;
  uint16_T m0_c7_d10_ThresholdHigh_Speed;
} SFBVH2_App_Layer_sfun_c7ConstantDataStruct;
typedef struct SFBVH2_App_Layer_sfun_c7StateStruct{
  unsigned char is_active_BVH2_App_Layer_sfun_c7;
  unsigned char is_BVH2_App_Layer_sfun_c7;
  unsigned char is_m0_c7_s1_Motor_stalled_Statemachine;
} SFBVH2_App_Layer_sfun_c7StateStruct;

typedef struct S_SFBVH2_App_Layer_sfun_c7InstanceStruct {
  SFBVH2_App_Layer_sfun_c7LocalDataStruct LocalData;
  SFBVH2_App_Layer_sfun_c7ConstantDataStruct ConstantData;
  SFBVH2_App_Layer_sfun_c7StateStruct State;
  SimStruct *S;
  ChartInfoStruct chartInfo;
  unsigned int chartNumber;
  unsigned int instanceNumber;
} SFBVH2_App_Layer_sfun_c7InstanceStruct;

#endif

