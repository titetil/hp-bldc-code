#ifndef __BVH2_App_Layer_sfun_c8_h__
#define __BVH2_App_Layer_sfun_c8_h__
#include "sfc_sf.h"
#include "sfc_mex.h"
extern void sf_BVH2_App_Layer_sfun_c8_get_check_sum(mxArray *plhs[]);
extern void BVH2_App_Layer_sfun_c8_registry(SimStruct *simStructPtr);
extern void BVH2_App_Layer_sfun_c8_sizes_registry(SimStruct *simStructPtr);
extern void BVH2_App_Layer_sfun_c8(void);

typedef struct SFBVH2_App_Layer_sfun_c8LocalDataStruct{
  uint8_T m0_c8_d2_DTCs[8];
  uint8_T m0_c8_d9_index;
  uint8_T m0_c8_d1_DTC;
  uint8_T m0_c8_d3_NDTC;
  boolean_T m0_c8_d4_NoDTC;
  uint8_T m0_c8_d7_TempDTC;
  uint8_T m0_c8_d8_count;
  uint8_T m0_c8_d6_SavedDTC;
  boolean_T m0_c8_d5_ST7_Ready;
} SFBVH2_App_Layer_sfun_c8LocalDataStruct;

typedef struct SFBVH2_App_Layer_sfun_c8ConstantDataStruct{
  uint8_T m0_c8_d21_WAITTIME;
  uint16_T m0_c8_d20_Low_Freq;
  uint16_T m0_c8_d19_High_Freq;
} SFBVH2_App_Layer_sfun_c8ConstantDataStruct;
typedef struct SFBVH2_App_Layer_sfun_c8StateStruct{
  unsigned char is_active_BVH2_App_Layer_sfun_c8;
  unsigned char is_active_m0_c8_s1_PWMinput_handling;
  unsigned char is_m0_c8_s1_PWMinput_handling;
} SFBVH2_App_Layer_sfun_c8StateStruct;

typedef struct S_SFBVH2_App_Layer_sfun_c8InstanceStruct {
  SFBVH2_App_Layer_sfun_c8LocalDataStruct LocalData;
  SFBVH2_App_Layer_sfun_c8ConstantDataStruct ConstantData;
  SFBVH2_App_Layer_sfun_c8StateStruct State;
  SimStruct *S;
  ChartInfoStruct chartInfo;
  unsigned int chartNumber;
  unsigned int instanceNumber;
} SFBVH2_App_Layer_sfun_c8InstanceStruct;

#endif

