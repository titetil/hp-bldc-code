#ifndef __BVH2_App_Layer_sfun_c2_h__
#define __BVH2_App_Layer_sfun_c2_h__
#include "sfc_sf.h"
#include "sfc_mex.h"
extern void sf_BVH2_App_Layer_sfun_c2_get_check_sum(mxArray *plhs[]);
extern void BVH2_App_Layer_sfun_c2_registry(SimStruct *simStructPtr);
extern void BVH2_App_Layer_sfun_c2_sizes_registry(SimStruct *simStructPtr);
extern void BVH2_App_Layer_sfun_c2(void);

typedef struct SFBVH2_App_Layer_sfun_c2ConstantDataStruct{
  uint8_T m0_c2_d10_UbatThresMax;
  uint8_T m0_c2_d8_UbatThresLimpHigh;
  uint8_T m0_c2_d11_UbatThresMin;
  uint8_T m0_c2_d9_UbatThresLimpLow;
} SFBVH2_App_Layer_sfun_c2ConstantDataStruct;
typedef struct SFBVH2_App_Layer_sfun_c2StateStruct{
  unsigned char is_active_BVH2_App_Layer_sfun_c2;
  unsigned char is_active_m0_c2_s1_Ubat_Handling;
  unsigned char is_m0_c2_s1_Ubat_Handling;
} SFBVH2_App_Layer_sfun_c2StateStruct;

typedef struct S_SFBVH2_App_Layer_sfun_c2InstanceStruct {
  SFBVH2_App_Layer_sfun_c2ConstantDataStruct ConstantData;
  SFBVH2_App_Layer_sfun_c2StateStruct State;
  SimStruct *S;
  ChartInfoStruct chartInfo;
  unsigned int chartNumber;
  unsigned int instanceNumber;
} SFBVH2_App_Layer_sfun_c2InstanceStruct;

#endif

