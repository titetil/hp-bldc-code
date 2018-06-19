#include "BVH2_App_Layer_sfun.h"
#include "BVH2_App_Layer_sfun_c7.h"
#define mexPrintf                       sf_mex_printf
#ifdef printf
#undef printf
#endif
#define printf                          sf_mex_printf
#define CHARTINSTANCE_CHARTNUMBER       (chartInstance.chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER    (chartInstance.instanceNumber)
#include "BVH2_App_Layer_sfun_debug_macros.h"
#define IN_NO_ACTIVE_CHILD              (0)
#define IN_m0_c7_s1_Motor_stalled_Statemachine 1
#define IN_m0_c7_s8_Stop                2
#define IN_m0_c7_s9_default             3
#define IN_m0_c7_s2_CntStart            1
#define IN_m0_c7_s3_Motor_Run           2
#define IN_m0_c7_s4_Motor_Stalled       3
#define IN_m0_c7_s5_Motor_Wait          4
#define IN_m0_c7_s6_SpeedBad_cnt        5
#define IN_m0_c7_s7_Speed_detection     6
#define m0_c7_d11_TresholdLow_Speed     chartInstance.ConstantData.m0_c7_d11_TresholdLow_Speed
#define m0_c7_d12_false                 chartInstance.ConstantData.m0_c7_d12_false
#define m0_c7_d13_true                  chartInstance.ConstantData.m0_c7_d13_true
#define m0_c7_d10_ThresholdHigh_Speed   chartInstance.ConstantData.m0_c7_d10_ThresholdHigh_Speed
static SFBVH2_App_Layer_sfun_c7InstanceStruct chartInstance;
#define InputData_m0_c7_d4_iMotor_off   (((boolean_T *)(ssGetInputPortSignal(chartInstance.S,0)))[0])
#define InputData_m0_c7_d5_Reset        (((boolean_T *)(ssGetInputPortSignal(chartInstance.S,1)))[0])
#define InputData_m0_c7_d6_iSpeed       (((uint16_T *)(ssGetInputPortSignal(chartInstance.S,2)))[0])
#define InputData_m0_c7_d7_iCurrentAlarm (((boolean_T *)(ssGetInputPortSignal(chartInstance.S,3)))[0])
#define OutputData_m0_c7_d8_oMotorStalled (((boolean_T *)(ssGetOutputPortSignal(chartInstance.S,1)))[0])
#define OutputData_m0_c7_d9_oStalledAlarm (((boolean_T *)(ssGetOutputPortSignal(chartInstance.S,2)))[0])
static uint16_T m0_c7_u16_s8_(int8_T b);
static uint16_T m0_c7_u16_s32_(int32_T b);
static uint8_T m0_c7_u8_s32_(int32_T b);
static uint8_T m0_c7_u8_s8_(int8_T b);

static uint16_T m0_c7_u16_s8_(int8_T b)
{
  uint16_T a;
  a = (uint16_T)b;
  if(b < 0) {
    sf_debug_overflow_detection(0);
  }
  return a;
}
static uint16_T m0_c7_u16_s32_(int32_T b)
{
  uint16_T a;
  a = (uint16_T)b;
  if(a != b) {
    sf_debug_overflow_detection(0);
  }
  return a;
}
static uint8_T m0_c7_u8_s32_(int32_T b)
{
  uint8_T a;
  a = (uint8_T)b;
  if(a != b) {
    sf_debug_overflow_detection(0);
  }
  return a;
}
static uint8_T m0_c7_u8_s8_(int8_T b)
{
  uint8_T a;
  a = (uint8_T)b;
  if(b < 0) {
    sf_debug_overflow_detection(0);
  }
  return a;
}

static void exit_atomic_m0_c7_s1_Motor_stalled_Statemachine(void);
static void exit_internal_m0_c7_s1_Motor_stalled_Statemachine(void);
static void exit_atomic_m0_c7_s2_CntStart(void);
static void enter_atomic_m0_c7_s3_Motor_Run(void);
static void exit_atomic_m0_c7_s3_Motor_Run(void);
static void enter_atomic_m0_c7_s4_Motor_Stalled(void);
static void exit_atomic_m0_c7_s4_Motor_Stalled(void);
static void exit_atomic_m0_c7_s5_Motor_Wait(void);
static void exit_atomic_m0_c7_s6_SpeedBad_cnt(void);
static void enter_atomic_m0_c7_s7_Speed_detection(void);
static void exit_atomic_m0_c7_s7_Speed_detection(void);
static void enter_atomic_m0_c7_s9_default(void);
static void exit_atomic_m0_c7_s9_default(void);
void BVH2_App_Layer_sfun_c7(void)
{
  real_T __sfTemp1;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,6);
  if(chartInstance.State.is_active_BVH2_App_Layer_sfun_c7 == 0) {
    _SFD_CC_CALL(CHART_ENTER_ENTRY_FUNCTION_TAG,6);
    chartInstance.State.is_active_BVH2_App_Layer_sfun_c7 = 1;
    _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,6);
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,7);
    if(CV_TRANSITION_EVAL(7, _SFD_CCP_CALL(7,0,(InputData_m0_c7_d5_Reset ==
        m0_c7_d13_true))) != 0) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,7);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,7);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,7);
      enter_atomic_m0_c7_s9_default();
    }
  } else {
    switch(chartInstance.State.is_BVH2_App_Layer_sfun_c7) {
     case IN_m0_c7_s1_Motor_stalled_Statemachine:
      CV_CHART_EVAL(6,0,1);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,3);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,3,0);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,4);
      if(CV_TRANSITION_EVAL(4, (_SFD_CCP_CALL(4,0,(InputData_m0_c7_d4_iMotor_off
           == m0_c7_d13_true)) != 0) || (_SFD_CCP_CALL(4,1,(
           InputData_m0_c7_d5_Reset == m0_c7_d13_true)) != 0)) != 0) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,4);
        _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,4);
        _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,4);
        exit_internal_m0_c7_s1_Motor_stalled_Statemachine();
        exit_atomic_m0_c7_s1_Motor_stalled_Statemachine();
        enter_atomic_m0_c7_s9_default();
      } else {
        switch(chartInstance.State.is_m0_c7_s1_Motor_stalled_Statemachine) {
         case IN_m0_c7_s2_CntStart:
          CV_STATE_EVAL(3,0,1);
          _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,4);
          _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,4,0);
          _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,15);
          if(CV_TRANSITION_EVAL(15,
            _SFD_CCP_CALL(15,0,(chartInstance.LocalData.m0_c7_d3_StateCnt > 0)))
           != 0) {
            _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,15);
            _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,15);
            _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,15);
            exit_atomic_m0_c7_s2_CntStart();
            enter_atomic_m0_c7_s7_Speed_detection();
          } else {
            _SFD_CS_CALL(STATE_BEFORE_DURING_ACTION_TAG,4);
            chartInstance.LocalData.m0_c7_d3_StateCnt =
              m0_c7_u16_s32_(chartInstance.LocalData.m0_c7_d3_StateCnt + 1);
            _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,4,1);
            _SFD_CS_CALL(STATE_AFTER_DURING_ACTION_TAG,4);
          }
          _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,4);
          break;
         case IN_m0_c7_s3_Motor_Run:
          CV_STATE_EVAL(3,0,2);
          _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,6);
          _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,6,0);
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,2);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,2);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,2);
          exit_atomic_m0_c7_s3_Motor_Run();
          enter_atomic_m0_c7_s7_Speed_detection();
          _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,6);
          break;
         case IN_m0_c7_s4_Motor_Stalled:
          CV_STATE_EVAL(3,0,3);
          _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,2);
          _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,2,0);
          _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,6);
          if(CV_TRANSITION_EVAL(6,
            _SFD_CCP_CALL(6,0,(chartInstance.LocalData.m0_c7_d3_StateCnt >
              200))) != 0) {
            _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,6);
            _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,6);
            _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,6);
            exit_atomic_m0_c7_s4_Motor_Stalled();
            _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,0);
            _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,0,0);
            chartInstance.State.is_m0_c7_s1_Motor_stalled_Statemachine =
              IN_m0_c7_s5_Motor_Wait;
            _SFD_CS_CALL(STATE_ACTIVE_TAG,0);
            _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,0);
            chartInstance.LocalData.m0_c7_d3_StateCnt = m0_c7_u16_s8_(0);
            _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,0,1);
            OutputData_m0_c7_d8_oMotorStalled = 0;
            _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,0,2);
            OutputData_m0_c7_d9_oStalledAlarm = 1;
            _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,0,3);
            chartInstance.LocalData.m0_c7_d2_RestartCounter =
              m0_c7_u8_s32_(chartInstance.LocalData.m0_c7_d2_RestartCounter + 1);
            _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,0,4);
            _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,0);
            _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
          } else {
            _SFD_CS_CALL(STATE_BEFORE_DURING_ACTION_TAG,2);
            sf_mex_printf("%s =\n", "StateCnt");
            chartInstance.LocalData.m0_c7_d3_StateCnt =
              m0_c7_u16_s32_(chartInstance.LocalData.m0_c7_d3_StateCnt + 1);
            __sfTemp1 = (real_T)chartInstance.LocalData.m0_c7_d3_StateCnt;
            ml_call_function("disp", 0, 1, 4, sf_mex_create_mx_array(&__sfTemp1,
              0, 0, 0));
            _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,2,1);
            _SFD_CS_CALL(STATE_AFTER_DURING_ACTION_TAG,2);
          }
          _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,2);
          break;
         case IN_m0_c7_s5_Motor_Wait:
          CV_STATE_EVAL(3,0,4);
          _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,0);
          _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,0,0);
          _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,1);
          if(CV_TRANSITION_EVAL(1,
            _SFD_CCP_CALL(1,0,(chartInstance.LocalData.m0_c7_d3_StateCnt > 10)))
           != 0) {
            _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,1);
            _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,1);
            _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,1);
            _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,10);
            if(CV_TRANSITION_EVAL(10,
              _SFD_CCP_CALL(10,0,(chartInstance.LocalData.m0_c7_d2_RestartCounter
                == 10))) != 0) {
              _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,10);
              _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,10);
              _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,10);
              exit_internal_m0_c7_s1_Motor_stalled_Statemachine();
              exit_atomic_m0_c7_s1_Motor_stalled_Statemachine();
              _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,1);
              _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,0);
              chartInstance.State.is_BVH2_App_Layer_sfun_c7 = IN_m0_c7_s8_Stop;
              _SFD_CS_CALL(STATE_ACTIVE_TAG,1);
              _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,1);
              OutputData_m0_c7_d9_oStalledAlarm = 1;
              _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,1);
              OutputData_m0_c7_d8_oMotorStalled = 1;
              _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,2);
              _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,1);
              _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,1);
              goto sf_label_1;
            } else {
              _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,0);
              if(CV_TRANSITION_EVAL(0,
                (_SFD_CCP_CALL(0,0,(InputData_m0_c7_d6_iSpeed <
                   m0_c7_d11_TresholdLow_Speed)) != 0) || (_SFD_CCP_CALL(0,1,(
                   InputData_m0_c7_d6_iSpeed > m0_c7_d10_ThresholdHigh_Speed))
                 != 0)) != 0) {
                if(sf_debug_transition_conflict_check_enabled()) {
                  unsigned int transitionList[2];
                  unsigned int numTransitions=1;
                  transitionList[0] = 0;
                  sf_debug_transition_conflict_check_begin();
                  if(chartInstance.LocalData.m0_c7_d3_StateCnt > 200) {
                    transitionList[numTransitions] = 13;
                    numTransitions++;
                  }
                  sf_debug_transition_conflict_check_end();
                  if(numTransitions>1) {
                    _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
                  }
                }
                _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,0);
                _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,0);
                _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,0);
                exit_atomic_m0_c7_s5_Motor_Wait();
                enter_atomic_m0_c7_s4_Motor_Stalled();
                goto sf_label_1;
              } else {
                _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,13);
                if(CV_TRANSITION_EVAL(13,
                  _SFD_CCP_CALL(13,0,(chartInstance.LocalData.m0_c7_d3_StateCnt
                    > 200))) != 0) {
                  _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,13);
                  _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,13);
                  _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,13);
                  exit_atomic_m0_c7_s5_Motor_Wait();
                  enter_atomic_m0_c7_s3_Motor_Run();
                  goto sf_label_1;
                }
              }
            }
          }
          _SFD_CS_CALL(STATE_BEFORE_DURING_ACTION_TAG,0);
          chartInstance.LocalData.m0_c7_d3_StateCnt =
            m0_c7_u16_s32_(chartInstance.LocalData.m0_c7_d3_StateCnt + 1);
          _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,0,1);
          _SFD_CS_CALL(STATE_AFTER_DURING_ACTION_TAG,0);
          sf_label_1:;
          _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
          break;
         case IN_m0_c7_s6_SpeedBad_cnt:
          CV_STATE_EVAL(3,0,5);
          _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,7);
          _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,7,0);
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,11);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,11);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,11);
          _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,9);
          if(CV_TRANSITION_EVAL(9,
            _SFD_CCP_CALL(9,0,(chartInstance.LocalData.m0_c7_d1_BadCnt > 100)))
           != 0) {
            if(sf_debug_transition_conflict_check_enabled()) {
              unsigned int transitionList[2];
              unsigned int numTransitions=1;
              transitionList[0] = 9;
              sf_debug_transition_conflict_check_begin();
              if((InputData_m0_c7_d6_iSpeed >= m0_c7_d11_TresholdLow_Speed) &&
               (InputData_m0_c7_d6_iSpeed <=
                m0_c7_d10_ThresholdHigh_Speed)) {
                transitionList[numTransitions] = 12;
                numTransitions++;
              }
              sf_debug_transition_conflict_check_end();
              if(numTransitions>1) {
                _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
              }
            }
            _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,9);
            _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,9);
            _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,9);
            exit_atomic_m0_c7_s6_SpeedBad_cnt();
            enter_atomic_m0_c7_s4_Motor_Stalled();
          } else {
            _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,12);
            if(CV_TRANSITION_EVAL(12,
              (_SFD_CCP_CALL(12,0,(InputData_m0_c7_d6_iSpeed >=
                 m0_c7_d11_TresholdLow_Speed)) != 0) &&
              (_SFD_CCP_CALL(12,1,(InputData_m0_c7_d6_iSpeed <=
                 m0_c7_d10_ThresholdHigh_Speed)) != 0)) != 0) {
              _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,12);
              _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,12);
              _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,12);
              exit_atomic_m0_c7_s6_SpeedBad_cnt();
              enter_atomic_m0_c7_s7_Speed_detection();
            } else {
              _SFD_CS_CALL(STATE_BEFORE_DURING_ACTION_TAG,7);
              chartInstance.LocalData.m0_c7_d1_BadCnt =
                m0_c7_u16_s32_(chartInstance.LocalData.m0_c7_d1_BadCnt + 1);
              _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,7,1);
              chartInstance.LocalData.m0_c7_d3_StateCnt =
                m0_c7_u16_s32_(chartInstance.LocalData.m0_c7_d3_StateCnt + 1);
              _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,7,2);
              _SFD_CS_CALL(STATE_AFTER_DURING_ACTION_TAG,7);
            }
          }
          _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,7);
          break;
         case IN_m0_c7_s7_Speed_detection:
          CV_STATE_EVAL(3,0,6);
          _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,8);
          _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,8,0);
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,14);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,14);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,14);
          _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,5);
          if(CV_TRANSITION_EVAL(5,
            _SFD_CCP_CALL(5,0,(chartInstance.LocalData.m0_c7_d3_StateCnt >
              1000))) != 0) {
            if(sf_debug_transition_conflict_check_enabled()) {
              unsigned int transitionList[2];
              unsigned int numTransitions=1;
              transitionList[0] = 5;
              sf_debug_transition_conflict_check_begin();
              if((InputData_m0_c7_d6_iSpeed < m0_c7_d11_TresholdLow_Speed) ||
               (InputData_m0_c7_d6_iSpeed >
                m0_c7_d10_ThresholdHigh_Speed)) {
                transitionList[numTransitions] = 3;
                numTransitions++;
              }
              sf_debug_transition_conflict_check_end();
              if(numTransitions>1) {
                _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
              }
            }
            _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,5);
            _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,5);
            _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,5);
            exit_atomic_m0_c7_s7_Speed_detection();
            enter_atomic_m0_c7_s3_Motor_Run();
          } else {
            _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,3);
            if(CV_TRANSITION_EVAL(3,
              (_SFD_CCP_CALL(3,0,(InputData_m0_c7_d6_iSpeed <
                 m0_c7_d11_TresholdLow_Speed)) != 0) || (_SFD_CCP_CALL(3,1,(
                 InputData_m0_c7_d6_iSpeed > m0_c7_d10_ThresholdHigh_Speed)) !=
               0)) != 0) {
              _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,3);
              _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,3);
              _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,3);
              exit_atomic_m0_c7_s7_Speed_detection();
              _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,7);
              _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,7,0);
              chartInstance.State.is_m0_c7_s1_Motor_stalled_Statemachine =
                IN_m0_c7_s6_SpeedBad_cnt;
              _SFD_CS_CALL(STATE_ACTIVE_TAG,7);
              _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,7);
              OutputData_m0_c7_d9_oStalledAlarm = 0;
              _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,7,1);
              chartInstance.LocalData.m0_c7_d3_StateCnt = m0_c7_u16_s8_(0);
              _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,7,2);
              _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,7);
              _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,7);
            } else {
              _SFD_CS_CALL(STATE_BEFORE_DURING_ACTION_TAG,8);
              chartInstance.LocalData.m0_c7_d3_StateCnt =
                m0_c7_u16_s32_(chartInstance.LocalData.m0_c7_d3_StateCnt + 1);
              _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,8,1);
              _SFD_CS_CALL(STATE_AFTER_DURING_ACTION_TAG,8);
            }
          }
          _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,8);
          break;
         default:
          CV_STATE_EVAL(3,0,0);
          break;
        }
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
      break;
     case IN_m0_c7_s8_Stop:
      CV_CHART_EVAL(6,0,2);
      break;
     case IN_m0_c7_s9_default:
      CV_CHART_EVAL(6,0,3);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,5);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,5,0);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,8);
      if(CV_TRANSITION_EVAL(8, _SFD_CCP_CALL(8,0,(InputData_m0_c7_d4_iMotor_off
          == m0_c7_d12_false))) != 0) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,8);
        _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,8);
        _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,8);
        exit_atomic_m0_c7_s9_default();
        _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,3);
        _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,3,0);
        chartInstance.State.is_BVH2_App_Layer_sfun_c7 =
          IN_m0_c7_s1_Motor_stalled_Statemachine;
        _SFD_CS_CALL(STATE_ACTIVE_TAG,3);
        _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
        _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,4);
        _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,4,0);
        chartInstance.State.is_m0_c7_s1_Motor_stalled_Statemachine =
          IN_m0_c7_s2_CntStart;
        _SFD_CS_CALL(STATE_ACTIVE_TAG,4);
        _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,4);
        chartInstance.LocalData.m0_c7_d3_StateCnt = m0_c7_u16_s8_(0);
        _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,4,1);
        chartInstance.LocalData.m0_c7_d1_BadCnt = m0_c7_u16_s8_(0);
        _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,4,2);
        _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,4);
        _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,4);
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,5);
      break;
     default:
      CV_CHART_EVAL(6,0,0);
      break;
    }
  }
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,6);
}

static void exit_atomic_m0_c7_s1_Motor_stalled_Statemachine(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,3);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,3,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c7 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,3);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
}

static void exit_internal_m0_c7_s1_Motor_stalled_Statemachine(void)
{
  switch(chartInstance.State.is_m0_c7_s1_Motor_stalled_Statemachine) {
   case IN_m0_c7_s2_CntStart:
    CV_STATE_EVAL(3,1,1);
    exit_atomic_m0_c7_s2_CntStart();
    break;
   case IN_m0_c7_s3_Motor_Run:
    CV_STATE_EVAL(3,1,2);
    exit_atomic_m0_c7_s3_Motor_Run();
    break;
   case IN_m0_c7_s4_Motor_Stalled:
    CV_STATE_EVAL(3,1,3);
    exit_atomic_m0_c7_s4_Motor_Stalled();
    break;
   case IN_m0_c7_s5_Motor_Wait:
    CV_STATE_EVAL(3,1,4);
    exit_atomic_m0_c7_s5_Motor_Wait();
    break;
   case IN_m0_c7_s6_SpeedBad_cnt:
    CV_STATE_EVAL(3,1,5);
    exit_atomic_m0_c7_s6_SpeedBad_cnt();
    break;
   case IN_m0_c7_s7_Speed_detection:
    CV_STATE_EVAL(3,1,6);
    exit_atomic_m0_c7_s7_Speed_detection();
    break;
   default:
    CV_STATE_EVAL(3,1,0);
    break;
  }
}

static void exit_atomic_m0_c7_s2_CntStart(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,4);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,4,0);
  _SFD_CS_CALL(STATE_BEFORE_EXIT_ACTION_TAG,4);
  chartInstance.LocalData.m0_c7_d3_StateCnt = m0_c7_u16_s8_(0);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,4,1);
  _SFD_CS_CALL(STATE_AFTER_EXIT_ACTION_TAG,4);
  chartInstance.State.is_m0_c7_s1_Motor_stalled_Statemachine =
    IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,4);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,4);
}

static void enter_atomic_m0_c7_s3_Motor_Run(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,6);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,6,0);
  chartInstance.State.is_m0_c7_s1_Motor_stalled_Statemachine =
    IN_m0_c7_s3_Motor_Run;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,6);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,6);
  chartInstance.LocalData.m0_c7_d3_StateCnt = m0_c7_u16_s8_(0);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,6,1);
  chartInstance.LocalData.m0_c7_d1_BadCnt = m0_c7_u16_s8_(0);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,6,2);
  OutputData_m0_c7_d8_oMotorStalled = 0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,6,3);
  OutputData_m0_c7_d9_oStalledAlarm = 0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,6,4);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,6);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,6);
}

static void exit_atomic_m0_c7_s3_Motor_Run(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,6);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,6,0);
  chartInstance.State.is_m0_c7_s1_Motor_stalled_Statemachine =
    IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,6);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,6);
}

static void enter_atomic_m0_c7_s4_Motor_Stalled(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,2);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,2,0);
  chartInstance.State.is_m0_c7_s1_Motor_stalled_Statemachine =
    IN_m0_c7_s4_Motor_Stalled;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,2);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,2);
  chartInstance.LocalData.m0_c7_d3_StateCnt = m0_c7_u16_s8_(0);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,2,1);
  OutputData_m0_c7_d8_oMotorStalled = 1;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,2,2);
  OutputData_m0_c7_d9_oStalledAlarm = 1;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,2,3);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,2);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,2);
}

static void exit_atomic_m0_c7_s4_Motor_Stalled(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,2);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,2,0);
  chartInstance.State.is_m0_c7_s1_Motor_stalled_Statemachine =
    IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,2);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,2);
}

static void exit_atomic_m0_c7_s5_Motor_Wait(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,0);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,0,0);
  chartInstance.State.is_m0_c7_s1_Motor_stalled_Statemachine =
    IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,0);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
}

static void exit_atomic_m0_c7_s6_SpeedBad_cnt(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,7);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,7,0);
  chartInstance.State.is_m0_c7_s1_Motor_stalled_Statemachine =
    IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,7);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,7);
}

static void enter_atomic_m0_c7_s7_Speed_detection(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,8);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,8,0);
  chartInstance.State.is_m0_c7_s1_Motor_stalled_Statemachine =
    IN_m0_c7_s7_Speed_detection;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,8);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,8);
}

static void exit_atomic_m0_c7_s7_Speed_detection(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,8);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,8,0);
  chartInstance.State.is_m0_c7_s1_Motor_stalled_Statemachine =
    IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,8);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,8);
}

static void enter_atomic_m0_c7_s9_default(void)
{
  real_T __sfTemp1;
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,5);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,5,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c7 = IN_m0_c7_s9_default;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,5);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,5);
  OutputData_m0_c7_d8_oMotorStalled = 0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,5,1);
  OutputData_m0_c7_d9_oStalledAlarm = 0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,5,2);
  sf_mex_printf("%s =\n", "RestartCounter");
  chartInstance.LocalData.m0_c7_d2_RestartCounter = m0_c7_u8_s8_(0);
  __sfTemp1 = (real_T)chartInstance.LocalData.m0_c7_d2_RestartCounter;
  ml_call_function("disp", 0, 1, 4, sf_mex_create_mx_array(&__sfTemp1, 0, 0, 0));
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,5,3);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,5);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,5);
}

static void exit_atomic_m0_c7_s9_default(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,5);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,5,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c7 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,5);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,5);
}

void sf_BVH2_App_Layer_sfun_c7_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2574554512U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3810991957U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1576294372U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2600446700U);
}
/*
 * Chart initialization function
 */
/* work around the buggy macro in simstruc.h until it is fixed */
#define cdrGetOutputPortReusable(S,port) \
  ( (S)->portInfo.outputs[(port)].attributes.optimOpts != \
   SS_NOT_REUSABLE_AND_GLOBAL )

static void initialize_BVH2_App_Layer_sfun_c7( SimStruct *S)
{

  {
    chartInstance.LocalData.m0_c7_d3_StateCnt = 0;
    chartInstance.LocalData.m0_c7_d1_BadCnt = 0;
    chartInstance.LocalData.m0_c7_d2_RestartCounter = 0;
    m0_c7_d11_TresholdLow_Speed = 5;
    m0_c7_d12_false = 0;
    m0_c7_d13_true = 1;
    m0_c7_d10_ThresholdHigh_Speed = 300;
    if(!cdrGetOutputPortReusable(S,1)) {
      OutputData_m0_c7_d8_oMotorStalled = 0;
    }
    if(!cdrGetOutputPortReusable(S,2)) {
      OutputData_m0_c7_d9_oStalledAlarm = 0;
    }
  }

  /* Initialize chart's state configuration */
  memset((void*)&(chartInstance.State),0,sizeof(chartInstance.State));

  {
    if(ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting */
      if(!sim_mode_is_rtw_gen(S)) {
        {
          unsigned int chartAlreadyPresent;
          chartAlreadyPresent =
            sf_debug_initialize_chart(_BVH2_App_LayerMachineNumber_,
            7,
            9,
            16,
            13,
            0,
            0,
            0,
            0,
            &(chartInstance.chartNumber),
            &(chartInstance.instanceNumber),
            ssGetPath((SimStruct *)S),
            (void *)S);
          if(chartAlreadyPresent==0) {
            /* this is the first instance */
            sf_debug_set_chart_disable_implicit_casting(_BVH2_App_LayerMachineNumber_,chartInstance.chartNumber,1);
            sf_debug_set_chart_event_thresholds(_BVH2_App_LayerMachineNumber_,
             chartInstance.chartNumber,
             0,
             0,
             0);

            _SFD_SET_DATA_PROPS(4,
             1,
             1,
             0,
             SF_UINT8,
             0,
             NULL,
             0,
             0.0,
             1.0,
             0);
            _SFD_SET_DATA_PROPS(10,
             0,
             0,
             0,
             SF_UINT16,
             0,
             NULL,
             0,
             0.0,
             1.0,
             0);
            _SFD_SET_DATA_PROPS(1,
             7,
             0,
             0,
             SF_UINT16,
             0,
             NULL,
             0,
             0.0,
             1.0,
             0);
            _SFD_SET_DATA_PROPS(8,
             2,
             0,
             1,
             SF_UINT8,
             0,
             NULL,
             0,
             0.0,
             1.0,
             0);
            _SFD_SET_DATA_PROPS(7,
             0,
             0,
             0,
             SF_UINT16,
             0,
             NULL,
             0,
             0.0,
             1.0,
             0);
            _SFD_SET_DATA_PROPS(2,
             1,
             1,
             0,
             SF_UINT8,
             0,
             NULL,
             0,
             0.0,
             1.0,
             0);
            _SFD_SET_DATA_PROPS(6,
             1,
             1,
             0,
             SF_UINT16,
             0,
             NULL,
             0,
             0.0,
             1.0,
             0);
            _SFD_SET_DATA_PROPS(0,
             2,
             0,
             1,
             SF_UINT8,
             0,
             NULL,
             0,
             0.0,
             1.0,
             0);
            _SFD_SET_DATA_PROPS(5,
             7,
             0,
             0,
             SF_UINT8,
             0,
             NULL,
             0,
             0.0,
             1.0,
             0);
            _SFD_SET_DATA_PROPS(9,
             7,
             0,
             0,
             SF_UINT8,
             0,
             NULL,
             0,
             0.0,
             1.0,
             0);
            _SFD_SET_DATA_PROPS(3,
             7,
             0,
             0,
             SF_UINT16,
             0,
             NULL,
             0,
             0.0,
             1.0,
             0);
            _SFD_SET_DATA_PROPS(12,
             0,
             0,
             0,
             SF_UINT8,
             0,
             NULL,
             0,
             0.0,
             1.0,
             0);
            _SFD_SET_DATA_PROPS(11,
             1,
             1,
             0,
             SF_UINT8,
             0,
             NULL,
             0,
             0.0,
             1.0,
             0);
            _SFD_STATE_INFO(3,0,0);
            _SFD_STATE_INFO(4,0,0);
            _SFD_STATE_INFO(6,0,0);
            _SFD_STATE_INFO(2,0,0);
            _SFD_STATE_INFO(0,0,0);
            _SFD_STATE_INFO(7,0,0);
            _SFD_STATE_INFO(8,0,0);
            _SFD_STATE_INFO(1,0,0);
            _SFD_STATE_INFO(5,0,0);
            _SFD_CH_SUBSTATE_COUNT(3);
            _SFD_CH_SUBSTATE_DECOMP(0);
            _SFD_CH_SUBSTATE_INDEX(0,3);
            _SFD_CH_SUBSTATE_INDEX(1,1);
            _SFD_CH_SUBSTATE_INDEX(2,5);
            _SFD_ST_SUBSTATE_COUNT(3,6);
            _SFD_ST_SUBSTATE_INDEX(3,0,4);
            _SFD_ST_SUBSTATE_INDEX(3,1,6);
            _SFD_ST_SUBSTATE_INDEX(3,2,2);
            _SFD_ST_SUBSTATE_INDEX(3,3,0);
            _SFD_ST_SUBSTATE_INDEX(3,4,7);
            _SFD_ST_SUBSTATE_INDEX(3,5,8);
            _SFD_ST_SUBSTATE_COUNT(4,0);
            _SFD_ST_SUBSTATE_COUNT(6,0);
            _SFD_ST_SUBSTATE_COUNT(2,0);
            _SFD_ST_SUBSTATE_COUNT(0,0);
            _SFD_ST_SUBSTATE_COUNT(7,0);
            _SFD_ST_SUBSTATE_COUNT(8,0);
            _SFD_ST_SUBSTATE_COUNT(1,0);
            _SFD_ST_SUBSTATE_COUNT(5,0);
          }
          _SFD_CV_INIT_CHART(3,1,0,0);
          {
            _SFD_CV_INIT_STATE(3,6,1,1,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(4,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(6,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(2,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(7,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(8,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(1,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(5,0,0,0,0,0,NULL,NULL);
          }

          {
            static unsigned int sStartGuardMap[] = {1,32};
            static unsigned int sEndGuardMap[] = {28,61};
            static int sPostFixPredicateTree[] = {0,1,-3};
            _SFD_CV_INIT_TRANS(12,2,&(sStartGuardMap[0]),&(sEndGuardMap[0]),3,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {2,24};
            static unsigned int sEndGuardMap[] = {20,37};
            static int sPostFixPredicateTree[] = {0,1,-2};
            _SFD_CV_INIT_TRANS(4,2,&(sStartGuardMap[0]),&(sEndGuardMap[0]),3,&(sPostFixPredicateTree[0]));
          }
          _SFD_CV_INIT_TRANS(14,0,NULL,NULL,0,NULL);

          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {20};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(8,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {14};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(7,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {16};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(5,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {3};
            static unsigned int sEndGuardMap[] = {16};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(15,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {11};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(9,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1,31};
            static unsigned int sEndGuardMap[] = {27,59};
            static int sPostFixPredicateTree[] = {0,1,-2};
            _SFD_CV_INIT_TRANS(3,2,&(sStartGuardMap[0]),&(sEndGuardMap[0]),3,&(sPostFixPredicateTree[0]));
          }
          _SFD_CV_INIT_TRANS(11,0,NULL,NULL,0,NULL);

          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {21};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(10,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {2};
            static unsigned int sEndGuardMap[] = {16};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(6,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1,31};
            static unsigned int sEndGuardMap[] = {27,59};
            static int sPostFixPredicateTree[] = {0,1,-2};
            _SFD_CV_INIT_TRANS(0,2,&(sStartGuardMap[0]),&(sEndGuardMap[0]),3,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {15};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(13,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {14};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(1,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          _SFD_CV_INIT_TRANS(2,0,NULL,NULL,0,NULL);

          _SFD_STATE_COV_WTS(3,1,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0};
            static unsigned int sEndEntryMap[] = {0};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(3,
             1,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(4,3,2,2);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,16,32};
            static unsigned int sEndEntryMap[] = {0,31,44};
            static unsigned int sStartDuringMap[] = {0,53};
            static unsigned int sEndDuringMap[] = {0,64};
            static unsigned int sStartExitMap[] = {0,72};
            static unsigned int sEndExitMap[] = {0,86};

            _SFD_STATE_COV_MAPS(4,
             3,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             2,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             2,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(6,5,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,17,30,43,60};
            static unsigned int sEndEntryMap[] = {0,29,42,59,79};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(6,
             5,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(2,4,2,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,21,36,53};
            static unsigned int sEndEntryMap[] = {0,35,52,71};
            static unsigned int sStartDuringMap[] = {0,80};
            static unsigned int sEndDuringMap[] = {0,90};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(2,
             4,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             2,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(0,5,2,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,18,33,50,68};
            static unsigned int sEndEntryMap[] = {0,32,49,67,85};
            static unsigned int sStartDuringMap[] = {0,95};
            static unsigned int sEndDuringMap[] = {0,105};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(0,
             5,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             2,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(7,3,3,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,20,41};
            static unsigned int sEndEntryMap[] = {0,40,54};
            static unsigned int sStartDuringMap[] = {0,63,74};
            static unsigned int sEndDuringMap[] = {0,72,85};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(7,
             3,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             3,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(8,1,2,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0};
            static unsigned int sEndEntryMap[] = {0};
            static unsigned int sStartDuringMap[] = {0,24};
            static unsigned int sEndDuringMap[] = {0,35};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(8,
             1,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             2,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(1,3,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,12,31};
            static unsigned int sEndEntryMap[] = {0,30,49};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(1,
             3,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(5,4,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,15,33,53};
            static unsigned int sEndEntryMap[] = {0,32,52,71};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(5,
             4,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_TRANS_COV_WTS(12,0,2,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1,32};
            static unsigned int sEndGuardMap[] = {28,61};
            _SFD_TRANS_COV_MAPS(12,
             0,NULL,NULL,
             2,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(4,0,2,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {2,24};
            static unsigned int sEndGuardMap[] = {20,37};
            _SFD_TRANS_COV_MAPS(4,
             0,NULL,NULL,
             2,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(14,0,0,0,0);
          if(chartAlreadyPresent==0)
          {
            _SFD_TRANS_COV_MAPS(14,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(8,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {20};
            _SFD_TRANS_COV_MAPS(8,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(7,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {14};
            _SFD_TRANS_COV_MAPS(7,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(5,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {16};
            _SFD_TRANS_COV_MAPS(5,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(15,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {3};
            static unsigned int sEndGuardMap[] = {16};
            _SFD_TRANS_COV_MAPS(15,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(9,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {11};
            _SFD_TRANS_COV_MAPS(9,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(3,0,2,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1,31};
            static unsigned int sEndGuardMap[] = {27,59};
            _SFD_TRANS_COV_MAPS(3,
             0,NULL,NULL,
             2,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(11,0,0,0,0);
          if(chartAlreadyPresent==0)
          {
            _SFD_TRANS_COV_MAPS(11,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(10,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {21};
            _SFD_TRANS_COV_MAPS(10,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(6,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {2};
            static unsigned int sEndGuardMap[] = {16};
            _SFD_TRANS_COV_MAPS(6,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(0,0,2,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1,31};
            static unsigned int sEndGuardMap[] = {27,59};
            _SFD_TRANS_COV_MAPS(0,
             0,NULL,NULL,
             2,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(13,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {15};
            _SFD_TRANS_COV_MAPS(13,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(1,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {14};
            _SFD_TRANS_COV_MAPS(1,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(2,0,0,0,0);
          if(chartAlreadyPresent==0)
          {
            _SFD_TRANS_COV_MAPS(2,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_SET_DATA_VALUE_PTR(4,(void *)(&InputData_m0_c7_d4_iMotor_off));
          _SFD_SET_DATA_VALUE_PTR(10,(void
            *)(&chartInstance.LocalData.m0_c7_d3_StateCnt));
          _SFD_SET_DATA_VALUE_PTR(1,(void *)(&m0_c7_d11_TresholdLow_Speed));
          _SFD_SET_DATA_VALUE_PTR(8,(void
            *)(&OutputData_m0_c7_d8_oMotorStalled));
          _SFD_SET_DATA_VALUE_PTR(7,(void
            *)(&chartInstance.LocalData.m0_c7_d1_BadCnt));
          _SFD_SET_DATA_VALUE_PTR(2,(void *)(&InputData_m0_c7_d5_Reset));
          _SFD_SET_DATA_VALUE_PTR(6,(void *)(&InputData_m0_c7_d6_iSpeed));
          _SFD_SET_DATA_VALUE_PTR(0,(void
            *)(&OutputData_m0_c7_d9_oStalledAlarm));
          _SFD_SET_DATA_VALUE_PTR(5,(void *)(&m0_c7_d12_false));
          _SFD_SET_DATA_VALUE_PTR(9,(void *)(&m0_c7_d13_true));
          _SFD_SET_DATA_VALUE_PTR(3,(void *)(&m0_c7_d10_ThresholdHigh_Speed));
          _SFD_SET_DATA_VALUE_PTR(12,(void
            *)(&chartInstance.LocalData.m0_c7_d2_RestartCounter));
          _SFD_SET_DATA_VALUE_PTR(11,(void
            *)(&InputData_m0_c7_d7_iCurrentAlarm));
        }
      }
    }else{
      sf_debug_reset_current_state_configuration(_BVH2_App_LayerMachineNumber_,chartInstance.chartNumber,chartInstance.instanceNumber);
    }
  }
  chartInstance.chartInfo.chartInitialized = 1;
}

static void enable_BVH2_App_Layer_sfun_c7( SimStruct *S)
{
}

static void disable_BVH2_App_Layer_sfun_c7( SimStruct *S)
{
}

void BVH2_App_Layer_sfun_c7_sizes_registry(SimStruct *S)
{
  ssSetSFInitOutput(S,0);
  ssSetNumInputPorts((SimStruct *)S, 4);
  ssSetInputPortDataType((SimStruct *)S,0,SS_BOOLEAN); /* InputData_m0_c7_d4_iMotor_off */
  ssSetInputPortRequiredContiguous(S,0,1);
  ssSetInputPortWidth((SimStruct *)S,0,1);
  ssSetInputPortDirectFeedThrough((SimStruct *)S,0,1);
  ssSetInputPortDataType((SimStruct *)S,1,SS_BOOLEAN); /* InputData_m0_c7_d5_Reset */
  ssSetInputPortRequiredContiguous(S,1,1);
  ssSetInputPortWidth((SimStruct *)S,1,1);
  ssSetInputPortDirectFeedThrough((SimStruct *)S,1,1);
  ssSetInputPortDataType((SimStruct *)S,2,SS_UINT16); /* InputData_m0_c7_d6_iSpeed */
  ssSetInputPortRequiredContiguous(S,2,1);
  ssSetInputPortWidth((SimStruct *)S,2,1);
  ssSetInputPortDirectFeedThrough((SimStruct *)S,2,1);
  ssSetInputPortDataType((SimStruct *)S,3,SS_BOOLEAN); /* InputData_m0_c7_d7_iCurrentAlarm */
  ssSetInputPortRequiredContiguous(S,3,1);
  ssSetInputPortWidth((SimStruct *)S,3,1);
  ssSetInputPortDirectFeedThrough((SimStruct *)S,3,1);
  ssSetNumOutputPorts((SimStruct *)S, 3);
  ssSetOutputPortDataType((SimStruct *)S,0,SS_DOUBLE);
  ssSetOutputPortWidth((SimStruct *)S,0,1);
  ssSetOutputPortDataType((SimStruct *)S,1,SS_BOOLEAN); /* OutputData_m0_c7_d8_oMotorStalled */
  ssSetOutputPortWidth((SimStruct *)S,1,1);
  ssSetOutputPortDataType((SimStruct *)S,2,SS_BOOLEAN); /* OutputData_m0_c7_d9_oStalledAlarm */
  ssSetOutputPortWidth((SimStruct *)S,2,1);
  if(sim_mode_is_rtw_gen(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("BVH2_App_Layer",7);
    ssSetStateflowIsInlinable((SimStruct *)S,chartIsInlinable);
    if(chartIsInlinable) {
      ssSetInputPortReusable((SimStruct *)S,0,1);
      ssSetInputPortReusable((SimStruct *)S,1,1);
      ssSetInputPortReusable((SimStruct *)S,2,1);
      ssSetInputPortReusable((SimStruct *)S,3,1);
      sf_mark_chart_expressionable_inputs((SimStruct *)S,"BVH2_App_Layer",7,4);
      sf_mark_chart_reusable_outputs((SimStruct *)S,"BVH2_App_Layer",7,2);
    }
    {
      int dtId;
      char *chartInstanceTypedefName =
        sf_chart_instance_typedef_name("BVH2_App_Layer",7);
      dtId = ssRegisterDataType(S, chartInstanceTypedefName);
      if (dtId == INVALID_DTYPE_ID ) return;
      /* Register the size of the udt */
      if (!ssSetDataTypeSize(S, dtId, 8)) return;
      if(!ssSetNumDWork(S,1)) return;
      ssSetDWorkDataType(S, 0, dtId);
      ssSetDWorkWidth(S, 0, 1);
      ssSetDWorkName(S, 0, "ChartInstance"); /*optional name, less than 16 chars*/
    }
  }
  ssSetChecksum0(S,(2574554512U));
  ssSetChecksum1(S,(3810991957U));
  ssSetChecksum2(S,(1576294372U));
  ssSetChecksum3(S,(2600446700U));
  ssSetExplicitFCSSCtrl(S,1);
}

void terminate_BVH2_App_Layer_sfun_c7(SimStruct *S)
{
}
static void mdlRTW_BVH2_App_Layer_sfun_c7(SimStruct *S)
{
}

void sf_BVH2_App_Layer_sfun_c7( void *);
void BVH2_App_Layer_sfun_c7_registry(SimStruct *S)
{
  chartInstance.chartInfo.chartInstance = NULL;
  chartInstance.chartInfo.chartInitialized = 0;
  chartInstance.chartInfo.sFunctionGateway = sf_BVH2_App_Layer_sfun_c7;
  chartInstance.chartInfo.initializeChart = initialize_BVH2_App_Layer_sfun_c7;
  chartInstance.chartInfo.terminateChart = terminate_BVH2_App_Layer_sfun_c7;
  chartInstance.chartInfo.enableChart = enable_BVH2_App_Layer_sfun_c7;
  chartInstance.chartInfo.disableChart = disable_BVH2_App_Layer_sfun_c7;
  chartInstance.chartInfo.mdlRTW = mdlRTW_BVH2_App_Layer_sfun_c7;
  chartInstance.chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance.chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance.chartInfo.storeCurrentConfiguration = NULL;
  chartInstance.chartInfo.sampleTime = INHERITED_SAMPLE_TIME;
  chartInstance.S = S;
  ssSetUserData((SimStruct *)S,(void *)(&(chartInstance.chartInfo))); /* register the chart instance with simstruct */
  ssSetSampleTime((SimStruct *)S, 0, chartInstance.chartInfo.sampleTime);
  if (chartInstance.chartInfo.sampleTime == INHERITED_SAMPLE_TIME) {
    ssSetOffsetTime((SimStruct *)S, 0, FIXED_IN_MINOR_STEP_OFFSET);
  } else if (chartInstance.chartInfo.sampleTime == CONTINUOUS_SAMPLE_TIME) {
    ssSetOffsetTime((SimStruct *)S, 0, 0.0);
  }
  ssSetCallSystemOutput((SimStruct *)S,0);
}

void sf_BVH2_App_Layer_sfun_c7(void *chartInstanceVoidPtr)
{
  /* Save current event being processed */
  uint8_T previousEvent;
  previousEvent = _sfEvent_;

  /* Update Stateflow time variable */
  _sfTime_ = ssGetT(chartInstance.S);

  /* Call this chart */
  _sfEvent_ = CALL_EVENT;
  BVH2_App_Layer_sfun_c7();
  _sfEvent_ = previousEvent;
}

