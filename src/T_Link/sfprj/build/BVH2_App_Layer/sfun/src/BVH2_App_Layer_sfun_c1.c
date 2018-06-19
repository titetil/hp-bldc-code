#include "BVH2_App_Layer_sfun.h"
#include "BVH2_App_Layer_sfun_c1.h"
#define mexPrintf                       sf_mex_printf
#ifdef printf
#undef printf
#endif
#define printf                          sf_mex_printf
#define CHARTINSTANCE_CHARTNUMBER       (chartInstance.chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER    (chartInstance.instanceNumber)
#include "BVH2_App_Layer_sfun_debug_macros.h"
#define IN_NO_ACTIVE_CHILD              (0)
#define IN_m0_c1_s1_Alarm_Off           1
#define IN_m0_c1_s2_CntOverCurrent      2
#define IN_m0_c1_s3_Reset               3
#define IN_m0_c1_s4_RestartCounter      4
#define IN_m0_c1_s5_Wait                5
#define IN_m0_c1_s6_greenState          6
#define IN_m0_c1_s7_redState            7
static SFBVH2_App_Layer_sfun_c1InstanceStruct chartInstance;
#define InputData_m0_c1_d3_iAlarm       (((boolean_T *)(ssGetInputPortSignal(chartInstance.S,0)))[0])
#define InputData_m0_c1_d4_iReset       (((boolean_T *)(ssGetInputPortSignal(chartInstance.S,1)))[0])
#define OutputData_m0_c1_d5_oShutoff    (((boolean_T *)(ssGetOutputPortSignal(chartInstance.S,1)))[0])
#define OutputData_m0_c1_d6_oCurrentAlarm (((boolean_T *)(ssGetOutputPortSignal(chartInstance.S,2)))[0])
static uint16_T m0_c1_u16_s32_(int32_T b);
static uint16_T m0_c1_u16_s8_(int8_T b);
static uint8_T m0_c1_u8_s32_(int32_T b);
static uint8_T m0_c1_u8_s8_(int8_T b);

static uint16_T m0_c1_u16_s32_(int32_T b)
{
  uint16_T a;
  a = (uint16_T)b;
  if(a != b) {
    sf_debug_overflow_detection(0);
  }
  return a;
}
static uint16_T m0_c1_u16_s8_(int8_T b)
{
  uint16_T a;
  a = (uint16_T)b;
  if(b < 0) {
    sf_debug_overflow_detection(0);
  }
  return a;
}
static uint8_T m0_c1_u8_s32_(int32_T b)
{
  uint8_T a;
  a = (uint8_T)b;
  if(a != b) {
    sf_debug_overflow_detection(0);
  }
  return a;
}
static uint8_T m0_c1_u8_s8_(int8_T b)
{
  uint8_T a;
  a = (uint8_T)b;
  if(b < 0) {
    sf_debug_overflow_detection(0);
  }
  return a;
}

static void enter_atomic_m0_c1_s1_Alarm_Off(void);
static void exit_atomic_m0_c1_s1_Alarm_Off(void);
static void enter_atomic_m0_c1_s2_CntOverCurrent(void);
static void exit_atomic_m0_c1_s2_CntOverCurrent(void);
static void exit_atomic_m0_c1_s3_Reset(void);
static void exit_atomic_m0_c1_s4_RestartCounter(void);
static void exit_atomic_m0_c1_s5_Wait(void);
static void enter_atomic_m0_c1_s6_greenState(void);
static void exit_atomic_m0_c1_s6_greenState(void);
static void enter_atomic_m0_c1_s7_redState(void);
static void exit_atomic_m0_c1_s7_redState(void);
void BVH2_App_Layer_sfun_c1(void)
{
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,0);
  if(chartInstance.State.is_active_BVH2_App_Layer_sfun_c1 == 0) {
    _SFD_CC_CALL(CHART_ENTER_ENTRY_FUNCTION_TAG,0);
    chartInstance.State.is_active_BVH2_App_Layer_sfun_c1 = 1;
    _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,4);
    if(CV_TRANSITION_EVAL(4, _SFD_CCP_CALL(4,0,(InputData_m0_c1_d4_iReset ==
        1))) != 0) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,4);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,4);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,4);
      _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,1);
      _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,0);
      chartInstance.State.is_BVH2_App_Layer_sfun_c1 = IN_m0_c1_s3_Reset;
      _SFD_CS_CALL(STATE_ACTIVE_TAG,1);
      _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,1);
      chartInstance.LocalData.m0_c1_d2_StateCnt = m0_c1_u16_s8_(0);
      _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,1);
      chartInstance.LocalData.m0_c1_d1_RestartCounter = m0_c1_u8_s8_(0);
      _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,2);
      _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,1);
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,1);
    }
  } else {
    switch(chartInstance.State.is_BVH2_App_Layer_sfun_c1) {
     case IN_m0_c1_s1_Alarm_Off:
      CV_CHART_EVAL(0,0,1);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,5);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,5,0);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,6);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,6);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,6);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,8);
      if(CV_TRANSITION_EVAL(8, _SFD_CCP_CALL(8,0,(InputData_m0_c1_d3_iAlarm ==
          1))) != 0) {
        if(sf_debug_transition_conflict_check_enabled()) {
          unsigned int transitionList[2];
          unsigned int numTransitions=1;
          transitionList[0] = 8;
          sf_debug_transition_conflict_check_begin();
          if(chartInstance.LocalData.m0_c1_d2_StateCnt > 100) {
            transitionList[numTransitions] = 10;
            numTransitions++;
          }
          sf_debug_transition_conflict_check_end();
          if(numTransitions>1) {
            _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
          }
        }
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,8);
        _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,8);
        _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,8);
        exit_atomic_m0_c1_s1_Alarm_Off();
        enter_atomic_m0_c1_s2_CntOverCurrent();
      } else {
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,10);
        if(CV_TRANSITION_EVAL(10,
          _SFD_CCP_CALL(10,0,(chartInstance.LocalData.m0_c1_d2_StateCnt > 100)))
         != 0) {
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,10);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,10);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,10);
          exit_atomic_m0_c1_s1_Alarm_Off();
          enter_atomic_m0_c1_s1_Alarm_Off();
        }
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,5);
      break;
     case IN_m0_c1_s2_CntOverCurrent:
      CV_CHART_EVAL(0,0,2);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,0);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,0,0);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,1);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,1);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,1);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,7);
      if(CV_TRANSITION_EVAL(7, _SFD_CCP_CALL(7,0,(InputData_m0_c1_d3_iAlarm ==
          0))) != 0) {
        if(sf_debug_transition_conflict_check_enabled()) {
          unsigned int transitionList[2];
          unsigned int numTransitions=1;
          transitionList[0] = 7;
          sf_debug_transition_conflict_check_begin();
          if(chartInstance.LocalData.m0_c1_d2_StateCnt > 1) {
            transitionList[numTransitions] = 9;
            numTransitions++;
          }
          sf_debug_transition_conflict_check_end();
          if(numTransitions>1) {
            _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
          }
        }
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,7);
        _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,7);
        _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,7);
        exit_atomic_m0_c1_s2_CntOverCurrent();
        enter_atomic_m0_c1_s6_greenState();
      } else {
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,9);
        if(CV_TRANSITION_EVAL(9,
          _SFD_CCP_CALL(9,0,(chartInstance.LocalData.m0_c1_d2_StateCnt > 1))) !=
         0) {
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,9);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,9);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,9);
          exit_atomic_m0_c1_s2_CntOverCurrent();
          enter_atomic_m0_c1_s7_redState();
        } else {
          _SFD_CS_CALL(STATE_BEFORE_DURING_ACTION_TAG,0);
          chartInstance.LocalData.m0_c1_d2_StateCnt =
            m0_c1_u16_s32_(chartInstance.LocalData.m0_c1_d2_StateCnt + 1);
          _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,0,1);
          _SFD_CS_CALL(STATE_AFTER_DURING_ACTION_TAG,0);
        }
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
      break;
     case IN_m0_c1_s3_Reset:
      CV_CHART_EVAL(0,0,3);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,1);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,1,0);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,2);
      if(CV_TRANSITION_EVAL(2,
        _SFD_CCP_CALL(2,0,(chartInstance.LocalData.m0_c1_d2_StateCnt > 50))) !=
       0) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,2);
        _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,2);
        _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,2);
        exit_atomic_m0_c1_s3_Reset();
        enter_atomic_m0_c1_s1_Alarm_Off();
      } else {
        _SFD_CS_CALL(STATE_BEFORE_DURING_ACTION_TAG,1);
        chartInstance.LocalData.m0_c1_d2_StateCnt =
          m0_c1_u16_s32_(chartInstance.LocalData.m0_c1_d2_StateCnt + 1);
        _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,1,1);
        _SFD_CS_CALL(STATE_AFTER_DURING_ACTION_TAG,1);
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,1);
      break;
     case IN_m0_c1_s4_RestartCounter:
      CV_CHART_EVAL(0,0,4);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,3);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,3,0);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,11);
      if(CV_TRANSITION_EVAL(11,
        _SFD_CCP_CALL(11,0,(chartInstance.LocalData.m0_c1_d1_RestartCounter <
          10))) != 0) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,11);
        _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,11);
        _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,11);
        exit_atomic_m0_c1_s4_RestartCounter();
        enter_atomic_m0_c1_s6_greenState();
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
      break;
     case IN_m0_c1_s5_Wait:
      CV_CHART_EVAL(0,0,5);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,6);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,6,0);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,5);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,5);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,5);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,13);
      if(CV_TRANSITION_EVAL(13,
        _SFD_CCP_CALL(13,0,(chartInstance.LocalData.m0_c1_d2_StateCnt > 200)))
       != 0) {
        if(sf_debug_transition_conflict_check_enabled()) {
          unsigned int transitionList[2];
          unsigned int numTransitions=1;
          transitionList[0] = 13;
          sf_debug_transition_conflict_check_begin();
          if(InputData_m0_c1_d3_iAlarm == 1) {
            transitionList[numTransitions] = 0;
            numTransitions++;
          }
          sf_debug_transition_conflict_check_end();
          if(numTransitions>1) {
            _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
          }
        }
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,13);
        _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,13);
        _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,13);
        exit_atomic_m0_c1_s5_Wait();
        _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,3);
        _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,3,0);
        chartInstance.State.is_BVH2_App_Layer_sfun_c1 =
          IN_m0_c1_s4_RestartCounter;
        _SFD_CS_CALL(STATE_ACTIVE_TAG,3);
        _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,3);
        chartInstance.LocalData.m0_c1_d1_RestartCounter =
          m0_c1_u8_s32_(chartInstance.LocalData.m0_c1_d1_RestartCounter + 1);
        _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,3,1);
        _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,3);
        _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
      } else {
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,0);
        if(CV_TRANSITION_EVAL(0, _SFD_CCP_CALL(0,0,(InputData_m0_c1_d3_iAlarm ==
            1))) != 0) {
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,0);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,0);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,0);
          exit_atomic_m0_c1_s5_Wait();
          enter_atomic_m0_c1_s7_redState();
        } else {
          _SFD_CS_CALL(STATE_BEFORE_DURING_ACTION_TAG,6);
          chartInstance.LocalData.m0_c1_d2_StateCnt =
            m0_c1_u16_s32_(chartInstance.LocalData.m0_c1_d2_StateCnt + 1);
          _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,6,1);
          _SFD_CS_CALL(STATE_AFTER_DURING_ACTION_TAG,6);
        }
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,6);
      break;
     case IN_m0_c1_s6_greenState:
      CV_CHART_EVAL(0,0,6);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,2);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,2,0);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,12);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,12);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,12);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,8);
      if(CV_TRANSITION_EVAL(8, _SFD_CCP_CALL(8,0,(InputData_m0_c1_d3_iAlarm ==
          1))) != 0) {
        if(sf_debug_transition_conflict_check_enabled()) {
          unsigned int transitionList[2];
          unsigned int numTransitions=1;
          transitionList[0] = 8;
          sf_debug_transition_conflict_check_begin();
          if(chartInstance.LocalData.m0_c1_d2_StateCnt > 100) {
            transitionList[numTransitions] = 10;
            numTransitions++;
          }
          sf_debug_transition_conflict_check_end();
          if(numTransitions>1) {
            _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
          }
        }
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,8);
        _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,8);
        _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,8);
        exit_atomic_m0_c1_s6_greenState();
        enter_atomic_m0_c1_s2_CntOverCurrent();
      } else {
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,10);
        if(CV_TRANSITION_EVAL(10,
          _SFD_CCP_CALL(10,0,(chartInstance.LocalData.m0_c1_d2_StateCnt > 100)))
         != 0) {
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,10);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,10);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,10);
          exit_atomic_m0_c1_s6_greenState();
          enter_atomic_m0_c1_s1_Alarm_Off();
        } else {
          _SFD_CS_CALL(STATE_BEFORE_DURING_ACTION_TAG,2);
          chartInstance.LocalData.m0_c1_d2_StateCnt =
            m0_c1_u16_s32_(chartInstance.LocalData.m0_c1_d2_StateCnt + 1);
          _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,2,1);
          _SFD_CS_CALL(STATE_AFTER_DURING_ACTION_TAG,2);
        }
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,2);
      break;
     case IN_m0_c1_s7_redState:
      CV_CHART_EVAL(0,0,7);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,4);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,4,0);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,3);
      if(CV_TRANSITION_EVAL(3, _SFD_CCP_CALL(3,0,(InputData_m0_c1_d3_iAlarm ==
          0))) != 0) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,3);
        _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,3);
        _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,3);
        exit_atomic_m0_c1_s7_redState();
        _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,6);
        _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,6,0);
        chartInstance.State.is_BVH2_App_Layer_sfun_c1 = IN_m0_c1_s5_Wait;
        _SFD_CS_CALL(STATE_ACTIVE_TAG,6);
        _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,6);
        chartInstance.LocalData.m0_c1_d2_StateCnt = m0_c1_u16_s8_(0);
        _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,6,1);
        _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,6);
        _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,6);
      } else {
        _SFD_CS_CALL(STATE_BEFORE_DURING_ACTION_TAG,4);
        chartInstance.LocalData.m0_c1_d2_StateCnt =
          m0_c1_u16_s32_(chartInstance.LocalData.m0_c1_d2_StateCnt + 1);
        _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,4,1);
        _SFD_CS_CALL(STATE_AFTER_DURING_ACTION_TAG,4);
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,4);
      break;
     default:
      CV_CHART_EVAL(0,0,0);
      break;
    }
  }
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
}

static void enter_atomic_m0_c1_s1_Alarm_Off(void)
{
  real_T __sfTemp1;
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,5);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,5,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c1 = IN_m0_c1_s1_Alarm_Off;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,5);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,5);
  OutputData_m0_c1_d5_oShutoff = 0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,5,1);
  OutputData_m0_c1_d6_oCurrentAlarm = 0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,5,2);
  sf_mex_printf("%s =\n", "RestartCounter");
  chartInstance.LocalData.m0_c1_d1_RestartCounter = m0_c1_u8_s8_(0);
  __sfTemp1 = (real_T)chartInstance.LocalData.m0_c1_d1_RestartCounter;
  ml_call_function("disp", 0, 1, 4, sf_mex_create_mx_array(&__sfTemp1, 0, 0, 0));
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,5,3);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,5);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,5);
}

static void exit_atomic_m0_c1_s1_Alarm_Off(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,5);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,5,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c1 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,5);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,5);
}

static void enter_atomic_m0_c1_s2_CntOverCurrent(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,0);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,0,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c1 = IN_m0_c1_s2_CntOverCurrent;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,0);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,0);
  chartInstance.LocalData.m0_c1_d2_StateCnt = m0_c1_u16_s8_(0);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,0,1);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,0);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
}

static void exit_atomic_m0_c1_s2_CntOverCurrent(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,0);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,0,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c1 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,0);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
}

static void exit_atomic_m0_c1_s3_Reset(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,1);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,1,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c1 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,1);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,1);
}

static void exit_atomic_m0_c1_s4_RestartCounter(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,3);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,3,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c1 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,3);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
}

static void exit_atomic_m0_c1_s5_Wait(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,6);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,6,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c1 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,6);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,6);
}

static void enter_atomic_m0_c1_s6_greenState(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,2);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,2,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c1 = IN_m0_c1_s6_greenState;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,2);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,2);
  OutputData_m0_c1_d5_oShutoff = 0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,2,1);
  OutputData_m0_c1_d6_oCurrentAlarm = 1;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,2,2);
  chartInstance.LocalData.m0_c1_d2_StateCnt = m0_c1_u16_s8_(0);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,2,3);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,2);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,2);
}

static void exit_atomic_m0_c1_s6_greenState(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,2);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,2,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c1 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,2);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,2);
}

static void enter_atomic_m0_c1_s7_redState(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,4);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,4,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c1 = IN_m0_c1_s7_redState;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,4);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,4);
  chartInstance.LocalData.m0_c1_d2_StateCnt = m0_c1_u16_s8_(0);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,4,1);
  OutputData_m0_c1_d5_oShutoff = 1;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,4,2);
  OutputData_m0_c1_d6_oCurrentAlarm = 1;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,4,3);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,4);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,4);
}

static void exit_atomic_m0_c1_s7_redState(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,4);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,4,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c1 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,4);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,4);
}

void sf_BVH2_App_Layer_sfun_c1_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(207060503U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(642359483U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1946607766U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(545177964U);
}
/*
 * Chart initialization function
 */
/* work around the buggy macro in simstruc.h until it is fixed */
#define cdrGetOutputPortReusable(S,port) \
  ( (S)->portInfo.outputs[(port)].attributes.optimOpts != \
   SS_NOT_REUSABLE_AND_GLOBAL )

static void initialize_BVH2_App_Layer_sfun_c1( SimStruct *S)
{

  {
    chartInstance.LocalData.m0_c1_d2_StateCnt = 0;
    chartInstance.LocalData.m0_c1_d1_RestartCounter = 0;
    if(!cdrGetOutputPortReusable(S,1)) {
      OutputData_m0_c1_d5_oShutoff = 0;
    }
    if(!cdrGetOutputPortReusable(S,2)) {
      OutputData_m0_c1_d6_oCurrentAlarm = 0;
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
            1,
            7,
            14,
            6,
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
            _SFD_SET_DATA_PROPS(5,
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
            _SFD_SET_DATA_PROPS(0,
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
            _SFD_SET_DATA_PROPS(1,
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
            _SFD_SET_DATA_PROPS(2,
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
            _SFD_SET_DATA_PROPS(3,
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
            _SFD_STATE_INFO(5,0,0);
            _SFD_STATE_INFO(0,0,0);
            _SFD_STATE_INFO(1,0,0);
            _SFD_STATE_INFO(3,0,0);
            _SFD_STATE_INFO(6,0,0);
            _SFD_STATE_INFO(2,0,0);
            _SFD_STATE_INFO(4,0,0);
            _SFD_CH_SUBSTATE_COUNT(7);
            _SFD_CH_SUBSTATE_DECOMP(0);
            _SFD_CH_SUBSTATE_INDEX(0,5);
            _SFD_CH_SUBSTATE_INDEX(1,0);
            _SFD_CH_SUBSTATE_INDEX(2,1);
            _SFD_CH_SUBSTATE_INDEX(3,3);
            _SFD_CH_SUBSTATE_INDEX(4,6);
            _SFD_CH_SUBSTATE_INDEX(5,2);
            _SFD_CH_SUBSTATE_INDEX(6,4);
            _SFD_ST_SUBSTATE_COUNT(5,0);
            _SFD_ST_SUBSTATE_COUNT(0,0);
            _SFD_ST_SUBSTATE_COUNT(1,0);
            _SFD_ST_SUBSTATE_COUNT(3,0);
            _SFD_ST_SUBSTATE_COUNT(6,0);
            _SFD_ST_SUBSTATE_COUNT(2,0);
            _SFD_ST_SUBSTATE_COUNT(4,0);
          }
          _SFD_CV_INIT_CHART(7,1,0,0);
          {
            _SFD_CV_INIT_STATE(5,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(1,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(3,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(6,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(2,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(4,0,0,0,0,0,NULL,NULL);
          }

          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {10};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(8,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {10};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(4,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {12};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(3,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {14};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(9,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          _SFD_CV_INIT_TRANS(1,0,NULL,NULL,0,NULL);

          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {12};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(7,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {16};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(13,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          _SFD_CV_INIT_TRANS(5,0,NULL,NULL,0,NULL);

          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {12};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(0,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          _SFD_CV_INIT_TRANS(6,0,NULL,NULL,0,NULL);

          _SFD_CV_INIT_TRANS(12,0,NULL,NULL,0,NULL);

          {
            static unsigned int sStartGuardMap[] = {2};
            static unsigned int sEndGuardMap[] = {17};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(10,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {12};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(2,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {20};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(11,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          _SFD_STATE_COV_WTS(5,4,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,17,33,52};
            static unsigned int sEndEntryMap[] = {0,32,51,70};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(5,
             4,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(0,2,2,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,22};
            static unsigned int sEndEntryMap[] = {0,36};
            static unsigned int sStartDuringMap[] = {0,45};
            static unsigned int sEndDuringMap[] = {0,56};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(0,
             2,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             2,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(1,3,2,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,13,25};
            static unsigned int sEndEntryMap[] = {0,24,44};
            static unsigned int sStartDuringMap[] = {0,53};
            static unsigned int sEndDuringMap[] = {0,64};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(1,
             3,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             2,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(3,2,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,22};
            static unsigned int sEndEntryMap[] = {0,39};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(3,
             2,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(6,2,2,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,12};
            static unsigned int sEndEntryMap[] = {0,25};
            static unsigned int sStartDuringMap[] = {0,34};
            static unsigned int sEndDuringMap[] = {0,44};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(6,
             2,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             2,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(2,4,2,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,18,34,53};
            static unsigned int sEndEntryMap[] = {0,33,52,66};
            static unsigned int sStartDuringMap[] = {0,75};
            static unsigned int sEndDuringMap[] = {0,85};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(2,
             4,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             2,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(4,4,2,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,16,28,42};
            static unsigned int sEndEntryMap[] = {0,27,41,60};
            static unsigned int sStartDuringMap[] = {0,69};
            static unsigned int sEndDuringMap[] = {0,79};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(4,
             4,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             2,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_TRANS_COV_WTS(8,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {10};
            _SFD_TRANS_COV_MAPS(8,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(4,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {10};
            _SFD_TRANS_COV_MAPS(4,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(3,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {12};
            _SFD_TRANS_COV_MAPS(3,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(9,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {14};
            _SFD_TRANS_COV_MAPS(9,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(1,0,0,0,0);
          if(chartAlreadyPresent==0)
          {
            _SFD_TRANS_COV_MAPS(1,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(7,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {12};
            _SFD_TRANS_COV_MAPS(7,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(13,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {16};
            _SFD_TRANS_COV_MAPS(13,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(5,0,0,0,0);
          if(chartAlreadyPresent==0)
          {
            _SFD_TRANS_COV_MAPS(5,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(0,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {12};
            _SFD_TRANS_COV_MAPS(0,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(6,0,0,0,0);
          if(chartAlreadyPresent==0)
          {
            _SFD_TRANS_COV_MAPS(6,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(12,0,0,0,0);
          if(chartAlreadyPresent==0)
          {
            _SFD_TRANS_COV_MAPS(12,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(10,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {2};
            static unsigned int sEndGuardMap[] = {17};
            _SFD_TRANS_COV_MAPS(10,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(2,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {12};
            _SFD_TRANS_COV_MAPS(2,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(11,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {20};
            _SFD_TRANS_COV_MAPS(11,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_SET_DATA_VALUE_PTR(4,(void
            *)(&chartInstance.LocalData.m0_c1_d2_StateCnt));
          _SFD_SET_DATA_VALUE_PTR(5,(void *)(&InputData_m0_c1_d3_iAlarm));
          _SFD_SET_DATA_VALUE_PTR(0,(void *)(&InputData_m0_c1_d4_iReset));
          _SFD_SET_DATA_VALUE_PTR(1,(void *)(&OutputData_m0_c1_d5_oShutoff));
          _SFD_SET_DATA_VALUE_PTR(2,(void
            *)(&OutputData_m0_c1_d6_oCurrentAlarm));
          _SFD_SET_DATA_VALUE_PTR(3,(void
            *)(&chartInstance.LocalData.m0_c1_d1_RestartCounter));
        }
      }
    }else{
      sf_debug_reset_current_state_configuration(_BVH2_App_LayerMachineNumber_,chartInstance.chartNumber,chartInstance.instanceNumber);
    }
  }
  chartInstance.chartInfo.chartInitialized = 1;
}

static void enable_BVH2_App_Layer_sfun_c1( SimStruct *S)
{
}

static void disable_BVH2_App_Layer_sfun_c1( SimStruct *S)
{
}

void BVH2_App_Layer_sfun_c1_sizes_registry(SimStruct *S)
{
  ssSetSFInitOutput(S,0);
  ssSetNumInputPorts((SimStruct *)S, 2);
  ssSetInputPortDataType((SimStruct *)S,0,SS_BOOLEAN); /* InputData_m0_c1_d3_iAlarm */
  ssSetInputPortRequiredContiguous(S,0,1);
  ssSetInputPortWidth((SimStruct *)S,0,1);
  ssSetInputPortDirectFeedThrough((SimStruct *)S,0,1);
  ssSetInputPortDataType((SimStruct *)S,1,SS_BOOLEAN); /* InputData_m0_c1_d4_iReset */
  ssSetInputPortRequiredContiguous(S,1,1);
  ssSetInputPortWidth((SimStruct *)S,1,1);
  ssSetInputPortDirectFeedThrough((SimStruct *)S,1,1);
  ssSetNumOutputPorts((SimStruct *)S, 3);
  ssSetOutputPortDataType((SimStruct *)S,0,SS_DOUBLE);
  ssSetOutputPortWidth((SimStruct *)S,0,1);
  ssSetOutputPortDataType((SimStruct *)S,1,SS_BOOLEAN); /* OutputData_m0_c1_d5_oShutoff */
  ssSetOutputPortWidth((SimStruct *)S,1,1);
  ssSetOutputPortDataType((SimStruct *)S,2,SS_BOOLEAN); /* OutputData_m0_c1_d6_oCurrentAlarm */
  ssSetOutputPortWidth((SimStruct *)S,2,1);
  if(sim_mode_is_rtw_gen(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("BVH2_App_Layer",1);
    ssSetStateflowIsInlinable((SimStruct *)S,chartIsInlinable);
    if(chartIsInlinable) {
      ssSetInputPortReusable((SimStruct *)S,0,1);
      ssSetInputPortReusable((SimStruct *)S,1,1);
      sf_mark_chart_expressionable_inputs((SimStruct *)S,"BVH2_App_Layer",1,2);
      sf_mark_chart_reusable_outputs((SimStruct *)S,"BVH2_App_Layer",1,2);
    }
    {
      int dtId;
      char *chartInstanceTypedefName =
        sf_chart_instance_typedef_name("BVH2_App_Layer",1);
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
  ssSetChecksum0(S,(207060503U));
  ssSetChecksum1(S,(642359483U));
  ssSetChecksum2(S,(1946607766U));
  ssSetChecksum3(S,(545177964U));
  ssSetExplicitFCSSCtrl(S,1);
}

void terminate_BVH2_App_Layer_sfun_c1(SimStruct *S)
{
}
static void mdlRTW_BVH2_App_Layer_sfun_c1(SimStruct *S)
{
}

void sf_BVH2_App_Layer_sfun_c1( void *);
void BVH2_App_Layer_sfun_c1_registry(SimStruct *S)
{
  chartInstance.chartInfo.chartInstance = NULL;
  chartInstance.chartInfo.chartInitialized = 0;
  chartInstance.chartInfo.sFunctionGateway = sf_BVH2_App_Layer_sfun_c1;
  chartInstance.chartInfo.initializeChart = initialize_BVH2_App_Layer_sfun_c1;
  chartInstance.chartInfo.terminateChart = terminate_BVH2_App_Layer_sfun_c1;
  chartInstance.chartInfo.enableChart = enable_BVH2_App_Layer_sfun_c1;
  chartInstance.chartInfo.disableChart = disable_BVH2_App_Layer_sfun_c1;
  chartInstance.chartInfo.mdlRTW = mdlRTW_BVH2_App_Layer_sfun_c1;
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

void sf_BVH2_App_Layer_sfun_c1(void *chartInstanceVoidPtr)
{
  /* Save current event being processed */
  uint8_T previousEvent;
  previousEvent = _sfEvent_;

  /* Update Stateflow time variable */
  _sfTime_ = ssGetT(chartInstance.S);

  /* Call this chart */
  _sfEvent_ = CALL_EVENT;
  BVH2_App_Layer_sfun_c1();
  _sfEvent_ = previousEvent;
}

