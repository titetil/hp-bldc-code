#include "BVH2_App_Layer_sfun.h"
#include "BVH2_App_Layer_sfun_c4.h"
#define mexPrintf                       sf_mex_printf
#ifdef printf
#undef printf
#endif
#define printf                          sf_mex_printf
#define CHARTINSTANCE_CHARTNUMBER       (chartInstance.chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER    (chartInstance.instanceNumber)
#include "BVH2_App_Layer_sfun_debug_macros.h"
#define IN_NO_ACTIVE_CHILD              (0)
#define IN_m0_c4_s1_Alarm_Off           1
#define IN_m0_c4_s2_Reset               2
#define IN_m0_c4_s3_Wait                3
#define IN_m0_c4_s4_Wrong_ETAT          4
#define IN_m0_c4_s5_greenState          5
#define IN_m0_c4_s6_redState            6
static SFBVH2_App_Layer_sfun_c4InstanceStruct chartInstance;
#define InputData_m0_c4_d2_iAlarm       (((boolean_T *)(ssGetInputPortSignal(chartInstance.S,0)))[0])
#define InputData_m0_c4_d3_iReset       (((boolean_T *)(ssGetInputPortSignal(chartInstance.S,1)))[0])
#define OutputData_m0_c4_d4_oShutoff    (((boolean_T *)(ssGetOutputPortSignal(chartInstance.S,1)))[0])
#define OutputData_m0_c4_d5_oAlarm      (((boolean_T *)(ssGetOutputPortSignal(chartInstance.S,2)))[0])
static uint16_T m0_c4_u16_s8_(int8_T b);
static uint16_T m0_c4_u16_s32_(int32_T b);

static uint16_T m0_c4_u16_s8_(int8_T b)
{
  uint16_T a;
  a = (uint16_T)b;
  if(b < 0) {
    sf_debug_overflow_detection(0);
  }
  return a;
}
static uint16_T m0_c4_u16_s32_(int32_T b)
{
  uint16_T a;
  a = (uint16_T)b;
  if(a != b) {
    sf_debug_overflow_detection(0);
  }
  return a;
}

static void enter_atomic_m0_c4_s1_Alarm_Off(void);
static void exit_atomic_m0_c4_s1_Alarm_Off(void);
static void exit_atomic_m0_c4_s2_Reset(void);
static void exit_atomic_m0_c4_s3_Wait(void);
static void enter_atomic_m0_c4_s4_Wrong_ETAT(void);
static void exit_atomic_m0_c4_s4_Wrong_ETAT(void);
static void enter_atomic_m0_c4_s5_greenState(void);
static void exit_atomic_m0_c4_s5_greenState(void);
static void enter_atomic_m0_c4_s6_redState(void);
static void exit_atomic_m0_c4_s6_redState(void);
void BVH2_App_Layer_sfun_c4(void)
{
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,3);
  if(chartInstance.State.is_active_BVH2_App_Layer_sfun_c4 == 0) {
    _SFD_CC_CALL(CHART_ENTER_ENTRY_FUNCTION_TAG,3);
    chartInstance.State.is_active_BVH2_App_Layer_sfun_c4 = 1;
    _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,10);
    if(CV_TRANSITION_EVAL(10, _SFD_CCP_CALL(10,0,(InputData_m0_c4_d3_iReset ==
        1))) != 0) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,10);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,10);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,10);
      _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,2);
      _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,2,0);
      chartInstance.State.is_BVH2_App_Layer_sfun_c4 = IN_m0_c4_s2_Reset;
      _SFD_CS_CALL(STATE_ACTIVE_TAG,2);
      _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,2);
      chartInstance.LocalData.m0_c4_d1_StateCnt = m0_c4_u16_s8_(0);
      _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,2,1);
      _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,2);
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,2);
    }
  } else {
    switch(chartInstance.State.is_BVH2_App_Layer_sfun_c4) {
     case IN_m0_c4_s1_Alarm_Off:
      CV_CHART_EVAL(3,0,1);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,4);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,4,0);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,5);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,5);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,5);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,4);
      if(CV_TRANSITION_EVAL(4, _SFD_CCP_CALL(4,0,(InputData_m0_c4_d2_iAlarm ==
          1))) != 0) {
        if(sf_debug_transition_conflict_check_enabled()) {
          unsigned int transitionList[2];
          unsigned int numTransitions=1;
          transitionList[0] = 4;
          sf_debug_transition_conflict_check_begin();
          if(chartInstance.LocalData.m0_c4_d1_StateCnt > 5) {
            transitionList[numTransitions] = 7;
            numTransitions++;
          }
          sf_debug_transition_conflict_check_end();
          if(numTransitions>1) {
            _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
          }
        }
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,4);
        _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,4);
        _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,4);
        exit_atomic_m0_c4_s1_Alarm_Off();
        enter_atomic_m0_c4_s4_Wrong_ETAT();
      } else {
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,7);
        if(CV_TRANSITION_EVAL(7,
          _SFD_CCP_CALL(7,0,(chartInstance.LocalData.m0_c4_d1_StateCnt > 5))) !=
         0) {
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,7);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,7);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,7);
          exit_atomic_m0_c4_s1_Alarm_Off();
          enter_atomic_m0_c4_s1_Alarm_Off();
        }
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,4);
      break;
     case IN_m0_c4_s2_Reset:
      CV_CHART_EVAL(3,0,2);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,2);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,2,0);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,2);
      if(CV_TRANSITION_EVAL(2,
        _SFD_CCP_CALL(2,0,(chartInstance.LocalData.m0_c4_d1_StateCnt > 50))) !=
       0) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,2);
        _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,2);
        _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,2);
        exit_atomic_m0_c4_s2_Reset();
        enter_atomic_m0_c4_s5_greenState();
      } else {
        _SFD_CS_CALL(STATE_BEFORE_DURING_ACTION_TAG,2);
        chartInstance.LocalData.m0_c4_d1_StateCnt =
          m0_c4_u16_s32_(chartInstance.LocalData.m0_c4_d1_StateCnt + 1);
        _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,2,1);
        _SFD_CS_CALL(STATE_AFTER_DURING_ACTION_TAG,2);
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,2);
      break;
     case IN_m0_c4_s3_Wait:
      CV_CHART_EVAL(3,0,3);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,3);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,3,0);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,3);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,3);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,3);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,11);
      if(CV_TRANSITION_EVAL(11,
        _SFD_CCP_CALL(11,0,(chartInstance.LocalData.m0_c4_d1_StateCnt > 15))) !=
       0) {
        if(sf_debug_transition_conflict_check_enabled()) {
          unsigned int transitionList[2];
          unsigned int numTransitions=1;
          transitionList[0] = 11;
          sf_debug_transition_conflict_check_begin();
          if(InputData_m0_c4_d2_iAlarm == 1) {
            transitionList[numTransitions] = 1;
            numTransitions++;
          }
          sf_debug_transition_conflict_check_end();
          if(numTransitions>1) {
            _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
          }
        }
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,11);
        _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,11);
        _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,11);
        exit_atomic_m0_c4_s3_Wait();
        enter_atomic_m0_c4_s5_greenState();
      } else {
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,1);
        if(CV_TRANSITION_EVAL(1, _SFD_CCP_CALL(1,0,(InputData_m0_c4_d2_iAlarm ==
            1))) != 0) {
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,1);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,1);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,1);
          exit_atomic_m0_c4_s3_Wait();
          enter_atomic_m0_c4_s6_redState();
        } else {
          _SFD_CS_CALL(STATE_BEFORE_DURING_ACTION_TAG,3);
          chartInstance.LocalData.m0_c4_d1_StateCnt =
            m0_c4_u16_s32_(chartInstance.LocalData.m0_c4_d1_StateCnt + 1);
          _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,3,1);
          _SFD_CS_CALL(STATE_AFTER_DURING_ACTION_TAG,3);
        }
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
      break;
     case IN_m0_c4_s4_Wrong_ETAT:
      CV_CHART_EVAL(3,0,4);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,0);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,0,0);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,6);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,6);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,6);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,12);
      if(CV_TRANSITION_EVAL(12, _SFD_CCP_CALL(12,0,(InputData_m0_c4_d2_iAlarm ==
          0))) != 0) {
        if(sf_debug_transition_conflict_check_enabled()) {
          unsigned int transitionList[2];
          unsigned int numTransitions=1;
          transitionList[0] = 12;
          sf_debug_transition_conflict_check_begin();
          if(chartInstance.LocalData.m0_c4_d1_StateCnt > 0) {
            transitionList[numTransitions] = 9;
            numTransitions++;
          }
          sf_debug_transition_conflict_check_end();
          if(numTransitions>1) {
            _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
          }
        }
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,12);
        _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,12);
        _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,12);
        exit_atomic_m0_c4_s4_Wrong_ETAT();
        enter_atomic_m0_c4_s1_Alarm_Off();
      } else {
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,9);
        if(CV_TRANSITION_EVAL(9,
          _SFD_CCP_CALL(9,0,(chartInstance.LocalData.m0_c4_d1_StateCnt > 0))) !=
         0) {
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,9);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,9);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,9);
          exit_atomic_m0_c4_s4_Wrong_ETAT();
          enter_atomic_m0_c4_s6_redState();
        } else {
          _SFD_CS_CALL(STATE_BEFORE_DURING_ACTION_TAG,0);
          chartInstance.LocalData.m0_c4_d1_StateCnt =
            m0_c4_u16_s32_(chartInstance.LocalData.m0_c4_d1_StateCnt + 1);
          _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,0,1);
          _SFD_CS_CALL(STATE_AFTER_DURING_ACTION_TAG,0);
        }
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
      break;
     case IN_m0_c4_s5_greenState:
      CV_CHART_EVAL(3,0,5);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,1);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,1,0);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,0);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,0);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,0);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,4);
      if(CV_TRANSITION_EVAL(4, _SFD_CCP_CALL(4,0,(InputData_m0_c4_d2_iAlarm ==
          1))) != 0) {
        if(sf_debug_transition_conflict_check_enabled()) {
          unsigned int transitionList[2];
          unsigned int numTransitions=1;
          transitionList[0] = 4;
          sf_debug_transition_conflict_check_begin();
          if(chartInstance.LocalData.m0_c4_d1_StateCnt > 5) {
            transitionList[numTransitions] = 7;
            numTransitions++;
          }
          sf_debug_transition_conflict_check_end();
          if(numTransitions>1) {
            _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
          }
        }
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,4);
        _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,4);
        _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,4);
        exit_atomic_m0_c4_s5_greenState();
        enter_atomic_m0_c4_s4_Wrong_ETAT();
      } else {
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,7);
        if(CV_TRANSITION_EVAL(7,
          _SFD_CCP_CALL(7,0,(chartInstance.LocalData.m0_c4_d1_StateCnt > 5))) !=
         0) {
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,7);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,7);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,7);
          exit_atomic_m0_c4_s5_greenState();
          enter_atomic_m0_c4_s1_Alarm_Off();
        } else {
          _SFD_CS_CALL(STATE_BEFORE_DURING_ACTION_TAG,1);
          chartInstance.LocalData.m0_c4_d1_StateCnt =
            m0_c4_u16_s32_(chartInstance.LocalData.m0_c4_d1_StateCnt + 1);
          _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,1,1);
          _SFD_CS_CALL(STATE_AFTER_DURING_ACTION_TAG,1);
        }
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,1);
      break;
     case IN_m0_c4_s6_redState:
      CV_CHART_EVAL(3,0,6);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,5);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,5,0);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,8);
      if(CV_TRANSITION_EVAL(8, _SFD_CCP_CALL(8,0,(InputData_m0_c4_d2_iAlarm ==
          0))) != 0) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,8);
        _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,8);
        _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,8);
        exit_atomic_m0_c4_s6_redState();
        _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,3);
        _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,3,0);
        chartInstance.State.is_BVH2_App_Layer_sfun_c4 = IN_m0_c4_s3_Wait;
        _SFD_CS_CALL(STATE_ACTIVE_TAG,3);
        _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,3);
        chartInstance.LocalData.m0_c4_d1_StateCnt = m0_c4_u16_s8_(0);
        _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,3,1);
        _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,3);
        _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,5);
      break;
     default:
      CV_CHART_EVAL(3,0,0);
      break;
    }
  }
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
}

static void enter_atomic_m0_c4_s1_Alarm_Off(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,4);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,4,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c4 = IN_m0_c4_s1_Alarm_Off;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,4);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,4);
  OutputData_m0_c4_d4_oShutoff = 0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,4,1);
  OutputData_m0_c4_d5_oAlarm = 0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,4,2);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,4);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,4);
}

static void exit_atomic_m0_c4_s1_Alarm_Off(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,4);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,4,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c4 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,4);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,4);
}

static void exit_atomic_m0_c4_s2_Reset(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,2);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,2,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c4 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,2);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,2);
}

static void exit_atomic_m0_c4_s3_Wait(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,3);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,3,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c4 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,3);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
}

static void enter_atomic_m0_c4_s4_Wrong_ETAT(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,0);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,0,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c4 = IN_m0_c4_s4_Wrong_ETAT;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,0);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,0);
  chartInstance.LocalData.m0_c4_d1_StateCnt = m0_c4_u16_s8_(0);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,0,1);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,0);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
}

static void exit_atomic_m0_c4_s4_Wrong_ETAT(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,0);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,0,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c4 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,0);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
}

static void enter_atomic_m0_c4_s5_greenState(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,1);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c4 = IN_m0_c4_s5_greenState;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,1);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,1);
  OutputData_m0_c4_d4_oShutoff = 0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,1);
  OutputData_m0_c4_d5_oAlarm = 0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,2);
  chartInstance.LocalData.m0_c4_d1_StateCnt = m0_c4_u16_s8_(0);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,3);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,1);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,1);
}

static void exit_atomic_m0_c4_s5_greenState(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,1);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,1,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c4 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,1);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,1);
}

static void enter_atomic_m0_c4_s6_redState(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,5);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,5,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c4 = IN_m0_c4_s6_redState;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,5);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,5);
  chartInstance.LocalData.m0_c4_d1_StateCnt = m0_c4_u16_s8_(0);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,5,1);
  OutputData_m0_c4_d4_oShutoff = 1;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,5,2);
  OutputData_m0_c4_d5_oAlarm = 1;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,5,3);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,5);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,5);
}

static void exit_atomic_m0_c4_s6_redState(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,5);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,5,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c4 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,5);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,5);
}

void sf_BVH2_App_Layer_sfun_c4_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(649361435U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3463553448U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2309531357U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3833722031U);
}
/*
 * Chart initialization function
 */
/* work around the buggy macro in simstruc.h until it is fixed */
#define cdrGetOutputPortReusable(S,port) \
  ( (S)->portInfo.outputs[(port)].attributes.optimOpts != \
   SS_NOT_REUSABLE_AND_GLOBAL )

static void initialize_BVH2_App_Layer_sfun_c4( SimStruct *S)
{

  {
    chartInstance.LocalData.m0_c4_d1_StateCnt = 0;
    if(!cdrGetOutputPortReusable(S,1)) {
      OutputData_m0_c4_d4_oShutoff = 0;
    }
    if(!cdrGetOutputPortReusable(S,2)) {
      OutputData_m0_c4_d5_oAlarm = 0;
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
            4,
            6,
            13,
            5,
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

            _SFD_SET_DATA_PROPS(0,
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
            _SFD_SET_DATA_PROPS(3,
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
            _SFD_STATE_INFO(4,0,0);
            _SFD_STATE_INFO(2,0,0);
            _SFD_STATE_INFO(3,0,0);
            _SFD_STATE_INFO(0,0,0);
            _SFD_STATE_INFO(1,0,0);
            _SFD_STATE_INFO(5,0,0);
            _SFD_CH_SUBSTATE_COUNT(6);
            _SFD_CH_SUBSTATE_DECOMP(0);
            _SFD_CH_SUBSTATE_INDEX(0,4);
            _SFD_CH_SUBSTATE_INDEX(1,2);
            _SFD_CH_SUBSTATE_INDEX(2,3);
            _SFD_CH_SUBSTATE_INDEX(3,0);
            _SFD_CH_SUBSTATE_INDEX(4,1);
            _SFD_CH_SUBSTATE_INDEX(5,5);
            _SFD_ST_SUBSTATE_COUNT(4,0);
            _SFD_ST_SUBSTATE_COUNT(2,0);
            _SFD_ST_SUBSTATE_COUNT(3,0);
            _SFD_ST_SUBSTATE_COUNT(0,0);
            _SFD_ST_SUBSTATE_COUNT(1,0);
            _SFD_ST_SUBSTATE_COUNT(5,0);
          }
          _SFD_CV_INIT_CHART(6,1,0,0);
          {
            _SFD_CV_INIT_STATE(4,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(2,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(3,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(1,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(5,0,0,0,0,0,NULL,NULL);
          }

          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {10};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(4,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {10};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(10,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {12};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(8,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {14};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(9,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          _SFD_CV_INIT_TRANS(6,0,NULL,NULL,0,NULL);

          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {12};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(12,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {12};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(2,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          _SFD_CV_INIT_TRANS(5,0,NULL,NULL,0,NULL);

          _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

          {
            static unsigned int sStartGuardMap[] = {2};
            static unsigned int sEndGuardMap[] = {15};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(7,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {15};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(11,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {12};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(1,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          _SFD_CV_INIT_TRANS(3,0,NULL,NULL,0,NULL);

          _SFD_STATE_COV_WTS(4,3,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,17,33};
            static unsigned int sEndEntryMap[] = {0,32,44};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(4,
             3,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(2,2,2,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,13};
            static unsigned int sEndEntryMap[] = {0,24};
            static unsigned int sStartDuringMap[] = {0,33};
            static unsigned int sEndDuringMap[] = {0,44};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(2,
             2,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             2,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(3,2,2,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,12};
            static unsigned int sEndEntryMap[] = {0,25};
            static unsigned int sStartDuringMap[] = {0,34};
            static unsigned int sEndDuringMap[] = {0,44};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(3,
             2,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             2,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(0,2,2,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,18};
            static unsigned int sEndEntryMap[] = {0,32};
            static unsigned int sStartDuringMap[] = {0,41};
            static unsigned int sEndDuringMap[] = {0,52};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(0,
             2,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             2,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(1,4,2,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,18,34,46};
            static unsigned int sEndEntryMap[] = {0,33,45,59};
            static unsigned int sStartDuringMap[] = {0,68};
            static unsigned int sEndDuringMap[] = {0,78};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(1,
             4,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             2,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(5,4,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,16,28,42};
            static unsigned int sEndEntryMap[] = {0,27,41,53};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(5,
             4,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
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
          _SFD_TRANS_COV_WTS(10,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {10};
            _SFD_TRANS_COV_MAPS(10,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(8,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {12};
            _SFD_TRANS_COV_MAPS(8,
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
          _SFD_TRANS_COV_WTS(6,0,0,0,0);
          if(chartAlreadyPresent==0)
          {
            _SFD_TRANS_COV_MAPS(6,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(12,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {12};
            _SFD_TRANS_COV_MAPS(12,
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
          _SFD_TRANS_COV_WTS(5,0,0,0,0);
          if(chartAlreadyPresent==0)
          {
            _SFD_TRANS_COV_MAPS(5,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(0,0,0,0,0);
          if(chartAlreadyPresent==0)
          {
            _SFD_TRANS_COV_MAPS(0,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(7,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {2};
            static unsigned int sEndGuardMap[] = {15};
            _SFD_TRANS_COV_MAPS(7,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(11,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {15};
            _SFD_TRANS_COV_MAPS(11,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(1,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {12};
            _SFD_TRANS_COV_MAPS(1,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(3,0,0,0,0);
          if(chartAlreadyPresent==0)
          {
            _SFD_TRANS_COV_MAPS(3,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_SET_DATA_VALUE_PTR(0,(void
            *)(&chartInstance.LocalData.m0_c4_d1_StateCnt));
          _SFD_SET_DATA_VALUE_PTR(4,(void *)(&InputData_m0_c4_d2_iAlarm));
          _SFD_SET_DATA_VALUE_PTR(3,(void *)(&InputData_m0_c4_d3_iReset));
          _SFD_SET_DATA_VALUE_PTR(1,(void *)(&OutputData_m0_c4_d4_oShutoff));
          _SFD_SET_DATA_VALUE_PTR(2,(void *)(&OutputData_m0_c4_d5_oAlarm));
        }
      }
    }else{
      sf_debug_reset_current_state_configuration(_BVH2_App_LayerMachineNumber_,chartInstance.chartNumber,chartInstance.instanceNumber);
    }
  }
  chartInstance.chartInfo.chartInitialized = 1;
}

static void enable_BVH2_App_Layer_sfun_c4( SimStruct *S)
{
}

static void disable_BVH2_App_Layer_sfun_c4( SimStruct *S)
{
}

void BVH2_App_Layer_sfun_c4_sizes_registry(SimStruct *S)
{
  ssSetSFInitOutput(S,0);
  ssSetNumInputPorts((SimStruct *)S, 2);
  ssSetInputPortDataType((SimStruct *)S,0,SS_BOOLEAN); /* InputData_m0_c4_d2_iAlarm */
  ssSetInputPortRequiredContiguous(S,0,1);
  ssSetInputPortWidth((SimStruct *)S,0,1);
  ssSetInputPortDirectFeedThrough((SimStruct *)S,0,1);
  ssSetInputPortDataType((SimStruct *)S,1,SS_BOOLEAN); /* InputData_m0_c4_d3_iReset */
  ssSetInputPortRequiredContiguous(S,1,1);
  ssSetInputPortWidth((SimStruct *)S,1,1);
  ssSetInputPortDirectFeedThrough((SimStruct *)S,1,1);
  ssSetNumOutputPorts((SimStruct *)S, 3);
  ssSetOutputPortDataType((SimStruct *)S,0,SS_DOUBLE);
  ssSetOutputPortWidth((SimStruct *)S,0,1);
  ssSetOutputPortDataType((SimStruct *)S,1,SS_BOOLEAN); /* OutputData_m0_c4_d4_oShutoff */
  ssSetOutputPortWidth((SimStruct *)S,1,1);
  ssSetOutputPortDataType((SimStruct *)S,2,SS_BOOLEAN); /* OutputData_m0_c4_d5_oAlarm */
  ssSetOutputPortWidth((SimStruct *)S,2,1);
  if(sim_mode_is_rtw_gen(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("BVH2_App_Layer",4);
    ssSetStateflowIsInlinable((SimStruct *)S,chartIsInlinable);
    if(chartIsInlinable) {
      ssSetInputPortReusable((SimStruct *)S,0,1);
      ssSetInputPortReusable((SimStruct *)S,1,1);
      sf_mark_chart_expressionable_inputs((SimStruct *)S,"BVH2_App_Layer",4,2);
      sf_mark_chart_reusable_outputs((SimStruct *)S,"BVH2_App_Layer",4,2);
    }
    {
      int dtId;
      char *chartInstanceTypedefName =
        sf_chart_instance_typedef_name("BVH2_App_Layer",4);
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
  ssSetChecksum0(S,(649361435U));
  ssSetChecksum1(S,(3463553448U));
  ssSetChecksum2(S,(2309531357U));
  ssSetChecksum3(S,(3833722031U));
  ssSetExplicitFCSSCtrl(S,1);
}

void terminate_BVH2_App_Layer_sfun_c4(SimStruct *S)
{
}
static void mdlRTW_BVH2_App_Layer_sfun_c4(SimStruct *S)
{
}

void sf_BVH2_App_Layer_sfun_c4( void *);
void BVH2_App_Layer_sfun_c4_registry(SimStruct *S)
{
  chartInstance.chartInfo.chartInstance = NULL;
  chartInstance.chartInfo.chartInitialized = 0;
  chartInstance.chartInfo.sFunctionGateway = sf_BVH2_App_Layer_sfun_c4;
  chartInstance.chartInfo.initializeChart = initialize_BVH2_App_Layer_sfun_c4;
  chartInstance.chartInfo.terminateChart = terminate_BVH2_App_Layer_sfun_c4;
  chartInstance.chartInfo.enableChart = enable_BVH2_App_Layer_sfun_c4;
  chartInstance.chartInfo.disableChart = disable_BVH2_App_Layer_sfun_c4;
  chartInstance.chartInfo.mdlRTW = mdlRTW_BVH2_App_Layer_sfun_c4;
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

void sf_BVH2_App_Layer_sfun_c4(void *chartInstanceVoidPtr)
{
  /* Save current event being processed */
  uint8_T previousEvent;
  previousEvent = _sfEvent_;

  /* Update Stateflow time variable */
  _sfTime_ = ssGetT(chartInstance.S);

  /* Call this chart */
  _sfEvent_ = CALL_EVENT;
  BVH2_App_Layer_sfun_c4();
  _sfEvent_ = previousEvent;
}

