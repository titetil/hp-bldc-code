#include "BVH2_App_Layer_sfun.h"
#include "BVH2_App_Layer_sfun_c3.h"
#define mexPrintf                       sf_mex_printf
#ifdef printf
#undef printf
#endif
#define printf                          sf_mex_printf
#define CHARTINSTANCE_CHARTNUMBER       (chartInstance.chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER    (chartInstance.instanceNumber)
#include "BVH2_App_Layer_sfun_debug_macros.h"
#define IN_NO_ACTIVE_CHILD              (0)
#define IN_m0_c3_s1_CntOverTemp         1
#define IN_m0_c3_s2_greenTemp           2
#define IN_m0_c3_s3_redTemp             3
#define IN_m0_c3_s4_reset               4
#define m0_c3_d9_Treshold_red           chartInstance.ConstantData.m0_c3_d9_Treshold_red
#define m0_c3_d8_Treshold_green         chartInstance.ConstantData.m0_c3_d8_Treshold_green
#define m0_c3_d7_Threshold_RedAlarm     chartInstance.ConstantData.m0_c3_d7_Threshold_RedAlarm
static SFBVH2_App_Layer_sfun_c3InstanceStruct chartInstance;
#define InputData_m0_c3_d2_iTemp        (((uint16_T *)(ssGetInputPortSignal(chartInstance.S,0)))[0])
#define InputData_m0_c3_d3_iReset       (((boolean_T *)(ssGetInputPortSignal(chartInstance.S,1)))[0])
#define OutputData_m0_c3_d4_oTempRedAlarm (((boolean_T *)(ssGetOutputPortSignal(chartInstance.S,1)))[0])
#define OutputData_m0_c3_d5_oTempAlarm  (((boolean_T *)(ssGetOutputPortSignal(chartInstance.S,2)))[0])
#define OutputData_m0_c3_d6_odPumpOff   (((boolean_T *)(ssGetOutputPortSignal(chartInstance.S,3)))[0])
static uint8_T m0_c3_u8_s32_(int32_T b);
static uint8_T m0_c3_u8_s8_(int8_T b);

static uint8_T m0_c3_u8_s32_(int32_T b)
{
  uint8_T a;
  a = (uint8_T)b;
  if(a != b) {
    sf_debug_overflow_detection(0);
  }
  return a;
}
static uint8_T m0_c3_u8_s8_(int8_T b)
{
  uint8_T a;
  a = (uint8_T)b;
  if(b < 0) {
    sf_debug_overflow_detection(0);
  }
  return a;
}

static void exit_atomic_m0_c3_s1_CntOverTemp(void);
static void enter_atomic_m0_c3_s2_greenTemp(void);
static void exit_atomic_m0_c3_s2_greenTemp(void);
static void exit_atomic_m0_c3_s3_redTemp(void);
static void exit_atomic_m0_c3_s4_reset(void);
void BVH2_App_Layer_sfun_c3(void)
{
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,2);
  if(chartInstance.State.is_active_BVH2_App_Layer_sfun_c3 == 0) {
    _SFD_CC_CALL(CHART_ENTER_ENTRY_FUNCTION_TAG,2);
    chartInstance.State.is_active_BVH2_App_Layer_sfun_c3 = 1;
    _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,2);
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,4);
    if(CV_TRANSITION_EVAL(4, _SFD_CCP_CALL(4,0,(InputData_m0_c3_d3_iReset ==
        1))) != 0) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,4);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,4);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,4);
      _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,3);
      _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,3,0);
      chartInstance.State.is_BVH2_App_Layer_sfun_c3 = IN_m0_c3_s4_reset;
      _SFD_CS_CALL(STATE_ACTIVE_TAG,3);
      _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,3);
      chartInstance.LocalData.m0_c3_d1_Counter = m0_c3_u8_s8_(0);
      _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,3,1);
      _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,3);
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
    }
  } else {
    switch(chartInstance.State.is_BVH2_App_Layer_sfun_c3) {
     case IN_m0_c3_s1_CntOverTemp:
      CV_CHART_EVAL(2,0,1);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,0);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,0,0);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,1);
      if(CV_TRANSITION_EVAL(1,
        _SFD_CCP_CALL(1,0,(chartInstance.LocalData.m0_c3_d1_Counter > 80))) !=
       0) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,1);
        _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,1);
        _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,1);
        exit_atomic_m0_c3_s1_CntOverTemp();
        enter_atomic_m0_c3_s2_greenTemp();
      } else {
        _SFD_CS_CALL(STATE_BEFORE_DURING_ACTION_TAG,0);
        chartInstance.LocalData.m0_c3_d1_Counter =
          m0_c3_u8_s32_(chartInstance.LocalData.m0_c3_d1_Counter + 1);
        _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,0,1);
        _SFD_CS_CALL(STATE_AFTER_DURING_ACTION_TAG,0);
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
      break;
     case IN_m0_c3_s2_greenTemp:
      CV_CHART_EVAL(2,0,2);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,1);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,1,0);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,0);
      if(CV_TRANSITION_EVAL(0, _SFD_CCP_CALL(0,0,(InputData_m0_c3_d2_iTemp <
          m0_c3_d9_Treshold_red))) != 0) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,0);
        _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,0);
        _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,0);
        exit_atomic_m0_c3_s2_greenTemp();
        _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,2);
        _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,2,0);
        chartInstance.State.is_BVH2_App_Layer_sfun_c3 = IN_m0_c3_s3_redTemp;
        _SFD_CS_CALL(STATE_ACTIVE_TAG,2);
        _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,2);
        OutputData_m0_c3_d4_oTempRedAlarm = 1;
        _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,2,1);
        OutputData_m0_c3_d5_oTempAlarm = 1;
        _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,2,2);
        OutputData_m0_c3_d6_odPumpOff = 0;
        _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,2,3);
        _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,2);
        _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,2);
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,1);
      break;
     case IN_m0_c3_s3_redTemp:
      CV_CHART_EVAL(2,0,3);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,2);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,2,0);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,3);
      if(CV_TRANSITION_EVAL(3, _SFD_CCP_CALL(3,0,(InputData_m0_c3_d2_iTemp >
          m0_c3_d8_Treshold_green))) != 0) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,3);
        _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,3);
        _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,3);
        exit_atomic_m0_c3_s3_redTemp();
        enter_atomic_m0_c3_s2_greenTemp();
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,2);
      break;
     case IN_m0_c3_s4_reset:
      CV_CHART_EVAL(2,0,4);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,3);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,3,0);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,2);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,2);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,2);
      exit_atomic_m0_c3_s4_reset();
      _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,0);
      _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,0,0);
      chartInstance.State.is_BVH2_App_Layer_sfun_c3 = IN_m0_c3_s1_CntOverTemp;
      _SFD_CS_CALL(STATE_ACTIVE_TAG,0);
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
      break;
     default:
      CV_CHART_EVAL(2,0,0);
      break;
    }
  }
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,2);
}

static void exit_atomic_m0_c3_s1_CntOverTemp(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,0);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,0,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c3 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,0);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
}

static void enter_atomic_m0_c3_s2_greenTemp(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,1);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c3 = IN_m0_c3_s2_greenTemp;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,1);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,1);
  OutputData_m0_c3_d4_oTempRedAlarm = 0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,1);
  OutputData_m0_c3_d5_oTempAlarm = 0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,2);
  OutputData_m0_c3_d6_odPumpOff = 0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,3);
  chartInstance.LocalData.m0_c3_d1_Counter = m0_c3_u8_s8_(0);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,4);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,1);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,1);
}

static void exit_atomic_m0_c3_s2_greenTemp(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,1);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,1,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c3 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,1);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,1);
}

static void exit_atomic_m0_c3_s3_redTemp(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,2);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,2,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c3 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,2);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,2);
}

static void exit_atomic_m0_c3_s4_reset(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,3);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,3,0);
  chartInstance.State.is_BVH2_App_Layer_sfun_c3 = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,3);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
}

void sf_BVH2_App_Layer_sfun_c3_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2852234367U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3353627137U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(504842130U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3049060185U);
}
/*
 * Chart initialization function
 */
/* work around the buggy macro in simstruc.h until it is fixed */
#define cdrGetOutputPortReusable(S,port) \
  ( (S)->portInfo.outputs[(port)].attributes.optimOpts != \
   SS_NOT_REUSABLE_AND_GLOBAL )

static void initialize_BVH2_App_Layer_sfun_c3( SimStruct *S)
{

  {
    chartInstance.LocalData.m0_c3_d1_Counter = 0;
    m0_c3_d9_Treshold_red = 72;
    m0_c3_d8_Treshold_green = 185;
    m0_c3_d7_Threshold_RedAlarm = 72;
    if(!cdrGetOutputPortReusable(S,1)) {
      OutputData_m0_c3_d4_oTempRedAlarm = 0;
    }
    if(!cdrGetOutputPortReusable(S,2)) {
      OutputData_m0_c3_d5_oTempAlarm = 0;
    }
    if(!cdrGetOutputPortReusable(S,3)) {
      OutputData_m0_c3_d6_odPumpOff = 0;
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
            3,
            4,
            5,
            9,
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

            _SFD_SET_DATA_PROPS(1,
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
            _SFD_SET_DATA_PROPS(8,
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
            _SFD_SET_DATA_PROPS(7,
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
            _SFD_SET_DATA_PROPS(4,
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
            _SFD_SET_DATA_PROPS(6,
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
            _SFD_SET_DATA_PROPS(0,
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
            _SFD_SET_DATA_PROPS(5,
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
            _SFD_STATE_INFO(0,0,0);
            _SFD_STATE_INFO(1,0,0);
            _SFD_STATE_INFO(2,0,0);
            _SFD_STATE_INFO(3,0,0);
            _SFD_CH_SUBSTATE_COUNT(4);
            _SFD_CH_SUBSTATE_DECOMP(0);
            _SFD_CH_SUBSTATE_INDEX(0,0);
            _SFD_CH_SUBSTATE_INDEX(1,1);
            _SFD_CH_SUBSTATE_INDEX(2,2);
            _SFD_CH_SUBSTATE_INDEX(3,3);
            _SFD_ST_SUBSTATE_COUNT(0,0);
            _SFD_ST_SUBSTATE_COUNT(1,0);
            _SFD_ST_SUBSTATE_COUNT(2,0);
            _SFD_ST_SUBSTATE_COUNT(3,0);
          }
          _SFD_CV_INIT_CHART(4,1,0,0);
          {
            _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(1,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(2,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(3,0,0,0,0,0,NULL,NULL);
          }

          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {13};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(1,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {21};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(0,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          _SFD_CV_INIT_TRANS(2,0,NULL,NULL,0,NULL);

          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {10};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(4,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {22};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(3,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          _SFD_STATE_COV_WTS(0,1,2,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0};
            static unsigned int sEndEntryMap[] = {0};
            static unsigned int sStartDuringMap[] = {0,20};
            static unsigned int sEndDuringMap[] = {0,29};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(0,
             1,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             2,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(1,5,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,17,37,54,69};
            static unsigned int sEndEntryMap[] = {0,36,53,68,81};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(1,
             5,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(2,4,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,15,35,52};
            static unsigned int sEndEntryMap[] = {0,34,51,66};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(2,
             4,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(3,2,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,13};
            static unsigned int sEndEntryMap[] = {0,25};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(3,
             2,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_TRANS_COV_WTS(1,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {13};
            _SFD_TRANS_COV_MAPS(1,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(0,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {21};
            _SFD_TRANS_COV_MAPS(0,
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
            static unsigned int sEndGuardMap[] = {22};
            _SFD_TRANS_COV_MAPS(3,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_SET_DATA_VALUE_PTR(1,(void *)(&InputData_m0_c3_d2_iTemp));
          _SFD_SET_DATA_VALUE_PTR(2,(void
            *)(&OutputData_m0_c3_d4_oTempRedAlarm));
          _SFD_SET_DATA_VALUE_PTR(8,(void *)(&m0_c3_d9_Treshold_red));
          _SFD_SET_DATA_VALUE_PTR(7,(void *)(&InputData_m0_c3_d3_iReset));
          _SFD_SET_DATA_VALUE_PTR(4,(void *)(&OutputData_m0_c3_d5_oTempAlarm));
          _SFD_SET_DATA_VALUE_PTR(6,(void *)(&m0_c3_d8_Treshold_green));
          _SFD_SET_DATA_VALUE_PTR(0,(void *)(&m0_c3_d7_Threshold_RedAlarm));
          _SFD_SET_DATA_VALUE_PTR(5,(void *)(&OutputData_m0_c3_d6_odPumpOff));
          _SFD_SET_DATA_VALUE_PTR(3,(void
            *)(&chartInstance.LocalData.m0_c3_d1_Counter));
        }
      }
    }else{
      sf_debug_reset_current_state_configuration(_BVH2_App_LayerMachineNumber_,chartInstance.chartNumber,chartInstance.instanceNumber);
    }
  }
  chartInstance.chartInfo.chartInitialized = 1;
}

static void enable_BVH2_App_Layer_sfun_c3( SimStruct *S)
{
}

static void disable_BVH2_App_Layer_sfun_c3( SimStruct *S)
{
}

void BVH2_App_Layer_sfun_c3_sizes_registry(SimStruct *S)
{
  ssSetSFInitOutput(S,0);
  ssSetNumInputPorts((SimStruct *)S, 2);
  ssSetInputPortDataType((SimStruct *)S,0,SS_UINT16); /* InputData_m0_c3_d2_iTemp */
  ssSetInputPortRequiredContiguous(S,0,1);
  ssSetInputPortWidth((SimStruct *)S,0,1);
  ssSetInputPortDirectFeedThrough((SimStruct *)S,0,1);
  ssSetInputPortDataType((SimStruct *)S,1,SS_BOOLEAN); /* InputData_m0_c3_d3_iReset */
  ssSetInputPortRequiredContiguous(S,1,1);
  ssSetInputPortWidth((SimStruct *)S,1,1);
  ssSetInputPortDirectFeedThrough((SimStruct *)S,1,1);
  ssSetNumOutputPorts((SimStruct *)S, 4);
  ssSetOutputPortDataType((SimStruct *)S,0,SS_DOUBLE);
  ssSetOutputPortWidth((SimStruct *)S,0,1);
  ssSetOutputPortDataType((SimStruct *)S,1,SS_BOOLEAN); /* OutputData_m0_c3_d4_oTempRedAlarm */
  ssSetOutputPortWidth((SimStruct *)S,1,1);
  ssSetOutputPortDataType((SimStruct *)S,2,SS_BOOLEAN); /* OutputData_m0_c3_d5_oTempAlarm */
  ssSetOutputPortWidth((SimStruct *)S,2,1);
  ssSetOutputPortDataType((SimStruct *)S,3,SS_BOOLEAN); /* OutputData_m0_c3_d6_odPumpOff */
  ssSetOutputPortWidth((SimStruct *)S,3,1);
  if(sim_mode_is_rtw_gen(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("BVH2_App_Layer",3);
    ssSetStateflowIsInlinable((SimStruct *)S,chartIsInlinable);
    if(chartIsInlinable) {
      ssSetInputPortReusable((SimStruct *)S,0,1);
      ssSetInputPortReusable((SimStruct *)S,1,1);
      sf_mark_chart_expressionable_inputs((SimStruct *)S,"BVH2_App_Layer",3,2);
      sf_mark_chart_reusable_outputs((SimStruct *)S,"BVH2_App_Layer",3,3);
    }
    {
      int dtId;
      char *chartInstanceTypedefName =
        sf_chart_instance_typedef_name("BVH2_App_Layer",3);
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
  ssSetChecksum0(S,(2852234367U));
  ssSetChecksum1(S,(3353627137U));
  ssSetChecksum2(S,(504842130U));
  ssSetChecksum3(S,(3049060185U));
  ssSetExplicitFCSSCtrl(S,1);
}

void terminate_BVH2_App_Layer_sfun_c3(SimStruct *S)
{
}
static void mdlRTW_BVH2_App_Layer_sfun_c3(SimStruct *S)
{
}

void sf_BVH2_App_Layer_sfun_c3( void *);
void BVH2_App_Layer_sfun_c3_registry(SimStruct *S)
{
  chartInstance.chartInfo.chartInstance = NULL;
  chartInstance.chartInfo.chartInitialized = 0;
  chartInstance.chartInfo.sFunctionGateway = sf_BVH2_App_Layer_sfun_c3;
  chartInstance.chartInfo.initializeChart = initialize_BVH2_App_Layer_sfun_c3;
  chartInstance.chartInfo.terminateChart = terminate_BVH2_App_Layer_sfun_c3;
  chartInstance.chartInfo.enableChart = enable_BVH2_App_Layer_sfun_c3;
  chartInstance.chartInfo.disableChart = disable_BVH2_App_Layer_sfun_c3;
  chartInstance.chartInfo.mdlRTW = mdlRTW_BVH2_App_Layer_sfun_c3;
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

void sf_BVH2_App_Layer_sfun_c3(void *chartInstanceVoidPtr)
{
  /* Save current event being processed */
  uint8_T previousEvent;
  previousEvent = _sfEvent_;

  /* Update Stateflow time variable */
  _sfTime_ = ssGetT(chartInstance.S);

  /* Call this chart */
  _sfEvent_ = CALL_EVENT;
  BVH2_App_Layer_sfun_c3();
  _sfEvent_ = previousEvent;
}

