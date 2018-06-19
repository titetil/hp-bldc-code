#include "BVH2_App_Layer_sfun.h"
#include "BVH2_App_Layer_sfun_c8.h"
#define mexPrintf                       sf_mex_printf
#ifdef printf
#undef printf
#endif
#define printf                          sf_mex_printf
#define CHARTINSTANCE_CHARTNUMBER       (chartInstance.chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER    (chartInstance.instanceNumber)
#include "BVH2_App_Layer_sfun_debug_macros.h"
#define IN_NO_ACTIVE_CHILD              (0)
#define IN_m0_c8_s2_Alarm               1
#define IN_m0_c8_s3_CMD_MJP_0_100       2
#define IN_m0_c8_s4_LimpModeLow         3
#define IN_m0_c8_s5_LimpModeLow1        4
#define IN_m0_c8_s6_Normal3             5
#define IN_m0_c8_s7_PumpOff2            6
#define IN_m0_c8_s8_SaturationHigh      7
#define IN_m0_c8_s9_SaturationLow       8
#define m0_c8_d21_WAITTIME              chartInstance.ConstantData.m0_c8_d21_WAITTIME
#define m0_c8_d20_Low_Freq              chartInstance.ConstantData.m0_c8_d20_Low_Freq
#define m0_c8_d19_High_Freq             chartInstance.ConstantData.m0_c8_d19_High_Freq
static SFBVH2_App_Layer_sfun_c8InstanceStruct chartInstance;
#define InputData_m0_c8_d10_iPWM_Freq   (((real_T *)(ssGetInputPortSignal(chartInstance.S,0)))[0])
#define InputData_m0_c8_d11_idPWM       (((real_T *)(ssGetInputPortSignal(chartInstance.S,1)))[0])
#define InputData_m0_c8_d12_iReset      (((real_T *)(ssGetInputPortSignal(chartInstance.S,2)))[0])
#define OutputData_m0_c8_d13_odFixedValueSel (((real_T *)(ssGetOutputPortSignal(chartInstance.S,1)))[0])
#define OutputData_m0_c8_d14_odPumpOff  (((real_T *)(ssGetOutputPortSignal(chartInstance.S,2)))[0])
#define OutputData_m0_c8_d15_odPI_Disable (((real_T *)(ssGetOutputPortSignal(chartInstance.S,3)))[0])
#define OutputData_m0_c8_d16_oPWM_SC_Alarm (((real_T *)(ssGetOutputPortSignal(chartInstance.S,4)))[0])
#define OutputData_m0_c8_d17_oPWM_Alarm (((real_T *)(ssGetOutputPortSignal(chartInstance.S,5)))[0])
#define OutputData_m0_c8_d18_odFixedLowValueSel (((real_T *)(ssGetOutputPortSignal(chartInstance.S,6)))[0])
static uint8_T m0_c8_u8_d_(real_T b);
static uint16_T m0_c8_u16_d_(real_T b);

static uint8_T m0_c8_u8_d_(real_T b)
{
  uint8_T a;
  a = (uint8_T)b;
  if((real_T)a != (b < 0.0 ? ceil(b) : floor(b))) {
    sf_debug_overflow_detection(0);
  }
  return a;
}
static uint16_T m0_c8_u16_d_(real_T b)
{
  uint16_T a;
  a = (uint16_T)b;
  if((real_T)a != (b < 0.0 ? ceil(b) : floor(b))) {
    sf_debug_overflow_detection(0);
  }
  return a;
}

static void enter_atomic_m0_c8_s2_Alarm(void);
static void exit_atomic_m0_c8_s2_Alarm(void);
static void enter_atomic_m0_c8_s3_CMD_MJP_0_100(void);
static void exit_atomic_m0_c8_s3_CMD_MJP_0_100(void);
static void enter_atomic_m0_c8_s4_LimpModeLow(void);
static void exit_atomic_m0_c8_s4_LimpModeLow(void);
static void enter_atomic_m0_c8_s5_LimpModeLow1(void);
static void exit_atomic_m0_c8_s5_LimpModeLow1(void);
static void enter_atomic_m0_c8_s6_Normal3(void);
static void exit_atomic_m0_c8_s6_Normal3(void);
static void enter_atomic_m0_c8_s7_PumpOff2(void);
static void exit_atomic_m0_c8_s7_PumpOff2(void);
static void enter_atomic_m0_c8_s8_SaturationHigh(void);
static void exit_atomic_m0_c8_s8_SaturationHigh(void);
static void enter_atomic_m0_c8_s9_SaturationLow(void);
static void exit_atomic_m0_c8_s9_SaturationLow(void);
void BVH2_App_Layer_sfun_c8(void)
{
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,7);
  if(chartInstance.State.is_active_BVH2_App_Layer_sfun_c8 == 0) {
    _SFD_CC_CALL(CHART_ENTER_ENTRY_FUNCTION_TAG,7);
    chartInstance.State.is_active_BVH2_App_Layer_sfun_c8 = 1;
    _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,7);
    _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,2);
    _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,2,0);
    chartInstance.State.is_active_m0_c8_s1_PWMinput_handling = 1;
    _SFD_CS_CALL(STATE_ACTIVE_TAG,2);
    _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,2);
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,2);
    if(CV_TRANSITION_EVAL(2,
      _SFD_CCP_CALL(2,0,(m0_c8_u8_d_(InputData_m0_c8_d12_iReset) == 1))) != 0) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,2);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,2);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,2);
      enter_atomic_m0_c8_s7_PumpOff2();
    }
  } else {
    _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,2);
    _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,2,0);
    switch(chartInstance.State.is_m0_c8_s1_PWMinput_handling) {
     case IN_m0_c8_s2_Alarm:
      CV_STATE_EVAL(2,0,1);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,8);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,8,0);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,6);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,6);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,6);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,11);
      if(CV_TRANSITION_EVAL(11,
        (_SFD_CCP_CALL(11,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 1)) != 0)
        || (_SFD_CCP_CALL(11,1,(
           m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 199)) != 0)) != 0) {
        if(sf_debug_transition_conflict_check_enabled()) {
          unsigned int transitionList[8];
          unsigned int numTransitions=1;
          transitionList[0] = 11;
          sf_debug_transition_conflict_check_begin();
          if((m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) < m0_c8_d20_Low_Freq)
           || (m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq
             ) > m0_c8_d19_High_Freq)) {
            transitionList[numTransitions] = 14;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5) {
            transitionList[numTransitions] = 15;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
            transitionList[numTransitions] = 12;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
            transitionList[numTransitions] = 7;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
            transitionList[numTransitions] = 3;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
            transitionList[numTransitions] = 4;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
            transitionList[numTransitions] = 16;
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
        exit_atomic_m0_c8_s2_Alarm();
        enter_atomic_m0_c8_s3_CMD_MJP_0_100();
      } else {
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,14);
        if(CV_TRANSITION_EVAL(14,
          (_SFD_CCP_CALL(14,0,(m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) <
             m0_c8_d20_Low_Freq))
           != 0) ||
          (_SFD_CCP_CALL(14,1,(m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) >
             m0_c8_d19_High_Freq)) != 0)) != 0) {
          if(sf_debug_transition_conflict_check_enabled()) {
            unsigned int transitionList[7];
            unsigned int numTransitions=1;
            transitionList[0] = 14;
            sf_debug_transition_conflict_check_begin();
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5) {
              transitionList[numTransitions] = 15;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
              transitionList[numTransitions] = 12;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
              transitionList[numTransitions] = 7;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
              transitionList[numTransitions] = 3;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
              transitionList[numTransitions] = 4;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
              transitionList[numTransitions] = 16;
              numTransitions++;
            }
            sf_debug_transition_conflict_check_end();
            if(numTransitions>1) {
              _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
            }
          }
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,14);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,14);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,14);
          exit_atomic_m0_c8_s2_Alarm();
          enter_atomic_m0_c8_s2_Alarm();
        } else {
          _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,15);
          if(CV_TRANSITION_EVAL(15,
            _SFD_CCP_CALL(15,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5)))
           != 0) {
            if(sf_debug_transition_conflict_check_enabled()) {
              unsigned int transitionList[6];
              unsigned int numTransitions=1;
              transitionList[0] = 15;
              sf_debug_transition_conflict_check_begin();
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
                transitionList[numTransitions] = 12;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
                transitionList[numTransitions] = 7;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                transitionList[numTransitions] = 3;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                transitionList[numTransitions] = 4;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                transitionList[numTransitions] = 16;
                numTransitions++;
              }
              sf_debug_transition_conflict_check_end();
              if(numTransitions>1) {
                _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
              }
            }
            _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,15);
            _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,15);
            _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,15);
            exit_atomic_m0_c8_s2_Alarm();
            enter_atomic_m0_c8_s4_LimpModeLow();
          } else {
            _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,12);
            if(CV_TRANSITION_EVAL(12,
              _SFD_CCP_CALL(12,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9)))
             != 0) {
              if(sf_debug_transition_conflict_check_enabled()) {
                unsigned int transitionList[5];
                unsigned int numTransitions=1;
                transitionList[0] = 12;
                sf_debug_transition_conflict_check_begin();
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
                  transitionList[numTransitions] = 7;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                  transitionList[numTransitions] = 3;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                  transitionList[numTransitions] = 4;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                  transitionList[numTransitions] = 16;
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
              exit_atomic_m0_c8_s2_Alarm();
              enter_atomic_m0_c8_s5_LimpModeLow1();
            } else {
              _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,7);
              if(CV_TRANSITION_EVAL(7,
                _SFD_CCP_CALL(7,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >
                  191))) != 0) {
                if(sf_debug_transition_conflict_check_enabled()) {
                  unsigned int transitionList[4];
                  unsigned int numTransitions=1;
                  transitionList[0] = 7;
                  sf_debug_transition_conflict_check_begin();
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                    transitionList[numTransitions] = 3;
                    numTransitions++;
                  }
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                    transitionList[numTransitions] = 4;
                    numTransitions++;
                  }
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                    transitionList[numTransitions] = 16;
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
                exit_atomic_m0_c8_s2_Alarm();
                enter_atomic_m0_c8_s8_SaturationHigh();
              } else {
                _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,3);
                if(CV_TRANSITION_EVAL(3,
                  _SFD_CCP_CALL(3,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >=
                    23))) != 0) {
                  if(sf_debug_transition_conflict_check_enabled()) {
                    unsigned int transitionList[3];
                    unsigned int numTransitions=1;
                    transitionList[0] = 3;
                    sf_debug_transition_conflict_check_begin();
                    if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                      transitionList[numTransitions] = 4;
                      numTransitions++;
                    }
                    if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                      transitionList[numTransitions] = 16;
                      numTransitions++;
                    }
                    sf_debug_transition_conflict_check_end();
                    if(numTransitions>1) {
                      _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
                    }
                  }
                  _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,3);
                  _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,3);
                  _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,3);
                  exit_atomic_m0_c8_s2_Alarm();
                  enter_atomic_m0_c8_s6_Normal3();
                } else {
                  _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,4);
                  if(CV_TRANSITION_EVAL(4,
                    _SFD_CCP_CALL(4,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >
                      19))) != 0) {
                    if(sf_debug_transition_conflict_check_enabled()) {
                      unsigned int transitionList[2];
                      unsigned int numTransitions=1;
                      transitionList[0] = 4;
                      sf_debug_transition_conflict_check_begin();
                      if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                        transitionList[numTransitions] = 16;
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
                    exit_atomic_m0_c8_s2_Alarm();
                    enter_atomic_m0_c8_s9_SaturationLow();
                  } else {
                    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,16);
                    if(CV_TRANSITION_EVAL(16,
                      _SFD_CCP_CALL(16,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM)
                        <= 19))) != 0) {
                      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,16);
                      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,16);
                      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,16);
                      exit_atomic_m0_c8_s2_Alarm();
                      enter_atomic_m0_c8_s7_PumpOff2();
                    }
                  }
                }
              }
            }
          }
        }
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,8);
      break;
     case IN_m0_c8_s3_CMD_MJP_0_100:
      CV_STATE_EVAL(2,0,2);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,9);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,9,0);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,0);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,0);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,0);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,11);
      if(CV_TRANSITION_EVAL(11,
        (_SFD_CCP_CALL(11,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 1)) != 0)
        || (_SFD_CCP_CALL(11,1,(
           m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 199)) != 0)) != 0) {
        if(sf_debug_transition_conflict_check_enabled()) {
          unsigned int transitionList[8];
          unsigned int numTransitions=1;
          transitionList[0] = 11;
          sf_debug_transition_conflict_check_begin();
          if((m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) < m0_c8_d20_Low_Freq)
           || (m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq
             ) > m0_c8_d19_High_Freq)) {
            transitionList[numTransitions] = 14;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5) {
            transitionList[numTransitions] = 15;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
            transitionList[numTransitions] = 12;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
            transitionList[numTransitions] = 7;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
            transitionList[numTransitions] = 3;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
            transitionList[numTransitions] = 4;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
            transitionList[numTransitions] = 16;
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
        exit_atomic_m0_c8_s3_CMD_MJP_0_100();
        enter_atomic_m0_c8_s3_CMD_MJP_0_100();
      } else {
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,14);
        if(CV_TRANSITION_EVAL(14,
          (_SFD_CCP_CALL(14,0,(m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) <
             m0_c8_d20_Low_Freq))
           != 0) ||
          (_SFD_CCP_CALL(14,1,(m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) >
             m0_c8_d19_High_Freq)) != 0)) != 0) {
          if(sf_debug_transition_conflict_check_enabled()) {
            unsigned int transitionList[7];
            unsigned int numTransitions=1;
            transitionList[0] = 14;
            sf_debug_transition_conflict_check_begin();
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5) {
              transitionList[numTransitions] = 15;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
              transitionList[numTransitions] = 12;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
              transitionList[numTransitions] = 7;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
              transitionList[numTransitions] = 3;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
              transitionList[numTransitions] = 4;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
              transitionList[numTransitions] = 16;
              numTransitions++;
            }
            sf_debug_transition_conflict_check_end();
            if(numTransitions>1) {
              _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
            }
          }
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,14);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,14);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,14);
          exit_atomic_m0_c8_s3_CMD_MJP_0_100();
          enter_atomic_m0_c8_s2_Alarm();
        } else {
          _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,15);
          if(CV_TRANSITION_EVAL(15,
            _SFD_CCP_CALL(15,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5)))
           != 0) {
            if(sf_debug_transition_conflict_check_enabled()) {
              unsigned int transitionList[6];
              unsigned int numTransitions=1;
              transitionList[0] = 15;
              sf_debug_transition_conflict_check_begin();
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
                transitionList[numTransitions] = 12;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
                transitionList[numTransitions] = 7;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                transitionList[numTransitions] = 3;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                transitionList[numTransitions] = 4;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                transitionList[numTransitions] = 16;
                numTransitions++;
              }
              sf_debug_transition_conflict_check_end();
              if(numTransitions>1) {
                _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
              }
            }
            _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,15);
            _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,15);
            _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,15);
            exit_atomic_m0_c8_s3_CMD_MJP_0_100();
            enter_atomic_m0_c8_s4_LimpModeLow();
          } else {
            _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,12);
            if(CV_TRANSITION_EVAL(12,
              _SFD_CCP_CALL(12,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9)))
             != 0) {
              if(sf_debug_transition_conflict_check_enabled()) {
                unsigned int transitionList[5];
                unsigned int numTransitions=1;
                transitionList[0] = 12;
                sf_debug_transition_conflict_check_begin();
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
                  transitionList[numTransitions] = 7;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                  transitionList[numTransitions] = 3;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                  transitionList[numTransitions] = 4;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                  transitionList[numTransitions] = 16;
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
              exit_atomic_m0_c8_s3_CMD_MJP_0_100();
              enter_atomic_m0_c8_s5_LimpModeLow1();
            } else {
              _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,7);
              if(CV_TRANSITION_EVAL(7,
                _SFD_CCP_CALL(7,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >
                  191))) != 0) {
                if(sf_debug_transition_conflict_check_enabled()) {
                  unsigned int transitionList[4];
                  unsigned int numTransitions=1;
                  transitionList[0] = 7;
                  sf_debug_transition_conflict_check_begin();
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                    transitionList[numTransitions] = 3;
                    numTransitions++;
                  }
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                    transitionList[numTransitions] = 4;
                    numTransitions++;
                  }
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                    transitionList[numTransitions] = 16;
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
                exit_atomic_m0_c8_s3_CMD_MJP_0_100();
                enter_atomic_m0_c8_s8_SaturationHigh();
              } else {
                _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,3);
                if(CV_TRANSITION_EVAL(3,
                  _SFD_CCP_CALL(3,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >=
                    23))) != 0) {
                  if(sf_debug_transition_conflict_check_enabled()) {
                    unsigned int transitionList[3];
                    unsigned int numTransitions=1;
                    transitionList[0] = 3;
                    sf_debug_transition_conflict_check_begin();
                    if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                      transitionList[numTransitions] = 4;
                      numTransitions++;
                    }
                    if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                      transitionList[numTransitions] = 16;
                      numTransitions++;
                    }
                    sf_debug_transition_conflict_check_end();
                    if(numTransitions>1) {
                      _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
                    }
                  }
                  _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,3);
                  _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,3);
                  _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,3);
                  exit_atomic_m0_c8_s3_CMD_MJP_0_100();
                  enter_atomic_m0_c8_s6_Normal3();
                } else {
                  _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,4);
                  if(CV_TRANSITION_EVAL(4,
                    _SFD_CCP_CALL(4,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >
                      19))) != 0) {
                    if(sf_debug_transition_conflict_check_enabled()) {
                      unsigned int transitionList[2];
                      unsigned int numTransitions=1;
                      transitionList[0] = 4;
                      sf_debug_transition_conflict_check_begin();
                      if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                        transitionList[numTransitions] = 16;
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
                    exit_atomic_m0_c8_s3_CMD_MJP_0_100();
                    enter_atomic_m0_c8_s9_SaturationLow();
                  } else {
                    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,16);
                    if(CV_TRANSITION_EVAL(16,
                      _SFD_CCP_CALL(16,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM)
                        <= 19))) != 0) {
                      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,16);
                      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,16);
                      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,16);
                      exit_atomic_m0_c8_s3_CMD_MJP_0_100();
                      enter_atomic_m0_c8_s7_PumpOff2();
                    }
                  }
                }
              }
            }
          }
        }
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,9);
      break;
     case IN_m0_c8_s4_LimpModeLow:
      CV_STATE_EVAL(2,0,3);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,7);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,7,0);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,5);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,5);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,5);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,11);
      if(CV_TRANSITION_EVAL(11,
        (_SFD_CCP_CALL(11,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 1)) != 0)
        || (_SFD_CCP_CALL(11,1,(
           m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 199)) != 0)) != 0) {
        if(sf_debug_transition_conflict_check_enabled()) {
          unsigned int transitionList[8];
          unsigned int numTransitions=1;
          transitionList[0] = 11;
          sf_debug_transition_conflict_check_begin();
          if((m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) < m0_c8_d20_Low_Freq)
           || (m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq
             ) > m0_c8_d19_High_Freq)) {
            transitionList[numTransitions] = 14;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5) {
            transitionList[numTransitions] = 15;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
            transitionList[numTransitions] = 12;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
            transitionList[numTransitions] = 7;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
            transitionList[numTransitions] = 3;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
            transitionList[numTransitions] = 4;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
            transitionList[numTransitions] = 16;
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
        exit_atomic_m0_c8_s4_LimpModeLow();
        enter_atomic_m0_c8_s3_CMD_MJP_0_100();
      } else {
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,14);
        if(CV_TRANSITION_EVAL(14,
          (_SFD_CCP_CALL(14,0,(m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) <
             m0_c8_d20_Low_Freq))
           != 0) ||
          (_SFD_CCP_CALL(14,1,(m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) >
             m0_c8_d19_High_Freq)) != 0)) != 0) {
          if(sf_debug_transition_conflict_check_enabled()) {
            unsigned int transitionList[7];
            unsigned int numTransitions=1;
            transitionList[0] = 14;
            sf_debug_transition_conflict_check_begin();
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5) {
              transitionList[numTransitions] = 15;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
              transitionList[numTransitions] = 12;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
              transitionList[numTransitions] = 7;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
              transitionList[numTransitions] = 3;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
              transitionList[numTransitions] = 4;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
              transitionList[numTransitions] = 16;
              numTransitions++;
            }
            sf_debug_transition_conflict_check_end();
            if(numTransitions>1) {
              _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
            }
          }
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,14);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,14);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,14);
          exit_atomic_m0_c8_s4_LimpModeLow();
          enter_atomic_m0_c8_s2_Alarm();
        } else {
          _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,15);
          if(CV_TRANSITION_EVAL(15,
            _SFD_CCP_CALL(15,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5)))
           != 0) {
            if(sf_debug_transition_conflict_check_enabled()) {
              unsigned int transitionList[6];
              unsigned int numTransitions=1;
              transitionList[0] = 15;
              sf_debug_transition_conflict_check_begin();
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
                transitionList[numTransitions] = 12;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
                transitionList[numTransitions] = 7;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                transitionList[numTransitions] = 3;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                transitionList[numTransitions] = 4;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                transitionList[numTransitions] = 16;
                numTransitions++;
              }
              sf_debug_transition_conflict_check_end();
              if(numTransitions>1) {
                _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
              }
            }
            _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,15);
            _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,15);
            _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,15);
            exit_atomic_m0_c8_s4_LimpModeLow();
            enter_atomic_m0_c8_s4_LimpModeLow();
          } else {
            _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,12);
            if(CV_TRANSITION_EVAL(12,
              _SFD_CCP_CALL(12,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9)))
             != 0) {
              if(sf_debug_transition_conflict_check_enabled()) {
                unsigned int transitionList[5];
                unsigned int numTransitions=1;
                transitionList[0] = 12;
                sf_debug_transition_conflict_check_begin();
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
                  transitionList[numTransitions] = 7;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                  transitionList[numTransitions] = 3;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                  transitionList[numTransitions] = 4;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                  transitionList[numTransitions] = 16;
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
              exit_atomic_m0_c8_s4_LimpModeLow();
              enter_atomic_m0_c8_s5_LimpModeLow1();
            } else {
              _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,7);
              if(CV_TRANSITION_EVAL(7,
                _SFD_CCP_CALL(7,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >
                  191))) != 0) {
                if(sf_debug_transition_conflict_check_enabled()) {
                  unsigned int transitionList[4];
                  unsigned int numTransitions=1;
                  transitionList[0] = 7;
                  sf_debug_transition_conflict_check_begin();
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                    transitionList[numTransitions] = 3;
                    numTransitions++;
                  }
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                    transitionList[numTransitions] = 4;
                    numTransitions++;
                  }
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                    transitionList[numTransitions] = 16;
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
                exit_atomic_m0_c8_s4_LimpModeLow();
                enter_atomic_m0_c8_s8_SaturationHigh();
              } else {
                _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,3);
                if(CV_TRANSITION_EVAL(3,
                  _SFD_CCP_CALL(3,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >=
                    23))) != 0) {
                  if(sf_debug_transition_conflict_check_enabled()) {
                    unsigned int transitionList[3];
                    unsigned int numTransitions=1;
                    transitionList[0] = 3;
                    sf_debug_transition_conflict_check_begin();
                    if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                      transitionList[numTransitions] = 4;
                      numTransitions++;
                    }
                    if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                      transitionList[numTransitions] = 16;
                      numTransitions++;
                    }
                    sf_debug_transition_conflict_check_end();
                    if(numTransitions>1) {
                      _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
                    }
                  }
                  _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,3);
                  _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,3);
                  _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,3);
                  exit_atomic_m0_c8_s4_LimpModeLow();
                  enter_atomic_m0_c8_s6_Normal3();
                } else {
                  _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,4);
                  if(CV_TRANSITION_EVAL(4,
                    _SFD_CCP_CALL(4,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >
                      19))) != 0) {
                    if(sf_debug_transition_conflict_check_enabled()) {
                      unsigned int transitionList[2];
                      unsigned int numTransitions=1;
                      transitionList[0] = 4;
                      sf_debug_transition_conflict_check_begin();
                      if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                        transitionList[numTransitions] = 16;
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
                    exit_atomic_m0_c8_s4_LimpModeLow();
                    enter_atomic_m0_c8_s9_SaturationLow();
                  } else {
                    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,16);
                    if(CV_TRANSITION_EVAL(16,
                      _SFD_CCP_CALL(16,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM)
                        <= 19))) != 0) {
                      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,16);
                      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,16);
                      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,16);
                      exit_atomic_m0_c8_s4_LimpModeLow();
                      enter_atomic_m0_c8_s7_PumpOff2();
                    }
                  }
                }
              }
            }
          }
        }
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,7);
      break;
     case IN_m0_c8_s5_LimpModeLow1:
      CV_STATE_EVAL(2,0,4);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,1);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,1,0);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,10);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,10);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,10);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,11);
      if(CV_TRANSITION_EVAL(11,
        (_SFD_CCP_CALL(11,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 1)) != 0)
        || (_SFD_CCP_CALL(11,1,(
           m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 199)) != 0)) != 0) {
        if(sf_debug_transition_conflict_check_enabled()) {
          unsigned int transitionList[8];
          unsigned int numTransitions=1;
          transitionList[0] = 11;
          sf_debug_transition_conflict_check_begin();
          if((m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) < m0_c8_d20_Low_Freq)
           || (m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq
             ) > m0_c8_d19_High_Freq)) {
            transitionList[numTransitions] = 14;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5) {
            transitionList[numTransitions] = 15;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
            transitionList[numTransitions] = 12;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
            transitionList[numTransitions] = 7;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
            transitionList[numTransitions] = 3;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
            transitionList[numTransitions] = 4;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
            transitionList[numTransitions] = 16;
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
        exit_atomic_m0_c8_s5_LimpModeLow1();
        enter_atomic_m0_c8_s3_CMD_MJP_0_100();
      } else {
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,14);
        if(CV_TRANSITION_EVAL(14,
          (_SFD_CCP_CALL(14,0,(m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) <
             m0_c8_d20_Low_Freq))
           != 0) ||
          (_SFD_CCP_CALL(14,1,(m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) >
             m0_c8_d19_High_Freq)) != 0)) != 0) {
          if(sf_debug_transition_conflict_check_enabled()) {
            unsigned int transitionList[7];
            unsigned int numTransitions=1;
            transitionList[0] = 14;
            sf_debug_transition_conflict_check_begin();
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5) {
              transitionList[numTransitions] = 15;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
              transitionList[numTransitions] = 12;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
              transitionList[numTransitions] = 7;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
              transitionList[numTransitions] = 3;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
              transitionList[numTransitions] = 4;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
              transitionList[numTransitions] = 16;
              numTransitions++;
            }
            sf_debug_transition_conflict_check_end();
            if(numTransitions>1) {
              _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
            }
          }
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,14);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,14);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,14);
          exit_atomic_m0_c8_s5_LimpModeLow1();
          enter_atomic_m0_c8_s2_Alarm();
        } else {
          _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,15);
          if(CV_TRANSITION_EVAL(15,
            _SFD_CCP_CALL(15,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5)))
           != 0) {
            if(sf_debug_transition_conflict_check_enabled()) {
              unsigned int transitionList[6];
              unsigned int numTransitions=1;
              transitionList[0] = 15;
              sf_debug_transition_conflict_check_begin();
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
                transitionList[numTransitions] = 12;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
                transitionList[numTransitions] = 7;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                transitionList[numTransitions] = 3;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                transitionList[numTransitions] = 4;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                transitionList[numTransitions] = 16;
                numTransitions++;
              }
              sf_debug_transition_conflict_check_end();
              if(numTransitions>1) {
                _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
              }
            }
            _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,15);
            _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,15);
            _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,15);
            exit_atomic_m0_c8_s5_LimpModeLow1();
            enter_atomic_m0_c8_s4_LimpModeLow();
          } else {
            _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,12);
            if(CV_TRANSITION_EVAL(12,
              _SFD_CCP_CALL(12,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9)))
             != 0) {
              if(sf_debug_transition_conflict_check_enabled()) {
                unsigned int transitionList[5];
                unsigned int numTransitions=1;
                transitionList[0] = 12;
                sf_debug_transition_conflict_check_begin();
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
                  transitionList[numTransitions] = 7;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                  transitionList[numTransitions] = 3;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                  transitionList[numTransitions] = 4;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                  transitionList[numTransitions] = 16;
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
              exit_atomic_m0_c8_s5_LimpModeLow1();
              enter_atomic_m0_c8_s5_LimpModeLow1();
            } else {
              _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,7);
              if(CV_TRANSITION_EVAL(7,
                _SFD_CCP_CALL(7,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >
                  191))) != 0) {
                if(sf_debug_transition_conflict_check_enabled()) {
                  unsigned int transitionList[4];
                  unsigned int numTransitions=1;
                  transitionList[0] = 7;
                  sf_debug_transition_conflict_check_begin();
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                    transitionList[numTransitions] = 3;
                    numTransitions++;
                  }
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                    transitionList[numTransitions] = 4;
                    numTransitions++;
                  }
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                    transitionList[numTransitions] = 16;
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
                exit_atomic_m0_c8_s5_LimpModeLow1();
                enter_atomic_m0_c8_s8_SaturationHigh();
              } else {
                _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,3);
                if(CV_TRANSITION_EVAL(3,
                  _SFD_CCP_CALL(3,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >=
                    23))) != 0) {
                  if(sf_debug_transition_conflict_check_enabled()) {
                    unsigned int transitionList[3];
                    unsigned int numTransitions=1;
                    transitionList[0] = 3;
                    sf_debug_transition_conflict_check_begin();
                    if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                      transitionList[numTransitions] = 4;
                      numTransitions++;
                    }
                    if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                      transitionList[numTransitions] = 16;
                      numTransitions++;
                    }
                    sf_debug_transition_conflict_check_end();
                    if(numTransitions>1) {
                      _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
                    }
                  }
                  _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,3);
                  _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,3);
                  _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,3);
                  exit_atomic_m0_c8_s5_LimpModeLow1();
                  enter_atomic_m0_c8_s6_Normal3();
                } else {
                  _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,4);
                  if(CV_TRANSITION_EVAL(4,
                    _SFD_CCP_CALL(4,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >
                      19))) != 0) {
                    if(sf_debug_transition_conflict_check_enabled()) {
                      unsigned int transitionList[2];
                      unsigned int numTransitions=1;
                      transitionList[0] = 4;
                      sf_debug_transition_conflict_check_begin();
                      if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                        transitionList[numTransitions] = 16;
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
                    exit_atomic_m0_c8_s5_LimpModeLow1();
                    enter_atomic_m0_c8_s9_SaturationLow();
                  } else {
                    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,16);
                    if(CV_TRANSITION_EVAL(16,
                      _SFD_CCP_CALL(16,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM)
                        <= 19))) != 0) {
                      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,16);
                      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,16);
                      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,16);
                      exit_atomic_m0_c8_s5_LimpModeLow1();
                      enter_atomic_m0_c8_s7_PumpOff2();
                    }
                  }
                }
              }
            }
          }
        }
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,1);
      break;
     case IN_m0_c8_s6_Normal3:
      CV_STATE_EVAL(2,0,5);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,4);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,4,0);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,9);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,9);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,9);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,11);
      if(CV_TRANSITION_EVAL(11,
        (_SFD_CCP_CALL(11,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 1)) != 0)
        || (_SFD_CCP_CALL(11,1,(
           m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 199)) != 0)) != 0) {
        if(sf_debug_transition_conflict_check_enabled()) {
          unsigned int transitionList[8];
          unsigned int numTransitions=1;
          transitionList[0] = 11;
          sf_debug_transition_conflict_check_begin();
          if((m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) < m0_c8_d20_Low_Freq)
           || (m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq
             ) > m0_c8_d19_High_Freq)) {
            transitionList[numTransitions] = 14;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5) {
            transitionList[numTransitions] = 15;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
            transitionList[numTransitions] = 12;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
            transitionList[numTransitions] = 7;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
            transitionList[numTransitions] = 3;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
            transitionList[numTransitions] = 4;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
            transitionList[numTransitions] = 16;
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
        exit_atomic_m0_c8_s6_Normal3();
        enter_atomic_m0_c8_s3_CMD_MJP_0_100();
      } else {
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,14);
        if(CV_TRANSITION_EVAL(14,
          (_SFD_CCP_CALL(14,0,(m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) <
             m0_c8_d20_Low_Freq))
           != 0) ||
          (_SFD_CCP_CALL(14,1,(m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) >
             m0_c8_d19_High_Freq)) != 0)) != 0) {
          if(sf_debug_transition_conflict_check_enabled()) {
            unsigned int transitionList[7];
            unsigned int numTransitions=1;
            transitionList[0] = 14;
            sf_debug_transition_conflict_check_begin();
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5) {
              transitionList[numTransitions] = 15;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
              transitionList[numTransitions] = 12;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
              transitionList[numTransitions] = 7;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
              transitionList[numTransitions] = 3;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
              transitionList[numTransitions] = 4;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
              transitionList[numTransitions] = 16;
              numTransitions++;
            }
            sf_debug_transition_conflict_check_end();
            if(numTransitions>1) {
              _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
            }
          }
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,14);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,14);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,14);
          exit_atomic_m0_c8_s6_Normal3();
          enter_atomic_m0_c8_s2_Alarm();
        } else {
          _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,15);
          if(CV_TRANSITION_EVAL(15,
            _SFD_CCP_CALL(15,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5)))
           != 0) {
            if(sf_debug_transition_conflict_check_enabled()) {
              unsigned int transitionList[6];
              unsigned int numTransitions=1;
              transitionList[0] = 15;
              sf_debug_transition_conflict_check_begin();
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
                transitionList[numTransitions] = 12;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
                transitionList[numTransitions] = 7;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                transitionList[numTransitions] = 3;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                transitionList[numTransitions] = 4;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                transitionList[numTransitions] = 16;
                numTransitions++;
              }
              sf_debug_transition_conflict_check_end();
              if(numTransitions>1) {
                _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
              }
            }
            _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,15);
            _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,15);
            _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,15);
            exit_atomic_m0_c8_s6_Normal3();
            enter_atomic_m0_c8_s4_LimpModeLow();
          } else {
            _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,12);
            if(CV_TRANSITION_EVAL(12,
              _SFD_CCP_CALL(12,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9)))
             != 0) {
              if(sf_debug_transition_conflict_check_enabled()) {
                unsigned int transitionList[5];
                unsigned int numTransitions=1;
                transitionList[0] = 12;
                sf_debug_transition_conflict_check_begin();
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
                  transitionList[numTransitions] = 7;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                  transitionList[numTransitions] = 3;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                  transitionList[numTransitions] = 4;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                  transitionList[numTransitions] = 16;
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
              exit_atomic_m0_c8_s6_Normal3();
              enter_atomic_m0_c8_s5_LimpModeLow1();
            } else {
              _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,7);
              if(CV_TRANSITION_EVAL(7,
                _SFD_CCP_CALL(7,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >
                  191))) != 0) {
                if(sf_debug_transition_conflict_check_enabled()) {
                  unsigned int transitionList[4];
                  unsigned int numTransitions=1;
                  transitionList[0] = 7;
                  sf_debug_transition_conflict_check_begin();
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                    transitionList[numTransitions] = 3;
                    numTransitions++;
                  }
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                    transitionList[numTransitions] = 4;
                    numTransitions++;
                  }
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                    transitionList[numTransitions] = 16;
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
                exit_atomic_m0_c8_s6_Normal3();
                enter_atomic_m0_c8_s8_SaturationHigh();
              } else {
                _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,3);
                if(CV_TRANSITION_EVAL(3,
                  _SFD_CCP_CALL(3,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >=
                    23))) != 0) {
                  if(sf_debug_transition_conflict_check_enabled()) {
                    unsigned int transitionList[3];
                    unsigned int numTransitions=1;
                    transitionList[0] = 3;
                    sf_debug_transition_conflict_check_begin();
                    if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                      transitionList[numTransitions] = 4;
                      numTransitions++;
                    }
                    if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                      transitionList[numTransitions] = 16;
                      numTransitions++;
                    }
                    sf_debug_transition_conflict_check_end();
                    if(numTransitions>1) {
                      _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
                    }
                  }
                  _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,3);
                  _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,3);
                  _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,3);
                  exit_atomic_m0_c8_s6_Normal3();
                  enter_atomic_m0_c8_s6_Normal3();
                } else {
                  _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,4);
                  if(CV_TRANSITION_EVAL(4,
                    _SFD_CCP_CALL(4,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >
                      19))) != 0) {
                    if(sf_debug_transition_conflict_check_enabled()) {
                      unsigned int transitionList[2];
                      unsigned int numTransitions=1;
                      transitionList[0] = 4;
                      sf_debug_transition_conflict_check_begin();
                      if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                        transitionList[numTransitions] = 16;
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
                    exit_atomic_m0_c8_s6_Normal3();
                    enter_atomic_m0_c8_s9_SaturationLow();
                  } else {
                    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,16);
                    if(CV_TRANSITION_EVAL(16,
                      _SFD_CCP_CALL(16,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM)
                        <= 19))) != 0) {
                      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,16);
                      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,16);
                      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,16);
                      exit_atomic_m0_c8_s6_Normal3();
                      enter_atomic_m0_c8_s7_PumpOff2();
                    }
                  }
                }
              }
            }
          }
        }
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,4);
      break;
     case IN_m0_c8_s7_PumpOff2:
      CV_STATE_EVAL(2,0,6);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,6);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,6,0);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,13);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,13);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,13);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,11);
      if(CV_TRANSITION_EVAL(11,
        (_SFD_CCP_CALL(11,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 1)) != 0)
        || (_SFD_CCP_CALL(11,1,(
           m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 199)) != 0)) != 0) {
        if(sf_debug_transition_conflict_check_enabled()) {
          unsigned int transitionList[8];
          unsigned int numTransitions=1;
          transitionList[0] = 11;
          sf_debug_transition_conflict_check_begin();
          if((m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) < m0_c8_d20_Low_Freq)
           || (m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq
             ) > m0_c8_d19_High_Freq)) {
            transitionList[numTransitions] = 14;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5) {
            transitionList[numTransitions] = 15;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
            transitionList[numTransitions] = 12;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
            transitionList[numTransitions] = 7;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
            transitionList[numTransitions] = 3;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
            transitionList[numTransitions] = 4;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
            transitionList[numTransitions] = 16;
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
        exit_atomic_m0_c8_s7_PumpOff2();
        enter_atomic_m0_c8_s3_CMD_MJP_0_100();
      } else {
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,14);
        if(CV_TRANSITION_EVAL(14,
          (_SFD_CCP_CALL(14,0,(m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) <
             m0_c8_d20_Low_Freq))
           != 0) ||
          (_SFD_CCP_CALL(14,1,(m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) >
             m0_c8_d19_High_Freq)) != 0)) != 0) {
          if(sf_debug_transition_conflict_check_enabled()) {
            unsigned int transitionList[7];
            unsigned int numTransitions=1;
            transitionList[0] = 14;
            sf_debug_transition_conflict_check_begin();
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5) {
              transitionList[numTransitions] = 15;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
              transitionList[numTransitions] = 12;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
              transitionList[numTransitions] = 7;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
              transitionList[numTransitions] = 3;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
              transitionList[numTransitions] = 4;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
              transitionList[numTransitions] = 16;
              numTransitions++;
            }
            sf_debug_transition_conflict_check_end();
            if(numTransitions>1) {
              _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
            }
          }
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,14);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,14);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,14);
          exit_atomic_m0_c8_s7_PumpOff2();
          enter_atomic_m0_c8_s2_Alarm();
        } else {
          _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,15);
          if(CV_TRANSITION_EVAL(15,
            _SFD_CCP_CALL(15,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5)))
           != 0) {
            if(sf_debug_transition_conflict_check_enabled()) {
              unsigned int transitionList[6];
              unsigned int numTransitions=1;
              transitionList[0] = 15;
              sf_debug_transition_conflict_check_begin();
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
                transitionList[numTransitions] = 12;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
                transitionList[numTransitions] = 7;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                transitionList[numTransitions] = 3;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                transitionList[numTransitions] = 4;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                transitionList[numTransitions] = 16;
                numTransitions++;
              }
              sf_debug_transition_conflict_check_end();
              if(numTransitions>1) {
                _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
              }
            }
            _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,15);
            _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,15);
            _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,15);
            exit_atomic_m0_c8_s7_PumpOff2();
            enter_atomic_m0_c8_s4_LimpModeLow();
          } else {
            _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,12);
            if(CV_TRANSITION_EVAL(12,
              _SFD_CCP_CALL(12,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9)))
             != 0) {
              if(sf_debug_transition_conflict_check_enabled()) {
                unsigned int transitionList[5];
                unsigned int numTransitions=1;
                transitionList[0] = 12;
                sf_debug_transition_conflict_check_begin();
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
                  transitionList[numTransitions] = 7;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                  transitionList[numTransitions] = 3;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                  transitionList[numTransitions] = 4;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                  transitionList[numTransitions] = 16;
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
              exit_atomic_m0_c8_s7_PumpOff2();
              enter_atomic_m0_c8_s5_LimpModeLow1();
            } else {
              _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,7);
              if(CV_TRANSITION_EVAL(7,
                _SFD_CCP_CALL(7,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >
                  191))) != 0) {
                if(sf_debug_transition_conflict_check_enabled()) {
                  unsigned int transitionList[4];
                  unsigned int numTransitions=1;
                  transitionList[0] = 7;
                  sf_debug_transition_conflict_check_begin();
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                    transitionList[numTransitions] = 3;
                    numTransitions++;
                  }
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                    transitionList[numTransitions] = 4;
                    numTransitions++;
                  }
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                    transitionList[numTransitions] = 16;
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
                exit_atomic_m0_c8_s7_PumpOff2();
                enter_atomic_m0_c8_s8_SaturationHigh();
              } else {
                _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,3);
                if(CV_TRANSITION_EVAL(3,
                  _SFD_CCP_CALL(3,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >=
                    23))) != 0) {
                  if(sf_debug_transition_conflict_check_enabled()) {
                    unsigned int transitionList[3];
                    unsigned int numTransitions=1;
                    transitionList[0] = 3;
                    sf_debug_transition_conflict_check_begin();
                    if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                      transitionList[numTransitions] = 4;
                      numTransitions++;
                    }
                    if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                      transitionList[numTransitions] = 16;
                      numTransitions++;
                    }
                    sf_debug_transition_conflict_check_end();
                    if(numTransitions>1) {
                      _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
                    }
                  }
                  _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,3);
                  _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,3);
                  _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,3);
                  exit_atomic_m0_c8_s7_PumpOff2();
                  enter_atomic_m0_c8_s6_Normal3();
                } else {
                  _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,4);
                  if(CV_TRANSITION_EVAL(4,
                    _SFD_CCP_CALL(4,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >
                      19))) != 0) {
                    if(sf_debug_transition_conflict_check_enabled()) {
                      unsigned int transitionList[2];
                      unsigned int numTransitions=1;
                      transitionList[0] = 4;
                      sf_debug_transition_conflict_check_begin();
                      if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                        transitionList[numTransitions] = 16;
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
                    exit_atomic_m0_c8_s7_PumpOff2();
                    enter_atomic_m0_c8_s9_SaturationLow();
                  } else {
                    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,16);
                    if(CV_TRANSITION_EVAL(16,
                      _SFD_CCP_CALL(16,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM)
                        <= 19))) != 0) {
                      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,16);
                      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,16);
                      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,16);
                      exit_atomic_m0_c8_s7_PumpOff2();
                      enter_atomic_m0_c8_s7_PumpOff2();
                    }
                  }
                }
              }
            }
          }
        }
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,6);
      break;
     case IN_m0_c8_s8_SaturationHigh:
      CV_STATE_EVAL(2,0,7);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,3);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,3,0);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,8);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,8);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,8);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,11);
      if(CV_TRANSITION_EVAL(11,
        (_SFD_CCP_CALL(11,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 1)) != 0)
        || (_SFD_CCP_CALL(11,1,(
           m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 199)) != 0)) != 0) {
        if(sf_debug_transition_conflict_check_enabled()) {
          unsigned int transitionList[8];
          unsigned int numTransitions=1;
          transitionList[0] = 11;
          sf_debug_transition_conflict_check_begin();
          if((m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) < m0_c8_d20_Low_Freq)
           || (m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq
             ) > m0_c8_d19_High_Freq)) {
            transitionList[numTransitions] = 14;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5) {
            transitionList[numTransitions] = 15;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
            transitionList[numTransitions] = 12;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
            transitionList[numTransitions] = 7;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
            transitionList[numTransitions] = 3;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
            transitionList[numTransitions] = 4;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
            transitionList[numTransitions] = 16;
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
        exit_atomic_m0_c8_s8_SaturationHigh();
        enter_atomic_m0_c8_s3_CMD_MJP_0_100();
      } else {
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,14);
        if(CV_TRANSITION_EVAL(14,
          (_SFD_CCP_CALL(14,0,(m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) <
             m0_c8_d20_Low_Freq))
           != 0) ||
          (_SFD_CCP_CALL(14,1,(m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) >
             m0_c8_d19_High_Freq)) != 0)) != 0) {
          if(sf_debug_transition_conflict_check_enabled()) {
            unsigned int transitionList[7];
            unsigned int numTransitions=1;
            transitionList[0] = 14;
            sf_debug_transition_conflict_check_begin();
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5) {
              transitionList[numTransitions] = 15;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
              transitionList[numTransitions] = 12;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
              transitionList[numTransitions] = 7;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
              transitionList[numTransitions] = 3;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
              transitionList[numTransitions] = 4;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
              transitionList[numTransitions] = 16;
              numTransitions++;
            }
            sf_debug_transition_conflict_check_end();
            if(numTransitions>1) {
              _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
            }
          }
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,14);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,14);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,14);
          exit_atomic_m0_c8_s8_SaturationHigh();
          enter_atomic_m0_c8_s2_Alarm();
        } else {
          _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,15);
          if(CV_TRANSITION_EVAL(15,
            _SFD_CCP_CALL(15,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5)))
           != 0) {
            if(sf_debug_transition_conflict_check_enabled()) {
              unsigned int transitionList[6];
              unsigned int numTransitions=1;
              transitionList[0] = 15;
              sf_debug_transition_conflict_check_begin();
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
                transitionList[numTransitions] = 12;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
                transitionList[numTransitions] = 7;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                transitionList[numTransitions] = 3;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                transitionList[numTransitions] = 4;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                transitionList[numTransitions] = 16;
                numTransitions++;
              }
              sf_debug_transition_conflict_check_end();
              if(numTransitions>1) {
                _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
              }
            }
            _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,15);
            _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,15);
            _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,15);
            exit_atomic_m0_c8_s8_SaturationHigh();
            enter_atomic_m0_c8_s4_LimpModeLow();
          } else {
            _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,12);
            if(CV_TRANSITION_EVAL(12,
              _SFD_CCP_CALL(12,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9)))
             != 0) {
              if(sf_debug_transition_conflict_check_enabled()) {
                unsigned int transitionList[5];
                unsigned int numTransitions=1;
                transitionList[0] = 12;
                sf_debug_transition_conflict_check_begin();
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
                  transitionList[numTransitions] = 7;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                  transitionList[numTransitions] = 3;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                  transitionList[numTransitions] = 4;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                  transitionList[numTransitions] = 16;
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
              exit_atomic_m0_c8_s8_SaturationHigh();
              enter_atomic_m0_c8_s5_LimpModeLow1();
            } else {
              _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,7);
              if(CV_TRANSITION_EVAL(7,
                _SFD_CCP_CALL(7,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >
                  191))) != 0) {
                if(sf_debug_transition_conflict_check_enabled()) {
                  unsigned int transitionList[4];
                  unsigned int numTransitions=1;
                  transitionList[0] = 7;
                  sf_debug_transition_conflict_check_begin();
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                    transitionList[numTransitions] = 3;
                    numTransitions++;
                  }
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                    transitionList[numTransitions] = 4;
                    numTransitions++;
                  }
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                    transitionList[numTransitions] = 16;
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
                exit_atomic_m0_c8_s8_SaturationHigh();
                enter_atomic_m0_c8_s8_SaturationHigh();
              } else {
                _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,3);
                if(CV_TRANSITION_EVAL(3,
                  _SFD_CCP_CALL(3,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >=
                    23))) != 0) {
                  if(sf_debug_transition_conflict_check_enabled()) {
                    unsigned int transitionList[3];
                    unsigned int numTransitions=1;
                    transitionList[0] = 3;
                    sf_debug_transition_conflict_check_begin();
                    if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                      transitionList[numTransitions] = 4;
                      numTransitions++;
                    }
                    if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                      transitionList[numTransitions] = 16;
                      numTransitions++;
                    }
                    sf_debug_transition_conflict_check_end();
                    if(numTransitions>1) {
                      _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
                    }
                  }
                  _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,3);
                  _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,3);
                  _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,3);
                  exit_atomic_m0_c8_s8_SaturationHigh();
                  enter_atomic_m0_c8_s6_Normal3();
                } else {
                  _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,4);
                  if(CV_TRANSITION_EVAL(4,
                    _SFD_CCP_CALL(4,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >
                      19))) != 0) {
                    if(sf_debug_transition_conflict_check_enabled()) {
                      unsigned int transitionList[2];
                      unsigned int numTransitions=1;
                      transitionList[0] = 4;
                      sf_debug_transition_conflict_check_begin();
                      if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                        transitionList[numTransitions] = 16;
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
                    exit_atomic_m0_c8_s8_SaturationHigh();
                    enter_atomic_m0_c8_s9_SaturationLow();
                  } else {
                    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,16);
                    if(CV_TRANSITION_EVAL(16,
                      _SFD_CCP_CALL(16,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM)
                        <= 19))) != 0) {
                      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,16);
                      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,16);
                      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,16);
                      exit_atomic_m0_c8_s8_SaturationHigh();
                      enter_atomic_m0_c8_s7_PumpOff2();
                    }
                  }
                }
              }
            }
          }
        }
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
      break;
     case IN_m0_c8_s9_SaturationLow:
      CV_STATE_EVAL(2,0,8);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,0);
      _SFD_CCS_CALL(STATE_DURING_COVERAGE_TAG,0,0);
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,1);
      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,1);
      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,1);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,11);
      if(CV_TRANSITION_EVAL(11,
        (_SFD_CCP_CALL(11,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 1)) != 0)
        || (_SFD_CCP_CALL(11,1,(
           m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 199)) != 0)) != 0) {
        if(sf_debug_transition_conflict_check_enabled()) {
          unsigned int transitionList[8];
          unsigned int numTransitions=1;
          transitionList[0] = 11;
          sf_debug_transition_conflict_check_begin();
          if((m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) < m0_c8_d20_Low_Freq)
           || (m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq
             ) > m0_c8_d19_High_Freq)) {
            transitionList[numTransitions] = 14;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5) {
            transitionList[numTransitions] = 15;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
            transitionList[numTransitions] = 12;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
            transitionList[numTransitions] = 7;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
            transitionList[numTransitions] = 3;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
            transitionList[numTransitions] = 4;
            numTransitions++;
          }
          if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
            transitionList[numTransitions] = 16;
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
        exit_atomic_m0_c8_s9_SaturationLow();
        enter_atomic_m0_c8_s3_CMD_MJP_0_100();
      } else {
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,14);
        if(CV_TRANSITION_EVAL(14,
          (_SFD_CCP_CALL(14,0,(m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) <
             m0_c8_d20_Low_Freq))
           != 0) ||
          (_SFD_CCP_CALL(14,1,(m0_c8_u16_d_(InputData_m0_c8_d10_iPWM_Freq) >
             m0_c8_d19_High_Freq)) != 0)) != 0) {
          if(sf_debug_transition_conflict_check_enabled()) {
            unsigned int transitionList[7];
            unsigned int numTransitions=1;
            transitionList[0] = 14;
            sf_debug_transition_conflict_check_begin();
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5) {
              transitionList[numTransitions] = 15;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
              transitionList[numTransitions] = 12;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
              transitionList[numTransitions] = 7;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
              transitionList[numTransitions] = 3;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
              transitionList[numTransitions] = 4;
              numTransitions++;
            }
            if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
              transitionList[numTransitions] = 16;
              numTransitions++;
            }
            sf_debug_transition_conflict_check_end();
            if(numTransitions>1) {
              _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
            }
          }
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,14);
          _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,14);
          _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,14);
          exit_atomic_m0_c8_s9_SaturationLow();
          enter_atomic_m0_c8_s2_Alarm();
        } else {
          _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,15);
          if(CV_TRANSITION_EVAL(15,
            _SFD_CCP_CALL(15,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 5)))
           != 0) {
            if(sf_debug_transition_conflict_check_enabled()) {
              unsigned int transitionList[6];
              unsigned int numTransitions=1;
              transitionList[0] = 15;
              sf_debug_transition_conflict_check_begin();
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9) {
                transitionList[numTransitions] = 12;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
                transitionList[numTransitions] = 7;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                transitionList[numTransitions] = 3;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                transitionList[numTransitions] = 4;
                numTransitions++;
              }
              if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                transitionList[numTransitions] = 16;
                numTransitions++;
              }
              sf_debug_transition_conflict_check_end();
              if(numTransitions>1) {
                _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
              }
            }
            _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,15);
            _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,15);
            _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,15);
            exit_atomic_m0_c8_s9_SaturationLow();
            enter_atomic_m0_c8_s4_LimpModeLow();
          } else {
            _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,12);
            if(CV_TRANSITION_EVAL(12,
              _SFD_CCP_CALL(12,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) < 9)))
             != 0) {
              if(sf_debug_transition_conflict_check_enabled()) {
                unsigned int transitionList[5];
                unsigned int numTransitions=1;
                transitionList[0] = 12;
                sf_debug_transition_conflict_check_begin();
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 191) {
                  transitionList[numTransitions] = 7;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                  transitionList[numTransitions] = 3;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                  transitionList[numTransitions] = 4;
                  numTransitions++;
                }
                if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                  transitionList[numTransitions] = 16;
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
              exit_atomic_m0_c8_s9_SaturationLow();
              enter_atomic_m0_c8_s5_LimpModeLow1();
            } else {
              _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,7);
              if(CV_TRANSITION_EVAL(7,
                _SFD_CCP_CALL(7,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >
                  191))) != 0) {
                if(sf_debug_transition_conflict_check_enabled()) {
                  unsigned int transitionList[4];
                  unsigned int numTransitions=1;
                  transitionList[0] = 7;
                  sf_debug_transition_conflict_check_begin();
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >= 23) {
                    transitionList[numTransitions] = 3;
                    numTransitions++;
                  }
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                    transitionList[numTransitions] = 4;
                    numTransitions++;
                  }
                  if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                    transitionList[numTransitions] = 16;
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
                exit_atomic_m0_c8_s9_SaturationLow();
                enter_atomic_m0_c8_s8_SaturationHigh();
              } else {
                _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,3);
                if(CV_TRANSITION_EVAL(3,
                  _SFD_CCP_CALL(3,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >=
                    23))) != 0) {
                  if(sf_debug_transition_conflict_check_enabled()) {
                    unsigned int transitionList[3];
                    unsigned int numTransitions=1;
                    transitionList[0] = 3;
                    sf_debug_transition_conflict_check_begin();
                    if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) > 19) {
                      transitionList[numTransitions] = 4;
                      numTransitions++;
                    }
                    if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                      transitionList[numTransitions] = 16;
                      numTransitions++;
                    }
                    sf_debug_transition_conflict_check_end();
                    if(numTransitions>1) {
                      _SFD_TRANSITION_CONFLICT(&(transitionList[0]),numTransitions);
                    }
                  }
                  _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,3);
                  _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,3);
                  _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,3);
                  exit_atomic_m0_c8_s9_SaturationLow();
                  enter_atomic_m0_c8_s6_Normal3();
                } else {
                  _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,4);
                  if(CV_TRANSITION_EVAL(4,
                    _SFD_CCP_CALL(4,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) >
                      19))) != 0) {
                    if(sf_debug_transition_conflict_check_enabled()) {
                      unsigned int transitionList[2];
                      unsigned int numTransitions=1;
                      transitionList[0] = 4;
                      sf_debug_transition_conflict_check_begin();
                      if(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM) <= 19) {
                        transitionList[numTransitions] = 16;
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
                    exit_atomic_m0_c8_s9_SaturationLow();
                    enter_atomic_m0_c8_s9_SaturationLow();
                  } else {
                    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG,16);
                    if(CV_TRANSITION_EVAL(16,
                      _SFD_CCP_CALL(16,0,(m0_c8_u8_d_(InputData_m0_c8_d11_idPWM)
                        <= 19))) != 0) {
                      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,16);
                      _SFD_CT_CALL(TRANSITION_WHEN_VALID_TAG,16);
                      _SFD_CT_CALL(TRANSITION_INACTIVE_TAG,16);
                      exit_atomic_m0_c8_s9_SaturationLow();
                      enter_atomic_m0_c8_s7_PumpOff2();
                    }
                  }
                }
              }
            }
          }
        }
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
      break;
     default:
      CV_STATE_EVAL(2,0,0);
      break;
    }
    _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,2);
  }
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,7);
}

static void enter_atomic_m0_c8_s2_Alarm(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,8);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,8,0);
  chartInstance.State.is_m0_c8_s1_PWMinput_handling = IN_m0_c8_s2_Alarm;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,8);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,8);
  OutputData_m0_c8_d14_odPumpOff = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,8,1);
  OutputData_m0_c8_d13_odFixedValueSel = 1.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,8,2);
  OutputData_m0_c8_d18_odFixedLowValueSel = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,8,3);
  OutputData_m0_c8_d15_odPI_Disable = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,8,4);
  OutputData_m0_c8_d16_oPWM_SC_Alarm = 1.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,8,5);
  OutputData_m0_c8_d17_oPWM_Alarm = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,8,6);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,8);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,8);
}

static void exit_atomic_m0_c8_s2_Alarm(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,8);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,8,0);
  chartInstance.State.is_m0_c8_s1_PWMinput_handling = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,8);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,8);
}

static void enter_atomic_m0_c8_s3_CMD_MJP_0_100(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,9);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,9,0);
  chartInstance.State.is_m0_c8_s1_PWMinput_handling = IN_m0_c8_s3_CMD_MJP_0_100;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,9);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,9);
  OutputData_m0_c8_d14_odPumpOff = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,9,1);
  OutputData_m0_c8_d13_odFixedValueSel = 1.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,9,2);
  OutputData_m0_c8_d18_odFixedLowValueSel = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,9,3);
  OutputData_m0_c8_d15_odPI_Disable = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,9,4);
  OutputData_m0_c8_d16_oPWM_SC_Alarm = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,9,5);
  OutputData_m0_c8_d17_oPWM_Alarm = 1.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,9,6);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,9);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,9);
}

static void exit_atomic_m0_c8_s3_CMD_MJP_0_100(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,9);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,9,0);
  chartInstance.State.is_m0_c8_s1_PWMinput_handling = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,9);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,9);
}

static void enter_atomic_m0_c8_s4_LimpModeLow(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,7);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,7,0);
  chartInstance.State.is_m0_c8_s1_PWMinput_handling = IN_m0_c8_s4_LimpModeLow;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,7);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,7);
  OutputData_m0_c8_d14_odPumpOff = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,7,1);
  OutputData_m0_c8_d13_odFixedValueSel = 1.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,7,2);
  OutputData_m0_c8_d18_odFixedLowValueSel = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,7,3);
  OutputData_m0_c8_d15_odPI_Disable = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,7,4);
  OutputData_m0_c8_d16_oPWM_SC_Alarm = 1.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,7,5);
  OutputData_m0_c8_d17_oPWM_Alarm = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,7,6);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,7);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,7);
}

static void exit_atomic_m0_c8_s4_LimpModeLow(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,7);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,7,0);
  chartInstance.State.is_m0_c8_s1_PWMinput_handling = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,7);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,7);
}

static void enter_atomic_m0_c8_s5_LimpModeLow1(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,1);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,0);
  chartInstance.State.is_m0_c8_s1_PWMinput_handling = IN_m0_c8_s5_LimpModeLow1;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,1);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,1);
  OutputData_m0_c8_d14_odPumpOff = 1.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,1);
  OutputData_m0_c8_d13_odFixedValueSel = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,2);
  OutputData_m0_c8_d18_odFixedLowValueSel = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,3);
  OutputData_m0_c8_d15_odPI_Disable = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,4);
  OutputData_m0_c8_d16_oPWM_SC_Alarm = 1.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,5);
  OutputData_m0_c8_d17_oPWM_Alarm = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,1,6);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,1);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,1);
}

static void exit_atomic_m0_c8_s5_LimpModeLow1(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,1);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,1,0);
  chartInstance.State.is_m0_c8_s1_PWMinput_handling = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,1);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,1);
}

static void enter_atomic_m0_c8_s6_Normal3(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,4);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,4,0);
  chartInstance.State.is_m0_c8_s1_PWMinput_handling = IN_m0_c8_s6_Normal3;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,4);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,4);
  OutputData_m0_c8_d14_odPumpOff = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,4,1);
  OutputData_m0_c8_d18_odFixedLowValueSel = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,4,2);
  OutputData_m0_c8_d13_odFixedValueSel = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,4,3);
  OutputData_m0_c8_d15_odPI_Disable = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,4,4);
  OutputData_m0_c8_d16_oPWM_SC_Alarm = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,4,5);
  OutputData_m0_c8_d17_oPWM_Alarm = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,4,6);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,4);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,4);
}

static void exit_atomic_m0_c8_s6_Normal3(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,4);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,4,0);
  chartInstance.State.is_m0_c8_s1_PWMinput_handling = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,4);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,4);
}

static void enter_atomic_m0_c8_s7_PumpOff2(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,6);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,6,0);
  chartInstance.State.is_m0_c8_s1_PWMinput_handling = IN_m0_c8_s7_PumpOff2;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,6);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,6);
  OutputData_m0_c8_d14_odPumpOff = 1.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,6,1);
  OutputData_m0_c8_d13_odFixedValueSel = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,6,2);
  OutputData_m0_c8_d18_odFixedLowValueSel = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,6,3);
  OutputData_m0_c8_d15_odPI_Disable = 1.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,6,4);
  OutputData_m0_c8_d16_oPWM_SC_Alarm = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,6,5);
  OutputData_m0_c8_d17_oPWM_Alarm = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,6,6);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,6);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,6);
}

static void exit_atomic_m0_c8_s7_PumpOff2(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,6);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,6,0);
  chartInstance.State.is_m0_c8_s1_PWMinput_handling = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,6);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,6);
}

static void enter_atomic_m0_c8_s8_SaturationHigh(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,3);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,3,0);
  chartInstance.State.is_m0_c8_s1_PWMinput_handling = IN_m0_c8_s8_SaturationHigh;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,3);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,3);
  OutputData_m0_c8_d14_odPumpOff = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,3,1);
  OutputData_m0_c8_d13_odFixedValueSel = 1.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,3,2);
  OutputData_m0_c8_d18_odFixedLowValueSel = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,3,3);
  OutputData_m0_c8_d15_odPI_Disable = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,3,4);
  OutputData_m0_c8_d16_oPWM_SC_Alarm = 1.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,3,5);
  OutputData_m0_c8_d17_oPWM_Alarm = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,3,6);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,3);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
}

static void exit_atomic_m0_c8_s8_SaturationHigh(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,3);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,3,0);
  chartInstance.State.is_m0_c8_s1_PWMinput_handling = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,3);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
}

static void enter_atomic_m0_c8_s9_SaturationLow(void)
{
  _SFD_CS_CALL(STATE_ENTER_ENTRY_FUNCTION_TAG,0);
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,0,0);
  chartInstance.State.is_m0_c8_s1_PWMinput_handling = IN_m0_c8_s9_SaturationLow;
  _SFD_CS_CALL(STATE_ACTIVE_TAG,0);
  _SFD_CS_CALL(STATE_BEFORE_ENTRY_ACTION_TAG,0);
  OutputData_m0_c8_d14_odPumpOff = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,0,1);
  OutputData_m0_c8_d13_odFixedValueSel = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,0,2);
  OutputData_m0_c8_d18_odFixedLowValueSel = 1.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,0,3);
  OutputData_m0_c8_d15_odPI_Disable = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,0,4);
  OutputData_m0_c8_d16_oPWM_SC_Alarm = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,0,5);
  OutputData_m0_c8_d17_oPWM_Alarm = 0.0;
  _SFD_CCS_CALL(STATE_ENTRY_COVERAGE_TAG,0,6);
  _SFD_CS_CALL(STATE_AFTER_ENTRY_ACTION_TAG,0);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
}

static void exit_atomic_m0_c8_s9_SaturationLow(void)
{
  _SFD_CS_CALL(STATE_ENTER_EXIT_FUNCTION_TAG,0);
  _SFD_CCS_CALL(STATE_EXIT_COVERAGE_TAG,0,0);
  chartInstance.State.is_m0_c8_s1_PWMinput_handling = IN_NO_ACTIVE_CHILD;
  _SFD_CS_CALL(STATE_INACTIVE_TAG,0);
  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
}

void sf_BVH2_App_Layer_sfun_c8_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2345472014U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1450511866U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1289644125U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3347275170U);
}
/*
 * Chart initialization function
 */
/* work around the buggy macro in simstruc.h until it is fixed */
#define cdrGetOutputPortReusable(S,port) \
  ( (S)->portInfo.outputs[(port)].attributes.optimOpts != \
   SS_NOT_REUSABLE_AND_GLOBAL )

static void initialize_BVH2_App_Layer_sfun_c8( SimStruct *S)
{

  {
    {
      uint8_T __sfTemp_i0;
      for(__sfTemp_i0 = 0; __sfTemp_i0 < 8; __sfTemp_i0++) {
        chartInstance.LocalData.m0_c8_d2_DTCs[__sfTemp_i0] = 0;
      }
    }
    chartInstance.LocalData.m0_c8_d9_index = 0;
    chartInstance.LocalData.m0_c8_d1_DTC = 0;
    chartInstance.LocalData.m0_c8_d3_NDTC = 8;
    chartInstance.LocalData.m0_c8_d4_NoDTC = 1;
    chartInstance.LocalData.m0_c8_d7_TempDTC = 0;
    chartInstance.LocalData.m0_c8_d8_count = 0;
    chartInstance.LocalData.m0_c8_d6_SavedDTC = 0;
    chartInstance.LocalData.m0_c8_d5_ST7_Ready = 0;
    m0_c8_d21_WAITTIME = 25;
    m0_c8_d20_Low_Freq = 36000;
    m0_c8_d19_High_Freq = 44000;
    if(!cdrGetOutputPortReusable(S,1)) {
      OutputData_m0_c8_d13_odFixedValueSel = 0.0;
    }
    if(!cdrGetOutputPortReusable(S,2)) {
      OutputData_m0_c8_d14_odPumpOff = 0.0;
    }
    if(!cdrGetOutputPortReusable(S,3)) {
      OutputData_m0_c8_d15_odPI_Disable = 0.0;
    }
    if(!cdrGetOutputPortReusable(S,4)) {
      OutputData_m0_c8_d16_oPWM_SC_Alarm = 0.0;
    }
    if(!cdrGetOutputPortReusable(S,5)) {
      OutputData_m0_c8_d17_oPWM_Alarm = 0.0;
    }
    if(!cdrGetOutputPortReusable(S,6)) {
      OutputData_m0_c8_d18_odFixedLowValueSel = 0.0;
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
            8,
            10,
            17,
            21,
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
            sf_debug_set_chart_disable_implicit_casting(_BVH2_App_LayerMachineNumber_,chartInstance.chartNumber,0);
            sf_debug_set_chart_event_thresholds(_BVH2_App_LayerMachineNumber_,
             chartInstance.chartNumber,
             0,
             0,
             0);

            _SFD_SET_DATA_PROPS(16,
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
            _SFD_SET_DATA_PROPS(8,
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
            {
              unsigned int dimVector[1];
              dimVector[0]= 8;
              _SFD_SET_DATA_PROPS(0,
               0,
               0,
               0,
               SF_UINT8,
               1,
               &(dimVector[0]),
               0,
               0.0,
               1.0,
               0);
            }
            _SFD_SET_DATA_PROPS(19,
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
            _SFD_SET_DATA_PROPS(17,
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
            _SFD_SET_DATA_PROPS(4,
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
            _SFD_SET_DATA_PROPS(5,
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
            _SFD_SET_DATA_PROPS(15,
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
            _SFD_SET_DATA_PROPS(7,
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
            _SFD_SET_DATA_PROPS(9,
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
            _SFD_SET_DATA_PROPS(2,
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
            _SFD_SET_DATA_PROPS(1,
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
            _SFD_SET_DATA_PROPS(6,
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
            _SFD_SET_DATA_PROPS(10,
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
            _SFD_SET_DATA_PROPS(18,
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
            _SFD_SET_DATA_PROPS(14,
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
            _SFD_SET_DATA_PROPS(20,
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
            _SFD_SET_DATA_PROPS(13,
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
            _SFD_STATE_INFO(2,0,1);
            _SFD_STATE_INFO(8,0,0);
            _SFD_STATE_INFO(9,0,0);
            _SFD_STATE_INFO(7,0,0);
            _SFD_STATE_INFO(1,0,0);
            _SFD_STATE_INFO(4,0,0);
            _SFD_STATE_INFO(6,0,0);
            _SFD_STATE_INFO(3,0,0);
            _SFD_STATE_INFO(0,0,0);
            _SFD_CH_SUBSTATE_COUNT(1);
            _SFD_CH_SUBSTATE_DECOMP(1);
            _SFD_CH_SUBSTATE_INDEX(0,2);
            _SFD_ST_SUBSTATE_COUNT(2,8);
            _SFD_ST_SUBSTATE_INDEX(2,0,8);
            _SFD_ST_SUBSTATE_INDEX(2,1,9);
            _SFD_ST_SUBSTATE_INDEX(2,2,7);
            _SFD_ST_SUBSTATE_INDEX(2,3,1);
            _SFD_ST_SUBSTATE_INDEX(2,4,4);
            _SFD_ST_SUBSTATE_INDEX(2,5,6);
            _SFD_ST_SUBSTATE_INDEX(2,6,3);
            _SFD_ST_SUBSTATE_INDEX(2,7,0);
            _SFD_ST_SUBSTATE_COUNT(8,0);
            _SFD_ST_SUBSTATE_COUNT(9,0);
            _SFD_ST_SUBSTATE_COUNT(7,0);
            _SFD_ST_SUBSTATE_COUNT(1,0);
            _SFD_ST_SUBSTATE_COUNT(4,0);
            _SFD_ST_SUBSTATE_COUNT(6,0);
            _SFD_ST_SUBSTATE_COUNT(3,0);
            _SFD_ST_SUBSTATE_COUNT(0,0);
          }
          _SFD_CV_INIT_CHART(1,0,0,0);
          {
            _SFD_CV_INIT_STATE(2,8,1,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(8,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(9,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(7,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(1,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(4,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(6,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(3,0,0,0,0,0,NULL,NULL);
          }
          {
            _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
          }

          _SFD_CV_INIT_TRANS(9,0,NULL,NULL,0,NULL);

          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {10};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(2,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {2};
            static unsigned int sEndGuardMap[] = {11};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(3,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {2};
            static unsigned int sEndGuardMap[] = {11};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(16,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          _SFD_CV_INIT_TRANS(13,0,NULL,NULL,0,NULL);

          _SFD_CV_INIT_TRANS(5,0,NULL,NULL,0,NULL);

          {
            static unsigned int sStartGuardMap[] = {3};
            static unsigned int sEndGuardMap[] = {13};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(15,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {4};
            static unsigned int sEndGuardMap[] = {15};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(7,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          _SFD_CV_INIT_TRANS(8,0,NULL,NULL,0,NULL);

          {
            static unsigned int sStartGuardMap[] = {2};
            static unsigned int sEndGuardMap[] = {10};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(4,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          _SFD_CV_INIT_TRANS(1,0,NULL,NULL,0,NULL);

          {
            static unsigned int sStartGuardMap[] = {4,32};
            static unsigned int sEndGuardMap[] = {24,53};
            static int sPostFixPredicateTree[] = {0,1,-2};
            _SFD_CV_INIT_TRANS(14,2,&(sStartGuardMap[0]),&(sEndGuardMap[0]),3,&(sPostFixPredicateTree[0]));
          }
          _SFD_CV_INIT_TRANS(6,0,NULL,NULL,0,NULL);

          _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

          {
            static unsigned int sStartGuardMap[] = {3,18};
            static unsigned int sEndGuardMap[] = {12,29};
            static int sPostFixPredicateTree[] = {0,1,-2};
            _SFD_CV_INIT_TRANS(11,2,&(sStartGuardMap[0]),&(sEndGuardMap[0]),3,&(sPostFixPredicateTree[0]));
          }
          {
            static unsigned int sStartGuardMap[] = {3};
            static unsigned int sEndGuardMap[] = {12};
            static int sPostFixPredicateTree[] = {0};
            _SFD_CV_INIT_TRANS(12,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
          }
          _SFD_CV_INIT_TRANS(10,0,NULL,NULL,0,NULL);

          _SFD_STATE_COV_WTS(2,1,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0};
            static unsigned int sEndEntryMap[] = {0};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(2,
             1,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(8,7,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,13,29,51,76,95,115};
            static unsigned int sEndEntryMap[] = {0,28,50,75,94,114,131};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(8,
             7,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(9,7,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,21,37,59,84,103,123};
            static unsigned int sEndEntryMap[] = {0,36,58,83,102,122,139};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(9,
             7,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(7,7,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,19,35,57,82,101,121};
            static unsigned int sEndEntryMap[] = {0,34,56,81,100,120,137};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(7,
             7,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(1,7,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,20,36,58,83,102,122};
            static unsigned int sEndEntryMap[] = {0,35,57,82,101,121,138};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(1,
             7,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(4,7,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,15,36,61,83,104,124};
            static unsigned int sEndEntryMap[] = {0,35,60,82,103,123,142};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(4,
             7,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(6,7,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,16,41,64,89,114,134};
            static unsigned int sEndEntryMap[] = {0,40,63,88,113,133,152};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(6,
             7,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(3,7,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,22,38,60,85,104,124};
            static unsigned int sEndEntryMap[] = {0,37,59,84,103,123,140};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(3,
             7,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_STATE_COV_WTS(0,7,1,1);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartEntryMap[] = {0,21,42,64,89,110,130};
            static unsigned int sEndEntryMap[] = {0,41,63,88,109,129,148};
            static unsigned int sStartDuringMap[] = {0};
            static unsigned int sEndDuringMap[] = {0};
            static unsigned int sStartExitMap[] = {0};
            static unsigned int sEndExitMap[] = {0};

            _SFD_STATE_COV_MAPS(0,
             7,&(sStartEntryMap[0]),&(sEndEntryMap[0]),
             1,&(sStartDuringMap[0]),&(sEndDuringMap[0]),
             1,&(sStartExitMap[0]),&(sEndExitMap[0]));
          }
          _SFD_TRANS_COV_WTS(9,0,0,0,0);
          if(chartAlreadyPresent==0)
          {
            _SFD_TRANS_COV_MAPS(9,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(2,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {1};
            static unsigned int sEndGuardMap[] = {10};
            _SFD_TRANS_COV_MAPS(2,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(3,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {2};
            static unsigned int sEndGuardMap[] = {11};
            _SFD_TRANS_COV_MAPS(3,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(16,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {2};
            static unsigned int sEndGuardMap[] = {11};
            _SFD_TRANS_COV_MAPS(16,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(13,0,0,0,0);
          if(chartAlreadyPresent==0)
          {
            _SFD_TRANS_COV_MAPS(13,
             0,NULL,NULL,
             0,NULL,NULL,
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
          _SFD_TRANS_COV_WTS(15,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {3};
            static unsigned int sEndGuardMap[] = {13};
            _SFD_TRANS_COV_MAPS(15,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(7,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {4};
            static unsigned int sEndGuardMap[] = {15};
            _SFD_TRANS_COV_MAPS(7,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(8,0,0,0,0);
          if(chartAlreadyPresent==0)
          {
            _SFD_TRANS_COV_MAPS(8,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(4,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {2};
            static unsigned int sEndGuardMap[] = {10};
            _SFD_TRANS_COV_MAPS(4,
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
          _SFD_TRANS_COV_WTS(14,0,2,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {4,32};
            static unsigned int sEndGuardMap[] = {24,53};
            _SFD_TRANS_COV_MAPS(14,
             0,NULL,NULL,
             2,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
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
          _SFD_TRANS_COV_WTS(0,0,0,0,0);
          if(chartAlreadyPresent==0)
          {
            _SFD_TRANS_COV_MAPS(0,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(11,0,2,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {3,18};
            static unsigned int sEndGuardMap[] = {12,29};
            _SFD_TRANS_COV_MAPS(11,
             0,NULL,NULL,
             2,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(12,0,1,0,0);
          if(chartAlreadyPresent==0)
          {
            static unsigned int sStartGuardMap[] = {3};
            static unsigned int sEndGuardMap[] = {12};
            _SFD_TRANS_COV_MAPS(12,
             0,NULL,NULL,
             1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_TRANS_COV_WTS(10,0,0,0,0);
          if(chartAlreadyPresent==0)
          {
            _SFD_TRANS_COV_MAPS(10,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL,
             0,NULL,NULL);
          }
          _SFD_SET_DATA_VALUE_PTR(16,(void *)(&InputData_m0_c8_d10_iPWM_Freq));
          _SFD_SET_DATA_VALUE_PTR(8,(void *)(&InputData_m0_c8_d11_idPWM));
          _SFD_SET_DATA_VALUE_PTR(0,(void
            *)(&(chartInstance.LocalData.m0_c8_d2_DTCs[0])));
          _SFD_SET_DATA_VALUE_PTR(19,(void
            *)(&OutputData_m0_c8_d13_odFixedValueSel));
          _SFD_SET_DATA_VALUE_PTR(17,(void *)(&OutputData_m0_c8_d14_odPumpOff));
          _SFD_SET_DATA_VALUE_PTR(4,(void
            *)(&chartInstance.LocalData.m0_c8_d9_index));
          _SFD_SET_DATA_VALUE_PTR(5,(void
            *)(&chartInstance.LocalData.m0_c8_d1_DTC));
          _SFD_SET_DATA_VALUE_PTR(15,(void
            *)(&chartInstance.LocalData.m0_c8_d3_NDTC));
          _SFD_SET_DATA_VALUE_PTR(7,(void
            *)(&chartInstance.LocalData.m0_c8_d4_NoDTC));
          _SFD_SET_DATA_VALUE_PTR(9,(void
            *)(&chartInstance.LocalData.m0_c8_d7_TempDTC));
          _SFD_SET_DATA_VALUE_PTR(2,(void *)(&m0_c8_d21_WAITTIME));
          _SFD_SET_DATA_VALUE_PTR(1,(void
            *)(&chartInstance.LocalData.m0_c8_d8_count));
          _SFD_SET_DATA_VALUE_PTR(6,(void
            *)(&chartInstance.LocalData.m0_c8_d6_SavedDTC));
          _SFD_SET_DATA_VALUE_PTR(11,(void
            *)(&OutputData_m0_c8_d15_odPI_Disable));
          _SFD_SET_DATA_VALUE_PTR(12,(void
            *)(&chartInstance.LocalData.m0_c8_d5_ST7_Ready));
          _SFD_SET_DATA_VALUE_PTR(10,(void
            *)(&OutputData_m0_c8_d16_oPWM_SC_Alarm));
          _SFD_SET_DATA_VALUE_PTR(18,(void *)(&OutputData_m0_c8_d17_oPWM_Alarm));
          _SFD_SET_DATA_VALUE_PTR(14,(void *)(&InputData_m0_c8_d12_iReset));
          _SFD_SET_DATA_VALUE_PTR(20,(void
            *)(&OutputData_m0_c8_d18_odFixedLowValueSel));
          _SFD_SET_DATA_VALUE_PTR(13,(void *)(&m0_c8_d20_Low_Freq));
          _SFD_SET_DATA_VALUE_PTR(3,(void *)(&m0_c8_d19_High_Freq));
        }
      }
    }else{
      sf_debug_reset_current_state_configuration(_BVH2_App_LayerMachineNumber_,chartInstance.chartNumber,chartInstance.instanceNumber);
    }
  }
  chartInstance.chartInfo.chartInitialized = 1;
}

static void enable_BVH2_App_Layer_sfun_c8( SimStruct *S)
{
}

static void disable_BVH2_App_Layer_sfun_c8( SimStruct *S)
{
}

void BVH2_App_Layer_sfun_c8_sizes_registry(SimStruct *S)
{
  ssSetSFInitOutput(S,0);
  ssSetNumInputPorts((SimStruct *)S, 3);
  ssSetInputPortDataType((SimStruct *)S,0,SS_DOUBLE); /* InputData_m0_c8_d10_iPWM_Freq */
  ssSetInputPortRequiredContiguous(S,0,1);
  ssSetInputPortWidth((SimStruct *)S,0,1);
  ssSetInputPortDirectFeedThrough((SimStruct *)S,0,1);
  ssSetInputPortDataType((SimStruct *)S,1,SS_DOUBLE); /* InputData_m0_c8_d11_idPWM */
  ssSetInputPortRequiredContiguous(S,1,1);
  ssSetInputPortWidth((SimStruct *)S,1,1);
  ssSetInputPortDirectFeedThrough((SimStruct *)S,1,1);
  ssSetInputPortDataType((SimStruct *)S,2,SS_DOUBLE); /* InputData_m0_c8_d12_iReset */
  ssSetInputPortRequiredContiguous(S,2,1);
  ssSetInputPortWidth((SimStruct *)S,2,1);
  ssSetInputPortDirectFeedThrough((SimStruct *)S,2,1);
  ssSetNumOutputPorts((SimStruct *)S, 7);
  ssSetOutputPortDataType((SimStruct *)S,0,SS_DOUBLE);
  ssSetOutputPortWidth((SimStruct *)S,0,1);
  ssSetOutputPortDataType((SimStruct *)S,1,SS_DOUBLE); /* OutputData_m0_c8_d13_odFixedValueSel */
  ssSetOutputPortWidth((SimStruct *)S,1,1);
  ssSetOutputPortDataType((SimStruct *)S,2,SS_DOUBLE); /* OutputData_m0_c8_d14_odPumpOff */
  ssSetOutputPortWidth((SimStruct *)S,2,1);
  ssSetOutputPortDataType((SimStruct *)S,3,SS_DOUBLE); /* OutputData_m0_c8_d15_odPI_Disable */
  ssSetOutputPortWidth((SimStruct *)S,3,1);
  ssSetOutputPortDataType((SimStruct *)S,4,SS_DOUBLE); /* OutputData_m0_c8_d16_oPWM_SC_Alarm */
  ssSetOutputPortWidth((SimStruct *)S,4,1);
  ssSetOutputPortDataType((SimStruct *)S,5,SS_DOUBLE); /* OutputData_m0_c8_d17_oPWM_Alarm */
  ssSetOutputPortWidth((SimStruct *)S,5,1);
  ssSetOutputPortDataType((SimStruct *)S,6,SS_DOUBLE); /* OutputData_m0_c8_d18_odFixedLowValueSel */
  ssSetOutputPortWidth((SimStruct *)S,6,1);
  if(sim_mode_is_rtw_gen(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("BVH2_App_Layer",8);
    ssSetStateflowIsInlinable((SimStruct *)S,chartIsInlinable);
    if(chartIsInlinable) {
      ssSetInputPortReusable((SimStruct *)S,0,1);
      ssSetInputPortReusable((SimStruct *)S,1,1);
      ssSetInputPortReusable((SimStruct *)S,2,1);
      sf_mark_chart_expressionable_inputs((SimStruct *)S,"BVH2_App_Layer",8,3);
      sf_mark_chart_reusable_outputs((SimStruct *)S,"BVH2_App_Layer",8,6);
    }
    {
      int dtId;
      char *chartInstanceTypedefName =
        sf_chart_instance_typedef_name("BVH2_App_Layer",8);
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
  ssSetChecksum0(S,(2345472014U));
  ssSetChecksum1(S,(1450511866U));
  ssSetChecksum2(S,(1289644125U));
  ssSetChecksum3(S,(3347275170U));
  ssSetExplicitFCSSCtrl(S,1);
}

void terminate_BVH2_App_Layer_sfun_c8(SimStruct *S)
{
}
static void mdlRTW_BVH2_App_Layer_sfun_c8(SimStruct *S)
{
}

void sf_BVH2_App_Layer_sfun_c8( void *);
void BVH2_App_Layer_sfun_c8_registry(SimStruct *S)
{
  chartInstance.chartInfo.chartInstance = NULL;
  chartInstance.chartInfo.chartInitialized = 0;
  chartInstance.chartInfo.sFunctionGateway = sf_BVH2_App_Layer_sfun_c8;
  chartInstance.chartInfo.initializeChart = initialize_BVH2_App_Layer_sfun_c8;
  chartInstance.chartInfo.terminateChart = terminate_BVH2_App_Layer_sfun_c8;
  chartInstance.chartInfo.enableChart = enable_BVH2_App_Layer_sfun_c8;
  chartInstance.chartInfo.disableChart = disable_BVH2_App_Layer_sfun_c8;
  chartInstance.chartInfo.mdlRTW = mdlRTW_BVH2_App_Layer_sfun_c8;
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

void sf_BVH2_App_Layer_sfun_c8(void *chartInstanceVoidPtr)
{
  /* Save current event being processed */
  uint8_T previousEvent;
  previousEvent = _sfEvent_;

  /* Update Stateflow time variable */
  _sfTime_ = ssGetT(chartInstance.S);

  /* Call this chart */
  _sfEvent_ = CALL_EVENT;
  BVH2_App_Layer_sfun_c8();
  _sfEvent_ = previousEvent;
}

