#include "BVH2_App_Layer_sfun.h"
#include "BVH2_App_Layer_sfun_c1.h"
#include "BVH2_App_Layer_sfun_c2.h"
#include "BVH2_App_Layer_sfun_c3.h"
#include "BVH2_App_Layer_sfun_c4.h"
#include "BVH2_App_Layer_sfun_c5.h"
#include "BVH2_App_Layer_sfun_c6.h"
#include "BVH2_App_Layer_sfun_c7.h"
#include "BVH2_App_Layer_sfun_c8.h"
#include "BVH2_App_Layer_sfun_c9.h"

/* Global machine event */
uint8_T _sfEvent_;
#include "BVH2_App_Layer_sfun_debug_macros.h"
unsigned int _BVH2_App_LayerMachineNumber_=UNREASONABLE_NUMBER;
unsigned int sf_BVH2_App_Layer_process_check_sum_call( int nlhs, mxArray *
 plhs[], int nrhs, const mxArray * prhs[] )
{
#ifdef MATLAB_MEX_FILE
  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) ) return 0;
  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if(strcmp(commandName,"sf_get_check_sum")) return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if(nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if(!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(573697867U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3579553188U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(640070963U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1206434391U);
    }else if(!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    }else if(!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2027058304U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2682626131U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1397024817U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1341245007U);
    }else if(nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch(chartFileNumber) {
       case 1:
        {
          extern void sf_BVH2_App_Layer_sfun_c1_get_check_sum(mxArray *plhs[]);
          sf_BVH2_App_Layer_sfun_c1_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_BVH2_App_Layer_sfun_c2_get_check_sum(mxArray *plhs[]);
          sf_BVH2_App_Layer_sfun_c2_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_BVH2_App_Layer_sfun_c3_get_check_sum(mxArray *plhs[]);
          sf_BVH2_App_Layer_sfun_c3_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_BVH2_App_Layer_sfun_c4_get_check_sum(mxArray *plhs[]);
          sf_BVH2_App_Layer_sfun_c4_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void sf_BVH2_App_Layer_sfun_c5_get_check_sum(mxArray *plhs[]);
          sf_BVH2_App_Layer_sfun_c5_get_check_sum(plhs);
          break;
        }

       case 6:
        {
          extern void sf_BVH2_App_Layer_sfun_c6_get_check_sum(mxArray *plhs[]);
          sf_BVH2_App_Layer_sfun_c6_get_check_sum(plhs);
          break;
        }

       case 7:
        {
          extern void sf_BVH2_App_Layer_sfun_c7_get_check_sum(mxArray *plhs[]);
          sf_BVH2_App_Layer_sfun_c7_get_check_sum(plhs);
          break;
        }

       case 8:
        {
          extern void sf_BVH2_App_Layer_sfun_c8_get_check_sum(mxArray *plhs[]);
          sf_BVH2_App_Layer_sfun_c8_get_check_sum(plhs);
          break;
        }

       case 9:
        {
          extern void sf_BVH2_App_Layer_sfun_c9_get_check_sum(mxArray *plhs[]);
          sf_BVH2_App_Layer_sfun_c9_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    }else if(!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2575433854U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1646467997U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3563589781U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1161868961U);
    }else {
      return 0;
    }
  } else{
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2813968827U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4005536713U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(4120663900U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1739769440U);
  }
  return 1;
#else
  return 0;
#endif
}
/* Stateflow time variable */
real_T _sfTime_;

/* Machine initialize */
void BVH2_App_Layer_initializer(void)
{
  _BVH2_App_LayerMachineNumber_ =
    sf_debug_initialize_machine("BVH2_App_Layer","sfun",0,9,0,0,0);
  sf_debug_set_machine_event_thresholds(_BVH2_App_LayerMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(_BVH2_App_LayerMachineNumber_,0);
}

unsigned int BVH2_App_Layer_registry(SimStruct *simstructPtr,char *chartName,
 int initializeFlag)
{
  if(!strcmp_ignore_ws(chartName,"BVH2_App_Layer/BVH2_Appl_Layer/Subsystem/BVH2_Appl_Layer/Current_Analysis_High/ SFunction "))
  {
    BVH2_App_Layer_sfun_c1_registry(simstructPtr);
    return 1;
  }
  if(!strcmp_ignore_ws(chartName,"BVH2_App_Layer/BVH2_Appl_Layer/Subsystem/BVH2_Appl_Layer/UbatHandling/ SFunction "))
  {
    BVH2_App_Layer_sfun_c2_registry(simstructPtr);
    return 1;
  }
  if(!strcmp_ignore_ws(chartName,"BVH2_App_Layer/BVH2_Appl_Layer/Subsystem/BVH2_Appl_Layer/Temperature_Alarm/ SFunction "))
  {
    BVH2_App_Layer_sfun_c3_registry(simstructPtr);
    return 1;
  }
  if(!strcmp_ignore_ws(chartName,"BVH2_App_Layer/BVH2_Appl_Layer/Subsystem/BVH2_Appl_Layer/Pic_etat_monitor/ SFunction "))
  {
    BVH2_App_Layer_sfun_c4_registry(simstructPtr);
    return 1;
  }
  if(!strcmp_ignore_ws(chartName,"BVH2_App_Layer/BVH2_Appl_Layer/Subsystem/BVH2_Appl_Layer/Current_Analysis_low/ SFunction "))
  {
    BVH2_App_Layer_sfun_c5_registry(simstructPtr);
    return 1;
  }
  if(!strcmp_ignore_ws(chartName,"BVH2_App_Layer/BVH2_Appl_Layer/Subsystem/BVH2_Appl_Layer/Dry_Running/ SFunction "))
  {
    BVH2_App_Layer_sfun_c6_registry(simstructPtr);
    return 1;
  }
  if(!strcmp_ignore_ws(chartName,"BVH2_App_Layer/BVH2_Appl_Layer/Subsystem/BVH2_Appl_Layer/Motor_Stalled/ SFunction "))
  {
    BVH2_App_Layer_sfun_c7_registry(simstructPtr);
    return 1;
  }
  if(!strcmp_ignore_ws(chartName,"BVH2_App_Layer/BVH2_Appl_Layer/Subsystem/BVH2_Appl_Layer/PWM_Detection/ SFunction "))
  {
    BVH2_App_Layer_sfun_c8_registry(simstructPtr);
    return 1;
  }
  if(!strcmp_ignore_ws(chartName,"BVH2_App_Layer/BVH2_Appl_Layer/Subsystem/BVH2_Appl_Layer/Dry_RunningAlarm/ SFunction "))
  {
    BVH2_App_Layer_sfun_c9_registry(simstructPtr);
    return 1;
  }
  return 0;
}
unsigned int BVH2_App_Layer_sizes_registry(SimStruct *simstructPtr,char
 *chartName)
{
  if(!strcmp_ignore_ws(chartName,"BVH2_App_Layer/BVH2_Appl_Layer/Subsystem/BVH2_Appl_Layer/Current_Analysis_High/ SFunction "))
  {
    BVH2_App_Layer_sfun_c1_sizes_registry(simstructPtr);
    return 1;
  }
  if(!strcmp_ignore_ws(chartName,"BVH2_App_Layer/BVH2_Appl_Layer/Subsystem/BVH2_Appl_Layer/UbatHandling/ SFunction "))
  {
    BVH2_App_Layer_sfun_c2_sizes_registry(simstructPtr);
    return 1;
  }
  if(!strcmp_ignore_ws(chartName,"BVH2_App_Layer/BVH2_Appl_Layer/Subsystem/BVH2_Appl_Layer/Temperature_Alarm/ SFunction "))
  {
    BVH2_App_Layer_sfun_c3_sizes_registry(simstructPtr);
    return 1;
  }
  if(!strcmp_ignore_ws(chartName,"BVH2_App_Layer/BVH2_Appl_Layer/Subsystem/BVH2_Appl_Layer/Pic_etat_monitor/ SFunction "))
  {
    BVH2_App_Layer_sfun_c4_sizes_registry(simstructPtr);
    return 1;
  }
  if(!strcmp_ignore_ws(chartName,"BVH2_App_Layer/BVH2_Appl_Layer/Subsystem/BVH2_Appl_Layer/Current_Analysis_low/ SFunction "))
  {
    BVH2_App_Layer_sfun_c5_sizes_registry(simstructPtr);
    return 1;
  }
  if(!strcmp_ignore_ws(chartName,"BVH2_App_Layer/BVH2_Appl_Layer/Subsystem/BVH2_Appl_Layer/Dry_Running/ SFunction "))
  {
    BVH2_App_Layer_sfun_c6_sizes_registry(simstructPtr);
    return 1;
  }
  if(!strcmp_ignore_ws(chartName,"BVH2_App_Layer/BVH2_Appl_Layer/Subsystem/BVH2_Appl_Layer/Motor_Stalled/ SFunction "))
  {
    BVH2_App_Layer_sfun_c7_sizes_registry(simstructPtr);
    return 1;
  }
  if(!strcmp_ignore_ws(chartName,"BVH2_App_Layer/BVH2_Appl_Layer/Subsystem/BVH2_Appl_Layer/PWM_Detection/ SFunction "))
  {
    BVH2_App_Layer_sfun_c8_sizes_registry(simstructPtr);
    return 1;
  }
  if(!strcmp_ignore_ws(chartName,"BVH2_App_Layer/BVH2_Appl_Layer/Subsystem/BVH2_Appl_Layer/Dry_RunningAlarm/ SFunction "))
  {
    BVH2_App_Layer_sfun_c9_sizes_registry(simstructPtr);
    return 1;
  }
  return 0;
}
void BVH2_App_Layer_terminator(void)
{
}

