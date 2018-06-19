#include "BVH2_App_Layer_sfun.h"
#include "sfcdebug.h"
#define PROCESS_MEX_SFUNCTION_CMD_LINE_CALL
unsigned int sf_process_check_sum_call( int nlhs, mxArray * plhs[], int nrhs,
 const mxArray * prhs[] )
{
  extern unsigned int sf_BVH2_App_Layer_process_check_sum_call( int nlhs,
   mxArray * plhs[], int nrhs, const mxArray * prhs[] );

  if(sf_BVH2_App_Layer_process_check_sum_call(nlhs,plhs,nrhs,prhs)) return 1;
  return 0;
}
extern unsigned int sf_debug_api( int nlhs, mxArray * plhs[], int nrhs, const
 mxArray * prhs[] );
static unsigned int ProcessMexSfunctionCmdLineCall(int nlhs, mxArray * plhs[],
 int nrhs, const mxArray * prhs[])
{
  if(sf_debug_api(nlhs,plhs,nrhs,prhs)) return 1;
  if(sf_process_check_sum_call(nlhs,plhs,nrhs,prhs)) return 1;
  return 0;
}
static unsigned int sfMachineGlobalTerminatorCallable = 0;
static unsigned int sfMachineGlobalInitializerCallable = 1;
extern unsigned int BVH2_App_Layer_registry(SimStruct *S,char *chartName,int
 initializeFlag);
unsigned int sf_machine_global_registry(SimStruct *simstructPtr, char
 *chartName, int initializeFlag)
{
  if(BVH2_App_Layer_registry(simstructPtr,chartName,initializeFlag)) return 1;
  return 0;
}
extern unsigned int BVH2_App_Layer_sizes_registry(SimStruct *S,char *chartName);
unsigned int sf_machine_global_sizes_registry(SimStruct *simstructPtr, char
 *chartName)
{
  if(BVH2_App_Layer_sizes_registry(simstructPtr,chartName)) return 1;
  return 0;
}
extern void BVH2_App_Layer_terminator(void);
void sf_machine_global_terminator(void)
{
  if(sfMachineGlobalTerminatorCallable) {
    sfMachineGlobalTerminatorCallable = 0;
    sfMachineGlobalInitializerCallable = 1;
    BVH2_App_Layer_terminator();
    sf_debug_terminate();
  }
  return;
}
extern void BVH2_App_Layer_initializer(void);
void sf_machine_global_initializer(void)
{
  if(sfMachineGlobalInitializerCallable) {
    sfMachineGlobalInitializerCallable = 0;
    sfMachineGlobalTerminatorCallable =1;
    BVH2_App_Layer_initializer();
  }
  return;
}
#include "simulink.c"                   /* MEX-file interface mechanism */

