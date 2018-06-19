/**************************************************************************************************\
 *** 
 *** Simulink model       : BVH2_App_Layer
 *** TargetLink subsystem : BVH2_App_Layer/BVH2_Appl_Layer
 *** Codefile             : bvh2_appl_layer_fri.h
 ***
 *** Generation date: 2015-06-22 15:49:17
 ***
 *** TargetLink version      : 2.0 from 22-Apr-2004
 *** Code generator version  : 2.0.0 build id 48 from 2004-04-22 13:23:03
 *** Copyright (c) 2004 dSPACE GmbH
\**************************************************************************************************/ 

#ifndef _BVH2_APPL_LAYER_FRI_H_
#define _BVH2_APPL_LAYER_FRI_H_

#include "tl_defines_b.h"
#include "tl_basetypes.h"
#include "tl_types.h"
#ifdef TL_FRAME
#include "BVH2_Appl_Layer_frm.h"
#endif
#include "BVH2_Appl_Layer.h"

/**********************************************************************\
   GLOBAL: global variables (RAM)
\**********************************************************************/
extern GLOBAL UInt16 ui16_Current_Thresh;
extern GLOBAL UInt16 ui16_PWM_Freq_mat;
extern GLOBAL UInt16 ui16_Speed_demand_mat;
extern GLOBAL UInt16 ui16_Speed_demand_mat_Max;
extern GLOBAL UInt16 ui16_Speed_demand_mat_min;
extern GLOBAL UInt16 ui16_Speed_mat;
extern GLOBAL UInt16 ui16_dryRun_Thresh;
extern GLOBAL UInt16 ui16_mat_Current;
extern GLOBAL UInt16 ui16_mat_inpTemp;
extern GLOBAL UInt16 ui8_BattVolt_mat;
extern GLOBAL UInt16 ui8_Ki_mat;
extern GLOBAL UInt16 ui8_fixed_start_speed_mat;
extern GLOBAL Bool bl_Pumpoff_Alarm;
extern GLOBAL Bool bool_CPU_TempAlarm;
extern GLOBAL Bool bool_CPU_TempRedAlarm;
extern GLOBAL Bool bool_ControlLoopMode;
extern GLOBAL Bool bool_DryRunningAlarm;
extern GLOBAL Bool bool_HighCurrentAlarm;
extern GLOBAL Bool bool_MotorStalled;
extern GLOBAL Bool bool_PIC_Alarm;
extern GLOBAL Bool bool_PWMin_Freq_err_Alarm;
extern GLOBAL Bool bool_PWMin_err_Alarm;
extern GLOBAL Bool bool_StalledMotorStop;
extern GLOBAL Bool bool_UbatAlarm;
extern GLOBAL Bool bool_mat_currAlarm_bldc;
extern GLOBAL Bool bool_mat_pic_etat;
extern GLOBAL Bool bool_start_demand_mat;
extern GLOBAL UInt8 ui8_Kp_mat;
extern GLOBAL UInt8 ui8_PWM_dc_mat;
extern GLOBAL UInt8 ui8_ResetMatlab;
extern GLOBAL UInt8 ui8_debug_out0;
extern GLOBAL UInt8 ui8_duty_cycle_mat;

extern Void BVH2_Appl_Layer(Void);

#endif/*_BVH2_APPL_LAYER_FRI_H_ */
