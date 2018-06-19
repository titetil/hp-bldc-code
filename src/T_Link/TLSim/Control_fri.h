/**************************************************************************************************\
 *** 
 *** Simulink model       : BVH2_App_Layer
 *** TargetLink subsystem : BVH2_App_Layer/Control
 *** Codefile             : control_fri.h
 ***
 *** Generation date: 2011-03-24 10:44:14
 ***
 *** TargetLink version      : 2.0 from 22-Apr-2004
 *** Code generator version  : 2.0.0 build id 48 from 2004-04-22 13:23:03
 *** Copyright (c) 2004 dSPACE GmbH
\**************************************************************************************************/ 

#ifndef _CONTROL_FRI_H_
#define _CONTROL_FRI_H_

#include "tl_defines_b.h"
#include "tl_basetypes.h"
#include "tl_types.h"
#ifdef TL_FRAME
#include "Control_frm.h"
#endif
#include "Control.h"

/**********************************************************************\
   GLOBAL: global variables (RAM)
\**********************************************************************/
extern GLOBAL UInt16 DutyCyclePowerstage;
extern GLOBAL Int16 Pump_On;
extern GLOBAL UInt16 adcPressure;
extern GLOBAL UInt16 adcTemp;
extern GLOBAL Int16 debug_out0;
extern GLOBAL Int16 debug_out1;
extern GLOBAL Bool EKPLaufAlarm;
extern GLOBAL Bool EKP_On;
extern GLOBAL UInt8 Ki;
extern GLOBAL UInt8 Kp /* LSB: 2^-8 OFF:  0 MIN/MAX:  0 .. 0.99609375 */;
extern GLOBAL UInt8 MotorFreq;
extern GLOBAL UInt8 OpenLoopFlag;
extern GLOBAL UInt8 ResetCounter;
extern GLOBAL UInt8 Second_timer;
extern GLOBAL UInt8 SpeedAlarm_fast;
extern GLOBAL UInt8 SpeedAlarm_slow;
extern GLOBAL Bool TempAlarm;
extern GLOBAL Bool VoltageAlarm;
extern GLOBAL UInt8 adcCurrent;
extern GLOBAL UInt8 adcDriverVoltage;
extern GLOBAL UInt8 adcVoltage;
extern GLOBAL UInt8 pwm3Pressure;

extern Void Control(Void);

#endif/*_CONTROL_FRI_H_ */
