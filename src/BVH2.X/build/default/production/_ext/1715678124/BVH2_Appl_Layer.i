
# 1 "../T_Link/BVH2_Appl_Layer.c"

# 17 "../T_Link/tl_basetypes.h"
typedef unsigned char Bool;
typedef float Float32;
typedef double Float64;
typedef signed short int Int16;
typedef signed long int Int32;
typedef signed char Int8;
typedef unsigned short int UInt16;
typedef unsigned long int UInt32;
typedef unsigned char UInt8;
typedef void Void;

# 66 "../T_Link/adv64.h"
typedef struct
{
UInt32 lo;
UInt32 hi;
} UInt64s;

typedef union
{
UInt32 lng;
UInt16 pr[2];
} UInt32s;

# 101
void negate(UInt64s *x);



UInt64s F__U64sMULU32U32(const UInt32 x, const UInt32 y);


UInt64s F__I64sMULI32U32(const Int32 x, const UInt32 y);

# 115
UInt64s F__I64sMULI32I32(const Int32 x, const Int32 y);


UInt16 div3by2(UInt16 u[3], UInt32s v);


UInt64s F__U64sDIVU64sU16(const UInt64s *u, UInt16 a);


UInt32 F__U32DIVU64sU16(const UInt64s *u, UInt16 a);


UInt64s F__U64sDIVU64sU32(const UInt64s *U, UInt32 v);


UInt64s F__I64sDIVI64sU32(const UInt64s *u, UInt32 a);


UInt64s F__I64sDIVU64sI32(const UInt64s *u, Int32 a);


UInt64s F__I64sDIVI64sI32(const UInt64s *u, Int32 a);


UInt32 F__U32DIVU64sU32(const UInt64s *U, UInt32 v);


Int32 F__I32DIVI64sU32(const UInt64s *u, UInt32 a);


Int32 F__I32DIVU64sI32(const UInt64s *u, Int32 a);


Int32 F__I32DIVI64sI32(const UInt64s *u, Int32 a);

# 157 "../T_Link/div.h"
void F__I64DIVI64I32(Int32 n_h, UInt32 n_l, Int32 d, Int32 *r_h, UInt32 *r_l);

# 212
void F__I64DIVI64U32(Int32 n_h, UInt32 n_l, UInt32 d, Int32 *r_h, UInt32 *r_l);

# 266
void F__I64DIVU64I32(UInt32 n_h, UInt32 n_l, Int32 d, Int32 *r_h, UInt32 *r_l);

# 322
void F__U64DIVU64U32(UInt32 n_h, UInt32 n_l, UInt32 d, UInt32 *r_h, UInt32 *r_l);

# 434
Int32 F__I32DIVI64I32(Int32 n_h, UInt32 n_l, Int32 d);

# 474
Int32 F__I32DIVI64U32(Int32 n_h, UInt32 n_l, UInt32 d);

# 512
Int32 F__I32DIVU64I32(UInt32 n_h, UInt32 n_l, Int32 d);

# 552
UInt32 F__U32DIVU64U32(UInt32 n_h, UInt32 n_l, UInt32 d);

# 47 "../T_Link/trig.h"
extern const UInt16 cosLUT[129];

# 64
Int16 F__I16SINI16(Int16 v);

# 72
Int32 F__I32SINI32_LE7(Int32 v);
Int32 F__I32SINI32_LE8(Int32 v);
Int32 F__I32SINI32_LE9(Int32 v);
Int32 F__I32SINI32_LE10(Int32 v);
Int32 F__I32SINI32_LE11(Int32 v);

# 163
Int32 F__I32SINI32_6TERMS(Int32 v);
Int32 F__I32SINI32_5TERMS(Int32 v);
Int32 F__I32SINI32_4TERMS(Int32 v);

Int32 F__I32COSI32_6TERMS(Int32 v);
Int32 F__I32COSI32_5TERMS(Int32 v);
Int32 F__I32COSI32_4TERMS(Int32 v);

# 204
Int16 F__I16TANI16(Int16 v, Int8 ysh);
Int16 F__I16TANI16_ARB(Int16 v, Int16 yN, Int16 yD);
Int32 F__I32TANI32_LE7(Int32 v, Int8 ysh);
Int32 F__I32TANI32_LE7_ARB(Int32 v, Int32 yN, Int32 yD);
Int32 F__I32TANI32_LE8(Int32 v, Int8 ysh);
Int32 F__I32TANI32_LE8_ARB(Int32 v, Int32 yN, Int32 yD);
Int32 F__I32TANI32_LE9(Int32 v, Int8 ysh);
Int32 F__I32TANI32_LE9_ARB(Int32 v, Int32 yN, Int32 yD);
Int32 F__I32TANI32_LE10(Int32 v, Int8 ysh);
Int32 F__I32TANI32_LE10_ARB(Int32 v, Int32 yN, Int32 yD);
Int32 F__I32TANI32_LE11(Int32 v, Int8 ysh);
Int32 F__I32TANI32_LE11_ARB(Int32 v, Int32 yN, Int32 yD);

# 268
Int16 F__I16ASINI16(Int16 v);

# 291
Int16 F__I16ATANI16(Int16 v, Int32 c);

Int16 F__I16ATAN2I16(Int16 v, Int32 c, Int16 y);

# 30 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\c90\math.h"
extern double fabs(double);
extern double floor(double);
extern double ceil(double);
extern double modf(double, double *);
extern double sqrt(double);
extern double atof(const char *);
extern double sin(double) ;
extern double cos(double) ;
extern double tan(double) ;
extern double asin(double) ;
extern double acos(double) ;
extern double atan(double);
extern double atan2(double, double) ;
extern double log(double);
extern double log10(double);
extern double pow(double, double) ;
extern double exp(double) ;
extern double sinh(double) ;
extern double cosh(double) ;
extern double tanh(double);
extern double eval_poly(double, const double *, int);
extern double frexp(double, int *);
extern double ldexp(double, int);
extern double fmod(double, double);
extern double trunc(double);
extern double round(double);

# 61 "../T_Link/BVH2_Appl_Layer.h"
struct tag_SIBFS_Current_Analysis_High_b_tp {
unsigned int Cb1_Current_Analysis_High_ns : 4;
unsigned int Cb1_glflag : 2;
unsigned int Cb1_Current_Analysis_High : 1;
};

struct tag_SIBFS_Current_Analysis_low_b_tp {
unsigned int Cb10_greenState : 1;
unsigned int Cb11_Wait : 1;
unsigned int Cb12_CntOverCurrent : 1;
unsigned int Cb13_redState : 1;
unsigned int Cb9_Current_Analysis_low : 1;
};

struct tag_SIBFS_Dry_RunningAlarm_b_tp {
unsigned int Cb19_Dry_RunningAlarm : 1;
unsigned int Cb20_greenState : 1;
unsigned int Cb21_DryRunningAlarm : 1;
unsigned int Cb22_DryRun66 : 1;
unsigned int Cb23_DryRun55 : 1;
};

struct tag_SIBFS_Dry_Running_b_tp {
unsigned int Cb14_Dry_Running : 1;
unsigned int Cb15_greenState : 1;
unsigned int Cb16_DryRunning : 1;
unsigned int Cb17_redState : 1;
unsigned int Cb18_CntOverCurrent : 1;
};

struct tag_SIBFS_Motor_Stalled_b_tp {
unsigned int Cb25_Motor_sta__Statemachine_ns : 3;
unsigned int Cb24_glflag : 2;
unsigned int Cb24_Motor_Stalled : 1;
unsigned int Cb25_Motor_stalled_Statemachine : 1;
unsigned int Cb32_default : 1;
unsigned int Cb33_Stop : 1;
};

struct tag_SIBFS_PWM_Detection_b_tp {
unsigned int Cb35_PWMinput_handling_ns : 4;
unsigned int Cb35_PWMinput_handling : 1;
};

struct tag_SIBFS_Pic_etat_monitor_b_tp {
unsigned int Cb44_Pic_etat_monitor_ns : 3;
unsigned int Cb44_glflag : 2;
unsigned int Cb44_Pic_etat_monitor : 1;
};

struct tag_SIBFS_Temperature_Alarm_b_tp {
unsigned int Cb51_Temperature_Alarm : 1;
unsigned int Cb52_CntOverTemp : 1;
unsigned int Cb53_reset : 1;
unsigned int Cb54_greenTemp : 1;
unsigned int Cb55_redTemp : 1;
};

struct tag_SIBFS_UbatHandling_b_tp {
unsigned int Aux_sflag3 : 3;
unsigned int Cb57_Ubat_Handling : 1;
unsigned int Cb58_SaturationHigh : 1;
unsigned int Cb59_SaturationLow : 1;
unsigned int Cb60_NormalUbat : 1;
};

# 136
extern const UInt16 Sb2_Fixed_Power;

# 141
extern  UInt16 ui16_Current_Thresh;
extern  UInt16 ui16_PWM_Freq_mat;
extern  UInt16 ui16_Speed_demand_mat;
extern  UInt16 ui16_Speed_demand_mat_Max;
extern  UInt16 ui16_Speed_demand_mat_min;
extern  UInt16 ui16_Speed_mat;
extern  UInt16 ui16_dryRun_Thresh;
extern  UInt16 ui16_mat_Current;
extern  UInt16 ui16_mat_inpTemp;
extern  UInt16 ui8_BattVolt_mat;
extern  UInt16 ui8_Ki_mat;
extern  UInt16 ui8_fixed_start_speed_mat;
extern  Bool bl_Pumpoff_Alarm;
extern  Bool bool_CPU_TempAlarm;
extern  Bool bool_CPU_TempRedAlarm;
extern  Bool bool_ControlLoopMode;
extern  Bool bool_DryRunningAlarm;
extern  Bool bool_HighCurrentAlarm;
extern  Bool bool_MotorStalled;
extern  Bool bool_PIC_Alarm;
extern  Bool bool_PWMin_Freq_err_Alarm;
extern  Bool bool_PWMin_err_Alarm;
extern  Bool bool_StalledMotorStop;
extern  Bool bool_UbatAlarm;
extern  Bool bool_mat_currAlarm_bldc;
extern  Bool bool_mat_pic_etat;
extern  Bool bool_start_demand_mat;
extern  UInt8 ui8_Kp_mat;
extern  UInt8 ui8_PWM_dc_mat;
extern  UInt8 ui8_ResetMatlab;
extern  UInt8 ui8_debug_out0;
extern  UInt16 ui16_duty_cycle_mat;

# 183
extern Void BVH2_Appl_Layer(Void);

# 206 "../T_Link/BVH2_Appl_Layer.c"
const UInt16 Sb2_Fixed_Power = 202;

# 211
 UInt16 ui16_Current_Thresh;
 UInt16 ui16_PWM_Freq_mat;
 UInt16 ui16_Speed_demand_mat;
 UInt16 ui16_Speed_demand_mat_Max;
 UInt16 ui16_Speed_demand_mat_min;
 UInt16 ui16_Speed_mat;
 UInt16 ui16_dryRun_Thresh;
 UInt16 ui16_mat_Current;
 UInt16 ui16_mat_inpTemp;
 UInt16 ui8_BattVolt_mat;
 UInt16 ui8_Ki_mat;
 UInt16 ui8_fixed_start_speed_mat;
 Bool bl_Pumpoff_Alarm;
 Bool bool_CPU_TempAlarm;
 Bool bool_CPU_TempRedAlarm;
 Bool bool_ControlLoopMode;
 Bool bool_DryRunningAlarm;
 Bool bool_HighCurrentAlarm;
 Bool bool_MotorStalled;
 Bool bool_PIC_Alarm;
 Bool bool_PWMin_Freq_err_Alarm;
 Bool bool_PWMin_err_Alarm;
 Bool bool_StalledMotorStop;
 Bool bool_UbatAlarm;
 Bool bool_mat_currAlarm_bldc;
 Bool bool_mat_pic_etat;
 Bool bool_start_demand_mat;
 UInt8 ui8_Kp_mat;
 UInt8 ui8_PWM_dc_mat;
 UInt8 ui8_ResetMatlab;
 UInt8 ui8_debug_out0;
 UInt16 ui16_duty_cycle_mat;

# 247
static UInt16 Cb1_StateCnt = 0;
static UInt16 Cb44_StateCnt = 0;
static UInt8 Cb1_RestartCounter = 0;

static struct tag_SIBFS_Current_Analysis_High_b_tp SIBFS_Current_Analysis_High_b = {
0 ,
0 ,
0
} ;

static struct tag_SIBFS_PWM_Detection_b_tp SIBFS_PWM_Detection_b = {
0 ,
0
} ;

static struct tag_SIBFS_Pic_etat_monitor_b_tp SIBFS_Pic_etat_monitor_b = {
0 ,
0 ,
0
} ;

static struct tag_SIBFS_UbatHandling_b_tp SIBFS_UbatHandling_b = {
0 ,
0 ,
0 ,
0 ,
0
} ;

# 279
static Bool Cb1_oCurrentAlarm = 0;
static Bool Cb1_oShutoff = 0;
static UInt8 Cb34_idPWM;
static Bool Cb34_oPWM_Alarm = 0;
static Bool Cb34_oPWM_SC_Alarm = 0;
static Bool Cb34_odFixedLowValueSel = 0;
static Bool Cb34_odFixedValueSel = 0;
static Bool Cb34_odPumpOff = 0;
static Bool Cb44_oAlarm = 0;
static Bool Cb44_oShutoff = 0;
static Bool Cb56_oUbat_Alarm_High = 0;
static Bool Cb56_odFixedValueSel = 0;
static Bool Cb56_odPumpOff = 0;
static Bool power_lockout = 0;
static UInt16 amps_per_volt_cnt = 0;

# 304
static Void Cb1_Current_An___High_node_fcn1(Void);
static Void Cb34_PWM_Detection_node_fcn1(Void);
static Void Cb44_Pic_etat_monitor_node_fcn1(Void);
static Void Cb56_UbatHandling_node_fcn2(Void);

# 333
Void BVH2_Appl_Layer(Void)
{

static UInt16 Cb14_StateCnt = 0;
static UInt16 Cb19_Counter = 0;
static UInt16 Cb24_BadCnt = 0;
static UInt16 Cb24_StateCnt = 0;
static UInt8 Cb24_RestartCounter = 0;
static UInt8 Cb51_Counter = 0;
static UInt8 Cb9_StateCnt = 0;

static struct tag_SIBFS_Current_Analysis_low_b_tp SIBFS_Current_Analysis_low_b = {
0 ,
0 ,
0 ,
0 ,
0
} ;

static struct tag_SIBFS_Dry_RunningAlarm_b_tp SIBFS_Dry_RunningAlarm_b = {
0 ,
0 ,
0 ,
0 ,
0
} ;

static struct tag_SIBFS_Dry_Running_b_tp SIBFS_Dry_Running_b = {
0 ,
0 ,
0 ,
0 ,
0
} ;

static struct tag_SIBFS_Motor_Stalled_b_tp SIBFS_Motor_Stalled_b = {
0 ,
0 ,
0 ,
0 ,
0 ,
0
} ;

static struct tag_SIBFS_Temperature_Alarm_b_tp SIBFS_Temperature_Alarm_b = {
0 ,
0 ,
0 ,
0 ,
0
} ;


Int16 Sb2_Error;
UInt16 Sb2_Switch2;
UInt16 Sb2_Switch5;
Bool Cb24_Reset;
Bool Sb1_Logical_Operator1;
Bool Sb1_Logical_Operator2;
Bool Sb1_Logical_Operator3;
Bool Sb1_Logical_Operator5;
Bool Sb2_Logical_Operator2;


static Bool Cb14_oDryRunAlarm = 0;
static Bool Cb19_oDryRun55 = 0;
static Bool Cb19_oDryRun66 = 0;
static Bool Cb24_oMotorStalled = 0;
static Bool Cb24_oStalledAlarm = 0;
static Bool Cb51_oTempAlarm = 0;
static Bool Cb51_oTempRedAlarm = 0;
static Bool Cb51_odPumpOff = 0;
static Bool Cb9_oCurrentAlarm = 0;


static Int32 X_Sb4_Intergrator = 80000 ;

# 410
static Bool Sb1_BVH2_Appl_Layer_FirstRun = 1;
static Bool X_Sb4_Intergrator_TriggerIn = 1;


switch (SIBFS_Pic_etat_monitor_b.Cb44_Pic_etat_monitor_ns) {
case (UInt8)4: {

if (Cb44_StateCnt > 50) {


SIBFS_Pic_etat_monitor_b.Cb44_Pic_etat_monitor_ns = (unsigned int) (UInt8)5;
Cb44_oShutoff = 0;
Cb44_oAlarm = 0;
Cb44_StateCnt = 0;
}
else {
Cb44_StateCnt = Cb44_StateCnt + 1;
}


break;
}
case (UInt8)5: {

Cb44_Pic_etat_monitor_node_fcn1();
if (SIBFS_Pic_etat_monitor_b.Cb44_glflag <= 2) {
Cb44_StateCnt = Cb44_StateCnt + 1;
}


break;
}
case (UInt8)3: {

Cb44_Pic_etat_monitor_node_fcn1();


break;
}
case (UInt8)6: {

if (Cb44_StateCnt > 15) {

SIBFS_Pic_etat_monitor_b.Cb44_Pic_etat_monitor_ns = (unsigned int) (UInt8)5;
Cb44_oShutoff = 0;
Cb44_oAlarm = 0;
Cb44_StateCnt = 0;
}
else {


if (bool_mat_pic_etat) {

SIBFS_Pic_etat_monitor_b.Cb44_Pic_etat_monitor_ns = (unsigned int) (UInt8)1;
Cb44_StateCnt = 0;
Cb44_oShutoff = 1;
Cb44_oAlarm = 1;
}
else {
Cb44_StateCnt = Cb44_StateCnt + 1;
}
}


break;
}
case (UInt8)2: {




if (!(bool_mat_pic_etat)) {

SIBFS_Pic_etat_monitor_b.Cb44_Pic_etat_monitor_ns = (unsigned int) (UInt8)3;
Cb44_oShutoff = 0;
Cb44_oAlarm = 0;
}
else {
if (Cb44_StateCnt) {

SIBFS_Pic_etat_monitor_b.Cb44_Pic_etat_monitor_ns = (unsigned int) (UInt8)1;
Cb44_StateCnt = 0;
Cb44_oShutoff = 1;
Cb44_oAlarm = 1;
}
else {
Cb44_StateCnt = Cb44_StateCnt + 1;
}
}


break;
}
case (UInt8)1: {




if (!(bool_mat_pic_etat)) {


SIBFS_Pic_etat_monitor_b.Cb44_Pic_etat_monitor_ns = (unsigned int) (UInt8)6;
Cb44_StateCnt = 0;
}


break;
}
default: {

if (!(SIBFS_Pic_etat_monitor_b.Cb44_Pic_etat_monitor)) {
SIBFS_Pic_etat_monitor_b.Cb44_Pic_etat_monitor = 1;



if (ui8_ResetMatlab != 0) {

SIBFS_Pic_etat_monitor_b.Cb44_Pic_etat_monitor_ns = (unsigned int) (UInt8)4;
Cb44_StateCnt = 0;
}
}
}
}




bool_PIC_Alarm = Cb44_oAlarm;



Cb34_idPWM = 50;




if (SIBFS_PWM_Detection_b.Cb35_PWMinput_handling) {

switch (SIBFS_PWM_Detection_b.Cb35_PWMinput_handling_ns) {
case (UInt8)3: {

# 552
Cb34_PWM_Detection_node_fcn1();


break;
}
case (UInt8)8: {

# 560
Cb34_PWM_Detection_node_fcn1();

# 564
break;
}
case (UInt8)4: {

# 569
Cb34_PWM_Detection_node_fcn1();

# 573
break;
}
case (UInt8)6: {

Cb34_PWM_Detection_node_fcn1();


break;
}
case (UInt8)7: {

# 585
Cb34_PWM_Detection_node_fcn1();

# 589
break;
}
case (UInt8)2: {

Cb34_PWM_Detection_node_fcn1();


break;
}
case (UInt8)5: {

# 601
Cb34_PWM_Detection_node_fcn1();

# 605
break;
}
case (UInt8)1: {

Cb34_PWM_Detection_node_fcn1();


break;
}
}


}
else {
SIBFS_PWM_Detection_b.Cb35_PWMinput_handling = 1;


if (ui8_ResetMatlab == 1) {

SIBFS_PWM_Detection_b.Cb35_PWMinput_handling_ns = (unsigned int) (UInt8)1;
Cb34_odPumpOff = 1;
Cb34_odFixedValueSel = 0;
Cb34_odFixedLowValueSel = 0;
Cb34_oPWM_SC_Alarm = 0;
Cb34_oPWM_Alarm = 0;
}
}




bool_PWMin_Freq_err_Alarm = Cb34_oPWM_SC_Alarm;




if (SIBFS_UbatHandling_b.Cb57_Ubat_Handling) {



if (SIBFS_UbatHandling_b.Cb58_SaturationHigh) {

SIBFS_UbatHandling_b.Aux_sflag3 = 2 ;


if (((UInt8)ui8_BattVolt_mat) < 139) {
Cb56_UbatHandling_node_fcn2();
}


}
else {

if (SIBFS_UbatHandling_b.Cb59_SaturationLow) {

SIBFS_UbatHandling_b.Aux_sflag3 = 3 ;


if (((UInt8)ui8_BattVolt_mat) > 49) {
Cb56_UbatHandling_node_fcn2();
}


}
else {

if (SIBFS_UbatHandling_b.Cb60_NormalUbat) {

SIBFS_UbatHandling_b.Aux_sflag3 = 1 ;
Cb56_UbatHandling_node_fcn2();


}
}
}


}
else {
SIBFS_UbatHandling_b.Cb57_Ubat_Handling = 1;


if (ui8_ResetMatlab == 1) {

SIBFS_UbatHandling_b.Cb60_NormalUbat = 1;
Cb56_odPumpOff = 0;
Cb56_odFixedValueSel = 0;
Cb56_oUbat_Alarm_High = 0;
}
}


Sb1_Logical_Operator2 = 0;




if (SIBFS_Temperature_Alarm_b.Cb52_CntOverTemp) {

if (Cb51_Counter > 80) {

# 707
SIBFS_Temperature_Alarm_b.Cb52_CntOverTemp = 0;
SIBFS_Temperature_Alarm_b.Cb54_greenTemp = 1;
Cb51_oTempRedAlarm = 0;
Cb51_oTempAlarm = 0;
Cb51_odPumpOff = 0;
Cb51_Counter = 0 ;
}
else {
Cb51_Counter = Cb51_Counter + 1 ;
}


}
else {

if (SIBFS_Temperature_Alarm_b.Cb53_reset) {

# 727
SIBFS_Temperature_Alarm_b.Cb53_reset = 0;
SIBFS_Temperature_Alarm_b.Cb52_CntOverTemp = 1;


}
else {

if (SIBFS_Temperature_Alarm_b.Cb54_greenTemp) {



if (ui16_mat_inpTemp < 72) {

# 741
SIBFS_Temperature_Alarm_b.Cb54_greenTemp = 0;
SIBFS_Temperature_Alarm_b.Cb55_redTemp = 1;
Cb51_oTempRedAlarm = 1;
Cb51_oTempAlarm = 1;


Cb51_odPumpOff = 0;
}


}
else {

if (SIBFS_Temperature_Alarm_b.Cb55_redTemp) {



if (ui16_mat_inpTemp > 185) {

# 761
SIBFS_Temperature_Alarm_b.Cb55_redTemp = 0;
SIBFS_Temperature_Alarm_b.Cb54_greenTemp = 1;
Cb51_oTempRedAlarm = 0;
Cb51_oTempAlarm = 0;
Cb51_odPumpOff = 0;
Cb51_Counter = 0 ;
}


}
else {

if (!(SIBFS_Temperature_Alarm_b.Cb51_Temperature_Alarm)) {
SIBFS_Temperature_Alarm_b.Cb51_Temperature_Alarm = 1;

# 778
if (ui8_ResetMatlab != 0) {

SIBFS_Temperature_Alarm_b.Cb53_reset = 1;
Cb51_Counter = 0 ;
}
}
}
}
}
}




switch (SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High_ns) {
case (UInt8)6: {

if (Cb1_StateCnt > 50) {

# 798
SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High_ns = (unsigned int)
(UInt8)5;
Cb1_oShutoff = 0;
Cb1_oCurrentAlarm = 0;
Cb1_RestartCounter = 0 ;
}
else {
Cb1_StateCnt = Cb1_StateCnt + 1 ;
}


break;
}
case (UInt8)3: {

Cb1_Current_An___High_node_fcn1();
if (SIBFS_Current_Analysis_High_b.Cb1_glflag <= 2) {
Cb1_StateCnt = Cb1_StateCnt + 1 ;
}


break;
}
case (UInt8)7: {

if (Cb1_RestartCounter < 10) {

# 826
SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High_ns = (unsigned int)
(UInt8)3;
Cb1_oShutoff = 0;
Cb1_oCurrentAlarm = 1;
Cb1_StateCnt = 0 ;
}


break;
}
case (UInt8)5: {

Cb1_Current_An___High_node_fcn1();


break;
}
case (UInt8)4: {

if (Cb1_StateCnt > 200) {

SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High_ns = (unsigned int)
(UInt8)7;
Cb1_RestartCounter = Cb1_RestartCounter + 1 ;
}
else {

# 854
if (bool_mat_currAlarm_bldc) {

SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High_ns = (unsigned int)
(UInt8)1;
Cb1_StateCnt = 0 ;

Cb1_oCurrentAlarm = 1;
}
else {
Cb1_StateCnt = Cb1_StateCnt + 1 ;
}
}


break;
}
case (UInt8)2: {

# 875
if (!(bool_mat_currAlarm_bldc)) {

SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High_ns = (unsigned int)
(UInt8)3;
Cb1_oShutoff = 0;
Cb1_oCurrentAlarm = 1;
Cb1_StateCnt = 0 ;
}
else {
if (Cb1_StateCnt > 1) {

SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High_ns = (unsigned int)
(UInt8)1;
Cb1_StateCnt = 0 ;

Cb1_oCurrentAlarm = 1;
}
else {
Cb1_StateCnt = Cb1_StateCnt + 1 ;
}
}


break;
}
case (UInt8)1: {

# 905
if (!(bool_mat_currAlarm_bldc)) {

# 908
SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High_ns = (unsigned int) (UInt8)4;
Cb1_StateCnt = 0 ;
}
else {
Cb1_StateCnt = Cb1_StateCnt + 1 ;
}


break;
}
default: {

if (!(SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High)) {
SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High = 1;

# 925
if (ui8_ResetMatlab != 0) {

SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High_ns = (unsigned int)
(UInt8)6;
Cb1_StateCnt = 0 ;
Cb1_RestartCounter = 0 ;
}
}
}
}


Sb1_Logical_Operator1 = Sb1_Logical_Operator2 || Cb56_odPumpOff || Cb51_odPumpOff ||
Cb1_oShutoff || Cb44_oShutoff;


Cb24_Reset = ui8_ResetMatlab != 0 ;




if (SIBFS_Motor_Stalled_b.Cb25_Motor_stalled_Statemachine) {

# 952
if (Sb1_Logical_Operator1 || Cb24_Reset) {

# 955
switch (SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns) {
case (UInt8)3: {
Cb24_StateCnt = 0 ;
SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns = 0 ;
break;
}
case (UInt8)2: {
SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns = 0 ;
break;
}
case (UInt8)1: {
SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns = 0 ;
break;
}
case (UInt8)5: {
SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns = 0 ;
break;
}
case (UInt8)6: {
SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns = 0 ;
break;
}
case (UInt8)4: {
SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns = 0 ;
break;
}
}
SIBFS_Motor_Stalled_b.Cb25_Motor_stalled_Statemachine = 0;
SIBFS_Motor_Stalled_b.Cb32_default = 1;
Cb24_oMotorStalled = 0;
Cb24_oStalledAlarm = 0;
Cb24_RestartCounter = 0 ;
}
else {
switch (SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns) {
case (UInt8)3: {

# 993
if (Cb24_StateCnt) {

# 997
Cb24_StateCnt = 0 ;
SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns = (unsigned int)
(UInt8)2;
}
else {
Cb24_StateCnt = Cb24_StateCnt + 1 ;
}

# 1007
break;
}
case (UInt8)2: {

# 1012
if (Cb24_StateCnt > 1000) {

# 1015
SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns = (unsigned int)
(UInt8)5;
Cb24_StateCnt = 0 ;
Cb24_BadCnt = 0 ;
Cb24_oMotorStalled = 0;


Cb24_oStalledAlarm = 0;
}
else {

# 1027
if ((ui16_Speed_mat < 5) || (ui16_Speed_mat > 300)) {

# 1030
SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns = (unsigned int)
(UInt8)1;
Cb24_oStalledAlarm = 0;
Cb24_StateCnt = 0 ;
}
else {
Cb24_StateCnt = Cb24_StateCnt + 1 ;
}
}

# 1042
break;
}
case (UInt8)1: {

# 1047
if (Cb24_BadCnt > 100) {

# 1050
SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns = (unsigned int)
(UInt8)4;
Cb24_StateCnt = 0 ;
Cb24_oMotorStalled = 1;
Cb24_oStalledAlarm = 1;
}
else {

# 1059
if ((ui16_Speed_mat >= 5) && (ui16_Speed_mat <= 300)) {

# 1062
SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns = (unsigned int)
(UInt8)2;
}
else {
Cb24_BadCnt = Cb24_BadCnt + 1 ;
Cb24_StateCnt = Cb24_StateCnt + 1 ;
}
}

# 1073
break;
}
case (UInt8)5: {

# 1082
SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns = (unsigned int)
(UInt8)2;

# 1087
break;
}
case (UInt8)6: {

# 1092
if (Cb24_StateCnt > 10) {
if (Cb24_RestartCounter == 10) {

SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns = 0 ;
SIBFS_Motor_Stalled_b.Cb25_Motor_stalled_Statemachine = 0;
SIBFS_Motor_Stalled_b.Cb33_Stop = 1;
Cb24_oStalledAlarm = 1;
Cb24_oMotorStalled = 1;
SIBFS_Motor_Stalled_b.Cb24_glflag = 3 ;
}
else {

# 1105
if ((ui16_Speed_mat < 5) || (ui16_Speed_mat > 300)) {

# 1108
SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns = (unsigned int)
(UInt8)4;
Cb24_StateCnt = 0 ;
Cb24_oMotorStalled = 1;
Cb24_oStalledAlarm = 1;
SIBFS_Motor_Stalled_b.Cb24_glflag = 3 ;
}
else {
if (Cb24_StateCnt > 200) {

# 1119
SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns = (unsigned int)
(UInt8)5;
Cb24_StateCnt = 0 ;
Cb24_BadCnt = 0 ;
Cb24_oMotorStalled = 0;


Cb24_oStalledAlarm = 0;
SIBFS_Motor_Stalled_b.Cb24_glflag = 3 ;
}
else {
SIBFS_Motor_Stalled_b.Cb24_glflag = 1 ;
}
}
}
}
else {
SIBFS_Motor_Stalled_b.Cb24_glflag = 0 ;
}
if (SIBFS_Motor_Stalled_b.Cb24_glflag <= 2) {
Cb24_StateCnt = Cb24_StateCnt + 1 ;
}

# 1144
break;
}
case (UInt8)4: {

# 1149
if (Cb24_StateCnt > 200) {

# 1153
SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns = (unsigned int)
(UInt8)6;
Cb24_StateCnt = 0 ;
Cb24_oMotorStalled = 0;
Cb24_oStalledAlarm = 1;
Cb24_RestartCounter = Cb24_RestartCounter + 1 ;
}
else {
Cb24_StateCnt = Cb24_StateCnt + 1 ;
}

# 1166
break;
}
}
}


}
else {

if (SIBFS_Motor_Stalled_b.Cb32_default) {

# 1180
if (!(Sb1_Logical_Operator1)) {

# 1183
SIBFS_Motor_Stalled_b.Cb32_default = 0;
SIBFS_Motor_Stalled_b.Cb25_Motor_stalled_Statemachine = 1;
SIBFS_Motor_Stalled_b.Cb25_Motor_sta__Statemachine_ns = (unsigned int) (UInt8)3;
Cb24_StateCnt = 0 ;
Cb24_BadCnt = 0 ;
}


}
else {

if (!(SIBFS_Motor_Stalled_b.Cb33_Stop)) {

if (!(SIBFS_Motor_Stalled_b.Cb24_Motor_Stalled)) {
SIBFS_Motor_Stalled_b.Cb24_Motor_Stalled = 1;


if (Cb24_Reset) {

SIBFS_Motor_Stalled_b.Cb32_default = 1;
Cb24_oMotorStalled = 0;
Cb24_oStalledAlarm = 0;
Cb24_RestartCounter = 0 ;
}
}
}
}
}




bool_StalledMotorStop = Cb24_oMotorStalled;
Sb1_Logical_Operator5 = Cb34_odFixedValueSel || Cb56_odFixedValueSel;

# 1220
if (bool_start_demand_mat) {
Sb2_Switch5 = ui8_fixed_start_speed_mat;
}
else {

# 1226
if (Cb34_odFixedLowValueSel) {
Sb2_Switch5 = ui16_Speed_demand_mat_min;
}
else {

# 1232
if (Sb1_Logical_Operator5) {
Sb2_Switch5 = ui16_Speed_demand_mat_Max;
}
else {
Sb2_Switch5 = ui16_Speed_demand_mat;
}
}
}


Sb2_Error = (Int16) (ui16_Speed_mat - Sb2_Switch5);




if (SIBFS_Dry_Running_b.Cb15_greenState) {

# 1254
if ((((Int32)ui16_mat_Current) < (ui16_dryRun_Thresh - 5)) && (ui16_Speed_mat < 400) &&
(ui16_Speed_mat > 80)) {

# 1263
}


}
else {

if (SIBFS_Dry_Running_b.Cb16_DryRunning) {



if (!(SIBFS_Dry_Running_b.Cb17_redState)) {

if (SIBFS_Dry_Running_b.Cb18_CntOverCurrent) {

# 1282
if ((((UInt32)ui16_mat_Current) > (ui16_dryRun_Thresh + 5)) || (ui16_Speed_mat >
400) || (ui16_Speed_mat < 80)) {

SIBFS_Dry_Running_b.Cb18_CntOverCurrent = 0;
SIBFS_Dry_Running_b.Cb16_DryRunning = 0;
SIBFS_Dry_Running_b.Cb15_greenState = 1;
Cb14_oDryRunAlarm = 0;
}
else {
if ((ui8_PWM_dc_mat < 20) && (ui8_PWM_dc_mat > 4)) {

SIBFS_Dry_Running_b.Cb18_CntOverCurrent = 0;
SIBFS_Dry_Running_b.Cb16_DryRunning = 0;
SIBFS_Dry_Running_b.Cb15_greenState = 1;
Cb14_oDryRunAlarm = 0;
}
else {
if (Cb14_StateCnt > 2000) {

SIBFS_Dry_Running_b.Cb18_CntOverCurrent = 0;
SIBFS_Dry_Running_b.Cb17_redState = 1;
Cb14_oDryRunAlarm = 1;
}
else {
Cb14_StateCnt = Cb14_StateCnt + 1 ;
}
}
}


}
}


}
else {

if (!(SIBFS_Dry_Running_b.Cb14_Dry_Running)) {
SIBFS_Dry_Running_b.Cb14_Dry_Running = 1;


if (ui8_ResetMatlab == 1) {

SIBFS_Dry_Running_b.Cb15_greenState = 1;
Cb14_oDryRunAlarm = 0;
}
}
}
}

# 1337
if (SIBFS_Dry_RunningAlarm_b.Cb20_greenState) {

# 1342
if (Cb14_oDryRunAlarm) {

# 1345
SIBFS_Dry_RunningAlarm_b.Cb20_greenState = 0;
SIBFS_Dry_RunningAlarm_b.Cb21_DryRunningAlarm = 1;
SIBFS_Dry_RunningAlarm_b.Cb22_DryRun66 = 1;
Cb19_Counter = 0 ;
Cb19_oDryRun66 = 1;
Cb19_oDryRun55 = 0;
}


}
else {

if (SIBFS_Dry_RunningAlarm_b.Cb21_DryRunningAlarm) {

# 1362
if (!(Cb14_oDryRunAlarm)) {

# 1367
if (SIBFS_Dry_RunningAlarm_b.Cb22_DryRun66) {
SIBFS_Dry_RunningAlarm_b.Cb22_DryRun66 = 0;
}
else {

if (SIBFS_Dry_RunningAlarm_b.Cb23_DryRun55) {
SIBFS_Dry_RunningAlarm_b.Cb23_DryRun55 = 0;
}
}
SIBFS_Dry_RunningAlarm_b.Cb21_DryRunningAlarm = 0;
SIBFS_Dry_RunningAlarm_b.Cb20_greenState = 1;
Cb19_oDryRun55 = 0;
Cb19_oDryRun66 = 0;
Cb19_Counter = 0 ;
}
else {

if (SIBFS_Dry_RunningAlarm_b.Cb22_DryRun66) {

# 1387
if (Cb19_Counter > 400) {

# 1390
SIBFS_Dry_RunningAlarm_b.Cb22_DryRun66 = 0;
SIBFS_Dry_RunningAlarm_b.Cb23_DryRun55 = 1;
Cb19_Counter = 0 ;
Cb19_oDryRun66 = 0;
Cb19_oDryRun55 = 1;
}
else {
Cb19_Counter = Cb19_Counter + 1 ;
}

# 1402
}
else {

if (SIBFS_Dry_RunningAlarm_b.Cb23_DryRun55) {

# 1408
if (Cb19_Counter > 400) {

# 1411
SIBFS_Dry_RunningAlarm_b.Cb23_DryRun55 = 0;
SIBFS_Dry_RunningAlarm_b.Cb22_DryRun66 = 1;
Cb19_Counter = 0 ;
Cb19_oDryRun66 = 1;
Cb19_oDryRun55 = 0;
}
else {
Cb19_Counter = Cb19_Counter + 1 ;
}

# 1423
}
}
}


}
else {

if (!(SIBFS_Dry_RunningAlarm_b.Cb19_Dry_RunningAlarm)) {
SIBFS_Dry_RunningAlarm_b.Cb19_Dry_RunningAlarm = 1;

# 1436
if (!(Cb14_oDryRunAlarm)) {

SIBFS_Dry_RunningAlarm_b.Cb20_greenState = 1;
Cb19_oDryRun55 = 0;
Cb19_oDryRun66 = 0;
Cb19_Counter = 0 ;
}
}
}
}




bool_DryRunningAlarm = Cb19_oDryRun66;


bool_CPU_TempAlarm = Cb51_oTempAlarm;




if (SIBFS_Current_Analysis_low_b.Cb10_greenState) {

# 1463
if (((Int32)ui16_mat_Current) > (ui16_Current_Thresh - 2)) {

# 1466
SIBFS_Current_Analysis_low_b.Cb10_greenState = 0;
SIBFS_Current_Analysis_low_b.Cb12_CntOverCurrent = 1;
Cb9_StateCnt = 0 ;
}


}
else {

if (SIBFS_Current_Analysis_low_b.Cb11_Wait) {

# 1480
if (((Int32)ui16_mat_Current) > (ui16_Current_Thresh - 2)) {

SIBFS_Current_Analysis_low_b.Cb11_Wait = 0;
SIBFS_Current_Analysis_low_b.Cb13_redState = 1;
Cb9_oCurrentAlarm = 1;
}
else {
if (Cb9_StateCnt > 100) {

SIBFS_Current_Analysis_low_b.Cb11_Wait = 0;
SIBFS_Current_Analysis_low_b.Cb10_greenState = 1;
Cb9_oCurrentAlarm = 0;
}
else {
Cb9_StateCnt = Cb9_StateCnt + 1 ;
}
}


}
else {

if (SIBFS_Current_Analysis_low_b.Cb12_CntOverCurrent) {

# 1507
if (((UInt32)ui16_mat_Current) < (ui16_Current_Thresh + 2)) {

SIBFS_Current_Analysis_low_b.Cb12_CntOverCurrent = 0;
SIBFS_Current_Analysis_low_b.Cb10_greenState = 1;
Cb9_oCurrentAlarm = 0;
}
else {
if (Cb9_StateCnt > 50) {

SIBFS_Current_Analysis_low_b.Cb12_CntOverCurrent = 0;
SIBFS_Current_Analysis_low_b.Cb13_redState = 1;
Cb9_oCurrentAlarm = 1;
}
else {
Cb9_StateCnt = Cb9_StateCnt + 1 ;
}
}


}
else {

if (SIBFS_Current_Analysis_low_b.Cb13_redState) {

# 1534
if (((UInt32)ui16_mat_Current) < (ui16_Current_Thresh + 2)) {

# 1537
SIBFS_Current_Analysis_low_b.Cb13_redState = 0;
SIBFS_Current_Analysis_low_b.Cb11_Wait = 1;
Cb9_StateCnt = 0 ;
}


}
else {

if (!(SIBFS_Current_Analysis_low_b.Cb9_Current_Analysis_low)) {
SIBFS_Current_Analysis_low_b.Cb9_Current_Analysis_low = 1;


if (ui8_ResetMatlab == 1) {

SIBFS_Current_Analysis_low_b.Cb10_greenState = 1;
Cb9_oCurrentAlarm = 0;
}
}
}
}
}
}




bool_HighCurrentAlarm = Cb9_oCurrentAlarm;


bool_PWMin_err_Alarm = Cb34_oPWM_Alarm;


bool_UbatAlarm = Cb56_oUbat_Alarm_High;
bool_MotorStalled = Cb24_oStalledAlarm || Cb1_oCurrentAlarm || Cb19_oDryRun55;


bool_CPU_TempRedAlarm = Cb51_oTempRedAlarm;


ui8_debug_out0 = (UInt8) Sb2_Switch5;
Sb1_Logical_Operator3 = Sb1_Logical_Operator1 || Cb24_oMotorStalled || Cb14_oDryRunAlarm;
Sb2_Logical_Operator2 = Sb1_Logical_Operator3 || bool_ControlLoopMode;
if ((Sb2_Logical_Operator2 ^ X_Sb4_Intergrator_TriggerIn) && (!(Sb1_BVH2_Appl_Layer_FirstRun)))
{
X_Sb4_Intergrator = 80000 ;
}

# 1587
if (Sb1_Logical_Operator3) {
Sb2_Switch2 = 0 ;
}
else {

# 1593
if (bool_ControlLoopMode) {

# 1596
if (Sb1_Logical_Operator5) {
Sb2_Switch2 = Sb2_Fixed_Power;
}
else {

UInt16 Sb3_Product1 ;
Int16 Sb3_Sum1;

# 1610
Sb3_Product1 = (UInt16) (((UInt16) (140 << 8)) / 160 );

# 1616
Sb3_Sum1 = (Int16) (((UInt16) ((((UInt32) ui8_PWM_dc_mat) * ((UInt32) Sb3_Product1)) >>
8)) + ((Int16) (((Int16) (-((Int16) (UInt16) ((((UInt32) Sb3_Product1) * 95) >> 7)))) + 200)));


Sb2_Switch2 = (UInt16) ( (Sb3_Sum1 > (Int16)200) ? 200 : ( (Sb3_Sum1 < (Int16)60) ? 60 : (UInt8)Sb3_Sum1 ) );
}
}
else {

Int16 Sb4_PI_sum;

# 1630
Sb4_PI_sum = (Int16) (((UInt16) (Int16) (X_Sb4_Intergrator / ((Int32) 800))) + ((UInt16)
(Sb2_Error * ((Int16) ui8_Kp_mat))));


Sb2_Switch2 = ( ( (Sb4_PI_sum > 0) && ((UInt16)Sb4_PI_sum > 202) ) ? 202 : ( ( (Sb4_PI_sum < 0) || ((UInt16)Sb4_PI_sum < 40) ) ? 40 : (UInt16)Sb4_PI_sum ) );
}
}

# 1664
if (Sb1_Logical_Operator3 || power_lockout){
ui16_duty_cycle_mat = 0;
power_lockout = 1;
}
else {
ui16_duty_cycle_mat = ui16_Speed_demand_mat;
}


bl_Pumpoff_Alarm = Sb1_Logical_Operator2;
X_Sb4_Intergrator_TriggerIn = Sb2_Logical_Operator2;

# 1678
X_Sb4_Intergrator = X_Sb4_Intergrator + ((Int32) (Int16) ((((Int16) ui8_Ki_mat) * Sb2_Error) <<
3));


X_Sb4_Intergrator = ( (X_Sb4_Intergrator > 161600) ? 161600 : ( (X_Sb4_Intergrator < 32000) ? 32000 : X_Sb4_Intergrator ) );


Sb1_BVH2_Appl_Layer_FirstRun = 0;
}

# 1709
static Void Cb1_Current_An___High_node_fcn1(Void)
{

# 1713
if (bool_mat_currAlarm_bldc) {

SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High_ns = (unsigned int)
(UInt8)2;
Cb1_StateCnt = 0 ;
SIBFS_Current_Analysis_High_b.Cb1_glflag = 3 ;
}
else {
if (Cb1_StateCnt > 100) {

SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High_ns = (unsigned int)
(UInt8)5;
Cb1_oShutoff = 0;
Cb1_oCurrentAlarm = 0;
Cb1_RestartCounter = 0 ;
SIBFS_Current_Analysis_High_b.Cb1_glflag = 3 ;
}
else {
SIBFS_Current_Analysis_High_b.Cb1_glflag = 1 ;
}
}
}

# 1753
static Void Cb34_PWM_Detection_node_fcn1(Void)
{
if ((Cb34_idPWM < 1) || (Cb34_idPWM > 199)) {
if (Cb34_idPWM > 199) {

SIBFS_PWM_Detection_b.Cb35_PWMinput_handling_ns = (unsigned int) (UInt8)7;
Cb34_odPumpOff = 0;
Cb34_odFixedValueSel = 1;
Cb34_odFixedLowValueSel = 0;
Cb34_oPWM_SC_Alarm = 0;
Cb34_oPWM_Alarm = 1;
}
}
else {

# 1769
if ((ui16_PWM_Freq_mat < 36000) || (ui16_PWM_Freq_mat > 44000)) {

SIBFS_PWM_Detection_b.Cb35_PWMinput_handling_ns = (unsigned int) (UInt8)6;
Cb34_odPumpOff = 0;
Cb34_odFixedValueSel = 1;
Cb34_odFixedLowValueSel = 0;
Cb34_oPWM_SC_Alarm = 1;
Cb34_oPWM_Alarm = 0;
}
else {
if (Cb34_idPWM <= 5) {

SIBFS_PWM_Detection_b.Cb35_PWMinput_handling_ns = (unsigned int) (UInt8)3;
Cb34_odPumpOff = 0;
Cb34_odFixedValueSel = 1;
Cb34_odFixedLowValueSel = 0;
Cb34_oPWM_SC_Alarm = 1;
Cb34_oPWM_Alarm = 0;
}
else {
if (Cb34_idPWM < 9) {

# 1792
SIBFS_PWM_Detection_b.Cb35_PWMinput_handling_ns = (unsigned int)
(UInt8)8;
Cb34_odPumpOff = 1;
Cb34_odFixedValueSel = 0;
Cb34_odFixedLowValueSel = 0;
Cb34_oPWM_SC_Alarm = 1;
Cb34_oPWM_Alarm = 0;
}
else {
if (Cb34_idPWM > 191) {

# 1804
SIBFS_PWM_Detection_b.Cb35_PWMinput_handling_ns = (unsigned int)
(UInt8)4;
Cb34_odPumpOff = 0;
Cb34_odFixedValueSel = 1;
Cb34_odFixedLowValueSel = 0;
Cb34_oPWM_SC_Alarm = 1;
Cb34_oPWM_Alarm = 0;
}
else {
if (Cb34_idPWM >= 23) {

# 1816
SIBFS_PWM_Detection_b.Cb35_PWMinput_handling_ns = (unsigned int)
(UInt8)2;
Cb34_odPumpOff = 0;
Cb34_odFixedLowValueSel = 0;
Cb34_odFixedValueSel = 0;
Cb34_oPWM_SC_Alarm = 0;
Cb34_oPWM_Alarm = 0;
}
else {
if (Cb34_idPWM > 19) {

# 1828
SIBFS_PWM_Detection_b.Cb35_PWMinput_handling_ns = (unsigned int)
(UInt8)5;
Cb34_odPumpOff = 0;
Cb34_odFixedValueSel = 0;
Cb34_odFixedLowValueSel = 1;
Cb34_oPWM_SC_Alarm = 0;
Cb34_oPWM_Alarm = 0;
}
else {
if (Cb34_idPWM <= 19) {

# 1840
SIBFS_PWM_Detection_b.Cb35_PWMinput_handling_ns = (unsigned int)
(UInt8)1;
Cb34_odPumpOff = 1;
Cb34_odFixedValueSel = 0;
Cb34_odFixedLowValueSel = 0;
Cb34_oPWM_SC_Alarm = 0;
Cb34_oPWM_Alarm = 0;
}
}
}
}
}
}
}
}
}

# 1874
static Void Cb44_Pic_etat_monitor_node_fcn1(Void)
{

# 1878
if (bool_mat_pic_etat) {

SIBFS_Pic_etat_monitor_b.Cb44_Pic_etat_monitor_ns = (unsigned int) (UInt8)2;
Cb44_StateCnt = 0 ;
SIBFS_Pic_etat_monitor_b.Cb44_glflag = 3 ;
}
else {
if (Cb44_StateCnt > 5) {

SIBFS_Pic_etat_monitor_b.Cb44_Pic_etat_monitor_ns = (unsigned int) (UInt8)3;
Cb44_oShutoff = 0;
Cb44_oAlarm = 0;
SIBFS_Pic_etat_monitor_b.Cb44_glflag = 3 ;
}
else {
SIBFS_Pic_etat_monitor_b.Cb44_glflag = 1 ;
}
}
}

# 1915
static Void Cb56_UbatHandling_node_fcn2(Void)
{

if (((UInt8)ui8_BattVolt_mat) > 147) {

switch (SIBFS_UbatHandling_b.Aux_sflag3) {
case 2: {
SIBFS_UbatHandling_b.Cb58_SaturationHigh = 0;
break;
}
case 3: {
SIBFS_UbatHandling_b.Cb59_SaturationLow = 0;
break;
}
default: {
SIBFS_UbatHandling_b.Cb60_NormalUbat = 0;
}
}
SIBFS_UbatHandling_b.Cb58_SaturationHigh = 1;
Cb56_odPumpOff = 1;
Cb56_odFixedValueSel = 0;
Cb56_oUbat_Alarm_High = 1;
}
else {
switch (SIBFS_UbatHandling_b.Aux_sflag3) {
case 2: {
SIBFS_UbatHandling_b.Cb58_SaturationHigh = 0;
break;
}
case 3: {
SIBFS_UbatHandling_b.Cb59_SaturationLow = 0;
break;
}
default: {
SIBFS_UbatHandling_b.Cb60_NormalUbat = 0;
}
}


if (((UInt8)ui8_BattVolt_mat) < 45) {

SIBFS_UbatHandling_b.Cb59_SaturationLow = 1;
Cb56_odPumpOff = 1;
Cb56_odFixedValueSel = 0;
Cb56_oUbat_Alarm_High = 1;
}
else {

SIBFS_UbatHandling_b.Cb60_NormalUbat = 1;
Cb56_odPumpOff = 0;
Cb56_odFixedValueSel = 0;
Cb56_oUbat_Alarm_High = 0;
}
}
}

