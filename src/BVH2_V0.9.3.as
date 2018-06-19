opt subtitle "HI-TECH Software Omniscient Code Generator (PRO mode) build 7612"

opt pagewidth 120

	opt pm

	processor	16F1936
clrc	macro
	bcf	3,0
	endm
clrz	macro
	bcf	3,2
	endm
setc	macro
	bsf	3,0
	endm
setz	macro
	bsf	3,2
	endm
skipc	macro
	btfss	3,0
	endm
skipz	macro
	btfss	3,2
	endm
skipnc	macro
	btfsc	3,0
	endm
skipnz	macro
	btfsc	3,2
	endm
indf	equ	0
indf0	equ	0
indf1	equ	1
pc	equ	2
pcl	equ	2
status	equ	3
fsr0l	equ	4
fsr0h	equ	5
fsr1l	equ	6
fsr1h	equ	7
bsr	equ	8
wreg	equ	9
intcon	equ	11
c	equ	1
z	equ	0
pclath	equ	10
# 92 "D:\Projekte\BVH2\BLDC\V0.9.3\src\main.c"
	psect config,class=CONFIG,delta=2 ;#
# 92 "D:\Projekte\BVH2\BLDC\V0.9.3\src\main.c"
	dw 0xFFFC & 0xFFFF & 0xFFBF & 0xFFFF ;#
# 103 "D:\Projekte\BVH2\BLDC\V0.9.3\src\main.c"
	psect config,class=CONFIG,delta=2 ;#
# 103 "D:\Projekte\BVH2\BLDC\V0.9.3\src\main.c"
	dw 0xFFFF & 0xDFFF & 0xFBFF ;#
	FNCALL	_main,_system_init
	FNCALL	_main,_clear_timer
	FNCALL	_main,___wmul
	FNCALL	_main,___awdiv
	FNCALL	_main,_Get_Analog_Value
	FNCALL	_main,_PWMReadDC
	FNCALL	_main,_BVH2_Appl_Layer
	FNCALL	_main,_InitMotorStop
	FNCALL	_main,_InitMotorRun
	FNCALL	_main,_SetDiagAlarm
	FNCALL	_main,_read_eeprom_data
	FNCALL	_main,_Receive_Diag
	FNCALL	_main,_EOL
	FNCALL	_system_init,_init_ports
	FNCALL	_system_init,_EnableMCP201
	FNCALL	_system_init,__ELINMIntInitialize
	FNCALL	_system_init,_Oscill_Source_Block
	FNCALL	_system_init,_timer_init
	FNCALL	_system_init,_PWM_Capture_init
	FNCALL	_system_init,_ADC_Init
	FNCALL	_system_init,_DiagInit
	FNCALL	_system_init,_FILTER_Init
	FNCALL	_system_init,_I_calibrationInit
	FNCALL	_system_init,_init_bldc
	FNCALL	_EOL,_Transmit_LIN_8Bytes
	FNCALL	_EOL,_read_eeprom_data
	FNCALL	_EOL,_NegativeAnswer
	FNCALL	_EOL,_cksum
	FNCALL	_EOL,_InitMotorStop
	FNCALL	_EOL,_ADC_Wait
	FNCALL	_EOL,_ADC_Read
	FNCALL	_EOL,___lwdiv
	FNCALL	_EOL,_write_eeprom_data
	FNCALL	_EOL,___wmul
	FNCALL	_EOL,_PWMReadDC
	FNCALL	_EOL,_PWM_Write_Out
	FNCALL	_Receive_Diag,__ELINMIntReceiveMessage
	FNCALL	_Receive_Diag,__ELINMIntGetPointer
	FNCALL	_NegativeAnswer,_Transmit_LIN_8Bytes
	FNCALL	_init_bldc,_InitMotorRun
	FNCALL	_BVH2_Appl_Layer,_Cb37_Pic_etat_monitor_node_fcn1
	FNCALL	_BVH2_Appl_Layer,_Cb27_PWM_Detection_node_fcn1
	FNCALL	_BVH2_Appl_Layer,_Cb53_UbatHandling_node_fcn2
	FNCALL	_BVH2_Appl_Layer,_Cb1_Current_An___High_node_fcn1
	FNCALL	_BVH2_Appl_Layer,___wmul
	FNCALL	_BVH2_Appl_Layer,___lmul
	FNCALL	_BVH2_Appl_Layer,___aldiv
	FNCALL	_I_calibrationInit,_read_eeprom_data
	FNCALL	_I_calibrationInit,_ADC_Wait
	FNCALL	_I_calibrationInit,_ADC_Read
	FNCALL	__ELINMIntReceiveMessage,__ELINMIntSendMessage
	FNCALL	_Transmit_LIN_8Bytes,__ELINMIntGetPointer
	FNCALL	_Transmit_LIN_8Bytes,__ELINMIntSendMessage
	FNCALL	_InitMotorRun,_commutate
	FNCALL	_FILTER_Init,_ADC_Wait
	FNCALL	_FILTER_Init,_ADC_Read
	FNCALL	_Cb53_UbatHandling_node_fcn2,_Cb53_UbatHandling_node_fcn1
	FNCALL	_timer_init,_clear_timer
	FNCALL	_init_ports,_read_eeprom_data
	FNCALL	__ELINMIntSendMessage,__ELINMIntCalcIDParity
	FNCALL	_PWMReadDC,___tmul
	FNCALL	_PWMReadDC,___ltdiv
	FNCALL	_SetDiagAlarm,_PWM_Write_Out
	FNCALL	_DiagInit,_PWM_Write_Out
	FNCALL	_commutate,___wmul
	FNCALL	_Get_Analog_Value,_FILTER_Ubat
	FNCALL	_Get_Analog_Value,_FILTER_IPhase
	FNCALL	_Get_Analog_Value,_FILTER_Temp
	FNROOT	_main
	FNCALL	_interrupt_handler,_PWM_CTRL
	FNCALL	_interrupt_handler,_Task1ms
	FNCALL	_interrupt_handler,_interrrupt_bldc
	FNCALL	_interrupt_handler,_ELINMIntHandler
	FNCALL	_interrupt_handler,_interrupt_PWMCapture
	FNCALL	_interrrupt_bldc,i1_commutate
	FNCALL	_interrrupt_bldc,_BLDCWait
	FNCALL	_interrrupt_bldc,i1_ADC_Wait
	FNCALL	_interrrupt_bldc,i1_ADC_Read
	FNCALL	_interrrupt_bldc,i1___wmul
	FNCALL	_interrrupt_bldc,i1___lwdiv
	FNCALL	i1_commutate,i1___wmul
	FNCALL	_ELINMIntHandler,__ELINMIntResetProtocol
	FNCALL	intlevel1,_interrupt_handler
	global	intlevel1
	FNROOT	intlevel1
	global	_ui8_selected_lid
	global	_ui16_I_cal_Ph1
	global	_ui16_I_cal_Ph2
	global	_ui16_I_cal_Ph3
	global	_ict_stamp
	global	_ui8_failure
	global	BVH2_Appl_Layer@X_Sb4_Intergrator
	global	BVH2_Appl_Layer@Sb1_BVH2_Appl_Layer_FirstRun
	global	BVH2_Appl_Layer@X_Sb4_Intergrator_TriggerIn
	global	_wkpoint
psect	idataBANK0,class=CODE,space=0,delta=2
global __pidataBANK0
__pidataBANK0:
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\main.c"
	line	122

;initializer for _ui8_selected_lid
	retlw	080h
psect	idataBANK2,class=CODE,space=0,delta=2
global __pidataBANK2
__pidataBANK2:
	line	116

;initializer for _ui16_I_cal_Ph1
	retlw	01h
	retlw	0

	line	117

;initializer for _ui16_I_cal_Ph2
	retlw	01h
	retlw	0

	line	118

;initializer for _ui16_I_cal_Ph3
	retlw	01h
	retlw	0

	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
	line	100

;initializer for _ict_stamp
	retlw	01h
	line	103

;initializer for _ui8_failure
	retlw	07h
psect	idataBANK3,class=CODE,space=0,delta=2
global __pidataBANK3
__pidataBANK3:
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\T_Link\BVH2_Appl_Layer.c"
	line	409

;initializer for BVH2_Appl_Layer@X_Sb4_Intergrator
	retlw	080h
	retlw	038h
	retlw	01h
	retlw	0

	line	411

;initializer for BVH2_Appl_Layer@Sb1_BVH2_Appl_Layer_FirstRun
	retlw	01h
	line	412

;initializer for BVH2_Appl_Layer@X_Sb4_Intergrator_TriggerIn
	retlw	01h
psect	idataBANK1,class=CODE,space=0,delta=2
global __pidataBANK1
__pidataBANK1:
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
	line	101

;initializer for _wkpoint
	retlw	01h
	global	_PWM_trans_table
psect	strings,class=STRING,delta=2
global __pstrings
__pstrings:
	global    __stringtab
__stringtab:
	retlw	0
psect	strings
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\config.h"
	line	255
_PWM_trans_table:
	retlw	06Eh
	retlw	06Eh
	retlw	06Eh
	retlw	0
	retlw	0
	retlw	0
	retlw	0
	retlw	0
	retlw	0
	retlw	0
	retlw	0D3h
	retlw	0D3h
	retlw	0D3h
	retlw	0D0h
	retlw	0CEh
	retlw	0CCh
	retlw	0C9h
	retlw	0C7h
	retlw	0C5h
	retlw	0C3h
	retlw	0C1h
	retlw	0BFh
	retlw	0BDh
	retlw	0BBh
	retlw	0BAh
	retlw	0B8h
	retlw	0B6h
	retlw	0B4h
	retlw	0B2h
	retlw	0B1h
	retlw	0AFh
	retlw	0ADh
	retlw	0ACh
	retlw	0AAh
	retlw	0A9h
	retlw	0A7h
	retlw	0A6h
	retlw	0A4h
	retlw	0A3h
	retlw	0A2h
	retlw	0A0h
	retlw	09Fh
	retlw	09Dh
	retlw	09Ch
	retlw	09Bh
	retlw	09Ah
	retlw	098h
	retlw	097h
	retlw	096h
	retlw	095h
	retlw	094h
	retlw	092h
	retlw	091h
	retlw	090h
	retlw	08Fh
	retlw	08Eh
	retlw	08Dh
	retlw	08Ch
	retlw	08Bh
	retlw	08Ah
	retlw	089h
	retlw	088h
	retlw	087h
	retlw	086h
	retlw	085h
	retlw	084h
	retlw	083h
	retlw	082h
	retlw	081h
	retlw	080h
	retlw	07Fh
	retlw	07Fh
	retlw	07Eh
	retlw	07Dh
	retlw	07Ch
	retlw	07Bh
	retlw	07Ah
	retlw	07Ah
	retlw	079h
	retlw	078h
	retlw	077h
	retlw	077h
	retlw	076h
	retlw	075h
	retlw	074h
	retlw	074h
	retlw	073h
	retlw	072h
	retlw	072h
	retlw	071h
	retlw	070h
	retlw	070h
	retlw	06Fh
	retlw	06Eh
	retlw	06Eh
	retlw	06Eh
	retlw	06Eh
	retlw	06Eh
	retlw	06Eh
	retlw	06Eh
	retlw	06Eh
	global	_PWM_trans_table
	global	_analog_value
	global	_comm_time
	global	_ui16_IPhase_bldc
	global	_MotorFlags
	global	__ELINMIntStatus
	global	_comm_state
	global	_pt
	global	_ui8_b_DResB0_c
	global	_ui8_b_DResLocID_c
	global	_ui8_duty_cycle_BLDC
	global	_rising_bemf_flag
	global	BVH2_Appl_Layer@Cb18_StateCnt
	global	_Cb1_StateCnt
	global	_Cb37_StateCnt
	global	_ui16_Capt_Val0
	global	_ui16_Capt_Val1
	global	_ui16_Capt_Val2
	global	_ui16_Ubemf_bldc
	global	_ui16_comm_time_max
	global	_ui16_phase_advancement
	global	_ui16_speed_fil
	global	BVH2_Appl_Layer@Cb8_StateCnt
	global	BVH2_Appl_Layer@SIBFS_Temperature_Alarmo_old_b
	global	_Bcnt
	global	_Cb1_oCurrentAlarm
	global	_Cb1_oShutoff
	global	_Cb27_oPWM_Alarm
	global	_Cb53_odFixedValueSel
	global	_ErrorCode
	global	_SIBFS_Current_Analysis_High_b
	global	_SIBFS_PWM_Detection_b
	global	_SIBFS_Pic_etat_monitor_b
	global	__ELINMIntMessageSize
	global	__ELINMIntRXMessageSize
	global	__ELINMIntReadBack
	global	__ELINMIntSpace
	global	_bool_CPU_TempAlarm
	global	_bool_CPU_TempRedAlarm
	global	_bool_ControlLoopMode
	global	_bool_DryRunningAlarm
	global	_bool_HighCurrentAlarm
	global	_bool_MotorStalled
	global	_bool_PWMin_Freq_err_Alarm
	global	_bool_PWMin_err_Alarm
	global	_bool_UbatAlarm
	global	_bool_mat_currAlarm_bldc
	global	_bool_start_demand_mat
	global	_ui8_BlankingCount
	global	_ui8_CompFlag
	global	_ui8_Duty_Cycle_In_Ratio
	global	_ui8_IPhase_sel
	global	_ui8_Kp_mat
	global	_ui8_PWM_FreqCnt
	global	_ui8_PWMinDC_sav
	global	_ui8_PWMin_failCnt
	global	_ui8_PWMoutvalue
	global	_ui8_Pulse_State
	global	_ui8_StartupPWM
	global	_ui8_Task_Cont1ms
	global	_ui8_Task_Cont3ms
	global	_ui8_Task_Cont5ms
	global	_ui8_UPhase_sel
	global	_ui8_Ubemf_sel
	global	_ui8_b_DResB1_c
	global	_ui8_b_DResB2_c
	global	_ui8_b_DResB3_c
	global	_ui8_b_DResB4_c
	global	_ui8_b_DResServID_c
	global	_ui8_duty_cycle_mat
	global	_ui8_given_supply
	global	_ui8_lastTaskvalue
	global	_ui8_sampleState
	global	_B
	global	_ui8_current_cal
	global	__ELINMIntTFrameMin
	global	__ELINMIntTHeaderMin
	global	_filter1
	global	_filter3
	global	_filterTempNTC
	global	_sum
	global	_ui16_IPhase1_bldc
	global	_ui16_NTC_Temp_bldc_mean
	global	_ui16_NTC_Temp_bldc_mean_cal
	global	_ui16_PWM_Freq_In
	global	_ui16_PWM_Freq_mat
	global	_ui16_fir_Bat_mittel
	global	_ui16_fir_IPhase_mean
	global	_ui16_mat_inpTemp
	global	_ui16_speed_rar
	global	BVH2_Appl_Layer@Cb13_oCurrentAlarm
	global	BVH2_Appl_Layer@Cb18_oMotorStalled
	global	BVH2_Appl_Layer@Cb18_oStalledAlarm
	global	BVH2_Appl_Layer@Cb44_Counter
	global	BVH2_Appl_Layer@Cb44_oTempAlarm
	global	BVH2_Appl_Layer@Cb44_oTempRedAlarm
	global	BVH2_Appl_Layer@Cb44_odPumpOff
	global	BVH2_Appl_Layer@Cb8_oCurrentAlarm
	global	_Cb27_oPWM_SC_Alarm
	global	_Cb27_odFixedLowValueSel
	global	_Cb27_odFixedValueSel
	global	_Cb27_odPumpOff
	global	_Cb37_oShutoff
	global	_Cb53_oUbat_Alarm_High
	global	_Cb53_odPumpOff
	global	_inputArray1
	global	_my_msg
	global	_checksum
	global	__ELINMIntSleepTimeout
	global	BVH2_Appl_Layer@Cb13_StateCnt
	global	BVH2_Appl_Layer@Cb18_BadCnt
	global	__ELINMIntRXCRC
	global	__ELINMIntTFrameMax
	global	__ELINMIntTHeaderMax
	global	_phase_delay_counter
	global	_ui16_IPhase2_bldc
	global	_ui16_IPhase3_bldc
	global	_ui16_NTC_Temp_bldc
	global	_ui16_Speed_demand_mat
	global	_ui16_Speed_demand_mat_Max
	global	_ui16_Speed_demand_mat_min
	global	_ui16_Temp_cal
	global	_ui8_Ki_mat
	global	_ui8_fixed_start_speed_mat
	global	_ui16_Speed_mat
	global	_ui16_mat_Current
	global	_ui16_step_cnt
	global	_ui8_BattVolt_mat
	global	BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b
	global	BVH2_Appl_Layer@SIBFS_Dry_Running_b
	global	BVH2_Appl_Layer@SIBFS_Motor_Stalled_b
	global	BVH2_Appl_Layer@SIBFS_Temperature_Alarm_b
	global	_Cb27_idPWM
	global	_SIBFS_UbatHandling_b
	global	__ELINMIntMessageBufferPointer
	global	__ELINMIntStatus1
	global	_phaseCal
	global	_ui8_PWM_dc_mat
	global	_ui8_Task_Cont100ms
	global	_ui8_error_Flags
	global	_windowPtr1
	global	_ui16_Ubat_bldc
	global	_ui16_Duty_Cycle_In
	global	_ui16_IPhase1_bldc_cal
	global	_ui16_IPhase2_bldc_cal
	global	_ui16_IPhase3_bldc_cal
	global	__ELINMIntMessageBuffer
	global	_ui8_ResetMatlab
psect	nvBANK2,class=BANK2,space=1
global __pnvBANK2
__pnvBANK2:
_ui8_ResetMatlab:
       ds      1

	global	_ui16_Current_Thresh
psect	nvBANK3,class=BANK3,space=1
global __pnvBANK3
__pnvBANK3:
_ui16_Current_Thresh:
       ds      2

	global	_ui16_dryRun_Thresh
psect	nvBANK4,class=BANK4,space=1
global __pnvBANK4
__pnvBANK4:
_ui16_dryRun_Thresh:
       ds      2

	global	_INTCON
_INTCON	set	11
	global	_PR2
_PR2	set	27
	global	_T1CON
_T1CON	set	24
	global	_T2CON
_T2CON	set	28
	global	_TMR0
_TMR0	set	21
	global	_TMR1H
_TMR1H	set	23
	global	_TMR1L
_TMR1L	set	22
	global	_TMR2
_TMR2	set	26
	global	_CCP1IF
_CCP1IF	set	138
	global	_CCP2IF
_CCP2IF	set	144
	global	_CCP3IF
_CCP3IF	set	156
	global	_CCP4IF
_CCP4IF	set	157
	global	_CCP5IF
_CCP5IF	set	158
	global	_GIE
_GIE	set	95
	global	_PEIE
_PEIE	set	94
	global	_RC5
_RC5	set	117
	global	_RCIF
_RCIF	set	141
	global	_TMR0IE
_TMR0IE	set	93
	global	_TMR1IF
_TMR1IF	set	136
	global	_TMR2IF
_TMR2IF	set	137
	global	_TMR4IF
_TMR4IF	set	153
	global	_TMR6IF
_TMR6IF	set	155
	global	_ADCON0
_ADCON0	set	157
	global	_ADCON1
_ADCON1	set	158
	global	_ADRESH
_ADRESH	set	156
	global	_ADRESL
_ADRESL	set	155
	global	_OPTION_REG
_OPTION_REG	set	149
	global	_OSCCON
_OSCCON	set	153
	global	_PIE1
_PIE1	set	145
	global	_PIE2
_PIE2	set	146
	global	_PIE3
_PIE3	set	147
	global	_TRISA
_TRISA	set	140
	global	_TRISB
_TRISB	set	141
	global	_TRISC
_TRISC	set	142
	global	_WDTCON
_WDTCON	set	151
	global	_ADON
_ADON	set	1256
	global	_C1IE
_C1IE	set	1173
	global	_CCP1IE
_CCP1IE	set	1162
	global	_CCP2IE
_CCP2IE	set	1168
	global	_CCP3IE
_CCP3IE	set	1180
	global	_CCP4IE
_CCP4IE	set	1181
	global	_CCP5IE
_CCP5IE	set	1182
	global	_GO_nDONE
_GO_nDONE	set	1257
	global	_TMR1IE
_TMR1IE	set	1160
	global	_TMR2IE
_TMR2IE	set	1161
	global	_TMR4IE
_TMR4IE	set	1177
	global	_TMR6IE
_TMR6IE	set	1179
	global	_CM1CON0
_CM1CON0	set	273
	global	_CM1CON1
_CM1CON1	set	274
	global	_FVRCON
_FVRCON	set	279
	global	_LATA
_LATA	set	268
	global	_LATB
_LATB	set	269
	global	_LATC
_LATC	set	270
	global	_C1OUT
_C1OUT	set	2190
	global	_LATB1
_LATB1	set	2153
	global	_LATB2
_LATB2	set	2154
	global	_LATC0
_LATC0	set	2160
	global	_LATC1
_LATC1	set	2161
	global	_LATC2
_LATC2	set	2162
	global	_LATC3
_LATC3	set	2163
	global	_LATC4
_LATC4	set	2164
	global	_ANSELA
_ANSELA	set	396
	global	_ANSELB
_ANSELB	set	397
	global	_BAUDCON
_BAUDCON	set	415
	global	_EEADRH
_EEADRH	set	402
	global	_EEADRL
_EEADRL	set	401
	global	_EECON1
_EECON1	set	405
	global	_EECON2
_EECON2	set	406
	global	_EEDATA
_EEDATA	set	403
	global	_EEDATH
_EEDATH	set	404
	global	_EEDATL
_EEDATL	set	403
	global	_RCREG
_RCREG	set	409
	global	_RCSTA
_RCSTA	set	413
	global	_SPBRG
_SPBRG	set	411
	global	_SPBRGH
_SPBRGH	set	412
	global	_TXREG
_TXREG	set	410
	global	_TXSTA
_TXSTA	set	414
	global	_CFGS
_CFGS	set	3246
	global	_EEPGD
_EEPGD	set	3247
	global	_RD
_RD	set	3240
	global	_SENDB
_SENDB	set	3315
	global	_WR
_WR	set	3241
	global	_WREN
_WREN	set	3242
	global	_CCP1AS
_CCP1AS	set	661
	global	_CCP1CON
_CCP1CON	set	659
	global	_CCP2CON
_CCP2CON	set	666
	global	_CCPR1H
_CCPR1H	set	658
	global	_CCPR1L
_CCPR1L	set	657
	global	_CCPR2H
_CCPR2H	set	665
	global	_CCPR2L
_CCPR2L	set	664
	global	_PSTR1CON
_PSTR1CON	set	662
	global	_PWM1CON
_PWM1CON	set	660
	global	_CCP1ASE
_CCP1ASE	set	5295
	global	_CCP3CON
_CCP3CON	set	787
	global	_CCP4CON
_CCP4CON	set	794
	global	_CCP5CON
_CCP5CON	set	798
	global	_CCPR3H
_CCPR3H	set	786
	global	_CCPR3L
_CCPR3L	set	785
	global	_CCPR4H
_CCPR4H	set	793
	global	_CCPR4L
_CCPR4L	set	792
	global	_CCPR5H
_CCPR5H	set	797
	global	_CCPR5L
_CCPR5L	set	796
	global	_T4CON
_T4CON	set	1047
	global	_T6CON
_T6CON	set	1054
	global	_TMR4
_TMR4	set	1045
	global	_TMR6
_TMR6	set	1052
	file	"BVH2_V0.9.3.as"
	line	#
psect cinit,class=CODE,delta=2
global start_initialization
start_initialization:

psect	bitbssBANK0,class=BANK0,bit,space=1
global __pbitbssBANK0
__pbitbssBANK0:
_rising_bemf_flag:
       ds      1

psect	bssBANK0,class=BANK0,space=1
global __pbssBANK0
__pbssBANK0:
_analog_value:
       ds      2

_comm_time:
       ds      2

_ui16_IPhase_bldc:
       ds      2

_MotorFlags:
       ds      1

__ELINMIntStatus:
       ds      1

_comm_state:
       ds      1

_pt:
       ds      1

_ui8_b_DResB0_c:
       ds      1

_ui8_b_DResLocID_c:
       ds      1

_ui8_duty_cycle_BLDC:
       ds      1

_ui16_Ubat_bldc:
       ds      2

psect	dataBANK0,class=BANK0,space=1
global __pdataBANK0
__pdataBANK0:
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\main.c"
	line	122
_ui8_selected_lid:
       ds      1

psect	bssBANK1,class=BANK1,space=1
global __pbssBANK1
__pbssBANK1:
_ui16_Speed_mat:
       ds      2

_ui16_mat_Current:
       ds      2

_ui16_step_cnt:
       ds      2

_ui8_BattVolt_mat:
       ds      2

BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b:
       ds      1

BVH2_Appl_Layer@SIBFS_Dry_Running_b:
       ds      1

BVH2_Appl_Layer@SIBFS_Motor_Stalled_b:
       ds      1

BVH2_Appl_Layer@SIBFS_Temperature_Alarm_b:
       ds      1

_Cb27_idPWM:
       ds      1

_SIBFS_UbatHandling_b:
       ds      1

__ELINMIntMessageBufferPointer:
       ds      1

__ELINMIntStatus1:
       ds      1

_phaseCal:
       ds      1

_ui8_PWM_dc_mat:
       ds      1

_ui8_Task_Cont100ms:
       ds      1

_ui8_error_Flags:
       ds      1

_windowPtr1:
       ds      1

psect	dataBANK1,class=BANK1,space=1
global __pdataBANK1
__pdataBANK1:
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
	line	101
_wkpoint:
       ds      1

psect	bssBANK2,class=BANK2,space=1
global __pbssBANK2
__pbssBANK2:
BVH2_Appl_Layer@Cb18_StateCnt:
       ds      2

_Cb1_StateCnt:
       ds      2

_Cb37_StateCnt:
       ds      2

_ui16_Capt_Val0:
       ds      2

_ui16_Capt_Val1:
       ds      2

_ui16_Capt_Val2:
       ds      2

_ui16_Ubemf_bldc:
       ds      2

_ui16_comm_time_max:
       ds      2

_ui16_phase_advancement:
       ds      2

_ui16_speed_fil:
       ds      2

BVH2_Appl_Layer@Cb8_StateCnt:
       ds      1

BVH2_Appl_Layer@SIBFS_Temperature_Alarmo_old_b:
       ds      1

_Bcnt:
       ds      1

_Cb1_oCurrentAlarm:
       ds      1

_Cb1_oShutoff:
       ds      1

_Cb27_oPWM_Alarm:
       ds      1

_Cb53_odFixedValueSel:
       ds      1

_ErrorCode:
       ds      1

_SIBFS_Current_Analysis_High_b:
       ds      1

_SIBFS_PWM_Detection_b:
       ds      1

_SIBFS_Pic_etat_monitor_b:
       ds      1

__ELINMIntMessageSize:
       ds      1

__ELINMIntRXMessageSize:
       ds      1

__ELINMIntReadBack:
       ds      1

__ELINMIntSpace:
       ds      1

_bool_CPU_TempAlarm:
       ds      1

_bool_CPU_TempRedAlarm:
       ds      1

_bool_ControlLoopMode:
       ds      1

_bool_DryRunningAlarm:
       ds      1

_bool_HighCurrentAlarm:
       ds      1

_bool_MotorStalled:
       ds      1

_bool_PWMin_Freq_err_Alarm:
       ds      1

_bool_PWMin_err_Alarm:
       ds      1

_bool_UbatAlarm:
       ds      1

_bool_mat_currAlarm_bldc:
       ds      1

_bool_start_demand_mat:
       ds      1

_ui8_BlankingCount:
       ds      1

_ui8_CompFlag:
       ds      1

_ui8_Duty_Cycle_In_Ratio:
       ds      1

_ui8_IPhase_sel:
       ds      1

_ui8_Kp_mat:
       ds      1

_ui8_PWM_FreqCnt:
       ds      1

_ui8_PWMinDC_sav:
       ds      1

_ui8_PWMin_failCnt:
       ds      1

_ui8_PWMoutvalue:
       ds      1

_ui8_Pulse_State:
       ds      1

_ui8_StartupPWM:
       ds      1

_ui8_Task_Cont1ms:
       ds      1

_ui8_Task_Cont3ms:
       ds      1

_ui8_Task_Cont5ms:
       ds      1

_ui8_UPhase_sel:
       ds      1

_ui8_Ubemf_sel:
       ds      1

_ui8_b_DResB1_c:
       ds      1

_ui8_b_DResB2_c:
       ds      1

_ui8_b_DResB3_c:
       ds      1

_ui8_b_DResB4_c:
       ds      1

_ui8_b_DResServID_c:
       ds      1

_ui8_duty_cycle_mat:
       ds      1

_ui8_given_supply:
       ds      1

_ui8_lastTaskvalue:
       ds      1

_ui8_sampleState:
       ds      1

psect	dataBANK2,class=BANK2,space=1
global __pdataBANK2
__pdataBANK2:
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\main.c"
	line	116
_ui16_I_cal_Ph1:
       ds      2

psect	dataBANK2
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\main.c"
	line	117
_ui16_I_cal_Ph2:
       ds      2

psect	dataBANK2
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\main.c"
	line	118
_ui16_I_cal_Ph3:
       ds      2

psect	dataBANK2
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
	line	100
_ict_stamp:
       ds      1

psect	dataBANK2
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
	line	103
_ui8_failure:
       ds      1

psect	bssBANK3,class=BANK3,space=1
global __pbssBANK3
__pbssBANK3:
_B:
       ds      16

_ui8_current_cal:
       ds      3

__ELINMIntTFrameMin:
       ds      2

__ELINMIntTHeaderMin:
       ds      2

_filter1:
       ds      2

_filter3:
       ds      2

_filterTempNTC:
       ds      2

_sum:
       ds      2

_ui16_IPhase1_bldc:
       ds      2

_ui16_NTC_Temp_bldc_mean:
       ds      2

_ui16_NTC_Temp_bldc_mean_cal:
       ds      2

_ui16_PWM_Freq_In:
       ds      2

_ui16_PWM_Freq_mat:
       ds      2

_ui16_fir_Bat_mittel:
       ds      2

_ui16_fir_IPhase_mean:
       ds      2

_ui16_mat_inpTemp:
       ds      2

_ui16_speed_rar:
       ds      2

BVH2_Appl_Layer@Cb13_oCurrentAlarm:
       ds      1

BVH2_Appl_Layer@Cb18_oMotorStalled:
       ds      1

BVH2_Appl_Layer@Cb18_oStalledAlarm:
       ds      1

BVH2_Appl_Layer@Cb44_Counter:
       ds      1

BVH2_Appl_Layer@Cb44_oTempAlarm:
       ds      1

BVH2_Appl_Layer@Cb44_oTempRedAlarm:
       ds      1

BVH2_Appl_Layer@Cb44_odPumpOff:
       ds      1

BVH2_Appl_Layer@Cb8_oCurrentAlarm:
       ds      1

_Cb27_oPWM_SC_Alarm:
       ds      1

_Cb27_odFixedLowValueSel:
       ds      1

_Cb27_odFixedValueSel:
       ds      1

_Cb27_odPumpOff:
       ds      1

_Cb37_oShutoff:
       ds      1

_Cb53_oUbat_Alarm_High:
       ds      1

_Cb53_odPumpOff:
       ds      1

_ui16_Duty_Cycle_In:
       ds      2

_ui16_IPhase1_bldc_cal:
       ds      2

_ui16_IPhase2_bldc_cal:
       ds      2

_ui16_IPhase3_bldc_cal:
       ds      2

psect	dataBANK3,class=BANK3,space=1
global __pdataBANK3
__pdataBANK3:
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\T_Link\BVH2_Appl_Layer.c"
	line	409
BVH2_Appl_Layer@X_Sb4_Intergrator:
       ds      4

psect	dataBANK3
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\T_Link\BVH2_Appl_Layer.c"
	line	411
BVH2_Appl_Layer@Sb1_BVH2_Appl_Layer_FirstRun:
       ds      1

psect	dataBANK3
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\T_Link\BVH2_Appl_Layer.c"
	line	412
BVH2_Appl_Layer@X_Sb4_Intergrator_TriggerIn:
       ds      1

psect	bssBANK4,class=BANK4,space=1
global __pbssBANK4
__pbssBANK4:
_inputArray1:
       ds      16

_my_msg:
       ds      8

_checksum:
       ds      6

__ELINMIntSleepTimeout:
       ds      4

BVH2_Appl_Layer@Cb13_StateCnt:
       ds      2

BVH2_Appl_Layer@Cb18_BadCnt:
       ds      2

__ELINMIntRXCRC:
       ds      2

__ELINMIntTFrameMax:
       ds      2

__ELINMIntTHeaderMax:
       ds      2

_phase_delay_counter:
       ds      2

_ui16_IPhase2_bldc:
       ds      2

_ui16_IPhase3_bldc:
       ds      2

_ui16_NTC_Temp_bldc:
       ds      2

_ui16_Speed_demand_mat:
       ds      2

_ui16_Speed_demand_mat_Max:
       ds      2

_ui16_Speed_demand_mat_min:
       ds      2

_ui16_Temp_cal:
       ds      2

_ui8_Ki_mat:
       ds      2

_ui8_fixed_start_speed_mat:
       ds      2

__ELINMIntMessageBuffer:
       ds      11

psect clrtext,class=CODE,delta=2
global clear_ram
;	Called with FSR0 containing the base address, and
;	WREG with the size to clear
clear_ram:
	clrwdt			;clear the watchdog before getting into this loop
clrloop:
	clrf	indf0		;clear RAM location pointed to by FSR
	addfsr	0,1
	decfsz wreg		;Have we reached the end of clearing yet?
	goto clrloop	;have we reached the end yet?
	retlw	0		;all done for this memory range, return
; Clear objects allocated to BITBANK0
psect cinit,class=CODE,delta=2
	global __pbitbssBANK0
	clrf	((__pbitbssBANK0/8)+0)&07Fh
; Clear objects allocated to BANK0
psect cinit,class=CODE,delta=2
	global __pbssBANK0
	movlw	low(__pbssBANK0)
	movwf	fsr0l
	movlw	high(__pbssBANK0)
	movwf	fsr0h
	movlw	0Fh
	fcall	clear_ram
; Clear objects allocated to BANK1
psect cinit,class=CODE,delta=2
	global __pbssBANK1
	movlw	low(__pbssBANK1)
	movwf	fsr0l
	movlw	high(__pbssBANK1)
	movwf	fsr0h
	movlw	015h
	fcall	clear_ram
; Clear objects allocated to BANK2
psect cinit,class=CODE,delta=2
	global __pbssBANK2
	movlw	low(__pbssBANK2)
	movwf	fsr0l
	movlw	high(__pbssBANK2)
	movwf	fsr0h
	movlw	047h
	fcall	clear_ram
; Clear objects allocated to BANK3
psect cinit,class=CODE,delta=2
	global __pbssBANK3
	movlw	low(__pbssBANK3)
	movwf	fsr0l
	movlw	high(__pbssBANK3)
	movwf	fsr0h
	movlw	048h
	fcall	clear_ram
; Clear objects allocated to BANK4
psect cinit,class=CODE,delta=2
	global __pbssBANK4
	movlw	low(__pbssBANK4)
	movwf	fsr0l
	movlw	high(__pbssBANK4)
	movwf	fsr0h
	movlw	04Bh
	fcall	clear_ram
; Initialize objects allocated to BANK0
	global __pidataBANK0,__pdataBANK0
psect cinit,class=CODE,delta=2
	fcall	__pidataBANK0+0		;fetch initializer
	movwf	__pdataBANK0+0&07fh		
; Initialize objects allocated to BANK1
	global __pidataBANK1,__pdataBANK1
psect cinit,class=CODE,delta=2
	movlb 1	; select bank1
	fcall	__pidataBANK1+0		;fetch initializer
	movwf	__pdataBANK1+0&07fh		
; Initialize objects allocated to BANK2
	global __pidataBANK2,__pdataBANK2
psect cinit,class=CODE,delta=2
	movlb 2	; select bank2
	fcall	__pidataBANK2+0		;fetch initializer
	movwf	__pdataBANK2+0&07fh		
	fcall	__pidataBANK2+1		;fetch initializer
	movwf	__pdataBANK2+1&07fh		
	fcall	__pidataBANK2+2		;fetch initializer
	movwf	__pdataBANK2+2&07fh		
	fcall	__pidataBANK2+3		;fetch initializer
	movwf	__pdataBANK2+3&07fh		
	fcall	__pidataBANK2+4		;fetch initializer
	movwf	__pdataBANK2+4&07fh		
	fcall	__pidataBANK2+5		;fetch initializer
	movwf	__pdataBANK2+5&07fh		
	fcall	__pidataBANK2+6		;fetch initializer
	movwf	__pdataBANK2+6&07fh		
	fcall	__pidataBANK2+7		;fetch initializer
	movwf	__pdataBANK2+7&07fh		
; Initialize objects allocated to BANK3
	global __pidataBANK3,__pdataBANK3
psect cinit,class=CODE,delta=2
	movlb 3	; select bank3
	fcall	__pidataBANK3+0		;fetch initializer
	movwf	__pdataBANK3+0&07fh		
	fcall	__pidataBANK3+1		;fetch initializer
	movwf	__pdataBANK3+1&07fh		
	fcall	__pidataBANK3+2		;fetch initializer
	movwf	__pdataBANK3+2&07fh		
	fcall	__pidataBANK3+3		;fetch initializer
	movwf	__pdataBANK3+3&07fh		
	fcall	__pidataBANK3+4		;fetch initializer
	movwf	__pdataBANK3+4&07fh		
	fcall	__pidataBANK3+5		;fetch initializer
	movwf	__pdataBANK3+5&07fh		
psect cinit,class=CODE,delta=2
global end_of_initialization

;End of C runtime variable initialization code

end_of_initialization:
movlb 0
ljmp _main	;jump to C main() function
psect	cstackBANK1,class=BANK1,space=1
global __pcstackBANK1
__pcstackBANK1:
	global	EOL@_dcnt
EOL@_dcnt:	; 1 bytes @ 0x0
	global	BVH2_Appl_Layer@Aux_U32_a
BVH2_Appl_Layer@Aux_U32_a:	; 4 bytes @ 0x0
	ds	1
	global	EOL@_dcnt_9399
EOL@_dcnt_9399:	; 1 bytes @ 0x1
	ds	1
	global	EOL@_dcnt_9400
EOL@_dcnt_9400:	; 1 bytes @ 0x2
	ds	1
	global	EOL@_dcnt_9401
EOL@_dcnt_9401:	; 1 bytes @ 0x3
	ds	1
	global	EOL@_dcnt_9402
EOL@_dcnt_9402:	; 1 bytes @ 0x4
	global	_BVH2_Appl_Layer$20288
_BVH2_Appl_Layer$20288:	; 4 bytes @ 0x4
	ds	1
	global	EOL@_dcnt_9403
EOL@_dcnt_9403:	; 1 bytes @ 0x5
	ds	1
	global	EOL@_dcnt_9404
EOL@_dcnt_9404:	; 1 bytes @ 0x6
	ds	1
	global	EOL@_dcnt_9405
EOL@_dcnt_9405:	; 1 bytes @ 0x7
	ds	1
	global	EOL@_dcnt_9406
EOL@_dcnt_9406:	; 1 bytes @ 0x8
	global	BVH2_Appl_Layer@Sb4_Product2
BVH2_Appl_Layer@Sb4_Product2:	; 2 bytes @ 0x8
	ds	1
	global	EOL@_dcnt_9407
EOL@_dcnt_9407:	; 1 bytes @ 0x9
	ds	1
	global	EOL@_dcnt_9408
EOL@_dcnt_9408:	; 1 bytes @ 0xA
	global	_BVH2_Appl_Layer$20278
_BVH2_Appl_Layer$20278:	; 2 bytes @ 0xA
	ds	1
	global	EOL@_dcnt_9409
EOL@_dcnt_9409:	; 1 bytes @ 0xB
	ds	1
	global	EOL@_dcnt_9410
EOL@_dcnt_9410:	; 1 bytes @ 0xC
	global	_BVH2_Appl_Layer$20279
_BVH2_Appl_Layer$20279:	; 2 bytes @ 0xC
	ds	1
	global	EOL@_dcnt_9411
EOL@_dcnt_9411:	; 1 bytes @ 0xD
	ds	1
	global	EOL@_dcnt_9412
EOL@_dcnt_9412:	; 1 bytes @ 0xE
	global	_BVH2_Appl_Layer$20285
_BVH2_Appl_Layer$20285:	; 2 bytes @ 0xE
	ds	1
	global	EOL@_dcnt_9413
EOL@_dcnt_9413:	; 1 bytes @ 0xF
	ds	1
	global	EOL@_dcnt_9414
EOL@_dcnt_9414:	; 1 bytes @ 0x10
	global	_BVH2_Appl_Layer$20287
_BVH2_Appl_Layer$20287:	; 2 bytes @ 0x10
	ds	1
	global	EOL@_dcnt_9415
EOL@_dcnt_9415:	; 1 bytes @ 0x11
	ds	1
	global	EOL@ui8_b_DResB5_RD
EOL@ui8_b_DResB5_RD:	; 1 bytes @ 0x12
	global	BVH2_Appl_Layer@Sb1_Logical_Operator2
BVH2_Appl_Layer@Sb1_Logical_Operator2:	; 1 bytes @ 0x12
	ds	1
	global	EOL@ui8_b_DResB4_RD
EOL@ui8_b_DResB4_RD:	; 1 bytes @ 0x13
	global	_BVH2_Appl_Layer$20275
_BVH2_Appl_Layer$20275:	; 1 bytes @ 0x13
	ds	1
	global	EOL@ui8_b_DResB0_RD
EOL@ui8_b_DResB0_RD:	; 1 bytes @ 0x14
	global	_BVH2_Appl_Layer$20276
_BVH2_Appl_Layer$20276:	; 1 bytes @ 0x14
	ds	1
	global	EOL@ui8_b_DResB1_RD
EOL@ui8_b_DResB1_RD:	; 1 bytes @ 0x15
	global	_BVH2_Appl_Layer$20277
_BVH2_Appl_Layer$20277:	; 1 bytes @ 0x15
	ds	1
	global	EOL@ui8_b_DResB2_RD
EOL@ui8_b_DResB2_RD:	; 1 bytes @ 0x16
	global	_BVH2_Appl_Layer$20280
_BVH2_Appl_Layer$20280:	; 1 bytes @ 0x16
	ds	1
	global	EOL@ui8_b_DResB3_RD
EOL@ui8_b_DResB3_RD:	; 1 bytes @ 0x17
	global	_BVH2_Appl_Layer$20281
_BVH2_Appl_Layer$20281:	; 1 bytes @ 0x17
	ds	1
	global	_BVH2_Appl_Layer$20282
_BVH2_Appl_Layer$20282:	; 1 bytes @ 0x18
	ds	5
	global	BVH2_Appl_Layer@Aux_S32_a
BVH2_Appl_Layer@Aux_S32_a:	; 4 bytes @ 0x1D
	ds	6
	global	BVH2_Appl_Layer@Sb2_Switch5
BVH2_Appl_Layer@Sb2_Switch5:	; 2 bytes @ 0x23
	ds	2
	global	BVH2_Appl_Layer@Aux_S32_b
BVH2_Appl_Layer@Aux_S32_b:	; 4 bytes @ 0x25
	ds	4
	global	BVH2_Appl_Layer@Aux_U32
BVH2_Appl_Layer@Aux_U32:	; 4 bytes @ 0x29
	ds	4
	global	BVH2_Appl_Layer@Cb18_Reset
BVH2_Appl_Layer@Cb18_Reset:	; 1 bytes @ 0x2D
	ds	1
	global	BVH2_Appl_Layer@Sb1_Logical_Operator3
BVH2_Appl_Layer@Sb1_Logical_Operator3:	; 1 bytes @ 0x2E
	ds	1
	global	BVH2_Appl_Layer@Sb1_Logical_Operator5
BVH2_Appl_Layer@Sb1_Logical_Operator5:	; 1 bytes @ 0x2F
	ds	1
	global	BVH2_Appl_Layer@Sb2_Logical_Operator2
BVH2_Appl_Layer@Sb2_Logical_Operator2:	; 1 bytes @ 0x30
	ds	1
	global	BVH2_Appl_Layer@Sb2_Error
BVH2_Appl_Layer@Sb2_Error:	; 2 bytes @ 0x31
	ds	2
	global	BVH2_Appl_Layer@Sb2_Switch2
BVH2_Appl_Layer@Sb2_Switch2:	; 2 bytes @ 0x33
	ds	2
	global	BVH2_Appl_Layer@Sb1_Logical_Operator1
BVH2_Appl_Layer@Sb1_Logical_Operator1:	; 1 bytes @ 0x35
	ds	1
	global	BVH2_Appl_Layer@Sb3_Sum1
BVH2_Appl_Layer@Sb3_Sum1:	; 2 bytes @ 0x36
	ds	2
	global	BVH2_Appl_Layer@Sb4_PI_sum
BVH2_Appl_Layer@Sb4_PI_sum:	; 2 bytes @ 0x38
	ds	2
psect	cstackCOMMON,class=COMMON,space=1
global __pcstackCOMMON
__pcstackCOMMON:
	global	?_InitMotorRun
?_InitMotorRun:	; 0 bytes @ 0x0
	global	?_commutate
?_commutate:	; 0 bytes @ 0x0
	global	?_PWM_Write_Out
?_PWM_Write_Out:	; 0 bytes @ 0x0
	global	?_NegativeAnswer
?_NegativeAnswer:	; 0 bytes @ 0x0
	global	?_PWM_CTRL
?_PWM_CTRL:	; 0 bytes @ 0x0
	global	??_PWM_CTRL
??_PWM_CTRL:	; 0 bytes @ 0x0
	global	?_ELINMIntHandler
?_ELINMIntHandler:	; 0 bytes @ 0x0
	global	?_interrupt_PWMCapture
?_interrupt_PWMCapture:	; 0 bytes @ 0x0
	global	??_interrupt_PWMCapture
??_interrupt_PWMCapture:	; 0 bytes @ 0x0
	global	?_Oscill_Source_Block
?_Oscill_Source_Block:	; 0 bytes @ 0x0
	global	?_timer_init
?_timer_init:	; 0 bytes @ 0x0
	global	?_PWM_Capture_init
?_PWM_Capture_init:	; 0 bytes @ 0x0
	global	?_clear_timer
?_clear_timer:	; 0 bytes @ 0x0
	global	?_BVH2_Appl_Layer
?_BVH2_Appl_Layer:	; 0 bytes @ 0x0
	global	?_Cb37_Pic_etat_monitor_node_fcn1
?_Cb37_Pic_etat_monitor_node_fcn1:	; 0 bytes @ 0x0
	global	?_Cb27_PWM_Detection_node_fcn1
?_Cb27_PWM_Detection_node_fcn1:	; 0 bytes @ 0x0
	global	?_Cb53_UbatHandling_node_fcn2
?_Cb53_UbatHandling_node_fcn2:	; 0 bytes @ 0x0
	global	?_Cb1_Current_An___High_node_fcn1
?_Cb1_Current_An___High_node_fcn1:	; 0 bytes @ 0x0
	global	?_ADC_Init
?_ADC_Init:	; 0 bytes @ 0x0
	global	?_ADC_Wait
?_ADC_Wait:	; 0 bytes @ 0x0
	global	?_FILTER_Init
?_FILTER_Init:	; 0 bytes @ 0x0
	global	?_FILTER_Ubat
?_FILTER_Ubat:	; 0 bytes @ 0x0
	global	?_FILTER_IPhase
?_FILTER_IPhase:	; 0 bytes @ 0x0
	global	?_FILTER_Temp
?_FILTER_Temp:	; 0 bytes @ 0x0
	global	?_Get_Analog_Value
?_Get_Analog_Value:	; 0 bytes @ 0x0
	global	?_BLDCWait
?_BLDCWait:	; 0 bytes @ 0x0
	global	??_BLDCWait
??_BLDCWait:	; 0 bytes @ 0x0
	global	?_init_bldc
?_init_bldc:	; 0 bytes @ 0x0
	global	?_interrrupt_bldc
?_interrrupt_bldc:	; 0 bytes @ 0x0
	global	?_InitMotorStop
?_InitMotorStop:	; 0 bytes @ 0x0
	global	?_DiagInit
?_DiagInit:	; 0 bytes @ 0x0
	global	?_SetDiagAlarm
?_SetDiagAlarm:	; 0 bytes @ 0x0
	global	?_EOL
?_EOL:	; 0 bytes @ 0x0
	global	?_Task1ms
?_Task1ms:	; 0 bytes @ 0x0
	global	??_Task1ms
??_Task1ms:	; 0 bytes @ 0x0
	global	?_interrupt_handler
?_interrupt_handler:	; 0 bytes @ 0x0
	global	?_Receive_Diag
?_Receive_Diag:	; 0 bytes @ 0x0
	global	?_EnableMCP201
?_EnableMCP201:	; 0 bytes @ 0x0
	global	?__ELINMIntResetProtocol
?__ELINMIntResetProtocol:	; 0 bytes @ 0x0
	global	??__ELINMIntResetProtocol
??__ELINMIntResetProtocol:	; 0 bytes @ 0x0
	global	?_I_calibrationInit
?_I_calibrationInit:	; 0 bytes @ 0x0
	global	?_init_ports
?_init_ports:	; 0 bytes @ 0x0
	global	?_system_init
?_system_init:	; 0 bytes @ 0x0
	global	?_main
?_main:	; 0 bytes @ 0x0
	global	?_Cb53_UbatHandling_node_fcn1
?_Cb53_UbatHandling_node_fcn1:	; 0 bytes @ 0x0
	global	?i1_ADC_Wait
?i1_ADC_Wait:	; 0 bytes @ 0x0
	global	??i1_ADC_Wait
??i1_ADC_Wait:	; 0 bytes @ 0x0
	global	?i1_commutate
?i1_commutate:	; 0 bytes @ 0x0
	global	?_read_eeprom_data
?_read_eeprom_data:	; 1 bytes @ 0x0
	global	?_PWMReadDC
?_PWMReadDC:	; 1 bytes @ 0x0
	global	?__ELINMIntInitialize
?__ELINMIntInitialize:	; 1 bytes @ 0x0
	global	?__ELINMIntCalcIDParity
?__ELINMIntCalcIDParity:	; 1 bytes @ 0x0
	global	?i1_ADC_Read
?i1_ADC_Read:	; 2 bytes @ 0x0
	global	?i1___wmul
?i1___wmul:	; 2 bytes @ 0x0
	global	?i1___lwdiv
?i1___lwdiv:	; 2 bytes @ 0x0
	global	__ELINMIntResetProtocol@code
__ELINMIntResetProtocol@code:	; 1 bytes @ 0x0
	global	i1___wmul@multiplier
i1___wmul@multiplier:	; 2 bytes @ 0x0
	global	i1___lwdiv@divisor
i1___lwdiv@divisor:	; 2 bytes @ 0x0
	ds	1
	global	??_ELINMIntHandler
??_ELINMIntHandler:	; 0 bytes @ 0x1
	ds	1
	global	??i1_ADC_Read
??i1_ADC_Read:	; 0 bytes @ 0x2
	global	i1ADC_Read@i
i1ADC_Read@i:	; 1 bytes @ 0x2
	global	i1___wmul@multiplicand
i1___wmul@multiplicand:	; 2 bytes @ 0x2
	global	i1___lwdiv@dividend
i1___lwdiv@dividend:	; 2 bytes @ 0x2
	ds	2
	global	??i1___wmul
??i1___wmul:	; 0 bytes @ 0x4
	global	??i1___lwdiv
??i1___lwdiv:	; 0 bytes @ 0x4
	global	i1___lwdiv@counter
i1___lwdiv@counter:	; 1 bytes @ 0x4
	global	i1___wmul@product
i1___wmul@product:	; 2 bytes @ 0x4
	ds	1
	global	i1___lwdiv@quotient
i1___lwdiv@quotient:	; 2 bytes @ 0x5
	ds	2
	global	??_interrrupt_bldc
??_interrrupt_bldc:	; 0 bytes @ 0x7
	ds	2
	global	interrrupt_bldc@ui32_tmp
interrrupt_bldc@ui32_tmp:	; 4 bytes @ 0x9
	ds	4
	global	??_interrupt_handler
??_interrupt_handler:	; 0 bytes @ 0xD
psect	cstackBANK0,class=BANK0,space=1
global __pcstackBANK0
__pcstackBANK0:
	global	??i1_commutate
??i1_commutate:	; 0 bytes @ 0x0
	ds	12
	global	??_PWM_Write_Out
??_PWM_Write_Out:	; 0 bytes @ 0xC
	global	??_read_eeprom_data
??_read_eeprom_data:	; 0 bytes @ 0xC
	global	??_Oscill_Source_Block
??_Oscill_Source_Block:	; 0 bytes @ 0xC
	global	??_PWM_Capture_init
??_PWM_Capture_init:	; 0 bytes @ 0xC
	global	??_clear_timer
??_clear_timer:	; 0 bytes @ 0xC
	global	??_Cb37_Pic_etat_monitor_node_fcn1
??_Cb37_Pic_etat_monitor_node_fcn1:	; 0 bytes @ 0xC
	global	??_Cb27_PWM_Detection_node_fcn1
??_Cb27_PWM_Detection_node_fcn1:	; 0 bytes @ 0xC
	global	??_Cb53_UbatHandling_node_fcn2
??_Cb53_UbatHandling_node_fcn2:	; 0 bytes @ 0xC
	global	??_Cb1_Current_An___High_node_fcn1
??_Cb1_Current_An___High_node_fcn1:	; 0 bytes @ 0xC
	global	??_ADC_Init
??_ADC_Init:	; 0 bytes @ 0xC
	global	??_ADC_Wait
??_ADC_Wait:	; 0 bytes @ 0xC
	global	??_FILTER_Ubat
??_FILTER_Ubat:	; 0 bytes @ 0xC
	global	??_FILTER_IPhase
??_FILTER_IPhase:	; 0 bytes @ 0xC
	global	??_FILTER_Temp
??_FILTER_Temp:	; 0 bytes @ 0xC
	global	??_InitMotorStop
??_InitMotorStop:	; 0 bytes @ 0xC
	global	??_EnableMCP201
??_EnableMCP201:	; 0 bytes @ 0xC
	global	??__ELINMIntInitialize
??__ELINMIntInitialize:	; 0 bytes @ 0xC
	global	??__ELINMIntCalcIDParity
??__ELINMIntCalcIDParity:	; 0 bytes @ 0xC
	global	??_Cb53_UbatHandling_node_fcn1
??_Cb53_UbatHandling_node_fcn1:	; 0 bytes @ 0xC
	global	?__ELINMIntGetPointer
?__ELINMIntGetPointer:	; 1 bytes @ 0xC
	global	?_ADC_Read
?_ADC_Read:	; 2 bytes @ 0xC
	global	?_cksum
?_cksum:	; 2 bytes @ 0xC
	global	?___wmul
?___wmul:	; 2 bytes @ 0xC
	global	?___lwdiv
?___lwdiv:	; 2 bytes @ 0xC
	global	?___tmul
?___tmul:	; 3 bytes @ 0xC
	global	?___lmul
?___lmul:	; 4 bytes @ 0xC
	global	?___aldiv
?___aldiv:	; 4 bytes @ 0xC
	global	read_eeprom_data@ui8_adress
read_eeprom_data@ui8_adress:	; 1 bytes @ 0xC
	global	EnableMCP201@_dcnt
EnableMCP201@_dcnt:	; 1 bytes @ 0xC
	global	__ELINMIntCalcIDParity@ELINM_idtr
__ELINMIntCalcIDParity@ELINM_idtr:	; 1 bytes @ 0xC
	global	__ELINMIntGetPointer@_ELINMInt_position
__ELINMIntGetPointer@_ELINMInt_position:	; 1 bytes @ 0xC
	global	PWM_Capture_init@ui8_CCP_Nb
PWM_Capture_init@ui8_CCP_Nb:	; 1 bytes @ 0xC
	global	PWM_Write_Out@ui8_DutyCycle_Out
PWM_Write_Out@ui8_DutyCycle_Out:	; 1 bytes @ 0xC
	global	clear_timer@ui8_TmrNb
clear_timer@ui8_TmrNb:	; 1 bytes @ 0xC
	global	___wmul@multiplier
___wmul@multiplier:	; 2 bytes @ 0xC
	global	___lwdiv@divisor
___lwdiv@divisor:	; 2 bytes @ 0xC
	global	___tmul@multiplier
___tmul@multiplier:	; 3 bytes @ 0xC
	global	___lmul@multiplier
___lmul@multiplier:	; 4 bytes @ 0xC
	global	___aldiv@divisor
___aldiv@divisor:	; 4 bytes @ 0xC
	ds	1
	global	??__ELINMIntGetPointer
??__ELINMIntGetPointer:	; 0 bytes @ 0xD
	global	?__ELINMIntSendMessage
?__ELINMIntSendMessage:	; 0 bytes @ 0xD
	global	??_timer_init
??_timer_init:	; 0 bytes @ 0xD
	global	??_DiagInit
??_DiagInit:	; 0 bytes @ 0xD
	global	??_SetDiagAlarm
??_SetDiagAlarm:	; 0 bytes @ 0xD
	global	read_eeprom_data@ui8_adress_data
read_eeprom_data@ui8_adress_data:	; 1 bytes @ 0xD
	global	EnableMCP201@_dcnt_15775
EnableMCP201@_dcnt_15775:	; 1 bytes @ 0xD
	global	__ELINMIntSendMessage@_ELINM_size
__ELINMIntSendMessage@_ELINM_size:	; 1 bytes @ 0xD
	global	__ELINMIntGetPointer@_ELINMInt_tag
__ELINMIntGetPointer@_ELINMInt_tag:	; 1 bytes @ 0xD
	global	timer_init@ui8_TmrNb
timer_init@ui8_TmrNb:	; 1 bytes @ 0xD
	ds	1
	global	??_ADC_Read
??_ADC_Read:	; 0 bytes @ 0xE
	global	??_cksum
??_cksum:	; 0 bytes @ 0xE
	global	??_init_ports
??_init_ports:	; 0 bytes @ 0xE
	global	ADC_Read@i
ADC_Read@i:	; 1 bytes @ 0xE
	global	EnableMCP201@_dcnt_15776
EnableMCP201@_dcnt_15776:	; 1 bytes @ 0xE
	global	cksum@counter
cksum@counter:	; 2 bytes @ 0xE
	global	__ELINMIntSendMessage@_ELINM_fmin
__ELINMIntSendMessage@_ELINM_fmin:	; 2 bytes @ 0xE
	global	___wmul@multiplicand
___wmul@multiplicand:	; 2 bytes @ 0xE
	global	___lwdiv@dividend
___lwdiv@dividend:	; 2 bytes @ 0xE
	ds	1
	global	??_FILTER_Init
??_FILTER_Init:	; 0 bytes @ 0xF
	global	??_I_calibrationInit
??_I_calibrationInit:	; 0 bytes @ 0xF
	global	___tmul@multiplicand
___tmul@multiplicand:	; 3 bytes @ 0xF
	ds	1
	global	??___wmul
??___wmul:	; 0 bytes @ 0x10
	global	??___lwdiv
??___lwdiv:	; 0 bytes @ 0x10
	global	___lwdiv@counter
___lwdiv@counter:	; 1 bytes @ 0x10
	global	cksum@sum
cksum@sum:	; 2 bytes @ 0x10
	global	__ELINMIntSendMessage@_ELINM_fmax
__ELINMIntSendMessage@_ELINM_fmax:	; 2 bytes @ 0x10
	global	___wmul@product
___wmul@product:	; 2 bytes @ 0x10
	global	___lmul@multiplicand
___lmul@multiplicand:	; 4 bytes @ 0x10
	global	___aldiv@dividend
___aldiv@dividend:	; 4 bytes @ 0x10
	ds	1
	global	___lwdiv@quotient
___lwdiv@quotient:	; 2 bytes @ 0x11
	ds	1
	global	??_commutate
??_commutate:	; 0 bytes @ 0x12
	global	??__ELINMIntSendMessage
??__ELINMIntSendMessage:	; 0 bytes @ 0x12
	global	??___tmul
??___tmul:	; 0 bytes @ 0x12
	global	?___awdiv
?___awdiv:	; 2 bytes @ 0x12
	global	__ELINMIntSendMessage@_ELINM_idr
__ELINMIntSendMessage@_ELINM_idr:	; 1 bytes @ 0x12
	global	cksum@addr
cksum@addr:	; 2 bytes @ 0x12
	global	___awdiv@divisor
___awdiv@divisor:	; 2 bytes @ 0x12
	global	___tmul@product
___tmul@product:	; 3 bytes @ 0x12
	ds	1
	global	?_write_eeprom_data
?_write_eeprom_data:	; 0 bytes @ 0x13
	global	write_eeprom_data@ui8_adress_data
write_eeprom_data@ui8_adress_data:	; 1 bytes @ 0x13
	global	__ELINMIntSendMessage@_ELINM_tid
__ELINMIntSendMessage@_ELINM_tid:	; 1 bytes @ 0x13
	ds	1
	global	??_write_eeprom_data
??_write_eeprom_data:	; 0 bytes @ 0x14
	global	??___lmul
??___lmul:	; 0 bytes @ 0x14
	global	??___aldiv
??___aldiv:	; 0 bytes @ 0x14
	global	write_eeprom_data@ui8_adress
write_eeprom_data@ui8_adress:	; 1 bytes @ 0x14
	global	___aldiv@counter
___aldiv@counter:	; 1 bytes @ 0x14
	global	__ELINMIntSendMessage@_ELINM_chk
__ELINMIntSendMessage@_ELINM_chk:	; 2 bytes @ 0x14
	global	___awdiv@dividend
___awdiv@dividend:	; 2 bytes @ 0x14
	global	___lmul@product
___lmul@product:	; 4 bytes @ 0x14
	ds	1
	global	?___ltdiv
?___ltdiv:	; 3 bytes @ 0x15
	global	___aldiv@sign
___aldiv@sign:	; 1 bytes @ 0x15
	global	___ltdiv@divisor
___ltdiv@divisor:	; 3 bytes @ 0x15
	ds	1
	global	??___awdiv
??___awdiv:	; 0 bytes @ 0x16
	global	__ELINMIntSendMessage@_ELINM_i
__ELINMIntSendMessage@_ELINM_i:	; 1 bytes @ 0x16
	global	___awdiv@counter
___awdiv@counter:	; 1 bytes @ 0x16
	global	___aldiv@quotient
___aldiv@quotient:	; 4 bytes @ 0x16
	ds	1
	global	?_Transmit_LIN_8Bytes
?_Transmit_LIN_8Bytes:	; 0 bytes @ 0x17
	global	?__ELINMIntReceiveMessage
?__ELINMIntReceiveMessage:	; 0 bytes @ 0x17
	global	Transmit_LIN_8Bytes@B0
Transmit_LIN_8Bytes@B0:	; 1 bytes @ 0x17
	global	__ELINMIntReceiveMessage@_ELINM_id
__ELINMIntReceiveMessage@_ELINM_id:	; 1 bytes @ 0x17
	global	___awdiv@sign
___awdiv@sign:	; 1 bytes @ 0x17
	ds	1
	global	??_Get_Analog_Value
??_Get_Analog_Value:	; 0 bytes @ 0x18
	global	Transmit_LIN_8Bytes@B1
Transmit_LIN_8Bytes@B1:	; 1 bytes @ 0x18
	global	__ELINMIntReceiveMessage@_ELINM_size
__ELINMIntReceiveMessage@_ELINM_size:	; 1 bytes @ 0x18
	global	___awdiv@quotient
___awdiv@quotient:	; 2 bytes @ 0x18
	global	___ltdiv@dividend
___ltdiv@dividend:	; 3 bytes @ 0x18
	ds	1
	global	??__ELINMIntReceiveMessage
??__ELINMIntReceiveMessage:	; 0 bytes @ 0x19
	global	Transmit_LIN_8Bytes@B2
Transmit_LIN_8Bytes@B2:	; 1 bytes @ 0x19
	global	__ELINMIntReceiveMessage@_ELINM_tag
__ELINMIntReceiveMessage@_ELINM_tag:	; 1 bytes @ 0x19
	ds	1
	global	??_BVH2_Appl_Layer
??_BVH2_Appl_Layer:	; 0 bytes @ 0x1A
	global	??_Receive_Diag
??_Receive_Diag:	; 0 bytes @ 0x1A
	global	Transmit_LIN_8Bytes@B3
Transmit_LIN_8Bytes@B3:	; 1 bytes @ 0x1A
	ds	1
	global	??___ltdiv
??___ltdiv:	; 0 bytes @ 0x1B
	global	Transmit_LIN_8Bytes@B4
Transmit_LIN_8Bytes@B4:	; 1 bytes @ 0x1B
	global	Receive_Diag@id
Receive_Diag@id:	; 1 bytes @ 0x1B
	global	___ltdiv@quotient
___ltdiv@quotient:	; 3 bytes @ 0x1B
	ds	1
	global	Transmit_LIN_8Bytes@B5
Transmit_LIN_8Bytes@B5:	; 1 bytes @ 0x1C
	ds	1
	global	Transmit_LIN_8Bytes@B6
Transmit_LIN_8Bytes@B6:	; 1 bytes @ 0x1D
	ds	1
	global	??_InitMotorRun
??_InitMotorRun:	; 0 bytes @ 0x1E
	global	??_init_bldc
??_init_bldc:	; 0 bytes @ 0x1E
	global	??_system_init
??_system_init:	; 0 bytes @ 0x1E
	global	Transmit_LIN_8Bytes@B7
Transmit_LIN_8Bytes@B7:	; 1 bytes @ 0x1E
	global	___ltdiv@counter
___ltdiv@counter:	; 1 bytes @ 0x1E
	ds	1
	global	??_Transmit_LIN_8Bytes
??_Transmit_LIN_8Bytes:	; 0 bytes @ 0x1F
	global	??_PWMReadDC
??_PWMReadDC:	; 0 bytes @ 0x1F
	global	Transmit_LIN_8Bytes@ID
Transmit_LIN_8Bytes@ID:	; 1 bytes @ 0x1F
	ds	1
	global	??_NegativeAnswer
??_NegativeAnswer:	; 0 bytes @ 0x20
	global	NegativeAnswer@LID
NegativeAnswer@LID:	; 1 bytes @ 0x20
	ds	1
	global	??_EOL
??_EOL:	; 0 bytes @ 0x21
	ds	3
	global	??_main
??_main:	; 0 bytes @ 0x24
	ds	4
;;Data sizes: Strings 0, constant 101, data 16, bss 254, persistent 5 stack 0
;;Auto spaces:   Size  Autos    Used
;; COMMON          14     13      13
;; BANK0           80     40      57
;; BANK1           80     58      80
;; BANK2           80      0      80
;; BANK3           80      0      80
;; BANK4           80      0      77
;; BANK5           80      0       0
;; BANK6           16      0       0

;;
;; Pointer list with targets:

;; ?i1_ADC_Read	unsigned int  size(1) Largest target is 0
;;
;; ?i1___lwdiv	unsigned int  size(1) Largest target is 0
;;
;; ?i1___wmul	unsigned int  size(1) Largest target is 0
;;
;; ?___aldiv	long  size(1) Largest target is 0
;;
;; ?___lmul	unsigned long  size(1) Largest target is 0
;;
;; ?___tmul	unsigned um size(1) Largest target is 0
;;
;; ?___ltdiv	unsigned um size(1) Largest target is 0
;;
;; ?___awdiv	int  size(1) Largest target is 0
;;
;; ?__ELINMIntGetPointer	PTR unsigned char  size(1) Largest target is 11
;;		 -> _ELINMIntMessageBuffer(BANK4[11]), 
;;
;; ?_cksum	unsigned int  size(1) Largest target is 0
;;
;; ?___lwdiv	unsigned int  size(1) Largest target is 0
;;
;; ?___wmul	unsigned int  size(1) Largest target is 0
;;
;; ?_ADC_Read	unsigned int  size(1) Largest target is 0
;;
;; sp___ELINMIntGetPointer	PTR unsigned char  size(1) Largest target is 11
;;		 -> _ELINMIntMessageBuffer(BANK4[11]), 
;;
;; pt	PTR unsigned char  size(1) Largest target is 11
;;		 -> NULL(NULL[0]), _ELINMIntMessageBuffer(BANK4[11]), 
;;


;;
;; Critical Paths under _main in COMMON
;;
;;   None.
;;
;; Critical Paths under _interrupt_handler in COMMON
;;
;;   _interrupt_handler->_interrrupt_bldc
;;   _interrrupt_bldc->i1___lwdiv
;;   i1_commutate->i1___wmul
;;   _ELINMIntHandler->__ELINMIntResetProtocol
;;
;; Critical Paths under _main in BANK0
;;
;;   _main->_EOL
;;   _EOL->_NegativeAnswer
;;   _Receive_Diag->__ELINMIntReceiveMessage
;;   _NegativeAnswer->_Transmit_LIN_8Bytes
;;   _BVH2_Appl_Layer->___aldiv
;;   _I_calibrationInit->_ADC_Read
;;   __ELINMIntReceiveMessage->__ELINMIntSendMessage
;;   _Transmit_LIN_8Bytes->__ELINMIntSendMessage
;;   _InitMotorRun->_commutate
;;   _FILTER_Init->_ADC_Read
;;   _timer_init->_clear_timer
;;   _init_ports->_read_eeprom_data
;;   __ELINMIntSendMessage->__ELINMIntCalcIDParity
;;   _PWMReadDC->___ltdiv
;;   _SetDiagAlarm->_PWM_Write_Out
;;   _DiagInit->_PWM_Write_Out
;;   _commutate->___wmul
;;   _Get_Analog_Value->_FILTER_Ubat
;;   ___ltdiv->___tmul
;;   ___awdiv->___wmul
;;   _write_eeprom_data->___lwdiv
;;
;; Critical Paths under _interrupt_handler in BANK0
;;
;;   _interrrupt_bldc->i1_commutate
;;
;; Critical Paths under _main in BANK1
;;
;;   _main->_BVH2_Appl_Layer
;;
;; Critical Paths under _interrupt_handler in BANK1
;;
;;   None.
;;
;; Critical Paths under _main in BANK2
;;
;;   None.
;;
;; Critical Paths under _interrupt_handler in BANK2
;;
;;   None.
;;
;; Critical Paths under _main in BANK3
;;
;;   None.
;;
;; Critical Paths under _interrupt_handler in BANK3
;;
;;   None.
;;
;; Critical Paths under _main in BANK4
;;
;;   None.
;;
;; Critical Paths under _interrupt_handler in BANK4
;;
;;   None.
;;
;; Critical Paths under _main in BANK5
;;
;;   None.
;;
;; Critical Paths under _interrupt_handler in BANK5
;;
;;   None.
;;
;; Critical Paths under _main in BANK6
;;
;;   None.
;;
;; Critical Paths under _interrupt_handler in BANK6
;;
;;   None.

;;
;;Main: autosize = 0, tempsize = 4, incstack = 0, save=0
;;

;;
;;Call Graph Tables:
;;
;; ---------------------------------------------------------------------------------
;; (Depth) Function   	        Calls       Base Space   Used Autos Params    Refs
;; ---------------------------------------------------------------------------------
;; (0) _main                                                 4     4      0   11996
;;                                             36 BANK0      4     4      0
;;                        _system_init
;;                        _clear_timer
;;                             ___wmul
;;                            ___awdiv
;;                   _Get_Analog_Value
;;                          _PWMReadDC
;;                    _BVH2_Appl_Layer
;;                      _InitMotorStop
;;                       _InitMotorRun
;;                       _SetDiagAlarm
;;                   _read_eeprom_data
;;                       _Receive_Diag
;;                                _EOL
;; ---------------------------------------------------------------------------------
;; (1) _system_init                                          0     0      0     509
;;                         _init_ports
;;                       _EnableMCP201
;;                __ELINMIntInitialize
;;                _Oscill_Source_Block
;;                         _timer_init
;;                   _PWM_Capture_init
;;                           _ADC_Init
;;                           _DiagInit
;;                        _FILTER_Init
;;                  _I_calibrationInit
;;                          _init_bldc
;; ---------------------------------------------------------------------------------
;; (1) _EOL                                                 27    27      0    7491
;;                                             33 BANK0      3     3      0
;;                                              0 BANK1     24    24      0
;;                _Transmit_LIN_8Bytes
;;                   _read_eeprom_data
;;                     _NegativeAnswer
;;                              _cksum
;;                      _InitMotorStop
;;                           _ADC_Wait
;;                           _ADC_Read
;;                            ___lwdiv
;;                  _write_eeprom_data
;;                             ___wmul
;;                          _PWMReadDC
;;                      _PWM_Write_Out
;; ---------------------------------------------------------------------------------
;; (1) _Receive_Diag                                         2     2      0     753
;;                                             26 BANK0      2     2      0
;;            __ELINMIntReceiveMessage
;;                __ELINMIntGetPointer
;; ---------------------------------------------------------------------------------
;; (2) _NegativeAnswer                                       1     1      0     957
;;                                             32 BANK0      1     1      0
;;                _Transmit_LIN_8Bytes
;; ---------------------------------------------------------------------------------
;; (2) _init_bldc                                            0     0      0     136
;;                       _InitMotorRun
;; ---------------------------------------------------------------------------------
;; (1) _BVH2_Appl_Layer                                     62    62      0    2176
;;                                             26 BANK0      4     4      0
;;                                              0 BANK1     58    58      0
;;    _Cb37_Pic_etat_monitor_node_fcn1
;;       _Cb27_PWM_Detection_node_fcn1
;;        _Cb53_UbatHandling_node_fcn2
;;    _Cb1_Current_An___High_node_fcn1
;;                             ___wmul
;;                             ___lmul
;;                            ___aldiv
;; ---------------------------------------------------------------------------------
;; (2) _I_calibrationInit                                    1     1      0     102
;;                                             15 BANK0      1     1      0
;;                   _read_eeprom_data
;;                           _ADC_Wait
;;                           _ADC_Read
;; ---------------------------------------------------------------------------------
;; (2) __ELINMIntReceiveMessage                              3     1      2     678
;;                                             23 BANK0      3     1      2
;;               __ELINMIntSendMessage
;; ---------------------------------------------------------------------------------
;; (3) _Transmit_LIN_8Bytes                                  9     1      8     926
;;                                             23 BANK0      9     1      8
;;                __ELINMIntGetPointer
;;               __ELINMIntSendMessage
;; ---------------------------------------------------------------------------------
;; (1) _InitMotorRun                                         0     0      0     136
;;                          _commutate
;; ---------------------------------------------------------------------------------
;; (2) _FILTER_Init                                          0     0      0      37
;;                           _ADC_Wait
;;                           _ADC_Read
;; ---------------------------------------------------------------------------------
;; (2) _Cb53_UbatHandling_node_fcn2                          0     0      0       0
;;        _Cb53_UbatHandling_node_fcn1
;; ---------------------------------------------------------------------------------
;; (2) _timer_init                                           1     1      0      44
;;                                             13 BANK0      1     1      0
;;                        _clear_timer
;; ---------------------------------------------------------------------------------
;; (2) _init_ports                                           0     0      0      65
;;                   _read_eeprom_data
;; ---------------------------------------------------------------------------------
;; (4) __ELINMIntSendMessage                                10     5      5     603
;;                                             13 BANK0     10     5      5
;;              __ELINMIntCalcIDParity
;; ---------------------------------------------------------------------------------
;; (2) _PWMReadDC                                            0     0      0     377
;;                             ___tmul
;;                            ___ltdiv
;; ---------------------------------------------------------------------------------
;; (1) _SetDiagAlarm                                         0     0      0      31
;;                      _PWM_Write_Out
;; ---------------------------------------------------------------------------------
;; (2) _DiagInit                                             0     0      0      31
;;                      _PWM_Write_Out
;; ---------------------------------------------------------------------------------
;; (2) _cksum                                                8     6      2     139
;;                                             12 BANK0      8     6      2
;; ---------------------------------------------------------------------------------
;; (2) _commutate                                           12    12      0     136
;;                                             18 BANK0     12    12      0
;;                             ___wmul
;; ---------------------------------------------------------------------------------
;; (1) _Get_Analog_Value                                     0     0      0       0
;;                        _FILTER_Ubat
;;                      _FILTER_IPhase
;;                        _FILTER_Temp
;; ---------------------------------------------------------------------------------
;; (2) _ADC_Wait                                             0     0      0       0
;; ---------------------------------------------------------------------------------
;; (3) ___tmul                                               9     3      6     136
;;                                             12 BANK0      9     3      6
;; ---------------------------------------------------------------------------------
;; (3) ___ltdiv                                             10     4      6     241
;;                                             21 BANK0     10     4      6
;;                             ___tmul (ARG)
;; ---------------------------------------------------------------------------------
;; (2) ___aldiv                                             14     6      8     300
;;                                             12 BANK0     14     6      8
;; ---------------------------------------------------------------------------------
;; (1) ___awdiv                                              8     4      4     300
;;                                             18 BANK0      8     4      4
;;                             ___wmul (ARG)
;; ---------------------------------------------------------------------------------
;; (2) ___lmul                                              12     4      8      92
;;                                             12 BANK0     12     4      8
;; ---------------------------------------------------------------------------------
;; (2) ___lwdiv                                              7     3      4     241
;;                                             12 BANK0      7     3      4
;; ---------------------------------------------------------------------------------
;; (2) ___wmul                                               6     2      4     136
;;                                             12 BANK0      6     2      4
;; ---------------------------------------------------------------------------------
;; (3) _Cb53_UbatHandling_node_fcn1                          0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _Cb1_Current_An___High_node_fcn1                      0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _Cb27_PWM_Detection_node_fcn1                         0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _Cb37_Pic_etat_monitor_node_fcn1                      0     0      0       0
;; ---------------------------------------------------------------------------------
;; (1) _clear_timer                                          1     1      0      22
;;                                             12 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (2) _PWM_Capture_init                                     1     1      0      22
;;                                             12 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (2) _Oscill_Source_Block                                  0     0      0       0
;; ---------------------------------------------------------------------------------
;; (5) __ELINMIntCalcIDParity                                1     1      0     309
;;                                             12 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (2) __ELINMIntInitialize                                  0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _EnableMCP201                                         3     3      0      72
;;                                             12 BANK0      3     3      0
;; ---------------------------------------------------------------------------------
;; (4) __ELINMIntGetPointer                                  2     1      1      44
;;                                             12 BANK0      2     1      1
;; ---------------------------------------------------------------------------------
;; (2) _write_eeprom_data                                    2     1      1      62
;;                                             19 BANK0      2     1      1
;;                            ___lwdiv (ARG)
;; ---------------------------------------------------------------------------------
;; (2) _read_eeprom_data                                     2     2      0      65
;;                                             12 BANK0      2     2      0
;; ---------------------------------------------------------------------------------
;; (2) _PWM_Write_Out                                        1     1      0      31
;;                                             12 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (2) _InitMotorStop                                        0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _FILTER_Temp                                          0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _FILTER_IPhase                                        0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _FILTER_Ubat                                         12    12      0       0
;;                                             12 BANK0     12    12      0
;; ---------------------------------------------------------------------------------
;; (2) _ADC_Read                                             3     1      2      37
;;                                             12 BANK0      3     1      2
;; ---------------------------------------------------------------------------------
;; (2) _ADC_Init                                             0     0      0       0
;; ---------------------------------------------------------------------------------
;; Estimated maximum stack depth 5
;; ---------------------------------------------------------------------------------
;; (Depth) Function   	        Calls       Base Space   Used Autos Params    Refs
;; ---------------------------------------------------------------------------------
;; (6) _interrupt_handler                                    0     0      0    1557
;;                           _PWM_CTRL
;;                            _Task1ms
;;                    _interrrupt_bldc
;;                    _ELINMIntHandler
;;               _interrupt_PWMCapture
;; ---------------------------------------------------------------------------------
;; (7) _interrrupt_bldc                                      6     6      0    1535
;;                                              7 COMMON     6     6      0
;;                        i1_commutate
;;                           _BLDCWait
;;                         i1_ADC_Wait
;;                         i1_ADC_Read
;;                           i1___wmul
;;                          i1___lwdiv
;; ---------------------------------------------------------------------------------
;; (8) i1_commutate                                         12    12      0     372
;;                                              0 BANK0     12    12      0
;;                           i1___wmul
;; ---------------------------------------------------------------------------------
;; (8) i1_ADC_Wait                                           0     0      0       0
;; ---------------------------------------------------------------------------------
;; (7) _ELINMIntHandler                                      4     4      0      22
;;                                              1 COMMON     4     4      0
;;             __ELINMIntResetProtocol
;; ---------------------------------------------------------------------------------
;; (8) _BLDCWait                                             0     0      0       0
;; ---------------------------------------------------------------------------------
;; (8) i1___lwdiv                                            7     3      4     656
;;                                              0 COMMON     7     3      4
;; ---------------------------------------------------------------------------------
;; (8) i1___wmul                                             6     2      4     372
;;                                              0 COMMON     6     2      4
;; ---------------------------------------------------------------------------------
;; (8) i1_ADC_Read                                           3     1      2      98
;;                                              0 COMMON     3     1      2
;; ---------------------------------------------------------------------------------
;; (8) __ELINMIntResetProtocol                               1     1      0      22
;;                                              0 COMMON     1     1      0
;; ---------------------------------------------------------------------------------
;; (7) _interrupt_PWMCapture                                 0     0      0       0
;; ---------------------------------------------------------------------------------
;; (7) _PWM_CTRL                                             0     0      0       0
;; ---------------------------------------------------------------------------------
;; (7) _Task1ms                                              0     0      0       0
;; ---------------------------------------------------------------------------------
;; Estimated maximum stack depth 8
;; ---------------------------------------------------------------------------------

;; Call Graph Graphs:

;; _main (ROOT)
;;   _system_init
;;     _init_ports
;;       _read_eeprom_data
;;     _EnableMCP201
;;     __ELINMIntInitialize
;;     _Oscill_Source_Block
;;     _timer_init
;;       _clear_timer
;;     _PWM_Capture_init
;;     _ADC_Init
;;     _DiagInit
;;       _PWM_Write_Out
;;     _FILTER_Init
;;       _ADC_Wait
;;       _ADC_Read
;;     _I_calibrationInit
;;       _read_eeprom_data
;;       _ADC_Wait
;;       _ADC_Read
;;     _init_bldc
;;       _InitMotorRun
;;         _commutate
;;           ___wmul
;;   _clear_timer
;;   ___wmul
;;   ___awdiv
;;     ___wmul (ARG)
;;   _Get_Analog_Value
;;     _FILTER_Ubat
;;     _FILTER_IPhase
;;     _FILTER_Temp
;;   _PWMReadDC
;;     ___tmul
;;     ___ltdiv
;;       ___tmul (ARG)
;;   _BVH2_Appl_Layer
;;     _Cb37_Pic_etat_monitor_node_fcn1
;;     _Cb27_PWM_Detection_node_fcn1
;;     _Cb53_UbatHandling_node_fcn2
;;       _Cb53_UbatHandling_node_fcn1
;;     _Cb1_Current_An___High_node_fcn1
;;     ___wmul
;;     ___lmul
;;     ___aldiv
;;   _InitMotorStop
;;   _InitMotorRun
;;     _commutate
;;       ___wmul
;;   _SetDiagAlarm
;;     _PWM_Write_Out
;;   _read_eeprom_data
;;   _Receive_Diag
;;     __ELINMIntReceiveMessage
;;       __ELINMIntSendMessage
;;         __ELINMIntCalcIDParity
;;     __ELINMIntGetPointer
;;   _EOL
;;     _Transmit_LIN_8Bytes
;;       __ELINMIntGetPointer
;;       __ELINMIntSendMessage
;;         __ELINMIntCalcIDParity
;;     _read_eeprom_data
;;     _NegativeAnswer
;;       _Transmit_LIN_8Bytes
;;         __ELINMIntGetPointer
;;         __ELINMIntSendMessage
;;           __ELINMIntCalcIDParity
;;     _cksum
;;     _InitMotorStop
;;     _ADC_Wait
;;     _ADC_Read
;;     ___lwdiv
;;     _write_eeprom_data
;;       ___lwdiv (ARG)
;;     ___wmul
;;     _PWMReadDC
;;       ___tmul
;;       ___ltdiv
;;         ___tmul (ARG)
;;     _PWM_Write_Out
;;
;; _interrupt_handler (ROOT)
;;   _PWM_CTRL
;;   _Task1ms
;;   _interrrupt_bldc
;;     i1_commutate
;;       i1___wmul
;;     _BLDCWait
;;     i1_ADC_Wait
;;     i1_ADC_Read
;;     i1___wmul
;;     i1___lwdiv
;;   _ELINMIntHandler
;;     __ELINMIntResetProtocol
;;   _interrupt_PWMCapture
;;

;; Address spaces:

;;Name               Size   Autos  Total    Cost      Usage
;;BIGRAM             1F0      0       0       0        0.0%
;;EEDATA             100      0       0       0        0.0%
;;NULL                 0      0       0       0        0.0%
;;CODE                 0      0       0       0        0.0%
;;BITCOMMON            E      0       0       1        0.0%
;;BITSFR0              0      0       0       1        0.0%
;;SFR0                 0      0       0       1        0.0%
;;COMMON               E      D       D       2       92.9%
;;BITSFR1              0      0       0       2        0.0%
;;SFR1                 0      0       0       2        0.0%
;;BITSFR2              0      0       0       3        0.0%
;;SFR2                 0      0       0       3        0.0%
;;STACK                0      0       D       3        0.0%
;;BITSFR3              0      0       0       4        0.0%
;;SFR3                 0      0       0       4        0.0%
;;ABS                  0      0     183       4        0.0%
;;BITBANK0            50      0       1       5        1.3%
;;BITSFR4              0      0       0       5        0.0%
;;SFR4                 0      0       0       5        0.0%
;;BANK0               50     28      39       6       71.3%
;;BITSFR5              0      0       0       6        0.0%
;;SFR5                 0      0       0       6        0.0%
;;BITBANK1            50      0       0       7        0.0%
;;BITSFR6              0      0       0       7        0.0%
;;SFR6                 0      0       0       7        0.0%
;;BANK1               50     3A      50       8      100.0%
;;BITSFR7              0      0       0       8        0.0%
;;SFR7                 0      0       0       8        0.0%
;;BITBANK2            50      0       0       9        0.0%
;;BITSFR8              0      0       0       9        0.0%
;;SFR8                 0      0       0       9        0.0%
;;BANK2               50      0      50      10      100.0%
;;BITSFR9              0      0       0      10        0.0%
;;SFR9                 0      0       0      10        0.0%
;;BITBANK3            50      0       0      11        0.0%
;;BITSFR10             0      0       0      11        0.0%
;;SFR10                0      0       0      11        0.0%
;;BANK3               50      0      50      12      100.0%
;;BITSFR11             0      0       0      12        0.0%
;;SFR11                0      0       0      12        0.0%
;;BITBANK4            50      0       0      13        0.0%
;;BITSFR12             0      0       0      13        0.0%
;;SFR12                0      0       0      13        0.0%
;;BANK4               50      0      4D      14       96.3%
;;BITSFR13             0      0       0      14        0.0%
;;SFR13                0      0       0      14        0.0%
;;BITBANK5            50      0       0      15        0.0%
;;BITSFR14             0      0       0      15        0.0%
;;SFR14                0      0       0      15        0.0%
;;BANK5               50      0       0      16        0.0%
;;BITSFR15             0      0       0      16        0.0%
;;SFR15                0      0       0      16        0.0%
;;BITBANK6            10      0       0      17        0.0%
;;BITSFR16             0      0       0      17        0.0%
;;SFR16                0      0       0      17        0.0%
;;BANK6               10      0       0      18        0.0%
;;BITSFR17             0      0       0      18        0.0%
;;SFR17                0      0       0      18        0.0%
;;BITSFR18             0      0       0      19        0.0%
;;SFR18                0      0       0      19        0.0%
;;DATA                 0      0     190      19        0.0%
;;BITSFR19             0      0       0      20        0.0%
;;SFR19                0      0       0      20        0.0%
;;BITSFR20             0      0       0      21        0.0%
;;SFR20                0      0       0      21        0.0%
;;BITSFR21             0      0       0      22        0.0%
;;SFR21                0      0       0      22        0.0%
;;BITSFR22             0      0       0      23        0.0%
;;SFR22                0      0       0      23        0.0%
;;BITSFR23             0      0       0      24        0.0%
;;SFR23                0      0       0      24        0.0%
;;BITSFR24             0      0       0      25        0.0%
;;SFR24                0      0       0      25        0.0%
;;BITSFR25             0      0       0      26        0.0%
;;SFR25                0      0       0      26        0.0%
;;BITSFR26             0      0       0      27        0.0%
;;SFR26                0      0       0      27        0.0%
;;BITSFR27             0      0       0      28        0.0%
;;SFR27                0      0       0      28        0.0%
;;BITSFR28             0      0       0      29        0.0%
;;SFR28                0      0       0      29        0.0%
;;BITSFR29             0      0       0      30        0.0%
;;SFR29                0      0       0      30        0.0%
;;BITSFR30             0      0       0      31        0.0%
;;SFR30                0      0       0      31        0.0%
;;BITSFR31             0      0       0      32        0.0%
;;SFR31                0      0       0      32        0.0%

	global	_main
psect	maintext,global,class=CODE,delta=2
global __pmaintext
__pmaintext:

;; *************** function _main *****************
;; Defined at:
;;		line 621 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, fsr1l, fsr1h, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 17F/0
;;		On exit  : 1F/2
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       4       0       0       0       0       0       0
;;      Totals:         0       4       0       0       0       0       0       0
;;Total ram usage:        4 bytes
;; Hardware stack levels required when called:    9
;; This function calls:
;;		_system_init
;;		_clear_timer
;;		___wmul
;;		___awdiv
;;		_Get_Analog_Value
;;		_PWMReadDC
;;		_BVH2_Appl_Layer
;;		_InitMotorStop
;;		_InitMotorRun
;;		_SetDiagAlarm
;;		_read_eeprom_data
;;		_Receive_Diag
;;		_EOL
;; This function is called by:
;;		Startup code after reset
;; This function uses a non-reentrant model
;;
psect	maintext
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\main.c"
	line	621
	global	__size_of_main
	__size_of_main	equ	__end_of_main-_main
	
_main:	
	opt	stack 7
; Regs used in _main: [allreg]
	line	631
	
l31036:	
;main.c: 631: system_init( );
	fcall	_system_init
	line	632
	
l31038:	
;main.c: 632: clear_timer( 4 );
	movlw	(04h)
	fcall	_clear_timer
	line	633
	
l31040:	
;main.c: 633: ui8_lastTaskvalue = 0;
	movlb 2	; select bank2
	clrf	(_ui8_lastTaskvalue)^0100h
	line	688
	
l31042:	
;main.c: 686: {
;main.c: 688: if( ( ui8_Task_Cont1ms - ui8_lastTaskvalue ) != 0)
	movlb 2	; select bank2
	movf	(_ui8_Task_Cont1ms)^0100h,w
	movlb 0	; select bank0
	movwf	(??_main+0)+0
	clrf	(??_main+0)+0+1
	movf	1+(??_main+0)+0,w
	movwf	(??_main+2)+0+1
	movlb 2	; select bank2
	movf	(_ui8_lastTaskvalue)^0100h,w
	movlb 0	; select bank0
	subwf	0+(??_main+0)+0,w
	movwf	(??_main+2)+0
	skipc
	decf	(??_main+2)+0+1,f
	movf	1+(??_main+2)+0,w
	iorwf	0+(??_main+2)+0,w
	skipnz
	goto	u11331
	goto	u11330
u11331:
	goto	l31042
u11330:
	line	696
	
l31044:	
;main.c: 690: {
;main.c: 696: ui8_lastTaskvalue = ui8_Task_Cont1ms;
	movlb 2	; select bank2
	movf	(_ui8_Task_Cont1ms)^0100h,w
	movwf	(_ui8_lastTaskvalue)^0100h
	line	699
	
l31046:	
;main.c: 699: ui8_Task_Cont3ms++;
	incf	(_ui8_Task_Cont3ms)^0100h,f
	line	700
	
l31048:	
;main.c: 700: ui8_Task_Cont5ms++;
	incf	(_ui8_Task_Cont5ms)^0100h,f
	line	701
	
l31050:	
;main.c: 701: ui8_Task_Cont100ms++;
	movlb 1	; select bank1
	incf	(_ui8_Task_Cont100ms)^080h,f
	line	760
	
l31052:	
;main.c: 760: if( ui8_error_Flags.bits.B1 == 1)
	btfss	(_ui8_error_Flags)^080h,1
	goto	u11341
	goto	u11340
u11341:
	goto	l31058
u11340:
	line	808
	
l31054:	
;main.c: 762: {
;main.c: 808: PR2 = ( unsigned char ) ( 32000000UL / ( 16 * 20000UL ) ) * 2;
	movlw	(0C8h)
	movlb 0	; select bank0
	movwf	(27)	;volatile
	line	809
	
l31056:	
;main.c: 809: ui8_duty_cycle_BLDC = ui8_duty_cycle_mat * 2;
	movlb 2	; select bank2
	lslf	(_ui8_duty_cycle_mat)^0100h,w
	movlb 0	; select bank0
	movwf	(_ui8_duty_cycle_BLDC)
	line	818
;main.c: 818: }
	goto	l31062
	line	869
	
l31058:	
;main.c: 821: else
;main.c: 822: {
;main.c: 869: PR2 = ( unsigned char ) ( 32000000UL / ( 16 * 20000UL ) ) * 5 / 4;
	movlw	(07Dh)
	movlb 0	; select bank0
	movwf	(27)	;volatile
	line	870
	
l31060:	
;main.c: 870: ui8_duty_cycle_BLDC = ui8_duty_cycle_mat * 5 / 4;
	movlw	04h
	movwf	(?___awdiv)
	clrf	(?___awdiv+1)
	movlb 2	; select bank2
	movf	(_ui8_duty_cycle_mat)^0100h,w
	movlb 0	; select bank0
	movwf	(?___wmul)
	clrf	(?___wmul+1)
	movlw	05h
	movwf	0+(?___wmul)+02h
	clrf	1+(?___wmul)+02h
	fcall	___wmul
	movf	(1+(?___wmul)),w
	movwf	1+(?___awdiv)+02h
	movf	(0+(?___wmul)),w
	movwf	0+(?___awdiv)+02h
	fcall	___awdiv
	movf	(0+(?___awdiv)),w
	movwf	(_ui8_duty_cycle_BLDC)
	line	936
	
l31062:	
;main.c: 880: }
;main.c: 936: Get_Analog_Value( );
	fcall	_Get_Analog_Value
	line	982
	
l31064:	
;main.c: 982: if( ui8_Task_Cont3ms > 2 )
	movlw	(03h)
	movlb 2	; select bank2
	subwf	(_ui8_Task_Cont3ms)^0100h,w
	skipc
	goto	u11351
	goto	u11350
u11351:
	goto	l31068
u11350:
	line	986
	
l31066:	
;main.c: 984: {
;main.c: 986: ui8_Task_Cont3ms = 0;
	clrf	(_ui8_Task_Cont3ms)^0100h
	line	1027
	
l31068:	
;main.c: 1021: }
;main.c: 1027: if( ui8_Task_Cont5ms > 4 )
	movlw	(05h)
	subwf	(_ui8_Task_Cont5ms)^0100h,w
	skipc
	goto	u11361
	goto	u11360
u11361:
	goto	l31128
u11360:
	line	1033
	
l31070:	
;main.c: 1029: {
;main.c: 1033: ui8_Task_Cont5ms = 0;
	clrf	(_ui8_Task_Cont5ms)^0100h
	line	1036
	
l31072:	
# 1036 "D:\Projekte\BVH2\BLDC\V0.9.3\src\main.c"
clrwdt ;#
psect	maintext
	line	1041
	
l31074:	
;main.c: 1041: ui8_PWM_dc_mat = PWMReadDC( );
	fcall	_PWMReadDC
	movlb 1	; select bank1
	movwf	(_ui8_PWM_dc_mat)^080h
	line	1042
	
l31076:	
;main.c: 1042: ui16_PWM_Freq_mat = ui16_PWM_Freq_In;
	movlb 3	; select bank3
	movf	(_ui16_PWM_Freq_In+1)^0180h,w
	movwf	(_ui16_PWM_Freq_mat+1)^0180h
	movf	(_ui16_PWM_Freq_In)^0180h,w
	movwf	(_ui16_PWM_Freq_mat)^0180h
	line	1043
	
l31078:	
;main.c: 1043: ui16_Speed_mat = ui16_speed_fil;
	movlb 2	; select bank2
	movf	(_ui16_speed_fil+1)^0100h,w
	movlb 1	; select bank1
	movwf	(_ui16_Speed_mat+1)^080h
	movlb 2	; select bank2
	movf	(_ui16_speed_fil)^0100h,w
	movlb 1	; select bank1
	movwf	(_ui16_Speed_mat)^080h
	line	1044
	
l31080:	
;main.c: 1044: ui16_mat_inpTemp = ui16_NTC_Temp_bldc_mean_cal;
	movlb 3	; select bank3
	movf	(_ui16_NTC_Temp_bldc_mean_cal+1)^0180h,w
	movwf	(_ui16_mat_inpTemp+1)^0180h
	movf	(_ui16_NTC_Temp_bldc_mean_cal)^0180h,w
	movwf	(_ui16_mat_inpTemp)^0180h
	line	1045
	
l31082:	
;main.c: 1045: ui8_Ki_mat = 5;
	movlw	05h
	movlb 4	; select bank4
	movwf	(_ui8_Ki_mat)^0200h
	clrf	(_ui8_Ki_mat+1)^0200h
	line	1046
	
l31084:	
;main.c: 1046: ui8_Kp_mat = 0;
	movlb 2	; select bank2
	clrf	(_ui8_Kp_mat)^0100h
	line	1056
	
l31086:	
;main.c: 1056: if( ui8_error_Flags.bits.B1 == 1 )
	movlb 1	; select bank1
	btfss	(_ui8_error_Flags)^080h,1
	goto	u11371
	goto	u11370
u11371:
	goto	l31090
u11370:
	line	1061
	
l31088:	
;main.c: 1058: {
;main.c: 1061: ui16_Speed_demand_mat = ( PWM_trans_table[ ( ui8_PWM_dc_mat>>1 ) ] )>>1;
	lsrf	(_ui8_PWM_dc_mat)^080h,w
	addlw	low(_PWM_trans_table|8000h)
	movlp	high __stringtab
	callw
	pagesel	$
	lsrf	wreg,f
	movlb 4	; select bank4
	movwf	(_ui16_Speed_demand_mat)^0200h
	clrf	(_ui16_Speed_demand_mat+1)^0200h
	line	1064
;main.c: 1064: ui16_Speed_demand_mat_Max = ( PWM_trans_table[ 95 ] )>>1;
	movlw	low(_PWM_trans_table|8000h+05Fh)
	movlp	high __stringtab
	callw
	pagesel	$
	lsrf	wreg,f
	movwf	(_ui16_Speed_demand_mat_Max)^0200h
	clrf	(_ui16_Speed_demand_mat_Max+1)^0200h
	line	1067
;main.c: 1067: ui16_Speed_demand_mat_min = ( PWM_trans_table[ 11 ] )>>1;
	movlw	low(_PWM_trans_table|8000h+0Bh)
	movlp	high __stringtab
	callw
	pagesel	$
	lsrf	wreg,f
	movwf	(_ui16_Speed_demand_mat_min)^0200h
	clrf	(_ui16_Speed_demand_mat_min+1)^0200h
	line	1069
;main.c: 1069: }
	goto	l31092
	line	1076
	
l31090:	
;main.c: 1072: else
;main.c: 1073: {
;main.c: 1076: ui16_Speed_demand_mat = PWM_trans_table[ ( ui8_PWM_dc_mat>>1 ) ] * 4 / 5;
	movlw	05h
	movlb 0	; select bank0
	movwf	(?___awdiv)
	clrf	(?___awdiv+1)
	movlb 1	; select bank1
	lsrf	(_ui8_PWM_dc_mat)^080h,w
	addlw	low(_PWM_trans_table|8000h)
	movlp	high __stringtab
	callw
	pagesel	$
	movlb 0	; select bank0
	movwf	(??_main+0)+0
	clrf	(??_main+0)+0+1
	lslf	(??_main+0)+0,f
	rlf	(??_main+0)+1,f
	lslf	(??_main+0)+0,f
	rlf	(??_main+0)+1,f
	movf	0+(??_main+0)+0,w
	movwf	0+(?___awdiv)+02h
	movf	1+(??_main+0)+0,w
	movwf	1+(?___awdiv)+02h
	fcall	___awdiv
	movf	(1+(?___awdiv)),w
	movlb 4	; select bank4
	movwf	(_ui16_Speed_demand_mat+1)^0200h
	movlb 0	; select bank0
	movf	(0+(?___awdiv)),w
	movlb 4	; select bank4
	movwf	(_ui16_Speed_demand_mat)^0200h
	line	1079
;main.c: 1079: ui16_Speed_demand_mat_Max = ( PWM_trans_table[ 95 ] ) * 4 / 5;
	movlw	05h
	movlb 0	; select bank0
	movwf	(?___awdiv)
	clrf	(?___awdiv+1)
	movlw	low(_PWM_trans_table|8000h+05Fh)
	movlp	high __stringtab
	callw
	pagesel	$
	movwf	(??_main+0)+0
	clrf	(??_main+0)+0+1
	lslf	(??_main+0)+0,f
	rlf	(??_main+0)+1,f
	lslf	(??_main+0)+0,f
	rlf	(??_main+0)+1,f
	movf	0+(??_main+0)+0,w
	movwf	0+(?___awdiv)+02h
	movf	1+(??_main+0)+0,w
	movwf	1+(?___awdiv)+02h
	fcall	___awdiv
	movf	(1+(?___awdiv)),w
	movlb 4	; select bank4
	movwf	(_ui16_Speed_demand_mat_Max+1)^0200h
	movlb 0	; select bank0
	movf	(0+(?___awdiv)),w
	movlb 4	; select bank4
	movwf	(_ui16_Speed_demand_mat_Max)^0200h
	line	1082
;main.c: 1082: ui16_Speed_demand_mat_min = ( PWM_trans_table[ 11 ] ) * 4 / 5;
	movlw	05h
	movlb 0	; select bank0
	movwf	(?___awdiv)
	clrf	(?___awdiv+1)
	movlw	low(_PWM_trans_table|8000h+0Bh)
	movlp	high __stringtab
	callw
	pagesel	$
	movwf	(??_main+0)+0
	clrf	(??_main+0)+0+1
	lslf	(??_main+0)+0,f
	rlf	(??_main+0)+1,f
	lslf	(??_main+0)+0,f
	rlf	(??_main+0)+1,f
	movf	0+(??_main+0)+0,w
	movwf	0+(?___awdiv)+02h
	movf	1+(??_main+0)+0,w
	movwf	1+(?___awdiv)+02h
	fcall	___awdiv
	movf	(1+(?___awdiv)),w
	movlb 4	; select bank4
	movwf	(_ui16_Speed_demand_mat_min+1)^0200h
	movlb 0	; select bank0
	movf	(0+(?___awdiv)),w
	movlb 4	; select bank4
	movwf	(_ui16_Speed_demand_mat_min)^0200h
	line	1119
	
l31092:	
;main.c: 1084: }
;main.c: 1119: ui8_BattVolt_mat = ( unsigned char )( ui16_fir_Bat_mittel>>2 );
	movlb 3	; select bank3
	movf	(_ui16_fir_Bat_mittel+1)^0180h,w
	movlb 0	; select bank0
	movwf	(??_main+0)+0+1
	movlb 3	; select bank3
	movf	(_ui16_fir_Bat_mittel)^0180h,w
	movlb 0	; select bank0
	movwf	(??_main+0)+0
	lsrf	(??_main+0)+1,f
	rrf	(??_main+0)+0,f
	lsrf	(??_main+0)+1,f
	rrf	(??_main+0)+0,f
	movf	0+(??_main+0)+0,w
	movlb 1	; select bank1
	movwf	(_ui8_BattVolt_mat)^080h
	clrf	(_ui8_BattVolt_mat+1)^080h
	line	1120
	
l31094:	
;main.c: 1120: ui16_mat_Current = ui16_fir_IPhase_mean.w;
	movlb 3	; select bank3
	movf	(_ui16_fir_IPhase_mean+1)^0180h,w
	movlb 1	; select bank1
	movwf	(_ui16_mat_Current+1)^080h
	movlb 3	; select bank3
	movf	(_ui16_fir_IPhase_mean)^0180h,w
	movlb 1	; select bank1
	movwf	(_ui16_mat_Current)^080h
	line	1121
	
l31096:	
;main.c: 1121: bool_mat_currAlarm_bldc = MotorFlags.bits.B0;
	movlw	0
	movlb 0	; select bank0
	btfsc	(_MotorFlags),0
	movlw	1
	movlb 2	; select bank2
	movwf	(_bool_mat_currAlarm_bldc)^0100h
	line	1132
	
l31098:	
;main.c: 1132: bool_ControlLoopMode = 0;
	clrf	(_bool_ControlLoopMode)^0100h
	line	1140
	
l31100:	
;main.c: 1140: BVH2_Appl_Layer( );
	fcall	_BVH2_Appl_Layer
	line	1143
	
l31102:	
;main.c: 1143: ui8_ResetMatlab = 0;
	movlb 2	; select bank2
	clrf	(_ui8_ResetMatlab)^0100h
	line	1149
	
l31104:	
;main.c: 1149: if( ui8_duty_cycle_mat == 0 )
	movf	(_ui8_duty_cycle_mat)^0100h,f
	skipz
	goto	u11381
	goto	u11380
u11381:
	goto	l31108
u11380:
	line	1153
	
l31106:	
;main.c: 1151: {
;main.c: 1153: InitMotorStop( );
	fcall	_InitMotorStop
	line	1156
;main.c: 1156: }
	goto	l31110
	line	1162
	
l31108:	
;main.c: 1159: else
;main.c: 1160: {
;main.c: 1162: InitMotorRun( );
	fcall	_InitMotorRun
	line	1172
	
l31110:	
;main.c: 1165: }
;main.c: 1172: ui8_error_Flags.bits.B0 = bool_PWMin_err_Alarm;
	movlb 2	; select bank2
	movf	(_bool_PWMin_err_Alarm)^0100h,w
	movlb 1	; select bank1
	bcf	(_ui8_error_Flags)^080h,0
	skipz
	bsf	(_ui8_error_Flags)^080h,0
	line	1173
	
l31112:	
;main.c: 1173: ui8_error_Flags.bits.B1 = bool_CPU_TempAlarm;
	movlb 2	; select bank2
	movf	(_bool_CPU_TempAlarm)^0100h,w
	movlb 1	; select bank1
	bcf	(_ui8_error_Flags)^080h,1
	skipz
	bsf	(_ui8_error_Flags)^080h,1
	line	1174
	
l31114:	
;main.c: 1174: ui8_error_Flags.bits.B7 = bool_CPU_TempRedAlarm;
	movlb 2	; select bank2
	movf	(_bool_CPU_TempRedAlarm)^0100h,w
	movlb 1	; select bank1
	bcf	(_ui8_error_Flags)^080h,7
	skipz
	bsf	(_ui8_error_Flags)^080h,7
	line	1175
	
l31116:	
;main.c: 1175: ui8_error_Flags.bits.B2 = bool_HighCurrentAlarm;
	movlb 2	; select bank2
	movf	(_bool_HighCurrentAlarm)^0100h,w
	movlb 1	; select bank1
	bcf	(_ui8_error_Flags)^080h,2
	skipz
	bsf	(_ui8_error_Flags)^080h,2
	line	1176
	
l31118:	
;main.c: 1176: ui8_error_Flags.bits.B3 = bool_MotorStalled;
	movlb 2	; select bank2
	movf	(_bool_MotorStalled)^0100h,w
	movlb 1	; select bank1
	bcf	(_ui8_error_Flags)^080h,3
	skipz
	bsf	(_ui8_error_Flags)^080h,3
	line	1177
	
l31120:	
;main.c: 1177: ui8_error_Flags.bits.B4 = bool_DryRunningAlarm;
	movlb 2	; select bank2
	movf	(_bool_DryRunningAlarm)^0100h,w
	movlb 1	; select bank1
	bcf	(_ui8_error_Flags)^080h,4
	skipz
	bsf	(_ui8_error_Flags)^080h,4
	line	1178
	
l31122:	
;main.c: 1178: ui8_error_Flags.bits.B6 = bool_UbatAlarm;
	movlb 2	; select bank2
	movf	(_bool_UbatAlarm)^0100h,w
	movlb 1	; select bank1
	bcf	(_ui8_error_Flags)^080h,6
	skipz
	bsf	(_ui8_error_Flags)^080h,6
	line	1179
	
l31124:	
;main.c: 1179: ui8_error_Flags.bits.B5 = bool_PWMin_Freq_err_Alarm;
	movlb 2	; select bank2
	movf	(_bool_PWMin_Freq_err_Alarm)^0100h,w
	movlb 1	; select bank1
	bcf	(_ui8_error_Flags)^080h,5
	skipz
	bsf	(_ui8_error_Flags)^080h,5
	line	1184
	
l31126:	
;main.c: 1184: MotorFlags.bits.B0 = 0;
	movlb 0	; select bank0
	bcf	(_MotorFlags),0
	line	1246
	
l31128:	
;main.c: 1240: }
;main.c: 1246: if( ui8_Task_Cont100ms == 25 )
	movlb 1	; select bank1
	movf	(_ui8_Task_Cont100ms)^080h,w
	xorlw	019h&0ffh
	skipz
	goto	u11391
	goto	u11390
u11391:
	goto	l31138
u11390:
	line	1259
	
l31130:	
;main.c: 1248: {
;main.c: 1259: SetDiagAlarm( );
	fcall	_SetDiagAlarm
	line	1298
	
l31132:	
;main.c: 1298: if ((read_eeprom_data(0x55)==0x55) || (RC5 == 0))
	movlw	(055h)
	fcall	_read_eeprom_data
	xorlw	055h&0ffh
	skipnz
	goto	u11401
	goto	u11400
u11401:
	goto	l31138
u11400:
	
l31134:	
	btfsc	(117/8),(117)&7
	goto	u11411
	goto	u11410
u11411:
	goto	l31136
u11410:
	goto	l31138
	line	1311
	
l31136:	
;main.c: 1307: else
;main.c: 1308: {
;main.c: 1311: Receive_Diag(0x11);
	movlw	(011h)
	fcall	_Receive_Diag
	line	1329
	
l31138:	
;main.c: 1315: }
;main.c: 1323: }
;main.c: 1329: if( ui8_Task_Cont100ms == 37 )
	movlb 1	; select bank1
	movf	(_ui8_Task_Cont100ms)^080h,w
	xorlw	025h&0ffh
	skipz
	goto	u11421
	goto	u11420
u11421:
	goto	l31150
u11420:
	line	1337
	
l31140:	
;main.c: 1331: {
;main.c: 1337: if ((read_eeprom_data(0x55)==0x55) || (RC5==0) )
	movlw	(055h)
	fcall	_read_eeprom_data
	xorlw	055h&0ffh
	skipnz
	goto	u11431
	goto	u11430
u11431:
	goto	l31146
u11430:
	
l31142:	
	btfsc	(117/8),(117)&7
	goto	u11441
	goto	u11440
u11441:
	goto	l31144
u11440:
	goto	l31150
	line	1350
	
l31144:	
;main.c: 1346: else
;main.c: 1347: {
;main.c: 1350: EOL();
	fcall	_EOL
	goto	l31150
	line	1367
	
l31146:	
	line	1467
	
l31150:	
;main.c: 1461: }
;main.c: 1467: if( ui8_Task_Cont100ms > 100 )
	movlw	(065h)
	movlb 1	; select bank1
	subwf	(_ui8_Task_Cont100ms)^080h,w
	skipc
	goto	u11451
	goto	u11450
u11451:
	goto	l15487
u11450:
	line	1473
	
l31152:	
;main.c: 1469: {
;main.c: 1473: ui8_Task_Cont100ms = 0;
	clrf	(_ui8_Task_Cont100ms)^080h
	line	1513
	
l15487:	
	line	1541
;main.c: 1513: }
;main.c: 1541: bool_start_demand_mat = 0;
	movlb 2	; select bank2
	clrf	(_bool_start_demand_mat)^0100h
	line	1547
;main.c: 1547: }
	goto	l31042
	global	start
	ljmp	start
	opt stack 0
psect	maintext
	line	1561
GLOBAL	__end_of_main
	__end_of_main:
;; =============== function _main ends ============

	signat	_main,88
	global	_system_init
psect	text2019,local,class=CODE,delta=2
global __ptext2019
__ptext2019:

;; *************** function _system_init *****************
;; Defined at:
;;		line 537 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, fsr1l, fsr1h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 17F/0
;;		On exit  : 1F/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    8
;; This function calls:
;;		_init_ports
;;		_EnableMCP201
;;		__ELINMIntInitialize
;;		_Oscill_Source_Block
;;		_timer_init
;;		_PWM_Capture_init
;;		_ADC_Init
;;		_DiagInit
;;		_FILTER_Init
;;		_I_calibrationInit
;;		_init_bldc
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2019
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\main.c"
	line	537
	global	__size_of_system_init
	__size_of_system_init	equ	__end_of_system_init-_system_init
	
_system_init:	
	opt	stack 7
; Regs used in _system_init: [wreg-status,0+pclath+cstack]
	line	539
	
l30980:	
;main.c: 539: OPTION_REG = 0b10000001;
	movlw	(081h)
	movlb 1	; select bank1
	movwf	(149)^080h	;volatile
	line	548
	
l30982:	
;main.c: 548: ui8_ResetMatlab = 1;
	movlb 2	; select bank2
	clrf	(_ui8_ResetMatlab)^0100h
	incf	(_ui8_ResetMatlab)^0100h,f
	line	549
	
l30984:	
;main.c: 549: GIE = 0;
	bcf	(95/8),(95)&7
	line	550
	
l30986:	
;main.c: 550: INTCON = 0;
	clrf	(11)	;volatile
	line	551
	
l30988:	
;main.c: 551: PIE1 = 0;
	movlb 1	; select bank1
	clrf	(145)^080h	;volatile
	line	552
	
l30990:	
;main.c: 552: PIE2 = 0;
	clrf	(146)^080h	;volatile
	line	553
	
l30992:	
;main.c: 555: ui16_Timer_VaL1 = 0;
	clrf	(147)^080h	;volatile
	line	556
	
l30994:	
;main.c: 556: ui8_Task_Cont3ms = 0;
	movlb 2	; select bank2
	clrf	(_ui8_Task_Cont3ms)^0100h
	line	557
	
l30996:	
;main.c: 557: ui8_Task_Cont5ms = 0;
	clrf	(_ui8_Task_Cont5ms)^0100h
	line	558
	
l30998:	
;main.c: 561: ui8_Sync_Cont10ms = 0;
	movlb 1	; select bank1
	clrf	(_ui8_Task_Cont100ms)^080h
	line	563
	
l31000:	
;main.c: 563: ui16_dryRun_Thresh = 5000;
	movlw	low(01388h)
	movlb 4	; select bank4
	movwf	(_ui16_dryRun_Thresh)^0200h
	movlw	high(01388h)
	movwf	((_ui16_dryRun_Thresh)^0200h)+1
	line	564
	
l31002:	
;main.c: 564: ui16_Current_Thresh = 184;
	movlw	0B8h
	movlb 3	; select bank3
	movwf	(_ui16_Current_Thresh)^0180h
	clrf	(_ui16_Current_Thresh+1)^0180h
	line	566
	
l31004:	
;main.c: 566: init_ports( );
	fcall	_init_ports
	line	570
	
l31006:	
;main.c: 570: EnableMCP201( );
	fcall	_EnableMCP201
	line	571
	
l31008:	
;main.c: 571: _ELINMIntInitialize( );
	fcall	__ELINMIntInitialize
	line	577
	
l31010:	
;main.c: 577: Oscill_Source_Block( );
	fcall	_Oscill_Source_Block
	line	578
	
l31012:	
;main.c: 578: timer_init( 1 );
	movlw	(01h)
	fcall	_timer_init
	line	579
	
l31014:	
;main.c: 579: timer_init( 4 );
	movlw	(04h)
	fcall	_timer_init
	line	580
	
l31016:	
;main.c: 580: timer_init( 6 );
	movlw	(06h)
	fcall	_timer_init
	line	583
	
l31018:	
;main.c: 583: PWM_Capture_init( 5 );
	movlw	(05h)
	fcall	_PWM_Capture_init
	line	584
	
l31020:	
;main.c: 584: ADC_Init( );
	fcall	_ADC_Init
	line	585
	
l31022:	
;main.c: 585: DiagInit( );
	fcall	_DiagInit
	line	586
	
l31024:	
;main.c: 586: FILTER_Init( );
	fcall	_FILTER_Init
	line	589
	
l31026:	
;main.c: 589: ui16_Temp_cal = 0;
	movlb 4	; select bank4
	clrf	(_ui16_Temp_cal)^0200h
	clrf	(_ui16_Temp_cal+1)^0200h
	line	599
	
l31028:	
;main.c: 599: I_calibrationInit();
	fcall	_I_calibrationInit
	line	601
	
l31030:	
;main.c: 601: init_bldc( );
	fcall	_init_bldc
	line	603
	
l31032:	
;main.c: 603: PEIE = 1;
	bsf	(94/8),(94)&7
	line	604
	
l31034:	
;main.c: 604: GIE = 1;
	bsf	(95/8),(95)&7
	line	606
	
l15462:	
	return
	opt stack 0
GLOBAL	__end_of_system_init
	__end_of_system_init:
;; =============== function _system_init ends ============

	signat	_system_init,88
	global	_EOL
psect	text2020,local,class=CODE,delta=2
global __ptext2020
__ptext2020:

;; *************** function _EOL *****************
;; Defined at:
;;		line 458 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  _dcnt           1   17[BANK1 ] unsigned char 
;;  _dcnt           1   16[BANK1 ] unsigned char 
;;  _dcnt           1   15[BANK1 ] unsigned char 
;;  _dcnt           1   14[BANK1 ] unsigned char 
;;  _dcnt           1   13[BANK1 ] unsigned char 
;;  _dcnt           1   12[BANK1 ] unsigned char 
;;  _dcnt           1   11[BANK1 ] unsigned char 
;;  _dcnt           1   10[BANK1 ] unsigned char 
;;  _dcnt           1    9[BANK1 ] unsigned char 
;;  _dcnt           1    8[BANK1 ] unsigned char 
;;  _dcnt           1    7[BANK1 ] unsigned char 
;;  _dcnt           1    6[BANK1 ] unsigned char 
;;  _dcnt           1    5[BANK1 ] unsigned char 
;;  _dcnt           1    4[BANK1 ] unsigned char 
;;  _dcnt           1    3[BANK1 ] unsigned char 
;;  _dcnt           1    2[BANK1 ] unsigned char 
;;  _dcnt           1    1[BANK1 ] unsigned char 
;;  _dcnt           1    0[BANK1 ] unsigned char 
;;  ui8_b_DResB3    1   23[BANK1 ] unsigned char 
;;  ui8_b_DResB2    1   22[BANK1 ] unsigned char 
;;  ui8_b_DResB1    1   21[BANK1 ] unsigned char 
;;  ui8_b_DResB0    1   20[BANK1 ] unsigned char 
;;  ui8_b_DResB4    1   19[BANK1 ] unsigned char 
;;  ui8_b_DResB5    1   18[BANK1 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, fsr1l, fsr1h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1C/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0      24       0       0       0       0       0
;;      Temps:          0       3       0       0       0       0       0       0
;;      Totals:         0       3      24       0       0       0       0       0
;;Total ram usage:       27 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    8
;; This function calls:
;;		_Transmit_LIN_8Bytes
;;		_read_eeprom_data
;;		_NegativeAnswer
;;		_cksum
;;		_InitMotorStop
;;		_ADC_Wait
;;		_ADC_Read
;;		___lwdiv
;;		_write_eeprom_data
;;		___wmul
;;		_PWMReadDC
;;		_PWM_Write_Out
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2020
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
	line	458
	global	__size_of_EOL
	__size_of_EOL	equ	__end_of_EOL-_EOL
	
_EOL:	
	opt	stack 7
; Regs used in _EOL: [wreg-status,0+pclath+cstack]
	line	469
	
l30426:	
;diag.c: 460: unsigned char ui8_b_DResB0_RD;
;diag.c: 461: unsigned char ui8_b_DResB1_RD;
;diag.c: 462: unsigned char ui8_b_DResB2_RD;
;diag.c: 463: unsigned char ui8_b_DResB3_RD;
;diag.c: 464: unsigned char ui8_b_DResB4_RD;
;diag.c: 465: unsigned char ui8_b_DResB5_RD;
;diag.c: 469: if( ui8_b_DResServID_c == 0x3b )
	movlb 2	; select bank2
	movf	(_ui8_b_DResServID_c)^0100h,w
	xorlw	03Bh&0ffh
	skipz
	goto	u10721
	goto	u10720
u10721:
	goto	l30978
u10720:
	line	477
	
l30428:	
;diag.c: 471: {
;diag.c: 477: ui8_selected_lid = ui8_b_DResLocID_c;
	movlb 0	; select bank0
	movf	(_ui8_b_DResLocID_c),w
	movwf	(_ui8_selected_lid)
	line	482
;diag.c: 482: switch( ui8_b_DResLocID_c)
	goto	l30976
	line	492
	
l30430:	
;diag.c: 490: {
;diag.c: 492: if( 0x80 == ui8_selected_lid )
	movf	(_ui8_selected_lid),w
	xorlw	080h&0ffh
	skipz
	goto	u10731
	goto	u10730
u10731:
	goto	l7650
u10730:
	line	496
	
l30432:	
;diag.c: 494: {
;diag.c: 496: ui8_selected_lid = 0x90;
	movlw	(090h)
	movwf	(_ui8_selected_lid)
	line	497
	
l30434:	
;diag.c: 497: ict_stamp = 1;
	movlb 2	; select bank2
	clrf	(_ict_stamp)^0100h
	incf	(_ict_stamp)^0100h,f
	line	499
	
l30436:	
;diag.c: 499: Transmit_LIN_8Bytes( 0x10, 0x3A, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 );
	movlw	(03Ah)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movlw	(080h)
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	clrf	0+(?_Transmit_LIN_8Bytes)+02h
	clrf	0+(?_Transmit_LIN_8Bytes)+03h
	clrf	0+(?_Transmit_LIN_8Bytes)+04h
	clrf	0+(?_Transmit_LIN_8Bytes)+05h
	clrf	0+(?_Transmit_LIN_8Bytes)+06h
	clrf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	503
;diag.c: 503: }
	goto	l7822
	line	516
;diag.c: 506: else
;diag.c: 507: {
	
l7650:	
	line	519
;diag.c: 516: }
;diag.c: 519: break;
	goto	l7822
	line	531
	
l30438:	
;diag.c: 529: {
;diag.c: 531: if( 0x90 == ui8_selected_lid )
	movf	(_ui8_selected_lid),w
	xorlw	090h&0ffh
	skipz
	goto	u10741
	goto	u10740
u10741:
	goto	l7650
u10740:
	goto	l30460
	line	559
	
l30442:	
;diag.c: 557: {
;diag.c: 559: ui8_b_DResB0_RD = read_eeprom_data( 0xf0 );
	movlw	(0F0h)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB0_RD)^080h
	line	560
;diag.c: 560: ui8_b_DResB1_RD = read_eeprom_data( 0xf1 );
	movlw	(0F1h)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB1_RD)^080h
	line	561
;diag.c: 561: ui8_b_DResB2_RD = read_eeprom_data( 0xf2 );
	movlw	(0F2h)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB2_RD)^080h
	line	562
;diag.c: 562: ui8_b_DResB3_RD = read_eeprom_data( 0xf3 );
	movlw	(0F3h)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB3_RD)^080h
	line	563
	
l30444:	
;diag.c: 563: ui8_b_DResB4_RD = 0;
	clrf	(EOL@ui8_b_DResB4_RD)^080h
	line	564
	
l30446:	
;diag.c: 564: ui8_b_DResB5_RD = 0;
	clrf	(EOL@ui8_b_DResB5_RD)^080h
	line	566
	
l30448:	
;diag.c: 566: ict_stamp = 2;
	movlw	(02h)
	movlb 2	; select bank2
	movwf	(_ict_stamp)^0100h
	line	567
;diag.c: 567: break;
	goto	l30462
	line	576
	
l30450:	
;diag.c: 574: {
;diag.c: 576: ui8_b_DResB0_RD = read_eeprom_data( 0xeb );
	movlw	(0EBh)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB0_RD)^080h
	line	577
;diag.c: 577: ui8_b_DResB1_RD = read_eeprom_data( 0xec );
	movlw	(0ECh)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB1_RD)^080h
	line	578
;diag.c: 578: ui8_b_DResB2_RD = read_eeprom_data( 0xed );
	movlw	(0EDh)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB2_RD)^080h
	line	579
;diag.c: 579: ui8_b_DResB3_RD = read_eeprom_data( 0xee );
	movlw	(0EEh)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB3_RD)^080h
	line	580
;diag.c: 580: ui8_b_DResB4_RD = read_eeprom_data( 0xef );
	movlw	(0EFh)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB4_RD)^080h
	line	581
	
l30452:	
;diag.c: 581: ui8_b_DResB5_RD = 0;
	clrf	(EOL@ui8_b_DResB5_RD)^080h
	line	583
	
l30454:	
;diag.c: 583: ui8_selected_lid = 0x91;
	movlw	(091h)
	movlb 0	; select bank0
	movwf	(_ui8_selected_lid)
	line	584
;diag.c: 584: break;
	goto	l30462
	line	594
	
l30456:	
;diag.c: 591: {
;diag.c: 594: NegativeAnswer(0x90);
	movlw	(090h)
	fcall	_NegativeAnswer
	line	595
;diag.c: 595: break;
	goto	l30462
	line	551
	
l30460:	
	movlb 2	; select bank2
	movf	(_ict_stamp)^0100h,w
	; Switch size 1, requested type "space"
; Number of cases is 2, Range of values is 1 to 2
; switch strategies available:
; Name         Instructions Cycles
; simple_byte            7     4 (average)
; direct_byte           13     9 (fixed)
; jumptable            263     9 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	1^0	; case 1
	skipnz
	goto	l30442
	xorlw	2^1	; case 2
	skipnz
	goto	l30450
	goto	l30456
	opt asmopt_on

	line	602
	
l30462:	
;diag.c: 602: if ((ui8_b_DResB0_RD != 0xFF) && (ui8_b_DResB1_RD != 0xFF) && (ui8_b_DResB2_RD != 0xFF) && (ui8_b_DResB3_RD != 0xFF) && (ui8_b_DResB4_RD != 0xFF) && (ui8_b_DResB5_RD != 0xFF))
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB0_RD)^080h,w
	xorlw	0FFh&0ffh
	skipnz
	goto	u10751
	goto	u10750
u10751:
	goto	l30476
u10750:
	
l30464:	
	movf	(EOL@ui8_b_DResB1_RD)^080h,w
	xorlw	0FFh&0ffh
	skipnz
	goto	u10761
	goto	u10760
u10761:
	goto	l30476
u10760:
	
l30466:	
	movf	(EOL@ui8_b_DResB2_RD)^080h,w
	xorlw	0FFh&0ffh
	skipnz
	goto	u10771
	goto	u10770
u10771:
	goto	l30476
u10770:
	
l30468:	
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	xorlw	0FFh&0ffh
	skipnz
	goto	u10781
	goto	u10780
u10781:
	goto	l30476
u10780:
	
l30470:	
	movf	(EOL@ui8_b_DResB4_RD)^080h,w
	xorlw	0FFh&0ffh
	skipnz
	goto	u10791
	goto	u10790
u10791:
	goto	l30476
u10790:
	
l30472:	
	movf	(EOL@ui8_b_DResB5_RD)^080h,w
	xorlw	0FFh&0ffh
	skipnz
	goto	u10801
	goto	u10800
u10801:
	goto	l30476
u10800:
	line	606
	
l30474:	
;diag.c: 604: {
;diag.c: 606: Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD, ui8_b_DResB1_RD, ui8_b_DResB2_RD, ui8_b_DResB3_RD, ui8_b_DResB4_RD, ui8_b_DResB5_RD );
	movlw	(03Ah)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB0_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB1_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+03h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB2_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB4_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+06h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB5_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	609
;diag.c: 609: }
	goto	l7822
	line	615
	
l30476:	
;diag.c: 612: else
;diag.c: 613: {
;diag.c: 615: Transmit_LIN_8Bytes( 0x10, 0x7F, 0x90, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );
	movlw	(07Fh)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movlw	(090h)
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+03h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+06h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	goto	l7822
	line	659
	
l30478:	
;diag.c: 657: {
;diag.c: 659: if( 0x91 == ui8_selected_lid )
	movf	(_ui8_selected_lid),w
	xorlw	091h&0ffh
	skipz
	goto	u10811
	goto	u10810
u10811:
	goto	l7650
u10810:
	line	672
	
l30480:	
;diag.c: 661: {
;diag.c: 672: ui8_b_DResB0_RD = read_eeprom_data( 0xf4 );
	movlw	(0F4h)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB0_RD)^080h
	line	673
;diag.c: 673: ui8_b_DResB1_RD = read_eeprom_data( 0xf5 );
	movlw	(0F5h)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB1_RD)^080h
	line	674
;diag.c: 674: ui8_b_DResB2_RD = read_eeprom_data( 0xf6 );
	movlw	(0F6h)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB2_RD)^080h
	line	675
;diag.c: 675: ui8_b_DResB3_RD = read_eeprom_data( 0xf7 );
	movlw	(0F7h)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB3_RD)^080h
	line	676
;diag.c: 676: ui8_b_DResB4_RD = read_eeprom_data( 0xf8 );
	movlw	(0F8h)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB4_RD)^080h
	line	677
	
l30482:	
	line	679
	
l30484:	
;diag.c: 679: if ((ui8_b_DResB0_RD == 0xFF) && (ui8_b_DResB1_RD == 0xFF) && (ui8_b_DResB2_RD == 0xFF) && (ui8_b_DResB3_RD == 0xFF) && (ui8_b_DResB4_RD == 0xFF))
	movf	(EOL@ui8_b_DResB0_RD)^080h,w
	xorlw	0FFh&0ffh
	skipz
	goto	u10821
	goto	u10820
u10821:
	goto	l30496
u10820:
	
l30486:	
	movf	(EOL@ui8_b_DResB1_RD)^080h,w
	xorlw	0FFh&0ffh
	skipz
	goto	u10831
	goto	u10830
u10831:
	goto	l30496
u10830:
	
l30488:	
	movf	(EOL@ui8_b_DResB2_RD)^080h,w
	xorlw	0FFh&0ffh
	skipz
	goto	u10841
	goto	u10840
u10841:
	goto	l30496
u10840:
	
l30490:	
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	xorlw	0FFh&0ffh
	skipz
	goto	u10851
	goto	u10850
u10851:
	goto	l30496
u10850:
	
l30492:	
	movf	(EOL@ui8_b_DResB4_RD)^080h,w
	xorlw	0FFh&0ffh
	skipz
	goto	u10861
	goto	u10860
u10861:
	goto	l30496
u10860:
	line	683
	
l30494:	
;diag.c: 681: {
;diag.c: 683: Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD, ui8_b_DResB1_RD, ui8_b_DResB2_RD, ui8_b_DResB3_RD, ui8_b_DResB4_RD, ui8_b_DResB5_RD );
	movlw	(03Ah)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB0_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB1_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+03h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB2_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB4_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+06h
	clrf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	686
;diag.c: 686: }
	goto	l30498
	line	692
	
l30496:	
;diag.c: 689: else
;diag.c: 690: {
;diag.c: 692: Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );
	movlw	(07Fh)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+03h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+06h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	705
	
l30498:	
;diag.c: 702: }
;diag.c: 705: ui8_selected_lid = 0x92;
	movlw	(092h)
	movlb 0	; select bank0
	movwf	(_ui8_selected_lid)
	line	708
;diag.c: 708: }
	goto	l7822
	line	836
	
l30500:	
;diag.c: 834: {
;diag.c: 836: if( 0x93 == ui8_selected_lid )
	movf	(_ui8_selected_lid),w
	xorlw	093h&0ffh
	skipz
	goto	u10871
	goto	u10870
u10871:
	goto	l7650
u10870:
	line	850
	
l30502:	
;diag.c: 838: {
;diag.c: 850: sum = cksum( );
	fcall	_cksum
	movf	(1+(?_cksum)),w
	movlb 3	; select bank3
	movwf	(_sum+1)^0180h
	movlb 0	; select bank0
	movf	(0+(?_cksum)),w
	movlb 3	; select bank3
	movwf	(_sum)^0180h
	line	851
	
l30504:	
;diag.c: 851: checksum[ 1 ] = ( (EEADRL=(0x1FFF)&0xff, EEADRH=(0x1FFF)>>8, WREN=0, EECON1 |= 0x80, RD=1, _nop(), _nop(), (EEDATH << 8) | EEDATA) );
	movlw	(0FFh)
	movwf	(401)^0180h	;volatile
	
l30506:	
	movlw	(01Fh)
	movwf	(402)^0180h	;volatile
	
l30508:	
	bcf	(3242/8)^0180h,(3242)&7
	
l30510:	
	bsf	(405)^0180h+(7/8),(7)&7	;volatile
	
l30512:	
	bsf	(3240/8)^0180h,(3240)&7
	
l30514:	
	nop
	
l30516:	
	nop
	
l30518:	
	movlb 3	; select bank3
	movf	(404)^0180h,w	;volatile
	movlb 4	; select bank4
	movwf	1+(_checksum)^0200h+02h
	movlb 3	; select bank3
	movf	(403)^0180h,w	;volatile
	movlb 4	; select bank4
	movwf	0+(_checksum)^0200h+02h
	line	852
	
l30520:	
;diag.c: 852: checksum[ 2 ] = ( (EEADRL=(0x1FFE)&0xff, EEADRH=(0x1FFE)>>8, WREN=0, EECON1 |= 0x80, RD=1, _nop(), _nop(), (EEDATH << 8) | EEDATA) );
	movlw	(0FEh)
	movlb 3	; select bank3
	movwf	(401)^0180h	;volatile
	
l30522:	
	movlw	(01Fh)
	movwf	(402)^0180h	;volatile
	
l30524:	
	bcf	(3242/8)^0180h,(3242)&7
	
l30526:	
	bsf	(405)^0180h+(7/8),(7)&7	;volatile
	
l30528:	
	bsf	(3240/8)^0180h,(3240)&7
	
l30530:	
	nop
	
l30532:	
	nop
	
l30534:	
	movlb 3	; select bank3
	movf	(404)^0180h,w	;volatile
	movlb 4	; select bank4
	movwf	1+(_checksum)^0200h+04h
	movlb 3	; select bank3
	movf	(403)^0180h,w	;volatile
	movlb 4	; select bank4
	movwf	0+(_checksum)^0200h+04h
	line	853
	
l30536:	
;diag.c: 853: checksum[ 0 ] = ( checksum[ 1 ]<<8 ) + checksum[ 2 ];
	movlw	(04h)
	addlw	_checksum&0ffh
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	0+(_checksum)^0200h+02h,w
	movlb 0	; select bank0
	movwf	(??_EOL+0)+0+1
	clrf	(??_EOL+0)+0
	moviw	[0]fsr1
	addwf	0+(??_EOL+0)+0,w
	movlb 4	; select bank4
	movwf	(_checksum)^0200h
	moviw	[1]fsr1
	movlb 0	; select bank0
	addwfc	1+(??_EOL+0)+0,w
	movlb 4	; select bank4
	movwf	(_checksum)^0200h+1
	line	856
	
l30538:	
	line	857
	
l30540:	
	line	858
	
l30542:	
;diag.c: 858: ui8_b_DResB2_RD = checksum[1] ;
	movf	0+(_checksum)^0200h+02h,w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB2_RD)^080h
	line	859
	
l30544:	
;diag.c: 859: ui8_b_DResB3_RD = checksum[2] ;
	movlb 4	; select bank4
	movf	0+(_checksum)^0200h+04h,w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB3_RD)^080h
	line	860
	
l30546:	
;diag.c: 860: ui8_b_DResB4_RD = (unsigned int)((sum)>>8) ;
	movlb 3	; select bank3
	movf	(_sum+1)^0180h,w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB4_RD)^080h
	line	861
	
l30548:	
;diag.c: 861: ui8_b_DResB5_RD = (unsigned int)(sum) ;
	movlb 3	; select bank3
	movf	(_sum)^0180h,w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB5_RD)^080h
	line	865
	
l30550:	
;diag.c: 865: if (checksum[0] == sum)
	movlb 3	; select bank3
	movf	(_sum+1)^0180h,w
	movlb 4	; select bank4
	xorwf	(_checksum+1)^0200h,w
	skipz
	goto	u10885
	movlb 3	; select bank3
	movf	(_sum)^0180h,w
	movlb 4	; select bank4
	xorwf	(_checksum)^0200h,w
u10885:

	skipz
	goto	u10881
	goto	u10880
u10881:
	goto	l30554
u10880:
	line	870
	
l30552:	
;diag.c: 867: {
;diag.c: 869: Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD, ui8_b_DResB1_RD, ui8_b_DResB2_RD,
;diag.c: 870: ui8_b_DResB3_RD, ui8_b_DResB4_RD, ui8_b_DResB5_RD );
	movlw	(03Ah)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	clrf	0+(?_Transmit_LIN_8Bytes)+02h
	incf	0+(?_Transmit_LIN_8Bytes)+02h,f
	clrf	0+(?_Transmit_LIN_8Bytes)+03h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB2_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB4_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+06h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB5_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	873
;diag.c: 873: }
	goto	l30556
	line	879
	
l30554:	
;diag.c: 876: else
;diag.c: 877: {
;diag.c: 879: Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );
	movlw	(07Fh)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+03h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+06h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	892
	
l30556:	
;diag.c: 889: }
;diag.c: 892: ui8_selected_lid = 0x94;
	movlw	(094h)
	movlb 0	; select bank0
	movwf	(_ui8_selected_lid)
	line	896
;diag.c: 896: }
	goto	l7822
	line	925
	
l30558:	
;diag.c: 923: {
;diag.c: 925: if( 0x94 == ui8_selected_lid )
	movf	(_ui8_selected_lid),w
	xorlw	094h&0ffh
	skipz
	goto	u10891
	goto	u10890
u10891:
	goto	l7650
u10890:
	line	940
	
l30560:	
;diag.c: 927: {
;diag.c: 940: ui8_given_supply = (ui8_b_DResB0_c);
	movf	(_ui8_b_DResB0_c),w
	movlb 2	; select bank2
	movwf	(_ui8_given_supply)^0100h
	line	942
	
l30562:	
;diag.c: 942: if (ui8_given_supply == 0)
	movf	(_ui8_given_supply)^0100h,f
	skipz
	goto	u10901
	goto	u10900
u10901:
	goto	l30566
u10900:
	line	946
	
l30564:	
;diag.c: 944: {
;diag.c: 946: ui8_given_supply = 0x6F;
	movlw	(06Fh)
	movwf	(_ui8_given_supply)^0100h
	line	951
	
l30566:	
;diag.c: 948: }
;diag.c: 951: ui8_b_DResB0_RD = ( unsigned char )( ui16_fir_Bat_mittel>>2 );
	movlb 3	; select bank3
	movf	(_ui16_fir_Bat_mittel+1)^0180h,w
	movlb 0	; select bank0
	movwf	(??_EOL+0)+0+1
	movlb 3	; select bank3
	movf	(_ui16_fir_Bat_mittel)^0180h,w
	movlb 0	; select bank0
	movwf	(??_EOL+0)+0
	lsrf	(??_EOL+0)+1,f
	rrf	(??_EOL+0)+0,f
	lsrf	(??_EOL+0)+1,f
	rrf	(??_EOL+0)+0,f
	movf	0+(??_EOL+0)+0,w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB0_RD)^080h
	line	952
	
l30568:	
	line	953
	
l30570:	
	line	954
	
l30572:	
	line	955
	
l30574:	
	line	956
	
l30576:	
	line	961
	
l30578:	
;diag.c: 961: if ((ui8_b_DResB0_RD > ui8_given_supply - 3) && (ui8_b_DResB0_RD < ui8_given_supply + 3) )
	movlb 2	; select bank2
	movf	(_ui8_given_supply)^0100h,w
	addlw	low(-3)
	movlb 0	; select bank0
	movwf	(??_EOL+0)+0
	movlw	high(-3)
	skipnc
	movlw	(high(-3)+1)&0ffh
	movwf	((??_EOL+0)+0)+1
	movf	1+(??_EOL+0)+0,w
	xorlw	80h
	movwf	(??_EOL+2)+0
	movlw	80h
	subwf	(??_EOL+2)+0,w
	skipz
	goto	u10915
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB0_RD)^080h,w
	movlb 0	; select bank0
	subwf	0+(??_EOL+0)+0,w
u10915:

	skipnc
	goto	u10911
	goto	u10910
u10911:
	goto	l30584
u10910:
	
l30580:	
	movlb 2	; select bank2
	movf	(_ui8_given_supply)^0100h,w
	addlw	low(03h)
	movlb 0	; select bank0
	movwf	(??_EOL+0)+0
	movlw	high(03h)
	skipnc
	movlw	(high(03h)+1)&0ffh
	movwf	((??_EOL+0)+0)+1
	movf	1+(??_EOL+0)+0,w
	xorlw	80h
	sublw	080h
	skipz
	goto	u10925
	movf	0+(??_EOL+0)+0,w
	movlb 1	; select bank1
	subwf	(EOL@ui8_b_DResB0_RD)^080h,w
u10925:

	skipnc
	goto	u10921
	goto	u10920
u10921:
	goto	l30584
u10920:
	line	968
	
l30582:	
;diag.c: 965: {
;diag.c: 967: Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD, ui8_b_DResB1_RD, ui8_b_DResB2_RD,
;diag.c: 968: ui8_b_DResB3_RD, ui8_b_DResB4_RD, ui8_b_DResB5_RD );
	movlw	(03Ah)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB0_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	clrf	0+(?_Transmit_LIN_8Bytes)+03h
	clrf	0+(?_Transmit_LIN_8Bytes)+04h
	clrf	0+(?_Transmit_LIN_8Bytes)+05h
	clrf	0+(?_Transmit_LIN_8Bytes)+06h
	clrf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	971
;diag.c: 971: }
	goto	l30586
	line	977
	
l30584:	
;diag.c: 974: else
;diag.c: 975: {
;diag.c: 977: Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );
	movlw	(07Fh)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+03h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+06h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	990
	
l30586:	
;diag.c: 987: }
;diag.c: 990: ui8_selected_lid = 0x95;
	movlw	(095h)
	movlb 0	; select bank0
	movwf	(_ui8_selected_lid)
	line	993
;diag.c: 993: }
	goto	l7822
	line	1024
	
l30588:	
;diag.c: 1022: {
;diag.c: 1024: if( 0x95 == ui8_selected_lid )
	movf	(_ui8_selected_lid),w
	xorlw	095h&0ffh
	skipz
	goto	u10931
	goto	u10930
u10931:
	goto	l30754
u10930:
	goto	l30752
	line	1039
	
l30592:	
;diag.c: 1037: {
;diag.c: 1039: InitMotorStop( );
	fcall	_InitMotorStop
	line	1042
	
l30594:	
;diag.c: 1042: PEIE = 0;
	bcf	(94/8),(94)&7
	line	1043
	
l30596:	
;diag.c: 1043: GIE = 0;
	bcf	(95/8),(95)&7
	line	1044
	
l30598:	
;diag.c: 1044: CCP1CON = 0x00;
	movlb 5	; select bank5
	clrf	(659)^0280h	;volatile
	line	1050
	
l30600:	
;diag.c: 1050: LATB2 = 1;
	movlb 2	; select bank2
	bsf	(2154/8)^0100h,(2154)&7
	line	1051
	
l30602:	
;diag.c: 1051: LATC3 = 1;
	bsf	(2163/8)^0100h,(2163)&7
	line	1054
	
l30604:	
;diag.c: 1054: LATC1 = 0;
	bcf	(2161/8)^0100h,(2161)&7
	line	1055
	
l30606:	
;diag.c: 1055: LATC4 = 0;
	bcf	(2164/8)^0100h,(2164)&7
	line	1057
	
l30608:	
;diag.c: 1061: { unsigned char _dcnt; if( 200 >= 4 ) _dcnt = ( 200 * ( 32000000UL ) / 2 ); else _dcnt = 1; while( --_dcnt > 0 ) { asm( "nop" ); asm( "nop" ); continue; } };
	movlw	(035h)
	movlb 1	; select bank1
	movwf	(157)^080h	;volatile
	line	1061
	
l30610:	
	clrf	(EOL@_dcnt)^080h
	goto	l7684
	
l7685:	
# 1061 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
# 1061 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
psect	text2020
	
l7684:	
	movlb 1	; select bank1
	decfsz	(EOL@_dcnt)^080h,f
	goto	u10941
	goto	u10940
u10941:
	goto	l7685
u10940:
	line	1062
	
l30612:	
	clrf	(EOL@_dcnt_9399)^080h
	goto	l7689
	
l7690:	
# 1062 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
# 1062 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
psect	text2020
	
l7689:	
	movlb 1	; select bank1
	decfsz	(EOL@_dcnt_9399)^080h,f
	goto	u10951
	goto	u10950
u10951:
	goto	l7690
u10950:
	line	1063
	
l30614:	
	clrf	(EOL@_dcnt_9400)^080h
	goto	l7694
	
l7695:	
# 1063 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
# 1063 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
psect	text2020
	
l7694:	
	movlb 1	; select bank1
	decfsz	(EOL@_dcnt_9400)^080h,f
	goto	u10961
	goto	u10960
u10961:
	goto	l7695
u10960:
	line	1064
	
l30616:	
	clrf	(EOL@_dcnt_9401)^080h
	goto	l7699
	
l7700:	
# 1064 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
# 1064 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
psect	text2020
	
l7699:	
	movlb 1	; select bank1
	decfsz	(EOL@_dcnt_9401)^080h,f
	goto	u10971
	goto	u10970
u10971:
	goto	l7700
u10970:
	line	1065
	
l30618:	
	clrf	(EOL@_dcnt_9402)^080h
	goto	l7704
	
l7705:	
# 1065 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
# 1065 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
psect	text2020
	
l7704:	
	movlb 1	; select bank1
	decfsz	(EOL@_dcnt_9402)^080h,f
	goto	u10981
	goto	u10980
u10981:
	goto	l7705
u10980:
	line	1066
	
l30620:	
	clrf	(EOL@_dcnt_9403)^080h
	goto	l7709
	
l7710:	
# 1066 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
# 1066 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
psect	text2020
	
l7709:	
	movlb 1	; select bank1
	decfsz	(EOL@_dcnt_9403)^080h,f
	goto	u10991
	goto	u10990
u10991:
	goto	l7710
u10990:
	line	1069
	
l30622:	
;diag.c: 1069: ADC_Wait();
	fcall	_ADC_Wait
	line	1070
	
l30624:	
;diag.c: 1070: ( GO_nDONE = 1 );
	movlb 1	; select bank1
	bsf	(1257/8)^080h,(1257)&7
	line	1071
	
l30626:	
;diag.c: 1071: ui16_IPhase2_bldc_cal.w = ADC_Read() - ui8_current_cal[1];
	fcall	_ADC_Read
	movf	(1+(?_ADC_Read)),w
	movlb 3	; select bank3
	movwf	(_ui16_IPhase2_bldc_cal+1)^0180h
	movlb 0	; select bank0
	movf	(0+(?_ADC_Read)),w
	movlb 3	; select bank3
	movwf	(_ui16_IPhase2_bldc_cal)^0180h
	movf	0+(_ui8_current_cal)^0180h+01h,w
	subwf	(_ui16_IPhase2_bldc_cal)^0180h,f
	skipc
	decf	(_ui16_IPhase2_bldc_cal+1)^0180h,f
	line	1077
	
l30628:	
;diag.c: 1077: LATC2 = 1;
	movlb 2	; select bank2
	bsf	(2162/8)^0100h,(2162)&7
	line	1078
	
l30630:	
;diag.c: 1078: LATC1 = 1;
	bsf	(2161/8)^0100h,(2161)&7
	line	1081
	
l30632:	
;diag.c: 1081: LATC3 = 0;
	bcf	(2163/8)^0100h,(2163)&7
	line	1082
	
l30634:	
;diag.c: 1082: LATC4 = 0;
	bcf	(2164/8)^0100h,(2164)&7
	line	1084
;diag.c: 1088: { unsigned char _dcnt; if( 200 >= 4 ) _dcnt = ( 200 * ( 32000000UL ) / 2 ); else _dcnt = 1; while( --_dcnt > 0 ) { asm( "nop" ); asm( "nop" ); continue; } };
	movlw	(09h)
	movlb 1	; select bank1
	movwf	(157)^080h	;volatile
	line	1088
	
l30636:	
	clrf	(EOL@_dcnt_9404)^080h
	goto	l7714
	
l7715:	
# 1088 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
# 1088 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
psect	text2020
	
l7714:	
	movlb 1	; select bank1
	decfsz	(EOL@_dcnt_9404)^080h,f
	goto	u11001
	goto	u11000
u11001:
	goto	l7715
u11000:
	line	1089
	
l30638:	
	clrf	(EOL@_dcnt_9405)^080h
	goto	l7719
	
l7720:	
# 1089 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
# 1089 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
psect	text2020
	
l7719:	
	movlb 1	; select bank1
	decfsz	(EOL@_dcnt_9405)^080h,f
	goto	u11011
	goto	u11010
u11011:
	goto	l7720
u11010:
	line	1090
	
l30640:	
	clrf	(EOL@_dcnt_9406)^080h
	goto	l7724
	
l7725:	
# 1090 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
# 1090 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
psect	text2020
	
l7724:	
	movlb 1	; select bank1
	decfsz	(EOL@_dcnt_9406)^080h,f
	goto	u11021
	goto	u11020
u11021:
	goto	l7725
u11020:
	line	1091
	
l30642:	
	clrf	(EOL@_dcnt_9407)^080h
	goto	l7729
	
l7730:	
# 1091 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
# 1091 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
psect	text2020
	
l7729:	
	movlb 1	; select bank1
	decfsz	(EOL@_dcnt_9407)^080h,f
	goto	u11031
	goto	u11030
u11031:
	goto	l7730
u11030:
	line	1092
	
l30644:	
	clrf	(EOL@_dcnt_9408)^080h
	goto	l7734
	
l7735:	
# 1092 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
# 1092 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
psect	text2020
	
l7734:	
	movlb 1	; select bank1
	decfsz	(EOL@_dcnt_9408)^080h,f
	goto	u11041
	goto	u11040
u11041:
	goto	l7735
u11040:
	line	1093
	
l30646:	
	clrf	(EOL@_dcnt_9409)^080h
	goto	l7739
	
l7740:	
# 1093 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
# 1093 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
psect	text2020
	
l7739:	
	movlb 1	; select bank1
	decfsz	(EOL@_dcnt_9409)^080h,f
	goto	u11051
	goto	u11050
u11051:
	goto	l7740
u11050:
	line	1096
	
l30648:	
;diag.c: 1096: ADC_Wait();
	fcall	_ADC_Wait
	line	1097
	
l30650:	
;diag.c: 1097: ( GO_nDONE = 1 );
	movlb 1	; select bank1
	bsf	(1257/8)^080h,(1257)&7
	line	1098
	
l30652:	
;diag.c: 1098: ui16_IPhase1_bldc_cal.w = ADC_Read() - ui8_current_cal[0];
	fcall	_ADC_Read
	movf	(1+(?_ADC_Read)),w
	movlb 3	; select bank3
	movwf	(_ui16_IPhase1_bldc_cal+1)^0180h
	movlb 0	; select bank0
	movf	(0+(?_ADC_Read)),w
	movlb 3	; select bank3
	movwf	(_ui16_IPhase1_bldc_cal)^0180h
	movf	(_ui8_current_cal)^0180h,w
	subwf	(_ui16_IPhase1_bldc_cal)^0180h,f
	skipc
	decf	(_ui16_IPhase1_bldc_cal+1)^0180h,f
	line	1104
	
l30654:	
;diag.c: 1104: LATB1 = 1;
	movlb 2	; select bank2
	bsf	(2153/8)^0100h,(2153)&7
	line	1105
	
l30656:	
;diag.c: 1105: LATC4 = 1;
	bsf	(2164/8)^0100h,(2164)&7
	line	1107
	
l30658:	
;diag.c: 1107: LATC1 = 0;
	bcf	(2161/8)^0100h,(2161)&7
	line	1108
	
l30660:	
;diag.c: 1108: LATC3 = 0;
	bcf	(2163/8)^0100h,(2163)&7
	line	1110
;diag.c: 1115: { unsigned char _dcnt; if( 200 >= 4 ) _dcnt = ( 200 * ( 32000000UL ) / 2 ); else _dcnt = 1; while( --_dcnt > 0 ) { asm( "nop" ); asm( "nop" ); continue; } };
	movlw	(011h)
	movlb 1	; select bank1
	movwf	(157)^080h	;volatile
	line	1115
	
l30662:	
	clrf	(EOL@_dcnt_9410)^080h
	goto	l7744
	
l7745:	
# 1115 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
# 1115 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
psect	text2020
	
l7744:	
	movlb 1	; select bank1
	decfsz	(EOL@_dcnt_9410)^080h,f
	goto	u11061
	goto	u11060
u11061:
	goto	l7745
u11060:
	line	1116
	
l30664:	
	clrf	(EOL@_dcnt_9411)^080h
	goto	l7749
	
l7750:	
# 1116 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
# 1116 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
psect	text2020
	
l7749:	
	movlb 1	; select bank1
	decfsz	(EOL@_dcnt_9411)^080h,f
	goto	u11071
	goto	u11070
u11071:
	goto	l7750
u11070:
	line	1117
	
l30666:	
	clrf	(EOL@_dcnt_9412)^080h
	goto	l7754
	
l7755:	
# 1117 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
# 1117 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
psect	text2020
	
l7754:	
	movlb 1	; select bank1
	decfsz	(EOL@_dcnt_9412)^080h,f
	goto	u11081
	goto	u11080
u11081:
	goto	l7755
u11080:
	line	1118
	
l30668:	
	clrf	(EOL@_dcnt_9413)^080h
	goto	l7759
	
l7760:	
# 1118 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
# 1118 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
psect	text2020
	
l7759:	
	movlb 1	; select bank1
	decfsz	(EOL@_dcnt_9413)^080h,f
	goto	u11091
	goto	u11090
u11091:
	goto	l7760
u11090:
	line	1119
	
l30670:	
	clrf	(EOL@_dcnt_9414)^080h
	goto	l7764
	
l7765:	
# 1119 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
# 1119 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
psect	text2020
	
l7764:	
	movlb 1	; select bank1
	decfsz	(EOL@_dcnt_9414)^080h,f
	goto	u11101
	goto	u11100
u11101:
	goto	l7765
u11100:
	line	1120
	
l30672:	
	clrf	(EOL@_dcnt_9415)^080h
	goto	l7769
	
l7770:	
# 1120 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
# 1120 "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
nop ;#
psect	text2020
	
l7769:	
	movlb 1	; select bank1
	decfsz	(EOL@_dcnt_9415)^080h,f
	goto	u11111
	goto	u11110
u11111:
	goto	l7770
u11110:
	line	1123
	
l30674:	
;diag.c: 1123: ADC_Wait();
	fcall	_ADC_Wait
	line	1124
	
l30676:	
;diag.c: 1124: ( GO_nDONE = 1 );
	movlb 1	; select bank1
	bsf	(1257/8)^080h,(1257)&7
	line	1125
	
l30678:	
;diag.c: 1125: ui16_IPhase3_bldc_cal.w = ADC_Read() - ui8_current_cal[2];
	fcall	_ADC_Read
	movf	(1+(?_ADC_Read)),w
	movlb 3	; select bank3
	movwf	(_ui16_IPhase3_bldc_cal+1)^0180h
	movlb 0	; select bank0
	movf	(0+(?_ADC_Read)),w
	movlb 3	; select bank3
	movwf	(_ui16_IPhase3_bldc_cal)^0180h
	movf	0+(_ui8_current_cal)^0180h+02h,w
	subwf	(_ui16_IPhase3_bldc_cal)^0180h,f
	skipc
	decf	(_ui16_IPhase3_bldc_cal+1)^0180h,f
	line	1128
	
l30680:	
;diag.c: 1128: LATC = LATC & 0b11100101;
	movlw	(0E5h)
	movlb 2	; select bank2
	andwf	(270)^0100h,f	;volatile
	line	1131
	
l30682:	
;diag.c: 1131: CCP1CON = 0x0c;
	movlw	(0Ch)
	movlb 5	; select bank5
	movwf	(659)^0280h	;volatile
	line	1132
	
l30684:	
;diag.c: 1132: PEIE = 1;
	bsf	(94/8),(94)&7
	line	1133
	
l30686:	
;diag.c: 1133: GIE = 1;
	bsf	(95/8),(95)&7
	line	1136
	
l30688:	
;diag.c: 1136: phaseCal = 1;
	movlb 1	; select bank1
	clrf	(_phaseCal)^080h
	incf	(_phaseCal)^080h,f
	line	1139
;diag.c: 1139: break;
	goto	l7822
	line	1151
	
l30690:	
;diag.c: 1146: {
;diag.c: 1151: write_eeprom_data( 0x02, ( unsigned char )( ( 0x7800 / ui16_IPhase1_bldc_cal.w )>>8 ) );
	movlb 3	; select bank3
	movf	(_ui16_IPhase1_bldc_cal+1)^0180h,w
	movlb 0	; select bank0
	movwf	(?___lwdiv+1)
	movlb 3	; select bank3
	movf	(_ui16_IPhase1_bldc_cal)^0180h,w
	movlb 0	; select bank0
	movwf	(?___lwdiv)
	movlw	low(07800h)
	movwf	0+(?___lwdiv)+02h
	movlw	high(07800h)
	movwf	(0+(?___lwdiv)+02h)+1
	fcall	___lwdiv
	movf	(1+(?___lwdiv)),w
	movwf	(?_write_eeprom_data)
	movlw	(02h)
	fcall	_write_eeprom_data
	line	1152
;diag.c: 1152: write_eeprom_data( 0x03, ( unsigned char )( 0x7800 / ui16_IPhase1_bldc_cal.w ) );
	movf	(_ui16_IPhase1_bldc_cal+1)^0180h,w
	movlb 0	; select bank0
	movwf	(?___lwdiv+1)
	movlb 3	; select bank3
	movf	(_ui16_IPhase1_bldc_cal)^0180h,w
	movlb 0	; select bank0
	movwf	(?___lwdiv)
	movlw	low(07800h)
	movwf	0+(?___lwdiv)+02h
	movlw	high(07800h)
	movwf	(0+(?___lwdiv)+02h)+1
	fcall	___lwdiv
	movf	(0+(?___lwdiv)),w
	movwf	(?_write_eeprom_data)
	movlw	(03h)
	fcall	_write_eeprom_data
	line	1155
	
l30692:	
;diag.c: 1155: ui16_I_cal_Ph1 = ( read_eeprom_data( 0x02 )<<8 ) | read_eeprom_data( 0x03 );
	movlw	(03h)
	fcall	_read_eeprom_data
	movwf	(??_EOL+0)+0
	movlw	(02h)
	fcall	_read_eeprom_data
	movlb 2	; select bank2
	movwf	(_ui16_I_cal_Ph1+1)^0100h
	movlb 0	; select bank0
	movf	0+(??_EOL+0)+0,w
	movlb 2	; select bank2
	movwf	(_ui16_I_cal_Ph1)^0100h
	line	1167
	
l30694:	
;diag.c: 1167: ui8_b_DResB0_RD = phaseCal;
	movlb 1	; select bank1
	movf	(_phaseCal)^080h,w
	movwf	(EOL@ui8_b_DResB0_RD)^080h
	line	1168
	
l30696:	
	line	1169
	
l30698:	
;diag.c: 1169: ui8_b_DResB2_RD = ( unsigned char )( ( ui16_IPhase1_bldc_cal.w * ui16_I_cal_Ph1 )>>8 );
	movlb 3	; select bank3
	movf	(_ui16_IPhase1_bldc_cal+1)^0180h,w
	movlb 0	; select bank0
	movwf	(?___wmul+1)
	movlb 3	; select bank3
	movf	(_ui16_IPhase1_bldc_cal)^0180h,w
	movlb 0	; select bank0
	movwf	(?___wmul)
	movlb 2	; select bank2
	movf	(_ui16_I_cal_Ph1+1)^0100h,w
	movlb 0	; select bank0
	movwf	1+(?___wmul)+02h
	movlb 2	; select bank2
	movf	(_ui16_I_cal_Ph1)^0100h,w
	movlb 0	; select bank0
	movwf	0+(?___wmul)+02h
	fcall	___wmul
	movf	(1+(?___wmul)),w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB2_RD)^080h
	line	1170
	
l30700:	
;diag.c: 1170: ui8_b_DResB3_RD = ( unsigned char )( ui16_I_cal_Ph1 >> 8);
	movlb 2	; select bank2
	movf	(_ui16_I_cal_Ph1+1)^0100h,w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB3_RD)^080h
	line	1171
	
l30702:	
;diag.c: 1171: ui8_b_DResB4_RD = ( unsigned char )( ui16_I_cal_Ph1 );
	movlb 2	; select bank2
	movf	(_ui16_I_cal_Ph1)^0100h,w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB4_RD)^080h
	line	1172
	
l30704:	
	line	1180
	
l30706:	
;diag.c: 1177: Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD,
;diag.c: 1178: ui8_b_DResB1_RD, ui8_b_DResB2_RD,
;diag.c: 1179: ui8_b_DResB3_RD, ui8_b_DResB4_RD,
;diag.c: 1180: ui8_b_DResB5_RD );
	movlw	(03Ah)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB0_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	clrf	0+(?_Transmit_LIN_8Bytes)+03h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB2_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB4_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+06h
	clrf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	1182
	
l30708:	
;diag.c: 1182: phaseCal = 2;
	movlw	(02h)
	movlb 1	; select bank1
	movwf	(_phaseCal)^080h
	line	1184
;diag.c: 1184: break;
	goto	l7822
	line	1195
	
l30710:	
;diag.c: 1191: {
;diag.c: 1195: write_eeprom_data( 0x04, ( unsigned char )( ( 0x7800 / ui16_IPhase2_bldc_cal.w )>>8 ) );
	movlb 3	; select bank3
	movf	(_ui16_IPhase2_bldc_cal+1)^0180h,w
	movlb 0	; select bank0
	movwf	(?___lwdiv+1)
	movlb 3	; select bank3
	movf	(_ui16_IPhase2_bldc_cal)^0180h,w
	movlb 0	; select bank0
	movwf	(?___lwdiv)
	movlw	low(07800h)
	movwf	0+(?___lwdiv)+02h
	movlw	high(07800h)
	movwf	(0+(?___lwdiv)+02h)+1
	fcall	___lwdiv
	movf	(1+(?___lwdiv)),w
	movwf	(?_write_eeprom_data)
	movlw	(04h)
	fcall	_write_eeprom_data
	line	1196
;diag.c: 1196: write_eeprom_data( 0x05, ( unsigned char )( 0x7800 / ui16_IPhase2_bldc_cal.w ) );
	movf	(_ui16_IPhase2_bldc_cal+1)^0180h,w
	movlb 0	; select bank0
	movwf	(?___lwdiv+1)
	movlb 3	; select bank3
	movf	(_ui16_IPhase2_bldc_cal)^0180h,w
	movlb 0	; select bank0
	movwf	(?___lwdiv)
	movlw	low(07800h)
	movwf	0+(?___lwdiv)+02h
	movlw	high(07800h)
	movwf	(0+(?___lwdiv)+02h)+1
	fcall	___lwdiv
	movf	(0+(?___lwdiv)),w
	movwf	(?_write_eeprom_data)
	movlw	(05h)
	fcall	_write_eeprom_data
	line	1199
	
l30712:	
;diag.c: 1199: ui16_I_cal_Ph2 = ( read_eeprom_data( 0x04 )<<8 ) | read_eeprom_data( 0x05 );
	movlw	(05h)
	fcall	_read_eeprom_data
	movwf	(??_EOL+0)+0
	movlw	(04h)
	fcall	_read_eeprom_data
	movlb 2	; select bank2
	movwf	(_ui16_I_cal_Ph2+1)^0100h
	movlb 0	; select bank0
	movf	0+(??_EOL+0)+0,w
	movlb 2	; select bank2
	movwf	(_ui16_I_cal_Ph2)^0100h
	line	1211
	
l30714:	
;diag.c: 1211: ui8_b_DResB0_RD = phaseCal;
	movlb 1	; select bank1
	movf	(_phaseCal)^080h,w
	movwf	(EOL@ui8_b_DResB0_RD)^080h
	line	1212
	
l30716:	
	line	1213
	
l30718:	
;diag.c: 1213: ui8_b_DResB2_RD = ( unsigned char )( ( ( ui16_IPhase2_bldc_cal.w * ui16_I_cal_Ph2 )>>8 ) );
	movlb 3	; select bank3
	movf	(_ui16_IPhase2_bldc_cal+1)^0180h,w
	movlb 0	; select bank0
	movwf	(?___wmul+1)
	movlb 3	; select bank3
	movf	(_ui16_IPhase2_bldc_cal)^0180h,w
	movlb 0	; select bank0
	movwf	(?___wmul)
	movlb 2	; select bank2
	movf	(_ui16_I_cal_Ph2+1)^0100h,w
	movlb 0	; select bank0
	movwf	1+(?___wmul)+02h
	movlb 2	; select bank2
	movf	(_ui16_I_cal_Ph2)^0100h,w
	movlb 0	; select bank0
	movwf	0+(?___wmul)+02h
	fcall	___wmul
	movf	(1+(?___wmul)),w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB2_RD)^080h
	line	1214
	
l30720:	
;diag.c: 1214: ui8_b_DResB3_RD = ( unsigned char )( ui16_I_cal_Ph2>>8 );
	movlb 2	; select bank2
	movf	(_ui16_I_cal_Ph2+1)^0100h,w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB3_RD)^080h
	line	1215
	
l30722:	
;diag.c: 1215: ui8_b_DResB4_RD = ( unsigned char )( ui16_I_cal_Ph2 );
	movlb 2	; select bank2
	movf	(_ui16_I_cal_Ph2)^0100h,w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB4_RD)^080h
	line	1216
	
l30724:	
	line	1224
	
l30726:	
;diag.c: 1221: Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD,
;diag.c: 1222: ui8_b_DResB1_RD, ui8_b_DResB2_RD,
;diag.c: 1223: ui8_b_DResB3_RD, ui8_b_DResB4_RD,
;diag.c: 1224: ui8_b_DResB5_RD );
	movlw	(03Ah)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB0_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	clrf	0+(?_Transmit_LIN_8Bytes)+03h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB2_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB4_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+06h
	clrf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	1227
	
l30728:	
;diag.c: 1227: phaseCal = 3;
	movlw	(03h)
	movlb 1	; select bank1
	movwf	(_phaseCal)^080h
	line	1229
;diag.c: 1229: break;
	goto	l7822
	line	1240
	
l30730:	
;diag.c: 1236: {
;diag.c: 1240: write_eeprom_data( 0x06, ( unsigned char )( ( 0x7800 / ui16_IPhase3_bldc_cal.w )>>8 ) );
	movlb 3	; select bank3
	movf	(_ui16_IPhase3_bldc_cal+1)^0180h,w
	movlb 0	; select bank0
	movwf	(?___lwdiv+1)
	movlb 3	; select bank3
	movf	(_ui16_IPhase3_bldc_cal)^0180h,w
	movlb 0	; select bank0
	movwf	(?___lwdiv)
	movlw	low(07800h)
	movwf	0+(?___lwdiv)+02h
	movlw	high(07800h)
	movwf	(0+(?___lwdiv)+02h)+1
	fcall	___lwdiv
	movf	(1+(?___lwdiv)),w
	movwf	(?_write_eeprom_data)
	movlw	(06h)
	fcall	_write_eeprom_data
	line	1241
;diag.c: 1241: write_eeprom_data( 0x07, ( unsigned char )( 0x7800 / ui16_IPhase3_bldc_cal.w ) );
	movf	(_ui16_IPhase3_bldc_cal+1)^0180h,w
	movlb 0	; select bank0
	movwf	(?___lwdiv+1)
	movlb 3	; select bank3
	movf	(_ui16_IPhase3_bldc_cal)^0180h,w
	movlb 0	; select bank0
	movwf	(?___lwdiv)
	movlw	low(07800h)
	movwf	0+(?___lwdiv)+02h
	movlw	high(07800h)
	movwf	(0+(?___lwdiv)+02h)+1
	fcall	___lwdiv
	movf	(0+(?___lwdiv)),w
	movwf	(?_write_eeprom_data)
	movlw	(07h)
	fcall	_write_eeprom_data
	line	1245
	
l30732:	
;diag.c: 1245: ui16_I_cal_Ph3 = ( read_eeprom_data( 0x06 )<<8 ) | read_eeprom_data( 0x07 );
	movlw	(07h)
	fcall	_read_eeprom_data
	movwf	(??_EOL+0)+0
	movlw	(06h)
	fcall	_read_eeprom_data
	movlb 2	; select bank2
	movwf	(_ui16_I_cal_Ph3+1)^0100h
	movlb 0	; select bank0
	movf	0+(??_EOL+0)+0,w
	movlb 2	; select bank2
	movwf	(_ui16_I_cal_Ph3)^0100h
	line	1258
	
l30734:	
;diag.c: 1258: ui8_b_DResB0_RD = phaseCal;
	movlb 1	; select bank1
	movf	(_phaseCal)^080h,w
	movwf	(EOL@ui8_b_DResB0_RD)^080h
	line	1259
	
l30736:	
	line	1260
	
l30738:	
;diag.c: 1260: ui8_b_DResB2_RD = ( unsigned char )( ( ( ui16_IPhase3_bldc_cal.w * ui16_I_cal_Ph3 )>>8 ) );
	movlb 3	; select bank3
	movf	(_ui16_IPhase3_bldc_cal+1)^0180h,w
	movlb 0	; select bank0
	movwf	(?___wmul+1)
	movlb 3	; select bank3
	movf	(_ui16_IPhase3_bldc_cal)^0180h,w
	movlb 0	; select bank0
	movwf	(?___wmul)
	movlb 2	; select bank2
	movf	(_ui16_I_cal_Ph3+1)^0100h,w
	movlb 0	; select bank0
	movwf	1+(?___wmul)+02h
	movlb 2	; select bank2
	movf	(_ui16_I_cal_Ph3)^0100h,w
	movlb 0	; select bank0
	movwf	0+(?___wmul)+02h
	fcall	___wmul
	movf	(1+(?___wmul)),w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB2_RD)^080h
	line	1261
	
l30740:	
;diag.c: 1261: ui8_b_DResB3_RD = ( unsigned char )( ui16_I_cal_Ph3>>8 );
	movlb 2	; select bank2
	movf	(_ui16_I_cal_Ph3+1)^0100h,w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB3_RD)^080h
	line	1262
	
l30742:	
;diag.c: 1262: ui8_b_DResB4_RD = ( unsigned char )( ui16_I_cal_Ph3 );
	movlb 2	; select bank2
	movf	(_ui16_I_cal_Ph3)^0100h,w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB4_RD)^080h
	line	1263
	
l30744:	
	line	1270
	
l30746:	
;diag.c: 1267: Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD,
;diag.c: 1268: ui8_b_DResB1_RD, ui8_b_DResB2_RD,
;diag.c: 1269: ui8_b_DResB3_RD, ui8_b_DResB4_RD,
;diag.c: 1270: ui8_b_DResB5_RD );
	movlw	(03Ah)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB0_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	clrf	0+(?_Transmit_LIN_8Bytes)+03h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB2_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB4_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+06h
	clrf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	1271
	
l30748:	
;diag.c: 1271: ui8_selected_lid = 0x98;
	movlw	(098h)
	movlb 0	; select bank0
	movwf	(_ui8_selected_lid)
	line	1274
;diag.c: 1274: break;
	goto	l7822
	line	1283
	
l30750:	
;diag.c: 1281: {
;diag.c: 1283: NegativeAnswer(ui8_b_DResLocID_c);
	movlb 0	; select bank0
	movf	(_ui8_b_DResLocID_c),w
	fcall	_NegativeAnswer
	line	1287
;diag.c: 1285: }
;diag.c: 1287: }
	goto	l7822
	line	1031
	
l30752:	
	movlb 1	; select bank1
	movf	(_phaseCal)^080h,w
	; Switch size 1, requested type "space"
; Number of cases is 4, Range of values is 0 to 3
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           13     7 (average)
; direct_byte           14     6 (fixed)
; jumptable            260     6 (fixed)
; rangetable             8     4 (fixed)
; spacedrange           13     6 (fixed)
; locatedrange           4     3 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	0^0	; case 0
	skipnz
	goto	l30592
	xorlw	1^0	; case 1
	skipnz
	goto	l30690
	xorlw	2^1	; case 2
	skipnz
	goto	l30710
	xorlw	3^2	; case 3
	skipnz
	goto	l30730
	goto	l30750
	opt asmopt_on

	line	1296
	
l30754:	
;diag.c: 1293: else
;diag.c: 1294: {
;diag.c: 1296: NegativeAnswer(ui8_b_DResLocID_c);
	movf	(_ui8_b_DResLocID_c),w
	fcall	_NegativeAnswer
	goto	l7822
	line	1313
	
l30756:	
;diag.c: 1311: {
;diag.c: 1313: if( 0x98 == ui8_selected_lid )
	movf	(_ui8_selected_lid),w
	xorlw	098h&0ffh
	skipz
	goto	u11121
	goto	u11120
u11121:
	goto	l7650
u11120:
	line	1328
	
l30758:	
;diag.c: 1315: {
;diag.c: 1328: wkpoint = ui8_b_DResB0_c;
	movf	(_ui8_b_DResB0_c),w
	movlb 1	; select bank1
	movwf	(_wkpoint)^080h
	line	1334
;diag.c: 1334: switch( ui8_b_DResB0_c )
	goto	l30852
	line	1344
	
l30760:	
;diag.c: 1342: {
;diag.c: 1344: if( wkpoint == ui8_b_DResB0_c )
	movlb 1	; select bank1
	movf	(_wkpoint)^080h,w
	movlb 0	; select bank0
	xorwf	(_ui8_b_DResB0_c),w
	skipz
	goto	u11131
	goto	u11130
u11131:
	goto	l7650
u11130:
	line	1348
	
l30762:	
;diag.c: 1346: {
;diag.c: 1348: ui8_b_DResB0_RD = ui8_b_DResB0_c;
	movf	(_ui8_b_DResB0_c),w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB0_RD)^080h
	line	1349
	
l30764:	
;diag.c: 1349: ui8_b_DResB1_RD = PWMReadDC( )>>1;
	fcall	_PWMReadDC
	lsrf	wreg,f
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB1_RD)^080h
	line	1350
	
l30766:	
;diag.c: 1350: ui8_b_DResB2_RD = ui16_speed_fil*5/4;
	movlb 2	; select bank2
	movf	(_ui16_speed_fil+1)^0100h,w
	movlb 0	; select bank0
	movwf	(?___wmul+1)
	movlb 2	; select bank2
	movf	(_ui16_speed_fil)^0100h,w
	movlb 0	; select bank0
	movwf	(?___wmul)
	movlw	05h
	movwf	0+(?___wmul)+02h
	clrf	1+(?___wmul)+02h
	fcall	___wmul
	movf	(0+?___wmul),w
	movwf	(??_EOL+0)+0
	movf	(1+?___wmul),w
	movwf	((??_EOL+0)+0+1)
	lsrf	(??_EOL+0)+1,f
	rrf	(??_EOL+0)+0,f
	lsrf	(??_EOL+0)+1,f
	rrf	(??_EOL+0)+0,f
	movf	0+(??_EOL+0)+0,w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB2_RD)^080h
	line	1351
	
l30768:	
;diag.c: 1351: ui8_b_DResB3_RD = PWM_trans_table[ ui8_b_DResB1_RD ];
	movf	(EOL@ui8_b_DResB1_RD)^080h,w
	addlw	low(_PWM_trans_table|8000h)
	movlp	high __stringtab
	callw
	pagesel	$
	movwf	(EOL@ui8_b_DResB3_RD)^080h
	line	1352
	
l30770:	
	line	1353
	
l30772:	
	line	1356
	
l30774:	
;diag.c: 1356: if ((ui8_b_DResB2_RD < ui8_b_DResB3_RD + 3) && (ui8_b_DResB2_RD > ui8_b_DResB3_RD - 3))
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	addlw	low(03h)
	movlb 0	; select bank0
	movwf	(??_EOL+0)+0
	movlw	high(03h)
	skipnc
	movlw	(high(03h)+1)&0ffh
	movwf	((??_EOL+0)+0)+1
	movf	1+(??_EOL+0)+0,w
	xorlw	80h
	sublw	080h
	skipz
	goto	u11145
	movf	0+(??_EOL+0)+0,w
	movlb 1	; select bank1
	subwf	(EOL@ui8_b_DResB2_RD)^080h,w
u11145:

	skipnc
	goto	u11141
	goto	u11140
u11141:
	goto	l30780
u11140:
	
l30776:	
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	addlw	low(-3)
	movlb 0	; select bank0
	movwf	(??_EOL+0)+0
	movlw	high(-3)
	skipnc
	movlw	(high(-3)+1)&0ffh
	movwf	((??_EOL+0)+0)+1
	movf	1+(??_EOL+0)+0,w
	xorlw	80h
	movwf	(??_EOL+2)+0
	movlw	80h
	subwf	(??_EOL+2)+0,w
	skipz
	goto	u11155
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB2_RD)^080h,w
	movlb 0	; select bank0
	subwf	0+(??_EOL+0)+0,w
u11155:

	skipnc
	goto	u11151
	goto	u11150
u11151:
	goto	l30780
u11150:
	line	1360
	
l30778:	
;diag.c: 1358: {
;diag.c: 1360: Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD, ui8_b_DResB1_RD, ui8_b_DResB2_RD, ui8_b_DResB3_RD, ui8_b_DResB4_RD, ui8_b_DResB5_RD );
	movlw	(03Ah)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB0_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB1_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+03h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB2_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	clrf	0+(?_Transmit_LIN_8Bytes)+06h
	clrf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	1363
;diag.c: 1363: }
	goto	l7822
	line	1369
	
l30780:	
;diag.c: 1366: else
;diag.c: 1367: {
;diag.c: 1369: Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, ui8_b_DResB0_c, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );
	movlw	(07Fh)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movf	(_ui8_b_DResB0_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+03h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+06h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	goto	l7822
	line	1521
	
l30782:	
;diag.c: 1519: {
;diag.c: 1521: if( wkpoint == ui8_b_DResB0_c )
	movlb 1	; select bank1
	movf	(_wkpoint)^080h,w
	movlb 0	; select bank0
	xorwf	(_ui8_b_DResB0_c),w
	skipz
	goto	u11161
	goto	u11160
u11161:
	goto	l7650
u11160:
	line	1525
	
l30784:	
;diag.c: 1523: {
;diag.c: 1525: ui8_b_DResB0_RD = ui8_b_DResB0_c;
	movf	(_ui8_b_DResB0_c),w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB0_RD)^080h
	line	1526
	
l30786:	
;diag.c: 1526: ui8_b_DResB1_RD = PWMReadDC( )>>1;
	fcall	_PWMReadDC
	lsrf	wreg,f
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB1_RD)^080h
	line	1527
	
l30788:	
;diag.c: 1527: ui8_b_DResB2_RD = ui16_speed_fil*5/4;
	movlb 2	; select bank2
	movf	(_ui16_speed_fil+1)^0100h,w
	movlb 0	; select bank0
	movwf	(?___wmul+1)
	movlb 2	; select bank2
	movf	(_ui16_speed_fil)^0100h,w
	movlb 0	; select bank0
	movwf	(?___wmul)
	movlw	05h
	movwf	0+(?___wmul)+02h
	clrf	1+(?___wmul)+02h
	fcall	___wmul
	movf	(0+?___wmul),w
	movwf	(??_EOL+0)+0
	movf	(1+?___wmul),w
	movwf	((??_EOL+0)+0+1)
	lsrf	(??_EOL+0)+1,f
	rrf	(??_EOL+0)+0,f
	lsrf	(??_EOL+0)+1,f
	rrf	(??_EOL+0)+0,f
	movf	0+(??_EOL+0)+0,w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB2_RD)^080h
	line	1528
	
l30790:	
;diag.c: 1528: ui8_b_DResB3_RD = PWM_trans_table[ ui8_b_DResB1_RD ];
	movf	(EOL@ui8_b_DResB1_RD)^080h,w
	addlw	low(_PWM_trans_table|8000h)
	movlp	high __stringtab
	callw
	pagesel	$
	movwf	(EOL@ui8_b_DResB3_RD)^080h
	line	1529
	
l30792:	
	line	1530
	
l30794:	
	line	1534
	
l30796:	
;diag.c: 1534: if ((ui8_b_DResB2_RD < ui8_b_DResB3_RD + 3) && (ui8_b_DResB2_RD > ui8_b_DResB3_RD - 3))
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	addlw	low(03h)
	movlb 0	; select bank0
	movwf	(??_EOL+0)+0
	movlw	high(03h)
	skipnc
	movlw	(high(03h)+1)&0ffh
	movwf	((??_EOL+0)+0)+1
	movf	1+(??_EOL+0)+0,w
	xorlw	80h
	sublw	080h
	skipz
	goto	u11175
	movf	0+(??_EOL+0)+0,w
	movlb 1	; select bank1
	subwf	(EOL@ui8_b_DResB2_RD)^080h,w
u11175:

	skipnc
	goto	u11171
	goto	u11170
u11171:
	goto	l30802
u11170:
	
l30798:	
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	addlw	low(-3)
	movlb 0	; select bank0
	movwf	(??_EOL+0)+0
	movlw	high(-3)
	skipnc
	movlw	(high(-3)+1)&0ffh
	movwf	((??_EOL+0)+0)+1
	movf	1+(??_EOL+0)+0,w
	xorlw	80h
	movwf	(??_EOL+2)+0
	movlw	80h
	subwf	(??_EOL+2)+0,w
	skipz
	goto	u11185
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB2_RD)^080h,w
	movlb 0	; select bank0
	subwf	0+(??_EOL+0)+0,w
u11185:

	skipnc
	goto	u11181
	goto	u11180
u11181:
	goto	l30802
u11180:
	line	1542
	
l30800:	
;diag.c: 1536: {
;diag.c: 1538: Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c,
;diag.c: 1539: ui8_b_DResB0_RD, ui8_b_DResB1_RD,
;diag.c: 1540: ui8_b_DResB2_RD, ui8_b_DResB3_RD,
;diag.c: 1541: ui8_b_DResB4_RD,
;diag.c: 1542: ui8_b_DResB5_RD );
	movlw	(03Ah)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB0_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB1_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+03h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB2_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	clrf	0+(?_Transmit_LIN_8Bytes)+06h
	clrf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	1545
;diag.c: 1545: }
	goto	l7822
	line	1551
	
l30802:	
;diag.c: 1548: else
;diag.c: 1549: {
;diag.c: 1551: Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, ui8_b_DResB0_c, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );
	movlw	(07Fh)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movf	(_ui8_b_DResB0_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+03h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+06h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	goto	l7822
	line	1609
	
l30804:	
;diag.c: 1607: {
;diag.c: 1609: if( wkpoint == ui8_b_DResB0_c )
	movlb 1	; select bank1
	movf	(_wkpoint)^080h,w
	movlb 0	; select bank0
	xorwf	(_ui8_b_DResB0_c),w
	skipz
	goto	u11191
	goto	u11190
u11191:
	goto	l7650
u11190:
	line	1613
	
l30806:	
;diag.c: 1611: {
;diag.c: 1613: ui8_b_DResB0_RD = ui8_b_DResB0_c;
	movf	(_ui8_b_DResB0_c),w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB0_RD)^080h
	line	1614
	
l30808:	
;diag.c: 1614: ui8_b_DResB1_RD = PWMReadDC( )>>1;
	fcall	_PWMReadDC
	lsrf	wreg,f
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB1_RD)^080h
	line	1615
	
l30810:	
;diag.c: 1615: ui8_b_DResB2_RD = ui16_speed_fil*5/4;
	movlb 2	; select bank2
	movf	(_ui16_speed_fil+1)^0100h,w
	movlb 0	; select bank0
	movwf	(?___wmul+1)
	movlb 2	; select bank2
	movf	(_ui16_speed_fil)^0100h,w
	movlb 0	; select bank0
	movwf	(?___wmul)
	movlw	05h
	movwf	0+(?___wmul)+02h
	clrf	1+(?___wmul)+02h
	fcall	___wmul
	movf	(0+?___wmul),w
	movwf	(??_EOL+0)+0
	movf	(1+?___wmul),w
	movwf	((??_EOL+0)+0+1)
	lsrf	(??_EOL+0)+1,f
	rrf	(??_EOL+0)+0,f
	lsrf	(??_EOL+0)+1,f
	rrf	(??_EOL+0)+0,f
	movf	0+(??_EOL+0)+0,w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB2_RD)^080h
	line	1616
	
l30812:	
;diag.c: 1616: ui8_b_DResB3_RD = PWM_trans_table[ ui8_b_DResB1_RD ];
	movf	(EOL@ui8_b_DResB1_RD)^080h,w
	addlw	low(_PWM_trans_table|8000h)
	movlp	high __stringtab
	callw
	pagesel	$
	movwf	(EOL@ui8_b_DResB3_RD)^080h
	line	1617
	
l30814:	
	line	1618
	
l30816:	
	line	1622
	
l30818:	
;diag.c: 1622: if ((ui8_b_DResB2_RD < ui8_b_DResB3_RD + 3) && (ui8_b_DResB2_RD > ui8_b_DResB3_RD - 3))
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	addlw	low(03h)
	movlb 0	; select bank0
	movwf	(??_EOL+0)+0
	movlw	high(03h)
	skipnc
	movlw	(high(03h)+1)&0ffh
	movwf	((??_EOL+0)+0)+1
	movf	1+(??_EOL+0)+0,w
	xorlw	80h
	sublw	080h
	skipz
	goto	u11205
	movf	0+(??_EOL+0)+0,w
	movlb 1	; select bank1
	subwf	(EOL@ui8_b_DResB2_RD)^080h,w
u11205:

	skipnc
	goto	u11201
	goto	u11200
u11201:
	goto	l30824
u11200:
	
l30820:	
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	addlw	low(-3)
	movlb 0	; select bank0
	movwf	(??_EOL+0)+0
	movlw	high(-3)
	skipnc
	movlw	(high(-3)+1)&0ffh
	movwf	((??_EOL+0)+0)+1
	movf	1+(??_EOL+0)+0,w
	xorlw	80h
	movwf	(??_EOL+2)+0
	movlw	80h
	subwf	(??_EOL+2)+0,w
	skipz
	goto	u11215
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB2_RD)^080h,w
	movlb 0	; select bank0
	subwf	0+(??_EOL+0)+0,w
u11215:

	skipnc
	goto	u11211
	goto	u11210
u11211:
	goto	l30824
u11210:
	line	1630
	
l30822:	
;diag.c: 1624: {
;diag.c: 1626: Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c,
;diag.c: 1627: ui8_b_DResB0_RD, ui8_b_DResB1_RD,
;diag.c: 1628: ui8_b_DResB2_RD, ui8_b_DResB3_RD,
;diag.c: 1629: ui8_b_DResB4_RD,
;diag.c: 1630: ui8_b_DResB5_RD );
	movlw	(03Ah)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB0_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB1_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+03h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB2_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	clrf	0+(?_Transmit_LIN_8Bytes)+06h
	clrf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	1633
;diag.c: 1633: }
	goto	l7822
	line	1639
	
l30824:	
;diag.c: 1636: else
;diag.c: 1637: {
;diag.c: 1639: Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, ui8_b_DResB0_c, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );
	movlw	(07Fh)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movf	(_ui8_b_DResB0_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+03h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+06h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	goto	l7822
	line	1695
	
l30826:	
;diag.c: 1693: {
;diag.c: 1695: if( wkpoint == ui8_b_DResB0_c )
	movlb 1	; select bank1
	movf	(_wkpoint)^080h,w
	movlb 0	; select bank0
	xorwf	(_ui8_b_DResB0_c),w
	skipz
	goto	u11221
	goto	u11220
u11221:
	goto	l7650
u11220:
	line	1699
	
l30828:	
;diag.c: 1697: {
;diag.c: 1699: ui8_b_DResB0_RD = ui8_b_DResB0_c;
	movf	(_ui8_b_DResB0_c),w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB0_RD)^080h
	line	1700
	
l30830:	
;diag.c: 1700: ui8_b_DResB1_RD = PWMReadDC( )>>1;
	fcall	_PWMReadDC
	lsrf	wreg,f
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB1_RD)^080h
	line	1701
	
l30832:	
;diag.c: 1701: ui8_b_DResB2_RD = ui16_speed_fil*5/4;
	movlb 2	; select bank2
	movf	(_ui16_speed_fil+1)^0100h,w
	movlb 0	; select bank0
	movwf	(?___wmul+1)
	movlb 2	; select bank2
	movf	(_ui16_speed_fil)^0100h,w
	movlb 0	; select bank0
	movwf	(?___wmul)
	movlw	05h
	movwf	0+(?___wmul)+02h
	clrf	1+(?___wmul)+02h
	fcall	___wmul
	movf	(0+?___wmul),w
	movwf	(??_EOL+0)+0
	movf	(1+?___wmul),w
	movwf	((??_EOL+0)+0+1)
	lsrf	(??_EOL+0)+1,f
	rrf	(??_EOL+0)+0,f
	lsrf	(??_EOL+0)+1,f
	rrf	(??_EOL+0)+0,f
	movf	0+(??_EOL+0)+0,w
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB2_RD)^080h
	line	1702
	
l30834:	
;diag.c: 1702: ui8_b_DResB3_RD = PWM_trans_table[ ui8_b_DResB1_RD ];
	movf	(EOL@ui8_b_DResB1_RD)^080h,w
	addlw	low(_PWM_trans_table|8000h)
	movlp	high __stringtab
	callw
	pagesel	$
	movwf	(EOL@ui8_b_DResB3_RD)^080h
	line	1703
	
l30836:	
	line	1704
	
l30838:	
	line	1707
	
l30840:	
;diag.c: 1707: if ((ui8_b_DResB2_RD < ui8_b_DResB3_RD + 3) && (ui8_b_DResB2_RD > ui8_b_DResB3_RD - 3))
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	addlw	low(03h)
	movlb 0	; select bank0
	movwf	(??_EOL+0)+0
	movlw	high(03h)
	skipnc
	movlw	(high(03h)+1)&0ffh
	movwf	((??_EOL+0)+0)+1
	movf	1+(??_EOL+0)+0,w
	xorlw	80h
	sublw	080h
	skipz
	goto	u11235
	movf	0+(??_EOL+0)+0,w
	movlb 1	; select bank1
	subwf	(EOL@ui8_b_DResB2_RD)^080h,w
u11235:

	skipnc
	goto	u11231
	goto	u11230
u11231:
	goto	l30846
u11230:
	
l30842:	
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	addlw	low(-3)
	movlb 0	; select bank0
	movwf	(??_EOL+0)+0
	movlw	high(-3)
	skipnc
	movlw	(high(-3)+1)&0ffh
	movwf	((??_EOL+0)+0)+1
	movf	1+(??_EOL+0)+0,w
	xorlw	80h
	movwf	(??_EOL+2)+0
	movlw	80h
	subwf	(??_EOL+2)+0,w
	skipz
	goto	u11245
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB2_RD)^080h,w
	movlb 0	; select bank0
	subwf	0+(??_EOL+0)+0,w
u11245:

	skipnc
	goto	u11241
	goto	u11240
u11241:
	goto	l30846
u11240:
	line	1715
	
l30844:	
;diag.c: 1709: {
;diag.c: 1711: Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c,
;diag.c: 1712: ui8_b_DResB0_RD, ui8_b_DResB1_RD,
;diag.c: 1713: ui8_b_DResB2_RD, ui8_b_DResB3_RD,
;diag.c: 1714: ui8_b_DResB4_RD,
;diag.c: 1715: ui8_b_DResB5_RD );
	movlw	(03Ah)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB0_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB1_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+03h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB2_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	clrf	0+(?_Transmit_LIN_8Bytes)+06h
	clrf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	1718
;diag.c: 1718: }
	goto	l7822
	line	1724
	
l30846:	
;diag.c: 1721: else
;diag.c: 1722: {
;diag.c: 1724: Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, ui8_b_DResB0_c, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );
	movlw	(07Fh)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movf	(_ui8_b_DResB0_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+03h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+06h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	goto	l7822
	line	1952
	
l30848:	
;diag.c: 1948: {
;diag.c: 1952: NegativeAnswer(ui8_b_DResLocID_c);
	movf	(_ui8_b_DResLocID_c),w
	fcall	_NegativeAnswer
	line	1953
;diag.c: 1953: break;
	goto	l7822
	line	1334
	
l30852:	
	movlb 0	; select bank0
	movf	(_ui8_b_DResB0_c),w
	; Switch size 1, requested type "space"
; Number of cases is 4, Range of values is 1 to 5
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           13     7 (average)
; direct_byte           19     9 (fixed)
; jumptable            263     9 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	1^0	; case 1
	skipnz
	goto	l30760
	xorlw	3^1	; case 3
	skipnz
	goto	l30782
	xorlw	4^3	; case 4
	skipnz
	goto	l30804
	xorlw	5^4	; case 5
	skipnz
	goto	l30826
	goto	l30848
	opt asmopt_on

	line	1994
	
l30854:	
;diag.c: 1992: {
;diag.c: 1994: if( 0x9f == ui8_selected_lid )
	movf	(_ui8_selected_lid),w
	xorlw	09Fh&0ffh
	skipz
	goto	u11251
	goto	u11250
u11251:
	goto	l7650
u11250:
	line	2000
	
l30856:	
;diag.c: 1996: {
;diag.c: 2000: ui8_failure = ui8_b_DResB0_c;
	movf	(_ui8_b_DResB0_c),w
	movlb 2	; select bank2
	movwf	(_ui8_failure)^0100h
	line	2005
;diag.c: 2005: switch (ui8_failure)
	goto	l30926
	line	2102
	
l30858:	
;diag.c: 2098: {
;diag.c: 2102: PWM_Write_Out( 55 );
	movlw	(037h)
	fcall	_PWM_Write_Out
	line	2103
	
l30860:	
;diag.c: 2103: ui8_error_Flags.bits.B0 = 0;
	movlb 1	; select bank1
	bcf	(_ui8_error_Flags)^080h,0
	line	2104
	
l30862:	
;diag.c: 2104: ui8_error_Flags.bits.B1 = 0;
	bcf	(_ui8_error_Flags)^080h,1
	line	2105
	
l30864:	
;diag.c: 2105: ui8_error_Flags.bits.B7 = 0;
	bcf	(_ui8_error_Flags)^080h,7
	line	2106
	
l30866:	
;diag.c: 2106: ui8_error_Flags.bits.B2 = 0;
	bcf	(_ui8_error_Flags)^080h,2
	line	2107
	
l30868:	
;diag.c: 2107: ui8_error_Flags.bits.B3 = 1;
	bsf	(_ui8_error_Flags)^080h,3
	line	2108
	
l30870:	
;diag.c: 2108: ui8_error_Flags.bits.B4 = 0;
	bcf	(_ui8_error_Flags)^080h,4
	line	2109
	
l30872:	
;diag.c: 2109: ui8_error_Flags.bits.B6 = 0;
	bcf	(_ui8_error_Flags)^080h,6
	line	2110
	
l30874:	
;diag.c: 2110: ui8_error_Flags.bits.B5 = 0;
	bcf	(_ui8_error_Flags)^080h,5
	line	2111
	
l30876:	
;diag.c: 2111: ui8_b_DResB1_RD = 55;
	movlw	(037h)
	movwf	(EOL@ui8_b_DResB1_RD)^080h
	line	2112
	
l30878:	
;diag.c: 2112: ui8_b_DResB0_RD = 8;
	movlw	(08h)
	movwf	(EOL@ui8_b_DResB0_RD)^080h
	line	2113
	
l30880:	
;diag.c: 2113: ui8_failure = 3;
	movlw	(03h)
	movlb 2	; select bank2
	movwf	(_ui8_failure)^0100h
	line	2114
;diag.c: 2114: break;
	goto	l30928
	line	2155
	
l30882:	
;diag.c: 2151: {
;diag.c: 2155: PWM_Write_Out( 77 );
	movlw	(04Dh)
	fcall	_PWM_Write_Out
	line	2156
	
l30884:	
;diag.c: 2156: ui8_error_Flags.bits.B0 = 0;
	movlb 1	; select bank1
	bcf	(_ui8_error_Flags)^080h,0
	line	2157
	
l30886:	
;diag.c: 2157: ui8_error_Flags.bits.B1 = 0;
	bcf	(_ui8_error_Flags)^080h,1
	line	2158
	
l30888:	
;diag.c: 2158: ui8_error_Flags.bits.B7 = 0;
	bcf	(_ui8_error_Flags)^080h,7
	line	2159
	
l30890:	
;diag.c: 2159: ui8_error_Flags.bits.B2 = 0;
	bcf	(_ui8_error_Flags)^080h,2
	line	2160
	
l30892:	
;diag.c: 2160: ui8_error_Flags.bits.B3 = 0;
	bcf	(_ui8_error_Flags)^080h,3
	line	2161
	
l30894:	
;diag.c: 2161: ui8_error_Flags.bits.B4 = 0;
	bcf	(_ui8_error_Flags)^080h,4
	line	2162
	
l30896:	
;diag.c: 2162: ui8_error_Flags.bits.B6 = 0;
	bcf	(_ui8_error_Flags)^080h,6
	line	2163
	
l30898:	
;diag.c: 2163: ui8_error_Flags.bits.B5 = 1;
	bsf	(_ui8_error_Flags)^080h,5
	line	2165
	
l30900:	
;diag.c: 2165: ui8_b_DResB1_RD = 77;
	movlw	(04Dh)
	movwf	(EOL@ui8_b_DResB1_RD)^080h
	line	2166
	
l30902:	
;diag.c: 2166: ui8_b_DResB0_RD= 1;
	clrf	(EOL@ui8_b_DResB0_RD)^080h
	incf	(EOL@ui8_b_DResB0_RD)^080h,f
	line	2167
	
l30904:	
;diag.c: 2167: ui8_failure = 1;
	movlb 2	; select bank2
	clrf	(_ui8_failure)^0100h
	incf	(_ui8_failure)^0100h,f
	line	2168
;diag.c: 2168: break;
	goto	l30928
	line	2209
	
l30906:	
;diag.c: 2205: {
;diag.c: 2209: PWM_Write_Out( 11 );
	movlw	(0Bh)
	fcall	_PWM_Write_Out
	line	2210
	
l30908:	
;diag.c: 2210: ui8_error_Flags.bits.B0 = 0;
	movlb 1	; select bank1
	bcf	(_ui8_error_Flags)^080h,0
	line	2211
	
l30910:	
;diag.c: 2211: ui8_error_Flags.bits.B1 = 0;
	bcf	(_ui8_error_Flags)^080h,1
	line	2212
	
l30912:	
;diag.c: 2212: ui8_error_Flags.bits.B7 = 0;
	bcf	(_ui8_error_Flags)^080h,7
	line	2213
	
l30914:	
;diag.c: 2213: ui8_error_Flags.bits.B2 = 0;
	bcf	(_ui8_error_Flags)^080h,2
	line	2214
	
l30916:	
;diag.c: 2214: ui8_error_Flags.bits.B3 = 0;
	bcf	(_ui8_error_Flags)^080h,3
	line	2215
	
l30918:	
;diag.c: 2215: ui8_error_Flags.bits.B4 = 0;
	bcf	(_ui8_error_Flags)^080h,4
	line	2216
	
l30920:	
;diag.c: 2216: ui8_error_Flags.bits.B6 = 0;
	bcf	(_ui8_error_Flags)^080h,6
	line	2217
	
l30922:	
;diag.c: 2217: ui8_error_Flags.bits.B5 = 0;
	bcf	(_ui8_error_Flags)^080h,5
	line	2218
;diag.c: 2218: break;
	goto	l30928
	line	2005
	
l30926:	
	movf	(_ui8_failure)^0100h,w
	; Switch size 1, requested type "space"
; Number of cases is 2, Range of values is 2 to 4
; switch strategies available:
; Name         Instructions Cycles
; simple_byte            7     4 (average)
; direct_byte           15     9 (fixed)
; jumptable            263     9 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	2^0	; case 2
	skipnz
	goto	l30882
	xorlw	4^2	; case 4
	skipnz
	goto	l30858
	goto	l30906
	opt asmopt_on

	line	2231
	
l30928:	
	line	2232
;diag.c: 2232: ui8_b_DResB3_RD = 0;
	movlb 1	; select bank1
	clrf	(EOL@ui8_b_DResB3_RD)^080h
	line	2233
;diag.c: 2233: ui8_b_DResB4_RD = 0;
	clrf	(EOL@ui8_b_DResB4_RD)^080h
	line	2234
;diag.c: 2234: ui8_b_DResB5_RD = 0;
	clrf	(EOL@ui8_b_DResB5_RD)^080h
	line	2240
	
l30930:	
;diag.c: 2237: Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD,
;diag.c: 2238: ui8_b_DResB1_RD, ui8_b_DResB2_RD,
;diag.c: 2239: ui8_b_DResB3_RD, ui8_b_DResB4_RD,
;diag.c: 2240: ui8_b_DResB5_RD );
	movlw	(03Ah)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB0_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB1_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+03h
	clrf	0+(?_Transmit_LIN_8Bytes)+04h
	clrf	0+(?_Transmit_LIN_8Bytes)+05h
	clrf	0+(?_Transmit_LIN_8Bytes)+06h
	clrf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	2243
;diag.c: 2243: }
	goto	l7822
	line	2273
	
l30932:	
;diag.c: 2271: {
;diag.c: 2273: if( 0xa0 == ui8_selected_lid )
	movf	(_ui8_selected_lid),w
	xorlw	0A0h&0ffh
	skipz
	goto	u11261
	goto	u11260
u11261:
	goto	l7650
u11260:
	line	2285
	
l30934:	
;diag.c: 2275: {
;diag.c: 2285: write_eeprom_data( 0xf4, ui8_b_DResB0_c );
	movf	(_ui8_b_DResB0_c),w
	movwf	(?_write_eeprom_data)
	movlw	(0F4h)
	fcall	_write_eeprom_data
	line	2286
;diag.c: 2286: write_eeprom_data( 0xf5, ui8_b_DResB1_c );
	movlb 2	; select bank2
	movf	(_ui8_b_DResB1_c)^0100h,w
	movlb 0	; select bank0
	movwf	(?_write_eeprom_data)
	movlw	(0F5h)
	fcall	_write_eeprom_data
	line	2287
;diag.c: 2287: write_eeprom_data( 0xf6, ui8_b_DResB2_c );
	movlb 2	; select bank2
	movf	(_ui8_b_DResB2_c)^0100h,w
	movlb 0	; select bank0
	movwf	(?_write_eeprom_data)
	movlw	(0F6h)
	fcall	_write_eeprom_data
	line	2288
;diag.c: 2288: write_eeprom_data( 0xf7, ui8_b_DResB3_c );
	movlb 2	; select bank2
	movf	(_ui8_b_DResB3_c)^0100h,w
	movlb 0	; select bank0
	movwf	(?_write_eeprom_data)
	movlw	(0F7h)
	fcall	_write_eeprom_data
	line	2289
;diag.c: 2289: write_eeprom_data( 0xf8, ui8_b_DResB4_c );
	movlb 2	; select bank2
	movf	(_ui8_b_DResB4_c)^0100h,w
	movlb 0	; select bank0
	movwf	(?_write_eeprom_data)
	movlw	(0F8h)
	fcall	_write_eeprom_data
	line	2301
	
l30936:	
;diag.c: 2301: ui8_b_DResB0_RD = read_eeprom_data( 0xf4 );
	movlw	(0F4h)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB0_RD)^080h
	line	2302
	
l30938:	
;diag.c: 2302: ui8_b_DResB1_RD = read_eeprom_data( 0xf5 );
	movlw	(0F5h)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB1_RD)^080h
	line	2303
	
l30940:	
;diag.c: 2303: ui8_b_DResB2_RD = read_eeprom_data( 0xf6 );
	movlw	(0F6h)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB2_RD)^080h
	line	2304
	
l30942:	
;diag.c: 2304: ui8_b_DResB3_RD = read_eeprom_data( 0xf7 );
	movlw	(0F7h)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB3_RD)^080h
	line	2305
	
l30944:	
;diag.c: 2305: ui8_b_DResB4_RD = read_eeprom_data( 0xf8 );
	movlw	(0F8h)
	fcall	_read_eeprom_data
	movlb 1	; select bank1
	movwf	(EOL@ui8_b_DResB4_RD)^080h
	line	2306
	
l30946:	
	line	2309
	
l30948:	
;diag.c: 2309: if ((ui8_b_DResB0_RD != 0xFF) && (ui8_b_DResB1_RD != 0xFF) && (ui8_b_DResB2_RD != 0xFF) && (ui8_b_DResB3_RD != 0xFF) && (ui8_b_DResB4_RD != 0xFF))
	movf	(EOL@ui8_b_DResB0_RD)^080h,w
	xorlw	0FFh&0ffh
	skipnz
	goto	u11271
	goto	u11270
u11271:
	goto	l30960
u11270:
	
l30950:	
	movf	(EOL@ui8_b_DResB1_RD)^080h,w
	xorlw	0FFh&0ffh
	skipnz
	goto	u11281
	goto	u11280
u11281:
	goto	l30960
u11280:
	
l30952:	
	movf	(EOL@ui8_b_DResB2_RD)^080h,w
	xorlw	0FFh&0ffh
	skipnz
	goto	u11291
	goto	u11290
u11291:
	goto	l30960
u11290:
	
l30954:	
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	xorlw	0FFh&0ffh
	skipnz
	goto	u11301
	goto	u11300
u11301:
	goto	l30960
u11300:
	
l30956:	
	movf	(EOL@ui8_b_DResB4_RD)^080h,w
	xorlw	0FFh&0ffh
	skipnz
	goto	u11311
	goto	u11310
u11311:
	goto	l30960
u11310:
	line	2313
	
l30958:	
;diag.c: 2311: {
;diag.c: 2313: Transmit_LIN_8Bytes( 0x10, 0x3A, ui8_b_DResLocID_c, ui8_b_DResB0_RD, ui8_b_DResB1_RD, ui8_b_DResB2_RD, ui8_b_DResB3_RD, ui8_b_DResB4_RD, ui8_b_DResB5_RD );
	movlw	(03Ah)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB0_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB1_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+03h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB2_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB3_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	movlb 1	; select bank1
	movf	(EOL@ui8_b_DResB4_RD)^080h,w
	movlb 0	; select bank0
	movwf	0+(?_Transmit_LIN_8Bytes)+06h
	clrf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	2316
;diag.c: 2316: }
	goto	l30962
	line	2322
	
l30960:	
;diag.c: 2319: else
;diag.c: 2320: {
;diag.c: 2322: Transmit_LIN_8Bytes( 0x10, 0x7F, ui8_b_DResLocID_c, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF );
	movlw	(07Fh)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+03h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+06h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	2336
	
l30962:	
;diag.c: 2332: }
;diag.c: 2336: ui8_selected_lid = 0xfa;
	movlw	(0FAh)
	movlb 0	; select bank0
	movwf	(_ui8_selected_lid)
	line	2339
;diag.c: 2339: }
	goto	l7822
	line	2367
	
l30964:	
;diag.c: 2365: {
;diag.c: 2367: if( 0xfa == ui8_selected_lid )
	movf	(_ui8_selected_lid),w
	xorlw	0FAh&0ffh
	skipz
	goto	u11321
	goto	u11320
u11321:
	goto	l7650
u11320:
	line	2371
	
l30966:	
;diag.c: 2369: {
;diag.c: 2371: Transmit_LIN_8Bytes( 0x10, 0x3a, ui8_b_DResLocID_c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 );
	movlw	(03Ah)
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(_ui8_b_DResLocID_c),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	clrf	0+(?_Transmit_LIN_8Bytes)+02h
	clrf	0+(?_Transmit_LIN_8Bytes)+03h
	clrf	0+(?_Transmit_LIN_8Bytes)+04h
	clrf	0+(?_Transmit_LIN_8Bytes)+05h
	clrf	0+(?_Transmit_LIN_8Bytes)+06h
	clrf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	2372
	
l30968:	
;diag.c: 2372: write_eeprom_data(0x55,0x55);
	movlw	(055h)
	movlb 0	; select bank0
	movwf	(?_write_eeprom_data)
	movlw	(055h)
	fcall	_write_eeprom_data
	line	2373
	
l30970:	
;diag.c: 2373: RC5 = 0;
	movlb 0	; select bank0
	bcf	(117/8),(117)&7
	line	2375
;diag.c: 2375: }
	goto	l7822
	line	2405
	
l30972:	
;diag.c: 2399: {
;diag.c: 2405: NegativeAnswer(ui8_selected_lid);
	movf	(_ui8_selected_lid),w
	fcall	_NegativeAnswer
	line	2406
;diag.c: 2406: break;
	goto	l7822
	line	482
	
l30976:	
	movf	(_ui8_b_DResLocID_c),w
	; Switch size 1, requested type "space"
; Number of cases is 10, Range of values is 128 to 250
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           31    16 (average)
; direct_byte          255     9 (fixed)
; jumptable            263     9 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	128^0	; case 128
	skipnz
	goto	l30430
	xorlw	144^128	; case 144
	skipnz
	goto	l30438
	xorlw	145^144	; case 145
	skipnz
	goto	l30478
	xorlw	147^145	; case 147
	skipnz
	goto	l30500
	xorlw	148^147	; case 148
	skipnz
	goto	l30558
	xorlw	149^148	; case 149
	skipnz
	goto	l30588
	xorlw	152^149	; case 152
	skipnz
	goto	l30756
	xorlw	159^152	; case 159
	skipnz
	goto	l30854
	xorlw	160^159	; case 160
	skipnz
	goto	l30932
	xorlw	250^160	; case 250
	skipnz
	goto	l30964
	goto	l30972
	opt asmopt_on

	line	2424
	
l30978:	
;diag.c: 2417: else
;diag.c: 2418: {
;diag.c: 2424: NegativeAnswer(ui8_selected_lid);
	movlb 0	; select bank0
	movf	(_ui8_selected_lid),w
	fcall	_NegativeAnswer
	line	2430
	
l7822:	
	return
	opt stack 0
GLOBAL	__end_of_EOL
	__end_of_EOL:
;; =============== function _EOL ends ============

	signat	_EOL,88
	global	_Receive_Diag
psect	text2021,local,class=CODE,delta=2
global __ptext2021
__ptext2021:

;; *************** function _Receive_Diag *****************
;; Defined at:
;;		line 317 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
;; Parameters:    Size  Location     Type
;;  id              1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  id              1   27[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr1l, fsr1h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/2
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       1       0       0       0       0       0       0
;;      Temps:          0       1       0       0       0       0       0       0
;;      Totals:         0       2       0       0       0       0       0       0
;;Total ram usage:        2 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    7
;; This function calls:
;;		__ELINMIntReceiveMessage
;;		__ELINMIntGetPointer
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2021
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
	line	317
	global	__size_of_Receive_Diag
	__size_of_Receive_Diag	equ	__end_of_Receive_Diag-_Receive_Diag
	
_Receive_Diag:	
	opt	stack 8
; Regs used in _Receive_Diag: [wreg+fsr1l-status,0+pclath+cstack]
;Receive_Diag@id stored from wreg
	movwf	(Receive_Diag@id)
	line	319
	
l30372:	
	line	322
;lin.c: 320: {
;lin.c: 321: ;
	
l13370:	
	line	319
	btfss	(__ELINMIntStatus),3
	goto	u10691
	goto	u10690
u10691:
	goto	l13370
u10690:
	line	324
	
l30374:	
;lin.c: 322: }
;lin.c: 323: ;
;lin.c: 324: _ELINMIntReceiveMessage ( 5, id, 8 );
	movf	(Receive_Diag@id),w
	movwf	(?__ELINMIntReceiveMessage)
	movlw	(08h)
	movwf	0+(?__ELINMIntReceiveMessage)+01h
	movlw	(05h)
	fcall	__ELINMIntReceiveMessage
	line	328
;lin.c: 326: {
	
l13373:	
	line	325
	btfss	(__ELINMIntStatus),3
	goto	u10701
	goto	u10700
u10701:
	goto	l13373
u10700:
	line	330
	
l30376:	
;lin.c: 328: }
;lin.c: 329: ;
;lin.c: 330: if( ( ErrorCode == ( _ELINMIntStatus.ELINMIntStatusByte & 0xF0 ) ))
	movlw	(0F0h)
	andwf	(__ELINMIntStatus),w
	movwf	(??_Receive_Diag+0)+0
	movf	0+(??_Receive_Diag+0)+0,w
	movlb 2	; select bank2
	xorwf	(_ErrorCode)^0100h,w
	skipz
	goto	u10711
	goto	u10710
u10711:
	goto	l30380
u10710:
	goto	l13378
	line	337
	
l30380:	
;lin.c: 335: else
;lin.c: 336: {
;lin.c: 337: pt = _ELINMIntGetPointer ( 5, 0 );
	movlb 0	; select bank0
	clrf	(?__ELINMIntGetPointer)
	movlw	(05h)
	fcall	__ELINMIntGetPointer
	movwf	(_pt)
	line	338
	
l30382:	
;lin.c: 338: my_msg[ 0 ] = *pt;
	movf	(_pt),w
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	indf1,w
	movlb 4	; select bank4
	movwf	(_my_msg)^0200h
	line	339
	
l30384:	
;lin.c: 339: pt++;
	movlb 0	; select bank0
	incf	(_pt),f
	line	340
	
l30386:	
;lin.c: 340: my_msg[ 1 ] = *pt;
	movf	(_pt),w
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	indf1,w
	movlb 4	; select bank4
	movwf	0+(_my_msg)^0200h+01h
	line	341
	
l30388:	
;lin.c: 341: pt++;
	movlb 0	; select bank0
	incf	(_pt),f
	line	342
	
l30390:	
;lin.c: 342: my_msg[ 2 ] = *pt;
	movf	(_pt),w
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	indf1,w
	movlb 4	; select bank4
	movwf	0+(_my_msg)^0200h+02h
	line	343
	
l30392:	
;lin.c: 343: pt++;
	movlb 0	; select bank0
	incf	(_pt),f
	line	344
	
l30394:	
;lin.c: 344: my_msg[ 3 ] = *pt;
	movf	(_pt),w
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	indf1,w
	movlb 4	; select bank4
	movwf	0+(_my_msg)^0200h+03h
	line	345
	
l30396:	
;lin.c: 345: pt++;
	movlb 0	; select bank0
	incf	(_pt),f
	line	346
	
l30398:	
;lin.c: 346: my_msg[ 4 ] = *pt;
	movf	(_pt),w
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	indf1,w
	movlb 4	; select bank4
	movwf	0+(_my_msg)^0200h+04h
	line	347
	
l30400:	
;lin.c: 347: pt++;
	movlb 0	; select bank0
	incf	(_pt),f
	line	348
	
l30402:	
;lin.c: 348: my_msg[ 5 ] = *pt;
	movf	(_pt),w
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	indf1,w
	movlb 4	; select bank4
	movwf	0+(_my_msg)^0200h+05h
	line	349
	
l30404:	
;lin.c: 349: pt++;
	movlb 0	; select bank0
	incf	(_pt),f
	line	350
	
l30406:	
;lin.c: 350: my_msg[ 6 ] = *pt;
	movf	(_pt),w
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	indf1,w
	movlb 4	; select bank4
	movwf	0+(_my_msg)^0200h+06h
	line	351
	
l30408:	
;lin.c: 351: pt++;
	movlb 0	; select bank0
	incf	(_pt),f
	line	352
	
l30410:	
;lin.c: 352: my_msg[ 7 ] = *pt;
	movf	(_pt),w
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	indf1,w
	movlb 4	; select bank4
	movwf	0+(_my_msg)^0200h+07h
	line	354
	
l30412:	
;lin.c: 354: ui8_b_DResServID_c = my_msg[ 0 ];
	movf	(_my_msg)^0200h,w
	movlb 2	; select bank2
	movwf	(_ui8_b_DResServID_c)^0100h
	line	355
	
l30414:	
;lin.c: 355: ui8_b_DResLocID_c = my_msg[ 1 ];
	movlb 4	; select bank4
	movf	0+(_my_msg)^0200h+01h,w
	movlb 0	; select bank0
	movwf	(_ui8_b_DResLocID_c)
	line	356
	
l30416:	
;lin.c: 356: ui8_b_DResB0_c = my_msg[ 2 ];
	movlb 4	; select bank4
	movf	0+(_my_msg)^0200h+02h,w
	movlb 0	; select bank0
	movwf	(_ui8_b_DResB0_c)
	line	357
	
l30418:	
;lin.c: 357: ui8_b_DResB1_c = my_msg[ 3 ];
	movlb 4	; select bank4
	movf	0+(_my_msg)^0200h+03h,w
	movlb 2	; select bank2
	movwf	(_ui8_b_DResB1_c)^0100h
	line	358
	
l30420:	
;lin.c: 358: ui8_b_DResB2_c = my_msg[ 4 ];
	movlb 4	; select bank4
	movf	0+(_my_msg)^0200h+04h,w
	movlb 2	; select bank2
	movwf	(_ui8_b_DResB2_c)^0100h
	line	359
	
l30422:	
;lin.c: 359: ui8_b_DResB3_c = my_msg[ 5 ];
	movlb 4	; select bank4
	movf	0+(_my_msg)^0200h+05h,w
	movlb 2	; select bank2
	movwf	(_ui8_b_DResB3_c)^0100h
	line	360
	
l30424:	
;lin.c: 361: ui8_b_DResB5_c = my_msg[ 7 ];
	movlb 4	; select bank4
	movf	0+(_my_msg)^0200h+06h,w
	movlb 2	; select bank2
	movwf	(_ui8_b_DResB4_c)^0100h
	line	367
	
l13378:	
	return
	opt stack 0
GLOBAL	__end_of_Receive_Diag
	__end_of_Receive_Diag:
;; =============== function _Receive_Diag ends ============

	signat	_Receive_Diag,4216
	global	_NegativeAnswer
psect	text2022,local,class=CODE,delta=2
global __ptext2022
__ptext2022:

;; *************** function _NegativeAnswer *****************
;; Defined at:
;;		line 2438 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
;; Parameters:    Size  Location     Type
;;  LID             1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  LID             1   32[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr1l, fsr1h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 1D/2
;;		On exit  : 1F/2
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       1       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       1       0       0       0       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    7
;; This function calls:
;;		_Transmit_LIN_8Bytes
;; This function is called by:
;;		_EOL
;; This function uses a non-reentrant model
;;
psect	text2022
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
	line	2438
	global	__size_of_NegativeAnswer
	__size_of_NegativeAnswer	equ	__end_of_NegativeAnswer-_NegativeAnswer
	
_NegativeAnswer:	
	opt	stack 7
; Regs used in _NegativeAnswer: [wreg+fsr1l-status,0+pclath+cstack]
;NegativeAnswer@LID stored from wreg
	line	2440
	movlb 0	; select bank0
	movwf	(NegativeAnswer@LID)
	
l30368:	
;diag.c: 2440: ui8_b_DResB0_c = 0xFF;
	movlw	(0FFh)
	movwf	(_ui8_b_DResB0_c)
	line	2441
;diag.c: 2441: ui8_b_DResB1_c = 0xFF;
	movlw	(0FFh)
	movlb 2	; select bank2
	movwf	(_ui8_b_DResB1_c)^0100h
	line	2442
;diag.c: 2442: ui8_b_DResB2_c = 0xFF;
	movlw	(0FFh)
	movwf	(_ui8_b_DResB2_c)^0100h
	line	2443
;diag.c: 2443: ui8_b_DResB3_c = 0xFF;
	movlw	(0FFh)
	movwf	(_ui8_b_DResB3_c)^0100h
	line	2444
;diag.c: 2445: ui8_b_DResB5_c = 0xFF;
	movlw	(0FFh)
	movwf	(_ui8_b_DResB4_c)^0100h
	line	2447
	
l30370:	
;diag.c: 2447: Transmit_LIN_8Bytes( 0x10, 0x7F, LID, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF );
	movlw	(07Fh)
	movlb 0	; select bank0
	movwf	(?_Transmit_LIN_8Bytes)
	movf	(NegativeAnswer@LID),w
	movwf	0+(?_Transmit_LIN_8Bytes)+01h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+02h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+03h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+04h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+05h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+06h
	movlw	(0FFh)
	movwf	0+(?_Transmit_LIN_8Bytes)+07h
	movlw	(010h)
	fcall	_Transmit_LIN_8Bytes
	line	2450
	
l7825:	
	return
	opt stack 0
GLOBAL	__end_of_NegativeAnswer
	__end_of_NegativeAnswer:
;; =============== function _NegativeAnswer ends ============

	signat	_NegativeAnswer,4216
	global	_init_bldc
psect	text2023,local,class=CODE,delta=2
global __ptext2023
__ptext2023:

;; *************** function _init_bldc *****************
;; Defined at:
;;		line 209 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\bldc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, fsr1l, fsr1h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 1F/3
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    7
;; This function calls:
;;		_InitMotorRun
;; This function is called by:
;;		_system_init
;; This function uses a non-reentrant model
;;
psect	text2023
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\bldc.c"
	line	209
	global	__size_of_init_bldc
	__size_of_init_bldc	equ	__end_of_init_bldc-_init_bldc
	
_init_bldc:	
	opt	stack 7
; Regs used in _init_bldc: [wreg-status,0+pclath+cstack]
	line	217
	
l30338:	
;bldc.c: 217: PSTR1CON = 0;
	movlb 5	; select bank5
	clrf	(662)^0280h	;volatile
	line	221
	
l30340:	
;bldc.c: 221: CM1CON0 = 0x80;
	movlw	(080h)
	movlb 2	; select bank2
	movwf	(273)^0100h	;volatile
	line	227
	
l30342:	
;bldc.c: 227: CM1CON1 = 0x00;
	clrf	(274)^0100h	;volatile
	line	229
	
l30344:	
;bldc.c: 229: C1IE = 0;
	movlb 1	; select bank1
	bcf	(1173/8)^080h,(1173)&7
	line	234
;bldc.c: 234: PWM1CON = 0x87;
	movlw	(087h)
	movlb 5	; select bank5
	movwf	(660)^0280h	;volatile
	line	238
	
l30346:	
;bldc.c: 238: CCP1AS = 0x800;
	clrf	(661)^0280h	;volatile
	line	239
	
l30348:	
;bldc.c: 239: CCP1CON = 0x0C;
	movlw	(0Ch)
	movwf	(659)^0280h	;volatile
	line	245
;bldc.c: 245: CCPR1L = 0;
	clrf	(657)^0280h	;volatile
	line	247
	
l30350:	
;bldc.c: 247: CCP2IE = 0;
	movlb 1	; select bank1
	bcf	(1168/8)^080h,(1168)&7
	line	248
	
l30352:	
;bldc.c: 248: CCP2CON = 0x04;
	movlw	(04h)
	movlb 5	; select bank5
	movwf	(666)^0280h	;volatile
	line	249
	
l30354:	
;bldc.c: 249: CCPR2L = 0xff;
	movlw	(0FFh)
	movwf	(664)^0280h	;volatile
	line	250
	
l30356:	
;bldc.c: 250: CCPR2H = 0xff;
	movlw	(0FFh)
	movwf	(665)^0280h	;volatile
	line	252
	
l30358:	
;bldc.c: 252: PR2 = ( unsigned char ) ( 32000000UL / ( 16 * 20000UL ) );
	movlw	(064h)
	movlb 0	; select bank0
	movwf	(27)	;volatile
	line	254
	
l30360:	
;bldc.c: 254: T2CON = 0x05;
	movlw	(05h)
	movwf	(28)	;volatile
	line	259
	
l30362:	
;bldc.c: 259: ui8_BlankingCount = ( unsigned char ) ( 0.002 * 20000UL );
	movlw	(028h)
	movlb 2	; select bank2
	movwf	(_ui8_BlankingCount)^0100h
	line	263
	
l30364:	
;bldc.c: 263: TMR2IE = 1;
	movlb 1	; select bank1
	bsf	(1161/8)^080h,(1161)&7
	line	266
;bldc.c: 266: MotorFlags.b = 0;
	movlb 0	; select bank0
	clrf	(_MotorFlags)
	line	269
	
l30366:	
;bldc.c: 269: InitMotorRun( );
	fcall	_InitMotorRun
	line	271
	
l3788:	
	return
	opt stack 0
GLOBAL	__end_of_init_bldc
	__end_of_init_bldc:
;; =============== function _init_bldc ends ============

	signat	_init_bldc,88
	global	_BVH2_Appl_Layer
psect	text2024,local,class=CODE,delta=2
global __ptext2024
__ptext2024:

;; *************** function _BVH2_Appl_Layer *****************
;; Defined at:
;;		line 335 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\T_Link\BVH2_Appl_Layer.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  Sb3_Sum1        2   54[BANK1 ] short 
;;  Sb3_Product1    2    0        unsigned short 
;;  Sb4_PI_sum      2   56[BANK1 ] short 
;;  Aux_U32         4   41[BANK1 ] unsigned long 
;;  Aux_S32_b       4   37[BANK1 ] long 
;;  Aux_S32_a       4   29[BANK1 ] long 
;;  Aux_S32         4    0        long 
;;  Aux_U32_a       4    0[BANK1 ] unsigned long 
;;  Sb2_Switch2     2   51[BANK1 ] unsigned short 
;;  Sb2_Error       2   49[BANK1 ] short 
;;  Sb2_Switch5     2   35[BANK1 ] unsigned short 
;;  Sb4_Product2    2    8[BANK1 ] short 
;;  Sb1_Logical_    1   53[BANK1 ] unsigned char 
;;  Sb2_Logical_    1   48[BANK1 ] unsigned char 
;;  Sb1_Logical_    1   47[BANK1 ] unsigned char 
;;  Sb1_Logical_    1   46[BANK1 ] unsigned char 
;;  Cb18_Reset      1   45[BANK1 ] unsigned char 
;;  Sb1_Logical_    1   18[BANK1 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 1F/2
;;		On exit  : 1F/3
;;		Unchanged: FFE00/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0      58       0       0       0       0       0
;;      Temps:          0       4       0       0       0       0       0       0
;;      Totals:         0       4      58       0       0       0       0       0
;;Total ram usage:       62 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    6
;; This function calls:
;;		_Cb37_Pic_etat_monitor_node_fcn1
;;		_Cb27_PWM_Detection_node_fcn1
;;		_Cb53_UbatHandling_node_fcn2
;;		_Cb1_Current_An___High_node_fcn1
;;		___wmul
;;		___lmul
;;		___aldiv
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2024
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\T_Link\BVH2_Appl_Layer.c"
	line	335
	global	__size_of_BVH2_Appl_Layer
	__size_of_BVH2_Appl_Layer	equ	__end_of_BVH2_Appl_Layer-_BVH2_Appl_Layer
	
_BVH2_Appl_Layer:	
	opt	stack 9
; Regs used in _BVH2_Appl_Layer: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	415
	
l29648:	
;BVH2_Appl_Layer.c: 337: static UInt16 Cb13_StateCnt = 0;
;BVH2_Appl_Layer.c: 338: static UInt16 Cb18_BadCnt = 0;
;BVH2_Appl_Layer.c: 339: static UInt16 Cb18_StateCnt = 0;
;BVH2_Appl_Layer.c: 340: static UInt8 Cb44_Counter = 0;
;BVH2_Appl_Layer.c: 341: static UInt8 Cb8_StateCnt = 0;
;BVH2_Appl_Layer.c: 343: static struct tag_SIBFS_Current_Analysis_low_ SIBFS_Current_Analysis_low_b = {
;BVH2_Appl_Layer.c: 344: 0 ,
;BVH2_Appl_Layer.c: 345: 0 ,
;BVH2_Appl_Layer.c: 346: 0 ,
;BVH2_Appl_Layer.c: 347: 0 ,
;BVH2_Appl_Layer.c: 348: 0
	goto	l29710
	line	418
	
l29650:	
;BVH2_Appl_Layer.c: 418: if (Cb37_StateCnt > 50) {
	movlw	high(033h)
	subwf	(_Cb37_StateCnt+1)^0100h,w
	movlw	low(033h)
	skipnz
	subwf	(_Cb37_StateCnt)^0100h,w
	skipc
	goto	u9481
	goto	u9480
u9481:
	goto	l29658
u9480:
	line	421
	
l29652:	
;BVH2_Appl_Layer.c: 421: SIBFS_Pic_etat_monitor_b.Cb37_Pic_etat_monitor_ns = (unsigned int) (UInt8)5;
	movf	(_SIBFS_Pic_etat_monitor_b)^0100h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(05h & ((1<<3)-1))<<0
	movwf	(_SIBFS_Pic_etat_monitor_b)^0100h
	line	422
	
l29654:	
;BVH2_Appl_Layer.c: 423: Cb37_oAlarm = 0;
	movlb 3	; select bank3
	clrf	(_Cb37_oShutoff)^0180h
	line	424
	
l29656:	
;BVH2_Appl_Layer.c: 424: Cb37_StateCnt = 0 ;
	movlb 2	; select bank2
	clrf	(_Cb37_StateCnt)^0100h
	clrf	(_Cb37_StateCnt+1)^0100h
	line	425
;BVH2_Appl_Layer.c: 425: }
	goto	l29712
	line	427
	
l29658:	
;BVH2_Appl_Layer.c: 426: else {
;BVH2_Appl_Layer.c: 427: Cb37_StateCnt = Cb37_StateCnt + 1 ;
	movlb 2	; select bank2
	incf	(_Cb37_StateCnt)^0100h,f
	skipnz
	incf	(_Cb37_StateCnt+1)^0100h,f
	goto	l29712
	line	435
	
l29660:	
;BVH2_Appl_Layer.c: 435: Cb37_Pic_etat_monitor_node_fcn1();
	fcall	_Cb37_Pic_etat_monitor_node_fcn1
	line	436
	
l29662:	
;BVH2_Appl_Layer.c: 436: if (SIBFS_Pic_etat_monitor_b.Cb37_glflag <= 2) {
	rrf	(_SIBFS_Pic_etat_monitor_b)^0100h,w
	movlb 0	; select bank0
	movwf	(??_BVH2_Appl_Layer+0)+0
	rrf	(??_BVH2_Appl_Layer+0)+0,f
	rrf	(??_BVH2_Appl_Layer+0)+0,w
	andlw	(1<<2)-1
	xorlw	03h
	skipnz
	goto	u9491
	goto	u9490
u9491:
	goto	l29712
u9490:
	goto	l29658
	line	445
	
l29666:	
;BVH2_Appl_Layer.c: 445: Cb37_Pic_etat_monitor_node_fcn1();
	fcall	_Cb37_Pic_etat_monitor_node_fcn1
	line	448
;BVH2_Appl_Layer.c: 448: break;
	goto	l29712
	line	452
	
l29668:	
;BVH2_Appl_Layer.c: 452: if (Cb37_StateCnt > 15) {
	movlw	high(010h)
	subwf	(_Cb37_StateCnt+1)^0100h,w
	movlw	low(010h)
	skipnz
	subwf	(_Cb37_StateCnt)^0100h,w
	skipc
	goto	u9501
	goto	u9500
u9501:
	goto	l29658
u9500:
	goto	l29652
	line	484
	
l29684:	
;BVH2_Appl_Layer.c: 484: SIBFS_Pic_etat_monitor_b.Cb37_Pic_etat_monitor_ns = (unsigned int) (UInt8)3;
	movf	(_SIBFS_Pic_etat_monitor_b)^0100h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(03h & ((1<<3)-1))<<0
	movwf	(_SIBFS_Pic_etat_monitor_b)^0100h
	line	485
	
l29686:	
;BVH2_Appl_Layer.c: 486: Cb37_oAlarm = 0;
	movlb 3	; select bank3
	clrf	(_Cb37_oShutoff)^0180h
	line	487
;BVH2_Appl_Layer.c: 487: }
	goto	l29712
	line	512
	
l29698:	
;BVH2_Appl_Layer.c: 512: SIBFS_Pic_etat_monitor_b.Cb37_Pic_etat_monitor_ns = (unsigned int) (UInt8)6;
	movf	(_SIBFS_Pic_etat_monitor_b)^0100h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(06h & ((1<<3)-1))<<0
	movwf	(_SIBFS_Pic_etat_monitor_b)^0100h
	goto	l29656
	line	519
;BVH2_Appl_Layer.c: 518: }
;BVH2_Appl_Layer.c: 519: default: {
	
l17529:	
	line	521
;BVH2_Appl_Layer.c: 521: if (!(SIBFS_Pic_etat_monitor_b.Cb37_Pic_etat_monitor)) {
	btfsc	(_SIBFS_Pic_etat_monitor_b)^0100h,5
	goto	u9511
	goto	u9510
u9511:
	goto	l29712
u9510:
	line	522
	
l29702:	
;BVH2_Appl_Layer.c: 522: SIBFS_Pic_etat_monitor_b.Cb37_Pic_etat_monitor = 1;
	bsf	(_SIBFS_Pic_etat_monitor_b)^0100h,5
	line	526
	
l29704:	
;BVH2_Appl_Layer.c: 526: if (ui8_ResetMatlab != 0) {
	movf	(_ui8_ResetMatlab)^0100h,w
	skipz
	goto	u9520
	goto	l29712
u9520:
	line	528
	
l29706:	
;BVH2_Appl_Layer.c: 528: SIBFS_Pic_etat_monitor_b.Cb37_Pic_etat_monitor_ns = (unsigned int) (UInt8)4;
	movf	(_SIBFS_Pic_etat_monitor_b)^0100h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(04h & ((1<<3)-1))<<0
	movwf	(_SIBFS_Pic_etat_monitor_b)^0100h
	goto	l29656
	line	415
	
l29710:	
	movf	(_SIBFS_Pic_etat_monitor_b)^0100h,w
	andlw	(1<<3)-1
	; Switch size 1, requested type "space"
; Number of cases is 6, Range of values is 1 to 6
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           19    10 (average)
; direct_byte           21     9 (fixed)
; jumptable            263     9 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	1^0	; case 1
	skipnz
	goto	l29698
	xorlw	2^1	; case 2
	skipnz
	goto	l29684
	xorlw	3^2	; case 3
	skipnz
	goto	l29666
	xorlw	4^3	; case 4
	skipnz
	goto	l29650
	xorlw	5^4	; case 5
	skipnz
	goto	l29660
	xorlw	6^5	; case 6
	skipnz
	goto	l29668
	goto	l17529
	opt asmopt_on

	line	541
	
l29712:	
;BVH2_Appl_Layer.c: 541: Cb27_idPWM = ui8_PWM_dc_mat;
	movlb 1	; select bank1
	movf	(_ui8_PWM_dc_mat)^080h,w
	movwf	(_Cb27_idPWM)^080h
	line	546
	
l29714:	
;BVH2_Appl_Layer.c: 546: if (SIBFS_PWM_Detection_b.Cb28_PWMinput_handling) {
	movlb 2	; select bank2
	btfss	(_SIBFS_PWM_Detection_b)^0100h,4
	goto	u9531
	goto	u9530
u9531:
	goto	l17532
u9530:
	goto	l29736
	line	552
	
l29718:	
;BVH2_Appl_Layer.c: 552: Cb27_PWM_Detection_node_fcn1();
	fcall	_Cb27_PWM_Detection_node_fcn1
	line	555
;BVH2_Appl_Layer.c: 555: break;
	goto	l29752
	line	548
	
l29736:	
	movf	(_SIBFS_PWM_Detection_b)^0100h,w
	andlw	(1<<4)-1
	; Switch size 1, requested type "space"
; Number of cases is 8, Range of values is 1 to 8
; switch strategies available:
; Name         Instructions Cycles
; direct_byte           25     9 (fixed)
; simple_byte           25    13 (average)
; jumptable            263     9 (fixed)
;	Chosen strategy is direct_byte

	addlw	-1
	skipc
goto l29752
	movwf fsr0l
	movlw	8
	subwf	fsr0l,w
skipnc
goto l29752
movlp high(S31324)
	lslf fsr0l,w
	addlw low(S31324)
	movwf pc
psect	swtext1,local,class=CONST,delta=2
global __pswtext1
__pswtext1:
S31324:
	ljmp	l29718
	ljmp	l29718
	ljmp	l29718
	ljmp	l29718
	ljmp	l29718
	ljmp	l29718
	ljmp	l29718
	ljmp	l29718
psect	text2024

	line	618
	
l17532:	
	line	619
;BVH2_Appl_Layer.c: 618: else {
;BVH2_Appl_Layer.c: 619: SIBFS_PWM_Detection_b.Cb28_PWMinput_handling = 1;
	bsf	(_SIBFS_PWM_Detection_b)^0100h,4
	line	622
	
l29738:	
;BVH2_Appl_Layer.c: 622: if (ui8_ResetMatlab == 1) {
	decf	(_ui8_ResetMatlab)^0100h,w
	skipz
	goto	u9541
	goto	u9540
u9541:
	goto	l29752
u9540:
	line	624
	
l29740:	
;BVH2_Appl_Layer.c: 624: SIBFS_PWM_Detection_b.Cb28_PWMinput_handling_ns = (unsigned int) (UInt8)1;
	movf	(_SIBFS_PWM_Detection_b)^0100h,w
	andlw	not (((1<<4)-1)<<0)
	iorlw	(01h & ((1<<4)-1))<<0
	movwf	(_SIBFS_PWM_Detection_b)^0100h
	line	625
	
l29742:	
;BVH2_Appl_Layer.c: 625: Cb27_odPumpOff = 1;
	movlb 3	; select bank3
	clrf	(_Cb27_odPumpOff)^0180h
	incf	(_Cb27_odPumpOff)^0180h,f
	line	626
	
l29744:	
;BVH2_Appl_Layer.c: 626: Cb27_odFixedValueSel = 0;
	clrf	(_Cb27_odFixedValueSel)^0180h
	line	627
	
l29746:	
;BVH2_Appl_Layer.c: 627: Cb27_odFixedLowValueSel = 0;
	clrf	(_Cb27_odFixedLowValueSel)^0180h
	line	628
	
l29748:	
;BVH2_Appl_Layer.c: 628: Cb27_oPWM_SC_Alarm = 0;
	clrf	(_Cb27_oPWM_SC_Alarm)^0180h
	line	629
	
l29750:	
;BVH2_Appl_Layer.c: 629: Cb27_oPWM_Alarm = 0;
	movlb 2	; select bank2
	clrf	(_Cb27_oPWM_Alarm)^0100h
	line	636
	
l29752:	
;BVH2_Appl_Layer.c: 630: }
;BVH2_Appl_Layer.c: 631: }
;BVH2_Appl_Layer.c: 636: bool_PWMin_Freq_err_Alarm = Cb27_oPWM_SC_Alarm;
	movlb 3	; select bank3
	movf	(_Cb27_oPWM_SC_Alarm)^0180h,w
	movlb 2	; select bank2
	movwf	(_bool_PWMin_Freq_err_Alarm)^0100h
	line	641
	
l29754:	
;BVH2_Appl_Layer.c: 641: if (SIBFS_UbatHandling_b.Cb54_Ubat_Handling) {
	movlb 1	; select bank1
	btfss	(_SIBFS_UbatHandling_b)^080h,3
	goto	u9551
	goto	u9550
u9551:
	goto	l17545
u9550:
	line	645
	
l29756:	
;BVH2_Appl_Layer.c: 645: if (SIBFS_UbatHandling_b.Cb55_SaturationHigh) {
	btfss	(_SIBFS_UbatHandling_b)^080h,4
	goto	u9561
	goto	u9560
u9561:
	goto	l17546
u9560:
	line	647
	
l29758:	
;BVH2_Appl_Layer.c: 647: SIBFS_UbatHandling_b.Aux_sflag3 = 4 ;
	movf	(_SIBFS_UbatHandling_b)^080h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(04h & ((1<<3)-1))<<0
	movwf	(_SIBFS_UbatHandling_b)^080h
	line	650
	
l29760:	
;BVH2_Appl_Layer.c: 650: if (((UInt8)ui8_BattVolt_mat) < 222) {
	movlw	(0DEh)
	subwf	(_ui8_BattVolt_mat)^080h,w
	skipnc
	goto	u9571
	goto	u9570
u9571:
	goto	l17558
u9570:
	line	651
	
l29762:	
;BVH2_Appl_Layer.c: 651: Cb53_UbatHandling_node_fcn2();
	fcall	_Cb53_UbatHandling_node_fcn2
	goto	l17558
	line	656
	
l17546:	
	line	658
;BVH2_Appl_Layer.c: 656: else {
;BVH2_Appl_Layer.c: 658: if (SIBFS_UbatHandling_b.Cb56_SaturationLow) {
	btfss	(_SIBFS_UbatHandling_b)^080h,5
	goto	u9581
	goto	u9580
u9581:
	goto	l17549
u9580:
	line	660
	
l29764:	
;BVH2_Appl_Layer.c: 660: SIBFS_UbatHandling_b.Aux_sflag3 = 1 ;
	movf	(_SIBFS_UbatHandling_b)^080h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(01h & ((1<<3)-1))<<0
	movwf	(_SIBFS_UbatHandling_b)^080h
	line	663
	
l29766:	
;BVH2_Appl_Layer.c: 663: if (((UInt8)ui8_BattVolt_mat) > 49) {
	movlw	(032h)
	subwf	(_ui8_BattVolt_mat)^080h,w
	skipc
	goto	u9591
	goto	u9590
u9591:
	goto	l17558
u9590:
	goto	l29762
	line	669
	
l17549:	
	line	671
;BVH2_Appl_Layer.c: 669: else {
;BVH2_Appl_Layer.c: 671: if (SIBFS_UbatHandling_b.Cb57_NormalUbat) {
	btfss	(_SIBFS_UbatHandling_b)^080h,6
	goto	u9601
	goto	u9600
u9601:
	goto	l17552
u9600:
	line	673
	
l29770:	
;BVH2_Appl_Layer.c: 673: SIBFS_UbatHandling_b.Aux_sflag3 = 3 ;
	movf	(_SIBFS_UbatHandling_b)^080h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(03h & ((1<<3)-1))<<0
	movwf	(_SIBFS_UbatHandling_b)^080h
	goto	l29762
	line	678
	
l17552:	
	line	680
;BVH2_Appl_Layer.c: 678: else {
;BVH2_Appl_Layer.c: 680: if (SIBFS_UbatHandling_b.Cb58_LimpRangeHigh) {
	btfss	(_SIBFS_UbatHandling_b)^080h,7
	goto	u9611
	goto	u9610
u9611:
	goto	l17558
u9610:
	line	683
	
l29774:	
;BVH2_Appl_Layer.c: 683: SIBFS_UbatHandling_b.Aux_sflag3 = 2 ;
	movf	(_SIBFS_UbatHandling_b)^080h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(02h & ((1<<3)-1))<<0
	movwf	(_SIBFS_UbatHandling_b)^080h
	line	687
	
l29776:	
;BVH2_Appl_Layer.c: 687: if ((((UInt8)ui8_BattVolt_mat) > 66) || (((UInt8)ui8_BattVolt_mat) < 45)) {
	movlw	(043h)
	subwf	(_ui8_BattVolt_mat)^080h,w
	skipnc
	goto	u9621
	goto	u9620
u9621:
	goto	l29762
u9620:
	
l29778:	
	movlw	(02Dh)
	subwf	(_ui8_BattVolt_mat)^080h,w
	skipnc
	goto	u9631
	goto	u9630
u9631:
	goto	l17558
u9630:
	goto	l29762
	line	700
	
l17545:	
	line	701
;BVH2_Appl_Layer.c: 700: else {
;BVH2_Appl_Layer.c: 701: SIBFS_UbatHandling_b.Cb54_Ubat_Handling = 1;
	bsf	(_SIBFS_UbatHandling_b)^080h,3
	line	704
	
l29782:	
;BVH2_Appl_Layer.c: 704: if (ui8_ResetMatlab == 1) {
	movlb 2	; select bank2
	decf	(_ui8_ResetMatlab)^0100h,w
	skipz
	goto	u9641
	goto	u9640
u9641:
	goto	l17558
u9640:
	line	706
	
l29784:	
;BVH2_Appl_Layer.c: 706: SIBFS_UbatHandling_b.Cb57_NormalUbat = 1;
	movlb 1	; select bank1
	bsf	(_SIBFS_UbatHandling_b)^080h,6
	line	707
	
l29786:	
;BVH2_Appl_Layer.c: 707: Cb53_odPumpOff = 0;
	movlb 3	; select bank3
	clrf	(_Cb53_odPumpOff)^0180h
	line	708
;BVH2_Appl_Layer.c: 708: Cb53_odFixedValueSel = 0;
	movlb 2	; select bank2
	clrf	(_Cb53_odFixedValueSel)^0100h
	line	709
;BVH2_Appl_Layer.c: 709: Cb53_oUbat_Alarm_High = 0;
	movlb 3	; select bank3
	clrf	(_Cb53_oUbat_Alarm_High)^0180h
	line	711
	
l17558:	
	line	714
;BVH2_Appl_Layer.c: 710: }
;BVH2_Appl_Layer.c: 711: }
;BVH2_Appl_Layer.c: 714: Sb1_Logical_Operator2 = Cb27_odPumpOff && (!(Cb53_odFixedValueSel));
	movlb 1	; select bank1
	clrf	(_BVH2_Appl_Layer$20275)^080h
	
l29788:	
	movlb 3	; select bank3
	movf	(_Cb27_odPumpOff)^0180h,w
	skipz
	goto	u9650
	goto	l29794
u9650:
	
l29790:	
	movlb 2	; select bank2
	movf	(_Cb53_odFixedValueSel)^0100h,f
	skipz
	goto	u9661
	goto	u9660
u9661:
	goto	l29794
u9660:
	
l29792:	
	movlb 1	; select bank1
	clrf	(_BVH2_Appl_Layer$20275)^080h
	incf	(_BVH2_Appl_Layer$20275)^080h,f
	
l29794:	
	movlb 1	; select bank1
	movf	(_BVH2_Appl_Layer$20275)^080h,w
	movwf	(BVH2_Appl_Layer@Sb1_Logical_Operator2)^080h
	line	719
	
l29796:	
;BVH2_Appl_Layer.c: 719: if (SIBFS_Temperature_Alarm_b.Cb45_CntOverTemp) {
	btfss	(BVH2_Appl_Layer@SIBFS_Temperature_Alarm_b)^080h,1
	goto	u9671
	goto	u9670
u9671:
	goto	l17562
u9670:
	line	721
	
l29798:	
;BVH2_Appl_Layer.c: 721: if (Cb44_Counter > 80) {
	movlw	(051h)
	movlb 3	; select bank3
	subwf	(BVH2_Appl_Layer@Cb44_Counter)^0180h,w
	skipc
	goto	u9681
	goto	u9680
u9681:
	goto	l29804
u9680:
	line	724
	
l29800:	
;BVH2_Appl_Layer.c: 724: SIBFS_Temperature_Alarm_b.Cb45_CntOverTemp = 0;
	movlb 1	; select bank1
	bcf	(BVH2_Appl_Layer@SIBFS_Temperature_Alarm_b)^080h,1
	line	725
;BVH2_Appl_Layer.c: 725: SIBFS_Temperature_Alarm_b.Cb47_greenTemp = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Temperature_Alarm_b)^080h,3
	line	726
	
l29802:	
;BVH2_Appl_Layer.c: 726: Cb44_oTempRedAlarm = 0;
	movlb 3	; select bank3
	clrf	(BVH2_Appl_Layer@Cb44_oTempRedAlarm)^0180h
	line	727
;BVH2_Appl_Layer.c: 727: Cb44_oTempAlarm = 0;
	clrf	(BVH2_Appl_Layer@Cb44_oTempAlarm)^0180h
	line	728
;BVH2_Appl_Layer.c: 728: Cb44_odPumpOff = 0;
	clrf	(BVH2_Appl_Layer@Cb44_odPumpOff)^0180h
	line	729
;BVH2_Appl_Layer.c: 729: Cb44_Counter = 0 ;
	clrf	(BVH2_Appl_Layer@Cb44_Counter)^0180h
	line	730
;BVH2_Appl_Layer.c: 730: }
	goto	l29906
	line	732
	
l29804:	
;BVH2_Appl_Layer.c: 731: else {
;BVH2_Appl_Layer.c: 732: Cb44_Counter = Cb44_Counter + 1 ;
	incf	(BVH2_Appl_Layer@Cb44_Counter)^0180h,f
	goto	l29906
	line	733
	
l17564:	
	line	736
;BVH2_Appl_Layer.c: 733: }
;BVH2_Appl_Layer.c: 736: }
	goto	l29906
	line	737
	
l17562:	
	line	739
;BVH2_Appl_Layer.c: 737: else {
;BVH2_Appl_Layer.c: 739: if (SIBFS_Temperature_Alarm_b.Cb46_reset) {
	btfss	(BVH2_Appl_Layer@SIBFS_Temperature_Alarm_b)^080h,2
	goto	u9691
	goto	u9690
u9691:
	goto	l17566
u9690:
	line	744
	
l29806:	
;BVH2_Appl_Layer.c: 744: SIBFS_Temperature_Alarm_b.Cb46_reset = 0;
	bcf	(BVH2_Appl_Layer@SIBFS_Temperature_Alarm_b)^080h,2
	line	745
;BVH2_Appl_Layer.c: 745: SIBFS_Temperature_Alarm_b.Cb45_CntOverTemp = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Temperature_Alarm_b)^080h,1
	line	748
;BVH2_Appl_Layer.c: 748: }
	goto	l29906
	line	749
	
l17566:	
	line	751
;BVH2_Appl_Layer.c: 749: else {
;BVH2_Appl_Layer.c: 751: if (SIBFS_Temperature_Alarm_b.Cb47_greenTemp) {
	btfss	(BVH2_Appl_Layer@SIBFS_Temperature_Alarm_b)^080h,3
	goto	u9701
	goto	u9700
u9701:
	goto	l17568
u9700:
	line	755
	
l29808:	
;BVH2_Appl_Layer.c: 755: if (ui16_mat_inpTemp < 72) {
	movlw	high(048h)
	movlb 3	; select bank3
	subwf	(_ui16_mat_inpTemp+1)^0180h,w
	movlw	low(048h)
	skipnz
	subwf	(_ui16_mat_inpTemp)^0180h,w
	skipnc
	goto	u9711
	goto	u9710
u9711:
	goto	l29906
u9710:
	line	758
	
l29810:	
;BVH2_Appl_Layer.c: 758: SIBFS_Temperature_Alarm_b.Cb47_greenTemp = 0;
	movlb 1	; select bank1
	bcf	(BVH2_Appl_Layer@SIBFS_Temperature_Alarm_b)^080h,3
	line	759
;BVH2_Appl_Layer.c: 759: SIBFS_Temperature_Alarm_b.Cb48_redTemp = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Temperature_Alarm_b)^080h,4
	line	760
;BVH2_Appl_Layer.c: 760: Cb44_oTempRedAlarm = 1;
	movlb 3	; select bank3
	clrf	(BVH2_Appl_Layer@Cb44_oTempRedAlarm)^0180h
	incf	(BVH2_Appl_Layer@Cb44_oTempRedAlarm)^0180h,f
	line	761
;BVH2_Appl_Layer.c: 761: Cb44_oTempAlarm = 1;
	clrf	(BVH2_Appl_Layer@Cb44_oTempAlarm)^0180h
	incf	(BVH2_Appl_Layer@Cb44_oTempAlarm)^0180h,f
	line	764
	
l29812:	
;BVH2_Appl_Layer.c: 764: Cb44_odPumpOff = 0;
	clrf	(BVH2_Appl_Layer@Cb44_odPumpOff)^0180h
	goto	l29906
	line	769
	
l17568:	
	line	771
;BVH2_Appl_Layer.c: 769: else {
;BVH2_Appl_Layer.c: 771: if (SIBFS_Temperature_Alarm_b.Cb48_redTemp) {
	btfss	(BVH2_Appl_Layer@SIBFS_Temperature_Alarm_b)^080h,4
	goto	u9721
	goto	u9720
u9721:
	goto	l17571
u9720:
	line	775
	
l29814:	
;BVH2_Appl_Layer.c: 775: if (ui16_mat_inpTemp > 185) {
	movlw	high(0BAh)
	movlb 3	; select bank3
	subwf	(_ui16_mat_inpTemp+1)^0180h,w
	movlw	low(0BAh)
	skipnz
	subwf	(_ui16_mat_inpTemp)^0180h,w
	skipc
	goto	u9731
	goto	u9730
u9731:
	goto	l29906
u9730:
	line	778
	
l29816:	
;BVH2_Appl_Layer.c: 778: SIBFS_Temperature_Alarm_b.Cb48_redTemp = 0;
	movlb 1	; select bank1
	bcf	(BVH2_Appl_Layer@SIBFS_Temperature_Alarm_b)^080h,4
	line	779
;BVH2_Appl_Layer.c: 779: SIBFS_Temperature_Alarm_b.Cb47_greenTemp = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Temperature_Alarm_b)^080h,3
	goto	l29802
	line	788
	
l17571:	
	line	790
;BVH2_Appl_Layer.c: 788: else {
;BVH2_Appl_Layer.c: 790: if (!(SIBFS_Temperature_Alarm_b.Cb44_Temperature_Alarm)) {
	btfsc	(BVH2_Appl_Layer@SIBFS_Temperature_Alarm_b)^080h,0
	goto	u9741
	goto	u9740
u9741:
	goto	l17564
u9740:
	line	791
	
l29820:	
;BVH2_Appl_Layer.c: 791: SIBFS_Temperature_Alarm_b.Cb44_Temperature_Alarm = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Temperature_Alarm_b)^080h,0
	line	795
	
l29822:	
;BVH2_Appl_Layer.c: 795: if (ui8_ResetMatlab != 0) {
	movlb 2	; select bank2
	movf	(_ui8_ResetMatlab)^0100h,w
	skipz
	goto	u9750
	goto	l29906
u9750:
	line	797
	
l29824:	
;BVH2_Appl_Layer.c: 797: SIBFS_Temperature_Alarm_b.Cb46_reset = 1;
	movlb 1	; select bank1
	bsf	(BVH2_Appl_Layer@SIBFS_Temperature_Alarm_b)^080h,2
	line	798
	
l29826:	
;BVH2_Appl_Layer.c: 798: Cb44_Counter = 0 ;
	movlb 3	; select bank3
	clrf	(BVH2_Appl_Layer@Cb44_Counter)^0180h
	goto	l29906
	line	812
	
l29828:	
;BVH2_Appl_Layer.c: 812: if (Cb1_StateCnt > 50) {
	movlw	high(033h)
	subwf	(_Cb1_StateCnt+1)^0100h,w
	movlw	low(033h)
	skipnz
	subwf	(_Cb1_StateCnt)^0100h,w
	skipc
	goto	u9761
	goto	u9760
u9761:
	goto	l29836
u9760:
	line	816
	
l29830:	
;BVH2_Appl_Layer.c: 815: SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High_ns = (unsigned int)
;BVH2_Appl_Layer.c: 816: (UInt8)5;
	movf	(_SIBFS_Current_Analysis_High_b)^0100h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(05h & ((1<<3)-1))<<0
	movwf	(_SIBFS_Current_Analysis_High_b)^0100h
	line	817
	
l29832:	
;BVH2_Appl_Layer.c: 817: Cb1_oShutoff = 0;
	clrf	(_Cb1_oShutoff)^0100h
	line	818
	
l29834:	
;BVH2_Appl_Layer.c: 818: Cb1_oCurrentAlarm = 0;
	clrf	(_Cb1_oCurrentAlarm)^0100h
	line	819
;BVH2_Appl_Layer.c: 819: }
	goto	l17580
	line	821
	
l29836:	
;BVH2_Appl_Layer.c: 820: else {
;BVH2_Appl_Layer.c: 821: Cb1_StateCnt = Cb1_StateCnt + 1 ;
	movlb 2	; select bank2
	incf	(_Cb1_StateCnt)^0100h,f
	skipnz
	incf	(_Cb1_StateCnt+1)^0100h,f
	goto	l17580
	line	829
	
l29838:	
;BVH2_Appl_Layer.c: 829: Cb1_Current_An___High_node_fcn1();
	fcall	_Cb1_Current_An___High_node_fcn1
	line	830
	
l29840:	
;BVH2_Appl_Layer.c: 830: if (SIBFS_Current_Analysis_High_b.Cb1_glflag <= 2) {
	rrf	(_SIBFS_Current_Analysis_High_b)^0100h,w
	movlb 0	; select bank0
	movwf	(??_BVH2_Appl_Layer+0)+0
	rrf	(??_BVH2_Appl_Layer+0)+0,f
	rrf	(??_BVH2_Appl_Layer+0)+0,w
	andlw	(1<<2)-1
	xorlw	03h
	skipnz
	goto	u9771
	goto	u9770
u9771:
	goto	l17580
u9770:
	goto	l29836
	line	839
	
l29844:	
;BVH2_Appl_Layer.c: 839: Cb1_Current_An___High_node_fcn1();
	fcall	_Cb1_Current_An___High_node_fcn1
	line	842
;BVH2_Appl_Layer.c: 842: break;
	goto	l17580
	line	846
	
l29846:	
;BVH2_Appl_Layer.c: 846: if (Cb1_StateCnt > 200) {
	movlw	high(0C9h)
	subwf	(_Cb1_StateCnt+1)^0100h,w
	movlw	low(0C9h)
	skipnz
	subwf	(_Cb1_StateCnt)^0100h,w
	skipc
	goto	u9781
	goto	u9780
u9781:
	goto	l29856
u9780:
	line	849
	
l29848:	
;BVH2_Appl_Layer.c: 848: SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High_ns = (unsigned int)
;BVH2_Appl_Layer.c: 849: (UInt8)3;
	movf	(_SIBFS_Current_Analysis_High_b)^0100h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(03h & ((1<<3)-1))<<0
	movwf	(_SIBFS_Current_Analysis_High_b)^0100h
	line	850
	
l29850:	
;BVH2_Appl_Layer.c: 850: Cb1_oShutoff = 0;
	clrf	(_Cb1_oShutoff)^0100h
	line	851
	
l29852:	
;BVH2_Appl_Layer.c: 851: Cb1_oCurrentAlarm = 1;
	clrf	(_Cb1_oCurrentAlarm)^0100h
	incf	(_Cb1_oCurrentAlarm)^0100h,f
	line	852
	
l29854:	
;BVH2_Appl_Layer.c: 852: Cb1_StateCnt = 0 ;
	clrf	(_Cb1_StateCnt)^0100h
	clrf	(_Cb1_StateCnt+1)^0100h
	line	853
;BVH2_Appl_Layer.c: 853: }
	goto	l17580
	line	857
	
l29856:	
;BVH2_Appl_Layer.c: 854: else {
;BVH2_Appl_Layer.c: 857: if (bool_mat_currAlarm_bldc) {
	movf	(_bool_mat_currAlarm_bldc)^0100h,w
	skipz
	goto	u9790
	goto	l29836
u9790:
	line	860
	
l29858:	
;BVH2_Appl_Layer.c: 859: SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High_ns = (unsigned int)
;BVH2_Appl_Layer.c: 860: (UInt8)1;
	movf	(_SIBFS_Current_Analysis_High_b)^0100h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(01h & ((1<<3)-1))<<0
	movwf	(_SIBFS_Current_Analysis_High_b)^0100h
	line	861
	
l29860:	
;BVH2_Appl_Layer.c: 861: Cb1_StateCnt = 0 ;
	clrf	(_Cb1_StateCnt)^0100h
	clrf	(_Cb1_StateCnt+1)^0100h
	line	862
	
l29862:	
;BVH2_Appl_Layer.c: 862: Cb1_oShutoff = 1;
	clrf	(_Cb1_oShutoff)^0100h
	incf	(_Cb1_oShutoff)^0100h,f
	line	863
	
l29864:	
;BVH2_Appl_Layer.c: 863: Cb1_oCurrentAlarm = 1;
	clrf	(_Cb1_oCurrentAlarm)^0100h
	incf	(_Cb1_oCurrentAlarm)^0100h,f
	line	864
;BVH2_Appl_Layer.c: 864: }
	goto	l17580
	line	878
	
l29868:	
;BVH2_Appl_Layer.c: 878: if (!(bool_mat_currAlarm_bldc)) {
	movf	(_bool_mat_currAlarm_bldc)^0100h,f
	skipz
	goto	u9801
	goto	u9800
u9801:
	goto	l29878
u9800:
	goto	l29848
	line	887
	
l29878:	
;BVH2_Appl_Layer.c: 886: else {
;BVH2_Appl_Layer.c: 887: if (Cb1_StateCnt > 1) {
	movlw	high(02h)
	subwf	(_Cb1_StateCnt+1)^0100h,w
	movlw	low(02h)
	skipnz
	subwf	(_Cb1_StateCnt)^0100h,w
	skipc
	goto	u9811
	goto	u9810
u9811:
	goto	l29836
u9810:
	goto	l29858
	line	908
	
l29890:	
;BVH2_Appl_Layer.c: 908: if (!(bool_mat_currAlarm_bldc)) {
	movf	(_bool_mat_currAlarm_bldc)^0100h,f
	skipz
	goto	u9821
	goto	u9820
u9821:
	goto	l29836
u9820:
	line	911
	
l29892:	
;BVH2_Appl_Layer.c: 911: SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High_ns = (unsigned int) (UInt8)4;
	movf	(_SIBFS_Current_Analysis_High_b)^0100h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(04h & ((1<<3)-1))<<0
	movwf	(_SIBFS_Current_Analysis_High_b)^0100h
	goto	l29854
	line	921
;BVH2_Appl_Layer.c: 920: }
;BVH2_Appl_Layer.c: 921: default: {
	
l17597:	
	line	923
;BVH2_Appl_Layer.c: 923: if (!(SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High)) {
	btfsc	(_SIBFS_Current_Analysis_High_b)^0100h,5
	goto	u9831
	goto	u9830
u9831:
	goto	l17580
u9830:
	line	924
	
l29898:	
;BVH2_Appl_Layer.c: 924: SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High = 1;
	bsf	(_SIBFS_Current_Analysis_High_b)^0100h,5
	line	928
	
l29900:	
;BVH2_Appl_Layer.c: 928: if (ui8_ResetMatlab != 0) {
	movf	(_ui8_ResetMatlab)^0100h,w
	skipz
	goto	u9840
	goto	l17580
u9840:
	line	931
	
l29902:	
;BVH2_Appl_Layer.c: 930: SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High_ns = (unsigned int)
;BVH2_Appl_Layer.c: 931: (UInt8)6;
	movf	(_SIBFS_Current_Analysis_High_b)^0100h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(06h & ((1<<3)-1))<<0
	movwf	(_SIBFS_Current_Analysis_High_b)^0100h
	goto	l29854
	line	809
	
l29906:	
	movlb 2	; select bank2
	movf	(_SIBFS_Current_Analysis_High_b)^0100h,w
	andlw	(1<<3)-1
	; Switch size 1, requested type "space"
; Number of cases is 6, Range of values is 1 to 6
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           19    10 (average)
; direct_byte           21     9 (fixed)
; jumptable            263     9 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	1^0	; case 1
	skipnz
	goto	l29890
	xorlw	2^1	; case 2
	skipnz
	goto	l29868
	xorlw	3^2	; case 3
	skipnz
	goto	l29838
	xorlw	4^3	; case 4
	skipnz
	goto	l29846
	xorlw	5^4	; case 5
	skipnz
	goto	l29844
	xorlw	6^5	; case 6
	skipnz
	goto	l29828
	goto	l17597
	opt asmopt_on

	line	936
	
l17580:	
	line	940
;BVH2_Appl_Layer.c: 939: Sb1_Logical_Operator1 = Sb1_Logical_Operator2 || Cb53_odPumpOff || Cb44_odPumpOff ||
;BVH2_Appl_Layer.c: 940: Cb1_oShutoff || Cb37_oShutoff;
	movlb 1	; select bank1
	clrf	(_BVH2_Appl_Layer$20276)^080h
	incf	(_BVH2_Appl_Layer$20276)^080h,f
	
l29908:	
	movf	(BVH2_Appl_Layer@Sb1_Logical_Operator2)^080h,f
	skipz
	goto	u9851
	goto	u9850
u9851:
	goto	l29920
u9850:
	
l29910:	
	movlb 3	; select bank3
	movf	(_Cb53_odPumpOff)^0180h,f
	skipz
	goto	u9861
	goto	u9860
u9861:
	goto	l29920
u9860:
	
l29912:	
	movf	(BVH2_Appl_Layer@Cb44_odPumpOff)^0180h,f
	skipz
	goto	u9871
	goto	u9870
u9871:
	goto	l29920
u9870:
	
l29914:	
	movlb 2	; select bank2
	movf	(_Cb1_oShutoff)^0100h,f
	skipz
	goto	u9881
	goto	u9880
u9881:
	goto	l29920
u9880:
	
l29916:	
	movlb 3	; select bank3
	movf	(_Cb37_oShutoff)^0180h,f
	skipz
	goto	u9891
	goto	u9890
u9891:
	goto	l29920
u9890:
	
l29918:	
	movlb 1	; select bank1
	clrf	(_BVH2_Appl_Layer$20276)^080h
	
l29920:	
	movlb 1	; select bank1
	movf	(_BVH2_Appl_Layer$20276)^080h,w
	movwf	(BVH2_Appl_Layer@Sb1_Logical_Operator1)^080h
	line	943
	
l29922:	
;BVH2_Appl_Layer.c: 943: Cb18_Reset = ui8_ResetMatlab != 0 ;
	movlw	0
	movlb 2	; select bank2
	movf	(_ui8_ResetMatlab)^0100h,f
	skipz
	movlw	1
	movlb 1	; select bank1
	movwf	(BVH2_Appl_Layer@Cb18_Reset)^080h
	line	948
	
l29924:	
;BVH2_Appl_Layer.c: 948: if (SIBFS_Motor_Stalled_b.Cb19_Motor_stalled_Statemachine) {
	btfss	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,6
	goto	u9901
	goto	u9900
u9901:
	goto	l17602
u9900:
	line	954
	
l29926:	
;BVH2_Appl_Layer.c: 954: if (Sb1_Logical_Operator1 || Cb18_Reset) {
	movf	(BVH2_Appl_Layer@Sb1_Logical_Operator1)^080h,f
	skipz
	goto	u9911
	goto	u9910
u9911:
	goto	l29946
u9910:
	
l29928:	
	movf	(BVH2_Appl_Layer@Cb18_Reset)^080h,w
	skipz
	goto	u9920
	goto	l30050
u9920:
	goto	l29946
	line	959
	
l29930:	
;BVH2_Appl_Layer.c: 959: Cb18_StateCnt = 0 ;
	movlb 2	; select bank2
	clrf	(BVH2_Appl_Layer@Cb18_StateCnt)^0100h
	clrf	(BVH2_Appl_Layer@Cb18_StateCnt+1)^0100h
	line	960
	
l29932:	
;BVH2_Appl_Layer.c: 960: SIBFS_Motor_Stalled_b.Cb19_Motor_sta__Statemachine_ns = 0 ;
	movlw	((0 & ((1<<3)-1))<<0)|not (((1<<3)-1)<<0)
	movlb 1	; select bank1
	andwf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,f
	line	961
;BVH2_Appl_Layer.c: 961: break;
	goto	l17608
	line	957
	
l29946:	
	movf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,w
	andlw	(1<<3)-1
	; Switch size 1, requested type "space"
; Number of cases is 6, Range of values is 1 to 6
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           19    10 (average)
; direct_byte           21     9 (fixed)
; jumptable            263     9 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	1^0	; case 1
	skipnz
	goto	l29932
	xorlw	2^1	; case 2
	skipnz
	goto	l29932
	xorlw	3^2	; case 3
	skipnz
	goto	l29930
	xorlw	4^3	; case 4
	skipnz
	goto	l29932
	xorlw	5^4	; case 5
	skipnz
	goto	l29932
	xorlw	6^5	; case 6
	skipnz
	goto	l29932
	goto	l17608
	opt asmopt_on

	line	983
	
l17608:	
	line	984
;BVH2_Appl_Layer.c: 984: SIBFS_Motor_Stalled_b.Cb19_Motor_stalled_Statemachine = 0;
	bcf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,6
	line	985
;BVH2_Appl_Layer.c: 985: SIBFS_Motor_Stalled_b.Cb26_default = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,7
	line	986
	
l29948:	
;BVH2_Appl_Layer.c: 986: Cb18_oMotorStalled = 0;
	movlb 3	; select bank3
	clrf	(BVH2_Appl_Layer@Cb18_oMotorStalled)^0180h
	line	987
;BVH2_Appl_Layer.c: 987: Cb18_oStalledAlarm = 0;
	clrf	(BVH2_Appl_Layer@Cb18_oStalledAlarm)^0180h
	line	988
;BVH2_Appl_Layer.c: 988: }
	goto	l30070
	line	994
	
l29950:	
;BVH2_Appl_Layer.c: 994: if (Cb18_StateCnt) {
	movlb 2	; select bank2
	movf	(BVH2_Appl_Layer@Cb18_StateCnt+1)^0100h,w
	iorwf	(BVH2_Appl_Layer@Cb18_StateCnt)^0100h,w
	skipnz
	goto	u9931
	goto	u9930
u9931:
	goto	l29956
u9930:
	line	998
	
l29952:	
;BVH2_Appl_Layer.c: 998: Cb18_StateCnt = 0 ;
	clrf	(BVH2_Appl_Layer@Cb18_StateCnt)^0100h
	clrf	(BVH2_Appl_Layer@Cb18_StateCnt+1)^0100h
	line	1000
	
l29954:	
;BVH2_Appl_Layer.c: 999: SIBFS_Motor_Stalled_b.Cb19_Motor_sta__Statemachine_ns = (unsigned int)
;BVH2_Appl_Layer.c: 1000: (UInt8)2;
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(02h & ((1<<3)-1))<<0
	movwf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h
	line	1001
;BVH2_Appl_Layer.c: 1001: }
	goto	l30070
	line	1003
	
l29956:	
;BVH2_Appl_Layer.c: 1002: else {
;BVH2_Appl_Layer.c: 1003: Cb18_StateCnt = Cb18_StateCnt + 1 ;
	movlb 2	; select bank2
	incf	(BVH2_Appl_Layer@Cb18_StateCnt)^0100h,f
	skipnz
	incf	(BVH2_Appl_Layer@Cb18_StateCnt+1)^0100h,f
	goto	l30070
	line	1013
	
l29958:	
;BVH2_Appl_Layer.c: 1013: if (Cb18_StateCnt > 1000) {
	movlw	high(03E9h)
	movlb 2	; select bank2
	subwf	(BVH2_Appl_Layer@Cb18_StateCnt+1)^0100h,w
	movlw	low(03E9h)
	skipnz
	subwf	(BVH2_Appl_Layer@Cb18_StateCnt)^0100h,w
	skipc
	goto	u9941
	goto	u9940
u9941:
	goto	l29970
u9940:
	line	1017
	
l29960:	
;BVH2_Appl_Layer.c: 1016: SIBFS_Motor_Stalled_b.Cb19_Motor_sta__Statemachine_ns = (unsigned int)
;BVH2_Appl_Layer.c: 1017: (UInt8)5;
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(05h & ((1<<3)-1))<<0
	movwf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h
	line	1018
	
l29962:	
;BVH2_Appl_Layer.c: 1018: Cb18_StateCnt = 0 ;
	movlb 2	; select bank2
	clrf	(BVH2_Appl_Layer@Cb18_StateCnt)^0100h
	clrf	(BVH2_Appl_Layer@Cb18_StateCnt+1)^0100h
	line	1019
	
l29964:	
;BVH2_Appl_Layer.c: 1019: Cb18_BadCnt = 0 ;
	movlb 4	; select bank4
	clrf	(BVH2_Appl_Layer@Cb18_BadCnt)^0200h
	clrf	(BVH2_Appl_Layer@Cb18_BadCnt+1)^0200h
	line	1020
	
l29966:	
;BVH2_Appl_Layer.c: 1020: Cb18_oMotorStalled = 0;
	movlb 3	; select bank3
	clrf	(BVH2_Appl_Layer@Cb18_oMotorStalled)^0180h
	line	1023
	
l29968:	
;BVH2_Appl_Layer.c: 1023: Cb18_oStalledAlarm = 0;
	clrf	(BVH2_Appl_Layer@Cb18_oStalledAlarm)^0180h
	line	1024
;BVH2_Appl_Layer.c: 1024: }
	goto	l30070
	line	1028
	
l29970:	
;BVH2_Appl_Layer.c: 1025: else {
;BVH2_Appl_Layer.c: 1028: if ((ui16_Speed_mat < 5) || (ui16_Speed_mat > 300)) {
	movlw	high(05h)
	movlb 1	; select bank1
	subwf	(_ui16_Speed_mat+1)^080h,w
	movlw	low(05h)
	skipnz
	subwf	(_ui16_Speed_mat)^080h,w
	skipc
	goto	u9951
	goto	u9950
u9951:
	goto	l29974
u9950:
	
l29972:	
	movlw	high(012Dh)
	subwf	(_ui16_Speed_mat+1)^080h,w
	movlw	low(012Dh)
	skipnz
	subwf	(_ui16_Speed_mat)^080h,w
	skipc
	goto	u9961
	goto	u9960
u9961:
	goto	l29956
u9960:
	line	1032
	
l29974:	
;BVH2_Appl_Layer.c: 1031: SIBFS_Motor_Stalled_b.Cb19_Motor_sta__Statemachine_ns = (unsigned int)
;BVH2_Appl_Layer.c: 1032: (UInt8)1;
	movf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(01h & ((1<<3)-1))<<0
	movwf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h
	line	1033
	
l29976:	
;BVH2_Appl_Layer.c: 1033: Cb18_oStalledAlarm = 0;
	movlb 3	; select bank3
	clrf	(BVH2_Appl_Layer@Cb18_oStalledAlarm)^0180h
	line	1034
	
l29978:	
;BVH2_Appl_Layer.c: 1034: Cb18_StateCnt = 0 ;
	movlb 2	; select bank2
	clrf	(BVH2_Appl_Layer@Cb18_StateCnt)^0100h
	clrf	(BVH2_Appl_Layer@Cb18_StateCnt+1)^0100h
	line	1035
;BVH2_Appl_Layer.c: 1035: }
	goto	l30070
	line	1048
	
l29982:	
;BVH2_Appl_Layer.c: 1048: if (Cb18_BadCnt > 100) {
	movlw	high(065h)
	movlb 4	; select bank4
	subwf	(BVH2_Appl_Layer@Cb18_BadCnt+1)^0200h,w
	movlw	low(065h)
	skipnz
	subwf	(BVH2_Appl_Layer@Cb18_BadCnt)^0200h,w
	skipc
	goto	u9971
	goto	u9970
u9971:
	goto	l29992
u9970:
	line	1052
	
l29984:	
;BVH2_Appl_Layer.c: 1051: SIBFS_Motor_Stalled_b.Cb19_Motor_sta__Statemachine_ns = (unsigned int)
;BVH2_Appl_Layer.c: 1052: (UInt8)4;
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(04h & ((1<<3)-1))<<0
	movwf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h
	line	1053
	
l29986:	
;BVH2_Appl_Layer.c: 1053: Cb18_StateCnt = 0 ;
	movlb 2	; select bank2
	clrf	(BVH2_Appl_Layer@Cb18_StateCnt)^0100h
	clrf	(BVH2_Appl_Layer@Cb18_StateCnt+1)^0100h
	line	1054
	
l29988:	
;BVH2_Appl_Layer.c: 1054: Cb18_oMotorStalled = 1;
	movlb 3	; select bank3
	clrf	(BVH2_Appl_Layer@Cb18_oMotorStalled)^0180h
	incf	(BVH2_Appl_Layer@Cb18_oMotorStalled)^0180h,f
	line	1055
	
l29990:	
;BVH2_Appl_Layer.c: 1055: Cb18_oStalledAlarm = 1;
	clrf	(BVH2_Appl_Layer@Cb18_oStalledAlarm)^0180h
	incf	(BVH2_Appl_Layer@Cb18_oStalledAlarm)^0180h,f
	line	1056
;BVH2_Appl_Layer.c: 1056: }
	goto	l30070
	line	1060
	
l29992:	
;BVH2_Appl_Layer.c: 1057: else {
;BVH2_Appl_Layer.c: 1060: if ((ui16_Speed_mat >= 5) && (ui16_Speed_mat <= 300)) {
	movlw	high(05h)
	movlb 1	; select bank1
	subwf	(_ui16_Speed_mat+1)^080h,w
	movlw	low(05h)
	skipnz
	subwf	(_ui16_Speed_mat)^080h,w
	skipc
	goto	u9981
	goto	u9980
u9981:
	goto	l29998
u9980:
	
l29994:	
	movlw	high(012Dh)
	subwf	(_ui16_Speed_mat+1)^080h,w
	movlw	low(012Dh)
	skipnz
	subwf	(_ui16_Speed_mat)^080h,w
	skipnc
	goto	u9991
	goto	u9990
u9991:
	goto	l29998
u9990:
	goto	l29954
	line	1067
	
l29998:	
;BVH2_Appl_Layer.c: 1066: else {
;BVH2_Appl_Layer.c: 1067: Cb18_BadCnt = Cb18_BadCnt + 1 ;
	movlb 4	; select bank4
	incf	(BVH2_Appl_Layer@Cb18_BadCnt)^0200h,f
	skipnz
	incf	(BVH2_Appl_Layer@Cb18_BadCnt+1)^0200h,f
	line	1068
;BVH2_Appl_Layer.c: 1068: Cb18_StateCnt = Cb18_StateCnt + 1 ;
	movlb 2	; select bank2
	incf	(BVH2_Appl_Layer@Cb18_StateCnt)^0100h,f
	skipnz
	incf	(BVH2_Appl_Layer@Cb18_StateCnt+1)^0100h,f
	goto	l30070
	line	1093
	
l30002:	
;BVH2_Appl_Layer.c: 1093: if (Cb18_StateCnt > 10) {
	movlw	high(0Bh)
	movlb 2	; select bank2
	subwf	(BVH2_Appl_Layer@Cb18_StateCnt+1)^0100h,w
	movlw	low(0Bh)
	skipnz
	subwf	(BVH2_Appl_Layer@Cb18_StateCnt)^0100h,w
	skipc
	goto	u10001
	goto	u10000
u10001:
	goto	l30032
u10000:
	line	1096
	
l30004:	
;BVH2_Appl_Layer.c: 1096: if ((ui16_Speed_mat < 5) || (ui16_Speed_mat > 300)) {
	movlw	high(05h)
	movlb 1	; select bank1
	subwf	(_ui16_Speed_mat+1)^080h,w
	movlw	low(05h)
	skipnz
	subwf	(_ui16_Speed_mat)^080h,w
	skipc
	goto	u10011
	goto	u10010
u10011:
	goto	l30008
u10010:
	
l30006:	
	movlw	high(012Dh)
	subwf	(_ui16_Speed_mat+1)^080h,w
	movlw	low(012Dh)
	skipnz
	subwf	(_ui16_Speed_mat)^080h,w
	skipc
	goto	u10021
	goto	u10020
u10021:
	goto	l30018
u10020:
	line	1100
	
l30008:	
;BVH2_Appl_Layer.c: 1099: SIBFS_Motor_Stalled_b.Cb19_Motor_sta__Statemachine_ns = (unsigned int)
;BVH2_Appl_Layer.c: 1100: (UInt8)4;
	movf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(04h & ((1<<3)-1))<<0
	movwf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h
	line	1101
	
l30010:	
;BVH2_Appl_Layer.c: 1101: Cb18_StateCnt = 0 ;
	movlb 2	; select bank2
	clrf	(BVH2_Appl_Layer@Cb18_StateCnt)^0100h
	clrf	(BVH2_Appl_Layer@Cb18_StateCnt+1)^0100h
	line	1102
	
l30012:	
;BVH2_Appl_Layer.c: 1102: Cb18_oMotorStalled = 1;
	movlb 3	; select bank3
	clrf	(BVH2_Appl_Layer@Cb18_oMotorStalled)^0180h
	incf	(BVH2_Appl_Layer@Cb18_oMotorStalled)^0180h,f
	line	1103
	
l30014:	
;BVH2_Appl_Layer.c: 1103: Cb18_oStalledAlarm = 1;
	clrf	(BVH2_Appl_Layer@Cb18_oStalledAlarm)^0180h
	incf	(BVH2_Appl_Layer@Cb18_oStalledAlarm)^0180h,f
	line	1104
	
l30016:	
;BVH2_Appl_Layer.c: 1104: SIBFS_Motor_Stalled_b.Cb18_glflag = 3 ;
	movlw	(03h & ((1<<2)-1))<<3
	movlb 1	; select bank1
	iorwf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,f
	line	1105
;BVH2_Appl_Layer.c: 1105: }
	goto	l17641
	line	1107
	
l30018:	
;BVH2_Appl_Layer.c: 1106: else {
;BVH2_Appl_Layer.c: 1107: if (Cb18_StateCnt > 200) {
	movlw	high(0C9h)
	movlb 2	; select bank2
	subwf	(BVH2_Appl_Layer@Cb18_StateCnt+1)^0100h,w
	movlw	low(0C9h)
	skipnz
	subwf	(BVH2_Appl_Layer@Cb18_StateCnt)^0100h,w
	skipc
	goto	u10031
	goto	u10030
u10031:
	goto	l30030
u10030:
	line	1111
	
l30020:	
;BVH2_Appl_Layer.c: 1110: SIBFS_Motor_Stalled_b.Cb19_Motor_sta__Statemachine_ns = (unsigned int)
;BVH2_Appl_Layer.c: 1111: (UInt8)5;
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(05h & ((1<<3)-1))<<0
	movwf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h
	line	1112
	
l30022:	
;BVH2_Appl_Layer.c: 1112: Cb18_StateCnt = 0 ;
	movlb 2	; select bank2
	clrf	(BVH2_Appl_Layer@Cb18_StateCnt)^0100h
	clrf	(BVH2_Appl_Layer@Cb18_StateCnt+1)^0100h
	line	1113
	
l30024:	
;BVH2_Appl_Layer.c: 1113: Cb18_BadCnt = 0 ;
	movlb 4	; select bank4
	clrf	(BVH2_Appl_Layer@Cb18_BadCnt)^0200h
	clrf	(BVH2_Appl_Layer@Cb18_BadCnt+1)^0200h
	line	1114
	
l30026:	
;BVH2_Appl_Layer.c: 1114: Cb18_oMotorStalled = 0;
	movlb 3	; select bank3
	clrf	(BVH2_Appl_Layer@Cb18_oMotorStalled)^0180h
	line	1117
	
l30028:	
;BVH2_Appl_Layer.c: 1117: Cb18_oStalledAlarm = 0;
	clrf	(BVH2_Appl_Layer@Cb18_oStalledAlarm)^0180h
	line	1118
;BVH2_Appl_Layer.c: 1118: SIBFS_Motor_Stalled_b.Cb18_glflag = 3 ;
	movlw	(03h & ((1<<2)-1))<<3
	movlb 1	; select bank1
	iorwf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,f
	line	1119
;BVH2_Appl_Layer.c: 1119: }
	goto	l17641
	line	1121
	
l30030:	
;BVH2_Appl_Layer.c: 1120: else {
;BVH2_Appl_Layer.c: 1121: SIBFS_Motor_Stalled_b.Cb18_glflag = 1 ;
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,w
	andlw	not (((1<<2)-1)<<3)
	iorlw	(01h & ((1<<2)-1))<<3
	movwf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h
	goto	l17641
	line	1126
	
l30032:	
;BVH2_Appl_Layer.c: 1125: else {
;BVH2_Appl_Layer.c: 1126: SIBFS_Motor_Stalled_b.Cb18_glflag = 0 ;
	movlw	((0 & ((1<<2)-1))<<3)|not (((1<<2)-1)<<3)
	movlb 1	; select bank1
	andwf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,f
	line	1127
	
l17641:	
	line	1128
;BVH2_Appl_Layer.c: 1127: }
;BVH2_Appl_Layer.c: 1128: if (SIBFS_Motor_Stalled_b.Cb18_glflag <= 2) {
	rrf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,w
	movlb 0	; select bank0
	movwf	(??_BVH2_Appl_Layer+0)+0
	rrf	(??_BVH2_Appl_Layer+0)+0,f
	rrf	(??_BVH2_Appl_Layer+0)+0,w
	andlw	(1<<2)-1
	xorlw	03h
	skipnz
	goto	u10041
	goto	u10040
u10041:
	goto	l17614
u10040:
	goto	l29956
	line	1139
	
l30036:	
;BVH2_Appl_Layer.c: 1139: if (Cb18_StateCnt > 200) {
	movlw	high(0C9h)
	movlb 2	; select bank2
	subwf	(BVH2_Appl_Layer@Cb18_StateCnt+1)^0100h,w
	movlw	low(0C9h)
	skipnz
	subwf	(BVH2_Appl_Layer@Cb18_StateCnt)^0100h,w
	skipc
	goto	u10051
	goto	u10050
u10051:
	goto	l29956
u10050:
	line	1144
	
l30038:	
;BVH2_Appl_Layer.c: 1143: SIBFS_Motor_Stalled_b.Cb19_Motor_sta__Statemachine_ns = (unsigned int)
;BVH2_Appl_Layer.c: 1144: (UInt8)6;
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(06h & ((1<<3)-1))<<0
	movwf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h
	line	1145
	
l30040:	
;BVH2_Appl_Layer.c: 1145: Cb18_StateCnt = 0 ;
	movlb 2	; select bank2
	clrf	(BVH2_Appl_Layer@Cb18_StateCnt)^0100h
	clrf	(BVH2_Appl_Layer@Cb18_StateCnt+1)^0100h
	line	1146
	
l30042:	
;BVH2_Appl_Layer.c: 1146: Cb18_oMotorStalled = 0;
	movlb 3	; select bank3
	clrf	(BVH2_Appl_Layer@Cb18_oMotorStalled)^0180h
	goto	l29990
	line	990
	
l30050:	
	movf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,w
	andlw	(1<<3)-1
	; Switch size 1, requested type "space"
; Number of cases is 6, Range of values is 1 to 6
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           19    10 (average)
; direct_byte           21     9 (fixed)
; jumptable            263     9 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	1^0	; case 1
	skipnz
	goto	l29982
	xorlw	2^1	; case 2
	skipnz
	goto	l29958
	xorlw	3^2	; case 3
	skipnz
	goto	l29950
	xorlw	4^3	; case 4
	skipnz
	goto	l30036
	xorlw	5^4	; case 5
	skipnz
	goto	l29954
	xorlw	6^5	; case 6
	skipnz
	goto	l30002
	goto	l30070
	opt asmopt_on

	line	1158
	
l17614:	
	line	1161
;BVH2_Appl_Layer.c: 1158: }
;BVH2_Appl_Layer.c: 1161: }
	goto	l30070
	line	1162
	
l17602:	
	line	1164
;BVH2_Appl_Layer.c: 1162: else {
;BVH2_Appl_Layer.c: 1164: if (SIBFS_Motor_Stalled_b.Cb26_default) {
	btfss	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,7
	goto	u10061
	goto	u10060
u10061:
	goto	l17647
u10060:
	line	1169
	
l30052:	
;BVH2_Appl_Layer.c: 1169: if (!(Sb1_Logical_Operator1)) {
	movf	(BVH2_Appl_Layer@Sb1_Logical_Operator1)^080h,f
	skipz
	goto	u10071
	goto	u10070
u10071:
	goto	l30070
u10070:
	line	1172
	
l30054:	
;BVH2_Appl_Layer.c: 1172: SIBFS_Motor_Stalled_b.Cb26_default = 0;
	bcf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,7
	line	1173
;BVH2_Appl_Layer.c: 1173: SIBFS_Motor_Stalled_b.Cb19_Motor_stalled_Statemachine = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,6
	line	1174
	
l30056:	
;BVH2_Appl_Layer.c: 1174: SIBFS_Motor_Stalled_b.Cb19_Motor_sta__Statemachine_ns = (unsigned int) (UInt8)3;
	movf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(03h & ((1<<3)-1))<<0
	movwf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h
	line	1175
	
l30058:	
;BVH2_Appl_Layer.c: 1175: Cb18_StateCnt = 0 ;
	movlb 2	; select bank2
	clrf	(BVH2_Appl_Layer@Cb18_StateCnt)^0100h
	clrf	(BVH2_Appl_Layer@Cb18_StateCnt+1)^0100h
	line	1176
	
l30060:	
;BVH2_Appl_Layer.c: 1176: Cb18_BadCnt = 0 ;
	movlb 4	; select bank4
	clrf	(BVH2_Appl_Layer@Cb18_BadCnt)^0200h
	clrf	(BVH2_Appl_Layer@Cb18_BadCnt+1)^0200h
	goto	l30070
	line	1181
	
l17647:	
	line	1183
;BVH2_Appl_Layer.c: 1181: else {
;BVH2_Appl_Layer.c: 1183: if (!(SIBFS_Motor_Stalled_b.Cb18_Motor_Stalled)) {
	btfsc	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,5
	goto	u10081
	goto	u10080
u10081:
	goto	l30070
u10080:
	line	1184
	
l30062:	
;BVH2_Appl_Layer.c: 1184: SIBFS_Motor_Stalled_b.Cb18_Motor_Stalled = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,5
	line	1187
	
l30064:	
;BVH2_Appl_Layer.c: 1187: if (Cb18_Reset) {
	movf	(BVH2_Appl_Layer@Cb18_Reset)^080h,w
	skipz
	goto	u10090
	goto	l30070
u10090:
	line	1189
	
l30066:	
;BVH2_Appl_Layer.c: 1189: SIBFS_Motor_Stalled_b.Cb26_default = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Motor_Stalled_b)^080h,7
	goto	l29948
	line	1203
	
l30070:	
;BVH2_Appl_Layer.c: 1203: bool_UbatAlarm = Cb53_oUbat_Alarm_High;
	movlb 3	; select bank3
	movf	(_Cb53_oUbat_Alarm_High)^0180h,w
	movlb 2	; select bank2
	movwf	(_bool_UbatAlarm)^0100h
	line	1204
	
l30072:	
;BVH2_Appl_Layer.c: 1204: Sb1_Logical_Operator5 = Cb27_odFixedValueSel || Cb53_odFixedValueSel;
	movlb 1	; select bank1
	clrf	(_BVH2_Appl_Layer$20277)^080h
	incf	(_BVH2_Appl_Layer$20277)^080h,f
	
l30074:	
	movlb 3	; select bank3
	movf	(_Cb27_odFixedValueSel)^0180h,f
	skipz
	goto	u10101
	goto	u10100
u10101:
	goto	l30080
u10100:
	
l30076:	
	movlb 2	; select bank2
	movf	(_Cb53_odFixedValueSel)^0100h,f
	skipz
	goto	u10111
	goto	u10110
u10111:
	goto	l30080
u10110:
	
l30078:	
	movlb 1	; select bank1
	clrf	(_BVH2_Appl_Layer$20277)^080h
	
l30080:	
	movlb 1	; select bank1
	movf	(_BVH2_Appl_Layer$20277)^080h,w
	movwf	(BVH2_Appl_Layer@Sb1_Logical_Operator5)^080h
	line	1208
	
l30082:	
;BVH2_Appl_Layer.c: 1208: if (bool_start_demand_mat) {
	movlb 2	; select bank2
	movf	(_bool_start_demand_mat)^0100h,w
	skipz
	goto	u10120
	goto	l30086
u10120:
	line	1209
	
l30084:	
;BVH2_Appl_Layer.c: 1209: Sb2_Switch5 = ui8_fixed_start_speed_mat;
	movlb 4	; select bank4
	movf	(_ui8_fixed_start_speed_mat+1)^0200h,w
	movlb 1	; select bank1
	movwf	(BVH2_Appl_Layer@Sb2_Switch5+1)^080h
	movlb 4	; select bank4
	movf	(_ui8_fixed_start_speed_mat)^0200h,w
	movlb 1	; select bank1
	movwf	(BVH2_Appl_Layer@Sb2_Switch5)^080h
	line	1210
;BVH2_Appl_Layer.c: 1210: }
	goto	l17655
	line	1214
	
l30086:	
;BVH2_Appl_Layer.c: 1211: else {
;BVH2_Appl_Layer.c: 1214: if (Cb27_odFixedLowValueSel) {
	movlb 3	; select bank3
	movf	(_Cb27_odFixedLowValueSel)^0180h,w
	skipz
	goto	u10130
	goto	l30090
u10130:
	line	1215
	
l30088:	
;BVH2_Appl_Layer.c: 1215: Sb2_Switch5 = ui16_Speed_demand_mat_min;
	movlb 4	; select bank4
	movf	(_ui16_Speed_demand_mat_min+1)^0200h,w
	movlb 1	; select bank1
	movwf	(BVH2_Appl_Layer@Sb2_Switch5+1)^080h
	movlb 4	; select bank4
	movf	(_ui16_Speed_demand_mat_min)^0200h,w
	movlb 1	; select bank1
	movwf	(BVH2_Appl_Layer@Sb2_Switch5)^080h
	line	1216
;BVH2_Appl_Layer.c: 1216: }
	goto	l17655
	line	1220
	
l30090:	
;BVH2_Appl_Layer.c: 1217: else {
;BVH2_Appl_Layer.c: 1220: if (Sb1_Logical_Operator5) {
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@Sb1_Logical_Operator5)^080h,w
	skipz
	goto	u10140
	goto	l30094
u10140:
	line	1221
	
l30092:	
;BVH2_Appl_Layer.c: 1221: Sb2_Switch5 = ui16_Speed_demand_mat_Max;
	movlb 4	; select bank4
	movf	(_ui16_Speed_demand_mat_Max+1)^0200h,w
	movlb 1	; select bank1
	movwf	(BVH2_Appl_Layer@Sb2_Switch5+1)^080h
	movlb 4	; select bank4
	movf	(_ui16_Speed_demand_mat_Max)^0200h,w
	movlb 1	; select bank1
	movwf	(BVH2_Appl_Layer@Sb2_Switch5)^080h
	line	1222
;BVH2_Appl_Layer.c: 1222: }
	goto	l17655
	line	1224
	
l30094:	
;BVH2_Appl_Layer.c: 1223: else {
;BVH2_Appl_Layer.c: 1224: Sb2_Switch5 = ui16_Speed_demand_mat;
	movlb 4	; select bank4
	movf	(_ui16_Speed_demand_mat+1)^0200h,w
	movlb 1	; select bank1
	movwf	(BVH2_Appl_Layer@Sb2_Switch5+1)^080h
	movlb 4	; select bank4
	movf	(_ui16_Speed_demand_mat)^0200h,w
	movlb 1	; select bank1
	movwf	(BVH2_Appl_Layer@Sb2_Switch5)^080h
	line	1227
	
l17655:	
	line	1230
;BVH2_Appl_Layer.c: 1225: }
;BVH2_Appl_Layer.c: 1226: }
;BVH2_Appl_Layer.c: 1227: }
;BVH2_Appl_Layer.c: 1230: Sb2_Error = (Int16) (ui16_Speed_mat - Sb2_Switch5);
	movf	(_ui16_Speed_mat+1)^080h,w
	movwf	(BVH2_Appl_Layer@Sb2_Error+1)^080h
	movf	(_ui16_Speed_mat)^080h,w
	movwf	(BVH2_Appl_Layer@Sb2_Error)^080h
	movf	(BVH2_Appl_Layer@Sb2_Switch5)^080h,w
	subwf	(BVH2_Appl_Layer@Sb2_Error)^080h,f
	movf	(BVH2_Appl_Layer@Sb2_Switch5+1)^080h,w
	subwfb	(BVH2_Appl_Layer@Sb2_Error+1)^080h,f
	line	1235
	
l30096:	
;BVH2_Appl_Layer.c: 1235: if (SIBFS_Dry_Running_b.Cb14_greenState) {
	btfss	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,1
	goto	u10151
	goto	u10150
u10151:
	goto	l17660
u10150:
	line	1239
	
l30098:	
;BVH2_Appl_Layer.c: 1239: Aux_U32 = ui16_mat_Current * ui16_Speed_mat;
	movf	(_ui16_mat_Current+1)^080h,w
	movlb 0	; select bank0
	movwf	(?___wmul+1)
	movlb 1	; select bank1
	movf	(_ui16_mat_Current)^080h,w
	movlb 0	; select bank0
	movwf	(?___wmul)
	movlb 1	; select bank1
	movf	(_ui16_Speed_mat+1)^080h,w
	movlb 0	; select bank0
	movwf	1+(?___wmul)+02h
	movlb 1	; select bank1
	movf	(_ui16_Speed_mat)^080h,w
	movlb 0	; select bank0
	movwf	0+(?___wmul)+02h
	fcall	___wmul
	movf	(0+(?___wmul)),w
	movlb 1	; select bank1
	movwf	(BVH2_Appl_Layer@Aux_U32)^080h
	movlb 0	; select bank0
	movf	(1+(?___wmul)),w
	movlb 1	; select bank1
	movwf	((BVH2_Appl_Layer@Aux_U32)^080h)+1
	clrf	2+((BVH2_Appl_Layer@Aux_U32)^080h)
	clrf	3+((BVH2_Appl_Layer@Aux_U32)^080h)
	line	1240
	
l30100:	
	
l30102:	
	movf	(BVH2_Appl_Layer@Aux_U32+3)^080h,w
	movwf	(BVH2_Appl_Layer@Aux_U32_a+3)^080h
	movf	(BVH2_Appl_Layer@Aux_U32+2)^080h,w
	movwf	(BVH2_Appl_Layer@Aux_U32_a+2)^080h
	movf	(BVH2_Appl_Layer@Aux_U32+1)^080h,w
	movwf	(BVH2_Appl_Layer@Aux_U32_a+1)^080h
	movf	(BVH2_Appl_Layer@Aux_U32)^080h,w
	movwf	(BVH2_Appl_Layer@Aux_U32_a)^080h

	line	1243
	
l30104:	
;BVH2_Appl_Layer.c: 1243: Aux_S32_a = ui16_dryRun_Thresh - 5 ;
	movlb 4	; select bank4
	movf	(_ui16_dryRun_Thresh)^0200h,w
	addlw	low(0FFFBh)
	movlb 1	; select bank1
	movwf	(BVH2_Appl_Layer@Aux_S32_a)^080h
	movlw	high(0FFFBh)
	movlb 4	; select bank4
	addwfc	(_ui16_dryRun_Thresh+1)^0200h,w
	movlb 1	; select bank1
	movwf	1+(BVH2_Appl_Layer@Aux_S32_a)^080h
	clrf	2+(BVH2_Appl_Layer@Aux_S32_a)^080h
	clrf	3+(BVH2_Appl_Layer@Aux_S32_a)^080h
	line	1244
	
l30106:	
;BVH2_Appl_Layer.c: 1244: Aux_S32_b = (Aux_S32_a) >> 31; Aux_U32 = (UInt32)(Aux_S32_a);;
	movf	(BVH2_Appl_Layer@Aux_S32_a+3)^080h,w
	movwf	(BVH2_Appl_Layer@Aux_S32_b+3)^080h
	movf	(BVH2_Appl_Layer@Aux_S32_a+2)^080h,w
	movwf	(BVH2_Appl_Layer@Aux_S32_b+2)^080h
	movf	(BVH2_Appl_Layer@Aux_S32_a+1)^080h,w
	movwf	(BVH2_Appl_Layer@Aux_S32_b+1)^080h
	movf	(BVH2_Appl_Layer@Aux_S32_a)^080h,w
	movwf	(BVH2_Appl_Layer@Aux_S32_b)^080h

	
l30108:	
	movlw	01Fh
	movlb 0	; select bank0
	movwf	(??_BVH2_Appl_Layer+0)+0
u10165:
	movlb 1	; select bank1
	asrf	(BVH2_Appl_Layer@Aux_S32_b+3)^080h,f
	rrf	(BVH2_Appl_Layer@Aux_S32_b+2)^080h,f
	rrf	(BVH2_Appl_Layer@Aux_S32_b+1)^080h,f
	rrf	(BVH2_Appl_Layer@Aux_S32_b)^080h,f
	movlb 0	; select bank0
	decfsz	(??_BVH2_Appl_Layer+0)+0&07fh,f
	goto	u10165

	
l30110:	
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@Aux_S32_a+3)^080h,w
	movwf	(BVH2_Appl_Layer@Aux_U32+3)^080h
	movf	(BVH2_Appl_Layer@Aux_S32_a+2)^080h,w
	movwf	(BVH2_Appl_Layer@Aux_U32+2)^080h
	movf	(BVH2_Appl_Layer@Aux_S32_a+1)^080h,w
	movwf	(BVH2_Appl_Layer@Aux_U32+1)^080h
	movf	(BVH2_Appl_Layer@Aux_S32_a)^080h,w
	movwf	(BVH2_Appl_Layer@Aux_U32)^080h

	line	1245
	
l30112:	
;BVH2_Appl_Layer.c: 1245: if (( (Aux_S32 < Aux_S32_b) ? 1 : ((Aux_S32 > Aux_S32_b) ? 0 : ((Aux_U32_a < Aux_U32) ? 1 : 0)) )) {
	movf	(BVH2_Appl_Layer@Aux_S32_b+3)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	0
	xorlw	80h
	subwf	btemp+1,w
	
	skipz
	goto	u10173
	movlw	0
	subwf	(BVH2_Appl_Layer@Aux_S32_b+2)^080h,w
	skipz
	goto	u10173
	movlw	0
	subwf	(BVH2_Appl_Layer@Aux_S32_b+1)^080h,w
	skipz
	goto	u10173
	movlw	01h
	subwf	(BVH2_Appl_Layer@Aux_S32_b)^080h,w
u10173:
	skipnc
	goto	u10171
	goto	u10170
u10171:
	goto	l30122
u10170:
	
l30114:	
	btfsc	(BVH2_Appl_Layer@Aux_S32_b+3)^080h,7
	goto	u10181
	goto	u10180
u10181:
	goto	l30118
u10180:
	
l30116:	
	movf	(BVH2_Appl_Layer@Aux_U32+3)^080h,w
	subwf	(BVH2_Appl_Layer@Aux_U32_a+3)^080h,w
	skipz
	goto	u10195
	movf	(BVH2_Appl_Layer@Aux_U32+2)^080h,w
	subwf	(BVH2_Appl_Layer@Aux_U32_a+2)^080h,w
	skipz
	goto	u10195
	movf	(BVH2_Appl_Layer@Aux_U32+1)^080h,w
	subwf	(BVH2_Appl_Layer@Aux_U32_a+1)^080h,w
	skipz
	goto	u10195
	movf	(BVH2_Appl_Layer@Aux_U32)^080h,w
	subwf	(BVH2_Appl_Layer@Aux_U32_a)^080h,w
u10195:
	movlw	0
	skipc
	movlw	1
	movwf	(_BVH2_Appl_Layer$20279)^080h
	clrf	(_BVH2_Appl_Layer$20279+1)^080h
	goto	l30120
	
l30118:	
	clrf	(_BVH2_Appl_Layer$20279)^080h
	clrf	(_BVH2_Appl_Layer$20279+1)^080h
	
l30120:	
	movf	(_BVH2_Appl_Layer$20279+1)^080h,w
	movwf	(_BVH2_Appl_Layer$20278+1)^080h
	movf	(_BVH2_Appl_Layer$20279)^080h,w
	movwf	(_BVH2_Appl_Layer$20278)^080h
	goto	l30124
	
l30122:	
	clrf	(_BVH2_Appl_Layer$20278)^080h
	incf	(_BVH2_Appl_Layer$20278)^080h,f
	clrf	(_BVH2_Appl_Layer$20278+1)^080h
	
l30124:	
	movf	(_BVH2_Appl_Layer$20278+1)^080h,w
	iorwf	(_BVH2_Appl_Layer$20278)^080h,w
	skipnz
	goto	u10201
	goto	u10200
u10201:
	goto	l30168
u10200:
	line	1248
	
l30126:	
;BVH2_Appl_Layer.c: 1248: SIBFS_Dry_Running_b.Cb14_greenState = 0;
	bcf	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,1
	line	1249
;BVH2_Appl_Layer.c: 1249: SIBFS_Dry_Running_b.Cb15_DryRunning = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,2
	line	1250
;BVH2_Appl_Layer.c: 1250: SIBFS_Dry_Running_b.Cb17_CntOverCurrent = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,4
	line	1251
	
l30128:	
;BVH2_Appl_Layer.c: 1251: Cb13_StateCnt = 0 ;
	movlb 4	; select bank4
	clrf	(BVH2_Appl_Layer@Cb13_StateCnt)^0200h
	clrf	(BVH2_Appl_Layer@Cb13_StateCnt+1)^0200h
	line	1252
;BVH2_Appl_Layer.c: 1252: Cb13_oCurrentAlarm = 0;
	movlb 3	; select bank3
	clrf	(BVH2_Appl_Layer@Cb13_oCurrentAlarm)^0180h
	goto	l30168
	line	1253
	
l17661:	
	line	1256
;BVH2_Appl_Layer.c: 1253: }
;BVH2_Appl_Layer.c: 1256: }
	goto	l30168
	line	1257
	
l17660:	
	line	1259
;BVH2_Appl_Layer.c: 1257: else {
;BVH2_Appl_Layer.c: 1259: if (SIBFS_Dry_Running_b.Cb15_DryRunning) {
	btfss	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,2
	goto	u10211
	goto	u10210
u10211:
	goto	l17671
u10210:
	line	1261
	
l30130:	
;BVH2_Appl_Layer.c: 1261: if ((ui8_PWM_dc_mat < 20) && (ui8_PWM_dc_mat > 4)) {
	movlw	(014h)
	subwf	(_ui8_PWM_dc_mat)^080h,w
	skipnc
	goto	u10221
	goto	u10220
u10221:
	goto	l17672
u10220:
	
l30132:	
	movlw	(05h)
	subwf	(_ui8_PWM_dc_mat)^080h,w
	skipc
	goto	u10231
	goto	u10230
u10231:
	goto	l17672
u10230:
	line	1266
	
l30134:	
;BVH2_Appl_Layer.c: 1266: if (SIBFS_Dry_Running_b.Cb16_redState) {
	btfss	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,3
	goto	u10241
	goto	u10240
u10241:
	goto	l17673
u10240:
	line	1267
	
l30136:	
;BVH2_Appl_Layer.c: 1267: SIBFS_Dry_Running_b.Cb16_redState = 0;
	bcf	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,3
	line	1268
;BVH2_Appl_Layer.c: 1268: }
	goto	l17674
	line	1269
	
l17673:	
	line	1271
;BVH2_Appl_Layer.c: 1269: else {
;BVH2_Appl_Layer.c: 1271: if (SIBFS_Dry_Running_b.Cb17_CntOverCurrent) {
	btfss	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,4
	goto	u10251
	goto	u10250
u10251:
	goto	l17674
u10250:
	line	1272
	
l30138:	
;BVH2_Appl_Layer.c: 1272: SIBFS_Dry_Running_b.Cb17_CntOverCurrent = 0;
	bcf	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,4
	line	1274
	
l17674:	
	line	1275
;BVH2_Appl_Layer.c: 1273: }
;BVH2_Appl_Layer.c: 1274: }
;BVH2_Appl_Layer.c: 1275: SIBFS_Dry_Running_b.Cb15_DryRunning = 0;
	bcf	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,2
	line	1276
;BVH2_Appl_Layer.c: 1276: SIBFS_Dry_Running_b.Cb14_greenState = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,1
	line	1277
	
l30140:	
;BVH2_Appl_Layer.c: 1277: Cb13_oCurrentAlarm = 0;
	movlb 3	; select bank3
	clrf	(BVH2_Appl_Layer@Cb13_oCurrentAlarm)^0180h
	line	1278
;BVH2_Appl_Layer.c: 1278: }
	goto	l30168
	line	1279
	
l17672:	
	line	1281
;BVH2_Appl_Layer.c: 1279: else {
;BVH2_Appl_Layer.c: 1281: if (SIBFS_Dry_Running_b.Cb16_redState) {
	btfss	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,3
	goto	u10261
	goto	u10260
u10261:
	goto	l17677
u10260:
	line	1286
	
l30142:	
;BVH2_Appl_Layer.c: 1286: if ((ui16_mat_Current * ui16_Speed_mat) > (ui16_dryRun_Thresh + 5)) {
	movf	(_ui16_mat_Current+1)^080h,w
	movlb 0	; select bank0
	movwf	(?___wmul+1)
	movlb 1	; select bank1
	movf	(_ui16_mat_Current)^080h,w
	movlb 0	; select bank0
	movwf	(?___wmul)
	movlb 1	; select bank1
	movf	(_ui16_Speed_mat+1)^080h,w
	movlb 0	; select bank0
	movwf	1+(?___wmul)+02h
	movlb 1	; select bank1
	movf	(_ui16_Speed_mat)^080h,w
	movlb 0	; select bank0
	movwf	0+(?___wmul)+02h
	fcall	___wmul
	movlb 4	; select bank4
	movf	(_ui16_dryRun_Thresh)^0200h,w
	addlw	low(05h)
	movlb 0	; select bank0
	movwf	(??_BVH2_Appl_Layer+0)+0
	movlw	high(05h)
	movlb 4	; select bank4
	addwfc	(_ui16_dryRun_Thresh+1)^0200h,w
	movlb 0	; select bank0
	movwf	1+(??_BVH2_Appl_Layer+0)+0
	movf	(1+(?___wmul)),w
	subwf	1+(??_BVH2_Appl_Layer+0)+0,w
	skipz
	goto	u10275
	movf	(0+(?___wmul)),w
	subwf	0+(??_BVH2_Appl_Layer+0)+0,w
u10275:
	skipnc
	goto	u10271
	goto	u10270
u10271:
	goto	l30168
u10270:
	line	1289
	
l30144:	
;BVH2_Appl_Layer.c: 1289: SIBFS_Dry_Running_b.Cb16_redState = 0;
	movlb 1	; select bank1
	bcf	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,3
	line	1290
;BVH2_Appl_Layer.c: 1290: SIBFS_Dry_Running_b.Cb15_DryRunning = 0;
	bcf	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,2
	line	1291
;BVH2_Appl_Layer.c: 1291: SIBFS_Dry_Running_b.Cb14_greenState = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,1
	goto	l30140
	line	1297
	
l17677:	
	line	1299
;BVH2_Appl_Layer.c: 1297: else {
;BVH2_Appl_Layer.c: 1299: if (SIBFS_Dry_Running_b.Cb17_CntOverCurrent) {
	btfss	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,4
	goto	u10281
	goto	u10280
u10281:
	goto	l17661
u10280:
	line	1305
	
l30148:	
;BVH2_Appl_Layer.c: 1305: if ((ui16_mat_Current * ui16_Speed_mat) > (ui16_dryRun_Thresh + 5)) {
	movf	(_ui16_mat_Current+1)^080h,w
	movlb 0	; select bank0
	movwf	(?___wmul+1)
	movlb 1	; select bank1
	movf	(_ui16_mat_Current)^080h,w
	movlb 0	; select bank0
	movwf	(?___wmul)
	movlb 1	; select bank1
	movf	(_ui16_Speed_mat+1)^080h,w
	movlb 0	; select bank0
	movwf	1+(?___wmul)+02h
	movlb 1	; select bank1
	movf	(_ui16_Speed_mat)^080h,w
	movlb 0	; select bank0
	movwf	0+(?___wmul)+02h
	fcall	___wmul
	movlb 4	; select bank4
	movf	(_ui16_dryRun_Thresh)^0200h,w
	addlw	low(05h)
	movlb 0	; select bank0
	movwf	(??_BVH2_Appl_Layer+0)+0
	movlw	high(05h)
	movlb 4	; select bank4
	addwfc	(_ui16_dryRun_Thresh+1)^0200h,w
	movlb 0	; select bank0
	movwf	1+(??_BVH2_Appl_Layer+0)+0
	movf	(1+(?___wmul)),w
	subwf	1+(??_BVH2_Appl_Layer+0)+0,w
	skipz
	goto	u10295
	movf	(0+(?___wmul)),w
	subwf	0+(??_BVH2_Appl_Layer+0)+0,w
u10295:
	skipnc
	goto	u10291
	goto	u10290
u10291:
	goto	l30154
u10290:
	line	1307
	
l30150:	
;BVH2_Appl_Layer.c: 1307: SIBFS_Dry_Running_b.Cb17_CntOverCurrent = 0;
	movlb 1	; select bank1
	bcf	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,4
	line	1308
;BVH2_Appl_Layer.c: 1308: SIBFS_Dry_Running_b.Cb15_DryRunning = 0;
	bcf	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,2
	line	1309
;BVH2_Appl_Layer.c: 1309: SIBFS_Dry_Running_b.Cb14_greenState = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,1
	goto	l30140
	line	1313
	
l30154:	
;BVH2_Appl_Layer.c: 1312: else {
;BVH2_Appl_Layer.c: 1313: if (Cb13_StateCnt > 2000) {
	movlw	high(07D1h)
	movlb 4	; select bank4
	subwf	(BVH2_Appl_Layer@Cb13_StateCnt+1)^0200h,w
	movlw	low(07D1h)
	skipnz
	subwf	(BVH2_Appl_Layer@Cb13_StateCnt)^0200h,w
	skipc
	goto	u10301
	goto	u10300
u10301:
	goto	l30158
u10300:
	line	1315
	
l30156:	
;BVH2_Appl_Layer.c: 1315: SIBFS_Dry_Running_b.Cb17_CntOverCurrent = 0;
	movlb 1	; select bank1
	bcf	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,4
	line	1316
;BVH2_Appl_Layer.c: 1316: SIBFS_Dry_Running_b.Cb16_redState = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,3
	line	1317
;BVH2_Appl_Layer.c: 1317: Cb13_oCurrentAlarm = 1;
	movlb 3	; select bank3
	clrf	(BVH2_Appl_Layer@Cb13_oCurrentAlarm)^0180h
	incf	(BVH2_Appl_Layer@Cb13_oCurrentAlarm)^0180h,f
	line	1318
;BVH2_Appl_Layer.c: 1318: }
	goto	l30168
	line	1320
	
l30158:	
;BVH2_Appl_Layer.c: 1319: else {
;BVH2_Appl_Layer.c: 1320: Cb13_StateCnt = Cb13_StateCnt + 1 ;
	incf	(BVH2_Appl_Layer@Cb13_StateCnt)^0200h,f
	skipnz
	incf	(BVH2_Appl_Layer@Cb13_StateCnt+1)^0200h,f
	goto	l30168
	line	1331
	
l17671:	
	line	1333
;BVH2_Appl_Layer.c: 1331: else {
;BVH2_Appl_Layer.c: 1333: if (!(SIBFS_Dry_Running_b.Cb13_Dry_Running)) {
	btfsc	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,0
	goto	u10311
	goto	u10310
u10311:
	goto	l30168
u10310:
	line	1334
	
l30160:	
;BVH2_Appl_Layer.c: 1334: SIBFS_Dry_Running_b.Cb13_Dry_Running = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,0
	line	1337
	
l30162:	
;BVH2_Appl_Layer.c: 1337: if (ui8_ResetMatlab == 1) {
	movlb 2	; select bank2
	decf	(_ui8_ResetMatlab)^0100h,w
	skipz
	goto	u10321
	goto	u10320
u10321:
	goto	l30168
u10320:
	line	1339
	
l30164:	
;BVH2_Appl_Layer.c: 1339: SIBFS_Dry_Running_b.Cb14_greenState = 1;
	movlb 1	; select bank1
	bsf	(BVH2_Appl_Layer@SIBFS_Dry_Running_b)^080h,1
	goto	l30140
	line	1349
	
l30168:	
;BVH2_Appl_Layer.c: 1341: }
;BVH2_Appl_Layer.c: 1342: }
;BVH2_Appl_Layer.c: 1343: }
;BVH2_Appl_Layer.c: 1344: }
;BVH2_Appl_Layer.c: 1349: bool_DryRunningAlarm = Cb13_oCurrentAlarm;
	movlb 3	; select bank3
	movf	(BVH2_Appl_Layer@Cb13_oCurrentAlarm)^0180h,w
	movlb 2	; select bank2
	movwf	(_bool_DryRunningAlarm)^0100h
	line	1352
	
l30170:	
;BVH2_Appl_Layer.c: 1352: bool_CPU_TempAlarm = Cb44_oTempAlarm;
	movlb 3	; select bank3
	movf	(BVH2_Appl_Layer@Cb44_oTempAlarm)^0180h,w
	movlb 2	; select bank2
	movwf	(_bool_CPU_TempAlarm)^0100h
	line	1357
	
l30172:	
;BVH2_Appl_Layer.c: 1357: if (SIBFS_Current_Analysis_low_b.Cb9_greenState) {
	movlb 1	; select bank1
	btfss	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,4
	goto	u10331
	goto	u10330
u10331:
	goto	l17688
u10330:
	line	1362
	
l30174:	
;BVH2_Appl_Layer.c: 1362: if (((Int32)ui16_mat_Current) > (ui16_Current_Thresh - 2)) {
	movlb 3	; select bank3
	movf	(_ui16_Current_Thresh)^0180h,w
	addlw	low(0FFFEh)
	movlb 0	; select bank0
	movwf	(??_BVH2_Appl_Layer+0)+0
	movlw	high(0FFFEh)
	movlb 3	; select bank3
	addwfc	(_ui16_Current_Thresh+1)^0180h,w
	movlb 0	; select bank0
	movwf	1+(??_BVH2_Appl_Layer+0)+0
	movlb 1	; select bank1
	movf	(_ui16_mat_Current+1)^080h,w
	movlb 0	; select bank0
	subwf	1+(??_BVH2_Appl_Layer+0)+0,w
	skipz
	goto	u10345
	movlb 1	; select bank1
	movf	(_ui16_mat_Current)^080h,w
	movlb 0	; select bank0
	subwf	0+(??_BVH2_Appl_Layer+0)+0,w
u10345:
	skipnc
	goto	u10341
	goto	u10340
u10341:
	goto	l30218
u10340:
	line	1365
	
l30176:	
;BVH2_Appl_Layer.c: 1365: SIBFS_Current_Analysis_low_b.Cb9_greenState = 0;
	movlb 1	; select bank1
	bcf	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,4
	line	1366
;BVH2_Appl_Layer.c: 1366: SIBFS_Current_Analysis_low_b.Cb11_CntOverCurrent = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,1
	line	1367
	
l30178:	
;BVH2_Appl_Layer.c: 1367: Cb8_StateCnt = 0 ;
	movlb 2	; select bank2
	clrf	(BVH2_Appl_Layer@Cb8_StateCnt)^0100h
	goto	l30218
	line	1372
	
l17688:	
	line	1374
;BVH2_Appl_Layer.c: 1372: else {
;BVH2_Appl_Layer.c: 1374: if (SIBFS_Current_Analysis_low_b.Cb10_Wait) {
	btfss	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,0
	goto	u10351
	goto	u10350
u10351:
	goto	l17691
u10350:
	line	1379
	
l30180:	
;BVH2_Appl_Layer.c: 1379: if (((Int32)ui16_mat_Current) > (ui16_Current_Thresh - 2)) {
	movlb 3	; select bank3
	movf	(_ui16_Current_Thresh)^0180h,w
	addlw	low(0FFFEh)
	movlb 0	; select bank0
	movwf	(??_BVH2_Appl_Layer+0)+0
	movlw	high(0FFFEh)
	movlb 3	; select bank3
	addwfc	(_ui16_Current_Thresh+1)^0180h,w
	movlb 0	; select bank0
	movwf	1+(??_BVH2_Appl_Layer+0)+0
	movlb 1	; select bank1
	movf	(_ui16_mat_Current+1)^080h,w
	movlb 0	; select bank0
	subwf	1+(??_BVH2_Appl_Layer+0)+0,w
	skipz
	goto	u10365
	movlb 1	; select bank1
	movf	(_ui16_mat_Current)^080h,w
	movlb 0	; select bank0
	subwf	0+(??_BVH2_Appl_Layer+0)+0,w
u10365:
	skipnc
	goto	u10361
	goto	u10360
u10361:
	goto	l30184
u10360:
	line	1381
	
l30182:	
;BVH2_Appl_Layer.c: 1381: SIBFS_Current_Analysis_low_b.Cb10_Wait = 0;
	movlb 1	; select bank1
	bcf	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,0
	line	1382
;BVH2_Appl_Layer.c: 1382: SIBFS_Current_Analysis_low_b.Cb12_redState = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,2
	line	1383
;BVH2_Appl_Layer.c: 1383: Cb8_oCurrentAlarm = 1;
	movlb 3	; select bank3
	clrf	(BVH2_Appl_Layer@Cb8_oCurrentAlarm)^0180h
	incf	(BVH2_Appl_Layer@Cb8_oCurrentAlarm)^0180h,f
	line	1384
;BVH2_Appl_Layer.c: 1384: }
	goto	l30218
	line	1386
	
l30184:	
;BVH2_Appl_Layer.c: 1385: else {
;BVH2_Appl_Layer.c: 1386: if (Cb8_StateCnt > 100) {
	movlw	(065h)
	movlb 2	; select bank2
	subwf	(BVH2_Appl_Layer@Cb8_StateCnt)^0100h,w
	skipc
	goto	u10371
	goto	u10370
u10371:
	goto	l30190
u10370:
	line	1388
	
l30186:	
;BVH2_Appl_Layer.c: 1388: SIBFS_Current_Analysis_low_b.Cb10_Wait = 0;
	movlb 1	; select bank1
	bcf	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,0
	line	1389
;BVH2_Appl_Layer.c: 1389: SIBFS_Current_Analysis_low_b.Cb9_greenState = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,4
	line	1390
	
l30188:	
;BVH2_Appl_Layer.c: 1390: Cb8_oCurrentAlarm = 0;
	movlb 3	; select bank3
	clrf	(BVH2_Appl_Layer@Cb8_oCurrentAlarm)^0180h
	line	1391
;BVH2_Appl_Layer.c: 1391: }
	goto	l30218
	line	1393
	
l30190:	
;BVH2_Appl_Layer.c: 1392: else {
;BVH2_Appl_Layer.c: 1393: Cb8_StateCnt = Cb8_StateCnt + 1 ;
	incf	(BVH2_Appl_Layer@Cb8_StateCnt)^0100h,f
	goto	l30218
	line	1399
	
l17691:	
	line	1401
;BVH2_Appl_Layer.c: 1399: else {
;BVH2_Appl_Layer.c: 1401: if (SIBFS_Current_Analysis_low_b.Cb11_CntOverCurrent) {
	btfss	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,1
	goto	u10381
	goto	u10380
u10381:
	goto	l17697
u10380:
	line	1406
	
l30192:	
;BVH2_Appl_Layer.c: 1406: if (((UInt32)ui16_mat_Current) < (ui16_Current_Thresh + 2)) {
	movlb 3	; select bank3
	movf	(_ui16_Current_Thresh)^0180h,w
	addlw	low(02h)
	movlb 0	; select bank0
	movwf	(??_BVH2_Appl_Layer+0)+0
	movlw	high(02h)
	movlb 3	; select bank3
	addwfc	(_ui16_Current_Thresh+1)^0180h,w
	movlb 0	; select bank0
	movwf	1+(??_BVH2_Appl_Layer+0)+0
	movf	1+(??_BVH2_Appl_Layer+0)+0,w
	movlb 1	; select bank1
	subwf	(_ui16_mat_Current+1)^080h,w
	skipz
	goto	u10395
	movlb 0	; select bank0
	movf	0+(??_BVH2_Appl_Layer+0)+0,w
	movlb 1	; select bank1
	subwf	(_ui16_mat_Current)^080h,w
u10395:
	skipnc
	goto	u10391
	goto	u10390
u10391:
	goto	l30198
u10390:
	line	1408
	
l30194:	
;BVH2_Appl_Layer.c: 1408: SIBFS_Current_Analysis_low_b.Cb11_CntOverCurrent = 0;
	bcf	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,1
	line	1409
;BVH2_Appl_Layer.c: 1409: SIBFS_Current_Analysis_low_b.Cb9_greenState = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,4
	goto	l30188
	line	1413
	
l30198:	
;BVH2_Appl_Layer.c: 1412: else {
;BVH2_Appl_Layer.c: 1413: if (Cb8_StateCnt > 50) {
	movlw	(033h)
	movlb 2	; select bank2
	subwf	(BVH2_Appl_Layer@Cb8_StateCnt)^0100h,w
	skipc
	goto	u10401
	goto	u10400
u10401:
	goto	l30190
u10400:
	line	1415
	
l30200:	
;BVH2_Appl_Layer.c: 1415: SIBFS_Current_Analysis_low_b.Cb11_CntOverCurrent = 0;
	movlb 1	; select bank1
	bcf	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,1
	line	1416
;BVH2_Appl_Layer.c: 1416: SIBFS_Current_Analysis_low_b.Cb12_redState = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,2
	line	1417
;BVH2_Appl_Layer.c: 1417: Cb8_oCurrentAlarm = 1;
	movlb 3	; select bank3
	clrf	(BVH2_Appl_Layer@Cb8_oCurrentAlarm)^0180h
	incf	(BVH2_Appl_Layer@Cb8_oCurrentAlarm)^0180h,f
	line	1418
;BVH2_Appl_Layer.c: 1418: }
	goto	l30218
	line	1426
	
l17697:	
	line	1428
;BVH2_Appl_Layer.c: 1426: else {
;BVH2_Appl_Layer.c: 1428: if (SIBFS_Current_Analysis_low_b.Cb12_redState) {
	btfss	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,2
	goto	u10411
	goto	u10410
u10411:
	goto	l17703
u10410:
	line	1433
	
l30204:	
;BVH2_Appl_Layer.c: 1433: if (((UInt32)ui16_mat_Current) < (ui16_Current_Thresh + 2)) {
	movlb 3	; select bank3
	movf	(_ui16_Current_Thresh)^0180h,w
	addlw	low(02h)
	movlb 0	; select bank0
	movwf	(??_BVH2_Appl_Layer+0)+0
	movlw	high(02h)
	movlb 3	; select bank3
	addwfc	(_ui16_Current_Thresh+1)^0180h,w
	movlb 0	; select bank0
	movwf	1+(??_BVH2_Appl_Layer+0)+0
	movf	1+(??_BVH2_Appl_Layer+0)+0,w
	movlb 1	; select bank1
	subwf	(_ui16_mat_Current+1)^080h,w
	skipz
	goto	u10425
	movlb 0	; select bank0
	movf	0+(??_BVH2_Appl_Layer+0)+0,w
	movlb 1	; select bank1
	subwf	(_ui16_mat_Current)^080h,w
u10425:
	skipnc
	goto	u10421
	goto	u10420
u10421:
	goto	l30218
u10420:
	line	1436
	
l30206:	
;BVH2_Appl_Layer.c: 1436: SIBFS_Current_Analysis_low_b.Cb12_redState = 0;
	bcf	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,2
	line	1437
;BVH2_Appl_Layer.c: 1437: SIBFS_Current_Analysis_low_b.Cb10_Wait = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,0
	goto	l30178
	line	1443
	
l17703:	
	line	1445
;BVH2_Appl_Layer.c: 1443: else {
;BVH2_Appl_Layer.c: 1445: if (!(SIBFS_Current_Analysis_low_b.Cb8_Current_Analysis_low)) {
	btfsc	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,3
	goto	u10431
	goto	u10430
u10431:
	goto	l30218
u10430:
	line	1446
	
l30210:	
;BVH2_Appl_Layer.c: 1446: SIBFS_Current_Analysis_low_b.Cb8_Current_Analysis_low = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,3
	line	1449
	
l30212:	
;BVH2_Appl_Layer.c: 1449: if (ui8_ResetMatlab == 1) {
	movlb 2	; select bank2
	decf	(_ui8_ResetMatlab)^0100h,w
	skipz
	goto	u10441
	goto	u10440
u10441:
	goto	l30218
u10440:
	line	1451
	
l30214:	
;BVH2_Appl_Layer.c: 1451: SIBFS_Current_Analysis_low_b.Cb9_greenState = 1;
	movlb 1	; select bank1
	bsf	(BVH2_Appl_Layer@SIBFS_Current_Analysis_low_b)^080h,4
	goto	l30188
	line	1463
	
l30218:	
;BVH2_Appl_Layer.c: 1453: }
;BVH2_Appl_Layer.c: 1454: }
;BVH2_Appl_Layer.c: 1455: }
;BVH2_Appl_Layer.c: 1456: }
;BVH2_Appl_Layer.c: 1457: }
;BVH2_Appl_Layer.c: 1458: }
;BVH2_Appl_Layer.c: 1463: bool_HighCurrentAlarm = Cb8_oCurrentAlarm;
	movlb 3	; select bank3
	movf	(BVH2_Appl_Layer@Cb8_oCurrentAlarm)^0180h,w
	movlb 2	; select bank2
	movwf	(_bool_HighCurrentAlarm)^0100h
	line	1466
	
l30220:	
;BVH2_Appl_Layer.c: 1466: bool_PWMin_err_Alarm = Cb27_oPWM_Alarm;
	movf	(_Cb27_oPWM_Alarm)^0100h,w
	movwf	(_bool_PWMin_err_Alarm)^0100h
	line	1467
	
l30222:	
;BVH2_Appl_Layer.c: 1467: bool_MotorStalled = Cb18_oStalledAlarm || Cb1_oCurrentAlarm;
	movlb 1	; select bank1
	clrf	(_BVH2_Appl_Layer$20280)^080h
	incf	(_BVH2_Appl_Layer$20280)^080h,f
	
l30224:	
	movlb 3	; select bank3
	movf	(BVH2_Appl_Layer@Cb18_oStalledAlarm)^0180h,f
	skipz
	goto	u10451
	goto	u10450
u10451:
	goto	l30230
u10450:
	
l30226:	
	movlb 2	; select bank2
	movf	(_Cb1_oCurrentAlarm)^0100h,f
	skipz
	goto	u10461
	goto	u10460
u10461:
	goto	l30230
u10460:
	
l30228:	
	movlb 1	; select bank1
	clrf	(_BVH2_Appl_Layer$20280)^080h
	
l30230:	
	movlb 1	; select bank1
	movf	(_BVH2_Appl_Layer$20280)^080h,w
	movlb 2	; select bank2
	movwf	(_bool_MotorStalled)^0100h
	line	1470
	
l30232:	
;BVH2_Appl_Layer.c: 1473: ui8_debug_out0 = (UInt8) Sb2_Switch5;
	movlb 3	; select bank3
	movf	(BVH2_Appl_Layer@Cb44_oTempRedAlarm)^0180h,w
	movlb 2	; select bank2
	movwf	(_bool_CPU_TempRedAlarm)^0100h
	line	1476
	
l30234:	
;BVH2_Appl_Layer.c: 1476: Sb4_Product2 = (Int16) ((((Int16) ui8_Ki_mat) * Sb2_Error) << 3);
	movlb 4	; select bank4
	movf	(_ui8_Ki_mat+1)^0200h,w
	movlb 0	; select bank0
	movwf	(?___wmul+1)
	movlb 4	; select bank4
	movf	(_ui8_Ki_mat)^0200h,w
	movlb 0	; select bank0
	movwf	(?___wmul)
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@Sb2_Error+1)^080h,w
	movlb 0	; select bank0
	movwf	1+(?___wmul)+02h
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@Sb2_Error)^080h,w
	movlb 0	; select bank0
	movwf	0+(?___wmul)+02h
	fcall	___wmul
	movf	(1+(?___wmul)),w
	movlb 1	; select bank1
	movwf	(BVH2_Appl_Layer@Sb4_Product2+1)^080h
	movlb 0	; select bank0
	movf	(0+(?___wmul)),w
	movlb 1	; select bank1
	movwf	(BVH2_Appl_Layer@Sb4_Product2)^080h
	
l30236:	
	lslf	(BVH2_Appl_Layer@Sb4_Product2)^080h,f
	rlf	(BVH2_Appl_Layer@Sb4_Product2+1)^080h,f
	lslf	(BVH2_Appl_Layer@Sb4_Product2)^080h,f
	rlf	(BVH2_Appl_Layer@Sb4_Product2+1)^080h,f
	lslf	(BVH2_Appl_Layer@Sb4_Product2)^080h,f
	rlf	(BVH2_Appl_Layer@Sb4_Product2+1)^080h,f
	line	1477
	
l30238:	
;BVH2_Appl_Layer.c: 1477: Sb1_Logical_Operator3 = Sb1_Logical_Operator1 || Cb18_oMotorStalled;
	clrf	(_BVH2_Appl_Layer$20281)^080h
	incf	(_BVH2_Appl_Layer$20281)^080h,f
	
l30240:	
	movf	(BVH2_Appl_Layer@Sb1_Logical_Operator1)^080h,f
	skipz
	goto	u10471
	goto	u10470
u10471:
	goto	l30246
u10470:
	
l30242:	
	movlb 3	; select bank3
	movf	(BVH2_Appl_Layer@Cb18_oMotorStalled)^0180h,f
	skipz
	goto	u10481
	goto	u10480
u10481:
	goto	l30246
u10480:
	
l30244:	
	movlb 1	; select bank1
	clrf	(_BVH2_Appl_Layer$20281)^080h
	
l30246:	
	movlb 1	; select bank1
	movf	(_BVH2_Appl_Layer$20281)^080h,w
	movwf	(BVH2_Appl_Layer@Sb1_Logical_Operator3)^080h
	line	1478
	
l30248:	
;BVH2_Appl_Layer.c: 1478: Sb2_Logical_Operator2 = Sb1_Logical_Operator3 || bool_ControlLoopMode;
	clrf	(_BVH2_Appl_Layer$20282)^080h
	incf	(_BVH2_Appl_Layer$20282)^080h,f
	
l30250:	
	movf	(BVH2_Appl_Layer@Sb1_Logical_Operator3)^080h,f
	skipz
	goto	u10491
	goto	u10490
u10491:
	goto	l30256
u10490:
	
l30252:	
	movlb 2	; select bank2
	movf	(_bool_ControlLoopMode)^0100h,f
	skipz
	goto	u10501
	goto	u10500
u10501:
	goto	l30256
u10500:
	
l30254:	
	movlb 1	; select bank1
	clrf	(_BVH2_Appl_Layer$20282)^080h
	
l30256:	
	movlb 1	; select bank1
	movf	(_BVH2_Appl_Layer$20282)^080h,w
	movwf	(BVH2_Appl_Layer@Sb2_Logical_Operator2)^080h
	line	1479
	
l30258:	
;BVH2_Appl_Layer.c: 1479: if ((Sb2_Logical_Operator2 ^ X_Sb4_Intergrator_TriggerIn) && (!(Sb1_BVH2_Appl_Layer_FirstRun)))
	movf	(BVH2_Appl_Layer@Sb2_Logical_Operator2)^080h,w
	movlb 3	; select bank3
	xorwf	(BVH2_Appl_Layer@X_Sb4_Intergrator_TriggerIn)^0180h,w
	btfsc	status,2
	goto	u10511
	goto	u10510
u10511:
	goto	l17714
u10510:
	
l30260:	
	movf	(BVH2_Appl_Layer@Sb1_BVH2_Appl_Layer_FirstRun)^0180h,f
	skipz
	goto	u10521
	goto	u10520
u10521:
	goto	l17714
u10520:
	line	1481
	
l30262:	
;BVH2_Appl_Layer.c: 1480: {
;BVH2_Appl_Layer.c: 1481: X_Sb4_Intergrator = 80000 ;
	movlw	0
	movwf	(BVH2_Appl_Layer@X_Sb4_Intergrator+3)^0180h
	movlw	01h
	movwf	(BVH2_Appl_Layer@X_Sb4_Intergrator+2)^0180h
	movlw	038h
	movwf	(BVH2_Appl_Layer@X_Sb4_Intergrator+1)^0180h
	movlw	080h
	movwf	(BVH2_Appl_Layer@X_Sb4_Intergrator)^0180h

	line	1482
	
l17714:	
	line	1486
;BVH2_Appl_Layer.c: 1482: }
;BVH2_Appl_Layer.c: 1486: if (Sb1_Logical_Operator3) {
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@Sb1_Logical_Operator3)^080h,w
	skipz
	goto	u10530
	goto	l30266
u10530:
	line	1487
	
l30264:	
;BVH2_Appl_Layer.c: 1487: Sb2_Switch2 = 0 ;
	clrf	(BVH2_Appl_Layer@Sb2_Switch2)^080h
	clrf	(BVH2_Appl_Layer@Sb2_Switch2+1)^080h
	line	1488
;BVH2_Appl_Layer.c: 1488: }
	goto	l30302
	line	1492
	
l30266:	
;BVH2_Appl_Layer.c: 1489: else {
;BVH2_Appl_Layer.c: 1492: if (bool_ControlLoopMode) {
	movlb 2	; select bank2
	movf	(_bool_ControlLoopMode)^0100h,w
	skipz
	goto	u10540
	goto	l30288
u10540:
	line	1495
	
l30268:	
;BVH2_Appl_Layer.c: 1495: if (Sb1_Logical_Operator5) {
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@Sb1_Logical_Operator5)^080h,w
	skipz
	goto	u10550
	goto	l30272
u10550:
	line	208
	
l30270:	
;BVH2_Appl_Layer.c: 1496: Sb2_Switch2 = Sb2_Fixed_Power;
	movlw	0CAh
	movwf	(BVH2_Appl_Layer@Sb2_Switch2)^080h
	clrf	(BVH2_Appl_Layer@Sb2_Switch2+1)^080h
	line	1497
;BVH2_Appl_Layer.c: 1497: }
	goto	l30302
	line	1509
	
l30272:	
	line	1516
	
l30274:	
;BVH2_Appl_Layer.c: 1515: Sb3_Sum1 = (Int16) (((UInt16) ((((UInt32) ui8_PWM_dc_mat) * ((UInt32) Sb3_Product1)) >>
;BVH2_Appl_Layer.c: 1516: 8)) + ((Int16) (((Int16) (-((Int16) (UInt16) ((((UInt32) Sb3_Product1) * 95) >> 7)))) + 200)));
	movf	(_ui8_PWM_dc_mat)^080h,w
	movlb 0	; select bank0
	movwf	(?___lmul)
	clrf	(?___lmul+1)
	clrf	(?___lmul+2)
	clrf	(?___lmul+3)

	movlw	0E0h
	movwf	0+(?___lmul)+04h
	clrf	1+(?___lmul)+04h
	clrf	2+(?___lmul)+04h
	clrf	3+(?___lmul)+04h

	fcall	___lmul
	movf	1+(((0+(?___lmul)))+1),w
	movlb 1	; select bank1
	movwf	(BVH2_Appl_Layer@Sb3_Sum1+1)^080h
	movlb 0	; select bank0
	movf	0+(((0+(?___lmul)))+1),w
	movlb 1	; select bank1
	movwf	(BVH2_Appl_Layer@Sb3_Sum1)^080h
	
l30276:	
	movlw	0E0h
	movlb 0	; select bank0
	movwf	(?___lmul)
	clrf	(?___lmul+1)
	clrf	(?___lmul+2)
	clrf	(?___lmul+3)

	movlw	05Fh
	movwf	0+(?___lmul)+04h
	clrf	1+(?___lmul)+04h
	clrf	2+(?___lmul)+04h
	clrf	3+(?___lmul)+04h

	fcall	___lmul
	movf	(0+?___lmul),w
	movwf	(??_BVH2_Appl_Layer+0)+0
	movf	(1+?___lmul),w
	movwf	((??_BVH2_Appl_Layer+0)+0+1)
	movf	(2+?___lmul),w
	movwf	((??_BVH2_Appl_Layer+0)+0+2)
	movf	(3+?___lmul),w
	movwf	((??_BVH2_Appl_Layer+0)+0+3)
	movlw	07h
u10565:
	lsrf	(??_BVH2_Appl_Layer+0)+3,f
	rrf	(??_BVH2_Appl_Layer+0)+2,f
	rrf	(??_BVH2_Appl_Layer+0)+1,f
	rrf	(??_BVH2_Appl_Layer+0)+0,f
u10560:
	decfsz	wreg,f
	goto	u10565
	movf	0+(??_BVH2_Appl_Layer+0)+0,w
	movlb 1	; select bank1
	subwf	(BVH2_Appl_Layer@Sb3_Sum1)^080h,f
	movlb 0	; select bank0
	movf	1+(??_BVH2_Appl_Layer+0)+0,w
	movlb 1	; select bank1
	subwfb	(BVH2_Appl_Layer@Sb3_Sum1+1)^080h,f
	movlw	0C8h
	addwf	(BVH2_Appl_Layer@Sb3_Sum1)^080h,f
	skipnc
	incf	(BVH2_Appl_Layer@Sb3_Sum1+1)^080h,f
	line	1519
	
l30278:	
;BVH2_Appl_Layer.c: 1519: Sb2_Switch2 = (UInt16) ( (Sb3_Sum1 > (Int16)200) ? 200 : ( (Sb3_Sum1 < (Int16)60) ? 60 : (UInt8)Sb3_Sum1 ) );
	movf	(BVH2_Appl_Layer@Sb3_Sum1+1)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0C9h))^80h
	subwf	btemp+1,w
	skipz
	goto	u10575
	movlw	low(0C9h)
	subwf	(BVH2_Appl_Layer@Sb3_Sum1)^080h,w
u10575:

	skipnc
	goto	u10571
	goto	u10570
u10571:
	goto	l30286
u10570:
	
l30280:	
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@Sb3_Sum1+1)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(03Ch))^80h
	subwf	btemp+1,w
	skipz
	goto	u10585
	movlw	low(03Ch)
	subwf	(BVH2_Appl_Layer@Sb3_Sum1)^080h,w
u10585:

	skipc
	goto	u10581
	goto	u10580
u10581:
	goto	l30284
u10580:
	
l30282:	
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@Sb3_Sum1)^080h,w
	movwf	(_BVH2_Appl_Layer$20285)^080h
	clrf	(_BVH2_Appl_Layer$20285+1)^080h
	goto	l17727
	
l30284:	
	movlw	03Ch
	movlb 1	; select bank1
	movwf	(_BVH2_Appl_Layer$20285)^080h
	clrf	(_BVH2_Appl_Layer$20285+1)^080h
	
l17727:	
	movf	(_BVH2_Appl_Layer$20285+1)^080h,w
	movwf	(BVH2_Appl_Layer@Sb2_Switch2+1)^080h
	movf	(_BVH2_Appl_Layer$20285)^080h,w
	movwf	(BVH2_Appl_Layer@Sb2_Switch2)^080h
	goto	l30302
	
l30286:	
	movlw	0C8h
	movlb 1	; select bank1
	movwf	(BVH2_Appl_Layer@Sb2_Switch2)^080h
	clrf	(BVH2_Appl_Layer@Sb2_Switch2+1)^080h
	goto	l30302
	line	1530
	
l30288:	
;BVH2_Appl_Layer.c: 1522: else {
;BVH2_Appl_Layer.c: 1524: Int16 Sb4_PI_sum;
;BVH2_Appl_Layer.c: 1529: Sb4_PI_sum = (Int16) (((UInt16) (Int16) (X_Sb4_Intergrator / ((Int32) 800))) + ((UInt16)
;BVH2_Appl_Layer.c: 1530: (Sb2_Error * ((Int16) ui8_Kp_mat))));
	movlw	0
	movlb 0	; select bank0
	movwf	(?___aldiv+3)
	movlw	0
	movwf	(?___aldiv+2)
	movlw	03h
	movwf	(?___aldiv+1)
	movlw	020h
	movwf	(?___aldiv)

	movlb 3	; select bank3
	movf	(BVH2_Appl_Layer@X_Sb4_Intergrator+3)^0180h,w
	movlb 0	; select bank0
	movwf	3+(?___aldiv)+04h
	movlb 3	; select bank3
	movf	(BVH2_Appl_Layer@X_Sb4_Intergrator+2)^0180h,w
	movlb 0	; select bank0
	movwf	2+(?___aldiv)+04h
	movlb 3	; select bank3
	movf	(BVH2_Appl_Layer@X_Sb4_Intergrator+1)^0180h,w
	movlb 0	; select bank0
	movwf	1+(?___aldiv)+04h
	movlb 3	; select bank3
	movf	(BVH2_Appl_Layer@X_Sb4_Intergrator)^0180h,w
	movlb 0	; select bank0
	movwf	0+(?___aldiv)+04h

	fcall	___aldiv
	movf	1+(((0+(?___aldiv)))),w
	movlb 1	; select bank1
	movwf	(BVH2_Appl_Layer@Sb4_PI_sum+1)^080h
	movlb 0	; select bank0
	movf	0+(((0+(?___aldiv)))),w
	movlb 1	; select bank1
	movwf	(BVH2_Appl_Layer@Sb4_PI_sum)^080h
	movlb 2	; select bank2
	movf	(_ui8_Kp_mat)^0100h,w
	movlb 0	; select bank0
	movwf	(?___wmul)
	clrf	(?___wmul+1)
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@Sb2_Error+1)^080h,w
	movlb 0	; select bank0
	movwf	1+(?___wmul)+02h
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@Sb2_Error)^080h,w
	movlb 0	; select bank0
	movwf	0+(?___wmul)+02h
	fcall	___wmul
	movf	(0+(?___wmul)),w
	movlb 1	; select bank1
	addwf	(BVH2_Appl_Layer@Sb4_PI_sum)^080h,f
	movlb 0	; select bank0
	movf	(1+(?___wmul)),w
	movlb 1	; select bank1
	addwfc	(BVH2_Appl_Layer@Sb4_PI_sum+1)^080h,f
	line	1533
	
l30290:	
;BVH2_Appl_Layer.c: 1533: Sb2_Switch2 = ( ( (Sb4_PI_sum > 0) && ((UInt16)Sb4_PI_sum > 202) ) ? 202 : ( ( (Sb4_PI_sum < 0) || ((UInt16)Sb4_PI_sum < 40) ) ? 40 : (UInt16)Sb4_PI_sum ) );
	movf	(BVH2_Appl_Layer@Sb4_PI_sum+1)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(01h))^80h
	subwf	btemp+1,w
	skipz
	goto	u10595
	movlw	low(01h)
	subwf	(BVH2_Appl_Layer@Sb4_PI_sum)^080h,w
u10595:

	skipc
	goto	u10591
	goto	u10590
u10591:
	goto	l17734
u10590:
	
l30292:	
	movlw	high(0CBh)
	movlb 1	; select bank1
	subwf	(BVH2_Appl_Layer@Sb4_PI_sum+1)^080h,w
	movlw	low(0CBh)
	skipnz
	subwf	(BVH2_Appl_Layer@Sb4_PI_sum)^080h,w
	skipnc
	goto	u10601
	goto	u10600
u10601:
	goto	l30270
u10600:
	
l17734:	
	movlb 1	; select bank1
	btfsc	(BVH2_Appl_Layer@Sb4_PI_sum+1)^080h,7
	goto	u10611
	goto	u10610
u10611:
	goto	l30298
u10610:
	
l30294:	
	movlw	high(028h)
	subwf	(BVH2_Appl_Layer@Sb4_PI_sum+1)^080h,w
	movlw	low(028h)
	skipnz
	subwf	(BVH2_Appl_Layer@Sb4_PI_sum)^080h,w
	skipc
	goto	u10621
	goto	u10620
u10621:
	goto	l30298
u10620:
	
l30296:	
	movf	(BVH2_Appl_Layer@Sb4_PI_sum+1)^080h,w
	movwf	(_BVH2_Appl_Layer$20287+1)^080h
	movf	(BVH2_Appl_Layer@Sb4_PI_sum)^080h,w
	movwf	(_BVH2_Appl_Layer$20287)^080h
	goto	l17738
	
l30298:	
	movlw	028h
	movwf	(_BVH2_Appl_Layer$20287)^080h
	clrf	(_BVH2_Appl_Layer$20287+1)^080h
	
l17738:	
	movf	(_BVH2_Appl_Layer$20287+1)^080h,w
	movwf	(BVH2_Appl_Layer@Sb2_Switch2+1)^080h
	movf	(_BVH2_Appl_Layer$20287)^080h,w
	movwf	(BVH2_Appl_Layer@Sb2_Switch2)^080h
	line	1539
	
l30302:	
;BVH2_Appl_Layer.c: 1542: bl_Pumpoff_Alarm = Sb1_Logical_Operator2;
	movf	(BVH2_Appl_Layer@Sb2_Switch2+1)^080h,w
	movlb 0	; select bank0
	movwf	(??_BVH2_Appl_Layer+0)+0+1
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@Sb2_Switch2)^080h,w
	movlb 0	; select bank0
	movwf	(??_BVH2_Appl_Layer+0)+0
	lsrf	(??_BVH2_Appl_Layer+0)+1,f
	rrf	(??_BVH2_Appl_Layer+0)+0,f
	movf	0+(??_BVH2_Appl_Layer+0)+0,w
	movlb 2	; select bank2
	movwf	(_ui8_duty_cycle_mat)^0100h
	line	1547
	
l30304:	
;BVH2_Appl_Layer.c: 1551: if (((UInt16)0) < 72) {
	btfss	(BVH2_Appl_Layer@SIBFS_Temperature_Alarmo_old_b)^0100h,1
	goto	u10631
	goto	u10630
u10631:
	goto	l17739
u10630:
	line	1554
	
l30306:	
;BVH2_Appl_Layer.c: 1554: SIBFS_Temperature_Alarmo_old_b.Cb50_greenTemp = 0;
	bcf	(BVH2_Appl_Layer@SIBFS_Temperature_Alarmo_old_b)^0100h,1
	line	1555
;BVH2_Appl_Layer.c: 1555: SIBFS_Temperature_Alarmo_old_b.Cb52_CntOverTemp = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Temperature_Alarmo_old_b)^0100h,3
	goto	l30318
	line	1560
	
l17739:	
	line	1562
;BVH2_Appl_Layer.c: 1560: else {
;BVH2_Appl_Layer.c: 1562: if (SIBFS_Temperature_Alarmo_old_b.Cb51_redTemp) {
	btfss	(BVH2_Appl_Layer@SIBFS_Temperature_Alarmo_old_b)^0100h,2
	goto	u10641
	goto	u10640
u10641:
	goto	l17742
u10640:
	goto	l30318
	line	1575
	
l17742:	
	line	1577
;BVH2_Appl_Layer.c: 1575: else {
;BVH2_Appl_Layer.c: 1577: if (SIBFS_Temperature_Alarmo_old_b.Cb52_CntOverTemp) {
	btfss	(BVH2_Appl_Layer@SIBFS_Temperature_Alarmo_old_b)^0100h,3
	goto	u10651
	goto	u10650
u10651:
	goto	l17745
u10650:
	line	1586
;BVH2_Appl_Layer.c: 1586: else {
;BVH2_Appl_Layer.c: 1588: if (((UInt16)0) < 72) {
	
l17746:	
	line	1590
;BVH2_Appl_Layer.c: 1590: SIBFS_Temperature_Alarmo_old_b.Cb52_CntOverTemp = 0;
	bcf	(BVH2_Appl_Layer@SIBFS_Temperature_Alarmo_old_b)^0100h,3
	line	1591
;BVH2_Appl_Layer.c: 1591: SIBFS_Temperature_Alarmo_old_b.Cb51_redTemp = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Temperature_Alarmo_old_b)^0100h,2
	goto	l30318
	line	1597
	
l17745:	
	line	1599
;BVH2_Appl_Layer.c: 1597: else {
;BVH2_Appl_Layer.c: 1599: if (!(SIBFS_Temperature_Alarmo_old_b.Cb49_Temperature_Alarmo_old)) {
	btfsc	(BVH2_Appl_Layer@SIBFS_Temperature_Alarmo_old_b)^0100h,0
	goto	u10661
	goto	u10660
u10661:
	goto	l30318
u10660:
	line	1600
	
l30316:	
;BVH2_Appl_Layer.c: 1600: SIBFS_Temperature_Alarmo_old_b.Cb49_Temperature_Alarmo_old = 1;
	bsf	(BVH2_Appl_Layer@SIBFS_Temperature_Alarmo_old_b)^0100h,0
	line	1607
	
l30318:	
;BVH2_Appl_Layer.c: 1601: }
;BVH2_Appl_Layer.c: 1602: }
;BVH2_Appl_Layer.c: 1603: }
;BVH2_Appl_Layer.c: 1604: }
;BVH2_Appl_Layer.c: 1607: X_Sb4_Intergrator_TriggerIn = Sb2_Logical_Operator2;
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@Sb2_Logical_Operator2)^080h,w
	movlb 3	; select bank3
	movwf	(BVH2_Appl_Layer@X_Sb4_Intergrator_TriggerIn)^0180h
	line	1610
;BVH2_Appl_Layer.c: 1610: X_Sb4_Intergrator = X_Sb4_Intergrator + ((Int32) Sb4_Product2);
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@Sb4_Product2)^080h,w
	movlb 0	; select bank0
	movwf	((??_BVH2_Appl_Layer+0)+0)
	movlb 1	; select bank1
	movf	(BVH2_Appl_Layer@Sb4_Product2+1)^080h,w
	movlb 0	; select bank0
	movwf	((??_BVH2_Appl_Layer+0)+0+1)
	movlw	0
	btfsc	((??_BVH2_Appl_Layer+0)+0+1),7
	movlw	255
	movwf	((??_BVH2_Appl_Layer+0)+0+2)
	movwf	((??_BVH2_Appl_Layer+0)+0+3)
	movf	0+(??_BVH2_Appl_Layer+0)+0,w
	movlb 3	; select bank3
	addwf	(BVH2_Appl_Layer@X_Sb4_Intergrator)^0180h,f
	movlb 0	; select bank0
	movf	1+(??_BVH2_Appl_Layer+0)+0,w
	movlb 3	; select bank3
	addwfc	(BVH2_Appl_Layer@X_Sb4_Intergrator+1)^0180h,f
	movlb 0	; select bank0
	movf	2+(??_BVH2_Appl_Layer+0)+0,w
	movlb 3	; select bank3
	addwfc	(BVH2_Appl_Layer@X_Sb4_Intergrator+2)^0180h,f
	movlb 0	; select bank0
	movf	3+(??_BVH2_Appl_Layer+0)+0,w
	movlb 3	; select bank3
	addwfc	(BVH2_Appl_Layer@X_Sb4_Intergrator+3)^0180h,f
	line	1613
	
l30320:	
;BVH2_Appl_Layer.c: 1613: X_Sb4_Intergrator = ( (X_Sb4_Intergrator > 161600) ? 161600 : ( (X_Sb4_Intergrator < 32000) ? 32000 : X_Sb4_Intergrator ) );
	movf	(BVH2_Appl_Layer@X_Sb4_Intergrator+3)^0180h,w
	xorlw	80h
	movwf	btemp+1
	movlw	0
	xorlw	80h
	subwf	btemp+1,w
	
	skipz
	goto	u10673
	movlw	02h
	subwf	(BVH2_Appl_Layer@X_Sb4_Intergrator+2)^0180h,w
	skipz
	goto	u10673
	movlw	077h
	subwf	(BVH2_Appl_Layer@X_Sb4_Intergrator+1)^0180h,w
	skipz
	goto	u10673
	movlw	041h
	subwf	(BVH2_Appl_Layer@X_Sb4_Intergrator)^0180h,w
u10673:
	skipnc
	goto	u10671
	goto	u10670
u10671:
	goto	l30328
u10670:
	
l30322:	
	movf	(BVH2_Appl_Layer@X_Sb4_Intergrator+3)^0180h,w
	xorlw	80h
	movwf	btemp+1
	movlw	0
	xorlw	80h
	subwf	btemp+1,w
	
	skipz
	goto	u10683
	movlw	0
	subwf	(BVH2_Appl_Layer@X_Sb4_Intergrator+2)^0180h,w
	skipz
	goto	u10683
	movlw	07Dh
	subwf	(BVH2_Appl_Layer@X_Sb4_Intergrator+1)^0180h,w
	skipz
	goto	u10683
	movlw	0
	subwf	(BVH2_Appl_Layer@X_Sb4_Intergrator)^0180h,w
u10683:
	skipc
	goto	u10681
	goto	u10680
u10681:
	goto	l30326
u10680:
	
l30324:	
	movf	(BVH2_Appl_Layer@X_Sb4_Intergrator+3)^0180h,w
	movlb 1	; select bank1
	movwf	(_BVH2_Appl_Layer$20288+3)^080h
	movlb 3	; select bank3
	movf	(BVH2_Appl_Layer@X_Sb4_Intergrator+2)^0180h,w
	movlb 1	; select bank1
	movwf	(_BVH2_Appl_Layer$20288+2)^080h
	movlb 3	; select bank3
	movf	(BVH2_Appl_Layer@X_Sb4_Intergrator+1)^0180h,w
	movlb 1	; select bank1
	movwf	(_BVH2_Appl_Layer$20288+1)^080h
	movlb 3	; select bank3
	movf	(BVH2_Appl_Layer@X_Sb4_Intergrator)^0180h,w
	movlb 1	; select bank1
	movwf	(_BVH2_Appl_Layer$20288)^080h

	goto	l17758
	
l30326:	
	movlw	0
	movlb 1	; select bank1
	movwf	(_BVH2_Appl_Layer$20288+3)^080h
	movlw	0
	movwf	(_BVH2_Appl_Layer$20288+2)^080h
	movlw	07Dh
	movwf	(_BVH2_Appl_Layer$20288+1)^080h
	movlw	0
	movwf	(_BVH2_Appl_Layer$20288)^080h

	
l17758:	
	movf	(_BVH2_Appl_Layer$20288+3)^080h,w
	movlb 3	; select bank3
	movwf	(BVH2_Appl_Layer@X_Sb4_Intergrator+3)^0180h
	movlb 1	; select bank1
	movf	(_BVH2_Appl_Layer$20288+2)^080h,w
	movlb 3	; select bank3
	movwf	(BVH2_Appl_Layer@X_Sb4_Intergrator+2)^0180h
	movlb 1	; select bank1
	movf	(_BVH2_Appl_Layer$20288+1)^080h,w
	movlb 3	; select bank3
	movwf	(BVH2_Appl_Layer@X_Sb4_Intergrator+1)^0180h
	movlb 1	; select bank1
	movf	(_BVH2_Appl_Layer$20288)^080h,w
	movlb 3	; select bank3
	movwf	(BVH2_Appl_Layer@X_Sb4_Intergrator)^0180h

	goto	l30330
	
l30328:	
	movlw	0
	movwf	(BVH2_Appl_Layer@X_Sb4_Intergrator+3)^0180h
	movlw	02h
	movwf	(BVH2_Appl_Layer@X_Sb4_Intergrator+2)^0180h
	movlw	077h
	movwf	(BVH2_Appl_Layer@X_Sb4_Intergrator+1)^0180h
	movlw	040h
	movwf	(BVH2_Appl_Layer@X_Sb4_Intergrator)^0180h

	line	1616
	
l30330:	
;BVH2_Appl_Layer.c: 1616: Sb1_BVH2_Appl_Layer_FirstRun = 0;
	clrf	(BVH2_Appl_Layer@Sb1_BVH2_Appl_Layer_FirstRun)^0180h
	line	1617
	
l17759:	
	return
	opt stack 0
GLOBAL	__end_of_BVH2_Appl_Layer
	__end_of_BVH2_Appl_Layer:
;; =============== function _BVH2_Appl_Layer ends ============

	signat	_BVH2_Appl_Layer,88
	global	_I_calibrationInit
psect	text2025,local,class=CODE,delta=2
global __ptext2025
__ptext2025:

;; *************** function _I_calibrationInit *****************
;; Defined at:
;;		line 166 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 1F/4
;;		On exit  : 1F/3
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       1       0       0       0       0       0       0
;;      Totals:         0       1       0       0       0       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_read_eeprom_data
;;		_ADC_Wait
;;		_ADC_Read
;; This function is called by:
;;		_system_init
;; This function uses a non-reentrant model
;;
psect	text2025
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\main.c"
	line	166
	global	__size_of_I_calibrationInit
	__size_of_I_calibrationInit	equ	__end_of_I_calibrationInit-_I_calibrationInit
	
_I_calibrationInit:	
	opt	stack 9
; Regs used in _I_calibrationInit: [wreg+status,2+status,0+pclath+cstack]
	line	168
	
l29588:	
;main.c: 168: if ( read_eeprom_data( 0x03 ) != 0x00 && read_eeprom_data( 0x03 ) != 0xFF)
	movlw	(03h)
	fcall	_read_eeprom_data
	xorlw	0&0ffh
	skipnz
	goto	u9421
	goto	u9420
u9421:
	goto	l29594
u9420:
	
l29590:	
	movlw	(03h)
	fcall	_read_eeprom_data
	xorlw	0FFh&0ffh
	skipnz
	goto	u9431
	goto	u9430
u9431:
	goto	l29594
u9430:
	line	172
	
l29592:	
;main.c: 170: {
;main.c: 172: ui16_I_cal_Ph1 = ( read_eeprom_data( 0x02 )<<8 ) | read_eeprom_data( 0x03 );
	movlw	(03h)
	fcall	_read_eeprom_data
	movwf	(??_I_calibrationInit+0)+0
	movlw	(02h)
	fcall	_read_eeprom_data
	movlb 2	; select bank2
	movwf	(_ui16_I_cal_Ph1+1)^0100h
	movlb 0	; select bank0
	movf	0+(??_I_calibrationInit+0)+0,w
	movlb 2	; select bank2
	movwf	(_ui16_I_cal_Ph1)^0100h
	line	175
;main.c: 175: }
	goto	l29596
	line	181
	
l29594:	
;main.c: 178: else
;main.c: 179: {
;main.c: 181: ui16_I_cal_Ph1 = 0x0100;
	movlw	low(0100h)
	movlb 2	; select bank2
	movwf	(_ui16_I_cal_Ph1)^0100h
	movlw	high(0100h)
	movwf	((_ui16_I_cal_Ph1)^0100h)+1
	line	187
	
l29596:	
;main.c: 184: }
;main.c: 187: if ( read_eeprom_data( 0x05 ) != 0x00 && read_eeprom_data( 0x05 ) != 0xFF)
	movlw	(05h)
	fcall	_read_eeprom_data
	xorlw	0&0ffh
	skipnz
	goto	u9441
	goto	u9440
u9441:
	goto	l29602
u9440:
	
l29598:	
	movlw	(05h)
	fcall	_read_eeprom_data
	xorlw	0FFh&0ffh
	skipnz
	goto	u9451
	goto	u9450
u9451:
	goto	l29602
u9450:
	line	191
	
l29600:	
;main.c: 189: {
;main.c: 191: ui16_I_cal_Ph2 = ( read_eeprom_data( 0x04 )<<8 ) | read_eeprom_data( 0x05 );
	movlw	(05h)
	fcall	_read_eeprom_data
	movwf	(??_I_calibrationInit+0)+0
	movlw	(04h)
	fcall	_read_eeprom_data
	movlb 2	; select bank2
	movwf	(_ui16_I_cal_Ph2+1)^0100h
	movlb 0	; select bank0
	movf	0+(??_I_calibrationInit+0)+0,w
	movlb 2	; select bank2
	movwf	(_ui16_I_cal_Ph2)^0100h
	line	194
;main.c: 194: }
	goto	l29604
	line	200
	
l29602:	
;main.c: 197: else
;main.c: 198: {
;main.c: 200: ui16_I_cal_Ph2 = 0x0100;
	movlw	low(0100h)
	movlb 2	; select bank2
	movwf	(_ui16_I_cal_Ph2)^0100h
	movlw	high(0100h)
	movwf	((_ui16_I_cal_Ph2)^0100h)+1
	line	206
	
l29604:	
;main.c: 203: }
;main.c: 206: if ( read_eeprom_data( 0x07 ) != 0x00 && read_eeprom_data( 0x07 ) != 0xFF)
	movlw	(07h)
	fcall	_read_eeprom_data
	xorlw	0&0ffh
	skipnz
	goto	u9461
	goto	u9460
u9461:
	goto	l29610
u9460:
	
l29606:	
	movlw	(07h)
	fcall	_read_eeprom_data
	xorlw	0FFh&0ffh
	skipnz
	goto	u9471
	goto	u9470
u9471:
	goto	l29610
u9470:
	line	210
	
l29608:	
;main.c: 208: {
;main.c: 210: ui16_I_cal_Ph3 = ( read_eeprom_data( 0x06 )<<8 ) | read_eeprom_data( 0x07 );
	movlw	(07h)
	fcall	_read_eeprom_data
	movwf	(??_I_calibrationInit+0)+0
	movlw	(06h)
	fcall	_read_eeprom_data
	movlb 2	; select bank2
	movwf	(_ui16_I_cal_Ph3+1)^0100h
	movlb 0	; select bank0
	movf	0+(??_I_calibrationInit+0)+0,w
	movlb 2	; select bank2
	movwf	(_ui16_I_cal_Ph3)^0100h
	line	212
;main.c: 212: }
	goto	l29612
	line	218
	
l29610:	
;main.c: 215: else
;main.c: 216: {
;main.c: 218: ui16_I_cal_Ph3 = 0x0100;
	movlw	low(0100h)
	movlb 2	; select bank2
	movwf	(_ui16_I_cal_Ph3)^0100h
	movlw	high(0100h)
	movwf	((_ui16_I_cal_Ph3)^0100h)+1
	line	314
	
l29612:	
;main.c: 221: }
;main.c: 314: LATC2 = 0;
	bcf	(2162/8)^0100h,(2162)&7
	line	315
	
l29614:	
;main.c: 315: LATB2 = 0;
	bcf	(2154/8)^0100h,(2154)&7
	line	316
	
l29616:	
;main.c: 316: LATB1 = 0;
	bcf	(2153/8)^0100h,(2153)&7
	line	317
	
l29618:	
;main.c: 317: LATC = LATC | 0b00011010;
	movlw	(01Ah)
	iorwf	(270)^0100h,f	;volatile
	line	319
	
l29620:	
;main.c: 319: ADC_Wait();
	fcall	_ADC_Wait
	line	320
	
l29622:	
;main.c: 320: ADC_Wait();
	fcall	_ADC_Wait
	line	322
	
l29624:	
;main.c: 322: ( ADCON0 = ( 0x04 << 2 ) | 0x01 );
	movlw	(011h)
	movlb 1	; select bank1
	movwf	(157)^080h	;volatile
	line	323
	
l29626:	
;main.c: 323: ADC_Wait();
	fcall	_ADC_Wait
	line	324
	
l29628:	
;main.c: 324: ( GO_nDONE = 1 );
	movlb 1	; select bank1
	bsf	(1257/8)^080h,(1257)&7
	line	327
	
l29630:	
;main.c: 327: ui8_current_cal[2] = ADC_Read();
	fcall	_ADC_Read
	movf	(0+(?_ADC_Read)),w
	movlb 3	; select bank3
	movwf	0+(_ui8_current_cal)^0180h+02h
	line	329
	
l29632:	
;main.c: 329: ( ADCON0 = ( 0x0D << 2 ) | 0x01 );
	movlw	(035h)
	movlb 1	; select bank1
	movwf	(157)^080h	;volatile
	line	330
	
l29634:	
;main.c: 330: ADC_Wait();
	fcall	_ADC_Wait
	line	331
	
l29636:	
;main.c: 331: ( GO_nDONE = 1 );
	movlb 1	; select bank1
	bsf	(1257/8)^080h,(1257)&7
	line	334
	
l29638:	
;main.c: 334: ui8_current_cal[1] = ADC_Read();
	fcall	_ADC_Read
	movf	(0+(?_ADC_Read)),w
	movlb 3	; select bank3
	movwf	0+(_ui8_current_cal)^0180h+01h
	line	336
	
l29640:	
;main.c: 336: ( ADCON0 = ( 0x02 << 2 ) | 0x01 );
	movlw	(09h)
	movlb 1	; select bank1
	movwf	(157)^080h	;volatile
	line	337
	
l29642:	
;main.c: 337: ADC_Wait();
	fcall	_ADC_Wait
	line	338
	
l29644:	
;main.c: 338: ( GO_nDONE = 1 );
	movlb 1	; select bank1
	bsf	(1257/8)^080h,(1257)&7
	line	341
	
l29646:	
;main.c: 341: ui8_current_cal[0] = ADC_Read();
	fcall	_ADC_Read
	movf	(0+(?_ADC_Read)),w
	movlb 3	; select bank3
	movwf	(_ui8_current_cal)^0180h
	line	347
	
l15452:	
	return
	opt stack 0
GLOBAL	__end_of_I_calibrationInit
	__end_of_I_calibrationInit:
;; =============== function _I_calibrationInit ends ============

	signat	_I_calibrationInit,88
	global	__ELINMIntReceiveMessage
psect	text2026,local,class=CODE,delta=2
global __ptext2026
__ptext2026:

;; *************** function __ELINMIntReceiveMessage *****************
;; Defined at:
;;		line 1326 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
;; Parameters:    Size  Location     Type
;;  _ELINM_tag      1    wreg     unsigned char 
;;  _ELINM_id       1   23[BANK0 ] unsigned char 
;;  _ELINM_size     1   24[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  _ELINM_tag      1   25[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr1l, fsr1h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       2       0       0       0       0       0       0
;;      Locals:         0       1       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       3       0       0       0       0       0       0
;;Total ram usage:        3 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    6
;; This function calls:
;;		__ELINMIntSendMessage
;; This function is called by:
;;		_Receive_Diag
;; This function uses a non-reentrant model
;;
psect	text2026
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
	line	1326
	global	__size_of__ELINMIntReceiveMessage
	__size_of__ELINMIntReceiveMessage	equ	__end_of__ELINMIntReceiveMessage-__ELINMIntReceiveMessage
	
__ELINMIntReceiveMessage:	
	opt	stack 8
; Regs used in __ELINMIntReceiveMessage: [wreg+fsr1l-status,0+pclath+cstack]
	line	1328
	
l29578:	
;lin.c: 1331: {
;lin.c: 1333: _ELINMIntMessageTag = _ELINM_tag;
	btfss	(__ELINMIntStatus),3
	goto	u9411
	goto	u9410
u9411:
	goto	l13486
u9410:
	line	1334
	
l29580:	
;lin.c: 1334: _ELINMIntRXMessageSize.SIZE = _ELINM_size;
	movf	(__ELINMIntReceiveMessage@_ELINM_size),w
	movlb 2	; select bank2
	movwf	(__ELINMIntRXMessageSize)^0100h
	line	1335
	
l29582:	
;lin.c: 1335: _ELINMIntStatus.ELINMINTSTS.RX = 1;
	movlb 0	; select bank0
	bsf	(__ELINMIntStatus),1
	line	1336
	
l29584:	
;lin.c: 1336: _ELINMIntRXCRC.CRC = 0;
	movlb 4	; select bank4
	clrf	(__ELINMIntRXCRC)^0200h
	clrf	(__ELINMIntRXCRC+1)^0200h
	line	1337
	
l29586:	
;lin.c: 1337: _ELINMIntSendMessage ( _ELINM_id, 0, ( ( ( ( ( 0 + 3 ) * 15L ) + 44L ) * ( 100L * 1000000L / 19200L ) / 128L ) / 100L ), ( ( ( ( ( ( ( ( 0 + 3 ) * 15L ) + 44L ) + 1L ) * 14L ) / 10L ) * ( 100L * 1000000L / 19200L ) / 128L ) / 100L ) );
	movlb 0	; select bank0
	clrf	(?__ELINMIntSendMessage)
	movlw	024h
	movwf	0+(?__ELINMIntSendMessage)+01h
	clrf	1+(?__ELINMIntSendMessage)+01h
	movlw	033h
	movwf	0+(?__ELINMIntSendMessage)+03h
	clrf	1+(?__ELINMIntSendMessage)+03h
	movf	(__ELINMIntReceiveMessage@_ELINM_id),w
	fcall	__ELINMIntSendMessage
	line	1343
	
l13486:	
	return
	opt stack 0
GLOBAL	__end_of__ELINMIntReceiveMessage
	__end_of__ELINMIntReceiveMessage:
;; =============== function __ELINMIntReceiveMessage ends ============

	signat	__ELINMIntReceiveMessage,12408
	global	_Transmit_LIN_8Bytes
psect	text2027,local,class=CODE,delta=2
global __ptext2027
__ptext2027:

;; *************** function _Transmit_LIN_8Bytes *****************
;; Defined at:
;;		line 171 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
;; Parameters:    Size  Location     Type
;;  ID              1    wreg     unsigned char 
;;  B0              1   23[BANK0 ] unsigned char 
;;  B1              1   24[BANK0 ] unsigned char 
;;  B2              1   25[BANK0 ] unsigned char 
;;  B3              1   26[BANK0 ] unsigned char 
;;  B4              1   27[BANK0 ] unsigned char 
;;  B5              1   28[BANK0 ] unsigned char 
;;  B6              1   29[BANK0 ] unsigned char 
;;  B7              1   30[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  ID              1   31[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr1l, fsr1h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/2
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       8       0       0       0       0       0       0
;;      Locals:         0       1       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       9       0       0       0       0       0       0
;;Total ram usage:        9 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    6
;; This function calls:
;;		__ELINMIntGetPointer
;;		__ELINMIntSendMessage
;; This function is called by:
;;		_EOL
;;		_NegativeAnswer
;; This function uses a non-reentrant model
;;
psect	text2027
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
	line	171
	global	__size_of_Transmit_LIN_8Bytes
	__size_of_Transmit_LIN_8Bytes	equ	__end_of_Transmit_LIN_8Bytes-_Transmit_LIN_8Bytes
	
_Transmit_LIN_8Bytes:	
	opt	stack 7
; Regs used in _Transmit_LIN_8Bytes: [wreg+fsr1l-status,0+pclath+cstack]
;Transmit_LIN_8Bytes@ID stored from wreg
	movwf	(Transmit_LIN_8Bytes@ID)
	line	178
	
l29536:	
	line	181
;lin.c: 179: {
;lin.c: 180: ;
	
l13349:	
	line	178
	btfss	(__ELINMIntStatus),3
	goto	u9401
	goto	u9400
u9401:
	goto	l13349
u9400:
	line	182
	
l29538:	
;lin.c: 181: }
;lin.c: 182: pt = _ELINMIntGetPointer ( 5, 2 );
	movlw	(02h)
	movwf	(?__ELINMIntGetPointer)
	movlw	(05h)
	fcall	__ELINMIntGetPointer
	movwf	(_pt)
	line	183
	
l29540:	
;lin.c: 183: *pt++ = B0;
	movf	(_pt),w
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	(Transmit_LIN_8Bytes@B0),w
	movwf	indf1
	
l29542:	
	incf	(_pt),f
	line	184
	
l29544:	
;lin.c: 184: *pt++ = B1;
	movf	(_pt),w
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	(Transmit_LIN_8Bytes@B1),w
	movwf	indf1
	
l29546:	
	incf	(_pt),f
	line	185
	
l29548:	
;lin.c: 185: *pt++ = B2;
	movf	(_pt),w
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	(Transmit_LIN_8Bytes@B2),w
	movwf	indf1
	
l29550:	
	incf	(_pt),f
	line	186
	
l29552:	
;lin.c: 186: *pt++ = B3;
	movf	(_pt),w
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	(Transmit_LIN_8Bytes@B3),w
	movwf	indf1
	
l29554:	
	incf	(_pt),f
	line	187
	
l29556:	
;lin.c: 187: *pt++ = B4;
	movf	(_pt),w
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	(Transmit_LIN_8Bytes@B4),w
	movwf	indf1
	
l29558:	
	incf	(_pt),f
	line	188
	
l29560:	
;lin.c: 188: *pt++ = B5;
	movf	(_pt),w
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	(Transmit_LIN_8Bytes@B5),w
	movwf	indf1
	
l29562:	
	incf	(_pt),f
	line	189
	
l29564:	
;lin.c: 189: *pt++ = B6;
	movf	(_pt),w
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	(Transmit_LIN_8Bytes@B6),w
	movwf	indf1
	
l29566:	
	incf	(_pt),f
	line	190
	
l29568:	
;lin.c: 190: *pt++ = B7;
	movf	(_pt),w
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	(Transmit_LIN_8Bytes@B7),w
	movwf	indf1
	
l29570:	
	incf	(_pt),f
	line	192
	
l29572:	
;lin.c: 192: _ELINMIntSendMessage ( ID, 8, ( ( ( ( ( 8 + 3 ) * 15L ) + 44L ) * ( 100L * 1000000L / 19200L ) / 128L ) / 100L ), ( ( ( ( ( ( ( ( 8 + 3 ) * 15L ) + 44L ) + 1L ) * 14L ) / 10L ) * ( 100L * 1000000L / 19200L ) / 128L ) / 100L ) );
	movlw	(08h)
	movwf	(?__ELINMIntSendMessage)
	movlw	055h
	movwf	0+(?__ELINMIntSendMessage)+01h
	clrf	1+(?__ELINMIntSendMessage)+01h
	movlw	077h
	movwf	0+(?__ELINMIntSendMessage)+03h
	clrf	1+(?__ELINMIntSendMessage)+03h
	movf	(Transmit_LIN_8Bytes@ID),w
	fcall	__ELINMIntSendMessage
	line	193
	
l29574:	
;lin.c: 193: if( ( ErrorCode = ( _ELINMIntStatus.ELINMIntStatusByte & 0xF0 ) ))
	movf	(__ELINMIntStatus),w
	movlb 2	; select bank2
	movwf	(_ErrorCode)^0100h
	
l29576:	
	movlw	(0F0h)
	andwf	(_ErrorCode)^0100h,f
	line	200
	
l13353:	
	return
	opt stack 0
GLOBAL	__end_of_Transmit_LIN_8Bytes
	__end_of_Transmit_LIN_8Bytes:
;; =============== function _Transmit_LIN_8Bytes ends ============

	signat	_Transmit_LIN_8Bytes,36984
	global	_InitMotorRun
psect	text2028,local,class=CODE,delta=2
global __ptext2028
__ptext2028:

;; *************** function _InitMotorRun *****************
;; Defined at:
;;		line 2108 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\bldc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, fsr1l, fsr1h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 1D/2
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    6
;; This function calls:
;;		_commutate
;; This function is called by:
;;		_init_bldc
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2028
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\bldc.c"
	line	2108
	global	__size_of_InitMotorRun
	__size_of_InitMotorRun	equ	__end_of_InitMotorRun-_InitMotorRun
	
_InitMotorRun:	
	opt	stack 9
; Regs used in _InitMotorRun: [wreg-status,0+pclath+cstack]
	line	2110
	
l29504:	
;bldc.c: 2110: if( 0 == MotorFlags.bits.B6 )
	movlb 0	; select bank0
	btfsc	(_MotorFlags),6
	goto	u9391
	goto	u9390
u9391:
	goto	l3906
u9390:
	line	2115
	
l29506:	
;bldc.c: 2113: {
;bldc.c: 2115: MotorFlags.bits.B5 = 0;
	bcf	(_MotorFlags),5
	line	2116
	
l29508:	
;bldc.c: 2116: ui16_comm_time_max = ( unsigned short ) ( ( 10 * 20000UL ) / ( 600 * 2 ) );
	movlw	0A6h
	movlb 2	; select bank2
	movwf	(_ui16_comm_time_max)^0100h
	clrf	(_ui16_comm_time_max+1)^0100h
	line	2117
	
l29510:	
;bldc.c: 2117: comm_state = 1;
	movlb 0	; select bank0
	clrf	(_comm_state)
	incf	(_comm_state),f
	line	2118
	
l29512:	
;bldc.c: 2118: ui16_step_cnt = 0;
	movlb 1	; select bank1
	clrf	(_ui16_step_cnt)^080h
	clrf	(_ui16_step_cnt+1)^080h
	line	2119
	
l29514:	
;bldc.c: 2119: ui8_duty_cycle_BLDC = 0;
	movlb 0	; select bank0
	clrf	(_ui8_duty_cycle_BLDC)
	line	2120
	
l29516:	
;bldc.c: 2121: bemf_filter = 0;
	clrf	(_comm_time)
	clrf	(_comm_time+1)
	line	2122
	
l29518:	
;bldc.c: 2123: phase_delay_counter_debug = 0;
	movlb 4	; select bank4
	clrf	(_phase_delay_counter)^0200h
	clrf	(_phase_delay_counter+1)^0200h
	line	2124
	
l29520:	
;bldc.c: 2124: PSTR1CON = 0;
	movlb 5	; select bank5
	clrf	(662)^0280h	;volatile
	line	2127
	
l29522:	
;bldc.c: 2127: MotorFlags.bits.B3 = 1 ;
	movlb 0	; select bank0
	bsf	(_MotorFlags),3
	line	2128
	
l29524:	
;bldc.c: 2129: ui16_phase_angle = 128;
	bsf	(_MotorFlags),6
	line	2130
	
l29526:	
;bldc.c: 2130: ui8_StartupPWM = 0 ;
	movlb 2	; select bank2
	clrf	(_ui8_StartupPWM)^0100h
	line	2131
	
l29528:	
;bldc.c: 2131: CCPR1L = 0 ;
	movlb 5	; select bank5
	clrf	(657)^0280h	;volatile
	line	2132
	
l29530:	
;bldc.c: 2133: ui8_zero_cros_cnt = 0 ;
	movlb 2	; select bank2
	clrf	(_ui8_sampleState)^0100h
	line	2134
	
l29532:	
;bldc.c: 2134: MotorFlags.bits.B4 = 1 ;
	movlb 0	; select bank0
	bsf	(_MotorFlags),4
	line	2142
	
l29534:	
;bldc.c: 2142: commutate( );
	fcall	_commutate
	line	2147
	
l3906:	
	return
	opt stack 0
GLOBAL	__end_of_InitMotorRun
	__end_of_InitMotorRun:
;; =============== function _InitMotorRun ends ============

	signat	_InitMotorRun,88
	global	_FILTER_Init
psect	text2029,local,class=CODE,delta=2
global __ptext2029
__ptext2029:

;; *************** function _FILTER_Init *****************
;; Defined at:
;;		line 228 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr1l, fsr1h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 1F/3
;;		On exit  : 1F/1
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_ADC_Wait
;;		_ADC_Read
;; This function is called by:
;;		_system_init
;; This function uses a non-reentrant model
;;
psect	text2029
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
	line	228
	global	__size_of_FILTER_Init
	__size_of_FILTER_Init	equ	__end_of_FILTER_Init-_FILTER_Init
	
_FILTER_Init:	
	opt	stack 9
; Regs used in _FILTER_Init: [wreg+fsr1l-status,0+pclath+cstack]
	line	230
	
l29488:	
;adc.c: 230: for( windowPtr1 = 0; windowPtr1 < 8; windowPtr1++ )
	movlb 1	; select bank1
	clrf	(_windowPtr1)^080h
	
l29490:	
	movlw	(08h)
	subwf	(_windowPtr1)^080h,w
	skipc
	goto	u9381
	goto	u9380
u9381:
	goto	l29494
u9380:
	goto	l1866
	line	234
	
l29494:	
;adc.c: 232: {
;adc.c: 234: ( ADCON0 = ( 0x0C << 2 ) | 0x01 );
	movlw	(031h)
	movwf	(157)^080h	;volatile
	line	235
	
l29496:	
;adc.c: 235: ADC_Wait( );
	fcall	_ADC_Wait
	line	236
;adc.c: 236: ui16_Ubat_bldc.w = ADC_Read( );
	fcall	_ADC_Read
	movf	(1+(?_ADC_Read)),w
	movwf	(_ui16_Ubat_bldc+1)	;volatile
	movf	(0+(?_ADC_Read)),w
	movwf	(_ui16_Ubat_bldc)	;volatile
	line	237
	
l29498:	
;adc.c: 237: inputArray1[ windowPtr1 ] = 350;
	movlb 1	; select bank1
	lslf	(_windowPtr1)^080h,w
	addlw	_inputArray1&0ffh
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movlw	low(015Eh)
	movwi	[0]fsr1
	movlw	high(015Eh)
	movwi	[1]fsr1
	line	230
	
l29500:	
	incf	(_windowPtr1)^080h,f
	goto	l29490
	line	242
	
l1866:	
	return
	opt stack 0
GLOBAL	__end_of_FILTER_Init
	__end_of_FILTER_Init:
;; =============== function _FILTER_Init ends ============

	signat	_FILTER_Init,88
	global	_Cb53_UbatHandling_node_fcn2
psect	text2030,local,class=CODE,delta=2
global __ptext2030
__ptext2030:

;; *************** function _Cb53_UbatHandling_node_fcn2 *****************
;; Defined at:
;;		line 1882 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\T_Link\BVH2_Appl_Layer.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 1C/1
;;		On exit  : 1F/3
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_Cb53_UbatHandling_node_fcn1
;; This function is called by:
;;		_BVH2_Appl_Layer
;; This function uses a non-reentrant model
;;
psect	text2030
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\T_Link\BVH2_Appl_Layer.c"
	line	1882
	global	__size_of_Cb53_UbatHandling_node_fcn2
	__size_of_Cb53_UbatHandling_node_fcn2	equ	__end_of_Cb53_UbatHandling_node_fcn2-_Cb53_UbatHandling_node_fcn2
	
_Cb53_UbatHandling_node_fcn2:	
	opt	stack 9
; Regs used in _Cb53_UbatHandling_node_fcn2: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	1884
	
l26390:	
;BVH2_Appl_Layer.c: 1884: if (((UInt8)ui8_BattVolt_mat) > 230) {
	movlw	(0E7h)
	movlb 1	; select bank1
	subwf	(_ui8_BattVolt_mat)^080h,w
	skipc
	goto	u6091
	goto	u6090
u6091:
	goto	l26402
u6090:
	line	1886
	
l26392:	
;BVH2_Appl_Layer.c: 1886: Cb53_UbatHandling_node_fcn1();
	fcall	_Cb53_UbatHandling_node_fcn1
	line	1887
	
l26394:	
;BVH2_Appl_Layer.c: 1887: SIBFS_UbatHandling_b.Cb55_SaturationHigh = 1;
	bsf	(_SIBFS_UbatHandling_b)^080h,4
	line	1888
	
l26396:	
;BVH2_Appl_Layer.c: 1888: Cb53_odPumpOff = 1;
	movlb 3	; select bank3
	clrf	(_Cb53_odPumpOff)^0180h
	incf	(_Cb53_odPumpOff)^0180h,f
	line	1889
	
l26398:	
;BVH2_Appl_Layer.c: 1889: Cb53_odFixedValueSel = 0;
	movlb 2	; select bank2
	clrf	(_Cb53_odFixedValueSel)^0100h
	line	1890
	
l26400:	
;BVH2_Appl_Layer.c: 1890: Cb53_oUbat_Alarm_High = 1;
	movlb 3	; select bank3
	clrf	(_Cb53_oUbat_Alarm_High)^0180h
	incf	(_Cb53_oUbat_Alarm_High)^0180h,f
	line	1891
;BVH2_Appl_Layer.c: 1891: }
	goto	l17813
	line	1894
	
l26402:	
;BVH2_Appl_Layer.c: 1892: else {
;BVH2_Appl_Layer.c: 1894: if (((UInt8)ui8_BattVolt_mat) < 45) {
	movlw	(02Dh)
	subwf	(_ui8_BattVolt_mat)^080h,w
	skipnc
	goto	u6101
	goto	u6100
u6101:
	goto	l26414
u6100:
	line	1896
	
l26404:	
;BVH2_Appl_Layer.c: 1896: Cb53_UbatHandling_node_fcn1();
	fcall	_Cb53_UbatHandling_node_fcn1
	line	1897
	
l26406:	
;BVH2_Appl_Layer.c: 1897: SIBFS_UbatHandling_b.Cb56_SaturationLow = 1;
	bsf	(_SIBFS_UbatHandling_b)^080h,5
	goto	l26396
	line	1903
	
l26414:	
;BVH2_Appl_Layer.c: 1902: else {
;BVH2_Appl_Layer.c: 1903: Cb53_UbatHandling_node_fcn1();
	fcall	_Cb53_UbatHandling_node_fcn1
	line	1906
	
l26416:	
;BVH2_Appl_Layer.c: 1906: if (((UInt8)ui8_BattVolt_mat) < 63) {
	movlw	(03Fh)
	subwf	(_ui8_BattVolt_mat)^080h,w
	skipnc
	goto	u6111
	goto	u6110
u6111:
	goto	l17811
u6110:
	line	1908
	
l26418:	
;BVH2_Appl_Layer.c: 1908: SIBFS_UbatHandling_b.Cb58_LimpRangeHigh = 1;
	bsf	(_SIBFS_UbatHandling_b)^080h,7
	line	1909
	
l26420:	
;BVH2_Appl_Layer.c: 1909: Cb53_odPumpOff = 0;
	movlb 3	; select bank3
	clrf	(_Cb53_odPumpOff)^0180h
	line	1910
	
l26422:	
;BVH2_Appl_Layer.c: 1910: Cb53_odFixedValueSel = 1;
	movlb 2	; select bank2
	clrf	(_Cb53_odFixedValueSel)^0100h
	incf	(_Cb53_odFixedValueSel)^0100h,f
	line	1911
;BVH2_Appl_Layer.c: 1911: Cb53_oUbat_Alarm_High = 0;
	movlb 3	; select bank3
	clrf	(_Cb53_oUbat_Alarm_High)^0180h
	line	1912
;BVH2_Appl_Layer.c: 1912: }
	goto	l17813
	line	1913
	
l17811:	
	line	1915
;BVH2_Appl_Layer.c: 1913: else {
;BVH2_Appl_Layer.c: 1915: SIBFS_UbatHandling_b.Cb57_NormalUbat = 1;
	bsf	(_SIBFS_UbatHandling_b)^080h,6
	line	1916
	
l26424:	
;BVH2_Appl_Layer.c: 1916: Cb53_odPumpOff = 0;
	movlb 3	; select bank3
	clrf	(_Cb53_odPumpOff)^0180h
	line	1917
;BVH2_Appl_Layer.c: 1917: Cb53_odFixedValueSel = 0;
	movlb 2	; select bank2
	clrf	(_Cb53_odFixedValueSel)^0100h
	line	1918
;BVH2_Appl_Layer.c: 1918: Cb53_oUbat_Alarm_High = 0;
	movlb 3	; select bank3
	clrf	(_Cb53_oUbat_Alarm_High)^0180h
	line	1922
	
l17813:	
	return
	opt stack 0
GLOBAL	__end_of_Cb53_UbatHandling_node_fcn2
	__end_of_Cb53_UbatHandling_node_fcn2:
;; =============== function _Cb53_UbatHandling_node_fcn2 ends ============

	signat	_Cb53_UbatHandling_node_fcn2,88
	global	_timer_init
psect	text2031,local,class=CODE,delta=2
global __ptext2031
__ptext2031:

;; *************** function _timer_init *****************
;; Defined at:
;;		line 75 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\timer.c"
;; Parameters:    Size  Location     Type
;;  ui8_TmrNb       1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  ui8_TmrNb       1   13[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 16/1
;;		On exit  : 16/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       1       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       1       0       0       0       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_clear_timer
;; This function is called by:
;;		_system_init
;; This function uses a non-reentrant model
;;
psect	text2031
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\timer.c"
	line	75
	global	__size_of_timer_init
	__size_of_timer_init	equ	__end_of_timer_init-_timer_init
	
_timer_init:	
	opt	stack 9
; Regs used in _timer_init: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;timer_init@ui8_TmrNb stored from wreg
	movlb 0	; select bank0
	movwf	(timer_init@ui8_TmrNb)
	line	77
	
l26356:	
;timer.c: 77: switch( ui8_TmrNb )
	goto	l26388
	line	86
	
l26358:	
;timer.c: 83: {
;timer.c: 86: clear_timer( 0 );
	movlw	(0)
	fcall	_clear_timer
	line	87
	
l26360:	
;timer.c: 87: TMR0IE = 0;
	bcf	(93/8),(93)&7
	line	88
;timer.c: 88: break;
	goto	l19630
	line	98
	
l26362:	
;timer.c: 95: {
;timer.c: 98: clear_timer( 1 );
	movlw	(01h)
	fcall	_clear_timer
	line	99
	
l26364:	
;timer.c: 99: T1CON = 0b01110101;
	movlw	(075h)
	movlb 0	; select bank0
	movwf	(24)	;volatile
	line	100
	
l26366:	
;timer.c: 100: TMR1IE = 0;
	movlb 1	; select bank1
	bcf	(1160/8)^080h,(1160)&7
	line	101
;timer.c: 101: break;
	goto	l19630
	line	111
	
l26368:	
;timer.c: 108: {
;timer.c: 111: clear_timer( 2 );
	movlw	(02h)
	fcall	_clear_timer
	line	112
	
l26370:	
;timer.c: 112: T2CON = 0b00000111;
	movlw	(07h)
	movlb 0	; select bank0
	movwf	(28)	;volatile
	line	113
	
l26372:	
;timer.c: 113: TMR2IE = 0;
	movlb 1	; select bank1
	bcf	(1161/8)^080h,(1161)&7
	line	114
;timer.c: 114: break;
	goto	l19630
	line	124
	
l26374:	
;timer.c: 121: {
;timer.c: 124: clear_timer( 4 );
	movlw	(04h)
	fcall	_clear_timer
	line	125
	
l26376:	
;timer.c: 125: T4CON = 0b00001110;
	movlw	(0Eh)
	movlb 8	; select bank8
	movwf	(1047)^0400h	;volatile
	line	129
	
l26378:	
;timer.c: 129: TMR4IE = 1;
	movlb 1	; select bank1
	bsf	(1177/8)^080h,(1177)&7
	line	131
;timer.c: 131: break;
	goto	l19630
	line	141
	
l26380:	
;timer.c: 138: {
;timer.c: 141: clear_timer( 6 );
	movlw	(06h)
	fcall	_clear_timer
	line	142
	
l26382:	
;timer.c: 142: T6CON = 0b00000101;
	movlw	(05h)
	movlb 8	; select bank8
	movwf	(1054)^0400h	;volatile
	line	147
	
l26384:	
;timer.c: 147: TMR6IE = 1;
	movlb 1	; select bank1
	bsf	(1179/8)^080h,(1179)&7
	line	169
;timer.c: 169: break;
	goto	l19630
	line	77
	
l26388:	
	movf	(timer_init@ui8_TmrNb),w
	; Switch size 1, requested type "space"
; Number of cases is 5, Range of values is 0 to 6
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           16     9 (average)
; direct_byte           20     6 (fixed)
; jumptable            260     6 (fixed)
; rangetable            11     4 (fixed)
; spacedrange           19     6 (fixed)
; locatedrange           7     3 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	0^0	; case 0
	skipnz
	goto	l26358
	xorlw	1^0	; case 1
	skipnz
	goto	l26362
	xorlw	2^1	; case 2
	skipnz
	goto	l26368
	xorlw	4^2	; case 4
	skipnz
	goto	l26374
	xorlw	6^4	; case 6
	skipnz
	goto	l26380
	goto	l19630
	opt asmopt_on

	line	186
	
l19630:	
	return
	opt stack 0
GLOBAL	__end_of_timer_init
	__end_of_timer_init:
;; =============== function _timer_init ends ============

	signat	_timer_init,4216
	global	_init_ports
psect	text2032,local,class=CODE,delta=2
global __ptext2032
__ptext2032:

;; *************** function _init_ports *****************
;; Defined at:
;;		line 396 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, pclath, cstack
;; Tracked objects:
;;		On entry : 17F/3
;;		On exit  : 17F/1
;;		Unchanged: FFE80/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_read_eeprom_data
;; This function is called by:
;;		_system_init
;; This function uses a non-reentrant model
;;
psect	text2032
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\main.c"
	line	396
	global	__size_of_init_ports
	__size_of_init_ports	equ	__end_of_init_ports-_init_ports
	
_init_ports:	
	opt	stack 9
; Regs used in _init_ports: [wreg+status,2+pclath+cstack]
	line	401
	
l29470:	
;main.c: 401: ANSELA = 0b00100111;
	movlw	(027h)
	movwf	(396)^0180h	;volatile
	line	402
;main.c: 402: ANSELB = 0b00101001;
	movlw	(029h)
	movwf	(397)^0180h	;volatile
	line	405
	
l29472:	
;main.c: 405: LATA = 0;
	movlb 2	; select bank2
	clrf	(268)^0100h	;volatile
	line	406
	
l29474:	
;main.c: 406: LATB = 0;
	clrf	(269)^0100h	;volatile
	line	407
	
l29476:	
;main.c: 407: LATC = 0;
	clrf	(270)^0100h	;volatile
	line	412
	
l29478:	
;main.c: 412: if ((read_eeprom_data(0x55)==0x55) || (RC5==0) )
	movlw	(055h)
	fcall	_read_eeprom_data
	xorlw	055h&0ffh
	skipnz
	goto	u9361
	goto	u9360
u9361:
	goto	l29482
u9360:
	
l29480:	
	btfsc	(117/8),(117)&7
	goto	u9371
	goto	u9370
u9371:
	goto	l29486
u9370:
	line	416
	
l29482:	
;main.c: 414: {
;main.c: 416: TRISA = 0b01111111;
	movlw	(07Fh)
	movlb 1	; select bank1
	movwf	(140)^080h	;volatile
	line	425
;main.c: 425: TRISB = 0b00111001;
	movlw	(039h)
	movwf	(141)^080h	;volatile
	line	434
	
l29484:	
;main.c: 434: TRISC = 0b00000000;
	clrf	(142)^080h	;volatile
	line	445
;main.c: 445: }
	goto	l15459
	line	451
	
l29486:	
;main.c: 448: else
;main.c: 449: {
;main.c: 451: TRISA = 0b01111111;
	movlw	(07Fh)
	movlb 1	; select bank1
	movwf	(140)^080h	;volatile
	line	460
;main.c: 460: TRISB = 0b00111001;
	movlw	(039h)
	movwf	(141)^080h	;volatile
	line	469
;main.c: 469: TRISC = 0b10000000;
	movlw	(080h)
	movwf	(142)^080h	;volatile
	line	521
	
l15459:	
	return
	opt stack 0
GLOBAL	__end_of_init_ports
	__end_of_init_ports:
;; =============== function _init_ports ends ============

	signat	_init_ports,88
	global	__ELINMIntSendMessage
psect	text2033,local,class=CODE,delta=2
global __ptext2033
__ptext2033:

;; *************** function __ELINMIntSendMessage *****************
;; Defined at:
;;		line 1184 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
;; Parameters:    Size  Location     Type
;;  _ELINM_idr      1    wreg     unsigned char 
;;  _ELINM_size     1   13[BANK0 ] unsigned char 
;;  _ELINM_fmin     2   14[BANK0 ] unsigned int 
;;  _ELINM_fmax     2   16[BANK0 ] unsigned int 
;; Auto vars:     Size  Location     Type
;;  _ELINM_idr      1   18[BANK0 ] unsigned char 
;;  _ELINM_chk      2   20[BANK0 ] unsigned int 
;;  _ELINM_i        1   22[BANK0 ] unsigned char 
;;  _ELINM_tid      1   19[BANK0 ] struct ELINMINT_ID
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr1l, fsr1h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       5       0       0       0       0       0       0
;;      Locals:         0       5       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0      10       0       0       0       0       0       0
;;Total ram usage:       10 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		__ELINMIntCalcIDParity
;; This function is called by:
;;		_Transmit_LIN_8Bytes
;;		__ELINMIntReceiveMessage
;; This function uses a non-reentrant model
;;
psect	text2033
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
	line	1184
	global	__size_of__ELINMIntSendMessage
	__size_of__ELINMIntSendMessage	equ	__end_of__ELINMIntSendMessage-__ELINMIntSendMessage
	
__ELINMIntSendMessage:	
	opt	stack 7
; Regs used in __ELINMIntSendMessage: [wreg+fsr1l-status,0+pclath+cstack]
;__ELINMIntSendMessage@_ELINM_idr stored from wreg
	line	1191
	movwf	(__ELINMIntSendMessage@_ELINM_idr)
	
l29414:	
;lin.c: 1186: char _ELINM_i;
;lin.c: 1187: ELINMINT_ID _ELINM_tid;
;lin.c: 1188: unsigned int _ELINM_chk;
;lin.c: 1191: _ELINM_tid.ID = _ELINM_idr;
	movf	(__ELINMIntSendMessage@_ELINM_idr),w
	movwf	(__ELINMIntSendMessage@_ELINM_tid)
	line	1192
	
l29416:	
;lin.c: 1192: _ELINMIntStatus.ELINMIntStatusByte &= 0x0F;
	movlw	(0Fh)
	andwf	(__ELINMIntStatus),f
	line	1194
	
l29418:	
;lin.c: 1194: _ELINMIntTHeaderMin = ( 34L * ( 100L * 1000000L / 19200L ) / 128L ) / 100;
	movlw	0Dh
	movlb 3	; select bank3
	movwf	(__ELINMIntTHeaderMin)^0180h
	clrf	(__ELINMIntTHeaderMin+1)^0180h
	line	1195
	
l29420:	
;lin.c: 1195: _ELINMIntTHeaderMax = ( ( ( ( 34L + 1 ) * 14L ) / 10L ) * ( 100L * 1000000L / 19200L ) / 128L ) / 100;
	movlw	013h
	movlb 4	; select bank4
	movwf	(__ELINMIntTHeaderMax)^0200h
	clrf	(__ELINMIntTHeaderMax+1)^0200h
	line	1196
	
l29422:	
;lin.c: 1196: _ELINMIntTFrameMin = _ELINM_fmin;
	movlb 0	; select bank0
	movf	(__ELINMIntSendMessage@_ELINM_fmin+1),w
	movlb 3	; select bank3
	movwf	(__ELINMIntTFrameMin+1)^0180h
	movlb 0	; select bank0
	movf	(__ELINMIntSendMessage@_ELINM_fmin),w
	movlb 3	; select bank3
	movwf	(__ELINMIntTFrameMin)^0180h
	line	1197
	
l29424:	
;lin.c: 1197: _ELINMIntTFrameMax = _ELINM_fmax;
	movlb 0	; select bank0
	movf	(__ELINMIntSendMessage@_ELINM_fmax+1),w
	movlb 4	; select bank4
	movwf	(__ELINMIntTFrameMax+1)^0200h
	movlb 0	; select bank0
	movf	(__ELINMIntSendMessage@_ELINM_fmax),w
	movlb 4	; select bank4
	movwf	(__ELINMIntTFrameMax)^0200h
	line	1207
	
l29426:	
;lin.c: 1207: if( _ELINMIntStatus.ELINMINTSTS.IDLE)
	movlb 0	; select bank0
	btfss	(__ELINMIntStatus),3
	goto	u9331
	goto	u9330
u9331:
	goto	l13482
u9330:
	line	1229
	
l29428:	
;lin.c: 1210: {
;lin.c: 1229: _ELINMIntMessageBuffer[ 0 ] = 0x55;
	movlw	(055h)
	movlb 4	; select bank4
	movwf	(__ELINMIntMessageBuffer)^0200h
	line	1230
	
l29430:	
;lin.c: 1230: _ELINMIntMessageBuffer[ 1 ] = _ELINMIntCalcIDParity( _ELINM_tid );
	movlb 0	; select bank0
	movf	(__ELINMIntSendMessage@_ELINM_tid),w
	fcall	__ELINMIntCalcIDParity
	movlb 4	; select bank4
	movwf	0+(__ELINMIntMessageBuffer)^0200h+01h
	line	1231
	
l29432:	
;lin.c: 1231: _ELINMIntMessageBufferPointer = 0;
	movlb 1	; select bank1
	clrf	(__ELINMIntMessageBufferPointer)^080h
	line	1238
	
l29434:	
;lin.c: 1238: if( _ELINMIntStatus.ELINMINTSTS.RX )
	movlb 0	; select bank0
	btfss	(__ELINMIntStatus),1
	goto	u9341
	goto	u9340
u9341:
	goto	l29438
u9340:
	line	1243
	
l29436:	
;lin.c: 1241: {
;lin.c: 1243: _ELINMIntMessageSize.SIZE = 2;
	movlw	(02h)
	movlb 2	; select bank2
	movwf	(__ELINMIntMessageSize)^0100h
	line	1246
;lin.c: 1246: }
	goto	l29454
	line	1252
	
l29438:	
;lin.c: 1249: else
;lin.c: 1250: {
;lin.c: 1252: _ELINMIntMessageSize.SIZE = _ELINM_size + 2;
	movf	(__ELINMIntSendMessage@_ELINM_size),w
	addlw	02h
	movlb 2	; select bank2
	movwf	(__ELINMIntMessageSize)^0100h
	line	1253
	
l29440:	
;lin.c: 1253: _ELINM_chk = 0;
	movlb 0	; select bank0
	clrf	(__ELINMIntSendMessage@_ELINM_chk)
	clrf	(__ELINMIntSendMessage@_ELINM_chk+1)
	line	1256
	
l29442:	
;lin.c: 1256: for( _ELINM_i = 2;
	movlw	(02h)
	movwf	(__ELINMIntSendMessage@_ELINM_i)
;lin.c: 1257: _ELINM_i < _ELINMIntMessageSize.SIZE;
;lin.c: 1258: _ELINM_i++ )
	goto	l29448
	line	1263
	
l29444:	
;lin.c: 1261: {
;lin.c: 1263: _ELINM_chk += _ELINMIntMessageBuffer[ _ELINM_i ];
	movf	(__ELINMIntSendMessage@_ELINM_i),w
	addlw	__ELINMIntMessageBuffer&0ffh
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	indf1,w
	addwf	(__ELINMIntSendMessage@_ELINM_chk),f
	skipnc
	incf	(__ELINMIntSendMessage@_ELINM_chk+1),f
	line	1258
	
l29446:	
	incf	(__ELINMIntSendMessage@_ELINM_i),f
	line	1257
	
l29448:	
	movlb 2	; select bank2
	movf	(__ELINMIntMessageSize)^0100h,w
	movlb 0	; select bank0
	subwf	(__ELINMIntSendMessage@_ELINM_i),w
	skipc
	goto	u9351
	goto	u9350
u9351:
	goto	l29444
u9350:
	line	1270
	
l29450:	
;lin.c: 1266: }
;lin.c: 1269: _ELINMIntMessageBuffer[ _ELINMIntMessageSize.SIZE ]
;lin.c: 1270: = ( ~( _ELINM_chk + ( _ELINM_chk>>8 ) ) );
	movlb 2	; select bank2
	movf	(__ELINMIntMessageSize)^0100h,w
	addlw	__ELINMIntMessageBuffer&0ffh
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movlb 0	; select bank0
	movf	(__ELINMIntSendMessage@_ELINM_chk+1),w
	addwf	(__ELINMIntSendMessage@_ELINM_chk),w
	xorlw	0ffh
	movwf	indf1
	line	1271
	
l29452:	
;lin.c: 1271: _ELINMIntMessageSize.SIZE++;
	movlb 2	; select bank2
	incf	(__ELINMIntMessageSize)^0100h,f
	line	1277
	
l29454:	
;lin.c: 1274: }
;lin.c: 1277: SENDB = 1;
	movlb 3	; select bank3
	bsf	(3315/8)^0180h,(3315)&7
	line	1278
	
l29456:	
;lin.c: 1278: TXREG = 0xFF;
	movlw	(0FFh)
	movwf	(410)^0180h	;volatile
	line	1279
	
l29458:	
;lin.c: 1279: _ELINMIntStatus.ELINMIntStatusByte &= 0x0F;
	movlw	(0Fh)
	movlb 0	; select bank0
	andwf	(__ELINMIntStatus),f
	line	1280
	
l29460:	
;lin.c: 1280: _ELINMIntStatus.ELINMINTSTS.TX = 1;
	bsf	(__ELINMIntStatus),0
	line	1281
	
l29462:	
;lin.c: 1281: _ELINMIntStatus1.ELINMINTSTS.HEADER = 1;
	movlb 1	; select bank1
	bsf	(__ELINMIntStatus1)^080h,1
	line	1282
	
l29464:	
;lin.c: 1282: _ELINMIntStatus1.ELINMINTSTS.FRAME = 1;
	bsf	(__ELINMIntStatus1)^080h,2
	line	1283
	
l29466:	
;lin.c: 1283: _ELINMIntReadBack = 0x00;
	movlb 2	; select bank2
	clrf	(__ELINMIntReadBack)^0100h
	line	1284
	
l29468:	
;lin.c: 1284: _ELINMIntStatus.ELINMINTSTS.IDLE = 0;
	movlb 0	; select bank0
	bcf	(__ELINMIntStatus),3
	line	1290
	
l13482:	
	return
	opt stack 0
GLOBAL	__end_of__ELINMIntSendMessage
	__end_of__ELINMIntSendMessage:
;; =============== function __ELINMIntSendMessage ends ============

	signat	__ELINMIntSendMessage,16504
	global	_PWMReadDC
psect	text2034,local,class=CODE,delta=2
global __ptext2034
__ptext2034:

;; *************** function _PWMReadDC *****************
;; Defined at:
;;		line 363 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\pwm.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;                  1    wreg      unsigned char 
;; Registers used:
;;		wreg, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/2
;;		On exit  : 1F/2
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		___tmul
;;		___ltdiv
;; This function is called by:
;;		_EOL
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2034
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\pwm.c"
	line	363
	global	__size_of_PWMReadDC
	__size_of_PWMReadDC	equ	__end_of_PWMReadDC-_PWMReadDC
	
_PWMReadDC:	
	opt	stack 9
; Regs used in _PWMReadDC: [wreg+status,2+status,0+pclath+cstack]
	line	366
	
l29394:	
;pwm.c: 366: ui8_PWMinDC_sav = (unsigned char) ( 200*(unsigned short long )(ui16_Duty_Cycle_In) / ui16_PWM_Freq_In ) ;
	movlb 3	; select bank3
	movf	(_ui16_PWM_Freq_In)^0180h,w
	movlb 0	; select bank0
	movwf	(?___ltdiv)
	movlb 3	; select bank3
	movf	(_ui16_PWM_Freq_In+1)^0180h,w
	movlb 0	; select bank0
	movwf	(?___ltdiv+1)
	clrf	(?___ltdiv+2)
	movlb 3	; select bank3
	movf	(_ui16_Duty_Cycle_In)^0180h,w
	movlb 0	; select bank0
	movwf	(?___tmul)
	movlb 3	; select bank3
	movf	(_ui16_Duty_Cycle_In+1)^0180h,w
	movlb 0	; select bank0
	movwf	(?___tmul+1)
	clrf	(?___tmul+2)
	movlw	0C8h
	movwf	0+(?___tmul)+03h
	clrf	1+(?___tmul)+03h
	clrf	2+(?___tmul)+03h
	fcall	___tmul
	movf	(0+(?___tmul)),w
	movwf	0+(?___ltdiv)+03h
	movf	(1+(?___tmul)),w
	movwf	1+(?___ltdiv)+03h
	movf	(2+(?___tmul)),w
	movwf	2+(?___ltdiv)+03h
	fcall	___ltdiv
	movf	(0+(?___ltdiv)),w
	movlb 2	; select bank2
	movwf	(_ui8_PWMinDC_sav)^0100h
	line	368
	
l29396:	
;pwm.c: 368: if( ui8_PWMinDC_sav == 0 )
	movf	(_ui8_PWMinDC_sav)^0100h,f
	skipz
	goto	u9311
	goto	u9310
u9311:
	goto	l29404
u9310:
	line	374
	
l29398:	
;pwm.c: 370: {
;pwm.c: 374: ui8_PWMin_failCnt++;
	incf	(_ui8_PWMin_failCnt)^0100h,f
	line	376
	
l29400:	
;pwm.c: 376: if( ui8_PWMin_failCnt >= 5 )
	movlw	(05h)
	subwf	(_ui8_PWMin_failCnt)^0100h,w
	skipc
	goto	u9321
	goto	u9320
u9321:
	goto	l29410
u9320:
	line	380
	
l29402:	
;pwm.c: 378: {
;pwm.c: 380: ui8_Duty_Cycle_In_Ratio = ui8_PWMinDC_sav;
	movf	(_ui8_PWMinDC_sav)^0100h,w
	movwf	(_ui8_Duty_Cycle_In_Ratio)^0100h
	goto	l29410
	line	396
	
l29404:	
;pwm.c: 391: else
;pwm.c: 392: {
;pwm.c: 396: ui8_PWMin_failCnt = 0;
	clrf	(_ui8_PWMin_failCnt)^0100h
	line	399
	
l29406:	
;pwm.c: 399: ui8_Duty_Cycle_In_Ratio = ui8_PWMinDC_sav;
	movf	(_ui8_PWMinDC_sav)^0100h,w
	movwf	(_ui8_Duty_Cycle_In_Ratio)^0100h
	line	401
	
l29408:	
;pwm.c: 401: ui16_Duty_Cycle_In = 0;
	movlb 3	; select bank3
	clrf	(_ui16_Duty_Cycle_In)^0180h
	clrf	(_ui16_Duty_Cycle_In+1)^0180h
	line	408
	
l29410:	
;pwm.c: 403: }
;pwm.c: 408: return ( 200 - ui8_Duty_Cycle_In_Ratio );
	movlb 2	; select bank2
	movf	(_ui8_Duty_Cycle_In_Ratio)^0100h,w
	sublw	0C8h
	line	418
	
l17343:	
	return
	opt stack 0
GLOBAL	__end_of_PWMReadDC
	__end_of_PWMReadDC:
;; =============== function _PWMReadDC ends ============

	signat	_PWMReadDC,89
	global	_SetDiagAlarm
psect	text2035,local,class=CODE,delta=2
global __ptext2035
__ptext2035:

;; *************** function _SetDiagAlarm *****************
;; Defined at:
;;		line 213 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 1F/1
;;		On exit  : 1F/2
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_PWM_Write_Out
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2035
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
	line	213
	global	__size_of_SetDiagAlarm
	__size_of_SetDiagAlarm	equ	__end_of_SetDiagAlarm-_SetDiagAlarm
	
_SetDiagAlarm:	
	opt	stack 10
; Regs used in _SetDiagAlarm: [wreg+status,2+status,0+pclath+cstack]
	line	219
	
l29368:	
;diag.c: 219: if( ui8_error_Flags.bits.B6 )
	btfss	(_ui8_error_Flags)^080h,6
	goto	u9241
	goto	u9240
u9241:
	goto	l29372
u9240:
	line	223
	
l29370:	
;diag.c: 221: {
;diag.c: 223: PWM_Write_Out( 88 );
	movlw	(058h)
	fcall	_PWM_Write_Out
	line	226
;diag.c: 226: }
	goto	l7640
	line	232
	
l29372:	
;diag.c: 229: else
;diag.c: 230: {
;diag.c: 232: if( bool_CPU_TempAlarm )
	movlb 2	; select bank2
	movf	(_bool_CPU_TempAlarm)^0100h,w
	skipz
	goto	u9250
	goto	l29376
u9250:
	line	237
	
l29374:	
;diag.c: 235: {
;diag.c: 237: PWM_Write_Out( 33 );
	movlw	(021h)
	fcall	_PWM_Write_Out
	line	240
;diag.c: 240: }
	goto	l7640
	line	246
	
l29376:	
;diag.c: 243: else
;diag.c: 244: {
;diag.c: 246: if( bool_DryRunningAlarm )
	movf	(_bool_DryRunningAlarm)^0100h,w
	skipz
	goto	u9260
	goto	l29380
u9260:
	line	250
	
l29378:	
;diag.c: 248: {
;diag.c: 250: PWM_Write_Out( 66 );
	movlw	(042h)
	fcall	_PWM_Write_Out
	line	253
;diag.c: 253: }
	goto	l7640
	line	259
	
l29380:	
;diag.c: 256: else
;diag.c: 257: {
;diag.c: 259: if( bool_MotorStalled )
	movf	(_bool_MotorStalled)^0100h,w
	skipz
	goto	u9270
	goto	l29384
u9270:
	line	263
	
l29382:	
;diag.c: 261: {
;diag.c: 263: PWM_Write_Out( 55 );
	movlw	(037h)
	fcall	_PWM_Write_Out
	line	266
;diag.c: 266: }
	goto	l7640
	line	272
	
l29384:	
;diag.c: 269: else
;diag.c: 270: {
;diag.c: 272: if( bool_HighCurrentAlarm )
	movf	(_bool_HighCurrentAlarm)^0100h,w
	skipz
	goto	u9280
	goto	l7634
u9280:
	line	276
	
l29386:	
;diag.c: 274: {
;diag.c: 276: PWM_Write_Out( 44 );
	movlw	(02Ch)
	fcall	_PWM_Write_Out
	line	278
;diag.c: 278: }
	goto	l7640
	line	281
	
l7634:	
	line	284
;diag.c: 281: else
;diag.c: 282: {
;diag.c: 284: if( ui8_error_Flags.bits.B5 )
	movlb 1	; select bank1
	btfss	(_ui8_error_Flags)^080h,5
	goto	u9291
	goto	u9290
u9291:
	goto	l7636
u9290:
	line	289
	
l29388:	
;diag.c: 287: {
;diag.c: 289: PWM_Write_Out( 77 );
	movlw	(04Dh)
	fcall	_PWM_Write_Out
	line	292
;diag.c: 292: }
	goto	l7640
	line	295
	
l7636:	
	line	298
;diag.c: 295: else
;diag.c: 296: {
;diag.c: 298: if( ui8_error_Flags.bits.B0 )
	btfss	(_ui8_error_Flags)^080h,0
	goto	u9301
	goto	u9300
u9301:
	goto	l29392
u9300:
	line	302
	
l29390:	
;diag.c: 300: {
;diag.c: 302: PWM_Write_Out( 22 );
	movlw	(016h)
	fcall	_PWM_Write_Out
	line	305
;diag.c: 305: }
	goto	l7640
	line	311
	
l29392:	
;diag.c: 308: else
;diag.c: 309: {
;diag.c: 311: PWM_Write_Out( 11 );
	movlw	(0Bh)
	fcall	_PWM_Write_Out
	line	344
	
l7640:	
	return
	opt stack 0
GLOBAL	__end_of_SetDiagAlarm
	__end_of_SetDiagAlarm:
;; =============== function _SetDiagAlarm ends ============

	signat	_SetDiagAlarm,88
	global	_DiagInit
psect	text2036,local,class=CODE,delta=2
global __ptext2036
__ptext2036:

;; *************** function _DiagInit *****************
;; Defined at:
;;		line 150 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 1F/2
;;		On exit  : 1F/3
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_PWM_Write_Out
;; This function is called by:
;;		_system_init
;; This function uses a non-reentrant model
;;
psect	text2036
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\diag.c"
	line	150
	global	__size_of_DiagInit
	__size_of_DiagInit	equ	__end_of_DiagInit-_DiagInit
	
_DiagInit:	
	opt	stack 9
; Regs used in _DiagInit: [wreg+status,2+status,0+pclath+cstack]
	line	162
	
l29358:	
;diag.c: 162: ui8_error_Flags.b = 0;
	movlb 1	; select bank1
	clrf	(_ui8_error_Flags)^080h
	line	163
	
l29360:	
;diag.c: 163: PWM_Write_Out( 11 );
	movlw	(0Bh)
	fcall	_PWM_Write_Out
	line	165
	
l29362:	
;diag.c: 165: ui16_IPhase1_bldc_cal.w = 0;
	movlb 3	; select bank3
	clrf	(_ui16_IPhase1_bldc_cal)^0180h
	clrf	(_ui16_IPhase1_bldc_cal+1)^0180h
	line	166
	
l29364:	
;diag.c: 166: ui16_IPhase2_bldc_cal.w = 0;
	clrf	(_ui16_IPhase2_bldc_cal)^0180h
	clrf	(_ui16_IPhase2_bldc_cal+1)^0180h
	line	167
	
l29366:	
;diag.c: 167: ui16_IPhase3_bldc_cal.w = 0;
	clrf	(_ui16_IPhase3_bldc_cal)^0180h
	clrf	(_ui16_IPhase3_bldc_cal+1)^0180h
	line	171
	
l7623:	
	return
	opt stack 0
GLOBAL	__end_of_DiagInit
	__end_of_DiagInit:
;; =============== function _DiagInit ends ============

	signat	_DiagInit,88
	global	_cksum
psect	text2037,local,class=CODE,delta=2
global __ptext2037
__ptext2037:

;; *************** function _cksum *****************
;; Defined at:
;;		line 6 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\cksum.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  addr            2   18[BANK0 ] unsigned short 
;;  sum             2   16[BANK0 ] unsigned int 
;;  counter         2   14[BANK0 ] unsigned int 
;; Return value:  Size  Location     Type
;;                  2   12[BANK0 ] unsigned int 
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       2       0       0       0       0       0       0
;;      Locals:         0       6       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       8       0       0       0       0       0       0
;;Total ram usage:        8 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_EOL
;; This function uses a non-reentrant model
;;
psect	text2037
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\cksum.c"
	line	6
	global	__size_of_cksum
	__size_of_cksum	equ	__end_of_cksum-_cksum
	
_cksum:	
	opt	stack 10
; Regs used in _cksum: [wreg+status,2+status,0]
	line	11
	
l29334:	
;cksum.c: 7: unsigned int sum;
;cksum.c: 8: unsigned short addr;
;cksum.c: 9: unsigned int counter;
;cksum.c: 11: addr = 0L;
	clrf	(cksum@addr)
	clrf	(cksum@addr+1)
	line	12
;cksum.c: 12: sum = 0;
	clrf	(cksum@sum)
	clrf	(cksum@sum+1)
	line	13
	
l29336:	
;cksum.c: 13: counter = ( (8192 - 2 - 1) - 0L + 1 );
	movlw	low(01FFEh)
	movwf	(cksum@counter)
	movlw	high(01FFEh)
	movwf	((cksum@counter))+1
	line	15
;cksum.c: 15: while(counter--){
	goto	l29352
	line	22
	
l29338:	
;cksum.c: 22: sum += (EEADRL=(addr)&0xff, EEADRH=(addr)>>8, WREN=0, EECON1 |= 0x80, RD=1, _nop(), _nop(), (EEDATH << 8) | EEDATA);
	movf	(cksum@addr),w
	movlb 3	; select bank3
	movwf	(401)^0180h	;volatile
	movlb 0	; select bank0
	movf	(cksum@addr+1),w
	movlb 3	; select bank3
	movwf	(402)^0180h	;volatile
	
l29340:	
	bcf	(3242/8)^0180h,(3242)&7
	
l29342:	
	bsf	(405)^0180h+(7/8),(7)&7	;volatile
	
l29344:	
	bsf	(3240/8)^0180h,(3240)&7
	
l29346:	
	nop
	
l29348:	
	nop
	movlb 3	; select bank3
	movf	(403)^0180h,w	;volatile
	movlb 0	; select bank0
	addwf	(cksum@sum),f
	movlb 3	; select bank3
	movf	(404)^0180h,w	;volatile
	movlb 0	; select bank0
	addwfc	(cksum@sum+1),f
	line	31
	
l29350:	
;cksum.c: 31: addr++;
	incf	(cksum@addr),f
	skipnz
	incf	(cksum@addr+1),f
	line	15
	
l29352:	
	movlw	low(01h)
	subwf	(cksum@counter),f
	movlw	high(01h)
	subwfb	(cksum@counter+1),f
	incf	((cksum@counter)),w
	skipnz
	incf	((cksum@counter+1)),w

	skipz
	goto	u9231
	goto	u9230
u9231:
	goto	l29338
u9230:
	line	33
	
l29354:	
;cksum.c: 32: }
;cksum.c: 33: return sum;
	movf	(cksum@sum+1),w
	movwf	(?_cksum+1)
	movf	(cksum@sum),w
	movwf	(?_cksum)
	line	34
	
l5720:	
	return
	opt stack 0
GLOBAL	__end_of_cksum
	__end_of_cksum:
;; =============== function _cksum ends ============

	signat	_cksum,90
	global	_commutate
psect	text2038,local,class=CODE,delta=2
global __ptext2038
__ptext2038:

;; *************** function _commutate *****************
;; Defined at:
;;		line 1364 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\bldc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, fsr1l, fsr1h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0      12       0       0       0       0       0       0
;;      Totals:         0      12       0       0       0       0       0       0
;;Total ram usage:       12 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		___wmul
;; This function is called by:
;;		_InitMotorRun
;; This function uses a non-reentrant model
;;
psect	text2038
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\bldc.c"
	line	1364
	global	__size_of_commutate
	__size_of_commutate	equ	__end_of_commutate-_commutate
	
_commutate:	
	opt	stack 9
; Regs used in _commutate: [wreg-status,0+pclath+cstack]
	line	1366
	
l29070:	
;bldc.c: 1366: if( ui16_step_cnt < 10 )
	movlw	high(0Ah)
	movlb 1	; select bank1
	subwf	(_ui16_step_cnt+1)^080h,w
	movlw	low(0Ah)
	skipnz
	subwf	(_ui16_step_cnt)^080h,w
	skipnc
	goto	u9041
	goto	u9040
u9041:
	goto	l29074
u9040:
	line	1370
	
l29072:	
;bldc.c: 1368: {
;bldc.c: 1370: ui16_step_cnt++;
	incf	(_ui16_step_cnt)^080h,f
	skipnz
	incf	(_ui16_step_cnt+1)^080h,f
	line	1375
	
l29074:	
;bldc.c: 1372: }
;bldc.c: 1375: B[ Bcnt++ ] = ( comm_time );
	movlb 2	; select bank2
	lslf	(_Bcnt)^0100h,w
	addlw	_B&0ffh
	movwf	fsr1l
	movlw 1	; select bank3/4
	movwf fsr1h	
	
	movlb 0	; select bank0
	movf	(_comm_time),w
	movwi	[0]fsr1
	movf	(_comm_time+1),w
	movwi	[1]fsr1
	
l29076:	
	movlb 2	; select bank2
	incf	(_Bcnt)^0100h,f
	line	1376
	
l29078:	
;bldc.c: 1376: Bcnt &= ( ( sizeof( B ) / sizeof( B[ 0 ] ) ) - 1 );
	movlw	(07h)
	andwf	(_Bcnt)^0100h,f
	line	1377
	
l29080:	
;bldc.c: 1377: ui16_speed_fil = ( B[ 0 ] + B[ 1 ] + B[ 2 ] + B[ 3 ] + B[ 4 ] + B[ 5 ] + B[ 6 ] + B[ 7 ] );
	movlw	(0Eh)
	addlw	_B&0ffh
	movwf	fsr1l
	movlw 1	; select bank3/4
	movwf fsr1h	
	
	movlw	(0Ch)
	addlw	_B&0ffh
	movwf	fsr0l
	movlw 1	; select bank3/4
	movwf fsr0h	
	
	movlb 3	; select bank3
	movf	0+(_B)^0180h+04h,w
	addwf	0+(_B)^0180h+02h,w
	movlb 0	; select bank0
	movwf	(??_commutate+0)+0
	movlb 3	; select bank3
	movf	1+(_B)^0180h+04h,w
	addwfc	1+(_B)^0180h+02h,w
	movlb 0	; select bank0
	movwf	1+(??_commutate+0)+0
	movlb 3	; select bank3
	movf	0+(_B)^0180h+06h,w
	movlb 0	; select bank0
	addwf	0+(??_commutate+0)+0,w
	movwf	(??_commutate+2)+0
	movlb 3	; select bank3
	movf	1+(_B)^0180h+06h,w
	movlb 0	; select bank0
	addwfc	1+(??_commutate+0)+0,w
	movwf	1+(??_commutate+2)+0
	movlb 3	; select bank3
	movf	0+(_B)^0180h+08h,w
	movlb 0	; select bank0
	addwf	0+(??_commutate+2)+0,w
	movwf	(??_commutate+4)+0
	movlb 3	; select bank3
	movf	1+(_B)^0180h+08h,w
	movlb 0	; select bank0
	addwfc	1+(??_commutate+2)+0,w
	movwf	1+(??_commutate+4)+0
	movlb 3	; select bank3
	movf	0+(_B)^0180h+0Ah,w
	movlb 0	; select bank0
	addwf	0+(??_commutate+4)+0,w
	movwf	(??_commutate+6)+0
	movlb 3	; select bank3
	movf	1+(_B)^0180h+0Ah,w
	movlb 0	; select bank0
	addwfc	1+(??_commutate+4)+0,w
	movwf	1+(??_commutate+6)+0
	moviw	[0]fsr0
	addwf	0+(??_commutate+6)+0,w
	movwf	(??_commutate+8)+0
	moviw	[1]fsr0
	addwfc	1+(??_commutate+6)+0,w
	movwf	(??_commutate+8)+0+1
	moviw	[0]fsr1
	addwf	0+(??_commutate+8)+0,w
	movwf	(??_commutate+10)+0
	moviw	[1]fsr1
	addwfc	1+(??_commutate+8)+0,w
	movwf	(??_commutate+10)+0+1
	movlb 3	; select bank3
	movf	(_B)^0180h,w
	movlb 0	; select bank0
	addwf	0+(??_commutate+10)+0,w
	movlb 2	; select bank2
	movwf	(_ui16_speed_fil)^0100h
	movlb 3	; select bank3
	movf	(_B+1)^0180h,w
	movlb 0	; select bank0
	addwfc	1+(??_commutate+10)+0,w
	movlb 2	; select bank2
	movwf	1+(_ui16_speed_fil)^0100h
	line	1378
	
l29082:	
;bldc.c: 1378: ui16_speed_rar = comm_time;
	movlb 0	; select bank0
	movf	(_comm_time+1),w
	movlb 3	; select bank3
	movwf	(_ui16_speed_rar+1)^0180h
	movlb 0	; select bank0
	movf	(_comm_time),w
	movlb 3	; select bank3
	movwf	(_ui16_speed_rar)^0180h
	line	1383
	
l29084:	
;bldc.c: 1383: MotorFlags.bits.B5 = 0;
	movlb 0	; select bank0
	bcf	(_MotorFlags),5
	line	1386
	
l29086:	
;bldc.c: 1390: bemf_filter = 62;
	movlb 2	; select bank2
	clrf	(_ui8_sampleState)^0100h
	incf	(_ui8_sampleState)^0100h,f
	line	1393
	
l29088:	
;bldc.c: 1393: MotorFlags.bits.B7 = 1;
	movlb 0	; select bank0
	bsf	(_MotorFlags),7
	line	1396
	
l29090:	
;bldc.c: 1396: if( MotorFlags.bits.B3 )
	btfss	(_MotorFlags),3
	goto	u9051
	goto	u9050
u9051:
	goto	l3855
u9050:
	line	1403
	
l29092:	
;bldc.c: 1398: {
;bldc.c: 1403: ui16_phase_advancement = ( ( ui16_speed_rar>>3 ) * 100 )>>7;
	movlb 3	; select bank3
	movf	(_ui16_speed_rar+1)^0180h,w
	movlb 0	; select bank0
	movwf	(?___wmul+1)
	movlb 3	; select bank3
	movf	(_ui16_speed_rar)^0180h,w
	movlb 0	; select bank0
	movwf	(?___wmul)
	lsrf	(?___wmul+1),f
	rrf	(?___wmul),f
	lsrf	(?___wmul+1),f
	rrf	(?___wmul),f
	lsrf	(?___wmul+1),f
	rrf	(?___wmul),f
	movlw	064h
	movwf	0+(?___wmul)+02h
	clrf	1+(?___wmul)+02h
	fcall	___wmul
	movf	(1+(?___wmul)),w
	movlb 2	; select bank2
	movwf	(_ui16_phase_advancement+1)^0100h
	movlb 0	; select bank0
	movf	(0+(?___wmul)),w
	movlb 2	; select bank2
	movwf	(_ui16_phase_advancement)^0100h
	
l29094:	
	movlw	07h
	
u9065:
	lsrf	(_ui16_phase_advancement+1)^0100h,f
	rrf	(_ui16_phase_advancement)^0100h,f
	decfsz	wreg,f
	goto	u9065
	line	1407
	
l29096:	
;bldc.c: 1407: if( comm_time > ui16_phase_advancement )
	movlb 0	; select bank0
	movf	(_comm_time+1),w
	movlb 2	; select bank2
	subwf	(_ui16_phase_advancement+1)^0100h,w
	skipz
	goto	u9075
	movlb 0	; select bank0
	movf	(_comm_time),w
	movlb 2	; select bank2
	subwf	(_ui16_phase_advancement)^0100h,w
u9075:
	skipnc
	goto	u9071
	goto	u9070
u9071:
	goto	l29100
u9070:
	line	1411
	
l29098:	
;bldc.c: 1409: {
;bldc.c: 1411: phase_delay_counter = ( ( comm_time ) - ui16_phase_advancement );
	movlb 0	; select bank0
	movf	(_comm_time+1),w
	movlb 4	; select bank4
	movwf	(_phase_delay_counter+1)^0200h
	movlb 0	; select bank0
	movf	(_comm_time),w
	movlb 4	; select bank4
	movwf	(_phase_delay_counter)^0200h
	movlb 2	; select bank2
	movf	(_ui16_phase_advancement)^0100h,w
	movlb 4	; select bank4
	subwf	(_phase_delay_counter)^0200h,f
	movlb 2	; select bank2
	movf	(_ui16_phase_advancement+1)^0100h,w
	movlb 4	; select bank4
	subwfb	(_phase_delay_counter+1)^0200h,f
	line	1413
;bldc.c: 1413: }
	goto	l29124
	line	1419
	
l29100:	
;bldc.c: 1416: else
;bldc.c: 1417: {
;bldc.c: 1419: phase_delay_counter = 0;
	movlb 4	; select bank4
	clrf	(_phase_delay_counter)^0200h
	clrf	(_phase_delay_counter+1)^0200h
	goto	l29124
	line	1451
	
l29102:	
;bldc.c: 1449: {
;bldc.c: 1451: ui16_comm_time_max = ( unsigned short ) ( ( 10 * 20000UL ) / ( 600 * 2 ) ) ;
	movlw	0A6h
	movlb 2	; select bank2
	movwf	(_ui16_comm_time_max)^0100h
	clrf	(_ui16_comm_time_max+1)^0100h
	line	1452
;bldc.c: 1452: break;
	goto	l29126
	line	1461
	
l29104:	
;bldc.c: 1459: {
;bldc.c: 1461: ui16_comm_time_max = ( unsigned short ) ( ( 10 * 20000UL ) / ( 200 * 2 ) ) ;
	movlw	low(01F4h)
	movlb 2	; select bank2
	movwf	(_ui16_comm_time_max)^0100h
	movlw	high(01F4h)
	movwf	((_ui16_comm_time_max)^0100h)+1
	line	1462
;bldc.c: 1462: break;
	goto	l29126
	line	1471
	
l29106:	
;bldc.c: 1469: {
;bldc.c: 1471: ui16_comm_time_max = ( unsigned short ) ( ( 10 * 20000UL ) / ( 800 * 2 ) ) ;
	movlw	07Dh
	movlb 2	; select bank2
	movwf	(_ui16_comm_time_max)^0100h
	clrf	(_ui16_comm_time_max+1)^0100h
	line	1472
;bldc.c: 1472: break;
	goto	l29126
	line	1481
	
l29108:	
;bldc.c: 1479: {
;bldc.c: 1481: ui16_comm_time_max = ( unsigned short ) ( ( 10 * 20000UL ) / ( 1200 * 2 ) ) ;
	movlw	053h
	movlb 2	; select bank2
	movwf	(_ui16_comm_time_max)^0100h
	clrf	(_ui16_comm_time_max+1)^0100h
	line	1482
;bldc.c: 1482: break;
	goto	l29126
	line	1491
	
l29110:	
;bldc.c: 1489: {
;bldc.c: 1491: ui16_comm_time_max = ( unsigned short ) ( ( 10 * 20000UL ) / ( 2400 * 2 ) ) ;
	movlw	029h
	movlb 2	; select bank2
	movwf	(_ui16_comm_time_max)^0100h
	clrf	(_ui16_comm_time_max+1)^0100h
	line	1492
;bldc.c: 1492: break;
	goto	l29126
	line	1501
	
l29112:	
;bldc.c: 1499: {
;bldc.c: 1501: ui16_comm_time_max = ( unsigned short ) ( ( 10 * 20000UL ) / ( 3000 * 2 ) ) ;
	movlw	021h
	movlb 2	; select bank2
	movwf	(_ui16_comm_time_max)^0100h
	clrf	(_ui16_comm_time_max+1)^0100h
	line	1502
;bldc.c: 1502: break;
	goto	l29126
	line	1443
	
l29124:	
	; Switch on 2 bytes has been partitioned into a top level switch of size 1, and 1 sub-switches
; Switch size 1, requested type "space"
; Number of cases is 1, Range of values is 0 to 0
; switch strategies available:
; Name         Instructions Cycles
; simple_byte            4     3 (average)
; direct_byte            8     6 (fixed)
; jumptable            260     6 (fixed)
; rangetable             5     4 (fixed)
; spacedrange            7     6 (fixed)
; locatedrange           1     3 (fixed)
;	Chosen strategy is simple_byte

	movlb 1	; select bank1
	movf (_ui16_step_cnt+1)^080h,w
	opt asmopt_off
	xorlw	0^0	; case 0
	skipnz
	goto	l31326
	goto	l29112
	opt asmopt_on
	
l31326:	
; Switch size 1, requested type "space"
; Number of cases is 10, Range of values is 0 to 9
; switch strategies available:
; Name         Instructions Cycles
; direct_byte           26     6 (fixed)
; simple_byte           31    16 (average)
; jumptable            260     6 (fixed)
; rangetable            14     4 (fixed)
; spacedrange           25     6 (fixed)
; locatedrange          10     3 (fixed)
;	Chosen strategy is direct_byte

	movf (_ui16_step_cnt)^080h,w
	movwf fsr0l
	movlw	10
	subwf	fsr0l,w
skipnc
goto l29112
movlp high(S31328)
	lslf fsr0l,w
	addlw low(S31328)
	movwf pc
psect	swtext2,local,class=CONST,delta=2
global __pswtext2
__pswtext2:
S31328:
	ljmp	l29102
	ljmp	l29104
	ljmp	l29106
	ljmp	l29108
	ljmp	l29110
	ljmp	l29112
	ljmp	l29112
	ljmp	l29112
	ljmp	l29112
	ljmp	l29112
psect	text2038

	line	1559
	
l29126:	
;bldc.c: 1559: if( ui16_comm_time_max < ( unsigned short ) ( ( 10 * 20000UL ) / ( 3000 * 2 ) ) )
	movlw	high(021h)
	subwf	(_ui16_comm_time_max+1)^0100h,w
	movlw	low(021h)
	skipnz
	subwf	(_ui16_comm_time_max)^0100h,w
	skipnc
	goto	u9081
	goto	u9080
u9081:
	goto	l29130
u9080:
	line	1563
	
l29128:	
;bldc.c: 1561: {
;bldc.c: 1563: ui16_comm_time_max = ( unsigned short ) ( ( 10 * 20000UL ) / ( 3000 * 2 ) );
	movlw	021h
	movwf	(_ui16_comm_time_max)^0100h
	clrf	(_ui16_comm_time_max+1)^0100h
	line	1569
	
l29130:	
;bldc.c: 1565: }
;bldc.c: 1569: if (MotorFlags.bits.B4)
	movlb 0	; select bank0
	btfss	(_MotorFlags),4
	goto	u9091
	goto	u9090
u9091:
	goto	l3873
u9090:
	line	1574
	
l29132:	
;bldc.c: 1571: {
;bldc.c: 1574: ui8_StartupPWM = 10 ;
	movlw	(0Ah)
	movlb 2	; select bank2
	movwf	(_ui8_StartupPWM)^0100h
	goto	l3873
	line	1576
	
l3872:	
	line	1579
;bldc.c: 1576: }
;bldc.c: 1579: }
	goto	l3873
	line	1582
	
l3855:	
	line	1587
;bldc.c: 1582: else
;bldc.c: 1583: {
;bldc.c: 1587: if( rising_bemf_flag)
	btfss	(_rising_bemf_flag/8),(_rising_bemf_flag)&7
	goto	u9101
	goto	u9100
u9101:
	goto	l29136
u9100:
	line	1592
	
l29134:	
;bldc.c: 1589: {
;bldc.c: 1592: phase_delay_counter = 0;
	movlb 4	; select bank4
	clrf	(_phase_delay_counter)^0200h
	clrf	(_phase_delay_counter+1)^0200h
	line	1594
;bldc.c: 1594: }
	goto	l29140
	line	1601
	
l29136:	
;bldc.c: 1597: else
;bldc.c: 1598: {
;bldc.c: 1601: phase_delay_counter = comm_time>>1;
	movf	(_comm_time+1),w
	movlb 4	; select bank4
	movwf	(_phase_delay_counter+1)^0200h
	movlb 0	; select bank0
	movf	(_comm_time),w
	movlb 4	; select bank4
	movwf	(_phase_delay_counter)^0200h
	
l29138:	
	lsrf	(_phase_delay_counter+1)^0200h,f
	rrf	(_phase_delay_counter)^0200h,f
	line	1610
	
l29140:	
;bldc.c: 1615: phase_delay_counter_debug = phase_delay_counter;
	clrf	(_phase_delay_counter)^0200h
	clrf	(_phase_delay_counter+1)^0200h
	line	1621
	
l29142:	
;bldc.c: 1621: ui16_comm_time_max = ( unsigned short ) ( ( 10 * 20000UL ) / ( 300 * 2 ) );
	movlw	low(014Dh)
	movlb 2	; select bank2
	movwf	(_ui16_comm_time_max)^0100h
	movlw	high(014Dh)
	movwf	((_ui16_comm_time_max)^0100h)+1
	line	1626
	
l29144:	
;bldc.c: 1626: if( MotorFlags.bits.B6 )
	movlb 0	; select bank0
	btfss	(_MotorFlags),6
	goto	u9111
	goto	u9110
u9111:
	goto	l29164
u9110:
	line	1632
	
l29146:	
;bldc.c: 1628: {
;bldc.c: 1632: if (CCPR1L == ui8_duty_cycle_BLDC)
	movlb 5	; select bank5
	movf	(657)^0280h,w	;volatile
	movlb 0	; select bank0
	xorwf	(_ui8_duty_cycle_BLDC),w
	skipz
	goto	u9121
	goto	u9120
u9121:
	goto	l29150
u9120:
	goto	l3873
	line	1644
	
l29150:	
;bldc.c: 1641: else
;bldc.c: 1642: {
;bldc.c: 1644: if( CCPR1L > ui8_duty_cycle_BLDC )
	movlb 5	; select bank5
	movf	(657)^0280h,w	;volatile
	movlb 0	; select bank0
	subwf	(_ui8_duty_cycle_BLDC),w
	skipnc
	goto	u9131
	goto	u9130
u9131:
	goto	l29158
u9130:
	line	1649
	
l29152:	
;bldc.c: 1647: {
;bldc.c: 1649: CCPR1L--;
	movlb 5	; select bank5
	decf	(657)^0280h,f	;volatile
	line	1651
	
l29154:	
;bldc.c: 1651: if (CCPR1L > ui8_duty_cycle_BLDC)
	movf	(657)^0280h,w	;volatile
	movlb 0	; select bank0
	subwf	(_ui8_duty_cycle_BLDC),w
	skipnc
	goto	u9141
	goto	u9140
u9141:
	goto	l3872
u9140:
	line	1655
	
l29156:	
;bldc.c: 1653: {
;bldc.c: 1655: CCPR1L--;
	movlb 5	; select bank5
	decf	(657)^0280h,f	;volatile
	goto	l3873
	line	1666
	
l29158:	
;bldc.c: 1663: else
;bldc.c: 1664: {
;bldc.c: 1666: CCPR1L++;
	movlb 5	; select bank5
	incf	(657)^0280h,f	;volatile
	line	1668
	
l29160:	
;bldc.c: 1668: if (CCPR1L < ui8_duty_cycle_BLDC)
	movlb 0	; select bank0
	movf	(_ui8_duty_cycle_BLDC),w
	movlb 5	; select bank5
	subwf	(657)^0280h,w	;volatile
	skipnc
	goto	u9151
	goto	u9150
u9151:
	goto	l3872
u9150:
	line	1672
	
l29162:	
;bldc.c: 1670: {
;bldc.c: 1672: CCPR1L++;
	incf	(657)^0280h,f	;volatile
	goto	l3873
	line	1706
	
l29164:	
;bldc.c: 1703: else
;bldc.c: 1704: {
;bldc.c: 1706: CCPR1L = 0;
	movlb 5	; select bank5
	clrf	(657)^0280h	;volatile
	line	1712
	
l3873:	
	line	1715
;bldc.c: 1708: }
;bldc.c: 1712: }
;bldc.c: 1715: comm_time = 0;
	movlb 0	; select bank0
	clrf	(_comm_time)
	clrf	(_comm_time+1)
	line	1719
	
l29166:	
;bldc.c: 1719: if( comm_state == 0xff )
	movf	(_comm_state),w
	xorlw	0FFh&0ffh
	skipz
	goto	u9161
	goto	u9160
u9161:
	goto	l29314
u9160:
	line	1723
	
l29168:	
;bldc.c: 1721: {
;bldc.c: 1723: comm_state = 6;
	movlw	(06h)
	movwf	(_comm_state)
	goto	l29314
	line	1741
	
l29170:	
;bldc.c: 1735: {
;bldc.c: 1741: PSTR1CON = 0b00000001;
	movlw	(01h)
	movlb 5	; select bank5
	movwf	(662)^0280h	;volatile
	line	1744
	
l29172:	
;bldc.c: 1744: if( MotorFlags.bits.B6 )
	movlb 0	; select bank0
	btfss	(_MotorFlags),6
	goto	u9171
	goto	u9170
u9171:
	goto	l29176
u9170:
	line	1748
	
l29174:	
;bldc.c: 1746: {
;bldc.c: 1748: LATC = ( LATC & 0b11100101 ) | 0b00001010;
	movlb 2	; select bank2
	movf	(270)^0100h,w
	andlw	0E5h
	iorlw	0Ah
	movwf	(270)^0100h	;volatile
	line	1754
	
l29176:	
;bldc.c: 1750: }
;bldc.c: 1754: LATB2 = 0;
	movlb 2	; select bank2
	bcf	(2154/8)^0100h,(2154)&7
	line	1755
	
l29178:	
;bldc.c: 1755: LATB1 = 0;
	bcf	(2153/8)^0100h,(2153)&7
	line	1756
	
l29180:	
;bldc.c: 1756: ui8_IPhase_sel = ( 0x02 << 2 ) | 0x01;
	movlw	(09h)
	movwf	(_ui8_IPhase_sel)^0100h
	line	1757
	
l29182:	
;bldc.c: 1757: ui8_UPhase_sel = ( 0x00 << 2 ) | 0x01;
	clrf	(_ui8_UPhase_sel)^0100h
	incf	(_ui8_UPhase_sel)^0100h,f
	line	1758
	
l29184:	
;bldc.c: 1758: ui8_Ubemf_sel = ( 0x09 << 2 ) | 0x01;
	movlw	(025h)
	movwf	(_ui8_Ubemf_sel)^0100h
	line	1759
	
l29186:	
;bldc.c: 1759: CM1CON1 = 0x02;
	movlw	(02h)
	movwf	(274)^0100h	;volatile
	line	1772
	
l29188:	
;bldc.c: 1772: CM1CON0 = 0x84;
	movlw	(084h)
	movwf	(273)^0100h	;volatile
	line	1773
	
l29190:	
;bldc.c: 1773: rising_bemf_flag = 1;
	movlb 0	; select bank0
	bsf	(_rising_bemf_flag/8),(_rising_bemf_flag)&7
	line	1774
	
l29192:	
;bldc.c: 1774: comm_state = 6;
	movlw	(06h)
	movwf	(_comm_state)
	line	1780
;bldc.c: 1780: break;
	goto	l29316
	line	1795
	
l29194:	
;bldc.c: 1790: {
;bldc.c: 1795: PSTR1CON = 0b00000001;
	movlw	(01h)
	movlb 5	; select bank5
	movwf	(662)^0280h	;volatile
	line	1798
	
l29196:	
;bldc.c: 1798: if( MotorFlags.bits.B6 )
	movlb 0	; select bank0
	btfss	(_MotorFlags),6
	goto	u9181
	goto	u9180
u9181:
	goto	l29200
u9180:
	line	1802
	
l29198:	
;bldc.c: 1800: {
;bldc.c: 1802: LATC = ( LATC & 0b11100101 ) | 0b00010010;
	movlb 2	; select bank2
	movf	(270)^0100h,w
	andlw	0E5h
	iorlw	012h
	movwf	(270)^0100h	;volatile
	line	1808
	
l29200:	
;bldc.c: 1804: }
;bldc.c: 1808: LATB2 = 1;
	movlb 2	; select bank2
	bsf	(2154/8)^0100h,(2154)&7
	line	1809
	
l29202:	
;bldc.c: 1809: LATB1 = 0;
	bcf	(2153/8)^0100h,(2153)&7
	line	1810
	
l29204:	
;bldc.c: 1810: ui8_IPhase_sel = ( 0x02 << 2 ) | 0x01;
	movlw	(09h)
	movwf	(_ui8_IPhase_sel)^0100h
	line	1811
	
l29206:	
;bldc.c: 1811: ui8_UPhase_sel = ( 0x00 << 2 ) | 0x01;
	clrf	(_ui8_UPhase_sel)^0100h
	incf	(_ui8_UPhase_sel)^0100h,f
	line	1812
	
l29208:	
;bldc.c: 1812: ui8_Ubemf_sel = ( 0x01 << 2 ) | 0x01;
	movlw	(05h)
	movwf	(_ui8_Ubemf_sel)^0100h
	line	1813
	
l29210:	
;bldc.c: 1813: CM1CON1 = 0x01;
	movlw	(01h)
	movwf	(274)^0100h	;volatile
	line	1827
	
l29212:	
;bldc.c: 1827: CM1CON0 = 0x94;
	movlw	(094h)
	movwf	(273)^0100h	;volatile
	line	1828
	
l29214:	
;bldc.c: 1828: rising_bemf_flag = 0;
	movlb 0	; select bank0
	bcf	(_rising_bemf_flag/8),(_rising_bemf_flag)&7
	line	1829
	
l29216:	
;bldc.c: 1829: comm_state = 1;
	clrf	(_comm_state)
	incf	(_comm_state),f
	line	1837
;bldc.c: 1837: break;
	goto	l29316
	line	1852
	
l29218:	
;bldc.c: 1847: {
;bldc.c: 1852: PSTR1CON = 0b00000010;
	movlw	(02h)
	movlb 5	; select bank5
	movwf	(662)^0280h	;volatile
	line	1855
	
l29220:	
;bldc.c: 1855: if( MotorFlags.bits.B6 )
	movlb 0	; select bank0
	btfss	(_MotorFlags),6
	goto	u9191
	goto	u9190
u9191:
	goto	l29224
u9190:
	line	1859
	
l29222:	
;bldc.c: 1857: {
;bldc.c: 1859: LATC = ( LATC & 0b11100101 ) | 0b00011000;
	movlb 2	; select bank2
	movf	(270)^0100h,w
	andlw	0E5h
	iorlw	018h
	movwf	(270)^0100h	;volatile
	line	1864
	
l29224:	
;bldc.c: 1861: }
;bldc.c: 1864: LATC2 = 0;
	movlb 2	; select bank2
	bcf	(2162/8)^0100h,(2162)&7
	line	1866
	
l29226:	
;bldc.c: 1866: LATB1 = 0;
	bcf	(2153/8)^0100h,(2153)&7
	line	1867
	
l29228:	
;bldc.c: 1867: ui8_IPhase_sel = ( 0x0D << 2 ) | 0x01;
	movlw	(035h)
	movwf	(_ui8_IPhase_sel)^0100h
	line	1868
	
l29230:	
;bldc.c: 1868: ui8_UPhase_sel = ( 0x01 << 2 ) | 0x01;
	movlw	(05h)
	movwf	(_ui8_UPhase_sel)^0100h
	line	1869
	
l29232:	
;bldc.c: 1869: ui8_Ubemf_sel = ( 0x00 << 2 ) | 0x01;
	clrf	(_ui8_Ubemf_sel)^0100h
	incf	(_ui8_Ubemf_sel)^0100h,f
	line	1870
	
l29234:	
;bldc.c: 1870: CM1CON1 = 0x00;
	clrf	(274)^0100h	;volatile
	line	1884
;bldc.c: 1884: CM1CON0 = 0x84;
	movlw	(084h)
	movwf	(273)^0100h	;volatile
	line	1885
	
l29236:	
;bldc.c: 1885: rising_bemf_flag = 1;
	movlb 0	; select bank0
	bsf	(_rising_bemf_flag/8),(_rising_bemf_flag)&7
	line	1886
;bldc.c: 1886: comm_state = 2;
	movlw	(02h)
	movwf	(_comm_state)
	line	1891
;bldc.c: 1891: break;
	goto	l29316
	line	1907
	
l29238:	
;bldc.c: 1901: {
;bldc.c: 1907: PSTR1CON = 0b00000010;
	movlw	(02h)
	movlb 5	; select bank5
	movwf	(662)^0280h	;volatile
	line	1910
	
l29240:	
;bldc.c: 1910: if( MotorFlags.bits.B6 )
	movlb 0	; select bank0
	btfss	(_MotorFlags),6
	goto	u9201
	goto	u9200
u9201:
	goto	l29244
u9200:
	line	1914
	
l29242:	
;bldc.c: 1912: {
;bldc.c: 1914: LATC = ( LATC & 0b11100101 ) | 0b00001010;
	movlb 2	; select bank2
	movf	(270)^0100h,w
	andlw	0E5h
	iorlw	0Ah
	movwf	(270)^0100h	;volatile
	line	1919
	
l29244:	
;bldc.c: 1916: }
;bldc.c: 1919: LATC2 = 0;
	movlb 2	; select bank2
	bcf	(2162/8)^0100h,(2162)&7
	line	1921
	
l29246:	
;bldc.c: 1921: LATB1 = 1;
	bsf	(2153/8)^0100h,(2153)&7
	line	1922
	
l29248:	
;bldc.c: 1922: ui8_IPhase_sel = ( 0x0D << 2 ) | 0x01;
	movlw	(035h)
	movwf	(_ui8_IPhase_sel)^0100h
	line	1923
	
l29250:	
;bldc.c: 1923: ui8_UPhase_sel = ( 0x01 << 2 ) | 0x01;
	movlw	(05h)
	movwf	(_ui8_UPhase_sel)^0100h
	line	1924
	
l29252:	
;bldc.c: 1924: ui8_Ubemf_sel = ( 0x09 << 2 ) | 0x01;
	movlw	(025h)
	movwf	(_ui8_Ubemf_sel)^0100h
	line	1925
	
l29254:	
;bldc.c: 1925: CM1CON1 = 0x02;
	movlw	(02h)
	movwf	(274)^0100h	;volatile
	line	1939
	
l29256:	
;bldc.c: 1939: CM1CON0 = 0x94;
	movlw	(094h)
	movwf	(273)^0100h	;volatile
	line	1940
	
l29258:	
;bldc.c: 1940: rising_bemf_flag = 0;
	movlb 0	; select bank0
	bcf	(_rising_bemf_flag/8),(_rising_bemf_flag)&7
	line	1941
	
l29260:	
;bldc.c: 1941: comm_state = 3;
	movlw	(03h)
	movwf	(_comm_state)
	line	1946
;bldc.c: 1946: break;
	goto	l29316
	line	1961
	
l29262:	
;bldc.c: 1955: {
;bldc.c: 1961: PSTR1CON = 0b00000100;
	movlw	(04h)
	movlb 5	; select bank5
	movwf	(662)^0280h	;volatile
	line	1964
	
l29264:	
;bldc.c: 1964: if( MotorFlags.bits.B6 )
	movlb 0	; select bank0
	btfss	(_MotorFlags),6
	goto	u9211
	goto	u9210
u9211:
	goto	l29268
u9210:
	line	1968
	
l29266:	
;bldc.c: 1966: {
;bldc.c: 1968: LATC = ( LATC & 0b11100101 ) | 0b00010010;
	movlb 2	; select bank2
	movf	(270)^0100h,w
	andlw	0E5h
	iorlw	012h
	movwf	(270)^0100h	;volatile
	line	1973
	
l29268:	
;bldc.c: 1970: }
;bldc.c: 1973: LATC2 = 0;
	movlb 2	; select bank2
	bcf	(2162/8)^0100h,(2162)&7
	line	1974
	
l29270:	
;bldc.c: 1974: LATB2 = 0;
	bcf	(2154/8)^0100h,(2154)&7
	line	1976
	
l29272:	
;bldc.c: 1976: ui8_IPhase_sel = ( 0x04 << 2 ) | 0x01;
	movlw	(011h)
	movwf	(_ui8_IPhase_sel)^0100h
	line	1977
	
l29274:	
;bldc.c: 1977: ui8_UPhase_sel = ( 0x09 << 2 ) | 0x01;
	movlw	(025h)
	movwf	(_ui8_UPhase_sel)^0100h
	line	1978
	
l29276:	
;bldc.c: 1978: ui8_Ubemf_sel = ( 0x01 << 2 ) | 0x01;
	movlw	(05h)
	movwf	(_ui8_Ubemf_sel)^0100h
	line	1979
	
l29278:	
;bldc.c: 1979: CM1CON1 = 0x01;
	movlw	(01h)
	movwf	(274)^0100h	;volatile
	line	1994
	
l29280:	
;bldc.c: 1994: CM1CON0 = 0x84;
	movlw	(084h)
	movwf	(273)^0100h	;volatile
	line	1995
	
l29282:	
;bldc.c: 1995: rising_bemf_flag = 1;
	movlb 0	; select bank0
	bsf	(_rising_bemf_flag/8),(_rising_bemf_flag)&7
	line	1996
	
l29284:	
;bldc.c: 1996: comm_state = 4;
	movlw	(04h)
	movwf	(_comm_state)
	line	2001
;bldc.c: 2001: break;
	goto	l29316
	line	2016
	
l29286:	
;bldc.c: 2011: {
;bldc.c: 2016: PSTR1CON = 0b00000100;
	movlw	(04h)
	movlb 5	; select bank5
	movwf	(662)^0280h	;volatile
	line	2019
	
l29288:	
;bldc.c: 2019: if( MotorFlags.bits.B6 )
	movlb 0	; select bank0
	btfss	(_MotorFlags),6
	goto	u9221
	goto	u9220
u9221:
	goto	l29292
u9220:
	line	2023
	
l29290:	
;bldc.c: 2021: {
;bldc.c: 2023: LATC = ( LATC & 0b11100101 ) | 0b00011000;
	movlb 2	; select bank2
	movf	(270)^0100h,w
	andlw	0E5h
	iorlw	018h
	movwf	(270)^0100h	;volatile
	line	2028
	
l29292:	
;bldc.c: 2025: }
;bldc.c: 2028: LATC2 = 1;
	movlb 2	; select bank2
	bsf	(2162/8)^0100h,(2162)&7
	line	2029
	
l29294:	
;bldc.c: 2029: LATB2 = 0;
	bcf	(2154/8)^0100h,(2154)&7
	line	2031
	
l29296:	
;bldc.c: 2031: ui8_IPhase_sel = ( 0x04 << 2 ) | 0x01;
	movlw	(011h)
	movwf	(_ui8_IPhase_sel)^0100h
	line	2032
	
l29298:	
;bldc.c: 2032: ui8_UPhase_sel = ( 0x09 << 2 ) | 0x01;
	movlw	(025h)
	movwf	(_ui8_UPhase_sel)^0100h
	line	2033
	
l29300:	
;bldc.c: 2033: ui8_Ubemf_sel = ( 0x00 << 2 ) | 0x01;
	clrf	(_ui8_Ubemf_sel)^0100h
	incf	(_ui8_Ubemf_sel)^0100h,f
	line	2034
	
l29302:	
;bldc.c: 2034: CM1CON1 = 0x00;
	clrf	(274)^0100h	;volatile
	line	2048
;bldc.c: 2048: CM1CON0 = 0x94;
	movlw	(094h)
	movwf	(273)^0100h	;volatile
	line	2049
	
l29304:	
;bldc.c: 2049: rising_bemf_flag = 0;
	movlb 0	; select bank0
	bcf	(_rising_bemf_flag/8),(_rising_bemf_flag)&7
	line	2050
;bldc.c: 2050: comm_state = 5;
	movlw	(05h)
	movwf	(_comm_state)
	line	2055
;bldc.c: 2055: break;
	goto	l29316
	line	2066
	
l29306:	
;bldc.c: 2064: {
;bldc.c: 2066: PSTR1CON = 0x00;
	movlb 5	; select bank5
	clrf	(662)^0280h	;volatile
	line	2067
;bldc.c: 2067: CM1CON0 = 0x00;
	movlb 2	; select bank2
	clrf	(273)^0100h	;volatile
	goto	l29214
	line	1728
	
l29314:	
	movf	(_comm_state),w
	; Switch size 1, requested type "space"
; Number of cases is 6, Range of values is 1 to 6
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           19    10 (average)
; direct_byte           21     9 (fixed)
; jumptable            263     9 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	1^0	; case 1
	skipnz
	goto	l29170
	xorlw	2^1	; case 2
	skipnz
	goto	l29194
	xorlw	3^2	; case 3
	skipnz
	goto	l29218
	xorlw	4^3	; case 4
	skipnz
	goto	l29238
	xorlw	5^4	; case 5
	skipnz
	goto	l29262
	xorlw	6^5	; case 6
	skipnz
	goto	l29286
	goto	l29306
	opt asmopt_on

	line	2083
	
l29316:	
;bldc.c: 2081: {
;bldc.c: 2083: ui8_BlankingCount = ( unsigned char ) ( 0.002 * 20000UL );
	movlw	(028h)
	movlb 2	; select bank2
	movwf	(_ui8_BlankingCount)^0100h
	line	2096
;bldc.c: 2085: }
	
l29320:	
;bldc.c: 2093: }
;bldc.c: 2096: comm_time = 0;
	movlb 0	; select bank0
	clrf	(_comm_time)
	clrf	(_comm_time+1)
	line	2099
	
l3902:	
	return
	opt stack 0
GLOBAL	__end_of_commutate
	__end_of_commutate:
;; =============== function _commutate ends ============

	signat	_commutate,88
	global	_Get_Analog_Value
psect	text2039,local,class=CODE,delta=2
global __ptext2039
__ptext2039:

;; *************** function _Get_Analog_Value *****************
;; Defined at:
;;		line 444 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, fsr1l, fsr1h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/3
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_FILTER_Ubat
;;		_FILTER_IPhase
;;		_FILTER_Temp
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2039
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
	line	444
	global	__size_of_Get_Analog_Value
	__size_of_Get_Analog_Value	equ	__end_of_Get_Analog_Value-_Get_Analog_Value
	
_Get_Analog_Value:	
	opt	stack 10
; Regs used in _Get_Analog_Value: [wreg-status,0+pclath+cstack]
	line	446
	
l25932:	
;adc.c: 446: FILTER_Ubat( );
	fcall	_FILTER_Ubat
	line	447
	
l25934:	
;adc.c: 447: FILTER_IPhase( );
	fcall	_FILTER_IPhase
	line	448
	
l25936:	
;adc.c: 448: FILTER_Temp( );
	fcall	_FILTER_Temp
	line	450
	
l1882:	
	return
	opt stack 0
GLOBAL	__end_of_Get_Analog_Value
	__end_of_Get_Analog_Value:
;; =============== function _Get_Analog_Value ends ============

	signat	_Get_Analog_Value,88
	global	_ADC_Wait
psect	text2040,local,class=CODE,delta=2
global __ptext2040
__ptext2040:

;; *************** function _ADC_Wait *****************
;; Defined at:
;;		line 130 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		None
;; Tracked objects:
;;		On entry : 0/1
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_FILTER_Init
;;		_EOL
;;		_I_calibrationInit
;; This function uses a non-reentrant model
;;
psect	text2040
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
	line	130
	global	__size_of_ADC_Wait
	__size_of_ADC_Wait	equ	__end_of_ADC_Wait-_ADC_Wait
	
_ADC_Wait:	
	opt	stack 10
; Regs used in _ADC_Wait: []
	line	133
	
l29068:	
;adc.c: 133: _nop();
	nop
	line	134
;adc.c: 134: _nop();
	nop
	line	135
;adc.c: 135: _nop();
	nop
	line	136
;adc.c: 136: _nop();
	nop
	line	137
;adc.c: 137: _nop();
	nop
	line	138
;adc.c: 138: _nop();
	nop
	line	139
;adc.c: 139: _nop();
	nop
	line	140
;adc.c: 140: _nop();
	nop
	line	142
;adc.c: 142: _nop();
	nop
	line	143
;adc.c: 143: _nop();
	nop
	line	144
;adc.c: 144: _nop();
	nop
	line	145
;adc.c: 145: _nop();
	nop
	line	146
;adc.c: 146: _nop();
	nop
	line	147
;adc.c: 147: _nop();
	nop
	line	148
;adc.c: 148: _nop();
	nop
	line	149
;adc.c: 149: _nop();
	nop
	line	151
;adc.c: 151: _nop();
	nop
	line	152
;adc.c: 152: _nop();
	nop
	line	153
;adc.c: 153: _nop();
	nop
	line	154
;adc.c: 154: _nop();
	nop
	line	155
;adc.c: 155: _nop();
	nop
	line	156
;adc.c: 156: _nop();
	nop
	line	157
;adc.c: 157: _nop();
	nop
	line	158
;adc.c: 158: _nop();
	nop
	line	160
	
l1854:	
	return
	opt stack 0
GLOBAL	__end_of_ADC_Wait
	__end_of_ADC_Wait:
;; =============== function _ADC_Wait ends ============

	signat	_ADC_Wait,88
	global	___tmul
psect	text2041,local,class=CODE,delta=2
global __ptext2041
__ptext2041:

;; *************** function ___tmul *****************
;; Defined at:
;;		line 3 in file "C:\Program Files\HI-TECH Software\PICC\9.81\sources\tmul.c"
;; Parameters:    Size  Location     Type
;;  multiplier      3   12[BANK0 ] unsigned um
;;  multiplicand    3   15[BANK0 ] unsigned um
;; Auto vars:     Size  Location     Type
;;  product         3   18[BANK0 ] unsigned um
;; Return value:  Size  Location     Type
;;                  3   12[BANK0 ] unsigned um
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       6       0       0       0       0       0       0
;;      Locals:         0       3       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       9       0       0       0       0       0       0
;;Total ram usage:        9 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_PWMReadDC
;; This function uses a non-reentrant model
;;
psect	text2041
	file	"C:\Program Files\HI-TECH Software\PICC\9.81\sources\tmul.c"
	line	3
	global	__size_of___tmul
	__size_of___tmul	equ	__end_of___tmul-___tmul
	
___tmul:	
	opt	stack 9
; Regs used in ___tmul: [wreg+status,2+status,0]
	line	4
	
l28736:	
	clrf	(___tmul@product)
	clrf	(___tmul@product+1)
	clrf	(___tmul@product+2)
	line	6
	
l20002:	
	line	7
	btfss	(___tmul@multiplier),(0)&7
	goto	u8751
	goto	u8750
u8751:
	goto	l28740
u8750:
	line	8
	
l28738:	
	movf	(___tmul@multiplicand),w
	addwf	(___tmul@product),f
	movf	(___tmul@multiplicand+1),w
	addwfc	(___tmul@product+1),f
	movf	(___tmul@multiplicand+2),w
	addwfc	(___tmul@product+2),f
	line	9
	
l28740:	
	lslf	(___tmul@multiplicand),f
	rlf	(___tmul@multiplicand+1),f
	rlf	(___tmul@multiplicand+2),f
	line	10
	
l28742:	
	lsrf	(___tmul@multiplier+2),f
	rrf	(___tmul@multiplier+1),f
	rrf	(___tmul@multiplier),f
	line	11
	movf	(___tmul@multiplier+2),w
	iorwf	(___tmul@multiplier+1),w
	iorwf	(___tmul@multiplier),w
	skipz
	goto	u8761
	goto	u8760
u8761:
	goto	l20002
u8760:
	line	12
	
l28744:	
	movf	(___tmul@product),w
	movwf	(?___tmul)
	movf	(___tmul@product+1),w
	movwf	(?___tmul+1)
	movf	(___tmul@product+2),w
	movwf	(?___tmul+2)
	line	13
	
l20005:	
	return
	opt stack 0
GLOBAL	__end_of___tmul
	__end_of___tmul:
;; =============== function ___tmul ends ============

	signat	___tmul,8315
	global	___ltdiv
psect	text2042,local,class=CODE,delta=2
global __ptext2042
__ptext2042:

;; *************** function ___ltdiv *****************
;; Defined at:
;;		line 5 in file "C:\Program Files\HI-TECH Software\PICC\9.81\sources\ltdiv.c"
;; Parameters:    Size  Location     Type
;;  divisor         3   21[BANK0 ] unsigned um
;;  dividend        3   24[BANK0 ] unsigned um
;; Auto vars:     Size  Location     Type
;;  quotient        3   27[BANK0 ] unsigned um
;;  counter         1   30[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  3   21[BANK0 ] unsigned um
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       6       0       0       0       0       0       0
;;      Locals:         0       4       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0      10       0       0       0       0       0       0
;;Total ram usage:       10 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_PWMReadDC
;; This function uses a non-reentrant model
;;
psect	text2042
	file	"C:\Program Files\HI-TECH Software\PICC\9.81\sources\ltdiv.c"
	line	5
	global	__size_of___ltdiv
	__size_of___ltdiv	equ	__end_of___ltdiv-___ltdiv
	
___ltdiv:	
	opt	stack 9
; Regs used in ___ltdiv: [wreg+status,2+status,0]
	line	9
	
l28710:	
	clrf	(___ltdiv@quotient)
	clrf	(___ltdiv@quotient+1)
	clrf	(___ltdiv@quotient+2)
	line	10
	
l28712:	
	movf	(___ltdiv@divisor+2),w
	iorwf	(___ltdiv@divisor+1),w
	iorwf	(___ltdiv@divisor),w
	skipnz
	goto	u8711
	goto	u8710
u8711:
	goto	l28732
u8710:
	line	11
	
l28714:	
	clrf	(___ltdiv@counter)
	incf	(___ltdiv@counter),f
	line	12
	goto	l28718
	line	13
	
l28716:	
	lslf	(___ltdiv@divisor),f
	rlf	(___ltdiv@divisor+1),f
	rlf	(___ltdiv@divisor+2),f
	line	14
	incf	(___ltdiv@counter),f
	line	12
	
l28718:	
	btfss	(___ltdiv@divisor+2),(23)&7
	goto	u8721
	goto	u8720
u8721:
	goto	l28716
u8720:
	line	17
	
l28720:	
	lslf	(___ltdiv@quotient),f
	rlf	(___ltdiv@quotient+1),f
	rlf	(___ltdiv@quotient+2),f
	line	18
	
l28722:	
	movf	(___ltdiv@divisor+2),w
	subwf	(___ltdiv@dividend+2),w
	skipz
	goto	u8735
	movf	(___ltdiv@divisor+1),w
	subwf	(___ltdiv@dividend+1),w
	skipz
	goto	u8735
	movf	(___ltdiv@divisor),w
	subwf	(___ltdiv@dividend),w
u8735:
	skipc
	goto	u8731
	goto	u8730
u8731:
	goto	l28728
u8730:
	line	19
	
l28724:	
	movf	(___ltdiv@divisor),w
	subwf	(___ltdiv@dividend),f
	movf	(___ltdiv@divisor+1),w
	subwfb	(___ltdiv@dividend+1),f
	movf	(___ltdiv@divisor+2),w
	subwfb	(___ltdiv@dividend+2),f
	line	20
	
l28726:	
	bsf	(___ltdiv@quotient)+(0/8),(0)&7
	line	22
	
l28728:	
	lsrf	(___ltdiv@divisor+2),f
	rrf	(___ltdiv@divisor+1),f
	rrf	(___ltdiv@divisor),f
	line	23
	
l28730:	
	decfsz	(___ltdiv@counter),f
	goto	u8741
	goto	u8740
u8741:
	goto	l28720
u8740:
	line	25
	
l28732:	
	movf	(___ltdiv@quotient),w
	movwf	(?___ltdiv)
	movf	(___ltdiv@quotient+1),w
	movwf	(?___ltdiv+1)
	movf	(___ltdiv@quotient+2),w
	movwf	(?___ltdiv+2)
	line	26
	
l19989:	
	return
	opt stack 0
GLOBAL	__end_of___ltdiv
	__end_of___ltdiv:
;; =============== function ___ltdiv ends ============

	signat	___ltdiv,8315
	global	___aldiv
psect	text2043,local,class=CODE,delta=2
global __ptext2043
__ptext2043:

;; *************** function ___aldiv *****************
;; Defined at:
;;		line 5 in file "C:\Program Files\HI-TECH Software\PICC\9.81\sources\aldiv.c"
;; Parameters:    Size  Location     Type
;;  divisor         4   12[BANK0 ] long 
;;  dividend        4   16[BANK0 ] long 
;; Auto vars:     Size  Location     Type
;;  quotient        4   22[BANK0 ] long 
;;  sign            1   21[BANK0 ] unsigned char 
;;  counter         1   20[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  4   12[BANK0 ] long 
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       8       0       0       0       0       0       0
;;      Locals:         0       6       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0      14       0       0       0       0       0       0
;;Total ram usage:       14 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_BVH2_Appl_Layer
;; This function uses a non-reentrant model
;;
psect	text2043
	file	"C:\Program Files\HI-TECH Software\PICC\9.81\sources\aldiv.c"
	line	5
	global	__size_of___aldiv
	__size_of___aldiv	equ	__end_of___aldiv-___aldiv
	
___aldiv:	
	opt	stack 10
; Regs used in ___aldiv: [wreg+status,2+status,0]
	line	9
	
l25532:	
	clrf	(___aldiv@sign)
	line	10
	
l25534:	
	btfss	(___aldiv@divisor+3),7
	goto	u5351
	goto	u5350
u5351:
	goto	l19871
u5350:
	line	11
	
l25536:	
	comf	(___aldiv@divisor),f
	comf	(___aldiv@divisor+1),f
	comf	(___aldiv@divisor+2),f
	comf	(___aldiv@divisor+3),f
	incf	(___aldiv@divisor),f
	skipnz
	incf	(___aldiv@divisor+1),f
	skipnz
	incf	(___aldiv@divisor+2),f
	skipnz
	incf	(___aldiv@divisor+3),f
	line	12
	clrf	(___aldiv@sign)
	incf	(___aldiv@sign),f
	line	13
	
l19871:	
	line	14
	btfss	(___aldiv@dividend+3),7
	goto	u5361
	goto	u5360
u5361:
	goto	l25542
u5360:
	line	15
	
l25538:	
	comf	(___aldiv@dividend),f
	comf	(___aldiv@dividend+1),f
	comf	(___aldiv@dividend+2),f
	comf	(___aldiv@dividend+3),f
	incf	(___aldiv@dividend),f
	skipnz
	incf	(___aldiv@dividend+1),f
	skipnz
	incf	(___aldiv@dividend+2),f
	skipnz
	incf	(___aldiv@dividend+3),f
	line	16
	
l25540:	
	movlw	(01h)
	xorwf	(___aldiv@sign),f
	line	18
	
l25542:	
	clrf	(___aldiv@quotient)
	clrf	(___aldiv@quotient+1)
	clrf	(___aldiv@quotient+2)
	clrf	(___aldiv@quotient+3)
	line	19
	
l25544:	
	movf	(___aldiv@divisor+3),w
	iorwf	(___aldiv@divisor+2),w
	iorwf	(___aldiv@divisor+1),w
	iorwf	(___aldiv@divisor),w
	skipnz
	goto	u5371
	goto	u5370
u5371:
	goto	l25564
u5370:
	line	20
	
l25546:	
	clrf	(___aldiv@counter)
	incf	(___aldiv@counter),f
	line	21
	goto	l25550
	line	22
	
l25548:	
	lslf	(___aldiv@divisor),f
	rlf	(___aldiv@divisor+1),f
	rlf	(___aldiv@divisor+2),f
	rlf	(___aldiv@divisor+3),f
	line	23
	incf	(___aldiv@counter),f
	line	21
	
l25550:	
	btfss	(___aldiv@divisor+3),(31)&7
	goto	u5381
	goto	u5380
u5381:
	goto	l25548
u5380:
	line	26
	
l25552:	
	lslf	(___aldiv@quotient),f
	rlf	(___aldiv@quotient+1),f
	rlf	(___aldiv@quotient+2),f
	rlf	(___aldiv@quotient+3),f
	line	27
	
l25554:	
	movf	(___aldiv@divisor+3),w
	subwf	(___aldiv@dividend+3),w
	skipz
	goto	u5395
	movf	(___aldiv@divisor+2),w
	subwf	(___aldiv@dividend+2),w
	skipz
	goto	u5395
	movf	(___aldiv@divisor+1),w
	subwf	(___aldiv@dividend+1),w
	skipz
	goto	u5395
	movf	(___aldiv@divisor),w
	subwf	(___aldiv@dividend),w
u5395:
	skipc
	goto	u5391
	goto	u5390
u5391:
	goto	l25560
u5390:
	line	28
	
l25556:	
	movf	(___aldiv@divisor),w
	subwf	(___aldiv@dividend),f
	movf	(___aldiv@divisor+1),w
	subwfb	(___aldiv@dividend+1),f
	movf	(___aldiv@divisor+2),w
	subwfb	(___aldiv@dividend+2),f
	movf	(___aldiv@divisor+3),w
	subwfb	(___aldiv@dividend+3),f
	line	29
	
l25558:	
	bsf	(___aldiv@quotient)+(0/8),(0)&7
	line	31
	
l25560:	
	lsrf	(___aldiv@divisor+3),f
	rrf	(___aldiv@divisor+2),f
	rrf	(___aldiv@divisor+1),f
	rrf	(___aldiv@divisor),f
	line	32
	
l25562:	
	decfsz	(___aldiv@counter),f
	goto	u5401
	goto	u5400
u5401:
	goto	l25552
u5400:
	line	34
	
l25564:	
	movf	(___aldiv@sign),w
	skipz
	goto	u5410
	goto	l25568
u5410:
	line	35
	
l25566:	
	comf	(___aldiv@quotient),f
	comf	(___aldiv@quotient+1),f
	comf	(___aldiv@quotient+2),f
	comf	(___aldiv@quotient+3),f
	incf	(___aldiv@quotient),f
	skipnz
	incf	(___aldiv@quotient+1),f
	skipnz
	incf	(___aldiv@quotient+2),f
	skipnz
	incf	(___aldiv@quotient+3),f
	line	36
	
l25568:	
	movf	(___aldiv@quotient+3),w
	movwf	(?___aldiv+3)
	movf	(___aldiv@quotient+2),w
	movwf	(?___aldiv+2)
	movf	(___aldiv@quotient+1),w
	movwf	(?___aldiv+1)
	movf	(___aldiv@quotient),w
	movwf	(?___aldiv)

	line	37
	
l19881:	
	return
	opt stack 0
GLOBAL	__end_of___aldiv
	__end_of___aldiv:
;; =============== function ___aldiv ends ============

	signat	___aldiv,8316
	global	___awdiv
psect	text2044,local,class=CODE,delta=2
global __ptext2044
__ptext2044:

;; *************** function ___awdiv *****************
;; Defined at:
;;		line 5 in file "C:\Program Files\HI-TECH Software\PICC\9.81\sources\awdiv.c"
;; Parameters:    Size  Location     Type
;;  divisor         2   18[BANK0 ] int 
;;  dividend        2   20[BANK0 ] int 
;; Auto vars:     Size  Location     Type
;;  quotient        2   24[BANK0 ] int 
;;  sign            1   23[BANK0 ] unsigned char 
;;  counter         1   22[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  2   18[BANK0 ] int 
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       4       0       0       0       0       0       0
;;      Locals:         0       4       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       8       0       0       0       0       0       0
;;Total ram usage:        8 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2044
	file	"C:\Program Files\HI-TECH Software\PICC\9.81\sources\awdiv.c"
	line	5
	global	__size_of___awdiv
	__size_of___awdiv	equ	__end_of___awdiv-___awdiv
	
___awdiv:	
	opt	stack 11
; Regs used in ___awdiv: [wreg+status,2+status,0]
	line	9
	
l25488:	
	clrf	(___awdiv@sign)
	line	10
	
l25490:	
	btfss	(___awdiv@divisor+1),7
	goto	u5281
	goto	u5280
u5281:
	goto	l25496
u5280:
	line	11
	
l25492:	
	comf	(___awdiv@divisor),f
	comf	(___awdiv@divisor+1),f
	incf	(___awdiv@divisor),f
	skipnz
	incf	(___awdiv@divisor+1),f
	line	12
	
l25494:	
	clrf	(___awdiv@sign)
	incf	(___awdiv@sign),f
	line	14
	
l25496:	
	btfss	(___awdiv@dividend+1),7
	goto	u5291
	goto	u5290
u5291:
	goto	l25502
u5290:
	line	15
	
l25498:	
	comf	(___awdiv@dividend),f
	comf	(___awdiv@dividend+1),f
	incf	(___awdiv@dividend),f
	skipnz
	incf	(___awdiv@dividend+1),f
	line	16
	
l25500:	
	movlw	(01h)
	xorwf	(___awdiv@sign),f
	line	18
	
l25502:	
	clrf	(___awdiv@quotient)
	clrf	(___awdiv@quotient+1)
	line	19
	
l25504:	
	movf	(___awdiv@divisor+1),w
	iorwf	(___awdiv@divisor),w
	skipnz
	goto	u5301
	goto	u5300
u5301:
	goto	l25524
u5300:
	line	20
	
l25506:	
	clrf	(___awdiv@counter)
	incf	(___awdiv@counter),f
	line	21
	goto	l25510
	line	22
	
l25508:	
	lslf	(___awdiv@divisor),f
	rlf	(___awdiv@divisor+1),f
	line	23
	incf	(___awdiv@counter),f
	line	21
	
l25510:	
	btfss	(___awdiv@divisor+1),(15)&7
	goto	u5311
	goto	u5310
u5311:
	goto	l25508
u5310:
	line	26
	
l25512:	
	lslf	(___awdiv@quotient),f
	rlf	(___awdiv@quotient+1),f
	line	27
	
l25514:	
	movf	(___awdiv@divisor+1),w
	subwf	(___awdiv@dividend+1),w
	skipz
	goto	u5325
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),w
u5325:
	skipc
	goto	u5321
	goto	u5320
u5321:
	goto	l25520
u5320:
	line	28
	
l25516:	
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),f
	movf	(___awdiv@divisor+1),w
	subwfb	(___awdiv@dividend+1),f
	line	29
	
l25518:	
	bsf	(___awdiv@quotient)+(0/8),(0)&7
	line	31
	
l25520:	
	lsrf	(___awdiv@divisor+1),f
	rrf	(___awdiv@divisor),f
	line	32
	
l25522:	
	decfsz	(___awdiv@counter),f
	goto	u5331
	goto	u5330
u5331:
	goto	l25512
u5330:
	line	34
	
l25524:	
	movf	(___awdiv@sign),w
	skipz
	goto	u5340
	goto	l25528
u5340:
	line	35
	
l25526:	
	comf	(___awdiv@quotient),f
	comf	(___awdiv@quotient+1),f
	incf	(___awdiv@quotient),f
	skipnz
	incf	(___awdiv@quotient+1),f
	line	36
	
l25528:	
	movf	(___awdiv@quotient+1),w
	movwf	(?___awdiv+1)
	movf	(___awdiv@quotient),w
	movwf	(?___awdiv)
	line	37
	
l19826:	
	return
	opt stack 0
GLOBAL	__end_of___awdiv
	__end_of___awdiv:
;; =============== function ___awdiv ends ============

	signat	___awdiv,8314
	global	___lmul
psect	text2045,local,class=CODE,delta=2
global __ptext2045
__ptext2045:

;; *************** function ___lmul *****************
;; Defined at:
;;		line 3 in file "C:\Program Files\HI-TECH Software\PICC\9.81\sources\lmul.c"
;; Parameters:    Size  Location     Type
;;  multiplier      4   12[BANK0 ] unsigned long 
;;  multiplicand    4   16[BANK0 ] unsigned long 
;; Auto vars:     Size  Location     Type
;;  product         4   20[BANK0 ] unsigned long 
;; Return value:  Size  Location     Type
;;                  4   12[BANK0 ] unsigned long 
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       8       0       0       0       0       0       0
;;      Locals:         0       4       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0      12       0       0       0       0       0       0
;;Total ram usage:       12 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_BVH2_Appl_Layer
;; This function uses a non-reentrant model
;;
psect	text2045
	file	"C:\Program Files\HI-TECH Software\PICC\9.81\sources\lmul.c"
	line	3
	global	__size_of___lmul
	__size_of___lmul	equ	__end_of___lmul-___lmul
	
___lmul:	
	opt	stack 10
; Regs used in ___lmul: [wreg+status,2+status,0]
	line	4
	
l25476:	
	clrf	(___lmul@product)
	clrf	(___lmul@product+1)
	clrf	(___lmul@product+2)
	clrf	(___lmul@product+3)
	line	6
	
l19751:	
	line	7
	btfss	(___lmul@multiplier),(0)&7
	goto	u5261
	goto	u5260
u5261:
	goto	l25480
u5260:
	line	8
	
l25478:	
	movf	(___lmul@multiplicand),w
	addwf	(___lmul@product),f
	movf	(___lmul@multiplicand+1),w
	addwfc	(___lmul@product+1),f
	movf	(___lmul@multiplicand+2),w
	addwfc	(___lmul@product+2),f
	movf	(___lmul@multiplicand+3),w
	addwfc	(___lmul@product+3),f
	line	9
	
l25480:	
	lslf	(___lmul@multiplicand),f
	rlf	(___lmul@multiplicand+1),f
	rlf	(___lmul@multiplicand+2),f
	rlf	(___lmul@multiplicand+3),f
	line	10
	
l25482:	
	lsrf	(___lmul@multiplier+3),f
	rrf	(___lmul@multiplier+2),f
	rrf	(___lmul@multiplier+1),f
	rrf	(___lmul@multiplier),f
	line	11
	movf	(___lmul@multiplier+3),w
	iorwf	(___lmul@multiplier+2),w
	iorwf	(___lmul@multiplier+1),w
	iorwf	(___lmul@multiplier),w
	skipz
	goto	u5271
	goto	u5270
u5271:
	goto	l19751
u5270:
	line	12
	
l25484:	
	movf	(___lmul@product+3),w
	movwf	(?___lmul+3)
	movf	(___lmul@product+2),w
	movwf	(?___lmul+2)
	movf	(___lmul@product+1),w
	movwf	(?___lmul+1)
	movf	(___lmul@product),w
	movwf	(?___lmul)

	line	13
	
l19754:	
	return
	opt stack 0
GLOBAL	__end_of___lmul
	__end_of___lmul:
;; =============== function ___lmul ends ============

	signat	___lmul,8316
	global	___lwdiv
psect	text2046,local,class=CODE,delta=2
global __ptext2046
__ptext2046:

;; *************** function ___lwdiv *****************
;; Defined at:
;;		line 5 in file "C:\Program Files\HI-TECH Software\PICC\9.81\sources\lwdiv.c"
;; Parameters:    Size  Location     Type
;;  divisor         2   12[BANK0 ] unsigned int 
;;  dividend        2   14[BANK0 ] unsigned int 
;; Auto vars:     Size  Location     Type
;;  quotient        2   17[BANK0 ] unsigned int 
;;  counter         1   16[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  2   12[BANK0 ] unsigned int 
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       4       0       0       0       0       0       0
;;      Locals:         0       3       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       7       0       0       0       0       0       0
;;Total ram usage:        7 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_EOL
;; This function uses a non-reentrant model
;;
psect	text2046
	file	"C:\Program Files\HI-TECH Software\PICC\9.81\sources\lwdiv.c"
	line	5
	global	__size_of___lwdiv
	__size_of___lwdiv	equ	__end_of___lwdiv-___lwdiv
	
___lwdiv:	
	opt	stack 10
; Regs used in ___lwdiv: [wreg+status,2+status,0]
	line	9
	
l28684:	
	clrf	(___lwdiv@quotient)
	clrf	(___lwdiv@quotient+1)
	line	10
	
l28686:	
	movf	(___lwdiv@divisor+1),w
	iorwf	(___lwdiv@divisor),w
	skipnz
	goto	u8671
	goto	u8670
u8671:
	goto	l28706
u8670:
	line	11
	
l28688:	
	clrf	(___lwdiv@counter)
	incf	(___lwdiv@counter),f
	line	12
	goto	l28692
	line	13
	
l28690:	
	lslf	(___lwdiv@divisor),f
	rlf	(___lwdiv@divisor+1),f
	line	14
	incf	(___lwdiv@counter),f
	line	12
	
l28692:	
	btfss	(___lwdiv@divisor+1),(15)&7
	goto	u8681
	goto	u8680
u8681:
	goto	l28690
u8680:
	line	17
	
l28694:	
	lslf	(___lwdiv@quotient),f
	rlf	(___lwdiv@quotient+1),f
	line	18
	
l28696:	
	movf	(___lwdiv@divisor+1),w
	subwf	(___lwdiv@dividend+1),w
	skipz
	goto	u8695
	movf	(___lwdiv@divisor),w
	subwf	(___lwdiv@dividend),w
u8695:
	skipc
	goto	u8691
	goto	u8690
u8691:
	goto	l28702
u8690:
	line	19
	
l28698:	
	movf	(___lwdiv@divisor),w
	subwf	(___lwdiv@dividend),f
	movf	(___lwdiv@divisor+1),w
	subwfb	(___lwdiv@dividend+1),f
	line	20
	
l28700:	
	bsf	(___lwdiv@quotient)+(0/8),(0)&7
	line	22
	
l28702:	
	lsrf	(___lwdiv@divisor+1),f
	rrf	(___lwdiv@divisor),f
	line	23
	
l28704:	
	decfsz	(___lwdiv@counter),f
	goto	u8701
	goto	u8700
u8701:
	goto	l28694
u8700:
	line	25
	
l28706:	
	movf	(___lwdiv@quotient+1),w
	movwf	(?___lwdiv+1)
	movf	(___lwdiv@quotient),w
	movwf	(?___lwdiv)
	line	26
	
l19689:	
	return
	opt stack 0
GLOBAL	__end_of___lwdiv
	__end_of___lwdiv:
;; =============== function ___lwdiv ends ============

	signat	___lwdiv,8314
	global	___wmul
psect	text2047,local,class=CODE,delta=2
global __ptext2047
__ptext2047:

;; *************** function ___wmul *****************
;; Defined at:
;;		line 3 in file "C:\Program Files\HI-TECH Software\PICC\9.81\sources\wmul.c"
;; Parameters:    Size  Location     Type
;;  multiplier      2   12[BANK0 ] unsigned int 
;;  multiplicand    2   14[BANK0 ] unsigned int 
;; Auto vars:     Size  Location     Type
;;  product         2   16[BANK0 ] unsigned int 
;; Return value:  Size  Location     Type
;;                  2   12[BANK0 ] unsigned int 
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       4       0       0       0       0       0       0
;;      Locals:         0       2       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       6       0       0       0       0       0       0
;;Total ram usage:        6 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_commutate
;;		_EOL
;;		_main
;;		_BVH2_Appl_Layer
;; This function uses a non-reentrant model
;;
psect	text2047
	file	"C:\Program Files\HI-TECH Software\PICC\9.81\sources\wmul.c"
	line	3
	global	__size_of___wmul
	__size_of___wmul	equ	__end_of___wmul-___wmul
	
___wmul:	
	opt	stack 10
; Regs used in ___wmul: [wreg+status,2+status,0]
	line	4
	
l28668:	
	clrf	(___wmul@product)
	clrf	(___wmul@product+1)
	line	7
	
l28670:	
	btfss	(___wmul@multiplier),(0)&7
	goto	u8651
	goto	u8650
u8651:
	goto	l28674
u8650:
	line	8
	
l28672:	
	movf	(___wmul@multiplicand),w
	addwf	(___wmul@product),f
	movf	(___wmul@multiplicand+1),w
	addwfc	(___wmul@product+1),f
	line	9
	
l28674:	
	lslf	(___wmul@multiplicand),f
	rlf	(___wmul@multiplicand+1),f
	line	10
	
l28676:	
	lsrf	(___wmul@multiplier+1),f
	rrf	(___wmul@multiplier),f
	line	11
	
l28678:	
	movf	((___wmul@multiplier+1)),w
	iorwf	((___wmul@multiplier)),w
	skipz
	goto	u8661
	goto	u8660
u8661:
	goto	l28670
u8660:
	line	12
	
l28680:	
	movf	(___wmul@product+1),w
	movwf	(?___wmul+1)
	movf	(___wmul@product),w
	movwf	(?___wmul)
	line	13
	
l19679:	
	return
	opt stack 0
GLOBAL	__end_of___wmul
	__end_of___wmul:
;; =============== function ___wmul ends ============

	signat	___wmul,8314
	global	_Cb53_UbatHandling_node_fcn1
psect	text2048,local,class=CODE,delta=2
global __ptext2048
__ptext2048:

;; *************** function _Cb53_UbatHandling_node_fcn1 *****************
;; Defined at:
;;		line 1844 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\T_Link\BVH2_Appl_Layer.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/1
;;		On exit  : 1F/1
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_Cb53_UbatHandling_node_fcn2
;; This function uses a non-reentrant model
;;
psect	text2048
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\T_Link\BVH2_Appl_Layer.c"
	line	1844
	global	__size_of_Cb53_UbatHandling_node_fcn1
	__size_of_Cb53_UbatHandling_node_fcn1	equ	__end_of_Cb53_UbatHandling_node_fcn1-_Cb53_UbatHandling_node_fcn1
	
_Cb53_UbatHandling_node_fcn1:	
	opt	stack 9
; Regs used in _Cb53_UbatHandling_node_fcn1: [wreg-fsr0h+status,2+status,0]
	line	1845
	
l25430:	
;BVH2_Appl_Layer.c: 1845: switch (SIBFS_UbatHandling_b.Aux_sflag3) {
	goto	l25432
	line	1846
;BVH2_Appl_Layer.c: 1846: case 4: {
	
l17799:	
	line	1847
;BVH2_Appl_Layer.c: 1847: SIBFS_UbatHandling_b.Cb55_SaturationHigh = 0;
	bcf	(_SIBFS_UbatHandling_b)^080h,4
	line	1848
;BVH2_Appl_Layer.c: 1848: break;
	goto	l17804
	line	1850
;BVH2_Appl_Layer.c: 1849: }
;BVH2_Appl_Layer.c: 1850: case 1: {
	
l17801:	
	line	1851
;BVH2_Appl_Layer.c: 1851: SIBFS_UbatHandling_b.Cb56_SaturationLow = 0;
	bcf	(_SIBFS_UbatHandling_b)^080h,5
	line	1852
;BVH2_Appl_Layer.c: 1852: break;
	goto	l17804
	line	1854
;BVH2_Appl_Layer.c: 1853: }
;BVH2_Appl_Layer.c: 1854: case 2: {
	
l17802:	
	line	1855
;BVH2_Appl_Layer.c: 1855: SIBFS_UbatHandling_b.Cb58_LimpRangeHigh = 0;
	bcf	(_SIBFS_UbatHandling_b)^080h,7
	line	1856
;BVH2_Appl_Layer.c: 1856: break;
	goto	l17804
	line	1858
;BVH2_Appl_Layer.c: 1857: }
;BVH2_Appl_Layer.c: 1858: default: {
	
l17803:	
	line	1859
;BVH2_Appl_Layer.c: 1859: SIBFS_UbatHandling_b.Cb57_NormalUbat = 0;
	bcf	(_SIBFS_UbatHandling_b)^080h,6
	line	1861
;BVH2_Appl_Layer.c: 1860: }
;BVH2_Appl_Layer.c: 1861: }
	goto	l17804
	line	1845
	
l25432:	
	movf	(_SIBFS_UbatHandling_b)^080h,w
	andlw	(1<<3)-1
	; Switch size 1, requested type "space"
; Number of cases is 3, Range of values is 1 to 4
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           10     6 (average)
; direct_byte           17     9 (fixed)
; jumptable            263     9 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	1^0	; case 1
	skipnz
	goto	l17801
	xorlw	2^1	; case 2
	skipnz
	goto	l17802
	xorlw	4^2	; case 4
	skipnz
	goto	l17799
	goto	l17803
	opt asmopt_on

	line	1862
	
l17804:	
	return
	opt stack 0
GLOBAL	__end_of_Cb53_UbatHandling_node_fcn1
	__end_of_Cb53_UbatHandling_node_fcn1:
;; =============== function _Cb53_UbatHandling_node_fcn1 ends ============

	signat	_Cb53_UbatHandling_node_fcn1,88
	global	_Cb1_Current_An___High_node_fcn1
psect	text2049,local,class=CODE,delta=2
global __ptext2049
__ptext2049:

;; *************** function _Cb1_Current_An___High_node_fcn1 *****************
;; Defined at:
;;		line 1641 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\T_Link\BVH2_Appl_Layer.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/2
;;		On exit  : 1F/2
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_BVH2_Appl_Layer
;; This function uses a non-reentrant model
;;
psect	text2049
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\T_Link\BVH2_Appl_Layer.c"
	line	1641
	global	__size_of_Cb1_Current_An___High_node_fcn1
	__size_of_Cb1_Current_An___High_node_fcn1	equ	__end_of_Cb1_Current_An___High_node_fcn1-_Cb1_Current_An___High_node_fcn1
	
_Cb1_Current_An___High_node_fcn1:	
	opt	stack 10
; Regs used in _Cb1_Current_An___High_node_fcn1: [wreg+status,2+status,0]
	line	1644
	
l25412:	
;BVH2_Appl_Layer.c: 1644: if (bool_mat_currAlarm_bldc) {
	movf	(_bool_mat_currAlarm_bldc)^0100h,w
	skipz
	goto	u5180
	goto	l25420
u5180:
	line	1647
	
l25414:	
;BVH2_Appl_Layer.c: 1646: SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High_ns = (unsigned int)
;BVH2_Appl_Layer.c: 1647: (UInt8)2;
	movf	(_SIBFS_Current_Analysis_High_b)^0100h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(02h & ((1<<3)-1))<<0
	movwf	(_SIBFS_Current_Analysis_High_b)^0100h
	line	1648
	
l25416:	
;BVH2_Appl_Layer.c: 1648: Cb1_StateCnt = 0 ;
	clrf	(_Cb1_StateCnt)^0100h
	clrf	(_Cb1_StateCnt+1)^0100h
	line	1649
	
l25418:	
;BVH2_Appl_Layer.c: 1649: SIBFS_Current_Analysis_High_b.Cb1_glflag = 3 ;
	movlw	(03h & ((1<<2)-1))<<3
	iorwf	(_SIBFS_Current_Analysis_High_b)^0100h,f
	line	1650
;BVH2_Appl_Layer.c: 1650: }
	goto	l17766
	line	1652
	
l25420:	
;BVH2_Appl_Layer.c: 1651: else {
;BVH2_Appl_Layer.c: 1652: if (Cb1_StateCnt > 100) {
	movlw	high(065h)
	subwf	(_Cb1_StateCnt+1)^0100h,w
	movlw	low(065h)
	skipnz
	subwf	(_Cb1_StateCnt)^0100h,w
	skipc
	goto	u5191
	goto	u5190
u5191:
	goto	l25428
u5190:
	line	1655
	
l25422:	
;BVH2_Appl_Layer.c: 1654: SIBFS_Current_Analysis_High_b.Cb1_Current_Analysis_High_ns = (unsigned int)
;BVH2_Appl_Layer.c: 1655: (UInt8)5;
	movf	(_SIBFS_Current_Analysis_High_b)^0100h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(05h & ((1<<3)-1))<<0
	movwf	(_SIBFS_Current_Analysis_High_b)^0100h
	line	1656
	
l25424:	
;BVH2_Appl_Layer.c: 1656: Cb1_oShutoff = 0;
	clrf	(_Cb1_oShutoff)^0100h
	line	1657
	
l25426:	
;BVH2_Appl_Layer.c: 1657: Cb1_oCurrentAlarm = 0;
	clrf	(_Cb1_oCurrentAlarm)^0100h
	line	1658
;BVH2_Appl_Layer.c: 1658: SIBFS_Current_Analysis_High_b.Cb1_glflag = 3 ;
	movlw	(03h & ((1<<2)-1))<<3
	iorwf	(_SIBFS_Current_Analysis_High_b)^0100h,f
	line	1659
;BVH2_Appl_Layer.c: 1659: }
	goto	l17766
	line	1661
	
l25428:	
;BVH2_Appl_Layer.c: 1660: else {
;BVH2_Appl_Layer.c: 1661: SIBFS_Current_Analysis_High_b.Cb1_glflag = 1 ;
	movf	(_SIBFS_Current_Analysis_High_b)^0100h,w
	andlw	not (((1<<2)-1)<<3)
	iorlw	(01h & ((1<<2)-1))<<3
	movwf	(_SIBFS_Current_Analysis_High_b)^0100h
	line	1664
	
l17766:	
	return
	opt stack 0
GLOBAL	__end_of_Cb1_Current_An___High_node_fcn1
	__end_of_Cb1_Current_An___High_node_fcn1:
;; =============== function _Cb1_Current_An___High_node_fcn1 ends ============

	signat	_Cb1_Current_An___High_node_fcn1,88
	global	_Cb27_PWM_Detection_node_fcn1
psect	text2050,local,class=CODE,delta=2
global __ptext2050
__ptext2050:

;; *************** function _Cb27_PWM_Detection_node_fcn1 *****************
;; Defined at:
;;		line 1684 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\T_Link\BVH2_Appl_Layer.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/2
;;		On exit  : 1C/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_BVH2_Appl_Layer
;; This function uses a non-reentrant model
;;
psect	text2050
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\T_Link\BVH2_Appl_Layer.c"
	line	1684
	global	__size_of_Cb27_PWM_Detection_node_fcn1
	__size_of_Cb27_PWM_Detection_node_fcn1	equ	__end_of_Cb27_PWM_Detection_node_fcn1-_Cb27_PWM_Detection_node_fcn1
	
_Cb27_PWM_Detection_node_fcn1:	
	opt	stack 10
; Regs used in _Cb27_PWM_Detection_node_fcn1: [wreg+status,2+status,0]
	line	1685
	
l25296:	
;BVH2_Appl_Layer.c: 1685: if ((Cb27_idPWM < 1) || (Cb27_idPWM > 199)) {
	movlb 1	; select bank1
	movf	(_Cb27_idPWM)^080h,w
	skipz
	goto	u5080
	goto	l25300
u5080:
	
l25298:	
	movlw	(0C8h)
	subwf	(_Cb27_idPWM)^080h,w
	skipc
	goto	u5091
	goto	u5090
u5091:
	goto	l25312
u5090:
	line	1687
	
l25300:	
;BVH2_Appl_Layer.c: 1687: SIBFS_PWM_Detection_b.Cb28_PWMinput_handling_ns = (unsigned int) (UInt8)7;
	movlb 2	; select bank2
	movf	(_SIBFS_PWM_Detection_b)^0100h,w
	andlw	not (((1<<4)-1)<<0)
	iorlw	(07h & ((1<<4)-1))<<0
	movwf	(_SIBFS_PWM_Detection_b)^0100h
	line	1688
	
l25302:	
;BVH2_Appl_Layer.c: 1688: Cb27_odPumpOff = 0;
	movlb 3	; select bank3
	clrf	(_Cb27_odPumpOff)^0180h
	line	1689
	
l25304:	
;BVH2_Appl_Layer.c: 1689: Cb27_odFixedValueSel = 1;
	clrf	(_Cb27_odFixedValueSel)^0180h
	incf	(_Cb27_odFixedValueSel)^0180h,f
	line	1690
	
l25306:	
;BVH2_Appl_Layer.c: 1690: Cb27_odFixedLowValueSel = 0;
	clrf	(_Cb27_odFixedLowValueSel)^0180h
	line	1691
	
l25308:	
;BVH2_Appl_Layer.c: 1691: Cb27_oPWM_SC_Alarm = 0;
	clrf	(_Cb27_oPWM_SC_Alarm)^0180h
	line	1692
	
l25310:	
;BVH2_Appl_Layer.c: 1692: Cb27_oPWM_Alarm = 1;
	movlb 2	; select bank2
	clrf	(_Cb27_oPWM_Alarm)^0100h
	incf	(_Cb27_oPWM_Alarm)^0100h,f
	line	1693
;BVH2_Appl_Layer.c: 1693: }
	goto	l17788
	line	1697
	
l25312:	
;BVH2_Appl_Layer.c: 1694: else {
;BVH2_Appl_Layer.c: 1697: if ((ui16_PWM_Freq_mat < 36000) || (ui16_PWM_Freq_mat > 44000)) {
	movlw	high(08CA0h)
	movlb 3	; select bank3
	subwf	(_ui16_PWM_Freq_mat+1)^0180h,w
	movlw	low(08CA0h)
	skipnz
	subwf	(_ui16_PWM_Freq_mat)^0180h,w
	skipc
	goto	u5101
	goto	u5100
u5101:
	goto	l25316
u5100:
	
l25314:	
	movlw	high(0ABE1h)
	subwf	(_ui16_PWM_Freq_mat+1)^0180h,w
	movlw	low(0ABE1h)
	skipnz
	subwf	(_ui16_PWM_Freq_mat)^0180h,w
	skipc
	goto	u5111
	goto	u5110
u5111:
	goto	l25328
u5110:
	line	1699
	
l25316:	
;BVH2_Appl_Layer.c: 1699: SIBFS_PWM_Detection_b.Cb28_PWMinput_handling_ns = (unsigned int) (UInt8)6;
	movlb 2	; select bank2
	movf	(_SIBFS_PWM_Detection_b)^0100h,w
	andlw	not (((1<<4)-1)<<0)
	iorlw	(06h & ((1<<4)-1))<<0
	movwf	(_SIBFS_PWM_Detection_b)^0100h
	line	1700
	
l25318:	
;BVH2_Appl_Layer.c: 1700: Cb27_odPumpOff = 0;
	movlb 3	; select bank3
	clrf	(_Cb27_odPumpOff)^0180h
	line	1701
	
l25320:	
;BVH2_Appl_Layer.c: 1701: Cb27_odFixedValueSel = 1;
	clrf	(_Cb27_odFixedValueSel)^0180h
	incf	(_Cb27_odFixedValueSel)^0180h,f
	line	1702
	
l25322:	
;BVH2_Appl_Layer.c: 1702: Cb27_odFixedLowValueSel = 0;
	clrf	(_Cb27_odFixedLowValueSel)^0180h
	line	1703
	
l25324:	
;BVH2_Appl_Layer.c: 1703: Cb27_oPWM_SC_Alarm = 1;
	clrf	(_Cb27_oPWM_SC_Alarm)^0180h
	incf	(_Cb27_oPWM_SC_Alarm)^0180h,f
	line	1704
	
l25326:	
;BVH2_Appl_Layer.c: 1704: Cb27_oPWM_Alarm = 0;
	movlb 2	; select bank2
	clrf	(_Cb27_oPWM_Alarm)^0100h
	line	1705
;BVH2_Appl_Layer.c: 1705: }
	goto	l17788
	line	1707
	
l25328:	
;BVH2_Appl_Layer.c: 1706: else {
;BVH2_Appl_Layer.c: 1707: if (Cb27_idPWM <= 5) {
	movlw	(06h)
	movlb 1	; select bank1
	subwf	(_Cb27_idPWM)^080h,w
	skipnc
	goto	u5121
	goto	u5120
u5121:
	goto	l25342
u5120:
	line	1709
	
l25330:	
;BVH2_Appl_Layer.c: 1709: SIBFS_PWM_Detection_b.Cb28_PWMinput_handling_ns = (unsigned int) (UInt8)3;
	movlb 2	; select bank2
	movf	(_SIBFS_PWM_Detection_b)^0100h,w
	andlw	not (((1<<4)-1)<<0)
	iorlw	(03h & ((1<<4)-1))<<0
	movwf	(_SIBFS_PWM_Detection_b)^0100h
	goto	l25318
	line	1717
	
l25342:	
;BVH2_Appl_Layer.c: 1716: else {
;BVH2_Appl_Layer.c: 1717: if (Cb27_idPWM < 9) {
	movlw	(09h)
	subwf	(_Cb27_idPWM)^080h,w
	skipnc
	goto	u5131
	goto	u5130
u5131:
	goto	l25356
u5130:
	line	1721
	
l25344:	
;BVH2_Appl_Layer.c: 1720: SIBFS_PWM_Detection_b.Cb28_PWMinput_handling_ns = (unsigned int)
;BVH2_Appl_Layer.c: 1721: (UInt8)8;
	movlb 2	; select bank2
	movf	(_SIBFS_PWM_Detection_b)^0100h,w
	andlw	not (((1<<4)-1)<<0)
	iorlw	(08h & ((1<<4)-1))<<0
	movwf	(_SIBFS_PWM_Detection_b)^0100h
	line	1722
	
l25346:	
;BVH2_Appl_Layer.c: 1722: Cb27_odPumpOff = 1;
	movlb 3	; select bank3
	clrf	(_Cb27_odPumpOff)^0180h
	incf	(_Cb27_odPumpOff)^0180h,f
	line	1723
	
l25348:	
;BVH2_Appl_Layer.c: 1723: Cb27_odFixedValueSel = 0;
	clrf	(_Cb27_odFixedValueSel)^0180h
	goto	l25322
	line	1729
	
l25356:	
;BVH2_Appl_Layer.c: 1728: else {
;BVH2_Appl_Layer.c: 1729: if (Cb27_idPWM > 191) {
	movlw	(0C0h)
	subwf	(_Cb27_idPWM)^080h,w
	skipc
	goto	u5141
	goto	u5140
u5141:
	goto	l25370
u5140:
	line	1733
	
l25358:	
;BVH2_Appl_Layer.c: 1732: SIBFS_PWM_Detection_b.Cb28_PWMinput_handling_ns = (unsigned int)
;BVH2_Appl_Layer.c: 1733: (UInt8)4;
	movlb 2	; select bank2
	movf	(_SIBFS_PWM_Detection_b)^0100h,w
	andlw	not (((1<<4)-1)<<0)
	iorlw	(04h & ((1<<4)-1))<<0
	movwf	(_SIBFS_PWM_Detection_b)^0100h
	goto	l25318
	line	1741
	
l25370:	
;BVH2_Appl_Layer.c: 1740: else {
;BVH2_Appl_Layer.c: 1741: if (Cb27_idPWM >= 23) {
	movlw	(017h)
	subwf	(_Cb27_idPWM)^080h,w
	skipc
	goto	u5151
	goto	u5150
u5151:
	goto	l25384
u5150:
	line	1745
	
l25372:	
;BVH2_Appl_Layer.c: 1744: SIBFS_PWM_Detection_b.Cb28_PWMinput_handling_ns = (unsigned int)
;BVH2_Appl_Layer.c: 1745: (UInt8)2;
	movlb 2	; select bank2
	movf	(_SIBFS_PWM_Detection_b)^0100h,w
	andlw	not (((1<<4)-1)<<0)
	iorlw	(02h & ((1<<4)-1))<<0
	movwf	(_SIBFS_PWM_Detection_b)^0100h
	line	1746
	
l25374:	
;BVH2_Appl_Layer.c: 1746: Cb27_odPumpOff = 0;
	movlb 3	; select bank3
	clrf	(_Cb27_odPumpOff)^0180h
	line	1747
	
l25376:	
;BVH2_Appl_Layer.c: 1747: Cb27_odFixedLowValueSel = 0;
	clrf	(_Cb27_odFixedLowValueSel)^0180h
	line	1748
	
l25378:	
;BVH2_Appl_Layer.c: 1748: Cb27_odFixedValueSel = 0;
	clrf	(_Cb27_odFixedValueSel)^0180h
	line	1749
	
l25380:	
;BVH2_Appl_Layer.c: 1749: Cb27_oPWM_SC_Alarm = 0;
	clrf	(_Cb27_oPWM_SC_Alarm)^0180h
	goto	l25326
	line	1753
	
l25384:	
;BVH2_Appl_Layer.c: 1752: else {
;BVH2_Appl_Layer.c: 1753: if (Cb27_idPWM > 19) {
	movlw	(014h)
	subwf	(_Cb27_idPWM)^080h,w
	skipc
	goto	u5161
	goto	u5160
u5161:
	goto	l25398
u5160:
	line	1757
	
l25386:	
;BVH2_Appl_Layer.c: 1756: SIBFS_PWM_Detection_b.Cb28_PWMinput_handling_ns = (unsigned int)
;BVH2_Appl_Layer.c: 1757: (UInt8)5;
	movlb 2	; select bank2
	movf	(_SIBFS_PWM_Detection_b)^0100h,w
	andlw	not (((1<<4)-1)<<0)
	iorlw	(05h & ((1<<4)-1))<<0
	movwf	(_SIBFS_PWM_Detection_b)^0100h
	line	1758
	
l25388:	
;BVH2_Appl_Layer.c: 1758: Cb27_odPumpOff = 0;
	movlb 3	; select bank3
	clrf	(_Cb27_odPumpOff)^0180h
	line	1759
	
l25390:	
;BVH2_Appl_Layer.c: 1759: Cb27_odFixedValueSel = 0;
	clrf	(_Cb27_odFixedValueSel)^0180h
	line	1760
	
l25392:	
;BVH2_Appl_Layer.c: 1760: Cb27_odFixedLowValueSel = 1;
	clrf	(_Cb27_odFixedLowValueSel)^0180h
	incf	(_Cb27_odFixedLowValueSel)^0180h,f
	goto	l25380
	line	1765
	
l25398:	
;BVH2_Appl_Layer.c: 1764: else {
;BVH2_Appl_Layer.c: 1765: if (Cb27_idPWM <= 19) {
	movlw	(014h)
	subwf	(_Cb27_idPWM)^080h,w
	skipnc
	goto	u5171
	goto	u5170
u5171:
	goto	l17782
u5170:
	line	1769
	
l25400:	
;BVH2_Appl_Layer.c: 1768: SIBFS_PWM_Detection_b.Cb28_PWMinput_handling_ns = (unsigned int)
;BVH2_Appl_Layer.c: 1769: (UInt8)1;
	movlb 2	; select bank2
	movf	(_SIBFS_PWM_Detection_b)^0100h,w
	andlw	not (((1<<4)-1)<<0)
	iorlw	(01h & ((1<<4)-1))<<0
	movwf	(_SIBFS_PWM_Detection_b)^0100h
	line	1770
	
l25402:	
;BVH2_Appl_Layer.c: 1770: Cb27_odPumpOff = 1;
	movlb 3	; select bank3
	clrf	(_Cb27_odPumpOff)^0180h
	incf	(_Cb27_odPumpOff)^0180h,f
	line	1771
	
l25404:	
;BVH2_Appl_Layer.c: 1771: Cb27_odFixedValueSel = 0;
	clrf	(_Cb27_odFixedValueSel)^0180h
	line	1772
	
l25406:	
;BVH2_Appl_Layer.c: 1772: Cb27_odFixedLowValueSel = 0;
	clrf	(_Cb27_odFixedLowValueSel)^0180h
	goto	l25380
	line	1778
	
l17782:	
	line	1783
	
l17788:	
	return
	opt stack 0
GLOBAL	__end_of_Cb27_PWM_Detection_node_fcn1
	__end_of_Cb27_PWM_Detection_node_fcn1:
;; =============== function _Cb27_PWM_Detection_node_fcn1 ends ============

	signat	_Cb27_PWM_Detection_node_fcn1,88
	global	_Cb37_Pic_etat_monitor_node_fcn1
psect	text2051,local,class=CODE,delta=2
global __ptext2051
__ptext2051:

;; *************** function _Cb37_Pic_etat_monitor_node_fcn1 *****************
;; Defined at:
;;		line 1803 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\T_Link\BVH2_Appl_Layer.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/2
;;		On exit  : 1F/2
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_BVH2_Appl_Layer
;; This function uses a non-reentrant model
;;
psect	text2051
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\T_Link\BVH2_Appl_Layer.c"
	line	1803
	global	__size_of_Cb37_Pic_etat_monitor_node_fcn1
	__size_of_Cb37_Pic_etat_monitor_node_fcn1	equ	__end_of_Cb37_Pic_etat_monitor_node_fcn1-_Cb37_Pic_etat_monitor_node_fcn1
	
_Cb37_Pic_etat_monitor_node_fcn1:	
	opt	stack 10
; Regs used in _Cb37_Pic_etat_monitor_node_fcn1: [wreg+status,2+status,0]
	line	1806
	
l25278:	
	line	1813
;BVH2_Appl_Layer.c: 1806: if (bool_mat_pic_etat) {
	
l25286:	
;BVH2_Appl_Layer.c: 1812: else {
;BVH2_Appl_Layer.c: 1813: if (Cb37_StateCnt > 5) {
	movlw	high(06h)
	subwf	(_Cb37_StateCnt+1)^0100h,w
	movlw	low(06h)
	skipnz
	subwf	(_Cb37_StateCnt)^0100h,w
	skipc
	goto	u5071
	goto	u5070
u5071:
	goto	l25294
u5070:
	line	1815
	
l25288:	
;BVH2_Appl_Layer.c: 1815: SIBFS_Pic_etat_monitor_b.Cb37_Pic_etat_monitor_ns = (unsigned int) (UInt8)3;
	movf	(_SIBFS_Pic_etat_monitor_b)^0100h,w
	andlw	not (((1<<3)-1)<<0)
	iorlw	(03h & ((1<<3)-1))<<0
	movwf	(_SIBFS_Pic_etat_monitor_b)^0100h
	line	1816
	
l25290:	
;BVH2_Appl_Layer.c: 1817: Cb37_oAlarm = 0;
	movlb 3	; select bank3
	clrf	(_Cb37_oShutoff)^0180h
	line	1818
	
l25292:	
;BVH2_Appl_Layer.c: 1818: SIBFS_Pic_etat_monitor_b.Cb37_glflag = 3 ;
	movlw	(03h & ((1<<2)-1))<<3
	movlb 2	; select bank2
	iorwf	(_SIBFS_Pic_etat_monitor_b)^0100h,f
	line	1819
;BVH2_Appl_Layer.c: 1819: }
	goto	l17795
	line	1821
	
l25294:	
;BVH2_Appl_Layer.c: 1820: else {
;BVH2_Appl_Layer.c: 1821: SIBFS_Pic_etat_monitor_b.Cb37_glflag = 1 ;
	movf	(_SIBFS_Pic_etat_monitor_b)^0100h,w
	andlw	not (((1<<2)-1)<<3)
	iorlw	(01h & ((1<<2)-1))<<3
	movwf	(_SIBFS_Pic_etat_monitor_b)^0100h
	line	1824
	
l17795:	
	return
	opt stack 0
GLOBAL	__end_of_Cb37_Pic_etat_monitor_node_fcn1
	__end_of_Cb37_Pic_etat_monitor_node_fcn1:
;; =============== function _Cb37_Pic_etat_monitor_node_fcn1 ends ============

	signat	_Cb37_Pic_etat_monitor_node_fcn1,88
	global	_clear_timer
psect	text2052,local,class=CODE,delta=2
global __ptext2052
__ptext2052:

;; *************** function _clear_timer *****************
;; Defined at:
;;		line 304 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\timer.c"
;; Parameters:    Size  Location     Type
;;  ui8_TmrNb       1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  ui8_TmrNb       1   12[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 17/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       1       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       1       0       0       0       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_main
;;		_timer_init
;; This function uses a non-reentrant model
;;
psect	text2052
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\timer.c"
	line	304
	global	__size_of_clear_timer
	__size_of_clear_timer	equ	__end_of_clear_timer-_clear_timer
	
_clear_timer:	
	opt	stack 11
; Regs used in _clear_timer: [wreg-fsr0h+status,2+status,0]
;clear_timer@ui8_TmrNb stored from wreg
	movwf	(clear_timer@ui8_TmrNb)
	line	306
	
l25262:	
;timer.c: 306: switch( ui8_TmrNb )
	goto	l25276
	line	314
	
l25264:	
;timer.c: 312: {
;timer.c: 314: TMR0 = 0;
	clrf	(21)	;volatile
	line	315
;timer.c: 315: break;
	goto	l19653
	line	324
	
l25266:	
;timer.c: 322: {
;timer.c: 324: TMR1L = 0x00;
	clrf	(22)	;volatile
	line	325
;timer.c: 325: TMR1H = 0x00;
	clrf	(23)	;volatile
	line	326
;timer.c: 326: break;
	goto	l19653
	line	335
	
l25268:	
;timer.c: 333: {
;timer.c: 335: TMR2 = 0;
	clrf	(26)	;volatile
	line	336
;timer.c: 336: break;
	goto	l19653
	line	345
	
l25270:	
;timer.c: 343: {
;timer.c: 345: TMR4 = 0;
	movlb 8	; select bank8
	clrf	(1045)^0400h	;volatile
	line	346
;timer.c: 346: break;
	goto	l19653
	line	355
	
l25272:	
;timer.c: 353: {
;timer.c: 355: TMR6 = 0;
	movlb 8	; select bank8
	clrf	(1052)^0400h	;volatile
	line	356
;timer.c: 356: break;
	goto	l19653
	line	306
	
l25276:	
	movf	(clear_timer@ui8_TmrNb),w
	; Switch size 1, requested type "space"
; Number of cases is 5, Range of values is 0 to 6
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           16     9 (average)
; direct_byte           20     6 (fixed)
; jumptable            260     6 (fixed)
; rangetable            11     4 (fixed)
; spacedrange           19     6 (fixed)
; locatedrange           7     3 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	0^0	; case 0
	skipnz
	goto	l25264
	xorlw	1^0	; case 1
	skipnz
	goto	l25266
	xorlw	2^1	; case 2
	skipnz
	goto	l25268
	xorlw	4^2	; case 4
	skipnz
	goto	l25270
	xorlw	6^4	; case 6
	skipnz
	goto	l25272
	goto	l19653
	opt asmopt_on

	line	373
	
l19653:	
	return
	opt stack 0
GLOBAL	__end_of_clear_timer
	__end_of_clear_timer:
;; =============== function _clear_timer ends ============

	signat	_clear_timer,4216
	global	_PWM_Capture_init
psect	text2053,local,class=CODE,delta=2
global __ptext2053
__ptext2053:

;; *************** function _PWM_Capture_init *****************
;; Defined at:
;;		line 80 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\pwm.c"
;; Parameters:    Size  Location     Type
;;  ui8_CCP_Nb      1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  ui8_CCP_Nb      1   12[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0
;; Tracked objects:
;;		On entry : 16/1
;;		On exit  : 18/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       1       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       1       0       0       0       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_system_init
;; This function uses a non-reentrant model
;;
psect	text2053
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\pwm.c"
	line	80
	global	__size_of_PWM_Capture_init
	__size_of_PWM_Capture_init	equ	__end_of_PWM_Capture_init-_PWM_Capture_init
	
_PWM_Capture_init:	
	opt	stack 10
; Regs used in _PWM_Capture_init: [wreg-fsr0h+status,2+status,0]
;PWM_Capture_init@ui8_CCP_Nb stored from wreg
	line	83
	movlb 0	; select bank0
	movwf	(PWM_Capture_init@ui8_CCP_Nb)
	
l25216:	
;pwm.c: 83: ui8_PWMinDC_sav = 0;
	movlb 2	; select bank2
	clrf	(_ui8_PWMinDC_sav)^0100h
	line	84
;pwm.c: 84: ui8_PWMin_failCnt = 0;
	clrf	(_ui8_PWMin_failCnt)^0100h
	line	93
;pwm.c: 93: switch( ui8_CCP_Nb )
	goto	l25260
	line	101
	
l25218:	
;pwm.c: 99: {
;pwm.c: 101: CCP1CON = 0x00;
	movlb 5	; select bank5
	clrf	(659)^0280h	;volatile
	line	102
;pwm.c: 102: CCPR1L = 0x00;
	clrf	(657)^0280h	;volatile
	line	103
;pwm.c: 103: CCPR1H = 0x00;
	clrf	(658)^0280h	;volatile
	line	104
	
l25220:	
;pwm.c: 104: CCP1IE = 1;
	movlb 1	; select bank1
	bsf	(1162/8)^080h,(1162)&7
	line	105
	
l25222:	
;pwm.c: 105: CCP1IF = 0;
	movlb 0	; select bank0
	bcf	(138/8),(138)&7
	line	106
	
l25224:	
;pwm.c: 106: CCP1CON = 0x05;
	movlw	(05h)
	movlb 5	; select bank5
	movwf	(659)^0280h	;volatile
	line	107
;pwm.c: 107: break;
	goto	l17328
	line	116
	
l25226:	
;pwm.c: 114: {
;pwm.c: 116: CCP2CON = 0x00;
	movlb 5	; select bank5
	clrf	(666)^0280h	;volatile
	line	117
;pwm.c: 117: CCPR2L = 0x00;
	clrf	(664)^0280h	;volatile
	line	118
;pwm.c: 118: CCPR2H = 0x00;
	clrf	(665)^0280h	;volatile
	line	119
	
l25228:	
;pwm.c: 119: CCP2IE = 1;
	movlb 1	; select bank1
	bsf	(1168/8)^080h,(1168)&7
	line	120
	
l25230:	
;pwm.c: 120: CCP2IF = 0;
	movlb 0	; select bank0
	bcf	(144/8),(144)&7
	line	121
	
l25232:	
;pwm.c: 121: CCP2CON = 0x05;
	movlw	(05h)
	movlb 5	; select bank5
	movwf	(666)^0280h	;volatile
	line	122
;pwm.c: 122: break;
	goto	l17328
	line	131
	
l25234:	
;pwm.c: 129: {
;pwm.c: 131: CCP3CON = 0x00;
	movlb 6	; select bank6
	clrf	(787)^0300h	;volatile
	line	132
;pwm.c: 132: CCPR3L = 0x00;
	clrf	(785)^0300h	;volatile
	line	133
;pwm.c: 133: CCPR3H = 0x00;
	clrf	(786)^0300h	;volatile
	line	134
	
l25236:	
;pwm.c: 134: CCP3IE = 1;
	movlb 1	; select bank1
	bsf	(1180/8)^080h,(1180)&7
	line	135
	
l25238:	
;pwm.c: 135: CCP3IF = 0;
	movlb 0	; select bank0
	bcf	(156/8),(156)&7
	line	136
	
l25240:	
;pwm.c: 136: CCP3CON = 0x05;
	movlw	(05h)
	movlb 6	; select bank6
	movwf	(787)^0300h	;volatile
	line	137
;pwm.c: 137: break;
	goto	l17328
	line	146
	
l25242:	
;pwm.c: 144: {
;pwm.c: 146: CCP4CON = 0x00;
	movlb 6	; select bank6
	clrf	(794)^0300h	;volatile
	line	147
;pwm.c: 147: CCPR4L = 0x00;
	clrf	(792)^0300h	;volatile
	line	148
;pwm.c: 148: CCPR4H = 0x00;
	clrf	(793)^0300h	;volatile
	line	149
	
l25244:	
;pwm.c: 149: CCP4IE = 1;
	movlb 1	; select bank1
	bsf	(1181/8)^080h,(1181)&7
	line	150
	
l25246:	
;pwm.c: 150: CCP4IF = 0;
	movlb 0	; select bank0
	bcf	(157/8),(157)&7
	line	151
	
l25248:	
;pwm.c: 151: CCP4CON = 0x05;
	movlw	(05h)
	movlb 6	; select bank6
	movwf	(794)^0300h	;volatile
	line	152
;pwm.c: 152: break;
	goto	l17328
	line	161
	
l25250:	
;pwm.c: 159: {
;pwm.c: 161: CCP5CON = 0x00;
	movlb 6	; select bank6
	clrf	(798)^0300h	;volatile
	line	162
;pwm.c: 162: CCPR5L = 0x00;
	clrf	(796)^0300h	;volatile
	line	163
;pwm.c: 163: CCPR5H = 0x00;
	clrf	(797)^0300h	;volatile
	line	164
	
l25252:	
;pwm.c: 164: CCP5IE = 1;
	movlb 1	; select bank1
	bsf	(1182/8)^080h,(1182)&7
	line	165
	
l25254:	
;pwm.c: 165: CCP5IF = 0;
	movlb 0	; select bank0
	bcf	(158/8),(158)&7
	line	166
	
l25256:	
;pwm.c: 166: CCP5CON = 0x05;
	movlw	(05h)
	movlb 6	; select bank6
	movwf	(798)^0300h	;volatile
	line	167
;pwm.c: 167: break;
	goto	l17328
	line	93
	
l25260:	
	movlb 0	; select bank0
	movf	(PWM_Capture_init@ui8_CCP_Nb),w
	; Switch size 1, requested type "space"
; Number of cases is 5, Range of values is 1 to 5
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           16     9 (average)
; direct_byte           19     9 (fixed)
; jumptable            263     9 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	1^0	; case 1
	skipnz
	goto	l25218
	xorlw	2^1	; case 2
	skipnz
	goto	l25226
	xorlw	3^2	; case 3
	skipnz
	goto	l25234
	xorlw	4^3	; case 4
	skipnz
	goto	l25242
	xorlw	5^4	; case 5
	skipnz
	goto	l25250
	goto	l17328
	opt asmopt_on

	line	183
	
l17328:	
	return
	opt stack 0
GLOBAL	__end_of_PWM_Capture_init
	__end_of_PWM_Capture_init:
;; =============== function _PWM_Capture_init ends ============

	signat	_PWM_Capture_init,4216
	global	_Oscill_Source_Block
psect	text2054,local,class=CODE,delta=2
global __ptext2054
__ptext2054:

;; *************** function _Oscill_Source_Block *****************
;; Defined at:
;;		line 480 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\timer.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/1
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_system_init
;; This function uses a non-reentrant model
;;
psect	text2054
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\timer.c"
	line	480
	global	__size_of_Oscill_Source_Block
	__size_of_Oscill_Source_Block	equ	__end_of_Oscill_Source_Block-_Oscill_Source_Block
	
_Oscill_Source_Block:	
	opt	stack 10
; Regs used in _Oscill_Source_Block: [wreg]
	line	482
	
l25214:	
;timer.c: 482: OSCCON = 0xF0;
	movlw	(0F0h)
	movlb 1	; select bank1
	movwf	(153)^080h	;volatile
	line	483
;timer.c: 483: WDTCON = 0b00010001;
	movlw	(011h)
	movwf	(151)^080h	;volatile
	line	485
	
l19667:	
	return
	opt stack 0
GLOBAL	__end_of_Oscill_Source_Block
	__end_of_Oscill_Source_Block:
;; =============== function _Oscill_Source_Block ends ============

	signat	_Oscill_Source_Block,88
	global	__ELINMIntCalcIDParity
psect	text2055,local,class=CODE,delta=2
global __ptext2055
__ptext2055:

;; *************** function __ELINMIntCalcIDParity *****************
;; Defined at:
;;		line 1051 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
;; Parameters:    Size  Location     Type
;;  ELINM_idtr      1    wreg     struct ELINMINT_ID
;; Auto vars:     Size  Location     Type
;;  ELINM_idtr      1   12[BANK0 ] struct ELINMINT_ID
;; Return value:  Size  Location     Type
;;                  1    wreg      unsigned char 
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       1       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       1       0       0       0       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		__ELINMIntSendMessage
;; This function uses a non-reentrant model
;;
psect	text2055
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
	line	1051
	global	__size_of__ELINMIntCalcIDParity
	__size_of__ELINMIntCalcIDParity	equ	__end_of__ELINMIntCalcIDParity-__ELINMIntCalcIDParity
	
__ELINMIntCalcIDParity:	
	opt	stack 7
; Regs used in __ELINMIntCalcIDParity: [wreg+status,2+status,0]
;__ELINMIntCalcIDParity@ELINM_idtr stored from wreg
	line	1053
	movwf	(__ELINMIntCalcIDParity@ELINM_idtr)
	
l28648:	
;lin.c: 1053: ELINM_idtr.ID &= 0x3F;
	movlw	(03Fh)
	andwf	(__ELINMIntCalcIDParity@ELINM_idtr),f
	line	1056
;lin.c: 1056: if( ELINM_idtr.IDbits.ID0)
	btfss	(__ELINMIntCalcIDParity@ELINM_idtr),0
	goto	u8571
	goto	u8570
u8571:
	goto	l13465
u8570:
	line	1061
	
l28650:	
;lin.c: 1059: {
;lin.c: 1061: ELINM_idtr.ID ^= 0x40;
	movlw	(040h)
	xorwf	(__ELINMIntCalcIDParity@ELINM_idtr),f
	line	1064
	
l13465:	
	line	1067
;lin.c: 1064: }
;lin.c: 1067: if( ELINM_idtr.IDbits.ID1 )
	btfss	(__ELINMIntCalcIDParity@ELINM_idtr),1
	goto	u8581
	goto	u8580
u8581:
	goto	l13466
u8580:
	line	1072
	
l28652:	
;lin.c: 1070: {
;lin.c: 1072: ELINM_idtr.ID ^= 0x40;
	movlw	(040h)
	xorwf	(__ELINMIntCalcIDParity@ELINM_idtr),f
	line	1075
	
l13466:	
	line	1078
;lin.c: 1075: }
;lin.c: 1078: if( ELINM_idtr.IDbits.ID2 )
	btfss	(__ELINMIntCalcIDParity@ELINM_idtr),2
	goto	u8591
	goto	u8590
u8591:
	goto	l13467
u8590:
	line	1083
	
l28654:	
;lin.c: 1081: {
;lin.c: 1083: ELINM_idtr.ID ^= 0x40;
	movlw	(040h)
	xorwf	(__ELINMIntCalcIDParity@ELINM_idtr),f
	line	1086
	
l13467:	
	line	1089
;lin.c: 1086: }
;lin.c: 1089: if( ELINM_idtr.IDbits.ID4 )
	btfss	(__ELINMIntCalcIDParity@ELINM_idtr),4
	goto	u8601
	goto	u8600
u8601:
	goto	l13468
u8600:
	line	1094
	
l28656:	
;lin.c: 1092: {
;lin.c: 1094: ELINM_idtr.ID ^= 0x40;
	movlw	(040h)
	xorwf	(__ELINMIntCalcIDParity@ELINM_idtr),f
	line	1097
	
l13468:	
	line	1100
;lin.c: 1097: }
;lin.c: 1100: ELINM_idtr.IDbits.ID7 = 1;
	bsf	(__ELINMIntCalcIDParity@ELINM_idtr),7
	line	1103
;lin.c: 1103: if( ELINM_idtr.IDbits.ID1 )
	btfss	(__ELINMIntCalcIDParity@ELINM_idtr),1
	goto	u8611
	goto	u8610
u8611:
	goto	l13469
u8610:
	line	1108
	
l28658:	
;lin.c: 1106: {
;lin.c: 1108: ELINM_idtr.ID ^= 0x80;
	movlw	(080h)
	xorwf	(__ELINMIntCalcIDParity@ELINM_idtr),f
	line	1111
	
l13469:	
	line	1114
;lin.c: 1111: }
;lin.c: 1114: if( ELINM_idtr.IDbits.ID3 )
	btfss	(__ELINMIntCalcIDParity@ELINM_idtr),3
	goto	u8621
	goto	u8620
u8621:
	goto	l13470
u8620:
	line	1119
	
l28660:	
;lin.c: 1117: {
;lin.c: 1119: ELINM_idtr.ID ^= 0x80;
	movlw	(080h)
	xorwf	(__ELINMIntCalcIDParity@ELINM_idtr),f
	line	1122
	
l13470:	
	line	1125
;lin.c: 1122: }
;lin.c: 1125: if( ELINM_idtr.IDbits.ID4 )
	btfss	(__ELINMIntCalcIDParity@ELINM_idtr),4
	goto	u8631
	goto	u8630
u8631:
	goto	l13471
u8630:
	line	1130
	
l28662:	
;lin.c: 1128: {
;lin.c: 1130: ELINM_idtr.ID ^= 0x80;
	movlw	(080h)
	xorwf	(__ELINMIntCalcIDParity@ELINM_idtr),f
	line	1133
	
l13471:	
	line	1136
;lin.c: 1133: }
;lin.c: 1136: if( ELINM_idtr.IDbits.ID5 )
	btfss	(__ELINMIntCalcIDParity@ELINM_idtr),5
	goto	u8641
	goto	u8640
u8641:
	goto	l13472
u8640:
	line	1141
	
l28664:	
;lin.c: 1139: {
;lin.c: 1141: ELINM_idtr.ID ^= 0x80;
	movlw	(080h)
	xorwf	(__ELINMIntCalcIDParity@ELINM_idtr),f
	line	1144
	
l13472:	
	line	1147
;lin.c: 1144: }
;lin.c: 1147: return ( ( BYTE )ELINM_idtr.ID );
	movf	(__ELINMIntCalcIDParity@ELINM_idtr),w
	line	1150
	
l13473:	
	return
	opt stack 0
GLOBAL	__end_of__ELINMIntCalcIDParity
	__end_of__ELINMIntCalcIDParity:
;; =============== function __ELINMIntCalcIDParity ends ============

	signat	__ELINMIntCalcIDParity,4217
	global	__ELINMIntInitialize
psect	text2056,local,class=CODE,delta=2
global __ptext2056
__ptext2056:

;; *************** function __ELINMIntInitialize *****************
;; Defined at:
;;		line 485 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;                  1    wreg      unsigned char 
;; Registers used:
;;		wreg, status,2
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_system_init
;; This function uses a non-reentrant model
;;
psect	text2056
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
	line	485
	global	__size_of__ELINMIntInitialize
	__size_of__ELINMIntInitialize	equ	__end_of__ELINMIntInitialize-__ELINMIntInitialize
	
__ELINMIntInitialize:	
	opt	stack 10
; Regs used in __ELINMIntInitialize: [wreg+status,2]
	line	487
	
l25178:	
;lin.c: 487: SPBRG = (((((((0x0AL*32000000L)/19200L)/0x04L)+5)/0x0AL)-2)&0x00FF);
	movlw	(09Fh)
	movlb 3	; select bank3
	movwf	(411)^0180h	;volatile
	line	488
;lin.c: 488: SPBRGH = (((((((0x0AL*32000000L)/19200L)/0x04L)+5)/0x0AL)-2)>>8);
	movlw	(01h)
	movwf	(412)^0180h	;volatile
	line	489
;lin.c: 489: TXSTA = 0x26;
	movlw	(026h)
	movwf	(414)^0180h	;volatile
	line	490
;lin.c: 490: RCSTA = 0x90;
	movlw	(090h)
	movwf	(413)^0180h	;volatile
	line	491
;lin.c: 491: BAUDCON = 0x48;
	movlw	(048h)
	movwf	(415)^0180h	;volatile
	line	492
	
l25180:	
;lin.c: 492: PIE1 = 0x00;
	movlb 1	; select bank1
	clrf	(145)^080h	;volatile
	line	493
	
l25182:	
;lin.c: 493: _ELINMIntSleepTimeout = ( ( 25000L * ( 100L * 1000000L / 19200L ) / 128L ) / 100L );
	movlw	0
	movlb 4	; select bank4
	movwf	(__ELINMIntSleepTimeout+3)^0200h
	movlw	0
	movwf	(__ELINMIntSleepTimeout+2)^0200h
	movlw	027h
	movwf	(__ELINMIntSleepTimeout+1)^0200h
	movlw	0BBh
	movwf	(__ELINMIntSleepTimeout)^0200h

	line	494
;lin.c: 494: _ELINMIntSpace = (0+(((53300L/(((19200L*128L)+5000L)/10000L))+5L)/100L)-1)/2;
	movlb 2	; select bank2
	clrf	(__ELINMIntSpace)^0100h
	line	495
;lin.c: 495: _ELINMIntStatus.ELINMIntStatusByte = 0;
	movlb 0	; select bank0
	clrf	(__ELINMIntStatus)
	line	496
;lin.c: 496: _ELINMIntStatus1.ELINMIntStatusByte = 0;
	movlb 1	; select bank1
	clrf	(__ELINMIntStatus1)^080h
	line	497
	
l25184:	
;lin.c: 497: _ELINMIntStatus.ELINMINTSTS.IDLE = 1;
	movlb 0	; select bank0
	bsf	(__ELINMIntStatus),3
	line	501
	
l13410:	
	return
	opt stack 0
GLOBAL	__end_of__ELINMIntInitialize
	__end_of__ELINMIntInitialize:
;; =============== function __ELINMIntInitialize ends ============

	signat	__ELINMIntInitialize,89
	global	_EnableMCP201
psect	text2057,local,class=CODE,delta=2
global __ptext2057
__ptext2057:

;; *************** function _EnableMCP201 *****************
;; Defined at:
;;		line 436 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  _dcnt           1   14[BANK0 ] unsigned char 
;;  _dcnt           1   13[BANK0 ] unsigned char 
;;  _dcnt           1   12[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		status,2
;; Tracked objects:
;;		On entry : 17F/1
;;		On exit  : 1F/0
;;		Unchanged: FFE00/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       3       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       3       0       0       0       0       0       0
;;Total ram usage:        3 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_system_init
;; This function uses a non-reentrant model
;;
psect	text2057
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
	line	436
	global	__size_of_EnableMCP201
	__size_of_EnableMCP201	equ	__end_of_EnableMCP201-_EnableMCP201
	
_EnableMCP201:	
	opt	stack 10
; Regs used in _EnableMCP201: [status,2]
	line	447
	
l25170:	
;lin.c: 448: { unsigned char _dcnt; if( 200L >= 4 ) _dcnt = ( 200L * ( 32000000UL ) / 2 ); else _dcnt = 1; while( --_dcnt > 0 ) { asm( "nop" ); asm( "nop" ); continue; } } ;
	movlb 0	; select bank0
	bcf	(117/8),(117)&7
	line	448
	
l25172:	
	clrf	(EnableMCP201@_dcnt)
	goto	l13394
	
l13395:	
# 448 "D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
nop ;#
# 448 "D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
nop ;#
psect	text2057
	
l13394:	
	movlb 0	; select bank0
	decfsz	(EnableMCP201@_dcnt),f
	goto	u4961
	goto	u4960
u4961:
	goto	l13395
u4960:
	
l13396:	
	line	449
;lin.c: 450: { unsigned char _dcnt; if( 200L >= 4 ) _dcnt = ( 200L * ( 32000000UL ) / 2 ); else _dcnt = 1; while( --_dcnt > 0 ) { asm( "nop" ); asm( "nop" ); continue; } } ;
	bsf	(117/8),(117)&7
	line	450
	
l25174:	
	clrf	(EnableMCP201@_dcnt_15775)
	goto	l13399
	
l13400:	
# 450 "D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
nop ;#
# 450 "D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
nop ;#
psect	text2057
	
l13399:	
	movlb 0	; select bank0
	decfsz	(EnableMCP201@_dcnt_15775),f
	goto	u4971
	goto	u4970
u4971:
	goto	l13400
u4970:
	
l13401:	
	line	451
;lin.c: 452: { unsigned char _dcnt; if( 200L >= 4 ) _dcnt = ( 200L * ( 32000000UL ) / 2 ); else _dcnt = 1; while( --_dcnt > 0 ) { asm( "nop" ); asm( "nop" ); continue; } } ;
	bcf	(117/8),(117)&7
	line	452
	
l25176:	
	clrf	(EnableMCP201@_dcnt_15776)
	goto	l13404
	
l13405:	
# 452 "D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
nop ;#
# 452 "D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
nop ;#
psect	text2057
	
l13404:	
	movlb 0	; select bank0
	decfsz	(EnableMCP201@_dcnt_15776),f
	goto	u4981
	goto	u4980
u4981:
	goto	l13405
u4980:
	
l13406:	
	line	453
;lin.c: 453: RC5 = 1;
	bsf	(117/8),(117)&7
	line	455
	
l13407:	
	return
	opt stack 0
GLOBAL	__end_of_EnableMCP201
	__end_of_EnableMCP201:
;; =============== function _EnableMCP201 ends ============

	signat	_EnableMCP201,88
	global	__ELINMIntGetPointer
psect	text2058,local,class=CODE,delta=2
global __ptext2058
__ptext2058:

;; *************** function __ELINMIntGetPointer *****************
;; Defined at:
;;		line 1379 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
;; Parameters:    Size  Location     Type
;;  _ELINMInt_ta    1    wreg     unsigned char 
;;  _ELINMInt_po    1   12[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  _ELINMInt_ta    1   13[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  1    wreg      PTR unsigned char 
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       1       0       0       0       0       0       0
;;      Locals:         0       1       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       2       0       0       0       0       0       0
;;Total ram usage:        2 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_Transmit_LIN_8Bytes
;;		_Receive_Diag
;; This function uses a non-reentrant model
;;
psect	text2058
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
	line	1379
	global	__size_of__ELINMIntGetPointer
	__size_of__ELINMIntGetPointer	equ	__end_of__ELINMIntGetPointer-__ELINMIntGetPointer
	
__ELINMIntGetPointer:	
	opt	stack 8
; Regs used in __ELINMIntGetPointer: [wreg+status,2+status,0]
	line	1382
	
l28644:	
;lin.c: 1382: return ( ( BYTE * )& _ELINMIntMessageBuffer[ _ELINMInt_position ] );
	movf	(__ELINMIntGetPointer@_ELINMInt_position),w
	addlw	__ELINMIntMessageBuffer&0ffh
	line	1385
	
l13489:	
	return
	opt stack 0
GLOBAL	__end_of__ELINMIntGetPointer
	__end_of__ELINMIntGetPointer:
;; =============== function __ELINMIntGetPointer ends ============

	signat	__ELINMIntGetPointer,8313
	global	_write_eeprom_data
psect	text2059,local,class=CODE,delta=2
global __ptext2059
__ptext2059:

;; *************** function _write_eeprom_data *****************
;; Defined at:
;;		line 86 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\eeprom.c"
;; Parameters:    Size  Location     Type
;;  ui8_adress      1    wreg     unsigned char 
;;  ui8_adress_d    1   19[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  ui8_adress      1   20[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/3
;;		Unchanged: FFE00/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       1       0       0       0       0       0       0
;;      Locals:         0       1       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       2       0       0       0       0       0       0
;;Total ram usage:        2 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_EOL
;; This function uses a non-reentrant model
;;
psect	text2059
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\eeprom.c"
	line	86
	global	__size_of_write_eeprom_data
	__size_of_write_eeprom_data	equ	__end_of_write_eeprom_data-_write_eeprom_data
	
_write_eeprom_data:	
	opt	stack 10
; Regs used in _write_eeprom_data: [wreg]
;write_eeprom_data@ui8_adress stored from wreg
	line	89
	movwf	(write_eeprom_data@ui8_adress)
	
l28596:	
;eeprom.c: 89: EEADRL = ui8_adress;
	movf	(write_eeprom_data@ui8_adress),w
	movlb 3	; select bank3
	movwf	(401)^0180h	;volatile
	line	90
;eeprom.c: 90: EEDATL = ui8_adress_data;
	movlb 0	; select bank0
	movf	(write_eeprom_data@ui8_adress_data),w
	movlb 3	; select bank3
	movwf	(403)^0180h	;volatile
	line	91
	
l28598:	
;eeprom.c: 91: CFGS = 0;
	bcf	(3246/8)^0180h,(3246)&7
	line	92
	
l28600:	
;eeprom.c: 92: EEPGD = 0;
	bcf	(3247/8)^0180h,(3247)&7
	line	93
	
l28602:	
;eeprom.c: 93: WREN = 1;
	bsf	(3242/8)^0180h,(3242)&7
	line	95
	
l28604:	
;eeprom.c: 95: GIE = 0;
	bcf	(95/8),(95)&7
	line	96
;eeprom.c: 96: EECON2 = 0x55;
	movlw	(055h)
	movwf	(406)^0180h	;volatile
	line	97
;eeprom.c: 97: EECON2 = 0xAA;
	movlw	(0AAh)
	movwf	(406)^0180h	;volatile
	line	98
	
l28606:	
;eeprom.c: 98: WR = 1;
	bsf	(3241/8)^0180h,(3241)&7
	line	99
	
l28608:	
;eeprom.c: 99: GIE = 1;
	bsf	(95/8),(95)&7
	line	100
	
l28610:	
;eeprom.c: 100: WREN = 0;
	bcf	(3242/8)^0180h,(3242)&7
	line	103
;eeprom.c: 103: while( WR )
	goto	l9635
	
l9636:	
	line	107
# 107 "D:\Projekte\BVH2\BLDC\V0.9.3\src\eeprom.c"
nop ;#
	line	108
# 108 "D:\Projekte\BVH2\BLDC\V0.9.3\src\eeprom.c"
nop ;#
psect	text2059
	line	110
	
l9635:	
	line	103
	movlb 3	; select bank3
	btfsc	(3241/8)^0180h,(3241)&7
	goto	u8501
	goto	u8500
u8501:
	goto	l9636
u8500:
	line	113
	
l9638:	
	return
	opt stack 0
GLOBAL	__end_of_write_eeprom_data
	__end_of_write_eeprom_data:
;; =============== function _write_eeprom_data ends ============

	signat	_write_eeprom_data,8312
	global	_read_eeprom_data
psect	text2060,local,class=CODE,delta=2
global __ptext2060
__ptext2060:

;; *************** function _read_eeprom_data *****************
;; Defined at:
;;		line 54 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\eeprom.c"
;; Parameters:    Size  Location     Type
;;  ui8_adress      1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  ui8_adress      1   12[BANK0 ] unsigned char 
;;  ui8_adress_d    1   13[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  1    wreg      unsigned char 
;; Registers used:
;;		wreg
;; Tracked objects:
;;		On entry : 18/2
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       2       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       2       0       0       0       0       0       0
;;Total ram usage:        2 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_EOL
;;		_I_calibrationInit
;;		_init_ports
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2060
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\eeprom.c"
	line	54
	global	__size_of_read_eeprom_data
	__size_of_read_eeprom_data	equ	__end_of_read_eeprom_data-_read_eeprom_data
	
_read_eeprom_data:	
	opt	stack 10
; Regs used in _read_eeprom_data: [wreg]
;read_eeprom_data@ui8_adress stored from wreg
	line	58
	movlb 0	; select bank0
	movwf	(read_eeprom_data@ui8_adress)
	
l28586:	
;eeprom.c: 56: unsigned char ui8_adress_data;
;eeprom.c: 58: EEADRL = ui8_adress;
	movf	(read_eeprom_data@ui8_adress),w
	movlb 3	; select bank3
	movwf	(401)^0180h	;volatile
	line	59
	
l28588:	
;eeprom.c: 59: EEPGD = 0;
	bcf	(3247/8)^0180h,(3247)&7
	line	60
	
l28590:	
;eeprom.c: 60: CFGS = 0;
	bcf	(3246/8)^0180h,(3246)&7
	line	61
	
l28592:	
;eeprom.c: 61: RD = 1;
	bsf	(3240/8)^0180h,(3240)&7
	line	62
;eeprom.c: 62: ui8_adress_data = EEDATL;
	movf	(403)^0180h,w	;volatile
	movlb 0	; select bank0
	movwf	(read_eeprom_data@ui8_adress_data)
	line	64
;eeprom.c: 64: return ui8_adress_data;
	movf	(read_eeprom_data@ui8_adress_data),w
	line	67
	
l9632:	
	return
	opt stack 0
GLOBAL	__end_of_read_eeprom_data
	__end_of_read_eeprom_data:
;; =============== function _read_eeprom_data ends ============

	signat	_read_eeprom_data,4217
	global	_PWM_Write_Out
psect	text2061,local,class=CODE,delta=2
global __ptext2061
__ptext2061:

;; *************** function _PWM_Write_Out *****************
;; Defined at:
;;		line 342 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\pwm.c"
;; Parameters:    Size  Location     Type
;;  ui8_DutyCycl    1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  ui8_DutyCycl    1   12[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg
;; Tracked objects:
;;		On entry : 1C/2
;;		On exit  : 1F/2
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       1       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       1       0       0       0       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_DiagInit
;;		_SetDiagAlarm
;;		_EOL
;; This function uses a non-reentrant model
;;
psect	text2061
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\pwm.c"
	line	342
	global	__size_of_PWM_Write_Out
	__size_of_PWM_Write_Out	equ	__end_of_PWM_Write_Out-_PWM_Write_Out
	
_PWM_Write_Out:	
	opt	stack 10
; Regs used in _PWM_Write_Out: [wreg]
;PWM_Write_Out@ui8_DutyCycle_Out stored from wreg
	line	344
	movlb 0	; select bank0
	movwf	(PWM_Write_Out@ui8_DutyCycle_Out)
	
l28584:	
;pwm.c: 344: ui8_PWMoutvalue = ui8_DutyCycle_Out;
	movf	(PWM_Write_Out@ui8_DutyCycle_Out),w
	movlb 2	; select bank2
	movwf	(_ui8_PWMoutvalue)^0100h
	line	346
	
l17337:	
	return
	opt stack 0
GLOBAL	__end_of_PWM_Write_Out
	__end_of_PWM_Write_Out:
;; =============== function _PWM_Write_Out ends ============

	signat	_PWM_Write_Out,4216
	global	_InitMotorStop
psect	text2062,local,class=CODE,delta=2
global __ptext2062
__ptext2062:

;; *************** function _InitMotorStop *****************
;; Defined at:
;;		line 2159 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\bldc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		status,2
;; Tracked objects:
;;		On entry : 1C/2
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_EOL
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2062
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\bldc.c"
	line	2159
	global	__size_of_InitMotorStop
	__size_of_InitMotorStop	equ	__end_of_InitMotorStop-_InitMotorStop
	
_InitMotorStop:	
	opt	stack 10
; Regs used in _InitMotorStop: [status,2]
	line	2161
	
l28574:	
;bldc.c: 2161: if( 1 == MotorFlags.bits.B6 )
	movlb 0	; select bank0
	btfss	(_MotorFlags),6
	goto	u8491
	goto	u8490
u8491:
	goto	l3910
u8490:
	line	2166
	
l28576:	
;bldc.c: 2164: {
;bldc.c: 2166: LATC1 = 0;
	movlb 2	; select bank2
	bcf	(2161/8)^0100h,(2161)&7
	line	2167
;bldc.c: 2167: LATC3 = 0;
	bcf	(2163/8)^0100h,(2163)&7
	line	2168
;bldc.c: 2168: LATC4 = 0;
	bcf	(2164/8)^0100h,(2164)&7
	line	2170
;bldc.c: 2170: CCP1ASE = 0;
	movlb 5	; select bank5
	bcf	(5295/8)^0280h,(5295)&7
	line	2171
	
l28578:	
;bldc.c: 2171: CCPR1L = 0;
	clrf	(657)^0280h	;volatile
	line	2172
	
l28580:	
;bldc.c: 2172: MotorFlags.bits.B6 = 0;
	movlb 0	; select bank0
	bcf	(_MotorFlags),6
	line	2173
	
l28582:	
;bldc.c: 2173: MotorFlags.bits.B7 = 0;
	bcf	(_MotorFlags),7
	line	2179
	
l3910:	
	return
	opt stack 0
GLOBAL	__end_of_InitMotorStop
	__end_of_InitMotorStop:
;; =============== function _InitMotorStop ends ============

	signat	_InitMotorStop,88
	global	_FILTER_Temp
psect	text2063,local,class=CODE,delta=2
global __ptext2063
__ptext2063:

;; *************** function _FILTER_Temp *****************
;; Defined at:
;;		line 402 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/3
;;		On exit  : 1F/3
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_Get_Analog_Value
;; This function uses a non-reentrant model
;;
psect	text2063
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
	line	402
	global	__size_of_FILTER_Temp
	__size_of_FILTER_Temp	equ	__end_of_FILTER_Temp-_FILTER_Temp
	
_FILTER_Temp:	
	opt	stack 10
; Regs used in _FILTER_Temp: [wreg+status,2+status,0]
	line	407
	
l24716:	
;adc.c: 407: filterTempNTC = filterTempNTC + ui16_NTC_Temp_bldc;
	movlb 4	; select bank4
	movf	(_ui16_NTC_Temp_bldc)^0200h,w
	movlb 3	; select bank3
	addwf	(_filterTempNTC)^0180h,f
	movlb 4	; select bank4
	movf	(_ui16_NTC_Temp_bldc+1)^0200h,w
	movlb 3	; select bank3
	addwfc	(_filterTempNTC+1)^0180h,f
	line	408
;adc.c: 408: ui16_NTC_Temp_bldc_mean = filterTempNTC>>6;
	movf	(_filterTempNTC+1)^0180h,w
	movwf	(_ui16_NTC_Temp_bldc_mean+1)^0180h
	movf	(_filterTempNTC)^0180h,w
	movwf	(_ui16_NTC_Temp_bldc_mean)^0180h
	
l24718:	
	movlw	06h
	
u4285:
	lsrf	(_ui16_NTC_Temp_bldc_mean+1)^0180h,f
	rrf	(_ui16_NTC_Temp_bldc_mean)^0180h,f
	decfsz	wreg,f
	goto	u4285
	line	409
	
l24720:	
;adc.c: 409: filterTempNTC = filterTempNTC - ui16_NTC_Temp_bldc_mean;
	movf	(_ui16_NTC_Temp_bldc_mean)^0180h,w
	subwf	(_filterTempNTC)^0180h,f
	movf	(_ui16_NTC_Temp_bldc_mean+1)^0180h,w
	subwfb	(_filterTempNTC+1)^0180h,f
	line	426
	
l24722:	
;adc.c: 426: ui16_NTC_Temp_bldc_mean_cal = ui16_NTC_Temp_bldc_mean + ui16_Temp_cal;
	movf	(_ui16_NTC_Temp_bldc_mean+1)^0180h,w
	movwf	(_ui16_NTC_Temp_bldc_mean_cal+1)^0180h
	movf	(_ui16_NTC_Temp_bldc_mean)^0180h,w
	movwf	(_ui16_NTC_Temp_bldc_mean_cal)^0180h
	
l24724:	
	movlb 4	; select bank4
	movf	(_ui16_Temp_cal)^0200h,w
	movlb 3	; select bank3
	addwf	(_ui16_NTC_Temp_bldc_mean_cal)^0180h,f
	movlb 4	; select bank4
	movf	(_ui16_Temp_cal+1)^0200h,w
	movlb 3	; select bank3
	addwfc	(_ui16_NTC_Temp_bldc_mean_cal+1)^0180h,f
	line	428
	
l1879:	
	return
	opt stack 0
GLOBAL	__end_of_FILTER_Temp
	__end_of_FILTER_Temp:
;; =============== function _FILTER_Temp ends ============

	signat	_FILTER_Temp,88
	global	_FILTER_IPhase
psect	text2064,local,class=CODE,delta=2
global __ptext2064
__ptext2064:

;; *************** function _FILTER_IPhase *****************
;; Defined at:
;;		line 359 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1D/1
;;		On exit  : 1F/3
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_Get_Analog_Value
;; This function uses a non-reentrant model
;;
psect	text2064
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
	line	359
	global	__size_of_FILTER_IPhase
	__size_of_FILTER_IPhase	equ	__end_of_FILTER_IPhase-_FILTER_IPhase
	
_FILTER_IPhase:	
	opt	stack 10
; Regs used in _FILTER_IPhase: [wreg+status,2+status,0]
	line	363
	
l24710:	
;adc.c: 363: filter3 = filter3 + ui16_IPhase1_bldc.w;
	movlb 3	; select bank3
	movf	(_ui16_IPhase1_bldc)^0180h,w	;volatile
	addwf	(_filter3)^0180h,f
	movf	(_ui16_IPhase1_bldc+1)^0180h,w	;volatile
	addwfc	(_filter3+1)^0180h,f
	line	364
;adc.c: 364: ui16_fir_IPhase_mean.w = filter3>>6;
	movf	(_filter3+1)^0180h,w
	movwf	(_ui16_fir_IPhase_mean+1)^0180h
	movf	(_filter3)^0180h,w
	movwf	(_ui16_fir_IPhase_mean)^0180h
	
l24712:	
	movlw	06h
	
u4275:
	lsrf	(_ui16_fir_IPhase_mean+1)^0180h,f
	rrf	(_ui16_fir_IPhase_mean)^0180h,f
	decfsz	wreg,f
	goto	u4275
	line	365
	
l24714:	
;adc.c: 365: filter3 = filter3 - ui16_fir_IPhase_mean.w;
	movf	(_ui16_fir_IPhase_mean)^0180h,w
	subwf	(_filter3)^0180h,f
	movf	(_ui16_fir_IPhase_mean+1)^0180h,w
	subwfb	(_filter3+1)^0180h,f
	line	376
	
l1876:	
	return
	opt stack 0
GLOBAL	__end_of_FILTER_IPhase
	__end_of_FILTER_IPhase:
;; =============== function _FILTER_IPhase ends ============

	signat	_FILTER_IPhase,88
	global	_FILTER_Ubat
psect	text2065,local,class=CODE,delta=2
global __ptext2065
__ptext2065:

;; *************** function _FILTER_Ubat *****************
;; Defined at:
;;		line 266 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, fsr1l, fsr1h, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1D/1
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0      12       0       0       0       0       0       0
;;      Totals:         0      12       0       0       0       0       0       0
;;Total ram usage:       12 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_Get_Analog_Value
;; This function uses a non-reentrant model
;;
psect	text2065
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
	line	266
	global	__size_of_FILTER_Ubat
	__size_of_FILTER_Ubat	equ	__end_of_FILTER_Ubat-_FILTER_Ubat
	
_FILTER_Ubat:	
	opt	stack 10
; Regs used in _FILTER_Ubat: [wreg-status,0]
	line	271
	
l24698:	
;adc.c: 271: inputArray1[ windowPtr1 ] = ui16_Ubat_bldc.w;
	movlb 1	; select bank1
	lslf	(_windowPtr1)^080h,w
	addlw	_inputArray1&0ffh
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movlb 0	; select bank0
	movf	(_ui16_Ubat_bldc),w	;volatile
	movwi	[0]fsr1
	movf	(_ui16_Ubat_bldc+1),w	;volatile
	movwi	[1]fsr1
	line	272
	
l24700:	
;adc.c: 272: filter1 = inputArray1[ 0 ] + inputArray1[ 1 ] + inputArray1[ 2 ] + inputArray1[ 3 ] + inputArray1[ 4 ] + inputArray1[ 5 ] + inputArray1[ 6 ] + inputArray1[ 7 ];
	movlw	(0Eh)
	addlw	_inputArray1&0ffh
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movlw	(0Ch)
	addlw	_inputArray1&0ffh
	movwf	fsr0l
	movlw 2	; select bank4/5
	movwf fsr0h	
	
	movlb 4	; select bank4
	movf	0+(_inputArray1)^0200h+04h,w
	addwf	0+(_inputArray1)^0200h+02h,w
	movlb 0	; select bank0
	movwf	(??_FILTER_Ubat+0)+0
	movlb 4	; select bank4
	movf	1+(_inputArray1)^0200h+04h,w
	addwfc	1+(_inputArray1)^0200h+02h,w
	movlb 0	; select bank0
	movwf	1+(??_FILTER_Ubat+0)+0
	movlb 4	; select bank4
	movf	0+(_inputArray1)^0200h+06h,w
	movlb 0	; select bank0
	addwf	0+(??_FILTER_Ubat+0)+0,w
	movwf	(??_FILTER_Ubat+2)+0
	movlb 4	; select bank4
	movf	1+(_inputArray1)^0200h+06h,w
	movlb 0	; select bank0
	addwfc	1+(??_FILTER_Ubat+0)+0,w
	movwf	1+(??_FILTER_Ubat+2)+0
	movlb 4	; select bank4
	movf	0+(_inputArray1)^0200h+08h,w
	movlb 0	; select bank0
	addwf	0+(??_FILTER_Ubat+2)+0,w
	movwf	(??_FILTER_Ubat+4)+0
	movlb 4	; select bank4
	movf	1+(_inputArray1)^0200h+08h,w
	movlb 0	; select bank0
	addwfc	1+(??_FILTER_Ubat+2)+0,w
	movwf	1+(??_FILTER_Ubat+4)+0
	movlb 4	; select bank4
	movf	0+(_inputArray1)^0200h+0Ah,w
	movlb 0	; select bank0
	addwf	0+(??_FILTER_Ubat+4)+0,w
	movwf	(??_FILTER_Ubat+6)+0
	movlb 4	; select bank4
	movf	1+(_inputArray1)^0200h+0Ah,w
	movlb 0	; select bank0
	addwfc	1+(??_FILTER_Ubat+4)+0,w
	movwf	1+(??_FILTER_Ubat+6)+0
	moviw	[0]fsr0
	addwf	0+(??_FILTER_Ubat+6)+0,w
	movwf	(??_FILTER_Ubat+8)+0
	moviw	[1]fsr0
	addwfc	1+(??_FILTER_Ubat+6)+0,w
	movwf	(??_FILTER_Ubat+8)+0+1
	moviw	[0]fsr1
	addwf	0+(??_FILTER_Ubat+8)+0,w
	movwf	(??_FILTER_Ubat+10)+0
	moviw	[1]fsr1
	addwfc	1+(??_FILTER_Ubat+8)+0,w
	movwf	(??_FILTER_Ubat+10)+0+1
	movlb 4	; select bank4
	movf	(_inputArray1)^0200h,w
	movlb 0	; select bank0
	addwf	0+(??_FILTER_Ubat+10)+0,w
	movlb 3	; select bank3
	movwf	(_filter1)^0180h
	movlb 4	; select bank4
	movf	(_inputArray1+1)^0200h,w
	movlb 0	; select bank0
	addwfc	1+(??_FILTER_Ubat+10)+0,w
	movlb 3	; select bank3
	movwf	1+(_filter1)^0180h
	line	273
	
l24702:	
;adc.c: 273: ui16_fir_Bat_mittel = filter1>>3;
	movf	(_filter1+1)^0180h,w
	movwf	(_ui16_fir_Bat_mittel+1)^0180h
	movf	(_filter1)^0180h,w
	movwf	(_ui16_fir_Bat_mittel)^0180h
	
l24704:	
	lsrf	(_ui16_fir_Bat_mittel+1)^0180h,f
	rrf	(_ui16_fir_Bat_mittel)^0180h,f
	lsrf	(_ui16_fir_Bat_mittel+1)^0180h,f
	rrf	(_ui16_fir_Bat_mittel)^0180h,f
	lsrf	(_ui16_fir_Bat_mittel+1)^0180h,f
	rrf	(_ui16_fir_Bat_mittel)^0180h,f
	line	275
	
l24706:	
;adc.c: 275: if( ++windowPtr1 >= 8 )
	movlw	(08h)
	movlb 1	; select bank1
	incf	(_windowPtr1)^080h,f
	subwf	((_windowPtr1)^080h),w
	skipc
	goto	u4261
	goto	u4260
u4261:
	goto	l1870
u4260:
	line	279
	
l24708:	
;adc.c: 277: {
;adc.c: 279: windowPtr1 = 0;
	clrf	(_windowPtr1)^080h
	line	280
;adc.c: 280: filter1 = 0;
	movlb 3	; select bank3
	clrf	(_filter1)^0180h
	clrf	(_filter1+1)^0180h
	line	296
	
l1870:	
	return
	opt stack 0
GLOBAL	__end_of_FILTER_Ubat
	__end_of_FILTER_Ubat:
;; =============== function _FILTER_Ubat ends ============

	signat	_FILTER_Ubat,88
	global	_ADC_Read
psect	text2066,local,class=CODE,delta=2
global __ptext2066
__ptext2066:

;; *************** function _ADC_Read *****************
;; Defined at:
;;		line 177 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  i               1   14[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  2   12[BANK0 ] unsigned int 
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 0/1
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       2       0       0       0       0       0       0
;;      Locals:         0       1       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       3       0       0       0       0       0       0
;;Total ram usage:        3 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_FILTER_Init
;;		_EOL
;;		_I_calibrationInit
;; This function uses a non-reentrant model
;;
psect	text2066
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
	line	177
	global	__size_of_ADC_Read
	__size_of_ADC_Read	equ	__end_of_ADC_Read-_ADC_Read
	
_ADC_Read:	
	opt	stack 10
; Regs used in _ADC_Read: [wreg+status,2+status,0]
	line	180
	
l28332:	
;adc.c: 179: unsigned char i;
;adc.c: 180: i = 0;
	movlb 0	; select bank0
	clrf	(ADC_Read@i)
	line	182
;adc.c: 182: while( GO_nDONE )
	goto	l1857
	line	186
	
l28334:	
;adc.c: 184: {
;adc.c: 186: i++;
	movlb 0	; select bank0
	incf	(ADC_Read@i),f
	line	189
	
l28336:	
;adc.c: 189: if( i > 64 )
	movlw	(041h)
	subwf	(ADC_Read@i),w
	skipc
	goto	u8201
	goto	u8200
u8201:
	goto	l1857
u8200:
	goto	l28340
	line	198
	
l1857:	
	line	182
	movlb 1	; select bank1
	btfsc	(1257/8)^080h,(1257)&7
	goto	u8211
	goto	u8210
u8211:
	goto	l28334
u8210:
	line	201
	
l28340:	
;adc.c: 195: }
;adc.c: 198: }
;adc.c: 201: analog_value.b.hi = ADRESH;
	movlb 1	; select bank1
	movf	(156)^080h,w	;volatile
	movlb 0	; select bank0
	movwf	0+(_analog_value)+01h	;volatile
	line	202
;adc.c: 202: analog_value.b.lo = ADRESL;
	movlb 1	; select bank1
	movf	(155)^080h,w	;volatile
	movlb 0	; select bank0
	movwf	(_analog_value)	;volatile
	line	204
;adc.c: 204: return analog_value.w;
	movf	(_analog_value+1),w	;volatile
	movwf	(?_ADC_Read+1)
	movf	(_analog_value),w	;volatile
	movwf	(?_ADC_Read)
	line	206
	
l1861:	
	return
	opt stack 0
GLOBAL	__end_of_ADC_Read
	__end_of_ADC_Read:
;; =============== function _ADC_Read ends ============

	signat	_ADC_Read,90
	global	_ADC_Init
psect	text2067,local,class=CODE,delta=2
global __ptext2067
__ptext2067:

;; *************** function _ADC_Init *****************
;; Defined at:
;;		line 104 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 18/1
;;		On exit  : 1F/2
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_system_init
;; This function uses a non-reentrant model
;;
psect	text2067
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
	line	104
	global	__size_of_ADC_Init
	__size_of_ADC_Init	equ	__end_of_ADC_Init-_ADC_Init
	
_ADC_Init:	
	opt	stack 10
; Regs used in _ADC_Init: [wreg+status,2+status,0]
	line	106
	
l24678:	
;adc.c: 106: ADCON1 = 0b10100000;
	movlw	(0A0h)
	movlb 1	; select bank1
	movwf	(158)^080h	;volatile
	line	108
	
l24680:	
;adc.c: 108: ADCON0 = 0b00000000;
	clrf	(157)^080h	;volatile
	line	109
	
l24682:	
;adc.c: 109: ADON = 1;
	bsf	(1256/8)^080h,(1256)&7
	line	110
	
l24684:	
;adc.c: 111: ui8_temp_calibration = 0;
	movlw	(030h)
	movlb 2	; select bank2
	iorwf	(279)^0100h,f	;volatile
	line	113
	
l1851:	
	return
	opt stack 0
GLOBAL	__end_of_ADC_Init
	__end_of_ADC_Init:
;; =============== function _ADC_Init ends ============

	signat	_ADC_Init,88
	global	_interrupt_handler
psect	intentry,class=CODE,delta=2
global __pintentry
__pintentry:

;; *************** function _interrupt_handler *****************
;; Defined at:
;;		line 79 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\interrupt.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, fsr1l, fsr1h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 1F/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_PWM_CTRL
;;		_Task1ms
;;		_interrrupt_bldc
;;		_ELINMIntHandler
;;		_interrupt_PWMCapture
;; This function is called by:
;;		Interrupt level 1
;; This function uses a non-reentrant model
;;
psect	intentry
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\interrupt.c"
	line	79
	global	__size_of_interrupt_handler
	__size_of_interrupt_handler	equ	__end_of_interrupt_handler-_interrupt_handler
	
_interrupt_handler:	
	opt	stack 7
; Regs used in _interrupt_handler: [wreg-status,0+pclath+cstack]
psect	intentry
	pagesel	$
	line	81
	
i1l28612:	
;interrupt.c: 81: if( TMR1IF )
	movlb 0	; select bank0
	btfss	(136/8),(136)&7
	goto	u851_21
	goto	u851_20
u851_21:
	goto	i1l11458
u851_20:
	line	85
	
i1l28614:	
;interrupt.c: 83: {
;interrupt.c: 85: TMR1IF = 0;
	bcf	(136/8),(136)&7
	line	87
	
i1l11458:	
	line	90
;interrupt.c: 87: }
;interrupt.c: 90: if( TMR4IF )
	btfss	(153/8),(153)&7
	goto	u852_21
	goto	u852_20
u852_21:
	goto	i1l28622
u852_20:
	line	94
	
i1l28616:	
;interrupt.c: 92: {
;interrupt.c: 94: TMR4IF = 0;
	bcf	(153/8),(153)&7
	line	96
	
i1l28618:	
;interrupt.c: 96: PWM_CTRL( );
	fcall	_PWM_CTRL
	line	98
	
i1l28620:	
;interrupt.c: 98: Task1ms( );
	fcall	_Task1ms
	line	106
	
i1l28622:	
;interrupt.c: 100: }
;interrupt.c: 106: if( TMR2IF )
	movlb 0	; select bank0
	btfss	(137/8),(137)&7
	goto	u853_21
	goto	u853_20
u853_21:
	goto	i1l28628
u853_20:
	line	110
	
i1l28624:	
;interrupt.c: 108: {
;interrupt.c: 110: TMR2IF = 0;
	bcf	(137/8),(137)&7
	line	112
	
i1l28626:	
;interrupt.c: 112: interrrupt_bldc( );
	fcall	_interrrupt_bldc
	line	117
	
i1l28628:	
;interrupt.c: 114: }
;interrupt.c: 117: if( TMR6IF )
	movlb 0	; select bank0
	btfss	(155/8),(155)&7
	goto	u854_21
	goto	u854_20
u854_21:
	goto	i1l28634
u854_20:
	line	121
	
i1l28630:	
;interrupt.c: 119: {
;interrupt.c: 121: TMR6IF = 0;
	bcf	(155/8),(155)&7
	line	126
	
i1l28632:	
;interrupt.c: 126: ELINMIntHandler( );
	fcall	_ELINMIntHandler
	line	134
	
i1l28634:	
;interrupt.c: 131: }
;interrupt.c: 134: if( CCP5IF )
	movlb 0	; select bank0
	btfss	(158/8),(158)&7
	goto	u855_21
	goto	u855_20
u855_21:
	goto	i1l28640
u855_20:
	line	138
	
i1l28636:	
;interrupt.c: 136: {
;interrupt.c: 138: CCP5IF = 0;
	bcf	(158/8),(158)&7
	line	140
	
i1l28638:	
;interrupt.c: 140: interrupt_PWMCapture( );
	fcall	_interrupt_PWMCapture
	line	145
	
i1l28640:	
;interrupt.c: 142: }
;interrupt.c: 145: if( CCP2IF )
	movlb 0	; select bank0
	btfss	(144/8),(144)&7
	goto	u856_21
	goto	u856_20
u856_21:
	goto	i1l11464
u856_20:
	line	149
	
i1l28642:	
;interrupt.c: 147: {
;interrupt.c: 149: CCP2IF = 0;
	bcf	(144/8),(144)&7
	line	154
	
i1l11464:	
	retfie
	opt stack 0
GLOBAL	__end_of_interrupt_handler
	__end_of_interrupt_handler:
;; =============== function _interrupt_handler ends ============

	signat	_interrupt_handler,88
	global	_interrrupt_bldc
psect	text2069,local,class=CODE,delta=2
global __ptext2069
__ptext2069:

;; *************** function _interrrupt_bldc *****************
;; Defined at:
;;		line 283 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\bldc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  ui32_tmp        4    9[COMMON] unsigned long 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, fsr1l, fsr1h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 19/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         4       0       0       0       0       0       0       0
;;      Temps:          2       0       0       0       0       0       0       0
;;      Totals:         6       0       0       0       0       0       0       0
;;Total ram usage:        6 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		i1_commutate
;;		_BLDCWait
;;		i1_ADC_Wait
;;		i1_ADC_Read
;;		i1___wmul
;;		i1___lwdiv
;; This function is called by:
;;		_interrupt_handler
;; This function uses a non-reentrant model
;;
psect	text2069
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\bldc.c"
	line	283
	global	__size_of_interrrupt_bldc
	__size_of_interrrupt_bldc	equ	__end_of_interrrupt_bldc-_interrrupt_bldc
	
_interrrupt_bldc:	
	opt	stack 7
; Regs used in _interrrupt_bldc: [wreg-status,0+pclath+cstack]
	line	287
	
i1l28346:	
;bldc.c: 285: unsigned long ui32_tmp ;
;bldc.c: 287: ++comm_time ;
	incf	(_comm_time),f
	skipnz
	incf	(_comm_time+1),f
	line	290
	
i1l28348:	
;bldc.c: 290: if( comm_time > ui16_comm_time_max )
	movf	(_comm_time+1),w
	movlb 2	; select bank2
	subwf	(_ui16_comm_time_max+1)^0100h,w
	skipz
	goto	u822_25
	movlb 0	; select bank0
	movf	(_comm_time),w
	movlb 2	; select bank2
	subwf	(_ui16_comm_time_max)^0100h,w
u822_25:
	skipnc
	goto	u822_21
	goto	u822_20
u822_21:
	goto	i1l28352
u822_20:
	line	295
	
i1l28350:	
;bldc.c: 293: {
;bldc.c: 295: commutate( );
	fcall	i1_commutate
	line	301
	
i1l28352:	
;bldc.c: 297: }
;bldc.c: 301: if( ui16_step_cnt < 10 )
	movlw	high(0Ah)
	movlb 1	; select bank1
	subwf	(_ui16_step_cnt+1)^080h,w
	movlw	low(0Ah)
	skipnz
	subwf	(_ui16_step_cnt)^080h,w
	skipnc
	goto	u823_21
	goto	u823_20
u823_21:
	goto	i1l3792
u823_20:
	goto	i1l28356
	line	310
	
i1l3792:	
	line	321
;bldc.c: 310: else
;bldc.c: 311: {
;bldc.c: 321: MotorFlags.bits.B3 = 0;
	movlb 0	; select bank0
	bcf	(_MotorFlags),3
	line	330
	
i1l28356:	
;bldc.c: 327: }
;bldc.c: 330: if (ui16_step_cnt >= 3 )
	movlw	high(03h)
	movlb 1	; select bank1
	subwf	(_ui16_step_cnt+1)^080h,w
	movlw	low(03h)
	skipnz
	subwf	(_ui16_step_cnt)^080h,w
	skipc
	goto	u824_21
	goto	u824_20
u824_21:
	goto	i1l28360
u824_20:
	line	334
	
i1l28358:	
;bldc.c: 332: {
;bldc.c: 334: MotorFlags.bits.B4 = 0 ;
	movlb 0	; select bank0
	bcf	(_MotorFlags),4
	line	339
	
i1l28360:	
;bldc.c: 336: }
;bldc.c: 339: BLDCWait( );
	fcall	_BLDCWait
	line	345
	
i1l28362:	
;bldc.c: 345: if( C1OUT )
	movlb 2	; select bank2
	btfss	(2190/8)^0100h,(2190)&7
	goto	u825_21
	goto	u825_20
u825_21:
	goto	i1l3795
u825_20:
	line	350
	
i1l28364:	
;bldc.c: 348: {
;bldc.c: 350: ui8_CompFlag = 0x00;
	clrf	(_ui8_CompFlag)^0100h
	line	352
;bldc.c: 352: }
	goto	i1l28522
	line	355
	
i1l3795:	
	line	361
;bldc.c: 355: else
;bldc.c: 356: {
;bldc.c: 361: ui8_CompFlag = 0x01;
	clrf	(_ui8_CompFlag)^0100h
	incf	(_ui8_CompFlag)^0100h,f
	goto	i1l28522
	line	383
	
i1l28366:	
;bldc.c: 377: {
;bldc.c: 383: ( ADCON0 = ( 0x0B << 2 ) | 0x01 );
	movlw	(02Dh)
	movlb 1	; select bank1
	movwf	(157)^080h	;volatile
	line	386
	
i1l28368:	
;bldc.c: 386: ADC_Wait( );
	fcall	i1_ADC_Wait
	line	390
	
i1l28370:	
;bldc.c: 390: ( GO_nDONE = 1 );
	movlb 1	; select bank1
	bsf	(1257/8)^080h,(1257)&7
	line	394
;bldc.c: 394: ui16_NTC_Temp_bldc = ADC_Read( );
	fcall	i1_ADC_Read
	movf	(1+(?i1_ADC_Read)),w
	movlb 4	; select bank4
	movwf	(_ui16_NTC_Temp_bldc+1)^0200h
	movf	(0+(?i1_ADC_Read)),w
	movwf	(_ui16_NTC_Temp_bldc)^0200h
	line	397
	
i1l28372:	
;bldc.c: 397: ( ADCON0 = ( 0x0C << 2 ) | 0x01 );
	movlw	(031h)
	movlb 1	; select bank1
	movwf	(157)^080h	;volatile
	line	400
;bldc.c: 400: ADC_Wait( );
	fcall	i1_ADC_Wait
	line	404
	
i1l28374:	
;bldc.c: 404: ( GO_nDONE = 1 );
	movlb 1	; select bank1
	bsf	(1257/8)^080h,(1257)&7
	line	407
	
i1l28376:	
;bldc.c: 407: ui8_sampleState++;
	movlb 2	; select bank2
	incf	(_ui8_sampleState)^0100h,f
	line	410
;bldc.c: 410: break;
	goto	i1l3799
	line	424
	
i1l28378:	
;bldc.c: 418: {
;bldc.c: 424: ( ADCON0 = ui8_IPhase_sel );
	movf	(_ui8_IPhase_sel)^0100h,w
	movlb 1	; select bank1
	movwf	(157)^080h	;volatile
	line	440
;bldc.c: 440: ui16_Ubat_bldc.b.lo = ADRESL;
	movf	(155)^080h,w	;volatile
	movlb 0	; select bank0
	movwf	(_ui16_Ubat_bldc)	;volatile
	line	441
;bldc.c: 441: ui16_Ubat_bldc.b.hi = ADRESH;
	movlb 1	; select bank1
	movf	(156)^080h,w	;volatile
	movlb 0	; select bank0
	movwf	0+(_ui16_Ubat_bldc)+01h	;volatile
	line	449
	
i1l28380:	
;bldc.c: 449: _nop();
	nop
	line	450
	
i1l28382:	
;bldc.c: 450: _nop();
	nop
	line	451
	
i1l28384:	
;bldc.c: 451: _nop();
	nop
	line	452
	
i1l28386:	
;bldc.c: 452: _nop();
	nop
	line	453
	
i1l28388:	
;bldc.c: 453: _nop();
	nop
	line	454
	
i1l28390:	
;bldc.c: 454: _nop();
	nop
	line	455
	
i1l28392:	
;bldc.c: 455: _nop();
	nop
	line	456
	
i1l28394:	
;bldc.c: 456: _nop();
	nop
	line	458
	
i1l28396:	
;bldc.c: 458: _nop();
	nop
	line	459
	
i1l28398:	
;bldc.c: 459: _nop();
	nop
	line	460
	
i1l28400:	
;bldc.c: 460: _nop();
	nop
	line	461
	
i1l28402:	
;bldc.c: 461: _nop();
	nop
	line	462
	
i1l28404:	
;bldc.c: 462: _nop();
	nop
	line	463
	
i1l28406:	
;bldc.c: 463: _nop();
	nop
	line	464
	
i1l28408:	
;bldc.c: 464: _nop();
	nop
	line	465
	
i1l28410:	
;bldc.c: 465: _nop();
	nop
	line	469
	
i1l28412:	
;bldc.c: 469: ADC_Wait();
	fcall	i1_ADC_Wait
	goto	i1l28374
	line	492
	
i1l28418:	
;bldc.c: 486: {
;bldc.c: 492: ui16_IPhase_bldc.w = ADC_Read( );
	fcall	i1_ADC_Read
	movf	(1+(?i1_ADC_Read)),w
	movwf	(_ui16_IPhase_bldc+1)	;volatile
	movf	(0+(?i1_ADC_Read)),w
	movwf	(_ui16_IPhase_bldc)	;volatile
	line	495
	
i1l28420:	
;bldc.c: 495: if (MotorFlags.bits.B6)
	btfss	(_MotorFlags),6
	goto	u826_21
	goto	u826_20
u826_21:
	goto	i1l28442
u826_20:
	goto	i1l28444
	line	507
	
i1l28424:	
;bldc.c: 505: {
;bldc.c: 507: if (ui16_IPhase_bldc.w > ( (unsigned int) ui8_current_cal[0]) )
	movlb 3	; select bank3
	movf	(_ui8_current_cal)^0180h,w
	movwf	(??_interrrupt_bldc+0)+0
	movlw	(0x0/2)
	movwf	(??_interrrupt_bldc+0)+0+1
	movlb 0	; select bank0
	movf	(_ui16_IPhase_bldc+1),w	;volatile
	subwf	1+(??_interrrupt_bldc+0)+0,w
	skipz
	goto	u827_25
	movf	(_ui16_IPhase_bldc),w	;volatile
	subwf	0+(??_interrrupt_bldc+0)+0,w
u827_25:
	skipnc
	goto	u827_21
	goto	u827_20
u827_21:
	goto	i1l28428
u827_20:
	line	511
	
i1l28426:	
;bldc.c: 509: {
;bldc.c: 511: ui16_IPhase1_bldc.w = (((ui16_IPhase_bldc.w - ui8_current_cal[0])*ui16_I_cal_Ph1)>>8);
	movf	(_ui16_IPhase_bldc+1),w	;volatile
	movwf	(?i1___wmul+1)
	movf	(_ui16_IPhase_bldc),w	;volatile
	movwf	(?i1___wmul)
	movlb 3	; select bank3
	movf	(_ui8_current_cal)^0180h,w
	subwf	(?i1___wmul),f
	skipc
	decf	(?i1___wmul+1),f
	movlb 2	; select bank2
	movf	(_ui16_I_cal_Ph1+1)^0100h,w
	movwf	1+(?i1___wmul)+02h
	movf	(_ui16_I_cal_Ph1)^0100h,w
	movwf	0+(?i1___wmul)+02h
	fcall	i1___wmul
	movf	(1+(?i1___wmul)),w
	movlb 3	; select bank3
	movwf	(_ui16_IPhase1_bldc)^0180h	;volatile
	clrf	(_ui16_IPhase1_bldc+1)^0180h	;volatile
	line	513
;bldc.c: 513: }
	goto	i1l28448
	line	519
	
i1l28428:	
;bldc.c: 516: else
;bldc.c: 517: {
;bldc.c: 519: ui16_IPhase1_bldc.w = 0 ;
	movlb 3	; select bank3
	clrf	(_ui16_IPhase1_bldc)^0180h	;volatile
	clrf	(_ui16_IPhase1_bldc+1)^0180h	;volatile
	goto	i1l28448
	line	533
	
i1l28430:	
;bldc.c: 531: {
;bldc.c: 533: if (ui16_IPhase_bldc.w > ( (unsigned int) ui8_current_cal[1] ) )
	movlb 3	; select bank3
	movf	0+(_ui8_current_cal)^0180h+01h,w
	movwf	(??_interrrupt_bldc+0)+0
	movlw	(0x0/2)
	movwf	(??_interrrupt_bldc+0)+0+1
	movlb 0	; select bank0
	movf	(_ui16_IPhase_bldc+1),w	;volatile
	subwf	1+(??_interrrupt_bldc+0)+0,w
	skipz
	goto	u828_25
	movf	(_ui16_IPhase_bldc),w	;volatile
	subwf	0+(??_interrrupt_bldc+0)+0,w
u828_25:
	skipnc
	goto	u828_21
	goto	u828_20
u828_21:
	goto	i1l28434
u828_20:
	line	537
	
i1l28432:	
;bldc.c: 535: {
;bldc.c: 537: ui16_IPhase2_bldc.w = (((ui16_IPhase_bldc.w - ui8_current_cal[1])*ui16_I_cal_Ph2)>>8);
	movf	(_ui16_IPhase_bldc+1),w	;volatile
	movwf	(?i1___wmul+1)
	movf	(_ui16_IPhase_bldc),w	;volatile
	movwf	(?i1___wmul)
	movlb 3	; select bank3
	movf	0+(_ui8_current_cal)^0180h+01h,w
	subwf	(?i1___wmul),f
	skipc
	decf	(?i1___wmul+1),f
	movlb 2	; select bank2
	movf	(_ui16_I_cal_Ph2+1)^0100h,w
	movwf	1+(?i1___wmul)+02h
	movf	(_ui16_I_cal_Ph2)^0100h,w
	movwf	0+(?i1___wmul)+02h
	fcall	i1___wmul
	movf	(1+(?i1___wmul)),w
	movlb 4	; select bank4
	movwf	(_ui16_IPhase2_bldc)^0200h	;volatile
	clrf	(_ui16_IPhase2_bldc+1)^0200h	;volatile
	line	540
;bldc.c: 540: }
	goto	i1l28448
	line	546
	
i1l28434:	
;bldc.c: 543: else
;bldc.c: 544: {
;bldc.c: 546: ui16_IPhase2_bldc.w = 0 ;
	movlb 4	; select bank4
	clrf	(_ui16_IPhase2_bldc)^0200h	;volatile
	clrf	(_ui16_IPhase2_bldc+1)^0200h	;volatile
	goto	i1l28448
	line	560
	
i1l28436:	
;bldc.c: 558: {
;bldc.c: 560: if (ui16_IPhase_bldc.w > ((unsigned int) ui8_current_cal[2]) )
	movlb 3	; select bank3
	movf	0+(_ui8_current_cal)^0180h+02h,w
	movwf	(??_interrrupt_bldc+0)+0
	movlw	(0x0/2)
	movwf	(??_interrrupt_bldc+0)+0+1
	movlb 0	; select bank0
	movf	(_ui16_IPhase_bldc+1),w	;volatile
	subwf	1+(??_interrrupt_bldc+0)+0,w
	skipz
	goto	u829_25
	movf	(_ui16_IPhase_bldc),w	;volatile
	subwf	0+(??_interrrupt_bldc+0)+0,w
u829_25:
	skipnc
	goto	u829_21
	goto	u829_20
u829_21:
	goto	i1l28440
u829_20:
	line	564
	
i1l28438:	
;bldc.c: 562: {
;bldc.c: 564: ui16_IPhase3_bldc.w = (((ui16_IPhase_bldc.w - ui8_current_cal[2])*ui16_I_cal_Ph3)>>8);
	movf	(_ui16_IPhase_bldc+1),w	;volatile
	movwf	(?i1___wmul+1)
	movf	(_ui16_IPhase_bldc),w	;volatile
	movwf	(?i1___wmul)
	movlb 3	; select bank3
	movf	0+(_ui8_current_cal)^0180h+02h,w
	subwf	(?i1___wmul),f
	skipc
	decf	(?i1___wmul+1),f
	movlb 2	; select bank2
	movf	(_ui16_I_cal_Ph3+1)^0100h,w
	movwf	1+(?i1___wmul)+02h
	movf	(_ui16_I_cal_Ph3)^0100h,w
	movwf	0+(?i1___wmul)+02h
	fcall	i1___wmul
	movf	(1+(?i1___wmul)),w
	movlb 4	; select bank4
	movwf	(_ui16_IPhase3_bldc)^0200h	;volatile
	clrf	(_ui16_IPhase3_bldc+1)^0200h	;volatile
	line	566
;bldc.c: 566: }
	goto	i1l28448
	line	572
	
i1l28440:	
;bldc.c: 569: else
;bldc.c: 570: {
;bldc.c: 572: ui16_IPhase3_bldc.w = 0 ;
	movlb 4	; select bank4
	clrf	(_ui16_IPhase3_bldc)^0200h	;volatile
	clrf	(_ui16_IPhase3_bldc+1)^0200h	;volatile
	goto	i1l28448
	line	586
	
i1l28442:	
;bldc.c: 584: {
;bldc.c: 586: ui16_IPhase1_bldc.w = 0;
	movlb 3	; select bank3
	clrf	(_ui16_IPhase1_bldc)^0180h	;volatile
	clrf	(_ui16_IPhase1_bldc+1)^0180h	;volatile
	line	587
;bldc.c: 587: ui16_IPhase2_bldc.w = 0;
	movlb 4	; select bank4
	clrf	(_ui16_IPhase2_bldc)^0200h	;volatile
	clrf	(_ui16_IPhase2_bldc+1)^0200h	;volatile
	line	588
;bldc.c: 588: ui16_IPhase3_bldc.w = 0;
	clrf	(_ui16_IPhase3_bldc)^0200h	;volatile
	clrf	(_ui16_IPhase3_bldc+1)^0200h	;volatile
	line	593
;bldc.c: 591: }
;bldc.c: 593: }
	goto	i1l28448
	line	499
	
i1l28444:	
	movlb 2	; select bank2
	movf	(_ui8_IPhase_sel)^0100h,w
	; Switch size 1, requested type "space"
; Number of cases is 3, Range of values is 9 to 53
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           10     6 (average)
; direct_byte           99     9 (fixed)
; jumptable            263     9 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	9^0	; case 9
	skipnz
	goto	i1l28424
	xorlw	17^9	; case 17
	skipnz
	goto	i1l28436
	xorlw	53^17	; case 53
	skipnz
	goto	i1l28430
	goto	i1l28442
	opt asmopt_on

	line	645
	
i1l28448:	
;bldc.c: 607: }
;bldc.c: 645: if( (ui16_IPhase_bldc.w > 300+40) )
	movlw	high(0155h)
	movlb 0	; select bank0
	subwf	(_ui16_IPhase_bldc+1),w	;volatile
	movlw	low(0155h)
	skipnz
	subwf	(_ui16_IPhase_bldc),w	;volatile
	skipc
	goto	u830_21
	goto	u830_20
u830_21:
	goto	i1l28454
u830_20:
	line	649
	
i1l28450:	
;bldc.c: 647: {
;bldc.c: 649: MotorFlags.bits.B0 = 1;
	bsf	(_MotorFlags),0
	line	653
	
i1l28452:	
;bldc.c: 653: LATC = LATC & 0b11100101;
	movlw	(0E5h)
	movlb 2	; select bank2
	andwf	(270)^0100h,f	;volatile
	line	673
;bldc.c: 659: }
	
i1l28454:	
;bldc.c: 667: }
;bldc.c: 673: ( ADCON0 = ui8_Ubemf_sel );
	movlb 2	; select bank2
	movf	(_ui8_Ubemf_sel)^0100h,w
	movlb 1	; select bank1
	movwf	(157)^080h	;volatile
	goto	i1l28412
	line	707
	
i1l28462:	
;bldc.c: 694: {
;bldc.c: 707: ui16_Ubemf_bldc.b.lo = ADRESL;
	movlb 1	; select bank1
	movf	(155)^080h,w	;volatile
	movlb 2	; select bank2
	movwf	(_ui16_Ubemf_bldc)^0100h	;volatile
	line	708
;bldc.c: 708: ui16_Ubemf_bldc.b.hi = ADRESH;
	movlb 1	; select bank1
	movf	(156)^080h,w	;volatile
	movlb 2	; select bank2
	movwf	0+(_ui16_Ubemf_bldc)^0100h+01h	;volatile
	line	712
	
i1l28464:	
;bldc.c: 712: ( GO_nDONE = 1 );
	movlb 1	; select bank1
	bsf	(1257/8)^080h,(1257)&7
	line	720
	
i1l28466:	
;bldc.c: 720: if( ( MotorFlags.bits.B6 ) && ( ui8_duty_cycle_BLDC != 0 ) )
	movlb 0	; select bank0
	btfss	(_MotorFlags),6
	goto	u831_21
	goto	u831_20
u831_21:
	goto	i1l28474
u831_20:
	
i1l28468:	
	movf	(_ui8_duty_cycle_BLDC),w
	skipz
	goto	u832_20
	goto	i1l28474
u832_20:
	line	724
	
i1l28470:	
;bldc.c: 722: {
;bldc.c: 724: if( rising_bemf_flag )
	btfss	(_rising_bemf_flag/8),(_rising_bemf_flag)&7
	goto	u833_21
	goto	u833_20
u833_21:
	goto	i1l28482
u833_20:
	line	728
	
i1l28472:	
;bldc.c: 726: {
;bldc.c: 728: if( ( ui16_Ubemf_bldc.w + 100 ) < ( ui16_Ubat_bldc.w ))
	movlb 2	; select bank2
	movf	(_ui16_Ubemf_bldc)^0100h,w	;volatile
	addlw	low(064h)
	movwf	(??_interrrupt_bldc+0)+0
	movlw	high(064h)
	addwfc	(_ui16_Ubemf_bldc+1)^0100h,w	;volatile
	movwf	1+(??_interrrupt_bldc+0)+0
	movlb 0	; select bank0
	movf	(_ui16_Ubat_bldc+1),w	;volatile
	subwf	1+(??_interrrupt_bldc+0)+0,w
	skipz
	goto	u834_25
	movf	(_ui16_Ubat_bldc),w	;volatile
	subwf	0+(??_interrrupt_bldc+0)+0,w
u834_25:
	skipnc
	goto	u834_21
	goto	u834_20
u834_21:
	goto	i1l3823
u834_20:
	line	735
	
i1l28474:	
;bldc.c: 731: {
;bldc.c: 735: ui8_sampleState++;
	movlb 2	; select bank2
	incf	(_ui8_sampleState)^0100h,f
	line	739
	
i1l28476:	
;bldc.c: 739: ( ADCON0 = ui8_UPhase_sel );
	movf	(_ui8_UPhase_sel)^0100h,w
	movlb 1	; select bank1
	movwf	(157)^080h	;volatile
	line	741
	
i1l28478:	
;bldc.c: 741: ADC_Wait( );
	fcall	i1_ADC_Wait
	line	745
	
i1l28480:	
;bldc.c: 745: ( GO_nDONE = 1 );
	movlb 1	; select bank1
	bsf	(1257/8)^080h,(1257)&7
	line	748
;bldc.c: 748: }
	goto	i1l3799
	line	766
	
i1l28482:	
;bldc.c: 763: else
;bldc.c: 764: {
;bldc.c: 766: if( ( ui16_Ubemf_bldc.w ) > ( 30 ) )
	movlw	high(01Fh)
	movlb 2	; select bank2
	subwf	(_ui16_Ubemf_bldc+1)^0100h,w	;volatile
	movlw	low(01Fh)
	skipnz
	subwf	(_ui16_Ubemf_bldc)^0100h,w	;volatile
	skipc
	goto	u835_21
	goto	u835_20
u835_21:
	goto	i1l3823
u835_20:
	goto	i1l28474
	line	797
	
i1l3823:	
	line	802
;bldc.c: 794: }
;bldc.c: 797: }
;bldc.c: 802: }
	goto	i1l3799
	line	866
	
i1l28500:	
;bldc.c: 866: ui8_BlankingCount = 0;
	clrf	(_ui8_BlankingCount)^0100h
	line	871
	
i1l28502:	
;bldc.c: 871: ui16_UPhase_bldc = ADC_Read( );
	fcall	i1_ADC_Read
	line	873
	
i1l28504:	
;bldc.c: 873: ( ADCON0 = ( 0x1D << 2 ) | 0x01 );
	movlw	(075h)
	movlb 1	; select bank1
	movwf	(157)^080h	;volatile
	line	876
	
i1l28506:	
;bldc.c: 876: ADC_Wait( );
	fcall	i1_ADC_Wait
	goto	i1l28376
	line	933
;bldc.c: 930: }
;bldc.c: 933: case 9:
	
i1l3831:	
	line	940
;bldc.c: 935: {
;bldc.c: 940: ( GO_nDONE = 1 );
	movlb 1	; select bank1
	bsf	(1257/8)^080h,(1257)&7
	line	944
	
i1l28516:	
;bldc.c: 944: ui16_CPU_Temp_bldc = ADC_Read( );
	fcall	i1_ADC_Read
	goto	i1l28376
	line	371
	
i1l28522:	
	movf	(_ui8_sampleState)^0100h,w
	; Switch size 1, requested type "space"
; Number of cases is 10, Range of values is 1 to 10
; switch strategies available:
; Name         Instructions Cycles
; direct_byte           29     9 (fixed)
; simple_byte           31    16 (average)
; jumptable            263     9 (fixed)
;	Chosen strategy is direct_byte

	addlw	-1
	skipc
goto i1l3799
	movwf fsr0l
	movlw	10
	subwf	fsr0l,w
skipnc
goto i1l3799
movlp high(i1S31330)
	lslf fsr0l,w
	addlw low(i1S31330)
	movwf pc
psect	swtext3,local,class=CONST,delta=2
global __pswtext3
__pswtext3:
i1S31330:
	ljmp	i1l28366
	ljmp	i1l28378
	ljmp	i1l28418
	ljmp	i1l28462
	ljmp	i1l28500
	ljmp	i1l28376
	ljmp	i1l28376
	ljmp	i1l28376
	ljmp	i1l3831
	ljmp	i1l3799
psect	text2069

	line	965
	
i1l3799:	
	line	1004
;bldc.c: 1004: if( MotorFlags.bits.B6 )
	movlb 0	; select bank0
	btfss	(_MotorFlags),6
	goto	u836_21
	goto	u836_20
u836_21:
	goto	i1l28550
u836_20:
	line	1008
	
i1l28524:	
;bldc.c: 1006: {
;bldc.c: 1008: if( MotorFlags.bits.B3)
	btfss	(_MotorFlags),3
	goto	u837_21
	goto	u837_20
u837_21:
	goto	i1l28552
u837_20:
	line	1012
	
i1l28526:	
;bldc.c: 1010: {
;bldc.c: 1012: bool_start_demand_mat = 1 ;
	movlb 2	; select bank2
	clrf	(_bool_start_demand_mat)^0100h
	incf	(_bool_start_demand_mat)^0100h,f
	line	1013
	
i1l28528:	
;bldc.c: 1014: ui16_Task_Cont500ms = 0 ;
	movlw	041h
	movlb 4	; select bank4
	movwf	(_ui8_fixed_start_speed_mat)^0200h
	clrf	(_ui8_fixed_start_speed_mat+1)^0200h
	line	1025
	
i1l28530:	
;bldc.c: 1025: if( ( comm_time % 4 ) == 0)
	movlb 0	; select bank0
	movf	(_comm_time),w
	andlw	03h
	btfss	status,2
	goto	u838_21
	goto	u838_20
u838_21:
	goto	i1l28552
u838_20:
	line	1029
	
i1l28532:	
;bldc.c: 1027: {
;bldc.c: 1029: if (MotorFlags.bits.B4)
	btfss	(_MotorFlags),4
	goto	u839_21
	goto	u839_20
u839_21:
	goto	i1l28538
u839_20:
	line	1033
	
i1l28534:	
;bldc.c: 1031: {
;bldc.c: 1033: if (ui16_Ubat_bldc.w)
	movf	(_ui16_Ubat_bldc+1),w	;volatile
	iorwf	(_ui16_Ubat_bldc),w	;volatile
	skipnz
	goto	u840_21
	goto	u840_20
u840_21:
	goto	i1l28542
u840_20:
	line	1038
	
i1l28536:	
;bldc.c: 1035: {
;bldc.c: 1038: ui32_tmp = ( 400 * 40 ) / ui16_Ubat_bldc.w ;
	movf	(_ui16_Ubat_bldc+1),w	;volatile
	movwf	(?i1___lwdiv+1)
	movf	(_ui16_Ubat_bldc),w	;volatile
	movwf	(?i1___lwdiv)
	movlw	low(03E80h)
	movwf	0+(?i1___lwdiv)+02h
	movlw	high(03E80h)
	movwf	(0+(?i1___lwdiv)+02h)+1
	fcall	i1___lwdiv
	movf	(0+(?i1___lwdiv)),w
	movwf	(interrrupt_bldc@ui32_tmp)
	movf	(1+(?i1___lwdiv)),w
	movwf	((interrrupt_bldc@ui32_tmp))+1
	clrf	2+((interrrupt_bldc@ui32_tmp))
	clrf	3+((interrrupt_bldc@ui32_tmp))
	goto	i1l28542
	line	1049
	
i1l28538:	
;bldc.c: 1046: else
;bldc.c: 1047: {
;bldc.c: 1049: if (ui16_Ubat_bldc.w)
	movf	(_ui16_Ubat_bldc+1),w	;volatile
	iorwf	(_ui16_Ubat_bldc),w	;volatile
	skipnz
	goto	u841_21
	goto	u841_20
u841_21:
	goto	i1l28542
u841_20:
	goto	i1l28536
	line	1062
	
i1l28542:	
;bldc.c: 1056: }
;bldc.c: 1059: }
;bldc.c: 1062: if( ui8_StartupPWM > ( (unsigned char) ui32_tmp) )
	movlb 2	; select bank2
	movf	(_ui8_StartupPWM)^0100h,w
	subwf	(interrrupt_bldc@ui32_tmp),w
	skipnc
	goto	u842_21
	goto	u842_20
u842_21:
	goto	i1l28546
u842_20:
	line	1066
	
i1l28544:	
;bldc.c: 1064: {
;bldc.c: 1066: ui8_StartupPWM--;
	decf	(_ui8_StartupPWM)^0100h,f
	line	1068
;bldc.c: 1068: }
	goto	i1l28548
	line	1074
	
i1l28546:	
;bldc.c: 1071: else
;bldc.c: 1072: {
;bldc.c: 1074: ui8_StartupPWM++;
	incf	(_ui8_StartupPWM)^0100h,f
	line	1079
	
i1l28548:	
;bldc.c: 1076: }
;bldc.c: 1079: CCPR1L = ui8_StartupPWM ;
	movf	(_ui8_StartupPWM)^0100h,w
	movlb 5	; select bank5
	movwf	(657)^0280h	;volatile
	goto	i1l28552
	line	1094
	
i1l28550:	
;bldc.c: 1090: else
;bldc.c: 1091: {
;bldc.c: 1094: CCPR1L = 0;
	movlb 5	; select bank5
	clrf	(657)^0280h	;volatile
	line	1100
	
i1l28552:	
;bldc.c: 1096: }
;bldc.c: 1100: if( 0 == ui8_BlankingCount )
	movlb 2	; select bank2
	movf	(_ui8_BlankingCount)^0100h,f
	skipz
	goto	u843_21
	goto	u843_20
u843_21:
	goto	i1l28568
u843_20:
	line	1112
	
i1l28554:	
;bldc.c: 1103: {
;bldc.c: 1112: if( ui8_CompFlag )
	movf	(_ui8_CompFlag)^0100h,w
	skipz
	goto	u844_20
	goto	i1l3844
u844_20:
	line	1120
	
i1l28556:	
;bldc.c: 1122: {
;bldc.c: 1126: ui8_zero_cros_cnt++;
	movlb 0	; select bank0
	btfsc	(_MotorFlags),5
	goto	u845_21
	goto	u845_20
u845_21:
	goto	i1l3844
u845_20:
	line	1128
	
i1l28558:	
;bldc.c: 1128: MotorFlags.bits.B5 = 1;
	bsf	(_MotorFlags),5
	line	1134
;bldc.c: 1131: }
;bldc.c: 1134: }
;bldc.c: 1137: CompFlag_prev = ui8_CompFlag;
	
i1l3844:	
	line	1285
;bldc.c: 1285: if( MotorFlags.bits.B5 )
	movlb 0	; select bank0
	btfss	(_MotorFlags),5
	goto	u846_21
	goto	u846_20
u846_21:
	goto	i1l3851
u846_20:
	line	1292
	
i1l28560:	
;bldc.c: 1288: {
;bldc.c: 1292: if( 0 == phase_delay_counter )
	movlb 4	; select bank4
	movf	((_phase_delay_counter+1)^0200h),w
	iorwf	((_phase_delay_counter)^0200h),w
	skipz
	goto	u847_21
	goto	u847_20
u847_21:
	goto	i1l28566
u847_20:
	line	1297
	
i1l28562:	
;bldc.c: 1295: {
;bldc.c: 1297: if( MotorFlags.bits.B3 == 0)
	movlb 0	; select bank0
	btfsc	(_MotorFlags),3
	goto	u848_21
	goto	u848_20
u848_21:
	goto	i1l3846
u848_20:
	line	1313
	
i1l28564:	
;bldc.c: 1299: {
;bldc.c: 1313: commutate( );
	fcall	i1_commutate
	goto	i1l3851
	line	1327
	
i1l28566:	
;bldc.c: 1324: else
;bldc.c: 1325: {
;bldc.c: 1327: phase_delay_counter--;
	movlw	-1
	addwf	(_phase_delay_counter)^0200h,f
	skipc
	decf	(_phase_delay_counter+1)^0200h,f
	goto	i1l3851
	line	1333
	
i1l3846:	
	line	1336
;bldc.c: 1329: }
;bldc.c: 1333: }
;bldc.c: 1336: }
	goto	i1l3851
	line	1342
	
i1l28568:	
;bldc.c: 1343: bemf_filter = 62;
	decf	(_ui8_BlankingCount)^0100h,f
	line	1355
	
i1l3851:	
	return
	opt stack 0
GLOBAL	__end_of_interrrupt_bldc
	__end_of_interrrupt_bldc:
;; =============== function _interrrupt_bldc ends ============

	signat	_interrrupt_bldc,88
	global	i1_commutate
psect	text2070,local,class=CODE,delta=2
global __ptext2070
__ptext2070:

;; *************** function i1_commutate *****************
;; Defined at:
;;		line 1364 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\bldc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, fsr1l, fsr1h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 1D/2
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0      12       0       0       0       0       0       0
;;      Totals:         0      12       0       0       0       0       0       0
;;Total ram usage:       12 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		i1___wmul
;; This function is called by:
;;		_interrrupt_bldc
;; This function uses a non-reentrant model
;;
psect	text2070
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\bldc.c"
	line	1364
	global	__size_ofi1_commutate
	__size_ofi1_commutate	equ	__end_ofi1_commutate-i1_commutate
	
i1_commutate:	
	opt	stack 7
; Regs used in i1_commutate: [wreg-status,0+pclath+cstack]
	line	1366
	
i1l28762:	
;bldc.c: 1366: if( ui16_step_cnt < 10 )
	movlw	high(0Ah)
	movlb 1	; select bank1
	subwf	(_ui16_step_cnt+1)^080h,w
	movlw	low(0Ah)
	skipnz
	subwf	(_ui16_step_cnt)^080h,w
	skipnc
	goto	u879_21
	goto	u879_20
u879_21:
	goto	i1l28766
u879_20:
	line	1370
	
i1l28764:	
;bldc.c: 1368: {
;bldc.c: 1370: ui16_step_cnt++;
	incf	(_ui16_step_cnt)^080h,f
	skipnz
	incf	(_ui16_step_cnt+1)^080h,f
	line	1375
	
i1l28766:	
;bldc.c: 1372: }
;bldc.c: 1375: B[ Bcnt++ ] = ( comm_time );
	movlb 2	; select bank2
	lslf	(_Bcnt)^0100h,w
	addlw	_B&0ffh
	movwf	fsr1l
	movlw 1	; select bank3/4
	movwf fsr1h	
	
	movlb 0	; select bank0
	movf	(_comm_time),w
	movwi	[0]fsr1
	movf	(_comm_time+1),w
	movwi	[1]fsr1
	
i1l28768:	
	movlb 2	; select bank2
	incf	(_Bcnt)^0100h,f
	line	1376
	
i1l28770:	
;bldc.c: 1376: Bcnt &= ( ( sizeof( B ) / sizeof( B[ 0 ] ) ) - 1 );
	movlw	(07h)
	andwf	(_Bcnt)^0100h,f
	line	1377
	
i1l28772:	
;bldc.c: 1377: ui16_speed_fil = ( B[ 0 ] + B[ 1 ] + B[ 2 ] + B[ 3 ] + B[ 4 ] + B[ 5 ] + B[ 6 ] + B[ 7 ] );
	movlw	(0Eh)
	addlw	_B&0ffh
	movwf	fsr1l
	movlw 1	; select bank3/4
	movwf fsr1h	
	
	movlw	(0Ch)
	addlw	_B&0ffh
	movwf	fsr0l
	movlw 1	; select bank3/4
	movwf fsr0h	
	
	movlb 3	; select bank3
	movf	0+(_B)^0180h+04h,w
	addwf	0+(_B)^0180h+02h,w
	movlb 0	; select bank0
	movwf	(??i1_commutate+0)+0
	movlb 3	; select bank3
	movf	1+(_B)^0180h+04h,w
	addwfc	1+(_B)^0180h+02h,w
	movlb 0	; select bank0
	movwf	1+(??i1_commutate+0)+0
	movlb 3	; select bank3
	movf	0+(_B)^0180h+06h,w
	movlb 0	; select bank0
	addwf	0+(??i1_commutate+0)+0,w
	movwf	(??i1_commutate+2)+0
	movlb 3	; select bank3
	movf	1+(_B)^0180h+06h,w
	movlb 0	; select bank0
	addwfc	1+(??i1_commutate+0)+0,w
	movwf	1+(??i1_commutate+2)+0
	movlb 3	; select bank3
	movf	0+(_B)^0180h+08h,w
	movlb 0	; select bank0
	addwf	0+(??i1_commutate+2)+0,w
	movwf	(??i1_commutate+4)+0
	movlb 3	; select bank3
	movf	1+(_B)^0180h+08h,w
	movlb 0	; select bank0
	addwfc	1+(??i1_commutate+2)+0,w
	movwf	1+(??i1_commutate+4)+0
	movlb 3	; select bank3
	movf	0+(_B)^0180h+0Ah,w
	movlb 0	; select bank0
	addwf	0+(??i1_commutate+4)+0,w
	movwf	(??i1_commutate+6)+0
	movlb 3	; select bank3
	movf	1+(_B)^0180h+0Ah,w
	movlb 0	; select bank0
	addwfc	1+(??i1_commutate+4)+0,w
	movwf	1+(??i1_commutate+6)+0
	moviw	[0]fsr0
	addwf	0+(??i1_commutate+6)+0,w
	movwf	(??i1_commutate+8)+0
	moviw	[1]fsr0
	addwfc	1+(??i1_commutate+6)+0,w
	movwf	(??i1_commutate+8)+0+1
	moviw	[0]fsr1
	addwf	0+(??i1_commutate+8)+0,w
	movwf	(??i1_commutate+10)+0
	moviw	[1]fsr1
	addwfc	1+(??i1_commutate+8)+0,w
	movwf	(??i1_commutate+10)+0+1
	movlb 3	; select bank3
	movf	(_B)^0180h,w
	movlb 0	; select bank0
	addwf	0+(??i1_commutate+10)+0,w
	movlb 2	; select bank2
	movwf	(_ui16_speed_fil)^0100h
	movlb 3	; select bank3
	movf	(_B+1)^0180h,w
	movlb 0	; select bank0
	addwfc	1+(??i1_commutate+10)+0,w
	movlb 2	; select bank2
	movwf	1+(_ui16_speed_fil)^0100h
	line	1378
	
i1l28774:	
;bldc.c: 1378: ui16_speed_rar = comm_time;
	movlb 0	; select bank0
	movf	(_comm_time+1),w
	movlb 3	; select bank3
	movwf	(_ui16_speed_rar+1)^0180h
	movlb 0	; select bank0
	movf	(_comm_time),w
	movlb 3	; select bank3
	movwf	(_ui16_speed_rar)^0180h
	line	1383
	
i1l28776:	
;bldc.c: 1383: MotorFlags.bits.B5 = 0;
	movlb 0	; select bank0
	bcf	(_MotorFlags),5
	line	1386
	
i1l28778:	
;bldc.c: 1390: bemf_filter = 62;
	movlb 2	; select bank2
	clrf	(_ui8_sampleState)^0100h
	incf	(_ui8_sampleState)^0100h,f
	line	1393
	
i1l28780:	
;bldc.c: 1393: MotorFlags.bits.B7 = 1;
	movlb 0	; select bank0
	bsf	(_MotorFlags),7
	line	1396
	
i1l28782:	
;bldc.c: 1396: if( MotorFlags.bits.B3 )
	btfss	(_MotorFlags),3
	goto	u880_21
	goto	u880_20
u880_21:
	goto	i1l3855
u880_20:
	line	1403
	
i1l28784:	
;bldc.c: 1398: {
;bldc.c: 1403: ui16_phase_advancement = ( ( ui16_speed_rar>>3 ) * 100 )>>7;
	movlb 3	; select bank3
	movf	(_ui16_speed_rar+1)^0180h,w
	movwf	(?i1___wmul+1)
	movf	(_ui16_speed_rar)^0180h,w
	movwf	(?i1___wmul)
	lsrf	(?i1___wmul+1),f
	rrf	(?i1___wmul),f
	lsrf	(?i1___wmul+1),f
	rrf	(?i1___wmul),f
	lsrf	(?i1___wmul+1),f
	rrf	(?i1___wmul),f
	movlw	064h
	movwf	0+(?i1___wmul)+02h
	clrf	1+(?i1___wmul)+02h
	fcall	i1___wmul
	movf	(1+(?i1___wmul)),w
	movlb 2	; select bank2
	movwf	(_ui16_phase_advancement+1)^0100h
	movf	(0+(?i1___wmul)),w
	movwf	(_ui16_phase_advancement)^0100h
	
i1l28786:	
	movlw	07h
	
u881_25:
	lsrf	(_ui16_phase_advancement+1)^0100h,f
	rrf	(_ui16_phase_advancement)^0100h,f
	decfsz	wreg,f
	goto	u881_25
	line	1407
	
i1l28788:	
;bldc.c: 1407: if( comm_time > ui16_phase_advancement )
	movlb 0	; select bank0
	movf	(_comm_time+1),w
	movlb 2	; select bank2
	subwf	(_ui16_phase_advancement+1)^0100h,w
	skipz
	goto	u882_25
	movlb 0	; select bank0
	movf	(_comm_time),w
	movlb 2	; select bank2
	subwf	(_ui16_phase_advancement)^0100h,w
u882_25:
	skipnc
	goto	u882_21
	goto	u882_20
u882_21:
	goto	i1l28792
u882_20:
	line	1411
	
i1l28790:	
;bldc.c: 1409: {
;bldc.c: 1411: phase_delay_counter = ( ( comm_time ) - ui16_phase_advancement );
	movlb 0	; select bank0
	movf	(_comm_time+1),w
	movlb 4	; select bank4
	movwf	(_phase_delay_counter+1)^0200h
	movlb 0	; select bank0
	movf	(_comm_time),w
	movlb 4	; select bank4
	movwf	(_phase_delay_counter)^0200h
	movlb 2	; select bank2
	movf	(_ui16_phase_advancement)^0100h,w
	movlb 4	; select bank4
	subwf	(_phase_delay_counter)^0200h,f
	movlb 2	; select bank2
	movf	(_ui16_phase_advancement+1)^0100h,w
	movlb 4	; select bank4
	subwfb	(_phase_delay_counter+1)^0200h,f
	line	1413
;bldc.c: 1413: }
	goto	i1l28816
	line	1419
	
i1l28792:	
;bldc.c: 1416: else
;bldc.c: 1417: {
;bldc.c: 1419: phase_delay_counter = 0;
	movlb 4	; select bank4
	clrf	(_phase_delay_counter)^0200h
	clrf	(_phase_delay_counter+1)^0200h
	goto	i1l28816
	line	1451
	
i1l28794:	
;bldc.c: 1449: {
;bldc.c: 1451: ui16_comm_time_max = ( unsigned short ) ( ( 10 * 20000UL ) / ( 600 * 2 ) ) ;
	movlw	0A6h
	movlb 2	; select bank2
	movwf	(_ui16_comm_time_max)^0100h
	clrf	(_ui16_comm_time_max+1)^0100h
	line	1452
;bldc.c: 1452: break;
	goto	i1l28818
	line	1461
	
i1l28796:	
;bldc.c: 1459: {
;bldc.c: 1461: ui16_comm_time_max = ( unsigned short ) ( ( 10 * 20000UL ) / ( 200 * 2 ) ) ;
	movlw	low(01F4h)
	movlb 2	; select bank2
	movwf	(_ui16_comm_time_max)^0100h
	movlw	high(01F4h)
	movwf	((_ui16_comm_time_max)^0100h)+1
	line	1462
;bldc.c: 1462: break;
	goto	i1l28818
	line	1471
	
i1l28798:	
;bldc.c: 1469: {
;bldc.c: 1471: ui16_comm_time_max = ( unsigned short ) ( ( 10 * 20000UL ) / ( 800 * 2 ) ) ;
	movlw	07Dh
	movlb 2	; select bank2
	movwf	(_ui16_comm_time_max)^0100h
	clrf	(_ui16_comm_time_max+1)^0100h
	line	1472
;bldc.c: 1472: break;
	goto	i1l28818
	line	1481
	
i1l28800:	
;bldc.c: 1479: {
;bldc.c: 1481: ui16_comm_time_max = ( unsigned short ) ( ( 10 * 20000UL ) / ( 1200 * 2 ) ) ;
	movlw	053h
	movlb 2	; select bank2
	movwf	(_ui16_comm_time_max)^0100h
	clrf	(_ui16_comm_time_max+1)^0100h
	line	1482
;bldc.c: 1482: break;
	goto	i1l28818
	line	1491
	
i1l28802:	
;bldc.c: 1489: {
;bldc.c: 1491: ui16_comm_time_max = ( unsigned short ) ( ( 10 * 20000UL ) / ( 2400 * 2 ) ) ;
	movlw	029h
	movlb 2	; select bank2
	movwf	(_ui16_comm_time_max)^0100h
	clrf	(_ui16_comm_time_max+1)^0100h
	line	1492
;bldc.c: 1492: break;
	goto	i1l28818
	line	1501
	
i1l28804:	
;bldc.c: 1499: {
;bldc.c: 1501: ui16_comm_time_max = ( unsigned short ) ( ( 10 * 20000UL ) / ( 3000 * 2 ) ) ;
	movlw	021h
	movlb 2	; select bank2
	movwf	(_ui16_comm_time_max)^0100h
	clrf	(_ui16_comm_time_max+1)^0100h
	line	1502
;bldc.c: 1502: break;
	goto	i1l28818
	line	1443
	
i1l28816:	
	; Switch on 2 bytes has been partitioned into a top level switch of size 1, and 1 sub-switches
; Switch size 1, requested type "space"
; Number of cases is 1, Range of values is 0 to 0
; switch strategies available:
; Name         Instructions Cycles
; simple_byte            4     3 (average)
; direct_byte            8     6 (fixed)
; jumptable            260     6 (fixed)
; rangetable             5     4 (fixed)
; spacedrange            7     6 (fixed)
; locatedrange           1     3 (fixed)
;	Chosen strategy is simple_byte

	movlb 1	; select bank1
	movf (_ui16_step_cnt+1)^080h,w
	opt asmopt_off
	xorlw	0^0	; case 0
	skipnz
	goto	i1l31332
	goto	i1l28804
	opt asmopt_on
	
i1l31332:	
; Switch size 1, requested type "space"
; Number of cases is 10, Range of values is 0 to 9
; switch strategies available:
; Name         Instructions Cycles
; direct_byte           26     6 (fixed)
; simple_byte           31    16 (average)
; jumptable            260     6 (fixed)
; rangetable            14     4 (fixed)
; spacedrange           25     6 (fixed)
; locatedrange          10     3 (fixed)
;	Chosen strategy is direct_byte

	movf (_ui16_step_cnt)^080h,w
	movwf fsr0l
	movlw	10
	subwf	fsr0l,w
skipnc
goto i1l28804
movlp high(i1S31334)
	lslf fsr0l,w
	addlw low(i1S31334)
	movwf pc
psect	swtext4,local,class=CONST,delta=2
global __pswtext4
__pswtext4:
i1S31334:
	ljmp	i1l28794
	ljmp	i1l28796
	ljmp	i1l28798
	ljmp	i1l28800
	ljmp	i1l28802
	ljmp	i1l28804
	ljmp	i1l28804
	ljmp	i1l28804
	ljmp	i1l28804
	ljmp	i1l28804
psect	text2070

	line	1559
	
i1l28818:	
;bldc.c: 1559: if( ui16_comm_time_max < ( unsigned short ) ( ( 10 * 20000UL ) / ( 3000 * 2 ) ) )
	movlw	high(021h)
	subwf	(_ui16_comm_time_max+1)^0100h,w
	movlw	low(021h)
	skipnz
	subwf	(_ui16_comm_time_max)^0100h,w
	skipnc
	goto	u883_21
	goto	u883_20
u883_21:
	goto	i1l28822
u883_20:
	line	1563
	
i1l28820:	
;bldc.c: 1561: {
;bldc.c: 1563: ui16_comm_time_max = ( unsigned short ) ( ( 10 * 20000UL ) / ( 3000 * 2 ) );
	movlw	021h
	movwf	(_ui16_comm_time_max)^0100h
	clrf	(_ui16_comm_time_max+1)^0100h
	line	1569
	
i1l28822:	
;bldc.c: 1565: }
;bldc.c: 1569: if (MotorFlags.bits.B4)
	movlb 0	; select bank0
	btfss	(_MotorFlags),4
	goto	u884_21
	goto	u884_20
u884_21:
	goto	i1l3873
u884_20:
	line	1574
	
i1l28824:	
;bldc.c: 1571: {
;bldc.c: 1574: ui8_StartupPWM = 10 ;
	movlw	(0Ah)
	movlb 2	; select bank2
	movwf	(_ui8_StartupPWM)^0100h
	goto	i1l3873
	line	1576
	
i1l3872:	
	line	1579
;bldc.c: 1576: }
;bldc.c: 1579: }
	goto	i1l3873
	line	1582
	
i1l3855:	
	line	1587
;bldc.c: 1582: else
;bldc.c: 1583: {
;bldc.c: 1587: if( rising_bemf_flag)
	btfss	(_rising_bemf_flag/8),(_rising_bemf_flag)&7
	goto	u885_21
	goto	u885_20
u885_21:
	goto	i1l28828
u885_20:
	line	1592
	
i1l28826:	
;bldc.c: 1589: {
;bldc.c: 1592: phase_delay_counter = 0;
	movlb 4	; select bank4
	clrf	(_phase_delay_counter)^0200h
	clrf	(_phase_delay_counter+1)^0200h
	line	1594
;bldc.c: 1594: }
	goto	i1l28832
	line	1601
	
i1l28828:	
;bldc.c: 1597: else
;bldc.c: 1598: {
;bldc.c: 1601: phase_delay_counter = comm_time>>1;
	movf	(_comm_time+1),w
	movlb 4	; select bank4
	movwf	(_phase_delay_counter+1)^0200h
	movlb 0	; select bank0
	movf	(_comm_time),w
	movlb 4	; select bank4
	movwf	(_phase_delay_counter)^0200h
	
i1l28830:	
	lsrf	(_phase_delay_counter+1)^0200h,f
	rrf	(_phase_delay_counter)^0200h,f
	line	1610
	
i1l28832:	
;bldc.c: 1615: phase_delay_counter_debug = phase_delay_counter;
	clrf	(_phase_delay_counter)^0200h
	clrf	(_phase_delay_counter+1)^0200h
	line	1621
	
i1l28834:	
;bldc.c: 1621: ui16_comm_time_max = ( unsigned short ) ( ( 10 * 20000UL ) / ( 300 * 2 ) );
	movlw	low(014Dh)
	movlb 2	; select bank2
	movwf	(_ui16_comm_time_max)^0100h
	movlw	high(014Dh)
	movwf	((_ui16_comm_time_max)^0100h)+1
	line	1626
	
i1l28836:	
;bldc.c: 1626: if( MotorFlags.bits.B6 )
	movlb 0	; select bank0
	btfss	(_MotorFlags),6
	goto	u886_21
	goto	u886_20
u886_21:
	goto	i1l28856
u886_20:
	line	1632
	
i1l28838:	
;bldc.c: 1628: {
;bldc.c: 1632: if (CCPR1L == ui8_duty_cycle_BLDC)
	movlb 5	; select bank5
	movf	(657)^0280h,w	;volatile
	movlb 0	; select bank0
	xorwf	(_ui8_duty_cycle_BLDC),w
	skipz
	goto	u887_21
	goto	u887_20
u887_21:
	goto	i1l28842
u887_20:
	goto	i1l3873
	line	1644
	
i1l28842:	
;bldc.c: 1641: else
;bldc.c: 1642: {
;bldc.c: 1644: if( CCPR1L > ui8_duty_cycle_BLDC )
	movlb 5	; select bank5
	movf	(657)^0280h,w	;volatile
	movlb 0	; select bank0
	subwf	(_ui8_duty_cycle_BLDC),w
	skipnc
	goto	u888_21
	goto	u888_20
u888_21:
	goto	i1l28850
u888_20:
	line	1649
	
i1l28844:	
;bldc.c: 1647: {
;bldc.c: 1649: CCPR1L--;
	movlb 5	; select bank5
	decf	(657)^0280h,f	;volatile
	line	1651
	
i1l28846:	
;bldc.c: 1651: if (CCPR1L > ui8_duty_cycle_BLDC)
	movf	(657)^0280h,w	;volatile
	movlb 0	; select bank0
	subwf	(_ui8_duty_cycle_BLDC),w
	skipnc
	goto	u889_21
	goto	u889_20
u889_21:
	goto	i1l3872
u889_20:
	line	1655
	
i1l28848:	
;bldc.c: 1653: {
;bldc.c: 1655: CCPR1L--;
	movlb 5	; select bank5
	decf	(657)^0280h,f	;volatile
	goto	i1l3873
	line	1666
	
i1l28850:	
;bldc.c: 1663: else
;bldc.c: 1664: {
;bldc.c: 1666: CCPR1L++;
	movlb 5	; select bank5
	incf	(657)^0280h,f	;volatile
	line	1668
	
i1l28852:	
;bldc.c: 1668: if (CCPR1L < ui8_duty_cycle_BLDC)
	movlb 0	; select bank0
	movf	(_ui8_duty_cycle_BLDC),w
	movlb 5	; select bank5
	subwf	(657)^0280h,w	;volatile
	skipnc
	goto	u890_21
	goto	u890_20
u890_21:
	goto	i1l3872
u890_20:
	line	1672
	
i1l28854:	
;bldc.c: 1670: {
;bldc.c: 1672: CCPR1L++;
	incf	(657)^0280h,f	;volatile
	goto	i1l3873
	line	1706
	
i1l28856:	
;bldc.c: 1703: else
;bldc.c: 1704: {
;bldc.c: 1706: CCPR1L = 0;
	movlb 5	; select bank5
	clrf	(657)^0280h	;volatile
	line	1712
	
i1l3873:	
	line	1715
;bldc.c: 1708: }
;bldc.c: 1712: }
;bldc.c: 1715: comm_time = 0;
	movlb 0	; select bank0
	clrf	(_comm_time)
	clrf	(_comm_time+1)
	line	1719
	
i1l28858:	
;bldc.c: 1719: if( comm_state == 0xff )
	movf	(_comm_state),w
	xorlw	0FFh&0ffh
	skipz
	goto	u891_21
	goto	u891_20
u891_21:
	goto	i1l29006
u891_20:
	line	1723
	
i1l28860:	
;bldc.c: 1721: {
;bldc.c: 1723: comm_state = 6;
	movlw	(06h)
	movwf	(_comm_state)
	goto	i1l29006
	line	1741
	
i1l28862:	
;bldc.c: 1735: {
;bldc.c: 1741: PSTR1CON = 0b00000001;
	movlw	(01h)
	movlb 5	; select bank5
	movwf	(662)^0280h	;volatile
	line	1744
	
i1l28864:	
;bldc.c: 1744: if( MotorFlags.bits.B6 )
	movlb 0	; select bank0
	btfss	(_MotorFlags),6
	goto	u892_21
	goto	u892_20
u892_21:
	goto	i1l28868
u892_20:
	line	1748
	
i1l28866:	
;bldc.c: 1746: {
;bldc.c: 1748: LATC = ( LATC & 0b11100101 ) | 0b00001010;
	movlb 2	; select bank2
	movf	(270)^0100h,w
	andlw	0E5h
	iorlw	0Ah
	movwf	(270)^0100h	;volatile
	line	1754
	
i1l28868:	
;bldc.c: 1750: }
;bldc.c: 1754: LATB2 = 0;
	movlb 2	; select bank2
	bcf	(2154/8)^0100h,(2154)&7
	line	1755
	
i1l28870:	
;bldc.c: 1755: LATB1 = 0;
	bcf	(2153/8)^0100h,(2153)&7
	line	1756
	
i1l28872:	
;bldc.c: 1756: ui8_IPhase_sel = ( 0x02 << 2 ) | 0x01;
	movlw	(09h)
	movwf	(_ui8_IPhase_sel)^0100h
	line	1757
	
i1l28874:	
;bldc.c: 1757: ui8_UPhase_sel = ( 0x00 << 2 ) | 0x01;
	clrf	(_ui8_UPhase_sel)^0100h
	incf	(_ui8_UPhase_sel)^0100h,f
	line	1758
	
i1l28876:	
;bldc.c: 1758: ui8_Ubemf_sel = ( 0x09 << 2 ) | 0x01;
	movlw	(025h)
	movwf	(_ui8_Ubemf_sel)^0100h
	line	1759
	
i1l28878:	
;bldc.c: 1759: CM1CON1 = 0x02;
	movlw	(02h)
	movwf	(274)^0100h	;volatile
	line	1772
	
i1l28880:	
;bldc.c: 1772: CM1CON0 = 0x84;
	movlw	(084h)
	movwf	(273)^0100h	;volatile
	line	1773
	
i1l28882:	
;bldc.c: 1773: rising_bemf_flag = 1;
	movlb 0	; select bank0
	bsf	(_rising_bemf_flag/8),(_rising_bemf_flag)&7
	line	1774
	
i1l28884:	
;bldc.c: 1774: comm_state = 6;
	movlw	(06h)
	movwf	(_comm_state)
	line	1780
;bldc.c: 1780: break;
	goto	i1l29008
	line	1795
	
i1l28886:	
;bldc.c: 1790: {
;bldc.c: 1795: PSTR1CON = 0b00000001;
	movlw	(01h)
	movlb 5	; select bank5
	movwf	(662)^0280h	;volatile
	line	1798
	
i1l28888:	
;bldc.c: 1798: if( MotorFlags.bits.B6 )
	movlb 0	; select bank0
	btfss	(_MotorFlags),6
	goto	u893_21
	goto	u893_20
u893_21:
	goto	i1l28892
u893_20:
	line	1802
	
i1l28890:	
;bldc.c: 1800: {
;bldc.c: 1802: LATC = ( LATC & 0b11100101 ) | 0b00010010;
	movlb 2	; select bank2
	movf	(270)^0100h,w
	andlw	0E5h
	iorlw	012h
	movwf	(270)^0100h	;volatile
	line	1808
	
i1l28892:	
;bldc.c: 1804: }
;bldc.c: 1808: LATB2 = 1;
	movlb 2	; select bank2
	bsf	(2154/8)^0100h,(2154)&7
	line	1809
	
i1l28894:	
;bldc.c: 1809: LATB1 = 0;
	bcf	(2153/8)^0100h,(2153)&7
	line	1810
	
i1l28896:	
;bldc.c: 1810: ui8_IPhase_sel = ( 0x02 << 2 ) | 0x01;
	movlw	(09h)
	movwf	(_ui8_IPhase_sel)^0100h
	line	1811
	
i1l28898:	
;bldc.c: 1811: ui8_UPhase_sel = ( 0x00 << 2 ) | 0x01;
	clrf	(_ui8_UPhase_sel)^0100h
	incf	(_ui8_UPhase_sel)^0100h,f
	line	1812
	
i1l28900:	
;bldc.c: 1812: ui8_Ubemf_sel = ( 0x01 << 2 ) | 0x01;
	movlw	(05h)
	movwf	(_ui8_Ubemf_sel)^0100h
	line	1813
	
i1l28902:	
;bldc.c: 1813: CM1CON1 = 0x01;
	movlw	(01h)
	movwf	(274)^0100h	;volatile
	line	1827
	
i1l28904:	
;bldc.c: 1827: CM1CON0 = 0x94;
	movlw	(094h)
	movwf	(273)^0100h	;volatile
	line	1828
	
i1l28906:	
;bldc.c: 1828: rising_bemf_flag = 0;
	movlb 0	; select bank0
	bcf	(_rising_bemf_flag/8),(_rising_bemf_flag)&7
	line	1829
	
i1l28908:	
;bldc.c: 1829: comm_state = 1;
	clrf	(_comm_state)
	incf	(_comm_state),f
	line	1837
;bldc.c: 1837: break;
	goto	i1l29008
	line	1852
	
i1l28910:	
;bldc.c: 1847: {
;bldc.c: 1852: PSTR1CON = 0b00000010;
	movlw	(02h)
	movlb 5	; select bank5
	movwf	(662)^0280h	;volatile
	line	1855
	
i1l28912:	
;bldc.c: 1855: if( MotorFlags.bits.B6 )
	movlb 0	; select bank0
	btfss	(_MotorFlags),6
	goto	u894_21
	goto	u894_20
u894_21:
	goto	i1l28916
u894_20:
	line	1859
	
i1l28914:	
;bldc.c: 1857: {
;bldc.c: 1859: LATC = ( LATC & 0b11100101 ) | 0b00011000;
	movlb 2	; select bank2
	movf	(270)^0100h,w
	andlw	0E5h
	iorlw	018h
	movwf	(270)^0100h	;volatile
	line	1864
	
i1l28916:	
;bldc.c: 1861: }
;bldc.c: 1864: LATC2 = 0;
	movlb 2	; select bank2
	bcf	(2162/8)^0100h,(2162)&7
	line	1866
	
i1l28918:	
;bldc.c: 1866: LATB1 = 0;
	bcf	(2153/8)^0100h,(2153)&7
	line	1867
	
i1l28920:	
;bldc.c: 1867: ui8_IPhase_sel = ( 0x0D << 2 ) | 0x01;
	movlw	(035h)
	movwf	(_ui8_IPhase_sel)^0100h
	line	1868
	
i1l28922:	
;bldc.c: 1868: ui8_UPhase_sel = ( 0x01 << 2 ) | 0x01;
	movlw	(05h)
	movwf	(_ui8_UPhase_sel)^0100h
	line	1869
	
i1l28924:	
;bldc.c: 1869: ui8_Ubemf_sel = ( 0x00 << 2 ) | 0x01;
	clrf	(_ui8_Ubemf_sel)^0100h
	incf	(_ui8_Ubemf_sel)^0100h,f
	line	1870
	
i1l28926:	
;bldc.c: 1870: CM1CON1 = 0x00;
	clrf	(274)^0100h	;volatile
	line	1884
;bldc.c: 1884: CM1CON0 = 0x84;
	movlw	(084h)
	movwf	(273)^0100h	;volatile
	line	1885
	
i1l28928:	
;bldc.c: 1885: rising_bemf_flag = 1;
	movlb 0	; select bank0
	bsf	(_rising_bemf_flag/8),(_rising_bemf_flag)&7
	line	1886
;bldc.c: 1886: comm_state = 2;
	movlw	(02h)
	movwf	(_comm_state)
	line	1891
;bldc.c: 1891: break;
	goto	i1l29008
	line	1907
	
i1l28930:	
;bldc.c: 1901: {
;bldc.c: 1907: PSTR1CON = 0b00000010;
	movlw	(02h)
	movlb 5	; select bank5
	movwf	(662)^0280h	;volatile
	line	1910
	
i1l28932:	
;bldc.c: 1910: if( MotorFlags.bits.B6 )
	movlb 0	; select bank0
	btfss	(_MotorFlags),6
	goto	u895_21
	goto	u895_20
u895_21:
	goto	i1l28936
u895_20:
	line	1914
	
i1l28934:	
;bldc.c: 1912: {
;bldc.c: 1914: LATC = ( LATC & 0b11100101 ) | 0b00001010;
	movlb 2	; select bank2
	movf	(270)^0100h,w
	andlw	0E5h
	iorlw	0Ah
	movwf	(270)^0100h	;volatile
	line	1919
	
i1l28936:	
;bldc.c: 1916: }
;bldc.c: 1919: LATC2 = 0;
	movlb 2	; select bank2
	bcf	(2162/8)^0100h,(2162)&7
	line	1921
	
i1l28938:	
;bldc.c: 1921: LATB1 = 1;
	bsf	(2153/8)^0100h,(2153)&7
	line	1922
	
i1l28940:	
;bldc.c: 1922: ui8_IPhase_sel = ( 0x0D << 2 ) | 0x01;
	movlw	(035h)
	movwf	(_ui8_IPhase_sel)^0100h
	line	1923
	
i1l28942:	
;bldc.c: 1923: ui8_UPhase_sel = ( 0x01 << 2 ) | 0x01;
	movlw	(05h)
	movwf	(_ui8_UPhase_sel)^0100h
	line	1924
	
i1l28944:	
;bldc.c: 1924: ui8_Ubemf_sel = ( 0x09 << 2 ) | 0x01;
	movlw	(025h)
	movwf	(_ui8_Ubemf_sel)^0100h
	line	1925
	
i1l28946:	
;bldc.c: 1925: CM1CON1 = 0x02;
	movlw	(02h)
	movwf	(274)^0100h	;volatile
	line	1939
	
i1l28948:	
;bldc.c: 1939: CM1CON0 = 0x94;
	movlw	(094h)
	movwf	(273)^0100h	;volatile
	line	1940
	
i1l28950:	
;bldc.c: 1940: rising_bemf_flag = 0;
	movlb 0	; select bank0
	bcf	(_rising_bemf_flag/8),(_rising_bemf_flag)&7
	line	1941
	
i1l28952:	
;bldc.c: 1941: comm_state = 3;
	movlw	(03h)
	movwf	(_comm_state)
	line	1946
;bldc.c: 1946: break;
	goto	i1l29008
	line	1961
	
i1l28954:	
;bldc.c: 1955: {
;bldc.c: 1961: PSTR1CON = 0b00000100;
	movlw	(04h)
	movlb 5	; select bank5
	movwf	(662)^0280h	;volatile
	line	1964
	
i1l28956:	
;bldc.c: 1964: if( MotorFlags.bits.B6 )
	movlb 0	; select bank0
	btfss	(_MotorFlags),6
	goto	u896_21
	goto	u896_20
u896_21:
	goto	i1l28960
u896_20:
	line	1968
	
i1l28958:	
;bldc.c: 1966: {
;bldc.c: 1968: LATC = ( LATC & 0b11100101 ) | 0b00010010;
	movlb 2	; select bank2
	movf	(270)^0100h,w
	andlw	0E5h
	iorlw	012h
	movwf	(270)^0100h	;volatile
	line	1973
	
i1l28960:	
;bldc.c: 1970: }
;bldc.c: 1973: LATC2 = 0;
	movlb 2	; select bank2
	bcf	(2162/8)^0100h,(2162)&7
	line	1974
	
i1l28962:	
;bldc.c: 1974: LATB2 = 0;
	bcf	(2154/8)^0100h,(2154)&7
	line	1976
	
i1l28964:	
;bldc.c: 1976: ui8_IPhase_sel = ( 0x04 << 2 ) | 0x01;
	movlw	(011h)
	movwf	(_ui8_IPhase_sel)^0100h
	line	1977
	
i1l28966:	
;bldc.c: 1977: ui8_UPhase_sel = ( 0x09 << 2 ) | 0x01;
	movlw	(025h)
	movwf	(_ui8_UPhase_sel)^0100h
	line	1978
	
i1l28968:	
;bldc.c: 1978: ui8_Ubemf_sel = ( 0x01 << 2 ) | 0x01;
	movlw	(05h)
	movwf	(_ui8_Ubemf_sel)^0100h
	line	1979
	
i1l28970:	
;bldc.c: 1979: CM1CON1 = 0x01;
	movlw	(01h)
	movwf	(274)^0100h	;volatile
	line	1994
	
i1l28972:	
;bldc.c: 1994: CM1CON0 = 0x84;
	movlw	(084h)
	movwf	(273)^0100h	;volatile
	line	1995
	
i1l28974:	
;bldc.c: 1995: rising_bemf_flag = 1;
	movlb 0	; select bank0
	bsf	(_rising_bemf_flag/8),(_rising_bemf_flag)&7
	line	1996
	
i1l28976:	
;bldc.c: 1996: comm_state = 4;
	movlw	(04h)
	movwf	(_comm_state)
	line	2001
;bldc.c: 2001: break;
	goto	i1l29008
	line	2016
	
i1l28978:	
;bldc.c: 2011: {
;bldc.c: 2016: PSTR1CON = 0b00000100;
	movlw	(04h)
	movlb 5	; select bank5
	movwf	(662)^0280h	;volatile
	line	2019
	
i1l28980:	
;bldc.c: 2019: if( MotorFlags.bits.B6 )
	movlb 0	; select bank0
	btfss	(_MotorFlags),6
	goto	u897_21
	goto	u897_20
u897_21:
	goto	i1l28984
u897_20:
	line	2023
	
i1l28982:	
;bldc.c: 2021: {
;bldc.c: 2023: LATC = ( LATC & 0b11100101 ) | 0b00011000;
	movlb 2	; select bank2
	movf	(270)^0100h,w
	andlw	0E5h
	iorlw	018h
	movwf	(270)^0100h	;volatile
	line	2028
	
i1l28984:	
;bldc.c: 2025: }
;bldc.c: 2028: LATC2 = 1;
	movlb 2	; select bank2
	bsf	(2162/8)^0100h,(2162)&7
	line	2029
	
i1l28986:	
;bldc.c: 2029: LATB2 = 0;
	bcf	(2154/8)^0100h,(2154)&7
	line	2031
	
i1l28988:	
;bldc.c: 2031: ui8_IPhase_sel = ( 0x04 << 2 ) | 0x01;
	movlw	(011h)
	movwf	(_ui8_IPhase_sel)^0100h
	line	2032
	
i1l28990:	
;bldc.c: 2032: ui8_UPhase_sel = ( 0x09 << 2 ) | 0x01;
	movlw	(025h)
	movwf	(_ui8_UPhase_sel)^0100h
	line	2033
	
i1l28992:	
;bldc.c: 2033: ui8_Ubemf_sel = ( 0x00 << 2 ) | 0x01;
	clrf	(_ui8_Ubemf_sel)^0100h
	incf	(_ui8_Ubemf_sel)^0100h,f
	line	2034
	
i1l28994:	
;bldc.c: 2034: CM1CON1 = 0x00;
	clrf	(274)^0100h	;volatile
	line	2048
;bldc.c: 2048: CM1CON0 = 0x94;
	movlw	(094h)
	movwf	(273)^0100h	;volatile
	line	2049
	
i1l28996:	
;bldc.c: 2049: rising_bemf_flag = 0;
	movlb 0	; select bank0
	bcf	(_rising_bemf_flag/8),(_rising_bemf_flag)&7
	line	2050
;bldc.c: 2050: comm_state = 5;
	movlw	(05h)
	movwf	(_comm_state)
	line	2055
;bldc.c: 2055: break;
	goto	i1l29008
	line	2066
	
i1l28998:	
;bldc.c: 2064: {
;bldc.c: 2066: PSTR1CON = 0x00;
	movlb 5	; select bank5
	clrf	(662)^0280h	;volatile
	line	2067
;bldc.c: 2067: CM1CON0 = 0x00;
	movlb 2	; select bank2
	clrf	(273)^0100h	;volatile
	goto	i1l28906
	line	1728
	
i1l29006:	
	movf	(_comm_state),w
	; Switch size 1, requested type "space"
; Number of cases is 6, Range of values is 1 to 6
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           19    10 (average)
; direct_byte           21     9 (fixed)
; jumptable            263     9 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	1^0	; case 1
	skipnz
	goto	i1l28862
	xorlw	2^1	; case 2
	skipnz
	goto	i1l28886
	xorlw	3^2	; case 3
	skipnz
	goto	i1l28910
	xorlw	4^3	; case 4
	skipnz
	goto	i1l28930
	xorlw	5^4	; case 5
	skipnz
	goto	i1l28954
	xorlw	6^5	; case 6
	skipnz
	goto	i1l28978
	goto	i1l28998
	opt asmopt_on

	line	2083
	
i1l29008:	
;bldc.c: 2081: {
;bldc.c: 2083: ui8_BlankingCount = ( unsigned char ) ( 0.002 * 20000UL );
	movlw	(028h)
	movlb 2	; select bank2
	movwf	(_ui8_BlankingCount)^0100h
	line	2096
;bldc.c: 2085: }
	
i1l29012:	
;bldc.c: 2093: }
;bldc.c: 2096: comm_time = 0;
	movlb 0	; select bank0
	clrf	(_comm_time)
	clrf	(_comm_time+1)
	line	2099
	
i1l3902:	
	return
	opt stack 0
GLOBAL	__end_ofi1_commutate
	__end_ofi1_commutate:
;; =============== function i1_commutate ends ============

	signat	i1_commutate,88
	global	i1_ADC_Wait
psect	text2071,local,class=CODE,delta=2
global __ptext2071
__ptext2071:

;; *************** function i1_ADC_Wait *****************
;; Defined at:
;;		line 130 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		None
;; Tracked objects:
;;		On entry : 0/1
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_interrrupt_bldc
;; This function uses a non-reentrant model
;;
psect	text2071
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
	line	130
	global	__size_ofi1_ADC_Wait
	__size_ofi1_ADC_Wait	equ	__end_ofi1_ADC_Wait-i1_ADC_Wait
	
i1_ADC_Wait:	
	opt	stack 8
; Regs used in i1_ADC_Wait: []
	line	133
	
i1l28748:	
;adc.c: 133: _nop();
	nop
	line	134
;adc.c: 134: _nop();
	nop
	line	135
;adc.c: 135: _nop();
	nop
	line	136
;adc.c: 136: _nop();
	nop
	line	137
;adc.c: 137: _nop();
	nop
	line	138
;adc.c: 138: _nop();
	nop
	line	139
;adc.c: 139: _nop();
	nop
	line	140
;adc.c: 140: _nop();
	nop
	line	142
;adc.c: 142: _nop();
	nop
	line	143
;adc.c: 143: _nop();
	nop
	line	144
;adc.c: 144: _nop();
	nop
	line	145
;adc.c: 145: _nop();
	nop
	line	146
;adc.c: 146: _nop();
	nop
	line	147
;adc.c: 147: _nop();
	nop
	line	148
;adc.c: 148: _nop();
	nop
	line	149
;adc.c: 149: _nop();
	nop
	line	151
;adc.c: 151: _nop();
	nop
	line	152
;adc.c: 152: _nop();
	nop
	line	153
;adc.c: 153: _nop();
	nop
	line	154
;adc.c: 154: _nop();
	nop
	line	155
;adc.c: 155: _nop();
	nop
	line	156
;adc.c: 156: _nop();
	nop
	line	157
;adc.c: 157: _nop();
	nop
	line	158
;adc.c: 158: _nop();
	nop
	line	160
	
i1l1854:	
	return
	opt stack 0
GLOBAL	__end_ofi1_ADC_Wait
	__end_ofi1_ADC_Wait:
;; =============== function i1_ADC_Wait ends ============

	signat	i1_ADC_Wait,88
	global	_ELINMIntHandler
psect	text2072,local,class=CODE,delta=2
global __ptext2072
__ptext2072:

;; *************** function _ELINMIntHandler *****************
;; Defined at:
;;		line 558 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr1l, fsr1h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 18/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          4       0       0       0       0       0       0       0
;;      Totals:         4       0       0       0       0       0       0       0
;;Total ram usage:        4 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		__ELINMIntResetProtocol
;; This function is called by:
;;		_interrupt_handler
;; This function uses a non-reentrant model
;;
psect	text2072
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
	line	558
	global	__size_of_ELINMIntHandler
	__size_of_ELINMIntHandler	equ	__end_of_ELINMIntHandler-_ELINMIntHandler
	
_ELINMIntHandler:	
	opt	stack 8
; Regs used in _ELINMIntHandler: [wreg+fsr1l-status,0+pclath+cstack]
	line	560
	
i1l25040:	
;lin.c: 560: if( SENDB == 0)
	movlb 3	; select bank3
	btfsc	(3315/8)^0180h,(3315)&7
	goto	u466_21
	goto	u466_20
u466_21:
	goto	i1l25116
u466_20:
	line	565
	
i1l25042:	
;lin.c: 563: {
;lin.c: 565: if( RCIF)
	movlb 0	; select bank0
	btfss	(141/8),(141)&7
	goto	u467_21
	goto	u467_20
u467_21:
	goto	i1l25116
u467_20:
	line	572
	
i1l25044:	
;lin.c: 568: {
;lin.c: 572: if( _ELINMIntStatus.ELINMINTSTS.IDLE)
	btfss	(__ELINMIntStatus),3
	goto	u468_21
	goto	u468_20
u468_21:
	goto	i1l13418
u468_20:
	line	577
	
i1l25046:	
;lin.c: 575: {
;lin.c: 577: if( ( RCSTA & 0x06 ) == 0)
	movlb 3	; select bank3
	movf	(413)^0180h,w
	andlw	06h
	btfss	status,2
	goto	u469_21
	goto	u469_20
u469_21:
	goto	i1l25116
u469_20:
	line	584
	
i1l25048:	
;lin.c: 580: {
;lin.c: 584: if( RCREG == 0x80)
	movf	(409)^0180h,w	;volatile
	xorlw	080h&0ffh
	skipz
	goto	u470_21
	goto	u470_20
u470_21:
	goto	i1l25116
u470_20:
	line	589
	
i1l25050:	
;lin.c: 587: {
;lin.c: 589: _ELINMIntSleepTimeout = ( ( 25000L * ( 100L * 1000000L / 19200L ) / 128L ) / 100L );
	movlw	0
	movlb 4	; select bank4
	movwf	(__ELINMIntSleepTimeout+3)^0200h
	movlw	0
	movwf	(__ELINMIntSleepTimeout+2)^0200h
	movlw	027h
	movwf	(__ELINMIntSleepTimeout+1)^0200h
	movlw	0BBh
	movwf	(__ELINMIntSleepTimeout)^0200h

	line	590
;lin.c: 590: _ELINMIntReadBack = RCREG;
	movlb 3	; select bank3
	movf	(409)^0180h,w	;volatile
	movlb 2	; select bank2
	movwf	(__ELINMIntReadBack)^0100h
	line	593
	
i1l25052:	
;lin.c: 593: if( _ELINMIntStatus1.ELINMINTSTS.WAKEUP_SENT)
	movlb 1	; select bank1
	btfss	(__ELINMIntStatus1)^080h,5
	goto	u471_21
	goto	u471_20
u471_21:
	goto	i1l13421
u471_20:
	line	598
	
i1l25054:	
;lin.c: 596: {
;lin.c: 598: _ELINMIntStatus1.ELINMINTSTS.WAKEUP_SENT = 0;
	bcf	(__ELINMIntStatus1)^080h,5
	line	601
;lin.c: 601: }
	goto	i1l25116
	line	604
	
i1l13421:	
	line	607
;lin.c: 604: else
;lin.c: 605: {
;lin.c: 607: _ELINMIntStatus1.ELINMINTSTS.WAKEUP = 1;
	bsf	(__ELINMIntStatus1)^080h,0
	goto	i1l25116
	line	613
	
i1l13420:	
	goto	i1l25116
	line	623
	
i1l13418:	
	line	626
;lin.c: 623: else
;lin.c: 624: {
;lin.c: 626: if( _ELINMIntStatus.ELINMINTSTS.TX)
	btfss	(__ELINMIntStatus),0
	goto	u472_21
	goto	u472_20
u472_21:
	goto	i1l13424
u472_20:
	line	633
	
i1l25056:	
;lin.c: 629: {
;lin.c: 633: if( ( RCSTA & 0x06 ) && _ELINMIntMessageBufferPointer)
	movlb 3	; select bank3
	movf	(413)^0180h,w
	andlw	06h
	btfsc	status,2
	goto	u473_21
	goto	u473_20
u473_21:
	goto	i1l25062
u473_20:
	
i1l25058:	
	movlb 1	; select bank1
	movf	(__ELINMIntMessageBufferPointer)^080h,w
	skipz
	goto	u474_20
	goto	i1l25062
u474_20:
	line	639
	
i1l25060:	
;lin.c: 636: {
;lin.c: 638: _ELINMIntResetProtocol(
;lin.c: 639: 0x08 + 0x04 + 0x70 );
	movlw	(07Ch)
	fcall	__ELINMIntResetProtocol
	line	642
;lin.c: 642: }
	goto	i1l25116
	line	651
	
i1l25062:	
;lin.c: 645: else
;lin.c: 646: {
;lin.c: 651: if( _ELINMIntMessageBufferPointer == 2)
	movlb 1	; select bank1
	movf	(__ELINMIntMessageBufferPointer)^080h,w
	xorlw	02h&0ffh
	skipz
	goto	u475_21
	goto	u475_20
u475_21:
	goto	i1l25066
u475_20:
	line	656
	
i1l25064:	
;lin.c: 654: {
;lin.c: 656: _ELINMIntStatus1.ELINMINTSTS.HEADER = 0;
	bcf	(__ELINMIntStatus1)^080h,1
	line	662
	
i1l25066:	
;lin.c: 659: }
;lin.c: 662: if( _ELINMIntSpace)
	movlb 2	; select bank2
	movf	(__ELINMIntSpace)^0100h,w
	skipz
	goto	u476_20
	goto	i1l25070
u476_20:
	line	667
	
i1l25068:	
;lin.c: 665: {
;lin.c: 667: _ELINMIntSpace--;
	decf	(__ELINMIntSpace)^0100h,f
	line	670
;lin.c: 670: }
	goto	i1l25116
	line	676
	
i1l25070:	
;lin.c: 673: else
;lin.c: 674: {
;lin.c: 676: if( _ELINMIntReadBack != RCREG)
	movf	(__ELINMIntReadBack)^0100h,w
	movlb 3	; select bank3
	xorwf	(409)^0180h,w	;volatile
	skipnz
	goto	u477_21
	goto	u477_20
u477_21:
	goto	i1l25074
u477_20:
	line	681
	
i1l25072:	
;lin.c: 679: {
;lin.c: 681: _ELINMIntResetProtocol(0x08 + 0x04 + 0x60 );
	movlw	(06Ch)
	fcall	__ELINMIntResetProtocol
	line	684
;lin.c: 684: }
	goto	i1l25116
	line	693
	
i1l25074:	
;lin.c: 687: else
;lin.c: 688: {
;lin.c: 693: if( _ELINMIntMessageSize.SIZE)
	movlb 2	; select bank2
	movf	(__ELINMIntMessageSize)^0100h,w
	skipz
	goto	u478_20
	goto	i1l13432
u478_20:
	line	698
	
i1l25076:	
;lin.c: 696: {
;lin.c: 698: _ELINMIntReadBack = RCREG;
	movlb 3	; select bank3
	movf	(409)^0180h,w	;volatile
	movlb 2	; select bank2
	movwf	(__ELINMIntReadBack)^0100h
	line	699
	
i1l25078:	
;lin.c: 699: _ELINMIntReadBack = _ELINMIntMessageBuffer[ _ELINMIntMessageBufferPointer ];
	movlb 1	; select bank1
	movf	(__ELINMIntMessageBufferPointer)^080h,w
	addlw	__ELINMIntMessageBuffer&0ffh
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movf	indf1,w
	movlb 2	; select bank2
	movwf	(__ELINMIntReadBack)^0100h
	line	700
	
i1l25080:	
;lin.c: 700: TXREG = _ELINMIntReadBack; _ELINMIntMessageSize.SIZE--;
	movf	(__ELINMIntReadBack)^0100h,w
	movlb 3	; select bank3
	movwf	(410)^0180h	;volatile
	
i1l25082:	
	movlb 2	; select bank2
	decf	(__ELINMIntMessageSize)^0100h,f
	line	701
	
i1l25084:	
;lin.c: 702: _ELINMIntSpace += (0+(((53300L/(((19200L*128L)+5000L)/10000L))+5L)/100L)-1)/2;
	movlb 1	; select bank1
	incf	(__ELINMIntMessageBufferPointer)^080h,f
	line	705
;lin.c: 705: }
	goto	i1l25116
	line	708
	
i1l13432:	
	line	714
;lin.c: 708: else
;lin.c: 709: {
;lin.c: 714: if( _ELINMIntStatus.ELINMINTSTS.RX)
	movlb 0	; select bank0
	btfss	(__ELINMIntStatus),1
	goto	u479_21
	goto	u479_20
u479_21:
	goto	i1l13434
u479_20:
	line	719
	
i1l25086:	
;lin.c: 717: {
;lin.c: 719: _ELINMIntStatus.ELINMINTSTS.TX = 0;
	bcf	(__ELINMIntStatus),0
	line	720
	
i1l25088:	
;lin.c: 720: _ELINMIntMessageBufferPointer = 0;
	movlb 1	; select bank1
	clrf	(__ELINMIntMessageBufferPointer)^080h
	line	723
;lin.c: 723: }
	goto	i1l25116
	line	726
	
i1l13434:	
	line	732
;lin.c: 726: else
;lin.c: 727: {
;lin.c: 732: _ELINMIntStatus1.ELINMINTSTS.FRAME = 0;
	movlb 1	; select bank1
	bcf	(__ELINMIntStatus1)^080h,2
	line	733
	
i1l25090:	
;lin.c: 733: _ELINMIntResetProtocol( 0x08 );
	movlw	(08h)
	fcall	__ELINMIntResetProtocol
	goto	i1l25116
	line	755
	
i1l13424:	
	line	758
;lin.c: 755: else
;lin.c: 756: {
;lin.c: 758: if( _ELINMIntStatus.ELINMINTSTS.RX)
	btfss	(__ELINMIntStatus),1
	goto	u480_21
	goto	u480_20
u480_21:
	goto	i1l13420
u480_20:
	line	765
	
i1l25092:	
;lin.c: 761: {
;lin.c: 765: if( RCSTA & 0x06)
	movlb 3	; select bank3
	movf	(413)^0180h,w
	andlw	06h
	btfsc	status,2
	goto	u481_21
	goto	u481_20
u481_21:
	goto	i1l25096
u481_20:
	line	770
	
i1l25094:	
;lin.c: 768: {
;lin.c: 770: _ELINMIntResetProtocol( 0x08 + 0x04 + 0x70 );
	movlw	(07Ch)
	fcall	__ELINMIntResetProtocol
	line	773
;lin.c: 773: }
	goto	i1l25116
	line	779
	
i1l25096:	
;lin.c: 776: else
;lin.c: 777: {
;lin.c: 779: if( _ELINMIntRXMessageSize.SIZE)
	movlb 2	; select bank2
	movf	(__ELINMIntRXMessageSize)^0100h,w
	skipz
	goto	u482_20
	goto	i1l25108
u482_20:
	line	784
	
i1l25098:	
;lin.c: 782: {
;lin.c: 784: _ELINMIntMessageBuffer [ _ELINMIntMessageBufferPointer ] = RCREG;
	movlb 1	; select bank1
	movf	(__ELINMIntMessageBufferPointer)^080h,w
	addlw	__ELINMIntMessageBuffer&0ffh
	movwf	fsr1l
	movlw 2	; select bank4/5
	movwf fsr1h	
	
	movlb 3	; select bank3
	movf	(409)^0180h,w	;volatile
	movwf	indf1
	line	785
	
i1l25100:	
;lin.c: 785: _ELINMIntRXMessageSize.SIZE--;
	movlb 2	; select bank2
	decf	(__ELINMIntRXMessageSize)^0100h,f
	line	786
	
i1l25102:	
;lin.c: 786: _ELINMIntRXCRC.CRC += RCREG;
	movlb 3	; select bank3
	movf	(409)^0180h,w	;volatile
	movlb 4	; select bank4
	addwf	(__ELINMIntRXCRC)^0200h,f
	skipnc
	incf	(__ELINMIntRXCRC+1)^0200h,f
	line	789
	
i1l25104:	
;lin.c: 789: if( _ELINMIntRXCRC.CRCbits.CRC8)
	btfss	0+(__ELINMIntRXCRC)^0200h+01h,0
	goto	u483_21
	goto	u483_20
u483_21:
	goto	i1l25084
u483_20:
	line	794
	
i1l25106:	
;lin.c: 792: {
;lin.c: 794: _ELINMIntRXCRC.CRCL++;
	incf	(__ELINMIntRXCRC)^0200h,f
	goto	i1l25084
	line	813
	
i1l25108:	
;lin.c: 813: _ELINMIntRXCRC.CRCL += RCREG + 1;
	movlb 3	; select bank3
	incf	(409)^0180h,w	;volatile
	movlb 4	; select bank4
	addwf	(__ELINMIntRXCRC)^0200h,f
	line	816
	
i1l25110:	
;lin.c: 816: if( _ELINMIntRXCRC.CRCL )
	movf	(__ELINMIntRXCRC)^0200h,w
	skipz
	goto	u484_20
	goto	i1l25114
u484_20:
	line	821
	
i1l25112:	
;lin.c: 819: {
;lin.c: 821: _ELINMIntResetProtocol( 0x08 + 0x04 + 0x50 );
	movlw	(05Ch)
	fcall	__ELINMIntResetProtocol
	line	824
;lin.c: 824: }
	goto	i1l25116
	line	830
	
i1l25114:	
;lin.c: 827: else
;lin.c: 828: {
;lin.c: 830: _ELINMIntResetProtocol( 0x08 );
	movlw	(08h)
	fcall	__ELINMIntResetProtocol
	line	859
	
i1l25116:	
;lin.c: 833: }
;lin.c: 836: }
;lin.c: 839: }
;lin.c: 843: }
;lin.c: 846: }
;lin.c: 849: }
;lin.c: 853: }
;lin.c: 856: }
;lin.c: 859: if( _ELINMIntStatus.ELINMINTSTS.IDLE == 0 )
	movlb 0	; select bank0
	btfsc	(__ELINMIntStatus),3
	goto	u485_21
	goto	u485_20
u485_21:
	goto	i1l13445
u485_20:
	line	866
	
i1l25118:	
;lin.c: 862: {
;lin.c: 866: if( _ELINMIntStatus1.ELINMINTSTS.FRAME == 1)
	movlb 1	; select bank1
	btfss	(__ELINMIntStatus1)^080h,2
	goto	u486_21
	goto	u486_20
u486_21:
	goto	i1l25142
u486_20:
	line	871
	
i1l25120:	
;lin.c: 869: {
;lin.c: 871: if( _ELINMIntStatus1.ELINMINTSTS.HEADER == 1)
	btfss	(__ELINMIntStatus1)^080h,1
	goto	u487_21
	goto	u487_20
u487_21:
	goto	i1l25130
u487_20:
	line	876
	
i1l25122:	
;lin.c: 874: {
;lin.c: 876: if( _ELINMIntTHeaderMin)
	movlb 3	; select bank3
	movf	(__ELINMIntTHeaderMin+1)^0180h,w
	iorwf	(__ELINMIntTHeaderMin)^0180h,w
	skipnz
	goto	u488_21
	goto	u488_20
u488_21:
	goto	i1l13448
u488_20:
	line	881
	
i1l25124:	
;lin.c: 879: {
;lin.c: 881: _ELINMIntTHeaderMin--;
	movlw	low(01h)
	subwf	(__ELINMIntTHeaderMin)^0180h,f
	movlw	high(01h)
	subwfb	(__ELINMIntTHeaderMin+1)^0180h,f
	line	884
	
i1l13448:	
	line	887
;lin.c: 884: }
;lin.c: 887: if( _ELINMIntTHeaderMax)
	movlb 4	; select bank4
	movf	(__ELINMIntTHeaderMax+1)^0200h,w
	iorwf	(__ELINMIntTHeaderMax)^0200h,w
	skipnz
	goto	u489_21
	goto	u489_20
u489_21:
	goto	i1l25128
u489_20:
	line	892
	
i1l25126:	
;lin.c: 890: {
;lin.c: 892: _ELINMIntTHeaderMax--;
	movlw	low(01h)
	subwf	(__ELINMIntTHeaderMax)^0200h,f
	movlw	high(01h)
	subwfb	(__ELINMIntTHeaderMax+1)^0200h,f
	line	895
;lin.c: 895: }
	goto	i1l25134
	line	902
	
i1l25128:	
;lin.c: 898: else
;lin.c: 899: {
;lin.c: 902: _ELINMIntResetProtocol( 0x08 + 0x04 + 0x20 );
	movlw	(02Ch)
	fcall	__ELINMIntResetProtocol
	goto	i1l25134
	line	917
	
i1l25130:	
;lin.c: 911: else
;lin.c: 912: {
;lin.c: 917: if( _ELINMIntTHeaderMin)
	movlb 3	; select bank3
	movf	(__ELINMIntTHeaderMin+1)^0180h,w
	iorwf	(__ELINMIntTHeaderMin)^0180h,w
	skipnz
	goto	u490_21
	goto	u490_20
u490_21:
	goto	i1l25134
u490_20:
	line	922
	
i1l25132:	
;lin.c: 920: {
;lin.c: 922: _ELINMIntResetProtocol( 0x08 + 0x04 + 0x10 );
	movlw	(01Ch)
	fcall	__ELINMIntResetProtocol
	line	931
	
i1l25134:	
;lin.c: 925: }
;lin.c: 928: }
;lin.c: 931: if( _ELINMIntTFrameMin)
	movlb 3	; select bank3
	movf	(__ELINMIntTFrameMin+1)^0180h,w
	iorwf	(__ELINMIntTFrameMin)^0180h,w
	skipnz
	goto	u491_21
	goto	u491_20
u491_21:
	goto	i1l13453
u491_20:
	line	936
	
i1l25136:	
;lin.c: 934: {
;lin.c: 936: _ELINMIntTFrameMin--;
	movlw	low(01h)
	subwf	(__ELINMIntTFrameMin)^0180h,f
	movlw	high(01h)
	subwfb	(__ELINMIntTFrameMin+1)^0180h,f
	line	939
	
i1l13453:	
	line	942
;lin.c: 939: }
;lin.c: 942: if( _ELINMIntTFrameMax)
	movlb 4	; select bank4
	movf	(__ELINMIntTFrameMax+1)^0200h,w
	iorwf	(__ELINMIntTFrameMax)^0200h,w
	skipnz
	goto	u492_21
	goto	u492_20
u492_21:
	goto	i1l25140
u492_20:
	line	947
	
i1l25138:	
;lin.c: 945: {
;lin.c: 947: _ELINMIntTFrameMax--;
	movlw	low(01h)
	subwf	(__ELINMIntTFrameMax)^0200h,f
	movlw	high(01h)
	subwfb	(__ELINMIntTFrameMax+1)^0200h,f
	line	950
;lin.c: 950: }
	goto	i1l13462
	line	957
	
i1l25140:	
;lin.c: 953: else
;lin.c: 954: {
;lin.c: 957: _ELINMIntResetProtocol( 0x08 + 0x04 + 0x40 );
	movlw	(04Ch)
	fcall	__ELINMIntResetProtocol
	goto	i1l13462
	line	972
	
i1l25142:	
;lin.c: 966: else
;lin.c: 967: {
;lin.c: 972: if( _ELINMIntTFrameMin)
	movlb 3	; select bank3
	movf	(__ELINMIntTFrameMin+1)^0180h,w
	iorwf	(__ELINMIntTFrameMin)^0180h,w
	skipnz
	goto	u493_21
	goto	u493_20
u493_21:
	goto	i1l13462
u493_20:
	line	977
	
i1l25144:	
;lin.c: 975: {
;lin.c: 977: _ELINMIntResetProtocol( 0x08 + 0x04 + 0x30 );
	movlw	(03Ch)
	fcall	__ELINMIntResetProtocol
	goto	i1l13462
	line	990
	
i1l13445:	
	line	993
;lin.c: 990: else
;lin.c: 991: {
;lin.c: 993: if( _ELINMIntStatus1.ELINMINTSTS.WAKEUP_SENT == 0)
	movlb 1	; select bank1
	btfsc	(__ELINMIntStatus1)^080h,5
	goto	u494_21
	goto	u494_20
u494_21:
	goto	i1l13462
u494_20:
	line	998
	
i1l25146:	
;lin.c: 996: {
;lin.c: 998: if( _ELINMIntSleepTimeout)
	movlb 4	; select bank4
	movf	(__ELINMIntSleepTimeout+3)^0200h,w
	iorwf	(__ELINMIntSleepTimeout+2)^0200h,w
	iorwf	(__ELINMIntSleepTimeout+1)^0200h,w
	iorwf	(__ELINMIntSleepTimeout)^0200h,w
	skipnz
	goto	u495_21
	goto	u495_20
u495_21:
	goto	i1l13460
u495_20:
	line	1003
	
i1l25148:	
;lin.c: 1001: {
;lin.c: 1003: _ELINMIntSleepTimeout--;
	movlw	01h
	movwf	((??_ELINMIntHandler+0)+0)
	movlw	0
	movwf	((??_ELINMIntHandler+0)+0+1)
	movlw	0
	movwf	((??_ELINMIntHandler+0)+0+2)
	movlw	0
	movwf	((??_ELINMIntHandler+0)+0+3)
	movf	0+(??_ELINMIntHandler+0)+0,w
	subwf	(__ELINMIntSleepTimeout)^0200h,f
	movf	1+(??_ELINMIntHandler+0)+0,w
	subwfb	(__ELINMIntSleepTimeout+1)^0200h,f
	movf	2+(??_ELINMIntHandler+0)+0,w
	subwfb	(__ELINMIntSleepTimeout+2)^0200h,f
	movf	3+(??_ELINMIntHandler+0)+0,w
	subwfb	(__ELINMIntSleepTimeout+3)^0200h,f
	line	1006
;lin.c: 1006: }
	goto	i1l13462
	line	1009
	
i1l13460:	
	line	1012
;lin.c: 1009: else
;lin.c: 1010: {
;lin.c: 1012: _ELINMIntStatus1.ELINMINTSTS.SLEEP_TIMEOUT = 1;
	movlb 1	; select bank1
	bsf	(__ELINMIntStatus1)^080h,6
	line	1024
	
i1l13462:	
	return
	opt stack 0
GLOBAL	__end_of_ELINMIntHandler
	__end_of_ELINMIntHandler:
;; =============== function _ELINMIntHandler ends ============

	signat	_ELINMIntHandler,88
	global	_BLDCWait
psect	text2073,local,class=CODE,delta=2
global __ptext2073
__ptext2073:

;; *************** function _BLDCWait *****************
;; Defined at:
;;		line 157 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\bldc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		None
;; Tracked objects:
;;		On entry : 1E/1
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_interrrupt_bldc
;; This function uses a non-reentrant model
;;
psect	text2073
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\bldc.c"
	line	157
	global	__size_of_BLDCWait
	__size_of_BLDCWait	equ	__end_of_BLDCWait-_BLDCWait
	
_BLDCWait:	
	opt	stack 8
; Regs used in _BLDCWait: []
	line	160
	
i1l28344:	
;bldc.c: 160: _nop();
	nop
	line	161
;bldc.c: 161: _nop();
	nop
	line	162
;bldc.c: 162: _nop();
	nop
	line	163
;bldc.c: 163: _nop();
	nop
	line	164
;bldc.c: 164: _nop();
	nop
	line	165
;bldc.c: 165: _nop();
	nop
	line	166
;bldc.c: 166: _nop();
	nop
	line	167
;bldc.c: 167: _nop();
	nop
	line	169
;bldc.c: 169: _nop();
	nop
	line	170
;bldc.c: 170: _nop();
	nop
	line	171
;bldc.c: 171: _nop();
	nop
	line	172
;bldc.c: 172: _nop();
	nop
	line	173
;bldc.c: 173: _nop();
	nop
	line	174
;bldc.c: 174: _nop();
	nop
	line	175
;bldc.c: 175: _nop();
	nop
	line	176
;bldc.c: 176: _nop();
	nop
	line	178
;bldc.c: 178: _nop();
	nop
	line	179
;bldc.c: 179: _nop();
	nop
	line	180
;bldc.c: 180: _nop();
	nop
	line	181
;bldc.c: 181: _nop();
	nop
	line	182
;bldc.c: 182: _nop();
	nop
	line	183
;bldc.c: 183: _nop();
	nop
	line	184
;bldc.c: 184: _nop();
	nop
	line	185
;bldc.c: 185: _nop();
	nop
	line	187
;bldc.c: 187: _nop();
	nop
	line	188
;bldc.c: 188: _nop();
	nop
	line	189
;bldc.c: 189: _nop();
	nop
	line	190
;bldc.c: 190: _nop();
	nop
	line	191
;bldc.c: 191: _nop();
	nop
	line	192
;bldc.c: 192: _nop();
	nop
	line	193
;bldc.c: 193: _nop();
	nop
	line	194
;bldc.c: 194: _nop();
	nop
	line	198
	
i1l3785:	
	return
	opt stack 0
GLOBAL	__end_of_BLDCWait
	__end_of_BLDCWait:
;; =============== function _BLDCWait ends ============

	signat	_BLDCWait,88
	global	i1___lwdiv
psect	text2074,local,class=CODE,delta=2
global __ptext2074
__ptext2074:

;; *************** function i1___lwdiv *****************
;; Defined at:
;;		line 5 in file "C:\Program Files\HI-TECH Software\PICC\9.81\sources\lwdiv.c"
;; Parameters:    Size  Location     Type
;;  __lwdiv         2    0[COMMON] unsigned int 
;;  __lwdiv         2    2[COMMON] unsigned int 
;; Auto vars:     Size  Location     Type
;;  __lwdiv         2    5[COMMON] unsigned int 
;;  __lwdiv         1    4[COMMON] unsigned char 
;; Return value:  Size  Location     Type
;;                  2    0[COMMON] unsigned int 
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         4       0       0       0       0       0       0       0
;;      Locals:         3       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         7       0       0       0       0       0       0       0
;;Total ram usage:        7 bytes
;; Hardware stack levels used:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_interrrupt_bldc
;; This function uses a non-reentrant model
;;
psect	text2074
	file	"C:\Program Files\HI-TECH Software\PICC\9.81\sources\lwdiv.c"
	line	5
	global	__size_ofi1___lwdiv
	__size_ofi1___lwdiv	equ	__end_ofi1___lwdiv-i1___lwdiv
	
i1___lwdiv:	
	opt	stack 8
; Regs used in i1___lwdiv: [wreg+status,2+status,0]
	line	9
	
i1l29042:	
	clrf	(i1___lwdiv@quotient)
	clrf	(i1___lwdiv@quotient+1)
	line	10
	
i1l29044:	
	movf	(i1___lwdiv@divisor+1),w
	iorwf	(i1___lwdiv@divisor),w
	skipnz
	goto	u900_21
	goto	u900_20
u900_21:
	goto	i1l29064
u900_20:
	line	11
	
i1l29046:	
	clrf	(i1___lwdiv@counter)
	incf	(i1___lwdiv@counter),f
	line	12
	goto	i1l29050
	line	13
	
i1l29048:	
	lslf	(i1___lwdiv@divisor),f
	rlf	(i1___lwdiv@divisor+1),f
	line	14
	incf	(i1___lwdiv@counter),f
	line	12
	
i1l29050:	
	btfss	(i1___lwdiv@divisor+1),(15)&7
	goto	u901_21
	goto	u901_20
u901_21:
	goto	i1l29048
u901_20:
	line	17
	
i1l29052:	
	lslf	(i1___lwdiv@quotient),f
	rlf	(i1___lwdiv@quotient+1),f
	line	18
	
i1l29054:	
	movf	(i1___lwdiv@divisor+1),w
	subwf	(i1___lwdiv@dividend+1),w
	skipz
	goto	u902_25
	movf	(i1___lwdiv@divisor),w
	subwf	(i1___lwdiv@dividend),w
u902_25:
	skipc
	goto	u902_21
	goto	u902_20
u902_21:
	goto	i1l29060
u902_20:
	line	19
	
i1l29056:	
	movf	(i1___lwdiv@divisor),w
	subwf	(i1___lwdiv@dividend),f
	movf	(i1___lwdiv@divisor+1),w
	subwfb	(i1___lwdiv@dividend+1),f
	line	20
	
i1l29058:	
	bsf	(i1___lwdiv@quotient)+(0/8),(0)&7
	line	22
	
i1l29060:	
	lsrf	(i1___lwdiv@divisor+1),f
	rrf	(i1___lwdiv@divisor),f
	line	23
	
i1l29062:	
	decfsz	(i1___lwdiv@counter),f
	goto	u903_21
	goto	u903_20
u903_21:
	goto	i1l29052
u903_20:
	line	25
	
i1l29064:	
	movf	(i1___lwdiv@quotient+1),w
	movwf	(?i1___lwdiv+1)
	movf	(i1___lwdiv@quotient),w
	movwf	(?i1___lwdiv)
	line	26
	
i1l19689:	
	return
	opt stack 0
GLOBAL	__end_ofi1___lwdiv
	__end_ofi1___lwdiv:
;; =============== function i1___lwdiv ends ============

	signat	i1___lwdiv,90
	global	i1___wmul
psect	text2075,local,class=CODE,delta=2
global __ptext2075
__ptext2075:

;; *************** function i1___wmul *****************
;; Defined at:
;;		line 3 in file "C:\Program Files\HI-TECH Software\PICC\9.81\sources\wmul.c"
;; Parameters:    Size  Location     Type
;;  __wmul          2    0[COMMON] unsigned int 
;;  __wmul          2    2[COMMON] unsigned int 
;; Auto vars:     Size  Location     Type
;;  __wmul          2    4[COMMON] unsigned int 
;; Return value:  Size  Location     Type
;;                  2    0[COMMON] unsigned int 
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1E/2
;;		On exit  : 1E/2
;;		Unchanged: FFFE1/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         4       0       0       0       0       0       0       0
;;      Locals:         2       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         6       0       0       0       0       0       0       0
;;Total ram usage:        6 bytes
;; Hardware stack levels used:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_interrrupt_bldc
;;		i1_commutate
;; This function uses a non-reentrant model
;;
psect	text2075
	file	"C:\Program Files\HI-TECH Software\PICC\9.81\sources\wmul.c"
	line	3
	global	__size_ofi1___wmul
	__size_ofi1___wmul	equ	__end_ofi1___wmul-i1___wmul
	
i1___wmul:	
	opt	stack 8
; Regs used in i1___wmul: [wreg+status,2+status,0]
	line	4
	
i1l29026:	
	clrf	(i1___wmul@product)
	clrf	(i1___wmul@product+1)
	line	7
	
i1l29028:	
	btfss	(i1___wmul@multiplier),(0)&7
	goto	u898_21
	goto	u898_20
u898_21:
	goto	i1l29032
u898_20:
	line	8
	
i1l29030:	
	movf	(i1___wmul@multiplicand),w
	addwf	(i1___wmul@product),f
	movf	(i1___wmul@multiplicand+1),w
	addwfc	(i1___wmul@product+1),f
	line	9
	
i1l29032:	
	lslf	(i1___wmul@multiplicand),f
	rlf	(i1___wmul@multiplicand+1),f
	line	10
	
i1l29034:	
	lsrf	(i1___wmul@multiplier+1),f
	rrf	(i1___wmul@multiplier),f
	line	11
	
i1l29036:	
	movf	((i1___wmul@multiplier+1)),w
	iorwf	((i1___wmul@multiplier)),w
	skipz
	goto	u899_21
	goto	u899_20
u899_21:
	goto	i1l29028
u899_20:
	line	12
	
i1l29038:	
	movf	(i1___wmul@product+1),w
	movwf	(?i1___wmul+1)
	movf	(i1___wmul@product),w
	movwf	(?i1___wmul)
	line	13
	
i1l19679:	
	return
	opt stack 0
GLOBAL	__end_ofi1___wmul
	__end_ofi1___wmul:
;; =============== function i1___wmul ends ============

	signat	i1___wmul,90
	global	i1_ADC_Read
psect	text2076,local,class=CODE,delta=2
global __ptext2076
__ptext2076:

;; *************** function i1_ADC_Read *****************
;; Defined at:
;;		line 177 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  ADC_Read        1    2[COMMON] unsigned char 
;; Return value:  Size  Location     Type
;;                  2    0[COMMON] unsigned int 
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1C/1
;;		On exit  : 1F/0
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         2       0       0       0       0       0       0       0
;;      Locals:         1       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         3       0       0       0       0       0       0       0
;;Total ram usage:        3 bytes
;; Hardware stack levels used:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_interrrupt_bldc
;; This function uses a non-reentrant model
;;
psect	text2076
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\adc.c"
	line	177
	global	__size_ofi1_ADC_Read
	__size_ofi1_ADC_Read	equ	__end_ofi1_ADC_Read-i1_ADC_Read
	
i1_ADC_Read:	
	opt	stack 8
; Regs used in i1_ADC_Read: [wreg+status,2+status,0]
	line	180
	
i1l28750:	
;adc.c: 179: unsigned char i;
;adc.c: 180: i = 0;
	clrf	(i1ADC_Read@i)
	line	182
;adc.c: 182: while( GO_nDONE )
	goto	i1l1857
	line	186
	
i1l28752:	
;adc.c: 184: {
;adc.c: 186: i++;
	incf	(i1ADC_Read@i),f
	line	189
	
i1l28754:	
;adc.c: 189: if( i > 64 )
	movlw	(041h)
	subwf	(i1ADC_Read@i),w
	skipc
	goto	u877_21
	goto	u877_20
u877_21:
	goto	i1l1857
u877_20:
	goto	i1l28758
	line	198
	
i1l1857:	
	line	182
	movlb 1	; select bank1
	btfsc	(1257/8)^080h,(1257)&7
	goto	u878_21
	goto	u878_20
u878_21:
	goto	i1l28752
u878_20:
	line	201
	
i1l28758:	
;adc.c: 195: }
;adc.c: 198: }
;adc.c: 201: analog_value.b.hi = ADRESH;
	movf	(156)^080h,w	;volatile
	movlb 0	; select bank0
	movwf	0+(_analog_value)+01h	;volatile
	line	202
;adc.c: 202: analog_value.b.lo = ADRESL;
	movlb 1	; select bank1
	movf	(155)^080h,w	;volatile
	movlb 0	; select bank0
	movwf	(_analog_value)	;volatile
	line	204
;adc.c: 204: return analog_value.w;
	movf	(_analog_value+1),w	;volatile
	movwf	(?i1_ADC_Read+1)
	movf	(_analog_value),w	;volatile
	movwf	(?i1_ADC_Read)
	line	206
	
i1l1861:	
	return
	opt stack 0
GLOBAL	__end_ofi1_ADC_Read
	__end_ofi1_ADC_Read:
;; =============== function i1_ADC_Read ends ============

	signat	i1_ADC_Read,90
	global	__ELINMIntResetProtocol
psect	text2077,local,class=CODE,delta=2
global __ptext2077
__ptext2077:

;; *************** function __ELINMIntResetProtocol *****************
;; Defined at:
;;		line 527 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
;; Parameters:    Size  Location     Type
;;  code            1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  code            1    0[COMMON] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2
;; Tracked objects:
;;		On entry : 18/1
;;		On exit  : 1F/4
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         1       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         1       0       0       0       0       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_ELINMIntHandler
;; This function uses a non-reentrant model
;;
psect	text2077
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\lin.c"
	line	527
	global	__size_of__ELINMIntResetProtocol
	__size_of__ELINMIntResetProtocol	equ	__end_of__ELINMIntResetProtocol-__ELINMIntResetProtocol
	
__ELINMIntResetProtocol:	
	opt	stack 8
; Regs used in __ELINMIntResetProtocol: [wreg+status,2]
;__ELINMIntResetProtocol@code stored from wreg
	line	529
	movwf	(__ELINMIntResetProtocol@code)
	
i1l25188:	
;lin.c: 529: _ELINMIntReadBack = RCREG;
	movlb 3	; select bank3
	movf	(409)^0180h,w	;volatile
	movlb 2	; select bank2
	movwf	(__ELINMIntReadBack)^0100h
	line	530
	
i1l25190:	
;lin.c: 530: _ELINMIntStatus1.ELINMIntStatusByte = 0;
	movlb 1	; select bank1
	clrf	(__ELINMIntStatus1)^080h
	line	531
	
i1l25192:	
;lin.c: 531: _ELINMIntRXCRC.CRC = 0;
	movlb 4	; select bank4
	clrf	(__ELINMIntRXCRC)^0200h
	clrf	(__ELINMIntRXCRC+1)^0200h
	line	532
;lin.c: 532: _ELINMIntStatus.ELINMIntStatusByte = code;
	movf	(__ELINMIntResetProtocol@code),w
	movlb 0	; select bank0
	movwf	(__ELINMIntStatus)
	line	533
;lin.c: 533: _ELINMIntSleepTimeout = ( ( 25000L * ( 100L * 1000000L / 19200L ) / 128L ) / 100L );
	movlw	0
	movlb 4	; select bank4
	movwf	(__ELINMIntSleepTimeout+3)^0200h
	movlw	0
	movwf	(__ELINMIntSleepTimeout+2)^0200h
	movlw	027h
	movwf	(__ELINMIntSleepTimeout+1)^0200h
	movlw	0BBh
	movwf	(__ELINMIntSleepTimeout)^0200h

	line	536
	
i1l13413:	
	return
	opt stack 0
GLOBAL	__end_of__ELINMIntResetProtocol
	__end_of__ELINMIntResetProtocol:
;; =============== function __ELINMIntResetProtocol ends ============

	signat	__ELINMIntResetProtocol,4216
	global	_interrupt_PWMCapture
psect	text2078,local,class=CODE,delta=2
global __ptext2078
__ptext2078:

;; *************** function _interrupt_PWMCapture *****************
;; Defined at:
;;		line 440 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\pwm.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1B/2
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_interrupt_handler
;; This function uses a non-reentrant model
;;
psect	text2078
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\pwm.c"
	line	440
	global	__size_of_interrupt_PWMCapture
	__size_of_interrupt_PWMCapture	equ	__end_of_interrupt_PWMCapture-_interrupt_PWMCapture
	
_interrupt_PWMCapture:	
	opt	stack 9
; Regs used in _interrupt_PWMCapture: [wreg-fsr0h+status,2+status,0]
	line	450
	
i1l25150:	
;pwm.c: 450: switch( ui8_Pulse_State )
	goto	i1l25164
	line	454
;pwm.c: 452: {
;pwm.c: 454: case 0:
	
i1l17347:	
	line	460
;pwm.c: 456: {
;pwm.c: 460: ui8_Pulse_State = 1;
	clrf	(_ui8_Pulse_State)^0100h
	incf	(_ui8_Pulse_State)^0100h,f
	line	461
	
i1l25152:	
;pwm.c: 461: ui16_Capt_Val0.b.lo = CCPR5L;
	movlb 6	; select bank6
	movf	(796)^0300h,w	;volatile
	movlb 2	; select bank2
	movwf	(_ui16_Capt_Val0)^0100h	;volatile
	line	462
;pwm.c: 462: ui16_Capt_Val0.b.hi = CCPR5H;
	movlb 6	; select bank6
	movf	(797)^0300h,w	;volatile
	movlb 2	; select bank2
	movwf	0+(_ui16_Capt_Val0)^0100h+01h	;volatile
	line	463
;pwm.c: 463: CCP5CON = 0x04;
	movlw	(04h)
	movlb 6	; select bank6
	movwf	(798)^0300h	;volatile
	line	464
;pwm.c: 464: break;
	goto	i1l17354
	line	476
	
i1l25154:	
;pwm.c: 472: {
;pwm.c: 476: ui8_Pulse_State = 2;
	movlw	(02h)
	movwf	(_ui8_Pulse_State)^0100h
	line	477
;pwm.c: 477: ui16_Capt_Val1.b.lo = CCPR5L;
	movlb 6	; select bank6
	movf	(796)^0300h,w	;volatile
	movlb 2	; select bank2
	movwf	(_ui16_Capt_Val1)^0100h	;volatile
	line	478
;pwm.c: 478: ui16_Capt_Val1.b.hi = CCPR5H;
	movlb 6	; select bank6
	movf	(797)^0300h,w	;volatile
	movlb 2	; select bank2
	movwf	0+(_ui16_Capt_Val1)^0100h+01h	;volatile
	line	479
;pwm.c: 479: CCP5CON = 0x05;
	movlw	(05h)
	movlb 6	; select bank6
	movwf	(798)^0300h	;volatile
	line	480
;pwm.c: 480: break;
	goto	i1l17354
	line	486
;pwm.c: 483: }
;pwm.c: 486: case 2:
	
i1l17350:	
	line	492
;pwm.c: 488: {
;pwm.c: 492: ui8_Pulse_State = 1;
	clrf	(_ui8_Pulse_State)^0100h
	incf	(_ui8_Pulse_State)^0100h,f
	line	493
	
i1l25156:	
;pwm.c: 493: ui16_Capt_Val2.b.lo = CCPR5L;
	movlb 6	; select bank6
	movf	(796)^0300h,w	;volatile
	movlb 2	; select bank2
	movwf	(_ui16_Capt_Val2)^0100h	;volatile
	line	494
;pwm.c: 494: ui16_Capt_Val2.b.hi = CCPR5H;
	movlb 6	; select bank6
	movf	(797)^0300h,w	;volatile
	movlb 2	; select bank2
	movwf	0+(_ui16_Capt_Val2)^0100h+01h	;volatile
	line	495
;pwm.c: 495: ui16_PWM_Freq_In = ui16_Capt_Val2.w - ui16_Capt_Val0.w;
	movf	(_ui16_Capt_Val2+1)^0100h,w	;volatile
	movlb 3	; select bank3
	movwf	(_ui16_PWM_Freq_In+1)^0180h
	movlb 2	; select bank2
	movf	(_ui16_Capt_Val2)^0100h,w	;volatile
	movlb 3	; select bank3
	movwf	(_ui16_PWM_Freq_In)^0180h
	movlb 2	; select bank2
	movf	(_ui16_Capt_Val0)^0100h,w	;volatile
	movlb 3	; select bank3
	subwf	(_ui16_PWM_Freq_In)^0180h,f
	movlb 2	; select bank2
	movf	(_ui16_Capt_Val0+1)^0100h,w	;volatile
	movlb 3	; select bank3
	subwfb	(_ui16_PWM_Freq_In+1)^0180h,f
	line	496
;pwm.c: 496: ui16_Duty_Cycle_In = ui16_Capt_Val1.w - ui16_Capt_Val0.w;
	movlb 2	; select bank2
	movf	(_ui16_Capt_Val1+1)^0100h,w	;volatile
	movlb 3	; select bank3
	movwf	(_ui16_Duty_Cycle_In+1)^0180h
	movlb 2	; select bank2
	movf	(_ui16_Capt_Val1)^0100h,w	;volatile
	movlb 3	; select bank3
	movwf	(_ui16_Duty_Cycle_In)^0180h
	movlb 2	; select bank2
	movf	(_ui16_Capt_Val0)^0100h,w	;volatile
	movlb 3	; select bank3
	subwf	(_ui16_Duty_Cycle_In)^0180h,f
	movlb 2	; select bank2
	movf	(_ui16_Capt_Val0+1)^0100h,w	;volatile
	movlb 3	; select bank3
	subwfb	(_ui16_Duty_Cycle_In+1)^0180h,f
	line	497
;pwm.c: 497: ui16_Capt_Val0.w = ui16_Capt_Val2.w;
	movlb 2	; select bank2
	movf	(_ui16_Capt_Val2+1)^0100h,w	;volatile
	movwf	(_ui16_Capt_Val0+1)^0100h	;volatile
	movf	(_ui16_Capt_Val2)^0100h,w	;volatile
	movwf	(_ui16_Capt_Val0)^0100h	;volatile
	line	498
;pwm.c: 501: if( ui16_Duty_Cycle_In )
	movlw	(04h)
	movlb 6	; select bank6
	movwf	(798)^0300h	;volatile
	line	501
	goto	i1l17354
	line	530
	
i1l25160:	
;pwm.c: 526: {
;pwm.c: 530: ui8_Pulse_State = 0;
	clrf	(_ui8_Pulse_State)^0100h
	line	531
;pwm.c: 531: break;
	goto	i1l17354
	line	450
	
i1l25164:	
	movlb 2	; select bank2
	movf	(_ui8_Pulse_State)^0100h,w
	; Switch size 1, requested type "space"
; Number of cases is 3, Range of values is 0 to 2
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           10     6 (average)
; direct_byte           12     6 (fixed)
; jumptable            260     6 (fixed)
; rangetable             7     4 (fixed)
; spacedrange           11     6 (fixed)
; locatedrange           3     3 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	0^0	; case 0
	skipnz
	goto	i1l17347
	xorlw	1^0	; case 1
	skipnz
	goto	i1l25154
	xorlw	2^1	; case 2
	skipnz
	goto	i1l17350
	goto	i1l25160
	opt asmopt_on

	line	539
	
i1l17354:	
	return
	opt stack 0
GLOBAL	__end_of_interrupt_PWMCapture
	__end_of_interrupt_PWMCapture:
;; =============== function _interrupt_PWMCapture ends ============

	signat	_interrupt_PWMCapture,88
	global	_PWM_CTRL
psect	text2079,local,class=CODE,delta=2
global __ptext2079
__ptext2079:

;; *************** function _PWM_CTRL *****************
;; Defined at:
;;		line 200 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\pwm.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 1F/0
;;		On exit  : 1F/2
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_interrupt_handler
;; This function uses a non-reentrant model
;;
psect	text2079
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\pwm.c"
	line	200
	global	__size_of_PWM_CTRL
	__size_of_PWM_CTRL	equ	__end_of_PWM_CTRL-_PWM_CTRL
	
_PWM_CTRL:	
	opt	stack 9
; Regs used in _PWM_CTRL: [wreg+status,2+status,0]
	line	288
	
i1l25028:	
;pwm.c: 288: ui8_PWM_FreqCnt++;
	movlb 2	; select bank2
	incf	(_ui8_PWM_FreqCnt)^0100h,f
	line	290
	
i1l25030:	
;pwm.c: 290: if( ui8_PWM_FreqCnt >= 100 )
	movlw	(064h)
	subwf	(_ui8_PWM_FreqCnt)^0100h,w
	skipc
	goto	u464_21
	goto	u464_20
u464_21:
	goto	i1l25036
u464_20:
	line	294
	
i1l25032:	
;pwm.c: 292: {
;pwm.c: 294: ui8_PWM_FreqCnt = 0;
	clrf	(_ui8_PWM_FreqCnt)^0100h
	line	295
	
i1l25034:	
;pwm.c: 295: LATC0 = 0;
	bcf	(2160/8)^0100h,(2160)&7
	line	298
;pwm.c: 298: }
	goto	i1l17334
	line	304
	
i1l25036:	
;pwm.c: 301: else
;pwm.c: 302: {
;pwm.c: 304: if( ui8_PWM_FreqCnt >= ui8_PWMoutvalue )
	movf	(_ui8_PWMoutvalue)^0100h,w
	subwf	(_ui8_PWM_FreqCnt)^0100h,w
	skipc
	goto	u465_21
	goto	u465_20
u465_21:
	goto	i1l17334
u465_20:
	line	308
	
i1l25038:	
;pwm.c: 306: {
;pwm.c: 308: LATC0 = 1;
	bsf	(2160/8)^0100h,(2160)&7
	line	326
	
i1l17334:	
	return
	opt stack 0
GLOBAL	__end_of_PWM_CTRL
	__end_of_PWM_CTRL:
;; =============== function _PWM_CTRL ends ============

	signat	_PWM_CTRL,88
	global	_Task1ms
psect	text2080,local,class=CODE,delta=2
global __ptext2080
__ptext2080:

;; *************** function _Task1ms *****************
;; Defined at:
;;		line 54 in file "D:\Projekte\BVH2\BLDC\V0.9.3\src\interrupt.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		status,2, status,0
;; Tracked objects:
;;		On entry : 1F/2
;;		On exit  : 1F/2
;;		Unchanged: FFFE0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK2   BANK3   BANK4   BANK5   BANK6
;;      Params:         0       0       0       0       0       0       0       0
;;      Locals:         0       0       0       0       0       0       0       0
;;      Temps:          0       0       0       0       0       0       0       0
;;      Totals:         0       0       0       0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_interrupt_handler
;; This function uses a non-reentrant model
;;
psect	text2080
	file	"D:\Projekte\BVH2\BLDC\V0.9.3\src\interrupt.c"
	line	54
	global	__size_of_Task1ms
	__size_of_Task1ms	equ	__end_of_Task1ms-_Task1ms
	
_Task1ms:	
	opt	stack 9
; Regs used in _Task1ms: [status]
	line	56
	
i1l24994:	
;interrupt.c: 56: ui8_Task_Cont1ms++;
	incf	(_ui8_Task_Cont1ms)^0100h,f
	line	58
	
i1l11455:	
	return
	opt stack 0
GLOBAL	__end_of_Task1ms
	__end_of_Task1ms:
;; =============== function _Task1ms ends ============

	signat	_Task1ms,88
psect	text2081,local,class=CODE,delta=2
global __ptext2081
__ptext2081:
	global	btemp
	btemp set 07Eh

	DABS	1,126,2	;btemp
	global	wtemp0
	wtemp0 set btemp
	end
