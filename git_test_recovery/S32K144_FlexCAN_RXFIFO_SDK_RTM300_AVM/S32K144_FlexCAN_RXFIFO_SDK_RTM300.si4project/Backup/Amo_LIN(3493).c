#define Amo_LIN_C_

#include "Cpu.h"
#include "Amo_main.h"
#include "flexcan_driver.h"
#include "Amo_STATE_Event.h"
#include "Amo_Timer.h"
#include "Amo_Calculate.h"
#include "Amo_CAN_Parsing.h"
#include "Amo_Mode_Setting.h"
#include "Amo_Cycle.h"

uint8_t FR_LH_Target_Pt_act_flag = 0;
uint8_t FR_RH_Target_Pt_act_flag = 0;

uint8_t lin_WakeupFlag = false;

extern uint8_t sleepFlag;


extern flexcan_msgbuff_t recvBuff1, recvBuff2;

extern uint16_t Hu_Fr_Dr_Vent_Side_Bdry_Up_X;
extern uint16_t Hu_Fr_Dr_Vent_Side_Bdry_Lo_X;
extern uint16_t Hu_FrDrEVntSidePt_X;
extern uint16_t Hu_FrDrEVntCtrPt_X;

extern Lvnt_1_data Can_Tx_Evnt_1;
extern Lvnt_2_data Can_Tx_Evnt_2;
extern Lvnt_3_data Can_Tx_Evnt_3;
extern Lvnt_4_data Can_Tx_Evnt_4;
extern Lvnt_5_data Can_Tx_Evnt_5;
extern Lvnt_6_data Can_Tx_Evnt_6;
extern Lvnt_7_data Can_Tx_Evnt_7;
extern Lvnt_8_data Can_Tx_Evnt_8;
extern Lvnt_9_data Can_Tx_Evnt_9;
//extern uint16_t selectEVnt;

extern lin_LH_EVNT_MASTER_CMD_11 LIN_LH_EVNT_MASTER_CMD;
extern lin_RH_EVNT_MASTER_CMD_21	LIN_RH_EVNT_MASTER_CMD;
extern lin_RC_EVNT_MASTER_CMD_31	LIN_RC_EVNT_MASTER_CMD;

extern int16_t Fr_Dr_side_leftright_target_backup;
extern int16_t Fr_Dr_side_updown_target_backup;
extern int16_t Fr_Dr_ctr_leftright_target_backup;
extern int16_t Fr_Dr_ctr_updown_target_backup;
extern int16_t Fr_Ps_side_leftright_target_backup;
extern int16_t Fr_Ps_side_updown_target_backup;
extern int16_t Fr_Ps_ctr_leftright_target_backup;
extern int16_t Fr_Ps_ctr_updown_target_backup;
extern int16_t Rr_Dr_side_leftright_target_backup;
extern int16_t Rr_Dr_side_updown_targe_backup;
extern int16_t Rr_Ps_ctr_leftright_target_backup;
extern int16_t Rr_Ps_ctr_updown_target_backup;

extern uint16_t Hu_FrDrEVntSidePt_X_backup;
extern uint16_t Hu_FrDrEVntSidePt_Y_backup;
extern uint16_t Hu_FrDrEVntCtrPt_X_backup;
extern uint16_t Hu_FrDrEVntCtrPt_Y_backup;
extern uint16_t Hu_FrPsEVntSidePt_X_backup;
extern uint16_t Hu_FrPsEVntSidePt_Y_backup;
extern uint16_t Hu_FrPsEVntCtrPt_X_backup;
extern uint16_t Hu_FrPsEVntCtrPt_Y_backup;
extern uint16_t Hu_RrDrEVntConsPt_X_backup;
extern uint16_t Hu_RrDrEVntConsPt_Y_backup;
extern uint16_t Hu_RrPsEVntConsPt_X_backup;
extern uint16_t Hu_RrPsEVntConsPt_Y_backup;


extern uint8_t Hu_Fr_dr_Vent_Side_signal_backup;
extern uint8_t Hu_Fr_dr_Vent_Center_signal_backup;
extern uint8_t Hu_Fr_Ps_Vent_Side_signal_backup;
extern uint8_t Hu_Fr_Ps_Vent_Center_signal_backup;

extern uint16_t Hu_FrDrEVntSidePt_X_invalid;
extern uint16_t Hu_FrDrEVntSidePt_Y_invalid;
extern uint16_t Hu_FrDrEVntCtrPt_X_invalid;
extern uint16_t Hu_FrDrEVntCtrPt_Y_invalid;

extern uint16_t Hu_FrPsEVntSidePt_X_invalid;
extern uint16_t Hu_FrPsEVntSidePt_Y_invalid;
extern uint16_t Hu_FrPsEVntCtrPt_X_invalid;
extern uint16_t Hu_FrPsEVntCtrPt_Y_invalid;


extern uint8_t recoveryFinishFlag_cllr;
extern uint8_t recoveryFinishFlag_clud;
extern uint8_t recoveryFinishFlag_sllr;
extern uint8_t recoveryFinishFlag_slud;
extern uint8_t recoveryFinishFlag_crlr;
extern uint8_t recoveryFinishFlag_crud;
extern uint8_t recoveryFinishFlag_srlr;
extern uint8_t recoveryFinishFlag_srud;





extern Drv_type ATCU_DriverSideType;

CanLin_oldNewValue_check_t CanValue =
{
	{	
		.Hu_FrDrEVntSidePt_X_oldVal = 150, //initial value
		.Hu_FrDrEVntSidePt_Y_oldVal = 180,
		.Hu_FrDrEVntCtrPt_X_oldVal = 150,
		.Hu_FrDrEVntCtrPt_Y_oldVal = 180
	},
	{
		.Hu_FrPsEVntSidePt_X_oldVal = 150,
		.Hu_FrPsEVntSidePt_Y_oldVal = 180,
		.Hu_FrPsEVntCtrPt_X_oldVal = 150,
		.Hu_FrPsEVntCtrPt_Y_oldVal = 180
	},
	{
		.Hu_RrDrEVntConsPt_X_oldVal = 150,
		.Hu_RrDrEVntConsPt_Y_oldVal = 180,
		.Hu_RrPsEVntConsPt_X_oldVal = 150,
		.Hu_RrPsEVntConsPt_Y_oldVal = 180
	}
};

SelectEVnt_t CurrentEVnt = 
{
	.selectEVnt = DefaultPt,
	.selectEVnt1 = DefaultPt
};

EvntMode_flag_t evntModeflags = {FALSE, };


void Lin_GoTo_Sleep(void)
{
//	lin_WakeupFlag = true;
	
	l_sch_set(LI0, LI0_GOTO_SLEEP_SCHEDULE, 0u);
	l_sch_set(LI1, LI1_GOTO_SLEEP_SCHEDULE, 0u);
	//l_sch_set(LI2, LI2_GOTO_SLEEP_SCHEDULE, 0u);
}

void Lin_GoTo_WakeUp(void)
{
	if(lin_WakeupFlag == true)
	{
		l_ifc_wake_up(LI0);
		l_ifc_wake_up(LI1);
		l_ifc_wake_up(LI2);
		
		lin_WakeupFlag = false;
	}
}

void Lin_FR_DR_BusOff_Check(FR_LinBusOff_flag_t *fault_flag)
{
	// SLLR LIN-Bus Off flag check
	if(l_flg_tst_LI0_FR_SLLR_EIDef_flag() || l_flg_tst_LI0_FR_SLLR_TSD_flag())
	{
		l_flg_clr_LI0_FR_SLLR_EIDef_flag();
		l_flg_clr_LI0_FR_SLLR_TSD_flag();

		if(fault_flag->FR_SLLR_LIN_BusOff == false)
		{
			FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLLR_ComFlt_Flag = false;
		}
	}
	else 
	{ 
		fault_flag->FR_SLLR_LIN_BusOff = true;

		if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
		{
			FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLLR_ComFlt_Flag = true;
		}
	}

	// SLUD LIN-Bus Off flag check
	if(l_flg_tst_LI0_FR_SLUD_EIDef_flag() || l_flg_tst_LI0_FR_SLUD_TSD_flag())
	{
		l_flg_clr_LI0_FR_SLUD_EIDef_flag();
		l_flg_clr_LI0_FR_SLUD_TSD_flag();

		if(fault_flag->FR_SLUD_LIN_BusOff == false)
		{
			FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLUD_ComFlt_Flag = false;
		}
	}
	else 
	{ 
		fault_flag->FR_SLUD_LIN_BusOff = true;

		if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
		{
			FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLUD_ComFlt_Flag = true;
		}
	}

	// CLLR LIN-Bus Off flag check
	if(l_flg_tst_LI0_FR_CLLR_EIDef_flag() || l_flg_tst_LI0_FR_CLLR_TSD_flag())
	{
		l_flg_clr_LI0_FR_CLLR_EIDef_flag();
		l_flg_clr_LI0_FR_CLLR_TSD_flag();

		if(fault_flag->FR_CLLR_LIN_BusOff == false)
		{
			FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLLR_ComFlt_Flag = false;
		}
	}
	else 
	{ 
		fault_flag->FR_CLLR_LIN_BusOff = true;

		if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
		{
			FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLLR_ComFlt_Flag = true;
		}
	}

	// CLUD LIN-Bus Off flag check
	if(l_flg_tst_LI0_FR_CLUD_EIDef_flag() || l_flg_tst_LI0_FR_CLUD_TSD_flag())
	{
		l_flg_clr_LI0_FR_CLUD_EIDef_flag();
		l_flg_clr_LI0_FR_CLUD_TSD_flag();

		if(fault_flag->FR_CLUD_LIN_BusOff == false)
		{
			FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLUD_ComFlt_Flag = false;
		}
	}
	else 
	{ 
		fault_flag->FR_CLUD_LIN_BusOff = true;

		if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
		{
			FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLUD_ComFlt_Flag = true;
		}
	}
}

void Lin_FR_PS_BusOff_Check(FR_LinBusOff_flag_t *fault_flag)
{
		// CRLR LIN-Bus Off flag check
	if(l_flg_tst_LI1_FR_CRLR_EIDef_flag() || l_flg_tst_LI1_FR_CRLR_TSD_flag())
	{
		l_flg_clr_LI1_FR_CRLR_EIDef_flag();
		l_flg_clr_LI1_FR_CRLR_TSD_flag();

		if(fault_flag->FR_CRLR_LIN_BusOff == false)
		{
			FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRLR_ComFlt_Flag = false;
		}
	}
	else 
	{ 
		fault_flag->FR_CRLR_LIN_BusOff = true;

		if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
		{
			FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRLR_ComFlt_Flag = true;
		}
	}

	// CRUD LIN-Bus Off flag check
	if(l_flg_tst_LI1_FR_CRUD_EIDef_flag() || l_flg_tst_LI1_FR_CRUD_TSD_flag())
	{
		l_flg_clr_LI1_FR_CRUD_EIDef_flag();
		l_flg_clr_LI1_FR_CRUD_TSD_flag();

		if(fault_flag->FR_CRUD_LIN_BusOff == false)
		{
			FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRUD_ComFlt_Flag = false;
		}
	}
	else 
	{ 
		fault_flag->FR_CRUD_LIN_BusOff = true;

		if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
		{
			FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRUD_ComFlt_Flag = true;
		}
	}


	// SRLR LIN-Bus Off flag check
	if(l_flg_tst_LI1_FR_SRLR_EIDef_flag() || l_flg_tst_LI1_FR_SRLR_TSD_flag())
	{
		l_flg_clr_LI1_FR_SRLR_EIDef_flag();
		l_flg_clr_LI1_FR_SRLR_TSD_flag();

		if(fault_flag->FR_SRLR_LIN_BusOff == false)
		{
			FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRLR_ComFlt_Flag = false;
		}
	}
	else 
	{ 
		fault_flag->FR_SRLR_LIN_BusOff = true;

		if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
		{
			FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRLR_ComFlt_Flag = true;
		}
	}

	// SRUD LIN-Bus Off flag check
	if(l_flg_tst_LI1_FR_SRUD_EIDef_flag() || l_flg_tst_LI1_FR_SRUD_TSD_flag())
	{
		l_flg_clr_LI1_FR_SRUD_EIDef_flag();
		l_flg_clr_LI1_FR_SRUD_TSD_flag();

		if(fault_flag->FR_SRUD_LIN_BusOff == false)
		{
			FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRUD_ComFlt_Flag = false;
		}
	}
	else 
	{ 
		fault_flag->FR_SRUD_LIN_BusOff = true;

		if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
		{
			FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRUD_ComFlt_Flag = true;
		}
	}

}

void lin_BusOff(void)
{
	Lin_FR_DR_BusOff_Check(&FR_LINBUSOff_FLAG);
	Lin_FR_PS_BusOff_Check(&FR_LINBUSOff_FLAG);
}

void Lin_Init(void)
{
	/* Initialize LIN network interface */
	l_sys_init();

	l_ifc_init(LI0);
	l_ifc_init(LI1);
	
#ifdef AMO_GN7_PE_SETTING_NONE
	l_ifc_init(LI2);
#endif

//	Lin_GoTo_WakeUp();

	/* Set Schedule table to Normal */

	l_sch_set(LI0, LI0_SCHEDULER_EVNT_NORMAL_FR_DRV, 0u);
	l_sch_set(LI1, LI1_SCHEDULER_EVNT_NORMAL_FR_PASS, 0u);

	Amo_timer_Stop(timer_90);
	Amo_timer_Start(timer_90, 500, true, lin_BusOff);
	
#ifdef AMO_GN7_PE_SETTING_NONE	
	l_sch_set(LI2, LI2_SCHEDULER_EVNT_NORMAL_RR_CTR, 0u);
#endif
	
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);
	
#ifdef AMO_GN7_PE_SETTING_NONE
	SetLIN_EVNT_RrSPEED(&LIN_RC_EVNT_MASTER_CMD, ACT_RPM_10);
#endif

}

void lin_sleep_task(void)
{
	if(sleepFlag == 1)
	{
		l_sch_set(LI0, LI0_GOTO_SLEEP_SCHEDULE, 0u);
		sleepFlag = 0;
	}
	if(sleepFlag == 2)
	{
		l_ifc_wake_up(LI0);
		l_sch_set(LI0, LI0_SCHEDULER_EVNT_NORMAL_FR_DRV, 0u);
		sleepFlag = 0;
	}
	 /* Check node state */
	if (LIN_NODE_STATE_SLEEP_MODE == lin_lld_get_state(LI0))
	{
//		PINS_DRV_SetPins(GPIO_PORTC, TEST_OUT1_MASK);
		/* Turn off all LEDs */
	}
	if (LIN_NODE_STATE_IDLE == lin_lld_get_state(LI0))
	{
//		PINS_DRV_ClearPins(GPIO_PORTC, TEST_OUT1_MASK);
		/* Turn off all LEDs */
	}
}

void Set_LinToCan_L_EVNT_HU_FrDrEvntPtDisp_XY(Lvnt_1_data *L1_DataBuf, Atcu_20_data *canPtDataBuf)
{
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntSidePtDisp_X = canPtDataBuf->data.L_ATCU_HU_FrDrEVntSidePt_X;
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = canPtDataBuf->data.L_ATCU_HU_FrDrEVntSidePt_Y;
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = canPtDataBuf->data.L_ATCU_HU_FrDrEVntCtrPt_X;
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = canPtDataBuf->data.L_ATCU_HU_FrDrEVntCtrPt_Y;

	Hu_FrDrEVntSidePt_X_backup = L1_DataBuf->data.L_EVNT_HU_FrDrEvntSidePtDisp_X;
	Hu_FrDrEVntSidePt_Y_backup = L1_DataBuf->data.L_EVNT_HU_FrDrEvntSidePtDisp_Y;
	Hu_FrDrEVntCtrPt_X_backup = L1_DataBuf->data.L_EVNT_HU_FrDrEvntCtrPtDisp_X;
	Hu_FrDrEVntCtrPt_Y_backup = L1_DataBuf->data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y;	
}

void Set_LinToCan_L_EVNT_HU_FrPsEvntPtDisp_XY(Lvnt_2_data *L2_DataBuf, Atcu_21_data *canPtDataBuf)
{
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntSidePtDisp_X = canPtDataBuf->data.L_ATCU_HU_FrPsEVntSidePt_X;
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = canPtDataBuf->data.L_ATCU_HU_FrPsEVntSidePt_Y;
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = canPtDataBuf->data.L_ATCU_HU_FrPsEVntCtrPt_X;
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = canPtDataBuf->data.L_ATCU_HU_FrPsEVntCtrPt_Y;

	Hu_FrPsEVntSidePt_X_backup = L2_DataBuf->data.L_EVNT_HU_FrPsEvntSidePtDisp_X;
	Hu_FrPsEVntSidePt_Y_backup = L2_DataBuf->data.L_EVNT_HU_FrPsEvntSidePtDisp_Y;
	Hu_FrPsEVntCtrPt_X_backup = L2_DataBuf->data.L_EVNT_HU_FrPsEvntCtrPtDisp_X;
	Hu_FrPsEVntCtrPt_Y_backup = L2_DataBuf->data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y;
}

void Set_LinToCan_L_EVNT_HU_RrEvntPtDisp_XY(Lvnt_3_data *L3_DataBuf, Atcu_22_data *canPtDataBuf)
{
	L3_DataBuf->data.L_EVNT_HU_RrDrEvntConsPtDisp_X = canPtDataBuf->data.L_ATCU_HU_RrDrEVntConsPt_X;
	L3_DataBuf->data.L_EVNT_HU_RrDrEvntConsPtDisp_Y = canPtDataBuf->data.L_ATCU_HU_RrDrEVntConsPt_Y;
	L3_DataBuf->data.L_EVNT_HU_RrPsEvntConsPtDisp_X = canPtDataBuf->data.L_ATCU_HU_RrPsEVntConsPt_X;
	L3_DataBuf->data.L_EVNT_HU_RrPsEvntConsPtDisp_Y = canPtDataBuf->data.L_ATCU_HU_RrPsEVntConsPt_Y;

	Hu_RrDrEVntConsPt_X_backup = L3_DataBuf->data.L_EVNT_HU_RrDrEvntConsPtDisp_X;
	Hu_RrDrEVntConsPt_Y_backup = L3_DataBuf->data.L_EVNT_HU_RrDrEvntConsPtDisp_Y;
	Hu_RrPsEVntConsPt_X_backup = L3_DataBuf->data.L_EVNT_HU_RrPsEvntConsPtDisp_X;
	Hu_RrPsEVntConsPt_Y_backup = L3_DataBuf->data.L_EVNT_HU_RrPsEvntConsPtDisp_Y;
}


void lin_master_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	if (l_flg_tst_LI0_FR_SLLR_ActuatorState_flag())
	{
	    //Clear this flag...
		l_flg_clr_LI0_FR_SLLR_ActuatorState_flag();
	}

	 /* Check node state */
	if (LIN_NODE_STATE_SLEEP_MODE == lin_lld_get_state(LI0))
	{
		/* Turn off all LEDs */
	}
}

void lin_Write_FrDrEVntPt_XY(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lvnt_3_data *L3_DataBuf)
{
	//if(L3_DataBuf->data.L_EVNT_FrDrEvntCtrFlt != E_ERROR)
	{
		l_u16_wr_LI0_EVNT_Front_Center_LH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition);
	}
	//if(L3_DataBuf->data.L_EVNT_FrDrEvntSideFlt != E_ERROR)
	{
		l_u16_wr_LI0_EVNT_Front_Side_LH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition);
		l_u16_wr_LI0_EVNT_Front_Side_LH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition);
	}
	//if(L3_DataBuf->data.L_EVNT_FrDrEvntCtrFlt != E_ERROR)
	{
		l_u16_wr_LI0_EVNT_Front_Center_LH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition);
	}
}

void lin_Write_FrPsEVntPt_XY(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lvnt_3_data *L3_DataBuf)
{
//	if(L3_DataBuf->data.L_EVNT_FrPsEvntCtrFlt != E_ERROR)
	{
		l_u16_wr_LI1_EVNT_Front_Center_RH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition);
		l_u16_wr_LI1_EVNT_Front_Center_RH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition);
	}
//	if(L3_DataBuf->data.L_EVNT_FrPsEvntSideFlt != E_ERROR)
	{
		l_u16_wr_LI1_EVNT_Front_Side_RH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition);
		l_u16_wr_LI1_EVNT_Front_Side_RH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition);
	}
}

void lin_Write_RrEVntConsPt_XY(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl)
{
	l_u16_wr_LI2_EVNT_Rear_Center_RH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition);
	l_u16_wr_LI2_EVNT_Rear_Center_RH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition);
	l_u16_wr_LI2_EVNT_Rear_Center_LH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition);
	l_u16_wr_LI2_EVNT_Rear_Center_LH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition);
}


void lin_FrDrmanualMode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	recoveryFinishFlag_cllr = FALSE;
	recoveryFinishFlag_clud = FALSE;
	recoveryFinishFlag_sllr = FALSE;
	recoveryFinishFlag_slud = FALSE;
	
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);
//	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_1); //test hkh

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	Amo_timer_Stop(timer_60);
//	Amo_timer_Start(timer_60, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_LH_Check);
	
	Set_LinToCan_L_EVNT_HU_FrDrEvntPtDisp_XY(&Can_Tx_Evnt_1, &Can_data_20_Info);
	//Set_LinToCan_Disp_FrDr_Mode(&Can_Tx_Evnt_4);
}

void lin_FrDrmanualMode_task_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl) //MODE BACKUP
{
	recoveryFinishFlag_cllr = FALSE;
	recoveryFinishFlag_clud = FALSE;
	recoveryFinishFlag_sllr = FALSE;
	recoveryFinishFlag_slud = FALSE;

	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);
	
	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	Amo_timer_Stop(timer_60);
//	Amo_timer_Start(timer_60, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_LH_Check);
	
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_X = Hu_FrDrEVntSidePt_X_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = Hu_FrDrEVntSidePt_Y_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = Hu_FrDrEVntCtrPt_X_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = Hu_FrDrEVntCtrPt_Y_backup;

	//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = Hu_Fr_Dr_Vent_Mode;
	//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = Hu_Fr_dr_Vent_Side_signal;
	//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = Hu_Fr_dr_Vent_Center_signal;
}

void lin_FrPsmanualMode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	recoveryFinishFlag_crlr = FALSE;
	recoveryFinishFlag_crud = FALSE;
	recoveryFinishFlag_srlr = FALSE;
	recoveryFinishFlag_srud = FALSE;

	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);
	
	if(l_flg_tst_LI1_EVNT_Front_Side_RH_LeftRight_TargetPosition_flag() || l_flg_tst_LI1_EVNT_Front_Center_RH_LeftRight_TargetPosition_flag())  //
	{
	  //Clear this flag...
	  l_flg_clr_LI1_EVNT_Front_Side_RH_LeftRight_TargetPosition_flag();
		l_flg_clr_LI1_EVNT_Front_Center_RH_LeftRight_TargetPosition_flag();

		lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
	
		Amo_timer_Stop(timer_63);
//		Amo_timer_Start(timer_63, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RH_Check);
	}

	Set_LinToCan_L_EVNT_HU_FrPsEvntPtDisp_XY(&Can_Tx_Evnt_2, &Can_data_21_Info);
	//Set_LinToCan_Disp_FrPs_Mode(&Can_Tx_Evnt_4);
}

void lin_FrPsmanualMode_task_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl) //MODE BACKUP
{
	recoveryFinishFlag_crlr = FALSE;
	recoveryFinishFlag_crud = FALSE;
	recoveryFinishFlag_srlr = FALSE;
	recoveryFinishFlag_srud = FALSE;
	
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);

	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
	
	Amo_timer_Stop(timer_63);
//	Amo_timer_Start(timer_63, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RH_Check);
	
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_X = Hu_FrPsEVntSidePt_X_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = Hu_FrPsEVntSidePt_Y_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = Hu_FrPsEVntCtrPt_X_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = Hu_FrPsEVntCtrPt_Y_backup;

	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = Hu_Fr_Ps_Vent_Mode;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = Hu_Fr_Ps_Vent_Side_signal;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = Hu_Fr_Ps_Vent_Center_signal;	
}

void lin_RrmanualMode_task(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl)
{
	SetLIN_EVNT_RrSPEED(&LIN_RC_EVNT_MASTER_CMD, ACT_RPM_10);
	
	if(l_flg_tst_LI2_EVNT_Rear_Center_LH_LeftRight_TargetPosition_flag() || l_flg_tst_LI2_EVNT_Rear_Center_RH_LeftRight_TargetPosition_flag())
	{
	    //Clear this flag...
	  l_flg_clr_LI2_EVNT_Rear_Center_LH_LeftRight_TargetPosition_flag();
		l_flg_clr_LI2_EVNT_Rear_Center_RH_LeftRight_TargetPosition_flag();

		lin_Write_RrEVntConsPt_XY(&LIN_RC_EVNT_MASTER_CMD);

		Amo_timer_Stop(timer_66);
		Amo_timer_Start(timer_66, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RC_Check);
	}
	
	Set_LinToCan_L_EVNT_HU_RrEvntPtDisp_XY(&Can_Tx_Evnt_3, &Can_data_22_Info);
	Set_LinToCan_Disp_RrDr_Mode(&Can_Tx_Evnt_4);
	Set_LinToCan_Disp_RrPs_Mode(&Can_Tx_Evnt_4);
}

void lin_FrDrVent_FullClose_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
	recoveryFinishFlag_cllr = FALSE;
	recoveryFinishFlag_clud = FALSE;
	recoveryFinishFlag_sllr = FALSE;
	recoveryFinishFlag_slud = FALSE;
	
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);

	if(Hu_Fr_dr_Vent_Side_signal_backup == 0x02) //fullclose
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);

		LIN_EVNT_PT.FrDrPt.prevPt_FR_SLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrDrPt.prevPt_FR_SLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;

		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = FullClose_min;
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = FullClose_UD;

		LIN_EVNT_PT.FrDrPt.currentPt_FR_SLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrDrPt.currentPt_FR_SLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;
	}
	
	if(Hu_Fr_dr_Vent_Side_signal_backup == 0x02 && Hu_Fr_dr_Vent_Center_signal_backup != 0x02)
	{
		if(Hu_Fr_Dr_Vent_Mode == SWING_MODE)
		{
			lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
		}	
	}
	else
	{
	
	}

	if(Hu_Fr_dr_Vent_Center_signal_backup == 0x02) //fullclose
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);

		lin_Pt->FrDrPt.prevPt_FR_CLLR = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		lin_Pt->FrDrPt.prevPt_FR_CLUD = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;

		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = FullClose_max;
		//lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = FullClose_UD;
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = FullClose_UD_Rev;

		lin_Pt->FrDrPt.currentPt_FR_CLLR = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		lin_Pt->FrDrPt.currentPt_FR_CLUD = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
	}
	
	if(Hu_Fr_dr_Vent_Center_signal_backup == 0x02 && Hu_Fr_dr_Vent_Side_signal_backup != 0x02)
	{
		if(Hu_Fr_Dr_Vent_Mode == SWING_MODE)
		{
			lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
		} 
	}
	else
	{

	}

		lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
		Amo_timer_Stop(timer_60);
//		Amo_timer_Start(timer_60, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_LH_Check);
		Set_LinToCan_Disp_FrDr_XY(&Can_Tx_Evnt_1);
		//Set_LinToCan_Disp_FrDr_Mode(&Can_Tx_Evnt_4);	

}

void RHD_lin_FrDrVent_FullClose_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	recoveryFinishFlag_cllr = FALSE;
	recoveryFinishFlag_clud = FALSE;
	recoveryFinishFlag_sllr = FALSE;
	recoveryFinishFlag_slud = FALSE;
	
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);

	if(Hu_Fr_Ps_Vent_Side_signal_backup == 0x02) //fullclose
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);

		LIN_EVNT_PT.FrDrPt.prevPt_FR_SLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrDrPt.prevPt_FR_SLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;
			
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = FullClose_min;
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = FullClose_UD;

		LIN_EVNT_PT.FrDrPt.currentPt_FR_SLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrDrPt.currentPt_FR_SLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;
	}
	if(Hu_Fr_dr_Vent_Side_signal_backup == 0x02 && Hu_Fr_dr_Vent_Center_signal_backup != 0x02)
	{
		if(Hu_Fr_Dr_Vent_Mode == SWING_MODE)
		{
			lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
		}	
	}
	else
	{
	
	}

	if(Hu_Fr_Ps_Vent_Center_signal_backup == 0x02) //fullclose
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);

		LIN_EVNT_PT.FrDrPt.prevPt_FR_CLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrDrPt.prevPt_FR_CLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;

		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = FullClose_max;
		//lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = FullClose_UD;
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = FullClose_UD_Rev;

		LIN_EVNT_PT.FrDrPt.currentPt_FR_CLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrDrPt.currentPt_FR_CLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
	}
	
	if(Hu_Fr_dr_Vent_Center_signal_backup == 0x02 && Hu_Fr_dr_Vent_Side_signal_backup != 0x02)
	{
		if(Hu_Fr_Dr_Vent_Mode == SWING_MODE)
		{
		lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
		} 
	}
	else
	{

	}

		lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
		Amo_timer_Stop(timer_60);
//		Amo_timer_Start(timer_60, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_LH_Check);
		Set_LinToCan_Disp_FrDr_XY(&Can_Tx_Evnt_1);
		//Set_LinToCan_Disp_FrDr_Mode(&Can_Tx_Evnt_4);	

}


void lin_FrDrVent_Focus_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
	recoveryFinishFlag_cllr = FALSE;
	recoveryFinishFlag_clud = FALSE;
	recoveryFinishFlag_sllr = FALSE;
	recoveryFinishFlag_slud = FALSE;
	
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);

	if(Hu_Fr_Dr_Vent_Mode == FACING_MODE) //focus mode
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		LIN_EVNT_PT.FrDrPt.prevPt_FR_SLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrDrPt.prevPt_FR_SLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;
			
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Half_max_LR;
		
#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = FocusSpread_UD;
#else
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 0;
#endif

		LIN_EVNT_PT.FrDrPt.currentPt_FR_SLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrDrPt.currentPt_FR_SLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;

	}

	if(Hu_Fr_Dr_Vent_Mode == FACING_MODE) //focus mode
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		lin_Pt->FrDrPt.prevPt_FR_CLLR = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		lin_Pt->FrDrPt.prevPt_FR_CLUD = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
				
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Half_min_LR;
		
#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = FocusSpread_UD_Rev;
#else
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 0;
#endif
		lin_Pt->FrDrPt.currentPt_FR_CLLR = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		lin_Pt->FrDrPt.currentPt_FR_CLUD = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
	}
	
		lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
		Amo_timer_Stop(timer_60);
//		Amo_timer_Start(timer_60, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_LH_Check);
		Set_LinToCan_Disp_FrDr_XY(&Can_Tx_Evnt_1);
		//Set_LinToCan_Disp_FrDr_Mode(&Can_Tx_Evnt_4);
}

void RHD_lin_FrDrVent_Focus_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	recoveryFinishFlag_cllr = FALSE;
	recoveryFinishFlag_clud = FALSE;
	recoveryFinishFlag_sllr = FALSE;
	recoveryFinishFlag_slud = FALSE;
	
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);

	if(Hu_Fr_Ps_Vent_Mode == FACING_MODE)//focus mode
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		LIN_EVNT_PT.FrDrPt.prevPt_FR_SLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrDrPt.prevPt_FR_SLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;
		
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Half_max_LR;
		
#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = FocusSpread_UD;
#else
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 0;
#endif

		LIN_EVNT_PT.FrDrPt.currentPt_FR_SLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrDrPt.currentPt_FR_SLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;

	}

	if(Hu_Fr_Ps_Vent_Mode == FACING_MODE) //focus mode
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		LIN_EVNT_PT.FrDrPt.prevPt_FR_CLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrDrPt.prevPt_FR_CLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
			
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Half_min_LR;
		
#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = FocusSpread_UD_Rev;
#else
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 0;
#endif

			LIN_EVNT_PT.FrDrPt.currentPt_FR_CLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrDrPt.currentPt_FR_CLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;

	}
	
		lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
		Amo_timer_Stop(timer_60);
		Amo_timer_Start(timer_60, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_LH_Check);
		Set_LinToCan_Disp_FrDr_XY(&Can_Tx_Evnt_1);
		//Set_LinToCan_Disp_FrDr_Mode(&Can_Tx_Evnt_4);
}


void lin_FrDrVent_Spread_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
	recoveryFinishFlag_cllr = FALSE;
	recoveryFinishFlag_clud = FALSE;
	recoveryFinishFlag_sllr = FALSE;
	recoveryFinishFlag_slud = FALSE;
	
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);

	if(Hu_Fr_Dr_Vent_Mode == AVOID_MODE) //spread mode
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		LIN_EVNT_PT.FrDrPt.prevPt_FR_SLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrDrPt.prevPt_FR_SLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;
					

		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition =  Half_min_LR;
		
#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = FocusSpread_UD;
#else
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
#endif

		LIN_EVNT_PT.FrDrPt.currentPt_FR_SLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrDrPt.currentPt_FR_SLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;

	}

	if(Hu_Fr_Dr_Vent_Mode == AVOID_MODE) //spread mode
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		lin_Pt->FrDrPt.prevPt_FR_CLLR = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		lin_Pt->FrDrPt.prevPt_FR_CLUD = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
		
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Half_max_LR;
		
#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = FocusSpread_UD_Rev;
#else
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
#endif
		lin_Pt->FrDrPt.currentPt_FR_CLLR = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		lin_Pt->FrDrPt.currentPt_FR_CLUD = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
	}
		lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
		Amo_timer_Stop(timer_60);
//		Amo_timer_Start(timer_60, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_LH_Check);
		Set_LinToCan_Disp_FrDr_XY(&Can_Tx_Evnt_1);
		//Set_LinToCan_Disp_FrDr_Mode(&Can_Tx_Evnt_4);
}

void RHD_lin_FrDrVent_Spread_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	recoveryFinishFlag_cllr = FALSE;
	recoveryFinishFlag_clud = FALSE;
	recoveryFinishFlag_sllr = FALSE;
	recoveryFinishFlag_slud = FALSE;
	
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);

	if(Hu_Fr_Ps_Vent_Mode == AVOID_MODE) //spread mode
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		LIN_EVNT_PT.FrDrPt.prevPt_FR_SLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrDrPt.prevPt_FR_SLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;

		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition =  Half_min_LR;
		
#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = FocusSpread_UD;
#else
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
#endif

		LIN_EVNT_PT.FrDrPt.currentPt_FR_SLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrDrPt.currentPt_FR_SLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;

	}

	if(Hu_Fr_Ps_Vent_Mode == AVOID_MODE) //spread mode
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		LIN_EVNT_PT.FrDrPt.prevPt_FR_CLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrDrPt.prevPt_FR_CLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
		
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Half_max_LR;		
#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = FocusSpread_UD_Rev;
#else
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
#endif

		LIN_EVNT_PT.FrDrPt.currentPt_FR_CLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrDrPt.currentPt_FR_CLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;

	}
		lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
		Amo_timer_Stop(timer_60);
//		Amo_timer_Start(timer_60, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_LH_Check);
		Set_LinToCan_Disp_FrDr_XY(&Can_Tx_Evnt_1);
		//Set_LinToCan_Disp_FrDr_Mode(&Can_Tx_Evnt_4);
}


void lin_FrDrVent_FullClose_Mode_task_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
	recoveryFinishFlag_cllr = FALSE;
	recoveryFinishFlag_clud = FALSE;
	recoveryFinishFlag_sllr = FALSE;
	recoveryFinishFlag_slud = FALSE;
	
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);
	
	if(Hu_Fr_dr_Vent_Side_signal_backup == 0x02) //fullclose
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);

		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = FullClose_min;
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = FullClose_UD;
	}

	if(Hu_Fr_dr_Vent_Center_signal_backup == 0x02) //fullclose
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);

		lin_Pt->FrDrPt.prevPt_FR_CLLR = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		lin_Pt->FrDrPt.prevPt_FR_CLUD = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;

		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = FullClose_max;
		//lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = FullClose_UD;
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = FullClose_UD_Rev;

		lin_Pt->FrDrPt.currentPt_FR_CLLR = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		lin_Pt->FrDrPt.currentPt_FR_CLUD = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
	}

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	Amo_timer_Stop(timer_60);
//	Amo_timer_Start(timer_60, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_LH_Check);
		
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_X = Hu_FrDrEVntSidePt_X_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = Hu_FrDrEVntSidePt_Y_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = Hu_FrDrEVntCtrPt_X_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = Hu_FrDrEVntCtrPt_Y_backup;

	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = Hu_Fr_Dr_Vent_Mode;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = Hu_Fr_dr_Vent_Side_signal;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = Hu_Fr_dr_Vent_Center_signal;			
}


void RHD_lin_FrDrVent_FullClose_Mode_task_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	recoveryFinishFlag_cllr = FALSE;
	recoveryFinishFlag_clud = FALSE;
	recoveryFinishFlag_sllr = FALSE;
	recoveryFinishFlag_slud = FALSE;
	
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);
	
	if(Hu_Fr_Ps_Vent_Side_signal_backup == 0x02) //fullclose
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);

		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = FullClose_min;
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = FullClose_UD;
	}

	if(Hu_Fr_Ps_Vent_Center_signal_backup == 0x02) //fullclose
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);

		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = FullClose_max;
		//lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = FullClose_UD;
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = FullClose_UD_Rev;
	}

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	Amo_timer_Stop(timer_60);
//	Amo_timer_Start(timer_60, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_LH_Check);
		
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_X = Hu_FrDrEVntSidePt_X_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = Hu_FrDrEVntSidePt_Y_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = Hu_FrDrEVntCtrPt_X_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = Hu_FrDrEVntCtrPt_Y_backup;

	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = Hu_Fr_Dr_Vent_Mode;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = Hu_Fr_dr_Vent_Side_signal;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = Hu_Fr_dr_Vent_Center_signal;			
}

void lin_FrDrVent_Focus_Mode_task_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);

	recoveryFinishFlag_cllr = FALSE;
	recoveryFinishFlag_clud = FALSE;
	recoveryFinishFlag_sllr = FALSE;
	recoveryFinishFlag_slud = FALSE;

	if(Hu_Fr_Dr_Vent_Mode == FACING_MODE) //focus mode
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Half_max_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = FocusSpread_UD;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 0;
		#endif

	}
		
	if(Hu_Fr_Dr_Vent_Mode == FACING_MODE) //focus mode
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		lin_Pt->FrDrPt.prevPt_FR_CLLR = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		lin_Pt->FrDrPt.prevPt_FR_CLUD = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
		
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Half_min_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = FocusSpread_UD_Rev;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 0;
		#endif

		lin_Pt->FrDrPt.currentPt_FR_CLLR = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		lin_Pt->FrDrPt.currentPt_FR_CLUD = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
	}

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	Amo_timer_Stop(timer_60);
//	Amo_timer_Start(timer_60, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_LH_Check);

	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_X = Hu_FrDrEVntSidePt_X_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = Hu_FrDrEVntSidePt_Y_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = Hu_FrDrEVntCtrPt_X_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = Hu_FrDrEVntCtrPt_Y_backup;

	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = Hu_Fr_Dr_Vent_Mode;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = Hu_Fr_dr_Vent_Side_signal;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = Hu_Fr_dr_Vent_Center_signal;			
}

void RHD_lin_FrDrVent_Focus_Mode_task_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	recoveryFinishFlag_cllr = FALSE;
	recoveryFinishFlag_clud = FALSE;
	recoveryFinishFlag_sllr = FALSE;
	recoveryFinishFlag_slud = FALSE;
	
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);

	if(Hu_Fr_Ps_Vent_Mode == FACING_MODE) //focus mode
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Half_max_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = FocusSpread_UD;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 0;
		#endif

	}
		
	if(Hu_Fr_Ps_Vent_Mode == FACING_MODE) //focus mode
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Half_min_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = FocusSpread_UD_Rev;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 0;
		#endif

	}

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	Amo_timer_Stop(timer_60);
//	Amo_timer_Start(timer_60, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_LH_Check);

	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_X = Hu_FrDrEVntSidePt_X_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = Hu_FrDrEVntSidePt_Y_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = Hu_FrDrEVntCtrPt_X_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = Hu_FrDrEVntCtrPt_Y_backup;

	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = Hu_Fr_Dr_Vent_Mode;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = Hu_Fr_dr_Vent_Side_signal;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = Hu_Fr_dr_Vent_Center_signal;			
}



void lin_FrDrVent_Spread_Mode_task_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);
	
	recoveryFinishFlag_cllr = FALSE;
	recoveryFinishFlag_clud = FALSE;
	recoveryFinishFlag_sllr = FALSE;
	recoveryFinishFlag_slud = FALSE;


	if(Hu_Fr_Dr_Vent_Mode == AVOID_MODE) //spread mode
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition =  Half_min_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = FocusSpread_UD;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
		#endif

	}
		
	if(Hu_Fr_Dr_Vent_Mode == AVOID_MODE) //spread mode
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		lin_Pt->FrDrPt.prevPt_FR_CLLR = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		lin_Pt->FrDrPt.prevPt_FR_CLUD = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
		
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Half_max_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = FocusSpread_UD_Rev;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
		#endif		

		lin_Pt->FrDrPt.currentPt_FR_CLLR = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		lin_Pt->FrDrPt.currentPt_FR_CLUD = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
	}

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	Amo_timer_Stop(timer_60);
//	Amo_timer_Start(timer_60, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_LH_Check);
	
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_X = Hu_FrDrEVntSidePt_X_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = Hu_FrDrEVntSidePt_Y_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = Hu_FrDrEVntCtrPt_X_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = Hu_FrDrEVntCtrPt_Y_backup;

	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = Hu_Fr_Dr_Vent_Mode;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = Hu_Fr_dr_Vent_Side_signal;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = Hu_Fr_dr_Vent_Center_signal;			
}


void RHD_lin_FrDrVent_Spread_Mode_task_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	recoveryFinishFlag_cllr = FALSE;
	recoveryFinishFlag_clud = FALSE;
	recoveryFinishFlag_sllr = FALSE;
	recoveryFinishFlag_slud = FALSE;
	
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);

	if(Hu_Fr_Ps_Vent_Mode == AVOID_MODE) //spread mode
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition =  Half_min_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = FocusSpread_UD;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
		#endif

	}
		
	if(Hu_Fr_Ps_Vent_Mode == AVOID_MODE) //spread mode
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Half_max_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = FocusSpread_UD_Rev;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
		#endif
	}

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	Amo_timer_Stop(timer_60);
//	Amo_timer_Start(timer_60, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_LH_Check);
	
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_X = Hu_FrDrEVntSidePt_X_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = Hu_FrDrEVntSidePt_Y_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = Hu_FrDrEVntCtrPt_X_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = Hu_FrDrEVntCtrPt_Y_backup;

	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = Hu_Fr_Dr_Vent_Mode;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = Hu_Fr_dr_Vent_Side_signal;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = Hu_Fr_dr_Vent_Center_signal;			
}


void lin_FrPsVent_FullClose_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	recoveryFinishFlag_crlr = FALSE;
	recoveryFinishFlag_crud = FALSE;
	recoveryFinishFlag_srlr = FALSE;
	recoveryFinishFlag_srud = FALSE;
	
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);

	if(Hu_Fr_Ps_Vent_Side_signal_backup == 0x02) //fullclose
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		LIN_EVNT_PT.FrPsPt.prevPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.prevPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
		
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = FullClose_max;
		//lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = FullClose_UD;
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = FullClose_UD_Rev;

		LIN_EVNT_PT.FrPsPt.currentPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.currentPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
	}
	if(Hu_Fr_Ps_Vent_Side_signal_backup == 0x02 && Hu_Fr_Ps_Vent_Center_signal_backup != 0x02)
	{
		if(Hu_Fr_Ps_Vent_Mode == SWING_MODE)
		{
			lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
		}	
	}
	else
	{
	
	}

	if(Hu_Fr_Ps_Vent_Center_signal_backup == 0x02) //fullclose
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		LIN_EVNT_PT.FrPsPt.prevPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.prevPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
		
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = FullClose_min;
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = FullClose_UD;

		LIN_EVNT_PT.FrPsPt.currentPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.currentPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
	}
	if(Hu_Fr_Ps_Vent_Center_signal_backup == 0x02 && Hu_Fr_Ps_Vent_Side_signal_backup != 0x02)
	{
		if(Hu_Fr_Ps_Vent_Mode == SWING_MODE)
		{
			lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
		} 
	}
	else
	{

	}

		lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
		Amo_timer_Stop(timer_63);
//		Amo_timer_Start(timer_63, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RH_Check);
		Set_LinToCan_Disp_FrPs_XY(&Can_Tx_Evnt_2);
		//Set_LinToCan_Disp_FrPs_Mode(&Can_Tx_Evnt_4);	
}

void RHD_lin_FrPsVent_FullClose_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	recoveryFinishFlag_crlr = FALSE;
	recoveryFinishFlag_crud = FALSE;
	recoveryFinishFlag_srlr = FALSE;
	recoveryFinishFlag_srud = FALSE;
	
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);

	if(Hu_Fr_dr_Vent_Side_signal_backup == 0x02) //fullclose
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		LIN_EVNT_PT.FrPsPt.prevPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.prevPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
			
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = FullClose_max;
		//lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = FullClose_UD;
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = FullClose_UD_Rev;
		
		LIN_EVNT_PT.FrPsPt.prevPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.prevPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
	}
	if(Hu_Fr_Ps_Vent_Side_signal_backup == 0x02 && Hu_Fr_Ps_Vent_Center_signal_backup != 0x02)
	{
		if(Hu_Fr_Ps_Vent_Mode == SWING_MODE)
		{
			lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
		}	
	}
	else
	{
	
	}

	if(Hu_Fr_dr_Vent_Center_signal_backup == 0x02) //fullclose
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		LIN_EVNT_PT.FrPsPt.prevPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.prevPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
			
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = FullClose_min;
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = FullClose_UD;

		LIN_EVNT_PT.FrPsPt.currentPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.currentPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
	}
	if(Hu_Fr_Ps_Vent_Center_signal_backup == 0x02 && Hu_Fr_Ps_Vent_Side_signal_backup != 0x02)
	{
		if(Hu_Fr_Ps_Vent_Mode == SWING_MODE)
		{
			lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
		} 
	}
	else
	{

	}

		lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
		Amo_timer_Stop(timer_63);
//		Amo_timer_Start(timer_63, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RH_Check);
		Set_LinToCan_Disp_FrPs_XY(&Can_Tx_Evnt_2);
		//Set_LinToCan_Disp_FrPs_Mode(&Can_Tx_Evnt_4);	
}



void lin_FrPsVent_Spread_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	recoveryFinishFlag_crlr = FALSE;
	recoveryFinishFlag_crud = FALSE;
	recoveryFinishFlag_srlr = FALSE;
	recoveryFinishFlag_srud = FALSE;
	
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);

	if(Hu_Fr_Ps_Vent_Mode == AVOID_MODE) //spread mode
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		LIN_EVNT_PT.FrPsPt.prevPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.prevPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;

		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Half_max_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = FocusSpread_UD_Rev;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
		#endif

		LIN_EVNT_PT.FrPsPt.currentPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.currentPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
	}
	
	if(Hu_Fr_Ps_Vent_Mode == AVOID_MODE) //spread mode
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		LIN_EVNT_PT.FrPsPt.prevPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.prevPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
		
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Half_min_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = FocusSpread_UD;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
		#endif

		LIN_EVNT_PT.FrPsPt.currentPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.currentPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
	}
		lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
		Amo_timer_Stop(timer_63);
//		Amo_timer_Start(timer_63, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RH_Check);
		Set_LinToCan_Disp_FrPs_XY(&Can_Tx_Evnt_2);
		//Set_LinToCan_Disp_FrPs_Mode(&Can_Tx_Evnt_4);		
}

void RHD_lin_FrPsVent_Spread_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	recoveryFinishFlag_crlr = FALSE;
	recoveryFinishFlag_crud = FALSE;
	recoveryFinishFlag_srlr = FALSE;
	recoveryFinishFlag_srud = FALSE;
	
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);

	if(Hu_Fr_Dr_Vent_Mode == AVOID_MODE) //spread mode
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		LIN_EVNT_PT.FrPsPt.prevPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.prevPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
		
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Half_max_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = FocusSpread_UD_Rev;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
		#endif
		
		LIN_EVNT_PT.FrPsPt.currentPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.currentPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;

	}
	
	if(Hu_Fr_Dr_Vent_Mode == AVOID_MODE) //spread mode
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		LIN_EVNT_PT.FrPsPt.prevPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.prevPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;

		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Half_min_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = FocusSpread_UD;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
		#endif

		LIN_EVNT_PT.FrPsPt.currentPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.currentPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;

	}
		lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
		Amo_timer_Stop(timer_63);
//		Amo_timer_Start(timer_63, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RH_Check);
		Set_LinToCan_Disp_FrPs_XY(&Can_Tx_Evnt_2);
		//Set_LinToCan_Disp_FrPs_Mode(&Can_Tx_Evnt_4);		
}


void lin_FrPsVent_Focus_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	recoveryFinishFlag_crlr = FALSE;
	recoveryFinishFlag_crud = FALSE;
	recoveryFinishFlag_srlr = FALSE;
	recoveryFinishFlag_srud = FALSE;
	
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);

	if(Hu_Fr_Ps_Vent_Mode == FACING_MODE) //focus mode
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		LIN_EVNT_PT.FrPsPt.prevPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.prevPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
			
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Half_min_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = FocusSpread_UD_Rev;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = 0;
		#endif

		LIN_EVNT_PT.FrPsPt.currentPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.currentPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;

	}
	
	if(Hu_Fr_Ps_Vent_Mode == FACING_MODE) //focus mode
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		LIN_EVNT_PT.FrPsPt.prevPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.prevPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
			
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Half_max_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = FocusSpread_UD;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = 0;
		#endif

		LIN_EVNT_PT.FrPsPt.currentPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.currentPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
	}
		lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
		Amo_timer_Stop(timer_63);
//		Amo_timer_Start(timer_63, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RH_Check);
		Set_LinToCan_Disp_FrPs_XY(&Can_Tx_Evnt_2);
		//Set_LinToCan_Disp_FrPs_Mode(&Can_Tx_Evnt_4);
}

void RHD_lin_FrPsVent_Focus_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	recoveryFinishFlag_crlr = FALSE;
	recoveryFinishFlag_crud = FALSE;
	recoveryFinishFlag_srlr = FALSE;
	recoveryFinishFlag_srud = FALSE;
	
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);

	if(Hu_Fr_Dr_Vent_Mode == FACING_MODE) //focus mode
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		LIN_EVNT_PT.FrPsPt.prevPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.prevPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
			
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Half_min_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = FocusSpread_UD_Rev;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = 0;
		#endif

		LIN_EVNT_PT.FrPsPt.currentPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.currentPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;

	}
	
	if(Hu_Fr_Dr_Vent_Mode == FACING_MODE) //focus mode
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

		LIN_EVNT_PT.FrPsPt.prevPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.prevPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
			
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Half_max_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = FocusSpread_UD;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = 0;
		#endif
		
		LIN_EVNT_PT.FrPsPt.currentPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
		LIN_EVNT_PT.FrPsPt.currentPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;

	}
		lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
		Amo_timer_Stop(timer_63);
//		Amo_timer_Start(timer_63, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RH_Check);
		Set_LinToCan_Disp_FrPs_XY(&Can_Tx_Evnt_2);
		//Set_LinToCan_Disp_FrPs_Mode(&Can_Tx_Evnt_4);
}


void lin_FrPsVent_FullClose_Mode_task_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	recoveryFinishFlag_crlr = FALSE;
	recoveryFinishFlag_crud = FALSE;
	recoveryFinishFlag_srlr = FALSE;
	recoveryFinishFlag_srud = FALSE;
	
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);

	if(Hu_Fr_Ps_Vent_Side_signal_backup == 0x02) //fullclose
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = FullClose_max;
		//lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = FullClose_UD;
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = FullClose_UD_Rev;
	}
		
	if(Hu_Fr_Ps_Vent_Center_signal_backup == 0x02) //fullclose
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = FullClose_min;
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = FullClose_UD;
	}
		
	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	Amo_timer_Stop(timer_63);
//	Amo_timer_Start(timer_63, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RH_Check);

	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_X = Hu_FrPsEVntSidePt_X_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = Hu_FrPsEVntSidePt_Y_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = Hu_FrPsEVntCtrPt_X_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = Hu_FrPsEVntCtrPt_Y_backup;

	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = Hu_Fr_Ps_Vent_Mode;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = Hu_Fr_Ps_Vent_Side_signal;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = Hu_Fr_Ps_Vent_Center_signal;
}


void RHD_lin_FrPsVent_FullClose_Mode_task_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	recoveryFinishFlag_crlr = FALSE;
	recoveryFinishFlag_crud = FALSE;
	recoveryFinishFlag_srlr = FALSE;
	recoveryFinishFlag_srud = FALSE;
	
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);

	if(Hu_Fr_dr_Vent_Side_signal_backup == 0x02) //fullclose
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = FullClose_max;
		//lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = FullClose_UD;
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = FullClose_UD_Rev;
	}
		
	if(Hu_Fr_dr_Vent_Center_signal_backup == 0x02) //fullclose
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = FullClose_min;
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = FullClose_UD;
	}
		
	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	Amo_timer_Stop(timer_63);
//	Amo_timer_Start(timer_63, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RH_Check);

	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_X = Hu_FrPsEVntSidePt_X_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = Hu_FrPsEVntSidePt_Y_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = Hu_FrPsEVntCtrPt_X_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = Hu_FrPsEVntCtrPt_Y_backup;

	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = Hu_Fr_Ps_Vent_Mode;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = Hu_Fr_Ps_Vent_Side_signal;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = Hu_Fr_Ps_Vent_Center_signal;
}

void lin_FrPsVent_Focus_Mode_task_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	recoveryFinishFlag_crlr = FALSE;
	recoveryFinishFlag_crud = FALSE;
	recoveryFinishFlag_srlr = FALSE;
	recoveryFinishFlag_srud = FALSE;
	
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);
	
	if(Hu_Fr_Ps_Vent_Mode == FACING_MODE) //focus mode
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Half_min_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = FocusSpread_UD_Rev;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = 0;
		#endif

	}
		
	if(Hu_Fr_Ps_Vent_Mode == FACING_MODE) //focus mode
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Half_max_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = FocusSpread_UD;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = 0;
		#endif

	}
		
	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	Amo_timer_Stop(timer_63);
//	Amo_timer_Start(timer_63, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RH_Check);

	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_X = Hu_FrPsEVntSidePt_X_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = Hu_FrPsEVntSidePt_Y_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = Hu_FrPsEVntCtrPt_X_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = Hu_FrPsEVntCtrPt_Y_backup;

	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = Hu_Fr_Ps_Vent_Mode;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = Hu_Fr_Ps_Vent_Side_signal;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = Hu_Fr_Ps_Vent_Center_signal;
}

void RHD_lin_FrPsVent_Focus_Mode_task_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	recoveryFinishFlag_crlr = FALSE;
	recoveryFinishFlag_crud = FALSE;
	recoveryFinishFlag_srlr = FALSE;
	recoveryFinishFlag_srud = FALSE;
	
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);
	
	if(Hu_Fr_Dr_Vent_Mode == FACING_MODE) //focus mode
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Half_min_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = FocusSpread_UD_Rev;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = 0;
		#endif

	}
		
	if(Hu_Fr_Dr_Vent_Mode == FACING_MODE) //focus mode
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Half_max_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = FocusSpread_UD;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = 0;
		#endif
	}
		
	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	Amo_timer_Stop(timer_63);
//	Amo_timer_Start(timer_63, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RH_Check);

	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_X = Hu_FrPsEVntSidePt_X_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = Hu_FrPsEVntSidePt_Y_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = Hu_FrPsEVntCtrPt_X_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = Hu_FrPsEVntCtrPt_Y_backup;

	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = Hu_Fr_Ps_Vent_Mode;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = Hu_Fr_Ps_Vent_Side_signal;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = Hu_Fr_Ps_Vent_Center_signal;
}



void lin_FrPsVent_Spread_Mode_task_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	recoveryFinishFlag_crlr = FALSE;
	recoveryFinishFlag_crud = FALSE;
	recoveryFinishFlag_srlr = FALSE;
	recoveryFinishFlag_srud = FALSE;
	
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);

	if(Hu_Fr_Ps_Vent_Mode == AVOID_MODE) //spread mode
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Half_max_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = FocusSpread_UD_Rev;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
		#endif

	}
		
	if(Hu_Fr_Ps_Vent_Mode == AVOID_MODE) //spread mode
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Half_min_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = FocusSpread_UD;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
		#endif

	}
		
	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	Amo_timer_Stop(timer_63);
//	Amo_timer_Start(timer_63, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RH_Check);

	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_X = Hu_FrPsEVntSidePt_X_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = Hu_FrPsEVntSidePt_Y_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = Hu_FrPsEVntCtrPt_X_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = Hu_FrPsEVntCtrPt_Y_backup;

	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = Hu_Fr_Ps_Vent_Mode;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = Hu_Fr_Ps_Vent_Side_signal;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = Hu_Fr_Ps_Vent_Center_signal;
}


void RHD_lin_FrPsVent_Spread_Mode_task_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	recoveryFinishFlag_crlr = FALSE;
	recoveryFinishFlag_crud = FALSE;
	recoveryFinishFlag_srlr = FALSE;
	recoveryFinishFlag_srud = FALSE;
	
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);

	if(Hu_Fr_Dr_Vent_Mode == AVOID_MODE) //spread mode
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Half_max_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = FocusSpread_UD_Rev;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
		#endif

	}
		
	if(Hu_Fr_Dr_Vent_Mode == AVOID_MODE) //spread mode
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Half_min_LR;
		#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = FocusSpread_UD;
		#else
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
		#endif
	}
		
	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	Amo_timer_Stop(timer_63);
//	Amo_timer_Start(timer_63, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RH_Check);

	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_X = Hu_FrPsEVntSidePt_X_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = Hu_FrPsEVntSidePt_Y_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = Hu_FrPsEVntCtrPt_X_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = Hu_FrPsEVntCtrPt_Y_backup;

	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = Hu_Fr_Ps_Vent_Mode;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = Hu_Fr_Ps_Vent_Side_signal;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = Hu_Fr_Ps_Vent_Center_signal;
}


void lin_RrVentMode_task(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl)
{
	SetLIN_EVNT_RrSPEED(&LIN_RC_EVNT_MASTER_CMD, ACT_RPM_10);

	if(l_flg_tst_LI2_EVNT_Rear_Center_LH_LeftRight_TargetPosition_flag() || l_flg_tst_LI2_EVNT_Rear_Center_RH_LeftRight_TargetPosition_flag())
	{
		//Clear this flag...
	  l_flg_clr_LI2_EVNT_Rear_Center_LH_LeftRight_TargetPosition_flag();
		l_flg_clr_LI2_EVNT_Rear_Center_RH_LeftRight_TargetPosition_flag();
	
		//if((Hu_Rr_Dr_Vent_Mode == DEFAULT_MODE) && (Hu_Rr_Dr_Vent_signal == 0x02)) //fullclose
		if(Hu_Rr_Dr_Vent_signal == 0x02) //fullclose
		{
			lin_RrCycleMode_TimerStop();
		
			lin_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = FullClose_min;
			lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = FullClose_UD;
		}
		
		//if((Hu_Rr_Ps_Vent_Mode == DEFAULT_MODE) && (Hu_Rr_Ps_Vent_signal == 0x02)) //fullclose
		if(Hu_Rr_Ps_Vent_signal == 0x02) //fullclose
		{
			lin_RrCycleMode_TimerStop();
			
			lin_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = FullClose_max;
			lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = FullClose_UD;
		}

		if((Hu_Rr_Dr_Vent_Mode == AVOID_MODE) && (Hu_Rr_Dr_Vent_signal == 0x01)) //spread mode
		{
			lin_RrCycleMode_TimerStop();
			
			lin_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = Half_min_LR;
			#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = FocusSpread_UD;
		#else
			lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
		#endif
		}
		
		if((Hu_Rr_Ps_Vent_Mode == AVOID_MODE) && (Hu_Rr_Ps_Vent_signal == 0x01)) //spread mode
		{
			lin_RrCycleMode_TimerStop();
			
			lin_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = Half_max_LR;
			#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = FocusSpread_UD;
		#else
			lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
		#endif

		}

		if((Hu_Rr_Dr_Vent_Mode == FACING_MODE) && (Hu_Rr_Dr_Vent_signal == 0x01)) //focus mode
		{
			lin_RrCycleMode_TimerStop();
			
			lin_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = Half_max_LR;
			#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = FocusSpread_UD;
		#else
			lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = 0;
		#endif
		}
		
		if(Hu_Rr_Ps_Vent_Mode == FACING_MODE && Hu_Rr_Ps_Vent_signal == 0x01) //focus mode
		{
			lin_RrCycleMode_TimerStop();
			
			lin_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = Half_min_LR;
			#ifdef ACT_ANGLE_UPDATE_2
		lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = FocusSpread_UD;
		#else
			lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = 0;
		#endif

		}
		
		lin_Write_RrEVntConsPt_XY(&LIN_RC_EVNT_MASTER_CMD);

		Amo_timer_Stop(timer_66);
		Amo_timer_Start(timer_66, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RC_Check);

		Set_LinToCan_Disp_Rr_XY(&Can_Tx_Evnt_3);
		Set_LinToCan_Disp_RrDr_Mode(&Can_Tx_Evnt_4);
		Set_LinToCan_Disp_RrPs_Mode(&Can_Tx_Evnt_4);
	}
}


void SetLIN_EVNT_FrDrSPEED(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, uint8_t rpm)
{
	if(rpm == ACT_RPM_10) { lin_Ctrl->LDATA.EVNT_SPEED = ACT_RPM_10; }
	else if(rpm == ACT_RPM_3) { lin_Ctrl->LDATA.EVNT_SPEED = ACT_RPM_3; }
	else if(rpm == ACT_RPM_2) { lin_Ctrl->LDATA.EVNT_SPEED = ACT_RPM_2; }
	else if(rpm == ACT_RPM_1) { lin_Ctrl->LDATA.EVNT_SPEED = ACT_RPM_1; }
	
	l_u8_wr_LI0_EVNT_SPEED(lin_Ctrl->LDATA.EVNT_SPEED);
}


void SetLIN_EVNT_FrPsSPEED(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, uint8_t rpm)
{
	if(rpm == ACT_RPM_10) { lin_Ctrl->LDATA.EVNT_SPEED = ACT_RPM_10; }
	else if(rpm == ACT_RPM_3) { lin_Ctrl->LDATA.EVNT_SPEED = ACT_RPM_3; }
	else if(rpm == ACT_RPM_2) { lin_Ctrl->LDATA.EVNT_SPEED = ACT_RPM_2; }
	else if(rpm == ACT_RPM_1) { lin_Ctrl->LDATA.EVNT_SPEED = ACT_RPM_1; }
	
	l_u8_wr_LI1_EVNT_SPEED(lin_Ctrl->LDATA.EVNT_SPEED);
}

void SetLIN_EVNT_RrSPEED(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, uint8_t rpm)
{

	if(rpm == ACT_RPM_10) 
	{ 
		lin_Ctrl->LDATA.EVNT_REAR_LH_SPEED = ACT_RPM_10;
		lin_Ctrl->LDATA.EVNT_REAR_RH_SPEED = ACT_RPM_10;
	}
	if(rpm == ACT_RPM_3) 
	{ 
		lin_Ctrl->LDATA.EVNT_REAR_LH_SPEED = ACT_RPM_3;
		lin_Ctrl->LDATA.EVNT_REAR_RH_SPEED = ACT_RPM_3; 
	}

	l_u8_wr_LI2_EVNT_REAR_LH_SPEED(lin_Ctrl->LDATA.EVNT_REAR_LH_SPEED);
	l_u8_wr_LI2_EVNT_REAR_RH_SPEED(lin_Ctrl->LDATA.EVNT_REAR_RH_SPEED);
}


void CanData_Update_Check_FrDrPt(Atcu_20_data *canPtDataBuf)
{
	CanValue.FrDrPt.Hu_FrDrEVntSidePt_X_newVal = canPtDataBuf->data.L_ATCU_HU_FrDrEVntSidePt_X;
	CanValue.FrDrPt.Hu_FrDrEVntSidePt_Y_newVal = canPtDataBuf->data.L_ATCU_HU_FrDrEVntSidePt_Y;
	CanValue.FrDrPt.Hu_FrDrEVntCtrPt_X_newVal = canPtDataBuf->data.L_ATCU_HU_FrDrEVntCtrPt_X;
	CanValue.FrDrPt.Hu_FrDrEVntCtrPt_Y_newVal = canPtDataBuf->data.L_ATCU_HU_FrDrEVntCtrPt_Y;


	if((CanValue.FrDrPt.Hu_FrDrEVntSidePt_X_oldVal != CanValue.FrDrPt.Hu_FrDrEVntSidePt_X_newVal) || \
		(CanValue.FrDrPt.Hu_FrDrEVntSidePt_Y_oldVal != CanValue.FrDrPt.Hu_FrDrEVntSidePt_Y_newVal))
	{
		CurrentEVnt.selectEVnt = FrDrSidePt_XY;
		
		CanValue.FrDrPt.Hu_FrDrEVntSidePt_X_oldVal = CanValue.FrDrPt.Hu_FrDrEVntSidePt_X_newVal;
		CanValue.FrDrPt.Hu_FrDrEVntSidePt_Y_oldVal = CanValue.FrDrPt.Hu_FrDrEVntSidePt_Y_newVal;

	}

	if((CanValue.FrDrPt.Hu_FrDrEVntCtrPt_X_oldVal != CanValue.FrDrPt.Hu_FrDrEVntCtrPt_X_newVal) || \
		(CanValue.FrDrPt.Hu_FrDrEVntCtrPt_Y_oldVal != CanValue.FrDrPt.Hu_FrDrEVntCtrPt_Y_newVal))
	{
		CurrentEVnt.selectEVnt1 = FrDrCtrPt_XY;
		
		CanValue.FrDrPt.Hu_FrDrEVntCtrPt_X_oldVal = CanValue.FrDrPt.Hu_FrDrEVntCtrPt_X_newVal;
		CanValue.FrDrPt.Hu_FrDrEVntCtrPt_Y_oldVal = CanValue.FrDrPt.Hu_FrDrEVntCtrPt_Y_newVal;
	}
}


void CanData_Update_Check_FrPsPt(Atcu_21_data *canPtDataBuf)
{
	CanValue.FrPsPt.Hu_FrPsEVntSidePt_X_newVal = canPtDataBuf->data.L_ATCU_HU_FrPsEVntSidePt_X;
	CanValue.FrPsPt.Hu_FrPsEVntSidePt_Y_newVal = canPtDataBuf->data.L_ATCU_HU_FrPsEVntSidePt_Y;
	CanValue.FrPsPt.Hu_FrPsEVntCtrPt_X_newVal = canPtDataBuf->data.L_ATCU_HU_FrPsEVntCtrPt_X;
	CanValue.FrPsPt.Hu_FrPsEVntCtrPt_Y_newVal = canPtDataBuf->data.L_ATCU_HU_FrPsEVntCtrPt_Y;


	if((CanValue.FrPsPt.Hu_FrPsEVntSidePt_X_oldVal != CanValue.FrPsPt.Hu_FrPsEVntSidePt_X_newVal) || \
		(CanValue.FrPsPt.Hu_FrPsEVntSidePt_Y_oldVal != CanValue.FrPsPt.Hu_FrPsEVntSidePt_Y_newVal))
	{
		CurrentEVnt.selectEVnt = FrPsSidePt_XY;
		
		CanValue.FrPsPt.Hu_FrPsEVntSidePt_X_oldVal = CanValue.FrPsPt.Hu_FrPsEVntSidePt_X_newVal;
		CanValue.FrPsPt.Hu_FrPsEVntSidePt_Y_oldVal = CanValue.FrPsPt.Hu_FrPsEVntSidePt_Y_newVal;

	}

	if((CanValue.FrPsPt.Hu_FrPsEVntCtrPt_X_oldVal != CanValue.FrPsPt.Hu_FrPsEVntCtrPt_X_newVal) || \
		(CanValue.FrPsPt.Hu_FrPsEVntCtrPt_Y_oldVal != CanValue.FrPsPt.Hu_FrPsEVntCtrPt_Y_newVal))
	{
		CurrentEVnt.selectEVnt1 = FrPsCtrPt_XY;
		
		CanValue.FrPsPt.Hu_FrPsEVntCtrPt_X_oldVal = CanValue.FrPsPt.Hu_FrPsEVntCtrPt_X_newVal;
		CanValue.FrPsPt.Hu_FrPsEVntCtrPt_Y_oldVal = CanValue.FrPsPt.Hu_FrPsEVntCtrPt_Y_newVal;
	}
}


void CanData_Update_Check_RrPt(Atcu_22_data *canPtDataBuf)
{
	CanValue.RrPt.Hu_RrDrEVntConsPt_X_newVal = canPtDataBuf->data.L_ATCU_HU_RrDrEVntConsPt_X;
	CanValue.RrPt.Hu_RrDrEVntConsPt_Y_newVal = canPtDataBuf->data.L_ATCU_HU_RrDrEVntConsPt_Y;
	CanValue.RrPt.Hu_RrPsEVntConsPt_X_newVal = canPtDataBuf->data.L_ATCU_HU_RrPsEVntConsPt_X;
	CanValue.RrPt.Hu_RrPsEVntConsPt_Y_newVal = canPtDataBuf->data.L_ATCU_HU_RrPsEVntConsPt_Y;


	if((CanValue.RrPt.Hu_RrDrEVntConsPt_X_oldVal != CanValue.RrPt.Hu_RrDrEVntConsPt_X_newVal) || \
		(CanValue.RrPt.Hu_RrDrEVntConsPt_Y_oldVal != CanValue.RrPt.Hu_RrDrEVntConsPt_Y_newVal))
	{
		CurrentEVnt.selectEVnt = RrDrCons_XY;
		
		CanValue.RrPt.Hu_RrDrEVntConsPt_X_oldVal = CanValue.RrPt.Hu_RrDrEVntConsPt_X_newVal;
		CanValue.RrPt.Hu_RrDrEVntConsPt_Y_oldVal = CanValue.RrPt.Hu_RrDrEVntConsPt_Y_newVal;

	}

	if((CanValue.RrPt.Hu_RrPsEVntConsPt_X_oldVal != CanValue.RrPt.Hu_RrPsEVntConsPt_X_newVal) || \
		(CanValue.RrPt.Hu_RrPsEVntConsPt_Y_oldVal != CanValue.RrPt.Hu_RrPsEVntConsPt_Y_newVal))
	{
		CurrentEVnt.selectEVnt1 = RrPsCons_XY;
		
		CanValue.RrPt.Hu_RrPsEVntConsPt_X_oldVal = CanValue.RrPt.Hu_RrPsEVntConsPt_X_newVal;
		CanValue.RrPt.Hu_RrPsEVntConsPt_Y_oldVal = CanValue.RrPt.Hu_RrPsEVntConsPt_Y_newVal;
	}
}



void FrDrLinData_Parsing(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
	if(Hu_Fr_dr_Vent_Side_signal_backup != 0x02)
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		if((Hu_FrDrEVntSidePt_X_invalid != 2047) || (Hu_FrDrEVntSidePt_Y_invalid != 2047))
		{

//			if(evntModeflags.FR_SL_Mode_Flag == TRUE) 
//			{ 
				lin_Pt->FrDrPt.prevPt_FR_SLLR = lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
				lin_Pt->FrDrPt.prevPt_FR_SLUD = lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;

//				evntModeflags.FR_SL_Mode_Flag = FALSE; 
//			}
			
			lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrDrEVntSidePt_X(&Can_data_20_Info, &Can_data_23_Info);
	  	lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = SET_L_ATCU_HU_FrDrEVntSidePt_Y(&Can_data_20_Info, &Can_data_23_Info);

			lin_Pt->FrDrPt.currentPt_FR_SLLR = lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
			lin_Pt->FrDrPt.currentPt_FR_SLUD = lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;
			
			Fr_Dr_side_leftright_target_backup = lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
			Fr_Dr_side_updown_target_backup = lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;		
		}
		
		if(Hu_Fr_Dr_Vent_Mode == FREE_MODE && Hu_Fr_Ps_Vent_Mode == SWING_MODE)
		{
			lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
		}	
	
		else
		{
	
		}
	}
	
	if(Hu_Fr_dr_Vent_Center_signal_backup != 0x02)
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		if((Hu_FrDrEVntCtrPt_X_invalid != 2047) || (Hu_FrDrEVntCtrPt_Y_invalid != 2047))
		{

//			if(evntModeflags.FR_CL_Mode_Flag == TRUE) 
//			{ 
				lin_Pt->FrDrPt.prevPt_FR_CLLR = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
				lin_Pt->FrDrPt.prevPt_FR_CLUD = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
				
//				evntModeflags.FR_CL_Mode_Flag = FALSE; 
//			}

			lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrDrEVntCtrPt_X(&Can_data_20_Info, &Can_data_23_Info);
			lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = SET_L_ATCU_HU_FrDrEVntCtrPt_Y(&Can_data_20_Info, &Can_data_23_Info);	

			lin_Pt->FrDrPt.currentPt_FR_CLLR = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
			lin_Pt->FrDrPt.currentPt_FR_CLUD = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
			
			Fr_Dr_ctr_leftright_target_backup = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
			Fr_Dr_ctr_updown_target_backup = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;		
		}
		
		if(Hu_Fr_Dr_Vent_Mode == FREE_MODE && Hu_Fr_Ps_Vent_Mode == SWING_MODE)
		{
			lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
		}	
	
		else
		{
	
		}
	}
}

void RHD_FrDrLinData_Parsing(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	if(Hu_Fr_Ps_Vent_Side_signal_backup != 0x02)
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		if((Hu_FrPsEVntSidePt_X_invalid != 2047) || (Hu_FrPsEVntSidePt_Y_invalid != 2047))
		{
			LIN_EVNT_PT.FrDrPt.prevPt_FR_SLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrDrPt.prevPt_FR_SLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;
			
			lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrDrEVntSidePt_X(&Can_data_20_Info, &Can_data_23_Info);
	  	lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = SET_L_ATCU_HU_FrDrEVntSidePt_Y(&Can_data_20_Info, &Can_data_23_Info);

			LIN_EVNT_PT.FrDrPt.currentPt_FR_SLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrDrPt.currentPt_FR_SLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;
		}
		
		Fr_Dr_side_leftright_target_backup = lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
		Fr_Dr_side_updown_target_backup = lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;		
	}
	
	if(Hu_Fr_Ps_Vent_Center_signal_backup != 0x02)
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		if((Hu_FrPsEVntCtrPt_X_invalid != 2047) || (Hu_FrPsEVntCtrPt_Y_invalid != 2047))
		{
			LIN_EVNT_PT.FrDrPt.prevPt_FR_CLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrDrPt.prevPt_FR_CLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
			
			lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrDrEVntCtrPt_X(&Can_data_20_Info, &Can_data_23_Info);
			lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = SET_L_ATCU_HU_FrDrEVntCtrPt_Y(&Can_data_20_Info, &Can_data_23_Info);	

			LIN_EVNT_PT.FrDrPt.currentPt_FR_CLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrDrPt.currentPt_FR_CLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
		}
		Fr_Dr_ctr_leftright_target_backup = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		Fr_Dr_ctr_updown_target_backup = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;		
	}
}


#ifdef KEY_ENABLE
void FrDrLinData_Parsing_Key(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrDrEVntSidePt_X_Key(&Can_data_20_Info_Key, &Can_data_23_Info);
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = SET_L_ATCU_HU_FrDrEVntSidePt_Y_Key(&Can_data_20_Info_Key, &Can_data_23_Info);

		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrDrEVntCtrPt_X_Key(&Can_data_20_Info_Key, &Can_data_23_Info);
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = SET_L_ATCU_HU_FrDrEVntCtrPt_Y_Key(&Can_data_20_Info_Key, &Can_data_23_Info);

}
#endif

void FrDrLinData_Parsing_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{

	if(Hu_Fr_dr_Vent_Side_signal_backup != 0x02)
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Fr_Dr_side_leftright_target_backup;
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Fr_Dr_side_updown_target_backup;
	}

	if(Hu_Fr_dr_Vent_Center_signal_backup != 0x02)
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Fr_Dr_ctr_leftright_target_backup;
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Fr_Dr_ctr_updown_target_backup;
	}

}

void RHD_FrDrLinData_Parsing_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{

	if(Hu_Fr_Ps_Vent_Side_signal_backup != 0x02)
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Fr_Dr_side_leftright_target_backup;
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Fr_Dr_side_updown_target_backup;
	}

	if(Hu_Fr_Ps_Vent_Center_signal_backup != 0x02)
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Fr_Dr_ctr_leftright_target_backup;
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Fr_Dr_ctr_updown_target_backup;
	}

}


void FrPsLinData_Parsing(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	if(Hu_Fr_Ps_Vent_Side_signal_backup != 0x02)
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		if((Hu_FrPsEVntSidePt_X_invalid != 2047) || (Hu_FrPsEVntSidePt_Y_invalid != 2047))
		{
			LIN_EVNT_PT.FrPsPt.prevPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrPsPt.prevPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
			
			lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrPsEVntSidePt_X(&Can_data_21_Info, &Can_data_24_Info);
			lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = SET_L_ATCU_HU_FrPsEVntSidePt_Y(&Can_data_21_Info, &Can_data_24_Info);	

			LIN_EVNT_PT.FrPsPt.currentPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrPsPt.currentPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
		}
		if(Hu_Fr_Ps_Vent_Mode == FREE_MODE && Hu_Fr_Dr_Vent_Mode == SWING_MODE)
		{
			lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
		}	
	
		else
		{
	
		}
		Fr_Ps_side_leftright_target_backup = lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
		Fr_Ps_side_updown_target_backup = lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;		
	}

	if(Hu_Fr_Ps_Vent_Center_signal_backup != 0x02)
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		if((Hu_FrPsEVntCtrPt_X_invalid != 2047) || (Hu_FrPsEVntCtrPt_Y_invalid != 2047))
		{
			LIN_EVNT_PT.FrPsPt.prevPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrPsPt.prevPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
			
			lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrPsEVntCtrPt_X(&Can_data_21_Info, &Can_data_24_Info);
			lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = SET_L_ATCU_HU_FrPsEVntCtrPt_Y(&Can_data_21_Info, &Can_data_24_Info);

			LIN_EVNT_PT.FrPsPt.currentPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrPsPt.currentPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
		}
		if(Hu_Fr_Ps_Vent_Mode == FREE_MODE && Hu_Fr_Dr_Vent_Mode == SWING_MODE)
		{
			lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
		}	
	
		else
		{
	
		}
		Fr_Ps_ctr_leftright_target_backup = lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
		Fr_Ps_ctr_updown_target_backup = lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;		
	}
}


void RHD_FrPsLinData_Parsing(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	if(Hu_Fr_dr_Vent_Side_signal_backup != 0x02)
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		if((Hu_FrDrEVntSidePt_X_invalid != 2047) || (Hu_FrDrEVntSidePt_Y_invalid != 2047))
		{
			LIN_EVNT_PT.FrPsPt.prevPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrPsPt.prevPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
			
			lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrPsEVntSidePt_X(&Can_data_21_Info, &Can_data_24_Info);
			lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = SET_L_ATCU_HU_FrPsEVntSidePt_Y(&Can_data_21_Info, &Can_data_24_Info);	
			
			LIN_EVNT_PT.FrPsPt.currentPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrPsPt.currentPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
		}
		Fr_Ps_side_leftright_target_backup = lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
		Fr_Ps_side_updown_target_backup = lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;		
	}

	if(Hu_Fr_dr_Vent_Center_signal_backup != 0x02)
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		if((Hu_FrDrEVntCtrPt_X_invalid != 2047) || (Hu_FrDrEVntCtrPt_Y_invalid != 2047))
		{
			LIN_EVNT_PT.FrPsPt.prevPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrPsPt.prevPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
		
			lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrPsEVntCtrPt_X(&Can_data_21_Info, &Can_data_24_Info);
			lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = SET_L_ATCU_HU_FrPsEVntCtrPt_Y(&Can_data_21_Info, &Can_data_24_Info);
			
			LIN_EVNT_PT.FrPsPt.currentPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrPsPt.currentPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
		}
		Fr_Ps_ctr_leftright_target_backup = lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
		Fr_Ps_ctr_updown_target_backup = lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;		
	}
}


#ifdef KEY_ENABLE
void FrPsLinData_Parsing_Key(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{

		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrPsEVntSidePt_X_Key(&Can_data_21_Info_Key, &Can_data_24_Info);
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = SET_L_ATCU_HU_FrPsEVntSidePt_Y_Key(&Can_data_21_Info_Key, &Can_data_24_Info);

		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrPsEVntCtrPt_X_Key(&Can_data_21_Info_Key, &Can_data_24_Info);
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = SET_L_ATCU_HU_FrPsEVntCtrPt_Y_Key(&Can_data_21_Info_Key, &Can_data_24_Info);
}
#endif

void FrPsLinData_Parsing_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{

	if(Hu_Fr_Ps_Vent_Side_signal_backup != 0x02)
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Fr_Ps_side_leftright_target_backup;
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Fr_Ps_side_updown_target_backup;
	}
	
	if(Hu_Fr_Ps_Vent_Center_signal_backup != 0x02)
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Fr_Ps_ctr_leftright_target_backup;
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Fr_Ps_ctr_updown_target_backup;
	}
}

void RHD_FrPsLinData_Parsing_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{

	if(Hu_Fr_dr_Vent_Side_signal_backup != 0x02)
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Fr_Ps_side_leftright_target_backup;
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Fr_Ps_side_updown_target_backup;
	}
	
	if(Hu_Fr_dr_Vent_Center_signal_backup != 0x02)
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Fr_Ps_ctr_leftright_target_backup;
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Fr_Ps_ctr_updown_target_backup;
	}
}


void RrLinData_Parsing(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl)
{
		lin_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = SET_L_ATCU_HU_RrDrEVntConsPt_X(&Can_data_22_Info, &Can_data_25_Info);
		lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = SET_L_ATCU_HU_RrDrEVntConsPt_Y(&Can_data_22_Info, &Can_data_25_Info);

		lin_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = SET_L_ATCU_HU_RrPsEVntConsPt_X(&Can_data_22_Info, &Can_data_25_Info);
		lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = SET_L_ATCU_HU_RrPsEVntConsPt_Y(&Can_data_22_Info, &Can_data_25_Info);
	
}


uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_X(Atcu_20_data *canPtDataBuf, Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[FrDrSidePt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_X * canFactor));
	CanLinValue[FrDrSidePt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrDrEVntSidePt_X;

	CanLinValue[FrDrSidePt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrDr_SidePt_X(Conv_Can_Pt_X);

	ret_value = CanLinValue[FrDrSidePt_X].Pt_X.LinData_Pt_X;
		
	return ret_value;
}

uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_X(Atcu_20_data *canPtDataBuf, Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[FrDrCtrPt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_X * canFactor));
	CanLinValue[FrDrCtrPt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrDrEVntCtrPt_X;

	CanLinValue[FrDrCtrPt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrDr_CtrPt_X(Conv_Can_Pt_X);
	
	ret_value = CanLinValue[FrDrCtrPt_X].Pt_X.LinData_Pt_X;
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_Y(Atcu_20_data *canPtDataBuf, Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[FrDrSidePt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_Y * canFactor));
	CanLinValue[FrDrSidePt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrDrEVntSidePt_Y;

	CanLinValue[FrDrSidePt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrDr_SidePt_Y(Conv_Can_Pt_Y);

	ret_value = CanLinValue[FrDrSidePt_Y].Pt_Y.LinData_Pt_Y;
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_Y(Atcu_20_data *canPtDataBuf, Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	
	CanLinValue[FrDrCtrPt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_Y * canFactor));
	CanLinValue[FrDrCtrPt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrDrEVntCtrPt_Y;

	CanLinValue[FrDrCtrPt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrDr_CtrPt_Y(Conv_Can_Pt_Y);

	ret_value = 4096 - CanLinValue[FrDrCtrPt_Y].Pt_Y.LinData_Pt_Y;

	return ret_value;
}


uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_X(Atcu_21_data *canPtDataBuf, Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[FrPsSidePt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_X * canFactor));
	CanLinValue[FrPsSidePt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrPsEVntSidePt_X;

	CanLinValue[FrPsSidePt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrPs_SidePt_X(Conv_Can_Pt_X);

	ret_value = CanLinValue[FrPsSidePt_X].Pt_X.LinData_Pt_X;
		
	return ret_value;
}

uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_X(Atcu_21_data *canPtDataBuf, Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[FrPsCtrPt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_X * canFactor));
	CanLinValue[FrPsCtrPt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrPsEVntCtrPt_X;

	CanLinValue[FrPsCtrPt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrPs_CtrPt_X(Conv_Can_Pt_X);
	
	ret_value = CanLinValue[FrPsCtrPt_X].Pt_X.LinData_Pt_X;
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_Y(Atcu_21_data *canPtDataBuf, Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[FrPsSidePt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_Y * canFactor));
	CanLinValue[FrPsSidePt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrPsEVntSidePt_Y;

	CanLinValue[FrPsSidePt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrPs_SidePt_Y(Conv_Can_Pt_Y);

	ret_value = 4096 - CanLinValue[FrPsSidePt_Y].Pt_Y.LinData_Pt_Y;
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_Y(Atcu_21_data *canPtDataBuf, Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	
	CanLinValue[FrPsCtrPt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_Y * canFactor));
	CanLinValue[FrPsCtrPt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrPsEVntCtrPt_Y;

	CanLinValue[FrPsCtrPt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrPs_CtrPt_Y(Conv_Can_Pt_Y);

	ret_value = CanLinValue[FrPsCtrPt_Y].Pt_Y.LinData_Pt_Y;
		
	return ret_value;
}

////////////////////////////////////////////////////////
uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_X_Key(Atcu_20_data_Key *canPtDataBuf, Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	CanLinValue[FrDrSidePt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_X * canFactor));
	CanLinValue[FrDrSidePt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrDrEVntSidePt_X_Key;
	CanLinValue[FrDrSidePt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrDr_SidePt_X(Conv_Can_Pt_X);
	ret_value = CanLinValue[FrDrSidePt_X].Pt_X.LinData_Pt_X;
	return ret_value;
}
uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_X_Key(Atcu_20_data_Key *canPtDataBuf, Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	CanLinValue[FrDrCtrPt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_X * canFactor));
	CanLinValue[FrDrCtrPt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrDrEVntCtrPt_X_Key;
	CanLinValue[FrDrCtrPt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrDr_CtrPt_X(Conv_Can_Pt_X);
	ret_value = CanLinValue[FrDrCtrPt_X].Pt_X.LinData_Pt_X;
	return ret_value;
}
uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_Y_Key(Atcu_20_data_Key *canPtDataBuf, Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	CanLinValue[FrDrSidePt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_Y * canFactor));
	CanLinValue[FrDrSidePt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrDrEVntSidePt_Y_Key;
	CanLinValue[FrDrSidePt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrDr_SidePt_Y(Conv_Can_Pt_Y);
	ret_value = CanLinValue[FrDrSidePt_Y].Pt_Y.LinData_Pt_Y;
	return ret_value;
}
uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_Y_Key(Atcu_20_data_Key *canPtDataBuf, Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	CanLinValue[FrDrCtrPt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_Y * canFactor));
	CanLinValue[FrDrCtrPt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrDrEVntCtrPt_Y_Key;
	CanLinValue[FrDrCtrPt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrDr_CtrPt_Y(Conv_Can_Pt_Y);
	ret_value = CanLinValue[FrDrCtrPt_Y].Pt_Y.LinData_Pt_Y;
	return ret_value;
}
uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_X_Key(Atcu_21_data_Key *canPtDataBuf, Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	CanLinValue[FrPsSidePt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_X * canFactor));
	CanLinValue[FrPsSidePt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrPsEVntSidePt_X_Key;
	CanLinValue[FrPsSidePt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrPs_SidePt_X(Conv_Can_Pt_X);
	ret_value = CanLinValue[FrPsSidePt_X].Pt_X.LinData_Pt_X;
	return ret_value;
}
uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_X_Key(Atcu_21_data_Key *canPtDataBuf, Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	CanLinValue[FrPsCtrPt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_X * canFactor));
	CanLinValue[FrPsCtrPt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrPsEVntCtrPt_X_Key;
	CanLinValue[FrPsCtrPt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrPs_CtrPt_X(Conv_Can_Pt_X);
	ret_value = CanLinValue[FrPsCtrPt_X].Pt_X.LinData_Pt_X;
	return ret_value;
}
uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_Y_Key(Atcu_21_data_Key *canPtDataBuf, Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	CanLinValue[FrPsSidePt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_Y * canFactor));
	CanLinValue[FrPsSidePt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrPsEVntSidePt_Y_Key;
	CanLinValue[FrPsSidePt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrPs_SidePt_Y(Conv_Can_Pt_Y);
	ret_value = CanLinValue[FrPsSidePt_Y].Pt_Y.LinData_Pt_Y;
	return ret_value;
}
uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_Y_Key(Atcu_21_data_Key *canPtDataBuf, Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	CanLinValue[FrPsCtrPt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_Y * canFactor));
	CanLinValue[FrPsCtrPt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrPsEVntCtrPt_Y_Key;
	CanLinValue[FrPsCtrPt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrPs_CtrPt_Y(Conv_Can_Pt_Y);
	ret_value = CanLinValue[FrPsCtrPt_Y].Pt_Y.LinData_Pt_Y;
	return ret_value;
}
///////////////////////////////////////////////



uint16_t SET_L_ATCU_HU_RrDrEVntConsPt_X(Atcu_22_data *canPtDataBuf, Atcu_25_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[RrDrCons_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryUp_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryLo_X * canFactor));
	CanLinValue[RrDrCons_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_RrDrEVntConsPt_X;

	CanLinValue[RrDrCons_X].Pt_X.LinData_Pt_X = Cal_CanToLin_RrDr_ConsPt_X(Conv_Can_Pt_X);

	ret_value = CanLinValue[RrDrCons_X].Pt_X.LinData_Pt_X;
		
	return ret_value;
}

uint16_t SET_L_ATCU_HU_RrPsEVntConsPt_X(Atcu_22_data *canPtDataBuf, Atcu_25_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[RrPsCons_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryUP_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryLo_X * canFactor));
	CanLinValue[RrPsCons_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_RrPsEVntConsPt_X;

	CanLinValue[RrPsCons_X].Pt_X.LinData_Pt_X = Cal_CanToLin_RrPs_ConsPt_X(Conv_Can_Pt_X);
	
	ret_value = CanLinValue[RrPsCons_X].Pt_X.LinData_Pt_X;
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_RrDrEVntConsPt_Y(Atcu_22_data *canPtDataBuf, Atcu_25_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[RrDrCons_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryUp_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryLo_Y * canFactor));
	CanLinValue[RrDrCons_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_RrDrEVntConsPt_Y;

	CanLinValue[RrDrCons_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_RrDr_ConsPt_Y(Conv_Can_Pt_Y);

	ret_value = CanLinValue[RrDrCons_Y].Pt_Y.LinData_Pt_Y;
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_RrPsEVntConsPt_Y(Atcu_22_data *canPtDataBuf, Atcu_25_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	
	CanLinValue[RrPsCons_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryUP_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryLo_Y * canFactor));
	CanLinValue[RrPsCons_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_RrPsEVntConsPt_Y;

	CanLinValue[RrPsCons_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_RrPs_ConsPt_Y(Conv_Can_Pt_Y);

	ret_value = CanLinValue[RrPsCons_Y].Pt_Y.LinData_Pt_Y;
		
	return ret_value;
}


uint16_t CONV_CanToLin_TargetPos_void(void)
{
	uint16_t ret_value = FALSE;

	int16_t LinPosRange = 600;
	int16_t CanPosRange = 0, CanPosition = 0, CanPosCenter, CanData = 0, linFactor = 0, LinData = 0;

	CanPosRange = (Hu_Fr_Dr_Vent_Side_Bdry_Up_X) - (Hu_Fr_Dr_Vent_Side_Bdry_Lo_X);

	CanPosition = Hu_FrDrEVntSidePt_X;

	CanPosCenter = CanPosRange>>1;

	CanData = CanPosition - CanPosCenter;
	linFactor = (LinPosRange / CanPosRange);
	LinData = (CanData * linFactor) & 0x7FF;

	return ret_value = LinData;
}



void Set_LinToCan_Disp_FrDr_XY(Lvnt_1_data *L1_DataBuf)
{
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntSidePtDisp_X = SET_L_EVNT_HU_FrDrEVntSidePt_X(&LIN_LH_EVNT_MASTER_CMD, &Can_data_23_Info);
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = SET_L_EVNT_HU_FrDrEVntSidePt_Y(&LIN_LH_EVNT_MASTER_CMD, &Can_data_23_Info);
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = SET_L_EVNT_HU_FrDrEVntCtrPt_X(&LIN_LH_EVNT_MASTER_CMD, &Can_data_23_Info);
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = SET_L_EVNT_HU_FrDrEVntCtrPt_Y(&LIN_LH_EVNT_MASTER_CMD, &Can_data_23_Info);

	Hu_FrDrEVntSidePt_X_backup = L1_DataBuf->data.L_EVNT_HU_FrDrEvntSidePtDisp_X;
	Hu_FrDrEVntSidePt_Y_backup = L1_DataBuf->data.L_EVNT_HU_FrDrEvntSidePtDisp_Y;
	Hu_FrDrEVntCtrPt_X_backup = L1_DataBuf->data.L_EVNT_HU_FrDrEvntCtrPtDisp_X;
	Hu_FrDrEVntCtrPt_Y_backup = L1_DataBuf->data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y;
}

uint16_t SET_L_EVNT_HU_FrDrEVntSidePt_X(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Atcu_23_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_FrDrEVntSidePt_Y(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Atcu_23_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_FrDrEVntCtrPt_X(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Atcu_23_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_FrDrEVntCtrPt_Y(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Atcu_23_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Reverse_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

void Set_LinToCan_Disp_FrPs_XY(Lvnt_2_data *L2_DataBuf)
{
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntSidePtDisp_X = SET_L_EVNT_HU_FrPsEVntSidePt_X(&LIN_RH_EVNT_MASTER_CMD, &Can_data_24_Info);
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = SET_L_EVNT_HU_FrPsEVntSidePt_Y(&LIN_RH_EVNT_MASTER_CMD, &Can_data_24_Info);
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = SET_L_EVNT_HU_FrPsEVntCtrPt_X(&LIN_RH_EVNT_MASTER_CMD, &Can_data_24_Info);
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = SET_L_EVNT_HU_FrPsEVntCtrPt_Y(&LIN_RH_EVNT_MASTER_CMD, &Can_data_24_Info);

	Hu_FrPsEVntSidePt_X_backup = L2_DataBuf->data.L_EVNT_HU_FrPsEvntSidePtDisp_X;
	Hu_FrPsEVntSidePt_Y_backup = L2_DataBuf->data.L_EVNT_HU_FrPsEvntSidePtDisp_Y;
	Hu_FrPsEVntCtrPt_X_backup = L2_DataBuf->data.L_EVNT_HU_FrPsEvntCtrPtDisp_X;
	Hu_FrPsEVntCtrPt_Y_backup = L2_DataBuf->data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y;
}

uint16_t SET_L_EVNT_HU_FrPsEVntSidePt_X(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Atcu_24_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_FrPsEVntSidePt_Y(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Atcu_24_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Reverse_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_FrPsEVntCtrPt_X(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Atcu_24_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_FrPsEVntCtrPt_Y(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Atcu_24_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}


void Set_LinToCan_Disp_Rr_XY(Lvnt_3_data *L3_DataBuf)
{
	L3_DataBuf->data.L_EVNT_HU_RrDrEvntConsPtDisp_X = SET_L_EVNT_HU_RrDrEVntSidePt_X(&LIN_RC_EVNT_MASTER_CMD, &Can_data_25_Info);
	L3_DataBuf->data.L_EVNT_HU_RrDrEvntConsPtDisp_Y = SET_L_EVNT_HU_RrDrEVntSidePt_Y(&LIN_RC_EVNT_MASTER_CMD, &Can_data_25_Info);
	L3_DataBuf->data.L_EVNT_HU_RrPsEvntConsPtDisp_X = SET_L_EVNT_HU_RrPsEVntCtrPt_X(&LIN_RC_EVNT_MASTER_CMD, &Can_data_25_Info);
	L3_DataBuf->data.L_EVNT_HU_RrPsEvntConsPtDisp_Y = SET_L_EVNT_HU_RrPsEVntCtrPt_Y(&LIN_RC_EVNT_MASTER_CMD, &Can_data_25_Info);
}

uint16_t SET_L_EVNT_HU_RrDrEVntSidePt_X(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, Atcu_25_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryUp_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_RrDrEVntSidePt_Y(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, Atcu_25_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryUp_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_RrPsEVntCtrPt_X(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, Atcu_25_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryUP_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_RrPsEVntCtrPt_Y(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, Atcu_25_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryUP_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}


void Set_LinToCan_Disp_FrDr_Mode(Lvnt_4_data *L4_DataBuf)
{
	L4_DataBuf->data.L_EVNT_FrDrEvntModeSet = Hu_Fr_Dr_Vent_Mode;
	L4_DataBuf->data.L_EVNT_FrDrEvntSideOpnCls = Hu_Fr_dr_Vent_Side_signal;
	L4_DataBuf->data.L_EVNT_FrDrEvntCtrOpnCls = Hu_Fr_dr_Vent_Center_signal;
}

void Set_LinToCan_Disp_FrPs_Mode(Lvnt_4_data *L4_DataBuf)
{
	L4_DataBuf->data.L_EVNT_FrPsEvntModeSet = Hu_Fr_Ps_Vent_Mode;
	L4_DataBuf->data.L_EVNT_FrPsEvntSideOpnCls = Hu_Fr_Ps_Vent_Side_signal;
	L4_DataBuf->data.L_EVNT_FrPsEvntCtrOpnCls = Hu_Fr_Ps_Vent_Center_signal;
}

void Set_LinToCan_Disp_RrDr_Mode(Lvnt_4_data *L4_DataBuf)
{
	L4_DataBuf->data.L_EVNT_RrDrEvntModeSet = Hu_Rr_Dr_Vent_Mode;
	L4_DataBuf->data.L_EVNT_RrDrEvntConsOpnCls = Hu_Rr_Dr_Vent_signal;
}

void Set_LinToCan_Disp_RrPs_Mode(Lvnt_4_data *L4_DataBuf)
{
	L4_DataBuf->data.L_EVNT_RrPsEvntModeSet = Hu_Rr_Ps_Vent_Mode;
	L4_DataBuf->data.L_EVNT_RrPsEvntConsOpnCls = Hu_Rr_Ps_Vent_signal;
}


