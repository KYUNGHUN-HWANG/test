#define Amo_LIN_RECOVERY_C_

#include "Cpu.h"
#include "Amo_main.h"
//#include "flexcan_driver.h"
//#include "Amo_STATE_Event.h"
#include "Amo_Timer.h"
//#include "Amo_Calculate.h"
//#include "Amo_CAN_Parsing.h"

uint8_t FR_SLLR_FailSafe_Flag = FALSE;
uint8_t FR_SLUD_FailSafe_Flag = FALSE;
uint8_t FR_CLLR_FailSafe_Flag = FALSE;
uint8_t FR_CLUD_FailSafe_Flag = FALSE;

uint8_t FR_CRLR_FailSafe_Flag = FALSE;
uint8_t FR_CRUD_FailSafe_Flag = FALSE;
uint8_t FR_SRLR_FailSafe_Flag = FALSE;
uint8_t FR_SRUD_FailSafe_Flag = FALSE;

uint8_t RR_CLLR_FailSafe_Flag = FALSE;
uint8_t RR_CLUD_FailSafe_Flag = FALSE;
uint8_t RR_CRLR_FailSafe_Flag = FALSE;
uint8_t RR_CRUD_FailSafe_Flag = FALSE;


uint8_t recoveryCount = 0;
uint8_t hardStopCount = 0;
uint8_t specialCmdCount_cllr = 0;
uint8_t specialCmdCount_clud = 0;
uint8_t specialCmdCount_sllr = 0;
uint8_t specialCmdCount_slud = 0;
uint8_t specialCmdCount_crlr = 0;
uint8_t specialCmdCount_crud = 0;
uint8_t specialCmdCount_srlr = 0;
uint8_t specialCmdCount_srud = 0;

uint8_t steplossCount_cllr = 0;
uint8_t steplossCount_clud = 0;
uint8_t steplossCount_sllr = 0;
uint8_t steplossCount_slud = 0;
uint8_t steplossCount_crlr = 0;
uint8_t steplossCount_crud = 0;
uint8_t steplossCount_srlr = 0;
uint8_t steplossCount_srud = 0;

uint8_t steplossCount_1_cllr = 0;
uint8_t steplossCount_1_clud = 0;
uint8_t steplossCount_1_sllr = 0;
uint8_t steplossCount_1_slud = 0;
uint8_t steplossCount_1_crlr = 0;
uint8_t steplossCount_1_crud = 0;
uint8_t steplossCount_1_srlr = 0;
uint8_t steplossCount_1_srud = 0;

uint8_t steplossCount_2_cllr = 0;
uint8_t steplossCount_2_clud = 0;
uint8_t steplossCount_2_sllr = 0;
uint8_t steplossCount_2_slud = 0;
uint8_t steplossCount_2_crlr = 0;
uint8_t steplossCount_2_crud = 0;
uint8_t steplossCount_2_srlr = 0;
uint8_t steplossCount_2_srud = 0;

uint8_t steplossCount_3_cllr = 0;
uint8_t steplossCount_3_clud = 0;
uint8_t steplossCount_3_sllr = 0;
uint8_t steplossCount_3_slud = 0;
uint8_t steplossCount_3_crlr = 0;
uint8_t steplossCount_3_crud = 0;
uint8_t steplossCount_3_srlr = 0;
uint8_t steplossCount_3_srud = 0;

uint8_t steplossCount_4_cllr = 0;
uint8_t steplossCount_4_clud = 0;
uint8_t steplossCount_4_sllr = 0;
uint8_t steplossCount_4_slud = 0;
uint8_t steplossCount_4_crlr = 0;
uint8_t steplossCount_4_crud = 0;
uint8_t steplossCount_4_srlr = 0;
uint8_t steplossCount_4_srud = 0;

uint8_t steplossCount_5_cllr = 0;
uint8_t steplossCount_5_clud = 0;
uint8_t steplossCount_5_sllr = 0;
uint8_t steplossCount_5_slud = 0;
uint8_t steplossCount_5_crlr = 0;
uint8_t steplossCount_5_crud = 0;
uint8_t steplossCount_5_srlr = 0;
uint8_t steplossCount_5_srud = 0;

uint8_t gotoTargetPtCount_cllr = 0;
uint8_t gotoTargetPtCount_clud = 0;
uint8_t gotoTargetPtCount_sllr = 0;
uint8_t gotoTargetPtCount_slud = 0;
uint8_t gotoTargetPtCount_crlr = 0;
uint8_t gotoTargetPtCount_crud = 0;
uint8_t gotoTargetPtCount_srlr = 0;
uint8_t gotoTargetPtCount_srud = 0;

uint8_t gotoZeroPtCount_cllr = 0;
uint8_t gotoZeroPtCount_clud = 0;
uint8_t gotoZeroPtCount_sllr = 0;
uint8_t gotoZeroPtCount_slud = 0;
uint8_t gotoZeroPtCount_crlr = 0;
uint8_t gotoZeroPtCount_crud = 0;
uint8_t gotoZeroPtCount_srlr = 0;
uint8_t gotoZeroPtCount_srud = 0;


uint16_t zeroPt = 0;
uint16_t currentPt_cllr = 0;
uint16_t currentPt_clud = 0;
uint16_t currentPt_sllr = 0;
uint16_t currentPt_slud = 0;
uint16_t currentPt_crlr = 0;
uint16_t currentPt_crud = 0;
uint16_t currentPt_srlr = 0;
uint16_t currentPt_srud = 0;

uint16_t stopPt_cllr = 0;
uint16_t stopPt_clud = 0;
uint16_t stopPt_sllr = 0;
uint16_t stopPt_slud = 0;
uint16_t stopPt_crlr = 0;
uint16_t stopPt_crud = 0;
uint16_t stopPt_srlr = 0;
uint16_t stopPt_srud = 0;

uint16_t startPt_cllr = 0;
uint16_t startPt_clud = 0;
uint16_t startPt_sllr = 0;
uint16_t startPt_slud = 0;
uint16_t startPt_crlr = 0;
uint16_t startPt_crud = 0;
uint16_t startPt_srlr = 0;
uint16_t startPt_srud = 0;

uint16_t recoveryPt_cllr = 0;
uint16_t recoveryPt_clud = 0;
uint16_t recoveryPt_sllr = 0;
uint16_t recoveryPt_slud = 0;
uint16_t recoveryPt_crlr = 0;
uint16_t recoveryPt_crud = 0;
uint16_t recoveryPt_srlr = 0;
uint16_t recoveryPt_srud = 0;


uint16_t deltaPt_cllr = 0;
uint16_t deltaPt_clud = 0;
uint16_t deltaPt_sllr = 0;
uint16_t deltaPt_slud = 0;
uint16_t deltaPt_crlr = 0;
uint16_t deltaPt_crud = 0;
uint16_t deltaPt_srlr = 0;
uint16_t deltaPt_srud = 0;

uint16_t absDeltaPt_cllr = 0;
uint16_t absDeltaPt_clud = 0;
uint16_t absDeltaPt_sllr = 0;
uint16_t absDeltaPt_slud = 0;
uint16_t absDeltaPt_crlr = 0;
uint16_t absDeltaPt_crud = 0;
uint16_t absDeltaPt_srlr = 0;
uint16_t absDeltaPt_srud = 0;

uint16_t canCenterPt = 0;

uint8_t eventCountTest_cllr = 0;
uint8_t eventCountTest_clud = 0;
uint8_t eventCountTest_sllr = 0;
uint8_t eventCountTest_slud = 0;
uint8_t eventCountTest_crlr = 0;
uint8_t eventCountTest_crud = 0;
uint8_t eventCountTest_srlr = 0;
uint8_t eventCountTest_srud = 0;

uint8_t recoveryPattern_cllr = 0;
uint8_t recoveryPattern_clud = 0;
uint8_t recoveryPattern_sllr = 0;
uint8_t recoveryPattern_slud = 0;
uint8_t recoveryPattern_crlr = 0;
uint8_t recoveryPattern_crud = 0;
uint8_t recoveryPattern_srlr = 0;
uint8_t recoveryPattern_srud = 0;


uint8_t gotoZeroPtFlag_cllr = FALSE;
uint8_t gotoZeroPtFlag_clud = FALSE;
uint8_t gotoZeroPtFlag_sllr = FALSE;
uint8_t gotoZeroPtFlag_slud = FALSE;
uint8_t gotoZeroPtFlag_crlr = FALSE;
uint8_t gotoZeroPtFlag_crud = FALSE;
uint8_t gotoZeroPtFlag_srlr = FALSE;
uint8_t gotoZeroPtFlag_srud = FALSE;

uint8_t Check300ms_flag_cllr = FALSE;
uint8_t Check300ms_flag_clud = FALSE;
uint8_t Check300ms_flag_sllr = FALSE;
uint8_t Check300ms_flag_slud = FALSE;
uint8_t Check300ms_flag_crlr = FALSE;
uint8_t Check300ms_flag_crud = FALSE;
uint8_t Check300ms_flag_srlr = FALSE;
uint8_t Check300ms_flag_srud = FALSE;

uint8_t Check300ms_flag_1_cllr = FALSE;
uint8_t Check300ms_flag_1_clud = FALSE;
uint8_t Check300ms_flag_1_sllr = FALSE;
uint8_t Check300ms_flag_1_slud = FALSE;
uint8_t Check300ms_flag_1_crlr = FALSE;
uint8_t Check300ms_flag_1_crud = FALSE;
uint8_t Check300ms_flag_1_srlr = FALSE;
uint8_t Check300ms_flag_1_srud = FALSE;

uint8_t Check300ms_flag_2_cllr = FALSE;
uint8_t Check300ms_flag_2_clud = FALSE;
uint8_t Check300ms_flag_2_sllr = FALSE;
uint8_t Check300ms_flag_2_slud = FALSE;
uint8_t Check300ms_flag_2_crlr = FALSE;
uint8_t Check300ms_flag_2_crud = FALSE;
uint8_t Check300ms_flag_2_srlr = FALSE;
uint8_t Check300ms_flag_2_srud = FALSE;

uint8_t Check300ms_flag_3_cllr = FALSE;
uint8_t Check300ms_flag_3_clud = FALSE;
uint8_t Check300ms_flag_3_sllr = FALSE;
uint8_t Check300ms_flag_3_slud = FALSE;
uint8_t Check300ms_flag_3_crlr = FALSE;
uint8_t Check300ms_flag_3_crud = FALSE;
uint8_t Check300ms_flag_3_srlr = FALSE;
uint8_t Check300ms_flag_3_srud = FALSE;

uint8_t Check300ms_flag_4_cllr = FALSE;
uint8_t Check300ms_flag_4_clud = FALSE;
uint8_t Check300ms_flag_4_sllr = FALSE;
uint8_t Check300ms_flag_4_slud = FALSE;
uint8_t Check300ms_flag_4_crlr = FALSE;
uint8_t Check300ms_flag_4_crud = FALSE;
uint8_t Check300ms_flag_4_srlr = FALSE;
uint8_t Check300ms_flag_4_srud = FALSE;

uint8_t Check300ms_flag_5_cllr = FALSE;
uint8_t Check300ms_flag_5_clud = FALSE;
uint8_t Check300ms_flag_5_sllr = FALSE;
uint8_t Check300ms_flag_5_slud = FALSE;
uint8_t Check300ms_flag_5_crlr = FALSE;
uint8_t Check300ms_flag_5_crud = FALSE;
uint8_t Check300ms_flag_5_srlr = FALSE;
uint8_t Check300ms_flag_5_srud = FALSE;


uint8_t recoveryModeFlag_cllr = FALSE;
uint8_t recoveryModeFlag_clud = FALSE;
uint8_t recoveryModeFlag_sllr = FALSE;
uint8_t recoveryModeFlag_slud = FALSE;
uint8_t recoveryModeFlag_crlr = FALSE;
uint8_t recoveryModeFlag_crud = FALSE;
uint8_t recoveryModeFlag_srlr = FALSE;
uint8_t recoveryModeFlag_srud = FALSE;

uint8_t zero_flag_cllr = FALSE;
uint8_t zero_flag_clud = FALSE;
uint8_t zero_flag_sllr = FALSE;
uint8_t zero_flag_slud = FALSE;
uint8_t zero_flag_crlr = FALSE;
uint8_t zero_flag_crud = FALSE;
uint8_t zero_flag_srlr = FALSE;
uint8_t zero_flag_srud = FALSE;

uint8_t lastTryFlag_cllr = FALSE;
uint8_t lastTryFlag_clud = FALSE;
uint8_t lastTryFlag_sllr = FALSE;
uint8_t lastTryFlag_slud = FALSE;
uint8_t lastTryFlag_crlr = FALSE;
uint8_t lastTryFlag_crud = FALSE;
uint8_t lastTryFlag_srlr = FALSE;
uint8_t lastTryFlag_srud = FALSE;

uint8_t recoveryFinishFlag_cllr = FALSE;
uint8_t recoveryFinishFlag_clud = FALSE;
uint8_t recoveryFinishFlag_sllr = FALSE;
uint8_t recoveryFinishFlag_slud = FALSE;
uint8_t recoveryFinishFlag_crlr = FALSE;
uint8_t recoveryFinishFlag_crud = FALSE;
uint8_t recoveryFinishFlag_srlr = FALSE;
uint8_t recoveryFinishFlag_srud = FALSE;

uint8_t goToTargetFlag_cllr = FALSE;
uint8_t goToTargetFlag_clud = FALSE;
uint8_t goToTargetFlag_sllr = FALSE;
uint8_t goToTargetFlag_slud = FALSE;
uint8_t goToTargetFlag_crlr = FALSE;
uint8_t goToTargetFlag_crud = FALSE;
uint8_t goToTargetFlag_srlr = FALSE;
uint8_t goToTargetFlag_srud = FALSE;

uint8_t Steploss_OpDone_TimeOutFlag_sllr = FALSE;
uint8_t Steploss_OpDone_TimeOutFlag_slud = FALSE;
uint8_t Steploss_OpDone_TimeOutFlag_cllr = FALSE;
uint8_t Steploss_OpDone_TimeOutFlag_clud = FALSE;
uint8_t Steploss_OpDone_TimeOutFlag_crlr = FALSE;
uint8_t Steploss_OpDone_TimeOutFlag_crud = FALSE;
uint8_t Steploss_OpDone_TimeOutFlag_srlr = FALSE;
uint8_t Steploss_OpDone_TimeOutFlag_srud = FALSE;

uint8_t cycleModeStopCount_sllr = 0;
uint8_t cycleModeStopCount_slud = 0;
uint8_t cycleModeStopCount_cllr = 0;
uint8_t cycleModeStopCount_clud = 0;
uint8_t cycleModeStopCount_crlr = 0;
uint8_t cycleModeStopCount_crud = 0;
uint8_t cycleModeStopCount_srlr = 0;
uint8_t cycleModeStopCount_srud = 0;


extern uint8_t sleepFlag;
extern uint8_t cycleModeFlag_cllr;
extern uint8_t cycleModeFlag_clud;
extern uint8_t cycleModeFlag_sllr;
extern uint8_t cycleModeFlag_slud;

extern uint8_t cycleModeFlag_crlr;
extern uint8_t cycleModeFlag_crud;
extern uint8_t cycleModeFlag_srlr;
extern uint8_t cycleModeFlag_srud;

extern uint8_t fullcloseModeFlag_cllr;
extern uint8_t fullcloseModeFlag_clud;
extern uint8_t fullcloseModeFlag_sllr;
extern uint8_t fullcloseModeFlag_slud;

extern uint8_t fullcloseModeFlag_crlr;
extern uint8_t fullcloseModeFlag_crud;
extern uint8_t fullcloseModeFlag_srlr;
extern uint8_t fullcloseModeFlag_srud;

extern uint8_t FrDrCycleMode_Flag;
extern uint8_t FrPsCycleMode_Flag;
extern uint8_t FrDrPsCycleMode_Flag;


Recovery_flag_t	recoveryFlags = {FALSE, };	

extern Lvnt_3_data Can_Tx_Evnt_3;
extern Lvnt_8_data Can_Tx_Evnt_8;
extern Lvnt_9_data Can_Tx_Evnt_9;

extern uint8_t secureMainFlag;



void Set_LH_Special_Cmd_Init(lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl)
{
	lin_SpecialCtrl->LDATA.EVNT_CMD = HARD_STOP_INIT;
	lin_SpecialCtrl->LDATA.EVNT_Broad = BROAD_CAST;
	
	l_u8_wr_LI0_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
	l_bool_wr_LI0_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);
}

void Set_RH_Special_Cmd_Init(lin_RH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl)
{
	lin_SpecialCtrl->LDATA.EVNT_CMD = HARD_STOP_INIT;
	lin_SpecialCtrl->LDATA.EVNT_Broad = BROAD_CAST;
	
	l_u8_wr_LI1_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
	l_bool_wr_LI1_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);
}

void Set_RC_Special_Cmd_Init(lin_RC_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl)
{
	lin_SpecialCtrl->LDATA.EVNT_CMD = HARD_STOP_INIT;
	lin_SpecialCtrl->LDATA.EVNT_Broad = BROAD_CAST;
	
	l_u8_wr_LI2_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
	l_bool_wr_LI2_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);
}



void Lin_LH_Scheduler_Normal_Start(void)
{
	Set_LH_Special_Cmd_Init(&LIN_LH_EVNT_SPECIAL_CMD);
		
	l_sch_set(LI0, LI0_SCHEDULER_EVNT_NORMAL_FR_DRV, 0u);
	Amo_timer_Stop(timer_62);
}

void Lin_RH_Scheduler_Normal_Start(void)
{
	Set_RH_Special_Cmd_Init(&LIN_RH_EVNT_SPECIAL_CMD);
		
	l_sch_set(LI1, LI1_SCHEDULER_EVNT_NORMAL_FR_PASS, 0u);
	Amo_timer_Stop(timer_65);
}

void Lin_RR_Scheduler_Normal_Start(void)
{
	Set_RC_Special_Cmd_Init(&LIN_RC_EVNT_SPECIAL_CMD);
		
	l_sch_set(LI2, LI2_SCHEDULER_EVNT_NORMAL_RR_CTR, 0u);
	Amo_timer_Stop(timer_68);
}



void Lin_LH_HardStop(t_lin_LH_FR_STATUS *lin_LH_Status, lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_LH_EVNT_MASTER_CMD_11 *lin_LH_Ctrl)							
{
	lin_SpecialCtrl->LDATA.EVNT_CMD = HARD_STOP_ACT;
	lin_SpecialCtrl->LDATA.EVNT_Broad = UNI_CAST;

	if(FR_SLLR_FailSafe_Flag == TRUE)
	{
//		if(lin_LH_Status->fr_sllr_status.LDATA.FR_SLLR_OpDone != 1)
		{
			lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_SLLR;

			l_u8_wr_LI0_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
			l_u8_wr_LI0_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
			l_bool_wr_LI0_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

			lin_LH_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = l_u16_rd_LI0_FR_SLLR_ActuatorState();
			l_u16_wr_LI0_EVNT_Front_Side_LH_LeftRight_TargetPosition(lin_LH_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition);

		}
		FR_SLLR_FailSafe_Flag = FALSE;
	}

	if(FR_SLUD_FailSafe_Flag == TRUE)
	{
//		if(lin_LH_Status->fr_slud_status.LDATA.FR_SLUD_OpDone != 1)
		{
			lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_SLUD;

			l_u8_wr_LI0_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
			l_u8_wr_LI0_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
			l_bool_wr_LI0_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

			lin_LH_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = l_u16_rd_LI0_FR_SLUD_ActuatorState();
			l_u16_wr_LI0_EVNT_Front_Side_LH_UpDown_TargetPosition(lin_LH_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition);

		}
		FR_SLUD_FailSafe_Flag = FALSE;
	}

	if(FR_CLLR_FailSafe_Flag == TRUE)
	{
//		if(lin_LH_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone != 1)
		{
			lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_CLLR;

			l_u8_wr_LI0_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
			l_u8_wr_LI0_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
			l_bool_wr_LI0_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

			lin_LH_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = l_u16_rd_LI0_FR_CLLR_ActuatorState();
			l_u16_wr_LI0_EVNT_Front_Center_LH_LeftRight_TargetPosition(lin_LH_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition);

		}
		FR_CLLR_FailSafe_Flag = FALSE;
	}

	if(FR_CLUD_FailSafe_Flag == TRUE)
	{
//		if(lin_LH_Status->fr_clud_status.LDATA.FR_CLUD_OpDone != 1)
		{
			lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_CLUD;

			l_u8_wr_LI0_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
			l_u8_wr_LI0_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
			l_bool_wr_LI0_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

			lin_LH_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = l_u16_rd_LI0_FR_CLUD_ActuatorState();
			l_u16_wr_LI0_EVNT_Front_Center_LH_UpDown_TargetPosition(lin_LH_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition);

		}
		FR_CLUD_FailSafe_Flag = FALSE;
	}

	Amo_timer_Stop(timer_62);
	Amo_timer_Start(timer_62, 20, true, Lin_LH_Scheduler_Normal_Start);
}


void Lin_RH_HardStop(t_lin_RH_FR_STATUS *lin_RH_Status, lin_RH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_RH_EVNT_MASTER_CMD_21 *lin_RH_Ctrl)							
{
	lin_SpecialCtrl->LDATA.EVNT_CMD = HARD_STOP_ACT;
	lin_SpecialCtrl->LDATA.EVNT_Broad = UNI_CAST;

	if(FR_CRLR_FailSafe_Flag == TRUE)
	{
//		if(lin_RH_Status->fr_crlr_status.LDATA.FR_CRLR_OpDone != 1)
		{
			lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_CRLR;

			l_u8_wr_LI1_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
			l_u8_wr_LI1_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
			l_bool_wr_LI1_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

			lin_RH_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = l_u16_rd_LI1_FR_CRLR_ActuatorState();
			l_u16_wr_LI1_EVNT_Front_Center_RH_LeftRight_TargetPosition(lin_RH_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition);

		}
		FR_CRLR_FailSafe_Flag = FALSE;
	}

	if(FR_CRUD_FailSafe_Flag == TRUE)
	{
//		if(lin_RH_Status->fr_crud_status.LDATA.FR_CRUD_OpDone != 1)
		{
			lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_CRUD;

			l_u8_wr_LI1_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
			l_u8_wr_LI1_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
			l_bool_wr_LI1_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

			lin_RH_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = l_u16_rd_LI1_FR_CRUD_ActuatorState();
			l_u16_wr_LI1_EVNT_Front_Center_RH_UpDown_TargetPosition(lin_RH_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition);
			
		}
		FR_CRUD_FailSafe_Flag = FALSE;
	}

	if(FR_SRLR_FailSafe_Flag == TRUE)
	{
//		if(lin_RH_Status->fr_srlr_status.LDATA.FR_SRLR_OpDone != 1)
		{
			lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_SRLR;

			l_u8_wr_LI1_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
			l_u8_wr_LI1_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
			l_bool_wr_LI1_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

			lin_RH_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = l_u16_rd_LI1_FR_SRLR_ActuatorState();
			l_u16_wr_LI1_EVNT_Front_Side_RH_LeftRight_TargetPosition(lin_RH_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition);

		}
		FR_SRLR_FailSafe_Flag = FALSE;
	}

	if(FR_SRUD_FailSafe_Flag == TRUE)
	{
//		if(lin_RH_Status->fr_srud_status.LDATA.FR_SRUD_OpDone != 1)
		{
			lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_SRUD;

			l_u8_wr_LI1_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
			l_u8_wr_LI1_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
			l_bool_wr_LI1_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

			lin_RH_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = l_u16_rd_LI1_FR_SRUD_ActuatorState();
			l_u16_wr_LI1_EVNT_Front_Side_RH_UpDown_TargetPosition(lin_RH_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition);

		}
		FR_SRUD_FailSafe_Flag = FALSE;
	}
	
	Amo_timer_Stop(timer_65);
	Amo_timer_Start(timer_65, 20, true, Lin_RH_Scheduler_Normal_Start);
}


void Lin_RC_HardStop(t_lin_RC_RR_STATUS *lin_RC_Status, lin_RC_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_RC_EVNT_MASTER_CMD_31 *lin_RC_Ctrl)							
{
	lin_SpecialCtrl->LDATA.EVNT_CMD = HARD_STOP_ACT;
	lin_SpecialCtrl->LDATA.EVNT_Broad = UNI_CAST;

	if(RR_CLLR_FailSafe_Flag == TRUE)
	{
//		if(lin_RC_Status->rr_cllr_status.LDATA.RR_CLLR_OpDone != 1)
		{
			lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_RR_CLLR;

			l_u8_wr_LI2_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
			l_u8_wr_LI2_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
			l_bool_wr_LI2_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

			lin_RC_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = l_u16_rd_LI2_RR_CLLR_ActuatorState();
			l_u16_wr_LI2_EVNT_Rear_Center_LH_LeftRight_TargetPosition(lin_RC_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition);
		}
		RR_CLLR_FailSafe_Flag = FALSE;
	}

	if(RR_CLUD_FailSafe_Flag == TRUE)
	{
//		if(lin_RC_Status->rr_clud_status.LDATA.RR_CLUD_OpDone != 1)
		{
			lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_RR_CLUD;

			l_u8_wr_LI2_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
			l_u8_wr_LI2_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
			l_bool_wr_LI2_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

			lin_RC_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = l_u16_rd_LI2_RR_CLUD_ActuatorState();
			l_u16_wr_LI2_EVNT_Rear_Center_LH_UpDown_TargetPosition(lin_RC_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition);
		}
		RR_CLUD_FailSafe_Flag = FALSE;
	}

	if(RR_CRLR_FailSafe_Flag == TRUE)
	{
//		if(lin_RC_Status->rr_crlr_status.LDATA.RR_CRLR_OpDone != 1)
		{
			lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_RR_CRLR;

			l_u8_wr_LI2_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
			l_u8_wr_LI2_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
			l_bool_wr_LI2_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

			lin_RC_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = l_u16_rd_LI2_RR_CRLR_ActuatorState();
			l_u16_wr_LI2_EVNT_Rear_Center_RH_LeftRight_TargetPosition(lin_RC_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition);
		}
		RR_CRLR_FailSafe_Flag = FALSE;
	}

	if(RR_CRUD_FailSafe_Flag == TRUE)
	{
//		if(lin_RC_Status->rr_crud_status.LDATA.RR_CRUD_OpDone != 1)
		{
			lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_RR_CRUD;

			l_u8_wr_LI2_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
			l_u8_wr_LI2_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
			l_bool_wr_LI2_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

			lin_RC_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = l_u16_rd_LI2_RR_CRUD_ActuatorState();
			l_u16_wr_LI2_EVNT_Rear_Center_RH_UpDown_TargetPosition(lin_RC_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition);
		}
		RR_CRUD_FailSafe_Flag = FALSE;
	}
	
	Amo_timer_Stop(timer_68);
	Amo_timer_Start(timer_68, 20, true, Lin_RR_Scheduler_Normal_Start);
}


void Evnt_LH_Special_Cmd(void)
{
	Lin_LH_HardStop(&LIN_LH_FR_STATUS, &LIN_LH_EVNT_SPECIAL_CMD, &LIN_LH_EVNT_MASTER_CMD);

	Amo_timer_Stop(timer_61);
}

void Evnt_RH_Special_Cmd(void)
{
	Lin_RH_HardStop(&LIN_RH_FR_STATUS, &LIN_RH_EVNT_SPECIAL_CMD, &LIN_RH_EVNT_MASTER_CMD);

	Amo_timer_Stop(timer_64);
}

void Evnt_RC_Special_Cmd(void)
{
	Lin_RC_HardStop(&LIN_RC_RR_STATUS, &LIN_RC_EVNT_SPECIAL_CMD, &LIN_RC_EVNT_MASTER_CMD);

	Amo_timer_Stop(timer_67);
}

void FR_DR_ExtFlt_SET(void)
{
	if(FR_SLLR_FailSafe_Flag == TRUE)
	{ 
		if(secureMainFlag == FALSE) { FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLLR_ExtFlt_Flag = true; }	
		FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_SLLR_ExtFlt = true;
	}
	if(FR_SLUD_FailSafe_Flag == TRUE)
	{ 
		if(secureMainFlag == FALSE) { FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLUD_ExtFlt_Flag = true; }
		FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_SLUD_ExtFlt = true;
	}
	if(FR_CLLR_FailSafe_Flag == TRUE)
	{ 
		if(secureMainFlag == FALSE) { FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLLR_ExtFlt_Flag = true; }
		FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_CLLR_ExtFlt = true;
	}
	if(FR_CLUD_FailSafe_Flag == TRUE)
	{ 
		if(secureMainFlag == FALSE) { FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLUD_ExtFlt_Flag = true; }
		FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_CLUD_ExtFlt = true;
	}
}

void FR_PS_ExtFlt_SET(void)
{
	if(FR_CRLR_FailSafe_Flag == TRUE)
	{ 
		if(secureMainFlag == FALSE) { FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRLR_ExtFlt_Flag = true; }
		FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_CRLR_ExtFlt = true;
	}
	if(FR_CRUD_FailSafe_Flag == TRUE)
	{ 
		if(secureMainFlag == FALSE) { FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRUD_ExtFlt_Flag = true; }
		FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_CRUD_ExtFlt = true;
	}
	if(FR_SRLR_FailSafe_Flag == TRUE)
	{ 
		if(secureMainFlag == FALSE) { FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRLR_ExtFlt_Flag = true; }
		FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_SRLR_ExtFlt = true;
	}
	if(FR_SRUD_FailSafe_Flag == TRUE)
	{ 
		if(secureMainFlag == FALSE) { FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRUD_ExtFlt_Flag = true; }
		FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_SRUD_ExtFlt = true;
	}
}


void Lin_Scheduler_SpecialCmd_Start(void)
{
	if(FR_SLLR_FailSafe_Flag == TRUE || FR_SLUD_FailSafe_Flag == TRUE || FR_CLLR_FailSafe_Flag == TRUE || FR_CLUD_FailSafe_Flag == TRUE)
	{
		FR_DR_ExtFlt_SET();

		l_sch_set(LI0, LI0_SCHEDULER_EVNT_SPECIAL_CMD_FR_DRV, 0u);

		Amo_timer_Stop(timer_61);
		Amo_timer_Start(timer_61, 20, true, Evnt_LH_Special_Cmd);
	}

	if(FR_CRLR_FailSafe_Flag == TRUE || FR_CRUD_FailSafe_Flag == TRUE || FR_SRLR_FailSafe_Flag == TRUE || FR_SRUD_FailSafe_Flag == TRUE)
	{
		FR_PS_ExtFlt_SET();
		
		l_sch_set(LI1, LI1_SCHEDULER_EVNT_SPECIAL_CMD_FR_PASS, 0u);

		Amo_timer_Stop(timer_64);
		Amo_timer_Start(timer_64, 20, true, Evnt_RH_Special_Cmd);
	}

	if(RR_CLLR_FailSafe_Flag == TRUE || RR_CLUD_FailSafe_Flag == TRUE || RR_CRLR_FailSafe_Flag == TRUE || RR_CRUD_FailSafe_Flag == TRUE)
	{
		l_sch_set(LI2, LI2_SCHEDULER_EVNT_SPECIAL_CMD_RR_CTR, 0u);

		Amo_timer_Stop(timer_67);
		Amo_timer_Start(timer_67, 20, true, Evnt_RC_Special_Cmd);
	}

}


void lin_Read_FR_LH_OpDone(t_lin_LH_FR_STATUS *lin_LH_Status)
{
	if(l_flg_tst_LI0_FR_SLLR_OpDone_flag())
	{
		l_flg_clr_LI0_FR_SLLR_OpDone_flag();

		if(FR_LINBUSOff_FLAG.FR_SLLR_LIN_BusOff == false) 
		{
			lin_LH_Status->fr_sllr_status.LDATA.FR_SLLR_OpDone = l_bool_rd_LI0_FR_SLLR_OpDone();

			if(lin_LH_Status->fr_sllr_status.LDATA.FR_SLLR_OpDone != 1) 
			{ 
				Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
				FR_SLLR_FailSafe_Flag = TRUE;
			}
		}
	}

	if(l_flg_tst_LI0_FR_SLUD_OpDone_flag())
	{
		l_flg_clr_LI0_FR_SLUD_OpDone_flag();

		if(FR_LINBUSOff_FLAG.FR_SLUD_LIN_BusOff == false) 
		{
			lin_LH_Status->fr_slud_status.LDATA.FR_SLUD_OpDone = l_bool_rd_LI0_FR_SLUD_OpDone();

			if(lin_LH_Status->fr_slud_status.LDATA.FR_SLUD_OpDone != 1) 
			{ 
				Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
				FR_SLUD_FailSafe_Flag = TRUE;
			}
		}
	}

	if(l_flg_tst_LI0_FR_CLLR_OpDone_flag())
	{
		l_flg_clr_LI0_FR_CLLR_OpDone_flag();

		if(FR_LINBUSOff_FLAG.FR_CLLR_LIN_BusOff == false) 
		{
			lin_LH_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone = l_bool_rd_LI0_FR_CLLR_OpDone();

			if(lin_LH_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone != 1) 
			{ 
				Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
				FR_CLLR_FailSafe_Flag = TRUE;
			}
		}
	}

	if(l_flg_tst_LI0_FR_CLUD_OpDone_flag())
	{
		l_flg_clr_LI0_FR_CLUD_OpDone_flag();

		if(FR_LINBUSOff_FLAG.FR_CLUD_LIN_BusOff == false) 
		{
			lin_LH_Status->fr_clud_status.LDATA.FR_CLUD_OpDone = l_bool_rd_LI0_FR_CLUD_OpDone();

			if(lin_LH_Status->fr_clud_status.LDATA.FR_CLUD_OpDone != 1) 
			{ 
				Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
				FR_CLUD_FailSafe_Flag = TRUE;
			}
		}
	}

}


void lin_Read_FR_RH_OpDone(t_lin_RH_FR_STATUS *lin_RH_Status)
{
	if(l_flg_tst_LI1_FR_CRLR_OpDone_flag())
	{
		l_flg_clr_LI1_FR_CRLR_OpDone_flag();

		if(FR_LINBUSOff_FLAG.FR_CRLR_LIN_BusOff == false) 
		{
			lin_RH_Status->fr_crlr_status.LDATA.FR_CRLR_OpDone = l_bool_rd_LI1_FR_CRLR_OpDone();

			if(lin_RH_Status->fr_crlr_status.LDATA.FR_CRLR_OpDone != 1) 
			{ 
				Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
				FR_CRLR_FailSafe_Flag = TRUE;
			}
		}
	}

	if(l_flg_tst_LI1_FR_CRUD_OpDone_flag())
	{
		l_flg_clr_LI1_FR_CRUD_OpDone_flag();

		if(FR_LINBUSOff_FLAG.FR_CRUD_LIN_BusOff == false) 
		{
			lin_RH_Status->fr_crud_status.LDATA.FR_CRUD_OpDone = l_bool_rd_LI1_FR_CRUD_OpDone();

			if(lin_RH_Status->fr_crud_status.LDATA.FR_CRUD_OpDone != 1) 
			{ 
				Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
				FR_CRUD_FailSafe_Flag = TRUE;
			}
		}
	}
	
	if(l_flg_tst_LI1_FR_SRLR_OpDone_flag())
	{
		l_flg_clr_LI1_FR_SRLR_OpDone_flag();

		if(FR_LINBUSOff_FLAG.FR_SRLR_LIN_BusOff == false) 
		{
			lin_RH_Status->fr_srlr_status.LDATA.FR_SRLR_OpDone = l_bool_rd_LI1_FR_SRLR_OpDone();

			if(lin_RH_Status->fr_srlr_status.LDATA.FR_SRLR_OpDone != 1) 
			{ 
				Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
				FR_SRLR_FailSafe_Flag = TRUE;
			}
		}
	}

	if(l_flg_tst_LI1_FR_SRUD_OpDone_flag())
	{
		l_flg_clr_LI1_FR_SRUD_OpDone_flag();

		if(FR_LINBUSOff_FLAG.FR_SRUD_LIN_BusOff == false) 
		{
			lin_RH_Status->fr_srud_status.LDATA.FR_SRUD_OpDone = l_bool_rd_LI1_FR_SRUD_OpDone();

			if(lin_RH_Status->fr_srud_status.LDATA.FR_SRUD_OpDone != 1) 
			{ 
				Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
				FR_SRUD_FailSafe_Flag = TRUE;
			}
		}
	}
}

void lin_Read_RR_RC_OpDone(t_lin_RC_RR_STATUS *lin_RR_Status)
{
	if(l_flg_tst_LI2_RR_CLLR_OpDone_flag())
	{
		l_flg_clr_LI2_RR_CLLR_OpDone_flag();

		lin_RR_Status->rr_cllr_status.LDATA.RR_CLLR_OpDone = l_bool_rd_LI2_RR_CLLR_OpDone();

		if(lin_RR_Status->rr_cllr_status.LDATA.RR_CLLR_OpDone != 1) 
		{ 
			Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
			RR_CLLR_FailSafe_Flag = TRUE;
		}
	}

	if(l_flg_tst_LI2_RR_CLUD_OpDone_flag())
	{
		l_flg_clr_LI2_RR_CLUD_OpDone_flag();

		lin_RR_Status->rr_clud_status.LDATA.RR_CLUD_OpDone = l_bool_rd_LI2_RR_CLUD_OpDone();

		if(lin_RR_Status->rr_clud_status.LDATA.RR_CLUD_OpDone != 1) 
		{ 
			Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
			RR_CLUD_FailSafe_Flag = TRUE;
		}
	}
	
	if(l_flg_tst_LI2_RR_CRLR_OpDone_flag())
	{
		l_flg_clr_LI2_RR_CRLR_OpDone_flag();

		lin_RR_Status->rr_crlr_status.LDATA.RR_CRLR_OpDone = l_bool_rd_LI2_RR_CRLR_OpDone();

		if(lin_RR_Status->rr_crlr_status.LDATA.RR_CRLR_OpDone != 1) 
		{ 
			Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
			RR_CRLR_FailSafe_Flag = TRUE;
		}
	}

	if(l_flg_tst_LI2_RR_CRUD_OpDone_flag())
	{
		l_flg_clr_LI2_RR_CRUD_OpDone_flag();

		lin_RR_Status->rr_crud_status.LDATA.RR_CRUD_OpDone = l_bool_rd_LI2_RR_CRUD_OpDone();

		if(lin_RR_Status->rr_crud_status.LDATA.RR_CRUD_OpDone != 1) 
		{ 
			Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
			RR_CRUD_FailSafe_Flag = TRUE;
		}
	}
}

void Opdone_TimeOut_LH_Check(void)
{

	lin_Read_FR_LH_OpDone(&LIN_LH_FR_STATUS);
	
	Amo_timer_Stop(timer_60);

}

void Opdone_TimeOut_RH_Check(void)
{
	lin_Read_FR_RH_OpDone(&LIN_RH_FR_STATUS);

	Amo_timer_Stop(timer_63);

}

void Opdone_TimeOut_RC_Check(void)
{
	lin_Read_RR_RC_OpDone(&LIN_RC_RR_STATUS);

	Amo_timer_Stop(timer_66);

}


void RecoveryStart_goToZero_FR_CLUD_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
//	recoveryCount++;

	gotoZeroPtCount_clud++;

	eventCountTest_clud++;

	zero_flag_clud = FALSE;

	recoveryModeFlag_clud = TRUE;

	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_2);

	
	lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 0;

	if(recoveryPattern_clud == Repeat_zeroPt || ((recoveryPattern_clud == Cant_go_to_zeroPt) && (eventCountTest_clud == 1)))
	{
		lin_Pt->FrDrPt.prevPt_FR_CLUD = lin_Pt->FrDrPt.currentPt_FR_CLUD;
	}

	
	lin_Pt->FrDrPt.currentPt_FR_CLUD = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	if(recoveryPattern_clud == Cant_go_to_zeroPt)
	{
		Check300ms_flag_1_clud = FALSE;
	}

}



void RecoveryStart_goToZero_FR_CLLR_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
//	recoveryCount++;

	gotoZeroPtCount_cllr++;

	eventCountTest_cllr++;

	zero_flag_cllr = FALSE;

	recoveryModeFlag_cllr = TRUE;

	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_2);

//	l_u8_wr_LI0_EVNT_SPEED(10);
	
	lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = 0;

	if(recoveryPattern_cllr == Repeat_zeroPt || ((recoveryPattern_cllr == Cant_go_to_zeroPt) && (eventCountTest_cllr == 1)))
	{
		lin_Pt->FrDrPt.prevPt_FR_CLLR = lin_Pt->FrDrPt.currentPt_FR_CLLR;
	}
	
	lin_Pt->FrDrPt.currentPt_FR_CLLR = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	if(recoveryPattern_cllr == Cant_go_to_zeroPt)
	{
		Check300ms_flag_1_cllr = FALSE;
	}

}

void RecoveryStart_goToZero_FR_SLLR_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
    // recoveryCount++;

    gotoZeroPtCount_sllr++;

    eventCountTest_sllr++;

    zero_flag_sllr = FALSE;

    recoveryModeFlag_sllr = TRUE;

    SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_2);

    lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 0;

    if(recoveryPattern_sllr == Repeat_zeroPt || ((recoveryPattern_sllr == Cant_go_to_zeroPt) && (eventCountTest_sllr == 1)))
    {
        lin_Pt->FrDrPt.prevPt_FR_SLLR = lin_Pt->FrDrPt.currentPt_FR_SLLR;
    }
    else
    {
        // Handle other cases if needed
    }

    lin_Pt->FrDrPt.currentPt_FR_SLLR = lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;

    lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

		if(recoveryPattern_sllr == Cant_go_to_zeroPt)
		{
			Check300ms_flag_1_sllr = FALSE;
		}
}

void RecoveryStart_goToZero_FR_SLUD_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
    // recoveryCount++;

    gotoZeroPtCount_slud++;

    eventCountTest_slud++;

    zero_flag_slud = FALSE;

    recoveryModeFlag_slud = TRUE;

    SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_2);

    lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 0;

    if(recoveryPattern_slud == Repeat_zeroPt || ((recoveryPattern_slud == Cant_go_to_zeroPt) && (eventCountTest_slud == 1)))
    {
        lin_Pt->FrDrPt.prevPt_FR_SLUD = lin_Pt->FrDrPt.currentPt_FR_SLUD;
    }
    else
    {
        // Handle other cases if needed
    }

    lin_Pt->FrDrPt.currentPt_FR_SLUD = lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;

    lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

		if(recoveryPattern_slud == Cant_go_to_zeroPt)
		{
			Check300ms_flag_1_slud = FALSE;
		}
}

void RecoveryStart_goToZero_FR_CRLR_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
    // recoveryCount++;

    gotoZeroPtCount_crlr++;

    eventCountTest_crlr++;

    zero_flag_crlr = FALSE;

    recoveryModeFlag_crlr = TRUE;

    SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_2);

    lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = 0;

    if(recoveryPattern_crlr == Repeat_zeroPt || ((recoveryPattern_crlr == Cant_go_to_zeroPt) && (eventCountTest_crlr == 1)))
    {
        lin_Pt->FrPsPt.prevPt_FR_CRLR = lin_Pt->FrPsPt.currentPt_FR_CRLR;
    }
    else
    {
        // Handle other cases if needed
    }

    lin_Pt->FrPsPt.currentPt_FR_CRLR = lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;

    lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

		if(recoveryPattern_crlr == Cant_go_to_zeroPt)
		{
			Check300ms_flag_1_crlr = FALSE;
		}
}

void RecoveryStart_goToZero_FR_CRUD_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
    // recoveryCount++;

    gotoZeroPtCount_crud++;

    eventCountTest_crud++;

    zero_flag_crud = FALSE;

    recoveryModeFlag_crud = TRUE;

    SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_2);

    lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = 0;

    if(recoveryPattern_crud == Repeat_zeroPt || ((recoveryPattern_crud == Cant_go_to_zeroPt) && (eventCountTest_crud == 1)))
    {
        lin_Pt->FrPsPt.prevPt_FR_CRUD = lin_Pt->FrPsPt.currentPt_FR_CRUD;
    }
    else
    {
        // Handle other cases if needed
    }

    lin_Pt->FrPsPt.currentPt_FR_CRUD = lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;

    lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

		if(recoveryPattern_crud == Cant_go_to_zeroPt)
		{
			Check300ms_flag_1_crud = FALSE;
		}
}

void RecoveryStart_goToZero_FR_SRLR_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
    // recoveryCount++;

    gotoZeroPtCount_srlr++;

    eventCountTest_srlr++;

    zero_flag_srlr = FALSE;

    recoveryModeFlag_srlr = TRUE;

    SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_2);

    lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = 0;

    if(recoveryPattern_srlr == Repeat_zeroPt || ((recoveryPattern_srlr == Cant_go_to_zeroPt) && (eventCountTest_srlr == 1)))
    {
        lin_Pt->FrPsPt.prevPt_FR_SRLR = lin_Pt->FrPsPt.currentPt_FR_SRLR;
    }
    else
    {
        // Handle other cases if needed
    }

    lin_Pt->FrPsPt.currentPt_FR_SRLR = lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;

    lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

		if(recoveryPattern_srlr == Cant_go_to_zeroPt)
		{
			Check300ms_flag_1_srlr = FALSE;
		}
}

void RecoveryStart_goToZero_FR_SRUD_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
    // recoveryCount++;

    gotoZeroPtCount_srud++;

    eventCountTest_srud++;

    zero_flag_srud = FALSE;

    recoveryModeFlag_srud = TRUE;

    SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_2);

    lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = 0;

    if(recoveryPattern_srud == Repeat_zeroPt || ((recoveryPattern_srud == Cant_go_to_zeroPt) && (eventCountTest_srud == 1)))
    {
        lin_Pt->FrPsPt.prevPt_FR_SRUD = lin_Pt->FrPsPt.currentPt_FR_SRUD;
    }
    else
    {
        // Handle other cases if needed
    }

    lin_Pt->FrPsPt.currentPt_FR_SRUD = lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;

    lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

		if(recoveryPattern_srud == Cant_go_to_zeroPt)
		{
			Check300ms_flag_1_srud = FALSE;
		}
}


void RecoveryMode_LH_1_cllr(void)
{
	Amo_timer_Stop(timer_98);
	RecoveryStart_goToZero_FR_CLLR_1(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
}

void RecoveryMode_LH_1_clud(void)
{
	Amo_timer_Stop(timer_98);
	RecoveryStart_goToZero_FR_CLUD_1(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
}

void RecoveryMode_LH_1_sllr(void)
{
	Amo_timer_Stop(timer_98);
	RecoveryStart_goToZero_FR_SLLR_1(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
}

void RecoveryMode_LH_1_slud(void)
{
	Amo_timer_Stop(timer_98);
	RecoveryStart_goToZero_FR_SLUD_1(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
}

void RecoveryMode_RH_1_crlr(void)
{
	Amo_timer_Stop(timer_123);
	RecoveryStart_goToZero_FR_CRLR_1(&LIN_RH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
}

void RecoveryMode_RH_1_crud(void)
{
	Amo_timer_Stop(timer_123);
	RecoveryStart_goToZero_FR_CRUD_1(&LIN_RH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
}

void RecoveryMode_RH_1_srlr(void)
{
	Amo_timer_Stop(timer_123);
	RecoveryStart_goToZero_FR_SRLR_1(&LIN_RH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
}

void RecoveryMode_RH_1_srud(void)
{
	Amo_timer_Stop(timer_123);
	RecoveryStart_goToZero_FR_SRUD_1(&LIN_RH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
}



void GO_To_TargetPt_cllr(void)
{
	Amo_timer_Stop(timer_94);
	Recovery_goToTargetPt_FR_CLLR_1(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
}

void GO_To_TargetPt_clud(void)
{
	Amo_timer_Stop(timer_97);
	Recovery_goToTargetPt_FR_CLUD_1(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
}

void GO_To_TargetPt_sllr(void)
{
	Amo_timer_Stop(timer_112);
	Recovery_goToTargetPt_FR_SLLR_1(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
}

void GO_To_TargetPt_slud(void)
{
	Amo_timer_Stop(timer_113);
	Recovery_goToTargetPt_FR_SLUD_1(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
}

void GO_To_TargetPt_crlr(void)
{
	Amo_timer_Stop(timer_114);
	Recovery_goToTargetPt_FR_CRLR_1(&LIN_RH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
}

void GO_To_TargetPt_crud(void)
{
	Amo_timer_Stop(timer_115);
	Recovery_goToTargetPt_FR_CRUD_1(&LIN_RH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
}

void GO_To_TargetPt_srlr(void)
{
	Amo_timer_Stop(timer_116);
	Recovery_goToTargetPt_FR_SRLR_1(&LIN_RH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
}

void GO_To_TargetPt_srud(void)
{
	Amo_timer_Stop(timer_117);
	Recovery_goToTargetPt_FR_SRUD_1(&LIN_RH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
}




void Lin_LH_Scheduler_Normal_Start_1(void)
{
//	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);
	
	Set_LH_Special_Cmd_Init(&LIN_LH_EVNT_SPECIAL_CMD);
		
	l_sch_set(LI0, LI0_SCHEDULER_EVNT_NORMAL_FR_DRV, 0u);
	Amo_timer_Stop(timer_62);
}

void Lin_RH_Scheduler_Normal_Start_1(void)
{
//	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);
	
	Set_RH_Special_Cmd_Init(&LIN_RH_EVNT_SPECIAL_CMD);
		
	l_sch_set(LI1, LI1_SCHEDULER_EVNT_NORMAL_FR_PASS, 0u);
	Amo_timer_Stop(timer_124);
}


/*
typedef enum 
{
	idleState,
  hardStop_sllr,      
  hardStop_slud,      
  hardStop_cllr,
  hardStop_clud
} HardStopState;

HardStopState currentState = hardStop_cllr;

void handleStopState(HardStopState state) 
{
    switch (state) 
		{
    case idleState:
        
        break;
				
    case hardStop_sllr:
       
        break;
		
    case hardStop_slud:
        
        break;
				
    case hardStop_cllr:
         Lin_LH_HardStop_1(&LIN_LH_FR_STATUS, &LIN_LH_EVNT_SPECIAL_CMD, &LIN_LH_EVNT_MASTER_CMD);
        break;
				
    case hardStop_clud:
        Lin_LH_HardStop_1_clud(&LIN_LH_FR_STATUS, &LIN_LH_EVNT_SPECIAL_CMD, &LIN_LH_EVNT_MASTER_CMD);
        break;

    default:
    	break;
        
    }
}

void changeState(HardStopState newState) 
{
//	printf("change Sequence: %d -> %d\n", currentState, newState);
	currentState = newState;
	handleStopState(currentState);
}
*/


void Evnt_LH_Special_Cmd_1(void)
{
	Amo_timer_Stop(timer_61);

	if(recoveryFlags.FR_CLLR_Recov_Flag == TRUE)
	{
		Lin_LH_HardStop_1_cllr(&LIN_LH_FR_STATUS, &LIN_LH_EVNT_SPECIAL_CMD, &LIN_LH_EVNT_MASTER_CMD);
	}
	if(recoveryFlags.FR_CLUD_Recov_Flag == TRUE)
	{
		Lin_LH_HardStop_1_clud(&LIN_LH_FR_STATUS, &LIN_LH_EVNT_SPECIAL_CMD, &LIN_LH_EVNT_MASTER_CMD);
	}
		if(recoveryFlags.FR_SLLR_Recov_Flag == TRUE)
	{
		Lin_LH_HardStop_1_sllr(&LIN_LH_FR_STATUS, &LIN_LH_EVNT_SPECIAL_CMD, &LIN_LH_EVNT_MASTER_CMD);
	}
	if(recoveryFlags.FR_SLUD_Recov_Flag == TRUE)
	{
		Lin_LH_HardStop_1_slud(&LIN_LH_FR_STATUS, &LIN_LH_EVNT_SPECIAL_CMD, &LIN_LH_EVNT_MASTER_CMD);
	}
	
}

void Evnt_RH_Special_Cmd_1(void)
{
	Amo_timer_Stop(timer_93);

	if(recoveryFlags.FR_CRLR_Recov_Flag == TRUE)
	{
		Lin_RH_HardStop_1_crlr(&LIN_RH_FR_STATUS, &LIN_RH_EVNT_SPECIAL_CMD, &LIN_RH_EVNT_MASTER_CMD);
	}
	if(recoveryFlags.FR_CRUD_Recov_Flag == TRUE)
	{
		Lin_RH_HardStop_1_crud(&LIN_RH_FR_STATUS, &LIN_RH_EVNT_SPECIAL_CMD, &LIN_RH_EVNT_MASTER_CMD);
	}
		if(recoveryFlags.FR_SRLR_Recov_Flag == TRUE)
	{
		Lin_RH_HardStop_1_srlr(&LIN_RH_FR_STATUS, &LIN_RH_EVNT_SPECIAL_CMD, &LIN_RH_EVNT_MASTER_CMD);
	}
	if(recoveryFlags.FR_SRUD_Recov_Flag == TRUE)
	{
		Lin_RH_HardStop_1_srud(&LIN_RH_FR_STATUS, &LIN_RH_EVNT_SPECIAL_CMD, &LIN_RH_EVNT_MASTER_CMD);
	}
	
}



void Lin_LH_HardStop_1_clud(t_lin_LH_FR_STATUS *lin_LH_Status, lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_LH_EVNT_MASTER_CMD_11 *lin_LH_Ctrl)	
{
	lin_SpecialCtrl->LDATA.EVNT_CMD = HARD_STOP_ACT;
	lin_SpecialCtrl->LDATA.EVNT_Broad = UNI_CAST;


	if(l_flg_tst_LI0_EVNT_SPECIAL_COMMAND_flag())
	{
		l_flg_clr_LI0_EVNT_SPECIAL_COMMAND_flag();
	
		specialCmdCount_clud++;
	
		if(recoveryFlags.FR_CLUD_Recov_Flag == TRUE)
		{
			lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_CLUD;

			l_u8_wr_LI0_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
			l_u8_wr_LI0_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
			l_bool_wr_LI0_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

			if(lastTryFlag_clud == TRUE)
			{
				FR_CLUD_FailSafe_Flag = TRUE;
				
				lin_LH_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = l_u16_rd_LI0_FR_CLUD_ActuatorState();
				l_u16_wr_LI0_EVNT_Front_Center_LH_UpDown_TargetPosition(lin_LH_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition);
			}
		}
	}
	if(specialCmdCount_clud != 2)
	{
//		Evnt_LH_Special_Cmd_1();
		
		Amo_timer_Start(timer_61, 50, false, Evnt_LH_Special_Cmd_1); //20
	}
		
	if(specialCmdCount_clud == 2)
	{
		specialCmdCount_clud = 0;
		
//		recoveryFlags.FR_CLLR_Recov_Flag = FALSE;

//		Lin_LH_Scheduler_Normal_Start_1();
		
		if(gotoZeroPtFlag_clud == TRUE)
		{
			//			goto 0 degree;
			Amo_timer_Stop(timer_98);
			Amo_timer_Start(timer_98, 50, false, RecoveryMode_LH_1_clud); //50 //1000
		}

		if(lastTryFlag_clud == TRUE)
		{		
			lastTryFlag_clud = FALSE;

			eventCountTest_clud = 0;
			recoveryPattern_clud = Recovery_None;
			
			recoveryFinishFlag_clud = TRUE;

			FR_DR_ExtFlt_SET();
			FR_PS_ExtFlt_SET();

			recoveryFlags.FR_CLUD_Recov_Flag = FALSE;

		}
		
		Amo_timer_Stop(timer_62);
		Amo_timer_Start(timer_62, 50, false, Lin_LH_Scheduler_Normal_Start_1); //20 //50 //500
	}

}


void Lin_LH_HardStop_1_cllr(t_lin_LH_FR_STATUS *lin_LH_Status, lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_LH_EVNT_MASTER_CMD_11 *lin_LH_Ctrl)	
{
	lin_SpecialCtrl->LDATA.EVNT_CMD = HARD_STOP_ACT;
	lin_SpecialCtrl->LDATA.EVNT_Broad = UNI_CAST;


	if(l_flg_tst_LI0_EVNT_SPECIAL_COMMAND_flag())
	{
		l_flg_clr_LI0_EVNT_SPECIAL_COMMAND_flag();
	
		specialCmdCount_cllr++;
	
		if(recoveryFlags.FR_CLLR_Recov_Flag == TRUE)
		{
			lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_CLLR;

			l_u8_wr_LI0_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
			l_u8_wr_LI0_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
			l_bool_wr_LI0_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

			if(lastTryFlag_cllr == TRUE)
			{
				FR_CLLR_FailSafe_Flag = TRUE;
				
				lin_LH_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = l_u16_rd_LI0_FR_CLLR_ActuatorState();
				l_u16_wr_LI0_EVNT_Front_Center_LH_LeftRight_TargetPosition(lin_LH_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition);
			}
		}
	}

	if(specialCmdCount_cllr != 2)
	{
//		Evnt_LH_Special_Cmd_1();
		
		Amo_timer_Start(timer_61, 50, false, Evnt_LH_Special_Cmd_1); //20
	}
		
	if(specialCmdCount_cllr == 2)
	{
		specialCmdCount_cllr = 0;
		
//		recoveryFlags.FR_CLLR_Recov_Flag = FALSE;

//		Lin_LH_Scheduler_Normal_Start_1();
		
		if(gotoZeroPtFlag_cllr == TRUE)
		{
			//			goto 0 degree;
			Amo_timer_Stop(timer_98);
			Amo_timer_Start(timer_98, 50, false, RecoveryMode_LH_1_cllr); //50 //1000
		}

		if(lastTryFlag_cllr == TRUE)
		{		
			lastTryFlag_cllr = FALSE;

			eventCountTest_cllr = 0;
			recoveryPattern_cllr = Recovery_None;
			
			recoveryFinishFlag_cllr = TRUE;

			FR_DR_ExtFlt_SET();
			FR_PS_ExtFlt_SET();

			recoveryFlags.FR_CLLR_Recov_Flag = FALSE;
		}

		Amo_timer_Stop(timer_62);
		Amo_timer_Start(timer_62, 50, false, Lin_LH_Scheduler_Normal_Start_1); //20 //50 //500

	}

}

void Lin_LH_HardStop_1_sllr(t_lin_LH_FR_STATUS *lin_LH_Status, lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_LH_EVNT_MASTER_CMD_11 *lin_LH_Ctrl)    
{
    lin_SpecialCtrl->LDATA.EVNT_CMD = HARD_STOP_ACT;
    lin_SpecialCtrl->LDATA.EVNT_Broad = UNI_CAST;

    if(l_flg_tst_LI0_EVNT_SPECIAL_COMMAND_flag())
    {
        l_flg_clr_LI0_EVNT_SPECIAL_COMMAND_flag();
    
        specialCmdCount_sllr++;

        if(recoveryFlags.FR_SLLR_Recov_Flag == TRUE)
        {
            lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_SLLR;

            l_u8_wr_LI0_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
            l_u8_wr_LI0_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
            l_bool_wr_LI0_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

            if(lastTryFlag_sllr == TRUE)
            {
                FR_SLLR_FailSafe_Flag = TRUE;
                
                lin_LH_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = l_u16_rd_LI0_FR_SLLR_ActuatorState();
                l_u16_wr_LI0_EVNT_Front_Side_LH_LeftRight_TargetPosition(lin_LH_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition);
            }
        }
    }

    if(specialCmdCount_sllr != 2)
    {
        Amo_timer_Start(timer_61, 50, false, Evnt_LH_Special_Cmd_1); //20
    }
        
    if(specialCmdCount_sllr == 2)
    {
        specialCmdCount_sllr = 0;
        
        if(gotoZeroPtFlag_sllr == TRUE)
        {
            Amo_timer_Stop(timer_98);
            Amo_timer_Start(timer_98, 50, false, RecoveryMode_LH_1_sllr); //50 //1000
        }

        if(lastTryFlag_sllr == TRUE)
        {        
            lastTryFlag_sllr = FALSE;

						eventCountTest_sllr = 0;
						recoveryPattern_sllr = Recovery_None;

            recoveryFinishFlag_sllr = TRUE;

            FR_DR_ExtFlt_SET();
            FR_PS_ExtFlt_SET();

            recoveryFlags.FR_SLLR_Recov_Flag = FALSE;
        }

        Amo_timer_Stop(timer_62);
        Amo_timer_Start(timer_62, 50, false, Lin_LH_Scheduler_Normal_Start_1); //20 //50 //500
    }
}

void Lin_LH_HardStop_1_slud(t_lin_LH_FR_STATUS *lin_LH_Status, lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_LH_EVNT_MASTER_CMD_11 *lin_LH_Ctrl)    
{
    lin_SpecialCtrl->LDATA.EVNT_CMD = HARD_STOP_ACT;
    lin_SpecialCtrl->LDATA.EVNT_Broad = UNI_CAST;

    if(l_flg_tst_LI0_EVNT_SPECIAL_COMMAND_flag())
    {
        l_flg_clr_LI0_EVNT_SPECIAL_COMMAND_flag();
    
        specialCmdCount_slud++;

        if(recoveryFlags.FR_SLUD_Recov_Flag == TRUE)
        {
            lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_SLUD;

            l_u8_wr_LI0_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
            l_u8_wr_LI0_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
            l_bool_wr_LI0_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

            if(lastTryFlag_slud == TRUE)
            {
                FR_SLUD_FailSafe_Flag = TRUE;
                
                lin_LH_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = l_u16_rd_LI0_FR_SLUD_ActuatorState();
                l_u16_wr_LI0_EVNT_Front_Side_LH_UpDown_TargetPosition(lin_LH_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition);
            }
        }
    }

    if(specialCmdCount_slud != 2)
    {
        Amo_timer_Start(timer_61, 50, false, Evnt_LH_Special_Cmd_1); //20
    }
        
    if(specialCmdCount_slud == 2)
    {
        specialCmdCount_slud = 0;
        
        if(gotoZeroPtFlag_slud == TRUE)
        {
            Amo_timer_Stop(timer_98);
            Amo_timer_Start(timer_98, 50, false, RecoveryMode_LH_1_slud); //50 //1000
        }

        if(lastTryFlag_slud == TRUE)
        {        
            lastTryFlag_slud = FALSE;

						eventCountTest_slud = 0;
						recoveryPattern_slud = Recovery_None;

            recoveryFinishFlag_slud = TRUE;

            FR_DR_ExtFlt_SET();
            FR_PS_ExtFlt_SET();

            recoveryFlags.FR_SLUD_Recov_Flag = FALSE;
        }

        Amo_timer_Stop(timer_62);
        Amo_timer_Start(timer_62, 50, false, Lin_LH_Scheduler_Normal_Start_1); //20 //50 //500
    }
}

void Lin_RH_HardStop_1_crlr(t_lin_RH_FR_STATUS *lin_RH_Status, lin_RH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_RH_EVNT_MASTER_CMD_21 *lin_RH_Ctrl)    
{
    lin_SpecialCtrl->LDATA.EVNT_CMD = HARD_STOP_ACT;
    lin_SpecialCtrl->LDATA.EVNT_Broad = UNI_CAST;

    if(l_flg_tst_LI1_EVNT_SPECIAL_COMMAND_flag())
    {
        l_flg_clr_LI1_EVNT_SPECIAL_COMMAND_flag();
    
        specialCmdCount_crlr++;

        if(recoveryFlags.FR_CRLR_Recov_Flag == TRUE)
        {
            lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_CRLR;

            l_u8_wr_LI1_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
            l_u8_wr_LI1_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
            l_bool_wr_LI1_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

            if(lastTryFlag_crlr == TRUE)
            {
                FR_CRLR_FailSafe_Flag = TRUE;
                
                lin_RH_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = l_u16_rd_LI1_FR_CRLR_ActuatorState();
                l_u16_wr_LI1_EVNT_Front_Center_RH_LeftRight_TargetPosition(lin_RH_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition);
            }
        }
    }

    if(specialCmdCount_crlr != 2)
    {
        Amo_timer_Start(timer_93, 50, false, Evnt_RH_Special_Cmd_1); //20
    }
        
    if(specialCmdCount_crlr == 2)
    {
        specialCmdCount_crlr = 0;
        
        if(gotoZeroPtFlag_crlr == TRUE)
        {
            Amo_timer_Stop(timer_123);
            Amo_timer_Start(timer_123, 50, false, RecoveryMode_RH_1_crlr); //50 //1000
        }

        if(lastTryFlag_crlr == TRUE)
        {        
            lastTryFlag_crlr = FALSE;

						eventCountTest_crlr = 0;
						recoveryPattern_crlr = Recovery_None;

            recoveryFinishFlag_crlr = TRUE;

            FR_DR_ExtFlt_SET();
            FR_PS_ExtFlt_SET();

            recoveryFlags.FR_CRLR_Recov_Flag = FALSE;
        }

        Amo_timer_Stop(timer_124);
        Amo_timer_Start(timer_124, 50, false, Lin_RH_Scheduler_Normal_Start_1); //20 //50 //500
    }
}

void Lin_RH_HardStop_1_crud(t_lin_RH_FR_STATUS *lin_RH_Status, lin_RH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_RH_EVNT_MASTER_CMD_21 *lin_RH_Ctrl)    
{
    lin_SpecialCtrl->LDATA.EVNT_CMD = HARD_STOP_ACT;
    lin_SpecialCtrl->LDATA.EVNT_Broad = UNI_CAST;

    if(l_flg_tst_LI1_EVNT_SPECIAL_COMMAND_flag())
    {
        l_flg_clr_LI1_EVNT_SPECIAL_COMMAND_flag();
    
        specialCmdCount_crud++;

        if(recoveryFlags.FR_CRUD_Recov_Flag == TRUE)
        {
            lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_CRUD;

            l_u8_wr_LI1_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
            l_u8_wr_LI1_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
            l_bool_wr_LI1_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

            if(lastTryFlag_crud == TRUE)
            {
                FR_CLUD_FailSafe_Flag = TRUE;
                
                lin_RH_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = l_u16_rd_LI1_FR_CRUD_ActuatorState();
                l_u16_wr_LI1_EVNT_Front_Center_RH_UpDown_TargetPosition(lin_RH_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition);
            }
        }
    }

    if(specialCmdCount_crud != 2)
    {
        Amo_timer_Start(timer_93, 50, false, Evnt_RH_Special_Cmd_1); //20
    }
        
    if(specialCmdCount_crud == 2)
    {
        specialCmdCount_crud = 0;
        
        if(gotoZeroPtFlag_crud == TRUE)
        {
            Amo_timer_Stop(timer_123);
            Amo_timer_Start(timer_123, 50, false, RecoveryMode_RH_1_crud); //50 //1000
        }

        if(lastTryFlag_crud == TRUE)
        {        
            lastTryFlag_crud = FALSE;

						eventCountTest_crud = 0;
						recoveryPattern_crud = Recovery_None;

            recoveryFinishFlag_crud = TRUE;

            FR_DR_ExtFlt_SET();
            FR_PS_ExtFlt_SET();

            recoveryFlags.FR_CRUD_Recov_Flag = FALSE;
        }

        Amo_timer_Stop(timer_124);
        Amo_timer_Start(timer_124, 50, false, Lin_RH_Scheduler_Normal_Start_1); //20 //50 //500
    }
}

void Lin_RH_HardStop_1_srlr(t_lin_RH_FR_STATUS *lin_RH_Status, lin_RH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_RH_EVNT_MASTER_CMD_21 *lin_RH_Ctrl)    
{
    lin_SpecialCtrl->LDATA.EVNT_CMD = HARD_STOP_ACT;
    lin_SpecialCtrl->LDATA.EVNT_Broad = UNI_CAST;

    if(l_flg_tst_LI1_EVNT_SPECIAL_COMMAND_flag())
    {
        l_flg_clr_LI1_EVNT_SPECIAL_COMMAND_flag();
    
        specialCmdCount_srlr++;

        if(recoveryFlags.FR_SRLR_Recov_Flag == TRUE)
        {
            lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_SRLR;

            l_u8_wr_LI1_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
            l_u8_wr_LI1_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
            l_bool_wr_LI1_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

            if(lastTryFlag_srlr == TRUE)
            {
                FR_SLLR_FailSafe_Flag = TRUE;
                
                lin_RH_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = l_u16_rd_LI1_FR_SRLR_ActuatorState();
                l_u16_wr_LI1_EVNT_Front_Side_RH_LeftRight_TargetPosition(lin_RH_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition);
            }
        }
    }

    if(specialCmdCount_srlr != 2)
    {
        Amo_timer_Start(timer_93, 50, false, Evnt_RH_Special_Cmd_1); //20
    }
        
    if(specialCmdCount_srlr == 2)
    {
        specialCmdCount_srlr = 0;
        
        if(gotoZeroPtFlag_srlr == TRUE)
        {
            Amo_timer_Stop(timer_123);
            Amo_timer_Start(timer_123, 50, false, RecoveryMode_RH_1_srlr); //50 //1000
        }

        if(lastTryFlag_srlr == TRUE)
        {        
            lastTryFlag_srlr = FALSE;

						eventCountTest_srlr = 0;
						recoveryPattern_srlr = Recovery_None;

            recoveryFinishFlag_srlr = TRUE;

            FR_DR_ExtFlt_SET();
            FR_PS_ExtFlt_SET();

            recoveryFlags.FR_SRLR_Recov_Flag = FALSE;
        }

        Amo_timer_Stop(timer_124);
        Amo_timer_Start(timer_124, 50, false, Lin_RH_Scheduler_Normal_Start_1); //20 //50 //500
    }
}

void Lin_RH_HardStop_1_srud(t_lin_RH_FR_STATUS *lin_RH_Status, lin_RH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_RH_EVNT_MASTER_CMD_21 *lin_RH_Ctrl)    
{
    lin_SpecialCtrl->LDATA.EVNT_CMD = HARD_STOP_ACT;
    lin_SpecialCtrl->LDATA.EVNT_Broad = UNI_CAST;

    if(l_flg_tst_LI1_EVNT_SPECIAL_COMMAND_flag())
    {
        l_flg_clr_LI1_EVNT_SPECIAL_COMMAND_flag();
    
        specialCmdCount_srud++;

        if(recoveryFlags.FR_SRUD_Recov_Flag == TRUE)
        {
            lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_SRUD;

            l_u8_wr_LI1_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
            l_u8_wr_LI1_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
            l_bool_wr_LI1_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

            if(lastTryFlag_srud == TRUE)
            {
                FR_SRUD_FailSafe_Flag = TRUE;
                
                lin_RH_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = l_u16_rd_LI1_FR_SRUD_ActuatorState();
                l_u16_wr_LI1_EVNT_Front_Side_RH_UpDown_TargetPosition(lin_RH_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition);
            }
        }
    }

    if(specialCmdCount_srud != 2)
    {
        Amo_timer_Start(timer_93, 50, false, Evnt_RH_Special_Cmd_1); //20
    }
        
    if(specialCmdCount_srud == 2)
    {
        specialCmdCount_srud = 0;
        
        if(gotoZeroPtFlag_srud == TRUE)
        {
            Amo_timer_Stop(timer_123);
            Amo_timer_Start(timer_123, 50, false, RecoveryMode_RH_1_srud); //50 //1000
        }

        if(lastTryFlag_srud == TRUE)
        {        
            lastTryFlag_srud = FALSE;

						eventCountTest_srud = 0;
						recoveryPattern_srud = Recovery_None;

            recoveryFinishFlag_srud = TRUE;

            FR_DR_ExtFlt_SET();
            FR_PS_ExtFlt_SET();

            recoveryFlags.FR_SRUD_Recov_Flag = FALSE;
        }

        Amo_timer_Stop(timer_124);
        Amo_timer_Start(timer_124, 50, false, Lin_RH_Scheduler_Normal_Start_1); //20 //50 //500
    }
}


void Lin_Scheduler_SpecialCmd_Start_1(void)
{
	if(recoveryFlags.FR_SLLR_Recov_Flag == TRUE || recoveryFlags.FR_SLUD_Recov_Flag == TRUE || recoveryFlags.FR_CLLR_Recov_Flag == TRUE || recoveryFlags.FR_CLUD_Recov_Flag == TRUE)
	{
		if(recoveryFlags.FR_CLLR_Recov_Flag == TRUE)
		{
			FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_CLLR_ExtFlt = TRUE;
		}
		if(recoveryFlags.FR_CLUD_Recov_Flag == TRUE)
		{
			FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_CLUD_ExtFlt = TRUE;
		}
		if(recoveryFlags.FR_SLLR_Recov_Flag == TRUE)
		{
			FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_SLLR_ExtFlt = TRUE;
		}
		if(recoveryFlags.FR_SLUD_Recov_Flag == TRUE)
		{
			FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_SLUD_ExtFlt = TRUE;
		}
		
		l_sch_set(LI0, LI0_SCHEDULER_EVNT_SPECIAL_CMD_FR_DRV, 0u);

		Amo_timer_Stop(timer_61);
		Amo_timer_Start(timer_61, 50, false, Evnt_LH_Special_Cmd_1); //20	
	}

	if(recoveryFlags.FR_SRLR_Recov_Flag == TRUE || recoveryFlags.FR_SRUD_Recov_Flag == TRUE || recoveryFlags.FR_CRLR_Recov_Flag == TRUE || recoveryFlags.FR_CRUD_Recov_Flag == TRUE)
	{
		if(recoveryFlags.FR_CRLR_Recov_Flag == TRUE)
		{
			FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_CRLR_ExtFlt = TRUE;
		}
		if(recoveryFlags.FR_CRUD_Recov_Flag == TRUE)
		{
			FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_CRUD_ExtFlt = TRUE;
		}
		if(recoveryFlags.FR_SRLR_Recov_Flag == TRUE)
		{
			FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_SRLR_ExtFlt = TRUE;
		}
		if(recoveryFlags.FR_SRUD_Recov_Flag == TRUE)
		{
			FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_SRUD_ExtFlt = TRUE;
		}
		
		l_sch_set(LI1, LI1_SCHEDULER_EVNT_SPECIAL_CMD_FR_PASS, 0u);

		Amo_timer_Stop(timer_93);
		Amo_timer_Start(timer_93, 50, false, Evnt_RH_Special_Cmd_1); //20	
	}
}


void goToTargetPt_OpDone_Check_sllr(void)
{
    if(LIN_LH_FR_STATUS.fr_sllr_status.LDATA.FR_SLLR_OpDone == 0) 
    { 
			Check300ms_flag_1_sllr = FALSE;
			
      goToTargetFlag_sllr = TRUE; 
      Amo_timer_Stop(timer_92);
    }
}

void goToTargetPt_OpDone_Check_slud(void)
{
    if(LIN_LH_FR_STATUS.fr_slud_status.LDATA.FR_SLUD_OpDone == 0) 
    { 
			Check300ms_flag_1_slud = FALSE;
			
      goToTargetFlag_slud = TRUE; 
      Amo_timer_Stop(timer_118);
    }
}

void goToTargetPt_OpDone_Check_cllr(void)
{
	if(LIN_LH_FR_STATUS.fr_cllr_status.LDATA.FR_CLLR_OpDone == 0) 
		{ 
			Check300ms_flag_1_cllr = FALSE;
			
			goToTargetFlag_cllr = TRUE; 
			Amo_timer_Stop(timer_95);
		}
}

void goToTargetPt_OpDone_Check_clud(void)
{
	if(LIN_LH_FR_STATUS.fr_clud_status.LDATA.FR_CLUD_OpDone == 0) 
		{ 
			Check300ms_flag_1_clud = FALSE;
			
			goToTargetFlag_clud = TRUE; 
			Amo_timer_Stop(timer_99);
		}
}

void goToTargetPt_OpDone_Check_crlr(void)
{
    if(LIN_RH_FR_STATUS.fr_crlr_status.LDATA.FR_CRLR_OpDone == 0) 
    { 
			Check300ms_flag_1_crlr = FALSE;
			
      goToTargetFlag_crlr = TRUE; 
      Amo_timer_Stop(timer_119);
    }
}

void goToTargetPt_OpDone_Check_crud(void)
{
    if(LIN_RH_FR_STATUS.fr_crud_status.LDATA.FR_CRUD_OpDone == 0) 
    { 
			Check300ms_flag_1_crud = FALSE;
			
      goToTargetFlag_crud = TRUE; 
      Amo_timer_Stop(timer_120);
    }
}

void goToTargetPt_OpDone_Check_srlr(void)
{
    if(LIN_RH_FR_STATUS.fr_srlr_status.LDATA.FR_SRLR_OpDone == 0) 
    { 
			Check300ms_flag_1_srlr = FALSE;
			
      goToTargetFlag_srlr = TRUE; 
      Amo_timer_Stop(timer_121);
    }
}

void goToTargetPt_OpDone_Check_srud(void)
{
    if(LIN_RH_FR_STATUS.fr_srud_status.LDATA.FR_SRUD_OpDone == 0) 
    { 
			Check300ms_flag_1_srud = FALSE;
			
      goToTargetFlag_srud = TRUE; 
      Amo_timer_Stop(timer_122);
    }
}



void Recovery_goToTargetPt_FR_SLLR_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
	if((lin_Pt->FrDrPt.prevPt_FR_SLLR) != (lin_Pt->FrDrPt.currentPt_FR_SLLR))
	{
		SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_2);

		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = lin_Pt->FrDrPt.prevPt_FR_SLLR; //290;

		lin_Pt->FrDrPt.prevPt_FR_SLLR = lin_Pt->FrDrPt.currentPt_FR_SLLR;

		lin_Pt->FrDrPt.currentPt_FR_SLLR = lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;

		lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

		Amo_timer_Stop(timer_92);
		Amo_timer_Start(timer_92, 300, true, goToTargetPt_OpDone_Check_sllr); //50 //1000

		if(Steploss_OpDone_TimeOutFlag_sllr == TRUE)
		{
			Steploss_OpDone_TimeOutFlag_sllr = FALSE;
			
			Amo_timer_Stop(timer_130);
			Amo_timer_Start(timer_130, 8000, false, ClearState_Steploss_OpDone_Check_sllr);
		}

	}
}

void Recovery_goToTargetPt_FR_SLUD_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
    if((lin_Pt->FrDrPt.prevPt_FR_SLUD) != (lin_Pt->FrDrPt.currentPt_FR_SLUD))
    {
        SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_2);

        lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = lin_Pt->FrDrPt.prevPt_FR_SLUD; //290;

        lin_Pt->FrDrPt.prevPt_FR_SLUD = lin_Pt->FrDrPt.currentPt_FR_SLUD;

        lin_Pt->FrDrPt.currentPt_FR_SLUD = lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;

        lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

        Amo_timer_Stop(timer_118);
				Amo_timer_Start(timer_118, 300, true, goToTargetPt_OpDone_Check_slud); //50 //1000

				if(Steploss_OpDone_TimeOutFlag_slud == TRUE)
				{
					Steploss_OpDone_TimeOutFlag_slud = FALSE;
					
					Amo_timer_Stop(timer_129);
					Amo_timer_Start(timer_129, 8000, false, ClearState_Steploss_OpDone_Check_slud);
				}
    }
}


void Recovery_goToTargetPt_FR_CLLR_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
	if((lin_Pt->FrDrPt.prevPt_FR_CLLR) != (lin_Pt->FrDrPt.currentPt_FR_CLLR))
	{
		SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_2);

		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = lin_Pt->FrDrPt.prevPt_FR_CLLR; //290;

		lin_Pt->FrDrPt.prevPt_FR_CLLR = lin_Pt->FrDrPt.currentPt_FR_CLLR;

		lin_Pt->FrDrPt.currentPt_FR_CLLR = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;

		lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

		Amo_timer_Stop(timer_95);
		Amo_timer_Start(timer_95, 300, true, goToTargetPt_OpDone_Check_cllr); //50 //1000

		if(Steploss_OpDone_TimeOutFlag_cllr == TRUE)
		{
			Steploss_OpDone_TimeOutFlag_cllr = FALSE;
			
			Amo_timer_Stop(timer_132);
			Amo_timer_Start(timer_132, 8000, false, ClearState_Steploss_OpDone_Check_cllr);
		}

	}
}


void Recovery_goToTargetPt_FR_CLUD_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
	if((lin_Pt->FrDrPt.prevPt_FR_CLUD) != (lin_Pt->FrDrPt.currentPt_FR_CLUD))
	{
		SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_2);

		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = lin_Pt->FrDrPt.prevPt_FR_CLUD; //290;

		lin_Pt->FrDrPt.prevPt_FR_CLUD = lin_Pt->FrDrPt.currentPt_FR_CLUD;

		lin_Pt->FrDrPt.currentPt_FR_CLUD = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;

		lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

		Amo_timer_Stop(timer_99);
		Amo_timer_Start(timer_99, 300, true, goToTargetPt_OpDone_Check_clud); //50 //1000

		if(Steploss_OpDone_TimeOutFlag_clud == TRUE)
		{
			Steploss_OpDone_TimeOutFlag_clud = FALSE;
			
			Amo_timer_Stop(timer_131);
			Amo_timer_Start(timer_131, 8000, false, ClearState_Steploss_OpDone_Check_clud);
		}

	}
}

void Recovery_goToTargetPt_FR_CRLR_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
    if((lin_Pt->FrPsPt.prevPt_FR_CRLR) != (lin_Pt->FrPsPt.currentPt_FR_CRLR))
    {
        SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_2);

        lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = lin_Pt->FrPsPt.prevPt_FR_CRLR; //290;

        lin_Pt->FrPsPt.prevPt_FR_CRLR = lin_Pt->FrPsPt.currentPt_FR_CRLR;

        lin_Pt->FrPsPt.currentPt_FR_CRLR = lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;

        lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

       	Amo_timer_Stop(timer_119);
				Amo_timer_Start(timer_119, 300, true, goToTargetPt_OpDone_Check_crlr); //50 //1000

				if(Steploss_OpDone_TimeOutFlag_crlr == TRUE)
				{
					Steploss_OpDone_TimeOutFlag_crlr = FALSE;
					
					Amo_timer_Stop(timer_128);
					Amo_timer_Start(timer_128, 8000, false, ClearState_Steploss_OpDone_Check_crlr);
				}
    }
}

void Recovery_goToTargetPt_FR_CRUD_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
    if((lin_Pt->FrPsPt.prevPt_FR_CRUD) != (lin_Pt->FrPsPt.currentPt_FR_CRUD))
    {
        SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_2);

        lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = lin_Pt->FrPsPt.prevPt_FR_CRUD; //290;

        lin_Pt->FrPsPt.prevPt_FR_CRUD = lin_Pt->FrPsPt.currentPt_FR_CRUD;

        lin_Pt->FrPsPt.currentPt_FR_CRUD = lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;

        lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

       	Amo_timer_Stop(timer_120);
				Amo_timer_Start(timer_120, 300, true, goToTargetPt_OpDone_Check_crud); //50 //1000

				if(Steploss_OpDone_TimeOutFlag_crud == TRUE)
				{
					Steploss_OpDone_TimeOutFlag_srud = FALSE;
					
					Amo_timer_Stop(timer_127);
					Amo_timer_Start(timer_127, 8000, false, ClearState_Steploss_OpDone_Check_crud);
				}
    }
}

void Recovery_goToTargetPt_FR_SRLR_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
    if((lin_Pt->FrPsPt.prevPt_FR_SRLR) != (lin_Pt->FrPsPt.currentPt_FR_SRLR))
    {
        SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_2);

        lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = lin_Pt->FrPsPt.prevPt_FR_SRLR; //290;

        lin_Pt->FrPsPt.prevPt_FR_SRLR = lin_Pt->FrPsPt.currentPt_FR_SRLR;

        lin_Pt->FrPsPt.currentPt_FR_SRLR = lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;

        lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

       	Amo_timer_Stop(timer_121);
				Amo_timer_Start(timer_121, 300, true, goToTargetPt_OpDone_Check_srlr); //50 //1000

				if(Steploss_OpDone_TimeOutFlag_srlr == TRUE)
				{
					Steploss_OpDone_TimeOutFlag_srlr = FALSE;
					
					Amo_timer_Stop(timer_126);
					Amo_timer_Start(timer_126, 8000, false, ClearState_Steploss_OpDone_Check_srlr);
				}
    }
}

void Recovery_goToTargetPt_FR_SRUD_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
    if((lin_Pt->FrPsPt.prevPt_FR_SRUD) != (lin_Pt->FrPsPt.currentPt_FR_SRUD))
    {
        SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_2);

        lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = lin_Pt->FrPsPt.prevPt_FR_SRUD; //290;

        lin_Pt->FrPsPt.prevPt_FR_SRUD = lin_Pt->FrPsPt.currentPt_FR_SRUD;

        lin_Pt->FrPsPt.currentPt_FR_SRUD = lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;

        lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

       	Amo_timer_Stop(timer_122);
				Amo_timer_Start(timer_122, 300, true, goToTargetPt_OpDone_Check_srud); //50 //1000

				if(Steploss_OpDone_TimeOutFlag_srud == TRUE)
				{
					Steploss_OpDone_TimeOutFlag_srud = FALSE;
					
					Amo_timer_Stop(timer_125);
					Amo_timer_Start(timer_125, 8000, false, ClearState_Steploss_OpDone_Check_srud);
				}
    }
}

void ClearState_Steploss_OpDone_Check_sllr(void)
{
		Amo_timer_Stop(timer_130);
		
		LIN_LH_FR_STATUS.fr_sllr_status.LDATA.FR_SLLR_Steploss = l_bool_rd_LI0_FR_SLLR_Steploss();
		LIN_LH_FR_STATUS.fr_sllr_status.LDATA.FR_SLLR_OpDone = l_bool_rd_LI0_FR_SLLR_OpDone();
		
    if(LIN_LH_FR_STATUS.fr_sllr_status.LDATA.FR_SLLR_OpDone == 0 && LIN_LH_FR_STATUS.fr_sllr_status.LDATA.FR_SLLR_Steploss == 1) 
    { 
      lastTryFlag_sllr = TRUE;
      recoveryFlags.FR_SLLR_Recov_Flag = TRUE;

      Lin_Scheduler_SpecialCmd_Start_1();
    }
}

void ClearState_Steploss_OpDone_Check_slud(void)
{
		Amo_timer_Stop(timer_129);
		
		LIN_LH_FR_STATUS.fr_slud_status.LDATA.FR_SLUD_Steploss = l_bool_rd_LI0_FR_SLUD_Steploss();
		LIN_LH_FR_STATUS.fr_slud_status.LDATA.FR_SLUD_OpDone = l_bool_rd_LI0_FR_SLUD_OpDone();
		
    if(LIN_LH_FR_STATUS.fr_slud_status.LDATA.FR_SLUD_OpDone == 0 && LIN_LH_FR_STATUS.fr_slud_status.LDATA.FR_SLUD_Steploss == 1) 
    { 
      lastTryFlag_slud = TRUE;
      recoveryFlags.FR_SLUD_Recov_Flag = TRUE;

      Lin_Scheduler_SpecialCmd_Start_1();
    }
}

void ClearState_Steploss_OpDone_Check_cllr(void)
{
		Amo_timer_Stop(timer_132);
		
		LIN_LH_FR_STATUS.fr_cllr_status.LDATA.FR_CLLR_Steploss = l_bool_rd_LI0_FR_CLLR_Steploss();
		LIN_LH_FR_STATUS.fr_cllr_status.LDATA.FR_CLLR_OpDone = l_bool_rd_LI0_FR_CLLR_OpDone();
		
    if(LIN_LH_FR_STATUS.fr_cllr_status.LDATA.FR_CLLR_OpDone == 0 && LIN_LH_FR_STATUS.fr_cllr_status.LDATA.FR_CLLR_Steploss == 1) 
    { 
      lastTryFlag_cllr = TRUE;
      recoveryFlags.FR_CLLR_Recov_Flag = TRUE;

      Lin_Scheduler_SpecialCmd_Start_1();
    }
}


void ClearState_Steploss_OpDone_Check_clud(void)
{
		Amo_timer_Stop(timer_131);
		
		LIN_LH_FR_STATUS.fr_clud_status.LDATA.FR_CLUD_Steploss = l_bool_rd_LI0_FR_CLUD_Steploss();
		LIN_LH_FR_STATUS.fr_clud_status.LDATA.FR_CLUD_OpDone = l_bool_rd_LI0_FR_CLUD_OpDone();
		
    if(LIN_LH_FR_STATUS.fr_clud_status.LDATA.FR_CLUD_OpDone == 0 && LIN_LH_FR_STATUS.fr_clud_status.LDATA.FR_CLUD_Steploss == 1) 
    { 
      lastTryFlag_clud = TRUE;
      recoveryFlags.FR_CLUD_Recov_Flag = TRUE;

      Lin_Scheduler_SpecialCmd_Start_1();
    }
}

void ClearState_Steploss_OpDone_Check_crlr(void)
{
		Amo_timer_Stop(timer_128);
		
		LIN_RH_FR_STATUS.fr_crlr_status.LDATA.FR_CRLR_Steploss = l_bool_rd_LI1_FR_CRLR_Steploss();
		LIN_RH_FR_STATUS.fr_crlr_status.LDATA.FR_CRLR_OpDone = l_bool_rd_LI1_FR_CRLR_OpDone();
		
    if(LIN_RH_FR_STATUS.fr_crlr_status.LDATA.FR_CRLR_OpDone == 0 && LIN_RH_FR_STATUS.fr_crlr_status.LDATA.FR_CRLR_Steploss == 1) 
    { 
      lastTryFlag_crlr = TRUE;
      recoveryFlags.FR_CRLR_Recov_Flag = TRUE;

      Lin_Scheduler_SpecialCmd_Start_1();
    }
}


void ClearState_Steploss_OpDone_Check_crud(void)
{
		Amo_timer_Stop(timer_127);
		
		LIN_RH_FR_STATUS.fr_crud_status.LDATA.FR_CRUD_Steploss = l_bool_rd_LI1_FR_CRUD_Steploss();
		LIN_RH_FR_STATUS.fr_crud_status.LDATA.FR_CRUD_OpDone = l_bool_rd_LI1_FR_CRUD_OpDone();
		
    if(LIN_RH_FR_STATUS.fr_crud_status.LDATA.FR_CRUD_OpDone == 0 && LIN_RH_FR_STATUS.fr_crud_status.LDATA.FR_CRUD_Steploss == 1) 
    { 
      lastTryFlag_crud = TRUE;
      recoveryFlags.FR_CRUD_Recov_Flag = TRUE;

      Lin_Scheduler_SpecialCmd_Start_1();
    }
}

void ClearState_Steploss_OpDone_Check_srlr(void)
{
		Amo_timer_Stop(timer_126);
		
		LIN_RH_FR_STATUS.fr_srlr_status.LDATA.FR_SRLR_Steploss = l_bool_rd_LI1_FR_SRLR_Steploss();
		LIN_RH_FR_STATUS.fr_srlr_status.LDATA.FR_SRLR_OpDone = l_bool_rd_LI1_FR_SRLR_OpDone();
		
    if(LIN_RH_FR_STATUS.fr_srlr_status.LDATA.FR_SRLR_OpDone == 0 && LIN_RH_FR_STATUS.fr_srlr_status.LDATA.FR_SRLR_Steploss == 1) 
    { 
      lastTryFlag_srlr = TRUE;
      recoveryFlags.FR_SRLR_Recov_Flag = TRUE;

      Lin_Scheduler_SpecialCmd_Start_1();
    }
}

void ClearState_Steploss_OpDone_Check_srud(void)
{
		Amo_timer_Stop(timer_125);
		
		LIN_RH_FR_STATUS.fr_srud_status.LDATA.FR_SRUD_Steploss = l_bool_rd_LI1_FR_SRUD_Steploss();
		LIN_RH_FR_STATUS.fr_srud_status.LDATA.FR_SRUD_OpDone = l_bool_rd_LI1_FR_SRUD_OpDone();
		
    if(LIN_RH_FR_STATUS.fr_srud_status.LDATA.FR_SRUD_OpDone == 0 && LIN_RH_FR_STATUS.fr_srud_status.LDATA.FR_SRUD_Steploss == 1) 
    { 
      lastTryFlag_srud = TRUE;
      recoveryFlags.FR_SRUD_Recov_Flag = TRUE;

      Lin_Scheduler_SpecialCmd_Start_1();
    }
}



void Recovery_Clear_goToTargetPt_FR_SRUD_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
    if((lin_Pt->FrPsPt.prevPt_FR_SRUD) != (lin_Pt->FrPsPt.currentPt_FR_SRUD))
    {
        SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_2);

        lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = lin_Pt->FrPsPt.prevPt_FR_SRUD; //290;

        lin_Pt->FrPsPt.prevPt_FR_SRUD = lin_Pt->FrPsPt.currentPt_FR_SRUD;

        lin_Pt->FrPsPt.currentPt_FR_SRUD = lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;

        lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

       	Amo_timer_Stop(timer_125);
				Amo_timer_Start(timer_125, 5000, false, ClearState_Steploss_OpDone_Check_srud); //50 //1000
    }
}




/*
void Check_300ms(void)
{

//	PINS_DRV_ClearPins(GPIO_PORTA, TEST_OUT1_MASK);

//	Check300ms_flag = 0;
	
	Amo_timer_Stop(timer_91);
	
	if(steplossCount >= 6)
	{
//		PINS_DRV_ClearPins(GPIO_PORTA, TEST_OUT1_MASK);

//		recoveryModeFlag = 1;
		
		steplossCount = 0;
		
//		Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);

		if(gotoTargetPtCount < 3)
		{	
//			hardStop;
			recoveryFlags.FR_CLLR_Recov_Flag = TRUE;
//			recovDoneFlag = TRUE;

			gotoZeroPtFlag = 1;
			gotoTargetPtFlag = 0;
			
			Lin_Scheduler_SpecialCmd_Start_1();

//			goto 0 degree;
//			Amo_timer_Stop(timer_82);
//			Amo_timer_Start(timer_82, 100, false, RecoveryMode_LH_1); //50 //1000
		}
		else
		{
//			hardStop;

			Check300ms_flag = 0;

			gotoTargetPtCount = 0;

			gotoZeroPtFlag = 0;
			gotoTargetPtFlag = 0;

			recoveryFlags.FR_CLLR_Recov_Flag = TRUE;
//			recovDoneFlag = TRUE;
			Lin_Scheduler_SpecialCmd_Start_1();
		}
		
	}
	else
	{
		steplossCount = 0;
	}
	
}
*/

/*
void Steploss_check(t_lin_LH_FR_STATUS *lin_Status)
{
	
	if(l_flg_tst_LI0_FR_CLLR_Steploss_flag())
	{
		l_flg_clr_LI0_FR_CLLR_Steploss_flag();

		lin_Status->fr_cllr_status.LDATA.FR_CLLR_Steploss = l_bool_rd_LI0_FR_CLLR_Steploss();
		lin_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone = l_bool_rd_LI0_FR_CLLR_OpDone();

		if(lin_Status->fr_cllr_status.LDATA.FR_CLLR_Steploss == 1 && lin_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone == 0)
		{
			steplossCount++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6

			if(Check300ms_flag == 0)
			{
				
//				PINS_DRV_SetPins(GPIO_PORTA, TEST_OUT1_MASK);
				
				Check300ms_flag = 1;
				
				Amo_timer_Stop(timer_91);
				Amo_timer_Start(timer_91, 300, false, Check_300ms);
			}
		}
		else if(lin_Status->fr_cllr_status.LDATA.FR_CLLR_Steploss == 0 && recoveryModeFlag == 1)
		{

			lin_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone = l_bool_rd_LI0_FR_CLLR_OpDone();
			zeroPt = l_u16_rd_LI0_FR_CLLR_ActuatorState();

//			if((zeroPt > 0 && zeroPt < 10) || (zeroPt > 2038 && zeroPt < 2048))
				
			if((lin_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone == true) && ((zeroPt >= 0 && zeroPt < 10) || (zeroPt > 2038 && zeroPt < 2048)))
			{
//				Check300ms_flag = 0;

				recoveryModeFlag = 0;
								
				gotoTargetPtCount++;


//				hardStop;
				recoveryFlags.FR_CLLR_Recov_Flag = TRUE;
//				recovDoneFlag = TRUE;

				gotoZeroPtFlag = 0;
				gotoTargetPtFlag = 1;
				
//				Lin_Scheduler_SpecialCmd_Start_tPT();
				
				Recovery_goToTargetPt_FR_CLLR_1(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);

//				goto target degree;
//				Amo_timer_Stop(timer_85);
//				Amo_timer_Start(timer_85, 100, false, Recovery_goToTargetPt); //50
				
			}
		}
	}

}
*/

void Check_300ms_cllr(void)
{

//	PINS_DRV_ClearPins(GPIO_PORTC, TEST_OUT1_MASK);
	
	Amo_timer_Stop(timer_91);
	
	if(steplossCount_cllr >= 6)
	{	
		if(cycleModeFlag_cllr == TRUE) { lin_FrDrCycleMode_TimerStop(); }
	
		steplossCount_cllr = 0;
		
		recoveryFlags.FR_CLLR_Recov_Flag = TRUE;

		gotoZeroPtFlag_cllr = TRUE;
	
		Lin_Scheduler_SpecialCmd_Start_1();

	}
	else
	{
		Check300ms_flag_cllr = FALSE;
		steplossCount_cllr = 0;
	}
	
}

void Check_300ms_clud(void)
{

	Amo_timer_Stop(timer_96);
	
	if(steplossCount_clud >= 6)
	{	
		if(cycleModeFlag_clud == TRUE) { lin_FrDrCycleMode_TimerStop(); }
	
		steplossCount_clud = 0;
		
		recoveryFlags.FR_CLUD_Recov_Flag = TRUE;

		gotoZeroPtFlag_clud = TRUE;
	
		Lin_Scheduler_SpecialCmd_Start_1();

	}
	else
	{
		Check300ms_flag_clud = FALSE;
		steplossCount_clud = 0;
	}
	
}

void Check_300ms_sllr(void)
{
    Amo_timer_Stop(timer_100);
    
    if(steplossCount_sllr >= 6)
    {   
        if(cycleModeFlag_sllr == TRUE) { lin_FrDrCycleMode_TimerStop(); }
    
        steplossCount_sllr = 0;
        
        recoveryFlags.FR_SLLR_Recov_Flag = TRUE;

        gotoZeroPtFlag_sllr = TRUE;
    
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_sllr = FALSE;
        steplossCount_sllr = 0;
    }
}

void Check_300ms_slud(void)
{
    Amo_timer_Stop(timer_101);
    
    if(steplossCount_slud >= 6)
    {   
        if(cycleModeFlag_slud == TRUE) { lin_FrDrCycleMode_TimerStop(); }
    
        steplossCount_slud = 0;
        
        recoveryFlags.FR_SLUD_Recov_Flag = TRUE;

        gotoZeroPtFlag_slud = TRUE;
    
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_slud = FALSE;
        steplossCount_slud = 0;
    }
}

void Check_300ms_crlr(void)
{
    Amo_timer_Stop(timer_102);
    
    if(steplossCount_crlr >= 6)
    {   
				if(cycleModeFlag_crlr == TRUE) { lin_FrPsCycleMode_TimerStop(); }
    
        steplossCount_crlr = 0;
        
        recoveryFlags.FR_CRLR_Recov_Flag = TRUE;

        gotoZeroPtFlag_crlr = TRUE;
    
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_crlr = FALSE;
        steplossCount_crlr = 0;
    }
}

void Check_300ms_crud(void)
{
    Amo_timer_Stop(timer_103);
    
    if(steplossCount_crud >= 6)
    {   
        if(cycleModeFlag_crud == TRUE) { lin_FrPsCycleMode_TimerStop(); }
    
        steplossCount_crud = 0;
        
        recoveryFlags.FR_CRUD_Recov_Flag = TRUE;

        gotoZeroPtFlag_crud = TRUE;
    
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_crud = FALSE;
        steplossCount_crud = 0;
    }
}

void Check_300ms_srlr(void)
{
    Amo_timer_Stop(timer_104);
    
    if(steplossCount_srlr >= 6)
    {   
        if(cycleModeFlag_srlr == TRUE) { lin_FrPsCycleMode_TimerStop(); }
    
        steplossCount_srlr = 0;
        
        recoveryFlags.FR_SRLR_Recov_Flag = TRUE;

        gotoZeroPtFlag_srlr = TRUE;
    
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_srlr = FALSE;
        steplossCount_srlr = 0;
    }
}

void Check_300ms_srud(void)
{
    Amo_timer_Stop(timer_105);
    
    if(steplossCount_srud >= 6)
    {   
        if(cycleModeFlag_srud == TRUE) { lin_FrPsCycleMode_TimerStop(); }
    
        steplossCount_srud = 0;
        
        recoveryFlags.FR_SRUD_Recov_Flag = TRUE;

        gotoZeroPtFlag_srud = TRUE;
    
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_srud = FALSE;
        steplossCount_srud = 0;
    }
}


void Check_300ms_1_cllr(void)
{
	Amo_timer_Stop(timer_91);
	
	if(steplossCount_1_cllr >= 6)
	{
		goToTargetFlag_cllr = FALSE; 
		
		steplossCount_1_cllr = 0;
		
		recoveryFlags.FR_CLLR_Recov_Flag = TRUE;

		gotoZeroPtFlag_cllr = TRUE;
		
		Lin_Scheduler_SpecialCmd_Start_1();
	}
	else
	{
		Check300ms_flag_1_cllr = FALSE;
		steplossCount_1_cllr = 0;
	}
	
}

void Check_300ms_1_clud(void)
{
	
	Amo_timer_Stop(timer_96);
	
	if(steplossCount_1_clud >= 6)
	{
		goToTargetFlag_clud = FALSE; 
		
		steplossCount_1_clud = 0;
		
		recoveryFlags.FR_CLUD_Recov_Flag = TRUE;

		gotoZeroPtFlag_clud = TRUE;
		
		Lin_Scheduler_SpecialCmd_Start_1();
	}
	else
	{
		Check300ms_flag_1_clud = FALSE;
		steplossCount_1_clud = 0;
	}
	
}

void Check_300ms_1_sllr(void)
{
    Amo_timer_Stop(timer_106);
    
    if(steplossCount_1_sllr >= 6)
    {       
        goToTargetFlag_sllr = FALSE; 
        
        steplossCount_1_sllr = 0;
        
        recoveryFlags.FR_SLLR_Recov_Flag = TRUE;

        gotoZeroPtFlag_sllr = TRUE;
        
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_1_sllr = FALSE;
        steplossCount_1_sllr = 0;
    }
}

void Check_300ms_1_slud(void)
{
    Amo_timer_Stop(timer_107);
    
    if(steplossCount_1_slud >= 6)
    {       
        goToTargetFlag_slud = FALSE; 
        
        steplossCount_1_slud = 0;
        
        recoveryFlags.FR_SLUD_Recov_Flag = TRUE;

        gotoZeroPtFlag_slud = TRUE;
        
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_1_slud = FALSE;
        steplossCount_1_slud = 0;
    }
}

void Check_300ms_1_crlr(void)
{
    Amo_timer_Stop(timer_108);
    
    if(steplossCount_1_crlr >= 6)
    {       
        goToTargetFlag_crlr = FALSE; 
        
        steplossCount_1_crlr = 0;
        
        recoveryFlags.FR_CRLR_Recov_Flag = TRUE;

        gotoZeroPtFlag_crlr = TRUE;
        
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_1_crlr = FALSE;
        steplossCount_1_crlr = 0;
    }
}

void Check_300ms_1_crud(void)
{
    Amo_timer_Stop(timer_109);
    
    if(steplossCount_1_crud >= 6)
    {       
        goToTargetFlag_crud = FALSE; 
        
        steplossCount_1_crud = 0;
        
        recoveryFlags.FR_CRUD_Recov_Flag = TRUE;

        gotoZeroPtFlag_crud = TRUE;
        
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_1_crud = FALSE;
        steplossCount_1_crud = 0;
    }
}

void Check_300ms_1_srlr(void)
{
    Amo_timer_Stop(timer_110);
    
    if(steplossCount_1_srlr >= 6)
    {       
        goToTargetFlag_srlr = FALSE; 
        
        steplossCount_1_srlr = 0;
        
        recoveryFlags.FR_SRLR_Recov_Flag = TRUE;

        gotoZeroPtFlag_srlr = TRUE;
        
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_1_srlr = FALSE;
        steplossCount_1_srlr = 0;
    }
}

void Check_300ms_1_srud(void)
{
    Amo_timer_Stop(timer_111);
    
    if(steplossCount_1_srud >= 6)
    {       
        goToTargetFlag_srud = FALSE; 
        
        steplossCount_1_srud = 0;
        
        recoveryFlags.FR_SRUD_Recov_Flag = TRUE;

        gotoZeroPtFlag_srud = TRUE;
        
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_1_srud = FALSE;
        steplossCount_1_srud = 0;
    }
}


void Check_300ms_2_cllr(void)
{
	Amo_timer_Stop(timer_91);
	
	if(steplossCount_2_cllr >= 6)
	{		
		goToTargetFlag_cllr = FALSE; 
		
		steplossCount_2_cllr = 0;
	
		recoveryFlags.FR_CLLR_Recov_Flag = TRUE;

		gotoZeroPtFlag_cllr = TRUE;
		
		Lin_Scheduler_SpecialCmd_Start_1();
	}
	else
	{
		Check300ms_flag_2_cllr = FALSE;
		steplossCount_2_cllr = 0;
	}
	
}


void Check_300ms_2_clud(void)
{
	Amo_timer_Stop(timer_96);
	
	if(steplossCount_2_clud >= 6)
	{		
		goToTargetFlag_clud = FALSE; 
		
		steplossCount_2_clud = 0;
	
		recoveryFlags.FR_CLUD_Recov_Flag = TRUE;

		gotoZeroPtFlag_clud = TRUE;
		
		Lin_Scheduler_SpecialCmd_Start_1();
	}
	else
	{
		Check300ms_flag_2_clud = FALSE;
		steplossCount_2_clud = 0;
	}
	
}

void Check_300ms_2_sllr(void)
{
    Amo_timer_Stop(timer_106);
    
    if(steplossCount_2_sllr >= 6)
    {       
        goToTargetFlag_sllr = FALSE; 
        
        steplossCount_2_sllr = 0;
    
        recoveryFlags.FR_SLLR_Recov_Flag = TRUE;

        gotoZeroPtFlag_sllr = TRUE;
        
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_2_sllr = FALSE;
        steplossCount_2_sllr = 0;
    }
}

void Check_300ms_2_slud(void)
{
    Amo_timer_Stop(timer_107);
    
    if(steplossCount_2_slud >= 6)
    {       
        goToTargetFlag_slud = FALSE; 
        
        steplossCount_2_slud = 0;
    
        recoveryFlags.FR_SLUD_Recov_Flag = TRUE;

        gotoZeroPtFlag_slud = TRUE;
        
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_2_slud = FALSE;
        steplossCount_2_slud = 0;
    }
}

void Check_300ms_2_crlr(void)
{
    Amo_timer_Stop(timer_108);
    
    if(steplossCount_2_crlr >= 6)
    {       
        goToTargetFlag_crlr = FALSE; 
        
        steplossCount_2_crlr = 0;
    
        recoveryFlags.FR_CRLR_Recov_Flag = TRUE;

        gotoZeroPtFlag_crlr = TRUE;
        
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_2_crlr = FALSE;
        steplossCount_2_crlr = 0;
    }
}

void Check_300ms_2_crud(void)
{
    Amo_timer_Stop(timer_109);
    
    if(steplossCount_2_crud >= 6)
    {       
        goToTargetFlag_crud = FALSE; 
        
        steplossCount_2_crud = 0;
    
        recoveryFlags.FR_CRUD_Recov_Flag = TRUE;

        gotoZeroPtFlag_crud = TRUE;
        
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_2_crud = FALSE;
        steplossCount_2_crud = 0;
    }
}

void Check_300ms_2_srlr(void)
{
    Amo_timer_Stop(timer_110);
    
    if(steplossCount_2_srlr >= 6)
    {       
        goToTargetFlag_srlr = FALSE; 
        
        steplossCount_2_srlr = 0;
    
        recoveryFlags.FR_SRLR_Recov_Flag = TRUE;

        gotoZeroPtFlag_srlr = TRUE;
        
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_2_srlr = FALSE;
        steplossCount_2_srlr = 0;
    }
}

void Check_300ms_2_srud(void)
{
    Amo_timer_Stop(timer_111);
    
    if(steplossCount_2_srud >= 6)
    {       
        goToTargetFlag_srud = FALSE; 
        
        steplossCount_2_srud = 0;
    
        recoveryFlags.FR_SRUD_Recov_Flag = TRUE;

        gotoZeroPtFlag_srud = TRUE;
        
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_2_srud = FALSE;
        steplossCount_2_srud = 0;
    }
}


void Check_300ms_3_cllr(void)
{
	Amo_timer_Stop(timer_91);
	
	if(steplossCount_3_cllr >= 6)
	{
		goToTargetFlag_cllr = FALSE; 
		
		steplossCount_3_cllr = 0;

		recoveryFlags.FR_CLLR_Recov_Flag = TRUE;

		gotoZeroPtFlag_cllr = TRUE;
		
		Lin_Scheduler_SpecialCmd_Start_1();
	}
	else
	{
		Check300ms_flag_3_cllr = FALSE;
		steplossCount_3_cllr = 0;
	}
	
}


void Check_300ms_3_clud(void)
{
	Amo_timer_Stop(timer_91);
	
	if(steplossCount_3_clud >= 6)
	{
		goToTargetFlag_clud = FALSE; 
		
		steplossCount_3_clud = 0;

		recoveryFlags.FR_CLUD_Recov_Flag = TRUE;

		gotoZeroPtFlag_clud = TRUE;
		
		Lin_Scheduler_SpecialCmd_Start_1();
	}
	else
	{
		Check300ms_flag_3_clud = FALSE;
		steplossCount_3_clud = 0;
	}
	
}

void Check_300ms_3_sllr(void)
{
    Amo_timer_Stop(timer_106);
    
    if(steplossCount_3_sllr >= 6)
    {
        goToTargetFlag_sllr = FALSE; 
        
        steplossCount_3_sllr = 0;

        recoveryFlags.FR_SLLR_Recov_Flag = TRUE;

        gotoZeroPtFlag_sllr = TRUE;
        
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_3_sllr = FALSE;
        steplossCount_3_sllr = 0;
    }
}

void Check_300ms_3_slud(void)
{
    Amo_timer_Stop(timer_107);
    
    if(steplossCount_3_slud >= 6)
    {
        goToTargetFlag_slud = FALSE; 
        
        steplossCount_3_slud = 0;

        recoveryFlags.FR_SLUD_Recov_Flag = TRUE;

        gotoZeroPtFlag_slud = TRUE;
        
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_3_slud = FALSE;
        steplossCount_3_slud = 0;
    }
}

void Check_300ms_3_crlr(void)
{
    Amo_timer_Stop(timer_108);
    
    if(steplossCount_3_crlr >= 6)
    {
        goToTargetFlag_crlr = FALSE; 
        
        steplossCount_3_crlr = 0;

        recoveryFlags.FR_CRLR_Recov_Flag = TRUE;

        gotoZeroPtFlag_crlr = TRUE;
        
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_3_crlr = FALSE;
        steplossCount_3_crlr = 0;
    }
}

void Check_300ms_3_crud(void)
{
    Amo_timer_Stop(timer_109);
    
    if(steplossCount_3_crud >= 6)
    {
        goToTargetFlag_crud = FALSE; 
        
        steplossCount_3_crud = 0;

        recoveryFlags.FR_CRUD_Recov_Flag = TRUE;

        gotoZeroPtFlag_crud = TRUE;
        
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_3_crud = FALSE;
        steplossCount_3_crud = 0;
    }
}

void Check_300ms_3_srlr(void)
{
    Amo_timer_Stop(timer_110);
    
    if(steplossCount_3_srlr >= 6)
    {
        goToTargetFlag_srlr = FALSE; 
        
        steplossCount_3_srlr = 0;

        recoveryFlags.FR_SRLR_Recov_Flag = TRUE;

        gotoZeroPtFlag_srlr = TRUE;
        
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_3_srlr = FALSE;
        steplossCount_3_srlr = 0;
    }
}

void Check_300ms_3_srud(void)
{
    Amo_timer_Stop(timer_111);
    
    if(steplossCount_3_srud >= 6)
    {
        goToTargetFlag_srud = FALSE; 
        
        steplossCount_3_srud = 0;

        recoveryFlags.FR_SRUD_Recov_Flag = TRUE;

        gotoZeroPtFlag_srud = TRUE;
        
        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_3_srud = FALSE;
        steplossCount_3_srud = 0;
    }
}


void Check_300ms_4_cllr(void)
{
	Amo_timer_Stop(timer_91);
	
	if(steplossCount_4_cllr >= 6)
	{
		goToTargetFlag_cllr = FALSE; 
		
		steplossCount_4_cllr = 0;

		recoveryFlags.FR_CLLR_Recov_Flag = TRUE;

		gotoZeroPtFlag_cllr = TRUE;
		
		Lin_Scheduler_SpecialCmd_Start_1();
	}
	else
	{
		Check300ms_flag_4_cllr = FALSE;
		steplossCount_4_cllr = 0;
	}
	
}

void Check_300ms_4_clud(void)
{
	Amo_timer_Stop(timer_91);
	
	if(steplossCount_4_clud >= 6)
	{
		goToTargetFlag_clud = FALSE; 
		
		steplossCount_4_clud = 0;

		recoveryFlags.FR_CLUD_Recov_Flag = TRUE;

		gotoZeroPtFlag_clud = TRUE;
		
		Lin_Scheduler_SpecialCmd_Start_1();
	}
	else
	{
		Check300ms_flag_4_clud = FALSE;
		steplossCount_4_clud = 0;
	}
	
}

void Check_300ms_4_sllr(void)
{
    Amo_timer_Stop(timer_106);

    if(steplossCount_4_sllr >= 6)
    {
        goToTargetFlag_sllr = FALSE;

        steplossCount_4_sllr = 0;

        recoveryFlags.FR_SLLR_Recov_Flag = TRUE;

        gotoZeroPtFlag_sllr = TRUE;

        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_4_sllr = FALSE;
        steplossCount_4_sllr = 0;
    }
}

void Check_300ms_4_slud(void)
{
    Amo_timer_Stop(timer_107);

    if(steplossCount_4_slud >= 6)
    {
        goToTargetFlag_slud = FALSE;

        steplossCount_4_slud = 0;

        recoveryFlags.FR_SLUD_Recov_Flag = TRUE;

        gotoZeroPtFlag_slud = TRUE;

        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_4_slud = FALSE;
        steplossCount_4_slud = 0;
    }
}

void Check_300ms_4_crlr(void)
{
    Amo_timer_Stop(timer_108);

    if(steplossCount_4_crlr >= 6)
    {
        goToTargetFlag_crlr = FALSE;

        steplossCount_4_crlr = 0;

        recoveryFlags.FR_CRLR_Recov_Flag = TRUE;

        gotoZeroPtFlag_crlr = TRUE;

        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_4_crlr = FALSE;
        steplossCount_4_crlr = 0;
    }
}

void Check_300ms_4_crud(void)
{
    Amo_timer_Stop(timer_109);

    if(steplossCount_4_crud >= 6)
    {
        goToTargetFlag_crud = FALSE;

        steplossCount_4_crud = 0;

        recoveryFlags.FR_CRUD_Recov_Flag = TRUE;

        gotoZeroPtFlag_crud = TRUE;

        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_4_crud = FALSE;
        steplossCount_4_crud = 0;
    }
}

void Check_300ms_4_srlr(void)
{
    Amo_timer_Stop(timer_110);

    if(steplossCount_4_srlr >= 6)
    {
        goToTargetFlag_srlr = FALSE;

        steplossCount_4_srlr = 0;

        recoveryFlags.FR_SRLR_Recov_Flag = TRUE;

        gotoZeroPtFlag_srlr = TRUE;

        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_4_srlr = FALSE;
        steplossCount_4_srlr = 0;
    }
}

void Check_300ms_4_srud(void)
{
    Amo_timer_Stop(timer_111);

    if(steplossCount_4_srud >= 6)
    {
        goToTargetFlag_srud = FALSE;

        steplossCount_4_srud = 0;

        recoveryFlags.FR_SRUD_Recov_Flag = TRUE;

        gotoZeroPtFlag_srud = TRUE;

        Lin_Scheduler_SpecialCmd_Start_1();
    }
    else
    {
        Check300ms_flag_4_srud = FALSE;
        steplossCount_4_srud = 0;
    }
}


void Check_300ms_5_cllr(void)
{
	Amo_timer_Stop(timer_91);
	
	if((steplossCount_1_cllr >= 6))// && (steplossCount_1_cllr <= 10))
	{
		steplossCount_1_cllr = 0;

		cycleModeStopCount_cllr++;

		goToTargetFlag_cllr = FALSE;

		if(cycleModeStopCount_cllr == 1)
		{
			if(cycleModeFlag_cllr == TRUE) { lin_FrDrCycleMode_TimerStop(); }
		}
		
		recoveryFlags.FR_CLLR_Recov_Flag = TRUE;
		
		if((gotoTargetPtCount_cllr < 5) && (gotoZeroPtCount_cllr < 5)) //recovery 5 times
		{	
			gotoZeroPtFlag_cllr = TRUE;
			
			Lin_Scheduler_SpecialCmd_Start_1();

		}
		else
		{	
//			eventCountTest_cllr = 0;

			cycleModeStopCount_cllr = 0;

			recoveryModeFlag_cllr = FALSE;

			Check300ms_flag_1_cllr = FALSE;

			gotoTargetPtCount_cllr = 0;
			gotoZeroPtCount_cllr = 0;

			gotoZeroPtFlag_cllr = FALSE;
			
			steplossCount_1_cllr = 0;

			lastTryFlag_cllr = TRUE;

//			recoveryPattern_cllr = Recovery_None;

			Lin_Scheduler_SpecialCmd_Start_1();
		}

	}
	else
	{
		Check300ms_flag_1_cllr = FALSE;
		steplossCount_1_cllr = 0;
//		recoveryPattern_cllr = Recovery_None;
	}
	
}


void Check_300ms_5_clud(void)
{
	Amo_timer_Stop(timer_96);
	
	if(steplossCount_1_clud >= 6)
	{
		cycleModeStopCount_clud++;
		
		steplossCount_1_clud = 0;

		goToTargetFlag_clud = FALSE;

		if(cycleModeStopCount_clud == 1)
		{
			if(cycleModeFlag_clud == TRUE) { lin_FrDrCycleMode_TimerStop(); }
		}
		
		recoveryFlags.FR_CLUD_Recov_Flag = TRUE;
		
		if((gotoTargetPtCount_clud < 5) && (gotoZeroPtCount_clud < 5)) //recovery 5 times
		{	
			gotoZeroPtFlag_clud = TRUE;
			
			Lin_Scheduler_SpecialCmd_Start_1();

		}
		else
		{	

			cycleModeStopCount_clud = 0;

			recoveryModeFlag_clud = FALSE;

			Check300ms_flag_1_clud = FALSE;

			gotoTargetPtCount_clud = 0;
			gotoZeroPtCount_clud = 0;

			gotoZeroPtFlag_clud = FALSE;
			
			steplossCount_1_clud = 0;

			lastTryFlag_clud = TRUE;

			Lin_Scheduler_SpecialCmd_Start_1();
		}
		
	}
	else
	{
		Check300ms_flag_1_clud = FALSE;
		steplossCount_1_clud = 0;
//		recoveryPattern_clud = Recovery_None;
	}
	
}


void Check_300ms_5_sllr(void)
{
	Amo_timer_Stop(timer_106);
	
	if((steplossCount_1_sllr >= 6))// && (steplossCount_1_cllr <= 10))
	{
		steplossCount_1_sllr = 0;

		cycleModeStopCount_sllr++;

		goToTargetFlag_sllr = FALSE;

		if(cycleModeStopCount_sllr == 1)
		{
			if(cycleModeFlag_sllr == TRUE) { lin_FrDrCycleMode_TimerStop(); }
		}
		
		recoveryFlags.FR_SLLR_Recov_Flag = TRUE;
		
		if((gotoTargetPtCount_sllr < 5) && (gotoZeroPtCount_sllr < 5)) //recovery 5 times
		{	
			gotoZeroPtFlag_sllr = TRUE;
			
			Lin_Scheduler_SpecialCmd_Start_1();

		}
		else
		{	
//			eventCountTest_cllr = 0;

			cycleModeStopCount_sllr = 0;

			recoveryModeFlag_sllr = FALSE;

			Check300ms_flag_1_sllr = FALSE;

			gotoTargetPtCount_sllr = 0;
			gotoZeroPtCount_sllr = 0;

			gotoZeroPtFlag_sllr = FALSE;
			
			steplossCount_1_sllr = 0;

			lastTryFlag_sllr = TRUE;

//			recoveryPattern_cllr = Recovery_None;

			Lin_Scheduler_SpecialCmd_Start_1();
		}

	}
	else
	{
		Check300ms_flag_1_sllr = FALSE;
		steplossCount_1_sllr = 0;
//		recoveryPattern_cllr = Recovery_None;
	}
	
}


void Check_300ms_5_slud(void)
{
	Amo_timer_Stop(timer_107);
	
	if(steplossCount_1_slud >= 6)
	{
		cycleModeStopCount_slud++;
		
		steplossCount_1_slud = 0;

		goToTargetFlag_slud = FALSE;

		if(cycleModeStopCount_slud == 1)
		{
			if(cycleModeFlag_slud == TRUE) { lin_FrDrCycleMode_TimerStop(); }
		}
		
		recoveryFlags.FR_SLUD_Recov_Flag = TRUE;
		
		if((gotoTargetPtCount_slud < 5) && (gotoZeroPtCount_slud < 5)) //recovery 5 times
		{	
			gotoZeroPtFlag_slud = TRUE;
			
			Lin_Scheduler_SpecialCmd_Start_1();

		}
		else
		{	

			cycleModeStopCount_slud = 0;

			recoveryModeFlag_slud = FALSE;

			Check300ms_flag_1_slud = FALSE;

			gotoTargetPtCount_slud = 0;
			gotoZeroPtCount_slud = 0;

			gotoZeroPtFlag_slud = FALSE;
			
			steplossCount_1_slud = 0;

			lastTryFlag_slud = TRUE;

			Lin_Scheduler_SpecialCmd_Start_1();
		}
		
	}
	else
	{
		Check300ms_flag_1_slud = FALSE;
		steplossCount_1_slud = 0;
//		recoveryPattern_clud = Recovery_None;
	}
	
}

void Check_300ms_5_crlr(void)
{
	Amo_timer_Stop(timer_108);
	
	if((steplossCount_1_crlr >= 6))// && (steplossCount_1_cllr <= 10))
	{
		steplossCount_1_crlr = 0;

		cycleModeStopCount_crlr++;

		goToTargetFlag_crlr = FALSE;

		if(cycleModeStopCount_crlr == 1)
		{
			if(cycleModeFlag_crlr == TRUE) { lin_FrPsCycleMode_TimerStop(); }
		}
		
		recoveryFlags.FR_CRLR_Recov_Flag = TRUE;
		
		if((gotoTargetPtCount_crlr < 5) && (gotoZeroPtCount_crlr < 5)) //recovery 5 times
		{	
			gotoZeroPtFlag_crlr = TRUE;
			
			Lin_Scheduler_SpecialCmd_Start_1();

		}
		else
		{	
//			eventCountTest_cllr = 0;

			cycleModeStopCount_crlr = 0;

			recoveryModeFlag_crlr = FALSE;

			Check300ms_flag_1_crlr = FALSE;

			gotoTargetPtCount_crlr = 0;
			gotoZeroPtCount_crlr = 0;

			gotoZeroPtFlag_crlr = FALSE;
			
			steplossCount_1_crlr = 0;

			lastTryFlag_crlr = TRUE;

//			recoveryPattern_cllr = Recovery_None;

			Lin_Scheduler_SpecialCmd_Start_1();
		}

	}
	else
	{
		Check300ms_flag_1_crlr = FALSE;
		steplossCount_1_crlr = 0;
//		recoveryPattern_cllr = Recovery_None;
	}
	
}

void Check_300ms_5_crud(void)
{
	Amo_timer_Stop(timer_109);
	
	if(steplossCount_1_crud >= 6)
	{
		cycleModeStopCount_crud++;
		
		steplossCount_1_crud = 0;

		goToTargetFlag_crud = FALSE;

		if(cycleModeStopCount_crud == 1)
		{
			if(cycleModeFlag_crud == TRUE) { lin_FrPsCycleMode_TimerStop(); }
		}
		
		recoveryFlags.FR_CRUD_Recov_Flag = TRUE;
		
		if((gotoTargetPtCount_crud < 5) && (gotoZeroPtCount_crud < 5)) //recovery 5 times
		{	
			gotoZeroPtFlag_crud = TRUE;
			
			Lin_Scheduler_SpecialCmd_Start_1();

		}
		else
		{	

			cycleModeStopCount_crud = 0;

			recoveryModeFlag_crud = FALSE;

			Check300ms_flag_1_crud = FALSE;

			gotoTargetPtCount_crud = 0;
			gotoZeroPtCount_crud = 0;

			gotoZeroPtFlag_crud = FALSE;
			
			steplossCount_1_crud = 0;

			lastTryFlag_crud = TRUE;

			Lin_Scheduler_SpecialCmd_Start_1();
		}
		
	}
	else
	{
		Check300ms_flag_1_crud = FALSE;
		steplossCount_1_crud = 0;
//		recoveryPattern_clud = Recovery_None;
	}
	
}


void Check_300ms_5_srlr(void)
{
	Amo_timer_Stop(timer_110);
	
	if((steplossCount_1_srlr >= 6))// && (steplossCount_1_cllr <= 10))
	{
		steplossCount_1_srlr = 0;

		cycleModeStopCount_srlr++;

		goToTargetFlag_srlr = FALSE;

		if(cycleModeStopCount_srlr == 1)
		{
			if(cycleModeFlag_srlr == TRUE) { lin_FrPsCycleMode_TimerStop(); }
		}
		
		recoveryFlags.FR_SRLR_Recov_Flag = TRUE;
		
		if((gotoTargetPtCount_srlr < 5) && (gotoZeroPtCount_srlr < 5)) //recovery 5 times
		{	
			gotoZeroPtFlag_srlr = TRUE;
			
			Lin_Scheduler_SpecialCmd_Start_1();

		}
		else
		{	
//			eventCountTest_cllr = 0;

			cycleModeStopCount_srlr = 0;

			recoveryModeFlag_srlr = FALSE;

			Check300ms_flag_1_srlr = FALSE;

			gotoTargetPtCount_srlr = 0;
			gotoZeroPtCount_srlr = 0;

			gotoZeroPtFlag_srlr = FALSE;
			
			steplossCount_1_srlr = 0;

			lastTryFlag_srlr = TRUE;

//			recoveryPattern_cllr = Recovery_None;

			Lin_Scheduler_SpecialCmd_Start_1();
		}

	}
	else
	{
		Check300ms_flag_1_srlr = FALSE;
		steplossCount_1_srlr = 0;
//		recoveryPattern_cllr = Recovery_None;
	}
	
}

void Check_300ms_5_srud(void)
{
	Amo_timer_Stop(timer_111);
	
	if(steplossCount_1_srud >= 6)
	{
		cycleModeStopCount_srud++;
		
		steplossCount_1_srud = 0;

		goToTargetFlag_srud = FALSE;

		if(cycleModeStopCount_srud == 1)
		{
			if(cycleModeFlag_srud == TRUE) { lin_FrPsCycleMode_TimerStop(); }
		}
		
		recoveryFlags.FR_SRUD_Recov_Flag = TRUE;
		
		if((gotoTargetPtCount_srud < 5) && (gotoZeroPtCount_srud < 5)) //recovery 5 times
		{	
			gotoZeroPtFlag_srud = TRUE;
			
			Lin_Scheduler_SpecialCmd_Start_1();

		}
		else
		{	

			cycleModeStopCount_srud = 0;

			recoveryModeFlag_srud = FALSE;

			Check300ms_flag_1_srud = FALSE;

			gotoTargetPtCount_srud = 0;
			gotoZeroPtCount_srud = 0;

			gotoZeroPtFlag_srud = FALSE;
			
			steplossCount_1_srud = 0;

			lastTryFlag_srud = TRUE;

			Lin_Scheduler_SpecialCmd_Start_1();
		}
		
	}
	else
	{
		Check300ms_flag_1_srud = FALSE;
		steplossCount_1_srud = 0;
//		recoveryPattern_clud = Recovery_None;
	}
	
}

uint16_t absValue(int16_t num)
{
	return (num < 0) ? -num : num;
}

void Handle_Recovery_StopPtCheck_cllr(Lin_EVntPt_t *lin_Pt)
{

#if defined(ACT_ANGLE_UPDATE_3)
	if(lin_Pt->FrDrPt.prevPt_FR_CLLR >= Max_LR && lin_Pt->FrDrPt.prevPt_FR_CLLR < 4096) { startPt_cllr = minusDegree; }
	else if(lin_Pt->FrDrPt.prevPt_FR_CLLR >= 0 && lin_Pt->FrDrPt.prevPt_FR_CLLR <= Min_LR) { startPt_cllr = plusDegree; }

	if(stopPt_cllr >= Max_LR && stopPt_cllr < 4096) { recoveryPt_cllr = minusDegree; }
	else if(stopPt_cllr >= 0 && stopPt_cllr <= Min_LR) { recoveryPt_cllr = plusDegree; }
#else

	if(lin_Pt->FrDrPt.prevPt_FR_CLLR >= Min_LR && lin_Pt->FrDrPt.prevPt_FR_CLLR < 4096) { startPt_cllr = minusDegree; }
	else if(lin_Pt->FrDrPt.prevPt_FR_CLLR >= 0 && lin_Pt->FrDrPt.prevPt_FR_CLLR <= Max_LR) { startPt_cllr = plusDegree; }

	if(stopPt_cllr >= Min_LR && stopPt_cllr < 4096) { recoveryPt_cllr = minusDegree; }
	else if(stopPt_cllr >= 0 && stopPt_cllr <= Max_LR) { recoveryPt_cllr = plusDegree; }
#endif	

	if(startPt_cllr == minusDegree)
	{
		if(recoveryPt_cllr == minusDegree)
		{
			if(lin_Pt->FrDrPt.prevPt_FR_CLLR < stopPt_cllr) { recoveryPattern_cllr = Cant_go_to_zeroPt; }
			else { recoveryPattern_cllr = Repeat_zeroPt; }
		}
		else if(recoveryPt_cllr == plusDegree) { recoveryPattern_cllr = Repeat_zeroPt; }
	}
	else if(startPt_cllr == plusDegree)
	{
		if(recoveryPt_cllr == plusDegree)
		{
			if(lin_Pt->FrDrPt.prevPt_FR_CLLR < stopPt_cllr) { recoveryPattern_cllr = Repeat_zeroPt; }
			else { recoveryPattern_cllr = Cant_go_to_zeroPt; }
		}
		else if(recoveryPt_cllr == minusDegree) { recoveryPattern_cllr = Repeat_zeroPt; }
	}
}


void Handle_Recovery_StopPtCheck_clud(Lin_EVntPt_t *lin_Pt)
{
	//					canCenterPt = GET_CanCenterPt_CLLR(&Can_data_23_Info);
	//					CanLinValue[FrDrCtrPt_X].Pt_X.Can_RangePt_Center_X;
	
	if(fullcloseModeFlag_clud == TRUE)
	{
		if(lin_Pt->FrDrPt.prevPt_FR_CLUD >= FullClose_UD_Rev && lin_Pt->FrDrPt.prevPt_FR_CLUD < 4096) { startPt_clud = minusDegree; }
		else if(lin_Pt->FrDrPt.prevPt_FR_CLUD >= 0 && lin_Pt->FrDrPt.prevPt_FR_CLUD <= Min_UD) { startPt_clud = plusDegree; }

		if(stopPt_clud >= FullClose_UD_Rev && stopPt_clud < 4096) { recoveryPt_clud = minusDegree; }
		else if(stopPt_clud >= 0 && stopPt_clud <= Min_UD) { recoveryPt_clud = plusDegree; }
	}
	else
	{
		if(lin_Pt->FrDrPt.prevPt_FR_CLUD >= Max_UD && lin_Pt->FrDrPt.prevPt_FR_CLUD < 4096) { startPt_clud = minusDegree; }
		else if(lin_Pt->FrDrPt.prevPt_FR_CLUD >= 0 && lin_Pt->FrDrPt.prevPt_FR_CLUD <= Min_UD) { startPt_clud = plusDegree; }

		if(stopPt_clud >= Max_UD && stopPt_clud < 4096) { recoveryPt_clud = minusDegree; }
		else if(stopPt_clud >= 0 && stopPt_clud <= Min_UD) { recoveryPt_clud = plusDegree; }
	}

	//					if(lin_Pt->FrDrPt.prevPt_FR_CLLR <= canCenterPt)
	if(startPt_clud == minusDegree)
	{
	//						if(stopPt <= canCenterPt)
		if(recoveryPt_clud == minusDegree)
		{
			if(lin_Pt->FrDrPt.prevPt_FR_CLUD < stopPt_clud) { recoveryPattern_clud = Cant_go_to_zeroPt; }
			else { recoveryPattern_clud = Repeat_zeroPt; }
		}
		else if(recoveryPt_clud == plusDegree) { recoveryPattern_clud = Repeat_zeroPt; }
	}
	else if(startPt_clud == plusDegree)
	{
		if(recoveryPt_clud == plusDegree)
		{
			if(lin_Pt->FrDrPt.prevPt_FR_CLUD < stopPt_clud) { recoveryPattern_clud = Repeat_zeroPt; }
			else { recoveryPattern_clud = Cant_go_to_zeroPt; }
		}
		else if(recoveryPt_clud == minusDegree) { recoveryPattern_clud = Repeat_zeroPt; }
	}
}

void Handle_Recovery_StopPtCheck_sllr(Lin_EVntPt_t *lin_Pt)
{

#if defined(ACT_ANGLE_UPDATE_3)
	if(lin_Pt->FrDrPt.prevPt_FR_SLLR >= Max_LR && lin_Pt->FrDrPt.prevPt_FR_SLLR < 4096) { startPt_sllr = minusDegree; }
	else if(lin_Pt->FrDrPt.prevPt_FR_SLLR >= 0 && lin_Pt->FrDrPt.prevPt_FR_SLLR <= Min_LR) { startPt_sllr = plusDegree; }

	if(stopPt_sllr >= Max_LR && stopPt_sllr < 4096) { recoveryPt_sllr = minusDegree; }
	else if(stopPt_sllr >= 0 && stopPt_sllr <= Min_LR) { recoveryPt_sllr = plusDegree; }
#else

	if(lin_Pt->FrDrPt.prevPt_FR_SLLR >= Min_LR && lin_Pt->FrDrPt.prevPt_FR_SLLR < 4096) { startPt_sllr = minusDegree; }
	else if(lin_Pt->FrDrPt.prevPt_FR_SLLR >= 0 && lin_Pt->FrDrPt.prevPt_FR_SLLR <= Max_LR) { startPt_sllr = plusDegree; }

	if(stopPt_sllr >= Min_LR && stopPt_sllr < 4096) { recoveryPt_sllr = minusDegree; }
	else if(stopPt_sllr >= 0 && stopPt_sllr <= Max_LR) { recoveryPt_sllr = plusDegree; }
#endif	

	if(startPt_sllr == minusDegree)
	{
		if(recoveryPt_sllr == minusDegree)
		{
			if(lin_Pt->FrDrPt.prevPt_FR_SLLR < stopPt_sllr) { recoveryPattern_sllr = Cant_go_to_zeroPt; }
			else { recoveryPattern_sllr = Repeat_zeroPt; }
		}
		else if(recoveryPt_sllr == plusDegree) { recoveryPattern_sllr = Repeat_zeroPt; }
	}
	else if(startPt_sllr == plusDegree)
	{
		if(recoveryPt_sllr == plusDegree)
		{
			if(lin_Pt->FrDrPt.prevPt_FR_SLLR < stopPt_sllr) { recoveryPattern_sllr = Repeat_zeroPt; }
			else { recoveryPattern_sllr = Cant_go_to_zeroPt; }
		}
		else if(recoveryPt_sllr == minusDegree) { recoveryPattern_sllr = Repeat_zeroPt; }
	}
}

void Handle_Recovery_StopPtCheck_slud(Lin_EVntPt_t *lin_Pt)
{
	//					canCenterPt = GET_CanCenterPt_CLLR(&Can_data_23_Info);
	//					CanLinValue[FrDrCtrPt_X].Pt_X.Can_RangePt_Center_X;
	
	if(fullcloseModeFlag_slud == TRUE)
	{
		if(lin_Pt->FrDrPt.prevPt_FR_SLUD >= FullClose_UD_Rev && lin_Pt->FrDrPt.prevPt_FR_SLUD < 4096) { startPt_slud = minusDegree; }
		else if(lin_Pt->FrDrPt.prevPt_FR_SLUD >= 0 && lin_Pt->FrDrPt.prevPt_FR_SLUD <= Min_UD) { startPt_slud = plusDegree; }

		if(stopPt_slud >= FullClose_UD_Rev && stopPt_slud < 4096) { recoveryPt_slud = minusDegree; }
		else if(stopPt_slud >= 0 && stopPt_slud <= Min_UD) { recoveryPt_slud = plusDegree; }
	}
	else
	{
		if(lin_Pt->FrDrPt.prevPt_FR_SLUD >= Max_UD && lin_Pt->FrDrPt.prevPt_FR_SLUD < 4096) { startPt_slud = minusDegree; }
		else if(lin_Pt->FrDrPt.prevPt_FR_SLUD >= 0 && lin_Pt->FrDrPt.prevPt_FR_SLUD <= Min_UD) { startPt_slud = plusDegree; }

		if(stopPt_slud >= Max_UD && stopPt_slud < 4096) { recoveryPt_slud = minusDegree; }
		else if(stopPt_slud >= 0 && stopPt_slud <= Min_UD) { recoveryPt_slud = plusDegree; }
	}

	//					if(lin_Pt->FrDrPt.prevPt_FR_CLLR <= canCenterPt)
	if(startPt_slud == minusDegree)
	{
	//						if(stopPt <= canCenterPt)
		if(recoveryPt_slud == minusDegree)
		{
			if(lin_Pt->FrDrPt.prevPt_FR_CLUD < stopPt_slud) { recoveryPattern_slud = Cant_go_to_zeroPt; }
			else { recoveryPattern_slud = Repeat_zeroPt; }
		}
		else if(recoveryPt_slud == plusDegree) { recoveryPattern_slud = Repeat_zeroPt; }
	}
	else if(startPt_slud == plusDegree)
	{
		if(recoveryPt_slud == plusDegree)
		{
			if(lin_Pt->FrDrPt.prevPt_FR_SLUD < stopPt_slud) { recoveryPattern_slud = Repeat_zeroPt; }
			else { recoveryPattern_slud = Cant_go_to_zeroPt; }
		}
		else if(recoveryPt_slud == minusDegree) { recoveryPattern_slud = Repeat_zeroPt; }
	}
}

void Handle_Recovery_StopPtCheck_crlr(Lin_EVntPt_t *lin_Pt)
{

#if defined(ACT_ANGLE_UPDATE_3)
	if(lin_Pt->FrPsPt.prevPt_FR_CRLR >= Max_LR && lin_Pt->FrPsPt.prevPt_FR_CRLR < 4096) { startPt_crlr = minusDegree; }
	else if(lin_Pt->FrPsPt.prevPt_FR_CRLR >= 0 && lin_Pt->FrPsPt.prevPt_FR_CRLR <= Min_LR) { startPt_crlr = plusDegree; }

	if(stopPt_crlr >= Max_LR && stopPt_crlr < 4096) { recoveryPt_crlr = minusDegree; }
	else if(stopPt_crlr >= 0 && stopPt_crlr <= Min_LR) { recoveryPt_crlr = plusDegree; }
#else

	if(lin_Pt->FrPsPt.prevPt_FR_CRLR >= Min_LR && lin_Pt->FrPsPt.prevPt_FR_CRLR < 4096) { startPt_crlr = minusDegree; }
	else if(lin_Pt->FrPsPt.prevPt_FR_CRLR >= 0 && lin_Pt->FrPsPt.prevPt_FR_CRLR <= Max_LR) { startPt_crlr = plusDegree; }

	if(stopPt_crlr >= Min_LR && stopPt_crlr < 4096) { recoveryPt_crlr = minusDegree; }
	else if(stopPt_crlr >= 0 && stopPt_crlr <= Max_LR) { recoveryPt_crlr = plusDegree; }
#endif	

	if(startPt_crlr == minusDegree)
	{
		if(recoveryPt_crlr == minusDegree)
		{
			if(lin_Pt->FrPsPt.prevPt_FR_CRLR < stopPt_crlr) { recoveryPattern_crlr = Cant_go_to_zeroPt; }
			else { recoveryPattern_crlr = Repeat_zeroPt; }
		}
		else if(recoveryPt_crlr == plusDegree) { recoveryPattern_crlr = Repeat_zeroPt; }
	}
	else if(startPt_crlr == plusDegree)
	{
		if(recoveryPt_crlr == plusDegree)
		{
			if(lin_Pt->FrPsPt.prevPt_FR_CRLR < stopPt_crlr) { recoveryPattern_crlr = Repeat_zeroPt; }
			else { recoveryPattern_crlr = Cant_go_to_zeroPt; }
		}
		else if(recoveryPt_crlr == minusDegree) { recoveryPattern_crlr = Repeat_zeroPt; }
	}
}

void Handle_Recovery_StopPtCheck_crud(Lin_EVntPt_t *lin_Pt)
{
	//					canCenterPt = GET_CanCenterPt_CLLR(&Can_data_23_Info);
	//					CanLinValue[FrDrCtrPt_X].Pt_X.Can_RangePt_Center_X;
	
	if(fullcloseModeFlag_crud == TRUE)
	{
		if(lin_Pt->FrPsPt.prevPt_FR_CRUD >= FullClose_UD_Rev && lin_Pt->FrPsPt.prevPt_FR_CRUD < 4096) { startPt_crud = minusDegree; }
		else if(lin_Pt->FrPsPt.prevPt_FR_CRUD >= 0 && lin_Pt->FrPsPt.prevPt_FR_CRUD <= Min_UD) { startPt_crud = plusDegree; }

		if(stopPt_crud >= FullClose_UD_Rev && stopPt_crud < 4096) { recoveryPt_crud = minusDegree; }
		else if(stopPt_crud >= 0 && stopPt_crud <= Min_UD) { recoveryPt_crud = plusDegree; }
	}
	else
	{
		if(lin_Pt->FrPsPt.prevPt_FR_CRUD >= Max_UD && lin_Pt->FrPsPt.prevPt_FR_CRUD < 4096) { startPt_crud = minusDegree; }
		else if(lin_Pt->FrPsPt.prevPt_FR_CRUD >= 0 && lin_Pt->FrPsPt.prevPt_FR_CRUD <= Min_UD) { startPt_crud = plusDegree; }

		if(stopPt_crud >= Max_UD && stopPt_crud < 4096) { recoveryPt_crud = minusDegree; }
		else if(stopPt_crud >= 0 && stopPt_crud <= Min_UD) { recoveryPt_crud = plusDegree; }
	}

	//					if(lin_Pt->FrDrPt.prevPt_FR_CLLR <= canCenterPt)
	if(startPt_crud == minusDegree)
	{
	//						if(stopPt <= canCenterPt)
		if(recoveryPt_crud == minusDegree)
		{
			if(lin_Pt->FrPsPt.prevPt_FR_CRUD < stopPt_crud) { recoveryPattern_crud = Cant_go_to_zeroPt; }
			else { recoveryPattern_crud = Repeat_zeroPt; }
		}
		else if(recoveryPt_crud == plusDegree) { recoveryPattern_crud = Repeat_zeroPt; }
	}
	else if(startPt_crud == plusDegree)
	{
		if(recoveryPt_crud == plusDegree)
		{
			if(lin_Pt->FrPsPt.prevPt_FR_CRUD < stopPt_crud) { recoveryPattern_crud = Repeat_zeroPt; }
			else { recoveryPattern_crud = Cant_go_to_zeroPt; }
		}
		else if(recoveryPt_crud == minusDegree) { recoveryPattern_crud = Repeat_zeroPt; }
	}
}

void Handle_Recovery_StopPtCheck_srlr(Lin_EVntPt_t *lin_Pt)
{

#if defined(ACT_ANGLE_UPDATE_3)
	if(lin_Pt->FrPsPt.prevPt_FR_SRLR >= Max_LR && lin_Pt->FrPsPt.prevPt_FR_SRLR < 4096) { startPt_srlr = minusDegree; }
	else if(lin_Pt->FrPsPt.prevPt_FR_SRLR >= 0 && lin_Pt->FrPsPt.prevPt_FR_SRLR <= Min_LR) { startPt_srlr = plusDegree; }

	if(stopPt_srlr >= Max_LR && stopPt_srlr < 4096) { recoveryPt_srlr = minusDegree; }
	else if(stopPt_srlr >= 0 && stopPt_srlr <= Min_LR) { recoveryPt_srlr = plusDegree; }
#else

	if(lin_Pt->FrPsPt.prevPt_FR_SRLR >= Min_LR && lin_Pt->FrPsPt.prevPt_FR_SRLR < 4096) { startPt_srlr = minusDegree; }
	else if(lin_Pt->FrPsPt.prevPt_FR_SRLR >= 0 && lin_Pt->FrPsPt.prevPt_FR_SRLR <= Max_LR) { startPt_srlr = plusDegree; }

	if(stopPt_srlr >= Min_LR && stopPt_srlr < 4096) { recoveryPt_srlr = minusDegree; }
	else if(stopPt_srlr >= 0 && stopPt_srlr <= Max_LR) { recoveryPt_srlr = plusDegree; }
#endif	

	if(startPt_srlr == minusDegree)
	{
		if(recoveryPt_srlr == minusDegree)
		{
			if(lin_Pt->FrPsPt.prevPt_FR_SRLR < stopPt_srlr) { recoveryPattern_srlr = Cant_go_to_zeroPt; }
			else { recoveryPattern_srlr = Repeat_zeroPt; }
		}
		else if(recoveryPt_srlr == plusDegree) { recoveryPattern_srlr = Repeat_zeroPt; }
	}
	else if(startPt_srlr == plusDegree)
	{
		if(recoveryPt_srlr == plusDegree)
		{
			if(lin_Pt->FrPsPt.prevPt_FR_SRLR < stopPt_srlr) { recoveryPattern_srlr = Repeat_zeroPt; }
			else { recoveryPattern_srlr = Cant_go_to_zeroPt; }
		}
		else if(recoveryPt_srlr == minusDegree) { recoveryPattern_srlr = Repeat_zeroPt; }
	}
}

void Handle_Recovery_StopPtCheck_srud(Lin_EVntPt_t *lin_Pt)
{
	//					canCenterPt = GET_CanCenterPt_CLLR(&Can_data_23_Info);
	//					CanLinValue[FrDrCtrPt_X].Pt_X.Can_RangePt_Center_X;
	
	if(fullcloseModeFlag_srud == TRUE)
	{
		if(lin_Pt->FrPsPt.prevPt_FR_SRUD >= FullClose_UD_Rev && lin_Pt->FrPsPt.prevPt_FR_SRUD < 4096) { startPt_srud = minusDegree; }
		else if(lin_Pt->FrPsPt.prevPt_FR_SRUD >= 0 && lin_Pt->FrPsPt.prevPt_FR_SRUD <= Min_UD) { startPt_srud = plusDegree; }

		if(stopPt_srud >= FullClose_UD_Rev && stopPt_srud < 4096) { recoveryPt_srud = minusDegree; }
		else if(stopPt_srud >= 0 && stopPt_srud <= Min_UD) { recoveryPt_srud = plusDegree; }
	}
	else
	{
		if(lin_Pt->FrPsPt.prevPt_FR_SRUD >= Max_UD && lin_Pt->FrPsPt.prevPt_FR_SRUD < 4096) { startPt_srud = minusDegree; }
		else if(lin_Pt->FrPsPt.prevPt_FR_SRUD >= 0 && lin_Pt->FrPsPt.prevPt_FR_SRUD <= Min_UD) { startPt_srud = plusDegree; }

		if(stopPt_srud >= Max_UD && stopPt_srud < 4096) { recoveryPt_srud = minusDegree; }
		else if(stopPt_srud >= 0 && stopPt_srud <= Min_UD) { recoveryPt_srud = plusDegree; }
	}

	//					if(lin_Pt->FrDrPt.prevPt_FR_CLLR <= canCenterPt)
	if(startPt_srud == minusDegree)
	{
	//						if(stopPt <= canCenterPt)
		if(recoveryPt_srud == minusDegree)
		{
			if(lin_Pt->FrPsPt.prevPt_FR_SRUD < stopPt_srud) { recoveryPattern_srud = Cant_go_to_zeroPt; }
			else { recoveryPattern_srud = Repeat_zeroPt; }
		}
		else if(recoveryPt_srud == plusDegree) { recoveryPattern_srud = Repeat_zeroPt; }
	}
	else if(startPt_srud == plusDegree)
	{
		if(recoveryPt_srud == plusDegree)
		{
			if(lin_Pt->FrPsPt.prevPt_FR_SRUD < stopPt_srud) { recoveryPattern_srud = Repeat_zeroPt; }
			else { recoveryPattern_srud = Cant_go_to_zeroPt; }
		}
		else if(recoveryPt_srud == minusDegree) { recoveryPattern_srud = Repeat_zeroPt; }
	}
}

void Recovery_CycleMode_ReStart(void)
{
	if(FrDrCycleMode_Flag == TRUE)
	{ 
		FrDrCycleMode_Flag = FALSE;
		lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD); 
	}
	
	if(FrPsCycleMode_Flag == TRUE)
	{ 
		FrPsCycleMode_Flag = FALSE;
		lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD); 
	}
	
	if(FrDrPsCycleMode_Flag == TRUE)
	{ 
		FrDrPsCycleMode_Flag = FALSE;
		lin_FrDrPsCycleMode_task(); 
	}
}


void Recovery_CantGoToZeroPt_ClearFunc_sllr(void)
{
	Recovery_Clear_FlagCountReset_sllr();

  if(cycleModeFlag_sllr == TRUE) { Recovery_CycleMode_ReStart(); }
  else 
	{ 
		Steploss_OpDone_TimeOutFlag_sllr = TRUE;
		Recovery_goToTargetPt_FR_SLLR_1(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT); 
	}
}

void Recovery_CantGoToZeroPt_ClearFunc_slud(void)
{
	Recovery_Clear_FlagCountReset_slud();

  if(cycleModeFlag_slud == TRUE) { Recovery_CycleMode_ReStart(); }
  else 
	{ 
		Steploss_OpDone_TimeOutFlag_slud = TRUE;
		Recovery_goToTargetPt_FR_SLUD_1(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT); 
	}
}


void Recovery_CantGoToZeroPt_ClearFunc_cllr(void)
{
	Recovery_Clear_FlagCountReset_cllr();

	if(cycleModeFlag_cllr == TRUE) { Recovery_CycleMode_ReStart(); }
	else 
	{ 
		Steploss_OpDone_TimeOutFlag_cllr = TRUE;
		Recovery_goToTargetPt_FR_CLLR_1(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT); 
	}
}

void Recovery_CantGoToZeroPt_ClearFunc_clud(void)
{
	Recovery_Clear_FlagCountReset_clud();

	if(cycleModeFlag_clud == TRUE) { Recovery_CycleMode_ReStart(); }
	else 
	{ 
		Steploss_OpDone_TimeOutFlag_clud = TRUE;
		Recovery_goToTargetPt_FR_CLUD_1(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT); 
	}
}

void Recovery_CantGoToZeroPt_ClearFunc_crlr(void)
{
	Recovery_Clear_FlagCountReset_crlr();

	if(cycleModeFlag_crlr == TRUE) { Recovery_CycleMode_ReStart(); }
	else 
	{ 
		Steploss_OpDone_TimeOutFlag_crlr = TRUE;
		Recovery_goToTargetPt_FR_CRLR_1(&LIN_RH_EVNT_MASTER_CMD, &LIN_EVNT_PT); 
	}
}

void Recovery_CantGoToZeroPt_ClearFunc_crud(void)
{
	Recovery_Clear_FlagCountReset_crud();

	if(cycleModeFlag_crud == TRUE) { Recovery_CycleMode_ReStart(); }
	else 
	{ 
		Steploss_OpDone_TimeOutFlag_crud = TRUE;
		Recovery_goToTargetPt_FR_CRUD_1(&LIN_RH_EVNT_MASTER_CMD, &LIN_EVNT_PT); 
	}
}

void Recovery_CantGoToZeroPt_ClearFunc_srlr(void)
{
	Recovery_Clear_FlagCountReset_srlr();

	if(cycleModeFlag_srlr == TRUE) { Recovery_CycleMode_ReStart(); }
	else 
	{ 
		Steploss_OpDone_TimeOutFlag_srlr = TRUE;
		Recovery_goToTargetPt_FR_SRLR_1(&LIN_RH_EVNT_MASTER_CMD, &LIN_EVNT_PT); 
	}
}

void Recovery_CantGoToZeroPt_ClearFunc_srud(void)
{
	Recovery_Clear_FlagCountReset_srud();

	if(cycleModeFlag_srud == TRUE) { Recovery_CycleMode_ReStart(); }
	else 
	{ 
		Steploss_OpDone_TimeOutFlag_srud = TRUE;
		Recovery_goToTargetPt_FR_SRUD_1(&LIN_RH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
//		Recovery_Clear_goToTargetPt_FR_SRUD_1(&LIN_RH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
	}
}


void Recovery_Repeat_zeroPt_ClearFunc_cllr(void)
{
	if(goToTargetFlag_cllr == TRUE)
	{
		goToTargetFlag_cllr = FALSE;
		
		Recovery_Clear_FlagCountReset_cllr();

		if(cycleModeFlag_cllr == TRUE) { Recovery_CycleMode_ReStart(); }
	}
}

void Recovery_Repeat_zeroPt_ClearFunc_clud(void)
{
	if(goToTargetFlag_clud == TRUE)
	{
		goToTargetFlag_clud = FALSE;
		
		Recovery_Clear_FlagCountReset_clud();

		if(cycleModeFlag_clud == TRUE) { Recovery_CycleMode_ReStart(); }
	}
}

void Recovery_Repeat_zeroPt_ClearFunc_sllr(void)
{
	if(goToTargetFlag_sllr == TRUE)
	{
		goToTargetFlag_sllr = FALSE;
		
		Recovery_Clear_FlagCountReset_sllr();

		if(cycleModeFlag_sllr == TRUE) { Recovery_CycleMode_ReStart(); }
	}
}

void Recovery_Repeat_zeroPt_ClearFunc_slud(void)
{
	if(goToTargetFlag_slud == TRUE)
	{
		goToTargetFlag_slud = FALSE;
		
		Recovery_Clear_FlagCountReset_slud();

		if(cycleModeFlag_slud == TRUE) { Recovery_CycleMode_ReStart(); }
	}
}

void Recovery_Repeat_zeroPt_ClearFunc_crlr(void)
{
	if(goToTargetFlag_crlr == TRUE)
	{
		goToTargetFlag_crlr = FALSE;
		
		Recovery_Clear_FlagCountReset_crlr();

		if(cycleModeFlag_crlr == TRUE) { Recovery_CycleMode_ReStart(); }
	}
}

void Recovery_Repeat_zeroPt_ClearFunc_crud(void)
{
	if(goToTargetFlag_crud == TRUE)
	{
		goToTargetFlag_crud = FALSE;
		
		Recovery_Clear_FlagCountReset_crud();

		if(cycleModeFlag_crud == TRUE) { Recovery_CycleMode_ReStart(); }
	}
}

void Recovery_Repeat_zeroPt_ClearFunc_srlr(void)
{
	if(goToTargetFlag_srlr == TRUE)
	{
		goToTargetFlag_srlr = FALSE;
		
		Recovery_Clear_FlagCountReset_srlr();

		if(cycleModeFlag_srlr == TRUE) { Recovery_CycleMode_ReStart(); }
	}
}

void Recovery_Repeat_zeroPt_ClearFunc_srud(void)
{
	if(goToTargetFlag_srud == TRUE)
	{
		goToTargetFlag_srud = FALSE;
		
		Recovery_Clear_FlagCountReset_srud();

		if(cycleModeFlag_srud == TRUE) { Recovery_CycleMode_ReStart(); }
	}
}


void Recovery_Pattern_CantGoToZeroPt_cllr(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
	
//	if(zero_flag_cllr == FALSE && eventCountTest_cllr >= 1)
	{
		if((lin_Status->fr_cllr_status.LDATA.FR_CLLR_Steploss == 1) && (absDeltaPt_cllr>0 && absDeltaPt_cllr<=80))
		{
			steplossCount_1_cllr++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6

			if(Check300ms_flag_1_cllr == FALSE)
			{			
				Check300ms_flag_1_cllr = TRUE;
				
				Amo_timer_Stop(timer_91);
				Amo_timer_Start(timer_91, 300, false, Check_300ms_5_cllr);
			}
		}
		else if((lin_Status->fr_cllr_status.LDATA.FR_CLLR_Steploss == 0) && (lin_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone == 1)) //recovery Clear
		{
			Recovery_CantGoToZeroPt_ClearFunc_cllr();
		}
	}
}

void Recovery_Pattern_CantGoToZeroPt_clud(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
	
//	if(zero_flag_cllr == FALSE && eventCountTest_cllr >= 1)
	{
		if((lin_Status->fr_clud_status.LDATA.FR_CLUD_Steploss == 1) && (absDeltaPt_clud>0 && absDeltaPt_clud<=80))
		{
			steplossCount_1_clud++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6

			if(Check300ms_flag_1_clud == FALSE)
			{			
				Check300ms_flag_1_clud = TRUE;
				
				Amo_timer_Stop(timer_96);
				Amo_timer_Start(timer_96, 300, false, Check_300ms_5_clud);
			}
		}
		else if((lin_Status->fr_clud_status.LDATA.FR_CLUD_Steploss == 0) && (lin_Status->fr_clud_status.LDATA.FR_CLUD_OpDone == 1)) //recovery Clear
		{
			Recovery_CantGoToZeroPt_ClearFunc_clud();
		}
	}
}

void Recovery_Pattern_CantGoToZeroPt_sllr(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
	
//	if(zero_flag_cllr == FALSE && eventCountTest_cllr >= 1)
	{
		if((lin_Status->fr_sllr_status.LDATA.FR_SLLR_Steploss == 1) && (absDeltaPt_sllr>0 && absDeltaPt_sllr<=80))
		{
			steplossCount_1_sllr++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6

			if(Check300ms_flag_1_sllr == FALSE)
			{			
				Check300ms_flag_1_sllr = TRUE;
				
				Amo_timer_Stop(timer_106);
				Amo_timer_Start(timer_106, 300, false, Check_300ms_5_sllr);
			}
		}
		else if((lin_Status->fr_sllr_status.LDATA.FR_SLLR_Steploss == 0) && (lin_Status->fr_sllr_status.LDATA.FR_SLLR_OpDone == 1)) //recovery Clear
		{
			Recovery_CantGoToZeroPt_ClearFunc_sllr();
		}
	}
}

void Recovery_Pattern_CantGoToZeroPt_slud(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
	
//	if(zero_flag_cllr == FALSE && eventCountTest_cllr >= 1)
	{
		if((lin_Status->fr_slud_status.LDATA.FR_SLUD_Steploss == 1) && (absDeltaPt_slud>0 && absDeltaPt_slud<=80))
		{
			steplossCount_1_slud++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6

			if(Check300ms_flag_1_slud == FALSE)
			{			
				Check300ms_flag_1_slud = TRUE;
				
				Amo_timer_Stop(timer_107);
				Amo_timer_Start(timer_107, 300, false, Check_300ms_5_slud);
			}
		}
		else if((lin_Status->fr_slud_status.LDATA.FR_SLUD_Steploss == 0) && (lin_Status->fr_slud_status.LDATA.FR_SLUD_OpDone == 1)) //recovery Clear
		{
			Recovery_CantGoToZeroPt_ClearFunc_slud();
		}
	}
}


void Recovery_Pattern_CantGoToZeroPt_crlr(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
	
//	if(zero_flag_cllr == FALSE && eventCountTest_cllr >= 1)
	{
		if((lin_Status->fr_crlr_status.LDATA.FR_CRLR_Steploss == 1) && (absDeltaPt_crlr>0 && absDeltaPt_crlr<=80))
		{
			steplossCount_1_crlr++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6

			if(Check300ms_flag_1_crlr == FALSE)
			{			
				Check300ms_flag_1_crlr = TRUE;
				
				Amo_timer_Stop(timer_108);
				Amo_timer_Start(timer_108, 300, false, Check_300ms_5_crlr);
			}
		}
		else if((lin_Status->fr_crlr_status.LDATA.FR_CRLR_Steploss == 0) && (lin_Status->fr_crlr_status.LDATA.FR_CRLR_OpDone == 1)) //recovery Clear
		{
			Recovery_CantGoToZeroPt_ClearFunc_crlr();
		}
	}
}

void Recovery_Pattern_CantGoToZeroPt_crud(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
	
//	if(zero_flag_cllr == FALSE && eventCountTest_cllr >= 1)
	{
		if((lin_Status->fr_crud_status.LDATA.FR_CRUD_Steploss == 1) && (absDeltaPt_crud>0 && absDeltaPt_crud<=80))
		{
			steplossCount_1_crud++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6

			if(Check300ms_flag_1_crud == FALSE)
			{			
				Check300ms_flag_1_crud = TRUE;
				
				Amo_timer_Stop(timer_109);
				Amo_timer_Start(timer_109, 300, false, Check_300ms_5_crud);
			}
		}
		else if((lin_Status->fr_crud_status.LDATA.FR_CRUD_Steploss == 0) && (lin_Status->fr_crud_status.LDATA.FR_CRUD_OpDone == 1)) //recovery Clear
		{
			Recovery_CantGoToZeroPt_ClearFunc_crud();
		}
	}
}

void Recovery_Pattern_CantGoToZeroPt_srlr(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
	
//	if(zero_flag_cllr == FALSE && eventCountTest_cllr >= 1)
	{
		if((lin_Status->fr_srlr_status.LDATA.FR_SRLR_Steploss == 1) && (absDeltaPt_srlr>0 && absDeltaPt_srlr<=80))
		{
			steplossCount_1_srlr++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6

			if(Check300ms_flag_1_srlr == FALSE)
			{			
				Check300ms_flag_1_srlr = TRUE;
				
				Amo_timer_Stop(timer_110);
				Amo_timer_Start(timer_110, 300, false, Check_300ms_5_srlr);
			}
		}
		else if((lin_Status->fr_srlr_status.LDATA.FR_SRLR_Steploss == 0) && (lin_Status->fr_srlr_status.LDATA.FR_SRLR_OpDone == 1)) //recovery Clear
		{
			Recovery_CantGoToZeroPt_ClearFunc_srlr();
		}
	}
}


void Recovery_Pattern_CantGoToZeroPt_srud(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
	
//	if(zero_flag_cllr == FALSE && eventCountTest_cllr >= 1)
	{
		if((lin_Status->fr_srud_status.LDATA.FR_SRUD_Steploss == 1) && (absDeltaPt_srud>0 && absDeltaPt_srud<=80))
		{
			steplossCount_1_srud++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6

			if(Check300ms_flag_1_srud == FALSE)
			{			
				Check300ms_flag_1_srud = TRUE;
				
				Amo_timer_Stop(timer_111);
				Amo_timer_Start(timer_111, 300, false, Check_300ms_5_srud);
			}
		}
		else if((lin_Status->fr_srud_status.LDATA.FR_SRUD_Steploss == 0) && (lin_Status->fr_srud_status.LDATA.FR_SRUD_OpDone == 1)) //recovery Clear
		{
			Recovery_CantGoToZeroPt_ClearFunc_srud();
		}
	}
}



void Recovery_Pattern_Repeat_zeroPt_cllr(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
//	if(zero_flag_cllr == TRUE && eventCountTest_cllr >= 1)
	{	
		if(lin_Status->fr_cllr_status.LDATA.FR_CLLR_Steploss == 1)// && (absDeltaPt>0 && absDeltaPt<=80))
		{
	//				zero_flag = 0;

			steplossCount_1_cllr++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6

			if(Check300ms_flag_1_cllr == FALSE)
			{			
				Check300ms_flag_1_cllr = TRUE;
				
				Amo_timer_Stop(timer_91);
				Amo_timer_Start(timer_91, 300, false, Check_300ms_5_cllr);
			}
		}
		else if((lin_Status->fr_cllr_status.LDATA.FR_CLLR_Steploss == 0) && (lin_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone == 1)) //recovery Clear
		{
			Recovery_Repeat_zeroPt_ClearFunc_cllr();
		}
		
		if((lin_Status->fr_cllr_status.LDATA.FR_CLLR_Steploss == 0) && (recoveryModeFlag_cllr == TRUE))
		{
			if((lin_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone == true)) //&& ((zeroPt >= 0 && zeroPt < 30) || (zeroPt > 2038 && zeroPt < 2048)))
			{

				zero_flag_cllr = TRUE;

				recoveryModeFlag_cllr = FALSE;
								
				gotoTargetPtCount_cllr++;

				recoveryFlags.FR_CLLR_Recov_Flag = TRUE;

				gotoZeroPtFlag_cllr = FALSE;

				Amo_timer_Stop(timer_94);
				Amo_timer_Start(timer_94, 300, false, GO_To_TargetPt_cllr); //50 //1000
				
			}
		}
	}
}


void Recovery_Pattern_Repeat_zeroPt_clud(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
//	if(zero_flag_cllr == TRUE && eventCountTest_cllr >= 1)
	{	
		if(lin_Status->fr_clud_status.LDATA.FR_CLUD_Steploss == 1)// && (absDeltaPt>0 && absDeltaPt<=80))
		{
	//				zero_flag = 0;

			steplossCount_1_clud++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6

			if(Check300ms_flag_1_clud == FALSE)
			{			
				Check300ms_flag_1_clud = TRUE;
				
				Amo_timer_Stop(timer_96);
				Amo_timer_Start(timer_96, 300, false, Check_300ms_5_clud);
			}
		}
		else if((lin_Status->fr_clud_status.LDATA.FR_CLUD_Steploss == 0) && (lin_Status->fr_clud_status.LDATA.FR_CLUD_OpDone == 1)) //recovery Clear
		{
			Recovery_Repeat_zeroPt_ClearFunc_clud();
		}
		
		if((lin_Status->fr_clud_status.LDATA.FR_CLUD_Steploss == 0) && (recoveryModeFlag_clud == TRUE))
		{
			if((lin_Status->fr_clud_status.LDATA.FR_CLUD_OpDone == true)) //&& ((zeroPt >= 0 && zeroPt < 30) || (zeroPt > 2038 && zeroPt < 2048)))
			{

				zero_flag_clud = TRUE;

				recoveryModeFlag_clud = FALSE;
								
				gotoTargetPtCount_clud++;

				recoveryFlags.FR_CLUD_Recov_Flag = TRUE;

				gotoZeroPtFlag_clud = FALSE;

				Amo_timer_Stop(timer_97);
				Amo_timer_Start(timer_97, 300, false, GO_To_TargetPt_clud); //50 //1000
				
			}
		}
	}
}

void Recovery_Pattern_Repeat_zeroPt_sllr(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
//	if(zero_flag_cllr == TRUE && eventCountTest_cllr >= 1)
	{	
		if(lin_Status->fr_sllr_status.LDATA.FR_SLLR_Steploss == 1)// && (absDeltaPt>0 && absDeltaPt<=80))
		{
	//				zero_flag = 0;

			steplossCount_1_sllr++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6

			if(Check300ms_flag_1_sllr == FALSE)
			{			
				Check300ms_flag_1_sllr = TRUE;
				
				Amo_timer_Stop(timer_106);
				Amo_timer_Start(timer_106, 300, false, Check_300ms_5_sllr);
			}
		}
		else if((lin_Status->fr_sllr_status.LDATA.FR_SLLR_Steploss == 0) && (lin_Status->fr_sllr_status.LDATA.FR_SLLR_OpDone == 1)) //recovery Clear
		{
			Recovery_Repeat_zeroPt_ClearFunc_sllr();
		}
		
		if((lin_Status->fr_sllr_status.LDATA.FR_SLLR_Steploss == 0) && (recoveryModeFlag_sllr == TRUE))
		{
			if((lin_Status->fr_sllr_status.LDATA.FR_SLLR_OpDone == true)) //&& ((zeroPt >= 0 && zeroPt < 30) || (zeroPt > 2038 && zeroPt < 2048)))
			{

				zero_flag_sllr = TRUE;

				recoveryModeFlag_sllr = FALSE;
								
				gotoTargetPtCount_sllr++;

				recoveryFlags.FR_SLLR_Recov_Flag = TRUE;

				gotoZeroPtFlag_sllr = FALSE;

				Amo_timer_Stop(timer_112);
				Amo_timer_Start(timer_112, 300, false, GO_To_TargetPt_sllr); //50 //1000
				
			}
		}
	}
}

void Recovery_Pattern_Repeat_zeroPt_slud(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
//	if(zero_flag_cllr == TRUE && eventCountTest_cllr >= 1)
	{	
		if(lin_Status->fr_slud_status.LDATA.FR_SLUD_Steploss == 1)// && (absDeltaPt>0 && absDeltaPt<=80))
		{
	//				zero_flag = 0;

			steplossCount_1_slud++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6

			if(Check300ms_flag_1_slud == FALSE)
			{			
				Check300ms_flag_1_slud = TRUE;
				
				Amo_timer_Stop(timer_107);
				Amo_timer_Start(timer_107, 300, false, Check_300ms_5_slud);
			}
		}
		else if((lin_Status->fr_slud_status.LDATA.FR_SLUD_Steploss == 0) && (lin_Status->fr_slud_status.LDATA.FR_SLUD_OpDone == 1)) //recovery Clear
		{
			Recovery_Repeat_zeroPt_ClearFunc_slud();
		}
		
		if((lin_Status->fr_slud_status.LDATA.FR_SLUD_Steploss == 0) && (recoveryModeFlag_slud == TRUE))
		{
			if((lin_Status->fr_slud_status.LDATA.FR_SLUD_OpDone == true)) //&& ((zeroPt >= 0 && zeroPt < 30) || (zeroPt > 2038 && zeroPt < 2048)))
			{

				zero_flag_slud = TRUE;

				recoveryModeFlag_slud = FALSE;
								
				gotoTargetPtCount_slud++;

				recoveryFlags.FR_SLUD_Recov_Flag = TRUE;

				gotoZeroPtFlag_slud = FALSE;

				Amo_timer_Stop(timer_113);
				Amo_timer_Start(timer_113, 300, false, GO_To_TargetPt_slud); //50 //1000
				
			}
		}
	}
}

void Recovery_Pattern_Repeat_zeroPt_crlr(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
//	if(zero_flag_cllr == TRUE && eventCountTest_cllr >= 1)
	{	
		if(lin_Status->fr_crlr_status.LDATA.FR_CRLR_Steploss == 1)// && (absDeltaPt>0 && absDeltaPt<=80))
		{
	//				zero_flag = 0;

			steplossCount_1_crlr++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6

			if(Check300ms_flag_1_crlr == FALSE)
			{			
				Check300ms_flag_1_crlr = TRUE;
				
				Amo_timer_Stop(timer_108);
				Amo_timer_Start(timer_108, 300, false, Check_300ms_5_crlr);
			}
		}
		else if((lin_Status->fr_crlr_status.LDATA.FR_CRLR_Steploss == 0) && (lin_Status->fr_crlr_status.LDATA.FR_CRLR_OpDone == 1)) //recovery Clear
		{
			Recovery_Repeat_zeroPt_ClearFunc_crlr();
		}
		
		if((lin_Status->fr_crlr_status.LDATA.FR_CRLR_Steploss == 0) && (recoveryModeFlag_crlr == TRUE))
		{
			if((lin_Status->fr_crlr_status.LDATA.FR_CRLR_OpDone == true)) //&& ((zeroPt >= 0 && zeroPt < 30) || (zeroPt > 2038 && zeroPt < 2048)))
			{

				zero_flag_crlr = TRUE;

				recoveryModeFlag_crlr = FALSE;
								
				gotoTargetPtCount_crlr++;

				recoveryFlags.FR_CRLR_Recov_Flag = TRUE;

				gotoZeroPtFlag_crlr = FALSE;

				Amo_timer_Stop(timer_114);
				Amo_timer_Start(timer_114, 300, false, GO_To_TargetPt_crlr); //50 //1000
				
			}
		}
	}
}

void Recovery_Pattern_Repeat_zeroPt_crud(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
//	if(zero_flag_cllr == TRUE && eventCountTest_cllr >= 1)
	{	
		if(lin_Status->fr_crud_status.LDATA.FR_CRUD_Steploss == 1)// && (absDeltaPt>0 && absDeltaPt<=80))
		{
	//				zero_flag = 0;

			steplossCount_1_crud++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6

			if(Check300ms_flag_1_crud == FALSE)
			{			
				Check300ms_flag_1_crud = TRUE;
				
				Amo_timer_Stop(timer_109);
				Amo_timer_Start(timer_109, 300, false, Check_300ms_5_crud);
			}
		}
		else if((lin_Status->fr_crud_status.LDATA.FR_CRUD_Steploss == 0) && (lin_Status->fr_crud_status.LDATA.FR_CRUD_OpDone == 1)) //recovery Clear
		{
			Recovery_Repeat_zeroPt_ClearFunc_crud();
		}
		
		if((lin_Status->fr_crud_status.LDATA.FR_CRUD_Steploss == 0) && (recoveryModeFlag_crud == TRUE))
		{
			if((lin_Status->fr_crud_status.LDATA.FR_CRUD_OpDone == true)) //&& ((zeroPt >= 0 && zeroPt < 30) || (zeroPt > 2038 && zeroPt < 2048)))
			{

				zero_flag_crud = TRUE;

				recoveryModeFlag_crud = FALSE;
								
				gotoTargetPtCount_crud++;

				recoveryFlags.FR_CRUD_Recov_Flag = TRUE;

				gotoZeroPtFlag_crud = FALSE;

				Amo_timer_Stop(timer_115);
				Amo_timer_Start(timer_115, 300, false, GO_To_TargetPt_crud); //50 //1000
				
			}
		}
	}
}


void Recovery_Pattern_Repeat_zeroPt_srlr(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
//	if(zero_flag_cllr == TRUE && eventCountTest_cllr >= 1)
	{	
		if(lin_Status->fr_srlr_status.LDATA.FR_SRLR_Steploss == 1)// && (absDeltaPt>0 && absDeltaPt<=80))
		{
	//				zero_flag = 0;

			steplossCount_1_srlr++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6

			if(Check300ms_flag_1_srlr == FALSE)
			{			
				Check300ms_flag_1_srlr = TRUE;
				
				Amo_timer_Stop(timer_110);
				Amo_timer_Start(timer_110, 300, false, Check_300ms_5_srlr);
			}
		}
		else if((lin_Status->fr_srlr_status.LDATA.FR_SRLR_Steploss == 0) && (lin_Status->fr_srlr_status.LDATA.FR_SRLR_OpDone == 1)) //recovery Clear
		{
			Recovery_Repeat_zeroPt_ClearFunc_srlr();
		}
		
		if((lin_Status->fr_srlr_status.LDATA.FR_SRLR_Steploss == 0) && (recoveryModeFlag_srlr == TRUE))
		{
			if((lin_Status->fr_srlr_status.LDATA.FR_SRLR_OpDone == true)) //&& ((zeroPt >= 0 && zeroPt < 30) || (zeroPt > 2038 && zeroPt < 2048)))
			{

				zero_flag_srlr = TRUE;

				recoveryModeFlag_srlr = FALSE;
								
				gotoTargetPtCount_srlr++;

				recoveryFlags.FR_SRLR_Recov_Flag = TRUE;

				gotoZeroPtFlag_srlr = FALSE;

				Amo_timer_Stop(timer_116);
				Amo_timer_Start(timer_116, 300, false, GO_To_TargetPt_srlr); //50 //1000
				
			}
		}
	}
}

void Recovery_Pattern_Repeat_zeroPt_srud(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
//	if(zero_flag_cllr == TRUE && eventCountTest_cllr >= 1)
	{	
		if(lin_Status->fr_srud_status.LDATA.FR_SRUD_Steploss == 1)// && (absDeltaPt>0 && absDeltaPt<=80))
		{
	//				zero_flag = 0;

			steplossCount_1_srud++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6

			if(Check300ms_flag_1_srud == FALSE)
			{			
				Check300ms_flag_1_srud = TRUE;
				
				Amo_timer_Stop(timer_111);
				Amo_timer_Start(timer_111, 300, false, Check_300ms_5_srud);
			}
		}
		else if((lin_Status->fr_srud_status.LDATA.FR_SRUD_Steploss == 0) && (lin_Status->fr_srud_status.LDATA.FR_SRUD_OpDone == 1)) //recovery Clear
		{
			Recovery_Repeat_zeroPt_ClearFunc_srud();
		}
		
		if((lin_Status->fr_srud_status.LDATA.FR_SRUD_Steploss == 0) && (recoveryModeFlag_srud == TRUE))
		{
			if((lin_Status->fr_srud_status.LDATA.FR_SRUD_OpDone == true)) //&& ((zeroPt >= 0 && zeroPt < 30) || (zeroPt > 2038 && zeroPt < 2048)))
			{

				zero_flag_srud = TRUE;

				recoveryModeFlag_srud = FALSE;
								
				gotoTargetPtCount_srud++;

				recoveryFlags.FR_SRUD_Recov_Flag = TRUE;

				gotoZeroPtFlag_srud = FALSE;

				Amo_timer_Stop(timer_117);
				Amo_timer_Start(timer_117, 300, false, GO_To_TargetPt_srud); //50 //1000
				
			}
		}
	}
}



void Steploss_check_cllr(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
	
	if(l_flg_tst_LI0_FR_CLLR_Steploss_flag())
	{
		l_flg_clr_LI0_FR_CLLR_Steploss_flag();

		lin_Status->fr_cllr_status.LDATA.FR_CLLR_Steploss = l_bool_rd_LI0_FR_CLLR_Steploss();
		lin_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone = l_bool_rd_LI0_FR_CLLR_OpDone();

		currentPt_cllr = l_u16_rd_LI0_FR_CLLR_ActuatorState();

		deltaPt_cllr = currentPt_cllr - stopPt_cllr;
		absDeltaPt_cllr = absValue(deltaPt_cllr);

		if((eventCountTest_cllr == 0) && (recoveryFinishFlag_cllr == FALSE))
		{
			if(lin_Status->fr_cllr_status.LDATA.FR_CLLR_Steploss == 1) // && lin_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone == 0)
			{	
				steplossCount_1_cllr++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6, 350ms=7, 400ms=8, 450ms=9, 500ms=10

				if(Check300ms_flag_1_cllr == FALSE)
				{				
					stopPt_cllr = l_u16_rd_LI0_FR_CLLR_ActuatorState();
									
					Check300ms_flag_1_cllr = TRUE;

					Handle_Recovery_StopPtCheck_cllr(&LIN_EVNT_PT);
					
					Amo_timer_Stop(timer_91);
					Amo_timer_Start(timer_91, 300, false, Check_300ms_5_cllr);
				}
			}
		}

		if(recoveryPattern_cllr == Cant_go_to_zeroPt)
		{
			Recovery_Pattern_CantGoToZeroPt_cllr(&LIN_LH_FR_STATUS, &LIN_EVNT_PT);
		}


		if(recoveryPattern_cllr == Repeat_zeroPt)
		{
			Recovery_Pattern_Repeat_zeroPt_cllr(&LIN_LH_FR_STATUS, &LIN_EVNT_PT);
		}
	}
}


void Steploss_check_clud(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
	
	if(l_flg_tst_LI0_FR_CLUD_Steploss_flag())
	{
		l_flg_clr_LI0_FR_CLUD_Steploss_flag();

		lin_Status->fr_clud_status.LDATA.FR_CLUD_Steploss = l_bool_rd_LI0_FR_CLUD_Steploss();
		lin_Status->fr_clud_status.LDATA.FR_CLUD_OpDone = l_bool_rd_LI0_FR_CLUD_OpDone();

		currentPt_clud = l_u16_rd_LI0_FR_CLUD_ActuatorState();

		deltaPt_clud = currentPt_clud - stopPt_clud;
		absDeltaPt_clud = absValue(deltaPt_clud);
		
		if((eventCountTest_clud == 0) && (recoveryFinishFlag_clud == FALSE))
		{
			if(lin_Status->fr_clud_status.LDATA.FR_CLUD_Steploss == 1)// && lin_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone == 0)
			{	
				steplossCount_1_clud++;

				if(Check300ms_flag_1_clud == FALSE)
				{				
					stopPt_clud = l_u16_rd_LI0_FR_CLUD_ActuatorState();
									
					Check300ms_flag_1_clud = TRUE;

					Handle_Recovery_StopPtCheck_clud(&LIN_EVNT_PT);
					
					Amo_timer_Stop(timer_96);
					Amo_timer_Start(timer_96, 300, false, Check_300ms_5_clud);
				}
			}
		}

		if(recoveryPattern_clud == Cant_go_to_zeroPt)
		{
			Recovery_Pattern_CantGoToZeroPt_clud(&LIN_LH_FR_STATUS, &LIN_EVNT_PT);
		}


		if(recoveryPattern_clud == Repeat_zeroPt)
		{
			Recovery_Pattern_Repeat_zeroPt_clud(&LIN_LH_FR_STATUS, &LIN_EVNT_PT);
		}
	}
}


void Steploss_check_sllr(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
	
	if(l_flg_tst_LI0_FR_SLLR_Steploss_flag())
	{
		l_flg_clr_LI0_FR_SLLR_Steploss_flag();

		lin_Status->fr_sllr_status.LDATA.FR_SLLR_Steploss = l_bool_rd_LI0_FR_SLLR_Steploss();
		lin_Status->fr_sllr_status.LDATA.FR_SLLR_OpDone = l_bool_rd_LI0_FR_SLLR_OpDone();

		currentPt_sllr = l_u16_rd_LI0_FR_SLLR_ActuatorState();

		deltaPt_sllr = currentPt_sllr - stopPt_sllr;
		absDeltaPt_sllr = absValue(deltaPt_sllr);

		if((eventCountTest_sllr == 0) && (recoveryFinishFlag_sllr == FALSE))
		{
			if(lin_Status->fr_sllr_status.LDATA.FR_SLLR_Steploss == 1) // && lin_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone == 0)
			{	
				steplossCount_1_sllr++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6, 350ms=7, 400ms=8, 450ms=9, 500ms=10

				if(Check300ms_flag_1_sllr == FALSE)
				{				
					stopPt_sllr = l_u16_rd_LI0_FR_SLLR_ActuatorState();
									
					Check300ms_flag_1_sllr = TRUE;

					Handle_Recovery_StopPtCheck_sllr(&LIN_EVNT_PT);
					
					Amo_timer_Stop(timer_106);
					Amo_timer_Start(timer_106, 300, false, Check_300ms_5_sllr);
				}
			}
		}

		if(recoveryPattern_sllr == Cant_go_to_zeroPt)
		{
			Recovery_Pattern_CantGoToZeroPt_sllr(&LIN_LH_FR_STATUS, &LIN_EVNT_PT);
		}


		if(recoveryPattern_sllr == Repeat_zeroPt)
		{
			Recovery_Pattern_Repeat_zeroPt_sllr(&LIN_LH_FR_STATUS, &LIN_EVNT_PT);
		}
	}
}

void Steploss_check_slud(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
	
	if(l_flg_tst_LI0_FR_SLUD_Steploss_flag())
	{
		l_flg_clr_LI0_FR_SLUD_Steploss_flag();

		lin_Status->fr_slud_status.LDATA.FR_SLUD_Steploss = l_bool_rd_LI0_FR_SLUD_Steploss();
		lin_Status->fr_slud_status.LDATA.FR_SLUD_OpDone = l_bool_rd_LI0_FR_SLUD_OpDone();

		currentPt_slud = l_u16_rd_LI0_FR_SLUD_ActuatorState();

		deltaPt_slud = currentPt_slud - stopPt_slud;
		absDeltaPt_slud = absValue(deltaPt_slud);
		
		if((eventCountTest_slud == 0) && (recoveryFinishFlag_slud == FALSE))
		{
			if(lin_Status->fr_slud_status.LDATA.FR_SLUD_Steploss == 1)// && lin_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone == 0)
			{	
				steplossCount_1_slud++;

				if(Check300ms_flag_1_slud == FALSE)
				{				
					stopPt_slud = l_u16_rd_LI0_FR_SLUD_ActuatorState();
									
					Check300ms_flag_1_slud = TRUE;

					Handle_Recovery_StopPtCheck_slud(&LIN_EVNT_PT);
					
					Amo_timer_Stop(timer_107);
					Amo_timer_Start(timer_107, 300, false, Check_300ms_5_slud);
				}
			}
		}

		if(recoveryPattern_slud == Cant_go_to_zeroPt)
		{
			Recovery_Pattern_CantGoToZeroPt_slud(&LIN_LH_FR_STATUS, &LIN_EVNT_PT);
		}


		if(recoveryPattern_slud == Repeat_zeroPt)
		{
			Recovery_Pattern_Repeat_zeroPt_slud(&LIN_LH_FR_STATUS, &LIN_EVNT_PT);
		}

		if((eventCountTest_slud == 0) && (recoveryPattern_slud == Recovery_None))
		{

		}
	}
}

void Steploss_check_crlr(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
	
	if(l_flg_tst_LI1_FR_CRLR_Steploss_flag())
	{
		l_flg_clr_LI1_FR_CRLR_Steploss_flag();

		lin_Status->fr_crlr_status.LDATA.FR_CRLR_Steploss = l_bool_rd_LI1_FR_CRLR_Steploss();
		lin_Status->fr_crlr_status.LDATA.FR_CRLR_OpDone = l_bool_rd_LI1_FR_CRLR_OpDone();

		currentPt_crlr = l_u16_rd_LI1_FR_CRLR_ActuatorState();

		deltaPt_crlr = currentPt_crlr - stopPt_crlr;
		absDeltaPt_crlr = absValue(deltaPt_crlr);

		if((eventCountTest_crlr == 0) && (recoveryFinishFlag_crlr == FALSE))
		{
			if(lin_Status->fr_crlr_status.LDATA.FR_CRLR_Steploss == 1) // && lin_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone == 0)
			{	
				steplossCount_1_crlr++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6, 350ms=7, 400ms=8, 450ms=9, 500ms=10

				if(Check300ms_flag_1_crlr == FALSE)
				{				
					stopPt_crlr = l_u16_rd_LI1_FR_CRLR_ActuatorState();
									
					Check300ms_flag_1_crlr = TRUE;

					Handle_Recovery_StopPtCheck_crlr(&LIN_EVNT_PT);
					
					Amo_timer_Stop(timer_108);
					Amo_timer_Start(timer_108, 300, false, Check_300ms_5_crlr);
				}
			}
		}

		if(recoveryPattern_crlr == Cant_go_to_zeroPt)
		{
			Recovery_Pattern_CantGoToZeroPt_crlr(&LIN_RH_FR_STATUS, &LIN_EVNT_PT);
		}


		if(recoveryPattern_crlr == Repeat_zeroPt)
		{
			Recovery_Pattern_Repeat_zeroPt_crlr(&LIN_RH_FR_STATUS, &LIN_EVNT_PT);
		}
	}
}

void Steploss_check_crud(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
	
	if(l_flg_tst_LI1_FR_CRUD_Steploss_flag())
	{
		l_flg_clr_LI1_FR_CRUD_Steploss_flag();

		lin_Status->fr_crud_status.LDATA.FR_CRUD_Steploss = l_bool_rd_LI1_FR_CRUD_Steploss();
		lin_Status->fr_crud_status.LDATA.FR_CRUD_OpDone = l_bool_rd_LI1_FR_CRUD_OpDone();

		currentPt_crud = l_u16_rd_LI1_FR_CRUD_ActuatorState();

		deltaPt_crud = currentPt_crud - stopPt_crud;
		absDeltaPt_crud = absValue(deltaPt_crud);
		
		if((eventCountTest_crud == 0) && (recoveryFinishFlag_crud == FALSE))
		{
			if(lin_Status->fr_crud_status.LDATA.FR_CRUD_Steploss == 1)// && lin_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone == 0)
			{	
				steplossCount_1_crud++;

				if(Check300ms_flag_1_crud == FALSE)
				{				
					stopPt_crud = l_u16_rd_LI1_FR_CRUD_ActuatorState();
									
					Check300ms_flag_1_crud = TRUE;

					Handle_Recovery_StopPtCheck_crud(&LIN_EVNT_PT);
					
					Amo_timer_Stop(timer_109);
					Amo_timer_Start(timer_109, 300, false, Check_300ms_5_crud);
				}
			}
		}

		if(recoveryPattern_crud == Cant_go_to_zeroPt)
		{
			Recovery_Pattern_CantGoToZeroPt_crud(&LIN_RH_FR_STATUS, &LIN_EVNT_PT);
		}


		if(recoveryPattern_crud == Repeat_zeroPt)
		{
			Recovery_Pattern_Repeat_zeroPt_crud(&LIN_RH_FR_STATUS, &LIN_EVNT_PT);
		}
	}
}

void Steploss_check_srlr(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
	
	if(l_flg_tst_LI1_FR_SRLR_Steploss_flag())
	{
		l_flg_clr_LI1_FR_SRLR_Steploss_flag();

		lin_Status->fr_srlr_status.LDATA.FR_SRLR_Steploss = l_bool_rd_LI1_FR_SRLR_Steploss();
		lin_Status->fr_srlr_status.LDATA.FR_SRLR_OpDone = l_bool_rd_LI1_FR_SRLR_OpDone();

		currentPt_srlr = l_u16_rd_LI1_FR_SRLR_ActuatorState();

		deltaPt_srlr = currentPt_srlr - stopPt_srlr;
		absDeltaPt_srlr = absValue(deltaPt_srlr);

		if((eventCountTest_srlr == 0) && (recoveryFinishFlag_srlr == FALSE))
		{
			if(lin_Status->fr_srlr_status.LDATA.FR_SRLR_Steploss == 1) // && lin_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone == 0)
			{	
				steplossCount_1_srlr++; //50ms=1 , 100ms=2, 150ms=3, 200ms=4, 250ms=5, 300ms=6, 350ms=7, 400ms=8, 450ms=9, 500ms=10

				if(Check300ms_flag_1_srlr == FALSE)
				{				
					stopPt_srlr = l_u16_rd_LI1_FR_SRLR_ActuatorState();
									
					Check300ms_flag_1_srlr = TRUE;

					Handle_Recovery_StopPtCheck_srlr(&LIN_EVNT_PT);
					
					Amo_timer_Stop(timer_110);
					Amo_timer_Start(timer_110, 300, false, Check_300ms_5_srlr);
				}
			}
		}

		if(recoveryPattern_srlr == Cant_go_to_zeroPt)
		{
			Recovery_Pattern_CantGoToZeroPt_srlr(&LIN_RH_FR_STATUS, &LIN_EVNT_PT);
		}


		if(recoveryPattern_srlr == Repeat_zeroPt)
		{
			Recovery_Pattern_Repeat_zeroPt_srlr(&LIN_RH_FR_STATUS, &LIN_EVNT_PT);
		}
	}
}


void Steploss_check_srud(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
	
	if(l_flg_tst_LI1_FR_SRUD_Steploss_flag())
	{
		l_flg_clr_LI1_FR_SRUD_Steploss_flag();

		lin_Status->fr_srud_status.LDATA.FR_SRUD_Steploss = l_bool_rd_LI1_FR_SRUD_Steploss();
		lin_Status->fr_srud_status.LDATA.FR_SRUD_OpDone = l_bool_rd_LI1_FR_SRUD_OpDone();

		currentPt_srud = l_u16_rd_LI1_FR_SRUD_ActuatorState();

		deltaPt_srud = currentPt_srud - stopPt_srud;
		absDeltaPt_srud = absValue(deltaPt_srud);
		
		if((eventCountTest_srud == 0) && (recoveryFinishFlag_srud == FALSE))
		{
			if(lin_Status->fr_srud_status.LDATA.FR_SRUD_Steploss == 1)// && lin_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone == 0)
			{	
				steplossCount_1_srud++;

				if(Check300ms_flag_1_srud == FALSE)
				{				
					stopPt_srud = l_u16_rd_LI1_FR_SRUD_ActuatorState();
									
					Check300ms_flag_1_srud = TRUE;

					Handle_Recovery_StopPtCheck_srud(&LIN_EVNT_PT);
					
					Amo_timer_Stop(timer_111);
					Amo_timer_Start(timer_111, 300, false, Check_300ms_5_srud);
				}
			}
		}

		if(recoveryPattern_srud == Cant_go_to_zeroPt)
		{
			Recovery_Pattern_CantGoToZeroPt_srud(&LIN_RH_FR_STATUS, &LIN_EVNT_PT);
		}


		if(recoveryPattern_srud == Repeat_zeroPt)
		{
			Recovery_Pattern_Repeat_zeroPt_srud(&LIN_RH_FR_STATUS, &LIN_EVNT_PT);
		}
	}
}


uint8_t steplossTimeOutCount_srud = 0;
uint8_t Check_5000ms_flag_srud = FALSE;


void Check_5000ms_srud(void)
{
    Amo_timer_Stop(timer_125);

    if(steplossTimeOutCount_srud >= 9)
    {
			steplossTimeOutCount_srud = 0;
			
      goToTargetFlag_srud = FALSE;

      eventCountTest_srud = 0;

      recoveryModeFlag_srud = FALSE;

      Check300ms_flag_srud = FALSE;
      Check300ms_flag_1_srud = FALSE;
      Check300ms_flag_2_srud = FALSE;
      Check300ms_flag_3_srud = FALSE;
      Check300ms_flag_4_srud = FALSE;
      Check300ms_flag_5_srud = FALSE;

      gotoTargetPtCount_srud = 0;
      gotoZeroPtCount_srud = 0;

      gotoZeroPtFlag_srud = FALSE;

      steplossCount_srud = 0;
      steplossCount_1_srud = 0;
      steplossCount_2_srud = 0;
      steplossCount_3_srud = 0;
      steplossCount_4_srud = 0;
      steplossCount_5_srud = 0;

      lastTryFlag_srud = TRUE;
      recoveryFlags.FR_SRUD_Recov_Flag = TRUE;

      Lin_Scheduler_SpecialCmd_Start_1();
    }
    
    else
    {
	    Check_5000ms_flag_srud = FALSE;
	    steplossTimeOutCount_srud = 0;
    }
}



void Steploss_TimeOut_Stop(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt)
{
	if(l_flg_tst_LI1_FR_SRUD_Steploss_flag())
	{
		l_flg_clr_LI1_FR_SRUD_Steploss_flag();
		
		lin_Status->fr_srud_status.LDATA.FR_SRUD_Steploss = l_bool_rd_LI1_FR_SRUD_Steploss();

		if(lin_Status->fr_srud_status.LDATA.FR_SRUD_Steploss == 1)
		{
			steplossTimeOutCount_srud++;

			if(Check_5000ms_flag_srud == FALSE)
			{
				Check_5000ms_flag_srud = TRUE;

				Amo_timer_Stop(timer_125);
				Amo_timer_Start(timer_125, 5000, false, Check_5000ms_srud);
			}
		}
		
		if(lin_Status->fr_srud_status.LDATA.FR_SRUD_Steploss == 0)
		{
			Check_5000ms_flag_srud = FALSE;
	    steplossTimeOutCount_srud = 0;
		}
		
	}
}


uint16_t GET_CanCenterPt_CLLR(Atcu_23_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_X;

	result = Cal_CAN_CenterPt(&data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t GET_CanCenterPt_CLUD(Atcu_23_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_Y;

	result = Cal_CAN_CenterPt(&data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t GET_CanCenterPt_SLLR(Atcu_23_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_X;

	result = Cal_CAN_CenterPt(&data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t GET_CanCenterPt_SLUD(Atcu_23_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_Y;

	result = Cal_CAN_CenterPt(&data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t GET_CanCenterPt_CRLR(Atcu_24_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_X;

	result = Cal_CAN_CenterPt(&data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t GET_CanCenterPt_CRUD(Atcu_24_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_Y;

	result = Cal_CAN_CenterPt(&data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t GET_CanCenterPt_SRLR(Atcu_24_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_X;

	result = Cal_CAN_CenterPt(&data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t GET_CanCenterPt_SRUD(Atcu_24_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_Y;

	result = Cal_CAN_CenterPt(&data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}


void Recovery_Clear_FlagCountReset_cllr(void)
{
	eventCountTest_cllr = 0;

	recoveryModeFlag_cllr = FALSE;

	Check300ms_flag_cllr = FALSE;
	Check300ms_flag_1_cllr = FALSE;
	Check300ms_flag_2_cllr = FALSE;
	Check300ms_flag_3_cllr = FALSE;
	Check300ms_flag_4_cllr = FALSE;
	Check300ms_flag_5_cllr = FALSE;

	gotoTargetPtCount_cllr = 0;
	gotoZeroPtCount_cllr = 0;

	gotoZeroPtFlag_cllr = FALSE;

	steplossCount_cllr = 0;
	steplossCount_1_cllr = 0;
	steplossCount_2_cllr = 0;
	steplossCount_3_cllr = 0;
	steplossCount_4_cllr = 0;
	steplossCount_5_cllr = 0;

//	lastTryFlag = TRUE;

	recoveryFinishFlag_cllr = TRUE;

	recoveryFlags.FR_CLLR_Recov_Flag = FALSE;

	FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_CLLR_ExtFlt = FALSE;

	recoveryPattern_cllr = Recovery_None;

	cycleModeStopCount_cllr = 0;

//	Lin_Scheduler_SpecialCmd_Start_1();
	
}

void Recovery_Clear_FlagCountReset_clud(void)
{
	eventCountTest_clud = 0;

	recoveryModeFlag_clud = FALSE;

	Check300ms_flag_clud = FALSE;
	Check300ms_flag_1_clud = FALSE;
	Check300ms_flag_2_clud = FALSE;
	Check300ms_flag_3_clud = FALSE;
	Check300ms_flag_4_clud = FALSE;
	Check300ms_flag_5_clud = FALSE;

	gotoTargetPtCount_clud = 0;
	gotoZeroPtCount_clud = 0;

	gotoZeroPtFlag_clud = FALSE;

	steplossCount_clud = 0;
	steplossCount_1_clud = 0;
	steplossCount_2_clud = 0;
	steplossCount_3_clud = 0;
	steplossCount_4_clud = 0;
	steplossCount_5_clud = 0;

//	lastTryFlag = TRUE;

	recoveryFinishFlag_clud = TRUE;

	recoveryFlags.FR_CLUD_Recov_Flag = FALSE;

	FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_CLUD_ExtFlt = FALSE;

	recoveryPattern_clud = Recovery_None;

	cycleModeStopCount_clud = 0;

//	Lin_Scheduler_SpecialCmd_Start_1();
	
}

void Recovery_Clear_FlagCountReset_sllr(void)
{
	eventCountTest_sllr = 0;

	recoveryModeFlag_sllr = FALSE;

	Check300ms_flag_sllr = FALSE;
	Check300ms_flag_1_sllr = FALSE;
	Check300ms_flag_2_sllr = FALSE;
	Check300ms_flag_3_sllr = FALSE;
	Check300ms_flag_4_sllr = FALSE;
	Check300ms_flag_5_sllr = FALSE;

	gotoTargetPtCount_sllr = 0;
	gotoZeroPtCount_sllr = 0;

	gotoZeroPtFlag_sllr = FALSE;

	steplossCount_sllr = 0;
	steplossCount_1_sllr = 0;
	steplossCount_2_sllr = 0;
	steplossCount_3_sllr = 0;
	steplossCount_4_sllr = 0;
	steplossCount_5_sllr = 0;

	recoveryFinishFlag_sllr = TRUE;

	recoveryFlags.FR_SLLR_Recov_Flag = FALSE;

	FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_SLLR_ExtFlt = FALSE;

	recoveryPattern_sllr = Recovery_None;

	cycleModeStopCount_sllr = 0;
}

void Recovery_Clear_FlagCountReset_slud(void)
{
	eventCountTest_slud = 0;

	recoveryModeFlag_slud = FALSE;

	Check300ms_flag_slud = FALSE;
	Check300ms_flag_1_slud = FALSE;
	Check300ms_flag_2_slud = FALSE;
	Check300ms_flag_3_slud = FALSE;
	Check300ms_flag_4_slud = FALSE;
	Check300ms_flag_5_slud = FALSE;

	gotoTargetPtCount_slud = 0;
	gotoZeroPtCount_slud = 0;

	gotoZeroPtFlag_slud = FALSE;

	steplossCount_slud = 0;
	steplossCount_1_slud = 0;
	steplossCount_2_slud = 0;
	steplossCount_3_slud = 0;
	steplossCount_4_slud = 0;
	steplossCount_5_slud = 0;

	recoveryFinishFlag_slud = TRUE;

	recoveryFlags.FR_SLUD_Recov_Flag = FALSE;

	FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_SLUD_ExtFlt = FALSE;

	recoveryPattern_slud = Recovery_None;

	cycleModeStopCount_slud = 0;
}

void Recovery_Clear_FlagCountReset_crlr(void)
{
	eventCountTest_crlr = 0;

	recoveryModeFlag_crlr = FALSE;

	Check300ms_flag_crlr = FALSE;
	Check300ms_flag_1_crlr = FALSE;
	Check300ms_flag_2_crlr = FALSE;
	Check300ms_flag_3_crlr = FALSE;
	Check300ms_flag_4_crlr = FALSE;
	Check300ms_flag_5_crlr = FALSE;

	gotoTargetPtCount_crlr = 0;
	gotoZeroPtCount_crlr = 0;

	gotoZeroPtFlag_crlr = FALSE;

	steplossCount_crlr = 0;
	steplossCount_1_crlr = 0;
	steplossCount_2_crlr = 0;
	steplossCount_3_crlr = 0;
	steplossCount_4_crlr = 0;
	steplossCount_5_crlr = 0;

	recoveryFinishFlag_crlr = TRUE;

	recoveryFlags.FR_CRLR_Recov_Flag = FALSE;

	FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_CRLR_ExtFlt = FALSE;

	recoveryPattern_crlr = Recovery_None;

	cycleModeStopCount_crlr = 0;
}

void Recovery_Clear_FlagCountReset_crud(void)
{
	eventCountTest_crud = 0;

	recoveryModeFlag_crud = FALSE;

	Check300ms_flag_crud = FALSE;
	Check300ms_flag_1_crud = FALSE;
	Check300ms_flag_2_crud = FALSE;
	Check300ms_flag_3_crud = FALSE;
	Check300ms_flag_4_crud = FALSE;
	Check300ms_flag_5_crud = FALSE;

	gotoTargetPtCount_crud = 0;
	gotoZeroPtCount_crud = 0;

	gotoZeroPtFlag_crud = FALSE;

	steplossCount_crud = 0;
	steplossCount_1_crud = 0;
	steplossCount_2_crud = 0;
	steplossCount_3_crud = 0;
	steplossCount_4_crud = 0;
	steplossCount_5_crud = 0;

	recoveryFinishFlag_crud = TRUE;

	recoveryFlags.FR_CRUD_Recov_Flag = FALSE;

	FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_CRUD_ExtFlt = FALSE;

	recoveryPattern_crud = Recovery_None;

	cycleModeStopCount_crud = 0;
}

void Recovery_Clear_FlagCountReset_srlr(void)
{
	eventCountTest_srlr = 0;

	recoveryModeFlag_srlr = FALSE;

	Check300ms_flag_srlr = FALSE;
	Check300ms_flag_1_srlr = FALSE;
	Check300ms_flag_2_srlr = FALSE;
	Check300ms_flag_3_srlr = FALSE;
	Check300ms_flag_4_srlr = FALSE;
	Check300ms_flag_5_srlr = FALSE;

	gotoTargetPtCount_srlr = 0;
	gotoZeroPtCount_srlr = 0;

	gotoZeroPtFlag_srlr = FALSE;

	steplossCount_srlr = 0;
	steplossCount_1_srlr = 0;
	steplossCount_2_srlr = 0;
	steplossCount_3_srlr = 0;
	steplossCount_4_srlr = 0;
	steplossCount_5_srlr = 0;

	recoveryFinishFlag_srlr = TRUE;

	recoveryFlags.FR_SRLR_Recov_Flag = FALSE;

	FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_SRLR_ExtFlt = FALSE;

	recoveryPattern_srlr = Recovery_None;

	cycleModeStopCount_srlr = 0;
}

void Recovery_Clear_FlagCountReset_srud(void)
{
	eventCountTest_srud = 0;

	recoveryModeFlag_srud = FALSE;

	Check300ms_flag_srud = FALSE;
	Check300ms_flag_1_srud = FALSE;
	Check300ms_flag_2_srud = FALSE;
	Check300ms_flag_3_srud = FALSE;
	Check300ms_flag_4_srud = FALSE;
	Check300ms_flag_5_srud = FALSE;

	gotoTargetPtCount_srud = 0;
	gotoZeroPtCount_srud = 0;

	gotoZeroPtFlag_srud = FALSE;

	steplossCount_srud = 0;
	steplossCount_1_srud = 0;
	steplossCount_2_srud = 0;
	steplossCount_3_srud = 0;
	steplossCount_4_srud = 0;
	steplossCount_5_srud = 0;

	recoveryFinishFlag_srud = TRUE;

	recoveryFlags.FR_SRUD_Recov_Flag = FALSE;

	FR_DTC_Flt_flag.FR_EXT_FLT.fltBit.FR_SRUD_ExtFlt = FALSE;

	recoveryPattern_srud = Recovery_None;

	cycleModeStopCount_srud = 0;
}

void Handle_RecoveryMode_Steploss(void)
{
	Steploss_check_cllr(&LIN_LH_FR_STATUS, &LIN_EVNT_PT);
	Steploss_check_clud(&LIN_LH_FR_STATUS, &LIN_EVNT_PT);
	Steploss_check_sllr(&LIN_LH_FR_STATUS, &LIN_EVNT_PT);
	Steploss_check_slud(&LIN_LH_FR_STATUS, &LIN_EVNT_PT);

	Steploss_check_crlr(&LIN_RH_FR_STATUS, &LIN_EVNT_PT);
	Steploss_check_crud(&LIN_RH_FR_STATUS, &LIN_EVNT_PT);
	Steploss_check_srlr(&LIN_RH_FR_STATUS, &LIN_EVNT_PT);
	Steploss_check_srud(&LIN_RH_FR_STATUS, &LIN_EVNT_PT);
}


