#define Amo_LIN_RECOVERY_C_

#include "Cpu.h"
#include "Amo_main.h"
//#include "flexcan_driver.h"
//#include "Amo_STATE_Event.h"
#include "Amo_Timer.h"
//#include "Amo_Calculate.h"
//#include "Amo_CAN_Parsing.h"

uint8_t FR_SLLR_OpDone = 1;
uint8_t FR_SLUD_OpDone = 1;
uint8_t FR_CLLR_OpDone = 1;
uint8_t FR_CLUD_OpDone = 1;

uint8_t FR_CRLR_OpDone = 1;
uint8_t FR_CRUD_OpDone = 1;
uint8_t FR_SRLR_OpDone = 1;
uint8_t FR_SRUD_OpDone = 1;


uint8_t FR_SLLR_OpDone_move_Flag = 0;
uint8_t FR_SLUD_OpDone_move_Flag = 0;
uint8_t FR_CLLR_OpDone_move_Flag = 0;
uint8_t FR_CLUD_OpDone_move_Flag = 0;

uint8_t FR_CRLR_OpDone_move_Flag = 0;
uint8_t FR_CRUD_OpDone_move_Flag = 0;
uint8_t FR_SRLR_OpDone_move_Flag = 0;
uint8_t FR_SRUD_OpDone_move_Flag = 0;



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
	Amo_timer_Stop(timer_68);
}


void Lin_LH_HardStop(t_lin_LH_FR_STATUS *lin_LH_Status, lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_LH_EVNT_MASTER_CMD_11 *lin_LH_Ctrl)							
{
	lin_SpecialCtrl->LDATA.EVNT_CMD = HARD_STOP_ACT;
	lin_SpecialCtrl->LDATA.EVNT_Broad = UNI_CAST;

	if(lin_LH_Status->fr_sllr_status.LDATA.FR_SLLR_OpDone != 1)
	{
		lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_SLLR;

		l_u8_wr_LI0_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
		l_u8_wr_LI0_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
		l_bool_wr_LI0_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

		lin_LH_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = l_u16_rd_LI0_FR_SLLR_ActuatorState();
		l_u16_wr_LI0_EVNT_Front_Side_LH_LeftRight_TargetPosition(lin_LH_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition);
	}
	
	if(lin_LH_Status->fr_slud_status.LDATA.FR_SLUD_OpDone != 1)
	{
		lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_SLUD;

		l_u8_wr_LI0_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
		l_u8_wr_LI0_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
		l_bool_wr_LI0_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

		lin_LH_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = l_u16_rd_LI0_FR_SLUD_ActuatorState();
		l_u16_wr_LI0_EVNT_Front_Side_LH_UpDown_TargetPosition(lin_LH_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition);
	}

/*
	if(lin_LH_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone != 1)
	{
		lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_CLLR;

		l_u8_wr_LI0_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
		l_u8_wr_LI0_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
		l_bool_wr_LI0_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

		lin_LH_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = l_u16_rd_LI0_FR_CLLR_ActuatorState();
		l_u16_wr_LI0_EVNT_Front_Center_LH_LeftRight_TargetPosition(lin_LH_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition);
	}

	if(lin_LH_Status->fr_clud_status.LDATA.FR_CLUD_OpDone != 1)
	{
		lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_CLUD;

		l_u8_wr_LI0_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
		l_u8_wr_LI0_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
		l_bool_wr_LI0_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

		lin_LH_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = l_u16_rd_LI0_FR_CLUD_ActuatorState();
		l_u16_wr_LI0_EVNT_Front_Center_LH_UpDown_TargetPosition(lin_LH_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition);
	}
*/
	Amo_timer_Stop(timer_62);
	Amo_timer_Start(timer_62, 20, true, Lin_LH_Scheduler_Normal_Start);
}


void Lin_RH_HardStop(t_lin_RH_FR_STATUS *lin_RH_Status, lin_RH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_RH_EVNT_MASTER_CMD_21 *lin_RH_Ctrl)							
{
	lin_SpecialCtrl->LDATA.EVNT_CMD = HARD_STOP_ACT;
	lin_SpecialCtrl->LDATA.EVNT_Broad = UNI_CAST;

	if(lin_RH_Status->fr_crlr_status.LDATA.FR_CRLR_OpDone != 1)
	{
		lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_CRLR;

		l_u8_wr_LI1_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
		l_u8_wr_LI1_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
		l_bool_wr_LI1_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

		lin_RH_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = l_u16_rd_LI1_FR_CRLR_ActuatorState();
		l_u16_wr_LI1_EVNT_Front_Center_RH_LeftRight_TargetPosition(lin_RH_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition);
	}
	
	if(lin_RH_Status->fr_crud_status.LDATA.FR_CRUD_OpDone != 1)
	{
		lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_CRUD;

		l_u8_wr_LI1_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
		l_u8_wr_LI1_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
		l_bool_wr_LI1_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

		lin_RH_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = l_u16_rd_LI1_FR_CRUD_ActuatorState();
		l_u16_wr_LI1_EVNT_Front_Center_RH_UpDown_TargetPosition(lin_RH_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition);
	}

/*
	if(lin_RH_Status->fr_srlr_status.LDATA.FR_SRLR_OpDone != 1)
	{
		lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_SRLR;

		l_u8_wr_LI1_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
		l_u8_wr_LI1_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
		l_bool_wr_LI1_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

		lin_RH_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = l_u16_rd_LI1_FR_SRLR_ActuatorState();
		l_u16_wr_LI1_EVNT_Front_Side_RH_LeftRight_TargetPosition(lin_RH_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition);
	}

	if(lin_RH_Status->fr_srud_status.LDATA.FR_SRUD_OpDone != 1)
	{
		lin_SpecialCtrl->LDATA.EVNT_ADDr = Evnt_Addr_FR_SRUD;

		l_u8_wr_LI1_EVNT_ADDr(lin_SpecialCtrl->LDATA.EVNT_ADDr);
		l_u8_wr_LI1_EVNT_CMD(lin_SpecialCtrl->LDATA.EVNT_CMD);
		l_bool_wr_LI1_EVNT_Broad(lin_SpecialCtrl->LDATA.EVNT_Broad);

		lin_RH_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = l_u16_rd_LI1_FR_SRUD_ActuatorState();
		l_u16_wr_LI1_EVNT_Front_Side_RH_UpDown_TargetPosition(lin_RH_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition);
	}
*/
	
	Amo_timer_Stop(timer_68);
	Amo_timer_Start(timer_68, 20, true, Lin_RH_Scheduler_Normal_Start);
}


void OpDone_LH_Move_Check(void)
{
	FR_SLLR_OpDone = l_bool_rd_LI0_FR_SLLR_OpDone();
	FR_SLUD_OpDone = l_bool_rd_LI0_FR_SLUD_OpDone();
	FR_CLLR_OpDone = l_bool_rd_LI0_FR_CLLR_OpDone();
	FR_CLUD_OpDone = l_bool_rd_LI0_FR_CLUD_OpDone();

	if(FR_SLLR_OpDone == 0) { FR_SLLR_OpDone_move_Flag = 1; }
	if(FR_SLUD_OpDone == 0) { FR_SLUD_OpDone_move_Flag = 1; }
	if(FR_CLLR_OpDone == 0) { FR_CLLR_OpDone_move_Flag = 1; }
	if(FR_CLUD_OpDone == 0) { FR_CLUD_OpDone_move_Flag = 1; }

	Amo_timer_Stop(timer_64);
}

void OpDone_RH_Move_Check(void)
{
	FR_CRLR_OpDone = l_bool_rd_LI1_FR_CRLR_OpDone();
	FR_CRUD_OpDone = l_bool_rd_LI1_FR_CRUD_OpDone();
	FR_SRLR_OpDone = l_bool_rd_LI1_FR_SRLR_OpDone();
	FR_SRUD_OpDone = l_bool_rd_LI1_FR_SRUD_OpDone();

	if(FR_CRLR_OpDone == 0) { FR_CRLR_OpDone_move_Flag = 1; }
	if(FR_CRUD_OpDone == 0) { FR_CRUD_OpDone_move_Flag = 1; }
	if(FR_SRLR_OpDone == 0) { FR_SRLR_OpDone_move_Flag = 1; }
	if(FR_SRUD_OpDone == 0) { FR_SRUD_OpDone_move_Flag = 1; }

	Amo_timer_Stop(timer_65);
}


void Evnt_LH_Special_Cmd(void)
{
	Lin_LH_HardStop(&LIN_LH_FR_STATUS, &LIN_LH_EVNT_SPECIAL_CMD, &LIN_LH_EVNT_MASTER_CMD);

	Amo_timer_Stop(timer_61);
}

void Evnt_RH_Special_Cmd(void)
{
	Lin_RH_HardStop(&LIN_RH_FR_STATUS, &LIN_RH_EVNT_SPECIAL_CMD, &LIN_RH_EVNT_MASTER_CMD);

	Amo_timer_Stop(timer_67);
}


void Lin_Scheduler_SpecialCmd_Start(void)
{
	if(FR_SLLR_OpDone_move_Flag == 1 || FR_SLUD_OpDone_move_Flag == 1 || FR_CLLR_OpDone_move_Flag == 1 || FR_CLUD_OpDone_move_Flag == 1)
	{
		if(FR_SLLR_OpDone_move_Flag == 1) { FR_SLLR_OpDone_move_Flag = 0; }
		if(FR_SLUD_OpDone_move_Flag == 1) { FR_SLUD_OpDone_move_Flag = 0; }
		if(FR_CLLR_OpDone_move_Flag == 1) { FR_CLLR_OpDone_move_Flag = 0; }
		if(FR_CLUD_OpDone_move_Flag == 1) { FR_CLUD_OpDone_move_Flag = 0; }
		
		l_sch_set(LI0, LI0_SCHEDULER_EVNT_SPECIAL_CMD_FR_DRV, 0u);

		Amo_timer_Stop(timer_61);
		Amo_timer_Start(timer_61, 20, true, Evnt_LH_Special_Cmd);
	}

	else if(FR_CRLR_OpDone_move_Flag == 1 || FR_CRUD_OpDone_move_Flag == 1 || FR_SRLR_OpDone_move_Flag == 1 || FR_SRUD_OpDone_move_Flag == 1)
	{
		if(FR_CRLR_OpDone_move_Flag == 1) { FR_CRLR_OpDone_move_Flag = 0; }
		if(FR_CRUD_OpDone_move_Flag == 1) { FR_CRUD_OpDone_move_Flag = 0; }
		if(FR_SRLR_OpDone_move_Flag == 1) { FR_SRLR_OpDone_move_Flag = 0; }
		if(FR_SRUD_OpDone_move_Flag == 1) { FR_SRUD_OpDone_move_Flag = 0; }
		
		l_sch_set(LI1, LI1_SCHEDULER_EVNT_SPECIAL_CMD_FR_PASS, 0u);

		Amo_timer_Stop(timer_67);
		Amo_timer_Start(timer_67, 20, true, Evnt_RH_Special_Cmd);
	}

}



void lin_Read_FR_LH_OpDone(t_lin_LH_FR_STATUS *lin_LH_Status)
{
	lin_LH_Status->fr_sllr_status.LDATA.FR_SLLR_OpDone = l_bool_rd_LI0_FR_SLLR_OpDone();
	lin_LH_Status->fr_slud_status.LDATA.FR_SLUD_OpDone = l_bool_rd_LI0_FR_SLUD_OpDone();
	lin_LH_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone = l_bool_rd_LI0_FR_CLLR_OpDone();
	lin_LH_Status->fr_clud_status.LDATA.FR_CLUD_OpDone = l_bool_rd_LI0_FR_CLUD_OpDone();

	if(FR_SLLR_OpDone_move_Flag == 1)
	{
		if(lin_LH_Status->fr_sllr_status.LDATA.FR_SLLR_OpDone != 1)
		{
			Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
		}
//		FR_SLLR_OpDone_move_Flag = 0;
	}

	if(FR_SLUD_OpDone_move_Flag == 1)
	{
		if(lin_LH_Status->fr_slud_status.LDATA.FR_SLUD_OpDone != 1)
		{
			Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
		}
//		FR_SLUD_OpDone_move_Flag = 0;
	}

	if(FR_CLLR_OpDone_move_Flag == 1)
	{
		if(lin_LH_Status->fr_cllr_status.LDATA.FR_CLLR_OpDone != 1)
		{
//			Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
		}
//		FR_CLLR_OpDone_move_Flag = 0;
	}

	if(FR_CLUD_OpDone_move_Flag == 1)
	{
		if(lin_LH_Status->fr_clud_status.LDATA.FR_CLUD_OpDone != 1)
		{
//			Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
		}
//		FR_CLUD_OpDone_move_Flag = 0;
	}
}


void lin_Read_FR_RH_OpDone(t_lin_RH_FR_STATUS *lin_RH_Status)
{
	lin_RH_Status->fr_crlr_status.LDATA.FR_CRLR_OpDone = l_bool_rd_LI1_FR_CRLR_OpDone();
	lin_RH_Status->fr_crud_status.LDATA.FR_CRUD_OpDone = l_bool_rd_LI1_FR_CRUD_OpDone();
	lin_RH_Status->fr_srlr_status.LDATA.FR_SRLR_OpDone = l_bool_rd_LI1_FR_SRLR_OpDone();
	lin_RH_Status->fr_srud_status.LDATA.FR_SRUD_OpDone = l_bool_rd_LI1_FR_SRUD_OpDone();

	if(FR_CRLR_OpDone_move_Flag == 1)
	{
		if(lin_RH_Status->fr_crlr_status.LDATA.FR_CRLR_OpDone != 1)
		{
			Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
		}
//		FR_CRLR_OpDone_move_Flag = 0;
	}

	if(FR_CRUD_OpDone_move_Flag == 1)
	{
		if(lin_RH_Status->fr_crud_status.LDATA.FR_CRUD_OpDone != 1)
		{
			Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
		}
//		FR_CRUD_OpDone_move_Flag = 0;
	}

	if(FR_SRLR_OpDone_move_Flag == 1)
	{
		if(lin_RH_Status->fr_srlr_status.LDATA.FR_SRLR_OpDone != 1)
		{
//			Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
		}
//		FR_SRLR_OpDone_move_Flag = 0;
	}

	if(FR_SRUD_OpDone_move_Flag == 1)
	{
		if(lin_RH_Status->fr_srud_status.LDATA.FR_SRUD_OpDone != 1)
		{
//			Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
		}
//		FR_SRUD_OpDone_move_Flag = 0;
	}

}


void lin_Read_RR_OpDone(t_lin_RC_RR_STATUS *lin_RR_Status)
{
	lin_RR_Status->rr_cllr_status.LDATA.RR_CLLR_OpDone = l_bool_rd_LI2_RR_CLLR_OpDone();
	lin_RR_Status->rr_clud_status.LDATA.RR_CLUD_OpDone = l_bool_rd_LI2_RR_CLUD_OpDone();
	lin_RR_Status->rr_crlr_status.LDATA.RR_CRLR_OpDone = l_bool_rd_LI2_RR_CRLR_OpDone();
	lin_RR_Status->rr_crud_status.LDATA.RR_CRUD_OpDone = l_bool_rd_LI2_RR_CRUD_OpDone();

	if((lin_RR_Status->rr_cllr_status.LDATA.RR_CLLR_OpDone != 1) || (lin_RR_Status->rr_clud_status.LDATA.RR_CLUD_OpDone != 1) || \
		(lin_RR_Status->rr_crlr_status.LDATA.RR_CRLR_OpDone != 1) || (lin_RR_Status->rr_crud_status.LDATA.RR_CRUD_OpDone != 1))
	{
//		Lin_Scheduler_SpecialCmd_Start();
		Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);
	}
}

void Opdone_TimeOut_LH_Check(void)
{
	if(l_flg_tst_LI0_FR_SLLR_OpDone_flag() || l_flg_tst_LI0_FR_SLUD_OpDone_flag() || l_flg_tst_LI0_FR_CLLR_OpDone_flag() || l_flg_tst_LI0_FR_CLUD_OpDone_flag())
	{
		l_flg_clr_LI0_FR_SLLR_OpDone_flag();
		l_flg_clr_LI0_FR_SLUD_OpDone_flag();
		l_flg_clr_LI0_FR_CLLR_OpDone_flag();
		l_flg_clr_LI0_FR_CLUD_OpDone_flag();

		lin_Read_FR_LH_OpDone(&LIN_LH_FR_STATUS);
	}
		
/*		
	if(l_flg_tst_LI2_RR_CLLR_OpDone_flag() || l_flg_tst_LI2_RR_CLUD_OpDone_flag() || l_flg_tst_LI2_RR_CRLR_OpDone_flag() || l_flg_tst_LI2_RR_CRUD_OpDone_flag())
	{
		l_flg_clr_LI2_RR_CLLR_OpDone_flag();
		l_flg_clr_LI2_RR_CLUD_OpDone_flag();
		l_flg_clr_LI2_RR_CRLR_OpDone_flag();
		l_flg_clr_LI2_RR_CRUD_OpDone_flag();
		
		lin_Read_RR_OpDone(&LIN_RC_RR_STATUS);
	}
*/
		Amo_timer_Stop(timer_60);

}

void Opdone_TimeOut_RH_Check(void)
{
	if(l_flg_tst_LI1_FR_SRLR_OpDone_flag() || l_flg_tst_LI1_FR_SRUD_OpDone_flag() || l_flg_tst_LI1_FR_CRLR_OpDone_flag() || l_flg_tst_LI1_FR_CRUD_OpDone_flag())
	{

		l_flg_clr_LI1_FR_SRLR_OpDone_flag();
		l_flg_clr_LI1_FR_SRUD_OpDone_flag();
		l_flg_clr_LI1_FR_CRLR_OpDone_flag();
		l_flg_clr_LI1_FR_CRUD_OpDone_flag();
		
		lin_Read_FR_RH_OpDone(&LIN_RH_FR_STATUS);
	}		
		Amo_timer_Stop(timer_66);

}



