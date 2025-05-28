#ifndef __AMO_CYCLE_C
#define __AMO_CYCLE_C

#include "Cpu.h"
#include "Amo_main.h"
#include "flexcan_driver.h"
#include "Amo_STATE_Event.h"
#include "Amo_Timer.h"
#include "Amo_Calculate.h"
#include "Amo_CAN_Parsing.h"
#include "Amo_Cycle.h"
#include "Amo_Lin.h"


extern Lvnt_1_data Can_Tx_Evnt_1;
extern Lvnt_2_data Can_Tx_Evnt_2;
extern Lvnt_3_data Can_Tx_Evnt_3;
extern Lvnt_4_data Can_Tx_Evnt_4;
extern Lvnt_5_data Can_Tx_Evnt_5;
extern Lvnt_6_data Can_Tx_Evnt_6;
extern Lvnt_7_data Can_Tx_Evnt_7;
extern Lvnt_8_data Can_Tx_Evnt_8;
extern Lvnt_9_data Can_Tx_Evnt_9;

typedef enum _lin_angle{
	FullClose_max = 792,  //79.2
	FullClose_min = 1256, //-79.2
	FullClose_UD = 1023,  //102.3
	Half_max_LR = 150,  //15
	Half_min_LR = 1898, //-15
	Max_LR = 290,  //30
	Min_LR = 1758, //-30
 	Min_UD = 240,  //24
	Max_UD = 1808, //-24
}Lin_angle;


int SLLR_OP_check;
int SLUD_OP_check;
int CLLR_OP_check;
int CLUD_OP_check;
int SRLR_OP_check;
int SRUD_OP_check;
int CRLR_OP_check;
int CRUD_OP_check;







void lin_FrDrPsCycleMode_task(void)
{
	lin_FrDrCycleMode_TimerStop();
	lin_FrPsCycleMode_TimerStop();

	lin_FrDrPsSwingData_Scenario0();

	Set_LinToCan_Disp_FrDr_Mode(&Can_Tx_Evnt_4);
	Set_LinToCan_Disp_FrPs_Mode(&Can_Tx_Evnt_4);
	Amo_timer_Stop(timer_38);
	Amo_timer_Start(timer_38, 10, true, Set_LinToCan_Disp_FrDrCycle_XY);
	Amo_timer_Stop(timer_39);
	Amo_timer_Start(timer_39, 10, true, Set_LinToCan_Disp_FrPsCycle_XY);
}

void lin_FrDrPsSwingData_Scenario0(void)
{
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 0;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 0;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = 0;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 0;

	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = 0;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = 0;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = 0;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = 0;

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD);

	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD);

	Amo_timer_Stop(timer_70);
	Amo_timer_Start(timer_70, 1000, false, OpDone_Check31);
}

void OpDone_Check31(void)
{

	SLLR_OP_check = l_bool_rd_LI0_FR_SLLR_OpDone();
	SLUD_OP_check = l_bool_rd_LI0_FR_SLUD_OpDone();
	CLLR_OP_check = l_bool_rd_LI0_FR_CLLR_OpDone();
	CLUD_OP_check = l_bool_rd_LI0_FR_CLUD_OpDone();

	SRLR_OP_check = l_bool_rd_LI1_FR_SRLR_OpDone();
	SRUD_OP_check = l_bool_rd_LI1_FR_SRUD_OpDone();
	CRLR_OP_check = l_bool_rd_LI1_FR_CRLR_OpDone();
	CRUD_OP_check = l_bool_rd_LI1_FR_CRUD_OpDone();

	Amo_timer_Stop(timer_71);
	Amo_timer_Start(timer_71, 50, false, OpDone_Check31);

	if(SLLR_OP_check == 1 && SLUD_OP_check == 1 && CLLR_OP_check == 1 && CLUD_OP_check == 1 && \
		SRLR_OP_check == 1 && SRUD_OP_check == 1 && CRLR_OP_check == 1 && CRUD_OP_check == 1)
	{
		lin_FrDrPsSwingData_Scenario1();
	}

}

void lin_FrDrPsSwingData_Scenario1(void)
{
	lin_FrDrCycleMode_TimerStop();
	lin_FrPsCycleMode_TimerStop();

	if(Hu_Fr_Dr_Vent_Mode != SWING_MODE)
	{
		lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
	}
	if(Hu_Fr_Ps_Vent_Mode != SWING_MODE)
	{
		lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
	}
	
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_3);
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_3);

	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Min_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 0;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Max_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 0;

	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = 0;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = 0;

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD);

	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD);

	Amo_timer_Stop(timer_72);
	Amo_timer_Start(timer_72, 1000, false, OpDone_Check32);
}


void OpDone_Check32(void)
{

	SLLR_OP_check = l_bool_rd_LI0_FR_SLLR_OpDone();
	SLUD_OP_check = l_bool_rd_LI0_FR_SLUD_OpDone();
	CLLR_OP_check = l_bool_rd_LI0_FR_CLLR_OpDone();
	CLUD_OP_check = l_bool_rd_LI0_FR_CLUD_OpDone();

	SRLR_OP_check = l_bool_rd_LI1_FR_SRLR_OpDone();
	SRUD_OP_check = l_bool_rd_LI1_FR_SRUD_OpDone();
	CRLR_OP_check = l_bool_rd_LI1_FR_CRLR_OpDone();
	CRUD_OP_check = l_bool_rd_LI1_FR_CRUD_OpDone();

	Amo_timer_Stop(timer_73);
	Amo_timer_Start(timer_73, 50, false, OpDone_Check32);

	if(SLLR_OP_check == 1 && SLUD_OP_check == 1 && CLLR_OP_check == 1 && CLUD_OP_check == 1 && \
		SRLR_OP_check == 1 && SRUD_OP_check == 1 && CRLR_OP_check == 1 && CRUD_OP_check == 1)
	{
		lin_FrDrPsSwingData_Scenario2();
	}

}


void lin_FrDrPsSwingData_Scenario2(void)
{
	lin_FrDrCycleMode_TimerStop();
	lin_FrPsCycleMode_TimerStop();
	
	if(Hu_Fr_Dr_Vent_Mode != SWING_MODE)
	{
		lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
	}
	if(Hu_Fr_Ps_Vent_Mode != SWING_MODE)
	{
		lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
	}

	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Min_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Min_UD;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Max_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Max_UD;

	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Max_UD;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Min_UD;

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD);

	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD);

	Amo_timer_Stop(timer_74);
	Amo_timer_Start(timer_74, 1000, false, OpDone_Check33);
}

void OpDone_Check33(void)
{

	SLLR_OP_check = l_bool_rd_LI0_FR_SLLR_OpDone();
	SLUD_OP_check = l_bool_rd_LI0_FR_SLUD_OpDone();
	CLLR_OP_check = l_bool_rd_LI0_FR_CLLR_OpDone();
	CLUD_OP_check = l_bool_rd_LI0_FR_CLUD_OpDone();

	SRLR_OP_check = l_bool_rd_LI1_FR_SRLR_OpDone();
	SRUD_OP_check = l_bool_rd_LI1_FR_SRUD_OpDone();
	CRLR_OP_check = l_bool_rd_LI1_FR_CRLR_OpDone();
	CRUD_OP_check = l_bool_rd_LI1_FR_CRUD_OpDone();

	Amo_timer_Stop(timer_75);
	Amo_timer_Start(timer_75, 50, false, OpDone_Check33);

	if(SLLR_OP_check == 1 && SLUD_OP_check == 1 && CLLR_OP_check == 1 && CLUD_OP_check == 1 && \
		SRLR_OP_check == 1 && SRUD_OP_check == 1 && CRLR_OP_check == 1 && CRUD_OP_check == 1)
	{
		lin_FrDrPsSwingData_Scenario3();
	}

}


void lin_FrDrPsSwingData_Scenario3(void)
{
	lin_FrDrCycleMode_TimerStop();
	lin_FrPsCycleMode_TimerStop();

	if(Hu_Fr_Dr_Vent_Mode != SWING_MODE)
	{
		lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
	}
	if(Hu_Fr_Ps_Vent_Mode != SWING_MODE)
	{
		lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
	}

	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Max_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Min_UD;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Min_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Max_UD;

	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Max_UD;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Min_UD;

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD);

	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD);

	Amo_timer_Stop(timer_76);
	Amo_timer_Start(timer_76, 1000, false, OpDone_Check34);
}

void OpDone_Check34(void)
{

	SLLR_OP_check = l_bool_rd_LI0_FR_SLLR_OpDone();
	SLUD_OP_check = l_bool_rd_LI0_FR_SLUD_OpDone();
	CLLR_OP_check = l_bool_rd_LI0_FR_CLLR_OpDone();
	CLUD_OP_check = l_bool_rd_LI0_FR_CLUD_OpDone();

	SRLR_OP_check = l_bool_rd_LI1_FR_SRLR_OpDone();
	SRUD_OP_check = l_bool_rd_LI1_FR_SRUD_OpDone();
	CRLR_OP_check = l_bool_rd_LI1_FR_CRLR_OpDone();
	CRUD_OP_check = l_bool_rd_LI1_FR_CRUD_OpDone();

	Amo_timer_Stop(timer_77);
	Amo_timer_Start(timer_77, 50, false, OpDone_Check34);

	if(SLLR_OP_check == 1 && SLUD_OP_check == 1 && CLLR_OP_check == 1 && CLUD_OP_check == 1 && \
		SRLR_OP_check == 1 && SRUD_OP_check == 1 && CRLR_OP_check == 1 && CRUD_OP_check == 1)
	{
		lin_FrDrPsSwingData_Scenario4();
	}

}


void lin_FrDrPsSwingData_Scenario4(void)
{
	lin_FrDrCycleMode_TimerStop();
	lin_FrPsCycleMode_TimerStop();

	if(Hu_Fr_Dr_Vent_Mode != SWING_MODE)
	{
		lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
	}
	if(Hu_Fr_Ps_Vent_Mode != SWING_MODE)
	{
		lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
	}

	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Max_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Max_UD;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Min_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Min_UD;

	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Min_UD;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Max_UD;

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD);

	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD);

	Amo_timer_Stop(timer_78);
	Amo_timer_Start(timer_78, 1000, false, OpDone_Check35);
}

void OpDone_Check35(void)
{

	SLLR_OP_check = l_bool_rd_LI0_FR_SLLR_OpDone();
	SLUD_OP_check = l_bool_rd_LI0_FR_SLUD_OpDone();
	CLLR_OP_check = l_bool_rd_LI0_FR_CLLR_OpDone();
	CLUD_OP_check = l_bool_rd_LI0_FR_CLUD_OpDone();

	SRLR_OP_check = l_bool_rd_LI1_FR_SRLR_OpDone();
	SRUD_OP_check = l_bool_rd_LI1_FR_SRUD_OpDone();
	CRLR_OP_check = l_bool_rd_LI1_FR_CRLR_OpDone();
	CRUD_OP_check = l_bool_rd_LI1_FR_CRUD_OpDone();

	Amo_timer_Stop(timer_79);
	Amo_timer_Start(timer_79, 50, false, OpDone_Check35);

	if(SLLR_OP_check == 1 && SLUD_OP_check == 1 && CLLR_OP_check == 1 && CLUD_OP_check == 1 && \
		SRLR_OP_check == 1 && SRUD_OP_check == 1 && CRLR_OP_check == 1 && CRUD_OP_check == 1)
	{
		lin_FrDrPsSwingData_Scenario5();
	}

}

void lin_FrDrPsSwingData_Scenario5(void)
{
	lin_FrDrCycleMode_TimerStop();
	lin_FrPsCycleMode_TimerStop();

	if(Hu_Fr_Dr_Vent_Mode != SWING_MODE)
	{
		lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
	}
	if(Hu_Fr_Ps_Vent_Mode != SWING_MODE)
	{
		lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
	}

	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Min_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Max_UD;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Max_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Min_UD;

	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Min_UD;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Max_UD;

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD);

	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD);

	Amo_timer_Stop(timer_80);
	Amo_timer_Start(timer_80, 1000, false, OpDone_Check36);
}

void OpDone_Check36(void)
{

	SLLR_OP_check = l_bool_rd_LI0_FR_SLLR_OpDone();
	SLUD_OP_check = l_bool_rd_LI0_FR_SLUD_OpDone();
	CLLR_OP_check = l_bool_rd_LI0_FR_CLLR_OpDone();
	CLUD_OP_check = l_bool_rd_LI0_FR_CLUD_OpDone();

	SRLR_OP_check = l_bool_rd_LI1_FR_SRLR_OpDone();
	SRUD_OP_check = l_bool_rd_LI1_FR_SRUD_OpDone();
	CRLR_OP_check = l_bool_rd_LI1_FR_CRLR_OpDone();
	CRUD_OP_check = l_bool_rd_LI1_FR_CRUD_OpDone();

	Amo_timer_Stop(timer_81);
	Amo_timer_Start(timer_81, 50, false, OpDone_Check36);

	if(SLLR_OP_check == 1 && SLUD_OP_check == 1 && CLLR_OP_check == 1 && CLUD_OP_check == 1 && \
		SRLR_OP_check == 1 && SRUD_OP_check == 1 && CRLR_OP_check == 1 && CRUD_OP_check == 1)
	{
		lin_FrDrPsSwingData_Scenario2();
	}

}


///////////////////////////////////////////////////////////////////////////////////

void lin_FrDrCycleMode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	lin_FrDrCycleMode_TimerStop();
	lin_FrPsCycleMode_TimerStop();
	lin_RrCycleMode_TimerStop();

	lin_FrDrSwingData_Scenario0();
	Set_LinToCan_Disp_FrDr_Mode(&Can_Tx_Evnt_4);
	Amo_timer_Stop(timer_35);
	Amo_timer_Start(timer_35, 10, true, Set_LinToCan_Disp_FrDrCycle_XY);
}


void lin_FrDrSwingData_Scenario0()
{
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 0;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 0;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = 0;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 0;

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD);


	Amo_timer_Stop(timer_16);
	Amo_timer_Start(timer_16, 1000, false, OpDone_Check1);


//	lin_FrDrCycleMode_TimerStop();
//	Amo_timer_Start(timer_16, 5000, false, lin_FrDrSwingData_Scenario1);  //1199
}


void OpDone_Check1(void)
{

	SLLR_OP_check = l_bool_rd_LI0_FR_SLLR_OpDone();
	SLUD_OP_check = l_bool_rd_LI0_FR_SLUD_OpDone();
	
	CLLR_OP_check = l_bool_rd_LI0_FR_CLLR_OpDone();
	CLUD_OP_check = l_bool_rd_LI0_FR_CLUD_OpDone();

	Amo_timer_Stop(timer_41);
	Amo_timer_Start(timer_41, 50, false, OpDone_Check1);

	if(SLLR_OP_check == 1 && SLUD_OP_check == 1 && CLLR_OP_check == 1 && CLUD_OP_check == 1)
	{
		lin_FrDrSwingData_Scenario1();
	}

}


void lin_FrDrSwingData_Scenario1()
{
	lin_FrDrCycleMode_TimerStop();
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_3);

	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Min_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 0;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Max_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 0;

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD);

	Amo_timer_Stop(timer_17);
	Amo_timer_Start(timer_17, 1000, false, OpDone_Check2);



//	lin_FrDrCycleMode_TimerStop();
//	Amo_timer_Start(timer_17, 5000, false, lin_FrDrSwingData_Scenario2);

}

void OpDone_Check2(void)
{
	SLLR_OP_check = l_bool_rd_LI0_FR_SLLR_OpDone();
	SLUD_OP_check = l_bool_rd_LI0_FR_SLUD_OpDone();
	
	CLLR_OP_check = l_bool_rd_LI0_FR_CLLR_OpDone();
	CLUD_OP_check = l_bool_rd_LI0_FR_CLUD_OpDone();

	Amo_timer_Stop(timer_42);
	Amo_timer_Start(timer_42, 50, false, OpDone_Check2);

	if(SLLR_OP_check == 1 && SLUD_OP_check == 1 && CLLR_OP_check == 1 && CLUD_OP_check == 1)
	{
		lin_FrDrSwingData_Scenario2();
	}

}


void lin_FrDrSwingData_Scenario2()
{
	lin_FrDrCycleMode_TimerStop();
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Min_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Min_UD; //250; //260 26degree;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Max_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Max_UD; //250; // 260;

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD);

	Amo_timer_Stop(timer_18);
	Amo_timer_Start(timer_18, 1000, false, OpDone_Check3);


//	lin_FrDrCycleMode_TimerStop();
//	Amo_timer_Start(timer_18, 5000, false, lin_FrDrSwingData_Scenario3);
}

void OpDone_Check3(void)
{
	SLLR_OP_check = l_bool_rd_LI0_FR_SLLR_OpDone();
	SLUD_OP_check = l_bool_rd_LI0_FR_SLUD_OpDone();
	
	CLLR_OP_check = l_bool_rd_LI0_FR_CLLR_OpDone();
	CLUD_OP_check = l_bool_rd_LI0_FR_CLUD_OpDone();

	Amo_timer_Stop(timer_43);
	Amo_timer_Start(timer_43, 50, false, OpDone_Check3);

	if(SLLR_OP_check == 1 && SLUD_OP_check == 1 && CLLR_OP_check == 1 && CLUD_OP_check == 1)
	{
		lin_FrDrSwingData_Scenario3();
	}

}

void lin_FrDrSwingData_Scenario3()
{
	lin_FrDrCycleMode_TimerStop();
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Max_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Min_UD; //250; //260;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Min_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Max_UD; //250; //260;

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD);

	Amo_timer_Stop(timer_19);
	Amo_timer_Start(timer_19, 1000, false, OpDone_Check4);


//	lin_FrDrCycleMode_TimerStop();
//	Amo_timer_Start(timer_19, 5000, false, lin_FrDrSwingData_Scenario4);
}

void OpDone_Check4(void)
{
	SLLR_OP_check = l_bool_rd_LI0_FR_SLLR_OpDone();
	SLUD_OP_check = l_bool_rd_LI0_FR_SLUD_OpDone();
	
	CLLR_OP_check = l_bool_rd_LI0_FR_CLLR_OpDone();
	CLUD_OP_check = l_bool_rd_LI0_FR_CLUD_OpDone();

	Amo_timer_Stop(timer_44);
	Amo_timer_Start(timer_44, 50, false, OpDone_Check4);

	if(SLLR_OP_check == 1 && SLUD_OP_check == 1 && CLLR_OP_check == 1 && CLUD_OP_check == 1)
	{
		lin_FrDrSwingData_Scenario4();
	}

}

void lin_FrDrSwingData_Scenario4()
{
	lin_FrDrCycleMode_TimerStop();
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Max_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Min_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD);

	Amo_timer_Stop(timer_20);
	Amo_timer_Start(timer_20, 1000, false, OpDone_Check5);


//	lin_FrDrCycleMode_TimerStop();
//	Amo_timer_Start(timer_20, 5000, false, lin_FrDrSwingData_Scenario5);
}

void OpDone_Check5(void)
{
	SLLR_OP_check = l_bool_rd_LI0_FR_SLLR_OpDone();
	SLUD_OP_check = l_bool_rd_LI0_FR_SLUD_OpDone();	

	CLLR_OP_check = l_bool_rd_LI0_FR_CLLR_OpDone();
	CLUD_OP_check = l_bool_rd_LI0_FR_CLUD_OpDone();

	Amo_timer_Stop(timer_45);
	Amo_timer_Start(timer_45, 50, false, OpDone_Check5);

	if(SLLR_OP_check == 1 && SLUD_OP_check == 1 && CLLR_OP_check == 1 && CLUD_OP_check == 1)
	{
		lin_FrDrSwingData_Scenario5();
	}
}

void lin_FrDrSwingData_Scenario5()
{
	lin_FrDrCycleMode_TimerStop();
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Min_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Max_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD);

	Amo_timer_Stop(timer_21);
	Amo_timer_Start(timer_21, 1000, false, OpDone_Check6);

//	lin_FrDrCycleMode_TimerStop();
//	Amo_timer_Start(timer_21, 5000, false, lin_FrDrSwingData_Scenario2);
}

void OpDone_Check6(void)
{
	SLLR_OP_check = l_bool_rd_LI0_FR_SLLR_OpDone();
	SLUD_OP_check = l_bool_rd_LI0_FR_SLUD_OpDone();

	CLLR_OP_check = l_bool_rd_LI0_FR_CLLR_OpDone();
	CLUD_OP_check = l_bool_rd_LI0_FR_CLUD_OpDone();

	Amo_timer_Stop(timer_46);
	Amo_timer_Start(timer_46, 50, false, OpDone_Check6);

	if(SLLR_OP_check == 1 && SLUD_OP_check == 1 && CLLR_OP_check == 1 && CLUD_OP_check == 1)
	{
		lin_FrDrSwingData_Scenario2();
	}
}

void lin_FrDrCycleMode_TimerStop(void)
{
	Amo_timer_Stop(timer_16);
	Amo_timer_Stop(timer_17);
	Amo_timer_Stop(timer_18);
	Amo_timer_Stop(timer_19);
	Amo_timer_Stop(timer_20);
	Amo_timer_Stop(timer_21);
	Amo_timer_Stop(timer_35);
	Amo_timer_Stop(timer_38);
	Amo_timer_Stop(timer_39);
	Amo_timer_Stop(timer_41);
	Amo_timer_Stop(timer_42);
	Amo_timer_Stop(timer_43);
	Amo_timer_Stop(timer_44);
	Amo_timer_Stop(timer_45);
	Amo_timer_Stop(timer_46);
	Amo_timer_Stop(timer_70);
	Amo_timer_Stop(timer_71);
	Amo_timer_Stop(timer_72);
	Amo_timer_Stop(timer_73);
	Amo_timer_Stop(timer_74);
	Amo_timer_Stop(timer_75);
	Amo_timer_Stop(timer_76);
	Amo_timer_Stop(timer_77);
	Amo_timer_Stop(timer_78);
	Amo_timer_Stop(timer_79);
	Amo_timer_Stop(timer_80);
	Amo_timer_Stop(timer_81);
}


////////////////////////////////////////////////////////////////////////////////////////

void lin_FrPsCycleMode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	lin_FrDrCycleMode_TimerStop();
	lin_RrCycleMode_TimerStop();

	lin_FrPsSwingData_Scenario0();
	Set_LinToCan_Disp_FrPs_Mode(&Can_Tx_Evnt_4);
	Amo_timer_Stop(timer_36);
	Amo_timer_Start(timer_36, 10, true, Set_LinToCan_Disp_FrPsCycle_XY);
}

void lin_FrPsSwingData_Scenario0()
{
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = 0;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = 0;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = 0;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = 0;
	
	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD);

	lin_FrPsCycleMode_TimerStop();
	Amo_timer_Start(timer_23, 1000, false, OpDone_Check11);  //1199
}

void OpDone_Check11(void)
{

	SRLR_OP_check = l_bool_rd_LI1_FR_SRLR_OpDone();
	SRUD_OP_check = l_bool_rd_LI1_FR_SRUD_OpDone();

	CRLR_OP_check = l_bool_rd_LI1_FR_CRLR_OpDone();
	CRUD_OP_check = l_bool_rd_LI1_FR_CRUD_OpDone();

	Amo_timer_Stop(timer_47);
	Amo_timer_Start(timer_47, 50, false, OpDone_Check11);

	if(SRLR_OP_check == 1 && SRUD_OP_check == 1 && CRLR_OP_check == 1 && CRUD_OP_check == 1)
	{
		lin_FrPsSwingData_Scenario1();
	}

}


void lin_FrPsSwingData_Scenario1()
{
	lin_FrPsCycleMode_TimerStop();
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_3);

	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = 0;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = 0;
	
	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD);

	lin_FrPsCycleMode_TimerStop();
	Amo_timer_Start(timer_24, 1000, false, OpDone_Check12);

}

void OpDone_Check12(void)
{

	SRLR_OP_check = l_bool_rd_LI1_FR_SRLR_OpDone();
	SRUD_OP_check = l_bool_rd_LI1_FR_SRUD_OpDone();

	CRLR_OP_check = l_bool_rd_LI1_FR_CRLR_OpDone();
	CRUD_OP_check = l_bool_rd_LI1_FR_CRUD_OpDone();

	Amo_timer_Stop(timer_48);
	Amo_timer_Start(timer_48, 50, false, OpDone_Check12);

	if(SRLR_OP_check == 1 && SRUD_OP_check == 1 && CRLR_OP_check == 1 && CRUD_OP_check == 1)
	{
		lin_FrPsSwingData_Scenario2();
	}

}


void lin_FrPsSwingData_Scenario2()
{
	lin_FrPsCycleMode_TimerStop();

	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Max_UD; //250; //260 26degree;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Min_UD; //250; // 260;

	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD);

	lin_FrPsCycleMode_TimerStop();
	Amo_timer_Start(timer_25, 1000, false, OpDone_Check13);
}


void OpDone_Check13(void)
{

	SRLR_OP_check = l_bool_rd_LI1_FR_SRLR_OpDone();
	SRUD_OP_check = l_bool_rd_LI1_FR_SRUD_OpDone();

	CRLR_OP_check = l_bool_rd_LI1_FR_CRLR_OpDone();
	CRUD_OP_check = l_bool_rd_LI1_FR_CRUD_OpDone();

	Amo_timer_Stop(timer_49);
	Amo_timer_Start(timer_49, 50, false, OpDone_Check13);

	if(SRLR_OP_check == 1 && SRUD_OP_check == 1 && CRLR_OP_check == 1 && CRUD_OP_check == 1)
	{
		lin_FrPsSwingData_Scenario3();
	}

}


void lin_FrPsSwingData_Scenario3()
{
	lin_FrPsCycleMode_TimerStop();

	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Max_UD; //250; //260;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Min_UD; //250; //260;

	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD);

	lin_FrPsCycleMode_TimerStop();
	Amo_timer_Start(timer_26, 1000, false, OpDone_Check14);
}

void OpDone_Check14(void)
{

	SRLR_OP_check = l_bool_rd_LI1_FR_SRLR_OpDone();
	SRUD_OP_check = l_bool_rd_LI1_FR_SRUD_OpDone();

	CRLR_OP_check = l_bool_rd_LI1_FR_CRLR_OpDone();
	CRUD_OP_check = l_bool_rd_LI1_FR_CRUD_OpDone();

	Amo_timer_Stop(timer_50);
	Amo_timer_Start(timer_50, 50, false, OpDone_Check14);

	if(SRLR_OP_check == 1 && SRUD_OP_check == 1 && CRLR_OP_check == 1 && CRUD_OP_check == 1)
	{
		lin_FrPsSwingData_Scenario4();
	}

}


void lin_FrPsSwingData_Scenario4()
{
	lin_FrPsCycleMode_TimerStop();

	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);

	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD);

	lin_FrPsCycleMode_TimerStop();
	Amo_timer_Start(timer_27, 1000, false, OpDone_Check15);
}

void OpDone_Check15(void)
{
	SRLR_OP_check = l_bool_rd_LI1_FR_SRLR_OpDone();
	SRUD_OP_check = l_bool_rd_LI1_FR_SRUD_OpDone();

	CRLR_OP_check = l_bool_rd_LI1_FR_CRLR_OpDone();
	CRUD_OP_check = l_bool_rd_LI1_FR_CRUD_OpDone();

	Amo_timer_Stop(timer_51);
	Amo_timer_Start(timer_51, 50, false, OpDone_Check15);

	if(SRLR_OP_check == 1 && SRUD_OP_check == 1 && CRLR_OP_check == 1 && CRUD_OP_check == 1)
	{
		lin_FrPsSwingData_Scenario5();
	}

}


void lin_FrPsSwingData_Scenario5()
{
	lin_FrPsCycleMode_TimerStop();

	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);

	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD);

	lin_FrPsCycleMode_TimerStop();
	Amo_timer_Start(timer_28, 1000, false, OpDone_Check16);
}

void OpDone_Check16(void)
{

	SRLR_OP_check = l_bool_rd_LI1_FR_SRLR_OpDone();
	SRUD_OP_check = l_bool_rd_LI1_FR_SRUD_OpDone();

	CRLR_OP_check = l_bool_rd_LI1_FR_CRLR_OpDone();
	CRUD_OP_check = l_bool_rd_LI1_FR_CRUD_OpDone();

	Amo_timer_Stop(timer_52);
	Amo_timer_Start(timer_52, 50, false, OpDone_Check16);

	if(SRLR_OP_check == 1 && SRUD_OP_check == 1 && CRLR_OP_check == 1 && CRUD_OP_check == 1)
	{
		lin_FrPsSwingData_Scenario2();
	}

}


void lin_FrPsCycleMode_TimerStop(void)
{
	Amo_timer_Stop(timer_23);
	Amo_timer_Stop(timer_24);
	Amo_timer_Stop(timer_25);
	Amo_timer_Stop(timer_26);
	Amo_timer_Stop(timer_27);
	Amo_timer_Stop(timer_28);
	Amo_timer_Stop(timer_36);
	Amo_timer_Stop(timer_38);
	Amo_timer_Stop(timer_47);
	Amo_timer_Stop(timer_48);
	Amo_timer_Stop(timer_49);
	Amo_timer_Stop(timer_50);
	Amo_timer_Stop(timer_51);
	Amo_timer_Stop(timer_52);
	Amo_timer_Stop(timer_70);
	Amo_timer_Stop(timer_71);
	Amo_timer_Stop(timer_72);
	Amo_timer_Stop(timer_73);
	Amo_timer_Stop(timer_74);
	Amo_timer_Stop(timer_75);
	Amo_timer_Stop(timer_76);
	Amo_timer_Stop(timer_77);
	Amo_timer_Stop(timer_78);
	Amo_timer_Stop(timer_79);
	Amo_timer_Stop(timer_80);
	Amo_timer_Stop(timer_81);
}

////////////////////////////////////////////////////////////////////

void lin_RrCycleMode_task(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl)
{
	lin_FrDrCycleMode_TimerStop();
	lin_FrPsCycleMode_TimerStop();

	lin_RrSwingData_Scenario0();
	Set_LinToCan_Disp_RrDr_Mode(&Can_Tx_Evnt_4);
	Set_LinToCan_Disp_RrPs_Mode(&Can_Tx_Evnt_4);
	Amo_timer_Stop(timer_37);
	Amo_timer_Start(timer_37, 10, true, Set_LinToCan_Disp_RrCycle_XY);
}

void lin_RrSwingData_Scenario0()
{
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = 0;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = 0;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = 0;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = 0;

	lin_Write_RrEVntConsPt_XY(&LIN_RC_EVNT_MASTER_CMD);
	
	lin_RrCycleMode_TimerStop();
	Amo_timer_Start(timer_29, 1199, false, lin_RrSwingData_Scenario1);  //1199
}

void lin_RrSwingData_Scenario1()
{
	SetLIN_EVNT_RrSPEED(&LIN_RC_EVNT_MASTER_CMD, ACT_RPM_3);

	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = Min_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = 0;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = 0;
	
	lin_Write_RrEVntConsPt_XY(&LIN_RC_EVNT_MASTER_CMD);

	lin_RrCycleMode_TimerStop();
	Amo_timer_Start(timer_30, 1701, false, lin_RrSwingData_Scenario2);

}

void lin_RrSwingData_Scenario2()
{
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = Min_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = Min_UD; //250; //260 26degree;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = Min_UD; //250; // 260;

	lin_Write_RrEVntConsPt_XY(&LIN_RC_EVNT_MASTER_CMD);
	
	lin_RrCycleMode_TimerStop();
	Amo_timer_Start(timer_31, 2502, false, lin_RrSwingData_Scenario3);
}

void lin_RrSwingData_Scenario3()
{
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = Max_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = Min_UD; //250; //260;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = Min_UD; //250; //260;

	lin_Write_RrEVntConsPt_XY(&LIN_RC_EVNT_MASTER_CMD);

	lin_RrCycleMode_TimerStop();
	Amo_timer_Start(timer_32, 1701, false, lin_RrSwingData_Scenario4);
}

void lin_RrSwingData_Scenario4()
{
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = Max_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);

	lin_Write_RrEVntConsPt_XY(&LIN_RC_EVNT_MASTER_CMD);

	lin_RrCycleMode_TimerStop();
	Amo_timer_Start(timer_33, 2502, false, lin_RrSwingData_Scenario5);
}

void lin_RrSwingData_Scenario5()
{
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = Min_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
 
	lin_Write_RrEVntConsPt_XY(&LIN_RC_EVNT_MASTER_CMD);

	lin_RrCycleMode_TimerStop();
	Amo_timer_Start(timer_34, 1701, false, lin_RrSwingData_Scenario2);
}

void lin_RrCycleMode_TimerStop(void)
{
	Amo_timer_Stop(timer_29);
	Amo_timer_Stop(timer_30);
	Amo_timer_Stop(timer_31);
	Amo_timer_Stop(timer_32);
	Amo_timer_Stop(timer_33);
	Amo_timer_Stop(timer_34);
	Amo_timer_Stop(timer_37);
}



////////////////////////////////////////////////////////////////////

void Set_LinToCan_Disp_FrDrCycle_XY(void)
{

	uint64_t FR_SLLR_ActuatorState = l_u16_rd_LI0_FR_SLLR_ActuatorState();
	uint64_t FR_SLUD_ActuatorState = l_u16_rd_LI0_FR_SLUD_ActuatorState();
	uint64_t FR_CLLR_ActuatorState = l_u16_rd_LI0_FR_CLLR_ActuatorState();
	uint64_t FR_CLUD_ActuatorState = l_u16_rd_LI0_FR_CLUD_ActuatorState();

	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_X = SET_L_EVNT_HU_CycleFrDrEVntSidePt_X(FR_SLLR_ActuatorState, &Can_data_23_Info);
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = SET_L_EVNT_HU_CycleFrDrEVntSidePt_Y(FR_SLUD_ActuatorState, &Can_data_23_Info);
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = SET_L_EVNT_HU_CycleFrDrEVntCtrPt_X(FR_CLLR_ActuatorState, &Can_data_23_Info);
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = SET_L_EVNT_HU_CycleFrDrEVntCtrPt_Y(FR_CLUD_ActuatorState, &Can_data_23_Info);

}

uint16_t SET_L_EVNT_HU_CycleFrDrEVntSidePt_X(uint64_t lin_Ctrl, Atcu_23_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_CycleFrDrEVntSidePt_Y(uint64_t lin_Ctrl, Atcu_23_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_CycleFrDrEVntCtrPt_X(uint64_t lin_Ctrl, Atcu_23_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_CycleFrDrEVntCtrPt_Y(uint64_t lin_Ctrl, Atcu_23_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

void Set_LinToCan_Disp_FrPsCycle_XY(void)
{

	uint64_t FR_SRLR_ActuatorState = l_u16_rd_LI1_FR_SRLR_ActuatorState();
	uint64_t FR_SRUD_ActuatorState = l_u16_rd_LI1_FR_SRUD_ActuatorState();
	uint64_t FR_CRLR_ActuatorState = l_u16_rd_LI1_FR_CRLR_ActuatorState();
	uint64_t FR_CRUD_ActuatorState = l_u16_rd_LI1_FR_CRUD_ActuatorState();

	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_X = SET_L_EVNT_HU_CycleFrPsEVntSidePt_X(FR_SRLR_ActuatorState, &Can_data_24_Info);
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = SET_L_EVNT_HU_CycleFrPsEVntSidePt_Y(FR_SRUD_ActuatorState, &Can_data_24_Info);
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = SET_L_EVNT_HU_CycleFrPsEVntCtrPt_X(FR_CRLR_ActuatorState, &Can_data_24_Info);
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = SET_L_EVNT_HU_CycleFrPsEVntCtrPt_Y(FR_CRUD_ActuatorState, &Can_data_24_Info);

}

uint16_t SET_L_EVNT_HU_CycleFrPsEVntSidePt_X(uint64_t lin_Ctrl, Atcu_24_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_CycleFrPsEVntSidePt_Y(uint64_t lin_Ctrl, Atcu_24_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_CycleFrPsEVntCtrPt_X(uint64_t lin_Ctrl, Atcu_24_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_CycleFrPsEVntCtrPt_Y(uint64_t lin_Ctrl, Atcu_24_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

void Set_LinToCan_Disp_RrCycle_XY(void)
{

	uint64_t RR_CLLR_ActuatorState = l_u16_rd_LI2_RR_CLLR_ActuatorState();
	uint64_t RR_CLUD_ActuatorState = l_u16_rd_LI2_RR_CLUD_ActuatorState();
	uint64_t RR_CRLR_ActuatorState = l_u16_rd_LI2_RR_CRLR_ActuatorState();
	uint64_t RR_CRUD_ActuatorState = l_u16_rd_LI2_RR_CRUD_ActuatorState();

	Can_Tx_Evnt_3.data.L_EVNT_HU_RrDrEvntConsPtDisp_X = SET_L_EVNT_HU_CycleRrDrEVntSidePt_X(RR_CLLR_ActuatorState, &Can_data_25_Info);
	Can_Tx_Evnt_3.data.L_EVNT_HU_RrDrEvntConsPtDisp_Y = SET_L_EVNT_HU_CycleRrDrEVntSidePt_Y(RR_CLUD_ActuatorState, &Can_data_25_Info);
	Can_Tx_Evnt_3.data.L_EVNT_HU_RrPsEvntConsPtDisp_X = SET_L_EVNT_HU_CycleRrPsEVntCtrPt_X(RR_CRLR_ActuatorState, &Can_data_25_Info);
	Can_Tx_Evnt_3.data.L_EVNT_HU_RrPsEvntConsPtDisp_Y = SET_L_EVNT_HU_CycleRrPsEVntCtrPt_Y(RR_CRUD_ActuatorState, &Can_data_25_Info);

}

uint16_t SET_L_EVNT_HU_CycleRrDrEVntSidePt_X(uint64_t lin_Ctrl, Atcu_25_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryUp_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_CycleRrDrEVntSidePt_Y(uint64_t lin_Ctrl, Atcu_25_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryUp_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_CycleRrPsEVntCtrPt_X(uint64_t lin_Ctrl, Atcu_25_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryUP_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_CycleRrPsEVntCtrPt_Y(uint64_t lin_Ctrl, Atcu_25_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryUP_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

























#if 0 

void lin_FrDrPsCycleMode_task(void)
{
	lin_FrDrCycleMode_TimerStop();
	lin_FrPsCycleMode_TimerStop();

	lin_FrDrSwingData_Scenario0();
	lin_FrPsSwingData_Scenario0();
	Set_LinToCan_Disp_FrDr_Mode(&Can_Tx_Evnt_4);
	Set_LinToCan_Disp_FrPs_Mode(&Can_Tx_Evnt_4);
	Amo_timer_Stop(timer_38);
	Amo_timer_Start(timer_38, 10, true, Set_LinToCan_Disp_FrDrCycle_XY);
	Amo_timer_Stop(timer_39);
	Amo_timer_Start(timer_39, 10, true, Set_LinToCan_Disp_FrPsCycle_XY);
}

void lin_FrDrCycleMode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	lin_FrPsCycleMode_TimerStop();
	lin_RrCycleMode_TimerStop();

	lin_FrDrSwingData_Scenario0();
	Set_LinToCan_Disp_FrDr_Mode(&Can_Tx_Evnt_4);
	Amo_timer_Stop(timer_35);
	Amo_timer_Start(timer_35, 10, true, Set_LinToCan_Disp_FrDrCycle_XY);
}

void lin_FrDrSwingData_Scenario0()
{
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 0;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 0;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = 0;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 0;

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD);

	lin_FrDrCycleMode_TimerStop();
	Amo_timer_Start(timer_16, 1199, false, lin_FrDrSwingData_Scenario1);  //1199
}

void lin_FrDrSwingData_Scenario1()
{
//	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_SPEED = 0x3;
//	l_u8_wr_LI0_EVNT_SPEED(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_SPEED);

	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_3);

	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Min_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 0;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Max_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 0;

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD);

	lin_FrDrCycleMode_TimerStop();
	Amo_timer_Start(timer_17, 1701, false, lin_FrDrSwingData_Scenario2);

}

void lin_FrDrSwingData_Scenario2()
{
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Min_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Min_UD; //250; //260 26degree;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Max_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Max_UD; //250; // 260;

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD);

	lin_FrDrCycleMode_TimerStop();
	Amo_timer_Start(timer_18, 2502, false, lin_FrDrSwingData_Scenario3);
}

void lin_FrDrSwingData_Scenario3()
{
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Max_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Min_UD; //250; //260;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Min_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Max_UD; //250; //260;

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD);

	lin_FrDrCycleMode_TimerStop();
	Amo_timer_Start(timer_19, 1701, false, lin_FrDrSwingData_Scenario4);
}

void lin_FrDrSwingData_Scenario4()
{
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Max_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Min_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD);

	lin_FrDrCycleMode_TimerStop();
	Amo_timer_Start(timer_20, 2502, false, lin_FrDrSwingData_Scenario5);
}

void lin_FrDrSwingData_Scenario5()
{
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Min_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Max_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD);

	lin_FrDrCycleMode_TimerStop();
	Amo_timer_Start(timer_21, 1701, false, lin_FrDrSwingData_Scenario2);
}


void lin_FrDrCycleMode_TimerStop(void)
{
	Amo_timer_Stop(timer_16);
	Amo_timer_Stop(timer_17);
	Amo_timer_Stop(timer_18);
	Amo_timer_Stop(timer_19);
	Amo_timer_Stop(timer_20);
	Amo_timer_Stop(timer_21);
	Amo_timer_Stop(timer_35);
}


void lin_FrPsCycleMode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	lin_FrDrCycleMode_TimerStop();
	lin_RrCycleMode_TimerStop();

	lin_FrPsSwingData_Scenario0();
	Set_LinToCan_Disp_FrPs_Mode(&Can_Tx_Evnt_4);
	Amo_timer_Stop(timer_36);
	Amo_timer_Start(timer_36, 10, true, Set_LinToCan_Disp_FrPsCycle_XY);
}

void lin_FrPsSwingData_Scenario0()
{
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = 0;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = 0;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = 0;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = 0;
	
	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD);

	lin_FrPsCycleMode_TimerStop();
	Amo_timer_Start(timer_23, 1199, false, lin_FrPsSwingData_Scenario1);  //1199
}

void lin_FrPsSwingData_Scenario1()
{
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_3);

	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = 0;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = 0;
	
	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD);

	lin_FrPsCycleMode_TimerStop();
	Amo_timer_Start(timer_24, 1701, false, lin_FrPsSwingData_Scenario2);

}

void lin_FrPsSwingData_Scenario2()
{
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Max_UD; //250; //260 26degree;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Min_UD; //250; // 260;

	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD);

	lin_FrPsCycleMode_TimerStop();
	Amo_timer_Start(timer_25, 2502, false, lin_FrPsSwingData_Scenario3);
}

void lin_FrPsSwingData_Scenario3()
{
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Max_UD; //250; //260;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Min_UD; //250; //260;

	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD);

	lin_FrPsCycleMode_TimerStop();
	Amo_timer_Start(timer_26, 1701, false, lin_FrPsSwingData_Scenario4);
}

void lin_FrPsSwingData_Scenario4()
{
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);

	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD);

	lin_FrPsCycleMode_TimerStop();
	Amo_timer_Start(timer_27, 2502, false, lin_FrPsSwingData_Scenario5);
}

void lin_FrPsSwingData_Scenario5()
{
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);

	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD);

	lin_FrPsCycleMode_TimerStop();
	Amo_timer_Start(timer_28, 1701, false, lin_FrPsSwingData_Scenario2);
}

void lin_FrPsCycleMode_TimerStop(void)
{
	Amo_timer_Stop(timer_23);
	Amo_timer_Stop(timer_24);
	Amo_timer_Stop(timer_25);
	Amo_timer_Stop(timer_26);
	Amo_timer_Stop(timer_27);
	Amo_timer_Stop(timer_28);
	Amo_timer_Stop(timer_36);
}


void lin_RrCycleMode_task(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl)
{
	lin_FrDrCycleMode_TimerStop();
	lin_FrPsCycleMode_TimerStop();

	lin_RrSwingData_Scenario0();
	Set_LinToCan_Disp_RrDr_Mode(&Can_Tx_Evnt_4);
	Set_LinToCan_Disp_RrPs_Mode(&Can_Tx_Evnt_4);
	Amo_timer_Stop(timer_37);
	Amo_timer_Start(timer_37, 10, true, Set_LinToCan_Disp_RrCycle_XY);
}

void lin_RrSwingData_Scenario0()
{
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = 0;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = 0;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = 0;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = 0;

	lin_Write_RrEVntConsPt_XY(&LIN_RC_EVNT_MASTER_CMD);
	
	lin_RrCycleMode_TimerStop();
	Amo_timer_Start(timer_29, 1199, false, lin_RrSwingData_Scenario1);  //1199
}

void lin_RrSwingData_Scenario1()
{
	SetLIN_EVNT_RrSPEED(&LIN_RC_EVNT_MASTER_CMD, ACT_RPM_3);

	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = Min_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = 0;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = 0;
	
	lin_Write_RrEVntConsPt_XY(&LIN_RC_EVNT_MASTER_CMD);

	lin_RrCycleMode_TimerStop();
	Amo_timer_Start(timer_30, 1701, false, lin_RrSwingData_Scenario2);

}

void lin_RrSwingData_Scenario2()
{
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = Min_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = Min_UD; //250; //260 26degree;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = Min_UD; //250; // 260;

	lin_Write_RrEVntConsPt_XY(&LIN_RC_EVNT_MASTER_CMD);
	
	lin_RrCycleMode_TimerStop();
	Amo_timer_Start(timer_31, 2502, false, lin_RrSwingData_Scenario3);
}

void lin_RrSwingData_Scenario3()
{
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = Max_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = Min_UD; //250; //260;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = Min_UD; //250; //260;

	lin_Write_RrEVntConsPt_XY(&LIN_RC_EVNT_MASTER_CMD);

	lin_RrCycleMode_TimerStop();
	Amo_timer_Start(timer_32, 1701, false, lin_RrSwingData_Scenario4);
}

void lin_RrSwingData_Scenario4()
{
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = Max_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = Min_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);

	lin_Write_RrEVntConsPt_XY(&LIN_RC_EVNT_MASTER_CMD);

	lin_RrCycleMode_TimerStop();
	Amo_timer_Start(timer_33, 2502, false, lin_RrSwingData_Scenario5);
}

void lin_RrSwingData_Scenario5()
{
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = Min_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = Max_LR;
	LIN_RC_EVNT_MASTER_CMD.LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = Max_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
 
	lin_Write_RrEVntConsPt_XY(&LIN_RC_EVNT_MASTER_CMD);

	lin_RrCycleMode_TimerStop();
	Amo_timer_Start(timer_34, 1701, false, lin_RrSwingData_Scenario2);
}

void lin_RrCycleMode_TimerStop(void)
{
	Amo_timer_Stop(timer_29);
	Amo_timer_Stop(timer_30);
	Amo_timer_Stop(timer_31);
	Amo_timer_Stop(timer_32);
	Amo_timer_Stop(timer_33);
	Amo_timer_Stop(timer_34);
	Amo_timer_Stop(timer_37);
}




void Set_LinToCan_Disp_FrDrCycle_XY(void)
{

	uint64_t FR_SLLR_ActuatorState = l_u16_rd_LI0_FR_SLLR_ActuatorState();
	uint64_t FR_SLUD_ActuatorState = l_u16_rd_LI0_FR_SLUD_ActuatorState();
	uint64_t FR_CLLR_ActuatorState = l_u16_rd_LI0_FR_CLLR_ActuatorState();
	uint64_t FR_CLUD_ActuatorState = l_u16_rd_LI0_FR_CLUD_ActuatorState();

	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_X = SET_L_EVNT_HU_CycleFrDrEVntSidePt_X(FR_SLLR_ActuatorState, &Can_data_23_Info);
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = SET_L_EVNT_HU_CycleFrDrEVntSidePt_Y(FR_SLUD_ActuatorState, &Can_data_23_Info);
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = SET_L_EVNT_HU_CycleFrDrEVntCtrPt_X(FR_CLLR_ActuatorState, &Can_data_23_Info);
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = SET_L_EVNT_HU_CycleFrDrEVntCtrPt_Y(FR_CLUD_ActuatorState, &Can_data_23_Info);

}

uint16_t SET_L_EVNT_HU_CycleFrDrEVntSidePt_X(uint64_t lin_Ctrl, Atcu_23_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_CycleFrDrEVntSidePt_Y(uint64_t lin_Ctrl, Atcu_23_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_CycleFrDrEVntCtrPt_X(uint64_t lin_Ctrl, Atcu_23_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_CycleFrDrEVntCtrPt_Y(uint64_t lin_Ctrl, Atcu_23_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

void Set_LinToCan_Disp_FrPsCycle_XY(void)
{

	uint64_t FR_SRLR_ActuatorState = l_u16_rd_LI1_FR_SRLR_ActuatorState();
	uint64_t FR_SRUD_ActuatorState = l_u16_rd_LI1_FR_SRUD_ActuatorState();
	uint64_t FR_CRLR_ActuatorState = l_u16_rd_LI1_FR_CRLR_ActuatorState();
	uint64_t FR_CRUD_ActuatorState = l_u16_rd_LI1_FR_CRUD_ActuatorState();

	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_X = SET_L_EVNT_HU_CycleFrPsEVntSidePt_X(FR_SRLR_ActuatorState, &Can_data_24_Info);
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = SET_L_EVNT_HU_CycleFrPsEVntSidePt_Y(FR_SRUD_ActuatorState, &Can_data_24_Info);
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = SET_L_EVNT_HU_CycleFrPsEVntCtrPt_X(FR_CRLR_ActuatorState, &Can_data_24_Info);
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = SET_L_EVNT_HU_CycleFrPsEVntCtrPt_Y(FR_CRUD_ActuatorState, &Can_data_24_Info);

}

uint16_t SET_L_EVNT_HU_CycleFrPsEVntSidePt_X(uint64_t lin_Ctrl, Atcu_24_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_CycleFrPsEVntSidePt_Y(uint64_t lin_Ctrl, Atcu_24_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_CycleFrPsEVntCtrPt_X(uint64_t lin_Ctrl, Atcu_24_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_CycleFrPsEVntCtrPt_Y(uint64_t lin_Ctrl, Atcu_24_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

void Set_LinToCan_Disp_RrCycle_XY(void)
{

	uint64_t RR_CLLR_ActuatorState = l_u16_rd_LI2_RR_CLLR_ActuatorState();
	uint64_t RR_CLUD_ActuatorState = l_u16_rd_LI2_RR_CLUD_ActuatorState();
	uint64_t RR_CRLR_ActuatorState = l_u16_rd_LI2_RR_CRLR_ActuatorState();
	uint64_t RR_CRUD_ActuatorState = l_u16_rd_LI2_RR_CRUD_ActuatorState();

	Can_Tx_Evnt_3.data.L_EVNT_HU_RrDrEvntConsPtDisp_X = SET_L_EVNT_HU_CycleRrDrEVntSidePt_X(RR_CLLR_ActuatorState, &Can_data_25_Info);
	Can_Tx_Evnt_3.data.L_EVNT_HU_RrDrEvntConsPtDisp_Y = SET_L_EVNT_HU_CycleRrDrEVntSidePt_Y(RR_CLUD_ActuatorState, &Can_data_25_Info);
	Can_Tx_Evnt_3.data.L_EVNT_HU_RrPsEvntConsPtDisp_X = SET_L_EVNT_HU_CycleRrPsEVntCtrPt_X(RR_CRLR_ActuatorState, &Can_data_25_Info);
	Can_Tx_Evnt_3.data.L_EVNT_HU_RrPsEvntConsPtDisp_Y = SET_L_EVNT_HU_CycleRrPsEVntCtrPt_Y(RR_CRUD_ActuatorState, &Can_data_25_Info);

}

uint16_t SET_L_EVNT_HU_CycleRrDrEVntSidePt_X(uint64_t lin_Ctrl, Atcu_25_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryUp_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_CycleRrDrEVntSidePt_Y(uint64_t lin_Ctrl, Atcu_25_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryUp_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_CycleRrPsEVntCtrPt_X(uint64_t lin_Ctrl, Atcu_25_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryUP_X;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_CycleRrPsEVntCtrPt_Y(uint64_t lin_Ctrl, Atcu_25_data *canBdryDataBuf)
{
	int16_t result;

	uint64_t TP_temp = lin_Ctrl;
	uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryUP_Y;
	uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}
#endif  /* if 0 */

#endif /* __UART_AMO_H */

